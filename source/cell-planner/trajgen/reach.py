"""Step 1: reachability + weld IK per (weld, robot).

For every weld and robot, ``expert.ik.WeldIK.solve_axis`` (5-DOF: position + tool axis, the spin about the tool
axis is free as in umarell's gate, the axis may tilt up to ``max_tilt_deg``) is run from HOME + structured +
random seeds. Each branch must keep a straight Cartesian line from the weld back to the standoff
``p - a * approach_offset`` (at least ``min_offset``) valid under every single-arm scene rule
(``SceneModel.extra_check``: corrected floor spheres, box rule incl. tool, rule W; WeldIK.check: joint limits, hull
self-distance >= 5 mm, hull floor, umarell floor). That line is the descent, and reversed, the lift-off.

Preference ("margins", docs/hierarchical.md H v2 constraint): tilt <= 3.5 deg and hull self-distance >= 15 mm
along the whole stroke. Options below that are kept only as ``thin`` fallbacks (penalised in the allocation).
Results are cached in ``data/trajgen/cache`` keyed by a hash of the scene, task and options.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from trajgen.cachedir import CACHE_DIR
from trajgen.model import SceneModel

ROOT = Path(__file__).resolve().parents[1]
CACHE = CACHE_DIR


@dataclass
class WeldOption:
    wid: str
    rid: str
    q: np.ndarray               # (6,) weld configuration
    descent: np.ndarray         # (m, 6) pre-weld -> weld joint knots (straight Cartesian line, fixed orientation)
    liftoff: np.ndarray         # (k, 6) weld -> retract knots (straight, on the same line)
    standoff_m: float
    retract_m: float
    spin_deg: float
    tilt_deg: float
    tilt_dir_deg: float
    min_self_dist: float        # min hull self-distance along the stroke (m)
    thin: bool
    pos_err_m: float
    axis_err_deg: float
    rot_err_deg: float

    @property
    def q_pre(self):
        return self.descent[0]

    @property
    def q_ret(self):
        return self.liftoff[-1]

    def to_json(self):
        d = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in self.__dict__.items()}
        return d

    @classmethod
    def from_json(cls, d):
        d = dict(d)
        for k in ("q", "descent", "liftoff"):
            d[k] = np.asarray(d[k], float)
        return cls(**d)


def _hash(scene, task, opts) -> str:
    h = hashlib.sha256()
    h.update(Path(scene.path).read_bytes() if scene.path else b"")
    h.update(Path(task.path).read_bytes() if task.path else b"")
    for r in scene.robots:                      # the robot models themselves (URDF content)
        h.update(Path(r.urdf).read_bytes() if Path(r.urdf).exists() else b"")
    # P3U round 2: the SRDF (disabled self-collision pairs of the IK validity) and the collision meshes the URDF
    # references; an in-place edit of either re-solves instead of reusing stale reach options
    for r in scene.robots:
        srdf = getattr(r, "srdf", None)
        if srdf is not None and Path(srdf).exists():
            h.update(b"srdf:" + Path(srdf).read_bytes())
        for f in _collision_meshes(r.urdf):
            h.update(f"mesh:{Path(f).name}:".encode() + (Path(f).read_bytes() if Path(f).exists() else b"missing"))
    h.update(json.dumps(opts, sort_keys=True).encode())
    return h.hexdigest()[:16]


def _collision_meshes(urdf) -> list:
    try:
        from robotmodel.urdf_robot import parse_urdf
        u = parse_urdf(str(urdf))
        return sorted({g.filename for l in u.links.values() for g in l.collisions if g.kind == "mesh" and g.filename})
    except Exception:  # noqa: BLE001  (an unreadable URDF fails loudly later)
        return []


def cache_dump(out: dict, reasons: dict) -> dict:
    """Reach cache, format 2: a list of {weld, robot, options, reason} records (ids round-trip exactly)."""
    return {"__format__": 2, "records": [{"weld": k[0], "robot": k[1], "options": [o.to_json() for o in v],
                                          "reason": reasons.get(f"{k[0]}|{k[1]}")} for k, v in out.items()]}


def cache_load(raw: dict):
    """-> ({(wid, rid): [WeldOption]}, reasons). Also reads format 1 ('wid|rid' keys, ids without '|')."""
    if raw.get("__format__") == 2:
        out = {(d["weld"], d["robot"]): [WeldOption.from_json(o) for o in d["options"]] for d in raw["records"]}
        reasons = {f"{d['weld']}|{d['robot']}": d["reason"] for d in raw["records"] if d.get("reason") is not None}
        return out, reasons
    out = {tuple(k.split("|")): [WeldOption.from_json(o) for o in v] for k, v in raw.items() if "|" in k}
    return out, raw.get("__reasons__", {})


def allowed(w, rid) -> bool:
    return w.allowed_robots is None or rid in w.allowed_robots


def cache_key(model: SceneModel, *, n_random: int = 120, max_tilt_deg: float = 3.5, margin_m: float = 0.015,
              n_steps: int = 40, max_options: int = 6, seed: int = 0) -> tuple[str, dict]:
    """(key, opts) of ``solve_reach``'s cache file for these options (the same defaults)."""
    opts = {"n_random": n_random, "max_tilt_deg": max_tilt_deg, "margin_m": margin_m, "n_steps": n_steps,
            "max_options": max_options, "self_margin": model.self_margin, "v": 3}
    if seed:
        opts["seed"] = int(seed)          # (seed 0 keeps the original cache key)
    if getattr(model, "electrode_on", False):   # electrode off-weld rule (round 2): also when set programmatically
        er = model.electrode_rule
        opts["electrode_off_weld"] = [float(er.radius), float(er.min_distance)]
        if getattr(er, "nodip", False):           # round 3 no-dip rule (trajgen.nodip)
            opts["nodip"] = [int(er.version), float(er.contact_depth), float(er.contact_point_radius)]
    return _hash(model.scene, model.task, opts), opts


def cache_path(cell_id: str, key: str) -> Path:
    """data/trajgen/cache/reach_<cell_id>_<key>.json (where ``solve_reach`` reads / writes its cache)."""
    return CACHE / f"reach_{cell_id}_{key}.json"


def solve_reach(model: SceneModel, *, n_random: int = 120, max_tilt_deg: float = 3.5, margin_m: float = 0.015,
                n_steps: int = 40, max_options: int = 6, cache: bool = True, seed: int = 0, log=print,
                workers: int = 1) -> dict:
    """{(wid, rid): [WeldOption, ...]} (best first; empty list = unreachable). ``workers`` > 1: the (weld, robot)
    pairs are solved in forked worker processes (trajgen.parallel); the random IK seeds are drawn in the parent in
    the serial order first, so the result is that of the serial run."""
    scene = model.scene
    key, opts = cache_key(model, n_random=n_random, max_tilt_deg=max_tilt_deg, margin_m=margin_m, n_steps=n_steps,
                          max_options=max_options, seed=seed)
    cf = cache_path(scene.cell_id, key)
    if cache and cf.exists():
        raw = json.load(open(cf))
        log(f"[reach] cache {cf.relative_to(ROOT) if cf.is_relative_to(ROOT) else cf}")
        out, model.reach_reasons = cache_load(raw)
        return out
    out, reasons, jobs, ctx = prepare_reach(model, n_random=n_random, max_tilt_deg=max_tilt_deg, margin_m=margin_m,
                                            n_steps=n_steps, max_options=max_options, log=log)
    from trajgen.parallel import pmap
    res_all = pmap(_reach_pair, ctx, jobs, workers, log, "reach")
    out = finish_reach(model, out, reasons, jobs, res_all, log=log)
    if cache:
        CACHE.mkdir(parents=True, exist_ok=True)
        import os
        tmp = cf.with_name(f"{cf.name}.{os.getpid()}.tmp")     # atomic: concurrent runs never read a partial file
        with open(tmp, "w") as f:
            json.dump(cache_dump(out, reasons), f)
        os.replace(tmp, cf)
    return out


def prepare_reach(model: SceneModel, *, n_random: int = 120, max_tilt_deg: float = 3.5, margin_m: float = 0.015,
                  n_steps: int = 40, max_options: int = 6, log=print):
    """The first half of ``solve_reach`` (no cache): -> (out, reasons, jobs, ctx). ``out`` holds the pairs settled
    without IK (not allowed, out of rough reach; None for the IK jobs), ``jobs`` the IK pairs (wid, rid, seeds) with
    their random seeds drawn from ``model.ik.rng`` in the serial order. ``_reach_pair(ctx, job)`` solves one job;
    its result depends only on the job, so the jobs may be solved in any order or subset (scenegen's early abort)."""
    scene, task = model.scene, model.task
    tilts = tuple(t for t in (1.0, 2.0, 3.0, 3.5, 4.0, 4.5) if t <= max_tilt_deg + 1e-9)
    ik = model.ik
    ik.margin_target = margin_m
    tol = scene.tolerances
    max_spin = None if tol.spin_free else float(tol.spin_tol_deg)
    out, reasons = {}, {}
    from trajgen.scene import rough_reach
    jobs = []                                    # (wid, rid, seeds): the IK pairs, seeds drawn in the serial order
    for w in task.welds:
        for rid in model.rids:
            if not allowed(w, rid):
                out[(w.id, rid)] = []
                reasons[f"{w.id}|{rid}"] = "not in the weld's allowed_robots"
                continue
            r = model.robots[rid]
            dist = float(np.linalg.norm(w.position - r.base_xyz))
            if dist > rough_reach(r) + 0.3:
                out[(w.id, rid)] = []
                reasons[f"{w.id}|{rid}"] = f"out of reach: {dist:.2f} m from the base (rough reach {rough_reach(r):.2f} m)"
                log(f"[reach] {w.id} {rid}: out of reach ({dist:.2f} m)")
                continue
            out[(w.id, rid)] = None
            seeds = [model.robots[rid].home.copy()] + ik.default_seeds(rid, n_random=n_random, target_world=w.position)
            jobs.append((w.id, rid, seeds))
    ctx = dict(model=model, n_steps=n_steps, tilts=tilts, max_spin=max_spin, margin_m=margin_m,
               max_options=max_options)
    return out, reasons, jobs, ctx


def finish_reach(model: SceneModel, out: dict, reasons: dict, jobs: list, res_all: list, log=print) -> dict:
    """The second half of ``solve_reach``: the job results (in ``jobs`` order) into ``out`` / ``reasons``;
    sets ``model.reach_reasons``."""
    for (wid, rid, _), (b, why) in zip(jobs, res_all):
        out[(wid, rid)] = b
        if why is not None:
            reasons[f"{wid}|{rid}"] = why
        log(f"[reach] {wid} {rid}: {len(b)} options" + (f" (best: spin {b[0].spin_deg:.0f} tilt {b[0].tilt_deg:.1f} "
            f"self {b[0].min_self_dist * 1000:.0f} mm standoff {b[0].standoff_m:.3f}{' THIN' if b[0].thin else ''})" if b else ""))
    out = {k: v for k, v in out.items()}
    model.reach_reasons = reasons
    return out


def _reach_pair(ctx, job):
    """IK options of one (weld, robot) pair (solve_reach), and the reason when there is none."""
    model = ctx["model"]
    wid, rid, seeds = job
    w = model.task.weld(wid)
    pl = model.scene.planning
    ik = model.ik
    ik.margin_target = ctx["margin_m"]
    Rint = ik.to_internal(w.R)
    n_steps, tilts, max_spin = ctx["n_steps"], ctx["tilts"], ctx["max_spin"]
    sols = ik.solve_axis(rid, p=w.position, R=Rint, seeds=seeds, approach=w.approach,
                         standoff=pl.approach_offset, min_standoff=pl.min_offset, n_steps=n_steps,
                         tilts=tilts, extra_check=model.extra_check(rid), max_spin=max_spin)
    res = []
    for c in sols:
        if c.path is None or len(c.path) < 2:
            continue
        desc = np.asarray(c.path, float)[::-1]
        lift = np.asarray(c.path, float)
        retract = c.standoff_m
        if abs(pl.retract_offset - pl.approach_offset) > 1e-9:
            lift, retract = _liftoff(model, rid, c, w, pl, n_steps)
            if lift is None:
                continue
        e = c.err
        res.append(WeldOption(w.id, rid, np.asarray(c.q, float), desc, lift, float(c.standoff_m), float(retract),
                              float(c.spin_deg), float(c.tilt_deg), float(c.tilt_dir_deg), float(c.path_min_sd),
                              bool(c.path_min_sd < ctx["margin_m"]), float(e.get("pos_m", np.nan)),
                              float(e.get("tool_axis_deg", np.nan)), float(e.get("rot_deg", np.nan))))
    # comfortable first, then full standoff, small tilt, small spin
    res.sort(key=lambda o: (o.thin, o.standoff_m < pl.approach_offset - 1e-9, o.tilt_deg, abs(o.spin_deg),
                            -o.min_self_dist))
    b = res[:ctx["max_options"]]
    why = None if b else diagnose(model, w, rid, seeds[:40], Rint, tilts, max_spin)
    return b, why


def diagnose(model, w, rid, seeds, Rint, tilts, max_spin) -> str:
    """Why a weld has no option for a robot: no IK solution at all, or which rules the IK solutions violate
    (at the weld pose and on the descent stroke)."""
    ik = model.ik
    qs = ik.solve_pose(rid, w.position, Rint, seeds)
    sols = ik.solve_axis(rid, p=w.position, R=Rint, seeds=seeds, approach=w.approach,
                         standoff=model.scene.planning.approach_offset, min_standoff=model.scene.planning.min_offset,
                         n_steps=10, tilts=tilts, extra_check=None, valid_only=False, max_spin=max_spin)
    if not qs and not sols:
        return "no IK solution for the weld pose (position + tool axis) within the joint limits"
    cnt = {}
    for c in sols:
        rs = list(c.reasons)
        if c.valid or rs == ["no_valid_descent"] or not rs:
            rs = [x for x in rs if x != "no_valid_descent"] + model.static_reasons(rid, c.q)
            if not rs:
                rs = ["descent/lift-off stroke blocked (the straight line to the standoff leaves the valid set)"]
        for x in rs:
            x = x.split("(")[0]
            cnt[x] = cnt.get(x, 0) + 1
    if not cnt:
        return "IK solutions exist but none is valid"
    return "IK branches found but every one fails: " + ", ".join(f"{k} x{v}" for k, v in sorted(cnt.items(), key=lambda kv: -kv[1]))


def _liftoff(model, rid, c, w, pl, n_steps):
    """Separate lift-off line when retract_offset != approach_offset (longest valid prefix >= min_offset)."""
    p1 = w.position - w.approach * pl.retract_offset
    path, ok, info = model.ik.cartesian_path(rid, c.q, w.position, p1, c.R_target, n_steps=n_steps)  # IK frame
    n_ok = 0
    ex = model.extra_check(rid)
    for k in range(1, len(path)):
        if np.abs(path[k] - path[k - 1]).max() > 0.2 or not model.ik.check(rid, path[k]).valid or not ex(path[k]):
            break
        n_ok = k
    L = pl.retract_offset * n_ok / n_steps
    if L < pl.min_offset - 1e-9:
        return None, 0.0
    return np.asarray(path[:n_ok + 1], float), L


def reach_table(reach: dict, welds, rids, reasons: dict | None = None) -> dict:
    """Compact JSON summary for the report."""
    out = {}
    reasons = reasons or {}
    for w in welds:
        row = {}
        for rid in rids:
            ops = reach.get((w.id, rid), [])
            row[rid] = {"n_options": len(ops), "comfortable": sum(not o.thin for o in ops),
                        "reason": None if ops else reasons.get(f"{w.id}|{rid}", "no option"),
                        "best": None if not ops else {"spin_deg": round(ops[0].spin_deg, 1), "tilt_deg": round(ops[0].tilt_deg, 2),
                                                      "min_self_dist_mm": round(ops[0].min_self_dist * 1000, 1),
                                                      "standoff_m": round(ops[0].standoff_m, 4), "thin": ops[0].thin}}
        out[w.id] = row
    return out
