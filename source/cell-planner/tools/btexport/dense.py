"""Our dense safety re-check of an exported plan (the motion umarell's executor would play), against its source.

The exported plan is replayed on the executor clock (player.bt_plan / trajgen.btplan rules: holds, 1.0 s completion
margins, Delay / WaitForRobot gates) and sampled at ``hz`` (default 400 Hz). Checks (tools.ctrl.safety, the same as
tools.retime's re-check):

* umarell cells (robot1 / robot2): ``check_cell`` -- umarell's sphere gate, floor / box rules, MuJoCo hulls (self,
  arm-arm, floor, fixture), rule W (TCP + electrode), joint limits of the cell's robots, weld pose / tool axis / entry
  direction / still hold at every weld leg end;
* N-arm umarell cells (arms other than robot1 / robot2, e.g. robot1..robot4): ``check_cell_n`` at 100 Hz --
  trajgen.verify.verify_timeline on the cell itself (trajgen.scene.load_original(verify=True): umarell's sphere gate
  and the corrected spheres on every pair, MuJoCo hulls arm-arm / self (SRDF pairs off) / floor / non-workpiece boxes,
  the box rule with the is_workpiece boxes exempt for the TCP, rule W at every weld, joint limits, weld pose / entry /
  still hold per weld event); umarell's own validators (--validate) stay two-arm;
* trajgen scenes (1-N robots): ``check_trajgen`` at 100 Hz -- trajgen.verify.verify_timeline (corrected + umarell sphere gates on
  every pair, hull buffer, self hulls, floor, boxes, rule W, joint limits, weld checks).

Validity = the fair_speed.md rule (``verdict``): no violation kind that the source does not have, none deeper than the
source's by more than the tolerance. The source's own motion (rollout at its env clock, plan at its executor clock) is
checked the same way.
"""
from __future__ import annotations

import os
import time

import numpy as np

SCENE_CHECK_HZ = 100.0     # trajgen.verify sampling in scenes (tools.retime's SceneContext uses the same)


def _cell_path(src) -> bool:
    return src.scene is None or bool(src.umarell_cell)


def _pair_cell(src) -> bool:
    """A umarell cell checked by check_cell (robot1 / robot2): the two-arm env model."""
    return _cell_path(src) and set(src.robot_ids) == {"robot1", "robot2"}


def _n_cell(src) -> bool:
    """A umarell cell with other arms (N-arm cells): check_cell_n."""
    return _cell_path(src) and not _pair_cell(src)


def _check_key(src, ts, Q, weld_events, hull_every) -> str | None:
    """Content key of a dense check of a umarell cell: the sampled motion itself (ts, Q, weld events), the cell's files
    (umarell_fast.plan_cache.tree_digest: content, not id / location / mtimes), the robot model in effect and the
    checker's code (tools/ctrl, tools/btexport, trajgen, env, robotmodel). None for trajgen scenes (not cached)."""
    import hashlib
    from pathlib import Path
    from env.cell import load_cell
    from tools.ctrl import safety as S
    from umarell_fast.plan_cache import canon, tree_digest
    if not _cell_path(src):
        return None
    root = Path(__file__).resolve().parents[2]
    h = hashlib.sha256()
    for sub in ("tools/ctrl", "tools/btexport", "trajgen", "env", "robotmodel"):
        for p in sorted((root / sub).rglob("*.py")):
            h.update(str(p.relative_to(root)).encode() + b"\0" + hashlib.sha256(p.read_bytes()).digest())
    cell = src.umarell_cell or src.cell_id
    h.update(tree_digest(load_cell(cell).root)[0].encode())
    h.update(canon([S._model_key(), list(src.robot_ids), hull_every, SCENE_CHECK_HZ,
                    [list(map(str, e)) for e in weld_events]]).encode())
    h.update(np.ascontiguousarray(np.asarray(ts, np.float64)).tobytes())
    h.update(np.ascontiguousarray(np.asarray(Q, np.float64)).tobytes())
    return h.hexdigest()


def check_motion(src, ts, Q, weld_events, *, hull_every=1):
    """Q (N, R, 6) in ``src.robot_ids`` order. With ``BTEXPORT_CHECK_CACHE_DIR`` set, a check of exactly this motion
    in exactly this cell (``_check_key``) done before is replayed from there (a re-plan that reproduces a plan: U3's
    warm task cache)."""
    cdir = os.environ.get("BTEXPORT_CHECK_CACHE_DIR", "").strip()
    key = None
    if cdir:
        import json
        from pathlib import Path
        try:
            key = _check_key(src, ts, Q, weld_events, hull_every)
        except Exception:  # noqa: BLE001 -- no key: check as before
            key = None
        f = Path(cdir) / f"{key}.json" if key else None
        if f is not None and f.is_file():
            try:
                rep = json.loads(f.read_text())
                rep["cached"] = True
                return rep
            except (OSError, ValueError):
                pass
    rep = _check_motion(src, ts, Q, weld_events, hull_every=hull_every)
    if cdir and key:
        import json
        import tempfile
        from pathlib import Path
        try:
            Path(cdir).mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=cdir, prefix=".chk-", suffix=".tmp")
            with os.fdopen(fd, "w") as fh:
                json.dump(rep, fh, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
            os.replace(tmp, Path(cdir) / f"{key}.json")
        except Exception:  # noqa: BLE001 -- the cache never fails a check
            pass
    return rep


def _check_motion(src, ts, Q, weld_events, *, hull_every=1):
    from tools.ctrl import safety as S
    if _n_cell(src):
        return S.check_cell_n(src.umarell_cell or src.cell_id, ts, Q, list(src.robot_ids), weld_events,
                              check_hz=SCENE_CHECK_HZ)
    if _cell_path(src):
        rids = list(src.robot_ids)
        order = [rids.index(r) for r in ("robot1", "robot2")]
        return S.check_cell(src.umarell_cell or src.cell_id, ts, np.asarray(Q)[:, order], weld_events,
                            hull_every=hull_every)
    from pathlib import Path
    sd = Path(src.scene)
    sd = sd.parent if sd.is_file() else sd               # check_trajgen reads <dir>/scene.yaml + <dir>/task.json
    if src.task is not None and Path(src.task).resolve() != (sd / "task.json").resolve():
        raise ValueError(f"dense check: task {src.task} is not {sd / 'task.json'}")
    return S.check_trajgen(str(sd), ts, Q, list(src.robot_ids), weld_events, check_hz=SCENE_CHECK_HZ)


def source_motion(src, *, hz=400.0):
    """The source's own dense motion (rollout at its own clock, plan at its executor clock) -> (ts, Q, weld events)."""
    from trajgen.ntimeline import NTimeline
    from tools.btexport.core import executor_of, replay, weld_events_of
    if src.kind == "rollout":
        t = np.asarray(src.t, float)
        q = np.asarray(src.q, float)
        ts = np.arange(t[0], t[-1] + 1e-9, 1.0 / hz)
        Q = np.stack([np.stack([np.interp(ts, t, q[:, r, j]) for j in range(6)], -1) for r in range(q.shape[1])], 1)
        ev = [(str(a), str(b), float(c), float(d)) for a, b, c, d in src.weld_events]
    else:
        legs, meta = replay(src.plan, src.robot_ids, src.home, executor_of(src))
        T = max([l.t_done for l in legs] + [l.t1 for l in legs])
        tl = NTimeline.from_legs(legs, robot_ids=src.robot_ids, dt=0.05, t_end=T, home=np.asarray(src.home, float))
        ts = np.arange(0.0, T + 1e-9, 1.0 / hz)
        Q = tl.sample(ts)
        ev = weld_events_of(src.plan["behaviour_tree"], legs)
    return ts, Q, ev


def source_check(src, *, hz=400.0, hull_every=1, motion=None) -> dict:
    """Dense check of the source itself (rollout at its own clock, plan at its executor clock)."""
    ts, Q, ev = source_motion(src, hz=hz) if motion is None else motion
    t0 = time.time()
    rep = check_motion(src, ts, Q, ev, hull_every=hull_every)
    rep["check_s"] = round(time.time() - t0, 1)
    return rep


def plan_check(plan, src, *, ref=None, hz=400.0, hull_every=1) -> dict:
    """Dense check of an exported plan; ``src`` = the ORIGINAL source (geometry, robots; reference violations)."""
    from trajgen.ntimeline import NTimeline
    from tools.btexport.core import executor_of, replay, weld_events_of
    from tools.ctrl import safety as S
    legs, meta = replay(plan, src.robot_ids, src.home, executor_of(src))
    T = max([l.t_done for l in legs] + [l.t1 for l in legs])
    tl = NTimeline.from_legs(legs, robot_ids=src.robot_ids, dt=0.05, t_end=T, home=np.asarray(src.home, float))
    ts = np.arange(0.0, T + 1e-9, 1.0 / hz)
    Q = tl.sample(ts)
    ev = weld_events_of(plan["behaviour_tree"], legs)
    t0 = time.time()
    rep = check_motion(src, ts, Q, ev, hull_every=hull_every)
    rep["check_s"] = round(time.time() - t0, 1)
    ref_note = None
    skipped = False
    if ref is None and not rep["violations"] and os.environ.get("BTEXPORT_ALWAYS_SOURCE_CHECK", "0") == "0":
        # The source's check only excuses violations the export inherits (verdict / verdict_trajgen judge the
        # export's violations against it): with none in the export it cannot change the verdict. Skipping it saves
        # a second full dense check (body_weld_3, 4 arms: ~85 s).
        ref, ref_note, skipped = {"violations": {}}, "not run: the export has no violations, so none can be new", True
    if ref is None:
        mo = source_motion(src, hz=hz)
        if mo[0].shape == ts.shape and np.array_equal(mo[0], ts) and np.array_equal(mo[1], Q) and \
                [tuple(e) for e in mo[2]] == [tuple(e) for e in ev]:
            # the export plays exactly the source's motion (e.g. the umarell profile keeps the tree): same check
            ref, ref_note = rep, "the source's motion is identical sample for sample: its check is the export's"
        else:
            ref = source_check(src, hz=hz, hull_every=hull_every, motion=mo)
    vf = S.verdict if _pair_cell(src) else S.verdict_trajgen
    valid, reasons, inherited = vf(rep["violations"], ref["violations"])
    jl = rep.get("joint_limit_excess_rad")
    try:                     # absolute URDF position limits on the dense samples (cells AND trajgen scenes)
        from tools.btexport.limits import limits_for
        lims = limits_for(cell_id=src.cell_id if src.scene is None else None, scene=src.scene)
        lo = np.stack([np.asarray(lims[r].lower, float) for r in src.robot_ids])
        hi = np.stack([np.asarray(lims[r].upper, float) for r in src.robot_ids])
        Qa = np.asarray(Q, float)
        jl_urdf = float(np.maximum(np.maximum(lo[None] - Qa, 0), np.maximum(Qa - hi[None], 0)).max())
    except Exception:  # noqa: BLE001
        jl_urdf = None
    return {"valid": bool(valid), "new_or_worse": reasons, "inherited": inherited, "hz": hz,
            "violations": rep["violations"], "source_violations": None if skipped else ref["violations"],
            "sphere_min_m": rep.get("sphere_min_clearance_m", rep.get("arm_arm_min_m")),
            "source_sphere_min_m": ref.get("sphere_min_clearance_m", ref.get("arm_arm_min_m")),
            "hull_contacts": rep.get("hull_contact_samples"), "joint_limit_excess_rad": jl if jl is not None else (
                round(jl_urdf, 6) if jl_urdf is not None else None),
            "urdf_joint_limit_excess_rad": None if jl_urdf is None else round(jl_urdf, 6),
            "rule_W": rep.get("rule_W"), "welds": rep.get("welds"), "checks": rep.get("checks"),
            "check_s": rep["check_s"], **({"source_check": ref_note} if ref_note else {}),
            **({"model": rep.get("model"), "pairs": rep.get("pairs"),
                "source_hull_contacts": ref.get("hull_contact_samples"),
                "not_checked": N_CELL_NOT_CHECKED} if _n_cell(src) else {})}


# what the N-arm dense check does not cover (vs check_cell + umarell's validators of a two-arm cell)
N_CELL_NOT_CHECKED = ("umarell's own validators and executor dry run (--validate is two-arm)",
                      "electrode-vs-workpiece-box hull contact (the is_workpiece boxes are exempt for the TCP / "
                      "electrode, as in umarell's box rule; only rule W guards the weld surfaces)",
                      "rule W's electrode points 10-30 mm up the tool axis (check_cell's informational extra)")
