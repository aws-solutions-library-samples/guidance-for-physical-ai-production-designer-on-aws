"""Dense safety / validity re-check of a motion (planned, smoothed, or tracked).

umarell cells (two robots; ``check_cell``), at every sample of the given grid:
  * umarell sphere gate: arm-arm sphere clearance > 0.20 m (env.kinematics.SphereModel, tested equal to umarell);
  * umarell floor and box rules (SphereModel.obstacle_terms);
  * MuJoCo convex-hull contacts self / arm-arm / floor / fixture (KukaWeldEnv; welding_cell_2t with
    ``fixture_hull=True``, the 737 obstacle boxes as box geoms, docs/p3_fixhull.md), every ``hull_every``-th sample;
  * rule W(20 mm, 3 mm) on the TCP of both arms over all 15 welds (docs/workpiece_decision.md), plus the electrode
    points 10 / 20 / 30 mm up the tool axis (informational, as in docs/strict_checks.md);
  * joint limits (robots.yaml);
  * per weld event (robot, weld, hold start, hold end): TCP position error <= 10 mm and tool-axis error <= 5 deg
    over the hold (umarell validate_approach), entry direction over the last 4 cm before the hold <= 15 deg
    (walk-back on the TCP path), TCP drift during the hold (still hold, 0.5 mm).
trajgen scenes (``check_trajgen``): trajgen.verify.verify_timeline (the generator's own dense verifier, corrected
rules, any robot count) on the motion wrapped as an NTimeline.
umarell cells with arms other than robot1 / robot2 (N-arm cells, ``check_cell_n``): the same verifier on the cell
itself (trajgen.scene.load_original(cell, verify=True): umarell's sphere model per arm, every pair; rule W at every
weld; MuJoCo hulls arm-arm / self / floor / non-workpiece boxes; the box rule with the is_workpiece boxes exempt for
the TCP; joint limits; weld pose / entry / still hold per weld event).

``violations`` / ``verdict``: a motion is INVALID against its reference (the source plan) if it violates a check
the reference passes, or deepens an inherited violation by more than 1 mm (the fair_speed.md rule).
"""
from __future__ import annotations

from functools import lru_cache

import numpy as np

R_W, TOL_W = 0.020, 0.003
ELECTRODE_S = (0.0, 0.01, 0.02, 0.03)
POS_TOL, AXIS_TOL_DEG, ENTRY_TOL_DEG, ENTRY_SPAN = 0.010, 5.0, 15.0, 0.04
STILL_TOL = 0.0005


def _r(x, n=5):
    return None if x is None or not np.isfinite(x) else round(float(x), n)


def _model_key():
    """The robot model in effect: robot override + limits policy (as env.cell.load_cell keys its cache)."""
    import os
    from robotmodel.override import active_override, dumps
    return dumps(active_override()), os.environ.get("KWR_LIMITS_SOURCE", "auto")


def _ctx(cell_id):
    """Dense-check context of a cell for the robot model in effect (P3U round 2: the cache was keyed on the cell
    only and kept the first robot after set_robot_override)."""
    return _ctx_cached(cell_id, *_model_key())


_ctx.cache_clear = lambda: _ctx_cached.cache_clear()     # backward compatible with the old lru_cache API


@lru_cache(maxsize=4)
def _ctx_cached(cell_id, _ov_json, _limits_env):
    import mujoco
    from env.cell import ROBOT_IDS, load_cell
    from env.kinematics import ArmKinematics
    from env.rewards import quat_xyzw_to_mat
    from env.weld_env import EnvConfig, KukaWeldEnv
    fh = cell_id != "welding_cell_2_small"
    env = KukaWeldEnv(EnvConfig(cell_id=cell_id, episode_len=10 ** 7, fixture_hull=fh))
    c = load_cell(cell_id)
    kin = [ArmKinematics(c.robots[r]) for r in ROBOT_IDS]
    P = np.array([w.pos for w in c.welds], float)
    A = np.array([np.asarray(w.approach, float) / np.linalg.norm(w.approach) for w in c.welds])
    Rw = quat_xyzw_to_mat(np.array([w.quat_xyzw for w in c.welds]))
    T = -Rw[:, :, 0]
    return dict(env=env, m=env.model, d=mujoco.MjData(env.model), kin=kin, P=P, A=A, T=T,
                ids=[w.wid for w in c.welds], rids=ROBOT_IDS, fixture_hull=fh,
                q_lo=np.stack([c.robots[r].q_lo for r in ROBOT_IDS]), q_hi=np.stack([c.robots[r].q_hi for r in ROBOT_IDS]))


def check_cell(cell_id, ts, Q, weld_events=(), hull_every=2, t_shift=0.0):
    """ts (N,), Q (N, 2, 6) -> report dict. ``t_shift`` is added to the weld-event times (tracking latency)."""
    import mujoco
    C = _ctx(cell_id)
    e, m, d = C["env"], C["m"], C["d"]
    ts = np.asarray(ts, float)
    Q = np.asarray(Q, float).reshape(len(ts), 2, 6)
    sm = e.spheres
    clr = sm.pair_min(Q[:, 0], Q[:, 1])[0]
    fl = [sm.obstacle_terms(r, Q[:, i]) for i, r in enumerate(C["rids"])]
    floor_m = np.stack([f["floor"] for f in fl], -1)
    box_m = np.stack([f["box"] for f in fl], -1)
    sph_m = np.stack([f["sphere"] for f in fl], -1)
    kinds = {k: 0 for k in ("self", "arm_arm", "floor", "fixture")}
    pen = {k: 0.0 for k in kinds}
    first = {}
    self_min = np.full(2, np.inf)
    for n in range(0, len(ts), hull_every):
        d.qpos[e.qadr] = Q[n]
        mujoco.mj_kinematics(m, d)
        mujoco.mj_collision(m, d)
        mj = e._mj_contacts(d)
        for k, v in mj.items():
            if v:
                kinds[k] += 1
                pen[k] = min(pen[k], min(x[2] for x in v))
                first.setdefault(k, (round(float(ts[n]), 4), v[:1]))
        if n % (hull_every * 10) == 0:
            self_min = np.minimum(self_min, e._self_dist_from(d)[:, 0])
    # TCP, electrode points
    pos, rot = [], []
    for i in range(2):
        p, R = C["kin"][i].tcp(Q[:, i])
        pos.append(p)
        rot.append(R)
    pos, rot = np.stack(pos, 1), np.stack(rot, 1)                    # (N,2,3), (N,2,3,3)
    es = np.asarray(ELECTRODE_S)
    X = pos[:, :, None] + es[None, None, :, None] * rot[:, :, None, :, 0]          # (N,2,S,3)
    P, A, T = C["P"], C["A"], C["T"]
    rel = X[:, :, :, None, :] - P
    dist = np.linalg.norm(rel, axis=-1)
    dep = np.minimum((rel * A).sum(-1), (rel * T).sum(-1))
    inball = np.where(dist < R_W, dep, -np.inf)                       # (N,2,S,W)
    w_tcp = inball[:, :, 0]
    viol = w_tcp > TOL_W
    W = dict(n_viol_samples=int(viol.any(-1).any(-1).sum()),
             welds=sorted({f"{C['rids'][i]}/{C['ids'][k]}" for i, k in zip(*np.nonzero(viol.any(0)))}),
             max_depth_mm=_r(w_tcp.max() * 1e3, 3) if np.isfinite(w_tcp.max()) else None)
    elec = {f"s{int(round(x * 1e3))}mm": dict(n_viol_samples=int((inball[:, :, j] > TOL_W).any(-1).any(-1).sum()),
                                              max_depth_mm=_r(inball[:, :, j].max() * 1e3, 3)
                                              if np.isfinite(inball[:, :, j].max()) else None)
            for j, x in enumerate(ELECTRODE_S) if x > 0}
    jl = float(np.maximum(Q - C["q_hi"], C["q_lo"] - Q).max())
    welds = weld_checks(C, ts, pos, rot, weld_events, t_shift)
    k = int(np.argmin(clr))
    out = dict(n_samples=int(len(ts)), hz=_r(1.0 / float(np.median(np.diff(ts))), 1), horizon_s=_r(ts[-1], 3),
               sphere_min_clearance_m=_r(clr[k]), sphere_min_t=_r(ts[k], 3),
               sphere_gate_violations=int((clr <= sm.warning_distance).sum()),
               floor_rule_min_m=_r(floor_m.min()), floor_rule_violations=int((floor_m < 0).sum()),
               box_rule_min_m=_r(box_m.min()) if np.isfinite(box_m.min()) else None,
               obstacle_violations=int(((box_m < 0) | (sph_m < 0)).sum()),
               hull_every=hull_every, hull_contact_samples=kinds, hull_worst_penetration_m={k_: _r(v) for k_, v in pen.items()},
               hull_first_contact=first, hull_self_min_m=[_r(x) for x in self_min],
               fixture_hull_checked=C["fixture_hull"],
               rule_W=W, rule_W_electrode=elec, joint_limit_excess_rad=_r(jl, 6), welds=welds)
    out["violations"] = violations(out)
    return out


def weld_checks(C, ts, pos, rot, weld_events, t_shift=0.0):
    ids = C["ids"]
    rows = []
    for (rid, wid, t0, t1) in weld_events:
        if wid not in ids:
            continue
        i = list(C["rids"]).index(rid)
        k = ids.index(wid)
        a, b = t0 + t_shift, t1 + t_shift
        mm = (ts >= a - 1e-9) & (ts <= b + 1e-9)
        if not mm.any():
            mm[int(np.argmin(np.abs(ts - b)))] = True
        idx = np.flatnonzero(mm)
        p = pos[idx, i]
        perr = np.linalg.norm(p - C["P"][k], axis=1)
        ax = -rot[idx, i, :, 0]
        aerr = np.degrees(np.arccos(np.clip((ax * C["T"][k]).sum(1), -1, 1)))
        drift = float(np.linalg.norm(p - p[0], axis=1).max())
        # entry: walk back from the hold start until 4 cm away
        n0 = idx[0]
        pe = pos[n0, i]
        back = pos[:n0 + 1, i]
        dd = np.linalg.norm(back - pe, axis=1)
        far = np.flatnonzero(dd >= ENTRY_SPAN)
        if far.size:
            v = pe - back[far[-1]]
            ent = float(np.degrees(np.arccos(np.clip(np.dot(v / np.linalg.norm(v), C["A"][k]), -1, 1))))
        else:
            ent = None
        rows.append(dict(robot=rid, weld=wid, hold_s=_r(b - a, 3), pos_err_max_mm=_r(perr.max() * 1e3, 3),
                         axis_err_max_deg=_r(aerr.max(), 3), drift_mm=_r(drift * 1e3, 4), entry_deg=_r(ent, 2)))
    if not rows:
        return dict(n=0)
    ent = [r["entry_deg"] for r in rows if r["entry_deg"] is not None]
    return dict(n=len(rows), pos_err_max_mm=max(r["pos_err_max_mm"] for r in rows),
                axis_err_max_deg=max(r["axis_err_max_deg"] for r in rows),
                drift_max_mm=max(r["drift_mm"] for r in rows), entry_max_deg=max(ent) if ent else None,
                n_pos_fail=sum(r["pos_err_max_mm"] > POS_TOL * 1e3 for r in rows),
                n_axis_fail=sum(r["axis_err_max_deg"] > AXIS_TOL_DEG for r in rows),
                n_entry_fail=sum((r["entry_deg"] or 0) > ENTRY_TOL_DEG for r in rows),
                n_not_still=sum(r["drift_mm"] > STILL_TOL * 1e3 for r in rows),
                worst=sorted(rows, key=lambda r: -r["pos_err_max_mm"])[:3])


def violations(r):
    v = {}
    if r["sphere_gate_violations"]:
        v["sphere_gate"] = r["sphere_min_clearance_m"]
    if r["floor_rule_violations"]:
        v["floor_rule"] = r["floor_rule_min_m"]
    if r["obstacle_violations"]:
        v["box_rule"] = r["box_rule_min_m"]
    for k, n in r["hull_contact_samples"].items():
        if n:
            v[f"hull_{k}"] = r["hull_worst_penetration_m"][k]
    if r["rule_W"]["n_viol_samples"]:
        v["rule_W"] = r["rule_W"]["max_depth_mm"]
    if r["joint_limit_excess_rad"] > 1e-9:
        v["joint_limits"] = r["joint_limit_excess_rad"]
    w = r["welds"]
    if w.get("n", 0):
        if w["n_pos_fail"]:
            v["weld_pos"] = w["pos_err_max_mm"]
        if w["n_axis_fail"]:
            v["weld_axis"] = w["axis_err_max_deg"]
        if w["n_entry_fail"]:
            v["weld_entry"] = w["entry_max_deg"]
        if w["n_not_still"]:
            v["hold_not_still"] = w["drift_max_mm"]
    return v


def verdict(rv, sv):
    """rv / sv: violations of the checked motion / of its reference -> (valid, new_or_worse, inherited)."""
    reasons, inherited = [], []
    for k, x in rv.items():
        if k not in sv:
            reasons.append(f"new {k} ({x})")
            continue
        y = sv[k]
        if k in ("rule_W", "weld_pos", "hold_not_still"):
            worse = (x - y) > 1.0                       # mm
        elif k in ("weld_axis", "weld_entry"):
            worse = (x - y) > 1.0                       # deg
        elif k.startswith("hull_") or k in ("sphere_gate", "floor_rule", "box_rule"):
            worse = (y - x) > 1e-3                      # m (lower = worse)
        else:
            worse = x > y + 1e-3
        (reasons if worse else inherited).append(f"{k} ({x} vs ref {y})")
    return not reasons, reasons, inherited


# ───────────────────────────────────────────── trajgen scenes
@lru_cache(maxsize=16)
def _tg_model(sample_dir):
    from trajgen.model import SceneModel
    from trajgen.scene import load_scene, load_task
    from pathlib import Path
    d = Path(sample_dir)
    return SceneModel(load_scene(d / "scene.yaml"), load_task(d / "task.json"))


def check_trajgen(sample_dir, ts, Q, robot_ids, weld_events=(), check_hz=100.0):
    """trajgen's verifier on a dense motion (wrapped as one leg per robot, linear between the given samples)."""
    return check_model(_tg_model(str(sample_dir)), ts, Q, robot_ids, weld_events, check_hz)


@lru_cache(maxsize=4)
def _cell_model_cached(cell_id, _ov_json, _limits_env):
    from trajgen.model import SceneModel
    from trajgen.scene import load_original, load_original_task
    # umarell's box rule (link spheres + TCP; its tool grid is checked arm-vs-arm only), the tool vs the boxes by the
    # URDF hulls (load_original verify: box_hull on for the non-workpiece boxes)
    return SceneModel(load_original(cell_id, verify=True), load_original_task(cell_id), box_tool_spheres=False)


def cell_model(cell_id):
    """trajgen SceneModel of a umarell cell (any number of arms; ``load_original(verify=True)``) for the robot model
    in effect (robot override + limits policy, as ``_ctx``)."""
    return _cell_model_cached(cell_id, *_model_key())


def check_cell_n(cell_id, ts, Q, robot_ids, weld_events=(), check_hz=100.0):
    """Dense check of a motion in a umarell cell with any arms (N-arm cells; robot1 / robot2 cells use
    ``check_cell``): trajgen's verifier on the cell itself, every robot pair (module docstring). Same report keys
    as ``check_trajgen``; judge it with ``verdict_trajgen``."""
    rep = check_model(cell_model(cell_id), ts, Q, robot_ids, weld_events, check_hz, detail=True)
    rep["model"] = f"umarell cell {cell_id} (trajgen.scene.load_original verify=True), {len(robot_ids)} arms"
    um = rep.get("umarell_sphere_min_m")
    # the umarell sphere gate's clearance (check_cell's sphere_min_clearance_m), else the corrected spheres'
    rep["sphere_min_clearance_m"] = um if isinstance(um, (int, float)) else rep.get("arm_arm_min_m")
    return rep


def check_model(model, ts, Q, robot_ids, weld_events=(), check_hz=100.0, detail=False):
    """trajgen.verify.verify_timeline on a dense motion ``Q`` (N, R, 6) in ``robot_ids`` order, against ``model``.
    ``detail``: also the per-pair clearances, umarell's sphere-gate minimum and the hull contact counts."""
    from player.timeline import Leg
    from trajgen.ntimeline import NTimeline
    from trajgen.verify import verify_timeline
    ts = np.asarray(ts, float)
    legs = []
    for r, rid in enumerate(robot_ids):
        lg = Leg(robot=rid, op_id=f"dense_{rid}", kind="move", t0=float(ts[0]), t1=float(ts[-1]), times=ts,
                 positions=np.asarray(Q)[:, r], phase=np.array([""] * len(ts), dtype="<U16"))
        lg.t_done = lg.t1
        legs.append(lg)
    tl = NTimeline(t=ts, q=np.asarray(Q), robot_ids=tuple(robot_ids), legs=legs, weld_events=list(weld_events),
                   home=np.asarray(Q)[0])
    rep = verify_timeline(model, tl, check_hz=check_hz, umarell_parity=True, dynamics=False)
    chk = rep.get("checks", {})
    v = {k: False for k, ok in chk.items() if not ok}
    out = dict(all_pass=bool(rep.get("all_pass")), checks=chk, violations=v,
               arm_arm_min_m=(rep.get("arm_arm_spheres") or {}).get("min_clearance_m"),
               rule_W=rep.get("rule_W"), welds=_tg_weld_summary(rep.get("welds")), raw_keys=sorted(rep))
    if detail:
        aa = rep.get("arm_arm_spheres") or {}
        hh = rep.get("arm_arm_hull") or {}
        out.update(umarell_sphere_min_m=aa.get("umarell_35_spheres_min_m"),
                   pairs={k: {kk: vv for kk, vv in r.items() if not isinstance(vv, (list, dict))}
                          for k, r in (aa.get("pairs") or {}).items()},
                   hull_contact_samples=hh.get("contact_samples"), hull_min_signed_distance_m=hh.get("min_signed_distance_m"),
                   self_hull_contact_samples=(rep.get("self_hull") or {}).get("contact_samples"),
                   box_hull=(rep.get("box_hull") or {}).get("violation_samples"))
    return out


def _tg_weld_summary(w):
    if not isinstance(w, dict):
        return w
    return {k: v for k, v in w.items() if not isinstance(v, (list, dict))}


def verdict_trajgen(rv, sv):
    reasons = [f"new {k}" for k in rv if k not in sv]
    return not reasons, reasons, [k for k in rv if k in sv]


def tcp_error(plan, Qa, Qr, holds_mask=None):
    """Max TCP position error (m) between actual Qa and reference Qr (N, R, 6), overall and inside ``holds_mask``
    (N, R) bool."""
    if plan.kind == "trajgen":
        m = _tg_model(str(__import__("tools.ctrl.plans", fromlist=["ROOT"]).ROOT / plan.source))
        f = lambda r, Q: m.tcp(plan.robot_ids[r], Q)[0]
    else:
        C = _ctx(plan.cell)
        f = lambda r, Q: C["kin"][r].tcp(Q)[0]
    e = np.stack([np.linalg.norm(f(r, Qa[:, r]) - f(r, Qr[:, r]), axis=1) for r in range(plan.R)], 1)
    out = dict(max_tcp_err_mm=_r(e.max() * 1e3, 3), p99_tcp_err_mm=_r(np.percentile(e.max(1), 99) * 1e3, 3))
    if holds_mask is not None and holds_mask.any():
        out["max_tcp_err_in_holds_mm"] = _r(e[holds_mask].max() * 1e3, 4)
    return out
