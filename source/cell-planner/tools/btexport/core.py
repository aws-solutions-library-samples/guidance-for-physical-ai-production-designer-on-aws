"""Source -> umarell behaviour-tree plan (JSON envelope + BTCPP v4 XML) with one set of rules for every tool.

Unified behaviour (whatever produced the motion):
  * envelope = umarell's ``plan_store.save_plan`` layout: {tree_id, cell_id, created_s, status, num_operations,
    behaviour_tree {tree_id, cell_id, BTCPP_format "4", main_tree_to_execute, root, blackboard, metadata, xml}};
    blackboard entries ``traj_<robot>_<op>`` = [{positions, time_s, phase}] (unique keys, trajgen.btplan rules);
  * executor-consistent timing: every leg's start on umarell's executor clock (completion margin 1.0 s, holds) is the
    intended one (``timing_check``); the plan is replayed with the executor rules (player.bt_plan for umarell-cell
    plans, any number of arms, and robot1/robot2 plans; trajgen.btplan.replay for N-robot trajgen scenes);
  * at-rest cuts: every leg starts and ends with its arm at rest, so no executor pause freezes a moving arm
    (``rest_check``, threshold 3 % v_max over the last / first 5 ms);
  * real holds: a weld leg ends on its weld (FK of the last waypoint, ``weld_end_check``) and the arm stands still
    there for hold_duration_s (0.5 s) + the margin;
  * metadata: exporter, source, profile, URDF limits (``tools.btexport.limits``), executor_margin_s and
    umarell_default_completion_margin_s, makespans (motion end, executor end, umarell's own schedule estimate),
    the check results.
Exporters reused, not duplicated: execute.py ``legs_sync_rest`` (rollouts), trajgen.btplan ``plan_from_steps`` /
the source tree (plans), player.bt_plan ``to_xml`` (byte-identical to umarell's btcpp.to_xml) and the replays.
"""
from __future__ import annotations

import copy
import json
import time
from pathlib import Path
from typing import Optional

import numpy as np

from tools.btexport import limits as L
from tools.btexport import retime as RT

ROOT_DIR = Path(__file__).resolve().parents[2]
EXPORTER = "tools.btexport"
EXPORTER_VERSION = 1
MARGIN_S = 1.0
REST_FRAC = 0.03
# absolute limit gate (P3U fix, 2026-10-12): every export must stay inside the URDF limits it records, whatever its
# source did. Velocity: exact segment slopes of the waypoints umarell's controller interpolates linearly (tolerance
# = the 1e-6 rad position rounding of the plan writer over >= 1 ms segments); acceleration: the 0.05 s second
# difference of execute.plan_dynamics (its documented discretisation tolerance, 5 %); position: the URDF
# <limit lower upper> (tolerance = the position rounding).
TOL_V = 1e-3
TOL_A = 0.05
TOL_Q = 1e-5
TOL_WELD_V = 0.01            # full profile: weld-operation speed <= weld_vfrac + this
TOL_JERK = 0.05              # retime backend: within-leaf jerk <= tools.retime's cap x (1 + this) (0.05 s metric on
                             # 100 Hz-resampled, linearly interpolated waypoints)
TIME_QUANTUM_S = 1e-4        # umarell trajectories carry time_s rounded to 0.1 ms (player Leg.rel_trajectory)
MIN_SEG_S = 5e-4


class Unsupported(ValueError):
    pass


def _r(x, n=4):
    return None if x is None else round(float(x), n)


WELD_V_RETIME = 0.28        # tools.retime's default weld-operation velocity fraction (umarell 0.7 x 0.4)


def _g_(d, *ks):
    for k in ks:
        if not isinstance(d, dict):
            return None
        d = d.get(k)
    return d


def _scene_weld_vscale(src):
    try:
        from trajgen.scene import load_scene
        return float(load_scene(src.scene).planning.approach_vscale)
    except Exception:  # noqa: BLE001
        return None


# ───────────────────────────────────────── plan structure ─────────────────────────────────────────
def _kind(n):
    from player.bt_plan import node_kind
    return node_kind(n)


def _leaves(node) -> list:
    k = _kind(node)
    if k in ("ExecuteTrajectory", "MoveHome", "WaitForRobot", "CheckSafety"):
        return [node]
    return [x for c in node.get("children") or [] for x in _leaves(c)]


def _step_nodes(node) -> list:
    """Top-level steps of a root Sequence; nested Sequences are flattened (they run their children one after the
    other, exactly like consecutive steps)."""
    out = []
    for c in node.get("children") or []:
        if _kind(c) in ("Sequence", "SequenceWithMemory"):
            out += _step_nodes(c)
        else:
            out.append(c)
    return out


def plan_shape(tree: dict) -> str:
    """"steps": root Sequence of steps, a step = a leaf, a Delay(leaf) or a Parallel of those (one per robot);
    "sequences": anything else the executor runs (per-robot sequences with WaitForRobot gates, ...)."""
    root = tree.get("root") or {}
    if _kind(root) not in ("Sequence", "SequenceWithMemory"):
        return "sequences"

    def leafish(n):
        k = _kind(n)
        if k in ("ExecuteTrajectory", "MoveHome"):
            return True
        if k == "Delay":
            kids = n.get("children") or []
            return len(kids) == 1 and _kind(kids[0]) in ("ExecuteTrajectory", "MoveHome")
        return False

    for c in _step_nodes(root):
        if leafish(c):
            continue
        if _kind(c) == "Parallel" and all(leafish(x) for x in c.get("children") or []):
            robots = [(_leaves(x)[0].get("ports") or {}).get("robot_ns") for x in c.get("children") or []]
            if len(set(robots)) == len(robots):
                continue
        return "sequences"
    return "steps"


def executor_of(src) -> Optional[str]:
    """The executor semantics a source's plans replay with: "umarell" for a plan in a umarell cell (no trajgen scene:
    player.bt_plan, any number of arms, blind MoveHome included), None (= by robot set) otherwise."""
    return "umarell" if (getattr(src, "scene", None) is None and getattr(src, "umarell_cell", None)) else None


def replay(plan: dict, robot_ids, home, executor: Optional[str] = None):
    """Executor-clock legs of a plan -> (legs, meta). ``executor``: "umarell" = player.bt_plan (umarell's executor
    rules) for any robot set; "trajgen" = trajgen.btplan; None = umarell for robot1 / robot2, trajgen otherwise."""
    if executor not in (None, "umarell", "trajgen"):
        raise ValueError(f"executor {executor!r}")
    pair = tuple(sorted(robot_ids)) == ("robot1", "robot2")
    if executor == "umarell" or (executor is None and pair):
        from player.bt_plan import timeline_from_bt_plan
        from player.timeline import ROBOT_IDS
        h = np.asarray(home, float).reshape(len(robot_ids), 6)
        if pair:
            h = np.stack([h[list(robot_ids).index(r)] for r in ROBOT_IDS])
            tl = timeline_from_bt_plan(plan, home=h)
        else:
            tl = timeline_from_bt_plan(plan, home=h, robot_ids=tuple(robot_ids))
        return tl.legs, dict(tl.meta)
    from trajgen.btplan import replay as nrep
    tl = nrep(plan, robot_ids, home)
    meta = dict(tl.meta)
    meta.setdefault("umarell_schedule_makespan_s", None)
    return tl.legs, meta


def steps_from_tree(tree: dict, legs) -> list:
    """Replayed legs grouped by the plan's top-level steps (shape "steps")."""
    pool = {}
    for l in sorted(legs, key=lambda l: l.t0):
        pool.setdefault((l.robot, l.op_id), []).append(l)
    out = []
    for c in _step_nodes(tree["root"]):
        st = []
        for lf in _leaves(c):
            p = lf.get("ports") or {}
            key = (p.get("robot_ns"), str(p.get("operation_id")) if _kind(lf) == "ExecuteTrajectory" else "home")
            lst = pool.get(key) or []
            if not lst:
                raise Unsupported(f"leaf {key} not found in the replay")
            st.append(lst.pop(0))
        out.append(st)
    return out


def group_steps(legs, robot_ids, tol=1e-6) -> list:
    legs = sorted([l for l in legs if not l.synthetic], key=lambda l: (l.t0, list(robot_ids).index(l.robot)))
    steps = []
    for l in legs:
        if steps and abs(steps[-1][0].t0 - l.t0) <= tol and l.robot not in {x.robot for x in steps[-1]}:
            steps[-1].append(l)
        else:
            steps.append([l])
    return steps


def normalize_tree(tree: dict) -> dict:
    """Make the tree valid BehaviorTree.CPP v4 and lossless through umarell's XML parser, without changing what
    umarell's executor does (in place; returns a summary):
      * a ``Delay`` without a child (trajgen.btplan's gate branches) gets a ``CheckSafety`` child (BT.CPP decorators
        take exactly one child; umarell's validate_behaviour_tree warns "Delay has no children"; CheckSafety is
        instantaneous in umarell's executor unless its collision monitor reports, in which case it aborts the run);
      * list / dict port values written inline (``depends_on_ops`` etc.; umarell's btcpp keeps only trajectory,
        interlock_target and depends_on_* (not _ops) on the blackboard) move to the blackboard as ``{key}``
        references, so ``btcpp.from_xml(xml, blackboard)`` gives the same tree back (inline JSON attributes come back
        as strings, which the executor would iterate character by character)."""
    import hashlib
    bb = tree.setdefault("blackboard", {})
    n_delay, n_moved = [0], [0]

    def key_for(node, pname, value):
        h = hashlib.sha1(json.dumps([node.get("name"), node.get("id"), pname, value], sort_keys=True,
                                    default=str).encode()).hexdigest()[:8]
        k = f"{pname}_{(node.get('ports') or {}).get('robot_ns', 'n')}_{h}"
        i = 0
        while k in bb and bb[k] != value:
            i += 1
            k = f"{pname}_{h}_{i}"
        return k

    def walk(n):
        ports = n.get("ports")
        if isinstance(ports, dict):
            for pname, v in list(ports.items()):
                if isinstance(v, (list, dict)):
                    k = key_for(n, pname, v)
                    bb[k] = v
                    ports[pname] = "{" + k + "}"
                    n_moved[0] += 1
        if _kind(n) == "Delay" and not (n.get("children") or []):
            h = hashlib.sha1(json.dumps([n.get("name"), n.get("id")], default=str).encode()).hexdigest()[:8]
            n["children"] = [{"type": "CheckSafety", "name": f"after_{n.get('name') or 'delay'}", "id": h,
                              "ports": {"condition": "estop_clear"}}]
            n_delay[0] += 1
        for c in n.get("children") or []:
            walk(c)

    walk(tree["root"])
    return {"childless_delays_given_checksafety": n_delay[0], "inline_ports_moved_to_blackboard": n_moved[0]}


# ───────────────────────────────────────── checks ─────────────────────────────────────────
def dense_dynamics(legs, robot_ids, home, v_max, a_max, T, *, hz=200.0, h=0.05) -> dict:
    """execute.plan_dynamics for any number of robots: joint velocity (central difference over 1/hz), acceleration
    and jerk (second / third differences with h = 0.05 s) of the replayed motion, as fractions of the URDF v_max and
    the a_max override, on a 200 Hz grid over [0, T]."""
    from trajgen.ntimeline import NTimeline
    tl = NTimeline(t=np.zeros(1), q=np.zeros((1, len(robot_ids), 6)), robot_ids=tuple(robot_ids), legs=list(legs),
                   home=np.asarray(home, float).reshape(len(robot_ids), 6))
    ts = np.arange(0.0, T + 1e-9, 1.0 / hz)
    d = 0.5 / hz
    qf = lambda x: tl.sample(np.clip(x, 0.0, T))
    v = (qf(ts + d) - qf(ts - d)) / (2 * d)
    a = (qf(ts + h) - 2 * qf(ts) + qf(ts - h)) / h ** 2
    j = (qf(ts + 1.5 * h) - 3 * qf(ts + 0.5 * h) + 3 * qf(ts - 0.5 * h) - qf(ts - 1.5 * h)) / h ** 3
    out = {"grid_hz": hz, "fd_h_s": h, "T_s": _r(T, 3)}
    for name, r in (("vel_frac_vmax", np.abs(v) / v_max), ("acc_frac_amax", np.abs(a) / a_max),
                    ("jerk_amax_per_s", np.abs(j) / a_max)):
        m = r.max(axis=(1, 2))
        k = int(m.argmax())
        rr, jj = np.unravel_index(int(r[k].argmax()), r.shape[1:])
        out[name] = {"max": _r(m.max(), 3), "t_max_s": _r(ts[k], 3), "at": f"{robot_ids[rr]} a{jj + 1}",
                     "p99": _r(np.percentile(m, 99), 3)}
    ma = (np.abs(a) / a_max).max(axis=(1, 2))
    out["acc_frac_samples_over"] = {str(x): _r((ma > x).mean(), 4) for x in (0.49, 1.0)}
    mv = (np.abs(v) / v_max).max(axis=(1, 2))
    out["time_frac_at_ge_0.9_vmax"] = _r((mv >= 0.9).mean(), 4)
    return out


def rest_check(legs, v_max_by_robot: dict, w: float = 0.005) -> dict:
    """Arm speed (fraction of v_max) over the first / last ``w`` s of every leg (umarell's own plans start and end
    every trajectory at rest; an arm frozen mid-motion by an executor pause shows up here). Each leg is measured on
    its own waypoints (linear interpolation, as the executor plays them), so the number of arms and the replay's
    schedule do not enter. ``not_at_rest``: every end over the threshold, ``[robot, op, "start" | "end", frac,
    "a<joint>"]`` in leg order."""
    from player.timeline import sample_leg
    rows, over = [], []
    for l in legs:
        if l.synthetic or len(l.times) < 2:
            continue
        vm = v_max_by_robot[l.robot]
        a = np.abs(sample_leg(l, np.array([l.t0 + w]))[0] - l.positions[0]) / w / vm
        b = np.abs(l.positions[-1] - sample_leg(l, np.array([l.t1 - w]))[0]) / w / vm
        rows.append((l.robot, l.op_id, float(a.max()), float(b.max())))
        for end, r in (("start", a), ("end", b)):
            if float(r.max()) > REST_FRAC:
                over.append([l.robot, l.op_id, end, _r(float(r.max()), 4), f"a{int(r.argmax()) + 1}"])
    worst = sorted(rows, key=lambda r: -max(r[2], r[3]))[:3]
    mx = max([max(r[2], r[3]) for r in rows], default=0.0)
    return {"n_legs": len(rows), "max_speed_frac_vmax": _r(mx, 4), "threshold": REST_FRAC,
            "n_legs_not_at_rest": sum(max(r[2], r[3]) > REST_FRAC for r in rows), "pass": bool(mx <= REST_FRAC),
            "worst": [(r, o, _r(a, 4), _r(b, 4)) for r, o, a, b in worst], "not_at_rest": over}


def weld_end_check(legs, tcp: dict, wpos: dict, tol_m: float = 0.010) -> dict:
    rows = []
    for l in legs:
        if l.kind != "weld":
            continue
        p = wpos.get(l.op_id)
        if p is None:
            rows.append((l.robot, l.op_id, None, float(l.hold_s)))
            continue
        e = float(np.linalg.norm(tcp[l.robot](l.positions[-1:])[0] - p))
        rows.append((l.robot, l.op_id, e, float(l.hold_s)))
    errs = [r[2] for r in rows if r[2] is not None]
    holds = [r[3] for r in rows]
    return {"n_weld_legs": len(rows), "max_end_error_mm": _r(max(errs) * 1000, 3) if errs else None,
            "unknown_welds": [r[1] for r in rows if r[2] is None], "min_hold_s": _r(min(holds), 3) if holds else None,
            "pass": bool(rows) and all(r[2] is not None and r[2] <= tol_m for r in rows) and min(holds) >= 0.5 - 1e-9,
            "tolerance_m": tol_m, "hold_rule": "hold_duration_s >= 0.5 at rest (umarell default hold)"}


def weld_events_of(tree: dict, legs) -> list:
    """[(robot, weld id, t_arrive, t_leave)] on the executor clock. Per-weld leaves: the weld legs (arrival at the leg
    end, hold_s). tools.retime.bt window exports (a leaf may hold several welds): the welds of the metadata
    ``leaves`` map, shifted by the 1.0 s completion margin of every earlier step."""
    md = tree.get("metadata") or {}
    lv = md.get("leaves")
    if lv and str(md.get("generator", "")).startswith("tools.retime"):
        m = float(md.get("completion_margin_s", MARGIN_S))
        return [(e["robot"], str(w[0]), float(w[1]) + m * int(e["step"]), float(w[2]) + m * int(e["step"]))
                for e in lv for w in e.get("welds") or []]
    return [(l.robot, str(l.op_id), float(l.t1), float(l.t1 + l.hold_s)) for l in legs if l.kind == "weld"]


def weld_event_check(events, sample, robot_ids, tcp: dict, wpos: dict, tol_m: float = 0.010) -> dict:
    """weld_end_check for window exports: FK at every weld's arrival on the replayed executor clock, and the
    length of its still interval (the hold inside the leaf)."""
    rows = []
    for rid, wid, ta, tl in events:
        p = wpos.get(wid)
        if p is None:
            rows.append((rid, wid, None, tl - ta))
            continue
        q = sample(np.array([ta, 0.5 * (ta + tl), tl]))[:, list(robot_ids).index(rid)]
        e = float(np.linalg.norm(tcp[rid](q) - p, axis=1).max())
        rows.append((rid, wid, e, tl - ta))
    errs = [r[2] for r in rows if r[2] is not None]
    holds = [r[3] for r in rows]
    return {"n_weld_legs": len(rows), "max_end_error_mm": _r(max(errs) * 1000, 3) if errs else None,
            "unknown_welds": [r[1] for r in rows if r[2] is None], "min_hold_s": _r(min(holds), 3) if holds else None,
            "pass": bool(rows) and all(r[2] is not None and r[2] <= tol_m for r in rows) and min(holds) >= 0.5 - 1e-6,
            "tolerance_m": tol_m, "hold_rule": "still interval >= 0.5 s inside the leaf (tools.retime window export)",
            "mode": "weld events of the window export's leaves map"}


def weld_op_speed_check(legs, tcp: dict, wpos: dict, v_max_by_robot: dict, radius: float = RT.WELD_RADIUS,
                        hz: float = 200.0) -> dict:
    """Peak joint speed (fraction of v_max, 200 Hz central differences) of each welding arm while its TCP is within
    ``radius`` of the weld: the end of the weld leg (approach descent) and the start of the arm's next leg
    (lift-off)."""
    from player.timeline import sample_leg
    by = {}
    for l in sorted(legs, key=lambda l: l.t0):
        by.setdefault(l.robot, []).append(l)
    rows = []
    for rid, ls in by.items():
        for i, l in enumerate(ls):
            if l.kind != "weld" or l.op_id not in wpos:
                continue
            for part, leg in (("approach", l), ("liftoff", ls[i + 1] if i + 1 < len(ls) else None)):
                if leg is None or leg.t1 - leg.t0 < 2.0 / hz:
                    continue
                ts = np.arange(leg.t0 + 0.5 / hz, leg.t1 - 0.5 / hz, 1.0 / hz)
                if len(ts) == 0:
                    continue
                d = 0.5 / hz
                v = np.abs(sample_leg(leg, ts + d) - sample_leg(leg, ts - d)) / (2 * d) / v_max_by_robot[rid]
                near = np.linalg.norm(tcp[rid](sample_leg(leg, ts)) - wpos[l.op_id], axis=1) <= radius
                vt = np.linalg.norm(tcp[rid](sample_leg(leg, ts + d)) - tcp[rid](sample_leg(leg, ts - d)), axis=1) / (2 * d)
                # the contiguous run at the leg end (approach) / start (lift-off), as in retime.weld_op_mask
                run = np.zeros_like(near)
                rng = range(len(ts) - 1, -1, -1) if part == "approach" else range(len(ts))
                for k in rng:
                    if not near[k]:
                        break
                    run[k] = True
                if run.any():
                    rows.append((rid, l.op_id, part, float(v[run].max()), float(vt[run].max())))
    mx = max([r[3] for r in rows], default=0.0)
    return {"max_speed_frac_vmax": _r(mx, 4), "max_tcp_speed_m_s": _r(max([r[4] for r in rows], default=0.0), 3),
            "n": len(rows), "radius_m": radius,
            "worst": [(a, b, c, _r(x, 4)) for a, b, c, x, _ in sorted(rows, key=lambda r: -r[3])[:3]]}


def weld_zone_speed_events(events, sample, T, robot_ids, tcp: dict, wpos: dict, v_max_by_robot: dict,
                           radius: float = RT.WELD_RADIUS, hz: float = 200.0, lookaround_s: float = 15.0) -> dict:
    """``weld_op_speed_check`` for exports whose leaves may hold several welds (tools.retime.bt window exports): per
    weld event (robot, weld, t_arrive, t_leave) on the replayed executor clock, the welding arm's peak joint speed
    (fraction of v_max) and TCP speed while its TCP is within ``radius`` of THAT weld, contiguous before the arrival
    (approach) and after the departure (lift-off), whatever the leaf structure. ``n`` = 0 -> None (not measured)."""
    rows = []
    d = 0.5 / hz
    for rid, wid, ta, tl in events:
        if wid not in wpos or rid not in robot_ids:
            continue
        r = list(robot_ids).index(rid)
        for part, ts in (("approach", np.arange(ta - 1.0 / hz, max(ta - lookaround_s, 0.0), -1.0 / hz)),
                         ("liftoff", np.arange(tl + 1.0 / hz, min(tl + lookaround_s, T), 1.0 / hz))):
            if len(ts) == 0:
                continue
            Q = sample(ts)[:, r]
            near = np.linalg.norm(tcp[rid](Q) - wpos[wid], axis=1) <= radius
            k = int(np.argmin(near)) if not near.all() else len(near)
            if k == 0:
                continue
            tt = ts[:k]
            qa, qb = sample(np.clip(tt + d, 0, T))[:, r], sample(np.clip(tt - d, 0, T))[:, r]
            v = np.abs(qa - qb) / (2 * d) / v_max_by_robot[rid]
            vt = np.linalg.norm(tcp[rid](qa) - tcp[rid](qb), axis=1) / (2 * d)
            rows.append((rid, wid, part, float(v.max()), float(vt.max())))
    if not rows:
        return {"max_speed_frac_vmax": None, "max_tcp_speed_m_s": None, "n": 0, "radius_m": radius,
                "mode": "weld events (window export)", "note": "not measured: no weld zone samples"}
    return {"max_speed_frac_vmax": _r(max(r[3] for r in rows), 4), "max_tcp_speed_m_s": _r(max(r[4] for r in rows), 3),
            "n": len(rows), "radius_m": radius, "mode": "weld events (window export)",
            "worst": [(a, b, c, _r(x, 4)) for a, b, c, x, _ in sorted(rows, key=lambda r: -r[3])[:3]]}


def _own_zone_fn(events, sample, T, robot_ids, tcp, wpos, radius, hz=50.0):
    """own_zone(robot, ts) for transit_near_welds: the arm's own dwell and its contiguous approach / lift-off within
    ``radius`` of that weld (+ 5 mm)."""
    ts_all = np.arange(0.0, T, 1.0 / hz)
    Q = sample(ts_all)
    masks = {rid: np.zeros(len(ts_all), bool) for rid in robot_ids}
    for rid, wid, ta, tl in events:
        if wid not in wpos or rid not in robot_ids:
            continue
        r = list(robot_ids).index(rid)
        d = np.linalg.norm(tcp[rid](Q[:, r]) - wpos[wid], axis=1) <= radius + RT.MASK_PAD
        ka, kd = int(np.clip(np.searchsorted(ts_all, ta), 0, len(ts_all) - 1)), \
            int(np.clip(np.searchsorted(ts_all, tl), 0, len(ts_all) - 1))
        m = masks[rid]
        m[ka:kd + 1] = True
        k = ka
        while k - 1 >= 0 and d[k - 1]:
            k -= 1
            m[k] = True
        k = kd
        while k + 1 < len(ts_all) and d[k + 1]:
            k += 1
            m[k] = True

    def own(rid, ts):
        return masks[rid][np.clip(np.round(np.asarray(ts) * hz).astype(int), 0, len(ts_all) - 1)]
    return own


def leaf_jerk_check(legs, a_max_by_robot: dict, *, h: float = 0.05, hz: float = 200.0) -> dict:
    """Within-leaf jerk of the exported waypoints (0.05 s third difference of the linearly interpolated trajectory, a
    stencil entirely inside one leaf; a_max per s). The leaf boundaries (rest points at cuts) are excluded: that is
    where the executor stops anyway."""
    from player.timeline import sample_leg
    best = (0.0, None, None, None)
    for l in legs:
        if l.synthetic or len(l.times) < 4 or l.t1 - l.t0 <= 3 * h + 2.0 / hz:
            continue
        c = np.arange(l.t0 + 1.5 * h, l.t1 - 1.5 * h + 1e-12, 1.0 / hz)
        f = lambda x: sample_leg(l, x)  # noqa: E731
        j = np.abs(f(c + 1.5 * h) - 3 * f(c + 0.5 * h) + 3 * f(c - 0.5 * h) - f(c - 1.5 * h)) / h ** 3 \
            / a_max_by_robot[l.robot]
        k = int(np.argmax(j.max(axis=1)))
        if j[k].max() > best[0]:
            best = (float(j[k].max()), l.robot, l.op_id, _r(c[k], 3))
    return {"max_amax_per_s": _r(best[0], 3), "at": list(best[1:]), "metric": "0.05 s third difference inside a leaf"}


def stops_near_welds(legs, tcp: dict, wpos: dict, v_max_by_robot: dict | None = None, *, inner: float = 0.05,
                     offset: float = RT.WELD_RADIUS, moved_s: float = 1.0, moved_rad: float = 1e-3,
                     move_frac: float = 0.005, at_weld_m: float = 0.010) -> dict:
    """Leaves that are not a weld arrival but end (the arm braked to rest at a cut) with the TCP near a weld of the
    task: within ``inner`` (5 cm: inside the approach / lift-off stroke) or within ``offset`` (umarell's
    approach_offset 0.10 m). Only arms that moved during the last ``moved_s`` of the leaf count (an arm parked or
    holding there did not brake; with ``v_max_by_robot``: peak joint speed >= ``move_frac`` x v_max there, the
    "moving" threshold of the cut selection, so an arm creeping slower than that is not counted); a leaf ending AT a weld pose (<= ``at_weld_m``, the weld tolerance) whose arm stays
    still into its next leaf is a hold that another arm's cut splits, not a hover."""
    from player.timeline import sample_leg
    W = list(wpos.items())
    rows = []
    nxt = {}
    by = {}
    for l in sorted([x for x in legs if not x.synthetic], key=lambda x: x.t0):
        by.setdefault(l.robot, []).append(l)
    for ls in by.values():
        for a_, b_ in zip(ls[:-1], ls[1:]):
            nxt[id(a_)] = b_
    for l in legs:
        if l.synthetic or l.kind in ("weld", "home") or len(l.times) < 2 or not W:
            continue
        q1 = np.asarray(l.positions[-1], float)
        q0 = sample_leg(l, np.array([max(l.t0, l.t1 - moved_s)]))[0]
        if np.abs(q1 - q0).max() < moved_rad:
            continue
        if v_max_by_robot is not None:
            ts = np.arange(max(l.t0, l.t1 - moved_s), l.t1, 0.005)
            if len(ts) >= 2:
                P = sample_leg(l, ts)
                vf = (np.abs(np.diff(P, axis=0)) / 0.005 / v_max_by_robot[l.robot]).max()
                if vf < move_frac:
                    continue
        p = tcp[l.robot](q1[None])[0]
        dw = [(float(np.linalg.norm(p - x)), w) for w, x in W]
        dmin, wid = min(dw)
        nl = nxt.get(id(l))
        holding = dmin <= at_weld_m and nl is not None and np.abs(
            sample_leg(nl, np.array([min(nl.t1, nl.t0 + 0.3)]))[0] - q1).max() < 1e-5
        if dmin <= offset and not holding:          # at a weld pose and still there after the cut: a split hold
            rows.append((l.robot, l.op_id, wid, _r(dmin * 1000, 1)))
    n_in = sum(r[3] <= inner * 1000 for r in rows)
    return {"n_within_inner": int(n_in), "n_within_offset": len(rows), "inner_m": inner, "offset_m": offset,
            "stops": sorted(rows, key=lambda r: r[3])[:12],
            "rule": "a leaf that is not a weld arrival ends with its (moving) arm braked to rest within inner_m of a "
                    "weld: a stop inside another weld's approach / lift-off"}


def transit_near_welds(legs, sample, T, robot_ids, tcp: dict, wpos: dict, own_zone, *, radii=(0.05, 0.10),
                       hz: float = 50.0) -> dict:
    """Process / near-miss observation (not a limit): peak TCP speed of every arm while its TCP passes within each
    radius of ANY weld point of the task, outside its own weld operations (``own_zone(robot, t) -> bool`` arrays:
    approach / lift-off / dwell of that arm's own welds). The weld-zone rules cover only the welding arm's own
    approach and lift-off; this reports the rest (transits / home moves past the workpiece)."""
    W = np.stack([np.asarray(x, float) for x in wpos.values()]) if wpos else np.zeros((0, 3))
    wids = list(wpos)
    ts = np.arange(0.0, T, 1.0 / hz)
    if not len(W) or len(ts) < 3:
        return {}
    Q = sample(ts)
    out = {}
    for r, rid in enumerate(robot_ids):
        P = tcp[rid](Q[:, r])
        vt = np.linalg.norm(np.gradient(P, ts, axis=0), axis=1)
        dd = np.linalg.norm(P[:, None, :] - W[None], axis=2)
        dmin, wi = dd.min(axis=1), dd.argmin(axis=1)
        mask = ~own_zone(rid, ts)
        for rad in radii:
            m = mask & (dmin <= rad)
            if not m.any():
                continue
            k = int(np.flatnonzero(m)[np.argmax(vt[m])])
            cur = out.get(str(rad))
            if cur is None or vt[k] > cur["max_tcp_speed_m_s"]:
                out[str(rad)] = {"max_tcp_speed_m_s": _r(vt[k], 3), "robot": rid, "weld": wids[wi[k]],
                                 "dist_mm": _r(dmin[k] * 1000, 1), "t_s": _r(ts[k], 2)}
    return {"by_radius_m": out, "scope": "non-weld-operation motion of every arm past ANY weld point (transits, home "
                                         "moves); the weld-zone rules cover only the welding arm's own approach / "
                                         "lift-off", "grid_hz": hz}


def timing_check(intended, legs) -> dict:
    got = {}
    for l in legs:
        got.setdefault((l.robot, l.op_id), []).append(l.t0)
    errs, missing = [], []
    seen = {}
    for l in intended:
        k = (l.robot, l.op_id)
        i = seen.get(k, 0)
        seen[k] = i + 1
        lst = sorted(got.get(k) or [])
        if i >= len(lst):
            missing.append(k)
            continue
        errs.append(abs(sorted([x.t0 for x in intended if (x.robot, x.op_id) == k])[i] - lst[i]))
    return {"max_start_error_s": _r(max(errs) if errs else 0.0, 6), "missing": missing,
            "pass": (not missing) and (max(errs) if errs else 0.0) <= 1e-3}


def snap_legs(legs, quantum: float = TIME_QUANTUM_S, min_seg_s: float = MIN_SEG_S) -> dict:
    """Put every leg's waypoints ON umarell's 0.1 ms time grid (relative to the leg start) and re-sample the leg's
    own piecewise-linear motion there (in place). The plan writer rounds time_s to 0.1 ms but keeps the positions
    of the unrounded instants; on the ~1 ms grid of a full-speed cut that made segment slopes jitter by up to
    (0.05 ms / segment), i.e. several % of v_max (P3U verification). Re-sampling a piecewise-linear curve keeps
    every new segment slope inside the range of the old ones. Interior points closer than ``min_seg_s`` to their
    predecessor are dropped; first / last positions (rest, weld pose) are kept exactly."""
    from player.timeline import sample_leg
    n_moved, dmax = 0, 0.0
    for l in legs:
        if l.synthetic or len(l.times) < 2 or abs(float(l.vscale) - 1.0) > 1e-12:
            continue
        rel = np.asarray(l.times, float) - float(l.t0)
        g = np.round(rel / quantum) * quantum
        keep = [0]
        for i in range(1, len(g) - 1):
            if g[i] - g[keep[-1]] >= min_seg_s - 1e-12 and g[-1] - g[i] >= min_seg_s - 1e-12:
                keep.append(i)
        if g[-1] > g[keep[-1]] + 1e-12:
            keep.append(len(g) - 1)
        elif len(keep) > 1:
            keep[-1] = len(g) - 1
        g = np.round(g[keep] / quantum) * quantum
        tn = float(l.t0) + g
        P = sample_leg(l, tn)
        P[0], P[-1] = l.positions[0], l.positions[-1]
        idx = np.clip(np.searchsorted(l.times, tn, side="left"), 0, len(l.times) - 1)
        ph = np.asarray(l.phase)[idx] if len(l.phase) == len(l.times) else l.phase
        dmax = max(dmax, float(np.abs(rel[keep] - g).max()))
        n_moved += 1
        dt_done = float(l.t_done) - float(l.t1)
        l.times, l.positions, l.phase = tn, P, np.asarray(ph, dtype="<U16")
        l.t1 = float(tn[-1])
        l.t_done = l.t1 + dt_done
    return {"legs": n_moved, "quantum_s": quantum, "min_segment_s": min_seg_s, "max_time_shift_s": _r(dmax, 6)}


def limits_check(legs, lims: dict, dyn: dict, *, tol_v: float = TOL_V, tol_a: float = TOL_A,
                 tol_q: float = TOL_Q) -> dict:
    """Absolute limit check of the exported (replayed) waypoints against the limits the plan records (URDF velocity
    and position limits, the acceleration override / default): peak segment slope |dq|/dt per joint <= v_max (1 +
    tol_v), dense 0.05 s acceleration <= a_max (1 + tol_a), positions inside [lower, upper] (+ tol_q). Fails also
    when the source already broke the limit (the export would command it)."""
    vrows, qx = [], []
    for l in legs:
        if len(l.times) < 2:
            continue
        lim = lims[l.robot]
        dt = np.diff(np.asarray(l.times, float))
        dq = np.abs(np.diff(np.asarray(l.positions, float), axis=0))
        ok = dt > 1e-9
        if ok.any():
            r = dq[ok] / dt[ok][:, None] / lim.v_max
            k, jj = np.unravel_index(int(r.argmax()), r.shape)
            vrows.append((float(r.max()), l.robot, l.op_id, f"a{jj + 1}", _r(float(np.asarray(l.times)[1:][ok][k]), 3)))
        if (dt <= 1e-9).any() and (dq[dt <= 1e-9] > 1e-9).any():
            vrows.append((float("inf"), l.robot, l.op_id, "zero-duration segment with motion", None))
        P = np.asarray(l.positions, float)
        lo = np.maximum(np.asarray(lim.lower) - P, 0.0)
        hi = np.maximum(P - np.asarray(lim.upper), 0.0)
        e = np.maximum(lo, hi)
        k, jj = np.unravel_index(int(e.argmax()), e.shape)
        qx.append((float(e.max()), l.robot, l.op_id, f"a{jj + 1}"))
    v = max(vrows, default=(0.0, None, None, None, None))
    q = max(qx, default=(0.0, None, None, None))
    a = (dyn.get("acc_frac_amax") or {}).get("max") or 0.0
    vd = (dyn.get("vel_frac_vmax") or {}).get("max") or 0.0
    out = {"segment_vel_frac_vmax_max": _r(v[0], 4), "segment_vel_at": list(v[1:]),
           "dense_vel_frac_vmax_max": vd, "dense_acc_frac_amax_max": a,
           "acc_at": (dyn.get("acc_frac_amax") or {}).get("at"),
           "joint_limit_excess_rad": _r(q[0], 6), "joint_limit_at": list(q[1:]),
           "tolerances": {"vel": tol_v, "acc": tol_a, "pos_rad": tol_q},
           "metric": "velocity: exact waypoint segment slopes (umarell / JTC linear interpolation); acceleration: "
                     "0.05 s second difference on a 200 Hz grid; position: waypoints vs URDF <limit lower upper>"}
    out["vel_pass"] = bool(v[0] <= 1.0 + tol_v and vd <= 1.0 + tol_v)
    out["acc_pass"] = bool(a <= 1.0 + tol_a)
    out["pos_pass"] = bool(q[0] <= tol_q)
    out["pass"] = out["vel_pass"] and out["acc_pass"] and out["pos_pass"]
    return out


def source_limits(src, lims: dict, legs0=None) -> dict:
    """Peak velocity / acceleration fraction and joint-limit excess of the SOURCE motion itself (rollout at its own
    clock, plan on its executor clock), so a report can tell an inherited limit breach from one the export made."""
    rids = list(src.robot_ids)
    vm = np.stack([lims[r].v_max for r in rids])
    am = np.stack([lims[r].a_max for r in rids])
    lo = np.stack([np.asarray(lims[r].lower, float) for r in rids])
    hi = np.stack([np.asarray(lims[r].upper, float) for r in rids])
    if src.kind == "rollout" and src.t is not None:
        t, q = np.asarray(src.t, float), np.asarray(src.q, float)
        v = np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None, None]
        ts = np.arange(t[0] + 0.05, t[-1] - 0.05, 0.005)
        f = lambda x: np.stack([np.stack([np.interp(x, t, q[:, r, j]) for j in range(6)], -1)   # noqa: E731
                                for r in range(q.shape[1])], 1)
        a = np.abs(f(ts + 0.05) - 2 * f(ts) + f(ts - 0.05)) / 0.05 ** 2 if len(ts) else np.zeros((1,) + q.shape[1:])
        P = q
    elif legs0 is not None:
        vv, PP = [], []
        for l in legs0:
            if len(l.times) < 2:
                continue
            i = rids.index(l.robot)
            dt = np.maximum(np.diff(l.times), 1e-9)
            vv.append((np.abs(np.diff(l.positions, axis=0)) / dt[:, None] / vm[i]).max())
            PP.append((i, np.asarray(l.positions, float)))
        T = max([l.t_done for l in legs0] + [l.t1 for l in legs0])
        dyn = dense_dynamics(legs0, rids, src.home, vm, am, T)
        exc = max([float(np.maximum(np.maximum(lo[i] - P, 0), np.maximum(P - hi[i], 0)).max()) for i, P in PP],
                  default=0.0)
        return {"segment_vel_frac_vmax_max": _r(max(vv, default=0.0), 4),
                "dense_acc_frac_amax_max": dyn["acc_frac_amax"]["max"], "joint_limit_excess_rad": _r(exc, 6)}
    else:
        return {}
    exc = float(np.maximum(np.maximum(lo[None] - P, 0), np.maximum(P - hi[None], 0)).max())
    return {"segment_vel_frac_vmax_max": _r(float((v / vm[None]).max()), 4),
            "dense_acc_frac_amax_max": _r(float((a / am[None]).max()), 3), "joint_limit_excess_rad": _r(exc, 6)}


def weld_phase_speed(legs, v_max_by_robot: dict) -> dict:
    """Peak joint speed (fraction of v_max) on waypoint segments whose both ends carry phase approach / retract (the
    weld strokes of trajgen / umarell plans)."""
    mx, at = 0.0, None
    for l in legs:
        ph = np.asarray(l.phase).astype(str)
        if len(l.times) < 2 or len(ph) != len(l.times):
            continue
        w = np.isin(ph, ("approach", "retract"))
        m = w[1:] & w[:-1]
        if not m.any():
            continue
        dt = np.maximum(np.diff(l.times), 1e-9)
        r = (np.abs(np.diff(l.positions, axis=0)) / dt[:, None] / v_max_by_robot[l.robot])[m].max()
        if r > mx:
            mx, at = float(r), (l.robot, l.op_id)
    return {"max_speed_frac_vmax": _r(mx, 4), "at": at}


# ───────────────────────────────────────── export ─────────────────────────────────────────
def _limits(src, accel=None):
    lims = L.limits_for(cell_id=src.cell_id if src.scene is None else None, scene=src.scene, accel=accel)
    v, a = L.stack(lims, src.robot_ids)
    return lims, v, a


def _rollout_forbid(src, rids, v_max, a_lim, tcp, wpos, radius):
    """Cut instants to avoid in a rollout's sync cut (P3U round 2): some arm moving inside its own weld operation
    (btexport's weld zone: TCP within ``radius`` of its weld, contiguous with the dwell) or within 5 cm of any weld,
    or braking into / starting from such a stretch (``nsync.forbidden_cuts`` with the cut's acceleration limit)."""
    from tools.btexport import nsync
    t = np.asarray(src.t, float)
    q = np.asarray(src.q, float)
    ev = [(r, w, int(np.searchsorted(t, a - 1e-9)), int(np.searchsorted(t, b - 1e-9))) for r, w, a, b in src.weld_events]
    own = RT.weld_op_mask(q, rids, tcp, ev, wpos, radius)
    inner, moving, _ = nsync.zone_masks(t, q, rids, tcp, wpos, v_max)
    return nsync.forbidden_cuts(t, q, (own & moving) | inner, moving, a_lim)


def _retimed_sync_cut(t, q, ev, rinfo, rids, v_max, a_max, tcp, wpos, src, *, acc_frac):
    """Per-weld (sync) cut of tools.retime's motion, P3U round 2: no cut where any arm moves inside its weld operation
    (tools.retime's approach / lift-off labels, or the TCP within 5 cm of a weld) or would have to brake into / start
    from one; inside weld zones the clock's acceleration obeys tools.retime's weld rule (weld_a x a_max); the
    within-leaf jerk of the braking is held to tools.retime's jerk cap. -> (legs, info, cut info)."""
    from tools.btexport import nsync
    from trajgen.ntimeline import NTimeline
    rep = rinfo.get("report") or {}
    prof = _g_(rep, "cfg", "resolved_profile") or {}
    jcap = prof.get("j_frac")
    weld_a = prof.get("weld_a")
    labels_t = labels = None
    try:
        z = np.load(rinfo["out"], allow_pickle=False)
        if "labels" in z.files:
            tl = NTimeline.load(rinfo["out"])
            order = [list(tl.robot_ids).index(r) for r in rids]
            labels_t, labels = np.asarray(tl.t, float), np.asarray(z["labels"])[:, order]
    except Exception:  # noqa: BLE001  (labels are optional: the 5 cm rule still applies)
        pass
    zone, moving, _ = nsync.zone_masks(t, q, rids, tcp, wpos, v_max, labels_t=labels_t, labels=labels)
    forbid = nsync.forbidden_cuts(t, q, zone, moving, a_max, jerk_cap=jcap)
    dt = float(np.median(np.diff(t)))
    sub = max(4, int(np.ceil(dt / 0.001 - 1e-9)))
    legs, info = nsync.legs_sync_rest_n(t, q, ev, rids, v_max, a_max, home_last=bool(
        np.abs(q[-1] - np.asarray(src.home)).max() < 1e-3), acc_frac=acc_frac, acc_mode="envelope", sub=sub,
        zone=zone, zone_afrac=weld_a, forbid=forbid, jerk_cap=jcap)
    cut = {k: info.get(k) for k in ("cut_in_weld_zone", "forbidden_instant_frac", "zone_afrac", "jerk_cap_amax_per_s",
                                    "leaf_jerk_max_amax_per_s", "jerk_iterations", "jerk_history", "softened_frac",
                                    "soft_min", "n_cuts", "warped_motion_s", "rollout_motion_s")}
    cut["labels"] = "tools.retime approach / lift-off labels + TCP within 5 cm of a weld" if labels is not None else \
        "TCP within 5 cm of a weld (no tools.retime labels in the npz)"
    return legs, info, cut


def _robot_model(src) -> dict:
    try:
        return L.robot_model(src)
    except Exception as e:  # noqa: BLE001  (informational; the limits record still names the URDF)
        return {"error": repr(e)}


def _geometry(src):
    if src.scene is not None:
        return RT.tcp_functions(scene=src.scene), RT.weld_positions(task=src.task)
    return RT.tcp_functions(cell_id=src.cell_id), RT.weld_positions(cell_id=src.cell_id)


def source_timeline(src):
    """The source's own executor-clock timeline, if it has one: an executor-clock timeline source (round 2) or a
    trajgen run's timeline.npz. -> (NTimeline, label) or (None, reason)."""
    from trajgen.ntimeline import NTimeline
    if src.meta.get("source_timing") and src.meta.get("timeline"):
        p = Path(src.meta["timeline"])
        p = p if p.is_absolute() else ROOT_DIR / p
        if p.exists():
            return NTimeline.load(p), "executor-clock timeline (the source)"
        return None, f"source timeline {p} not found"
    if src.kind == "plan" and src.run_dir is not None and (Path(src.run_dir) / "timeline.npz").exists():
        return NTimeline.load(Path(src.run_dir) / "timeline.npz"), "trajgen timeline.npz (executor clock)"
    return None, "the source has no executor-clock timeline"


def bt_fidelity(src, tl_bt, robot_ids, *, retimed: bool = False) -> dict:
    """``bt_fidelity_rad`` (docs/n2_contract.md 3.2.3 / 8.3): max |q| difference between the exported plan's executor
    replay ``tl_bt`` and the source's executor-clock timeline, over the union of both grids and every leg sample time.
    None (with the reason) for re-timed exports (another clock by design) or sources without such a timeline."""
    if retimed:
        return {"bt_fidelity_rad": None, "reason": "re-timed export (the plan runs on a new clock by design)"}
    try:
        tl, label = source_timeline(src)
    except Exception as e:  # noqa: BLE001
        return {"bt_fidelity_rad": None, "reason": f"source timeline unreadable: {e!r}"}
    if tl is None:
        return {"bt_fidelity_rad": None, "reason": label}
    rids = list(robot_ids)
    if set(tl.robot_ids) != set(rids):
        return {"bt_fidelity_rad": None, "reason": f"timeline robots {list(tl.robot_ids)} != plan {rids}"}
    col = [list(tl.robot_ids).index(r) for r in rids]
    T = max(float(tl.t[-1]), float(tl_bt.t[-1]), max([l.t1 for l in tl.legs] + [0.0]),
            max([l.t1 for l in tl_bt.legs] + [0.0]))
    parts = [np.asarray(tl.t, float), np.asarray(tl_bt.t, float), np.array([T])]
    parts += [np.asarray(l.times, float) for l in tl.legs] + [np.asarray(l.times, float) for l in tl_bt.legs]
    ts = np.unique(np.clip(np.concatenate(parts), 0.0, T))
    d = np.abs(tl_bt.sample(ts) - tl.sample(ts)[:, col])
    k = np.unravel_index(int(np.argmax(d)), d.shape) if d.size else (0, 0, 0)
    end_bt = max([float(getattr(l, "t_done", l.t1)) for l in tl_bt.legs] + [0.0])
    end_src = max([float(getattr(l, "t_done", l.t1)) for l in tl.legs] + [0.0])
    return {"bt_fidelity_rad": round(float(d.max()) if d.size else 0.0, 9), "against": label,
            "n_samples": int(len(ts)), "t_max_s": round(float(ts[k[0]]), 4) if d.size else None,
            "robot": rids[k[1]] if d.size else None, "executor_end_bt_s": round(end_bt, 4),
            "executor_end_source_s": round(end_src, 4)}


def unique_work_dir(tree_id: str) -> Path:
    """data/btexport/work/retime_<tree id>_<pid>_<random hex>, created (one per export; round 2)."""
    import os
    import re
    import uuid
    safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(tree_id))[:80] or "export"
    d = ROOT_DIR / "data" / "btexport" / "work" / f"retime_{safe}_{os.getpid()}_{uuid.uuid4().hex[:8]}"
    d.mkdir(parents=True, exist_ok=False)
    return d


def export(src, *, profile: str = "umarell", tree_id: Optional[str] = None, out=None, xml=None,
           acc_frac_umarell: float = 0.49, acc_frac_full: float = 1.0, weld_vfrac: float = RT.WELD_VFRAC,
           weld_radius: float = RT.WELD_RADIUS, full_backend: str = "warp", work=None, backend: Optional[str] = None,
           retime_cut: str = "sync", retime_mode: str = "auto", retime_timeout: int = 7200,
           accel=None, retime_hz: float = 100.0, weld_source_cap: bool = False,
           weld_afrac: Optional[float] = None, limits_gate: bool = True,
           allow_partial: bool = False, compare_source: bool = True, keep_work: bool = False,
           vass: str = "off", vass_nodes: bool = False, vass_required: bool = False,
           vass_cfg: Optional[dict] = None) -> tuple[dict, dict]:
    """Export ``src`` -> (plan envelope, report); the arguments of ``_export_impl`` (its docstring has the rules).

    ``vass`` (round 3, docs/r3_vass.md §4): "off" (default: the plan exactly as before) | "plan" | "fifo" |
    "priority": after the export, ``vass.integrate.annotate_plan`` derives the VASS interlock zone table of the
    exported plan on the source's geometry (trajgen scene, or scenes/<cell> for umarell cells) and annotates every
    leg with the ``interlocks`` port + ``metadata.vass`` (+ RequestInterlock / ReleaseInterlock nodes with
    ``vass_nodes``); the motion and its timing are unchanged, today's executors ignore the port. The report gets
    ``vass`` (zones, derivation time, the interlocked nominal run); a derivation error fails ``export_pass`` and the
    plan is written without interlocks. The table is also written next to ``out`` (<stem>.zones.json).

    ``work``: the retime backend's work dir (tools.retime's input, output and cache). Default (round 2,
    docs/n2_traj2.md): a unique dir data/btexport/work/retime_<tree id>_<pid>_<random> per export, removed afterwards
    unless ``keep_work`` (round 1 used data/btexport/work/retime_<tree id>, and the CLI's default tree id is the plan
    file stem, so parallel ``plan.json`` exports on one host shared it and swapped or corrupted each other's retimed
    files). An explicit ``work`` is used as given and kept (its retimed.npz cache then applies)."""
    tid = tree_id or f"btx_{src.cell_id}_{int(time.time())}"
    be = backend if backend is not None else ("retime" if full_backend == "retime" else "native")
    auto = None
    if work is None and be == "retime":
        auto = work = unique_work_dir(tid)
    if vass not in ("off", "plan", "fifo", "priority"):
        raise ValueError(f"vass {vass!r}")
    w_out, w_xml = (None, None) if vass != "off" else (out, xml)
    try:
        plan, rep = _export_impl(src, profile=profile, tree_id=tid, out=w_out, xml=w_xml, acc_frac_umarell=acc_frac_umarell,
                                 acc_frac_full=acc_frac_full, weld_vfrac=weld_vfrac, weld_radius=weld_radius,
                                 full_backend=full_backend, work=work, backend=backend, retime_cut=retime_cut,
                                 retime_mode=retime_mode, retime_timeout=retime_timeout, accel=accel,
                                 retime_hz=retime_hz, weld_source_cap=weld_source_cap, weld_afrac=weld_afrac,
                                 limits_gate=limits_gate, allow_partial=allow_partial, compare_source=compare_source)
    finally:
        if auto is not None and not keep_work:
            import shutil
            shutil.rmtree(auto, ignore_errors=True)
    if auto is not None:
        rep.setdefault("profile_detail", {})["work_dir"] = {"path": str(auto), "unique": True,
                                                            "removed": not keep_work}
        if not keep_work and isinstance(rep.get("source"), dict) and rep["source"].get("retimed_timeline"):
            rep["source"]["retimed_timeline_removed"] = True
    if vass != "off":
        plan, rep = _vass_annotate(src, plan, rep, vass, out=out, xml=xml, nodes=vass_nodes, required=vass_required,
                                   cfg=vass_cfg)
    return plan, rep


def _vass_failed_marker(plan: dict, mode: str, info: dict) -> dict:
    """Round 4 (verification): a plan whose interlock derivation failed is written without interlocks, but no longer
    looks deployable: ``metadata.vass`` = {"status": "derivation_failed", "interlocked": false, "deployable": false,
    code, mode} (no ``table``: ``vass.zones.table_of`` gives None, and ``plan_sha256`` / ``strip_vass`` ignore it as
    every VASS addition). A multi-robot plan without a table may only run clock-timed, where a lag can collide
    (DECISIONS #6)."""
    import copy
    p = copy.copy(plan)
    if isinstance(p.get("behaviour_tree"), dict):
        t = p["behaviour_tree"] = dict(p["behaviour_tree"])
    else:
        t = p
    md = t["metadata"] = dict(t.get("metadata") or {})
    md["vass"] = {"status": "derivation_failed", "mode": mode, "code": info.get("code"), "interlocked": False,
                  "deployable": False,
                  "note": "no interlock zone table: not for interlocked deployment (a clock-timed run may collide "
                          "under lag)", "error": str(info.get("error") or info.get("problems") or "")[:300]}
    return p


def _vass_annotate(src, plan, rep, mode, *, out=None, xml=None, nodes=False, required=False, cfg=None):
    """VASS interlocks on an exported plan (round 3): derive + annotate, then write ``out`` / ``xml`` as
    ``_export_impl`` would have."""
    from vass.integrate import annotate_plan
    zp = Path(out).with_name(Path(out).stem + ".zones.json") if out is not None else None
    t0 = time.time()
    try:
        pv, info = annotate_plan(plan, scene=src.scene, cell_id=None if src.scene is not None else src.cell_id,
                                 robot_ids=list(src.robot_ids), home=src.home, mode=mode, nodes=nodes,
                                 required=required, cfg=cfg, zones_out=zp)
    except (FileNotFoundError, ValueError) as e:
        pv, info = plan, {"mode": mode, "ok": False, "code": "config", "error": repr(e)[:500]}
    info["wall_s"] = _r(time.time() - t0, 2)
    rep["vass"] = info
    if info.get("ok"):
        plan = pv
    else:
        rep["export_pass"] = False
        rep.setdefault("warnings", []).append(f"vass {mode}: derivation failed ({info.get('code')}): "
                                              f"{str(info.get('error') or info.get('problems'))[:300]}; "
                                              "plan written without interlocks")
        plan = _vass_failed_marker(plan, mode, info)
    if out is not None:
        o = Path(out)
        o.parent.mkdir(parents=True, exist_ok=True)
        try:
            txt = json.dumps(plan, indent=1, allow_nan=False)
        except ValueError as e:
            raise Unsupported(f"the export contains non-finite numbers ({e}); not written") from None
        o.write_text(txt)
        rep["plan"] = str(o)
    if xml is not None:
        x = Path(xml)
        x.parent.mkdir(parents=True, exist_ok=True)
        x.write_text(plan["behaviour_tree"]["xml"])
        rep["xml"] = str(x)
    return plan, rep


def _export_impl(src, *, profile: str = "umarell", tree_id: Optional[str] = None, out=None, xml=None,
           acc_frac_umarell: float = 0.49, acc_frac_full: float = 1.0, weld_vfrac: float = RT.WELD_VFRAC,
           weld_radius: float = RT.WELD_RADIUS, full_backend: str = "warp", work=None, backend: Optional[str] = None,
           retime_cut: str = "sync", retime_mode: str = "auto", retime_timeout: int = 7200,
           accel=None, retime_hz: float = 100.0, weld_source_cap: bool = False,
           weld_afrac: Optional[float] = None, limits_gate: bool = True,
           allow_partial: bool = False, compare_source: bool = True) -> tuple[dict, dict]:
    """Export ``src`` (tools.btexport.sources.Source) -> (plan envelope, report). Writes ``out`` (JSON) and ``xml``.

    ``backend``: "native" (umarell profile: the source's timing; full profile: this package's common-clock warp) or
    "retime" (track SPEED's ``python -m tools.retime --profile <profile>`` first: per-arm jerk-limited TOPP, weld
    operations at umarell's weld speed, synchronisation waits, its own 400 Hz re-check; any number of robots).
    Default: "retime" if ``full_backend == "retime"`` else "native" (``full_backend`` is the version-1 name).
    A gated N-robot plan (trajgen priority) in the full profile has no native path (Unsupported): use "retime".
    ``retime_cut``: how the retimed motion becomes a tree -- "sync" (per-weld leaves, all arms at rest at every
    executor pause, tools.btexport.nsync for any number of robots: no cut where an arm moves inside, or would brake
    into, a weld operation; the braking keeps the weld acceleration rule in weld zones and tools.retime's jerk cap;
    if some weld's dwell window offers no such instant the export falls back to "windows" with a warning,
    ``profile_detail.sync_fallback``; the cut adds a 1.5 s executor pause per weld) or "windows" (tools.retime.bt:
    cut only where all arms rest, the retimed clock exactly, a leaf may hold several welds -> umarell's per-weld
    validate_approach does not apply).
    ``accel``: explicit acceleration limits for the checks ([6] or {robot: [6]}); for a run whose controllers and
    retimer should use them too, set a robot override instead (robotmodel.override, CLI --robot-accel).
    Full-profile warp, weld operations: joint speed <= ``weld_vfrac`` x v_max (0.28); optional ``weld_afrac``
    (acceleration <= weld_afrac x a_max there, e.g. umarell's weld rule 0.078; default None = full acceleration) and
    ``weld_source_cap`` (never faster than the source inside a weld operation). Defaults keep the version-1 warp;
    the report gives the weld-operation speed of the export AND of the source (``checks.weld_op_speed_vs_source``),
    because the warp may run a weld stroke faster than the source did (up to the 0.28 v_max cap). ``limits_gate`` (default True): ``export_pass`` requires
    the absolute URDF limit check (``limits_check``). ``allow_partial``: a source that did not complete every weld
    of its task may pass (else ``export_pass`` is False and the missing welds are listed)."""
    from robotmodel.override import active_override
    if src.scene is not None and active_override() is not None:
        raise Unsupported("a robot override (--urdf / --robot-accel / --robot-override / KWR_ROBOT_OVERRIDE) is "
                          "not applied to trajgen scene sources (the scene's robots[i].urdf / joints define the "
                          "robots, and the plan was made with them): point the scene at the other URDF and re-plan "
                          "with trajgen instead")
    if not (0.0 < float(acc_frac_full) <= 1.0):
        raise ValueError(f"acc_frac_full must be in (0, 1], got {acc_frac_full}")
    if not (0.0 < float(weld_vfrac) <= 1.0):
        raise ValueError(f"weld_vfrac must be in (0, 1], got {weld_vfrac}")
    if weld_afrac is not None and not (0.0 < float(weld_afrac) <= 1.0):
        raise ValueError(f"weld_afrac must be in (0, 1] or None, got {weld_afrac}")
    if not (float(weld_radius) > 0.0):
        raise ValueError(f"weld_radius must be > 0, got {weld_radius}")
    import execute as X
    from player.bt_plan import to_xml
    from trajgen.btplan import plan_from_steps

    if profile not in ("umarell", "full"):
        raise ValueError(f"profile {profile!r}")
    t_wall = time.time()
    from tools.btexport.sources import check_source
    check_source(src)                      # malformed input -> Unsupported (CLI exit 2), never a NaN plan
    lims, v_max, a_max = _limits(src, accel)
    if not (np.all(np.isfinite(a_max)) and np.all(a_max > 0) and np.all(np.isfinite(v_max)) and np.all(v_max > 0)):
        raise ValueError(f"limits must be finite and > 0: v_max {v_max.tolist()}, a_max {a_max.tolist()}")
    tcp, wpos = _geometry(src)
    rids = list(src.robot_ids)
    vmax_by = {r: v_max[i] for i, r in enumerate(rids)}
    rep = {"exporter": EXPORTER, "version": EXPORTER_VERSION, "source": {"spec": src.spec, "kind": src.kind, **src.meta},
           "profile": profile, "cell_id": src.cell_id, "robots": rids}
    tid = tree_id or f"btx_{src.cell_id}_{int(time.time())}"
    pinfo = {"profile": profile}
    intended = None
    legs0 = None
    src0 = src

    if backend is None:
        backend = "retime" if full_backend == "retime" else "native"
    if backend not in ("native", "retime"):
        raise ValueError(f"backend {backend!r}")
    if retime_cut not in ("sync", "windows"):
        raise ValueError(f"retime_cut {retime_cut!r}")
    pinfo["backend"] = backend
    retimed = False
    kept_tree = False                      # the source's tree is exported unchanged (bt_fidelity_rad applies)
    pre_sync = None
    if backend == "retime":
        import dataclasses
        wk = Path(work or (ROOT_DIR / "data" / "btexport" / "work" / f"retime_{tid}"))
        t2, q2, ev2, rinfo = RT.via_tools_retime(src, wk, profile=profile, mode=retime_mode, timeout=retime_timeout)
        pinfo.update(retime_backend="tools.retime (track SPEED)", tools_retime=rinfo, retime_cut=retime_cut)
        rep["source"]["retimed_by"] = "tools.retime"
        rep["source"]["retimed_timeline"] = rinfo.get("out")
        if not (rinfo.get("report") or {}).get("safety", {}).get("valid", False):
            rep.setdefault("warnings", []).append("tools.retime's own dense re-check did not pass (see "
                                                  "profile_detail.tools_retime.report.safety)")
        if not ev2:
            raise Unsupported("retimed motion has no weld events (nothing to export)")
        if retime_cut == "sync":
            if retime_hz:        # uniform grid for the cut (tools.retime writes 250 Hz; smooth C2 motion, so linear
                # interpolation at 100 Hz is within ~0.1 mrad and keeps the plans' waypoint count manageable)
                tu = np.linspace(t2[0], t2[-1], int(np.ceil((t2[-1] - t2[0]) * retime_hz - 1e-9)) + 1)
                q2u = np.stack([np.stack([np.interp(tu, t2, q2[:, r, j]) for j in range(6)], -1)
                                for r in range(q2.shape[1])], 1)
                pinfo["retime_resample_hz"] = retime_hz
            else:
                tu, q2u = t2, q2
            pre_sync = _retimed_sync_cut(tu, q2u, ev2, rinfo, rids, v_max, a_max, tcp, wpos, src,
                                         acc_frac=acc_frac_umarell if profile == "umarell" else acc_frac_full)
            pinfo["sync_cut"] = pre_sync[2]
            if pre_sync[2].get("cut_in_weld_zone"):
                # a weld's dwell window lies entirely where another arm moves inside (or brakes into) a weld
                # operation: a per-weld cut there would stop that arm mid-approach / mid-lift-off -> window export
                pinfo["sync_fallback"] = "windows"
                rep.setdefault("warnings", []).append(
                    f"sync cut refused: the dwell windows of {pre_sync[2]['cut_in_weld_zone']} overlap another arm's "
                    "weld operation (or its braking distance); exported with the window cut (tools.retime.bt: the "
                    "retimed clock exactly, a leaf may hold several welds)")
                retime_cut = "windows"
                pre_sync = None
            else:
                t2, q2 = tu, q2u
        if retime_cut == "windows":
            from tools.retime.bt import export_bt
            from trajgen.ntimeline import NTimeline
            tl = NTimeline.load(rinfo["out"])
            bt_path = wk / "retimed_windows_plan.json"
            binfo = export_bt(tl, src.cell_id, bt_path, tree_id=tid, meta=dict(profile=profile))
            pinfo["windows_export"] = binfo
            src = dataclasses.replace(src, kind="plan", plan=json.load(open(bt_path)),
                                      meta=dict(src.meta, generator="tools.retime (windows export)"))
        else:
            src = dataclasses.replace(src, kind="rollout", t=t2, q=q2, weld_events=ev2,
                                      at_home=bool(np.abs(q2[-1] - np.asarray(src.home)).max() < 1e-3))
        retimed = True
    if src.kind == "rollout":
        if len(rids) != 2 and not retimed and (src.scene is None or bool(src.umarell_cell)):
            raise Unsupported("rollouts in a umarell cell are two-arm (env) motions")
        if not src.weld_events:
            raise Unsupported("rollout has no weld events (nothing to export)")
        dt_src = float(np.median(np.diff(src.t)))
        dense = dt_src < 0.049
        kw = dict(dt=dt_src, sub=4) if dense else {}
        if retimed:
            # a retimed input runs at full speed into the cuts: a fine grid of <= 1 ms keeps the braking to rest
            # resolved (the last fine step before a rest moves sqrt(2 A dq); 2.5 ms steps left 2-3 % v_max over the
            # last 5 ms of a leg, the export's rest threshold is 3 %)
            kw = dict(dt=dt_src, sub=max(4, int(np.ceil(dt_src / 0.001 - 1e-9))))
        # full warp: 12 fine points per 20 Hz sample (sync_rest keeps every 4th where the clock differs from the
        # source: 3 waypoints per sample; same peaks and makespan as sub=40 within 0.1 s / 0.005 a_max)
        kw_full = kw or {"sub": 12}
        cell_path = src.scene is None or bool(src.umarell_cell)       # robot1/robot2 of a umarell cell
        if pre_sync is not None:
            legs, info, _ = pre_sync
            pinfo.update(method=f"tools.retime {profile} profile, then tools.btexport.nsync (R-arm sync_rest: cuts "
                                "outside every arm's weld operations, weld acceleration rule and jerk cap kept in "
                                "the braking, never faster than its input)",
                         acc_frac=acc_frac_umarell if profile == "umarell" else acc_frac_full, acc_mode="envelope")
        elif (profile == "umarell" or retimed) and not cell_path:
            from tools.btexport import nsync
            legs, info = nsync.legs_sync_rest_n(src.t, src.q, src.weld_events, rids, v_max, a_max,
                                                home_last=src.at_home,
                                                acc_frac=acc_frac_umarell if profile == "umarell" else acc_frac_full,
                                                acc_mode="envelope", sub=kw.get("sub", 4),
                                                forbid=_rollout_forbid(src, rids, v_max, a_max * acc_frac_umarell,
                                                                       tcp, wpos, weld_radius))
            pinfo.update(method=(f"tools.retime {profile} profile, then " if retimed else "") +
                         "tools.btexport.nsync (R-arm sync_rest, common-clock warp, never faster than its input)",
                         acc_frac=acc_frac_umarell if profile == "umarell" else acc_frac_full, acc_mode="envelope")
        elif profile == "umarell" or retimed:
            if len(rids) != 2:
                raise Unsupported("rollouts in a umarell cell are two-arm (env) motions")
            legs, info = X.legs_sync_rest(src.cell_id, src.t, src.q, src.weld_events, home_last=src.at_home,
                                          acc_frac=acc_frac_umarell if profile == "umarell" else acc_frac_full,
                                          acc_mode="envelope",
                                          forbid=_rollout_forbid(src, rids, v_max, a_max * (
                                              acc_frac_umarell if profile == "umarell" else acc_frac_full),
                                              tcp, wpos, weld_radius), **kw)
            pinfo.update(method=(f"tools.retime {profile} profile, then " if retimed else "") +
                         "execute.legs_sync_rest (sync_rest, common-clock warp, never faster than the source)",
                         acc_frac=acc_frac_umarell if profile == "umarell" else acc_frac_full, acc_mode="envelope")
        elif not cell_path:
            raise Unsupported("native full warp of a scene rollout: use backend retime")
        else:
            winfo = {}
            warp = RT.make_rollout_warp(src.t, src.q, rids, v_max, tcp, src.weld_events, wpos, radius=weld_radius,
                                        weld_vfrac=weld_vfrac, info=winfo, weld_source_cap=weld_source_cap,
                                        weld_afrac=weld_afrac)
            legs, info = X.legs_sync_rest(src.cell_id, src.t, src.q, src.weld_events, home_last=src.at_home,
                                          acc_frac=acc_frac_full, acc_mode="strict", warp_fn=warp,
                                          forbid=_rollout_forbid(src, rids, v_max, a_max * acc_frac_full, tcp, wpos,
                                                                 weld_radius), **kw_full)
            pinfo.update(method="execute.legs_sync_rest + full-speed common-clock warp (tools.btexport.retime)",
                         acc_frac=acc_frac_full, acc_mode="strict", **winfo)
        pinfo["sync_rest"] = {k: info.get(k) for k in ("n_cuts", "n_welds", "uncut", "warp_scale", "warp_iterations",
                                                       "warped_motion_s", "rollout_motion_s", "slowdown_s",
                                                       "waypoints_per_arm", "cut_in_weld_zone")}
        if info.get("cut_in_weld_zone") and pre_sync is None:
            rep.setdefault("warnings", []).append(
                f"the dwell windows of {info['cut_in_weld_zone']} offer no instant with every other arm outside its "
                "weld operations: those cuts stop an arm inside (or braking into) one (checks.stops_near_welds)")
        if info.get("uncut"):
            rep.setdefault("warnings", []).append(f"welds whose dwell window touches the rollout start/end (no leg "
                                                  f"ends there): {info['uncut']}")
        pinfo["time_grid"] = snap_legs(legs)
        steps = group_steps(legs, rids)
        intended = [l for st in steps for l in st]
        md = {"generator": "controller rollout", "export_mode": "sync_rest", "sync":
              "every step is a Parallel of all arms on one clock; every executor pause with all arms at rest"}
        plan = plan_from_steps(src.cell_id, steps, rids, tree_id=tid, created_s=time.time(), metadata_extra=md)
    else:
        orig = copy.deepcopy(src.plan)
        tree0 = orig["behaviour_tree"]
        shape = plan_shape(tree0)
        legs0, m0 = replay(orig, rids, src.home, executor_of(src))
        rep["source"]["plan_shape"] = shape
        rep["source"]["source_timing"] = {k: m0.get(k) for k in ("makespan_motion_s", "executor_end_s")}
        speed_done = src.meta.get("trajgen_full_speed") == "full" or src.meta.get("trajgen_speed") == "full"
        src_timing = bool(src.meta.get("source_timing"))       # executor-clock timeline (round 2): legs 1:1
        if profile == "umarell" or speed_done or retimed or src_timing:
            kept_tree = not retimed
            plan = orig
            plan["tree_id"] = tid
            plan["cell_id"] = src.cell_id
            tree = plan["behaviour_tree"]
            tree["tree_id"], tree["cell_id"] = tid, src.cell_id
            pinfo.update(method=("tools.retime %s profile, then tools.retime.bt all-at-rest windows" % profile
                                 if retimed else "source tree kept (legs, gates, holds unchanged)") +
                         ("; trajgen --speed full" if speed_done else "") +
                         ("; executor-clock timeline exported 1:1 with source timing" if src_timing and not retimed
                          else ""))
            md = {"export_mode": "tree"}
            md.setdefault("generator", src.meta.get("generator"))
            tree.setdefault("metadata", {})
            for k, v in md.items():
                tree["metadata"].setdefault(k, v)
            intended = legs0
        else:
            if shape != "steps":
                raise Unsupported("full profile of a gated (per-robot sequence) plan: retiming legs would break its "
                                  "WaitForRobot/Delay gates; use backend 'retime' (--backend retime: tools.retime, "
                                  "then a rest-synchronous per-weld cut) or a trajgen full-speed mode")
            if any(l.synthetic for l in legs0):
                raise Unsupported("plan has executor-generated moves (interlock / blind MoveHome)")
            steps0 = steps_from_tree(tree0, legs0)
            target = a_max * acc_frac_full

            def dyn(tau, qf):
                Q = qf.reshape(len(tau), -1)
                ts = np.arange(tau[0], tau[-1] + 1e-9, 0.005)
                f = lambda x: np.stack([np.interp(np.clip(x, tau[0], tau[-1]), tau, Q[:, k]) for k in range(Q.shape[1])], -1)
                acc = np.abs(f(ts + 0.05) - 2 * f(ts) + f(ts - 0.05)) / 0.05 ** 2
                return float((acc / target.reshape(-1)).max())
            steps, rinfo = RT.retime_steps(steps0, rids, src.home, v_max, a_max, tcp, wpos, margin=MARGIN_S,
                                           acc_frac=acc_frac_full, radius=weld_radius, weld_vfrac=weld_vfrac, dyn_check=dyn,
                                           weld_source_cap=weld_source_cap, weld_afrac=weld_afrac)
            pinfo["time_grid"] = snap_legs([l for st in steps for l in st])
            pinfo.update(method="per-step common-clock full-speed retime (tools.btexport.retime.retime_steps)",
                         **{k: v for k, v in rinfo.items() if k != "steps"},
                         steps_summary={"n": len(rinfo["steps"]),
                                        "src_motion_s": _r(sum(r["src_s"] for r in rinfo["steps"]), 3),
                                        "new_motion_s": _r(sum(r["new_s"] for r in rinfo["steps"]), 3)})
            intended = [l for st in steps for l in st]
            md = dict(tree0.get("metadata") or {})
            md.update(export_mode="steps_retimed", generator=src.meta.get("generator"))
            for k in ("steps", "operations", "verified", "speed"):
                md.pop(k, None)
            plan = plan_from_steps(src.cell_id, steps, rids, tree_id=tid, created_s=time.time(), metadata_extra=md)

    tree = plan["behaviour_tree"]
    pinfo["normalized"] = normalize_tree(tree)
    if profile == "full" and src.kind == "plan" and "speed" in (tree.get("metadata") or {}) and \
            tree["metadata"].get("speed") != "full":
        tree["metadata"]["source_speed"] = tree["metadata"].pop("speed")
    # ── checks on the exported plan itself ──
    legs, meta = replay(plan, rids, src.home, executor_of(src))
    T = max([l.t_done for l in legs] + [l.t1 for l in legs])
    win_md = (tree.get("metadata") or {}).get("leaves") is not None and \
        str((tree.get("metadata") or {}).get("generator", "")).startswith("tools.retime")
    if win_md:
        from trajgen.ntimeline import NTimeline
        _tl = NTimeline.from_legs(legs, robot_ids=tuple(rids), dt=0.05, home=np.asarray(src.home, float))
        welds_chk = weld_event_check(weld_events_of(tree, legs), _tl.sample, rids, tcp, wpos)
    else:
        welds_chk = weld_end_check(legs, tcp, wpos)
    from trajgen.ntimeline import NTimeline as _NT
    _tlx = _NT.from_legs(legs, robot_ids=tuple(rids), dt=0.05, home=np.asarray(src.home, float))
    fid = bt_fidelity(src0, _tlx, rids, retimed=not kept_tree)
    _evs = weld_events_of(tree, legs)
    chk = {"timing": timing_check(intended, legs) if intended is not None else None,
           "rest": rest_check(legs, vmax_by),
           "welds": welds_chk,
           "weld_op_speed": (weld_zone_speed_events(_evs, _tlx.sample, T, rids, tcp, wpos, vmax_by, weld_radius)
                             if win_md else weld_op_speed_check(legs, tcp, wpos, vmax_by, weld_radius)),
           "dynamics": dense_dynamics(legs, rids, src.home, v_max, a_max, T)}
    amax_by = {r: a_max[i] for i, r in enumerate(rids)}
    chk["leaf_jerk"] = leaf_jerk_check(legs, amax_by)
    chk["stops_near_welds"] = stops_near_welds(legs, tcp, wpos, vmax_by)
    try:
        chk["transit_near_welds"] = transit_near_welds(legs, _tlx.sample, T, rids, tcp, wpos,
                                                       _own_zone_fn(_evs, _tlx.sample, T, rids, tcp, wpos, weld_radius))
    except Exception as e:  # noqa: BLE001  (informational)
        chk["transit_near_welds"] = {"error": repr(e)}
    if backend == "retime" and retimed:
        # tools.retime's motion law is jerk-limited and its weld operations are never interrupted: the export must
        # keep both (P3U round 2: the sync cut's braking doubled the jerk and stopped arms mid-approach)
        jc = _g_(pinfo, "tools_retime", "report", "cfg", "resolved_profile", "j_frac")
        sync_cut = (pinfo.get("retime_cut") == "sync" and pinfo.get("sync_fallback") is None)
        chk["retime_integrity"] = {
            "jerk_cap_amax_per_s": jc, "leaf_jerk_max": chk["leaf_jerk"]["max_amax_per_s"],
            "jerk_pass": jc is None or chk["leaf_jerk"]["max_amax_per_s"] <= float(jc) * (1 + TOL_JERK),
            "stops_in_weld_zone": chk["stops_near_welds"]["n_within_inner"] if sync_cut else None,
            "stops_pass": (not sync_cut) or chk["stops_near_welds"]["n_within_inner"] == 0,
            "cut": "windows (fallback)" if pinfo.get("sync_fallback") else pinfo.get("retime_cut"),
            "retimed_fd_a_ratio_max": _g_(pinfo, "tools_retime", "report", "dynamics", "all", "fd_a_ratio_max"),
            "retimed_qd_vs_fd_ratio_max": _g_(pinfo, "tools_retime", "report", "dynamics", "all", "qd_vs_fd_ratio_max")}
        fa = chk["retime_integrity"]["retimed_fd_a_ratio_max"]
        chk["retime_integrity"]["consistency_pass"] = fa is None or float(fa) <= 1.0 + TOL_A
        chk["retime_integrity"]["pass"] = bool(chk["retime_integrity"]["jerk_pass"] and
                                               chk["retime_integrity"]["stops_pass"] and
                                               chk["retime_integrity"]["consistency_pass"])
    ops = [str((lf.get("ports") or {}).get("operation_id")) for lf in _leaves(tree["root"])
           if _kind(lf) == "ExecuteTrajectory"]
    chk["unique_operation_ids"] = len(ops) == len(set(ops))
    # ── absolute URDF limits (velocity, acceleration, position), whatever the source did ──
    chk["limits"] = limits_check(legs, lims, chk["dynamics"])
    try:
        sl = source_limits(src0, lims, replay(src0.plan, rids, src0.home, executor_of(src0))[0] if src0.kind == "plan" else None)
    except Exception as e:  # noqa: BLE001  (informational)
        sl = {"error": repr(e)}
    chk["limits"]["source"] = sl
    chk["limits"]["inherited"] = {
        "vel": bool(not chk["limits"]["vel_pass"] and (sl.get("segment_vel_frac_vmax_max") or 0) > 1 + TOL_V),
        "acc": bool(not chk["limits"]["acc_pass"] and (sl.get("dense_acc_frac_amax_max") or 0) > 1 + TOL_A),
        "pos": bool(not chk["limits"]["pos_pass"] and (sl.get("joint_limit_excess_rad") or 0) > TOL_Q)}
    chk["limits"]["gate"] = bool(limits_gate)
    # ── weld operations stay slow (full profile), measured the way the backend planned them ──
    if profile == "full":
        if backend == "retime":
            rw = _g_(pinfo, "tools_retime", "report", "dynamics", "all", "weld_v_ratio_max")
            cap = float(_g_(pinfo, "tools_retime", "report", "cfg", "weld_v") or WELD_V_RETIME)
            chk["weld_rule"] = {"metric": "tools.retime labelled approach / lift-off (its own report)",
                                "max_speed_frac_vmax": rw, "cap": cap,
                                "pass": rw is not None and float(rw) <= cap + TOL_WELD_V}
        elif src.kind == "plan" and (src.meta.get("trajgen_full_speed") == "full" or
                                     src.meta.get("trajgen_speed") == "full"):
            ws = weld_phase_speed(legs, vmax_by)
            cap = _scene_weld_vscale(src) or weld_vfrac
            chk["weld_rule"] = {"metric": "waypoint segments with phase approach / retract (trajgen weld strokes)",
                                **ws, "cap": cap, "pass": ws["max_speed_frac_vmax"] <= cap + TOL_WELD_V}
        else:
            ws = chk["weld_op_speed"]["max_speed_frac_vmax"] or 0.0
            chk["weld_rule"] = {"metric": "btexport weld zone (TCP within weld_radius of the weld, contiguous at the "
                                          "leg end / next leg start)", "max_speed_frac_vmax": ws, "cap": weld_vfrac,
                                "weld_source_cap": bool(weld_source_cap),
                                "pass": ws <= weld_vfrac + TOL_WELD_V}
    # ── completeness: every weld of the task is in the export ──
    done = sorted({str(e[1]) for e in weld_events_of(tree, legs)})
    missing = sorted(set(map(str, wpos)) - set(done))
    chk["complete"] = {"n_task_welds": len(wpos), "n_exported_welds": len(done), "missing": missing,
                       "source_success": src0.meta.get("success"), "allow_partial": bool(allow_partial),
                       "pass": (not missing and src0.meta.get("success", True) is not False) or bool(allow_partial)}
    timing = {"makespan_motion_s": meta.get("makespan_motion_s"), "executor_end_s": meta.get("executor_end_s"),
              "umarell_schedule_makespan_s": meta.get("umarell_schedule_makespan_s"),
              "last_weld_done_s": _r(max([l.t1 + l.hold_s for l in legs if l.kind == "weld"], default=0.0), 3)}
    metadata = tree.setdefault("metadata", {})
    metadata.update({
        "exporter": EXPORTER, "exporter_version": EXPORTER_VERSION,
        "source": {"spec": src.spec, "kind": src.kind,
                   **{k: v for k, v in src.meta.items() if k in ("controller", "plan_file", "trajgen_run", "timeline",
                                                                 "exec_result", "generator", "scene", "task",
                                                                 "lookahead", "lookahead_models")}},
        "profile": profile, "profile_detail": {k: v for k, v in pinfo.items() if k != "sync_rest"},
        "robots": rids, "n_robots": len(rids),
        "limits": {r: lims[r].record() for r in rids},
        "robot_model": _robot_model(src),
        "executor_margin_s": MARGIN_S, "umarell_default_completion_margin_s": 1.0,
        "weld_hold_s": chk["welds"]["min_hold_s"],
        "timing": timing,
        "checks": {"timing_max_start_error_s": (chk["timing"] or {}).get("max_start_error_s"),
                   "legs_at_rest_max_speed_frac_vmax": chk["rest"]["max_speed_frac_vmax"],
                   "weld_legs_max_end_error_mm": chk["welds"]["max_end_error_mm"],
                   "weld_op_max_speed_frac_vmax": chk["weld_op_speed"]["max_speed_frac_vmax"],
                   "peak_vel_frac_vmax": chk["dynamics"]["vel_frac_vmax"]["max"],
                   "peak_acc_frac_amax": chk["dynamics"]["acc_frac_amax"]["max"],
                   "peak_segment_vel_frac_vmax": chk["limits"]["segment_vel_frac_vmax_max"],
                   "joint_limit_excess_rad": chk["limits"]["joint_limit_excess_rad"],
                   "urdf_limits_pass": chk["limits"]["pass"],
                   "missing_welds": chk["complete"]["missing"]},
    })
    metadata["steps"] = len(_step_nodes(tree["root"])) if plan_shape(tree) == "steps" else metadata.get("steps", 1)
    n_exec = sum(_kind(lf) == "ExecuteTrajectory" for lf in _leaves(tree["root"]))
    metadata["operations"] = n_exec
    plan["num_operations"] = n_exec
    plan.setdefault("status", "ok")
    plan.setdefault("created_s", time.time())
    tree["BTCPP_format"] = "4"
    tree.setdefault("main_tree_to_execute", "MainTree")
    tree["xml"] = to_xml(tree)
    order = ["tree_id", "cell_id", "created_s", "status", "num_operations", "behaviour_tree"]
    plan = {k: plan[k] for k in order if k in plan} | {k: v for k, v in plan.items() if k not in order}
    torder = ["tree_id", "cell_id", "BTCPP_format", "main_tree_to_execute", "root", "blackboard", "metadata", "xml"]
    plan["behaviour_tree"] = {k: tree[k] for k in torder if k in tree} | {k: v for k, v in tree.items() if k not in torder}
    s_end, sv, src_weld = None, {}, None
    if src0.kind == "plan":
        _l0, _m0 = replay(src0.plan, rids, src0.home, executor_of(src0))
        s_end = _m0.get("executor_end_s")
        if profile == "full":
            src_weld = weld_op_speed_check(_l0, tcp, wpos, vmax_by, weld_radius)
        sv = {"source_clock": "the source plan on umarell's executor clock"}
    elif profile == "full" and compare_source:
        try:          # the source's own timing = its umarell-profile export (execute.py sync_rest, 0.49 a_max envelope)
            _, r0 = export(src0, profile="umarell", tree_id=f"{tid}_srctiming", acc_frac_umarell=acc_frac_umarell,
                           compare_source=False, limits_gate=False, allow_partial=True)
            s_end = r0["timing"]["executor_end_s"]
            src_weld = r0["checks"]["weld_op_speed"]
            sv = {"source_clock": "the source's umarell-profile export (its own timing) on umarell's executor clock"}
        except Exception as e:  # noqa: BLE001
            sv = {"error": repr(e)}
    if src0.kind == "rollout" and src0.t is not None:
        sv["source_rollout_s"] = _r(float(src0.t[-1] - src0.t[0]), 4)
    sv["source_end_s"] = _r(s_end, 4)
    if src_weld is not None:
        ex = chk["weld_op_speed"]
        chk["weld_op_speed_vs_source"] = {
            "zone": f"TCP within {weld_radius} m of the weld, contiguous at the weld leg end / next leg start",
            "export_joint_frac_vmax": ex["max_speed_frac_vmax"], "source_joint_frac_vmax": src_weld["max_speed_frac_vmax"],
            "export_tcp_m_s": ex.get("max_tcp_speed_m_s"), "source_tcp_m_s": src_weld.get("max_tcp_speed_m_s"),
            "faster_than_source": None if ex["max_speed_frac_vmax"] is None or src_weld["max_speed_frac_vmax"] is None
            else bool(ex["max_speed_frac_vmax"] > src_weld["max_speed_frac_vmax"] + 1e-3)}
    sv["export_executor_end_s"] = timing["executor_end_s"]
    sv["export_motion_s"] = timing["makespan_motion_s"]
    if profile == "full" and s_end is not None and timing["executor_end_s"] is not None:
        sv["full_not_slower_than_source"] = bool(timing["executor_end_s"] <= float(s_end) + 1e-6)
        if not sv["full_not_slower_than_source"]:
            rep.setdefault("warnings", []).append(
                f"full profile slower than the source's own timing ({timing['executor_end_s']} s executor end vs "
                f"{_r(s_end, 3)} s): requirement 2 (full speed outside the weld operations) not met by this export")
    rep["speed_vs_source"] = sv
    # per-waypoint velocities / accelerations (tools/ctrl's smoother writes them): dropped when the export rebuilds
    # the trajectories from positions (P3U round 2: silently lost before)
    if src0.kind == "plan" and src0.plan is not None:
        def _has_derivs(pl):
            for v in ((pl.get("behaviour_tree") or {}).get("blackboard") or {}).values():
                if isinstance(v, list) and v and isinstance(v[0], dict) and ("velocities" in v[0] or
                                                                             "accelerations" in v[0]):
                    return True
            return False
        if _has_derivs(src0.plan) and not _has_derivs(plan):
            rep.setdefault("warnings", []).append(
                "the source waypoints carry velocities / accelerations (e.g. tools/ctrl smoothing); this export "
                "rebuilt the trajectories from positions on a new clock and dropped them: run the tools/ctrl smoother "
                "after the full-speed export if the controller needs them")
            chk["derivatives_dropped"] = True
    rep["bt_fidelity_rad"] = fid["bt_fidelity_rad"]
    rep["bt_fidelity"] = fid
    rep.update(tree_id=tid, profile_detail=pinfo, timing=timing, checks=chk,
               export_pass=bool((chk["timing"] or {"pass": True})["pass"] and chk["rest"]["pass"] and chk["welds"]["pass"]
                                and chk["unique_operation_ids"] and (chk["limits"]["pass"] or not limits_gate)
                                and (chk.get("weld_rule") or {"pass": True})["pass"] and chk["complete"]["pass"]
                                and (chk.get("retime_integrity") or {"pass": True})["pass"]),
               wall_s=_r(time.time() - t_wall, 1))
    if out is not None:
        out = Path(out)
        out.parent.mkdir(parents=True, exist_ok=True)
        try:
            txt = json.dumps(plan, indent=1, allow_nan=False)
        except ValueError as e:            # never write NaN / Infinity tokens (not JSON, RFC 8259)
            raise Unsupported(f"the export contains non-finite numbers ({e}); not written") from None
        out.write_text(txt)
        rep["plan"] = str(out)
    if xml is not None:
        xml = Path(xml)
        xml.parent.mkdir(parents=True, exist_ok=True)
        xml.write_text(plan["behaviour_tree"]["xml"])
        rep["xml"] = str(xml)
    return plan, rep
