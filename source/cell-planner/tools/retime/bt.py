"""umarell behaviour-tree export of a retimed timeline (direct support; BTCPP v4 JSON with blackboard + XML).

umarell's executor (player.bt_plan / trajgen.btplan rules) runs a root Sequence of steps; a step is a Parallel of
one leaf per robot, a leaf may sit behind a Delay, and a step completes 1.0 s (completion margin) after its last
leaf. To keep the retimed shared clock EXACTLY (the relative timing of the arms is what the safety re-check
certified), the program is cut only at instants where every arm is at rest; each robot's motion inside such a window
is ONE leaf starting at the window start (a stationary prefix encodes the robot's wait; its welds' still holds are
part of the trajectory, ``hold_duration_s`` 0; the window's welds are listed in the metadata ``leaves``; a leaf
whose motion ends with the arrival at its only weld is named after that weld, so umarell's per-weld
validate_approach applies to it). The executor clock is therefore the retimed clock plus 1.0 s per
step (all arms at rest during the margin, so the relative timing is unchanged). The export is replayed with
trajgen.btplan.replay and compared with the retimed timeline (start errors, same-instant joint difference after
removing the margins).

For per-weld leaves (umarell's validate_approach per weld) use tools.btexport (track BTX) on the retimed timeline
(``--from timeline:<npz>``); its rollout path re-cuts the motion at rests with execute.py's sync_rest warp, which
never runs faster than its source but may slow parts of a full-speed timeline down (its 0.49 a_max envelope).
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

COMPLETION_MARGIN_S = 1.0


def _windows(tl, eps=1e-9):
    """All-at-rest cut instants of an NTimeline (legs = moving intervals)."""
    iv = sorted((l.t0, l.t1) for l in tl.legs)
    blocks = []
    for a, b in iv:
        if blocks and a <= blocks[-1][1] + eps:
            blocks[-1][1] = max(blocks[-1][1], b)
        else:
            blocks.append([a, b])
    return blocks


def _grid_times(rel, min_seg_s: float = 1e-3, quantum: float = 1e-4) -> np.ndarray:
    """Relative waypoint times (sorted, rel[0] = 0) -> the same instants on the ``quantum`` grid (umarell's 0.1 ms
    time_s resolution), strictly increasing, interior points at least ``min_seg_s`` after their predecessor; the
    first and the last instant are always kept."""
    g = np.round(np.asarray(rel, float) / quantum) * quantum
    if len(g) < 2:
        return g
    keep = [0]
    for i in range(1, len(g) - 1):
        if g[i] - g[keep[-1]] >= min_seg_s - 1e-9 and g[-1] - g[i] >= min_seg_s - 1e-9:
            keep.append(i)
    if g[-1] > g[keep[-1]] + 1e-9:
        keep.append(len(g) - 1)
    elif len(keep) > 1:
        keep[-1] = len(g) - 1
    return np.round(g[keep] / quantum) * quantum


def export_bt(tl, cell_id: str, out, *, tree_id: str | None = None, meta: dict | None = None, dt_out: float = 0.012,
              min_seg_s: float = 1e-3):
    """Write <out> (umarell plan envelope JSON) and <out>.xml; returns a report dict."""
    from player.timeline import Leg
    from trajgen.btplan import plan_from_steps, replay
    rids = tuple(tl.robot_ids)
    blocks = _windows(tl)
    ev = list(tl.weld_events)
    steps, shift = [], 0.0
    leaf_map = []
    assigned = set()
    for n, (a, b) in enumerate(blocks):
        legs = []
        w0 = blocks[n - 1][1] if n else 0.0          # window = previous all-rest cut .. this block's end
        for r, rid in enumerate(rids):
            mine = [l for l in tl.legs if l.robot == rid and l.t0 >= a - 1e-9 and l.t1 <= b + 1e-9]
            if not mine:
                continue
            t0, t1 = w0, max(l.t1 for l in mine)          # stationary prefix = the robot's wait in this window
            k = max(1, int(np.ceil((t1 - t0) / dt_out - 1e-9)))
            ts = np.r_[t0 + np.arange(k) * (t1 - t0) / k, t1]
            ts = np.unique(np.r_[ts, [x for l in mine for x in (l.t0, l.t1)]])
            # umarell's trajectories carry time_s rounded to 0.1 ms (player Leg.rel_trajectory). Sampling the
            # positions at the unrounded instants and letting the writer round the stamps made the segment slopes
            # jitter by up to (0.05 ms / segment) (several % of v_max on short segments; P3U verification): the
            # waypoints are put ON the 0.1 ms grid (relative to the leaf start) and sampled there, and interior
            # points closer than ``min_seg_s`` to their predecessor are dropped (the leaf end is kept)
            ts = t0 + _grid_times(ts - t0, min_seg_s)
            q = tl.sample(ts)[:, r]
            # a weld belongs to the window of its weld leg (leg end and event arrival differ by up to one 250 Hz
            # sample, so a pure time test could push an arrival just past the block end and lose the weld)
            mine_w = {str(l.op_id) for l in mine if l.kind == "weld"}
            welds = [e for e in ev if e[0] == rid and (rid, str(e[1])) not in assigned and
                     (str(e[1]) in mine_w or a - 1e-6 <= e[2] <= b + 1e-6)]
            assigned.update((rid, str(e[1])) for e in welds)
            op = f"{rid}_S{n:02d}" + ("_" + "_".join(str(e[1]) for e in welds) if welds else "")
            # a leaf whose motion ENDS with its only weld's arrival is that weld's leaf (op_id = weld id), so
            # umarell's per-weld validate_approach applies; the still hold continues into the next window's
            # stationary prefix (hold_duration_s 0: the hold is part of the trajectories)
            last = max(mine, key=lambda l: l.t1)          # the robot's last leg in this window
            single = len(welds) == 1 and last.kind == "weld" and str(last.op_id) == str(welds[0][1])
            if single:
                op = str(welds[0][1])
            kind = "home" if (n == len(blocks) - 1 and not welds and np.allclose(q[-1], tl.home[r], atol=1e-4)) \
                else ("weld" if welds else "move")
            lg = Leg(robot=rid, op_id=op, kind="move" if kind == "home" else kind, t0=float(t0 + shift),
                     t1=float(t1 + shift), times=ts + shift, positions=q,
                     phase=np.array(["retimed"] * len(ts), dtype="<U16"), hold_s=0.0,
                     operation_type="weld" if welds else "move")
            lg.t_done = lg.t1 + COMPLETION_MARGIN_S
            legs.append(lg)
            leaf_map.append(dict(step=n, robot=rid, op_id=op, ends_at_weld=bool(single),
                                 welds=[[e[1], round(e[2], 4), round(e[3], 4)] for e in welds]))
        if legs:
            steps.append(legs)
            shift += COMPLETION_MARGIN_S
    md = dict(generator="tools.retime (track SPEED)", export="all-at-rest steps, one leaf per robot per step",
              completion_margin_s=COMPLETION_MARGIN_S, leaves=leaf_map, **(meta or {}))
    plan = plan_from_steps(cell_id, steps, rids, tree_id=tree_id or f"retimed_{cell_id}", metadata_extra=md)
    # the welds of each leaf as a port (informational; umarell's executor ignores unknown ports)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(plan))
    out.with_suffix(".xml").write_text(plan["behaviour_tree"]["xml"])
    # replay check
    rp = replay(plan, rids, tl.home)
    starts = {(l.robot, l.op_id): l.t0 for l in rp.legs}
    errs = [abs(starts[(l.robot, l.op_id)] - l.t0) for st in steps for l in st if (l.robot, l.op_id) in starts]
    # same-instant difference: map the executor clock back by removing the margins
    T = float(tl.t[-1])
    ts = np.linspace(0, T, 2001)
    cut_ends = [b for (a, b) in blocks]
    off = np.array([COMPLETION_MARGIN_S * sum(1 for c in cut_ends[:-1] if c < x - 1e-9) for x in ts])
    dq = float(np.abs(rp.sample(ts + off) - tl.sample(ts)).max())
    return dict(path=str(out), xml=str(out.with_suffix(".xml")), n_steps=len(steps), n_leaves=len(leaf_map),
                executor_end_s=round(float(rp.meta["executor_end_s"]), 3), retimed_end_s=round(T, 3),
                max_start_error_s=round(max(errs) if errs else 0.0, 6), replay_max_joint_diff_rad=round(dq, 6))
