"""umarell BT plan JSON for any number of robots, and its replay on one clock (executor rules of
``player.bt_plan``: Sequence / Parallel / Delay / ExecuteTrajectory / MoveHome / WaitForRobot).

Two plan shapes:
  * ``plan_from_steps``: root Sequence of steps, a step is a Parallel of one leaf per moving robot (umarell's
    sync-at-rest model; trajgen.search output);
  * ``plan_from_sequences``: root Sequence [ Parallel [ Sequence(robot_1), ..., Sequence(robot_R) ] ]; every robot
    runs its own sequence of legs on the common clock; a leg's start is held relative to the other robots' completed
    operations by gate branches Sequence[WaitForRobot(op), Delay(d)] that run in Parallel with the robot's previous
    leg (trajgen.coord output). Ungated start delays are a stationary prefix of the leg's own trajectory.

``replay(plan, robot_ids, home)`` -> ``trajgen.ntimeline.NTimeline`` on the executor clock. For two robots named
robot1/robot2 it gives the same legs as ``player.bt_plan.timeline_from_bt_plan`` (tests/test_trajgen_nrobot.py).
"""
from __future__ import annotations

import time
import uuid
from typing import Optional, Sequence

import numpy as np

from player.bt_plan import COMPLETION_MARGIN_S, DEFAULT_HOLD_S, node_kind, port, resolve, to_xml
from player.timeline import Leg
from trajgen.ntimeline import NTimeline


def _nid() -> str:
    return uuid.uuid4().hex[:8]


def _leaf(leg: Leg, key: str, nid) -> dict:
    if leg.kind == "home":
        return {"type": "MoveHome", "name": f"home_{leg.robot}", "id": nid(),
                "ports": {"robot_ns": leg.robot, "trajectory": "{" + key + "}"}}
    otype = leg.operation_type or ("weld" if leg.kind == "weld" else "move")
    return {"type": "ExecuteTrajectory", "name": f"{otype}_{leg.robot}_{leg.op_id}", "id": nid(),
            "ports": {"robot_ns": leg.robot, "operation_id": leg.op_id, "operation_type": otype,
                      "trajectory": "{" + key + "}", "hold_duration_s": float(leg.hold_s),
                      "velocity_scale": float(leg.vscale), "has_interlock": False}}


def _envelope(cell_id, root, bb, n_exec, tree_id, created_s, metadata_extra, n_steps):
    meta = {"steps": n_steps, "operations": n_exec}
    if metadata_extra:
        meta.update(metadata_extra)
    tree = {"tree_id": tree_id, "cell_id": cell_id, "BTCPP_format": "4", "main_tree_to_execute": "MainTree",
            "root": root, "blackboard": bb, "metadata": meta}
    tree["xml"] = to_xml(tree)
    return {"tree_id": tree_id, "cell_id": cell_id, "created_s": time.time() if created_s is None else created_s,
            "status": "partial" if (metadata_extra or {}).get("partial") else "ok", "num_operations": n_exec,
            "behaviour_tree": tree}


def _ptr(v) -> str:
    """Blackboard key of a port value '{key}' (umarell btcpp.resolve / player.bt_plan.resolve: value[1:-1]); the
    id's own braces are kept (round-3 finding: str.strip('{}') also removed a trailing '}' of the weld id)."""
    if isinstance(v, str) and len(v) >= 2 and v.startswith("{") and v.endswith("}"):
        return v[1:-1]
    return ""


def assign_bb_keys(legs) -> None:
    """Give every leg a unique blackboard key, in place: the readable ``traj_<robot>_<op>`` where it is unique,
    else that key + '_' + a hash of the (robot, op) pair (round-3 finding: robot 'a' + weld 'b_X' and robot 'a_b' +
    weld 'X' both gave traj_a_b_X and the export crashed)."""
    import hashlib
    import json as _json
    from collections import Counter

    keys = [l.bb_key or f"traj_{l.robot}_{l.op_id}" for l in legs]
    cnt = Counter(keys)
    used = {k for k in keys if cnt[k] == 1}
    for l, k in zip(legs, keys):
        if cnt[k] > 1:
            h = hashlib.sha1(_json.dumps([str(l.robot), str(l.op_id)]).encode()).hexdigest()[:8]
            k2, i = f"{k}_{h}", 0
            while k2 in used:
                i += 1
                k2 = f"{k}_{h}_{i}"
            k = k2
        used.add(k)
        l.bb_key = k


def _ids(seed: int):
    rng = np.random.default_rng(seed)
    return lambda: "%08x" % int(rng.integers(0, 2 ** 32))


def plan_from_steps(cell_id: str, steps: list, robot_ids, *, tree_id: Optional[str] = None,
                    created_s: Optional[float] = 0.0, metadata_extra: Optional[dict] = None) -> dict:
    """``steps``: list of lists of legs (one leg per robot per step), executor-consistent."""
    nid = _ids(0)
    bb, children, n_exec = {}, [], 0
    assign_bb_keys([l for step in steps for l in step])
    for step in steps:
        t_step = min(l.t0 for l in step)
        nodes = []
        for leg in sorted(step, key=lambda l: list(robot_ids).index(l.robot)):
            key = leg.bb_key or f"traj_{leg.robot}_{leg.op_id}"
            if key in bb:
                raise ValueError(f"duplicate blackboard key {key}")
            bb[key] = leg.rel_trajectory()
            node = _leaf(leg, key, nid)
            n_exec += node["type"] == "ExecuteTrajectory"
            delay = leg.t0 - t_step
            if delay > 5e-4:
                node = {"type": "Delay", "name": f"delay_{node['name']}", "ports": {"delay_msec": int(round(delay * 1000))},
                        "children": [node]}
            nodes.append(node)
        children.append(nodes[0] if len(nodes) == 1 else
                        {"type": "Parallel", "id": nid(), "ports": {"success_count": -1, "failure_count": 1}, "children": nodes})
    root = {"type": "Sequence", "id": nid(), "ports": {}, "children": children}
    return _envelope(cell_id, root, bb, n_exec, tree_id or f"trajgen_{cell_id}", created_s, metadata_extra, len(children))


def plan_from_sequences(cell_id: str, legs: Sequence[Leg], robot_ids, deps: dict, *, tree_id: Optional[str] = None,
                        created_s: Optional[float] = 0.0, metadata_extra: Optional[dict] = None) -> dict:
    """Per-robot sequences in one Parallel. ``deps``: {(robot, op_id): [gate, ...]} for the leg ``op_id``:
      * (other_robot, other_op_id, delay_s): relative gate (trajgen.coord.legs_from_tracks). A branch
        Sequence[WaitForRobot(other op), Delay(delay_s)] runs in Parallel with the robot's previous leg (before the
        first leg: on its own), so the leg starts at max(previous leg complete, other op complete + delay_s);
      * (other_robot, other_op_id): a WaitForRobot directly in front of the leg.
    Legs must be executor-consistent with these rules."""
    nid = _ids(0)
    bb, seqs, n_exec = {}, [], 0
    assign_bb_keys(list(legs))
    for rid in robot_ids:
        mine = sorted([l for l in legs if l.robot == rid], key=lambda l: l.t0)
        if not mine:
            continue
        kids = []
        for i, leg in enumerate(mine):
            dd = deps.get((leg.robot, leg.op_id)) or []
            rel = [g for g in dd if len(g) == 3]
            direct = [g[:2] for g in dd if len(g) == 2]
            if rel:
                branches = []
                for r, o, dl in rel:
                    w = {"type": "WaitForRobot", "name": f"wait_{rid}_{leg.op_id}_{r}", "id": nid(),
                         "ports": {"robot_ns": rid, "depends_on_ops": [{"robot": r, "operation_id": o}]}}
                    br = [w]
                    if dl > 5e-4:
                        br.append({"type": "Delay", "name": f"delay_{rid}_{leg.op_id}_{r}", "id": nid(),
                                   "ports": {"delay_msec": int(round(dl * 1000))}, "children": []})
                    branches.append({"type": "Sequence", "name": f"gate_{rid}_{leg.op_id}_{r}", "id": nid(),
                                     "ports": {}, "children": br})
                if kids:                       # run the gates in parallel with the previous leg
                    kids[-1] = {"type": "Parallel", "name": f"leg_and_gates_{rid}_{leg.op_id}", "id": nid(),
                                "ports": {"success_count": -1, "failure_count": 1}, "children": [kids[-1]] + branches}
                else:
                    kids.append(branches[0] if len(branches) == 1 else
                                {"type": "Parallel", "name": f"gates_{rid}_{leg.op_id}", "id": nid(),
                                 "ports": {"success_count": -1, "failure_count": 1}, "children": branches})
            if direct:
                kids.append({"type": "WaitForRobot", "name": f"wait_{rid}_{leg.op_id}", "id": nid(),
                             "ports": {"robot_ns": rid, "depends_on_ops": [{"robot": r, "operation_id": o}
                                                                           for r, o in direct]}})
            key = leg.bb_key or f"traj_{leg.robot}_{leg.op_id}"
            if key in bb:
                raise ValueError(f"duplicate blackboard key {key}")
            bb[key] = leg.rel_trajectory()
            node = _leaf(leg, key, nid)
            n_exec += node["type"] == "ExecuteTrajectory"
            kids.append(node)
        seqs.append({"type": "Sequence", "name": f"program_{rid}", "id": nid(), "ports": {}, "children": kids})
    par = {"type": "Parallel", "id": nid(), "ports": {"success_count": -1, "failure_count": 1}, "children": seqs}
    root = {"type": "Sequence", "id": nid(), "ports": {}, "children": [par if len(seqs) > 1 else (seqs[0] if seqs else par)]}
    return _envelope(cell_id, root, bb, n_exec, tree_id or f"trajgen_{cell_id}", created_s, metadata_extra, 1)


# ─────────────────────────────────────────────────────────────────────────────  replay
class _Replay:
    def __init__(self, tree, robot_ids, home, margin, prev, lag=None):
        self.bb = dict(tree.get("blackboard") or {})
        self.rids = list(robot_ids)
        self.home = home
        self.margin = margin
        self.legs: list[Leg] = []
        self.completed: dict = {}
        self.unresolved = 0
        self.prev = prev
        self.lag = lag                      # optional lag(rid, k) -> extra completion latency (s) of rid's k-th leg
        self.n_done = {}

    def _late(self, rid):
        k = self.n_done.get(rid, 0)
        self.n_done[rid] = k + 1
        return 0.0 if self.lag is None else float(self.lag(rid, k))

    def run(self, node, t):
        k = node_kind(node)
        kids = node.get("children") or []
        if k in ("Sequence", "SequenceWithMemory"):
            for c in kids:
                t = self.run(c, t)
            return t
        if k == "Parallel":
            return max([self.run(c, t) for c in kids] + [t])
        if k == "Delay":
            t += float(port(node, "delay_msec", 0) or 0) / 1000.0
            for c in kids:
                t = self.run(c, t)
            return t
        if k == "ExecuteTrajectory":
            rid, op = port(node, "robot_ns"), str(port(node, "operation_id"))
            traj = resolve(port(node, "trajectory"), self.bb)
            if not isinstance(traj, list) or not traj:
                raise ValueError(f"ExecuteTrajectory {op}: empty trajectory")
            vs = float(port(node, "velocity_scale", 1.0)) or 1.0
            hold = float(port(node, "hold_duration_s", DEFAULT_HOLD_S))
            t += float(port(node, "start_delay_s", 0.0) or 0.0)
            otype = str(port(node, "operation_type", "weld"))
            bbk = port(node, "trajectory")
            leg = Leg.from_trajectory(rid, op, "weld" if otype == "weld" else "move", t, traj, vs, hold_s=hold,
                                      operation_type=otype, node_type="ExecuteTrajectory",
                                      bb_key=_ptr(bbk))
            leg.t_done = t + float(traj[-1]["time_s"]) / vs + hold + self.margin + self._late(rid)
            self.legs.append(leg)
            self.completed[(rid, op)] = leg.t_done
            return leg.t_done
        if k == "MoveHome":
            rid = port(node, "robot_ns")
            traj = resolve(port(node, "trajectory"), self.bb)
            if not isinstance(traj, list) or not traj:
                raise ValueError("MoveHome without a trajectory is not produced by trajgen")
            bbk = port(node, "trajectory")
            leg = Leg.from_trajectory(rid, "home", "home", t, traj, 1.0, node_type="MoveHome",
                                      bb_key=_ptr(bbk))
            leg.t_done = t + float(traj[-1]["time_s"]) + self.margin + self._late(rid)
            self.legs.append(leg)
            return leg.t_done
        if k == "WaitForRobot":
            ready = t
            for d in resolve(port(node, "depends_on_ops"), self.bb) or []:
                key = (str(d.get("robot")), str(d.get("operation_id")))
                done = self.completed.get(key)
                if done is None and self.prev is not None:
                    done = self.prev.completed.get(key)
                if done is None:
                    self.unresolved += 1
                    continue
                ready = max(ready, done)
            return ready
        if k == "CheckSafety":
            return t
        raise ValueError(f"unsupported node {k!r}")


def replay(plan: dict, robot_ids, home, *, dt: float = 0.05, margin: float = COMPLETION_MARGIN_S,
           lag=None) -> NTimeline:
    """Executor-clock timeline of a plan (fixed point over WaitForRobot gates; Parallel children run in order).
    ``lag(rid, k)``: optional extra completion latency (s) of robot rid's k-th leg (executor poll lateness)."""
    tree = plan["behaviour_tree"] if "behaviour_tree" in plan else plan
    root = tree["root"]
    home = np.asarray(home, float).reshape(len(robot_ids), 6)
    prev = None
    for _ in range(400):                       # gate chains may alternate between robots along the whole program
        rp = _Replay(tree, robot_ids, home, margin, prev, lag)
        t_end = rp.run(root, 0.0)
        if prev is not None and prev.completed == rp.completed and rp.unresolved == 0:
            break
        if prev is None and rp.unresolved == 0:
            break
        prev = rp
    else:
        raise RuntimeError("plan replay: WaitForRobot gates did not converge (cyclic dependencies?)")
    meta = {"executor_end_s": round(t_end, 4), "makespan_motion_s": round(max([l.t1 for l in rp.legs] + [0.0]), 4),
            "n_legs": len(rp.legs), "completion_margin_s": margin}
    return NTimeline.from_legs(rp.legs, robot_ids=robot_ids, dt=dt, t_end=t_end, home=home, meta=meta)


def timing_check(legs: Sequence[Leg], tl: NTimeline) -> dict:
    """Intended leg start times vs the replay of the exported plan."""
    got = {(l.robot, l.op_id): l.t0 for l in tl.legs}
    rows = [(l.robot, l.op_id, l.t0, got.get((l.robot, l.op_id))) for l in legs]
    errs = [abs(a - b) for _, _, a, b in rows if b is not None]
    return {"max_start_error_s": max(errs) if errs else 0.0, "missing": [r[:2] for r in rows if r[3] is None]}
