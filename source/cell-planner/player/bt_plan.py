"""umarell BehaviourTree plan <-> Timeline.

``timeline_from_bt_plan`` replays a saved plan on one continuous clock using the rules of
umarell's executor (``third_party/sample-umarell/agents/cell/bt_executor.py``) and robot
stack. Line numbers refer to the repo copy (HEAD eccb1bc):

* ``Sequence`` / ``SequenceWithMemory``: children one after another (bt_executor.py:298).
* ``Parallel``: all children start at the same instant (one thread each) and the node ends
  when the last child ends (bt_executor.py:322-345).
* ``Delay``: wait ``delay_msec`` then run the child (bt_executor.py:305-320).
* ``ExecuteTrajectory`` (bt_executor.py:347-446):
    1. legacy ``start_delay_s`` sleep (:355, :366);
    2. optional interlock: if ``has_interlock`` and ``interlock_target.joint_positions``
       a one-point move with ``time_s = home_time_s`` (robots.yaml
       planning.interlock_move_time_s, 2.5 s) is dispatched, then a Cartesian gate waits
       for the TCP (:372-386);
    3. the trajectory is dispatched; the robot agent divides every ``time_s`` by
       ``velocity_scale`` (agents/robot/robot_agent.py:189-191);
    4. the node completes after ``traj_end / velocity_scale + hold_duration_s +
       completion_margin_s`` (:398; hold port default 0.5 at :352, margin 1.0 at :87).
       The arm is stationary at the last waypoint for the hold and the margin (the
       robot agent sleeps ``hold_s`` after the FJT goal finishes, robot_agent.py:264).
* ``MoveHome`` with a trajectory: dispatched with hold 0 / vscale 1, completes after
  ``traj_end + completion_margin_s`` (:557-563). Without one: a one-point blind home with
  ``time_s = home_return_time_s`` (7.0 s) and completion after 7.0 + 1.0 s (:564-576).
* ``WaitForRobot``: waits for completion events (``depends_on_ops``), joint targets
  (``depends_on_joints``, tol 0.15 rad) or TCP positions (:481-512).
* ``CheckSafety``: instantaneous (:542).

The controller (controller/pinocchio_robot_controller.py) replays the dispatched points
with linear interpolation (``_sample_trajectory``, :352). Its enrichment steps are no-ops
for plan trajectories: the FJT goal carries zero velocities (robot_agent.py:69), so
``_enrich_deceleration`` (:307-309) and ``_enrich_hold`` (:339-341) return the points
unchanged and ``_enrich_interpolate`` only inserts collinear points.

Modelling choices that differ from the live system (none of them is exercised by the two
saved 2_small plans, which only use Sequence / Parallel / ExecuteTrajectory / MoveHome):
* dispatch latency and the executor's 20 Hz polling (each node ends on the first tick
  after its wall duration, i.e. up to +50 ms per node) are ignored;
* one-point moves (interlock, blind home) are drawn as linear joint moves over their
  ``time_s``; the sim controller actually jumps to the single point
  (``_sample_trajectory`` returns ``points[0]`` while ``elapsed <= points[0].t``);
* an interlock is assumed to take its full ``interlock_move_time_s`` before the main
  trajectory is dispatched;
* ``depends_on_positions`` (Cartesian) gates need FK and are treated as already satisfied
  (a warning is recorded in ``meta["warnings"]``).

Arms: ``robot1`` / ``robot2`` by default; ``robot_ids`` replays an N-arm cell's plan with the same executor rules
(``Timeline.robot_ids``; the default pair gives exactly the old two-arm result).

Pure numpy + json + xml (stdlib); runs in both venvs.
"""
from __future__ import annotations

import json
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional, Sequence
from xml.dom import minidom

import numpy as np

from player.timeline import DT, HOME, ROBOT_IDS, Leg, Timeline, sample_leg

# ── executor constants (agents/cell/bt_executor.py ExecConfig + robots.yaml planning) ──
COMPLETION_MARGIN_S = 1.0     # ExecConfig.completion_margin_s (bt_executor.py:87)
DEFAULT_HOLD_S = 0.5          # bt_port(node, "hold_duration_s", 0.5) (bt_executor.py:352)
INTERLOCK_MOVE_TIME_S = 2.5   # ExecConfig.home_time_s <- planning.interlock_move_time_s
HOME_RETURN_TIME_S = 7.0      # ExecConfig.home_return_time_s <- planning.home_return_time_s
JOINT_GATE_TOL_RAD = 0.15     # WaitForRobot depends_on_joints default (bt_executor.py:497)

# ── btcpp vocabulary (agents/trajectory/tools/btcpp.py), copied so this stays pure python ──
LEGACY_TO_BTCPP = {"SEQUENCE": "Sequence", "PARALLEL": "Parallel", "EXECUTE": "ExecuteTrajectory",
                   "HOME": "MoveHome", "INTERLOCK_WAIT": "WaitForRobot", "SAFETY_GATE": "CheckSafety"}
LEAF_PORTS = {
    "ExecuteTrajectory": {"_kind": "Action", "robot_ns": None, "operation_id": None,
                          "operation_type": "weld", "trajectory": None, "hold_duration_s": 0.5,
                          "velocity_scale": 1.0, "has_interlock": False, "interlock_target": "",
                          "depends_on_robots": "", "depends_on_positions": ""},
    "MoveHome": {"_kind": "Action", "robot_ns": None, "trajectory": ""},
    "WaitForRobot": {"_kind": "Action", "robot_ns": None, "depends_on_robots": "",
                     "depends_on_joints": "", "depends_on_positions": "", "depends_on_ops": ""},
    "CheckSafety": {"_kind": "Condition", "condition": "estop_clear"},
}
EXECUTOR_RULES = {
    "parallel": "children start together; node ends with the last child",
    "execute_wall_s": "start_delay + traj_end/velocity_scale + hold_duration_s + completion_margin_s",
    "move_home_wall_s": "traj_end + completion_margin_s (planned) | 7.0 + 1.0 (blind)",
    "completion_margin_s": COMPLETION_MARGIN_S, "default_hold_s": DEFAULT_HOLD_S,
    "interpolation": "linear on time (_sample_at_time / _sample_trajectory)",
    "idle_arm": "holds its last pose",
}


def node_kind(node) -> Optional[str]:
    if not isinstance(node, dict) or not node.get("type"):
        return None
    t = str(node["type"])
    return LEGACY_TO_BTCPP.get(t, t)


def port(node: dict, name: str, default=None):
    """btcpp.port: ``ports`` dict first, then legacy top-level fields."""
    ports = node.get("ports")
    if isinstance(ports, dict) and name in ports:
        return ports[name]
    if name in node:
        return node[name]
    if name == "trajectory" and "precomputed_trajectory" in node:
        return node["precomputed_trajectory"]
    return default


def resolve(value, bb: dict):
    if isinstance(value, str) and len(value) > 2 and value[0] == "{" and value[-1] == "}":
        return bb.get(value[1:-1].strip())
    return value


def _children(node: dict) -> list:
    return node.get("children") or []


def load_plan(path_or_dict) -> dict:
    """Plan envelope ``{tree_id, cell_id, ..., behaviour_tree}`` or a bare tree -> envelope."""
    if isinstance(path_or_dict, (str, Path)):
        d = json.load(open(path_or_dict))
    else:
        d = path_or_dict
    if "behaviour_tree" not in d:
        d = {"tree_id": d.get("tree_id"), "cell_id": d.get("cell_id"), "behaviour_tree": d}
    return d


# ─────────────────────────────────────────────────────────────────────────────
#  plan -> timeline
# ─────────────────────────────────────────────────────────────────────────────
class _Replay:
    def __init__(self, tree: dict, home: np.ndarray, margin: float, il_time: float,
                 home_time: float, prev: Optional["_Replay"], robot_ids: Sequence[str] = ROBOT_IDS):
        self.bb = dict(tree.get("blackboard") or {})
        self.home = home
        self.rids = tuple(robot_ids)
        self.margin, self.il_time, self.home_time = margin, il_time, home_time
        self.legs: list[Leg] = []
        self.completed: dict = {}      # (robot, op_id) -> completion time
        self.steps: list[dict] = []
        self.warnings: list[str] = []
        self.prev = prev               # previous fixed-point pass (for WaitForRobot)

    # pose of ``rid`` at time t from the legs placed so far
    def _q_at(self, rid: str, t: float) -> np.ndarray:
        q = self.home[self._ri(rid)].copy()
        for leg in sorted((l for l in self.legs if l.robot == rid), key=lambda l: l.t0):
            if leg.t0 <= t + 1e-12:
                q = sample_leg(leg, np.array([t]))[0]
        return q

    def _ri(self, rid) -> int:
        if rid not in self.rids:
            raise ValueError(f"robot_ns {rid!r} is not one of the replayed robots {list(self.rids)}")
        return self.rids.index(rid)

    def run(self, node: dict, t: float, depth: int = 0) -> float:
        kind = node_kind(node)
        if kind in ("Sequence", "SequenceWithMemory"):
            for c in _children(node):
                t = self.run(c, t, depth + 1)
            return t
        if kind == "Parallel":
            ends = [self.run(c, t, depth + 1) for c in _children(node)]
            return max(ends + [t])
        if kind == "Delay":
            t += float(port(node, "delay_msec", 0) or 0) / 1000.0
            for c in _children(node):
                t = self.run(c, t, depth + 1)
            return t
        if kind == "ExecuteTrajectory":
            return self._execute(node, t)
        if kind == "MoveHome":
            return self._home(node, t)
        if kind == "WaitForRobot":
            return self._wait(node, t)
        if kind == "CheckSafety":
            return t
        raise ValueError(f"executor would refuse unregistered node {kind!r} (bt_executor.py:288)")

    def _execute(self, node: dict, t: float) -> float:
        rid = port(node, "robot_ns")
        op = port(node, "operation_id")
        traj = resolve(port(node, "trajectory"), self.bb)
        if not isinstance(traj, list) or not traj:
            raise ValueError(f"ExecuteTrajectory {op}: empty trajectory (executor: empty_trajectory)")
        vscale = float(port(node, "velocity_scale", 1.0)) or 1.0
        hold = float(port(node, "hold_duration_s", DEFAULT_HOLD_S))
        t += float(port(node, "start_delay_s", 0.0) or 0.0)
        itgt = resolve(port(node, "interlock_target"), self.bb)
        if port(node, "has_interlock") and isinstance(itgt, dict):
            if itgt.get("joint_positions"):
                q0 = self._q_at(rid, t)
                il = [{"positions": list(q0), "time_s": 0.0},
                      {"positions": list(itgt["joint_positions"])[:6], "time_s": self.il_time}]
                leg = Leg.from_trajectory(rid, f"{op}:interlock", "move", t, il,
                                          node_type="ExecuteTrajectory", synthetic="interlock")
                leg.t_done = leg.t1
                self.legs.append(leg)
                t = leg.t1
            if itgt.get("ee_position"):
                self.warnings.append(f"{op}: Cartesian interlock gate assumed satisfied")
        otype = str(port(node, "operation_type", "weld"))
        bbk = port(node, "trajectory")
        leg = Leg.from_trajectory(rid, str(op), "weld" if otype == "weld" else "move", t, traj, vscale,
                                  hold_s=hold, operation_type=otype, node_type="ExecuteTrajectory",
                                  bb_key=bbk.strip("{}") if isinstance(bbk, str) else "")
        leg.t_done = t + float(traj[-1]["time_s"]) / vscale + hold + self.margin
        self.legs.append(leg)
        self.completed[(rid, str(op))] = leg.t_done
        return leg.t_done

    def _home(self, node: dict, t: float) -> float:
        rid = port(node, "robot_ns")
        traj = resolve(port(node, "trajectory"), self.bb)
        bbk = port(node, "trajectory")
        if isinstance(traj, list) and traj:
            leg = Leg.from_trajectory(rid, "home", "home", t, traj, 1.0, node_type="MoveHome",
                                      bb_key=bbk.strip("{}") if isinstance(bbk, str) else "")
            leg.t_done = t + float(traj[-1]["time_s"]) + self.margin
        else:
            q0 = self._q_at(rid, t)
            blind = [{"positions": list(q0), "time_s": 0.0},
                     {"positions": list(self.home[self._ri(rid)]), "time_s": self.home_time}]
            leg = Leg.from_trajectory(rid, "home", "home", t, blind, 1.0, node_type="MoveHome",
                                      synthetic="blind_home")
            leg.t_done = t + self.home_time + self.margin
        self.legs.append(leg)
        return leg.t_done

    def _wait(self, node: dict, t: float) -> float:
        t_ready = t
        ops = resolve(port(node, "depends_on_ops"), self.bb) or []
        for d in ops:
            key = (str(d.get("robot")), str(d.get("operation_id")))
            done = self.completed.get(key)
            if done is None and self.prev is not None:
                done = self.prev.completed.get(key)
            if done is None:
                self.warnings.append(f"WaitForRobot: {key} not resolved in this pass")
                continue
            t_ready = max(t_ready, done)
        for d in (resolve(port(node, "depends_on_joints"), self.bb) or []):
            src = self.prev if self.prev is not None else self
            rid, target = d.get("robot"), d.get("joint_positions")
            tol = float(d.get("tolerance_rad", JOINT_GATE_TOL_RAD))
            if not (rid and target):
                continue
            legs = sorted([l for l in src.legs if l.robot == rid], key=lambda l: l.t0)
            t_end = max([l.t_done for l in src.legs] + [t_ready]) + 1.0
            grid = np.arange(t_ready, t_end, 0.01)
            tl = Timeline(t=grid, q=np.zeros((len(grid), len(self.rids), 6)), legs=legs, home=self.home,
                          robot_ids=self.rids)
            qs = tl.sample(grid)[:, self._ri(rid)]
            err = np.abs((qs - np.asarray(target)[:6] + np.pi) % (2 * np.pi) - np.pi).max(axis=1)
            hit = np.flatnonzero(err <= tol)
            if hit.size:
                t_ready = max(t_ready, float(grid[hit[0]]))
            else:
                self.warnings.append(f"WaitForRobot: {rid} never reaches joint gate (pass)")
        if resolve(port(node, "depends_on_positions"), self.bb):
            self.warnings.append("WaitForRobot depends_on_positions assumed satisfied (no FK)")
        return t_ready


def timeline_from_bt_plan(path_or_dict, *, dt: float = DT, home: Optional[np.ndarray] = None,
                          completion_margin_s: float = COMPLETION_MARGIN_S,
                          interlock_move_time_s: float = INTERLOCK_MOVE_TIME_S,
                          home_return_time_s: float = HOME_RETURN_TIME_S,
                          t_end: Optional[float] = None, robot_ids: Optional[Sequence[str]] = None) -> Timeline:
    """Replay a saved umarell plan on one clock with the executor rules (module docstring).

    ``robot_ids``: the cell's arms in ``home`` order (default robot1 / robot2); the timeline has one column each.

    The 20 Hz grid runs from 0 to the executor end (``t_done`` of the last node, i.e. the
    final MoveHome's trajectory end + 1.0 s margin) unless ``t_end`` is given.
    ``meta`` carries ``makespan_motion_s`` (last arm stops), ``executor_end_s``,
    ``umarell_schedule_makespan_s`` (umarell's own estimate in
    trajectory_quality._timeline: trajectory + hold, no margin) and the per-step schedule.
    """
    env = load_plan(path_or_dict)
    tree = env["behaviour_tree"]
    rids = ROBOT_IDS if robot_ids is None else tuple(robot_ids)
    home = np.stack([HOME] * len(rids)) if home is None else np.asarray(home, float).reshape(len(rids), 6)
    root = tree.get("root")
    if not isinstance(root, dict):
        raise ValueError("behaviour_tree has no 'root' node")

    prev = None
    for _ in range(10):  # fixed point only matters for WaitForRobot gates
        rp = _Replay(tree, home, completion_margin_s, interlock_move_time_s, home_return_time_s, prev, rids)
        t_exec = rp.run(root, 0.0)
        if prev is not None and prev.completed == rp.completed:
            break
        if not _has_kind(root, "WaitForRobot"):
            break
        prev = rp

    # step table: top-level children of the root sequence
    steps = []
    if node_kind(root) in ("Sequence", "SequenceWithMemory"):
        tcur = 0.0
        for i, c in enumerate(_children(root)):
            sub = _Replay(tree, home, completion_margin_s, interlock_move_time_s, home_return_time_s, rp, rids)
            sub.legs = [l for l in rp.legs]  # poses for synthetic moves
            t_next = sub.run(c, tcur)
            ops = [(port(n, "robot_ns"), port(n, "operation_id") or "home") for n in _leaves(c)]
            steps.append({"index": i, "kind": node_kind(c), "t_start": round(tcur, 4),
                          "t_end": round(t_next, 4), "ops": ops})
            tcur = t_next
    meta = {
        "source": str(path_or_dict) if isinstance(path_or_dict, (str, Path)) else "dict",
        "tree_id": env.get("tree_id") or tree.get("tree_id"),
        "cell_id": env.get("cell_id") or tree.get("cell_id"),
        "makespan_motion_s": round(max(l.t1 for l in rp.legs), 4),
        "executor_end_s": round(t_exec, 4),
        "umarell_schedule_makespan_s": round(_umarell_schedule_makespan(root, rp.bb), 4),
        "n_legs": len(rp.legs), "rules": EXECUTOR_RULES,
        "completion_margin_s": completion_margin_s, "steps": steps, "warnings": rp.warnings,
    }
    return Timeline.from_legs(rp.legs, dt=dt, t_end=t_exec if t_end is None else t_end,
                              home=home, meta=meta, robot_ids=rids)


def _has_kind(node, kind) -> bool:
    return node_kind(node) == kind or any(_has_kind(c, kind) for c in _children(node))


def _leaves(node) -> list:
    k = node_kind(node)
    if k in ("ExecuteTrajectory", "MoveHome", "WaitForRobot", "CheckSafety"):
        return [node]
    return [x for c in _children(node) for x in _leaves(c)]


def _umarell_schedule_makespan(node, bb, start=0.0) -> float:
    """Re-implementation of trajectory_quality._timeline (trajectory + hold, no margin)."""
    k = node_kind(node)
    kids = _children(node)
    if k == "Parallel":
        return max([start] + [_umarell_schedule_makespan(c, bb, start) for c in kids])
    if k == "Delay" or kids or k in ("Sequence", "SequenceWithMemory"):
        cur = start + (float(port(node, "delay_msec", 0) or 0) / 1000.0 if k == "Delay" else 0.0)
        for c in kids:
            cur = _umarell_schedule_makespan(c, bb, cur)
        return cur
    traj = resolve(port(node, "trajectory"), bb)
    base = 0.0
    if isinstance(traj, list) and len(traj) >= 2:
        base = float(traj[-1]["time_s"]) - float(traj[0]["time_s"])
    hold = float(port(node, "hold_duration_s", 0.0) or 0.0)
    return start + max(0.0, base) + max(0.0, hold)


# ─────────────────────────────────────────────────────────────────────────────
#  legs -> plan JSON (umarell's exact format)
# ─────────────────────────────────────────────────────────────────────────────
def _attr(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float, str)):
        return str(value)
    return json.dumps(value, separators=(",", ":"))


def _append_xml(parent, node):
    el = ET.SubElement(parent, node_kind(node))
    if node.get("name"):
        el.set("name", str(node["name"]))
    for k, v in (node.get("ports") or {}).items():
        if v is None or v == "":
            continue
        el.set(k, _attr(v))
    for c in _children(node):
        _append_xml(el, c)


def to_xml(tree: dict) -> str:
    """Byte-identical copy of btcpp.to_xml (include_model=True)."""
    root = ET.Element("root", {"BTCPP_format": "4"})
    main_id = tree.get("main_tree_to_execute") or "MainTree"
    root.set("main_tree_to_execute", main_id)
    bt_el = ET.SubElement(root, "BehaviorTree", {"ID": main_id})
    if isinstance(tree.get("root"), dict):
        _append_xml(bt_el, tree["root"])
    model = ET.SubElement(root, "TreeNodesModel")
    for nid, spec in LEAF_PORTS.items():
        el = ET.SubElement(model, spec.get("_kind", "Action"), {"ID": nid})
        for pname, pdefault in spec.items():
            if pname == "_kind":
                continue
            attrs = {"name": pname}
            if pdefault is not None and pdefault != "":
                attrs["default"] = _attr(pdefault)
            ET.SubElement(el, "input_port", attrs)
    raw = ET.tostring(root, encoding="unicode")
    pretty = minidom.parseString(raw).toprettyxml(indent="  ")
    return "\n".join(line for line in pretty.splitlines() if line.strip()) + "\n"


def _nid() -> str:
    return uuid.uuid4().hex[:8]


def _leaf_node(leg: Leg, key: str) -> dict:
    if leg.kind == "home":
        return {"type": "MoveHome", "name": f"home_{leg.robot}", "id": _nid(),
                "ports": {"robot_ns": leg.robot, "trajectory": "{" + key + "}"}}
    otype = leg.operation_type or ("weld" if leg.kind == "weld" else "move")
    return {"type": "ExecuteTrajectory", "name": f"{otype}_{leg.robot}_{leg.op_id}", "id": _nid(),
            "ports": {"robot_ns": leg.robot, "operation_id": leg.op_id, "operation_type": otype,
                      "trajectory": "{" + key + "}", "hold_duration_s": float(leg.hold_s),
                      "velocity_scale": float(leg.vscale), "has_interlock": False}}


def _order(robot_ids: Optional[Sequence[str]]) -> tuple:
    return ROBOT_IDS if robot_ids is None else tuple(robot_ids)


def group_steps(legs: Sequence[Leg], tol: float = 1e-6, robot_ids: Optional[Sequence[str]] = None) -> list[list[Leg]]:
    """Group legs into steps by identical start time (umarell's Parallel steps)."""
    rids = _order(robot_ids)
    legs = sorted([l for l in legs if not l.synthetic], key=lambda l: (l.t0, rids.index(l.robot)))
    steps: list[list[Leg]] = []
    for l in legs:
        if steps and abs(steps[-1][0].t0 - l.t0) <= tol and l.robot not in {x.robot for x in steps[-1]}:
            steps[-1].append(l)
        else:
            steps.append([l])
    return steps


def bt_plan_from_legs(cell_id: str, legs: Sequence[Leg], out_path=None, *, tree_id: Optional[str] = None,
                      steps: Optional[list] = None, created_s: Optional[float] = None,
                      metadata_extra: Optional[dict] = None,
                      robot_ids: Optional[Sequence[str]] = None) -> dict:
    """Write legs as a plan envelope in umarell's format (``plan_store.save_plan`` layout:
    ``tree_id, cell_id, created_s, status, num_operations, behaviour_tree{tree_id, cell_id,
    BTCPP_format, main_tree_to_execute, root, blackboard, metadata, xml}``).

    ``steps``: list of lists of legs (one leg per robot per step). Default: legs grouped by
    equal start time (``group_steps``). Each step becomes a ``Parallel`` (2 legs) or a bare
    leaf (1 leg); a leg that starts later than its step gets a ``Delay`` decorator, the
    way ``btcpp.to_btcpp`` encodes ``start_delay_s``. Legs of kind "home" become
    ``MoveHome`` leaves. Trajectories go into the blackboard as ``traj_<robot>_<op>`` lists
    of ``{positions, time_s, phase}`` (positions rounded to 6 decimals, time to 4, as in
    the saved plans). Legs must already be executor-consistent (see
    ``legs_from_rollout``); ``plan_timing_check`` reports any mismatch.
    """
    rids = _order(robot_ids)
    steps = group_steps(legs, robot_ids=rids) if steps is None else steps
    tree_id = tree_id or f"bt_{int(time.time())}_{cell_id}"
    bb: dict = {}
    children = []
    n_exec = 0
    for step in steps:
        t_step = min(l.t0 for l in step)
        nodes = []
        for leg in sorted(step, key=lambda l: rids.index(l.robot)):
            key = leg.bb_key or f"traj_{leg.robot}_{leg.op_id}"
            if key in bb:
                raise ValueError(f"duplicate blackboard key {key} (umarell refuses duplicate operation_ids)")
            bb[key] = leg.rel_trajectory()
            node = _leaf_node(leg, key)
            n_exec += node["type"] == "ExecuteTrajectory"
            delay = leg.t0 - t_step
            if delay > 5e-4:
                node = {"type": "Delay", "name": f"delay_{node['name']}",
                        "ports": {"delay_msec": int(round(delay * 1000))}, "children": [node]}
            nodes.append(node)
        if len(nodes) == 1:
            children.append(nodes[0])
        else:
            children.append({"type": "Parallel", "id": _nid(),
                             "ports": {"success_count": -1, "failure_count": 1}, "children": nodes})
    root = {"type": "Sequence", "id": _nid(), "ports": {}, "children": children}
    meta = {"steps": len(children), "operations": n_exec}
    if metadata_extra:
        meta.update(metadata_extra)
    tree = {"tree_id": tree_id, "cell_id": cell_id, "BTCPP_format": "4",
            "main_tree_to_execute": "MainTree", "root": root, "blackboard": bb, "metadata": meta}
    tree["xml"] = to_xml(tree)
    env = {"tree_id": tree_id, "cell_id": cell_id,
           "created_s": time.time() if created_s is None else created_s,
           "status": "ok", "num_operations": n_exec, "behaviour_tree": tree}
    if out_path is not None:
        out_path = Path(out_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "w") as f:
            json.dump(env, f, indent=2)
    return env


def plan_timing_check(legs: Sequence[Leg], plan: dict, robot_ids: Optional[Sequence[str]] = None) -> dict:
    """Compare the legs' intended start times with what the executor rules give for
    the exported plan. Returns max |dt0| and the per-leg list."""
    tl = timeline_from_bt_plan(plan, robot_ids=robot_ids)
    got = {(l.robot, l.op_id): l.t0 for l in tl.legs if not l.synthetic}
    rows = [(l.robot, l.op_id, l.t0, got.get((l.robot, l.op_id))) for l in legs if not l.synthetic]
    errs = [abs(a - b) for _, _, a, b in rows if b is not None]
    return {"max_start_error_s": max(errs) if errs else 0.0, "rows": rows,
            "missing": [r[:2] for r in rows if r[3] is None]}


def legs_from_rollout(t: np.ndarray, q: np.ndarray, cut_times: Sequence[float], *,
                      weld_arrivals: Sequence = (), hold_s: float = 0.0,
                      completion_margin_s: float = COMPLETION_MARGIN_S,
                      phase: Optional[np.ndarray] = None, home_last: bool = True,
                      robot_ids: Optional[Sequence[str]] = None) -> list[Leg]:
    """Cut a rollout (t (N,), q (N,R,6); two arms robot1 / robot2 unless ``robot_ids``) into executor-consistent
    synchronous steps.

    Both arms are cut at the same ``cut_times`` (e.g. every weld completion of either arm),
    so every step is a ``Parallel`` of two legs with equal duration. The executor then
    inserts ``hold_s + completion_margin_s`` between consecutive steps, during which BOTH
    arms stand still at their step-end poses; the pairs of configurations that occur at
    the same instant are exactly the rollout's, only paused. ``weld_arrivals``:
    ``[(robot, weld_id, t_arrive)]``; a robot's leg ending within one sample of its
    arrival gets ``kind="weld"`` and ``op_id=weld_id``. The returned legs carry the
    executor clock (``t0`` shifted by the inserted pauses). ``phase`` (N, 2) optional.
    The last step is exported as ``MoveHome`` legs if ``home_last``.
    """
    rids = _order(robot_ids)
    t = np.asarray(t, float)
    q = np.asarray(q, float).reshape(len(t), len(rids), 6)
    # snap cuts to sample indices so every leg starts and ends on a rollout sample
    idx = sorted({int(np.argmin(np.abs(t - c))) for c in cut_times if t[0] < c < t[-1]} - {0, len(t) - 1})
    bounds = [0] + idx + [len(t) - 1]
    legs: list[Leg] = []
    shift = 0.0
    for k in range(len(bounds) - 1):
        i0, i1 = bounds[k], bounds[k + 1]
        ts = t[i0:i1 + 1]
        a, b = float(ts[0]), float(ts[-1])
        last = k == len(bounds) - 2
        for r, rid in enumerate(rids):
            pos = q[i0:i1 + 1, r]
            ph = (np.asarray(phase)[i0:i1 + 1, r] if phase is not None else np.array(["transit"] * len(ts)))
            kind, op = "move", f"{rid.upper()}_S{k:02d}"
            for (wr, wid, ta) in weld_arrivals:      # the arm arrives at this weld inside the leg
                if wr == rid and (a < ta <= b + 1e-9 or (k == 0 and ta <= b + 1e-9)):
                    kind, op = "weld", str(wid)
            if last and home_last:
                kind, op = "home", "home"
            leg = Leg(robot=rid, op_id=op, kind=kind, t0=a + shift, t1=b + shift,
                      times=ts + shift, positions=pos.copy(), phase=np.asarray(ph, dtype="<U16"),
                      hold_s=0.0 if kind == "home" else hold_s, vscale=1.0,
                      operation_type="" if kind == "home" else ("weld" if kind == "weld" else "move"),
                      node_type="MoveHome" if kind == "home" else "ExecuteTrajectory",
                      bb_key=f"traj_{rid}_n{k}" if kind == "home" else "")
            leg.t_done = leg.t1 + leg.hold_s + completion_margin_s
            legs.append(leg)
        shift += (0.0 if last else hold_s) + completion_margin_s
    return legs


def default_timeline_path(plan_name: str) -> Path:
    root = Path(__file__).resolve().parents[1]
    return root / "data" / "timelines" / f"umarell_{plan_name}.npz"
