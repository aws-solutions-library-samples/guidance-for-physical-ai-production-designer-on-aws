"""Process acknowledgement hook and simulated execution using Umarell's real executor."""
from __future__ import annotations

import threading
import time
import json
import numpy as np


def initial_velocity(executor, node, n):
    from agents.trajectory.tools.btcpp import port, resolve
    value = port(node, "initial_velocity", None)
    if isinstance(value, str) and value.startswith("["):
        value = json.loads(value)
    else:
        value = resolve(value, executor._bb)
    if value is None:
        return [0.] * n
    if not isinstance(value, list) or len(value) != n or not np.all(np.isfinite(value)):
        raise ValueError("invalid initial velocity feedback gate")
    return value


def execute_phase(executor, node):
    from agents.trajectory.tools.btcpp import port, resolve
    rid, weld = port(node, "robot_ns"), port(node, "weld_id")
    phase, duration = port(node, "phase"), float(port(node, "duration_s", -1))
    if phase not in {"squeeze", "weld", "hold"} or duration < 0:
        return False, {"reason_code": "invalid_weld_phase", "failed_node_id": node.get("id")}
    action = getattr(executor._rt, "weld_phase", None)
    if action is None:
        return False, {"reason_code": "weld_process_driver_missing", "failed_node_id": node.get("id")}
    settings = port(node, "settings")
    if isinstance(settings, str) and settings.startswith('{"'):
        settings = json.loads(settings)
    else:
        settings = resolve(settings, executor._bb)
    if not isinstance(settings, dict):
        return False, {"reason_code": "weld_settings_missing"}
    ack = action(rid, weld, phase, "begin", duration, settings)
    if not ack:
        return False, {"reason_code": "weld_phase_unacknowledged", "failed_node_id": node.get("id")}
    # Motion stays at rest throughout all three phases. A replacement request
    # cannot run here; installation is allowed only at acknowledged rest boundaries.
    failure = executor._interruptible_sleep(duration)
    ack = action(rid, weld, phase, "abort" if failure else "end", duration, settings)
    if failure or not ack:
        return False, failure or {"reason_code": "weld_phase_unacknowledged"}
    executor._executed += 1
    return True, None


def execute_boundary(executor, node):
    from agents.trajectory.tools.btcpp import port
    action = getattr(executor._rt, "weld_boundary", None)
    if action is None:
        return False, {"reason_code": "boundary_acknowledgement_driver_missing"}
    acknowledgement = action(port(node, "boundary_id"))
    if not acknowledgement:
        return False, {"reason_code": "boundary_acknowledgement_failed"}
    executor._executed += 1
    if isinstance(acknowledgement, dict) and acknowledgement.get("replace_pending"):
        return False, {"reason_code": "pending_work_replaced"}
    return True, None


class SimulatedRuntime:
    def __init__(self, cell, *, speed=40., jitter_s=0., slowdown=1., initial_state=None,
                 pause_s=0., boundary_callback=None):
        self.cell = cell
        self.speed, self.jitter, self.slowdown = speed, jitter_s, slowdown
        self.started = time.monotonic()
        self.lock = threading.RLock()
        self.active = {}
        self.initial = {r: np.radians(c["home_pose_deg"]).tolist() for r, c in cell.robots.items()}
        self.initial_velocity = {r: [0.] * len(q) for r, q in self.initial.items()}
        if initial_state:
            self.initial = {r: list(s["positions"]) for r, s in initial_state["robots"].items()}
            self.initial_velocity = {r: list(s["velocities"]) for r, s in initial_state["robots"].items()}
        self.pause_s, self.boundary_callback = pause_s, boundary_callback
        self.completed, self.boundaries = [], []
        self.events, self.estops = [], []

    def now(self):
        return (time.monotonic() - self.started) * self.speed

    def robots(self):
        return [("arms", r) for r in self.cell.robots]

    def dispatch(self, robot_id, task):
        from utils.fjt_trajectory import with_derivatives
        with self.lock:
            points = task["trajectory"]
            if not task.get("controller_exact"):
                points = with_derivatives(points)
            self.active[robot_id] = (self.now() + self.jitter, points)
            self.events.append({"time_s": self.now(), "robot": robot_id,
                                "kind": "motion", "operation": task["operation_id"]})

    def latest_status(self, robot_id):
        from utils.fjt_trajectory import sample
        with self.lock:
            if robot_id in self.active:
                t, points = self.active[robot_id]
                elapsed = max(0., self.now() - t) / self.slowdown
                q, v, _ = sample(points, elapsed, "time_s")
                v = [x / self.slowdown for x in v]
            else:
                q, v = self.initial[robot_id], self.initial_velocity[robot_id]
            return {"joint_positions": q, "joint_velocities": v, "ee_position": [0, 0, 0],
                    "trajectory_time_s": elapsed if robot_id in self.active else 0.}

    def weld_phase(self, robot, weld, phase, edge, duration, settings):
        from .phases import process_settings
        if not any(settings == process_settings(p) for p in self.cell.programs.values()):
            return False
        with self.lock:
            self.events.append({"time_s": self.now(), "robot": robot, "weld": weld,
                                "kind": "process", "phase": phase, "edge": edge,
                                "duration_s": duration, "settings": settings})
            if phase == "weld" and edge == "end":
                self.completed.append(weld)
        return True

    def weld_boundary(self, boundary_id):
        states = {r: {"positions": self.latest_status(r)["joint_positions"],
                      "velocities": self.latest_status(r)["joint_velocities"]} for r in self.cell.robots}
        if any(max(map(abs, s["velocities"])) > 1e-8 for s in states.values()):
            return False
        measured = {"mode": "measured", "timestamp_s": time.time(), "robots": states}
        self.boundaries.append({"boundary_id": boundary_id, "state": measured,
                                "completed_weld_ids": list(self.completed)})
        if self.boundary_callback:
            self.boundary_callback(boundary_id, measured, list(self.completed))
        # A controlled pause occurs at an acknowledged rest boundary. Emergency
        # stops remain aborts and need a newly validated measured-state request.
        if self.pause_s:
            time.sleep(self.pause_s / self.speed)
        return True

    def estop(self, reason):
        self.estops.append(reason)


class CertificateMonitor:
    """For simulation of a tree already certified for independent arm clocks."""
    def imminent(self):
        return None


def simulate(cell, tree, *, jitter_s=0., slowdown=1., pause_s=0., boundary_callback=None):
    from agents.cell.bt_executor import BehaviourTreeExecutor, ExecConfig
    from agents.trajectory.tools.btcpp import to_xml, from_xml
    from .store import digest
    metadata = tree.get("metadata", {})
    if (metadata.get("scene_revision") != cell.geometry_hash or not cell.unchanged()
            or metadata.get("program_revision") != digest(cell.program_keys.get(metadata.get("program_id")))):
        return {"success": False, "failure": {"reason_code": "stale_scene"},
                "process_completed": [], "events": [], "estops": []}
    parsed = from_xml(to_xml(tree), blackboard=tree["blackboard"])
    parsed["cell_id"] = tree["cell_id"]
    parsed["metadata"] = tree.get("metadata", {})
    runtime = SimulatedRuntime(cell, jitter_s=jitter_s, slowdown=slowdown,
                               initial_state=tree.get("metadata", {}).get("initial_state"),
                               pause_s=pause_s, boundary_callback=boundary_callback)
    cfg = ExecConfig(tick_hz=1000., completion_margin_s=0., start_pose_tolerance_rad=.001,
                     arrival_tolerance_rad=.001, arrival_timeout_s=60.,
                     schedule_deviation_rad=.1, schedule_deviation_grace_s=2.,
                     schedule_check_warmup_s=0., cell_id=cell.cell_id)
    result = BehaviourTreeExecutor(runtime, CertificateMonitor(), cfg).execute(parsed)
    result["events"] = runtime.events
    result["estops"] = runtime.estops
    result["boundaries"] = runtime.boundaries
    result["final_positions"] = {r: runtime.latest_status(r)["joint_positions"] for r in cell.robots}
    result["final_velocities"] = {r: runtime.latest_status(r)["joint_velocities"] for r in cell.robots}
    result["process_completed"] = [e["weld"] for e in runtime.events
                                   if e.get("phase") == "weld" and e.get("edge") == "end"]
    return result
