"""Versioned pending-work replacement at executor-acknowledged rest boundaries."""
from __future__ import annotations

import copy
import threading
import time

from .request import InvalidRequest
from .store import digest


class ExecutionSession:
    """Wrap a real Umarell runtime; queue() never interrupts committed motion or pulse.

    A runtime must implement the native motion/feedback interface and acknowledged
    weld_phase(). The same class is used with SimulatedRuntime in acceptance tests.
    """
    def __init__(self, service, runtime, monitor, config):
        self.service, self.runtime, self.monitor, self.config = service, runtime, monitor, config
        self.lock = threading.RLock()
        self.pending = None
        self.sequence = 0
        self.current = None
        self.replacement = None
        self.completed = set()
        self.updates = []

    def queue(self, request):
        with self.lock:
            if self.current is not None and request.get("cell_id") != self.current["behaviour_tree"]["cell_id"]:
                raise InvalidRequest("execution replacement must refer to the same installed cell")
            self.sequence += 1
            self.pending = self.sequence, copy.deepcopy(request)
            return self.sequence

    def run(self, plan):
        from agents.cell.bt_executor import BehaviourTreeExecutor
        self.current = copy.deepcopy(plan)
        self.completed = set(plan.get("completed_weld_ids", []))
        for _ in range(65):
            self.service.commit(self.current)
            executor = BehaviourTreeExecutor(self, self.monitor, self.config)
            outcome = executor.execute(self.current["behaviour_tree"])
            if outcome.get("failure", {}).get("reason_code") == "pending_work_replaced" and self.replacement:
                self.current, self.replacement = self.replacement, None
                continue
            return {**outcome, "version_hash": self.current["version_hash"],
                    "completed_weld_ids": sorted(self.completed), "updates": self.updates}
        return {"success": False, "failure": {"reason_code": "update_limit"},
                "completed_weld_ids": sorted(self.completed), "updates": self.updates}

    def __getattr__(self, name):
        return getattr(self.runtime, name)

    def weld_phase(self, robot, weld, phase, edge, duration, settings):
        acknowledged = self.runtime.weld_phase(robot, weld, phase, edge, duration, settings)
        if acknowledged and phase == "weld" and edge == "end":
            with self.lock:
                self.completed.add(weld)
        return acknowledged

    def weld_boundary(self, boundary_id):
        with self.lock:
            item = copy.deepcopy(self.pending)
            completed = sorted(self.completed)
        cell = self.service.load(self.current["behaviour_tree"]["cell_id"])
        robots = {}
        for rid in cell.robots:
            status = self.runtime.latest_status(rid) or {}
            q, v = status.get("joint_positions"), status.get("joint_velocities")
            if q is None or v is None or any(abs(x) > 1e-8 for x in v):
                return False
            robots[rid] = {"positions": list(q), "velocities": list(v)}
        state = {"mode": "measured", "timestamp_s": time.time(), "robots": robots}
        version = self.current["version_hash"]
        self.service.acknowledge_boundary(version, boundary_id, state, completed)
        if item is None:
            return True
        sequence, raw = item
        # Completion is acknowledged, never inferred from elapsed time.
        raw["completed_weld_ids"] = completed
        raw["initial_state"] = state
        raw["update"] = {"previous_version": version, "boundary_id": boundary_id,
                         "acknowledged_state_hash": digest(state)}
        candidate = self.service.plan(raw)
        with self.lock:
            if self.pending is None or self.pending[0] != sequence:
                self.updates.append({"sequence": sequence, "status": "superseded_at_boundary"})
                return True
            self.pending = None
            self.updates.append({"sequence": sequence, "status": candidate["status"],
                                 "reason": candidate.get("reason"), "version_hash": candidate.get("version_hash"),
                                 "boundary_id": boundary_id, "completed_weld_ids": completed})
            if candidate["status"] == "success":
                self.replacement = candidate
                return {"replace_pending": True}
        # An unsuccessful search leaves the committed remaining work intact.
        return True
