"""Strict common request contract. Parsing never silently edits the work list."""
from __future__ import annotations

from dataclasses import dataclass
import math
import time

from .store import digest


class InvalidRequest(ValueError):
    pass


def ids(value, name: str, universe: set[str], *, nonempty=False) -> tuple[str, ...]:
    if not isinstance(value, list) or any(not isinstance(x, str) for x in value):
        raise InvalidRequest(f"{name} must be a list of exact string IDs")
    if len(value) != len(set(value)):
        raise InvalidRequest(f"duplicate {name}")
    if set(value) - universe:
        raise InvalidRequest(f"unknown {name}: {sorted(set(value) - universe)}")
    if nonempty and not value:
        raise InvalidRequest(f"{name} must be nonempty")
    return tuple(sorted(value))


def finite_vector(value, n, name):
    if (not isinstance(value, list) or len(value) != n
            or any(isinstance(x, bool) or not isinstance(x, (int, float))
                   or not math.isfinite(x) for x in value)):
        raise InvalidRequest(f"{name} must have {n} finite numbers")
    return [float(x) for x in value]


@dataclass(frozen=True)
class Request:
    raw: dict
    selected: tuple[str, ...]
    pending: tuple[str, ...]
    completed: tuple[str, ...]
    enabled: tuple[str, ...]
    initial: dict
    program_id: str
    request_hash: str
    state_hash: str

    @classmethod
    def parse(cls, raw: dict, cell, *, now: float | None = None):
        if not isinstance(raw, dict) or raw.get("schema_version") != "weld-request-v1":
            raise InvalidRequest("schema_version must be weld-request-v1")
        allowed = {"schema_version", "request_id", "cell_id", "program_id", "weld_ids",
                   "enabled_robot_ids", "completed_weld_ids", "initial_state",
                   "terminal_state", "speed_profile", "scene_revision", "update"}
        if raw.keys() - allowed:
            raise InvalidRequest(f"unknown request fields: {sorted(raw.keys() - allowed)}")
        if not isinstance(raw.get("request_id"), str) or not raw["request_id"]:
            raise InvalidRequest("request_id is required")
        if raw.get("cell_id") != cell.cell_id:
            raise InvalidRequest("cell_id mismatch")
        pid = raw.get("program_id")
        if pid is None:
            if len(cell.programs) != 1:
                raise InvalidRequest("ambiguous program; program_id is required")
            pid = next(iter(cell.programs))
        if pid not in cell.programs:
            raise InvalidRequest("unknown program_id")
        if cell.program_errors[pid]:
            raise InvalidRequest("; ".join(cell.program_errors[pid]))
        operations = {o["id"] for o in cell.programs[pid]["operations"]}
        selected = ids(raw.get("weld_ids"), "weld_ids", operations)
        completed = ids(raw.get("completed_weld_ids", []), "completed_weld_ids", operations)
        if not set(completed) <= set(selected):
            raise InvalidRequest("completed_weld_ids must be a subset of weld_ids")
        order = cell.programs[pid].get("extensions", {}).get("strategy", {}).get("sequence_groups_order", [])
        if completed and order:
            ranks = {o["id"]: order.index(o["sequence_group"]) for o in cell.programs[pid]["operations"]}
            pending = set(selected) - set(completed)
            if pending and max(ranks[w] for w in completed) > min(ranks[w] for w in pending):
                raise InvalidRequest("completed work contradicts sequence-group order")
        enabled = ids(raw.get("enabled_robot_ids"), "enabled_robot_ids",
                      set(cell.robots), nonempty=True)
        if raw.get("speed_profile") != "cell_limits":
            raise InvalidRequest("only the native cell_limits profile is supported")
        if raw.get("terminal_state") not in {"home", "hold"}:
            raise InvalidRequest("terminal_state must explicitly be home or hold")
        if raw.get("scene_revision", cell.geometry_hash) != cell.geometry_hash:
            raise InvalidRequest("stale scene_revision")
        initial = raw.get("initial_state")
        if not isinstance(initial, dict):
            raise InvalidRequest("initial_state is required")
        if initial.get("mode") == "configured_home":
            if set(initial) != {"mode"}:
                raise InvalidRequest("configured_home takes no measured-state fields")
            initial = {"mode": "configured_home", "robots": {
                r: {"positions": [math.radians(x) for x in c["home_pose_deg"]],
                    "velocities": [0.] * len(c["home_pose_deg"])}
                for r, c in cell.robots.items()}}
        elif initial.get("mode") == "measured":
            if set(initial) != {"mode", "timestamp_s", "robots"}:
                raise InvalidRequest("measured state requires timestamp_s and all robots")
            stamp = initial.get("timestamp_s")
            if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
                    or not math.isfinite(stamp)):
                raise InvalidRequest("timestamp_s must be a finite Unix timestamp")
            age = (time.time() if now is None else now) - stamp
            if not -0.1 <= age <= 5:
                raise InvalidRequest(f"stale/future measured state ({age:.3f}s)")
            if not isinstance(initial["robots"], dict) or set(initial["robots"]) != set(cell.robots):
                raise InvalidRequest("positions and velocities required for every installed robot")
            for r, c in cell.robots.items():
                state = initial["robots"][r]
                if not isinstance(state, dict) or set(state) != {"positions", "velocities"}:
                    raise InvalidRequest(f"{r} requires positions and velocities")
                n = len(c["joint_limits_lower"])
                q = finite_vector(state["positions"], n, f"{r}.positions")
                v = finite_vector(state["velocities"], n, f"{r}.velocities")
                if any(x < lo or x > hi for x, lo, hi in zip(q, c["joint_limits_lower"], c["joint_limits_upper"])):
                    raise InvalidRequest(f"{r} initial position exceeds joint limits")
                if any(abs(x) > lim for x, lim in zip(v, c["joint_velocity_limits"])):
                    raise InvalidRequest(f"{r} initial velocity exceeds joint limits")
        else:
            raise InvalidRequest("unknown initial_state mode")
        update = raw.get("update")
        if update is not None:
            if (not isinstance(update, dict)
                    or set(update) != {"previous_version", "boundary_id", "acknowledged_state_hash"}
                    or initial["mode"] != "measured"
                    or any(not isinstance(x, str) or not x for x in update.values())):
                raise InvalidRequest("updates require version, acknowledged boundary and measured state")
            if update["acknowledged_state_hash"] != digest(initial):
                raise InvalidRequest("update does not match acknowledged state")
        normalized = {**raw, "program_id": pid, "weld_ids": selected,
                      "enabled_robot_ids": enabled, "completed_weld_ids": completed,
                      "initial_state": initial}
        return cls(raw, selected, tuple(x for x in selected if x not in completed),
                   completed, enabled, initial, pid, digest(normalized), digest(initial))
