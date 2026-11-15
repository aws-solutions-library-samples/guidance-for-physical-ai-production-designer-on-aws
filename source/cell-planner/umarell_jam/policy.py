"""Immutable candidate guidance with the ordinary planner as the release boundary."""
from __future__ import annotations

import time

from .request import Request
from .store import digest


def plan_with_policy(service, raw, immutable_cache, choose, *, timeout_s=30.):
    """Let a policy restrict robot/IK candidates, with deterministic fallback.

    ``choose`` receives ``{weld_id: [candidate, ...]}`` and returns the same
    mapping with one or more choices per weld. Each choice has robot_id and
    trajectory_hash. It cannot submit trajectories or execution certificates.
    The policy callback must honor its caller's latency budget.
    """
    started = time.perf_counter()
    info = {"fallback": False, "candidate_count": 0}
    result = None
    try:
        cell = service.load(raw["cell_id"])
        request = Request.parse(raw, cell)
        candidates = {}
        for weld in request.pending:
            candidates[weld] = []
            for candidate in immutable_cache.candidates(
                    cell.cell_id, request.program_id, weld, list(request.enabled)):
                if candidate["geometry_hash"] != cell.geometry_hash:
                    continue
                candidates[weld].append({
                    "robot_id": candidate["robot_id"], "weld_id": weld,
                    "trajectory_hash": digest(candidate["points"]),
                    "component_key": candidate["component_key"],
                    "motion_duration_s": candidate["points"][-1]["time_s"],
                    "end_positions": candidate["points"][-1]["positions"]})
            if not candidates[weld]:
                raise ValueError("immutable export has no matching candidate for " + weld)
        info["candidate_count"] = sum(map(len, candidates.values()))
        selection = choose(candidates)
        info["selection_s"] = time.perf_counter()-started
        remaining = max(0., timeout_s-info["selection_s"])
        result = service.plan(raw, timeout_s=remaining, candidate_filter=selection)
        info["attempt_status"] = result["status"]
        info["attempt_reason"] = result.get("reason")
    except Exception as exc:
        info["attempt_status"] = "invalid_guidance"
        info["attempt_reason"] = str(exc)
    if result is None or result["status"] != "success":
        info["fallback"] = True
        result = service.plan(raw, timeout_s=max(0., timeout_s-(time.perf_counter()-started)))
    elapsed = time.perf_counter()-started
    result["policy"] = info
    result["latency"]["policy_total_s"] = elapsed
    result["latency"]["elapsed_s"] = elapsed
    result["latency"]["time_to_first_validated_bt_s"] = elapsed if result["status"] == "success" else None
    return result
