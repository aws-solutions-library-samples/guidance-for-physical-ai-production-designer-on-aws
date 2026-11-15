"""Umarell integration: reuse its backend, chaining records and collision fixes."""
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import time

from .store import Store, digest


def code_revision(root: Path):
    # Execution/BT presentation changes do not invalidate geometric components.
    # Their separate revision belongs to the final plan manifest.
    return digest({str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                   for directory in ("agents/trajectory/backends", "agents/trajectory/tools", "utils")
                   for p in sorted((root / directory).rglob("*.py"))
                   if p.name not in {"btcpp.py", "behaviour_tree.py", "coordinated_tree.py", "plan_store.py"}})


def setup(umarell: Path, workspace: Path, scratch: Path, cell_id: str):
    from umarell_fast.u2 import session
    env = session.env_for(umarell, cell_id, workspace / "cells", scratch, {
        "OMPL_PARALLEL_CANDIDATES": "0", "UMARELL_MODEL_CACHE": "1", "PLAN_STEP_GATE": "0",
        "OMPL_TIME_BUDGET_S": "1.5", "OMPL_SEED": "1949"})
    session.setup(umarell, cell_id, env, kernels="none", seed=1949)
    from umarell_fast.workers import install_kernels
    install_kernels("x76")


def motion_request(op, rid, pose, params):
    return {"robot_ns": rid, "operation_id": op["id"], "operation_type": "weld",
            "target": dict(zip("xyz", op["position"])),
            "approach_vector": op["approach_vector"], "approach": True,
            "approach_offset_m": params["approach_offset_z"],
            "retreat_offset_m": params["retract_offset_z"],
            "approach_velocity_scale": next(
                (p["speed_percentage"] / 100 for p in params["weld_phases"]
                 if p["phase"] == "contact" and "speed_percentage" in p), 0.1),
            "orientation": pose["orientation"], "constrain_orientation": True,
            "free_spin": False, "applied_tool_spin_deg": pose["spin"]}


def prepare_candidate(cell, program_id, rid, op, poses, *, planner_revision, geometry=None):
    """HOME-to-weld geometric candidates; return is unknown, never infeasible, on exhaustion."""
    import math
    from umarell_fast import workers
    from .geometry import Geometry

    geom = geometry or Geometry(cell)
    home = {r: [math.radians(x) for x in c["home_pose_deg"]] for r, c in cell.robots.items()}
    params = cell.programs[program_id]["extensions"]["weld_params"]
    failures = []
    started = time.perf_counter()
    for pose in poses:
        req = motion_request(op, rid, pose, params)
        res = workers.plan_chain(cell.cell_id, [
            {"ns": rid, "op_id": op["id"], "start": home[rid], "req": req}],
            {"approach_offset_m": params["approach_offset_z"], "velocity_scale": .6})
        if not res.get("ok"):
            failures.append({"pose": pose, "native": res})
            continue
        leg = res["legs"][0]
        # Endpoints are necessary checks; dense/controller release remains separate.
        states = {**home, rid: leg["end"]}
        violation = geom.check(states, {rid: op["position"]})
        if violation:
            failures.append({"pose": pose, "endpoint_violation": violation})
            continue
        return {"status": "candidate", "robot": rid, "weld": op["id"], "pose": pose,
                "leg": leg, "planner_revision": planner_revision,
                "wall_s": time.perf_counter() - started, "attempts": len(failures) + 1}
    return {"status": "timeout_or_unknown", "robot": rid, "weld": op["id"],
            "failures": failures, "wall_s": time.perf_counter() - started}
