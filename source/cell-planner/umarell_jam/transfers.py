"""Directed, branch-specific transfers between retracted weld poses."""
from __future__ import annotations

import hashlib
from pathlib import Path
import time

import numpy as np

from .connectors import certify_connector, revision as connector_revision
from .store import digest
from .trajectory import retime_native, limit_check, Trajectory
from .validation import compatible


def key(cell, a, b, planner):
    return {"geometry": cell.geometry_hash, "limits": cell.limits_hash, "robot": a["robot"],
            "start": a["phase_pack"]["points"]["retract"][-1]["positions"],
            "goal": b["phase_pack"]["points"]["contact"][0]["positions"],
            "from_weld": a["weld"], "to_weld": b["weld"], "planner": planner,
            "checker": connector_revision(),
            "builder": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


def build(cell, geometry, a, b, store, planner, budget=180.):
    from umarell_fast import workers
    from agents.trajectory.tools import planner_backend
    dep = key(cell, a, b, planner)
    def generate():
        started = time.monotonic()
        rid = a["robot"]
        workers.reset_session(cell.cell_id)
        workers.inject(cell.cell_id, rid, dep["start"], None)
        raw = planner_backend.plan_home_trajectory(cell.cell_id, rid, dep["start"], dep["goal"],
                                                   velocity_scale=.6)
        if not raw:
            return {"status": "timeout_or_unknown", "reason": "native_transfer_search_exhausted"}
        points = retime_native(raw, cell.robots[rid], .6)
        states = {r: np.radians(c["home_pose_deg"]).tolist() for r, c in cell.robots.items()}
        states[rid] = dep["start"]
        cert = certify_connector(cell, geometry, rid, points, states, deadline=started + budget)
        limits = limit_check(Trajectory(points), cell.robots[rid], .6)
        if not cert["ok"] or not limits["ok"]:
            return {"status": "timeout_or_unknown", "certificate": cert, "limits": limits}
        return {"status": "certified", "points": points, "certificate": cert,
                "limits": limits, "build_wall_s": time.monotonic() - started}
    return store.obtain("transfer", dep, generate)


def optimize_adjacent(cell, batches, store, planner):
    """Use a transfer only with a sufficient all-clock proof across both batches."""
    used = []
    margin = cell.config["collision"]["min_clearance"]
    boundaries = []
    for batch in batches:
        chains = {}
        for item in batch:
            chains.setdefault(item["robot"], []).append(item)
        for chain in chains.values():
            for a, b in zip(chain, chain[1:]):
                boundaries.append(([a], [b], batch))
    for previous, following in zip(batches, batches[1:]):
        last = {item["robot"]: item for item in previous}
        first = {}
        for item in following:
            first.setdefault(item["robot"], item)
        boundaries.append((list(last.values()), list(first.values()), following))
    all_accepted = []
    for previous, following, active in boundaries:
        candidates = []
        for a in previous:
            for b in following:
                if a["robot"] != b["robot"]:
                    continue
                dep = key(cell, a, b, planner)
                transfer = store.get("transfer", dep)
                if not transfer or transfer["status"] != "certified":
                    continue
                cert = transfer["certificate"]
                if (not cert.get("ok") or cert.get("geometry_hash") != cell.geometry_hash
                        or cert.get("trajectory_hash") != digest(transfer["points"])):
                    continue
                old_time = a["phase_pack"]["points"]["home"][-1]["time_s"] + b["phase_pack"]["points"]["approach"][-1]["time_s"]
                if transfer["points"][-1]["time_s"] >= old_time:
                    continue
                candidates.append((a, b, transfer, old_time - transfer["points"][-1]["time_s"]))
        # After the boundary only the following batch can be moving. The
        # transfer certificate includes the parked starting configuration.
        certificates = [(x["robot"], x["certificate"]) for x in active]
        for a, b, transfer, saving in sorted(candidates, key=lambda v: -v[-1]):
            cert = transfer["certificate"]
            if any(r != a["robot"] and not compatible(cert, other, margin) for r, other in certificates):
                continue
            if any(r != a["robot"] and not compatible(cert, other, margin) for r, other in all_accepted):
                continue
            a["skip_home"] = True
            b["transfer"] = transfer
            all_accepted.append((a["robot"], cert))
            used.append({"robot": a["robot"], "from_weld": a["weld"], "to_weld": b["weld"],
                         "saved_motion_s": saving, "certificate_hash": digest(cert)})
    return used
