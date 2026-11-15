"""Isolated, hard-bounded native routing for a blocked measured rest connector."""
from __future__ import annotations

import json
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np

from .connectors import certify_connector, revision as checker_revision
from .store import atomic_json
from .revision import source_sha256
from .trajectory import retime_native, limit_check, Trajectory


def worker(request_file, result_file):
    from .native import setup
    spec = json.loads(Path(request_file).read_text())
    sys.path.insert(0, spec["umarell"])
    setup(Path(spec["umarell"]), Path(spec["workspace"]), Path(result_file).parent / "scratch", spec["cell"])
    from umarell_fast import workers
    from agents.trajectory.tools import planner_backend
    workers.reset_session(spec["cell"])
    workers.inject(spec["cell"], spec["robot"], spec["start"], None)
    points = planner_backend.plan_home_trajectory(spec["cell"], spec["robot"], spec["start"], spec["goal"],
                                                 velocity_scale=spec["speed"])
    atomic_json(Path(result_file), {"points": points})


def repair_start(service, cell, request, geometry, budget_s, deadline):
    if any(max(map(abs, s["velocities"])) > 1e-8 for s in request.initial["robots"].values()):
        return {"ok": False, "reason": "native_connector_repair_requires_acknowledged_rest"}
    states = {r: list(s["positions"]) for r, s in request.initial["robots"].items()}
    if geometry.check(states):
        return {"ok": False, "reason": "initial_scene_collision"}
    home = {r: np.radians(c["home_pose_deg"]).tolist() for r, c in cell.robots.items()}
    batches, certificates = [], []
    start = time.monotonic()
    deadline = min(deadline, start + budget_s)
    for rid in request.enabled:
        if np.max(abs(np.array(states[rid]) - home[rid])) <= 1e-8:
            continue
        remaining = min(budget_s - (time.monotonic() - start), deadline - time.monotonic())
        if remaining <= 0:
            return {"ok": False, "reason": "native_connector_repair_deadline"}
        speed = min(cell.config["planning"]["max_velocity_scaling"], cell.config["planning"]["home_velocity_scale"])
        dep = {"geometry": cell.geometry_hash, "limits": cell.limits_hash, "robot": rid,
               "states": {r: list(q) for r, q in states.items()}, "goal": home[rid],
               "checker": checker_revision(), "router": source_sha256(Path(__file__))}
        cached = service.store.get("connector_route", dep)
        if cached is not None and cached.get("status") == "certified":
            batches.append([{"robot": rid, "points": cached["points"], "phase": "repaired_start_home",
                             "certificate": cached["certificate"]}])
            certificates.append(cached["certificate"])
            states[rid] = home[rid]
            continue
        with tempfile.TemporaryDirectory(prefix="weld-connector-") as directory:
            folder = Path(directory)
            atomic_json(folder / "request.json", {"umarell": str(service.umarell),
                "workspace": str(service.workspace), "cell": cell.cell_id, "robot": rid,
                "start": states[rid], "goal": home[rid], "speed": speed})
            with (folder / "planner.log").open("w") as log:
                own_group = os.environ.get("WELD_CONNECTOR_PARENT_GROUP") != "1"
                process = subprocess.Popen([sys.executable, "-m", "umarell_jam.route_connectors",
                    str(folder / "request.json"), str(folder / "result.json")],
                    env={**os.environ, "PYTHONPATH": str(Path(__file__).resolve().parents[1])},
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=own_group)
                try:
                    process.wait(timeout=remaining)
                except subprocess.TimeoutExpired:
                    if own_group:
                        os.killpg(process.pid, signal.SIGKILL)
                    else:
                        process.kill()
                    process.wait()
                    return {"ok": False, "reason": "native_connector_routing_timeout"}
            if process.returncode != 0 or not (folder / "result.json").exists():
                return {"ok": False, "reason": "native_connector_worker_failed",
                        "detail": (folder / "planner.log").read_text()[-2000:]}
            raw = json.loads((folder / "result.json").read_text())["points"]
        if not raw:
            return {"ok": False, "reason": "native_connector_search_exhausted"}
        points = retime_native(raw, cell.robots[rid], speed)
        limits = limit_check(Trajectory(points), cell.robots[rid], speed)
        if not limits["ok"]:
            return {"ok": False, "reason": "native_connector_limits", "detail": limits}
        cert = certify_connector(cell, geometry, rid, points, states, deadline=deadline)
        if not cert["ok"]:
            return cert
        stored, _ = service.store.obtain("connector_route", dep,
            lambda: {"status": "certified", "points": points, "certificate": cert})
        points, cert = stored["points"], stored["certificate"]
        batches.append([{"robot": rid, "points": points, "phase": "repaired_start_home",
                         "certificate": cert}])
        certificates.append(cert)
        states[rid] = home[rid]
    return {"ok": True, "batches": batches, "states": states, "certificates": certificates,
            "native_repair_s": time.monotonic() - start}


if __name__ == "__main__":
    worker(*sys.argv[1:])
