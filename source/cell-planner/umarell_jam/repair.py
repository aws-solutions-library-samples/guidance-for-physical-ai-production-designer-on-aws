"""Hard-bounded repair of only the missing robot/weld components in an isolated process."""
from __future__ import annotations

import os
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


def repair(service, request, missing, budget_s):
    if budget_s <= .05:
        return {"status": "timeout_or_unknown", "reason": "repair_budget_exhausted"}
    folder = service.output.parent / "repair-logs"
    folder.mkdir(parents=True, exist_ok=True)
    token = f"{request.request_hash}-{uuid.uuid4().hex}"
    log = folder / f"{token}.log"
    spec_path, receipt = folder / f"{token}.spec.json", folder / f"{token}.result.json"
    from .store import atomic_json
    cell = service.load(request.raw["cell_id"])
    atomic_json(spec_path, {"workspace": str(service.workspace), "cache": str(service.store.root),
                          "umarell": str(service.umarell), "cell": cell.cell_id,
                          "geometry": cell.geometry_hash, "program": request.program_id,
                          "robots": list(request.enabled), "welds": list(missing),
                          "budget_s": budget_s, "scratch": str(folder / token),
                          "receipt": str(receipt)})
    # A fixed argv, no shell: this interpreter, this package's worker module and the spec file written above under
    # the service's own output folder (the request reaches the worker only as that file's JSON). opengrep / semgrep
    # p/ci's Django subprocess-injection rule still taints cmd (on the Popen and on each TimeoutExpired(cmd, ...)):
    # a false positive, suppressed on those lines (reviewed 2026-11-15).
    cmd = [sys.executable, "-m", "umarell_jam.repair_worker", str(spec_path)]
    env = {**os.environ, "PYTHONPATH": str(Path(__file__).resolve().parents[1])}
    start = time.perf_counter()
    next_scene_check = start
    with log.open("w") as stream:
        # nosemgrep: python.django.security.injection.command.subprocess-injection.subprocess-injection
        worker = subprocess.Popen(cmd, stdout=stream, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        try:
            while worker.poll() is None:
                from agents.trajectory.tools import plan_cancellation
                if plan_cancellation.is_cancelled():
                    # nosemgrep: python.django.security.injection.command.subprocess-injection.subprocess-injection
                    raise subprocess.TimeoutExpired(cmd, budget_s)
                if time.perf_counter() >= next_scene_check:
                    if not cell.unchanged():
                        # nosemgrep: python.django.security.injection.command.subprocess-injection.subprocess-injection
                        raise subprocess.TimeoutExpired(cmd, budget_s)
                    next_scene_check = time.perf_counter()+1.
                if time.perf_counter() - start >= budget_s:
                    # nosemgrep: python.django.security.injection.command.subprocess-injection.subprocess-injection
                    raise subprocess.TimeoutExpired(cmd, budget_s)
                time.sleep(.05)
            status = "completed" if worker.returncode == 0 else "timeout_or_unknown"
        except subprocess.TimeoutExpired:
            os.killpg(worker.pid, signal.SIGTERM)
            try:
                worker.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                os.killpg(worker.pid, signal.SIGKILL)
                worker.wait()
            status = "timeout_or_unknown"
    details = json.loads(receipt.read_text()) if receipt.exists() else {}
    if details.get("status") not in {None, "running"}:
        status = details["status"]
    return {"status": status, "wall_s": time.perf_counter() - start, "budget_s": budget_s,
            "affected_weld_ids": missing, "log": str(log), "whole_cache_rebuild": False,
            "details": details, "receipt": str(receipt), "worker_returncode": worker.returncode}
