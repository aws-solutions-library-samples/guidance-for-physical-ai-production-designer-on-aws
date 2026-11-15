"""Isolate native session setup while certifying reusable exact-state connectors."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace

from .store import Store, atomic_json
from .revision import source_sha256


def prepare(service, cell, request, *, deadline, repair_budget_s=0.):
    directory = Path(__file__).parent
    dep = {"geometry": cell.geometry_hash, "limits": cell.limits_hash,
           "program": cell.program_keys[request.program_id], "enabled": request.enabled,
           "completed": request.completed, "robots": request.initial["robots"],
           "return_home": bool(request.pending or request.raw["terminal_state"] == "home"),
           "planner": service.revision, "implementation": service.implementation,
           "checkers": {name: source_sha256(directory / name)
                        for name in ("initial_connection.py", "connectors.py", "route_connectors.py",
                                     "electrode_geometry.py", "phases.py", "primitive_index.py")}}
    cached = service.store.get("initial_connection", dep)
    if cached is not None:
        return {**cached, "cache_hit": True}
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        return {"ok": False, "reason": "initial_connection_deadline"}
    with tempfile.TemporaryDirectory(prefix="weld-initial-") as directory:
        folder = Path(directory)
        atomic_json(folder / "request.json", {
            "workspace": str(service.workspace), "umarell": str(service.umarell),
            "cache": str(service.store.root), "raw": request.raw,
            "accepted_at": request.initial.get("timestamp_s", time.time()),
            "deadline": deadline, "repair_budget_s": repair_budget_s,
            "planner": service.revision, "implementation": service.implementation})
        with (folder / "worker.log").open("w") as log:
            process = subprocess.Popen([sys.executable, "-m", "umarell_jam.initial_connection",
                                        str(folder / "request.json"), str(folder / "result.json")],
                env={**os.environ, "PYTHONPATH": str(Path(__file__).resolve().parents[1])},
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            from agents.trajectory.tools import plan_cancellation
            while process.poll() is None:
                cancelled = plan_cancellation.is_cancelled()
                remaining = deadline - time.monotonic()
                if cancelled or remaining <= 0:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                    return {"ok": False, "reason": "cancelled" if cancelled else "initial_connection_worker_timeout"}
                try:
                    process.wait(timeout=min(.1, remaining))
                except subprocess.TimeoutExpired:
                    pass
        if process.returncode or not (folder / "result.json").exists():
            return {"ok": False, "reason": "initial_connection_worker_failed",
                    "detail": (folder / "worker.log").read_text()[-3000:]}
        value = json.loads((folder / "result.json").read_text())
    if value.get("ok"):
        return service.store.obtain("initial_connection", dep, lambda: value)[0]
    return value


def worker(input_file, output_file):
    spec = json.loads(Path(input_file).read_text())
    sys.path.insert(0, spec["umarell"])
    from .cell import Cell
    from .request import Request
    from .native import setup
    from .electrode_geometry import RefinedElectrodeGeometry
    from .connectors import prepare_start
    cell = Cell(spec["workspace"], spec["raw"]["cell_id"])
    request = Request.parse(spec["raw"], cell, now=spec["accepted_at"])
    setup(Path(spec["umarell"]), Path(spec["workspace"]), Path(output_file).parent / "scratch", cell.cell_id)
    geometry, store = RefinedElectrodeGeometry(cell), Store(spec["cache"])
    value = prepare_start(cell, request, store, geometry, deadline=spec["deadline"],
                          planner_revision=spec["planner"], implementation=spec["implementation"])
    if not value["ok"] and spec["repair_budget_s"] > 0:
        from .route_connectors import repair_start
        os.environ["WELD_CONNECTOR_PARENT_GROUP"] = "1"
        service = SimpleNamespace(workspace=Path(spec["workspace"]), umarell=Path(spec["umarell"]), store=store)
        value = repair_start(service, cell, request, geometry, spec["repair_budget_s"], spec["deadline"])
    atomic_json(Path(output_file), value)


if __name__ == "__main__":
    worker(*sys.argv[1:])
