"""Prepare phase/rest pair certificates on full programs, never whole-plan hits (session tools/coverage_phase_prepare.py).

One full request per cell and valid program (every weld, every installed arm, from and to HOME) is planned with
``prepare_phase_pairs=True``: the bounded phase search publishes its pair certificates as components. The plan
itself is not cached.
"""
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import multiprocessing as mp

from ..service import PlannerService
from ..store import atomic_json


def worker(layout, cid, timeout):
    layout.activate()
    service = PlannerService(layout.workspace, layout.components,
                             layout.umarell, layout.prepare_runs/"coverage-v2/phase-preparation-plans")
    cell = service.load(cid)
    rows = []
    for pid in [p for p in cell.programs if not cell.program_errors[p]]:
        request = {"schema_version": "weld-request-v1", "request_id": "phase-preparation-"+cid,
                   "cell_id": cid, "program_id": pid, "enabled_robot_ids": list(cell.robots),
                   "weld_ids": [o["id"] for o in cell.programs[pid]["operations"]],
                   "completed_weld_ids": [], "initial_state": {"mode": "configured_home"},
                   "terminal_state": "home", "speed_profile": "cell_limits"}
        result = service.plan(request, timeout_s=timeout, prepare_phase_pairs=True)
        name = cid + ".json" if len(cell.programs) == 1 else f"{cid}--{pid}.json"
        atomic_json(layout.prepare_runs/"coverage-v2/phase-preparation"/name,
                    {"request": request, "result": result})
        rows.append({"cell": cid, "program": pid, **{k: result.get(k) for k in (
            "status", "reason", "timing", "route_search", "phase_search", "latency")}})
    return rows


def run(layout, cells=None, workers=4, timeout=1200.):
    layout.activate()
    rows = []
    with ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn")) as pool:
        futures = [pool.submit(worker, layout, cid, timeout) for cid in layout.cells(cells)]
        for future in as_completed(futures):
            for row in future.result():
                print(json.dumps(row), flush=True)
                rows.append(row)
    return {"requests": len(rows), "success": sum(r.get("status") == "success" for r in rows)}
