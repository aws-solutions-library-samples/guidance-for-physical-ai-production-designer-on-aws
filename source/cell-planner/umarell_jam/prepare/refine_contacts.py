"""Reuse certified transits and independently certify smoother contact timing (session tools/refine_contacts.py)."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import multiprocessing as mp
from pathlib import Path
import sys
import time

from ..cell import Cell
from ..electrode_geometry import RefinedElectrodeGeometry
from ..native import setup, code_revision
from ..preparation import implementation_revision
from ..primitive_index import lookup
from ..contact_refinement import key, refine
from ..phases import phase_pack
from ..store import Store, atomic_json

MODELS = {}


def worker(layout, cid, pid, rid, weld, budget):
    layout.activate()
    if cid not in MODELS:
        cell = Cell(layout.workspace, cid)
        setup(layout.umarell, layout.workspace,
              layout.prepare_runs/"coverage-v2/refinement-workers", cid)
        MODELS[cid] = cell, RefinedElectrodeGeometry(cell)
    cell, geometry = MODELS[cid]
    store = Store(layout.components)
    planner, checker = code_revision(layout.umarell), implementation_revision()
    dep = key(cell, pid, rid, weld, planner, checker)
    existing = store.get("contact_refinement", dep)
    if existing and existing["status"] == "certified":
        return {"cell": cid, "robot": rid, "weld": weld, "status": "certified", "cache_hit": True}
    value, _ = lookup(store, cell, pid, rid, weld, planner, checker)
    if not value or value["status"] != "certified":
        return {"cell": cid, "robot": rid, "weld": weld, "status": "missing_primitive"}
    choices = []
    for variant in value["variants"]:
        item = {**variant, "robot": rid, "weld": weld}
        pack = phase_pack(cell, pid, item, store)
        choices.append((pack["points"]["approach"][-1]["time_s"], item))
    started = time.monotonic()
    good, failures = [], []
    for _, item in sorted(choices, key=lambda x: x[0]):
        if time.monotonic()-started >= budget:
            break
        candidate, report = refine(cell, geometry, pid, item, store, deadline=started+budget)
        print(json.dumps({"cell": cid, "robot": rid, "weld": weld, **report}), flush=True)
        if candidate:
            good.append(candidate)
            if len(good) >= 2:
                break
        else:
            failures.append(report)
    payload = {"cell": cid, "program": pid, "robot": rid, "weld": weld,
               "status": "certified" if good else "timeout_or_unknown",
               "variants": good, "failures": failures, "wall_s": time.monotonic()-started}
    store.obtain("contact_refinement", dep, lambda: payload)
    atomic_json(layout.prepare_runs/"coverage-v2/contact-refinement"/f"{cid}-{weld}-{rid}.json", payload)
    return {k: v for k, v in payload.items() if k not in {"variants", "failures"}}



def run(layout, cells=None, welds=None, workers=12, budget=180.):
    from .common import log_path, run_batches
    layout.activate()
    jobs = []
    store = Store(layout.components)
    planner, checker = code_revision(layout.umarell), implementation_revision()
    for cid in layout.cells(cells):
        cell = Cell(layout.workspace, cid)
        for pid, program in cell.programs.items():
            if cell.program_errors[pid]:
                continue
            for op in program["operations"]:
                if welds and op["id"] not in welds:
                    continue
                for rid in cell.robots:
                    value, _ = lookup(store, cell, pid, rid, op["id"], planner, checker)
                    if value and value["status"] == "certified":
                        jobs.append((layout, cell.cell_id, pid, rid, op["id"], budget))
    print(json.dumps({"jobs": len(jobs)}), flush=True)
    rows = run_batches(worker, jobs, workers, workers*2, log_path(layout, "coverage-v2/contact-refinement.jsonl"),
                       lambda job, exc: {"job": job[1:5], "status": "timeout_or_unknown", "error": repr(exc)})
    return {"jobs": len(jobs), "certified": sum(r.get("status") == "certified" for r in rows)}
