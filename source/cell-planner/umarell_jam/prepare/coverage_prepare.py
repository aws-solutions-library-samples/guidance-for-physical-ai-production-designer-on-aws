"""Resumable parallel branch discovery and strict certification for real cells (session tools/coverage_prepare.py)."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import multiprocessing as mp
from pathlib import Path
import sys
import time

from ..cell import Cell
from ..native import code_revision
from ..preparation import implementation_revision
from ..primitive_index import lookup, supplemental_key
from ..store import Store


def worker(job):
    from .coverage_probe import probe
    from .certify_branches import build
    layout, cid, pid, rid, weld, budget, variants = job
    layout.activate()
    cell = Cell(layout.workspace, cid)
    store = Store(layout.components)
    key = supplemental_key(cell, pid, rid, weld, code_revision(layout.umarell),
                            implementation_revision())
    existing = store.get("primitive_supplement", key)
    if existing and existing["status"] == "certified":
        return {"cell": cid, "robot": rid, "weld": weld, "status": "certified", "cache_hit": True}
    path = layout.prepare_runs / "coverage-v2/probes" / f"{cid}-{weld}-{rid}.json"
    if path.exists():
        diagnostic = json.loads(path.read_text())
    else:
        diagnostic = None
    if not diagnostic or diagnostic.get("geometry") != cell.geometry_hash or not diagnostic.get("solutions"):
        diagnostic = probe(layout, cid, weld, rid, min(90., budget/3), 16, path)
    if not diagnostic["solutions"]:
        return {k: diagnostic[k] for k in ("cell", "robot", "weld", "status", "counts", "wall_s")}
    return build(layout, path, budget, variants, supplemental=True)



def run(layout, cells=None, welds=None, robots=None, workers=8, budget=180., variants=2, eligible_only=False):
    """``eligible_only``: improve already certified pairs and the nearest arm for missing welds."""
    import numpy as np
    from .common import log_path
    layout.activate()
    store = Store(layout.components)
    planner, checker = code_revision(layout.umarell), implementation_revision()
    jobs = []
    for cid in layout.cells(cells):
        cell = Cell(layout.workspace, cid)
        for pid, program in cell.programs.items():
            if cell.program_errors[pid]:
                continue
            for op in program["operations"]:
                if welds and op["id"] not in welds:
                    continue
                nearest = min(cell.robots, key=lambda r: np.linalg.norm(
                    np.asarray(cell.robots[r]["base_position"])-op["position"]))
                for rid in cell.robots:
                    if robots and rid not in robots:
                        continue
                    if eligible_only and rid != nearest and (
                            lookup(store, cell, pid, rid, op["id"], planner, checker)[0] or {}).get("status") != "certified":
                        continue
                    jobs.append((layout, cell.cell_id, pid, rid, op["id"], budget, variants))
    logfile = log_path(layout, "coverage-v2/preparation.jsonl")
    rows = []
    with logfile.open("a") as log:
        # Recycle whole pools to bound native model memory without replacement-worker deadlocks.
        for start in range(0, len(jobs), workers):
            with ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn")) as pool:
                pending = {pool.submit(worker, job): job for job in jobs[start:start+workers]}
                for future in as_completed(pending):
                    try:
                        result = future.result()
                    except Exception as exc:
                        result = {"job": pending[future][1:5], "status": "timeout_or_unknown", "error": repr(exc)}
                    result["recorded_at"] = time.time()
                    log.write(json.dumps(result)+"\n")
                    log.flush()
                    print(json.dumps(result), flush=True)
                    rows.append(result)
    return {"jobs": len(jobs), "certified": sum(r.get("status") == "certified" for r in rows)}
