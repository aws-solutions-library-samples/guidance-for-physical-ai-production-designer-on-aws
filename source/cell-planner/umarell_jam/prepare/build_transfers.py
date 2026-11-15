"""Prepare bounded directed transfers between certified branches; no subset plans (session tools/build_transfers.py)."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import multiprocessing as mp
from pathlib import Path
import sys

import numpy as np

from ..cell import Cell
from ..geometry import Geometry
from ..native import setup, code_revision
from ..preparation import dependencies, implementation_revision
from ..primitive_index import lookup
from ..phases import phase_pack
from ..store import Store
from ..transfers import build, key

MODELS = {}


def worker(layout, cid, pid, rid, wa, wb, budget, resume, branch_policy):
    layout.activate()
    if cid not in MODELS:
        cell = Cell(layout.workspace, cid)
        setup(layout.umarell, layout.workspace, layout.prepare_runs / "transfer-workers", cid)
        MODELS[cid] = cell, Geometry(cell)
    cell, geometry = MODELS[cid]
    store = Store(layout.components)
    planner, implementation = code_revision(layout.umarell), implementation_revision()
    items = []
    for weld in (wa, wb):
        primitive, dep = lookup(store, cell, pid, rid, weld, planner, implementation)
        if not primitive or primitive["status"] != "certified":
            return {"cell": cid, "robot": rid, "from": wa, "to": wb, "status": "missing_primitive"}
        variants = primitive["variants"] if branch_policy == "fastest" else primitive["variants"][:1]
        choices = []
        for variant in variants:
            item = {**variant, "robot": rid, "weld": weld}
            item["phase_pack"] = phase_pack(cell, pid, item, store)
            choices.append(item)
        item = min(choices, key=lambda x: sum(points[-1]["time_s"]
                                              for points in x["phase_pack"]["points"].values()))
        items.append(item)
    value = store.get("transfer", key(cell, *items, planner)) if resume else None
    if value is None:
        value, hit = build(cell, geometry, *items, store, planner, budget)
    else:
        hit = True
    return {"cell": cid, "robot": rid, "from": wa, "to": wb, "status": value["status"],
            "branch_policy": branch_policy,
            "cache_hit": hit, "build_wall_s": value.get("build_wall_s"), "reason": value.get("reason"),
            "certificate": {k: v for k, v in value.get("certificate", {}).items() if k != "occupancy"}}



def run(layout, cells=None, workers=8, budget=180., resume=False, branch_policy="first"):
    """``resume``: keep the current bounded attempt's unknown records (default: retry them).
    ``branch_policy``: "first" (the first available branch) or "fastest" (the fastest retimed branch)."""
    from .common import log_path, run_batches
    layout.activate()
    store = Store(layout.components)
    planner, implementation = code_revision(layout.umarell), implementation_revision()
    jobs = []
    for cid in layout.cells(cells):
        cell = Cell(layout.workspace, cid)
        for pid, program in cell.programs.items():
            if cell.program_errors[pid]:
                continue
            for rid in cell.robots:
                eligible = [op for op in program["operations"] if
                            (lookup(store, cell, pid, rid, op["id"], planner, implementation)[0]
                             or {}).get("status") == "certified"]
                for a in eligible:
                    for b in eligible:
                        if a != b and np.linalg.norm(np.array(a["position"]) - b["position"]) <= 2.:
                            jobs.append((layout, cell.cell_id, pid, rid, a["id"], b["id"], budget,
                                         resume, branch_policy))
    # Bound retained model memory with explicit pool batches. This also avoids
    # relying on worker replacement while many futures are already queued.
    rows = run_batches(worker, jobs, workers, workers * 4, log_path(layout, "transfers.jsonl"),
                       lambda job, exc: {"job": job[1:], "status": "timeout_or_unknown", "reason": repr(exc)})
    return {"jobs": len(jobs), "certified": sum(r.get("status") == "certified" for r in rows)}
