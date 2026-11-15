"""Prepare pair compatibility only for candidate motions in the same sequence group (session tools/build_coordination.py)."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import itertools
import json
import multiprocessing as mp
from pathlib import Path
import sys

from ..cell import Cell
from ..geometry import Geometry
from ..native import code_revision
from ..preparation import dependencies, implementation_revision
from ..primitive_index import lookup
from ..store import Store
from ..coordination_cache import prepare

MODELS = {}


def worker(layout, cid, a, b, budget):
    layout.activate()
    if cid not in MODELS:
        cell = Cell(layout.workspace, cid)
        MODELS[cid] = cell, Geometry(cell)
    cell, geometry = MODELS[cid]
    value, hit = prepare(cell, geometry, a, b, Store(layout.components), budget)
    return {"cell": cid, "a": [a["robot"], a["weld"], a["certificate"]["trajectory_hash"]],
            "b": [b["robot"], b["weld"], b["certificate"]["trajectory_hash"]],
            "cache_hit": hit, **value}



def run(layout, cells=None, workers=4, budget=20.):
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
            groups = {}
            for op in program["operations"]:
                for rid in cell.robots:
                    value, _ = lookup(store, cell, pid, rid, op["id"], planner, implementation)
                    if value and value["status"] == "certified":
                        for variant in value["variants"]:
                            groups.setdefault(op["sequence_group"], []).append({
                                **variant, "robot": rid, "weld": op["id"]})
            for values in groups.values():
                for a, b in itertools.combinations(values, 2):
                    if a["robot"] != b["robot"] and a["weld"] != b["weld"]:
                        jobs.append((layout, cell.cell_id, a, b, budget))
    rows = run_batches(worker, jobs, workers, max(1, len(jobs)), log_path(layout, "coordination.jsonl"),
                       lambda job, exc: {"job": (job[1], job[2]["weld"], job[3]["weld"]),
                                         "status": "timeout_or_unknown", "reason": repr(exc)})
    return {"jobs": len(jobs), "statuses": dict(__import__("collections").Counter(r.get("status") for r in rows))}
