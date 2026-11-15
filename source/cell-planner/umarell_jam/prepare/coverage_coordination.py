"""Prepare continuous pair proofs for primitives and directed transfers (session tools/coverage_coordination.py)."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from itertools import combinations
import json
import multiprocessing as mp
from pathlib import Path
import sys

from ..cell import Cell
from ..geometry import Geometry
from ..native import code_revision
from ..preparation import implementation_revision
from ..primitive_index import lookup
from ..coordination_cache import accepts, dependencies, prepare
from ..store import Store, digest

MODELS = {}


def worker(layout, cid, a, b, budget):
    layout.activate()
    if cid not in MODELS:
        cell = Cell(layout.workspace, cid)
        MODELS[cid] = cell, Geometry(cell)
    cell, geometry = MODELS[cid]
    value, hit = prepare(cell, geometry, a, b, Store(layout.components), budget)
    return {"cell": cid, "a": [a["robot"], a["weld"]], "b": [b["robot"], b["weld"]],
            "cache_hit": hit, **value}



def run(layout, cells=None, workers=12, budget=2.):
    from .common import log_path, run_batches
    layout.activate()
    store = Store(layout.components)
    planner, checker = code_revision(layout.umarell), implementation_revision()
    transfers = [json.loads(p.read_text()) for p in (layout.components/"transfer").glob("*.json")]
    jobs, seen = [], set()
    for cid in layout.cells(cells):
        cell = Cell(layout.workspace, cid)
        for pid, program in cell.programs.items():
            if cell.program_errors[pid]:
                continue
            group = {o["id"]: o["sequence_group"] for o in program["operations"]}
            items = []
            for op in program["operations"]:
                for rid in cell.robots:
                    value, _ = lookup(store, cell, pid, rid, op["id"], planner, checker)
                    if not value or value["status"] != "certified":
                        continue
                    items.extend({**x, "robot": rid, "weld": op["id"],
                                  "group": group[op["id"]], "kind": "primitive"} for x in value["variants"])
            for doc in transfers:
                dep, value = doc["dependencies"], doc["payload"]
                if (dep["geometry"] == cell.geometry_hash and value["status"] == "certified"
                        and dep["to_weld"] in group):
                    items.append({**value, "robot": dep["robot"],
                                  "weld": dep["from_weld"]+"->"+dep["to_weld"],
                                  "group": group[dep["to_weld"]], "kind": "transfer"})
            for a, b in combinations(items, 2):
                if a["robot"] == b["robot"] or a["group"] != b["group"]:
                    continue
                if a["kind"] == b["kind"] == "primitive" and a["weld"] == b["weld"]:
                    continue
                dep = dependencies(cell, a, b)
                address = digest(dep)
                if address in seen:
                    continue
                seen.add(address)
                if accepts(cell, a, b, store):
                    continue
                old = store.get("coordination", dep)
                if old and old.get("status") == "conflict":
                    continue
                jobs.append((layout, cell.cell_id, a, b, budget))
    print(json.dumps({"pair_jobs": len(jobs), "budget_s": budget}), flush=True)
    rows = run_batches(worker, jobs, workers, workers*16, log_path(layout, "coverage-v2/coordination.jsonl"),
                       lambda job, exc: {"cell": job[1], "a": job[2]["weld"], "b": job[3]["weld"],
                                         "status": "timeout_or_unknown", "error": repr(exc)})
    return {"jobs": len(jobs), "statuses": dict(__import__("collections").Counter(r.get("status") for r in rows))}
