"""Bounded alternative transfer generation for the fastest certified branches (session tools/coverage_transfers.py)."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
import multiprocessing as mp
from pathlib import Path
import sys
import time

import numpy as np

from ..cell import Cell
from ..electrode_geometry import RefinedElectrodeGeometry
from ..native import setup, code_revision
from ..preparation import implementation_revision
from ..primitive_index import lookup
from ..phases import phase_pack
from ..transfers import key
from ..branch_routes import route
from ..connectors import certify_connector
from ..trajectory import Trajectory, limit_check
from ..store import Store, digest

MODELS = {}


def worker(layout, cid, a, b, planner, budget):
    layout.activate()
    if cid not in MODELS:
        cell = Cell(layout.workspace, cid)
        setup(layout.umarell, layout.workspace,
              layout.prepare_runs / "coverage-v2/transfer-workers", cid)
        MODELS[cid] = cell, RefinedElectrodeGeometry(cell)
    cell, geometry = MODELS[cid]
    store = Store(layout.components)
    dep = key(cell, a, b, planner)
    existing = store.get("transfer", dep)
    if existing and existing["status"] == "certified":
        return {"cell": cid, "robot": a["robot"], "from": a["weld"], "to": b["weld"],
                "status": "certified", "cache_hit": True}
    started = time.monotonic()
    points, search = route(cell, geometry, a["robot"], dep["start"], dep["goal"],
                           budget_s=min(12., budget/3), seed=int(digest(dep)[:8], 16))
    value = {"status": "timeout_or_unknown", "search": search}
    if points:
        old = (a["phase_pack"]["points"]["home"][-1]["time_s"]
               + b["phase_pack"]["points"]["approach"][-1]["time_s"])
        limits = limit_check(Trajectory(points), cell.robots[a["robot"]], .6)
        if points[-1]["time_s"] < old and limits["ok"]:
            states = {r: np.radians(c["home_pose_deg"]).tolist() for r, c in cell.robots.items()}
            states[a["robot"]] = dep["start"]
            cert = certify_connector(cell, geometry, a["robot"], points, states,
                                     deadline=started+budget)
            if cert["ok"]:
                value = {"status": "certified", "points": points, "certificate": cert,
                         "limits": limits, "search": search,
                         "build_wall_s": time.monotonic()-started,
                         "candidate_builder": hashlib.sha256(
                             Path(__file__).parents[1].joinpath("branch_routes.py").read_bytes()).hexdigest(),
                         "certification_builder": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
            else:
                value["certificate"] = cert
        else:
            value["reason"] = "no_transfer_time_saving_or_limit_failure"
    # Both candidate generators use the same transfer validity contract and
    # checker. Generation provenance is separate from the certified payload.
    store.obtain("transfer", dep, lambda: value)
    return {"cell": cid, "robot": a["robot"], "from": a["weld"], "to": b["weld"],
            "key": digest(dep), "wall_s": time.monotonic()-started,
            **{k: v for k, v in value.items() if k not in {"points", "certificate", "limits"}}}



def run(layout, cells=None, workers=12, budget=60., branches=2):
    from .common import log_path, run_batches
    layout.activate()
    store = Store(layout.components)
    planner, checker = code_revision(layout.umarell), implementation_revision()
    jobs, seen = [], set()
    for cid in layout.cells(cells):
        cell = Cell(layout.workspace, cid)
        for pid, program in cell.programs.items():
            if cell.program_errors[pid]:
                continue
            order = program["extensions"]["strategy"]["sequence_groups_order"]
            groups = {o["id"]: order.index(o["sequence_group"]) for o in program["operations"]}
            for rid in cell.robots:
                items = {}
                for op in program["operations"]:
                    value, _ = lookup(store, cell, pid, rid, op["id"], planner, checker)
                    if not value or value["status"] != "certified":
                        continue
                    candidates = []
                    for variant in value["variants"]:
                        item = {**variant, "robot": rid, "weld": op["id"]}
                        item["phase_pack"] = phase_pack(cell, pid, item, store)
                        candidates.append(item)
                    unique = {}
                    for item in sorted(candidates, key=lambda x: sum(
                            p[-1]["time_s"] for p in x["phase_pack"]["points"].values())):
                        endpoint = tuple(item["phase_pack"]["points"]["contact"][0]["positions"])
                        unique.setdefault(endpoint, item)
                    items[op["id"]] = list(unique.values())[:branches]
                for wa, first in items.items():
                    for wb, second in items.items():
                        if wa == wb or groups[wa] > groups[wb]:
                            continue
                        for a in first:
                            for b in second:
                                dep = key(cell, a, b, planner)
                                address = digest(dep)
                                value = store.get("transfer", dep)
                                if address not in seen and (not value or value["status"] != "certified"):
                                    jobs.append((layout, cell.cell_id, a, b, planner, budget))
                                    seen.add(address)
    print(json.dumps({"jobs": len(jobs), "workers": workers}), flush=True)
    rows = run_batches(worker, jobs, workers, workers*2, log_path(layout, "coverage-v2/transfers.jsonl"),
                       lambda job, exc: {"cell": job[1], "from": job[2]["weld"], "to": job[3]["weld"],
                                         "status": "timeout_or_unknown", "error": repr(exc)})
    return {"jobs": len(jobs), "certified": sum(r.get("status") == "certified" for r in rows)}
