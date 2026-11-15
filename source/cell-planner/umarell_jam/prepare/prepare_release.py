"""Prepare controller phase packs and stroke proofs, without caching any whole request (session tools/prepare_release.py)."""
from concurrent.futures import ProcessPoolExecutor, as_completed
import argparse
import json
import multiprocessing as mp
from pathlib import Path
import sys
import time

from ..cell import Cell
from ..native import code_revision
from ..preparation import implementation_revision
from ..primitive_index import lookup
from ..phases import phase_pack
from ..release import stroke_proof
from ..store import Store, atomic_json, digest


def worker(layout, cid):
    layout.activate()
    started = time.perf_counter()
    cell = Cell(layout.workspace, cid)
    planner, implementation = code_revision(layout.umarell), implementation_revision()
    store = Store(layout.components)
    rows = []
    for pid, program in cell.programs.items():
        if cell.program_errors[pid]:
            continue
        for op in program["operations"]:
            for rid in cell.robots:
                entry, dep = lookup(store, cell, pid, rid, op["id"], planner, implementation)
                if not entry or entry["status"] != "certified":
                    continue
                for index, variant in enumerate(entry["variants"]):
                    row = {"cell": cid, "program": pid, "robot": rid, "weld": op["id"],
                           "variant": index, "component_key": digest(dep),
                           "trajectory_hash": variant["certificate"]["trajectory_hash"]}
                    item = {**variant, "robot": rid, "weld": op["id"]}
                    try:
                        phases = phase_pack(cell, pid, item, store)
                        stroke = stroke_proof(cell, item, store, pid)
                        row.update(status="ready" if stroke["ok"] else "timeout_or_unknown",
                                   stroke=stroke, phase_checks=phases["checks"],
                                   durations={p: points[-1]["time_s"] for p, points in phases["points"].items()})
                    except Exception as exc:
                        row.update(status="timeout_or_unknown", reason=repr(exc))
                    rows.append(row)
    return {"cell": cid, "wall_s": time.perf_counter()-started, "variants": rows}



def run(layout, cells=None, workers=5):
    layout.activate()
    cells = layout.cells(cells)
    rows = []
    with ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn")) as pool:
        futures = [pool.submit(worker, layout, cid) for cid in cells]
        for future in as_completed(futures):
            row = future.result()
            rows.append(row)
            print(json.dumps({"cell": row["cell"], "wall_s": row["wall_s"],
                              "ready": sum(v["status"] == "ready" for v in row["variants"]),
                              "variants": len(row["variants"])}), flush=True)
    atomic_json(layout.prepare_runs / "release-coverage.json", rows)
    return {"cells": len(rows), "ready": sum(v["status"] == "ready" for r in rows for v in r["variants"]),
            "variants": sum(len(r["variants"]) for r in rows)}
