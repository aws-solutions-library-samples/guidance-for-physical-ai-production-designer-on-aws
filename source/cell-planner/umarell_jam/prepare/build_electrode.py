"""Bounded additional candidates using the refined, unchanged electrode surface (session tools/build_electrode.py)."""
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
from ..electrode_geometry import RefinedElectrodeGeometry, revision
from ..native import setup, code_revision, prepare_candidate, motion_request
from ..preparation import dependencies, implementation_revision
from ..primitive_index import lookup
from ..store import Store, digest
from ..trajectory import retime_native
from ..validation import certify

MODELS = {}


def worker(layout, cid, pid, rid, weld, budget, max_poses):
    layout.activate()
    from agents.trajectory.tools.weld_orientation import candidates, policy
    from umarell_fast import workers as W
    started = time.monotonic()
    if cid not in MODELS:
        cell = Cell(layout.workspace, cid)
        setup(layout.umarell, layout.workspace, layout.prepare_runs / "electrode-workers", cid)
        MODELS[cid] = cell, RefinedElectrodeGeometry(cell)
    cell, geometry = MODELS[cid]
    planner, implementation = code_revision(layout.umarell), implementation_revision()
    store = Store(layout.components)
    base = dependencies(cell, pid, rid, weld, planner, implementation)
    dep = {"base": base, "electrode_refinement": revision()}
    cfg = cell.robots[rid]
    op = next(o for o in cell.programs[pid]["operations"] if o["id"] == weld)
    params = cell.programs[pid]["extensions"]["weld_params"]

    def generate():
        failures, good, examined = [], [], 0
        for spin, tilt, direction, ori in [(0., 0., 0., op["orientation"])] + candidates(op["orientation"], policy(cfg)):
            if time.monotonic() - started >= budget:
                break
            if geometry.target_tool_check(rid, op["position"], ori):
                continue
            pose = {"spin": spin, "tilt": tilt, "tilt_direction": direction, "orientation": ori}
            rawdep = {**dep, "pose": pose}
            raw, _ = store.obtain("refined_native", rawdep, lambda: prepare_candidate(
                cell, pid, rid, op, [pose], planner_revision=planner, geometry=geometry))
            examined += 1
            if raw["status"] == "candidate":
                leg = raw["leg"]
                W.reset_session(cid)
                W.inject(cid, rid, leg["end"], leg.get("last_pose"),
                         {weld: leg.get("weld_pose")}, {weld: leg.get("op_spec")})
                points = retime_native(leg["traj"], cfg, .6)
                cert = certify(cell, geometry, rid, op, points,
                               deadline=min(started + budget, time.monotonic() + 180.))
                if cert["ok"]:
                    cert.update(checker_version="full-scene-electrode-subdivision-v1",
                                electrode_refinement=revision(), refinement=geometry.refinement_counts[rid])
                    good.append({"points": points, "certificate": cert, "pose": pose})
                else:
                    failures.append({"pose": pose, "certificate": cert})
            else:
                failures.append({"pose": pose, "native": raw})
            if len(good) >= 2 or examined >= max_poses:
                break
        return {"status": "certified" if good else "timeout_or_unknown", "variants": good,
                "cell": cid, "program": pid, "robot": rid, "weld": weld, "failures": failures,
                "attempts": examined, "build_wall_s": time.monotonic() - started,
                "refinement": geometry.refinement_counts[rid]}
    payload, hit = store.obtain("refined_primitive", dep, generate)
    return {k: v for k, v in payload.items() if k not in {"variants", "failures"}} | {
        "cache_hit": hit, "key": digest(dep)}



def run(layout, cells=("body_weld_1",), welds=("W01",), workers=2, budget=600., poses=16, missing=False):
    """``missing``: build only welds with no certified installed-robot option."""
    from .common import log_path, run_batches
    layout.activate()
    jobs = []
    store = Store(layout.components)
    planner, implementation = code_revision(layout.umarell), implementation_revision()
    for cid in cells:
        cell = Cell(layout.workspace, cid)
        for pid, program in cell.programs.items():
            if cell.program_errors[pid]:
                continue
            for rid in cell.robots:
                for op in program["operations"]:
                    if missing and any((lookup(store, cell, pid, robot, op["id"], planner, implementation)[0]
                                        or {}).get("status") == "certified" for robot in cell.robots):
                        continue
                    if missing or not welds or op["id"] in welds:
                        jobs.append((layout, cid, pid, rid, op["id"], budget, poses))
    rows = run_batches(worker, jobs, workers, workers * 4, log_path(layout, "electrode-build.jsonl"),
                       lambda job, exc: {"job": job[1:], "status": "timeout_or_unknown", "exception": repr(exc)})
    return {"jobs": len(jobs), "certified": sum(r.get("status") == "certified" for r in rows)}
