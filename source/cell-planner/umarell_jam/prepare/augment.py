"""Resumable staged wrist-branch search, using the same frozen release certificates (session tools/augment.py)."""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
import math
import multiprocessing as mp
from pathlib import Path
import time

import numpy as np

from ..cell import Cell
from ..geometry import Geometry
from ..native import setup, code_revision, motion_request
from ..preparation import dependencies, implementation_revision
from ..store import Store, digest
from ..trajectory import retime_native
from ..validation import certify
from ..release import stroke_proof

MODELS = {}


def worker(layout, cid, pid, rid, weld, budget, max_poses, variants):
    layout.activate()
    from umarell_fast import workers as W
    from agents.trajectory.tools.weld_orientation import candidates, policy
    started = time.monotonic()
    if cid not in MODELS:
        cell = Cell(layout.workspace, cid)
        setup(layout.umarell, layout.workspace, layout.prepare_runs / "augment-workers", cid)
        MODELS[cid] = cell, Geometry(cell)
    cell, geometry = MODELS[cid]
    store = Store(layout.components)
    planner, checker = code_revision(layout.umarell), implementation_revision()
    dep = dependencies(cell, pid, rid, weld, planner, checker)
    present = store.get("primitive", dep)
    if present and present["status"] == "certified":
        return {"cell": cid, "robot": rid, "weld": weld, "status": "certified", "cache_hit": True,
                "key": digest(dep)}
    cfg = cell.robots[rid]
    home = np.radians(cfg["home_pose_deg"]).tolist()
    op = next(o for o in cell.programs[pid]["operations"] if o["id"] == weld)
    params = cell.programs[pid]["extensions"]["weld_params"]
    failures, good = [], []
    poses = []
    for spin, tilt, direction, ori in [(0., 0., 0., op["orientation"])] + candidates(op["orientation"], policy(cfg)):
        if geometry.target_tool_check(rid, op["position"], ori) is None:
            poses.append({"spin": spin, "tilt": tilt, "tilt_direction": direction, "orientation": ori})
    # Round-robin tilt directions/spin signs so a bounded search does not spend
    # its entire budget on adjacent poses from one region of the orientation grid.
    buckets = {}
    for pose in poses:
        buckets.setdefault((pose["tilt"], pose["tilt_direction"], pose["spin"] >= 0), []).append(pose)
    diverse = []
    while any(buckets.values()):
        for bucket in buckets.values():
            if bucket:
                diverse.append(bucket.pop(len(bucket)//2))
    seeds = [home]
    for q3, q4, q5 in ((0, 90, 0), (0, -90, 0), (90, 0, 0), (-90, 0, 0),
                       (180, 0, 180), (-180, 0, -180), (90, 90, 90), (-90, -90, -90)):
        q = np.array(home)
        q[3:] += np.radians([q3, q4, q5])
        q = np.clip(q, cfg["joint_limits_lower"], cfg["joint_limits_upper"])
        seeds.append(q.tolist())
    attempts = 0
    for pose in diverse[:max_poses]:
        if time.monotonic() - started >= budget:
            break
        req = motion_request(op, rid, pose, params)
        # Staging explicitly uses another IK branch; it never treats the seed as
        # the robot's start pose. Native HOME-to-stage planning is still required.
        for seed_index, seed in enumerate(seeds):
            if time.monotonic() - started >= budget:
                break
            W.reset_session(cid)
            W.inject(cid, rid, home, None)
            stroke = W.stroke_from(cid, rid, seed, req, params["approach_offset_z"])
            attempts += 1
            if not stroke.get("clear"):
                failures.append({"pose": pose, "seed": seed_index, "reason": stroke.get("why")})
                continue
            states = {r: np.radians(c["home_pose_deg"]).tolist() for r, c in cell.robots.items()}
            states[rid] = stroke["end_q"]
            hit = geometry.check(states, {rid: op["position"]})
            if hit:
                failures.append({"pose": pose, "seed": seed_index, "endpoint_violation": hit})
                continue
            rawdep = {"base": dep, "pose": pose, "seed": seed, "method": "native_stage_v1"}
            raw = store.get("staged_native", rawdep)
            if raw is None:
                result = W.plan_chain(cid, [
                    {"ns": rid, "op_id": weld + "_stage", "start": home,
                     "req": {"stage_for": req, "seeds": [seed]}},
                    {"ns": rid, "op_id": weld, "req": req}],
                    {"approach_offset_m": params["approach_offset_z"], "velocity_scale": .6})
                raw = {"status": "candidate" if result.get("ok") else "timeout_or_unknown", "result": result}
                store.obtain("staged_native", rawdep, lambda: raw)
            if raw["status"] != "candidate":
                failures.append({"pose": pose, "seed": seed_index, "native": raw["result"]})
                continue
            legs = raw["result"]["legs"]
            stage, stroke_points = legs[0]["traj"], legs[1]["traj"]
            offset = stage[-1]["time_s"]
            points = [{**p, "phase": "transit"} for p in stage]
            points += [{**p, "time_s": p["time_s"] + offset} for p in stroke_points[1:]]
            W.reset_session(cid)
            leg = legs[-1]
            W.inject(cid, rid, leg["end"], leg.get("last_pose"),
                     {weld: leg.get("weld_pose")}, {weld: leg.get("op_spec")})
            points = retime_native(points, cfg, .6)
            cert = certify(cell, geometry, rid, op, points,
                           deadline=min(started + budget, time.monotonic() + 180.))
            item = {"points": points, "certificate": cert, "pose": pose,
                    "branch_seed": seed, "method": "native_stage_v1"}
            if cert["ok"]:
                proof = stroke_proof(cell, {**item, "robot": rid, "weld": weld}, store)
                if proof["ok"]:
                    good.append(item)
                else:
                    failures.append({"pose": pose, "seed": seed_index, "release": proof})
            else:
                failures.append({"pose": pose, "seed": seed_index, "certificate": cert})
            if len(good) >= variants:
                break
        if len(good) >= variants:
            break
    payload = {"status": "certified" if good else "timeout_or_unknown", "variants": good,
               "failures": failures, "robot": rid, "weld": weld, "cell": cid, "program": pid,
               "attempts": attempts, "build_wall_s": time.monotonic() - started,
               "search_code": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    store.obtain("primitive", dep, lambda: payload)
    return {k: v for k, v in payload.items() if k not in {"variants", "failures"}} | {
        "certified_variants": len(good), "key": digest(dep)}


def unfinished(layout, jobs, logfile):
    """The jobs whose pair has no logged attempt for its *current* component key.

    A pair is skipped only when the log holds a row with the key the job would build now: digest of its dependencies
    (the cell's geometry / limits / program keys, the planner and checker revisions, ...). Keying the resume on
    (cell, robot, weld) alone skipped every pair of a cell whose geometry or program had changed since the logged run
    (a moved robot, a re-run Prepare Cell), so augment ran no jobs and left the new geometry without its variants.
    Rows without a key (older logs, a worker exception) never skip a job."""
    attempted = set()
    for line in logfile.read_text().splitlines():
        try:
            row = json.loads(line)
        except ValueError:
            continue
        if row.get("key"):
            attempted.add(row["key"])
    if not attempted:
        return list(jobs)
    planner, checker = code_revision(layout.umarell), implementation_revision()
    cells = {}
    keep = []
    for job in jobs:
        _, cid, pid, rid, weld = job[:5]
        if cid not in cells:
            cells[cid] = Cell(layout.workspace, cid)
        if digest(dependencies(cells[cid], pid, rid, weld, planner, checker)) not in attempted:
            keep.append(job)
    return keep


def run(layout, cells=None, workers=16, budget=900., poses=24, variants=2, welds=None, robots=None, resume=False):
    """``resume``: skip completed pairs from this run's log, retaining their explicit unknown statuses."""
    from .common import log_path, run_batches
    layout.activate()
    jobs = []
    for cid in layout.cells(cells):
        cell = Cell(layout.workspace, cid)
        for pid, program in cell.programs.items():
            if cell.program_errors[pid]:
                continue
            for op in program["operations"]:
                if welds and op["id"] not in welds:
                    continue
                # Nearby arms first; all arms remain installed collision bodies.
                for rid in sorted(cell.robots, key=lambda r: np.linalg.norm(
                        np.asarray(cell.robots[r]["base_position"]) - op["position"])):
                    if not robots or rid in robots:
                        jobs.append((layout, cell.cell_id, pid, rid, op["id"], budget, poses, variants))
    logfile = log_path(layout, "augmentation.jsonl")
    if resume and logfile.exists():
        jobs = unfinished(layout, jobs, logfile)
    rows = run_batches(worker, jobs, workers, workers * 4, logfile,
                       lambda job, exc: {"job": job[1:5], "status": "timeout_or_unknown", "exception": repr(exc)})
    from ..manifest import export_manifest
    export_manifest(layout.workspace, layout.components, layout.umarell, layout.manifest)
    return {"jobs": len(jobs), "certified": sum(r.get("status") == "certified" for r in rows)}
