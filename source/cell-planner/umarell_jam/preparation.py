"""Resumable offline primitive preparation with per-key atomic publication."""
from __future__ import annotations

import math
from pathlib import Path
import time

from .cell import Cell
from .store import Store, digest, atomic_json
from .native import setup, code_revision, prepare_candidate

_MODELS = {}


def implementation_revision():
    import hashlib
    root = Path(__file__).parent
    safety_modules = {"__init__.py", "cell.py", "geometry.py", "trajectory.py",
                      "validation.py", "native.py", "preparation.py", "store.py"}
    return digest({p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in sorted(root.glob("*.py")) if p.name in safety_modules})


def dependencies(cell, pid, rid, weld, planner, checker):
    params = cell.programs[pid]["extensions"]["weld_params"]
    # Dwell/pulse changes affect assembly, never this geometric/timed motion.
    motion = {"geometry": cell.program_keys[pid]["geometry"], "limits": cell.limits_hash,
              "phase_speeds": {p["phase"]: p.get("speed_percentage")
                               for p in params["weld_phases"] if p["type"] == "move"}}
    return {"cell_geometry": cell.geometry_hash, "motion": motion,
            "robot": rid, "weld": weld, "planner": planner, "implementation": checker,
            "start": cell.robots[rid]["home_pose_deg"], "kind": "home_weld_return"}


def build_job(workspace, cache, umarell, scratch, cid, pid, rid, weld, max_poses=4,
              validation_budget_s=120):
    from .geometry import Geometry
    from .trajectory import retime_native
    from .validation import certify
    workspace, umarell = Path(workspace).resolve(), Path(umarell).resolve()
    key = str(workspace), cid
    if key not in _MODELS:
        cell = Cell(workspace, cid)
        setup(umarell, workspace, Path(scratch), cid)
        _MODELS[key] = cell, Geometry(cell), code_revision(umarell)
    cell, geometry, planner = _MODELS[key]
    from agents.trajectory.tools.weld_orientation import candidates, policy
    store = Store(cache)
    version = implementation_revision()
    dep = dependencies(cell, pid, rid, weld, planner, version)
    op = next(o for o in cell.programs[pid]["operations"] if o["id"] == weld)
    cfg = cell.robots[rid]

    def build():
        started = time.perf_counter()
        good, failed = [], []
        examined = 0
        for spin, tilt, direction, ori in [(0., 0., 0., op["orientation"])] + candidates(op["orientation"], policy(cfg)):
            if geometry.target_tool_check(rid, op["position"], ori):
                continue
            pose = {"spin": spin, "tilt": tilt, "tilt_direction": direction, "orientation": ori}
            rawdep = {k: v for k, v in dep.items() if k not in {"implementation", "kind"}}
            rawdep.update(kind="native_candidate", pose=pose)
            raw, _ = store.obtain("native", rawdep, lambda: prepare_candidate(
                cell, pid, rid, op, [pose], planner_revision=planner, geometry=geometry))
            examined += 1
            if raw["status"] == "candidate":
                from umarell_fast import workers
                leg = raw["leg"]
                workers.reset_session(cell.cell_id)
                workers.inject(cell.cell_id, rid, leg["end"], leg.get("last_pose"),
                               {weld: leg.get("weld_pose")}, {weld: leg.get("op_spec")})
                points = retime_native(raw["leg"]["traj"], cfg, .6)
                cert = certify(cell, geometry, rid, op, points,
                               deadline=time.monotonic() + validation_budget_s)
                if cert["ok"]:
                    good.append({"points": points, "certificate": cert, "pose": pose})
                    break
                failed.append({"pose": pose, "certificate": cert})
            else:
                failed.append({"pose": pose, "native_failure": raw})
            if examined >= max_poses:
                break
        return {"status": "certified" if good else "timeout_or_unknown",
                "variants": good, "failures": failed, "robot": rid, "weld": weld,
                "cell": cid, "program": pid, "attempts": examined,
                "build_wall_s": time.perf_counter() - started}

    payload, hit = store.obtain("primitive", dep, build)
    return {"cell": cid, "program": pid, "robot": rid, "weld": weld,
            "dependencies": dep, "key": digest(dep), "cache_hit": hit,
            "status": payload["status"], "build_wall_s": payload.get("build_wall_s"),
            "failures": payload.get("failures", [])}


def export_manifest(workspace, store_root, umarell, output):
    store = Store(store_root)
    planner, implementation = code_revision(Path(umarell)), implementation_revision()
    rows = []
    for p in sorted((Path(workspace) / "cells").iterdir()):
        if not p.is_dir():
            continue
        cell = Cell(workspace, p.name)
        row = cell.manifest_export()
        row["coverage"] = []
        for pid, program in cell.programs.items():
            for rid in cell.robots:
                for op in program["operations"]:
                    dep = dependencies(cell, pid, rid, op["id"], planner, implementation)
                    value = store.get("primitive", dep)
                    row["coverage"].append({"program": pid, "robot": rid, "weld": op["id"],
                                            "key": digest(dep), "dependencies": dep,
                                            "status": value["status"] if value else "missing"})
        rows.append(row)
    manifest = {"schema_version": "umarell-cache-export-v1", "planner": planner,
                "implementation": implementation, "cells": rows, "storage": store.inspect(),
                "whole_request_cache_enabled": False,
                "safety": "Only certified primitives carry geometry/controller certificates. Unknown is not infeasible."}
    atomic_json(Path(output), manifest)
    return manifest
