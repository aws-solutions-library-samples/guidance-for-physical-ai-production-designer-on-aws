"""Connect diagnosed whole-arm branches to HOME and apply unchanged release checks (session tools/certify_branches.py)."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.interpolate import CubicSpline

from ..cell import Cell
from ..connectors import direct
from ..electrode_geometry import RefinedElectrodeGeometry, revision
from ..native import setup, code_revision
from ..preparation import dependencies, implementation_revision
from ..release import stroke_proof
from ..phases import phase_pack
from ..store import Store, atomic_json
from ..trajectory import Trajectory, retime_native
from ..validation import certify
from ..primitive_index import supplemental_key


def smooth_stroke(positions, cfg, contact_scale):
    """Smooth Cartesian progress removes artificial stops at equally spaced IK knots."""
    curve = CubicSpline(np.linspace(0., 1., len(positions)), np.asarray(positions), axis=0)
    points = []
    for t in np.linspace(0., 1., 33):
        s = 10*t**3 - 15*t**4 + 6*t**5
        ds = 30*t**2 - 60*t**3 + 30*t**4
        dds = 60*t - 180*t**2 + 120*t**3
        points.append({"positions": curve(s).tolist(), "velocities": (curve(s, 1)*ds).tolist(),
                       "accelerations": (curve(s, 2)*ds**2+curve(s, 1)*dds).tolist(),
                       "time_s": float(t), "phase": "approach"})
    return retime_native(points, cfg, min(contact_scale, .6))


def build(layout, source, budget, variants, supplemental=False):
    layout.activate()
    probe = json.loads(source.read_text())
    cid, rid, pid, weld = (probe[k] for k in ("cell", "robot", "program", "weld"))
    cell = Cell(layout.workspace, cid)
    if cell.geometry_hash != probe["geometry"]:
        raise ValueError("stale diagnostic scene")
    setup(layout.umarell, layout.workspace,
          layout.prepare_runs / "coverage-v2/branch-workers", cid)
    geometry = RefinedElectrodeGeometry(cell)
    store = Store(layout.components)
    base = dependencies(cell, pid, rid, weld, code_revision(layout.umarell),
                        implementation_revision())
    dep = (supplemental_key(cell, pid, rid, weld, code_revision(layout.umarell),
                            implementation_revision()) if supplemental else
           {"base": base, "electrode_refinement": revision()})
    layer = "primitive_supplement" if supplemental else "refined_primitive"
    found = store.get(layer, dep)
    if found and found["status"] == "certified":
        return {"cell": cid, "robot": rid, "weld": weld, "status": "certified", "cache_hit": True}
    cfg = cell.robots[rid]
    op = next(o for o in cell.programs[pid]["operations"] if o["id"] == weld)
    states = {r: np.radians(c["home_pose_deg"]).tolist() for r, c in cell.robots.items()}
    home = states[rid]
    from ..branch_routes import route
    started = time.monotonic()
    good, failures = [], []
    choices = sorted(probe["solutions"], key=lambda s: np.linalg.norm(
        np.asarray(s["stroke"][0]) - home))
    for index, solution in enumerate(choices):
        if time.monotonic() - started >= budget:
            break
        via = solution["stroke"][0]
        stage = direct(home, [0.] * 6, via, cfg, .6)
        if stage is not None:
            path = Trajectory(stage)
            if any(geometry.check({**states, rid: path.sample(float(t)).tolist()})
                   for t in np.linspace(0., path.duration, 65)):
                stage = None
        method = "direct_quintic"
        if stage is None:
            stage, route_stats = route(cell, geometry, rid, home, via,
                                       budget_s=min(20., max(0., budget-(time.monotonic()-started))),
                                       seed=20261110+index)
            method = "bounded_joint_route"
            if stage is None:
                failures.append({"candidate": index, **route_stats})
                continue
        if not stage:
            failures.append({"candidate": index, "reason": "no_home_to_standoff_path"})
            continue
        stage = [dict(p, phase="transit") for p in stage]
        stage[-1]["positions"] = via
        stage = retime_native(stage, cfg, .6)
        contact_scale = next(p["speed_percentage"] / 100
                             for p in cell.programs[pid]["extensions"]["weld_params"]["weld_phases"]
                             if p["phase"] == "contact")
        stroke = smooth_stroke(solution["stroke"], cfg, contact_scale)
        points = stage + [dict(p, time_s=p["time_s"] + stage[-1]["time_s"]) for p in stroke[1:]]
        cert = certify(cell, geometry, rid, op, points, deadline=started+budget)
        item = {"points": points, "certificate": cert, "pose": solution["pose"],
                "branch_seed": solution["seed"], "method": "whole_arm_ik_" + method,
                "search_code": hashlib.sha256(Path(__file__).with_name("coverage_probe.py").read_bytes()).hexdigest(),
                "builder_code": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
        proof = stroke_proof(cell, {**item, "robot": rid, "weld": weld}, store, pid) if cert["ok"] else cert
        if proof["ok"]:
            pack = phase_pack(cell, pid, {**item, "robot": rid, "weld": weld}, store)
            good.append(item)
            atomic_json(layout.prepare_runs / "coverage-v2/certification-checkpoints" / source.name,
                        {"cell": cid, "program": pid, "robot": rid, "weld": weld,
                         "status": "certified", "variants": good, "failures": failures})
            print(json.dumps({"cell": cid, "weld": weld, "robot": rid, "candidate": index,
                              "status": "certified", "phase_motion_s": {
                                  k: v[-1]["time_s"] for k, v in pack["points"].items()}}), flush=True)
            if len(good) >= variants:
                break
        else:
            failures.append({"candidate": index, "method": method, "certificate": proof})
            print(json.dumps({"cell": cid, "robot": rid, "candidate": index, "failure": proof}), flush=True)
    payload = {"status": "certified" if good else "timeout_or_unknown", "cell": cid,
               "program": pid, "robot": rid, "weld": weld, "variants": good,
               "failures": failures, "build_wall_s": time.monotonic()-started,
               "diagnostic_source": str(source.relative_to(layout.root)),
               "diagnostic_sha256": hashlib.sha256(source.read_bytes()).hexdigest()}
    store.obtain(layer, dep, lambda: payload)
    output = layout.prepare_runs / "coverage-v2/certification" / source.name
    atomic_json(output, payload)
    return {k: v for k, v in payload.items() if k not in {"variants", "failures"}}

