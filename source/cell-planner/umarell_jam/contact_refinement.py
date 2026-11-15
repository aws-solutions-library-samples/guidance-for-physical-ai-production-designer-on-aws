"""Retain a certified transit while smoothing progress along its weld stroke."""
from __future__ import annotations

import hashlib
from pathlib import Path
import time

import numpy as np
from scipy.interpolate import CubicSpline

from .phases import phase_pack
from .release import stroke_proof
from .store import digest
from .trajectory import Trajectory, retime_native
from .validation import certify


def revision():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def key(cell, pid, rid, weld, planner, checker):
    from .primitive_index import reuse_key
    return {"schema": "contact-progress-refinement-v1",
            "base": reuse_key(cell, pid, rid, weld, planner, checker),
            "refiner": revision(),
            "retimer": hashlib.sha256(Path(__file__).with_name("phases.py").read_bytes()).hexdigest()}


def refine(cell, geometry, pid, item, store, *, deadline):
    """Generate a new candidate and recertify its full interpolant, including transit."""
    rid, weld = item["robot"], item["weld"]
    program = cell.programs[pid]
    op = next(o for o in program["operations"] if o["id"] == weld)
    pack = phase_pack(cell, pid, item, store)
    original = Trajectory(pack["points"]["contact"])
    axis = np.asarray(op["approach_vector"], dtype=float)
    axis /= np.linalg.norm(axis)
    positions, progress = [], []
    for t in np.linspace(0., original.duration, 65):
        q = original.sample(float(t))
        tcp = geometry.place(rid, q)[2]
        distance = float((tcp.translation-np.asarray(op["position"])) @ axis)
        if progress and distance <= progress[-1]+1e-8:
            continue
        positions.append(q)
        progress.append(distance)
    if len(progress) < 4 or progress[-1]-progress[0] < .001:
        return None, {"reason": "stroke_progress_not_resolvable"}
    s = (np.asarray(progress)-progress[0])/(progress[-1]-progress[0])
    curve = CubicSpline(s, np.asarray(positions), axis=0)
    points = []
    for t in np.linspace(0., 1., 49):
        p = 10*t**3-15*t**4+6*t**5
        v = 30*t**2-60*t**3+30*t**4
        a = 60*t-180*t**2+120*t**3
        points.append({"positions": curve(p).tolist(), "velocities": (curve(p, 1)*v).tolist(),
                       "accelerations": (curve(p, 2)*v*v+curve(p, 1)*a).tolist(),
                       "time_s": float(t), "phase": "approach"})
    points[0]["positions"] = pack["points"]["contact"][0]["positions"]
    points[-1]["positions"] = pack["points"]["contact"][-1]["positions"]
    ceiling = min(cell.config["planning"]["max_velocity_scaling"], next(
        p["speed_percentage"]/100 for p in program["extensions"]["weld_params"]["weld_phases"]
        if p["phase"] == "contact"))
    contact = retime_native(points, cell.robots[rid], ceiling)
    if contact[-1]["time_s"] >= original.duration*.98:
        return None, {"reason": "no_material_contact_time_improvement",
                      "original_s": original.duration, "candidate_s": contact[-1]["time_s"]}
    transit = [dict(p, phase="transit") for p in pack["points"]["approach"]]
    combined = transit+[dict(p, time_s=p["time_s"]+transit[-1]["time_s"]) for p in contact[1:]]
    proof = certify(cell, geometry, rid, op, combined, deadline=deadline)
    if not proof["ok"]:
        return None, {"reason": "refined_curve_not_certified", "certificate": proof}
    candidate = {k: v for k, v in item.items() if k not in {"phase_pack", "duration", "key", "transfer", "skip_home"}}
    candidate.update(points=combined, certificate=proof,
                     method="certified_transit_smooth_contact_progress",
                     source_trajectory_hash=digest(item["points"]), refiner=revision(),
                     original_contact_s=original.duration, refined_contact_s=contact[-1]["time_s"])
    release = stroke_proof(cell, candidate, store, pid)
    if not release["ok"]:
        return None, {"reason": "refined_stroke_not_certified", "certificate": release}
    phase_pack(cell, pid, candidate, store)
    return candidate, {"status": "certified", "original_contact_s": original.duration,
                       "refined_contact_s": contact[-1]["time_s"]}
