"""Uniform phase retiming preserves the certified geometric controller curve."""
from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np

from .store import digest
from .trajectory import Trajectory, limit_check


def process_settings(program):
    ext = program["extensions"]
    return {**ext["weld_params"], "product_task": ext.get("product_task", {})}


def scaled(points, factor):
    return [{**p, "time_s": p["time_s"] * factor,
             "velocities": [v / factor for v in p["velocities"]],
             "accelerations": [a / factor**2 for a in p["accelerations"]]} for p in points]


def phase_pack(cell, pid, item, store):
    params = cell.programs[pid]["extensions"]["weld_params"]
    dep = {"trajectory": digest(item["points"]), "limits": cell.limits_hash,
           "phase_speeds": {p["phase"]: p.get("speed_percentage") for p in params["weld_phases"]},
           "robot": item["robot"], "retimer": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    cached = store.get("phase_pack", dep)
    if cached is not None:
        return cached
    points, rid = item["points"], item["robot"]
    cfg = cell.robots[rid]
    cut = next((max(0, i-1) for i, p in enumerate(points) if p.get("phase") == "approach"), len(points)-1)
    offset = points[cut]["time_s"]
    approach, contact = points[:cut+1], [{**p, "time_s": p["time_s"] - offset} for p in points[cut:]]
    raw = {"approach": approach, "contact": contact, "retract": Trajectory(contact).reversed(),
           "home": Trajectory(approach).reversed()}
    speeds = {p["phase"]: p.get("speed_percentage", 100) / 100 for p in params["weld_phases"]}
    speeds["home"] = cell.config["planning"]["home_velocity_scale"]
    out, checks = {}, {}
    for phase, pts in raw.items():
        path = Trajectory(pts)
        ceiling = min(cell.config["planning"]["max_velocity_scaling"], speeds[phase])
        vlim, alim = np.asarray(cfg["joint_velocity_limits"]) * ceiling, np.asarray(cfg["joint_acceleration_limits"]) * ceiling**2
        factor = max(float(np.max(path.peaks(1) / vlim)),
                     float(np.sqrt(np.max(path.peaks(2) / alim))), .02)
        if cfg.get("joint_jerk_limits"):
            factor = max(factor, float(np.cbrt(np.max(path.peaks(3) / cfg["joint_jerk_limits"]))))
        # Forward approach/contact retain the native planner's TCP-speed timing.
        # Retraction may accelerate along the same curve up to its native phase
        # percentage; HOME also obeys its separately configured ceiling.
        if phase in {"approach", "contact"}:
            factor = max(1., factor)
        factor *= 1.00001
        out[phase] = scaled(pts, factor)
        retimed = Trajectory(out[phase])
        checks[phase] = limit_check(retimed, cfg, ceiling)
        if not checks[phase]["ok"]:
            raise ValueError(f"phase limits failed: {phase} {checks[phase]}")
        if any(max(map(abs, p[f])) > 1e-9 for p in (out[phase][0], out[phase][-1])
               for f in ("velocities", "accelerations")):
            raise ValueError("phase split is not a controller rest boundary")
        # Independent replay of the exported timing and exact geometric
        # correspondence. The original swept proof covers every intermediate
        # configuration on this identical curve, at any traversal speed.
        from utils.fjt_trajectory import sample
        count = 0
        for t in np.unique(np.r_[np.arange(0., retimed.duration, .0025), retimed.times, retimed.duration]):
            q = retimed.sample(float(t))
            if (np.max(abs(q - path.sample(float(t) / factor))) > 1e-8
                    or np.max(abs(q - sample(out[phase], float(t), "time_s")[0])) > 1e-8):
                raise ValueError("retimed controller curve equivalence failed")
            count += 1
        checks[phase].update(replay_samples=count, replay_hz=400, geometric_curve_identical=True,
                             time_scale=factor)
    payload = {"status": "certified", "points": out, "checks": checks,
               "source_certificate_hash": digest(item["certificate"])}
    return store.obtain("phase_pack", dep, lambda: payload)[0]
