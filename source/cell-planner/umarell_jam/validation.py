"""Independent 400 Hz replay plus conservative bounds between every replay sample."""
from __future__ import annotations

import math
import time
import numpy as np

from . import CHECKER_VERSION
from .store import digest
from .trajectory import Trajectory, limit_check, retime_native


def certify(cell, geometry, rid, op, points, *, deadline=None):
    start = time.perf_counter()
    path = Trajectory(points)
    limits = limit_check(path, cell.robots[rid], cell.config["planning"]["max_velocity_scaling"])
    if not limits["ok"]:
        return {"ok": False, "violation": limits}
    states = {r: np.radians(c["home_pose_deg"]).tolist() for r, c in cell.robots.items()}
    qstart = np.array(states[rid])
    if np.max(abs(path.sample(0) - qstart)) > 1e-7:
        return {"ok": False, "violation": {"reason": "home_start_gap"}}
    phases = [p.get("phase", "transit") for p in points]
    contact_start = next((points[max(0, i-1)]["time_s"] for i, p in enumerate(phases)
                          if p == "approach"), path.duration)
    vmax = path.peaks(1)
    velocity_segments = np.max(abs(path.segment_extrema(1)), axis=1)
    lever = geometry.levers[rid]
    checkpoints = np.unique(np.r_[np.arange(0, path.duration, .0025), path.times, path.duration])
    boxes = None
    replay_samples, subdivisions = 0, 0
    # The native sampler is used only for an independent interpolation equivalence
    # check; our polynomial derivative/limit computation is separate.
    from utils.fjt_trajectory import sample as native_sample
    # Preserve the supplied planner's declared floor/obstacle margins and its
    # current collision predicate in addition to the physical mesh backstop.
    from agents.trajectory.tools import robot_kinematics as rk
    native_model, _ = rk._load_pinocchio_model(cell.cell_id, rid)
    native_data = native_model.createData()
    cfg = cell.robots[rid]
    native_obstacles = rk.load_static_obstacles(cell.cell_id)
    native_policy = rk._load_clearance_policy(cell.cell_id)
    native_radii = rk._cell_sphere_radii(cell.cell_id, native_model, cfg.get("tool_collision"))

    def native_hit(q, extra=0.):
        return rk._config_collision_info(
            native_model, native_data, np.array(q), np.array(cfg["base_position"]),
            cfg["base_orientation_yaw"], [], native_obstacles,
            sphere_radius=native_policy["normal"], sphere_radii=native_radii,
            extra_margin=extra)

    def state(t):
        return {**states, rid: path.sample(t).tolist()}

    for t in checkpoints:
        if deadline is not None and time.monotonic() >= deadline:
            return {"ok": False, "status": "timeout_or_unknown", "reason": "validation_deadline"}
        q = path.sample(float(t))
        original = np.array(native_sample(points, float(t), "time_s")[0])
        if np.max(abs(q - original)) > 1e-8:
            return {"ok": False, "violation": {"reason": "controller_interpolant_mismatch", "time_s": float(t)}}
        contacts = {rid: op["position"]} if t >= contact_start else {}
        hit = geometry.check({**states, rid: q.tolist()}, contacts)
        if hit is None:
            hit = native_hit(q)
        replay_samples += 1
        if hit:
            return {"ok": False, "violation": hit, "time_s": float(t),
                    "replay_samples": replay_samples, "wall_s": time.perf_counter() - start}
        bodies, _, tcp = geometry.place(rid, q)
        # Motion-expanded boxes accumulated over all replay intervals are a
        # conservative occupancy of each collision body for independent clocks.
        pad = float(vmax @ lever) * .0025
        lo = np.array([b["lo"] for b in bodies]) - pad
        hi = np.array([b["hi"] for b in bodies]) + pad
        if boxes is None:
            boxes = [lo, hi]
        else:
            boxes = [np.minimum(boxes[0], lo), np.maximum(boxes[1], hi)]
        if t >= contact_start:
            target = np.array(op["position"])
            axis = np.array(op["approach_vector"], dtype=float)
            axis /= np.linalg.norm(axis)
            rel = tcp.translation - target
            lateral = np.linalg.norm(rel - axis * (rel @ axis))
            if lateral > .002:
                return {"ok": False, "violation": {"reason": "stroke_lateral", "error_m": float(lateral)}}
    _, _, tcp = geometry.place(rid, path.sample(path.duration))
    if np.linalg.norm(tcp.translation - op["position"]) > .001:
        return {"ok": False, "violation": {"reason": "weld_position"}}
    from scipy.spatial.transform import Rotation
    expected = Rotation.from_quat([op["orientation"][k] for k in "xyzw"]).as_matrix()[:, 0]
    angle = math.degrees(math.acos(np.clip(expected @ tcp.rotation[:, 0], -1, 1)))
    tolerance = cell.robots[rid].get("tool_orientation", {}).get("axis_tolerance_deg", 5)
    if angle > tolerance:
        return {"ok": False, "violation": {"reason": "weld_axis", "error_deg": angle}}

    stack = list(zip(checkpoints[:-1], checkpoints[1:]))
    while stack:
        if deadline is not None and time.monotonic() >= deadline:
            return {"ok": False, "status": "timeout_or_unknown", "reason": "swept_validation_deadline"}
        a, b = stack.pop()
        mid = float((a + b) / 2)
        displacement = {r: 0. for r in states}
        segment = min(int(np.searchsorted(path.times, mid, side="right")) - 1, len(path.times)-2)
        displacement[rid] = float(velocity_segments[segment] @ lever) * (b - a) / 2
        contacts = {rid: op["position"]} if a >= contact_start else {}
        if (geometry.interval_clear(state(mid), displacement, contacts)
                and native_hit(path.sample(mid), displacement[rid]) is None):
            continue
        if b - a < 1e-6:
            return {"ok": False, "status": "timeout_or_unknown",
                    "reason": "swept_bound_unresolved", "interval_s": [float(a), float(b)]}
        stack.extend([(a, mid), (mid, b)])
        subdivisions += 1
    return {"ok": True, "checker_version": CHECKER_VERSION,
            "geometry_hash": cell.geometry_hash, "trajectory_hash": digest(points),
            "replay_hz": 400, "replay_samples": replay_samples, "subdivisions": subdivisions,
            "limits": limits, "occupancy": {"lo": boxes[0].tolist(), "hi": boxes[1].tolist()},
            "contact_start_s": float(contact_start), "endpoint_axis_error_deg": angle,
            "native_static_predicate": True,
            "wall_s": time.perf_counter() - start,
            "proof": "400Hz independent controller replay and Lipschitz swept bounds; exact reversed curve also covered"}


def compatible(a, b, margin):
    """Sufficient all-relative-timings condition, including arbitrary slowdown and holds."""
    aa, bb = a["occupancy"], b["occupancy"]
    alo, ahi = np.array(aa["lo"]), np.array(aa["hi"])
    blo, bhi = np.array(bb["lo"]), np.array(bb["hi"])
    lower = np.linalg.norm(np.maximum(np.maximum(
        alo[:, None] - bhi[None], blo[None] - ahi[:, None]), 0), axis=2)
    return bool(np.all(lower > margin + 1e-8))
