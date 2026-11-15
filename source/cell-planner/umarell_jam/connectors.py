"""Bounded, exact-state connectors. No rounded state is ever a certificate key."""
from __future__ import annotations

import hashlib
from pathlib import Path
import time

import numpy as np

from .geometry import Geometry
from .store import digest
from .trajectory import Trajectory


def revision():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def endpoint(q, v, t):
    return {"positions": list(map(float, q)), "velocities": list(map(float, v)),
            "accelerations": [0.] * len(q), "time_s": float(t), "phase": "connector"}


def direct(q, v, goal, cfg, ceiling, *, brake=False):
    """Find a two-knot quintic satisfying analytic limits, including the initial v."""
    q, v, goal = np.asarray(q), np.asarray(v), np.asarray(goal)
    velocity = np.asarray(cfg["joint_velocity_limits"]) * ceiling
    acceleration = np.asarray(cfg["joint_acceleration_limits"]) * ceiling**2
    duration = max(.08, float(np.max(2 * abs(goal - q) / velocity)),
                   float(np.max(2 * abs(v) / acceleration)))
    for _ in range(32):
        end = q + v * duration / 2 if brake else goal
        points = [endpoint(q, v, 0.), endpoint(end, np.zeros(len(q)), duration)]
        path = Trajectory(points)
        lo, hi = path.extrema(0)
        if np.any(lo < np.asarray(cfg["joint_limits_lower"]) - 1e-9) or np.any(
                hi > np.asarray(cfg["joint_limits_upper"]) + 1e-9):
            return None
        if (np.all(path.peaks(1) <= np.maximum(velocity, abs(v)) + 1e-9)
                and np.all(path.peaks(2) <= acceleration + 1e-9)
                and (not cfg.get("joint_jerk_limits")
                     or np.all(path.peaks(3) <= cfg["joint_jerk_limits"]))):
            return points
        duration *= 1.2
    return None


def certify_connector(cell, geometry, rid, points, states, *, deadline):
    """400 Hz controller replay and swept full-scene validation of one connector."""
    from utils.fjt_trajectory import sample as native_sample
    from agents.trajectory.tools import robot_kinematics as rk
    path = Trajectory(points)
    if np.max(abs(np.asarray(states[rid]) - path.sample(0))) > 1e-9:
        return {"ok": False, "reason": "actual_start_gap"}
    cfg = cell.robots[rid]
    model, _ = rk._load_pinocchio_model(cell.cell_id, rid)
    data = model.createData()
    obstacles = rk.load_static_obstacles(cell.cell_id)
    policy = rk._load_clearance_policy(cell.cell_id)
    radii = rk._cell_sphere_radii(cell.cell_id, model, cfg.get("tool_collision"))

    def native_hit(q, margin=0.):
        return rk._config_collision_info(
            model, data, np.asarray(q), np.asarray(cfg["base_position"]),
            cfg["base_orientation_yaw"], [], obstacles, sphere_radius=policy["normal"],
            sphere_radii=radii, extra_margin=margin)

    checkpoints = np.unique(np.r_[np.arange(0., path.duration, .0025), path.times, path.duration])
    velocity = path.peaks(1)
    lever = geometry.levers[rid]
    boxes = None
    for t in checkpoints:
        if time.monotonic() >= deadline:
            return {"ok": False, "reason": "connector_validation_deadline"}
        q = path.sample(float(t))
        if np.max(abs(q - native_sample(points, float(t), "time_s")[0])) > 1e-8:
            return {"ok": False, "reason": "connector_interpolant_mismatch"}
        hit = geometry.check({**states, rid: q.tolist()}) or native_hit(q)
        if hit:
            return {"ok": False, "reason": "connector_collision", "violation": hit, "time_s": float(t)}
        bodies, _, _ = geometry.place(rid, q)
        pad = float(velocity @ lever) * .0025
        lo, hi = np.array([b["lo"] for b in bodies]) - pad, np.array([b["hi"] for b in bodies]) + pad
        boxes = [lo, hi] if boxes is None else [np.minimum(boxes[0], lo), np.maximum(boxes[1], hi)]
    stack = list(zip(checkpoints[:-1], checkpoints[1:]))
    subdivisions = 0
    while stack:
        if time.monotonic() >= deadline:
            return {"ok": False, "reason": "connector_swept_deadline"}
        a, b = stack.pop()
        mid = float((a + b) / 2)
        displacement = {r: 0. for r in states}
        displacement[rid] = float(velocity @ lever) * (b - a) / 2
        q = path.sample(mid)
        if (geometry.interval_clear({**states, rid: q.tolist()}, displacement)
                and native_hit(q, displacement[rid]) is None):
            continue
        if b - a < 1e-6:
            return {"ok": False, "reason": "connector_swept_unresolved"}
        stack.extend([(a, mid), (mid, b)])
        subdivisions += 1
    return {"ok": True, "geometry_hash": cell.geometry_hash, "trajectory_hash": digest(points),
            "initial_states_hash": digest(states), "checker": revision(), "replay_hz": 400,
            "replay_samples": len(checkpoints), "subdivisions": subdivisions,
            "occupancy": {"lo": boxes[0].tolist(), "hi": boxes[1].tolist()},
            "velocity_peak": velocity.tolist(), "acceleration_peak": path.peaks(2).tolist(),
            "jerk_peak": path.peaks(3).tolist()}


def prepare_start(cell, request, store, geometry, *, deadline, planner_revision=None, implementation=None):
    """Brake moving enabled arms, then connect them to HOME; disabled arms stay put."""
    from .validation import compatible
    home = {r: np.radians(c["home_pose_deg"]).tolist() for r, c in cell.robots.items()}
    states = {r: list(s["positions"]) for r, s in request.initial["robots"].items()}
    moving = [r for r, s in request.initial["robots"].items() if max(map(abs, s["velocities"])) > 1e-8]
    if set(moving) - set(request.enabled):
        return {"ok": False, "reason": "moving_inactive_arm_requires_controller_acknowledgement"}
    # An acknowledged completed weld may stand at a certified contact endpoint.
    # Match the exact configuration and use its full reverse curve, including
    # the on-axis withdrawal. No straight connector through the panel is tried.
    known_returns = {}
    if planner_revision and implementation:
        from .primitive_index import lookup
        for rid in request.enabled:
            if rid in moving:
                continue
            for weld in request.completed:
                entry, _ = lookup(store, cell, request.program_id, rid, weld, planner_revision, implementation)
                if not entry or entry.get("status") != "certified":
                    continue
                for variant in entry["variants"]:
                    if not all(r == rid or np.max(abs(np.asarray(q) - home[r])) < 1e-8
                               for r, q in states.items()):
                        continue
                    if np.max(abs(np.asarray(states[rid]) - variant["points"][-1]["positions"])) < 1e-8:
                        known_returns[rid] = {**variant, "weld": weld, "at_contact": True,
                                              "return_points": Trajectory(variant["points"]).reversed()}
                    else:
                        from .phases import phase_pack
                        phases = phase_pack(cell, request.program_id,
                                            {**variant, "robot": rid, "weld": weld}, store)["points"]
                        if np.max(abs(np.asarray(states[rid]) - phases["home"][0]["positions"])) < 1e-8:
                            known_returns[rid] = {**variant, "weld": weld, "at_contact": False,
                                                  "return_points": phases["home"]}
    contact_targets = {rid: next(o["position"] for o in cell.programs[request.program_id]["operations"]
                                if o["id"] == known_returns[rid]["weld"])
                       for rid in known_returns if known_returns[rid]["at_contact"]}
    hit = geometry.check(states, contact_targets)
    if hit:
        return {"ok": False, "reason": "initial_scene_collision", "violation": hit}
    batches, proofs = [], []
    ceiling = cell.config["planning"]["max_velocity_scaling"]

    def checked(rid, goal, brake=False):
        velocity = request.initial["robots"][rid]["velocities"] if brake else [0.] * len(goal)
        points = direct(states[rid], velocity, goal, cell.robots[rid], ceiling, brake=brake)
        if points is None:
            return None, {"ok": False, "reason": "no_limit_valid_direct_connector"}
        dep = {"geometry": cell.geometry_hash, "limits": cell.limits_hash, "checker": revision(),
               "states": states, "robot": rid, "points": points, "kind": "actual_state_connector"}
        cert = store.get("connector", dep)
        if cert is None:
            cert = certify_connector(cell, geometry, rid, points, states, deadline=deadline)
            if cert["ok"]:
                cert = store.obtain("connector", dep, lambda: cert)[0]
        return {"robot": rid, "points": points, "phase": "brake" if brake else "start_home"}, cert

    brake_batch = []
    for rid in moving:
        motion, cert = checked(rid, home[rid], True)
        if not cert["ok"]:
            return cert
        motion["certificate"] = cert
        if any(not compatible(x["certificate"], cert, cell.config["collision"]["min_clearance"])
               for x in brake_batch):
            return {"ok": False, "reason": "coupled_brake_occupancy_unresolved"}
        brake_batch.append(motion)
    if brake_batch:
        batches.append(brake_batch)
        for item in brake_batch:
            states[item["robot"]] = item["points"][-1]["positions"]
            proofs.append(item["certificate"])
    # Empty hold requests explicitly request no motion.
    return_home = request.pending or request.raw["terminal_state"] == "home"
    if return_home:
        for rid in request.enabled:
            if np.max(abs(np.asarray(states[rid]) - home[rid])) <= 1e-9:
                continue
            if rid in known_returns:
                variant = known_returns[rid]
                points = variant["return_points"]
                from .phases import scaled
                from .trajectory import limit_check
                path = Trajectory(points)
                scale = min(ceiling, cell.config["planning"]["home_velocity_scale"])
                cfg = cell.robots[rid]
                factor = max(1., float(np.max(path.peaks(1) / (np.asarray(cfg["joint_velocity_limits"]) * scale))),
                             float(np.sqrt(np.max(path.peaks(2) / (np.asarray(cfg["joint_acceleration_limits"]) * scale**2)))))
                points = scaled(points, factor * 1.00001)
                cert = {**variant["certificate"], "trajectory_hash": digest(points),
                        "source_trajectory_hash": variant["certificate"]["trajectory_hash"],
                        "limits": limit_check(Trajectory(points), cfg, scale),
                        "proof": "exact reverse of certified completed-weld curve"}
                if not cert["limits"]["ok"]:
                    return {"ok": False, "reason": "completed_weld_return_limits"}
                motion = {"robot": rid, "points": points, "phase": "completed_weld_retract",
                          "certificate": cert}
            else:
                motion, cert = checked(rid, home[rid])
            if not cert["ok"]:
                return cert
            motion["certificate"] = cert
            batches.append([motion])
            proofs.append(cert)
            states[rid] = home[rid]
    return {"ok": True, "batches": batches, "states": states, "certificates": proofs}


def bystanders_clear(geometry, certificate, rid, states):
    """Reuse a moving arm's swept occupancy against the actual stationary bystanders."""
    lo, hi = np.asarray(certificate["occupancy"]["lo"]), np.asarray(certificate["occupancy"]["hi"])
    margin = geometry.cell.config["collision"]["min_clearance"]
    for other, q in states.items():
        if other == rid:
            continue
        bodies, _, _ = geometry.place(other, q)
        blo, bhi = np.array([b["lo"] for b in bodies]), np.array([b["hi"] for b in bodies])
        lower = np.linalg.norm(np.maximum(np.maximum(lo[:, None] - bhi[None],
                                                     blo[None] - hi[:, None]), 0), axis=2)
        if np.any(lower <= margin + 1e-8):
            return False
    return True
