"""Retrieve old numerical guesses and certify newly planned paths in this scene.

The source geometry may differ arbitrarily. Source certificates NEVER authorize
the repaired path: publication requires the ordinary current-scene checks.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import time

import numpy as np

from .store import digest


def key(cell, pid, rid, weld, planner, implementation):
    from .primitive_index import reuse_key
    return {"base": reuse_key(cell, pid, rid, weld, planner, implementation),
            "seed_repair": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


def candidates(store, cid, rid, weld, joints, *, deadline, limit=12):
    """Bounded legacy-compatible discovery; checksums are integrity, not safety."""
    found, seen = [], set()
    layers = ("placement_primitive", "primitive_index", "primitive_supplement",
              "refined_primitive", "primitive", "contact_refinement")
    examined = 0
    for layer in layers:
        for path in sorted((store.root / layer).glob("*.json")):
            examined += 1
            if time.monotonic() >= deadline or examined > 4096:
                return sorted(found, key=lambda x: x["points"][-1]["time_s"])[:limit]
            try:
                doc = json.loads(path.read_bytes())
                if (doc["key"] != path.stem or doc.get("layer") != layer
                        or digest(doc["dependencies"]) != doc["key"]):
                    continue
                payload = store.get(layer, doc["dependencies"])
                if not payload or payload.get("status") != "certified":
                    continue
                if (payload.get("cell"), payload.get("robot"), payload.get("weld")) != (cid, rid, weld):
                    continue
                for item in payload["variants"]:
                    points = item["points"]
                    identity = digest(points)
                    cert = item["certificate"]
                    if (identity in seen or not cert.get("ok")
                            or cert.get("trajectory_hash") != identity
                            or len(points) < 2):
                        continue
                    for point in points:
                        q = np.asarray(point["positions"], dtype=float)
                        if q.shape != (joints,) or not np.all(np.isfinite(q)):
                            raise ValueError("incompatible numerical seed")
                    orientation = item["pose"]["orientation"]
                    quat = np.asarray([orientation[k] for k in "xyzw"], dtype=float)
                    if not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < 1e-8:
                        continue
                    seen.add(identity)
                    found.append({"points": points, "pose": item["pose"],
                                  "source_geometry": cert["geometry_hash"],
                                  "source_component": doc["key"], "source_layer": layer,
                                  "source_trajectory": identity,
                                  "source_validation_wall_s": float(cert.get("wall_s", 60.))})
            except (OSError, ValueError, KeyError, TypeError, IndexError):
                continue
    return sorted(found, key=lambda x: x["points"][-1]["time_s"])[:limit]


class BranchIK:
    def __init__(self, geometry, rid):
        import pinocchio as pin
        arm = geometry.arms[rid]
        self.model, self.frame = arm["m"], arm["tcp"]
        self.data, self.world = self.model.createData(), arm["world"]
        cfg = geometry.cell.robots[rid]
        self.lower = np.asarray(cfg["joint_limits_lower"]) + 1e-8
        self.upper = np.asarray(cfg["joint_limits_upper"]) - 1e-8
        self.offset = pin.SE3(np.eye(3), np.asarray(cfg.get("tcp_offset", [0., 0., 0.])))

    def solve(self, position, orientation, seed):
        import pinocchio as pin
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation
        desired = self.world.inverse() * pin.SE3(
            Rotation.from_quat([orientation[k] for k in "xyzw"]).as_matrix(),
            np.asarray(position)) * self.offset.inverse()

        def residual(q):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            return pin.log6(self.data.oMf[self.frame].actInv(desired)).vector.copy()

        def jac(q):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            relative = self.data.oMf[self.frame].actInv(desired)
            return -pin.Jlog6(relative.inverse()) @ pin.computeFrameJacobian(
                self.model, self.data, q, self.frame, pin.LOCAL)

        result = least_squares(residual, np.clip(seed, self.lower, self.upper), jac=jac,
                               bounds=(self.lower, self.upper), max_nfev=90,
                               ftol=1e-9, xtol=1e-9, gtol=1e-9)
        error = residual(result.x)
        return result.x if np.linalg.norm(error[:3]) <= 1e-5 and np.linalg.norm(error[3:]) <= 1e-4 else None


def smooth_stroke(positions, cfg, scale):
    from scipy.interpolate import CubicSpline
    from .trajectory import retime_native
    curve = CubicSpline(np.linspace(0., 1., len(positions)), np.asarray(positions), axis=0)
    points = []
    for t in np.linspace(0., 1., 33):
        s, ds, dds = (10*t**3-15*t**4+6*t**5, 30*t**2-60*t**3+30*t**4,
                      60*t-180*t**2+120*t**3)
        points.append({"positions": curve(s).tolist(), "velocities": (curve(s, 1)*ds).tolist(),
                       "accelerations": (curve(s, 2)*ds**2+curve(s, 1)*dds).tolist(),
                       "time_s": float(t), "phase": "approach"})
    return retime_native(points, cfg, min(scale, .6))


def blended_transit(source, goal, cfg):
    from .trajectory import retime_native
    points = source["points"]
    cut = next((i for i, p in enumerate(points) if p.get("phase") == "approach"), 0)-1
    if cut < 1 or points[cut]["time_s"] <= 0:
        return None
    delta = np.asarray(goal)-points[cut]["positions"]
    duration = points[cut]["time_s"]
    transit = []
    for p in points[:cut+1]:
        s = p["time_s"]/duration
        w, dw, ddw = (10*s**3-15*s**4+6*s**5, (30*s**2-60*s**3+30*s**4)/duration,
                       (60*s-180*s**2+120*s**3)/duration**2)
        transit.append({**p, "phase": "transit",
                        "positions": (np.asarray(p["positions"])+w*delta).tolist(),
                        "velocities": (np.asarray(p.get("velocities", np.zeros(len(goal))))+dw*delta).tolist(),
                        "accelerations": (np.asarray(p.get("accelerations", np.zeros(len(goal))))+ddw*delta).tolist()})
    transit[-1]["positions"] = list(goal)
    return retime_native(transit, cfg, .6)


def adapt(cell, geometry, pid, rid, weld, sources, store, planner, implementation, *, deadline):
    from .branch_routes import route
    from .connectors import direct
    from .phases import phase_pack
    from .release import stroke_proof
    from .trajectory import Trajectory, limit_check, retime_native
    from .validation import certify
    started = time.monotonic()
    cfg = cell.robots[rid]
    op = next(o for o in cell.programs[pid]["operations"] if o["id"] == weld)
    params = cell.programs[pid]["extensions"]["weld_params"]
    states = {r: np.radians(c["home_pose_deg"]).tolist() for r, c in cell.robots.items()}
    home = np.asarray(states[rid])
    axis = np.asarray(op["approach_vector"], dtype=float)
    axis /= np.linalg.norm(axis)
    solver, attempts, seen = BranchIK(geometry, rid), [], set()
    rng = np.random.default_rng(1949)

    def free(points, contact=False):
        if points is None:
            return False
        path = Trajectory(points)
        if not limit_check(path, cfg, cell.config["planning"]["max_velocity_scaling"])["ok"]:
            return False
        for t in np.linspace(0., path.duration, 65):
            if time.monotonic() >= deadline:
                return False
            if geometry.check({**states, rid: path.sample(float(t)).tolist()},
                              {rid: op["position"]} if contact else {}):
                return False
        return True

    for source in sources:
        if time.monotonic() >= deadline:
            break
        orientation = source["pose"]["orientation"]
        old = np.asarray(source["points"][-1]["positions"])
        # Large moves can change the useful branch; do not impose a small
        # joint-distance cutoff on the first IK solution.
        guesses = [old, home] + [np.clip(old+rng.normal(0., .8, len(home)), solver.lower, solver.upper)
                                for _ in range(4)]
        for guess_index, guess in enumerate(guesses):
            if time.monotonic() >= deadline:
                break
            attempt = {k: source[k] for k in ("source_geometry", "source_component", "source_trajectory")}
            attempt["guess"] = guess_index
            attempts.append(attempt)
            q = solver.solve(op["position"], orientation, guess)
            if q is None:
                attempt["reason"] = "seeded_ik_unknown"
                continue
            branch = tuple(np.round(q, 4))
            if branch in seen:
                attempt["reason"] = "duplicate_branch"
                continue
            seen.add(branch)
            hit = geometry.check({**states, rid: q.tolist()}, {rid: op["position"]})
            if hit:
                attempt.update(reason="endpoint_collision", violation=hit)
                continue
            stroke = [q.tolist()]
            for distance in np.linspace(0., params["approach_offset_z"], 13)[1:]:
                if time.monotonic() >= deadline:
                    break
                new = solver.solve(np.asarray(op["position"])-axis*distance, orientation, q)
                if new is None or np.max(abs(new-q)) > .35:
                    break
                stroke.append(new.tolist())
                q = new
            if len(stroke) != 13:
                attempt["reason"] = "stroke_ik_unknown"
                continue
            stroke.reverse()
            scale = next(p["speed_percentage"]/100 for p in params["weld_phases"] if p["phase"] == "contact")
            contact = smooth_stroke(stroke, cfg, scale)
            if not free(contact, True):
                attempt["reason"] = "stroke_collision_or_limits"
                continue
            transit = blended_transit(source, stroke[0], cfg)
            method = "cached_transit_blend"
            if not free(transit):
                transit = direct(home, np.zeros(len(home)), stroke[0], cfg, .6)
                method = "cached_ik_direct_transit"
            if not free(transit):
                transit, stats = route(cell, geometry, rid, home, stroke[0],
                                       budget_s=min(12., max(0., deadline-time.monotonic())), seed=1949+len(attempts))
                method = "cached_ik_rerouted_transit"
                attempt["route"] = stats
            if transit is None:
                attempt["reason"] = "transit_unknown"
                continue
            transit = retime_native([dict(p, phase="transit") for p in transit], cfg, .6)
            points = transit + [dict(p, time_s=p["time_s"]+transit[-1]["time_s"]) for p in contact[1:]]
            attempt["method"] = method
            cert = certify(cell, geometry, rid, op, points, deadline=deadline)
            attempt["certificate"] = {k: v for k, v in cert.items() if k != "occupancy"}
            if not cert["ok"]:
                continue
            item = {"points": points, "certificate": cert, "pose": source["pose"],
                    "robot": rid, "weld": weld, "method": method,
                    "source_geometry": source["source_geometry"],
                    "source_trajectory_hash": source["source_trajectory"]}
            proof = stroke_proof(cell, item, store, pid)
            attempt["stroke_proof"] = proof
            if not proof["ok"]:
                continue
            phase_pack(cell, pid, item, store)
            if not cell.unchanged() or time.monotonic() >= deadline:
                attempt["reason"] = "scene_changed_or_deadline_before_publication"
                break
            payload = {"status": "certified", "cell": cell.cell_id, "program": pid,
                       "robot": rid, "weld": weld, "variants": [item],
                       "build_wall_s": time.monotonic()-started, "seed_repair": True}
            store.obtain("placement_primitive", key(cell, pid, rid, weld, planner, implementation),
                         lambda: payload)
            return {"status": "certified", "wall_s": time.monotonic()-started, "attempts": attempts}
    return {"status": "timeout_or_unknown", "wall_s": time.monotonic()-started, "attempts": attempts}
