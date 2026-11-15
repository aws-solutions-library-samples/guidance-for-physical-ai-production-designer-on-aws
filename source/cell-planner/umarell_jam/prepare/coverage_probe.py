"""Diagnose missing welds with whole-arm IK branches and explicit stroke checks (session tools/coverage_probe.py).

This produces candidates and rejection witnesses. Only the existing full
controller/geometry certificate can authorize a candidate for the planner.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


from ..cell import Cell
from ..electrode_geometry import RefinedElectrodeGeometry
from ..native import setup
from ..store import atomic_json


class BranchIK:
    def __init__(self, geometry, rid):
        arm = geometry.arms[rid]
        self.model, self.frame = arm["m"], arm["tcp"]
        self.data = self.model.createData()
        self.world = arm["world"]
        cfg = geometry.cell.robots[rid]
        self.lower = np.asarray(cfg["joint_limits_lower"]) + 1e-8
        self.upper = np.asarray(cfg["joint_limits_upper"]) - 1e-8
        self.offset = pin.SE3(np.eye(3), np.asarray(cfg.get("tcp_offset", [0., 0., 0.])))

    def solve(self, position, orientation, seed, iterations=70):
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
                               bounds=(self.lower, self.upper), max_nfev=iterations,
                               ftol=1e-9, xtol=1e-9, gtol=1e-9)
        error = residual(result.x)
        if np.linalg.norm(error[:3]) > 1e-5 or np.linalg.norm(error[3:]) > 1e-4:
            return None
        return result.x


def probe(layout, cid, weld, rid, budget, seed_count, output):
    layout.activate()
    cell = Cell(layout.workspace, cid)
    setup(layout.umarell, layout.workspace,
          layout.prepare_runs / "coverage-v2/probe-workers", cid)
    geometry = RefinedElectrodeGeometry(cell)
    solver = BranchIK(geometry, rid)
    from agents.trajectory.tools.weld_orientation import candidates, policy
    from agents.trajectory.tools import robot_kinematics as rk
    model, _ = rk._load_pinocchio_model(cid, rid)
    data = model.createData()
    cfg = cell.robots[rid]
    native_obstacles = rk.load_static_obstacles(cid)
    native_policy = rk._load_clearance_policy(cid)
    native_radii = rk._cell_sphere_radii(cid, model, cfg.get("tool_collision"))

    def native_check(q):
        return rk._config_collision_info(
            model, data, np.asarray(q), np.asarray(cfg["base_position"]),
            cfg["base_orientation_yaw"], [], native_obstacles,
            sphere_radius=native_policy["normal"], sphere_radii=native_radii)

    pid, program = next((pid, p) for pid, p in cell.programs.items()
                        if not cell.program_errors[pid] and any(o["id"] == weld for o in p["operations"]))
    op = next(o for o in program["operations"] if o["id"] == weld)
    states = {r: np.radians(c["home_pose_deg"]).tolist() for r, c in cell.robots.items()}
    home = np.asarray(states[rid])
    rng = np.random.default_rng(20261110)
    seeds = [home] + [rng.uniform(solver.lower, solver.upper) for _ in range(seed_count - 1)]
    poses = [(0., 0., 0., op["orientation"])] + candidates(op["orientation"], policy(cfg))
    # Visit spins and tilt directions broadly instead of exhausting one local region.
    poses = [poses[0]] + [poses[i] for i in rng.permutation(np.arange(1, len(poses)))]
    counts, witnesses, solutions = Counter(), {}, []
    started = time.monotonic()
    axis = np.asarray(op["approach_vector"], dtype=float)
    axis /= np.linalg.norm(axis)
    offset = program["extensions"]["weld_params"]["approach_offset_z"]
    seen = set()

    def reject(reason, witness=None):
        counts[reason] += 1
        if witness is not None and reason not in witnesses:
            witnesses[reason] = witness

    def save():
        result = {"cell": cid, "program": pid, "robot": rid, "weld": weld,
                  "geometry": cell.geometry_hash, "seed": 20261110,
                  "budget_s": budget, "wall_s": time.monotonic() - started,
                  "counts": dict(counts), "witnesses": witnesses, "solutions": solutions,
                  "status": "candidates" if solutions else "timeout_or_unknown"}
        atomic_json(output, result)
        return result

    for spin, tilt, direction, orientation in poses:
        if time.monotonic() - started >= budget:
            break
        hit = geometry.target_tool_check(rid, op["position"], orientation)
        if hit:
            reject("target_tool_collision", hit)
            continue
        counts["poses"] += 1
        for seed in seeds:
            if time.monotonic() - started >= budget:
                break
            counts["ik_attempts"] += 1
            q = solver.solve(op["position"], orientation, seed)
            if q is None:
                reject("ik_did_not_converge")
                continue
            key = tuple(np.round(q, 3))
            if key in seen:
                reject("duplicate_branch")
                continue
            seen.add(key)
            counts["ik_solutions"] += 1
            hit = geometry.check({**states, rid: q.tolist()}, {rid: op["position"]})
            if hit:
                reject("endpoint_geometry", {"q": q.tolist(), "collision": hit})
                continue
            hit = native_check(q)
            if hit:
                reject("endpoint_native", {"q": q.tolist(), "collision": hit})
                continue
            counts["clear_endpoints"] += 1
            stroke = [q.tolist()]
            # Grow outward from the contact, retaining the entire arm branch.
            for distance in np.linspace(0., offset, 13)[1:]:
                q_next = solver.solve(np.asarray(op["position"]) - axis * distance, orientation, q)
                if q_next is None or np.max(abs(q_next - q)) > .3:
                    reject("stroke_ik_branch", {"distance_m": float(distance)})
                    break
                hit = geometry.check({**states, rid: q_next.tolist()}, {rid: op["position"]})
                if hit is None:
                    hit = native_check(q_next)
                if hit:
                    reject("stroke_collision", {"distance_m": float(distance), "collision": hit})
                    break
                stroke.append(q_next.tolist())
                q = q_next
            if len(stroke) == 13:
                solutions.append({"pose": {"spin": spin, "tilt": tilt,
                                          "tilt_direction": direction, "orientation": orientation},
                                  "stroke": list(reversed(stroke)),
                                  "seed": seed.tolist()})
                counts["clear_strokes"] += 1
                print(json.dumps({"cell": cid, "robot": rid, "weld": weld,
                                  "clear_strokes": len(solutions), "seconds": time.monotonic()-started}),
                      flush=True)
                save()
                if len(solutions) >= 8:
                    return save()
        save()
    return save()

