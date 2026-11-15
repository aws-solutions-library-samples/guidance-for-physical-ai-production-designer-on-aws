"""Bounded joint-space candidate routing; final certification is mandatory."""
from __future__ import annotations

import time
import numpy as np

from .trajectory import retime_native


def route(cell, geometry, rid, start, goal, budget_s=15., seed=20261110):
    """Bidirectional RRT with deterministic sampling and collision-checked shortcuts.

    Discrete search checks are filters only. The caller must certify the complete
    exported interpolant before putting this candidate in a reusable cache.
    """
    from agents.trajectory.tools import robot_kinematics as rk
    model, _ = rk._load_pinocchio_model(cell.cell_id, rid)
    data = model.createData()
    cfg = cell.robots[rid]
    obstacles = rk.load_static_obstacles(cell.cell_id)
    policy = rk._load_clearance_policy(cell.cell_id)
    radii = rk._cell_sphere_radii(cell.cell_id, model, cfg.get("tool_collision"))
    base = np.asarray(cfg["base_position"])
    states = {r: np.radians(c["home_pose_deg"]).tolist() for r, c in cell.robots.items()}
    lower, upper = np.asarray(cfg["joint_limits_lower"]), np.asarray(cfg["joint_limits_upper"])
    deadline = time.monotonic() + budget_s
    checks = 0

    def free(q):
        nonlocal checks
        checks += 1
        if time.monotonic() >= deadline:
            return False
        if rk._config_collision_info(model, data, q, base, cfg["base_orientation_yaw"], [], obstacles,
                                     sphere_radius=policy["normal"], sphere_radii=radii):
            return False
        return geometry.check({**states, rid: q.tolist()}) is None

    def line(a, b):
        steps = max(1, int(np.ceil(np.max(abs(b-a))/.035)))
        return all(free(a + t*(b-a)) for t in np.linspace(0., 1., steps+1)[1:])

    start, goal = np.asarray(start), np.asarray(goal)
    if not free(start) or not free(goal):
        return None, {"reason": "endpoint_collision_or_deadline", "checks": checks}
    path = [start, goal] if line(start, goal) else None
    rng = np.random.default_rng(seed)
    trees = [([start], [-1]), ([goal], [-1])]
    swapped = False

    def grow(tree, target):
        nodes, parents = tree
        index = int(np.argmin(np.linalg.norm(np.asarray(nodes)-target, axis=1)))
        q = nodes[index]
        delta = target-q
        norm = np.linalg.norm(delta)
        dest = target if norm <= .3 else q + delta*(.3/norm)
        if not line(q, dest):
            return None
        nodes.append(dest)
        parents.append(index)
        return len(nodes)-1

    def backtrack(tree, index):
        nodes, parents = tree
        out = []
        while index != -1:
            out.append(nodes[index])
            index = parents[index]
        return out[::-1]

    iterations = 0
    while path is None and time.monotonic() < deadline:
        target = trees[1][0][-1] if rng.random() < .2 else rng.uniform(lower, upper)
        index = grow(trees[0], target)
        iterations += 1
        if index is not None:
            destination = trees[0][0][index]
            other = grow(trees[1], destination)
            while other is not None and time.monotonic() < deadline:
                if np.max(abs(trees[1][0][other]-destination)) < 1e-9:
                    a, b = backtrack(trees[0], index), backtrack(trees[1], other)
                    path = a + b[-2::-1]
                    if swapped:
                        path.reverse()
                    break
                other = grow(trees[1], destination)
        trees.reverse()
        swapped = not swapped
    if path is None:
        return None, {"reason": "bounded_branch_route_unknown", "checks": checks, "iterations": iterations}
    # Greedy furthest-visible shortcuts preserve the checked endpoints.
    simplified, index = [path[0]], 0
    while index < len(path)-1:
        target = len(path)-1
        while target > index+1 and not line(path[index], path[target]):
            target -= 1
        simplified.append(path[target])
        index = target
    speed = np.asarray(cfg["joint_velocity_limits"])*.6
    points, elapsed = [], 0.
    for i, q in enumerate(simplified):
        if i:
            elapsed += max(.2, float(np.max(abs(q-simplified[i-1])/speed))*1.5)
        points.append({"positions": q.tolist(), "time_s": elapsed, "phase": "transit"})
    return retime_native(points, cfg, .6), {
        "method": "bounded_bidirectional_joint_rrt", "checks": checks,
        "iterations": iterations, "knots": len(points), "seed": seed}
