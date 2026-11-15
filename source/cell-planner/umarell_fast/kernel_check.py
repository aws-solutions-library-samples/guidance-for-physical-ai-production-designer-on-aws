"""Install-time guard for the vectorised collision kernels (umarell_fast.kernels_np).

The numpy kernels re-implement umarell's ``_config_is_free`` / ``_config_collision_info`` predicates. They were
verified against umarell HEAD eccb1bc (docs/uf_profile.md section 5: 100 % agreement on 9000 configurations), but a
working copy with the upstream fixes (02 tool spheres, 03 tool vs floor / obstacles) changes the predicate. So
before a run trusts them, :func:`selfcheck` compares the installed kernel with the original on random
configurations of every arm of the cell -- with the cell's static obstacles, its per-sphere radii and tool, and the
other arm as a moving obstacle -- and the caller falls back to the original functions on any disagreement.
"""
from __future__ import annotations

import time

import numpy as np


def selfcheck(cell_id: str, originals: dict, n: int = 120, seed: int = 0, max_s: float = 60.0) -> dict:
    """Compare the installed ``robot_kinematics`` kernels with ``originals`` (as returned by
    ``kernels_np.install``). Returns ``{"ok", "checked", "mismatches", "examples", "wall_s"}``."""
    from agents.trajectory.tools import robot_kinematics as rk
    from agents.trajectory.tools.planning_tools import _cell_robots

    t0 = time.monotonic()
    rng = np.random.default_rng(seed)
    robots = sorted(_cell_robots(cell_id))
    static = rk.load_static_obstacles(cell_id)
    models = {}
    for ns in robots:
        cfg = rk._load_robot_config(cell_id, ns)
        model, _ = rk._load_pinocchio_model(cell_id, ns)
        models[ns] = (model, cfg)
    checked = mism = 0
    examples: list = []
    orig_free = originals.get("_config_is_free")
    orig_info = originals.get("_config_collision_info")
    new_free = rk._config_is_free
    new_info = rk._config_collision_info
    for i in range(n):
        if time.monotonic() - t0 > max_s:
            break
        for ns in robots:
            model, cfg = models[ns]
            base = np.array(cfg["base_position"], float)
            yaw = float(cfg.get("base_orientation_yaw", 0.0))
            tool = cfg.get("tool_collision")
            lo = np.array(cfg.get("joint_limits_lower", [-3.0] * model.nq), float)[:model.nq]
            hi = np.array(cfg.get("joint_limits_upper", [3.0] * model.nq), float)[:model.nq]
            q = lo + (hi - lo) * rng.random(model.nq)
            radii = rk._cell_sphere_radii(cell_id, model, tool)
            moving = []
            others = [o for o in robots if o != ns]
            if others and i % 2 == 0:
                om, ocfg = models[others[0]]
                oq = (np.array(ocfg.get("joint_limits_lower", [-3.0] * om.nq), float)[:om.nq]
                      + rng.random(om.nq) * 2.0 - 1.0)
                for p in rk._config_spheres(om, om.createData(), oq, np.array(ocfg["base_position"], float),
                                            float(ocfg.get("base_orientation_yaw", 0.0)),
                                            ocfg.get("tool_collision")):
                    moving.append((np.asarray(p, float), 0.12))
            margin = float(rng.choice([0.0, 0.02, 0.05]))
            kw = dict(sphere_radius=0.12, tool_cfg=tool, sphere_radii=radii, extra_margin=margin)
            if orig_free is not None:
                a = bool(orig_free(model, model.createData(), q, base, yaw, moving, static, **kw))
                try:
                    b = bool(new_free(model, model.createData(), q, base, yaw, moving, static, **kw))
                except Exception as exc:  # noqa: BLE001 -- a kernel that cannot run disagrees
                    b = f"error: {exc!r}"
                checked += 1
                if a != b:
                    mism += 1
                    if len(examples) < 5:
                        examples.append({"fn": "_config_is_free", "robot": ns, "q": q.round(4).tolist(),
                                         "orig": a, "new": b})
            if orig_info is not None:
                ikw = dict(sphere_radius=0.12, sphere_radii=radii, extra_margin=margin)
                try:
                    import inspect
                    if "tool_cfg" in inspect.signature(orig_info).parameters:
                        ikw["tool_cfg"] = tool
                except (TypeError, ValueError):
                    pass
                a = orig_info(model, model.createData(), q, base, yaw, moving, static, **ikw)
                try:
                    b = new_info(model, model.createData(), q, base, yaw, moving, static, **ikw)
                except Exception as exc:  # noqa: BLE001
                    b = {"error": repr(exc)}
                checked += 1
                if (a is None) != (b is None) or (a is not None and a != b):
                    mism += 1
                    if len(examples) < 5:
                        examples.append({"fn": "_config_collision_info", "robot": ns, "q": q.round(4).tolist(),
                                         "orig": a, "new": b})
    return {"ok": mism == 0 and checked > 0, "checked": checked, "mismatches": mism, "examples": examples,
            "wall_s": round(time.monotonic() - t0, 2)}


def restore(originals: dict) -> None:
    """Put the original umarell functions back (after a failed self-check)."""
    from agents.trajectory.tools import robot_kinematics as rk, planning_tools as pt, rrt_planner as rp
    if "_config_is_free" in originals:
        rk._config_is_free = originals["_config_is_free"]
        rp._config_is_free = originals["_config_is_free"]
    if "_check_robot_pair_collision" in originals:
        rk._check_robot_pair_collision = originals["_check_robot_pair_collision"]
        pt._check_robot_pair_collision = originals["_check_robot_pair_collision"]
    if "_config_collision_info" in originals:
        rk._config_collision_info = originals["_config_collision_info"]
