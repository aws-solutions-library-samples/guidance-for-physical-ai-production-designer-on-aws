"""numpy collision kernels for a umarell working copy WITH the upstream fixes 02 + 03 applied.

``umarell_fast.kernels_np`` reproduces umarell HEAD's predicates (tool spheres checked against the other arm only,
centre-based floor rule). docs/upstream/patches/03 changes ``_config_is_free`` / ``_config_collision_info``:

* floor: no sphere SURFACE below ``FLOOR_Z_M`` (-0.05 m), base frames skipped by ``_fixed_frame_count``, the
  parametric tool spheres included (tool radius = ``_tool_static_radius``);
* tool spheres are also tested against the static boxes / spheres (tool radius + margin);
* tool vs the other arm unchanged (uniform pair radius); the diagnosis tests arm-only against the other arm.

These drop-ins implement exactly that in one broadcast per test (same voxel broad phase as kernels_np, sized for
the larger of the arm and tool radii). Agreement with the patched originals is checked by
``python -m umarell_fast.u2.kernels_fix --repo data/umarell_fast/repo_U2_fix --cell <cell> --n 2000`` (random
configurations; must report 100 %). Install only into a working copy that has patch 03 (``install`` refuses
otherwise).
"""
from __future__ import annotations

import numpy as np

from umarell_fast import kernels_np as K


def _floor_ok(rk, model, S: np.ndarray, T: np.ndarray, sphere_radius: float, sphere_radii, tool_r: float) -> bool:
    n0 = rk._fixed_frame_count(model)
    n_r = 0 if sphere_radii is None else len(sphere_radii)
    idx = np.arange(n0, S.shape[0])
    if idx.size:
        r = np.full(idx.size, float(sphere_radius))
        m = idx < n_r
        if m.any():
            r[m] = np.asarray(sphere_radii, float)[idx[m]]
        if np.any(S[idx, 2] - r < rk.FLOOR_Z_M):
            return False
    if T.size and np.any(T[:, 2] - tool_r < rk.FLOOR_Z_M):
        return False
    return True


def config_is_free_fix(model, col_data, q, base, yaw, obstacle_spheres, static_obstacles, sphere_radius: float = 0.12,
                       tool_cfg=None, sphere_radii=None, extra_margin: float = 0.0,
                       _tool_vs_moving: bool = True) -> bool:
    import pinocchio as pin
    from agents.trajectory.tools import robot_kinematics as rk
    pin.forwardKinematics(model, col_data, q)
    pin.updateFramePlacements(model, col_data)
    S = K.sphere_positions_np(model, col_data, base, yaw)
    T = K.tool_spheres_np(model, col_data, base, yaw, tool_cfg)
    tool_r = float(rk._tool_static_radius(sphere_radius, sphere_radii))
    if not _floor_ok(rk, model, S, T, sphere_radius, sphere_radii, tool_r):
        return False
    if obstacle_spheres:
        P, R = K._obstacle_array(obstacle_spheres)
        A = np.concatenate([S, T], 0) if (_tool_vs_moving and T.size) else S
        d2 = ((A[:, None, :] - P[None, :, :]) ** 2).sum(-1)
        lim = R[None, :] + sphere_radius
        if np.any(d2 < lim * lim):
            return False
    if static_obstacles:
        bc, bh, bm, sc, sr = K._compile_static(static_obstacles, extra_margin)
        X = S[2:]
        n_r = 0 if sphere_radii is None else len(sphere_radii)
        rad = np.full(X.shape[0], float(sphere_radius))
        if n_r > 2:
            k = min(n_r - 2, X.shape[0])
            rad[:k] = np.asarray(sphere_radii, float)[2:2 + k]
        if T.size:
            X = np.concatenate([X, T], 0)
            rad = np.concatenate([rad, np.full(T.shape[0], tool_r)])
        if bc.shape[0] >= K.GRID_MIN_BOXES:
            lo, shape, grid = K._box_grid(bc, bh, bm, float(rad.max()) if rad.size else float(sphere_radius))
            idx = np.floor((X - lo) / K.GRID_H).astype(int)
            inside = np.all((idx >= 0) & (idx < shape), 1)
            cand = np.zeros(X.shape[0], bool)
            ii = idx[inside]
            cand[inside] = grid[ii[:, 0], ii[:, 1], ii[:, 2]]
            Xb, radb = X[cand], rad[cand]
        else:
            Xb, radb = X, rad
        if bc.shape[0] and Xb.shape[0]:
            D = Xb[:, None, :] - bc[None, :, :]
            Cl = np.clip(D, -bh[None], bh[None])
            dist2 = ((D - Cl) ** 2).sum(-1)
            clr = radb[:, None] + bm[None, :]
            if np.any(dist2 < clr * clr):
                return False
        if sc.shape[0]:
            dist2 = ((X[:, None, :] - sc[None, :, :]) ** 2).sum(-1)
            clr = rad[:, None] + sr[None, :]
            if np.any(dist2 < clr * clr):
                return False
    return True


def make_collision_info_fix(orig):
    def _config_collision_info(model, col_data, q, base, yaw, obstacle_spheres, static_obstacles,
                               sphere_radius: float = 0.12, sphere_radii=None, extra_margin: float = 0.0,
                               tool_cfg=None):
        if config_is_free_fix(model, col_data, q, base, yaw, obstacle_spheres, static_obstacles,
                              sphere_radius=sphere_radius, tool_cfg=tool_cfg, sphere_radii=sphere_radii,
                              extra_margin=extra_margin, _tool_vs_moving=False):
            return None
        return orig(model, col_data, q, base, yaw, obstacle_spheres, static_obstacles, sphere_radius=sphere_radius,
                    sphere_radii=sphere_radii, extra_margin=extra_margin, tool_cfg=tool_cfg)
    _config_collision_info._uf_fast = True
    return _config_collision_info


def install() -> dict:
    from agents.trajectory.tools import robot_kinematics as rk, planning_tools as pt, rrt_planner as rp
    if hasattr(rk, "_self_free") or hasattr(rk, "gate_groups"):
        # umarell 76ea13a (see umarell_fast/u1/kernels.py:install): eccb1bc's predicate, would drop HEAD's checks
        raise RuntimeError("umarell_fast.u2.kernels_fix: drop-in kernels target umarell eccb1bc; "
                           "not installable on 76ea13a")
    if not hasattr(rk, "_tool_static_radius") or not hasattr(rk, "FLOOR_Z_M"):
        raise RuntimeError("kernels 'fix' need a working copy with docs/upstream/patches/03 applied")
    orig = {"_config_is_free": rk._config_is_free, "_config_collision_info": rk._config_collision_info}
    rk._config_is_free = config_is_free_fix
    rp._config_is_free = config_is_free_fix
    rk._check_robot_pair_collision = K.check_robot_pair_collision_np
    pt._check_robot_pair_collision = K.check_robot_pair_collision_np
    if not getattr(rk._config_collision_info, "_uf_fast", False):
        rk._config_collision_info = make_collision_info_fix(rk._config_collision_info)
    from umarell_fast import fastio
    fastio.install()
    return orig


def _bench(repo: str, cell: str, n: int, seed: int = 0) -> dict:
    """Agreement of the fix kernels with the patched originals on random configurations (+ other arm moving)."""
    import os
    import sys
    import time
    from pathlib import Path
    from umarell_fast.u2 import session as S
    run = Path(repo).resolve().parent / f"kbench_fix_{cell}"
    base = S.make_cell_base(Path(repo), run, cell)
    S.setup(repo, cell, S.env_for(Path(repo), cell, base, run / "scratch"), kernels="none")
    from agents.trajectory.tools import robot_kinematics as rk
    from agents.trajectory.tools.robot_kinematics import _load_pinocchio_model, _load_robot_config
    from agents.trajectory.tools.robot_kinematics import load_static_obstacles, _cell_sphere_radii, _config_spheres
    orig_free, orig_info = rk._config_is_free, rk._config_collision_info
    rng = np.random.default_rng(seed)
    static = load_static_obstacles(cell)
    out = {"cell": cell, "n": n, "free": {"agree": 0, "n": 0}, "info": {"agree": 0, "n": 0}, "t_orig": 0.0,
           "t_np": 0.0}
    robots = ["robot1", "robot2"]
    models = {r: _load_pinocchio_model(cell, r) for r in robots}
    cfgs = {r: _load_robot_config(cell, r) for r in robots}
    fast_info = make_collision_info_fix(orig_info)
    for i in range(n):
        r, o = robots[i % 2], robots[(i + 1) % 2]
        model, _ = models[r]
        c = cfgs[r]
        lo = np.array(c.get("joint_limits_lower", [-3] * 6)); hi = np.array(c.get("joint_limits_upper", [3] * 6))
        q = rng.uniform(lo, hi)
        mo, _ = models[o]
        co = cfgs[o]
        qo = rng.uniform(np.array(co.get("joint_limits_lower", [-3] * 6)), np.array(co.get("joint_limits_upper",
                                                                                          [3] * 6)))
        mov = [(p, 0.14) for p in _config_spheres(mo, mo.createData(), qo, np.array(co["base_position"]),
                                                   float(co.get("base_orientation_yaw", 0.0)),
                                                   co.get("tool_collision"))] if i % 3 else []
        base_r = np.array(c["base_position"]); yaw = float(c.get("base_orientation_yaw", 0.0))
        tool = c.get("tool_collision")
        radii = _cell_sphere_radii(cell, model, tool)
        em = float(rng.choice([0.0, 0.02]))
        d = model.createData()
        t0 = time.perf_counter(); a = orig_free(model, d, q, base_r, yaw, mov, static, 0.12, tool, radii, em)
        t1 = time.perf_counter(); b = config_is_free_fix(model, d, q, base_r, yaw, mov, static, 0.12, tool, radii, em)
        t2 = time.perf_counter()
        out["t_orig"] += t1 - t0; out["t_np"] += t2 - t1
        out["free"]["n"] += 1; out["free"]["agree"] += int(a == b)
        out["free"]["n_free_orig"] = out["free"].get("n_free_orig", 0) + int(bool(a))
        if i % 4 == 0:
            ia = orig_info(model, d, q, base_r, yaw, mov, static, sphere_radius=0.12, sphere_radii=radii,
                           extra_margin=em, tool_cfg=tool)
            ib = fast_info(model, d, q, base_r, yaw, mov, static, sphere_radius=0.12, sphere_radii=radii,
                           extra_margin=em, tool_cfg=tool)
            out["info"]["n"] += 1; out["info"]["agree"] += int(ia == ib)
    out["free"]["rate"] = out["free"]["agree"] / max(1, out["free"]["n"])
    out["info"]["rate"] = out["info"]["agree"] / max(1, out["info"]["n"])
    out["us_orig"] = round(1e6 * out["t_orig"] / max(1, n), 1)
    out["us_np"] = round(1e6 * out["t_np"] / max(1, n), 1)
    return out


if __name__ == "__main__":
    import argparse
    import json
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", required=True)
    ap.add_argument("--cell", default="welding_cell_2_small")
    ap.add_argument("--n", type=int, default=2000)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out = Path(a.out).resolve() if a.out else None      # setup() chdirs into the working copy
    res = _bench(str(Path(a.repo).resolve()), a.cell, a.n)
    print(json.dumps(res, indent=1))
    if out:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(res, indent=1))
