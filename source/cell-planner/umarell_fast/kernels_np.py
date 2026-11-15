"""numpy-vectorised drop-ins for umarell's per-configuration collision kernels (umarell HEAD eccb1bc).

Prototype of the profile track (docs/uf_profile.md §5). Same signatures and the same boolean / numeric results as

* ``agents.trajectory.tools.robot_kinematics._config_is_free``      -> :func:`config_is_free_np`
* ``agents.trajectory.tools.robot_kinematics._check_robot_pair_collision`` -> :func:`check_robot_pair_collision_np`
* ``agents.trajectory.tools.robot_kinematics._sphere_positions`` / ``_tool_spheres`` (array forms)

The originals walk every sphere x obstacle pair in Python (one ``np.linalg.norm`` / ``np.clip`` per pair); for the
737 fixture boxes of welding_cell_2t that is ~30 000 numpy calls per OMPL state check. Here every check is one
broadcast. The static obstacle list is compiled to arrays once per list object (cached by ``id``; the planner
passes the same list for a whole segment).

Install into a scratch umarell copy with :func:`install` (monkeypatches the module attributes, which every caller
resolves at call time: ``ompl_planner`` imports ``_config_is_free`` inside its functions). Never import this into
third_party/ or ~/industrial/starter.
"""
from __future__ import annotations

import numpy as np

_STATIC_CACHE: dict = {}
_LINK_STEP_DEFAULT = 0.12


def _compile_static(static_obstacles: list, extra_margin: float):
    key = (id(static_obstacles), len(static_obstacles), float(extra_margin))
    hit = _STATIC_CACHE.get(key)
    if hit is not None and hit[0] is static_obstacles:
        return hit[1]
    bc, bh, bm, sc, sr, sm = [], [], [], [], [], []
    for obs in static_obstacles:
        if obs.get("is_workpiece", False):
            continue
        margin = float(obs.get("collision_margin", 0.05)) + extra_margin
        t = obs.get("type", "")
        if t == "box":
            bc.append(obs["center"]); bh.append(obs["half_extents"]); bm.append(margin)
        elif t == "sphere":
            sc.append(obs["center"]); sr.append(float(obs["radius"])); sm.append(margin)
    comp = (np.asarray(bc, float).reshape(-1, 3), np.asarray(bh, float).reshape(-1, 3), np.asarray(bm, float),
            np.asarray(sc, float).reshape(-1, 3), np.asarray(sr, float) + np.asarray(sm, float))
    if len(_STATIC_CACHE) > 64:
        _STATIC_CACHE.clear()
    _STATIC_CACHE[key] = (static_obstacles, comp)
    return comp


_GRID_CACHE: dict = {}
GRID_H = 0.05          # broad-phase voxel size (m)
GRID_MIN_BOXES = 64    # below this the plain broadcast is already cheap


def _box_grid(bc: np.ndarray, bh: np.ndarray, bm: np.ndarray, r_max: float):
    """Conservative broad phase for the static boxes: a voxel is marked when ANY point in it could be within
    ``r_max + margin`` of a box (box AABB inflated by clearance + one voxel). A sphere centre in an unmarked voxel
    (or outside the grid) cannot touch any box, so only centres in marked voxels go to the exact test."""
    key = (id(bc), float(r_max))
    hit = _GRID_CACHE.get(key)
    if hit is not None and hit[0] is bc:
        return hit[1]
    infl = (r_max + bm.max() + GRID_H)
    lo = (bc - bh).min(0) - infl
    hi = (bc + bh).max(0) + infl
    shape = np.maximum(1, np.ceil((hi - lo) / GRID_H).astype(int))
    grid = np.zeros(tuple(shape), bool)
    for c, h, m in zip(bc, bh, bm):
        e = h + r_max + m + GRID_H
        i0 = np.clip(np.floor((c - e - lo) / GRID_H).astype(int), 0, shape - 1)
        i1 = np.clip(np.floor((c + e - lo) / GRID_H).astype(int), 0, shape - 1)
        grid[i0[0]:i1[0] + 1, i0[1]:i1[1] + 1, i0[2]:i1[2] + 1] = True
    g = (lo, shape, grid)
    _GRID_CACHE[key] = (bc, g)
    return g


def _yaw_world(P: np.ndarray, base, yaw: float) -> np.ndarray:
    cy, sy = np.cos(yaw), np.sin(yaw)
    out = np.empty_like(P)
    out[:, 0] = base[0] + P[:, 0] * cy - P[:, 1] * sy
    out[:, 1] = base[1] + P[:, 0] * sy + P[:, 1] * cy
    out[:, 2] = base[2] + P[:, 2]
    return out


_FILL_CACHE: dict = {}


def _fill_plan(model, J: np.ndarray, step: float):
    """Link-fill layout per model: for consecutive joint origins a -> b the original emits a + d*k*step for
    k = 1..int(|b-a|/step) while k*step < |b-a|. |b-a| is fixed by the link geometry of a serial chain (it does not
    depend on q), so the layout is computed once; it is re-validated against the current segment lengths and
    rebuilt if they ever differ (e.g. a prismatic joint)."""
    seg = np.linalg.norm(J[1:] - J[:-1], axis=1)
    key = (id(model), float(step))
    hit = _FILL_CACHE.get(key)
    if hit is not None and hit[0].shape == seg.shape and np.allclose(hit[0], seg, atol=1e-9):
        return hit[1], hit[2], hit[3]
    ia, tt = [], []
    for j, L in enumerate(seg):
        n = int(L / step)
        if n < 1:
            continue
        for k in range(1, n + 1):
            t = k * step
            if t >= L:
                break
            ia.append(j); tt.append(t)
    ia = np.asarray(ia, int); tt = np.asarray(tt, float)
    _FILL_CACHE[key] = (seg, ia, tt, None)
    return ia, tt, None


def sphere_positions_np(model, data, base, yaw, step: float = None) -> np.ndarray:
    """Array form of ``_sphere_positions`` (frame origins in frame order, then link-fill spheres)."""
    from agents.trajectory.tools import robot_kinematics as rk
    step = rk._LINK_SPHERE_STEP_M if step is None else step
    F = np.array([data.oMf[i].translation for i in range(len(model.frames))], float).reshape(-1, 3)
    Fw = _yaw_world(F, base, yaw)
    if step <= 0:
        return Fw
    J = _yaw_world(np.array([data.oMi[j].translation for j in range(1, model.njoints)], float).reshape(-1, 3),
                   base, yaw)
    ia, tt, _ = _fill_plan(model, J, step)
    if ia.size == 0:
        return Fw
    A = J[ia]
    D = J[ia + 1] - A
    L = np.linalg.norm(D, axis=1)
    return np.concatenate([Fw, A + D * (tt / L)[:, None]], 0)


def tool_spheres_np(model, data, base, yaw, tool_cfg) -> np.ndarray:
    from agents.trajectory.tools import robot_kinematics as rk
    offs = rk._tool_offsets(tool_cfg)
    if not offs:
        return np.zeros((0, 3))
    token = tool_cfg.get("mount_link", "")
    mkey = (id(model), token)
    idx = rk._TOOL_MOUNT_IDX.get(mkey)
    if idx is None:
        idx = next((i for i, f in enumerate(model.frames) if token and token in f.name), -1)
        rk._TOOL_MOUNT_IDX[mkey] = idx
    if idx < 0:
        return np.zeros((0, 3))
    O = _offs_array(offs)
    oMf = data.oMf[idx]
    P = O @ np.asarray(oMf.rotation).T + np.asarray(oMf.translation)
    return _yaw_world(P, base, yaw)


_OFFS_ARR: dict = {}


def _offs_array(offs: list) -> np.ndarray:
    a = _OFFS_ARR.get(id(offs))
    if a is None or a[0] is not offs:
        a = (offs, np.asarray(offs, float).reshape(-1, 3))
        _OFFS_ARR[id(offs)] = a
    return a[1]


_OBS_ARR: dict = {}


def _obstacle_array(obstacle_spheres: list):
    a = _OBS_ARR.get(id(obstacle_spheres))
    if a is not None and a[0] is obstacle_spheres and a[1] == len(obstacle_spheres):
        return a[2], a[3]
    if obstacle_spheres:
        P = np.asarray([p for p, _ in obstacle_spheres], float).reshape(-1, 3)
        R = np.asarray([r for _, r in obstacle_spheres], float)
    else:
        P, R = np.zeros((0, 3)), np.zeros(0)
    if len(_OBS_ARR) > 256:
        _OBS_ARR.clear()
    _OBS_ARR[id(obstacle_spheres)] = (obstacle_spheres, len(obstacle_spheres), P, R)
    return P, R


def config_is_free_np(model, col_data, q, base, yaw, obstacle_spheres, static_obstacles, sphere_radius: float = 0.12,
                      tool_cfg=None, sphere_radii=None, extra_margin: float = 0.0, _tool_vs_moving: bool = True) -> bool:
    import pinocchio as pin
    pin.forwardKinematics(model, col_data, q)
    pin.updateFramePlacements(model, col_data)
    S = sphere_positions_np(model, col_data, base, yaw)
    # floor (arm links only; skip base frames)
    if S.shape[0] > 3 and np.any(S[3:, 2] < -0.05):
        return False
    if obstacle_spheres:
        P, R = _obstacle_array(obstacle_spheres)
        T = tool_spheres_np(model, col_data, base, yaw, tool_cfg) if _tool_vs_moving else np.zeros((0, 3))
        A = np.concatenate([S, T], 0) if T.size else S
        d2 = ((A[:, None, :] - P[None, :, :]) ** 2).sum(-1)
        lim = R[None, :] + sphere_radius
        if np.any(d2 < lim * lim):
            return False
    if static_obstacles:
        bc, bh, bm, sc, sr = _compile_static(static_obstacles, extra_margin)
        X = S[2:]
        n_r = 0 if sphere_radii is None else len(sphere_radii)
        rad = np.full(X.shape[0], float(sphere_radius))
        if n_r > 2:
            k = min(n_r - 2, X.shape[0])
            rad[:k] = np.asarray(sphere_radii, float)[2:2 + k]
        if bc.shape[0] >= GRID_MIN_BOXES:
            lo, shape, grid = _box_grid(bc, bh, bm, float(rad.max()) if rad.size else float(sphere_radius))
            idx = np.floor((X - lo) / GRID_H).astype(int)
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


def check_robot_pair_collision_np(s1, s2, radius: float = 0.12):
    A = np.asarray(s1, float).reshape(-1, 3)
    B = np.asarray(s2, float).reshape(-1, 3)
    if not A.size or not B.size:
        return False, float("inf")
    d = np.sqrt(((A[:, None, :] - B[None, :, :]) ** 2).sum(-1)).min() - 2 * radius
    return bool(d < 0), float(d)


def make_collision_info_fast(orig):
    """``_config_collision_info`` with a vectorised fast path: the original's predicate is the same as
    ``_config_is_free`` minus the tool-vs-other-arm test, so a configuration that passes that predicate returns
    ``None`` without the per-pair loops; anything else is diagnosed by the original (identical dict)."""
    def _config_collision_info(model, col_data, q, base, yaw, obstacle_spheres, static_obstacles,
                               sphere_radius: float = 0.12, sphere_radii=None, extra_margin: float = 0.0):
        if config_is_free_np(model, col_data, q, base, yaw, obstacle_spheres, static_obstacles,
                             sphere_radius=sphere_radius, tool_cfg=None, sphere_radii=sphere_radii,
                             extra_margin=extra_margin, _tool_vs_moving=False):
            return None
        return orig(model, col_data, q, base, yaw, obstacle_spheres, static_obstacles,
                    sphere_radius=sphere_radius, sphere_radii=sphere_radii, extra_margin=extra_margin)
    _config_collision_info._uf_fast = True
    return _config_collision_info


def install(plus: bool = False) -> dict:
    """Monkeypatch the umarell module attributes (scratch copies only). Returns the originals."""
    from agents.trajectory.tools import robot_kinematics as rk, planning_tools as pt, rrt_planner as rp
    if hasattr(rk, "_self_free") or hasattr(rk, "gate_groups"):
        # umarell 76ea13a: its predicate also checks the arm against itself, the link envelope, the tool hulls and
        # the workpiece rule, and the pair check takes groups / radii / stopping distances; this drop-in re-creates
        # eccb1bc's predicate and would silently drop all of that (umarell_fast/COORDINATION.md, 2026-10-23)
        raise RuntimeError("umarell_fast.kernels_np: drop-in kernels target umarell eccb1bc; not installable on 76ea13a")
    orig = {"_config_is_free": rk._config_is_free, "_check_robot_pair_collision": rk._check_robot_pair_collision}
    rk._config_is_free = config_is_free_np
    rp._config_is_free = config_is_free_np
    rk._check_robot_pair_collision = check_robot_pair_collision_np
    pt._check_robot_pair_collision = check_robot_pair_collision_np
    if plus:
        # also: vectorised fast path for the per-sample collision diagnosis + parse-once YAML
        orig["_config_collision_info"] = rk._config_collision_info
        if not getattr(rk._config_collision_info, "_uf_fast", False):
            rk._config_collision_info = make_collision_info_fast(rk._config_collision_info)
        from umarell_fast import fastio
        fastio.install()
    return orig
