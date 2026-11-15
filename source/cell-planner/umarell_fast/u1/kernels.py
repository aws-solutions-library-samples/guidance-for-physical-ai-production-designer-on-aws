"""U1 vectorised collision kernels for umarell (HEAD eccb1bc), drop-in replacements.

The approach of ``env/kinematics.py`` (own numpy forward kinematics, batched over any leading dimension, no
pinocchio in the hot loop) ported into umarell's ``robot_kinematics`` as drop-ins with the same signatures and the
same results:

=========================================  ===============================================================
umarell (robot_kinematics)                 U1
=========================================  ===============================================================
``_config_is_free``                        :func:`config_is_free` (numpy FK + broadcast tests, voxel broad
                                           phase for >= 64 boxes)
``_config_collision_info``                 :func:`make_collision_info` (vectorised "is it free" fast path;
                                           a colliding configuration is diagnosed by the original, so the
                                           dict is identical)
``_config_spheres``                        :func:`config_spheres` (list of the same 77 centres, same order)
``_check_robot_pair_collision``            :func:`check_robot_pair_collision`
``inter_robot_clearance``                  :func:`inter_robot_clearance` (all samples of the shared clock in
                                           one batched FK + one pairwise-distance pass per chunk)
=========================================  ===============================================================

The forward kinematics is built from the loaded pinocchio model itself (joint placements, revolute axes, frame
parents and placements), so it works for any serial revolute arm umarell loads; a model with other joint types falls
back to the original functions. Two predicate variants are mirrored, chosen at call time from the installed umarell:

* ``orig``: umarell HEAD eccb1bc (tool spheres only against the other arm; floor rule on sphere centres of
  ``spheres[3:]``);
* ``fix03``: with ``docs/upstream/patches/03_tool_floor_obstacles.diff`` applied (``robot_kinematics._floor_breach``
  exists): tool spheres also against the floor and static obstacles at the cell tool radius, radius-aware floor rule
  from the first non-fixed frame.

Equality with the originals is tested in ``tests/test_umarell_fast_u1.py`` (booleans exact, distances to 1e-9).
Install into a working copy with :func:`install` (never into third_party/).
"""
from __future__ import annotations

import os
import sys
import threading

import numpy as np

_LOCK = threading.RLock()
STATS = {"fast_free": 0, "fallback_free": 0, "irc_fast": 0, "irc_fallback": 0}

GRID_H = 0.05          # broad-phase voxel size (m)
GRID_MIN_BOXES = 64    # below this the plain broadcast is already cheap
IRC_CHUNK = 64         # time samples per pairwise-distance chunk in inter_robot_clearance


# ----------------------------------------------------------------------------- forward kinematics
def _rot_axis(kind: str, axis, c, s):
    """Rotation matrices (..., 3, 3) about a revolute axis for cos/sin arrays c, s (any shape)."""
    c = np.asarray(c, float); s = np.asarray(s, float)
    R = np.zeros(c.shape + (3, 3))
    if kind in ("RX", "RY", "RZ"):
        k = "XYZ".index(kind[1])
        i, j = [(1, 2), (2, 0), (0, 1)][k]
        R[..., k, k] = 1.0
        R[..., i, i] = c; R[..., j, j] = c
        R[..., i, j] = -s; R[..., j, i] = s
        return R
    # RevoluteUnaligned: Rodrigues (pinocchio: c I + s [a]x + (1-c) a a^T)
    a = np.asarray(axis, float)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    aa = np.outer(a, a)
    return (c[..., None, None] * np.eye(3) + s[..., None, None] * K + (1.0 - c)[..., None, None] * aa)


class FastArm:
    """numpy forward kinematics of one pinocchio model (serial revolute chain), plus umarell's sphere layout."""

    def __init__(self, model, step: float):
        self.model = model
        nj = model.njoints
        self.nq = model.nq
        self.parents = [int(model.parents[j]) for j in range(nj)]
        self.kind, self.axis, self.iq = [None], [None], [None]
        self.jR = [np.eye(3)]; self.jp = [np.zeros(3)]
        for j in range(1, nj):
            jm = model.joints[j]
            sn = jm.shortname()
            if sn in ("JointModelRX", "JointModelRY", "JointModelRZ"):
                kind, axis = sn[-2:], None
            elif sn == "JointModelRevoluteUnaligned":
                kind, axis = "RU", np.asarray(jm.axis, float).copy()
            else:
                raise NotImplementedError(f"joint type {sn}")
            if jm.nq != 1:
                raise NotImplementedError("multi-dof joint")
            self.kind.append(kind); self.axis.append(axis); self.iq.append(int(jm.idx_q))
            M = model.jointPlacements[j]
            self.jR.append(np.asarray(M.rotation, float).copy()); self.jp.append(np.asarray(M.translation, float).copy())
        self.nj = nj
        self.nf = len(model.frames)
        fpar, fR, fp = [], [], []
        for f in model.frames:
            par = getattr(f, "parentJoint", None)
            if par is None or callable(par):
                par = f.parent
            fpar.append(int(par))
            fR.append(np.asarray(f.placement.rotation, float).copy())
            fp.append(np.asarray(f.placement.translation, float).copy())
        self.fpar = np.array(fpar, int)
        self.fR = np.stack(fR); self.fp = np.stack(fp)
        # serial-chain check (link fill walks consecutive joint origins j -> j+1)
        self.serial = all(self.parents[j] == j - 1 for j in range(1, nj))
        # link-fill layout: |oMi[j+1].p - oMi[j].p| = |jointPlacement[j+1].p| for a serial chain
        self.step = float(step)
        ia, tt = [], []
        self.fill_ok = self.serial
        if step > 0:
            for j in range(1, nj - 1):
                L = float(np.linalg.norm(self.jp[j + 1]))
                n = int(L / step)
                if n >= 1 and abs(L / step - round(L / step)) < 1e-6:
                    self.fill_ok = False         # count unstable under rounding -> original
                for k in range(1, n + 1):
                    t = k * step
                    if t >= L:
                        break
                    ia.append(j); tt.append(t)
        self.fill_a = np.asarray(ia, int); self.fill_t = np.asarray(tt, float)
        self.nfill = len(tt)

    # -- joint placements oMi (robot base frame), batched over leading dims of q
    def joints(self, q):
        q = np.asarray(q, float)
        lead = q.shape[:-1]
        R = np.empty(lead + (self.nj, 3, 3)); p = np.empty(lead + (self.nj, 3))
        R[..., 0, :, :] = np.eye(3); p[..., 0, :] = 0.0
        for j in range(1, self.nj):
            qa = q[..., self.iq[j]]
            Rq = _rot_axis(self.kind[j], self.axis[j], np.cos(qa), np.sin(qa))
            Rl = self.jR[j] @ Rq
            par = self.parents[j]
            Rp = R[..., par, :, :]
            p[..., j, :] = np.einsum("...ij,j->...i", Rp, self.jp[j]) + p[..., par, :]
            R[..., j, :, :] = Rp @ Rl
        return R, p

    def frames(self, R, p):
        """(R_f, p_f) frame placements oMf from joint placements."""
        Rj = R[..., self.fpar, :, :]
        pf = np.einsum("...fij,fj->...fi", Rj, self.fp) + p[..., self.fpar, :]
        Rf = Rj @ self.fR
        return Rf, pf


def _to_world(P, base, yaw):
    """umarell's per-point base transform (same arithmetic as _frame_world / _link_fill_positions)."""
    cy, sy = np.cos(yaw), np.sin(yaw)
    out = np.empty_like(P)
    out[..., 0] = base[0] + P[..., 0] * cy - P[..., 1] * sy
    out[..., 1] = base[1] + P[..., 0] * sy + P[..., 1] * cy
    out[..., 2] = base[2] + P[..., 2]
    return out


_ARMS: dict = {}


def fast_arm(model):
    """Cached :class:`FastArm` for a pinocchio model, or ``None`` when the model is not supported."""
    from agents.trajectory.tools import robot_kinematics as rk
    step = float(rk._LINK_SPHERE_STEP_M)
    key = id(model)
    hit = _ARMS.get(key)
    if hit is not None and hit[0] is model and hit[1] == step:
        return hit[2]
    try:
        fa = FastArm(model, step)
        if not fa.fill_ok:
            fa = None
    except Exception:  # noqa: BLE001 - unsupported model: use the originals
        fa = None
    with _LOCK:
        if len(_ARMS) > 256:
            _ARMS.clear()
        _ARMS[key] = (model, step, fa)
    return fa


_TOOL: dict = {}


def _tool_layout(model, tool_cfg):
    """(offsets (n,3), mount frame index) exactly as ``_tool_spheres`` resolves them; (empty, -1) when none."""
    from agents.trajectory.tools import robot_kinematics as rk
    offs = rk._tool_offsets(tool_cfg)
    if not offs:
        return np.zeros((0, 3)), -1
    token = tool_cfg.get("mount_link", "")
    mkey = (id(model), token)
    idx = rk._TOOL_MOUNT_IDX.get(mkey)
    if idx is None:
        idx = next((i for i, f in enumerate(model.frames) if token and token in f.name), -1)
        rk._TOOL_MOUNT_IDX[mkey] = idx
    if idx < 0:
        return np.zeros((0, 3)), -1
    k = id(offs)
    hit = _TOOL.get(k)
    if hit is None or hit[0] is not offs:
        hit = (offs, np.asarray(offs, float).reshape(-1, 3))
        with _LOCK:
            if len(_TOOL) > 256:
                _TOOL.clear()
            _TOOL[k] = hit
    return hit[1], idx


def arm_spheres_pin(fa: FastArm, data, q, base, yaw, tool_cfg=None, with_tool=True):
    """Single-configuration (S, T) via pinocchio's C++ FK (cheaper than numpy FK for one q): the frame and joint
    origins are read from ``data`` exactly as umarell does, the fill / tool / base transforms are vectorised."""
    import pinocchio as pin
    model = fa.model
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    oMf, oMi = data.oMf, data.oMi
    P = np.array([oMf[i].translation for i in range(fa.nf)] + [oMi[j].translation for j in range(1, fa.nj)],
                 float)
    W = _to_world(P, base, yaw)
    Sf = W[:fa.nf]
    if fa.step > 0 and fa.nfill:
        J = W[fa.nf:]
        A = J[fa.fill_a - 1]
        D = J[fa.fill_a] - A
        L = np.sqrt(np.sum(D * D, axis=-1, keepdims=True))
        S = np.concatenate([Sf, A + (D / L) * fa.fill_t[:, None]], 0)
    else:
        S = Sf
    if not with_tool or not tool_cfg:
        return S, _EMPTY3
    O, idx = _tool_layout(model, tool_cfg)
    if idx < 0 or not O.size:
        return S, _EMPTY3
    M = oMf[idx]
    T = O @ np.asarray(M.rotation).T + np.asarray(M.translation)
    return S, _to_world(T, base, yaw)


_EMPTY3 = np.zeros((0, 3))


def arm_spheres(fa: FastArm, q, base, yaw, tool_cfg=None, with_tool=True):
    """(S, T): world sphere centres of ``_sphere_positions`` (frames + fill) and ``_tool_spheres``, batched."""
    R, p = fa.joints(q)
    Rf, pf = fa.frames(R, p)
    Sf = _to_world(pf, base, yaw)
    if fa.step > 0 and fa.nfill:
        J = _to_world(p[..., 1:, :], base, yaw)            # world joint origins, index j-1
        A = J[..., fa.fill_a - 1, :]
        D = J[..., fa.fill_a, :] - A
        L = np.sqrt(np.sum(D * D, axis=-1, keepdims=True))
        S = np.concatenate([Sf, A + (D / L) * fa.fill_t[:, None]], axis=-2)
    else:
        S = Sf
    if not with_tool or not tool_cfg:
        return S, np.zeros(S.shape[:-2] + (0, 3))
    O, idx = _tool_layout(fa.model, tool_cfg)
    if idx < 0 or not O.size:
        return S, np.zeros(S.shape[:-2] + (0, 3))
    Rm = Rf[..., idx, :, :]; tm = pf[..., idx, :]
    T = np.einsum("...ij,nj->...ni", Rm, O) + tm[..., None, :]
    return S, _to_world(T, base, yaw)


# ----------------------------------------------------------------------------- static obstacles
_STATIC: dict = {}


def _compile_static(static_obstacles: list, extra_margin: float):
    key = (id(static_obstacles), len(static_obstacles), float(extra_margin))
    hit = _STATIC.get(key)
    if hit is not None and hit[0] is static_obstacles and hit[1] == tuple(map(id, static_obstacles[:1] + static_obstacles[-1:])):
        return hit[2]
    sig = (tuple(map(id, static_obstacles)), float(extra_margin))
    hit2 = _STATIC.get(sig)
    if hit2 is not None:
        comp = hit2
    else:
        bc, bh, bm, sc, sr = [], [], [], [], []
        for obs in static_obstacles:
            if obs.get("is_workpiece", False):
                continue
            margin = float(obs.get("collision_margin", 0.05)) + extra_margin
            t = obs.get("type", "")
            if t == "box":
                bc.append(obs["center"]); bh.append(obs["half_extents"]); bm.append(margin)
            elif t == "sphere":
                sc.append(obs["center"]); sr.append(float(obs["radius"]) + margin)
        comp = {"bc": np.asarray(bc, float).reshape(-1, 3), "bh": np.asarray(bh, float).reshape(-1, 3),
                "bm": np.asarray(bm, float), "sc": np.asarray(sc, float).reshape(-1, 3),
                "srm": np.asarray(sr, float), "grid": {}}
    with _LOCK:
        if len(_STATIC) > 128:
            _STATIC.clear()
        _STATIC[sig] = comp
        _STATIC[key] = (static_obstacles, tuple(map(id, static_obstacles[:1] + static_obstacles[-1:])), comp)
    return comp


def _box_grid(comp, r_max: float):
    """Conservative voxel broad phase: a voxel is marked when ANY point in it could be within r_max + margin of a
    box. A sphere centre in an unmarked voxel (or outside the grid) cannot touch any box."""
    g = comp["grid"].get(round(r_max, 12))
    if g is not None:
        return g
    bc, bh, bm = comp["bc"], comp["bh"], comp["bm"]
    infl = r_max + bm.max() + GRID_H
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
    comp["grid"][round(r_max, 12)] = g
    return g


def _static_hit(comp, X, rad) -> bool:
    """Any centre X[i] (radius rad[i]) within clearance of a static box / sphere (strict <, as umarell)."""
    if not X.shape[0]:
        return False
    bc = comp["bc"]
    if bc.shape[0]:
        if bc.shape[0] >= GRID_MIN_BOXES:
            lo, shape, grid = _box_grid(comp, float(rad.max()))
            idx = np.floor((X - lo) / GRID_H).astype(int)
            inside = np.all((idx >= 0) & (idx < shape), 1)
            cand = np.zeros(X.shape[0], bool)
            ii = idx[inside]
            cand[inside] = grid[ii[:, 0], ii[:, 1], ii[:, 2]]
            Xb, rb = X[cand], rad[cand]
        else:
            Xb, rb = X, rad
        if Xb.shape[0]:
            D = Xb[:, None, :] - bc[None, :, :]
            Cl = np.clip(D, -comp["bh"][None], comp["bh"][None])
            d = np.sqrt(((D - Cl) ** 2).sum(-1))
            if np.any(d < rb[:, None] + comp["bm"][None, :]):
                return True
    sc = comp["sc"]
    if sc.shape[0]:
        d = np.sqrt(((X[:, None, :] - sc[None, :, :]) ** 2).sum(-1))
        if np.any(d < rad[:, None] + comp["srm"][None, :]):
            return True
    return False


_MOV: dict = {}


def _moving_arrays(obstacle_spheres: list):
    """(P, R, c, P0, |P0|^2) for a moving-obstacle list, cached by list identity (the planner passes one list per
    segment, thousands of checks)."""
    k = id(obstacle_spheres)
    hit = _MOV.get(k)
    if hit is not None and hit[0] is obstacle_spheres and hit[1] == len(obstacle_spheres):
        return hit[2]
    P = np.asarray([p for p, _ in obstacle_spheres], float).reshape(-1, 3)
    R = np.asarray([r for _, r in obstacle_spheres], float)
    c = P.mean(0) if P.shape[0] else np.zeros(3)
    P0 = P - c
    arr = (P, R, c, P0, (P0 * P0).sum(1), {})
    with _LOCK:
        if len(_MOV) > 256:
            _MOV.clear()
        _MOV[k] = (obstacle_spheres, len(obstacle_spheres), arr)
    return arr


GRAM_GUARD = 1e-9   # m^2; the Gram-expansion error is ~1e-13 m^2 for cell-sized coordinates


def _moving_hit(A, obstacle_spheres, sphere_radius: float) -> bool:
    """Any |A_i - P_j| < R_j + sphere_radius (umarell's strict test). Located with one BLAS Gram product; pairs within
    GRAM_GUARD of the threshold are re-tested exactly, so the boolean equals the original's."""
    P, R, c, P0, P2, lim_cache = _moving_arrays(obstacle_spheres)
    lim = lim_cache.get(sphere_radius)
    if lim is None:
        lim = R + sphere_radius
        lim_cache[sphere_radius] = lim = (lim, lim * lim)
    lim1, lim2 = lim
    A0 = A - c
    m = (A0 * A0).sum(1)[:, None] + P2[None, :] - 2.0 * (A0 @ P0.T) - lim2[None, :]
    mn = float(m.min()) if m.size else np.inf
    if mn < -GRAM_GUARD:
        return True
    if mn >= GRAM_GUARD:
        return False
    ii, jj = np.nonzero(m < GRAM_GUARD)
    D = A[ii] - P[jj]
    return bool(np.any(np.sqrt((D * D).sum(1)) < lim1[jj]))


def _variant() -> str:
    from agents.trajectory.tools import robot_kinematics as rk
    return "fix03" if hasattr(rk, "_floor_breach") else "orig"


_FIXED: dict = {}


def _fixed_frames(model) -> int:
    from agents.trajectory.tools import robot_kinematics as rk
    return int(rk._fixed_frame_count(model))


# ----------------------------------------------------------------------------- drop-ins
_ORIG: dict = {}


def config_is_free(model, col_data, q, base, yaw, obstacle_spheres, static_obstacles, sphere_radius: float = 0.12,
                   tool_cfg=None, sphere_radii=None, extra_margin: float = 0.0, _tool_vs_moving: bool = True) -> bool:
    """Drop-in for ``robot_kinematics._config_is_free`` (same predicate; see module doc for the variants)."""
    fa = fast_arm(model)
    if fa is None:
        STATS["fallback_free"] += 1
        return _ORIG["_config_is_free"](model, col_data, q, base, yaw, obstacle_spheres, static_obstacles,
                                        sphere_radius=sphere_radius, tool_cfg=tool_cfg, sphere_radii=sphere_radii,
                                        extra_margin=extra_margin)
    STATS["fast_free"] += 1
    variant = _variant()
    q = np.asarray(q, float)[: fa.nq]
    need_tool = bool(tool_cfg) and (variant == "fix03" or (_tool_vs_moving and obstacle_spheres))
    S, T = arm_spheres_pin(fa, col_data, q, base, yaw, tool_cfg, with_tool=need_tool)
    n_r = 0 if sphere_radii is None else len(sphere_radii)
    rad = np.full(S.shape[0], float(sphere_radius))
    if n_r:
        k = min(n_r, S.shape[0])
        rad[:k] = np.asarray(sphere_radii, float)[:k]
    if variant == "orig":
        if S.shape[0] > 3 and np.any(S[3:, 2] < -0.05):
            return False
        tool_r = None
    else:
        from agents.trajectory.tools import robot_kinematics as rk
        f0 = _fixed_frames(model)
        tool_r = float(rk._tool_static_radius(sphere_radius, sphere_radii))
        if np.any(S[f0:, 2] - rad[f0:] < rk.FLOOR_Z_M):
            return False
        if T.shape[0] and np.any(T[:, 2] - tool_r < rk.FLOOR_Z_M):
            return False
    if obstacle_spheres:
        A = np.concatenate([S, T], 0) if (_tool_vs_moving and T.shape[0]) else S
        if _moving_hit(A, obstacle_spheres, float(sphere_radius)):
            return False
    if static_obstacles:
        comp = _compile_static(static_obstacles, extra_margin)
        X, rx = S[2:], rad[2:]
        if tool_r is not None and T.shape[0]:
            X = np.concatenate([X, T], 0)
            rx = np.concatenate([rx, np.full(T.shape[0], tool_r)])
        if _static_hit(comp, X, rx):
            return False
    return True


def make_collision_info(orig):
    """``_config_collision_info`` with a vectorised fast path: a configuration that passes the same predicate
    (without the tool-vs-other-arm term, which the diagnosis does not apply) returns ``None``; a colliding one is
    diagnosed by the original, so the returned dict is identical."""
    def _config_collision_info(model, col_data, q, base, yaw, obstacle_spheres, static_obstacles,
                               sphere_radius: float = 0.12, sphere_radii=None, extra_margin: float = 0.0, **kw):
        tool_cfg = kw.get("tool_cfg")
        if fast_arm(model) is not None and config_is_free(
                model, col_data, q, base, yaw, obstacle_spheres, static_obstacles, sphere_radius=sphere_radius,
                tool_cfg=tool_cfg, sphere_radii=sphere_radii, extra_margin=extra_margin, _tool_vs_moving=False):
            return None
        return orig(model, col_data, q, base, yaw, obstacle_spheres, static_obstacles, sphere_radius=sphere_radius,
                    sphere_radii=sphere_radii, extra_margin=extra_margin, **kw)
    _config_collision_info._uf_fast = True
    _config_collision_info.__wrapped__ = orig
    return _config_collision_info


def config_spheres(model, data_r, q, base, yaw, tool_cfg=None):
    """Drop-in for ``_config_spheres``: list of the frame, link-fill and tool sphere centres (same order)."""
    fa = fast_arm(model)
    if fa is None:
        return _ORIG["_config_spheres"](model, data_r, q, base, yaw, tool_cfg)
    S, T = arm_spheres_pin(fa, data_r, np.asarray(q, float), base, yaw, tool_cfg, with_tool=True)
    return list(np.concatenate([S, T], 0)) if T.shape[0] else list(S)


def check_robot_pair_collision(s1, s2, radius: float = 0.12):
    """Drop-in for ``_check_robot_pair_collision``: (collision, min over pairs of |a-b| - 2 radius)."""
    A = np.asarray(s1, float).reshape(-1, 3)
    B = np.asarray(s2, float).reshape(-1, 3)
    if not A.size or not B.size:
        return False, float("inf")
    d = np.sqrt(((A[:, None, :] - B[None, :, :]) ** 2).sum(-1)) - 2 * radius
    m = float(d.min())
    return m < 0, m


def _sample_all(traj, grid):
    """Vectorised ``time_parameterization._sample_at_time`` over a time grid (None when it cannot be exact)."""
    times = np.asarray([float(p.get("time_s", 0.0)) for p in traj], float)
    if times.size == 0 or np.any(np.diff(times) < 0):
        return None
    try:
        Q = np.asarray([p["positions"] for p in traj], float)
    except Exception:  # noqa: BLE001 - ragged
        return None
    g = np.asarray(grid, float)
    out = np.empty((g.size, Q.shape[1]))
    first = g <= times[0]
    last = (~first) & (g >= times[-1])
    mid = ~(first | last)
    out[first] = Q[0]
    out[last] = Q[-1]
    if np.any(mid):
        gm = g[mid]
        hi = np.searchsorted(times, gm, side="left")   # first index with times >= t
        lo = np.maximum(hi - 1, 0)
        span = times[hi] - times[lo]
        a, b = Q[lo], Q[hi]
        with np.errstate(divide="ignore", invalid="ignore"):
            f = (gm - times[lo]) / span
        res = a + f[:, None] * (b - a)
        res[span <= 0] = b[span <= 0]
        out[mid] = res
    return out


def inter_robot_clearance(cell_id, trajectories, collision_radius_m: float = 0.12, n_samples: int = 20,
                          warning_distance_m=None):
    """Drop-in for ``robot_kinematics.inter_robot_clearance`` (identical result dict), batched over the clock."""
    from agents.trajectory.tools import robot_kinematics as rk
    names = [ns for ns, tr in (trajectories or {}).items() if tr]
    if len(names) < 2:
        return _ORIG["inter_robot_clearance"](cell_id, trajectories, collision_radius_m=collision_radius_m,
                                              n_samples=n_samples, warning_distance_m=warning_distance_m)
    models, bases, yaws, tools, fas = {}, {}, {}, {}, {}
    for ns in names:
        models[ns], _ = rk._load_pinocchio_model(cell_id, ns)
        cfg = rk._load_robot_config(cell_id, ns)
        bases[ns] = np.array(cfg["base_position"])
        yaws[ns] = float(cfg.get("base_orientation_yaw", 0.0))
        tools[ns] = cfg.get("tool_collision")
        fas[ns] = fast_arm(models[ns])
    warning_distance = (float(warning_distance_m) if warning_distance_m is not None
                        else rk._load_collision_config(cell_id)["warning_distance"])
    horizon = max(float(trajectories[ns][-1].get("time_s", 0.0)) for ns in names)
    n = rk.validation_sample_count(horizon, floor=int(n_samples))
    grid = [horizon * i / (n - 1) for i in range(n)] if horizon > 0 else [0.0]
    Qs = {}
    for ns in names:
        Q = _sample_all(trajectories[ns], grid) if fas[ns] is not None else None
        if Q is None or Q.shape[1] != fas[ns].nq:
            STATS["irc_fallback"] += 1
            return _ORIG["inter_robot_clearance"](cell_id, trajectories, collision_radius_m=collision_radius_m,
                                                  n_samples=n_samples, warning_distance_m=warning_distance_m)
        Qs[ns] = Q
    STATS["irc_fast"] += 1
    pairs = [(i, j) for i in range(len(names)) for j in range(i + 1, len(names))]
    G = len(grid)
    clr = np.empty((G, len(pairs)))
    for c0 in range(0, G, IRC_CHUNK):
        c1 = min(G, c0 + IRC_CHUNK)
        sph = {}
        for ns in names:
            S, T = arm_spheres(fas[ns], Qs[ns][c0:c1], bases[ns], yaws[ns], tools[ns], with_tool=True)
            sph[ns] = np.concatenate([S, T], axis=-2)
        for k, (i, j) in enumerate(pairs):
            A, B = sph[names[i]], sph[names[j]]
            d = np.sqrt(((A[:, :, None, :] - B[:, None, :, :]) ** 2).sum(-1))
            clr[c0:c1, k] = d.reshape(c1 - c0, -1).min(1) - 2 * collision_radius_m
    flat = int(np.argmin(clr.reshape(-1)))
    ti, pk = divmod(flat, len(pairs))
    min_clearance = float(clr[ti, pk])
    worst_pair = [names[pairs[pk][0]], names[pairs[pk][1]]]
    bad = (clr < 0) | (clr <= warning_distance)
    collision_t = None
    entries = []   # (grid index, pair index, 0 sphere / 1 hull, key, entry): the original's insertion order
    if bad.any():
        for k, (i, j) in enumerate(pairs):
            idx = np.nonzero(bad[:, k])[0]
            if idx.size:
                t = grid[int(idx[0])]
                a, b = names[i], names[j]
                entries.append((int(idx[0]), k, 0, f"{a}|{b}",
                                {"robots": [a, b], "time_s": round(t, 4),
                                 "clearance_m": round(float(clr[idx[0], k]), 4),
                                 "penetrating": bool(clr[idx[0], k] < 0)}))
    # Upstream fix 04 / 04v2: a cell may declare a hull backstop (collision.hull_min_distance). The hull distance is
    # the original's own (hull_clearance.arm_hull_distance at the original's samples), so the result stays identical.
    hull_min = rk._load_collision_config(cell_id).get("hull_min_distance")
    min_hull, worst_hull_pair = float("inf"), None
    if hull_min is not None:
        from agents.trajectory.tools.hull_clearance import arm_hull_distance
        from agents.trajectory.tools.time_parameterization import _sample_at_time
        seen = set()
        for gi, t in enumerate(grid):
            qs = {ns: _sample_at_time(trajectories[ns], t) for ns in names}
            for k, (i, j) in enumerate(pairs):
                a, b = names[i], names[j]
                hd, hp = arm_hull_distance(cell_id, a, qs[a], b, qs[b])
                if hd < min_hull:
                    min_hull, worst_hull_pair = hd, [f"{a}:{hp[0]}", f"{b}:{hp[1]}"]
                if hd < hull_min and k not in seen:
                    seen.add(k)
                    entries.append((gi, k, 1, f"{a}|{b}|hull",
                                    {"robots": [a, b], "time_s": round(t, 4), "hull_distance_m": round(hd, 4),
                                     "hulls": hp, "penetrating": bool(hd < 0)}))
    colliding: dict = {}
    if entries:
        entries.sort(key=lambda e: (e[0], e[1], e[2]))
        colliding = {e[3]: e[4] for e in entries}
        collision_t = round(grid[entries[0][0]], 4)
    hull_ok = hull_min is None or min_hull >= hull_min
    result = {
        "collision_free": min_clearance > warning_distance and hull_ok,
        "min_clearance_m": round(min_clearance, 4),
        "warning_distance_m": warning_distance,
        "robots": names,
        "pairs_checked": len(names) * (len(names) - 1) // 2,
        "worst_pair": worst_pair,
        "samples_checked": len(grid),
        "monitor_rate_hz": rk.MONITOR_RATE_HZ,
    }
    if hull_min is not None:
        result["hull_min_distance_m"] = hull_min
        result["min_hull_distance_m"] = round(min_hull, 4)
        result["worst_hull_pair"] = worst_hull_pair
    if colliding:
        result["collision_time_s"] = collision_t
        result["colliding_pairs"] = list(colliding.values())
    return result


# ----------------------------------------------------------------------------- install
_NAMES = ("_config_is_free", "_config_collision_info", "_config_spheres", "_check_robot_pair_collision",
          "inter_robot_clearance")


def install() -> dict:
    """Replace the five kernels in robot_kinematics AND in every already-imported ``agents.*`` module that bound
    them by name (``from robot_kinematics import ...``). Import the umarell tool modules first. Idempotent."""
    from agents.trajectory.tools import robot_kinematics as rk
    if getattr(rk, "_uf_u1_kernels", False):
        return dict(_ORIG)
    if hasattr(rk, "_self_free") or hasattr(rk, "gate_groups"):
        # umarell 76ea13a: its predicate also checks the arm against itself, the link envelope, the tool hulls and
        # the workpiece rule, and the pair check takes groups / radii / stopping distances; this drop-in re-creates
        # eccb1bc's predicate and would silently drop all of that (umarell_fast/COORDINATION.md, 2026-10-23)
        raise RuntimeError("umarell_fast.u1.kernels: drop-in kernels target umarell eccb1bc; not installable on 76ea13a")
    for nm in _NAMES:
        _ORIG[nm] = getattr(rk, nm)
    new = {"_config_is_free": config_is_free,
           "_config_collision_info": make_collision_info(_ORIG["_config_collision_info"]),
           "_config_spheres": config_spheres,
           "_check_robot_pair_collision": check_robot_pair_collision,
           "inter_robot_clearance": inter_robot_clearance}
    by_id = {id(_ORIG[nm]): new[nm] for nm in _NAMES}
    for mname, mod in list(sys.modules.items()):
        if not (mname.startswith("agents.") or mname.startswith("utils.")) or mod is None:
            continue
        for attr, val in list(vars(mod).items()):
            rep = by_id.get(id(val))
            if rep is not None:
                setattr(mod, attr, rep)
    rk._uf_u1_kernels = True
    return dict(_ORIG)


def uninstall() -> None:
    from agents.trajectory.tools import robot_kinematics as rk
    if not getattr(rk, "_uf_u1_kernels", False):
        return
    cur = {nm: getattr(rk, nm) for nm in _NAMES}
    back = {id(cur[nm]): _ORIG[nm] for nm in _NAMES}
    for mname, mod in list(sys.modules.items()):
        if not (mname.startswith("agents.") or mname.startswith("utils.")) or mod is None:
            continue
        for attr, val in list(vars(mod).items()):
            rep = back.get(id(val))
            if rep is not None:
                setattr(mod, attr, rep)
    rk._uf_u1_kernels = False
