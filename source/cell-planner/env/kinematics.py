"""Vectorised numpy kinematics + umarell's collision-sphere model (Track K).

A numerically identical (to ~1e-12) reimplementation of
``third_party/sample-umarell/agents/trajectory/tools/robot_kinematics.py``:

* ``_config_spheres``        -> :meth:`SphereModel.spheres` (77 per arm, same order)
* ``_check_robot_pair_collision(s1, s2, 0.12)[1]`` -> :meth:`SphereModel.pair_clearance`
* ``inter_robot_clearance``  -> :meth:`SphereModel.inter_robot_clearance`
* ``_config_is_free`` / ``_config_collision_info`` (static part: floor + boxes/spheres
  from obstacles.yaml) -> :meth:`SphereModel.obstacle_clearance`, ``obstacle_free``,
  ``obstacle_collision_info``

Everything is batched over arbitrary leading dimensions of ``q`` (``(..., 6)``) and needs
only numpy (no pinocchio). It is cross-checked against umarell itself in
``tests/test_kinematics.py`` with reference data from ``scripts/make_umarell_reference.py``.

Sphere order per arm (umarell's ORDERING CONTRACT, 77 = 17 + 25 + 35):

* ``0..16``  pinocchio frame origins in frame-index order: universe, root_joint, World,
  base_link, then (joint_aK, link_K) for K = 1..6, then ``welding_tcp`` (added at load
  time by ``utils.arm_model.add_tcp_frame``).
* ``17..41`` link-fill spheres every 0.12 m between consecutive joint origins
  (J1-J2: 3, J2-J3: 12, J3-J4: 10; J4/J5/J6 coincide).
* ``42..76`` tool spheres from ``tool_collision`` (axis 5 + plates 12 + 9 + 9) in the
  ``link_6`` frame.

umarell rules replicated (see ``docs`` in the module functions):

* arm-vs-arm: all 77 x 77 spheres, uniform radius 0.12 (``collision_radius_m``),
  clearance = min centre distance - 2 r; collision-free iff clearance > warning_distance
  (0.20).
* floor: sphere CENTRES of indices 3..41 (frames from base_link on + fill, NOT the tool
  spheres) must have z >= -0.05. No radius is involved. The ``plane`` obstacle in
  obstacles.yaml is ignored by umarell's predicate (only ``box`` and ``sphere`` are read).
* boxes / spheres: indices 2..41 (tool spheres skipped); radius = ``robot_link_radius``
  (0.12) except the frames after the deepest ``mount_link`` frame (only ``welding_tcp``,
  index 16) which take ``tool_link_radius`` (0.05); per-obstacle
  ``collision_margin`` (fallback 0.05, not ``default_collision_margin``) + extra_margin;
  ``is_workpiece`` obstacles skipped. Collision iff distance(centre, box) < r + margin
  (strict), so a signed clearance ``< 0`` is a collision and exactly 0 is free.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

import numpy as np

from env.cell import ROBOT_IDS, Cell, RobotCfg, load_cell

# umarell constants (robot_kinematics.py)
LINK_SPHERE_STEP_M = 0.12          # _LINK_SPHERE_STEP_M (env LINK_SPHERE_STEP_M default)
MONITOR_RATE_HZ = 25.0             # MONITOR_RATE_HZ
FLOOR_Z_MIN = -0.05                # hard-coded floor threshold in _config_is_free
DEFAULT_OBST_MARGIN = 0.05         # obs.get("collision_margin", 0.05)
PAIR_RADIUS = 0.12                 # inter_robot_clearance(collision_radius_m=0.12)


# --------------------------------------------------------------------------- helpers
def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    """Rz(y) @ Ry(p) @ Rx(r) (URDF / pinocchio / umarell ``_rpy_matrix`` convention)."""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return Rz @ Ry @ Rx


def _floats(s: str | None, n: int = 3) -> np.ndarray:
    return np.zeros(n) if s is None else np.array([float(v) for v in s.split()], float)


@lru_cache(maxsize=None)
def _parse_urdf(path: str):
    """Serial chain from the URDF root to the last movable joint's child.

    Returns (fixed_prefix, joints) where joints is a list of dicts
    {name, child, xyz, rpy, axis, lower, upper, velocity}. Fixed joints before the first
    revolute joint are composed into ``fixed_prefix`` (4x4). The URDF is read by
    ``robotmodel.urdf_robot.parse_urdf`` (the project's single URDF parser, docs/p3_urdf.md)."""
    from robotmodel.urdf_robot import default_tip_link, parse_urdf
    u = parse_urdf(path)
    joints = {j.child: dict(name=j.name, type=j.type, parent=j.parent, child=j.child, xyz=np.array(j.xyz, float),
                            rpy=np.array(j.rpy, float), axis=np.array(j.axis, float), lower=j.lower, upper=j.upper,
                            velocity=j.velocity) for j in u.joints}
    tip = default_tip_link(u)
    chain = []
    link = tip
    while link in joints:
        chain.append(joints[link]); link = joints[link]["parent"]
    chain.reverse()
    prefix = np.eye(4)
    rev = []
    for j in chain:
        T = np.eye(4); T[:3, :3] = rpy_matrix(*j["rpy"]); T[:3, 3] = j["xyz"]
        if j["type"] == "fixed":
            if rev:
                raise NotImplementedError("fixed joint inside the arm chain")
            prefix = prefix @ T
        else:
            rev.append(j)
    rev[0]["base_link"] = rev[0]["parent"]
    return prefix, rev


def _axis_rot(axis_id: int, c: np.ndarray, s: np.ndarray) -> np.ndarray:
    """(...,3,3) rotation about the unit coordinate axis ``axis_id`` (0=x,1=y,2=z)."""
    R = np.zeros(c.shape + (3, 3))
    i, j = [(1, 2), (2, 0), (0, 1)][axis_id]
    R[..., axis_id, axis_id] = 1.0
    R[..., i, i] = c; R[..., j, j] = c
    R[..., i, j] = -s; R[..., j, i] = s
    return R


# --------------------------------------------------------------------------- kinematics
class ArmKinematics:
    """Forward kinematics / Jacobian of one KR240 arm, world frame (base + yaw applied).

    Frame list (``frame_names``) mirrors umarell's pinocchio model frame order (17 frames,
    last = ``welding_tcp``). ``q`` may have any leading batch shape ``(..., 6)``.
    """

    def __init__(self, robot: RobotCfg):
        self.robot = robot
        self.rid = robot.rid
        prefix, joints = _parse_urdf(str(robot.urdf))
        if len(joints) != 6:
            raise ValueError(f"expected 6 revolute joints, got {len(joints)}")
        if not np.allclose(prefix, np.eye(4)):
            raise NotImplementedError("non-identity fixed prefix before joint_a1")
        self.joint_names = [j["name"] for j in joints]
        self.link_names = [j["child"] for j in joints]
        self._Rp = np.stack([rpy_matrix(*j["rpy"]) for j in joints])        # (6,3,3)
        self._pp = np.stack([j["xyz"] for j in joints])                      # (6,3)
        self._axis = []
        for j in joints:
            a = j["axis"] / np.linalg.norm(j["axis"])
            k = int(np.argmax(np.abs(a)))
            if not np.isclose(abs(a[k]), 1.0):
                raise NotImplementedError("only coordinate-axis revolute joints supported")
            self._axis.append((k, float(np.sign(a[k]))))
        self.base = np.asarray(robot.base, float)
        self.yaw = float(robot.yaw)
        self._cy, self._sy = math.cos(self.yaw), math.sin(self.yaw)
        self.Rbase = np.array([[self._cy, -self._sy, 0.0], [self._sy, self._cy, 0.0], [0, 0, 1.0]])
        self.tcp_offset = np.asarray(robot.tcp_offset, float)
        rot = getattr(robot, "tcp_rot", None)       # the TCP frame's axes in link_6 (a rotated tip_link), else link_6's
        self.tcp_rot = np.eye(3) if rot is None else np.asarray(rot, float)
        self.q_lo, self.q_hi = np.asarray(robot.q_lo), np.asarray(robot.q_hi)
        # pinocchio frame order for this URDF: universe, root_joint, World, base_link, then
        # (joint, child link) pairs, then the TCP OP_FRAME. All placements are identity
        # w.r.t. their parent joint except the TCP. parent joint index: 0 = universe.
        self.frame_names = ["universe", "root_joint", "World", joints[0].get("base_link", "like_kr240r2900_base_link")]
        self.frame_parent = [0, 0, 0, 0]
        for k, (jn, ln) in enumerate(zip(self.joint_names, self.link_names), start=1):
            self.frame_names += [jn, ln]; self.frame_parent += [k, k]
        self.frame_names.append("welding_tcp"); self.frame_parent.append(6)
        self.frame_parent = np.array(self.frame_parent)
        self.n_frames = len(self.frame_names)
        self.tcp_frame = self.n_frames - 1

    # -- local (robot base frame, i.e. pinocchio's oMi) --------------------------------
    def joint_poses_local(self, q):
        """(R (...,7,3,3), p (...,7,3)) joint placements oMi in the robot base frame,
        index 0 = universe (identity), 1..6 = joint_a1..a6 (pinocchio arithmetic order)."""
        q = np.asarray(q, float)
        lead = q.shape[:-1]
        Rs = np.empty(lead + (7, 3, 3)); ps = np.empty(lead + (7, 3))
        Rs[..., 0, :, :] = np.eye(3); ps[..., 0, :] = 0.0
        c, s = np.cos(q), np.sin(q)
        R, p = Rs[..., 0, :, :], ps[..., 0, :]
        for i, (ax, sg) in enumerate(self._axis):
            Rli = self._Rp[i] @ _axis_rot(ax, c[..., i], sg * s[..., i])     # liMi rotation
            p = R @ self._pp[i] + p
            R = R @ Rli
            Rs[..., i + 1, :, :] = R; ps[..., i + 1, :] = p
        return Rs, ps

    def _to_world(self, p):
        """umarell's per-point base transform: base + Rz(yaw) p (same arithmetic)."""
        cy, sy, b = self._cy, self._sy, self.base
        out = np.empty_like(p)
        out[..., 0] = b[0] + p[..., 0] * cy - p[..., 1] * sy
        out[..., 1] = b[1] + p[..., 0] * sy + p[..., 1] * cy
        out[..., 2] = b[2] + p[..., 2]
        return out

    # -- world --------------------------------------------------------------------------
    def frames(self, q) -> np.ndarray:
        """(..., 17, 4, 4) world transforms of the pinocchio frames (base + yaw applied)."""
        Rs, ps = self.joint_poses_local(q)
        Rf = Rs[..., self.frame_parent, :, :].copy()
        pf = ps[..., self.frame_parent, :].copy()
        pf[..., -1, :] = pf[..., -1, :] + Rf[..., -1, :, :] @ self.tcp_offset
        Rf[..., -1, :, :] = Rf[..., -1, :, :] @ self.tcp_rot
        T = np.zeros(pf.shape[:-1] + (4, 4))
        T[..., :3, :3] = self.Rbase @ Rf
        T[..., :3, 3] = self._to_world(pf)
        T[..., 3, 3] = 1.0
        return T

    def tcp(self, q):
        """TCP world pose: (pos (...,3), rot (...,3,3)). Tool axis = -rot[..., :, 0]."""
        Rs, ps = self.joint_poses_local(q)
        R6, p6 = Rs[..., 6, :, :], ps[..., 6, :]
        p = p6 + R6 @ self.tcp_offset
        return self._to_world(p), self.Rbase @ R6 @ self.tcp_rot

    def jacobian(self, q) -> np.ndarray:
        """(..., 6, 6) world-frame geometric Jacobian at the TCP point, rows
        [linear (3); angular (3)] (pinocchio LOCAL_WORLD_ALIGNED convention)."""
        Rs, ps = self.joint_poses_local(q)
        p_tcp = ps[..., 6, :] + Rs[..., 6, :, :] @ self.tcp_offset
        J = np.empty(np.shape(q)[:-1] + (6, 6))
        for i, (ax, sg) in enumerate(self._axis):
            z = sg * Rs[..., i + 1, :, ax]                         # joint axis in base frame
            r = p_tcp - ps[..., i + 1, :]
            J[..., :3, i] = np.cross(z, r)
            J[..., 3:, i] = z
        J[..., :3, :] = self.Rbase @ J[..., :3, :]
        J[..., 3:, :] = self.Rbase @ J[..., 3:, :]
        return J

    KEYPOINT_NAMES = ("j1", "j2", "j3", "wrist", "tool_mid", "tcp")

    def keypoints(self, q) -> np.ndarray:
        """(..., 6, 3) world keypoints for graph observations: joint origins a1, a2, a3,
        wrist centre (a4 = a5 = a6 = flange), tool midpoint (half the TCP offset), TCP."""
        Rs, ps = self.joint_poses_local(q)
        R6 = Rs[..., 6, :, :]
        pts = np.stack([ps[..., 1, :], ps[..., 2, :], ps[..., 3, :], ps[..., 5, :],
                        ps[..., 6, :] + R6 @ (0.5 * self.tcp_offset),
                        ps[..., 6, :] + R6 @ self.tcp_offset], axis=-2)
        return self._to_world(pts)


# --------------------------------------------------------------------------- tool model
def tool_offsets(tool_cfg: dict | None) -> np.ndarray:
    """(n,3) tool sphere centres in the mount-link frame; exact copy of umarell
    ``_tool_offsets`` (including the accumulating ``x += step`` loop)."""
    if not tool_cfg:
        return np.zeros((0, 3))
    offs = []
    ax = tool_cfg.get("axis")
    if ax:
        d = np.array(ax.get("direction", [-1.0, 0.0, 0.0]), dtype=float)
        n = float(np.linalg.norm(d)) or 1.0
        d = d / n
        s = float(ax.get("start_m", 0.15)); e = float(ax.get("end_m", 0.9))
        step = float(ax.get("step_m", 0.15))
        x = s
        while x <= e + 1e-9:
            offs.append(d * x)
            x += step
    for plate in tool_cfg.get("plates", []) or []:
        c = np.array(plate.get("center", [0.0, 0.0, 0.0]), dtype=float)
        rpy = plate.get("rpy", [0.0, 0.0, 0.0])
        Rp = rpy_matrix(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        w, h = plate.get("size", [0.3, 0.45])
        gu, gv = plate.get("grid", [5, 5])
        for u in np.linspace(-float(w) / 2.0, float(w) / 2.0, int(gu)):
            for v in np.linspace(-float(h) / 2.0, float(h) / 2.0, int(gv)):
                offs.append(c + Rp @ np.array([u, v, 0.0]))
    return np.array(offs, float).reshape(-1, 3)


# --------------------------------------------------------------------------- spheres
class _ArmSpheres:
    def __init__(self, kin: ArmKinematics, cell: Cell, step: float = LINK_SPHERE_STEP_M):
        self.kin = kin
        tool_cfg = kin.robot.tool_collision or None
        self.tool_off = tool_offsets(tool_cfg)
        token = (tool_cfg or {}).get("mount_link", "")
        names = kin.frame_names
        # _tool_spheres: FIRST frame whose name contains the token (frame index)
        self.mount_frame = next((i for i, f in enumerate(names) if token and token in f), -1)
        if self.tool_off.size and self.mount_frame < 0:
            self.tool_off = np.zeros((0, 3))
        # _tool_frame_start: one past the DEEPEST matching frame
        deepest = max((i for i, f in enumerate(names) if token and token in f), default=-1)
        self.tool_frame_start = len(names) if deepest < 0 else deepest + 1
        # link-fill sphere counts: joint segments 1..5 (j, j+1) are rigid, so the counts
        # are configuration-independent. Replicates n = int(seg/step); t = k*step < seg.
        _, ps = kin.joint_poses_local(np.zeros(6))
        self.fill_seg, self.fill_t = [], []
        for j in range(1, 6):
            seg = float(np.linalg.norm(ps[j + 1] - ps[j]))
            n = int(seg / step) if step > 0 else 0
            for k in range(1, n + 1):
                t = k * step
                if t >= seg:
                    break
                self.fill_seg.append(j); self.fill_t.append(t)
            if n >= 1 and abs(seg / step - round(seg / step)) < 1e-6:
                raise RuntimeError("link length is a multiple of the fill step; count unstable")
        self.fill_seg = np.array(self.fill_seg, int)
        self.fill_t = np.array(self.fill_t, float)
        self.n_frame = kin.n_frames
        self.n_fill = len(self.fill_t)
        self.n_tool = len(self.tool_off)
        self.n = self.n_frame + self.n_fill + self.n_tool
        self.sl_frame = slice(0, self.n_frame)
        self.sl_fill = slice(self.n_frame, self.n_frame + self.n_fill)
        self.sl_tool = slice(self.n_frame + self.n_fill, self.n)
        pol = cell.clearance
        self.arm_radius = float(pol.get("robot_link_radius", 0.12))
        tr = pol.get("tool_link_radius")
        self.tool_radius = self.arm_radius if tr is None else float(tr)
        # Static-check radii (_cell_sphere_radii + fallback sphere_radius for fill spheres)
        r = np.full(self.n, self.arm_radius)
        if abs(self.tool_radius - self.arm_radius) >= 1e-12:
            r[self.tool_frame_start:self.n_frame] = self.tool_radius
        r[self.sl_tool] = np.nan                       # tool spheres are not checked
        self.obst_radii = r
        idx = np.arange(self.n)
        self.floor_idx = idx[3:self.n_frame + self.n_fill]   # spheres[3:] of _sphere_positions
        self.box_idx = idx[2:self.n_frame + self.n_fill]     # range(2, len(spheres))
        # exact duplicate centres (coincident frame origins, e.g. joint/body frame pairs and
        # the a4/a5/a6 wrist) are dropped from the min-distance searches; the first index of
        # each duplicate group is kept, so reported indices are umarell's first occurrence.
        qs = np.random.default_rng(0).uniform(kin.q_lo, kin.q_hi, (4, 6))
        S = self.spheres(qs)
        keep = [i for i in range(self.n) if not any(np.array_equal(S[:, i], S[:, k]) for k in range(i))]
        self.uniq = np.array(keep, int)
        ks = set(keep)
        self.floor_idx_u = np.array([i for i in self.floor_idx if i in ks], int)
        self.box_idx_u = np.array([i for i in self.box_idx if i in ks], int)

    def spheres(self, q) -> np.ndarray:
        kin = self.kin
        Rs, ps = kin.joint_poses_local(q)
        # frame-origin block (local), as data.oMf[i].translation
        pf = ps[..., kin.frame_parent, :].copy()
        R6 = Rs[..., 6, :, :]
        pf[..., -1, :] = ps[..., 6, :] + R6 @ kin.tcp_offset
        world_f = kin._to_world(pf)
        # fill: between world-frame joint origins (umarell transforms a, b first)
        wj = kin._to_world(ps)
        a = wj[..., self.fill_seg, :]
        b = wj[..., self.fill_seg + 1, :]
        diff = b - a
        seg = np.sqrt(np.sum(diff * diff, axis=-1, keepdims=True))
        fill = a + (diff / seg) * self.fill_t[:, None]
        # tool: R @ off + t on the mount frame, then base transform
        Rm = Rs[..., kin.frame_parent[self.mount_frame], :, :]
        tm = ps[..., kin.frame_parent[self.mount_frame], :]
        tool = kin._to_world(np.einsum("...ij,nj->...ni", Rm, self.tool_off) + tm[..., None, :])
        return np.concatenate([world_f, fill, tool], axis=-2)


def _pairwise_min(A: np.ndarray, B: np.ndarray):
    """min_{i,j} |A_i - B_j| over the last two axes, with (i, j). A (...,N,3), B (...,M,3).

    Uses the Gram expansion to locate the minimum, then recomputes the exact distance of
    the arg-min pair the way umarell does (``np.linalg.norm(a - b)``)."""
    c = 0.5 * (A.mean(axis=-2, keepdims=True) + B.mean(axis=-2, keepdims=True))
    A0, B0 = A - c, B - c
    d2 = (np.sum(A0 * A0, -1)[..., :, None] + np.sum(B0 * B0, -1)[..., None, :]
          - 2.0 * (A0 @ np.swapaxes(B0, -1, -2)))
    M = d2.shape[-1]
    flat = d2.reshape(d2.shape[:-2] + (-1,))
    k = np.argmin(flat, axis=-1)
    i, j = np.divmod(k, M)
    a = np.take_along_axis(A, i[..., None, None], axis=-2)[..., 0, :]
    b = np.take_along_axis(B, j[..., None, None], axis=-2)[..., 0, :]
    d = np.sqrt(np.sum((a - b) ** 2, axis=-1))
    # guard: rows where another (distinct) pair lies within the Gram rounding error of
    # the minimum get the exact brute-force distance (rare; exact duplicates are harmless)
    near = flat <= np.take_along_axis(flat, k[..., None], -1) + 1e-9
    amb = np.nonzero(near.sum(-1).reshape(-1) > 1)[0]
    if len(amb):
        lead = d.shape
        Af, Bf = A.reshape((-1,) + A.shape[-2:]), B.reshape((-1,) + B.shape[-2:])
        d, i, j = d.reshape(-1).copy(), i.reshape(-1).copy(), j.reshape(-1).copy()
        nf = near.reshape(-1, flat.shape[-1])
        D = np.sqrt(np.sum((Af[amb][:, :, None, :] - Bf[amb][:, None, :, :]) ** 2, -1)).reshape(len(amb), -1)
        D = np.where(nf[amb], D, np.inf)
        k2 = np.argmin(D, axis=-1)
        d[amb] = D[np.arange(len(amb)), k2]
        i[amb], j[amb] = np.divmod(k2, M)
        d, i, j = d.reshape(lead), i.reshape(lead), j.reshape(lead)
    return d, i, j


class SphereModel:
    """umarell's sphere collision model for both arms of a cell (vectorised)."""

    def __init__(self, cell: Cell | str = "welding_cell_2_small", step: float = LINK_SPHERE_STEP_M,
                 pair_radius: float = PAIR_RADIUS):
        if isinstance(cell, str):
            cell = load_cell(cell)
        self.cell = cell
        self.kin = {rid: ArmKinematics(cell.robots[rid]) for rid in ROBOT_IDS}
        self._arm = {rid: _ArmSpheres(self.kin[rid], cell, step) for rid in ROBOT_IDS}
        self.n_spheres = self._arm[ROBOT_IDS[0]].n
        self.pair_radius = float(pair_radius)
        self.warning_distance = float(cell.collision.get("warning_distance", 0.20))
        # static obstacles in obstacles.yaml order (planes ignored, as umarell does)
        bc, bh, bm, bid, sc, sr, sm, sid = [], [], [], [], [], [], [], []
        for o in cell.obstacles:
            if o.get("is_workpiece", False):
                continue
            m = float(o.get("collision_margin", DEFAULT_OBST_MARGIN))
            if o.get("type", "") == "box":
                bc.append(o["center"]); bh.append(o["half_extents"]); bm.append(m); bid.append(str(o.get("id", "obstacle")))
            elif o.get("type", "") == "sphere":
                sc.append(o["center"]); sr.append(float(o["radius"])); sm.append(m); sid.append(str(o.get("id", "obstacle")))
        self.box_center = np.array(bc, float).reshape(-1, 3)
        self.box_half = np.array(bh, float).reshape(-1, 3)
        self.box_margin = np.array(bm, float)
        self.box_ids = bid
        self.sph_center = np.array(sc, float).reshape(-1, 3)
        self.sph_radius = np.array(sr, float)
        self.sph_margin = np.array(sm, float)
        self.sph_ids = sid
        # order of static obstacles as umarell iterates them (for collision_info)
        self._static_order = [("box" if o.get("type") == "box" else "sphere", str(o.get("id", "obstacle")))
                              for o in cell.obstacles
                              if not o.get("is_workpiece", False) and o.get("type") in ("box", "sphere")]
        if len(self.box_center):
            self.box_lo = (self.box_center - self.box_half).min(0)
            self.box_hi = (self.box_center + self.box_half).max(0)

    # ---------------------------------------------------------------- geometry
    def spheres(self, rid: str, q) -> np.ndarray:
        """(..., 77, 3) world sphere centres, same order as umarell ``_config_spheres``."""
        return self._arm[rid].spheres(q)

    def radii(self, rid: str, kind: str = "pair") -> np.ndarray:
        """(77,) radii. ``kind="pair"``: uniform 0.12 used arm-vs-arm. ``kind="obstacle"``:
        radii used against static obstacles (0.12 arm, 0.05 TCP frame, NaN = not checked)."""
        if kind == "pair":
            return np.full(self.n_spheres, self.pair_radius)
        return self._arm[rid].obst_radii.copy()

    def sphere_groups(self, rid: str) -> dict:
        a = self._arm[rid]
        return {"frame": a.sl_frame, "fill": a.sl_fill, "tool": a.sl_tool,
                "floor_idx": a.floor_idx, "obstacle_idx": a.box_idx,
                "frame_names": list(a.kin.frame_names)}

    def keypoints(self, rid: str, q) -> np.ndarray:
        return self.kin[rid].keypoints(q)

    # ---------------------------------------------------------------- arm vs arm
    def pair_min(self, q1, q2, chunk: int = 2048):
        """(clearance (...), i (...), j (...)): min sphere-sphere clearance between robot1
        (config ``q1``) and robot2 (``q2``) and the indices of the closest pair."""
        q1, q2 = np.broadcast_arrays(np.asarray(q1, float), np.asarray(q2, float))
        lead = q1.shape[:-1]
        f1, f2 = q1.reshape(-1, 6), q2.reshape(-1, 6)
        d = np.empty(len(f1)); I = np.empty(len(f1), int); J = np.empty(len(f1), int)
        for s in range(0, len(f1), chunk):
            sl = slice(s, s + chunk)
            u1, u2 = self._arm[ROBOT_IDS[0]].uniq, self._arm[ROBOT_IDS[1]].uniq
            S1 = self.spheres(ROBOT_IDS[0], f1[sl])[:, u1]; S2 = self.spheres(ROBOT_IDS[1], f2[sl])[:, u2]
            d[sl], i_, j_ = _pairwise_min(S1, S2)
            I[sl], J[sl] = u1[i_], u2[j_]
        c = d - 2.0 * self.pair_radius
        return c.reshape(lead), I.reshape(lead), J.reshape(lead)

    def pair_min_from_spheres(self, S1, S2):
        """Like :meth:`pair_min` but from precomputed ``spheres()`` outputs (..., 77, 3)."""
        u1, u2 = self._arm[ROBOT_IDS[0]].uniq, self._arm[ROBOT_IDS[1]].uniq
        d, i, j = _pairwise_min(np.asarray(S1)[..., u1, :], np.asarray(S2)[..., u2, :])
        return d - 2.0 * self.pair_radius, u1[i], u2[j]

    def pair_clearance(self, q1, q2) -> np.ndarray:
        """== ``_check_robot_pair_collision(spheres1, spheres2, 0.12)[1]`` (batched)."""
        return self.pair_min(q1, q2)[0]

    def pair_collision_free(self, q1, q2) -> np.ndarray:
        """umarell's gate: clearance > warning_distance (0.20)."""
        return self.pair_clearance(q1, q2) > self.warning_distance

    # ---------------------------------------------------------------- static obstacles
    def obstacle_terms(self, rid: str, q, extra_margin: float = 0.0, chunk: int = 32) -> dict:
        """Per-config obstacle margins (umarell rules). Keys:
        ``floor`` (...) = min z + 0.05 over floor-checked sphere centres,
        ``floor_idx`` sphere index, ``box`` (...) = min (dist - r - margin) over boxes,
        ``box_obst`` box index, ``box_idx`` sphere index (inf / -1 if no boxes), and the
        same for ``sphere`` obstacles; ``clearance`` = min of all."""
        a = self._arm[rid]
        q = np.asarray(q, float)
        lead = q.shape[:-1]
        S = self.spheres(rid, q).reshape(-1, a.n, 3)
        N = len(S)
        zf = S[:, a.floor_idx_u, 2] - FLOOR_Z_MIN
        kf = np.argmin(zf, axis=1)
        out = {"floor": zf[np.arange(N), kf], "floor_idx": a.floor_idx_u[kf]}
        rb = a.obst_radii[a.box_idx_u]
        for kind, C, H, Mg in (("box", self.box_center, self.box_half, self.box_margin),
                               ("sphere", self.sph_center, None, self.sph_margin)):
            val = np.full(N, np.inf); oi = np.full(N, -1); si = np.full(N, -1)
            if len(C):
                for s in range(0, N, chunk):
                    P = S[s:s + chunk][:, a.box_idx_u, :]                        # (n,M,3)
                    # per-axis accumulation (n,M,B): |p - c| - h clipped at 0 == the
                    # distance from p to umarell's clamped closest point, per axis
                    acc = None
                    for ax in range(3):
                        dd = np.abs(P[:, :, ax, None] - C[:, ax])
                        if kind == "box":
                            dd -= H[:, ax]
                            np.maximum(dd, 0.0, out=dd)
                        dd *= dd
                        acc = dd if acc is None else acc + dd
                    np.sqrt(acc, out=acc)
                    if kind == "box":
                        g = acc - (rb[None, :, None] + Mg[None, None] + extra_margin)
                    else:
                        g = acc - (rb[None, :, None] + self.sph_radius[None, None] + Mg[None, None] + extra_margin)
                    flat = g.reshape(len(g), -1)
                    k = np.argmin(flat, axis=1)
                    mi, bi = np.divmod(k, g.shape[2])
                    val[s:s + chunk] = flat[np.arange(len(k)), k]
                    oi[s:s + chunk] = bi; si[s:s + chunk] = a.box_idx_u[mi]
            out[kind] = val; out[kind + "_obst"] = oi; out[kind + "_idx"] = si
        out["clearance"] = np.minimum(out["floor"], np.minimum(out["box"], out["sphere"]))
        return {k: v.reshape(lead) for k, v in out.items()}

    def obstacle_clearance(self, rid: str, q, extra_margin: float = 0.0) -> np.ndarray:
        """Signed min margin vs floor + static obstacles; ``< 0`` = collision (umarell uses
        strict ``<``, so exactly 0 is still free)."""
        return self.obstacle_terms(rid, q, extra_margin)["clearance"]

    def obstacle_free(self, rid: str, q, extra_margin: float = 0.0) -> np.ndarray:
        """== ``_config_is_free(..., obstacle_spheres=[], static_obstacles, ...)``."""
        return self.obstacle_clearance(rid, q, extra_margin) >= 0.0

    def obstacle_collision_info(self, rid: str, q, extra_margin: float = 0.0) -> dict | None:
        """Single config: the FIRST violation in umarell's order (floor by sphere index,
        then obstacles in yaml order, sphere index inner), like ``_config_collision_info``
        (type, object_id, link_index, point, link_point, clearance_m; rounded to 4 dp)."""
        a = self._arm[rid]
        S = self.spheres(rid, np.asarray(q, float).reshape(6))
        r4 = lambda v: [round(float(x), 4) for x in v]
        for idx in a.floor_idx:
            if S[idx, 2] < FLOOR_Z_MIN:
                return {"type": "floor", "object_id": "floor", "link_index": int(idx),
                        "point": [round(float(S[idx, 0]), 4), round(float(S[idx, 1]), 4), -0.05],
                        "link_point": r4(S[idx]), "clearance_m": round(float(S[idx, 2]) + 0.05, 4)}
        P = S[a.box_idx]
        rb = a.obst_radii[a.box_idx]
        bi = si = 0
        for kind, oid in self._static_order:
            if kind == "box":
                c, h, m = self.box_center[bi], self.box_half[bi], self.box_margin[bi]; bi += 1
                closest = c + np.clip(P - c, -h, h)
                dist = np.sqrt(np.sum((P - closest) ** 2, -1))
                clr = rb + m + extra_margin
                bad = np.nonzero(dist < clr)[0]
                if len(bad):
                    k = bad[0]
                    return {"type": "box", "object_id": oid, "link_index": int(a.box_idx[k]),
                            "point": r4(closest[k]), "link_point": r4(P[k]),
                            "clearance_m": round(float(dist[k] - clr[k]), 4)}
            else:
                c, rr, m = self.sph_center[si], self.sph_radius[si], self.sph_margin[si]; si += 1
                gap = np.sqrt(np.sum((P - c) ** 2, -1)) - (rb + rr + m + extra_margin)
                bad = np.nonzero(gap < 0)[0]
                if len(bad):
                    k = bad[0]
                    return {"type": "sphere", "object_id": oid, "link_index": int(a.box_idx[k]),
                            "point": r4(c), "link_point": r4(P[k]), "clearance_m": round(float(gap[k]), 4)}
        return None

    # ---------------------------------------------------------------- trajectories
    @staticmethod
    def sample_at_time(times, positions, t) -> np.ndarray:
        """umarell ``_sample_at_time`` (linear interp, hold ends), vectorised over ``t``."""
        times = np.asarray(times, float); positions = np.asarray(positions, float)
        t = np.asarray(t, float)
        hi = np.searchsorted(times, t, side="left").clip(0, len(times) - 1)
        lo = np.maximum(hi - 1, 0)
        span = times[hi] - times[lo]
        with np.errstate(divide="ignore", invalid="ignore"):
            f = np.where(span > 0, (t - times[lo]) / np.where(span > 0, span, 1.0), 1.0)
        out = positions[lo] + f[..., None] * (positions[hi] - positions[lo])
        out = np.where((span <= 0)[..., None], positions[hi], out)
        out = np.where((t <= times[0])[..., None], positions[0], out)
        out = np.where((t >= times[-1])[..., None], positions[-1], out)
        return out

    def inter_robot_clearance(self, trajectories: dict, n_samples: int = 20,
                              warning_distance_m: float | None = None,
                              collision_radius_m: float | None = None) -> dict:
        """Replica of umarell ``inter_robot_clearance`` for the two arms.

        ``trajectories``: {rid: [{"positions": [...], "time_s": float}, ...]} or
        {rid: (times (N,), q (N,6))}. Grid: n = max(n_samples, 2, ceil(H*25)+1) samples
        evenly over [0, H], H = max last time_s. Adds raw arrays ``t``, ``clearance``,
        ``closest_pair`` and the unrounded ``min_clearance_raw``."""
        def unpack(tr):
            if isinstance(tr, (list, tuple)) and tr and isinstance(tr[0], dict):
                return (np.array([float(p.get("time_s", 0.0)) for p in tr]),
                        np.array([p["positions"][:6] for p in tr], float))
            return np.asarray(tr[0], float), np.asarray(tr[1], float)
        names = [ns for ns, tr in (trajectories or {}).items() if len(tr)]
        if len(names) < 2:
            return {"collision_free": True, "min_clearance_m": float("inf"), "robots": names,
                    "pairs_checked": 0}
        if names != list(ROBOT_IDS):
            raise ValueError(f"expected trajectories for {ROBOT_IDS}, got {names}")
        wd = self.warning_distance if warning_distance_m is None else float(warning_distance_m)
        tq = {ns: unpack(trajectories[ns]) for ns in names}
        horizon = max(float(tq[ns][0][-1]) for ns in names)
        n = max(int(n_samples), 2, int(np.ceil(max(0.0, horizon) * MONITOR_RATE_HZ)) + 1)
        grid = np.array([horizon * i / (n - 1) for i in range(n)]) if horizon > 0 else np.array([0.0])
        q1 = self.sample_at_time(*tq[names[0]], grid)
        q2 = self.sample_at_time(*tq[names[1]], grid)
        c, i, j = self.pair_min(q1, q2)
        if collision_radius_m is not None:
            c = c + 2.0 * self.pair_radius - 2.0 * float(collision_radius_m)
        k = int(np.argmin(c))
        res = {"collision_free": bool(c[k] > wd), "min_clearance_m": round(float(c[k]), 4),
               "min_clearance_raw": float(c[k]), "min_clearance_time_s": float(grid[k]),
               "warning_distance_m": wd, "robots": names, "pairs_checked": 1,
               "worst_pair": names, "samples_checked": len(grid), "monitor_rate_hz": MONITOR_RATE_HZ,
               "t": grid, "clearance": c, "closest_pair": np.stack([i, j], -1)}
        bad = np.nonzero((c < 0) | (c <= wd))[0]
        if len(bad):
            b = bad[0]
            res["collision_time_s"] = round(float(grid[b]), 4)
            res["colliding_pairs"] = [{"robots": names, "time_s": round(float(grid[b]), 4),
                                       "clearance_m": round(float(c[b]), 4), "penetrating": bool(c[b] < 0)}]
        return res


def sphere_model(cell_id: str = "welding_cell_2_small") -> SphereModel:
    """Cached SphereModel per cell and robot model in effect (robot override + KWR_LIMITS_SOURCE, the key of
    env.cell.load_cell; P3U round 2: keyed on the cell only, it kept the first robot after set_robot_override)."""
    import os
    from robotmodel.override import active_override, dumps
    return _sphere_model(cell_id, dumps(active_override()), os.environ.get("KWR_LIMITS_SOURCE", "auto"))


@lru_cache(maxsize=None)
def _sphere_model(cell_id: str, _ov_json: str, _limits_env: str) -> SphereModel:
    return SphereModel(load_cell(cell_id))


sphere_model.cache_clear = lambda: _sphere_model.cache_clear()      # the old lru_cache API
