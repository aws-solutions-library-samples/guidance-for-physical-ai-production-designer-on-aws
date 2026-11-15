"""One robot of a scene: URDF chain, world placement (full base rotation), batched numpy FK, collision spheres and
collision-hull vertices. Robot-count and placement agnostic (no robot ids, no cell lookups).

    rm = RobotModel(scene.robot("robot1"))
    p, R = rm.tcp(Q)                 # (..., 3), (..., 3, 3) world TCP (scene TCP frame: tip_link * [tcp_R, tcp_offset])
    S = rm.spheres(Q)                # (..., n_link + n_tool, 3) link spheres (umarell layout) + the robot's tool spheres
    U = rm.umarell_spheres(Q)        # link spheres + umarell's tool grid (only with umarell_tool given)
    T = rm.link_transforms(Q)        # (..., n_chain_links, 4, 4) world transforms of the chain links

Link-sphere layout (umarell's ``_config_spheres`` order, 42 spheres for a 6-axis chain with fill step 0.12 on the
KR240): 3 spheres at the URDF root (universe / root_joint / World), base_link, (joint_k, link_k) for k = 1..6 (both at
the joint origin), the TCP; then fill spheres every ``fill_step`` between consecutive joint origins 1..6. Tool
spheres are rigid in the robot's ``tool_spheres.frame`` link.
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from functools import lru_cache

import numpy as np

from trajgen.scene import RobotSpec, rpy_matrix


def _floats(s, n=3):
    return np.zeros(n) if s is None else np.array([float(v) for v in str(s).split()], float)


def _T(R=None, p=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


@lru_cache(maxsize=None)
def parse_chain(urdf: str, tip_link: str):
    """Serial chain root -> tip_link. Returns dict(root, links, joints, prefix, fixed_links) with
    joints = [{name, child, T_origin (4x4, in the parent joint frame, fixed joints folded in), axis, lower, upper}]
    (exactly 6 movable joints required), links = [base_link, link_1 .. link_6] (child links of the movable joints,
    base_link = parent of the first one), fixed_links = {link: (joint index 0..6, T in that joint frame)} for every
    link on the chain incl. fixed ones after the last movable joint."""
    from robotmodel.urdf_robot import parse_urdf     # the project's single URDF parser (docs/p3_urdf.md)
    u = parse_urdf(urdf)
    by_child = {}
    for j in u.joints:
        by_child[j.child] = dict(
            name=j.name, type=j.type, parent=j.parent, child=j.child,
            T=_T(rpy_matrix(*np.array(j.rpy, float)), np.array(j.xyz, float)),
            axis=np.array(j.axis, float), lower=j.lower, upper=j.upper, velocity=j.velocity, effort=j.effort)
    links = set(u.links)
    if tip_link not in links:
        raise ValueError(f"tip_link {tip_link} not in {urdf}")
    chain = []
    link = tip_link
    while link in by_child:
        chain.append(by_child[link])
        link = by_child[link]["parent"]
    chain.reverse()
    root_link = link
    movable = [j for j in chain if j["type"] in ("revolute", "continuous")]
    if len(movable) != 6:
        raise ValueError(f"{urdf}: expected 6 revolute joints between {root_link} and {tip_link}, got {len(movable)}")
    if any(j["type"] not in ("revolute", "continuous", "fixed") for j in chain):
        raise ValueError(f"{urdf}: only revolute/continuous/fixed joints are supported on the arm chain")
    prefix = np.eye(4)
    joints, acc = [], np.eye(4)
    fixed_links = {root_link: (0, np.eye(4))}
    k = 0
    base_link = root_link
    for j in chain:
        if j["type"] == "fixed":
            acc = acc @ j["T"]
            if k == 0:
                prefix = prefix @ j["T"]
                acc = np.eye(4)
                base_link = j["child"]
                fixed_links[j["child"]] = (0, prefix.copy())
            else:
                fixed_links[j["child"]] = (k, acc.copy())
            continue
        ax = j["axis"] / np.linalg.norm(j["axis"])
        joints.append(dict(name=j["name"], child=j["child"], T_origin=acc @ j["T"], axis=ax,
                           lower=j["lower"], upper=j["upper"], velocity=j["velocity"], effort=j["effort"]))
        acc = np.eye(4)
        k += 1
        fixed_links[j["child"]] = (k, np.eye(4))
    # the prefix maps the root frame to base_link; the first joint origin is expressed in base_link
    return dict(root=root_link, base_link=base_link, prefix=prefix, joints=joints,
                links=[base_link] + [j["child"] for j in joints], fixed_links=fixed_links)


@lru_cache(maxsize=None)
def collision_hulls(urdf: str) -> dict:
    """{link name: [(n, 3) vertex arrays in the link frame, <origin xyz rpy> applied]} of every <collision> mesh."""
    import trimesh

    root = ET.parse(urdf).getroot()
    base = os.path.dirname(urdf)
    out = {}
    for link in root.findall("link"):
        L = []
        for c in link.findall("collision"):
            o = c.find("origin")
            xyz = _floats(o.get("xyz") if o is not None else None)
            rpy = _floats(o.get("rpy") if o is not None else None)
            g = c.find("geometry")
            mesh = g.find("mesh") if g is not None else None
            if mesh is None:
                box = g.find("box") if g is not None else None
                if box is None:
                    continue
                h = 0.5 * _floats(box.get("size"))
                V = np.array([[sx, sy, sz] for sx in (-h[0], h[0]) for sy in (-h[1], h[1]) for sz in (-h[2], h[2])])
            else:
                fn = mesh.get("filename")
                if fn.startswith("package://") or fn.startswith("file://"):
                    raise ValueError(f"{urdf}: mesh URI {fn} not supported; use paths relative to the URDF")
                m = trimesh.load(os.path.join(base, fn), force="mesh", process=False)
                V = np.asarray(m.vertices, float) * _floats(mesh.get("scale") or "1 1 1")
            V = V @ rpy_matrix(*rpy).T + xyz
            L.append(V)
        if L:
            out[link.get("name")] = L
    return out


def _rodrigues(axis, c, s):
    """(..., 3, 3) rotation about the unit ``axis`` by the angle with cos c, sin s (broadcast)."""
    a = np.asarray(axis, float)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    K2 = K @ K
    c = np.asarray(c)[..., None, None]
    s = np.asarray(s)[..., None, None]
    return np.eye(3) + s * K + (1 - c) * K2


class RobotModel:
    def __init__(self, robot: RobotSpec, umarell_tool: dict | None = None, umarell_radius: float = 0.12):
        self.spec = robot
        self.rid = robot.id
        self.urdf = str(robot.urdf)
        ch = parse_chain(self.urdf, robot.tool.tip_link)
        self.chain = ch
        self.link_names = list(ch["links"])                    # base_link, link_1..link_6
        self.joint_names = [j["name"] for j in ch["joints"]]
        self._To = np.stack([j["T_origin"] for j in ch["joints"]])  # (6, 4, 4)
        self._axis = [j["axis"] for j in ch["joints"]]
        self._coord_axis = []
        for a in self._axis:                                    # fast path for coordinate axes
            k = int(np.argmax(np.abs(a)))
            self._coord_axis.append((k, float(np.sign(a[k]))) if np.isclose(abs(a[k]), 1.0) else None)
        self.base = np.asarray(robot.base_xyz, float)
        self.Rb = np.asarray(robot.base_R, float)
        self.prefix = ch["prefix"]
        self.q_lo, self.q_hi = np.asarray(robot.q_lo, float), np.asarray(robot.q_hi, float)
        self.home = np.asarray(robot.home, float)
        tip_k, T_tip = ch["fixed_links"][robot.tool.tip_link]
        self.tip_joint, self.T_tip = tip_k, T_tip
        self.T_tcp = T_tip @ _T(robot.tool.tcp_R, robot.tool.tcp_offset)   # TCP in the frame of joint tip_k
        self.tool_axis_local = np.asarray(robot.tool.axis, float)
        c = robot.coll
        self.link_radius, self.tcp_radius, self.fill_step = c.link_radius, c.tcp_radius, c.fill_step
        ts = c.tool_spheres
        if ts.frame not in ch["fixed_links"]:
            raise ValueError(f"{robot.id}: tool_spheres.frame {ts.frame} is not on the arm chain")
        self.tool_joint, T_m = ch["fixed_links"][ts.frame]
        self.tool_c = ts.centers @ T_m[:3, :3].T + T_m[:3, 3]   # in the frame of joint tool_joint
        self.tool_r = np.asarray(ts.radii, float)
        # ---- link-sphere layout
        self.frame_joint = np.array([0, 0, 0, 0] + [k for k in range(1, 7) for _ in (0, 1)], int)
        self.frame_names = ["universe", "root_joint", "World", self.link_names[0]] + \
            [n for k in range(6) for n in (self.joint_names[k], self.link_names[k + 1])] + ["welding_tcp"]
        self.n_frame = len(self.frame_joint) + 1                # + TCP
        self.tcp_frame = self.n_frame - 1
        _, ps = self.joint_poses_local(np.zeros(6))
        self.fill_seg, self.fill_t = [], []
        for j in range(1, 6):
            seg = float(np.linalg.norm(ps[j + 1] - ps[j]))
            n = int(seg / self.fill_step) if self.fill_step > 0 else 0
            for k in range(1, n + 1):
                t = k * self.fill_step
                if t >= seg:
                    break
                self.fill_seg.append(j)
                self.fill_t.append(t)
        self.fill_seg = np.array(self.fill_seg, int)
        self.fill_t = np.array(self.fill_t, float)
        self.n_link = self.n_frame + len(self.fill_t)
        self.base_frame = 3
        r = np.full(self.n_link, self.link_radius)
        r[self.tcp_frame] = self.tcp_radius
        self.link_r = r
        self.n_tool = len(self.tool_r)
        self.sphere_r = np.concatenate([r, self.tool_r])
        self.electrode = int(np.argmin(np.linalg.norm(ts.centers - self._tcp_in_frame(ts.frame), axis=1))) if self.n_tool else -1
        # ---- umarell's tool grid (parity model)
        self.um_tool = None
        if umarell_tool:
            from env.kinematics import tool_offsets
            off = tool_offsets(umarell_tool)
            token = umarell_tool.get("mount_link", "link_6")
            mount = next((ln for ln in self.link_names if token and token in ln), None)
            if mount is None or not len(off):
                self.um_tool = None
            else:
                kk, Tm = ch["fixed_links"][mount]
                self.um_joint = kk
                self.um_tool = off @ Tm[:3, :3].T + Tm[:3, 3]
        self.um_radius = float(umarell_radius)
        # ---- unique sphere centres for the pair checks (coincident frame origins dropped)
        Q = np.random.default_rng(0).uniform(self.q_lo, self.q_hi, (4, 6))
        S = self.spheres(Q)
        self.pair_idx = _unique_cols(S)
        if self.um_tool is not None:
            self.um_pair_idx = _unique_cols(self.umarell_spheres(Q))
        self._hulls = None

    def _tcp_in_frame(self, frame):
        """TCP position in the frame of link ``frame`` (for picking the electrode sphere)."""
        k, Tm = self.chain["fixed_links"][frame]
        if k != self.tip_joint:
            return np.full(3, np.inf)
        return np.linalg.solve(Tm, np.append(self.T_tcp[:3, 3], 1.0))[:3]

    # ------------------------------------------------------------------ FK
    def joint_poses_local(self, q):
        """(R (..., 7, 3, 3), p (..., 7, 3)) joint frames in the URDF ROOT frame; index 0 = base_link."""
        q = np.asarray(q, float)
        lead = q.shape[:-1]
        Rs = np.empty(lead + (7, 3, 3))
        ps = np.empty(lead + (7, 3))
        R = np.broadcast_to(self.prefix[:3, :3], lead + (3, 3)).copy()
        p = np.broadcast_to(self.prefix[:3, 3], lead + (3,)).copy()
        Rs[..., 0, :, :], ps[..., 0, :] = R, p
        c, s = np.cos(q), np.sin(q)
        for i in range(6):
            To = self._To[i]
            p = R @ To[:3, 3] + p
            R = R @ To[:3, :3]
            ca = self._coord_axis[i]
            if ca is not None:
                Rq = _axis_rot(ca[0], c[..., i], ca[1] * s[..., i])
            else:
                Rq = _rodrigues(self._axis[i], c[..., i], s[..., i])
            R = R @ Rq
            Rs[..., i + 1, :, :], ps[..., i + 1, :] = R, p
        return Rs, ps

    def joint_poses(self, q):
        """World joint frames (base placement applied)."""
        Rs, ps = self.joint_poses_local(q)
        return self.Rb @ Rs, ps @ self.Rb.T + self.base

    def tcp(self, q):
        """World TCP (position (..., 3), rotation (..., 3, 3)) in the scene's TCP frame."""
        Rs, ps = self.joint_poses(q)
        k = self.tip_joint
        R = Rs[..., k, :, :]
        return R @ self.T_tcp[:3, 3] + ps[..., k, :], R @ self.T_tcp[:3, :3]

    def tool_axis_world(self, q):
        _, R = self.tcp(q)
        return R @ self.tool_axis_local

    def link_transforms(self, q):
        """(..., 7, 4, 4) world transforms of [base_link, link_1 .. link_6]."""
        Rs, ps = self.joint_poses(q)
        T = np.zeros(Rs.shape[:-2] + (4, 4))
        T[..., :3, :3] = Rs
        T[..., :3, 3] = ps
        T[..., 3, 3] = 1.0
        return T

    # ------------------------------------------------------------------ spheres
    def _link_part(self, Rs, ps):
        pf = ps[..., self.frame_joint, :]
        R6 = Rs[..., self.tip_joint, :, :]
        tcp = R6 @ self.T_tcp[:3, 3] + ps[..., self.tip_joint, :]
        a = ps[..., self.fill_seg, :]
        b = ps[..., self.fill_seg + 1, :]
        d = b - a
        seg = np.sqrt(np.sum(d * d, axis=-1, keepdims=True))
        fill = a + (d / seg) * self.fill_t[:, None]
        # the 3 root frames sit at the URDF root (= base_link origin without a prefix)
        root = np.broadcast_to(self.base, pf[..., :3, :].shape)
        return np.concatenate([root, pf[..., 3:, :], tcp[..., None, :], fill], axis=-2)

    def spheres(self, q) -> np.ndarray:
        """(..., n_link + n_tool, 3) world sphere centres (radii: ``sphere_r``)."""
        Rs, ps = self.joint_poses(q)
        L = self._link_part(Rs, ps)
        k = self.tool_joint
        tool = np.einsum("...ij,nj->...ni", Rs[..., k, :, :], self.tool_c) + ps[..., k, None, :]
        return np.concatenate([L, tool], axis=-2)

    def umarell_spheres(self, q) -> np.ndarray:
        """Link spheres + umarell's tool grid (radius ``um_radius`` for every sphere in the arm-arm gate)."""
        if self.um_tool is None:
            raise ValueError(f"{self.rid}: no umarell tool grid (scene declares no umarell cell)")
        Rs, ps = self.joint_poses(q)
        L = self._link_part(Rs, ps)
        k = self.um_joint
        tool = np.einsum("...ij,nj->...ni", Rs[..., k, :, :], self.um_tool) + ps[..., k, None, :]
        return np.concatenate([L, tool], axis=-2)

    # ------------------------------------------------------------------ hulls
    @property
    def hulls(self):
        """{chain joint index: (n, 3) vertices in that joint frame} of every collision hull on the chain links."""
        if self._hulls is None:
            H = collision_hulls(self.urdf)
            out = {}
            for ln, Vs in H.items():
                if ln not in self.chain["fixed_links"]:
                    continue
                k, Tl = self.chain["fixed_links"][ln]
                V = np.concatenate(Vs) @ Tl[:3, :3].T + Tl[:3, 3]
                out[k] = V if k not in out else np.concatenate([out[k], V])
            self._hulls = out
        return self._hulls

    def hull_min_z(self, q, include_base: bool = False) -> np.ndarray:
        """(...) lowest world z over the collision-hull vertices of links 1..6 (and base_link if asked)."""
        Rs, ps = self.joint_poses(q)
        zmin = None
        for k, V in self.hulls.items():
            if k == 0 and not include_base:
                continue
            z = np.einsum("...j,nj->...n", Rs[..., k, 2, :], V) + ps[..., k, 2][..., None]
            m = z.min(-1)
            zmin = m if zmin is None else np.minimum(zmin, m)
        return zmin if zmin is not None else np.full(np.shape(q)[:-1], np.inf)


def _axis_rot(axis_id: int, c, s):
    c = np.asarray(c, float)
    R = np.zeros(c.shape + (3, 3))
    i, j = [(1, 2), (2, 0), (0, 1)][axis_id]
    R[..., axis_id, axis_id] = 1.0
    R[..., i, i] = c
    R[..., j, j] = c
    R[..., i, j] = -s
    R[..., j, i] = s
    return R


def _unique_cols(S):
    """Indices of the distinct sphere columns of S (n_q, n, 3) (first of every coincident group)."""
    keep = []
    for i in range(S.shape[1]):
        if not any(np.allclose(S[:, i], S[:, k], atol=1e-12) for k in keep):
            keep.append(i)
    return np.array(keep, int)


def internal_tcp_rotation(axis_local) -> np.ndarray:
    """C with C @ [-1, 0, 0] = axis_local: the IK works in the frame R_tcp @ C, whose -x is the tool axis (the frame
    expert.ik.WeldIK assumes; its spin is about the tool axis)."""
    from trajgen.scene import frame_from_axis
    return frame_from_axis(-np.asarray(axis_local, float))
