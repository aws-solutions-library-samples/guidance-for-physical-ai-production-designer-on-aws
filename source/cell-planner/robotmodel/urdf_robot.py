"""URDF robot model: the single source of a robot's kinematics and joint limits (track P3-URDF, docs/p3_urdf.md).

    from robotmodel import URDFRobot
    rm = URDFRobot("path/to/robot.urdf", tip_link="like_kr240r2900_link_6", tcp_offset=[-0.9092, 0.0005, -0.0056],
                   base_xyz=[0.07, -3.32, 0.0], base_yaw=0.0)
    rm.q_lo, rm.q_hi, rm.v_max, rm.effort        # from <limit lower upper velocity effort> of the movable joints
    rm.a_max, rm.j_max                           # NOT in URDFs: explicit override, else a per-model default (REVIEW)
    p, R = rm.tcp(Q)                             # (..., 3), (..., 3, 3) world TCP, batched over Q (..., n_dof)
    J = rm.jacobian(Q)                           # (..., 6, n_dof) world [linear; angular] at the TCP
    T = rm.link_transforms(Q)                    # {link name: (..., 4, 4)} world transforms of every chain link

What is read from the URDF: every <link> (visual / collision geometry: mesh file resolved to an absolute path, box,
cylinder, sphere, each with its <origin>), every <joint> (type, parent, child, <origin xyz rpy>, <axis>, <limit lower
upper velocity effort>). The arm is the serial chain from the URDF root to ``tip_link`` (default: the child of the
deepest movable joint); fixed joints anywhere on the chain are folded into the next movable joint's origin (or into
the tip transform after the last one). Revolute, continuous and prismatic joints are supported.

TCP: ``tip_link`` frame * [tcp_R, tcp_offset] (``tcp_rpy`` is an alternative to ``tcp_R``). World placement:
``base_xyz`` + ``base_R`` (or ``base_yaw``) applied to the URDF root frame.

Acceleration / jerk limits: a URDF has no acceleration limits. ``a_max`` is, in this order, the ``accel`` argument
(robots.yaml ``joint_acceleration_limits`` / scene.yaml ``joints.acceleration`` / a CLI override), else a default per
robot model (``ACCEL_DEFAULTS``; KR240 R2900: [3, 3, 3, 6, 6, 8] rad/s^2, umarell's "REVIEW" default, NOT measured),
else None. ``accel_source`` records which. Jerk: only an explicit ``jerk`` override (else None).

Pure numpy; FK / Jacobian cross-checked against env/kinematics.py ArmKinematics, MuJoCo and pinocchio to 1e-9
(tests/test_urdf_robot.py).
"""
from __future__ import annotations

import json
import math
import os
import warnings
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

import numpy as np

# Acceleration defaults per robot model (URDFs carry none). Keys are matched as substrings of the URDF <robot name>,
# the file name or the explicit ``model`` argument. REVIEW: umarell's robots.yaml default, not from a datasheet.
ACCEL_DEFAULTS = {
    "kr240r2900": dict(a_max=[3.0, 3.0, 3.0, 6.0, 6.0, 8.0], source="default:kr240r2900 (umarell robots.yaml "
                                                                     "REVIEW default, not measured)"),
}
MOVABLE = ("revolute", "continuous", "prismatic")


# ----------------------------------------------------------------------------------------------- math helpers
def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    """URDF rpy -> R = Rz(y) Ry(p) Rx(r) (same arithmetic as env/kinematics.py and trajgen/scene.py)."""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return Rz @ Ry @ Rx


def rot_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _floats(s, n=3) -> np.ndarray:
    return np.zeros(n) if s is None else np.array([float(v) for v in str(s).split()], float)


def _T(R=None, p=None) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def _axis_rot(axis_id: int, c, s):
    """(..., 3, 3) rotation about the coordinate axis ``axis_id`` (0 = x, 1 = y, 2 = z)."""
    c = np.asarray(c, float)
    R = np.zeros(c.shape + (3, 3))
    i, j = [(1, 2), (2, 0), (0, 1)][axis_id]
    R[..., axis_id, axis_id] = 1.0
    R[..., i, i] = c
    R[..., j, j] = c
    R[..., i, j] = -s
    R[..., j, i] = s
    return R


def _rodrigues(axis, c, s):
    a = np.asarray(axis, float)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    c = np.asarray(c)[..., None, None]
    s = np.asarray(s)[..., None, None]
    return np.eye(3) + s * K + (1 - c) * (K @ K)


# ----------------------------------------------------------------------------------------------- data classes
@dataclass(frozen=True)
class Geometry:
    kind: str                         # "mesh" | "box" | "cylinder" | "sphere"
    xyz: tuple                        # <origin xyz> in the link frame
    rpy: tuple
    filename: str | None = None       # mesh: absolute path (package:// and file:// resolved where possible)
    scale: tuple = (1.0, 1.0, 1.0)
    size: tuple | None = None         # box: full extents
    radius: float | None = None       # cylinder / sphere
    length: float | None = None       # cylinder
    name: str | None = None

    @property
    def T(self) -> np.ndarray:
        return _T(rpy_matrix(*self.rpy), np.asarray(self.xyz, float))


@dataclass(frozen=True)
class Link:
    name: str
    visuals: tuple = ()
    collisions: tuple = ()
    mass: float | None = None


@dataclass(frozen=True)
class Joint:
    name: str
    type: str
    parent: str
    child: str
    xyz: tuple
    rpy: tuple
    axis: tuple
    lower: float | None
    upper: float | None
    velocity: float | None
    effort: float | None

    @property
    def T_origin(self) -> np.ndarray:
        """<origin> as a 4x4 (child frame at q = 0 in the parent link frame)."""
        return _T(rpy_matrix(*self.rpy), np.asarray(self.xyz, float))

    @property
    def movable(self) -> bool:
        return self.type in MOVABLE


@dataclass(frozen=True)
class URDFData:
    path: str
    name: str
    links: dict = field(repr=False)          # name -> Link
    joints: tuple = field(repr=False)        # Joint, file order
    by_child: dict = field(repr=False)       # child link -> Joint
    root: str = ""


def _resolve_mesh(fn: str, base: str) -> str:
    if fn.startswith("file://"):
        fn = fn[len("file://"):]
    if fn.startswith("package://"):
        rest = fn[len("package://"):]
        pkg, _, sub = rest.partition("/")
        d = Path(base).resolve()
        for anc in [d] + list(d.parents):                   # <ancestor>/<pkg>/<sub>, or the package is the URDF dir
            if (anc / pkg / sub).exists():
                return str(anc / pkg / sub)
            if anc.name == pkg and (anc / sub).exists():
                return str(anc / sub)
        return fn                                           # unresolved: kept as given
    return fn if os.path.isabs(fn) else os.path.normpath(os.path.join(base, fn))


def _geom(e, base: str) -> Geometry | None:
    o = e.find("origin")
    xyz = tuple(_floats(o.get("xyz") if o is not None else None))
    rpy = tuple(_floats(o.get("rpy") if o is not None else None))
    g = e.find("geometry")
    if g is None:
        return None
    nm = e.get("name")
    m = g.find("mesh")
    if m is not None:
        return Geometry("mesh", xyz, rpy, filename=_resolve_mesh(m.get("filename"), base),
                        scale=tuple(_floats(m.get("scale") or "1 1 1")), name=nm)
    b = g.find("box")
    if b is not None:
        return Geometry("box", xyz, rpy, size=tuple(_floats(b.get("size"))), name=nm)
    c = g.find("cylinder")
    if c is not None:
        return Geometry("cylinder", xyz, rpy, radius=float(c.get("radius")), length=float(c.get("length")), name=nm)
    s = g.find("sphere")
    if s is not None:
        return Geometry("sphere", xyz, rpy, radius=float(s.get("radius")), name=nm)
    return None


def _fattr(e, k):
    v = e.get(k) if e is not None else None
    return float(v) if v not in (None, "") else None


@lru_cache(maxsize=None)
def parse_urdf(path: str) -> URDFData:
    """Parse a URDF file (cached per path; the file is assumed not to change during a process)."""
    path = str(path)
    root = ET.parse(path).getroot()
    base = os.path.dirname(os.path.abspath(path))
    links = {}
    for l in root.findall("link"):
        inert = l.find("inertial")
        mass = _fattr(inert.find("mass"), "value") if inert is not None and inert.find("mass") is not None else None
        links[l.get("name")] = Link(
            l.get("name"), tuple(g for g in (_geom(v, base) for v in l.findall("visual")) if g is not None),
            tuple(g for g in (_geom(c, base) for c in l.findall("collision")) if g is not None), mass)
    joints, by_child = [], {}
    for j in root.findall("joint"):
        o, a, lim = j.find("origin"), j.find("axis"), j.find("limit")
        jt = Joint(
            name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            xyz=tuple(_floats(o.get("xyz") if o is not None else None)),
            rpy=tuple(_floats(o.get("rpy") if o is not None else None)),
            axis=tuple(_floats(a.get("xyz") if a is not None else "1 0 0")),
            lower=_fattr(lim, "lower"), upper=_fattr(lim, "upper"), velocity=_fattr(lim, "velocity"),
            effort=_fattr(lim, "effort"))
        joints.append(jt)
        by_child[jt.child] = jt
    roots = [n for n in links if n not in by_child]
    if len(roots) != 1:
        raise ValueError(f"{path}: expected exactly one root link, got {roots}")
    return URDFData(path, root.get("name", ""), links, tuple(joints), by_child, roots[0])


def default_tip_link(u: URDFData) -> str:
    """Child link of the deepest movable joint (umarell's ``tip_link`` rule)."""
    def depth(link):
        d = 0
        while link in u.by_child:
            link = u.by_child[link].parent
            d += 1
        return d
    mv = [j for j in u.joints if j.movable]
    if not mv:
        raise ValueError(f"{u.path}: no movable joints")
    return max((j.child for j in mv), key=depth)


def kinematics_md5(path) -> str:
    """md5 of what a URDF says about the robot's geometry, without its <limit> elements: every joint (name, type,
    parent, child, origin, axis) and every link's collision geometry (kind, origin, size / radius / length / scale,
    mesh file content). Two URDFs with the same value move and collide identically; they may differ in their joint
    limits (velocity, position). Used to tell a limits-only URDF edit from a kinematic one (P3U round 2)."""
    import hashlib
    u = parse_urdf(str(Path(path).resolve()))
    h = hashlib.md5()
    r9 = lambda v: [round(float(x), 9) for x in v]   # noqa: E731
    for j in sorted(u.joints, key=lambda j: j.name):
        h.update(json.dumps([j.name, j.type, j.parent, j.child, r9(j.xyz), r9(j.rpy), r9(j.axis)]).encode())
    for name in sorted(u.links):
        for g in u.links[name].collisions:
            fm = None
            if g.filename and Path(g.filename).is_file():
                fm = hashlib.md5(Path(g.filename).read_bytes()).hexdigest()
            h.update(json.dumps([name, g.kind, r9(g.xyz), r9(g.rpy), fm, r9(g.scale),
                                 r9(g.size) if g.size else None, g.radius, g.length]).encode())
    return h.hexdigest()


def fixed_descendants(u: URDFData, link: str) -> dict:
    """{child link: T (4x4) in ``link``'s frame} of every link attached to ``link`` through fixed joints only."""
    out, todo = {}, [(link, np.eye(4))]
    while todo:
        parent, Tp = todo.pop()
        for j in u.joints:
            if j.parent == parent and j.type == "fixed":
                T = Tp @ j.T_origin
                out[j.child] = T
                todo.append((j.child, T))
    return out


TCP_NAMES = ("tcp", "tool0", "tool_tcp", "weld_tcp", "ee", "end_effector", "flange_tcp")


def tcp_link_note(rm, tcp_source: str) -> str | None:
    """A warning text when the URDF carries fixed links after ``tip_link`` (candidate TCP frames) but the TCP in use
    comes from a hand-copied offset (``tcp_source``), else None."""
    c = getattr(rm, "tcp_candidates", {}) or {}
    if not c:
        return None
    desc = ", ".join(f"{n} at {np.round(T[:3, 3], 4).tolist()}" + (" (TCP-like name)" if any(
        k in n.lower() for k in TCP_NAMES) else "") for n, T in sorted(c.items()))
    return (f"{Path(rm.path).name}: fixed link(s) after tip_link {rm.tip_link}: {desc}; the TCP used is {tcp_source} "
            f"{np.round(rm.tcp_offset, 4).tolist()} in {rm.tip_link}. If the URDF's frame is the TCP, name it as "
            "tip_link (robot override 'tip_link', robots.yaml / scene tool tip_link) with tcp_offset [0, 0, 0]")


def accel_default(*names: str) -> dict | None:
    for key, v in ACCEL_DEFAULTS.items():
        if any(key in (n or "").lower().replace("_", "").replace("-", "") for n in names):
            return v
    return None


# ----------------------------------------------------------------------------------------------- model
class URDFRobot:
    """Serial-chain robot model read from a URDF (see the module docstring)."""

    def __init__(self, urdf, *, tip_link: str | None = None, tcp_offset=None, tcp_R=None, tcp_rpy=None,
                 accel=None, jerk=None, model: str | None = None, base_xyz=None, base_R=None, base_yaw=None,
                 accel_source: str | None = None):
        self.path = str(Path(urdf).resolve())
        u = parse_urdf(self.path)
        self.data = u
        self.name = u.name
        self.links = u.links
        self.joints = {j.name: j for j in u.joints}
        self.root_link = u.root
        self.tip_link = tip_link or default_tip_link(u)
        if self.tip_link not in u.links:
            raise ValueError(f"{self.path}: tip_link {self.tip_link!r} not in the URDF")
        chain, link = [], self.tip_link
        while link in u.by_child:
            chain.append(u.by_child[link])
            link = u.by_child[link].parent
        chain.reverse()
        self.chain = chain                                   # all joints root -> tip (incl. fixed)
        bad = [j.name for j in chain if j.type not in MOVABLE + ("fixed",)]
        if bad:
            raise ValueError(f"{self.path}: unsupported joint types on the chain: {bad}")
        self.movable = [j for j in chain if j.movable]
        if not self.movable:
            raise ValueError(f"{self.path}: no movable joint between {self.root_link} and {self.tip_link}")
        self.n_dof = len(self.movable)
        self.joint_names = [j.name for j in self.movable]
        self.joint_types = [j.type for j in self.movable]
        # fold fixed joints: prefix (root -> parent link of joint 1), per-joint origins, tip transform
        self.prefix = np.eye(4)
        T_or, acc, k = [], np.eye(4), 0
        self.fixed_links = {self.root_link: (0, np.eye(4))}  # link -> (movable-joint frame index 0..n, T in it)
        self.base_link = self.root_link
        for j in chain:
            if not j.movable:
                acc = acc @ j.T_origin
                if k == 0:
                    self.prefix = self.prefix @ j.T_origin
                    acc = np.eye(4)
                    self.base_link = j.child
                    self.fixed_links[j.child] = (0, self.prefix.copy())   # root frame; re-expressed below
                else:
                    self.fixed_links[j.child] = (k, acc.copy())
                continue
            T_or.append(acc @ j.T_origin)
            acc = np.eye(4)
            k += 1
            self.fixed_links[j.child] = (k, np.eye(4))
        Pi = np.linalg.inv(self.prefix)                      # links before joint 1: express them in frame 0
        for ln, (kk, Tl) in list(self.fixed_links.items()):
            if kk == 0:
                self.fixed_links[ln] = (0, Pi @ Tl)
        self._To = np.stack(T_or)                            # (n, 4, 4) in the previous movable joint frame
        self._axis = [np.asarray(j.axis, float) / np.linalg.norm(j.axis) for j in self.movable]
        self._coord = []
        for a in self._axis:
            i = int(np.argmax(np.abs(a)))
            self._coord.append((i, float(np.sign(a[i]))) if np.isclose(abs(a[i]), 1.0) else None)
        self._prism = np.array([j.type == "prismatic" for j in self.movable])
        self.link_names = [self.base_link] + [j.child for j in self.movable]
        # TCP
        tk, T_tip = self.fixed_links[self.tip_link]
        if tk != self.n_dof:
            raise ValueError("tip_link must be after the last movable joint")
        if tcp_R is None:
            tcp_R = np.eye(3) if tcp_rpy is None else rpy_matrix(*np.asarray(tcp_rpy, float))
        self.tcp_offset = np.zeros(3) if tcp_offset is None else np.asarray(tcp_offset, float).reshape(3)
        self.tcp_R = np.asarray(tcp_R, float).reshape(3, 3)
        self.T_tip = T_tip
        self.T_tcp = T_tip @ _T(self.tcp_R, self.tcp_offset)  # TCP in the last movable joint frame
        # links hanging off tip_link through fixed joints only: candidate TCP frames the URDF itself carries
        # ({name: 4x4 in the tip_link frame}); the TCP used is tip_link * tcp_offset unless tip_link names one
        self.tcp_candidates = fixed_descendants(u, self.tip_link)
        # world placement
        self.base_xyz = np.zeros(3) if base_xyz is None else np.asarray(base_xyz, float).reshape(3)
        if base_R is not None:
            self.base_R = np.asarray(base_R, float).reshape(3, 3)
        else:
            self.base_R = rot_z(float(base_yaw or 0.0))
        # limits
        inf = np.inf
        self.q_lo = np.array([j.lower if j.lower is not None and j.type != "continuous" else -inf for j in self.movable])
        self.q_hi = np.array([j.upper if j.upper is not None and j.type != "continuous" else inf for j in self.movable])
        self.v_max = np.array([j.velocity if j.velocity is not None else np.nan for j in self.movable])
        self.effort = np.array([j.effort if j.effort is not None else np.nan for j in self.movable])
        self.model = model or ""
        if accel is not None:
            self.a_max = np.asarray(accel, float).reshape(self.n_dof)
            self.accel_source = accel_source or "override"
        else:
            d = accel_default(self.model, self.name, Path(self.path).stem)
            if d is not None and len(d["a_max"]) == self.n_dof:
                self.a_max, self.accel_source = np.asarray(d["a_max"], float), d["source"]
            else:
                self.a_max, self.accel_source = None, "none (URDFs carry no acceleration limits)"
        self.j_max = None if jerk is None else np.asarray(jerk, float).reshape(self.n_dof)

    # ------------------------------------------------------------------ FK
    def joint_poses_local(self, q):
        """(R (..., n+1, 3, 3), p (..., n+1, 3)) movable-joint frames in the URDF ROOT frame; index 0 = base_link
        (after the fixed prefix), i = frame of movable joint i (= its child link frame)."""
        q = np.asarray(q, float)
        lead = q.shape[:-1]
        n = self.n_dof
        Rs = np.empty(lead + (n + 1, 3, 3))
        ps = np.empty(lead + (n + 1, 3))
        R = np.broadcast_to(self.prefix[:3, :3], lead + (3, 3)).copy()
        p = np.broadcast_to(self.prefix[:3, 3], lead + (3,)).copy()
        Rs[..., 0, :, :], ps[..., 0, :] = R, p
        for i in range(n):
            To = self._To[i]
            p = R @ To[:3, 3] + p
            R = R @ To[:3, :3]
            if self._prism[i]:
                p = p + (R @ self._axis[i]) * q[..., i, None]
            else:
                c, s = np.cos(q[..., i]), np.sin(q[..., i])
                ca = self._coord[i]
                R = R @ (_axis_rot(ca[0], c, ca[1] * s) if ca is not None else _rodrigues(self._axis[i], c, s))
            Rs[..., i + 1, :, :], ps[..., i + 1, :] = R, p
        return Rs, ps

    def joint_poses(self, q):
        """World movable-joint frames (base placement applied)."""
        Rs, ps = self.joint_poses_local(q)
        return self.base_R @ Rs, ps @ self.base_R.T + self.base_xyz

    def tcp(self, q):
        """World TCP: (position (..., 3), rotation (..., 3, 3))."""
        Rs, ps = self.joint_poses(q)
        R = Rs[..., -1, :, :]
        return R @ self.T_tcp[:3, 3] + ps[..., -1, :], R @ self.T_tcp[:3, :3]

    def jacobian(self, q):
        """(..., 6, n) world-frame geometric Jacobian at the TCP point, rows [linear; angular]
        (pinocchio LOCAL_WORLD_ALIGNED convention)."""
        Rs, ps = self.joint_poses(q)
        p_tcp = Rs[..., -1, :, :] @ self.T_tcp[:3, 3] + ps[..., -1, :]
        J = np.zeros(np.shape(q)[:-1] + (6, self.n_dof))
        for i in range(self.n_dof):
            z = Rs[..., i + 1, :, :] @ self._axis[i]
            if self._prism[i]:
                J[..., :3, i] = z
            else:
                J[..., :3, i] = np.cross(z, p_tcp - ps[..., i + 1, :])
                J[..., 3:, i] = z
        return J

    def link_transforms(self, q) -> dict:
        """{link name: (..., 4, 4) world transform} for every link on the chain (fixed ones included)."""
        Rs, ps = self.joint_poses(q)
        out = {}
        for ln, (k, Tl) in self.fixed_links.items():
            T = np.zeros(Rs.shape[:-3] + (4, 4))
            R = Rs[..., k, :, :]
            T[..., :3, :3] = R @ Tl[:3, :3]
            T[..., :3, 3] = R @ Tl[:3, 3] + ps[..., k, :]
            T[..., 3, 3] = 1.0
            out[ln] = T
        return out

    # ------------------------------------------------------------------ limits / info
    def limits(self) -> dict:
        tl = lambda a: None if a is None else [float(x) for x in a]   # noqa: E731
        return dict(joint_names=self.joint_names, lower=tl(self.q_lo), upper=tl(self.q_hi), velocity=tl(self.v_max),
                    effort=tl(self.effort), acceleration=tl(self.a_max), acceleration_source=self.accel_source,
                    jerk=tl(self.j_max))

    def chain_lengths(self) -> list:
        """Distances between consecutive movable-joint origins at q = 0 (m)."""
        _, ps = self.joint_poses_local(np.zeros(self.n_dof))
        return [float(np.linalg.norm(ps[i + 1] - ps[i])) for i in range(self.n_dof)]

    def summary(self) -> dict:
        p, R = self.tcp(np.zeros(self.n_dof))
        return dict(urdf=self.path, robot_name=self.name, root_link=self.root_link, base_link=self.base_link,
                    tip_link=self.tip_link, n_dof=self.n_dof, joint_types=self.joint_types, limits=self.limits(),
                    tcp_offset=self.tcp_offset.tolist(), chain_lengths_m=self.chain_lengths(),
                    tcp_at_zero=p.tolist(), n_links=len(self.links),
                    collision_geoms={k: len(v.collisions) for k, v in self.links.items() if v.collisions})


# ----------------------------------------------------------------------------------------------- config checks
def compare_limits(urdf_limits: dict, config: dict, *, atol: float = 1e-6, rtol: float = 1e-6) -> list[str]:
    """Mismatches between URDF limits and hand-written config values (keys lower/upper/velocity, lists).

    Tolerance covers the 6-7 significant digits robots.yaml rounds to (e.g. 2.722714 vs 2.7227136)."""
    out = []
    for k in ("lower", "upper", "velocity"):
        c, u = config.get(k), urdf_limits.get(k)
        if c is None or u is None:
            continue
        c, u = np.asarray(c, float), np.asarray(u, float)
        if c.shape != u.shape:
            out.append(f"{k}: {len(c)} config values vs {len(u)} URDF joints")
            continue
        bad = ~np.isclose(c, u, atol=atol, rtol=rtol) & ~(np.isinf(u) & np.isinf(c))
        for i in np.flatnonzero(bad):
            out.append(f"{k}[{i}] {urdf_limits['joint_names'][i]}: config {c[i]:.9g} vs URDF {u[i]:.9g}")
    return out


def warn_mismatch(where: str, problems: list[str], resolution: str) -> None:
    if problems:
        warnings.warn(f"{where}: joint limits differ from the URDF ({resolution}): " + "; ".join(problems),
                      UserWarning, stacklevel=3)
