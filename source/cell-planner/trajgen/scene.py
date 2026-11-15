"""Loader and validator for the scene format of ``scenes/SCHEMA.md`` (scene.yaml + task.json).

    from trajgen.scene import load_scene, load_task, validate, load_original
    scene = load_scene("scenes/welding_cell_2_small")          # dir or path to scene.yaml
    task = load_task("scenes/welding_cell_2_small/task.json")  # umarell car_door_spot_welding format
    problems = validate(scene, task)                            # [] when consistent (raises on hard errors)
    orig = load_original("welding_cell_2_small")                # umarell's own files via env.cell.load_cell

A scene has 1..4 robots (more load, with a warning). Every robot carries its own model: URDF, SRDF, TCP (offset and
rotation), tool axis, tool collision spheres, disabled self-collision pairs, base position and full base orientation
(yaw, rpy or quaternion), home and joint limits. Scene-level ``collision.tool_spheres`` /
``collision.self_collision.disabled_pairs`` / ``collision.link_spheres`` are defaults for robots that do not override
them (``robots[i].collision``).

Bridges to the project code (yaw-only bases, robot ids robot1/robot2): ``scene.robot_cfg(rid)`` ->
``env.cell.RobotCfg`` and ``scene.to_cell(task)`` -> ``env.cell.Cell``. The generator itself does not use them.

Pure numpy + pyyaml; ``check_tool_coverage`` needs trimesh + scipy (both in ``.venv``).
"""
from __future__ import annotations

import re
import json
import logging
import os
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCHEMA_VERSION = 2
SUPPORTED_SCHEMA = (1, 2)
MAX_ROBOTS = 4
HULL_MIN_DEFAULT = 0.03      # collision.arm_arm.hull_min_distance_m default (robot-robot hull buffer, m)


def _num(v, ctx: str) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        raise SceneError(f"{ctx}: not numeric ({v!r})") from None


class SceneError(ValueError):
    """Hard schema error (missing key, wrong shape, missing file, unsupported value)."""


# ----------------------------------------------------------------------------------------------- data
@dataclass
class ToolSpec:
    tip_link: str
    tcp_offset: np.ndarray            # (3,) in the tip_link frame
    axis: np.ndarray                  # (3,) unit tool axis in the TCP frame (flange -> tip)
    tcp_rotation: str = "identity"    # "identity" or "custom" (tcp_R then holds the rotation)
    tcp_R: np.ndarray = field(default_factory=lambda: np.eye(3))   # TCP frame axes in the tip_link frame
    electrode_hulls: tuple | None = None   # collision-mesh names exempt from the workpiece box-hull rule (None = the
                                           # tip-link hull containing the TCP; trajgen.mjscene.electrode_hull_names)


@dataclass
class ToolSpheres:
    frame: str                        # link the centres are expressed in
    centers: np.ndarray               # (n, 3)
    radii: np.ndarray                 # (n,)
    min_vertex_fraction: float = 0.99


@dataclass
class RobotCollision:
    """Per-robot collision model (defaults from the scene-level ``collision`` block)."""
    link_radius: float
    fill_step: float
    tcp_radius: float
    tool_spheres: ToolSpheres
    disabled_pairs: list
    inherit_pairs: bool = False       # True: the scene-level collision.self_collision.disabled_pairs apply


@dataclass
class RobotSpec:
    id: str
    model: str
    urdf: Path
    srdf: Path | None
    base_xyz: np.ndarray              # (3,)
    yaw: float                        # yaw part of the base orientation (exact when the base is yaw-only)
    home: np.ndarray                  # (6,) rad
    joint_names: list[str]
    q_lo: np.ndarray
    q_hi: np.ndarray
    v_max: np.ndarray                 # rad/s
    a_max: np.ndarray                 # rad/s^2 (before the planning acceleration rule)
    tool: ToolSpec
    base_R: np.ndarray = field(default_factory=lambda: np.eye(3))   # world rotation of the URDF root
    coll: RobotCollision | None = None
    umarell_id: str | None = None     # id of this robot in the declared umarell cell (parity model)
    limits_info: dict = field(default_factory=dict)   # where each joint-limit vector came from (URDF / scene.yaml)

    @property
    def yaw_only(self) -> bool:
        return bool(np.allclose(self.base_R, rot_z(self.yaw), atol=1e-12))


@dataclass
class CollisionSpec:
    link_radius: float                # umarell robot_link_radius (default for every robot)
    fill_step: float                  # link-fill sphere spacing
    tcp_radius: float                 # umarell tool_link_radius (welding_tcp frame sphere)
    tool_spheres: ToolSpheres         # default tool spheres (robots[0]'s when every robot overrides)
    pair_radius: float                # radius of every sphere in the arm-vs-arm check
    arm_arm_gate: float               # clearance must stay > this
    hull_backstop: bool
    self_geometry: str
    self_min_distance: float
    disabled_pairs: list[tuple[str, str]]
    hull_min_distance: float = HULL_MIN_DEFAULT   # robot-robot hull distance must stay >= this (hull_backstop)


@dataclass
class Box:
    id: str
    center: np.ndarray
    half_extents: np.ndarray
    margin: float
    is_workpiece: bool = False
    R: np.ndarray | None = None       # box axes in the world (None = axis-aligned)


@dataclass
class FloorSpec:
    z: float
    margin: float
    sphere_rule: bool
    hull_rule: bool
    enabled: bool = True


@dataclass
class ElectrodeRule:
    """Electrode off-weld rule (docs/n2_contract.md 3.1, scene key ``obstacles.box_hull.electrode_off_weld``): while
    the TCP is farther than ``radius`` from every weld of the task, the signed distance between the electrode hull(s)
    (``tool.electrode_hulls``, default the tip-link hull containing the TCP) and every workpiece box must be >=
    ``min_distance`` (planners); the verifier's ``electrode_off_weld`` check fails where that distance is more than
    ``score_tol`` below ``min_distance``. Default off: scenes without the key are unchanged."""
    enabled: bool = False
    radius: float = 0.05              # m, weld neighbourhood (TCP distance to the nearest weld)
    min_distance: float = 0.0         # m, planners: electrode hull - workpiece box signed distance >= this elsewhere
    score_tol: float = 0.005          # m, verifier: fails at a penetration (min_distance - distance) > this
    # round 3 no-dip rule (trajgen.nodip, docs/r3_nodip.md; ``obstacles.box_hull.no_dip``): with ``nodip`` the
    # exemption inside ``radius`` (= the contact ball around the weld TCP) holds only for contacts within
    # ``contact_point_radius`` of the weld point and at most ``contact_depth`` deep (verifier: only in the robot's
    # contact window of that weld); the defaults keep the round-2 rule unchanged
    nodip: bool = False
    contact_depth: float = 0.0        # m, max penetration inside the exemption (nodip)
    contact_point_radius: float = 0.0  # m, contact position within this of the weld point (nodip)
    version: int = 0                  # trajgen.nodip.NODIP_VERSION when nodip, else 0


@dataclass
class BoxHullRule:
    """URDF collision hulls vs the obstacle boxes (MuJoCo), on top of the sphere box rule."""
    enabled: bool = True
    min_distance: float = 0.0         # hull-box signed distance must stay >= this
    workpiece: bool = False           # also check workpiece boxes (every hull except the electrode hull(s); NR 1.5)
    electrode: ElectrodeRule = field(default_factory=ElectrodeRule)   # electrode off-weld rule (round 2, default off)


@dataclass
class WorkpieceRule:
    rule: str                         # "wedge" (docs/workpiece_decision.md) or "none"
    radius: float
    tol: float
    points: str = "tcp"


@dataclass
class PlanningSpec:
    velocity_scale: float
    acceleration_rule: str            # "vscale_squared" | "vscale" | "none"
    approach_velocity_fraction: float
    home_velocity_scale: float
    approach_offset: float
    retract_offset: float
    min_offset: float
    liftoff_required: bool
    liftoff_straight: bool
    liftoff_tube: float
    descent_straight: bool
    descent_tube: float
    dwell_s: float
    executor_margin_s: float
    sample_dt: float
    check_dt: float
    max_jerk_factor: float | None = None

    def accel_scale(self, vscale: float | None = None) -> float:
        """Fraction of a_max allowed at velocity scale ``vscale`` (default: transit)."""
        v = self.velocity_scale if vscale is None else float(vscale)
        return {"vscale_squared": v * v, "vscale": v, "none": 1.0}[self.acceleration_rule]

    @property
    def approach_vscale(self) -> float:
        return self.velocity_scale * self.approach_velocity_fraction


@dataclass
class Tolerances:
    position: float
    tool_axis_deg: float
    entry_direction_deg: float
    entry_span: float
    spin_free: bool = True
    spin_tol_deg: float = 5.0         # used only when spin_free is false


@dataclass
class Scene:
    cell_id: str
    path: Path | None                 # scene.yaml (None for load_original)
    robots: list[RobotSpec]
    collision: CollisionSpec
    floor: FloorSpec
    boxes: list[Box]
    workpiece: WorkpieceRule
    planning: PlanningSpec
    tolerances: Tolerances
    description: str = ""
    raw: dict = field(default_factory=dict, repr=False)
    origin: str = "scene"             # "scene" | "umarell_original"
    umarell_cell: str | None = None   # umarell cell this scene declares parity with (None = no parity model)
    umarell_parity: bool = False      # default of the generator's --umarell-parity option
    box_hull: BoxHullRule = field(default_factory=BoxHullRule)
    schema_version: int = SCHEMA_VERSION
    notes: list = field(default_factory=list)   # loader notes (v1 migration etc.)

    def robot(self, rid: str) -> RobotSpec:
        for r in self.robots:
            if r.id == rid:
                return r
        raise KeyError(rid)

    @property
    def robot_ids(self) -> tuple[str, ...]:
        return tuple(r.id for r in self.robots)

    def disabled_pairs_of(self, rid: str) -> list:
        """The robot's disabled self-collision pairs (its own, or the scene-level list it inherits)."""
        r = self.robot(rid)
        return list(self.collision.disabled_pairs) if r.coll.inherit_pairs else list(r.coll.disabled_pairs)

    @property
    def tool_axis_local(self) -> np.ndarray:
        """The TCP-frame tool axis shared by every robot (validate() enforces that they agree)."""
        return self.robots[0].tool.axis if self.robots else np.array([-1.0, 0.0, 0.0])

    def robot_cfg(self, rid: str):
        """``env.cell.RobotCfg`` view of a scene robot (tool_collision = {}; yaw-only bases)."""
        from env.cell import RobotCfg

        r = self.robot(rid)
        if not r.yaw_only:
            raise SceneError(f"{rid}: env.cell.RobotCfg supports yaw-only bases")
        return RobotCfg(rid=r.id, base=r.base_xyz.copy(), yaw=r.yaw, home=r.home.copy(), q_lo=r.q_lo.copy(),
                        q_hi=r.q_hi.copy(), v_max=r.v_max.copy(), a_max=r.a_max.copy(),
                        tcp_offset=r.tool.tcp_offset.copy(), tool_collision={}, urdf=r.urdf)

    def to_cell(self, task: "Task"):
        """``env.cell.Cell`` view (for env/expert code that takes a Cell). Obstacles are re-emitted in
        umarell's obstacles.yaml form (floor plane + boxes). Not used by the generator."""
        from env.cell import Cell, Weld as CWeld

        obst = [{"id": "floor", "type": "plane", "normal": [0.0, 0.0, 1.0], "offset": -self.floor.z,
                 "collision_margin": self.floor.margin}]
        obst += [{"id": b.id, "type": "box", "center": b.center.tolist(), "half_extents": b.half_extents.tolist(),
                  "collision_margin": b.margin, "is_workpiece": b.is_workpiece} for b in self.boxes]
        welds = [CWeld(w.id, w.position.copy(), w.weld_quat.copy(), w.approach.copy(), w.group, w.priority)
                 for w in task.welds]
        collision = {"warning_distance": self.collision.arm_arm_gate}
        clearance = {"default_collision_margin": 0.05, "robot_link_radius": self.collision.link_radius,
                     "tool_link_radius": self.collision.tcp_radius,
                     "approach_velocity_fraction": self.planning.approach_velocity_fraction}
        planning = {"max_velocity_scaling": self.planning.velocity_scale,
                    "home_velocity_scale": self.planning.home_velocity_scale}
        root = self.robots[0].urdf.parents[2] if self.robots else Path(".")
        return Cell(self.cell_id, root, {r.id: self.robot_cfg(r.id) for r in self.robots}, welds, obst,
                    collision, clearance, planning, task.raw)


@dataclass
class WeldRuleW:
    enabled: bool
    radius: float
    tol: float


@dataclass
class Weld:
    id: str
    position: np.ndarray              # (3,) world
    approach: np.ndarray              # (3,) UNIT direction of tool travel INTO the weld (descent direction)
    weld_quat: np.ndarray             # (4,) xyzw, TCP frame orientation at the weld (normalised)
    transit_quat: np.ndarray | None   # (4,) xyzw `orientation` (umarell: orientation during transit)
    group: str
    priority: int
    placeholder: bool = False         # position is a "replace with CAD value" placeholder
    original_position: np.ndarray | None = None
    note: str = ""
    allowed_robots: list | None = None    # None = any robot of the scene
    rule_w: dict | None = None            # per-weld rule W override {"enabled", "radius_m", "tol_m"} (None = scene)
    axis_local: np.ndarray = field(default_factory=lambda: np.array([-1.0, 0.0, 0.0]))  # set by bind_task

    @property
    def R(self) -> np.ndarray:
        return quat_to_R(self.weld_quat)

    @property
    def tool_axis(self) -> np.ndarray:
        """Required tool axis in the world: R(weld_orientation) @ tool.axis (TCP frame -x for the stick tool)."""
        return self.R @ self.axis_local

    def pre_weld(self, offset: float) -> np.ndarray:
        """Standoff point ``offset`` back along the approach line: p - a * offset."""
        return self.position - self.approach * float(offset)

    def rule_w_for(self, scene: Scene) -> WeldRuleW:
        wp = scene.workpiece
        on = wp.rule == "wedge"
        rad, tol = float(wp.radius), float(wp.tol)
        o = self.rule_w
        if o is not None:
            on = bool(o.get("enabled", True))
            rad = float(o.get("radius_m", rad))
            tol = float(o.get("tol_m", tol))
        return WeldRuleW(on, rad, tol)


@dataclass
class Task:
    program: str
    cell_id: str
    welds: list[Weld]
    sequence_groups_order: list[str]
    path: Path | None = None
    raw: dict = field(default_factory=dict, repr=False)

    def weld(self, wid: str) -> Weld:
        for w in self.welds:
            if w.id == wid:
                return w
        raise KeyError(wid)


# ----------------------------------------------------------------------------------------------- utils
def quat_to_R(q_xyzw) -> np.ndarray:
    x, y, z, w = np.asarray(q_xyzw, float) / np.linalg.norm(q_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R) -> np.ndarray:
    """Rotation matrix -> xyzw quaternion (w >= 0)."""
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        q = np.array([(R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s, 0.25 / s])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = np.zeros(4)
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s
    q = q / np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def rot_z(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    """URDF rpy -> R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return Rz @ Ry @ Rx


def frame_from_axis(x_axis, hint=(0.0, 0.0, 1.0)) -> np.ndarray:
    """A right-handed rotation whose first column is ``x_axis`` (unit), second column orthogonal to it and as close
    as possible to ``hint`` x x (deterministic)."""
    x = np.asarray(x_axis, float)
    x = x / np.linalg.norm(x)
    h = np.asarray(hint, float)
    if abs(x @ h) > 0.95:
        h = np.array([1.0, 0.0, 0.0]) if abs(x[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    y = np.cross(h, x)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    return np.column_stack([x, y, z])


def _arr(d, key, n=None, ctx=""):
    if not isinstance(d, dict) or key not in d:
        raise SceneError(f"{ctx}: missing key {key!r}")
    try:
        a = np.asarray(d[key], float)
    except (TypeError, ValueError) as e:
        raise SceneError(f"{ctx}.{key}: not numeric ({e})") from None
    if n is not None and a.shape != (n,):
        raise SceneError(f"{ctx}.{key}: expected {n} numbers, got shape {a.shape}")
    return a


_LIMIT_LOG = logging.getLogger("trajgen.scene")
LIMITS_POLICY_ENV = "TRAJGEN_SCENE_LIMITS"     # "scene": legacy, a differing joints.<key> overrides the URDF (warning)


def _joint_limits(j: dict, urdf: Path | None, tip: str, model: str, ctx: str, notes: list) -> tuple[dict, dict]:
    """Joint names / lower / upper / velocity / acceleration of one robot (docs/p3_urdf.md).

    The URDF is the source of truth: a key omitted in ``joints`` is read from the URDF (<limit lower upper velocity>,
    the movable joint names on the chain to ``tip``). A key given in scene.yaml must match the URDF (within the 6-7
    digit rounding of hand-copied values); if it does not, a SceneError is raised (since 2026-10-12: the URDF is the
    source of the limits, a stale hand copy must not silently replace it; ``TRAJGEN_SCENE_LIMITS=scene`` restores the
    old behaviour: the scene value overrides the URDF, with a logged warning and a scene note). A deliberately
    tighter limit is ``joints.velocity_cap`` / ``lower_cap`` / ``upper_cap`` ([6], per joint the tighter of the cap
    and the URDF value: a cap never loosens the URDF). ``acceleration`` (not in URDFs) defaults to robotmodel's per-model default (KR240
    R2900: umarell's REVIEW default [3, 3, 3, 6, 6, 8]), with a note."""
    if not isinstance(j, dict):
        raise SceneError(f"{ctx}.joints: expected a mapping")
    rm, err = None, None
    if urdf is not None and Path(urdf).exists():
        try:
            from robotmodel.urdf_robot import URDFRobot
            rm = URDFRobot(urdf, tip_link=tip, model=model)
        except Exception as e:  # noqa: BLE001  (scene validation reports the URDF problem separately)
            err = f"{type(e).__name__}: {e}"
    out, info = {}, {}
    if "names" in j:
        out["names"] = list(j["names"])
        if rm is not None and out["names"] != rm.joint_names:
            notes.append(f"{ctx}.joints.names {out['names']} differ from the URDF chain {rm.joint_names}")
    elif rm is not None:
        out["names"] = list(rm.joint_names)
    else:
        raise SceneError(f"{ctx}.joints: missing key 'names' and the URDF is unreadable ({err or 'not found'})")
    info["names"] = "scene.yaml" if "names" in j else "urdf"
    from robotmodel.urdf_robot import compare_limits
    legacy = os.environ.get(LIMITS_POLICY_ENV, "").strip().lower() == "scene"
    for key, attr in (("lower", "q_lo"), ("upper", "q_hi"), ("velocity", "v_max")):
        if key in j:
            v = _arr(j, key, 6, ctx + ".joints")
            info[key] = "scene.yaml"
            if rm is not None:
                probs = compare_limits({"joint_names": rm.joint_names, key: getattr(rm, attr)}, {key: v})
                if probs and not legacy:
                    raise SceneError(
                        f"{ctx}.joints.{key} differs from the URDF ({Path(urdf).name}): " + "; ".join(probs) +
                        f". The URDF is the source of the joint limits: remove joints.{key} to follow the URDF, or "
                        f"give joints.{key}_cap for a deliberately tighter limit ({LIMITS_POLICY_ENV}=scene restores "
                        "the old 'scene value overrides the URDF' behaviour)")
                if probs:
                    msg = f"{ctx}.joints.{key} overrides the URDF ({Path(urdf).name}): " + "; ".join(probs)
                    notes.append(msg)
                    _LIMIT_LOG.warning(msg)
                    info[key] = "scene.yaml (overrides URDF)"
                else:
                    info[key] = "scene.yaml (matches URDF)"
        elif rm is not None:
            v = np.asarray(getattr(rm, attr), float).copy()
            if v.shape != (6,) or not np.all(np.isfinite(v)):
                raise SceneError(f"{ctx}.joints.{key}: not given and the URDF has no finite <limit {key}> on all 6 "
                                 f"joints ({v.tolist()})")
            info[key] = "urdf"
        else:
            raise SceneError(f"{ctx}.joints: missing key {key!r} and the URDF is unreadable ({err or 'not found'})")
        cap_key = f"{key}_cap"
        if cap_key in j:                    # a deliberate, tighter limit than the URDF's (never a looser one)
            c = _arr(j, cap_key, 6, ctx + ".joints")
            binds = (c > v) if key == "lower" else (c < v)
            v = np.maximum(v, c) if key == "lower" else np.minimum(v, c)
            info[key] = f"{info[key]}, capped by scene.yaml joints.{cap_key} on joints {np.flatnonzero(binds).tolist()}"
            notes.append(f"{ctx}.joints.{key}: {info[key]}: {np.round(v, 7).tolist()}")
        out[key] = v
    if "acceleration" in j:
        out["acceleration"] = _arr(j, "acceleration", 6, ctx + ".joints")
        info["acceleration"] = "scene.yaml"
    else:
        from robotmodel.urdf_robot import accel_default
        d = (rm.a_max, rm.accel_source) if rm is not None and rm.a_max is not None else None
        if d is None:
            dd = accel_default(model, Path(urdf).stem if urdf is not None else "")
            d = (np.asarray(dd["a_max"], float), dd["source"]) if dd else None
        if d is None or len(d[0]) != 6:
            raise SceneError(f"{ctx}.joints: missing key 'acceleration' (URDFs carry no acceleration limits and there "
                             f"is no default for model {model!r})")
        out["acceleration"] = np.asarray(d[0], float).copy()
        info["acceleration"] = d[1]
        notes.append(f"{ctx}.joints.acceleration not given: {out['acceleration'].tolist()} rad/s^2 from {d[1]}")
    return out, info


def _req(d, key, ctx=""):
    if not isinstance(d, dict) or key not in d:
        raise SceneError(f"{ctx}: missing key {key!r}")
    return d[key]


def _resolve(p: str | None, scene_dir: Path, cells_root: Path) -> Path | None:
    if p is None:
        return None
    p = str(p)
    if p.startswith("cells:"):
        return (cells_root / p[len("cells:"):]).resolve()
    q = Path(p)
    return q if q.is_absolute() else (scene_dir / q).resolve()


def _quat(d) -> np.ndarray:
    if isinstance(d, dict):
        q = np.array([d["x"], d["y"], d["z"], d["w"]], float)
    else:
        q = np.asarray(d, float)
    return q


def _rotation(d: dict, ctx: str) -> tuple[np.ndarray, float]:
    """Rotation of a ``base`` / box block: ``yaw`` (rad), ``yaw_deg``, ``rpy`` [r, p, y] (rad, URDF convention),
    ``rpy_deg`` or ``quat`` [x, y, z, w]. At most one may be given. Returns (R, yaw)."""
    keys = [k for k in ("yaw", "yaw_deg", "rpy", "rpy_deg", "quat") if k in d]
    if len(keys) > 1:
        raise SceneError(f"{ctx}: give only one of yaw / yaw_deg / rpy / rpy_deg / quat, got {keys}")
    if not keys:
        return np.eye(3), 0.0
    k = keys[0]
    if k in ("yaw", "yaw_deg"):
        y = float(d[k]) if k == "yaw" else np.deg2rad(float(d[k]))
        return rot_z(y), y
    if k in ("rpy", "rpy_deg"):
        v = _arr(d, k, 3, ctx)
        v = v if k == "rpy" else np.deg2rad(v)
        R = rpy_matrix(*v)
    else:
        q = _arr(d, "quat", 4, ctx)
        if abs(np.linalg.norm(q) - 1) > 1e-3:
            raise SceneError(f"{ctx}.quat: norm {np.linalg.norm(q):.5f} is not 1")
        R = quat_to_R(q)
    return R, float(np.arctan2(R[1, 0], R[0, 0]))


def _tool_spheres(ts: dict, ctx: str) -> ToolSpheres:
    if not isinstance(ts, dict):
        raise SceneError(f"{ctx}: expected a mapping")
    S = np.asarray(_req(ts, "spheres", ctx), float)
    if S.size == 0 or S.ndim != 2 or S.shape[1] != 4:
        raise SceneError(f"{ctx}.spheres: expected a list of [x, y, z, r]")
    cov = ts.get("coverage_requirement", {}) or {}
    return ToolSpheres(str(_req(ts, "frame", ctx)), S[:, :3].copy(), S[:, 3].copy(),
                       float(cov.get("min_vertex_fraction", 0.99)))


def _tcp_rotation(t: dict, ctx: str) -> tuple[str, np.ndarray]:
    v = t.get("tcp_rotation", "identity")
    if v in (None, "identity"):
        return "identity", np.eye(3)
    if isinstance(v, dict):
        R, _ = _rotation(v, ctx + ".tcp_rotation")
        return "custom", R
    raise SceneError(f"{ctx}.tcp_rotation: 'identity' or a mapping {{rpy|rpy_deg|quat: ...}}, got {v!r}")


# ----------------------------------------------------------------------------------------------- scene
def _electrode_hulls(t: dict, ctx: str):
    """``tool.electrode_hulls``: optional list of collision-mesh names (file stems, e.g. World_hull_0)."""
    v = t.get("electrode_hulls")
    if v is None:
        return None
    if isinstance(v, str):
        v = [v]
    if not isinstance(v, (list, tuple)) or not all(isinstance(x, str) and x for x in v):
        raise SceneError(f"{ctx}.electrode_hulls: expected a list of collision-mesh names")
    return tuple(v)


def _electrode_rule(e) -> ElectrodeRule:
    """``obstacles.box_hull.electrode_off_weld``: {enabled (default true when the key is given), radius_m,
    min_distance_m, score_tol_m}; absent / null = off."""
    if e is None or e is False:
        return ElectrodeRule()
    if e is True:
        e = {"enabled": True}
    if not isinstance(e, dict):
        raise SceneError("obstacles.box_hull.electrode_off_weld: expected a mapping {enabled, radius_m, "
                         "min_distance_m, score_tol_m}")
    unknown = sorted(set(e) - {"enabled", "radius_m", "min_distance_m", "score_tol_m"})
    if unknown:
        raise SceneError(f"obstacles.box_hull.electrode_off_weld: unknown keys {unknown}")
    d = ElectrodeRule()
    r = ElectrodeRule(bool(e.get("enabled", True)),
                      _num(e.get("radius_m", d.radius), "obstacles.box_hull.electrode_off_weld.radius_m"),
                      _num(e.get("min_distance_m", d.min_distance), "obstacles.box_hull.electrode_off_weld.min_distance_m"),
                      _num(e.get("score_tol_m", d.score_tol), "obstacles.box_hull.electrode_off_weld.score_tol_m"))
    if r.radius < 0 or r.score_tol < 0:
        raise SceneError("obstacles.box_hull.electrode_off_weld: radius_m and score_tol_m must be >= 0")
    return r


def nodip_rule(spec=None) -> ElectrodeRule:
    """The ElectrodeRule of the no-dip rule (trajgen.nodip): ``spec`` None / True = the version's constants, or a
    mapping {version, contact_tcp_radius_m, contact_point_radius_m, contact_depth_m, min_distance_m} overriding
    them (scorer threshold: any penetration, i.e. score_tol = min_distance)."""
    from trajgen import nodip as N
    e = {} if spec in (None, True) else dict(spec)
    unknown = sorted(set(e) - {"enabled", "version", "contact_tcp_radius_m", "contact_point_radius_m",
                               "contact_depth_m", "min_distance_m"})
    if unknown:
        raise SceneError(f"obstacles.box_hull.no_dip: unknown keys {unknown}")
    ver = int(e.get("version", N.NODIP_VERSION))
    if ver != N.NODIP_VERSION:
        raise SceneError(f"obstacles.box_hull.no_dip: version {ver} not supported (this code: {N.NODIP_VERSION})")
    md = _num(e.get("min_distance_m", N.PLAN_MIN_DISTANCE_M), "obstacles.box_hull.no_dip.min_distance_m")
    r = ElectrodeRule(True, _num(e.get("contact_tcp_radius_m", N.CONTACT_TCP_RADIUS_M), "no_dip.contact_tcp_radius_m"),
                      md, md + N.SCORE_TOL_M, True,
                      _num(e.get("contact_depth_m", N.CONTACT_DEPTH_M), "no_dip.contact_depth_m"),
                      _num(e.get("contact_point_radius_m", N.CONTACT_POINT_RADIUS_M), "no_dip.contact_point_radius_m"),
                      ver)
    if min(r.radius, r.contact_depth, r.contact_point_radius, r.min_distance) < 0:
        raise SceneError("obstacles.box_hull.no_dip: radii, depth and min_distance_m must be >= 0")
    return r


def _nodip_spec(v):
    """``obstacles.box_hull.no_dip`` -> (on, spec): absent = trajgen.nodip.default_on()."""
    if v is None:
        from trajgen.nodip import default_on
        return default_on(), None
    if isinstance(v, bool):
        return v, None
    if isinstance(v, dict):
        return bool(v.get("enabled", True)), v
    raise SceneError("obstacles.box_hull.no_dip: expected true / false or a mapping {enabled, version, "
                     "contact_tcp_radius_m, contact_point_radius_m, contact_depth_m, min_distance_m}")


def apply_nodip(box_hull: "BoxHullRule", boxes, on: bool, spec=None) -> "BoxHullRule":
    """BoxHullRule with the no-dip rule applied (``on`` and at least one workpiece box): workpiece boxes become hull
    geoms for every hull except the electrode, and the electrode rule is trajgen.nodip's. Unchanged otherwise."""
    from dataclasses import replace
    if not on or not box_hull.enabled or not any(b.is_workpiece for b in boxes):
        return box_hull
    return replace(box_hull, workpiece=True, electrode=nodip_rule(spec))


def with_nodip(scene: "Scene", on: bool = True, **spec) -> "Scene":
    """Copy of ``scene`` with the no-dip rule switched on / off (``spec``: no_dip mapping keys, e.g.
    contact_depth_m=0.002), also written into ``scene.raw`` (caches keyed on the scene content see it). Off restores
    the scene's own declared box-hull rule (round 2: electrode_off_weld as declared)."""
    import copy
    s = copy.copy(scene)
    raw = copy.deepcopy(scene.raw)
    bh_raw = raw.setdefault("obstacles", {}).setdefault("box_hull", {})
    base = BoxHullRule(bool(bh_raw.get("enabled", True)), float(bh_raw.get("min_distance_m", 0.0)),
                       bool(bh_raw.get("workpiece", False)), _electrode_rule(bh_raw.get("electrode_off_weld"))) \
        if scene.raw else getattr(scene, "_box_hull_declared", scene.box_hull)
    bh_raw["no_dip"] = ({"enabled": True, **spec} if spec else True) if on else False
    s.box_hull = apply_nodip(base, scene.boxes, bool(on), spec or None)
    s._box_hull_declared = base
    s.raw = raw
    return s


def nodip_on(scene: "Scene") -> bool:
    """True when the scene's box-hull rule is the no-dip rule (trajgen.nodip)."""
    return bool(scene.box_hull.enabled and scene.box_hull.electrode.enabled and scene.box_hull.electrode.nodip)


def with_electrode_rule(scene: "Scene", enabled: bool = True, **kw) -> "Scene":
    """Copy of ``scene`` with the electrode off-weld rule set (``ElectrodeRule`` fields as keywords: radius,
    min_distance, score_tol), also written into ``scene.raw`` so that caches keyed on the scene content see it.
    For tests and regression runs on scenes that do not declare the rule (v1 scenes)."""
    import copy
    from dataclasses import replace
    rule = replace(scene.box_hull.electrode, enabled=bool(enabled), **kw)
    s = copy.copy(scene)
    s.box_hull = replace(scene.box_hull, electrode=rule)
    raw = copy.deepcopy(scene.raw)
    raw.setdefault("obstacles", {}).setdefault("box_hull", {})["electrode_off_weld"] = {
        "enabled": rule.enabled, "radius_m": rule.radius, "min_distance_m": rule.min_distance,
        "score_tol_m": rule.score_tol}
    s.raw = raw
    return s


def load_scene(path: str | Path, cells_root: str | Path | None = None) -> Scene:
    """Load scene.yaml (``path`` may be the scene directory). Raises SceneError on schema errors."""
    path = Path(path)
    if path.is_dir():
        path = path / "scene.yaml"
    if not path.exists():
        raise SceneError(f"scene file not found: {path}")
    try:
        raw = yaml.safe_load(open(path))
    except yaml.YAMLError as e:
        raise SceneError(f"{path}: invalid YAML: {e}") from None
    if not isinstance(raw, dict):
        raise SceneError(f"{path}: not a mapping")
    sdir = path.parent.resolve()
    ver = int(raw.get("schema_version", -1))
    if ver not in SUPPORTED_SCHEMA:
        raise SceneError(f"{path}: schema_version must be one of {SUPPORTED_SCHEMA}, got {raw.get('schema_version')}")
    notes = []
    if cells_root is None:
        cells_root = os.environ.get("UMARELL_CELLS") or (raw.get("assets", {}) or {}).get("cells_root")
    cr = Path(cells_root) if cells_root else sdir
    cr = cr if cr.is_absolute() else (sdir / cr).resolve()

    c = _req(raw, "collision", "scene")
    ls, aa = (_req(c, k, "collision") for k in ("link_spheres", "arm_arm"))
    sc = c.get("self_collision", {}) or {}
    def_ts = _tool_spheres(c["tool_spheres"], "collision.tool_spheres") if c.get("tool_spheres") else None
    def_pairs = [tuple(p) for p in sc.get("disabled_pairs", []) or []]
    for p in def_pairs:
        if len(p) != 2:
            raise SceneError(f"collision.self_collision.disabled_pairs: {p} is not a pair")

    robots = []
    rlist = _req(raw, "robots", "scene")
    if not isinstance(rlist, list) or not rlist:
        raise SceneError("scene.robots: expected a non-empty list")
    for i, r in enumerate(rlist):
        ctx = f"robots[{i}]"
        j = r.get("joints") or {}           # optional since P3-URDF: every limit can come from the URDF
        t = _req(r, "tool", ctx)
        b = _req(r, "base", ctx)
        if "home_deg" in r:
            home = np.deg2rad(_arr(r, "home_deg", 6, ctx))
        else:
            home = _arr(r, "home", 6, ctx)
        ax = _arr(t, "axis", 3, ctx + ".tool")
        if np.linalg.norm(ax) < 1e-9:
            raise SceneError(f"{ctx}.tool.axis: zero vector")
        R_b, yaw = _rotation(b, ctx + ".base")
        trot, tR = _tcp_rotation(t, ctx + ".tool")
        rc = r.get("collision", {}) or {}
        rls = rc.get("link_spheres", {}) or {}
        ts = _tool_spheres(rc["tool_spheres"], ctx + ".collision.tool_spheres") if rc.get("tool_spheres") else def_ts
        if ts is None:
            raise SceneError(f"{ctx}: no tool_spheres (neither collision.tool_spheres nor {ctx}.collision.tool_spheres)")
        pairs = [tuple(p) for p in rc["disabled_pairs"]] if "disabled_pairs" in rc else list(def_pairs)
        coll_r = RobotCollision(float(rls.get("radius", ls["radius"])), float(rls.get("fill_step", ls["fill_step"])),
                                float(rls.get("tcp_radius", ls["tcp_radius"])), ts, pairs,
                                inherit_pairs="disabled_pairs" not in rc)
        urdf = _resolve(_req(r, "urdf", ctx), sdir, cr)
        tip = str(_req(t, "tip_link", ctx + ".tool"))
        jl, linfo = _joint_limits(j, urdf, tip, str(r.get("model", "")), ctx, notes)
        robots.append(RobotSpec(
            id=str(_req(r, "id", ctx)), model=str(r.get("model", "")),
            urdf=urdf, srdf=_resolve(r.get("srdf"), sdir, cr),
            base_xyz=_arr(b, "xyz", 3, ctx + ".base"), yaw=float(yaw), home=home,
            joint_names=jl["names"], q_lo=jl["lower"], q_hi=jl["upper"], v_max=jl["velocity"], a_max=jl["acceleration"],
            limits_info=linfo,
            tool=ToolSpec(tip_link=tip, tcp_offset=_arr(t, "tcp_offset", 3, ctx + ".tool"),
                          axis=ax / np.linalg.norm(ax), tcp_rotation=trot, tcp_R=tR,
                          electrode_hulls=_electrode_hulls(t, ctx + ".tool")),
            base_R=R_b, coll=coll_r, umarell_id=r.get("umarell_id")))

    r0 = robots[0].coll
    coll = CollisionSpec(
        link_radius=float(ls["radius"]), fill_step=float(ls["fill_step"]), tcp_radius=float(ls["tcp_radius"]),
        tool_spheres=def_ts if def_ts is not None else r0.tool_spheres,
        pair_radius=float(aa["pair_radius"]), arm_arm_gate=float(aa["gate_m"]), hull_backstop=bool(aa.get("hull_backstop", True)),
        hull_min_distance=_num(aa.get("hull_min_distance_m", HULL_MIN_DEFAULT), "collision.arm_arm.hull_min_distance_m"),
        self_geometry=str(sc.get("geometry", "urdf_collision_hulls")), self_min_distance=float(sc.get("min_distance_m", 0.0)),
        disabled_pairs=def_pairs if def_pairs or def_ts is not None else list(r0.disabled_pairs))

    o = raw.get("obstacles", {}) or {}
    f = o.get("floor")
    if f is None or f is False:
        floor = FloorSpec(-np.inf, 0.0, False, False, enabled=False)
    else:
        floor = FloorSpec(float(f.get("z", 0.0)), float(f.get("margin_m", 0.0)), bool(f.get("sphere_rule", True)),
                          bool(f.get("hull_rule", True)))
    dm = float(o.get("default_box_margin", 0.05))
    boxes = []
    for k, bx in enumerate(o.get("boxes", []) or []):
        ctx = f"obstacles.boxes[{k}]"
        R_box, _ = _rotation(bx, ctx)
        boxes.append(Box(str(bx.get("id", f"box{k}")), _arr(bx, "center", 3, ctx), _arr(bx, "half_extents", 3, ctx),
                         float(bx.get("collision_margin", dm)), bool(bx.get("is_workpiece", False)),
                         None if np.allclose(R_box, np.eye(3)) else R_box))
    bh = o.get("box_hull", {}) or {}
    box_hull = BoxHullRule(bool(bh.get("enabled", True)), float(bh.get("min_distance_m", 0.0)),
                           bool(bh.get("workpiece", False)), _electrode_rule(bh.get("electrode_off_weld")))
    box_hull_declared = box_hull
    nd_on, nd_spec = _nodip_spec(bh.get("no_dip"))
    box_hull = apply_nodip(box_hull, boxes, nd_on, nd_spec)          # round 3 no-dip rule (trajgen.nodip)

    w = raw.get("workpiece", {"rule": "none"}) or {"rule": "none"}
    wp = WorkpieceRule(str(w.get("rule", "none")), float(w.get("radius_m", 0.0)), float(w.get("tol_m", 0.0)), str(w.get("points", "tcp")))

    p = _req(raw, "planning", "scene")
    lo, de = p.get("liftoff", {}) or {}, p.get("descent", {}) or {}
    try:
        plan = PlanningSpec(
            velocity_scale=float(p["velocity_scale"]), acceleration_rule=str(p.get("acceleration_rule", "vscale_squared")),
            approach_velocity_fraction=float(p["approach_velocity_fraction"]), home_velocity_scale=float(p["home_velocity_scale"]),
            approach_offset=float(p["approach_offset_m"]), retract_offset=float(p["retract_offset_m"]),
            min_offset=float(p.get("min_offset_m", p["approach_offset_m"])),
            liftoff_required=bool(lo.get("required", True)), liftoff_straight=bool(lo.get("straight_on_axis", True)),
            liftoff_tube=float(lo.get("tube_m", 0.002)), descent_straight=bool(de.get("straight_on_axis", True)),
            descent_tube=float(de.get("tube_m", 0.002)), dwell_s=float(p["dwell_s"]),
            executor_margin_s=float(p.get("executor_margin_s", 1.0)), sample_dt=float(p["sample_dt"]),
            check_dt=float(p.get("check_dt", p["sample_dt"])),
            max_jerk_factor=None if p.get("max_jerk_factor") is None else float(p["max_jerk_factor"]))
    except KeyError as e:
        raise SceneError(f"planning: missing key {e}") from None
    t = _req(raw, "tolerances", "scene")
    try:
        tol = Tolerances(float(t["position_m"]), float(t["tool_axis_deg"]), float(t["entry_direction_deg"]),
                         float(t["entry_span_m"]), bool(t.get("spin_free", True)), float(t.get("spin_tol_deg", 5.0)))
    except KeyError as e:
        raise SceneError(f"tolerances: missing key {e}") from None

    um = raw.get("umarell")
    cell_id = str(_req(raw, "cell_id", "scene"))
    if um:
        um_cell, um_par = str(_req(um, "cell_id", "umarell")), bool(um.get("parity", True))
    elif ver == 1 and _umarell_cell_exists(cell_id):
        um_cell, um_par = cell_id, True          # v1 scenes: the parity model was looked up by cell_id
        notes.append(f"schema_version 1: umarell parity cell inferred from cell_id ({cell_id}); declare it as "
                     "'umarell: {cell_id: ...}' (schema_version 2)")
    else:
        um_cell, um_par = None, False
    out = Scene(cell_id, path.resolve(), robots, coll, floor, boxes, wp, plan, tol,
                str(raw.get("description", "")).strip(), raw, umarell_cell=um_cell, umarell_parity=um_par,
                box_hull=box_hull, schema_version=ver, notes=notes)
    out._box_hull_declared = box_hull_declared
    if box_hull is not box_hull_declared and bh.get("no_dip") is None:
        # absent key + default on: record the rule in raw so that content-keyed caches see it
        out.raw = dict(raw)
        o2 = dict(out.raw.get("obstacles") or {})
        o2["box_hull"] = {**(o2.get("box_hull") or {}), "no_dip": True}
        out.raw["obstacles"] = o2
    return out


def _umarell_cell_exists(cell_id: str) -> bool:
    try:
        from env.cell import CELLS
    except Exception:  # noqa: BLE001
        return False
    return (CELLS / cell_id / "agentic_planner" / "config" / "robots.yaml").exists()


# ----------------------------------------------------------------------------------------------- task
def load_task(path: str | Path) -> Task:
    """Load a umarell task program (car_door_spot_welding.json format). Only ``type == "weld"`` operations
    are turned into welds; approach vectors and quaternions are normalised (validate() reports how far off
    unit they were)."""
    path = Path(path)
    if path.is_dir():
        path = path / "task.json"
    if not path.exists():
        raise SceneError(f"task file not found: {path}")
    try:
        raw = json.load(open(path))
    except json.JSONDecodeError as e:
        raise SceneError(f"{path}: invalid JSON: {e}") from None
    welds = []
    for k, op in enumerate(_req(raw, "operations", "task")):
        if op.get("type") != "weld":
            continue
        ctx = f"operations[{k}] ({op.get('id')})"
        a = _arr(op, "position", 3, ctx)
        av = _arr(op, "approach_vector", 3, ctx)
        if np.linalg.norm(av) < 1e-9:
            raise SceneError(f"{ctx}: zero approach_vector")
        wq = _quat(_req(op, "weld_orientation", ctx) if "weld_orientation" in op else _req(op, "orientation", ctx))
        if np.linalg.norm(wq) < 1e-9:
            raise SceneError(f"{ctx}: zero quaternion")
        tq = _quat(op["orientation"]) if "orientation" in op else None
        rw = op.get("rule_w")
        if rw is not None and not isinstance(rw, (bool, dict)):
            raise SceneError(f"{ctx}.rule_w: false/true or a mapping {{radius_m, tol_m}}")
        rw = None if rw is None else ({"enabled": bool(rw)} if isinstance(rw, bool) else dict(rw))
        ar = op.get("allowed_robots")
        welds.append(Weld(
            id=str(_req(op, "id", ctx)), position=a, approach=av / np.linalg.norm(av), weld_quat=wq / np.linalg.norm(wq),
            transit_quat=None if tq is None else tq / np.linalg.norm(tq), group=str(op.get("sequence_group", "")),
            priority=int(op.get("priority", 1)), placeholder=bool(op.get("placeholder", "position_note" in op)),
            original_position=None if "original_position" not in op else np.asarray(op["original_position"], float),
            note=str(op.get("position_note", "")), allowed_robots=None if ar is None else [str(x) for x in ar],
            rule_w=rw))
    strat = (raw.get("extensions", {}) or {}).get("strategy", {}) or {}
    return Task(str(raw.get("program", "")), str(raw.get("cell_id", "")), welds,
                list(strat.get("sequence_groups_order", [])), path.resolve(), raw)


def bind_task(scene: Scene, task: Task) -> Task:
    """Attach the scene's tool axis convention to every weld (required tool axis = R(weld) @ tool.axis)."""
    for w in task.welds:
        w.axis_local = np.asarray(scene.tool_axis_local, float).copy()
    return task


# ----------------------------------------------------------------------------------------------- validation
def _urdf_links(urdf: Path) -> set[str]:
    import xml.etree.ElementTree as ET

    return {l.get("name") for l in ET.parse(urdf).getroot().findall("link")}


def _srdf_pairs(srdf: Path) -> set[frozenset]:
    import xml.etree.ElementTree as ET

    return {frozenset((d.get("link1"), d.get("link2"))) for d in ET.parse(srdf).getroot().findall("disable_collisions")}


def rough_reach(robot: RobotSpec) -> float:
    """Rough reach radius from the base origin: sum of the joint origin offsets of the chain + |tcp_offset|."""
    try:
        from env.kinematics import _parse_urdf
        _, joints = _parse_urdf(str(robot.urdf))
        return float(sum(np.linalg.norm(j["xyz"]) for j in joints) + np.linalg.norm(robot.tool.tcp_offset))
    except Exception as e:  # noqa: BLE001
        import warnings
        warnings.warn(f"rough_reach: cannot read the chain of {robot.urdf} ({e!r}); using the KR240 R2900's 2.9 m "
                      "(the reach-based checks of this robot are then not from its URDF)", UserWarning, stacklevel=2)
        return 2.9 + float(np.linalg.norm(robot.tool.tcp_offset))


_YIELD_RE = re.compile(r"_YIELD_\d+")


def reserved_id(wid) -> str:
    """Why a weld id is reserved (empty string if it is fine): 'home' is the HOME operation of the BT plans,
    '<ROBOT>_YIELD_<n>' the yield moves, '|' separates ids in report keys."""
    w = str(wid)
    if not w or w.strip() != w:
        return "must be non-empty, without surrounding blanks"
    if w.lower() == "home":
        return "reserved ('home' is the HOME operation id of the plans)"
    if _YIELD_RE.search(w):
        return "reserved pattern '_YIELD_<n>' (yield-to-HOME operation ids)"
    if "|" in w:
        return "'|' is reserved (report keys 'weld|robot')"
    return ""


def validate(scene: Scene, task: Task | None = None, check_files: bool = True, coverage: bool = False) -> list[str]:
    """Consistency checks. Raises SceneError on hard errors; returns a list of warnings (strings).

    Hard: no robots, duplicate ids, non-increasing joint limits, home outside limits, non-positive limits/radii,
    missing URDF/SRDF, tip_link / tool-sphere frame / disabled-pair links not in the URDF, SRDF pairs not listed,
    robots with different tool axes, unsupported values (self_collision.geometry, workpiece.points/rule),
    task cell_id mismatch, duplicate weld ids, |quat| or |approach| off unit by > 1e-3, unknown allowed_robots,
    a weld below the floor, a tool axis pointing against the approach.
    Warnings: more than 4 robots, placeholder welds, lean (tool axis vs approach > 30 deg), weld out of rough reach
    of every (allowed) base, stricter-than-requested stroke settings, tool-sphere coverage (``coverage=True``)."""
    err, warn = [], list(scene.notes)
    ids = [r.id for r in scene.robots]
    if not ids:
        err.append("scene has no robots")
    if len(ids) > MAX_ROBOTS:
        warn.append(f"{len(ids)} robots: trajgen is tested for 1..{MAX_ROBOTS}")
    if len(set(ids)) != len(ids):
        err.append(f"duplicate robot ids {ids}")
    for rid in ids:
        if not str(rid) or "|" in str(rid) or str(rid).strip() != str(rid):
            err.append(f"robot id {rid!r}: must be non-empty, without '|' or surrounding blanks")
    for r in scene.robots:
        if np.any(r.q_hi <= r.q_lo):
            err.append(f"{r.id}: joint upper <= lower")
        if np.any(r.home < r.q_lo) or np.any(r.home > r.q_hi):
            err.append(f"{r.id}: home outside joint limits")
        if np.any(r.v_max <= 0) or np.any(r.a_max <= 0):
            err.append(f"{r.id}: non-positive velocity/acceleration limit")
        if len(r.joint_names) != 6:
            err.append(f"{r.id}: expected 6 joint names")
        if abs(np.linalg.det(r.base_R) - 1) > 1e-6:
            err.append(f"{r.id}: base rotation is not a rotation")
        cr = r.coll
        for name, v in (("link_radius", cr.link_radius), ("tcp_radius", cr.tcp_radius), ("fill_step", cr.fill_step)):
            if v <= 0:
                err.append(f"{r.id}: collision.{name} must be > 0")
        if np.any(cr.tool_spheres.radii <= 0):
            err.append(f"{r.id}: tool sphere radius <= 0")
        if not np.allclose(r.tool.axis, scene.robots[0].tool.axis, atol=1e-9):
            err.append(f"{r.id}: tool.axis {r.tool.axis.tolist()} differs from {scene.robots[0].id}'s "
                       f"(weld_orientation would mean different tool axes; not supported)")
        if check_files:
            for k, p in (("urdf", r.urdf), ("srdf", r.srdf)):
                if p is not None and not p.exists():
                    err.append(f"{r.id}: {k} not found: {p}")
            if r.urdf.exists():
                links = _urdf_links(r.urdf)
                if r.tool.tip_link not in links:
                    err.append(f"{r.id}: tip_link {r.tool.tip_link} not in URDF")
                if cr.tool_spheres.frame not in links:
                    err.append(f"{r.id}: tool_spheres.frame {cr.tool_spheres.frame} not in the URDF")
                pairs = scene.disabled_pairs_of(r.id)
                for a, b in pairs:
                    if a not in links or b not in links:
                        err.append(f"{r.id}: disabled pair ({a}, {b}) has a link not in the URDF")
                if r.srdf is not None and r.srdf.exists():
                    missing = _srdf_pairs(r.srdf) - {frozenset(p) for p in pairs}
                    if missing:
                        err.append(f"{r.id}: SRDF disables pairs the scene does not list: {sorted(map(sorted, missing))}")
    c = scene.collision
    for name, v in (("link_radius", c.link_radius), ("tcp_radius", c.tcp_radius), ("pair_radius", c.pair_radius),
                    ("fill_step", c.fill_step)):
        if v <= 0:
            err.append(f"collision.{name} must be > 0")
    if not np.isfinite(c.hull_min_distance) or c.hull_min_distance < 0:
        err.append(f"collision.arm_arm.hull_min_distance_m must be >= 0, got {c.hull_min_distance}")
    if c.self_geometry != "urdf_collision_hulls":
        err.append(f"collision.self_collision.geometry {c.self_geometry!r} not supported (only urdf_collision_hulls)")
    if scene.workpiece.rule not in ("wedge", "none"):
        err.append(f"workpiece.rule {scene.workpiece.rule!r} not supported (wedge | none)")
    if scene.workpiece.points != "tcp":
        err.append(f"workpiece.points {scene.workpiece.points!r} not supported (only tcp)")
    pl = scene.planning
    if not (0 < pl.velocity_scale <= 1 and 0 < pl.home_velocity_scale <= 1 and 0 < pl.approach_velocity_fraction <= 1):
        err.append("planning speed fractions must be in (0, 1]")
    if pl.acceleration_rule not in ("vscale_squared", "vscale", "none"):
        err.append(f"unknown acceleration_rule {pl.acceleration_rule}")
    if not (pl.min_offset <= pl.approach_offset and pl.min_offset <= pl.retract_offset):
        err.append("min_offset_m must be <= approach_offset_m and retract_offset_m")
    if pl.sample_dt <= 0 or pl.check_dt <= 0:
        err.append("planning.sample_dt and check_dt must be > 0")
    for k, v in (("dwell_s", pl.dwell_s), ("executor_margin_s", pl.executor_margin_s)):
        if v < 0:
            err.append(f"planning.{k} must be >= 0")
        elif pl.sample_dt > 0 and abs(v / pl.sample_dt - round(v / pl.sample_dt)) > 1e-6:
            warn.append(f"planning.{k} {v} is not a multiple of sample_dt {pl.sample_dt}: rounded up on the grid")
    if pl.min_offset < scene.tolerances.entry_span:
        warn.append(f"min_offset_m {pl.min_offset} < entry_span_m {scene.tolerances.entry_span}: entry check may see transit")
    if not pl.liftoff_required or not pl.liftoff_straight or not pl.descent_straight:
        warn.append("trajgen always makes straight on-axis descents and lift-offs (stricter than "
                    "liftoff.required/straight_on_axis false)")
    if not scene.tolerances.spin_free and scene.tolerances.spin_tol_deg < 0:
        err.append("tolerances.spin_tol_deg must be >= 0")
    if task is not None:
        bind_task(scene, task)
        if task.cell_id and task.cell_id != scene.cell_id:
            err.append(f"task cell_id {task.cell_id} != scene cell_id {scene.cell_id}")
        wids = [w.id for w in task.welds]
        if len(set(wids)) != len(wids):
            err.append("duplicate weld ids")
        for wid in wids:
            why = reserved_id(wid)
            if why:
                err.append(f"weld id {wid!r}: {why}")
        if not task.welds:
            err.append("task has no weld operations")
        for op in task.raw.get("operations", []):
            if op.get("type") != "weld":
                continue
            for key in ("weld_orientation", "orientation"):
                if key in op and abs(np.linalg.norm(_quat(op[key])) - 1) > 1e-3:
                    err.append(f"{op['id']}: |{key}| = {np.linalg.norm(_quat(op[key])):.5f}")
            if abs(np.linalg.norm(op["approach_vector"]) - 1) > 1e-3:
                err.append(f"{op['id']}: approach_vector not unit")
        reach = {r.id: rough_reach(r) for r in scene.robots}
        lean = []
        for w in task.welds:
            if w.allowed_robots is not None:
                bad = [x for x in w.allowed_robots if x not in ids]
                if bad:
                    err.append(f"{w.id}: allowed_robots {bad} are not scene robots {ids}")
            if w.placeholder:
                warn.append(f"{w.id}: placeholder position {w.position.tolist()} ({w.note})")
            ang = np.degrees(np.arccos(np.clip(w.tool_axis @ w.approach, -1, 1)))
            if ang > 30:
                lean.append(f"{w.id} {ang:.0f}")
            if ang > 90:
                err.append(f"{w.id}: tool axis points against the approach direction ({ang:.0f} deg)")
            cand = [r for r in scene.robots if w.allowed_robots is None or r.id in w.allowed_robots]
            if cand and min(np.linalg.norm(w.position - r.base_xyz) - reach[r.id] for r in cand) > 0:
                warn.append(f"{w.id}: farther than the rough reach of every allowed base")
            if scene.floor.enabled and w.position[2] <= scene.floor.z:
                err.append(f"{w.id}: below the floor")
        if lean:
            warn.append("tool axis leans > 30 deg off approach_vector (entry is not along the electrode): " + ", ".join(lean))
    if coverage and scene.robots and check_files:
        for r in scene.robots:
            rep = check_tool_coverage(scene, r.id)
            if rep["vertices"]["covered_frac"] < r.coll.tool_spheres.min_vertex_fraction:
                warn.append(f"{r.id}: tool spheres cover only {rep['vertices']['covered_frac']:.1%} of the hull vertices")
    if err:
        raise SceneError("; ".join(err))
    return warn


def check_tool_coverage(scene: Scene, rid: str | None = None, n_surface: int = 20000) -> dict:
    """Coverage of a robot's tool spheres against the URDF hulls of the sphere frame
    (origins with rpy applied): see ``trajgen.tool_spheres.report``."""
    from trajgen.tool_spheres import report

    r = scene.robot(rid) if rid else scene.robots[0]
    ts = r.coll.tool_spheres if r.coll is not None else scene.collision.tool_spheres
    return report(r.urdf, ts.centers, ts.radii, n_surface=n_surface, link=ts.frame)


# ----------------------------------------------------------------------------------------------- original
def load_original(cell_id: str = "welding_cell_2_small", *, verify: bool = False) -> Scene:
    """The umarell ORIGINAL cell (third_party/sample-umarell via ``env.cell.load_cell``) in the same
    dataclasses, for comparison. Its tool spheres are umarell's 35 grid spheres (``env.kinematics.tool_offsets``,
    radius 0.12), the floor is umarell's actual predicate (sphere centres z >= -0.05, no radius, tool
    spheres exempt: sphere_rule False / hull_rule False here), no self-collision check, no workpiece,
    and its offsets/dwell are what umarell executes (0.10 m default standoff, 0.5 s hold).

    Any number of arms: every robot of robots.yaml with its own tool grid, robot model (``robot_model``), tip link
    and TCP frame (a rotated ``tcp_rot``) and SRDF (cell.yaml ``srdf:``).

    ``verify=True`` (the dense re-check of N-arm umarell cells, tools.btexport.dense): the rules of
    ``tools.ctrl.safety.check_cell`` on top -- rule W (wedge, 20 mm / 3 mm, TCP) at every weld of the task;
    the cell's ``is_workpiece`` boxes stay exempt for the TCP / electrode spheres (the welds' contact surface),
    every other box is also checked against the URDF collision hulls (MuJoCo, >= 0); arm-arm hull contact
    (MuJoCo, >= 0) and hull self-contact with the SRDF's disabled pairs removed are violations."""
    import yaml

    from env.cell import load_cell, srdf_of
    from env.kinematics import tool_offsets

    cell = load_cell(cell_id)
    pol = cell.clearance
    try:
        arms = (yaml.safe_load(open(cell.root / "agentic_planner" / "config" / "robots.yaml")) or {})["robots"]["arms"]
    except (OSError, KeyError, TypeError):
        arms = {}
    robots = []
    for rid, rc in cell.robots.items():
        tip = rc.tip_link or "like_kr240r2900_link_6"
        C = tool_offsets(rc.tool_collision)
        ts = ToolSpheres(tip, C, np.full(len(C), 0.12))
        srdf = srdf_of(rc)
        pairs = sorted(tuple(sorted(x)) for x in _srdf_pairs(srdf) if len(x) == 2) if verify and srdf.exists() else []
        rot = getattr(rc, "tcp_rot", None)
        tool = ToolSpec(tip, np.asarray(rc.tcp_offset, float), np.array([-1.0, 0.0, 0.0])) if rot is None else \
            ToolSpec(tip, np.asarray(rc.tcp_offset, float), np.array([-1.0, 0.0, 0.0]), "custom", np.asarray(rot, float))
        robots.append(RobotSpec(
            id=rid, model=str((arms.get(rid) or {}).get("robot_model") or "kr240r2900"), urdf=Path(rc.urdf),
            srdf=srdf, base_xyz=np.asarray(rc.base, float), yaw=float(rc.yaw), home=np.asarray(rc.home, float),
            joint_names=[f"joint_a{k}" for k in range(1, 7)], q_lo=np.asarray(rc.q_lo), q_hi=np.asarray(rc.q_hi),
            v_max=np.asarray(rc.v_max), a_max=np.asarray(rc.a_max),
            tool=tool, base_R=rot_z(float(rc.yaw)),
            coll=RobotCollision(float(pol.get("robot_link_radius", 0.12)), 0.12, float(pol.get("tool_link_radius", 0.05)),
                                ts, pairs), umarell_id=rid))
    r0 = next(iter(cell.robots.values()))                 # the scene-level default: the first arm's tool grid
    tc = r0.tool_collision
    C = tool_offsets(tc)
    floor_obs = next((o for o in cell.obstacles if o.get("type") == "plane"), {})
    coll = CollisionSpec(
        link_radius=float(pol.get("robot_link_radius", 0.12)), fill_step=0.12, tcp_radius=float(pol.get("tool_link_radius", 0.05)),
        tool_spheres=ToolSpheres(r0.tip_link or "like_kr240r2900_link_6", C, np.full(len(C), 0.12)),
        pair_radius=0.12, arm_arm_gate=float(cell.collision.get("warning_distance", 0.20)), hull_backstop=bool(verify),
        hull_min_distance=0.0, self_geometry="urdf_collision_hulls" if verify else "none",
        self_min_distance=0.0 if verify else -np.inf, disabled_pairs=[])
    floor = FloorSpec(z=-0.05, margin=0.0, sphere_rule=False, hull_rule=False)
    boxes = [Box(str(o.get("id", "")), np.asarray(o["center"], float), np.asarray(o["half_extents"], float),
                 float(o.get("collision_margin", 0.05)), bool(o.get("is_workpiece", False)))
             for o in cell.obstacles if o.get("type") == "box"]
    pc = cell.planning
    plan = PlanningSpec(
        velocity_scale=float(pc.get("max_velocity_scaling", 0.7)), acceleration_rule="vscale_squared",
        approach_velocity_fraction=float(pol.get("approach_velocity_fraction", 0.4)),
        home_velocity_scale=float(pc.get("home_velocity_scale", 0.4)), approach_offset=0.10, retract_offset=0.10,
        min_offset=0.10, liftoff_required=False, liftoff_straight=False, liftoff_tube=np.inf, descent_straight=True,
        descent_tube=np.inf, dwell_s=0.5, executor_margin_s=1.0, sample_dt=0.05, check_dt=1 / 25)
    from env.cell import APPROACH_TOL_DEG, ORI_TOL_DEG, POS_TOL_M

    tol = Tolerances(POS_TOL_M, ORI_TOL_DEG, APPROACH_TOL_DEG, 0.04, True)
    wp = WorkpieceRule("wedge", 0.020, 0.003, "tcp") if verify else WorkpieceRule("none", 0.0, 0.0)
    sc = Scene(cell_id, None, robots, coll, floor, boxes, wp, plan, tol,
               str(cell.task.get("description", "")), {"floor_obstacle": floor_obs, "tool_collision": tc}, "umarell_original",
               umarell_cell=cell_id, umarell_parity=True, box_hull=BoxHullRule(bool(verify), 0.0))
    return sc


def load_original_task(cell_id: str = "welding_cell_2_small") -> Task:
    """umarell's task program as env.cell reads it (agentic_planner/tasks/car_door_spot_welding.json)."""
    from env.cell import CELLS

    return load_task(CELLS / cell_id / "agentic_planner" / "tasks" / "car_door_spot_welding.json")


def summary(scene: Scene, task: Task | None = None) -> str:
    """Human-readable one-screen summary (used by ``python -m trajgen.scene``)."""
    L = [f"scene {scene.cell_id} ({scene.origin}, schema {scene.schema_version}) {scene.path or ''}"]
    for r in scene.robots:
        rpy = np.rad2deg(_R_to_rpy(r.base_R)).round(2).tolist()
        L.append(f"  {r.id}: base {r.base_xyz.tolist()} rpy(deg) {rpy} home(deg) {np.rad2deg(r.home).round(1).tolist()} "
                 f"tcp {r.tool.tcp_offset.tolist()} tool spheres {len(r.coll.tool_spheres.radii)} "
                 f"urdf {'ok' if r.urdf.exists() else 'MISSING'}")
    c = scene.collision
    L.append(f"  arm-arm gate {c.arm_arm_gate} m @ r {c.pair_radius}; hull backstop {c.hull_backstop} (min {c.hull_min_distance} m); "
             f"umarell parity cell {scene.umarell_cell} (default {'on' if scene.umarell_parity else 'off'})")
    L.append(f"  floor z {scene.floor.z} margin {scene.floor.margin} spheres {scene.floor.sphere_rule} hulls {scene.floor.hull_rule}; "
             f"boxes {len(scene.boxes)} (hull rule {scene.box_hull.enabled}"
             + (f", electrode off-weld rule r {scene.box_hull.electrode.radius} m" if scene.box_hull.electrode.enabled
                else "") + f"); workpiece {scene.workpiece.rule}")
    p = scene.planning
    L.append(f"  planning v {p.velocity_scale} a x{p.accel_scale():.2f} approach x{p.approach_velocity_fraction} home {p.home_velocity_scale} "
             f"offsets {p.approach_offset}/{p.retract_offset} dwell {p.dwell_s} dt {p.sample_dt} liftoff {p.liftoff_required}")
    if task is not None:
        L.append(f"  task {task.program}: {len(task.welds)} welds, groups {task.sequence_groups_order}, "
                 f"placeholders {[w.id for w in task.welds if w.placeholder]}")
    return "\n".join(L)


def _R_to_rpy(R):
    R = np.asarray(R, float)
    p = np.arcsin(-np.clip(R[2, 0], -1, 1))
    if abs(np.cos(p)) > 1e-9:
        return np.array([np.arctan2(R[2, 1], R[2, 2]), p, np.arctan2(R[1, 0], R[0, 0])])
    return np.array([np.arctan2(-R[1, 2], R[1, 1]), p, 0.0])


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Load + validate a scene (scenes/SCHEMA.md).")
    ap.add_argument("--scene", default=str(ROOT / "scenes" / "welding_cell_2_small"))
    ap.add_argument("--task", default=None, help="task.json (default: <scene dir>/task.json)")
    ap.add_argument("--coverage", action="store_true", help="also check tool-sphere coverage of the URDF hulls")
    a = ap.parse_args()
    s = load_scene(a.scene)
    t = load_task(a.task or (s.path.parent / "task.json"))
    w = validate(s, t, coverage=a.coverage)
    print(summary(s, t))
    for x in w:
        print("  warning:", x)
    if a.coverage:
        for r in s.robots:
            print(r.id, json.dumps(check_tool_coverage(s, r.id)["vertices"]))
