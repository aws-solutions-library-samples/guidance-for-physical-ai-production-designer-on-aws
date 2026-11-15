"""Cell description shared by every module: robots, welds, obstacles, limits.

Everything is read from the umarell repo copy in ``third_party/sample-umarell`` so the
numbers stay identical to what the umarell planner and validators use.

Robot model (track P3-URDF, docs/p3_urdf.md): every robot's URDF is parsed by ``robotmodel.URDFRobot`` and its
<limit lower upper velocity> are checked against robots.yaml (``RobotCfg.limits_check``; a UserWarning on a mismatch
beyond robots.yaml's 6-7 digit rounding). Default policy ``KWR_LIMITS_SOURCE=auto`` (P3-URDF fix, 2026-10-12): the
URDF is the source of the limits; robots.yaml's hand copies are used only where they equal the URDF within that
rounding (``limits_source="config"``: bit-identical to every published result), otherwise the URDF values are used
and a UserWarning names the mismatch (``limits_source="urdf"``; e.g. after an in-place URDF edit).
``KWR_LIMITS_SOURCE=config`` restores the old behaviour (robots.yaml always, warning only); ``=urdf`` always uses the
exact URDF numbers. A robot override
(``robotmodel.override``: env var KWR_ROBOT_OVERRIDE, ``set_robot_override``, or execute.py --urdf/--robot-base)
replaces the URDF, base pose, home, TCP or acceleration limits of the robots; with an override URDF the limits come
from that URDF. URDFs carry no acceleration limits: a_max = override, else robots.yaml joint_acceleration_limits
(umarell "REVIEW" default), else the per-model default in robotmodel.urdf_robot.ACCEL_DEFAULTS.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field, replace
from functools import lru_cache
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
UMARELL = Path(os.environ.get("UMARELL_REPO", ROOT / "third_party" / "sample-umarell"))
CELLS = UMARELL / "cells"
ROBOT_IDS = ("robot1", "robot2")
N_JOINTS = 6
FIXTURE_POS = np.array([-0.9819119731486827, -1.2386438543248453, 0.0])
FIXTURE_GLB = CELLS / "urdf-fixture-collection" / "GEO_FIXTURE_OP1000" / "meshes_lowres" / "World.glb"


@dataclass
class RobotCfg:
    rid: str
    base: np.ndarray            # (3,) world position of the base link
    yaw: float
    home: np.ndarray            # (6,) rad
    q_lo: np.ndarray
    q_hi: np.ndarray
    v_max: np.ndarray           # URDF joint velocity limits, rad/s
    a_max: np.ndarray
    tcp_offset: np.ndarray      # (3,) TCP in the link_6 frame; tool axis = TCP frame -x
    tool_collision: dict
    urdf: Path
    # P3-URDF (all optional, defaults keep the old constructor calls valid)
    j_max: np.ndarray | None = None                   # rad/s^3, only from an explicit override
    tip_link: str = ""                                # link whose frame tcp_offset is in (last movable link)
    limits_source: str = "config"                     # values used: "config" (robots.yaml, = URDF) | "urdf"
    accel_source: str = "robots.yaml"
    limits_check: dict = field(default_factory=dict)  # URDF vs robots.yaml comparison (mismatches listed)
    tcp_source: str = "robots.yaml tcp_offset"        # where the TCP comes from (P3U round 2)
    srdf: Path | None = None                          # cell.yaml's srdf: (default: beside the URDF), see srdf_of
    tcp_rot: np.ndarray | None = None                 # (3, 3) TCP frame axes in the link_6 frame (None: link_6's):
                                                      # a rotated tip_link, e.g. a gun at 90 deg to the flange axis


def srdf_of(robot) -> Path:
    """The robot's SRDF: its own ``srdf`` (cell.yaml's srdf:) if it has one, else the one beside its URDF."""
    srdf = getattr(robot, "srdf", None)
    return Path(srdf) if srdf else Path(robot.urdf).with_suffix(".srdf")


@dataclass
class Weld:
    wid: str
    pos: np.ndarray             # (3,) world
    quat_xyzw: np.ndarray       # (4,) weld_orientation (TCP frame in world)
    approach: np.ndarray        # (3,) unit approach vector
    group: str
    priority: int


@dataclass
class Cell:
    cell_id: str
    root: Path
    robots: dict[str, RobotCfg]
    welds: list[Weld]
    obstacles: list[dict]       # raw obstacles.yaml entries (floor plane + boxes)
    collision: dict             # robots.yaml `collision` block (warning_distance 0.20 ...)
    clearance: dict             # obstacles.yaml clearance policy (robot_link_radius ...)
    planning: dict
    task: dict = field(repr=False)
    robot_override: dict | None = None    # robotmodel.override spec in effect when the cell was loaded

    @property
    def boxes(self) -> tuple[np.ndarray, np.ndarray]:
        """(centers (B,3), half_extents (B,3)) of all box obstacles."""
        bs = [o for o in self.obstacles if o["type"] == "box"]
        if not bs:
            return np.zeros((0, 3)), np.zeros((0, 3))
        return (np.array([b["center"] for b in bs], float), np.array([b["half_extents"] for b in bs], float))

    @property
    def velocity_scale(self) -> float:
        return float(self.planning.get("max_velocity_scaling", 0.7))

    @property
    def has_fixture(self) -> bool:
        return self.cell_id != "welding_cell_2_small"


def _quat(d: dict) -> np.ndarray:
    return np.array([d["x"], d["y"], d["z"], d["w"]], float)


def load_cell(cell_id: str = "welding_cell_2_small", robot_override=None) -> Cell:
    """The cell; ``robot_override`` (spec, see robotmodel.override) defaults to the process-wide override
    (KWR_ROBOT_OVERRIDE), None when unset. Cached per (cell, override, limits source)."""
    from robotmodel.override import active_override, dumps, normalize
    ov = normalize(robot_override) if robot_override is not None else active_override()
    return _load_cell(cell_id, dumps(ov), os.environ.get("KWR_LIMITS_SOURCE", "auto"))


def _robot_cfg(rid: str, c: dict, urdf: Path, ov: dict, limits_env: str) -> RobotCfg:
    from robotmodel.urdf_robot import URDFRobot, compare_limits, warn_mismatch
    urdf = Path(ov["urdf"]) if ov.get("urdf") else urdf
    tip = ov.get("tip_link") or c.get("tip_link") or None
    tcp = np.array(ov.get("tcp_offset", c["tcp_offset"]), float)
    if ov.get("accel") is not None:
        accel, asrc = ov["accel"], "override"
    elif c.get("joint_acceleration_limits") is not None:
        accel, asrc = c["joint_acceleration_limits"], "robots.yaml joint_acceleration_limits (umarell REVIEW default)"
    else:
        accel, asrc = None, None
    m = URDFRobot(urdf, tip_link=tip, tcp_offset=tcp, accel=accel, jerk=ov.get("jerk"), model=c.get("robot_model"),
                  accel_source=asrc)
    if m.n_dof != N_JOINTS:
        raise ValueError(f"{rid}: {urdf} has {m.n_dof} movable joints on the arm chain; env/ needs {N_JOINTS}")
    if m.a_max is None:
        raise ValueError(f"{rid}: no acceleration limits for {urdf} (URDFs carry none): give robots.yaml "
                         "joint_acceleration_limits or an override 'accel'")
    tcp_source = ("robot override tcp_offset" if ov.get("tcp_offset") is not None else "robots.yaml tcp_offset") + \
        (f" in override tip_link {tip}" if ov.get("tip_link") else "")
    from robotmodel.urdf_robot import tcp_link_note
    note = tcp_link_note(m, tcp_source) if not ov.get("tip_link") else None
    if note:
        import warnings
        warnings.warn(f"{rid}: {note}", UserWarning, stacklevel=3)
    tcp_rot = None
    if not np.allclose(m.T_tip, np.eye(4), atol=1e-12):
        # tip_link hangs off the last movable link through fixed joints: env/ expresses the TCP in that link's frame,
        # with the fixed joints' rotation as the TCP frame's axes (a tool working at an angle to the flange axis)
        tcp = m.T_tcp[:3, 3].copy()
        if not np.allclose(m.T_tcp[:3, :3], np.eye(3), atol=1e-12):
            tcp_rot = m.T_tcp[:3, :3].copy()
    ul = m.limits()
    cfg = {"lower": c.get("joint_limits_lower"), "upper": c.get("joint_limits_upper"),
           "velocity": c.get("joint_velocity_limits")}
    src = ov.get("limits") or ("urdf" if ov.get("urdf") else limits_env)
    if src not in ("auto", "config", "urdf"):
        raise ValueError(f"{rid}: limits source {src!r} (KWR_LIMITS_SOURCE / override 'limits'): auto | config | urdf")
    policy = src
    if src in ("config", "auto") and any(v is None for v in cfg.values()):
        src = "urdf"                                    # robots.yaml without limits: read them from the URDF
    probs = compare_limits(ul, cfg)
    if src == "auto":
        # default: the URDF is the source of the limits. robots.yaml's hand copies are a cross-check; where they
        # agree with the URDF (within their 6-7 digit rounding) they are used as they are (bit-identical results),
        # where they do not (an edited / replaced URDF), the URDF wins, with a warning
        src = "urdf" if probs else "config"
    if probs:
        warn_mismatch(f"{rid} robots.yaml vs {urdf.name}", probs,
                      "robots.yaml values kept (KWR_LIMITS_SOURCE=config)" if src == "config" else
                      "URDF values used (robots.yaml is only a cross-check)")
    if src == "config":
        q_lo, q_hi, v_max = (np.array(cfg[k], float) for k in ("lower", "upper", "velocity"))
    else:
        q_lo, q_hi, v_max = m.q_lo.copy(), m.q_hi.copy(), m.v_max.copy()
    if not np.all(np.isfinite(v_max)) or np.any(v_max <= 0):
        raise ValueError(f"{rid}: {urdf} lacks a positive <limit velocity> on every arm joint: {v_max}")
    base = np.array(ov["base"] if ov.get("base") is not None else c["base_position"], float)
    yaw = float(ov["yaw"]) if ov.get("yaw") is not None else float(c.get("base_orientation_yaw", 0.0))
    home = np.deg2rad(np.array(ov["home_deg"] if ov.get("home_deg") is not None else c["home_pose_deg"], float))
    return RobotCfg(
        rid=rid, base=base, yaw=yaw, home=home, q_lo=q_lo, q_hi=q_hi, v_max=v_max, a_max=np.array(m.a_max, float),
        tcp_offset=tcp, tcp_rot=tcp_rot, tool_collision=c.get("tool_collision", {}), urdf=urdf, j_max=m.j_max,
        tip_link=m.link_names[-1],
        limits_source=src, accel_source=m.accel_source, tcp_source=tcp_source,
        limits_check={"urdf": str(urdf), "mismatches": probs, "urdf_limits": ul, "source": src,
                      "policy": policy})


@lru_cache(maxsize=None)
def _load_cell(cell_id: str, ov_json: str, limits_env: str) -> Cell:
    from robotmodel.override import override_for
    ovd = json.loads(ov_json) if ov_json else None
    root = CELLS / cell_id
    ap = root / "agentic_planner"
    rcfg = yaml.safe_load(open(ap / "config" / "robots.yaml"))
    ocfg = yaml.safe_load(open(ap / "config" / "obstacles.yaml"))
    manifest = yaml.safe_load(open(ap / "cell.yaml"))
    urdfs = {r["id"]: root / r["urdf"] for r in manifest["robots"]}
    # A reference cell's SRDFs are its own (agentic_planner/srdf/, the backend's Prepare Cell), not beside the
    # catalogue's URDFs: cell.yaml's srdf: says where, relative to the cell like urdf:.
    srdfs = {r["id"]: root / r["srdf"] for r in manifest["robots"] if r.get("srdf")}
    robots = {}
    for rid, c in rcfg["robots"]["arms"].items():
        robots[rid] = replace(_robot_cfg(rid, c, urdfs[rid], override_for(ovd, rid), limits_env), srdf=srdfs.get(rid))
    task = json.load(open(ap / "tasks" / "car_door_spot_welding.json"))
    welds = [Weld(o["id"], np.array(o["position"], float), _quat(o["weld_orientation"]),
                  np.array(o["approach_vector"], float), o.get("sequence_group", ""), int(o.get("priority", 1)))
             for o in task["operations"] if o["type"] == "weld"]
    clearance = {k: v for k, v in ocfg.items() if k != "obstacles"}
    return Cell(cell_id, root, robots, welds, ocfg["obstacles"], rcfg["collision"], clearance,
                rcfg.get("planning", {}), task, robot_override=ovd)


def plan_path(name: str, cell_id: str = "welding_cell_2_small") -> Path:
    """Saved umarell plan, e.g. plan_path('bt_1790851970')."""
    return CELLS / cell_id / "agentic_planner" / "plans" / f"{name}_{cell_id}.json"


# Tolerances of umarell's validate_approach (agents/trajectory/tools/pinocchio.py).
POS_TOL_M = 0.010
ORI_TOL_DEG = 5.0
APPROACH_TOL_DEG = 15.0
