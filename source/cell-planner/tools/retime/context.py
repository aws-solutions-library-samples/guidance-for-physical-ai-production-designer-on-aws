"""Cell / scene context of a retiming: robots, URDF limits, welds, FK, arm-arm clearance and the dense re-check.

Two kinds, same interface:
* ``cell``  — a umarell cell (``third_party/sample-umarell/cells/<id>``; two arms robot1 / robot2). FK and spheres
  are env/kinematics (tested equal to umarell); the dense re-check is tools.ctrl.safety.check_cell (umarell sphere
  gate, floor / box rules, MuJoCo hulls self / arm-arm / floor / fixture (2t: ``fixture_hull=True``), rule W TCP +
  electrode, joint limits, weld pose / axis / entry / still hold).
* ``scene`` — a trajgen scene directory (scene.yaml + task.json; 1-N robots). FK and spheres are trajgen.model; the
  dense re-check is trajgen.verify.verify_timeline via tools.ctrl.safety.check_trajgen (corrected + umarell sphere
  gates on every pair, MuJoCo hull buffer, self hulls, floor, boxes, rule W, weld checks, continuous certificate).

Joint limits come from the robot's URDF (``<limit lower upper velocity>``, robotmodel.URDFRobot, the project's single
URDF parser). URDFs carry no acceleration limits: they come from the per-robot override of the configuration
(cell: robots.yaml ``joint_acceleration_limits``; scene: scene.yaml ``joints.acceleration``), else robotmodel's
per-model default (KR240 R2900: [3, 3, 3, 6, 6, 8] rad/s^2, umarell's REVIEW default, not measured), or an explicit
``accel`` / ``jerk`` override passed to :func:`load_context`. ``limits_report`` records every source and every
mismatch between the URDF and the hand-copied config values.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


@dataclass
class RobotLimits:
    rid: str
    urdf: str
    joint_names: list
    q_lo: np.ndarray
    q_hi: np.ndarray
    v_max: np.ndarray            # rad/s, URDF <limit velocity> (robots.yaml's where tighter: v_source)
    a_max: np.ndarray            # rad/s^2, override / default (not in URDFs)
    j_max: np.ndarray | None     # rad/s^3, optional
    accel_source: str
    jerk_source: str
    config_mismatch: list = field(default_factory=list)
    v_source: str = "URDF <limit velocity>"
    q_source: str = "URDF <limit lower upper>"

    def to_dict(self):
        f = lambda a: None if a is None else [round(float(x), 7) for x in a]   # noqa: E731
        return dict(robot=self.rid, urdf=_rel(self.urdf), joint_names=self.joint_names, lower=f(self.q_lo),
                    upper=f(self.q_hi), velocity=f(self.v_max), velocity_source=self.v_source,
                    position_source=self.q_source,
                    acceleration=f(self.a_max), acceleration_source=self.accel_source, jerk=f(self.j_max),
                    jerk_source=self.jerk_source, config_vs_urdf_mismatches=self.config_mismatch)


def _rel(p):
    try:
        return str(Path(p).resolve().relative_to(ROOT))
    except ValueError:
        return str(p)


def _limits(rid, urdf, tip_link, tcp_offset, cfg_accel, cfg_accel_src, config_vals, accel=None, jerk=None):
    from robotmodel import URDFRobot, compare_limits
    if accel is not None:
        a, src = np.asarray(accel, float), "override (load_context accel=)"
    elif cfg_accel is not None:
        a, src = np.asarray(cfg_accel, float), cfg_accel_src
    else:
        a, src = None, None
    rm = URDFRobot(urdf, tip_link=tip_link, tcp_offset=tcp_offset, accel=a, accel_source=src, model="kr240r2900"
                   if "kr240" in str(urdf).lower() else None)
    if rm.a_max is None:
        raise ValueError(f"{rid}: no acceleration limits (URDFs carry none); give joint_acceleration_limits in the "
                         f"config or an accel override")
    lim = rm.limits()
    mism = compare_limits(lim, config_vals) if config_vals else []
    if jerk is not None:
        j, jsrc = np.asarray(jerk, float), "override (load_context jerk=)"
    else:
        j, jsrc = None, "none (profile jerk = j_frac x a_max per s)"
    # robots.yaml's speeds / joint ranges where tighter than the URDF's (the exporter's rule, tools.btexport.limits)
    from tools.btexport.limits import tighten
    t = tighten(rm.q_lo, rm.q_hi, rm.v_max, config_vals, f"{rid} robots.yaml")
    return RobotLimits(rid=rid, urdf=rm.path, joint_names=rm.joint_names, q_lo=t["lower"], q_hi=t["upper"],
                       v_max=t["velocity"], a_max=np.asarray(rm.a_max, float), j_max=j, v_source=t["v_source"],
                       q_source=t["q_source"],
                       accel_source=rm.accel_source, jerk_source=jsrc, config_mismatch=mism)


class Context:
    """Common interface; see :class:`CellContext` and :class:`SceneContext`."""
    kind: str
    name: str
    robot_ids: tuple
    limits: dict                  # rid -> RobotLimits
    welds: dict                   # wid -> (3,) world position
    gate: float                   # arm-arm sphere gate (clearance must be > gate)
    approach_offset_m: float = 0.10       # umarell approach_offset_m default (planner_backend / ompl_planner)

    def limits_report(self):
        return {r: self.limits[r].to_dict() for r in self.robot_ids}

    def stack(self, key):
        return np.stack([getattr(self.limits[r], key) for r in self.robot_ids])


class CellContext(Context):
    kind = "cell"

    def __init__(self, cell_id: str, accel=None, jerk=None):
        import yaml
        from env.cell import load_cell
        from env.kinematics import ArmKinematics, SphereModel
        self.name = cell_id
        self.cell = c = load_cell(cell_id)
        self.robot_ids = tuple(c.robots)
        rcfg = yaml.safe_load(open(c.root / "agentic_planner" / "config" / "robots.yaml"))["robots"]["arms"]
        self.limits = {}
        for rid, rc in c.robots.items():
            a = rcfg[rid].get("joint_acceleration_limits")
            asrc = "robots.yaml joint_acceleration_limits (umarell: REVIEW default)" if a else None
            if rc.accel_source == "override":          # robotmodel.override accel (KWR_ROBOT_OVERRIDE / --robot-accel)
                a, asrc = rc.a_max, "robot override accel (robotmodel.override)"
            cfg = dict(lower=rcfg[rid].get("joint_limits_lower"), upper=rcfg[rid].get("joint_limits_upper"),
                       velocity=rcfg[rid].get("joint_velocity_limits"))
            if c.robot_override is not None:           # the override URDF replaces robots.yaml's hand copies
                cfg = None
            self.limits[rid] = _limits(rid, rc.urdf, rc.tip_link or rcfg[rid].get("tip_link"), rc.tcp_offset, a,
                                       asrc, cfg, accel, jerk)
        self.kin = {r: ArmKinematics(c.robots[r]) for r in self.robot_ids}
        self.spheres = SphereModel(c)
        self.gate = float(self.spheres.warning_distance)
        self.welds = {w.wid: np.asarray(w.pos, float) for w in c.welds}
        self._mj = None

    def tcp(self, rid, Q):
        return self.kin[rid].tcp(np.asarray(Q, float))[0]

    def pair_clearance(self, ra, rb, Qa, Qb):
        """umarell sphere-gate clearance (m), batched."""
        if (ra, rb) == ("robot2", "robot1"):
            Qa, Qb = Qb, Qa
        return self.spheres.pair_min(np.asarray(Qa, float), np.asarray(Qb, float))[0]

    def hull_pair_contact(self, ra, rb, Qa, Qb):
        """MuJoCo convex-hull arm-arm penetration (bool per sample)."""
        import mujoco
        if self._mj is None:
            from env.weld_env import EnvConfig, KukaWeldEnv
            e = KukaWeldEnv(EnvConfig(cell_id=self.name, episode_len=10 ** 7))
            self._mj = (e, mujoco.MjData(e.model))
        e, d = self._mj
        ia, ib = self.robot_ids.index(ra), self.robot_ids.index(rb)
        out = np.zeros(len(Qa), bool)
        q = np.zeros((2, 6))
        for n in range(len(Qa)):
            q[ia], q[ib] = Qa[n], Qb[n]
            d.qpos[e.qadr] = q
            mujoco.mj_kinematics(e.model, d)
            mujoco.mj_collision(e.model, d)
            out[n] = bool(e._mj_contacts(d)["arm_arm"])
        return out

    def full_check(self, ts, Q, weld_events, hull_every=1):
        from tools.ctrl.safety import check_cell
        Q = np.asarray(Q, float)
        order = [self.robot_ids.index(r) for r in ("robot1", "robot2")]
        return check_cell(self.name, ts, Q[:, order], weld_events, hull_every=hull_every)

    @staticmethod
    def verdict(rv, sv):
        from tools.ctrl.safety import verdict
        return verdict(rv, sv)


class SceneContext(Context):
    kind = "scene"

    def __init__(self, scene_dir, accel=None, jerk=None):
        import yaml
        from robotmodel.override import active_override
        if active_override() is not None:
            raise ValueError("SceneContext: a robot override (KWR_ROBOT_OVERRIDE) is not applied to trajgen scenes; "
                             "the scene's robots[i].urdf defines the robots (use accel= for other acceleration limits)")
        from trajgen.model import SceneModel
        from trajgen.scene import load_scene, load_task
        d = Path(scene_dir)
        if d.is_file():
            d = d.parent
        self.dir = d.resolve()
        self.name = _rel(self.dir)
        self.scene = load_scene(d / "scene.yaml")
        self.task = load_task(d / "task.json")
        self.model = SceneModel(self.scene, self.task)
        raw = {r["id"]: r for r in yaml.safe_load(open(d / "scene.yaml"))["robots"]}
        self.robot_ids = tuple(r.id for r in self.scene.robots)
        self.limits = {}
        for r in self.scene.robots:
            j = raw[r.id].get("joints", {})
            a = j.get("acceleration")
            cfg = dict(lower=j.get("lower"), upper=j.get("upper"), velocity=j.get("velocity"))
            self.limits[r.id] = _limits(r.id, r.urdf, r.tool.tip_link, r.tool.tcp_offset, a,
                                        "scene.yaml joints.acceleration" if a is not None else None, cfg, accel, jerk)
        self.gate = float(self.scene.collision.arm_arm_gate)
        self.um_gate = float(self.model.umarell.warning_distance) if self.model.umarell is not None else None
        self.welds = {w.id: np.asarray(w.position, float) for w in self.task.welds}

    def tcp(self, rid, Q):
        return self.model.tcp(rid, np.asarray(Q, float))[0]

    def pair_clearance(self, ra, rb, Qa, Qb):
        """min over the scene's corrected sphere gate and (when available) umarell's 35-sphere gate, each relative to
        its own gate, expressed against ``self.gate``."""
        c = self.model.pair_clearance(ra, rb, np.asarray(Qa, float), np.asarray(Qb, float))
        if self.model.umarell is not None:
            cu = self.model.umarell_pair_clearance(ra, rb, np.asarray(Qa, float), np.asarray(Qb, float))
            c = np.minimum(c, cu - self.um_gate + self.gate)
        return c

    def hull_pair_contact(self, ra, rb, Qa, Qb):
        return np.asarray(self.model.mj_pair_contact(ra, rb, np.asarray(Qa, float), np.asarray(Qb, float)), bool)

    def full_check(self, ts, Q, weld_events, hull_every=1):
        from tools.ctrl.safety import check_trajgen
        return check_trajgen(self.dir, ts, Q, self.robot_ids, weld_events)

    @staticmethod
    def verdict(rv, sv):
        from tools.ctrl.safety import verdict_trajgen
        return verdict_trajgen(rv, sv)


@lru_cache(maxsize=8)
def _cached(kind, name, accel, jerk, override_json=""):
    a = None if accel is None else list(accel)
    j = None if jerk is None else list(jerk)
    return CellContext(name, a, j) if kind == "cell" else SceneContext(name, a, j)


def load_context(cell: str | None = None, scene: str | None = None, accel=None, jerk=None) -> Context:
    """``cell`` = umarell cell id, or ``scene`` = trajgen scene dir / scene.yaml (exactly one)."""
    if (cell is None) == (scene is None):
        raise ValueError("give exactly one of cell / scene")
    t = lambda x: None if x is None else tuple(float(v) for v in x)   # noqa: E731
    from robotmodel.override import active_override, dumps
    ov = dumps(active_override())                 # a robot override changes URDF / accel: separate cache entry
    if cell is not None:
        return _cached("cell", cell, t(accel), t(jerk), ov)
    p = Path(scene)
    if not p.is_absolute():
        p = (ROOT / p) if (ROOT / p).exists() else p.resolve()
    return _cached("scene", str(p.resolve()), t(accel), t(jerk), ov)


def guess_context(meta: dict, path: str | None = None):
    """(cell, scene) from a timeline's meta (cell / cell_id / scene keys) or a trajgen sample path."""
    if meta.get("scene"):
        return None, str(meta["scene"])
    for k in ("cell", "cell_id"):
        if meta.get(k):
            return str(meta[k]), None
    if path is not None:
        p = Path(path).resolve()
        for anc in p.parents:
            if (anc / "scene.yaml").exists() and (anc / "task.json").exists():
                return None, str(anc)
    return None, None


def dumps(x):
    return json.dumps(x, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
