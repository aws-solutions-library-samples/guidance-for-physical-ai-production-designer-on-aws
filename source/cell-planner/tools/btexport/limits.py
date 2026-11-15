"""Robot kinematics limits for the exporter, read from the URDF (no hand-copied limits).

* joint names, position limits and ``<limit velocity>`` of the movable joints on the chain base -> tip: parsed from
  the robot's URDF by ``robotmodel.URDFRobot`` (the project's single URDF parser, track URDF) or, without it, by this
  module's reader of the ``<limit>`` elements;
* acceleration limits are NOT in a URDF. They come from an optional per-robot override, in this order:
  ``scene.yaml robots[i].joints.acceleration`` (trajgen scenes) or ``robots.yaml joint_acceleration_limits``
  (umarell cells), else the KR240 default ``[3, 3, 3, 6, 6, 8]`` rad/s^2. That default is the value umarell's
  robots.yaml carries with the comment ``# REVIEW`` (not a datasheet value; scenes/ERRATA.md E13), so every record
  says where its accelerations came from (``a_source``) and flags the default with ``a_review=True``.

The hand-copied values of the cell / scene files (robots.yaml ``joint_velocity_limits``, scene.yaml
``joints.velocity``) are cross-checked (``v_copy_max_abs_diff``). A umarell cell's robots.yaml velocity and position
limits that are TIGHTER than the URDF's (the robot controller's axis speeds and software limits, which Prepare Cell
writes from the robot model's planner.yaml) bound the export too (``tighten``: the smaller speed, the narrower range,
per joint; ``v_source`` / ``q_source`` say which applied): umarell planned with them, and a full-profile export
timed to the URDF's ``<limit velocity>`` would run the robot faster than it can. Where robots.yaml equals the URDF
within its 6-7 digit rounding, the URDF numbers are used as they are.

Pure numpy + stdlib xml + yaml; runs in both venvs.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
KR240_ACC_DEFAULT = np.array([3.0, 3.0, 3.0, 6.0, 6.0, 8.0])   # umarell robots.yaml "REVIEW" (rad/s^2)


@dataclass
class JointLimits:
    robot: str
    urdf: str
    joint_names: list
    lower: np.ndarray
    upper: np.ndarray
    v_max: np.ndarray            # URDF <limit velocity>, rad/s
    a_max: np.ndarray            # override (see module doc), rad/s^2
    a_source: str
    a_review: bool
    v_copy: Optional[np.ndarray] = None    # the hand-copied velocity of the cell / scene file (cross-check only)
    notes: list = field(default_factory=list)
    v_source: str = "URDF <limit velocity>"
    q_source: str = "URDF <limit lower upper>"

    def record(self) -> dict:
        d = {"urdf": _rel(self.urdf), "joint_names": list(self.joint_names),
             "v_max_rad_s": [round(float(v), 6) for v in self.v_max], "v_source": self.v_source,
             "a_max_rad_s2": [round(float(v), 6) for v in self.a_max], "a_source": self.a_source,
             "a_review": bool(self.a_review),
             "lower": [round(float(v), 6) for v in self.lower], "upper": [round(float(v), 6) for v in self.upper],
             "q_source": self.q_source}
        if self.v_copy is not None:
            d["v_copy_max_abs_diff"] = round(float(np.abs(self.v_copy - self.v_max).max()), 9)
        if self.notes:
            d["notes"] = list(self.notes)
        return d


def _rel(p) -> str:
    p = Path(p).resolve()
    return str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p)


def urdf_chain_limits(urdf: str | Path, tip_link: Optional[str] = None) -> dict:
    """Names, lower, upper, velocity of the movable joints base -> ``tip_link`` (default: the child of the deepest
    movable joint), read by the project's single URDF parser ``robotmodel.URDFRobot`` (track URDF, docs/p3_urdf.md)
    when it is there, else by this module's own reader of the ``<limit>`` elements. Returns arrays (n,)."""
    try:
        from robotmodel import URDFRobot
    except ImportError:
        URDFRobot = None
    if URDFRobot is not None:
        rm = URDFRobot(str(urdf), tip_link=tip_link)
        return {"names": list(rm.joint_names), "lower": np.asarray(rm.q_lo, float), "upper": np.asarray(rm.q_hi, float),
                "velocity": np.asarray(rm.v_max, float), "tip_link": rm.tip_link, "parser": "robotmodel.URDFRobot"}
    root = ET.parse(str(urdf)).getroot()
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
    if tip_link is None:
        depth = lambda l: 0 if l not in by_child else 1 + depth(by_child[l].find("parent").get("link"))  # noqa: E731
        tip_link = max((j.find("child").get("link") for j in by_child.values()
                        if j.get("type") in ("revolute", "continuous", "prismatic")), key=depth)
    chain, link = [], tip_link
    while link in by_child:
        chain.append(by_child[link])
        link = by_child[link].find("parent").get("link")
    chain = [j for j in reversed(chain) if j.get("type") in ("revolute", "continuous", "prismatic")]
    names, lo, hi, vel = [], [], [], []
    for el in chain:
        lim = el.find("limit")
        if lim is None or lim.get("velocity") is None:
            raise ValueError(f"{urdf}: joint {el.get('name')} has no <limit velocity>")
        names.append(el.get("name"))
        vel.append(float(lim.get("velocity")))
        lo.append(float(lim.get("lower", "-inf")))
        hi.append(float(lim.get("upper", "inf")))
    return {"names": names, "lower": np.array(lo), "upper": np.array(hi), "velocity": np.array(vel),
            "tip_link": tip_link, "parser": "tools.btexport.limits (xml)"}


def tighten(lower, upper, velocity, cfg: Optional[dict], where: str = "robots.yaml", atol: float = 1e-6,
            rtol: float = 1e-6) -> dict:
    """The URDF's position / velocity limits narrowed to a cell file's (``cfg``: {"lower", "upper", "velocity"}, any
    of them None) where the file's are tighter beyond its 6-7 digit rounding: per joint the larger lower, the smaller
    upper, the smaller velocity. -> {"lower", "upper", "velocity", "v_source", "q_source", "tightened": [names]}.
    A list of another length than the URDF's chain, or a non-positive / non-finite speed, is refused (ValueError)."""
    lo, hi, v = (np.array(x, float) for x in (lower, upper, velocity))
    out = {"v_source": "URDF <limit velocity>", "q_source": "URDF <limit lower upper>", "tightened": []}
    cfg = cfg or {}

    def get(key):
        x = cfg.get(key)
        if x is None:
            return None
        x = np.asarray(x, float).reshape(-1)
        if x.shape != v.shape or not np.all(np.isfinite(x)):
            raise ValueError(f"{where} {key} {x.tolist()}: not {v.size} finite values (the URDF chain's joints)")
        return x

    tight = lambda a, b: a < b - (atol + rtol * np.abs(b))  # noqa: E731  (a tighter than b beyond the rounding)
    cv = get("velocity")
    if cv is not None:
        if np.any(cv <= 0):
            raise ValueError(f"{where} joint_velocity_limits {cv.tolist()}: not > 0")
        m = tight(cv, v)
        if m.any():
            v = np.where(m, cv, v)
            out["v_source"] = f"min(URDF <limit velocity>, {where} joint_velocity_limits): {where} on joints " + \
                ", ".join(str(i + 1) for i in np.flatnonzero(m))
            out["tightened"].append("velocity")
    cl = get("lower")
    ch = get("upper")
    ml = tight(-cl, -lo) if cl is not None else np.zeros_like(lo, bool)
    mh = tight(ch, hi) if ch is not None else np.zeros_like(hi, bool)
    if ml.any() or mh.any():
        lo, hi = np.where(ml, cl if cl is not None else lo, lo), np.where(mh, ch if ch is not None else hi, hi)
        if np.any(lo >= hi):
            raise ValueError(f"{where} joint limits leave an empty range with the URDF's: {lo.tolist()} / {hi.tolist()}")
        out["q_source"] = f"URDF <limit lower upper> narrowed to {where} joint_limits_lower / _upper on joints " + \
            ", ".join(str(i + 1) for i in np.flatnonzero(ml | mh))
        out["tightened"].append("position")
    out.update(lower=lo, upper=hi, velocity=v)
    return out


def _acc_override(raw_acc, ctx: str):
    if raw_acc is None:
        return KR240_ACC_DEFAULT.copy(), "default KR240 [3,3,3,6,6,8] (umarell robots.yaml REVIEW value)", True
    a = np.asarray(raw_acc, float).reshape(6)
    review = bool(np.allclose(a, KR240_ACC_DEFAULT))
    return a, ctx + (" (= umarell's REVIEW default)" if review else ""), review


def _explicit_acc(accel):
    """``accel`` = None | [6] | {robot id or "*": [6]} (the CLI's --accel / an API override)."""
    if accel is None:
        return lambda rid: None
    if isinstance(accel, dict):
        return lambda rid: accel.get(rid, accel.get("*"))
    return lambda rid: accel


def cell_limits(cell_id: str, accel=None) -> dict:
    """{robot id: JointLimits} of a umarell cell, as ``env.cell.load_cell`` builds it (so a robot override --
    robotmodel.override / KWR_ROBOT_OVERRIDE / --urdf -- moves the exporter, the controllers and tools.retime to the
    same URDF and acceleration limits). Velocity and position limits: the URDF's ``<limit>`` (regardless of
    KWR_LIMITS_SOURCE), narrowed to robots.yaml's where those are tighter (``tighten``; not with a robot override,
    whose URDF replaces robots.yaml's copies); accelerations: ``accel`` (explicit) > override ``accel`` > robots.yaml
    ``joint_acceleration_limits`` > KR240 default."""
    import yaml
    from env.cell import load_cell
    c = load_cell(cell_id)
    rcfg = yaml.safe_load(open(c.root / "agentic_planner" / "config" / "robots.yaml"))["robots"]["arms"]
    ex = _explicit_acc(accel)
    out = {}
    for rid, rc in c.robots.items():
        cfg = rcfg.get(rid, {})
        u = urdf_chain_limits(rc.urdf, rc.tip_link or cfg.get("tip_link"))
        if ex(rid) is not None:
            a, src, rev = _acc_override(ex(rid), "explicit --accel override")
        elif rc.accel_source == "override":
            a, src, rev = _acc_override(rc.a_max, "robot override accel (robotmodel.override)")
        elif cfg.get("joint_acceleration_limits") is not None:
            a, src, rev = _acc_override(cfg["joint_acceleration_limits"],
                                        f"{cell_id}/agentic_planner/config/robots.yaml joint_acceleration_limits")
        else:
            a, src, rev = _acc_override(rc.a_max, f"robotmodel per-model default ({rc.accel_source})")
        own = c.robot_override is None
        t = tighten(u["lower"], u["upper"], u["velocity"],
                    {"lower": cfg.get("joint_limits_lower"), "upper": cfg.get("joint_limits_upper"),
                     "velocity": cfg.get("joint_velocity_limits")} if own else None,
                    f"{cell_id}/agentic_planner/config/robots.yaml {rid}")
        lim = JointLimits(rid, str(rc.urdf), u["names"], t["lower"], t["upper"], t["velocity"], a, src, rev,
                          v_copy=np.asarray(cfg["joint_velocity_limits"], float)
                          if cfg.get("joint_velocity_limits") and own else None,
                          v_source=t["v_source"], q_source=t["q_source"])
        if c.robot_override is not None:
            lim.notes.append(f"robot override active ({(c.robot_override or {}).get('label', 'unlabelled')}): "
                             f"URDF {_rel(rc.urdf)}")
        if not np.allclose(rc.a_max, a):
            lim.notes.append(f"env.cell a_max {np.round(rc.a_max, 6).tolist()} differs from the export's (explicit "
                             f"--accel): the controllers ran with env.cell's")
        out[rid] = lim
    return out


def scene_limits(scene_path: str | Path, accel=None) -> dict:
    """{robot id: JointLimits} of a trajgen scene.yaml (trajgen.scene.load_scene reads the URDF; joints may omit
    names / limits). Velocity / position: the URDF; accelerations: ``accel`` > scene.yaml ``joints.acceleration`` >
    robotmodel's per-model default (trajgen.scene's ``limits_info`` source)."""
    from trajgen.scene import load_scene
    sc = load_scene(scene_path)
    sp = Path(scene_path)
    ex = _explicit_acc(accel)
    out = {}
    for r in sc.robots:
        u = urdf_chain_limits(r.urdf, r.tool.tip_link)
        info = getattr(r, "limits_info", {}) or {}
        if ex(r.id) is not None:
            a, src, rev = _acc_override(ex(r.id), "explicit --accel override")
        elif info.get("acceleration") == "scene.yaml":
            a, src, rev = _acc_override(r.a_max, f"{_rel(sp)} robots[{r.id}].joints.acceleration")
        else:
            a, src, rev = _acc_override(r.a_max, f"robotmodel per-model default ({info.get('acceleration', '?')})")
        lim = JointLimits(r.id, str(r.urdf), u["names"], u["lower"], u["upper"], u["velocity"], a, src, rev,
                          v_copy=np.asarray(r.v_max, float))
        if list(u["names"]) != list(r.joint_names):
            lim.notes.append(f"scene joint names {r.joint_names} != URDF chain {u['names']}")
        if info.get("velocity") and "overrides URDF" in str(info.get("velocity")):
            lim.notes.append("scene.yaml joints.velocity overrides the URDF (trajgen planned with it); the export "
                             "checks against the URDF")
        out[r.id] = lim
    return out


def limits_for(cell_id: Optional[str] = None, scene: Optional[str | Path] = None, accel=None) -> dict:
    """Scene first (any number of robots), else the umarell cell."""
    if scene is not None:
        return scene_limits(scene, accel)
    return cell_limits(cell_id, accel)


def stack(lims: dict, robot_ids) -> tuple[np.ndarray, np.ndarray]:
    """(v_max (R, 6), a_max (R, 6)) in ``robot_ids`` order."""
    return (np.stack([lims[r].v_max for r in robot_ids]), np.stack([lims[r].a_max for r in robot_ids]))



def _md5(p) -> Optional[str]:
    import hashlib
    p = Path(p)
    return hashlib.md5(p.read_bytes()).hexdigest() if p.is_file() else None


def inputs_fingerprint(cell_id: Optional[str] = None, scene: Optional[str | Path] = None) -> dict:
    """md5 of every file that defines the robots / limits / task of a cell or a trajgen scene: the URDF (and SRDF)
    of every robot (after a robot override), robots.yaml / cell.yaml / obstacles.yaml / the task (cells), scene.yaml
    / task.json (scenes). Part of the cache keys of trajgen runs and tools.retime results, so an in-place edit of a
    URDF or a scene re-plans / re-times instead of reusing a stale result."""
    out = {}
    if scene is not None:
        sp = Path(scene)
        sd = sp.parent if sp.is_file() else sp
        out["scene.yaml"] = _md5(sd / "scene.yaml")
        out["task.json"] = _md5(sd / "task.json")
        try:
            from trajgen.scene import load_scene
            for r in load_scene(sd / "scene.yaml").robots:
                out[f"{r.id}.urdf"] = _md5(r.urdf)
                if r.srdf is not None:
                    out[f"{r.id}.srdf"] = _md5(r.srdf)
        except Exception as e:  # noqa: BLE001  (an unloadable scene fails later, loudly)
            out["scene_error"] = repr(e)
        return out
    from env.cell import load_cell
    c = load_cell(cell_id)
    ap = c.root / "agentic_planner"
    for k in ("config/robots.yaml", "config/obstacles.yaml", "cell.yaml", "tasks/car_door_spot_welding.json"):
        out[k] = _md5(ap / k)
    for rid, rc in c.robots.items():
        out[f"{rid}.urdf"] = _md5(rc.urdf)
    from robotmodel.override import active_override, dumps
    out["robot_override"] = dumps(active_override())
    return out


# ───────────────────────────────────────── robot model of a plan (P3U round 2) ─────────────────────────────────────────
GEOMETRY_KEYS = ("urdf", "base", "yaw", "tip_link", "tcp_offset", "home")


def _vec(x, n=6):
    return None if x is None else [round(float(v), n) for v in np.asarray(x, float).reshape(-1)]


def _kin_md5(p):
    try:
        from robotmodel.urdf_robot import kinematics_md5
        return kinematics_md5(p)
    except Exception:  # noqa: BLE001
        return None


def _robot_entry(rc, tcp_source: str) -> dict:
    return {"urdf": _rel(rc.urdf), "urdf_md5": _md5(rc.urdf), "kinematics_md5": _kin_md5(rc.urdf),
            "base": _vec(rc.base), "yaw": round(float(rc.yaw), 9),
            "tip_link": rc.tip_link, "tcp_offset": _vec(rc.tcp_offset, 7), "tcp_source": tcp_source,
            "home_rad": _vec(rc.home, 7), "a_max_rad_s2": _vec(rc.a_max), "a_source": rc.accel_source}


def override_effect(cell_id: str) -> Optional[dict]:
    """What the active robot override (robotmodel.override) actually changes in umarell cell ``cell_id``, per robot,
    against the cell's own files: URDF (content md5), base, yaw, tip_link, tcp_offset, home, acceleration limits.
    None without an override. ``unknown_robots``: override robot ids that are neither "*" nor in the cell (a typo:
    the CLI refuses them). ``geometry_changed``: the URDF / base / TCP / home differ, so umarell's validators (which
    load the cell's own robot) do not apply; an acceleration-only override leaves the geometry unchanged."""
    import os
    from env.cell import _load_cell, load_cell
    from robotmodel.override import active_override
    ov = active_override()
    if ov is None:
        return None
    own = _load_cell(cell_id, "", os.environ.get("KWR_LIMITS_SOURCE", "auto"))
    cur = load_cell(cell_id)
    out = {"label": ov.get("label"), "spec": ov,
           "unknown_robots": sorted(set((ov.get("robots") or {})) - {"*"} - set(own.robots)),
           "cell_robots": sorted(own.robots), "robots": {}}
    for rid, a in own.robots.items():
        b = cur.robots[rid]
        ch = {}
        if _md5(a.urdf) != _md5(b.urdf):
            from robotmodel.urdf_robot import kinematics_md5
            same_kin = kinematics_md5(a.urdf) == kinematics_md5(b.urdf)
            ch["urdf_limits_only" if same_kin else "urdf"] = {"from": _rel(a.urdf), "to": _rel(b.urdf),
                                                              "md5": _md5(b.urdf), "kinematics_md5": kinematics_md5(b.urdf)}
        if not np.allclose(a.base, b.base, atol=1e-12):
            ch["base"] = {"from": _vec(a.base), "to": _vec(b.base)}
        if abs(float(a.yaw) - float(b.yaw)) > 1e-12:
            ch["yaw"] = {"from": float(a.yaw), "to": float(b.yaw)}
        if a.tip_link != b.tip_link:
            ch["tip_link"] = {"from": a.tip_link, "to": b.tip_link}
        if not np.allclose(a.tcp_offset, b.tcp_offset, atol=1e-12):
            ch["tcp_offset"] = {"from": _vec(a.tcp_offset, 7), "to": _vec(b.tcp_offset, 7)}
        if not np.allclose(a.home, b.home, atol=1e-12):
            ch["home"] = {"from": _vec(a.home, 7), "to": _vec(b.home, 7)}
        if not np.allclose(a.a_max, b.a_max):
            ch["accel"] = {"from": _vec(a.a_max), "to": _vec(b.a_max)}
        for k, x, y in (("v_max", a.v_max, b.v_max), ("lower", a.q_lo, b.q_lo), ("upper", a.q_hi, b.q_hi)):
            if not np.allclose(x, y):
                ch[k] = {"from": _vec(x), "to": _vec(y)}
        if ch:
            ch["geometry_changed"] = any(k in ch for k in GEOMETRY_KEYS)
            out["robots"][rid] = ch
    out["geometry_changed"] = any(e.get("geometry_changed") for e in out["robots"].values())
    out["applied"] = bool(out["robots"])
    return out


def robot_model(src) -> dict:
    """``behaviour_tree.metadata.robot_model``: the robot model the plan assumes, per robot (URDF path + md5, base,
    yaw, tip_link, tcp_offset and its source, home, accelerations), plus the robot override actually applied (umarell
    cells) -- so a plan made for another URDF / base cannot be mistaken for one of the cell's own robots
    (tools.btexport.umarell_check compares it with the cell)."""
    out = {"robots": {}, "override": None}
    if src.scene is not None:
        from trajgen.scene import load_scene
        sc = load_scene(src.scene)
        for r in sc.robots:
            if r.id not in src.robot_ids:
                continue
            out["robots"][r.id] = {"urdf": _rel(r.urdf), "urdf_md5": _md5(r.urdf), "kinematics_md5": _kin_md5(r.urdf),
                                   "base": _vec(r.base_xyz),
                                   "yaw": round(float(r.yaw), 9), "tip_link": getattr(r.tool, "tip_link", None),
                                   "tcp_source": f"{_rel(src.scene)} robots[{r.id}].tool", "home_rad": _vec(r.home, 7)}
        out["source"] = f"trajgen scene {_rel(src.scene)}"
        return out
    from env.cell import load_cell
    c = load_cell(src.cell_id)
    eff = override_effect(src.cell_id)
    for rid in src.robot_ids:
        rc = c.robots[rid]
        e = (eff or {}).get("robots", {}).get(rid, {})
        tsrc = getattr(rc, "tcp_source", None) or (
            "robot override tcp_offset" if "tcp_offset" in e else "robots.yaml tcp_offset")
        out["robots"][rid] = _robot_entry(rc, tsrc)
    out["source"] = f"umarell cell {src.cell_id}" + (" + robot override" if eff and eff["applied"] else "")
    if eff is not None:
        out["override"] = {k: eff[k] for k in ("label", "applied", "geometry_changed", "robots", "unknown_robots")}
    return out
