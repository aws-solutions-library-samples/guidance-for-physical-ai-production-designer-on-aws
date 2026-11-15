"""Full-speed profile (phase 3, track TGFULL, docs/p3_tgfull.md): ``--speed full`` / ``planning.speed_profile: full``.

Every motion that is not a welding operation (joint transits, parking / yield legs, the return to HOME) is timed at
velocity scale 1.0 of the URDF joint velocity limits (``<limit velocity>``, read with ``robotmodel.parse_urdf``),
with the scene's joint acceleration limits (``robots[i].joints.acceleration``; KR240 R2900 default
[3, 3, 3, 6, 6, 8] rad/s^2 = umarell's REVIEW value, not measured) at scale 1.0 and a jerk limit (default 10 a_max
per second, the soft hardware reference of docs/p3_ctrl.md, as tools/retime's ``full`` profile). Every segment is
the jerk-limited rest-to-rest S-curve of ``trajgen.motion.SCurve``, so the speed ramps up from rest (or from the end
of a slow weld stroke, where the arm is at rest) to full speed and back down: there is no instantaneous switch
between a slow and a fast segment.

The welding operations keep umarell's slow rule exactly as in ``--speed umarell``: the straight descent within the
approach offset (phase ``approach``), the hold, and the straight lift-off (phase ``retract``) at
``velocity_scale * approach_velocity_fraction`` (0.28 v_max) with acceleration ``accel_rule(that)`` (0.078 a_max)
and the scene's jerk factor (``planning.max_jerk_factor``, else 4 a_max/s).

Scene keys (all optional; trajgen.scene ignores keys it does not know, they are read here from ``scene.raw``)::

    planning:
      speed_profile: full            # umarell (default) | fast | full; the CLI --speed overrides it
      full_speed:                    # only used by the full profile
        velocity_scale: 1.0          # fraction of the URDF velocity limit for non-weld motion (default 1.0)
        acceleration_scale: 1.0      # fraction of joints.acceleration (default 1.0)
        jerk_factor: 10.0            # jerk limit in a_max per second (default 10; --jerk-factor overrides)

The coordination (steps / priority), the lateness gate, the dense verification and the continuous certificate all
run on the legs timed with these caps, i.e. on the real full-speed timing.
"""
from __future__ import annotations

import logging

import numpy as np

PROFILES = ("umarell", "fast", "full")
FULL_JERK_FACTOR = 10.0
WELD_PHASES = ("approach", "retract")
MOVE_PHASES = ("transit", "home")
log = logging.getLogger("trajgen.fullspeed")


def _planning_raw(scene) -> dict:
    return dict((getattr(scene, "raw", None) or {}).get("planning") or {})


def scene_profile(scene) -> str | None:
    """``planning.speed_profile`` of the scene (None if absent)."""
    p = _planning_raw(scene).get("speed_profile")
    if p is None:
        return None
    p = str(p).strip().lower()
    if p not in PROFILES:
        from trajgen.scene import SceneError
        raise SceneError(f"planning.speed_profile must be one of {PROFILES}, got {p!r}")
    return p


def resolve_speed(scene, speed: str | None) -> str:
    """The speed mode of a run: the explicit ``speed`` (CLI / API) if given, else the scene's
    ``planning.speed_profile``, else "umarell"."""
    if speed is not None:
        s = str(speed).strip().lower()
        if s not in PROFILES:
            raise ValueError(f"unknown speed mode {speed!r} (one of {PROFILES})")
        return s
    return scene_profile(scene) or "umarell"


def full_options(scene) -> dict:
    fs = dict(_planning_raw(scene).get("full_speed") or {})
    o = {"velocity_scale": float(fs.get("velocity_scale", 1.0)),
         "acceleration_scale": float(fs.get("acceleration_scale", 1.0)),
         "jerk_factor": None if fs.get("jerk_factor") is None else float(fs["jerk_factor"])}
    for k in ("velocity_scale", "acceleration_scale"):
        if not (0.0 < o[k] <= 1.0):
            from trajgen.scene import SceneError
            raise SceneError(f"planning.full_speed.{k} must be in (0, 1], got {o[k]}")
    if o["jerk_factor"] is not None and not o["jerk_factor"] > 0:
        from trajgen.scene import SceneError
        raise SceneError("planning.full_speed.jerk_factor must be > 0")
    return o


def urdf_velocity(robot) -> tuple[np.ndarray, str]:
    """(6,) joint velocity limits from the robot's URDF ``<limit velocity>`` (robotmodel's parser, the project's
    only URDF parser), in the order of ``robot.joint_names``; falls back to ``robot.v_max`` (which trajgen.scene
    already reads from / checks against the URDF) if a joint carries no velocity attribute."""
    try:
        from robotmodel import parse_urdf
        u = parse_urdf(str(robot.urdf))
        by = {j.name: getattr(j, "velocity", None) for j in u.joints}
        v = [by.get(n) for n in robot.joint_names]
        if all(x is not None and np.isfinite(x) and x > 0 for x in v):
            return np.asarray(v, float), f"URDF <limit velocity> ({robot.urdf.name})"
    except Exception as e:  # noqa: BLE001
        log.warning("%s: URDF velocity limits not readable (%s); using the scene's joints.velocity", robot.id, e)
    return np.asarray(robot.v_max, float).copy(), f"scene robots.{robot.id}.joints.velocity ({robot.limits_info.get('velocity')})"


def full_speeds(scene, rid, jerk_factor: float | None = None):
    """``trajgen.schedule.Speeds`` of the full profile for robot ``rid`` (module docstring)."""
    from trajgen.schedule import Speeds
    pl = scene.planning
    r = scene.robot(rid)
    o = full_options(scene)
    v_urdf, v_src = urdf_velocity(r)
    # the URDF limit itself where the scene's value matches it (trajgen.scene's tolerance for hand-copied numbers,
    # 1e-6 abs + 1e-6 rel: e.g. 10.73377 vs the URDF's 10.7337749); a scene value that really differs is a deliberate
    # override (scene.py warns): the smaller of the two is used then
    v_sc = np.asarray(r.v_max, float)
    differ = np.abs(v_urdf - v_sc) > 1e-6 + 1e-6 * np.abs(v_urdf)
    v_lim = np.where(differ, np.minimum(v_urdf, v_sc), v_urdf)
    if differ.any():
        log.warning("%s: scene joints.velocity %s differs from the URDF %s; full speed uses the smaller value",
                    rid, list(v_sc), list(v_urdf))
        v_src += f"; joints {np.flatnonzero(differ).tolist()}: min(URDF, scene override)"
    va = pl.approach_vscale
    jf_weld = pl.max_jerk_factor or 4.0           # the weld strokes exactly as in --speed umarell
    jf_move = jerk_factor if jerk_factor is not None else (o["jerk_factor"] or FULL_JERK_FACTOR)
    a_max = np.asarray(r.a_max, float)
    # weld strokes: the same limit as the transit (where the scene's copy matches the URDF its own value, so the
    # committed plans are bit-identical; a differing joint (legacy TRAJGEN_SCENE_LIMITS=scene override) the smaller)
    v_weld = np.where(differ, v_lim, v_sc)
    v = {"transit": v_lim * o["velocity_scale"], "home": v_lim * o["velocity_scale"], "approach": v_weld * va}
    a = {"transit": a_max * o["acceleration_scale"], "home": a_max * o["acceleration_scale"],
         "approach": a_max * pl.accel_scale(va)}
    j = {"transit": a_max * jf_move, "home": a_max * jf_move, "approach": a_max * jf_weld}
    v["retract"], a["retract"], j["retract"] = v["approach"], a["approach"], j["approach"]
    a_src = str(r.limits_info.get("acceleration", "scene.yaml"))
    review = bool(np.allclose(a_max, [3, 3, 3, 6, 6, 8])) or "REVIEW" in a_src
    info = {"profile": "full",
            "vscale": {"transit": o["velocity_scale"], "approach": va, "home": o["velocity_scale"]},
            "accel_frac": {"transit": o["acceleration_scale"], "approach": pl.accel_scale(va),
                           "home": o["acceleration_scale"]},
            "jerk_factor": jf_move, "jerk_factor_weld": jf_weld,
            "v_limit_rad_s": [round(float(x), 7) for x in v_lim], "v_source": v_src,
            "a_limit_rad_s2": [float(x) for x in a_max],
            "a_source": a_src + (" - REVIEW: umarell's default [3, 3, 3, 6, 6, 8], not a measured KUKA value"
                                 if review else ""),
            "j_limit_rad_s3": [float(x) for x in a_max * jf_move],
            "weld_phases": "approach (straight descent within the approach offset) and retract (straight lift-off) "
                           "keep umarell's slow rule; the hold is a stand-still"}
    return Speeds(v, a, j, "full", info)


# ------------------------------------------------------------------------------------------- dense dynamics check
def _pad(L, n: int = 3):
    P = np.asarray(L.positions, float)
    ph = np.asarray(L.phase).astype(str)
    return (np.vstack([np.repeat(P[:1], n, 0), P, np.repeat(P[-1:], n, 0)]),
            np.concatenate([[ph[0]] * n, ph, [ph[-1]] * n]))


def speed_profile_check(legs, speeds: dict, dt: float, v_limit: dict | None = None, a_limit: dict | None = None,
                        tol_v: float = 1e-6, tol_a: float = 0.02, tol_j: float = 0.02) -> dict:
    """Peak joint velocity / acceleration / jerk per phase class of the executed 20 Hz waypoints (each leg padded
    with rest samples: the arm starts and ends every leg at rest), as fractions of the robot's limits (URDF v,
    scene a, jerk cap) and of the phase caps the profile planned with.

    v = first, a = second, j = third differences of the samples (h = dt): bounds of the continuous S-curve's
    |v| <= V, |a| <= A, |j| <= J (mean-value property of finite differences). Classes: ``move`` (transit, home;
    the 'wait' stand-stills count here), ``weld`` (approach descent, lift-off). ``weld_slow``: every weld-phase
    sample interval within the slow weld caps. Passes: v <= cap (1e-6), a <= cap (2 %), jerk of the move class
    <= cap (2 %; the weld strokes' jerk is reported only: their joint paths are curved Cartesian lines, whose
    curvature terms the per-segment jerk cap does not bound)."""
    out = {}
    worst = {"move": {"v": 0.0, "a": 0.0, "j": 0.0}, "weld": {"v": 0.0, "a": 0.0, "j": 0.0}}
    frac = {"move": {"v": 0.0, "a": 0.0, "j": 0.0}, "weld": {"v": 0.0, "a": 0.0, "j": 0.0}}
    at = {}
    weld_ok = True
    for L in legs:
        if len(L.positions) < 2:
            continue
        sp = speeds[L.robot]
        vl = (v_limit or {}).get(L.robot, sp.v["transit"])
        al = (a_limit or {}).get(L.robot, sp.a["transit"])
        jl = sp.j["transit"]
        Q, ph = _pad(L)
        cls = np.array(["weld" if p in WELD_PHASES else "move" for p in ph])
        capv = np.stack([sp.v.get(p, sp.v["transit"]) if p != "wait" else sp.v["transit"] for p in ph])
        capa = np.stack([sp.a.get(p, sp.a["transit"]) if p != "wait" else sp.a["transit"] for p in ph])
        capj = np.stack([sp.j.get(p, sp.j["transit"]) if p != "wait" else sp.j["transit"] for p in ph])
        v = np.diff(Q, axis=0) / dt
        a = np.diff(Q, 2, axis=0) / dt ** 2
        j = np.diff(Q, 3, axis=0) / dt ** 3
        for name, X, w in (("v", v, 1), ("a", a, 2), ("j", j, 3)):
            cap = {"v": capv, "a": capa, "j": capj}[name]
            lim = {"v": vl, "a": al, "j": jl}[name]
            for k in range(len(X)):
                if name == "v":                 # interval k -> k+1 belongs to the segment sample k+1 ends
                    key, cc = cls[k + 1], cap[k + 1]
                else:                           # spans samples k..k+w: weld only if every later sample is weld;
                    idx = list(range(k, k + w + 1))     # mixed (only at rest junctions): the larger cap
                    key = "weld" if all(cls[i] == "weld" for i in idx[1:]) else "move"
                    cc = np.max(np.stack([cap[i] for i in idx]), 0)
                r = float((np.abs(X[k]) / cc).max())
                if r > worst[key][name]:
                    worst[key][name] = r
                    at[f"{key}_{name}"] = (L.robot, L.op_id, str(ph[min(k + 1, len(ph) - 1)]))
                frac[key][name] = max(frac[key][name], float((np.abs(X[k]) / lim).max()))
                if name == "v" and key == "weld" and r > 1.0 + tol_v:
                    weld_ok = False
    rnd = lambda d: {k: round(x, 4) for k, x in d.items()}         # noqa: E731
    out = {"over_phase_cap": {k: rnd(v) for k, v in worst.items()},
           "fraction_of_limits": {k: rnd(v) for k, v in frac.items()},
           "at": at, "weld_slow": bool(weld_ok),
           "note": "20 Hz waypoint differences (v: 1st, a: 2nd, j: 3rd, h = dt), legs padded with rest samples; "
                   "fraction_of_limits: v / URDF v_max, a / scene a_max, j / the full-speed jerk cap"}
    out["pass"] = bool(weld_ok and worst["move"]["v"] <= 1 + tol_v and worst["weld"]["v"] <= 1 + tol_v
                       and worst["move"]["a"] <= 1 + tol_a and worst["weld"]["a"] <= 1 + tol_a
                       and worst["move"]["j"] <= 1 + tol_j)
    return out


def dense_segment_check(seg, hz: float = 1000.0) -> dict:
    """Continuous-profile check of one ``trajgen.motion.Segment`` (its own sample() at ``hz``, central
    differences): peak |v| / v_cap, |a| / a_cap, |j| / j_cap over the joints. Used by the tests."""
    if seg.prof is None or seg.T <= 0:
        return {"v": 0.0, "a": 0.0, "j": 0.0}
    h = 1.0 / hz
    ts = np.arange(-3 * h, seg.T + 3 * h + 1e-12, h)
    q = seg.sample(ts)                                      # clipped at the ends = rest before / after
    v = np.diff(q, axis=0) / h
    a = np.diff(q, 2, axis=0) / h ** 2
    j = np.diff(q, 3, axis=0) / h ** 3
    jc = seg.j_cap if seg.j_cap is not None else np.full(6, np.inf)
    return {"v": float((np.abs(v) / seg.v_cap).max()), "a": float((np.abs(a) / seg.a_cap).max()),
            "j": float((np.abs(j) / jc).max())}
