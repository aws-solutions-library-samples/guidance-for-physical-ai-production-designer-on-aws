"""Worker-process functions of the umarell-fast orchestrator (shared by U2 / U3).

Every function here runs in a worker process that was initialised with :func:`init` (umarell imported from the
variant's working copy, a planning session opened exactly as ``agents/trajectory/agent.py:_plan`` opens one, the
collision kernels installed, no LLM reachable). They call umarell's own tools through ``.func`` -- the same code
the LLM agent calls -- and only return plain data, so the parent can cache the trajectories in its own session and
assemble the tree with umarell's assembler.

Chaining state is INJECTED per call instead of being left to whatever the worker planned before:
``trajectory_cache.set_last_config`` = where the arm actually is (the END of its previous trajectory, so every
leg chains by construction; umarell itself records the IK goal there, which differs from the descent's end by up
to 0.03 rad and is the source of the chain breaks the LLM spent ~30 % of its turns repairing), and
``set_last_pose`` = the working pose the planner RECORDED for the arm's previous leg (so it performs its on-axis
lift-off, at the orientation that leg really ended in -- a weld whose pose the planner spun / tilted (fix 12) lifts
off at that pose). Every planning task starts from a cleared session (trajectory cache and chaining state), so
nothing a worker planned for another task can leak into it.

umarell 76ea13a keys more on the chaining state: every weld's pose (``set_weld_pose``: a leg's lift-off is matched
by where its TCP is, ``seam_at_start``), every operation's specified stroke (``set_op_spec``: the assembler checks
each weld leg against it) and home refusals (recorded by the home planning itself, in the process that assembles).
A planned leg returns its records (``weld_pose``, ``op_spec``, ``last_pose``); the orchestrator hands an arm's
records of its earlier legs to every task that plans from them (``welds`` / ``specs``, loaded with
``trajectory_cache.restore``) and all of them to the assembly.
"""
from __future__ import annotations

import os
import time
from typing import Optional

_INIT: dict = {}


def init(repo: str, cell: str, env: dict, kernels: str, seed: Optional[int] = None) -> None:
    """Process initializer: open the umarell session (idempotent)."""
    if _INIT.get("ready"):
        return
    from umarell_fast.u2 import session as S
    if seed is not None:
        # one stream per worker; OMPL_SEED is honoured by patch U3_02 at OMPL import time
        seed = int(seed) * 1009 + (os.getpid() % 1000)
        env = {**env, "OMPL_SEED": str(seed % (2 ** 31 - 1) or 1)}
    S.setup(repo, cell, env, kernels="none", seed=seed)
    orig = install_kernels(kernels)
    _INIT.update(ready=True, cell=cell, kernels=kernels, pid=os.getpid(), orig=orig)


def kernel_selfcheck(cell: str, n: int = 40, max_s: float = 15.0) -> dict:
    """Self-check of this worker's installed kernels against the originals it replaced (umarell_fast.kernel_check);
    run in a worker so it overlaps with the planning instead of delaying it."""
    from umarell_fast import kernel_check
    t0 = time.monotonic()
    orig = _INIT.get("orig") or {}
    if not orig:
        return {"ok": True, "checked": 0, "note": "no kernels installed", "pid": os.getpid(), "wall_s": 0.0}
    try:
        res = kernel_check.selfcheck(cell, orig, n=n, max_s=max_s)
    except Exception as exc:  # noqa: BLE001
        res = {"ok": False, "error": repr(exc)}
    res.update(pid=os.getpid(), wall_s=round(time.monotonic() - t0, 3), kernels=_INIT.get("kernels"))
    return res


KERNELS = ("none", "np", "npplus", "fix", "u1", "x76")
# bases whose own collision checks already are x76's (13d446b, 08d5a39 and the public 8a02db9): x76 is a no-op there
X76_UPSTREAM = ("08d5a39", "8a02db9")


def install_kernels(kernels: str) -> dict:
    """Install a collision-kernel set into this process's umarell session; returns the originals.

    ``np`` / ``npplus``: umarell_fast.kernels_np (profile track; umarell HEAD predicates; npplus adds the diagnosis
    fast path + parse-once YAML). ``fix``: umarell_fast.u2.kernels_fix (the same for a copy with upstream fix 03).
    ``u1``: umarell_fast.u1.kernels (own numpy FK, both predicate variants, also a batched
    ``inter_robot_clearance``) + parse-once YAML. ``none``: umarell's own Python checks.
    np / npplus / fix / u1 mirror umarell eccb1bc's predicate and refuse to install on 76ea13a. ``x76``
    (umarell_fast.kernels_76ea13a, 76ea13a only): umarell's own predicate with its moving-obstacle loop vectorised
    (exactly equal) and a parse-once cell manifest.
    """
    if kernels == "none":
        return {}
    if kernels in ("np", "npplus"):
        from umarell_fast import kernels_np
        return kernels_np.install(plus=(kernels == "npplus"))
    if kernels == "fix":
        from umarell_fast.u2 import kernels_fix
        return kernels_fix.install()
    if kernels == "x76":
        if umarell_base() in X76_UPSTREAM:
            # umarell vectorised / compiled exactly what x76 speeds up (its _moving_hit, the Numba pair kernel, the
            # YAML parse caches); x76's source guards refuse there. umarell's own predicate is the x76 predicate.
            print(f"[uf] x76: not installed on umarell {umarell_base()} (umarell's own checks are the fast ones)",
                  flush=True)
            return {}
        from umarell_fast import kernels_76ea13a
        return kernels_76ea13a.install()
    if kernels == "u1":
        from umarell_fast.u1 import kernels as k1
        from umarell_fast import fastio
        orig = k1.install()
        fastio.install()
        return orig
    raise ValueError(f"unknown kernels {kernels!r}; one of {KERNELS}")


def uninstall_kernels(kernels: str, orig: dict) -> None:
    """Put umarell's own collision functions back in this process (``orig`` = what :func:`install_kernels`
    returned); used when a kernel self-check fails and the run falls back to the originals."""
    if kernels == "none" or not orig:
        return
    if kernels == "x76":
        from umarell_fast import kernels_76ea13a
        kernels_76ea13a.uninstall()
        return
    if kernels == "u1":
        from umarell_fast.u1 import kernels as k1
        k1.uninstall()
        return
    from umarell_fast import kernel_check
    kernel_check.restore(orig)


def umarell_base() -> str:
    """The base of the umarell this process imported (session.setup records it from UF_APPLIED.json)."""
    from umarell_fast.u2 import session as S
    return S._STATE.get("base") or "eccb1bc"


def is_head() -> bool:
    """umarell 76ea13a (or later): planners in backends/, weld poses / op specs / home refusals in the cache."""
    return umarell_base() != "eccb1bc"


def ping() -> dict:
    return {"pid": os.getpid(), **{k: v for k, v in _INIT.items() if k not in ("ready", "orig")}}


# ─── state injection ─────────────────────────────────────────────────────────

def reset_session(cell: str) -> None:
    """Forget every trajectory and all chaining state of ``cell`` in this process (a fresh planning session)."""
    from agents.trajectory.tools import trajectory_cache as tc
    tc.clear_trajectory_cache(cell)
    tc.reset_robot_configs(cell)


def inject(cell: str, ns: str, start: Optional[list], prev_pose: Optional[dict],
           welds: Optional[dict] = None, specs: Optional[dict] = None) -> None:
    """One arm's chaining state: where it stands, the working pose it stands in, and (76ea13a) the weld poses /
    operation specs of its earlier legs (``{op_id: record}`` as returned by :func:`leg_records`)."""
    from agents.trajectory.tools import trajectory_cache as tc
    if start is not None:
        tc.set_last_config(cell, ns, list(start))
    if prev_pose and prev_pose.get("orientation"):
        tc.set_last_pose(cell, ns, list(prev_pose["target"]), list(prev_pose["approach_vector"]),
                         dict(prev_pose["orientation"]))
    else:
        tc.clear_last_pose(cell, ns)
    if welds or specs:
        if not is_head():
            raise RuntimeError("weld-pose / op-spec records are umarell 76ea13a state; this is eccb1bc")
        tc.restore(cell, {"weld_poses": [{"robot_ns": ns, "operation_id": op, "pose": pose}
                                         for op, pose in (welds or {}).items() if pose],
                          "op_specs": [{"robot_ns": ns, "operation_id": op, "spec": spec}
                                       for op, spec in (specs or {}).items() if spec]})


def leg_records(cell: str, ns: str, op_id: str) -> dict:
    """What the planner recorded for a leg it just shipped: the working pose the arm now stands in
    (``last_pose``; None after a plain move) and, on 76ea13a, the operation's weld pose and specified stroke."""
    from agents.trajectory.tools import trajectory_cache as tc
    out = {"last_pose": tc.get_last_pose(cell, ns)}
    if is_head():
        out["weld_pose"] = tc.chain_state(cell, ns, op_id).get("weld_pose")
        out["op_spec"] = tc.op_spec(cell, ns, op_id)
    return out


def leg_pose(req: dict) -> Optional[dict]:
    """The working pose a request leaves the arm in (what the planner records with ``set_last_pose``), or None
    for a plain move (no orientation constraint)."""
    import numpy as np
    ori = req.get("orientation") or {}
    if not (req.get("constrain_orientation") and ori):
        return None
    t = req["target"]
    av = np.asarray(req.get("approach_vector", [0, 0, -1]), float)
    av = av / (np.linalg.norm(av) + 1e-9)
    return {"target": [float(t["x"]), float(t["y"]), float(t["z"])], "approach_vector": av.tolist(),
            "orientation": dict(ori)}


# ─── prediction of where a leg ends (IK only) ─────────────────────────────────

def predict_end(cell: str, ns: str, start: list, req: dict, approach_offset_m: float = 0.1) -> Optional[list]:
    """Where ``plan_trajectories_ompl`` will leave the arm for ``req`` started at ``start``, by IK alone.

    Mirrors the planner: the approach waypoint is solved warm-started from the chained start; an oriented weld then
    descends in 6 straight-line IK steps warm-started along the way (``_cartesian_descent``) and ends at the last
    of them; a plain move ends at the target IK (the end of its OMPL segment). Exact as long as the planner's
    transit segment reaches the approach waypoint (it is non-fatal for it not to); the orchestrator verifies every
    leg's real end against the prediction and re-plans the dependants when they differ.
    """
    import numpy as np
    from agents.trajectory.tools.pinocchio import run_ik
    ik = getattr(run_ik, "func", run_ik)
    t = req["target"]
    x, y, z = float(t["x"]), float(t["y"]), float(t["z"])
    ori = req.get("orientation") or {}
    io = {}
    if req.get("constrain_orientation") and ori:
        io = {"qx": float(ori.get("x", 0.0)), "qy": float(ori.get("y", 0.707)), "qz": float(ori.get("z", 0.0)),
              "qw": float(ori.get("w", 0.707)), "constrain_orientation": True}
    if not io:
        r = ik(cell, ns, x, y, z, warm_start_joints=list(start))
        return list(r["joint_positions"]) if r.get("success") else None
    av = np.asarray(req.get("approach_vector", [0, 0, -1]), float)
    av = av / (np.linalg.norm(av) + 1e-9)
    try:
        off = float(req.get("approach_offset_m", approach_offset_m))
    except (TypeError, ValueError):
        off = approach_offset_m
    if not off > 0.0:
        off = approach_offset_m
    goal = np.array([x, y, z])
    ap = goal - av * off
    # Upstream fix 12 (IK spin/tilt): the planner may weld with a spun / tilted orientation when the exact quaternion
    # self-collides; ask it the same question so the predicted end matches (a different pick only costs the
    # orchestrator's re-plan: the real end is verified).
    o = predicted_weld_orientation(cell, ns, start, req, off)
    if o is not None:
        io.update(qx=float(o["x"]), qy=float(o["y"]), qz=float(o["z"]), qw=float(o["w"]))
    r = ik(cell, ns, *map(float, ap), warm_start_joints=list(start), **io)
    if not r.get("success"):
        return None
    warm = r["joint_positions"]
    for i in range(1, 7):
        a = i / 6
        p = ap * (1.0 - a) + goal * a
        r = ik(cell, ns, *map(float, p), warm_start_joints=warm, **io)
        if not r.get("success"):
            return None
        warm = r["joint_positions"]
    return list(warm)


def predicted_weld_orientation(cell: str, ns: str, start: list, req: dict, off: float) -> Optional[dict]:
    """The weld orientation the planner will use for ``req`` started at ``start`` when it is not the specified
    one, else None.

    * 76ea13a: the facade picks it before dispatch (``planner_backend._auto_weld_orientation`` ->
      ``tools/weld_orientation.search``, judged at the stroke the backend plans with), after ``apply_tool_spin``;
    * eccb1bc + fix 12: ``ompl_planner.weld_orientation_for_request`` (the planner's fixture predicate adds the
      approach-phase speed margin, unknown here: 0.0).
    A base whose API is missing raises (no silent fallback to the specified pose)."""
    if is_head():
        from agents.trajectory.tools import planner_backend as pb
        r = pb.apply_tool_spin(dict(req, robot_ns=ns, current_joints=list(start)))
        if r.get("approach_offset_m") is None:
            r["approach_offset_m"] = off
        _new, chosen, _rep = pb._auto_weld_orientation(cell, r, off)
        return dict(chosen["orientation"]) if chosen else None
    from agents.trajectory.tools import ompl_planner as _op
    if not hasattr(_op, "weld_orientation_for_request"):
        return None                         # eccb1bc without fix 12: the planner never changes the pose
    try:
        if not _op._weld_free_spin(req):
            return None
        sel = _op.weld_orientation_for_request(cell, dict(req, robot_ns=ns), list(start), approach_offset_m=off)
    except Exception:  # noqa: BLE001 - prediction only; the real leg end is verified by the orchestrator
        return None
    return dict(sel["orientation"]) if sel.get("changed") else None


def predict_chain(cell: str, ns: str, start: list, reqs: list, approach_offset_m: float = 0.1) -> dict:
    t0 = time.monotonic()
    ends: list = []
    cur = list(start)
    for req in reqs:
        e = predict_end(cell, ns, cur, req, approach_offset_m)
        ends.append(e)
        if e is not None:
            cur = e
    return {"ns": ns, "ends": ends, "wall_s": round(time.monotonic() - t0, 4), "pid": os.getpid()}


# ─── planning ────────────────────────────────────────────────────────────────

def _validate_weld(cell: str, ns: str, req: dict, traj: list) -> Optional[dict]:
    """umarell's validate_approach on the planned weld (the check the LLM ran as a separate turn)."""
    if not (req.get("constrain_orientation") and req.get("orientation")) or not traj:
        return None
    from agents.trajectory.tools.pinocchio import validate_approach
    t = req["target"]
    res = validate_approach.func(cell_id=cell, robot_ns=ns, expected_orientation=req["orientation"],
                                 trajectory=traj, target_position=[t["x"], t["y"], t["z"]],
                                 approach_vector=req.get("approach_vector"))
    keep = ("matches", "tool_axis_error_deg", "orientation_error_deg", "position_error_m",
            "approach_direction_error_deg", "error")
    return {k: res.get(k) for k in keep if k in res}


def plan_chain(cell: str, legs: list, kw: dict, validate: bool = True) -> dict:
    """Plan ``legs`` of ONE arm in order (usually one leg; a recovery candidate may be [via, weld]).

    ``legs``: ``[{"ns", "op_id", "req", "start"?, "prev_pose"?}]`` -- ``start``/``prev_pose`` of the first leg are
    injected; later legs chain from the actual end of the leg before. Stops at the first failure.
    """
    from agents.trajectory.tools.planning_tools import run_trajectory_planner
    t0 = time.monotonic()
    out: list = []
    start = legs[0].get("start") if legs else None
    prev_pose = legs[0].get("prev_pose") if legs else None
    reset_session(cell)
    if legs:
        inject(cell, legs[0]["ns"], start, prev_pose, legs[0].get("welds"), legs[0].get("specs"))
    stage = None
    for leg in legs:
        ns, req = leg["ns"], dict(leg["req"])
        req.pop("current_joints", None)
        pinned = False
        if stage is not None and not req.get("stage_for"):
            req = pin_stage(req, stage)        # the weld after a staging move: the pose chosen from the stand-off
            pinned, stage = "applied_tool_spin_deg" in req and "applied_tool_spin_deg" not in leg["req"], None
        inject(cell, ns, start, prev_pose)     # later legs: the planner's own weld records of this chain stay
        t1 = time.monotonic()
        if req.get("home_return"):
            res = plan_home_return(cell, ns, start, kw.get("velocity_scale"))
        elif req.get("stage_for"):
            res = plan_stage(cell, ns, start, req, float(kw.get("approach_offset_m") or 0.12),
                             kw.get("velocity_scale"))
            stage = res.get("stage") if res.get("trajectories") else None
        else:
            res = run_trajectory_planner.func(cell_id=cell, motion_requests=[req], **kw)
        wall = time.monotonic() - t1
        traj = (res.get("trajectories") or {}).get(ns)
        fails = [f for f in (res.get("failures") or []) if f.get("robot_ns", ns) == ns]
        rec = {"op_id": leg["op_id"], "ns": ns, "ok": bool(traj) and not fails, "traj": traj,
               "failures": fails, "planning_time_s": res.get("planning_time_s"), "wall_s": round(wall, 3),
               "start": start, "liftoff_warnings": res.get("liftoff_warnings"),
               "weld_orientations": res.get("weld_orientations")}
        if res.get("stage") is not None:
            rec["stage"] = res["stage"]
        if pinned:
            rec["req"] = req                   # the request as planned (the orchestrator keeps it on the leg)
        if rec["ok"]:
            rec["end"] = list(traj[-1]["positions"])
            rec.update(leg_records(cell, ns, leg["op_id"]))
            if validate:
                # judged against the task's pose, as every leg (the pinned spin / tilt is the search's choice)
                rec["approach"] = _validate_weld(cell, ns, dict(leg["req"]) if pinned else req, traj)
        out.append(rec)
        if not rec["ok"]:
            break
        start, prev_pose = rec["end"], rec.get("last_pose")
    return {"legs": out, "ok": bool(out) and all(r["ok"] for r in out) and len(out) == len(legs),
            "wall_s": round(time.monotonic() - t0, 3), "pid": os.getpid()}


def splice_weld_leg(cell: str, cached: dict, spec: dict, kw: dict) -> Optional[dict]:
    """A weld leg planned from ``spec["start"]`` by re-using ``cached`` -- the same arm's planned leg into the same
    weld request from another start: its approach (the stand-off configuration and the descent into the weld, which
    do not depend on where the arm came from) is kept, and only the transit from the new start to the stand-off is
    planned, by umarell's own joint-target planner (``planner_backend.plan_home_trajectory``: the on-axis lift-off
    out of the weld the arm stands on, a collision-free path, the platform's leg check -- the staging move's
    planner). The result is shaped as :func:`plan_chain`'s leg record; None when the cached leg has no approach,
    the transit cannot be planned, or the spliced leg fails umarell's leg check (``leg_verdict.refusal`` with the
    static obstacles). The weld records (weld pose, op spec, last pose) are the cached leg's: same request."""
    from agents.trajectory.tools import leg_verdict as lv
    from agents.trajectory.tools import planner_backend as pb
    ns, req, start = spec["ns"], spec["req"], spec.get("start")
    traj = cached.get("traj") or []
    k = next((i for i, p in enumerate(traj) if p.get("phase") == "approach"), None)
    if not start or not k:
        return None
    q_a = list(traj[k]["positions"])
    t0 = time.monotonic()
    reset_session(cell)
    inject(cell, ns, start, spec.get("prev_pose"), spec.get("welds"), spec.get("specs"))
    vkw = {} if kw.get("velocity_scale") is None else {"velocity_scale": float(kw["velocity_scale"])}
    transit = pb.plan_home_trajectory(cell, ns, list(start), q_a, **vkw)
    if not transit:
        return None
    t_end = float(transit[-1].get("time_s") or 0.0)
    shift = t_end - float(traj[k].get("time_s") or 0.0)
    new = [dict(p, phase="transit") for p in transit] + \
        [dict(p, time_s=float(p.get("time_s") or 0.0) + shift) for p in traj[k + 1:]]
    if lv.refusal(cell, ns, new, static=True) is not None:
        return None
    rec = {**cached, "start": list(start), "traj": new, "end": list(new[-1]["positions"]),
           "planning_time_s": round(time.monotonic() - t0, 3), "wall_s": round(time.monotonic() - t0, 3),
           "spliced_from": list(cached.get("start") or [])}
    rec["approach"] = _validate_weld(cell, ns, req, new)
    if rec["approach"] is not None and rec["approach"].get("matches") is False:
        return None
    return rec


def home_joints(cell: str, ns: str) -> list:
    """The arm's HOME as the planner assumes it (the cell's declared home, else umarell's fallback)."""
    from agents.home_pose import HOME_JOINTS_FALLBACK, home_pose
    return [float(v) for v in home_pose(cell, ns, HOME_JOINTS_FALLBACK)]


def plan_home_return(cell: str, ns: str, start: list, velocity_scale=None) -> dict:
    """A planned return to HOME from ``start`` as one leg of a chain (a request with ``home_return``): umarell's own
    ``planner_backend.plan_home_trajectory`` -- the on-axis lift-off out of the weld the arm stands on, then a
    collision-free joint-space path to the arm's HOME (staged when there is no direct one), checked by the platform's
    leg check -- the return the assembler plans at the end of a program. Shaped as ``run_trajectory_planner``'s
    result, so the next leg chains from HOME exactly (the start its weld plans from when it is an arm's first)."""
    from agents.trajectory.tools import trajectory_cache as tc
    try:
        from agents.trajectory.tools import planner_backend as pb      # 76ea13a: the platform's leg check too
    except ImportError:                                                  # eccb1bc: the OMPL planner's own
        from agents.trajectory.tools import ompl_planner as pb
    home = home_joints(cell, ns)
    t0 = time.monotonic()
    kw = {} if velocity_scale is None else {"velocity_scale": float(velocity_scale)}
    traj = pb.plan_home_trajectory(cell, ns, list(start), home, **kw)
    out = {"planning_time_s": round(time.monotonic() - t0, 3)}
    if traj:
        out["trajectories"] = {ns: traj}
        tc.clear_last_pose(cell, ns)
        return out
    why = None
    try:
        why = tc.home_refusal(cell, ns)
    except Exception:  # noqa: BLE001 -- eccb1bc records no refusals: the generic reason
        pass
    why = dict(why or {})
    out["failures"] = [{"robot_ns": ns, "reason_code": why.get("reason_code") or "home_return_failed",
                        "error": why.get("error") or "no planned return to HOME from this pose"}]
    return out


def stroke_from(cell: str, ns: str, seed: list, req: dict, approach_offset_m: float = 0.12) -> dict:
    """Whether the planner's terminal stroke into ``req``'s weld is clear when the leg starts at ``seed``, by the
    planner's own code path (backends/ompl/planner.py, umarell 76ea13a): the weld-pose search (tool spin / tilt,
    ``planner_backend._auto_weld_orientation``) from ``seed``, ``run_ik`` at the weld and at the stand-off
    warm-started from ``seed``, the weld pose's self-collision, then ``leg_builder.cartesian_descent`` from the
    stand-off against the cell's boxes at the approach-speed margin.

    The warm start decides the arm's branch: from HOME and from the end of a previous weld the same weld can land on
    different wrist / elbow branches, one with a clear descent, one whose forearm hits the panel (body_weld_3: W03
    on robot1 is clear from a wrist-up seed, blocked at step 4/6 by link_5 from HOME -- every one of the 407 spin /
    tilt poses). Returns ``{"clear", "via_q" (the stand-off configuration the descent starts from), "end_q",
    "orientation" + "spin_deg" (the pose the search chose, None: the specified one), "why"}``."""
    import numpy as np
    from agents.trajectory.tools import leg_builder as lb
    from agents.trajectory.tools import planner_backend as pb
    from agents.trajectory.tools.pinocchio import run_ik
    from agents.trajectory.tools.robot_kinematics import (
        _cell_sphere_radii, _load_clearance_policy, _load_pinocchio_model, _load_planning_config,
        _load_robot_config, _self_violation, _speed_margin, load_static_obstacles)
    try:
        off = float(req.get("approach_offset_m") or approach_offset_m)
    except (TypeError, ValueError):
        off = approach_offset_m
    r = pb.apply_tool_spin(dict(req, robot_ns=ns, current_joints=[float(v) for v in seed]))
    if r.get("approach_offset_m") is None:
        r["approach_offset_m"] = off
    _new, chosen, _rep = pb._auto_weld_orientation(cell, r, off)
    o = dict((chosen or {}).get("orientation") or r["orientation"])
    out: dict = {"clear": False, "seed": [float(v) for v in seed],
                 "orientation": dict(chosen["orientation"]) if chosen else None,
                 "spin_deg": chosen.get("spin_deg") if chosen else None}
    model, _ = _load_pinocchio_model(cell, ns)
    rcfg = _load_robot_config(cell, ns)
    t = r["target"]
    p = np.array([float(t["x"]), float(t["y"]), float(t["z"])])
    av = np.asarray(r.get("approach_vector", [0, 0, -1]), float)
    av = av / (np.linalg.norm(av) + 1e-9)
    via_xyz = p - av * off
    ik_ori = {"qx": float(o.get("x", 0.0)), "qy": float(o.get("y", 0.0)), "qz": float(o.get("z", 0.0)),
              "qw": float(o.get("w", 1.0)), "constrain_orientation": True}
    warm = [float(v) for v in seed]
    goal = run_ik.invoke({"cell_id": cell, "robot_ns": ns, "x": float(p[0]), "y": float(p[1]), "z": float(p[2]),
                          "warm_start_joints": warm, **ik_ori})
    if not goal.get("success"):
        return {**out, "why": "ik_failed"}
    goal_q = np.asarray(goal["joint_positions"][:model.nq], float)
    if _self_violation(model, goal_q) is not None:
        return {**out, "why": "goal_self_collision"}
    app = run_ik.invoke({"cell_id": cell, "robot_ns": ns, "x": float(via_xyz[0]), "y": float(via_xyz[1]),
                         "z": float(via_xyz[2]), "warm_start_joints": warm, **ik_ori})
    via = app["joint_positions"] if app.get("success") else \
        lb.approach_ik(run_ik, cell, ns, via_xyz.tolist(), ik_ori, [goal_q.tolist()])
    if not via:
        return {**out, "why": "approach_unreachable"}
    tool_cfg = rcfg.get("tool_collision")
    pol = _load_clearance_policy(cell)
    max_vs = float(_load_planning_config(cell).get("max_velocity_scaling") or 1.0)
    try:
        avs = float(r.get("approach_velocity_scale"))
    except (TypeError, ValueError):
        avs = max_vs * float(pol["approach_velocity_fraction"])
    avs = max(min(avs, max_vs), 0.01)
    why: dict = {}
    wps = lb.cartesian_descent(model, run_ik, cell, ns, via_xyz.tolist(), p.tolist(), ik_ori,
                               np.asarray(via[:model.nq], float), np.asarray(rcfg["base_position"], float),
                               float(rcfg.get("base_orientation_yaw", 0.0)), [], load_static_obstacles(cell),
                               pol["normal"], tool_cfg, sphere_radii=_cell_sphere_radii(cell, model, tool_cfg),
                               extra_margin=_speed_margin(cell, avs), why=why)
    if wps is None:
        info = why.get("collision") or {}
        return {**out, "why": f"{why.get('cause')} at step {why.get('step')}/{why.get('steps')} "
                              f"{info.get('object_id') or ''}".strip()}
    return {**out, "clear": True, "via_q": [float(v) for v in via[:model.nq]],
            "end_q": [float(v) for v in wps[-1]]}


def plan_stage(cell: str, ns: str, start: list, req: dict, approach_offset_m: float = 0.12,
               velocity_scale=None) -> dict:
    """A staging move before a weld (a request with ``stage_for``: the weld's request, and ``seeds``: warm starts to
    try): the first seed (in order) whose stroke into the weld is clear (:func:`stroke_from`) gives the stand-off
    configuration to stage at; the arm is planned there from ``start`` by umarell's own joint-target planner
    (``plan_home_trajectory`` with that configuration as its goal: the on-axis lift-off out of the weld it stands on,
    a collision-free path, staged when there is no direct one, the platform's leg check). The weld then plans from
    the stand-off on the clear branch. Shaped as ``run_trajectory_planner``'s result, plus ``stage`` (the seed, the
    stand-off and the weld pose to pin on the weld's request: the search's choice from the staged start)."""
    from agents.trajectory.tools import trajectory_cache as tc
    from agents.trajectory.tools import planner_backend as pb
    weld = dict(req["stage_for"])
    t0 = time.monotonic()
    tried = []
    for seed in [s for s in req.get("seeds") or [] if s]:
        try:
            s = stroke_from(cell, ns, seed, weld, approach_offset_m)
        except Exception as exc:  # noqa: BLE001 -- one seed's failure is that seed's
            s = {"clear": False, "why": repr(exc)}
        tried.append({"seed": [round(float(v), 4) for v in seed], "clear": s.get("clear"), "why": s.get("why")})
        if not s.get("clear"):
            continue
        kw = {} if velocity_scale is None else {"velocity_scale": float(velocity_scale)}
        traj = pb.plan_home_trajectory(cell, ns, list(start), s["via_q"], **kw)
        if not traj:
            tried[-1]["why"] = "no planned path to the stand-off"
            continue
        tc.clear_last_pose(cell, ns)
        return {"trajectories": {ns: traj}, "planning_time_s": round(time.monotonic() - t0, 3),
                "stage": {"seed": s["seed"], "via_q": s["via_q"], "orientation": s.get("orientation"),
                          "spin_deg": s.get("spin_deg"), "tried": tried}}
    return {"planning_time_s": round(time.monotonic() - t0, 3), "stage": {"tried": tried},
            "failures": [{"robot_ns": ns, "reason_code": "no_clear_stage",
                          "error": f"no seed of {len(tried)} gives a clear stroke into {weld.get('operation_id')} "
                                   f"(or no path to its stand-off)"}]}


def pin_stage(req: dict, stage: Optional[dict]) -> dict:
    """The weld request after a staging move: the weld pose the search chose from the staged start pinned on it
    (``applied_tool_spin_deg``: the planner plans it as is, without searching again from a start a re-plan may
    have moved), unchanged when the specified pose is the one."""
    if not stage or not stage.get("orientation"):
        return req
    return {**req, "orientation": dict(stage["orientation"]),
            "applied_tool_spin_deg": float(stage.get("spin_deg") or 0.0)}


def plan_coupled(cell: str, legs: list, kw: dict, validate: bool = True) -> dict:
    """Plan one leg per arm in ONE planner call (umarell's coupled PARALLEL step: each later arm is routed
    around the swept volume of the arms before it). ``legs`` in the order they should be planned."""
    from agents.trajectory.tools.planning_tools import run_trajectory_planner
    t0 = time.monotonic()
    reqs = []
    reset_session(cell)
    for leg in legs:
        inject(cell, leg["ns"], leg.get("start"), leg.get("prev_pose"), leg.get("welds"), leg.get("specs"))
        r = dict(leg["req"])
        r.pop("current_joints", None)
        reqs.append(r)
    res = run_trajectory_planner.func(cell_id=cell, motion_requests=reqs, **kw)
    out = []
    for leg, req in zip(legs, reqs):
        ns = leg["ns"]
        traj = (res.get("trajectories") or {}).get(ns)
        fails = [f for f in (res.get("failures") or []) if f.get("robot_ns") == ns]
        rec = {"op_id": leg["op_id"], "ns": ns, "ok": bool(traj) and not fails, "traj": traj, "failures": fails,
               "planning_time_s": res.get("planning_time_s"), "start": leg.get("start")}
        if rec["ok"]:
            rec["end"] = list(traj[-1]["positions"])
            rec.update(leg_records(cell, ns, leg["op_id"]))
            if validate:
                rec["approach"] = _validate_weld(cell, ns, req, traj)
        out.append(rec)
    return {"legs": out, "ok": all(r["ok"] for r in out), "wall_s": round(time.monotonic() - t0, 3),
            "pid": os.getpid()}


# ─── gating (read-only checks) ───────────────────────────────────────────────

def gate_step(cell: str, legs: dict, delays: Optional[dict] = None, bystanders: Optional[dict] = None,
              allow_parallel: bool = True) -> dict:
    """umarell's assembler gate (``behaviour_tree._gate_multi_arm_step``) for one step, exactly as the assembler
    will call it -- so the orchestrator can repair steps before assembly, all steps at once."""
    from agents.trajectory.tools.behaviour_tree import _gate_multi_arm_step
    t0 = time.monotonic()
    rep = _gate_multi_arm_step(cell, legs, allow_parallel=allow_parallel, delays=delays or {},
                               bystanders=bystanders or {})
    return {"report": _slim(rep), "mode": rep.get("mode"), "wall_s": round(time.monotonic() - t0, 3),
            "pid": os.getpid()}


def start_delays(cell: str, legs: dict) -> dict:
    """umarell's ``planning_tools.find_start_delays`` (the delay search behind ``delay_fix``)."""
    from agents.trajectory.tools.planning_tools import find_start_delays
    t0 = time.monotonic()
    fix = find_start_delays(cell, legs)
    return {"fix": fix, "wall_s": round(time.monotonic() - t0, 3), "pid": os.getpid()}


def _slim(rep: dict) -> dict:
    """A gate report without bulky arrays (it is returned through a pipe and logged)."""
    out = {}
    for k, v in (rep or {}).items():
        if k in ("parallel", "standing") and isinstance(v, dict):
            out[k] = {kk: vv for kk, vv in v.items()
                      if kk in ("collision_free", "min_clearance_m", "warning_distance_m", "worst_pair",
                                "collision_time_s", "error", "static_collision", "static_breaches")}
        else:
            out[k] = v
    return out


# ─── trial assembly ──────────────────────────────────────────────────────────

def assemble_trial(cell: str, trajs: list, arm_state: dict, plan: dict) -> dict:
    """Run umarell's ``assemble_behaviour_tree`` on a candidate plan in this worker.

    ``trajs``: ``[(ns, op_id, traj)]`` loaded into this worker's trajectory cache (cleared first);
    ``arm_state``: ``{ns: {"end": q, "pose": pose|None, "welds"?, "specs"?}}`` -- the chaining state the home step
    plans from (``_home_liftoff`` reads the last working pose; on 76ea13a also every weld pose, which the tree
    carries as ``metadata.weld_poses``, and every op spec, which the assembler's approach audit checks each weld leg
    against). Lets the orchestrator try several plan shapes at once (home_at_end true/false, final park poses);
    the parent re-runs the chosen one in its own session.
    """
    from agents.trajectory.tools import trajectory_cache as tc
    from agents.trajectory.tools.behaviour_tree import assemble_behaviour_tree
    t0 = time.monotonic()
    reset_session(cell)
    for ns, op_id, traj in trajs:
        tc.cache_trajectory(cell, ns, op_id, traj)
    for ns, st in (arm_state or {}).items():
        inject(cell, ns, st.get("end"), st.get("pose"), st.get("welds"), st.get("specs"))
    res = assemble_behaviour_tree.func(cell_id=cell, plan=plan)
    ok = bool(res.get("success"))
    slim = {k: v for k, v in res.items() if k != "behaviour_tree"}
    return {"ok": ok, "result": slim, "behaviour_tree": res.get("behaviour_tree") if ok else None,
            "wall_s": round(time.monotonic() - t0, 3), "pid": os.getpid()}


def gate_many(cell: str, checks: list) -> dict:
    """Several gate calls in one task: ``checks`` = ``[(legs, delays, bystanders)]``; returns the modes (None =
    refused) and the slim reports. Used to judge a repair candidate that changes two steps at once."""
    from agents.trajectory.tools.behaviour_tree import _gate_multi_arm_step
    t0 = time.monotonic()
    modes, reps = [], []
    for legs, delays, bys in checks:
        if len(legs) < 2 and not bys:
            modes.append("PARALLEL")
            reps.append({"note": "single arm, no bystander"})
            continue
        rep = _gate_multi_arm_step(cell, legs, allow_parallel=True, delays=delays or {}, bystanders=bys or {})
        modes.append(rep.get("mode"))
        reps.append(_slim(rep))
    return {"modes": modes, "reports": reps, "wall_s": round(time.monotonic() - t0, 3), "pid": os.getpid()}
