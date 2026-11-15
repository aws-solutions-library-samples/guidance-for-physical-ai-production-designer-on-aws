#!/usr/bin/env python3
"""The planner's own approach check for check_reachability (mode full): can each robot that the IK says reaches a
weld also make the straight on-axis working stroke into it, by umarell's predicate?

    CELL_BASE_PATH=<cells> <umarell>/.venv-planner/bin/python -I approach_check.py --umarell <umarell> < job.json

The reach IK (reach_check.py, trajgen) solves the weld pose with the arm's link spheres and the tool against the
product's box only. umarell_u3 plans the terminal stroke with umarell's tools on the prepared cell, and refuses a weld
whose stroke the arm envelope (robots.yaml link_collision, at link_collision.radius_m plus each box's
collision_margin) or the tool cannot make through the cell's obstacle boxes (the converter's voxel shell of the
product: coarse <fixture>_v*, near-weld <fixture>_f*, is_workpiece) as goal_in_collision. This script runs that
refusal's code path for each robot x weld, as the planner calls it (backends/ompl/planner.py):

1. the weld-pose search (tools/weld_orientation.search: tool spin, tilt up to the tool's max_tilt_deg), judged at
   the stroke, from the robot's HOME (the planner: from where the robot's previous leg ended);
2. pinocchio.run_ik at the weld and at the stand-off (the stroke back along the approach), warm-started there;
   the weld configuration must not fold the arm into itself (robot_kinematics._self_violation);
3. leg_builder.cartesian_descent from the stand-off into the weld (6 IK steps, each checked with
   robot_kinematics._config_is_free against load_static_obstacles at the cell's link radius and the approach-speed
   margin, then the tool-contact rule on the whole stroke); a refusal is leg_builder.descent_failure's.

U3's strokes are tried in its order (its approach offset 0.12 m, then its recovery offsets 0.2 m and 0.08 m) until
one is clear.

Chained (the job's ``chains``): the planner plans a weld from where the arm's PREVIOUS weld ended, not from HOME --
the weld-pose search, the IK at the weld and at the stand-off and the descent are all warm-started from the previous
weld's end, so they can land on another wrist / elbow branch whose stroke hits the panel, and the leg first lifts the
tool out of the previous seam along that weld's axis (a blocked lift-off refuses the leg: liftoff_blocked). Each
chain (one robot's welds in their order) is walked as the planner does: the first weld from HOME, every later one
after the on-axis lift-off out of its predecessor (U3's retreat strokes: the stroke, then its recovery offsets
0.08 / 0.05 / 0.03 m) and with the predecessor's end configuration as the warm start; a weld that is blocked is
skipped (U3 drops it from the arm's chain), so the next one chains from the last clear one. The job on stdin:

    {"cell_id", "workers", "strokes_m": [0.12, 0.2, 0.08], "approach_velocity_scale": 0.25,
     "pairs": [{"robot" (the cell's robot id, robots.yaml), "weld", "position", "approach_vector" (tool -> weld),
                "orientation" {x, y, z, w}}],
     "chains": [{"robot", "welds": [{"weld", "position", "approach_vector", "orientation"}, ...]}],
     "retreats_m": [0.08, 0.05, 0.03]}

stdout: one NDJSON line per pair as it is checked,

    {"robot", "weld", "clear": bool, "stroke_m" (the clear stroke), "reason_code" (umarell's), "reason",
     "blocker": {"type", "object_id", "link", "is_workpiece", "step", "steps", "clearance_m"},
     "search": {"status", "candidates", "spin_deg", "tilt_deg"}, "strokes": [...], "elapsed_s"}

and per chained weld (after the pairs)

    {"robot", "weld", "chained": true, "index", "predecessor" (the weld it chains from, null: from HOME), "clear",
     "stroke_m", "reason_code" (liftoff_blocked | umarell's descent refusal), "reason", "blocker",
     "liftoff": {"clear", "retreat_m"} (null for the first weld), "search", "elapsed_s"}

then {"done": true}; on a failure of the whole job {"error": ...} and exit code 1. The top level is standard library
only, so the module and its tests load without umarell.
"""
from __future__ import annotations

import json
import os
import sys
import time

STROKES_M = (0.12, 0.2, 0.08)        # umarell_fast U3: Params.approach_offset_m, then its recovery offsets
RETREATS_M = (0.08, 0.05, 0.03)      # umarell_fast U3: Params.retreat_offsets_m (a liftoff_blocked leg's lever)
APPROACH_VELOCITY_SCALE = 0.25       # U3's Params.approach_velocity_scale (the stroke's speed margin)


def link_name(model, info: dict) -> str:
    """The arm body of a collision ``info`` (robot_kinematics._config_collision_info): its frame, the link its
    envelope sphere is on, or the tool."""
    if info.get("type") == "self":
        return str(info.get("object_id") or "the arm")
    if info.get("type") == "tool_penetration" or info.get("tool_sphere") is not None:
        return "the tool"
    try:
        frames = model.frames
        if info.get("envelope_sphere") is not None:
            from agents.trajectory.tools.robot_kinematics import _robot_spec
            env = _robot_spec(model)["env"]
            return str(frames[int(env["fids"][int(info["envelope_sphere"])])].name)
        if info.get("link_index") is not None and int(info["link_index"]) < len(frames):
            return str(frames[int(info["link_index"])].name)
    except Exception:  # noqa: BLE001 - a name for the message only
        pass
    return "the arm"


def blocker(model, why: dict) -> dict:
    """What stopped a descent (leg_builder's ``why``), for the report."""
    info = why.get("collision") or {}
    out = {"type": info.get("type") or why.get("cause"), "object_id": info.get("object_id"),
           "link": link_name(model, info) if info else None, "is_workpiece": bool(info.get("is_workpiece")),
           "step": why.get("step"), "steps": why.get("steps"), "clearance_m": info.get("clearance_m")}
    return {k: v for k, v in out.items() if v is not None}


def stroke_check(env: dict, pair: dict, stroke_m: float, seed=None) -> dict:
    """One stroke of one pair: ``{"stroke_m", "clear", "reason_code", "reason", "blocker", "search"}``; ``seed``: the
    configuration the planner starts the leg from (its warm start; default the robot's HOME). A clear stroke also
    returns ``end_q`` (where the descent leaves the arm, on the seam) and ``pose`` (the weld pose it holds)."""
    import numpy as np
    from agents.trajectory.tools import leg_builder as lb
    from agents.trajectory.tools import weld_orientation as wo
    from agents.trajectory.tools.robot_kinematics import _self_violation

    cell, ns, model = env["cell_id"], pair["robot"], env["model"]
    p = np.asarray([float(v) for v in pair["position"]], float)
    av = np.asarray([float(v) for v in pair["approach_vector"]], float)
    av = av / (np.linalg.norm(av) + 1e-9)
    req = {"robot_ns": ns, "operation_id": str(pair["weld"]), "operation_type": "weld",
           "target": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])},
           "approach_vector": av.tolist(), "approach": True, "constrain_orientation": True,
           "orientation": dict(pair["orientation"]), "approach_offset_m": float(stroke_m),
           "approach_velocity_scale": env["approach_velocity_scale"]}
    out: dict = {"stroke_m": float(stroke_m), "clear": False}
    warm = [float(v) for v in (env["home"] if seed is None else seed)]
    found = wo.search(cell, req, warm)
    chosen = found.get("chosen") or {}
    out["search"] = {"status": found.get("status"), "candidates": found.get("candidates", 0),
                     **({"spin_deg": chosen.get("spin_deg"), "tilt_deg": chosen.get("tilt_deg")} if chosen else {})}
    o = chosen.get("orientation") or req["orientation"]
    ik_ori = {"qx": float(o.get("x", 0.0)), "qy": float(o.get("y", 0.0)), "qz": float(o.get("z", 0.0)),
              "qw": float(o.get("w", 1.0)), "constrain_orientation": True}
    ik = env["run_ik"]
    via_xyz = p - av * float(stroke_m)
    goal = ik.invoke({"cell_id": cell, "robot_ns": ns, "x": float(p[0]), "y": float(p[1]), "z": float(p[2]),
                      "warm_start_joints": warm, **ik_ori})
    if not goal.get("success"):
        return {**out, "reason_code": "ik_failed",
                "reason": f"the planner's IK finds no oriented solution at the weld ({goal.get('reason')})"}
    goal_q = np.asarray(goal["joint_positions"][:model.nq], float)
    hit = _self_violation(model, goal_q)
    if hit is not None:
        return {**out, "reason_code": "goal_self_collision", "blocker": blocker(model, {"collision": hit}),
                "reason": f"the weld pose folds the arm into itself ({hit.get('object_id')})"}
    app = ik.invoke({"cell_id": cell, "robot_ns": ns, "x": float(via_xyz[0]), "y": float(via_xyz[1]),
                     "z": float(via_xyz[2]), "warm_start_joints": warm, **ik_ori})
    via = app["joint_positions"] if app.get("success") else \
        lb.approach_ik(ik, cell, ns, via_xyz.tolist(), ik_ori, [goal_q.tolist()])
    if not via:
        f = lb.approach_waypoint_failure(ns, float(stroke_m))
        return {**out, "reason_code": f.get("reason_code") or "approach_unreachable",
                "reason": f.get("reason") or f.get("error") or "the stand-off has no solution at the weld's lean"}
    why: dict = {}
    wps = lb.cartesian_descent(model, ik, cell, ns, via_xyz.tolist(), p.tolist(), ik_ori,
                               np.asarray(via[:model.nq], float), env["base"], env["yaw"], [], env["boxes"],
                               env["radius"], env["tool_cfg"], sphere_radii=env["radii"],
                               extra_margin=env["margin"], why=why)
    if wps is not None:
        return {**out, "clear": True, "end_q": [float(v) for v in wps[-1]],
                "pose": {"target": p.tolist(), "approach_vector": av.tolist(), "orientation": dict(o)}}
    f = lb.descent_failure(why)
    return {**out, "reason_code": f.get("reason_code"), "reason": f.get("reason"), "blocker": blocker(model, why)}


def liftoff_check(env: dict, pose: dict, start_q, retreat_m: float) -> bool:
    """The planner's on-axis lift-off out of the weld ``pose`` the arm stands on at ``start_q``
    (ompl/planner.py: leg_builder.cartesian_retreat from the seam back along that weld's approach vector, at its
    orientation, ``retreat_m`` long, against the cell's boxes at the approach-speed margin): True when clear."""
    import numpy as np
    from agents.trajectory.tools import leg_builder as lb
    av = np.asarray(pose["approach_vector"], float)
    av = av / (np.linalg.norm(av) + 1e-9)
    tgt = np.asarray(pose["target"], float)
    o = pose["orientation"]
    ik_ori = {"qx": float(o.get("x", 0.0)), "qy": float(o.get("y", 0.0)), "qz": float(o.get("z", 0.0)),
              "qw": float(o.get("w", 1.0)), "constrain_orientation": True}
    wps = lb.cartesian_retreat(env["model"], env["run_ik"], env["cell_id"], env["ns"], tgt.tolist(),
                               (tgt - av * float(retreat_m)).tolist(), ik_ori, np.asarray(start_q, float),
                               env["base"], env["yaw"], [], env["boxes"], env["radius"], env["tool_cfg"],
                               sphere_radii=env["radii"], extra_margin=env["margin"])
    return bool(wps)


def check_chain(job: dict, chain: dict) -> list:
    """One robot's welds in their order, each from where the planner would start it (module doc, Chained): the
    first from HOME, every later one after the lift-off out of the last clear weld and warm-started from its end.
    One output line per weld."""
    ns = chain["robot"]
    strokes = [float(s) for s in (job.get("strokes_m") or STROKES_M)]
    lines = []
    prev = None                          # (weld id, its pose, its end configuration)
    try:
        env = dict(robot_env(job, ns), ns=ns)
    except Exception as e:  # noqa: BLE001 - the chain's failure is every weld's
        return [{"robot": ns, "weld": str(w.get("weld")), "chained": True, "index": i, "clear": None,
                 "reason_code": "error", "reason": f"{type(e).__name__}: {e}"}
                for i, w in enumerate(chain.get("welds") or [])]
    for i, w in enumerate(chain.get("welds") or []):
        t0 = time.time()
        pair = {**w, "robot": ns}
        line = {"robot": ns, "weld": str(w.get("weld")), "chained": True, "index": i,
                "predecessor": prev[0] if prev else None, "clear": False, "stroke_m": None, "liftoff": None}
        try:
            seed = prev[2] if prev else None
            if prev is not None:
                tried = [strokes[0]] + [float(r) for r in (job.get("retreats_m") or RETREATS_M)]
                ok = next((r for r in tried if liftoff_check(env, prev[1], prev[2], r)), None)
                line["liftoff"] = {"clear": ok is not None, "retreat_m": ok, "tried_m": tried}
            if prev is not None and line["liftoff"]["clear"] is False:
                line.update(reason_code="liftoff_blocked",
                            reason=f"the on-axis lift-off out of {prev[0]} is blocked at every retreat "
                                   f"({', '.join(str(r) for r in line['liftoff']['tried_m'])} m)")
            else:
                first = win = None
                for s in strokes:
                    r = stroke_check(env, pair, s, seed)
                    first = first or r
                    if r["clear"]:
                        win = r
                        break
                line["search"] = (win or first).get("search")
                if win is not None:
                    line.update(clear=True, stroke_m=win["stroke_m"])
                    prev = (line["weld"], win["pose"], win["end_q"])
                else:
                    line.update(reason_code=first.get("reason_code"), reason=first.get("reason"),
                                blocker=first.get("blocker") or {})
        except Exception as e:  # noqa: BLE001 - one weld's failure is that weld's line
            line.update(clear=None, reason_code="error", reason=f"{type(e).__name__}: {e}")
        line["elapsed_s"] = round(time.time() - t0, 2)
        lines.append(line)
    return lines


_ENV: dict = {}


def robot_env(job: dict, ns: str) -> dict:
    """What the planner loads for robot ``ns`` (ompl/planner.py plan_trajectories_ompl), once per process."""
    if ns in _ENV:
        return _ENV[ns]
    import numpy as np
    from agents.trajectory.tools import leg_builder as lb
    from agents.trajectory.tools.pinocchio import run_ik
    from agents.trajectory.tools.robot_kinematics import (
        _cell_sphere_radii, _load_clearance_policy, _load_pinocchio_model, _load_planning_config,
        _load_robot_config, _speed_margin, load_static_obstacles)
    cell = job["cell_id"]
    model, _ = _load_pinocchio_model(cell, ns)
    rcfg = _load_robot_config(cell, ns)
    tool_cfg = rcfg.get("tool_collision")
    max_vs = float(_load_planning_config(cell)["max_velocity_scaling"])
    avs = max(min(float(job.get("approach_velocity_scale") or APPROACH_VELOCITY_SCALE), max_vs), 0.01)
    if "boxes" not in _ENV:
        _ENV["boxes"] = load_static_obstacles(cell)
    _ENV[ns] = {
        "cell_id": cell, "model": model, "run_ik": run_ik, "tool_cfg": tool_cfg,
        "base": np.asarray(rcfg["base_position"], float), "yaw": float(rcfg.get("base_orientation_yaw", 0.0)),
        "radii": _cell_sphere_radii(cell, model, tool_cfg), "radius": _load_clearance_policy(cell)["normal"],
        "margin": _speed_margin(cell, avs), "approach_velocity_scale": avs, "boxes": _ENV["boxes"],
        "home": [float(v) for v in lb.home_config(cell, ns, [0.0] * model.nq)],
    }
    return _ENV[ns]


def check_pair(job: dict, pair: dict) -> dict:
    """Every stroke of U3's until one is clear; the pair's output line."""
    t0 = time.time()
    line = {"robot": pair["robot"], "weld": pair["weld"], "clear": False}
    try:
        env = robot_env(job, pair["robot"])
        strokes = []
        for s in job.get("strokes_m") or STROKES_M:
            r = stroke_check(env, pair, float(s))
            strokes.append(r)
            if r["clear"]:
                break
        first = strokes[0]
        win = next((r for r in strokes if r["clear"]), None)
        line.update(clear=win is not None, stroke_m=win["stroke_m"] if win else None,
                    search=(win or first).get("search"),
                    strokes=[{k: v for k, v in r.items() if k in ("stroke_m", "clear", "reason_code", "blocker")}
                             for r in strokes])
        if win is None:
            line.update(reason_code=first.get("reason_code"), reason=first.get("reason"),
                        blocker=first.get("blocker") or {})
    except Exception as e:  # noqa: BLE001 - one pair's failure is that pair's result
        line.update(clear=None, reason_code="error", reason=f"{type(e).__name__}: {e}")
    line["elapsed_s"] = round(time.time() - t0, 2)
    return line


def _worker(args):
    job, pair = args
    return check_pair(job, pair)


def _chain_worker(args):
    job, chain = args
    return check_chain(job, chain)


def emit(out, value: dict) -> None:
    out.write(json.dumps(value, default=str) + "\n")
    out.flush()


def run(job: dict, out) -> int:
    pairs = job.get("pairs") or []
    chains = [c for c in job.get("chains") or [] if c.get("welds")]
    if not job.get("cell_id"):
        emit(out, {"error": "the job has no cell_id"})
        return 1
    if not pairs and not chains:
        emit(out, {"done": True})
        return 0
    try:
        import agents.trajectory.tools.leg_builder  # noqa: F401 - umarell is importable
        import agents.trajectory.tools.weld_orientation  # noqa: F401
        for ns in sorted({p["robot"] for p in pairs} | {c["robot"] for c in chains}):
            robot_env(job, ns)          # loaded before the fork: the workers share it
    except Exception as e:  # noqa: BLE001 - reported to the router
        emit(out, {"error": f"{type(e).__name__}: {e}"})
        return 1
    tasks = [(_worker, (job, p)) for p in pairs] + [(_chain_worker, (job, c)) for c in chains]
    workers = max(1, min(int(job.get("workers") or os.cpu_count() or 1), len(tasks)))

    def lines_of(res):
        return res if isinstance(res, list) else [res]

    if workers == 1:
        for fn, args in tasks:
            for line in lines_of(fn(args)):
                emit(out, line)
    else:
        import multiprocessing as mp
        with mp.get_context("fork").Pool(workers) as pool:
            for res in pool.imap_unordered(_call, tasks):
                for line in lines_of(res):
                    emit(out, line)
    emit(out, {"done": True})
    return 0


def _call(task):
    fn, args = task
    return fn(args)


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--umarell", default="/var/lib/factoryos/umarell", help="the umarell checkout (agents/, utils/)")
    a = ap.parse_args(argv)
    sys.path.insert(0, a.umarell)
    try:
        job = json.load(sys.stdin)
    except ValueError as e:
        emit(sys.stdout, {"error": f"invalid job JSON: {e}"})
        return 1
    return run(job, sys.stdout)


if __name__ == "__main__":
    sys.exit(main())
