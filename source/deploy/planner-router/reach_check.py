#!/usr/bin/env python3
"""Robo Fitter reachability check: one robot of a Fitter scene on its welds, with the cell planner's IK.

    <cell planner venv>/bin/python -I reach_check.py --planner-src /opt/factoryos-cell-planner/router/src < job.json

The FactoryOS backend (services/reach/ik.rs) starts one process per robot and writes the job on stdin:

    {"mode": "quick" | "full",
     "robots": [{"id", "urdf", "base_xyz" (the URDF root, pedestal included, scene origin subtracted),
                 "yaw_deg", "tilt_bc_deg", "home_deg", "tcp": {"tip_link", "offset", "axis"}}],
     "welds": [{"id", "position", "approach" (unit, tool -> weld)}],
     "floor_z", "tolerances": {"position_m", "tool_axis_deg"}}

and reads one NDJSON line per robot x weld from stdout as it is solved,

    {"robot", "weld", "reachable", "q_deg", "reasons": [...], "err": {"pos_m", "tool_axis_deg"}, "elapsed_s", "detail"}

then {"done": true}. On a failure that stops the whole job: {"error": ...} and exit code 1.

quick: ``SceneIK.solve_pose`` at the weld pose and ``check`` (joint limits, self collision, floor, pose
tolerance), spin about the tool axis 0 deg, then 180 deg: about 1 s per weld. full: ``SceneIK.solve_axis`` with
the approach (the straight retract to a 0.10 m stand-off must stay valid for >= 0.045 m; spin free, tilt up to
4.5 deg): about 20 s per weld. Fixtures are not checked (the backend checks the product's box); in mode full the
planner router then re-checks every reached weld x robot of a prepared cell with the planner's own approach
predicate (approach_check.py: the arm envelope and the tool on the straight stroke through the cell's obstacle boxes).

The top level is standard library only, so the module (and its tests) load without the venv; trajgen, numpy
and yaml are imported from --planner-src and the venv once the job runs.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import tempfile
import time
# Parses the cell's robot URDF (a file of the planner's own checkout or catalogue) with the stdlib's expat (no defusedxml dependency here): since Python 3.7.1 ElementTree resolves
# no external entities (no XXE), and expat >= 2.4.1 refuses billion-laughs / quadratic-blowup entity expansion.
import xml.etree.ElementTree as ET  # nosec B405  # nosemgrep: python.lang.security.use-defused-xml.use-defused-xml

# trajgen's raw reasons (`self_collision(0.0012)`, `floor(-0.010)`, ...) -> the backend's ReachReason names.
REASONS = {
    "joint_limits": "joint_limits",
    "self_collision": "self_collision",
    "floor": "floor",
    "umarell_floor": "floor",
    "pose_tolerance": "pose_tolerance",
    "no_solution": "no_solution",
    "base_tilt_unsupported": "base_tilt_unsupported",
}
ACCELERATION_DEFAULT = [3.0, 3.0, 3.0, 6.0, 6.0, 8.0]   # umarell's REVIEW default; IK does not use it
TOOL_SPHERE_RADIUS = 0.07
TOOL_SPHERE_START = 0.15
TOOL_SPHERE_STEP = 0.10


def normalise_reason(raw: str) -> str:
    """trajgen's reason -> the backend's name (unknown ones: `error`)."""
    name = str(raw).split("(", 1)[0].strip()
    return REASONS.get(name, "error")


def normalise_reasons(raws) -> list:
    out = []
    for r in raws:
        n = normalise_reason(r)
        if n not in out:
            out.append(n)
    return out


def disabled_pairs(urdf: str) -> list:
    """Self-collision pairs to skip: every joint's parent and child link (adjacent chain links)."""
    root = ET.parse(urdf).getroot()
    pairs = []
    for j in root.findall("joint"):
        p, c = j.find("parent"), j.find("child")
        if p is None or c is None:
            continue
        pair = [p.get("link"), c.get("link")]
        if pair[0] and pair[1] and pair not in pairs:
            pairs.append(pair)
    return pairs


def _norm(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v] if n > 1e-12 else [0.0, 0.0, 0.0]


def tool_spheres(tcp: dict) -> dict:
    """Spheres (radius 0.07 m) along the tool, from 0.15 m out to its TCP, in the tip link's frame."""
    offset = [float(x) for x in tcp["offset"]]
    length = math.sqrt(sum(x * x for x in offset))
    direction = _norm(offset) if length > 1e-9 else _norm(tcp.get("axis") or [-1.0, 0.0, 0.0])
    spheres, s = [], TOOL_SPHERE_START
    while s < length - 1e-9:
        spheres.append([direction[0] * s, direction[1] * s, direction[2] * s, TOOL_SPHERE_RADIUS])
        s += TOOL_SPHERE_STEP
    spheres.append([direction[0] * length, direction[1] * length, direction[2] * length, TOOL_SPHERE_RADIUS])
    return {"frame": tcp["tip_link"], "spheres": spheres}


def model_name(urdf: str) -> str:
    """robotmodel's model key, from the dressed URDF's file name (`kr240r2900__sticktool` -> `kr240r2900`)."""
    stem = os.path.splitext(os.path.basename(urdf))[0]
    return stem.split("__", 1)[0]


def scene_dict(job: dict) -> dict:
    """The trajgen scene (schema 2) of the job's robot(s), in the format of the planner's scene.yaml."""
    robots = []
    for r in job["robots"]:
        tcp = r["tcp"]
        robots.append({
            "id": r["id"], "model": model_name(r["urdf"]), "urdf": r["urdf"],
            "base": {"xyz": [float(x) for x in r["base_xyz"]], "yaw_deg": float(r.get("yaw_deg", 0.0))},
            "home_deg": [float(x) for x in r["home_deg"]],
            "joints": {"acceleration": ACCELERATION_DEFAULT},
            "tool": {"tip_link": tcp["tip_link"], "tcp_offset": [float(x) for x in tcp["offset"]],
                     "tcp_rotation": "identity", "axis": [float(x) for x in tcp["axis"]]},
            "collision": {"tool_spheres": tool_spheres(tcp), "disabled_pairs": disabled_pairs(r["urdf"])},
        })
    tol = job.get("tolerances") or {}
    return {
        "schema_version": 2, "cell_id": "fitter_reach_check", "description": "Robo Fitter reachability check",
        "assets": {"cells_root": tempfile.gettempdir()},
        "robots": robots,
        "collision": {"link_spheres": {"radius": 0.12, "fill_step": 0.12, "tcp_radius": 0.05},
                      "arm_arm": {"pair_radius": 0.12, "gate_m": 0.2},
                      "self_collision": {"geometry": "urdf_collision_hulls", "min_distance_m": 0.0}},
        "workpiece": {"rule": "none"},
        "obstacles": {"default_box_margin": 0.05, "box_hull": {"enabled": True, "min_distance_m": 0.0}, "boxes": [],
                      "floor": {"z": float(job.get("floor_z", 0.0)), "margin_m": 0.05, "sphere_rule": True,
                                "hull_rule": True}},
        "planning": {"velocity_scale": 0.7, "acceleration_rule": "vscale_squared", "approach_velocity_fraction": 0.4,
                     "home_velocity_scale": 0.4, "min_offset_m": 0.045, "executor_margin_s": 1.0, "check_dt": 0.0125,
                     "max_jerk_factor": None, "approach_offset_m": 0.1, "retract_offset_m": 0.1, "dwell_s": 0.5,
                     "sample_dt": 0.05},
        "tolerances": {"position_m": float(tol.get("position_m", 0.01)), "tool_axis_deg": float(tol.get("tool_axis_deg", 5.0)),
                       "entry_direction_deg": 15.0, "entry_span_m": 0.04, "spin_free": True},
    }


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _frame(v):
    """An orthonormal basis (columns) whose first axis is v."""
    x = _norm(v)
    helper = [1.0, 0.0, 0.0] if abs(x[0]) < 0.9 else [0.0, 1.0, 0.0]
    y = _norm(_cross(x, helper))
    z = _cross(x, y)
    return [[x[i], y[i], z[i]] for i in range(3)]


def tcp_rotation(axis_local, approach, spin_deg: float = 0.0):
    """The TCP frame's rotation (world) that points the tool axis (`axis_local`, TCP frame) along `approach`,
    turned by `spin_deg` about it: R @ axis_local = approach."""
    A, T = _frame(approach), _frame(axis_local)
    R = [[sum(A[i][k] * T[j][k] for k in range(3)) for j in range(3)] for i in range(3)]   # A @ T^T
    if spin_deg:
        a, c, s = _norm(approach), math.cos(math.radians(spin_deg)), math.sin(math.radians(spin_deg))
        K = [[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]]
        K2 = [[sum(K[i][k] * K[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        S = [[(1.0 if i == j else 0.0) + s * K[i][j] + (1 - c) * K2[i][j] for j in range(3)] for i in range(3)]
        R = [[sum(S[i][k] * R[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return R


def pair_line(robot: str, weld: str, result, elapsed: float, detail: str = "") -> dict:
    """An output line from a trajgen IKResult (or None: no solution)."""
    if result is None:
        return {"robot": robot, "weld": weld, "reachable": False, "q_deg": [], "reasons": ["no_solution"],
                "err": {}, "elapsed_s": round(elapsed, 3), "detail": detail or "the IK found no configuration"}
    err = {k: float(result.err[k]) for k in ("pos_m", "tool_axis_deg") if k in (result.err or {})}
    raw = list(result.reasons or [])
    return {"robot": robot, "weld": weld, "reachable": bool(result.valid),
            "q_deg": [round(math.degrees(float(v)), 4) for v in result.q],
            "reasons": [] if result.valid else normalise_reasons(raw) or ["no_solution"],
            "err": err, "elapsed_s": round(elapsed, 3), "detail": detail or ", ".join(str(r) for r in raw)}


def emit(out, value: dict) -> None:
    out.write(json.dumps(value) + "\n")
    out.flush()


def best(results):
    """The first valid result (the solver's preference order), else the closest one."""
    if not results:
        return None
    for r in results:
        if r.valid:
            return r
    return min(results, key=lambda r: (len(r.reasons or []), (r.err or {}).get("pos_m", 1e9)))


def solve_quick(ik, rid, p, approach, axis_local, np):
    try:
        from expert.ik import cluster
    except ImportError:          # an older planner: every converged configuration
        def cluster(qs, tol=0.15):
            return qs
    results = []
    for spin in (0.0, 180.0):
        R = ik.to_internal(np.array(tcp_rotation(axis_local, approach, spin)))
        qs = ik.solve_pose(rid, p, R)
        results += [ik.check(rid, q, p, R) for q in cluster(qs, 0.15)]
        if any(r.valid for r in results):
            break
    return best(results)


def solve_full(ik, rid, p, approach, axis_local, np):
    R = ik.to_internal(np.array(tcp_rotation(axis_local, approach)))
    return best(ik.solve_axis(rid, p=p, R=R, valid_only=False, approach=np.array(approach, float), standoff=0.10,
                              min_standoff=0.045))


def run(job: dict, planner_src: str, out) -> int:
    robots = job.get("robots") or []
    welds = job.get("welds") or []
    mode = job.get("mode", "quick")
    if mode not in ("quick", "full"):
        emit(out, {"error": f"unknown mode {mode!r}"})
        return 1
    upright = [r for r in robots if not any(abs(float(v)) > 1e-6 for v in r.get("tilt_bc_deg") or [0, 0])]
    for r in robots:
        if r not in upright:
            for w in welds:
                emit(out, {"robot": r["id"], "weld": w["id"], "reachable": False, "q_deg": [],
                           "reasons": ["base_tilt_unsupported"], "err": {}, "elapsed_s": 0.0,
                           "detail": "the IK takes upright bases only (orientation b, c = 0)"})
    if not upright or not welds:
        emit(out, {"done": True})
        return 0
    sys.path.insert(0, planner_src)
    os.chdir(planner_src)
    work = tempfile.mkdtemp(prefix="fos-reach-")
    try:
        import numpy as np
        import yaml
        from trajgen.ik import SceneIK
        from trajgen.scene import load_scene
        scene_file = os.path.join(work, "scene.yaml")
        with open(scene_file, "w") as f:
            yaml.safe_dump(scene_dict({**job, "robots": upright}), f, sort_keys=False)
        ik = SceneIK(load_scene(scene_file))
    except Exception as e:  # noqa: BLE001 - reported to the backend
        emit(out, {"error": f"{type(e).__name__}: {e}"})
        shutil.rmtree(work, ignore_errors=True)
        return 1
    try:
        solve = solve_full if mode == "full" else solve_quick
        for r in upright:
            axis_local = [float(x) for x in r["tcp"]["axis"]]
            for w in welds:
                t0 = time.time()
                p = np.array([float(x) for x in w["position"]])
                approach = _norm([float(x) for x in w["approach"]])
                try:
                    result = solve(ik, r["id"], p, approach, axis_local, np)
                    emit(out, pair_line(r["id"], w["id"], result, time.time() - t0))
                except Exception as e:  # noqa: BLE001 - one weld's failure is that weld's result
                    emit(out, {"robot": r["id"], "weld": w["id"], "reachable": False, "q_deg": [], "reasons": ["error"],
                               "err": {}, "elapsed_s": round(time.time() - t0, 3), "detail": f"{type(e).__name__}: {e}"})
        emit(out, {"done": True})
        return 0
    finally:
        shutil.rmtree(work, ignore_errors=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--planner-src", default="/opt/factoryos-cell-planner/router/src",
                    help="factoryos-cell-planner sources (trajgen, expert, robotmodel)")
    a = ap.parse_args(argv)
    try:
        job = json.load(sys.stdin)
    except ValueError as e:
        emit(sys.stdout, {"error": f"invalid job JSON: {e}"})
        return 1
    return run(job, a.planner_src, sys.stdout)


if __name__ == "__main__":
    sys.exit(main())
