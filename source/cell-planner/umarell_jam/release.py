"""Final process, exported-interpolant and stroke constraints, independently checked."""
from __future__ import annotations

from collections import Counter
import hashlib
from pathlib import Path
import threading

import numpy as np
from scipy.spatial.transform import Rotation

from .store import digest
from .trajectory import Trajectory

_GEOMETRY = {}
_STROKE_LOCK = threading.RLock()


def stroke_proof(cell, item, store, pid=None):
    # Pinocchio data objects are mutable. Concurrent requests may share the
    # prepared geometry but must not update its FK placements simultaneously.
    with _STROKE_LOCK:
        return _stroke_proof(cell, item, store, pid)


def _stroke_proof(cell, item, store, pid=None):
    revision = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    programs = [cell.programs[pid]] if pid else cell.programs.values()
    op = next(o for p in programs for o in p["operations"] if o["id"] == item["weld"])
    dep = {"geometry": cell.geometry_hash, "trajectory": digest(item["points"]),
           "operation": op, "robot": item["robot"], "release_checker": revision}
    found = store.get("stroke", dep)
    if found is not None:
        return found
    from .geometry import Geometry
    if cell.geometry_hash not in _GEOMETRY:
        _GEOMETRY[cell.geometry_hash] = Geometry(cell)
    geometry = _GEOMETRY[cell.geometry_hash]
    rid = item["robot"]
    path = Trajectory(item["points"])
    start = item["certificate"]["contact_start_s"]
    axis = np.asarray(op["approach_vector"], dtype=float)
    axis /= np.linalg.norm(axis)
    expected = Rotation.from_quat([op["orientation"][k] for k in "xyzw"]).as_matrix()[:, 0]
    tolerance = np.radians(cell.robots[rid].get("tool_orientation", {}).get("axis_tolerance_deg", 5.))
    samples = np.unique(np.r_[np.arange(start, path.duration, .0025), path.times[path.times >= start], path.duration])
    velocities = np.max(abs(path.segment_extrema(1)), axis=1)
    peak_angle = peak_lateral = 0.
    for a, b in zip(samples[:-1], samples[1:]):
        mid = float((a+b)/2)
        segment = min(int(np.searchsorted(path.times, mid, side="right")) - 1, len(path.times)-2)
        vmax = velocities[segment]
        _, _, tcp = geometry.place(rid, path.sample(mid))
        rel = tcp.translation - op["position"]
        lateral = float(np.linalg.norm(rel - axis * (rel @ axis)))
        angle = float(np.arccos(np.clip(expected @ tcp.rotation[:, 0], -1., 1.)))
        peak_lateral, peak_angle = max(peak_lateral, lateral), max(peak_angle, angle)
        # Any link point and its tool axis remain inside these Lipschitz bounds.
        if lateral + float(vmax @ geometry.levers[rid]) * (b-a)/2 > .002:
            return {"ok": False, "reason": "continuous_stroke_lateral_unresolved"}
        if angle + float(np.sum(vmax)) * (b-a)/2 > tolerance:
            return {"ok": False, "reason": "continuous_stroke_axis_unresolved"}
    proof = {"ok": True, "trajectory_hash": dep["trajectory"], "samples": len(samples),
             "stroke_axis_max_deg": float(np.degrees(peak_angle)), "stroke_lateral_max_m": peak_lateral,
             "checker": revision, "continuous_bound": True}
    return store.obtain("stroke", dep, lambda: proof)[0]


def validate_export(cell, request, tree, batches, start_plan, store):
    """Certify the concrete tree shape; arbitrary later edits require a new release."""
    bb = tree["blackboard"]
    expected = {}
    for batch in batches:
        for item in batch:
            proof = stroke_proof(cell, item, store, request.program_id)
            if not proof["ok"]:
                return proof
            rid, weld = item["robot"], item["weld"]
            expected[(rid, weld)] = dict(item["phase_pack"]["points"])
            if item.get("transfer"):
                transfer = item["transfer"]
                if (not transfer["certificate"]["ok"]
                        or transfer["certificate"]["trajectory_hash"] != digest(transfer["points"])):
                    return {"ok": False, "reason": "transfer_certificate_mismatch"}
                expected[(rid, weld)]["approach"] = transfer["points"]
    for index, batch in enumerate(start_plan["batches"]):
        for item in batch:
            expected[(item["robot"], f"initial_{index}")] = {item["phase"]: item["points"]}
            if digest(item["points"]) != item["certificate"]["trajectory_hash"]:
                return {"ok": False, "reason": "connector_certificate_mismatch"}
    phases = [p for p in cell.programs[request.program_id]["extensions"]["weld_params"]["weld_phases"]
              if p["type"] in {"wait", "weld_pulse"}]
    process, motions, used = [], [], []
    states = {r: np.asarray(s["positions"]) for r, s in request.initial["robots"].items()}
    velocities = {r: np.asarray(s["velocities"]) for r, s in request.initial["robots"].items()}

    def walk(node):
        kind = node["type"]
        if kind in {"Sequence", "Parallel"}:
            if kind == "Parallel":
                armsets = []
                for child in node["children"]:
                    arms = set()
                    def collect(n):
                        if n.get("ports", {}).get("robot_ns"):
                            arms.add(n["ports"]["robot_ns"])
                        for c in n.get("children", []):
                            collect(c)
                    collect(child)
                    if any(arms & other for other in armsets):
                        raise ValueError("concurrent commands to the same arm")
                    armsets.append(arms)
            for child in node["children"]:
                walk(child)
            return
        ports = node.get("ports", {})
        if kind == "WeldBoundary":
            if any(np.max(abs(v)) > 1e-8 for v in velocities.values()):
                raise ValueError("nonrest release boundary")
            return
        rid = ports["robot_ns"]
        if rid not in request.enabled:
            raise ValueError("command sent to inactive arm")
        if kind == "ExecuteTrajectory":
            ref = ports["trajectory"]
            if not (isinstance(ref, str) and ref.startswith("{") and ref.endswith("}")):
                raise ValueError("trajectory must have a concrete blackboard reference")
            key = ref[1:-1]
            points = bb[key]
            used.append(key)
            weld, phase = ports["operation_id"].rsplit(":", 1)
            if digest(points) != digest(expected[(rid, weld)][phase]):
                raise ValueError("exported controller curve differs from certified curve")
            path = Trajectory(points)
            if path.n != 6:
                raise ValueError("installed Umarell robot driver supports six joints")
            if (not ports.get("controller_exact") or not ports.get("independent_clocks")
                    or ports["velocity_scale"] != 1. or ports["hold_duration_s"] != 0.):
                raise ValueError("controller would reinterpret certified points")
            if np.max(abs(states[rid] - path.sample(0))) > 1e-8:
                raise ValueError("position discontinuity at phase join")
            if np.max(abs(velocities[rid] - points[0]["velocities"])) > 1e-8:
                raise ValueError("velocity discontinuity at phase join")
            # The real exact ROS goal builder forces these final values to zero.
            if any(max(map(abs, points[-1][field])) > 1e-9 for field in ("velocities", "accelerations")):
                raise ValueError("controller endpoint clamp would alter certified polynomial")
            states[rid], velocities[rid] = np.asarray(points[-1]["positions"]), np.asarray(points[-1]["velocities"])
            motions.append((rid, weld, phase))
            return
        if kind == "WeldPhase":
            weld, phase = ports["weld_id"], ports["phase"]
            if not motions or (rid, weld, "contact") not in motions:
                raise ValueError("process before contact")
            if np.max(abs(states[rid] - expected[(rid, weld)]["contact"][-1]["positions"])) > 1e-8:
                raise ValueError("process away from certified weld pose")
            if np.max(abs(velocities[rid])) > 1e-9:
                raise ValueError("process while moving")
            spec = next(p for p in phases if p["phase"] == phase)
            if ports["duration_s"] != spec["duration_s"]:
                raise ValueError("process duration mismatch")
            from .phases import process_settings
            if ports["settings"] != process_settings(cell.programs[request.program_id]):
                raise ValueError("process settings mismatch")
            process.append((rid, weld, phase))
            return
        raise ValueError("unexpected node in released tree")

    try:
        walk(tree["root"])
        if Counter(used) != Counter(bb.keys()):
            raise ValueError("unused or reused trajectory reference")
        if Counter(w for _, w, p in process if p == "weld") != Counter(request.pending):
            raise ValueError("wrong pulse multiset")
        for rid, weld in {(r, w) for r, w, _ in process}:
            if [p for r, w, p in process if (r, w) == (rid, weld)] != [p["phase"] for p in phases]:
                raise ValueError("process phase order mismatch")
            actual = [p for r, w, p in motions if (r, w) == (rid, weld)]
            if actual not in (["approach", "contact", "retract"], ["approach", "contact", "retract", "home"]):
                raise ValueError("missing or reordered process motion")
        if request.raw["terminal_state"] == "home":
            for rid in request.enabled:
                if np.max(abs(states[rid] - np.radians(cell.robots[rid]["home_pose_deg"]))) > 1e-8:
                    raise ValueError("terminal HOME mismatch")
        if any(np.max(abs(velocities[r])) > 1e-8 for r in request.enabled):
            raise ValueError("terminal arm not at rest")
    except (ValueError, KeyError, TypeError, StopIteration) as exc:
        return {"ok": False, "reason": str(exc)}
    return {"ok": True, "exact_controller_payloads": len(used), "pulse_count": len(request.pending),
            "process_order": True, "continuous_stroke_constraints": True,
            "all_phase_joins": True, "terminal_rest": True}
