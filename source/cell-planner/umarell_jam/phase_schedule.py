"""Rest-boundary scheduling of approaches, complete weld strokes, and returns.

Parallel phases require independent-clock collision certificates. A complete
contact/squeeze/pulse/hold/retract sequence is an indivisible scheduling action.
"""
from __future__ import annotations

import copy
import hashlib
import heapq
from itertools import combinations, product
from pathlib import Path
import time

import numpy as np

from .coordination_cache import accepts, prepare as prepare_pair, dependencies as pair_key
from .store import digest
from .trajectory import Trajectory


def revision():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def envelope(cell, geometry, rid, points, source_certificate, store):
    """A conservative swept occupancy of an already certified controller curve."""
    dep = {"schema": "certified-phase-envelope-v1", "geometry": cell.geometry_hash,
           "robot": rid, "trajectory": digest(points), "source": digest(source_certificate),
           "checker": revision()}
    value = store.get("phase_envelope", dep)
    if value is not None:
        return value
    if not source_certificate.get("ok") or source_certificate["geometry_hash"] != cell.geometry_hash:
        raise ValueError("phase envelope needs a current source certificate")
    path = Trajectory(points)
    velocities = np.max(abs(path.segment_extrema(1)), axis=1)
    lower = upper = None
    intervals = 0
    for index, (a, b) in enumerate(zip(path.times[:-1], path.times[1:])):
        speed_bound = float(velocities[index] @ geometry.levers[rid])
        count = max(1, int(np.ceil((b-a)*speed_bound/.08)))
        times = np.linspace(a, b, count+1)
        for start, end in zip(times[:-1], times[1:]):
            bodies = geometry.place(rid, path.sample(float((start+end)/2)))[0]
            radius = speed_bound*(end-start)/2
            lo = np.array([x["lo"] for x in bodies])-radius
            hi = np.array([x["hi"] for x in bodies])+radius
            lower = lo if lower is None else np.minimum(lower, lo)
            upper = hi if upper is None else np.maximum(upper, hi)
            intervals += 1
    value = {"ok": True, "status": "certified", "geometry_hash": cell.geometry_hash,
             "trajectory_hash": digest(points), "source_certificate": digest(source_certificate),
             "occupancy": {"lo": lower.tolist(), "hi": upper.tolist()},
             "intervals": intervals, "checker": revision(),
             "proof": "source-certified subcurve with analytic velocity bounds"}
    return store.obtain("phase_envelope", dep, lambda: value)[0]


def fixed(cell, geometry, rid, q, store):
    dep = {"schema": "phase-rest-envelope-v1", "geometry": cell.geometry_hash,
           "robot": rid, "positions": list(q), "checker": revision()}
    value = store.get("phase_rest", dep)
    if value is not None:
        return value
    bodies = geometry.place(rid, q)[0]
    points = [{"time_s": t, "positions": list(q), "velocities": [0.]*len(q),
               "accelerations": [0.]*len(q)} for t in (0., 1.)]
    cert = {"ok": True, "geometry_hash": cell.geometry_hash, "trajectory_hash": digest(points),
            "occupancy": {"lo": [x["lo"].tolist() for x in bodies],
                          "hi": [x["hi"].tolist() for x in bodies]},
            "checker": revision(), "proof": "exact acknowledged rest configuration"}
    value = {"status": "certified", "robot": rid, "points": points, "certificate": cert}
    return store.obtain("phase_rest", dep, lambda: value)[0]


def tasks(cell, request, batches, tree, geometry, store):
    operations = {}
    def collect(node):
        if node.get("id", "").startswith("operation_"):
            operations[node["id"]] = node["children"]
        for child in node.get("children", []):
            collect(child)
    collect(tree["root"])
    program = cell.programs[request.program_id]
    groups = {o["id"]: o["sequence_group"] for o in program["operations"]}
    result = {g: {} for g in program["extensions"]["strategy"]["sequence_groups_order"]}
    for batch in batches:
        for item in batch:
            rid, weld = item["robot"], item["weld"]
            leaves = operations[f"operation_{rid}_{weld}"]
            curve = item["phase_pack"]["points"]
            chunks = [("approach", leaves[:1]), ("process", leaves[1:-1] if leaves[-1]["id"].endswith("_home") else leaves[1:])]
            if leaves[-1]["id"].endswith("_home"):
                chunks.append(("home", leaves[-1:]))
            for phase, nodes in chunks:
                source = item["certificate"]
                points = curve["contact" if phase == "process" else phase]
                if phase == "approach" and item.get("transfer"):
                    source, points = item["transfer"]["certificate"], item["transfer"]["points"]
                cert = envelope(cell, geometry, rid, points, source, store)
                duration, pulse_offset = 0., None
                end = points[-1]["positions"]
                for node in nodes:
                    if node["type"] == "ExecuteTrajectory":
                        p = tree["blackboard"][node["ports"]["trajectory"][1:-1]]
                        duration += p[-1]["time_s"]
                        end = p[-1]["positions"]
                    else:
                        duration += node["ports"]["duration_s"]
                        if node["ports"]["phase"] == "weld":
                            pulse_offset = duration
                task = {"robot": rid, "weld": weld, "phase": phase, "points": points,
                        "certificate": cert, "duration": duration, "end": end,
                        "pulse_offset": pulse_offset,
                        "retract_offset": duration if phase == "process" else None,
                        "sources": [{"robot": rid, "certificate": source}],
                        "nodes": nodes}
                result[groups[weld]].setdefault(rid, []).append(task)
    return result


def schedule(cell, request, batches, tree, start_plan, geometry, store, *,
             budget_s=.5, prepare=False, pair_budget_s=.5, serial=False):
    started = time.monotonic()
    task_groups = tasks(cell, request, batches, tree, geometry, store)
    strategy = cell.programs[request.program_id]["extensions"]["strategy"]
    cap = (strategy.get("max_concurrent_robots", len(cell.robots))
           if strategy.get("parallel_within_group") else 1)
    if serial:
        cap = 1
    states = {r: list(q) for r, q in start_plan["states"].items()}
    prefix = [node for node in tree["root"]["children"] if node["id"].startswith("start_")]
    elapsed = sum(max(tree["blackboard"][leaf["ports"]["trajectory"][1:-1]][-1]["time_s"]
                      for leaf in node["children"]) for node in prefix)
    children, interlocks, certificates = list(prefix), [], []
    final_weld = final_retract = simultaneous = 0.
    stats = {"expanded_states": 0, "pair_checks": 0, "pair_preparations": 0, "search_truncated": False}
    deadline = time.monotonic()+budget_s
    known = {}
    object_pairs = {}

    def clear(a, b):
        address = tuple(sorted((id(a), id(b))))
        if address not in object_pairs:
            object_pairs[address] = check_pair(a, b)
        return object_pairs[address]

    def check_pair(a, b):
        if a.get("sources") and b.get("sources") and all(
                accepts(cell, x, y, store) for x in a["sources"] for y in b["sources"]):
            return True
        if a.get("pieces") or b.get("pieces"):
            return all(clear(x, y) for x in a.get("pieces", [a]) for y in b.get("pieces", [b]))
        address = tuple(sorted(((a["robot"], digest(a["certificate"])),
                                (b["robot"], digest(b["certificate"])))))
        if address not in known:
            stats["pair_checks"] += 1
            ok = accepts(cell, a, b, store)
            if not ok and prepare:
                value, hit = prepare_pair(cell, geometry, a, b, store, pair_budget_s)
                stats["pair_preparations"] += not hit
                ok = value.get("status") == "certified"
            known[address] = ok
        return known[address]

    for group, queues in task_groups.items():
        if not queues:
            continue
        object_pairs.clear()
        robots = sorted(queues)
        goal = tuple(len(queues[r]) for r in robots)
        initial = (0,)*len(robots)
        choices = {}
        for rid, queue in queues.items():
            for index in range(len(queue)):
                choices[rid, index] = []
                for end in range(index+1, len(queue)+1):
                    pieces = queue[index:end]
                    if pieces[-1]["weld"] != queue[index]["weld"] and end != len(queue):
                        continue
                    duration = 0.
                    pulse_offset = retract_offset = None
                    sources = {}
                    for piece in pieces:
                        if piece["pulse_offset"] is not None:
                            pulse_offset = duration+piece["pulse_offset"]
                        if piece["retract_offset"] is not None:
                            retract_offset = duration+piece["retract_offset"]
                        duration += piece["duration"]
                        for source in piece["sources"]:
                            sources[digest(source)] = source
                    choices[rid, index].append({
                        "robot": rid, "weld": queue[index]["weld"],
                        "weld_ids": sorted({x["weld"] for x in pieces}),
                        "phase": "+".join(x["phase"] for x in pieces), "pieces": pieces,
                        "advance": end-index, "duration": duration, "end": pieces[-1]["end"],
                        "pulse_offset": pulse_offset, "retract_offset": retract_offset,
                        "sources": list(sources.values()),
                        "certificate": {"parts": [digest(x["certificate"]) for x in pieces]},
                        "nodes": [node for x in pieces for node in x["nodes"]]})
        parked = {}
        for rid in cell.robots:
            positions = [states[rid]]
            positions += [task["end"] for task in queues.get(rid, [])]
            parked[rid] = [fixed(cell, geometry, rid, q, store) for q in positions]
        # Preparation visits all pair combinations independently of which
        # subset of actions the online search will select.
        if prepare:
            for i, a in enumerate(robots):
                for b in list(cell.robots):
                    if a == b:
                        continue
                    for task in queues[a]:
                        for other in [*queues.get(b, []), *parked[b]]:
                            clear(task, other)
            deadline = time.monotonic()+budget_s
        distance, parent = {initial: 0.}, {}
        heap = [(0., initial)]
        best_goal = None
        while heap:
            cost, state = heapq.heappop(heap)
            if cost != distance[state]:
                continue
            if state == goal:
                best_goal = state
                break
            if time.monotonic() >= deadline or stats["expanded_states"] >= 20000:
                stats["search_truncated"] = True
                break
            stats["expanded_states"] += 1
            ready = [i for i, n in enumerate(state) if n < goal[i]]
            for size in range(1, min(cap, len(ready))+1):
                for selected in combinations(ready, size):
                    for active in product(*(choices[robots[i], state[i]] for i in selected)):
                        if any(not clear(a, b) for a, b in combinations(active, 2)):
                            continue
                        idle = [parked[r][state[robots.index(r)] if r in robots else 0]
                                for r in cell.robots if r not in {robots[i] for i in selected}]
                        if any(not clear(a, b) for a in active for b in idle):
                            continue
                        advances = {i: task["advance"] for i, task in zip(selected, active)}
                        next_state = tuple(n+advances.get(i, 0) for i, n in enumerate(state))
                        new_cost = cost+max(x["duration"] for x in active)
                        if new_cost < distance.get(next_state, float("inf"))-1e-9:
                            distance[next_state] = new_cost
                            parent[next_state] = state, active
                            heapq.heappush(heap, (new_cost, next_state))
        if best_goal is None:
            return None, {"reason": "phase_schedule_unknown", **stats,
                          "wall_s": time.monotonic()-started}
        steps, state = [], goal
        while state != initial:
            previous, active = parent[state]
            steps.append(active)
            state = previous
        for index, active in enumerate(reversed(steps)):
            group_children = []
            for task in active:
                if task["pulse_offset"] is not None:
                    final_weld = max(final_weld, elapsed+task["pulse_offset"])
                    final_retract = max(final_retract, elapsed+task["retract_offset"])
                group_children.append({"type": "Sequence",
                                       "id": f"phase_{group}_{index}_{task['robot']}",
                                       "children": task["nodes"]})
                states[task["robot"]] = task["end"]
            children.append({"type": "Parallel", "id": f"phase_batch_{group}_{index}",
                             "ports": {"success_count": -1, "failure_count": 1},
                             "children": group_children})
            if len(active) > 1:
                simultaneous += min(x["duration"] for x in active)
            elapsed += max(x["duration"] for x in active)
            certificates.append({"group": group, "step": index,
                                 "active": [{"robot": x["robot"], "weld": x["weld"], "phase": x["phase"],
                                             "weld_ids": x["weld_ids"],
                                             "certificate": digest(x["certificate"])} for x in active]})
        boundary = f"phase_group_{group}_ready"
        children.append({"type": "WeldBoundary", "id": boundary, "ports": {"boundary_id": boundary}})
        interlocks.append({"boundary": boundary, "type": "all_arms_acknowledged_rest",
                           "robots": robots, "proof": "independent-clock phase and parked-arm certificates"})
        deadline = time.monotonic()+budget_s
    result = copy.deepcopy(tree)
    result["root"]["children"] = children
    result["metadata"]["interlocks"] = interlocks
    result["metadata"]["phase_coordination"] = certificates
    timing = {"final_weld_s": final_weld, "final_retract_s": final_retract, "final_ready_s": elapsed,
              "simultaneous_execution_s": simultaneous}
    return (result, timing), {**stats, "wall_s": time.monotonic()-started}
