"""Validate the actual phase tree's concurrency independently of schedule search."""
from collections import Counter

import numpy as np

from .coordination_cache import accepts
from .phase_schedule import envelope, fixed


def validate(cell, request, tree, batches, start_plan, geometry, store):
    items = {(x["robot"], x["weld"]): x for batch in batches for x in batch}
    states = {r: list(q) for r, q in start_plan["states"].items()}
    phases = cell.programs[request.program_id]["extensions"]["weld_params"]["weld_phases"]
    process_phases = [p["phase"] for p in phases if p["type"] in {"wait", "weld_pulse"}]
    strategy = cell.programs[request.program_id]["extensions"]["strategy"]
    cap = strategy.get("max_concurrent_robots", len(cell.robots)) if strategy.get("parallel_within_group") else 1
    group_order = strategy["sequence_groups_order"]
    weld_groups = {o["id"]: o["sequence_group"] for o in cell.programs[request.program_id]["operations"]}
    last_group = -1
    pair_checks = rounds = 0

    def leaves(node):
        if node["type"] in {"Sequence", "Parallel"}:
            return [leaf for child in node["children"] for leaf in leaves(child)]
        return [node]

    def clear(a, b):
        nonlocal pair_checks
        pair_checks += 1
        if a.get("source") and b.get("source") and accepts(cell, a["source"], b["source"], store):
            return True
        return accepts(cell, a, b, store)

    try:
        for node in tree["root"]["children"]:
            if node["id"].startswith("start_"):
                continue  # Independently checked by validate_export/start certificates.
            if node["type"] == "WeldBoundary":
                continue
            if node["type"] != "Parallel" or not node["id"].startswith("phase_batch_"):
                raise ValueError("unexpected phase-schedule node")
            active, motion_names, processes = {}, {}, {}
            for leaf in leaves(node):
                if leaf["type"] not in {"ExecuteTrajectory", "WeldPhase"}:
                    raise ValueError("unexpected phase leaf")
                ports, kind = leaf["ports"], leaf["type"]
                rid = ports["robot_ns"]
                if kind == "WeldPhase":
                    processes.setdefault((rid, ports["weld_id"]), []).append(ports["phase"])
                    continue
                weld, phase = ports["operation_id"].rsplit(":", 1)
                item = items[rid, weld]
                group_index = group_order.index(weld_groups[weld])
                if group_index < last_group:
                    raise ValueError("phase schedule violates process group order")
                last_group = group_index
                motion_names.setdefault((rid, weld), []).append(phase)
                actual = tree["blackboard"][ports["trajectory"][1:-1]]
                if np.max(abs(np.asarray(actual[0]["positions"])-states[rid])) > 1e-8:
                    raise ValueError("phase schedule has an actual-state gap")
                states[rid] = actual[-1]["positions"]
                # Retraction traverses exactly the certified contact curve.
                source = item["certificate"]
                points = item["phase_pack"]["points"]["contact" if phase == "retract" else phase]
                if phase == "approach" and item.get("transfer"):
                    source, points = item["transfer"]["certificate"], item["transfer"]["points"]
                active.setdefault(rid, []).append({
                    "robot": rid, "points": points,
                    "certificate": envelope(cell, geometry, rid, points, source, store),
                    "source": {"robot": rid, "certificate": source}})
            if len(active) > cap:
                raise ValueError("phase concurrency cap exceeded")
            for (rid, weld), names in motion_names.items():
                if ("contact" in names) != ("retract" in names):
                    raise ValueError("contact and retract must remain in one committed action")
                if "contact" in names and processes.get((rid, weld)) != process_phases:
                    raise ValueError("committed contact action omits or reorders weld phases")
            if any("contact" not in motion_names.get(k, []) for k in processes):
                raise ValueError("weld process separated from its contact/retraction action")
            robots = sorted(active)
            for i, rid in enumerate(robots):
                for other in robots[i+1:]:
                    if not all(clear(a, b) for a in active[rid] for b in active[other]):
                        raise ValueError("missing independent-clock phase-pair certificate")
                for other in cell.robots:
                    if other in active:
                        continue
                    rest = fixed(cell, geometry, other, states[other], store)
                    if not all(clear(a, rest) for a in active[rid]):
                        raise ValueError("phase path is not certified against a parked arm")
            rounds += 1
        return {"ok": True, "rounds": rounds, "pair_checks": pair_checks,
                "complete_process_actions": True, "independent_clocks": True}
    except (ValueError, KeyError) as exc:
        return {"ok": False, "reason": str(exc)}
