"""Bounded assignment and route search with certified transfer costs.

Every parallel route pair is safe for independent clocks, including every
transfer and parked endpoint. Search bounds affect completeness only.
"""
from __future__ import annotations

from functools import lru_cache
import time

from .store import digest
from .transfers import key as transfer_key
from .validation import compatible


def within_phase_limits(cell, program, robot, transfer):
    """A geometric transfer certificate cannot override current phase speeds."""
    try:
        approach = next(p for p in program["extensions"]["weld_params"]["weld_phases"]
                        if p.get("phase") == "approach")
        ceiling = min(cell.config["planning"]["max_velocity_scaling"],
                      approach["speed_percentage"]/100)
        cfg, cert = cell.robots[robot], transfer["certificate"]
        for field, peak, scale in (
                ("joint_velocity_limits", "velocity_peak", ceiling),
                ("joint_acceleration_limits", "acceleration_peak", ceiling**2),
                ("joint_jerk_limits", "jerk_peak", 1.)):
            if not cfg.get(field):
                continue
            if len(cert[peak]) != len(cfg[field]) or any(
                    not 0. <= value <= limit*scale+1e-7
                    for value, limit in zip(cert[peak], cfg[field])):
                return False
        return True
    except (KeyError, TypeError, StopIteration):
        return False


def schedule(cell, program, options, store, planner, *, serial=False, budget_s=1.0,
             inactive=None, bystanders_clear=None):
    strategy = program["extensions"]["strategy"]
    cap = (strategy.get("max_concurrent_robots", len(cell.robots))
           if strategy.get("parallel_within_group") else 1)
    if serial:
        cap = 1
    dwell = sum(p["duration_s"] for p in program["extensions"]["weld_params"]["weld_phases"]
                if p["type"] in {"wait", "weld_pulse"})
    margin = cell.config["collision"]["min_clearance"]
    certificate_pairs, edge_cache = {}, {}
    batches, stats = [], {"route_states": 0, "parallel_candidates": 0, "search_truncated": False}
    deadline = time.monotonic() + max(0., budget_s)

    def edge(a, b):
        address = id(a), id(b)
        if address not in edge_cache:
            value = store.get("transfer", transfer_key(cell, a, b, planner))
            if value and value.get("status") == "certified":
                cert = value["certificate"]
                old = (a["phase_pack"]["points"]["home"][-1]["time_s"]
                       + b["phase_pack"]["points"]["approach"][-1]["time_s"])
                if (not cert.get("ok") or cert.get("geometry_hash") != cell.geometry_hash
                        or cert.get("trajectory_hash") != digest(value["points"])
                        or value["points"][-1]["time_s"] >= old
                        or not within_phase_limits(cell, program, a["robot"], value)
                        or (inactive and not bystanders_clear(cell, cert, a["robot"], inactive))):
                    value = None
            else:
                value = None
            edge_cache[address] = value
        return edge_cache[address]

    def cert_fits(a, b, robot_a, robot_b):
        address = tuple(sorted((id(a), id(b))))
        if address not in certificate_pairs:
            from .coordination_cache import accepts
            certificate_pairs[address] = accepts(
                cell, {"robot": robot_a, "certificate": a},
                {"robot": robot_b, "certificate": b}, store)
        return certificate_pairs[address]

    def route_fits(a, b):
        return all(cert_fits(x, y, a["robot"], b["robot"])
                   for x in a["certificates"] for y in b["certificates"])

    for group in strategy["sequence_groups_order"]:
        welds = sorted(o["id"] for o in program["operations"]
                       if o.get("sequence_group") == group and o["id"] in options)
        if not welds:
            continue
        bits = {w: 1 << i for i, w in enumerate(welds)}
        routes = {}
        # A complete HOME-return singleton incumbent exists even if the
        # bounded route expansion exhausts its budget.
        for robot in sorted({x["robot"] for w in welds for x in options[w]}):
            choices = [x for w in welds for x in options[w] if x["robot"] == robot]
            states = {}
            for index, item in enumerate(choices):
                mask = bits[item["weld"]]
                states[(mask, index)] = {
                    "mask": mask, "robot": robot, "items": (item,), "edges": (),
                    "cost": item["duration"] + dwell, "certificates": (item["certificate"],)}
            for size in range(1, len(welds)):
                current = [((mask, last), route) for (mask, last), route in states.items()
                           if mask.bit_count() == size]
                for (mask, _), route in current:
                    if time.monotonic() >= deadline:
                        stats["search_truncated"] = True
                        break
                    a = route["items"][-1]
                    for index, item in enumerate(choices):
                        if mask & bits[item["weld"]]:
                            continue
                        transfer = edge(a, item)
                        saving = 0.
                        certificates = route["certificates"] + (item["certificate"],)
                        if transfer:
                            saving = (a["phase_pack"]["points"]["home"][-1]["time_s"]
                                      + item["phase_pack"]["points"]["approach"][-1]["time_s"]
                                      - transfer["points"][-1]["time_s"])
                            certificates += (transfer["certificate"],)
                        candidate = {"mask": mask | bits[item["weld"]], "robot": robot,
                                     "items": route["items"] + (item,),
                                     "edges": route["edges"] + (transfer,),
                                     "cost": route["cost"] + item["duration"] + dwell - saving,
                                     "certificates": certificates}
                        address = candidate["mask"], index
                        if address not in states or candidate["cost"] < states[address]["cost"] - 1e-9:
                            states[address] = candidate
                if stats["search_truncated"]:
                    break
            stats["route_states"] += len(states)
            # Retain endpoint/branch alternatives. A slower route may have the
            # clearance needed to run alongside another arm.
            grouped = {}
            for route in states.values():
                grouped.setdefault(route["mask"], []).append(route)
            routes[robot] = [r for values in grouped.values()
                             for r in sorted(values, key=lambda x: x["cost"])[:6]]
        robots = sorted(routes)
        best = {}

        def remember(selected, mask):
            cost = max(r["cost"] for r in selected)
            previous = best.get(mask)
            if previous is None or cost < previous[0] - 1e-9:
                best[mask] = cost, tuple(selected)
            stats["parallel_candidates"] += 1

        for values in routes.values():
            for route in values:
                remember([route], route["mask"])

        def combine(selected, mask, start):
            if len(selected) >= cap:
                return
            for index in range(start, len(robots)):
                for route in routes[robots[index]]:
                    if time.monotonic() >= deadline:
                        stats["search_truncated"] = True
                        return
                    if mask & route["mask"] or not all(route_fits(route, x) for x in selected):
                        continue
                    extended = [*selected, route]
                    remember(extended, mask | route["mask"])
                    combine(extended, mask | route["mask"], index+1)
        combine([], 0, 0)

        @lru_cache(None)
        def solve(mask):
            if not mask:
                return 0., ()
            first = mask & -mask
            incumbent = None
            for subset, (cost, _) in best.items():
                if subset & first and subset & mask == subset:
                    tail_cost, tail = solve(mask ^ subset)
                    value = cost + tail_cost, (subset, *tail)
                    if incumbent is None or value[0] < incumbent[0] - 1e-9:
                        incumbent = value
            return incumbent

        _, masks = solve((1 << len(welds))-1)
        for mask in masks:
            batch = []
            for route in best[mask][1]:
                chain = [dict(x) for x in route["items"]]
                for index, transfer in enumerate(route["edges"]):
                    if transfer:
                        chain[index]["skip_home"] = True
                        chain[index+1]["transfer"] = transfer
                batch.extend(chain)
            batches.append(batch)
    return batches, stats


def transfer_records(batches):
    """Describe transfers already present in a route candidate."""
    records = []
    for batch in batches:
        previous = {}
        for item in batch:
            robot = item["robot"]
            if item.get("transfer"):
                a = previous[robot]
                transfer = item["transfer"]
                records.append({
                    "robot": robot, "from_weld": a["weld"], "to_weld": item["weld"],
                    "saved_motion_s": a["phase_pack"]["points"]["home"][-1]["time_s"]
                    + item["phase_pack"]["points"]["approach"][-1]["time_s"]
                    - transfer["points"][-1]["time_s"],
                    "certificate_hash": digest(transfer["certificate"])})
            previous[robot] = item
    return records


def apply_transfers(cell, batches, store, planner, *, program, terminal="home",
                    inactive=None, bystanders_clear=None, budget_s=.05):
    """Choose compatible transfers by complete batch makespan, not arm-local savings."""
    from .coordination_cache import accepts
    boundaries = []
    for batch in batches:
        chains = {}
        for item in batch:
            item.pop("transfer", None)
            item.pop("skip_home", None)
            chains.setdefault(item["robot"], []).append(item)
        for chain in chains.values():
            boundaries.extend(([a], [b], batch) for a, b in zip(chain, chain[1:]))
    for previous, following in zip(batches, batches[1:]):
        last = {x["robot"]: x for x in previous}
        first = {}
        for item in following:
            first.setdefault(item["robot"], item)
        boundaries.append((list(last.values()), list(first.values()), following))
    candidates, seen = [], set()
    batch_index = {id(batch): index for index, batch in enumerate(batches)}
    item_batch = {id(item): index for index, batch in enumerate(batches) for item in batch}
    for previous, following, active in boundaries:
        for a in previous:
            for b in following:
                if a["robot"] != b["robot"]:
                    continue
                address = id(a), id(b)
                if address in seen:
                    continue
                seen.add(address)
                transfer = store.get("transfer", transfer_key(cell, a, b, planner))
                if not transfer or transfer.get("status") != "certified":
                    continue
                cert = transfer["certificate"]
                if (not cert.get("ok") or cert.get("geometry_hash") != cell.geometry_hash
                        or cert.get("trajectory_hash") != digest(transfer["points"])
                        or not within_phase_limits(cell, program, a["robot"], transfer)):
                    continue
                saving = (a["phase_pack"]["points"]["home"][-1]["time_s"]
                          + b["phase_pack"]["points"]["approach"][-1]["time_s"]
                          - transfer["points"][-1]["time_s"])
                if saving <= 0:
                    continue
                if inactive and not bystanders_clear(cell, cert, a["robot"], inactive):
                    continue
                candidate = {"robot": a["robot"], "certificate": cert}
                if any(x["robot"] != a["robot"] and not accepts(cell, candidate, x, store)
                       for x in active):
                    continue
                candidates.append({"saving": saving, "a": a, "b": b, "transfer": transfer,
                                   "motion": candidate, "batch": batch_index[id(active)]})
    candidates.sort(key=lambda x: -x["saving"])
    conflicts = [0]*len(candidates)
    for i, a in enumerate(candidates):
        for j, b in enumerate(candidates[:i]):
            # Rest barriers separate destination batches. Within a batch,
            # different arms need a proof for every selected transfer pair.
            if (a["batch"] == b["batch"] and a["a"]["robot"] != b["a"]["robot"]
                    and not accepts(cell, a["motion"], b["motion"], store)):
                conflicts[i] |= 1 << j
                conflicts[j] |= 1 << i
    dwell = sum(p["duration_s"] for p in program["extensions"]["weld_params"]["weld_phases"]
                if p["type"] in {"wait", "weld_pulse"})
    durations = []
    for index, batch in enumerate(batches):
        totals, last = {}, {x["robot"]: x for x in batch}
        for item in batch:
            rid, phases = item["robot"], item["phase_pack"]["points"]
            duration = sum(phases[p][-1]["time_s"] for p in ("approach", "contact", "retract"))+dwell
            if terminal == "home" or index != len(batches)-1 or item is not last[rid]:
                duration += phases["home"][-1]["time_s"]
            totals[rid] = totals.get(rid, 0.)+duration
        durations.append(totals)
    best_cost = sum(max(row.values(), default=0.) for row in durations)
    best_mask, best_saving = 0, 0.
    deadline = time.monotonic()+max(0., budget_s)

    def adjust(candidate, sign):
        a, b, transfer = candidate["a"], candidate["b"], candidate["transfer"]
        rid = a["robot"]
        durations[item_batch[id(a)]][rid] -= sign*a["phase_pack"]["points"]["home"][-1]["time_s"]
        durations[item_batch[id(b)]][rid] += sign*(
            transfer["points"][-1]["time_s"]-b["phase_pack"]["points"]["approach"][-1]["time_s"])

    def search(index, mask, saving):
        nonlocal best_cost, best_mask, best_saving
        cost = sum(max(row.values(), default=0.) for row in durations)
        if cost < best_cost-1e-9 or (abs(cost-best_cost) <= 1e-9 and saving > best_saving):
            best_cost, best_mask, best_saving = cost, mask, saving
        if index == len(candidates) or time.monotonic() >= deadline:
            return
        if not mask & conflicts[index]:
            adjust(candidates[index], 1.)
            search(index+1, mask | (1 << index), saving+candidates[index]["saving"])
            adjust(candidates[index], -1.)
        search(index+1, mask, saving)

    search(0, 0, 0.)
    records = []
    for index, candidate in enumerate(candidates):
        if best_mask & (1 << index):
            a, b, transfer = candidate["a"], candidate["b"], candidate["transfer"]
            a["skip_home"], b["transfer"] = True, transfer
            records.append({"robot": a["robot"], "from_weld": a["weld"], "to_weld": b["weld"],
                            "saved_motion_s": candidate["saving"],
                            "certificate_hash": digest(transfer["certificate"])})
    return records
