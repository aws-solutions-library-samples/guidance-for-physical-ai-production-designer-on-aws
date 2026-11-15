"""Routine request planner over immutable, certified components."""
from __future__ import annotations

from collections import Counter
from pathlib import Path
import json
import os
import threading
import time

import numpy as np

from .cell import Cell, CellLoadError
from .request import Request, InvalidRequest
from .store import Store, atomic_json, digest
from .runtime_store import RuntimeStore
from .preparation import dependencies, implementation_revision
from .native import code_revision
from .trajectory import Trajectory
from .validation import compatible


class _LazyGeometry:
    """Prepared phase proofs need no FK model until a component is missing."""
    def __init__(self, cell):
        self._cell, self._model = cell, None
        self._lock = threading.RLock()

    def __getattr__(self, name):
        with self._lock:
            if self._model is None:
                from .electrode_geometry import RefinedElectrodeGeometry
                self._model = RefinedElectrodeGeometry(self._cell)
            return getattr(self._model, name)


class PlannerService:
    def __init__(self, workspace, cache, umarell, output):
        self.workspace, self.umarell = Path(workspace).resolve(), Path(umarell).resolve()
        self.store = RuntimeStore(cache)
        self.output = Path(output).resolve()
        self.cells = {}
        self.paths = {}
        self.revision = code_revision(self.umarell)
        self.implementation = implementation_revision()
        import hashlib
        self.runtime_revision = digest({rel: hashlib.sha256((self.umarell / rel).read_bytes()).hexdigest()
                                        for rel in ("agents/cell/bt_executor.py",
                                                    "agents/robot/robot_agent.py",
                                                    "agents/trajectory/tools/btcpp.py")})
        self._lock = threading.RLock()
        self._geometry_lock = threading.RLock()
        self.geometries = {}
        self._versions = {}
        self.committed = {}
        self.boundaries = {}

    def load(self, cid):
        if cid not in self.cells or not self.cells[cid].unchanged():
            candidate = Cell(self.workspace, cid)
            from .capacity import validate_capacity
            validate_capacity(candidate)
            self.cells[cid] = candidate
        return self.cells[cid]

    def _bystanders_clear(self, cell, certificate, robot, states):
        from .connectors import bystanders_clear
        from .electrode_geometry import RefinedElectrodeGeometry
        with self._geometry_lock:
            cached = self.geometries.get(cell.cell_id)
            if cached is None or cached[0] != cell.geometry_hash:
                cached = cell.geometry_hash, RefinedElectrodeGeometry(cell)
                self.geometries[cell.cell_id] = cached
            return bystanders_clear(cached[1], certificate, robot, states)

    def acknowledge_boundary(self, version, boundary_id, measured_state, completed_ids):
        """Only the executor may call after acknowledging rest at the named boundary."""
        if version not in self.committed:
            raise InvalidRequest("unknown committed version")
        plan = self.committed[version]
        if boundary_id not in {b["boundary"] for b in plan["interlocks"]}:
            raise InvalidRequest("unknown boundary for committed version")
        if not set(completed_ids) <= set(plan["selected_weld_ids"]):
            raise InvalidRequest("acknowledged completed work outside committed selection")
        if any(any(abs(v) > 1e-8 for v in s["velocities"]) for s in measured_state["robots"].values()):
            raise InvalidRequest("boundary acknowledgement must be at rest")
        self.boundaries[(version, boundary_id)] = (digest(measured_state), tuple(sorted(completed_ids)))

    def commit(self, result):
        if result.get("status") != "success":
            raise InvalidRequest("cannot commit a non-executable result")
        import copy
        import hashlib
        tree = copy.deepcopy(result["behaviour_tree"])
        tree["metadata"].pop("version_hash", None)
        integration = digest({p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in sorted(Path(__file__).parent.glob("*.py"))})
        expected = digest({"request": result["request_hash"], "tree": tree,
                           "implementation": self.implementation, "planner": self.revision,
                           "runtime": self.runtime_revision, "integration": integration})
        if expected != result["version_hash"]:
            raise InvalidRequest("released tree or implementation changed before commit")
        for result_key, metadata_key in (
                ("selected_weld_ids", "selected_weld_ids"), ("completed_weld_ids", "completed_weld_ids"),
                ("enabled_robot_ids", "enabled_robot_ids"), ("interlocks", "interlocks")):
            if result[result_key] != tree["metadata"][metadata_key]:
                raise InvalidRequest("result metadata differs from released tree")
        cell = self.load(result["behaviour_tree"]["cell_id"])
        if cell.geometry_hash != result["scene_revision"]:
            raise InvalidRequest("cannot commit a plan for a stale scene")
        if digest(cell.program_keys[result["program_id"]]) != result["behaviour_tree"]["metadata"]["program_revision"]:
            raise InvalidRequest("cannot commit a plan for a stale program or motion profile")
        self.committed[result["version_hash"]] = copy.deepcopy(result)

    def plan(self, raw, *, timeout_s=30., tree_id=None, repair_budget_s=0., serial=False,
             candidate_filter=None, prepare_phase_pairs=False):
        started = time.perf_counter()
        import copy
        raw = copy.deepcopy(raw)
        stages = {}
        response = {"schema_version": "weld-result-v1", "request_id": raw.get("request_id") if isinstance(raw, dict) else None,
                    "executable": False, "whole_request_cache_hit": False,
                    "_cache_before": (self.store.hits, self.store.misses, self.store.corrupt)}
        try:
            if not isinstance(raw, dict) or not isinstance(raw.get("cell_id"), str):
                raise InvalidRequest("cell_id is required")
            from agents.trajectory.tools import plan_cancellation
            if plan_cancellation.is_cancelled():
                response.update(status="timeout_or_unknown", reason="cancelled")
                return self._finish(response, started, stages)
            cid = raw["cell_id"]
            with self._lock:
                generation = self._versions.get(cid, 0) + 1
                self._versions[cid] = generation
            cell = self.load(cid)
            request = Request.parse(raw, cell)
            stages["load_and_request_s"] = time.perf_counter() - started
            response.update(request_hash=request.request_hash, state_hash=request.state_hash,
                            scene_revision=cell.geometry_hash, selected_weld_ids=list(request.selected),
                            pending_weld_ids=list(request.pending), completed_weld_ids=list(request.completed),
                            enabled_robot_ids=list(request.enabled), program_id=request.program_id)
            if (not request.pending and raw["terminal_state"] == "hold"
                    and any(max(map(abs, s["velocities"])) > 1e-8
                            for s in request.initial["robots"].values())):
                response.update(status="timeout_or_unknown", reason="empty_hold_requires_acknowledged_rest")
                return self._finish(response, started, stages)
            if raw.get("update"):
                upd = raw["update"]
                ack = self.boundaries.get((upd["previous_version"], upd["boundary_id"]))
                if ack != (request.state_hash, request.completed):
                    raise InvalidRequest("replacement lacks matching executor boundary acknowledgement")
            if request.pending and repair_budget_s <= 0:
                # A routine cache miss cannot release a plan. Establish it
                # before spending time on exact-state connector certification.
                from .primitive_index import lookup
                unavailable = [weld for weld in request.pending if not any(
                    (lookup(self.store, cell, request.program_id, robot, weld,
                            self.revision, self.implementation)[0] or {}).get("status") == "certified"
                    for robot in request.enabled)]
                if unavailable:
                    response.update(status="timeout_or_unknown", reason="missing_certified_primitives",
                                    missing_weld_ids=unavailable, primitive_cache_hits=0)
                    return self._finish(response, started, stages)
            home = {r: np.radians(c["home_pose_deg"]) for r, c in cell.robots.items()}
            start_plan = {"ok": True, "batches": [], "certificates": [],
                          "states": {r: list(s["positions"]) for r, s in request.initial["robots"].items()}}
            measured_off_home = any(
                np.max(abs(np.array(s["positions"]) - home[r])) > 1e-8
                or max(map(abs, s["velocities"])) > 1e-8 for r, s in request.initial["robots"].items())
            if not request.pending and not measured_off_home:
                from .electrode_geometry import RefinedElectrodeGeometry, revision as electrode_revision
                dep = {"geometry": cell.geometry_hash, "states": start_plan["states"],
                       "checker": self.implementation, "electrode_checker": electrode_revision()}
                rest = self.store.get("rest_state", dep)
                if rest is None:
                    rest = {"ok": True}
                    hit = RefinedElectrodeGeometry(cell).check(start_plan["states"])
                    if hit:
                        rest = {"ok": False, "violation": hit}
                    rest = self.store.obtain("rest_state", dep, lambda: rest)[0]
                if not rest["ok"]:
                    response.update(status="timeout_or_unknown", reason="initial_scene_collision",
                                    connector_validation=rest)
                    return self._finish(response, started, stages)
            if measured_off_home:
                from .initial_connection import prepare
                t = time.perf_counter()
                start_plan = prepare(self, cell, request,
                    deadline=time.monotonic() + max(0., timeout_s - (time.perf_counter() - started)),
                    repair_budget_s=repair_budget_s)
                stages["actual_state_connectors_s"] = time.perf_counter() - t
                if not start_plan["ok"]:
                    response.update(status="timeout_or_unknown", reason=start_plan.get("reason"),
                                    connector_validation=start_plan)
                    return self._finish(response, started, stages)
            program = cell.programs[request.program_id]
            params = program["extensions"]["weld_params"]
            options = {}
            hit_count = 0
            for weld in request.pending:
                options[weld] = []
                for robot in request.enabled:
                    from .primitive_index import lookup
                    entry, dep = lookup(self.store, cell, request.program_id, robot, weld,
                                        self.revision, self.implementation)
                    if entry is None or entry["status"] != "certified":
                        continue
                    for variant in entry["variants"]:
                        cert = variant["certificate"]
                        if not (cert.get("ok") and cert.get("geometry_hash") == cell.geometry_hash
                                and cert.get("trajectory_hash") == digest(variant["points"])):
                            continue
                        hit_count += 1
                        options[weld].append({"weld": weld, "robot": robot, **variant,
                                              "key": digest(dep), "duration": variant["points"][-1]["time_s"]})
            missing = [w for w, variants in options.items() if not variants]
            if missing and repair_budget_s > 0:
                from .repair import repair
                t = time.perf_counter()
                repaired = repair(self, request, missing, min(repair_budget_s,
                    max(0., timeout_s - (time.perf_counter() - started))))
                stages["bounded_repair_s"] = time.perf_counter() - t
                response["repair"] = repaired
                for weld in missing:
                    for robot in request.enabled:
                        entry, dep = lookup(self.store, cell, request.program_id, robot, weld,
                                            self.revision, self.implementation)
                        if entry and entry["status"] == "certified":
                            for variant in entry["variants"]:
                                cert = variant["certificate"]
                                if (cert.get("ok") and cert.get("geometry_hash") == cell.geometry_hash
                                        and cert.get("trajectory_hash") == digest(variant["points"])):
                                    options[weld].append({"weld": weld, "robot": robot, **variant,
                                                         "key": digest(dep), "duration": variant["points"][-1]["time_s"]})
                missing = [w for w, variants in options.items() if not variants]
            inactive = {r: q for r, q in start_plan["states"].items()
                        if r not in request.enabled and np.max(abs(np.asarray(q) - home[r])) > 1e-8}
            if inactive:
                options = {w: [v for v in values if self._bystanders_clear(
                    cell, v["certificate"], v["robot"], inactive)] for w, values in options.items()}
                missing = [w for w, variants in options.items() if not variants]
            response["primitive_cache_hits"] = hit_count
            if missing:
                response.update(status="timeout_or_unknown", reason="missing_certified_primitives",
                                missing_weld_ids=missing)
                return self._finish(response, started, stages)
            from .phases import phase_pack
            from .release import stroke_proof
            t = time.perf_counter()
            rejected = []
            for weld, values in options.items():
                releasable = []
                for item in values:
                    try:
                        item["phase_pack"] = phase_pack(cell, request.program_id, item, self.store)
                        proof = stroke_proof(cell, item, self.store, request.program_id)
                        if not proof["ok"]:
                            raise ValueError(proof.get("reason"))
                    except ValueError as exc:
                        rejected.append({"weld": weld, "robot": item["robot"], "reason": str(exc)})
                        continue
                    item["duration"] = sum(p[-1]["time_s"] for p in item["phase_pack"]["points"].values())
                    releasable.append(item)
                options[weld] = releasable
            stages["phase_retime_s"] = time.perf_counter() - t
            response["rejected_candidates"] = rejected
            missing = [w for w, values in options.items() if not values]
            if missing:
                response.update(status="timeout_or_unknown", reason="no_releasable_primitive",
                                missing_weld_ids=missing)
                return self._finish(response, started, stages)
            if candidate_filter is not None:
                if not isinstance(candidate_filter, dict) or set(candidate_filter) != set(request.pending):
                    raise InvalidRequest("policy candidate filter must cover exactly the pending welds")
                for weld, allowed in candidate_filter.items():
                    if not isinstance(allowed, list) or not allowed:
                        raise InvalidRequest("each pending weld needs a nonempty policy candidate list")
                    identities = {(choice["robot_id"], choice["trajectory_hash"]) for choice in allowed}
                    selected = [item for item in options[weld]
                                if (item["robot"], digest(item["points"])) in identities]
                    if len({(item["robot"], digest(item["points"])) for item in selected}) != len(identities):
                        raise InvalidRequest("policy candidate is stale, disabled, or uncertified")
                    options[weld] = selected
                response["candidate_filter_applied"] = True
            t = time.perf_counter()
            batches = self._schedule(cell, program, options, raw["terminal_state"], serial=serial)
            from .route_schedule import apply_transfers, within_phase_limits
            transfers = apply_transfers(cell, batches, self.store, self.revision,
                                        program=program, terminal=raw["terminal_state"],
                                        inactive=inactive, bystanders_clear=self._bystanders_clear)
            if inactive:
                # Offline transfers were checked with other arms at HOME.
                # Their own swept occupancy must also clear actual inactive
                # poses; primitive-path clearance alone does not prove this.
                items = {(x["robot"], x["weld"]): x for batch in batches for x in batch}
                accepted = []
                for transfer in transfers:
                    rid = transfer["robot"]
                    destination = items[(rid, transfer["to_weld"])]
                    if self._bystanders_clear(cell, destination["transfer"]["certificate"], rid, inactive):
                        accepted.append(transfer)
                    else:
                        items[(rid, transfer["from_weld"])].pop("skip_home", None)
                        destination.pop("transfer")
                response["inactive_pose_transfer_rejections"] = len(transfers) - len(accepted)
                transfers = accepted
            stages["schedule_s"] = time.perf_counter() - t
            t = time.perf_counter()
            tree, timing, assignments = self._assemble(cell, request, batches, start_plan)
            if len(request.pending) > 1:
                from .route_schedule import schedule as route_schedule
                route_options = {
                    weld: [{k: v for k, v in item.items() if k not in {"transfer", "skip_home"}}
                           for item in values] for weld, values in options.items()}
                route_started = time.perf_counter()
                candidate, search = route_schedule(
                    cell, program, route_options, self.store, self.revision, serial=serial,
                    budget_s=min(.5, max(0., timeout_s - (time.perf_counter() - started) - 1.)),
                    inactive=inactive, bystanders_clear=self._bystanders_clear)
                # Recheck all internal transfers together with cross-boundary
                # transfers before accepting any altered parked configuration.
                for batch in candidate:
                    for item in batch:
                        item.pop("transfer", None)
                        item.pop("skip_home", None)
                candidate_transfers = apply_transfers(
                    cell, candidate, self.store, self.revision,
                    program=program, terminal=raw["terminal_state"],
                    inactive=inactive, bystanders_clear=self._bystanders_clear)
                if inactive:
                    candidate_items = {(x["robot"], x["weld"]): x for batch in candidate for x in batch}
                    accepted = []
                    for transfer in candidate_transfers:
                        rid = transfer["robot"]
                        destination = candidate_items[(rid, transfer["to_weld"])]
                        if self._bystanders_clear(cell, destination["transfer"]["certificate"], rid, inactive):
                            accepted.append(transfer)
                        else:
                            candidate_items[(rid, transfer["from_weld"])].pop("skip_home", None)
                            destination.pop("transfer")
                    candidate_transfers = accepted
                candidate_tree, candidate_timing, candidate_assignments = self._assemble(
                    cell, request, candidate, start_plan)
                response["route_search"] = {
                    **search, "baseline_ready_s": timing["final_ready_s"],
                    "candidate_ready_s": candidate_timing["final_ready_s"], "selected": False}
                if candidate_timing["final_ready_s"] < timing["final_ready_s"] - 1e-8:
                    batches, tree, timing, assignments = (
                        candidate, candidate_tree, candidate_timing, candidate_assignments)
                    transfers = candidate_transfers
                    response["route_search"]["selected"] = True
                stages["route_search_s"] = time.perf_counter() - route_started
            if tree_id:
                tree["tree_id"] = tree_id
            stages["retime_interlock_export_s"] = time.perf_counter() - t - stages.get("route_search_s", 0.)
            cycle = program["extensions"].get("cycle_time", {})
            response["cycle_time_constraints"] = cycle
            if len(request.pending) > 1 and (prepare_phase_pairs or (
                    cycle.get("max_s") is not None and timing["final_ready_s"] > cycle["max_s"])):
                from .phase_schedule import schedule as phase_schedule
                phase_started = time.perf_counter()
                with self._geometry_lock:
                    cached = self.geometries.get(cell.cell_id)
                    if cached is None or cached[0] != cell.geometry_hash:
                        cached = cell.geometry_hash, _LazyGeometry(cell)
                        self.geometries[cell.cell_id] = cached
                    phased, phase_search = phase_schedule(
                        cell, request, batches, tree, start_plan, cached[1], self.store,
                        budget_s=min(.5, max(0., timeout_s-(time.perf_counter()-started)-1.)),
                        prepare=prepare_phase_pairs, serial=serial)
                response["phase_search"] = {**phase_search, "selected": False}
                if phased is not None:
                    phase_tree, phase_timing = phased
                    response["phase_search"]["candidate_ready_s"] = phase_timing["final_ready_s"]
                    if phase_timing["final_ready_s"] < timing["final_ready_s"]-1e-8:
                        tree, timing = phase_tree, phase_timing
                        response["phase_search"]["selected"] = True
                stages["phase_schedule_s"] = time.perf_counter()-phase_started
            if cycle.get("max_s") is not None and timing["final_ready_s"] > cycle["max_s"] + 1e-8:
                response.update(status="timeout_or_unknown", reason="native_cycle_time_limit_not_met",
                                timing=timing, assignments=assignments)
                return self._finish(response, started, stages)
            from agents.trajectory.tools.behaviour_tree import validate_behaviour_tree
            from agents.trajectory.tools.btcpp import to_xml, from_xml
            t = time.perf_counter()
            structural = validate_behaviour_tree.func(tree)
            parsed = from_xml(to_xml(tree), blackboard=tree["blackboard"])
            roundtrip = validate_behaviour_tree.func(parsed)
            pulses = []
            for batch in batches:
                for item in batch:
                    if digest(item["points"]) != item["certificate"]["trajectory_hash"]:
                        raise ValueError("exported motion does not match certificate")
                    if item.get("transfer") and not within_phase_limits(
                            cell, program, item["robot"], item["transfer"]):
                        raise ValueError("transfer exceeds current approach-phase limits")
                    pulses.append(item["weld"])
            if Counter(pulses) != Counter(request.pending):
                raise ValueError("exact selected-work validation failed")
            if not structural["valid"] or not roundtrip["valid"]:
                raise ValueError(f"BT loader/XML roundtrip failed: {structural}, {roundtrip}")
            from .release import validate_export
            export_validation = validate_export(cell, request, tree, batches, start_plan, self.store)
            if not export_validation["ok"]:
                raise ValueError(f"export validation failed: {export_validation}")
            if tree["metadata"].get("phase_coordination"):
                from .phase_validation import validate as validate_phase_tree
                with self._geometry_lock:
                    phase_validation = validate_phase_tree(
                        cell, request, tree, batches, start_plan, self.geometries[cell.cell_id][1], self.store)
                if not phase_validation["ok"]:
                    raise ValueError(f"phase concurrency validation failed: {phase_validation}")
                export_validation["phase_coordination"] = phase_validation
            from .execution import simulate
            simulated = simulate(cell, tree, jitter_s=.02, slowdown=1.05)
            if (not simulated["success"]
                    or Counter(simulated["process_completed"]) != Counter(request.pending)):
                failure_path = self.output.parent / "execution-failures" / f"{request.request_hash}.json"
                atomic_json(failure_path, simulated)
                response["execution_failure_artifact"] = str(failure_path)
                raise ValueError(f"simulated execution failed: {simulated.get('failure', 'pulse mismatch')}")
            if raw["terminal_state"] == "home" and any(
                    np.max(abs(np.array(simulated["final_positions"][r]) - home[r])) > 1e-6
                    for r in request.enabled):
                raise ValueError("simulated terminal HOME mismatch")
            if any(max(map(abs, simulated["final_velocities"][r])) > 1e-8 for r in request.enabled):
                raise ValueError("simulated terminal velocity is not at rest")
            stages["final_checks_s"] = time.perf_counter() - t
            if time.perf_counter() - started > timeout_s:
                response.update(status="timeout_or_unknown", reason="request_deadline")
                return self._finish(response, started, stages)
            import hashlib
            integration_revision = digest({p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                           for p in sorted(Path(__file__).parent.glob("*.py"))})
            version = digest({"request": request.request_hash, "tree": tree,
                              "implementation": self.implementation, "planner": self.revision,
                              "runtime": self.runtime_revision, "integration": integration_revision})
            tree["metadata"]["version_hash"] = version
            # Publish only the current generation; stale workers cannot install a plan.
            publication_started = time.perf_counter()
            with self._lock:
                if plan_cancellation.is_cancelled():
                    response.update(status="timeout_or_unknown", reason="cancelled")
                    return self._finish(response, started, stages)
                if self._versions[cid] != generation:
                    response.update(status="timeout_or_unknown", reason="superseded_request")
                    return self._finish(response, started, stages)
                if not cell.unchanged():
                    response.update(status="timeout_or_unknown", reason="scene_or_program_changed_during_request")
                    return self._finish(response, started, stages)
                import tempfile
                import shutil
                import hashlib
                self.output.mkdir(parents=True, exist_ok=True)
                folder = self.output / version
                pending = Path(tempfile.mkdtemp(prefix=".pending-", dir=self.output))
                try:
                    atomic_json(pending / "bt.json", tree)
                    atomic_json(pending / "blackboard.json", tree["blackboard"])
                    with (pending / "bt.xml").open("w") as stream:
                        stream.write(to_xml(tree))
                        stream.flush()
                        os.fsync(stream.fileno())
                    checksums = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in pending.iterdir()}
                    atomic_json(pending / "manifest.json", {
                        "schema_version": "weld-plan-bundle-v1", "version_hash": version,
                        "files": checksums, "request_hash": request.request_hash,
                        "geometry_hash": cell.geometry_hash, "validated": True})
                    if folder.exists():
                        previous = json.loads((folder / "manifest.json").read_text())
                        if previous["files"] != checksums:
                            raise ValueError("conflicting immutable plan publication")
                    else:
                        os.replace(pending, folder)
                finally:
                    if pending.exists():
                        shutil.rmtree(pending)
            stages["publication_s"] = time.perf_counter() - publication_started
            response.update(status="success", executable=True, version_hash=version,
                            assignments=assignments, timing=timing, bt_json=str(folder / "bt.json"),
                            directed_transfers=transfers,
                            bt_xml=str(folder / "bt.xml"), behaviour_tree=tree,
                            effective_limits=cell.robots, process_settings=tree["metadata"]["process"],
                            final_validation={"ok": True, "bt_loader": structural, "xml_loader": roundtrip,
                                              "geometry_proof": "immutable primitive certificates and all-clock swept occupancy compatibility",
                                              "replay_hz": 400, "certificate_reuse": True,
                                              "export_validation": export_validation,
                                              "simulated_execution": {k: v for k, v in simulated.items() if k != "events"}},
                            geometry_cache_provenance={"geometry": cell.geometry_hash, "planner": self.revision,
                                                       "implementation": self.implementation},
                            interlocks=tree["metadata"]["interlocks"])
        except (InvalidRequest, CellLoadError, KeyError, TypeError) as exc:
            response.update(status="invalid_request", reason=str(exc))
        except Exception as exc:
            response.update(status="timeout_or_unknown", reason=repr(exc))
        return self._finish(response, started, stages)

    def _finish(self, response, start, stages):
        before = response.pop("_cache_before")
        response["cache_accesses"] = dict(zip(("hits", "misses", "corrupt"),
            (now - old for now, old in zip((self.store.hits, self.store.misses, self.store.corrupt), before))))
        elapsed = time.perf_counter() - start
        response["latency"] = {**stages, "elapsed_s": elapsed,
                              "time_to_first_validated_bt_s": elapsed if response.get("status") == "success" else None}
        return response

    def _schedule(self, cell, program, options, terminal, *, serial=False):
        from functools import lru_cache
        from .coordination_cache import accepts
        strategy = program["extensions"]["strategy"]
        cap = strategy.get("max_concurrent_robots", len(cell.robots)) if strategy.get("parallel_within_group") else 1
        if serial:
            cap = 1
        batches = []
        fit_cache = {}
        dwell = sum(p["duration_s"] for p in program["extensions"]["weld_params"]["weld_phases"]
                    if p["type"] in {"wait", "weld_pulse"})

        def fits(a, b):
            key = tuple(sorted((id(a), id(b))))
            if key not in fit_cache:
                fit_cache[key] = accepts(cell, a, b, self.store)
            return fit_cache[key]

        for group in strategy["sequence_groups_order"]:
            welds = sorted(o["id"] for o in program["operations"]
                           if o.get("sequence_group") == group and o["id"] in options)
            if not welds:
                continue
            bits = {w: 1 << i for i, w in enumerate(welds)}
            choices = sorted([item for w in welds for item in options[w]],
                             key=lambda x: (x["weld"], x["duration"], x["robot"]))
            best_for_mask = {}

            def remember(batch):
                mask = sum(bits[x["weld"]] for x in batch)
                cost = max(x["duration"] for x in batch) + dwell
                value = cost, sum(x["duration"] for x in batch), batch
                old = best_for_mask.get(mask)
                if old is None or value[:2] < old[:2]:
                    best_for_mask[mask] = value

            # A complete serial incumbent and useful greedy parallel packs are
            # available before the bounded clique enumeration.
            for item in choices:
                remember([item])
            for seed in choices:
                batch = [seed]
                for item in sorted(choices, key=lambda x: (x["duration"], x["weld"], x["robot"])):
                    if len(batch) >= cap:
                        break
                    if any(x["weld"] == item["weld"] or x["robot"] == item["robot"] for x in batch):
                        continue
                    if all(fits(x, item) for x in batch):
                        batch.append(item)
                remember(batch)
            deadline = time.monotonic() + .2

            def enumerate_batches(batch, start, mask, arms):
                if len(batch) >= cap or time.monotonic() >= deadline:
                    return
                for index in range(start, len(choices)):
                    item = choices[index]
                    if mask & bits[item["weld"]] or item["robot"] in arms:
                        continue
                    if all(fits(other, item) for other in batch):
                        extended = [*batch, item]
                        remember(extended)
                        enumerate_batches(extended, index+1, mask | bits[item["weld"]], arms | {item["robot"]})
                    if time.monotonic() >= deadline:
                        return
            enumerate_batches([], 0, 0, set())

            @lru_cache(None)
            def solve(mask):
                if mask == 0:
                    return 0., ()
                first = mask & -mask
                best = None
                for subset, (cost, _, batch) in best_for_mask.items():
                    if subset & first and subset & mask == subset:
                        tail_cost, tail = solve(mask ^ subset)
                        value = cost + tail_cost, (subset, *tail)
                        if best is None or value[0] < best[0] - 1e-10:
                            best = value
                return best
            _, order = solve((1 << len(welds)) - 1)
            batches.extend(best_for_mask[mask][2] for mask in order)
        groups = {o["id"]: o.get("sequence_group") for o in program["operations"]}
        compacted = []
        for batch in batches:
            if compacted:
                previous = compacted[-1]
                combined = previous + batch
                if (groups[previous[0]["weld"]] == groups[batch[0]["weld"]]
                        and len({x["robot"] for x in combined}) <= cap
                        and all(a["robot"] == b["robot"] or fits(a, b)
                                for i, a in enumerate(combined) for b in combined[i+1:])):
                    previous.extend(batch)
                    continue
            compacted.append(list(batch))
        return compacted

    def _assemble(self, cell, request, batches, start_plan):
        from agents.trajectory.tools.btcpp import to_xml
        program = cell.programs[request.program_id]
        params = program["extensions"]["weld_params"]
        from .phases import process_settings
        settings = process_settings(program)
        waits = [p for p in params["weld_phases"] if p["type"] in {"wait", "weld_pulse"}]
        bb, children, assignments, interlocks = {}, [], {}, []
        elapsed = final_weld = final_retract = 0.
        simultaneous = 0.

        def motion(rid, weld, phase, pts):
            key = f"{rid}_{weld}_{phase}"
            bb[key] = pts
            return {"type": "ExecuteTrajectory", "id": key, "ports": {
                "robot_ns": rid, "operation_id": f"{weld}:{phase}", "operation_type": "move",
                "trajectory": "{" + key + "}", "hold_duration_s": 0.,
                "velocity_scale": 1., "controller_exact": True,
                "independent_clocks": True,
                "initial_velocity": pts[0]["velocities"]}}

        for index, batch in enumerate(start_plan["batches"]):
            children.append({"type": "Parallel", "id": f"start_{index}", "ports": {
                "success_count": -1, "failure_count": 1}, "children": [
                    motion(item["robot"], f"initial_{index}", item["phase"], item["points"]) for item in batch]})
            elapsed += max(item["points"][-1]["time_s"] for item in batch)

        for k, batch in enumerate(batches):
            rows, ready = {}, {}
            last_item = {x["robot"]: x for x in batch}
            for item in batch:
                rid, weld = item["robot"], item["weld"]
                phases = item["phase_pack"]["points"]
                approach, contact, back_contact, back_approach = (
                    phases[p] for p in ("approach", "contact", "retract", "home"))
                if item.get("transfer"):
                    approach = item["transfer"]["points"]
                leaves = [motion(rid, weld, "approach", approach), motion(rid, weld, "contact", contact)]
                at = ready.get(rid, elapsed) + approach[-1]["time_s"] + contact[-1]["time_s"]
                for wait in waits:
                    leaves.append({"type": "WeldPhase", "id": f"{rid}_{weld}_{wait['phase']}",
                                   "ports": {"robot_ns": rid, "weld_id": weld, "phase": wait["phase"],
                                             "duration_s": wait["duration_s"], "settings": settings}})
                    at += wait["duration_s"]
                    if wait["phase"] == "weld":
                        final_weld = max(final_weld, at)
                leaves.append(motion(rid, weld, "retract", back_contact))
                at += back_contact[-1]["time_s"]
                final_retract = max(final_retract, at)
                # Return to the certified ready boundary after every operation.
                if not item.get("skip_home") and (
                        request.raw["terminal_state"] == "home" or k != len(batches) - 1 or item is not last_item[rid]):
                    leaves.append(motion(rid, weld, "home", back_approach))
                    at += back_approach[-1]["time_s"]
                ready[rid] = at
                rows.setdefault(rid, []).append({"type": "Sequence", "id": f"operation_{rid}_{weld}", "children": leaves})
                assignments[weld] = rid
            children.append({"type": "Parallel", "id": f"batch_{k}", "ports": {
                "success_count": -1, "failure_count": 1}, "children": [
                    {"type": "Sequence", "id": f"route_{k}_{rid}", "children": route}
                    for rid, route in rows.items()]})
            boundary = f"batch_{k}_ready"
            children.append({"type": "WeldBoundary", "id": boundary,
                             "ports": {"boundary_id": boundary}})
            interlocks.append({"boundary": boundary, "type": "all_arms_acknowledged_rest",
                               "robots": sorted(rows),
                               "proof": "pairwise conservative swept occupancy under independent clocks"})
            if len(rows) > 1:
                simultaneous += min(ready.values()) - elapsed
            elapsed = max(ready.values())
        tree = {"tree_id": "weld_" + request.request_hash[:20], "cell_id": cell.cell_id,
                "BTCPP_format": "4", "root": {"type": "Sequence", "id": "program", "children": children},
                "blackboard": bb, "metadata": {"schema_version": "weld-request-v1",
                    "selected_weld_ids": list(request.selected), "pending_weld_ids": list(request.pending),
                    "request_hash": request.request_hash, "scene_revision": cell.geometry_hash,
                    "program_id": request.program_id,
                    "program_revision": digest(cell.program_keys[request.program_id]),
                    "initial_state": request.initial, "terminal_state": request.raw["terminal_state"],
                    "enabled_robot_ids": list(request.enabled),
                    "completed_weld_ids": list(request.completed),
                    "component_provenance": [
                        {"weld": item["weld"], "robot": item["robot"], "key": item["key"],
                         "trajectory_hash": digest(item["points"]),
                         "certificate_hash": digest(item["certificate"]),
                         "phase_checks": item["phase_pack"]["checks"],
                         "transfer_certificate": digest(item["transfer"]["certificate"]) if item.get("transfer") else None}
                        for batch in batches for item in batch],
                    "initial_connector_certificates": start_plan["certificates"],
                    "effective_limits": {r: {k: v for k, v in c.items() if k.startswith("joint_")}
                                         for r, c in cell.robots.items()},
                    "interlocks": interlocks, "process": settings,
                    "requires": ["weld-cache-integration-v1", "acknowledged-weld-process-driver"]}}
        return tree, {"final_weld_s": final_weld, "final_retract_s": final_retract, "final_ready_s": elapsed,
                      "simultaneous_execution_s": simultaneous}, assignments


_SERVICE = None


def native_entry(cell_id, operations, constraints, tree_id=None):
    """Called inside Umarell's existing per-cell lock and cancellation scope."""
    global _SERVICE
    started = time.perf_counter()
    raw = constraints["weld_request"]
    if raw.get("cell_id") != cell_id or set(raw.get("weld_ids", [])) != {o["id"] for o in operations}:
        return {"success": False, "status": "invalid_request", "error": "operation/request mismatch"}
    import math
    timeout = constraints.get("weld_timeout_s", 30.)
    repair_budget = constraints.get("weld_repair_budget_s", 0.)
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) or v < 0 for v in (timeout, repair_budget)):
        return {"success": False, "status": "invalid_request", "error": "invalid weld planning budget"}
    if _SERVICE is None:
        _SERVICE = PlannerService(os.environ["WELD_WORKSPACE"], os.environ["WELD_COMPONENT_CACHE"],
                                  os.environ["WELD_UMARELL_ROOT"], os.environ["WELD_OUTPUT"])
    from agents.trajectory.tools import plan_cancellation
    result = _SERVICE.plan(raw, tree_id=tree_id, timeout_s=timeout, repair_budget_s=repair_budget)
    if callable(getattr(plan_cancellation, "is_cancelled", None)) and plan_cancellation.is_cancelled():
        return {"success": False, "status": "timeout_or_unknown", "error": "cancelled"}
    if result["status"] == "success":
        from agents.trajectory.tools.plan_store import save_plan
        if save_plan(cell_id, result["behaviour_tree"]) is None:
            return {"success": False, "status": "timeout_or_unknown", "error": "plan_store_failed"}
    result["latency"]["elapsed_s"] = time.perf_counter() - started
    result["latency"]["time_to_first_validated_bt_s"] = (
        result["latency"]["elapsed_s"] if result["status"] == "success" else None)
    return {**result, "success": result["status"] == "success",
            "planning_time_s": result["latency"]["elapsed_s"]}
