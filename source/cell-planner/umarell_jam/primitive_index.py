"""A per-weld dependency index over the original program-scoped preparation store."""
import hashlib
from pathlib import Path
from .preparation import dependencies


def reuse_key(cell, pid, rid, weld, planner, implementation):
    program = cell.programs[pid]
    op = next(o for o in program["operations"] if o["id"] == weld)
    params = program["extensions"]["weld_params"]
    return {"schema": "primitive-weld-index-v1", "geometry": cell.geometry_hash,
            "electrode_checker": hashlib.sha256(Path(__file__).with_name("electrode_geometry.py").read_bytes()).hexdigest(),
            "limits": cell.limits_hash, "robot": rid, "planner": planner,
            "implementation": implementation, "home": cell.robots[rid]["home_pose_deg"],
            "operation": {k: v for k, v in op.items() if k in {
                "id", "type", "position", "orientation", "weld_orientation", "approach_vector"}},
            "approach": params["approach_offset_z"], "retract": params["retract_offset_z"],
            "phase_speeds": {p["phase"]: p.get("speed_percentage") for p in params["weld_phases"]
                             if p["type"] == "move"}}


def supplemental_key(cell, pid, rid, weld, planner, implementation):
    return {"schema": "primitive-supplement-v1",
            "base": reuse_key(cell, pid, rid, weld, planner, implementation),
            "route_builder": hashlib.sha256(Path(__file__).with_name("branch_routes.py").read_bytes()).hexdigest()}


def lookup(store, cell, pid, rid, weld, planner, implementation):
    original, original_key = _branch_lookup(store, cell, pid, rid, weld, planner, implementation)
    from .seed_repair import key as placement_key
    from .store import digest
    moved_key = placement_key(cell, pid, rid, weld, planner, implementation)
    moved = store.get("placement_primitive", moved_key)
    if moved and moved.get("status") == "certified":
        if not original or original.get("status") != "certified":
            original, original_key = moved, moved_key
        else:
            variants = {digest(x["points"]): x for value in (original, moved) for x in value["variants"]}
            original_key = {"original": original_key, "placement": moved_key,
                            "original_payload": digest(original), "placement_payload": digest(moved)}
            original = {**original, "variants": list(variants.values())}
    from .contact_refinement import key as refinement_key
    from .store import digest
    extra_key = refinement_key(cell, pid, rid, weld, planner, implementation)
    extra = store.get("contact_refinement", extra_key)
    if extra is None or extra.get("status") != "certified":
        return original, original_key
    key = {"schema": "refined-primitive-index-v1", "original": original_key,
           "original_payload": digest(original), "refinement": extra_key,
           "refinement_payload": digest(extra)}
    found = store.get("primitive_index", key)
    if found is not None:
        return found, key
    variants = {digest(x["points"]): x for value in (original, extra)
                if value and value.get("status") == "certified" for x in value["variants"]}
    payload = {**extra, "variants": list(variants.values()),
               "source_dependencies": [original_key, extra_key]}
    return store.obtain("primitive_index", key, lambda: payload)[0], key


def _branch_lookup(store, cell, pid, rid, weld, planner, implementation):
    original, original_key = _original_lookup(store, cell, pid, rid, weld, planner, implementation)
    extra_key = supplemental_key(cell, pid, rid, weld, planner, implementation)
    extra = store.get("primitive_supplement", extra_key)
    if extra is None or extra.get("status") != "certified":
        return original, original_key
    from .store import digest
    key = {"schema": "combined-primitive-index-v1", "original": original_key,
           "original_payload": digest(original), "supplement": extra_key,
           "supplement_payload": digest(extra)}
    found = store.get("primitive_index", key)
    if found is not None:
        return found, key
    variants = {}
    for value in (original, extra):
        if value and value.get("status") == "certified":
            for variant in value["variants"]:
                variants[digest(variant["points"])] = variant
    payload = {**extra, "variants": list(variants.values()),
               "source_dependencies": [original_key, extra_key]}
    return store.obtain("primitive_index", key, lambda: payload)[0], key


def _original_lookup(store, cell, pid, rid, weld, planner, implementation):
    key = reuse_key(cell, pid, rid, weld, planner, implementation)
    geometric_key = {k: v for k, v in key.items() if k not in {"limits", "phase_speeds", "schema"}}
    geometric_key["schema"] = "primitive-geometric-index-v1"
    value = store.get("primitive_index", key)
    if value is not None and value.get("status") == "certified":
        if store.get("geometric_index", geometric_key) is None:
            store.obtain("geometric_index", geometric_key, lambda: value)
        return value, key
    geometric = store.get("geometric_index", geometric_key)
    if geometric is not None and geometric.get("status") == "certified":
        # Joint position limits are part of the geometry key. Changed dynamic
        # limits are independently retimed and checked by phase_pack before use.
        geometric = {**geometric, "geometry_retiming_reuse": True}
        return store.obtain("primitive_index", key, lambda: geometric)[0], key
    original_key = dependencies(cell, pid, rid, weld, planner, implementation)
    original = store.get("primitive", original_key)
    if original is None or original.get("status") != "certified":
        refined_key = {"base": original_key, "electrode_refinement": key["electrode_checker"]}
        refined = store.get("refined_primitive", refined_key)
        if refined and refined.get("status") == "certified":
            original, original_key = refined, refined_key
    if original is not None and original.get("status") == "certified":
        # This publication is possible only while the complete original program
        # key matches. Later an edit to another weld can reuse this unaffected
        # component without pretending the old program as a whole is unchanged.
        original = {**original, "source_dependencies": original_key}
        store.obtain("geometric_index", geometric_key, lambda: original)
        return store.obtain("primitive_index", key, lambda: original)[0], key
    return original, original_key
