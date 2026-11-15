"""Combined provenance for strict and refined immutable component layers."""
import hashlib
from pathlib import Path

from .cell import Cell, CellLoadError
from .layout import cell_ids
from .native import code_revision
from .preparation import implementation_revision
from .primitive_index import lookup
from .store import Store, atomic_json, digest


def export_manifest(workspace, store_root, umarell, output):
    store = Store(store_root)
    planner, implementation = code_revision(Path(umarell)), implementation_revision()
    cells = []
    for cell_id in cell_ids(workspace):  # not every directory of cells/ is a cell (.backup, unprepared cells)
        try:
            cell = Cell(workspace, cell_id)
        except CellLoadError as exc:
            # A cell of the workspace that does not load (missing URDF, scene/config mismatch, ...) is recorded, not
            # raised: prepare exports this manifest after its steps, and another cell must not stop that.
            cells.append({"cell_id": cell_id, "load_error": f"{type(exc).__name__}: {exc}", "coverage": []})
            continue
        row = cell.manifest_export()
        row["coverage"] = []
        for pid, program in cell.programs.items():
            for rid in cell.robots:
                for op in program["operations"]:
                    value, dep = lookup(store, cell, pid, rid, op["id"], planner, implementation)
                    row["coverage"].append({"program": pid, "robot": rid, "weld": op["id"],
                        "key": digest(dep), "dependencies": dep,
                        "status": ("program_invalid" if cell.program_errors[pid] else
                                   value.get("status", "missing") if value else "missing"),
                        "variants": len(value.get("variants", [])) if value else 0,
                        "geometry_retiming_reuse": bool(value and value.get("geometry_retiming_reuse")),
                        "certificates": [v["certificate"]["trajectory_hash"] for v in value.get("variants", [])]
                                         if value else []})
        cells.append(row)
    manifest = {"schema_version": "umarell-cache-export-v1", "planner": planner,
                "implementation": implementation, "cells": cells, "storage": store.inspect(),
                "integration_files": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                      for p in sorted(Path(__file__).parent.glob("*.py"))},
                "whole_request_cache_enabled": False,
                "release_contract": "Component certificates require matching geometry and constraints; phase retiming, exact process/joins, pair proofs, loader/XML/executor and publication checks remain required.",
                "capacity": "Tested 2 and 4 arms; six bounded revolute joints per arm. Arbitrary installed-arm IDs; no two-arm assumption."}
    atomic_json(Path(output), manifest)
    return manifest
