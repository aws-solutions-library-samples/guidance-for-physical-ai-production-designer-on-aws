"""One isolated, cancellable repair worker for one program and scene revision."""
import argparse
import json
from pathlib import Path
import time

from .cell import Cell
from .native import setup, code_revision
from .preparation import build_job, implementation_revision
from .primitive_index import lookup
from .seed_repair import adapt, candidates
from .store import Store, atomic_json


def run(spec):
    start = time.monotonic()
    deadline = start+spec["budget_s"]
    workspace, native = Path(spec["workspace"]), Path(spec["umarell"])
    cell = Cell(workspace, spec["cell"])
    if cell.geometry_hash != spec["geometry"]:
        return {"status": "timeout_or_unknown", "reason": "scene_changed_before_repair"}
    store = Store(spec["cache"])
    planner, implementation = code_revision(native), implementation_revision()
    pid = spec["program"]
    geometry = None
    rows = []

    def covered(weld):
        return any((lookup(store, cell, pid, r, weld, planner, implementation)[0] or {}).get("status") == "certified"
                   for r in spec["robots"])

    def checkpoint():
        atomic_json(Path(spec["receipt"]), {"status": "running", "components": rows,
                                           "wall_s": time.monotonic()-start})

    for index, weld in enumerate(spec["welds"]):
        if covered(weld):
            continue
        remaining_welds = len(spec["welds"])-index
        seed_deadline = min(deadline, time.monotonic()+min(100., .6*max(0., deadline-time.monotonic())/remaining_welds))
        target = next(o["position"] for o in cell.programs[pid]["operations"] if o["id"] == weld)
        distance = {r: sum((a-b)**2 for a, b in zip(target, cell.robots[r]["base_position"]))**.5
                    for r in spec["robots"]}
        seeds = {r: candidates(store, cell.cell_id, r, weld, len(cell.robots[r]["home_pose_deg"]),
                               deadline=min(seed_deadline, time.monotonic()+3.)) for r in spec["robots"]}
        # Fixed robot-ID order can produce an unnecessarily serial program.
        # Prior check cost is only a search heuristic; all checks run again.
        seed_order = sorted(spec["robots"], key=lambda r: (
            not bool(seeds[r]), (seeds[r][0]["source_validation_wall_s"] if seeds[r] else 0.)+10*distance[r], r))
        for rid in seed_order:
            sources = seeds[rid]
            if not sources or time.monotonic() >= seed_deadline:
                continue
            if geometry is None:
                from .electrode_geometry import RefinedElectrodeGeometry
                setup(native, workspace, Path(spec["scratch"]), cell.cell_id)
                geometry = RefinedElectrodeGeometry(cell)
            result = adapt(cell, geometry, pid, rid, weld, sources, store, planner, implementation,
                           deadline=seed_deadline)
            rows.append({"robot": rid, "weld": weld, "mode": "cached_seed", "sources": len(sources), **result})
            checkpoint()
            if result["status"] == "certified":
                break
        if covered(weld):
            continue
        for rid in sorted(spec["robots"], key=lambda r: (distance[r], r)):
            if time.monotonic() >= deadline or not cell.unchanged():
                break
            result = build_job(workspace, store.root, native, spec["scratch"],
                               cell.cell_id, pid, rid, weld, max_poses=2,
                               validation_budget_s=max(.01, deadline-time.monotonic()))
            rows.append({"robot": rid, "weld": weld, "mode": "native_fallback", **result})
            checkpoint()
            if covered(weld):
                break
    return {"status": "completed" if all(covered(w) for w in spec["welds"]) else "timeout_or_unknown",
            "components": rows, "wall_s": time.monotonic()-start}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("spec", type=Path)
    spec = json.loads(parser.parse_args().spec.read_text())
    result = run(spec)
    atomic_json(Path(spec["receipt"]), result)
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
