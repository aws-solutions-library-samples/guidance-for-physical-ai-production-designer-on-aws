"""Base HOME-weld-return primitives (the session's ``weld_cache build``): one job per program, robot and weld."""
from __future__ import annotations

import json

from .common import log_path, run_batches


def jobs_for(layout, cells=None, robots=None, welds=None, max_poses=4, validation_budget=120.):
    from ..capacity import validate_capacity
    from ..cell import Cell
    jobs = []
    for cid in layout.cells(cells):
        c = Cell(layout.workspace, cid)
        validate_capacity(c)
        for pid, program in c.programs.items():
            if c.program_errors[pid]:
                print(json.dumps({"cell": c.cell_id, "program": pid,
                                  "status": "invalid_request", "errors": c.program_errors[pid]}), flush=True)
                continue
            for rid in c.robots:
                if robots and rid not in robots:
                    continue
                for op in program["operations"]:
                    if not welds or op["id"] in welds:
                        jobs.append((str(layout.workspace), str(layout.components), str(layout.umarell),
                                     str(layout.prepare_runs / "cache-workers"), c.cell_id, pid, rid,
                                     op["id"], max_poses, validation_budget))
    return jobs


def run(layout, cells=None, robots=None, welds=None, workers=2, max_poses=4, validation_budget=120.,
        run_log=None):
    from ..preparation import build_job
    layout.activate()
    jobs = jobs_for(layout, cells, robots, welds, max_poses, validation_budget)
    run_log = run_log or log_path(layout, "cache-build.jsonl")
    rows = run_batches(build_job, jobs, workers, max(1, len(jobs)), run_log,
                       lambda job, exc: {"job": job[4:8], "status": "timeout_or_unknown", "exception": repr(exc)},
                       hide=("dependencies", "failures"))
    return {"jobs": len(jobs), "certified": sum(r.get("status") == "certified" for r in rows)}
