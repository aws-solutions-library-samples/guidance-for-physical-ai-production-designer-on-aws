"""Shared helpers of the preparation steps: cell iteration, recycled spawn pools, JSON-lines logs."""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import multiprocessing as mp
from pathlib import Path


def cells(layout, selected=None):
    """``Cell`` objects of ``selected`` (default: every cell directory of the workspace)."""
    from ..cell import Cell
    return [Cell(layout.workspace, cid) for cid in layout.cells(selected)]


def revisions(layout):
    from ..native import code_revision
    from ..preparation import implementation_revision
    return code_revision(layout.umarell), implementation_revision()


def log_path(layout, name: str) -> Path:
    path = layout.prepare_runs / name
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def run_batches(fn, jobs, workers, batch, log_file: Path, on_error, *, star=True, hide=()):
    """Run ``fn(*job)`` (``fn(job)`` when ``star`` is False) in spawn pools of ``workers`` processes, a fresh pool
    every ``batch`` jobs (bounds native model memory without replacement-worker deadlocks); one JSON line per
    result in ``log_file`` and on stdout. ``on_error(job, exc)`` makes the row of a failed job."""
    rows = []
    with Path(log_file).open("a") as log:
        for start in range(0, len(jobs), batch):
            with ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn")) as pool:
                futures = {(pool.submit(fn, *job) if star else pool.submit(fn, job)): job
                           for job in jobs[start:start + batch]}
                for future in as_completed(futures):
                    try:
                        row = future.result()
                    except Exception as exc:  # noqa: BLE001 - recorded as an unknown, never infeasible
                        row = on_error(futures[future], exc)
                    log.write(json.dumps(row) + "\n")
                    log.flush()
                    print(json.dumps({k: v for k, v in row.items() if k not in hide}), flush=True)
                    rows.append(row)
    return rows
