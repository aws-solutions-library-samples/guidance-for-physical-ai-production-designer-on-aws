"""Process-parallel evaluation for trajgen (round p3, track TGPERF): a fork-based worker pool that runs independent
pieces of the pipeline (IK reach per weld and robot, static eligibility, coordinator runs of (assignment, window,
priority order), the order search, the balance step, lateness-repair candidates, the step searches) in parallel.

Design rules:
  * results are those of the serial code: every task is a pure function of its inputs (the planners' caches only
    memoise deterministic results), the tasks of one stage are evaluated speculatively in parallel and the serial
    selection logic is then applied to them in the serial order (so a parallel run gives the same plan as a serial
    one, bit for bit);
  * workers are forked (Linux) after the parent has built the scene model, so they inherit it and every cache
    filled so far; legs computed in a worker are sent back and merged into the parent's caches;
  * ``workers``: an int, or None = the TRAJGEN_WORKERS environment variable, else os.cpu_count(); 1 = serial (no
    pool). A stage with one task runs in the parent.
"""
from __future__ import annotations

import multiprocessing as mp
import os
import time

_CTX: dict = {}


def n_workers(workers=None) -> int:
    if workers is None:
        env = os.environ.get("TRAJGEN_WORKERS", "").strip()
        workers = int(env) if env else (os.cpu_count() or 1)
    return max(1, int(workers))


def _call(i):
    fn, ctx, items = _CTX["fn"], _CTX["ctx"], _CTX["items"]
    t0 = time.time()
    out = fn(ctx, items[i])
    return out, time.time() - t0


def pmap(fn, ctx, items, workers: int = 1, log=None, label: str = ""):
    """[fn(ctx, item) for item in items], evaluated in forked worker processes when workers > 1 (ctx and items
    are inherited by fork, only the results are pickled). Returns the results in item order."""
    items = list(items)
    w = min(int(workers), len(items))
    t0 = time.time()
    if w <= 1:
        out = [fn(ctx, x) for x in items]
        if log and label and len(items) > 1:
            log(f"[parallel] {label}: {len(items)} tasks serial, {time.time() - t0:.1f} s")
        return out
    prev = dict(_CTX)
    _CTX.update(fn=fn, ctx=ctx, items=items)
    try:
        with mp.get_context("fork").Pool(w) as pool:
            res = pool.map(_call, range(len(items)), chunksize=1)
    finally:
        _CTX.clear()
        _CTX.update(prev)
    if log and label:
        busy = sum(t for _, t in res)
        log(f"[parallel] {label}: {len(items)} tasks on {w} workers, wall {time.time() - t0:.1f} s, "
            f"task sum {busy:.1f} s")
    return [r for r, _ in res]
