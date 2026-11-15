"""``prepare``: the session's preparation sequence, in order, resumable, with a run log under ``R/logs``.

    python -m umarell_jam --root R --workspace W prepare [--cells …] [--workers N] [--budget-scale X]
        [--steps …] [--from-step S] [--rerun] [--dry-run]

The steps and their session arguments (docs/umarell_jam.md, "Prepare"); ``--workers N`` replaces every step's
worker count, ``--budget-scale X`` multiplies every time budget:

==========================  =======================================================================
build                       base HOME-weld-return primitives (workers 2)
augment                     staged wrist-branch search (workers 8, budget 600, poses 24, variants 2, resume)
build_electrode             refined-electrode candidates for welds without any (workers 4, --missing)
prepare_release             phase packs and stroke proofs (workers 5)
build_transfers             directed transfers, first branch (workers 8, budget 120)
build_transfers_fastest     directed transfers, fastest branch (workers 8, budget 120)
build_coordination          pair compatibility in sequence groups (workers 6, budget 15)
coverage_prepare            whole-arm IK branch discovery + certification (workers 16, budget 120, variants 1)
refine_contacts             smoother contact timing (workers 12, budget 180)
coverage_transfers          alternative transfers (workers 12, budget 60, branches 2)
coverage_coordination       continuous pair proofs for primitives and transfers (workers 12, budget 2)
coverage_phase_prepare      phase/rest pair certificates on full programs (workers 4, timeout 1200)
manifest                    R/logs/cache-manifest.json (coverage and provenance)
==========================  =======================================================================

Resume: every step reuses the checksummed components it finds (``Store.obtain``: certified entries are reused,
inconclusive ones are searched again). In addition a step that completed with the same inputs -- its arguments,
the cells' geometry / limits / program keys, the planner and checker revisions, and the completion of every step
before it -- is skipped (``--rerun`` runs it anyway). ``R/logs/prepare-state.json`` records the completions;
``R/logs/prepare-<UTC time>.jsonl`` logs every step of a run (start, end, summary or error).

Numerical stack: the run log's ``start`` row and every completion in the state record the stack
(:func:`umarell_jam.stack.fingerprint`). Without toppra a run that would execute a component-writing step is
refused (exit 1, nothing written): umarell's Hermite fallback would write differently timed certificates under the
same keys. ``--allow-stack-mismatch`` overrides; ``--dry-run`` and ``--steps manifest`` are never refused.
"""
from __future__ import annotations

import json
import time
import traceback

from ..store import atomic_json, digest

# (name, module, session keyword arguments, names of the time-budget arguments)
STEPS = [
    ("build", "build", {"workers": 2, "max_poses": 4, "validation_budget": 120.}, ("validation_budget",)),
    ("augment", "augment", {"workers": 8, "budget": 600., "poses": 24, "variants": 2, "resume": True}, ("budget",)),
    ("build_electrode", "build_electrode", {"workers": 4, "budget": 600., "poses": 16, "missing": True}, ("budget",)),
    ("prepare_release", "prepare_release", {"workers": 5}, ()),
    ("build_transfers", "build_transfers", {"workers": 8, "budget": 120.}, ("budget",)),
    ("build_transfers_fastest", "build_transfers", {"workers": 8, "budget": 120., "branch_policy": "fastest"},
     ("budget",)),
    ("build_coordination", "build_coordination", {"workers": 6, "budget": 15.}, ("budget",)),
    ("coverage_prepare", "coverage_prepare", {"workers": 16, "budget": 120., "variants": 1}, ("budget",)),
    ("refine_contacts", "refine_contacts", {"workers": 12, "budget": 180.}, ("budget",)),
    ("coverage_transfers", "coverage_transfers", {"workers": 12, "budget": 60., "branches": 2}, ("budget",)),
    ("coverage_coordination", "coverage_coordination", {"workers": 12, "budget": 2.}, ("budget",)),
    ("coverage_phase_prepare", "coverage_phase_prepare", {"workers": 4, "timeout": 1200.}, ("timeout",)),
    ("manifest", None, {}, ()),
]
NAMES = [s[0] for s in STEPS]


def step_kwargs(name: str, cells, workers, budget_scale) -> dict:
    _, _, kwargs, budgets = next(s for s in STEPS if s[0] == name)
    kwargs = dict(kwargs)
    if workers and "workers" in kwargs:
        kwargs["workers"] = int(workers)
    for b in budgets:
        kwargs[b] = float(kwargs[b]) * float(budget_scale)
    kwargs["cells"] = list(cells)
    return kwargs


def _inputs(layout, cells) -> dict:
    from ..cell import Cell
    from ..native import code_revision
    from ..preparation import implementation_revision
    out = {"planner": code_revision(layout.umarell), "implementation": implementation_revision(), "cells": {}}
    for cid in cells:
        c = Cell(layout.workspace, cid)
        out["cells"][cid] = {"geometry": c.geometry_hash, "limits": c.limits_hash, "programs": c.program_keys}
    return out


def run(layout, cells=None, workers=None, budget_scale=1., steps=None, from_step=None, rerun=False,
        dry_run=False, allow_stack_mismatch=False) -> int:
    import importlib
    from ..stack import StackMismatch, fingerprint, require_toppra
    if budget_scale <= 0:
        raise SystemExit("--budget-scale must be > 0")
    unknown = [s for s in (steps or []) + ([from_step] if from_step else []) if s not in NAMES]
    if unknown:
        raise SystemExit(f"unknown step(s) {unknown}; steps: {NAMES}")
    selected = [n for n in NAMES if (not steps or n in steps)]
    if from_step:
        selected = [n for n in selected if NAMES.index(n) >= NAMES.index(from_step)]
    stack = fingerprint()
    if not dry_run and any(n != "manifest" for n in selected):
        try:
            require_toppra(allow_stack_mismatch, stack)
        except StackMismatch as exc:
            print(json.dumps({"event": "refused", "reason": str(exc), "stack": stack}), flush=True)
            return 1
    cells = layout.cells(cells)
    layout.activate()
    inputs = _inputs(layout, cells)
    layout.logs.mkdir(parents=True, exist_ok=True)
    state_file = layout.logs / "prepare-state.json"
    try:
        state = json.loads(state_file.read_text())
    except (OSError, ValueError):
        state = {}
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    log_file = layout.logs / f"prepare-{stamp}.jsonl"
    previous = digest(inputs)

    def log(row):
        row = {"t": time.time(), **row}
        with log_file.open("a") as f:
            f.write(json.dumps(row, default=str) + "\n")
        print(json.dumps(row, default=str), flush=True)

    log({"event": "start", "root": str(layout.root), "workspace": str(layout.workspace),
         "components": str(layout.components), "cells": cells, "workers": workers, "budget_scale": budget_scale,
         "steps": selected, "inputs": inputs, "stack": stack, "allow_stack_mismatch": allow_stack_mismatch})
    failed = False
    for name in NAMES:
        kwargs = step_kwargs(name, cells, workers, budget_scale)
        fingerprint = digest({"step": name, "kwargs": kwargs, "before": previous})
        done = state.get(name, {})
        if name not in selected:
            # an unselected step keeps its last completion as the input of the next one
            previous = done.get("completion", fingerprint)
            continue
        if not rerun and done.get("fingerprint") == fingerprint:
            log({"event": "skip", "step": name, "reason": "completed with the same inputs",
                 "completed": done.get("finished")})
            previous = done["completion"]
            continue
        if dry_run:
            log({"event": "would_run", "step": name, "kwargs": kwargs})
            previous = fingerprint
            continue
        log({"event": "step_start", "step": name, "kwargs": kwargs})
        started = time.monotonic()
        try:
            if name == "manifest":
                from ..manifest import export_manifest
                m = export_manifest(layout.workspace, layout.components, layout.umarell, layout.manifest)
                summary = {"manifest": str(layout.manifest), "bytes": m["storage"]["bytes"]}
            else:
                module = importlib.import_module(f"{__package__}.{next(s[1] for s in STEPS if s[0] == name)}")
                summary = module.run(layout, **kwargs)
        except Exception as exc:  # noqa: BLE001 - recorded; later steps would build on a partial store
            log({"event": "step_failed", "step": name, "wall_s": round(time.monotonic() - started, 3),
                 "error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc()[-4000:]})
            failed = True
            break
        completion = digest({"fingerprint": fingerprint, "finished": time.time()})
        state[name] = {"fingerprint": fingerprint, "completion": completion,
                       "finished": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                       "wall_s": round(time.monotonic() - started, 3), "summary": summary, "stack": stack}
        atomic_json(state_file, state)
        log({"event": "step_done", "step": name, "wall_s": state[name]["wall_s"], "summary": summary})
        previous = completion
    log({"event": "end", "failed": failed, "log": str(log_file)})
    return 1 if failed else 0
