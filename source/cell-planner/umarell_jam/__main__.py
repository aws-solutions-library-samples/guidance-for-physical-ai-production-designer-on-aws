"""CLI: setup, staging, preparation, requests, FactoryOS jobs, cache bundles, inspection and execution checks.

    python -m umarell_jam [--root R] [--workspace W] [--umarell U] [--cache C] [--output O] <command> ...

``--root`` (default data/umarell_jam) holds the components, the patched Umarell copy, plans, runs and logs
(umarell_jam/layout.py). ``--workspace`` is a FactoryOS data root (cells/, catalogue/, products/; default R/workspace
when staged, else /var/lib/factoryos). ``--umarell``: a plain Umarell 76ea13a checkout that R/umarell is built from
(default the repository's third_party/sample-umarell-76ea13a), or an already patched copy used as it is.
``--cache`` / ``--output`` override R/components and R/plans/bundles.

Commands: setup, install, stage, inspect, inventory, build, prepare, manifest (alias export), plan, plan-cell, serve,
simulate, evaluate, bundle, import, push. ``import`` / ``push`` / ``inspect`` need only the standard
library (``bundle`` also PyYAML).

Provenance: umarell_jam is ``umarell_fast.weld_cache``, the package of the preparation session that built the first
component caches (an earlier line of development of this repository's umarell_fast), moved to a top-level
package. Its numerical modules are byte-identical; ``umarell_jam/__init__.py`` keeps the original bytes (it is one
of the modules ``preparation.implementation_revision`` hashes), so the provenance is recorded here and in
docs/umarell_jam.md.
"""
from __future__ import annotations

import os

# The preparation session ran every command with single-threaded BLAS (numerical reproducibility, worker memory).
for _name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS"):
    os.environ.setdefault(_name, "1")

import argparse  # noqa: E402
import itertools  # noqa: E402
import json  # noqa: E402
from pathlib import Path  # noqa: E402
import sys  # noqa: E402

from .layout import Layout  # noqa: E402


def parser():
    ap = argparse.ArgumentParser(prog="python -m umarell_jam", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=None, help="writable root (default data/umarell_jam)")
    ap.add_argument("--workspace", type=Path, default=None, help="FactoryOS data root (cells/, catalogue/, products/)")
    ap.add_argument("--umarell", type=Path, default=None,
                    help="plain Umarell 76ea13a checkout R/umarell is built from, or an already patched copy")
    ap.add_argument("--cache", "--components", dest="cache", type=Path, default=None,
                    help="component store (default R/components)")
    ap.add_argument("--output", type=Path, default=None, help="published plan bundles (default R/plans/bundles)")
    commands = ap.add_subparsers(dest="command", required=True)
    s = commands.add_parser("setup", help="create / refresh R/umarell (apply.py U3 + fixes v8 + integration hooks)")
    s.add_argument("--force", action="store_true")
    install = commands.add_parser("install", help="install the integration hooks into the patched copy")
    install.add_argument("--patch", type=Path, default=None, help="also write the applied diff here")
    stage = commands.add_parser("stage", help="optional private copy of a FactoryOS data root")
    stage.add_argument("--source", type=Path, default=Path("/var/lib/factoryos"))
    stage.add_argument("--dest", type=Path, default=None, help="default R/workspace")
    stage.add_argument("--cells", nargs="+")
    commands.add_parser("inspect")
    commands.add_parser("inventory")
    build = commands.add_parser("build", help="base primitives (the first preparation step)")
    build.add_argument("--cells", nargs="+")
    build.add_argument("--robots", nargs="+")
    build.add_argument("--welds", nargs="+")
    build.add_argument("--workers", type=int, default=2)
    build.add_argument("--max-poses", type=int, default=4)
    build.add_argument("--validation-budget", type=float, default=120.)
    build.add_argument("--manifest", type=Path, default=None, help="default R/logs/cache-manifest.json")
    build.add_argument("--run-log", type=Path, default=None, help="default R/runs/prepare/cache-build.jsonl")
    build.add_argument("--allow-stack-mismatch", action="store_true",
                       help="write components even without toppra (Hermite-retimed certificates, same keys)")
    prepare = commands.add_parser("prepare", help="the whole preparation sequence, resumable")
    prepare.add_argument("--cells", nargs="+")
    prepare.add_argument("--workers", type=int, default=None,
                         help="worker processes of every step (default: each step's session count)")
    prepare.add_argument("--budget-scale", type=float, default=1.,
                         help="multiplies every step's time budget (default 1: the session's budgets)")
    prepare.add_argument("--steps", nargs="+", help="run only these steps (in sequence order)")
    prepare.add_argument("--from-step", help="start at this step")
    prepare.add_argument("--rerun", action="store_true", help="run steps already completed with the same inputs")
    prepare.add_argument("--dry-run", action="store_true", help="print the step plan only")
    prepare.add_argument("--allow-stack-mismatch", action="store_true",
                         help="write components even without toppra (Hermite-retimed certificates, same keys)")
    for name in ("manifest", "export"):
        m = commands.add_parser(name, help="write the cache manifest (coverage and provenance)")
        m.add_argument("--manifest", type=Path, default=None, help="default R/logs/cache-manifest.json")
    plan = commands.add_parser("plan", help="plan one weld-request-v1 JSON")
    plan.add_argument("request", type=Path)
    plan.add_argument("--result", type=Path)
    plan.add_argument("--timeout", type=float, default=30.)
    plan.add_argument("--repair-budget", type=float, default=0.)
    plan.add_argument("--serial", action="store_true")
    plan.add_argument("--allow-stack-mismatch", action="store_true",
                      help="repair even without toppra (default: the repair budget is clamped to 0, cache-only)")
    pc = commands.add_parser("plan-cell", help="FactoryOS job: plan a cell's task program, write PLAN.json")
    pc.add_argument("--cell-id", required=True)
    pc.add_argument("--task-file", type=Path)
    pc.add_argument("--welds", nargs="+")
    pc.add_argument("--robots", nargs="+")
    pc.add_argument("--speed", default="max", help="max | normal (both plan at the cell limits; recorded)")
    pc.add_argument("--timeout", type=float, default=300.)
    pc.add_argument("--repair-budget", type=float, default=0.)
    pc.add_argument("--prepare-budget", type=float, default=0.)
    pc.add_argument("--job", default=None, help="job id (default <cell>-<UTC time>-<pid>): R/runs/<job>/")
    pc.add_argument("--out", required=True, type=Path)
    pc.add_argument("--allow-stack-mismatch", action="store_true",
                    help="repair / prepare even without toppra (default: both budgets clamped to 0, cache-only)")
    commands.add_parser("serve", help="one JSON request per input line")
    run = commands.add_parser("simulate")
    run.add_argument("tree", type=Path)
    run.add_argument("--jitter", type=float, default=0.)
    run.add_argument("--slowdown", type=float, default=1.)
    run.add_argument("--pause", type=float, default=0.)
    evaluate = commands.add_parser("evaluate")
    evaluate.add_argument("--results", type=Path, default=None, help="default R/runs/subsets.jsonl")
    evaluate.add_argument("--cells", nargs="+")
    evaluate.add_argument("--limit", type=int)
    b = commands.add_parser("bundle", help="portable checksummed component bundle of cells (catalogue upload)")
    b.add_argument("--out", type=Path, default=None, help="bundle file or directory (default R/bundles/)")
    b.add_argument("--cells", nargs="+")
    imp = commands.add_parser("import", help="verify and import component bundles")
    imp.add_argument("bundles", nargs="+", type=Path)
    imp.add_argument("--strict-stack", action=argparse.BooleanOptionalAction, default=None,
                     help="reject a bundle whose numerical stack differs from this machine's (default: warn; "
                          "$UMARELL_JAM_STRICT_STACK=1 makes strict the default)")
    push = commands.add_parser("push", help="verify a bundle (test import) and upload it with aws s3 cp")
    push.add_argument("bundle", type=Path)
    push.add_argument("--dest", required=True, help="s3://<bucket>/catalogue/umarell-jam-cache/")
    push.add_argument("--region", default=None)
    return ap


def main(argv=None):
    args = parser().parse_args(argv)
    layout = Layout.make(args.root, args.workspace, args.cache, args.umarell, args.output)
    source = None if layout.umarell_override is not None else args.umarell
    if args.command == "plan-cell":
        from .plan_cell import run as plan_cell
        return plan_cell(layout, args, source)
    if args.command == "inspect":
        from .store import Store
        print(json.dumps(Store(layout.components).inspect(), indent=2))
        return 0
    if args.command == "bundle":
        from .bundle import BundleError, make_bundle
        try:
            print(json.dumps(make_bundle(layout, args.out, args.cells), indent=1))
        except (BundleError, OSError) as exc:
            print(json.dumps({"error": f"{type(exc).__name__}: {exc}"}))
            return 1
        return 0
    if args.command == "import":
        from .bundle import BundleError, import_bundle
        from .stack import strict_default
        strict = strict_default() if args.strict_stack is None else args.strict_stack
        failed = 0
        for bundle in args.bundles:
            try:
                print(json.dumps(import_bundle(layout, bundle, strict_stack=strict)), flush=True)
            except (BundleError, OSError) as exc:
                failed += 1
                print(json.dumps({"bundle": str(bundle), "rejected": f"{type(exc).__name__}: {exc}"}), flush=True)
        return 1 if failed else 0
    if args.command == "push":
        from .bundle import BundleError, push as push_bundle
        try:
            print(json.dumps(push_bundle(args.bundle, args.dest, region=args.region)))
        except (BundleError, OSError, RuntimeError) as exc:
            print(json.dumps({"bundle": str(args.bundle), "error": f"{type(exc).__name__}: {exc}"}))
            return 1
        return 0
    if args.command == "stage":
        from .stage import stage
        print(json.dumps(stage(args.source, args.dest or layout.root / "workspace", args.cells), indent=1))
        return 0
    if args.command == "inventory":
        from .cell import Cell
        print(json.dumps([Cell(layout.workspace, cid).manifest_export() for cid in layout.cells()], indent=2))
        return 0
    from . import native_copy
    if args.command == "setup":
        if layout.umarell_override is not None:
            with native_copy.ready(layout) as stamp:
                print(json.dumps(stamp, indent=1))
            return 0
        print(json.dumps(native_copy.ensure(layout, source, force=args.force), indent=1))
        return 0
    if args.command == "install":
        from .install import install
        layout.activate()
        print(install(layout.umarell, args.patch))
        return 0
    with native_copy.ready(layout, source, log=lambda m: print(m, flush=True)):
        layout.activate()
        return command(layout, args)


def command(layout, args):
    from .cell import Cell
    from .store import atomic_json
    if args.command in {"build", "manifest", "export"}:
        from .manifest import export_manifest
        target = args.manifest or layout.manifest
        if args.command == "build":
            from .stack import StackMismatch, require_toppra
            try:
                require_toppra(args.allow_stack_mismatch)
            except StackMismatch as exc:
                print(json.dumps({"refused": str(exc)}), flush=True)
                return 1
            from .prepare.build import run as build
            build(layout, args.cells, args.robots, args.welds, args.workers, args.max_poses,
                  args.validation_budget, args.run_log)
        manifest = export_manifest(layout.workspace, layout.components, layout.umarell, target)
        print(json.dumps({"manifest": str(target), "bytes": manifest["storage"]["bytes"]}))
        return 0
    if args.command == "prepare":
        from .prepare.pipeline import run as prepare
        return prepare(layout, cells=args.cells, workers=args.workers, budget_scale=args.budget_scale,
                       steps=args.steps, from_step=args.from_step, rerun=args.rerun, dry_run=args.dry_run,
                       allow_stack_mismatch=args.allow_stack_mismatch)
    if args.command == "simulate":
        from .execution import simulate
        tree = json.loads(args.tree.read_text())
        print(json.dumps(simulate(Cell(layout.workspace, tree["cell_id"]), tree,
                                  jitter_s=args.jitter, slowdown=args.slowdown, pause_s=args.pause), indent=2))
        return 0
    from .service import PlannerService
    service = PlannerService(layout.workspace, layout.components, layout.umarell, layout.plans)
    if args.command == "plan":
        from .stack import repair_guard
        guard = repair_guard({"--repair-budget": args.repair_budget}, args.allow_stack_mismatch)
        result = service.plan(json.loads(args.request.read_text()), timeout_s=args.timeout,
                              repair_budget_s=0. if guard else args.repair_budget, serial=args.serial)
        if guard:
            result["stack_guard"] = guard
        if args.result:
            atomic_json(args.result, result)
        print(json.dumps({k: v for k, v in result.items() if k != "behaviour_tree"}, indent=2))
        return 0
    if args.command == "serve":
        for line in sys.stdin:
            try:
                print(json.dumps(service.plan(json.loads(line))), flush=True)
            except ValueError as exc:
                print(json.dumps({"status": "invalid_request", "reason": str(exc)}), flush=True)
        return 0
    if args.command == "evaluate":
        count = 0
        results = args.results or layout.runs / "subsets.jsonl"
        results.parent.mkdir(parents=True, exist_ok=True)
        with results.open("w") as output:
            for cid in layout.cells(args.cells):
                cell = service.load(cid)
                for pid, program in cell.programs.items():
                    welds = [o["id"] for o in program["operations"]]
                    for size in range(1, len(welds)+1):
                        for subset in itertools.combinations(welds, size):
                            request = {"schema_version": "weld-request-v1", "request_id": f"heldout-{count}",
                                       "cell_id": cell.cell_id, "program_id": pid, "weld_ids": list(subset),
                                       "enabled_robot_ids": list(cell.robots), "completed_weld_ids": [],
                                       "initial_state": {"mode": "configured_home"},
                                       "terminal_state": "home", "speed_profile": "cell_limits"}
                            result = service.plan(request)
                            compact = {k: v for k, v in result.items()
                                       if k not in {"behaviour_tree", "effective_limits", "process_settings", "interlocks"}}
                            compact.update(cell_id=cell.cell_id, program_id=pid, weld_ids=list(subset))
                            output.write(json.dumps(compact) + "\n")
                            count += 1
                            if args.limit and count >= args.limit:
                                return 0
                        output.flush()
                    print(json.dumps({"cell": cell.cell_id, "program": pid, "requests_total": count}), flush=True)
        return 0
    raise SystemExit(f"unknown command {args.command}")


if __name__ == "__main__":
    sys.exit(main())
