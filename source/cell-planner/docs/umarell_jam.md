# umarell_jam: a reusable, certified Umarell weld planner

`umarell_jam` (top-level package, sibling of `umarell_fast/`) implements `weld-request-v1` on Umarell's numerical
planner, patched (Umarell 8a02db9, the public commit: umarell_jam's series `umarell_jam/patches_8a02db9`, the
upstream fix set v10 rebased; or Umarell 76ea13a: upstream fix set v8 + U3 hooks, what the preparation session ran), from a content-addressed cache of certified components
(weld primitives, phase packs, stroke proofs, transfers, pair proofs). It returns a complete, checked tree or an
explicit non-executable result. The routine path performs no LLM calls and does not cache whole requests.
Preparation is independent of policy training.

Provenance: `umarell_jam` is `umarell_fast.weld_cache`, the package of the preparation session that built the first
component caches (an earlier line of development of this repository's `umarell_fast`, "the session" below),
ported to a top-level package with a writable `--root` and a FactoryOS `--workspace`, and the session's
preparation tools (outside its repository) ported to `umarell_jam/prepare/`. The numerical code is byte-identical; see
"Cache compatibility" for what that guarantees. It reuses `umarell_fast` (apply.py, the u2 session, the workers
and the x76 kernels) and does not duplicate it.

## Layout: `--root` and `--workspace`

```
python -m umarell_jam [--root R] [--workspace W] [--umarell U] [--cache C] [--output O] <command> ...
```

* `--root R` (default `data/umarell_jam` in this repository, gitignored): everything written.
  * `R/components/`: the component Store (`--cache C` overrides it);
  * `R/umarell/`: the patched Umarell copy (`setup`);
  * `R/plans/bundles/<version>/`: published plan bundles (bt.json, blackboard.json, bt.xml, manifest.json),
    `R/plans/repair-logs/` and `R/plans/execution-failures/` beside them (`--output O` overrides the bundle dir);
  * `R/runs/<job>/`: one `plan-cell` job (progress.json, request.json, result.json); `R/runs/prepare/`: the
    preparation steps' JSON-lines logs, diagnostics (probes, checkpoints) and worker scratch;
  * `R/logs/`: `cache-manifest.json`, `prepare-<UTC>.jsonl`, `prepare-state.json`, `integration.diff`;
  * `R/bundles/`: default output of `bundle`; `R/workspace/`: only after `stage`;
  * `R/.setup.lock`: shared while a job plans on `R/umarell`, exclusive while it is rebuilt.
* `--workspace W`: a FactoryOS data root with `cells/`, `catalogue/` and `products/` (`/var/lib/factoryos` on a
  workspace). Default: `R/workspace` when it was staged, else `/var/lib/factoryos`. `Cell` maps every
  `/var/lib/factoryos/...` reference (and absolute symlink) onto W, so the cell keys of W do not depend on where W
  is. Umarell's own planner (the native candidate generation of `prepare` and of repairs) reads the cell's absolute
  references literally: plan natively either on `/var/lib/factoryos` itself or on a staged copy.
* `--umarell U`: the plain Umarell checkout `R/umarell` is built from: the public 8a02db9
  (https://github.com/aws-samples/sample-industrial-multi-robot-agents, `8a02db9ba7db260366451f8c38bafb378cd65c29`)
  or 76ea13a, recognised by its `.umarell.sha256` stamp (the pinned commit, as the orchestrator's fetch-umarell.sh
  writes it) or its layout (`native_copy.tree_base`; any other base is a setup failure). Default
  `third_party/sample-umarell-8a02db9` when present, else `third_party/sample-umarell-76ea13a`. A directory that already is a patched copy (UF_APPLIED.json +
  WELD_INTEGRATION.json, with hooks that import `umarell_jam`) is used as it is. A copy installed by the session's
  package (hooks importing `umarell_fast.weld_cache`) is refused: let umarell_jam build `R/umarell` (its planner
  revision is the same, see below).

Every command sets `OPENBLAS_NUM_THREADS=1` and `OMP_NUM_THREADS=1` unless they are set (the session's
environment). The numerical runtime needs Python 3.12, NumPy, SciPy, Pinocchio, hpp-fcl (imported as `hppfcl`),
OMPL, TOPP-RA, PyYAML and the bundled Umarell dependencies; LangGraph is not needed. `inspect`, `import` and `push`
need only the standard library (`bundle` also PyYAML).

## Setup

```
python -m umarell_jam --root R [--umarell <plain 8a02db9 or 76ea13a checkout>] setup [--force]
```

Into a temporary sibling:

* 8a02db9: the checkout copied (rsync, without .git / caches / venvs), then `umarell_jam/patches_8a02db9/series.txt`
  applied in order (`patch -p1 --forward --batch`; a rejected hunk fails setup): the upstream fix set v10
  (docs/upstream/patches_08d5a39) rebased onto 8a02db9 without 28 (8a02db9 contains it), then the U3 hook U3_02
  (OMPL seed). `UF_APPLIED.json` is written by native_copy (base 8a02db9). umarell_fast/apply.py is not used.
* 76ea13a: `umarell_fast/apply.py --variant U3 --base 76ea13a --with-upstream-fixes --src <checkout>` (fix set v8
  and the U3 hook patches).

Then `install` (umarell_jam/install.py: the integration hooks, recorded
in `umarell_jam/patches_8a02db9/integration.diff` on 8a02db9, `umarell_jam/patches/integration.diff` on
76ea13a: the same edits, other line numbers; the diff of the actual edits is written to `R/logs/integration.diff`
and compared), then renamed to `R/umarell`. The stamp `R/umarell/UMARELL_JAM_COPY.json` records the checkout
base, path and file fingerprint, every patch (76ea13a: also apply.py), the installer and the integration record (a
76ea13a stamp has exactly the keys it always had, so an existing 76ea13a copy stays current); any difference
rebuilds the copy. Every planning command (`plan`, `plan-cell`, `serve`, `build`, `prepare`, `simulate`,
`evaluate`, `manifest`) runs this check first. `umarell_fast/apply.py` gained `--src` for this; on 8a02db9 /
08d5a39 `umarell_fast.workers.install_kernels("x76")` (which `native.setup` asks for) installs nothing: umarell
vectorised and Numba-compiled the checks x76 speeds up itself (its source guards refuse there), so umarell's own
predicate runs (with numba when it imports and `NUMBA_GEOM` is not 0, else its NumPy path; the jam venv has no
numba).

The hooks touch agents/trajectory/agent.py (the `constraints["weld_request"]` entry), agents/trajectory/tools/
btcpp.py and behaviour_tree.py (the WeldPhase / WeldBoundary nodes and ports), agents/cell/bt_executor.py (their
execution, exact controller tolerance, path tracking) and agents/robot/robot_agent.py. None of them is in the
planner revision (`native.code_revision` hashes agents/trajectory/backends, agents/trajectory/tools without btcpp.py,
behaviour_tree.py, coordinated_tree.py, plan_store.py, and utils).

## Prepare: building a cache from scratch

```
python -m umarell_jam --root R --workspace W prepare [--cells …] [--workers N] [--budget-scale X] \
    [--steps …] [--from-step S] [--rerun] [--dry-run] [--allow-stack-mismatch]
```

The session's sequence as importable steps (`umarell_jam/prepare/`), in this order, with the session's arguments;
`--workers N` replaces every step's worker count, `--budget-scale X` multiplies every time budget:

| step | module (session tool) | session arguments |
|---|---|---|
| build | prepare/build.py (`weld_cache build`) | workers 2, max poses 4, validation budget 120 s |
| augment | prepare/augment.py (tools/augment.py) | workers 8, budget 600 s, poses 24, variants 2, resume |
| build_electrode | prepare/build_electrode.py | workers 4, budget 600 s, poses 16, `--missing` |
| prepare_release | prepare/prepare_release.py | workers 5 |
| build_transfers | prepare/build_transfers.py | workers 8, budget 120 s, first branch |
| build_transfers_fastest | prepare/build_transfers.py | workers 8, budget 120 s, `--branch-policy fastest` |
| build_coordination | prepare/build_coordination.py | workers 6, budget 15 s |
| coverage_prepare | prepare/coverage_prepare.py (+ coverage_probe.py, certify_branches.py) | workers 16, budget 120 s, variants 1 |
| refine_contacts | prepare/refine_contacts.py | workers 12, budget 180 s |
| coverage_transfers | prepare/coverage_transfers.py | workers 12, budget 60 s, branches 2 |
| coverage_coordination | prepare/coverage_coordination.py | workers 12, budget 2 s |
| coverage_phase_prepare | prepare/coverage_phase_prepare.py | workers 4, timeout 1200 s |
| manifest | manifest.py | `R/logs/cache-manifest.json` |

Without `--cells` every cell directory of W is prepared. Each step module also has `run(layout, ...)` with the
session tool's options (cells, welds, robots, budgets) for single steps. `build` alone is also a command
(`python -m umarell_jam build ...`, then the manifest).

Resume: every step reuses the checksummed components it finds (`Store.obtain`: certified entries are reused,
timeout entries are inconclusive and searched again; the augment step also skips pairs already attempted in
`R/runs/prepare/augmentation.jsonl`). On top of that a step that completed with the same inputs (its arguments,
the cells' geometry / limits / program keys, the planner and checker revisions and the completion of every step
before it) is skipped; `--rerun` runs it anyway, `--steps` / `--from-step` select steps. The run log is
`R/logs/prepare-<UTC>.jsonl` (start, every step's start / done / failure with its summary, end), completions are in
`R/logs/prepare-state.json`. A failed step stops the run (exit 1).

Numerical stack: the run log's `start` row and every completion in `prepare-state.json` record `stack`
(`umarell_jam/stack.py` `fingerprint()`, see "Cache compatibility"). Without toppra, `prepare` (and `build`)
refuses to write components (exit 1, a `refused` row, nothing written): umarell's time parameterisation would fall
back to Hermite retiming and write differently timed certificates under the same component keys, since the keys do
not contain the stack. `--allow-stack-mismatch` overrides this; `--dry-run` and `--steps manifest` are never
refused. Cache-only planning is still allowed without toppra: in the FactoryOS workspace umarell venv (pin 3.7.0,
coal 3.0.1 through the hppfcl shim, numpy 2.2.6, no toppra), cache-only planning gave identical trees. Repair is
not cache-only: without toppra, `plan --repair-budget` and `plan-cell --repair-budget` / `--prepare-budget` are
clamped to 0 (`stack.repair_guard`). The result then carries `stack_guard: "repair disabled: toppra missing"`
(plan-cell: also result.json), a note goes to stderr, and a cache miss is the ordinary
`missing_certified_primitives` (plan-cell exit 3, which the router's umarell_u3 fallback handles).
`--allow-stack-mismatch` on `plan` / `plan-cell` restores the repair. The stack is
not part of a step's completion fingerprint, so a step completed under another stack is not rerun automatically
(use `--rerun`).

Limit native threads and workers: collision models consume substantial RAM; the steps recycle worker pools as the
session did. The two transfer passes cover the first available branch and the fastest retimed branch. Endpoint
keys remain explicit; a transfer for one IK branch cannot connect a different branch.

Not ported: the session's cloud launchers, benchmarks, evaluations, audits, reports and packaging
(tools/cloud*.py, benchmark*, evaluate.py, state_cases.py, invalidation.py, audit_*, report*, package_*, freeze_export
and the shell pipelines).

Differences from the session tools, none numerical: paths come from the layout; every step takes `cells` (the
session ran most steps on every staged cell); `coverage_phase_prepare` plans every valid program of a cell
(the session named one program per cell by its id); the provenance fields `search_code` / `builder_code` /
`certification_builder` of newly built payloads hash the ported modules (they are payload provenance, never keys).

## FactoryOS jobs: `plan-cell`

```
python -m umarell_jam --root R --workspace W [--umarell <plain umarell 8a02db9 / 76ea13a checkout>] plan-cell \
    --cell-id ID [--task-file F.json] [--welds W01 …] [--robots robot1 …] [--speed max|normal] \
    [--timeout S] [--repair-budget S] [--prepare-budget S] [--job JOB] [--allow-stack-mismatch] --out PLAN.json
```

1. The weld-request-v1 is built from the cell's task program: `--task-file` (it must be one of the cell's own
   programs in `W/cells/<ID>/agentic_planner/tasks/`, equal as JSON, since the component keys follow the cell's
   programs), else the cell's single program, else invalid_request. All welds / all installed robots unless
   `--welds` / `--robots` (space separated), `initial_state` configured_home, `terminal_state` home,
   `speed_profile` cell_limits. `--speed` is recorded; both values plan at the cell limits (the certificates are
   tied to them; there is no other profile).
2. `R/umarell` is set up if missing or stale (above), under `R/.setup.lock`.
3. `PlannerService.plan(request, timeout_s=--timeout, repair_budget_s=--repair-budget)` and `commit`. When the
   result is `missing_certified_primitives` and `--prepare-budget` > 0, the bounded, isolated native
   repair / preparation of exactly the missing welds for the enabled robots runs (`repair.repair`: cached seeds,
   then the native fallback, one hard budget, its own process group), then the request is planned once more.
   Without toppra both budgets are clamped to 0 unless `--allow-stack-mismatch` is given (result `stack_guard`
   "repair disabled: toppra missing", a stderr note; see "Prepare"): the job plans cache-only and a miss exits 3.
4. `PLAN.json` is umarell's plan-store record, as `plan_store.save_plan` writes it and as U3's plan files are:
   `{"tree_id", "cell_id", "created_s", "status": "ok", "num_operations", "behaviour_tree"}`. The tree is the
   native tree converted for plain umarell and btexport (`plan_cell.to_umarell_tree`):
   * each weld's contact motion becomes the weld leg (`operation_id` the weld id, `operation_type` "weld",
     `hold_duration_s` the sum of its squeeze / weld / hold dwells), the `WeldPhase` nodes are removed;
   * `WeldBoundary` nodes (rest acknowledgements without motion) are removed;
   * the native executor's ports `controller_exact`, `independent_clocks`, `initial_velocity` are dropped;
   * every trajectory is densified to waypoint gaps <= 0.02 s by sampling the exact quintic controller curve
     (positions, velocities, accelerations; every original waypoint kept): the native transfers have knots seconds
     apart, which a linear interpolation (umarell's executor, btexport) would turn into another motion;
   * metadata keeps the native metadata, plus `generator: umarell_jam` and `umarell_jam_conversion`.
   The native tree stays published in `R/plans/bundles/<version>/` (result `native_bundle`).
5. `PLAN.json.result.json` (also `R/runs/<job>/result.json`): `status` (success | invalid_request | infeasible |
   timeout_or_unknown | setup_failed), `reason`, `timings` (setup_s, load_s, plan_s, prepare_s, replan_s, total_s,
   cycle: the plan's final_weld_s / final_retract_s / final_ready_s / simultaneous_execution_s), `cache`
   {hits, misses}, `weld_ids`, `robots`, and `exit_code`, `job`, `job_dir`, `program_id`, `scene_revision`, `plan`,
   `version_hash`, `native_bundle`, `assignments`, `missing_weld_ids`, `prepare`, `attempts`, `umarell_copy`,
   `stack` (the numerical library stack of the job, `stack.fingerprint()`).
6. `R/runs/<job>/progress.json` ({job, pid, phase, detail, started_s, updated_s, elapsed_s, beats, ...}) is written on
   every phase change (starting, setup, load, plan, prepare, replan, commit, done) and every 15 s by a heartbeat
   thread. The job id: `--job`, else `$UMARELL_JAM_JOB`, else `<cell>-<UTC>-<pid>`; the first output line names
   the file.

Exit codes: 0 success, 2 invalid request (bad options, unknown / unloadable cell, task program, welds, robots),
3 no complete plan (timeout_or_unknown, infeasible, a refused commit), 4 setup failed (no 8a02db9 / 76ea13a checkout, a patch
or install failure, numerical runtime missing).

btexport: `python -m tools.btexport --from plan:PLAN.json` loads and exports the converted plan in the default
`umarell` profile (checked on the session's body_weld_2 example: all export checks pass, `--allow-partial` for a
weld subset). Its replay of the plan follows umarell's executor rules (1 s completion margin per leg), so its
`executor_end_s` is longer than the native `final_ready_s`; its `umarell_schedule_makespan_s` equals it. The plan
is a per-robot sequence tree (each batch a Parallel of per-robot Sequences on independent clocks), which
btexport's `--profile full` refuses ("full profile of a gated (per-robot sequence) plan"): export umarell_jam plans
with the `umarell` profile; they are already timed at the cell limits. u3_retime leaves the converted plans
unchanged (their replayed acceleration stays below its 0.95 x a_max target).

## Cache compatibility

Component keys are `digest(dependencies)` (canonical JSON, sha256). The dependencies contain the cell's
geometry / limits / program keys, ids, poses, trajectory hashes, the planner revision `native.code_revision`
(sha256 of the patched copy's planner sources by *relative* path), the checker revision
`preparation.implementation_revision` (sha256 of eight safety modules by *file name*: `__init__`, cell, geometry,
trajectory, validation, native, preparation, store) and per-layer source hashes of further modules (connectors,
contact_refinement + phases, coordination_cache, electrode_geometry, phase_schedule, primitive_index ->
electrode_geometry + branch_routes, release, seed_repair, transfers, initial_connection -> six checkers,
route_connectors). No key contains a module path or package name; `SCHEMA`, `CHECKER_VERSION` and
`RETIMER_VERSION` are unchanged constants.

All those modules are byte-identical to `umarell_fast/weld_cache` except `initial_connection.py` and
`route_connectors.py`, which start workers by module name (`-m umarell_jam.<module>`, repository root
`parents[1]`). Their revisions are computed by `revision.source_sha256` with exactly those edits undone (each
edit must occur exactly once), which reproduces the original sha256. So `implementation_revision()` is
`1c905fc8…7a21` as before, every per-module revision is the original one, and an R/umarell built by `setup` from
the same 76ea13a checkout has the original planner revision (`32e3896b…3b50` on a
third_party/sample-umarell-76ea13a checkout, equal to the `geometry_cache_provenance.planner` of the session's example
results). tests/umarell_jam/test_cache_compat.py pins the per-module revisions and `implementation_revision()`
on every run. It pins the planner revision `32e3896b3992637d4cbd8dc02ea3bf736d73ac3b5d959e2e8b0baa34167f3b50` only
when `UMARELL_JAM_UMARELL` names a setup copy (R/umarell) or a plain 76ea13a checkout, which the test sets up into a
temporary directory first (`test_planner_revision_is_the_pinned_one`; skipped otherwise, since the checkout is
not part of every test environment).

On the public 8a02db9 the planner revision is a new one, `4bd36dc2d731eb4710836ba414dc0f861f2ebd0f7da3ef32a8866de0dd5676d6`
(pinned by the same test for an 8a02db9 checkout or copy): every component key contains it, so a cache certified on
76ea13a (the session's, the catalogue's 76ea13a bundles) is a complete miss on 8a02db9 and the other way round.
The checker revision `implementation_revision()` and every per-module revision are unchanged (no umarell_jam module
changed). A public install starts with an empty cache: `--repair-budget` / `--prepare-budget` fill it (the cold
plan), and its bundles carry the 8a02db9 planner revision in `planner` (and in their file name).

Consequences:

* An existing component cache built by the session stays readable with the same certificates: point `--cache` at
  it (or copy it to `R/components`) and plan the same workspace. The session planned a *staged* workspace, whose
  cell files contain its absolute location (staging rewrites `/var/lib/factoryos/` references into the copy), and
  that content is part of the geometry key: its components hit for `--workspace` = that staged directory (same
  path, same files), not for the live `/var/lib/factoryos` cells or a re-stage elsewhere. For those, rebuild with
  `prepare` (or let `--repair-budget` / `--prepare-budget` use the old entries as numerical seeds: seeds carry no
  authority, every repaired path is recertified).
* Keys do not depend on the workspace root otherwise: byte-identical cell files at different roots give the same
  keys (live FactoryOS cells reference `/var/lib/factoryos/...`, which `Cell` maps onto the root). A cache or bundle
  built on one workspace hits on another with the same cell files (tests/umarell_jam/test_bundle.py).
* What forces a rebuild: any change to the numerical / checker modules above (intended invalidation), a different
  Umarell checkout or patch set (planner revision), changed cell content (geometry / limits / program keys; per
  weld, unaffected welds are reused), or a different numerical library build. The last one is not part of any key:
  a cache prepared under another stack has the same keys. It is recorded instead (below), and an upgrade of the
  numerical stack requires a fresh cache.
* Numerical stack: `umarell_jam/stack.py` `fingerprint()` returns the versions of numpy, scipy, pinocchio, hppfcl
  (`hppfcl_is_coal_shim`: coal's compatibility module; then `hppfcl` is coal's version, also in `coal`), ompl and
  toppra (None when absent), `platform.machine()`, `sys.platform` and the Python major.minor. The versions come
  from `importlib.metadata`, so nothing heavy is imported. The session's stack: pin 2.7.0, hpp-fcl 2.4.4, numpy
  1.26.4, scipy 1.13.1, ompl 2.0.1, toppra 0.6.3, aarch64 Linux, CPython 3.12 (requirements-umarell.txt pins it
  except toppra, which PyPI has as an sdist only: `pip install -r requirements-umarell.txt toppra==0.6.3`).
  Bundles (`MANIFEST.json` `stack`), prepare (run log and state) and plan-cell (result.json) record it; `import`
  compares it, and `prepare` refuses without toppra (above). stack.py is not key-relevant: no key hashes it.
* Not a key: the plan `version_hash` (request, tree, planner, checker, runtime and the sha256 of every top-level
  `umarell_jam/*.py`) identifies a release and differs from the session's for the same tree; it is checked only
  between `plan` and `commit` in one process.

## Shared cache: `bundle`, `import`, `push`

```
python -m umarell_jam --root R bundle [--out F.tgz | DIR] [--cells …]
python -m umarell_jam --root R import [--strict-stack | --no-strict-stack] F.tgz …
python -m umarell_jam --root R push F.tgz --dest s3://<bucket>/catalogue/umarell-jam-cache/ [--region REGION]
```

A bundle (umarell_jam/bundle.py, standard library only) is a tar.gz with `MANIFEST.json`
(`bundle_schema` factoryos-planner-cache-v1, `kind` umarell-jam-components, `name` the suggested file name
`<cell>__<planner[:8]>-<impl[:8]>.tgz`, `component_schema`, `planner`, `impl` (= `implementation`),
`cell_planner_commit`, `cells` [{cell_id, geometry_hash, limits_hash}], `files` {relpath: sha256}, `created`,
`stack` the numerical stack of the machine that wrote it) and
the component files, unchanged, under `umarell-jam/<layer>/<key>.json`. It holds every valid Store entry of the
cells' geometry keys (a cell of the workspace: its current key; a cell not in the workspace, or every cell without
`--cells`: the keys the Store holds certified primitives for, so no workspace is needed), plus the timing-only phase
packs of the trajectories certified in it. Timeout entries, locks, pending writes and entries carrying a
certificate for another scene are left out.

`import` verifies a bundle completely before writing anything: regular files only, no `..`, no links, only
`MANIFEST.json` and `umarell-jam/<layer>/<key>.json`; the schema and kind; every member against its sha256 and the
member set against the manifest; the Store checks of every component (schema, key = digest(dependencies), payload
checksum); every component belongs to one of the manifest's geometry keys and every certificate in it
(`geometry_hash`) names that key. Any failure rejects the bundle (exit 1). Valid entries already present are never
overwritten (invalid ones are replaced), writes use `atomic_json` under the per-key lock and respect the Store's 2 GB
cap. Imported components are ordinary cache entries: the planner's lookups (keys with the current cell's
geometry, limits, planner and checker revisions) and all release checks decide whether they are used.

Stack check: `import` compares the bundle's `stack` with the local fingerprint. If any numerical package version
(numpy, scipy, pinocchio, hppfcl, the coal shim flag, ompl, toppra) or the machine differs, it prints a warning on
stderr that lists the differences and imports the bundle anyway (the result row carries `stack_differences` and
`stack_warning`). `--strict-stack` rejects such a bundle instead (exit 1, nothing written). The router-facing
default is configurable: `UMARELL_JAM_STRICT_STACK=1` makes strict the default, and `--no-strict-stack` overrides it
for one call. A bundle without `stack` (written before it was recorded) produces a warning, never a rejection, even
in strict mode. `push` skips the stack check: the uploader's stack is not the consumer's.

Authenticity: `import` checks integrity, not authenticity. The manifest and its checksums are written by whoever
writes the bundle, so a self-consistent forged bundle (recomputed keys, payload checksums, member sha256s and
certificates naming the bundle's geometry key) is accepted. Trust rests on who can write the catalogue prefix
(`catalogue/umarell-jam-cache/`): catalogue admins only. Planning still re-checks every used trajectory against its
certificate (`trajectory_hash` = digest of the points, the certificate's `geometry_hash` = the current cell's) and
runs the release checks, so a corrupted or mismatched entry is not used. These checks are consistency checks: they
do not authenticate a certificate.

`push` test-imports the bundle into an empty temporary Store, then runs
`aws s3 cp F <dest>/<name> --metadata sha256=<sha256> --only-show-errors` (`bundle.push`, callable by the router).

## Request and result

```json
{
  "schema_version": "weld-request-v1",
  "request_id": "unseen-selection",
  "cell_id": "body_weld_1",
  "program_id": "body_weld_1__spot_weld_1",
  "weld_ids": ["W02", "W03"],
  "enabled_robot_ids": ["robot1", "robot2"],
  "completed_weld_ids": [],
  "initial_state": {"mode": "configured_home"},
  "terminal_state": "home",
  "speed_profile": "cell_limits"
}
```

IDs are exact and order-independent. Duplicates, unknown fields/IDs, an ambiguous
program, contradictory group completion, stale scene revisions and unresolved
native program contradictions are rejected. Completed IDs must be included in
`weld_ids`; only the remaining IDs are pulsed. Empty selections are supported.
`hold` requests no terminal HOME move; `home` requests enabled arms at HOME/rest,
including when no weld remains. All installed arms stay in the collision scene.
An empty `hold` request requires acknowledged rest; a moving state returns
unknown without adding an implicit braking command.

For measured starts, replace `initial_state` with:

```json
{
  "mode": "measured",
  "timestamp_s": 1794312000.0,
  "robots": {
    "robot1": {"positions": [0, 0, 0, 0, 0, 0], "velocities": [0, 0, 0, 0, 0, 0]},
    "robot2": {"positions": [0, 0, 0, 0, 0, 0], "velocities": [0, 0, 0, 0, 0, 0]}
  }
}
```

That timestamp and configuration are illustrative; supply current Unix seconds
and actual radians/radians-per-second in configured joint order for **every**
installed robot. The freshness window is five seconds, with 0.1 s future clock
tolerance. States must satisfy actual joint limits. Moving inactive arms require a
controller acknowledgement and return unknown. Exact quintic brakes and HOME
connectors are independently certified. At-rest native routing is available with
a positive repair budget. Exact completed-weld contact/retracted endpoints can
reuse a certified return when the other installed arms are at HOME. Unsupported
connectors return unknown; they never reset the start state.
Native setup and connector validation run in a bounded process so that model
bindings, cancellation context and existing trajectory sessions in the calling
process are preserved. Certified start connections are reused by exact physical
state and dependencies; timestamps are revalidated on each new request.

CLI: `python -m umarell_jam --root R --workspace W plan request.json [--result r.json] [--timeout 30]
[--repair-budget 0] [--serial] [--allow-stack-mismatch]` (prints the result without the tree; without toppra the
repair budget is clamped to 0 and the result carries `stack_guard`, see "Prepare"), `serve` (one JSON request per input line),
`simulate bt.json [--jitter --slowdown --pause]`, `evaluate [--cells] [--limit]` (every program subset, to
`R/runs/subsets.jsonl`). Python:

```python
from umarell_jam.layout import Layout
from umarell_jam.service import PlannerService

layout = Layout.make("data/umarell_jam", "/var/lib/factoryos").activate()   # after `setup`
service = PlannerService(layout.workspace, layout.components, layout.umarell, layout.plans)
result = service.plan(request, timeout_s=30, repair_budget_s=0)
if result["status"] == "success":
    service.commit(result)
```

The installed adapter also accepts `constraints["weld_request"]` through
`agents.trajectory.agent.plan`; the operations argument must match the exact
selected operations. Existing model binding, cancellation context, tree ID and
native plan-store publication are retained (environment `WELD_WORKSPACE`,
`WELD_COMPONENT_CACHE`, `WELD_UMARELL_ROOT`, `WELD_OUTPUT` name the service's directories).

Results distinguish `success`, `invalid_request`, `infeasible` and
`timeout_or_unknown`. This implementation does not issue global infeasibility
claims from bounded searches. Only success includes an executable tree. Returned
timing separates final pulse, final retract and final ready. A native `max_s`
cycle limit is enforced; exceeding it returns unknown with candidate timing.
`target_s` is reported as an objective. Full request latency includes required
retiming, interlocks, loader/XML checks, simulated execution and publication.

Each publication contains `bt.json`, `blackboard.json`, `bt.xml`, and a checksummed
transaction manifest. The tree includes exact controller points, selected and
completed IDs, assignments in the result, initial states, process settings,
effective limits, scene/program/request/version hashes, component provenance and
rest-boundary interlocks. Hardware integration requires an acknowledged weld
process driver; absence of that driver fails execution rather than silently
crediting a pulse.

## Safe execution updates

`continuation.ExecutionSession(service, runtime, monitor, config)` wraps the real
Umarell executor. Call `run(plan)` and `queue(replacement_request)` from the
controller/application. Queueing preserves committed motion and pulses. At the
next rest boundary the wrapper reads all joint positions/velocities, records
completed pulse acknowledgements and plans pending work with:

```json
{
  "previous_version": "...",
  "boundary_id": "batch_0_ready",
  "acknowledged_state_hash": "..."
}
```

These are the `update` fields. Callers cannot install an update without a matching
executor acknowledgement in the service. A failed or superseded replacement
leaves the committed pending work intact. Controlled pause/resume occurs at
acknowledged rest boundaries. Emergency stops abort the current run; resume
requires a fresh measured-state request and certification.

## Geometry, timing and certificates

The importer resolves manifests, `.view` transforms, URDF/SRDF, the actual dressed
gun and rotated TCP, and all fixture/workpiece mesh dependencies. Missing geometry
is a load error. Robot/self/pair checks use conservative URDF convex hulls and
local SRDF exclusions; actual gun triangles and fixture/workpiece triangles form
the geometry backstop. The native patched obstacle/floor predicate is retained.
Disabled robots remain physical obstacles.

Contact is restricted to the distal electrode surface within 20 mm of the TCP
and the native 50 mm target volume, in the certified contact/retract portion.
Gun housing and contact arms remain ordinary collision geometry. The supplementary
checker subdivides tip triangles crossing the 20 mm boundary without changing
their surface. Boundary triangles unresolved at 2 mm remain ordinary geometry.
Refined certificates carry a separate checker hash.

HOME/approach/contact curves originate in native U3/OMPL and native weld-stroke
generation. Each primitive receives independent 400 Hz replay of the actual
quintic controller interpolant, analytic position/velocity/acceleration/jerk
extrema and continuous conservative interval bounds. Phase packs split at rest,
preserve the geometric curve, apply native phase/effective speed limits, and
validate exact reversal and controller interpolation. TCP stroke axis/lateral
constraints are checked continuously. Squeeze/pulse/hold use the supplied exact
durations, current, force and product settings.

Scheduling enumerates compatible batches with a bounded search and uses a
bitmask dynamic program per sequence group. It preserves a complete incumbent,
native concurrency caps and group order, compacts compatible per-arm routes, and
uses certified directed transfers where available. The transfer's own swept
occupancy must also clear every inactive arm at its
actual measured pose. An unresolved transfer falls back to the certified HOME
route. Bystander geometry is retained by cell revision behind an FK lock.
Transfer subsets are scored by the complete synchronized schedule duration.
A shorter path for one arm is retained only when it does not lengthen that
schedule; combinations of transfers are considered together.
Whole-path occupancy or
continuous two-clock certificates prove independence from relative arm clocks.
This permits delays, slowdowns and rest pauses; a nominal shared timeline alone
is insufficient. Runtime checks exact starts, at-rest arrivals and membership
of the measured configuration in the certified curve. The simulation monitor
relies on these certificates and is not a hardware obstacle sensor.

The coverage extension also searches assignment, endpoint branch and weld order
together using transfer costs. It compares the assembled candidate against the
ordinary scheduler and keeps the faster complete candidate. If that candidate
exceeds the native cycle limit, a second bounded search can overlap approaches
and returns at rest boundaries. Contact, squeeze, pulse, hold and retraction
remain one committed action. Every concurrently dispatched path and parked arm
needs an independent-clock proof; a separate validator checks the emitted phase
tree. These searches are bounded and do not prove global optimality or
infeasibility. `prepare_phase_pairs=True` prepares additional pair certificates
offline; it does not cache complete request plans.

Additional whole-arm IK and bidirectional joint-space routing provide candidates
for difficult welds and transfers. Their discrete collision checks are search
filters only. Every retained candidate still passes the full controller and
geometry certification. Contact refinement fits joint position against monotone
Cartesian stroke progress, applies a smooth progress law, retimes under the same
limits, and recertifies the entire changed curve. Original certified alternatives
remain available. Separate content-addressed layers record supplemental
primitives, refined contacts, phase envelopes and pair proofs.

The current driver supports six bounded revolute joints per arm. Two- and
four-arm cells were tested. Robot IDs and selection sizes are not hard-coded.
Prismatic/unbounded joints and a nonzero speed-dependent clearance profile are
explicitly rejected until their continuous bounds are implemented.

## Cache, invalidation and policy import

Layers include strict/refined native candidates and primitives, per-weld geometry
indices, phase packs, stroke proofs, exact-state connectors, directed transfers
and pair compatibility. `Store` uses canonical JSON hashes, payload checksums,
atomic fsync/rename, per-key locks, transactional publication and a 2 GB growth
cap. Incomplete/corrupt/version-mismatched entries are ignored. Full model objects
are retained per service/process; their source content is rechecked.
The service also keeps validated component payloads in memory, with a 64 MiB
serialized-file-size budget (Python object overhead is additional).
Reads return isolated copies. File replacement, metadata changes or deletion
invalidate that copy; subsequent reads repeat the store's schema, dependency and
payload-checksum checks. This avoids repeated JSON decoding while leaving
request selection, scheduling and final plan validation uncached.
Prepared phase proofs can be read without constructing an FK geometry model;
the model is constructed lazily when a missing phase or parked-pose proof needs it.

Dependencies include geometry/model/mesh/SRDF/TCP/base/obstacle content, joint
order and position limits, target/approach/contact semantics, dynamic limits,
planner/checker/retimer versions and numerical tolerances. Per-weld indices permit
unaffected weld reuse. Dynamic-limit and dwell changes can preserve geometric
curves but require new phase/process checks. Timing-independent swept proofs
remain valid under monotone retiming of the same curve. Actual start connectors
use exact states, not quantised lookup keys.

When `repair_budget_s` (CLI `--repair-budget`) is positive, missing components
first search existing cache entries for numerical robot/weld path seeds, even
when their geometry hashes differ. The same cell, robot and weld IDs and joint
dimension are required. This also reads legacy caches; no pre-move registration
is needed. The scan is bounded and verifies component integrity. Old certificates
are not transferred: the seeds contain no execution authority.

Repair solves IK in the current scene from cached arm branches, tries alternative
branches, grows the fixed world-space weld stroke, and adjusts the cached transit.
Robot attempts are ranked by previous validation cost and current target distance,
instead of assigning every missing weld to the first enabled robot.
If necessary it tries a new direct or bounded joint-space route. Every accepted
path gets current-scene 400 Hz replay and continuous collision/limit checks,
stroke validation and phase retiming. Repaired components use a separately
versioned `placement_primitive` layer. Final scheduling, interlocks, BT/XML
loading, simulated execution, HOME return and scene-at-commit checks remain
mandatory. Moving an inactive neighbour also invalidates the scene certificate.

The repair subprocess shares one hard request budget across seed adaptation and
native fallback, reserves time for the latter, and is terminated on cancellation,
deadline or scene change. Results expose per-component seed/fallback attempts and
source geometry/trajectory hashes under `repair.details`. Failure remains
`timeout_or_unknown`; a moved base may make the requested weld unreachable or
leave the cell in collision. There is no promise that an arbitrary large move is
feasible or that seed repair always beats fresh planning.

Keep old entries in the same private writable component directory to use this
automatically:

```python
result = service.plan(request, timeout_s=270, repair_budget_s=240)
```

The existing Umarell entry point accepts the same bounds as
`constraints["weld_timeout_s"]` and `constraints["weld_repair_budget_s"]`,
alongside `constraints["weld_request"]`.

The default zero repair budget remains a bounded cache-only request. The CLI (`plan`, `plan-cell`) clamps a
positive budget to 0 when toppra is missing (`stack_guard`, see "Prepare"); the Python API and the Umarell entry
point's `constraints["weld_repair_budget_s"]` do not. An immutable
export must be copied into a writable runtime directory before using it as that
directory; preserve the immutable original.

`R/logs/cache-manifest.json` (`manifest`, alias `export`) records coverage and provenance.
Import an immutable export into a private writable cache or use it as a read-only seed;
never share a mutable store across two tasks. A staged workspace's keys depend on its
location (see "Cache compatibility"); restage/rebuild under a new root rather than
rewriting a certificate key. Component certificates are reusable evidence, not
preauthorised whole plans. The importing policy must still perform the documented
request release checks. `umarell_jam/policy_import.py` is a standalone standard-library
reader of the session's immutable export (FILES.json + cache-manifest.json):
`ImmutableCache(export_directory).candidates(cell_id, program_id, weld_id, enabled_robot_ids)`
verifies file and payload checksums and yields numerical candidates, exact branch
states and certificates. This data import has no Umarell or shared-code dependency.

`policy.plan_with_policy` demonstrates the execution handoff without a trained
checkpoint or changes to the policy session:

```python
from umarell_jam.policy_import import ImmutableCache
from umarell_jam.policy import plan_with_policy

def choose(candidates):
    return {weld: [min(values, key=lambda v: v["motion_duration_s"])]
            for weld, values in candidates.items()}

result = plan_with_policy(service, request, ImmutableCache(export_directory), choose)
```

The callback sees numerical candidate features and selects robot/trajectory
identities. The service accepts only current certified candidates and retains all
release checks. Invalid guidance or an unsuccessful guided plan triggers the
ordinary planner within the remaining request budget. The callback must respect
its latency budget. This example establishes an import and execution interface; it is not
evidence that a trained policy improves planning.

## Tests and known limits

`tests/umarell_jam/`: request contract, Store races / corruption, runtime store, controller polynomials, route
scheduling, seed repair (from the session), plus cache compatibility (pinned revisions), bundles (round trip,
tamper / kind / member rejection, push with a stubbed aws, key portability across workspace roots, the manifest's
stack, the stack warning, `--strict-stack` and `UMARELL_JAM_STRICT_STACK`), prepare (resume, the refusal without
toppra on a monkeypatched fingerprint, the fingerprint keys) and plan-cell (request, conversion, btexport loader and
replay, progress, exit codes, the result's stack, cache-only `plan` / `plan-cell` without toppra and
`--allow-stack-mismatch`). The planner-revision pin in test_cache_compat.py needs
`UMARELL_JAM_UMARELL` (a setup copy or a plain 8a02db9 / 76ea13a checkout). The real-cell tests in
test_release_runtime.py need a prepared cache: `UMARELL_JAM_ROOT` (default data/umarell_jam) with components and
`R/umarell`, or `UMARELL_JAM_CACHE`, `UMARELL_JAM_UMARELL` (a patched copy with umarell_jam hooks),
`UMARELL_JAM_WORKSPACE` with body_weld_1; the policy test also `UMARELL_JAM_EXPORT` (an immutable export directory).
Without them they skip.

No AMP/LLM procedure is involved: request interpretation is structured, and native numerical geometry dominates
preparation. Search and scheduling are bounded and incomplete. Unknown coverage and exceeded cycle limits are
reported as `timeout_or_unknown`, never as infeasible. A program with contradictory sequence groups is rejected.
The implementation does not claim all feasible requests have been discovered or certified.
