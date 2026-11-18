"""U3 driver: one deterministic (no-model) umarell plan with the parallel orchestrator.

    .venv-umarell-full/bin/python -m umarell_fast.u3 --cell welding_cell_2_small --tag s_u3_1 --workers 32
    .venv-umarell-full/bin/python -m umarell_fast.u3 --cell welding_cell_2_small --with-upstream-fixes --tag s_u3f_1

Creates / reuses the variant's working copy (``umarell_fast/apply.py``: data/umarell_fast/repo_U3[_fixes]/), a
per-run CELL_BASE_PATH under data/umarell_fast/runs_U3/<tag>/ (cell copied, gitignored -- contains the cell assets),
runs the orchestrator, and writes:

* ``results/umarell_fast/u3/<tag>.json``: outcome, per-phase times, every task (submit / done / worker), legs,
  steps, gate repairs, kernel self-check, worker utilisation;
* ``results/umarell_fast/u3/plans/<tree_id>.json``: umarell's saved plan (joint trajectories + BT; no meshes);
* a zero-cost line in ``results/umarell_fast/bedrock_ledger.jsonl`` (U3 makes no Bedrock call).

Speed (``--speed``): ``default`` plans as U3 always did (transit velocity_scale 0.3, approach 0.25, the cell's
``planning:`` block as it is: max_velocity_scaling 0.7 caps every requested speed, home_velocity_scale 0.4).
``max`` (:data:`SPEED_PRESETS`) plans the transit at 1.0 x the joint velocity limits and lifts the cell's ceiling
for this run (``planning.max_velocity_scaling`` and ``home_velocity_scale`` 1.0 in the per-run cell copy, never in
the cell itself), so umarell times and gates every leg at the speed it is meant to run at (the gate's speed margin
grows with it); the terminal weld stroke stays at 0.28 x v_max (umarell's weld-zone speed today, 0.7 x 0.4, and
btexport's ``full`` profile weld rule ``weld_vfrac``). Export such a plan with ``btexport --profile full`` (URDF
velocity limits, the robots' acceleration limits): umarell's own timing on a host without TOPP-RA is its Hermite
fallback, which does not reach the limits at any velocity_scale. Explicit ``--velocity-scale`` /
``--approach-velocity-scale`` / ``--max-velocity-scaling`` / ``--home-velocity-scale`` override the preset.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


BASE_ENV = "UMARELL_BASE"

# U3's planning speeds per --speed. None = U3's historic values and the cell's own planning: block.
SPEED_PRESETS = {
    "default": {"velocity_scale": 0.3, "approach_velocity_scale": 0.25,
                "max_velocity_scaling": None, "home_velocity_scale": None},
    # every leg at the robots' limits, the weld stroke at umarell's weld-zone speed (0.7 x 0.4 = btexport's 0.28)
    "max": {"velocity_scale": 1.0, "approach_velocity_scale": 0.28,
            "max_velocity_scaling": 1.0, "home_velocity_scale": 1.0},
}
PLANNING_KEYS = ("max_velocity_scaling", "home_velocity_scale")


def resolve_speed(speed: str, **explicit) -> dict:
    """The speeds of one run: the ``speed`` preset, each explicit (not None) value on top. Every value is a
    fraction of the joint velocity limits in (0, 1]."""
    if speed not in SPEED_PRESETS:
        raise SystemExit(f"[u3] unknown --speed {speed!r} (one of {sorted(SPEED_PRESETS)})")
    out = dict(SPEED_PRESETS[speed])
    for k, v in explicit.items():
        if k not in out:
            raise KeyError(k)
        if v is not None:
            out[k] = float(v)
    for k, v in out.items():
        if v is not None and not (0.0 < float(v) <= 1.0):
            raise SystemExit(f"[u3] {k} must be in (0, 1], not {v}")
    return out


def set_planning_speeds(cell_dir: Path, values: dict) -> dict:
    """Set ``planning.<key>`` (:data:`PLANNING_KEYS`, the non-None ones of ``values``) in the robots.yaml of the
    per-run cell copy ``cell_dir`` (either layout), line by line so umarell's comments stay; a key the block lacks
    is added to it. The file is replaced, not written through: the copy may link it to the real cell.
    Returns {key: [old, new]} (old None when absent); {} when nothing to set."""
    import re
    want = {k: float(values[k]) for k in PLANNING_KEYS if values.get(k) is not None}
    if not want:
        return {}
    path = next((Path(cell_dir) / rel for rel in ("agentic_planner/config/robots.yaml", "config/robots.yaml")
                 if (Path(cell_dir) / rel).is_file()), None)
    if path is None:
        raise SystemExit(f"[u3] no robots.yaml in {cell_dir}: cannot set the planning speeds")
    lines = path.read_text().splitlines(keepends=True)
    start = next((i for i, l in enumerate(lines) if re.match(r"planning:\s*(#.*)?$", l.rstrip("\n"))), None)
    if start is None:
        lines += ([] if not lines or lines[-1].endswith("\n") else ["\n"]) + ["planning:\n"]
        start = len(lines) - 1
    end = start + 1
    while end < len(lines) and (not lines[end].strip() or lines[end].startswith((" ", "\t", "#"))):
        end += 1
    changed, seen = {}, set()
    for i in range(start + 1, end):
        m = re.match(r"(\s+)(\w+):\s*([^#\s]+)?(\s*#.*)?$", lines[i].rstrip("\n"))
        if not m or m.group(2) not in want:
            continue
        k = m.group(2)
        seen.add(k)
        try:
            old = float(m.group(3))
        except (TypeError, ValueError):
            old = None
        lines[i] = f"{m.group(1)}{k}: {want[k]:g}    # U3 --speed: was {m.group(3)} (this run's cell copy only)\n"
        changed[k] = [old, want[k]]
    add = [f"  {k}: {want[k]:g}    # U3 --speed (this run's cell copy only)\n" for k in want if k not in seen]
    for k in want:
        if k not in seen:
            changed[k] = [None, want[k]]
    at = end
    while at > start + 1 and (not lines[at - 1].strip() or lines[at - 1].lstrip().startswith("#")):
        at -= 1                            # after the block's last key, not after the comments that follow it
    lines[at:at] = add
    tmp = path.with_name(f".{path.name}.u3tmp")
    tmp.write_text("".join(lines))
    if path.is_symlink():
        path.unlink()
    os.replace(tmp, path)
    return changed


def tree_base(src: Path) -> str | None:
    """Which umarell base a source tree's layout is: ``8a02db9`` (the public umarell: the 08d5a39 layout with
    upstream d749dc5's ``compacting_prompt`` in agents/trajectory/context_compaction.py and the public default cell,
    ``DEFAULT_CELL_ID = "welding_cell_2_small"`` in agents/cell_defaults.py), ``08d5a39`` (the dc60669 layout plus
    the compiled collision kernels, agents/trajectory/tools/fastgeom.py, and the reuse memo,
    agents/trajectory/tools/plan_memo.py),
    ``dc60669`` (the backends layout plus the converter's weld
    surface module, utils/weld_surface.py, and the per-run planning jobs, agents/trajectory/plan_jobs.py),
    ``76ea13a`` (planner backends are packages, agents/trajectory/backends/ompl/planner.py), ``eccb1bc``
    (agents/trajectory/tools/ompl_planner.py is the planner itself), None (not an umarell tree)."""
    src = Path(src)
    if (src / "agents" / "trajectory" / "backends" / "ompl" / "planner.py").is_file():
        if (src / "utils" / "weld_surface.py").is_file() and (src / "agents" / "trajectory" / "plan_jobs.py").is_file():
            tools = src / "agents" / "trajectory" / "tools"
            if (tools / "fastgeom.py").is_file() and (tools / "plan_memo.py").is_file():
                return "8a02db9" if is_public_8a02db9(src) else "08d5a39"
            return "dc60669"
        return "76ea13a"
    if (src / "agents" / "trajectory" / "tools" / "ompl_planner.py").is_file():
        return "eccb1bc"
    return None


def _has(path: Path, text: str) -> bool:
    try:
        return text in path.read_text()
    except (OSError, UnicodeDecodeError):
        return False


def is_public_8a02db9(src: Path) -> bool:
    """An 08d5a39-layout tree that is the public 8a02db9: d749dc5's prompt callable (08d5a39 has the pre_model_hook;
    our fix 28 adds the callable to a PATCHED 08d5a39 only) and the public default cell (08d5a39: welding_cell_2x).
    Neither file is touched by fix set v10 / v11 or the hook patches."""
    src = Path(src)
    return (_has(src / "agents" / "trajectory" / "context_compaction.py", "def compacting_prompt(")
            and _has(src / "agents" / "cell_defaults.py", 'DEFAULT_CELL_ID = "welding_cell_2_small"'))


def select_base(A, requested: str | None = None, log=print) -> str:
    """The umarell base U3 plans on.

    1. ``requested`` (``--base``), else the environment variable ``UMARELL_BASE``;
    2. else apply.py's default (the base of its default fix set: v9 -> dc60669); when that base's source tree
       (``A.BASES[base].src``) is absent and another base's is present, the newest of those (logged), so an install
       that only has ``third_party/sample-umarell-76ea13a`` keeps planning on 76ea13a / set v8, one with only
       ``third_party/sample-umarell`` on eccb1bc / set v5.

    The chosen base's source tree must exist and have that base's layout (:func:`tree_base`), else SystemExit: a
    76ea13a checkout linked as ``third_party/sample-umarell`` (or the reverse) is refused instead of patched with
    the other base's fix set."""
    if not hasattr(A, "BASES"):            # an apply.py from before 2026-10-23: one base
        return "eccb1bc"
    want = requested or os.environ.get(BASE_ENV) or None
    if want is not None and want not in A.BASES:
        raise SystemExit(f"[u3] unknown umarell base {want!r} (one of {sorted(A.BASES)})")
    base = want or A.FIX_SET_BASE[A.DEFAULT_FIX_SET]
    if want is None and not A.BASES[base].src.is_dir():
        # newest first (BASES lists them oldest first)
        other = [b for b in reversed(list(A.BASES)) if b != base and A.BASES[b].src.is_dir()]
        if other:
            log(f"[u3] default base {base}: {A.BASES[base].src} is missing -> base {other[0]} "
                f"({A.BASES[other[0]].src})")
            base = other[0]
    src = A.BASES[base].src
    if not src.is_dir():
        raise SystemExit(f"[u3] umarell base {base}: source tree {src} not found")
    have = tree_base(src)
    if have != base:
        raise SystemExit(f"[u3] umarell base {base}: {src} is a {have or 'non-umarell'} tree "
                         f"(set --base / {BASE_ENV}, or link the checkout under the base's name)")
    return base


ECCB1BC_DROPINS = ("u1", "np", "npplus", "fix")


def cell_workpiece_boxes(cell_base: Path, cell: str) -> list:
    """Ids of the ``is_workpiece`` obstacles of a cell (its generated obstacles.yaml, either layout)."""
    import yaml
    for rel in ("agentic_planner/config/obstacles.yaml", "config/obstacles.yaml"):
        p = Path(cell_base) / cell / rel
        if p.is_file():
            try:
                doc = yaml.safe_load(p.read_text()) or {}
            except yaml.YAMLError:
                return []
            return [str(o.get("id")) for o in (doc.get("obstacles") or [])
                    if isinstance(o, dict) and o.get("is_workpiece")]
    return []


def choose_kernels(requested: str, ubase: str, patches: list, workpiece: list, fixes: bool = False,
                   log=print) -> str:
    """The kernel set for ``--kernels``. ``auto``: x76 on 76ea13a / dc60669 (umarell's own predicate); on eccb1bc u1 when
    present, else npplus / fix -- EXCEPT for a cell with ``is_workpiece`` boxes on a copy with the no-workpiece-dip
    fix (22 / 22v2): the eccb1bc drop-ins (u1 / np / npplus / fix) skip workpiece boxes like eccb1bc does, fix 22
    checks them, so the drop-ins would disagree with the predicate (self-check fails) -> umarell's own (none).
    An explicit drop-in request is honoured with a warning (the self-check still guards it)."""
    nodip = any("no_workpiece_dip" in Path(str(p)).stem for p in patches)
    clash = bool(workpiece) and nodip
    why = (f"cell has {len(workpiece)} is_workpiece box(es) (e.g. {workpiece[0]}) and the copy has the "
           f"no-workpiece-dip fix: the eccb1bc drop-in kernels skip workpiece boxes, fix 22 does not") if clash else ""
    if ubase in ("08d5a39", "8a02db9"):
        # 08d5a39 (13d446b) vectorised / compiled exactly what x76 speeds up (_moving_hit in _config_is_free, the
        # Numba pair_min_gap in _check_robot_pair_collision, the YAML parse caches): x76's source guards refuse to
        # install there (the loop it mirrors is gone, the pair check is not the proven original). auto -> none
        # (umarell's own, already fast, predicate); an explicit x76 would fail in every worker's install.
        if requested == "auto":
            return "none"
        if requested == "x76":
            log(f"[u3] WARNING kernels x76 on {ubase}: its guards refuse (umarell vectorised those checks itself); "
                "use auto / none")
    if requested == "auto" and ubase != "eccb1bc":
        # the eccb1bc drop-ins mirror eccb1bc's predicate (no arm-vs-itself check, no envelope / tool hulls /
        # workpiece rule, 3-argument pair check) and refuse to install here; x76 keeps umarell's own predicate
        # (static obstacles incl. workpiece boxes are checked by umarell's original) and only vectorises its
        # moving-obstacle loop (exactly equal; umarell_fast/kernels_76ea13a.py)
        return "x76"
    if requested == "auto":
        if clash:
            log(f"[u3] kernels auto -> none: {why}")
            return "none"
        # U1's kernels (own numpy FK, both predicate variants, batched inter_robot_clearance) when present,
        # else the profile track's npplus / U2's fix-aware set; every set is self-checked during the run
        try:
            import umarell_fast.u1.kernels  # noqa: F401
            return "u1"
        except Exception:  # noqa: BLE001
            return "fix" if fixes else "npplus"
    if clash and requested in ECCB1BC_DROPINS:
        log(f"[u3] WARNING kernels {requested}: {why}; expect the self-check to fail and the run to fall back")
    return requested


def copy_spec(A, fixes: bool, base: str) -> tuple:
    """(destination, [(patch, sha256_16)], apply.py base args) of U3's working copy on ``base``."""
    import hashlib
    if hasattr(A, "BASES"):
        B = A.BASES[base]
        dest = ROOT / "data" / "umarell_fast" / ("repo_U3" + B.suffix + ("_fix" if fixes else ""))
        want = (list(A.fix_diffs(B.default_set)) if fixes else []) + list(A.hook_diffs("U3", base))
        pin = ["--base", base]
    else:
        dest = ROOT / "data" / "umarell_fast" / ("repo_U3" + ("_fix" if fixes else ""))
        want = (list(A.fix_diffs()) if fixes else []) + sorted(A.OURS.glob("U3_*.diff"))
        pin = []
    return dest, [(str(p.relative_to(ROOT)), hashlib.sha256(p.read_bytes()).hexdigest()[:16]) for p in want], pin


def working_copy(A, fixes: bool, base: str | None = None):
    """The variant's working copy via the shared umarell_fast/apply.py: ``data/umarell_fast/repo_U3[_<base>][_fix]``
    (apply.py's own names), with the base's default fix set (v5 on eccb1bc, v8 on 76ea13a, v9 on dc60669) and the base's U3 hook
    patches; re-created when its recorded patch list (UF_APPLIED.json) is not the current one. ``base``: see
    :func:`select_base` (None = ``UMARELL_BASE`` / apply.py's default)."""
    base = select_base(A, base)
    dest, want, pin = copy_spec(A, fixes, base)
    marker = dest / "UF_APPLIED.json"
    have = None
    if marker.exists():
        info = json.loads(marker.read_text())
        have = [(p["patch"], p["sha256_16"]) for p in info.get("patches", [])]
        if info.get("base", "eccb1bc") != base:
            have = None
    if have != want:
        A.main(["--variant", "U3", "--dest", str(dest), "--force"] + pin
               + (["--with-upstream-fixes"] if fixes else []))
    return dest, json.loads(marker.read_text())


def run_u2_engine(a, repo: Path, run_dir: Path, kernels: str, workers: int, tree_id: str, extra: dict) -> dict:
    """U2's orchestrator, every decision by its rule, no model client at all (``llm=None``, ``decider=None``)."""
    from umarell_fast.u2.decisions import Router
    from umarell_fast.u2.orchestrator import U2
    router = Router("rules", run_dir, decider=None, llm=None)
    cfg = {"cell": a.cell, "task": a.task, "repo": str(repo), "run_dir": str(run_dir / "u2engine"),
           "router": router, "llm": None, "kernels": kernels, "workers": workers, "seed": a.seed,
           "velocity_scale": a.velocity_scale, "verbose": False, "tree_id": tree_id, "env": extra}
    t0 = time.monotonic()
    try:
        res = U2(cfg).run()
    except Exception as exc:  # noqa: BLE001
        import traceback
        res = {"success": False, "error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc()}
    res.setdefault("wall_s", round(time.monotonic() - t0, 3))
    res["decision_summary"] = router.summary()
    for k in ("plan_file", "plan_path"):
        if res.get(k):
            res["plan_file"] = res[k]
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="U3: deterministic parallel umarell orchestrator (no LLM)")
    ap.add_argument("--cell", default="welding_cell_2_small")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--task", default="car_door_spot_welding.json")
    ap.add_argument("--with-upstream-fixes", action="store_true",
                    help="working copy with the base's default upstream fix set (v11 on 8a02db9, v10 on 08d5a39, v9 on dc60669, v8 on 76ea13a, v5 on eccb1bc)")
    ap.add_argument("--base", default=None,
                    help="umarell base: 8a02db9 | 08d5a39 | dc60669 | 76ea13a | eccb1bc (default: $UMARELL_BASE, else apply.py's default, "
                         "dc60669; else the newest base whose third_party/sample-umarell[-<base>] exists)")
    ap.add_argument("--workers", type=int, default=0, help="worker processes (default: cpu count, max 48)")
    ap.add_argument("--kernels", default="auto",
                    help="none | np | npplus | fix | u1 | x76 | auto. auto: on eccb1bc u1 (else npplus / fix); on "
                         "76ea13a / dc60669 x76 (umarell's own predicate, moving-obstacle loop vectorised; the eccb1bc drop-ins "
                         "re-create eccb1bc's predicate and refuse to install there); on 08d5a39 / 8a02db9 none (umarell vectorised and "
                         "Numba-compiled those checks itself; x76's guards refuse there). Every "
                         "set is self-checked against umarell's own predicate during the run; on a mismatch the planning is "
                         "re-run on umarell's own functions. auto on eccb1bc picks none for a cell with is_workpiece "
                         "boxes when the copy has fix 22 (the drop-ins skip workpiece boxes)")
    ap.add_argument("--ompl-parallel", choices=("auto", "on", "off"), default="auto",
                    help="OMPL_PARALLEL_CANDIDATES=1 in the workers (umarell's own switch: it forks its 4 candidate "
                         "solves). auto = on when the host has >= 16 CPUs")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--speed", choices=sorted(SPEED_PRESETS), default="default",
                    help="default: transit 0.3, approach 0.25, the cell's planning: block as is; max: transit 1.0, "
                         "approach 0.28, planning.max_velocity_scaling / home_velocity_scale 1.0 in this run's cell "
                         "copy (export with btexport --profile full)")
    ap.add_argument("--velocity-scale", type=float, default=None, help="transit speed (default: the --speed preset)")
    ap.add_argument("--approach-velocity-scale", type=float, default=None,
                    help="terminal weld stroke speed (default: the --speed preset)")
    ap.add_argument("--max-velocity-scaling", type=float, default=None,
                    help="the cell's speed ceiling for this run (planning.max_velocity_scaling in the run's copy)")
    ap.add_argument("--home-velocity-scale", type=float, default=None,
                    help="the return-to-HOME speed for this run (planning.home_velocity_scale in the run's copy)")
    ap.add_argument("--approach-offset", type=float, default=0.12)
    ap.add_argument("--engine", choices=("u3", "u2"), default="u3",
                    help="u3: umarell_fast/orchestrator.py with the rules Policy; u2: U2's orchestrator "
                         "(umarell_fast/u2/orchestrator.py) with its decision router in mode 'rules' and NO decider "
                         "and NO LLM client (every choice by its rule; park poses by code search only)")
    ap.add_argument("--max-repairs", type=int, default=12, help="blocked-step repairs before giving up")
    ap.add_argument("--out", default=str(ROOT / "results" / "umarell_fast" / "u3"))
    a = ap.parse_args(argv)
    a.out = str(Path(a.out).resolve())  # the umarell session chdirs into the working copy; pin --out to the caller's cwd
    speeds = resolve_speed(a.speed, velocity_scale=a.velocity_scale, approach_velocity_scale=a.approach_velocity_scale,
                           max_velocity_scaling=a.max_velocity_scaling, home_velocity_scale=a.home_velocity_scale)
    a.velocity_scale, a.approach_velocity_scale = speeds["velocity_scale"], speeds["approach_velocity_scale"]

    from umarell_fast import apply as A
    from umarell_fast.u2 import session as S
    from concurrent.futures.process import BrokenProcessPool
    from umarell_fast.orchestrator import Orchestrator, Params, PoolFailure
    from umarell_fast.u3 import RulePolicy

    fixes = a.with_upstream_fixes
    tag = a.tag or f"{'t' if a.cell.endswith('2t') else 's'}_u3{'f' if fixes else ''}_{int(time.time())}"
    t0 = time.time()
    repo, info = working_copy(A, fixes, a.base)
    ubase = info.get("base") or "eccb1bc"
    if a.engine == "u2" and ubase != "eccb1bc":
        raise SystemExit("[u3] --engine u2 (umarell_fast/u2) targets umarell eccb1bc; use --base eccb1bc")
    run_dir = ROOT / "data" / "umarell_fast" / "runs_U3" / tag
    base = S.make_cell_base(repo, run_dir, a.cell)
    planning_set = set_planning_speeds(base / a.cell, speeds)
    if planning_set:
        print(f"[u3] speed {a.speed}: planning " + ", ".join(f"{k} {o} -> {n:g}" for k, (o, n) in planning_set.items())
              + " (this run's cell copy)", flush=True)
    kernels = choose_kernels(a.kernels, ubase, [p["patch"] for p in info.get("patches", [])],
                             cell_workpiece_boxes(base, a.cell), fixes=bool(fixes),
                             log=lambda m: print(m, flush=True))
    ncpu = os.cpu_count() or 2
    ompl_par = a.ompl_parallel == "on" or (a.ompl_parallel == "auto" and ncpu >= 16)
    # with umarell's candidate fork every planning task can use 4 cores: one worker per 4 CPUs
    workers = a.workers or (max(4, ncpu // 4) if ompl_par else min(48, ncpu))
    extra = {"OMPL_PARALLEL_CANDIDATES": "1" if ompl_par else "0", "UMARELL_MODEL_CACHE": "1"}
    if ubase != "eccb1bc":
        # 76ea13a gates every planned step inside run_trajectory_planner against the OTHER arms' last configs --
        # in a worker those are whatever it planned before. U3 gates every step itself (gate_all) with the
        # assembler's bystanders, and the assembler gates again, so the in-call gate is off in U3's processes.
        extra["PLAN_STEP_GATE"] = "0"
    env = S.env_for(repo, a.cell, base, run_dir / "scratch", extra)
    params = Params(velocity_scale=a.velocity_scale, approach_velocity_scale=a.approach_velocity_scale,
                    approach_offset_m=a.approach_offset, max_repairs=a.max_repairs)
    tree_id = f"bt_{int(time.time())}_{a.cell}"
    print(f"[u3] tag {tag} cell {a.cell} base {ubase} fixes {fixes} ({info.get('upstream_fixes_set')}) workers "
          f"{workers} kernels {kernels} engine {a.engine} repo {repo}", flush=True)
    if a.engine == "u2":
        res = run_u2_engine(a, repo, run_dir, kernels, workers, tree_id, extra)
    else:
        orch = Orchestrator(repo, a.cell, base, env, RulePolicy(params), workers=workers, kernels=kernels,
                            seed=a.seed, task=a.task, log=lambda m: print(m, flush=True))
        failed = False
        try:
            res = orch.run(tree_id=tree_id)
        except (PoolFailure, BrokenProcessPool) as exc:
            # a worker died (the OOM killer) or hung: fail now, with the reason, instead of waiting forever
            failed = True
            res = {"success": False, "error": f"worker pool failure: {exc}", "pool_failure": str(exc),
                   "wall_s": round(time.time() - t0, 3)}
            print(f"[u3] error: worker pool failure: {exc}", flush=True)
        finally:
            # kill the workers even after a success: the less preferred assembly trials / dropped candidates still
            # running are not needed, and the interpreter's exit would otherwise wait for them (body_weld_3: up to
            # ~4 min after the plan was done)
            orch.close(kill=True)
    res.update(tag=tag, variant="U3", engine=a.engine, umarell_base=ubase,
               upstream_fixes_set=info.get("upstream_fixes_set"), policy=RulePolicy.name, cell=a.cell, with_upstream_fixes=fixes,
               workers=workers, kernels_requested=kernels, ompl_parallel=ompl_par, seed=a.seed,
               params=vars(params), speed={"preset": a.speed, **speeds, "planning_set": planning_set}, repo=str(repo.relative_to(ROOT)), patches=[p["patch"] for p in info.get("patches", [])],
               host_cpus=os.cpu_count(), total_wall_s=round(time.time() - t0, 3),
               bedrock={"calls": 0, "input_tokens": 0, "output_tokens": 0, "cost_usd": 0.0},
               created=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))
    out = Path(a.out)
    (out / "plans").mkdir(parents=True, exist_ok=True)
    if res.get("plan_file") and Path(res["plan_file"]).exists():
        dst = out / "plans" / Path(res["plan_file"]).name
        shutil.copy2(res["plan_file"], dst)
        res["plan_copy"] = str(dst.relative_to(ROOT)) if dst.is_relative_to(ROOT) else str(dst)
    (out / f"{tag}.json").write_text(json.dumps(res, indent=1, default=str))
    led = ROOT / "results" / "umarell_fast" / "bedrock_ledger.jsonl"
    with open(led, "a") as f:
        f.write(json.dumps({"t": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "agent": "uf-u3",
                            "run": f"U3 {tag} {a.cell}", "input_tokens": 0, "output_tokens": 0, "cost_usd": 0.0,
                            "note": "U3 is rules-only: no Bedrock call"}) + "\n")
    summ = {k: res.get(k) for k in ("tag", "success", "umarell_base", "upstream_fixes_set", "tree_id", "wall_s", "total_wall_s", "n_steps",
                                    "unreachable", "reallocated", "serialised_steps", "phase_t", "speed")}
    summ["welds"] = sum(1 for l in res.get("legs", []) if l["kind"] == "weld" and l["status"] == "ok")
    summ["stats"] = res.get("stats")
    print("[u3] RESULT " + json.dumps(summ, default=str), flush=True)
    return 0 if res.get("success") else 1


if __name__ == "__main__":
    sys.exit(main())
