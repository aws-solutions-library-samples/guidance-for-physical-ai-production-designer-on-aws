#!/usr/bin/env python3
"""One umarell_u3 job of the planner router: factoryos-cell-planner's U3 plans the cell, btexport checks the plan.

    python3 -I umarell_u3.py --cell-planner <src> --umarell <checkout> --python <umarell venv python> \
        --root <writable dir> --cell-id <id> --cell-dir <FactoryOS cell> [--own-cell] [--task-file <json>] \
        --job-dir <dir> [--workers N] [--speed default|max] -- <btexport command line with {u3_plan}>

U3 (factoryos-cell-planner umarell_fast/u3, `python -m umarell_fast.u3`) is umarell's own planning tools (OMPL,
its gates, its behaviour-tree assembler) driven by a deterministic parallel orchestrator instead of the LLM, on a
copy of the plain umarell commit with the upstream fixes for it applied (eccb1bc: docs/upstream/patches, set v5;
76ea13a: docs/upstream/patches_76ea13a, set v8; dc60669: docs/upstream/patches_dc60669, set v9; UMARELL_BASE tells U3
which). No Bedrock call. The plain commit is
<umarell>-base (deploy/umarell-base.txt) when the chat's checkout already has the fixes, else the checkout itself.

The cell planner's source tree is read-only and U3 writes under its own root (data/umarell_fast: the patched copy and
the per-run cell copies; results/umarell_fast), so it runs from ``--root``: a copy of umarell_fast/ and
docs/upstream/patches*/, third_party/sample-umarell[-<base>] -> the plain commit (what umarell_fast/apply.py copies and
patches). The patched copy is rebuilt when the cell planner or the umarell checkout changed. The router runs jobs
in parallel: the setup (the copy in --root, the patched copy, the staged cell) holds --root/.setup.lock, and the
router runs one U3 job per cell at a time (U3 plans the staged cells/<cell id> of the shared patched copy).

The cell U3 plans (in the patched copy's cells/<cell id>):
* umarell's own cell (welding_cell_2_small, welding_cell_2t: the patches add the tool spheres and the hull margin to
  their robots.yaml / obstacles.yaml) with the FactoryOS cell's robots/ (the meshes: umarell's archive has only the
  URDFs, and the self-collision fix checks the meshes) and, given --task-file, the cell's task program;
* with --own-cell (a cell umarell does not have, or one whose layout changed): the FactoryOS cell itself (its
  prepared agentic_planner/, copied; everything else linked) and its task program.

--speed max (the router's options.speed "max", its default; "normal" omits --speed): U3 plans at the robots' limits (`umarell_fast.u3 --speed max`: transit
velocity_scale 1.0, the weld stroke 0.28, planning.max_velocity_scaling / home_velocity_scale 1.0 in U3's per-run
cell copy only; the FactoryOS cell is not changed); the router's btexport line then carries --profile full (the
robots' joint limits: per joint the tighter of the URDF and the cell's robots.yaml, which Prepare Cell fills from the
robot sidecar, e.g. catalogue urdf-robot-collection/kr240r2900/planner.yaml: measured accelerations, datasheet speeds,
controller software limits). A cell planner whose U3 has no --speed is refused (exit 4).

Before that, u3_retime.py re-times the junctions of U3's plan whose replayed acceleration jumps (umarell's Hermite
fallback without TOPP-RA overshoots where a short lift-off step meets a long transit; u3_retime's doc): U3's own plan
is kept as u3/<tree id>.u3-original.json, the report's u3.retime says what changed (--no-retime: U3's plan as is).
Then the btexport command line (the router's, with {u3_plan} = U3's (re-timed) plan) exports and checks the plan like
any other planner's. Exit codes: btexport's (0 checks pass, 1 a check failed: published unverified, 2 btexport
cannot read the plan: U3's plan is published unverified with exit 1), 3 U3 found no complete plan, 4 setup failed.
Standard library only (run with the router's python3).
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import traceback
from pathlib import Path

SKIP = ("plans", "logs", "recordings", "checkpoints")
HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:   # -I: the script's folder is not on sys.path
    sys.path.insert(0, str(HERE))
import catalogue_cache as CC  # noqa: E402


WORKSPACE_ENV = Path("/etc/factoryos/workspace.env")
CATALOGUE_CACHE_PREFIX = CC.U3_PREFIX                 # in the workspaces' data bucket (FACTORY_OS_S3_BUCKET)


def catalogue_uri(prefix: str, override_var: str, env_file: Path | None = None) -> str | None:
    """catalogue_cache.catalogue_uri with this module's workspace.env."""
    return CC.catalogue_uri(prefix, override_var, env_file or WORKSPACE_ENV)


def catalogue_cache_uri(env_file: Path | None = None) -> str | None:
    """Where the shared U3 plan caches live: ``FACTORY_OS_U3_CACHE_S3`` (an s3:// prefix; ``0`` turns the pull off),
    else ``s3://<FACTORY_OS_S3_BUCKET>/catalogue/u3-plan-cache/`` of a workspace on S3 storage (its
    /etc/factoryos/workspace.env; every workspace role reads ``catalogue/*``, catalogue admins write it)."""
    return catalogue_uri(CATALOGUE_CACHE_PREFIX, "FACTORY_OS_U3_CACHE_S3", env_file)


def bucket_region(region: str | None = None) -> str | None:
    """The data bucket's own region first (workspace.env): an AWS_REGION in the environment may name another one."""
    return CC.bucket_region(region, WORKSPACE_ENV)


def _plan_cache_env(cache_dir: Path) -> dict:
    return {**{k: v for k, v in os.environ.items() if k in ("PATH", "HOME", "LANG")}, "U3_CACHE_DIR": str(cache_dir)}


def pull_catalogue_caches(cache_dir: Path, src: Path, region: str | None = None, timeout_s: float = 90.0) -> str:
    """Bring the catalogue's U3 plan caches into this host's cache before planning: ``aws s3 sync`` of the
    catalogue prefix into ``<cache>/../catalogue-pull`` and ``python -m umarell_fast.plan_cache import`` of every
    bundle new or changed since the last import (entries already here are kept; catalogue_cache.pull). One job at a
    time (a lock); never fails the plan -- returns what happened, for the log."""
    uri = catalogue_cache_uri()
    if not uri:
        return "no catalogue cache location (not a workspace on S3 storage)"
    return CC.pull(uri, cache_dir.parent / "catalogue-pull",
                   lambda b: [sys.executable, "-m", "umarell_fast.plan_cache", "import", str(b)],
                   env=_plan_cache_env(cache_dir), cwd=src, region=region, timeout_s=timeout_s, env_file=WORKSPACE_ENV)


def cell_planner_commit(src: Path) -> str:
    """The installed cell planner's commit (its .cell-planner.txt pin, as the router reads it), else git's, else ''."""
    pin = src / ".cell-planner.txt"
    if pin.is_file():
        parts = pin.read_text().split()
        if parts:
            return parts[0][:7]
    try:
        r = subprocess.run(["git", "-C", str(src), "rev-parse", "--short=7", "HEAD"], capture_output=True, text=True,
                           timeout=5)
        return r.stdout.strip()[:7]
    except (OSError, subprocess.SubprocessError):
        return ""


def cache_bundle(src: Path, cache_dir: Path, cell: str, out_dir: Path) -> tuple[Path, str]:
    """This host's U3 cache of ``cell`` as a catalogue bundle (``plan_cache --root <cache> export --cell``) and its
    catalogue name <cell>__<cell planner commit[:7]>. PushRefused when there is nothing to bundle."""
    commit = cell_planner_commit(src)
    if not commit:
        raise CC.PushRefused(f"no cell planner commit for {src} (.cell-planner.txt): no bundle name")
    out = Path(out_dir) / f"{cell}__{commit}.tgz"
    r = subprocess.run([sys.executable, "-m", "umarell_fast.plan_cache", "--root", str(cache_dir), "export", "--cell",
                        cell, "--out", str(out)], cwd=str(src), env=_plan_cache_env(cache_dir), capture_output=True,
                       text=True, timeout=600, stdin=subprocess.DEVNULL)
    if r.returncode != 0 or not out.is_file():
        raise CC.PushRefused(f"plan_cache export of {cell} failed (exit {r.returncode}): "
                             f"{(r.stderr or r.stdout).strip()[-300:]}")
    if (r.stdout.strip().split() or ["0"])[0] == "0":
        raise CC.PushRefused(f"this host has no U3 cache of cell {cell} ({cache_dir})")
    return out, out.stem


def publish_cache(src: Path, cache_dir: Path, cell: str, dry_run: bool = False) -> dict:
    """Save this host's U3 cache of ``cell`` to catalogue/u3-plan-cache/ (catalogue_cache.push: the gate, a test
    import, ``aws s3 cp``)."""
    CC.push_gate(WORKSPACE_ENV)
    with tempfile.TemporaryDirectory(prefix="u3-cache-bundle-") as tmp:
        bundle, name = cache_bundle(src, cache_dir, cell, Path(tmp))
        return CC.push(bundle, CATALOGUE_CACHE_PREFIX, name,
                       lambda t: [sys.executable, "-m", "umarell_fast.plan_cache", "--root", str(t), "import",
                                  str(bundle)],
                       env=_plan_cache_env(cache_dir), cwd=src, env_file=WORKSPACE_ENV, dry_run=dry_run)


def forget_failed_replay(export_code: int, cache_info: dict | None) -> bool:
    """A plan btexport did not pass (exit != 0) is not one to replay: drop the replay journal U3 wrote for this cell's
    cache (the cached task results stay -- each was a valid result of its task -- but the next run plans its own
    path instead of repeating this plan). Returns whether a journal was removed."""
    if export_code == 0 or not cache_info or not cache_info.get("dir"):
        return False
    j = Path(cache_info["dir"]) / "journal.json"
    try:
        j.unlink()
    except OSError:
        return False
    log(f"plan cache: export checks failed -> replay journal of this cell dropped ({j})")
    return True


def log(msg: str) -> None:
    print(f"[u3] {msg}", flush=True)


def has_meshes(robots: Path) -> bool:
    return robots.is_dir() and any((d / "meshes").is_dir() for d in robots.iterdir() if d.is_dir())


# The umarell commits U3 knows (umarell_fast/u3 --base), by their layout: 76ea13a moved the planners to
# agents/trajectory/backends/, dc60669 added utils/weld_surface.py and agents/trajectory/plan_jobs.py (the cell
# planner's umarell_fast/u3 tree_base); apply.py takes each from third_party/<name> with its own fix set.
BASES = {"08d5a39": "sample-umarell-08d5a39", "dc60669": "sample-umarell-dc60669", "76ea13a": "sample-umarell-76ea13a",
         "eccb1bc": "sample-umarell"}


def u3_source(umarell: Path) -> Path:
    """The plain umarell commit U3 patches: <umarell>-base (deploy/umarell-base.txt) when the chat's checkout is
    that commit with the fixes already applied, else the checkout itself (the plain commit, as before)."""
    base = umarell.with_name(umarell.name + "-base")
    return base.resolve() if (base / "agents").is_dir() else umarell


def base_of(source: Path) -> str:
    if not (source / "agents" / "trajectory" / "backends").is_dir():
        return "eccb1bc"
    if (source / "utils" / "weld_surface.py").is_file() and (source / "agents" / "trajectory" / "plan_jobs.py").is_file():
        # 08d5a39 (perf/body-weld-2-planning) added the compiled geometry kernels and the plan memo
        tools = source / "agents" / "trajectory" / "tools"
        if (tools / "fastgeom.py").is_file() and (tools / "plan_memo.py").is_file():
            return "08d5a39"
        return "dc60669"
    return "76ea13a"


def patch_dirs(src: Path) -> list:
    """The cell planner's upstream fix folders (docs/upstream/patches, patches_<commit>)."""
    return sorted(d for d in (src / "docs" / "upstream").glob("patches*") if d.is_dir())


def stamp_of(src: Path, umarell: Path) -> str:
    """What the copy in --root is made of: the cell planner's archive and the umarell checkout."""
    h = hashlib.sha256()
    for f in (src / ".cell-planner.sha256", umarell / ".umarell.sha256"):
        h.update(f.read_bytes() if f.is_file() else str(f.resolve()).encode())
    for d in [src / "umarell_fast"] + patch_dirs(src):   # a hand-made install: its files
        for f in sorted(d.rglob("*")):
            if f.is_file() and "__pycache__" not in f.parts:
                st = f.stat()
                h.update(f"{f.relative_to(src)} {st.st_size} {st.st_mtime_ns}\n".encode())
    return h.hexdigest()


def sync_root(src: Path, umarell: Path, root: Path) -> None:
    root.mkdir(parents=True, exist_ok=True)
    (root / "results" / "umarell_fast").mkdir(parents=True, exist_ok=True)   # U3 appends its $0 ledger line there
    stamp = stamp_of(src, umarell)
    marker = root / ".factoryos-stamp"
    if marker.is_file() and marker.read_text().strip() == stamp:
        return
    log(f"copying umarell_fast and the upstream patches from {src} to {root}")
    for rel in ("umarell_fast", "docs", "data/umarell_fast"):
        shutil.rmtree(root / rel, ignore_errors=True)
    shutil.copytree(src / "umarell_fast", root / "umarell_fast", ignore=shutil.ignore_patterns("__pycache__"))
    for d in patch_dirs(src):
        shutil.copytree(d, root / "docs" / "upstream" / d.name)
    tp = root / "third_party"
    tp.mkdir(exist_ok=True)
    for name in BASES.values():   # only the checkout's own base is linked: U3 refuses the other base's patches
        link = tp / name
        if link.is_symlink() or link.exists():
            link.unlink()
    (tp / BASES[base_of(umarell)]).symlink_to(umarell)
    marker.write_text(stamp + "\n")


def working_copy(py: Path, root: Path, env: dict) -> Path:
    """The patched copy (umarell_fast.u3's own working_copy: re-created when its patch list changed)."""
    code = ("import sys; sys.path.insert(0, sys.argv[1]); from umarell_fast import apply as A; "
            "from umarell_fast.u3.__main__ import working_copy; print('WC', working_copy(A, True)[0])")
    r = subprocess.run([str(py), "-c", code, str(root)], cwd=str(root), env=env, capture_output=True, text=True)
    for line in (r.stdout + r.stderr).splitlines():
        if line.startswith("[apply]"):
            log(line)
    if r.returncode != 0:
        sys.stderr.write(r.stdout[-4000:] + r.stderr[-4000:])
        raise SystemExit(f"[u3] error: could not build the patched umarell copy (exit {r.returncode})")
    wc = [l[3:] for l in r.stdout.splitlines() if l.startswith("WC ")]
    return Path(wc[-1])


def copy_task(task_file: Path, tasks: Path) -> str:
    tasks.mkdir(parents=True, exist_ok=True)
    name = "factoryos_task.json"
    shutil.copyfile(task_file, tasks / name)
    return name


def stage_cell(wc: Path, umarell: Path, cid: str, cell_dir: Path, own: bool, task_file: Path | None) -> str:
    """Put the cell U3 plans into the patched copy; returns the task file name for --task."""
    cdir = wc / "cells" / cid
    upstream = (umarell / "cells" / cid).is_dir()
    if own:
        if upstream and not (cdir / ".factoryos-cell").is_file():
            # umarell's own (patched) cell: moved aside, put back when the layout matches again
            aside = wc / "cells" / f".{cid}.umarell"
            shutil.rmtree(aside, ignore_errors=True)
            if cdir.exists():
                os.replace(cdir, aside)
        shutil.rmtree(cdir, ignore_errors=True)
        cdir.mkdir(parents=True)
        (cdir / ".factoryos-cell").write_text(str(cell_dir) + "\n")
        ap = cell_dir / "agentic_planner"
        shutil.copytree(ap, cdir / "agentic_planner", symlinks=True, ignore=shutil.ignore_patterns(*SKIP, "tasks"))
        # A FactoryOS reference cell has no scene.yaml or robots/ of its own: its view (.view/, the backend's) lays
        # out its Robo Fitter scene and the catalogue's robots / the products as a legacy cell has them.
        view = cell_dir / ".view"
        if (view / "scene.yaml").is_file() and not (cell_dir / "scene.yaml").is_file():
            for name in ("scene.yaml", "robots", "fixtures", "floorplans"):
                if (view / name).exists() and not (cdir / name).exists():
                    (cdir / name).symlink_to(view / name)
        for entry in sorted(cell_dir.iterdir()):
            if entry.name not in ("agentic_planner", ".view") and not (cdir / entry.name).exists():
                (cdir / entry.name).symlink_to(entry)
        if task_file is None:
            raise SystemExit(f"[u3] error: cell {cid} has no task program")
        return copy_task(task_file, cdir / "agentic_planner" / "tasks")
    if (cdir / ".factoryos-cell").is_file():
        shutil.rmtree(cdir)
        aside = wc / "cells" / f".{cid}.umarell"
        if aside.is_dir():
            os.replace(aside, cdir)
    if not cdir.is_dir():
        raise SystemExit(f"[u3] error: the patched umarell copy has no cell {cid}")
    robots = cdir / "robots"
    if has_meshes(cell_dir / "robots"):
        if robots.is_symlink() or robots.is_file():
            robots.unlink()
        elif robots.is_dir():
            shutil.rmtree(robots)
        robots.symlink_to(cell_dir / "robots")
    elif not has_meshes(robots):
        log(f"warning: neither {cell_dir / 'robots'} nor umarell's cell has robot meshes: the self-collision fix "
            "has nothing to check")
    tasks = cdir / "agentic_planner" / "tasks"
    stale = tasks / "factoryos_task.json"
    if stale.exists():
        stale.unlink()
    return copy_task(task_file, tasks) if task_file else "car_door_spot_welding.json"


def supports_speed(src: Path) -> bool:
    """Whether the cell planner's U3 takes --speed (umarell_fast/u3/__main__.py has its presets)."""
    try:
        return "SPEED_PRESETS" in (src / "umarell_fast" / "u3" / "__main__.py").read_text()
    except OSError:
        return False


def retime(plan: Path, robots_yaml: Path, keep: Path) -> dict:
    """u3_retime on U3's saved plan, in place (U3's own plan kept as ``keep``); the report's u3.retime."""
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    import u3_retime
    try:
        tree = json.loads(plan.read_text())
        info = u3_retime.retime_plan(tree, u3_retime.robot_limits(robots_yaml))
    except (OSError, ValueError) as e:
        log(f"retime: skipped ({e})")
        return {"changed": False, "error": str(e)}
    for line in u3_retime.summary_lines(info):
        log(f"retime: {line}")
    out = {"changed": info["changed"], "target_frac_amax": info.get("target_frac_amax"),
           "legs": {k: r for k, r in (info.get("legs") or {}).items() if r.get("changed") or r.get("unrepaired")}}
    if info["changed"]:
        shutil.copyfile(plan, keep)
        tmp = plan.with_name(f".{plan.name}.tmp")
        tmp.write_text(json.dumps(tree))
        os.replace(tmp, plan)
        out["original"] = str(keep)
    else:
        log("retime: no junction above "
            f"{info.get('target_frac_amax', u3_retime.TARGET)} x a_max, U3's plan as is")
    return out


def export_and_check(plan: Path, export_cmd: list, placeholder: str) -> tuple[int, Path | None, float]:
    """The router's btexport command line on ``plan`` (``placeholder`` in it = the plan's path): (exit code, the
    report's path, export seconds). btexport's exit 2 (it cannot read the plan, e.g. not a two-arm cell) publishes the
    planner's own plan unverified: exit 1, with a report saying so."""
    export_cmd = [c.replace(placeholder, str(plan)) for c in export_cmd]
    t_export = time.time()
    r = subprocess.run(export_cmd, stdin=subprocess.DEVNULL)
    export_s = round(time.time() - t_export, 1)   # btexport: export, URDF limit checks, dense check
    report_path = Path(export_cmd[export_cmd.index("--report") + 1]) if "--report" in export_cmd else None
    out_path = Path(export_cmd[export_cmd.index("--out") + 1]) if "--out" in export_cmd else None
    code = r.returncode
    if code == 2 and out_path and report_path:
        tree = json.loads(plan.read_text())
        out_path.write_text(json.dumps(tree))
        report_path.write_text(json.dumps({"export_pass": False, "checks": {"export": {"pass": False}},
                                           "timing": {}, "warnings": [
            "btexport could not export / check this plan (exit 2); umarell's own gates passed it"]}))
        code = 1
    return code, report_path, export_s


def merge_report(report_path: Path | None, key: str, info: dict) -> None:
    """Add ``info`` to btexport's report as ``key`` (the router's plan_status shows it), atomically."""
    if not report_path or not report_path.is_file():
        return
    try:
        rep = json.loads(report_path.read_text())
        rep[key] = info
        tmp = report_path.with_name(f".{report_path.name}.tmp")
        tmp.write_text(json.dumps(rep, indent=1))
        os.replace(tmp, report_path)
    except (OSError, ValueError):
        pass


def auto_push(src: Path, cache_dir: Path, cell: str) -> str:
    """The router's opt-in save after a job btexport passed (FACTORY_OS_CATALOGUE_CACHE_PUSH=1): never raises, never
    fails the job; returns what happened (logged in full)."""
    try:
        r = publish_cache(src, cache_dir, cell)
        note = f"saved {r['uri']} ({r['bytes']} bytes, sha256 {r['sha256'][:12]})"
    except CC.PushRefused as exc:
        note = f"not saved: {exc}"
    except Exception:  # noqa: BLE001 -- a save never fails the job
        note = "not saved: " + traceback.format_exc().strip().replace("\n", " | ")
    log(f"catalogue cache push: {note}")
    return note


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--" not in argv:
        print("usage: umarell_u3.py <options> -- <btexport command line>", file=sys.stderr)
        return 4
    i = argv.index("--")
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--cell-planner", required=True)
    ap.add_argument("--umarell", required=True)
    ap.add_argument("--python", required=True, help="the umarell venv's python (U3 runs umarell's tools)")
    ap.add_argument("--root", required=True, help="writable root U3 runs from")
    ap.add_argument("--cell-id", required=True)
    ap.add_argument("--cell-dir", required=True)
    ap.add_argument("--own-cell", action="store_true")
    ap.add_argument("--task-file", default=None)
    ap.add_argument("--job-dir", required=True)
    ap.add_argument("--workers", type=int, default=int(os.environ.get("FACTORY_OS_U3_WORKERS", "0") or 0))
    ap.add_argument("--speed", choices=("default", "max"), default="default",
                    help="max: U3 plans at the robots' limits (umarell_fast.u3 --speed max)")
    ap.add_argument("--no-retime", action="store_true",
                    default=os.environ.get("FACTORY_OS_U3_RETIME", "1") == "0",
                    help="export U3's plan as is (no u3_retime junction repair)")
    a = ap.parse_args(argv[:i])
    export_cmd = argv[i + 1:]
    src, umarell, root = Path(a.cell_planner), Path(a.umarell).resolve(), Path(a.root)
    source = u3_source(umarell)   # the plain commit U3 patches (cells and meshes still come from --umarell)
    job = Path(a.job_dir)
    job.mkdir(parents=True, exist_ok=True)
    env = {k: v for k, v in os.environ.items() if k in ("PATH", "HOME", "LANG", "LC_ALL", "TMPDIR", "USER")}
    env.update(PYTHONUNBUFFERED="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1",
               UMARELL_BASE=base_of(source))
    env.update({k: v for k, v in os.environ.items() if k.startswith("UF_")})   # U3's own knobs (UF_PAIR_VERIFY, ...)
    # U3's persistent task cache (umarell_fast/plan_cache.py, cell planners that have it): keyed by the cell's
    # content, so a re-plan of an unchanged cell -- or the same reference cell on any workspace -- replays the work
    # done before. <work>/../cache/u3 (FactoryOS: /var/lib/factoryos/cell-planner/cache/u3); FACTORY_OS_U3_CACHE=0
    # turns it off, FACTORY_OS_U3_CACHE_DIR moves it.
    if os.environ.get("FACTORY_OS_U3_CACHE", "1") != "0" and (src / "umarell_fast" / "plan_cache.py").is_file():
        env.update(U3_PLAN_CACHE="1", U3_CACHE_DIR=os.environ.get("FACTORY_OS_U3_CACHE_DIR")
                   or str(root.resolve().parent.parent / "cache" / "u3"))
        # btexport's dense check of a motion it checked before in the same cell (a warm re-plan reproduces the plan)
        os.environ.setdefault("BTEXPORT_CHECK_CACHE_DIR", str(Path(env["U3_CACHE_DIR"]).parent / "btexport-check"))
        # the catalogue's shared caches first (body_weld_3 & co, pre-populated): a cell planned anywhere is warm here
        t_pull = time.time()
        try:
            note = pull_catalogue_caches(Path(env["U3_CACHE_DIR"]), src)
        except Exception as exc:  # noqa: BLE001 -- the shared cache never fails a plan
            note = f"failed ({exc!r})"
        log(f"plan cache from the catalogue: {note} ({time.time() - t_pull:.1f} s)")
    t0 = time.time()
    try:
        if not (src / "umarell_fast" / "u3").is_dir():
            raise SystemExit(f"[u3] error: {src} has no umarell_fast/u3 (cell planner too old)")
        if a.speed != "default" and not supports_speed(src):
            raise SystemExit(f"[u3] error: {src}'s U3 has no --speed (cell planner too old for speed {a.speed})")
        root.mkdir(parents=True, exist_ok=True)
        with open(root / ".setup.lock", "w") as lock:   # U3 jobs of other cells set up at the same time
            fcntl.flock(lock, fcntl.LOCK_EX)
            sync_root(src, source, root)
            wc = working_copy(Path(a.python), root, env)
            task = stage_cell(wc, umarell, a.cell_id, Path(a.cell_dir), a.own_cell,
                              Path(a.task_file) if a.task_file else None)
    except SystemExit as e:
        print(e.code if isinstance(e.code, str) else f"[u3] error: setup failed ({e.code})", flush=True)
        return 4
    t_setup = time.time() - t0
    tag = job.name
    out = job / "u3"
    cmd = [a.python, "-m", "umarell_fast.u3", "--cell", a.cell_id, "--with-upstream-fixes", "--tag", tag,
           "--task", task, "--out", str(out)]
    if a.workers > 0:
        cmd += ["--workers", str(a.workers)]
    if a.speed != "default":
        cmd += ["--speed", a.speed]
    log(f"planning {a.cell_id} ({'the FactoryOS cell' if a.own_cell else 'umarell cell + FactoryOS meshes'}, "
        f"task {task}; speed {a.speed}; setup {t_setup:.1f} s)")
    result_line = None
    proc = subprocess.Popen(cmd, cwd=str(root), env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            bufsize=1)
    for line in proc.stdout:
        line = line.rstrip("\n")
        if line.startswith("[u3] RESULT "):
            result_line = line[len("[u3] RESULT "):]
        print(line if line.startswith("[u3]") else f"  {line}", flush=True)
    code = proc.wait()
    shutil.rmtree(root / "data" / "umarell_fast" / "runs_U3" / tag, ignore_errors=True)
    t_plan = time.time() - t0 - t_setup
    summary = {}
    try:
        summary = json.loads(result_line) if result_line else {}
    except ValueError:
        pass
    res = {}
    try:
        res = json.loads((out / f"{tag}.json").read_text())
    except (OSError, ValueError):
        pass
    plan_file = res.get("plan_file")
    plan = out / "plans" / Path(plan_file).name if plan_file else None
    if code != 0 or not res.get("success") or not plan or not plan.is_file():
        why = res.get("error") or (f"{len(res.get('unreachable') or [])} unreachable operation(s)"
                                   if res.get("unreachable") else f"exit {code}")
        print(f"[u3] error: U3 found no complete plan ({why})", flush=True)
        return 3
    u3 = {"wall_s": summary.get("wall_s") or res.get("wall_s"), "setup_s": round(t_setup, 1),
          "plan_s": round(t_plan, 1), "welds": summary.get("welds"), "n_steps": summary.get("n_steps"),
          "workers": res.get("workers"), "kernels": res.get("kernels_requested"),
          "patches": [Path(p).stem for p in res.get("patches") or []], "own_cell": a.own_cell, "task": task,
          "reallocated": res.get("reallocated"), "serialised_steps": res.get("serialised_steps"),
          "speed": res.get("speed") or {"preset": a.speed},
          # where U3's time went: its phases (cumulative s from its start), per task kind busy time, its plan cache
          "phase_t": res.get("phase_t"), "by_kind": (res.get("stats") or {}).get("by_kind"),
          "utilisation": (res.get("stats") or {}).get("utilisation"), "cache": res.get("cache")}
    if not a.no_retime:
        t_rt = time.time()
        u3["retime"] = retime(plan, wc / "cells" / a.cell_id / "agentic_planner" / "config" / "robots.yaml",
                              out / f"{plan.stem}.u3-original.json")
        u3["retime_s"] = round(time.time() - t_rt, 1)
    log(f"plan {plan.name}: {u3['welds']} welds, {u3['wall_s']} s planning; exporting and checking it")
    code, report_path, u3["export_s"] = export_and_check(plan, export_cmd, "{u3_plan}")
    forget_failed_replay(code, u3.get("cache"))
    if code == 0 and env.get("U3_CACHE_DIR") and CC.auto_push_enabled():
        u3["catalogue_push"] = auto_push(src, Path(env["U3_CACHE_DIR"]), a.cell_id)
    merge_report(report_path, "u3", u3)
    return code


if __name__ == "__main__":
    sys.exit(main())
