#!/usr/bin/env python3
"""One umarell_jam job of the planner router: factoryos-cell-planner's umarell_jam plans the cell from its component
cache, btexport checks the plan; on a miss it can fall back to umarell_u3.

    python3 -I umarell_jam.py --cell-planner <src> --umarell <checkout> --python <umarell_jam venv python> \
        --root <writable dir> --workspace <FactoryOS data root> --cell-id <cell folder> --cell-dir <FactoryOS cell> \
        --job-dir <dir> [--task-file <json>] [--speed max|normal] [--welds W01,W02] [--robots r1,r2] [--timeout S] \
        [--repair-budget S] [--prepare-budget S] [--progress-timeout S] [--no-retime] \
        [--fallback-u3 --u3-root <dir> --u3-python <umarell venv python> --u3-cell-id <id> [--u3-own-cell]] \
        -- <btexport command line with {jam_plan}>

umarell_jam (factoryos-cell-planner umarell_jam/, `python -m umarell_jam`) is a cache-based umarell weld planner:
umarell 76ea13a with the cell planner's fix set v8 (docs/upstream/patches_76ea13a), U3's hook patches and its own
integration patch. It reuses certified weld primitives, transfers and phase schedules keyed by content hashes of the
cell geometry: ~0.5 s from a warm component cache, ~60 s cold (its patched umarell copy), against ~25 min for U3 on
a large cell. Its cache is built offline (`python -m umarell_jam --root R --workspace W prepare --cells ...`,
minutes per robot / weld pair); a plan-cell run repairs misses within --repair-budget (and prepares within
--prepare-budget), else it exits 3. Its CLI, called as is:

    python -m umarell_jam --root R --workspace W [--umarell <plain umarell checkout>] plan-cell --cell-id ID
        [--task-file F.json] [--welds W01 ...] [--robots robot1 ...] [--speed max|normal] [--timeout S]
        [--repair-budget S] [--prepare-budget S] --job JOB --out PLAN.json

(cell planner docs/umarell_jam.md) exit 0 success, 2 invalid request, 3 no complete plan (unknown / infeasible /
timeout, a miss not repaired in budget), 4 setup failed; PLAN.json.result.json {status, reason, timings, cache: {hits,
misses}, weld_ids, robots, ...}; R/runs/<job>/progress.json every 15 s. --task-file must be one of the cell's own
programs (W/cells/<ID>/agentic_planner/tasks): the router passes the cell's program, never umarell's copy. Both
speeds plan at the cell limits (--speed is recorded). umarell_jam holds R/.setup.lock itself (shared while it plans,
exclusive while it rebuilds R/umarell): this script takes no lock. It needs the session's numerical stack (pin 2.7.0,
hpp-fcl 2.4.4, numpy 1.26.4, toppra 0.6.3: the cache keys depend on it), so --python is its own venv
(provision-cell-planner.sh: /opt/factoryos-cell-planner/router/jam-venv), not umarell's.

--root (the router's <work>/umarell_jam, FACTORY_OS_JAM_ROOT) persists across jobs: the component cache
(<root>/components), the patched umarell copy, the runs. The catalogue's shared bundles (catalogue_cache.py:
s3://<data bucket>/catalogue/umarell-jam-cache/, FACTORY_OS_JAM_CACHE_S3; 0 = off) new since the last pull are synced
and `umarell_jam --root R import`ed before planning (entries already here kept). After a plan that filled cache misses
the cell's cache is saved back when the router has FACTORY_OS_CATALOGUE_CACHE_PUSH=1 (off by default; a catalogue
admin's workspace only; `bundle`, a test import, `aws s3 cp`: catalogue_cache.push); publish_cache.py saves one by
hand. A failed save never fails the job.

The plain umarell commit (--umarell of umarell_jam): FACTORY_OS_JAM_UMARELL, else <umarell>-76ea13a, else
<umarell>-base or the checkout itself when that is 76ea13a (umarell_u3.base_of); none: umarell_jam is called without
--umarell (its own default under R, else its exit 4).

A hang (neither R/runs/<job>/progress.json nor the output changed for --progress-timeout s, default 300) or a run
past --timeout + 120 s is killed: no complete plan (3). With --fallback-u3, a run without a plan (3, 4, a crash, a
hang) falls back to umarell_u3 (umarell_u3.main: the same job dir, its own root --u3-root, the same speed and task);
an invalid request (2) never does. The job dir's jam.json says what umarell_jam did (status, reason, cache hits /
misses, timings, fallback), and the btexport report gets it as "jam".

Then (as umarell_u3.py) u3_retime.py re-times the junctions of the plan (it leaves umarell_jam's converted plans
unchanged; --no-retime, FACTORY_OS_JAM_RETIME=0: skipped) and the router's btexport command line ({jam_plan} = the plan)
exports and checks it, in the umarell profile (btexport's full profile refuses umarell_jam's per-robot-sequence
trees; they are already timed at the cell limits). The umarell_u3 fallback exports U3's max-speed plan in the full
profile, as a umarell_u3 job does. Exit codes: btexport's (0
checks pass, 1 a check failed: published unverified, 2 btexport cannot read the plan: published unverified with exit
1), 2 umarell_jam refused the request, 3 no complete plan, 4 setup failed (with --fallback-u3: umarell_u3's codes).
Standard library only (run with the router's python3).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:   # -I: the script's folder is not on sys.path
    sys.path.insert(0, str(HERE))
import catalogue_cache as CC  # noqa: E402
import umarell_u3 as U  # noqa: E402

JAM_BASE = "76ea13a"                                  # the umarell commit umarell_jam is built on (fix set v8)
COMPONENTS = "components"                             # the component cache in --root
CACHE_PREFIX = CC.JAM_PREFIX                          # catalogue/umarell-jam-cache/ in the workspaces' data bucket
MANIFEST_KIND = "umarell-jam-components"              # umarell_jam bundle's MANIFEST.json kind
PROGRESS_TIMEOUT_S = 300.0                            # umarell_jam updates progress.json at least every 60 s
KILL_GRACE_S = 120.0                                  # past --timeout: umarell_jam did not stop by itself
FALLBACK_MARK = "falling back to umarell_u3"          # the router's phase: fallback (server.py looks for it)
EXIT_INVALID, EXIT_NO_PLAN, EXIT_SETUP = 2, 3, 4


def log(msg: str) -> None:
    print(f"[jam] {msg}", flush=True)


# ---------------------------------------------------------------- the plain umarell commit and the root
def jam_source(umarell: Path) -> Path | None:
    """The plain umarell 76ea13a umarell_jam patches (its --umarell), or None."""
    over = os.environ.get("FACTORY_OS_JAM_UMARELL", "").strip()
    if over:
        return Path(over).resolve() if (Path(over) / "agents").is_dir() else None
    for cand in (umarell.with_name(f"{umarell.name}-{JAM_BASE}"), U.u3_source(umarell)):
        if (cand / "agents").is_dir() and U.base_of(cand) == JAM_BASE:
            return cand.resolve()
    return None


_NUMBER = re.compile(r"-?\d+(?:\.\d+)?")


def _errors(x, out: list) -> list:
    """Every failure message (``error`` strings) nested in a receipt component's failures, depth first."""
    if isinstance(x, dict):
        if isinstance(x.get("error"), str) and x["error"]:
            out.append(x["error"])
        for v in x.values():
            if isinstance(v, (dict, list)):
                _errors(v, out)
    elif isinstance(x, list):
        for v in x:
            _errors(v, out)
    return out


def diagnose(res: dict, limit: int = 3) -> dict:
    """Why umarell_jam found no complete plan, from its result and the repair / prepare receipts it names.

    The result's own reason is a status (``missing_certified_primitives``) and a miss count, which reads as "build the
    cache". The receipts say why each weld failed: the most frequent causes (messages grouped with their numbers
    blanked, so robot1 / robot2 and contact depths group together), and ``blocker`` when one cause is shared by every
    failed component -- e.g. a robot model in self-collision at HOME, which no cache or budget can fix."""
    receipts = []
    if (res.get("prepare") or {}).get("receipt"):
        receipts.append(res["prepare"]["receipt"])
    for attempt in res.get("attempts") or []:
        rep = (attempt or {}).get("repair") or {}
        if rep.get("receipt"):
            receipts.append(rep["receipt"])
    groups: dict = {}
    failed = certified = 0
    for path in dict.fromkeys(receipts):
        try:
            receipt = json.loads(Path(path).read_text())
        except (OSError, ValueError):
            continue
        for comp in receipt.get("components") or []:
            if comp.get("status") == "certified":
                certified += 1
                continue
            msgs = _errors(comp.get("failures") or [], [])
            if not msgs:
                continue
            failed += 1
            for msg in dict.fromkeys(m.splitlines()[0] for m in msgs):
                group = groups.setdefault(_NUMBER.sub("#", msg)[:200], [0, msg[:300]])
                group[0] += 1
    top = sorted(groups.values(), key=lambda g: -g[0])[:limit]
    out = {"missing_weld_ids": list(res.get("missing_weld_ids") or []), "failed_components": failed,
           "certified_components": certified, "causes": [{"count": n, "error": e} for n, e in top]}
    if failed and top and top[0][0] == failed:
        out["blocker"] = top[0][1]
    return out


def diagnosis_text(d: dict) -> str:
    """The diagnosis for the job's error line: the missing welds and the shared (or most frequent) cause."""
    text = ""
    ids = d.get("missing_weld_ids") or []
    if ids:
        text += f"; missing {len(ids)} weld(s): {', '.join(ids[:6])}{', ...' if len(ids) > 6 else ''}"
    if d.get("blocker"):
        text += f"; every failed attempt: {d['blocker'][:240]}"
    elif d.get("causes"):
        c = d["causes"][0]
        text += f"; most frequent failure ({c['count']} of {d['failed_components']}): {c['error'][:200]}"
    return text


def cache_entries(root: Path) -> int:
    d = root / COMPONENTS
    try:
        return sum(1 for e in d.iterdir() if not e.name.startswith("."))
    except OSError:
        return 0


# ---------------------------------------------------------------- the catalogue's shared caches (S3)
# catalogue_cache.py does the S3 side for umarell_u3 and umarell_jam alike: the pull before every plan (bundles new
# since the last pull, `umarell_jam --root R import F`), the save (publish_cache.py, or after a job that filled
# misses with FACTORY_OS_CATALOGUE_CACHE_PUSH=1): `umarell_jam --root R bundle --out F --cells C`, a test import
# into an empty root, `aws s3 cp` to catalogue/umarell-jam-cache/<name>.tgz.
class JamCli:
    """`python -m umarell_jam --root R [--workspace W] <subcommand ...>` in the umarell venv, from the cell
    planner (plan-cell takes the workspace; bundle / import only the root)."""

    def __init__(self, python: str, src: Path, root: Path, workspace: Path, env: dict | None = None):
        self.python, self.src, self.root, self.workspace = python, Path(src), Path(root), Path(workspace)
        self.env = env if env is not None else {k: v for k, v in os.environ.items() if k in ("PATH", "HOME", "LANG")}

    def cmd(self, *args: str, workspace: bool = True, root: Path | None = None) -> list:
        return [self.python, "-m", "umarell_jam", "--root", str(Path(root or self.root).resolve()),
                *(["--workspace", str(self.workspace.resolve())] if workspace else []), *args]

    def run(self, *args: str, workspace: bool = True, timeout_s: float = 600.0) -> subprocess.CompletedProcess:
        return subprocess.run(self.cmd(*args, workspace=workspace), cwd=str(self.src), env=self.env,
                              capture_output=True, text=True, timeout=timeout_s, stdin=subprocess.DEVNULL)


def cache_uri(env_file: Path | None = None) -> str | None:
    """s3://<data bucket>/catalogue/umarell-jam-cache/ of a workspace on S3 storage; FACTORY_OS_JAM_CACHE_S3 (an
    s3:// prefix, used as is; 0 = off)."""
    return CC.catalogue_uri(CACHE_PREFIX, "FACTORY_OS_JAM_CACHE_S3", env_file)


def pull_shared_cache(cli: JamCli, uri: str | None = None, aws: str | None = None, region: str | None = None,
                      timeout_s: float = 90.0) -> str:
    """The catalogue's umarell_jam bundles into this host's cache before planning (catalogue_cache.pull: new or
    changed bundles only). No router lock: `umarell_jam import` verifies a bundle before writing and writes each
    entry atomically under the Store's per-key lock, so plan-cell runs may read the cache meanwhile. Never raises."""
    uri = uri or cache_uri()
    if not uri:
        return "no shared cache location (not a workspace on S3 storage, or FACTORY_OS_JAM_CACHE_S3=0)"
    return CC.pull(uri, cli.root / "catalogue-pull", lambda b: cli.cmd("import", str(b), workspace=False),
                   env=cli.env, cwd=cli.src, region=region, timeout_s=timeout_s, aws=aws)


def _manifest(bundle: Path) -> dict:
    try:
        with tarfile.open(bundle) as tf:
            m = next((x for x in tf.getmembers() if Path(x.name).name == "MANIFEST.json" and x.isfile()), None)
            return json.loads(tf.extractfile(m).read()) if m else {}
    except (OSError, ValueError, tarfile.TarError, AttributeError):
        return {}


def cache_bundle(cli: JamCli, cell: str, out_dir: Path) -> tuple[Path, str]:
    """This host's umarell_jam cache of ``cell`` as a catalogue bundle (``umarell_jam --root R bundle --out F
    --cells <cell>``) and its catalogue name <cell>__<planner[:8]>-<impl[:8]>: the one umarell_jam's bundle picked
    (its file name), else from its MANIFEST.json. PushRefused when there is nothing to save."""
    out_dir = Path(out_dir)
    try:   # --out a directory: umarell_jam names the bundle (MANIFEST.json "name")
        r = cli.run("bundle", "--out", str(out_dir), "--cells", cell, workspace=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise CC.PushRefused(f"umarell_jam bundle of {cell} failed ({exc})") from None
    if r.returncode != 0:
        raise CC.PushRefused(f"umarell_jam bundle of {cell} failed (exit {r.returncode}): "
                             f"{(r.stderr or r.stdout).strip()[-300:]}")
    made = sorted(out_dir.glob("*.tgz"))
    if len(made) != 1:
        raise CC.PushRefused(f"umarell_jam bundle of {cell} wrote {len(made)} bundles in {out_dir}")
    bundle, m = made[0], _manifest(made[0])
    if m.get("kind") != MANIFEST_KIND:
        raise CC.PushRefused(f"{bundle.name} is no umarell_jam component bundle (MANIFEST.json kind "
                             f"{m.get('kind')!r}, not {MANIFEST_KIND!r})")
    if bundle.stem.startswith(f"{cell}__"):
        return bundle, bundle.stem
    name = str(m.get("name") or "").removesuffix(".tgz")
    planner, impl = (str(m.get(k) or "") for k in ("planner", "impl"))
    if not name and planner and impl:
        name = f"{cell}__{planner[:8]}-{impl[:8]}"
    if not name.startswith(f"{cell}__"):
        raise CC.PushRefused(f"no catalogue name for {bundle.name} (MANIFEST.json: name / planner + impl)")
    return bundle, name


def publish_cache(cli: JamCli, cell: str, dry_run: bool = False) -> dict:
    """Save this host's umarell_jam cache of ``cell`` to catalogue/umarell-jam-cache/ (catalogue_cache.push: the gate
    first, a test import into an empty root, ``aws s3 cp``)."""
    CC.push_gate()
    with tempfile.TemporaryDirectory(prefix="jam-cache-bundle-") as tmp:
        bundle, name = cache_bundle(cli, cell, Path(tmp))
        return CC.push(bundle, CACHE_PREFIX, name, lambda t: cli.cmd("import", str(bundle), workspace=False, root=t),
                       env=cli.env, cwd=cli.src, dry_run=dry_run)


def auto_push(cli: JamCli, cell: str) -> str:
    """The router's opt-in save after a job that filled cache misses (FACTORY_OS_CATALOGUE_CACHE_PUSH=1): never
    raises, never fails the job; returns what happened (logged in full)."""
    try:
        r = publish_cache(cli, cell)
        note = f"saved {r['uri']} ({r['bytes']} bytes, sha256 {r['sha256'][:12]})"
    except CC.PushRefused as exc:
        note = f"not saved: {exc}"
    except Exception:  # noqa: BLE001 -- a save never fails the job
        note = "not saved: " + traceback.format_exc().strip().replace("\n", " | ")
    log(f"catalogue cache push: {note}")
    return note


# ---------------------------------------------------------------- the plan-cell run
def plan_cell_cmd(a: argparse.Namespace, source: Path | None, out: Path, job: str | None = None) -> list:
    """umarell_jam's plan-cell command line (its contract), from this script's options."""
    cmd = [a.python, "-m", "umarell_jam", "--root", str(Path(a.root).resolve()),
           "--workspace", str(Path(a.workspace).resolve())]
    if source is not None:
        cmd += ["--umarell", str(source)]
    cmd += ["plan-cell", "--cell-id", a.cell_id]
    if a.task_file:
        cmd += ["--task-file", str(Path(a.task_file).resolve())]
    if a.welds:
        cmd += ["--welds", *a.welds]
    if a.robots:
        cmd += ["--robots", *a.robots]
    cmd += ["--speed", a.speed]
    for flag, v in (("--timeout", a.timeout), ("--repair-budget", a.repair_budget),
                    ("--prepare-budget", a.prepare_budget)):
        if v is not None:
            cmd += [flag, f"{v:g}"]
    if job:
        cmd += ["--job", job]
    return cmd + ["--out", str(out)]


def csv(value: str) -> list:
    return [v.strip() for v in value.split(",") if v.strip()]


class Watch:
    """The run's liveness: its output lines and R/runs/<job>/progress.json (the job: $UMARELL_JAM_JOB, else the one
    run folder that appeared after the start)."""

    def __init__(self, root: Path, tag: str):
        self.runs = root / "runs"
        self.tag = tag
        self.before = set(self.runs.iterdir()) if self.runs.is_dir() else set()
        self.last = time.time()
        self.sig = None
        self.progress: dict = {}
        self.file: Path | None = None

    def touch(self) -> None:
        self.last = time.time()

    def _file(self) -> Path | None:
        if self.file is not None:
            return self.file
        own = self.runs / self.tag / "progress.json"
        if own.is_file():
            self.file = own
            return own
        new = [d for d in self.runs.iterdir() if d not in self.before and (d / "progress.json").is_file()] \
            if self.runs.is_dir() else []
        if len(new) == 1:
            self.file = new[0] / "progress.json"
        return self.file

    def poll(self) -> None:
        f = self._file()
        if f is None:
            return
        try:
            st = f.stat()
            sig = (st.st_mtime_ns, st.st_size)
        except OSError:
            return
        if sig != self.sig:
            self.sig = sig
            self.touch()
            try:
                self.progress = json.loads(f.read_text())
            except (OSError, ValueError):
                pass

    def idle_s(self) -> float:
        return time.time() - self.last


def _kill(proc: subprocess.Popen) -> None:
    """umarell_jam's whole process group (it runs in its own session: its planning workers too)."""
    for sig, wait in ((signal.SIGTERM, 10.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(proc.pid, sig)
        except (ProcessLookupError, PermissionError):
            return
        try:
            proc.wait(wait)
            return
        except subprocess.TimeoutExpired:
            continue


def run_jam(cmd: list, cwd: Path, env: dict, watch: Watch, timeout_s: float | None,
            progress_timeout_s: float) -> tuple[int, tuple[str, str] | None]:
    """umarell_jam's run: (its exit code, None or why it was killed: ("hung" | "timeout", reason)). Its output is
    echoed (indented, but its own [jam] lines)."""
    proc = subprocess.Popen(cmd, cwd=str(cwd), env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            bufsize=1, stdin=subprocess.DEVNULL, start_new_session=True)
    prev = {s: signal.getsignal(s) for s in (signal.SIGTERM, signal.SIGINT)}

    def stop(signum, frame):  # the router cancels / times out the job: umarell_jam's session goes too
        _kill(proc)
        raise SystemExit(128 + signum)

    for s in prev:
        signal.signal(s, stop)

    def pump():
        for line in proc.stdout:
            watch.touch()
            line = line.rstrip("\n")
            print(line if line.startswith("[jam]") else f"  {line}", flush=True)

    reader = threading.Thread(target=pump, daemon=True)
    reader.start()
    t0, killed = time.time(), None
    try:
        while proc.poll() is None:
            watch.poll()
            if watch.idle_s() > progress_timeout_s:
                killed = ("hung", f"no progress for {watch.idle_s():.0f} s (progress_timeout_s {progress_timeout_s:g})")
            elif timeout_s and time.time() - t0 > timeout_s + KILL_GRACE_S:
                killed = ("timeout", f"still running {time.time() - t0:.0f} s, past its --timeout {timeout_s:g} s")
            if killed:
                log(f"{killed[0]}: {killed[1]}; stopping umarell_jam")
                _kill(proc)
                break
            time.sleep(0.2)
        code = proc.wait()
    finally:
        for s, h in prev.items():
            signal.signal(s, h)
    reader.join(5)
    watch.poll()
    return code, killed


def write_json(path: Path, value: dict) -> None:
    tmp = path.with_name(f".{path.name}.tmp")
    tmp.write_text(json.dumps(value, indent=1))
    os.replace(tmp, path)


def retime_plan(plan: Path, a: argparse.Namespace, out: Path) -> dict | None:
    """u3_retime on the plan (umarell_u3.retime), with the cell's prepared robots.yaml, else umarell's for the cell."""
    for robots in (Path(a.cell_dir) / "agentic_planner" / "config" / "robots.yaml",
                   Path(a.umarell) / "cells" / (a.u3_cell_id or a.cell_id) / "agentic_planner" / "config"
                   / "robots.yaml"):
        if robots.is_file():
            return U.retime(plan, robots, out / f"{plan.stem}.jam-original.json")
    log("retime: skipped (no robots.yaml for the cell's joint limits)")
    return None


def fallback_u3(a: argparse.Namespace, export_cmd: list, why: str) -> int:
    """umarell_u3 on the same job (its own root, the same speed and task program): umarell_u3.main."""
    log(f"{FALLBACK_MARK}: {why}")
    argv = ["--cell-planner", a.cell_planner, "--umarell", a.umarell, "--python", a.u3_python or a.python,
            "--root", a.u3_root,
            "--cell-id", a.u3_cell_id or a.cell_id, "--cell-dir", a.cell_dir, "--job-dir", a.job_dir]
    if a.u3_own_cell:
        argv.append("--own-cell")
    if a.task_file:
        argv += ["--task-file", a.task_file]
    if a.speed == "max":
        argv += ["--speed", "max"]
    if a.no_retime:
        argv.append("--no-retime")
    export = [c.replace("{jam_plan}", "{u3_plan}") for c in export_cmd]
    if a.speed == "max" and "--profile" in export:
        # U3's max-speed plan is exported in the full profile only (umarell's own timing of it breaks the
        # acceleration limits without TOPP-RA); umarell_jam's per-robot-sequence plans in the umarell profile only
        export[export.index("--profile") + 1] = "full"
    return U.main(argv + ["--", *export])


def parse(argv: list) -> tuple[argparse.Namespace, list]:
    i = argv.index("--")
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--cell-planner", required=True)
    ap.add_argument("--umarell", required=True, help="the chat's umarell checkout (the plain commit: jam_source)")
    ap.add_argument("--python", required=True,
                    help="umarell_jam's python: its own venv (the session's numerical stack: its cache keys need it)")
    ap.add_argument("--root", required=True, help="umarell_jam's writable root: component cache, patched copy, runs")
    ap.add_argument("--workspace", required=True, help="the FactoryOS data root (cells/, catalogue/, products/)")
    ap.add_argument("--cell-id", required=True, help="the cell's folder in <workspace>/cells")
    ap.add_argument("--cell-dir", required=True)
    ap.add_argument("--task-file", default=None)
    ap.add_argument("--job-dir", required=True)
    ap.add_argument("--speed", choices=("max", "normal"), default="max")
    ap.add_argument("--welds", type=csv, default=None, help="comma-separated weld ids (a subset of the program)")
    ap.add_argument("--robots", type=csv, default=None, help="comma-separated robot names (a subset of the cell's)")
    ap.add_argument("--timeout", type=float, default=None, help="umarell_jam's own planning timeout (s)")
    ap.add_argument("--repair-budget", type=float, default=None)
    ap.add_argument("--prepare-budget", type=float, default=None)
    ap.add_argument("--progress-timeout", type=float,
                    default=float(os.environ.get("FACTORY_OS_JAM_PROGRESS_TIMEOUT_S") or PROGRESS_TIMEOUT_S))
    ap.add_argument("--no-retime", action="store_true", default=os.environ.get("FACTORY_OS_JAM_RETIME", "1") == "0")
    ap.add_argument("--fallback-u3", action="store_true", help="no plan: umarell_u3 plans the job instead")
    ap.add_argument("--u3-root", default=None, help="umarell_u3's root (the router's <work>/umarell_u3)")
    ap.add_argument("--u3-python", default=None, help="the umarell venv's python for the fallback (default --python)")
    ap.add_argument("--u3-cell-id", default=None, help="umarell_u3's cell id (the planner cell id)")
    ap.add_argument("--u3-own-cell", action="store_true")
    a = ap.parse_args(argv[:i])
    if a.fallback_u3 and not a.u3_root:
        ap.error("--fallback-u3 needs --u3-root")
    return a, argv[i + 1:]


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--" not in argv:
        print("usage: umarell_jam.py <options> -- <btexport command line>", file=sys.stderr)
        return EXIT_SETUP
    a, export_cmd = parse(argv)
    src, root, job = Path(a.cell_planner), Path(a.root), Path(a.job_dir)
    job.mkdir(parents=True, exist_ok=True)
    out = job / "jam"
    out.mkdir(parents=True, exist_ok=True)
    plan = out / "plan.json"
    info: dict = {"status": None, "speed": a.speed, "welds": a.welds, "robots": a.robots, "timeout_s": a.timeout,
                  "repair_budget_s": a.repair_budget, "prepare_budget_s": a.prepare_budget,
                  "fallback": {"enabled": a.fallback_u3, "used": False}}

    def finish(code: int, status: str, reason: str | None = None) -> int:
        info.update(status=status, exit_code=code, **({"reason": reason} if reason else {}))
        write_json(job / "jam.json", info)
        return code

    def no_plan(code: int, status: str, reason: str) -> int:
        if code != EXIT_INVALID and a.fallback_u3:
            info["fallback"].update(used=True, why=f"{status}: {reason}")
            finish(code, status, reason)
            u3_code = fallback_u3(a, export_cmd, f"{status}: {reason}")
            info["fallback"]["exit_code"] = u3_code
            write_json(job / "jam.json", info)
            report = Path(export_cmd[export_cmd.index("--report") + 1]) if "--report" in export_cmd else None
            U.merge_report(report, "jam", info)
            if u3_code in (EXIT_NO_PLAN, EXIT_SETUP):
                log(f"error: no complete plan: umarell_jam {status} ({reason}); its umarell_u3 fallback "
                    f"{'found none' if u3_code == EXIT_NO_PLAN else 'could not set up'} either")
            return u3_code
        verb = {EXIT_INVALID: "refused the request", EXIT_SETUP: "setup failed"}.get(code, "found no complete plan")
        log(f"error: umarell_jam {verb} ({status}: {reason})"
            + ("" if code == EXIT_INVALID or a.fallback_u3 else "; no fallback (options.fallback false)"))
        return finish(code, status, reason)

    if not (src / "umarell_jam").is_dir():
        return no_plan(EXIT_SETUP, "setup_failed", f"{src} has no umarell_jam (cell planner too old)")
    umarell = Path(a.umarell).resolve()
    source = jam_source(umarell)
    info["umarell"] = str(source) if source else None
    if source is None:
        log(f"warning: no plain umarell {JAM_BASE} (FACTORY_OS_JAM_UMARELL, {umarell}-{JAM_BASE}, {umarell}-base): "
            "umarell_jam runs without --umarell")
    root.mkdir(parents=True, exist_ok=True)
    tag = job.name
    env = {k: v for k, v in os.environ.items() if k in ("PATH", "HOME", "LANG", "LC_ALL", "TMPDIR", "USER")}
    env.update(PYTHONUNBUFFERED="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1",
               UMARELL_BASE=JAM_BASE, UMARELL_JAM_JOB=tag)
    env.update({k: v for k, v in os.environ.items() if k.startswith(("UF_", "JAM_", "UMARELL_JAM_"))})
    cli = JamCli(a.python, src, root, Path(a.workspace), env)
    t0 = time.time()
    if os.environ.get("FACTORY_OS_JAM_CACHE_PULL", "1") != "0":
        try:
            note = pull_shared_cache(cli)
        except Exception as exc:  # noqa: BLE001 -- the shared cache never fails a plan
            note = f"failed ({exc!r})"
        log(f"shared cache from the catalogue: {note} ({time.time() - t0:.1f} s)")
        info["cache_pull"] = note
    info["cache_entries"] = cache_entries(root)
    if not info["cache_entries"]:
        log(f"warning: the component cache {root / COMPONENTS} is empty: every weld is a miss "
            "(prepare it: python -m umarell_jam --root R --workspace W prepare --cells ...)")
    cmd = plan_cell_cmd(a, source, plan, tag)
    log(f"planning {a.cell_id} (speed {a.speed}; {info['cache_entries']} cache entries; "
        f"fallback {'umarell_u3' if a.fallback_u3 else 'none'})")
    watch = Watch(root, tag)
    t_run = time.time()
    # No lock here: umarell_jam takes R/.setup.lock itself (shared while it plans on R/umarell, exclusive while it
    # rebuilds it), and its heartbeat keeps progress.json fresh while it waits for it.
    code, killed = run_jam(cmd, src, env, watch, a.timeout, a.progress_timeout)
    res: dict = {}
    try:
        res = json.loads(plan.with_name(plan.name + ".result.json").read_text())
    except (OSError, ValueError):
        pass
    info.update(wall_s=round(time.time() - t_run, 2), pull_s=round(t_run - t0, 2), jam_exit=code,
                cache=res.get("cache"), timings=res.get("timings"), weld_ids=res.get("weld_ids"),
                robots_planned=res.get("robots"), progress=watch.progress or None)
    if killed:
        return no_plan(EXIT_NO_PLAN, *killed)
    status, reason = res.get("status") or "", res.get("reason") or ""
    if code == EXIT_INVALID:
        return no_plan(EXIT_INVALID, status or "invalid_request", reason or "exit 2")
    if code == EXIT_NO_PLAN:
        miss = (res.get("cache") or {}).get("misses")
        info["diagnosis"] = diagnose(res)
        return no_plan(EXIT_NO_PLAN, status or "no_plan",
                       (reason or "exit 3") + (f"; {miss} cache miss(es)" if miss else "")
                       + diagnosis_text(info["diagnosis"]))
    if code != 0:
        return no_plan(EXIT_SETUP, status or "setup_failed", reason or f"exit {code}")
    if not plan.is_file():
        return no_plan(EXIT_NO_PLAN, "no_plan", f"exit 0 without {plan.name}")
    if not a.no_retime:
        t_rt = time.time()
        info["retime"] = retime_plan(plan, a, out)
        info["retime_s"] = round(time.time() - t_rt, 1)
    cache = res.get("cache") or {}
    log(f"plan {plan.name}: {len(res.get('weld_ids') or [])} welds, {info['wall_s']} s planning, cache "
        f"{cache.get('hits', '?')} hit(s) / {cache.get('misses', '?')} miss(es); exporting and checking it")
    code, report_path, info["export_s"] = U.export_and_check(plan, export_cmd, "{jam_plan}")
    if code == 0 and cache.get("misses") and CC.auto_push_enabled():
        # the misses it filled are new components: saved to the catalogue (opt-in; never fails the job)
        info["catalogue_push"] = auto_push(cli, a.cell_id)
    finish(code, status or "ok")
    U.merge_report(report_path, "jam", info)
    return code


if __name__ == "__main__":
    sys.exit(main())
