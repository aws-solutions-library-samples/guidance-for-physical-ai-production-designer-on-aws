#!/usr/bin/env python3
"""Remote ("burst") planning for the planner router: one umarell_u3 job on an ephemeral EC2 instance.

A small workspace (t4g.large: 2 vCPUs) plans a 4-robot cell with U3 in 2 OMPL workers, ~4 x as long as an 8-vCPU one.
The router (server.py) can instead run the very same job on a big arm64 instance in the same account and region:

1. the job's inputs go to S3 (s3://<bucket>/jobs/<workspace>/<job key>/): inputs.tgz holds the job's cell view
   (symlinks, as the router built it), the FactoryOS cell folder with its symlinks resolved (its .view/ links into the
   catalogue become real files; no plans/, logs/, recordings/), whatever else the view links to outside the installed
   planners, the task program and the router's umarell_u3.py / u3_retime.py, and what the cell's planner model
   (cell.yaml, config/robots.yaml, obstacles.yaml, scene.yaml, the task, the cell's URDFs) and the command line name by
   absolute path outside all that (a converted cell's dressed robot URDFs in the catalogue with their meshes, the
   tool's URDF and meshes beside its planner.yaml; add_references, size-capped, logged), all at their absolute paths
   on the workspace; job.json says what to install (the cell planner / umarell / umarell-base archives pinned in
   deploy/cell-planner.txt, umarell.txt, umarell-base.txt, from the CI archive, checked by sha256; the two venvs from
   requirements-*.txt, cached in the bucket's cache/ prefix) and the command line: exactly the local job's
   (umarell_u3.py ... -- btexport ...), its working directory and environment;
2. the router launches an instance from the planner-burst launch template (infra/lib/planner-burst.ts: Ubuntu 24.04
   arm64, the planner instance profile, no ingress, shutdown = terminate), trying the instance types in order in
   every subnet (InsufficientInstanceCapacity), on-demand or spot (config `market`);
3. on the instance burst_instance.py installs everything at the workspace's paths (as provision-cell-planner.sh /
   provision.sh do), runs the command with FACTORY_OS_U3_WORKERS = its CPUs (capped by its memory), uploads progress.json, the logs and at
   the end result.tgz (the job dir: plan, report, u3/, btexport.log) and status.json {exit_code}, and powers off
   (= terminates);
4. the router polls status.json (writing a heartbeat), downloads the job dir and publishes the plan exactly as for a
   local job (server.py _finish: exit codes, "unverified").

The instance is never rebooted (user data masks reboot.target: the account's patching association reboots instances
after installing updates, which killed U3 mid-plan) and runs its job once (a second start reports stage "restart").
The router falls back to planning locally when the instance's progress.json stops changing for progress_timeout_s.
The instance ends itself: when it is done, when the router's heartbeat is older than heartbeat_timeout_s (the
router restarted, the workspace stopped), past its lifetime cap (user data's watchdog), and the stack's reaper
Lambda terminates any planner-burst instance older than its max hours. The router also terminates it on cancel /
timeout. A launch that fails (no rights yet: the CDK change not deployed; no capacity; no config parameter) makes
the router plan locally instead, with the capacity-scaled timeout, and plan_status says so.

Config: the SSM parameter /factoryos/planner-burst/config (JSON, written by the CDK stack: region, bucket,
launch_template_id, subnets, instance_types, market, max_concurrent, artifact_bucket, artifact_region), overridable by
FACTORYOS_BURST_* env vars (FACTORYOS_BURST_CONFIG: a JSON file or JSON text instead of the parameter). A config
without artifact_bucket / artifact_region (the archive of the deploy/*.txt pins) takes the internal build's defaults
(deploy/internal.env), and remote planning is unavailable without an archive. The workspace's name (the S3 prefix,
the instance tag its IAM rights are scoped by) is FACTORY_OS_WORKSPACE of /etc/factoryos/workspace.env. Without
burst_instance.py next to this file remote planning is not installed: load_config says so before anything else.

Dry run (prints what it would launch: config, the inputs it would pack, the S3 keys, the run-instances calls in
fallback order, the user data; with --check-permissions also `aws ec2 run-instances --dry-run`):

    python3 -I burst.py plan --job-dir /var/lib/factoryos/cell-planner/work/jobs/<tree id> [--cell-dir <cell>] \
        [--config <json>] [--check-permissions] [--inputs-out /tmp/inputs.tgz]

Standard library only; AWS through the aws CLI (the workspace has it: provision.sh, user data).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
import tarfile
import time
import urllib.request
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Optional

HERE = Path(__file__).resolve().parent
DEPLOY = HERE.parent
CONFIG_PARAM = "/factoryos/planner-burst/config"
WORKSPACE_ENV = Path("/etc/factoryos/workspace.env")
# The internal build's archive defaults (FACTORYOS_DEFAULT_BUILDS_BUCKET / _REGION), as provision.sh reads them; the
# public export has no such file.
INTERNAL_ENV = DEPLOY / "internal.env"
TAG = "FactoryosPlannerBurst"           # the instance's tag: the workspace's name (its IAM rights are scoped by it)
DEFAULT_TYPES = ["c7g.8xlarge", "c7gd.8xlarge", "m7g.8xlarge", "c6g.8xlarge", "m6g.8xlarge", "c7g.4xlarge"]
ROUTER_FILES = ("umarell_u3.py", "u3_retime.py", "catalogue_cache.py")   # umarell_u3.py imports both
SKIP_IN_CELL = {"agentic_planner/plans", "agentic_planner/logs", "agentic_planner/recordings",
                "agentic_planner/checkpoints", "plans", "logs", "recordings"}
MAX_ENTRY_MB = 1024.0                 # a cell entry larger than this is left out (e.g. a sample-data link)
MAX_REFS_MB = 2048.0                  # all files the cell's model references by absolute path, together
# Never packed as a reference: the instance's own system (a command line's /usr/bin/python3, ...).
SYSTEM_ROOTS = tuple(Path(p) for p in ("/usr", "/bin", "/sbin", "/lib", "/lib64", "/etc", "/proc", "/sys", "/dev",
                                       "/run", "/boot", "/snap", "/var/lib/apt", "/var/cache"))
MODEL_SUFFIXES = (".yaml", ".yml", ".json")
URDF_SUFFIXES = (".urdf", ".xacro")
_ABS_PATH = re.compile(r"""(?:^|(?<=[\s"'=:,\[(]))(/(?:[\w.@+~-]+/)+[\w.@+~-]+)""", re.M)
_URDF_FILE = re.compile(r"""filename\s*=\s*["']([^"']+)["']""")
INSTANCE_SCRIPT = HERE / "burst_instance.py"
SIZES = {"medium": 1, "large": 2, "xlarge": 4, "2xlarge": 8, "4xlarge": 16, "8xlarge": 32, "12xlarge": 48,
         "16xlarge": 64, "24xlarge": 96, "metal": 64}


class BurstUnavailable(Exception):
    """Remote planning can't be used for this job (no config, no rights, no capacity, a setup failure on the
    instance): the router plans it locally instead."""


def vcpus(instance_type: str) -> int:
    """The vCPUs of an instance type by its size (c7g.8xlarge: 32), as EC2 sizes them."""
    return SIZES.get(instance_type.rpartition(".")[2], 0) or 0


@dataclass
class BurstConfig:
    region: str
    bucket: str
    launch_template_id: str
    subnets: list
    workspace: str
    instance_types: list = field(default_factory=lambda: list(DEFAULT_TYPES))
    market: str = "on-demand"          # on-demand | spot | spot,on-demand (spot first, then on-demand)
    max_concurrent: int = 2
    prefix: str = "jobs"
    heartbeat_timeout_s: int = 600
    progress_timeout_s: int = 900      # the instance uploads progress.json every 20 s: none for this long = it is gone
    poll_s: float = 15.0
    keep: bool = False                 # debugging: the instance stays up after the job (the watchdog still ends it)
    artifact_bucket: str = ""          # the archive of the deploy/*.txt keys, as provision.sh's (load_config: required)
    artifact_region: str = "us-east-1"

    @property
    def markets(self) -> list:
        m = [x.strip() for x in str(self.market or "on-demand").split(",") if x.strip()]
        return [x for x in m if x in ("on-demand", "spot")] or ["on-demand"]


def read_env_file(path: Path) -> dict:
    out = {}
    try:
        for line in path.read_text().splitlines():
            k, sep, v = line.strip().partition("=")
            if sep and not k.startswith("#"):
                out[k.strip()] = v.strip().strip('"').strip("'")
    except OSError:
        pass
    return out


def imds_region(timeout: float = 1.0) -> Optional[str]:
    try:
        req = urllib.request.Request("http://169.254.169.254/latest/api/token", method="PUT",
                                     headers={"X-aws-ec2-metadata-token-ttl-seconds": "60"})
        token = urllib.request.urlopen(req, timeout=timeout).read().decode()
        req = urllib.request.Request("http://169.254.169.254/latest/meta-data/placement/region",
                                     headers={"X-aws-ec2-metadata-token": token})
        return urllib.request.urlopen(req, timeout=timeout).read().decode().strip() or None
    except (OSError, ValueError):
        return None


Runner = Callable[[list, Optional[bytes], float], "subprocess.CompletedProcess"]


def _run(cmd: list, data: Optional[bytes], timeout: float) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, input=data, capture_output=True, timeout=timeout)


class Aws:
    """The aws CLI calls remote planning makes (tests replace `runner`)."""

    def __init__(self, region: str, runner: Optional[Runner] = None, exe: str = "aws"):
        self.region, self.runner, self.exe = region, runner or _run, exe

    def cli(self, *args, data: Optional[bytes] = None, timeout: float = 120.0, check: bool = True) -> str:
        cmd = [self.exe, *map(str, args), "--region", self.region]
        try:
            r = self.runner(cmd, data, timeout)
        except (OSError, subprocess.SubprocessError) as e:
            raise BurstUnavailable(f"aws {' '.join(map(str, args[:2]))}: {e}") from e
        out = r.stdout.decode(errors="replace") if isinstance(r.stdout, bytes) else (r.stdout or "")
        if check and r.returncode != 0:
            err = r.stderr.decode(errors="replace") if isinstance(r.stderr, bytes) else (r.stderr or "")
            raise BurstUnavailable(f"aws {' '.join(map(str, args[:2]))}: {err.strip()[-400:] or f'exit {r.returncode}'}")
        return out

    def get_parameter(self, name: str) -> str:
        return self.cli("ssm", "get-parameter", "--name", name, "--query", "Parameter.Value", "--output", "text",
                        timeout=20).strip()

    def put_bytes(self, url: str, body: bytes) -> None:
        self.cli("s3", "cp", "-", url, "--only-show-errors", data=body)

    def put_file(self, path: Path, url: str) -> None:
        self.cli("s3", "cp", str(path), url, "--only-show-errors", timeout=900)

    def get_bytes(self, url: str) -> Optional[bytes]:
        """The object, None if it does not exist (or can't be read)."""
        cmd = [self.exe, "s3", "cp", url, "-", "--region", self.region]
        try:
            r = self.runner(cmd, None, 120)
        except (OSError, subprocess.SubprocessError):
            return None
        if r.returncode != 0:
            return None
        return r.stdout if isinstance(r.stdout, bytes) else (r.stdout or "").encode()

    def get_file(self, url: str, path: Path) -> None:
        self.cli("s3", "cp", url, str(path), "--only-show-errors", timeout=900)

    def run_instance(self, args: list) -> str:
        return self.cli(*args, timeout=60).strip()

    def instance_state(self, iid: str) -> Optional[str]:
        try:
            return self.cli("ec2", "describe-instances", "--instance-ids", iid, "--query",
                            "Reservations[0].Instances[0].State.Name", "--output", "text", timeout=30).strip() or None
        except BurstUnavailable:
            return None

    def terminate(self, iid: str) -> None:
        try:
            self.cli("ec2", "terminate-instances", "--instance-ids", iid, timeout=30)
        except BurstUnavailable as e:
            print(f"[burst] terminating {iid}: {e}", file=sys.stderr, flush=True)


def load_config(env: Optional[dict] = None, aws_factory: Callable[[str], Aws] = Aws,
                workspace_env: Path = WORKSPACE_ENV, internal_env: Path = INTERNAL_ENV) -> BurstConfig:
    """The burst config: FACTORYOS_BURST_CONFIG (JSON text or a file) or the SSM parameter, then FACTORYOS_BURST_*
    overrides; the archive the config does not name: internal_env's defaults. Raises BurstUnavailable with the reason
    when there is none (first of all when burst_instance.py, which the instance runs, is not installed)."""
    if not INSTANCE_SCRIPT.is_file():
        raise BurstUnavailable(f"not installed on this host: no {INSTANCE_SCRIPT.name}")
    env = os.environ if env is None else env
    ws_env = read_env_file(workspace_env)
    defaults = read_env_file(internal_env)
    raw = (env.get("FACTORYOS_BURST_CONFIG") or "").strip()
    if not raw and not (env.get("FACTORYOS_BURST_WORKSPACE") or ws_env.get("FACTORY_OS_WORKSPACE")):
        # before any network call: not a workspace (a CI runner, a laptop)
        raise BurstUnavailable("no workspace name (FACTORY_OS_WORKSPACE in /etc/factoryos/workspace.env): remote "
                               "planning is for personal workspaces")
    region = env.get("FACTORYOS_BURST_REGION") or ws_env.get("REGION") or (imds_region() if not raw else "") or ""
    if raw and not raw.startswith("{"):
        try:
            raw = Path(raw).read_text()
        except OSError as e:
            raise BurstUnavailable(f"FACTORYOS_BURST_CONFIG {raw}: {e}") from e
    if not raw:
        if not region:
            raise BurstUnavailable("no AWS region (not an EC2 workspace?)")
        try:
            raw = aws_factory(region).get_parameter(env.get("FACTORYOS_BURST_PARAM") or CONFIG_PARAM)
        except BurstUnavailable as e:
            raise BurstUnavailable(f"no planner-burst config ({CONFIG_PARAM}; the infra stack not deployed yet?): {e}")
    try:
        c = json.loads(raw)
    except ValueError as e:
        raise BurstUnavailable(f"the planner-burst config is not JSON: {e}") from e
    workspace = env.get("FACTORYOS_BURST_WORKSPACE") or ws_env.get("FACTORY_OS_WORKSPACE") or c.get("workspace") or ""
    if not workspace:
        raise BurstUnavailable("no workspace name (FACTORY_OS_WORKSPACE in /etc/factoryos/workspace.env): remote "
                               "planning is for personal workspaces")
    types = env.get("FACTORYOS_BURST_INSTANCE_TYPES")
    subnets = c.get("subnets") or []
    if isinstance(subnets, str):
        subnets = [s for s in subnets.split(",") if s]
    cfg = BurstConfig(
        region=c.get("region") or region, bucket=c.get("bucket") or "", launch_template_id=c.get("launch_template_id") or "",
        subnets=subnets, workspace=workspace,
        instance_types=(types.split() if types else c.get("instance_types") or list(DEFAULT_TYPES)),
        market=env.get("FACTORYOS_BURST_MARKET") or c.get("market") or "on-demand",
        max_concurrent=int(env.get("FACTORYOS_BURST_MAX") or c.get("max_concurrent") or 2),
        prefix=c.get("prefix") or "jobs",
        heartbeat_timeout_s=int(c.get("heartbeat_timeout_s") or 600),
        progress_timeout_s=int(c.get("progress_timeout_s") or 900),
        artifact_bucket=c.get("artifact_bucket") or defaults.get("FACTORYOS_DEFAULT_BUILDS_BUCKET") or "",
        artifact_region=c.get("artifact_region") or defaults.get("FACTORYOS_DEFAULT_BUILDS_REGION") or "us-east-1",
        keep=(env.get("FACTORYOS_BURST_KEEP") == "1"))
    missing = [k for k in ("region", "bucket", "launch_template_id", "subnets", "artifact_bucket") if not getattr(cfg, k)]
    if missing:
        raise BurstUnavailable(f"the planner-burst config lacks {', '.join(missing)}")
    if not re.fullmatch(r"[a-z0-9][a-z0-9_-]{0,62}", cfg.workspace):
        raise BurstUnavailable(f"workspace name {cfg.workspace!r} is not usable as a tag / S3 prefix")
    return cfg


# ───────────────────────────────────────── the pins ─────────────────────────────────────────
def read_pin(path: Path) -> Optional[dict]:
    """`<commit> <sha256> <key>` of a deploy/*.txt pin; None if the file has none."""
    try:
        for line in path.read_text().splitlines():
            if line.strip() and not line.lstrip().startswith("#"):
                commit, sha, key = (line.split() + ["", "", ""])[:3]
                return {"commit": commit, "sha256": sha, "key": key} if key else None
    except OSError:
        pass
    return None


def installed_sha(marker: Path) -> Optional[str]:
    try:
        return marker.read_text().split()[0]
    except (OSError, IndexError):
        return None


def pins(cell_planner: Path, python: Path, umarell: Path, umarell_python: Path, deploy: Path = DEPLOY) -> dict:
    """What the instance installs, at the workspace's paths: the archives the release pins, which must be the ones
    installed here (else the remote run would not be this host's run: BurstUnavailable)."""
    out = {}
    cp, um, base = read_pin(deploy / "cell-planner.txt"), read_pin(deploy / "umarell.txt"), read_pin(deploy / "umarell-base.txt")
    if not cp or not um:
        raise BurstUnavailable("the release pins no cell planner / umarell archive (deploy/*.txt)")
    for name, pin, marker in (("cell planner", cp, cell_planner / ".cell-planner.sha256"),
                              ("umarell", um, umarell / ".umarell.sha256")):
        have = installed_sha(marker)
        if have != pin["sha256"]:
            raise BurstUnavailable(f"the installed {name} ({have or 'unknown'}) is not the release's pin "
                                   f"({pin['sha256'][:12]}): the instance could not install the same one")
    base_dir = umarell.with_name(umarell.name + "-base")
    if base and (base_dir / "agents").is_dir():
        if installed_sha(base_dir / ".umarell.sha256") != base["sha256"]:
            raise BurstUnavailable("the installed umarell-base is not the release's pin")
        out["umarell_base"] = {**base, "dir": str(base_dir)}
    for req in ("requirements-cell-planner.txt", "requirements-planner.txt"):
        if not (deploy / req).is_file():
            raise BurstUnavailable(f"deploy/{req} is missing")
    generated = cell_planner / "env" / "assets" / "generated"
    out["cell_planner"] = {**cp, "dir": str(cell_planner), "venv": str(python.parent.parent),
                           "generated": os.readlink(generated) if generated.is_symlink() else None,
                           "requirements": (deploy / "requirements-cell-planner.txt").read_text(),
                           "pin_line": (cell_planner / ".cell-planner.txt").read_text().strip()
                           if (cell_planner / ".cell-planner.txt").is_file() else ""}
    out["umarell"] = {**um, "dir": str(umarell), "venv": str(umarell_python.parent.parent),
                      "requirements": (deploy / "requirements-planner.txt").read_text()}
    return out


# ───────────────────────────────────────── the inputs ─────────────────────────────────────────
def _arc(p: Path) -> str:
    return str(p).lstrip("/")


def _under(p: Path, roots: list) -> bool:
    return any(p == r or r in p.parents for r in roots)


def _tree_files(src: Path, rel: Path = Path(), seen: Optional[set] = None):
    """(logical path relative to src, real file) under src, symlinks followed (cycles skipped)."""
    seen = set() if seen is None else seen
    real = src.resolve()
    if real in seen:
        return
    seen.add(real)
    try:
        entries = sorted(src.iterdir())
    except OSError:
        return
    for e in entries:
        if e.is_dir():
            yield from _tree_files(e, rel / e.name, seen)
        elif e.is_file():
            yield rel / e.name, e


def _size_mb(files: list) -> float:
    total = 0
    for _, f in files:
        try:
            total += f.stat().st_size
        except OSError:
            pass
    return total / 1e6


@dataclass
class InputPlan:
    """What inputs.tgz holds: directories laid out with their symlinks kept, trees and files resolved."""
    links: list = field(default_factory=list)     # dirs packed as they are (the job's view: symlinks)
    trees: list = field(default_factory=list)     # (dir, files [(rel, real)]) packed resolved
    files: list = field(default_factory=list)     # (path, real) packed resolved
    skipped: list = field(default_factory=list)   # (path, why)
    # what the cell's planner model / the command line reference by absolute path outside the rest (packed at that
    # path: the instance extracts at /): [{"path", "from", "files": [(path, real)], "mb"}]
    referenced: list = field(default_factory=list)
    missing: list = field(default_factory=list)   # (path, referenced by): not on this host either

    def ref_files(self) -> list:
        return [x for r in self.referenced for x in r["files"]]

    def required(self) -> list:
        """The absolute paths the instance must have after extracting the inputs (burst_instance.py checks them)."""
        return sorted({str(p) for p, _ in self.ref_files()})

    def summary(self) -> dict:
        refs = self.ref_files()
        return {"links": [str(p) for p in self.links],
                "trees": [{"path": str(d), "files": len(fs), "mb": round(_size_mb(fs), 1)} for d, fs in self.trees],
                "files": [str(p) for p, _ in self.files], "skipped": [f"{p}: {w}" for p, w in self.skipped],
                "referenced": [{"path": str(r["path"]), "from": r["from"], "files": len(r["files"]),
                                "mb": round(r["mb"], 1)} for r in self.referenced],
                "missing": [f"{p} (referenced by {w})" for p, w in self.missing],
                "mb": round(sum(_size_mb(fs) for _, fs in self.trees)
                            + _size_mb([(None, r) for _, r in self.files + refs]), 1)}

    def log_lines(self) -> list:
        """What the references added (and what is still missing), for the router's log."""
        out = [f"+ {r['path']} ({len(r['files'])} file(s), {r['mb']:.1f} MB; referenced by {r['from']})"
               for r in self.referenced]
        out += [f"skipped {p}: {w}" for p, w in self.skipped if "referenced" in w]
        out += [f"missing {p} (referenced by {w}; not on this host either)" for p, w in self.missing]
        return out


def plan_inputs(view: Optional[Path], cell_dir: Path, installed: list, task_file: Optional[Path] = None,
                router_dir: Path = HERE, max_entry_mb: float = MAX_ENTRY_MB, cmd: Optional[list] = None,
                exclude: Optional[list] = None, max_refs_mb: float = MAX_REFS_MB) -> InputPlan:
    """The job's inputs: its view (symlinks kept), the cell folder (resolved, without its plans / logs; an entry above
    max_entry_mb left out), every other path the view links to outside the installed planners and the cell (resolved),
    the task program, the router's U3 scripts; then what the cell's planner model and the command line reference by
    absolute path outside all that (add_references: e.g. cell.yaml's dressed URDFs in the catalogue with their meshes).
    exclude: paths never packed as a reference (the job dir: the job's outputs)."""
    installed = [Path(p).resolve() for p in installed] + [Path(p) for p in installed]
    ip = InputPlan()
    cell_dir = Path(cell_dir)
    cell_real = cell_dir.resolve()
    skip_real = [cell_real / s for s in SKIP_IN_CELL]
    for entry in sorted(cell_dir.iterdir()):
        if entry.name in SKIP_IN_CELL:
            continue
        if entry.is_dir():
            # (not via a link either: .view/agentic_planner -> ../agentic_planner would pack the plans again)
            files = [(r, f) for r, f in _tree_files(entry)
                     if not any(f"{entry.name}/{r}".startswith(s + "/") for s in SKIP_IN_CELL)
                     and not _under(f.resolve(), skip_real)]
        elif entry.is_file():
            files = [(Path(), entry)]
        else:
            ip.skipped.append((entry, "dangling link"))
            continue
        mb = _size_mb(files)
        if mb > max_entry_mb:
            ip.skipped.append((entry, f"{mb:.0f} MB > {max_entry_mb:.0f} MB"))
            continue
        ip.trees.append((cell_dir / entry.name, files))
    if view is not None and view.exists():
        ip.links.append(view)
        outside = []
        for link in sorted(_links(view)):
            target = Path(os.path.realpath(link))
            raw = Path(os.readlink(link))
            raw = raw if raw.is_absolute() else (link.parent / raw)
            if _under(target, installed) or _under(raw, installed) or _under(raw, [cell_dir]) \
                    or _under(target, [cell_real]) or not target.exists():
                continue
            outside.append(raw)
        for p in sorted(set(outside)):
            if any(_under(p, [q]) for q in outside if q != p):
                continue
            if p.is_dir():
                files = list(_tree_files(p))
                mb = _size_mb(files)
                if mb > max_entry_mb:
                    ip.skipped.append((p, f"{mb:.0f} MB > {max_entry_mb:.0f} MB"))
                    continue
                ip.trees.append((p, files))
            elif p.is_file():
                ip.files.append((p, p))
    if task_file is not None and task_file.is_file() and not _under(task_file, installed + [cell_dir]):
        ip.files.append((task_file, task_file))
    for name in ROUTER_FILES:
        if (router_dir / name).is_file():
            ip.files.append((router_dir / name, router_dir / name))
    add_references(ip, cell_dir, installed, task_file=task_file, cmd=cmd,
                   exclude=[Path(router_dir)] + [Path(p) for p in exclude or []],
                   max_entry_mb=max_entry_mb, max_refs_mb=max_refs_mb)
    return ip


# ─────────────────────────────── the cell's references by absolute path ───────────────────────────────
def model_files(cell_dir: Path, task_file: Optional[Path] = None) -> list:
    """The cell's planner model files that may name other files by absolute path: agentic_planner/'s cell.yaml,
    inputs.json, ..., config/*.yaml (robots.yaml, obstacles.yaml), srdf/, the cell's and its view's scene.yaml /
    cell.yaml, the task program; and every URDF in the cell (its view's robots/, fixtures/: their meshes)."""
    cell_dir = Path(cell_dir)
    ap = cell_dir / "agentic_planner"
    out = []
    for d, suffixes in ((ap, MODEL_SUFFIXES), (ap / "config", MODEL_SUFFIXES), (ap / "srdf", (".srdf",)),
                        (cell_dir, MODEL_SUFFIXES), (cell_dir / ".view", MODEL_SUFFIXES)):
        try:
            out += [p for p in sorted(d.iterdir()) if p.is_file() and p.suffix in suffixes]
        except OSError:
            pass
    if task_file is not None and Path(task_file).is_file():
        out.append(Path(task_file))
    seen = set()
    for top in ("robots", "fixtures", ".view"):
        for _, f in _tree_files(cell_dir / top) if (cell_dir / top).is_dir() else ():
            if f.suffix in URDF_SUFFIXES and f.resolve() not in seen:
                seen.add(f.resolve())
                out.append(f)
    return out


def _cell_rel(path: Path, cell_dir: Path) -> str:
    try:
        return str(path.relative_to(cell_dir))
    except ValueError:
        return str(path)


def absolute_refs(text: str) -> list:
    """The absolute paths a YAML / JSON / URDF text names (in values and in comments: a comment's path that does not
    exist is dropped later, one that does is small and harmless)."""
    return sorted(set(_ABS_PATH.findall(text)))


def urdf_meshes(urdf: Path) -> list:
    """The files a URDF's filename="..." attributes name: relative ones against the URDF's folder (its logical path:
    a linked URDF's meshes sit beside the link), file:// and absolute ones as they are; package:// ones are left
    out (nothing on the workspace resolves them either)."""
    try:
        text = urdf.read_text(errors="replace")
    except OSError:
        return []
    out = []
    for name in sorted(set(_URDF_FILE.findall(text))):
        if name.startswith("package://"):
            continue
        if name.startswith("file://"):
            name = name[len("file://"):]
        p = Path(name)
        out.append(Path(os.path.normpath(p if p.is_absolute() else urdf.parent / p)))
    return out


def _cmd_paths(cmd: list) -> list:
    """The existing files a command line names (`--x /a/b`, `--x=/a/b`, `plan:/a/b`): not its directories (the job
    dir, the work root: the job's outputs) nor interpreters (<venv>/bin/python: the instance installs its own)."""
    out = []
    for arg in cmd or []:
        a = str(arg)
        if "{" in a:
            continue
        for part in (a, a.partition("=")[2], a.partition(":")[2]):
            if part.startswith("/") and Path(part).is_file() and Path(part).parent.name != "bin":
                out.append(Path(part))
                break
    return out


def add_references(ip: InputPlan, cell_dir: Path, installed: list, task_file: Optional[Path] = None,
                   cmd: Optional[list] = None, exclude: Optional[list] = None,
                   max_entry_mb: float = MAX_ENTRY_MB, max_refs_mb: float = MAX_REFS_MB) -> None:
    """Add to ip what the cell's planner model (model_files) and the command line name by absolute path outside what
    ip already packs, at the same absolute path: a cell made by convert_isaac_scene.py names its robots' dressed
    URDFs in the catalogue (/var/lib/factoryos/catalogue/urdf-dressed-robots/<r>/<r>.urdf); its robots.yaml the tool's
    planner model (urdf-tool-collection/<tool>/planner.yaml). Precise, not the whole catalogue: a referenced file;
    a URDF with the meshes it names (and its sibling SRDF); a referenced file's folder's URDFs with theirs (a tool's
    planner.yaml: the tool's URDF and meshes); a referenced folder as a tree. A reference above max_entry_mb, or past
    max_refs_mb together, is left out (ip.skipped); one that does not exist here either goes to ip.missing."""
    cell_dir = Path(cell_dir)
    packed_roots = [Path(p) for p in ip.links] + [Path(d) for d, _ in ip.trees] + [Path(cell_dir), cell_dir.resolve()]
    packed_files = {str(p) for p, _ in ip.files}
    skip_roots = list(installed) + [Path(p) for p in exclude or []] + list(SYSTEM_ROOTS)
    total_mb = 0.0
    done: set = set()

    def outside(p: Path) -> bool:
        return not (_under(p, packed_roots) or _under(p, skip_roots) or str(p) in packed_files or str(p) in done)

    def with_meshes(urdf: Path) -> list:
        files = [urdf]
        for m in urdf_meshes(urdf):
            if outside(m) and m.is_file():
                files.append(m)
        srdf = urdf.with_suffix(".srdf")
        if srdf.is_file() and outside(srdf):
            files.append(srdf)
        return files

    refs: list = []          # (path, referenced by)
    for f in model_files(cell_dir, task_file):
        try:
            text = f.read_text(errors="replace") if f.stat().st_size < 20e6 else ""
        except OSError:
            continue
        names = absolute_refs(text)
        if f.suffix in URDF_SUFFIXES:
            names += [str(m) for m in urdf_meshes(f) if m.is_absolute()]
        refs += [(Path(os.path.normpath(n)), _cell_rel(f, cell_dir)) for n in names]
    refs += [(p, "the command line") for p in _cmd_paths(cmd or [])]
    seen_missing: set = set()
    for p, src in refs:
        if not outside(p):
            continue
        if not p.exists():
            if p.suffix in URDF_SUFFIXES + MODEL_SUFFIXES + (".srdf", ".stl", ".obj", ".dae") \
                    and str(p) not in seen_missing and not _under(p, skip_roots):
                seen_missing.add(str(p))
                ip.missing.append((p, src))
            continue
        if p.is_dir():
            files = [(p / rel, real) for rel, real in _tree_files(p)]
        elif p.suffix in URDF_SUFFIXES:
            files = [(x, x) for x in with_meshes(p)]
        else:
            files = [(p, p)]
            for sib in sorted(p.parent.glob("*")):
                if sib.suffix in URDF_SUFFIXES and sib.is_file() and outside(sib):
                    files += [(x, x) for x in with_meshes(sib) if str(x) not in {str(a) for a, _ in files}]
        files = [(a, r) for a, r in files if str(a) not in done]
        if not files:
            continue
        mb = _size_mb(files)
        if mb > max_entry_mb:
            ip.skipped.append((p, f"referenced by {src}: {mb:.0f} MB > {max_entry_mb:.0f} MB"))
            continue
        if total_mb + mb > max_refs_mb:
            ip.skipped.append((p, f"referenced by {src}: the references would pass {max_refs_mb:.0f} MB"))
            continue
        total_mb += mb
        done.update(str(a) for a, _ in files)
        ip.referenced.append({"path": p, "from": src, "files": files, "mb": mb})


def _links(view: Path):
    for dirpath, dirnames, filenames in os.walk(view):
        for n in dirnames + filenames:
            p = Path(dirpath) / n
            if p.is_symlink():
                yield p


def write_inputs(ip: InputPlan, out: Path) -> None:
    with tarfile.open(out, "w:gz", compresslevel=5) as tar:
        for d in ip.links:
            tar.add(str(d), arcname=_arc(d), recursive=True)        # symlinks stay symlinks
        for d, files in ip.trees:
            for rel, real in files:
                arc = _arc(d / rel) if str(rel) not in ("", ".") else _arc(d)
                tar.add(str(real.resolve()), arcname=arc, recursive=False)
        for p, real in ip.files + ip.ref_files():
            tar.add(str(real.resolve()), arcname=_arc(p), recursive=False)


# ───────────────────────────────────────── launching ─────────────────────────────────────────
def job_key(tree_id: str) -> str:
    return f"{tree_id}-{uuid.uuid4().hex[:6]}"


def s3_prefix(cfg: BurstConfig, key: str) -> str:
    return f"s3://{cfg.bucket}/{cfg.prefix}/{cfg.workspace}/{key}"


def user_data(cfg: BurstConfig, prefix: str, max_minutes: int) -> str:
    """The instance's boot script: no reboots (the account's patching), a lifetime / heartbeat watchdog, the aws CLI,
    then burst_instance.py as a service (systemd; it runs the job once: a second start reports and powers off)."""
    q = shlex.quote
    hb = f"{prefix}/heartbeat"
    keep = "1" if cfg.keep else "0"
    return f"""#!/bin/bash
# factoryos planner burst job {prefix}
set -euxo pipefail
# Never reboot: the account's patching (SSM association AWS-RunPatchBaseline, Operation Install, on every instance minutes
# after it registers) installs updates and reboots -- that killed U3 mid-plan and the job never came back (jobs
# bt_1793445025 / bt_1793450505 / bt_1793451359_body_weld_3). Masking reboot.target makes that reboot fail; poweroff
# (the job's end, the watchdog) still ends the instance. needrestart (apt's hook) lists services, never restarts them.
systemctl mask --runtime reboot.target kexec.target ctrl-alt-del.target || true
mkdir -p /etc/needrestart/conf.d
echo "\\$nrconf{{restart}} = 'l';" > /etc/needrestart/conf.d/99-fos-burst.conf
echo $(( $(date +%s) + {int(max_minutes)} * 60 )) > /etc/fos-burst-deadline
cat > /usr/local/bin/fos-burst-watchdog.sh <<'EOF'
#!/bin/bash
# Ends the instance (shutdown behaviour terminate) past its lifetime cap, or when the router's heartbeat is older
# than {cfg.heartbeat_timeout_s} s (the router restarted, the workspace stopped, the job was cancelled).
deadline=$(cat /etc/fos-burst-deadline)
while :; do
  now=$(date +%s)
  if [ "$now" -ge "$deadline" ]; then echo "fos-burst: lifetime cap reached"; break; fi
  hb=$(aws s3 cp {q(hb)} - --region {q(cfg.region)} 2>/dev/null || true)
  case "$hb" in
    ''|*[!0-9]*) ;;
    *) if [ $(( now - hb )) -gt {int(cfg.heartbeat_timeout_s)} ]; then echo "fos-burst: no heartbeat for $(( now - hb )) s"; break; fi ;;
  esac
  sleep 30
done
systemctl poweroff
EOF
chmod +x /usr/local/bin/fos-burst-watchdog.sh
cat > /etc/systemd/system/fos-burst-watchdog.service <<'EOF'
[Unit]
Description=factoryos planner burst: lifetime cap and router heartbeat
[Service]
ExecStart=/usr/local/bin/fos-burst-watchdog.sh
Restart=always
[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable fos-burst-watchdog.service
systemctl start --no-block fos-burst-watchdog.service
export DEBIAN_FRONTEND=noninteractive
apt-get -o DPkg::Lock::Timeout=600 update -y -qq
apt-get -o DPkg::Lock::Timeout=600 install -y -qq curl unzip >/dev/null
if ! command -v aws >/dev/null 2>&1; then
  curl -fsSL "https://awscli.amazonaws.com/awscli-exe-linux-$(uname -m).zip" -o /tmp/awscliv2.zip
  unzip -q /tmp/awscliv2.zip -d /tmp && /tmp/aws/install && rm -rf /tmp/aws /tmp/awscliv2.zip
fi
aws s3 cp {q(prefix + '/instance.py')} /usr/local/bin/fos-burst.py --region {q(cfg.region)} --only-show-errors
aws s3 cp {q(prefix + '/job.json')} /etc/fos-burst-job.json --region {q(cfg.region)} --only-show-errors
cat > /etc/systemd/system/fos-burst.service <<'EOF'
[Unit]
Description=factoryos planner burst job
After=network-online.target
Wants=network-online.target
[Service]
Type=oneshot
Environment=FOS_BURST_KEEP={keep}
ExecStart=/usr/bin/python3 -I /usr/local/bin/fos-burst.py /etc/fos-burst-job.json
StandardOutput=append:/var/log/fos-burst.log
StandardError=append:/var/log/fos-burst.log
TimeoutStartSec=0
[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable fos-burst.service
systemctl start --no-block fos-burst.service
"""


def run_instances_args(cfg: BurstConfig, itype: str, subnet: str, market: str, user_data_path: Path,
                       tags: dict, dry_run: bool = False) -> list:
    tag_list = ",".join(f"{{Key={k},Value={v}}}" for k, v in tags.items())
    args = ["ec2", "run-instances", "--launch-template", f"LaunchTemplateId={cfg.launch_template_id},Version=$Latest",
            "--instance-type", itype, "--subnet-id", subnet, "--user-data", f"file://{user_data_path}",
            "--instance-initiated-shutdown-behavior", "terminate",
            "--tag-specifications", f"ResourceType=instance,Tags=[{tag_list}]",
            f"ResourceType=volume,Tags=[{tag_list}]",
            "--query", "Instances[0].InstanceId", "--output", "text"]
    if market == "spot":
        args[2:2] = ["--instance-market-options",
                     "MarketType=spot,SpotOptions={SpotInstanceType=one-time,InstanceInterruptionBehavior=terminate}"]
    if dry_run:
        args.append("--dry-run")
    return args


def launch_attempts(cfg: BurstConfig) -> list:
    """(market, instance type, subnet) in the order they are tried."""
    return [(m, t, s) for m in cfg.markets for t in cfg.instance_types for s in cfg.subnets]


def tags_for(cfg: BurstConfig, tree_id: str, key: str) -> dict:
    return {TAG: cfg.workspace, "Name": f"factoryos-planner-burst-{cfg.workspace}-{tree_id}"[:250],
            "FactoryosPlannerJob": key}


def job_spec(cfg: BurstConfig, key: str, tree_id: str, job_dir: Path, cmd: list, cwd: str, env: dict,
             pin_info: dict, extra_dirs: list, cell_dir: Optional[Path] = None,
             required: Optional[list] = None) -> dict:
    """job.json: what burst_instance.py installs and runs; `required`: the files the inputs must hold (the cell's
    references by absolute path, InputPlan.required()), checked after extracting them, with the cell's cell.yaml."""
    return {"version": 1, "key": key, "tree_id": tree_id, "job_dir": str(job_dir), "cmd": cmd, "cwd": cwd,
            "env": env, "pins": pin_info, "artifact_bucket": cfg.artifact_bucket, "artifact_region": cfg.artifact_region,
            "bucket": cfg.bucket, "prefix": f"{cfg.prefix}/{cfg.workspace}/{key}", "region": cfg.region,
            "cache_prefix": "cache", "dirs": [str(d) for d in extra_dirs], "keep": cfg.keep,
            "cell_dir": str(cell_dir) if cell_dir else None, "required": list(required or [])}


class Launcher:
    """One remote job's AWS side (server.py drives it: upload, launch, poll, fetch, terminate)."""

    def __init__(self, cfg: BurstConfig, aws: Optional[Aws] = None, tree_id: str = "job", key: Optional[str] = None):
        self.cfg = cfg
        self.aws = aws or Aws(cfg.region)
        self.tree_id = tree_id
        self.key = key or job_key(tree_id)
        self.prefix = s3_prefix(cfg, self.key)
        self.instance_id: Optional[str] = None
        self.instance_type: Optional[str] = None
        self.market: Optional[str] = None
        self.attempts: list = []

    def upload(self, inputs: Path, spec: dict, instance_script: Path = INSTANCE_SCRIPT) -> None:
        self.heartbeat()
        self.aws.put_file(inputs, f"{self.prefix}/inputs.tgz")
        self.aws.put_bytes(f"{self.prefix}/job.json", json.dumps(spec, indent=1).encode())
        self.aws.put_bytes(f"{self.prefix}/instance.py", instance_script.read_bytes())

    def heartbeat(self) -> None:
        try:
            self.aws.put_bytes(f"{self.prefix}/heartbeat", str(int(time.time())).encode())
        except BurstUnavailable as e:
            print(f"[burst] heartbeat: {e}", file=sys.stderr, flush=True)

    def launch(self, work: Path, max_minutes: int) -> str:
        """Run one instance, trying the types / subnets / markets in order; BurstUnavailable if none launched."""
        ud = work / "user-data.sh"
        ud.write_text(user_data(self.cfg, self.prefix, max_minutes))
        tags = tags_for(self.cfg, self.tree_id, self.key)
        last = ""
        for market, itype, subnet in launch_attempts(self.cfg):
            try:
                iid = self.aws.run_instance(run_instances_args(self.cfg, itype, subnet, market, ud, tags))
            except BurstUnavailable as e:
                last = str(e)
                self.attempts.append(f"{market} {itype} in {subnet}: {last[-160:]}")
                if not re.search(r"Insufficient|capacity|Unsupported|not supported|MaxSpot|SpotMax|"
                                 r"InstanceLimitExceeded|VcpuLimitExceeded|Unavailable", last, re.I):
                    break        # e.g. UnauthorizedOperation: no other type / subnet would launch
                continue
            if iid.startswith("i-"):
                self.instance_id, self.instance_type, self.market = iid, itype, market
                return iid
            last = f"run-instances returned {iid!r}"
        raise BurstUnavailable(f"no instance launched ({len(self.attempts)} attempt(s)): {last[-300:]}")

    def status(self) -> Optional[dict]:
        body = self.aws.get_bytes(f"{self.prefix}/status.json")
        if not body:
            return None
        try:
            return json.loads(body)
        except ValueError:
            return None

    def progress(self) -> dict:
        body = self.aws.get_bytes(f"{self.prefix}/progress.json")
        try:
            return json.loads(body) if body else {}
        except ValueError:
            return {}

    def fetch_log(self, dest: Path) -> None:
        body = self.aws.get_bytes(f"{self.prefix}/btexport.log")
        if body:
            tmp = dest.with_name(f".{dest.name}.tmp")
            tmp.write_bytes(body)
            os.replace(tmp, dest)

    def instance_log(self) -> str:
        body = self.aws.get_bytes(f"{self.prefix}/instance.log")
        return body.decode(errors="replace") if body else ""

    def fetch_result(self, job_dir: Path) -> None:
        tmp = job_dir.parent / f".{job_dir.name}.result.tgz"
        self.aws.get_file(f"{self.prefix}/result.tgz", tmp)
        try:
            extract_result(tmp, job_dir)
        finally:
            tmp.unlink(missing_ok=True)

    def state(self) -> Optional[str]:
        return self.aws.instance_state(self.instance_id) if self.instance_id else None

    def terminate(self) -> None:
        if self.instance_id and not self.cfg.keep:
            self.aws.terminate(self.instance_id)


def extract_result(tgz: Path, job_dir: Path) -> None:
    """The instance's job dir (members relative to it) into job_dir; nothing outside it, no links."""
    job_dir.mkdir(parents=True, exist_ok=True)
    root = job_dir.resolve()
    with tarfile.open(tgz, "r:gz") as tar:
        for m in tar.getmembers():
            dest = (job_dir / m.name).resolve()
            if not (dest == root or root in dest.parents) or not (m.isfile() or m.isdir()):
                continue
            if m.isdir():
                dest.mkdir(parents=True, exist_ok=True)
                continue
            dest.parent.mkdir(parents=True, exist_ok=True)
            src = tar.extractfile(m)
            if src is not None:
                with open(dest, "wb") as out:
                    out.write(src.read())


# ───────────────────────────────────────── dry run ─────────────────────────────────────────
def describe_plan(cfg: BurstConfig, tree_id: str, ip: Optional[InputPlan], max_minutes: int, cmd: list) -> dict:
    key = f"{tree_id}-<random>"
    prefix = s3_prefix(cfg, key)
    tags = tags_for(cfg, tree_id, key)
    return {"config": asdict(cfg), "s3_prefix": prefix,
            "uploads": [f"{prefix}/{n}" for n in ("heartbeat", "inputs.tgz", "job.json", "instance.py")],
            "results": [f"{prefix}/{n}" for n in ("progress.json", "btexport.log", "instance.log", "result.tgz",
                                                  "status.json")],
            "inputs": ip.summary() if ip else None,
            "launch_attempts": [" ".join(["aws", *run_instances_args(cfg, t, s, m, Path("user-data.sh"), tags),
                                          "--region", cfg.region]) for m, t, s in launch_attempts(cfg)],
            "instance_lifetime_cap_min": max_minutes, "command": cmd,
            "user_data": user_data(cfg, prefix, max_minutes)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="what", required=True)
    p = sub.add_parser("plan", help="print what a remote job would launch (nothing is launched or uploaded)")
    p.add_argument("--job-dir", help="a router job dir (its cmd.json: the command, the view)")
    p.add_argument("--cell-dir", help="the job's FactoryOS cell folder (default: the command's --cell-dir)")
    p.add_argument("--config", help="burst config JSON (text or file) instead of the SSM parameter")
    p.add_argument("--workspace", help="the workspace name (default: /etc/factoryos/workspace.env)")
    p.add_argument("--max-minutes", type=int, default=90)
    p.add_argument("--inputs-out", help="also write the job's inputs.tgz there (to inspect it; nothing is uploaded)")
    p.add_argument("--check-permissions", action="store_true",
                   help="also `aws ec2 run-instances --dry-run` with the first type / subnet (launches nothing)")
    a = ap.parse_args(argv)
    env = dict(os.environ)
    if a.config:
        env["FACTORYOS_BURST_CONFIG"] = a.config
    if a.workspace:
        env["FACTORYOS_BURST_WORKSPACE"] = a.workspace
    try:
        cfg = load_config(env)
    except BurstUnavailable as e:
        print(f"[burst] not available: {e}", file=sys.stderr)
        return 1
    cmd, ip, tree_id = [], None, "bt_0_cell"
    if a.job_dir:
        jd = Path(a.job_dir)
        info = json.loads((jd / "cmd.json").read_text())
        cmd, tree_id = info.get("cmd") or [], jd.name
        cell = Path(a.cell_dir) if a.cell_dir else (Path(cmd[cmd.index("--cell-dir") + 1]) if "--cell-dir" in cmd else None)
        installed = [x for x in (cmd[cmd.index(o) + 1] for o in ("--cell-planner", "--umarell") if o in cmd)]
        installed += [str(Path(x).with_name(Path(x).name + "-base")) for x in installed[1:2]]
        # the venvs: umarell's (--python), the cell planner's (the btexport line's interpreter)
        installed += [str(Path(cmd[cmd.index(o) + 1]).parent.parent) for o in ("--python", "--")
                      if o in cmd[:-1]]
        view = Path(info["UMARELL_REPO"]) if info.get("UMARELL_REPO") else None   # the job's view (gone once it ended)
        task = Path(cmd[cmd.index("--task-file") + 1]) if "--task-file" in cmd[:-1] else None
        if cell:
            ip = plan_inputs(view if view and view.exists() else None, cell, installed, task, cmd=cmd,
                             exclude=[jd])
            for line in ip.log_lines():
                print(f"[burst] inputs: {line}", file=sys.stderr)
            if a.inputs_out:
                write_inputs(ip, Path(a.inputs_out))
                print(f"[burst] inputs written to {a.inputs_out} ({Path(a.inputs_out).stat().st_size / 1e6:.1f} MB; "
                      "nothing uploaded)", file=sys.stderr)
    out = describe_plan(cfg, tree_id, ip, a.max_minutes, cmd)
    print(json.dumps({k: v for k, v in out.items() if k != "user_data"}, indent=1))
    print("----- user data -----\n" + out["user_data"])
    if a.check_permissions:
        m, t, s = launch_attempts(cfg)[0]
        aws = Aws(cfg.region)
        ud = Path("/tmp") / f"fos-burst-ud-{os.getpid()}.sh"
        ud.write_text(out["user_data"])
        try:
            aws.cli(*run_instances_args(cfg, t, s, m, ud, tags_for(cfg, tree_id, "dry-run"), dry_run=True))
        except BurstUnavailable as e:
            ok = "DryRunOperation" in str(e)
            print(f"[burst] run-instances --dry-run: {'allowed' if ok else 'REFUSED'}: {str(e)[-300:]}")
            return 0 if ok else 2
        finally:
            ud.unlink(missing_ok=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
