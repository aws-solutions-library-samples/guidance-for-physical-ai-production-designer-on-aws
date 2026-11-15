#!/usr/bin/env python3
"""The planners' shared caches in the workspaces' catalogue: s3://<data bucket>/catalogue/<planner cache>/*.tgz.

A planner whose results are keyed by content (umarell_u3's task cache, umarell_fast/plan_cache.py; umarell_jam's
component cache) shares them between workspaces as bundles under its own prefix:

* catalogue/u3-plan-cache/                 umarell_u3: `python -m umarell_fast.plan_cache --root C export --cell X
                                           --out F` / `import F`; named <cell>__<cell planner commit[:7]>.tgz
* catalogue/umarell-jam-cache/76ea13a/     umarell_jam: `python -m umarell_jam --root R bundle --out F --cells X` /
                                           `--root R import F` (MANIFEST.json kind "umarell-jam-components"); named
                                           <cell>__<planner[:8]>-<impl[:8]>.tgz (its bundle command picks it)

pull (before planning, every workspace: the workspace roles read catalogue/*): ``aws s3 sync`` of the prefix's *.tgz
into a mirror folder, then the planner's importer on every bundle new or changed since the last pull; a bundle counts
as imported only when the importer exited 0 (a bad one is tried again next time). Never raises.

push (save; a catalogue admin's workspace only): the bundle is first imported into an empty temporary cache with the
planner's own importer (a bundle it can't read is never uploaded), then ``aws s3 cp`` to <prefix><name>.tgz with
``--metadata sha256=<hex>``: never a sync, never a delete. The same name overwrites (the bucket's versioning keeps the
old one 30 days), which bounds the bundles to one per cell and planner version. Refused before any AWS call when this
host is no workspace on S3 storage (no /etc/factoryos/workspace.env: e.g. a planner-burst instance;
FACTORY_OS_STORAGE != s3) or its user is no catalogue admin (FACTORY_OS_CATALOGUE_ADMINS empty or none); an
AccessDenied from S3 is refused with what it means.

Triggers: publish_cache.py (an admin's command on the workspace), and after a successful job when the router has
FACTORY_OS_CATALOGUE_CACHE_PUSH=1 (auto_push_enabled; off by default; a push failure never fails the job).
Standard library only.
"""
from __future__ import annotations

import fcntl
import hashlib
import json
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Callable, Optional

WORKSPACE_ENV = Path("/etc/factoryos/workspace.env")
U3_PREFIX = "catalogue/u3-plan-cache/"
JAM_PREFIX = "catalogue/umarell-jam-cache/"         # + <umarell base>/
AUTO_PUSH_ENV = "FACTORY_OS_CATALOGUE_CACHE_PUSH"
NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


class PushRefused(Exception):
    """A push this host must not (or cannot) make: nothing was uploaded."""


def read_env(env_file: Optional[Path] = None) -> Optional[dict]:
    """/etc/factoryos/workspace.env as a dict; None when there is none (not a workspace)."""
    try:
        text = (env_file or WORKSPACE_ENV).read_text()
    except OSError:
        return None
    return {k.strip(): v.strip() for k, v in (ln.split("=", 1) for ln in text.splitlines()
                                              if "=" in ln and not ln.lstrip().startswith("#"))}


def catalogue_uri(prefix: str, override_var: str, env_file: Optional[Path] = None) -> Optional[str]:
    """A shared prefix in the workspaces' data bucket: ``$<override_var>`` (an s3:// prefix; ``0`` turns it off),
    else ``s3://<FACTORY_OS_S3_BUCKET>/<prefix>`` of a workspace on S3 storage (its workspace.env)."""
    over = os.environ.get(override_var, "").strip()
    if over == "0":
        return None
    if over:
        return over.rstrip("/") + "/"
    kv = read_env(env_file)
    if not kv:
        return None
    bucket = kv.get("FACTORY_OS_S3_BUCKET", "")
    if kv.get("FACTORY_OS_STORAGE", "") != "s3" or not bucket:
        return None
    return f"s3://{bucket}/{prefix}"


def bucket_region(region: Optional[str] = None, env_file: Optional[Path] = None) -> Optional[str]:
    """The data bucket's own region first (workspace.env): an AWS_REGION in the environment may name another one."""
    if not region:
        region = (read_env(env_file) or {}).get("FACTORY_OS_S3_REGION")
    return region or os.environ.get("FACTORY_OS_S3_REGION") or os.environ.get("AWS_REGION")


def aws_cli() -> Optional[str]:
    aws = shutil.which("aws") or "/usr/local/bin/aws"
    return aws if Path(aws).exists() else None


def auto_push_enabled() -> bool:
    """The router's opt-in save after a successful job (FACTORY_OS_CATALOGUE_CACHE_PUSH=1)."""
    return os.environ.get(AUTO_PUSH_ENV, "0").strip() == "1"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------- pull
def pull(uri: str, mirror: Path, importer: Callable[[Path], list], env: Optional[dict] = None,
         cwd: Optional[Path] = None, region: Optional[str] = None, timeout_s: float = 90.0,
         import_lock: Optional[Path] = None, aws: Optional[str] = None, env_file: Optional[Path] = None) -> str:
    """``aws s3 sync`` of ``uri``'s *.tgz into ``mirror``, then ``importer(bundle)`` (an argv, run in ``cwd`` with
    ``env``) on every bundle new or changed since the last pull, holding ``import_lock`` (exclusive) while importing
    when given. One pull at a time per mirror (a lock). Never raises: returns what happened, for the log."""
    aws = aws or aws_cli()
    if not aws:
        return "no aws CLI"
    mirror.mkdir(parents=True, exist_ok=True)
    region = bucket_region(region, env_file)
    with open(mirror / ".lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        cmd = [aws, "s3", "sync", uri, str(mirror), "--exclude", "*", "--include", "*.tgz", "--only-show-errors"]
        if region:
            cmd += ["--region", region]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return f"sync from {uri} failed ({exc})"
        if r.returncode != 0:
            return f"sync from {uri} failed: {(r.stderr or r.stdout).strip()[-200:]}"
        done_f = mirror / ".imported.json"
        try:
            done = json.loads(done_f.read_text())
        except (OSError, ValueError):
            done = {}
        bundles = sorted(mirror.glob("*.tgz"))
        sigs = {b.name: sha256_file(b) for b in bundles}      # content: a re-downloaded same file is not new
        todo = [b for b in bundles if done.get(b.name) != sigs[b.name]]
        imported, failed = [], []
        if todo:
            held = open(import_lock, "w") if import_lock else None
            try:
                if held:
                    fcntl.flock(held, fcntl.LOCK_EX)
                for b in todo:
                    try:
                        ok = subprocess.run(importer(b), cwd=str(cwd) if cwd else None, env=env, capture_output=True,
                                            text=True, timeout=timeout_s, stdin=subprocess.DEVNULL).returncode == 0
                    except (OSError, subprocess.TimeoutExpired):
                        ok = False
                    if ok:                                    # only a bundle its importer took counts as imported
                        done[b.name] = sigs[b.name]
                        imported.append(b.name)
                    else:
                        failed.append(b.name)
            finally:
                if held:
                    held.close()
        tmp = done_f.with_suffix(".tmp")
        tmp.write_text(json.dumps(done, indent=1))
        os.replace(tmp, done_f)
    return (f"{len(bundles)} bundle(s) at {uri}; imported {', '.join(imported) or 'none new'}"
            + (f"; FAILED {', '.join(failed)}" if failed else ""))


# ---------------------------------------------------------------- push
def push_gate(env_file: Optional[Path] = None) -> dict:
    """The workspace.env of a host that may save to the catalogue, else PushRefused (before any AWS call)."""
    kv = read_env(env_file)
    if kv is None:
        raise PushRefused(f"not a workspace ({env_file or WORKSPACE_ENV} missing, e.g. a planner-burst instance): "
                          "save the cache from the workspace (publish_cache.py)")
    if kv.get("FACTORY_OS_STORAGE") != "s3" or not kv.get("FACTORY_OS_S3_BUCKET"):
        raise PushRefused("this workspace is not on S3 storage (FACTORY_OS_STORAGE != s3): there is no shared "
                          "catalogue to save to")
    admins = kv.get("FACTORY_OS_CATALOGUE_ADMINS", "")
    if admins in ("", "none"):
        raise PushRefused("this workspace's user is no catalogue admin (FACTORY_OS_CATALOGUE_ADMINS "
                          f"{admins or 'empty'}): only catalogue admins save caches to the catalogue")
    return kv


def push(bundle: Path, prefix: str, name: str, test_importer: Callable[[Path], list], env: Optional[dict] = None,
         cwd: Optional[Path] = None, env_file: Optional[Path] = None, dry_run: bool = False,
         aws: Optional[str] = None, timeout_s: float = 600.0) -> dict:
    """Save ``bundle`` as s3://<data bucket>/<prefix><name>.tgz: the gate, a test import into an empty temporary
    cache (``test_importer(tmp_root)``: an argv, run in ``cwd`` with ``env``), then ``aws s3 cp`` with
    ``--metadata sha256=<hex>``. Returns {uri, sha256, bytes, dry_run}; PushRefused when it must not upload,
    RuntimeError when the upload failed."""
    kv = push_gate(env_file)
    if name.endswith(".tgz"):
        name = name[:-4]
    if not NAME.match(name) or not prefix.startswith("catalogue/") or ".." in prefix:
        raise PushRefused(f"not a catalogue cache key: {prefix}{name}.tgz")
    bundle = Path(bundle)
    if not bundle.is_file() or bundle.stat().st_size == 0:
        raise PushRefused(f"no bundle {bundle}")
    with tempfile.TemporaryDirectory(prefix="cache-test-import-") as tmp:
        try:
            r = subprocess.run(test_importer(Path(tmp)), cwd=str(cwd) if cwd else None, env=env, capture_output=True,
                               text=True, timeout=timeout_s, stdin=subprocess.DEVNULL)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise PushRefused(f"the test import of {bundle.name} failed ({exc}): not uploaded") from None
        if r.returncode != 0:
            raise PushRefused(f"the test import of {bundle.name} failed (exit {r.returncode}: "
                              f"{(r.stderr or r.stdout).strip()[-300:]}): not uploaded")
    sha = sha256_file(bundle)
    uri = f"s3://{kv['FACTORY_OS_S3_BUCKET']}/{prefix}{name}.tgz"
    out = {"uri": uri, "sha256": sha, "bytes": bundle.stat().st_size, "dry_run": dry_run}
    if dry_run:
        return out
    aws = aws or aws_cli()
    if not aws:
        raise PushRefused("no aws CLI")
    cmd = [aws, "s3", "cp", str(bundle), uri, "--metadata", f"sha256={sha}", "--only-show-errors"]
    region = bucket_region(None, env_file)
    if region:
        cmd += ["--region", region]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f"upload to {uri} failed ({exc})") from None
    if r.returncode != 0:
        err = (r.stderr or r.stdout).strip()
        if "AccessDenied" in err or "Access Denied" in err:
            raise PushRefused(f"S3 denied the upload to {uri} (AccessDenied): this workspace's role may not write "
                              "catalogue/* -- only catalogue admins' workspace roles may (FactoryosCatalogueAdmin); "
                              "if you were made one, restart the workspace for the new role")
        raise RuntimeError(f"upload to {uri} failed: {err[-300:]}")
    return out
