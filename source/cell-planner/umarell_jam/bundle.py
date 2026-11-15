"""Portable, checksummed component bundles for the shared FactoryOS planner cache (import / push: standard library).

``python -m umarell_jam --root R bundle --out F.tgz [--cells …]`` writes a tar.gz::

    MANIFEST.json                       {"bundle_schema": "factoryos-planner-cache-v1",
                                         "kind": "umarell-jam-components", "name" (the suggested file name
                                         <cell>__<planner[:8]>-<impl[:8]>.tgz), "component_schema", "planner",
                                         "impl" (= "implementation"), "cell_planner_commit",
                                         "cells": [{"cell_id", "geometry_hash", "limits_hash"}],
                                         "files": {relpath: sha256}, "created",
                                         "stack": the numerical stack (umarell_jam/stack.py) of the
                                         machine that wrote the bundle}
    umarell-jam/<layer>/<key>.json      component files exactly as in the Store

A bundle holds the components of the given cells' geometry keys -- a cell of the workspace: its current key; a cell
the workspace does not have, or every cell without ``--cells``: the keys the Store holds certified primitives for, so
``bundle`` needs no workspace -- that is every valid Store entry (schema, key = digest(dependencies), payload checksum) whose dependencies name
one of the cells' ``geometry_hash`` -- and the timing-only ``phase_pack`` entries of the trajectories certified in
the bundle (they name the cell's ``limits_hash``). ``timeout_or_unknown`` entries, ``.lock`` / ``.pending-*`` files
and entries carrying a certificate for another scene are left out.

``import F.tgz …`` verifies a bundle completely before writing anything: only regular files under ``umarell-jam/``
plus MANIFEST.json (no ``..``, no links, no other top-level members), the bundle schema and kind, every member
against its manifest sha256 (no member missing, none extra), the Store checks of every component, that every
component belongs to one of the manifest's geometry keys and that every certificate in it (each ``geometry_hash``
field of the payload) names the same key. Any failure rejects the whole bundle (exit 1). Valid entries already in
the Store are never overwritten; the Store's 2 GB growth cap holds. Entries are written with ``atomic_json`` under
the per-key lock. The manifest's ``stack`` is compared with the local one (:func:`.stack.differences`: the
numerical package versions and the machine): a difference, or a bundle without ``stack``, is a warning on stderr
(and ``stack_differences`` / ``stack_warning`` in the result row); with ``strict_stack`` (``import --strict-stack``,
default ``$UMARELL_JAM_STRICT_STACK=1``) a difference rejects the bundle. The checks establish integrity, not
authenticity: a self-consistent forged bundle passes, so trust rests on who can write the catalogue prefix. Imported components stay ordinary cache entries: the planner's lookups (keys with the current
cell's geometry, limits, planner and checker revisions) and every release check decide whether they are used.

Component keys do not depend on the workspace's absolute location: ``Cell`` hashes files by path relative to the
workspace and maps ``/var/lib/factoryos/`` references onto it, so a bundle built on one FactoryOS workspace hits on
another one with byte-identical cell files. A *staged* copy (``stage``) rewrites those references to its own path,
which is file content: its keys are those of that location only (docs/umarell_jam.md, "Cache compatibility").
"""
from __future__ import annotations

import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import time

from . import SCHEMA
from . import stack as numerical_stack
from .store import Store, atomic_json, canonical, digest

BUNDLE_SCHEMA = "factoryos-planner-cache-v1"
KIND = "umarell-jam-components"
PREFIX = "umarell-jam"
MANIFEST = "MANIFEST.json"
REPO = Path(__file__).resolve().parents[1]


class BundleError(ValueError):
    pass


def _contains(value, needle: str) -> bool:
    if isinstance(value, dict):
        return any(_contains(v, needle) for v in value.values())
    if isinstance(value, list):
        return any(_contains(v, needle) for v in value)
    return value == needle


def _collect(value, key: str, out: set) -> set:
    if isinstance(value, dict):
        for k, v in value.items():
            if k == key and isinstance(v, str):
                out.add(v)
            else:
                _collect(v, key, out)
    elif isinstance(value, list):
        for v in value:
            _collect(v, key, out)
    return out


def valid(doc, layer: str, stem: str) -> bool:
    """Store.get / Store.inspect's checks of one component file."""
    try:
        return (isinstance(doc, dict) and doc["schema_version"] == SCHEMA and doc.get("layer", layer) == layer
                and layer.replace("_", "").isalnum() and len(stem) == 64 and int(stem, 16) >= 0
                and doc["key"] == stem == digest(doc["dependencies"])
                and digest(doc["payload"]) == doc["payload_sha256"])
    except (KeyError, TypeError, ValueError):
        return False


def _geometry_of(doc, geometries: dict, trajectories: dict):
    """The geometry key a component belongs to (None: none of the bundle's)."""
    dep = doc["dependencies"]
    if doc.get("layer") == "phase_pack":
        for geometry, limits in geometries.items():
            if dep.get("limits") == limits and dep.get("trajectory") in trajectories.get(geometry, ()):
                return geometry
        return None
    owners = [g for g in geometries if _contains(dep, g)]
    if len(owners) != 1:
        return None
    certificates = _collect(doc["payload"], "geometry_hash", set())
    return owners[0] if certificates <= {owners[0]} else None


def select(store_root: Path, geometries: dict) -> dict:
    """{relpath: path} of the bundleable components of ``geometries`` ({geometry_hash: limits_hash})."""
    store_root = Path(store_root)
    chosen, phase_packs, trajectories = {}, [], {}
    for path in sorted(store_root.glob("*/*.json")):
        layer = path.parent.name
        if path.name.startswith(".pending-"):
            continue
        try:
            doc = json.loads(path.read_bytes())
        except (OSError, ValueError):
            continue
        if not valid(doc, layer, path.stem) or not isinstance(doc["payload"], dict) \
                or doc["payload"].get("status") == "timeout_or_unknown":
            continue
        doc = {**doc, "layer": layer}
        if layer == "phase_pack":
            phase_packs.append((path, doc))
            continue
        geometry = _geometry_of(doc, geometries, trajectories)
        if geometry is not None:
            chosen[f"{PREFIX}/{layer}/{path.stem}.json"] = path
            _collect(doc["payload"], "trajectory_hash", trajectories.setdefault(geometry, set()))
    for path, doc in phase_packs:
        if _geometry_of(doc, geometries, trajectories) is not None:
            chosen[f"{PREFIX}/phase_pack/{path.stem}.json"] = path
    return chosen


def cell_planner_commit() -> str | None:
    if os.environ.get("FACTORY_OS_CELL_PLANNER_COMMIT"):
        return os.environ["FACTORY_OS_CELL_PLANNER_COMMIT"].strip()
    try:
        r = subprocess.run(["git", "-C", str(REPO), "rev-parse", "HEAD"], capture_output=True, text=True, timeout=10)
        if r.returncode == 0:
            return r.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        pass
    stamp = REPO / ".cell-planner.sha256"
    return stamp.read_text().strip() if stamp.is_file() else None


def default_name(cells: list, planner: str, implementation: str) -> str:
    who = "+".join(cells) if len(cells) <= 3 else f"{len(cells)}cells"
    return f"{who}__{(planner or 'none')[:8]}-{implementation[:8]}.tgz"


def store_entries(store_root: Path) -> list:
    """[{cell_id, geometry_hash, limits_hash}] of every scene the Store holds certified primitives for (the base,
    refined, supplemental and placement primitive layers name their cell in the payload)."""
    found = {}
    for layer in ("primitive", "refined_primitive", "primitive_supplement", "placement_primitive"):
        for path in sorted(Path(store_root).glob(f"{layer}/*.json")):
            try:
                doc = json.loads(path.read_bytes())
            except (OSError, ValueError):
                continue
            if not valid(doc, layer, path.stem) or not isinstance(doc["payload"], dict) \
                    or doc["payload"].get("status") != "certified" or not isinstance(doc["payload"].get("cell"), str):
                continue
            dep = doc["dependencies"]
            base = dep.get("base") if isinstance(dep.get("base"), dict) else {}
            base = base.get("base") if isinstance(base.get("base"), dict) else base
            geometry = dep.get("cell_geometry") or base.get("cell_geometry") or base.get("geometry")
            limits = (dep.get("motion") or {}).get("limits") or (base.get("motion") or {}).get("limits") \
                or base.get("limits")
            if isinstance(geometry, str) and isinstance(limits, str):
                found[(doc["payload"]["cell"], geometry)] = limits
    return [{"cell_id": c, "geometry_hash": g, "limits_hash": lim} for (c, g), lim in sorted(found.items())]


def make_bundle(layout, out: Path | None = None, cells=None) -> dict:
    """Write the bundle of ``cells``; returns its summary. A cell of the workspace contributes its current
    geometry key; a cell the workspace does not have (or every cell, without ``cells``) the geometry keys the Store
    holds certified primitives for, so ``bundle`` needs no workspace."""
    from .native import code_revision
    from .preparation import implementation_revision
    stored = store_entries(layout.components)
    entries = []
    for cid in (cells or []):
        current = None
        if (layout.workspace / "cells" / cid).is_dir():
            from .cell import Cell
            try:
                cell = Cell(layout.workspace, cid)
                current = {"cell_id": cid, "geometry_hash": cell.geometry_hash, "limits_hash": cell.limits_hash}
            except Exception as exc:  # noqa: BLE001 - the Store's keys of the cell instead
                print(json.dumps({"cell": cid, "workspace_cell_unloadable": f"{type(exc).__name__}: {exc}"}),
                      flush=True)
        found = [current] if current else [e for e in stored if e["cell_id"] == cid]
        if not found:
            raise BundleError(f"cell {cid}: not in the workspace {layout.workspace} and no certified components "
                              f"in {layout.components}")
        entries += found
    if not cells:
        entries = stored
    if not entries:
        raise BundleError(f"nothing to bundle: no certified components in {layout.components}")
    planner = code_revision(layout.umarell) if (layout.umarell / "agents").is_dir() else None
    if out is None:
        out = layout.root / "bundles"
    return write_bundle(layout.components, entries, out, planner, implementation_revision())


def write_bundle(store_root: Path, entries: list, out: Path, planner, implementation: str,
                 stack: dict | None = None) -> dict:
    """The bundle of ``entries`` ([{cell_id, geometry_hash, limits_hash}]) from ``store_root``; ``out`` a .tgz file
    or a directory (then :func:`default_name`); ``stack`` default the local :func:`.stack.fingerprint`."""
    stack = numerical_stack.fingerprint() if stack is None else stack
    chosen = select(store_root, {e["geometry_hash"]: e["limits_hash"] for e in entries})
    files = {rel: hashlib.sha256(path.read_bytes()).hexdigest() for rel, path in sorted(chosen.items())}
    name = default_name(sorted({e["cell_id"] for e in entries}), planner, implementation)
    manifest = {"bundle_schema": BUNDLE_SCHEMA, "kind": KIND, "name": name, "component_schema": SCHEMA,
                "planner": planner, "impl": implementation, "implementation": implementation,
                "cell_planner_commit": cell_planner_commit(), "cells": entries, "files": files,
                "created": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "stack": stack}
    out = Path(out)
    if out.suffix != ".tgz":
        out = out / name
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_name(f".{out.name}.tmp-{os.getpid()}")
    with tarfile.open(tmp, "w:gz") as tar:
        data = json.dumps(manifest, indent=1, sort_keys=True).encode()
        info = tarfile.TarInfo(MANIFEST)
        info.size, info.mtime, info.mode = len(data), int(time.time()), 0o644
        tar.addfile(info, io.BytesIO(data))
        for rel, path in sorted(chosen.items()):
            data = path.read_bytes()
            if hashlib.sha256(data).hexdigest() != files[rel]:
                raise BundleError(f"{path} changed while bundling")
            info = tarfile.TarInfo(rel)
            info.size, info.mtime, info.mode = len(data), int(path.stat().st_mtime), 0o644
            tar.addfile(info, io.BytesIO(data))
    os.replace(tmp, out)
    layers = {}
    for rel in files:
        layers[rel.split("/")[1]] = layers.get(rel.split("/")[1], 0) + 1
    return {"bundle": str(out), "sha256": hashlib.sha256(out.read_bytes()).hexdigest(), "bytes": out.stat().st_size,
            "cells": entries, "components": len(files), "layers": layers, "planner": planner,
            "implementation": implementation, "stack": stack}


def verify(bundle: Path) -> tuple[dict, dict]:
    """(manifest, {(layer, key): doc}) of a fully verified bundle; raises BundleError."""
    try:
        tar = tarfile.open(bundle, "r:gz")
    except (OSError, tarfile.TarError) as exc:
        raise BundleError(f"not a readable tar.gz: {exc}") from exc
    with tar:
        members, data = {}, {}
        for m in tar.getmembers():
            parts = Path(m.name).parts
            if m.name.startswith("/") or ".." in parts:
                raise BundleError(f"unsafe member name {m.name!r}")
            if m.isdir() and parts and parts[0] == PREFIX:
                continue
            if not m.isfile():
                raise BundleError(f"member {m.name!r} is not a regular file")
            if m.name != MANIFEST and (len(parts) != 3 or parts[0] != PREFIX):
                raise BundleError(f"unknown member {m.name!r}")
            if m.name in members:
                raise BundleError(f"duplicate member {m.name!r}")
            members[m.name] = m
            data[m.name] = tar.extractfile(m).read()
    if MANIFEST not in data:
        raise BundleError("no MANIFEST.json")
    try:
        manifest = json.loads(data.pop(MANIFEST))
    except ValueError as exc:
        raise BundleError(f"unreadable MANIFEST.json: {exc}") from exc
    if manifest.get("bundle_schema") != BUNDLE_SCHEMA or manifest.get("kind") != KIND:
        raise BundleError(f"wrong bundle kind {manifest.get('bundle_schema')!r}/{manifest.get('kind')!r} "
                          f"(expected {BUNDLE_SCHEMA}/{KIND})")
    if manifest.get("component_schema") != SCHEMA:
        raise BundleError(f"component schema {manifest.get('component_schema')!r}, expected {SCHEMA}")
    files = manifest.get("files")
    if not isinstance(files, dict) or set(files) != set(data):
        listed = set(files) if isinstance(files, dict) else set()
        raise BundleError(f"manifest/member mismatch: {len(listed - set(data))} listed but missing, "
                          f"{len(set(data) - listed)} present but not listed")
    geometries = {}
    for e in manifest.get("cells") or []:
        if not (isinstance(e, dict) and isinstance(e.get("geometry_hash"), str)
                and isinstance(e.get("limits_hash"), str)):
            raise BundleError(f"bad cells entry {e!r}")
        geometries[e["geometry_hash"]] = e["limits_hash"]
    docs, trajectories, phase_packs = {}, {}, []
    for rel in sorted(data):
        if hashlib.sha256(data[rel]).hexdigest() != files[rel]:
            raise BundleError(f"checksum mismatch: {rel}")
        _, layer, name = Path(rel).parts
        stem = name.removesuffix(".json")
        try:
            doc = json.loads(data[rel])
        except ValueError as exc:
            raise BundleError(f"unreadable component {rel}: {exc}") from exc
        if not name.endswith(".json") or not valid(doc, layer, stem):
            raise BundleError(f"component fails the Store checks: {rel}")
        if isinstance(doc["payload"], dict) and doc["payload"].get("status") == "timeout_or_unknown":
            raise BundleError(f"bundle carries an inconclusive (timeout_or_unknown) entry: {rel}")
        if layer == "phase_pack":
            phase_packs.append((rel, stem, doc))
            continue
        geometry = _geometry_of({**doc, "layer": layer}, geometries, trajectories)
        if geometry is None:
            raise BundleError(f"component is not of the bundle's geometry keys (or certifies another scene): {rel}")
        _collect(doc["payload"], "trajectory_hash", trajectories.setdefault(geometry, set()))
        docs[(layer, stem)] = doc
    for rel, stem, doc in phase_packs:
        if _geometry_of({**doc, "layer": "phase_pack"}, geometries, trajectories) is None:
            raise BundleError(f"phase pack of no trajectory certified in the bundle: {rel}")
        docs[("phase_pack", stem)] = doc
    return manifest, docs


def check_stack(manifest: dict, strict: bool = False, local: dict | None = None, warn=None) -> dict:
    """{"stack_differences": …, "stack_warning": …} of the bundle's ``stack`` against ``local`` (default the local
    fingerprint). A difference warns (``warn``, default stderr); with ``strict`` it raises BundleError. A bundle
    without ``stack`` (written before it was recorded) only warns."""
    if warn is None:
        def warn(message):
            print(message, file=sys.stderr, flush=True)
    recorded = manifest.get("stack")
    if not isinstance(recorded, dict):
        message = (f"WARNING: bundle {manifest.get('name')!r} records no numerical stack (written before the stack "
                   "was recorded): it may have been prepared under other numerical libraries")
        warn(message)
        return {"stack_differences": None, "stack_warning": message}
    diff = numerical_stack.differences(recorded, local)
    if not diff:
        return {"stack_differences": {}, "stack_warning": None}
    message = (f"numerical stack of bundle {manifest.get('name')!r} differs from this machine's: "
               f"{numerical_stack.describe(diff)}")
    if strict:
        raise BundleError(f"{message} (--strict-stack / ${numerical_stack.STRICT_ENV}=1)")
    message = f"WARNING: {message}; importing anyway (--strict-stack refuses)"
    warn(message)
    return {"stack_differences": diff, "stack_warning": message}


def import_bundle(layout, bundle: Path, *, strict_stack: bool = False, stack_check: bool = True,
                  local_stack: dict | None = None) -> dict:
    """Verify ``bundle`` completely (and its numerical stack, :func:`check_stack`, unless ``stack_check`` is false),
    then write its components that are not already valid in the Store."""
    manifest, docs = verify(bundle)
    stack_row = check_stack(manifest, strict_stack, local_stack) if stack_check else {}
    store = Store(layout.components)
    store.root.mkdir(parents=True, exist_ok=True)
    with store.lock("metadata", "0" * 64):
        size = sum(p.stat().st_size for p in store.root.glob("*/*.json"))
    kept = imported = replaced = 0
    for (layer, stem), doc in sorted(docs.items()):
        path = store.path(layer, stem)
        with store.lock(layer, stem):                  # Store.obtain's order: the key lock, then metadata
            try:
                present = valid(json.loads(path.read_bytes()), layer, stem)
            except (OSError, ValueError):
                present = False
            if present:
                kept += 1
                continue
            with store.lock("metadata", "0" * 64):
                old = path.stat().st_size if path.exists() else 0
                extra = len(canonical(doc)) - old
                if size + extra > store.max_bytes:
                    raise OSError(f"component cache capacity exceeded ({store.max_bytes} bytes) after "
                                  f"{imported} imported component(s)")
                replaced += bool(old)
                atomic_json(path, doc)
                size += extra
            imported += 1
    return {"bundle": str(bundle), "cells": manifest.get("cells"), "planner": manifest.get("planner"),
            "implementation": manifest.get("implementation"), "components": len(docs), "imported": imported,
            "replaced_invalid": replaced, "kept": kept, **stack_row}


def push(bundle: Path, dest: str, *, aws: str | None = None, region: str | None = None, timeout_s: float = 600.) -> dict:
    """Upload a bundle to the catalogue after a test import into an empty temporary Store:
    ``aws s3 cp <bundle> <dest>/<name> --metadata sha256=<sha256>``. Raises BundleError / RuntimeError."""
    bundle = Path(bundle)
    if not dest.startswith("s3://"):
        raise BundleError(f"--dest must be an s3:// prefix, not {dest!r}")
    with tempfile.TemporaryDirectory(prefix="umarell-jam-push-") as tmp:
        from types import SimpleNamespace
        # the uploader's stack is not the consumer's: importing machines compare the stack
        check = import_bundle(SimpleNamespace(components=Path(tmp) / "components"), bundle, stack_check=False)
    sha = hashlib.sha256(bundle.read_bytes()).hexdigest()
    target = dest if not dest.endswith("/") and dest.rsplit("/", 1)[-1].endswith(".tgz") \
        else dest.rstrip("/") + "/" + bundle.name
    cmd = [aws or os.environ.get("AWS_CLI", "aws"), "s3", "cp", str(bundle), target,
           "--metadata", f"sha256={sha}", "--only-show-errors"]
    if region:
        cmd += ["--region", region]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    if r.returncode != 0:
        raise RuntimeError(f"aws s3 cp failed ({r.returncode}): {(r.stderr or r.stdout).strip()[-400:]}")
    return {"bundle": str(bundle), "uri": target, "sha256": sha, "components": check["components"],
            "cells": check["cells"]}
