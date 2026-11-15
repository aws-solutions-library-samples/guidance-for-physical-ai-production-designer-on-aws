"""Immutable checksummed components; atomic publication under per-key locks."""
from __future__ import annotations

from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Callable

from . import SCHEMA


def canonical(value) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def digest(value) -> str:
    return hashlib.sha256(canonical(value)).hexdigest()


def atomic_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=".pending-", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(canonical(value))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
        fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    finally:
        if os.path.exists(name):
            os.unlink(name)


class Store:
    def __init__(self, root: str | Path, max_bytes: int = 2_000_000_000):
        self.root = Path(root).resolve()
        self.max_bytes = max_bytes
        self.hits = self.misses = self.corrupt = 0

    def path(self, layer: str, key: str) -> Path:
        if not layer.replace("_", "").isalnum() or len(key) != 64:
            raise ValueError("invalid component address")
        int(key, 16)
        return self.root / layer / f"{key}.json"

    @contextmanager
    def lock(self, layer: str, key: str):
        p = self.path(layer, key).with_suffix(".lock")
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("a") as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            yield

    def get(self, layer: str, dependencies: dict):
        key = digest(dependencies)
        try:
            doc = json.loads(self.path(layer, key).read_bytes())
            if (doc["schema_version"] != SCHEMA or doc["key"] != key
                    or doc["dependencies"] != dependencies
                    or digest(doc["payload"]) != doc["payload_sha256"]):
                raise ValueError("checksum, dependencies or schema mismatch")
        except FileNotFoundError:
            self.misses += 1
            return None
        except (OSError, ValueError, KeyError, TypeError):
            self.corrupt += 1
            self.misses += 1
            return None
        self.hits += 1
        return doc["payload"]

    def obtain(self, layer: str, dependencies: dict, build: Callable):
        """Timeouts may be recorded but are not reusable negative proofs."""
        key = digest(dependencies)
        with self.lock(layer, key):
            found = self.get(layer, dependencies)
            if found is not None and found.get("status") != "timeout_or_unknown":
                return found, True
            payload = build()
            doc = {"schema_version": SCHEMA, "key": key, "layer": layer,
                   "dependencies": dependencies, "payload": payload,
                   "payload_sha256": digest(payload)}
            # A global publication lock makes the growth cap race-free. Existing
            # components are immutable; a bad entry is replaced only at its key.
            with self.lock("metadata", "0" * 64):
                size = sum(p.stat().st_size for p in self.root.glob("*/*.json"))
                old = self.path(layer, key)
                extra = len(canonical(doc)) - (old.stat().st_size if old.exists() else 0)
                if size + extra > self.max_bytes:
                    raise OSError("component cache capacity exceeded")
                atomic_json(old, doc)
            return payload, False

    def inspect(self) -> dict:
        entries = []
        for p in sorted(self.root.glob("*/*.json")):
            try:
                d = json.loads(p.read_bytes())
                good = (d["schema_version"] == SCHEMA
                        and digest(d["dependencies"]) == d["key"] == p.stem
                        and digest(d["payload"]) == d["payload_sha256"])
                entries.append({"layer": p.parent.name, "key": p.stem,
                                "valid_checksum": good, "bytes": p.stat().st_size,
                                "status": d["payload"].get("status")})
            except (ValueError, KeyError, TypeError, OSError):
                entries.append({"path": str(p), "valid_checksum": False})
        return {"schema_version": SCHEMA, "entries": entries,
                "bytes": sum(e.get("bytes", 0) for e in entries),
                "incomplete_writes": len(list(self.root.glob("*/.pending-*")))}
