"""Package-independent source revisions of the modules whose bytes enter component keys.

Component keys hash the source of the checker, retimer and router modules (``sha256(file bytes)``, by file name).
umarell_jam is umarell_fast.weld_cache moved to a top-level package. Every key-relevant module is byte-identical to
its weld_cache original except the two that start a worker process by module name: their spawn line names the
package and the repository root (``parents[2]`` in umarell_fast/weld_cache, ``parents[1]`` here).

:func:`source_sha256` hashes a module with exactly those port edits undone, so its revision is the weld_cache
revision of the same numerical code and an existing component cache keeps its keys and certificates. Every edit in
:data:`PORT_EDITS` must occur exactly once (else ValueError): any other change to a module still changes its
revision, as before. tests/umarell_jam/test_cache_compat.py pins the resulting revisions.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

LEGACY_PACKAGE = "umarell_fast.weld_cache"

_SPAWN_ROOT = (b'env={**os.environ, "PYTHONPATH": str(Path(__file__).resolve().parents[1])}',
               b'env={**os.environ, "PYTHONPATH": str(Path(__file__).resolve().parents[2])}')

# file name -> [(bytes in umarell_jam, bytes in umarell_fast/weld_cache)]
PORT_EDITS = {
    "initial_connection.py": [
        (b"from .store import Store, atomic_json\nfrom .revision import source_sha256\n",
         b"from .store import Store, atomic_json\n"),
        (b'"checkers": {name: source_sha256(directory / name)',
         b'"checkers": {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()'),
        (b'"-m", "umarell_jam.initial_connection"', b'"-m", "umarell_fast.weld_cache.initial_connection"'),
        _SPAWN_ROOT,
    ],
    "route_connectors.py": [
        (b"from .store import atomic_json\nfrom .revision import source_sha256\n",
         b"from .store import atomic_json\n"),
        (b'"router": source_sha256(Path(__file__))}',
         b'"router": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}'),
        (b'"-m", "umarell_jam.route_connectors"', b'"-m", "umarell_fast.weld_cache.route_connectors"'),
        _SPAWN_ROOT,
    ],
}


def canonical_source(path: str | Path) -> bytes:
    """The module's bytes as they were in umarell_fast/weld_cache (identity for every other module)."""
    path = Path(path)
    data = path.read_bytes()
    for new, old in PORT_EDITS.get(path.name, ()):
        if data.count(new) != 1:
            raise ValueError(f"{path.name}: port edit {new[:60]!r} not found exactly once; "
                             "update umarell_jam/revision.py PORT_EDITS with the module")
        data = data.replace(new, old)
    return data


def source_sha256(path: str | Path) -> str:
    return hashlib.sha256(canonical_source(path)).hexdigest()
