"""Create or refresh the patched Umarell copy ``R/umarell`` under ``R/.setup.lock``.

The copy is what the weld_cache session planned on: ``umarell_fast/apply.py --variant U3 --with-upstream-fixes``
(base 76ea13a, fix set v8, the U3 hook patches) of a plain Umarell 76ea13a checkout, then :func:`install.install`
(the integration hooks recorded in ``umarell_jam/patches/integration.diff``). The component keys hash the copy's
planner sources by relative path (``native.code_revision``), so its location does not matter.

The copy is rebuilt (into a temporary sibling, then renamed into place) when its stamp ``UMARELL_JAM_COPY.json``
differs from what this checkout would build: the checkout path and file fingerprint, apply.py, the patch list,
the installer and the integration record.
"""
from __future__ import annotations

from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import time

BASE = "76ea13a"
STAMP = "UMARELL_JAM_COPY.json"
HERE = Path(__file__).resolve().parent
INTEGRATION_DIFF = HERE / "patches" / "integration.diff"
HOOKED = ("agents/trajectory/agent.py", "agents/cell/bt_executor.py")      # the hooks that import umarell_jam


class SetupError(RuntimeError):
    pass


def _sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_fingerprint(src: Path) -> str:
    """Relative path, size and mtime of every file of the checkout (no .git / caches / venvs)."""
    h = hashlib.sha256()
    for directory, dirs, files in os.walk(src):
        dirs[:] = sorted(d for d in dirs if d not in {".git", "__pycache__", ".pytest_cache", ".hypothesis"}
                         and not d.startswith(".venv"))
        for name in sorted(files):
            p = Path(directory) / name
            try:
                st = p.stat()
            except OSError:
                continue
            h.update(f"{p.relative_to(src)} {st.st_size} {st.st_mtime_ns}\n".encode())
    return h.hexdigest()


def default_source() -> Path:
    from umarell_fast import apply as A
    return A.BASES[BASE].src


def wanted(src: Path) -> dict:
    from umarell_fast import apply as A
    B = A.BASES[BASE]
    diffs = list(A.fix_diffs(B.default_set)) + list(A.hook_diffs("U3", BASE))
    return {"schema_version": "umarell-jam-copy-v1", "base": BASE, "fix_set": B.default_set,
            "source": str(src), "source_fingerprint": source_fingerprint(src),
            "apply": _sha(Path(A.__file__)),
            "patches": [[p.name, _sha(p)] for p in diffs],
            "installer": _sha(HERE / "install.py"), "integration": _sha(INTEGRATION_DIFF)}


@contextmanager
def setup_lock(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        yield stream


def current(dest: Path, want: dict) -> bool:
    try:
        have = json.loads((dest / STAMP).read_text())
    except (OSError, ValueError):
        return False
    return {k: have.get(k) for k in want} == want


@contextmanager
def ready(layout, source: Path | None = None, *, log=print, force=False):
    """Yield the stamp of a current ``layout.umarell`` while holding a SHARED lock on ``R/.setup.lock``: jobs plan
    concurrently on a current copy; a missing or stale copy is rebuilt under the EXCLUSIVE lock, which waits until
    no job plans on the old copy. Raises SetupError."""
    if layout.umarell_override is not None:      # an explicitly given, already patched copy: used as it is
        hooks = [rel for rel in HOOKED if "umarell_jam." not in (layout.umarell_override / rel).read_text()]
        if hooks:
            raise SetupError(f"{layout.umarell_override}: its integration hooks ({', '.join(hooks)}) do not import "
                             "umarell_jam (a copy installed by an older package?); pass the plain Umarell checkout "
                             "as --umarell, or omit it, and umarell_jam builds R/umarell")
        yield {"source": str(layout.umarell_override), "prebuilt": True,
               **json.loads((layout.umarell_override / "UF_APPLIED.json").read_text())}
        return
    src = Path(source).resolve() if source else default_source()
    layout.setup_lock.parent.mkdir(parents=True, exist_ok=True)
    with layout.setup_lock.open("a") as stream:
        fcntl.flock(stream, fcntl.LOCK_SH)
        want = None if force else wanted(src)
        if want is None or not current(layout.umarell, want):
            fcntl.flock(stream, fcntl.LOCK_UN)       # two shared holders upgrading in place would deadlock
            fcntl.flock(stream, fcntl.LOCK_EX)
            _ensure_locked(layout, src, log=log, force=force)
            fcntl.flock(stream, fcntl.LOCK_SH)
        yield json.loads((layout.umarell / STAMP).read_text())


def ensure(layout, source: Path | None = None, *, log=print, force=False) -> dict:
    """The stamp of a current ``layout.umarell``; builds it when missing or stale. Raises SetupError."""
    with setup_lock(layout.setup_lock):
        return _ensure_locked(layout, source, log=log, force=force)


def _ensure_locked(layout, source, *, log, force):
    src = Path(source).resolve() if source else default_source()
    if not (src / "agents" / "trajectory" / "backends" / "ompl" / "planner.py").is_file():
        raise SetupError(f"{src} is not a plain Umarell {BASE} checkout "
                         "(no agents/trajectory/backends/ompl/planner.py)")
    dest = layout.umarell
    want = wanted(src)
    if not force and current(dest, want):
        return json.loads((dest / STAMP).read_text())
    log(f"[umarell_jam] building the patched Umarell copy {dest} from {src}")
    started = time.monotonic()
    tmp = dest.with_name(f".{dest.name}.build-{os.getpid()}")
    shutil.rmtree(tmp, ignore_errors=True)
    from umarell_fast import apply as A
    from .install import install
    try:
        A.main(["--variant", "U3", "--base", BASE, "--with-upstream-fixes", "--dest", str(tmp), "--force",
                "--src", str(src)])
        layout.logs.mkdir(parents=True, exist_ok=True)
        record = layout.logs / "integration.diff"
        record.unlink(missing_ok=True)
        changes = install(tmp, record)
    except (SystemExit, Exception) as exc:  # noqa: BLE001 - apply.py reports failures by SystemExit
        shutil.rmtree(tmp, ignore_errors=True)
        raise SetupError(f"could not build the patched Umarell copy: {exc}") from exc
    recorded = record.read_bytes() if record.is_file() else b""
    stamp = {**want, "created": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
             "integration_edits": changes,
             "integration_matches_record": recorded == INTEGRATION_DIFF.read_bytes()}
    if not stamp["integration_matches_record"]:
        log(f"[umarell_jam] WARNING: the installed hooks differ from {INTEGRATION_DIFF} "
            f"(this checkout's diff: {record})")
    (tmp / STAMP).write_text(json.dumps(stamp, indent=1))
    old = dest.with_name(f".{dest.name}.old-{os.getpid()}")
    if dest.exists():
        os.replace(dest, old)
    os.replace(tmp, dest)
    shutil.rmtree(old, ignore_errors=True)
    log(f"[umarell_jam] patched Umarell copy ready ({time.monotonic() - started:.1f} s)")
    return stamp
