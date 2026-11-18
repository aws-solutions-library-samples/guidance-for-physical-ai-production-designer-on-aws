"""Create or refresh the patched Umarell copy ``R/umarell`` under ``R/.setup.lock``.

Two plain Umarell bases (:data:`BASES`); which one a checkout is, is read from the checkout (:func:`base_of`):

* ``8a02db9`` (the default; the public https://github.com/aws-samples/sample-industrial-multi-robot-agents,
  a sanitized 08d5a39): umarell_jam's own series ``umarell_jam/patches_8a02db9/series.txt`` (the upstream fix set
  v10 rebased onto the public commit, without 28, which the public commit contains, plus the U3 hook U3_02), then
  :func:`install.install`, recorded in ``umarell_jam/patches_8a02db9/integration.diff``. No apply.py, no tarball:
  the public commit as fetched from git.
* ``76ea13a`` (what the weld_cache session planned on; the internal servers' archive): ``umarell_fast/apply.py
  --variant U3 --with-upstream-fixes`` (base 76ea13a, fix set v8, the U3 hook patches), then :func:`install.install`
  (recorded in ``umarell_jam/patches/integration.diff``). Its stamp is the one this module always wrote, so an
  existing 76ea13a ``R/umarell`` stays current.

The component keys hash the copy's planner sources by relative path (``native.code_revision``), so its location does
not matter, but its base does: components certified on one base are cache misses on the other.

The copy is rebuilt (into a temporary sibling, then renamed into place) when its stamp ``UMARELL_JAM_COPY.json``
differs from what this checkout would build: the base, the checkout path and file fingerprint, the patch list, the
installer and the integration record (76ea13a: also apply.py).
"""
from __future__ import annotations

from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
STAMP = "UMARELL_JAM_COPY.json"
HOOKED = ("agents/trajectory/agent.py", "agents/cell/bt_executor.py")      # the hooks that import umarell_jam
BASES = {
    "8a02db9": SimpleNamespace(
        head="8a02db9", commit="8a02db9ba7db260366451f8c38bafb378cd65c29",
        url="https://github.com/aws-samples/sample-industrial-multi-robot-agents.git",
        src=REPO / "third_party" / "sample-umarell-8a02db9", series=HERE / "patches_8a02db9",
        integration=HERE / "patches_8a02db9" / "integration.diff"),
    "76ea13a": SimpleNamespace(
        head="76ea13a", commit="76ea13aafdeede10be11936cf0830613e8717907", url=None,
        src=REPO / "third_party" / "sample-umarell-76ea13a", series=None,
        integration=HERE / "patches" / "integration.diff"),
}
DEFAULT_BASE = "8a02db9"
BASE = DEFAULT_BASE                       # the default base (older callers read BASE)
INTEGRATION_DIFF = BASES["76ea13a"].integration     # 76ea13a's record (older callers); per base: BASES[b].integration


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


def _has(path: Path, text: str) -> bool:
    try:
        return text in path.read_text(errors="replace")
    except OSError:
        return False


def tree_base(src: Path) -> str | None:
    """Which Umarell a plain checkout is: its ``.umarell.sha256`` stamp (the orchestrator's fetch-umarell.sh writes
    the pinned commit there) when that names a base of :data:`BASES`, else its layout (the rules of umarell_fast/u3
    tree_base and the router's umarell_u3.base_of): ``8a02db9`` (the 08d5a39 layout, fastgeom.py + plan_memo.py,
    with upstream d749dc5's prompt callable and the public default cell), ``08d5a39``, ``dc60669`` (weld_surface.py
    + plan_jobs.py), ``76ea13a`` (planner backends packages), ``eccb1bc``, None (no Umarell tree)."""
    src = Path(src)
    try:
        stamped = (src / ".umarell.sha256").read_text().strip()
    except OSError:
        stamped = ""
    for name, b in BASES.items():
        if stamped and b.commit.startswith(stamped[:40]) and len(stamped) >= 7:
            return name
    traj = src / "agents" / "trajectory"
    if (traj / "backends" / "ompl" / "planner.py").is_file():
        if (src / "utils" / "weld_surface.py").is_file() and (traj / "plan_jobs.py").is_file():
            if (traj / "tools" / "fastgeom.py").is_file() and (traj / "tools" / "plan_memo.py").is_file():
                if _has(traj / "context_compaction.py", "def compacting_prompt(") and _has(
                        src / "agents" / "cell_defaults.py", 'DEFAULT_CELL_ID = "welding_cell_2_small"'):
                    return "8a02db9"
                return "08d5a39"
            return "dc60669"
        return "76ea13a"
    if (traj / "tools" / "ompl_planner.py").is_file():
        return "eccb1bc"
    return None


def base_of(src: Path) -> str:
    """The base of :data:`BASES` a plain checkout is; SetupError for anything else."""
    name = tree_base(src)
    if name not in BASES:
        raise SetupError(f"{src} is not a plain Umarell checkout umarell_jam builds on "
                         f"({' or '.join(BASES)}; its layout: {name or 'no Umarell tree'})")
    return name


def default_source() -> Path:
    """The repository's third_party/sample-umarell-<base>: 8a02db9's when present, else 76ea13a's."""
    for b in BASES.values():
        if b.src.is_dir():
            return b.src
    return BASES[DEFAULT_BASE].src


def series(base: str) -> list:
    """The patch files of a base's own series (series.txt: one file name per line, ``#`` comments)."""
    folder = BASES[base].series
    names = [line.split("#", 1)[0].strip() for line in (folder / "series.txt").read_text().splitlines()]
    return [folder / n for n in names if n]


def wanted(src: Path, base: str | None = None) -> dict:
    base = base or base_of(src)
    B = BASES[base]
    if B.series is None:                  # 76ea13a: apply.py's fix set v8 + U3 hooks (the stamp as it always was)
        from umarell_fast import apply as A
        AB = A.BASES[base]
        diffs = list(A.fix_diffs(AB.default_set)) + list(A.hook_diffs("U3", base))
        return {"schema_version": "umarell-jam-copy-v1", "base": base, "fix_set": AB.default_set,
                "source": str(src), "source_fingerprint": source_fingerprint(src),
                "apply": _sha(Path(A.__file__)),
                "patches": [[p.name, _sha(p)] for p in diffs],
                "installer": _sha(HERE / "install.py"), "integration": _sha(B.integration)}
    return {"schema_version": "umarell-jam-copy-v1", "base": base, "commit": B.commit,
            "fix_set": B.series.name, "source": str(src), "source_fingerprint": source_fingerprint(src),
            "patches": [[p.name, _sha(p)] for p in series(base)],
            "installer": _sha(HERE / "install.py"), "integration": _sha(B.integration)}


def apply_series(dest: Path, base: str, src: Path) -> None:
    """Copy a plain checkout to ``dest`` and apply the base's own series (``patch -p1 --forward --batch``, as
    apply.py does; any rejected hunk fails), then write ``UF_APPLIED.json`` (variant U3, the base, the patches)."""
    B = BASES[base]
    diffs = series(base)
    missing = [p.name for p in diffs if not p.is_file()]
    if missing:
        raise SetupError(f"{B.series}: missing patch file(s): {', '.join(missing)}")
    dest.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["rsync", "-a", "--exclude", ".git", "--exclude", "__pycache__", "--exclude", ".venv*",
                    "--exclude", ".hypothesis", "--exclude", ".pytest_cache", f"{src}/", f"{dest}/"], check=True)
    applied = []
    for diff in diffs:
        r = subprocess.run(["patch", "-p1", "--forward", "--batch", "--no-backup-if-mismatch", "-d", str(dest),
                            "-i", str(diff)], capture_output=True, text=True)
        if r.returncode != 0 or list(dest.rglob("*.rej")):
            raise SetupError(f"patch failed: {diff.name} on {src} (base {base}):\n{r.stdout}{r.stderr}")
        applied.append({"patch": str(diff.relative_to(REPO)), "sha256_16": _sha(diff)[:16]})
    (dest / "UF_APPLIED.json").write_text(json.dumps({
        "variant": "U3", "source": str(src), "umarell_head": B.head, "umarell_commit": B.commit, "base": base,
        "with_upstream_fixes": True, "upstream_fixes_set": B.series.name, "patches": applied,
        "overlay": f"umarell_fast.u3 (imported from {REPO}, not copied)", "built_by": "umarell_jam.native_copy",
        "created": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}, indent=1))


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
    base = base_of(src)
    layout.setup_lock.parent.mkdir(parents=True, exist_ok=True)
    with layout.setup_lock.open("a") as stream:
        fcntl.flock(stream, fcntl.LOCK_SH)
        want = None if force else wanted(src, base)
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
        raise SetupError(f"{src} is not a plain Umarell {' / '.join(BASES)} checkout "
                         "(no agents/trajectory/backends/ompl/planner.py)")
    base = base_of(src)
    B = BASES[base]
    dest = layout.umarell
    want = wanted(src, base)
    if not force and current(dest, want):
        return json.loads((dest / STAMP).read_text())
    log(f"[umarell_jam] building the patched Umarell copy {dest} from {src} (base {base})")
    started = time.monotonic()
    tmp = dest.with_name(f".{dest.name}.build-{os.getpid()}")
    shutil.rmtree(tmp, ignore_errors=True)
    from .install import install
    try:
        if B.series is None:
            from umarell_fast import apply as A
            A.main(["--variant", "U3", "--base", base, "--with-upstream-fixes", "--dest", str(tmp), "--force",
                    "--src", str(src)])
        else:
            apply_series(tmp, base, src)
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
             "integration_matches_record": recorded == B.integration.read_bytes()}
    if not stamp["integration_matches_record"]:
        log(f"[umarell_jam] WARNING: the installed hooks differ from {B.integration} "
            f"(this checkout's diff: {record})")
    (tmp / STAMP).write_text(json.dumps(stamp, indent=1))
    old = dest.with_name(f".{dest.name}.old-{os.getpid()}")
    if dest.exists():
        os.replace(dest, old)
    os.replace(tmp, dest)
    shutil.rmtree(old, ignore_errors=True)
    log(f"[umarell_jam] patched Umarell copy ready ({time.monotonic() - started:.1f} s)")
    return stamp
