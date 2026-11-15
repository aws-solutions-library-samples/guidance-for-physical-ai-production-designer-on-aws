"""A persistent, content-addressed cache of U3's planning tasks (umarell_fast.workers results), shared by jobs.

Every task U3 runs in its worker pool (``Orchestrator.submit``) is a function of plain data: the call's arguments plus
the planning session the worker opened on the cell (its robots, tools, obstacles, task programs, planning config,
the patched umarell code and the collision kernels). This module keys a task by exactly that content:

* the **task**: the worker function's name and its arguments, the cell id left out (canonical JSON);
* the **cell content**: the sha256 of every file of the run's cell copy (``CELL_BASE_PATH/<cell>``), following
  symlinks, by path relative to the cell (``plans/``, ``logs/``, ``recordings/``, ``checkpoints/`` and the views'
  bookkeeping left out). The cell id, its location and file mtimes are not part of it: a re-plan of the same cell,
  a copy of it under another name, and the same reference cell on another workspace all hit;
* the **code**: umarell_fast's own sources and the working copy's applied patch set (``UF_APPLIED.json``), the
  kernels, and the planner environment that changes results (OMPL / gate settings; paths left out).

A hit replays the result the task returned before, so a fully warm run repeats that run's decisions. It is a valid
result for these inputs: the same code checked it. Everything U3 checks after the pool (its own
``validate_behaviour_tree`` on the saved plan) and btexport's export checks and dense check run as before.

On disk: ``$U3_CACHE_DIR`` (FactoryOS: /var/lib/factoryos/cell-planner/cache/u3, set by the router's umarell_u3.py),
``<dir>/v1/<cell key[:2]>/<cell key>/<task key>.pkl`` (pickle, written to a temp file and renamed: concurrent jobs
never see a partial entry). A cell's entries are one directory, so the cache of a cell can be shipped (tar) and
pre-populated on another host (``python -m umarell_fast.plan_cache export|import``). Entries for failed planning tasks
(OMPL is time-budgeted, so a miss can be luck) expire after ``U3_CACHE_NEG_TTL_S`` (default 7 days).

Off unless ``U3_PLAN_CACHE=1``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import pickle
import sys
import tarfile
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Optional

VERSION = "v1"
HERE = Path(__file__).resolve().parent
SKIP_DIRS = {"plans", "logs", "recordings", "checkpoints", "__pycache__", ".git"}
SKIP_FILES = {".DS_Store"}
# planner environment that changes results; paths and ids (CELL_BASE_PATH, CELL_ID, PLANNER_SCRATCH_PATH, the
# Bedrock model, which U3 never calls) are left out
ENV_PREFIXES = ("OMPL_", "PLAN_", "UF_", "UMARELL_", "HOME_", "TRAJECTORY_", "U3_SPEED")
ENV_SKIP = {"UMARELL_REPO", "UMARELL_BASE", "U3_CACHE_DIR", "U3_PLAN_CACHE", "OMPL_SEED"}
# tasks whose failure may be luck (time-budgeted OMPL): a failed result expires
PLANNING = {"plan_chain", "plan_coupled", "assemble_trial"}
# tasks never cached: pool plumbing, per-process checks
NEVER = {"ping", "kernel_selfcheck", "init"}
# tasks reusable from a SIMILAR cell (same robots / tools / planning config, other obstacles or welds) after their
# legs pass umarell's own leg check in the current cell (``revalidate``)
SIMILAR = {"plan_chain", "plan_coupled", "gate_step", "gate_many"}


def enabled() -> bool:
    return os.environ.get("U3_PLAN_CACHE", "0").strip() not in ("", "0", "false", "off")


def cache_root() -> Path:
    env = os.environ.get("U3_CACHE_DIR", "").strip()
    if env:
        return Path(env)
    return Path(tempfile.gettempdir()) / f"u3-plan-cache-{os.getuid() if hasattr(os, 'getuid') else 'user'}"


def _enc(o: Any):
    """JSON for the key: numpy arrays / scalars as lists / floats, dataclasses and the rest by their fields / repr."""
    if hasattr(o, "tolist"):
        return o.tolist()
    if isinstance(o, (set, frozenset)):
        return sorted(o, key=repr)
    if isinstance(o, Path):
        return str(o)
    if hasattr(o, "__dict__"):
        return {"__type__": type(o).__name__, **{k: v for k, v in vars(o).items() if not k.startswith("_")}}
    return repr(o)


def canon(o: Any) -> str:
    return json.dumps(o, sort_keys=True, separators=(",", ":"), default=_enc)


def _sha(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def tree_digest(root: Path, skip_dirs=SKIP_DIRS) -> tuple[str, int]:
    """sha256 over (relative path, sha256 of content) of every file below ``root`` (symlinks followed; a symlink loop
    or a dangling link is hashed by its target string). Returns (digest, number of files)."""
    root = Path(root)
    h = hashlib.sha256()
    n = 0
    seen: set = set()
    for dirpath, dirnames, filenames in os.walk(root, followlinks=True):
        real = os.path.realpath(dirpath)
        if real in seen:                       # a symlink loop
            dirnames[:] = []
            continue
        seen.add(real)
        dirnames[:] = sorted(d for d in dirnames if d not in skip_dirs and not d.startswith(".u3-"))
        rel_dir = os.path.relpath(dirpath, root)
        for f in sorted(filenames):
            if f in SKIP_FILES or f.endswith((".pyc", ".tmp", ".lock")):
                continue
            p = os.path.join(dirpath, f)
            rel = os.path.normpath(os.path.join(rel_dir, f))
            try:
                with open(p, "rb") as fh:
                    d = hashlib.sha256()
                    for chunk in iter(lambda: fh.read(1 << 20), b""):
                        d.update(chunk)
                    digest = d.hexdigest()
            except OSError:
                digest = "link:" + (os.readlink(p) if os.path.islink(p) else "unreadable")
            h.update(rel.encode() + b"\0" + digest.encode() + b"\n")
            n += 1
    return h.hexdigest(), n


def geo_digest(cell_dir: Path, cell: Optional[str] = None) -> str:
    """The robot side of a cell: what a planned leg's result depends on apart from the cell's obstacles and its weld
    programs -- the robot packages (``robots/``: URDFs, meshes), ``agentic_planner/cell.yaml``, the SRDFs and every
    planning config file but ``obstacles.yaml`` (``config/*.yaml``: robots.yaml with the bases, tools, limits,
    clearance policy). Obstacles, task programs, refine points, fixtures and the scene are left out: a result reused
    across them is re-checked against them (:func:`revalidate`). The cell's id in those files (cell.yaml ``name`` /
    ``cell_id``, the generated headers) is replaced by a placeholder: a copy of a cell under another name matches."""
    cell_dir = Path(cell_dir)
    cid = (cell or cell_dir.name).encode()
    h = hashlib.sha256()
    parts = [("robots", tree_digest(cell_dir / "robots")[0] if (cell_dir / "robots").exists() else "none"),
             ("srdf", tree_digest(cell_dir / "agentic_planner" / "srdf")[0]
              if (cell_dir / "agentic_planner" / "srdf").exists() else "none")]
    ap = cell_dir / "agentic_planner"
    for f in [ap / "cell.yaml"] + sorted((ap / "config").glob("*.yaml")):
        if f.name == "obstacles.yaml" or not f.is_file():
            continue
        parts.append((str(f.relative_to(cell_dir)), _sha(f.read_bytes().replace(cid, b"<cell>"))))
    for name, d in parts:
        h.update(name.encode() + b"\0" + d.encode() + b"\n")
    return h.hexdigest()


APPLIED_KEYS = ("variant", "umarell_commit", "base", "with_upstream_fixes", "upstream_fixes_set", "patches")


def applied_digest(path: Path) -> str:
    """What the working copy's UF_APPLIED.json says about its CODE: the umarell commit, the fix set and every applied
    patch with its hash. Its ``created`` time and the paths it was built from are left out -- they change on every
    rebuild (every release) and on every host, and a key with them never matches a cache built elsewhere."""
    try:
        d = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return "none"
    return _sha(canon({k: d.get(k) for k in APPLIED_KEYS}).encode())


def code_digest(repo: Optional[Path], kernels: str) -> str:
    """umarell_fast's sources, the working copy's applied patches (UF_APPLIED.json), the kernels."""
    h = hashlib.sha256()
    for p in sorted(HERE.rglob("*.py")):
        if "__pycache__" in p.parts:
            continue
        h.update(str(p.relative_to(HERE)).encode() + b"\0" + _sha(p.read_bytes()).encode() + b"\n")
    if repo is not None:
        h.update(b"applied\0" + applied_digest(Path(repo) / "UF_APPLIED.json").encode())
    h.update(b"kernels\0" + str(kernels).encode())
    return h.hexdigest()


def env_digest(env: dict) -> str:
    keep = {k: str(v) for k, v in (env or {}).items() if k.startswith(ENV_PREFIXES) and k not in ENV_SKIP}
    return _sha(canon(keep).encode())


class PlanCache:
    """The cache of one U3 run (one cell, one code version). Thread-safe (the pool's done callbacks write)."""

    def __init__(self, root: Path, cell: str, cell_dir: Path, repo: Optional[Path], kernels: str, env: dict,
                 neg_ttl_s: Optional[float] = None, log=None):
        t0 = time.monotonic()
        self.cell = cell
        self.root = Path(root) / VERSION
        cell_d, self.n_files = tree_digest(cell_dir)
        self.cell_digest = cell_d
        self.code = code_digest(repo, kernels)
        self.envd = env_digest(env)
        # one directory per (cell content, code, env): a cell's cache ships as one folder
        self.key = _sha(f"{cell_d}|{self.code}|{self.envd}".encode())
        self.dir = self.root / self.key[:2] / self.key
        # results reusable from cells with the same robot side (other obstacles / welds), after revalidate
        self.geo = _sha(f"{geo_digest(cell_dir, cell)}|{self.code}|{self.envd}".encode())
        self.similar_dir = self.root / "similar" / self.geo[:2] / self.geo
        # the cell's obstacles: a failed result from a similar cell is reused only with the same obstacles
        obst = Path(cell_dir) / "agentic_planner" / "config" / "obstacles.yaml"
        self.obst = _sha(obst.read_bytes()) if obst.is_file() else "none"
        self.neg_ttl_s = float(os.environ.get("U3_CACHE_NEG_TTL_S", 7 * 86400) if neg_ttl_s is None else neg_ttl_s)
        self.lock = threading.Lock()
        self.stats = {"hits": 0, "misses": 0, "stores": 0, "expired": 0, "errors": 0, "similar_tried": 0,
                      "similar_reused": 0, "splice_tried": 0, "splice_used": 0, "by_fn": {}}
        self.digest_s = round(time.monotonic() - t0, 3)
        self.log = log or (lambda m: None)

    def info(self) -> dict:
        return {"dir": str(self.dir), "key": self.key, "similar_dir": str(self.similar_dir),
                "cell_digest": self.cell_digest, "cell_files": self.n_files,
                "code_digest": self.code[:16], "env_digest": self.envd[:16], "digest_s": self.digest_s,
                **self.stats}

    def task_key(self, fn_name: str, args: tuple) -> str:
        # the first argument of every worker task is the cell id: replaced by the cell's content (the cache dir)
        a = list(args)
        if a and a[0] == self.cell:
            a[0] = "<cell>"
        return _sha(f"{fn_name}|{canon(a)}".encode())

    def splice_candidate(self, fn_name: str, args: tuple) -> Optional[str]:
        """A planned leg of the same arm into the same weld request (other start): this cell's, else a similar
        cell's. Opt-in (``U3_CACHE_SPLICE=1``): it re-used 2-4 legs per body_weld_3 run for no measurable gain, and
        one of the runs that used it found no assembly (2026-11-04, not attributed yet)."""
        if os.environ.get("U3_CACHE_SPLICE", "0") != "1":
            return None
        wk = weld_key(fn_name, args)
        if not wk:
            return None
        for d in (self.dir, self.similar_dir):
            p = d / f"{wk}.pkl"
            if p.is_file():
                return str(p)
        return None

    def similar_candidate(self, fn_name: str, key: str) -> Optional[str]:
        """The path of a result of this task planned in a cell with the same robot side, or None."""
        if fn_name not in SIMILAR:
            return None
        p = self.similar_dir / f"{key}.pkl"
        return str(p) if p.is_file() else None

    def _count(self, fn_name: str, what: str) -> None:
        with self.lock:
            self.stats[what] += 1
            d = self.stats["by_fn"].setdefault(fn_name, {"hits": 0, "misses": 0, "stores": 0, "similar_tried": 0,
                                                         "similar_reused": 0, "splice_tried": 0,
                                                         "splice_used": 0})
            if what in d:
                d[what] += 1

    def get(self, fn_name: str, key: str):
        """(True, result) on a hit, (False, None) on a miss."""
        if fn_name in NEVER:
            return False, None
        p = self.dir / f"{key}.pkl"
        try:
            with open(p, "rb") as fh:
                ent = pickle.load(fh)
        except FileNotFoundError:
            self._count(fn_name, "misses")
            return False, None
        except Exception:  # noqa: BLE001 -- a corrupt entry is a miss (and is replaced)
            self._count(fn_name, "errors")
            self._count(fn_name, "misses")
            return False, None
        if ent.get("expires") and time.time() > ent["expires"]:
            self._count(fn_name, "expired")
            self._count(fn_name, "misses")
            return False, None
        self._count(fn_name, "hits")
        return True, ent["result"]

    def put(self, fn_name: str, key: str, result: Any) -> None:
        if fn_name in NEVER:
            return
        failed = fn_name in PLANNING and isinstance(result, dict) and not result.get("ok")
        ent = {"fn": fn_name, "t": time.time(), "result": result,
               "expires": (time.time() + self.neg_ttl_s) if failed else None}
        try:
            self.dir.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=self.dir, prefix=".u3-", suffix=".tmp")
            with os.fdopen(fd, "wb") as fh:
                pickle.dump(ent, fh, protocol=pickle.HIGHEST_PROTOCOL)
            os.replace(tmp, self.dir / f"{key}.pkl")
            self._count(fn_name, "stores")
        except Exception as exc:  # noqa: BLE001 -- the cache never fails a plan
            self._count(fn_name, "errors")
            self.log(f"[uf] plan cache: could not store {fn_name} ({exc!r})")

    def read_journal(self) -> list:
        """The task keys in the order the last successful run of this cell used their results (journal.json)."""
        try:
            return list(json.loads((self.dir / "journal.json").read_text()).get("keys") or [])
        except (OSError, ValueError, AttributeError):
            return []

    def read_journal_dropped(self) -> list:
        """The task keys the last successful run of this cell dropped at a deadline (journal.json ``dropped``)."""
        try:
            return list(json.loads((self.dir / "journal.json").read_text()).get("dropped") or [])
        except (OSError, ValueError, AttributeError):
            return []

    def write_journal(self, keys: list, dropped: Optional[list] = None) -> None:
        try:
            self.dir.mkdir(parents=True, exist_ok=True)
            tmp = self.dir / ".u3-journal.tmp"
            tmp.write_text(json.dumps({"keys": list(keys), "dropped": list(dropped or [])}))
            os.replace(tmp, self.dir / "journal.json")
        except OSError:
            pass

    def write_meta(self, extra: Optional[dict] = None) -> None:
        try:
            self.dir.mkdir(parents=True, exist_ok=True)
            meta = {"cell": self.cell, "cell_digest": self.cell_digest, "code_digest": self.code,
                    "env_digest": self.envd, "updated": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    # the similar-cell entries this cell's runs wrote (shipped with it: they serve OTHER cells)
                    "similar_rel": str(self.similar_dir.relative_to(self.root)),
                    **(extra or {})}
            tmp = self.dir / ".u3-meta.tmp"
            tmp.write_text(json.dumps(meta, indent=1))
            os.replace(tmp, self.dir / "meta.json")
        except OSError:
            pass


def _store(d: Path, key: str, ent: dict) -> None:
    d.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=d, prefix=".u3-", suffix=".tmp")
    with os.fdopen(fd, "wb") as fh:
        pickle.dump(ent, fh, protocol=pickle.HIGHEST_PROTOCOL)
    os.replace(tmp, d / f"{key}.pkl")


def reusable(fn_name: str, res: Any) -> bool:
    """A result worth offering to similar cells: a planned chain / coupled step, an accepted gate (a refusal or a
    failed plan may be the obstacles' doing, so it is recomputed there)."""
    if not isinstance(res, dict):
        return False
    if fn_name in ("plan_chain", "plan_coupled"):
        return bool(res.get("ok"))
    if fn_name == "gate_step":
        return res.get("mode") is not None
    if fn_name == "gate_many":
        return bool(res.get("modes")) and all(m is not None for m in res["modes"])
    return False


def _legs_of(fn_name: str, args: tuple, res: dict) -> list:
    """(robot ns, trajectory) of every leg a result stands on: the planned legs, the gated legs and parked arms."""
    from agents.trajectory.tools.behaviour_tree import _parked
    if fn_name in ("plan_chain", "plan_coupled"):
        return [(r["ns"], r["traj"]) for r in res.get("legs") or [] if r.get("traj")]
    checks = [(args[1], args[3] if len(args) > 3 else None)] if fn_name == "gate_step" else \
        [(c[0], c[2] if len(c) > 2 else None) for c in args[1]]
    out = []
    for legs, bys in checks:
        out += [(ns, t) for ns, t in (legs or {}).items() if t]
        out += [(ns, _parked(q)) for ns, q in (bys or {}).items() if q and ns not in (legs or {})]
    return out


def revalidate(fn_name: str, args: tuple, res: dict) -> bool:
    """Whether a result planned in a similar cell holds in this one: every leg it stands on passes umarell's own
    single-arm leg check here (``leg_verdict.refusal(static=True)``: joint box, the arm against itself, the tool out of
    the workpiece / fixture boxes away from this cell's welds, the static obstacles at the runtime margins) -- the
    check every gate and the assembler apply to every leg. The arm-vs-arm part of a gate depends only on the robot
    side, which the similar key holds. Any error: not valid."""
    try:
        from agents.trajectory.tools import leg_verdict as lv
        cell = args[0]
        legs = _legs_of(fn_name, args, res)
        return bool(legs) and all(lv.refusal(cell, ns, traj, static=True) is None for ns, traj in legs)
    except Exception:  # noqa: BLE001
        return False


def weld_key(fn_name: str, args: tuple) -> Optional[str]:
    """For a single weld leg (plan_chain of one oriented weld request), its key WITHOUT the start / chaining state:
    the same arm into the same weld request with the same planner settings (:func:`splice_or_run`)."""
    if fn_name != "plan_chain" or len(args) < 3 or len(args[1]) != 1:
        return None
    spec = args[1][0]
    req = spec.get("req") or {}
    if req.get("home_return") or req.get("stage_for") or not (req.get("constrain_orientation") and
                                                              req.get("orientation")):
        return None
    return "w-" + _sha(f"plan_chain_weld|{canon([spec.get('ns'), spec.get('op_id'), req, args[2]])}".encode())


def _store_weld(dirs: list, fn_name: str, args: tuple, res: Any) -> None:
    """A planned single weld leg, indexed by :func:`weld_key` (the latest one) in ``dirs``."""
    wk = weld_key(fn_name, args)
    if not wk or not isinstance(res, dict) or not res.get("ok"):
        return
    rec = (res.get("legs") or [None])[0]
    if not rec or not rec.get("traj"):
        return
    for d in dirs:
        if d:
            _store(Path(d), wk, {"fn": "weld_leg", "t": time.time(), "result": rec, "expires": None})


def run_and_store(cache_dir: str, key: str, fn_name: str, neg_ttl_s: float, fn, *args, similar_dir: str = "",
                  obst: str = ""):
    """Worker side of a cache miss: run the task and store its result from the worker (no copy through the parent,
    which may already be using the result), also for similar cells (with the obstacles' digest); return it."""
    res = fn(*args)
    try:
        failed = fn_name in PLANNING and isinstance(res, dict) and not res.get("ok")
        ent = {"fn": fn_name, "t": time.time(), "result": res, "expires": (time.time() + neg_ttl_s) if failed else None,
               "obst": obst}
        _store(Path(cache_dir), key, ent)
        if similar_dir and fn_name in SIMILAR:
            if not reusable(fn_name, res) and ent["expires"] is None:
                ent = dict(ent, expires=time.time() + neg_ttl_s)    # a refused gate: as a failed plan elsewhere
            _store(Path(similar_dir), key, ent)
        _store_weld([cache_dir, similar_dir], fn_name, args, res)
    except Exception:  # noqa: BLE001 -- the cache never fails a task
        pass
    return res


def reuse_or_run(candidate: str, cache_dir: str, key: str, fn_name: str, neg_ttl_s: float, fn, *args,
                 similar_dir: str = "", obst: str = ""):
    """Worker side of a similar-cell candidate (stored as this cell's own entry when used, marked ``_cache``):

    * a planned / accepted result: used when every leg passes :func:`revalidate` in this cell;
    * a failed plan / refused gate: used, until it expires, when this cell has the same obstacles (only its weld
      programs differ; reusing a failure never makes a plan unsafe -- at worst a leg that might now plan is
      recovered or reallocated as it was there);
    else the task runs as on a miss."""
    try:
        with open(candidate, "rb") as fh:
            ent = pickle.load(fh)
        res = ent["result"]
        use = None
        if reusable(fn_name, res):
            if revalidate(fn_name, args, res):
                use = "similar"
        elif obst and ent.get("obst") == obst and not (ent.get("expires") and time.time() > ent["expires"]):
            use = "similar_failure"
        if use:
            res = dict(res, _cache=use)
            try:
                _store(Path(cache_dir), key, {"fn": fn_name, "t": time.time(), "result": res, "obst": obst,
                                              "expires": ent.get("expires") if use == "similar_failure" else None})
            except Exception:  # noqa: BLE001
                pass
            return res
    except Exception:  # noqa: BLE001 -- an unreadable candidate: plan it
        pass
    return run_and_store(cache_dir, key, fn_name, neg_ttl_s, fn, *args, similar_dir=similar_dir, obst=obst)


def splice_or_run(candidate: str, cache_dir: str, key: str, fn_name: str, neg_ttl_s: float, fn, *args,
                  similar_dir: str = "", obst: str = ""):
    """Worker side of a weld leg whose exact key missed but whose :func:`weld_key` has a planned leg (this cell or a
    similar one, from another start): ``workers.splice_weld_leg`` (the cached approach + a newly planned transit,
    re-checked); stored as this task's entry (``_cache: splice``). Else the task runs as on a miss."""
    try:
        from umarell_fast import workers as W
        with open(candidate, "rb") as fh:
            cached = pickle.load(fh)["result"]
        rec = W.splice_weld_leg(args[0], cached, args[1][0], args[2])
        if rec is not None:
            res = {"legs": [rec], "ok": True, "wall_s": rec.get("wall_s"), "pid": os.getpid(), "_cache": "splice"}
            try:
                _store(Path(cache_dir), key, {"fn": fn_name, "t": time.time(), "result": res, "obst": obst,
                                              "expires": None})
            except Exception:  # noqa: BLE001
                pass
            return res
    except Exception:  # noqa: BLE001 -- the splice is an optimisation: plan the leg
        pass
    return run_and_store(cache_dir, key, fn_name, neg_ttl_s, fn, *args, similar_dir=similar_dir, obst=obst)


# ─── shipping a cell's cache (pre-populating another host) ──────────────────────────────────────────────────────

def export_dirs(root: Path, cell: Optional[str], out: Path) -> int:
    """Tar every cache directory (of ``cell``, or all) under ``root``, with the similar-cell entries its runs wrote
    and btexport's check cache; returns the number of cell directories."""
    base = Path(root) / VERSION
    n = 0
    added: set = set()
    with tarfile.open(out, "w:gz") as tar:
        for meta in sorted(base.glob("*/*/meta.json")):
            try:
                m = json.loads(meta.read_text())
            except (OSError, ValueError):
                continue
            if cell and m.get("cell") != cell:
                continue
            tar.add(meta.parent, arcname=str(meta.parent.relative_to(Path(root))))
            n += 1
            sim = base / str(m.get("similar_rel") or "")
            if m.get("similar_rel") and sim.is_dir() and str(sim) not in added:
                tar.add(sim, arcname=f"{VERSION}/{m['similar_rel']}")
                added.add(str(sim))
        # btexport's dense check cache next to it (tools/btexport/dense.py, BTEXPORT_CHECK_CACHE_DIR: the router's
        # <cache>/btexport-check), small (a few KB per checked plan): shipped whole
        chk = Path(root).parent / "btexport-check"
        if chk.is_dir():
            for f in sorted(chk.glob("*.json")):
                tar.add(f, arcname=f"btexport-check/{f.name}")
    return n


def import_tar(root: Path, src: Path) -> int:
    """Unpack a tar from :func:`export_dirs`: the task cache into ``root``, btexport's check cache next to it
    (entries already there are kept). Returns the number of files written."""
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    n = 0
    with tarfile.open(src, "r:*") as tar:
        for m in tar.getmembers():
            if ".." in Path(m.name).parts or not (m.isfile() or m.isdir()):
                continue
            if m.name.startswith("btexport-check/"):
                base = root.parent                     # the check cache lives next to the task cache
            elif m.name.startswith(VERSION + "/"):
                base = root
            else:
                continue
            if m.isfile() and (base / m.name).exists():
                continue
            base.mkdir(parents=True, exist_ok=True)
            tar.extract(m, base, filter="data") if sys.version_info >= (3, 12) else tar.extract(m, base)
            n += m.isfile()
    return n


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m umarell_fast.plan_cache")
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("export", help="tar a cell's cache directories (pre-populate another host)")
    e.add_argument("--cell")
    e.add_argument("--out", required=True)
    i = sub.add_parser("import", help="unpack a tar from export into the cache")
    i.add_argument("tar")
    sub.add_parser("ls", help="list the cache directories")
    ap.add_argument("--root", default=None, help="cache root (default $U3_CACHE_DIR)")
    a = ap.parse_args(argv)
    root = Path(a.root) if a.root else cache_root()
    if a.cmd == "export":
        print(f"{export_dirs(root, a.cell, Path(a.out))} cache dir(s) -> {a.out}")
    elif a.cmd == "import":
        print(f"{import_tar(root, Path(a.tar))} file(s) imported into {root}")
    else:
        for meta in sorted((root / VERSION).glob("*/*/meta.json")):
            m = json.loads(meta.read_text())
            n = sum(1 for _ in meta.parent.glob("*.pkl"))
            print(f"{meta.parent.name[:12]}  {m.get('cell'):<20} {n:>5} entries  updated {m.get('updated')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
