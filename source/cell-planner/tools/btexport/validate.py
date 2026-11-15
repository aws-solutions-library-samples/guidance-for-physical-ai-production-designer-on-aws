"""Project-venv side of the umarell validation: prepare a validation directory, make the scratch umarell repo, run
``tools.btexport.umarell_check`` in .venv-umarell, return its report."""
from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
UMARELL_PY = ROOT / ".venv-umarell" / "bin" / "python"
DEFAULT_REPO = ROOT / "data" / "btexport" / "umarell_repo"
SOURCE_REPO = ROOT / "third_party" / "sample-umarell"


class _Lock:
    """Exclusive inter-process file lock (fcntl), so parallel matrix workers do not race on the scratch repo."""

    def __init__(self, path: Path):
        self.path = Path(path)

    def __enter__(self):
        import fcntl
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(self.path, "w")
        fcntl.flock(self.f, fcntl.LOCK_EX)
        return self

    def __exit__(self, *a):
        import fcntl
        fcntl.flock(self.f, fcntl.LOCK_UN)
        self.f.close()


def source_repo() -> Path:
    """The umarell repo in effect for this process: env.cell.UMARELL (the UMARELL_REPO the controllers, the exporter
    and the dense check read; default third_party/sample-umarell)."""
    try:
        from env.cell import UMARELL
        return Path(UMARELL).resolve()
    except Exception:  # noqa: BLE001
        return Path(os.environ.get("UMARELL_REPO", SOURCE_REPO)).resolve()


def repo_fingerprint(source: Path) -> str:
    """Fingerprint of a umarell repo's cells/ tree: path, size and mtime of every file, plus the content md5 of the
    robot / cell / task files (*.urdf, *.srdf, *.yaml, *.json; small), so an in-place edit of a URDF, robots.yaml or
    task re-syncs the scratch copy even if the edit kept the mtime."""
    import hashlib
    h = hashlib.md5()
    h.update(str(Path(source).resolve()).encode())
    cells = Path(source) / "cells"
    for dp, dns, fns in os.walk(cells):
        dns[:] = sorted(d for d in dns if d not in ("__pycache__", ".git", "logs"))
        for fn in sorted(fns):
            f = Path(dp) / fn
            try:
                st = f.stat()
            except OSError:
                continue
            h.update(f"{f.relative_to(cells)}|{st.st_size}|{st.st_mtime_ns}".encode())
            if f.suffix in (".urdf", ".srdf", ".yaml", ".yml", ".json") and st.st_size < 5_000_000:
                h.update(hashlib.md5(f.read_bytes()).digest())
    return h.hexdigest()


def scratch_path(repo: Path, source: Path) -> Path:
    """The default scratch dir serves third_party/sample-umarell; another UMARELL_REPO gets its own scratch dir next
    to it (never mixed)."""
    repo = Path(repo)
    if repo.resolve() == DEFAULT_REPO.resolve() and Path(source).resolve() != SOURCE_REPO.resolve():
        import hashlib
        return repo.with_name(f"{repo.name}_{hashlib.md5(str(Path(source).resolve()).encode()).hexdigest()[:10]}")
    return repo


def ensure_repo(repo: Path = DEFAULT_REPO, source: Path | None = None) -> Path:
    """Scratch copy of the umarell repo in effect (``source``, default ``source_repo()`` = UMARELL_REPO /
    third_party/sample-umarell; the source is never written). Keyed on ``repo_fingerprint(source)``: re-synced
    (rsync, incremental) whenever a URDF, robots.yaml, task or any other cell file changed since the last copy
    (P3U round 2: a one-time copy of third_party judged plans for in-place-edited cells on the original robot).
    Locked: parallel workers wait for one copy instead of using a half-copied tree."""
    source = Path(source).resolve() if source is not None else source_repo()
    repo = scratch_path(repo, source)
    fp = repo_fingerprint(source)
    marker = repo / ".copy_complete"
    with _Lock(repo.parent / f".{repo.name}.lock"):
        cur = None
        if marker.exists():
            try:
                cur = json.loads(marker.read_text() or "{}")
            except ValueError:
                cur = {}
        if not cur or cur.get("source") != str(source) or cur.get("fingerprint") != fp:
            repo.parent.mkdir(parents=True, exist_ok=True)
            marker.unlink(missing_ok=True)
            # --checksum: an in-place edit that kept size and mtime is copied too (exported N-robot cells under
            # <repo>/cells/<scene cell> are not in the source and stay: no --delete). Another UMARELL_REPO is
            # overlaid on a copy of third_party's umarell (its code, e.g. agents/, may be missing from a copy that
            # holds only the edited cells); symlinks in it are copied as the files they point to (-L)
            ex = ["--exclude", ".git", "--exclude", "__pycache__"]
            if source != SOURCE_REPO.resolve():
                subprocess.run(["rsync", "-a", "--checksum", *ex, f"{SOURCE_REPO}/", f"{repo}/"], check=True)
                subprocess.run(["rsync", "-a", "-L", "--checksum", *ex, f"{source}/", f"{repo}/"], check=True)
            else:
                subprocess.run(["rsync", "-a", "--checksum", *ex, f"{source}/", f"{repo}/"], check=True)
            marker.write_text(json.dumps({"source": str(source), "fingerprint": fp,
                                          "synced_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}))
    return repo


def prepare(run: Path, plan: dict, src, legs=None) -> Path:
    """run/: plan.json, plan.xml, timeline.npz (replay, N-robot format), check.json, report.json (trajgen-style)."""
    import numpy as np
    from tools.btexport.core import executor_of, replay
    from trajgen.ntimeline import NTimeline
    run = Path(run)
    run.mkdir(parents=True, exist_ok=True)
    (run / "plan.json").write_text(json.dumps(plan))
    (run / "plan.xml").write_text(plan["behaviour_tree"]["xml"])
    if legs is None:
        legs, _ = replay(plan, src.robot_ids, src.home, executor_of(src))
    tl = NTimeline.from_legs(legs, robot_ids=src.robot_ids, dt=0.05, home=np.asarray(src.home, float))
    tl.save(run / "timeline.npz")
    info = {"cell_id": plan["cell_id"], "umarell_cell": src.umarell_cell if src.umarell_cell else None,
            "scene": str(src.scene) if src.scene else None, "task": str(src.task) if src.task else None,
            "robots": list(src.robot_ids)}
    if src.scene is not None and not src.umarell_cell:
        (run / "report.json").write_text(json.dumps({"scene": str(src.scene), "task": str(src.task), "status": "ok"}))
    (run / "check.json").write_text(json.dumps(info, indent=1))
    return run


def run_check(run: Path, *, repo: Path = DEFAULT_REPO, executor: bool = True, mesh_audit: bool = False,
              timeout: int = 7200, validators: bool = True, executor_xml: bool = False) -> dict:
    if not UMARELL_PY.exists():
        return {"skipped": "no .venv-umarell", "pass": None}
    src_repo = source_repo()
    repo = ensure_repo(repo, src_repo)
    cmd = [str(UMARELL_PY), "-m", "tools.btexport.umarell_check", "--run", str(run), "--repo", str(repo)]
    if not executor:
        cmd.append("--no-executor")
    if mesh_audit:
        cmd.append("--mesh-audit")
    if not validators:
        cmd.append("--no-validators")
    if executor_xml:
        cmd.append("--executor-xml")
    env = dict(os.environ)
    env.pop("KWR_ROBOT_OVERRIDE", None)          # umarell's side always uses the cell's own files
    env["PYTHONPATH"] = str(ROOT)
    env.pop("MUJOCO_GL", None)
    t0 = time.time()
    # exported N-robot cells (trajgen scenes) are re-created under <repo>/cells/<cell id> by every check of that
    # cell (rmtree + write): one check per exported cell at a time
    info = json.load(open(Path(run) / "check.json")) if (Path(run) / "check.json").exists() else {}
    lock = repo / f".cell_{info.get('cell_id')}.lock" if info.get("scene") and not info.get("umarell_cell") else None
    if lock is not None:
        with _Lock(lock):
            p = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True, timeout=timeout, env=env)
    else:
        p = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True, timeout=timeout, env=env)
    (Path(run) / "umarell_check.log").write_text(p.stdout[-50000:] + "\n--- stderr ---\n" + p.stderr[-50000:])
    out = Path(run) / "umarell_check.json"
    if not out.exists():
        return {"error": p.stderr[-3000:], "returncode": p.returncode, "pass": False}
    rep = json.load(open(out))
    rep["subprocess_wall_s"] = round(time.time() - t0, 1)
    rep["scratch_repo"] = {"path": str(repo), "source": str(src_repo)}
    return rep


def summary(rep: dict) -> dict:
    """Flat row of a umarell_check report."""
    if not rep or "pass_by_part" not in rep:
        return {"pass": rep.get("pass") if rep else None, "error": (rep or {}).get("error", "")[-300:]}
    v = rep.get("validators") or {}
    ex = rep.get("executor") or {}
    s = {"pass": rep["pass"], **{f"{k}_pass": x for k, x in rep["pass_by_part"].items()},
         "validate_behaviour_tree": rep["structure"].get("valid"),
         "xml_unknown_kinds": rep["xml"].get("unknown_node_kinds"),
         "inter_robot_clearance_min_m": v.get("inter_robot_clearance_min_m") or
         (v.get("inter_robot_clearance") or {}).get("min_clearance_m"),
         "validate_approach": v.get("validate_approach"), "obstacles_free": v.get("obstacles_free"),
         "per_step_or_leg": v.get("per_step_min_clearance_m") if "per_step_min_clearance_m" in v
         else v.get("validate_trajectory_set_per_leg"),
         "executor_success": (ex.get("result") or {}).get("success"), "executor_max_lag_s": ex.get("max_lag_s"),
         "executor_wall_s": ex.get("wall_s"),
         "dispatched_clearance_min_m": (ex.get("dispatched_motion_clearance") or {}).get("min_clearance_m"),
         "umarell_schedule_makespan_s": (rep.get("schedule") or {}).get("makespan_s")}
    if "mesh_self_collision_free" in v:
        s["mesh_self_collision_free"] = v["mesh_self_collision_free"]
    if rep.get("robot_model") is not None:
        s["robot_model_mismatches"] = rep["robot_model"].get("mismatches") or None
    for k in ("validators_skipped", "scratch_repo"):
        if rep.get(k) is not None:
            s[k] = rep[k]
    return s

