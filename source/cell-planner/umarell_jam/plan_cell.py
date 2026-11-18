"""``plan-cell``: one FactoryOS planning job (the planner router's umarell_jam planner).

    python -m umarell_jam --root R --workspace W [--umarell <plain umarell 8a02db9 / 76ea13a checkout>] plan-cell \
        --cell-id ID [--task-file F.json] [--welds W01 …] [--robots robot1 …] [--speed max|normal] \
        [--timeout S] [--repair-budget S] [--prepare-budget S] [--job JOB] --out PLAN.json

``R/runs/<job>/``: ``--job``, else ``$UMARELL_JAM_JOB``, else ``<cell>-<UTC time>-<pid>`` (printed first).

1. A weld-request-v1 from the cell's task program: ``--task-file`` (it must be one of the cell's own programs,
   ``W/cells/<ID>/agentic_planner/tasks/*.json``, byte-for-byte as JSON: component keys follow the cell's own
   programs), else the cell's single program, else invalid_request. Every weld / every installed robot unless
   ``--welds`` / ``--robots``; initial ``configured_home``, terminal ``home``, ``speed_profile`` ``cell_limits`` (the
   planner's only profile: its certificates are tied to the cell's limits; ``--speed`` is recorded, both values plan
   at the cell limits).
2. ``R/umarell`` created or refreshed if missing or stale (:mod:`.native_copy`, ``R/.setup.lock``; shared while
   the job plans).
3. ``PlannerService.plan`` (``--timeout``, ``--repair-budget``) and ``commit``. When certified primitives are missing
   and ``--prepare-budget`` > 0: the bounded, isolated native repair/preparation of exactly the missing welds for the
   enabled robots (``repair.repair``: cached seeds, then native fallback), then one more plan.
4. ``PLAN.json``: umarell's plan-store envelope (``plan_store.save_plan``: tree_id, cell_id, created_s, status
   "ok", num_operations, behaviour_tree) holding the tree converted for plain umarell / btexport
   (:func:`to_umarell_tree`); the native tree stays published in ``R/plans/bundles/<version>/``.
   ``PLAN.json.result.json``: {status, reason, timings, cache: {hits, misses}, weld_ids, robots, stack, …}
   (``stack``: the numerical library stack, :func:`umarell_jam.stack.fingerprint`). Without toppra (and without
   ``--allow-stack-mismatch``) ``--repair-budget`` / ``--prepare-budget`` are clamped to 0: the job plans
   cache-only, a miss is the ordinary missing_certified_primitives (exit 3), and the result carries
   ``stack_guard`` "repair disabled: toppra missing" (:func:`umarell_jam.stack.repair_guard`).
5. ``R/runs/<job>/progress.json`` rewritten at least every :data:`HEARTBEAT_S` seconds while the job runs.

Exit codes: 0 success, 2 invalid request, 3 no complete plan (unknown / infeasible / timeout), 4 setup failed.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import threading
import time
import traceback

HEARTBEAT_S = 15.0
EXIT = {"success": 0, "invalid_request": 2, "infeasible": 3, "timeout_or_unknown": 3, "setup_failed": 4}
SPEEDS = ("max", "normal")
# ports of the native (integration-hook) executor that plain umarell / btexport do not know
NATIVE_PORTS = ("controller_exact", "independent_clocks", "initial_velocity")


def _atomic(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    tmp.write_text(json.dumps(value, indent=1, default=str))
    os.replace(tmp, path)


class Progress:
    """``progress.json`` of a job: written on every phase change and by a heartbeat thread."""

    def __init__(self, path: Path, **info):
        self.path, self.info = Path(path), dict(info)
        self.started = time.time()
        self.phase, self.beats, self.detail = "starting", 0, {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.write()
        self._thread = threading.Thread(target=self._beat, name="umarell-jam-progress", daemon=True)
        self._thread.start()

    def _beat(self):
        while not self._stop.wait(HEARTBEAT_S):
            self.write()

    def set(self, phase: str, **detail):
        with self._lock:
            self.phase, self.detail = phase, detail
        self.write()

    def write(self, **extra):
        with self._lock:
            self.beats += 1
            now = time.time()
            doc = {**self.info, "pid": os.getpid(), "phase": self.phase, "detail": self.detail,
                   "started_s": self.started, "updated_s": now, "elapsed_s": round(now - self.started, 3),
                   "heartbeat_s": HEARTBEAT_S, "beats": self.beats, **extra}
            try:
                _atomic(self.path, doc)
            except OSError:
                pass

    def close(self, **final):
        self._stop.set()
        self._thread.join(timeout=2)
        with self._lock:
            self.phase = "done"
        self.write(**final)


# ─── request ──────────────────────────────────────────────────────────────────

class RequestError(ValueError):
    pass


def program_for(cell, task_file: Path | None) -> str:
    if task_file is None:
        if len(cell.programs) != 1:
            raise RequestError(f"cell {cell.cell_id} has {len(cell.programs)} task programs "
                               f"({sorted(cell.programs)}): --task-file is required")
        return next(iter(cell.programs))
    try:
        doc = json.loads(Path(task_file).read_text())
    except (OSError, ValueError) as exc:
        raise RequestError(f"cannot read --task-file {task_file}: {exc}") from exc
    pid = doc.get("program") if isinstance(doc, dict) else None
    if pid not in cell.programs:
        raise RequestError(f"--task-file program {pid!r} is not one of cell {cell.cell_id}'s task programs "
                           f"{sorted(cell.programs)} (the planner plans the cell's own programs)")
    if cell.programs[pid] != doc:
        raise RequestError(f"--task-file differs from cell {cell.cell_id}'s program {pid!r} "
                           "(agentic_planner/tasks): plan the cell's own program")
    return pid


def build_request(cell, job: str, task_file=None, welds=None, robots=None) -> dict:
    pid = program_for(cell, task_file)
    return {"schema_version": "weld-request-v1", "request_id": job, "cell_id": cell.cell_id,
            "program_id": pid,
            "weld_ids": list(welds) if welds else [o["id"] for o in cell.programs[pid]["operations"]],
            "enabled_robot_ids": list(robots) if robots else list(cell.robots),
            "completed_weld_ids": [], "initial_state": {"mode": "configured_home"},
            "terminal_state": "home", "speed_profile": "cell_limits"}


# ─── plan conversion ──────────────────────────────────────────────────────────

DENSE_DT_S = 0.02


def densify(points: list, max_dt: float = DENSE_DT_S) -> list:
    """The same controller curve with no waypoint gap above ``max_dt``: every original waypoint unchanged, samples of
    the exact quintic controller interpolant (positions, velocities, accelerations) in between. The native
    executor interpolates the waypoints quintically (``controller_exact``); a joint_trajectory_controller without
    accelerations, umarell's executor and btexport's checks interpolate linearly, which on the native few-knot
    transfers (seconds between knots) would be another motion."""
    import math
    from .trajectory import Trajectory
    if len(points) < 2 or not all("velocities" in p and "accelerations" in p for p in points):
        return points
    curve = Trajectory(points)
    out = [points[0]]
    for a, b in zip(points, points[1:]):
        t0, t1 = float(a["time_s"]), float(b["time_s"])
        n = math.ceil((t1 - t0) / max_dt - 1e-9)
        for j in range(1, n):
            t = t0 + (t1 - t0) * j / n
            out.append({**{k: v for k, v in a.items() if k not in ("positions", "velocities", "accelerations",
                                                                     "time_s")},
                        "positions": curve.sample(t).tolist(), "velocities": curve.sample(t, 1).tolist(),
                        "accelerations": curve.sample(t, 2).tolist(), "time_s": t})
        out.append(b)
    return out


def to_umarell_tree(tree: dict, max_dt: float = DENSE_DT_S) -> dict:
    """The native tree as plain umarell's executor and btexport run it (same motions, same timing).

    * each weld's ``contact`` ExecuteTrajectory becomes the weld leg: ``operation_id`` the weld id,
      ``operation_type`` "weld", ``hold_duration_s`` the sum of the weld's squeeze / weld / hold ``WeldPhase``
      dwells (which follow the contact motion at rest), and the WeldPhase nodes are removed;
    * ``WeldBoundary`` nodes (the native rest-boundary acknowledgements, no motion) are removed;
    * the native executor's ports (:data:`NATIVE_PORTS`) are dropped; every other port, the tree shape
      (Sequence / Parallel) and the metadata are kept;
    * every trajectory is :func:`densify`-ed to waypoint gaps of at most ``max_dt`` (the same curve).
    """
    import copy
    tree = copy.deepcopy(tree)
    folded = {"welds": 0, "weld_phases": 0, "boundaries": 0}

    def convert(node):
        kids = node.get("children")
        if isinstance(kids, list):
            out, contact = [], None
            for child in kids:
                kind = child.get("type")
                if kind == "WeldBoundary":
                    folded["boundaries"] += 1
                    continue
                if kind == "WeldPhase":
                    p = child.get("ports") or {}
                    if contact is None or (contact["ports"].get("robot_ns") != p.get("robot_ns")):
                        raise ValueError(f"WeldPhase {child.get('id')} does not follow its weld's contact motion")
                    contact["ports"]["hold_duration_s"] = float(contact["ports"].get("hold_duration_s") or 0.) \
                        + float(p.get("duration_s") or 0.)
                    contact["ports"]["operation_id"] = p.get("weld_id")
                    contact["ports"]["operation_type"] = "weld"
                    folded["weld_phases"] += 1
                    continue
                convert(child)
                ports = child.get("ports")
                if child.get("type") == "ExecuteTrajectory" and isinstance(ports, dict):
                    for name in NATIVE_PORTS:
                        ports.pop(name, None)
                    contact = child if str(ports.get("operation_id", "")).endswith(":contact") else None
                    if contact is not None:
                        folded["welds"] += 1
                else:
                    contact = None
                out.append(child)
            node["children"] = out
        return node

    convert(tree["root"])
    dense = 0
    for key, points in list(tree.get("blackboard", {}).items()):
        if isinstance(points, list) and points and isinstance(points[0], dict) and "positions" in points[0]:
            tree["blackboard"][key] = densify(points, max_dt)
            dense += len(tree["blackboard"][key]) - len(points)
    folded["densified_waypoints_added"] = dense
    if any(str((n.get("ports") or {}).get("operation_id", "")).endswith(":contact")
           for n in _leaves(tree["root"])):
        raise ValueError("a contact motion without its weld process phases")
    md = tree.setdefault("metadata", {})
    md["generator"] = "umarell_jam"
    md["umarell_jam_conversion"] = {
        "from": "native weld-request-v1 tree (umarell_jam integration hooks)", **folded,
        "rule": "contact motion + squeeze/weld/hold dwells = one weld ExecuteTrajectory (hold = dwell sum); "
                "WeldBoundary rest acknowledgements removed; native executor ports dropped; waypoints densified "
                f"to <= {max_dt} s on the exact quintic controller curve"}
    return tree


def _leaves(node):
    kids = node.get("children") or []
    if not kids:
        return [node]
    return [leaf for c in kids for leaf in _leaves(c)]


def envelope(tree: dict) -> dict:
    """umarell's plan-store record (agents/trajectory/tools/plan_store.save_plan)."""
    try:
        from agents.trajectory.tools.plan_store import STATUS_OK, _count_execute
        status, count = STATUS_OK, _count_execute(tree)
    except Exception:  # noqa: BLE001 - the same record without the umarell copy on sys.path
        status = "ok"
        count = sum(1 for leaf in _leaves(tree["root"]) if leaf.get("type") == "ExecuteTrajectory")
    return {"tree_id": tree["tree_id"], "cell_id": tree["cell_id"], "created_s": time.time(), "status": status,
            "num_operations": count, "behaviour_tree": tree}


# ─── the job ──────────────────────────────────────────────────────────────────

def job_id(cell_id: str) -> str:
    return f"{cell_id}-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}-{os.getpid()}"


def run(layout, args, umarell_source=None) -> int:
    t0 = time.monotonic()
    out = Path(args.out).resolve()
    result_path = out.with_name(out.name + ".result.json")
    job = args.job or os.environ.get("UMARELL_JAM_JOB", "").strip() or job_id(args.cell_id)
    if "/" in job or job in {".", ".."}:
        job = job_id(args.cell_id)
    job_dir = layout.runs / job
    job_dir.mkdir(parents=True, exist_ok=True)
    progress = Progress(job_dir / "progress.json", job=job, cell_id=args.cell_id, out=str(out),
                        result=str(result_path))
    print(f"[umarell_jam] job {job} progress {job_dir / 'progress.json'}", flush=True)
    result = {"status": "timeout_or_unknown", "reason": None, "timings": {}, "cache": {"hits": 0, "misses": 0},
              "weld_ids": list(args.welds or []), "robots": list(args.robots or []), "cell_id": args.cell_id,
              "job": job, "job_dir": str(job_dir), "plan": None, "speed": args.speed,
              "speed_profile": "cell_limits"}
    try:
        from .stack import fingerprint
        result["stack"] = fingerprint()
    except Exception as exc:  # noqa: BLE001 - provenance only, never a reason to fail the job
        result["stack"] = {"error": f"{type(exc).__name__}: {exc}"}
    timings = result["timings"]

    def finish(status, reason=None, **extra):
        result.update(status=status, reason=reason, **extra)
        timings["total_s"] = round(time.monotonic() - t0, 3)
        result["exit_code"] = EXIT.get(status, 3)
        _atomic(result_path, result)
        _atomic(job_dir / "result.json", result)
        progress.close(status=status, exit_code=result["exit_code"])
        print(f"[umarell_jam] {status}" + (f": {reason}" if reason else "") + f" ({timings['total_s']} s)",
              flush=True)
        return result["exit_code"]

    if out.exists():
        out.unlink()
    if args.speed not in SPEEDS:
        return finish("invalid_request", f"--speed {args.speed!r} (one of {list(SPEEDS)})")
    for name in ("timeout", "repair_budget", "prepare_budget"):
        v = getattr(args, name)
        if not (v == v and v >= 0 and v != float("inf")):
            return finish("invalid_request", f"--{name.replace('_', '-')} must be finite and >= 0")
    try:
        from .stack import repair_guard
        guard = repair_guard({"--repair-budget": args.repair_budget, "--prepare-budget": args.prepare_budget},
                             getattr(args, "allow_stack_mismatch", False),
                             None if "error" in result["stack"] else result["stack"])
    except Exception as exc:  # noqa: BLE001 - an unreadable stack must not enable writes silently
        guard = f"repair disabled: stack unreadable ({type(exc).__name__})"
        print(f"[umarell_jam] {guard}: repair / prepare budgets clamped to 0", file=sys.stderr, flush=True)
    if guard:
        result["stack_guard"] = guard
        args.repair_budget = args.prepare_budget = 0.
    from . import native_copy
    progress.set("setup")
    t = time.monotonic()
    try:
        ctx = native_copy.ready(layout, umarell_source, log=lambda m: print(m, flush=True))
        stamp = ctx.__enter__()
    except Exception as exc:  # noqa: BLE001 - every setup failure is exit 4
        timings["setup_s"] = round(time.monotonic() - t, 3)
        return finish("setup_failed", f"{type(exc).__name__}: {exc}")
    timings["setup_s"] = round(time.monotonic() - t, 3)
    try:
        return _plan(layout, args, out, result, timings, progress, finish, job, job_dir, stamp)
    except Exception as exc:  # noqa: BLE001
        (job_dir / "traceback.txt").write_text(traceback.format_exc())
        return finish("timeout_or_unknown", f"{type(exc).__name__}: {exc}")
    finally:
        ctx.__exit__(None, None, None)


def _plan(layout, args, out, result, timings, progress, finish, job, job_dir, stamp) -> int:
    layout.activate()
    progress.set("load")
    t = time.monotonic()
    try:
        from .service import PlannerService
        from .cell import CellLoadError
        from .request import InvalidRequest, Request
    except Exception as exc:  # noqa: BLE001 - the numerical runtime itself is missing
        return finish("setup_failed", f"cannot import the planner runtime: {type(exc).__name__}: {exc}")
    service = PlannerService(layout.workspace, layout.components, layout.umarell, layout.plans)
    try:
        cell = service.load(args.cell_id)
        raw = build_request(cell, job, Path(args.task_file) if args.task_file else None, args.welds, args.robots)
        Request.parse(raw, cell)
    except (CellLoadError, InvalidRequest, RequestError, OSError, KeyError, TypeError) as exc:
        return finish("invalid_request", f"{type(exc).__name__}: {exc}")
    _atomic(job_dir / "request.json", raw)
    result.update(weld_ids=raw["weld_ids"], robots=raw["enabled_robot_ids"], program_id=raw["program_id"],
                  scene_revision=cell.geometry_hash, umarell_copy={k: stamp.get(k) for k in ("base", "fix_set",
                  "source", "created")})
    timings["load_s"] = round(time.monotonic() - t, 3)
    attempts = []

    def plan(label):
        progress.set(label, timeout_s=args.timeout, repair_budget_s=args.repair_budget)
        t = time.monotonic()
        res = service.plan(raw, timeout_s=args.timeout, repair_budget_s=args.repair_budget)
        timings[f"{label}_s"] = round(time.monotonic() - t, 3)
        acc = res.get("cache_accesses") or {}
        result["cache"]["hits"] += acc.get("hits", 0)
        result["cache"]["misses"] += acc.get("misses", 0)
        attempts.append({"label": label, "status": res.get("status"), "reason": res.get("reason"),
                         "missing_weld_ids": res.get("missing_weld_ids"), "latency": res.get("latency"),
                         "repair": res.get("repair")})
        return res

    res = plan("plan")
    if (res.get("status") == "timeout_or_unknown" and res.get("reason") == "missing_certified_primitives"
            and args.prepare_budget > 0):
        missing = list(res.get("missing_weld_ids") or [])
        progress.set("prepare", missing_weld_ids=missing, budget_s=args.prepare_budget)
        t = time.monotonic()
        from .repair import repair
        prepared = repair(service, Request.parse(raw, cell), missing, args.prepare_budget)
        timings["prepare_s"] = round(time.monotonic() - t, 3)
        result["prepare"] = {k: prepared.get(k) for k in ("status", "wall_s", "budget_s", "affected_weld_ids",
                                                          "log", "receipt")}
        res = plan("replan")
    result["attempts"] = attempts
    status = res.get("status")
    if status != "success":
        return finish(status if status in EXIT else "timeout_or_unknown", res.get("reason"),
                      missing_weld_ids=res.get("missing_weld_ids"), cycle=res.get("timing"))
    progress.set("commit")
    try:
        service.commit(res)
    except InvalidRequest as exc:
        return finish("timeout_or_unknown", f"commit refused: {exc}")
    tree = to_umarell_tree(res["behaviour_tree"])
    _atomic(out, envelope(tree))
    timings["cycle"] = res.get("timing")
    return finish("success", None, plan=str(out), version_hash=res["version_hash"],
                  native_bundle=str(Path(res["bt_json"]).parent), assignments=res.get("assignments"),
                  cycle=res.get("timing"), latency=res.get("latency"))
