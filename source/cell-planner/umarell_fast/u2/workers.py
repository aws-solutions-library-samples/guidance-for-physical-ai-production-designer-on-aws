"""Worker processes: umarell's planner / assembler in parallel, with the arm state injected per task.

Why processes: umarell's collision checks are Python (GIL-bound), so threads do not plan in parallel
(docs/uf_profile.md §3). Each worker opens its own planning session on the same working copy and CELL_BASE_PATH
(``session.setup``), and every task brings the state it needs:

* ``plan`` task: the per-robot chaining state umarell keeps in ``trajectory_cache`` -- the last planned goal joints
  (``set_last_config``: where the next leg starts and the IK warm start) and the last working pose
  (``set_last_pose``: the on-axis lift-off from the previous weld) -- followed by one or more
  ``run_trajectory_planner`` calls executed in order (the state flows from call to call exactly as in one
  session). ``current_joints`` is NOT used to inject the start (umarell skips the lift-off when it is pinned,
  docs/upstream/05). Returns every call's result and the state after it.
* ``assemble`` task: a ``trajectory_cache.snapshot`` (trajectories + final states) restored into an empty cache,
  then ``assemble_behaviour_tree``; returns the result plus umarell's own schedule metrics of the tree.

Results come back to the orchestrator, which owns the authoritative cache.
"""
from __future__ import annotations

import os
import time
import traceback

from umarell_fast.u2 import session as S

_CFG: dict = {}


def init_worker(repo: str, cell: str, env: dict, kernels: str, seed: int = 0) -> None:
    os.environ.update(env)
    S.setup(repo, cell, env, kernels=kernels)
    S.seed_rngs(int(seed) * 7919 + os.getpid(), ompl_too=True)
    _CFG.update(repo=repo, cell=cell, kernels=kernels)


def _inject(cell: str, states: dict) -> None:
    from agents.trajectory.tools import trajectory_cache as tc
    tc.clear_trajectory_cache(cell)
    tc.reset_robot_configs(cell)
    for ns, st in (states or {}).items():
        if st.get("last_config"):
            tc.set_last_config(cell, ns, st["last_config"])
        lp = st.get("last_pose")
        if lp:
            tc.set_last_pose(cell, ns, lp["target"], lp["approach_vector"], lp.get("orientation"))


def _state(cell: str, ns: str) -> dict:
    from agents.trajectory.tools import trajectory_cache as tc
    return {"last_config": tc.get_last_config(cell, ns), "last_pose": tc.get_last_pose(cell, ns)}


def plan_task(task: dict) -> dict:
    """task = {"key", "states": {ns: state}, "seed", "calls": [{"motion_requests": [...], "velocity_scale"?,
    "approach_offset_m"?}], "stop_on_fail": bool}"""
    cell = S._STATE["cell"]
    t0 = time.perf_counter()
    out = {"key": task.get("key"), "pid": os.getpid(), "calls": []}
    try:
        _inject(cell, task.get("states"))
        if task.get("seed") is not None:
            S.seed_rngs(int(task["seed"]))
        for c in task["calls"]:
            kw = {k: c[k] for k in ("velocity_scale", "approach_offset_m") if c.get(k) is not None}
            tc0 = time.perf_counter()
            res = S.call("run_trajectory_planner", cell_id=cell, motion_requests=c["motion_requests"], **kw)
            robots = [r["robot_ns"] for r in c["motion_requests"]]
            ok = all(r in (res.get("trajectories") or {}) for r in robots)
            out["calls"].append({"result": res, "ok": ok, "wall_s": round(time.perf_counter() - tc0, 4),
                                 "state_out": {ns: _state(cell, ns) for ns in robots}})
            if not ok and task.get("stop_on_fail", True):
                break
        out["ok"] = bool(out["calls"]) and all(c["ok"] for c in out["calls"]) and \
            len(out["calls"]) == len(task["calls"])
        sc = task.get("score_against")
        if out["ok"] and sc:
            last = out["calls"][-1]["result"]["trajectories"][sc["ns"]]
            out["clearance"] = park_clearance(cell, sc["ns"], last[-1]["positions"], sc["others"])
    except BaseException as exc:  # noqa: BLE001 -- PlanCancelled is a BaseException; report, never kill the pool
        out["ok"] = False
        out["error"] = f"{type(exc).__name__}: {exc}"[:500]
        out["trace"] = traceback.format_exc()[-1500:]
    out["wall_s"] = round(time.perf_counter() - t0, 4)
    return out


def assemble_task(task: dict) -> dict:
    """task = {"key", "snapshot": trajectory_cache.snapshot(...), "plan": {...}}"""
    cell = S._STATE["cell"]
    t0 = time.perf_counter()
    out = {"key": task.get("key"), "pid": os.getpid()}
    try:
        from agents.trajectory.tools import trajectory_cache as tc
        tc.clear_trajectory_cache(cell)
        tc.reset_robot_configs(cell)
        tc.restore(cell, task["snapshot"])
        if task.get("seed") is not None:
            S.seed_rngs(int(task["seed"]))
        res = S.call("assemble_behaviour_tree", cell_id=cell, plan=task["plan"])
        out["result"] = res
        out["ok"] = bool(res.get("success"))
        if out["ok"]:
            out["schedule"] = schedule_of(res["behaviour_tree"])
    except BaseException as exc:  # noqa: BLE001
        out["ok"] = False
        out["error"] = f"{type(exc).__name__}: {exc}"[:500]
        out["trace"] = traceback.format_exc()[-1500:]
    out["wall_s"] = round(time.perf_counter() - t0, 4)
    return out


def schedule_of(tree: dict) -> dict:
    from agents.trajectory.tools import trajectory_quality as tq
    try:
        m = tq.schedule_metrics(tree)
    except Exception as exc:  # noqa: BLE001
        return {"error": str(exc)}
    return {k: m.get(k) for k in ("makespan_s", "parallel_efficiency", "serial_fraction", "concurrent_s",
                                  "single_arm_s", "motion_legs", "unpaired_time_s", "load_balance") if k in m} or m


def park_clearance(cell: str, ns: str, q: list, others: dict) -> float:
    """Min arm-to-arm clearance of the other arm's legs ``others`` ({ns: [traj, ...]}) with arm ``ns`` standing at
    ``q`` (umarell's validate_trajectory_set against a parked arm, as the assembler gate checks bystanders)."""
    from agents.trajectory.tools.behaviour_tree import _parked
    worst = float("inf")
    for o, trajs in (others or {}).items():
        for tr in trajs:
            if not tr:
                continue
            r = S.call("validate_trajectory_set", cell_id=cell, trajectories={o: tr, ns: _parked(q)},
                       suggest_delay=False)
            m = r.get("min_clearance_m")
            if m is not None:
                worst = min(worst, float(m))
    return worst


def func_task(task: dict) -> dict:
    """Read-only umarell computations off the main process: ``find_start_delays`` of one step's legs, or the
    clearance score of a park pose."""
    cell = S._STATE["cell"]
    t0 = time.perf_counter()
    out = {"key": task.get("key")}
    try:
        if task["fn"] == "find_start_delays":
            from agents.trajectory.tools.planning_tools import find_start_delays
            out["value"] = find_start_delays(cell, task["legs"])
        elif task["fn"] == "park_clearance":
            out["value"] = park_clearance(cell, task["ns"], task["q"], task["others"])
        else:
            raise ValueError(task["fn"])
        out["ok"] = True
    except BaseException as exc:  # noqa: BLE001
        out["ok"] = False
        out["error"] = f"{type(exc).__name__}: {exc}"[:500]
    out["wall_s"] = round(time.perf_counter() - t0, 4)
    return out
