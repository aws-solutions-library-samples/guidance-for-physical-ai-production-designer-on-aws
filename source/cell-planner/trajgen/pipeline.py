"""The trajectory generator: scene.yaml + task.json -> N-robot joint timeline, umarell BT plan, report.

    from trajgen import generate
    res = generate("scenes/welding_cell_2_small/scene.yaml", "scenes/welding_cell_2_small/task.json",
                   out="data/trajgen/sample")
    res.status              # "complete" | "partial" (allow_partial) | "failed"
    res.timeline            # trajgen.ntimeline.NTimeline, executor clock (== replay of res.plan)
    res.plan                # umarell BT plan envelope (dict); None when failed
    res.report              # dict (also out/report.json, written in every case)

Pipeline (deterministic, hand-written, nothing learned):
  1. reachability + 5-DOF weld IK per weld and robot (trajgen.reach), with a reason for every unreachable pair
  2. allocation + order (trajgen.alloc): two robots: sweep split + free search; any count: greedy + local search
  3. legs: on-axis lift-off, checked joint transit (direct / vias / RRT-Connect), straight descent (trajgen.schedule)
  4. S-curve time parameterisation under the scene's caps (trajgen.motion)
  5. coordination on one clock: "steps" (two robots, umarell's sync-at-rest step model, beam search, trajgen.search)
     or "priority" (any robot count, prioritized planning with start delays / waits / yields, trajgen.coord)
  6. outputs + dense verification (trajgen.verify)

Fail loudly: unreachable welds, welds no robot can do while the others stand at HOME, or no safe schedule -> no plan
is written (report.json lists the failed welds and the reasons; the CLI exits 2). ``allow_partial`` writes the plan of
the remaining welds, marked partial in plan.json, timeline meta and report.json (the CLI exits 3).
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


class AssignmentError(ValueError):
    """A given assignment is inconsistent with the scene / task."""


@dataclass
class GenerateResult:
    timeline: object
    timeline_motion: object
    plan: dict | None
    report: dict
    legs: list
    legs_motion: list
    paths: dict = field(default_factory=dict)

    @property
    def status(self) -> str:
        return self.report.get("status", "failed")

    @property
    def ok(self) -> bool:
        return self.status == "complete" and bool(self.report.get("verdict", {}).get("all_pass"))


def parse_assignment(a):
    """auto | dict | JSON string | path to a JSON file -> None or {rid: [wid, ...]} (no scene checks)."""
    if a in (None, "auto"):
        return None
    if isinstance(a, dict):
        d = a
    else:
        s = str(a).strip()
        if s.startswith("{"):
            try:
                d = json.loads(s)
            except json.JSONDecodeError as e:
                raise AssignmentError(f"--assignment: invalid JSON: {e}") from None
        else:
            p = Path(s)
            if not p.exists():
                raise AssignmentError(f"--assignment: file not found: {p}")
            try:
                d = json.load(open(p))
            except json.JSONDecodeError as e:
                raise AssignmentError(f"--assignment {p}: invalid JSON: {e}") from None
    if not isinstance(d, dict) or not all(isinstance(v, (list, tuple)) for v in d.values()):
        raise AssignmentError('--assignment must map robot ids to lists of weld ids, e.g. {"robot1": ["W01"]}')
    return {str(k): [str(x) for x in v] for k, v in d.items()}


_parse_assignment = parse_assignment


def check_assignment(given: dict, scene, task, reach=None, allow_partial: bool = False) -> dict:
    """Validate a given assignment; returns it with every scene robot as a key. Raises AssignmentError naming the
    offending robots / welds."""
    rids = list(scene.robot_ids)
    wids = [w.id for w in task.welds]
    bad_r = [k for k in given if k not in rids]
    if bad_r:
        raise AssignmentError(f"assignment: unknown robot(s) {bad_r} (scene robots {rids})")
    seen, dup = {}, []
    for r, ws in given.items():
        for w in ws:
            if w in seen:
                dup.append(f"{w} ({seen[w]}, {r})")
            seen[w] = r
    if dup:
        raise AssignmentError(f"assignment: weld(s) assigned more than once: {dup}")
    unk = [w for w in seen if w not in wids]
    if unk:
        raise AssignmentError(f"assignment: unknown weld id(s) {unk}")
    miss = [w for w in wids if w not in seen]
    if miss and not allow_partial:
        raise AssignmentError(f"assignment: task weld(s) not assigned to any robot: {miss} (or use allow_partial)")
    out = {r: list(given.get(r, [])) for r in rids}
    for r, ws in out.items():
        for w in ws:
            aw = task.weld(w).allowed_robots
            if aw is not None and r not in aw:
                raise AssignmentError(f"assignment: {w} given to {r}, not in its allowed_robots {aw}")
    if reach is not None:
        nope = [f"{w} -> {r}" for r, ws in out.items() for w in ws if not reach.get((w, r))]
        if nope:
            reasons = getattr(reach, "reasons", {})
            raise AssignmentError("assignment: robot cannot reach the weld: " + "; ".join(
                f"{x} ({reasons.get(x.replace(' -> ', '|'), 'no valid weld option')})" for x in nope))
    return out


def build_legs(model, steps, dt, margin: float):
    """Search steps -> (motion-clock legs, executor-clock legs, executor steps) as player.timeline.Leg."""
    from player.timeline import Leg

    motion, execl, ex_steps = [], [], []
    n_yield = {r: 0 for r in model.rids}
    n_park = {r: 0 for r in model.rids}
    wids = {w.id for w in model.task.welds}
    last = {r: max([k for k, s in enumerate(steps) if r in s.legs], default=-1) for r in model.rids}
    for k, s in enumerate(steps):
        row = []
        for rid, L in s.legs.items():
            if L.target == "park":            # parking leaf (lift-off only, round p3)
                n_park[rid] += 1
                op = f"{rid.upper()}_PARK_{n_park[rid]}"
                while op in wids:
                    op += "_P"
                kind, otype, node = "move", "move", "ExecuteTrajectory"
            elif L.target == "home" and k == last[rid]:
                kind, op, otype, node = "home", "home", "", "MoveHome"
            elif L.target == "home":            # a yield: back HOME while welds remain
                n_yield[rid] += 1
                kind, op, otype, node = "move", f"{rid.upper()}_YIELD_{n_yield[rid]}", "move", "ExecuteTrajectory"
            else:
                kind, op, otype, node = "weld", L.target, "weld", "ExecuteTrajectory"
            for clock, out in ((0.0, motion), (k * margin, execl)):
                t0 = s.t0 + clock
                times = np.round(t0 + np.arange(len(L.Q)) * dt, 9)
                leg = Leg(robot=rid, op_id=op, kind=kind, t0=float(t0), t1=float(times[-1]), times=times,
                          positions=L.Q.copy(), phase=L.phase.copy(), hold_s=float(L.hold_s), vscale=1.0,
                          operation_type=otype, node_type=node, bb_key=f"traj_{rid}_{op}")
                leg.t_done = leg.t1 + leg.hold_s + (margin if out is execl else 0.0)
                out.append(leg)
                if out is execl:
                    row.append(leg)
        ex_steps.append(row)
    return motion, execl, ex_steps


class _Reach(dict):
    reasons: dict = {}


def _thin_hints(rep):
    """Annotate failed welds with the robots whose thin options were dropped by the margin preference (and suggest
    --allow-thin): such a failure may be solvable with the thin options."""
    td = [tuple(x) for x in (rep.get("options") or {}).get("thin_dropped") or []]
    if not td:
        return
    by_w = {}
    for w, r in td:
        by_w.setdefault(w, []).append(r)
    for f in rep.get("failed_welds") or []:
        w = f.get("weld")
        rs = by_w.get(w, []) if w is not None else sorted({r for _, r in td})
        if f.get("stage") == "coordination" and not rs:
            rs = sorted({r for _, r in td})
        if rs:
            f["thin_dropped_robots"] = rs
            f["hint"] = (f"thin options (valid, below the preferred hull self-distance) of {rs} were dropped because "
                         "another robot had a comfortable one; retry with --allow-thin")


def _write_failure(out, rep, log):
    _thin_hints(rep)
    paths = {}
    if out is not None:
        out = Path(out)
        out.mkdir(parents=True, exist_ok=True)
        for f in ("timeline.npz", "timeline_2robot.npz", "timeline_motion.npz", "plan.json"):
            if (out / f).exists():                     # never leave an older plan next to a failed report
                (out / f).unlink()
        paths["report"] = str(out / "report.json")
        rep["outputs"] = paths
        with open(out / "report.json", "w") as f:
            json.dump(rep, f, indent=1, default=_jsonable)
    log(f"[trajgen] FAILED: {json.dumps(rep.get('failed_welds'), default=_jsonable)}")
    return GenerateResult(None, None, None, rep, [], [], paths)


def generate(scene_path, task_path=None, *, out=None, assignment="auto", speed: str | None = None, seed: int = 0,
             n_candidates: int | None = None, check_hz: float | None = None, clearance_margin: float = 0.01,
             jerk_factor: float | None = None, max_tilt_deg: float = 3.5, margin_m: float = 0.015,
             n_random: int = 120, verify: bool = True, cache: bool = True, name: str | None = None,
             window: int = 2, group_order: str = "per_arm", umarell_parity: bool | None = None, beam: int = 32,
             k_cand: int = 6, objective: str = "executor", allow_thin: bool = False, map_stride: int = 2,
             h_weights=(1.0, 1.6), per_done_set: int = 0, n_branch: int = 1, coordination: str = "auto",
             allow_partial: bool = False, skew_s: float = 0.1, n_orders: int = 4, sync_waits: bool = True,
             lag_check: bool = True, lateness_s: float = 0.2, repair_iters: int = 6, order_search: str = "auto",
             workers: int | None = None, park: bool = True, search: str = "wide", repair_pool: int = 6,
             lateness_pool: int = 16, delay_search: bool = True, steps_portfolio="auto", log=print,
             electrode_rule: bool | None = None, nodip: bool | None = None, _fallback_info: dict | None = None,
             _keep_thin: tuple = ()) -> GenerateResult:
    """Run the full pipeline (module docstring). ``assignment``: "auto", a dict {rid: [wid, ...]} (ordered queues,
    used as given), a JSON string or a path to a JSON file. ``speed``: "umarell" (the scene's planning rules),
    "fast" (accel a_max * v_s instead of * v_s^2, HOME at transit speed) or "full" (non-weld motion at 1.0 x the URDF
    velocity limits with the scene's acceleration and a jerk limit, weld strokes as umarell; trajgen.fullspeed);
    None = the scene's ``planning.speed_profile``, else "umarell". ``coordination``: "auto" (steps for two
    robots, else priority), "steps" or "priority". ``umarell_parity``: None = the scene's declaration. Writes
    timeline.npz (+ timeline_2robot.npz for two robots, timeline_motion.npz for steps), plan.json and report.json
    into ``out`` if given; on failure only report.json. Any older output in ``out`` is removed first.

    ``window``: queue head window; None = 2, and for priority coordination with 3+ robots a portfolio of windows
    2 and 3 (the best executor end wins). Status: "complete" (verified, or verify=False), "unverified" (a complete
    plan was written but the dense verification failed: plan.json status "unverified"), "partial", "failed".

    ``workers``: worker processes for the parallel stages (trajgen.parallel; None = TRAJGEN_WORKERS or the CPU
    count, 1 = serial). The plan does not depend on it.

    ``electrode_rule``: the electrode off-weld rule (docs/n2_contract.md 3.1): None = as the scene declares
    (``obstacles.box_hull.electrode_off_weld``; scenegen v2 scenes enable it), True / False = force it on / off.
    ``nodip``: the round-3 no-dip rule (trajgen.nodip, docs/r3_nodip.md): None = as the scene declares
    (``obstacles.box_hull.no_dip``, absent = ``trajgen.nodip.default_on()``), True / False = force it on / off (it
    replaces the electrode off-weld rule on scenes with workpiece boxes; applied after ``electrode_rule``).

    ``park``: wait at the retract pose instead of on the weld (lift-off-only parking legs; round p3).
    ``search`` (priority coordination): "wide" (round p3) = every priority order (<= 4 robots) of every
    (assignment, window) and of the balance neighbours, the +lateness check on the ``lateness_pool`` shortest
    results and the lateness repair of up to ``repair_pool`` shorter failing ones, the shortest passing plan wins;
    "round4" = the round-4 flow (4 orders, hill climb, balance step, repair of the best + 4).
    ``steps_portfolio`` (steps coordination): (beam, k_cand) pairs searched, each with every heuristic weight and
    every start (free search, allocation candidates); "auto" = (beam, k_cand), (beam, k_cand + 2), (2 beam, k_cand)
    (round p3; None or [] = (beam, k_cand) only, the round-4 portfolio). The plan with the shortest executor end
    after the post-passes (early HOME, parking) wins, ties: shorter weld stays, then the search objective."""
    from trajgen.parallel import n_workers
    W = n_workers(workers)
    from trajgen.alloc import CostModel, allocate, allocate_n
    from trajgen.model import SceneModel
    from trajgen.reach import reach_table, solve_reach
    from trajgen.scene import load_scene, load_task, validate
    from trajgen.schedule import PairChecker, Planner, make_speeds

    t_wall = time.time()
    if out is not None:
        clear_outputs(out)
    scene = load_scene(scene_path)
    if electrode_rule is not None and bool(electrode_rule) != bool(scene.box_hull.electrode.enabled):
        from trajgen.scene import with_electrode_rule
        scene = with_electrode_rule(scene, bool(electrode_rule))
    if nodip is not None:
        from trajgen.scene import nodip_on, with_nodip
        if bool(nodip) != nodip_on(scene):
            scene = with_nodip(scene, bool(nodip))
    task = load_task(task_path or (scene.path.parent / "task.json"))
    warnings = validate(scene, task)
    given = parse_assignment(assignment)
    if given is not None:
        given = check_assignment(given, scene, task, allow_partial=allow_partial)
    from trajgen.fullspeed import resolve_speed
    speed = resolve_speed(scene, speed)
    pl = scene.planning
    dt = pl.sample_dt
    check_hz = float(check_hz) if check_hz else max(100.0, 1.0 / pl.check_dt)
    model = SceneModel(scene, task)
    model.ik.rng = np.random.default_rng(seed)
    if W > 1 and len(scene.boxes):
        model._box_field()                     # build / load the box distance field once, before the workers fork
    R = model.n
    coord_mode = coordination if coordination != "auto" else ("steps" if R == 2 else "priority")
    if coord_mode == "steps" and R != 2:
        raise AssignmentError("coordination 'steps' needs exactly two robots (use 'priority')")
    n_candidates_in = n_candidates
    if n_candidates is None:                  # allocation candidates: steps 1 (+ the free search), priority 2
        n_candidates = 1 if coord_mode == "steps" else 2
    window_in = window
    if window is None:
        windows = [2, 3] if (coord_mode == "priority" and R >= 3) else [2]
    else:
        windows = [int(window)]
    window = windows[0]
    if umarell_parity is None:
        umarell_parity = bool(scene.umarell_parity)
    if umarell_parity and model.umarell is None:
        warnings.append(f"umarell parity requested but unavailable: {model.parity_info.get('reason')}")
        umarell_parity = False
    log(f"[trajgen] {scene.cell_id}: {R} robots {list(model.rids)}, {len(task.welds)} welds, speed {speed}, "
        f"{len(scene.boxes)} boxes, coordination {coord_mode}, parity {umarell_parity}")
    rep = {"generator": "trajgen", "status": "running",
           "scene": str(scene.path.relative_to(ROOT)) if scene.path.is_relative_to(ROOT) else str(scene.path),
           "task": str(task.path), "cell_id": scene.cell_id, "robots": list(model.rids), "speed": speed, "seed": seed,
           "coordination": coord_mode,
           "options": {"n_candidates": n_candidates, "check_hz": check_hz, "clearance_margin_m": clearance_margin,
                       "max_tilt_deg": max_tilt_deg, "margin_m": margin_m, "assignment": "given" if given else "auto",
                       "window": windows if len(windows) > 1 else window, "group_order": group_order, "umarell_parity": bool(umarell_parity),
                       "beam": beam, "k_cand": k_cand, "objective": objective, "allow_thin": allow_thin,
                       "map_stride": map_stride, "n_branch": n_branch, "h_weights": list(h_weights),
                       "per_done_set": per_done_set, "allow_partial": allow_partial, "skew_s": skew_s,
                       "n_orders": n_orders, "sync_waits": sync_waits, "workers": W, "park": bool(park),
                       "search": search, "repair_pool": repair_pool, "lateness_pool": lateness_pool,
                       "delay_search": bool(delay_search)},
           "steps_portfolio": None,
           "umarell_parity_model": model.parity_info, "scene_warnings": warnings}
    if _fallback_info:
        rep["steps_fallback"] = _fallback_info
    if _keep_thin:
        rep["options"]["thin_restored"] = [list(x) for x in _keep_thin]
    if scene.box_hull.electrode.enabled:              # electrode off-weld rule (round 2; absent when off)
        er = scene.box_hull.electrode
        rep["electrode_rule"] = {"enabled": True, "radius_m": er.radius, "min_distance_m": er.min_distance,
                                 "score_tol_m": er.score_tol,
                                 "source": "scene" if electrode_rule is None else "generate(electrode_rule=True)"}
        if getattr(er, "nodip", False):               # round 3 no-dip rule (trajgen.nodip)
            from trajgen.nodip import rule_dict
            rep["electrode_rule"].update(rule_dict(), contact_depth_m=er.contact_depth,
                                         contact_point_radius_m=er.contact_point_radius, radius_m=er.radius,
                                         source="scene" if nodip is None else f"generate(nodip={bool(nodip)})")

    # 0. every robot's HOME must satisfy the single-arm rules (SCHEMA.md section 10); a robot starts and ends there
    bad_home = _homes_valid(model)
    if bad_home:
        rep.update(status="failed", failed_welds=[{"weld": None, "stage": "homes", "robot": r, "reason": why}
                                                  for r, why in bad_home],
                   wall_s={"total": round(time.time() - t_wall, 1)})
        return _write_failure(out, rep, log)

    # 1. reachability
    t0 = time.time()
    reach = _Reach(solve_reach(model, n_random=n_random, max_tilt_deg=max_tilt_deg, margin_m=margin_m,
                               cache=cache, seed=seed, log=log, workers=W))
    reach.reasons = dict(getattr(model, "reach_reasons", {}))
    t_reach = time.time() - t0
    thin_dropped = []
    thin_note = {}
    # margin preference: thin options (valid, below the preferred self-distance) only where no robot has a
    # comfortable one. Steps coordination applies it here (a failed step search falls back to priority, which
    # re-applies it after the eligibility test); priority coordination applies it AFTER the eligibility test (round-4
    # finding: dropping a robot's thin options because another robot has a comfortable one that is then blocked by
    # a robot standing at HOME declared solvable welds infeasible)
    thin_late = (not allow_thin) and coord_mode == "priority"
    if not allow_thin and not thin_late:
        for w in task.welds:
            if any(not o.thin for r in model.rids for o in reach.get((w.id, r), [])):
                for r in model.rids:
                    ops = reach.get((w.id, r), [])
                    if (w.id, r) in _keep_thin:      # retry after a failed step search: thin options restored
                        continue
                    if ops and all(o.thin for o in ops):
                        thin_dropped.append((w.id, r))
                        thin_note[(w.id, r)] = "only thin options (another robot has a comfortable one)"
                        reach.reasons[f"{w.id}|{r}"] = thin_note[(w.id, r)]
                    reach[(w.id, r)] = [o for o in ops if not o.thin]
        if thin_dropped:
            log(f"[trajgen] thin-only options dropped (another arm has a comfortable one): {thin_dropped}")
    rep["options"]["thin_dropped"] = thin_dropped
    rep["reach"] = reach_table(reach, task.welds, model.rids, reach.reasons)
    rep["wall_s"] = {"reach": round(t_reach, 1)}
    failed = []
    for w in task.welds:
        if not any(reach.get((w.id, r)) for r in model.rids):
            failed.append({"weld": w.id, "stage": "reach", "reason": "no robot can reach the weld with a valid "
                           "straight descent / lift-off", "per_robot": {r: reach.reasons.get(f"{w.id}|{r}", "no option")
                                                                        for r in model.rids}})
    if given is not None:
        try:
            given = check_assignment(given, scene, task, reach=reach, allow_partial=allow_partial)
        except AssignmentError as e:
            if not allow_partial:
                raise
            log(f"[trajgen] {e}; dropping those welds (allow_partial)")
            given = {r: [w for w in ws if reach.get((w, r))] for r, ws in given.items()}
    speeds = {r: make_speeds(scene, r, speed, jerk_factor) for r in model.rids}
    planner = Planner(model, speeds, dt, pl.dwell_s, log=log)
    planner.rrt_seed = seed
    checker = PairChecker(model, dt, check_hz=check_hz, clearance_margin=clearance_margin, umarell_parity=umarell_parity,
                          skew_s=skew_s if coord_mode == "steps" else 0.0)   # (coord applies its own skew window)
    rep["speeds"] = {r: speeds[r].info for r in model.rids}

    # eligibility for the priority coordinator: options whose HOME -> weld -> HOME legs are clear of the other
    # robots standing at HOME (the deadlock-freedom condition, trajgen.coord)
    coord = None
    if coord_mode == "priority":
        from trajgen.coord import Coordinator
        cm0 = CostModel(model, planner, reach)
        coord = Coordinator(model, planner, checker, reach, cm0, task, executor_margin=pl.executor_margin_s,
                            skew_s=skew_s, window=window, n_branch=n_branch, group_order=group_order, log=log)
        coord.park = bool(park)
        homes_ok = _homes_clear(model, checker)
        if homes_ok is not True:
            failed.append({"weld": None, "stage": "homes", "reason": homes_ok})
        elig_rep = {}
        thin_kept = []
        t1 = time.time()
        se = coord.static_eligible_all([(r, w.id) for w in task.welds for r in model.rids if reach.get((w.id, r))], W)
        rep["wall_s"]["eligibility"] = round(time.time() - t1, 1)
        for w in task.welds:
            row = {}
            for r in model.rids:
                if not reach.get((w.id, r)):
                    continue
                ks, why = se[(r, w.id)]
                coord.eligible[(w.id, r)] = ks
                row[r] = {"eligible_options": ks, "reason": None if ks else why}
                if not ks:
                    reach.reasons[f"{w.id}|{r}"] = why
            if thin_late:              # the margin preference, among the ELIGIBLE options only
                ops_of = lambda r: reach.get((w.id, r), [])                         # noqa: E731
                comfy = any(not ops_of(r)[k].thin for r in row for k in coord.eligible.get((w.id, r), []))
                if comfy:
                    for r in row:
                        ks = coord.eligible.get((w.id, r), [])
                        k2 = [k for k in ks if not ops_of(r)[k].thin]
                        if ks and not k2:
                            thin_dropped.append((w.id, r))
                            why = "only thin options (another robot has a comfortable eligible one)"
                            thin_note[(w.id, r)] = why
                            reach.reasons[f"{w.id}|{r}"] = why
                            row[r]["reason"] = why
                        coord.eligible[(w.id, r)] = k2
                        row[r]["eligible_options"] = k2
                elif any(coord.eligible.get((w.id, r)) for r in row):
                    thin_kept.append(w.id)
            elig_rep[w.id] = row
            if any(reach.get((w.id, r)) for r in model.rids) and not any(v["eligible_options"] for v in row.values()):
                blocked = any("another robot" in (v["reason"] or "") for v in row.values())
                why = ("every robot that reaches the weld collides with another robot standing at HOME" if blocked
                       else "no robot that reaches the weld has a collision-free joint transit HOME -> weld -> HOME")
                failed.append({"weld": w.id, "stage": "eligibility", "reason": why,
                               "per_robot": {r: v["reason"] for r, v in row.items()}})
        rep["eligibility"] = elig_rep
        if thin_late:
            if thin_dropped:
                log(f"[trajgen] thin-only options dropped after eligibility (another robot has a comfortable eligible "
                    f"option): {thin_dropped}")
            if thin_kept:
                log(f"[trajgen] thin options kept (no comfortable eligible option): {thin_kept}")
            rep["options"]["thin_dropped"] = thin_dropped
            rep["options"]["thin_kept_welds"] = thin_kept
            rep["reach"] = reach_table(reach, task.welds, model.rids, reach.reasons)
    for (w_, r_), why in thin_note.items():       # mark the dropped thin options in the reach table
        row_ = rep["reach"].get(w_, {}).get(r_)
        if row_ is not None:
            row_["thin_dropped"] = why
        if given is not None:
            bad = [f"{w} -> {r}" for r, ws in given.items() for w in ws if not coord.eligible.get((w, r))]
            if bad and not allow_partial:
                raise AssignmentError("assignment: the robot's weld legs collide with another robot standing at HOME "
                                      "(not schedulable by prioritized planning): " + "; ".join(bad))
            given = {r: [w for w in ws if coord.eligible.get((w, r))] for r, ws in given.items()}
    if failed and not allow_partial:
        rep.update(status="failed", failed_welds=failed, wall_s={**rep["wall_s"], "total": round(time.time() - t_wall, 1)})
        return _write_failure(out, rep, log)
    drop = {f["weld"] for f in failed if f.get("weld")}
    target_welds = [w.id for w in task.welds if w.id not in drop]

    # 2 + 5. allocation, order and coordination
    t0 = time.time()
    results = []
    if coord_mode == "steps":
        from trajgen.search import Search
        cm = CostModel(model, planner, reach)
        runs = []
        if given is not None:
            runs.append(("given", given, 1))
            alloc_info = {"given": given}
        else:
            runs.append(("free", None, None))
            if n_candidates > 0:
                cands, alloc_info = allocate(model, cm, task, n_candidates=n_candidates, group_order=group_order, log=log)
                runs += [(f"alloc_k{c['k']}", c["queues"], window) for c in cands]
            else:
                alloc_info = {}
        if steps_portfolio == "auto":
            pf = [(beam, k_cand), (beam, k_cand + 2), (2 * beam, k_cand)]
        elif steps_portfolio:
            pf = [tuple(int(v) for v in x) for x in steps_portfolio]
        else:
            pf = [(beam, k_cand)]
        rep["steps_portfolio"] = [list(x) for x in pf]
        runs = [((f"{lab}_h{hw:g}" if len(h_weights) > 1 else lab) + ("" if (b_, k_) == (beam, k_cand) else
                 f"_b{b_}k{k_}"), q, w, hw, b_, k_) for b_, k_ in pf for lab, q, w in runs for hw in h_weights]
        sctx = dict(model=model, planner=planner, checker=checker, reach=reach, cm=cm, task=task,
                    kw=dict(objective=objective, executor_margin=pl.executor_margin_s,
                            group_order=group_order, map_stride=map_stride, per_done_set=per_done_set,
                            n_branch=n_branch, log=log), target=frozenset(target_welds), park=bool(park))
        from trajgen.parallel import pmap
        for res in pmap(_task_step_search, sctx, runs, W, log, "step searches"):
            log(f"[trajgen] {res['label']}: complete {res['complete']} objective {res['objective']:.2f} "
                f"steps {len(res['steps'])} ({res['wall_s']:.0f} s, {res['stats']})")
            results.append(res)
        # the shortest executor end after the post-passes; ties: shorter stays, the search objective, portfolio order
        if steps_portfolio == "auto" or steps_portfolio:
            best = min(results, key=lambda r: (r.get("final_s", np.inf), r.get("stay_excess_s", 0.0), r["objective"]))
        else:                                       # round-4 portfolio: the search objective decides (as in round 4)
            best = min(results, key=lambda r: r["objective"])
        if not best["complete"]:
            common = dict(speed=speed, seed=seed, n_candidates=n_candidates_in, check_hz=check_hz,
                          clearance_margin=clearance_margin, jerk_factor=jerk_factor, max_tilt_deg=max_tilt_deg,
                          margin_m=margin_m, n_random=n_random, verify=verify, cache=cache, name=name,
                          window=window_in, group_order=group_order, umarell_parity=umarell_parity, beam=beam,
                          k_cand=k_cand, objective=objective, allow_thin=allow_thin, map_stride=map_stride,
                          h_weights=h_weights, per_done_set=per_done_set, n_branch=n_branch,
                          allow_partial=allow_partial, skew_s=skew_s, n_orders=n_orders, sync_waits=sync_waits,
                          lag_check=lag_check, lateness_s=lateness_s, repair_iters=repair_iters,
                          order_search=order_search, workers=W, park=park, search=search,
                          repair_pool=repair_pool, lateness_pool=lateness_pool, delay_search=delay_search,
                          steps_portfolio=steps_portfolio, log=log, electrode_rule=electrode_rule, nodip=nodip)
            if thin_dropped and not _keep_thin:      # recheck with the dropped thin-only options restored
                log(f"[trajgen] steps coordination found no complete schedule: retrying with the thin options of "
                    f"{thin_dropped} restored")
                fb = {"reason": "steps coordination found no complete schedule with the thin-only options dropped",
                      "searches": [{"label": r["label"], "complete": r["complete"]} for r in results]}
                return generate(scene_path, task_path, out=out, assignment=assignment if given is None else given,
                                coordination=coordination, _fallback_info=fb, _keep_thin=tuple(thin_dropped),
                                **common)
            if coordination == "auto":
                log("[trajgen] steps coordination found no complete schedule: falling back to priority coordination")
                fb = {"reason": "steps coordination found no complete schedule",
                      "searches": [{"label": r["label"], "complete": r["complete"]} for r in results]}
                fb0 = (_fallback_info or {})
                if fb0:
                    fb["earlier"] = fb0
                return generate(scene_path, task_path, out=out, assignment=assignment if given is None else given,
                                coordination="priority", _fallback_info=fb, **common)
            rep.update(status="failed", failed_welds=[{"weld": w, "stage": "coordination", "reason":
                                                       "no complete two-arm step schedule found"} for w in target_welds],
                       searches=_search_rows(results))
            return _write_failure(out, rep, log)
        # tie-break on weld stays (round-4 finding): an arm with no work left goes HOME during the other arm's
        # 'alone' steps when that does not lengthen them (same makespan, shorter stay on the weld); computed in the
        # search task (_task_step_search) for every complete search
        steps, eh = best["early_home"]
        rep["early_home"] = eh
        if eh:
            log(f"[trajgen] early HOME post-pass: {eh}")
        if park:                     # parking post-pass (round p3): wait at the retract pose, not on the weld
            steps, pk = best["park"]
            rep["park"] = pk
            log(f"[trajgen] parking post-pass: {sum(1 for x in pk if x.get('how'))} legs parked, "
                f"{sum(1 for x in pk if not x.get('how'))} not")
        rep["searches"] = _search_rows(results)
        rep["allocation"] = alloc_info
    else:
        from trajgen.alloc import order_arm
        from trajgen.coord import priority_orders
        cmE = CostModel(model, planner, _eligible_reach(reach, coord.eligible))
        elig = {w: [r for r in model.rids if coord.eligible.get((w, r))] for w in target_welds}
        if given is not None:
            cands = [{"queues": given, "proxy": None, "T": {}}]
            alloc_info = {"given": given}
        else:
            cands, alloc_info = allocate_n(model, cmE, task, elig, n_candidates=max(1, n_candidates),
                                           group_order=group_order, leg_margin_s=pl.executor_margin_s, log=log)
        coord.cost = cmE
        runs_p = [(ci, c, wv) for wv in windows for ci, c in enumerate(cands)]
        meta_p, jobs = [], []
        for ci, c, wv in runs_p:
            queues = {r: [w for w in c["queues"].get(r, []) if w in target_welds] for r in model.rids}
            groups = task.sequence_groups_order if group_order != "ignore" else []
            if given is None:                       # keep the alloc order (groups, NN + 2-opt)
                loads = c["T"]
            else:
                loads = {r: order_arm(cmE, r, queues[r], groups, task)[1] if queues[r] else 0.0 for r in model.rids}
            orders = priority_orders(model.rids, loads, n_orders)
            if search == "wide" and order_search != "off" and R <= 4:
                orders = priority_orders(model.rids, loads, 24)            # every priority order
            meta_p.append((ci, c, wv, queues, orders))
            jobs += [(queues, o, wv, {}) for o in orders]
        t1 = time.time()
        all_res = coord.run_orders(jobs, W, label="coordination runs")        # every (assignment, window, order)
        t_runs = time.time() - t1
        k_res = 0
        for ci, c, wv, queues, orders in meta_p:
            coord.window = max(1, int(wv))
            res = coord.select(orders, all_res[k_res:k_res + len(orders)])
            k_res += len(orders)
            lab = f"alloc{ci}" + (f"_w{wv}" if len(windows) > 1 else "")
            results.append({"label": lab, "queues_in": queues, "orders": orders, "result": res, "window": wv,
                            "wall_s": t_runs if W > 1 else t_runs * len(orders) / max(1, len(jobs)),
                            "order_results": list(coord.order_results),
                            "all_results": list(zip([tuple(o) for o in orders], all_res[k_res - len(orders):k_res]))})
            log(f"[trajgen] {lab}: {'complete' if res.ok else 'FAILED'} "
                + (f"executor end {res.objective_s:.2f} s order {res.order}" if res.ok else str(res.failed)))
        ok = [r for r in results if r["result"].ok]
        rep["allocation"] = alloc_info
        rep["searches"] = [{"label": r["label"], "window": r.get("window"), "queues_in": r["queues_in"], "orders": r["orders"],
                            "complete": r["result"].ok, "objective_s": None if not r["result"].ok else
                            round(r["result"].objective_s, 3), "failed": r["result"].failed, "wall_s": round(r["wall_s"], 1),
                            "stats": r["result"].stats, "order_results": r.get("order_results")} for r in results]
        if not ok:
            fl = results[-1]["result"].failed if results else []
            if not allow_partial:
                rep.update(status="failed", failed_welds=[{"weld": w, "stage": "coordination", "reason": f["reason"],
                                                           "robot": f["robot"]} for f in fl for w in (f["welds"] or [None])])
                return _write_failure(out, rep, log)
            # partial: drop the welds the coordinator could not place and retry once
            bad = {w for f in fl for w in f["welds"]}
            failed += [{"weld": w, "stage": "coordination", "reason": f["reason"]} for f in fl for w in f["welds"]]
            target_welds = [w for w in target_welds if w not in bad]
            q2 = {r: [w for w in results[-1]["queues_in"][r] if w not in bad] for r in model.rids}
            coord.window = windows[0]
            res = coord.run(q2, priority_orders(model.rids, {}, n_orders))
            if not res.ok:
                rep.update(status="failed", failed_welds=failed)
                return _write_failure(out, rep, log)
            results.append({"label": "partial", "queues_in": q2, "orders": [], "result": res, "wall_s": 0.0})
            ok = [results[-1]]
        best_r = min(ok, key=lambda r: r["result"].objective_s)
        if search == "wide" and best_r["label"] != "partial":
            best_r = _wide_search(rep, coord, cmE, model, task, scene, pl, results, ok, best_r, elig, given, group_order,
                                  dt, R, W, n_orders, order_search, lateness_s, repair_iters, repair_pool, lateness_pool,
                                  check_hz, sync_waits, windows, log, delay_search=delay_search)
        # order search on the best (assignment, window): pairwise swaps of the priority order (hill climb; "all":
        # every permutation for <= 4 robots). Round-4 finding: 4 of 24 orders left 6 % of the makespan.
        alt_orders = []
        if search != "wide" and order_search != "off" and R >= 3 and best_r["label"] != "partial":
            t1 = time.time()
            coord.window = max(1, int(best_r.get("window", windows[0])))
            os_rep = _order_search(coord, best_r, model.rids, order_search, log,
                                   batch=lambda os_, w_=coord.window, q_=best_r["queues_in"]: coord.run_orders(
                                       [(q_, o, w_, {}) for o in os_], W, label="order search"))
            rep["order_search"] = {**os_rep["report"], "wall_s": round(time.time() - t1, 1)}
            prev_best = best_r
            if os_rep["result"] is not None and os_rep["result"].objective_s < best_r["result"].objective_s - 1e-9:
                best_r = {**best_r, "label": best_r["label"] + "_os", "result": os_rep["result"]}
            # near-best other orders of the same (assignment, window): extra candidates for the lateness repair
            alt_orders = []
            for k, rr in enumerate(sorted([x for x in os_rep["results"] + [prev_best["result"]]
                                           if x is not best_r["result"]], key=lambda x: x.objective_s)):
                if rr.objective_s <= best_r["result"].objective_s * 1.05 and len(alt_orders) < 2 and \
                        all(list(rr.order) != list(a["result"].order) for a in alt_orders):
                    alt_orders.append({**prev_best, "label": f"{prev_best['label']}_o{k}", "result": rr})
        # balance step (round-4 finding: the shipped four_robots plan had 56 / 47 s of busy time on two robots that
        # share welds): if the coordinated plan's busy spread exceeds 5 s, the two best single-weld moves off the
        # busiest robot (by the cost model) are coordinated with the best window and order (+ one default order);
        # the shorter coordinated makespan wins
        if search != "wide" and given is None and R >= 2 and best_r["label"] != "partial":
            t1 = time.time()
            coord.window = max(1, int(best_r.get("window", windows[0])))
            bres = _balance_step(coord, cmE, model, task, best_r, elig, group_order, dt, n_orders, log, workers=W)
            if bres is not None:
                rep["balance_step"] = {**bres["report"], "wall_s": round(time.time() - t1, 1)}
                if bres["result"] is not None:
                    best_r = {**best_r, "label": best_r["label"] + "_bal", "result": bres["result"],
                              "queues_in": bres["queues"]}
        # lateness repair (round-4 finding): replay the plan with every leg completing lateness_s late (all robots,
        # and each robot alone); where the clearance depended on a tighter alignment, widen the skew window of the
        # later-planned robot's leg and re-plan that order
        if search != "wide" and lateness_s and lateness_s > 0 and R > 1 and repair_iters > 0:
            # repaired: the best result and the next two (assignment, window) results; the shortest passing wins
            t1 = time.time()
            base = best_r["label"].split("_os")[0].split("_bal")[0]
            pool = [best_r] + alt_orders + sorted([r for r in ok if r["label"] != base],
                                                  key=lambda r: r["result"].objective_s)[:2]
            tried = []
            lr_args = (model, scene.cell_id, dt, pl.executor_margin_s, lateness_s, repair_iters, check_hz, sync_waits,
                       log, windows[0])
            if W > 1 and len(pool) > 1:                   # every candidate in parallel; the serial choice below
                from trajgen.parallel import pmap
                coord._fork_keys = set(coord._legs)
                lres = pmap(_task_lateness_repair, (coord, lr_args), pool, W, log, "lateness repair")
            else:
                lres = None
            for i_c, cand in enumerate(pool):
                if lres is None:
                    res_r, lr = _task_lateness_repair((coord, lr_args), cand)[:2]
                else:
                    res_r, lr, new = lres[i_c]
                    coord.merge_legs(new)
                lr["label"] = cand["label"]
                tried.append((cand, res_r, lr))
                if lr["pass"] and lr["iterations"] == 0:
                    break                                 # the best one needs no repair
            passing = [x for x in tried if x[2]["pass"]]
            cand, res_r, lr = min(passing or tried, key=lambda x: x[1].objective_s)
            lr["candidates"] = [{"label": x[2]["label"], "pass": x[2]["pass"], "iterations": x[2]["iterations"],
                                 "executor_end_before_s": x[2]["executor_end_before_s"],
                                 "executor_end_after_s": x[2]["executor_end_after_s"],
                                 "widened_legs": x[2]["widened_legs"]} for x in tried]
            lr["wall_s"] = round(time.time() - t1, 1)
            rep["lateness_repair"] = lr
            if res_r is not cand["result"] or cand is not best_r:
                best_r = {**cand, "label": cand["label"] + ("_lr" if res_r is not cand["result"] else ""),
                          "result": res_r}
        best = {"label": best_r["label"], "result": best_r["result"], "unreachable": sorted(drop),
                "window": best_r.get("window", windows[0])}
    rep["wall_s"]["coordination"] = round(time.time() - t0, 1)
    partial = bool(failed)
    status = "partial" if partial else "complete"

    # 6. outputs
    from trajgen.btplan import plan_from_sequences, plan_from_steps, replay, timing_check
    from trajgen.ntimeline import NTimeline
    homes = np.stack([model.robots[r].home for r in model.rids])
    meta = {"source": "trajgen", "scene": str(scene.path), "task": str(task.path), "speed": speed, "seed": seed,
            "robots": list(model.rids), "coordination": coord_mode, "status": status}
    tree_id = f"trajgen_{name or scene.cell_id}"
    mextra = {"generator": "trajgen", "speed": speed, "seed": seed, "robots": list(model.rids), "coordination": coord_mode,
              "partial": partial}
    if partial:
        mextra["missing_welds"] = sorted({f["weld"] for f in failed if f.get("weld")})
    # the executor completion margin the plan's timing (and every gate delay) was computed with (round-4 finding:
    # plan.json did not record it; umarell's ExecConfig.completion_margin_s defaults to 1.0)
    from player.bt_plan import COMPLETION_MARGIN_S
    mextra["executor_margin_s"] = float(pl.executor_margin_s)
    mextra["umarell_default_completion_margin_s"] = float(COMPLETION_MARGIN_S)
    if abs(pl.executor_margin_s - COMPLETION_MARGIN_S) > 1e-9:
        msg = (f"planning.executor_margin_s {pl.executor_margin_s:g} differs from umarell's default "
               f"ExecConfig.completion_margin_s {COMPLETION_MARGIN_S:g}: run the plan with completion_margin_s = "
               f"{pl.executor_margin_s:g} (plan metadata executor_margin_s); at the default every leg completes "
               f"{COMPLETION_MARGIN_S - pl.executor_margin_s:+g} s off the planned time")
        mextra["executor_margin_warning"] = msg
        if msg not in warnings:
            warnings.append(msg)
        log(f"[trajgen] WARNING: {msg}")
    tl_m, legs_m = None, []
    if coord_mode == "steps":
        legs_m, legs_x, ex_steps = build_legs(model, steps, dt, pl.executor_margin_s)
        tl_m = NTimeline.from_legs(legs_m, robot_ids=model.rids, dt=dt, home=homes,
                                   meta={**meta, "clock": "motion (no executor margins)"})
        mextra["sync"] = "every step starts and ends with both arms at rest"
        plan = plan_from_steps(scene.cell_id, ex_steps, model.rids, tree_id=tree_id, metadata_extra=mextra)
        queues_final = {r: [L.target for s_ in steps for rr, L in s_.legs.items()
                            if rr == r and L.target not in ("home", "park")]
                        for r in model.rids}
        chosen = {"search": best["label"], "queues": queues_final, "unreachable": best["unreachable"],
                  "steps": [{"t0": round(s.t0, 3), "dur": round(s.t_end - s.t0, 3), "note": s.note,
                             "legs": {r: {"target": L.target, "wait_s": round(L.wait_n * dt, 3), "vias": L.vias,
                                          "T_s": round(L.T, 3)} for r, L in s.legs.items()},
                             "min_clearance_m": round(s.min_clearance, 4)} for s in steps]}
    else:
        from trajgen.coord import legs_from_tracks
        res = best["result"]
        legs_x, deps = legs_from_tracks(res.tracks, model.rids, dt, sync=sync_waits)
        mextra["sync"] = ("per-robot sequences; every leg start held relative to the other robots' completed operations "
                          "(WaitForRobot + Delay branches in parallel with the previous leg)" if sync_waits
                          else "per-robot sequences on one clock (open-loop timing)")
        mextra["skew_window_s"] = coord.skew_s
        plan = plan_from_sequences(scene.cell_id, legs_x, model.rids, deps, tree_id=tree_id, metadata_extra=mextra)
        queues_final = {r: [cl.target for cl in res.tracks[r].legs if cl.kind == "weld"] for r in model.rids}
        chosen = {"search": best["label"], "window": best["window"], "priority_order": res.order, "queues": queues_final,
                  "unreachable": best["unreachable"],
                  "legs": {r: [{"target": cl.target, "kind": cl.kind, "t_dispatch": round(cl.k_disp * dt, 3),
                                "wait_s": round(cl.d * dt, 3), "t_start": round(cl.k_start * dt, 3),
                                "T_s": round(cl.n * dt, 3), "t_done": round(cl.k_done * dt, 3), "vias": cl.leg.vias,
                                "min_clearance_m": None if not np.isfinite(cl.cmin) else round(float(cl.cmin), 4)}
                               for cl in res.tracks[r].legs] for r in model.rids},
                  "wait_gates": {f"{k[0]}|{k[1]}": [list(g) for g in v] for k, v in deps.items()}}
    tl_x = replay(plan, model.rids, homes, dt=dt, margin=pl.executor_margin_s)
    tl_x.meta.update({**meta, "clock": "umarell executor (plan replay)", "partial": partial})
    timing = timing_check(legs_x, tl_x)
    own = NTimeline.from_legs(legs_x, robot_ids=model.rids, dt=dt, home=homes)
    nT = min(len(own.t), len(tl_x.t))
    plan_vs_legs = float(np.abs(own.q[:nT] - tl_x.q[:nT]).max())
    pc = {"max_start_error_s": timing["max_start_error_s"], "missing": timing["missing"],
          "replay_vs_legs_max_rad": plan_vs_legs}
    if R == 2 and tuple(model.rids) == ("robot1", "robot2"):
        try:
            from player.bt_plan import timeline_from_bt_plan
            tp = timeline_from_bt_plan(plan, home=homes)
            n2 = min(len(tp.t), len(tl_x.t))
            pc["player_replay_max_rad"] = float(np.abs(tp.q[:n2] - tl_x.q[:n2]).max())
            pc["player_replay_end_s"] = round(float(tp.t[-1]), 3)
        except Exception as e:  # noqa: BLE001
            pc["player_replay_error"] = str(e)
    rep.update({
        "status": status, "failed_welds": failed, "chosen": chosen,
        "makespan": {"executor_end_s": round(tl_x.meta.get("executor_end_s", tl_x.executor_end), 3),
                     "executor_motion_end_s": round(tl_x.makespan, 3),
                     "executor_last_weld_done_s": round(max((e[3] for e in tl_x.weld_events), default=0.0), 3),
                     "executor_margin_s": pl.executor_margin_s, "n_legs": len(legs_x)},
        "plan_consistency": pc, "planner_stats": {**planner.stats, "pair_checks": checker.n_checks,
                          "sphere_pairs": checker.n_pairs_sph, "sphere_pairs_exact": checker.n_exact_sph,
                          "sphere_pairs_need": getattr(checker, "n_need_sph", 0)}})
    if tl_m is not None:
        rep["makespan"].update({"motion_s": round(tl_m.makespan, 3), "motion_last_weld_done_s":
                                round(max((e[3] for e in tl_m.weld_events), default=0.0), 3), "n_steps": len(steps)})
    from trajgen.verify import phase_caps_check
    rep["phase_caps"] = phase_caps_check(legs_m or legs_x, speeds, dt)
    if speed == "full":                          # round p3 TGFULL: v / a / jerk per phase class, weld strokes slow
        from trajgen.fullspeed import speed_profile_check
        rep["speed_profile"] = speed_profile_check(
            legs_m or legs_x, speeds, dt, v_limit={r: np.asarray(speeds[r].info["v_limit_rad_s"]) for r in model.rids},
            a_limit={r: scene.robot(r).a_max for r in model.rids})
    rep["workload"] = workload(legs_x, tl_x, model.rids, dt)
    rep["weld_stays"] = weld_stays(legs_x, model.rids, pl.dwell_s, pl.executor_margin_s)
    if rep["weld_stays"]["n_flagged"]:
        log(f"[trajgen] note: {rep['weld_stays']['n_flagged']} weld stays above {rep['weld_stays']['flag_above_s']} s "
            f"(max {rep['weld_stays']['max_stay_s']} s at {rep['weld_stays']['max_at']['weld']}): the robot waits "
            "for other robots with the tool on the weld point")
    rep["group_order_check"] = group_order_inversions(tl_x, task, group_order)
    if rep["group_order_check"]["n_inversions"]:
        log(f"[trajgen] note: {rep['group_order_check']['n_inversions']} cross-robot sequence-group inversions "
            f"(--group-order {group_order} orders the groups per arm only; use --group-order global for cell-wide "
            "group phases)")
    if verify:
        t1 = time.time()
        log("[trajgen] verifying the executor-clock timeline densely ...")
        light = R > 2
        skew_win = coord.skew_s if coord_mode == "priority" else checker.skew_n * dt
        # the dense verification and the plan-level lateness / lag checks are independent: one task each (parallel)
        vt = [("verify_executor", ("verify", tl_x, dict(light=light, skew_window_s=skew_win)))]
        if tl_m is not None:
            vt.append(("verify_motion", ("verify", tl_m, dict(light=True, skew_window_s=0.0, skew_s=()))))
        if lateness_s and lateness_s > 0 and R > 1:
            vt.append(("lateness_check", ("lateness", None, {})))
        if lag_check and R > 1:
            vt += [("lag_tolerance", ("lag", tl_x, {})), ("lateness_tolerance", ("ltol", None, {}))]
        from trajgen.parallel import pmap
        vres = dict(zip([k for k, _ in vt], pmap(_task_verify, dict(model=model, plan=plan, dt=dt, check_hz=check_hz,
                                                                   margin=pl.executor_margin_s, parity=umarell_parity,
                                                                   lateness_s=lateness_s), [x for _, x in vt],
                                                 W, log, "verification")))
        rep["verify_executor"] = vres["verify_executor"]
        if tl_m is not None:
            rep["verify_motion"] = vres["verify_motion"]
        rep["wall_s"]["verify"] = round(time.time() - t1, 1)
        v = rep["verify_executor"]
        ok_dyn = rep["phase_caps"]["max_vel_over_phase_cap"] <= 1.0 + 1e-6 and \
            rep["phase_caps"]["max_acc_over_phase_cap"] <= 1.0 + 0.02
        rep["verdict"] = {**v["checks"], "phase_caps": bool(ok_dyn),
                          "plan_consistent": bool(timing["max_start_error_s"] < 1e-6 and plan_vs_legs < 1e-6
                                                  and pc.get("player_replay_max_rad", 0.0) < 1e-5),
                          "complete": not partial}
        if "speed_profile" in rep:
            rep["verdict"]["speed_profile"] = bool(rep["speed_profile"]["pass"])
        if lateness_s and lateness_s > 0 and R > 1:     # gated: every leg completing lateness_s late
            lc = vres["lateness_check"]
            rep["lateness_check"] = lc
            rep["verdict"]["lateness"] = bool(lc["pass"])
        info_only = set() if umarell_parity else {"arm_arm_umarell_spheres"}   # informational without parity
        if partial:                                   # judged on the scheduled welds; still not complete
            info_only |= {"welds"}
            rep["verdict"]["welds_scheduled_ok"] = bool(v["welds"]["n_ok"] == len(v["welds"]["rows"]))
        rep["verdict"]["all_pass"] = bool(all(x for k, x in rep["verdict"].items() if k not in info_only))
        if info_only:
            rep["verdict"]["informational"] = sorted(info_only)
        if lag_check and R > 1:
            rep["lag_tolerance"] = vres["lag_tolerance"]
            rep["lateness_tolerance"] = vres["lateness_tolerance"]
            plan["behaviour_tree"]["metadata"]["lateness_tolerance_s"] = rep["lateness_tolerance"]["min_tolerated_s"]
        bad = sorted(k for k, x in rep["verdict"].items()
                     if isinstance(x, bool) and not x and k not in info_only and k not in ("all_pass", "complete"))
        if bad:
            log(f"[trajgen] VERIFICATION FAILED ({bad}): the plan is written but marked unverified")
            rep["verification_failed"] = bad
            if status == "complete":
                status = "unverified"
            rep["status"] = status
            plan["status"] = "unverified" if status == "unverified" else plan["status"]
            plan["behaviour_tree"]["metadata"]["verified"] = False
            plan["behaviour_tree"]["metadata"]["verification_failed"] = bad
            tl_x.meta["status"] = status
        else:
            plan["behaviour_tree"]["metadata"]["verified"] = True
    _thin_hints(rep)
    rep["wall_s"]["total"] = round(time.time() - t_wall, 1)
    paths = {}
    if out is not None:
        out = Path(out)
        out.mkdir(parents=True, exist_ok=True)
        for f in ("timeline_2robot.npz", "timeline_motion.npz"):
            if (out / f).exists():
                (out / f).unlink()
        paths = {"timeline": str(tl_x.save(out / "timeline.npz"))}
        if R == 2:
            paths["timeline_2robot"] = str(tl_x.to_player().save(out / "timeline_2robot.npz"))
        if tl_m is not None:
            paths["timeline_motion"] = str(tl_m.save(out / "timeline_motion.npz"))
        with open(out / "plan.json", "w") as f:
            json.dump(plan, f, indent=1)
        paths["plan"] = str(out / "plan.json")
        paths["report"] = str(out / "report.json")
        rep["outputs"] = paths
        with open(out / "report.json", "w") as f:
            json.dump(rep, f, indent=1, default=_jsonable)
    if "verdict" in rep:
        log(f"[trajgen] verdict {rep['verdict']}")
    log(f"[trajgen] {status}: executor end {rep['makespan']['executor_end_s']} s, {len(legs_x)} legs, "
        f"wall {rep['wall_s']['total']} s")
    return GenerateResult(tl_x, tl_m, plan, rep, legs_x, legs_m, paths)


OUTPUT_FILES = ("timeline.npz", "timeline_2robot.npz", "timeline_motion.npz", "plan.json", "report.json")


def clear_outputs(out):
    """Remove the generator's output files from ``out`` (a run never leaves an older plan or report behind)."""
    out = Path(out)
    for f in OUTPUT_FILES:
        if (out / f).exists():
            (out / f).unlink()


def workload(legs, tl, rids, dt) -> dict:
    """Per-robot busy time (motion without the stationary wait prefixes + holds + executor margins), idle time,
    finish time and yields; the per-robot busy lower bound of the makespan (max busy) and the makespan / bound."""
    rows = {}
    for r in rids:
        mine = [l for l in legs if l.robot == r]
        motion = sum(float(np.count_nonzero(l.phase[1:] != "wait")) * dt for l in mine)
        hold = sum(float(l.hold_s) for l in mine)
        margin = sum(float(l.t_done - l.t1 - l.hold_s) for l in mine)
        end = max([float(l.t_done) for l in mine] + [0.0])
        busy = motion + hold + margin
        rows[r] = {"welds": sum(1 for l in mine if l.kind == "weld"),
                   "yields": sum(1 for l in mine if l.kind == "move" and "_YIELD_" in str(l.op_id)),
                   "parks": sum(1 for l in mine if l.kind == "move" and "_PARK_" in str(l.op_id)),
                   "busy_s": round(busy, 3), "motion_s": round(motion, 3), "idle_s": round(end - busy, 3),
                   "end_s": round(end, 3)}
    lb = max([v["busy_s"] for v in rows.values()] + [0.0])
    mean = sum(v["busy_s"] for v in rows.values()) / max(1, len(rows))
    T = float(tl.meta.get("executor_end_s", tl.executor_end))
    ends = [v["end_s"] for v in rows.values() if v["end_s"] > 0]
    return {"per_robot": rows, "busy_lower_bound_s": round(lb, 3), "makespan_over_lb": round(T / lb, 3) if lb else None,
            "mean_busy_s": round(mean, 3), "makespan_over_mean_busy": round(T / mean, 3) if mean else None,
            "end_spread_s": round(max(ends) - min(ends), 3) if ends else 0.0,
            "note": "busy_lower_bound = max over robots of its own motion + holds + margins in THIS plan (no waits): "
                    "it depends on the allocation, so 1.00 does not mean balanced; mean_busy = sum(busy) / N is the "
                    "allocation-independent reference (perfect balance of this plan's work)"}


def weld_stays(legs, rids, dwell_s: float, margin_s: float, flag_factor: float = 2.0) -> dict:
    """How long each robot stays with the tool at a weld point: from the end of the descent to the first moving
    sample of its next leg (lift-off). The executor's minimum is dwell_s + executor_margin_s; waits for other robots
    (priority gates, Delays, steps-mode start delays) are spent there too, because the lift-off is part of the next
    leg (docs/trajgen.md 2.4). Stays above flag_factor x (dwell + margin) are flagged."""
    base = dwell_s + margin_s
    rows, per = [], {}
    for r in rids:
        mine = sorted([l for l in legs if l.robot == r], key=lambda l: l.t0)
        for a, b in zip(mine[:-1], mine[1:]):
            if a.kind != "weld":
                continue
            P = np.asarray(b.positions)
            mv = np.flatnonzero(np.abs(P - P[0]).max(1) > 1e-9)
            t_move = float(b.times[mv[0] - 1]) if len(mv) else float(b.t1)
            stay = t_move - float(a.t1)
            rows.append({"robot": r, "weld": a.op_id, "next": b.op_id, "stay_s": round(stay, 3),
                         "excess_s": round(stay - base, 3)})
            per[r] = max(per.get(r, 0.0), stay)
    rows.sort(key=lambda x: -x["stay_s"])
    flagged = [x for x in rows if x["stay_s"] > flag_factor * base + 1e-9]
    return {"min_expected_s": round(base, 3), "max_stay_s": rows[0]["stay_s"] if rows else None,
            "max_at": rows[0] if rows else None, "per_robot_max_s": {k: round(v, 3) for k, v in per.items()},
            "total_excess_s": round(sum(max(0.0, x["excess_s"]) for x in rows), 3),
            "n_flagged": len(flagged), "flag_above_s": round(flag_factor * base, 3), "longest": rows[:5],
            "note": "the tool stays on the weld point while the robot waits for other robots (the lift-off is the "
                    "start of the next leg); informational"}


def group_order_inversions(tl, task, group_order: str, n_examples: int = 5) -> dict:
    """Cross-robot sequence-group inversions: a weld of a later group (sequence_groups_order) whose dwell starts
    before a weld of an earlier group done by another robot has finished. ``--group-order per_arm`` (default)
    orders the groups within each arm only, so inversions are allowed there; ``global`` excludes them."""
    g = list(task.sequence_groups_order)
    if len(g) < 2:
        return {"group_order": group_order, "groups": g, "n_inversions": 0, "examples": []}
    rank = {w.id: (g.index(w.group) if w.group in g else len(g)) for w in task.welds}
    ev = [(r, w, t0, t1) for (r, w, t0, t1) in tl.weld_events if w in rank]
    inv = []
    for (ra, wa, a0, a1) in ev:
        for (rb, wb, b0, b1) in ev:
            if ra != rb and rank[wa] < rank[wb] and b0 < a1 - 1e-9:
                inv.append({"later": f"{wb} ({rb}, {task.weld(wb).group}) at {b0:.2f} s",
                            "earlier": f"{wa} ({ra}, {task.weld(wa).group}) done {a1:.2f} s"})
    return {"group_order": group_order, "groups": g, "n_inversions": len(inv), "examples": inv[:n_examples],
            "note": "per_arm orders the groups within each robot only; global plans each group as a cell-wide phase"}


def _priority_plan(res, model, dt, cell_id, sync=True):
    from trajgen.btplan import plan_from_sequences
    from trajgen.coord import legs_from_tracks
    legs_x, deps = legs_from_tracks(res.tracks, model.rids, dt, sync=sync)
    return plan_from_sequences(cell_id, legs_x, model.rids, deps)


def _order_search(coord, best_r, rids, mode, log, batch=None):
    """Priority-order search for one (assignment, window): 'auto' = hill climb over pairwise swaps from the best
    order tried so far (keep a swap when the executor makespan drops); 'all' = every permutation (<= 4 robots,
    else the hill climb). ``batch(orders) -> [results]`` evaluates several orders at once (parallel): every
    untried swap of a round is evaluated speculatively and the serial first-improvement rule is applied to them in
    the serial order, so the outcome is that of the serial climb (the speculative results that the serial climb
    would not have evaluated are kept aside and reused if a later round needs them)."""
    import itertools
    queues = best_r["queues_in"]
    tried = {tuple(o): v for o, v in (best_r.get("order_results") or [])}
    best_res = best_r["result"]
    cur = list(best_res.order)
    tried.setdefault(tuple(cur), round(best_res.objective_s, 3))
    best_new = None

    results = []
    spec = {}

    def prefetch(orders):
        todo = [tuple(o) for o in orders if tuple(o) not in tried and tuple(o) not in spec]
        if batch is not None and len(todo) > 1:
            for o, r in zip(todo, batch([list(o) for o in todo])):
                spec[o] = r

    def run(order):
        nonlocal best_res, best_new
        r = spec.pop(tuple(order), None)
        if r is None:
            r = coord.run_order(queues, list(order))
        tried[tuple(order)] = round(r.objective_s, 3) if r.ok else None
        if r.ok:
            results.append(r)
        log(f"[order-search] {list(order)}: " + (f"executor end {r.objective_s:.2f} s" if r.ok else "FAILED"))
        if r.ok and r.objective_s < best_res.objective_s - 1e-9:
            best_res = best_new = r
        return r

    if mode == "all" and len(rids) <= 4:
        prefetch(list(itertools.permutations(rids)))
        for perm in itertools.permutations(rids):
            if perm not in tried:
                run(perm)
    else:
        while True:                                   # first-improvement hill climb over pairwise swaps
            base, found = list(best_res.order), False
            sw = []
            for i in range(len(base)):
                for j in range(i + 1, len(base)):
                    o = list(base)
                    o[i], o[j] = o[j], o[i]
                    sw.append(o)
            prefetch(sw)
            for i in range(len(base)):
                for j in range(i + 1, len(base)):
                    o = list(base)
                    o[i], o[j] = o[j], o[i]
                    if tuple(o) not in tried and run(o) is best_res:
                        found = True
                        break
                if found:
                    break
            if not found:
                break
    vals = [v for v in tried.values() if v is not None]
    rep = {"mode": mode, "n_orders": len(tried), "orders": [[list(k), v] for k, v in tried.items()],
           "best_order": list(best_res.order), "best_executor_end_s": round(best_res.objective_s, 3),
           "spread_s": [min(vals), max(vals)] if vals else None, "improved": best_new is not None}
    return {"result": best_new, "report": rep, "results": results}


def _task_lateness_check(ctx, res):
    model, cell_id, dt, margin, delta, check_hz, sync = ctx
    from trajgen.verify import lateness_check
    lc = lateness_check(model, _priority_plan(res, model, dt, cell_id, sync), dt=dt, margin=margin, delta_s=delta,
                        check_hz=check_hz)
    return {"pass": bool(lc["pass"]), "sphere_min_m": {k: v["sphere_min_m"] for k, v in lc["patterns"].items()},
            "hull_min_m": {k: v["hull_min_m"] for k, v in lc["patterns"].items()}}


def _wide_search(rep, coord, cm, model, task, scene, pl, results, ok, best_r, elig, given, group_order, dt, R, W,
                 n_orders, order_search, lateness_s, repair_iters, repair_pool, lateness_pool, check_hz, sync, windows,
                 log, delay_search: bool = True, n_seeds: int = 2, max_moves: int = 48):
    """Round p3 ("wide") priority search after the portfolio runs (every order of every (assignment, window), run by
    the caller): (1) the balance neighbours of the best plan (two single-weld moves off the busiest robot, when the
    busy spread exceeds 5 s), every order each; (2) R > 4: the pairwise-swap order climb on the best; (3) the
    +lateness_s check of the ``lateness_pool`` shortest distinct plans (parallel) and the lateness repair of up to
    ``repair_pool`` failing plans shorter than the best passing one (parallel). The shortest passing plan wins
    (no passing plan: the shortest repaired one, which then fails the gated lateness check). Every stage is a set of
    independent pure evaluations, so the result does not depend on the worker count."""
    from trajgen.alloc import order_arm
    from trajgen.coord import priority_orders
    from trajgen.parallel import pmap
    t1 = time.time()
    # every evaluated (assignment, window, order) result: [(label, queues, window, CoordResult)]
    pool = []
    for r in results:
        for o, res in (r.get("all_results") or []):
            if res.ok:
                pool.append({"label": f"{r['label']}_o{len(pool)}", "run": r["label"], "queues_in": r["queues_in"],
                             "window": r["window"], "result": res})
    order_rep = {"mode": "all" if R <= 4 else order_search, "n_orders": len(pool),
                 "orders": [[list(x["result"].order), x["run"], round(x["result"].objective_s, 3)] for x in pool]}
    # (1) balance neighbours of the best plan
    bal = {"applied": False}
    if given is None and R >= 2:
        res = best_r["result"]
        busy = _busy(res, dt)
        spread = max(busy.values()) - min(busy.values())
        bal = {"busy_s": {r: round(v, 2) for r, v in busy.items()}, "spread_s": round(spread, 2), "threshold_s": 5.0,
               "tried": []}
        if spread > 5.0:
            rb = max(busy, key=busy.get)
            groups = task.sequence_groups_order if group_order != "ignore" else []
            q0 = {r: list(v) for r, v in best_r["queues_in"].items()}
            margin = coord.margin_n * dt

            def proxy(q):
                return {r: (order_arm(cm, r, q[r], groups, task)[1] + (len(q[r]) + 1) * margin) if q[r] else 0.0
                        for r in q}
            moves = []
            for w in q0.get(rb, []):
                for r2 in elig.get(w, []):
                    if r2 == rb:
                        continue
                    q = {r: list(v) for r, v in q0.items()}
                    q[rb].remove(w)
                    q[r2].append(w)
                    pa, pb = proxy({rb: q0[rb], r2: q0[r2]}), proxy({rb: q[rb], r2: q[r2]})
                    pred = dict(busy)
                    pred[rb] += pb[rb] - pa[rb]
                    pred[r2] += pb[r2] - pa[r2]
                    moves.append((max(pred.values()), w, r2, q))
            moves.sort(key=lambda x: x[0])
            todo = []
            for pmax, w, r2, q in moves[:2]:
                if pmax >= max(busy.values()) - 0.5:
                    continue
                qq = {r: order_arm(cm, r, q[r], groups, task)[0] if q[r] else [] for r in q}
                orders = priority_orders(model.rids, {}, 24 if R <= 4 else n_orders)
                if list(res.order) not in orders:
                    orders = [list(res.order)] + orders
                todo.append((pmax, w, r2, qq, orders))
            jobs = [(qq, o, best_r["window"], {}) for _, _, _, qq, orders in todo for o in orders]
            allr = coord.run_orders(jobs, W, label="balance neighbours") if jobs else []
            kk = 0
            for pmax, w, r2, qq, orders in todo:
                rs = allr[kk:kk + len(orders)]
                kk += len(orders)
                okr = [x for x in rs if x.ok]
                bal["tried"].append({"move": f"{w} {rb}->{r2}", "predicted_max_busy_s": round(pmax, 2),
                                     "executor_end_s": round(min(x.objective_s for x in okr), 3) if okr else None})
                for o, x in zip(orders, rs):
                    if x.ok:
                        pool.append({"label": f"{best_r['label']}_bal{len(bal['tried'])}_o{len(pool)}",
                                     "run": f"balance {w} {rb}->{r2}", "queues_in": qq, "window": best_r["window"],
                                     "result": x})
                log(f"[balance] {w} {rb}->{r2}: " + (f"best executor end {min(x.objective_s for x in okr):.2f} s"
                                                     if okr else "FAILED"))
    # (2) more than 4 robots: the order climb on the best
    if R > 4 and order_search != "off":
        coord.window = max(1, int(best_r.get("window", windows[0])))
        os_rep = _order_search(coord, best_r, model.rids, order_search, log,
                               batch=lambda os_, w_=coord.window, q_=best_r["queues_in"]: coord.run_orders(
                                   [(q_, o, w_, {}) for o in os_], W, label="order search"))
        for x in os_rep["results"]:
            pool.append({"label": f"{best_r['label']}_os{len(pool)}", "run": "order climb",
                         "queues_in": best_r["queues_in"], "window": best_r["window"], "result": x})
        order_rep["climb"] = os_rep["report"]
    # (2b) start-delay local search: on the shortest plans, delay a leg of a higher-priority robot that runs while
    # the last-finishing robot waits, by that robot's waiting leg (motion + hold) or half of it, and re-plan
    if delay_search and R >= 2:
        def dkey(x):
            return (json.dumps(x["queues_in"], sort_keys=True), tuple(x["result"].order), x["window"],
                    json.dumps(sorted((list(k), v) for k, v in (getattr(x["result"], "min_delay", None) or {}).items())))
        seeds, sk = [], set()
        for x in sorted(pool, key=lambda x: x["result"].objective_s):
            if dkey(x) not in sk:
                sk.add(dkey(x))
                seeds.append(x)
            if len(seeds) >= n_seeds:
                break
        jobs, meta = [], []
        for sd in seeds:
            res = sd["result"]
            tr, order = res.tracks, list(res.order)
            pos = {r: i for i, r in enumerate(order)}
            b = max(tr, key=lambda r: max([cl.k_done for cl in tr[r].legs] + [0]))
            md0 = dict(getattr(res, "min_delay", None) or {})
            moves = {}
            for cl in tr[b].legs:
                if cl.d <= 0:
                    continue
                w0, w1, span = cl.k_disp, cl.k_start, cl.n + cl.hold_n
                for r in order[:pos.get(b, 0)]:
                    legs_r = tr[r].legs
                    for i, cj in enumerate(legs_r):
                        if cj.kind not in ("weld", "home") or not (cj.k_start < w1 and cj.k_done > w0):
                            continue
                        prev = legs_r[i - 1] if i > 0 else None
                        if prev is not None and prev.kind == "park":      # parked: the unparked leg's delay
                            d_log = max(0, cj.k_start - prev.n - prev.k_disp)
                        else:
                            d_log = cj.d
                        key = (r, cj.target if cj.kind == "weld" else "home")
                        for dl in (span, span // 2):
                            md = {**md0, key: int(d_log + dl)}
                            mk = json.dumps(sorted((list(k), v) for k, v in md.items()))
                            ov = min(cj.k_done, w1) - max(cj.k_start, w0)
                            if mk not in moves or moves[mk][0] < ov:
                                moves[mk] = (ov, md, f"{key[0]}:{key[1]}+{dl * dt:.2f}s")
            for ov, md, lab in sorted(moves.values(), key=lambda x: -x[0])[:max_moves // max(1, len(seeds))]:
                jobs.append((sd["queues_in"], order, sd["window"], {}, md))
                meta.append((sd, lab))
        dres = coord.run_orders(jobs, W, label="start-delay search") if jobs else []
        n_better = 0
        for (sd, lab), x in zip(meta, dres):
            if x.ok:
                n_better += x.objective_s < sd["result"].objective_s - 1e-9
                pool.append({"label": f"{sd['label']}_d{len(pool)}", "run": f"delay {lab}", "queues_in": sd["queues_in"],
                             "window": sd["window"], "result": x})
        best_d = min((x.objective_s for x in dres if x.ok), default=None)
        order_rep["delay_search"] = {"seeds": [x["label"] for x in seeds], "n_moves": len(jobs), "n_better": int(n_better),
                                     "best_executor_end_s": None if best_d is None else round(best_d, 3)}
        log(f"[delay-search] {len(jobs)} moves on {len(seeds)} plans, {n_better} shorter"
            + (f", best {best_d:.2f} s" if best_d is not None else ""))
    # distinct plans, shortest first (stable: evaluation order)
    seen, uniq = set(), []
    for x in sorted(pool, key=lambda x: x["result"].objective_s):
        key = (json.dumps(x["queues_in"], sort_keys=True), tuple(x["result"].order), x["window"],
               json.dumps(sorted((list(k), v) for k, v in (getattr(x["result"], "min_delay", None) or {}).items())))
        if key not in seen:
            seen.add(key)
            uniq.append(x)
    best_unrep = uniq[0] if uniq else best_r
    order_rep.update(best_executor_end_s=round(best_unrep["result"].objective_s, 3),
                     best_order=list(best_unrep["result"].order),
                     spread_s=[round(uniq[0]["result"].objective_s, 3), round(uniq[-1]["result"].objective_s, 3)]
                     if uniq else None)
    rep["order_search"] = order_rep
    bal["applied"] = "_bal" in best_unrep["label"]
    rep["balance_step"] = bal
    if not (lateness_s and lateness_s > 0 and R > 1):
        rep["search_wide"] = {"wall_s": round(time.time() - t1, 1), "n_plans": len(uniq)}
        return best_unrep
    # (3) lateness check of the shortest plans, then the repair of shorter failing ones
    chk = uniq[:max(1, lateness_pool)]
    lctx = (model, scene.cell_id, dt, pl.executor_margin_s, lateness_s, check_hz, sync)
    lcs = pmap(_task_lateness_check, lctx, [x["result"] for x in chk], W, log, "lateness check")
    passing = [x for x, lc in zip(chk, lcs) if lc["pass"]]
    p_best = min((x["result"].objective_s for x in passing), default=np.inf)
    to_fix = [x for x, lc in zip(chk, lcs) if not lc["pass"] and x["result"].objective_s < p_best - 1e-9]
    to_fix = to_fix[:max(0, repair_pool)] if repair_iters > 0 else []
    lr_args = (model, scene.cell_id, dt, pl.executor_margin_s, lateness_s, repair_iters, check_hz, sync, log,
               windows[0])
    coord._fork_keys = set(coord._legs)
    lres = pmap(_task_lateness_repair, (coord, lr_args), to_fix, W, log, "lateness repair") if to_fix else []
    coord.min_delay = {}
    cands = [(x, x["result"], {"label": x["label"], "pass": True, "iterations": 0,
                               "executor_end_before_s": round(x["result"].objective_s, 3),
                               "executor_end_after_s": round(x["result"].objective_s, 3), "widened_legs": {}})
             for x in passing]
    for x, (res_r, lr, new) in zip(to_fix, lres):
        coord.merge_legs(new)
        lr["label"] = x["label"]
        cands.append((x, res_r, lr))
    good = [c for c in cands if c[2]["pass"]]
    pick = min(good or cands, key=lambda c: c[1].objective_s) if (good or cands) else (best_unrep, best_unrep["result"], {
        "label": best_unrep["label"], "pass": False, "iterations": 0, "executor_end_before_s": None,
        "executor_end_after_s": None, "widened_legs": {}})
    x, res_r, lr = pick
    out = {**lr, "delta_s": lateness_s,
           "checked": [{"label": c["label"], "executor_end_s": round(c["result"].objective_s, 3), **lc}
                       for c, lc in zip(chk, lcs)],
           "candidates": [{"label": c[2]["label"], "pass": c[2]["pass"], "iterations": c[2]["iterations"],
                           "executor_end_before_s": c[2]["executor_end_before_s"],
                           "executor_end_after_s": c[2]["executor_end_after_s"],
                           "widened_legs": c[2]["widened_legs"]} for c in cands],
           "chosen": x["label"]}
    rep["lateness_repair"] = out
    rep["search_wide"] = {"wall_s": round(time.time() - t1, 1), "n_plans": len(uniq), "n_checked": len(chk),
                          "n_passing": len(passing), "n_repaired": len(to_fix)}
    log(f"[search] {len(uniq)} plans, {len(passing)}/{len(chk)} pass +{lateness_s:g} s, {len(to_fix)} repaired; "
        f"chosen {x['label']} {res_r.objective_s:.2f} s (pass {lr['pass']})")
    return {**x, "label": x["label"] + ("_lr" if res_r is not x["result"] else ""), "result": res_r}


def _busy(res, dt):
    """{rid: busy s} of a coordinator result: motion + hold + margin of every leg (no stationary prefixes)."""
    return {r: sum((cl.n + cl.hold_n + cl.margin_n) for cl in t.legs) * dt for r, t in res.tracks.items()}


def _balance_step(coord, cm, model, task, best_r, elig, group_order, dt, n_orders, log, spread_s: float = 5.0,
                  n_moves: int = 2, workers: int = 1):
    from trajgen.alloc import order_arm
    from trajgen.coord import priority_orders
    res = best_r["result"]
    busy = _busy(res, dt)
    spread = max(busy.values()) - min(busy.values())
    rep = {"busy_s": {r: round(v, 2) for r, v in busy.items()}, "spread_s": round(spread, 2), "threshold_s": spread_s}
    if spread <= spread_s:
        rep["applied"] = False
        return {"report": rep, "result": None, "queues": None}
    rb = max(busy, key=busy.get)
    groups = task.sequence_groups_order if group_order != "ignore" else []
    q0 = {r: list(v) for r, v in best_r["queues_in"].items()}
    margin = coord.margin_n * dt

    def proxy(q):
        return {r: (order_arm(cm, r, q[r], groups, task)[1] + (len(q[r]) + 1) * margin) if q[r] else 0.0 for r in q}

    moves = []
    for w in q0.get(rb, []):
        for r2 in elig.get(w, []):
            if r2 == rb:
                continue
            q = {r: list(v) for r, v in q0.items()}
            q[rb].remove(w)
            q[r2].append(w)
            # predicted: the coordinated busy of the others + the proxy change of the two robots involved
            pa, pb = proxy({rb: q0[rb], r2: q0[r2]}), proxy({rb: q[rb], r2: q[r2]})
            pred = dict(busy)
            pred[rb] += pb[rb] - pa[rb]
            pred[r2] += pb[r2] - pa[r2]
            moves.append((max(pred.values()), w, r2, q))
    moves.sort(key=lambda x: x[0])
    tried, best = [], None
    todo = []
    for pmax, w, r2, q in moves[:n_moves]:
        if pmax >= max(busy.values()) - 0.5:
            continue
        qq = {r: order_arm(cm, r, q[r], groups, task)[0] if q[r] else [] for r in q}
        orders = [list(res.order)] + [o for o in priority_orders(model.rids, {}, n_orders)[:1] if o != list(res.order)]
        todo.append((pmax, w, r2, qq, orders))
    jobs = [(qq, o, coord.window, {}) for _, _, _, qq, orders in todo for o in orders]
    allr = coord.run_orders(jobs, workers, label="balance step") if jobs else []
    k = 0
    for pmax, w, r2, qq, orders in todo:
        r_ = coord.select(orders, allr[k:k + len(orders)])
        k += len(orders)
        tried.append({"move": f"{w} {rb}->{r2}", "predicted_max_busy_s": round(pmax, 2),
                      "executor_end_s": round(r_.objective_s, 3) if r_.ok else None})
        log(f"[balance] {w} {rb}->{r2}: " + (f"executor end {r_.objective_s:.2f} s" if r_.ok else "FAILED"))
        if r_.ok and r_.objective_s < (best[0].objective_s if best else res.objective_s) - 1e-9:
            best = (r_, qq)
    rep.update(applied=best is not None, tried=tried, executor_end_before_s=round(res.objective_s, 3),
               executor_end_after_s=round(best[0].objective_s, 3) if best else round(res.objective_s, 3))
    return {"report": rep, "result": best[0] if best else None, "queues": best[1] if best else None}


def _task_lateness_repair(ctx, cand):
    coord, (model, cell_id, dt, margin, delta, iters, check_hz, sync, log, w0) = ctx
    coord.window = max(1, int(cand.get("window", w0)))
    coord.min_delay = dict(getattr(cand["result"], "min_delay", None) or {})
    res_r, lr = _lateness_repair(coord, model, cand, cell_id, dt, margin, delta, iters, check_hz, sync, log)
    return res_r, lr, coord.new_legs()


def _lateness_repair(coord, model, best_r, cell_id, dt, margin, delta, iters, check_hz, sync, log):
    """Gated lateness check + repair (priority plans): replay with every leg delta late (all robots, each robot
    alone); for every violating (pair, legs) widen the skew window of the later-planned robot's active leg by delta
    and re-plan the same assignment / order; up to ``iters`` re-plans. Returns (result, report)."""
    from trajgen.verify import lateness_check
    res = best_r["result"]
    queues = best_r["queues_in"]
    order = list(res.order)
    pos = {r: i for i, r in enumerate(order)}
    step = max(1, int(round(delta / coord.dt)))
    cap = coord.w_base + 6 * step
    hist = []
    coord.w_extra = {}
    for it in range(iters + 1):
        plan = _priority_plan(res, model, dt, cell_id, sync)
        lc = lateness_check(model, plan, dt=dt, margin=margin, delta_s=delta, check_hz=check_hz)
        hist.append({"iteration": it, "executor_end_s": round(res.objective_s, 3), "pass": lc["pass"],
                     "patterns": {k: {"pass": v["pass"], "sphere_min_m": v["sphere_min_m"], "hull_min_m": v["hull_min_m"],
                                      "violations": v["violations"][:4]} for k, v in lc["patterns"].items()}})
        log(f"[lateness] iteration {it}: executor end {res.objective_s:.2f} s, +{delta:g} s/leg check "
            + ("pass" if lc["pass"] else "FAIL " + str({k: v["sphere_min_m"] for k, v in lc["patterns"].items()
                                                        if not v["pass"]})))
        if lc["pass"] or it == iters:
            break
        changed = False
        keys = []
        for v in (x for pat in lc["patterns"].values() for x in pat["violations"]):
            rs = list(v["legs"])
            later = max(rs, key=lambda r: pos.get(r, -1))
            op = v["legs"][later]
            if op == "after_last":
                later = min(rs, key=lambda r: pos.get(r, -1))
                op = v["legs"][later]
                if op == "after_last":
                    continue
            if (later, op) not in keys:
                keys.append((later, op))
        for key in keys:                                  # one step per leg and iteration
            new = coord.w_for(*key) + step
            if new <= cap:
                coord.w_extra[key] = new
                changed = True
        if not changed:
            break
        r2 = coord.run_order(queues, order)
        if not r2.ok:
            log(f"[lateness] re-plan with widened windows failed: {r2.failed}; keeping the previous plan")
            hist[-1]["replan_failed"] = r2.failed
            break
        res = r2
    final = hist[-1]
    rep = {"delta_s": delta, "pass": bool(final["pass"]), "iterations": len(hist) - 1,
           "widened_legs": {f"{k[0]}|{k[1]}": round(v * coord.dt, 3) for k, v in coord.w_extra.items()},
           "executor_end_before_s": hist[0]["executor_end_s"], "executor_end_after_s": final["executor_end_s"],
           "history": hist}
    coord.w_extra_final = dict(coord.w_extra)
    return res, rep


def _task_step_search(ctx, run):
    """One step search (steps coordination): Search(...).run(), its executable steps and the early-HOME post-pass
    (trajgen.pipeline.generate, in a worker when parallel)."""
    from trajgen.search import Search
    label, queues, win, hw = run[:4]
    beam, k_cand = run[4:6] if len(run) > 4 else (32, 6)
    t1 = time.time()
    srch = Search(ctx["model"], ctx["planner"], ctx["checker"], ctx["reach"], ctx["cm"], ctx["task"], queues=queues,
                  window=win or 1, h_weight=hw, beam=beam, k_cand=k_cand, **ctx["kw"])
    srch.target = frozenset(w for w in srch.target if w in ctx["target"])
    final = srch.run()
    res = {"label": label, "queues_in": queues, "stats": srch.stats, "unreachable": srch.unreachable}
    if final is None:
        res.update(complete=False, steps=[], objective=np.inf, early_home=([], []), park=([], []))
    else:
        steps = srch.steps_of(final)
        eh = srch.early_home(steps)
        pk = srch.park_steps(eh[0]) if ctx.get("park") else (eh[0], [])
        fin = pk[0]
        margin = ctx["kw"]["executor_margin"]
        stay = 0.0                       # time an arm rests on a weld while the other moves (excl. its own leg)
        for k_, st in enumerate(fin):
            for r_ in srch.rids:
                if r_ in st.legs:
                    continue
                prev = [s_.legs[r_] for s_ in fin[:k_] if r_ in s_.legs]
                if prev and prev[-1].target not in ("home", "park"):
                    stay += (st.t_end - st.t0) + margin
        res.update(complete=True, steps=steps, objective=final.t, makespan_motion=final.t_motion, by=final.by,
                   early_home=eh, park=pk, final_s=round(sum(s_.t_end - s_.t0 for s_ in fin) + margin * len(fin), 6),
                   stay_excess_s=round(stay, 6), beam=beam, k_cand=k_cand)
    res["wall_s"] = time.time() - t1
    return res


def _search_rows(results):
    return [{"label": r["label"], "queues_in": r["queues_in"], "complete": r["complete"],
             "objective_s": None if not r["complete"] else round(r["objective"], 3),
             "executor_end_after_post_passes_s": r.get("final_s"), "rest_on_weld_s": r.get("stay_excess_s"),
             "beam": r.get("beam"), "k_cand": r.get("k_cand"), "steps": len(r["steps"]),
             "wall_s": round(r["wall_s"], 1), "stats": r["stats"]} for r in results]


def _eligible_reach(reach, eligible):
    """reach restricted to the eligible options (indices preserved by building new lists in option order)."""
    out = _Reach()
    for (w, r), ops in reach.items():
        ks = eligible.get((w, r))
        out[(w, r)] = ops if ks is None else [ops[k] for k in ks]
    out.reasons = getattr(reach, "reasons", {})
    return out


def _homes_clear(model, checker):
    """All robots standing at HOME must be clear of each other (corrected spheres, parity, hulls)."""
    rids = model.rids
    for i, a in enumerate(rids):
        for b in rids[i + 1:]:
            qa, qb = model.robots[a].home[None], model.robots[b].home[None]
            c = float(model.pair_clearance(a, b, qa, qb)[0])
            if c <= checker.gate:
                return f"{a} and {b} at HOME: sphere clearance {c:.3f} m <= gate {checker.gate:.3f} m"
            if checker.parity:
                cu = float(model.umarell_pair_clearance(a, b, qa, qb)[0])
                if cu <= checker.gate_u:
                    return f"{a} and {b} at HOME: umarell-sphere clearance {cu:.3f} m <= gate {checker.gate_u:.3f} m"
            if checker.hull and model.mj_pair_contact(a, b, qa, qb, checker.hull_thr)[0]:
                dmin = float(model.mj_pair_min_distance(a, b, qa, qb)[0])
                return (f"{a} and {b} at HOME: hull distance {dmin:.3f} m < hull_min_distance_m "
                        f"{checker.hull_thr:.3f} m")
    return True


def _homes_valid(model) -> list:
    """[(robot, reason)] for every robot whose HOME violates a single-arm rule: joint limits, floor (spheres and
    hulls), boxes (spheres and hulls), rule W, hull self-distance below the scene's self_collision.min_distance_m."""
    out = []
    smin = model.scene.collision.self_min_distance
    for r in model.rids:
        h = model.robots[r].home
        why = [x for x in model.static_reasons(r, h) if x != "self_collision"]
        sd = float(model.ik.self_distance(r, h))
        if np.isfinite(smin) and sd < smin:
            why.append(f"self_collision (hull self-distance {sd * 1000:.1f} mm < {smin * 1000:.1f} mm)")
        if why:
            out.append((r, f"{r}: HOME violates {', '.join(why)}"))
    return out


def _task_verify(ctx, item):
    """One verification task of generate (trajgen.verify functions, unchanged): the dense timeline check, the
    gated lateness check, the lag and lateness tolerances."""
    kind, tl, kw = item
    m, plan, dt, hz, margin = ctx["model"], ctx["plan"], ctx["dt"], ctx["check_hz"], ctx["margin"]
    if kind == "verify":
        return verify_timeline_safe(m, tl, hz, ctx["parity"], **kw)
    from trajgen.verify import lag_tolerance, lateness_check, lateness_tolerance
    if kind == "lateness":
        return lateness_check(m, plan, dt=dt, margin=margin, delta_s=ctx["lateness_s"], check_hz=hz)
    if kind == "lag":
        return lag_tolerance(m, plan, tl, dt=dt, margin=margin, check_hz=hz)
    return lateness_tolerance(m, plan, dt=dt, margin=margin, check_hz=hz)


def verify_timeline_safe(model, tl, check_hz, umarell_parity=True, light=False, skew_window_s=0.1, skew_s=(0.05, 0.1)):
    from trajgen.verify import verify_timeline
    return verify_timeline(model, tl, check_hz=check_hz, hull_dist_every=10 if light else 5,
                           self_dist_every=5 if light else 1, umarell_parity=umarell_parity,
                           skew_window_s=skew_window_s, skew_s=skew_s)


def _jsonable(o):
    if hasattr(o, "tolist"):
        return o.tolist()
    if isinstance(o, (set, tuple, frozenset)):
        return list(o)
    if isinstance(o, (np.bool_,)):
        return bool(o)
    return str(o)
