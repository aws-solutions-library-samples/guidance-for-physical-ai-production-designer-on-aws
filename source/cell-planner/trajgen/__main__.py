"""CLI: python -m trajgen --scene <scene.yaml> --task <task.json> --out <dir> [--assignment auto|<json>|<file>]
[--speed umarell|fast|full] [--coordination auto|steps|priority] [--allow-partial] [--seed N]
(see trajgen/pipeline.py and docs/trajgen.md)

Exit codes: 0 complete plan, verification passed (or --no-verify); 1 plan written but the dense verification failed
(report.json and plan.json status "unverified", or a partial plan that fails a check);
2 failed: no plan written (report.json lists the failed welds and the reasons; also for scene / assignment errors);
3 partial plan written (--allow-partial; plan.json and report.json are marked partial);
4 internal error (uncaught exception): report.json has status "failed" and the traceback, no plan.
Every run first removes the generator's older output files from --out."""
from __future__ import annotations

import argparse
import json
import sys
import traceback
from pathlib import Path


def _weights(s):
    try:
        w = tuple(float(x) for x in str(s).split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected a comma list of numbers, e.g. 1.0,1.6; got {s!r}") from None
    if not w or any(x <= 0 for x in w):
        raise argparse.ArgumentTypeError("heuristic weights must be > 0")
    return w


def _portfolio(v: str):
    v = (v or "").strip().lower()
    if v == "auto":
        return "auto"
    if v in ("", "none", "off"):
        return None
    return [tuple(int(x) for x in p.split("x")) for p in v.split(",")]


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m trajgen", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", required=True, help="scene.yaml (or its directory)")
    ap.add_argument("--task", default=None, help="task.json (default: next to the scene)")
    ap.add_argument("--out", required=True, help="output directory (timeline.npz, timeline_2robot.npz for two robots, "
                    "timeline_motion.npz for steps coordination, plan.json, report.json)")
    ap.add_argument("--assignment", default="auto", help='auto | JSON {"robot1": ["W03", ...], ...} | path to such a file')
    ap.add_argument("--speed", choices=["umarell", "fast", "full"], default=None,
                    help="umarell: the scene's planning rules; fast: accel a_max*v_s, HOME at transit speed; full: "
                         "every non-weld motion at 1.0 x the URDF velocity limits, the scene's acceleration and a jerk "
                         "limit (S-curve ramps), weld descent / hold / lift-off as umarell (docs/p3_tgfull.md). "
                         "Default: the scene's planning.speed_profile, else umarell")
    ap.add_argument("--coordination", choices=["auto", "steps", "priority"], default="auto",
                    help="auto: steps (two-arm sync-at-rest beam search) for 2 robots, else priority (prioritized planning)")
    ap.add_argument("--allow-partial", action="store_true", help="write the plan of the schedulable welds (marked partial)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--candidates", type=int, default=None,
                    help="allocation candidates searched / coordinated (default: 1 for steps, 2 for priority)")
    ap.add_argument("--beam", type=int, default=32, help="beam width of the step search (steps)")
    ap.add_argument("--k-cand", type=int, default=6, help="candidate welds per arm and state (steps)")
    ap.add_argument("--window", type=int, default=None,
                    help="queue head window (welds considered next per robot); default 2, and for priority with 3+ "
                         "robots both 2 and 3 are tried (best executor end wins)")
    ap.add_argument("--orders", type=int, default=4, help="priority orders tried per allocation (priority)")
    ap.add_argument("--skew", type=float, default=0.1, help="dispatch skew tolerance of the priority schedule (s)")
    ap.add_argument("--no-sync-waits", action="store_true", help="priority: no WaitForRobot gates in the plan")
    ap.add_argument("--group-order", choices=["per_arm", "global", "ignore"], default="per_arm")
    par = ap.add_mutually_exclusive_group()
    par.add_argument("--no-umarell-parity", action="store_true",
                     help="arm-arm gate on the corrected spheres only (default: as the scene declares)")
    par.add_argument("--umarell-parity", action="store_true", help="require the parity gate (scene must declare umarell)")
    ap.add_argument("--allow-thin", action="store_true",
                    help="allow weld solutions below the margin preference even where another arm has a comfortable one")
    ap.add_argument("--objective", choices=["executor", "motion"], default="executor")
    ap.add_argument("--h-weights", type=_weights, default=(1.0, 1.6), help="heuristic weights of the steps portfolio")
    ap.add_argument("--check-hz", type=float, default=None, help="dense check rate (default max(100, 1/check_dt))")
    ap.add_argument("--clearance-margin", type=float, default=0.01, help="planning margin above the arm-arm gate (m)")
    ap.add_argument("--jerk-factor", type=float, default=None, help="jerk cap in a_max/s (default: scene, else 4 / 8 fast; full: the "
                    "non-weld jerk, default planning.full_speed.jerk_factor, else 10)")
    ap.add_argument("--max-tilt", type=float, default=3.5, help="max tool-axis tilt of a weld solution (deg)")
    ap.add_argument("--margin", type=float, default=0.015, help="preferred hull self-distance along a stroke (m)")
    ap.add_argument("--n-random", type=int, default=120, help="random IK seeds per weld and robot")
    ap.add_argument("--electrode-rule", choices=["scene", "on", "off"], default="scene",
                    help="electrode off-weld rule (docs/n2_contract.md 3.1): as the scene declares "
                         "(obstacles.box_hull.electrode_off_weld; scenegen v2 scenes enable it), or force it on / off")
    ap.add_argument("--nodip", choices=["scene", "on", "off"], default="scene",
                    help="round-3 no-dip rule (trajgen.nodip, docs/r3_nodip.md): as the scene declares "
                         "(obstacles.box_hull.no_dip; absent = $TRAJGEN_NODIP, else the code default), or force it "
                         "on / off")
    ap.add_argument("--no-verify", action="store_true")
    ap.add_argument("--no-cache", action="store_true")
    ap.add_argument("--name", default=None)
    ap.add_argument("--no-lag-check", action="store_true",
                    help="skip the informational lag / lateness-tolerance scans")
    ap.add_argument("--lateness", type=float, default=0.2,
                    help="gated per-leg completion lateness (s) the plan must tolerate (all robots, and each robot "
                         "alone); priority plans are repaired by widening leg skew windows; 0 = off")
    ap.add_argument("--repair-iters", type=int, default=6, help="lateness repair re-plans (priority)")
    ap.add_argument("--order-search", choices=["auto", "all", "off"], default="auto",
                    help="priority (3+ robots): after the portfolio, hill climb over pairwise swaps of the best "
                         "priority order (auto) or try every order (all, <= 4 robots)")
    ap.add_argument("--workers", type=int, default=None,
                    help="worker processes for the parallel stages (default: TRAJGEN_WORKERS, else the CPU count; "
                         "1 = serial). The plan does not depend on it")
    ap.add_argument("--no-park", action="store_true",
                    help="wait with the tool on the weld point (round-4 behaviour) instead of lifting off first and "
                         "waiting at the retract pose")
    ap.add_argument("--search", choices=["wide", "round4"], default="wide",
                    help="priority: 'wide' = every order (<= 4 robots) of every assignment / window + balance "
                         "neighbours, lateness check of the shortest plans and repair of shorter failing ones; "
                         "'round4' = the round-4 flow")
    ap.add_argument("--repair-pool", type=int, default=6, help="priority, wide: plans given the lateness repair")
    ap.add_argument("--lateness-pool", type=int, default=16, help="priority, wide: shortest plans lateness-checked")
    ap.add_argument("--no-delay-search", action="store_true", help="priority, wide: no start-delay local search")
    ap.add_argument("--steps-portfolio", default="auto",
                    help="steps: (beam)x(k_cand) pairs searched, e.g. '32x6,32x8,64x6'; 'auto' = (beam, k), (beam, k+2), "
                         "(2 beam, k); 'none' = (beam, k) only (round 4)")
    a = ap.parse_args(argv)
    from trajgen.pipeline import AssignmentError, clear_outputs, generate, parse_assignment
    from trajgen.scene import SceneError

    def fail(err: str, code: int, tb: str | None = None):
        rep = {"generator": "trajgen", "status": "failed", "error": err, "exit_code": code,
               "scene": a.scene, "task": a.task, "failed_welds": []}
        if tb:
            rep["traceback"] = tb
        try:
            out = Path(a.out)
            out.mkdir(parents=True, exist_ok=True)
            clear_outputs(out)
            with open(out / "report.json", "w") as f:
                json.dump(rep, f, indent=1)
        except OSError:
            pass
        print(json.dumps({k: v for k, v in rep.items() if k != "traceback"}, indent=1))
        if tb:
            print(tb, file=sys.stderr)
        return code

    try:
        assignment = parse_assignment(a.assignment)        # before any computation
    except AssignmentError as e:
        return fail(str(e), 2)
    parity = False if a.no_umarell_parity else (True if a.umarell_parity else None)
    try:
        res = generate(a.scene, a.task, out=a.out, assignment=assignment or "auto", speed=a.speed, seed=a.seed,
                       n_candidates=a.candidates, check_hz=a.check_hz, clearance_margin=a.clearance_margin,
                       jerk_factor=a.jerk_factor, max_tilt_deg=a.max_tilt, margin_m=a.margin, verify=not a.no_verify,
                       cache=not a.no_cache, name=a.name, beam=a.beam, k_cand=a.k_cand, group_order=a.group_order,
                       umarell_parity=parity, allow_thin=a.allow_thin, objective=a.objective, h_weights=a.h_weights,
                       coordination=a.coordination, allow_partial=a.allow_partial, skew_s=a.skew, n_orders=a.orders,
                       window=a.window, sync_waits=not a.no_sync_waits, n_random=a.n_random,
                       lag_check=not a.no_lag_check, lateness_s=a.lateness, repair_iters=a.repair_iters,
                       order_search=a.order_search, workers=a.workers,
                       park=not a.no_park, search=a.search, repair_pool=a.repair_pool,
                       lateness_pool=a.lateness_pool, delay_search=not a.no_delay_search,
                       steps_portfolio=_portfolio(a.steps_portfolio),
                       electrode_rule={"scene": None, "on": True, "off": False}[a.electrode_rule],
                       nodip={"scene": None, "on": True, "off": False}[a.nodip])
    except (SceneError, AssignmentError) as e:
        return fail(f"{type(e).__name__}: {e}", 2)
    except Exception as e:  # noqa: BLE001
        return fail(f"internal error: {type(e).__name__}: {e}", 4, traceback.format_exc())
    r = res.report
    print(json.dumps({"status": res.status, "makespan": r.get("makespan"), "verdict": r.get("verdict"),
                      "failed_welds": r.get("failed_welds"), "outputs": res.paths}, indent=1, default=str))
    if res.status == "failed":
        return 2
    if res.status == "partial":
        return 1 if res.report.get("verification_failed") else 3
    if res.status == "unverified":
        return 1
    return 0 if (a.no_verify or res.ok) else 1


if __name__ == "__main__":
    sys.exit(main())
