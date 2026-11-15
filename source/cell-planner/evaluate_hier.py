"""Evaluate the hierarchical scheduler (Track H): canonical task + held-out randomised programs.

    PYTHONPATH=. .venv/bin/python evaluate_hier.py --checkpoint runs/hier_f/selected_f65250.pt --legacy-v2 \
        --tag hier_final_policy --episodes 200 --workers 3 [--export] [--speed-profile umarell_rule]
    PYTHONPATH=. .venv/bin/python evaluate_hier.py --policy heuristic --tag hier_x_heuristic --episodes 200
    PYTHONPATH=. .venv/bin/python evaluate_hier.py --policy expert --tag hier_x_expert --episodes 200

Canonical: the exact 15-weld program from HOME (IK solved fresh at the canonical poses), greedy policy. NOTE:
for runs trained with --p-canonical > 0 the canonical program is a TRAINING (and selection) program.
Held-out: seeds 50000+ (never used for training episodes): weld subsets (40 % all 15, else uniform 2..15), +-3 cm
noise per weld solved FRESH with the IK (never in any pool), start noise 0.1 rad. ``--ood`` switches to the
out-of-distribution set (seeds 60000+): +-5 cm noise, 8-12 welds, start noise 0.25 rad.
Every 20 Hz step runs through ``KukaWeldEnv`` (umarell sphere gate + MuJoCo hull self / arm-arm / floor [+ fixture]).

Per episode: env-clock makespan (last weld done), min clearance over all env sub-steps and a dense 400 Hz minimum,
and the executor clock (``player.bt_plan.legs_from_rollout`` of the rollout + return HOME, the same export as
``--export``: last weld done = weld-leg arrival + 0.5 s hold, motion end, executor end).
``--speed-profile``: executor limits (rl.hier.executor.PROFILES); the expert is run with the matching transit
speed / acceleration / descent speed (its HOME moves use the transit limits; its descent has no acceleration limit).
``--export`` (canonical): timeline (player.timeline, weld_events) -> ``--timeline``, umarell BT plan ->
results/<tag>_plan_canonical.json for scripts/umarell_validate.py.
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent

_G = {}
OOD = dict(pos_noise=0.05, n_welds=(8, 12), p_full=0.0, start_noise=0.25)


def make_policy(spec, thr, no_lookahead):
    if spec == "expert":
        return None
    if spec == "heuristic":
        from rl.hier.heuristic import heuristic_action
        return lambda obs: heuristic_action(obs)
    import torch
    torch.set_num_threads(1)
    from rl.hier.features import blind
    from rl.hier.model import HierQNet, greedy_action
    sd = torch.load(spec, map_location="cpu", weights_only=False)
    net = HierQNet(**sd["cfg"])
    net.load_state_dict(sd["net"])
    net.eval()
    thr = thr if thr > 0 else None
    see = blind if no_lookahead else (lambda o: o)
    return lambda obs: greedy_action(net, see(obs), thr=thr)


def make_cfg(o, record):
    from rl.hier.hier_env import HierConfig
    kw = dict(record=record, cell_id=o["cell"], speed_profile=o["speed_profile"], max_tilt_deg=o["max_tilt"],
              min_self_dist=o["min_self_dist"])
    if o["legacy_v2"]:
        kw["time_feature"] = "elapsed"           # same (dynamic) time limit for every policy
    if o["ood"]:
        kw.update(pos_noise=OOD["pos_noise"], n_welds=OOD["n_welds"], p_full=OOD["p_full"], start_noise=OOD["start_noise"])
    return HierConfig(**kw)


def _init(o, record):
    import os
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    from rl.hier.hier_env import HierWeldEnv
    from rl.hier.instances import InstanceSolver, load_pool
    _G["o"] = o
    _G["policy"] = make_policy(o["spec"], o["safe_thr"], o["no_lookahead"])
    cfg = make_cfg(o, record=True)               # always record: exec clock + dense clearance
    if o["select"]:      # model selection: the eval pool (seed 777 offsets), like train_hier's evaluator
        _G["env"] = HierWeldEnv(cfg, pool=load_pool(o["cell"], tag="eval"))
    else:
        _G["env"] = HierWeldEnv(cfg, solver=InstanceSolver(o["cell"]))


# ---------------------------------------------------------------------------- per-episode metrics
def dense_min_clearance(sm, q, sub=20):
    """Min umarell sphere clearance along the piecewise-linear joint path q (N,2,6) at sub x 20 Hz (400 Hz)."""
    q = np.asarray(q, float)
    if len(q) < 2:
        return float(sm.pair_clearance(q[0, 0], q[0, 1]))
    f = (np.arange(sub) / sub)[None, :, None, None]
    qq = (q[:-1, None] + f * (q[1:, None] - q[:-1, None])).reshape(-1, 2, 6)
    qq = np.concatenate([qq, q[-1:]])
    out = np.inf
    for s in range(0, len(qq), 4000):
        out = min(out, float(sm.pair_clearance(qq[s:s + 4000, 0], qq[s:s + 4000, 1]).min()))
    return out


def home_tail(env_wrap_or_env, ex, q0):
    """Both arms HOME after the last weld (executor home primitives, every sample + midpoint checked with
    env.check_collisions). Tries simultaneous, then robot1-first, then robot2-first."""
    env = env_wrap_or_env
    for order in ("both", (0, 1), (1, 0)):
        qs, ok, clr = [], True, np.inf
        if order == "both":
            p = [ex.go_home(r, q0[i]).wps for i, r in enumerate(("robot1", "robot2"))]
            n = max(len(x) for x in p)
            for k in range(n):
                qs.append(np.stack([p[i][min(k, len(p[i]) - 1)] for i in range(2)]))
        else:
            q = q0.copy()
            for i in order:
                for w in ex.go_home(("robot1", "robot2")[i], q[i]).wps:
                    q = q.copy(); q[i] = w
                    qs.append(q)
        prev = q0
        for q in qs:
            for f in (0.5, 1.0):
                c = env.check_collisions(prev + f * (q - prev))
                clr = min(clr, c["min_clearance"])
                if c["collision"]:
                    ok = False
            prev = q
            if not ok:
                break
        if ok:
            return dict(order=str(order), q=np.array(qs).reshape(-1, 2, 6), min_clearance=float(clr), ok=True)
    # fallback (fixture cell): one arm after the other with Track X's collision-checked transit planner
    # (direct / raised / via HOME / RRT-Connect against self, floor and fixture), arm-arm checked here
    if "homeplanner" not in _G:
        from expert.scripted import ScriptedExpert
        from rl.hier.instances import table_path
        _G["homeplanner"] = ScriptedExpert(env, table_path=table_path(env.cell.cell_id))
    pl = _G["homeplanner"]
    for order in ((0, 1), (1, 0)):
        q, qs, ok, clr = q0.copy(), [], True, np.inf
        for i in order:
            rid = ("robot1", "robot2")[i]
            vias = pl.transit(rid, q[i], pl.cell.robots[rid].home)
            if vias is None:
                ok = False
                break
            for w in pl.dense_transit(rid, vias):
                q = q.copy(); q[i] = w
                qs.append(q)
        prev = q0
        for qq in (qs if ok else []):
            for f in (0.5, 1.0):
                c = env.check_collisions(prev + f * (qq - prev))
                clr = min(clr, c["min_clearance"])
                ok &= not c["collision"]
            prev = qq
            if not ok:
                break
        if ok and qs:
            return dict(order=f"planned{order}", q=np.array(qs).reshape(-1, 2, 6), min_clearance=float(clr), ok=True)
    return dict(order=None, q=np.zeros((0, 2, 6)), min_clearance=None, ok=False)


def exec_clock(t, q, ev, arrivals, hold_s=0.5):
    """Executor clock of the rollout exported like --export (cuts at every weld completion)."""
    from player.bt_plan import legs_from_rollout
    cuts = sorted({e[3] for e in ev})
    legs = legs_from_rollout(t, q, cuts, weld_arrivals=arrivals, hold_s=hold_s)
    welds = [l for l in legs if l.kind == "weld"]
    return dict(exec_last_weld_s=float(max(l.t1 for l in welds) + hold_s) if welds else None,
                exec_motion_end_s=float(max(l.t1 for l in legs)), exec_end_s=float(max(l.t1 for l in legs) + 1.0),
                n_cuts=len(cuts)), legs


def finish(res, env, ex, t, q, ev, arrivals):
    """Return HOME + executor clock + dense clearance (for any controller's rollout)."""
    dt = 0.05
    t, q = np.asarray(t, float), np.asarray(q, float).reshape(-1, 2, 6)
    if res["success"]:
        tail = home_tail(env, ex, q[-1].copy())
        res["home_tail"] = dict(order=tail["order"], ok=tail["ok"], min_clearance=tail["min_clearance"],
                                steps=int(len(tail["q"])))
        if not len(tail["q"]):      # no collision-free return HOME found: hold 1 s (reported, arms stay put)
            tail["q"] = np.repeat(q[-1:], 20, 0)
        t = np.concatenate([t, t[-1] + dt * np.arange(1, len(tail["q"]) + 1)])
        q = np.concatenate([q, tail["q"]])
        res["cycle_time_with_home_s"] = float(t[-1])
        ec, legs = exec_clock(t, q, ev, arrivals)
        res.update(ec)
    else:
        legs = None
    res["dense_min_clearance"] = dense_min_clearance(env.spheres, q)
    return res, t, q, legs


def run_episode(henv, policy, seed, canonical):
    t0 = time.time()
    fresh = not _G["o"]["select"]
    obs = henv.reset(seed=seed, options={"canonical": True, "fresh": fresh} if canonical
                     else {"no_canonical": True, "fresh": fresh})
    t_ik = time.time() - t0
    acts = []
    while True:
        a = policy(obs)
        acts.append(int(a))
        obs, r, done, info = henv.step(a)
        if done:
            break
    col = info["collision"]
    hist = [(round(t * henv.dt, 3), int(i), int(a)) for t, i, a in henv.history]
    res = dict(seed=seed, canonical=canonical, wids=henv.wids, infeasible=henv.infeasible, n_welds=info["n_welds"],
               welds_done=info["welds_done"], success=info["success"],
               collision=None if col is None else col["kind"], collision_t=None if col is None else col["t"],
               timeout=info["timeout"], makespan=info["makespan"], t_end=info["t"],
               min_clearance=info["min_clearance"], decisions=len(acts), weld_log=[list(x) for x in henv.weld_log],
               allocation={rid: [e[1] for e in henv.weld_log if e[0] == rid] for rid in ("robot1", "robot2")},
               decisions_log=hist, wall_s=time.time() - t0, ik_s=t_ik)
    ev = [tuple(e) for e in henv.weld_log]
    arr = {(r, w): tt for r, w, tt in henv.arrivals}
    # weld legs end ON the weld (actual arrival), but never after the env completed the weld (the pose check can
    # complete it during a slow descent, before the arm reaches the weld configuration)
    arrivals = [(r, w, min(arr.get((r, w), a), b)) for r, w, a, b in ev]
    res, t, q, legs = finish(res, henv.env, henv.ex, henv.rec_t, henv.rec_q, ev, arrivals)
    return res, (t, q, ev, arrivals, legs)


def expert_episode(henv, seed, canonical, max_s=None):
    """Track X's ScriptedExpert (fixed 'expert' split, reservation coordination) on exactly the same
    program as ``run_episode`` (same weld subset, noisy positions, start pose), same env checks, executor limits
    matched to --speed-profile (transit v/a, descent v)."""
    from env.weld_env import EnvConfig, KukaWeldEnv
    from expert.scripted import ScriptedExpert
    from rl.hier.instances import table_path
    from scripts.run_expert import EXPERT_QUEUES
    t0 = time.time()
    rng = np.random.default_rng(seed)    # same draws as HierWeldEnv.reset(seed, {"no_canonical": True, "fresh": True})
    prog = henv.sample_program(rng, canonical=canonical, fresh=True)
    if "xenv" not in _G:
        P = henv.ex.profile
        e = KukaWeldEnv(EnvConfig(cell_id=henv.cfg.cell_id, episode_len=10 ** 7, assignment=EXPERT_QUEUES))
        _G["xenv"] = e
        _G["xpert"] = ScriptedExpert(e, table_path=table_path(henv.cfg.cell_id),
                                     transit_frac=P.transit_v / e.vscale, acc_frac=P.transit_a,
                                     approach_frac=P.desc_v / e.vscale)
    e, ex = _G["xenv"], _G["xpert"]
    obs, _ = e.reset(options={"canonical": True, "welds": prog["wids"], "q0": prog["q0"]})
    n = len(prog["wids"])
    e._task_pos[:n] = prog["pos"]
    obs = e._obs()
    ex.reset()
    max_s = max_s or (henv.cfg.time_base + henv.cfg.time_per_weld * n)
    clr, col, info = np.inf, None, {"success": n == 0, "welds_done": 0}
    ts, qs = [0.0], [e.q.copy()]
    while e._t * e.dt < max_s and not info.get("success"):
        obs, r, term, trunc, info = e.step(ex.act(obs))
        ts.append(e._t * e.dt); qs.append(e.q.copy())
        clr = min(clr, info.get("min_clearance_substeps", info["min_clearance"]))
        if info["collision"]:
            col = info["collision_kind"]
            break
    ok = bool(info.get("success")) and col is None
    ev = [(r, w, a * e.dt, b * e.dt) for r, w, a, b in e.weld_events]
    res = dict(seed=seed, canonical=canonical, wids=prog["wids"], infeasible=[], n_welds=n,
               welds_done=int(info["welds_done"]), success=ok, collision=col, collision_t=None,
               timeout=not ok and col is None, makespan=(max(x[3] for x in ev)) if ok else None,
               t_end=e._t * e.dt, min_clearance=float(clr), decisions=0, weld_log=[list(x) for x in ev],
               allocation={rid: [x[1] for x in ev if x[0] == rid] for rid in ("robot1", "robot2")},
               decisions_log=[], wall_s=time.time() - t0, ik_s=0.0, plan_fail=list(ex.stats["plan_fail"]))
    arrivals = [(r, w, a) for r, w, a, b in ev]
    res, t, q, legs = finish(res, e, henv.ex, ts, qs, ev, arrivals)
    return res, (t, q, ev, arrivals, legs)


def _job(seed):
    if _G["o"]["spec"] == "expert":
        return expert_episode(_G["env"], seed, False)[0]
    return run_episode(_G["env"], _G["policy"], seed, False)[0]


def export_canonical(henv, res, traj, args):
    from player.bt_plan import bt_plan_from_legs
    from player.timeline import timeline_from_arrays
    t, q, ev, arrivals, legs = traj
    meta = dict(controller="hierarchical GNN scheduler (Track H)" if args.policy is None else args.policy,
                checkpoint=args.checkpoint, source="evaluate_hier.py", speed_profile=args.speed_profile,
                cell=args.cell, cycle_time_s=res["makespan"], cycle_time_with_home_s=res.get("cycle_time_with_home_s"),
                allocation=res["allocation"], max_tilt=args.max_tilt, min_self_dist=args.min_self_dist)
    tl = timeline_from_arrays(t, q, weld_events=ev, meta=meta)
    out = Path(args.timeline)
    tl.save(out)
    res["timeline"] = str(out)
    res["arrivals"] = [list(x) for x in arrivals]
    if res["success"] and legs is not None:
        pp = ROOT / "results" / f"{args.tag}_plan_canonical.json"
        bt_plan_from_legs(henv.cell.cell_id, legs, pp, metadata_extra={"source": "kuka_weld_rl Track H hierarchical"})
        res["umarell_plan"] = str(pp)
    return res


def summarize(rows, seeds, wall):
    n = len(rows)
    feas = [r for r in rows if not r["infeasible"]]
    succ = [r for r in rows if r["success"]]
    m = lambda k, rr: float(np.mean([r[k] for r in rr if r.get(k) is not None])) if any(r.get(k) is not None for r in rr) else None
    f15 = [r for r in succ if r["n_welds"] == 15]
    return dict(
        n=n, seeds=[seeds[0], seeds[-1]], wall_s=wall,
        success_rate=sum(r["success"] for r in rows) / n,
        success_rate_feasible=sum(r["success"] for r in feas) / max(1, len(feas)),
        n_infeasible=n - len(feas),
        collision_rate=sum(r["collision"] is not None for r in rows) / n,
        collision_kinds={k: sum(r["collision"] == k for r in rows) for k in ("arm_arm", "self", "floor", "fixture")},
        timeout_rate=sum(r["timeout"] for r in rows) / n,
        weld_completion=sum(r["welds_done"] for r in rows) / max(1, sum(r["n_welds"] for r in rows)),
        mean_makespan_s_success=m("makespan", succ),
        mean_exec_last_weld_s_success=m("exec_last_weld_s", succ), mean_exec_end_s_success=m("exec_end_s", succ),
        full15_success=sum(r["success"] for r in rows if r["n_welds"] == 15), full15_n=sum(r["n_welds"] == 15 for r in rows),
        full15_mean_makespan_s=m("makespan", f15), full15_mean_exec_last_weld_s=m("exec_last_weld_s", f15),
        full15_mean_exec_end_s=m("exec_end_s", f15),
        min_clearance=float(min(r["min_clearance"] for r in rows)),
        dense_min_clearance=float(min(r["dense_min_clearance"] for r in rows)),
        rows=rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", default=None)
    ap.add_argument("--policy", default=None, help="heuristic | expert (Track X ScriptedExpert on the same programs)")
    ap.add_argument("--tag", required=True)
    ap.add_argument("--episodes", type=int, default=100)
    ap.add_argument("--seed0", type=int, default=None, help="default 50000 (held-out), 60000 (--ood), 900001 (--select)")
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--export", action="store_true")
    ap.add_argument("--select", action="store_true", help="model selection on the eval pool, not held-out")
    ap.add_argument("--ood", action="store_true", help="out-of-distribution set: +-5 cm, 8-12 welds, start noise 0.25 rad")
    ap.add_argument("--legacy-v2", action="store_true", help="v2 runs (hier_e/f): elapsed-time feature")
    ap.add_argument("--safe-thr", type=float, default=0.3, help="learned safety-critic threshold (<=0: plain argmax Q)")
    ap.add_argument("--no-lookahead", action="store_true", help="ablation: zero the lookahead features for the net")
    ap.add_argument("--speed-profile", default="ours")
    ap.add_argument("--max-tilt", type=float, default=None)
    ap.add_argument("--min-self-dist", type=float, default=None)
    ap.add_argument("--cell", default="welding_cell_2_small")
    ap.add_argument("--timeline", default=None, help="canonical timeline path (with --export)")
    ap.add_argument("--robustness", action="store_true", help="canonical: weld margins under tracking noise")
    args = ap.parse_args()
    spec = args.policy or args.checkpoint
    if args.seed0 is None:
        args.seed0 = 900001 if args.select else (60000 if args.ood else 50000)
    if args.timeline is None:
        args.timeline = str(ROOT / "results" / f"{args.tag}_canonical_timeline.npz")
    o = dict(spec=spec, safe_thr=args.safe_thr, legacy_v2=args.legacy_v2, select=args.select, ood=args.ood,
             no_lookahead=args.no_lookahead, speed_profile=args.speed_profile, max_tilt=args.max_tilt,
             min_self_dist=args.min_self_dist, cell=args.cell)
    out = dict(policy=spec, options=o)
    p = ROOT / "results" / f"{args.tag}_eval.json"
    p.parent.mkdir(exist_ok=True)
    # held-out first, in a spawn pool created before any torch / OpenMP work in this process (fork hang, m11)
    if args.episodes:
        seeds = [args.seed0 + k for k in range(args.episodes)]
        t0 = time.time()
        with mp.get_context("spawn").Pool(args.workers, initializer=_init, initargs=(o, False)) as pool:
            rows = pool.map(_job, seeds, chunksize=1)
        h = out["held_out"] = summarize(rows, seeds, time.time() - t0)
        print(f"[{'ood' if args.ood else 'held-out'}] n {h['n']} success {h['success_rate']:.3f} "
              f"({h['n_infeasible']} infeasible) collision {h['collision_rate']:.3f} {h['collision_kinds']} "
              f"timeout {h['timeout_rate']:.3f} welds {h['weld_completion']:.3f} makespan {h['mean_makespan_s_success']} "
              f"exec {h['mean_exec_last_weld_s_success']} full15 {h['full15_success']}/{h['full15_n']} "
              f"{h['full15_mean_makespan_s']} min_clr {h['min_clearance']:.3f} dense {h['dense_min_clearance']:.3f}",
              flush=True)
    _init(o, record=True)
    henv, pol = _G["env"], _G["policy"]
    res, traj = expert_episode(henv, 0, True) if spec == "expert" else run_episode(henv, pol, 0, True)
    print(f"[canonical] welds {res['welds_done']}/{res['n_welds']} collision {res['collision']} makespan "
          f"{res['makespan']} exec_last_weld {res.get('exec_last_weld_s')} exec_end {res.get('exec_end_s')} "
          f"min_clr {res['min_clearance']:.3f} dense {res['dense_min_clearance']:.3f} alloc {res['allocation']}", flush=True)
    if args.robustness:
        from rl.hier.robustness import rollout_self_distance, summary, weld_margins
        t, q, ev, arrivals, legs = traj
        k = lambda tt: int(min(len(q) - 1, round(tt / 0.05)))
        arr = [(r, w, q[k(b), ("robot1", "robot2").index(r)]) for r, w, a, b in ev]   # on the weld at completion
        pos = {w.wid: w.pos for w in henv.cell.welds}
        res["margins"] = weld_margins(args.cell, arr, pos)
        res["margins_summary"] = summary(res["margins"])
        res["rollout_self_distance"] = rollout_self_distance(args.cell, q)
        print("[margins]", json.dumps(res["margins_summary"]), "self", res["rollout_self_distance"], flush=True)
    if args.export:
        res = export_canonical(henv, res, traj, args)
        print(f"timeline -> {res['timeline']}  plan -> {res.get('umarell_plan')}", flush=True)
    out["canonical"] = res
    json.dump(out, open(p, "w"), indent=1, default=lambda x: x.tolist() if hasattr(x, "tolist") else str(x))
    print("->", p)


if __name__ == "__main__":
    main()
