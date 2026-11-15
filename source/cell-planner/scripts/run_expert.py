"""Run the scripted expert in KukaWeldEnv: canonical task + randomised variants.

    .venv/bin/python scripts/run_expert.py [--assignment expert|greedy|umarell] [--n-random 20] [--workers 4]

Reports welds done, collisions, cycle time (time until the last weld completes = env episode end),
min arm-arm clearance. Saves demonstrations (transitions in the env's obs format) to
data/demos/expert_<tag>.npz with keys ``obs/<k>``, ``next_obs/<k>``, ``action``, ``reward``,
``terminated``, ``truncated``, ``done``, ``episode`` (episode index per transition), plus a joint
timeline per canonical run (``player.timeline.Timeline`` npz if available, else t/q npz) and a JSON
summary in data/demos/expert_summary_<tag>.json.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from env.cell import ROBOT_IDS  # noqa: E402

# Feasible split for the hull-self-collision-aware IK table (data/weld_ik.json): robot1 cannot
# reach W10/W13/W14 and self-collides on W01/W04/W07; robot2 cannot reach W02/W03.
# Order: both arms start at the far ends of the door and robot2 finishes the x=-1.15 column
# (W07, W04, W01) after robot1 is done with the -y rows.
EXPERT_QUEUES = {
    "robot1": ["W03", "W02", "W06", "W05", "W09", "W08"],
    "robot2": ["W15", "W13", "W14", "W10", "W12", "W11", "W07", "W04", "W01"],
}


def make_env(cell, assignment, **kw):
    from env.weld_env import EnvConfig, KukaWeldEnv
    a = EXPERT_QUEUES if assignment == "expert" else assignment
    return KukaWeldEnv(EnvConfig(cell_id=cell, assignment=a, **kw))


def rollout(env, expert, options, max_steps=None, record=True, home_tail=True):
    obs, info = env.reset(seed=options.pop("_seed", None), options=options)
    expert.reset()
    O, A, R, NO, TE, TR, CO = [], [], [], [], [], [], []
    ts, qs = [0.0], [obs["robot_q"].astype(np.float64).copy()]
    min_clr = float(info["min_clearance"])
    kinds = []
    max_steps = max_steps or env.cfg.episode_len
    t0 = time.time()
    for k in range(max_steps):
        a = expert.act(obs)
        nobs, r, term, trunc, info = env.step(a)
        if record:
            O.append(obs); A.append(a); R.append(r); NO.append(nobs); TE.append(term); TR.append(trunc)
            CO.append(bool(info["collision"]))
        ts.append(info["time_s"]); qs.append(nobs["robot_q"].astype(np.float64).copy())
        min_clr = min(min_clr, float(info["min_clearance"]))
        if info["collision"]:
            kinds.append((info["t"], info["collision_kind"]))
        obs = nobs
        if term or trunc:
            break
    # Return both arms HOME after the last weld (the env episode ends at the last weld; umarell's
    # makespan includes the final MoveHome). Same coordination code, kinematic integration here,
    # every configuration checked with env.check_collisions.
    home_steps, tail_coll = 0, []
    if info["success"] and home_tail:
        o = {k: np.array(v) for k, v in obs.items()}
        o["robot_goal_mask"] = np.zeros_like(o["robot_goal_mask"])
        qcur = o["robot_q"].astype(np.float64)
        for k in range(1200):
            a = expert.act(o).astype(np.float64).reshape(2, 6)
            qcur = np.clip(qcur + np.clip(a, -1, 1) * env.v_max * env.dt, env.q_lo, env.q_hi)
            o["robot_q"] = qcur.astype(np.float32)
            qcur = o["robot_q"].astype(np.float64)
            c = env.check_collisions(qcur)
            min_clr = min(min_clr, c["min_clearance"])
            if c["collision"]:
                tail_coll.append((k, c["kind"]))
            ts.append(ts[-1] + env.dt); qs.append(qcur.copy())
            home_steps += 1
            if expert.idle():
                break
    res = dict(home_tail_s=home_steps * env.dt, home_tail_collisions=tail_coll,
               cycle_time_with_home_s=float(info["time_s"]) + home_steps * env.dt,
               at_home=bool(np.allclose(qs[-1], env.home, atol=1e-6)),
               welds_done=int(info["welds_done"]), n_welds=int(info["n_welds"]), success=bool(info["success"]),
               collision=bool(kinds), collision_kinds=kinds, steps=int(info["t"]), cycle_time_s=float(info["time_s"]),
               min_clearance=min_clr, weld_events=[list(e) for e in env.weld_events],
               queues={rid: [env.task_ids[k] for k in env.queues[i]] for i, rid in enumerate(ROBOT_IDS)},
               expert_stats={k: (v if not isinstance(v, float) else float(v)) for k, v in expert.stats.items()},
               wall_s=time.time() - t0)
    res["legs"] = expert_legs(env, expert)
    traj = dict(obs=O, action=A, reward=R, next_obs=NO, terminated=TE, truncated=TR, collision=CO,
                t=np.array(ts), q=np.array(qs))
    return res, traj


def expert_legs(env, expert):
    """Committed expert plans as player.timeline.Leg-compatible dicts (absolute times)."""
    out = []
    ph_map = {"retract": "transit", "transit": "transit", "home": "transit", "descent": "approach"}
    for L in expert.legs:
        p = L["plan"]
        n = len(p.wps)
        if n == 0:
            continue
        t0 = L["step"] * env.dt
        times = t0 + np.arange(n + 1) * env.dt
        pos = np.vstack([L["q0"][None], p.wps])
        phase = ["transit"] + [ph_map.get(x, "transit") for x in p.phase]
        weld = p.weld_q is not None
        op = env.task_ids[p.weld_idx] if (weld and p.weld_idx is not None) else "HOME"
        out.append(dict(robot=L["robot"], op_id=op, kind="weld" if weld else "home", t0=float(t0),
                        t1=float(times[-1]), times=times.tolist(), positions=pos.tolist(), phase=phase,
                        t_done=float(times[-1]) + (0.5 if weld else 0.0), hold_s=0.5 if weld else 0.0,
                        operation_type="weld" if weld else "", node_type="ExecuteTrajectory" if weld else "MoveHome"))
    return out


def stack_demos(trajs):
    out = {}
    eps = []
    for e, tr in enumerate(trajs):
        if not tr["action"]:
            continue
        eps.append(np.full(len(tr["action"]), e, np.int32))
        for key in ("obs", "next_obs"):
            for k in tr[key][0]:
                out.setdefault(f"{key}/{k}", []).append(np.stack([o[k] for o in tr[key]]))
        out.setdefault("action", []).append(np.array(tr["action"], np.float32))
        out.setdefault("reward", []).append(np.array(tr["reward"], np.float32))
        te, trn = np.array(tr["terminated"]), np.array(tr["truncated"])
        out.setdefault("terminated", []).append(te)
        out.setdefault("truncated", []).append(trn)
        out.setdefault("done", []).append(te | trn)
    d = {k: np.concatenate(v) for k, v in out.items()}
    d["episode"] = np.concatenate(eps) if eps else np.zeros(0, np.int32)
    return d


def save_demos_rl(path, trajs, meta):
    """Track R's demo format (rl/demos.py save_demos: obs.<key> rows incl. final obs, action (L,2,6),
    reward, terminated, collision, ep_len, meta)."""
    from rl.demos import save_demos
    from rl.her_buffer import EpisodeRecorder
    eps = []
    for tr in trajs:
        if not tr["action"]:
            continue
        rec = EpisodeRecorder()
        rec.reset(tr["obs"][0])
        for a, r, te, co, no in zip(tr["action"], tr["reward"], tr["terminated"], tr["collision"], tr["next_obs"]):
            rec.add(np.asarray(a).reshape(2, -1), r, te, co, no)
        eps.append(rec.flush())
    save_demos(path, eps, meta)
    return len(eps), int(sum(len(e["reward"]) for e in eps))


def save_timeline(path, traj, res, label):
    path = Path(path)
    q = traj["q"].reshape(-1, 2, 6)
    ev = [(r, w, a * 0.05, b * 0.05) for r, w, a, b in res["weld_events"]]
    meta = {"controller": label, "source": "scripts/run_expert.py", "cycle_time_s": res["cycle_time_s"],
            "cycle_time_with_home_s": res["cycle_time_with_home_s"]}
    try:
        from player.timeline import Leg, Timeline
        legs = [Leg.from_json(dict(d)) for d in res["legs"]]
        tl = Timeline(t=traj["t"], q=q, legs=legs, weld_events=ev, meta=meta, home=q[0].copy())
        tl.save(path)
    except Exception as e:  # pragma: no cover - fallback if Track P changes its API
        np.savez(path, t=traj["t"], q=q, note=f"Timeline save failed: {e}")
    return path


def export_plan(path, traj, res, cell, dt=0.05):
    """Canonical rollout -> umarell BT plan JSON (player.bt_plan.legs_from_rollout: both arms cut at
    every weld completion; the executor's hold + completion margin become pauses)."""
    from player.bt_plan import bt_plan_from_legs, legs_from_rollout
    t, q = traj["t"], traj["q"].reshape(-1, 2, 6)
    cuts = sorted({e[3] * dt for e in res["weld_events"]})
    arrivals = [(r, w, a * dt) for r, w, a, b in res["weld_events"]]
    legs = legs_from_rollout(t, q, cuts, weld_arrivals=arrivals, hold_s=0.5)
    plan = bt_plan_from_legs(cell, legs, path, metadata_extra={"source": "kuka_weld_rl scripted expert"})
    return plan


def _random_job(args):
    cell, assignment, seed, kw = args
    from expert.scripted import ScriptedExpert
    env = make_env(cell, assignment, **kw)
    ex = ScriptedExpert(env)
    res, traj = rollout(env, ex, {"_seed": seed})
    res["seed"] = seed
    res.pop("legs", None)
    return res, traj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cell", default="welding_cell_2_small")
    ap.add_argument("--assignment", default="expert", help="expert | greedy | umarell")
    ap.add_argument("--n-random", type=int, default=20)
    ap.add_argument("--seed0", type=int, default=1000)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--tag", default=None)
    ap.add_argument("--no-demos", action="store_true")
    a = ap.parse_args()
    from expert.scripted import ScriptedExpert
    tag = a.tag or f"{a.assignment}_{a.cell.replace('welding_cell_', '')}"
    out = ROOT / "data" / "demos"
    out.mkdir(parents=True, exist_ok=True)

    env = make_env(a.cell, a.assignment)
    ex = ScriptedExpert(env)
    res, traj = rollout(env, ex, {"canonical": True})
    print(f"[canonical] welds {res['welds_done']}/{res['n_welds']} collision {res['collision_kinds']} "
          f"cycle {res['cycle_time_s']:.2f}s (+home {res['cycle_time_with_home_s']:.2f}s, tail coll "
          f"{res['home_tail_collisions']}) min_clr {res['min_clearance']:.3f} stats {res['expert_stats']}")
    tl = save_timeline(out / f"expert_timeline_canonical_{tag}.npz", traj, res, f"scripted expert ({a.assignment})")
    summary = {"cell": a.cell, "assignment": a.assignment, "canonical": res, "timeline": str(tl.relative_to(ROOT))}
    if res["success"]:
        try:
            pp = out / f"expert_plan_canonical_{tag}.json"
            export_plan(pp, traj, res, a.cell)
            summary["umarell_plan"] = str(pp.relative_to(ROOT))
            print(f"umarell plan -> {pp}")
        except Exception as e:  # pragma: no cover
            print("plan export failed:", e)
    trajs = [traj]

    if a.n_random:
        jobs = [(a.cell, a.assignment, a.seed0 + i, {}) for i in range(a.n_random)]
        with Pool(a.workers) as pool:
            rr = pool.map(_random_job, jobs, chunksize=1)
        rows = [r for r, _ in rr]
        # demos: successful, collision-free episodes only
        trajs += [t for r, t in rr if r["success"] and not r["collision"]]
        for r in rows:
            print(f"[seed {r['seed']}] welds {r['welds_done']}/{r['n_welds']} coll {r['collision_kinds'][:1]} "
                  f"t {r['cycle_time_s']:.1f}s clr {r['min_clearance']:.3f} fails {len(r['expert_stats']['plan_fail'])}")
        n = len(rows)
        summary["random"] = {
            "n": n, "seeds": [r["seed"] for r in rows],
            "success_rate": sum(r["success"] for r in rows) / n,
            "collision_rate": sum(r["collision"] for r in rows) / n,
            "weld_completion": sum(r["welds_done"] for r in rows) / max(1, sum(r["n_welds"] for r in rows)),
            "mean_cycle_time_s_success": float(np.mean([r["cycle_time_s"] for r in rows if r["success"]] or [np.nan])),
            "min_clearance": float(min(r["min_clearance"] for r in rows)),
            # episodes where some weld had no valid IK at all (the env's +-3 cm noise can push a weld
            # out of the assigned arm's reach); success rate over the remaining ones:
            "n_plan_fail": sum(bool(r["expert_stats"]["plan_fail"]) for r in rows),
            "success_rate_feasible": (sum(r["success"] for r in rows if not r["expert_stats"]["plan_fail"])
                                      / max(1, sum(not r["expert_stats"]["plan_fail"] for r in rows))),
            "home_tail_collisions": sum(bool(r["home_tail_collisions"]) for r in rows),
            "rows": rows,
        }
        print({k: v for k, v in summary["random"].items() if k not in ("rows", "seeds")})
    if not a.no_demos:
        d = stack_demos(trajs)
        p = out / f"expert_{tag}_transitions.npz"
        np.savez_compressed(p, **d)
        p2 = out / f"expert_{tag}.npz"
        n_ep, n_tr = save_demos_rl(p2, trajs, {"source": "scripts/run_expert.py ScriptedExpert", "cell": a.cell,
                                               "assignment": a.assignment, "episode_0": "canonical",
                                               "seeds": [a.seed0 + i for i in range(a.n_random)],
                                               "filter": "successful collision-free episodes only"})
        summary["demos"] = {"path_rl_format": str(p2.relative_to(ROOT)), "path_transitions": str(p.relative_to(ROOT)),
                            "n_transitions": int(len(d["action"])), "n_episodes": n_ep, "episode_0": "canonical",
                            "note": "successful collision-free episodes only (canonical + randomised)"}
        print(f"demos -> {p2} (rl/demos format) + {p} ({len(d['action'])} transitions)")
    json.dump(summary, open(out / f"expert_summary_{tag}.json", "w"), indent=1, default=lambda o: o.tolist()
              if hasattr(o, "tolist") else str(o))


if __name__ == "__main__":
    main()
