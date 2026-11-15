"""Evaluate a trained policy (or a baseline) on KukaWeldEnv.

    python evaluate.py --checkpoint runs/x/ckpt_latest.pt --stage 3 --episodes 100 --canonical
    python evaluate.py --policy expert --stage 0 --episodes 50        # DLS reach expert (stages 0/1)
    python evaluate.py --checkpoint runs/x/final.pt --stage 0 --out results/x_stage0.json

Held-out variants use seeds `--seed + i` (default 50_000+): the seed drives both the curriculum
sample (weld subset, queues, start pose) and env.reset. Training draws episode seeds from a 2^31
range per worker RNG and the in-training eval uses 10_000+i, so these are disjoint in practice.
Results: success (all welds of the episode done, no collision), weld completion fraction,
collision rate overall and per kind (arm_arm / self / floor / fixture), cycle time (s) of
successful episodes, mean return, min arm-arm clearance. JSON goes to results/ (or --out).
"""
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np

KINDS = ("arm_arm", "self", "floor", "fixture")


def make_policy(agent, amap=None):
    """Deterministic policy: list of env obs -> (n, 12) env actions (through the action map)."""
    from rl.action_map import ActionMap
    amap = amap or ActionMap()

    def fn(obs_list):
        from rl.her_buffer import clean_obs
        b = {k: np.stack([clean_obs(o)[k] for o in obs_list]) for k in clean_obs(obs_list[0])}
        u = amap.mask_u(agent.act(b, noise=0.0), b)
        return amap.to_env(u, b).reshape(len(obs_list), -1)
    return fn


def run_episodes(policy_fn, env_kwargs: dict, spec, seeds, n_parallel: int = 16, canonical: bool = False,
                 envs: list | None = None, per_env_reset=None, record_q: bool = False) -> list[dict]:
    """Deterministic lockstep rollouts. policy_fn(list of obs) -> (n, 12). Returns per-episode dicts."""
    from rl.curriculum import reset_env
    from rl.vec_env import make_env
    seeds = list(seeds)
    n_par = max(1, min(n_parallel, len(seeds)))
    envs = envs or [make_env(env_kwargs) for _ in range(n_par)]
    results = []
    todo = list(seeds)
    slots = [None] * len(envs)

    def start(i):
        if not todo:
            slots[i] = None
            return
        s = todo.pop(0)
        env = envs[i]
        if canonical:
            env.cfg.episode_len = getattr(spec, "episode_len", None) or 2000
            if getattr(spec, "assignment", None):   # same split as the curriculum's stage-3 canonical episodes
                env.cfg.assignment = spec.assignment
            obs, info = env.reset(seed=s, options={"canonical": True})
        else:
            obs, info = reset_env(env, spec, np.random.default_rng(s), seed=s)
        if per_env_reset is not None:
            per_env_reset(i, env, obs)
        slots[i] = dict(seed=s, obs=obs, ret=0.0, len=0, min_clear=float(info["min_clearance"]),
                        q=[env.q.copy()] if record_q else None)

    for i in range(len(envs)):
        start(i)
    while any(s is not None for s in slots):
        live = [i for i, s in enumerate(slots) if s is not None]
        acts = policy_fn([slots[i]["obs"] for i in live])
        for i, a in zip(live, acts):
            st = slots[i]
            obs, r, term, trunc, info = envs[i].step(a)
            st["obs"], st["ret"], st["len"] = obs, st["ret"] + r, st["len"] + 1
            st["min_clear"] = min(st["min_clear"], float(info["min_clearance"]))
            if record_q:
                st["q"].append(envs[i].q.copy())
            if term or trunc:
                res = dict(seed=st["seed"], success=bool(info["success"]) and not info["collision"],
                           welds_done=int(info["welds_done"]), n_welds=int(info["n_welds"]),
                           weld_frac=info["welds_done"] / max(1, info["n_welds"]),
                           collision=bool(info["collision"]), collision_kind=info["collision_kind"],
                           len=st["len"], time_s=st["len"] * envs[i].dt, ret=st["ret"],
                           min_clearance=st["min_clear"], truncated=bool(trunc))
                if record_q:
                    res["q"] = np.array(st["q"])
                    dt = envs[i].dt   # weld events in seconds (env steps -> s), player.timeline format
                    res["weld_events"] = [(r_, w_, float(a_) * dt, float(b_) * dt) for r_, w_, a_, b_ in
                                          getattr(envs[i], "weld_events", [])]
                results.append(res)
                start(i)
    return results


def summarize(res: list[dict]) -> dict:
    if not res:
        return {}
    m = lambda k: float(np.mean([r[k] for r in res]))
    out = dict(episodes=len(res), success=m("success"), weld_frac=m("weld_frac"), welds_done=m("welds_done"),
               collision=m("collision"), ret=m("ret"), len=m("len"), min_clearance=float(np.min([r["min_clearance"] for r in res])))
    for k in KINDS:
        out[f"coll_{k}"] = float(np.mean([r["collision_kind"] == k for r in res]))
    succ = [r["time_s"] for r in res if r["success"]]
    out["cycle_time_s"] = float(np.mean(succ)) if succ else None
    return out


def make_expert(env):
    """Track X's ScriptedExpert if available, else the single-arm DLSReachExpert (stages 0/1)."""
    try:
        from expert.scripted import ScriptedExpert
        return ScriptedExpert(env)
    except ImportError:
        from rl.demos import DLSReachExpert
        return DLSReachExpert(env)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--checkpoint")
    p.add_argument("--policy", default="agent", choices=["agent", "expert", "zero", "random"])
    p.add_argument("--stage", type=int, default=3)
    p.add_argument("--episodes", type=int, default=100)
    p.add_argument("--seed", type=int, default=50_000)
    p.add_argument("--canonical", action="store_true", help="also run the canonical 15-weld task from HOME")
    p.add_argument("--canonical-only", action="store_true")
    p.add_argument("--n-parallel", type=int, default=16)
    p.add_argument("--assignment", default=None)
    p.add_argument("--episode-len", type=int, default=None)
    p.add_argument("--device", default="cpu")
    p.add_argument("--save-q", default=None, help="npz path for the canonical joint trajectory")
    p.add_argument("--save-timeline", default=None,
                   help="player.timeline Timeline .npz of the canonical rollout (q + weld_events), e.g. "
                        "data/timelines/policy_e2e_canonical.npz")
    p.add_argument("--out", default=None)
    a = p.parse_args()

    from rl.curriculum import StageSpec
    ck, env_kwargs, spec_kw = {}, {}, {}
    if a.policy == "agent":
        from rl.td3 import load_agent
        if not a.checkpoint:
            raise SystemExit("--checkpoint is required for --policy agent")
        agent, ck = load_agent(a.checkpoint, a.device)
        env_kwargs = dict(ck.get("env_kwargs", {}))
        spec_kw = dict(ck.get("stage_spec", {}))
        from rl.action_map import ActionMap
        fn = make_policy(agent, ActionMap(ck.get("action_map"), env_kwargs.get("cell_id", "welding_cell_2_small")))
    spec_kw.update(stage=a.stage, mix=0.0)
    if a.assignment:
        spec_kw["assignment"] = a.assignment
    if a.episode_len:
        spec_kw["episode_len"] = a.episode_len
    spec = StageSpec(**{k: v for k, v in spec_kw.items() if k in StageSpec.__dataclass_fields__})
    from rl.vec_env import make_env
    envs = [make_env(env_kwargs) for _ in range(min(a.n_parallel, max(1, a.episodes)))]
    per_reset = None
    if a.policy == "expert":
        ex = make_expert(envs[0])
        fn = lambda obs_list: np.stack([ex.act(o) for o in obs_list])
        envs = envs[:1]   # stateful expert: one env at a time

        def per_reset(i, env, obs):
            if hasattr(ex, "reset"):
                ex.reset()
    elif a.policy == "zero":
        fn = lambda obs_list: np.zeros((len(obs_list), 12))
    elif a.policy == "random":
        rng = np.random.default_rng(0)
        fn = lambda obs_list: rng.uniform(-1, 1, (len(obs_list), 12))

    res_all = {}
    t0 = time.time()
    if not a.canonical_only:
        res_all["variants"] = run_episodes(fn, env_kwargs, spec, range(a.seed, a.seed + a.episodes),
                                           a.n_parallel, envs=envs, per_env_reset=per_reset)
    if a.canonical or a.canonical_only:
        res_all["canonical"] = run_episodes(fn, env_kwargs, spec, [a.seed], canonical=True, envs=envs[:1],
                                            per_env_reset=per_reset, record_q=bool(a.save_q or a.save_timeline))
        c0 = res_all["canonical"][0]
        if a.save_timeline:
            from player.timeline import timeline_from_arrays
            q = c0["q"]
            t = np.arange(len(q)) * envs[0].dt
            meta = {k: c0[k] for k in ("success", "welds_done", "n_welds", "collision", "collision_kind",
                                        "time_s", "min_clearance")}
            meta.update(controller="e2e" if a.policy == "agent" else a.policy, checkpoint=a.checkpoint,
                        makespan_motion_s=float(t[-1]), steps=len(q))
            tl = timeline_from_arrays(t, q, weld_events=c0.get("weld_events", []), meta=meta, op_prefix="E2E")
            tl.save(a.save_timeline)
            print("wrote", a.save_timeline, f"({len(c0.get('weld_events', []))} weld events)")
        if a.save_q:
            np.savez(a.save_q, q=c0["q"], dt=envs[0].dt)
        c0.pop("q", None)
    print(f"eval wall time {time.time() - t0:.1f}s")
    out = {"checkpoint": a.checkpoint, "policy": a.policy, "stage": a.stage, "seed0": a.seed,
           "spec": spec.to_dict(), "env_kwargs": {k: v for k, v in env_kwargs.items() if k != "reward"}}
    for name, res in res_all.items():
        for r in res:
            r.pop("q", None)
        out[name] = summarize(res)
        out[name + "_episodes"] = res
        print(name, json.dumps(out[name], indent=None))
    path = a.out or os.path.join("results", f"eval_{Path(a.checkpoint).parent.name if a.checkpoint else a.policy}"
                                            f"_stage{a.stage}.json")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        json.dump(out, f, indent=1, default=float)
    print("wrote", path)


if __name__ == "__main__":
    main()
