"""Curriculum stages and per-episode reset sampling for KukaWeldEnv.

Stage 0  reach1  : one random arm, one weld it can reach (data/weld_ik.json), start near an IK
                   solution of that weld (joint noise sigma ~ U(near_sigma)); the other arm idles
                   near HOME. Short episodes.
Stage 1  queue1  : one arm, a queue of 2..5 reachable welds, start HOME + noise (or near the
                   first weld with prob p_near).
Stage 2  both    : both arms, random weld subsets (2..10), env assignment (greedy by default),
                   start HOME + noise.
Stage 3  full    : all 15 welds (randomised positions, queue shuffles), plus the exact canonical
                   task from HOME with prob p_canonical.
Each stage also replays earlier stages with probability `mix` (anti-forgetting).

`sample_episode(env, stage, rng)` mutates `env.cfg` fields that reset() reads (episode_len,
assignment, active_robots) and returns reset options; `reset_env(env, stage, rng, seed)` does both.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from functools import lru_cache

import numpy as np

from env.cell import ROBOT_IDS, ROOT


@dataclass
class StageSpec:
    stage: int = 0
    episode_len: int | None = None       # None = per-stage default (scales with queue length)
    near_sigma: tuple = (0.02, 0.15)     # rad, stage 0/1 start noise around an IK solution
    p_pre_weld: float = 0.5              # stage 0: start around the pre-weld (standoff) solution
    p_near: float = 0.3                  # stage 1: start near the first weld instead of HOME
    queue_len: tuple = (2, 5)            # stage 1
    n_welds: tuple = (2, 10)             # stage 2
    p_canonical: float = 0.25            # stage 3
    assignment: str = "greedy"           # stages 2-3: env assignment ("greedy" | "umarell")
    start_noise: float = 0.1             # HOME start noise (rad)
    mix: float = 0.2                     # prob. of an episode from an earlier stage
    steps_per_weld: int = 200            # episode length budget per weld in the longest queue
    base_len: int = 100

    def to_dict(self):
        return asdict(self)


@lru_cache(maxsize=4)
def ik_table(cell_id: str = "welding_cell_2_small") -> dict:
    """{rid: {wid: {"q": [(6,)...], "pre": [(6,)...]}}} of usable IK solutions (Track X)."""
    p = ROOT / "data" / ("weld_ik.json" if cell_id == "welding_cell_2_small" else f"weld_ik_{cell_id}.json")
    out = {r: {} for r in ROBOT_IDS}
    if not p.exists():
        return out
    d = json.load(open(p))
    from robotmodel.stamp import check_table              # warns if solved for another robot model (P3U round 2)
    check_table(p, cell_id, "IK table")
    for wid, w in d["welds"].items():
        for rid, rr in w["robots"].items():
            sols = [s for s in rr.get("solutions", []) if s.get("usable", s.get("valid"))]
            if sols:
                out[rid][wid] = {"q": [np.array(s["q"]) for s in sols],
                                 "pre": [np.array(s["pre_weld"]["q"]) for s in sols
                                         if s.get("pre_weld") and s["pre_weld"].get("valid")]}
    return out


def robust_free(env, q0, rng, n: int = 6, sigma: float = 0.04) -> bool:
    """q0 and n random perturbations (sigma rad) are all collision-free: not on a contact boundary."""
    if env.check_collisions(q0)["collision"]:
        return False
    for _ in range(n):
        if env.check_collisions(np.clip(q0 + rng.normal(0, sigma, q0.shape), env.q_lo, env.q_hi))["collision"]:
            return False
    return True


def _free_start(env, q, rng, sigma_other, idle_rid=None, tries=20, robust: int = 6):
    """Collision-free (2,6) start away from contact boundaries; q: dict rid -> nominal joint vector."""
    for _ in range(tries):
        q0 = np.stack([q[r] for r in ROBOT_IDS]).copy()
        for i, r in enumerate(ROBOT_IDS):
            if r == idle_rid:
                q0[i] = env.home[i] + rng.normal(0, sigma_other, 6)
        q0 = np.clip(q0, env.q_lo, env.q_hi)
        if robust_free(env, q0, rng, n=robust):
            return q0
        q = {r: (v + rng.normal(0, 0.05, 6)) for r, v in q.items()}
    return None


def sample_episode(env, spec: StageSpec, rng: np.random.Generator) -> dict:
    """Configure env.cfg for one episode of `spec` and return reset options."""
    stage = spec.stage
    if stage > 0 and rng.random() < spec.mix:
        stage = int(rng.integers(0, stage))
    env._curriculum_stage = stage           # reported in the episode's info (rl.vec_env)
    tab = ik_table(env.cfg.cell_id)
    env.cfg.active_robots = ROBOT_IDS
    if stage in (0, 1):
        rids = [r for r in ROBOT_IDS if tab[r]] or list(ROBOT_IDS)
        rid = rids[int(rng.integers(len(rids)))]
        other = [r for r in ROBOT_IDS if r != rid][0]
        reach = sorted(tab[rid]) or [w.wid for w in env.cell.welds]
        if stage == 0:
            welds = [reach[int(rng.integers(len(reach)))]]
        else:
            k = int(rng.integers(spec.queue_len[0], spec.queue_len[1] + 1))
            welds = list(rng.choice(reach, min(k, len(reach)), replace=False))
        sig = rng.uniform(*spec.near_sigma)
        near = stage == 0 or rng.random() < spec.p_near
        q0 = None
        if near and welds[0] in tab[rid]:
            ent = tab[rid][welds[0]]
            pool = ent["pre"] if (ent["pre"] and rng.random() < spec.p_pre_weld) else ent["q"]
            qn = pool[int(rng.integers(len(pool)))] + rng.normal(0, sig, 6)
            i = ROBOT_IDS.index(rid)
            q0 = _free_start(env, {rid: qn, other: env.home[1 - i]}, rng, spec.start_noise, idle_rid=other)
        if q0 is None:
            q0 = env._sample_start(spec.start_noise)
        env.cfg.episode_len = spec.episode_len or (spec.base_len + spec.steps_per_weld * len(welds))
        return {"welds": welds, "assignment": {rid: welds, other: []}, "q0": q0}
    env.cfg.assignment = spec.assignment
    if stage == 2:
        n = int(rng.integers(spec.n_welds[0], spec.n_welds[1] + 1))
        env.cfg.episode_len = spec.episode_len or (spec.base_len + spec.steps_per_weld * int(np.ceil(n / 2 + 1)))
        return {"n_welds": (n, n), "start_noise": spec.start_noise}
    env.cfg.episode_len = spec.episode_len or (spec.base_len + spec.steps_per_weld * 9)
    if rng.random() < spec.p_canonical:
        return {"canonical": True}
    return {"n_welds": (15, 15), "start_noise": spec.start_noise}


def reset_env(env, spec: StageSpec, rng: np.random.Generator, seed: int | None = None):
    opts = sample_episode(env, spec, rng)
    return env.reset(seed=seed, options=opts)


DEFAULT_STAGES = {s: StageSpec(stage=s) for s in range(4)}
