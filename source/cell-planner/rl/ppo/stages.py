"""PPO curriculum: rl.curriculum stages 0-3 plus env-default variants and automatic promotion.

Stages (rl/curriculum.py): 0 single-arm reach of one weld from near an IK / pre-weld solution, 1 single-arm
queue of 2-5 welds, 2 both arms with 2-10 random welds, 3 full 15-weld program (+ the exact canonical task).
Additions for PPO:
  * stages 2-3 use the scripted expert's feasible split (`EXPERT_QUEUES`, scripts/run_expert.py) by default,
    so the canonical task and the demonstrations share the assignment and the cycle time is comparable to
    the expert's 46.0 s;
  * stage 3 also draws, with prob. `p_default`, an episode from the env's default randomisation
    (1-15 welds, +-3 cm, queue shuffles, start noise 0.1 rad, 2000 steps) -- the distribution of the
    held-out variants in execute.py / evaluate_ppo.py;
  * deterministic evaluation slots (one or more envs per rank act with the policy mean and are not
    trained on) draw the *current* stage only (no mixing); in stage 3 they run the canonical task with
    prob. `det_canonical`.
Promotion (rank 0, `Promoter`): when the deterministic success rate of the current stage over the last
`window` finished eval episodes is >= `success`, with collision rate <= `max_collision`, after at least
`min_iters` iterations in the stage.
"""
from __future__ import annotations

import json
from collections import deque
from dataclasses import asdict, dataclass, replace

import numpy as np

from rl.curriculum import StageSpec, sample_episode

# copy of scripts/run_expert.py EXPERT_QUEUES (Track X; feasible with the hull-aware IK table)
EXPERT_QUEUES = {
    "robot1": ["W03", "W02", "W06", "W05", "W09", "W08"],
    "robot2": ["W15", "W13", "W14", "W10", "W12", "W11", "W07", "W04", "W01"],
}


def assignment_of(name):
    if isinstance(name, dict):
        return name
    if name == "expert":
        return {k: list(v) for k, v in EXPERT_QUEUES.items()}
    return name   # "greedy" | "umarell"


@dataclass
class StageCfg:
    assignment: str = "expert"      # stages 2-3 (and canonical): expert | greedy | umarell
    p_default: float = 0.3          # stage 3: env-default randomised episode
    default_len: int = 2000
    det_canonical: float = 0.5      # stage 3 eval slots: canonical task
    mix: float = 0.2                # replay of earlier stages (training slots)
    p_canonical: float = 0.25       # stage 3 training slots: canonical
    stage3_len: int = 1900
    # v2 (I-7): stages 2-3 draw random reachability-feasible weld splits and queue orders, start noise
    # U(0, start_noise_max) rad and position offsets U(0, pos_noise_max) per axis; `home_extra` steps are added
    # to every episode for the homing phase (EnvConfig.require_home)
    split: str = "expert"           # "expert" (v1) | "random"
    p_expert_split: float = 0.25
    pos_noise_max: float = 0.05
    start_noise_max: float = 0.3
    home_extra: int = 0
    home_from_stage: int = 1        # EnvConfig.require_home is switched on from this stage on (if the env has it)
    wp_hard_from_stage: int = 0     # stages below this train with EnvConfig.workpiece_soft (penalty, no termination)
    max_tilt: float | None = None   # v2: only IK solutions with tool-axis tilt <= this (deg) count as reachable
                                    # (tight training gates: W01/W04 need 3-4.5 deg tilt on robot1)

    def to_dict(self):
        return asdict(self)


def stage_spec(stage: int, cfg: StageCfg, deterministic: bool = False) -> StageSpec:
    a = assignment_of(cfg.assignment)
    return StageSpec(stage=stage, assignment=a, mix=0.0 if deterministic else cfg.mix,
                     p_canonical=(cfg.det_canonical if deterministic else cfg.p_canonical))
    # episode length: rl.curriculum defaults per sampled stage (stage 3: 100 + 200 x 9 = 1900 steps)


_TILT = {"max": None}


def tilt_filtered_table(cell_id: str = "welding_cell_2_small") -> dict:
    """rl.curriculum.ik_table restricted to solutions with tilt <= _TILT['max'] (same format)."""
    from env.cell import ROOT as R0
    p = R0 / "data" / ("weld_ik.json" if cell_id == "welding_cell_2_small" else f"weld_ik_{cell_id}.json")
    key = (cell_id, _TILT["max"])
    if key in _TAB_CACHE:
        return _TAB_CACHE[key]
    out = {r: {} for r in ("robot1", "robot2")}
    d = json.load(open(p))
    from robotmodel.stamp import check_table              # warns if solved for another robot model (P3U round 2)
    check_table(p, cell_id, "IK table")
    for wid, w in d["welds"].items():
        for rid, rr in w["robots"].items():
            sols = [s for s in rr.get("solutions", []) if s.get("usable", s.get("valid"))
                    and (_TILT["max"] is None or s.get("tilt_deg", 0.0) <= _TILT["max"])]
            if sols:
                out[rid][wid] = {"q": [np.array(s["q"]) for s in sols],
                                 "pre": [np.array(s["pre_weld"]["q"]) for s in sols
                                         if s.get("pre_weld") and s["pre_weld"].get("valid")]}
    _TAB_CACHE[key] = out
    return out


_TAB_CACHE = {}


def set_max_tilt(max_tilt):
    """Restrict reachability (stages 0-1 via rl.curriculum, random splits) to IK solutions with small tilt."""
    import rl.curriculum as cur
    _TILT["max"] = max_tilt
    if max_tilt is not None:
        cur.ik_table = tilt_filtered_table


def random_split(env, wids, rng):
    """Random reachability-feasible split (data/weld_ik.json usable IK solutions) and random queue orders."""
    import rl.curriculum as cur
    tab = cur.ik_table(env.cfg.cell_id)
    q = {"robot1": [], "robot2": []}
    for w in wids:
        cand = [r for r in ("robot1", "robot2") if w in tab[r]] or ["robot1", "robot2"]
        q[cand[int(rng.integers(len(cand)))]].append(w)
    return {r: [v[j] for j in rng.permutation(len(v))] for r, v in q.items()}


def start_clear(env, q0, wids, lateral=0.15) -> bool:
    """False if a TCP starts below the plane of an episode weld within `lateral` of it (i.e. under the panel)."""
    if q0 is None or not getattr(env.cfg, "workpiece", False):
        return True
    import mujoco
    d = env._scratch
    d.qpos[env.qadr] = np.asarray(q0).reshape(2, 6)
    mujoco.mj_kinematics(env.model, d)
    for i in range(2):
        p = d.site_xpos[env.tcp_site[i]]
        for w in wids:
            wp = env.weld_by_id[w].pos
            if np.linalg.norm(p[:2] - wp[:2]) < lateral and p[2] < wp[2] + 0.005:
                return False
    return True


def reset_options_v2(env, stage: int, cfg: StageCfg, rng: np.random.Generator, deterministic=False,
                     force_kind: str | None = None) -> dict:
    st = stage
    if not deterministic and stage > 0 and force_kind is None and rng.random() < cfg.mix:
        st = int(rng.integers(0, stage))
    env.cfg.active_robots = ("robot1", "robot2")
    if not hasattr(env, "_ppo_home"):
        env._ppo_home = bool(env.cfg.require_home)
    env.cfg.require_home = env._ppo_home and st >= cfg.home_from_stage
    if getattr(env, "wp", None) is not None or getattr(env, "wedge", None) is not None:
        env.cfg.workpiece_soft = st < cfg.wp_hard_from_stage
    if st <= 1 and force_kind is None:
        for _ in range(10):
            opts = sample_episode(env, replace(stage_spec(st, cfg, True), stage=st), rng)
            if start_clear(env, opts.get("q0"), opts.get("welds", [])):
                break
        else:
            opts["q0"] = env.home.copy()
        env._ppo_kind = f"s{st}"
        env.cfg.episode_len += cfg.home_extra if env.cfg.require_home else 0
        return opts
    env._curriculum_stage = st
    if force_kind:
        kind = force_kind
    elif st == 2:
        kind = "s2"
    else:
        r = rng.random()
        pc = cfg.det_canonical if deterministic else cfg.p_canonical
        kind = "canonical" if r < pc else ("default" if r < pc + (1 - pc) * cfg.p_default else "full")
    env._ppo_kind = kind
    if kind == "canonical":
        env.cfg.assignment = assignment_of(cfg.assignment)
        env.cfg.episode_len = cfg.stage3_len + cfg.home_extra
        return {"canonical": True}
    n = {"s2": lambda: int(rng.integers(2, 11)), "full": lambda: 15, "default": lambda: int(rng.integers(1, 16))}[kind]()
    wids = sorted(rng.choice(env.wids, n, replace=False).tolist())
    if rng.random() < cfg.p_expert_split:
        ex = assignment_of(cfg.assignment) if isinstance(assignment_of(cfg.assignment), dict) else EXPERT_QUEUES
        queues = {r: [w for w in ex[r] if w in wids] for r in ("robot1", "robot2")}
    else:
        queues = random_split(env, wids, rng)
    longest = max(len(v) for v in queues.values())
    env.cfg.episode_len = min(2400, 100 + 200 * (longest + 1)) + cfg.home_extra
    return {"welds": wids, "assignment": queues, "start_noise": float(rng.uniform(0, cfg.start_noise_max)),
            "pos_noise": float(rng.uniform(0, cfg.pos_noise_max))}


def reset_options(env, stage: int, cfg: StageCfg, rng: np.random.Generator, deterministic=False) -> dict:
    """Configure env.cfg for one episode and return reset options (+ marks env._ppo_kind)."""
    if cfg.max_tilt is not None and _TILT["max"] != cfg.max_tilt:
        set_max_tilt(cfg.max_tilt)
    if getattr(env, "wp", None) is not None or getattr(env, "wedge", None) is not None:
        env.cfg.workpiece_soft = stage < cfg.wp_hard_from_stage
    if cfg.split == "random":
        return reset_options_v2(env, stage, cfg, rng, deterministic)
    spec = stage_spec(stage, cfg, deterministic)
    kind = None
    if stage >= 3:
        r = rng.random()
        if deterministic:
            kind = "canonical" if r < cfg.det_canonical else (
                "default" if r < cfg.det_canonical + (1 - cfg.det_canonical) * cfg.p_default else "full")
        else:
            kind = "default" if r < cfg.p_default else None
        if kind in ("canonical", "full"):
            spec = replace(spec, p_canonical=1.0 if kind == "canonical" else 0.0)
    if kind == "default":
        env._curriculum_stage = 3
        env.cfg.active_robots = ("robot1", "robot2")
        env.cfg.assignment = assignment_of(cfg.assignment)
        env.cfg.episode_len = cfg.default_len
        env.cfg.n_welds = (1, 15)
        env._ppo_kind = "default"
        return {}
    opts = sample_episode(env, spec, rng)
    env._ppo_kind = "canonical" if opts.get("canonical") else f"s{env._curriculum_stage}"
    return opts


class Promoter:
    def __init__(self, stage=0, max_stage=3, success=0.8, max_collision=0.1, window=200, min_iters=20,
                 min_episodes=100):
        self.stage, self.max_stage = stage, max_stage
        self.success, self.max_collision = success, max_collision
        self.window, self.min_iters, self.min_episodes = window, min_iters, min_episodes
        self.hist = deque(maxlen=window)
        self.iters_in_stage = 0

    def state(self):
        return dict(stage=self.stage, hist=list(self.hist), iters_in_stage=self.iters_in_stage)

    def load(self, st):
        self.stage = st["stage"]
        self.hist = deque(st.get("hist", []), maxlen=self.window)
        self.iters_in_stage = st.get("iters_in_stage", 0)

    def add(self, episodes):
        """episodes: dicts from the rollout (det, stage, success, collision)."""
        for e in episodes:
            if e["det"] and e["stage"] == self.stage and e.get("kind") != "default":
                self.hist.append((bool(e["success"]), bool(e["collision"])))

    def rates(self):
        if not self.hist:
            return 0.0, 0.0, 0
        h = np.array(self.hist, float)
        return float(h[:, 0].mean()), float(h[:, 1].mean()), len(h)

    def step(self) -> bool:
        """Call once per iteration; returns True when promoted."""
        self.iters_in_stage += 1
        s, c, n = self.rates()
        if (self.stage < self.max_stage and self.iters_in_stage >= self.min_iters and n >= self.min_episodes
                and s >= self.success and c <= self.max_collision):
            self.stage += 1
            self.hist.clear()
            self.iters_in_stage = 0
            return True
        return False
