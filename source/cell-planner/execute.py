#!/usr/bin/env python3
"""Controller-agnostic execution + verification pipeline (TASK.md step 4, Track Ex).

    PYTHONPATH=. .venv/bin/python execute.py --controller expert[:<assignment>] [--cell welding_cell_2_small]
    PYTHONPATH=. .venv/bin/python execute.py --controller umarell:bt_1790851970 --on-collision continue
    PYTHONPATH=. .venv/bin/python execute.py --controller hier:runs/h1/best.pt --safety-filter on
    PYTHONPATH=. .venv/bin/python execute.py --controller residual:<ckpt> --variants 100 --seed-base 50000
    PYTHONPATH=. .venv/bin/python execute.py --controller e2e:<ckpt> --skip-validate

Pipeline for the canonical 15-weld task from HOME (deterministic):
  1. rollout   env.step with the controller's actions (optionally through the safety filter),
               until all welds are done (or a collision / the step limit), then a coordinated return
               HOME (``--home-tail expert``: the scripted expert's homing planner, reported separately).
               Records t, q (N,2,6), raw + executed actions, weld events, collisions per kind.
  2. replay    a FRESH env follows the recorded joint timeline (feed-forward actions, no termination):
               welds counted by the env's own pose check + dwell, collisions per kind per step, and an
               exact-pose pass (env.check_collisions per sample) split into the umarell gate (sphere
               arm-arm clearance > 0.20 m, sphere floor/box rule) and the MuJoCo hull check (self, floor,
               arm-arm, fixture contacts), plus signed hull distances (mj_geomDistance: self, arm-arm).
  3. export    umarell BT plan JSON (``--export-mode sync_rest``, default: like sync_min below, and every
               executor pause happens with both arms at rest: one common-clock time warp brakes both arms to rest
               at each cut within 0.49 a_max (or the rollout's own peak) and re-accelerates them; the path and
               the pairing of the two arms in time are unchanged. Dense 200 Hz v / a / jerk check of the
               exported plan in the report. ``sync_min``: both arms cut synchronously, but only at a
               minimal set of instants that stab every weld's in-tolerance window, so every weld leg ends
               on its weld; each cut costs the executor's fixed 1.0 s completion margin. ``sync_all`` =
               ``player.bt_plan.legs_from_rollout`` cut at every weld completion with hold 0.5 s.)
               The exported plan is replayed on the executor clock (player.bt_plan.timeline_from_bt_plan)
               and that timeline goes through the same fresh-env replay.
  4. validate  scripts/umarell_validate.py in .venv-umarell on the exported plan (validate_trajectory_set
               per step, full-timeline inter_robot_clearance, validate_approach per weld, obstacle checks,
               hpp-fcl mesh self-collision audit).
  5. results/exec_<name>.json with everything; joint timelines in data/exec/.

``--variants N --seed-base 50000``: N held-out randomised variants (env default randomisation:
1-15 welds, +-3 cm weld noise, queue shuffles, start noise 0.1 rad; seeds seed_base+i), rollout only.

Controllers (``--controller kind[:arg]``): expert[:expert|greedy|umarell] (scripted expert, Track X),
umarell:<plan name or plan .json> (feed-forward replay of a saved umarell plan on its executor clock),
hier:<ckpt>, residual:<ckpt>, e2e:<ckpt>, ppo:<ckpt> (interface: docs/DESIGN.md, Track Ex change log).
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
import subprocess
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from env.cell import ROBOT_IDS, plan_path  # noqa: E402

KINDS = ("arm_arm", "self", "floor", "fixture")
DT = 0.05
UMARELL_PY = ROOT / ".venv-umarell" / "bin" / "python"
CELL_SHORT = {"welding_cell_2_small": "2_small", "welding_cell_2t": "2t"}


def _r(x, n=4):
    return None if x is None else round(float(x), n)


def rel(p) -> str:
    p = Path(p).resolve()
    return str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p)


def _jsonable(o):
    if hasattr(o, "tolist"):
        return o.tolist()
    if isinstance(o, (set, tuple)):
        return list(o)
    return str(o)


# ════════════════════════════════════════════════════════════════════════════
#  env
# ════════════════════════════════════════════════════════════════════════════
def make_env(cell: str, assignment=None, **overrides):
    from env.weld_env import EnvConfig, KukaWeldEnv
    from env.rewards import RewardConfig
    kw = dict(overrides)
    rew = kw.pop("reward", None)
    cfg = EnvConfig(cell_id=cell, **kw)
    if assignment is not None:
        cfg.assignment = assignment
    if rew is not None:
        cfg.reward = rew if isinstance(rew, RewardConfig) else RewardConfig(**rew)
    return KukaWeldEnv(cfg)


# ════════════════════════════════════════════════════════════════════════════
#  controllers
# ════════════════════════════════════════════════════════════════════════════
class ExpertController:
    """Track X ScriptedExpert (expert/scripted.py)."""

    def __init__(self, env):
        from expert.scripted import ScriptedExpert
        self.ex = ScriptedExpert(env)
        self.stats = self.ex.stats

    def reset(self, obs):
        self.ex.reset()
        self.stats = self.ex.stats

    def act(self, obs):
        return self.ex.act(obs)

    def idle(self):
        return self.ex.idle()


class TimelineController:
    """Feed-forward replay of a joint timeline sampled at the env dt: a_k = (q[k+1] - env.q) / (v_max dt).
    Reads env.q (float64) instead of the float32 obs so the replay does not drift."""

    def __init__(self, env, tl, queues=None, stats=None):
        self.env, self.tl, self.k = env, tl, 0
        self.queues = queues
        self.stats = {"max_abs_action": 0.0, **(stats or {})}

    def reset(self, obs):
        self.k = 0

    def act(self, obs):
        k = min(self.k + 1, len(self.tl.q) - 1)
        a = (self.tl.q[k] - self.env.q) / (self.env.v_max * self.env.dt)
        self.stats["max_abs_action"] = max(self.stats["max_abs_action"], float(np.abs(a).max()))
        self.k += 1
        return np.clip(a, -1, 1).ravel()

    @property
    def done(self):
        return self.k >= len(self.tl.q) - 1


def hier_controller(arg, env):
    """Track H (hierarchical scheduler over motion primitives, evaluate_hier.py): the policy is not a per-step
    joint-velocity controller, so its canonical episode is generated by its own HierWeldEnv + executor and the
    resulting 20 Hz joint timeline (incl. its own return HOME) is then driven through KukaWeldEnv by
    TimelineController (independent env check, export, validation). ``arg``: ckpt[,legacy_v2][,thr=0.3]
    (legacy_v2 is implied for runs/hier_e, runs/hier_f, as in docs/hierarchical.md)."""
    import evaluate_hier as EH
    parts = arg.split(",")
    ckpt = parts[0]
    legacy = "legacy_v2" in parts or any(x in ckpt for x in ("hier_e/", "hier_f/"))
    thr = 0.3
    la = None
    for p in parts[1:]:
        if p.startswith("thr="):
            thr = float(p[4:])
        elif p.startswith("la="):
            la = p[3:]
    la_mode, la_models, la_note = _hier_lookahead(ckpt, la)
    o = dict(spec=ckpt, safe_thr=thr, no_lookahead=False, cell=env.cfg.cell_id, speed_profile="ours", max_tilt=None,
             min_self_dist=None, legacy_v2=legacy, ood=False, select=False)
    with _hier_la_installed(la_mode, la_models):
        EH._init(o, True)
        res, (t, q, ev, arrivals, legs) = EH.run_episode(EH._G["env"], EH._G["policy"], 0, True)
    from player.timeline import timeline_from_arrays
    tl = timeline_from_arrays(np.round(np.asarray(t, float), 10), q, weld_events=[tuple(e) for e in ev])
    queues = {rid: [w for r_, w, *_ in sorted(ev, key=lambda e: e[3]) if r_ == rid] for rid in ROBOT_IDS}
    st = {k: res.get(k) for k in ("success", "welds_done", "n_welds", "makespan", "cycle_time_with_home_s",
                                  "collision", "allocation", "decisions", "home_tail")}
    st.update(source="evaluate_hier.run_episode (canonical)", legacy_v2=legacy, safe_thr=thr, checkpoint=ckpt,
              lookahead=la_mode or "engineered", lookahead_models=la_models)
    if la_note:
        st["lookahead_note"] = la_note
    return TimelineController(env, tl, queues=queues, stats=st)


def _hier_lookahead(ckpt, la):
    """Lookahead of a hier checkpoint (P3U round 2; track G's learned lookahead, rl/hier/la_features.py). ``la``:
    None (default: the HIER_LA env var if set, else the engineered lookahead, as before), ``auto`` (the mode the
    checkpoint was trained with: <run dir>/la_args.json), ``engineered``, or ``<mode>[:<model dir>]`` (field,
    field_dense, opt, distill). -> (mode or None, model dir or None, note). A learned lookahead was trained on the
    cell's own robot: under a robot override that changes the kinematics it is refused (like rl.hier.instances
    .load_pool)."""
    import os
    import warnings
    run = Path(ckpt).parent
    la_args = {}
    if (run / "la_args.json").exists():
        try:
            la_args = json.load(open(run / "la_args.json"))
        except (OSError, ValueError):
            la_args = {}
    trained = la_args.get("lookahead")
    note = None
    if la is None:
        env_v = os.environ.get("HIER_LA")
        if env_v:
            la = env_v
        elif trained and trained != "engineered":
            note = (f"{ckpt} was trained with the learned lookahead {trained!r} (la_args.json); running the engineered "
                    f"lookahead (pass la=auto or la={trained} to use the learned one)")
            warnings.warn(note, UserWarning, stacklevel=3)
            return None, None, note
        else:
            return None, None, None
    if la == "auto":
        if not trained:
            return None, None, f"la=auto: no la_args.json lookahead in {run}; engineered lookahead"
        mode, models = trained, la_args.get("la_models")
    else:
        mode, _, models = la.partition(":")
        models = models or la_args.get("la_models")
    if mode not in ("engineered", "field", "field_dense", "opt", "distill"):
        raise SystemExit(f"hier: unknown lookahead mode {mode!r} (engineered | field | field_dense | opt | distill | auto)")
    if mode == "engineered":
        return None, None, note
    from robotmodel.override import active_override
    ov = active_override()
    geo = ("urdf", "base", "yaw", "tip_link", "tcp_offset", "home_deg")
    if ov is not None and any(k in (e or {}) for e in (ov.get("robots") or {}).values() for k in geo):
        raise SystemExit(f"hier lookahead {mode!r}: the learned clearance predictor ({models}) was trained on the "
                         "cell's own robot kinematics; a robot override that changes the URDF / base / TCP / home is "
                         "not supported for it (use la=engineered, or retrain it, rl/hier/la_*)")
    from rl.hier.la_features import DEFAULT_MODELS
    return mode, models or DEFAULT_MODELS, note


class _hier_la_installed:
    """Install a learned lookahead for one episode, then restore the previous HierWeldEnv.obs / HIER_LA."""

    def __init__(self, mode, models):
        self.mode, self.models = mode, models

    def __enter__(self):
        import os
        if self.mode is None:
            return self
        from rl.hier import hier_env as HE
        from rl.hier import la_features as LF
        self.prev_obs, self.prev_env = HE.HierWeldEnv.obs, os.environ.get("HIER_LA")
        self.prev_state = dict(LF._STATE)
        LF.install(self.mode, self.models)
        return self

    def __exit__(self, *exc):
        import os
        if self.mode is None:
            return False
        from rl.hier import hier_env as HE
        from rl.hier import la_features as LF
        HE.HierWeldEnv.obs = self.prev_obs
        LF._STATE.clear()
        LF._STATE.update(self.prev_state)
        if self.prev_env is None:
            os.environ.pop("HIER_LA", None)
        else:
            os.environ["HIER_LA"] = self.prev_env
        return False


class TD3Controller:
    """Built-in e2e loader: rl.td3.load_agent + evaluate.make_policy (action map from the checkpoint)."""

    def __init__(self, ckpt, env):
        from evaluate import make_policy
        from rl.action_map import ActionMap
        from rl.td3 import load_agent
        agent, ck = load_agent(ckpt, "cpu")
        self.fn = make_policy(agent, ActionMap(ck.get("action_map"), env.cfg.cell_id))
        self.stats = {"checkpoint_keys": sorted(k for k in ck if not k.startswith("agent"))}

    def reset(self, obs):
        pass

    def act(self, obs):
        return np.asarray(self.fn([obs])[0], np.float64)


class RandomController:
    """Test-only: uniform random actions (seeded) -- exercises the safety filter and collision paths."""

    def __init__(self, seed=0, scale=1.0):
        self.seed, self.scale = int(seed), float(scale)
        self.stats = {}

    def reset(self, obs):
        self.rng = np.random.default_rng(self.seed)

    def act(self, obs):
        return self.rng.uniform(-self.scale, self.scale, 12)


POLICY_MODULES = {"hier": ("rl.hier.controller",),
                  "residual": ("rl.residual", "rl.residual.controller", "rl.residual_controller"),
                  "e2e": ("rl.e2e", "rl.e2e.controller"),
                  "ppo": ("rl.ppo", "rl.ppo.controller")}


def _policy_module(kind):
    errs = []
    for name in POLICY_MODULES[kind]:
        try:
            mod = importlib.import_module(name)
        except ImportError as e:
            errs.append(f"{name}: {e}")
            continue
        if hasattr(mod, "load_controller"):
            return mod
        errs.append(f"{name}: no load_controller")
    return errs


def parse_controller(spec: str):
    kind, _, arg = spec.partition(":")
    if kind not in ("expert", "umarell", "random", "timeline", "hier_la") + tuple(POLICY_MODULES):
        raise SystemExit(f"unknown controller kind {kind!r}")
    return kind, arg


def controller_env_kwargs(kind, arg) -> dict:
    """EnvConfig overrides a controller needs (assignment for expert / umarell, checkpoint env_kwargs)."""
    if kind == "expert":
        a = arg or "expert"
        if a == "expert":
            from scripts.run_expert import EXPERT_QUEUES
            return {"assignment": {k: list(v) for k, v in EXPERT_QUEUES.items()}}
        return {"assignment": a}
    if kind in ("umarell", "random", "timeline", "hier", "hier_la"):
        return {}
    mod = _policy_module(kind)
    if not isinstance(mod, list) and hasattr(mod, "env_kwargs"):
        return dict(mod.env_kwargs(arg))
    if kind == "e2e" and isinstance(mod, list):
        import torch
        ck = torch.load(arg, map_location="cpu", weights_only=False)
        return {k: v for k, v in dict(ck.get("env_kwargs", {})).items() if k != "cell_id"}
    return {}


def build_controller(kind, arg, env):
    if kind == "expert":
        return ExpertController(env)
    if kind == "umarell":
        return TimelineController(env, umarell_timeline(arg))
    if kind == "random":
        return RandomController(int(arg or 0))
    if kind == "timeline":
        from player.timeline import Timeline
        tl = Timeline.load(arg)
        return TimelineController(env, tl, queues={rid: [w for r_, w, *_ in sorted(tl.weld_events, key=lambda e: e[3])
                                                         if r_ == rid] for rid in ROBOT_IDS})
    if kind == "hier_la":        # hier scheduler with the lookahead it was trained with (track G; la_args.json)
        return hier_controller(arg if ",la=" in arg else arg + ",la=auto", env)
    if kind == "hier" and isinstance(_policy_module(kind), list):
        return hier_controller(arg, env)
    mod = _policy_module(kind)
    if isinstance(mod, list):
        if kind == "e2e":
            return TD3Controller(arg, env)
        raise SystemExit(f"controller {kind}: no loader yet ({'; '.join(mod)}). "
                         f"Implement load_controller(ckpt, env) as in docs/DESIGN.md (Track Ex).")
    return mod.load_controller(arg, env)


def umarell_plan_file(arg: str) -> Path:
    p = Path(arg)
    return p if p.suffix == ".json" else plan_path(arg)


def umarell_timeline(arg: str):
    from player.bt_plan import timeline_from_bt_plan
    return timeline_from_bt_plan(str(umarell_plan_file(arg)))


def queues_from_timeline(tl) -> dict:
    return {rid: [l.op_id for l in tl.robot_legs(rid) if l.kind == "weld"] for rid in ROBOT_IDS}


# ════════════════════════════════════════════════════════════════════════════
#  safety filter
# ════════════════════════════════════════════════════════════════════════════
class SafetyFilter:
    """One-step-lookahead projection filter on the normalised joint-velocity action.

    The candidate next configuration (with the env's own collision sub-samples) must pass
    ``env.check_collisions`` (umarell gate: sphere arm-arm clearance > 0.20 m + ``margin``, sphere
    floor/box rule; hull check: no self / floor / arm-arm / fixture contact). If the raw action fails,
    the closest safe candidate (in action norm) is executed; per arm the options are: raw, scaled by
    0.5 / 0.25, stopped, or raw with one joint zeroed (all combinations of the two arms). Stopping is always feasible in the kinematic
    env if the current state is free, so the filter only fails when it starts in collision.
    Every substitution is an *intervention* (counted, with the reason)."""

    SCALES = (0.5, 0.25)

    def __init__(self, env, margin=0.005, enabled=True):
        self.env, self.margin, self.enabled = env, margin, enabled
        self.n_steps = self.n_interventions = self.n_unrecoverable = 0
        self.reasons = Counter()
        self.log = []

    def _ok(self, q0, a):
        env = self.env
        a = np.clip(np.asarray(a, float).reshape(2, 6), -1, 1)
        q1 = np.clip(q0 + a * env.v_max * env.dt, env.q_lo, env.q_hi)
        ns = max(1, env.cfg.n_substeps)
        why = []
        for s in range(1, ns + 1):
            c = env.check_collisions(q0 + (q1 - q0) * s / ns)
            if c["collision"]:
                why += c["kinds"]
            elif c["min_clearance"] <= env.spheres.warning_distance + self.margin:
                why.append("clearance_margin")
            if why:
                return False, why
        return True, why

    def __call__(self, a, t=None):
        a = np.asarray(a, float).reshape(-1)
        if not self.enabled:
            return a, False
        self.n_steps += 1
        q0 = self.env.q
        ok, why = self._ok(q0, a)
        if ok:
            return a, False
        A = a.reshape(2, 6)
        opts = []
        for i in range(2):
            o = [A[i] * s for s in (1.0,) + self.SCALES + (0.0,)]
            for j in range(6):
                if A[i, j] != 0:
                    c = A[i].copy()
                    c[j] = 0
                    o.append(c)
            opts.append(o)
        cands = [np.stack([x, y]) for x in opts[0] for y in opts[1]][1:]
        cands.sort(key=lambda c: float(np.linalg.norm(c - A)))
        chosen = None
        for c in cands:
            if self._ok(q0, c)[0]:
                chosen = c.reshape(-1)
                break
        if chosen is None:
            chosen = np.zeros(12)
            if not self._ok(q0, chosen)[0]:
                self.n_unrecoverable += 1
        self.n_interventions += 1
        for w in set(why):
            self.reasons[w] += 1
        if len(self.log) < 200:
            self.log.append({"t": _r(t, 3), "why": sorted(set(why)),
                             "scale_r1": _r(np.abs(chosen[:6]).sum() / max(1e-9, np.abs(a[:6]).sum()), 3),
                             "scale_r2": _r(np.abs(chosen[6:]).sum() / max(1e-9, np.abs(a[6:]).sum()), 3)})
        return chosen, True

    def summary(self):
        return {"enabled": self.enabled, "margin_m": self.margin, "steps": self.n_steps,
                "interventions": self.n_interventions,
                "intervention_rate": _r(self.n_interventions / max(1, self.n_steps), 4),
                "unrecoverable": self.n_unrecoverable, "reasons": dict(self.reasons), "first": self.log[:50]}


# ════════════════════════════════════════════════════════════════════════════
#  rollout
# ════════════════════════════════════════════════════════════════════════════
def rollout(env, ctrl, *, reset_options=None, seed=None, max_steps=None, sfilter=None, on_collision="stop",
            home_tail="expert", tail_max=1200, record=True, home_tol=1e-4):
    """Run ``ctrl`` in ``env``. Returns a record dict (arrays + scalars). ``home_tol`` (rad): max |q - HOME| that
    counts as "at HOME" (tail stop + MoveHome export); default 1e-4 (PPO v4 self-homing: --home-tol 0.01)."""
    env.cfg.terminate_on_collision = on_collision == "stop"
    if max_steps is not None:
        env.cfg.episode_len = int(max_steps)
    obs, info = env.reset(seed=seed, options=reset_options)
    ctrl.reset(obs)
    t, q = [0.0], [env.q.copy()]
    a_raw, a_exe, filt = [], [], []
    coll_steps = defaultdict(list)
    clr = [float(info["min_clearance"])]
    first_coll = None
    n_welds = int(info["n_welds"])
    t_wall = time.time()
    k = 0
    term = trunc = False
    while True:
        a = np.asarray(ctrl.act(obs), float).reshape(-1)
        ae, hit = sfilter(a, t[-1]) if sfilter is not None else (a, False)
        obs, r, term, trunc, info = env.step(ae)
        k += 1
        if record:
            a_raw.append(a); a_exe.append(ae); filt.append(hit)
        t.append(info["time_s"]); q.append(env.q.copy()); clr.append(float(info["min_clearance"]))
        if info["collision"]:
            kinds = env.check_collisions(env.q)["kinds"] or [info["collision_kind"]]
            for kd in kinds:
                coll_steps[kd].append(k)
            if first_coll is None:
                first_coll = {"t_s": _r(info["time_s"], 3), "kind": info["collision_kind"], "kinds": kinds,
                              "contacts": {kk: v[:3] for kk, v in (info.get("contacts") or {}).items() if v}}
        if isinstance(ctrl, TimelineController):
            if ctrl.done or (max_steps is not None and k >= max_steps):
                break
            continue
        if term or trunc or info["success"]:
            break
    success_env = bool(info["success"])
    weld_events = [(r_, w, s * env.dt, e * env.dt) for r_, w, s, e in env.weld_events]
    t_last_weld = _r(max([e[3] for e in weld_events], default=None), 4)
    n_main = k
    # ── return HOME (umarell's makespan includes the final MoveHome) ────────────
    tail = {"mode": home_tail, "steps": 0, "collisions": [], "at_home": None}
    can_tail = success_env and not first_coll and home_tail != "off" and not isinstance(ctrl, TimelineController)
    if can_tail:
        homer = ctrl if (home_tail == "controller" or isinstance(ctrl, ExpertController)) else ExpertController(env)
        if homer is not ctrl:
            homer.reset(obs)
        for _ in range(tail_max):
            a = np.asarray(homer.act(obs), float).reshape(-1)
            obs, r, term, trunc, info = env.step(a)
            k += 1
            if record:
                a_raw.append(a); a_exe.append(a); filt.append(False)
            t.append(info["time_s"]); q.append(env.q.copy()); clr.append(float(info["min_clearance"]))
            tail["steps"] += 1
            if info["collision"]:
                tail["collisions"].append((_r(info["time_s"], 3), info["collision_kind"]))
                for kd in env.check_collisions(env.q)["kinds"] or [info["collision_kind"]]:
                    coll_steps[kd].append(k)
            at_home = bool(np.abs(env.q - env.home).max() < home_tol)
            if at_home and (not hasattr(homer, "idle") or homer.idle()):
                break
        tail["homer"] = "controller" if homer is ctrl else "scripted expert homing (ScriptedExpert, no goals)"
    tail["at_home"] = bool(np.abs(env.q - env.home).max() < home_tol)
    tail["home_err_rad"] = float(np.abs(env.q - env.home).max())
    tail["duration_s"] = _r(tail["steps"] * env.dt, 3)
    rec = dict(t=np.round(np.array(t), 10), q=np.array(q), a_raw=np.array(a_raw), a_exec=np.array(a_exe),
               filtered=np.array(filt, bool), clearance=np.array(clr), weld_events=weld_events,
               n_main_steps=n_main, n_welds=n_welds, welds_done=int(info["welds_done"]),
               success_env=success_env, collision=bool(coll_steps), first_collision=first_coll,
               collision_steps={kd: len(v) for kd, v in coll_steps.items()},
               terminated=bool(term), truncated=bool(trunc), t_last_weld_s=t_last_weld,
               t_end_s=float(t[-1]), home_tail=tail, wall_s=time.time() - t_wall,
               queues={rid: [env.task_ids[i] for i in env.queues[j]] for j, rid in enumerate(ROBOT_IDS)},
               controller_stats=_stats(ctrl))
    rec["success"] = bool(success_env and not coll_steps)
    return rec


def _stats(ctrl):
    s = getattr(ctrl, "stats", None)
    if not isinstance(s, dict):
        return None
    try:
        return json.loads(json.dumps(s, default=_jsonable))
    except Exception:
        return {"repr": str(s)[:500]}


# ════════════════════════════════════════════════════════════════════════════
#  independent replay in a fresh env
# ════════════════════════════════════════════════════════════════════════════
def _intervals(ts, mask):
    out, start, prev = [], None, None
    for tt, m in zip(ts, mask):
        if m and start is None:
            start = tt
        if not m and start is not None:
            out.append([_r(start, 3), _r(prev, 3)])
            start = None
        prev = tt
    if start is not None:
        out.append([_r(start, 3), _r(prev, 3)])
    return out


class HullDistance:
    """Signed convex-hull distances in the env's MuJoCo model (mj_geomDistance), the same geoms and
    exclusions as the env's contact check: self pairs (same arm, different non-adjacent bodies, not
    SRDF-excluded) and all arm-arm pairs."""

    def __init__(self, env, distmax=2.0):
        import mujoco
        self.mj, self.env, self.distmax = mujoco, env, distmax
        m = env.model
        hull = [g for g in range(m.ngeom) if m.geom_contype[g] and env.geom_owner[g] >= 0]
        excl = {int(s) for s in m.exclude_signature}
        filterparent = not (m.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
        self.self_pairs = {0: [], 1: []}
        self.arm_pairs = []
        for i, g1 in enumerate(hull):
            for g2 in hull[i + 1:]:
                b1, b2 = int(m.geom_bodyid[g1]), int(m.geom_bodyid[g2])
                o1, o2 = env.geom_owner[g1], env.geom_owner[g2]
                if o1 != o2:
                    self.arm_pairs.append((g1, g2))
                    continue
                if b1 == b2:
                    continue
                lo, hi = min(b1, b2), max(b1, b2)
                if ((lo << 16) + hi) in excl:
                    continue
                if filterparent and ((m.body_parentid[b1] == b2 and b2 != 0) or (m.body_parentid[b2] == b1 and b1 != 0)):
                    continue
                self.self_pairs[int(o1)].append((g1, g2))
        self.d = mujoco.MjData(m)
        self.ft = np.zeros(6)

    def _name(self, g):
        m = self.env.model
        return f"{m.body(m.geom_bodyid[g]).name}#{g}"

    def __call__(self, q):
        m, d = self.env.model, self.d
        d.qpos[self.env.qadr] = np.asarray(q, float).reshape(2, 6)
        self.mj.mj_kinematics(m, d)
        out = {}
        for r in (0, 1):
            best = (np.inf, None)
            for g1, g2 in self.self_pairs[r]:
                x = self.mj.mj_geomDistance(m, d, g1, g2, self.distmax, self.ft)
                if x < best[0]:
                    best = (x, (g1, g2))
            out[f"self_{ROBOT_IDS[r]}"] = best
        best = (np.inf, None)
        for g1, g2 in self.arm_pairs:
            x = self.mj.mj_geomDistance(m, d, g1, g2, self.distmax, self.ft)
            if x < best[0]:
                best = (x, (g1, g2))
        out["arm_arm"] = best
        return out


def replay_check(cell, t, q, queues, *, hull_every=2, label="rollout"):
    """Fresh env, follow (t, q) exactly. Returns step pass + exact-pose pass (gate vs hull)."""
    from env.kinematics import FLOOR_Z_MIN  # noqa: F401  (documented rule, used via env)
    t, q = np.asarray(t, float), np.asarray(q, float).reshape(len(t), 2, 6)
    # ---- step pass: env pose check + dwell decides the welds; collisions do not terminate
    env = make_env(cell, assignment=queues, episode_len=len(t) + 10, terminate_on_collision=False)
    env.reset(options={"canonical": True, "q0": q[0]})
    vdt = env.v_max * env.dt
    kinds = defaultdict(list)
    track, amax = [0.0], 0.0
    info = {}
    for k in range(len(t) - 1):
        a = (q[k + 1] - env.q) / vdt
        amax = max(amax, float(np.abs(a).max()))
        _, _, _, _, info = env.step(np.clip(a, -1, 1).ravel())
        track.append(float(np.abs(env.q - q[k + 1]).max()))
        if info["collision"]:
            for kd in env.check_collisions(env.q)["kinds"] or [info["collision_kind"]]:
                kinds[kd].append(k + 1)
    ev = [(r_, w, _r(s * env.dt, 3), _r(e * env.dt, 3)) for r_, w, s, e in env.weld_events]
    step = {"welds_done": int(info.get("welds_done", 0)), "n_welds": int(info.get("n_welds", 0)),
            "success": bool(info.get("success")), "weld_events_s": ev,
            "t_last_weld_s": max([e[3] for e in ev], default=None),
            "missing": sorted({w for qu in queues.values() for w in qu} - {e[1] for e in ev}),
            "collision_steps": {kd: len(v) for kd, v in kinds.items()},
            "max_tracking_error_rad": _r(max(track), 6), "max_abs_action": _r(amax, 4),
            "action_saturated": bool(amax > 1.0 + 1e-6)}
    # ---- exact-pose pass: every sample, umarell gate vs hull check, hull distances
    env2 = make_env(cell, assignment=queues)
    warn = env2.spheres.warning_distance
    hd = HullDistance(env2)
    n = len(t)
    clr = np.empty(n); floor_m = np.full(n, np.inf); box_m = np.full(n, np.inf)
    hull = {kd: np.zeros(n, bool) for kd in KINDS}
    pairs = defaultdict(lambda: [np.inf, 0])
    for k in range(n):
        c = env2.check_collisions(q[k])
        clr[k] = c["min_clearance"]
        if c["floor_margin"] is not None:
            floor_m[k] = float(np.min(c["floor_margin"]))
        if c["box_margin"] is not None:
            box_m[k] = float(np.min(c["box_margin"]))
        for kd, lst in (c.get("contacts") or {}).items():
            if lst:
                hull[kd][k] = True
            for b1, b2, dd in lst:
                p = pairs[(kd, b1, b2)]
                p[0] = min(p[0], dd); p[1] += 1
    gate_arm = clr <= warn
    gate_floor = floor_m < 0
    gate_box = box_m < 0
    hdist = {"self_robot1": (np.inf, None, None), "self_robot2": (np.inf, None, None), "arm_arm": (np.inf, None, None)}
    idx = sorted(set(range(0, n, max(1, hull_every))) | {n - 1})
    for k in idx:
        res = hd(q[k])
        for key, (x, pr) in res.items():
            if x < hdist[key][0]:
                hdist[key] = (x, pr, float(t[k]))
    # dense sphere sweep between samples (linear interpolation, as umarell's _sample_at_time)
    fr = np.linspace(0, 1, 6)[:-1]
    qd = (q[:-1, None] + (q[1:] - q[:-1])[:, None] * fr[None, :, None, None]).reshape(-1, 2, 6)
    qd = np.concatenate([qd, q[-1:]], 0)
    clr_d = env2.spheres.pair_clearance(qd[:, 0], qd[:, 1])
    td = np.concatenate([(t[:-1, None] + (t[1:] - t[:-1])[:, None] * fr[None]).ravel(), t[-1:]])
    hull_any = np.any(np.stack(list(hull.values())), 0)
    gate_any = gate_arm | gate_floor | gate_box
    gate_dense_ok = bool((clr_d > warn).all())
    exact = {
        "samples": n, "duration_s": _r(t[-1] - t[0], 3),
        "umarell_gate": {
            "pass": bool(not gate_any.any()) and gate_dense_ok,
            "arm_arm_violation_samples": int(gate_arm.sum()), "floor_violation_samples": int(gate_floor.sum()),
            "box_violation_samples": int(gate_box.sum()),
            "min_arm_arm_clearance_m": _r(clr.min()), "t_min_clearance_s": _r(t[int(clr.argmin())], 3),
            "dense_x5_min_arm_arm_clearance_m": _r(clr_d.min()), "dense_x5_t_min_s": _r(td[int(clr_d.argmin())], 3),
            "dense_x5_violation_samples": int((clr_d <= warn).sum()),
            "min_floor_margin_m": _r(floor_m.min()) if np.isfinite(floor_m).any() else None,
            "min_box_margin_m": _r(box_m.min()) if np.isfinite(box_m).any() else None,
            "violation_intervals_s": _intervals(t, gate_any)[:20]},
        "hull_check": {
            "pass": bool(not hull_any.any()),
            "samples_by_kind": {kd: int(v.sum()) for kd, v in hull.items()},
            "duration_s_by_kind": {kd: _r(v.sum() * DT, 2) for kd, v in hull.items()},
            "intervals_s": {kd: _intervals(t, v)[:20] for kd, v in hull.items() if v.any()},
            "contact_pairs": [{"kind": kd, "body1": a, "body2": b, "min_dist_m": _r(v[0]), "samples": v[1]}
                              for (kd, a, b), v in sorted(pairs.items(), key=lambda x: x[1][0])][:20]},
        "hull_distance_m": {key: {"min_m": _r(x, 4) if np.isfinite(x) else None,
                                  "pair": None if pr is None else [hd._name(pr[0]), hd._name(pr[1])],
                                  "t_s": _r(tt, 3), "distmax_m": hd.distmax}
                            for key, (x, pr, tt) in hdist.items()},
        "hull_distance_every_n_samples": hull_every,
    }
    return {"label": label, "step": step, "exact": exact}


# ════════════════════════════════════════════════════════════════════════════
#  export to umarell plan JSON
# ════════════════════════════════════════════════════════════════════════════
def weld_windows(t, q, weld_events, dt=DT):
    """Per weld: the sample interval in which the arm is at the weld in tolerance and the leg may end:
    the env's dwell window (the last ``dwell`` matched states up to completion) extended while the arm
    stays exactly still afterwards. -> [(robot, wid, k_first, k_last, k_complete)]"""
    out = []
    for r_, w, s, e in weld_events:
        ri = ROBOT_IDS.index(r_)
        kc = int(round(e / dt))
        kf = int(round(s / dt)) + 1        # env: t_start = completion - dwell_steps (state before the 1st match)
        kl = kc
        while kl + 1 < len(t) and np.abs(q[kl + 1, ri] - q[kc, ri]).max() < 1e-9:
            kl += 1
        out.append((r_, w, kf, kl, kc))
    return out


def min_stabbing_cuts(windows):
    """Fewest cut indices such that every window [kf, kl] contains one (greedy by right end).
    Returns {weld index -> cut index} and the sorted cut list."""
    order = sorted(range(len(windows)), key=lambda i: windows[i][3])
    cut_of, cuts = {}, []
    for i in order:
        kf, kl = windows[i][2], windows[i][3]
        hit = [c for c in cuts if kf <= c <= kl]
        if hit:
            cut_of[i] = hit[-1]
        else:
            cuts.append(kl)
            cut_of[i] = kl
    return cut_of, sorted(set(cuts))


def legs_sync_min(t, q, weld_events, *, hold_s=0.5, margin=1.0, home_last=True, dt=DT):
    """Both arms cut at the same instants (every step is a Parallel of two equal-duration legs, so the
    pairs of configurations that occur at the same instant are exactly the rollout's), but only at a
    minimal set of instants that puts the end of a leg inside every weld's in-tolerance window. A weld
    leg's ``hold_duration_s`` tops the in-trajectory time at the weld up to ``hold_s``; the executor adds
    its fixed ``completion_margin_s`` (1.0 s) at every cut with both arms standing."""
    from player.timeline import Leg
    t, q = np.asarray(t, float), np.asarray(q, float).reshape(len(t), 2, 6)
    win = weld_windows(t, q, weld_events, dt)
    cut_of, cuts = min_stabbing_cuts(win)
    cuts = [c for c in cuts if 0 < c < len(t) - 1]
    bounds = [0] + cuts + [len(t) - 1]
    weld_at = {}
    for i, c in cut_of.items():
        weld_at[(win[i][0], c)] = i
    legs, shift = [], 0.0
    for s in range(len(bounds) - 1):
        i0, i1 = bounds[s], bounds[s + 1]
        last = s == len(bounds) - 2
        ts = t[i0:i1 + 1]
        step_legs = []
        for ri, rid in enumerate(ROBOT_IDS):
            kind, op, hold = "move", f"{rid.upper()}_S{s:02d}", 0.0
            wi = weld_at.get((rid, i1))
            if wi is not None:
                kind, op = "weld", win[wi][1]
                hold = max(0.0, round(hold_s - (i1 - win[wi][2]) * dt, 4))
            if last and home_last:
                kind, op, hold = "home", "home", 0.0
            ph = np.array(["transit"] * len(ts), dtype="<U16")
            leg = Leg(robot=rid, op_id=op, kind=kind, t0=float(ts[0] + shift), t1=float(ts[-1] + shift),
                      times=ts + shift, positions=q[i0:i1 + 1, ri].copy(), phase=ph, hold_s=hold, vscale=1.0,
                      operation_type="" if kind == "home" else ("weld" if kind == "weld" else "move"),
                      node_type="MoveHome" if kind == "home" else "ExecuteTrajectory",
                      bb_key=f"traj_{rid}_n{s}" if kind == "home" else "")
            leg.t_done = leg.t1 + leg.hold_s + margin
            step_legs.append(leg)
        legs += step_legs
        shift += max(l.hold_s for l in step_legs) + margin if not last else 0.0
    # welds that could not be cut (window touching the rollout start/end) are reported by the caller
    return legs, {"n_cuts": len(cuts), "n_welds": len(win),
                  "uncut": [win[i][1] for i, c in cut_of.items() if not (0 < c < len(t) - 1)]}


# ════════════════════════════════════════════════════════════════════════════
#  dynamics check (velocity / acceleration / jerk on a dense grid)
# ════════════════════════════════════════════════════════════════════════════
def limits(cell):
    from env.cell import load_cell
    c = load_cell(cell)
    return (np.stack([c.robots[r].v_max for r in ROBOT_IDS]), np.stack([c.robots[r].a_max for r in ROBOT_IDS]))


def plan_dynamics(sample, T, v_max, a_max, *, hz=200.0, h=0.05, t0=0.0):
    """Joint velocity / acceleration / jerk of a two-arm motion ``sample(ts) -> (n, 2, 6)`` on a dense grid.

    Evaluated every 1/hz s on [t0, T] (default 200 Hz). Velocity: central difference over 1/hz (exact for the
    piecewise-linear interpolation every umarell controller / validator uses). Acceleration and jerk: central
    second / third differences with a fixed h = 0.05 s (= the plans' 20 Hz sample interval and the env's
    acceleration definition): a velocity step dv at a waypoint shows up as dv / 0.05 s, so an instant stop of a
    moving arm (what an executor pause does to an arm that is not at rest) is caught. Ratios are per joint
    against the URDF limits (jerk in a_max per second)."""
    ts = np.arange(t0, T + 1e-9, 1.0 / hz)
    d = 0.5 / hz
    q = lambda x: sample(np.clip(x, t0, T))
    v = (q(ts + d) - q(ts - d)) / (2 * d)
    a = (q(ts + h) - 2 * q(ts) + q(ts - h)) / h ** 2
    j = (q(ts + 1.5 * h) - 3 * q(ts + 0.5 * h) + 3 * q(ts - 0.5 * h) - q(ts - 1.5 * h)) / h ** 3
    rv, ra, rj = np.abs(v) / v_max, np.abs(a) / a_max, np.abs(j) / a_max
    out = {"grid_hz": hz, "fd_h_s": h, "samples": int(len(ts)), "T_s": _r(T - t0, 3)}
    for name, r in (("vel_frac_vmax", rv), ("acc_frac_amax", ra), ("jerk_amax_per_s", rj)):
        m = r.max(axis=(1, 2))
        k = int(m.argmax())
        rr, jj = np.unravel_index(int(r[k].argmax()), (2, 6))
        out[name] = {"max": _r(m.max(), 3), "t_max_s": _r(ts[k], 3), "at": f"{ROBOT_IDS[rr]} a{jj + 1}",
                     "p99": _r(np.percentile(m, 99), 3),
                     "per_joint_max": {rid: [_r(x, 3) for x in r[:, i].max(0)] for i, rid in enumerate(ROBOT_IDS)}}
    ma = ra.max(axis=(1, 2))
    out["acc_frac_samples_over"] = {str(x): _r((ma > x).mean(), 4) for x in (0.49, 0.7, 1.0)}
    return out


def timeline_dynamics(tl, cell, **kw):
    v_max, a_max = limits(cell)
    return plan_dynamics(tl.sample, float(tl.t[-1]), v_max, a_max, **kw)


def array_dynamics(t, q, cell, **kw):
    from player.timeline import timeline_from_arrays
    return timeline_dynamics(timeline_from_arrays(t, q), cell, **kw)


# ════════════════════════════════════════════════════════════════════════════
#  rest-to-rest synchronous export (sync_rest)
# ════════════════════════════════════════════════════════════════════════════
def choose_cuts(windows, speed_cost, forbid=None):
    """Fewest cut indices stabbing every weld window; among the instants that stab the most open windows,
    the one where both arms move slowest (``speed_cost[k]``: time to brake to rest) -> least slowdown.
    ``forbid`` (optional (n,) bool, default None = old behaviour): instants that are avoided whenever the window
    offers another one (an arm moving inside a weld operation; tools.btexport, P3U round 2)."""
    open_ = set(range(len(windows)))
    cut_of, cuts = {}, []
    while open_:
        i = min(open_, key=lambda i: windows[i][3])          # earliest right end must be stabbed now
        kf, kl = windows[i][2], windows[i][3]
        best = None
        for c in range(kf, kl + 1):
            cover = [j for j in open_ if windows[j][2] <= c <= windows[j][3]]
            # most windows first; then less braking (0.05 s buckets); then the latest instant (deepest in the
            # dwell: the env had the longest in-tolerance hold there)
            key = (-len(cover), int(speed_cost[c] / 0.05), -c)
            if forbid is not None:
                key = (bool(forbid[c]),) + key
            if best is None or key < best[0]:
                best = (key, c, cover)
        _, c, cover = best
        cuts.append(c)
        for j in cover:
            cut_of[j] = c
            open_.discard(j)
    return cut_of, sorted(set(cuts))


def time_warp(t, q, rest_idx, a_lim, *, sub=10, h=0.05, a_mvc=None):
    """Common-clock time scaling of a two-arm rollout so that both arms come to rest at the samples
    ``rest_idx`` (and at the start / end) with joint accelerations <= ``a_lim`` (2, 6).

    The path is kept exactly (same configurations, same pairing of the two arms in time: one clock for both),
    only its timing changes; the clock never runs faster than the rollout (u = (ds/dtau)^2 <= 1). Phase-plane
    passes (TOPP style) on a grid of ``sub`` points per rollout sample: with path derivatives q', q'' of the
    rollout (central differences, h = 0.05 s), joint accelerations are q' u'/2 + q'' u; forward / backward
    passes from every rest point bound u'. Returns (s (M,), tau (M,), q (M, 2, 6), u (M,)) on the fine grid."""
    t = np.asarray(t, float)
    q = np.asarray(q, float).reshape(len(t), 2, 6)
    n = len(t)
    ds = (t[1] - t[0]) / sub
    s = t[0] + np.arange((n - 1) * sub + 1) * ds
    Q = q.reshape(n, 12)
    interp = lambda x: np.stack([np.interp(np.clip(x, t[0], t[-1]), t, Q[:, k]) for k in range(12)], -1)
    qf = interp(s)
    d1 = (interp(s + h / 2) - interp(s - h / 2)) / h
    d2 = (interp(s + h) - 2 * qf + interp(s - h)) / h ** 2
    A = np.asarray(a_lim, float).reshape(12)
    Am = A if a_mvc is None else np.asarray(a_mvc, float).reshape(12)
    ad1, ad2 = np.abs(d1), np.abs(d2)
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.minimum(1.0, np.nanmin(np.where(ad2 > 1e-12, (Am / ad2) ** 2, np.inf), axis=1))
    rest = sorted({0, len(s) - 1} | {int(k) * sub for k in rest_idx})
    u[rest] = 0.0
    eps = 1e-9

    # The path is piecewise linear (20 Hz kinks): a kink of the rollout seen at clock speed sigma = sqrt(u)
    # shows up in the 0.05 s difference metric as |q''| sigma (not |q''| u), so the kink term is bounded with
    # sqrt(u) (conservative in its sign): |q' u'/2| + |q''| sqrt(u) <= A.
    def bound(ui, i):
        # forward: u_next <= u_i + 2 ds (A - |q''| sqrt(u_i)) / |q'|
        m = ad1[i] > 1e-9
        if not m.any():
            return 1.0
        return float(np.min(ui + 2 * ds * np.maximum(A[m] - ad2[i, m] * np.sqrt(ui), 0.0) / ad1[i, m]))

    for i in range(len(s) - 1):
        u[i + 1] = min(u[i + 1], bound(u[i], i))
    for i in range(len(s) - 1, 0, -1):
        m = ad1[i - 1] > 1e-9
        if m.any():
            # backward: u_{i-1} <= u_i + 2 ds (A - |q''| sqrt(u_{i-1})) / |q'|, sqrt(u_{i-1}) bounded from above
            up = np.minimum(1.0, u[i] + 2 * ds * A[m] / ad1[i - 1, m])
            ub = np.min(u[i] + 2 * ds * np.maximum(A[m] - ad2[i - 1, m] * np.sqrt(up), 0.0) / ad1[i - 1, m])
            u[i - 1] = min(u[i - 1], float(ub))
    su = np.sqrt(np.maximum(u, 0.0))
    dtau = 2 * ds / np.maximum(su[:-1] + su[1:], eps)
    tau = np.concatenate([[0.0], np.cumsum(dtau)])
    return s, tau, qf.reshape(-1, 2, 6), u


def legs_sync_rest(cell, t, q, weld_events, *, hold_s=0.5, margin=1.0, home_last=True, acc_frac=0.49,
                   acc_mode="envelope", sub=40, dt=DT, max_iter=4, warp_fn=None, forbid=None):
    """Synchronous export in which every executor pause happens with BOTH arms at rest.

    1. cut instants: fewest instants that stab every weld's in-tolerance window (each weld leg still ends on its
       weld), preferring instants where the arms move slowly (``choose_cuts``);
    2. common-clock time warp (``time_warp``): both arms brake to rest at every cut and at the start / end, and
       re-accelerate after it, within the acceleration limit ``A`` = acc_frac * a_max (``acc_mode="strict"``) or
       max(acc_frac * a_max, the rollout's own peak per joint) (``"envelope"``, default: the export adds no
       acceleration beyond what the rollout itself does, and stays <= acc_frac * a_max wherever the rollout does);
       re-run with a smaller A until the dense check (``plan_dynamics``) of the warped motion meets the target;
    3. legs: both arms cut at the same instants; weld legs hold ``hold_s`` (a real stationary hold, the arms
       are at rest), then the executor's 1.0 s margin.
    Configuration pairs at equal clock times are exactly the rollout's (one clock for both arms), so the
    arm-arm clearance along the path is unchanged; only the timing stretches near the cuts.

    ``warp_fn`` (default None = ``time_warp``): a replacement with the same signature and return value
    ``(s, tau, qf, u)`` on a grid of ``sub`` points per rollout sample (tools.btexport's full-speed profile, which
    lets the common clock run faster than the rollout outside the weld operations).
    ``forbid`` (default None): (n,) bool, instants avoided as cuts where the weld window allows (choose_cuts);
    ``info["cut_in_weld_zone"]`` lists the welds whose cut had to be a forbidden instant."""
    from player.timeline import Leg
    t, q = np.asarray(t, float), np.asarray(q, float).reshape(len(t), 2, 6)
    v_max, a_max = limits(cell)
    win = weld_windows(t, q, weld_events, dt)
    vel = np.abs(np.gradient(q, t, axis=0))                               # (n, 2, 6)
    roll = array_dynamics(t, q, cell)
    peak = np.array([roll["acc_frac_amax"]["per_joint_max"][r] for r in ROBOT_IDS], float)   # (2, 6) / a_max
    target = np.full((2, 6), acc_frac) if acc_mode == "strict" else np.maximum(acc_frac, peak)
    cost = (vel / (target * a_max)).max(axis=(1, 2))                     # braking time ~ v / a
    cut_of, cuts = choose_cuts(win, cost, forbid)
    forced = sorted({win[i][1] for i, c in cut_of.items() if forbid is not None and forbid[c]})
    cuts = [c for c in cuts if 0 < c < len(t) - 1]
    scale = 1.0
    for it in range(max_iter):
        s, tau, qf, u = (warp_fn or time_warp)(t, q, cuts, target * a_max * scale, sub=sub, a_mvc=target * a_max)
        from player.timeline import timeline_from_arrays
        dyn = plan_dynamics(timeline_from_arrays(tau, qf).sample, float(tau[-1]), v_max, a_max)
        got = np.array([dyn["acc_frac_amax"]["per_joint_max"][r] for r in ROBOT_IDS], float)
        over = float(np.max(got / target))
        if over <= 1.05:     # discretisation tolerance of the 0.05 s difference metric
            break
        if it < max_iter - 1:
            scale /= over * 1.02
    # keep the rollout samples + the fine points inside warped (slowed) stretches
    # waypoints: the rollout samples, every 4th fine point where the clock is slowed, every fine point close to
    # rest (u < 0.25, the last / first few cm before / after a pause)
    ii = np.arange(len(s))
    on_sample = (ii % sub) == 0
    slowed = (np.abs(u - 1.0) > 1e-3) & (ii % 4 == 0)     # == (u < 0.999) for time_warp (u <= 1)
    near = u < 0.25
    keep = on_sample | slowed | near | np.r_[near[1:], False] | np.r_[False, near[:-1]]
    kidx = np.flatnonzero(keep)
    cut_f = [c * sub for c in cuts]
    bounds = [0] + cut_f + [len(s) - 1]
    weld_at = {(win[i][0], c * sub): i for i, c in cut_of.items()}
    legs, shift = [], 0.0
    for k in range(len(bounds) - 1):
        f0, f1 = bounds[k], bounds[k + 1]
        idx = kidx[(kidx >= f0) & (kidx <= f1)]
        ts = tau[idx]
        last = k == len(bounds) - 2
        step = []
        for ri, rid in enumerate(ROBOT_IDS):
            kind, op, hold = "move", f"{rid.upper()}_S{k:02d}", 0.0
            wi = weld_at.get((rid, f1))
            if wi is not None:
                kind, op, hold = "weld", win[wi][1], hold_s
            if last and home_last:
                kind, op, hold = "home", "home", 0.0
            leg = Leg(robot=rid, op_id=op, kind=kind, t0=float(ts[0] + shift), t1=float(ts[-1] + shift),
                      times=ts + shift, positions=qf[idx, ri].copy(), phase=np.array(["transit"] * len(ts), dtype="<U16"),
                      hold_s=hold, vscale=1.0, operation_type="" if kind == "home" else ("weld" if kind == "weld" else "move"),
                      node_type="MoveHome" if kind == "home" else "ExecuteTrajectory",
                      bb_key=f"traj_{rid}_n{k}" if kind == "home" else "")
            leg.t_done = leg.t1 + leg.hold_s + margin
            step.append(leg)
        legs += step
        shift += max(l.hold_s for l in step) + margin if not last else 0.0
    info = {"n_cuts": len(cuts), "n_welds": len(win), "uncut": [win[i][1] for i, c in cut_of.items()
                                                                 if not (0 < c < len(t) - 1)],
            "acc_mode": acc_mode, "acc_frac": acc_frac, "acc_target_frac_per_joint":
                {r: [_r(x, 3) for x in target[i]] for i, r in enumerate(ROBOT_IDS)},
            "rollout_acc_peak_frac": _r(peak.max(), 3), "warp_scale": _r(scale, 4), "warp_iterations": it + 1,
            "warped_motion_s": _r(tau[-1], 3), "rollout_motion_s": _r(t[-1] - t[0], 3),
            "slowdown_s": _r(tau[-1] - (t[-1] - t[0]), 3), "waypoints_per_arm": int(len(kidx)),
            "cut_speed_cost_s": [_r(cost[c], 3) for c in cuts]}
    if forbid is not None:
        info["cut_in_weld_zone"] = forced
    return legs, info


def export_plan(cell, t, q, weld_events, out_path, *, mode="sync_rest", at_home=True, tree_id=None, meta=None,
                acc_frac=0.49, acc_mode="envelope"):
    from player.bt_plan import bt_plan_from_legs, legs_from_rollout, plan_timing_check, timeline_from_bt_plan
    if mode == "sync_rest":
        legs, info = legs_sync_rest(cell, t, q, weld_events, home_last=at_home, acc_frac=acc_frac, acc_mode=acc_mode)
    elif mode == "sync_all":
        cuts = sorted({e[3] for e in weld_events})
        arrivals = [(r_, w, s) for r_, w, s, e in weld_events]
        legs = legs_from_rollout(t, q, cuts, weld_arrivals=arrivals, hold_s=0.5, home_last=at_home)
        info = {"n_cuts": len(cuts), "n_welds": len(weld_events)}
    else:
        legs, info = legs_sync_min(t, q, weld_events, home_last=at_home)
    plan = bt_plan_from_legs(cell, legs, out_path, tree_id=tree_id,
                             metadata_extra=dict(meta or {}, export_mode=mode))
    chk = plan_timing_check(legs, plan)
    tl = timeline_from_bt_plan(plan)
    n_pause = info["n_cuts"] + 1
    info.update(mode=mode, plan=rel(out_path), n_steps=plan["behaviour_tree"]["metadata"]["steps"],
                timing_check_max_start_error_s=_r(chk["max_start_error_s"], 6), timing_missing=chk["missing"],
                makespan_motion_s=tl.meta["makespan_motion_s"], executor_end_s=tl.meta["executor_end_s"],
                umarell_schedule_makespan_s=tl.meta["umarell_schedule_makespan_s"],
                executor_overhead_s=_r(tl.meta["executor_end_s"] - float(t[-1]), 3),
                pauses=n_pause)
    info["dynamics_exported"] = timeline_dynamics(tl, cell)
    info["weld_legs"] = weld_leg_stats(cell, legs)
    info["pauses_at_rest"] = pause_speeds(tl, cell)
    return plan, tl, info


def pause_speeds(tl, cell):
    """Speed of each arm (fraction of v_max, over the last / first 5 ms) when the executor pauses between two
    steps. umarell's own plans pause with both arms at rest; an arm frozen mid-motion shows up here."""
    v_max, _ = limits(cell)
    ends = sorted({round(l.t1, 6) for l in tl.legs if l.t_done > l.t1 + 1e-6})
    rows = []
    for te in ends[:-1] if ends else []:
        nxt = min([l.t0 for l in tl.legs if l.t0 > te + 1e-6], default=None)
        if nxt is None:
            continue
        w = 0.005
        before = np.abs(tl.sample(np.array([te]))[0] - tl.sample(np.array([te - w]))[0]) / w / v_max
        after = np.abs(tl.sample(np.array([nxt + w]))[0] - tl.sample(np.array([nxt]))[0]) / w / v_max
        rows.append((te, float(before.max()), float(after.max())))
    mx = max([max(b, c) for _, b, c in rows], default=0.0)
    return {"n_pauses": len(rows), "max_speed_frac_vmax": _r(mx, 4),
            "n_pauses_with_arm_moving_gt_0.03": sum(max(b, c) > 0.03 for _, b, c in rows),
            "worst": [(_r(t, 3), _r(b, 4), _r(c, 4)) for t, b, c in sorted(rows, key=lambda x: -max(x[1], x[2]))[:3]]}


def weld_leg_stats(cell, legs):
    """TCP travel of every exported weld leg. umarell's validate_approach takes the entry direction from the
    last 0.04 m of the leg's own waypoints; a weld leg shorter than that (the previous cut fell close to the
    weld) makes the entry check see only part of the approach, so short legs are flagged."""
    from env.cell import load_cell
    from env.kinematics import ArmKinematics
    c = load_cell(cell)
    kin = {r: ArmKinematics(c.robots[r]) for r in ROBOT_IDS}
    rows = []
    for l in legs:
        if l.kind != "weld":
            continue
        p = kin[l.robot].tcp(l.positions)[0]
        rows.append((l.robot, l.op_id, float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())))
    tr = [r[2] for r in rows]
    return {"min_tcp_travel_m": _r(min(tr), 4) if tr else None,
            "short_legs_lt_0.04m": [(r, w, _r(x, 4)) for r, w, x in rows if x < 0.04]}


# ════════════════════════════════════════════════════════════════════════════
#  umarell validators (umarell venv, subprocess)
# ════════════════════════════════════════════════════════════════════════════
def umarell_validate(plan_file, cell, out_json, *, mesh_audit=True, timeout=3600):
    if not UMARELL_PY.exists():
        return {"skipped": "no .venv-umarell"}
    cmd = [str(UMARELL_PY), str(ROOT / "scripts" / "umarell_validate.py"), str(plan_file), "--cell", cell,
           "--out", str(out_json)] + (["--mesh-audit"] if mesh_audit else [])
    t0 = time.time()
    p = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True, timeout=timeout)
    if p.returncode != 0 or not Path(out_json).exists():
        return {"error": p.stderr[-3000:], "returncode": p.returncode}
    return umarell_report_summary(out_json, wall_s=time.time() - t0)


def umarell_report_summary(out_json, wall_s=None) -> dict:
    """The ``umarell_validation`` block of an exec JSON from a scripts/umarell_validate.py report (also used to
    re-summarise after ``umarell_validate.py --update``, scripts/r3_validate_batch.py resummarize)."""
    rep = json.load(open(out_json))
    v = dict(rep["verdict"])
    ma = rep.get("mesh_audit") or {}
    g = ma.get("grid_20hz") or {}
    summary = {
        "report": rel(out_json), "wall_s": _r(wall_s, 1), "verdict": v,
        "validate_trajectory_set_per_step_free": v.get("per_step_collision_free"),
        "per_step_min_clearance_m": (rep.get("per_step_summary") or {}).get("min_clearance_m"),
        "inter_robot_clearance_full_timeline": {k: rep["full_timeline"].get(k) for k in
                                                ("collision_free", "min_clearance_m", "samples_checked", "horizon_s")},
        "full_eval_25hz_min_m": rep["full_eval_25hz"]["min_clearance_m"],
        "dense_min_m": rep["dense"]["min_clearance_m"],
        "validate_approach": {"n_matches": rep["approach"]["n_matches"], "task_welds": rep["approach"]["task_welds"],
                              "missing": rep["approach"]["missing"], "duplicates": rep["approach"]["duplicates"],
                              "n_strict_geodesic_ok": rep["approach"]["n_strict_ok"],
                              "worst_pos_mm": max([w.get("position_error_mm") or 0 for w in rep["approach"]["welds"]],
                                                  default=None),
                              "worst_tool_axis_deg": max([w.get("tool_axis_error_deg") or 0
                                                          for w in rep["approach"]["welds"]], default=None),
                              "worst_entry_deg": max([w.get("approach_direction_error_deg") or 0
                                                      for w in rep["approach"]["welds"]], default=None)},
        "obstacles_free": rep["obstacles"]["all_free"],
        "structure": {r: {k: rep["structure"][r][k] for k in ("max_start_jump_rad", "joint_limit_violations",
                                                               "max_joint_speed_fraction_of_urdf", "ends_at_home_rad")}
                      for r in ROBOT_IDS},
    }
    if g:
        summary["mesh_audit"] = {
            "self_collision_free": v.get("mesh_self_collision_free"),
            "min_self_dist_m": {r: g[r]["min_self_dist_m"] for r in ROBOT_IDS},
            "penetrating_time_s": {r: g[r]["penetrating_time_s"] for r in ROBOT_IDS},
            "arm_arm_mesh_min_dist_m": g.get("arm_arm_mesh_min_dist_m"),
            "floor_min_z_m": min(g[r]["floor_min_z_m"] for r in ROBOT_IDS)}
    # validator v2 (docs/r3_validate.md): per_step_collision_free is on the executed timeline (Delay nodes applied);
    # joint limits are our check (umarell HEAD has none), so they gate all_pass_incl_mesh, not umarell_validators_pass.
    # A v1 report has no joint_limits section: joint_limits_ok None counts as "not checked", not as a failure.
    jl = rep.get("joint_limits")
    summary["joint_limits"] = None if jl is None else {
        "ok": jl.get("ok"), "max_excess_deg": jl.get("max_excess_deg"), "a5_max_excess_deg": jl.get("a5_max_excess_deg"),
        "where": jl.get("where"), "min_margin_rad": {r: (d or {}).get("min_margin_rad") for r, d in (jl.get("robots") or {}).items()}}
    summary["joint_limits_ok"] = v.get("joint_limits_ok")
    gate = bool(v.get("arm_arm_collision_free_full_timeline")) and bool(v.get("per_step_collision_free")) \
        and bool(v.get("obstacles_free")) and v.get("welds_validate_approach", "").split("/")[0] == \
        str(rep["approach"]["task_welds"])
    summary["umarell_validators_pass"] = gate
    summary["all_pass_incl_mesh"] = gate and bool(v.get("mesh_self_collision_free")) and v.get("joint_limits_ok") is not False
    return summary


# ════════════════════════════════════════════════════════════════════════════
#  variants
# ════════════════════════════════════════════════════════════════════════════
def run_variants(kind, arg, cell, env_kw, n, seed_base, safety, max_steps=2000):
    env = make_env(cell, **env_kw)
    ctrl = build_controller(kind, arg, env)
    sf = SafetyFilter(env, enabled=safety)
    rows = []
    t0 = time.time()
    for i in range(n):
        s = seed_base + i
        rec = rollout(env, ctrl, seed=s, reset_options=None, max_steps=max_steps, sfilter=sf if safety else None,
                      home_tail="off", record=False)
        st = rec["controller_stats"] or {}
        rows.append({"seed": s, "success": rec["success"], "welds_done": rec["welds_done"], "n_welds": rec["n_welds"],
                     "collision": rec["collision"], "collision_kind": (rec["first_collision"] or {}).get("kind"),
                     "truncated": rec["truncated"], "time_s": _r(rec["t_end_s"], 3),
                     "min_clearance_m": _r(rec["clearance"].min()),
                     "plan_fail": bool(st.get("plan_fail")) if "plan_fail" in st else None})
        if (i + 1) % 10 == 0:
            print(f"  variants {i + 1}/{n} ({time.time() - t0:.0f} s)", flush=True)
    m = lambda key: float(np.mean([r[key] for r in rows])) if rows else None
    out = {"n": n, "seed_base": seed_base, "randomisation": "env defaults (EnvConfig): n_welds 1-15, pos_noise "
           "+-3 cm, shuffle_prob 0.5, start_noise 0.1 rad; env.reset(seed=seed_base+i)",
           "success_rate": m("success"), "collision_rate": m("collision"),
           "weld_completion": sum(r["welds_done"] for r in rows) / max(1, sum(r["n_welds"] for r in rows)),
           "mean_welds_done": m("welds_done"), "truncation_rate": m("truncated"),
           "collision_rate_by_kind": {kd: float(np.mean([r["collision_kind"] == kd for r in rows])) for kd in KINDS},
           "mean_time_s_success": float(np.mean([r["time_s"] for r in rows if r["success"]])) if any(
               r["success"] for r in rows) else None,
           "min_clearance_m": min(r["min_clearance_m"] for r in rows) if rows else None,
           "wall_s": _r(time.time() - t0, 1), "safety_filter": sf.summary() if safety else None, "rows": rows}
    pf = [r for r in rows if r["plan_fail"] is not None]
    if pf:
        feas = [r for r in pf if not r["plan_fail"]]
        out["n_infeasible_plan_fail"] = len(pf) - len(feas)
        out["success_rate_feasible"] = float(np.mean([r["success"] for r in feas])) if feas else None
    return out


# ════════════════════════════════════════════════════════════════════════════
#  main
# ════════════════════════════════════════════════════════════════════════════
def default_name(kind, arg, cell, safety):
    if kind == "expert":
        tag = f"expert_{arg or 'expert'}"
    elif kind == "random":
        tag = f"random_{arg or 0}"
    elif kind == "timeline":
        tag = f"timeline_{Path(arg).stem}"
    elif kind in ("hier", "hier_la"):
        p = Path(arg.split(",")[0])
        tag = f"{kind}_{p.parent.name}_{p.stem}"
    elif kind == "umarell":
        tag = f"umarell_{Path(arg).stem.replace('_welding_cell_2_small', '')}"
    else:
        p = Path(arg)
        tag = f"{kind}_{p.parent.name}_{p.stem}" if p.parent.name else f"{kind}_{p.stem}"
    return f"{tag}_{CELL_SHORT.get(cell, cell)}_{'filtered' if safety else 'raw'}"


def save_timeline(path, t, q, weld_events, meta):
    from player.timeline import timeline_from_arrays
    tl = timeline_from_arrays(t, q, weld_events=[tuple(e) for e in weld_events], meta=meta)
    tl.save(path)
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--controller", required=True, help="expert[:assignment] | umarell:<plan> | hier:<ckpt> | "
                                                         "residual:<ckpt> | e2e:<ckpt>")
    ap.add_argument("--cell", default="welding_cell_2_small")
    ap.add_argument("--safety-filter", choices=["off", "on"], default="off")
    ap.add_argument("--filter-margin", type=float, default=0.005, help="extra arm-arm clearance margin (m)")
    ap.add_argument("--on-collision", choices=["stop", "continue"], default=None,
                    help="default: stop (policies, expert), continue (umarell replays)")
    ap.add_argument("--home-tail", choices=["expert", "controller", "off"], default="expert")
    ap.add_argument("--max-steps", type=int, default=3000)
    ap.add_argument("--home-tol", type=float, default=1e-4, help="max |q - HOME| (rad) that counts as at HOME "
                    "(tail stop, MoveHome export); PPO v4 homes itself to its env's home_tol (0.01)")
    ap.add_argument("--export-mode", choices=["sync_rest", "sync_min", "sync_all"], default="sync_rest",
                    help="sync_rest (default): every executor pause with both arms at rest (common-clock time warp); "
                         "sync_min / sync_all: legacy, freeze moving arms at the pauses (acceleration spikes)")
    ap.add_argument("--export-acc-frac", type=float, default=0.49, help="sync_rest acceleration limit (x a_max); "
                    "0.49 = umarell's vscale^2 rule at velocity scale 0.7")
    ap.add_argument("--export-acc-mode", choices=["envelope", "strict"], default="envelope",
                    help="envelope: limit = max(frac, rollout's own per-joint peak); strict: frac everywhere "
                         "(re-times the whole rollout if it is faster)")
    ap.add_argument("--skip-validate", action="store_true")
    ap.add_argument("--no-mesh-audit", action="store_true")
    ap.add_argument("--skip-canonical", action="store_true", help="variants only")
    ap.add_argument("--variants", type=int, default=0)
    ap.add_argument("--seed-base", type=int, default=50000)
    ap.add_argument("--variant-max-steps", type=int, default=2000)
    ap.add_argument("--name", default=None)
    ap.add_argument("--out-dir", default=str(ROOT / "results"))
    ap.add_argument("--data-dir", default=str(ROOT / "data" / "exec"))
    ap.add_argument("--hull-every", type=int, default=2, help="hull-distance sample stride in the replay")
    from robotmodel.override import add_cli_args, apply_cli_args
    add_cli_args(ap)      # --urdf / --robot-base / --robot-accel / --robot-override (docs/p3_urdf.md), default off
    a = ap.parse_args(argv)
    robot_ov = apply_cli_args(a)
    if robot_ov is None:
        from robotmodel.override import active_override
        robot_ov = active_override()      # KWR_ROBOT_OVERRIDE from the environment

    kind, arg = parse_controller(a.controller)
    safety = a.safety_filter == "on"
    name = a.name or default_name(kind, arg, a.cell, safety)
    if robot_ov is not None and not a.name:   # never overwrite the results of the cell's own robots
        import hashlib
        from robotmodel.override import dumps
        name += "_robot" + hashlib.sha1(dumps(robot_ov).encode()).hexdigest()[:8]
    out_dir, data_dir = Path(a.out_dir), Path(a.data_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    data_dir.mkdir(parents=True, exist_ok=True)
    on_coll = a.on_collision or ("continue" if kind == "umarell" else "stop")
    env_kw = controller_env_kwargs(kind, arg)
    env_kw.pop("cell_id", None)
    res = {"name": name, "controller": a.controller, "kind": kind, "cell": a.cell, "safety_filter": a.safety_filter,
           "on_collision": on_coll, "home_tail": a.home_tail, "export_mode": a.export_mode,
           "env_overrides": json.loads(json.dumps(env_kw, default=_jsonable)),
           "created": time.strftime("%Y-%m-%d %H:%M:%S"), "argv": sys.argv[1:] if argv is None else argv}
    if robot_ov is not None:
        from env.cell import load_cell
        c_ov = load_cell(a.cell)
        res["robot_override"] = robot_ov
        res["robot_limits"] = {r: {"urdf": rel(c_ov.robots[r].urdf), "q_lo": c_ov.robots[r].q_lo.tolist(),
                                   "q_hi": c_ov.robots[r].q_hi.tolist(), "v_max": c_ov.robots[r].v_max.tolist(),
                                   "a_max": c_ov.robots[r].a_max.tolist(), "accel_source": c_ov.robots[r].accel_source,
                                   "base": c_ov.robots[r].base.tolist(), "yaw": c_ov.robots[r].yaw,
                                   "limits_source": c_ov.robots[r].limits_source} for r in ROBOT_IDS}
        print(f"[execute] robot override active: {json.dumps(robot_ov)}", flush=True)
        if not a.skip_validate:
            a.skip_validate = True     # umarell's validators load the cell's own URDF / bases from its repo
            res["umarell_validation_skipped"] = ("robot override: umarell's validators would judge the plan with the "
                                                 "cell's original URDF and bases")
    print(f"[execute] {name}: controller {a.controller} cell {a.cell} filter {a.safety_filter}", flush=True)

    if not a.skip_canonical:
        env = make_env(a.cell, **env_kw)
        ctrl = build_controller(kind, arg, env)
        sf = SafetyFilter(env, margin=a.filter_margin, enabled=safety) if safety else None
        opts = {"canonical": True}
        plan_src = None
        if kind == "umarell":
            tl_u = ctrl.tl
            plan_src = umarell_plan_file(arg)
            env.cfg.assignment = queues_from_timeline(tl_u)
            opts["q0"] = tl_u.q[0]
            res["umarell_plan"] = {"file": rel(plan_src),
                                   "makespan_motion_s": tl_u.meta["makespan_motion_s"],
                                   "executor_end_s": tl_u.meta["executor_end_s"],
                                   "umarell_schedule_makespan_s": tl_u.meta["umarell_schedule_makespan_s"],
                                   "n_legs": tl_u.meta["n_legs"]}
        if isinstance(ctrl, TimelineController) and kind != "umarell":
            env.cfg.assignment = ctrl.queues or queues_from_timeline(ctrl.tl)
            opts["q0"] = ctrl.tl.q[0]
        n_max = len(ctrl.tl.q) + 10 if isinstance(ctrl, TimelineController) else a.max_steps
        rec = rollout(env, ctrl, reset_options=opts, max_steps=n_max, sfilter=sf, on_collision=on_coll,
                      home_tail=a.home_tail, home_tol=a.home_tol)
        print(f"[rollout] welds {rec['welds_done']}/{rec['n_welds']} success {rec['success']} collisions "
              f"{rec['collision_steps']} t_last_weld {rec['t_last_weld_s']} t_end {rec['t_end_s']:.2f} "
              f"({rec['wall_s']:.0f} s wall)", flush=True)
        tl_path = data_dir / f"{name}_timeline.npz"
        meta = {"controller": a.controller, "cell": a.cell, "safety_filter": a.safety_filter, "source": "execute.py",
                "t_last_weld_s": rec["t_last_weld_s"], "clock": "env (20 Hz control steps)"}
        save_timeline(tl_path, rec["t"], rec["q"], rec["weld_events"], meta)
        np.savez_compressed(data_dir / f"{name}_actions.npz", t=rec["t"], a_raw=rec["a_raw"], a_exec=rec["a_exec"],
                            filtered=rec["filtered"], clearance=rec["clearance"])
        res["rollout"] = {k: rec[k] for k in ("welds_done", "n_welds", "success", "success_env", "collision",
                                              "collision_steps", "first_collision", "terminated", "truncated",
                                              "t_last_weld_s", "t_end_s", "home_tail", "queues", "n_main_steps",
                                              "controller_stats")}
        res["rollout"]["weld_events_s"] = [(r_, w, _r(s, 3), _r(e, 3)) for r_, w, s, e in rec["weld_events"]]
        res["rollout"]["min_clearance_m"] = _r(rec["clearance"].min())
        res["rollout"]["wall_s"] = _r(rec["wall_s"], 1)
        res["rollout"]["timeline"] = rel(tl_path)
        res["safety_filter_stats"] = sf.summary() if sf else {"enabled": False}
        res["dynamics_rollout"] = array_dynamics(rec["t"], rec["q"], a.cell)
        if kind == "umarell":   # the env rollout of a plan; the plan itself is checked below (export)
            res["dynamics_rollout"]["note"] = "env replay of the plan (halts at colliding sub-steps)"
        # replay in a fresh env
        t0 = time.time()
        res["replay"] = replay_check(a.cell, rec["t"], rec["q"], rec["queues"], hull_every=a.hull_every)
        print(f"[replay] welds {res['replay']['step']['welds_done']}/{res['replay']['step']['n_welds']} "
              f"step collisions {res['replay']['step']['collision_steps']} gate pass "
              f"{res['replay']['exact']['umarell_gate']['pass']} hull pass {res['replay']['exact']['hull_check']['pass']} "
              f"({time.time() - t0:.0f} s)", flush=True)
        # export
        if kind == "umarell":
            from player.bt_plan import load_plan
            plan_file = plan_src
            tl_x = tl_u
            res["export"] = {"mode": "original umarell plan (not re-exported)", "plan": res["umarell_plan"]["file"],
                             "makespan_motion_s": tl_u.meta["makespan_motion_s"],
                             "executor_end_s": tl_u.meta["executor_end_s"],
                             "n_steps": len(load_plan(str(plan_src))["behaviour_tree"]["root"]["children"]),
                             "dynamics_exported": timeline_dynamics(tl_u, a.cell),
                             "pauses_at_rest": pause_speeds(tl_u, a.cell)}
        elif rec["welds_done"] > 0:
            plan_file = data_dir / f"{name}_plan.json"
            _, tl_x, info = export_plan(a.cell, rec["t"], rec["q"], rec["weld_events"], plan_file, mode=a.export_mode,
                                        at_home=rec["home_tail"]["at_home"], tree_id=f"exec_{name}",
                                        meta={"source": f"kuka_weld_rl execute.py {a.controller}"},
                                        acc_frac=a.export_acc_frac, acc_mode=a.export_acc_mode)
            res["export"] = info
            dx = info["dynamics_exported"]
            print(f"[export] {info['mode']}: {info['n_cuts']} cuts, motion {info['makespan_motion_s']} s, executor end "
                  f"{info['executor_end_s']} s (overhead {info['executor_overhead_s']} s), timing err "
                  f"{info['timing_check_max_start_error_s']}; peak v {dx['vel_frac_vmax']['max']} v_max, "
                  f"a {dx['acc_frac_amax']['max']} a_max, jerk {dx['jerk_amax_per_s']['max']} a_max/s", flush=True)
        else:
            plan_file, tl_x = None, None
            res["export"] = {"skipped": "no welds done"}
        if tl_x is not None:
            t0 = time.time()
            qx = queues_from_timeline(tl_x)
            res["replay_exported"] = replay_check(a.cell, tl_x.t, tl_x.q, qx, hull_every=a.hull_every,
                                                  label="exported plan, executor clock")
            print(f"[replay exported] welds {res['replay_exported']['step']['welds_done']}/"
                  f"{res['replay_exported']['step']['n_welds']} gate {res['replay_exported']['exact']['umarell_gate']['pass']}"
                  f" hull {res['replay_exported']['exact']['hull_check']['pass']} ({time.time() - t0:.0f} s)", flush=True)
            if not a.skip_validate:
                t0 = time.time()
                res["umarell_validation"] = umarell_validate(plan_file, a.cell, data_dir / f"{name}_umarell_report.json",
                                                             mesh_audit=not a.no_mesh_audit)
                uv = res["umarell_validation"]
                print(f"[validate] {uv.get('verdict', uv)} ({time.time() - t0:.0f} s)", flush=True)
        res["summary"] = summarize(res)
    if a.variants:
        print(f"[variants] {a.variants} from seed {a.seed_base}", flush=True)
        res["variants"] = run_variants(kind, arg, a.cell, env_kw, a.variants, a.seed_base, safety,
                                       a.variant_max_steps) if kind not in ("umarell", "timeline", "hier", "hier_la") else {
            "skipped": "canonical timeline only (umarell plans exist only for the canonical task; hier variants: "
                       "evaluate_hier.py)"}
    out = out_dir / f"exec_{name}.json"
    if a.skip_canonical and out.exists():   # keep the canonical part of an earlier run
        old = json.load(open(out))
        old.update({k: v for k, v in res.items() if k in ("variants", "created", "argv")})
        res = old
    elif "variants" not in res and out.exists():   # keep the variants of an earlier run (not re-run now)
        old = json.load(open(out))
        if "variants" in old:
            res["variants"] = old["variants"]
            res["variants_from_earlier_run"] = old.get("created")
    with open(out, "w") as f:
        json.dump(res, f, indent=1, default=_jsonable)
    print(f"[execute] -> {out}")
    if "summary" in res:
        print(json.dumps(res["summary"], indent=1, default=_jsonable))
    return res


def summarize(res) -> dict:
    """Flat row for docs/results.md (scripts/make_report.py)."""
    ro, rp = res.get("rollout", {}), res.get("replay", {})
    rx, ex, uv = res.get("replay_exported", {}), res.get("export", {}), res.get("umarell_validation", {})
    g = (rp.get("exact") or {}).get("umarell_gate", {})
    h = (rp.get("exact") or {}).get("hull_check", {})
    hd = (rp.get("exact") or {}).get("hull_distance_m", {})
    gx = (rx.get("exact") or {}).get("umarell_gate", {})
    hx = (rx.get("exact") or {}).get("hull_check", {})
    s = {
        "welds_rollout": f"{ro.get('welds_done')}/{ro.get('n_welds')}",
        "welds_replay": f"{(rp.get('step') or {}).get('welds_done')}/{(rp.get('step') or {}).get('n_welds')}",
        "welds_replay_exported": (f"{(rx.get('step') or {}).get('welds_done')}/{(rx.get('step') or {}).get('n_welds')}"
                                  if rx else None),
        "rollout_collision_steps": ro.get("collision_steps"),
        "t_last_weld_env_s": ro.get("t_last_weld_s"), "t_end_env_s": _r(ro.get("t_end_s"), 3),
        "home_tail_s": (ro.get("home_tail") or {}).get("duration_s"),
        "executor_motion_s": ex.get("makespan_motion_s"), "executor_end_s": ex.get("executor_end_s"),
        "export_cuts": ex.get("n_cuts"),
        "gate_pass_rollout": g.get("pass"), "gate_pass_exported": gx.get("pass"),
        "min_clearance_env_m": g.get("min_arm_arm_clearance_m"),
        "hull_pass_rollout": h.get("pass"), "hull_pass_exported": hx.get("pass"),
        "hull_samples_by_kind": h.get("samples_by_kind"),
        "min_hull_self_m": {k: (hd.get(f"self_{k}") or {}).get("min_m") for k in ROBOT_IDS},
        "min_hull_arm_arm_m": (hd.get("arm_arm") or {}).get("min_m"),
        "filter_interventions": (res.get("safety_filter_stats") or {}).get("interventions"),
    }
    if uv and "verdict" in uv:
        s.update({"umarell_validators_pass": uv["umarell_validators_pass"],
                  "umarell_min_clearance_m": uv["inter_robot_clearance_full_timeline"]["min_clearance_m"],
                  "umarell_dense_min_m": uv["dense_min_m"],
                  "validate_approach": f"{uv['validate_approach']['n_matches']}/{uv['validate_approach']['task_welds']}",
                  "per_step_free": uv["validate_trajectory_set_per_step_free"], "obstacles_free": uv["obstacles_free"],
                  "mesh_self_free": (uv.get("mesh_audit") or {}).get("self_collision_free"),
                  "mesh_min_self_m": (uv.get("mesh_audit") or {}).get("min_self_dist_m"),
                  "mesh_arm_arm_m": (uv.get("mesh_audit") or {}).get("arm_arm_mesh_min_dist_m"),
                  "joint_limits_ok": uv.get("joint_limits_ok")})
    elif uv:
        s["umarell_validation"] = uv
    rxs = rx.get("step") or {}
    s["success_strict"] = bool(ro.get("success")) and bool(g.get("pass")) and bool(h.get("pass")) and \
        (not uv or "verdict" not in uv or bool(uv.get("all_pass_incl_mesh"))) and \
        (not rx or rxs.get("welds_done") == rxs.get("n_welds"))
    return s


if __name__ == "__main__":
    main()
