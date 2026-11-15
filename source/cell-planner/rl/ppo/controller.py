"""execute.py controller interface (docs/DESIGN.md, Track Ex) for PPO checkpoints.

    from rl.ppo.controller import load_controller, env_kwargs
    env = execute.make_env(cell, **env_kwargs(ckpt)); ctrl = load_controller(ckpt, env)

Deterministic: the action is the Gaussian policy's mean, mapped through the checkpoint's action map.
"""
from __future__ import annotations

from functools import lru_cache

import numpy as np
import torch

from rl.action_map import ActionMap, ActionMapConfig

from .features import FeatureBuilder, FeatureSpec
from .model import ActorCritic


@lru_cache(maxsize=8)
def _load(ckpt: str):
    return torch.load(ckpt, map_location="cpu", weights_only=False)


def env_kwargs(ckpt: str, train: bool = False) -> dict:
    """EnvConfig overrides for evaluation: v2 checkpoints train with tighter weld gates and are evaluated at
    umarell's gates (`eval_env_kwargs`); `train=True` returns the training env."""
    ck = _load(str(ckpt))
    # v3 (rule W): the weld is triggered at the policy's internal gates (5 mm / 3.5 deg / 10 deg, as trained), i.e.
    # the controller decides when to weld; umarell's validate_approach then judges the result at umarell's gates
    v3 = bool(ck["env_kwargs"].get("wedge"))
    ek = dict(ck["env_kwargs"] if train or v3 or "eval_env_kwargs" not in ck else ck["eval_env_kwargs"])
    ek.pop("cell_id", None)
    return ek


def build_model(ck: dict) -> ActorCritic:
    spec = FeatureSpec(**ck["spec"])
    mc = ck["model_cfg"]
    cls, kw = ActorCritic, {}
    if mc.get("idle_head"):   # PPO v4 (docs/p3_ppo4.md): separate output head for arms without a goal
        from .v4_model import ActorCriticV4 as cls
        kw = dict(idle_log_std_init=mc.get("idle_log_std_init", -1.6), idle_gain=mc.get("idle_gain", 1.0))
    m = cls(spec, hidden=mc["hidden"], layers=mc["layers"], heads=mc["heads"],
            log_std_init=mc["log_std_init"], log_std_min=mc["log_std_min"],
            control_idle=mc.get("control_idle", False), **kw)
    m.load_state_dict(ck["model"])
    m.eval()
    return m


class PPOController:
    def __init__(self, ckpt, env=None):
        ck = _load(str(ckpt))
        self.ckpt = str(ckpt)
        self.model = build_model(ck)
        cell = ck["env_kwargs"].get("cell_id", "welding_cell_2_small")
        self.fb = FeatureBuilder(self.model.spec, cell)
        self.amap = ActionMap(ActionMapConfig(**ck["action_map"]), cell)
        al = float(ck.get("args", {}).get("accel_limit", 0.0) or 0.0)
        from .accel import AccelLimit, MotionFilter
        self.accel = AccelLimit(al, cell) if al > 0 else None
        self.motion = MotionFilter(1, cell, **ck["motion"]) if ck.get("motion") else None
        if self.motion is not None and ck.get("args", {}).get("v4"):   # PPO v4 (docs/p3_ppo4.md): still hold
            from .v4_motion import MotionFilter4, motion_kwargs_from_args
            from .v4_motion import apply_vscale
            mk = motion_kwargs_from_args(ck["args"])
            self.motion = MotionFilter4(1, cell, **{**ck["motion"], **{k: mk.pop(k) for k in ("home_v", "home_a")
                                                                      if k in mk}}, **mk)
            apply_vscale(self.amap, ck["args"])
        self.idle_joint = bool(ck.get("args", {}).get("idle_joint", 0))
        self.v4_idle_linear = bool(ck.get("args", {}).get("v4") and ck["args"].get("idle_linear", 0))
        torch.set_num_threads(1)   # reproducibility (docs/verification_ppo.md I-13): fixed single-thread inference
        self.stats = dict(ckpt=self.ckpt, iter=ck.get("iter"), env_steps=ck.get("env_steps"),
                          stage=ck.get("promoter", {}).get("stage"))
        if env is not None and self.model.spec.self_dist and not env.cfg.self_dist:
            raise ValueError("PPO checkpoint needs EnvConfig.self_dist=True (use env_kwargs(ckpt))")

    def reset(self, obs):
        if self.motion is not None:
            self.motion.reset()

    @torch.no_grad()
    def act_batch(self, obs_list):
        b = {k: np.stack([o[k] for o in obs_list]) for k in obs_list[0]}
        if self.motion is not None:
            assert len(obs_list) == 1, "the v2 motion filter is stateful: one env per controller"
            b["robot_extra"] = self.motion.features(b)
        with torch.backends.mkldnn.flags(enabled=False):
            f = torch.from_numpy(self.fb(b))
            mean, act = self.model.actor(f)
        u = (mean * act.unsqueeze(-1)).numpy()
        a = self.amap.to_env(u, b)
        if self.idle_joint:
            if self.v4_idle_linear:   # PPO v4: linear joint-space map for goal-less arms (rl/ppo/v4_motion.py)
                from .v4_motion import idle_actions_v4
                a = idle_actions_v4(a, u, b, self.amap)
            else:
                from .rollout import idle_joint_actions
                a = idle_joint_actions(a, u, b, self.amap)
        if self.accel is not None:
            a = self.accel(a, b)
        if self.motion is not None:
            a = self.motion(a, b)
        return a.reshape(len(obs_list), -1)

    def act(self, obs):
        return self.act_batch([obs])[0]


def load_controller(ckpt, env=None):
    return PPOController(ckpt, env)
