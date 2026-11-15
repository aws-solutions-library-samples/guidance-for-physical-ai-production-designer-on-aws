"""PPO v4 actor-critic: rl.ppo.model.ActorCritic plus a separate head for arms WITHOUT a goal weld (homing / idle).

Why: v4 drives goal-less arms in joint space (rl.ppo.rollout.idle_joint_actions) so that they can return to HOME (the
twist map cannot unwind the wrist at HOME's singularity). v3's mean output for such arms was a TCP twist; read as a
joint velocity it makes the arm drift into the floor or itself right after its last weld. In the first v4 attempt
(runs ppo_v4_{a,b,2t_a,2t_b}, killed after ~25 iterations) PPO answered that within 20 iterations by no longer
completing welds (stage-0 weld fraction 0.02). Here the goal-less arms get their own output head `mu_idle`, zero-
initialised (tanh(0) = 0: the arm stands still), and their own exploration std `log_std_idle`; everything else
(encoders, the goal head, the critic) is v3's, loaded unchanged with --init-from. Homing is then learned from the
home potentials alone (no demonstrations, no scripted homing).

The goal mask is the feature row's `active` slot (= obs robot_goal_mask), so the policy stays a function of the
observation. Log-probs and entropy use the per-robot std (goal head: log_std, idle head: log_std_idle).
"""
from __future__ import annotations

import math

import torch
import torch.nn as nn

from models.gnn_policy import mlp

from .model import N_ACT, ActorCritic


class ActorCriticV4(ActorCritic):
    def __init__(self, spec, hidden=64, layers=2, heads=4, log_std_init=-1.2, log_std_min=-4.0, control_idle=True,
                 idle_log_std_init=-1.6, idle_gain=1.0, **_):
        super().__init__(spec, hidden=hidden, layers=layers, heads=heads, log_std_init=log_std_init,
                         log_std_min=log_std_min, control_idle=True)
        self.mu_idle = mlp(hidden, hidden, N_ACT)
        with torch.no_grad():
            self.mu_idle[-1].weight.zero_()
            self.mu_idle[-1].bias.zero_()
        self.log_std_idle = nn.Parameter(torch.full((N_ACT,), float(idle_log_std_init)))
        # idle_gain k: mean = tanh(k * mu_idle(h)). The head starts at 0 and Adam moves weights by ~lr per step, so with
        # k = 1 the homing speed grows ~20x slower than needed (run ppo_v4_a2: |W| 0.08 after 75 iterations); k = 10
        # is the same as a 10x learning rate on this head's output (the adaptive LR is shared by all parameters)
        self.idle_gain = float(idle_gain)
        self.cfg.update(idle_head=True, idle_log_std_init=float(idle_log_std_init), idle_gain=self.idle_gain)

    def _heads(self, flat):
        """Goal-less arms (homing / idle) use the idle head, arms with a goal v3's head."""
        g, goal = self.graph(flat)
        h = self.actor_enc(g)["robot"]                         # (B, R, H)
        idle = (goal < 0.5).unsqueeze(-1)                       # (B, R, 1)
        mean = torch.where(idle, torch.tanh(self.idle_gain * self.mu_idle(h)), torch.tanh(self.mu(h)))
        return mean, torch.ones_like(goal), idle

    def actor(self, flat):
        mean, act, _ = self._heads(flat)
        return mean, act

    def _ls(self, idle):
        return torch.where(idle, self.log_std_idle.clamp(min=self.log_std_min), self.log_std.clamp(min=self.log_std_min))

    def _log_prob(self, mean, u, act, idle):
        ls = self._ls(idle)
        lp = -0.5 * ((u - mean) / ls.exp()) ** 2 - ls - 0.5 * math.log(2 * math.pi)
        return (lp.sum(-1) * act).sum(-1)

    def evaluate(self, flat, u):
        mean, act, idle = self._heads(flat)
        ls = self._ls(idle)
        ent = ((ls + 0.5 * math.log(2 * math.pi * math.e)).sum(-1) * act).sum(-1)
        return self._log_prob(mean, u, act, idle), ent, self.value(flat), mean, act

    @torch.no_grad()
    def act(self, flat, deterministic=False, generator=None):
        mean, act, idle = self._heads(flat)
        if deterministic:
            u = mean
        else:
            u = mean + self._ls(idle).exp() * torch.randn(mean.shape, generator=generator)
        return u, self._log_prob(mean, u, act, idle), mean
