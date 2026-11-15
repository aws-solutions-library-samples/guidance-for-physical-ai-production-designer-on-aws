"""GNN actor-critic for PPO.

Actor and critic are separate `models.gnn_policy.HeteroGNNEncoder`s (dense path) over the compact graph
of `rl.ppo.features`. The actor head maps each robot embedding to the mean of a diagonal Gaussian over
that robot's 6 action numbers (`tanh` squashed mean, state-independent learned log-std shared by both
robots). The critic pools the robot embeddings (mean) into V(s).

Observation normalisation: running mean / std of the robot-node features and the robot-robot edge
features (the task-edge features are hand-scaled to O(1) by the graph builder and contain padding).
The statistics are part of the state dict (buffers), so the workers get them with the weights.
"""
from __future__ import annotations

import math

import torch
import torch.nn as nn

from models.gnn_policy import HeteroGNNEncoder, mlp

from .features import RR, FeatureSpec, to_dense_graph, unpack

N_ACT = 6


class RunningNorm(nn.Module):
    def __init__(self, shape, clip=5.0, eps=1e-4):
        super().__init__()
        self.register_buffer("mean", torch.zeros(shape))
        self.register_buffer("var", torch.ones(shape))
        self.register_buffer("count", torch.tensor(eps, dtype=torch.float64))
        self.clip = clip

    @torch.no_grad()
    def update(self, x: torch.Tensor):
        """x (N, *shape)."""
        x = x.reshape(-1, *self.mean.shape).double()
        n = x.shape[0]
        if n == 0:
            return
        bm, bv = x.mean(0), x.var(0, unbiased=False)
        tot = self.count + n
        d = bm - self.mean.double()
        m = self.mean.double() + d * n / tot
        m2 = self.var.double() * self.count + bv * n + d * d * self.count * n / tot
        self.mean.copy_(m.float())
        self.var.copy_((m2 / tot).float())
        self.count.copy_(tot)

    def forward(self, x):
        return ((x - self.mean) / torch.sqrt(self.var + 1e-6)).clamp(-self.clip, self.clip)


class ActorCritic(nn.Module):
    def __init__(self, spec: FeatureSpec, hidden=64, layers=2, heads=4, log_std_init=-1.2, log_std_min=-4.0,
                 control_idle=False):
        super().__init__()
        self.spec = spec
        # control_idle: the policy also drives arms without a goal weld (action map idle_zero=False); otherwise
        # their action is zeroed and excluded from log-probs
        self.control_idle = bool(control_idle)
        self.cfg = dict(hidden=hidden, layers=layers, heads=heads, log_std_init=log_std_init, log_std_min=log_std_min,
                        control_idle=self.control_idle)
        Fr = spec.Fr
        self.norm_r = RunningNorm((Fr,))
        self.norm_rr = RunningNorm((spec.dims[RR],))
        self.actor_enc = HeteroGNNEncoder(Fr, spec.n_kp, hidden, layers, heads)
        self.critic_enc = HeteroGNNEncoder(Fr, spec.n_kp, hidden, layers, heads)
        if spec.sphere_k:   # wider robot-robot edges (rl.ppo.features sphere pairs)
            for enc in (self.actor_enc, self.critic_enc):
                enc.edge_enc["robot__to__robot"] = mlp(spec.dims[RR], hidden, hidden)
        self.mu = mlp(hidden, hidden, N_ACT)
        self.v = mlp(hidden, hidden, 1)
        self.log_std = nn.Parameter(torch.full((N_ACT,), float(log_std_init)))
        self.log_std_min = log_std_min
        with torch.no_grad():  # small initial policy output (RoboBallet / TD3 practice)
            self.mu[-1].weight.mul_(0.1)
            self.mu[-1].bias.zero_()
            self.v[-1].weight.mul_(0.1)

    # ---------------------------------------------------------------- graph
    def graph(self, flat):
        p = unpack(flat, self.spec)
        p["robot_x"] = self.norm_r(p["robot_x"])
        p["e_rr"] = self.norm_rr(p["e_rr"])
        return to_dense_graph(p, self.spec), p["active"]

    @torch.no_grad()
    def update_norm(self, flat):
        p = unpack(flat, self.spec)
        self.norm_r.update(p["robot_x"].reshape(-1, self.spec.Fr))
        self.norm_rr.update(p["e_rr"].reshape(-1, p["e_rr"].shape[-1]))

    def std(self):
        return self.log_std.clamp(min=self.log_std_min).exp()

    # ---------------------------------------------------------------- heads
    def actor(self, flat):
        """-> mean (B, R, 6), active (B, R)."""
        g, act = self.graph(flat)
        if self.control_idle:
            act = torch.ones_like(act)
        return torch.tanh(self.mu(self.actor_enc(g)["robot"])), act

    def value(self, flat):
        g, _ = self.graph(flat)
        return self.v(self.critic_enc(g)["robot"]).squeeze(-1).mean(-1)

    def log_prob(self, mean, u, active):
        """Diagonal Gaussian log-prob summed over the active robots' 6 dims -> (B,)."""
        ls = self.log_std.clamp(min=self.log_std_min)
        lp = -0.5 * ((u - mean) / ls.exp()) ** 2 - ls - 0.5 * math.log(2 * math.pi)
        return (lp.sum(-1) * active).sum(-1)

    def entropy(self, active):
        ls = self.log_std.clamp(min=self.log_std_min)
        return (ls + 0.5 * math.log(2 * math.pi * math.e)).sum() * active.sum(-1)

    def evaluate(self, flat, u):
        mean, act = self.actor(flat)
        return self.log_prob(mean, u, act), self.entropy(act), self.value(flat), mean, act

    @torch.no_grad()
    def act(self, flat, deterministic=False, generator=None):
        mean, act = self.actor(flat)
        if deterministic:
            u = mean
        else:
            u = mean + self.std() * torch.randn(mean.shape, generator=generator)
        u = u * act.unsqueeze(-1)
        return u, self.log_prob(mean, u, act), mean


def load_compatible(model: nn.Module, state: dict) -> list:
    """Load `state`, growing tensors whose new shape is larger (extra input features): Linear weights get the
    old columns and zeros for the new ones, normaliser means 0 / vars 1. Returns the names that were grown."""
    own = model.state_dict()
    grown = []
    for k, v in state.items():
        if k not in own:
            continue
        t = own[k]
        if t.shape == v.shape:
            t.copy_(v)
            continue
        if t.dim() == v.dim() and all(a >= b for a, b in zip(t.shape, v.shape)):
            if k.endswith("var"):
                t.fill_(1.0)
            else:
                t.zero_()
            t[tuple(slice(0, n) for n in v.shape)] = v
            grown.append(k)
        else:
            raise ValueError(f"incompatible tensor {k}: {tuple(v.shape)} -> {tuple(t.shape)}")
    return grown


def actor_state(model: ActorCritic) -> dict:
    """Parameters + buffers the rollout workers need (actor + normalisers)."""
    return {k: v for k, v in model.state_dict().items() if not k.startswith(("critic_enc.", "v."))}
