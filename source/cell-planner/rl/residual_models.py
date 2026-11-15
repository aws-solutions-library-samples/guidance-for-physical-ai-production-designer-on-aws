"""GNN actor / twin critics and TD3 for the residual track (Track RR).

Same heterogeneous attention GNN as ``models.gnn_policy`` (``HeteroGNNEncoder``, robot / task /
obstacle nodes, geometry in each robot's base frame), with two changes:
  * robot nodes get the base-controller features ``res_feat`` (``rl.residual_env``) appended
    (``ResGraphBuilder``);
  * the actor outputs ACT_DIM = 7 numbers per robot (6 joint-velocity residual + 1 clock-rate
    residual), and the critics take that 7-D action.
``ResidualTD3`` reuses all of ``rl.td3.TD3`` (n-step targets, target smoothing, delayed actor,
optional BC) and only builds these networks.
"""
from __future__ import annotations

import copy
import dataclasses

import torch
import torch.nn as nn
from torch_geometric.data import HeteroData
from torch_geometric.utils import scatter

from models.gnn_policy import HeteroGNNEncoder, mlp
from models.graph_builder import DenseGraph, GraphBuilder
from rl.residual_env import ACT_DIM, RES_FEAT_DIM
from rl.td3 import TD3, TD3Config


class ResGraphBuilder(GraphBuilder):
    def __init__(self, *a, n_extra: int = RES_FEAT_DIM, **kw):
        super().__init__(*a, **kw)
        self.n_extra = n_extra
        self.robot_dim = self.robot_dim + n_extra

    def config(self) -> dict:
        d = super().config()
        d["n_extra"] = self.n_extra
        return d

    def dense(self, s: dict) -> DenseGraph:
        dg = super().dense(s)
        x = s["res_feat"] if "res_feat" in s else dg.x["robot"].new_zeros(*dg.x["robot"].shape[:2], self.n_extra)
        dg.x["robot"] = torch.cat([dg.x["robot"], x], -1)
        return dg


class ResActor(nn.Module):
    def __init__(self, robot_in, n_kp, hidden=64, layers=2, heads=4, act_dim=ACT_DIM):
        super().__init__()
        self.act_dim = act_dim
        self.encoder = HeteroGNNEncoder(robot_in, n_kp, hidden, layers, heads)
        self.head = mlp(hidden, hidden, act_dim)

    def forward(self, g):
        a = torch.tanh(self.head(self.encoder(g)["robot"]))
        if isinstance(g, DenseGraph):
            return a * g.robot_mask.unsqueeze(-1)
        out = a.new_zeros(*g.robot_mask.shape, self.act_dim)
        out[g.robot_mask] = a
        return out


class ResCritic(nn.Module):
    def __init__(self, robot_in, n_kp, hidden=64, layers=2, heads=4, act_dim=ACT_DIM):
        super().__init__()
        self.encoder = HeteroGNNEncoder(robot_in + act_dim, n_kp, hidden, layers, heads)
        self.head = mlp(hidden, hidden, 1)

    def forward(self, g, action, return_per_robot=False):
        if isinstance(g, DenseGraph):
            robot_x = torch.cat([g.x["robot"], action], -1)
            q_i = self.head(self.encoder(g, robot_x)["robot"]).squeeze(-1)
            m = g.robot_mask.float()
            q = (q_i * m).sum(1) / m.sum(1).clamp(min=1)
            return (q, q_i) if return_per_robot else q
        robot_x = torch.cat([g["robot"].x, action[g.robot_mask]], -1)
        q_i = self.head(self.encoder(g, robot_x)["robot"]).squeeze(-1)
        q = scatter(q_i, g["robot"].batch, dim=0, dim_size=g.num_graphs, reduce="mean")
        return (q, q_i) if return_per_robot else q


class ResidualTD3(TD3):
    """rl.td3.TD3 with the residual networks (7-D actions, res_feat robot features).

    ``prior_l2`` > 0: actor loss = -Q / mean|Q| + prior_l2 * mean_robots |u|^2, i.e. a pull towards the
    zero residual (= the expert) that the critic has to outweigh (TD3+BC-style Q normalisation)."""

    def __init__(self, cfg: TD3Config, graph_cfg: dict | None = None, device: str = "cpu", prior_l2: float = 0.0):
        self.prior_l2 = float(prior_l2)
        cfg = dataclasses.replace(cfg)
        self.device = torch.device(device)
        if cfg.backend == "auto":
            cfg.backend = "dense" if self.device.type == "cuda" else "pyg"
        self.cfg = cfg
        self.gb = ResGraphBuilder(**(graph_cfg or {}))
        self.graph_cfg = self.gb.config()
        nk, rin = self.gb.n_kp, self.gb.robot_dim
        self.actor = ResActor(rin, nk, cfg.hidden, cfg.layers, cfg.heads).to(self.device)
        with torch.no_grad():
            last = self.actor.head[-1]
            last.weight.mul_(cfg.actor_out_init)
            last.bias.zero_()
        self.critic1 = ResCritic(rin, nk, cfg.hidden, cfg.layers, cfg.heads).to(self.device)
        self.critic2 = ResCritic(rin, nk, cfg.hidden, cfg.layers, cfg.heads).to(self.device)
        self.actor_t = copy.deepcopy(self.actor).requires_grad_(False)
        self.critic1_t = copy.deepcopy(self.critic1).requires_grad_(False)
        self.critic2_t = copy.deepcopy(self.critic2).requires_grad_(False)
        self.critic_params = list(self.critic1.parameters()) + list(self.critic2.parameters())
        self.actor_params = list(self.actor.parameters())
        fused = self.device.type == "cuda"
        self.actor_opt = torch.optim.Adam(self.actor_params, lr=cfg.actor_lr, fused=fused)
        self.critic_opt = torch.optim.Adam(self.critic_params, lr=cfg.critic_lr, fused=fused)
        self.online_params = [p for n in (self.actor, self.critic1, self.critic2) for p in n.parameters()]
        self.target_params = [p for n in (self.actor_t, self.critic1_t, self.critic2_t) for p in n.parameters()]
        self.n_updates = 0
        self.action_l2 = torch.zeros((), device=self.device)
        self.bc_w = torch.zeros((), device=self.device)
        self._critic_loss_fn, self._actor_loss_fn = self._critic_loss, self._actor_loss
        if hasattr(self, "_bc_loss"):          # newer rl/td3.py (Track G): BC loss used by bc_only updates
            self._bc_loss_fn = self._bc_loss
        if cfg.compile:
            assert cfg.backend == "dense", "torch.compile needs the fixed-shape dense backend"
            self._critic_loss_fn = torch.compile(self._critic_loss, mode=cfg.compile_mode)
            self._actor_loss_fn = torch.compile(self._actor_loss, mode=cfg.compile_mode)


    def _actor_loss(self, obs, act, is_demo, l2_coef, bc_w, bc_only: bool = False):
        if bc_only or self.prior_l2 <= 0:
            return super()._actor_loss(obs, act, is_demo, l2_coef, bc_w, bc_only)
        g = self.graph(obs)
        a = self.actor(g)
        m = g.robot_mask.float()
        q = self.critic1(g, a)
        lam = 1.0 / q.abs().mean().detach().clamp(min=1e-3)
        prior = ((((a ** 2).sum(-1)) * m).sum(-1) / m.sum(-1).clamp(min=1)).mean()
        loss = -lam * q.mean() + self.prior_l2 * prior
        return loss, prior.detach(), torch.ones((), device=q.device)


def load_residual_agent(path, device="cpu"):
    ck = torch.load(path, map_location=device, weights_only=False)
    cfg = dict(ck["td3_cfg"])
    if not str(device).startswith("cuda"):     # GPU runs use dense + torch.compile; same parameters on pyg
        cfg.update(backend="auto", compile=False)
    agent = ResidualTD3(TD3Config(**cfg), ck.get("graph_cfg"), device,
                        prior_l2=float(ck.get("args", {}).get("prior_l2", 0.0)))
    agent.load_state_dict(ck["agent"])
    return agent, ck
