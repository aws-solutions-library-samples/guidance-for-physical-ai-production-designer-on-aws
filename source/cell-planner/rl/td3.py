"""TD3 (Fujimoto et al. 2018) with GNN actor / twin GNN critics, n-step targets and demo BC.

Ported from ~/roboballet/rl/td3.py. Additions:
  * batches are {"obs", "next_obs", "action", "reward", "terminated", "disc", "is_demo"} as produced
    by `rl.her_buffer.HERReplayBuffer.sample` (+ `merge_batches` for a demo buffer);
  * optional behaviour-cloning term on demo transitions with a Q-filter (Nair et al. 2018):
        L_actor = -Q1(s, pi(s)) / mean|Q1|  +  bc_weight * mean_demo( 1[Q1(s,a_D) > Q1(s,pi(s))] * |pi(s) - a_D|^2 )
    The Q normalisation (as in TD3+BC, Fujimoto & Gu 2021) makes bc_weight scale-free. With
    bc_weight = 0 the plain TD3 objective -Q is used.
"""
from __future__ import annotations

import copy
import dataclasses
from dataclasses import dataclass

import numpy as np
import torch
import torch.nn.functional as F

from models.gnn_policy import GNNActor, GNNCritic
from models.graph_builder import GraphBuilder


@dataclass
class TD3Config:
    hidden: int = 128
    layers: int = 3
    heads: int = 4
    actor_lr: float = 3e-4
    critic_lr: float = 3e-4
    gamma: float = 0.99
    tau: float = 0.005
    policy_noise: float = 0.1      # target smoothing (normalised action units)
    noise_clip: float = 0.3
    policy_delay: int = 2
    batch_size: int = 256
    grad_clip: float = 10.0
    backend: str = "auto"          # "pyg" | "dense" | "auto" (pyg on CPU; dense + CUDA graphs on GPU)
    compile: bool = False
    compile_mode: str = "default"
    action_l2: float = 0.0
    n_step: int = 5
    actor_out_init: float = 0.1    # <1: start near zero joint velocity
    bc_weight: float = 0.0         # BC on demo transitions (0 = off)
    q_filter: bool = False         # only clone demo actions the critic rates better than pi(s) (Nair et al.);
                                   # off by default: early critics overrate saturated OOD actions (smoke runs)


def _to_t(d: dict, device):
    return {k: torch.as_tensor(v, dtype=torch.float32, device=device) for k, v in d.items()}


def merge_batches(a: dict, b: dict | None) -> dict:
    """Concatenate two sampled batches (agent + demo); adds/keeps an `is_demo` flag."""
    if b is None:
        out = dict(a)
        out.setdefault("is_demo", np.zeros(len(a["reward"]), np.float32))
        return out
    out = {}
    for k in ("obs", "next_obs"):
        out[k] = {kk: np.concatenate([a[k][kk], b[k][kk]]) for kk in a[k]}
    for k in ("action", "reward", "terminated", "disc"):
        out[k] = np.concatenate([a[k], b[k]])
    if "bc_action" in a or "bc_action" in b:   # BC targets (default: the stored action)
        out["bc_action"] = np.concatenate([a.get("bc_action", a["action"]), b.get("bc_action", b["action"])])
    out["is_demo"] = np.concatenate([np.zeros(len(a["reward"]), np.float32), np.ones(len(b["reward"]), np.float32)])
    return out


class TD3:
    def __init__(self, cfg: TD3Config, graph_cfg: dict | None = None, device: str = "cpu"):
        cfg = dataclasses.replace(cfg)
        self.device = torch.device(device)
        if self.device.type == "cuda":   # TF32 matmuls: 28 -> 19 ms per h128/L3/B256 update on an L4 (Track G)
            torch.set_float32_matmul_precision("high")
        if cfg.backend == "auto":
            cuda = self.device.type == "cuda"
            # CPU: pyg skips padded task/obstacle slots (3-5x faster than dense here);
            # CUDA: dense fixed shapes + CUDA graphs (launch-bound otherwise), as in RoboBallet.
            cfg.backend = "dense" if cuda else "pyg"
            if cuda:
                cfg.compile, cfg.compile_mode = True, "reduce-overhead"
        self.cfg = cfg
        self.gb = GraphBuilder(**(graph_cfg or {}))
        self.graph_cfg = self.gb.config()
        nk = self.gb.n_kp
        ri = self.gb.robot_dim
        self.actor = GNNActor(cfg.hidden, cfg.layers, cfg.heads, nk, robot_in=ri).to(self.device)
        if cfg.actor_out_init != 1.0:
            with torch.no_grad():
                last = self.actor.head[-1]
                last.weight.mul_(cfg.actor_out_init)
                last.bias.mul_(cfg.actor_out_init)
        self.critic1 = GNNCritic(cfg.hidden, cfg.layers, cfg.heads, nk, robot_in=ri).to(self.device)
        self.critic2 = GNNCritic(cfg.hidden, cfg.layers, cfg.heads, nk, robot_in=ri).to(self.device)
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
        self._bc_loss_fn = self._bc_loss
        if cfg.compile:
            self._bc_loss_fn = torch.compile(self._bc_loss, mode=cfg.compile_mode)
            assert cfg.backend == "dense", "torch.compile needs the fixed-shape dense backend"
            self._critic_loss_fn = torch.compile(self._critic_loss, mode=cfg.compile_mode)
            self._actor_loss_fn = torch.compile(self._actor_loss, mode=cfg.compile_mode)

    def graph(self, s: dict):
        return self.gb.dense(s) if self.cfg.backend == "dense" else self.gb.build(s)

    @torch.no_grad()
    def act(self, obs_batch: dict, noise: float = 0.0) -> np.ndarray:
        """obs_batch: dict of numpy arrays (B, ...). Returns (B, R, 6) actions in [-1, 1]."""
        g = self.graph(_to_t(obs_batch, self.device))
        a = self.actor(g)
        if noise > 0:
            a = (a + noise * torch.randn_like(a)).clamp(-1, 1)
        a = a * g.robot_mask.unsqueeze(-1)
        return a.cpu().numpy()

    def _critic_loss(self, obs, nxt, act, rew, term, disc):
        c = self.cfg
        g, g2 = self.graph(obs), self.graph(nxt)
        rmask = g.robot_mask.unsqueeze(-1).float()
        with torch.no_grad():
            noise = (torch.randn_like(act) * c.policy_noise).clamp(-c.noise_clip, c.noise_clip)
            a2 = ((self.actor_t(g2) + noise).clamp(-1, 1)) * rmask
            q_t = torch.min(self.critic1_t(g2, a2), self.critic2_t(g2, a2))
            y = rew + disc * (1 - term) * q_t
        q1, q2 = self.critic1(g, act), self.critic2(g, act)
        loss = F.mse_loss(q1, y) + F.mse_loss(q2, y)
        return loss, q1.detach().mean(), y.mean()

    def _bc_loss(self, obs, act, is_demo, l2_coef, bc_w):
        return self._actor_loss(obs, act, is_demo, l2_coef, bc_w, bc_only=True)

    def _actor_loss(self, obs, act, is_demo, l2_coef, bc_w, bc_only: bool = False):
        g = self.graph(obs)
        a = self.actor(g)
        m = g.robot_mask.float()
        # BC targets may be NaN for (row, robot) pairs that should not be cloned (Track G: expert "waiting")
        valid = torch.isfinite(act).all(-1).float()
        act = torch.nan_to_num(act)
        mb = m * valid
        if bc_only:   # pretraining: pure behaviour cloning on the (demo) batch
            se = (((a - act) ** 2).sum(-1) * mb).sum(-1) / mb.sum(-1).clamp(min=1)
            bc = (se * is_demo).sum() / is_demo.sum().clamp(min=1)
            return bc, bc.detach(), torch.ones((), device=bc.device)
        q = self.critic1(g, a)
        l2 = ((a ** 2).mean(-1) * m).sum() / m.sum().clamp(min=1)
        if self.cfg.bc_weight > 0:
            lam = 1.0 / q.abs().mean().detach().clamp(min=1e-3)
            se = (((a - act) ** 2).sum(-1) * mb).sum(-1) / mb.sum(-1).clamp(min=1)     # (B,)
            keep = is_demo
            if self.cfg.q_filter:
                with torch.no_grad():
                    keep = keep * (self.critic1(g, act) > q).float()
            bc = (se * keep).sum() / is_demo.sum().clamp(min=1)
            loss = -lam * q.mean() + bc_w * bc + l2_coef * l2
            return loss, bc.detach(), keep.sum().detach() / is_demo.sum().clamp(min=1)
        loss = -q.mean() + l2_coef * l2
        z = torch.zeros((), device=q.device)
        return loss, z, z

    def update(self, batch: dict, bc_only: bool = False) -> dict:
        """One TD3 step on a sampled batch. Returns detached scalar tensors (no host sync).
        bc_only: critic TD step as usual, actor step = behaviour cloning only (demo pretraining)."""
        c = self.cfg
        obs, nxt = _to_t(batch["obs"], self.device), _to_t(batch["next_obs"], self.device)
        act = torch.as_tensor(batch["action"], dtype=torch.float32, device=self.device)
        bc_act = torch.as_tensor(batch["bc_action"], dtype=torch.float32, device=self.device) \
            if "bc_action" in batch else act
        rew = torch.as_tensor(batch["reward"], dtype=torch.float32, device=self.device)
        term = torch.as_tensor(batch["terminated"], dtype=torch.float32, device=self.device)
        disc = torch.as_tensor(batch["disc"], dtype=torch.float32, device=self.device)
        is_demo = torch.as_tensor(batch.get("is_demo", np.zeros(len(rew), np.float32)), dtype=torch.float32,
                                  device=self.device)

        critic_loss, q_mean, y_mean = self._critic_loss_fn(obs, nxt, act, rew, term, disc)
        self.critic_opt.zero_grad(set_to_none=True)
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic_params, c.grad_clip, foreach=True)
        self.critic_opt.step()
        self.n_updates += 1
        info = {"loss/critic": critic_loss.detach().clone(), "q/mean": q_mean.clone(), "q/target": y_mean.clone()}

        if bc_only:
            demo = is_demo if is_demo.sum() > 0 else torch.ones_like(is_demo)
            actor_loss, bc, keep = self._bc_loss_fn(obs, bc_act, demo, self.action_l2, self.bc_w)
            self.actor_opt.zero_grad(set_to_none=True)
            actor_loss.backward(inputs=self.actor_params)
            torch.nn.utils.clip_grad_norm_(self.actor_params, c.grad_clip, foreach=True)
            self.actor_opt.step()
            info["loss/bc"] = bc.clone()
            with torch.no_grad():
                torch._foreach_lerp_(self.target_params, self.online_params, c.tau)
        elif self.n_updates % c.policy_delay == 0:
            actor_loss, bc, keep = self._actor_loss_fn(obs, bc_act, is_demo, self.action_l2.fill_(c.action_l2),
                                                       self.bc_w.fill_(c.bc_weight))
            self.actor_opt.zero_grad(set_to_none=True)
            actor_loss.backward(inputs=self.actor_params)
            torch.nn.utils.clip_grad_norm_(self.actor_params, c.grad_clip, foreach=True)
            self.actor_opt.step()
            info["loss/actor"] = actor_loss.detach().clone()
            if c.bc_weight > 0:
                info["loss/bc"] = bc.clone()
                info["bc/q_filter_keep"] = keep.clone()
            with torch.no_grad():
                torch._foreach_lerp_(self.target_params, self.online_params, c.tau)
        return info

    def bc_update(self, batch: dict) -> dict:
        """Actor-only behaviour cloning step on (demo) batch rows (no critic update; Track G pure-BC runs)."""
        c = self.cfg
        obs = _to_t(batch["obs"], self.device)
        tgt = batch["bc_action"] if "bc_action" in batch else batch["action"]
        act = torch.as_tensor(tgt, dtype=torch.float32, device=self.device)
        is_demo = torch.as_tensor(batch.get("is_demo", np.ones(len(tgt), np.float32)), dtype=torch.float32,
                                  device=self.device)
        demo = is_demo if is_demo.sum() > 0 else torch.ones_like(is_demo)
        loss, bc, _ = self._bc_loss_fn(obs, act, demo, self.action_l2, self.bc_w)
        self.actor_opt.zero_grad(set_to_none=True)
        loss.backward(inputs=self.actor_params)
        torch.nn.utils.clip_grad_norm_(self.actor_params, c.grad_clip, foreach=True)
        self.actor_opt.step()
        with torch.no_grad():
            torch._foreach_lerp_([p for p in self.actor_t.parameters()], self.actor_params, c.tau)
        return {"loss/bc": bc.detach().clone()}

    def state_dict(self):
        d = {k: getattr(self, k).state_dict() for k in
             ("actor", "critic1", "critic2", "actor_t", "critic1_t", "critic2_t", "actor_opt", "critic_opt")}
        d["n_updates"] = self.n_updates
        return d

    def load_state_dict(self, sd, actor_only: bool = False):
        keys = ("actor", "actor_t") if actor_only else \
            ("actor", "critic1", "critic2", "actor_t", "critic1_t", "critic2_t", "actor_opt", "critic_opt")
        for k in keys:
            if k in sd:
                getattr(self, k).load_state_dict(sd[k])
        if not actor_only:
            self.n_updates = int(sd.get("n_updates", 0))


def load_agent(path, device="cpu"):
    """Checkpoint -> (TD3 agent, checkpoint dict)."""
    ck = torch.load(path, map_location=device, weights_only=False)
    agent = TD3(TD3Config(**ck["td3_cfg"]), ck.get("graph_cfg"), device)
    agent.load_state_dict(ck["agent"])
    return agent, ck
