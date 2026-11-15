"""Graph-attention Q network for the hierarchical scheduler (Track H).

Nodes: 2 robot nodes (slot 0 = deciding arm) + up to T_MAX weld-task nodes. Message passing is
multi-head attention on the complete robot/task graph (a graph transformer: every node attends to
every valid node; node type embeddings), L layers, pre-LayerNorm residual blocks.

Q heads (one scalar per action, shared weights, so the network handles any number of welds):
  weld edge (t, b): MLP([h_robot_dec, h_robot_other, h_task_t, enc(cand_feat[t, b])])
  special j       : MLP([h_robot_dec, h_robot_other, mean_t h_task, enc(spec_feat[j])])
Output (B, N_ACT) with invalid actions set to -1e9 by ``masked_q``.
"""
from __future__ import annotations

import torch
import torch.nn as nn

from .features import FC, FR, FS, FT


def mlp(i, h, o, n=2):
    layers, d = [], i
    for _ in range(n - 1):
        layers += [nn.Linear(d, h), nn.ReLU()]
        d = h
    layers.append(nn.Linear(d, o))
    return nn.Sequential(*layers)


class Block(nn.Module):
    def __init__(self, h, heads):
        super().__init__()
        self.ln1, self.ln2 = nn.LayerNorm(h), nn.LayerNorm(h)
        self.att = nn.MultiheadAttention(h, heads, batch_first=True)
        self.ff = mlp(h, 2 * h, h)

    def forward(self, x, pad):
        y = self.ln1(x)
        x = x + self.att(y, y, y, key_padding_mask=pad, need_weights=False)[0]
        return x + self.ff(self.ln2(x))


class HierQNet(nn.Module):
    def __init__(self, hidden=128, layers=3, heads=4):
        super().__init__()
        H = hidden
        self.cfg = dict(hidden=hidden, layers=layers, heads=heads)
        self.enc_r = mlp(FR, H, H)
        self.enc_t = mlp(FT, H, H)
        self.enc_c = mlp(FC, H, H)
        self.enc_s = mlp(FS, H, H)
        self.type_emb = nn.Parameter(torch.zeros(3, H))     # deciding robot, other robot, task
        self.blocks = nn.ModuleList([Block(H, heads) for _ in range(layers)])
        self.ln = nn.LayerNorm(H)
        # first layer of the edge head split per input block (same function as Linear on the concat,
        # but computed per node and broadcast; only valid (task, branch) edges are evaluated)
        self.qc_r, self.qc_o, self.qc_t = nn.Linear(H, H), nn.Linear(H, H, bias=False), nn.Linear(H, H, bias=False)
        self.qc_c = nn.Linear(H, H, bias=False)
        self.q_c = mlp(H, H, 2, n=2)          # [Q, collision logit]
        self.q_s = mlp(4 * H, H, 2, n=3)

    def forward(self, robot, task, task_mask, cand, spec):
        return self.forward_both(robot, task, task_mask, cand, spec)[0]

    def forward_both(self, robot, task, task_mask, cand, spec):
        """-> Q (B, N_ACT), collision logit (B, N_ACT) (learned safety critic)."""
        B, T = task.shape[:2]
        K = cand.shape[2]
        xr = self.enc_r(robot) + self.type_emb[:2]
        xt = self.enc_t(task) + self.type_emb[2]
        x = torch.cat([xr, xt], 1)
        pad = torch.cat([torch.zeros(B, 2, dtype=torch.bool, device=task.device), ~task_mask], 1)
        for blk in self.blocks:
            x = blk(x, pad)
        x = self.ln(x)
        hd, ho, ht = x[:, 0], x[:, 1], x[:, 2:]
        valid = cand[..., 0] > 0.5                                       # (B,T,K) candidate exists
        bi, ti, ki = valid.nonzero(as_tuple=True)
        qc = cand.new_full((B, T, K), -1e9)
        cc = cand.new_full((B, T, K), 10.0)
        if len(bi):
            a = self.qc_r(hd) + self.qc_o(ho)                            # (B,H)
            bt = self.qc_t(ht)                                           # (B,T,H)
            ec = self.qc_c(torch.relu(self.enc_c(cand[bi, ti, ki]) ))    # (E,H)
            z = torch.relu(a[bi] + bt[bi, ti] + ec)
            o = self.q_c(z)
            qc = qc.index_put((bi, ti, ki), o[:, 0])
            cc = cc.index_put((bi, ti, ki), o[:, 1])
        qc, cc = qc.reshape(B, T * K), cc.reshape(B, T * K)
        tm = task_mask.float().unsqueeze(-1)
        pool = (ht * tm).sum(1) / tm.sum(1).clamp(min=1.0)
        es = self.enc_s(spec)                                            # (B,4,H)
        S = spec.shape[1]
        zs = torch.cat([hd[:, None].expand(B, S, -1), ho[:, None].expand(B, S, -1), pool[:, None].expand(B, S, -1), es], -1)
        os_ = self.q_s(zs)
        return torch.cat([qc, os_[..., 0]], 1), torch.cat([cc, os_[..., 1]], 1)


def masked_q(q, mask):
    return q.masked_fill(~mask, -1e9)


def to_torch(obs: dict, device="cpu"):
    """Single obs dict or batched numpy dict -> tensors (adds the batch dim if needed)."""
    out = {}
    for k, v in obs.items():
        t = torch.as_tensor(v)
        if k in ("task_mask", "mask"):
            t = t.bool()
        else:
            t = t.float()
        out[k] = t.to(device)
    if out["robot"].dim() == 2:
        out = {k: v.unsqueeze(0) for k, v in out.items()}
    return out


def safe_q(q, logit, mask, thr):
    """Q restricted to valid actions whose learned collision probability is < thr; if there is none,
    the valid action with the lowest collision probability."""
    ok = mask & (torch.sigmoid(logit) < thr)
    none = ~ok.any(-1, keepdim=True)
    sel = torch.where(none, mask & (logit <= logit.masked_fill(~mask, 1e9).min(-1, keepdim=True).values), ok)
    return q.masked_fill(~sel, -1e9)


@torch.no_grad()
def greedy_action(net, obs, device="cpu", thr=None):
    """argmax Q over valid actions (``thr`` given: only over actions the safety critic deems safe)."""
    b = to_torch(obs, device)
    q, c = net.forward_both(b["robot"], b["task"], b["task_mask"], b["cand"], b["spec"])
    if thr is None:
        return int(masked_q(q, b["mask"]).argmax(-1)[0])
    return int(safe_q(q, c, b["mask"], thr).argmax(-1)[0])


@torch.no_grad()
def q_and_risk(net, obs, device="cpu"):
    b = to_torch(obs, device)
    q, c = net.forward_both(b["robot"], b["task"], b["task_mask"], b["cand"], b["spec"])
    return q[0].numpy(), torch.sigmoid(c[0]).numpy()
