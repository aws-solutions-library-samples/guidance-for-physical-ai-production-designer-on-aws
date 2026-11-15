"""Heterogeneous attention GNN actor / twin critics (ported from ~/roboballet/models/gnn_policy.py).

Message passing uses `EdgeAttnConv`, multi-head graph attention whose messages are MLPs of
(receiver, sender, edge features), so relative geometry shapes the message *content*, not
only the attention weights (stock GATConv/GATv2Conv).

Two numerically equivalent execution paths share the same parameters:
  * PyG path  : `HeteroData` + `HeteroConv` / `MessagePassing` (sparse, any graph sizes)
  * dense path: `DenseGraph` (padded tensors + masked softmax); fixed shapes, compile-able.

Actor : robot embedding -> 6 joint velocities in [-1, 1] (MLP shared across robots).
Critic: robot features + that robot's action -> per-robot Q_i; Q(s, a) = mean_i Q_i.
"""
from __future__ import annotations

import torch
import torch.nn as nn
from torch_geometric.data import HeteroData
from torch_geometric.nn import HeteroConv, MessagePassing
from torch_geometric.utils import scatter, softmax

from .graph_builder import DEFAULT_N_KP, EDGE_KEYS, N_JOINTS, OBST_DIM, TASK_DIM, DenseGraph, edge_dims, robot_dim

NODE_TYPES = ("robot", "task", "obstacle")
UPDATED_TYPES = ("robot", "task")  # obstacles are static: never receive messages


def mlp(i, h, o, n_hidden=1, act=nn.ReLU):
    layers, d = [], i
    for _ in range(n_hidden):
        layers += [nn.Linear(d, h), act()]
        d = h
    layers.append(nn.Linear(d, o))
    return nn.Sequential(*layers)


class EdgeAttnConv(MessagePassing):
    """m_ij = W2 relu(Wd x_i + Ws x_j + We e_ij + b);  out_i = sum_j softmax_j(a(relu m_ij)) * m_ij (per head)."""

    def __init__(self, dim: int, edge_dim: int, heads: int = 4):
        super().__init__(aggr="add", node_dim=0)
        assert dim % heads == 0
        self.dim, self.heads = dim, heads
        self.lin_dst = nn.Linear(dim, dim)
        self.lin_src = nn.Linear(dim, dim, bias=False)
        self.lin_edge = nn.Linear(edge_dim, dim, bias=False)
        self.lin_out = nn.Linear(dim, dim)
        self.att = nn.Linear(dim, heads)

    def forward(self, x, edge_index, edge_attr):
        x_src, x_dst = x if isinstance(x, tuple) else (x, x)
        return self.propagate(edge_index, x=(self.lin_src(x_src), self.lin_dst(x_dst)), edge_attr=edge_attr,
                              size=(x_src.size(0), x_dst.size(0)))

    def message(self, x_i, x_j, edge_attr, index, ptr, size_i):
        m = self.lin_out(torch.relu(x_i + x_j + self.lin_edge(edge_attr)))
        a = softmax(self.att(torch.relu(m)), index, ptr, size_i)  # (E, heads)
        return (m.view(-1, self.heads, self.dim // self.heads) * a.unsqueeze(-1)).view(-1, self.dim)

    def dense(self, x_dst, x_src, e, mask):
        """x_dst (B,Nd,H), x_src (B,Ns,H), e (B,Nd,Ns,H), mask (B,Nd,Ns)."""
        if x_src.shape[1] == 0 or x_dst.shape[1] == 0:
            return x_dst.new_zeros(x_dst.shape[0], x_dst.shape[1], self.dim)
        m = self.lin_out(torch.relu(self.lin_dst(x_dst).unsqueeze(2) + self.lin_src(x_src).unsqueeze(1)
                                    + self.lin_edge(e)))
        mk = mask.unsqueeze(-1)
        a = torch.softmax(self.att(torch.relu(m)).masked_fill(~mk, -1e9), dim=2) * mk  # no valid edge -> 0
        B, Nd, Ns, _ = m.shape
        out = (m.view(B, Nd, Ns, self.heads, -1) * a.unsqueeze(-1)).sum(2)
        return out.reshape(B, Nd, self.dim)


class HeteroGNNEncoder(nn.Module):
    def __init__(self, robot_in: int, n_kp: int = DEFAULT_N_KP, hidden: int = 128, layers: int = 3, heads: int = 4):
        super().__init__()
        self.enc = nn.ModuleDict({
            "robot": mlp(robot_in, hidden, hidden),
            "task": mlp(TASK_DIM, hidden, hidden),
            "obstacle": mlp(OBST_DIM, hidden, hidden),
        })
        self.edge_keys = list(EDGE_KEYS)
        ed = edge_dims(n_kp)
        self.edge_enc = nn.ModuleDict({"__".join(k): mlp(ed[k], hidden, hidden) for k in self.edge_keys})
        self.convs = nn.ModuleList([
            HeteroConv({k: EdgeAttnConv(hidden, hidden, heads) for k in self.edge_keys}, aggr="sum")
            for _ in range(layers)
        ])
        self.update = nn.ModuleList([
            nn.ModuleDict({t: mlp(2 * hidden, hidden, hidden) for t in UPDATED_TYPES}) for _ in range(layers)
        ])
        self.norm = nn.ModuleList([
            nn.ModuleDict({t: nn.LayerNorm(hidden) for t in UPDATED_TYPES}) for _ in range(layers)
        ])

    def _node_update(self, layer, h, agg):
        for t in UPDATED_TYPES:
            h[t] = self.norm[layer][t](h[t] + self.update[layer][t](torch.cat([h[t], agg[t]], -1)))
        return h

    def forward(self, g: HeteroData | DenseGraph, robot_x: torch.Tensor | None = None) -> dict:
        """Node embeddings: flat (N_t, H) for HeteroData, padded (B, N_t, H) for DenseGraph."""
        x = dict(g.x) if isinstance(g, DenseGraph) else {t: g[t].x for t in NODE_TYPES}
        if robot_x is not None:
            x["robot"] = robot_x
        h = {t: self.enc[t](x[t]) for t in NODE_TYPES}
        if isinstance(g, DenseGraph):
            e = {k: self.edge_enc["__".join(k)](g.e[k]) for k in self.edge_keys}
            for layer, conv in enumerate(self.convs):
                agg = {t: torch.zeros_like(h[t]) for t in UPDATED_TYPES}
                for k in self.edge_keys:
                    src, _, dst = k
                    agg[dst] = agg[dst] + conv.convs[k].dense(h[dst], h[src], e[k], g.emask[k])
                h = self._node_update(layer, h, agg)
            return h
        e = {k: self.edge_enc["__".join(k)](g[k].edge_attr) for k in self.edge_keys}
        ei = {k: g[k].edge_index for k in self.edge_keys}
        for layer, conv in enumerate(self.convs):
            out = conv(h, ei, edge_attr_dict=e)
            agg = {t: out.get(t, torch.zeros_like(h[t])) for t in UPDATED_TYPES}
            h = self._node_update(layer, h, agg)
        return h


class GNNActor(nn.Module):
    def __init__(self, hidden=128, layers=3, heads=4, n_kp: int = DEFAULT_N_KP, robot_in: int | None = None):
        super().__init__()
        self.encoder = HeteroGNNEncoder(robot_in or robot_dim(n_kp), n_kp, hidden, layers, heads)
        self.head = mlp(hidden, hidden, N_JOINTS)

    def forward(self, g: HeteroData | DenseGraph) -> torch.Tensor:
        """Padded actions (B, R, 6) in [-1, 1]; padded robots get 0."""
        a = torch.tanh(self.head(self.encoder(g)["robot"]))
        if isinstance(g, DenseGraph):
            return a * g.robot_mask.unsqueeze(-1)
        out = a.new_zeros(*g.robot_mask.shape, N_JOINTS)
        out[g.robot_mask] = a
        return out


class GNNCritic(nn.Module):
    """Q(s, a): the padded (B, R, 6) action is appended to the robot node features."""

    def __init__(self, hidden=128, layers=3, heads=4, n_kp: int = DEFAULT_N_KP, robot_in: int | None = None):
        super().__init__()
        self.encoder = HeteroGNNEncoder((robot_in or robot_dim(n_kp)) + N_JOINTS, n_kp, hidden, layers, heads)
        self.head = mlp(hidden, hidden, 1)

    def forward(self, g: HeteroData | DenseGraph, action: torch.Tensor, return_per_robot=False):
        if isinstance(g, DenseGraph):
            robot_x = torch.cat([g.x["robot"], action], -1)
            q_i = self.head(self.encoder(g, robot_x)["robot"]).squeeze(-1)  # (B, R)
            m = g.robot_mask.float()
            q = (q_i * m).sum(1) / m.sum(1).clamp(min=1)
            return (q, q_i) if return_per_robot else q
        robot_x = torch.cat([g["robot"].x, action[g.robot_mask]], -1)
        q_i = self.head(self.encoder(g, robot_x)["robot"]).squeeze(-1)
        q = scatter(q_i, g["robot"].batch, dim=0, dim_size=g.num_graphs, reduce="mean")
        return (q, q_i) if return_per_robot else q
