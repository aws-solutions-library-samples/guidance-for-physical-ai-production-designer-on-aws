"""Compact graph features for PPO: obs -> flat float32 rows -> DenseGraph.

The rollout workers compute the graph features once per env step (they need them for the actor
anyway) and store them as one flat float32 row per sample, so the learner never re-runs the graph
builder. The features are exactly `models.graph_builder.GraphBuilder.dense` (robot nodes, robot-robot
edges, task-robot edges, optional obstacles), with one throughput change:

* **task subset**: only the `n_sel` next open welds of each robot's queue (lowest `task_rank` among the
  welds it owns) are kept as task nodes, i.e. `2 * n_sel` task slots instead of 16. The current goal is
  also on the robot node itself. Every robot still sees the other robot's upcoming welds. This cuts the
  dense task-edge tensors (the bulk of the GNN cost) by 16 / (2 n_sel).
* obstacles are dropped unless `obstacles=True` (welding_cell_2_small has none).

Row layout (`FeatureSpec.slices`): robot_x (R, Fr) | e_rr (R, R, Frr) | task_x (Ts, 4) |
e_tr (R, Ts, Fe) | m_task (Ts,) | active (R,) [| obst_x (O, 3) | e_or (R, O, Fo) | m_obst (O,)].
`active` = robot_goal_mask (the action of a robot without a goal is zeroed by the action map, so it is
excluded from log-probs, entropy and BC).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
import torch

from models.graph_builder import (OBS_KEYS, OBST_DIM, TASK_DIM, DenseGraph, GraphBuilder, edge_dims,
                                  robot_dim)

R = 2
RR = ("robot", "to", "robot")
TR = ("task", "to", "robot")
RT = ("robot", "to", "task")
OR = ("obstacle", "to", "robot")


@dataclass
class FeatureSpec:
    n_kp: int = 9
    self_dist: bool = True
    n_sel: int = 3          # next open welds per robot kept as task nodes
    obstacles: bool = False
    n_obst: int = 16
    sphere_k: int = 0       # > 0: append the k closest umarell sphere pairs (arm-arm gate) to the robot-robot edges
    extra: int = 0          # v2: extra robot-node features from obs["robot_extra"] (B, R, extra) (motion filter state)
    wp: bool = False        # v2: workpiece distances obs["robot_wp_dist"] (B, R, 2) -> 4 robot-node features
    no_time: bool = False   # v2: time_left is replaced by 1 (policy independent of the episode horizon)
    wedge: bool = False     # v3: obs["robot_wedge_margin"] (B, R) -> 2 robot-node features
    fixture_k: int = 0      # 2t: k spheres closest to the fixture (rl/ppo/fixture.py) -> 8k robot-node features
    fixture_cell: str = "welding_cell_2t"

    SPHERE_F = 7            # per pair: own sphere centre (3), vector to the other arm's sphere (3), tanh(clearance)

    def to_dict(self):
        return asdict(self)

    @property
    def Fr(self):
        return (robot_dim(self.n_kp, self.self_dist) + self.extra + (4 if self.wp else 0) + (2 if self.wedge else 0)
                + 8 * self.fixture_k)

    @property
    def Ts(self):
        return R * self.n_sel

    @property
    def dims(self):
        d = dict(edge_dims(self.n_kp))
        d[RR] = d[RR] + self.SPHERE_F * self.sphere_k
        return d

    @property
    def slices(self) -> dict:
        ed = self.dims
        parts = [("robot_x", (R, self.Fr)), ("e_rr", (R, R, ed[RR])), ("task_x", (self.Ts, TASK_DIM)),
                 ("e_tr", (R, self.Ts, ed[TR])), ("m_task", (self.Ts,)), ("active", (R,))]
        if self.obstacles:
            parts += [("obst_x", (self.n_obst, OBST_DIM)), ("e_or", (R, self.n_obst, ed[OR])),
                      ("m_obst", (self.n_obst,))]
        out, k = {}, 0
        for name, shp in parts:
            n = int(np.prod(shp))
            out[name] = (k, k + n, shp)
            k += n
        return out

    @property
    def dim(self) -> int:
        return max(b for _, b, _ in self.slices.values())


class FeatureBuilder:
    """Batched env obs (dict of numpy arrays with a leading batch dim, or list of obs) -> (B, D) float32."""

    def __init__(self, spec: FeatureSpec, cell_id: str = "welding_cell_2_small"):
        from models.graph_builder import default_joint_scales
        self.spec = spec
        lo, hi, vs = default_joint_scales(cell_id)
        self.gb = GraphBuilder(lo, hi, vs, n_kp=spec.n_kp, self_dist=spec.self_dist)
        self.sl = spec.slices
        if spec.sphere_k:
            from env.cell import ROBOT_IDS, load_cell
            from env.kinematics import SphereModel
            self.sm = SphereModel(load_cell(cell_id))
            self.uniq = [self.sm._arm[r].uniq for r in ROBOT_IDS]
            self.rids = ROBOT_IDS
        if spec.fixture_k:
            from .fixture import field
            self.fx = field(spec.fixture_cell)

    def sphere_edges(self, q: np.ndarray, base: np.ndarray) -> torch.Tensor:
        """k closest umarell sphere pairs (unique spheres, pair radius 0.12) -> (B, R, R, 7k) rr-edge features.

        Edge [b, i, j] (receiver i): own sphere centre in i's base frame / 2 m, vector to j's sphere (i's frame)
        / 0.5 m, tanh(clearance / 0.25 m), clearance = centre distance - 2 x 0.12 (umarell's metric)."""
        k = self.spec.sphere_k
        B = q.shape[0]
        S = [self.sm.spheres(r, q[:, i])[:, self.uniq[i]] for i, r in enumerate(self.rids)]   # (B, n_i, 3)
        D = np.linalg.norm(S[0][:, :, None] - S[1][:, None], axis=-1)                          # (B, n1, n2)
        flat = D.reshape(B, -1)
        idx = np.argpartition(flat, k - 1, axis=1)[:, :k]
        idx = np.take_along_axis(idx, np.argsort(np.take_along_axis(flat, idx, 1), 1), 1)
        i1, i2 = np.unravel_index(idx, D.shape[1:])
        p = [np.take_along_axis(S[0], i1[..., None], 1), np.take_along_axis(S[1], i2[..., None], 1)]  # (B,k,3)
        clr = np.take_along_axis(flat, idx, 1) - 2 * self.sm.pair_radius
        out = np.zeros((B, R, R, self.spec.SPHERE_F * k), np.float32)
        for i in range(R):
            j = 1 - i
            c, s_ = np.cos(base[:, i, 3]), np.sin(base[:, i, 3])
            Rinv = np.stack([np.stack([c, s_, 0 * c], -1), np.stack([-s_, c, 0 * c], -1),
                             np.stack([0 * c, 0 * c, 1 + 0 * c], -1)], -2)                     # (B,3,3)
            own = np.einsum("bij,bkj->bki", Rinv, p[i] - base[:, None, i, :3])
            vec = np.einsum("bij,bkj->bki", Rinv, p[j] - p[i])
            f = np.concatenate([own / 2.0, vec / 0.5, np.tanh(clr / 0.25)[..., None]], -1)       # (B,k,7)
            out[:, i, j] = f.reshape(B, -1)
        return torch.from_numpy(out)

    @torch.no_grad()
    def __call__(self, obs) -> np.ndarray:
        if isinstance(obs, (list, tuple)):
            obs = {k: np.stack([o[k] for o in obs]) for k in obs[0] if k in OBS_KEYS}
        s = {k: torch.as_tensor(np.asarray(v), dtype=torch.float32) for k, v in obs.items()
             if k in OBS_KEYS or k in ("robot_extra", "robot_wp_dist", "robot_wedge_margin")}
        extra = None
        if self.spec.sphere_k:
            extra = self.sphere_edges(np.asarray(obs["robot_q"], np.float64), np.asarray(obs["robot_base"], np.float64))
        if self.spec.fixture_k:
            from .fixture import fixture_features
            s["robot_fixture"] = torch.from_numpy(fixture_features(self.fx, obs["robot_q"], obs["robot_base"],
                                                                   self.spec.fixture_k))
        return self.from_torch(s, extra).numpy()

    @torch.no_grad()
    def from_torch(self, s: dict, rr_extra: torch.Tensor | None = None) -> torch.Tensor:
        sp = self.spec
        if sp.no_time and "time_left" in s:
            s = dict(s, time_left=torch.ones_like(s["time_left"]))
        dg = self.gb.dense(s)
        rx = dg.x["robot"]
        add = []
        if sp.extra:
            add.append(s["robot_extra"][..., :sp.extra])
        if sp.wp:
            w = s["robot_wp_dist"]
            add += [torch.tanh(w / 0.02), w / 0.1]
        if sp.wedge:
            m = s["robot_wedge_margin"].unsqueeze(-1)
            add += [torch.tanh(m / 0.005), m / 0.05]
        if sp.fixture_k:
            add.append(s["robot_fixture"].to(rx.dtype))
        if add:
            rx = torch.cat([rx] + add, -1)
        B = s["robot_q"].shape[0]
        T = s["task_mask"].shape[1]
        owner = s["task_owner"]                                   # (B,T,R)
        open_ = (s["task_mask"] > 0.5) & (s["task_status"] < 0.5)  # (B,T)
        rank = s["task_rank"] if "task_rank" in s else torch.zeros_like(s["task_mask"])
        idx, val = [], []
        k = min(sp.n_sel, T)
        for i in range(R):
            ok = open_ & (owner[..., i] > 0.5)
            key = torch.where(ok, rank, rank + 10.0) + torch.arange(T) * 1e-4   # stable: queue order
            top = torch.topk(-key, k, dim=1).indices                           # (B,k) lowest keys
            idx.append(top)
            val.append(torch.gather(ok, 1, top))
            if k < sp.n_sel:
                pad = sp.n_sel - k
                idx.append(torch.zeros(B, pad, dtype=torch.long))
                val.append(torch.zeros(B, pad, dtype=torch.bool))
        idx = torch.cat(idx, 1)                                   # (B,Ts)
        m_task = torch.cat(val, 1).float()
        task_x = torch.gather(dg.x["task"], 1, idx[..., None].expand(-1, -1, TASK_DIM)) * m_task[..., None]
        etr = dg.e[TR]                                            # (B,R,T,Fe)
        e_tr = torch.gather(etr, 2, idx[:, None, :, None].expand(-1, R, -1, etr.shape[-1])) * m_task[:, None, :, None]
        e_rr = dg.e[RR] if rr_extra is None else torch.cat([dg.e[RR], rr_extra.to(dg.e[RR].dtype)], -1)
        parts = [rx, e_rr, task_x, e_tr, m_task, s["robot_goal_mask"]]
        if sp.obstacles:
            parts += [dg.x["obstacle"], dg.e[OR], dg.mask["obstacle"].float()]
        return torch.cat([p.reshape(B, -1) for p in parts], 1).contiguous()


def unpack(flat: torch.Tensor, spec: FeatureSpec) -> dict:
    B = flat.shape[0]
    return {n: flat[:, a:b].reshape(B, *shp) for n, (a, b, shp) in spec.slices.items()}


def to_dense_graph(parts: dict, spec: FeatureSpec) -> DenseGraph:
    """Unpacked parts (possibly normalised robot_x / e_rr) -> DenseGraph for HeteroGNNEncoder (dense path)."""
    rx = parts["robot_x"]
    B, dev = rx.shape[0], rx.device
    rm = torch.ones(B, R, dtype=torch.bool, device=dev)
    tm = parts["m_task"] > 0.5
    eye = torch.eye(R, dtype=torch.bool, device=dev).unsqueeze(0)
    if spec.obstacles:
        ox, eor, om = parts["obst_x"], parts["e_or"], parts["m_obst"] > 0.5
    else:
        ox = rx.new_zeros(B, 0, OBST_DIM)
        eor = rx.new_zeros(B, R, 0, spec.dims[OR])
        om = torch.zeros(B, 0, dtype=torch.bool, device=dev)
    m_tr = rm.unsqueeze(2) & tm.unsqueeze(1)
    return DenseGraph(
        x={"robot": rx, "task": parts["task_x"], "obstacle": ox},
        mask={"robot": rm, "task": tm, "obstacle": om},
        e={TR: parts["e_tr"], RT: parts["e_tr"].transpose(1, 2), OR: eor, RR: parts["e_rr"]},
        emask={TR: m_tr, RT: m_tr.transpose(1, 2), OR: rm.unsqueeze(2) & om.unsqueeze(1),
               RR: rm.unsqueeze(2) & rm.unsqueeze(1) & ~eye},
    )
