"""Environment observation (DESIGN.md obs dict) -> heterogeneous graph.

Ported from ~/roboballet/models/graph_builder.py and extended to the KUKA welding obs:
6-DoF arms, full TCP pose, goal pose (current weld) with fine-scale error features, dwell,
weld-task nodes with pose / approach / status / owner, and obstacle boxes.

Works on padded, batched observation tensors (B, R|T|O, ...) plus masks and emits either
  * a `DenseGraph` (padded tensors + masks; fixed shapes, compile-able, fast for these tiny
    graphs), or
  * a single batched `torch_geometric.data.HeteroData` containing only the valid nodes.

Node types:  robot, task, obstacle
Edge types:  (robot, to, robot)     all ordered pairs i != j
             (obstacle, to, robot)  obstacle -> robot
             (task, to, robot)      task -> robot   (allocation / goal)
             (robot, to, task)      robot -> task   (lets a weld "see" competing arms)

Every geometric quantity on a node or edge that is received by robot i is expressed in robot
i's base frame (translation + yaw), so the policy is invariant to where the cell sits in the
world and equivariant to permuting the robots. Task nodes carry no geometry; their geometry
is on the edges. Box obstacles are world axis-aligned: for base yaws that are not multiples
of 90 deg their extent feature is a conservative rotated AABB (as in RoboBallet).

Obs keys (DESIGN.md + env/rewards.py, Track E): robot_q (R,6) robot_qd (R,6) robot_base (R,4)
robot_tcp_pos (R,3) robot_tcp_rot6 (R,6) robot_kp (R,K,3) robot_goal (R,9) robot_goal_mask (R,)
robot_dwell (R,) task_pos (T,3) task_rot6 (T,6) task_approach (T,3) task_status (T,)
task_owner (T,R) task_mask (T,) obst_pos (O,3) obst_size (O,3) obst_mask (O,) min_clearance (1,)
Optional: robot_goal_approach (R,3), robot_entry (R,3) (zeros if absent), task_rank (T,) (queue position / 16,
1 = done or unqueued), time_left (1,), robot_mask (R,) (ones). rot6 = [R[:,0], R[:,1]] (first two columns).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch
from torch_geometric.data import HeteroData

N_JOINTS = 6
DEFAULT_N_KP = 6
POS_SCALE = 2.0      # m; coarse geometry is divided by this (KR240 reach ~2.9 m)
FINE_POS = 0.02      # m; tanh(err / FINE_POS) resolves the last few cm (tolerance 10 mm)
FINE_DIST = 0.05     # m
CLEAR_SCALE = 0.5    # m; arm-arm clearance feature scale (umarell gate 0.20 m)

OBS_KEYS = ["robot_q", "robot_qd", "robot_base", "robot_tcp_pos", "robot_tcp_rot6", "robot_kp",
            "robot_goal", "robot_goal_mask", "robot_dwell", "robot_goal_approach", "robot_entry", "robot_mask",
            "task_pos", "task_rot6", "task_approach", "task_status", "task_owner", "task_rank", "task_mask",
            "obst_pos", "obst_size", "obst_mask", "min_clearance", "time_left", "robot_self_dist"]


def kp_split(n_kp: int) -> tuple[int, int]:
    """(n_arm, n_tool): env keypoints = arm chain (shoulder .. wrist) followed by tool points."""
    n_tool = 4 if n_kp == 9 else 2
    return n_kp - n_tool, n_tool


def self_dim(n_kp: int) -> int:
    n_arm, n_tool = kp_split(n_kp)
    return 2 * n_tool * (n_arm - 1)


SELF_DIST_DIM = 6   # optional (GraphBuilder(self_dist=True)): obs robot_self_dist (R,3) as tanh(d/5 cm), d/0.3 m
SELF_DIST_FAR = 0.3


def robot_dim(n_kp: int, self_dist: bool = False) -> int:
    # q, qd (12) | tcp pos, rot6 (9) | kp (3K) | goal pos, rot6 (9) | err tcp-frame fine+coarse (6)
    # | |e| lin + log (2) | goal axis in tcp frame (3) | axis err (2) | rel rot6 (6)
    # | goal mask, dwell (2) | clearance (1) | goal approach, entry dir (base frame), their angle (7)
    # | time left (1) | self-proximity: tool point -> arm segment distances, linear + tanh
    return 12 + 9 + 3 * n_kp + 9 + 6 + 2 + 3 + 2 + 6 + 2 + 1 + 7 + 1 + self_dim(n_kp) + (SELF_DIST_DIM if self_dist else 0)


TASK_DIM = 4     # done, open, assigned-to-anyone, queue rank (0 = head of its robot's queue, 1 = done/none)
OBST_DIM = 3     # half extents
TASK_EDGE_DIM = 3 + 6 + 3 + 3 + 2 + 1 + 4


def edge_dims(n_kp: int) -> dict:
    return {
        ("task", "to", "robot"): TASK_EDGE_DIM,
        ("robot", "to", "task"): TASK_EDGE_DIM,
        ("obstacle", "to", "robot"): 3 + 3 + n_kp + 1,
        # sender kp (3K), tcp (3), tool axis (3), goal pos + mask (4), pair min dists (2K), min (1), clearance (1)
        ("robot", "to", "robot"): 5 * n_kp + 12,
    }


EDGE_KEYS = [("task", "to", "robot"), ("robot", "to", "task"), ("obstacle", "to", "robot"),
             ("robot", "to", "robot")]


def _rot_inv(yaw):
    """Rz(-yaw) as (..., 3, 3)."""
    c, s = torch.cos(yaw), torch.sin(yaw)
    z, o = torch.zeros_like(c), torch.ones_like(c)
    return torch.stack([torch.stack([c, s, z], -1), torch.stack([-s, c, z], -1), torch.stack([z, z, o], -1)], -2)


def rot6_to_mat(r6):
    """(..., 6) first two columns -> (..., 3, 3) rotation (Gram-Schmidt; zeros stay finite)."""
    a, b = r6[..., :3], r6[..., 3:6]
    c0 = a / a.norm(dim=-1, keepdim=True).clamp(min=1e-8)
    b = b - (c0 * b).sum(-1, keepdim=True) * c0
    c1 = b / b.norm(dim=-1, keepdim=True).clamp(min=1e-8)
    c2 = torch.cross(c0, c1, dim=-1)
    return torch.stack([c0, c1, c2], -1)


def _box_sdf(p, centre, half):
    q = (p - centre).abs() - half
    return q.clamp(min=0).norm(dim=-1) + q.max(-1).values.clamp(max=0)


def _seg_dist(p, a, b):
    """Distance from points p (..., P, 3) to segments a-b (..., S, 3) -> (..., P, S)."""
    p, a, b = p.unsqueeze(-2), a.unsqueeze(-3), b.unsqueeze(-3)
    ab = b - a
    t = (((p - a) * ab).sum(-1) / (ab * ab).sum(-1).clamp(min=1e-9)).clamp(0, 1)
    return (p - (a + t.unsqueeze(-1) * ab)).norm(dim=-1)


def _angle(u, v):
    """Angle (rad) between vectors (..., 3), robust near 0 (atan2 form)."""
    cr = torch.cross(u, v, dim=-1).norm(dim=-1)
    return torch.atan2(cr, (u * v).sum(-1))


def obs_to_torch(obs: dict, device="cpu") -> dict:
    """Single env observation (numpy, unbatched) -> batched torch dict (B=1)."""
    return {k: torch.as_tensor(np.asarray(v), dtype=torch.float32, device=device).unsqueeze(0)
            for k, v in obs.items() if k in OBS_KEYS}


def stack_obs(obs_list: list[dict], device="cpu") -> dict:
    keys = [k for k in OBS_KEYS if k in obs_list[0]]
    return {k: torch.as_tensor(np.stack([o[k] for o in obs_list]), dtype=torch.float32, device=device) for k in keys}


@dataclass
class DenseGraph:
    """Padded graph. x[t]: (B, N_t, F_t), mask[t]: (B, N_t); e[k]: (B, N_dst, N_src, F_k), emask[k]."""
    x: dict
    mask: dict
    e: dict
    emask: dict

    @property
    def robot_mask(self):
        return self.mask["robot"]

    @property
    def num_graphs(self):
        return self.mask["robot"].shape[0]


class GraphBuilder:
    """Builds graphs from batched obs tensors.

    q_lo/q_hi: joint limits (6,) for normalisation; v_scale: joint speed (6,) at |action| = 1
    (URDF v_max x velocity scaling 0.7), used to normalise robot_qd (rad/s).
    """

    def __init__(self, q_lo=None, q_hi=None, v_scale=None, n_kp: int = DEFAULT_N_KP, self_dist: bool = False):
        if q_lo is None or q_hi is None or v_scale is None:
            lo, hi, vs = default_joint_scales()
            q_lo = lo if q_lo is None else q_lo
            q_hi = hi if q_hi is None else q_hi
            v_scale = vs if v_scale is None else v_scale
        self.q_lo = torch.as_tensor(np.asarray(q_lo), dtype=torch.float32)
        self.q_hi = torch.as_tensor(np.asarray(q_hi), dtype=torch.float32)
        self.v_scale = torch.as_tensor(np.asarray(v_scale), dtype=torch.float32)
        self.n_kp = n_kp
        self.self_dist = bool(self_dist)
        self.robot_dim = robot_dim(n_kp, self.self_dist)
        self.edge_dims = edge_dims(n_kp)

    def config(self) -> dict:
        d = dict(q_lo=self.q_lo.tolist(), q_hi=self.q_hi.tolist(), v_scale=self.v_scale.tolist(), n_kp=self.n_kp)
        if self.self_dist:
            d["self_dist"] = True
        return d

    def __call__(self, s: dict) -> HeteroData:
        return self.build(s)

    def from_obs(self, obs: dict, device="cpu") -> HeteroData:
        return self.build(obs_to_torch(obs, device))

    def build(self, s: dict) -> HeteroData:
        return self.to_hetero(self.dense(s))

    # ------------------------------------------------------------------ dense features
    def dense(self, s: dict) -> DenseGraph:
        B, R = s["robot_q"].shape[:2]
        dev = s["robot_q"].device
        rm = s["robot_mask"] > 0.5 if "robot_mask" in s else torch.ones(B, R, dtype=torch.bool, device=dev)
        tm, om = s["task_mask"] > 0.5, s["obst_mask"] > 0.5
        T, O = tm.shape[1], om.shape[1]
        K = s["robot_kp"].shape[2]
        assert K == self.n_kp, f"obs has {K} keypoints, GraphBuilder expects {self.n_kp}"
        base = s["robot_base"]
        Rinv = _rot_inv(base[..., 3])  # (B,R,3,3)
        bpos = base[..., :3]

        def to_local(x):   # (B, R, ..., 3) world points -> receiving robot's base frame
            extra = x.dim() - 3
            Ri = Rinv.view(B, R, *([1] * extra), 3, 3)
            bp = bpos.view(B, R, *([1] * extra), 3)
            return (Ri @ (x - bp).unsqueeze(-1)).squeeze(-1)

        def rot_local(v):  # (B, R, ..., 3) world directions -> receiving robot's base frame
            extra = v.dim() - 3
            return (Rinv.view(B, R, *([1] * extra), 3, 3) @ v.unsqueeze(-1)).squeeze(-1)

        def rot6_local(r6):
            return torch.cat([rot_local(r6[..., :3]), rot_local(r6[..., 3:6])], -1)

        q_lo, q_hi, v_scale = self.q_lo.to(dev), self.q_hi.to(dev), self.v_scale.to(dev)
        qn = (s["robot_q"] - (q_lo + q_hi) / 2) / ((q_hi - q_lo) / 2)
        qdn = s["robot_qd"] / v_scale
        tcp_w = s["robot_tcp_pos"]
        tcp_l = to_local(tcp_w)
        Rtcp_w = rot6_to_mat(s["robot_tcp_rot6"])           # (B,R,3,3)
        axis_w = -Rtcp_w[..., :, 0]                          # tool axis, world
        kp_w = s["robot_kp"]
        kp_l = to_local(kp_w)

        # self-proximity (link_3 vs tool contacts dominate): tool points vs arm segments
        n_arm, n_tool = kp_split(K)
        sd = _seg_dist(kp_w[:, :, n_arm:], kp_w[:, :, :n_arm - 1], kp_w[:, :, 1:n_arm]).flatten(2)
        self_feats = torch.cat([sd / POS_SCALE, torch.tanh(sd / 0.25)], -1)
        gm = s["robot_goal_mask"].unsqueeze(-1)               # (B,R,1)
        goal = s["robot_goal"]
        gpos_w, grot6_w = goal[..., :3], goal[..., 3:9]
        Rg_w = rot6_to_mat(grot6_w)
        e_w = gpos_w - tcp_w
        e_tcp = (Rtcp_w.transpose(-1, -2) @ e_w.unsqueeze(-1)).squeeze(-1)
        en = e_w.norm(dim=-1, keepdim=True)
        gaxis_w = -Rg_w[..., :, 0]
        gaxis_tcp = (Rtcp_w.transpose(-1, -2) @ gaxis_w.unsqueeze(-1)).squeeze(-1)
        ax_err = _angle(axis_w, gaxis_w).unsqueeze(-1)
        Rrel = Rtcp_w.transpose(-1, -2) @ Rg_w
        rel6 = torch.cat([Rrel[..., :, 0], Rrel[..., :, 1]], -1)
        goal_feats = torch.cat([
            to_local(gpos_w) / POS_SCALE, rot6_local(grot6_w),
            torch.tanh(e_tcp / FINE_POS), e_tcp / POS_SCALE,
            en / POS_SCALE, (torch.log10(en + 1e-3) + 1.5) / 1.5,
            gaxis_tcp, ax_err / np.pi, torch.tanh(ax_err / np.deg2rad(10.0)), rel6], -1) * gm
        clear = s["min_clearance"].view(B, 1, 1).expand(B, R, 1)
        zeros3 = torch.zeros_like(tcp_w)
        app_w = s.get("robot_goal_approach", zeros3)
        ent_w = s.get("robot_entry", zeros3)
        has_ent = (ent_w.norm(dim=-1, keepdim=True) > 0.5).float()
        ent_err = _angle(ent_w, app_w).unsqueeze(-1) * has_ent * gm
        robot_x = torch.cat([
            qn, qdn, tcp_l / POS_SCALE, rot6_local(s["robot_tcp_rot6"]), kp_l.flatten(2) / POS_SCALE,
            goal_feats, gm, s["robot_dwell"].unsqueeze(-1), torch.tanh(clear / CLEAR_SCALE),
            rot_local(app_w) * gm, rot_local(ent_w), ent_err / np.pi,
            s["time_left"].view(B, 1, 1).expand(B, R, 1) if "time_left" in s else torch.ones_like(clear),
            self_feats], -1)
        if self.self_dist:   # MuJoCo hull self-distances (env EnvConfig.self_dist); far if absent
            sdh = s["robot_self_dist"] if "robot_self_dist" in s else torch.full((B, R, 3), SELF_DIST_FAR, device=dev)
            robot_x = torch.cat([robot_x, torch.tanh(sdh / FINE_DIST), sdh / SELF_DIST_FAR], -1)

        # ---- tasks
        status = s["task_status"]
        owner = s["task_owner"]                               # (B,T,R)
        rank = s["task_rank"] if "task_rank" in s else 1 - owner.sum(-1).clamp(max=1) * (1 - status)
        task_x = torch.stack([status, 1 - status, owner.sum(-1).clamp(max=1), rank], -1)
        tpos_l = to_local(s["task_pos"].unsqueeze(1).expand(B, R, T, 3))
        trot6_l = rot6_local(s["task_rot6"].unsqueeze(1).expand(B, R, T, 6))
        tapp_l = rot_local(s["task_approach"].unsqueeze(1).expand(B, R, T, 3))
        diff = tpos_l - tcp_l.unsqueeze(2)
        dist = diff.norm(dim=-1, keepdim=True)
        taxis_w = -rot6_to_mat(s["task_rot6"])[..., :, 0]    # (B,T,3)
        t_ax_err = _angle(axis_w.unsqueeze(2).expand(B, R, T, 3), taxis_w.unsqueeze(1).expand(B, R, T, 3))
        own_recv = owner.transpose(1, 2)                      # (B,R,T): owned by receiving robot
        own_other = owner.sum(-1).clamp(max=1).unsqueeze(1) - own_recv
        is_goal = ((s["task_pos"].unsqueeze(1) - gpos_w.unsqueeze(2)).norm(dim=-1) < 1e-4).float() \
            * s["robot_goal_mask"].unsqueeze(-1) * (1 - status).unsqueeze(1)
        f_tr = torch.cat([tpos_l / POS_SCALE, trot6_l, tapp_l, diff / POS_SCALE, dist / POS_SCALE,
                          torch.tanh(dist / FINE_DIST), (t_ax_err / np.pi).unsqueeze(-1),
                          status.unsqueeze(1).expand(B, R, T).unsqueeze(-1),
                          own_recv.unsqueeze(-1), own_other.unsqueeze(-1), is_goal.unsqueeze(-1)], -1)

        # ---- obstacles (world AABBs)
        obst_x = s["obst_size"] / POS_SCALE
        ol = to_local(s["obst_pos"].unsqueeze(1).expand(B, R, O, 3))
        half_l = (Rinv.abs().unsqueeze(2) @ s["obst_size"].unsqueeze(1).unsqueeze(-1)).squeeze(-1)
        sdf = _box_sdf(kp_w.unsqueeze(2), s["obst_pos"][:, None, :, None], s["obst_size"][:, None, :, None])
        sdf_tcp = _box_sdf(tcp_w.unsqueeze(2), s["obst_pos"].unsqueeze(1), s["obst_size"].unsqueeze(1))
        f_or = torch.cat([ol / POS_SCALE, half_l / POS_SCALE, sdf / POS_SCALE, sdf_tcp.unsqueeze(-1) / POS_SCALE], -1)

        # ---- robot j -> robot i: [b, i, j]
        kp_j = to_local(kp_w.unsqueeze(1).expand(B, R, R, K, 3))
        tcp_j = to_local(tcp_w.unsqueeze(1).expand(B, R, R, 3))
        axis_j = rot_local(axis_w.unsqueeze(1).expand(B, R, R, 3))
        goal_j = to_local(gpos_w.unsqueeze(1).expand(B, R, R, 3)) * s["robot_goal_mask"][:, None, :, None]
        pair = (kp_w.unsqueeze(2).unsqueeze(4) - kp_w.unsqueeze(1).unsqueeze(3)).norm(dim=-1)  # (B,i,j,Ki,Kj)
        pmin = pair.flatten(3).min(-1).values.unsqueeze(-1)
        f_rr = torch.cat([kp_j.flatten(3) / POS_SCALE, tcp_j / POS_SCALE, axis_j, goal_j / POS_SCALE,
                          s["robot_goal_mask"].unsqueeze(1).expand(B, R, R).unsqueeze(-1),
                          pair.min(-1).values / POS_SCALE, pair.min(-2).values / POS_SCALE,
                          torch.tanh(pmin / CLEAR_SCALE), torch.tanh(clear.unsqueeze(2).expand(B, R, R, 1) / CLEAR_SCALE)], -1)

        m_tr = rm.unsqueeze(2) & tm.unsqueeze(1)
        eye = torch.eye(R, dtype=torch.bool, device=dev).unsqueeze(0)
        return DenseGraph(
            x={"robot": robot_x, "task": task_x, "obstacle": obst_x},
            mask={"robot": rm, "task": tm, "obstacle": om},
            e={("task", "to", "robot"): f_tr, ("robot", "to", "task"): f_tr.transpose(1, 2),
               ("obstacle", "to", "robot"): f_or, ("robot", "to", "robot"): f_rr},
            emask={("task", "to", "robot"): m_tr, ("robot", "to", "task"): m_tr.transpose(1, 2),
                   ("obstacle", "to", "robot"): rm.unsqueeze(2) & om.unsqueeze(1),
                   ("robot", "to", "robot"): rm.unsqueeze(2) & rm.unsqueeze(1) & ~eye},
        )

    @staticmethod
    def to_hetero(dg: DenseGraph) -> HeteroData:
        B = dg.num_graphs
        dev = dg.robot_mask.device
        gid = {t: (m.flatten().cumsum(0) - 1).view(m.shape) for t, m in dg.mask.items()}
        data = HeteroData()
        for t, m in dg.mask.items():
            data[t].x = dg.x[t][m]
            data[t].batch = torch.arange(B, device=dev).unsqueeze(1).expand(m.shape)[m]
        for k, em in dg.emask.items():
            src, _, dst = k
            b, i, j = em.nonzero(as_tuple=True)  # i: receiver, j: sender
            data[k].edge_index = torch.stack([gid[src][b, j], gid[dst][b, i]])
            data[k].edge_attr = dg.e[k][b, i, j]
        data.num_graphs = B
        data.robot_mask = dg.robot_mask
        return data


def default_joint_scales(cell_id: str = "welding_cell_2_small"):
    """(q_lo, q_hi, v_scale) from the cell (env.cell: the URDF limits, robotmodel). Shape (6,) when every robot
    has the same limits (the shipped cells: unchanged), else (R, 6) per robot in ROBOT_IDS order (broadcasts
    against the (B, R, 6) joint observations), so a per-robot override reaches the GNN normalisation. No silent
    fallback: a missing cell raises (P3U fix; the old except-branch returned hard-coded KR240 numbers)."""
    from env.cell import ROBOT_IDS, load_cell
    c = load_cell(cell_id)
    lo = np.stack([c.robots[r].q_lo for r in ROBOT_IDS])
    hi = np.stack([c.robots[r].q_hi for r in ROBOT_IDS])
    vs = np.stack([c.robots[r].v_max for r in ROBOT_IDS]) * c.velocity_scale
    if all(np.array_equal(x[0], x[i]) for x in (lo, hi, vs) for i in range(1, len(ROBOT_IDS))):
        return lo[0], hi[0], vs[0]
    return lo, hi, vs
