"""Agent-side action parameterisation: policy output u (B, 2, 6) in [-1, 1] -> env action a (B, 2, 6).

The env always receives normalised joint velocities a (q += a * v_max * 0.7 * dt). The map is a
fixed, state-dependent function a = f(u, s), so the MDP seen by TD3 over u stays Markov. The
replay buffer stores u (what the actor produced), and the critic learns Q(s, u).

kind="joint": a = g(u), with g(u) = sign(u) |u|^power (power > 1 gives fine resolution near zero).
kind="twist": the 6 outputs are a TCP twist [v (3), w (3)] in the robot's base frame,
    v = g(u[:3]) * v_lin, w = g(u[3:]) * v_ang, mapped to joint velocities by damped least squares
    qd = J^T (J J^T + lambda^2 I)^-1 [v; w] (J = world-frame TCP Jacobian, env.kinematics). The
    result is scaled down uniformly if any joint would exceed its limit. For a 6-DoF arm this is a
    reparameterisation of joint velocity (it is exact away from singularities), and it makes
    "goal error in the base/TCP frame -> action" close to linear.
idle_zero: robots without a goal weld (robot_goal_mask = 0) get a = 0 (and the stored u is
    zeroed too), so an idle arm parks instead of drifting into a self-collision.

`from_env(a, obs)` is the inverse used to convert demonstration (env) actions to u. Values
outside the range are clipped and counted (`clip_frac`).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


@dataclass
class ActionMapConfig:
    kind: str = "twist"        # "joint" | "twist" (twist: best in the stage-0 BC comparison, docs/training.md)
    power: float = 2.0         # g(u) = sign(u)|u|^power
    v_lin: float = 0.6         # m/s at |u| = 1 (twist)
    v_ang: float = 1.2         # rad/s at |u| = 1 (twist)
    damping: float = 0.03      # DLS lambda (twist)
    idle_zero: bool = True
    # Track G: idle arms (no goal) return HOME on a straight joint-space line instead of freezing
    # (|a| <= idle_home_speed, proportional gain idle_home_gain per step). Needs idle_zero (the stored u stays 0).
    # Reason: the expert parks idle arms at HOME; frozen arms parked at their last weld make the other arm's
    # cloned "wait for the other arm" behaviour a fixed point (stage 2/3 stalls).
    idle_home: bool = False
    idle_home_speed: float = 0.5
    idle_home_gain: float = 0.3
    # Track G2: kind="hybrid" = "twist" for robots whose TCP is within switch_dist of their goal weld (precise
    # straight-line descent), "joint" otherwise (joint-space transits; the twist map cannot represent joint motion at
    # the wrist singularity of HOME). The switch is a fixed function of the observation (TCP pos, goal pos, mask).
    switch_dist: float = 0.2


class ActionMap:
    def __init__(self, cfg: ActionMapConfig | dict | None = None, cell_id: str = "welding_cell_2_small"):
        if isinstance(cfg, dict):
            cfg = ActionMapConfig(**cfg)
        self.cfg = cfg or ActionMapConfig()
        self.cell_id = cell_id
        self._kin = None
        self.clip_frac = 0.0

    def to_dict(self):
        return asdict(self.cfg)

    # ---------------------------------------------------------------- helpers
    def _g(self, u):
        p = self.cfg.power
        return u if p == 1.0 else np.sign(u) * np.abs(u) ** p

    def _ginv(self, a):
        p = self.cfg.power
        return a if p == 1.0 else np.sign(a) * np.abs(a) ** (1.0 / p)

    def _setup(self):
        if self._kin is None:
            from env.cell import ROBOT_IDS, load_cell
            from env.kinematics import ArmKinematics
            cell = load_cell(self.cell_id)
            self._kin = [ArmKinematics(cell.robots[r]) for r in ROBOT_IDS]
            self._vscale = np.stack([cell.robots[r].v_max for r in ROBOT_IDS]) * cell.velocity_scale  # (2,6)
            self._yaw = np.array([cell.robots[r].yaw for r in ROBOT_IDS])

    def _rot_base(self, i):
        c, s = np.cos(self._yaw[i]), np.sin(self._yaw[i])
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])

    @staticmethod
    def _active(obs, n):
        if "robot_goal_mask" in obs:
            return np.asarray(obs["robot_goal_mask"]).reshape(n, -1) > 0.5
        return np.ones((n, 2), bool)

    # ---------------------------------------------------------------- maps
    def near_mask(self, obs: dict, B: int) -> np.ndarray:
        """(B,2) bool: hybrid map uses the twist branch (TCP within switch_dist of the goal weld)."""
        tcp = np.asarray(obs["robot_tcp_pos"], np.float64).reshape(B, 2, 3)
        goal = np.asarray(obs["robot_goal"], np.float64).reshape(B, 2, -1)[..., :3]
        return (np.linalg.norm(goal - tcp, axis=-1) < self.cfg.switch_dist) & self._active(obs, B)

    def _sub(self, kind):
        if not hasattr(self, "_subs"):
            from dataclasses import replace
            self._subs = {k: ActionMap(replace(self.cfg, kind=k), self.cell_id) for k in ("joint", "twist")}
        return self._subs[kind]

    def to_env(self, u, obs: dict) -> np.ndarray:
        """u (B,2,6) policy output, obs batched dict (robot_q (B,2,6), robot_goal_mask (B,2))."""
        u = np.clip(np.asarray(u, np.float64), -1, 1)
        B = u.shape[0]
        if self.cfg.kind == "hybrid":
            near = self.near_mask(obs, B)[..., None]
            return np.where(near, self._sub("twist").to_env(u, obs), self._sub("joint").to_env(u, obs)).astype(np.float32)
        if self.cfg.kind == "joint":
            a = self._g(u)
        else:
            self._setup()
            q = np.asarray(obs["robot_q"], np.float64).reshape(B, 2, 6)
            a = np.zeros_like(u)
            for i in range(2):
                Rb = self._rot_base(i)
                tw = np.concatenate([self._g(u[:, i, :3]) * self.cfg.v_lin, self._g(u[:, i, 3:]) * self.cfg.v_ang], -1)
                tw_w = np.concatenate([tw[:, :3] @ Rb.T, tw[:, 3:] @ Rb.T], -1)          # base -> world
                J = self._kin[i].jacobian(q[:, i])                                          # (B,6,6)
                JJt = J @ J.transpose(0, 2, 1) + self.cfg.damping ** 2 * np.eye(6)
                qd = np.einsum("bji,bj->bi", J, np.linalg.solve(JJt, tw_w[..., None])[..., 0])
                ai = qd / self._vscale[i]
                m = np.abs(ai).max(-1, keepdims=True)
                a[:, i] = ai / np.maximum(m, 1.0)
        if self.cfg.idle_zero:
            act = self._active(obs, B)
            a = a * act[..., None]
            if getattr(self.cfg, "idle_home", False) and (~act).any():
                a = np.where(act[..., None], a, self._home_action(obs, B))
        return a.astype(np.float32)

    def _home_action(self, obs, B):
        """(B,2,6) joint-space P-step towards HOME, uniformly scaled to |a| <= idle_home_speed."""
        if not hasattr(self, "_home"):
            from env.cell import ROBOT_IDS, load_cell
            cell = load_cell(self.cell_id)
            self._home = np.stack([cell.robots[r].home for r in ROBOT_IDS])
            self._vstep = np.stack([cell.robots[r].v_max for r in ROBOT_IDS]) * cell.velocity_scale * 0.05
        q = np.asarray(obs["robot_q"], np.float64).reshape(B, 2, 6)
        a = self.cfg.idle_home_gain * (self._home - q) / self._vstep
        m = np.abs(a).max(-1, keepdims=True)
        return a / np.maximum(m / self.cfg.idle_home_speed, 1.0)

    def mask_u(self, u, obs: dict) -> np.ndarray:
        """The u that is stored in the replay buffer (idle robots zeroed)."""
        u = np.clip(np.asarray(u, np.float32), -1, 1)
        if self.cfg.idle_zero:
            u = u * self._active(obs, u.shape[0])[..., None]
        return u

    def from_env(self, a, obs: dict) -> np.ndarray:
        """Inverse map for demonstrations: env action a (B,2,6) -> u (B,2,6), clipped to [-1, 1]."""
        a = np.asarray(a, np.float64).reshape(-1, 2, 6)
        B = a.shape[0]
        if self.cfg.kind == "hybrid":
            near = self.near_mask(obs, B)[..., None]
            ut, tw = self._sub("twist").from_env(a, obs), self._sub("twist")
            uj = self._sub("joint").from_env(a, obs)
            self.clip_frac = float(tw.clip_frac * near.mean())
            return np.where(near, ut, uj).astype(np.float32)
        if self.cfg.kind == "joint":
            u = self._ginv(a)
        else:
            self._setup()
            q = np.asarray(obs["robot_q"], np.float64).reshape(B, 2, 6)
            u = np.zeros_like(a)
            for i in range(2):
                Rb = self._rot_base(i)
                J = self._kin[i].jacobian(q[:, i])
                tw_w = np.einsum("bij,bj->bi", J, a[:, i] * self._vscale[i])
                tw = np.concatenate([tw_w[:, :3] @ Rb, tw_w[:, 3:] @ Rb], -1)               # world -> base
                u[:, i] = np.concatenate([self._ginv(tw[:, :3] / self.cfg.v_lin), self._ginv(tw[:, 3:] / self.cfg.v_ang)], -1)
        self.clip_frac = float((np.abs(u) > 1).mean())
        u = np.clip(u, -1, 1)
        return self.mask_u(u, obs)

    def convert_episode(self, ep: dict) -> dict:
        """Demo episode with env actions -> same episode with policy-space actions u."""
        obs_t = {k: v[:-1] for k, v in ep["obs_seq"].items()}
        out = dict(ep)
        out["action"] = self.from_env(ep["action"], obs_t)
        if "bc_action" in ep:
            clip = self.clip_frac
            out["bc_action"] = self.from_env(ep["bc_action"], obs_t)
            self.clip_frac = clip
        return out
