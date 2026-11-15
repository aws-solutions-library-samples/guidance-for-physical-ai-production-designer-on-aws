"""Joint acceleration limit as part of the action parameterisation (fixed, state-dependent; not a safety filter).

The env is kinematic: any normalised joint velocity a in [-1, 1] is applied instantly. Policies trained without a
limit exploit that (bang-bang velocity changes, peaks of several x the URDF acceleration limits), while the scripted
expert and umarell's TOPP-RA plans respect `alpha x a_max`. With `AccelLimit(alpha)` the env action is clipped to
    |a - a_prev| <= alpha * a_max * dt / (v_max * velocity_scale),   a_prev = robot_qd / (v_max * velocity_scale)
where robot_qd (in the observation, so the MDP stays Markov) is the velocity of the previous control step. The
policy sees the limit through the obs (qd) and the reward; it never looks at collisions.
"""
from __future__ import annotations

import numpy as np


class AccelLimit:
    def __init__(self, alpha: float, cell_id: str = "welding_cell_2_small", dt: float = 0.05):
        from env.cell import ROBOT_IDS, load_cell
        c = load_cell(cell_id)
        self.alpha = float(alpha)
        vs = np.stack([c.robots[r].v_max for r in ROBOT_IDS]) * c.velocity_scale       # (2,6) rad/s at |a| = 1
        self.vs = vs
        self.da = self.alpha * np.stack([c.robots[r].a_max for r in ROBOT_IDS]) * dt / vs  # (2,6) max change of a

    def __call__(self, a: np.ndarray, obs: dict) -> np.ndarray:
        """a (B, 2, 6) env actions, obs batched (robot_qd (B, 2, 6))."""
        if self.alpha <= 0:
            return a
        a_prev = np.asarray(obs["robot_qd"], np.float64).reshape(a.shape) / self.vs
        return np.clip(a, a_prev - self.da, a_prev + self.da).clip(-1, 1).astype(np.float32)


class MotionFilter:
    """v2 action filter: umarell's planning rules as part of the action parameterisation (stateful per env).

    Per robot, on normalised joint velocities a (q_dot = a * v_max * velocity_scale):
      1. speed targets
         * idle / homing arm (no goal weld): |q_dot_j| <= home_v * v_max (umarell home_velocity_scale 0.4)
         * approach zone (TCP within `zone_r` of its goal weld): |q_dot_j| <= approach_v * v_max (umarell
           approach_velocity_fraction 0.4 x velocity scale 0.7 = 0.28) and TCP speed <= zone_tcp (m/s)
         * dwelling (pose inside tolerance, obs robot_dwell > 0): TCP speed <= dwell_tcp (a still hold)
      2. acceleration: |q_ddot_j| <= alpha * a_max (alpha = 0.49 = 0.7^2, umarell's vscale^2 rule; idle arms
         home_a = 0.16 = 0.4^2)
      3. jerk: |q_dddot_j| <= jerk * a_max per second
    Speed targets are applied first, then the acceleration and jerk bounds around the previous velocity, so the
    targets are reached as fast as the bounds allow. They can be exceeded transiently (e.g. entering the zone too
    fast). The rollout penalises that excess (rl/ppo/rollout.py), and evaluation reports it. Uses only the
    observation plus the previous commanded acceleration (`acc_prev`, also a policy feature), never collisions.
    """

    def __init__(self, n_envs: int, cell_id: str = "welding_cell_2_small", dt: float = 0.05, alpha: float = 0.49,
                 jerk: float = 5.0, home_v: float = 0.4, home_a: float = 0.16, approach_v: float = 0.28,
                 zone_r: float = 0.05, zone_tcp: float = 0.10, dwell_tcp: float = 0.01):
        from env.cell import ROBOT_IDS, load_cell
        from env.kinematics import ArmKinematics
        c = load_cell(cell_id)
        self.vmax = np.stack([c.robots[r].v_max for r in ROBOT_IDS])
        self.amax = np.stack([c.robots[r].a_max for r in ROBOT_IDS])
        self.vs = self.vmax * c.velocity_scale
        self.kin = [ArmKinematics(c.robots[r]) for r in ROBOT_IDS]
        self.dt = dt
        self.cfg = dict(alpha=alpha, jerk=jerk, home_v=home_v, home_a=home_a, approach_v=approach_v, zone_r=zone_r,
                        zone_tcp=zone_tcp, dwell_tcp=dwell_tcp)
        self.astep = alpha * self.amax * dt / self.vs          # (2,6) max change of a per step
        self.astep_home = home_a * self.amax * dt / self.vs
        self.jstep = jerk * self.amax * dt * dt / self.vs       # max change of the per-step change
        self.acc = np.zeros((n_envs, 2, 6))

    def reset(self, i=None):
        if i is None:
            self.acc[:] = 0
        else:
            self.acc[i] = 0

    def zone(self, obs):
        """(B,2) bool: TCP within zone_r of its goal weld."""
        gm = np.asarray(obs["robot_goal_mask"]) > 0.5
        d = np.linalg.norm(np.asarray(obs["robot_goal"])[..., :3] - np.asarray(obs["robot_tcp_pos"]), axis=-1)
        return gm & (d < self.cfg["zone_r"])

    def features(self, obs, idx=None):
        """(B,2,8) extra robot-node features: [acc_prev / astep (6), in zone, idle]."""
        acc = self.acc if idx is None else self.acc[idx]
        gm = np.asarray(obs["robot_goal_mask"]) > 0.5
        return np.concatenate([acc / self.astep, self.zone(obs)[..., None], (~gm)[..., None]], -1).astype(np.float32)

    def __call__(self, a, obs, idx=None):
        """a (B,2,6) env actions from the action map; obs batched; idx: env indices of the batch (state rows)."""
        c = self.cfg
        a = np.asarray(a, np.float64).copy()
        B = a.shape[0]
        idx = np.arange(B) if idx is None else np.asarray(idx)
        gm = np.asarray(obs["robot_goal_mask"]) > 0.5
        zone = self.zone(obs)
        dwell = np.asarray(obs["robot_dwell"]) > 0
        q = np.asarray(obs["robot_q"], np.float64)
        a_prev = np.asarray(obs["robot_qd"], np.float64) / self.vs
        for i in range(2):
            vcap = np.where(~gm[:, i], c["home_v"], np.where(zone[:, i], c["approach_v"], 1.0))[:, None] * self.vmax[i] / self.vs[i]
            a[:, i] = np.clip(a[:, i], -vcap, vcap)
            tcap = np.where(dwell[:, i] & gm[:, i], c["dwell_tcp"], np.where(zone[:, i], c["zone_tcp"], np.inf))
            m = np.isfinite(tcap)
            if m.any():
                J = self.kin[i].jacobian(q[m, i])[:, :3]                            # (b,3,6) world linear
                v = np.einsum("bij,bj->bi", J, a[m, i] * self.vs[i])
                sp = np.linalg.norm(v, axis=-1)
                a[m, i] *= np.minimum(1.0, tcap[m] / np.maximum(sp, 1e-9))[:, None]
            ast = np.where(gm[:, i, None], self.astep[i], self.astep_home[i])
            acc = a[:, i] - a_prev[:, i]
            acc_p = self.acc[idx, i]
            acc = np.clip(acc, acc_p - self.jstep[i], acc_p + self.jstep[i])
            acc = np.clip(acc, -ast, ast)
            a[:, i] = np.clip(a_prev[:, i] + acc, -1, 1)
            self.acc[idx, i] = a[:, i] - a_prev[:, i]
        return a.astype(np.float32)
