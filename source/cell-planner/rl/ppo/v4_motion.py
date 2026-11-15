"""PPO v4 motion filter: rl.ppo.accel.MotionFilter plus a truly stationary weld hold (docs/p3_ppo4.md).

Two rules are added to the v3 filter (umarell's planning rules as part of the action parameterisation; the filter
never looks at collisions):

  * `still_hold`: while an arm dwells on its weld (obs robot_dwell > 0 and it has a goal), its velocity target is 0
    instead of v3's 1 mm/s TCP cap. The stop is reached under the same acceleration / jerk bounds as any other
    target (from the arrival speed of <= 0.1 m/s this takes 1-2 steps), after which the joints are exactly constant.
    v3 / PPO 2t crept at the dwell cap and were never still to 1e-6 rad, so the fair-speed retimer
    (docs/fair_speed.md) charged them a 7.5 s hold deficit.
  * `hold_steps` (k): after a weld completes (the arm's goal changes), the arm is kept still until it has been
    exactly still for k consecutive control steps (k = 10: 0.5 s, umarell's executor hold), counting the still
    steps of the dwell. The weld's still run around its completion instant is then >= 0.5 s.

State per env: previous commanded acceleration (as v3), previous goal (completion detection) and the count of
consecutive exactly-still steps per arm. Everything is computed from the observation and this state.
"""
from __future__ import annotations

import numpy as np

from .accel import MotionFilter


class MotionFilter4(MotionFilter):
    def __init__(self, n_envs: int, cell_id: str = "welding_cell_2_small", dt: float = 0.05, *,
                 still_hold: bool = True, hold_steps: int = 10, velocity_scale: float | None = None,
                 near_any: bool = False, **kw):
        super().__init__(n_envs, cell_id, dt, **kw)
        self.still_hold = bool(still_hold)
        self.hold_steps = int(hold_steps)
        # full-speed variant (docs/p3_ppo4.md §2.4): the env's velocity scale (EnvConfig.velocity_scale = 1.0: a = 1 is
        # the cell's v_max, read from env.cell) instead of the cell's planning scale 0.7; acceleration / jerk bounds are
        # re-derived for it (alpha * a_max, jerk * a_max / s, a_max from env.cell = robots.yaml)
        self.velocity_scale = velocity_scale
        if velocity_scale is not None:
            c = self.cfg
            self.vs = self.vmax * float(velocity_scale)
            self.astep = c["alpha"] * self.amax * dt / self.vs
            self.astep_home = c["home_a"] * self.amax * dt / self.vs
            self.jstep = c["jerk"] * self.amax * dt * dt / self.vs
        # near_any: the approach-zone caps (0.28 v_max joint, zone_tcp m/s TCP) hold within zone_r of ANY episode weld
        # (final approach, hold, lift-off / departure), not only near the arm's own goal
        self.near_any = bool(near_any)
        self.cfg.update(still_hold=self.still_hold, hold_steps=self.hold_steps, velocity_scale=velocity_scale,
                        near_any=self.near_any)
        self.prev_goal = np.full((n_envs, 2, 3), np.nan)
        self.prev_mask = np.zeros((n_envs, 2), bool)
        self.still = np.zeros((n_envs, 2), np.int64)
        self.freeze = np.zeros((n_envs, 2), bool)

    def reset(self, i=None):
        super().reset(i)
        sl = slice(None) if i is None else i
        self.prev_goal[sl] = np.nan
        self.prev_mask[sl] = False
        self.still[sl] = 0
        self.freeze[sl] = False

    def __call__(self, a, obs, idx=None):
        a = np.asarray(a, np.float64).copy()
        B = a.shape[0]
        idx = np.arange(B) if idx is None else np.asarray(idx)
        gm = np.asarray(obs["robot_goal_mask"]) > 0.5
        goal = np.asarray(obs["robot_goal"], np.float64)[..., :3]
        qd = np.asarray(obs["robot_qd"], np.float64)
        dwell = (np.asarray(obs["robot_dwell"]) > 0) & gm
        # exactly-still bookkeeping (the env integrates q += a v dt: a = 0 gives qd == 0 exactly)
        still_now = np.all(qd == 0.0, axis=-1)                                          # (B,2)
        self.still[idx] = np.where(still_now, self.still[idx] + 1, 0)
        pg, pm = self.prev_goal[idx], self.prev_mask[idx]
        known = ~np.isnan(pg[..., 0])
        changed = known & pm & (~gm | (np.linalg.norm(goal - np.nan_to_num(pg), axis=-1) > 1e-9))
        fr = (self.freeze[idx] | changed) & (self.still[idx] < self.hold_steps) if self.hold_steps > 0 \
            else np.zeros_like(changed)
        self.freeze[idx] = fr
        self.prev_goal[idx] = np.where(gm[..., None], goal, np.nan)
        self.prev_mask[idx] = gm
        stop = fr | (dwell if self.still_hold else False)
        if np.any(stop):
            a[stop] = 0.0              # velocity target 0; the acceleration / jerk bounds below shape the stop
        out = super().__call__(a, obs, idx=idx)
        return out

    def zone(self, obs):
        z = super().zone(obs)
        if not self.near_any:
            return z
        return z | (near_weld_dist(obs) < self.cfg["zone_r"])

    def features(self, obs, idx=None):
        """v3 features [acc_prev / astep (6), in zone, idle]; for arms WITHOUT a goal the zone slot instead flags
        "TCP within 10 cm of any episode weld" (where rl.ppo.rollout.idle_joint_actions keeps the twist map for the
        lift-off; rl/ppo/v4_model.py routes those arms to the v3 head)."""
        f = super().features(obs, idx)
        gm = np.asarray(obs["robot_goal_mask"]) > 0.5
        if np.all(gm):
            return f
        tp, tm = np.asarray(obs["task_pos"]), np.asarray(obs["task_mask"]) > 0.5
        d = np.linalg.norm(np.asarray(obs["robot_tcp_pos"])[:, :, None] - tp[:, None], axis=-1)
        near = np.where(tm[:, None], d, np.inf).min(-1) < 0.10
        f[..., 6] = np.where(gm, f[..., 6], near.astype(np.float32))
        return f

    def state_dict(self):
        return dict(acc=self.acc.copy(), prev_goal=self.prev_goal.copy(), prev_mask=self.prev_mask.copy(),
                    still=self.still.copy(), freeze=self.freeze.copy())


def idle_actions_v4(a, u, ob, amap, near_r: float = 0.10):
    """Goal-less arms (v4): within `near_r` of any episode weld the TCP twist map of the action map is kept (lift-off
    along the tool axis; as rl.ppo.rollout.idle_joint_actions), elsewhere the output is a joint velocity with a LINEAR
    map a = u (v3's idle_joint_actions uses a = sign(u) u^2, which makes a zero-initialised head ~u-times slower)."""
    idle = np.asarray(ob["robot_goal_mask"]) < 0.5
    if not idle.any():
        return a
    tp, tm = np.asarray(ob["task_pos"]), np.asarray(ob["task_mask"]) > 0.5
    d = np.linalg.norm(np.asarray(ob["robot_tcp_pos"])[:, :, None] - tp[:, None], axis=-1)
    near = np.where(tm[:, None], d, np.inf).min(-1) < near_r
    far = idle & ~near
    if far.any():
        a = np.array(a, copy=True)
        a[far] = np.clip(np.asarray(u, np.float64), -1, 1)[far]
    return a


def near_weld_dist(ob) -> np.ndarray:
    """(B, 2) TCP distance to the nearest episode weld."""
    tp, tm = np.asarray(ob["task_pos"]), np.asarray(ob["task_mask"]) > 0.5
    d = np.linalg.norm(np.asarray(ob["robot_tcp_pos"])[:, :, None] - tp[:, None], axis=-1)
    return np.where(tm[:, None], d, np.inf).min(-1)


def motion_kwargs_from_args(args: dict) -> dict:
    """v4 filter options stored in a checkpoint's args (train_ppo_v4.py)."""
    kw = dict(still_hold=bool(args.get("v4_still_hold", 1)), hold_steps=int(args.get("v4_hold_steps", 10)))
    if args.get("v4_vscale"):
        kw.update(velocity_scale=float(args["v4_vscale"]), near_any=bool(args.get("v4_near_any", 0)),
                  home_v=float(args.get("v4_home_v", 0.4)), home_a=float(args.get("v4_home_a", 0.16)))
    return kw


def apply_vscale(amap, args: dict):
    """Action map consistent with the env's velocity scale (full-speed variant): ActionMap normalises joint velocities
    by the cell's planning scale (0.7); with EnvConfig.velocity_scale = v4_vscale it must use that scale instead."""
    if args.get("v4_vscale"):
        amap._setup()
        amap._vscale = amap._vscale / amap_cell_scale(amap) * float(args["v4_vscale"])
    return amap


def amap_cell_scale(amap) -> float:
    from env.cell import load_cell
    return float(load_cell(amap.cell_id).velocity_scale)
