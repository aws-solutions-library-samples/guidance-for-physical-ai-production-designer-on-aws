"""Per-rank env group: steps `n_envs` KukaWeldEnvs in lockstep with the rank's own actor copy.

Every PPO rank (process) owns a few envs (DD-PPO style): it collects a `T`-step segment with the current
parameters, then takes part in the data-parallel update. The first `n_det` envs of a rank are
deterministic evaluation slots (policy mean, current stage only, not trained on).

Small-batch inference on aarch64 torch: oneDNN costs ~0.26 ms per Linear call at batch 16 (vs 0.02 ms for
the native kernels), so the rollout runs with `torch.backends.mkldnn.flags(enabled=False)`; the update
(large batches, oneDNN 5x faster) keeps it on.
"""
from __future__ import annotations

import time

import numpy as np
import torch

from env.cell import ROBOT_IDS
from rl.action_map import ActionMap

from .features import FeatureBuilder
from .stages import StageCfg, reset_options


def make_env(env_kwargs: dict):
    from rl.vec_env import make_env as _mk
    return _mk(env_kwargs)


def idle_joint_actions(a, u, ob, amap):
    """v2: arms without a goal weld (homing / idle) are driven in joint space, a = g(u) (power map), instead of
    through the TCP twist map: homing is a joint-space target (wound wrist joints cannot be unwound by twists).
    Exception: within 10 cm of a weld the twist map is kept (lift-off)."""
    idle = np.asarray(ob["robot_goal_mask"]) < 0.5
    # near the panel (TCP within 10 cm of any episode weld) idle arms keep the TCP twist map, so they can lift
    # straight off a finished weld; joint space only once clear
    tp, tm = np.asarray(ob["task_pos"]), np.asarray(ob["task_mask"]) > 0.5           # (B,T,3), (B,T)
    d = np.linalg.norm(np.asarray(ob["robot_tcp_pos"])[:, :, None] - tp[:, None], axis=-1)  # (B,2,T)
    near = (np.where(tm[:, None], d, np.inf).min(-1) < 0.10)
    idle = idle & ~near
    if idle.any():
        a = np.array(a, copy=True)
        a[idle] = amap._g(np.clip(np.asarray(u, np.float64), -1, 1))[idle]
    return a


def _fine_phi(o):
    """(R,) precision bump exp(-(pos / 1 cm + tool-axis err / 5 deg)) per robot with a goal (0 otherwise)."""
    em = np.asarray(o["robot_goal_errmag"], np.float64)
    return np.exp(-(em[..., 0] / 0.01 + em[..., 1] / np.deg2rad(5.0))) * (np.asarray(o["robot_goal_mask"]) > 0.5)


def _below_phi(o, lateral=0.15, margin=0.01):
    """-sum over robots with a goal of max(0, z_weld + margin - z_tcp) within `lateral` (xy) of the weld."""
    g = np.asarray(o["robot_goal"])[..., :3]
    p = np.asarray(o["robot_tcp_pos"])
    lat = np.linalg.norm(p[..., :2] - g[..., :2], axis=-1)
    depth = np.maximum(0.0, g[..., 2] + margin - p[..., 2]) * (lat < lateral) * (np.asarray(o["robot_goal_mask"]) > 0.5)
    return -float(depth.sum())


class RankEnvs:
    def __init__(self, env_kwargs: dict, n_envs: int, n_det: int, seed: int, stage_cfg: StageCfg,
                 feat: FeatureBuilder, amap: ActionMap, stage: int = 0, idle_home_coef: float = 0.0,
                 accel_limit: float = 0.0, motion: dict | None = None, zone_coef: float = 0.0,
                 bootstrap_trunc: bool = False, below_coef: float = 0.0, fine_coef: float = 0.0,
                 home_fine: float = 0.0, idle_joint: bool = False):
        self.envs = [make_env(env_kwargs) for _ in range(n_envs)]
        self.n, self.n_det = n_envs, n_det
        self.rngs = [np.random.default_rng([seed, i]) for i in range(n_envs)]
        self.gen = torch.Generator().manual_seed(int(seed) * 7919 + 1)
        self.scfg = stage_cfg
        self.feat = feat
        self.amap = amap
        self.stage = stage
        # potential shaping for arms without a goal weld: phi = -coef * |q - HOME| (rad); lets an idle arm learn
        # to clear the cell instead of parking at its last weld (where it can block the other arm)
        self.idle_home_coef = float(idle_home_coef)
        from .accel import AccelLimit
        self.accel = AccelLimit(accel_limit, self.envs[0].cfg.cell_id, self.envs[0].dt) if accel_limit > 0 else None
        # v2: umarell planning rules as a stateful motion filter (rl/ppo/accel.MotionFilter) + penalty on the
        # transient excess over its approach-zone speed caps; truncation bootstrapping (no time_left in the obs)
        from .accel import MotionFilter
        self.motion = MotionFilter(n_envs, self.envs[0].cfg.cell_id, self.envs[0].dt, **motion) if motion else None
        self.zone_coef = float(zone_coef)
        # v2: potential shaping against being below the current goal's weld plane near the weld (approach from above)
        self.below_coef = float(below_coef)
        self.wp_soft_coef = 0.5
        self.fine_coef = float(fine_coef)
        self.home_fine = float(home_fine)
        self.idle_joint = bool(idle_joint)
        self.bootstrap_trunc = bool(bootstrap_trunc)
        self.home = self.envs[0].home.copy()
        self.obs = [None] * n_envs
        self.ep = [None] * n_envs
        for i in range(n_envs):
            self._reset(i)

    def _reset(self, i):
        env, rng = self.envs[i], self.rngs[i]
        opts = reset_options(env, self.stage, self.scfg, rng, deterministic=i < self.n_det)
        obs, info = env.reset(seed=int(rng.integers(2 ** 31)), options=opts)
        self.obs[i] = obs
        if getattr(self, "motion", None) is not None:
            self.motion.reset(i)
        self.ep[i] = dict(ret=0.0, len=0, stage=int(getattr(env, "_curriculum_stage", self.stage)),
                          kind=getattr(env, "_ppo_kind", "?"), det=i < self.n_det, min_clr=10.0, min_self=1.0,
                          n_welds=int(info["n_welds"]))

    def set_stage(self, stage):
        self.stage = stage
        for i in range(self.n):
            self._reset(i)

    def _batch(self):
        b = {k: np.stack([o[k] for o in self.obs]) for k in self.obs[0]}
        if self.motion is not None:
            b["robot_extra"] = self.motion.features(b)
        return b

    @torch.no_grad()
    def collect(self, model, T: int) -> tuple[dict, list]:
        E = self.n
        feats = None
        U = np.zeros((T, E, 2, 6), np.float32)
        LP = np.zeros((T, E), np.float32)
        REW = np.zeros((T, E), np.float32)
        DONE = np.zeros((T, E), np.float32)
        episodes = []
        boot = []
        t_env = t_pol = 0.0
        with torch.backends.mkldnn.flags(enabled=False):
            for t in range(T):
                t0 = time.perf_counter()
                ob = self._batch()
                f = self.feat(ob)
                if feats is None:
                    feats = np.zeros((T, E, f.shape[1]), np.float32)
                feats[t] = f
                ft = torch.from_numpy(f)
                u, lp, mean = model.act(ft, generator=self.gen)
                if self.n_det:
                    gm = ft.new_tensor(ob["robot_goal_mask"][:self.n_det])
                    u[:self.n_det] = mean[:self.n_det] * (1.0 if model.control_idle else gm.unsqueeze(-1))
                u = u.numpy()
                U[t], LP[t] = u, lp.numpy()
                a = self.amap.to_env(np.clip(u, -1, 1), ob)
                if self.idle_joint:
                    a = idle_joint_actions(a, u, ob, self.amap)
                if self.accel is not None:
                    a = self.accel(a, ob)
                if self.motion is not None:
                    zone = self.motion.zone(ob)
                    a = self.motion(a, ob)
                t1 = time.perf_counter()
                t_pol += t1 - t0
                for i, env in enumerate(self.envs):
                    q_prev, gm_prev = ob["robot_q"][i], ob["robot_goal_mask"][i]
                    obs, r, term, trunc, info = env.step(a[i].reshape(-1))
                    if self.idle_home_coef:
                        idle = (gm_prev < 0.5) & (obs["robot_goal_mask"] < 0.5)
                        dphi = np.linalg.norm(q_prev - self.home, axis=-1) - np.linalg.norm(obs["robot_q"] - self.home, axis=-1)
                        r += self.idle_home_coef * float((dphi * idle).sum())
                        if self.home_fine:   # precision bump at HOME (EnvConfig.home_tol is 0.02 rad)
                            f0 = np.exp(-np.abs(q_prev - self.home).max(-1) / 0.05)
                            f1 = np.exp(-np.abs(obs["robot_q"] - self.home).max(-1) / 0.05)
                            r += self.home_fine * float(((f1 - f0) * idle).sum())
                    e = self.ep[i]
                    e["ret"] += r
                    e["len"] += 1
                    e["min_clr"] = min(e["min_clr"], float(info["min_clearance"]))
                    if info.get("self_dist") is not None:
                        e["min_self"] = min(e["min_self"], float(np.min(info["self_dist"])))
                    if info.get("workpiece_depth"):
                        r -= self.wp_soft_coef * float(info["workpiece_depth"]) / 0.01   # per cm of penetration
                    if self.fine_coef:   # potential shaping near the goal; skipped where the goal changed
                        o0 = {k: v[i] for k, v in ob.items()}
                        same = np.all(np.abs(o0["robot_goal"] - obs["robot_goal"]) < 1e-6, -1)
                        r += self.fine_coef * float(((_fine_phi(obs) - _fine_phi(o0)) * same).sum())
                    if self.below_coef:
                        r += self.below_coef * float(_below_phi(obs) - _below_phi({k: v[i] for k, v in ob.items()}))
                    if self.motion is not None and self.zone_coef and zone[i].any():
                        mc = self.motion.cfg
                        vj = np.abs(obs["robot_qd"]) / self.motion.vmax                     # (2,6) fraction of v_max
                        vt = np.linalg.norm(obs["robot_tcp_pos"] - ob["robot_tcp_pos"][i], axis=-1) / env.dt
                        ex = (np.maximum(vj - mc["approach_v"], 0).sum(-1) / mc["approach_v"]
                              + np.maximum(vt - mc["zone_tcp"], 0) / mc["zone_tcp"])
                        r -= self.zone_coef * float((ex * zone[i]).sum())
                        e = self.ep[i]
                        e["zone_excess"] = max(e.get("zone_excess", 0.0), float((ex * zone[i]).max()))
                    REW[t, i] = r
                    if trunc and not term and self.bootstrap_trunc and i >= self.n_det:
                        fb = {k: v[None] for k, v in obs.items()}
                        if self.motion is not None:
                            fb["robot_extra"] = self.motion.features(fb, idx=[i])
                        boot.append((t, i, self.feat(fb)[0]))
                    if term or trunc:
                        DONE[t, i] = 1.0
                        e.update(success=bool(info["success"]) and not info["collision"],
                                 collision=bool(info["collision"]), collision_kind=info["collision_kind"],
                                 welds_done=int(info["welds_done"]), n_welds=int(info["n_welds"]),
                                 time_s=float(info["time_s"]), truncated=bool(trunc and not term),
                                 t_last_weld=max([w[3] for w in env.weld_events], default=0) * env.dt)
                        episodes.append(e)
                        self._reset(i)
                    else:
                        self.obs[i] = obs
                t_env += time.perf_counter() - t1
            last = self.feat(self._batch())
        out = dict(feat=feats, u=U, logp=LP, rew=REW, done=DONE, last_feat=last, boot=boot,
                   train_mask=(np.arange(E) >= self.n_det).astype(np.float32), t_env=t_env, t_pol=t_pol)
        return out, episodes
