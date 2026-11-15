"""Demonstration transitions: file format, recording, buffer seeding, and a small reach expert.

File format (`.npz`, written by `save_demos`), all episodes concatenated:
    obs.<key>   (sum(L_i + 1), ...)  observation rows: obs before each step + final obs, per episode
    action      (sum L_i, 2, 6)      normalised joint velocities in [-1, 1]
    reward, terminated, collision (sum L_i,)
    ep_len      (n_episodes,)        L_i
    meta        json string (source, cell, stage, ...)
Any controller can produce it: `record_demos(env, act_fn, n, reset_fn)` with act_fn(obs) -> (12,)
(e.g. Track X's ScriptedExpert(env).act, or `DLSReachExpert` below). Only obs keys in
`rl.her_buffer.STORE_KEYS` are kept.

`DLSReachExpert` (for curriculum stages 0/1, one active arm, other arm idle; no arm-arm
coordination): joint-space move to the nearest usable IK pre-weld solution (data/weld_ik.json),
damped-least-squares to the actual (noisy) pre-weld pose, straight-line DLS descent along the
approach vector to the weld, then hold still for the dwell. Orientation is servoed on the tool axis
only (umarell's gate; the spin about the axis is kept), matching the 5-DoF IK table.
"""
from __future__ import annotations

import json

import numpy as np

from .her_buffer import EpisodeRecorder, HERReplayBuffer, clean_obs


def save_demos(path, episodes: list[dict], meta: dict | None = None):
    keys = list(episodes[0]["obs_seq"])
    d = {f"obs.{k}": np.concatenate([e["obs_seq"][k] for e in episodes]) for k in keys}
    for k in ("action", "reward", "terminated", "collision"):
        d[k] = np.concatenate([np.asarray(e[k]) for e in episodes])
    d["action"] = d["action"].reshape(len(d["reward"]), 2, -1).astype(np.float32)
    if all("bc_action" in e for e in episodes):
        d["bc_action"] = np.concatenate([e["bc_action"] for e in episodes]).reshape(d["action"].shape).astype(np.float32)
    d["ep_len"] = np.array([len(e["reward"]) for e in episodes], np.int64)
    d["meta"] = np.array(json.dumps(meta or {}))
    np.savez_compressed(path, **d)


def load_demos(path) -> list[dict]:
    with np.load(path, allow_pickle=False) as f:   # read each array once (NpzFile[k] re-decompresses)
        z = {k: f[k] for k in f.files}
    keys = [k[4:] for k in z if k.startswith("obs.")]
    eps, r0, t0 = [], 0, 0
    for L in z["ep_len"]:
        L = int(L)
        eps.append(dict(obs_seq={k: z[f"obs.{k}"][r0:r0 + L + 1] for k in keys},
                        action=z["action"][t0:t0 + L], reward=z["reward"][t0:t0 + L],
                        terminated=z["terminated"][t0:t0 + L], collision=z["collision"][t0:t0 + L]))
        if "bc_action" in z:
            eps[-1]["bc_action"] = z["bc_action"][t0:t0 + L]
        r0 += L + 1
        t0 += L
    return eps


def fill_buffer(buf: HERReplayBuffer, paths) -> int:
    """Add all episodes of one or more demo files to `buf`. Returns the number of transitions."""
    n = 0
    for p in ([paths] if isinstance(paths, str) else paths):
        for ep in load_demos(p):
            buf.add_episode(**ep)
            n += len(ep["reward"])
    return n


def record_demos(env, act_fn, n_episodes: int, reset_fn, success_only: bool = False, noise: float = 0.0,
                 rng=None, max_tries: int | None = None, on_reset=None, label_clean: bool = False,
                 noise_near: float = 0.0):
    """Roll out `act_fn` (obs -> (12,)) for n episodes; reset_fn(env) -> (obs, info).
    noise: Gaussian std added to the executed env action. label_clean (DART, Laskey et al. 2017): each
    episode also gets `bc_action` = the controller's clean action (the BC target), while `action` is what
    was executed (for the critic). noise_near > 0: per-robot noise scaled by clip(goal distance / noise_near,
    0.03, 1) (as the training exploration), so the expert can still hold the 10 mm dwell. Returns (episodes, stats)."""
    rng = rng or np.random.default_rng(0)
    eps, stats = [], dict(tried=0, success=0, collision=0)
    max_tries = max_tries or 5 * n_episodes
    while len(eps) < n_episodes and stats["tried"] < max_tries:
        obs, info = reset_fn(env)
        if on_reset is not None:
            on_reset(env, obs)
        rec = EpisodeRecorder()
        rec.reset(obs)
        clean = []
        while True:
            a = np.asarray(act_fn(obs), np.float64).reshape(-1)
            clean.append(np.clip(a, -1, 1).reshape(2, -1))
            if noise > 0:
                sc = np.ones((2, 1))
                if noise_near > 0:
                    err = np.linalg.norm(obs["robot_goal"][:, :3] - obs["robot_tcp_pos"], axis=-1)
                    sc[:, 0] = np.where(obs["robot_goal_mask"] > 0.5, np.clip(err / noise_near, 0.03, 1.0), 1.0)
                a = a + noise * (rng.normal(size=(2, a.size // 2)) * sc).reshape(-1)
            a = np.clip(a, -1, 1)
            obs, r, term, trunc, info = env.step(a)
            rec.add(a.reshape(2, -1), r, term, info["collision"], obs)
            if term or trunc:
                break
        stats["tried"] += 1
        stats["success"] += bool(info["success"])
        stats["collision"] += bool(info["collision"])
        if success_only and not info["success"]:
            continue
        ep = rec.flush()
        if label_clean:
            ep["bc_action"] = np.stack(clean).astype(np.float32)
        eps.append(ep)
    return eps, stats


class DLSReachExpert:
    """Single-arm IK/DLS reach-and-dwell controller (see module docstring)."""

    def __init__(self, env, standoff: float = 0.10, descent_speed: float = 0.08, damping: float = 0.05,
                 joint_speed: float = 0.6, gain: float = 0.6, axis_only: bool = True):
        self.axis_only = axis_only
        from env.kinematics import ArmKinematics
        from env.cell import ROBOT_IDS
        from rl.curriculum import ik_table
        self.env = env
        self.kin = [ArmKinematics(env.cell.robots[r]) for r in ROBOT_IDS]
        self.rids = ROBOT_IDS
        self.tab = ik_table(env.cfg.cell_id)
        self.standoff, self.descent_speed, self.damping = standoff, descent_speed, damping
        self.joint_speed, self.gain = joint_speed, gain
        self.reset()

    def reset(self):
        self.phase = [None, None]
        self.goal_key = [None, None]
        self.s = [0.0, 0.0]
        self.q_pre = [None, None]

    def _dls(self, i, q, p_t, R_t, max_lin=0.1):
        p, Rm = self.kin[i].tcp(q)
        e_p = p_t - p
        n = np.linalg.norm(e_p)
        if n > max_lin:
            e_p = e_p * max_lin / n
        if self.axis_only:   # umarell gates the tool axis only: rotate the axis, keep the current spin
            ac, ag = -Rm[:, 0], -R_t[:, 0]
            cr = np.cross(ac, ag)
            sn = np.linalg.norm(cr)
            ang = np.arctan2(sn, float(ac @ ag))
            e_w = cr / sn * ang if sn > 1e-9 else np.zeros(3)
        else:
            from env.rewards import rotvec_from_mat
            e_w = rotvec_from_mat(R_t @ Rm.T)
        e = np.concatenate([e_p, 0.5 * e_w])
        J = self.kin[i].jacobian(q)
        dq = J.T @ np.linalg.solve(J @ J.T + self.damping ** 2 * np.eye(6), e)
        return dq, np.linalg.norm(p_t - p), np.linalg.norm(e_w)

    def act(self, obs) -> np.ndarray:
        env = self.env
        a = np.zeros((2, 6))
        vstep = env.v_max * env.dt                     # rad per step at |a| = 1
        for i in range(2):
            if obs["robot_goal_mask"][i] < 0.5:
                continue
            g = obs["robot_goal"][i].astype(np.float64)
            p_w = g[:3]
            from env.rewards import rot6_to_mat
            R_w = rot6_to_mat(g[3:9])
            app = obs["robot_goal_approach"][i].astype(np.float64)
            app = app / max(np.linalg.norm(app), 1e-9)
            key = tuple(np.round(p_w, 5))
            q = env.q[i]
            if key != self.goal_key[i]:
                self.goal_key[i] = key
                self.phase[i] = "joint"
                wid = env.task_ids[env._goal_idx(i)]
                ent = self.tab[self.rids[i]].get(wid)
                pre = ent["pre"] if ent and ent["pre"] else (ent["q"] if ent else [])
                self.q_pre[i] = min(pre, key=lambda s: np.abs(s - q).sum()) if pre else None
                if self.q_pre[i] is None:
                    self.phase[i] = "pre"
            p_pre = p_w - self.standoff * app
            if self.phase[i] == "joint":
                d = self.q_pre[i] - q
                if np.abs(d).max() < 0.02:
                    self.phase[i] = "pre"
                else:
                    a[i] = np.clip(d / vstep[i], -self.joint_speed, self.joint_speed)
                    continue
            if self.phase[i] == "pre":
                dq, ep, er = self._dls(i, q, p_pre, R_w)
                if ep < 0.002 and er < np.deg2rad(0.5):  # er = tool-axis error (axis_only)
                    self.phase[i] = "descend"
                    self.s[i] = 0.0
                else:
                    a[i] = np.clip(self.gain * dq / vstep[i], -self.joint_speed, self.joint_speed)
                    continue
            if self.phase[i] == "descend":
                self.s[i] = min(self.standoff, self.s[i] + self.descent_speed * env.dt)
                tgt = p_pre + self.s[i] * app
                dq, ep, er = self._dls(i, q, tgt, R_w)
                a[i] = np.clip(dq / vstep[i], -1, 1)
                if self.s[i] >= self.standoff and ep < 0.001:
                    self.phase[i] = "hold"
                continue
            # hold: tiny corrections only if drifting out of tolerance
            a[i] = 0.0
        return a.reshape(-1)


# ---------------------------------------------------------------------------- parallel recording (Track G)
def _record_job(job):
    """Worker: (env_kwargs, kind, spec_dict, n, noise, seed) -> (episodes, stats). Top level for spawn."""
    import dataclasses
    import os
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    from rl.curriculum import StageSpec, reset_env
    from rl.vec_env import make_env
    env_kwargs, kind, spec_d, n, noise, seed = job[:6]
    noise_near = job[6] if len(job) > 6 else 0.15
    env = make_env(env_kwargs)
    spec = StageSpec(**spec_d)
    rng = np.random.default_rng(seed)
    if kind == "scripted":
        from evaluate import make_expert
        ex = make_expert(env)
    else:
        ex = DLSReachExpert(env)
    return record_demos(env, ex.act, n, lambda e: reset_env(e, spec, rng), success_only=True,
                        noise=noise, rng=rng, on_reset=lambda e, o: ex.reset(), label_clean=noise > 0,
                        noise_near=noise_near, max_tries=20 * n)


def record_demos_parallel(env_kwargs: dict, kind: str, spec, n: int, noise: float = 0.0, seed: int = 0,
                          n_proc: int = 4, noise_near: float = 0.15):
    """Record n successful episodes of curriculum `spec` with `n_proc` processes (spawn)."""
    import multiprocessing as mp
    from dataclasses import asdict
    n_proc = max(1, min(n_proc, n))
    per = [n // n_proc + (i < n % n_proc) for i in range(n_proc)]
    jobs = [(env_kwargs, kind, asdict(spec), k, noise, seed * 1000 + i, noise_near) for i, k in enumerate(per) if k > 0]
    if len(jobs) == 1:
        outs = [_record_job(jobs[0])]
    else:
        with mp.get_context("spawn").Pool(len(jobs)) as pool:
            outs = pool.map(_record_job, jobs)
    eps, stats = [], dict(tried=0, success=0, collision=0)
    for e, s in outs:
        eps += e
        for k in stats:
            stats[k] += s[k]
    return eps, stats
