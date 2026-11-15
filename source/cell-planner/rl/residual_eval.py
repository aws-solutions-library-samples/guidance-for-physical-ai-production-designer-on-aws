"""Rollouts + metrics for the residual policy (shared by train_residual.py and evaluate_residual.py)."""
from __future__ import annotations

import numpy as np

from env.cell import ROBOT_IDS


def agent_policy(agent):
    def fn(obs):
        b = {k: np.asarray(v)[None] for k, v in obs.items() if isinstance(v, np.ndarray)}
        return agent.act(b, noise=0.0)[0]
    return fn


def zero_policy(obs):
    return np.zeros((2, 7), np.float32)


def heuristic_yield_policy(thr: float = 0.4):
    """Hand-written coordination on the clock channel (a baseline and a demo source for the noyield
    base): if the predicted clearance with both arms continuing (res_feat[15]) drops below thr, the
    arm whose stopping leaves more clearance (res_feat[16]) stops (u_rate = -1, sigma -> 0), the other
    continues at the expert's speed. Uses only the observation."""
    from rl.residual_env import CLR_SCALE, WARN

    def inv(f):
        return float(np.arctanh(np.clip(f, -0.999999, 0.999999)) * CLR_SCALE / 4 + WARN)

    def fn(obs):
        F = obs["res_feat"]
        u = np.zeros((2, 7), np.float32)
        if inv(F[0, 15]) < thr:
            y = 0 if inv(F[0, 16]) >= inv(F[1, 16]) else 1
            u[y, 6] = -1.0
        return u
    return fn


def rollout(renv, policy_fn, options=None, seed=None, record=False, max_steps=None) -> dict:
    """One deterministic episode. Returns metrics (+ t, q, weld_events if record)."""
    obs, info = renv.reset(seed=seed, options=options)
    env = renv.env
    a_max = np.stack([env.cell.robots[r].a_max for r in ROBOT_IDS])
    qs = [obs["robot_q"].astype(np.float64).copy()]
    min_clr = float(info["min_clearance"])
    jr_sum, jr_n, sig, dev, n_yield, n_fast, n_plan = 0.0, 0, [], [], 0, 0, 0
    u_abs = []
    steps = 0
    max_steps = max_steps or env.cfg.episode_len
    for steps in range(1, max_steps + 1):
        u = np.asarray(policy_fn(obs), np.float32).reshape(2, 7)
        ue = renv.effective_u(u)
        obs, r, term, trunc, info = renv.step(u)
        qs.append(obs["robot_q"].astype(np.float64).copy())
        min_clr = min(min_clr, float(info["min_clearance"]))
        rs = info["res"]
        g = rs["gate"] > 0.5
        if g.any():
            jr_sum += float(np.abs(rs["joint_res"][g]).mean()) * g.sum()
            jr_n += int(g.sum())
        hp = renv._has_plan
        for k in range(2):
            if hp[k]:
                n_plan += 1
                sig.append(float(rs["sigma"][k]))
                n_yield += rs["sigma"][k] < 0.5
                n_fast += rs["sigma"][k] > 1.05
        dev.append(float(np.abs(rs["a"] - rs["a_base"]).max()))
        u_abs.append(np.abs(ue).mean(0))
        if term or trunc:
            break
    q = np.array(qs)
    dt = env.dt
    v = np.diff(q, axis=0) / dt
    acc = np.diff(v, axis=0) / dt if len(v) > 1 else np.zeros((0, 2, 6))
    acc_ratio = (np.abs(acc) / a_max[None]).max() if len(acc) else 0.0
    ev = [(rid, wid, ts * dt, te * dt) for rid, wid, ts, te in env.weld_events]
    out = dict(welds_done=int(info["welds_done"]), n_welds=int(info["n_welds"]), success=bool(info["success"]),
               collision=bool(info["collision"]), collision_kind=info["collision_kind"],
               steps=steps, time_s=steps * dt,
               makespan_s=(max(e[3] for e in ev) if (ev and info["success"]) else None),
               min_clearance=min_clr,
               res_joint_mean=jr_sum / max(1, jr_n), res_joint_gated_frac=jr_n / max(1, 2 * steps),
               sigma_mean=float(np.mean(sig)) if sig else 1.0, sigma_std=float(np.std(sig)) if sig else 0.0,
               yield_frac=n_yield / max(1, n_plan), fast_frac=n_fast / max(1, n_plan),
               act_dev_mean=float(np.mean(dev)), act_dev_max=float(np.max(dev)),
               u_abs_mean=np.mean(u_abs, 0).mean(0).tolist() if u_abs else [0.0] * 7,
               max_acc_ratio=float(acc_ratio),
               acc_over_frac=float((np.abs(acc) / a_max[None] > 1.0).any(-1).any(-1).mean()) if len(acc) else 0.0)
    if record:
        out.update(t=np.arange(len(q)) * dt, q=q, weld_events=ev)
    return out


def summarize(eps: list) -> dict:
    n = len(eps)
    if n == 0:
        return {}
    kinds = {}
    for e in eps:
        if e["collision"]:
            kinds[e["collision_kind"]] = kinds.get(e["collision_kind"], 0) + 1
    ms = [e["makespan_s"] for e in eps if e["makespan_s"] is not None]
    return dict(n=n, success_rate=float(np.mean([e["success"] for e in eps])),
                collision_rate=float(np.mean([e["collision"] for e in eps])), collision_kinds=kinds,
                weld_frac=float(np.sum([e["welds_done"] for e in eps]) / max(1, np.sum([e["n_welds"] for e in eps]))),
                mean_makespan_success_s=float(np.mean(ms)) if ms else None,
                min_clearance=float(np.min([e["min_clearance"] for e in eps])),
                res_joint_mean=float(np.mean([e["res_joint_mean"] for e in eps])),
                sigma_mean=float(np.mean([e["sigma_mean"] for e in eps])),
                yield_frac=float(np.mean([e["yield_frac"] for e in eps])),
                fast_frac=float(np.mean([e["fast_frac"] for e in eps])),
                act_dev_mean=float(np.mean([e["act_dev_mean"] for e in eps])),
                max_acc_ratio=float(np.max([e["max_acc_ratio"] for e in eps])))
