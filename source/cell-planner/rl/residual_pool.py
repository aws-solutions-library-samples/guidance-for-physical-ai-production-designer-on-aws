"""Auto-resetting pool of ``ResidualEnv`` instances (in-process or worker processes) for train_residual.py.

Episode sampler: with probability ``canonical_frac`` the exact canonical 15-weld task from HOME,
otherwise the env's default randomisation (1-15 welds, +-3 cm weld noise, queue shuffles, start
noise 0.1 rad) with a training seed (>= 10**6, disjoint from the eval seeds 50000+/90000+).
Every step returns (obs, reward, terminated, truncated, slim info, effective u, reset_obs | None).
"""
from __future__ import annotations

import multiprocessing as mp
import os
import traceback

import numpy as np

INFO_KEYS = ("welds_done", "n_welds", "success", "collision", "collision_kind", "min_clearance", "t")


def make_res_env(spec: dict):
    from env.rewards import RewardConfig
    from rl.residual_env import ResidualConfig, ResidualEnv, default_env_config
    rew = RewardConfig(**spec.get("reward", {}))
    env_cfg = default_env_config(reward=rew, **spec.get("env", {}))
    return ResidualEnv(env_cfg, ResidualConfig(**spec.get("residual", {})))


def slim(info, renv):
    out = {k: info.get(k) for k in INFO_KEYS}
    out["min_clearance"] = float(out["min_clearance"])
    r = info.get("res")
    if r is not None:
        out["res_joint"] = float(np.abs(r["joint_res"]).sum() / max(1.0, 6 * r["gate"].sum()))
        out["res_gate"] = float(r["gate"].sum())
        out["sigma"] = r["sigma"].tolist()
        out["has_plan"] = renv._has_plan.tolist() if hasattr(renv, "_has_plan") else [True, True]
        out["dev"] = float(np.abs(r["a"] - r["a_base"]).max())
    return out


class _Group:
    def __init__(self, spec, n, seed):
        self.envs = [make_res_env(spec) for _ in range(n)]
        self.rngs = [np.random.default_rng([seed, i]) for i in range(n)]
        self.canonical_frac = float(spec.get("canonical_frac", 0.2))
        self.ep = [None] * n

    def _reset(self, i):
        rng = self.rngs[i]
        if rng.uniform() < self.canonical_frac:
            obs, info = self.envs[i].reset(seed=int(rng.integers(10 ** 6, 2 ** 31)), options={"canonical": True})
            self.ep[i] = "canonical"
        else:
            obs, info = self.envs[i].reset(seed=int(rng.integers(10 ** 6, 2 ** 31)))
            self.ep[i] = "random"
        return obs

    def reset_all(self):
        return [self._reset(i) for i in range(len(self.envs))]

    def set_bounds(self, jb, rb):
        for e in self.envs:
            e.set_bounds(jb, rb)

    def step(self, actions):
        out = []
        for i, (env, u) in enumerate(zip(self.envs, actions)):
            ue = env.effective_u(u)
            obs, r, term, trunc, info = env.step(u)
            si = slim(info, env)
            si["kind_ep"] = self.ep[i]
            ro = self._reset(i) if (term or trunc) else None
            out.append((obs, float(r), bool(term), bool(trunc), si, ue, ro))
        return out


def _worker(remote, spec, n, seed):
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    try:
        import torch
        torch.set_num_threads(1)
    except Exception:
        pass
    try:
        grp = _Group(spec, n, seed)
        remote.send(("ok", None))
    except Exception:
        remote.send(("error", traceback.format_exc()))
        return
    while True:
        cmd, data = remote.recv()
        try:
            if cmd == "step":
                remote.send(("ok", grp.step(data)))
            elif cmd == "reset_all":
                remote.send(("ok", grp.reset_all()))
            elif cmd == "bounds":
                grp.set_bounds(*data)
                remote.send(("ok", None))
            elif cmd == "close":
                remote.send(("ok", None))
                break
        except Exception:
            remote.send(("error", traceback.format_exc()))


class ResPool:
    def __init__(self, spec: dict, n_envs: int, n_workers: int = 0, seed: int = 0):
        self.n_envs, self.n_workers = n_envs, min(n_workers, n_envs)
        if self.n_workers == 0:
            self.local = _Group(spec, n_envs, seed)
            return
        self.local = None
        ctx = mp.get_context("forkserver")
        base, extra = divmod(n_envs, self.n_workers)
        self.split = [base + (i < extra) for i in range(self.n_workers)]
        self.remotes, self.procs = [], []
        for w, n in enumerate(self.split):
            a, b = ctx.Pipe()
            p = ctx.Process(target=_worker, args=(b, spec, n, seed * 1000 + w), daemon=True)
            p.start()
            b.close()
            self.remotes.append(a)
            self.procs.append(p)
        for r in self.remotes:
            self._check(r.recv())

    @staticmethod
    def _check(msg):
        st, data = msg
        if st != "ok":
            raise RuntimeError(f"env worker failed:\n{data}")
        return data

    def _all(self, cmd, data=None):
        for r in self.remotes:
            r.send((cmd, data))
        return [self._check(r.recv()) for r in self.remotes]

    def reset_all(self):
        if self.local:
            return self.local.reset_all()
        return [o for part in self._all("reset_all") for o in part]

    def set_bounds(self, jb, rb):
        if self.local:
            return self.local.set_bounds(jb, rb)
        self._all("bounds", (jb, rb))

    def step(self, actions):
        actions = list(actions)
        if self.local:
            return self.local.step(actions)
        k = 0
        for r, n in zip(self.remotes, self.split):
            r.send(("step", actions[k:k + n]))
            k += n
        return [x for r in self.remotes for x in self._check(r.recv())]

    def close(self):
        if self.local:
            return
        for r in self.remotes:
            try:
                r.send(("close", None))
                r.recv()
            except Exception:
                pass
        for p in self.procs:
            p.join(timeout=5)
