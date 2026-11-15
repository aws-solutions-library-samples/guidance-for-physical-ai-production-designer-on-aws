"""Process pool of KukaWeldEnv instances with curriculum-aware auto-reset.

Each worker process owns `envs_per_worker` envs. `step_async(actions)` / `step_wait()` let the
learner run gradient updates while the workers step (the env step is ~1 ms, an update on CPU is
~100 ms, so overlapping matters little on CPU but helps on GPU). On episode end a worker returns
the final observation and immediately resets that env for the next episode of the current
curriculum stage (`set_stage`). `n_workers=0` runs everything in-process (debugging, tests).

Infos are slimmed to picklable scalars: welds_done, n_welds, success, collision, collision_kind,
min_clearance, t.
"""
from __future__ import annotations

import multiprocessing as mp
import os
import traceback

import numpy as np

INFO_KEYS = ("welds_done", "n_welds", "success", "collision", "collision_kind", "min_clearance", "t")


def slim_info(info: dict) -> dict:
    out = {k: info.get(k) for k in INFO_KEYS}
    out["min_clearance"] = float(out["min_clearance"]) if out["min_clearance"] is not None else None
    return out


def make_env(env_kwargs: dict):
    from env.weld_env import EnvConfig, KukaWeldEnv
    from env.rewards import RewardConfig
    kw = dict(env_kwargs)
    rew = kw.pop("reward", None)
    cfg = EnvConfig(**kw)
    if rew is not None:
        cfg.reward = rew if isinstance(rew, RewardConfig) else RewardConfig(**rew)
    return KukaWeldEnv(cfg)


class _EnvGroup:
    """A few envs + curriculum reset logic (runs inside a worker or in-process)."""

    def __init__(self, env_kwargs, n, seed, env_fn=None):
        from rl.curriculum import StageSpec
        self.envs = [(env_fn or make_env)(env_kwargs) for _ in range(n)]
        self.rngs = [np.random.default_rng([seed, i]) for i in range(n)]
        self.spec = StageSpec(stage=0)

    def _reset(self, i):
        from rl.curriculum import reset_env
        seed = int(self.rngs[i].integers(2 ** 31))
        obs, info = reset_env(self.envs[i], self.spec, self.rngs[i], seed=seed)
        return obs

    def set_stage(self, spec):
        self.spec = spec
        return [self._reset(i) for i in range(len(self.envs))]

    def step(self, actions):
        out = []
        for i, (env, a) in enumerate(zip(self.envs, actions)):
            obs, r, term, trunc, info = env.step(a)
            info = slim_info(info)
            info["stage"] = getattr(env, "_curriculum_stage", None)
            reset_obs = self._reset(i) if (term or trunc) else None
            out.append((obs, float(r), bool(term), bool(trunc), info, reset_obs))
        return out

    def close(self):
        for e in self.envs:
            e.close()


def _worker(remote, env_kwargs, n, seed, env_fn):
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    try:
        import torch
        torch.set_num_threads(1)
    except Exception:
        pass
    try:
        grp = _EnvGroup(env_kwargs, n, seed, env_fn)
        remote.send(("ok", None))
    except Exception:
        remote.send(("error", traceback.format_exc()))
        return
    while True:
        cmd, data = remote.recv()
        try:
            if cmd == "step":
                remote.send(("ok", grp.step(data)))
            elif cmd == "set_stage":
                remote.send(("ok", grp.set_stage(data)))
            elif cmd == "close":
                grp.close()
                remote.send(("ok", None))
                break
            else:
                remote.send(("error", f"unknown command {cmd}"))
        except Exception:
            remote.send(("error", traceback.format_exc()))


class EnvPool:
    def __init__(self, env_kwargs: dict, n_envs: int, n_workers: int = 0, seed: int = 0, env_fn=None,
                 start_method: str = "forkserver"):
        self.n_envs = n_envs
        self.n_workers = min(n_workers, n_envs)
        if self.n_workers == 0:
            self.local = _EnvGroup(env_kwargs, n_envs, seed, env_fn)
            self.split = [n_envs]
            return
        self.local = None
        ctx = mp.get_context(start_method)
        base, extra = divmod(n_envs, self.n_workers)
        self.split = [base + (i < extra) for i in range(self.n_workers)]
        self.remotes, self.procs = [], []
        for w, n in enumerate(self.split):
            a, b = ctx.Pipe()
            p = ctx.Process(target=_worker, args=(b, env_kwargs, n, seed * 1000 + w, env_fn), daemon=True)
            p.start()
            b.close()
            self.remotes.append(a)
            self.procs.append(p)
        for r in self.remotes:
            self._check(r.recv())
        self._waiting = False

    @staticmethod
    def _check(msg):
        st, data = msg
        if st != "ok":
            raise RuntimeError(f"env worker failed:\n{data}")
        return data

    def set_stage(self, spec) -> list:
        """Switch curriculum stage; resets every env. Returns the new observations."""
        if self.local is not None:
            return self.local.set_stage(spec)
        for r in self.remotes:
            r.send(("set_stage", spec))
        return [o for r in self.remotes for o in self._check(r.recv())]

    def step_async(self, actions):
        actions = list(actions)
        if self.local is not None:
            self._pending = self.local.step(actions)
            return
        k = 0
        for r, n in zip(self.remotes, self.split):
            r.send(("step", actions[k:k + n]))
            k += n
        self._waiting = True

    def step_wait(self) -> list:
        if self.local is not None:
            return self._pending
        out = [x for r in self.remotes for x in self._check(r.recv())]
        self._waiting = False
        return out

    def step(self, actions) -> list:
        self.step_async(actions)
        return self.step_wait()

    def close(self):
        if self.local is not None:
            self.local.close()
            return
        for r in self.remotes:
            try:
                r.send(("close", None))
                r.recv()
            except Exception:
                pass
        for p in self.procs:
            p.join(timeout=5)
