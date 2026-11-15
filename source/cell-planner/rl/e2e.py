"""Track G: end-to-end GNN TD3 checkpoints as `execute.py --controller e2e:<ckpt>` controllers.

Interface: docs/DESIGN.md (Track Ex change log). The controller is the deterministic actor followed by the
checkpoint's action map (twist -> DLS joint velocities, idle arms zeroed), exactly as `evaluate.make_policy`.
"""
from __future__ import annotations

import numpy as np

# EnvConfig fields a training run may have changed that matter for acting (obs layout / collision sub-steps)
_ENV_KEYS = ("self_dist", "self_dist_max", "n_substeps", "cell_id")


def env_kwargs(ckpt: str) -> dict:
    """EnvConfig overrides the checkpoint needs (e.g. self_dist=True adds the robot_self_dist obs)."""
    import torch
    ck = torch.load(ckpt, map_location="cpu", weights_only=False)
    out = {k: v for k, v in dict(ck.get("env_kwargs", {})).items() if k in _ENV_KEYS}
    out["assignment"] = dict(ck.get("stage_spec", {})).get("assignment", "greedy")   # stage-3 split used in training
    return out


class E2EController:
    def __init__(self, agent, amap, ck: dict):
        from evaluate import make_policy
        self._fn = make_policy(agent, amap)
        self.stats = {"stage": ck.get("stage"), "steps": ck.get("steps"), "last_eval": ck.get("last_eval", {})}

    def reset(self, obs: dict) -> None:
        pass

    def act(self, obs: dict) -> np.ndarray:
        return np.asarray(self._fn([obs])[0], np.float32).reshape(-1)


def load_controller(ckpt: str, env=None, device: str = "cpu") -> E2EController:
    from rl.action_map import ActionMap
    from rl.td3 import load_agent
    agent, ck = load_agent(ckpt, device)
    cell = dict(ck.get("env_kwargs", {})).get("cell_id", "welding_cell_2_small")
    if env is not None and ck.get("graph_cfg", {}).get("self_dist") and not env.cfg.self_dist:
        raise ValueError("checkpoint expects EnvConfig(self_dist=True); build the env with rl.e2e.env_kwargs(ckpt)")
    return E2EController(agent, ActionMap(ck.get("action_map"), cell), ck)
