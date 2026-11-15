"""Track PPO: on-policy PPO with massive CPU parallelism for the end-to-end KUKA weld policy.

Modules:
  features  obs dict -> compact graph features (flat float32 rows) -> models' DenseGraph
  model     GNN actor-critic (reuses models.gnn_policy.HeteroGNNEncoder), Gaussian policy
  rollout   shared-memory rollout buffers + env worker processes (env + actor inference)
  stages    curriculum wrapper around rl.curriculum (+ env-default variants) and promotion
  ppo       GAE, PPO update, BC auxiliary loss
  controller  load_controller(ckpt, env) / env_kwargs(ckpt) for execute.py (DESIGN.md, Track Ex)
"""


def load_controller(ckpt, env):
    from .controller import load_controller as _lc
    return _lc(ckpt, env)


def env_kwargs(ckpt):
    from .controller import env_kwargs as _ek
    return _ek(ckpt)
