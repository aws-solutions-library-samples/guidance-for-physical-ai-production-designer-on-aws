"""kuka_weld_rl environment package.

Lazy exports so that ``import env.cell`` / ``env.kinematics`` do not pull in MuJoCo/gymnasium:
    from env import KukaWeldEnv, EnvConfig, RewardConfig, build_spec, build_model
"""
_EXPORTS = {
    "KukaWeldEnv": "weld_env", "EnvConfig": "weld_env",
    "RewardConfig": "rewards", "compute_reward": "rewards",
    "build_spec": "scene", "build_model": "scene",
    "load_cell": "cell",
}


def __getattr__(name):
    if name in _EXPORTS:
        import importlib
        return getattr(importlib.import_module(f".{_EXPORTS[name]}", __name__), name)
    raise AttributeError(name)


__all__ = list(_EXPORTS)
