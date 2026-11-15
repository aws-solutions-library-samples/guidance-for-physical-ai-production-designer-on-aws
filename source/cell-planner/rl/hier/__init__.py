"""Track H: hierarchical GNN scheduler over motion primitives (TASK.md fallback).

instances.py  per-episode weld IK branches (cached pools for training, on-the-fly for eval)
executor.py   motion primitives (transit / descent / dwell / retract / wait / home), no coordination
hier_env.py   semi-MDP wrapper around env.weld_env.KukaWeldEnv (every 20 Hz step goes through it)
features.py   decision-point graph features (robot / task nodes, candidate edges, lookahead terms)
model.py      graph-attention Q network over (robot, task, branch) edges + wait/home options
dqn.py        double DQN + n-step semi-MDP returns + prioritized replay + demo margin loss
heuristic.py  greedy-safe scheduler in the same action space (demos + baseline)
"""
