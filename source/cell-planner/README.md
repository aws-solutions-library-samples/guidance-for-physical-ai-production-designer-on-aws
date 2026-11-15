# cell-planner: the planners' runtime

The part of the cell planner Physical AI Production Designer runs: what the planner router
(`deploy/planner-router/`) and the Robo Fitter's reach check call, and nothing else. It is installed read-only on
the host (`deploy/provision-cell-planner.sh`) and run in its own Python 3.12 environment
(`deploy/requirements-cell-planner.txt`); umarell_jam runs in a second one, of its own numerical stack
(`requirements-umarell.txt`).

| Path | What |
|---|---|
| `tools/btexport/` | `python -m tools.btexport`: turns a planner's output into the behaviour-tree plan the cell runs, retimes it to the robots' limits and checks it (limits, collisions, dense sampling) |
| `tools/retime/`, `tools/ctrl/` | time parameterisation and the controller-side checks btexport uses |
| `execute.py`, `evaluate*.py`, `rl/`, `models/`, `env/`, `expert/` | the learned policies (hierarchical RL with a learned lookahead, PPO) and their environment: rolling a policy out on a cell |
| `runs/` | the policies' checkpoints, those `deploy/planner-router/planners.json` names |
| `trajgen/` | the scripted multi-robot generator, and its precomputed sample for the reference cell (`trajgen/samples/welding_cell_2_small/`) |
| `umarell_fast/` | U3 (`python -m umarell_fast.u3`): umarell's own planning tools driven by a deterministic parallel orchestrator, on a copy of umarell with the fixes of `docs/upstream/patches_08d5a39/` applied (`umarell_fast/apply.py`) |
| `umarell_jam/` | umarell jam (`python -m umarell_jam`, run by `deploy/planner-router/umarell_jam.py`): U3's planning from a cache of certified weld primitives, transfers and phase schedules, on its own patched copy of the plain umarell 76ea13a (`deploy/umarell-jam.pin`) with the fixes of `docs/upstream/patches_76ea13a/` and U3's hooks (`umarell_fast/patches_76ea13a/`). Seconds where its cache is warm; a public install starts with an empty cache, so the first job on a cell prepares its components (minutes, `prepare_budget_s` in planners.json) or falls back to U3. See `docs/umarell_jam.md` |
| `requirements-umarell.txt` | umarell_jam's environment (`jam-venv`, `deploy/provision-cell-planner.sh`): pinocchio 2.7.0, hpp-fcl 2.4.4, numpy 1.26.4, OMPL; toppra 0.6.3 is built on the host from its sdist (`deploy/toppra.pin`). Its cache keys depend on this stack |
| `player/`, `robotmodel/` | plan and timeline formats, robot model overrides |
| `data/` | the weld IK tables of the reference cells |
| `scenes/` | the reference cell's scene (format: `scenes/SCHEMA.md`) |

The planners and their limits are listed in `deploy/planner-router/planners.json`: the learned policies plan the
cells they were trained on, U3 and the agentic planner plan any cell, umarell jam the cells it lists that have a
task program of their own (`agentic_planner/tasks/`).
