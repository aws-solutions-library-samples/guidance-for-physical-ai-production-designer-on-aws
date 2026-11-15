# Fix set v10: our umarell fixes on umarell 08d5a39

Base: umarell (`aws-samples/sample-umarell`) main **`08d5a39`** (`08d5a390a498cef123cfe36becfc45ee61876cd9`,
2026-11-04, "Merge pull request #13 from aws-samples/perf/body-weld-2-planning"). Previous base: `dc60669`
(`../patches_dc60669`, set v9). Order: `series.txt`, the same files in the same order as v9. Every patch applies to a
pristine 08d5a39 in that order with `patch -p1 --forward --batch` and with `git apply`, with no offset and no fuzz
(`tests/test_umarell_fast_08d5a39.py`). The base is opt-in: `umarell_fast/apply.py --base 08d5a39` or
`UMARELL_BASE=08d5a39`. The default stays dc60669 / v9.

## What umarell changed since dc60669 (4 commits, PR #13)

* `b7e7664`: reuse and speed.
  * `tools/plan_memo.py`: an MRTA memo and a run-scoped home-step memo.
  * `tools/leg_reuse.py` and `experience_store.py`: the leg store.
  * `tools/path_experience.py`: compound path experience.
  * `OMPL_PARALLEL_CANDIDATES` is now used by ompl-compound too.
  * `HOME_PARALLEL_FASTPATH`.
  * `retry_budget.py`: `PLAN_RETRY_CAP`, default 3, active only inside `agent.plan`.
  * Barrier-free stepwise scheduling.
  * YAML parsed once per content (`cell_layout.load_yaml_text`, `cell_store.read_yaml`).
  * Vectorised frame transforms and moving-obstacle check (`_moving_hit`).
  * The cross-run stores are off in code (`PLAN_MEMO_PROCESS`, `PLAN_LEG_STORE`, `PLAN_EXPERIENCE` default 0). They are on in docker-compose.yml.
* `13d446b`: `tools/fastgeom.py`, Numba kernels for `_moving_hit`, `_first_box_hit`, `_check_robot_pair_collision` and `_link_fill_positions`.
  * They are used when numba imports and `NUMBA_GEOM` is not 0.
  * `numba>=0.59` is added to docker/requirements.txt.
  * The `leg_verdict` cache now holds the judge itself, not its `id()`.
* `34d7b2f`: the coordination conflict test no longer builds the full distance matrix.
* `094a139`: `context_compaction.py` (`CONTEXT_COMPACT`, default 1). The model sees old turns as digests, through a `pre_model_hook`.
  * The perf branch's `d749dc5` (not on main) moves this into the prompt callable, because the hook costs a third recursion step per turn.

## The series

| patch | against v9 | why |
|---|---|---|
| `04v3_hull_backstop` | unchanged (context only) | Still missing. Every new gate path of 08d5a39 goes through calls 04v3 already covers: `inter_robot_clearance` in the planners, plus `validate_trajectory_set`. That includes the home fast path (`_gate_multi_arm_step` calls `validate_trajectory_set`) and leg-store hits (the step gate). |
| `22v3_no_workpiece_dip` | **adjusted** | 13d446b puts `fastgeom.box_first_hit(P, R, own, comp, extra_margin, tool)` at the top of `_first_box_hit`. That kernel implements upstream's blanket `tool_contact` exemption: the tool's spheres are exempt anywhere on the box, which is the dip 22 is about.<br><br>A plain rebase applies cleanly, but it is broken twice over:<ul><li>22v3 renames the parameter `tool` to `wp_rule`, so the kernel line raises `NameError` on every call when numba is installed.</li><li>Fixing only the name would bring the blanket exemption back.</li></ul>v10's 22v3 uses the kernel only when the workpiece rule has nothing to do (no `tool_contact` box, or no torch sphere in the call). It is then called with no tool mask, so there is no exemption, which is the NumPy body's answer in that case. Under the rule the NumPy body decides.<br><br>Cost: on cells with workpiece boxes, checks of states with torch spheres stay on NumPy. That is 08d5a39's NumPy speed, which is still faster than dc60669. |
| `20v2_driver_continuous` | unchanged (context only) | |
| `23v2_commanded_acceleration` | unchanged (context only) | 08d5a39 adds no new `parameterize` calls in the planners. |
| `21v5_vass_interlocks` | **test adjusted** | Code unchanged (context only). Its source-order test `tests/cell/test_lockstep_vass.py::test_annotation_runs_after_the_assemblers_move_merge` now reads `_assemble_behaviour_tree`, because 08d5a39 moved the assembler's body there and the tool only wraps it with timings. The order it checks (move merge before the VASS annotation) is unchanged. |
| `24v2_voxel_boundary_inward` | unchanged | |
| `25v2_near_weld_refinement` | unchanged | |
| `27_hermite_rest_ends` | unchanged | |

## Our hook patches (`umarell_fast/patches_08d5a39`)

* `U1_01_agent_hooks`: context only. The `create_react_agent` call now carries upstream's `pre_model_hook=compact_for_model` next to the hooks' `tool_arg` / `prompt` / `**extra`.
* `U1_02_llm_effort`: unchanged.
* `U3_02_ompl_seed`: unchanged.
* `U3_01_robot_config_cache`: **dropped**. 08d5a39's `arm_model._robot_config` parses robots.yaml once per content and returns a fresh copy (`cell_layout.load_yaml_text`). `UMARELL_MODEL_CACHE` no longer does anything.

## U3 on 08d5a39

* `kernels_76ea13a` (x76) refuses to install. Its guards trip on two functions:
  * `_config_is_free`: the moving-obstacle loop it mirrors became `_moving_hit`.
  * `_check_robot_pair_collision`: it is not the proven original. Its sha changed with the Numba `pair_min_gap` branch.
  * `_load_clearance_policy` still passes its guard.
* The guards are unchanged. `u3 --kernels auto` picks `none` on 08d5a39: umarell's own predicate, which now contains upstream's vectorised and compiled version of what x76 did.
* `u3/__main__.py:tree_base` reports `08d5a39` for a dc60669-layout tree that also has `tools/fastgeom.py` and `tools/plan_memo.py`.

## Verification (2026-11-05, Mac, `.venv-umarell-mac` + numba 0.61.2)

* umarell's own suite on 08d5a39 with this series and `U3_02` applied: 56 failed / 10 errors, the same set as on pristine 08d5a39. The failures are host-only gaps. 2408 passed, against 2275 on pristine.
* U3 rules-only run with numba active: `welding_cell_2_small`, `--base 08d5a39 --with-upstream-fixes`, kernels `none`, 6 workers. Success, 15/15 welds, 98 s wall time.
