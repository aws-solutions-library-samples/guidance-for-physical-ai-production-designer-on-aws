# Scene and task input format (schema_version 2)

This is the input of the trajectory generator (`trajgen`). A scene has **1 to 4 robots** (more load, with a warning),
each with its own model, base pose and limits, and any number of welds with arbitrary positions, approach vectors
and orientations. `schema_version: 1` files still load (§11). A scene directory holds three files:

```
scenes/<cell_id>/
  scene.yaml   the cell: robots, collision model, obstacles, workpiece rule, planning/executor parameters, tolerances
  task.json    the weld program, in umarell's car_door_spot_welding format
  ERRATA.md    (sample scenes only) what differs from the umarell originals, and why
```

Loader and validator: `trajgen/scene.py`.

Ready-to-run copies of every scene in this folder, with their verified generator outputs, a description and a
preview image, are in `trajgen/samples/` (`python -m trajgen.samples list`; the files here stay canonical).

```python
from trajgen.scene import load_scene, load_task, validate, load_original
scene = load_scene("scenes/welding_cell_2_small")        # SceneError on schema errors
task  = load_task("scenes/welding_cell_2_small/task.json")
warnings = validate(scene, task, coverage=True)          # SceneError on hard errors, list of warnings otherwise
orig = load_original("welding_cell_2_small")             # the umarell cell, same dataclasses, for comparison
```

`python -m trajgen.scene --scene <dir> [--task <json>] [--coverage]` prints a summary and the warnings.

## Contents

1. [Conventions](#1-conventions)
2. [scene.yaml: assets and robots](#2-sceneyaml-assets-and-robots)
3. [scene.yaml: collision](#3-sceneyaml-collision)
4. [scene.yaml: obstacles](#4-sceneyaml-obstacles)
5. [scene.yaml: workpiece](#5-sceneyaml-workpiece)
6. [scene.yaml: planning and executor](#6-sceneyaml-planning-and-executor)
7. [scene.yaml: tolerances](#7-sceneyaml-tolerances)
8. [task.json](#8-taskjson)
9. [Validation](#9-validation)
10. [Writing a new scene](#10-writing-a-new-scene)

## 1. Conventions

- **Units.** SI units: m, rad, s. A key ending in `_deg` is in degrees. The world frame is z up, with the floor
  surface at the `obstacles.floor.z` given in the scene (0 in the sample).
- **Quaternions.** `xyzw`, matching umarell's JSON. The loader normalises them and rejects a norm that is off by
  more than 1e-3.
- **Joint vectors.** Joint vectors are 6-long, in the order a1..a6 of `joints.names`. Robots are listed in the
  order the generator uses (the second axis of the output timeline `q (N_t, R, 6)`). Robot ids are free strings
  without `|` and surrounding blanks.
- **URDF rpy.** `R = Rz(yaw) · Ry(pitch) · Rx(roll)`.
- **TCP frame.** The TCP frame is the `tool.tip_link` frame translated by `tool.tcp_offset` and rotated by
  `tool.tcp_rotation` (`identity`, or a mapping with one of `rpy`, `rpy_deg`, `quat`). The **tool axis** (flange to
  electrode tip) is `tool.axis` in the TCP frame. For the stick tool this is **-x**, so the world tool axis is
  `-R_tcp[:, 0]`. Every robot of a scene must have the same `tool.axis` (a weld's `weld_orientation` then means the
  same tool axis for all of them); the TCP offsets, rotations and tools may differ.
- **Spin.** The rotation about the tool axis is free unless `tolerances.spin_free` is false (§7).

## 2. scene.yaml: assets and robots

```yaml
schema_version: 2
cell_id: welding_cell_2_small
description: free text
umarell: {cell_id: welding_cell_2_small, parity: true}   # optional, see "umarell parity" below
assets:
  cells_root: ../../third_party/sample-umarell/cells
robots:                                    # 1..4 robots
  - id: robot1                             # any unique string
    model: kr240r2900
    urdf: cells:welding_cell_2_small/robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf
    srdf: cells:welding_cell_2_small/robots/kr240r2900__sticktool/kr240r2900__sticktool.srdf
    base: {xyz: [x, y, z], yaw: 0.0}       # or yaw_deg / rpy [r, p, y] / rpy_deg / quat [x, y, z, w]
    home_deg: [0, -90, 90, 0, 0, 0]        # or home: [...] in rad
    joints:                                # optional block, every key optional (read from the URDF, see below)
      names: [joint_a1, ..., joint_a6]
      lower: [...]                         # rad
      upper: [...]                         # rad
      velocity: [...]                      # rad/s (URDF <limit velocity>)
      acceleration: [...]                  # rad/s^2, before the planning acceleration rule (§6)
    tool:
      tip_link: like_kr240r2900_link_6
      tcp_offset: [-0.9092, 0.0005, -0.0056]
      tcp_rotation: identity               # or {rpy: [...]} / {rpy_deg: [...]} / {quat: [...]}
      axis: [-1, 0, 0]
    collision:                             # optional per-robot overrides of the scene-level defaults (§3)
      link_spheres: {radius: 0.12, fill_step: 0.12, tcp_radius: 0.05}
      tool_spheres: {frame: like_kr240r2900_link_6, spheres: [[x, y, z, r], ...]}
      disabled_pairs: [[link_a, link_b], ...]
    umarell_id: robot1                     # optional: this robot's id in the declared umarell cell
```

**Robot model.** Each robot is a serial chain of exactly 6 revolute joints from the URDF root to `tool.tip_link`
(fixed joints before, between or after them are folded in; revolute axes may be any unit vector). Collision hulls
are the URDF `<collision>` meshes (paths relative to the URDF; `package://` URIs are not supported) or boxes.

**Joint limits from the URDF** (docs/p3_urdf.md). An omitted `joints.names` / `lower` / `upper` / `velocity` is read
from the URDF (the movable joints on the chain to `tool.tip_link`, their `<limit lower upper velocity>`). A given value
must match the URDF within the rounding of hand-copied numbers (1e-6 abs + 1e-6 rel); if it does not, the scene value
overrides the URDF, a warning is logged and added to the scene notes (`RobotSpec.limits_info` says which source each
vector came from). URDFs carry no acceleration limits: an omitted `acceleration` is the per-model default of
`robotmodel.urdf_robot.ACCEL_DEFAULTS` (KR240 R2900: [3, 3, 3, 6, 6, 8] rad/s^2, umarell's REVIEW default, not
measured), with a note; a model without a default needs the key.

**Asset paths.**
- A path that starts with `cells:` is relative to `assets.cells_root`.
- Any other relative path is relative to the scene file. Use these for assets copied next to the scene.
- An absolute path is used as is.

**Choosing `cells_root`.** The first of these that is set is used:
1. `load_scene(..., cells_root=...)`;
2. the `UMARELL_CELLS` environment variable;
3. `assets.cells_root`, relative to the scene file.

**Base pose.** `base` is the world pose of the URDF root (`base_link`): a translation `xyz` plus at most one
rotation key: `yaw` (rad) / `yaw_deg` about world z, `rpy` / `rpy_deg` (URDF convention R = Rz·Ry·Rx) or `quat`
(xyzw). Positions are `base.xyz + R_base · p_local`; with a yaw only this is umarell's `base + Rz(yaw) · p`.
A ceiling-mounted robot is e.g. `rpy_deg: [180, 0, yaw]` (scenes/synthetic_3r). A robot's own pedestal must not be
an obstacle box: the base_link sphere and hull would always touch it (put the base at the pedestal top instead).

**umarell parity.** `umarell: {cell_id: <cell>, parity: true}` declares the umarell cell the scene is derived
from. The generator then also enforces umarell's own arm-arm gate: umarell's 35-sphere tool grid
(`tool_collision` from that cell's robots.yaml, per robot via `umarell_id`, default the same id, else the cell's
robot in the same list position), placed on THIS scene's robots (URDF, bases, homes). Plans then pass umarell's
validators unchanged. Without the block there is no parity model (`--umarell-parity` is refused).

**YAML anchors.** You may use YAML anchors to share identical blocks, for example `tool: &stick_tool` and then
`tool: *stick_tool`.

## 3. scene.yaml: collision

The scene-level `collision` block holds the defaults for every robot; `robots[i].collision` overrides
`link_spheres`, `tool_spheres` and `disabled_pairs` per robot (for example a robot with another tool).

### 3.1 Link spheres (arm structure)

```yaml
collision:
  link_spheres: {radius: 0.12, fill_step: 0.12, tcp_radius: 0.05}
```

This is umarell's arm model (`agents/trajectory/tools/robot_kinematics.py`, reimplemented in
`env/kinematics.py`). For a 6-axis URDF it has 42 spheres per arm:
- **Frame spheres:** one sphere at every pinocchio frame origin. The frames are universe, root_joint, World,
  base_link, the (joint, link) pairs, and `welding_tcp`.
- **Fill spheres:** spheres every `fill_step` between consecutive joint origins.
- **Radii:** `radius` for every sphere, except `tcp_radius` for the TCP frame sphere.

The spheres sit on the kinematic skeleton. They do **not** envelope the KR240 castings (ERRATA E15). This is why
the hull checks below exist.

### 3.2 Arm vs arm

```yaml
  arm_arm: {pair_radius: 0.12, gate_m: 0.20, hull_backstop: true, hull_min_distance_m: 0.03}
```

- **Sphere clearance.** For **every pair of robots** (a, b), the clearance is `min |c_i(a) - c_j(b)| - 2 · pair_radius`,
  over all link, fill, TCP and tool spheres of both arms. It must stay **> gate_m** at every check sample. This is
  umarell's gate (`warning_distance`). Every sphere counts with `pair_radius` here, whatever its own radius.
- **Hull backstop.** With `hull_backstop: true`, the URDF collision hulls of any two arms must also stay at least
  `hull_min_distance_m` apart (MuJoCo signed distance; optional, default **0.03 m**, `0` = no contact only). The
  spheres miss parts of the castings (link_2 / link_3 hull vertices up to 0.45 / 0.53 m outside the link spheres), so
  this is the physical rule. trajgen holds it at the nominal timing **and** with any one robot shifted by every
  check-grid step within its `--skew` window (default ±0.1 s: one 20 Hz executor poll of dispatch / motion lag, with
  margin); the verifier gates both (`arm_arm_hull`, `arm_arm_hull_skew`). `false` makes the hull checks
  informational.

### 3.3 Tool spheres

```yaml
  tool_spheres:
    frame: like_kr240r2900_link_6
    spheres: [[x, y, z, r], ...]          # centres in `frame`, own radius r
    coverage_requirement: {min_vertex_fraction: 0.99, max_gap: own_radius}
```

- **Contents.** An explicit list. It is not umarell's axis/plate grid, which cannot express the rotated hulls.
- **Requirement.** At least `min_vertex_fraction` of the vertices of the frame link's collision hulls must lie
  within their sphere's own radius, with the hulls placed by the full `<origin xyz rpy>`.
  - `validate(..., coverage=True)` and `tests/test_trajgen.py` check this.
  - `trajgen/tool_spheres.py` refits the set.
- **Sample set.** 18 spheres: 7 × r 0.07 on the stick and 11 × r 0.12 on the flange, discs and plate. Coverage is
  100 %, against 72.9 % for the original 35 (ERRATA E1).

Tool geometry in the `link_6` frame, with the origins applied (the original comments had it wrong, ERRATA E2):

| hull | part | x | y | z |
|---|---|---|---|---|
| link_6_hull_0 | flange | -0.275..-0.239 | ±0.095 | ±0.095 |
| World_hull_1 / _2 | adapter discs | -0.346..-0.244 | ±0.10 | ±0.11 |
| World_hull_3 | RAH plate, normal to the tool axis | -0.365..-0.288 | -0.174..0.182 | -0.262..0.249 |
| World_hull_4 | 80 mm bar | -0.873..-0.308 | ±0.043 | -0.048..0.037 |
| World_hull_0 | electrode, TCP at x = -0.909 | -0.909..-0.857 | ±0.022 | -0.026..0.016 |

### 3.4 Self collision

```yaml
  self_collision:
    geometry: urdf_collision_hulls
    min_distance_m: 0.0
    disabled_pairs: [[link_a, link_b], ...]
```

- **Geometry.** Every `<collision>` mesh of every link, origins applied. All of them are convex here.
- **Requirement.** For any two geoms on different links of the same arm, the signed distance must stay
  ≥ `min_distance_m`, unless their link pair is listed in `disabled_pairs`.
- **Disabled pairs.** A listed pair disables all geoms of both links, so the tool hulls on link_6 are not checked
  against link_5.
- **SRDF.** The list must contain every `disable_collisions` pair of the robot's SRDF. The validator enforces
  this. Adding pairs beyond the SRDF is allowed, but has to be justified in ERRATA.
- **Origin.** umarell has no such check (ERRATA E5).

## 4. scene.yaml: obstacles

```yaml
obstacles:
  floor: {z: 0.0, margin_m: 0.05, sphere_rule: true, hull_rule: true}    # floor: null = no floor
  default_box_margin: 0.05
  box_hull: {enabled: true, min_distance_m: 0.0}
  boxes:
    - {id: fx_0001, center: [x, y, z], half_extents: [hx, hy, hz], collision_margin: 0.05, is_workpiece: false}
    - {id: post, center: [x, y, z], half_extents: [...], yaw_deg: 30}    # optional rotation: yaw / rpy / quat
```

### Floor

The floor is a horizontal plane at `z`. Two rules apply, and both are enforced at every check sample.

- **Sphere rule.** `centre_z - r ≥ z + margin_m` must hold for:
  - every link-frame sphere *after* base_link (base_link stands on the floor);
  - every fill sphere;
  - the TCP sphere;
  - **every tool sphere**, with its own radius.
- **Hull rule.** The minimum world z over the vertices of every collision hull except base_link's, tool hulls
  included, must be ≥ `z`.

### Boxes

The box rule is umarell's rule, with the tool included:
- **Which spheres.** Every sphere except the three root frames (universe, root_joint, World). Tool spheres count
  with their own radius.
- **Distance.** The distance is measured from the sphere centre to the box surface. The box is not inflated.
- **Violation.** A sphere violates the rule when that distance is `< r + collision_margin`.
- **Workpiece boxes.** Boxes with `is_workpiece: true` are skipped for the TCP sphere and the electrode sphere
  (the tool sphere nearest the TCP). They still count for everything else.
- **Hull rule** (`box_hull`, new in schema 2). The URDF collision hulls of every arm (all links incl. the tool) must
  keep a MuJoCo signed distance >= `min_distance_m` to every non-workpiece box. The link spheres do not envelope the
  castings (ERRATA E15), so the sphere rule alone can pass a link hull inside a box (welding_cell_2t ERRATA T5).
  Workpiece boxes are checked by the sphere rule only (the electrode must touch them).
- **Electrode off-weld rule** (`box_hull.electrode_off_weld`, optional, nrobot round 2; docs/n2_traj2.md §2,
  docs/n2_contract.md §3.1). `electrode_off_weld: {enabled: true, radius_m: 0.05, min_distance_m: 0.0,
  score_tol_m: 0.005}`; absent or `null` = off (every older scene). When on, it applies to every workpiece box:
  while the TCP is farther than `radius_m` from every weld of the task, the signed distance between the electrode
  hull(s) (`tool.electrode_hulls`, else the tip hull that contains the TCP) and the box must be >= `min_distance_m`.
  Planners enforce it in every single-arm check; `trajgen.verify` adds the check `electrode_off_weld` (fails above
  `score_tol_m` penetration). scenegen v2 writes it into every scene.
- **No-dip rule** (`box_hull.no_dip`, round 3; trajgen/nodip.py, docs/r3_nodip.md). The electrode / tool never
  enters a workpiece box, except the weld contact itself. `no_dip: true | false | {enabled, version: 1,
  contact_tcp_radius_m: 0.005, contact_point_radius_m: 0.010, contact_depth_m: 0.0015, min_distance_m: 0.001}`. An
  absent key follows `trajgen.nodip.default_on()`, i.e. `$TRAJGEN_NODIP` (0 / 1) or the code default. On a scene
  with workpiece boxes the rule:
  * replaces the electrode off-weld rule;
  * turns on `workpiece` (every non-electrode hull vs the workpiece boxes);
  * checks the electrode hull(s) against every workpiece box everywhere: >= `min_distance_m` for planners and the
    env, any penetration fails in the verifier;
  * exempts only the robot's own weld contact: TCP within `contact_tcp_radius_m` of the weld point, contact point
    within `contact_point_radius_m`, depth <= `contact_depth_m`, and in the verifier only during the dwell and the
    uninterrupted contact around it.

  The verifier runs this check (still named `electrode_off_weld`) and the hull rule at 400 Hz.

## 5. scene.yaml: workpiece

```yaml
workpiece: {rule: wedge, radius_m: 0.020, tol_m: 0.003, points: tcp}
```

This is rule W from docs/workpiece_decision.md. `rule: none` turns it off. It is the default for every weld; a
weld can override it in task.json (`rule_w`, §8).

For every weld of the task, take:
- p = the weld position;
- a = the unit approach_vector;
- t = the required tool axis, `-R(weld_orientation)[:, 0]`.

The TCP x of either arm violates the rule, at any time of the cycle, when both of these hold:
- `|x - p| < radius_m`;
- `min((x - p)·a, (x - p)·t) > tol_m`.

## 6. scene.yaml: planning and executor

| key | meaning | sample |
|---|---|---|
| `velocity_scale` | transit joint-speed cap, as a fraction of `joints.velocity` | 0.7 |
| `acceleration_rule` | `vscale_squared`: accel cap = `acceleration · s²` at speed scale s (umarell TOPP-RA); `vscale`: `· s`; `none`: `· 1` | vscale_squared, so 0.49 a_max at 0.7 |
| `approach_velocity_fraction` | descent speed = this × `velocity_scale` | 0.4, so 0.28 |
| `home_velocity_scale` | absolute speed scale of the return to HOME (capped by `velocity_scale`) | 0.4 |
| `approach_offset_m` | pre-weld standoff: `p_pre = p - a · approach_offset_m` | 0.10 |
| `retract_offset_m` | lift-off length: `p_ret = p - a · retract_offset_m` | 0.10 |
| `min_offset_m` | shortest allowed standoff or lift-off, where the full one is blocked (≥ `tolerances.entry_span_m`) | 0.045 |
| `descent: {straight_on_axis, tube_m}` | the descent p_pre → p is a straight Cartesian line at the weld orientation; the TCP stays within `tube_m` of it | true, 0.002 |
| `liftoff: {required, straight_on_axis, tube_m}` | after the dwell, a straight lift-off p → p_ret at the weld orientation **before** any transit. A joint-space move out of the seam is forbidden (ERRATA E7) | true, true, 0.002 |
| `dwell_s` | time standing still at the weld | 0.5 |
| `executor_margin_s` | umarell executor slack after each `ExecuteTrajectory` leg, for exports to umarell BT plans. Every leg's completion and every WaitForRobot delay of the plan is computed with it, so the plan must be run with umarell's `ExecConfig.completion_margin_s` equal to this value (recorded as `metadata.executor_margin_s` in plan.json; trajgen warns when it differs from umarell's default 1.0) | 1.0 |
| `sample_dt` | output trajectory sample interval | 0.05 (20 Hz) |
| `check_dt` | interval at which collision and workpiece checks are evaluated, by interpolating the trajectory. The generator's dense checks run at max(100 Hz, 1/check_dt) unless `--check-hz` is given | 0.0125 |
| `max_jerk_factor` | optional jerk cap in units of a_max/s. `null` = none. umarell plans reach about 4.5 | null |

**Speed of each phase.**

| Phase | Joint speed cap | Joint acceleration cap |
|---|---|---|
| transit | `velocity_scale · v_max` | `accel_scale(velocity_scale) · a_max` |
| descent and lift-off | `velocity_scale · approach_velocity_fraction · v_max` | same rule at that scale |
| home return | `home_velocity_scale · v_max` | same rule at that scale |

`PlanningSpec.accel_scale(s)` and `PlanningSpec.approach_vscale` compute these.

**Speed profile (optional, docs/p3_tgfull.md).** `speed_profile: umarell | fast | full` selects the profile when
the generator is run without `--speed` (the CLI option wins; default `umarell` = the table above). `full`: every
non-weld motion (transit, parking / yield legs, return to HOME) at 1.0 × the URDF `<limit velocity>` and 1.0 ×
`joints.acceleration` (KR240 default [3, 3, 3, 6, 6, 8] rad/s², umarell's REVIEW value), jerk-limited at
`full_speed.jerk_factor` × a_max per second (default 10), every segment a rest-to-rest S-curve (it accelerates to
full speed and back, never switches speed instantly); descent, hold and lift-off keep the umarell rule of the table
above (0.28 v_max / 0.078 a_max at the sample values). Optional sub-block (only read by `full`):

```yaml
planning:
  speed_profile: full
  full_speed: {velocity_scale: 1.0, acceleration_scale: 1.0, jerk_factor: 10.0}
```

## 7. scene.yaml: tolerances

These are umarell's `validate_approach` gates (`agents/trajectory/tools/pinocchio.py:476-639`).

| key | meaning | sample |
|---|---|---|
| `position_m` | TCP position error at the weld | 0.010 |
| `tool_axis_deg` | angle between the achieved and required tool axis. Only the axis is gated, not the full rotation | 5.0 |
| `entry_direction_deg` | angle between the TCP motion over the last `entry_span_m` before the weld and `approach_vector` | 15.0 |
| `entry_span_m` | length of the entry window | 0.04 |
| `spin_free` | the rotation about the tool axis is free (umarell semantics; needed for robot1 at W01/W04, ERRATA E5) | true |
| `spin_tol_deg` | with `spin_free: false`: the spin may deviate from `weld_orientation` by at most this, and the verifier gates the geodesic error at `spin_tol_deg + tool_axis_deg` | 5.0 |

## 8. task.json

umarell's task-program format (`car_door_spot_welding.json`), unchanged in structure. Only the fields below are
read. Everything else is passed through in `Task.raw`.

```json
{
  "program": "car_door_spot_welding",
  "cell_id": "welding_cell_2_small",
  "extensions": {
    "strategy": {"sequence_groups_order": ["structural", "secondary"], "...": "..."},
    "weld_params": {"approach_offset_m": 0.10, "retract_offset_m": 0.10, "dwell_s": 0.5, "...": "..."},
    "conventions": {"approach_vector": "...", "...": "..."}
  },
  "operations": [
    {"id": "W01", "type": "weld", "position": [x, y, z], "approach_vector": [0, 0, -1],
     "orientation": {"x": 0, "y": -0.7071, "z": 0, "w": 0.7071},
     "weld_orientation": {"x": 0, "y": 0.923877, "z": 0, "w": -0.382689},
     "sequence_group": "structural", "priority": 1,
     "placeholder": false, "original_position": [x, y, z], "position_note": "...",
     "allowed_robots": ["robot1"], "rule_w": {"radius_m": 0.02, "tol_m": 0.003}}
  ]
}
```

| field | required | meaning |
|---|---|---|
| `id` | yes | unique weld id, a free string except the reserved ones (hard error): `home` (any case, the HOME operation of the plans), anything containing `_YIELD_<digits>` (yield-to-HOME operation ids), `|` (report keys `weld|robot`), empty or with surrounding blanks |
| `type` | yes | only `"weld"` operations become welds |
| `position` | yes | world position of the weld point, which is the TCP target |
| `approach_vector` | yes | **unit direction in which the TCP TRAVELS INTO the weld point** (the descent direction), any direction: `[0,0,-1]` = come down from above, `[1,0,0]` = a side weld entered moving in +x, `[0.707,0,-0.707]` = tilted. Pre-weld standoff = `position - approach_vector · approach_offset_m`; the lift-off goes back out along the same line. It is **not** the surface normal pointing at the robot, and **not** the tool axis (ERRATA E6). The entry gate compares the TCP's motion over the last 0.04 m with it |
| `weld_orientation` | yes (else `orientation` is used) | required TCP-frame orientation at the weld, as a world quaternion. Required tool axis = `-R[:, 0]` |
| `orientation` | no | tool-down transit orientation (informational) |
| `sequence_group` | no | `structural` / `secondary`. Groups run in `extensions.strategy.sequence_groups_order`: with the default `--group-order per_arm` **within each robot's program only** (one robot may start a later group while another is still on an earlier one; `report.json` `group_order_check` counts such cross-robot inversions); `--group-order global` makes each group a cell-wide phase (all robots HOME between groups); `ignore` drops the order |
| `priority` | no | int, default 1 |
| `placeholder` | no | `true` = the position is not from CAD. The validator warns. Defaults to true when `position_note` is present |
| `original_position`, `position_note` | no | provenance of a moved weld |
| `allowed_robots` | no | robot ids that may make this weld (default: any). Others are not even tried |
| `rule_w` | no | per-weld rule W: `false` (off for this weld), `true`, or `{radius_m, tol_m}` overriding the scene's `workpiece` values |

**Program-level fields.**
- `extensions.weld_params` documents the program's stroke and dwell values. The **scene's `planning` block is
  authoritative** for the generator.
- `extensions.conventions` repeats the semantics above inside the file, so the file is self-describing.

## 9. Validation

`validate(scene, task, check_files=True, coverage=False)` raises `SceneError` on any hard error.

**Hard errors:**
- no robots; duplicate robot or weld ids; reserved weld ids (§8) or a robot id with `|`;
- joint `upper <= lower`; home outside the joint limits; a non-positive limit or radius;
- a base rotation given twice (`yaw` and `rpy` ...) or not a rotation;
- a missing URDF or SRDF; `tip_link`, a robot's `tool_spheres.frame` or a disabled-pair link that is not in its URDF;
- an SRDF pair missing from the robot's `disabled_pairs`;
- robots with different `tool.axis`;
- unsupported values: `self_collision.geometry` other than `urdf_collision_hulls`, `workpiece.rule` other than
  wedge / none, `workpiece.points` other than tcp;
- speed fractions outside (0, 1]; an unknown acceleration rule; `min_offset_m` greater than an offset;
- a task `cell_id` that does not match the scene;
- a quaternion or approach-vector norm more than 1e-3 off 1;
- `allowed_robots` naming a robot the scene does not have;
- a weld below the floor; a tool axis pointing against the approach (> 90°).

**Warnings:**
- more than 4 robots;
- placeholder welds;
- a tool axis leaning more than 30° off the approach (in the sample, the 10 outer-column welds at 45°);
- a weld out of the rough reach (sum of the URDF joint offsets + |tcp_offset|) of every allowed base;
- `min_offset_m < entry_span_m`; `dwell_s` / `executor_margin_s` not a multiple of `sample_dt`;
- `liftoff.required`, `liftoff.straight_on_axis` or `descent.straight_on_axis` false (trajgen always makes straight
  on-axis strokes, which is stricter);
- tool-sphere coverage below the requirement, per robot (only with `coverage=True`).

**At generation time** (`python -m trajgen`) the remaining conditions are checked and reported, never silently
dropped: a robot whose HOME violates a single-arm rule (stage `homes`, naming the rule: joint, floor_sphere,
floor_hull, box, box_hull, rule_W, self_collision), a weld no robot can reach with a valid straight stroke (reason per robot: out of reach / no IK solution /
the rules the IK branches violate / stroke blocked), a weld whose every robot collides with another robot standing at
HOME (priority coordination), robots too close at HOME (sphere gate, or hulls closer than `hull_min_distance_m`), a given assignment that names unknown robots or welds, lists
a weld twice, omits one or gives it to a robot that cannot reach it. The run then writes only `report.json`
(`status: failed`, `failed_welds` with stage and reasons) and exits 2; `--allow-partial` writes the plan of the
remaining welds marked partial.

## 10. Writing a new scene

1. **Start from a sample.** Copy `scenes/welding_cell_2_small/` (two robots, umarell parity) or one of
   `scenes/synthetic_{1r,3r,4r}/` (1, 3 and 4 robots, no umarell counterpart, generated by
   `scripts/trajgen_make_synthetic.py` from `trajgen/synth.py`). Set `cell_id`, the robots (URDF, SRDF, base pose,
   home, limits, tool) and the obstacles. Remove the `umarell:` block unless the scene is a umarell cell. Robot ids
   and weld ids are free strings (apart from the reserved ones above); any combination works, the plan's
   blackboard keys are made unique where `traj_<robot>_<op>` would clash.
2. **Homes.** Every robot's home must be valid (limits, self, floor, boxes) and clear of every other robot's home:
   robots wait there. A compact home (e.g. `[0, -90, 0, 0, -90, 0]` deg, the arm upright) lets robots stand closer.
3. **Refit the tool spheres** if a tool differs: run
   `python -m trajgen.tool_spheres`, or call `trajgen.tool_spheres.fit(urdf)`, and paste the centres and radii
   (per robot under `robots[i].collision.tool_spheres` if only some robots carry that tool).
   Check them with `python -m trajgen.scene --coverage`.
4. **Copy the SRDF pairs** into `collision.self_collision.disabled_pairs` (or per robot).
5. **Write `task.json`** in the format of §8, with unit `approach_vector`s in the travel-into-the-weld sense and
   `weld_orientation`s whose tool axis (`R @ tool.axis`) is within the tool-axis tolerance of the approach (or leans
   deliberately, as the sample's outer columns).
6. **Compare with umarell's own cell** if the scene is derived from one:
   `load_original(cell_id)` loads the umarell cell into the same dataclasses, and `trajgen.checks` evaluates
   both sets of rules on joint timelines.

## 11. Changes from schema_version 1

Version 1 files load unchanged. New in version 2 (all optional): the `umarell:` block (in version 1 the parity cell
is inferred from `cell_id` when such a umarell cell exists, with a note in the warnings), base rotations other than
`yaw`, `tool.tcp_rotation` other than identity, per-robot `collision` overrides and `umarell_id`, `obstacles.box_hull`
(default enabled), rotated boxes, `floor: null`, `tolerances.spin_tol_deg`, and the per-weld `allowed_robots` and
`rule_w`.
