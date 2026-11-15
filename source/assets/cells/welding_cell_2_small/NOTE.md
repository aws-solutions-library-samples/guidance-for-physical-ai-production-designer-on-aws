# welding_cell_2_small

> **A FactoryOS reference cell** (`factoryos-cell.json`, format 2): this folder holds the cell's
> outputs and its task program, its inputs stay in the catalogue, where Prepare Cell and the planners
> read them through the cell's view (`.view/`, which the backend builds):
>
> | | where |
> |---|---|
> | scene (Robo Fitter) | `catalogue/scenes/welding_cell_2_small/scene.yaml` |
> | robots (2x) | `catalogue/urdf-dressed-robots/kr240r2900__sticktool` (the Robo Dresser's KR240 R2900 + sticktool, its TCP frame in the URDF) |
> | fixture | `catalogue/urdf-fixture-collection/car_door_fixture`: umarell's car-door fixture model, the boxes of its `welding_cell_2s` `obstacles.yaml` (container floor, walls, clamp rails, weld surface panel) as one mesh, 0.10 m lower than umarell's (tops at z = 1.10 m: the reason is in its URDF) |
> | floor plan | `catalogue/floor-plans/hall-a-floor-plan.obj` |
> | task program | `agentic_planner/tasks/car_door_spot_welding.json`: the 15 weld points (3x5 grid, z = 1.35 m: 0.25 m above the weld surface panel's top, 0.20 m above the fixture's voxel shell) |
>
> Prepare Cell writes `agentic_planner/cell.yaml`, `config/` (robots.yaml, obstacles.yaml: the
> fixture's voxel shell), `srdf/` and `inputs.json`; umarell_jam and umarell_u3 plan the program.
> The program is the umarell cell's, with two changes: every weld's `weld_orientation` is its
> `orientation`, the vertical one (umarell leaned the outer columns 45 degrees at the weld as a
> terminal wrist adjustment: umarell_jam rejects a weld_orientation that differs from the
> orientation, and leaned, W01 has no collision-free pose), and `cycle_time` has no `max_s` (umarell's 30 s, a hard limit for
> umarell_jam, which umarell's own plan, 91.97 s, never met). The robots and their positions are umarell's
> `welding_cell_2_small`'s; its free-space layout (no fixture) has the fixture of `welding_cell_2s`
> back, so umarell_jam and the export checks plan against the cell's geometry.
>
> **The weld points float above the panel**, as in umarell's data (there 0.15 m, see "A known
> inconsistency" below): the electrodes stop 0.25 m above the weld surface panel at every weld. The
> fixture is an obstacle to clear (fixtures:, not a product), not an `is_workpiece` part the
> electrodes touch, so this sample never exercises workpiece contact (fix 22, the sticktool's
> `tool_contact_radius`); a cell with a product placed (Robo Fitter, its welds on the part's faces)
> does.
>
> Until 2026-11 this was a self-contained legacy copy of umarell's cell (its own `scene.yaml`,
> `robots/`, `floorplans/`, a `tasks/` copy), which Prepare Cell could not resolve
> (`../urdf-dressed-robots/…` from the cell's folder) and umarell_jam could not load (no view).
> `fos-migrate-cells` turns such an existing copy into a reference cell of its own scene.
>
> The notes below are inherited from umarell's cell and describe that layout. The standalone player and its
> scripts below read that layout: run them from the view (`.view/run.sh`, `python3 .view/scripts/selftest.py`),
> which has it, with a plan in `agentic_planner/plans/`.

# welding-cell-2s

2× KUKA KR240 R2900-2 with sticktool welding guns, facing each other across a
GEO_FIXTURE_OP1000 fixture. Ported from [umarell](../../../umarell)'s
`cells/welding_cell_2s` into the cellsim self-contained cell format.

The point of this cell: **it is driven by planned trajectories instead of KRL.**
Where cell-runner derives joint angles by interpreting KRL programs (Cartesian
PTP/LIN → IK → OTG), this cell replays the already-planned, collision-free joint
trajectories umarell's trajectory agent produced.

---

## Layout

This one directory is both the `cellDir` and the `envDir`.

```
welding-cell-2s/
├── scene.yaml      3D scene manifest (fitted-robots format, unchanged from the
│                   2-robot-challenge bundle)
├── io_names.json   PLC/robot IO names (cell-runner compatibility)
├── krl/            empty — this cell is trajectory-driven, not KRL-driven
├── robots/         dressed URDF + SRDF + meshes/ + meshes_lowres/
├── fixtures/       GEO_FIXTURE_OP1000 URDF + meshes/ + meshes_lowres/  (on disk,
│                   not referenced by scene.yaml — see "What the scene shows")
├── floorplans/     hall_a floor plan (OBJ)
├── tasks/          PRE-SPLIT COPY of car_door_spot_welding.json. Dead: the
│                   agents read agentic_planner/tasks/. Safe to delete.
├── agentic_planner/  the planning agents' workspace — everything their converter
│   │                 generates. Nothing above this line is theirs.
│   ├── cell.yaml     robot ids + scene_id mapping + cell metadata. Paths in it
│   │                 are relative to THIS cell dir, not to agentic_planner/
│   ├── config/       robots.yaml, obstacles.yaml (the collision world),
│   │                 cyclonedds.xml
│   ├── tasks/        car_door_spot_welding.json — the 15 weld targets
│   └── plans/ logs/ recordings/
└── scripts/
    ├── trajectory_player.py      standalone player (no backend needed)
    ├── selftest.py               verifies the player + every viewer asset
    └── backend_protocol_test.py  verifies the real cell-runner backend
```

Assets came from `fitted-robots/2-robot-challenge-urdf.zip`, which is already a
self-contained bundle with `scene.yaml` at its root and bundle-relative asset
paths — exactly what the rerun-viewer's `/env/<scene>` lookup expects.

### What the scene shows

`scene.yaml` draws **what umarell draws**, which is not the CAD fixture.
`umarell/docker/launch_viz.py` builds its RViz view from three things: the
robot URDFs, semi-transparent CUBE markers from `config/obstacles.yaml`, and
weld-target markers from `tasks/*.json`. The GEO_FIXTURE_OP1000 mesh never appears
there. The box set is what the motion planner actually avoided, so showing it
keeps the view honest about what the trajectory was planned against — a CAD
mesh would imply a fidelity the plan does not have.

So the scene references `obstacles:` and `weld_points:` instead of a
`fixtures:` block. Both source files are byte-identical copies of umarell's.
The viewer renders 10 obstacle boxes (the `floor` plane is skipped — the grid
already represents the floor) and 15 weld points, colour-coded exactly as
umarell does it:

| | colour |
|---|---|
| workpiece / weld surface | orange `(242,140,26,140)` |
| fixture rig (walls, rails) | steel blue `(140,166,191,77)` |
| structural welds | red `(242,38,26)` |
| secondary welds | gold `(255,191,0)` |

Dropping the fixture also cut the RRD from 3.3 MB to 857 KB (894 meshes gone),
which noticeably eases the webview bridge.

The CAD fixture is still on disk under `fixtures/` and can be re-enabled by
restoring a `fixtures:` entry in `scene.yaml`.

#### A known inconsistency in umarell's data

The weld points sit **0.15 m above the surface they are supposed to be on** (in umarell's
`welding_cell_2s`; in this reference cell the fixture is 0.10 m lower, so the gap is 0.25 m):

| | z (umarell) | z (this cell) |
|---|---|---|
| `weld_surface_panel` top face (`obstacles.yaml`; here `car_door_fixture`) | 1.20 m | 1.10 m |
| all 15 weld points (`car_door_spot_welding.json`) | 1.35 m | 1.35 m |

`obstacles.yaml`'s own comment states "weld points sit ON that surface at
z=1.20", so one of the two files is wrong. This is **upstream in umarell** —
both files here are unmodified copies, and umarell's RViz view shows the same
gap, since it draws boxes at their centres and weld points at their raw
positions. Nothing here compensates for it; that would hide the discrepancy
rather than resolve it. Worth deciding which file is authoritative before
trusting either as a weld-surface reference.

### Mesh LOD

`meshes_lowres/` trees were generated with `../../decimate_lowres.py`:

| | visual | low |
|---|---|---|
| robot (12 meshes) | 14.5 MB | 0.6 MB |
| fixture (894 meshes) | 62.6 MB | 6.9 MB |

The extension defaults to `roboticCellTools.meshLod: low`, and that LOD does a
blind `meshes/` → `meshes_lowres/` path swap with no fallback — so without these
trees nothing would render. Both are present; the self-test fetches all 906.

---

## Why the ids differ, and how they are bridged

Three id spaces meet in this cell:

| space | robot 1 | robot 2 |
|---|---|---|
| scene manifest (`scene.yaml`) | `kr240r2900__sticktool` | `kr240r2900__sticktool_2` |
| telemetry / umarell planner | `robot1` | `robot2` |
| KRL/PLC plant ids (unused here) | — | — |

`cell.yaml`'s `robots: [{id, scene_id}]` bridges the first two. The rerun-viewer
reads that mapping from `/env/cell.yaml` (its `fetch_scene_id_map`) and resolves
every scene `robot_id` to the telemetry `id`, so the scene and the trajectory
stream line up without editing either.

The file itself now lives at `agentic_planner/cell.yaml` — the planning agents'
converter generates it, so it sits in their workspace rather than among the bundle
files. `/env/cell.yaml` is kept as an **alias**: the viewer is a separate
application and that URL is its contract, so `trajectory_player.py` resolves the
alias through the same probe it reads the manifest with (`cell_yaml_path`), and a
cell that still has the manifest at its root serves that one instead. The paths
recorded *inside* the manifest are relative to the cell directory, not to the
manifest — `robots/…` still means `<cell>/robots/…`.

Joint names need no mapping: the viewer's dressed-URDF path assigns telemetry
joints to the URDF's revolute chain **positionally** (base → tip), so umarell's
`positions[0..5]` land on `joint_a1..joint_a6`.

---

## Running it

The cell-runner plays trajectories natively, so there is nothing to start by
hand. In Factory OS pick **welding_cell_2_small** as the sample cell (step 1, or
Configuration → Scene → Sample cell), then Load & Play a plan.

Or drive the backend directly:

```
cell-runner --server --cell <this-dir> --env <this-dir> --scene scene.yaml --port 9902
```

(This note predates Factory OS: where it mentions the Kiro extension, its 3D
Cell viewer, the rerun-viewer or the RRD, those are retired; Factory OS shows
the cell as a renderer-neutral scene.)

## The wire protocol

One port, two interfaces — the same pair cell-runner exposes:

```
GET  /env/<path>   static assets (scene.yaml, cell.yaml, URDFs, meshes)
                   `cell.yaml` is an alias for agentic_planner/cell.yaml
WS   /ws           telemetry, cell-runner's NDJSON request/response protocol
POST /rpc          the same protocol over HTTP, for scripting
```

Requests: `state`, `start`, `stop`, `reset`, `set_speed`, `seek`.

`state` returns the shape `live_client.rs` parses, with the render loop's only
real input being a 6-element `joints` array per robot:

```json
{"id": 1, "ok": true, "result": {
  "running": true, "cycle": 0,
  "krl":    [{"robot_id": "robot1", "joints": [0.0, -1.57, 2.2, 0.0, -0.8, 0.0],
              "position": null, "active_folge": 11, "motion_progress": 0.42,
              "operation_id": "W01"}],
  "robots": [{"robot_id": "robot1", "phase": "weld", "model": "kr240_r2900_ultra"}],
  "playback": {"t": 5.31, "duration": 91.97, "speed": 1.0}
}}
```

`position` (Cartesian) is `null`: playback does not synthesize forward
kinematics, and `live_client` treats the field as optional. Joints are
authoritative — they are what the planner solved and what the viewer renders.

---

## How a BehaviourTree becomes motion

The plan is compiled **once** into an absolute timeline, then a playhead samples
it. The trajectories already carry TOPP-RA intra-node timing and the planner had
already resolved inter-robot conflicts into explicit `INTERLOCK_WAIT` gates, so
each gate is resolved at build time against the depended-on robot's scheduled
motion. The result is deterministic and seekable, rather than a re-run of the
live gate loop.

Node handling mirrors umarell's `agents/cell/bt_executor.py`:

| node | playback |
|---|---|
| `SEQUENCE` | children back to back |
| `PARALLEL` | children from a common start; ends at the latest child |
| `EXECUTE` | `start_delay_s`, optional interlock pre-move (2.5 s), trajectory scaled by `velocity_scale`, then `hold_duration_s` + 1.0 s settle |
| `INTERLOCK_WAIT` | resolved to the first time the awaited robot is within `tolerance_rad` of the target pose — including while it *holds* a pose it reached earlier |
| `HOME` | the planned return path when present, else a 7 s move to home |
| `SAFETY_GATE` | no-op (nothing to gate in playback) |

Constants (`HOME_TIME_S`, `COMPLETION_MARGIN_S`, `HOME_JOINTS`) and the angle
comparison are taken from umarell's `ExecConfig`, including its full-turn
wrapping — weld poses put wrist joints near ±π, where a naive comparison reads a
spurious 2π error.

### The current plan

`bt_1785273909_welding_cell_2s.json` — 15 spot welds across 8 steps, 91.97 s:

```
  0.00 -> 13.16  robot1  W01      3.00 -> 13.92  robot2  W15
 14.92 -> 21.33  robot1  W02     14.92 -> 24.14  robot2  W13
 25.14 -> 33.02  robot1  W03     25.14 -> 32.71  robot2  W14
 34.02 -> 44.26  robot1  W06     35.17 -> 43.64  robot2  W10
 45.26 -> 55.09  robot1  W09     45.41 -> 54.41  robot2  W07
 56.09 -> 63.74  robot1  W05     56.09 -> 67.16  robot2  W12
 68.16 -> 78.99  robot1  W04     68.16 -> 75.46  robot2  W11
                                 79.99 -> 86.37  robot2  W08
 87.37 -> 90.97  robot1  home    87.37 -> 90.44  robot2  home
```

All six interlock gates resolve, and per-robot chaining is exact: every
segment's first waypoint equals the previous segment's last (worst
discontinuity 0.000000 rad), so the arms move weld-to-weld with no teleports.

Drop another umarell plan into `trajectories/` and pass `--plan` to replay it;
nothing here is specific to this one.

---

## Adding more plans

Plans come from umarell:

```bash
# in the umarell repo, after a planning run
cp cells/welding_cell_2s/plans/bt_*.json \
   <this-cell>/trajectories/
```

Planning needs LLM credentials; **replaying does not** — the plan carries every
joint angle, so the player only interpolates.

---

## Native integration

Trajectory playback is now a **first-class cell-runner motion source**, so the
external player is no longer required. The cell-runner gained:

| verb | purpose |
|---|---|
| `trajectory_list` | plans under `trajectories/` |
| `trajectory_load` | compile a plan (by `path` or inline `plan`) and collision-check it |
| `trajectory_play` / `_pause` / `_stop` | transport |
| `trajectory_seek` / `_set_speed` / `_set_loop` | playhead |
| `trajectory_state` | playhead + per-robot phase/operation |
| `trajectory_unload` | back to the KRL interpreter |

While a plan is loaded its joint angles **overlay** the same `krl[i].joints`
telemetry field the KRL path publishes, which is the one reason motion appears in
the 3D viewer with no viewer or RRD changes. Unloading restores the KRL state
untouched.

In Factory OS, step 1 (Load & Play) drives it.

### Two cell.yaml fields that matter

Both were needed to get the cell fully working, and are easy to miss:

- **`urdf:` per robot** — overrides the default `<env>/model/<model>.urdf`
  lookup. Without it the trajectory provider degrades to heuristic timing and
  the collision checker cannot load any geometry at all.
- **A sibling `.srdf`** next to the dressed URDF
  (`robots/kr240r2900__sticktool/kr240r2900__sticktool.srdf`, taken from
  umarell). The checker retains *all* self-collision pairs when no SRDF is
  present, so adjacent links that touch by design (`base_link` / `link_1`)
  report as colliding at every waypoint. With it, only real interference is
  reported.

The cell-runner also needs `--scene` to place robot bases; without it every base
defaults to the origin and collision results are meaningless.

### Collision findings for the current plan

With collision enabled, `trajectory_load` reports one genuine interference:

```
robot1/like_kr240r2900_link_3_0  vs  robot1/like_kr240r2900_link_6_4
  SelfCollision   progress 0.065   distance -0.0114 m
```

The arm's own welding gun (a `link_6` collision hull) penetrates `link_3` by
about 11 mm early in the cycle. umarell's planner models the tool as a swept axis
plus mounting plate rather than its mesh, so a mesh-level interference this small
is not something it would necessarily reject — worth reviewing before running
this plan on hardware.

## The standalone player

`scripts/trajectory_player.py` remains useful and speaks the same protocol: it
needs no backend, no Rust, and no dependencies, so it is the quickest way to
check a plan or to develop against the protocol.

```bash
./run.sh --loop                  # serve on 9902 with a plan preloaded
./run.sh --no-plan               # start empty; load over the wire
python3 scripts/selftest.py      # 49 checks
```

It reports `collision: {checked: false}` — collision checking needs the
cell-runner backend.

## Verifying

```bash
# the standalone player + every asset the viewer fetches
python3 scripts/selftest.py                          # 49 checks

# the real backend (start cell-runner first, then)
python3 scripts/backend_protocol_test.py 9902        # 31 checks
```

## Factory OS note

In umarell this cell's robot meshes and floor plan are gitignored as duplicates of
`cells/welding_cell_2t`. Factory OS ships only this cell, so they are committed here and the cell is
self-contained: a release carries everything its 3D view needs.
