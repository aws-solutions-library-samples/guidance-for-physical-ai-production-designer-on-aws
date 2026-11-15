# Concepts

Domain vocabulary used across the dresser codebase, the catalogue
layout, and the REST API. If you just want to use the tool,
[`cli.md`](cli.md) and [`api.md`](api.md) are more direct. Come here
when you need the ground truth on naming, scoring, or merge
semantics.

## Catalogue layout

The catalogue is a structured directory tree that holds every URDF
the dresser can consume or produce:

```
<catalogue>/
├── urdf-robot-collection/        # input: robot arms
│   └── <robot_id>/
│       ├── <robot_id>.urdf
│       └── meshes/
├── urdf-tool-collection/         # input: end-effector tools
│   └── <tool_id>/
│       ├── <tool_id>.urdf
│       └── meshes/
├── urdf-fixture-collection/      # input: static cell fixtures (robo-fitting-room only)
│   └── <fixture_id>/
│       ├── <fixture_id>.urdf
│       └── meshes/               # absent for a fixture built from primitives
├── urdf-dressed-robots/          # output: dressed (robot+tool) URDFs
│   └── <robot_id>__<tool_id>/
│       ├── <robot_id>__<tool_id>.urdf
│       └── meshes/
│           ├── robot/            # robot meshes (+ .mtl) and their hulls
│           ├── tool/             # tool meshes (+ .mtl) and their hulls
│           └── 3dreach/          # reach_envelope.obj
├── floor-plans/                  # input: 2D OBJ floor plans (robo-fitting-room)
└── fitted-robots/                # input: scene manifests (robo-fitting-room)
```

In this workspace the catalogue is `assets/catalogue/`: the robot
`kr240r2900`, the tool `sticktool` and the dressed robot
`kr240r2900__sticktool`.

The dresser owns three of these directories: it reads
`urdf-robot-collection/` and `urdf-tool-collection/` and writes
`urdf-dressed-robots/`. The other three are robo-fitting-room inputs;
they live in the same tree so one root addresses everything:
`--catalogue` for the dresser's `list`, `batch` and `serve`,
`--asset-root` for the fitter. `dress` needs no catalogue: it takes
file paths.

Each robot and tool lives in its own directory **whose name matches
the URDF stem**. `urdf-robot-collection/foo/foo.urdf` is discoverable;
a stray `bar.urdf` inside `foo/` is not, and a directory without
`<name>.urdf` is skipped. That rule is `catalogue::scan`, behind `list`
and `GET /api/v1/catalogue`, which sorts the entries by name. Dressed
robots follow it too (`urdf-dressed-robots/<combo>/<combo>.urdf`,
listed by `GET /api/v1/catalogue/dressed`). Other files in a collection
or an entry directory are ignored: the dresser follows only the URDF's
`<mesh filename>` references. Fixtures keep the convention by habit:
the dresser never reads them, and the fitter takes each fixture's path
from the scene manifest.

The directory name is the entry's id. `POST /api/v1/dress` resolves
`robot_id` and `tool_id` to `<collection>/<id>/<id>.urdf`; each must be
one entry name, without `/`, `\`, `:` or `..` (`400 invalid_id`
otherwise). A dressed robot's directory and file are named
`<robot_id>__<tool_id>`. Where each run writes:

- `POST /api/v1/dress`: `<catalogue>/urdf-dressed-robots/<robot_id>__<tool_id>/`.
- CLI `dress`: the `--output` file, with its `meshes/` beside it. Point
  it at `<catalogue>/urdf-dressed-robots/<combo>/<combo>.urdf` to file
  the result in the catalogue.
- CLI `batch`: `<output>/<robot_id>__<tool_id>/` per pair; use
  `--output <catalogue>/urdf-dressed-robots`.
- `POST /api/v1/batch`: the same tree in a job folder in the system
  temp directory, returned as a zip; the catalogue is left untouched.

Listing (`list`, `GET /api/v1/catalogue`) parses and validates every
entry and reports its status: `valid` (no diagnostics), `incomplete`
(at least one: the errors, or the warnings when there are no errors)
or `error` (the URDF doesn't parse). An entry with a blocking
validation error is therefore `incomplete`, not `error`. Robot entries
add the detected flange link and the joint and link counts, tool
entries the number of visuals.

## Robot URDF

The arm. Conventionally a 6-DOF KUKA / ABB / Fanuc industrial-arm
URDF, but the dresser is agnostic within these limits:

- Plain URDF with a connected kinematic tree; expand xacro first.
- Mesh `filename`s are relative to the URDF's folder (or absolute);
  `package://` and `file://` URIs are not resolved.
- The geometry steps (hulls, flange face, auto-orient, reach) read OBJ
  and binary STL meshes. Other formats (DAE) are copied but contribute
  no vertices, and an ASCII STL is misread.
- Meshes are in metres: those steps ignore `<mesh scale>` (the copied
  visual keeps it).
- No `<transmission>` or `<ros2_control>` blocks: the `<joint>`
  elements inside them are read as robot joints and fail the parse
  with `joint '<name>' has unknown or missing type`.

The **flange** is the link the tool will attach to. Default flange
detection picks the deepest leaf: among the links that are some
joint's child and no joint's parent, the one with the most joints up
to a root (fixed joints count; ties go to the name that sorts first).
When the chain ends in frame links on fixed joints (`flange`, `tool0`),
the flange is the last frame; such a frame usually has no mesh, so the
tool lands on its origin. A robot without joints has no flange
(`cannot detect flange: no leaf links found`). Override with
`--attach-to <link_name>` (CLI `dress`) or `attach_to`
(`POST /api/v1/dress`) if the URDF has a non-leaf flange (e.g. a wrist
plate that branches); the link must exist (`attach-to link '<name>' not
found in robot`). `batch` always detects the flange. The override
moves the tool, not auto-orient's outward direction
([Stage 1](#stage-1-direction-alignment)).

**Joint axes.** A joint without `<axis>` turns about (1, 0, 0), the
URDF default. The axis of the joint whose child is the flange link is
the axis the tool is placed along ([Merge semantics](#merge-semantics)),
and the axis of the last joint in the chain, fixed or not, is
auto-orient's outward direction; a chain that ends in a fixed `tool0`
joint without `<axis>` gives both that joint's (1, 0, 0). The dressed
URDF writes an `<axis>` on every joint, so an omitted one comes out as
`<axis xyz="1 0 0"/>`. robo-fitting-room parses URDFs with the same
parser and the same default.

## Tool URDF

The end-effector — gripper, riveter, welding torch, etc. A tool is one
rigid body: its root link (the link that is no joint's child,
conventionally `World`) carries all visual and collision geometry. It
may also declare a geometry-less TCP link ([below](#tool-centre-point-tcp)).
The catalogue's `sticktool` is exactly that: `World`, `tcp`, and the
fixed `tcp_joint` between them.

For auto-orient and the merge to land the tool correctly, the tool URDF
must satisfy:

1. **Origin at the docking center.** The URDF origin `(0, 0, 0)` must
   be the exact point that mates with the robot's flange. Geometry is
   positioned relative to that point via `<origin xyz=… rpy=…>`.
2. **Outward-pointing geometry.** Auto-orient takes the tool's
   direction from the mean of its visuals' `<origin xyz>` (Stage 1, the
   main score) and checks the centroid of its mesh vertices (Stage 2):
   both should lie on the working side of the docking point. Visual
   origins that the mesh vertices cancel out (meshes exported with
   their own offset) mislead Stage 1, and visuals all at `xyz="0 0 0"`
   leave it at +X; pass `--orient` for such a tool.
3. **All geometry on the root link.** The dresser copies every tool
   link's `<visual>` and `<collision>` with only its own `<origin>` and
   ignores the tool's joints, so geometry on a child link, even on a
   fixed joint, lands as if it hung off the root. The tool's links,
   joints and `<inertial>` don't reach the dressed URDF; only its
   geometry and its TCP do.

None of these rules is checked yet: `validate --tool` (`"tool": true`
on `POST /api/v1/validate`) runs the same checks as robot mode, because
the tool-origin check is a stub (FR-017, BL-009).

### Tool centre point (TCP)

A tool declares its working point (a stick tool's electrode tip, a
gun's TCP) as a link named `tcp`, or `<anything>_tcp` (any case),
reached from the root link through fixed joints. Its pose is given in
the root (docking) frame:

```xml
<link name="tcp"/>
<joint name="tcp_joint" type="fixed">
  <origin xyz="0.67 0 0" rpy="0 0 0"/>
  <parent link="World"/>
  <child link="tcp"/>
</joint>
```

A TCP link behind a movable joint is ignored; of several, the first in
file order wins. The merge carries it into the dressed URDF as a fixed
frame on the flange ([Dressed URDF](#dressed-urdf)). A tool without one
dresses without the frame, and the only notice is a warning in the log,
`tool '<name>' declares no tcp link: the dressed robot gets no TCP
frame` (shown with `RUST_LOG=warn`).

The `tcp` module is the library side: `dressed_tcp` reads the frame
back (a TCP link fixed to the robot's tip link, the child of its last
movable joint), and `mount_transform`, `add_frame_to_urdf_text`,
`set_tcp_frame_in_urdf_text` and `estimate_tool_tip` add or move the
frame in a URDF dressed without one. The CLI and the REST API don't
expose them.

## Dressed URDF

The output of `dress`: one self-contained URDF that keeps the robot's
kinematic tree and adds the tool as geometry, in this order:

- `<robot name="<robot>__<tool>">`: the robot URDF's `name` (`robot` if
  empty), then the tool's `name` (else its first link that is neither
  `World` nor a TCP link, else `tool`). File and directory names come
  from `--output` or the catalogue ids, not from this attribute:
  `kr240r2900` and `sticktool` are both unnamed, so their dressed URDF
  says `robot__tool`.
- `<reach_envelope …/>` ([Reach envelope](#reach-envelope)).
- The robot's links in input order. A link with generated hulls has
  them in place of its `<collision>` elements
  ([Hull buffers](#hull-buffers)); the flange link also carries the
  tool's visuals and collisions after its own, rotated by the chosen
  orientation and moved to the flange face.
- `<link name="tcp"/>`, when the tool declares a TCP.
- The robot's joints in input order, then the fixed `tcp_joint` from
  the flange link to `tcp`, at the declared pose with the tool's
  mounting transform applied. If the robot already has a link or joint
  of those names, the frame becomes `tcp_2` / `tcp_joint_2`, and so on.

All meshes (robot + tool) are copied beside the dressed URDF: robot
meshes and hulls into `meshes/robot/`, tool meshes and hulls into
`meshes/tool/`, the reach hull into `meshes/3dreach/`, with mesh paths
rewritten to the relative locations.

Everything else in the inputs is dropped: `<material>` colours (OBJ
meshes keep theirs through the copied `.mtl`), `<gazebo>`,
`<dynamics>`, `<mimic>`, `<safety_controller>`, `<calibration>`, the
names of visuals and collisions, and comments. The tool contributes
geometry only: its mass and inertia are not added to the flange link.
Numbers are rewritten with at most 10 decimals, and a `<limit>`
attribute missing from the input comes out as `0`.

**The directory split is load-bearing.**
[robo-fitting-room](../../robo-fitting-room/) recognises the tool's
geometry by a mesh path containing `meshes/tool/` (any case, either
separator): for `--full-tools`, for the link a tool payload merges into
and for its mass model. The dresser's `tcp::estimate_tool_tip` finds
the tool's meshes by the same prefix. So the dresser never puts tool
meshes under `meshes/robot/`, or robot meshes under `meshes/tool/`.

## Auto-orientation algorithm

When `--orient` is omitted or `0,0,0` (`orient: [0, 0, 0]` over REST;
`batch` always), the dresser picks one of 64 candidate rotations
(`{0, 90, 180, 270}` per axis) using a purely geometric three-stage
scoring algorithm. An orientation, picked or given, is applied as URDF
`rpy` (roll about X, pitch about Y, yaw about Z, fixed axes) about the
tool's origin, in the flange link's frame. Because `0,0,0` means auto,
the unrotated mount can't be forced. The 64 candidates hold only 24
distinct rotations, so ties are common.

### Stage 1: direction alignment

Compute the tool's "pointing direction": the unit vector from the
origin `(0,0,0)` to the mean of the tool visuals' `<origin xyz>`
positions. Each `<visual>` counts once, whatever its geometry; mesh
vertices are not read. If the mean is at the origin, the direction is
+X.

Compute the robot's "flange outward direction": walk from the link
named `World` (else the first `<link>`), taking at each link the first
joint, in file order, that has it as parent, for at most 20 joints, and
multiply the joints' `<origin rpy>` rotations (home pose). The outward
direction is the `<axis>` of the last joint walked, rotated by that
product. The walk doesn't know which flange was picked: at a branch it
takes the first child joint, and `--attach-to` doesn't change it.

For each candidate orientation, rotate the tool direction and dot it
with the flange outward vector: `normal_dot`, higher is better. The
rotated tool direction is in the flange link's frame and the outward
vector in the walk's start frame; the score compares them as they are.

### Stage 2: centroid-outward check

After rotation and the move to the flange face, compute
`centroid_outward = (c − p) · n`: `c` is the centroid of the tool's
visual-mesh vertices (every vertex counts, so dense meshes weigh
more), `p` the flange-face point and `n` the outward direction. A
positive value means the tool's centroid lies beyond the flange face,
so the tool extends outward rather than back into the arm. Candidates
that fail this are demoted but not rejected outright (some tools
genuinely point back, like a welding torch).

### Stage 3: AABB intersection tiebreak

Count the tool's visual-mesh vertices, placed as in Stage 2, that fall
inside the axis-aligned bounding box of the robot's visual meshes. The
box places each mesh by its visual `<origin>` only, without the link
poses, so it overlays the links' meshes in their own frames rather than
the arm at home pose. Fewer intersections = better.

### Final pick

1. Sort the 64 candidates by `normal_dot`, highest first. The sort is
   stable: equal scores keep enumeration order (X outermost).
2. The band is every candidate within 0.05 of the best `normal_dot`. A
   band of one wins.
3. Otherwise the band members with `centroid_outward > 0` compete: the
   fewest `intersections` wins, even over a higher `normal_dot`; on
   equal counts, the earlier in sorted order.
4. If no band member has `centroid_outward > 0`, the best `normal_dot`
   wins.

Run with `RUST_LOG=info` to see the pick (env_logger shows only errors
by default):

```
[… INFO  urdf_robo_dresser::merger] Auto-orient: loaded <n> mesh components into RAM
[… INFO  urdf_robo_dresser::merger] Auto-orient: picked [X, Y, Z] (normal_dot=…, centroid_outward=…, intersections=…)
```

`RUST_LOG=debug` adds the tool direction
(`Tool direction vector (from origin to visual centroid): [x, y, z]`);
per-candidate scores are not logged.

## Hull buffers

`dress` and `batch` give each `<mesh>` visual of every link its own
convex hull for collision detection: the mesh's vertices are scaled
outward from their centroid by the buffer factor, then the convex
envelope is computed with the quickhull algorithm (`parry3d`). Only
meshes that read as OBJ or binary STL with at least 4 vertices qualify;
box, cylinder and sphere visuals get none. A link that gets at least
one hull has **all** its `<collision>` elements replaced by the hulls,
primitive ones included; a link without any keeps its own. A hull sits
at its visual's `<origin>` with scale 1 (the visual's `<mesh scale>` is
not applied).

| Mode | Scale factor | Use case |
|---|---|---|
| `tight` | 1.00× | No inflation: the mesh's convex hull, concave regions filled. The smallest hulls |
| `medium` | 1.05× | +5% inflation. **Default**. Good balance for general simulation |
| `max` | 1.15× | +15% inflation. Generous safety margin or low-quality input meshes |

Per-component (rather than per-link) hulls keep the collision geometry
tight: a robot link with three separate visual meshes gets three hulls,
not one fat envelope around all three.

Choose the mode with `--hull` (CLI `dress`, `batch`) or `hull`
(`POST /api/v1/dress`, `/api/v1/batch`); anything but the three
lower-case names fails with `invalid hull buffer '<x>' — use tight,
medium, or max` (exit `1`, or `400 invalid_hull_token`). Hulls are
written as `meshes/robot/<link>_hull_<i>.obj` and
`meshes/tool/<link>_hull_<i>.obj`, `<i>` the visual's index in its link
(characters other than letters, digits, `_` and `-` in the link name
become `_`). The CLI and REST handlers generate the hulls for both
inputs before merging; `merge()` itself doesn't. A failed hull is a
warning in the CLI and ignored over REST.

## Merge semantics

`merge(robot, tool, opts)` produces the dressed URDF by:

1. **Flange resolution** — `opts.attach_to`, else the deepest leaf
   ([Robot URDF](#robot-urdf)).
2. **Flange face** — the placement axis `a` is the `<axis>` of the
   joint whose child is the flange link, (1, 0, 0) if there is none.
   The face offset `d` is the largest projection on `a` of the flange
   link's visual-mesh vertices, each placed by its visual `<origin>`,
   and 0 when the link has no readable mesh. The tool's origin goes to
   `a · d` in the flange link's frame. Only the +axis side is searched,
   so the sign of `<axis>` matters.
3. **Orientation** — `opts.orient`, or auto-orient when it is
   `[0, 0, 0]`.
4. **Tool injection** — the visuals and collisions of every tool link
   are collected with only their own `<origin>`, rotated about the
   tool's origin by the orientation, moved to the flange face and
   appended after the flange link's own. The tool's links and joints
   are dropped, so no names are prefixed (`opts.tool_prefix` is
   unused), and `opts.offset`, an extra 6-DOF offset, is not applied
   yet (FR-021).
5. **Mesh path rewrite** — every robot and tool `<mesh filename>`
   becomes `./meshes/robot/<file name>` or `./meshes/tool/<file name>`.
   Subfolders are flattened, so mesh file names must be unique within
   the robot and within the tool: a second file of the same name
   silently resolves to the first.
6. **Mesh copy** — each referenced file, in any format, is copied
   there with the `.mtl` of the same stem (files an `.mtl` references,
   such as textures, are not). Files already in the output are kept,
   not overwritten. A missing source is skipped and its path still
   rewritten; validation warns about it beforehand (`W003`). The output
   never references a path outside its own directory.
7. **Name** — `<robot>__<tool>` ([Dressed URDF](#dressed-urdf)).
8. **Reach envelope** — [below](#reach-envelope); writes
   `meshes/3dreach/reach_envelope.obj`.
9. **TCP frame** — the `tcp` link and the fixed `tcp_joint` when the
   tool declares a TCP ([Dressed URDF](#dressed-urdf)), else a warning.

`merge()` generates no hulls: CLI `dress` / `batch` and
`POST /api/v1/dress` / `batch` call `collision::generate_hulls` on both
inputs first ([Library API](cli.md#library-api)). What is reproducible
is under [Determinism](#determinism).

## Reach envelope

Every dressed URDF records where the tool can reach (FR-028), as the
first child of `<robot>`:

```xml
<reach_envelope max_radius_2d="3.49" volume_3d="108.06" hull_mesh="./meshes/3dreach/reach_envelope.obj"/>
```

- **Chain.** From the root link, follow one joint per link, at most 20.
  At a branch a revolute, continuous or prismatic joint wins over a
  fixed one; among equals, the first in file order.
- **Sampling.** Each movable joint takes 10 evenly spaced values from
  `lower` to `upper`, inclusive (continuous joints and joints without
  `<limit>`: −π to π); fixed joints stay at 0. A 6-axis arm gives 10⁶
  poses.
- **Tool tip.** For each pose, the end of the chain, pushed out along
  the last chain joint's `<axis>` by the flange-face offset plus the
  distance from the tool's origin to its farthest visual-mesh vertex.
- **Result.** Samples below z = 0 of the root frame are dropped.
  `max_radius_2d` (m) is the largest horizontal distance from the root
  frame's Z axis, `volume_3d` (m³) the volume of the samples' convex
  hull, which is written to `meshes/3dreach/reach_envelope.obj`. With
  fewer than 4 samples, or samples flat on an axis (spread ≤ 0.1 mm),
  `volume_3d` is `0`, `hull_mesh` is empty and no file is written.

The parser reads the element back into `UrdfRobot::reach`, and the
viewer's **Reach envelope** toggle draws the OBJ.

## Validation rules

`validate(urdf, path, is_tool)` runs these checks:

| Check | Code | Severity |
|---|---|---|
| No two links, and no two joints, share a name | `E014` | error |
| Every joint's parent and child link is declared | `E012` | error |
| Some link is no joint's child (none means a cycle) | `E011` | error |
| With two or more such root links, every link is reachable from one | `E011` | error |
| Revolute and prismatic joints with a `<limit>`: `lower < upper` | `W013` | warning |
| Every visual and collision mesh file exists (its `filename` joined to the URDF's folder as written) | `W003` | warning |
| A link with visuals has at least one collision (dressing generates hulls anyway) | `W016` | warning |

Tool mode (`--tool`, `"tool": true`) adds nothing yet: the tool-origin
check (FR-017) is a stub. Not checked either: a single root (several
roots, i.e. separate trees, pass), a cycle next to a single root, and a
link that is the child of two joints. An unknown or missing joint type
and malformed XML are parse errors, not diagnostics: the run stops
before validation.

Errors stop `validate` (exit `1`) and `dress` (exit `1`;
`POST /api/v1/dress` answers `422 validation_failed`). Warnings are
printed, or returned in `diagnostics`, and stop nothing.
`POST /api/v1/validate` always answers `200`, with `ok: false` on
errors. `batch`, CLI and REST, skips validation: a pair fails only when
parsing or merging fails. Listing turns the diagnostics into each
entry's status ([Catalogue layout](#catalogue-layout)).

## Determinism

Byte-identical output for identical inputs is a requirement (NFR-003)
that is met only in part. The dress path samples no time or randomness
and writes nothing in hash order, so one build on one machine should
repeat its output for the same inputs, flags and an empty output
directory, but no test checks this yet (`nfr_003_deterministic_output`
is a `todo!`). Auto-orient chooses between near-equal floating-point
scores, so another platform's math library may pick another
orientation (BL-010). Numbers are written with at most 10 decimals.

- Filesystem readdir order: listings are sorted by name. `batch` (CLI
  and REST) visits pairs in readdir order, which changes only the order
  of its console lines and of the entries in the REST batch zip.
- Hash-map iteration order: never reaches the output. Links and joints
  are written in input order, with the tool's geometry after the flange
  link's own and the TCP frame last; hash sets serve lookups only (the
  `E011` message lists unreachable links in arbitrary order).
- System time and random numbers: never used for dress output. REST
  batch job ids are random UUIDs and the server's ETags come from file
  modification times: transport metadata, not output.
- The output directory: re-dressing into an existing one is not a clean
  rebuild. The URDF, the hulls and the reach envelope are rewritten, but
  a copied mesh already there is kept even if its source changed. Delete
  the output directory to re-dress from scratch.

## Cross-crate flow

```
urdf-robot-collection/ ─┐
                        ├─→  urdf-robo-dresser  ─→  urdf-dressed-robots/
urdf-tool-collection/  ─┘    (dress, batch)                  │
                                                             ▼
urdf-fixture-collection/ ─────────────────────────→  robo-fitting-room  ─→  USDA zip  ─→  Isaac Sim
                                                             ↑
                                                fitted-robots/<scene>.yaml
                                                floor-plans/<plan>.obj
```

The two crates share the catalogue convention and the URDF model.
robo-fitting-room links `urdf-robo-dresser` as a library
(`default-features = false`, so none of the server dependencies) and
parses every URDF with its `parse_urdf`, so the parser rules above,
such as the `<axis>` default and the supported subset, hold in both.
The dressed robot itself crosses as files: a scene manifest names it by
a `dressed_urdf` path relative to the manifest
(`../urdf-dressed-robots/<combo>/<combo>.urdf`). The two binaries run
and deploy separately, but the fitter doesn't build without the
dresser's source next to it.

For a cell headed to robo-fitting-room, give robots and tools OBJ
visual meshes: the dresser copies visual meshes in their source format
(only its hulls and the reach hull are OBJ by construction), and the
fitter reads OBJ only.
