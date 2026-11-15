# Concepts

This document explains the domain vocabulary used across the fitting-room
codebase, the manifest schema, and the REST API. If you just want to use the
tool, [`cli.md`](cli.md) and [`api.md`](api.md) are more direct; come here when
something in the code or a doc says "see concepts.md" and you want the ground
truth. The physics details — formulas, units, constants — are in
[`physics.md`](physics.md).

## Manifest

A YAML file declaring the contents and layout of one robotic cell: which
robots and fixtures to place, at what world-space poses, optionally with
pedestals, tool payloads and a 2D floor plan. The manifest is the single input
to `compose`; the zip — the manifest itself plus one USDA stage — is the
output. See [`manifests.md`](manifests.md) for the field-by-field reference.

## Dressed URDF

A composite URDF produced by [`urdf-robo-dresser`](../../urdf-robo-dresser/)
that merges a base robot URDF (a six-axis arm, say) with an end-effector tool
URDF (a gripper, a welding gun) into a single file. Tool meshes live under
`meshes/tool/`, robot meshes under `meshes/robot/`. A mesh path containing
`meshes/tool/` (any case, either separator) is how compose tells the tool's
geometry from the robot's: for the `--full-tools` flag, for the link a tool
payload merges into, and for the mass model.

Any URDF with OBJ meshes composes, dressed or not. For a URDF whose tool isn't
under `meshes/tool/`, the manifest's `payload.link` names the link the tool is
mounted on: that link and the links fixed below it are then the tool. Only OBJ
meshes are read; an STL or DAE mesh composes as an empty prim without geometry,
collider or mass (BL-25), which `validate` warns about.

## Fixture URDF

A URDF representing static cell infrastructure (clamping fixtures, conveyors,
safety fences). Fixtures are articulation-free by definition: compose rejects a
fixture URDF with any non-`fixed` joint, which catches authoring mistakes
(`validate` doesn't check this yet, BL-29).

A fixture is emitted as static colliders: every link at its place (forward
kinematics over the fixed joints) with its visual and collision meshes, and no
rigid body, mass, contact report or sensor, no world anchor and no joints. A
collider without a rigid body is static in UsdPhysics, so nothing can shove it,
and nothing in the stage refers to a fixture's links.

## Scene origin

An optional `scene_origin: {x,y,z}` field on the manifest. When set, its value
is subtracted from every robot and fixture `position` and from the floor-plan
offset before USDA emission. The point of this is ergonomic viewport
navigation: real cells often sit far from their CAD origin, which puts the
camera nowhere near anything; subtracting `scene_origin` moves the cell to the
world origin.

The subtraction is applied at parse time, so every downstream piece of the
pipeline sees already-centred coordinates. The manifest keeps the original
positions verbatim — the edit is non-destructive — and the composed layer
records the subtracted origin as `robo_fitting_room:scene_origin_m`. A payload's
centre of mass is link-local and isn't shifted.

## Home pose and the start of the simulation

The joint vector a robot starts in: one value per actuated joint — every
non-fixed joint, in URDF declaration order. Per robot, the manifest's
`home_pose_deg` gives it, in degrees for a revolute or continuous joint and in
metres for a prismatic one. Without it the robot gets the KUKA-style "stowed"
pose `[0°, -90°, +90°, 0°, 0°, 0°]` by joint index on its turning joints (0
beyond the sixth), every prismatic joint at 0 m: a folded-upright configuration
that doesn't intersect the floor or the robot's own base. Revolute angles are
kept as given (190° stays 190°, which a ±350° wrist tells apart from −170°),
continuous ones are wrapped into (−180°, 180°], and every value is in the URDF's
own convention, along its axis as signed. A home value outside its joint's URDF
limits — given or the default — fails validation.

The same values are written into three places:

1. each link's `xformOp:transform` bind pose — forward kinematics pre-computed
   in compose — which keeps every joint's two frames together (omni.physx checks
   the bodies against the joint frames at load and warns about "disjointed body
   transforms" otherwise) and shows the home pose before Play;
2. each joint's drive `targetPosition`, so the drive holds the pose;
3. each joint's initial state, `PhysicsJointStateAPI`: position = the home
   value (the identical text), velocity = 0.

The third is what PhysX starts the simulation from. When an articulation enters
the scene, PhysX overwrites its non-root link poses from the joint positions;
without an authored joint state every joint would start at 0, so every arm
would jump to its URDF zero pose at Play — into its neighbours — and its drives
would whip it back. With the state authored, Play starts where the stage shows,
at rest on the drive targets. See [`physics.md`](physics.md#the-start-of-the-simulation).

## Articulation root

Every robot in the composed USDA carries `PhysicsArticulationRootAPI` +
`PhysxArticulationAPI` on its top-level Xform. This tells PhysX to treat the
robot's link tree as a single articulation — a connected graph of rigid bodies
linked by reduced-coordinate joints, solved as one (64 position and 4 velocity
iterations per step, self-collisions off). The alternative ("disconnected rigid
bodies + explicit joint constraints") is much less stable for industrial-arm
masses and drive stiffnesses.

The articulation root is the entry point for joint-state queries and for the
`IsaacArticulationController` OmniGraph node. The robot's root link is pinned
to the world by a `PhysicsFixedJoint "world_anchor"` at the robot's placement
(raised by its pedestal).

## Mass model

Every link gets its mass, centre of mass, principal moments and principal axes
from one generic rule, with no table for particular robot models; the drives
are tuned on the same numbers. A link keeps its URDF inertial unless it is a
placeholder — less dense, spread over the link's geometry, than a quarter of the
link density — and then takes its mass, centre of mass and inertia from that
geometry at the link density (1200 kg/m³, or the robot's `link_density_kg_m3`).
Its tool's mass comes on top. A link with neither geometry nor a usable inertial
weighs 0.1 kg when a joint moves it (a flange, `tool0`, a TCP frame) and 10 kg
when it is the URDF root or welded to it. See
[`physics.md`](physics.md#mass-model).

## Tool and payload

A robot's manifest entry can carry `payload`: its tool as a rigid mass, with an
optional centre of mass and inertia. It merges into the carrier link —
`payload.link`, else the link carrying the most `meshes/tool/` collision meshes,
else (with a warning) the deepest leaf link. Without an explicit centre of mass
or inertia, the mass is spread over the tool's collision geometry; the tool's
geometry isn't weighed at the link density on top. See
[`physics.md`](physics.md#tool-and-payload).

## Drives

Each actuated joint gets a position drive tuned on the mass model at the home
pose: stiffness 1.5e5 per degree (8.6e6 N·m/rad; a linear drive's from the mass
it moves), damping for a damping ratio of 0.7 on the composite inertia the joint
moves, the URDF effort as the force limit (a finite derived cap where the URDF
gives a placeholder) and the URDF velocity limit. See
[`physics.md`](physics.md#drives).

## Collisions and the welded base

Collision meshes become convex hulls; URDF boxes, cylinders and spheres collide
as themselves. A robot doesn't collide with itself; it collides with the other
robots, the fixtures, the pedestals and the ground. The robot's welded root
links — the URDF root and every link fixed to it, such as `world_link` and the
base — filter out (`PhysicsFilteredPairsAPI`) their own pedestal and the ground's
collider: the base hull sits a few millimetres into what it stands on, and
neither can move, so their contact would only flood the base's contact reports.
See [`physics.md`](physics.md#collisions-and-filtering).

## Pedestals

A robot can stand on a pedestal (`pedestal: {height_mm, diameter_mm}`): the
robot is raised by the height and a static collision cylinder of the given
diameter fills the gap, from the robot's placement height up to its raised foot.
The cylinder is matte, in the default material.

## Floor plan and footprint layers

A 2D CAD plan exported as an OBJ of `l` lines (a DXF file is listed in the
editor's palette but not read, BL-22). Compose draws it on the ground as
`/World/floor_plan`: every segment a 2.5 cm wide flat strip in a matte slate
colour, lifted 2 mm above the ground — one mesh for the general lines, one for
the footprints. A layer (`# Layer: <name>` comment) or group (`o`, `g`) holds
footprints when the manifest's `floor_plan.footprint_layers` lists it
(case-insensitively) or when its name contains "footprint". There is no built-in
layer name: a plan whose footprints carry a CAD layer code needs that layer
listed. The drawing is rendered geometry without a collider, and it isn't part
of the camera framing. Vertices are taken Z up as written (an up axis for Y-up
exports is BL-27; `validate` warns when a plan isn't flat).

## Environment: a stage that loads offline

The stage references no asset at all, so it loads offline. `/Environment` holds
a replica of Kit's "Default" viewport light rig — the rig Kit lights a stage
without lights with: a sun 35° above the horizon in the north-east and a dome —
a matte 18 % grey ground square at z = 0 under the cell (at least 100 m across),
and the ground's collider, an infinite plane (hidden). Isaac Sim's FlatGrid
isn't used. The render settings are left to Isaac Sim: the layer's
`renderSettings` is empty and no render mode is authored.

## Camera rig and the opening view

Six cameras under `/World/Cameras` — four isometric corners (`ISO_NE`, `ISO_NW`,
`ISO_SE`, `ISO_SW`), a top-down `PLAN` and a low `ELEVATION` — frame the cell's
bounding box (the robots at their home poses, the pedestals, the fixtures; not
the floor plan or the ground) from one shared distance, each locked against
viewport navigation and carrying Kit's own exposure. Kit's opening perspective
view is seeded onto the same box from the `ISO_NE` direction. See
[`usda.md`](usda.md#cameras).

## Layer provenance and model kinds

The layer's `doc` is `<name>: <description>` (the bare name without a
description), its `customLayerData` records the generator
(`robo-fitting-room <version>`) and the subtracted scene origin, and
`framesPerSecond` is 60 next to `timeCodesPerSecond`. The model hierarchy is
marked: `/World` is an assembly, `/World/robots` and `/World/fixtures` are
groups, every robot and fixture is a component.

## Topic schema

All ROS 2 topics emitted by the composed USDA follow the two-segment namespace
contract `/<cell_id>/<robot_id>/<channel>`:

| Channel | Topic | Type | Direction |
|---|---|---|---|
| Joint telemetry | `/<cell_id>/<robot_id>/joint_states` | `sensor_msgs/msg/JointState` | pub |
| Joint command | `/<cell_id>/<robot_id>/joint_command` | `sensor_msgs/msg/JointState` | sub |
| Per-link contact | `/<cell_id>/<robot_id>/contacts/<link>` | `std_msgs/msg/Bool` | pub |
| Sim clock | `/<cell_id>/clock` | `rosgraph_msgs/msg/Clock` | pub |

`cell_id` comes from the manifest's `name` field, `robot_id` from each robot's
manifest entry. Both segments are run through `sanitize_ros_segment` so the
result satisfies `rmw_validate_namespace` / `rmw_validate_topic_name`: every
character outside `[A-Za-z0-9_]` becomes `_`, and a digit-leading (or empty)
value gets an `r_` prefix. A scene named `Demo Cell` publishes
`/Demo_Cell/R1/joint_states`; a robot `42` in it, `/Demo_Cell/r_42/…`.

Prim names follow a different rule (`sanitize_prim_name`): a leading digit gets
a `_` in front, every other character outside `[A-Za-z0-9_]` — the first
included — becomes `_`, and a name that is exactly `World` (a robot's,
fixture's, link's or joint's) becomes `world_link`. The robot `42` is the prim
`/World/robots/_42` with its graph `/World/ActionGraph__42`; a robot `-R1` is
`/World/robots/_R1`. The `<link>` of a contact topic is the link's prim name,
not its URDF name: a link `World` publishes on `contacts/world_link`, `link-6`
on `contacts/link_6`.

This schema is the same under FastDDS (Isaac default) and Cyclone DDS
(`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`).

## Contact graphs

Per-link contact telemetry uses the per-robot OmniGraph (we don't emit a
separate graph for contacts). For every link with at least one `<collision>`
block, compose emits inside the existing per-robot graph:

```
ContactSensor_<link>   (IsaacReadContactSensor, csPrim → /World/robots/<robot>/<link>/ContactSensor_node)
        │ outputs:execOut, outputs:inContact
        ▼
PublishContact_<link>  (ROS2Publisher, std_msgs/msg/Bool, topicName = "contacts/<link>")
        │ outputs to ROS 2
        ▼
   /<cell_id>/<robot_id>/contacts/<link>
```

The publisher's exec input is wired to the sensor's `outputs:execOut` (NOT the
playback tick), so the topic stays silent until a contact event lands; its
`data` is the sensor's `inContact`. The generic `ROS2Publisher` is used because
Isaac Sim 5.1 has no `ROS2PublishContactSensor` node. Each collision-bearing
link prim also carries `PhysxContactReportAPI` (threshold 0: every contact
reports) and a child `IsaacContactSensor` prim with `enabled = 1` and
`radius = -1` (full-body queries). Fixtures carry no sensors: a robot link
touching one reports the contact on its own topic.

## Visual fidelity tiers

The compose pass has two independent fidelity flags:

- `--full-robots` (default `false`): keep the robot arm's visual prims.
  Default drops them and shows the collision hulls instead, each in its link's
  paint (the material of the link's first non-tool mesh visual, else the matte
  default), for a smaller zip and a faster Isaac load.
- `--full-tools` (default `true`): keep the tool's visual prims. Tools are the
  `meshes/tool/` geometry the dresser writes, or a named `payload.link`'s. Set
  `--full-tools=false` to drop the tool visuals and show only the tool hulls.

Both flags only decide which `visual_*` prims are emitted and which hulls are
drawn; URDF primitives count like meshes. Every collision hull, the joint
authority, the contact sensor wiring, the masses and the articulation tuning
are the same either way. Fixtures always keep their visuals, hulls hidden. See
[`cli.md`](cli.md#visual-fidelity-matrix) for the matrix.

## Warnings and errors

Validation errors block compose: the CLI, REST and stdio all validate before
they compose. Warnings flag what composes, but probably not as the manifest
means it — a mesh that isn't OBJ, is missing or is given as a URI; a payload
that falls back to the deepest leaf link or has nothing to spread over; a floor
plan without lines, without a footprint layer or not flat — and never fail
anything. `validate` prints them, the validate endpoints return them and compose
logs them, together with the number of OBJ meshes using more than one material
(each is bound to its first, BL-26). The full lists:
[errors](manifests.md#validation-errors) and [warnings](manifests.md#warnings).

## Manifest keys are kept

A manifest may carry keys the tool doesn't know; the parser ignores them. The
server and stdio `save` write a manifest verbatim and `load` returns it byte
for byte, the URDF bundle rewrites only its asset paths, and the editor carries
every key it doesn't edit along on save. Note the flip side: a misspelled
optional key is ignored, not reported.

## Determinism

Identical manifests produce identical USDA text (NFR-105). This matters for CI
snapshots and for content-addressed deployment. The composer never reads the
system clock, the build profile, or any other non-manifest input that could
perturb the output: materials and meshes are kept in sorted maps, links and
joints are written in URDF order, numbers in a fixed format. The URDF bundle zip
is byte-identical across runs.
