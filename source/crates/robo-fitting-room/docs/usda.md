# USDA layout

What `compose` writes to the inlined `.usda` inside the output zip, prim by
prim. Use this when you're inspecting the stage in Isaac's Stage tree, or
writing a downstream consumer that reads the USD directly. The physics behind
the numbers — joint states, drives, masses, collisions — is explained in
[`physics.md`](physics.md).

The stage is one ASCII layer (`#usda 1.0`), Z up, in metres, with every mesh
inlined: no references, no payloads, no sublayers, no asset paths, so it loads
offline and doesn't need Isaac's asset converter.

## Layer header

```usda
#usda 1.0
(
    customLayerData = {
        dictionary cameraSettings = {
            dictionary Front = {
                double3 position = (5, 0, 0)
                double radius = 5
            }
            dictionary Perspective = {
                double3 position = (…)
                double3 target = (…)
            }
            dictionary Right = {
                double3 position = (0, -5, 0)
                double radius = 5
            }
            dictionary Top = {
                double3 position = (0, 0, 5)
                double radius = 5
            }
            string boundCamera = "/OmniverseKit_Persp"
        }
        dictionary omni_layer = {
            string authoring_layer = "./demo-cell.usda"
            dictionary locked = {
            }
            dictionary muteness = {
            }
        }
        dictionary renderSettings = {
        }
        string "robo_fitting_room:generator" = "robo-fitting-room <version>"
        double3 "robo_fitting_room:scene_origin_m" = (50, 20, 0)
    }
    defaultPrim = "World"
    endTimeCode = 1000000
    framesPerSecond = 60
    metersPerUnit = 1
    startTimeCode = 0
    timeCodesPerSecond = 60
    upAxis = "Z"
    doc = "Demo Cell: Two arms sharing one station"
)
```

- `Perspective` seeds Kit's opening view (`/OmniverseKit_Persp`, which stays
  the bound camera): it looks at the centre of the scene's bounding box from the
  `ISO_NE` direction, at the distance that frames the box through Kit's default
  18.147562 mm lens ([cameras](#cameras)). A scene without geometry keeps the
  template's view, position (5, 5, 5).
- `renderSettings` is empty, so Isaac Sim's own render settings apply.
- `robo_fitting_room:generator` is the crate version that wrote the stage;
  `robo_fitting_room:scene_origin_m` the manifest's `scene_origin` ((0, 0, 0)
  without one), which every placement had subtracted.
- `doc` is `<name>: <description>`, or the bare name without a description;
  quotes, backslashes, `\n`, `\r` and `\t` are escaped.

## Top-level prim hierarchy

In emission order:

```
/World                              ← def Xform, defaultPrim, kind = "assembly"
├── PhysicsScene                    ← PhysxSceneAPI: gravity −Z, 120 Hz, GPU capacities
├── robots                          ← def Scope, kind = "group" (only with robots)
│   └── <robot_id>                  ← kind = "component": PhysicsArticulationRootAPI + PhysxArticulationAPI
│       ├── <link> …                ← every link, flat: PhysicsRigidBodyAPI + PhysicsMassAPI, bind pose
│       │   ├── visual_<i>          ← Mesh | Cube | Cylinder | Sphere, bound to a /World/Looks material
│       │   ├── collision_<i>       ← the same types + PhysicsCollisionAPI (meshes as convex hulls)
│       │   └── ContactSensor_node  ← IsaacContactSensor (links with a collision)
│       ├── <joint> …               ← every URDF joint: PhysicsRevoluteJoint | PhysicsPrismaticJoint | PhysicsFixedJoint
│       └── world_anchor            ← PhysicsFixedJoint pinning the root link
├── fixtures                        ← def Scope, kind = "group" (only with fixtures)
│   └── <fixture_id>                ← kind = "component": static colliders
│       └── <link> …                ← plain Xforms: visual_<i>, collision_<i> (hidden)
├── pedestals                       ← def Scope (only when a robot has a pedestal higher than 0)
│   └── pedestal_<robot_id>
│       └── socket                  ← static collision Cylinder
├── floor_plan                      ← the floor-plan drawing (only with segments)
│   ├── Looks/M_floorplan
│   ├── lines                       ← Mesh: the general lines
│   └── footprints                  ← Mesh: the footprint layers
├── Looks                           ← def Scope of UsdPreviewSurface materials
├── ActionGraph_clock               ← OmniGraph: /<cell>/clock publisher (only with robots)
├── ActionGraph_<robot_id> …        ← OmniGraph per robot with an actuated joint (motion + contacts)
└── Cameras                         ← six rig cameras (only when there is geometry to frame)

/Environment                        ← lights, ground, ground collider
/Render                             ← hidden: render product, OmniverseGlobalRenderSettings, LdrColor
```

Prim names are the manifest IDs and URDF names through `sanitize_prim_name`
([`manifests.md`](manifests.md#names-become-prim-names-and-topics)).

In Isaac Sim 5.1 the composed cell's robots stand on a matte grey ground with
the floor plan drawn on it; `/Environment` holds the replica of Kit's Default
light rig, and `/World` also holds `pedestals`, `floor_plan` and `Cameras`.

## Robots

### Articulation root

```usda
    def Xform "R1" (
        prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]
        kind = "component"
    )
    {
        uniform bool physxArticulation:enabledSelfCollisions = 0
        uniform int physxArticulation:solverPositionIterationCount = 64
        uniform int physxArticulation:solverVelocityIterationCount = 4
        uniform float physxArticulation:sleepThreshold = 0.005
        uniform float physxArticulation:stabilizationThreshold = 0.001
        double3 xformOp:translate = (1.5, 1, 0.5)
        quatd xformOp:orient = (0.707107, 0, 0, 0.707107)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
```

The translate is the manifest position minus `scene_origin`, raised by the
pedestal's height; the orient is the ABC angles as a quaternion (w, x, y, z).
Its children are flat: every link, then every joint, then `world_anchor`.

### Links

```usda
        def Xform "base_link" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxContactReportAPI", "PhysicsFilteredPairsAPI"]
        )
        {
            matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            uniform token[] xformOpOrder = ["xformOp:transform"]
            rel physics:filteredPairs = [</World/pedestals/pedestal_R1/socket>, </Environment/GroundCollider>]
            float physxContactReport:threshold = 0
            float physics:mass = …
            point3f physics:centerOfMass = (…)
            float3 physics:diagonalInertia = (…)
            quatf physics:principalAxes = (1, 0, 0, 0)
            …visual_<i>, collision_<i>, ContactSensor_node…
        }
```

- **Every** articulated link carries `PhysicsRigidBodyAPI` and `PhysicsMassAPI`
  — the URDF root included: fixed-base behaviour comes from `world_anchor`.
  The mass properties come from the mass model
  ([`physics.md`](physics.md#mass-model)); `physics:principalAxes` comes with
  every `physics:diagonalInertia` (UsdPhysics wants both or neither), the
  identity when the tensor is diagonal in the link's axes.
- `matrix4d xformOp:transform` is the link's bind pose relative to the robot
  root, forward kinematics at the home pose (written transposed, row vectors).
- `PhysxContactReportAPI` (threshold 0: every contact reports) and the
  `ContactSensor_node` child only on links with at least one `<collision>`.
- `PhysicsFilteredPairsAPI` only on the welded root links (the URDF root and
  every link fixed to it): its own pedestal's `socket`, if it has one, and the
  ground collider.

**Visuals** are `visual_<i>`, `i` the visual's index in the URDF, emitted as
the fidelity flags decide ([`cli.md`](cli.md#visual-fidelity-matrix)):

- a mesh becomes a `Mesh` with its OBJ data inlined — `uniform token
  subdivisionScheme = "none"`, `point3f[] points` (the URDF scale applied),
  `int[] faceVertexIndices` (polygons fan-triangulated), `int[] faceVertexCounts`
  (all 3); no normals;
- a URDF box becomes a `Cube` (`double size = 1`, extent ±0.5) scaled to its size
  by `xformOp:scale`, a cylinder a `Cylinder` (`uniform token axis = "Z"`,
  `height`, `radius`, `extent`), a sphere a `Sphere` (`radius`, `extent`);
- every visual has `MaterialBindingAPI` and `rel material:binding` to
  `/World/Looks/<material>` — its OBJ's first `usemtl` material, else
  `factoryos_default` — and the URDF origin as `xformOp:translate` +
  `xformOp:orient`.

**Collisions** are `collision_<i>`, always emitted:

```usda
            def Mesh "collision_0" (
                prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
            )
            {
                uniform token purpose = "guide"
                uniform token physics:approximation = "convexHull"
                uniform token subdivisionScheme = "none"
                point3f[] points = […]
                int[] faceVertexIndices = […]
                int[] faceVertexCounts = […]
                double3 xformOp:translate = (0, 0, 0)
                quatd xformOp:orient = (1, 0, 0, 0)
                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
            }
```

A primitive collider is its `Cube` / `Cylinder` / `Sphere` with
`PhysicsCollisionAPI` but no mesh-collision API, which PhysX collides with as
it is. A hidden hull
has `purpose = "guide"`; a shown one (a lean robot's, or a tool's with
`--full-tools=false`) has no purpose but `MaterialBindingAPI`, bound to its
link's paint — the material of the link's first non-tool mesh visual — else to
`factoryos_default`.

**Contact sensor**, on links with a collision:

```usda
            def IsaacContactSensor "ContactSensor_node"
            {
                bool enabled = 1
                float radius = -1
                float minThreshold = 0
                float maxThreshold = 100000
                float3 translate = (0, 0, 0)
            }
```

### Joints

Every URDF joint becomes a prim between its links' prims: revolute and
continuous joints a `PhysicsRevoluteJoint`, prismatic ones a
`PhysicsPrismaticJoint`, fixed ones a `PhysicsFixedJoint` (bodies and frames
only); floating and planar joints are skipped with a warning. An actuated joint,
here the smallest case from the crate's tests — a revolute joint about +Z with
limits of ±1.5 rad, a URDF effort of 100 and a velocity of 1 rad/s, moving a
0.1 kg frame link:

```usda
        def PhysicsRevoluteJoint "joint_0" (
            prepend apiSchemas = ["PhysicsDriveAPI:angular", "PhysicsJointStateAPI:angular", "PhysxJointAPI"]
        )
        {
            rel physics:body0 = </World/robots/R/link_0>
            rel physics:body1 = </World/robots/R/link_1>
            uniform token physics:axis = "Z"
            float physics:lowerLimit = -85.943669
            float physics:upperLimit = 85.943669
            float drive:angular:physics:stiffness = 150000
            float drive:angular:physics:damping = 0.716329
            float drive:angular:physics:maxForce = 100
            float drive:angular:physics:targetPosition = 0
            float state:angular:physics:position = 0
            float state:angular:physics:velocity = 0
            float physxJoint:maxJointVelocity = 57.29578
        }
```

| Attribute | Value |
|---|---|
| `physics:axis` | `X`, `Y` or `Z` for a URDF axis along +X, +Y or +Z; any other axis — negative or skewed — is `X`, with the rotation taking +X onto the axis folded into both frames |
| `physics:lowerLimit` / `upperLimit` | the URDF limits: degrees on a revolute joint, metres on a prismatic one; none on a continuous joint or without `<limit>` |
| `drive:<angular\|linear>:physics:stiffness` | 1.5e5 per degree; a linear drive's from the mass it moves |
| `drive:…:physics:damping` | for a damping ratio of 0.7 on the composite inertia the joint moves |
| `drive:…:physics:maxForce` | the URDF effort, or a derived finite cap where the URDF gives a placeholder |
| `drive:…:physics:targetPosition` | the home value: degrees (metres on a prismatic joint) |
| `state:…:physics:position` / `velocity` | the home value (the identical text) and 0: the joint's start state |
| `physxJoint:maxJointVelocity` | the URDF velocity in deg/s (m/s on a prismatic joint), with `PhysxJointAPI`; omitted without a positive one |
| `physics:localPos0` / `localRot0` | the URDF joint origin (written when it isn't zero, or the axis is folded); `localRot0` = R(origin rpy)·Q with a folded axis |
| `physics:localPos1` / `localRot1` | `(0, 0, 0)` and Q, with a folded axis only |

The formulas are in [`physics.md`](physics.md#drives).

### World anchor

```usda
        def PhysicsFixedJoint "world_anchor"
        {
            rel physics:body0
            rel physics:body1 = </World/robots/R1/world_link>
            point3f physics:localPos0 = (1.5, 1, 0.5)
            quatf physics:localRot0 = (0.707107, 0, 0, 0.707107)
            point3f physics:localPos1 = (0, 0, 0)
            quatf physics:localRot1 = (1, 0, 0, 0)
        }
```

`body0` empty is the world; `body1` is the URDF root link (the first link that
is no joint's child); `localPos0` / `localRot0` are the robot's placement,
raised by its pedestal like the robot's Xform, so PhysX never has to snap the
joint together.

## Fixtures

```
/World/fixtures/<fixture_id>        ← Xform, kind = "component", placement translate + orient
    <link>                          ← def Xform with its matrix4d pose (FK over the fixed joints), no API schemas
        visual_<i>                  ← as a robot's, always emitted
        collision_<i>               ← PhysicsCollisionAPI (+ PhysicsMeshCollisionAPI, convexHull), purpose = "guide"
```

No rigid body, mass, contact report or sensor, no joints and no anchor: a
collider without a rigid body is static in UsdPhysics.

## Pedestals

```usda
    def Scope "pedestals"
    {
        def Xform "pedestal_R1"
        {
            double3 xformOp:translate = (1.5, 1, 0.25)
            quatd xformOp:orient = (1, 0, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]

            def Cylinder "socket" (
                prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
            )
            {
                rel material:binding = </World/Looks/factoryos_default>
                uniform token axis = "Z"
                double height = 0.5
                double radius = 0.4
                float3[] extent = [(-0.4, -0.4, -0.25), (0.4, 0.4, 0.25)]
            }
        }
    }
```

One per robot whose `pedestal.height_mm` is above 0: centred at half its height
above the robot's placement, so it spans the placement height to the raised
foot.

## Floor plan

```
/World/floor_plan                   ← Xform, xformOp:translate = (0, 0, 0.002)
    Looks/M_floorplan               ← UsdPreviewSurface: diffuseColor (0.046665, 0.061246, 0.0865), metallic 0, roughness 0.9
    lines                           ← Mesh: the general lines
    footprints                      ← Mesh: the footprint layers
```

Each mesh holds one 2.5 cm wide strip per segment, at the segment's own height:
four corners counter-clockwise from above, two triangles, vertex normals
(0, 0, 1), `extent`, `doubleSided = 1`, `subdivisionScheme = "none"`, bound to
`M_floorplan` (slate, sRGB #3d4653, linear). Segments shorter than 1 nm or not
finite are skipped; a layer without segments gets no mesh, a plan without any
no prim. The drawing has no `purpose` — Kit hides `guide` prims — and no
collider: nothing to stand on.

## Looks

`/World/Looks` holds a `Material` with a `UsdPreviewSurface` shader per OBJ
material, named `<urdf-stem>__<material>` (sanitised), and `factoryos_default`:

```usda
        def Material "arm__gripper__paint_orange"
        {
            token outputs:surface.connect = </World/Looks/arm__gripper__paint_orange/Shader.outputs:surface>

            def Shader "Shader"
            {
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = (…)
                float inputs:metallic = 0
                float inputs:roughness = …
                float inputs:opacity = 1
                token outputs:surface
            }
        }
```

- `diffuseColor` is the MTL's `Kd` decoded from sRGB to linear
  (`c ≤ 0.04045 ? c / 12.92 : ((c + 0.055) / 1.055)^2.4`; `Kd 0.5` → 0.214041); a
  material whose colour isn't known — no `Kd`, missing from its MTL, an MTL that
  can't be read — gets the default's linear 0.45 grey.
- `metallic` is 0 (MTL has no metalness), `roughness` is
  `clamp(1 / (1 + 0.05·Ns), 0.05, 1)`, `opacity` is `d` (or `1 − Tr`).
- `factoryos_default` — 0.45 grey, roughness 0.666667 — dresses meshes without
  a material, URDF primitives, the pedestals and shown hulls of unpainted links.
- An OBJ with several `usemtl` materials is bound to its first (BL-26); within
  one URDF the first MTL definition of a material name wins (BL-28).

## ActionGraphs (ROS 2 wiring)

Each robot with an actuated joint gets a `def OmniGraph "ActionGraph_<robot_id>"`
(`evaluator:type = "execution"`, `pipelineStage = "pipelineStageSimulation"`)
containing:

1. `omni.graph.action.OnPlaybackTick` — drives the per-frame chain
2. `isaacsim.core.nodes.IsaacReadSimulationTime` — sim-time stamp source
3. `isaacsim.ros2.bridge.ROS2PublishJointState` (`PublishJointState`, target
   `/World/robots/<robot_id>`) → `/<cell>/<robot>/joint_states`
4. `isaacsim.ros2.bridge.ROS2SubscribeJointState` (`SubscribeJointState`) ←
   `/<cell>/<robot>/joint_command`
5. `isaacsim.core.nodes.IsaacArticulationController` (`ArticulationController`)
   — applies the subscribed position, velocity and effort commands, triggered by
   the subscriber
6. `isaacsim.sensors.physics.IsaacReadContactSensor` (`ContactSensor_<link>`) ×N
   + matching `isaacsim.ros2.bridge.ROS2Publisher` (`PublishContact_<link>`,
   `std_msgs/msg/Bool`) on `/<cell>/<robot>/contacts/<link>` — one pair per
   collision-bearing link, the publisher triggered by the sensor's `execOut`, not
   `OnPlaybackTick.outputs:tick`, so the topic only fires when a contact event
   lands

The scene-level `def OmniGraph "ActionGraph_clock"` publishes
`rosgraph_msgs/Clock` (`isaacsim.ros2.bridge.ROS2PublishClock`, stamped by its
own `IsaacReadSimulationTime`) on `/<cell>/clock`. The ROS 2 nodes set
`inputs:nodeNamespace` (`<cell>` or `<cell>/<robot>`, sanitised) and
`inputs:topicName`; connections are authored with `add … .connect` and
relationship inputs with `add rel`, the forms the OmniGraph loader resolves
reliably.

### Joint-name mapping

`inputs:jointNames` of the articulation controller is auto-derived from the
URDF actuated-joint declaration order, with `fixed` joints filtered out — the
URDF's own names, typically `["joint_1", "joint_2", …, "joint_6"]` for a 6-axis
arm. Command and state messages use these names.

### What is NOT in the graph yet

- A `FollowJointTrajectory` action server (BL-11) — needed before
  MoveIt-style trajectory execution can target the simulator
- Cell-level collision aggregation (BL-24) — a single cell-wide topic with rich
  event data (force, normal, the other body); deferred, the per-link `Bool`
  topics are the contact feed

## Cameras

```usda
        def Camera "ISO_NE" (
            prepend apiSchemas = ["OmniRtxCameraAutoExposureAPI_1", "OmniRtxCameraExposureAPI_1"]
        )
        {
            float2 clippingRange = (…, 10000)
            float exposure:fStop = 5
            float exposure:responsivity = 1.1026709
            float exposure:time = 0.02
            float focalLength = 32.992786
            float horizontalAperture = 20.955
            float verticalAperture = 11.787187
            custom bool omni:kit:cameraLock = 1
            quatd xformOp:orient = (…)
            double3 xformOp:translate = (…)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
        }
```

`/World/Cameras` holds six cameras, in this order:

| Camera | Looks from | Elevation |
|---|---|---|
| `ISO_NE` | azimuth 45° (counter-clockwise from +X) | 26° |
| `ISO_NW` | azimuth 135° | 26° |
| `PLAN` | straight down, north (+Y) up, tilted 2° (`Rx(−2°)`) | — |
| `ISO_SE` | azimuth −45° | 26° |
| `ISO_SW` | azimuth −135° | 26° |
| `ELEVATION` | azimuth 0° (from +X) | 7° |

- They aim at the centre of the scene's bounding box — the robots' collision
  geometry at their home poses (a link without collisions with its visuals),
  the pedestals, the fixtures' visuals and collisions; not the floor plan or the
  ground — from one shared distance D: the shortest, but at least 1 m, at which
  every view frames all eight corners of the box.
- The lens is 32.992786 mm on a 20.955 × 11.787 mm film back (the 1280 × 720
  render product's aspect; about 35° × 20° field of view), the clipping range
  (max(0.01, 0.20471·D), 10000) m.
- `omni:kit:cameraLock` keeps viewport navigation from moving them, and the
  exposure APIs carry Kit's own exposure (f/5, 0.02 s, responsivity 1.1026709:
  a scale of 1/1133.6 at ISO 100), so a render product other than the
  viewport's sees the scene as bright as the viewport does.
- Poses are written at full double precision. A scene with nothing to frame
  gets no rig.

## Environment

```usda
def Xform "Environment"
{
    …
    def DistantLight "defaultLight" (
        prepend apiSchemas = ["ShapingAPI"]
    )
    {
        float inputs:angle = 0.53
        float inputs:colorTemperature = 7250
        bool inputs:enableColorTemperature = 1
        float inputs:exposure = 10
        float inputs:intensity = 1
        float inputs:shaping:cone:angle = 180
        token visibility = "inherited"
        quatd xformOp:orient = (0.33944435018566016, 0.17670354420260315, 0.42660009293331813, 0.8194911538891423)
        …
    }

    def DomeLight "domeLight" (
        prepend apiSchemas = ["ShapingAPI"]
    )
    {
        bool visibleInPrimaryRay = 0
        float inputs:colorTemperature = 6150
        bool inputs:enableColorTemperature = 1
        float inputs:exposure = 9
        float inputs:intensity = 1
        float inputs:shaping:cone:angle = 180
        token inputs:texture:format = "latlong"
        token visibility = "inherited"
        quatd xformOp:orient = (0.8558159940603218, 0, 0, 0.5172803730188719)
        …
    }

    def Scope "Looks" { def Material "M_ground" { … } }
    def Mesh "Ground" ( prepend apiSchemas = ["MaterialBindingAPI"] ) { … }
    def Plane "GroundCollider" ( prepend apiSchemas = ["PhysicsCollisionAPI"] ) { … }
}
```

- The two lights replicate Kit's "Default" viewport light rig — what Kit lights
  a stage without lights with — with every attribute its lights author: the sun
  stands 35° above the horizon in the north-east (azimuth 45°), the dome is
  unseen by the camera. Both sit at the origin (a distant or dome light's
  position doesn't matter); no `omni:rtx:usdluxVersion` is authored, as in the
  rig.
- `Ground` is one quad facing up at z = 0, bound to
  `/Environment/Looks/M_ground` (`diffuseColor` 0.18 linear, metallic 0,
  roughness 0.85). It is centred under the scene's bounding box (the origin
  without one) and reaches ten times as far from that centre as the box and the
  floor plan do, but at least 50 m: a side of 100 m or more.
- `GroundCollider` is an infinite `Plane` at z = 0 (`axis = "Z"`,
  `purpose = "guide"`, so it isn't drawn), which the robots' welded root links
  filter.

## Render

`/Render` (hidden in the Stage window) holds what Kit saves with a stage, typed
as Isaac Sim 6.x (Kit 110) writes it:

- `OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_0`, the
  viewport's `RenderProduct`: camera `/OmniverseKit_Persp`, resolution
  (1280, 720), the dome as the background (`omni:rtx:background:source:type =
  "domeLight"`), `uint` sample and region limits, ordered var `LdrColor`;
- `RenderSettings "OmniverseGlobalRenderSettings"`, with the render product;
- `Vars/LdrColor`, a `RenderVar` with `uniform token sourceType = "raw"`.

No `omni:rtx:rendermode` is authored: Isaac Sim's own default render mode
applies.

## PhysicsScene

```usda
    def PhysicsScene "PhysicsScene" (
        prepend apiSchemas = ["PhysxSceneAPI"]
    )
    {
        vector3f physics:gravityDirection = (0, 0, -1)
        float physics:gravityMagnitude = 9.81
        bool physxScene:enableGPUDynamics = 1
        uint physxScene:gpuFoundLostAggregatePairsCapacity = 8192
        uint physxScene:gpuFoundLostPairsCapacity = 262144
        uint physxScene:gpuMaxRigidContactCount = 1048576
        uint physxScene:gpuMaxRigidPatchCount = 81920
        uint physxScene:gpuTotalAggregatePairsCapacity = 4096
        uniform uint physxScene:maxPositionIterationCount = 64
        uniform uint physxScene:maxVelocityIterationCount = 4
        uint physxScene:timeStepsPerSecond = 120
    }
```

Why these names and types: [`physics.md`](physics.md#physics-scene). The GPU
capacities are sized for a dense multi-robot cell and none is below PhysX's
default. Scenes with more robots, denser fixtures, or aggressive contacts may
need higher caps; PhysX prints `needs to increase
PxGpuDynamicsMemoryConfig::*` warnings when it spills, with the exact value to
configure.

## Number format

Most numbers are written with up to six decimals, trailing zeros trimmed, in
scientific notation below 1e-6 or from 1e15 in magnitude, and a non-finite
value as 0. The camera rig's poses and exposure, the opening view, the light
orientations and the scene origin are written at full double precision (the
shortest text that reads back as the same value); the rig's clipping range and
lens use the six-decimal format.

## Determinism

Identical manifests produce byte-identical USDA output (NFR-105, checked by
the tests): materials and meshes are kept in sorted maps, links and joints are
written in URDF order and numbers in the fixed format above, and nothing depends
on the clock.

This matters for CI / regression testing: two composes of one manifest diff
clean, so a diff of two stages shows exactly what a manifest or URDF change did.
