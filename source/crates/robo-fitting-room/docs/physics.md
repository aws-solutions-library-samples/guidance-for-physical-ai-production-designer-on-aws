# Physics

What `compose` authors for PhysX, and why: the state every joint starts in,
the drives, the mass model they are tuned on, tool payloads, collisions and the
PhysicsScene. Everything is derived from the URDFs and the manifest; the
constants live in `src/drives.rs` (home values, mass model, drives) and
`src/compose.rs` (emission). The prims themselves are listed in
[`usda.md`](usda.md).

## Units

UsdPhysics' units, as authored:

| Quantity | Angular joint (revolute, continuous) | Linear joint (prismatic) |
|---|---|---|
| target, state, limits | degrees | metres |
| velocity limit | deg/s | m/s |
| drive stiffness | N·m/deg | N/m |
| drive damping | N·m·s/deg | N·s/m |
| drive max force | N·m | N |

Masses are in kg, lengths in m, inertias in kg·m². omni.physx converts the
angular gains to per radian (× 180/π): the formulas below work in SI units (per
radian) and the angular stiffness and damping are written × π/180.

## The start of the simulation

### Home values

Each actuated joint — every non-fixed joint, in URDF declaration order — has one
home value (`drives::actuated_home_values`), its single source for the link bind
poses, the drive targets and the initial joint states:

- **revolute**: the manifest's `home_pose_deg` entry, in degrees, unwrapped
  (190° stays 190°, which a ±350° wrist tells apart from −170°);
- **continuous**: the angle wrapped into (−180°, 180°];
- **prismatic**: the entry as given, in metres;
- **floating, planar**: 0 (they aren't emitted).

Without `home_pose_deg` the turning joints take `KUKA_HOME_POSE_DEG`
`[0, -90, 90, 0, 0, 0]` by index (0 beyond the sixth) and every prismatic joint
0 m. Validation rejects a revolute or prismatic home value outside the joint's
URDF limits by more than 1e-4° (1e-6 m) — given or default: the joint would
start in violation of its limit, which PhysX resolves with a jump before the
first step.

### Bind poses and joint states

Each link's `matrix4d xformOp:transform` is its pose relative to the robot root
at the home pose: `child = parent · T(joint origin) · M(home)`, where `M` turns
by the home angle about the joint's unit axis, as signed, or slides by the home
value along it. These bind poses keep every joint's two frames together —
omni.physx checks the bodies against the joint frames at load and warns about
"disjointed body transforms" otherwise — and show the home pose before Play.

They don't set where the simulation starts. When an articulation enters the
scene, PhysX overwrites its non-root link poses from the joint positions, and
those start at the joint's authored state — at 0 without one. So every actuated
joint carries `PhysicsJointStateAPI:angular` (`:linear` on a prismatic joint) in
the same `prepend apiSchemas` as its drive:

```usda
            float drive:angular:physics:targetPosition = 12.5
            float state:angular:physics:position = 12.5
            float state:angular:physics:velocity = 0
```

The state is the drive's target, the identical text, at rest. Without it every
arm would jump to its URDF zero pose at Play — into its neighbours — and its
drives would whip it back; with it, Play starts where the stage shows.

### Joint axes

A URDF axis along +X, +Y or +Z (within 1e-9, after normalising) is that
`physics:axis` token, with the URDF's joint frames. Any other axis — negative
(`0 0 -1`) or skewed — is written as the token `X` with the rotation Q, the
shortest rotation taking +X onto the unit URDF axis (a half turn about +Z for
−X), folded into both joint frames: `localRot0 = R(origin rpy)·Q`,
`localPos1 = (0, 0, 0)`, `localRot1 = Q`. The joint's +X is then the URDF axis in
both bodies, so the angle (or slide) keeps the URDF's sign: limits, targets,
states and the ROS joint states and commands are all in the URDF's convention
and agree with the bind poses. A joint without `<axis>` turns about URDF's
default, +X; a zero axis on an actuated joint fails validation.

Revolute and continuous joints become `PhysicsRevoluteJoint` (a continuous one
without limits), prismatic joints `PhysicsPrismaticJoint`, fixed joints
`PhysicsFixedJoint`; floating and planar joints are skipped with a warning.

## Drives

Every actuated joint gets a position drive (`PhysicsDriveAPI:angular` or
`:linear`, `drives::drive_params`), tuned on the mass model at the home pose.
Let k be its stiffness and I the composite inertia it moves, both in SI units.

**Moved inertia.** The joint moves its child link and every link below it.
For a turning joint, I is their composite moment of inertia about the joint
axis — the line through the child link's origin along the axis — at the home
pose:

```
I = Σ_i  aᵀ·(R_i·I_i·R_iᵀ)·a + m_i·|d⊥,i|²
```

with `a` the unit axis, `R_i` link i's orientation, `I_i` its inertia tensor
about its centre of mass, `m_i` its mass and `d⊥,i` the offset of its centre of
mass from the axis, perpendicular to it. For a prismatic joint, I is the moved
mass M = Σ m_i.

| Attribute | Value |
|---|---|
| `stiffness` | angular: 1.5e5 N·m/deg (`DRIVE_STIFFNESS`; k = 8.594e6 N·m/rad). Linear: k = max(M, 0.1 kg)·g / δ N/m, with δ = 1 mm (`LINEAR_DRIVE_SAG_M`) |
| `damping` | D = 2·ζ·√(k·I) with ζ = 0.7 (`DRIVE_DAMPING_RATIO`); written × π/180 on an angular joint |
| `maxForce` | the URDF `effort` when it is finite, > 0 and below 1e30; otherwise the derived cap below |
| `targetPosition` | the home value |
| `physxJoint:maxJointVelocity` | the URDF `velocity` in deg/s (m/s on a prismatic joint), via `PhysxJointAPI`; omitted when the URDF gives no positive one |

**Why these gains.** omni.physx solves joint drives implicitly, so the gains
don't limit the step size; they set each joint's natural frequency and, with
the inertia it moves, its damping ratio. 8.6e6 N·m/rad is a natural frequency
√(k/I) of about 99 rad/s on a moved inertia of about 880 kg·m², a large arm's.
A linear drive's stiffness follows the mass it moves: the axis sags 1 mm holding
its load against gravity, and its natural frequency is √(g/δ) ≈ 99 rad/s
(15.8 Hz) whatever it carries — a fixed stiffness would leave a heavy carriage
soft and a light one stiff. The damping is computed per joint on the inertia it
actually moves, so every joint is damped at ζ = 0.7.

**The derived force cap.** An effort of 1e30 or more (`PLACEHOLDER_EFFORT`) is a
placeholder, not a limit: robot URDFs often carry FLT_MAX (3.4e38) where their
exporter knew no effort, and `urdf-robo-dresser` copies a robot's `<limit>` into
the dressed URDF as it is. An effort of 0 or none (a `<limit>` without `effort`
reads as 0) is no limit either. Such a joint gets a finite cap that doesn't
limit nominal tracking:

```
maxForce = max(1, 2·τ_hold + I·v / 0.25 s + D·v)
```

- τ_hold = g·Σ m_i·(r_i + |c_i|), a bound on gravity's torque in any pose: r_i
  the summed lengths of the joint offsets from the child link down to link i,
  |c_i| the distance of its centre of mass from its origin. On a prismatic
  joint τ_hold = g·M, the moved weight.
- I·v / 0.25 s accelerates the moved inertia from rest to the velocity limit v
  in 0.25 s (`CAP_ACCELERATION_TIME_S`); v is the URDF velocity, else 1 rad/s
  (1 m/s, `CAP_FALLBACK_VELOCITY`).
- D·v is the drive's own damping drag at that velocity: the drive tracks
  position commands against its damping.

g is 9.81 m/s², the PhysicsScene's gravity.

### Worked examples

From the crate's tests. A revolute joint about +Z moving one 10 kg link whose
centre of mass is 1 m out along X, inertia diag(0.1, 0.2, 0.3) kg·m², URDF
effort FLT_MAX, velocity 2 rad/s:

| | |
|---|---|
| I | 0.3 + 10·1² = 10.3 kg·m² |
| stiffness | 1.5e5 N·m/deg (k = 8.594e6 N·m/rad) |
| D | 2·0.7·√(8.594e6 · 10.3) = 13 172 N·m·s/rad, written as `damping = 229.89576` (N·m·s/deg) |
| maxForce | 2·9.81·10·1 + 10.3·2/0.25 + 13 172·2 ≈ 26 623 N·m |
| maxJointVelocity | 2 rad/s, written as 114.591559 deg/s |

A prismatic joint moving a 5 kg carriage, URDF effort 0, velocity 0.5 m/s:

| | |
|---|---|
| stiffness | 5·9.81 / 0.001 = 49 050 N/m (natural frequency √(g/δ) ≈ 99 rad/s) |
| damping | 2·0.7·√(49 050 · 5) ≈ 693.3 N·s/m |
| maxForce | 2·9.81·5 + 5·0.5/0.25 + 693.3·0.5 ≈ 454.8 N |
| maxJointVelocity | 0.5 m/s |

A slide moving 60 kg is 588 600 N/m stiff.

## Mass model

Every link's `PhysicsMassAPI` — `physics:mass`, `physics:centerOfMass`,
`physics:diagonalInertia` and `physics:principalAxes` — comes from one function,
`drives::link_mass_properties`, which the drive tuning above uses too. There is
no table for particular robot models. A link is the sum of two bodies.

**The robot's body**, of the link's robot geometry: its collision elements that
aren't the tool's, or — when it has no such collision — its visual elements
that aren't. Meshes count with their URDF scale and origin applied, their volume
V by the divergence theorem (signed tetrahedra on each closed mesh's triangles,
an inward-wound mesh with its sign flipped); a mesh with fewer than 4 points or
triangles, or enclosing no volume, is skipped with a warning. URDF boxes,
cylinders and spheres count analytically. Then:

1. the URDF inertial as authored — its mass, its origin as the centre of mass,
   its full tensor (products of inertia too) turned into the link's axes by the
   origin's rpy — when it is usable (finite, mass > 0) and no placeholder: the
   link has no robot geometry (V = 0), or the inertial's mass over V is at least
   a quarter (`PLACEHOLDER_DENSITY_FRACTION`) of the link density;
2. else, with robot geometry, that geometry at the link density: mass ρ·V, the
   solid's centroid and inertia;
3. else none.

**The tool's body**, of the link's tool collision geometry: the robot's payload
when this link carries it ([below](#tool-and-payload)), else the tool hulls at
the link density.

The two are combined — the masses add up, the centre of mass is their
mass-weighted mean, the inertia about it is the parallel-axis sum
I + m·(|d|²·E − d·dᵀ) — and diagonalised into principal moments and axes; axes
within 1e-9 of the link's own are written as the identity, (1, 0, 0, 0).

**A link with neither** body gets a fallback, so that no rigid body is massless
and PhysX never falls back to its "negative mass, small sphere" path:

| Link | Mass | Inertia |
|---|---|---|
| the URDF root, or welded to it through fixed joints only (a `world_link`, a base frame) | 10 kg | (1, 1, 1) kg·m² |
| a frame link a joint moves (a flange, `tool0`, a TCP) | 0.1 kg | 1e-4 kg·m², a solid 5 cm sphere's |

The welded links can't move (the world anchor holds them), so their mass loads
no joint; a frame link's 0.1 kg doesn't skew the chain it hangs on or the drives
tuned on it.

**The link density** ρ is the robot's manifest `link_density_kg_m3` (finite and
> 0, else a validation error), by default 1200 kg/m³
(`DEFAULT_LINK_DENSITY_KG_M3`). It is mass per volume of the link's closed
collision geometry, not of its material: a link's casting, motors and gearboxes
fill only part of the hull around them. For heavy six-axis industrial arms the
datasheet mass over the summed convex hulls of their links is of the order of
10³ kg/m³, more or less with how closely the hulls wrap (looser hulls, such as a
convex decomposition's, give a lower figure), so a derived arm can come out
above or below its datasheet mass: set `link_density_kg_m3` to the datasheet
mass over the links' summed hull volume to match it.

Example: on a link whose hull encloses 0.1 m³, a 2 kg URDF inertial (20 kg/m³,
below 300 kg/m³ = 0.25 · 1200) is a placeholder and the link becomes 120 kg with
the hull's centroid and inertia; a 60 kg inertial (600 kg/m³) is kept with its
tensor. With `link_density_kg_m3: 2500` the line is 625 kg/m³, and the 60 kg is a
placeholder too. `RUST_LOG=debug` logs each inertial taken for a placeholder.

## Tool and payload

A robot's tool is found by `tool::ToolLayout`. The **carrier** — the link a
payload merges into — is:

1. the manifest's `payload.link`, when it names a link of the URDF;
2. else the link carrying the most `meshes/tool/` collision meshes (the first on
   a tie);
3. else the deepest leaf link — with a warning, since it is a guess.

The **tool** is the carrier and the links joined below it through fixed joints
only. `--full-tools` and, in the mass model, the split of each link's collisions
into the robot's and the tool's take every `meshes/tool/` mesh (visuals and
collisions) as the tool's, on whichever link it is — or, for a named carrier
whose tool links have none (a URDF whose tool doesn't follow the dresser's
layout), all the geometry of the tool's links. A payload is spread over the tool
collisions of the tool's links only.

The manifest's `payload: {mass_kg, center_of_mass_m?, inertia_kgm2?, link?}`
replaces the tool's own geometry in the carrier's mass:

- an explicit `center_of_mass_m` (carrier frame) and `inertia_kgm2` (principal
  moments about the centre of mass, in the carrier's axes) are taken as given;
- a missing one comes from spreading the mass over the tool's collision geometry
  at uniform density: every tool link's tool hulls and primitives, brought into
  the carrier's frame by the bind poses at home — their centroid, and their
  inertia about it, times the mass;
- with no volume to spread over, the payload is a point mass at the hull
  vertices' centroid (else at the carrier's origin) with a 5 cm solid sphere's
  inertia, never a zero tensor (which UsdPhysics ignores); `validate` warns when
  the tool's links have no tool collision geometry at all.

The tool's geometry isn't weighed at the link density on top: the carrier's
tool body is the payload, and the tool's other links weigh only their own robot
geometry or inertials (else a frame's 0.1 kg); a `meshes/tool/` hull on a link
outside the tool still weighs the link density. The merged link is combined and
diagonalised as above, and its joints' drives are tuned on it. Without a payload
the tool hulls weigh the link density like the rest of the robot.

## Collisions and filtering

- A URDF collision mesh becomes a `Mesh` with `PhysicsCollisionAPI` +
  `PhysicsMeshCollisionAPI`, `physics:approximation = "convexHull"`: PhysX
  collides with the convex hull of each mesh, such as the hulls
  `urdf-robo-dresser` generates. A URDF box, cylinder or sphere becomes its
  `Cube`, `Cylinder` or `Sphere` with `PhysicsCollisionAPI` and collides as that
  exact shape.
- A robot doesn't collide with itself (`physxArticulation:enabledSelfCollisions
  = 0`); it collides with the other robots, the fixtures, the pedestals and the
  ground.
- Every robot link with a collision reports its contacts:
  `PhysxContactReportAPI` with threshold 0 (every contact) and an
  `IsaacContactSensor` child feeding the link's ROS 2 contact topic
  ([`concepts.md`](concepts.md#contact-graphs)).
- The welded root links — the URDF root and every link fixed to it — carry
  `PhysicsFilteredPairsAPI` against their own pedestal's cylinder (if the robot
  has one) and `/Environment/GroundCollider`. The base hull sits a few
  millimetres into what it stands on and neither can move, so their contact
  would only flood the base's contact reports. Other links, and other robots'
  pedestals, aren't filtered.
- Static colliders — collision prims without a rigid body, which nothing can
  move: every fixture link, every pedestal cylinder, and the ground collider (an
  infinite plane at z = 0). Fixtures carry no mass, contact report or sensor;
  the floor-plan drawing has no collider at all.
- No physics materials (friction, restitution), collision groups or contact
  offsets are authored: PhysX's defaults apply.

## Physics scene

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

- `PhysxSceneAPI` is the applied-schema name of PhysX's scene API
  (`PhysxSchemaPhysxSceneAPI` is only its C++ class name). OpenUSD skips an
  unknown name in `apiSchemas` without a warning, and PhysX reads its
  attributes by the schema's names and types, so every name and type is the
  schema's: the GPU capacities and the step rate are `uint`, the iteration caps
  `uniform uint`.
- Gravity is UsdPhysics' `physics:gravityDirection` / `physics:gravityMagnitude`:
  9.81 m/s² along −Z.
- 120 steps per second (dt ≈ 8.3 ms). The drives don't bound the step (they are
  implicit); contacts and the once-per-step articulation forces do.
- `maxPositionIterationCount` / `maxVelocityIterationCount` cap every actor's
  own iteration counts, so the caps let the articulations' 64 position and 4
  velocity iterations through; a lower cap would silently lower them.
- The GPU capacities are sized for a dense multi-robot cell with fixtures, none
  below PhysX's default.
- No `NewtonSceneAPI`: Isaac Sim 6.x applies it itself.

Each robot's articulation root adds `solverPositionIterationCount = 64`,
`solverVelocityIterationCount = 4`, `sleepThreshold = 0.005` and
`stabilizationThreshold = 0.001`.

## What the manifest controls

| Manifest field | Effect on the physics |
|---|---|
| `home_pose_deg` | bind poses, drive targets, initial joint states — and so the composite inertias the damping and caps are computed on |
| `payload` | the carrier's mass properties, and every drive that moves it |
| `link_density_kg_m3` | the derived link masses and the placeholder line, and the drives tuned on them |
| `pedestal` | the robot's height, the pedestal collider, and the welded links' filter |

The drive constants (`DRIVE_STIFFNESS`, `DRIVE_DAMPING_RATIO`,
`LINEAR_DRIVE_SAG_M`, `CAP_ACCELERATION_TIME_S`), the fallback masses and the
default link density are constants in `src/drives.rs`, not manifest settings.

## Not modelled yet

- STL and DAE meshes (BL-25): they compose as empty prims, without collider or
  mass; `validate` warns.
- Movable fixture joints (BL-29): compose rejects a fixture's non-fixed joint.
- Static collision checks at the home pose (FR-131) and reach envelopes
  (FR-130).
- A home-limit slack taken from the precision a URDF limit is written with
  (BL-30): a limit written as `1.57` doesn't admit a 90° home.
