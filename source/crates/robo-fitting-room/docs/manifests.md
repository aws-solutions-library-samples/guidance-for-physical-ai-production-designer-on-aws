# Manifest schema

Scene definitions are YAML. Positions in metres, orientations in degrees (ABC
convention = ZYX-Euler: A = yaw about Z, B = pitch about Y, C = roll about X;
the rotation is `Rz(A)·Ry(B)·Rx(C)`). URDF and floor-plan paths are resolved
relative to the manifest file's folder — for a manifest posted to the REST or
stdio validate call, relative to `<asset-root>/fitted-robots/`. Numbers may be
written as integers or decimals.

## Full example

```yaml
name: "Demo Cell"
description: "Two arms sharing one station"

# Optional. Subtracted from every placement and from the floor-plan offset
# so the composed scene is centred near the world origin (FR-106).
scene_origin: { x: 50.0, y: 20.0, z: 0.0 }

robots:
  - robot_id: "R1"
    dressed_urdf: "../urdf-dressed-robots/arm__gripper/arm__gripper.urdf"
    position: { x: 51.5, y: 21.0, z: 0.0 }
    orientation: { a: 90.0, b: 0.0, c: 0.0 }
    home_pose_deg: [0, -90, 90, 0, 45, 0]      # optional; one value per actuated joint
    pedestal: { height_mm: 500, diameter_mm: 800 }   # optional
    payload:                                     # optional; the tool as a rigid mass
      mass_kg: 80
      center_of_mass_m: { x: 0.0, y: 0.0, z: 0.25 }
      inertia_kgm2: { ixx: 4.0, iyy: 4.0, izz: 2.0 }
    link_density_kg_m3: 1000                     # optional; default 1200

  - robot_id: "R2"
    dressed_urdf: "../urdf-dressed-robots/arm__gripper/arm__gripper.urdf"
    position: { x: 48.5, y: 21.0, z: 0.0 }
    orientation: { a: -90.0, b: 0.0, c: 0.0 }

fixtures:
  - id: "station_a"
    urdf: "../urdf-fixture-collection/station_a/station_a.urdf"
    position: { x: 50.0, y: 21.0, z: 0.0 }
    orientation: { a: 0.0, b: 0.0, c: 0.0 }

floor_plan:
  geometry_file: "../floor-plans/cell-floor-plan.obj"
  scale: 1.0
  footprint_layers: [ZONE_7]                     # optional
```

Here the robots, the fixture and the floor plan share the plan's CAD
coordinates, and `scene_origin` moves the cell to the world origin: R1 composes
at (1.5, 1, 0), raised to z = 0.5 by its pedestal. R2 has no `home_pose_deg` and
starts in the default pose.

## Field reference

### Top level

| Field | Type | Required? | Notes |
|---|---|---|---|
| `name` | string | yes | Human-readable cell name. **Load-bearing**: doubles as `cell_id` in every emitted ROS 2 topic (`/<cell_id>/<robot_id>/...`), so renaming a manifest is a runtime-visible event; it is also the layer's `doc` |
| `description` | string | no | Free text; the layer's `doc` is `<name>: <description>` |
| `scene_origin` | `{x,y,z}` | no | Subtracted from every placement and from the floor-plan offset at parse time. Use to centre a real-world cell at the USD world origin. Recorded in the layer as `robo_fitting_room:scene_origin_m` |
| `robots` | array | no | One entry per articulated robot in the cell |
| `fixtures` | array | no | Static cell infrastructure (clamping fixtures, conveyors, fences) |
| `floor_plan` | object | no | 2D CAD plan, drawn on the ground (FR-107) |

### `robots[]`

| Field | Type | Required? | Notes |
|---|---|---|---|
| `robot_id` | string | yes | Stable ID. Names the prim `/World/robots/<robot_id>` and is the second segment of the robot's ROS 2 namespace; both sanitise it ([below](#names-become-prim-names-and-topics)). Uniqueness isn't checked: keep IDs unique, after sanitising too |
| `dressed_urdf` | path | yes | The robot's URDF (output of `urdf-robo-dresser dress`, or any URDF with OBJ meshes), relative to the manifest |
| `position` | `{x,y,z}` | yes | World-space metres. Subject to `scene_origin` subtraction |
| `orientation` | `{a,b,c}` | yes | Degrees, ABC = ZYX-Euler |
| `home_pose_deg` | `[number, …]` | no | One value per actuated (non-fixed) joint, in URDF declaration order: degrees on a revolute or continuous joint, metres on a prismatic one. Each revolute and prismatic value must lie within its URDF limits (where the URDF gives a `<limit>`). It is the bind pose, every drive's target and the initial joint state. Omit it for the [default pose](#the-default-home-pose) |
| `pedestal` | object | no | A mounting pedestal under the robot ([below](#pedestal)) |
| `payload` | object | no | The tool as a rigid mass ([below](#payload)) |
| `link_density_kg_m3` | number | no | The effective density (kg/m³, finite and > 0) the robot's derived link masses use, and the line below which a URDF inertial counts as a placeholder; default 1200 ([`physics.md`](physics.md#mass-model)) |

#### `pedestal`

| Field | Type | Default | Notes |
|---|---|---|---|
| `height_mm` | number | 0 | Finite and ≥ 0. Above 0 the robot (and its world anchor) is raised by it, and a static collision cylinder fills the gap, from the robot's `position.z` to its raised foot |
| `diameter_mm` | number | 800 | Finite and > 0 (checked even at height 0): the cylinder's diameter |

#### `payload`

| Field | Type | Required? | Notes |
|---|---|---|---|
| `mass_kg` | number | yes | Finite and > 0 |
| `center_of_mass_m` | `{x,y,z}` | no | Finite; metres, in the carrier link's frame. Not shifted by `scene_origin` |
| `inertia_kgm2` | `{ixx,iyy,izz}` | no | Principal moments (kg·m²) about the centre of mass, in the carrier link's axes; finite, > 0, and those of a rigid body: none larger than the sum of the other two |
| `link` | string | no | The carrier: a link of the URDF. It and the links fixed below it are the tool. Without it the carrier is the link with the most `meshes/tool/` collision meshes, else the deepest leaf link (a warning) |

A missing centre of mass or inertia comes from spreading the mass over the
tool's collision geometry ([`physics.md`](physics.md#tool-and-payload)).

### `fixtures[]`

| Field | Type | Required? | Notes |
|---|---|---|---|
| `id` | string | yes | Stable ID for the prim path (`/World/fixtures/<id>`, sanitised like a robot's) |
| `urdf` | path | yes | Path to a fixture URDF. **Must contain only `fixed` joints** — non-fixed joints fail at compose time |
| `position` | `{x,y,z}` | yes | World-space metres. Subject to `scene_origin` subtraction |
| `orientation` | `{a,b,c}` | yes | Degrees, ABC = ZYX-Euler |

### `floor_plan`

| Field | Type | Required? | Notes |
|---|---|---|---|
| `geometry_file` | path | yes | An OBJ of `l` lines ([below](#floor-plan-obj)), read while parsing: a missing file fails the parse (`Floor plan not found`). A `.dxf` is read as text and yields no line (a warning) |
| `offset` | `{x,y,z}` | no | Added to the plan's coordinates |
| `cell_origin_dxf` | `{x,y,z}` | no | The plan point that should land at the origin; used only without `offset`, as an offset of `−cell_origin_dxf`. When both are set, `offset` wins |
| `scale` | number | no | Default `1.0`. Multiplied into the plan's coordinates |
| `footprint_layers` | `[string, …]` | no | The layers or groups drawn as footprints, by name (a list, even for one name) |

A plan vertex `v` lands in the world at

```
v · scale + (offset, else −cell_origin_dxf, else 0) − scene_origin
```

#### Floor-plan OBJ

1. **Lines.** `l` elements: a line of n vertex indices is a polyline of n − 1
   segments. An index token may be `v`, `v/vt`, `v/vt/vn` or `v//vn` (the vertex
   part counts), 1-based or negative (counting back from the last vertex read);
   a pair with an index that doesn't resolve is skipped.
2. **Vertices.** `v x y z`, in metres after `scale`. They are taken Z up as
   written: a plan should be flat in Z (a Y-up export lies on its side, BL-27).
3. **Everything else** — faces (`f`), normals, texture coordinates — is ignored.
4. **Layers.** A `# Layer: <name>` comment, an `o <name>` or a `g <name>`
   directive (the name is the rest of the line) starts a layer; the `l` lines
   after it belong to it. A layer holds footprints when its name — trimmed,
   compared case-insensitively — is one of `footprint_layers`, or when it
   contains `footprint` in any case. A listed name must match the whole layer
   name: listing `ZONE` doesn't make `ZONE_7` a footprint layer.

The floor plan is drawn on the ground as `/World/floor_plan`, its general lines
and its footprints as two meshes of 2.5 cm wide strips
([`usda.md`](usda.md#floor-plan)), and its segment count is reported by compose.
It has no collider and doesn't move the camera framing.

### The default home pose

A robot without `home_pose_deg` gets the KUKA-style stowed pose
`[0, -90, 90, 0, 0, 0]` (degrees, `KUKA_HOME_POSE_DEG`) by actuated-joint index
on its revolute and continuous joints, 0 beyond the sixth joint, and 0 m on
every prismatic joint. It is checked against the URDF limits like a given pose:
a robot whose joints can't take it needs its own `home_pose_deg`.

### Names become prim names and topics

`robot_id` and `id` become prim names through `sanitize_prim_name`: a leading
digit gets a `_` in front, every other character outside `[A-Za-z0-9_]` — the
first included — becomes `_`, and a name that is exactly `World` (a robot's,
fixture's, link's or joint's) becomes `world_link`. The ROS 2 namespace uses
`sanitize_ros_segment` instead: every character outside `[A-Za-z0-9_]` becomes
`_`, and a digit-leading value gets an `r_` prefix. A robot `42` is the prim
`/World/robots/_42` and publishes on `/<cell_id>/r_42/joint_states`. Two IDs
that sanitise to the same name collide.

## Validation errors

`robo-fitting-room validate <manifest>` runs every check below (so do the
validate endpoints, and compose before it composes). Each robot and fixture
check names the `robot_id` / fixture `id`; the parse, floor-plan and URDF-file
errors name the file instead. All but the first five messages carry the prefix
`Validation failed: `.

| Error | Meaning | Fix |
|---|---|---|
| `Scene parse error: <path>: …` | The manifest can't be read | Check the path |
| `Scene parse error: yaml parse: …` | The YAML doesn't parse, a required field is missing or has the wrong type | Fix the YAML at the position the message gives |
| `Floor plan not found: <path>: …` | `floor_plan.geometry_file` can't be read | Check the path is relative to the manifest file's folder |
| `URDF not found: <path>` | A `dressed_urdf` or fixture `urdf` path doesn't resolve | Check the path is relative to the manifest file's folder |
| `URDF invalid: <path>: file does not contain a <robot> element` | The path resolves but isn't a URDF (or, with the read error instead, can't be read) | Pass a valid URDF (output of `urdf-robo-dresser dress`) |
| `Robot '<id>': non-finite position or orientation` | `.nan` / `.inf` in `position` or `orientation` | Replace with finite values |
| `Robot '<id>': joint '<joint>' has no axis direction (axis [x, y, z])` | An actuated joint's `<axis>` is zero or not finite | Give the joint its axis |
| `Robot '<id>': home_pose_deg contains non-finite values` | `.nan` / `.inf` in the home pose | Replace with finite values |
| `Robot '<id>': home_pose_deg has N entries but URDF has M actuated joints` | Length mismatch | Fix the array length, or remove the field to use the default pose |
| `Robot '<id>': home angle <v> deg of joint '<joint>' is outside its URDF limits [<lower>, <upper>] deg` (`home position <v> m … m` for a prismatic joint) | A home value — given or the default — lies outside the joint's limits (slack 1e-4° / 1e-6 m) | Choose a home value inside the limits |
| `Robot '<id>': pedestal height_mm must be finite and >= 0` | | Fix the height |
| `Robot '<id>': pedestal diameter_mm must be finite and > 0` | | Fix (or drop) the diameter |
| `Robot '<id>': payload mass_kg must be finite and > 0` | | Fix the mass |
| `Robot '<id>': payload center_of_mass_m must be finite` | | Fix the centre of mass |
| `Robot '<id>': payload inertia_kgm2 must be finite and > 0` | | Fix the moments |
| `Robot '<id>': payload inertia_kgm2 is no rigid body's: each moment must be at most the sum of the other two` | No rigid body has these principal moments | Fix the moments, or omit them to spread the mass over the tool |
| `Robot '<id>': payload link '<link>' is no link of its URDF` | `payload.link` names no link | Name a link of the robot's URDF |
| `Robot '<id>': link_density_kg_m3 must be finite and > 0` | | Fix (or drop) the density |
| `Fixture '<id>': non-finite position or orientation` | `.nan` / `.inf` in `position` or `orientation` | Replace with finite values |

These fail at compose time only (`validate` passes them):

| Error | Meaning | Fix |
|---|---|---|
| `Validation failed: Fixture '<id>' has non-fixed joint '<joint>' (type: <type>); fixtures must use fixed joints only` | Fixture URDF has a non-`fixed` joint (BL-29) | Convert to `fixed` or model it as a robot, not a fixture |
| `URDF parse error: <path>: …` | A URDF that contains `<robot` but doesn't parse; `validate` skips its joint, home-pose and payload checks | Fix the URDF |
| `USD write error: open <mesh> for OBJ parse: …` | A mesh that is missing or given as a URI (both warned about) | Ship the mesh next to the URDF, referenced by a relative path |
| `Unsupported output format: …` / `USD write error: create <path>: …` | `--output` doesn't end in `.zip`, or its folder doesn't exist | Fix the output path |

## Warnings

Warnings report what composes, but probably not as the manifest means it; they
never fail validation or compose. `validate` prints them (`WARNING:`), the
validate endpoints return them (`warnings`), compose logs them.

| Warning | Meaning |
|---|---|
| `Robot '<id>': N mesh(es) given as a URI, which compose doesn't resolve …` (or `Fixture '<id>'`) | `package://` or `file://` mesh references; compose fails where it emits them. Use paths relative to the URDF |
| `… N mesh(es) aren't OBJ files; only OBJ meshes are read, so they compose as empty prims without geometry, collider or mass …` | STL / DAE meshes (BL-25); `urdf-robo-dresser` writes OBJ |
| `… N mesh file(s) not found next to the URDF, so compose fails where it emits them …` | Missing mesh files |
| `Robot '<id>': no link has a tool collision mesh (meshes/tool/), so the payload goes on the deepest leaf link '<link>'; name its link with payload.link` | The payload's carrier is a guess |
| `Robot '<id>': the payload has no explicit center_of_mass_m and inertia_kgm2 and its tool has no collision geometry to spread it over, so it is a point mass …` | The payload becomes a point mass with a 5 cm sphere's inertia |
| `Robot '<id>': no link to carry the payload; it is ignored` | The URDF has no link |
| ``The floor plan has no line segment: an OBJ of `l` lines is read (a DXF file isn't: BL-22)`` | Nothing to draw |
| `The floor plan has no footprint: none of its N layer(s) (…) is listed in floor_plan.footprint_layers or named like a footprint, so all its lines are drawn as lines` | The plan has named layers but no footprint layer: list the footprint layers |
| `The floor plan spans <h> m in Z against <a> m across: it is read Z up, …` | The plan's Z extent exceeds a tenth of its horizontal extent: a Y-up export (BL-27) |

Each mesh warning counts the meshes and names up to three of them. Compose also
logs how many OBJ meshes use more than one material: each is bound to its first
material only (BL-26).

## The `name` field is load-bearing

Beyond display, the manifest's `name` becomes the `cell_id` segment in every
ROS 2 topic the composed USDA publishes:

```
/<cell_id>/<robot_id>/joint_states
/<cell_id>/<robot_id>/joint_command
/<cell_id>/<robot_id>/contacts/<link>
/<cell_id>/clock
```

Renaming a manifest after operators have built dashboards / consumers keyed on
the topic names is a **runtime-visible event** — they will need to retarget.
The `name` is sanitised via `sanitize_ros_segment` before being used as a topic
segment (non-alphanumerics become `_`, digit-leading values get an `r_` prefix):
`Demo Cell` publishes under `/Demo_Cell/`.

## Unknown keys

The parser ignores keys it doesn't know — there is no strict mode — so a
manifest can carry its own metadata, and the server, the stdio transport, the
URDF bundle and the editor keep such keys ([`api.md`](api.md#manifest-keys-are-kept)).
The flip side: a misspelled optional key (`home_pose` for `home_pose_deg`, say)
is silently ignored.
