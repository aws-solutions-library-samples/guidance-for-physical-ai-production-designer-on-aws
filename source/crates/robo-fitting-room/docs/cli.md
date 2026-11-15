# CLI reference

`robo-fitting-room` ships four subcommands. `validate` and `compose` are
one-shot: they read a manifest and exit. `serve` is the long-running HTTP
server (REST API + the 2D editor, see [`api.md`](api.md)), which also speaks
MCP on stdin/stdout when an MCP client spawns it
([`api.md`](api.md#mcp-transport)); `stdio-serve` offers the same operations
as newline-delimited JSON over stdin/stdout, without opening a port
([`api.md`](api.md#stdio-transport)).

| Build | Subcommands |
|---|---|
| default (features `server`, `mcp` and `stdio`) | `validate`, `compose`, `serve` (with MCP), `stdio-serve` |
| `--no-default-features --features server` | `validate`, `compose`, `serve` without MCP (no rmcp in the build) |
| `--no-default-features` | `validate`, `compose` |
| `--no-default-features --features stdio` | `validate`, `compose`, `stdio-serve`, without axum, tokio, utoipa or rmcp in the build |

## Building

The crate is a member of the workspace, whose default member is the viewer, so
name the package when building from the workspace root:

```sh
cargo build --release -p robo-fitting-room
cargo build --release -p robo-fitting-room --no-default-features
cargo build --release -p robo-fitting-room --no-default-features --features stdio
```

Inside `crates/robo-fitting-room/` a plain `cargo build --release` builds this
crate. Either way the binary lands in the workspace's `target/release/`. The
examples below call it `robo-fitting-room`; `<asset-root>` is an asset tree laid
out as [`serve`](#serve-rest-api-and-2d-editor) expects.

```
$ robo-fitting-room --help        (abridged)
robo-fitting-room — manifest-driven scene composition + 2D scene editor
…
Manifest-driven robotic-cell scene composer (CLI + REST server)

Usage: robo-fitting-room <COMMAND>

Commands:
  validate     [CLI] Validate a scene manifest (YAML structure + URDF paths + placement values)
  compose      [CLI] Compose a manifest into a USDA stage zip ready to load in Isaac Sim
  serve        [SERVER] Boot the HTTP server: 2D editor SPA at `/`, REST API at `/api/v1`, Swagger UI at `/docs`
  stdio-serve  [STDIO] Long-running JSON request/response loop over stdin/stdout
  help         Print this message or the help of the given subcommand(s)
…
```

`<command> --help` lists a subcommand's flags (`serve --help` adds a route
table); `--version` prints the crate version. The quick start at the end of the
top-level help uses example paths.

---

## `validate`: check a scene manifest

```sh
robo-fitting-room validate <asset-root>/fitted-robots/demo-cell.yaml
```

```
Scene: "Demo Cell" — 2 robots, 1 fixtures, floor_plan: yes
OK: all checks passed
```

Exits `0` when there are no errors. Each error is printed to stderr as
`  ERROR: <message>` and the exit code is `1`; a manifest that doesn't parse
(`Scene parse error: …`, `Floor plan not found: …`) prints a single `ERROR: …`
line instead of the `Scene:` line. Warnings — what composes, but probably not as
the manifest means it — go to stderr as `  WARNING: <message>` and never fail
the check; here for the demo manifest without its `footprint_layers: [ZONE_7]`:

```
Scene: "Demo Cell" — 2 robots, 1 fixtures, floor_plan: yes
  WARNING: The floor plan has no footprint: none of its 2 layer(s) (WALLS, ZONE_7) is listed in floor_plan.footprint_layers or named like a footprint, so all its lines are drawn as lines
OK: all checks passed (1 warning(s))
```

Validation covers:

- every `dressed_urdf` and fixture `urdf` exists and contains a `<robot>` element
- finite `position` and `orientation` values (NaN / infinity rejected)
- every actuated joint has an axis direction (a zero axis is rejected)
- `home_pose_deg`: finite values, one per actuated joint, and every revolute or
  prismatic home value — the given one, or the default when it is omitted —
  inside the joint's URDF limits (slack 1e-4° / 1e-6 m)
- `pedestal`: `height_mm` finite and ≥ 0, `diameter_mm` finite and > 0
- `payload`: mass finite and > 0, centre of mass finite, principal moments
  finite, > 0 and those of a rigid body, `link` a link of the URDF
- `link_density_kg_m3` finite and > 0
- the floor-plan file can be read (it is read while parsing)

Not caught by `validate`, but fatal at compose time: a fixture URDF with a
non-fixed joint, a URDF that contains `<robot` but doesn't parse (its joint,
home-pose and payload checks are then skipped), a mesh that is missing or given
as a URI (both are warnings). Duplicate IDs aren't checked. Every message, with
its fix, is listed in [`manifests.md`](manifests.md#validation-errors).

`validate` only reads (the manifest, its URDFs and its floor plan): no file is
written, no network is used. Safe to run in CI on every manifest change.

---

## `compose`: build the simulation scene

```sh
robo-fitting-room compose \
    --manifest <asset-root>/fitted-robots/demo-cell.yaml \
    --output   /tmp/demo-cell.zip
```

```
Composing scene "Demo Cell" → /tmp/demo-cell.zip ...
  Robot R1: dressed_urdf = arm__gripper.urdf
  Robot R2: dressed_urdf = arm__gripper.urdf
  Fixture station_a: urdf = station_a.urdf
  Robots: collision hulls only (use --full-robots for visuals)
  Tools: full visual fidelity
Written: /tmp/demo-cell.zip (2 robots, 1 fixtures, <segments> floor-plan segments)
  Inline meshes: <meshes> (<vertices> vertices, <faces> faces)
  Zip: <bytes> bytes (usda: <bytes> bytes)
```

`compose` validates first, exactly as `validate` does, and stops with exit
code `1` on any error. The scene's warnings, and a count of OBJ meshes with more
than one material (only the first is bound, BL-26), are logged to stderr
(env_logger: `RUST_LOG`, default `info`). The segment count is every parsed
floor-plan line, footprints included; the mesh count, vertices and faces are the
inlined mesh prims'.

| Flag | Default | Description |
|---|---|---|
| `--manifest` / `-m` | required | The scene manifest YAML |
| `--output` / `-o` | required | The `.zip` to write, created or overwritten. Its extension must be `zip`, its folder must exist |
| `--backend` | `isaac-sim` | The only backend; any other value exits with `ERROR: backend '<name>' not yet supported (only 'isaac-sim')` (MuJoCo / Gazebo are BL-04 / BL-05) |
| `--full-robots` | off | A switch: emit the robots' visual meshes. Without it the robot visuals are dropped and the robots' collision hulls are shown, in their link's paint |
| `--full-tools` | `true` | Takes a value: `--full-tools=false` drops the tool visuals and shows the tool hulls |
| `--urdf-bundle` | off | Also write `<output-stem>-urdf.zip` next to the output: the scene's source files as a self-contained package ([below](#the-urdf-source-bundle)) |

Tool geometry is a mesh whose path contains `meshes/tool/` (any case, either
separator), as `urdf-robo-dresser` lays it out, or — for a URDF whose tool lives
elsewhere — every visual and collision of the manifest's `payload.link` and the
links fixed below it ([`physics.md`](physics.md#tool-and-payload)). Fixtures
always come with their full visuals and hidden hulls. Neither flag changes the
physics: collision geometry, masses, joints, drives and sensors are the same in
every combination.

The zip holds exactly two entries, in this order: the manifest, under its own
file name and byte for byte, and `<manifest-stem>.usda`, the stage with every
mesh inlined — no per-mesh references, no asset-converter step at load time.
The USDA is named after the manifest, not after `--output`.
[`usda.md`](usda.md) walks through its content.

### Worked examples

```sh
# Default: robot collision hulls + tool visuals
robo-fitting-room compose -m scene.yaml -o /tmp/scene.zip

# Full visual fidelity, for screenshots and demos
robo-fitting-room compose -m scene.yaml -o /tmp/scene.zip --full-robots

# Hulls only: the smallest zip, for batch runs
robo-fitting-room compose -m scene.yaml -o /tmp/scene.zip --full-tools=false

# Plus the URDF source bundle: writes /tmp/scene.zip and /tmp/scene-urdf.zip
robo-fitting-room compose -m scene.yaml -o /tmp/scene.zip --urdf-bundle
```

### Visual fidelity matrix

| `--full-robots` | `--full-tools` | Mesh prims emitted | What the viewport shows |
|---|---|---|---|
| off (default) | true (default) | every hull + the tool visuals | robot hulls (painted) + tool visuals |
| off | false | every hull | robot and tool hulls |
| on | true | every hull + every visual | full robot and tool visuals, hulls hidden |
| on | false | every hull + the robot visuals | robot visuals + tool hulls |

Every collision hull is emitted in all four (it is the physics geometry), so
`off` / `false` gives the smallest zip and `on` / `true` the largest; how large
depends on the URDFs' meshes — `compose` prints the inline mesh count and the
zip and USDA sizes of each run.

### The URDF source bundle

`--urdf-bundle` writes a second zip, `<output-stem>-urdf.zip` in the output's
folder, after the USDA zip, which it doesn't change:

```
Bundling source URDFs → /tmp/scene-urdf.zip ...
Written: /tmp/scene-urdf.zip (<n> robot URDFs, <n> fixture URDFs, <n> files, floor plan: yes)
  Bundle zip: <bytes> bytes
```

Its entries, in this order, without a top-level folder:

1. `<manifest-stem>.yaml` — the manifest, re-serialised with every key it has;
   only `robots[].dressed_urdf`, `fixtures[].urdf` and
   `floor_plan.geometry_file` are rewritten to the bundled paths (comments and
   formatting are not kept).
2. `robots/<folder>/…` and `fixtures/<folder>/…` — each distinct URDF directory,
   copied byte for byte with everything in it (sorted; symlinks are skipped).
   Robots sharing a dressed URDF share its folder. The folder is the
   directory's name with every character outside `[A-Za-z0-9_-]` replaced by
   `_`, and `-2`, `-3`, … on a clash.
3. `floorplans/<plan-stem>/<plan file>` — the floor-plan file.

A URDF that references a mesh by an absolute path or outside its own directory
fails the bundle (`… cannot build a self-contained bundle`). Extracted anywhere,
the bundled manifest validates and composes without the original tree, and two
runs give byte-identical bundles.

---

## `serve`: REST API and 2D editor

```sh
robo-fitting-room serve --asset-root <asset-root>
```

| Flag | Default | |
|---|---|---|
| `--host` | `127.0.0.1` | Bind address: an IPv4 literal, or an IPv6 one in brackets (`[::1]`); a host name such as `localhost` is rejected. Keep the loopback default (below) |
| `--port` | `3040` | TCP port |
| `--asset-root` | required | The asset tree every request path is resolved in, and sandboxed to. It must exist, else `ERROR: Validation failed: asset_root <dir> not found: …` and exit code `1` |
| `--web-root` | — | Serve the editor from this folder instead of the copy embedded at build time; development only (below) |

The asset tree:

```
<asset-root>/
├── fitted-robots/                             scene manifests (the editor saves here; validate bodies resolve here)
├── urdf-dressed-robots/<id>/<id>.urdf         the robot palette
├── urdf-fixture-collection/<id>/<id>.urdf     the fixture palette
└── floor-plans/                               *.obj floor plans (*.dxf are listed, not read)
```

The server prints a banner to stderr — the listening address, the URL to open,
the asset root, where the editor comes from (`embedded (rebuild to update)`
or `<dir> (live)`) and whether MCP is on — and runs until killed, or until the
MCP client that spawned it closes stdin ([below](#mcp-beside-rest)). Served
paths:

| Path | What |
|---|---|
| `/` | 2D scene-layout editor (any other unmatched path returns it too) |
| `/docs` | Swagger UI |
| `/openapi.json` | OpenAPI 3 spec |
| `/healthz` | Liveness probe: `{status: "ok", version}` |
| `/readyz` | Readiness probe: 200 once the asset root resolves and holds a `*.urdf` within four folder levels, 503 otherwise |
| `/api/v1/...` | Versioned REST surface — see [`api.md`](api.md) |
| `/api/...` | Deprecated unversioned aliases of the same routes |

There is no authentication. CORS answers loopback origins only, so another
site's page open in your browser can't read the API's responses or send it a
JSON request, but every program on this machine can. Keep the loopback bind, and reach the server from another machine through an
SSH port forward (`ssh -L 3040:127.0.0.1:3040 <host>`) rather than a wider
`--host`.

`--web-root` serves the editor's files from disk, for live JS editing without
rebuilding the binary. It is for development only: the server joins a request's
URL path to the folder without checking it, so a path with `..` segments reads
any file the server process can read. Use it on the loopback bind, on a machine
you don't share:

```sh
robo-fitting-room serve --asset-root <asset-root> --web-root crates/robo-fitting-room/web
```

The embedded copy is taken from `web/` at build time; `build.rs` re-runs the
build whenever a file there changes. The editor loads js-yaml from
`web/vendor/` (vendored, see its `VENDOR.md`), so the browser needs no internet
access.

### MCP beside REST

With the default `mcp` feature, `serve` checks its stdin once, after binding
the port (a bind failure still exits `1`):

- A terminal: no MCP; the banner says `MCP            off (stdin is a terminal)`.
- Anything else — an MCP client, a pipe, `/dev/null`: the process also speaks
  MCP on stdin/stdout, every operation of the spec a tool, answered by the same
  router and state; the banner says `MCP            stdio, artifacts in <dir>`.
  stdout then carries the MCP messages only.
- Once the client has sent a request, the end of stdin shuts REST down
  gracefully and `serve` exits `0`; an MCP failure shuts it down too, with
  `ERROR: IO error: MCP: …` and exit code `1`.
- If stdin ends before any request (`/dev/null` under a container runtime,
  `nohup` or a service manager), MCP stops quietly and REST runs on.

Zips the tools return are saved in `<dir>`, a new folder in the system temp
folder for every process (mode 0700 on Unix), kept after exit. An MCP client
starts its own
`serve`, so its registration needs a port of its own (`--port 3041`, say).
Tools, arguments, results and the Claude Code and Kiro registrations:
[`api.md`](api.md#mcp-transport).

---

## `stdio-serve`: JSON over stdin/stdout

```sh
robo-fitting-room stdio-serve --asset-root <asset-root>
```

One JSON request per line on stdin, one JSON response per line on stdout,
logs on stderr; it exits `0` when stdin closes. Every method mirrors a REST
route, behind the same sandbox and library calls. `--asset-root` must exist, as
for `serve`; `--web-root` is accepted, for parity with `serve`, and ignored.
Protocol and methods: [`api.md`](api.md#stdio-transport).

```sh
printf '%s\n' '{"id":1,"method":"health.check"}' | robo-fitting-room stdio-serve --asset-root <asset-root>
# {"id":1,"result":{"status":"ok","version":"<version>"}}
```

---

## Logging

Every subcommand logs through env_logger to stderr: `RUST_LOG` sets the level
(default `info`; `RUST_LOG=debug` shows, among others, which URDF inertials were
taken for placeholders). Stdout carries only the results: the lines shown above,
the responses of `stdio-serve`, and the MCP messages of `serve`.

## Library API

Every transport (CLI, REST, stdio) calls the same library functions:

```rust
use std::path::Path;

use robo_fitting_room::{
    compose_stage_with_options, compose_urdf_bundle, parse_scene, scene_warnings,
    validate_scene, ComposeOptions,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let manifest = Path::new("fitted-robots/demo-cell.yaml");
    let scene = parse_scene(manifest)?;

    let errors = validate_scene(&scene); // Vec<FittingRoomError>; empty = valid
    assert!(errors.is_empty(), "{errors:?}");
    for warning in scene_warnings(&scene) {
        eprintln!("WARNING: {warning}");
    }

    let opts = ComposeOptions { full_robots: false, full_tools: true };
    let report = compose_stage_with_options(&scene, manifest, Path::new("demo-cell.zip"), &opts)?;
    println!("{} meshes, {} bytes", report.mesh_count, report.zip_bytes);

    let bundle = compose_urdf_bundle(&scene, manifest, Path::new("demo-cell-urdf.zip"))?;
    println!("{} files", bundle.file_count);
    Ok(())
}
```

- `parse_scene(path)` reads the manifest and its floor plan and resolves
  relative paths against the manifest's folder; `parse_scene_from_yaml(text,
  dir)` does the same for a manifest in memory.
- `compose_stage_with_options` and `compose_urdf_bundle` don't validate: call
  `validate_scene` first, as the CLI, REST and stdio transports do.
  `compose_stage(scene, manifest, out)` composes with `ComposeOptions::default()`
  (`full_robots: false`, `full_tools: true`).
- `ComposeReport` carries `zip_path`, `usda_name`, `manifest_name`,
  `robot_count`, `fixture_count`, `floor_plan_segments`, `mesh_count`,
  `total_vertices`, `total_faces`, `usda_bytes` and `zip_bytes`;
  `UrdfBundleReport` carries `zip_path`, `scene_yaml_name`, `robot_urdf_count`,
  `fixture_urdf_count`, `bundle_dir_count`, `floor_plan_bundled`, `file_count`
  and `zip_bytes`.
- The building blocks are public modules too: `drives` (home values, mass
  model, drives), `camera_rig`, `floor_plan`, `tool`, `mtl`, `obj`, `math3`.
- An embedder that needs only the library depends on the crate with
  `default-features = false`, which leaves axum, tokio and utoipa out.

## Testing

```sh
cargo test -p robo-fitting-room                       # unit + integration tests (server, MCP and stdio included)
cargo test -p robo-fitting-room --test requirements   # the FR/NFR spec; fails by design on its todo!()s
RFR_UPDATE_SNAPSHOTS=1 cargo test -p robo-fitting-room --test openapi_snapshot   # refresh openapi.json
# every manifest of a real asset tree; from the workspace root, with an absolute
# path, since the test runs in crates/robo-fitting-room/
RFR_ASSET_ROOT=$PWD/assets/catalogue cargo test -p robo-fitting-room \
    --features real-assets --test real_assets -- --nocapture
```

`tests/requirements.rs` maps every requirement in
[REQUIREMENTS.md](../REQUIREMENTS.md) to a function named after its FR/NFR ID.
Implemented requirements have real assertions; pending ones panic with a
descriptive `todo!()` — green = implemented, red = pending — which is why the
file is left out of plain `cargo test` (`test = false` in `Cargo.toml`).

`tests/real_assets.rs` validates every manifest in `<asset-root>/fitted-robots/`
and composes the smallest; without `RFR_ASSET_ROOT` it looks for
`<crate>/../assets` (`crates/assets/`, which this workspace doesn't have, so
every test is skipped), and it skips manifests whose files are missing (and
every test when none is left).
