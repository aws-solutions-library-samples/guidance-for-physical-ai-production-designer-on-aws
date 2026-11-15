# CLI reference

`urdf-robo-dresser` ships four one-shot subcommands (`dress`, `batch`,
`validate`, `list`) plus the long-running REST server (`serve`), which
also speaks MCP on stdin/stdout when an MCP client spawns it. They
run the same library functions (`parse_urdf`, `validate`,
`collision::generate_hulls`, `merge`, `serialize_urdf`,
`catalogue::scan`). The server adds what only a server needs (the
dressed-robot listing, the viewer's preload bundles, raw catalogue
bytes under `/assets/` and the 3D viewer), and it writes its results
into the catalogue (`dress`) or a temp zip (`batch`) rather than to an
`--output` path.

```
$ urdf-robo-dresser --help        (abridged)
urdf-robo-dresser — robot+tool URDF merger (CLI + REST server)
…
Usage: urdf-robo-dresser <COMMAND>

Commands:
  dress     [CLI] Merge ONE robot URDF with ONE tool URDF and write the dressed URDF to disk
  batch     [CLI] Permutation-merge every robot × every tool in a catalogue
  validate  [CLI] Statically validate a URDF file
  list      [CLI] List robots + tools discoverable in a catalogue
  serve     [SERVER] Boot the HTTP server: 3D viewer SPA at `/`, REST API at `/api/v1`, Swagger UI at `/docs`
  help      Print this message or the help of the given subcommand(s)
…
```

`urdf-robo-dresser <cmd> --help` lists a subcommand's flags; `dress`,
`batch` and `validate` add examples, `serve` a route table, and
`--version` prints the crate version. The help's example paths
(`../assets/…`, `gripper_tool`) don't exist in this workspace: use
`assets/catalogue/…` and `sticktool`, as below.

The REST `serve` mode is documented separately in [`api.md`](api.md).

## Building

The crate is a member of the workspace, whose default member is the
viewer, so name the package when building from the workspace root:

```sh
cargo build --release -p urdf-robo-dresser                         # CLI + REST server and MCP
cargo build --release -p urdf-robo-dresser --no-default-features   # CLI only: no `serve`, no axum/tokio/utoipa
```

Inside `crates/urdf-robo-dresser/` a plain `cargo build --release`
builds this crate. Either way the binary lands in the workspace's
`target/release/`. The examples below call it `urdf-robo-dresser` and
run from the workspace root, whose catalogue is `assets/catalogue/`.

| Feature | Effect |
|---|---|
| `server` (default) | The `serve` subcommand and the `server` module |
| `mcp` (default) | MCP on stdin/stdout beside `serve`'s REST API ([below](#mcp-beside-rest)), and the `mcp` module; implies `server` |
| `real-assets` | Enables `tests/real_assets.rs` ([Testing](#testing)) |
| `viewer` | Declared; gates nothing |
| `full` | `server` + `viewer` |

`dress --view` needs `urdf-viz` on `PATH` (`cargo install urdf-viz`).

---

## `dress` — merge a robot with a tool

The core operation. Parses both URDFs, auto-detects the flange link,
computes the tool orientation, generates convex-hull collisions, and
writes a self-contained dressed URDF carrying the tool's TCP frame and
the reach envelope.

```sh
urdf-robo-dresser dress \
    --robot   <ROBOT_URDF> \
    --tool    <TOOL_URDF> \
    --output  <OUTPUT_URDF> \
    [--orient X,Y,Z] \
    [--hull tight|medium|max] \
    [--attach-to LINK_NAME] \
    [--view]
```

| Flag | Default | Description |
|---|---|---|
| `--robot` | required | Robot base URDF |
| `--tool` | required | Tool URDF to attach |
| `--output` | required | Dressed URDF path; `meshes/` lands beside it (folders are created) |
| `--orient` | `0,0,0` (= auto) | Tool rotation `x,y,z` in degrees, each `0`, `90`, `180` or `270`, applied as URDF roll / pitch / yaw about the tool's origin in the flange link's frame. `0,0,0` runs [auto-orient](concepts.md#auto-orientation-algorithm), so the unrotated mount can't be forced |
| `--hull` | `medium` | Collision hulls: one convex hull per visual mesh (OBJ or binary STL), its vertices scaled about their centroid by 1.00 (`tight`), 1.05 (`medium`) or 1.15 (`max`). Lower case only ([hull buffers](concepts.md#hull-buffers)) |
| `--attach-to` | auto | Link to mount the tool, and its TCP frame, on. Default: the deepest leaf link, a joint's child that is no joint's parent, with the most joints up to the root (ties: the name that sorts first). `tcp::dressed_tcp` reads the TCP back from the robot's tip link (the child of its last movable joint) |
| `--view` | off | After writing, run `urdf-viz <output>` and wait until it is closed; a missing `urdf-viz` is ignored |

`dress` prints `Dressed URDF written to <output>` on stdout. The
diagnostics of both inputs go to stderr, one line each,
`<error|warning> [<code>] [<file>:<element>]: <message>`: errors stop
the run, warnings don't (when the robot has errors, the tool's
diagnostics aren't shown). A failed hull generation is a `warning: …`
and the dress goes on. Exit codes:

- `0`: written.
- `1`: a bad `--orient` or `--hull` (`error: orientation must be 0, 90,
  180, or 270 — got 45`), a URDF that can't be read or parsed, a
  validation error, an unknown `--attach-to` link (`error: URDF parse
  error in : attach-to link '<name>' not found in robot`), or a failed
  mesh copy or write.
- `2`: a usage error (a missing or unknown flag).

### Pipeline

1. Parse the robot and tool URDFs (typed kinematic trees)
2. Validate both, the tool in tool mode: errors (no root or unreachable
   links, a dangling parent or child, duplicate names) stop the run;
   warnings (a missing mesh, `lower >= upper`, a visual without
   collision) don't
3. Generate the convex hulls (`--hull`), tool first, then robot, into
   `meshes/tool/` and `meshes/robot/` beside the output
4. Resolve the flange link: `--attach-to`, else the deepest leaf
5. Find the flange face, the farthest point of the flange link's visual
   meshes along its joint axis; the tool's origin (its docking
   interface) goes there
6. Orientation: `--orient`, or, when it is `0,0,0`, the best of 64
   candidates (`{0, 90, 180, 270}` per axis)
7. Inject every tool visual and collision into the flange link, rotated
   and moved to the flange face as one rigid group
8. Copy the meshes (visuals, hulls, each mesh's `.mtl`) into
   `meshes/robot/` and `meshes/tool/` and rewrite their paths
9. Compute the [reach envelope](concepts.md#reach-envelope): a
   `<reach_envelope>` element and `meshes/3dreach/reach_envelope.obj`
10. Add the tool's [TCP](concepts.md#tool-centre-point-tcp), when it
    declares one, as a fixed `tcp_joint` from the flange link to a
    `tcp` link
11. Write the URDF (deterministic serialization)

### Worked examples

```sh
# Single dress, auto-orient
urdf-robo-dresser dress \
    --robot  assets/catalogue/urdf-robot-collection/kr240r2900/kr240r2900.urdf \
    --tool   assets/catalogue/urdf-tool-collection/sticktool/sticktool.urdf \
    --output /tmp/kr240r2900__sticktool/kr240r2900__sticktool.urdf

# Force orientation + flange link (like_kr240r2900_link_6 is also the detected flange)
urdf-robo-dresser dress \
    --robot  assets/catalogue/urdf-robot-collection/kr240r2900/kr240r2900.urdf \
    --tool   assets/catalogue/urdf-tool-collection/sticktool/sticktool.urdf \
    --output /tmp/kr240r2900__sticktool-0-90-0/kr240r2900__sticktool.urdf \
    --attach-to like_kr240r2900_link_6 \
    --orient 0,90,0

# Generous collision margin (safety zones / low-quality mesh inputs)
urdf-robo-dresser dress \
    --robot  assets/catalogue/urdf-robot-collection/kr240r2900/kr240r2900.urdf \
    --tool   assets/catalogue/urdf-tool-collection/sticktool/sticktool.urdf \
    --output /tmp/kr240r2900__sticktool-max/kr240r2900__sticktool.urdf \
    --hull max --view
```

Give each dressed URDF its own folder: the `meshes/` beside it is shared
by everything written there, and every run overwrites the hulls and the
reach envelope in it.

### Output layout

```
/tmp/kr240r2900__sticktool/
├── kr240r2900__sticktool.urdf
└── meshes/
    ├── robot/
    │   ├── like_kr240r2900_*_visuals_*.obj (+ .mtl)   robot visual meshes
    │   └── like_kr240r2900_*_hull_0.obj               convex hulls, in place of the robot's collisions
    ├── tool/
    │   ├── <tool visual mesh>.obj (+ .mtl)            tool visual meshes
    │   └── World_hull_0.obj … World_hull_4.obj        tool hulls (link World, one per visual)
    └── 3dreach/
        └── reach_envelope.obj                         convex hull of the sampled tool-tip positions
```

The URDF starts with the `<reach_envelope …/>` element, carries the
tool's visuals and collisions on the flange link, and ends with the
`tcp` link and the fixed `tcp_joint` when the tool declares a TCP
([Dressed URDF](concepts.md#dressed-urdf)). Hulls are named
`<link>_hull_<n>.obj`, `<n>` the visual's index in its link.

Every mesh path in the URDF is relative to it (`./meshes/robot/…`,
`./meshes/tool/…`, `./meshes/3dreach/…`), so the output directory can
be moved or zipped, provided every source mesh was found. Source paths
are resolved against the source URDF's folder; `package://` and
`file://` URIs are not resolved. A mesh that isn't found is not copied,
yet its reference is still rewritten (`validate` reports it as `W003`).
Meshes are flattened into `meshes/robot/` and `meshes/tool/` by file
name, so two source meshes with the same name end up as one. Only each
mesh's same-stem `.mtl` comes along, no textures. Files already in
those folders are kept, not overwritten: start from an empty folder
after changing a source mesh.

---

## `batch` — every robot × every tool

Walks `<catalogue>/urdf-robot-collection/` × `urdf-tool-collection/`
and writes one dressed URDF per pair under `<output>/<robot>__<tool>/`.
Continues past individual failures and prints a final tally.

```sh
urdf-robo-dresser batch \
    --catalogue <CATALOGUE_DIR> \
    --output    <OUTPUT_DIR> \
    [--hull tight|medium|max]
```

`--hull` works as for `dress`. Every pair gets the detected flange and
auto-orient (`batch` has no `--attach-to` or `--orient`) and, unlike
`dress`, no validation: a broken input shows up as a parse or merge
failure. Pairs run in directory-listing order, and a re-run rewrites
their folders in place. Progress goes to stdout
(`  <robot>__<tool> -> <path>`); skips, hull warnings and
`  FAILED <robot>__<tool>: <error>` go to stderr. Exit codes:

- `0`: every pair was written.
- `1`: any failure, reported after all pairs ran; or, up front, a bad
  `--hull` or a collection folder that is missing or holds no
  `<name>/<name>.urdf` (`error: robot collection not found at …`,
  `error: no tool URDFs found in …`).

A pair folder or URDF that can't be written aborts the run (a panic).

Catalogue layout — each robot and each tool lives in its own
directory matching its URDF stem; other folders are ignored:

```
<catalogue>/
├── urdf-robot-collection/
│   ├── <robot_a>/
│   │   ├── <robot_a>.urdf
│   │   └── meshes/
│   └── <robot_b>/...
└── urdf-tool-collection/
    ├── <tool_1>/
    │   ├── <tool_1>.urdf
    │   └── meshes/
    └── <tool_2>/...
```

Output, each pair laid out like `dress`'s output:

```
<output>/
├── <robot_a>__<tool_1>/
│   ├── <robot_a>__<tool_1>.urdf
│   └── meshes/
├── <robot_a>__<tool_2>/...
└── <robot_b>__<tool_1>/...
```

### Worked example

```sh
urdf-robo-dresser batch \
    --catalogue assets/catalogue \
    --output    /tmp/all-dressed
```

```
Found 1 robot(s) and 1 tool(s)
  kr240r2900__sticktool -> /tmp/all-dressed/kr240r2900__sticktool/kr240r2900__sticktool.urdf

Batch complete: 1 success, 0 failed
```

With `RUST_LOG=info`, stderr also shows each pair's hull and
auto-orient lines ([Environment variables](#environment-variables)).

`--output assets/catalogue/urdf-dressed-robots` is the recommended way
to populate the `urdf-dressed-robots/` catalogue that
[`robo-fitting-room`](../../robo-fitting-room/) consumes; here that
rewrites the checked-in `kr240r2900__sticktool/` in place.

---

## `validate` — static URDF check

```sh
urdf-robo-dresser validate <URDF_PATH> [--tool]
```

Exits `0` when there is no error (warnings are printed but don't fail
it) and `1` on any error, or when the file can't be read or parsed
(`error: URDF parse error in <path>: …`). Each diagnostic is one line on
stderr, `<error|warning> [<code>] [<file>:<element>]: <message>`; on
success stdout gets `Valid URDF: <links> links, <joints> joints`. No
filesystem writes — safe in CI on every URDF change.

| Check | Code |
|---|---|
| Duplicate link or joint names | `E014` error |
| A joint's parent or child link isn't declared | `E012` error |
| No root link (a cycle), or, with several roots, a link reachable from none | `E011` error |
| `lower >= upper` in a revolute or prismatic joint's `<limit>` | `W013` warning |
| A mesh file that doesn't exist (resolved against the URDF's folder) | `W003` warning |
| A link with visuals but no collision | `W016` warning |

`--tool` adds no check yet: the tool-origin check is a stub
([validation rules](concepts.md#validation-rules)).

```sh
# Robot URDF
urdf-robo-dresser validate \
    assets/catalogue/urdf-robot-collection/kr240r2900/kr240r2900.urdf
# Valid URDF: 8 links, 7 joints

# Tool URDF (tool mode)
urdf-robo-dresser validate \
    assets/catalogue/urdf-tool-collection/sticktool/sticktool.urdf \
    --tool
# Valid URDF: 2 links, 1 joints
```

---

## `list` — discover the catalogue

```sh
urdf-robo-dresser list --catalogue <DIR> [--json]
```

`list` scans `urdf-robot-collection/` and `urdf-tool-collection/` (each
entry `<name>/<name>.urdf`) and validates every entry, reading only.
Default output is a table with two sections (robots, tools):

```
$ urdf-robo-dresser list --catalogue assets/catalogue
ROBOTS (1):
  Name                           Flange                         Joints  Links  Status
  ------------------------------------------------------------------------------------------
  kr240r2900                     like_kr240r2900_link_6              7      8  valid

TOOLS (1):
  Name                                      Visuals  Status
  -----------------------------------------------------------------
  sticktool                                       5  valid
```

With `--json`, emits a stable shape suitable for piping into other
tools:

```json
{
  "robots": [
    {"name": "kr240r2900", "flange": "like_kr240r2900_link_6", "joints": 7, "links": 8, "status": "valid"}
  ],
  "tools": [
    {"name": "sticktool", "visuals": 5, "status": "valid"}
  ]
}
```

`name` is the entry's folder name, the id `POST /api/v1/dress` takes;
`flange` is the detected flange link and `visuals` the tool's visual
count. `status` is `valid` (no diagnostics), `incomplete` (any
validation error or warning) or `error` (the URDF doesn't parse).
Entries are sorted by name. The REST listing names its fields
differently ([`api.md`](api.md#catalogue-listing)). Dressed robots
aren't listed; `GET /api/v1/catalogue/dressed` lists them. `list` exits
`1` with `No assets found in <dir>` when neither collection has an
entry (e.g. `--catalogue assets` instead of `assets/catalogue`), and `0`
otherwise, `incomplete` and `error` entries included.

Useful for shelling out from CI or another binary that needs to know
what the catalogue contains before issuing dress requests.

---

## `serve` — REST + Swagger UI

```sh
urdf-robo-dresser serve --catalogue <DIR> [--port <PORT>] [--web-root <DIR>]
```

| Flag | Default | Description |
|---|---|---|
| `--catalogue` | required | Catalogue root the server scans + sandboxes API requests under. It must exist: `serve` resolves it at start and exits `1` otherwise |
| `--port` | `3030` | TCP port |
| `--web-root` | — | Serve the 3D viewer from this folder instead of the copy embedded at build time; development only (below) |

`serve` binds `127.0.0.1` only (there is no `--host`), prints a banner
to stderr (the listening address, the viewer and `/docs` URLs, the
catalogue, the SPA source, `embedded` or `disk (<dir>)`, and whether MCP
is on) and runs until killed, or until the MCP client that spawned it
closes stdin ([below](#mcp-beside-rest)); a port in use exits `1`. There is no authentication;
CORS answers loopback origins only, so another site's page open in
your browser can't read the API's responses or send it a JSON request. Unlike the CLI,
the server writes into the catalogue: `POST /api/v1/dress` puts its
result in `<catalogue>/urdf-dressed-robots/<robot_id>__<tool_id>/`.
Batch jobs build their zip in the system temp folder and are kept in
memory only.

`--web-root` serves the viewer's files from disk, for live JS editing
without rebuilding the binary; the embedded copy is taken from `web/`
at build time, and `build.rs` re-runs the build whenever a file there
changes. Use it on a machine you don't share: the server joins a
request's URL path to the folder without checking it, so a path with
`..` segments reads files outside it.

Available in default builds (`server` is a default Cargo feature).
Absent only with `--no-default-features`. See [`api.md`](api.md) for
the full REST surface.

### MCP beside REST

With the default `mcp` feature, `serve` checks its stdin once, after
binding the port:

- A terminal: no MCP; the banner says
  `MCP             off (stdin is a terminal)`.
- Anything else — an MCP client, a pipe, `/dev/null`: the process also
  speaks MCP on stdin/stdout, every operation of the spec a tool,
  answered by the same router and state (a batch job started over MCP
  shows over REST); the banner says
  `MCP             stdio, artifacts in <dir>`. stdout then carries the
  MCP messages only.
- Once the client has sent a request, the end of stdin shuts REST down
  gracefully and `serve` exits `0`; an MCP failure shuts it down too,
  with `error: IO error: MCP: …` and exit code `1`.
- If stdin ends before any request (`/dev/null` under a container
  runtime, `nohup` or a service manager), MCP stops quietly and REST
  runs on.

Zips the tools return are saved in `<dir>`, a new folder in the system
temp folder for every process (mode 0700 on Unix), kept after exit. An
MCP client
starts its own `serve`, so its registration needs a port of its own
(`--port 3031`, say). Tools, arguments, results and the Claude Code and
Kiro registrations: [`api.md`](api.md#mcp-transport).

---

## Library API

Both transports (CLI and REST) call the same lib functions. Run from
the workspace root:

```rust
use std::path::Path;

use urdf_robo_dresser::collision::{generate_hulls, HullBuffer};
use urdf_robo_dresser::{merge, parse_urdf, serialize_urdf, tcp, validate, MergeOptions, Severity};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot_path = Path::new("assets/catalogue/urdf-robot-collection/kr240r2900/kr240r2900.urdf");
    let tool_path = Path::new("assets/catalogue/urdf-tool-collection/sticktool/sticktool.urdf");
    let (robot_dir, tool_dir) = (robot_path.parent().unwrap(), tool_path.parent().unwrap());
    let out_dir = Path::new("/tmp/kr240r2900__sticktool");

    let mut robot = parse_urdf(robot_path)?;
    let mut tool = parse_urdf(tool_path)?;

    let mut diags = validate(&robot, robot_path, false);
    diags.extend(validate(&tool, tool_path, true));
    if diags.iter().any(|d| d.severity == Severity::Error) {
        return Err(format!("{diags:?}").into());
    }

    // The hulls replace the collisions; they land in <out_dir>/meshes/{tool,robot}/.
    generate_hulls(&mut tool, tool_dir, out_dir, "tool", HullBuffer::Medium)?;
    generate_hulls(&mut robot, robot_dir, out_dir, "robot", HullBuffer::Medium)?;

    let opts = MergeOptions {
        output_dir: out_dir.to_path_buf(),
        robot_urdf_dir: robot_dir.to_path_buf(),
        tool_urdf_dir: tool_dir.to_path_buf(),
        ..MergeOptions::default() // attach_to: None = detected flange, orient: [0, 0, 0] = auto-orient
    };
    let dressed = merge(&robot, &tool, &opts)?; // copies the meshes, writes meshes/3dreach/
    std::fs::write(out_dir.join("kr240r2900__sticktool.urdf"), serialize_urdf(&dressed))?;

    if let Some((tip, pose)) = tcp::dressed_tcp(&dressed) {
        println!("TCP on {tip} at {:?}", pose.xyz);
    }
    Ok(())
}
```

- `MergeOptions` implements `Default` (`attach_to: None`, `orient:
  [0, 0, 0]`, folders `.`). `merge` reads `attach_to`, `orient`,
  `output_dir`, `robot_urdf_dir` and `tool_urdf_dir`; `offset` and
  `tool_prefix` are accepted but ignored (FR-021 is pending, and no
  tool links are added or renamed).
- The URDF folders must be the ones the URDFs' mesh paths are relative
  to: with another folder, no hull is generated and every mesh path
  dangles, silently.
- `merge` is not pure: it creates `<output_dir>/meshes/robot/` and
  `meshes/tool/`, copies the meshes into them and writes
  `meshes/3dreach/reach_envelope.obj`; `generate_hulls` writes the hull
  OBJs. Writing the URDF is left to the caller (`serialize_urdf`
  returns the bytes).
- Crate root: `parse_urdf(path)`, `parse_urdf_bytes(bytes, source)`,
  `validate(urdf, path, is_tool) -> Vec<Diagnostic>`, `merge`,
  `serialize_urdf(urdf) -> Vec<u8>`; the URDF types (`UrdfRobot`,
  `Link`, `Joint`, `JointType`, `JointLimit`, `Origin`, `Visual`,
  `Collision`, `Geometry`, `Inertial`, `InertiaMatrix`), `MergeOptions`,
  `ReachEnvelope`, and `DresserError`, `Diagnostic`, `Severity`.
- Modules: `collision` (`generate_hulls`, `HullBuffer`), `catalogue`
  (`scan(root) -> Catalogue`, with the `to_json` and `print_table` that
  `list` prints), `reach` (`compute_reach_envelope`), `mesh_reader` (OBJ
  and binary-STL vertices, AABB, centroid), `tcp` (the
  [TCP](concepts.md#tool-centre-point-tcp): `tool_tcp`, `tcp_frame`,
  `tip_link`, `dressed_tcp`, and helpers that add or move the frame in
  an already-dressed URDF), and `parser`, `serializer`, `validator`,
  `merger`, `types`, `error`. With the `server` feature, `server`
  (`router(state)`, async `run(host, port, catalogue, web_root)`); with
  `mcp`, `mcp` (`facade(router, catalogue)`: the spec's operations as
  MCP tools over that router).
- An embedder that needs only the library depends on the crate with
  `default-features = false`, which leaves axum, tokio and utoipa out;
  `factoryos-backend` and `robo-fitting-room` do.

## Environment variables

Every subcommand logs through env_logger to stderr, each record
prefixed `[<time> <LEVEL> <module>]`; stdout carries only the results
shown above, and `serve`'s MCP messages. Without `RUST_LOG` only errors are logged, and the dresser
logs none, so it is silent by default.

| Variable | Effect |
|---|---|
| `RUST_LOG=warn` | Tools that declare no TCP (`tool '<name>' declares no tcp link: the dressed robot gets no TCP frame`) and hull OBJs that couldn't be written |
| `RUST_LOG=info` | Also the hulls generated per link, the auto-orient pick and the tool meshes it loaded, and `serve`'s listening address |
| `RUST_LOG=debug` | Also the tool direction vector auto-orient scores against. It applies to every crate that logs; `RUST_LOG=urdf_robo_dresser=debug` limits it to the dresser |
| `URD_UPDATE_SNAPSHOTS=1` | Tests only: `tests/openapi_snapshot.rs` rewrites `openapi.json` instead of comparing |

## Testing

```sh
cargo test -p urdf-robo-dresser                          # unit tests, tests/server.rs and tests/mcp.rs, OpenAPI snapshot
cargo test -p urdf-robo-dresser --no-default-features    # CLI-only build: the server tests compile to nothing
cargo test -p urdf-robo-dresser --test requirements      # the FR/NFR spec; fails by design on its todo!()s
URD_UPDATE_SNAPSHOTS=1 cargo test -p urdf-robo-dresser --test openapi_snapshot   # refresh openapi.json after an API change
cargo test -p urdf-robo-dresser --features real-assets --test real_assets -- --nocapture   # opt-in real-catalogue tests
```

`tests/openapi_snapshot.rs` compares the spec generated from the
handlers with the checked-in `openapi.json`
([`api.md`](api.md#schema-of-record)). `tests/requirements.rs` maps
every requirement in [REQUIREMENTS.md](../REQUIREMENTS.md) to a
function named after its FR/NFR ID: implemented ones assert, pending
ones panic with a `todo!()`, which is why the file is left out of plain
`cargo test` (`test = false` in `Cargo.toml`).

The tests that read real URDFs look for them under `<crate>/../assets/`
(`crates/assets/`), not under this workspace's `assets/catalogue/`: the
unit tests of the parser, the serializer round trip, flange detection,
the merge and the OBJ reader return early and pass, every test in
`tests/real_assets.rs` (which also expects tools this catalogue doesn't
have) prints `Skipping real-asset test — missing under …` and passes,
and the catalogue tests in `tests/requirements.rs` fail.
