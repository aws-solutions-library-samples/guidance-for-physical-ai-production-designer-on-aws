# REST API, MCP and stdio transports

The `serve` subcommand boots an in-binary HTTP server hosting the 2D
scene-layout editor and a versioned REST API over the compose pipeline;
`stdio-serve` offers the same operations as newline-delimited JSON over
stdin/stdout ([below](#stdio-transport)). Every CLI operation — validate,
compose, the URDF bundle — is reachable through both; all three transports call
the same Rust library functions, so behaviour is identical. When an MCP client
spawns `serve`, it also offers every REST operation as an MCP tool on
stdin/stdout, answered by the same handlers ([MCP transport](#mcp-transport)).

## Schema of record

The authoritative wire schema is the **OpenAPI 3 spec** at
[`openapi.json`](../openapi.json) (checked into the crate root). The running
server serves it too, with Swagger UI at `/docs` to try any endpoint without
writing curl:

```sh
curl -s http://localhost:3040/openapi.json | jq '.info.version'   # → "0.2.0", the API's version
curl -s http://localhost:3040/healthz      | jq '.version'        # → "<version>", the crate's
```

`tests/openapi_snapshot.rs` compares the spec generated from the handlers'
`#[utoipa::path]` annotations with the checked-in file, so any change to a
route, parameter or schema fails the tests until the snapshot is refreshed
deliberately:

```sh
RFR_UPDATE_SNAPSHOTS=1 cargo test -p robo-fitting-room --test openapi_snapshot
```

The spec lists the 11 `/api/v1/*` routes and the two probes. It doesn't list the
deprecated aliases, the editor's files or the stdio transport. Two of its texts
are older than the code: `JointInfo.kind` names only `revolute`, `continuous`
and `prismatic` (`urdf/joints` lists floating and planar joints too), and
`info.license` says `MIT OR Apache-2.0`, where the crate is MIT-0.

## Versioning policy

- `/api/v1/...` is the stable contract: its paths and shapes change only
  additively.
- An unversioned `/api/...` alias of every `/api/v1/...` route is still served,
  answering with `Deprecation: true` and `Sunset: use /api/v1` headers; the spec
  announces their removal in the next release. The embedded editor still calls
  these aliases.
- A change to the spec always shows up as a diff of `openapi.json`: the snapshot
  test fails until the file is refreshed.

## Authentication, CORS, limits

There is no authentication, and the request limits are axum's defaults.
Every endpoint is open to every client that reaches the port. CORS answers
loopback origins only (`http[s]://localhost`, `127.0.0.0/8`, `[::1]`, any
port), so a page served from this machine may call the API from the browser,
but another site's page open in the same browser can neither read a response
nor send a JSON `POST` (no CORS headers for its origin, no credentials for
any). Run the server on a trusted network —
local dev, a sidecar on a cluster-internal service, or behind a reverse proxy
that handles auth. The default bind address is `127.0.0.1`. An MCP client that
spawns `serve` brings this API up too, for as long as its session runs
([MCP transport](#mcp-transport)). The server doesn't change axum's
request-body limit (2 MB in axum 0.8), which matters for large OBJ uploads.

## Health & readiness

| Path | Method | Returns |
|---|---|---|
| `/healthz` | GET | `200` as soon as the server accepts connections: `{"status":"ok","version":"<version>"}` (the crate version) |
| `/readyz` | GET | `200` with `{"ready":true,"asset_root":…,"urdf_count":N}` once the asset root resolves and at least one `*.urdf` lies within four folder levels below it; otherwise `503` with `{"ready":false,"asset_root":…,"urdf_count":0,"errors":[…]}` — a readiness body, not the error envelope |

```sh
curl -s http://localhost:3040/healthz | jq .
curl -s http://localhost:3040/readyz  | jq .
```

These probes sit at the **root**, not under `/api/v1` — the Kubernetes
convention every probe configurator expects out of the box.

## Error envelope

Every error a handler returns carries a typed JSON body:

```json
{
  "code": "invalid_path",
  "message": "Validation failed: asset path may not contain '..': ../etc/passwd"
}
```

- `code` is a stable machine-readable token (no spaces, no version drift),
  matchable in client code.
- `message` is human-readable; errors from the library keep their prefix
  (`Validation failed: …`, `Scene parse error: …`).
- `hint` is optional, only present with an actionable remediation — today on
  `manifest_invalid`: `"POST the manifest body to /api/v1/validate to inspect each error"`.

The shape is `components.schemas.ErrorEnvelope` in the spec. The codes:

| Status | Codes |
|---|---|
| 400 | `invalid_path`, `invalid_manifest_path`, `manifest_parse_failed`, `manifest_invalid`, `urdf_parse_failed`, `floor_plan_parse_failed`, `floor_plan_unsupported`, `multipart_invalid`, `missing_filename`, `missing_file_body`, `unsupported_extension` |
| 404 | `manifest_not_found` |
| 500 | `compose_failed`, `bundle_failed`, `zip_read_failed`, `response_build_failed`, `mkdir_failed`, `write_failed`, `floor_plans_mkdir_failed` |

Not every non-2xx response is an envelope: requests axum rejects before a
handler runs — a body that isn't the JSON the route expects (or lacks
`Content-Type: application/json`), a missing query parameter, a wrong method
(`405`) — get axum's own plain-text or empty bodies, and `/readyz`'s `503`
carries the readiness body. An unknown path, under `/api/` too, falls through to
the editor and returns its `index.html`.

## Endpoints

Request bodies, query parameters and response shapes are in the spec; Swagger UI
at `/docs` is the interactive reference. Every path parameter is relative to
`--asset-root` ([sandboxing](#path-sandboxing)).

| Path | Method | Tag | Request | Response (200) |
|---|---|---|---|---|
| `/api/v1/assets/robots` | GET | assets | — | `[{id, urdf_rel}]`: every `urdf-dressed-robots/<id>/` that holds `<id>.urdf`, sorted by id (`[]` without the folder) |
| `/api/v1/assets/fixtures` | GET | assets | — | the same under `urdf-fixture-collection/` |
| `/api/v1/assets/floor-plans` | GET | assets | — | `[{name, ext, rel}]`: the `.obj` and `.dxf` files directly in `floor-plans/` (any case; `ext` lower case), sorted by name |
| `/api/v1/assets/floor-plans/upload-obj` | POST | assets | multipart: `filename` (text) and `file` (the OBJ) | `{stored_path, bytes}` |
| `/api/v1/urdf/joints` | POST | urdf | `{path}` | `{joints: [{name, kind, axis, lower_deg, upper_deg, lower_m?, upper_m?}]}` |
| `/api/v1/floor-plan` | POST | floor-plan | `{path, footprint_layers?}` | `{segments, footprint_segments}` |
| `/api/v1/validate` | POST | compose | the manifest YAML as the raw body | `{ok, errors, warnings?}` |
| `/api/v1/compose` | POST | compose | query `manifest_path`, `full_robots` (default `false`), `full_tools` (default `true`) | `application/zip`, `Content-Disposition: attachment; filename="<stem>.zip"` |
| `/api/v1/compose-urdf-bundle` | POST | compose | query `manifest_path` | `application/zip`, `filename="<stem>-urdf.zip"` |
| `/api/v1/scenes/load` | POST | scene | `{path}` | the file's text, `application/x-yaml` |
| `/api/v1/scenes/save` | POST | scene | `{path, yaml}` | `{ok: true, bytes}` |
| `/healthz` | GET | system | — | see above |
| `/readyz` | GET | system | — | see above |

Details, per endpoint:

- **upload-obj** — the name is the `filename` field (else the `file` part's own
  file name), cut to its last path component, with every character outside
  `[A-Za-z0-9._-]` replaced by `_`; it must end in `.obj` (any case). The file
  is written to `floor-plans/<name>` (the folder is created; an existing file
  is overwritten) and `stored_path` is that path, relative to the asset root.
  Errors: 400 `multipart_invalid`,
  `missing_filename`, `missing_file_body`, `unsupported_extension`; 500
  `floor_plans_mkdir_failed`, `write_failed`.
- **urdf/joints** — every non-fixed joint, in URDF declaration order: `kind`
  (`revolute`, `continuous`, `prismatic`, `floating` or `planar`), `axis` (the
  URDF's `xyz` as written, `[1, 0, 0]` without `<axis>`) and the `<limit>`
  bounds: degrees in `lower_deg` / `upper_deg` on a revolute or continuous joint,
  metres in `lower_m` / `upper_m` on a prismatic one (whose `*_deg` are `null`;
  `*_m` appear only on prismatic joints with a limit). The editor builds its
  home-pose form from it. Errors: 400 `invalid_path`, `urdf_parse_failed`.
- **floor-plan** — the OBJ's line segments, each `[[x, y, z], [x, y, z]]`, in
  the file's own coordinates (offset 0, scale 1), split as compose splits them:
  `footprint_layers` (alias `footprintLayers`) as in the manifest, plus every
  layer named like a footprint ([`manifests.md`](manifests.md#floor-plan-obj)).
  Errors: 400 `invalid_path`; `floor_plan_unsupported` for a `.dxf`
  (``<path>: DXF floor plans aren't read (BL-22); export the plan as an OBJ of `l` lines``);
  `floor_plan_parse_failed` for a file that can't be read.
- **validate** — always `200`: `ok` is the verdict, `errors` the validation
  errors as strings, `warnings` ([`manifests.md`](manifests.md#warnings)) present
  only when there are some. The body is the manifest text (UTF-8; the content
  type isn't checked); its relative paths resolve against
  `<asset-root>/fitted-robots/`, where the editor keeps manifests. A manifest
  that doesn't parse gives `ok: false` with its one parse error.
- **compose** — resolves `manifest_path`, parses the manifest (its paths
  relative to its own folder), validates it, composes it into a temporary
  `.__compose_<stem>.zip` next to the manifest, streams that back and deletes
  it. The manifest's folder must be writable; two concurrent composes of one
  manifest share the temporary name. Errors: 400 `invalid_manifest_path`,
  `manifest_parse_failed`, `manifest_invalid` (the validation errors joined
  with `; `, with the hint above); 500 `compose_failed`, `zip_read_failed`,
  `response_build_failed`.
- **compose-urdf-bundle** — the same flow for the URDF source bundle
  ([`cli.md`](cli.md#the-urdf-source-bundle)), through `.__urdf_bundle_<stem>.zip`;
  `bundle_failed` in place of `compose_failed`.
- **scenes/load** — the file's text as it is. Errors: 400 `invalid_path`, 404
  `manifest_not_found` (any read failure).
- **scenes/save** — writes `yaml` verbatim to `path`, creating its folders;
  any file name inside the asset root is allowed. `bytes` is the YAML's length
  in bytes. Errors: 400 `invalid_path`; 500 `mkdir_failed`, `write_failed`.

## Compose recipe

End-to-end, as the editor's **Generate USDA** does it: validate the manifest,
save it under the asset root, compose it, download the zip.

```sh
# 1. Validate (relative paths resolve against <asset-root>/fitted-robots/)
curl -s -X POST http://localhost:3040/api/v1/validate \
    -H "Content-Type: application/x-yaml" \
    --data-binary @demo-cell.yaml | jq .

# 2. Save it as fitted-robots/demo-cell.yaml
jq -Rs '{path: "fitted-robots/demo-cell.yaml", yaml: .}' demo-cell.yaml |
  curl -s -X POST http://localhost:3040/api/v1/scenes/save \
    -H "Content-Type: application/json" --data-binary @- | jq .

# 3. Compose; -o captures the zip
curl -s -X POST -o /tmp/demo-cell.zip \
    "http://localhost:3040/api/v1/compose?manifest_path=fitted-robots/demo-cell.yaml&full_robots=false&full_tools=true"
unzip -l /tmp/demo-cell.zip

# 4. Optionally, the URDF source bundle
curl -s -X POST -o /tmp/demo-cell-urdf.zip \
    "http://localhost:3040/api/v1/compose-urdf-bundle?manifest_path=fitted-robots/demo-cell.yaml"
```

Both compose routes are `POST`; a `GET` gets `405`.

## Path sandboxing

Every path parameter (`path`, `manifest_path`) is resolved against the
configured `--asset-root` by one check, `AppState::resolve_asset`
(`src/transport_common/state.rs`), shared by REST and stdio:

- an absolute path is rejected: `asset path must be relative: <path>`;
- so is any `..` path component: `asset path may not contain '..': <path>`
  (`a..b` is an ordinary name);
- the canonical result — symlinks resolved, through its deepest existing
  ancestor for a file that doesn't exist yet — must lie inside the canonical
  asset root: `asset path escapes asset_root: <path>`.

A rejected path is a `400 invalid_path` (`invalid_manifest_path` on the compose
routes), never a directory listing or a read outside the root. The check covers
the request's parameters; the paths *inside* a manifest are resolved as written,
relative to the manifest — another reason to keep the server on a trusted
network.

## Manifest keys are kept

The server never drops a manifest key it doesn't know: `scenes/save` writes
the YAML verbatim and `scenes/load` returns it byte for byte; `validate` accepts
unknown keys (the parser has no strict mode, so a misspelled optional key is
ignored, not reported); the URDF bundle re-serialises the manifest with every
key and rewrites only its three asset paths. `tests/server.rs` and
`tests/stdio_integration.rs` check this with a manifest full of keys nothing
knows.

## stdio transport

`robo-fitting-room stdio-serve --asset-root <dir>` reads requests from stdin and
writes responses to stdout, for embedding (an editor extension, a test harness)
without a network port. It offers every REST route as a method, with the same
sandbox, library calls and error codes.

**Framing.** Newline-delimited JSON: one request per line in, one response per
line out, in request order, flushed after each. Blank lines are skipped without
a response. End of input ends the process with exit code `0`; logs go to stderr.

**Request.** `{"id": <unsigned integer>, "method": "<name>", "params": <value>}`.
`params` defaults to `null`; other keys (a `"jsonrpc"`, say) are ignored.

**Response.** `{"id": N, "result": …}` or
`{"id": N, "error": {"code": …, "message": "…", "data": {"code": "…", "hint": "…"}}}`
(`hint` only where there is one). There is no `jsonrpc` field.

| `error.code` | `error.data.code` | When |
|---|---|---|
| `-32700` | `parse_error` | the line isn't a request: not JSON, or without an unsigned integer `id` or a `method`. Its `id` is `null` |
| `-32601` | `method_not_found` | an unknown method |
| `-32602` | `invalid_params` | the params don't fit the method (`Invalid params: …`) |
| `400`, `404`, `500` | the REST error code (`invalid_path`, `manifest_invalid`, …) | the application errors of the matching REST route, with its message |

| Method | Params | Result | REST counterpart |
|---|---|---|---|
| `health.check` | — | `{status, version}` | `GET /healthz` |
| `health.ready` | — | `{ready, asset_root, urdf_count, errors?}`; not ready is a result, not an error | `GET /readyz` |
| `assets.listRobots` | — | `[{id, urdf_rel}]` | `GET /api/v1/assets/robots` |
| `assets.listFixtures` | — | `[{id, urdf_rel}]` | `GET /api/v1/assets/fixtures` |
| `assets.listFloorPlans` | — | `[{name, ext, rel}]` | `GET /api/v1/assets/floor-plans` |
| `assets.uploadFloorPlanObj` | `{filename, dataBase64}` | `{stored_path, bytes}` | `POST …/upload-obj`; the OBJ base64-encoded (RFC 4648, whitespace and padding ignored); a bad character: 400 `invalid_base64` |
| `urdf.joints` | `{path}` | `{joints: […]}` | `POST /api/v1/urdf/joints` |
| `floorPlan.segments` | `{path, footprintLayers?}` (or `footprint_layers`) | `{segments, footprint_segments}` | `POST /api/v1/floor-plan` |
| `compose.validate` | `{yaml}` | `{ok, errors, warnings?}`, always a result | `POST /api/v1/validate` |
| `compose.run` | `{manifestPath, fullRobots?, fullTools?}` (defaults `false`, `true`) | `{zipPath, downloadName, bytes}`, `downloadName` = `<stem>.zip` | `POST /api/v1/compose` |
| `compose.urdfBundle` | `{manifestPath}` | `{zipPath, downloadName, bytes}`, `downloadName` = `<stem>-urdf.zip` | `POST /api/v1/compose-urdf-bundle` |
| `scene.load` | `{path}` | `{yaml}` | `POST /api/v1/scenes/load` |
| `scene.save` | `{path, yaml}` | `{ok, bytes}` | `POST /api/v1/scenes/save` |

The compose methods don't return the zip inline: they write it to the OS temp
folder as `rfr-compose-<pid>-<stem>-<n>.zip` and return its path — the caller
owns the file and moves or deletes it. Their validation failure is
`manifest_invalid` with the hint `call compose.validate to inspect each error`;
a zip that can't be stat'ed afterwards is a 500 `zip_stat_failed`. The compose
methods' params and results and the upload's `dataBase64` are camelCase
(`floorPlan.segments` takes `footprintLayers` too); `compose.validate`'s
`{yaml}` and `scene.load`'s `{yaml}` wrap what REST sends or returns as a raw
body; every other shape is the REST one, snake_case.

```
→ {"id":1,"method":"health.check"}
← {"id":1,"result":{"status":"ok","version":"<version>"}}
→ {"id":2,"method":"urdf.joints","params":{"path":"../etc/passwd"}}
← {"id":2,"error":{"code":400,"message":"Validation failed: asset path may not contain '..': ../etc/passwd","data":{"code":"invalid_path"}}}
→ not json
← {"id":null,"error":{"code":-32700,"message":"Parse error: request line is not valid JSON","data":{"code":"parse_error"}}}
```

## MCP transport

When an MCP client spawns `serve`, the process also speaks MCP on
stdin/stdout. Every operation of the spec is a tool, answered in process by the
router REST serves, with the same handlers, error codes and
[sandbox](#path-sandboxing), while REST keeps serving on its port; the two share
one state. MCP opens no socket of its own. The protocol side is rmcp 3.6, which
takes both the 2026-07-28 revision (`server/discover`, metadata on every
request) and the `initialize` handshake of the earlier ones. `src/mcp.rs` wires
it up, the workspace crate `openapi-mcp` turns the spec into tools; it is the
Cargo feature `mcp`, on by default.

**When.** `serve` decides once at start, after the REST bind, which stays fatal
(exit code `1`, no MCP either):

- stdin is a terminal: no MCP, and `serve` runs as it always did. The banner
  says `MCP            off (stdin is a terminal)`.
- Otherwise — an MCP client, a pipe, `/dev/null` — MCP runs on stdio, and the
  banner says `MCP            stdio, artifacts in <dir>`. If the tools can't be
  set up (the artifact folder can't be created, say), it says `off (<error>)`
  and REST runs alone.
- Once the client has sent a request, the end of stdin ends the session: a call
  still running gets 5 seconds to answer, REST shuts down gracefully (requests
  in flight finish) and `serve` exits `0`.
- If stdin ends before any request — `/dev/null` under a container runtime,
  `nohup` or a service manager, or input that is no MCP message — MCP stops
  quietly and REST runs on until the process is killed.
- If the MCP side fails (a client whose first message is no request, say), REST
  shuts down too and `serve` exits `1` with `ERROR: IO error: MCP: …`.
- stdout carries the MCP messages only. The banner and the logs go to stderr;
  at the default level, `info`, that includes rmcp's lines on the session.

**Tools.** A tool's name is the operationId, its title the summary, and its
description the summary, the description and a closing `HTTP: <method> <path>`
line, cut to 2,000 characters. The GET tools and the read-only POSTs
`validate_scene`, `load_scene`, `list_urdf_joints` and
`get_floor_plan_segments` carry `readOnlyHint` and `idempotentHint`, and every
tool carries `openWorldHint: false`. The server instructions describe the asset
tree and the usual flow (list, `save_scene`, `validate_scene`,
`compose_scene`) and name the artifact folder.

| Tool | Route | Arguments | Result |
|---|---|---|---|
| `list_robots` | `GET /api/v1/assets/robots` | — | `{items: [{id, urdf_rel}]}` |
| `list_fixtures` | `GET /api/v1/assets/fixtures` | — | `{items: [{id, urdf_rel}]}` |
| `list_floor_plans` | `GET /api/v1/assets/floor-plans` | — | `{items: [{name, ext, rel}]}` |
| `upload_floor_plan_obj` | `POST /api/v1/assets/floor-plans/upload-obj` | `filename`, `file` (base64) | `{stored_path, bytes}` |
| `list_urdf_joints` | `POST /api/v1/urdf/joints` | `path` | `{joints: […]}` |
| `get_floor_plan_segments` | `POST /api/v1/floor-plan` | `path`, `footprint_layers?` | `{segments, footprint_segments}` |
| `validate_scene` | `POST /api/v1/validate` | `yaml`, the manifest text | `{ok, errors, warnings?}` |
| `compose_scene` | `POST /api/v1/compose` | `manifest_path`, `full_robots?`, `full_tools?` | the zip, saved |
| `compose_urdf_bundle` | `POST /api/v1/compose-urdf-bundle` | `manifest_path` | the zip, saved |
| `load_scene` | `POST /api/v1/scenes/load` | `path` | the YAML, as text |
| `save_scene` | `POST /api/v1/scenes/save` | `path`, `yaml` | `{ok, bytes}` |
| `healthz` | `GET /healthz` | — | `{status, version}` |
| `readyz` | `GET /readyz` | — | `{ready, asset_root, urdf_count}`; not ready is an error result |

**Arguments.** Every `inputSchema` is an object with
`additionalProperties: false`, taken from the spec: the query parameters and
the JSON body's fields are the arguments, with the spec's descriptions and
defaults (`full_robots` `false`, `full_tools` `true`, `footprint_layers` `[]`);
`validate_scene`'s raw YAML body is the string `yaml`; the upload's multipart
fields are `filename` and `file`, the OBJ in base64 (standard or URL-safe
alphabet, padding optional, whitespace ignored, a `data:…;base64,` prefix
allowed), and axum's 2 MB body limit applies to the decoded file. An argument
left out is left to the server's default. An unknown or missing argument is an
error result with `code: "invalid_arguments"`, which lists the tool's
arguments.

**Results.**

- JSON: an object is the result's `structuredContent` and, as JSON text, its
  text; an array is wrapped as `{items: […]}`.
- Text: `load_scene`'s YAML is the text, without `structuredContent`.
- A zip (`compose_scene`, `compose_urdf_bundle`) is never inlined. It is saved
  in the artifact folder as `<stem>-<random>.zip`, after the response's file
  name (`<manifest-stem>.zip`, `<manifest-stem>-urdf.zip`), and the result is
  `{path, bytes, content_type, filename}` plus a `file://` resource link.
- The artifact folder is new for every `serve` process:
  `robo-fitting-room-mcp-<random>` in the system temp folder (`$TMPDIR`), mode
  0700, its files 0600 (on Unix). The banner and the server instructions name
  it. It is kept when the process exits and nothing deletes its files; there is
  no flag to move it.

**Errors.**

- An HTTP status of 400 or more is an error result (`isError: true`) with
  `{http_status, body}`; `body` is the [error envelope](#error-envelope),
  axum's plain-text rejection, or `/readyz`'s readiness body.
- A failure of the facade itself is an error result with `{code, message}`:
  `invalid_arguments`, or `internal_error` for a response that can't be read or
  saved. A handler that panics gives `internal_error` too in a debug build; the
  release profile aborts on a panic (`panic = "abort"`), which ends the process.
- An unknown tool name is a JSON-RPC error, `-32602`.

Calls run one at a time.

**Registration.** Each session starts its own `serve`, which binds its port
before anything else, so give the registration a port of its own: one that a
`serve` in a terminal (3040 by default) or another session doesn't hold. While
the session runs, the REST API and the editor are on that port. Claude Code:

```sh
claude mcp add robo-fitting-room -- \
    /path/to/robo-fitting-room serve --asset-root /path/to/asset-root --port 3041
```

`--scope project` writes the entry to the project's `.mcp.json` instead. Kiro
reads `.kiro/settings/mcp.json` in the workspace (or `~/.kiro/settings/mcp.json`
for every workspace):

```json
{
  "mcpServers": {
    "robo-fitting-room": {
      "command": "/path/to/robo-fitting-room",
      "args": ["serve", "--asset-root", "/path/to/asset-root", "--port", "3041"],
      "disabled": false
    }
  }
}
```

Use absolute paths: the client chooses the working folder.

## Editor app

The SPA at `/` is a plain ES-modules application — no framework, no build
step. Source under [`web/`](../web/), embedded into the binary at build time
(`serve --web-root` serves a folder instead, for development only: a URL path
with `..` segments reads files outside it,
[`cli.md`](cli.md#serve-rest-api-and-2d-editor)). It loads js-yaml 4.1.1 from
`/vendor/js-yaml/js-yaml.min.js`, vendored under `web/vendor/` (see its
`VENDOR.md`), so the browser needs no internet access, and it calls the
deprecated unversioned `/api/*` aliases of the routes above.

- **Palette** — the robots, fixtures and floor plans of the asset tree. Click a
  robot or fixture to add it at the origin (its id made unique with `_2`, `_3`,
  …); click a floor plan to attach it.
- **Canvas** — top-down, in metres: left-drag moves the selected item, right-drag
  pans, the wheel zooms. Robots are discs of 0.4 m radius with a heading line
  (`orientation.a`), fixtures 1 m squares; the floor plan's footprint layers are
  drawn in orange. Robots and fixtures are drawn at their manifest coordinates,
  the floor plan at its offset (or `−cell_origin_dxf`) and scale; `scene_origin`
  isn't applied in the editor (compose subtracts it from both alike).
- **Inspector** — a robot's id, URDF, position, orientation and one home-pose
  field per actuated joint, in ° for turning joints and m for sliding ones;
  *Reset to default* and *Use default (drop override)* both put the default
  pose in the form; *Remove robot*. A fixture's id, URDF, position and
  orientation. Numbers are exact inputs, no sliders.
- **Load Scene YAML…** opens a local file in the browser; its server path
  becomes `fitted-robots/<file name>`. **Save…** writes the scene under
  `fitted-robots/` (`scenes/save`).
- **Generate USDA** opens a dialog — full robot visuals (off), full tool visuals
  (on), the URDF source bundle (off), the download name
  (`scene-<yyyymmddhhmmss>.zip`, local time) — then validates the scene
  (errors stop it, warnings go to the browser console), saves it to its current
  path (set by the last load or **Save…**, else `fitted-robots/scene.yaml`),
  overwriting the server's copy without asking, composes it and downloads the
  zip, and the bundle as `<name>-urdf.zip` if ticked.

On save the editor writes the keys it knows first — among them the scene's
`name`, `description` and `scene_origin`, which it has no fields for — and then
every other key as it read it: a robot's `pedestal`, `payload` and
`link_density_kg_m3`, the floor plan's `footprint_layers`, any custom key.
`footprint_layers` stays when the same plan is picked again and is dropped, with
a note in the status line, when another plan is picked. A robot without a
recorded home pose is saved with the default pose sized to its URDF's actuated
joints (or without `home_pose_deg` if the joints can't be fetched); a recorded
pose of the wrong length is kept as it is until a joint is edited or reset. The
YAML is re-written, so comments are lost and the editor's defaults (an empty
`description`, `scale: 1`) appear. The editor has no fields for the scene's
name and origin or the floor plan's offset, scale and `cell_origin_dxf`, and no
upload control (the API has one).

## Running on Cyclone DDS

The composer's emitted topic schema passes both `rmw_validate_namespace` and
`rmw_validate_topic_name`, so Isaac Sim can run with either FastDDS (default) or
Cyclone DDS:

```sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# launch Isaac Sim — every /<cell>/<robot>/joint_states topic resolves
# without bridge-side validation errors
```

The contract is checked by `ros2_topics_are_cyclone_dds_compatible` in
`tests/requirements.rs` (`cargo test -p robo-fitting-room --test requirements
ros2_topics_are_cyclone_dds_compatible`), against a stress manifest with
digit-leading names.
