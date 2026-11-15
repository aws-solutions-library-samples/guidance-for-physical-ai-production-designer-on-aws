# REST API

The `serve` subcommand boots an axum-based HTTP server hosting a
versioned REST API over the dressing pipeline, the 3D viewer at `/`
and Swagger UI at `/docs`. Its handlers call the library functions the
CLI calls (`catalogue::scan`, `parse_urdf`, `validate`,
`generate_hulls`, `merge`, `serialize_urdf`), but the REST side is
catalogue-driven: `dress` takes catalogue ids and always writes to
`<catalogue>/urdf-dressed-robots/<robot_id>__<tool_id>/`, `validate`
takes a catalogue-relative path, `batch` returns a zip instead of
writing to an output folder, and the listing has its own JSON shape,
not `list --json`'s. `dress --view` has no REST counterpart; the
dressed listing, the preload routes, `/assets` and the viewer have no
CLI counterpart. When an MCP client spawns `serve`, it also offers the
spec's operations as MCP tools on stdin/stdout, answered by the same
handlers ([MCP transport](#mcp-transport)).

## Schema-of-record

The authoritative wire schema is the **OpenAPI 3.x spec** at
[`openapi.json`](../openapi.json). Pull it from the running server too:

```sh
curl -s http://localhost:3030/openapi.json | jq '.info.version'   # → "0.1.0", the API's version
curl -s http://localhost:3030/healthz      | jq '.version'        # → "<version>", the crate's
```

`tests/openapi_snapshot.rs` compares the spec generated from the
handlers' `#[utoipa::path]` annotations with the checked-in file on
every `cargo test`, so any change to a route, parameter or schema fails
until the snapshot is refreshed deliberately:

```sh
URD_UPDATE_SNAPSHOTS=1 cargo test -p urdf-robo-dresser --test openapi_snapshot
```

The spec lists the 11 operations of the `catalogue`, `dress`,
`validate`, `batch` and `system` tags ([Endpoints](#endpoints)). It
leaves out the preload routes, `/assets/{*path}`, the viewer's files,
`/openapi.json` and `/docs`. Its `info.license` says
`MIT OR Apache-2.0`, where the crate is MIT-0.

The Swagger UI at `/docs` (loaded from the running server) lets you
click "Try it out" against every operation in the spec without writing
curl.

## Versioning policy

- `/api/v1/...` is the stable contract: its paths and DTO shapes change
  only additively. Additive routes such as `/api/v1/catalogue/dressed`
  and `/api/v1/preload` live under `/api/v1/` itself; a breaking change
  would get `/api/v2/`. There are no other versions or aliases.
- Any change to the generated spec, additive or breaking, fails
  `tests/openapi_snapshot.rs` until `openapi.json` is refreshed. The
  test doesn't check `info.version`, which is set by hand in
  `src/server/openapi.rs`.

## Authentication, CORS, bind address

There is no authentication. CORS answers loopback origins only
(`http[s]://localhost`, `127.0.0.0/8`, `[::1]`, any port): a page served
from this machine may call the API from the browser, another site's page
open in the same browser gets no CORS headers, so it can neither read a
response nor send a JSON `POST`. `serve` binds `127.0.0.1` only (there is
no `--host` flag), so only clients on the same host, or in the same pod
network namespace, can reach it. Run
it for local development, through a tunnel or port forward
(`ssh -L 3030:127.0.0.1:3030 <host>`), or behind a reverse proxy beside
it that handles auth. An MCP client that spawns `serve` brings this API
up too, for as long as its session runs ([MCP transport](#mcp-transport)).

## Health & readiness

| Path | Method | Returns |
|---|---|---|
| `/healthz` | GET | `200` as soon as the server accepts connections: `{"status":"ok","version":"<version>"}` (the crate version) |
| `/readyz` | GET | `200` with `{"ready":true,"catalogue_root":…,"robot_count":N,"tool_count":M}` when the catalogue holds at least one robot or one tool (`<collection>/<name>/<name>.urdf`); otherwise `503` with `{"ready":false,…,"errors":["catalogue is empty (no robots or tools)"]}`, a readiness body, not the error envelope |

```sh
curl -s http://localhost:3030/healthz | jq .
curl -s http://localhost:3030/readyz  | jq .
```

`--catalogue` is resolved once at startup: a path that doesn't exist
stops `serve`. Every `/readyz` call re-scans the catalogue, parsing and
validating each robot and tool URDF, so a large catalogue makes the
probe slow.

These probes sit at the **root**, NOT under `/api/v1` — the
Kubernetes convention every probe configurator expects out of the
box. Because `serve` listens on `127.0.0.1` only, a kubelet `httpGet`
probe, which connects to the pod IP, can't reach them: probe through a
sidecar proxy, or with an `exec` probe such as
`curl -fsS http://127.0.0.1:3030/readyz`.

## Error envelope

Every error a handler returns carries a typed JSON body:

```json
{
  "code": "robot_not_in_catalogue",
  "message": "robot_id `kr240r2901` resolves to <catalogue>/urdf-robot-collection/kr240r2901/kr240r2901.urdf which does not exist"
}
```

- `code` is a stable machine-readable token (no spaces). Matchable in
  client code.
- `message` is human-readable.
- `hint` is optional — only populated when there's an actionable
  remediation pointer: on `invalid_id` (`list_catalogue_robots (GET
  /api/v1/catalogue/robots) and list_catalogue_tools (GET
  /api/v1/catalogue/tools) list the valid ids`) and on
  `validation_failed` (`<n> blocking diagnostics — call /api/v1/validate
  to inspect`, where `<n>` counts every diagnostic of both inputs,
  warnings included).

The shape is `components.schemas.ErrorEnvelope` in the spec. The codes:

| Status | Codes |
|---|---|
| 400 | `invalid_id`, `invalid_orient`, `invalid_hull_token`, `robot_urdf_parse_failed`, `tool_urdf_parse_failed` (dress); `invalid_path`, `urdf_parse_failed` (validate); `invalid_hull_token` (batch); `invalid_job_id` (jobs); `invalid_asset_path` (`/assets`); `invalid_collection`, `invalid_entry` (preload) |
| 404 | `robot_not_in_catalogue`, `tool_not_in_catalogue` (dress); `unknown_job`, `job_not_completed` (jobs); `asset_not_found` (`/assets`); `entry_not_found`, `entry_empty` (preload) |
| 422 | `validation_failed` (dress) |
| 500 | `output_mkdir_failed`, `merge_failed`, `write_failed` (dress); `artifact_read_failed`, `response_build_failed` (jobs); `preload_read_failed`, `preload_changed_during_read`, `preload_scan_failed` (preload); `catalogue_scan_failed` (listings; declared, never raised) |
| 503 | `catalogue_empty` (batch) |

Not every non-2xx response is an envelope. Requests axum rejects before
a handler runs get its plain-text bodies: `400` for a body that isn't
JSON, `415` without `Content-Type: application/json`, and `422` for JSON
that doesn't fit the request (a missing `robot_id`, an `orient` that
isn't three non-negative integers). That `422` shares its status with
`validation_failed`: tell them apart by the content type. A wrong
method on a known route gets `405` with an empty body, and `/readyz`'s
`503` carries the readiness body. An unknown path, under `/api/v1/`
too, falls through to the viewer and returns its `index.html` with
`200`.

## Endpoints

| Path | Method | Tag | operationId | Purpose |
|---|---|---|---|---|
| `/api/v1/catalogue` | GET | catalogue | `list_catalogue` | Combined listing of all robots + tools ([below](#catalogue-listing)) |
| `/api/v1/catalogue/robots` | GET | catalogue | `list_catalogue_robots` | Robots only |
| `/api/v1/catalogue/tools` | GET | catalogue | `list_catalogue_tools` | Tools only |
| `/api/v1/catalogue/dressed` | GET | catalogue | `list_dressed_robots` | Every `urdf-dressed-robots/<name>/` holding `<name>.urdf`, sorted by name → `[{name, urdf_rel}]` (`[]` without the folder) |
| `/api/v1/dress` | POST | dress | `dress_robot` | Merge one robot + one tool by catalogue id into `urdf-dressed-robots/<robot_id>__<tool_id>/` → `{ok, dressed_urdf_rel, diagnostics}` |
| `/api/v1/validate` | POST | validate | `validate_urdf` | Validate the URDF at a catalogue-relative `path` → `{ok, diagnostics}` |
| `/api/v1/batch` | POST | batch | `start_batch` | Long-running permutation merge → `202 Accepted` + job id |
| `/api/v1/jobs/{id}` | GET | batch | `get_job_status` | Poll batch job status |
| `/api/v1/jobs/{id}/artifact` | GET | batch | `get_job_artifact` | Download finished batch as `application/zip` |
| `/healthz` | GET | system | `healthz` | Liveness probe |
| `/readyz` | GET | system | `readyz` | Readiness probe |
| `/api/v1/preload` | GET | — | — | The viewer's cache index → `[{collection, name, etag, files, bytes}]`, one row per `<collection>/<name>/<name>.urdf` in `urdf-robot-collection/`, `urdf-tool-collection/` and `urdf-dressed-robots/`, sorted |
| `/api/v1/preload/{collection}/{name}` | GET | — | — | One entry's `.urdf`, `.obj`, `.mtl`, `.stl` and `.dae` files in one `application/octet-stream` body, `[u32 LE manifest length][manifest JSON [{rel, off, len}]][file bytes]`, with an `ETag`, `Cache-Control: public, max-age=0, must-revalidate` and `304` on a matching `If-None-Match` |
| `/assets/{*path}` | GET | — | — | Any file under the catalogue root as raw bytes (the viewer fetches URDFs and meshes through it), path-sandboxed via `resolve_asset`: `Content-Type` from the extension, an `ETag`, `Cache-Control: public, max-age=31536000, immutable`, and `304` on a matching `If-None-Match` |
| `/openapi.json`, `/docs` | GET | — | — | The spec; Swagger UI |
| `/` and any unmatched path | any | — | — | Embedded 3D viewer SPA: its files with `Cache-Control: no-cache` and an `ETag`, anything else `index.html` |

The spec covers the operations with an operationId; the preload routes,
`/assets/{*path}`, the viewer and the docs routes carry no
`#[utoipa::path]` decorator and are absent from it.

Full request/response schemas are in [`openapi.json`](../openapi.json) and
the Swagger UI at `/docs`.

## Web UI

`serve` also hosts a built-in single-page 3D viewer at `/` (WebGL;
three.js r0.169.0 and urdf-loader 0.12.6, vendored under `web/vendor/`,
no CDN):

- **Catalogue** — robots and tools (`list_catalogue`), each with a
  validity dot from its `status`, and the **Dressed** list
  (`list_dressed_robots`). Clicking a robot or a tool previews its
  source URDF; clicking a dressed robot loads it without re-dressing.
- **Inspector** — the selected pair, the hull buffer, X/Y/Z −90/+90
  orientation steps and **Dress** (`dress_robot`). With a pair
  selected, every orientation step dresses again and rewrites
  `urdf-dressed-robots/<robot>__<tool>/`. Below: the output path (with
  a *new URDF written* badge), the diagnostics, and one slider per
  non-fixed joint.
- **Viewport** — drag empty space to orbit, scroll to zoom, right-drag
  to pan, drag a joint to articulate it (hovering shows its name).
  **Collision hulls** overlays the `<collision>` meshes; **Reach
  envelope** overlays a loaded dressed robot's
  `meshes/3dreach/reach_envelope.obj`.
- **Preload all** — fills the browser cache (Cache Storage, kept across
  sessions) with every robot, tool and dressed robot, one
  `GET /api/v1/preload/{collection}/{name}` each, after
  `GET /api/v1/preload` shows which entries are stale. An entry's own
  cache dot fetches just that entry.

URDF and mesh bytes come over `/assets/{*path}`; the viewer doesn't call
validate or batch. Embedded in an editor webview, it takes the
editor's theme colours.

Pass `--web-root <dir>` to `serve` to load the SPA from disk instead of
the embedded bundle — handy for iterating on the viewer's JS without
rebuilding, on a machine you don't share ([`cli.md`](cli.md)).

## Catalogue listing

```sh
curl -s http://localhost:3030/api/v1/catalogue | jq .
```

```json
{
  "robots": [
    {"name": "kr240r2900", "flange_link": "like_kr240r2900_link_6", "joint_count": 7, "link_count": 8, "status": {"kind": "valid"}}
  ],
  "tools": [
    {"name": "sticktool", "visual_count": 5, "status": {"kind": "valid"}}
  ]
}
```

Entries are the `<name>/<name>.urdf` folders of
`urdf-robot-collection/` and `urdf-tool-collection/`, sorted by name;
`name` is the id `POST /api/v1/dress` takes, and `flange_link` the
detected flange. `status` is `{"kind":"valid"}`,
`{"kind":"incomplete","issues":[…]}` (the validation errors' messages,
or the warnings' when there are no errors) or
`{"kind":"error","message":…}` (the URDF doesn't parse).
`GET /api/v1/catalogue/robots` and `/tools` return the two arrays
alone. Every call re-scans the disk.

## Dress (one-shot)

```sh
curl -s -X POST http://localhost:3030/api/v1/dress \
    -H "Content-Type: application/json" \
    -d '{
        "robot_id": "kr240r2900",
        "tool_id":  "sticktool",
        "hull": "medium"
    }' | jq .
```

Response shape:

```json
{
  "ok": true,
  "dressed_urdf_rel": "urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf",
  "diagnostics": []
}
```

`diagnostics` lists the non-blocking warnings of both inputs, each
`{severity, code, message, location?}` as in [Validate](#validate). The
URDF itself is on disk, not in the response:
`GET /assets/urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf`
serves it. The orientation auto-orient picked isn't returned either;
the server logs it at `info` level.

Fields:

| Field | Type | Notes |
|---|---|---|
| `robot_id` | string | Required. A `robots[].name` from the listing, naming `<catalogue>/urdf-robot-collection/<robot_id>/<robot_id>.urdf`. It must be one entry name: an empty id, `.`, `..`, an absolute path, an id with `/`, `\`, `:` or a NUL byte, or one longer than 255 bytes is `400 invalid_id`; a missing file is `404 robot_not_in_catalogue` |
| `tool_id` | string | Required. A `tools[].name`, naming `<catalogue>/urdf-tool-collection/<tool_id>/<tool_id>.urdf`; the same rules (`404 tool_not_in_catalogue`) |
| `orient` | `[u16, u16, u16]` | Optional, default `[0, 0, 0]`. Degrees about X, Y and Z, each `0`, `90`, `180` or `270` (else `400 invalid_orient`). `[0, 0, 0]`, omitted or sent, runs auto-orient |
| `attach_to` | string | Optional. Override flange-link detection (`flange_link` in the listing); a link the robot doesn't have is `500 merge_failed` |
| `hull` | string | `tight` / `medium` (default) / `max`, lower case (else `400 invalid_hull_token`) |

A dress runs in this order and stops at the first failure:

1. Both ids are checked (`400 invalid_id`) before any path is built,
   then looked up (`404`).
2. `orient` and `hull` are checked (`400`).
3. Both URDFs are parsed (`400 robot_urdf_parse_failed` /
   `tool_urdf_parse_failed`) and validated, the tool in tool mode. Any
   error is `422 validation_failed`, whose body only counts the
   diagnostics: run `POST /api/v1/validate` on each input to see them.
4. Only then is `<catalogue>/urdf-dressed-robots/<robot_id>__<tool_id>/`
   created (`500 output_mkdir_failed`), so a request refused so far
   leaves nothing behind.
5. The collision hulls are generated (a failure is ignored), the merge
   runs (`500 merge_failed`) and the URDF is written as
   `<robot_id>__<tool_id>.urdf` (`500 write_failed`). A merge failure,
   such as an unknown `attach_to`, leaves the folder with the hulls
   written so far.

The folder then holds what [`dress`'s output](cli.md#output-layout)
holds, including the tool's [TCP frame](concepts.md#tool-centre-point-tcp)
when it declares one; a tool without one only logs a warning, and
nothing reaches `diagnostics`. Dressing the same pair again overwrites
the URDF and the hulls but keeps the mesh files already copied there.

## Validate

```sh
curl -s -X POST http://localhost:3030/api/v1/validate \
    -H "Content-Type: application/json" \
    -d '{"path": "urdf-robot-collection/kr240r2900/kr240r2900.urdf", "tool": false}' | jq .
```

```json
{"ok": true, "diagnostics": []}
```

`path` is relative to the catalogue root, so dressed URDFs can be
validated too; `tool` (default `false`) selects tool mode, which adds
no check yet ([validation rules](concepts.md#validation-rules)).
Unknown keys are ignored, so a misspelt `is_tool` validates as a robot.

The response is always `200` unless the request itself fails: `ok` is
`false` when any diagnostic is an error. A diagnostic is
`{severity, code, message, location?}`: `severity` is `error` or
`warning`, `code` one of `E011`, `E012`, `E014`, `W003`, `W013`,
`W016`, and `location` the absolute URDF path and the element, e.g.
`<catalogue>/urdf-robot-collection/kr240r2900/kr240r2900.urdf:joint 'joint_a1'`.
Errors: `400 invalid_path` for an absolute path or a `..` component,
`400 urdf_parse_failed` for a file that is missing, unreadable or not
URDF XML. The CLI's `validate` runs the same checks but prints them as
lines and exits `1` on errors ([`cli.md`](cli.md)).

## Batch (long-running)

```sh
JOB=$(curl -s -X POST http://localhost:3030/api/v1/batch \
    -H "Content-Type: application/json" \
    -d '{"hull": "medium"}' | jq -r .job_id)

# Poll
curl -s http://localhost:3030/api/v1/jobs/$JOB | jq .
# {"job_id":"<uuid>","state":"running","progress":0.0}
# … {"job_id":"<uuid>","state":"completed","progress":1.0,"success":1,"failed":0,"artifact_url":"/api/v1/jobs/<uuid>/artifact"}

# When state == "completed", grab the zip
curl -s -o all-dressed.zip http://localhost:3030/api/v1/jobs/$JOB/artifact
unzip -l all-dressed.zip
```

Job lifecycle (every body has `job_id` and `state`):

| State | Body adds | Meaning |
|---|---|---|
| `queued` | — | Accepted, not yet started |
| `running` | `progress` | Pairs done ÷ (robots × tools), 0.0 to 1.0, updated after each pair |
| `completed` | `progress: 1.0`, `success`, `failed`, `artifact_url` | Every pair was tried, failures included; `failed` counts the pairs that didn't parse or merge (a robot that doesn't parse counts once). Artifact ready |
| `failed` | `message` | The job couldn't run: a collection without `<name>/<name>.urdf` entries (`catalogue empty`), a temp folder that couldn't be created, or a failed zip. No artifact |

`POST /api/v1/batch` returns `202 Accepted`, NOT `200`, with
`{"job_id":"<uuid>","status_url":"/api/v1/jobs/<uuid>"}`; there is no
`Location:` header. The JSON body is required: `{}` takes the
defaults.

A batch dresses every robot × tool pair as `POST /api/v1/dress` would
with only `hull` given (detected flange, auto-orient), but without
validation and outside the catalogue: it works in the OS temp folder,
`urdf-robo-dresser-batch-<job_id>/`, and zips that tree to
`urdf-robo-dresser-batch-<job_id>.zip` beside it. Neither is deleted.
The zip holds one `<robot>__<tool>/` folder per pair, laid out like
`urdf-dressed-robots/` (a failed pair may leave a folder with hulls and
no URDF), so `unzip -o all-dressed.zip -d <catalogue>/urdf-dressed-robots/`
installs the results. The download comes as
`Content-Disposition: attachment; filename="batch-<job_id>.zip"`;
before completion it is `404 job_not_completed`. Errors:
`400 invalid_hull_token`; `503 catalogue_empty` when
`urdf-robot-collection/` or `urdf-tool-collection/` is missing (folders
that exist but hold no entries give `202`, then a `failed` job);
`400 invalid_job_id` for an id that isn't a UUID; `404 unknown_job` for
one the registry doesn't hold.

The `JobRegistry` is in-memory only and keeps every job until the
server stops; restarting it discards all job state. For durable
batches, use the CLI `batch` subcommand, which writes directly to disk.

## Path sandboxing

Request paths are resolved against the canonical `--catalogue` root by
`AppState::resolve_asset` (`src/server/state.rs`): an absolute path is
rejected (`asset path must be relative: <path>`), and so is any `..`
component (`asset path may not contain '..': <path>`; `a..b` is an
ordinary name). Both messages arrive prefixed `IO error: `. The check is
lexical: symlinks aren't resolved, so a symlink inside the catalogue is
followed wherever it points. Per route:

- `POST /api/v1/dress`: `robot_id` and `tool_id` are entry names, not
  paths. Anything but one path segment is `400 invalid_id`
  ([above](#dress-one-shot)), checked before a path is built.
- `POST /api/v1/validate` `path`: `400 invalid_path`.
- `GET /assets/{*path}`: `400 invalid_asset_path` (axum percent-decodes
  first, so `%2e%2e` counts as `..`). A missing file or a folder is
  `404 asset_not_found`, never a listing. Every file under the root is
  served, not only URDFs and meshes. HTTP clients usually normalise
  `/assets/../x` to `/x` before sending, and that request gets the
  viewer's `index.html`.
- `GET /api/v1/preload/{collection}/{name}`: `collection` must be
  `urdf-robot-collection`, `urdf-tool-collection` or
  `urdf-dressed-robots` (`400 invalid_collection`), and `name` one
  segment without `/`, `\` or `..` (`400 invalid_entry`).

Path-traversal probes return the typed error envelope, never a
directory listing.

## MCP transport

When an MCP client spawns `serve`, the process also speaks MCP on
stdin/stdout. Every operation of the spec (the operationId column of
[Endpoints](#endpoints)) is a tool, answered in process by the router
REST serves, with the same handlers, error codes and
[sandbox](#path-sandboxing), while REST keeps serving on its port. The
two share one state: a batch job started over MCP shows in
`GET /api/v1/jobs/{id}`, and one started over REST can be polled over
MCP. MCP opens no socket of its own. The protocol side is rmcp 3.6,
which takes both the 2026-07-28 revision (`server/discover`, metadata
on every request) and the `initialize` handshake of the earlier ones.
`src/mcp.rs` wires it up, the workspace crate `openapi-mcp` turns the
spec into tools; it is the Cargo feature `mcp`, on by default.

**When.** `serve` decides once at start, after the REST bind, which
stays fatal (a port in use exits `1`, without MCP):

- stdin is a terminal: no MCP, and `serve` runs as it always did. The
  banner says `MCP             off (stdin is a terminal)`.
- Otherwise — an MCP client, a pipe, `/dev/null` — MCP runs on stdio,
  and the banner says `MCP             stdio, artifacts in <dir>`. If the
  tools can't be set up (the artifact folder can't be created, say), it
  says `off (<error>)` and REST runs alone.
- Once the client has sent a request, the end of stdin ends the
  session: a call still running gets 5 seconds to answer, REST shuts
  down gracefully (requests in flight finish) and `serve` exits `0`.
- If stdin ends before any request — `/dev/null` under a container
  runtime, `nohup` or a service manager, or input that is no MCP
  message — MCP stops quietly and REST runs on until the process is
  killed.
- If the MCP side fails (a client whose first message is no request,
  say), REST shuts down too and `serve` exits `1` with
  `error: IO error: MCP: …`.
- stdout carries the MCP messages only. The banner and the logs go to
  stderr; `RUST_LOG=info` adds rmcp's lines on the session.

**Tools.** A tool's name is the operationId, its title the summary, and
its description the summary, the description and a closing
`HTTP: <method> <path>` line, cut to 2,000 characters. The GET tools and
`validate_urdf` carry `readOnlyHint` and `idempotentHint`, and every
tool carries `openWorldHint: false`. The server instructions describe
the catalogue layout, say that `robot_id` and `tool_id` are single entry
names, give the batch flow and name the artifact folder.

| Tool | Route | Arguments | Result |
|---|---|---|---|
| `list_catalogue` | `GET /api/v1/catalogue` | — | `{robots, tools}` |
| `list_catalogue_robots` | `GET /api/v1/catalogue/robots` | — | `{items: [{name, flange_link, joint_count, link_count, status}]}` |
| `list_catalogue_tools` | `GET /api/v1/catalogue/tools` | — | `{items: [{name, visual_count, status}]}` |
| `list_dressed_robots` | `GET /api/v1/catalogue/dressed` | — | `{items: [{name, urdf_rel}]}` |
| `dress_robot` | `POST /api/v1/dress` | `robot_id`, `tool_id`, `attach_to?`, `orient?`, `hull?` | `{ok, dressed_urdf_rel, diagnostics}` |
| `validate_urdf` | `POST /api/v1/validate` | `path`, `tool?` | `{ok, diagnostics}` |
| `start_batch` | `POST /api/v1/batch` | `hull?` | `{job_id, status_url}` (HTTP 202) |
| `get_job_status` | `GET /api/v1/jobs/{id}` | `id` | `{job_id, state, …}` |
| `get_job_artifact` | `GET /api/v1/jobs/{id}/artifact` | `id` | the zip, saved |
| `healthz` | `GET /healthz` | — | `{status, version}` |
| `readyz` | `GET /readyz` | — | `{ready, catalogue_root, robot_count, tool_count}`; not ready is an error result |

A batch over MCP: `start_batch`, then `get_job_status` with
`id` = `job_id` until `state` is `completed` (or `failed`), then
`get_job_artifact` with the same `id`. Jobs live in this process only:
when the client restarts `serve`, they are gone (`404 unknown_job`).
`dress_robot` writes into the served catalogue, as over REST.

**Arguments.** Every `inputSchema` is an object with
`additionalProperties: false`, taken from the spec: the path parameter
and the JSON body's fields are the arguments, with the spec's
descriptions and defaults. `dress_robot` takes `robot_id`, `tool_id`,
`attach_to` (a string, or `null` for the detected flange), `orient`
(three integers, default `[0, 0, 0]`) and `hull` (default `medium`);
`validate_urdf` takes `path` and `tool` (default `false`);
`start_batch` takes `hull`, and sends `{}` without it. The job tools'
`id` is sent as one percent-encoded path segment, so `a/b` reaches the
handler as `a/b` (`400 invalid_job_id`); an empty `id`, `.` or `..` is
refused before any request. An argument left out is left to the
server's default. An unknown or missing argument is an error result
with `code: "invalid_arguments"`, which lists the tool's arguments; a
refused `id` gets the same code.

**Results.**

- JSON: an object is the result's `structuredContent` and, as JSON
  text, its text; an array (the three single listings) is wrapped as
  `{items: […]}`. `start_batch`'s 202 is a result like any 2xx.
- The zip of `get_job_artifact` is never inlined. It is saved in the
  artifact folder as `batch-<job_id>-<random>.zip`, after the response's
  file name (`batch-<job_id>.zip`), and the result is
  `{path, bytes, content_type, filename}` plus a `file://` resource
  link. The job's own zip stays in the temp folder as well
  ([Batch](#batch-long-running)).
- The artifact folder is new for every `serve` process:
  `urdf-robo-dresser-mcp-<random>` in the system temp folder
  (`$TMPDIR`), mode 0700, its files 0600 (on Unix). The banner and the
  server instructions name it. It is kept when the process exits and
  nothing deletes its files; there is no flag to move it.

**Errors.**

- An HTTP status of 400 or more is an error result (`isError: true`)
  with `{http_status, body}`; `body` is the
  [error envelope](#error-envelope) (`invalid_id`, `unknown_job`, …),
  axum's plain-text rejection, or `/readyz`'s readiness body.
- A failure of the facade itself is an error result with
  `{code, message}`: `invalid_arguments`, or `internal_error` for a
  response that can't be read or saved. A handler that panics gives
  `internal_error` too in a debug build; the release profile aborts on a
  panic (`panic = "abort"`), which ends the process.
- An unknown tool name is a JSON-RPC error, `-32602`.

Calls run one at a time.

**Registration.** Each session starts its own `serve`, which binds its
port before anything else, so give the registration a port of its own:
one that a `serve` in a terminal (3030 by default) or another session
doesn't hold. While the session runs, the REST API and the viewer are on
`http://127.0.0.1:<port>/`. Claude Code:

```sh
claude mcp add urdf-robo-dresser -- \
    /path/to/urdf-robo-dresser serve --catalogue /path/to/catalogue --port 3031
```

`--scope project` writes the entry to the project's `.mcp.json`
instead. Kiro reads `.kiro/settings/mcp.json` in the workspace (or
`~/.kiro/settings/mcp.json` for every workspace):

```json
{
  "mcpServers": {
    "urdf-robo-dresser": {
      "command": "/path/to/urdf-robo-dresser",
      "args": ["serve", "--catalogue", "/path/to/catalogue", "--port", "3031"],
      "disabled": false
    }
  }
}
```

Use absolute paths: the client chooses the working folder.

## Concurrency

The shared state is the canonical catalogue path, the job registry
(`Arc<tokio::sync::RwLock<JobRegistry>>`) and the optional `--web-root`
folder; MCP, when on, calls the same router and so shares it, one call
at a time. There is no cached catalogue: every listing and `/readyz` call
re-scans the disk, parsing and validating each URDF. Handlers do their
CPU-bound work in place: a dress parses, validates, builds hulls and
merges on the async worker thread serving it, and a batch is one
spawned task that dresses the pairs one after another. The runtime is
multi-threaded, so other requests are served meanwhile on the remaining
workers. Nothing serialises writes: two dresses of the same pair at
once write the same `urdf-dressed-robots/<robot>__<tool>/` folder, and
the viewer's orientation buttons dress on every click.

## Cross-crate use

[robo-fitting-room](../../robo-fitting-room/)'s manifests reference
dressed URDFs by relative path. It links this crate as a library
(`default-features = false`, for URDF parsing) and never calls the
dresser over HTTP: the two servers share the folder tree. A typical
workflow, from the workspace root:

```sh
# 1. Boot the dresser server on the catalogue
urdf-robo-dresser serve --catalogue assets/catalogue

# 2. Dress a pair: writes assets/catalogue/urdf-dressed-robots/kr240r2900__sticktool/
curl -s -X POST http://localhost:3030/api/v1/dress \
    -H "Content-Type: application/json" \
    -d '{"robot_id": "kr240r2900", "tool_id": "sticktool"}' | jq -r .dressed_urdf_rel
#    or every pair: POST /api/v1/batch, poll, download the zip, then
#    unzip -o all-dressed.zip -d assets/catalogue/urdf-dressed-robots/

# 3. Boot the fitting room on the same tree (http://localhost:3040);
#    GET /api/v1/assets/robots lists the dressed robot
robo-fitting-room serve --asset-root assets/catalogue

# 4. Author a manifest in robo-fitting-room's editor referencing
#    ../urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf
# 5. Compose a USDA scene → Isaac Sim
```

Step 5 is robo-fitting-room's
[compose recipe](../../robo-fitting-room/docs/api.md#compose-recipe).
