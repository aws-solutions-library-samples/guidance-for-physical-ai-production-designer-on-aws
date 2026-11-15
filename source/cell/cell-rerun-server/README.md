# cell-rerun-server

A single, self-contained **supervisor binary** that embeds the `cell-runner`
backend (and its shared libraries) and drives it over an **NDJSON stdin/stdout
control channel** (mirroring the robo-fitting-room `stdio-serve` model).
factoryos-backend's `services/cell.rs` is its user: it runs `stdio-serve` +
`start` for the simulated cell-runner (on `cellRunnerWsPort`, or a free port
when that is taken), whose WebSocket feeds `factoryos/sim/state`.

The name is historical: it no longer embeds or runs anything Rerun. The 3D view
is renderer-neutral (`factoryos/scene/<source>`, geometry over
`GET /api/v1/assets/{sha256}`; Rerun lives only in the viewer's `scene3d/rerun`).
The Kiro extension's `rrd-serve` subcommand, the `rerun-viewer` it ran and the
extension's `cell.request` proxy were removed on 2026-10-25, when the extension
was retired.

## Why a supervisor

`cell-runner --server` serves the cell controller WebSocket (`/ws`) and the
scene/mesh static files (`/env/`) on one port (default `9902`), on loopback
(`--host`, default `127.0.0.1`). That WebSocket must stay on a local port, so
stdin/stdout is the **control channel**: the caller spawns this one binary and
sends `start` / `stop` / `status`, and the binary extracts and launches the
cell-runner, reporting its URLs back. The cell-runner inherits this binary's
environment: `CELL_RUNNER_HOST=0.0.0.0` for the caller makes it reachable from
other hosts too (any address set there must include loopback, which this binary
and factoryos-backend connect to).

## Embedding

The cell-runner binary and its shared libraries are packed into a single zip
that is `include_bytes!`-embedded at build time. Point the build at that archive
with the `CELL_RERUN_BACKEND_ARCHIVE` env var:

```bash
# archive layout:  bin/cell-runner  lib/*.so
CELL_RERUN_BACKEND_ARCHIVE=/path/to/backend.zip cargo build --release
```

`cell/scripts/build-cell-rerun-server.sh` does all of it (build the
cell-runner, collect and check its libraries on Linux, embed them) and leaves
the result in `cell/target/release/cell-rerun-server`, where factoryos-backend
looks in a checkout (`crates/factoryos-backend/src/paths.rs`).

A plain `cargo build` (no env var) still compiles — it embeds an empty archive
and the binary reports `backendsEmbedded: false`; `start` then returns a
descriptive error.

At runtime the embedded archive is extracted once into a per-version temp dir,
the loader search path (`DYLD_LIBRARY_PATH` / `LD_LIBRARY_PATH`) is pointed at
the extracted `lib/`, and the cell-runner is spawned.

## Control protocol (NDJSON — one JSON object per line)

```
in :  {"id": <u64>, "method": "<name>", "params": <any>}
out:  {"id": <u64>, "result": <any>}   |   {"id": <u64|null>, "error": {code, message}}
```

Methods:

| method         | params                                   | result |
|----------------|------------------------------------------|--------|
| `health.check` | —                                        | `{status, backendsEmbedded, running}` |
| `start`        | `{cellDir, envDir, scene?, cellPort?}`   | `{ready, cellPort, cellWsUrl, cellHttpUrl}` (idempotent) |
| `status`       | —                                        | status snapshot |
| `stop`         | —                                        | `{stopped: true}` |

Reading until stdin EOF stops the cell-runner and exits `0`.

## CLI (manual testing)

```bash
cell-rerun-server stdio-serve                 # the embedding mode
cell-rerun-server run --cell <dir> --env <dir> [--scene s.yaml] \
    [--cell-port 9902]                        # foreground cell-runner; Ctrl-D to stop
cell-rerun-server --version
```

`9902` is the default of a manual run. Factory OS's backend passes its own
(`cellRunnerWsPort`, or a free one when that is taken).
