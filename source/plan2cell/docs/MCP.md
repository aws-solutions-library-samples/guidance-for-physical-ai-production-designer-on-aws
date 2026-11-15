# plan2cell MCP server

`plan2cell mcp` exposes the plan2cell pipeline as [Model Context Protocol](https://modelcontextprotocol.io)
tools, so an agent can plan, check, build and (in a `sim` build) co-simulate a robot cell. It
speaks two transports from the same binary:

| Transport | Command | Use it for |
|---|---|---|
| stdio | `plan2cell mcp --stdio` | an MCP client on the same machine that starts the server as a child process (Claude Code, IDE agents) |
| streamable HTTP | `plan2cell mcp --http ADDR` | a long-running service other machines or several clients reach at `http://ADDR/mcp` |

Build it with `cargo build --release -p plan2cell-cli` (add `--features sim` for `plan2cell_simulate`;
that pulls in trust-runtime and takes a while).

## Tools

All tools take the plan either as a file under the root or inline, and the configuration either as
a file under the root, inline TOML, or the server's default (`-c`). The build options default to
the config's `[build]` section; every one can be overridden per call.

| Tool | What it does | Main arguments |
|---|---|---|
| `plan2cell_zones` | Plan only: interlock zones with their ticket order, each robot's ops (zones, permit, expected time), makespan, PLAN_ID. | plan |
| `plan2cell_check` | Build everything in memory and run every check (planning invariants, trust ST type check, strict KRL validator, KRL round trip); write nothing. | plan, config, options |
| `plan2cell_build` | Build; return the summary and the file list, optionally the content of chosen files, and optionally write the build under the root. | plan, config, options, `output_dir`, `overwrite`, `include_files` |
| `plan2cell_explain_robot` | One robot's KRL plan: ops, masks, estimated vs planned vs watchdog times, warnings. | plan, config, options, `robot` |
| `plan2cell_default_config` | The default `plan2cell.toml`, commented. | — |
| `plan2cell_list_files` | Plans, configs and outputs under the root (one directory level). | `dir` |
| `plan2cell_read_file` | Read a file under the root (at most 512 KiB), e.g. a generated Folge or `INSTALL.md`. | `path` |
| `plan2cell_simulate` | (`--features sim` only) Co-simulate the build: generated head-PLC ST against the robots (the executor of the generated KRL, or the device-simulator ST), checked against the plan. Minutes per cycle. | plan, config, options, `cycles`, `robots` |

Common arguments:

| Argument | Meaning |
|---|---|
| `plan_path` / `plan_text` (+ `plan_name`) | the plan file relative to the root, or its text (exactly one) |
| `blackboard_path` | blackboard JSON for a plain BT.CPP XML tree |
| `config_path` / `config_toml` | `plan2cell.toml` relative to the root, or its text (at most one) |
| `profile` | `sim` or `vw-ks24` |
| `plc` | `codesys` or `none` |
| `robot_sim`, `krl`, `st_check`, `strict` | booleans |
| `krl_robots` | list of robot namespaces |

Results are JSON in a text block. A failure of the pipeline (a rejected plan, a missing
`e_axes`, a non-empty output directory, ...) is a tool result with `isError: true` and
`{"error": "...", "exit_code": N}` using the CLI's exit codes, so the agent can read and fix it.

A typical session: `plan2cell_list_files` -> `plan2cell_zones` (understand the plan) ->
`plan2cell_check` (with the intended profile and options) -> `plan2cell_build` with `output_dir` ->
`plan2cell_read_file` on `krl/<ns>/INSTALL.md`.

## Security

The HTTP transport makes code generation reachable over the network, so the server is strict by
default:

- **One root directory.** `--root DIR` (default: the working directory). Every path argument is
  relative to it; absolute paths, `..` and symlinks that lead out of it are rejected.
- **Writing is opt-in.** `plan2cell_build` writes only with `--allow-write`, only under the root, and
  never into a non-empty directory unless the call passes `overwrite: true` (which replaces the
  previous build's files, as `plan2cell build --force` does).
- **Loopback by default.** `--http` on `127.0.0.1` / `::1` works without authentication. Any other
  address is refused unless a bearer token is set (`--token` or `PLAN2CELL_MCP_TOKEN`, at least 16
  characters); every request to `/mcp` must then carry `Authorization: Bearer <token>` (compared in
  constant time). `/healthz` is open.
- **Host header checking.** Only loopback host names are accepted unless you add the names
  clients use with `--allowed-host` (protects a local server against DNS rebinding).
- No TLS in the server itself: put it behind a TLS-terminating reverse proxy for anything beyond
  a trusted network.

## Client setup

**Claude Code (stdio):**

```sh
claude mcp add plan2cell -- /path/to/plan2cell mcp --stdio --root /path/to/plans -c plan2cell.toml --allow-write
```

**Any client with a JSON config (stdio):**

```json
{
  "mcpServers": {
    "plan2cell": {
      "command": "/path/to/plan2cell",
      "args": ["mcp", "--stdio", "--root", "/path/to/plans", "-c", "plan2cell.toml"]
    }
  }
}
```

**Streamable HTTP on another machine:**

```sh
export PLAN2CELL_MCP_TOKEN=$(openssl rand -hex 24)
plan2cell mcp --http 0.0.0.0:8765 --allowed-host cell-pc.local --root /srv/plans -c plan2cell.toml --allow-write
```

```sh
claude mcp add --transport http plan2cell http://cell-pc.local:8765/mcp \
  --header "Authorization: Bearer $PLAN2CELL_MCP_TOKEN"
```

## Implementation

[`crates/plan2cell-mcp`](../crates/plan2cell-mcp) on [rmcp](https://crates.io/crates/rmcp) 3.5:
`src/tools.rs` (the tools; each runs the pipeline on a blocking thread), `src/paths.rs` (the root
sandbox), `src/lib.rs` (`serve_stdio`, `serve_http` with the token middleware). Tests
(`crates/plan2cell-mcp/tests/mcp.rs`) drive it with rmcp's client in process and over HTTP, including
the 401 cases and path escapes; `crates/plan2cell-cli/tests/cli.rs` spawns `plan2cell mcp --stdio`.
