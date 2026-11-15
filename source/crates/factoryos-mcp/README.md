# factoryos-mcp

The Factory OS workflow as a stdio MCP server for the workspace's chat agent (`kiro-cli acp`, agent
`deploy/kiro/factoryos-planner.json`, steering `deploy/kiro/steering/factoryos-*.md`). kiro-cli starts
it per chat session; every tool is a route of the workspace's `factoryos-backend`, called over loopback.

```text
factoryos-mcp [--backend http://127.0.0.1:80] [--timeout-s 900] [--artifact-dir DIR]
factoryos-mcp --list      # name and kind (read / write / destructive) of every tool
factoryos-mcp --openapi   # the OpenAPI document the tools are made from
```

## Tools

Curated in `tools/*.yaml`, one file per workflow step, turned into an OpenAPI document and served by
`openapi-mcp`. The format is in `src/spec.rs`.

| file | tools | backend |
|---|---|---|
| product.yaml | `product_*`: products, add / move / regroup / remove / generate welds, snap onto the surface, check | `services/product.rs`, `product_welds.rs` |
| dresser.yaml | `dresser_*`: catalogue, dress (orient + any-angle spin), TCP, `dresser_orient_sweep` | `dresser.rs`, `orient_sweep.rs` |
| scene.yaml | `scene_*`, `cell_*`: scenes, product at a height on a support, robots, `scene_auto_place_robots`, reach check, cells | `fitter.rs`, `scene_edit.rs`, `reach/` |
| planning.yaml | `cell_plans`, task programs, Prepare Cell (the planners are the planner router's tools) | `cell.rs`, `agentic.rs` |
| bt_view.yaml | `bt_*`: behaviour tree playback with breakpoints and stepping; `view_camera` | `cell.rs` → cell-runner, `view_cmd.rs` → viewer |
| codegen.yaml | `codegen_*`, `hardware_*`, `process_*` (the process definition: IOs) | `codegen.rs`, `hardware.rs`, `process_def.rs` |
| vcell.yaml | `vcell_*`: templates, status, power, deploy check / fix / deploy, production, `vcell_kvc` (one robot controller, an op of the cell's template) | `infra.rs` → `plan2cell cell kvc` |

`read_only` and `destructive` set the MCP hints; the agent file's `allowedTools` lists the read-only
tools (and the simulated playback), so kiro-cli asks the user before every other one.

A tool with `verb:` is a cell-runner verb: the proxy sends `POST /api/v1/cell/command
{"type": verb, "params": <arguments>}` and turns a refused verb (`ok: false`) into a tool error.

## Adding a tool

Add the backend route (its constant in `factoryos-protocol`'s `api`), then an entry in the right
`tools/*.yaml`; `tests/protocol.rs` checks that every tool's path is a declared route. If it only
reads, list it in the agent file's `allowedTools`.
