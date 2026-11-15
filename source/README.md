# Physical AI Production Designer: the source tree

This directory is the runnable tree of the
[Guidance for Physical AI Production Designer on AWS](../README.md): everything the Guidance builds and
runs. The Guidance's own README (architecture, cost, prerequisites, deployment, cleanup) is [../README.md](../README.md).
The paths and commands below are relative to this directory (`source/`).

Physical AI Production Designer (FactoryOS in the code: `factoryos-*`) designs a multi-robot welding cell and plans its
motion in the browser: from a robot and a welding tool to a laid-out cell, a welding task on a product,
collision-checked trajectories for every robot, and the cell's control code.

## What it does

1. **Robo Dresser**: mounts a tool on a robot (URDF and meshes) and computes the dressed robot's collision hulls.
2. **Robo Fitter**: lays the dressed robots, fixtures and a floor plan out into a cell, and checks which robot
   reaches which weld.
3. **Product & Task**: the product to weld (here a box and a cube with spot welds) and the welding task on it.
4. **Planning**: assigns the welds to the robots and plans every motion, with a choice of planners:
   - learned policies (hierarchical reinforcement learning and PPO; the checkpoints are included) and a scripted
     generator, for the reference cell they were trained on;
   - umarell's planning tools (OMPL, its gates and behaviour-tree assembler), driven by a deterministic parallel
     orchestrator, for any cell;
   - optionally, an agent that plans from a request in plain words (Kiro, with models on Amazon Bedrock).

   Every plan becomes a behaviour tree and is checked before it is offered: joint and speed limits, robot-robot and
   robot-environment collisions, the timing of every leg.
5. **Behaviour tree executor**: runs the plan on the simulated cell (the cell runtime: Pinocchio kinematics, Coal
   collision checks) and shows it in the 3D view.
6. **Control code**: generates the head PLC's program (IEC 61131-3 Structured Text, for CODESYS) and the robots'
   KUKA KRL programs from the plan.

## Layout

| Path | What |
|---|---|
| `crates/factoryos-viewer/` | the viewer: native, and WebAssembly for the browser (Rust, egui, on Rerun 0.38.1) |
| `crates/factoryos-backend/` | the backend: HTTP and WebSocket API, the Zenoh bus, Robo Dresser, Robo Fitter, Product & Task, the planner chat, code generation |
| `crates/factoryos-protocol/`, `crates/factoryos-client/` | the wire protocol, and the viewer's client for it |
| `crates/factoryos-mcp/`, `crates/openapi-mcp/` | the MCP server the planning agent calls the tools through |
| `crates/urdf-robo-dresser/`, `crates/robo-fitting-room/` | the Robo Dresser and Robo Fitter libraries |
| `crates/factoryos-pty/`, `crates/factoryos-terminal-agent/` | terminals in the browser |
| `crates/factoryos-urdf-materials/` | URDF mesh colours from their `.mtl` files |
| `cell/` | the cell runtime (a workspace of its own): `cell-runner` (Pinocchio and Coal through a C++ wrapper, the behaviour-tree executor), `krl-parser`, `cell-rerun-server` |
| `plan2cell/` | behaviour-tree plans to control code (a workspace of its own): `bt2plc` (Structured Text), `bt2krl` (KRL), the `plan2cell` CLI and MCP server, the behaviour-tree view |
| `cell-planner/` | the planners' runtime: the policies and their checkpoints, the scripted generator, the U3 orchestrator, the plan exporter and its checks (see `cell-planner/README.md`) |
| `deploy/` | the host's setup: installing a release, its Python environments, the planner router (`deploy/planner-router/`), the systemd units, the agent's configuration (`deploy/kiro/`) |
| `web/` | the web viewer's page and its zenoh-ts bundle |
| `third_party/rerun/` | the Rerun release the viewer builds on (`PIN`) and the patches applied to it |
| `assets/` | sample data: the reference cell `welding_cell_2_small` and the catalogue (a KUKA KR 240 R2900 robot, a stick welding tool, fixtures, products, scenes, a floor plan) |
| `scripts/` | build scripts |
| `docs/` | documentation |
| `factoryos.config.json` | the backend's settings; empty paths resolve against this directory |

The one-step deployment of the Guidance (an AWS CDK app) is in [../deployment/](../deployment/), next to this
directory; README images are in [../assets/images/](../assets/images/).

## Building from source

On Ubuntu 24.04 with the build dependencies of the Guidance README's
[Operating System](../README.md#operating-system) section. The viewer and the backend (Rust, the toolchain of
`rust-toolchain.toml`):

```sh
scripts/fetch-rerun.sh                  # Rerun at third_party/rerun/PIN into third_party/rerun/src, patched
cargo build --release --locked -p factoryos-backend -p factoryos-mcp
scripts/build-web.sh                    # the web viewer into web/dist/
```

`cargo build --release --workspace` also builds the native viewer. The cell runtime needs Pinocchio and Coal (found
through pkg-config): `cell/scripts/build-cell-rerun-server.sh`. plan2cell is built in `plan2cell/` with
`cargo build --release --locked -p plan2cell-cli --features sim,cell`. The planners need Python 3.12 with
`deploy/requirements-cell-planner.txt`.

Tests:

```sh
cargo test --release --locked -p factoryos-protocol -p factoryos-client -p factoryos-backend
(cd cell && cargo test --release --locked --workspace --features cell-runner/collision)
(cd plan2cell && cargo test --release --locked --workspace)
python3 -m unittest discover -s deploy/planner-router/tests
```

## Running locally

```sh
./target/release/factoryos-backend --bind 127.0.0.1:8080 --web-root web/dist
# then open http://127.0.0.1:8080/
```

Built in this directory, the backend finds the sample cells, the catalogue, `factoryos.config.json`, the cell
runtime (`cell/target/release/cell-rerun-server`) and plan2cell (`plan2cell/target/release/plan2cell`) here.

The backend has no authentication of its own: run it on loopback (`--bind 127.0.0.1:8080`) and reach it through an
SSH tunnel or AWS Systems Manager port forwarding, or behind an authenticating proxy (the one-step deployment puts it
behind Amazon CloudFront with an Amazon Cognito sign-in).

The viewer plans through the chat of a Kiro CLI agent, for every planner (**Plan Cell**). Using it needs a Kiro
subscription and is subject to Kiro's terms ([kiro.dev](https://kiro.dev)); only the agentic planner also needs
Amazon Bedrock.

## Security

See [CONTRIBUTING](../CONTRIBUTING.md#security-issue-notifications) for more information.

## License

This library is licensed under the MIT-0 License. See the [LICENSE](../LICENSE) file. The third-party software it
builds on, and their licences: [THIRD-PARTY-NOTICES.md](../THIRD-PARTY-NOTICES.md).
