# plan2cell

plan2cell turns the behaviour trees produced by the FactoryOS planner (umarell, driven by
factoryos-orchestrator) into the code that runs a robot cell, and lets you look at and execute
those trees. One build produces, from one plan:
- **head-PLC code** that coordinates the robots over VASS / PROFINET (CODESYS Structured Text),
  plus a simulator project per robot and a full description of the CODESYS projects;
- **KUKA KRL robot programs** (KSS 8.x) that implement the robot side of the same protocol;
- the two **consistent by construction**: one PLAN_ID, one protocol profile, one zone map.

The same pipeline is a CLI (`plan2cell`), an **MCP server** (stdio and network), and a set of
libraries. The repository also contains **btgui**, a Groot-style viewer and step executor for
the trees.

> **Not safety-rated, not yet tested on a controller.** The interlock zones are logical ordering
> interlocks derived from the plan, not from geometry; a real cell still needs safety-rated
> workspace monitoring. The generated KRL passes a strict validator, an IR round trip and a
> co-simulation against the generated head PLC, but has not run on a KUKA controller yet; see
> [`crates/bt2krl/docs/USER_GUIDE.md`](crates/bt2krl/docs/USER_GUIDE.md#6-before-the-first-automatic-run).

## Repository layout

One Cargo workspace (`Cargo.toml` at the root, toolchain pinned in `rust-toolchain.toml`):

| Path | What it is |
|---|---|
| [`crates/plan2cell`](crates/plan2cell) | The combined pipeline: plan -> head PLC + robot programs, with build options and one `plan2cell.toml`. |
| [`crates/plan2cell-cli`](crates/plan2cell-cli) | The `plan2cell` command: `build`, `check`, `zones`, `explain`, `simulate`, `config`, `options`, `mcp`. |
| [`crates/plan2cell-mcp`](crates/plan2cell-mcp) | The pipeline as MCP tools, over stdio and streamable HTTP ([`docs/MCP.md`](docs/MCP.md)). |
| [`crates/bt2plc`](crates/bt2plc/README.md) | Head-PLC generator (CODESYS), robot simulators, project/hardware manifests, trust type check and co-simulation. Design: [`crates/bt2plc/docs/`](crates/bt2plc/docs). |
| [`crates/bt2krl`](crates/bt2krl/README.md) | KUKA KRL generator, profiles `sim` and `vw-ks24`. User guide: [`crates/bt2krl/docs/USER_GUIDE.md`](crates/bt2krl/docs/USER_GUIDE.md). |
| [`crates/bt-core`](crates/bt-core) | BehaviorTree.CPP model (XML), builtin node manifest, Groot layout. Used by bt2plc for XML input and by btgui. |
| [`crates/bt-exec`](crates/bt-exec) | A BT.CPP v4 interpreter that records every status transition. |
| [`crates/bt-cell`](crates/bt-cell) | factoryos cell plans on bt-exec: plan parsing, motion schedule, cell leaves. |
| [`crates/bt-view`](crates/bt-view), [`crates/btgui`](crates/btgui/README.md) | The Groot-style viewer panel (egui) and its desktop window. An embedder can overlay a live source: `uid_for_operation`, `Action::SetExternalActive` (a ring of its own) and `drift` (FactoryOS' Behaviour Tree pane follows the cell with them). |
| [`examples/`](examples) | `plan2cell build` output for the two sample plans (`plc/`, `krl/`, `BUILD.md`), and their `plan2cell.toml`. |
| [`test_data/`](test_data) | Sample plans and BT.CPP trees shared by all crates' tests. |
| [`docs/`](docs) | Cross-crate documentation (MCP server, btgui screenshot). |
| [`ci/`](ci/README.md), [`.gitlab-ci.yml`](.gitlab-ci.yml) | CI: builds and tests in the dev AWS account, Linux binaries to the release S3 bucket. |
| [`crates/kuka-krl-parser`](crates/kuka-krl-parser), [`crates/krl-parser-smoke`](crates/krl-parser-smoke) | A copy of krl-runner's lenient KRL parser, and the parser smoke check of generated KRL with it. |

## Quick start

Requirements: Rust (rustup installs the pinned 1.96.0), and network access to GitHub on the first
build (trust-platform git dependencies).

```sh
cargo run --release -p plan2cell-cli -- zones test_data/bt_1790934549_welding_cell_2_small.json
cargo run --release -p plan2cell-cli -- build test_data/bt_1790934549_welding_cell_2_small.json \
    -c examples/plan2cell.toml -o out/welding
cargo run --release -p btgui -- test_data/bt_1790934549_welding_cell_2_small.json   # view / execute the tree
```

`out/welding/` then holds `plc/` (head PLC, robot simulators, `project.json`, `MCP_PROMPT.md`),
`krl/<ns>/` (the robot programs with their `INSTALL.md`) and `BUILD.md`.

To build the CODESYS projects, give an agent with a CODESYS MCP server (on Windows)
`plc/MCP_PROMPT.md` and `plc/project.json`; `plc/PROJECT.md` is the same for a manual build. Run
them in the docker simulation of `profinet-headplc-robot-simulation`
(`plc/docker/docker-compose.robots.yml` adds a simulator container for each extra robot).

## The `plan2cell` command

```
plan2cell build    <plan> -o DIR [-c plan2cell.toml] [options] [--force] [--dry-run]
plan2cell check    <plan> [-c ...] [options] [--json]     build in memory, validate everything, write nothing
plan2cell zones    <plan> [-c ...]                        interlock zones, ticket order, each robot's ops
plan2cell hardware <plan> [-c ...] [--json]               effective network, stations, IPs, KUKA VCs; every addressing issue
plan2cell explain  <plan> --robot NS [-c ...]             one robot's KRL plan and timing
plan2cell simulate <plan> [--cycles N] [--robots krl|device-st]   co-simulation (build with --features sim)
plan2cell config                                          the default plan2cell.toml
plan2cell options  [--json]                               PLC dialects, robot languages, profiles, defaults
plan2cell mcp      --stdio | --http ADDR [--root DIR] [--allow-write] [--token T]

options: --profile sim|vw-ks24   --plc codesys|none   --robot-sim/--no-robot-sim
         --krl/--no-krl   --krl-robots ns,..   --no-check   --strict   --blackboard FILE
         --plc-dialect codesys|twincat3|tia-scl|plcopen-xml|rockwell-l5x
         --robot-language krl|rapid|urscript|karel|inform   --krl-e-axes-default none
build:   --json   one JSON report on stdout
```

### Machine-readable interface (front ends)

FactoryOS' Control Code Export step runs the binary rather than linking the crates (the
trust-platform dependencies pin another zenoh than the backend's). Two commands are meant for that:

- `plan2cell options --json` prints `plan2cell/options@1`:
  - `plc_dialects`, generated from bt2plc's `Flavour` (`implemented` is `Flavour::implemented()`: only `codesys`);
  - `robot_languages`: `krl` (implemented), and `rapid`, `urscript`, `karel`, `inform` (planned);
  - `profiles` (`sim`, `vw-ks24`) and `job_numbers` (`word_le`, `byte`), each `{id, label, implemented, description}`;
  - `defaults`: `profile`, `plc_dialect`, `robot_language`, `robot_sim`, `krl`, `st_check`, `strict`, `job_number`, `opcua_symbols`;
  - `st_check_available` and `sim_available`: the features of this binary.
- `plan2cell build <plan> -c plan2cell.toml -o DIR --json` writes the build as without `--json`,
  then prints one `plan2cell/build-report@1` document on stdout. It does so whether the build
  succeeds or fails; the exit code stays the same. The report has:
  - `ok`, `exit_code` and `error`;
  - `summary_line` and `summary` (the `plan2cell-build.json` summary);
  - `diagnostics`: `[{severity, code?, message, file?, line?, location?, source}]`. `source` is
    `plan`, `plc`, `krl` or `st`, the trust check, whose lines carry `file:line` relative to the
    build directory. A `--strict` failure also lists the warnings it promoted;
  - `files`: `[{path, bytes}]`;
  - `notes`, `out` and `written` (false with `--dry-run`).
- `plan2cell hardware <plan> -c plan2cell.toml --json` prints `plan2cell/hardware@1`, the hardware
  a build of this plan and config uses (bt2plc's own `Config::addressing`, so no constants of its own):
  - `network {headIp, netmask, prefix, gateway, controllerStation, nic, robotHostBase, ipPoolFirst,
    ipPoolLast}` (`gateway` `0.0.0.0` = none; the pool follows the head subnet unless `ip_pool_first` /
    `ip_pool_last` are set, and must lie in it) and `networkDefaults` (the same without `[plc.network]`);
    device names must be IEC identifiers that are not names of the generated projects or IEC words
    (`bt2plc::hardware::RESERVED_DEVICE_NAMES`, issues `robots.<ns>.deviceName`);
  - `robots [{ns, index, tag, station, deviceName, ip, model, external, safetyBypass, safetyModule,
    idle, defaults {station, deviceName, ip}}]` in plan order; `defaults` = without that robot's
    `[[plc.robot]]` station / device_name / ip;
  - `kukaVc {kliPrefix, kliGw, headIp, controllers [{robotNs, kliIp, station}]}`: one controller per
    `external` robot, `kliIp` = its IP, `station` = its station (`kvc-cell.env` `C<N>_KLI_IP`, `C<N>_STATION`);
  - `ok` and `issues [{field, message}]`: every broken rule at once (`field` e.g. `network.gateway`,
    `robots.robot2.station`, or `config`). The rules are `bt2plc::hardware::check`, which the plan also
    applies, so `build` refuses the same config: NameOfStation per IEC 61158-6-10 (at most 240
    characters, labels of 1-63 separated by `.`, only `a-z0-9-`, no `-` at a label's start or end, not
    `n.n.n.n`, not starting with `port-xyz` / `port-xyz-abcde`), robot IPs inside the head subnet and
    not its network, broadcast, gateway or head address, IPs and stations (the head's included) unique.
  - Exit 0 without issues, 3 with issues (the report is printed either way). When the plan cannot be
    read, `{"ok": false, "error": {code, message}, "issues": []}` with the plan's exit code.
  - Overrides: `[plc.network]` (`head_ip`, `netmask`, `gateway`, `controller_station`,
    `robot_host_base`) and `[[plc.robot]]` (`station`, `device_name`, `ip`, `model`, `external`, ...).
- `--plc-dialect` and `--robot-language` select the target. A planned one fails cleanly with exit
  code 5 (bt2plc's `NotImplemented` for PLC dialects); an unknown id is a usage error (2).
- `--krl-e-axes-default none` declares `e_axes = "none"` for each robot whose `[[krl.robot]]` does
  not declare its external axes. This is an explicit opt-in, since bt2krl itself never guesses.
  Each robot it covers gets a note.

What to build:

| You want | Flags |
|---|---|
| Everything for the docker simulation | (defaults: `--profile sim`, CODESYS, robot simulators, KRL) |
| Head PLC only | `--no-krl` |
| Robot programs only | `--plc none` (the plan outputs they come from are still written) |
| Real VW KS24 robots | `--profile vw-ks24` (head and robots switch together; no simulators, no blind homing) |
| Some robots only | `--krl-robots robot1,robot3` |

`--profile` drives both sides: with `vw-ks24` the build sets the 8-bit job number and the 16-zone
limit, never puts PLAN_ID on bytes the VW standard uses, takes the zone -> interlock map from
`[[plc.robot]] interlocks` for both the head and the KRL, and says in `BUILD.md` what it adjusted.

Exit codes are those of bt2plc / bt2krl: 0 ok, 1 plan rejected, 2 usage, 3 config, 4 I/O,
5 not available in this build, 6 `--strict`, 7 verification or simulation failed, 8 generated
code rejected.

The two generators keep their own commands (`cargo run -p bt2plc -- ...`,
`cargo run -p bt2krl -- ...`) for working on one side only.

### plan2cell.toml

```toml
[build]                 # defaults of the flags above
profile = "sim"
plc = "codesys"
robot_sim = true
krl = true

[plc]                   # exactly bt2plc.toml: cell, protocol, timing, network, codesys, robot_sim
[[plc.robot]]
ns = "robot1"

[krl]                   # exactly bt2krl.toml: output, motion, sim, vw_ks24
[[krl.robot]]
ns = "robot1"
e_axes = "none"         # required per robot when KRL is built
model = "KR240R2900_ULTRA"
```

`plan2cell config` prints every key with its default. The references are in
[`crates/bt2plc/README.md`](crates/bt2plc/README.md) and
[`crates/bt2krl/docs/USER_GUIDE.md`](crates/bt2krl/docs/USER_GUIDE.md#4-bt2krltoml-reference).

## MCP server

```sh
plan2cell mcp --stdio --root . -c examples/plan2cell.toml                    # for a client that starts it
plan2cell mcp --http 127.0.0.1:8765 --root . --allow-write                 # local network service
PLAN2CELL_MCP_TOKEN=... plan2cell mcp --http 0.0.0.0:8765 --allowed-host cell-pc.local --root /srv/plans
```

Tools: `plan2cell_zones`, `plan2cell_check`, `plan2cell_build`, `plan2cell_explain_robot`,
`plan2cell_default_config`, `plan2cell_list_files`, `plan2cell_read_file`, and `plan2cell_simulate` in a
`--features sim` build. Files are confined to `--root`, writing is opt-in, and a non-loopback
address needs a bearer token. Details and client setup: [`docs/MCP.md`](docs/MCP.md).

## Run a cell on AWS

[`infra/`](infra) (CDK) builds the CODESYS PROFINET simulation in the dev account: a PLC host with one
CODESYS Control SL container per PLC, a Windows CODESYS IDE with a CODESYS MCP server, and a Linux
workspace with plan2cell, Claude Code and Kiro. On the workspace, `plan2cell cell` (build with
`--features cell`) takes a plan to running, verified PLCs:

```sh
plan2cell cell up test_data/bt_1790934549_welding_cell_2_small.json -c examples/plan2cell.toml -o ~/cells/welding
```

Deploy order, open items, agent prompt (`/cell-up`), troubleshooting and teardown:
[`docs/CELL.md`](docs/CELL.md). The same per-workspace cell can also run on EKS (preview, beside EC2): the PLC and the KUKA VCs as Linux
containers on a PROFINET overlay (Multus + VXLAN across the nodes), the IDE as a Windows container, chosen per cell
(`runtime: eks`): [`infra/eks/README.md`](infra/eks/README.md). With `-c liveViewer=true` the stack adds a FactoryOS viewer that shows the
running cell live (KUKA joints over vass-io Zenoh, head PLC state over OPC UA, read-only), behind the
FactoryOS edge as workspace `plan2cell-live`: [Live visualisation](docs/CELL.md#live-visualisation-factoryos-viewer). Not yet deployed or run against a real CODESYS.

## How a behaviour tree becomes PLC and robot code

The PLC does not sequence individual ops. Each robot runs its own op list inside one production
Folge. Every cross-robot ordering in the tree becomes an **interlock zone**: a mutex the robot
requests before the op and releases after it, and the head PLC grants it in the planned ticket
order. Ops the tree runs in parallel never share a zone. The order the zones enforce equals the
tree's happens-before order exactly; this is checked on every plan and property-tested on random
trees.

```
welding_cell_2_small -> 2 zones
IL01 = robot1.W01 > robot2.W15 > robot1.W03 > robot2.W10 > robot1.W05 > robot2.W12 > robot1.R1_PARK > robot2.W09
IL02 = robot2.W13 > robot1.W02 > robot2.W14 > robot1.W06 > robot2.W11 > robot1.W04 > robot2.W07 > robot1.HOME
```

bt2krl then writes, per robot, the Folge that does exactly that robot side: report the op index,
request the op's zones, wait for the grant, move along the planned joint trajectory, hold,
release. Protocol, node mapping and outputs: [`crates/bt2plc/README.md`](crates/bt2plc/README.md);
the robot side and both profiles: [`crates/bt2krl/README.md`](crates/bt2krl/README.md).

## Tests

```sh
cargo test                                   # every crate but the GUI shell (fast)
cargo test -p btgui                          # the desktop window (needs a display stack to build)
cargo test -p bt2plc --features sim          # + head PLC co-simulated with the device-simulator ST
cargo test -p bt2krl --features cosim        # + head PLC co-simulated with the KRL executor (both profiles)
cargo test -p plan2cell --features sim         # + a combined build co-simulated end to end
```

The co-simulation suites build trust-runtime and run tens of minutes of virtual-time simulation.
`examples/` is checked against a fresh build by `cargo test -p plan2cell`; regenerate it with
`plan2cell build test_data/<plan> -c examples/plan2cell.toml -o examples/<cell> --force`.

## CI and releases

Every pipeline builds and tests on ephemeral x86 and arm64 EC2 instances in the dev account
(factoryos-orchestrator's build stack) and archives `plan2cell-linux-<x64|arm64>-<id>.tar.gz`
(`plan2cell`, `bt2plc`, `bt2krl`, `kuka-krl-parser`, `btgui`) in the CI archive bucket,
`s3://factoryos-builds-<account>/plan2cell/` (`releases/<tag>/`, `regular-builds/p<pipeline>/`).
A tag also creates a GitLab Release. Setup and details: [`ci/README.md`](ci/README.md).

## Status

| Area | State |
|---|---|
| CODESYS head PLC + robot simulators | Type-checked with trust and co-simulated: the welding cell runs 2 cycles at 64.7 s against a 64.3 s plan with every dependency respected; abort -> recovery and permit overflow are tested. The first real CODESYS build through MCP is still open. |
| KUKA KRL robot programs | Profiles `sim` and `vw-ks24`, PTP motion (spline refused: unverified on KSS 8.2). Strict validator, IR round trip, co-simulation against the generated head PLC in both profiles. Not yet run on a controller. |
| MCP server | stdio and streamable HTTP, sandboxed to a root directory, bearer token for network use. |
| btgui | Viewer and step executor for BT.CPP trees and factoryos plans. |
| TwinCAT 3, Siemens TIA SCL, PLCopen XML, Rockwell L5X | Planned (`bt2plc --flavour` exits 5). |

## Related projects

- **factoryos-orchestrator / umarell planner:** produces the behaviour-tree plans (input).
- **profinet-headplc-robot-simulation:** the CODESYS head-PLC <-> KUKA VASS PROFINET simulation the
  templates and hardware configuration are derived from.
- **robotic-cell-code-runner:** the VW KS24 robot exports and krl-runner's KRL parser that bt2krl's
  checks are built on (read-only references).
