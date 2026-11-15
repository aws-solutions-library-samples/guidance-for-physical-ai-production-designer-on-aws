# bt2plc

`bt2plc` turns a behaviour-tree plan from the umarell planner (factoryos-orchestrator) into head-PLC
code that coordinates KUKA robots over VASS / PROFINET. It produces three things:
- **head-PLC code** for CODESYS;
- **a simulator project per robot**, so the cell can run in the docker simulation of
  `profinet-headplc-robot-simulation`;
- **a description of the CODESYS projects and their hardware configuration** (`project.json`,
  `PROJECT.md`, `MCP_PROMPT.md`), which an agent with a CODESYS MCP server uses to build the real projects.

Before it writes anything, the generated ST is type-checked with the embedded
[trust-platform](https://github.com/johannesPettersson80/trust-platform) compiler. The `sim` feature also
co-simulates the head PLC together with the robot simulators.

```sh
cargo run --release -p bt2plc -- generate test_data/bt_1790934549_welding_cell_2_small.json -o out/welding
cargo run --release -p bt2plc -- zones test_data/bt_1790934549_welding_cell_2_small.json
```

## How the tree becomes PLC code

There is no central sequencer. Each robot runs its own op list on its own, inside **one production
Folge** (default 11). That list ends with its MoveHome ops.

Every cross-robot ordering in the tree becomes an **interlock zone**: a mutex held by the robot and
guarded by the head PLC.
- `Sequence` step boundaries and `WaitForRobot.depends_on_ops` become zones.
- `Parallel` branches never share a zone.
- Before an op that depends on another robot, the robot raises the REQUEST bits of the op's zones.
- The PLC grants each zone to one robot at a time, in the planned **ticket order**.
- The robot runs the op, drops REQUEST, and waits for GRANT to drop (a 4-phase handshake).

```
welding_cell_2_small -> 2 zones
IL01 = robot1.W01 > robot2.W15 > robot1.W03 > robot2.W10 > robot1.W05 > robot2.W12 > robot1.R1_PARK > robot2.W09
IL02 = robot2.W13 > robot1.W02 > robot2.W14 > robot1.W06 > robot2.W11 > robot1.W04 > robot2.W07 > robot1.HOME
robot1: W01[IL01] W02[IL02] W03[IL01] W06[IL02] W05[IL01] W04[IL02] R1_PARK[IL01] HOME[IL02]
```

Zones are chains of ops that the tree already orders, so they never serialise ops the tree lets run
in parallel.
- Covering every reduced cross-robot edge enforces the tree's happens-before order **exactly**.
- Planned grants with all-or-nothing multi-zone requests cannot deadlock.
- `ir::validate` checks the equality of the two orders on every plan, and the property tests check
  it on random trees.

The ticket order needs no geometry. `--grant fcfs` grants a free zone to whichever robot asks first
instead. That still guarantees mutual exclusion, but **not** the tree's order, so it prints a warning.

Up to 31 zones fit in the DWORD. Dependencies beyond that (for example a leader/follower tree with
many gates) use bit 31, the **permit** bit. The PLC grants it once the listed predecessor ops of the
other robots are done.

| BT node | Mapping |
|---|---|
| Sequence, SequenceWithMemory, SubTree | ordered steps |
| Parallel (`success_count=-1`, `failure_count=1`) | concurrent branches; other thresholds are rejected (E003) |
| ExecuteTrajectory | one op of the robot program (trajectory + hold) |
| MoveHome | op (planned path, or a blind joint move home) |
| WaitForRobot `depends_on_ops` | zone dependencies |
| Delay | robot-side start delay after the grants (W007 if the paths differ) |
| Timeout | per-op watchdog limit |
| CheckSafety | continuous safety supervision (W005) |
| `depends_on_joints/positions`, `has_interlock` | error (E019/E020); `--ignore-unsupported-gates` makes them warnings |
| anything else | error E002 with the BT path |

## Protocol profiles

`protocol.profile` selects the robot-side protocol the head PLC talks:

| | `sim` (default) | `vw-ks24` |
|---|---|---|
| Robots | the generated device simulators, a plain KSS controller | real VW KS24 robots (KSS 8.2, TPVW 8.1.8) |
| Folge handshake | JobRequest / JobAck / ProgStart / ProgRunning / ProgFinished | `PRG_NR` `$IN[1..8]`, `SRB` `$IN[9]`, echo `$OUT[1..8]`, Folge end `$OUT[14]`, ready `$OUT[9]` |
| Op index | bytes 4-5 (configurable) | `P_PktNr_SPS` `$OUT[97..112]` |
| Zones | REQUEST/GRANT DWORDs, 4-phase, up to 31 zones + permit bit | the standard robot interlocks: request `$OUT[80+n]`, grant `$IN[80+n]`, release `$OUT[40+n]`, partner release `$IN[40+n]`, at most 16, no permit bit |
| Homing | blind HOME Folge | none: `cell.src` starts a Folge only at home, so recovery is manual (`fbCell.bHomingRequired`) |
| PLAN_ID | bytes 4-5 | optional, on a free byte (`protocol.plan_id_byte`, 0 = off) |

In `vw-ks24` the head maps the two VW interlock signals onto one REQUEST bit per zone:
`A(80+n) OR NOT A(40+n)` ("requesting or occupying"), so the arbiter, the ticket order and the
mutual-exclusion proof are unchanged. The VW scheme has no 4-phase handshake, so the robot must
hold its release level long enough for the head to see it (bt2krl emits a dwell after every
release) and `ERR_HANDSHAKE` is debounced. Zone to interlock index comes from
`[[robot]] interlocks` and is written into `program.json`, so bt2krl uses the same map.

The robot side for both profiles is generated by [`../bt2krl`](../bt2krl/README.md) (bt2krl); `plan2cell build` runs both together.

## VASS telegram additions (offsets configurable)

| Direction | Bytes | Content |
|---|---|---|
| robot → PLC | 4-5 | current op index (1-based) |
| robot → PLC | 28-31 | zone REQUEST bits (bit z-1 = zone z, bit 31 = permit) |
| PLC → robot | 4-5 | plan id, written with the production job (the robot rejects a mismatch) |
| PLC → robot | 12-15 | zone GRANT bits |

All other bits are used as deployed:
- DrivesOn, ProgStart, Stop, EXT, Reset (held 1 s), JobRequest, and the Folge number in bytes 2-3
  (`--job-number byte` uses byte 2 only);
- joint angles in bytes 16-27;
- PROFIsafe in slot 1.

## Outputs

```
out/
  ZONES.md  cell_plan.json  robots/<ns>/program.json           flavour-independent plan, robot program specs
  project.json  PROJECT.md  MCP_PROMPT.md                       CODESYS projects + hardware configuration
  codesys/head/{DUT,GVL,POU}/*.st  io_mapping.csv               head PLC (POU = <Name>.st + <Name>_Body.st)
  codesys/robots/<ns>/{DUT,GVL,POU}/*.st  io_mapping.csv        one robot simulator project per robot
  docker/docker-compose.robots.yml  docker/README.md            extra device containers for robots 2..N
  trust/<project>/src/*.st                                       the same code rendered for trust-platform
```

The CODESYS files follow the existing project's conventions:
- declaration and implementation are kept separate (CODESYS object model, and MCP `set_pou_code`);
- Hungarian prefixes;
- `(* *)` comments.

Templates live in `templates/codesys/` and are written in the common subset of CODESYS and trust
described in `docs/ST_DIALECT.md`.

`project.json` takes its DeviceIDs from the working exports
(`profinet-headplc-robot-simulation/codesys-project/exports`). It contains:
- the PLC: `4102 / '0000 0005' / 4.20.0.0`;
- Ethernet `110`;
- PN-Controller `80 / 4.7.1.0`;
- per robot a KUKA GSDML device (`81`, DAP `DIM 2`) with FDIO64 and DIO256 modules, station
  `kuka-robi<i>` and IP `192.168.100.(10+i)`; with `[[robot]] safety_module = false`
  (SIMULATION ONLY, needs `safety_bypass = true` and `external = true`, else a config error,
  exit 3) DAP `DIM 3` (`'... - DIM 3'`, `'V8.3 - HW 1'`) with slot 1 empty (`"empty": true`,
  `"module_id": null`, step `ensure_empty_slot`: the FDIO64 CODESYS plugs there by default is
  removed) and DIO256 in slot 2; no FDIO64 IO mapping rows for it (its `bySafe*` stay unmapped,
  the bypass ignores them);
- parameter ids, IO mappings, tasks, libraries, the objects in creation order, and build steps.

Robot addressing and every protocol constant can be changed in `bt2plc.toml`
(`bt2plc config > bt2plc.toml`).

## Commands

```
bt2plc generate <plan> -o DIR [-c bt2plc.toml] [--grant planned|fcfs] [--job-number word-le|byte]
               [--prod-folge N] [--home-folge N] [--robots ns,..] [--no-robot-sim] [--no-check] [--strict] [--force] [--dry-run]
bt2plc check    <plan> [--json] [--cross-check]   plan + in-memory generation + trust check, writes nothing
bt2plc zones    <plan> [--format text|md|json]
bt2plc simulate <plan> [--cycles N] [--timeline]  needs --features sim
bt2plc config | bt2plc flavours
```

The input is a plan JSON (envelope, a bare `behaviour_tree` object, or a root node) or BT.CPP v4 XML
(via [`bt-core`](../bt-core), the BT.CPP model the btgui viewer uses too, with `--blackboard` for the trajectories).

Exit codes:

| Code | Meaning |
|---|---|
| 0 | ok |
| 1 | plan rejected |
| 2 | usage |
| 3 | config |
| 4 | I/O |
| 5 | flavour not implemented |
| 6 | `--strict` |
| 7 | simulation check failed |
| 8 | generated ST rejected by trust |

## Flavours

Only `codesys` is implemented. `twincat3`, `tia-scl`, `plcopen-xml` and `rockwell-l5x` are planned;
selecting one exits with code 5.

What a new flavour can reuse:
- the IR (`ir::CellPlan`);
- the constant-table layouts (plain CSR arrays, integer milliseconds);
- `backend::st_fmt` for IEC-ST dialects;
- the flavour-independent outputs.

What it adds:
- an implementation of `backend::Backend` (plan → `PlcProject`s → files);
- its own templates.

The FBs avoid pointers, `REFERENCE`, methods and OOP, so SCL and L5X ports are possible.

## Tests

From the repository root:

```sh
cargo test -p bt2plc                       # IR, property tests (random trees), lint, golden, CLI, trust type check
cargo test -p bt2plc --features sim        # + co-simulation of the generated ST (trust-runtime; slow first build)
BT2PLC_BLESS=1 cargo test -p bt2plc --test golden   # update golden files
```

## Limitations

- **Not safety-rated.** Zones are logical ordering interlocks on a standard PLC, derived from the plan,
  not from geometry. Collision freedom relies on the planner's validation. A real cell still needs
  safety-rated workspace monitoring.
- **Abort recovery.** After an abort mid-cycle, or a start-up with any robot away from home, the cell waits
  for `bRecoveryConfirm` before it homes the robots blind, one after the other. The cycle then restarts
  from op 1.
- **CODESYS compile.** The real acceptance test is the CODESYS compile through `MCP_PROMPT.md` on
  Windows. trust is stricter than CODESYS, but it is a different compiler.
- **Robot programs.** KRL robot programs are generated by [`../bt2krl`](../bt2krl/README.md) (bt2krl) from `robots/<ns>/program.json` (`schema_minor` 1: zones, timing, phases and the
  robot-side protocol constants; `[[robot]] model` names the robot model, `[[robot]] joint_map`
  selects an (unvalidated) URDF -> KUKA preset instead of `joint_sign`/`joint_offset_deg`).
