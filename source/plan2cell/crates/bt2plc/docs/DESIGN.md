# bt2plc design document (implementation-ready)

Scope: a Rust CLI, this crate (`plan2cell/crates/bt2plc`; designed as `groot/bt2code/plc` of the behaviour-tree project). It lowers an umarell BT plan to a happy-path IR (HB order, reduced cross-robot edges, zones), then emits the following. Nothing is executed.
- CODESYS head-PLC code
- complete per-robot CODESYS device-sim applications
- a machine-readable project/hardware manifest, PROJECT.md and MCP_PROMPT.md
- flavour-independent artefacts

---

## 0. Gaps and flaws in the decided design, and how this design closes them

| # | Gap / flaw | Resolution (adopted below) |
|---|---|---|
| F1 | "Release when the holder drops REQUEST" is ambiguous. A robot that stops, faults or loses its PN connection also drops REQUEST (inputs zeroed). The zone would then be re-granted while the arm may still be inside it. | A release is valid only if the holder robot is `bJobActive` (ProgRunning, production job) and not in error. Otherwise `ERR_RELEASE_INVALID`: the arbiter fails closed (all grants dropped, latched until a reset that rewinds the tickets). |
| F2 | A robot that keeps REQUEST high from op k to op k+1 for the same zone (consecutive tickets of one robot, possible after greedy appends) makes the release invisible. | The 4-phase handshake is mandatory. The robot must see `(GRANT AND mask)=0` before raising REQUEST for the next op. The PLC clears a GRANT bit in the scan it sees the REQUEST bit drop. |
| F3 | Robot program and PLC tables can drift (old robot program, new PLC tables). | 16-bit `PLAN_ID` (fold of the plan fingerprint). The PLC writes it to PLC→robot param bytes 4-5 with the production job. The robot rejects ProgStart on a mismatch (error `16#0103`). On top of that, the PLC checks the step sequence (+1 only), request mask == table mask, own-ticket mismatch, tickets fully consumed at cycle end, and max step == op count. |
| F4 | `bJobDone`-style latches race: a previous job's done flag is seen as the completion of the next command. | Level-request / done-ack handshake. `bJobDone` is cleared while `bCmdJob=FALSE`. The cell drops the command after seeing done. A command edge that arrives while the robot FB is not idle is latched. |
| F5 | Very short jobs (home Folge with the robot already home, or a 1-op program) can go Running→Finished between two PLC scans. The existing FSM then times out in WAIT_RUNNING. | The device clears ProgFinished on job accept, so `ProgFinished=1` seen after JobAck means *this* job finished. WAIT_RUNNING goes straight to DONE in that case. |
| F6 | CODESYS device-object names are project-wide IEC identifiers. Two robots cannot both have modules named `SafeIO_FDIO64` / `VASS_DIO256` in the head project. | Head modules are named `SafeIO_FDIO64_R<i>` / `VASS_DIO256_R<i>`. Device-sim projects (one robot each) keep the export names. |
| F7 | `Delay` before a `WaitForRobot` (or several delay paths into one op) cannot be expressed exactly by "robot waits δ after its grants". | `start_delay(o)` = max delay along op-free paths into `o`. The robot waits it *after* all grants. This is exact when min == max, otherwise conservative and warns W007. |
| F8 | `Timeout` bounds a whole subtree; the PLC watchdog is per op. | Per-op `tLimit := min(computed, msec)` for every op in the child. Warns W006 if the child has more than one op. |
| F9 | A `WaitForRobot` that references a MoveHome never completes in the reference executor (`_exec_home` never calls `_mark_completed`). | Refs resolve only to ExecuteTrajectory `(robot_ns, operation_id)`. Anything else is E006. |
| F10 | FCFS is only deadlock-free with atomic grants (hold-and-wait otherwise). In a zones-only design it enforces no cross-robot order at all (see §3.9). | Grants are atomic (all zones of an op in one scan) for both policies. FCFS visits robots oldest-request-first (ties by index) and prints a loud W011. |
| F11 | Robots that appear in the cell but not in the BT (e.g. robot2 in `bt_1791274820_cell_2robot`) are invisible to the planner output. | They are excluded by default. `[[robot]] include_if_idle=true` / `--robots` adds them (bring-up and homing only, no production job). |
| F12 | CODESYS forbids zero-length arrays (0 zones, robot without ops). | Every table has a `*_DIM = MAX(1,n)` bound plus a real count. Dummy rows are `(0,0)`. |
| F13 | A program that does not end at home makes every auto-cycle blind-home that robot. | Warning W016. |
| F13a | A first waypoint ≠ home pose (teleport in sim) | Warning W016. |
| F13b | Inter-op chaining gaps | Warning W009. |
| F14 | Zone-wait watchdog: a fixed limit gives false positives on long plans. | Absolute deadline since cycle start: `f·ASAP_start(x) + m·depth(x) + m_zone`. This is provably never exceeded while every op respects its own watchdog `f·d+m` (§3.8). |
| F15 | The existing head FSM quirks: fixed 15 s timeout incl. WAIT_FINISHED; a reset edge jumps to INIT so the robot never sees a 500 ms reset; the echo is never checked; the FB is hard-wired to GVL_IO. | Fixed in FB_VassRobot/FB_Cell (§4.3). |

---

## 1. Crate layout

Standalone package, own workspace root. Paths are relative to the package root (now `plan2cell/crates/bt2plc`).

```
Cargo.toml                         package manifest (below); empty [workspace] = own root
Cargo.lock                         generated (cargo generate-lockfile --offline)
rust-toolchain.toml                1.96.0 pin (same as siblings)
README.md                          usage, outputs, protocol summary, links to ZONES/PROJECT docs
bt2plc.example.toml                fully commented default config (== `bt2plc config --print-default`)
.gitignore                         /target, /out
src/main.rs                        clap CLI, subcommand dispatch, diagnostics printing, exit codes
src/lib.rs                         //! pipeline API: load → lower → plan → generate; re-exports
src/error.rs                       Error (thiserror), Diag/DiagCode/Severity, Diagnostics, BtPath, exit codes
src/config.rs                      TOML Config structs + defaults + validation + robot addressing derivation
src/input/mod.rs                   InputDoc, format sniffing (envelope / bare tree / XML), load()
src/input/envelope.rs              serde structs: planner envelope + behaviour_tree object
src/input/json_tree.rs             RawNode (JSON mirror) → NormNode; legacy dialect normalisation
src/input/xml_tree.rs              bt_core::Document → NormNode; SubTree remap resolution; _skipIf rejection
src/input/blackboard.rs            Blackboard, {key} resolution, typed port access, trajectory parsing/validation
src/ir/mod.rs                      CellPlan, Robot, Op, OpKind, Edge, Zone, Ticket, Trajectory, Waypoint, Schedule
src/ir/bitset.rs                   fixed-size u64 bitset (insert/contains/union_with/iter)
src/ir/lower.rs                    NormNode → EventGraph (frontiers, barriers, gates, delay/timeout attrs)
src/ir/hb.rs                       Kahn topo + cycle extraction, ancestor bitsets, linear extension, robot chains
src/ir/reduce.rs                   pred_Q via two-pointer, cross-edge reduction, third-robot pruning
src/ir/zones.rs                    greedy chain cover, tie-breaks, renumbering, masks, limit check
src/ir/schedule.rs                 ASAP schedule over EventGraph, depth, expected/limit/deadline per op
src/ir/decimate.rs                 optional joint-space RDP decimation (keeps ends + phase boundaries)
src/ir/validate.rs                 IR invariants (chains, ticket order, masks) — used before codegen + tests
src/sim/mod.rs                     time-stepped protocol simulator (PLC scan, robot cycle, IO latency)
src/sim/arbiter.rs                 line-by-line Rust mirror of FB_ZoneArbiter
src/sim/robot.rs                   mirror of FB_ProgramPlayer zone/step logic
src/sim/check.rs                   HB/mutual-exclusion/ASAP-equivalence/overlap checkers
src/backend/mod.rs                 Backend trait, Flavour enum, backend_for(), Output file set (+ manifest of written files)
src/backend/common/mod.rs          shared helpers
src/backend/common/tables.rs       CellTables (CSR tables) — single source for GVL_Cell AND sim
src/backend/common/st_fmt.rs       ST literals: REAL, TIME, STRING escaping, identifiers, comment sanitising
src/backend/common/cell_plan_json.rs   cell_plan.json
src/backend/common/zones_md.rs     ZONES.md
src/backend/common/program_json.rs robots/<ns>/program.json
src/backend/common/docker.rs       docker/docker-compose.robots.yml + docker/README.md
src/backend/codesys/mod.rs         CodesysBackend::generate (head + device sims + manifests)
src/backend/codesys/head.rs        head objects: static templates + GVL_Cell/GVL_IO/PLC_PRG emitters
src/backend/codesys/device_sim.rs  per-robot objects: verbatim/adapted templates + GVL_Program emitter
src/backend/codesys/project_json.rs  serde model of project.json + builder (device tree, IO, tasks, libs, steps)
src/backend/codesys/io_csv.rs      "CoDeSys Mapping Export V1.2" CSV per project
src/backend/codesys/docs.rs        PROJECT.md + MCP_PROMPT.md
src/backend/codesys/script.rs      optional build_project.py (CODESYS ScriptEngine, IronPython)
src/backend/twincat.rs             stub → Error::NotImplemented(Flavour::Twincat3)
src/backend/tia_scl.rs             stub
src/backend/plcopen.rs             stub
src/backend/l5x.rs                 stub
src/util/hash.rs                   FNV-1a 64 (fingerprint, PLAN_ID fold) — no extra deps
templates/codesys/head/DUT/{E_GrantPolicy,E_CellError,E_RobotState,E_CellState,ST_Ticket,ST_OpInfo,ST_RobotIn,ST_RobotOut}.st
templates/codesys/head/POU/{FB_VassRobot,FB_VassRobot_Body,FB_ZoneArbiter,FB_ZoneArbiter_Body,FB_Cell,FB_Cell_Body}.st
templates/codesys/device/DUT/{E_RobotMode,E_ProgramState,ST_SafeInput,ST_SafeOutput,ST_PROFIsafe_Trailer,ST_VASS_StatusWord,ST_VASS_ControlWord,ST_VASS_InputTelegram,ST_VASS_OutputTelegram}.st   (verbatim split of VASS_Types.st)
templates/codesys/device/DUT/{E_PlayerState,ST_OpDef,ST_Waypoint}.st                   (new)
templates/codesys/device/GVL/{GVL_IO,GVL_DeviceParams}.st                              (verbatim)
templates/codesys/device/POU/{FB_PROFIsafe_Simulator,FB_PROFIsafe_Simulator_Body,FB_JointAngleEncoder,FB_JointAngleEncoder_Body}.st (verbatim)
templates/codesys/device/POU/{FB_ProgramPlayer,FB_ProgramPlayer_Body}.st                (new)
templates/codesys/device/POU/{FB_VASS_DeviceSimulator,FB_VASS_DeviceSimulator_Body,PLC_PRG,PLC_PRG_Body}.st (adapted)
tests/common/mod.rs                fixture loading, plan builders, golden compare (BT2PLC_BLESS=1 rewrites)
../../test_data/                  both sample plans (shared with btgui); handcrafted trees are built in tests/common
tests/golden/<sample>/...          expected outputs (see §8)
tests/lowering.rs  tests/zones_welding.rs  tests/protocol_sim.rs  tests/proptest_trees.rs
tests/golden.rs    tests/st_lint.rs        tests/cli.rs            tests/template_drift.rs (opt-in)
```

The templates are copied into the crate once; the existing projects stay untouched. The opt-in `tests/template_drift.rs` (env `BT2PLC_DRIFT=1`) compares the verbatim templates with the originals under `.../groot/profinet-headplc-robot-simulation/codesys-project/kuka-vass-device-simulator/src`.

**Cargo.toml** (the versions are the ones already resolved in btgui/orchestrator `Cargo.lock` and present in `~/.cargo/registry/cache`):
```toml
[package]
name = "bt2plc"
version = "0.1.0"
edition = "2024"
rust-version = "1.96"
license = "MIT"
publish = false
description = "umarell BT.CPP v4 plan -> head-PLC code (CODESYS), robot simulators and build manifests"

# Own workspace root: never attach to a parent workspace.
[workspace]

[lib]
path = "src/lib.rs"

[[bin]]
name = "bt2plc"
path = "src/main.rs"

[dependencies]
bt-core    = { path = "../../btgui/crates/bt-core" }   # pulls roxmltree 0.20.0
serde      = { version = "1.0.229", features = ["derive"] }
serde_json = "1.0.151"
toml       = "0.8.23"          # same major as urdf-robo-dresser (toml_edit 0.22.27 cached)
clap       = { version = "4.6.1", features = ["derive"] }
thiserror  = "2.0.19"
anyhow     = "1.0.104"         # main.rs only

[dev-dependencies]
proptest          = "1.11.0"   # rand 0.9.5 / rand_chacha 0.9.0 / rand_xorshift 0.4.0 cached
pretty_assertions = "1.4.1"
tempfile          = "3.27.0"

[profile.dev.package."*"]
opt-level = 2
```
bt-core inherits `edition`/`roxmltree` from `btgui/Cargo.toml`, and Cargo resolves that from the dependency's own workspace root. To bootstrap offline: `cargo generate-lockfile --offline && cargo build --offline`. If the index cache lacks an entry, run once online.

**rust-toolchain.toml**
```toml
# Same toolchain as btgui / factoryos-orchestrator.
[toolchain]
channel = "1.96.0"
components = ["rustfmt", "clippy"]
```

Conventions:
- `//!` module docs and `// ───── section ─────` banners.
- Library errors use `thiserror`; anyhow is used only in `main`.
- Warnings are collected in `Diagnostics` (btgui-style warning vector, typed).
- Inline `#[cfg(test)]` plus `tests/*.rs`.
- Deterministic output: BTreeMap/Vec only (no HashMap iteration in outputs), no timestamps, fixed float formatting.

---

## 2. Rust types

### 2.1 Input model
```rust
// input/envelope.rs
#[derive(Debug, Deserialize)]
pub struct Envelope {
    pub tree_id: Option<String>,
    pub cell_id: Option<String>,
    pub created_s: Option<f64>,
    pub status: Option<String>,              // "ok" | "failed"; non-ok → E001 (no override)
    pub num_operations: Option<u32>,         // sanity check vs counted ExecuteTrajectory (warn W019)
    pub behaviour_tree: BehaviourTree,
}
#[derive(Debug, Deserialize)]
pub struct BehaviourTree {
    pub tree_id: Option<String>,
    pub cell_id: Option<String>,
    #[serde(rename = "BTCPP_format")] pub btcpp_format: Option<serde_json::Value>, // "4" or 4
    pub main_tree_to_execute: Option<String>,
    pub root: Option<RawNode>,
    #[serde(default)] pub blackboard: BTreeMap<String, serde_json::Value>,
    #[serde(default)] pub metadata: serde_json::Value,
    pub xml: Option<String>,
}
// input/json_tree.rs — JSON mirror (also legacy dialect: ports as top-level fields)
#[derive(Debug, Clone, Deserialize)]
pub struct RawNode {
    #[serde(rename = "type")] pub kind: String,
    pub name: Option<String>,
    pub id: Option<String>,
    #[serde(default)] pub ports: BTreeMap<String, serde_json::Value>,
    #[serde(default)] pub children: Vec<RawNode>,
    #[serde(flatten)] pub legacy: BTreeMap<String, serde_json::Value>,
}
// normalised node, shared by JSON and XML paths
#[derive(Debug, Clone, PartialEq)]
pub enum NodeType { Sequence, SequenceWithMemory, Parallel, Delay, Timeout, SubTree,
                    ExecuteTrajectory, MoveHome, WaitForRobot, CheckSafety, Unsupported(String) }
#[derive(Debug, Clone)]
pub enum PortValue { Json(serde_json::Value), Text(String) }   // Text = XML attribute
#[derive(Debug, Clone)]
pub struct NormNode {
    pub ty: NodeType, pub raw_type: String, pub legacy: bool,
    pub name: Option<String>, pub id: Option<String>,
    pub ports: BTreeMap<String, PortValue>,
    pub children: Vec<NormNode>,
    pub path: BtPath,        // "/Sequence[0]/Parallel[3]/ExecuteTrajectory[1]"
    pub dfs: usize,          // pre-order index (determinism key)
    pub line: Option<u32>,   // XML line
}
pub struct InputDoc {
    pub format: SourceFormat,          // EnvelopeJson | TreeJson | Xml
    pub file_name: String, pub fingerprint: u64,
    pub tree_id: String, pub cell_id: String, pub created_s: Option<f64>,
    pub num_operations: Option<u32>,
    pub root: NormNode, pub blackboard: Blackboard, pub xml: Option<String>,
}
pub fn load(path: &Path, opts: &LoadOptions, d: &mut Diagnostics) -> Result<InputDoc, Error>;
pub struct LoadOptions { pub source: SourceSel /*Auto|Json|Xml*/, pub blackboard: Option<PathBuf> }
// input/blackboard.rs
pub struct Blackboard(pub BTreeMap<String, serde_json::Value>);
impl Blackboard {
    pub fn resolve<'a>(&'a self, v: &'a PortValue) -> Result<Resolved<'a>, Error>; // {key} → entry
    pub fn trajectory(&self, v: &PortValue, path: &BtPath) -> Result<Option<Trajectory>, Error>;
}
impl NormNode {
    pub fn port_str(&self, n: &str) -> Option<String>;
    pub fn port_f64(&self, n: &str) -> Result<Option<f64>, Error>;   // "0.5" | 0.5
    pub fn port_bool(&self, n: &str) -> Result<Option<bool>, Error>; // "true" | true
    pub fn port_list<T: DeserializeOwned>(&self, n: &str, bb: &Blackboard) -> Result<Vec<T>, Error>;
        // JSON list | JSON-in-XML-attr string | "{key}" | "" → []
}
#[derive(Deserialize)] pub struct OpRef { pub robot: String, pub operation_id: String }
```
Legacy dialect map: `SEQUENCE→Sequence`, `PARALLEL→Parallel`, `EXECUTE→ExecuteTrajectory`, `HOME→MoveHome`, `INTERLOCK_WAIT→WaitForRobot`, `SAFETY_GATE→CheckSafety`. Ports are read from `ports` first, then from top-level fields. `precomputed_trajectory` is an alias of `trajectory`. A legacy `start_delay_s>0` becomes a synthetic `Delay` parent (as in btcpp `to_btcpp`). Using the legacy dialect raises W014.

XML path: `bt_core::Document::parse(xml)` then `instantiate(doc.main_tree_id())`. Skip `nodes[0]` (synthetic Root). A `SubTree` node is transparent. Port value = `Port.value` (attribute); fallback defaults come from our own vocabulary table (btcpp `LEAF_PORTS`), not from TreeNodesModel. Inside a subtree, `{k}` resolves by walking up the SubTree ancestors (port `k` literal → value; pointer → continue; `_autoremap` → same key upward). At the top it resolves against the envelope or `--blackboard`; anything unresolved is E013. Any `_skipIf/_while/_successIf/_failureIf/_onSuccess…` is E002.

### 2.2 IR
```rust
pub type OpId = usize; pub type RobotIdx = usize; pub type ZoneIdx = usize;
pub struct CellPlan {
    pub tree_id: String, pub cell_id: String, pub source: SourceInfo,
    pub fingerprint: u64, pub plan_id: u16,          // fold64→16, 0 mapped to 1
    pub robots: Vec<Robot>,                          // BT order, then config idle robots
    pub ops: Vec<Op>,                                // sorted by linear-extension position
    pub cross_edges: Vec<Edge>,                      // reduced; sorted (pos(to), pos(from))
    pub zones: Vec<Zone>,                            // renumbered by pos(first member)
    pub schedule: Schedule,
    pub grant_policy: GrantPolicy,
    pub safety_check: bool,                          // a CheckSafety was present
    pub diagnostics: Vec<Diag>,
}
pub struct Robot {
    pub index: RobotIdx, pub ns: String, pub tag: String,      // "R1"
    pub ops: Vec<OpId>,                                        // program order (1-based index = pos+1)
    pub home_pose_rad: [f64; 6], pub home_source: HomeSource,
    pub addr: RobotAddressing,      // station, device_name, ip, joint_sign, joint_offset_deg, prod_folge
    pub idle: bool,                 // declared in config, no ops
}
pub enum HomeSource { Config, LastWaypointOfFinalMoveHome(String), FirstWaypoint(String), Fallback }
pub struct Op {
    pub id: OpId, pub robot: RobotIdx, pub index: u16,          // 1-based in robot program
    pub operation_id: String,                                    // "W01", "W01@0-2", "robot1_HOME", "HOME"
    pub kind: OpKind,
    pub bt: BtRef,                                               // node id, name, path, dfs
    pub pos: usize,                                              // global linear-extension position
    pub trajectory: Option<Trajectory>,                          // None = blind MoveHome
    pub hold_s: f64, pub velocity_scale: f64,
    pub start_delay_s: f64, pub delay_exact: bool,
    pub timeout_override_s: Option<f64>,
    pub zones: Vec<ZoneIdx>, pub zone_mask: u32,
    pub expected_s: f64, pub limit_s: f64, pub zone_deadline_s: f64,
}
pub enum OpKind { Execute { operation_type: String }, MoveHome }
pub struct Trajectory { pub key: Option<String>, pub waypoints: Vec<Waypoint>, pub original_len: usize }
pub struct Waypoint { pub t_s: f64, pub q_rad: [f64; 6], pub phase: Phase }
pub enum Phase { Transit, Approach, Other(String) }
pub struct Edge { pub from: OpId, pub to: OpId }                 // cross-robot, from <HB to
pub struct Zone { pub index: ZoneIdx, pub name: String /*"IL01"*/, pub tickets: Vec<Ticket> }
pub struct Ticket { pub op: OpId, pub robot: RobotIdx, pub op_index: u16 }
pub struct Schedule { pub start_s: Vec<f64>, pub end_s: Vec<f64>, pub depth: Vec<u32>, pub makespan_s: f64 }

// internal (ir::lower / ir::hb)
pub struct EventGraph { pub ev: Vec<Event>, pub succ: Vec<Vec<usize>>, pub pred: Vec<Vec<usize>> }
pub enum EventKind { Op(OpDraft), Gate { robot: String, refs: Vec<OpRef> }, Barrier, Delay { ms: u64 } }
pub struct Event { pub kind: EventKind, pub key: usize /*dfs of creating node*/, pub path: BtPath }
pub struct Hb { pub n_ops: usize, pub anc: Vec<BitSet>, pub order: Vec<OpId>, pub pos: Vec<usize> }
impl Hb { pub fn before(&self, a: OpId, b: OpId) -> bool { self.anc[b].contains(a) }
          pub fn concurrent(&self, a: OpId, b: OpId) -> bool { a != b && !self.before(a,b) && !self.before(b,a) } }
```

### 2.3 Config (TOML, every field has a default, `deny_unknown_fields`)
```rust
#[derive(Debug, Clone, Serialize, Deserialize)] #[serde(default, deny_unknown_fields)]
pub struct Config { pub cell: CellCfg, pub protocol: ProtocolCfg, pub timing: TimingCfg,
    pub network: NetworkCfg, pub codesys: CodesysCfg, pub robot_sim: RobotSimCfg,
    #[serde(rename = "robot")] pub robots: Vec<RobotCfg> }
pub struct CellCfg {
    pub prod_folge: u16,            // 11
    pub home_folge: u16,            // 125
    pub job_number: JobNumber,      // WordLe (bytes 2-3 LE) | Byte (byte 2, byte 3 = 0)
    pub grant_policy: GrantPolicy,  // Planned | Fcfs
    pub auto_run: bool,             // true
    pub home_strategy: HomeStrategy,// Sequential | Parallel
    pub prune_transitive: bool,     // true (no effect with 2 robots)
}
pub struct ProtocolCfg {
    pub step_byte: u8,              // 4   robot→PLC, WORD LE (wCurrentStep)
    pub request_byte: u8,           // 28  robot→PLC, DWORD LE, bit z-1 = zone z
    pub grant_byte: u8,             // 12  PLC→robot, DWORD LE
    pub plan_id_byte: u8,           // 4   PLC→robot, WORD LE (param bytes 4-5)
    pub max_zones: u8,              // 32
    pub reset_pulse_ms: u32,        // 1000 (robot needs >= 500)
    pub start_pulse_ms: u32,        // 500
    pub stop_pulse_ms: u32,         // 1000
    pub handshake_timeout_ms: u32,  // 5000
    pub mismatch_debounce_ms: u32,  // 200
    pub stop_debounce_ms: u32,      // 200 (JOB_RUNNING: ProgRunning + ProgFinished low -> stopped)
    pub cycle_pause_ms: u32,        // 2000
    pub start_limit_ms: u32,        // 15000
    pub abort_limit_ms: u32,        // 10000
}
pub struct TimingCfg {
    pub watchdog_factor: f64,       // 1.5
    pub watchdog_margin_s: f64,     // 5.0
    pub zone_wait_margin_s: f64,    // 10.0
    pub home_return_time_s: f64,    // 7.0 (blind MoveHome and HOME Folge)
    pub completion_margin_s: f64,   // 0.0 (reference executor uses 1.0; added to expected)
}
pub struct NetworkCfg {
    pub head_ip: Ipv4Addr,          // 192.168.100.10
    pub netmask: Ipv4Addr,          // 255.255.255.0
    pub gateway: Ipv4Addr,          // 0.0.0.0
    pub nic: String,                // "eth1"
    pub controller_station: String, // "controller"
    pub ip_pool_first: Ipv4Addr, pub ip_pool_last: Ipv4Addr,   // .2 / .254
    pub robot_host_base: u8,        // 10 → robot i at .(10+i)
    pub head_gateway_port: u16,     // 11740
    pub head_opcua_port: u16,       // 4840
    pub device_gateway_port: u16,   // 11741
    pub docker_network: String,     // "profinet_net"
}
pub struct CodesysCfg {
    pub ide_version: String,        // "CODESYS V3.5 SP22 Patch 1"
    pub head_project: String,       // "HeadPLC_{cell}"
    pub device_project: String,     // "RobotSim_{cell}_{tag}"
    pub main_task_ms: u32,          // 4
    pub device_main_task_ms: u32,   // 20 (as in KUKA_VASS_Robot_Sim.export)
    pub emit_build_script: bool,    // true
    pub gsdml_file: String,         // "GSDML-V2.25-KUKA-Roboter-GmbH-VKR C4-Device-V1.0-20121030.xml"
}
pub struct RobotSimCfg {
    pub enabled: bool,              // true
    pub decimate_tol_deg: f64,      // 0.0 = off
    pub max_waypoints_per_robot: u32, // 4000 (hard cap 32000: INT index)
    pub at_home_tol_deg: f64,       // 0.5
    pub start_tol_deg: f64,         // 1.0
    pub chain_tol_deg: f64,         // 0.5 (W009)
}
pub struct RobotCfg { pub ns: String, pub tag: Option<String>, pub station: Option<String>,
    pub device_name: Option<String>, pub ip: Option<Ipv4Addr>, pub home_pose_deg: Option<[f64;6]>,
    pub joint_sign: Option<[i8;6]>, pub joint_offset_deg: Option<[f64;6]>,
    pub prod_folge: Option<u16>, pub include_if_idle: bool }
#[serde(rename_all="snake_case")] pub enum JobNumber { WordLe, Byte }
#[serde(rename_all="snake_case")] pub enum GrantPolicy { Planned, Fcfs }
#[serde(rename_all="snake_case")] pub enum HomeStrategy { Sequential, Parallel }
impl Config { pub fn validate(&self) -> Result<(), Error>; }
```
Validation checks:
- Folge range: ≤255 in Byte mode.
- `prod != home`.
- No byte-field overlaps. Robot→PLC: {0-3 status/echo, 4-5 step, 10-11 error, 16-27 joints, request 4 bytes}. PLC→robot: {0-3, plan id 2 bytes, 8-9 speed, grant 4 bytes}. Everything must stay within 0..31.
- `max_zones ≤ 32`.
- Unique tag/station/IP/device names; IPs in the head subnet; no robot IP equal to `head_ip`.

Defaults per robot i (1-based, BT order): `tag=R<i>`, `station=kuka-robi<i>`, `device_name=kuka_robi<i>`, `ip=192.168.100.(10+i)`.

Home pose resolution chain:
1. config
2. last waypoint of the robot's final MoveHome trajectory
3. first waypoint of its first trajectory
4. `(0,-1.57,2.2,0,-0.8,0)` rad, with W015

### 2.4 Backend trait, Flavour, Output
```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum, Serialize, Deserialize)]
pub enum Flavour { Codesys, Twincat3, TiaScl, PlcopenXml, RockwellL5x }
pub struct Capabilities { pub head: bool, pub robot_sim: bool, pub manifest: bool }
pub trait Backend {
    fn flavour(&self) -> Flavour;
    fn capabilities(&self) -> Capabilities;
    fn generate(&self, plan: &CellPlan, cfg: &Config, out: &mut Output, d: &mut Diagnostics)
        -> Result<(), Error>;
}
pub fn backend_for(f: Flavour) -> Box<dyn Backend>;     // stubs → Err(Error::NotImplemented(f))
pub fn common_outputs(plan: &CellPlan, cfg: &Config, out: &mut Output) -> Result<(), Error>; // flavour-independent
pub struct Output { files: BTreeMap<PathBuf, Vec<u8>> }  // relative paths, '/' separators, LF
impl Output {
    pub fn add_text(&mut self, rel: impl Into<PathBuf>, s: String) -> Result<(), Error>; // dup → error
    pub fn paths(&self) -> impl Iterator<Item=&Path>;
    pub fn write_to(&self, dir: &Path, force: bool) -> Result<(), Error>;
      // non-empty dir without --force → E(io). With --force: overwrite, then delete files listed in
      // the previous `.bt2plc-manifest` that are no longer produced; rewrite the manifest.
}
#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("{0}")] Plan(Diag),                                  // exit 1
    #[error("config: {0}")] Config(String),                      // exit 3
    #[error("{path}: {source}")] Io { path: PathBuf, source: std::io::Error }, // exit 4
    #[error("json: {0}")] Json(#[from] serde_json::Error),       // exit 1
    #[error("xml: {0}")] Xml(String),                            // exit 1
    #[error("flavour {0:?} is not implemented yet")] NotImplemented(Flavour), // exit 5
    #[error("{0} warning(s) promoted to errors by --strict")] Strict(usize),  // exit 6
}
```

---

## 3. Algorithms

### 3.1 Lowering (BT → EventGraph): first/last frontiers
An empty frontier means the node is transparent.
```
lower(n, ctx) -> Frontier{first, last}
  ExecuteTrajectory | MoveHome:
      e = op_event(n, timeout=ctx.timeout); return ({e},{e})
  WaitForRobot:
      warn W001/W002/W004 for depends_on_joints/positions/robots (ignored; --strict → error)
      refs = port_list::<OpRef>("depends_on_ops"); if refs.empty: return ∅
      g = gate(n, refs); pending.push(g); return ({g},{g})
  CheckSafety:  plan.safety_check = true; warn W005; return ∅
  Sequence | SequenceWithMemory:
      acc = ∅
      for c in children: f = lower(c, ctx); if f.empty: continue
          if acc.empty: acc = f else { connect(acc.last, f.first); acc.last = f.last }
      return acc
  Parallel:
      sc = success_count ?? success_threshold ?? -1 ; fc = failure_count ?? failure_threshold ?? 1
      require (sc == -1 || sc == #children) && fc == 1   else E003
      acc = ∅; for c: f = lower(c); acc.first ∪= f.first; acc.last ∪= f.last; return acc
  Delay:   require 1 child else E015; d = delay_event(delay_msec)
           f = lower(child); if f.empty return ({d},{d}); edges d→f.first; return ({d}, f.last)
  Timeout: require 1 child; ctx' = ctx{timeout = min(ctx.timeout, msec)}; f = lower(child, ctx')
           if ops_in(child) > 1: warn W006; return f
  SubTree: return lower(only child)
  else:    E002 "unsupported node <type> at <path>; supported: …"
connect(L, F): if |L| == 1 or |F| == 1: add L×F    // ≤ max(|L|,|F|) edges
               else b = barrier(); add L→b, b→F
After the walk:
  index = {(robot_ns, operation_id) → op} over ExecuteTrajectory (duplicate → E007)
  MoveHome ids: "HOME", then "HOME_2", … (suffix on collision)
  for g in pending, r in g.refs: u = index[r] else E006; add u→g
```
ExecuteTrajectory leaves are validated:
- `robot_ns` and `operation_id` are required.
- The trajectory must resolve (E008).
- Trajectory content (E010): ≥2 waypoints, exactly 6 joints, finite values, `time_s` non-decreasing; a non-zero start time is shifted to 0 with W020.
- Joint range: `|deg|` < 655.34 (E017, INT16 encoding).
- `has_interlock`/`interlock_target` → W003.
- `velocity_scale ≠ 1` → I008 (supported by playback scaling).

MoveHome without a trajectory raises W010 (blind home, `home_return_time_s`).

Complexity is O(N + R_refs): every node is visited once, and barriers keep the edge count linear.

### 3.2 HB closure (bitsets), cycle detection, linear extension
```
topo = Kahn(EventGraph) with min-heap on (event.key, event.id)       // deterministic
if |topo| < |events|: find a cycle among the remaining events (DFS with stack) → E005
     message lists the cycle as BT paths, e.g. gate(robot1@/…/WaitForRobot[0]) ← robot2:W13 ← gate(robot2@…) ← robot1:W02
for e in topo:                                   // anc over ops only
     anc[e] = ⋃_{p∈pred(e)} ( anc[p] ∪ ({p} if p is op) )
order = [e for e in topo if e is op];  pos[op] = index in order
start_delay: for e in topo: dmax[e] = max_{p∈pred(e), p not op} dmax[p] (+ms if Delay), likewise dmin
     op o: start_delay = max_{non-op p ∈ pred(o)} dmax[p]; exact = (dmax == dmin) else W007
```
Complexity: O((V+E)·n_ops/64) for the closure and O((V+E) log V) for the extension. For SP trees the extension equals BT DFS order. For coordinated trees it is the "earliest DFS-ready" order.

### 3.3 Per-robot total order
```
for robot R: chain_R = ops of R sorted by pos
   for i: require hb.before(chain_R[i], chain_R[i+1]) else E004
          "robot R runs a and b concurrently (paths …): a robot cannot be in two Parallel branches"
```
Consecutive comparability plus transitivity gives a total order. Complexity O(n_ops).

### 3.4 Cross-edge reduction
Since anc(v) is downward closed, `anc(v) ∩ chain_Q` is a prefix of chain_Q, and it is monotone along chain_R.
```
for each ordered pair (R, Q), R ≠ Q:
   p = 0                                           // length of prefix of chain_Q ⊆ anc(v)
   prev = None
   for v in chain_R:
       while p < |chain_Q| and hb.before(chain_Q[p], v): p += 1
       cur = (p > 0) ? chain_Q[p-1] : None         // pred_Q(v)
       if cur != None and cur != prev: E1.push(cur → v)
       prev = cur
if prune_transitive:                                // only matters for ≥3 robots
   drop (u→v) if ∃ S ∉ {robot(u), robot(v)}: s = pred_S(v) exists and hb.before(u, s)
sort E by (pos(v), pos(u))
```
Complexity: O(R²·n_ops) for the reduction, plus O(R³·n_ops) bit tests for pruning.

**Lemma R (soundness and completeness).** closure(PO ∪ E) restricted to ops equals HB.
- ⊆: every edge is in HB.
- ⊇: induct on pos(v). Take u <HB v with robot(u)=Q ≠ R=robot(v). If u <HB prev_R(v), induction gives u ⇝ prev_R(v), then PO to v. Otherwise u ≤_Q u* := pred_Q(v) and pred_Q(prev_R(v)) <_Q u ≤ u*, so (u*→v) ∈ E1.
- Pruning: if (u*→v) was pruned via S, then u* <HB s = pred_S(v). Induction gives u ⇝ s (pos(s) < pos(v)), and the same argument applies to (s, v). The successive preds p1 <HB p2 <HB … all equal pred_X(v) for pairwise distinct robots X (a repeated robot would force equality). The chain therefore ends after ≤ R steps at a kept edge or at the prev-case.
- For two robots E1 is already a transitive reduction.

### 3.5 Zones: greedy chain cover
```
zones = []                                         // each: members[], created (int)
for (u→v) in E (sorted by pos(v), pos(u)):
   C1 = [z : last(z) == u]
   C  = C1 nonempty ? C1 : [z : hb.before(last(z), u)]
   rank C by: (a) robots(z) ⊆ {robot(u), robot(v)} first, (b) smaller z.created first
   if C nonempty: z = C[0]; if last(z) != u: z.push(u); z.push(v)
   else zones.push(new zone [u, v], created = zones.len())
renumber zones stably by pos(first member); name "IL%02d"; bit = index
if |zones| > max_zones: E009
mask[op] |= 1 << z.index for each member; tickets = members in list order
```
Complexity: O(|E|·Z) with O(1) HB tests, Z ≤ 32. An exact minimum alternative exists (Dilworth on the edge poset "e1 ≺ e2 ⇔ v1 ≤HB u2" via bipartite matching, O(E^2.5)) but is not default. It is listed in §9.

Invariant: every zone is a strict HB chain in list order (pos strictly increasing).
- Appending v after last==u: u <HB v.
- Appending u, v after last <HB u: last <HB u <HB v.
- New zone: u <HB v.
- No duplicates: u already in z implies u <HB last(z), which contradicts last ≤HB u. A v cannot repeat because all members have pos ≤ pos(v) and v is appended once per zone.

### 3.6 Proofs
Assumptions: robots execute their generated programs in order, op durations are finite, and every op whose mask ≠ 0 requests its zones (§4).

**(a) Every reduced edge is covered with the correct ticket order.** Each (u→v) is processed once and leaves u immediately before v in some zone. Later appends only extend the tail, so adjacency is preserved. Under planned grants v's ticket in z becomes current only after u's ticket is released. Release happens after u completes (the robot drops REQUEST after the hold). Hence start(v) ≥ end(u). With Lemma R and PO (robot program order) every HB pair is enforced.

**(b) Zones never serialize BT-concurrent ops.** The start of op x waits only for:
- (i) prev_R(x) to complete (HB-before x);
- (ii) for each zone z ∋ x, the release of every earlier ticket in z. These are HB-before x because z is a chain;
- (iii) the start delay.

Release of w depends only on w's own predecessors, all of which are HB-before x by transitivity. So the enabling condition of x depends only on completions in anc(x). A concurrent y ∉ anc(x) never delays x, and y and x are never in one zone (zone members are pairwise comparable). Atomic grants add no waiting under planned: while x is the current ticket of z, nobody else can take z.

**(d) BT schedule equivalence.** With zero communication latency and any durations d, start_zone(x) = max_{p∈P(x)} end(p), where P(x) = {prev_R(x)} ∪ ticket-preds(x) ⊆ anc(x). For any a ∈ anc(x) there is a path a ⇝ p ∈ P(x) with end(a) ≤ end(p) (by induction). So start_zone(x) = max_{a∈anc(x)} end(a) (+delay) = ASAP_HB(x). That equals what umarell `bt_executor.py` produces: Sequence join = all of the previous step; WaitForRobot = listed completions. The protocol reproduces the BT timing exactly, up to handshake latency ≤ depth(x)·(scan + IO latency) and conservative delays (W007). For the welding sample this is a makespan of 64.30 s at expected durations.

**(c) Deadlock freedom (planned, all-or-nothing).** Suppose the system is stuck with incomplete ops. Let x be the incomplete op with minimal pos.
1. All of anc(x) is complete (anc positions are smaller).
2. prev_R(x) is complete, so robot R is at x and requests mask(x) with wStep = index(x).
3. For every z ∋ x, all earlier tickets are HB-before x, hence complete and released (a release needs only "REQUEST low while running OK", which the robot does unconditionally after the hold).
4. Therefore x is the current ticket of every z ∋ x, all those zones are free, and request == mask and step == ticket hold. The arbiter grants atomically and x completes, a contradiction.

Each op acquires once, so there is no livelock. The same argument holds with non-atomic grants under planned, because ticket order is a global linear extension.

### 3.7 What FCFS loses (precisely)
In a zones-only design every cross-robot HB pair is enforced only through ticket order. FCFS drops ticket order:
- It enforces **no cross-robot ordering at all**. The only guarantee left is mutual exclusion among members of the same zone, which the BT already guaranteed (members are HB-ordered).
- Example: r1.W02 may run before r2.W13. The planner validated trajectories (collision-free, chained, paired) for the BT order, so that validation no longer applies.
- `bAllConsumed`, the ticket-skip and own-ticket checks are disabled.

What FCFS keeps:
- Deadlock freedom: atomic grants mean no hold-and-wait, and holds are finite.
- Starvation freedom: the oldest request is served first.
- BT parallelism: concurrent ops are never in one zone.

FCFS is meaningful only when zones are physically symmetric regions. The zones here are derived from ordering, not geometry, so W011 says so.

### 3.8 Schedule, watchdogs, deadlines
- `expected(x) = start_delay + traj_end/velocity_scale + hold + completion_margin`. MoveHome uses hold 0; blind home uses `home_return_time_s`.
- `limit(x) = min(f·expected + m, timeout_override)`.
- `ASAP` runs over the EventGraph (op = expected, Delay = ms, gates/barriers = 0). `depth(x)` = longest op-count chain before x.
- `zone_deadline(x) = f·ASAP_start(x) + m·depth(x) + m_zone`, measured since all robots started.

Proof of no false positives: if every op takes ≤ f·d + m (otherwise its watchdog fires first), then start'(x) ≤ max over chains Σ(f·d_i + m) ≤ f·ASAP_start(x) + m·depth(x). m_zone absorbs start skew and handshake latency.

### 3.9 Hand-worked welding sample (`bt_1790934549_welding_cell_2_small.json`)
Steps: P1{r1.W01,r2.W13} P2{W02,W15} P3{W03,W14} P4{W06,W10} P5{W05,W11} P6{W04,W12} r1.R1_PARK r2.W09 r2.W08 r2.W07 P7{r1.HOME,r2.HOME}.

Linear extension (pos): 1 W01, 2 W13, 3 W02, 4 W15, 5 W03, 6 W14, 7 W06, 8 W10, 9 W05, 10 W11, 11 W04, 12 W12, 13 R1_PARK, 14 W09, 15 W08, 16 W07, 17 r1.HOME, 18 r2.HOME.

| v (robot1) | pred_r2(v) | pred of prev | kept |
|---|---|---|---|
| W01 | – | – | |
| W02 | W13 | – | ✓ |
| W03 | W15 | W13 | ✓ |
| W06 | W14 | W15 | ✓ |
| W05 | W10 | W14 | ✓ |
| W04 | W11 | W10 | ✓ |
| R1_PARK | W12 | W11 | ✓ |
| HOME | W07 | W12 | ✓ |

| v (robot2) | pred_r1(v) | pred of prev | kept |
|---|---|---|---|
| W13 | – | | |
| W15 | W01 | – | ✓ |
| W14 | W02 | W01 | ✓ |
| W10 | W03 | | ✓ |
| W11 | W06 | | ✓ |
| W12 | W05 | | ✓ |
| W09 | R1_PARK | W05 | ✓ |
| W08 | R1_PARK | R1_PARK | ✗ |
| W07 | R1_PARK | R1_PARK | ✗ |
| HOME | R1_PARK | R1_PARK | ✗ |

The table gives 13 reduced edges. Greedy processing, sorted by pos(v):

| # | edge | action |
|---|---|---|
| 1 | W13→W02 | new A=[W13,W02] |
| 2 | W01→W15 | A.last=W02 not ≤HB W01 → new B=[W01,W15] |
| 3 | W15→W03 | B.last==u → B+=W03 |
| 4 | W02→W14 | A+=W14 |
| 5 | W14→W06 | A+=W06 |
| 6 | W03→W10 | B+=W10 |
| 7 | W10→W05 | B+=W05 |
| 8 | W06→W11 | A+=W11 |
| 9 | W11→W04 | A+=W04 |
| 10 | W05→W12 | B+=W12 |
| 11 | W12→R1_PARK | B+=R1_PARK |
| 12 | R1_PARK→W09 | B+=W09 |
| 13 | W07→r1.HOME | no last==W07; both lasts (W04, W09) ≤HB W07; same pair; oldest = A → A+=[W07, HOME] |

Renumbering by first-member position (W01 pos 1, W13 pos 2):
- **IL01 = r1.W01 > r2.W15 > r1.W03 > r2.W10 > r1.W05 > r2.W12 > r1.R1_PARK > r2.W09**
- **IL02 = r2.W13 > r1.W02 > r2.W14 > r1.W06 > r2.W11 > r1.W04 > r2.W07 > r1.HOME**

Robot programs:
- robot1: W01[IL01] W02[IL02] W03[IL01] W06[IL02] W05[IL01] W04[IL02] R1_PARK[IL01] HOME[IL02]
- robot2: W13[IL02] W15[IL01] W14[IL02] W10[IL01] W11[IL02] W12[IL01] W09[IL01] W08 W07[IL02] HOME

Check of (d): W02 and W15 both start at max(end W01, end W13) = 11.132 s, as the BT Parallel step does. The `bt_1791274820_cell_2robot` sample (robot1 only: STAGE_R1, HOME) gives 0 edges, 0 zones and 1 robot.

---

## 4. Wire protocol

Two profiles (`protocol.profile`): `sim` (sections 4.1-4.3 below) and `vw-ks24` (section 4.4).

### 4.1 Byte map (additions in bold; everything else as deployed)
PLC→robot `aVassOutputData`:
- [0].0 DrivesOn (level), .1 ProgStart (500 ms pulse), .2 Stop, .3 Cancel, .4 EXT, .5 Reset (held ≥ T_RESET_PULSE=1 s), .6 HomeRequest (unused by the head), .7 MoveEnable (TRUE except RS_OFF; a KUKA maps $MOVE_ENABLE to it)
- [1].0 JobRequest
- bytes 2-3: job number (WORD LE, or byte 2 only with byte 3 = 0 in `byte` mode)
- **bytes 4-5: PLAN_ID (WORD LE), written only during a production job**
- 8-9: speed
- **bytes 12-15: GRANT DWORD LE (byte 12 = zones 1-8, bit (z-1) mod 8 of byte 12 + (z-1) div 8)**, non-zero only while the job is active

Robot→PLC `aVassInputData`:
- standard bits as deployed
- 2-3: echo (or byte 2 only)
- **4-5: wCurrentStep = op index (1-based), 0 = no program**
- 10-11: error code
- 16-27: joints
- **28-31: REQUEST DWORD LE, same bit layout**

Safe channel unchanged: head writes `bySafeOut0=16#01`, `bySafeOut1=16#0F`; reads In0.0 and In1.4.

### 4.2 Zone handshake (robot side, per op k with mask M = mask(k))
```
1  wStep := k                       (also for M = 0 — feeds the PLC watchdog)
2  if M ≠ 0: REQUEST := M           (same telegram as wStep; KRL: write step first)
3  if M ≠ 0: wait (GRANT AND M) = M
4  wait start_delay(k); play trajectory (×velocity_scale×override); hold
5  if M ≠ 0: REQUEST := 0
6  if M ≠ 0: wait (GRANT AND M) = 0  (4-phase: mandatory before the next REQUEST)
7  k := k+1; after the last op: ProgFinished
```
PLC (FB_ZoneArbiter) runs this every scan:
1. **Release**: for each held zone whose holder's REQUEST bit is low, if the holder is running OK, clear the holder and its GRANT bit and (planned) advance the ticket. Otherwise raise `ERR_RELEASE_INVALID`.
2. **Grant**: a robot is pending if it is OK, has REQUEST ≠ 0 and GRANT = 0. Its request must equal `mask(wStep)`; a mismatch persisting for 200 ms is `ERR_REQ_MISMATCH`. Under planned, each zone in the mask must be free and its current ticket must be (robot, wStep). If so, grant every zone in one scan (all-or-nothing). An own ticket for another op, or a consumed zone, is `ERR_TICKET_SKIPPED`. A holder raising extra bits is `ERR_UNEXPECTED_REQUEST`.
3. Any fault drops all grants and latches until reset.

Supervision (FB_Cell):
- The step must advance exactly +1 (`ERR_STEP_SEQUENCE`).
- The per-op watchdog starts at the step change (M=0) or at the grant (M≠0); limit `tLimit` (`ERR_OP_WATCHDOG`).
- Zone wait past `tZoneDeadline` since cycle start: `ERR_ZONE_WAIT_TIMEOUT`.
- A zone op with no REQUEST and no grant for 5 s: `ERR_ROBOT_STALLED`.
- At ProgFinished, max step ≠ op count: `ERR_PROGRAM_INCOMPLETE`.
- At cycle end, tickets not consumed: `ERR_TICKETS_LEFT`.

**Reset/abort semantics.** Any fault (robot error, protocol, watchdog, operator stop) triggers this sequence:
1. Cell goes to ABORTING: all GRANT bits drop, tickets rewind, every busy robot gets Stop+Cancel held 1 s.
2. Cell goes to ERROR.
3. On an operator reset edge, robots in error get Reset held 1 s and are released; the robot must report Error=0 within 5 s.
4. Bring-up runs again, then blind homing (Folge 125, sequential by default) for robots not AtHome.
5. The cell is READY and a new cycle starts from ticket 1 / op 1. Resuming mid-cycle is not supported.

Robot side on Stop/Cancel, drives-off, E-stop or error: abort the program, REQUEST := 0, wStep := 0, hold the pose. If the GRANT drops while executing, the sim ignores it (the Stop follows within one cycle). A future KRL may treat a GRANT drop as a stop request (§9).

### 4.3 Job handshake (production Folge and HOME Folge, FB_VassRobot)
| State | PLC writes | Advance when | Timeout (5 s) |
|---|---|---|---|
| SAFETY_ENABLE | SafeOut0=01, SafeOut1=0F | SafeIn0.0 ∧ SafeIn1.4 | ERR_SAFETY_TIMEOUT |
| DRIVES_ON | [0].0 | DrivesPowered | ERR_DRIVES_TIMEOUT |
| MODE_EXT | [0].0,[0].4 | ModeExt → IDLE | ERR_EXT_TIMEOUT |
| JOB_WAIT_READY | JobReq=0 | Ready ∧ ¬JobAck | ERR_NOT_READY |
| JOB_SEND | job nr (+PLAN_ID if production), JobReq=1 | JobAck: **echo == job**, else ERR_ECHO_MISMATCH | ERR_ACK_TIMEOUT |
| JOB_START | JobReq=0, ProgStart=1 for 500 ms | pulse elapsed | – |
| JOB_WAIT_RUNNING | ProgStart=0 | ProgRunning → RUNNING; ProgFinished → DONE (F5) | ERR_START_TIMEOUT |
| JOB_RUNNING | grants | ProgFinished ∧ ¬ProgRunning → DONE; ¬ProgRunning ∧ ¬ProgFinished for T_STOP_DEBOUNCE (200 ms) → ERR_ROBOT_STOPPED (a one-frame gap at the Folge end is no stop) | none here: per-op watchdogs in FB_Cell; HOME uses T_HOME_LIMIT |
| STOPPING | Stop=Cancel=1 for ≥ 1 s | ¬ProgRunning → IDLE | ERR_STOP_TIMEOUT |
| RESET | Reset=1 for 1 s (≥ 500 ms) | released ∧ ¬Error → SAFETY_ENABLE, else ERR_RESET_FAILED | – |

Global checks: robot error in IDLE/job → ERR_ROBOT_ERROR; safety lost → ERR_SAFETY_LOST; drives lost → ERR_DRIVES_LOST. The HOME Folge (125) is the same handshake without PLAN_ID and without zones. The device plays a joint move to home over 7 s.

### 4.4 Profile `vw-ks24` (real VW KS24 robots)

The VW tech package on the robot owns the Folge handshake; the head speaks the KS24 signal map
(`vw_usr_s.dat`, verified on the 8 exported robots). Signal indices are `GVL_Cell.VW_*`.

| What | Signal |
|---|---|
| Folge number / echo | `PRG_NR` `$IN[1..8]` (byte 0) / `PRG_NR_REFL` `$OUT[1..8]` |
| Start | `SRB` `$IN[9]`, held as a level until the robot leaves its home point |
| Folge end | `$OUT[14]` (`O_R_F_ENDE`); the head then clears `PRG_NR` and waits for it to fall |
| Home | `$OUT[15]` (`O_R_PFO`) |
| Ready | `$OUT[9]` (`O_R_Bereit`, drops while a Folge runs) |
| Drives / safety circuit | `$IN[10]` (`I_R_Ant_Ein`) / `$OUT[10]` (`O_R_RK100`) + `$OUT[13]` (`O_R_Notaus`) |
| Auto mode | `$OUT[12]` (`O_R_Auto`) |
| Op index | `P_PktNr_SPS` `$OUT[97..112]` |
| Error code | `VW_FEH_BIN_A` `$OUT[225..232]` |
| Acknowledge | `$IN[26]` (`I_R_Quit_St`) |
| Interlock n (1..16) | request `$OUT[80+n]`, grant `$IN[80+n]`, release `$OUT[40+n]`, partner release `$IN[40+n]` |

**Zone mapping.** Zone z of robot r uses interlock `GVL_Cell.anInterlock[(r-1)*N_ZONES+z]`
(`[[robot]] interlocks`, default: the zones a robot uses numbered 1, 2, …; also written to
`program.json` so bt2krl uses the same map). FB_VassRobot synthesises the arbiter's REQUEST bit
as `A(80+n) OR NOT A(40+n)`: high while the robot requests the interlock **and** while it is
inside it ("occupied" = `A(40+n)` FALSE), and low again when the robot sets `A(40+n)`. The
arbiter, the ticket order, the mismatch checks and the mutual-exclusion argument are therefore
unchanged. FB_Cell writes, per robot and interlock: the grant `E(80+n)` from the arbiter, and the
partner release `E(40+n)` as the AND of the other members' `A(40+n)`.

**Differences that are inherent to the VW scheme:**
- No 4-phase handshake: the robot does not wait for the grant to drop. It only holds its release
  level long enough for the head to see it (bt2krl emits a dwell after every release, and the
  level must survive at least one PROFINET cycle plus one head scan). `ERR_HANDSHAKE` is
  therefore debounced with `T_MISMATCH_DEBOUNCE` in this profile.
- No Stop/Cancel bits: an abort drops the drives `E10`, clears `PRG_NR` and withdraws the partner
  releases, which stops the robot on its path through its own online driving condition
  (`$CYCFLAG[254]`, `VW(#FB_STOP)`).
- No blind HOME Folge: VW `cell.src` refuses to start a Folge unless `$IN_HOME` is set, so
  CS_HOMING_SELECT only waits for every robot to report PF0 and sets `fbCell.bHomingRequired`;
  the operator jogs the robots home. `ERR_HOME_FAILED` after `T_HOME_LIMIT` names the robot.
- No drives feedback: KS24 has no "drives on" status bit. The head uses the safety-circuit group
  `A10` plus auto mode `A12`, so a drive fault is seen only through those.
- No PLAN_ID signal: optional on a free telegram byte (`protocol.plan_id_byte`, 0 = no check).
  Valid start bytes (two bytes unused on all 8 sample robots): 12, 13, 28, 29. The sim default 4 is rejected: bytes 4-5 carry the partner releases E41-48.
- The real VW PLC blocks (`FB_207_Rob_Frg_Ver`, `FB_209_Rob_Ver_ext`) arbitrate the requests
  **first-come-first-served** (whoever asks while the partner has not reserved). That keeps mutual
  exclusion but not the behaviour tree's order, which is why this profile replaces them rather
  than running next to them.

---

## 5. Generated CODESYS objects

### 5.1 Head PLC (`codesys/head/`), in creation order
| # | Object | Kind | Origin | Files |
|---|---|---|---|---|
| 1 | E_GrantPolicy | DUT enum | template | DUT/E_GrantPolicy.st |
| 2 | E_CellError | DUT enum | template | DUT/E_CellError.st |
| 3 | E_RobotState | DUT enum | template | DUT/E_RobotState.st |
| 4 | E_CellState | DUT enum | template | DUT/E_CellState.st |
| 5 | ST_Ticket | DUT struct | template | DUT/ST_Ticket.st |
| 6 | ST_OpInfo | DUT struct | template | DUT/ST_OpInfo.st |
| 7 | ST_RobotIn | DUT struct | template | DUT/ST_RobotIn.st |
| 8 | ST_RobotOut | DUT struct | template | DUT/ST_RobotOut.st |
| 9 | GVL_Cell | GVL CONSTANT | **generated** | GVL/GVL_Cell.st |
| 10 | GVL_IO | GVL | **generated** | GVL/GVL_IO.st |
| 11 | FB_VassRobot | POU FB | template | POU/FB_VassRobot.st + POU/FB_VassRobot_Body.st |
| 12 | FB_ZoneArbiter | POU FB | template | POU/FB_ZoneArbiter.st + _Body.st |
| 13 | FB_Cell | POU FB | template | POU/FB_Cell.st + _Body.st |
| 14 | PLC_PRG | POU PRG | **generated** (replaces default) | POU/PLC_PRG.st + _Body.st |

DUT sketches:
```
TYPE E_GrantPolicy : ( GP_PLANNED := 0, GP_FCFS := 1 ); END_TYPE
TYPE E_RobotState : ( RS_OFF := 0, RS_SAFETY_ENABLE := 1, RS_DRIVES_ON := 2, RS_MODE_EXT := 3,
    RS_IDLE := 10, RS_JOB_WAIT_READY := 20, RS_JOB_SEND := 21, RS_JOB_START := 22,
    RS_JOB_WAIT_RUNNING := 23, RS_JOB_RUNNING := 24, RS_JOB_DONE := 25,
    RS_STOPPING := 30, RS_RESET := 40, RS_ERROR := 99 ); END_TYPE
TYPE E_CellState : ( CS_INIT := 0, CS_BRINGUP := 10, CS_HOMING_SELECT := 20, CS_HOMING_RUN := 21,
    CS_READY := 30, CS_START_JOBS := 40, CS_RUNNING := 50, CS_CYCLE_DONE := 60,
    CS_ABORTING := 80, CS_RESETTING := 90, CS_ERROR := 99 ); END_TYPE
TYPE E_CellError : ( ERR_NONE := 0,
    ERR_SAFETY_TIMEOUT := 10, ERR_DRIVES_TIMEOUT := 11, ERR_EXT_TIMEOUT := 12, ERR_NOT_READY := 13,
    ERR_ACK_TIMEOUT := 14, ERR_ECHO_MISMATCH := 15, ERR_START_TIMEOUT := 16, ERR_ROBOT_STOPPED := 17,
    ERR_ROBOT_ERROR := 18, ERR_SAFETY_LOST := 19, ERR_STOP_TIMEOUT := 20, ERR_RESET_FAILED := 21,
    ERR_DRIVES_LOST := 22,
    ERR_OP_WATCHDOG := 30, ERR_ZONE_WAIT_TIMEOUT := 31, ERR_STEP_SEQUENCE := 32, ERR_REQ_MISMATCH := 33,
    ERR_TICKET_SKIPPED := 34, ERR_PROGRAM_INCOMPLETE := 35, ERR_RELEASE_INVALID := 36,
    ERR_ROBOT_STALLED := 37, ERR_UNEXPECTED_REQUEST := 38, ERR_TICKETS_LEFT := 39,
    ERR_HOME_FAILED := 40, ERR_OPERATOR_STOP := 50 ); END_TYPE
TYPE ST_Ticket : STRUCT nRobot : INT; nOp : INT; END_STRUCT END_TYPE        (* 1-based; 0 = none *)
TYPE ST_OpInfo : STRUCT nRobot : INT; nOp : INT; dwZoneMask : DWORD; tExpected : TIME;
    tLimit : TIME; tZoneDeadline : TIME; sName : STRING(32); END_STRUCT END_TYPE
TYPE ST_RobotIn  : STRUCT abySafeIn  : ARRAY[0..7] OF BYTE; aVassIn  : ARRAY[0..31] OF BYTE; END_STRUCT END_TYPE
TYPE ST_RobotOut : STRUCT abySafeOut : ARRAY[0..7] OF BYTE; aVassOut : ARRAY[0..31] OF BYTE; END_STRUCT END_TYPE
```
(Real files use one member per line, `(* *)` comments and banner headers.)

**GVL_IO (generated)**: `{attribute 'qualified_only'} VAR_GLOBAL`. Per robot there is a banner `R1 - robot1 - kuka_robi1 (kuka-robi1, 192.168.100.11)`, followed by `bySafeIn0_R1..bySafeIn7_R1 : BYTE;`, `bySafeOut0_R1..7`, `aVassInputData_R1 : ARRAY[0..31] OF BYTE;` and `aVassOutputData_R1`.

**GVL_Cell (generated), welding sample**:
```
(*
 * GVL_Cell - generated by bt2plc 0.1.0 - DO NOT EDIT
 * Plan   : bt_1790934549_welding_cell_2_small (cell welding_cell_2_small)
 * Zones  : IL01 = R1#1 W01 > R2#2 W15 > R1#3 W03 > R2#4 W10 > R1#5 W05 > R2#6 W12 > R1#7 R1_PARK > R2#7 W09
 *          IL02 = R2#1 W13 > R1#2 W02 > R2#3 W14 > R1#4 W06 > R2#5 W11 > R1#6 W04 > R2#9 W07 > R1#8 HOME
 *)
{attribute 'qualified_only'}
VAR_GLOBAL CONSTANT
    (* ========================================================================
       DIMENSIONS (array bounds use the *_DIM values, which are >= 1)
       ======================================================================== *)
    N_ROBOTS            : INT := 2;
    N_ZONES             : INT := 2;
    N_ZONES_DIM         : INT := 2;
    N_TICKETS_DIM       : INT := 16;
    N_OPS_DIM           : INT := 18;

    (* ========================================================================
       PROTOCOL / TIMING
       ======================================================================== *)
    PLAN_ID             : WORD := 16#5A3C;      (* example value: fold of plan fingerprint *)
    PROD_FOLGE          : WORD := 11;
    HOME_FOLGE          : WORD := 125;
    JOB_NUMBER_BYTE     : BOOL := FALSE;        (* FALSE: WORD LE bytes 2-3, TRUE: byte 2 *)
    GRANT_POLICY        : E_GrantPolicy := E_GrantPolicy.GP_PLANNED;
    HOME_SEQUENTIAL     : BOOL := TRUE;
    STEP_BYTE           : INT := 4;             (* robot -> PLC *)
    REQ_BYTE            : INT := 28;            (* robot -> PLC *)
    GRANT_BYTE          : INT := 12;            (* PLC -> robot *)
    PLAN_ID_BYTE        : INT := 4;             (* PLC -> robot *)
    T_RESET_PULSE       : TIME := T#1S;
    T_START_PULSE       : TIME := T#500MS;
    T_STOP_PULSE        : TIME := T#1S;
    T_HANDSHAKE         : TIME := T#5S;
    T_MISMATCH_DEBOUNCE : TIME := T#200MS;
    T_STOP_DEBOUNCE     : TIME := T#200MS;
    T_CYCLE_PAUSE       : TIME := T#2S;
    T_START_LIMIT       : TIME := T#15S;
    T_ABORT_LIMIT       : TIME := T#10S;
    T_HOME_LIMIT        : TIME := T#15S500MS;   (* 7 s x 1.5 + 5 s *)

    (* ========================================================================
       ROBOTS (index = BT order)
       ======================================================================== *)
    asRobotTag          : ARRAY[1..2] OF STRING(16) := ['R1', 'R2'];
    asRobotNs           : ARRAY[1..2] OF STRING(32) := ['robot1', 'robot2'];
    anRobotFirstOp      : ARRAY[1..2] OF INT := [1, 9];
    anRobotOpCount      : ARRAY[1..2] OF INT := [8, 10];

    (* ========================================================================
       ZONES - tickets of zone z: aTickets[anZoneFirst[z] .. anZoneFirst[z] + anZoneCount[z] - 1]
       ======================================================================== *)
    asZoneName          : ARRAY[1..2] OF STRING(8) := ['IL01', 'IL02'];
    anZoneFirst         : ARRAY[1..2] OF INT := [1, 9];
    anZoneCount         : ARRAY[1..2] OF INT := [8, 8];
    aTickets            : ARRAY[1..16] OF ST_Ticket := [
        (* IL01 *)
        (nRobot := 1, nOp := 1),    (* R1 #1 W01 *)
        (nRobot := 2, nOp := 2),    (* R2 #2 W15 *)
        (nRobot := 1, nOp := 3),    (* R1 #3 W03 *)
        (nRobot := 2, nOp := 4),    (* R2 #4 W10 *)
        (nRobot := 1, nOp := 5),    (* R1 #5 W05 *)
        (nRobot := 2, nOp := 6),    (* R2 #6 W12 *)
        (nRobot := 1, nOp := 7),    (* R1 #7 R1_PARK *)
        (nRobot := 2, nOp := 7),    (* R2 #7 W09 *)
        (* IL02 *)
        (nRobot := 2, nOp := 1),    (* R2 #1 W13 *)
        (nRobot := 1, nOp := 2),    (* R1 #2 W02 *)
        (nRobot := 2, nOp := 3),    (* R2 #3 W14 *)
        (nRobot := 1, nOp := 4),    (* R1 #4 W06 *)
        (nRobot := 2, nOp := 5),    (* R2 #5 W11 *)
        (nRobot := 1, nOp := 6),    (* R1 #6 W04 *)
        (nRobot := 2, nOp := 9),    (* R2 #9 W07 *)
        (nRobot := 1, nOp := 8)     (* R1 #8 HOME *)
    ];

    (* ========================================================================
       OPS - op k of robot r: aOps[anRobotFirstOp[r] + k - 1]
       tLimit = 1.5 x tExpected + 5 s ; tZoneDeadline = 1.5 x ASAP start + 5 s x depth + 10 s
       ======================================================================== *)
    aOps                : ARRAY[1..18] OF ST_OpInfo := [
        (nRobot := 1, nOp := 1,  dwZoneMask := 16#00000001, tExpected := T#11S132MS, tLimit := T#21S698MS, tZoneDeadline := T#10S,         sName := 'W01'),
        (nRobot := 1, nOp := 2,  dwZoneMask := 16#00000002, tExpected := T#4S321MS,  tLimit := T#11S482MS, tZoneDeadline := T#31S698MS,    sName := 'W02'),
        (nRobot := 1, nOp := 3,  dwZoneMask := 16#00000001, tExpected := T#5S950MS,  tLimit := T#13S926MS, tZoneDeadline := T#49S336MS,    sName := 'W03'),
        (nRobot := 1, nOp := 4,  dwZoneMask := 16#00000002, tExpected := T#2S672MS,  tLimit := T#9S8MS,    tZoneDeadline := T#1M3S261MS,   sName := 'W06'),
        (nRobot := 1, nOp := 5,  dwZoneMask := 16#00000001, tExpected := T#5S912MS,  tLimit := T#13S868MS, tZoneDeadline := T#1M15S749MS,  sName := 'W05'),
        (nRobot := 1, nOp := 6,  dwZoneMask := 16#00000002, tExpected := T#4S550MS,  tLimit := T#11S825MS, tZoneDeadline := T#1M29S618MS,  sName := 'W04'),
        (nRobot := 1, nOp := 7,  dwZoneMask := 16#00000001, tExpected := T#2S671MS,  tLimit := T#9S7MS,    tZoneDeadline := T#1M41S850MS,  sName := 'R1_PARK'),
        (nRobot := 1, nOp := 8,  dwZoneMask := 16#00000002, tExpected := T#8S330MS,  tLimit := T#17S496MS, tZoneDeadline := T#2M23S950MS,  sName := 'HOME'),
        (nRobot := 2, nOp := 1,  dwZoneMask := 16#00000002, tExpected := T#9S686MS,  tLimit := T#19S529MS, tZoneDeadline := T#10S,         sName := 'W13'),
        (nRobot := 2, nOp := 2,  dwZoneMask := 16#00000001, tExpected := T#8S425MS,  tLimit := T#17S638MS, tZoneDeadline := T#31S698MS,    sName := 'W15'),
        (nRobot := 2, nOp := 3,  dwZoneMask := 16#00000002, tExpected := T#4S724MS,  tLimit := T#12S86MS,  tZoneDeadline := T#49S336MS,    sName := 'W14'),
        (nRobot := 2, nOp := 4,  dwZoneMask := 16#00000001, tExpected := T#4S992MS,  tLimit := T#12S488MS, tZoneDeadline := T#1M3S261MS,   sName := 'W10'),
        (nRobot := 2, nOp := 5,  dwZoneMask := 16#00000002, tExpected := T#5S36MS,   tLimit := T#12S554MS, tZoneDeadline := T#1M15S749MS,  sName := 'W11'),
        (nRobot := 2, nOp := 6,  dwZoneMask := 16#00000001, tExpected := T#4S821MS,  tLimit := T#12S232MS, tZoneDeadline := T#1M29S618MS,  sName := 'W12'),
        (nRobot := 2, nOp := 7,  dwZoneMask := 16#00000001, tExpected := T#2S699MS,  tLimit := T#9S48MS,   tZoneDeadline := T#1M50S857MS,  sName := 'W09'),
        (nRobot := 2, nOp := 8,  dwZoneMask := 16#00000000, tExpected := T#4S594MS,  tLimit := T#11S891MS, tZoneDeadline := T#1M59S905MS,  sName := 'W08'),
        (nRobot := 2, nOp := 9,  dwZoneMask := 16#00000002, tExpected := T#4S769MS,  tLimit := T#12S154MS, tZoneDeadline := T#2M11S796MS,  sName := 'W07'),
        (nRobot := 2, nOp := 10, dwZoneMask := 16#00000000, tExpected := T#6S229MS,  tLimit := T#14S344MS, tZoneDeadline := T#2M23S950MS,  sName := 'HOME')
    ];
END_VAR
```
Formatting rules:
- TIME literals: ms = round-half-away(s·1000), emitted as `T#[nH][nM][nS][nMS]`; zero is `T#0MS`.
- Strings are escaped (`$'`, `$$`), ASCII only, truncated to 32 characters for ST (the full id stays in JSON).
- Comments are sanitised (`*)` becomes `* )`).
- With 0 zones: `N_ZONES := 0; N_ZONES_DIM := 1; anZoneFirst := [1]; anZoneCount := [0]; aTickets : ARRAY[1..1] OF ST_Ticket := [(nRobot := 0, nOp := 0)]`.

#### FB_VassRobot
```
(*
 * FB_VassRobot
 *
 * VW VASS protocol driver for ONE robot (KUKA VKRC nexxt V8.2 DIM 2 via PROFINET).
 * Portable: no pointers, no REFERENCE, no OOP - robot IO is passed as array
 * copies (stIn / stOut) by the caller.
 *
 *   Bring-up : SAFETY_ENABLE -> DRIVES_ON -> MODE_EXT -> IDLE
 *   Job      : WAIT_READY -> SEND (echo checked) -> START (500 ms) -> WAIT_RUNNING -> RUNNING -> DONE
 *   Abort    : STOPPING (Stop + Cancel held T_STOP_PULSE)
 *   Recovery : ERROR -> RESET (Reset held T_RESET_PULSE >= 500 ms) -> SAFETY_ENABLE
 *   Command handshake: bCmdJob level request, bJobDone acknowledge (cleared while bCmdJob = FALSE)
 *)

FUNCTION_BLOCK FB_VassRobot
VAR_INPUT
    bEnable             : BOOL := TRUE;
    bPowerOn            : BOOL;             (* Level: bring robot up *)
    bCmdJob             : BOOL;             (* Rising edge (latched): run job wJobNr *)
    wJobNr              : WORD;             (* Folge number *)
    bCmdStop            : BOOL;             (* Level: abort active job *)
    bCmdReset           : BOOL;             (* Rising edge in RS_ERROR: reset robot *)
    dwZoneGrant         : DWORD;            (* From FB_ZoneArbiter *)
    stIn                : ST_RobotIn;       (* Copy of this robot's GVL_IO inputs *)
END_VAR

VAR_OUTPUT
    stOut               : ST_RobotOut;      (* Copy to this robot's GVL_IO outputs *)
    eState              : E_RobotState;
    eError              : E_CellError;
    bIdle               : BOOL;
    bBusy               : BOOL;             (* Job, stopping or resetting *)
    bJobActive          : BOOL;             (* Job started and ProgRunning *)
    bJobDone            : BOOL;
    bError              : BOOL;
    (* Decoded robot status *)
    bDrivesOn           : BOOL;
    bProgRunning        : BOOL;
    bAtHome             : BOOL;
    bModeExt            : BOOL;
    bRobotReady         : BOOL;
    bRobotError         : BOOL;
    bProgFinished       : BOOL;
    bJobAck             : BOOL;
    bSafetyOK           : BOOL;
    wActiveJob          : WORD;
    wStep               : WORD;
    dwZoneReq           : DWORD;
    wRobotErrorCode     : WORD;
END_VAR

VAR
    _eState             : E_RobotState := E_RobotState.RS_OFF;
    _eLastState         : E_RobotState := E_RobotState.RS_OFF;
    _wJob               : WORD;
    _bJobPending        : BOOL;
    _bInJob             : BOOL;
    _bUp                : BOOL;
    _bSafeEnable        : BOOL;
    _bDrivesOnCmd       : BOOL;
    _bExtCmd            : BOOL;
    _bStartCmd          : BOOL;
    _bStopCmd           : BOOL;
    _bCancelCmd         : BOOL;
    _bResetCmd          : BOOL;
    _bJobReqCmd         : BOOL;
    _dwGrantOut         : DWORD;
    _tonStep            : TON;              (* Step timeout, restarts on state change *)
    _tonPulse           : TON;              (* Start / stop / reset pulse width *)
    _rtrigJob           : R_TRIG;
    _rtrigReset         : R_TRIG;
    _i                  : INT;
END_VAR
```
```
(*
 * FB_VassRobot - Implementation
 *)

IF NOT bEnable THEN
    FOR _i := 0 TO 31 DO
        stOut.aVassOut[_i] := 0;
    END_FOR
    FOR _i := 0 TO 7 DO
        stOut.abySafeOut[_i] := 0;
    END_FOR
    _eState := E_RobotState.RS_OFF;
    eState := _eState;
    bIdle := FALSE;
    bBusy := FALSE;
    bJobActive := FALSE;
    RETURN;
END_IF

(* ============================================================================
   DECODE ROBOT STATUS (robot -> PLC)
   ============================================================================ *)
bDrivesOn     := stIn.aVassIn[0].0;
bProgRunning  := stIn.aVassIn[0].1;
bAtHome       := stIn.aVassIn[0].2;
bModeExt      := stIn.aVassIn[0].4;
bRobotReady   := stIn.aVassIn[0].5;
bRobotError   := stIn.aVassIn[0].6;
bProgFinished := stIn.aVassIn[1].2;
bJobAck       := stIn.aVassIn[1].3;

IF GVL_Cell.JOB_NUMBER_BYTE THEN
    wActiveJob := BYTE_TO_WORD(stIn.aVassIn[2]);
ELSE
    wActiveJob := BYTE_TO_WORD(stIn.aVassIn[2]) OR SHL(BYTE_TO_WORD(stIn.aVassIn[3]), 8);
END_IF

wStep := BYTE_TO_WORD(stIn.aVassIn[GVL_Cell.STEP_BYTE])
         OR SHL(BYTE_TO_WORD(stIn.aVassIn[GVL_Cell.STEP_BYTE + 1]), 8);

dwZoneReq := BYTE_TO_DWORD(stIn.aVassIn[GVL_Cell.REQ_BYTE])
             OR SHL(BYTE_TO_DWORD(stIn.aVassIn[GVL_Cell.REQ_BYTE + 1]), 8)
             OR SHL(BYTE_TO_DWORD(stIn.aVassIn[GVL_Cell.REQ_BYTE + 2]), 16)
             OR SHL(BYTE_TO_DWORD(stIn.aVassIn[GVL_Cell.REQ_BYTE + 3]), 24);

wRobotErrorCode := BYTE_TO_WORD(stIn.aVassIn[10]) OR SHL(BYTE_TO_WORD(stIn.aVassIn[11]), 8);
bSafetyOK := stIn.abySafeIn[0].0 AND stIn.abySafeIn[1].4;

(* ============================================================================
   EDGES / STEP TIMER
   ============================================================================ *)
_rtrigJob(CLK := bCmdJob);
_rtrigReset(CLK := bCmdReset);
IF _rtrigJob.Q THEN
    _bJobPending := TRUE;
END_IF
IF NOT bCmdJob THEN
    _bJobPending := FALSE;
END_IF

IF _eState <> _eLastState THEN
    _tonStep(IN := FALSE);
    _tonPulse(IN := FALSE);
    _eLastState := _eState;
END_IF
_tonStep(IN := TRUE, PT := GVL_Cell.T_HANDSHAKE);

(* ============================================================================
   GLOBAL TRANSITIONS
   ============================================================================ *)
_bInJob := (_eState = E_RobotState.RS_JOB_WAIT_READY) OR (_eState = E_RobotState.RS_JOB_SEND)
        OR (_eState = E_RobotState.RS_JOB_START) OR (_eState = E_RobotState.RS_JOB_WAIT_RUNNING)
        OR (_eState = E_RobotState.RS_JOB_RUNNING);
_bUp := _bInJob OR (_eState = E_RobotState.RS_IDLE);

IF _bInJob AND bCmdStop THEN
    _eState := E_RobotState.RS_STOPPING;
ELSIF _bUp AND bRobotError THEN
    eError := E_CellError.ERR_ROBOT_ERROR;
    _eState := E_RobotState.RS_ERROR;
ELSIF _bUp AND NOT bSafetyOK THEN
    eError := E_CellError.ERR_SAFETY_LOST;
    _eState := E_RobotState.RS_ERROR;
ELSIF _bUp AND NOT bDrivesOn THEN
    eError := E_CellError.ERR_DRIVES_LOST;
    _eState := E_RobotState.RS_ERROR;
END_IF

(* ============================================================================
   STATE MACHINE
   ============================================================================ *)
CASE _eState OF

E_RobotState.RS_OFF:
    _bSafeEnable := FALSE;
    _bDrivesOnCmd := FALSE;
    _bExtCmd := FALSE;
    _bJobReqCmd := FALSE;
    _bStartCmd := FALSE;
    _bStopCmd := FALSE;
    _bCancelCmd := FALSE;
    _bResetCmd := FALSE;
    IF bPowerOn THEN
        _eState := E_RobotState.RS_SAFETY_ENABLE;
    END_IF

E_RobotState.RS_SAFETY_ENABLE:
    _bSafeEnable := TRUE;
    IF bSafetyOK THEN
        _eState := E_RobotState.RS_DRIVES_ON;
    ELSIF _tonStep.Q THEN
        eError := E_CellError.ERR_SAFETY_TIMEOUT;
        _eState := E_RobotState.RS_ERROR;
    END_IF

E_RobotState.RS_DRIVES_ON:
    _bDrivesOnCmd := TRUE;
    IF bDrivesOn THEN
        _eState := E_RobotState.RS_MODE_EXT;
    ELSIF _tonStep.Q THEN
        eError := E_CellError.ERR_DRIVES_TIMEOUT;
        _eState := E_RobotState.RS_ERROR;
    END_IF

E_RobotState.RS_MODE_EXT:
    _bExtCmd := TRUE;
    IF bModeExt THEN
        _eState := E_RobotState.RS_IDLE;
    ELSIF _tonStep.Q THEN
        eError := E_CellError.ERR_EXT_TIMEOUT;
        _eState := E_RobotState.RS_ERROR;
    END_IF

E_RobotState.RS_IDLE:
    _bJobReqCmd := FALSE;
    _bStartCmd := FALSE;
    _bStopCmd := FALSE;
    _bCancelCmd := FALSE;
    IF NOT bCmdJob THEN
        bJobDone := FALSE;
    END_IF
    IF NOT bPowerOn THEN
        _eState := E_RobotState.RS_OFF;
    ELSIF _bJobPending THEN
        _bJobPending := FALSE;
        _wJob := wJobNr;
        bJobDone := FALSE;
        _eState := E_RobotState.RS_JOB_WAIT_READY;
    END_IF

E_RobotState.RS_JOB_WAIT_READY:
    _bJobReqCmd := FALSE;                   (* guarantees a rising edge *)
    IF bRobotReady AND NOT bJobAck THEN
        _eState := E_RobotState.RS_JOB_SEND;
    ELSIF _tonStep.Q THEN
        eError := E_CellError.ERR_NOT_READY;
        _eState := E_RobotState.RS_ERROR;
    END_IF

E_RobotState.RS_JOB_SEND:
    _bJobReqCmd := TRUE;
    IF bJobAck THEN
        _bJobReqCmd := FALSE;
        IF wActiveJob = _wJob THEN
            _eState := E_RobotState.RS_JOB_START;
        ELSE
            eError := E_CellError.ERR_ECHO_MISMATCH;
            _eState := E_RobotState.RS_ERROR;
        END_IF
    ELSIF _tonStep.Q THEN
        _bJobReqCmd := FALSE;
        eError := E_CellError.ERR_ACK_TIMEOUT;
        _eState := E_RobotState.RS_ERROR;
    END_IF

E_RobotState.RS_JOB_START:
    _bJobReqCmd := FALSE;
    _bStartCmd := TRUE;
    _tonPulse(IN := TRUE, PT := GVL_Cell.T_START_PULSE);
    IF _tonPulse.Q THEN
        _bStartCmd := FALSE;
        _eState := E_RobotState.RS_JOB_WAIT_RUNNING;
    END_IF

E_RobotState.RS_JOB_WAIT_RUNNING:
    _bStartCmd := FALSE;
    IF bProgRunning THEN
        _eState := E_RobotState.RS_JOB_RUNNING;
    ELSIF bProgFinished THEN
        (* finished between two scans - ProgFinished is cleared on job accept *)
        _eState := E_RobotState.RS_JOB_DONE;
    ELSIF _tonStep.Q THEN
        eError := E_CellError.ERR_START_TIMEOUT;
        _eState := E_RobotState.RS_ERROR;
    END_IF

E_RobotState.RS_JOB_RUNNING:
    (* a robot may publish ProgRunning low one bus frame before ProgFinished: debounce *)
    _tonStopDebounce(IN := NOT bProgRunning AND NOT bProgFinished, PT := GVL_Cell.T_STOP_DEBOUNCE);
    IF bProgFinished AND NOT bProgRunning THEN
        _eState := E_RobotState.RS_JOB_DONE;
    ELSIF _tonStopDebounce.Q THEN
        eError := E_CellError.ERR_ROBOT_STOPPED;
        _eState := E_RobotState.RS_ERROR;
    END_IF

E_RobotState.RS_JOB_DONE:
    bJobDone := TRUE;
    _eState := E_RobotState.RS_IDLE;

E_RobotState.RS_STOPPING:
    _bJobReqCmd := FALSE;
    _bStartCmd := FALSE;
    _bStopCmd := TRUE;
    _bCancelCmd := TRUE;
    _tonPulse(IN := TRUE, PT := GVL_Cell.T_STOP_PULSE);
    IF _tonPulse.Q AND NOT bProgRunning THEN
        _bStopCmd := FALSE;
        _bCancelCmd := FALSE;
        _eState := E_RobotState.RS_IDLE;
    ELSIF _tonStep.Q THEN
        _bStopCmd := FALSE;
        _bCancelCmd := FALSE;
        eError := E_CellError.ERR_STOP_TIMEOUT;
        _eState := E_RobotState.RS_ERROR;
    END_IF

E_RobotState.RS_ERROR:
    _bJobReqCmd := FALSE;
    _bStartCmd := FALSE;
    _bStopCmd := FALSE;
    _bCancelCmd := FALSE;
    _bResetCmd := FALSE;
    IF _rtrigReset.Q THEN
        _eState := E_RobotState.RS_RESET;
    END_IF

E_RobotState.RS_RESET:
    _bResetCmd := TRUE;                     (* held >= 500 ms - robot requirement *)
    _tonPulse(IN := TRUE, PT := GVL_Cell.T_RESET_PULSE);
    IF _tonPulse.Q THEN
        _bResetCmd := FALSE;
        IF NOT bRobotError THEN
            eError := E_CellError.ERR_NONE;
            _eState := E_RobotState.RS_SAFETY_ENABLE;
        ELSE
            eError := E_CellError.ERR_RESET_FAILED;
            _eState := E_RobotState.RS_ERROR;
        END_IF
    END_IF

END_CASE

(* ============================================================================
   ENCODE COMMANDS (PLC -> robot)
   ============================================================================ *)
_bInJob := (_eState = E_RobotState.RS_JOB_WAIT_READY) OR (_eState = E_RobotState.RS_JOB_SEND)
        OR (_eState = E_RobotState.RS_JOB_START) OR (_eState = E_RobotState.RS_JOB_WAIT_RUNNING)
        OR (_eState = E_RobotState.RS_JOB_RUNNING);
bJobActive := bProgRunning AND ((_eState = E_RobotState.RS_JOB_START)
              OR (_eState = E_RobotState.RS_JOB_WAIT_RUNNING) OR (_eState = E_RobotState.RS_JOB_RUNNING));

stOut.aVassOut[0].0 := _bDrivesOnCmd;
stOut.aVassOut[0].1 := _bStartCmd;
stOut.aVassOut[0].2 := _bStopCmd;
stOut.aVassOut[0].3 := _bCancelCmd;
stOut.aVassOut[0].4 := _bExtCmd;
stOut.aVassOut[0].5 := _bResetCmd;
stOut.aVassOut[0].6 := FALSE;               (* homing uses HOME_FOLGE, not HomeRequest *)
stOut.aVassOut[0].7 := FALSE;
stOut.aVassOut[1].0 := _bJobReqCmd;

IF _bInJob THEN
    stOut.aVassOut[2] := WORD_TO_BYTE(_wJob AND 16#FF);
    IF GVL_Cell.JOB_NUMBER_BYTE THEN
        stOut.aVassOut[3] := 0;
    ELSE
        stOut.aVassOut[3] := WORD_TO_BYTE(SHR(_wJob, 8) AND 16#FF);
    END_IF
ELSE
    stOut.aVassOut[2] := 0;
    stOut.aVassOut[3] := 0;
END_IF

IF _bInJob AND _wJob = GVL_Cell.PROD_FOLGE THEN
    stOut.aVassOut[GVL_Cell.PLAN_ID_BYTE]     := WORD_TO_BYTE(GVL_Cell.PLAN_ID AND 16#FF);
    stOut.aVassOut[GVL_Cell.PLAN_ID_BYTE + 1] := WORD_TO_BYTE(SHR(GVL_Cell.PLAN_ID, 8) AND 16#FF);
ELSE
    stOut.aVassOut[GVL_Cell.PLAN_ID_BYTE]     := 0;
    stOut.aVassOut[GVL_Cell.PLAN_ID_BYTE + 1] := 0;
END_IF

IF bJobActive THEN
    _dwGrantOut := dwZoneGrant;
ELSE
    _dwGrantOut := 0;
END_IF
stOut.aVassOut[GVL_Cell.GRANT_BYTE]     := DWORD_TO_BYTE(_dwGrantOut AND 16#FF);
stOut.aVassOut[GVL_Cell.GRANT_BYTE + 1] := DWORD_TO_BYTE(SHR(_dwGrantOut, 8) AND 16#FF);
stOut.aVassOut[GVL_Cell.GRANT_BYTE + 2] := DWORD_TO_BYTE(SHR(_dwGrantOut, 16) AND 16#FF);
stOut.aVassOut[GVL_Cell.GRANT_BYTE + 3] := DWORD_TO_BYTE(SHR(_dwGrantOut, 24) AND 16#FF);

IF _bSafeEnable THEN
    stOut.abySafeOut[0] := 16#01;           (* drive enable *)
    stOut.abySafeOut[1] := 16#0F;           (* zones 1-4 enable *)
ELSE
    stOut.abySafeOut[0] := 0;
    stOut.abySafeOut[1] := 0;
END_IF

(* ============================================================================
   PUBLISH
   ============================================================================ *)
eState := _eState;
bIdle  := (_eState = E_RobotState.RS_IDLE);
bBusy  := _bInJob OR (_eState = E_RobotState.RS_STOPPING) OR (_eState = E_RobotState.RS_RESET);
bError := (_eState = E_RobotState.RS_ERROR) OR (_eState = E_RobotState.RS_RESET);
```

#### FB_ZoneArbiter
```
(*
 * FB_ZoneArbiter
 *
 * Zone (mutex) arbiter, data-driven by GVL_Cell (aTickets, anZoneFirst/Count, aOps[].dwZoneMask).
 * GP_PLANNED: zone granted only to its next ticket holder, only if the request equals
 *             the op's full mask and the reported op index matches the ticket.
 * GP_FCFS   : free zones to any requester whose request equals the op mask, oldest
 *             request first, ties by robot index. BT ordering NOT preserved.
 * Both: all zones of an op in the same scan (all-or-nothing). Fail closed: any
 * violation latches bFault and drops all grants until bReset.
 *)

FUNCTION_BLOCK FB_ZoneArbiter
VAR_INPUT
    bReset          : BOOL;             (* Level: rewind tickets, clear holders and grants *)
    bEnable         : BOOL;             (* New grants allowed *)
    ePolicy         : E_GrantPolicy := E_GrantPolicy.GP_PLANNED;
    tMismatch       : TIME := T#200MS;
    abRobotOk       : ARRAY[1..GVL_Cell.N_ROBOTS] OF BOOL;
    awStep          : ARRAY[1..GVL_Cell.N_ROBOTS] OF WORD;
    adwReq          : ARRAY[1..GVL_Cell.N_ROBOTS] OF DWORD;
END_VAR

VAR_OUTPUT
    adwGrant        : ARRAY[1..GVL_Cell.N_ROBOTS] OF DWORD;
    anHolder        : ARRAY[1..GVL_Cell.N_ZONES_DIM] OF INT;    (* 0 = free *)
    anTicket        : ARRAY[1..GVL_Cell.N_ZONES_DIM] OF INT;    (* next ticket index in aTickets *)
    bAllConsumed    : BOOL;
    bFault          : BOOL;
    eFault          : E_CellError;
    nFaultRobot     : INT;
    nFaultZone      : INT;
END_VAR

VAR
    _udiScan        : UDINT;
    _audiWaitSince  : ARRAY[1..GVL_Cell.N_ROBOTS] OF UDINT;
    _abPicked       : ARRAY[1..GVL_Cell.N_ROBOTS] OF BOOL;
    _atonMismatch   : ARRAY[1..GVL_Cell.N_ROBOTS] OF TON;
    _eNew           : E_CellError;
    _nNewRobot      : INT;
    _nNewZone       : INT;
    _k              : INT;
    _r              : INT;
    _rr             : INT;
    _rSel           : INT;
    _z              : INT;
    _iOp            : INT;
    _nStep          : INT;
    _nLastTicket    : INT;
    _dwBit          : DWORD;
    _dwExp          : DWORD;
    _bOk            : BOOL;
    _bPending       : BOOL;
END_VAR
```
```
(*
 * FB_ZoneArbiter - Implementation
 *)

IF bReset THEN
    FOR _z := 1 TO GVL_Cell.N_ZONES_DIM DO
        anHolder[_z] := 0;
        anTicket[_z] := GVL_Cell.anZoneFirst[_z];
    END_FOR
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        adwGrant[_r] := 0;
        _audiWaitSince[_r] := 0;
        _atonMismatch[_r](IN := FALSE);
    END_FOR
    bFault := FALSE;
    eFault := E_CellError.ERR_NONE;
    nFaultRobot := 0;
    nFaultZone := 0;
    bAllConsumed := (GVL_Cell.N_ZONES = 0);
    RETURN;
END_IF

IF bFault THEN
    RETURN;                                 (* latched until bReset *)
END_IF

_udiScan := _udiScan + 1;
_eNew := E_CellError.ERR_NONE;

(* ============================================================================
   1. RELEASE - holder dropped its REQUEST bit
   ============================================================================ *)
FOR _z := 1 TO GVL_Cell.N_ZONES DO
    _rr := anHolder[_z];
    IF _rr <> 0 THEN
        _dwBit := SHL(DWORD#1, _z - 1);
        IF (adwReq[_rr] AND _dwBit) = 0 THEN
            IF abRobotOk[_rr] THEN
                anHolder[_z] := 0;
                adwGrant[_rr] := adwGrant[_rr] AND NOT _dwBit;
                IF ePolicy = E_GrantPolicy.GP_PLANNED THEN
                    anTicket[_z] := anTicket[_z] + 1;
                END_IF
            ELSIF _eNew = E_CellError.ERR_NONE THEN
                (* request vanished because the robot stopped / faulted / lost
                   connection: the zone may still be occupied - never re-grant *)
                _eNew := E_CellError.ERR_RELEASE_INVALID;
                _nNewRobot := _rr;
                _nNewZone := _z;
            END_IF
        END_IF
    END_IF
END_FOR

(* ============================================================================
   2. GRANT
   ============================================================================ *)
FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
    _abPicked[_r] := FALSE;
    IF adwReq[_r] = 0 OR adwGrant[_r] <> 0 THEN
        _audiWaitSince[_r] := 0;
    ELSIF _audiWaitSince[_r] = 0 THEN
        _audiWaitSince[_r] := _udiScan;
    END_IF
END_FOR

FOR _k := 1 TO GVL_Cell.N_ROBOTS DO
    (* --- visiting order: PLANNED = robot index, FCFS = oldest request first --- *)
    _rSel := 0;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        IF NOT _abPicked[_r] THEN
            IF _rSel = 0 THEN
                _rSel := _r;
            ELSIF ePolicy = E_GrantPolicy.GP_FCFS AND _audiWaitSince[_r] <> 0
                  AND (_audiWaitSince[_rSel] = 0 OR _audiWaitSince[_r] < _audiWaitSince[_rSel]) THEN
                _rSel := _r;
            END_IF
        END_IF
    END_FOR
    _rr := _rSel;
    _abPicked[_rr] := TRUE;

    (* --- expected mask of the op the robot reports --- *)
    _nStep := WORD_TO_INT(awStep[_rr]);
    _dwExp := 0;
    IF _nStep >= 1 AND _nStep <= GVL_Cell.anRobotOpCount[_rr] THEN
        _iOp := GVL_Cell.anRobotFirstOp[_rr] + _nStep - 1;
        _dwExp := GVL_Cell.aOps[_iOp].dwZoneMask;
    END_IF

    _bPending := bEnable AND abRobotOk[_rr] AND (adwReq[_rr] <> 0) AND (adwGrant[_rr] = 0);

    _atonMismatch[_rr](IN := _bPending AND (adwReq[_rr] <> _dwExp), PT := tMismatch);
    IF _atonMismatch[_rr].Q AND _eNew = E_CellError.ERR_NONE THEN
        _eNew := E_CellError.ERR_REQ_MISMATCH;
        _nNewRobot := _rr;
        _nNewZone := 0;
    END_IF

    IF adwGrant[_rr] <> 0 AND (adwReq[_rr] AND NOT adwGrant[_rr]) <> 0
       AND _eNew = E_CellError.ERR_NONE THEN
        _eNew := E_CellError.ERR_UNEXPECTED_REQUEST;
        _nNewRobot := _rr;
        _nNewZone := 0;
    END_IF

    IF _bPending AND adwReq[_rr] = _dwExp THEN
        _bOk := TRUE;
        FOR _z := 1 TO GVL_Cell.N_ZONES DO
            _dwBit := SHL(DWORD#1, _z - 1);
            IF (_dwExp AND _dwBit) <> 0 THEN
                IF anHolder[_z] <> 0 THEN
                    _bOk := FALSE;
                ELSIF ePolicy = E_GrantPolicy.GP_PLANNED THEN
                    _nLastTicket := GVL_Cell.anZoneFirst[_z] + GVL_Cell.anZoneCount[_z] - 1;
                    IF anTicket[_z] > _nLastTicket THEN
                        _bOk := FALSE;
                        IF _eNew = E_CellError.ERR_NONE THEN
                            _eNew := E_CellError.ERR_TICKET_SKIPPED;
                            _nNewRobot := _rr;
                            _nNewZone := _z;
                        END_IF
                    ELSIF GVL_Cell.aTickets[anTicket[_z]].nRobot <> _rr
                          OR GVL_Cell.aTickets[anTicket[_z]].nOp <> _nStep THEN
                        _bOk := FALSE;              (* not this op's turn yet *)
                        IF GVL_Cell.aTickets[anTicket[_z]].nRobot = _rr
                           AND _eNew = E_CellError.ERR_NONE THEN
                            _eNew := E_CellError.ERR_TICKET_SKIPPED;
                            _nNewRobot := _rr;
                            _nNewZone := _z;
                        END_IF
                    END_IF
                END_IF
            END_IF
        END_FOR

        IF _bOk THEN
            FOR _z := 1 TO GVL_Cell.N_ZONES DO
                IF (_dwExp AND SHL(DWORD#1, _z - 1)) <> 0 THEN
                    anHolder[_z] := _rr;
                END_IF
            END_FOR
            adwGrant[_rr] := _dwExp;
            _audiWaitSince[_rr] := 0;
        END_IF
    END_IF
END_FOR

(* ============================================================================
   3. FAULT LATCH / CYCLE COMPLETENESS
   ============================================================================ *)
IF _eNew <> E_CellError.ERR_NONE THEN
    bFault := TRUE;
    eFault := _eNew;
    nFaultRobot := _nNewRobot;
    nFaultZone := _nNewZone;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        adwGrant[_r] := 0;
    END_FOR
END_IF

bAllConsumed := TRUE;
FOR _z := 1 TO GVL_Cell.N_ZONES DO
    IF anHolder[_z] <> 0 THEN
        bAllConsumed := FALSE;
    ELSIF ePolicy = E_GrantPolicy.GP_PLANNED
          AND anTicket[_z] <= GVL_Cell.anZoneFirst[_z] + GVL_Cell.anZoneCount[_z] - 1 THEN
        bAllConsumed := FALSE;
    END_IF
END_FOR
```

#### FB_Cell
```
(*
 * FB_Cell
 *
 * Cell control for N robots: bring-up, blind homing (HOME_FOLGE), production cycle
 * (PROD_FOLGE started on every robot with ops; robots run autonomously), zone
 * arbitration, supervision (op index sequence, per-op watchdog, zone-wait deadline,
 * stall), abort (stop all, rewind tickets) and reset (Reset held >= 500 ms).
 *)

FUNCTION_BLOCK FB_Cell
VAR_INPUT
    bEnable             : BOOL := TRUE;
    bAutoRun            : BOOL := TRUE;     (* Level: start a cycle whenever READY *)
    bStart              : BOOL;             (* Rising edge: start one cycle *)
    bStop               : BOOL;             (* Rising edge: abort *)
    bReset              : BOOL;             (* Rising edge in CS_ERROR *)
    astIn               : ARRAY[1..GVL_Cell.N_ROBOTS] OF ST_RobotIn;
END_VAR

VAR_OUTPUT
    astOut              : ARRAY[1..GVL_Cell.N_ROBOTS] OF ST_RobotOut;
    eState              : E_CellState;
    eError              : E_CellError;
    nErrorRobot         : INT;
    nErrorOp            : INT;
    nErrorZone          : INT;
    bCycleActive        : BOOL;
    bError              : BOOL;
    udiCycles           : UDINT;
    tLastCycle          : TIME;
    awStep              : ARRAY[1..GVL_Cell.N_ROBOTS] OF WORD;
    adwGrant            : ARRAY[1..GVL_Cell.N_ROBOTS] OF DWORD;
END_VAR

VAR
    afbRobot            : ARRAY[1..GVL_Cell.N_ROBOTS] OF FB_VassRobot;
    fbArbiter           : FB_ZoneArbiter;
    _eState             : E_CellState := E_CellState.CS_INIT;
    _eLastState         : E_CellState := E_CellState.CS_INIT;
    _bFault             : BOOL;
    _bPowerOn           : BOOL;
    _bArbReset          : BOOL := TRUE;
    _abJobCmd           : ARRAY[1..GVL_Cell.N_ROBOTS] OF BOOL;
    _awJob              : ARRAY[1..GVL_Cell.N_ROBOTS] OF WORD;
    _abStopCmd          : ARRAY[1..GVL_Cell.N_ROBOTS] OF BOOL;
    _abResetCmd         : ARRAY[1..GVL_Cell.N_ROBOTS] OF BOOL;
    _abHoming           : ARRAY[1..GVL_Cell.N_ROBOTS] OF BOOL;
    _abRobotOk          : ARRAY[1..GVL_Cell.N_ROBOTS] OF BOOL;
    _awStepIn           : ARRAY[1..GVL_Cell.N_ROBOTS] OF WORD;
    _adwReqIn           : ARRAY[1..GVL_Cell.N_ROBOTS] OF DWORD;
    _awLastStep         : ARRAY[1..GVL_Cell.N_ROBOTS] OF WORD;
    _awMaxStep          : ARRAY[1..GVL_Cell.N_ROBOTS] OF WORD;
    _abGranted          : ARRAY[1..GVL_Cell.N_ROBOTS] OF BOOL;
    _atonOp             : ARRAY[1..GVL_Cell.N_ROBOTS] OF TON;
    _atonStall          : ARRAY[1..GVL_Cell.N_ROBOTS] OF TON;
    _tonState           : TON;
    _tonCycle           : TON;
    _tLimit             : TIME;
    _rtrigStart         : R_TRIG;
    _rtrigStop          : R_TRIG;
    _rtrigReset         : R_TRIG;
    _eNew               : E_CellError;
    _nNewRobot          : INT;
    _nNewOp             : INT;
    _nHomeCount         : INT;
    _r                  : INT;
    _iOp                : INT;
    _nStep              : INT;
    _bAll               : BOOL;
END_VAR
```
```
(*
 * FB_Cell - Implementation
 *)

IF NOT bEnable THEN
    _eState := E_CellState.CS_INIT;
END_IF

(* ============================================================================
   EDGES / STATE TIMER
   ============================================================================ *)
_rtrigStart(CLK := bStart);
_rtrigStop(CLK := bStop);
_rtrigReset(CLK := bReset);

IF _eState <> _eLastState THEN
    _tonState(IN := FALSE);
    _eLastState := _eState;
END_IF
CASE _eState OF
E_CellState.CS_HOMING_RUN:
    _tLimit := GVL_Cell.T_HOME_LIMIT;
E_CellState.CS_START_JOBS:
    _tLimit := GVL_Cell.T_START_LIMIT;
E_CellState.CS_CYCLE_DONE:
    _tLimit := GVL_Cell.T_CYCLE_PAUSE;
E_CellState.CS_ABORTING, E_CellState.CS_RESETTING:
    _tLimit := GVL_Cell.T_ABORT_LIMIT;
ELSE
    _tLimit := T#24H;
END_CASE
_tonState(IN := TRUE, PT := _tLimit);

(* ============================================================================
   ROBOTS (commands from the previous scan)
   ============================================================================ *)
FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
    afbRobot[_r](
        bEnable     := bEnable,
        bPowerOn    := _bPowerOn,
        bCmdJob     := _abJobCmd[_r],
        wJobNr      := _awJob[_r],
        bCmdStop    := _abStopCmd[_r],
        bCmdReset   := _abResetCmd[_r],
        dwZoneGrant := fbArbiter.adwGrant[_r],
        stIn        := astIn[_r]);
    astOut[_r] := afbRobot[_r].stOut;
END_FOR

(* ============================================================================
   ZONE ARBITER
   ============================================================================ *)
FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
    _abRobotOk[_r] := afbRobot[_r].bJobActive AND (_awJob[_r] = GVL_Cell.PROD_FOLGE)
                      AND NOT afbRobot[_r].bRobotError;
    _awStepIn[_r] := afbRobot[_r].wStep;
    _adwReqIn[_r] := afbRobot[_r].dwZoneReq;
END_FOR
fbArbiter(
    bReset    := _bArbReset,
    bEnable   := (_eState = E_CellState.CS_RUNNING),
    ePolicy   := GVL_Cell.GRANT_POLICY,
    tMismatch := GVL_Cell.T_MISMATCH_DEBOUNCE,
    abRobotOk := _abRobotOk,
    awStep    := _awStepIn,
    adwReq    := _adwReqIn);
_bArbReset := FALSE;

(* ============================================================================
   SUPERVISION (production cycle)
   ============================================================================ *)
_eNew := E_CellError.ERR_NONE;
IF _eState = E_CellState.CS_RUNNING THEN
    _tonCycle(IN := TRUE, PT := T#24H);
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        IF GVL_Cell.anRobotOpCount[_r] > 0 AND afbRobot[_r].bJobActive THEN
            _nStep := WORD_TO_INT(afbRobot[_r].wStep);

            (* --- op index advances by exactly +1 --- *)
            IF afbRobot[_r].wStep <> _awLastStep[_r] THEN
                IF (_nStep <> WORD_TO_INT(_awLastStep[_r]) + 1)
                   OR (_nStep > GVL_Cell.anRobotOpCount[_r]) THEN
                    IF _eNew = E_CellError.ERR_NONE THEN
                        _eNew := E_CellError.ERR_STEP_SEQUENCE;
                        _nNewRobot := _r;
                        _nNewOp := _nStep;
                    END_IF
                END_IF
                _awLastStep[_r] := afbRobot[_r].wStep;
                IF afbRobot[_r].wStep > _awMaxStep[_r] THEN
                    _awMaxStep[_r] := afbRobot[_r].wStep;
                END_IF
                _abGranted[_r] := FALSE;
                _atonOp[_r](IN := FALSE);
                _atonStall[_r](IN := FALSE);
            END_IF

            IF _nStep >= 1 AND _nStep <= GVL_Cell.anRobotOpCount[_r] THEN
                _iOp := GVL_Cell.anRobotFirstOp[_r] + _nStep - 1;
                IF GVL_Cell.aOps[_iOp].dwZoneMask = 0 THEN
                    _atonOp[_r](IN := TRUE, PT := GVL_Cell.aOps[_iOp].tLimit);
                ELSE
                    IF fbArbiter.adwGrant[_r] <> 0 THEN
                        _abGranted[_r] := TRUE;
                    END_IF
                    _atonOp[_r](IN := _abGranted[_r], PT := GVL_Cell.aOps[_iOp].tLimit);

                    _atonStall[_r](IN := NOT _abGranted[_r] AND (afbRobot[_r].dwZoneReq = 0),
                                   PT := GVL_Cell.T_HANDSHAKE);
                    IF _atonStall[_r].Q AND _eNew = E_CellError.ERR_NONE THEN
                        _eNew := E_CellError.ERR_ROBOT_STALLED;
                        _nNewRobot := _r;
                        _nNewOp := _nStep;
                    END_IF

                    IF NOT _abGranted[_r] AND (afbRobot[_r].dwZoneReq <> 0)
                       AND (_tonCycle.ET > GVL_Cell.aOps[_iOp].tZoneDeadline)
                       AND _eNew = E_CellError.ERR_NONE THEN
                        _eNew := E_CellError.ERR_ZONE_WAIT_TIMEOUT;
                        _nNewRobot := _r;
                        _nNewOp := _nStep;
                    END_IF
                END_IF
                IF _atonOp[_r].Q AND _eNew = E_CellError.ERR_NONE THEN
                    _eNew := E_CellError.ERR_OP_WATCHDOG;
                    _nNewRobot := _r;
                    _nNewOp := _nStep;
                END_IF
            END_IF
        END_IF
    END_FOR
END_IF

(* ============================================================================
   FAULT COLLECTION (first wins: robot > supervision > arbiter > operator)
   ============================================================================ *)
IF _eState <> E_CellState.CS_INIT AND _eState <> E_CellState.CS_ABORTING
   AND _eState <> E_CellState.CS_ERROR AND _eState <> E_CellState.CS_RESETTING THEN
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        IF afbRobot[_r].bError AND NOT _bFault THEN
            _bFault := TRUE;
            eError := afbRobot[_r].eError;
            nErrorRobot := _r;
            nErrorOp := WORD_TO_INT(afbRobot[_r].wStep);
            nErrorZone := 0;
        END_IF
    END_FOR
    IF _eNew <> E_CellError.ERR_NONE AND NOT _bFault THEN
        _bFault := TRUE;
        eError := _eNew;
        nErrorRobot := _nNewRobot;
        nErrorOp := _nNewOp;
        nErrorZone := 0;
    END_IF
    IF fbArbiter.bFault AND NOT _bFault THEN
        _bFault := TRUE;
        eError := fbArbiter.eFault;
        nErrorRobot := fbArbiter.nFaultRobot;
        nErrorOp := 0;
        nErrorZone := fbArbiter.nFaultZone;
    END_IF
    IF _rtrigStop.Q AND NOT _bFault THEN
        _bFault := TRUE;
        eError := E_CellError.ERR_OPERATOR_STOP;
        nErrorRobot := 0;
        nErrorOp := 0;
        nErrorZone := 0;
    END_IF
    IF _bFault THEN
        _eState := E_CellState.CS_ABORTING;
    END_IF
END_IF

(* ============================================================================
   STATE MACHINE
   ============================================================================ *)
CASE _eState OF

E_CellState.CS_INIT:
    _bPowerOn := FALSE;
    _bArbReset := TRUE;
    IF bEnable THEN
        _eState := E_CellState.CS_BRINGUP;
    END_IF

E_CellState.CS_BRINGUP:
    _bPowerOn := TRUE;
    _bAll := TRUE;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        IF NOT afbRobot[_r].bIdle THEN
            _bAll := FALSE;
        END_IF
    END_FOR
    IF _bAll THEN
        _eState := E_CellState.CS_HOMING_SELECT;
    END_IF

E_CellState.CS_HOMING_SELECT:
    _nHomeCount := 0;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        _abHoming[_r] := FALSE;
        IF NOT afbRobot[_r].bAtHome AND (NOT GVL_Cell.HOME_SEQUENTIAL OR _nHomeCount = 0) THEN
            _abHoming[_r] := TRUE;
            _awJob[_r] := GVL_Cell.HOME_FOLGE;
            _nHomeCount := _nHomeCount + 1;
        END_IF
    END_FOR
    IF _nHomeCount = 0 THEN
        _eState := E_CellState.CS_READY;
    ELSE
        _eState := E_CellState.CS_HOMING_RUN;
    END_IF

E_CellState.CS_HOMING_RUN:
    _bAll := TRUE;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        IF _abHoming[_r] THEN
            _abJobCmd[_r] := TRUE;
            IF NOT afbRobot[_r].bJobDone THEN
                _bAll := FALSE;
            ELSIF NOT afbRobot[_r].bAtHome AND NOT _bFault THEN
                _bFault := TRUE;
                eError := E_CellError.ERR_HOME_FAILED;
                nErrorRobot := _r;
            END_IF
        END_IF
    END_FOR
    IF _bFault THEN
        _eState := E_CellState.CS_ABORTING;
    ELSIF _bAll THEN
        FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
            IF _abHoming[_r] THEN
                _abJobCmd[_r] := FALSE;
                _abHoming[_r] := FALSE;
            END_IF
        END_FOR
        _eState := E_CellState.CS_HOMING_SELECT;
    ELSIF _tonState.Q THEN
        _bFault := TRUE;
        eError := E_CellError.ERR_HOME_FAILED;
        _eState := E_CellState.CS_ABORTING;
    END_IF

E_CellState.CS_READY:
    IF _rtrigStart.Q OR bAutoRun THEN
        _bAll := TRUE;
        FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
            IF GVL_Cell.anRobotOpCount[_r] > 0 AND NOT afbRobot[_r].bAtHome THEN
                _bAll := FALSE;             (* programs start at the home pose *)
            END_IF
        END_FOR
        IF _bAll THEN
            _bArbReset := TRUE;             (* rewind tickets for this cycle *)
            FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
                _awLastStep[_r] := 0;
                _awMaxStep[_r] := 0;
                _abGranted[_r] := FALSE;
                _atonOp[_r](IN := FALSE);
                _atonStall[_r](IN := FALSE);
                IF GVL_Cell.anRobotOpCount[_r] > 0 THEN
                    _awJob[_r] := GVL_Cell.PROD_FOLGE;
                END_IF
            END_FOR
            _tonCycle(IN := FALSE);
            _eState := E_CellState.CS_START_JOBS;
        ELSE
            _eState := E_CellState.CS_HOMING_SELECT;
        END_IF
    END_IF

E_CellState.CS_START_JOBS:
    _bAll := TRUE;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        IF GVL_Cell.anRobotOpCount[_r] > 0 THEN
            _abJobCmd[_r] := TRUE;
            IF NOT (afbRobot[_r].bJobActive OR afbRobot[_r].bJobDone) THEN
                _bAll := FALSE;
            END_IF
        END_IF
    END_FOR
    IF _bAll THEN
        _eState := E_CellState.CS_RUNNING;
    ELSIF _tonState.Q THEN
        _bFault := TRUE;
        eError := E_CellError.ERR_START_TIMEOUT;
        _eState := E_CellState.CS_ABORTING;
    END_IF

E_CellState.CS_RUNNING:
    _bAll := TRUE;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        IF GVL_Cell.anRobotOpCount[_r] > 0 THEN
            IF NOT afbRobot[_r].bJobDone THEN
                _bAll := FALSE;
            ELSIF WORD_TO_INT(_awMaxStep[_r]) <> GVL_Cell.anRobotOpCount[_r] AND NOT _bFault THEN
                _bFault := TRUE;
                eError := E_CellError.ERR_PROGRAM_INCOMPLETE;
                nErrorRobot := _r;
                nErrorOp := WORD_TO_INT(_awMaxStep[_r]);
            END_IF
        END_IF
    END_FOR
    IF _bFault THEN
        _eState := E_CellState.CS_ABORTING;
    ELSIF _bAll THEN
        IF NOT fbArbiter.bAllConsumed THEN
            _bFault := TRUE;
            eError := E_CellError.ERR_TICKETS_LEFT;
            _eState := E_CellState.CS_ABORTING;
        ELSE
            FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
                _abJobCmd[_r] := FALSE;
            END_FOR
            udiCycles := udiCycles + 1;
            tLastCycle := _tonCycle.ET;
            _eState := E_CellState.CS_CYCLE_DONE;
        END_IF
    END_IF

E_CellState.CS_CYCLE_DONE:
    IF _tonState.Q THEN
        _eState := E_CellState.CS_READY;
    END_IF

E_CellState.CS_ABORTING:
    _bArbReset := TRUE;
    _bAll := TRUE;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        _abJobCmd[_r] := FALSE;
        _abHoming[_r] := FALSE;
        _abStopCmd[_r] := TRUE;
        IF afbRobot[_r].bBusy THEN
            _bAll := FALSE;
        END_IF
    END_FOR
    IF _bAll OR _tonState.Q THEN
        _eState := E_CellState.CS_ERROR;
    END_IF

E_CellState.CS_ERROR:
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        _abStopCmd[_r] := FALSE;
        _abResetCmd[_r] := FALSE;           (* re-arm reset edges *)
    END_FOR
    IF _rtrigReset.Q THEN
        _eState := E_CellState.CS_RESETTING;
    END_IF

E_CellState.CS_RESETTING:
    _bAll := TRUE;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        _abResetCmd[_r] := afbRobot[_r].bError;
        IF afbRobot[_r].bError OR afbRobot[_r].bBusy THEN
            _bAll := FALSE;
        END_IF
    END_FOR
    IF _bAll THEN
        FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
            _abResetCmd[_r] := FALSE;
        END_FOR
        _bFault := FALSE;
        eError := E_CellError.ERR_NONE;
        nErrorRobot := 0;
        nErrorOp := 0;
        nErrorZone := 0;
        _bArbReset := TRUE;
        _eState := E_CellState.CS_BRINGUP;  (* then blind homing *)
    ELSIF _tonState.Q THEN
        _eState := E_CellState.CS_ERROR;
    END_IF

END_CASE

(* ============================================================================
   OUTPUTS
   ============================================================================ *)
eState := _eState;
bError := _bFault;
bCycleActive := (_eState = E_CellState.CS_START_JOBS) OR (_eState = E_CellState.CS_RUNNING);
FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
    awStep[_r] := afbRobot[_r].wStep;
    adwGrant[_r] := fbArbiter.adwGrant[_r];
END_FOR
```

**PLC_PRG (generated)**. The declaration holds `fbCell`, the operator flags (`bAutoRun := <cfg.auto_run>`, `bStart`, `bStop`, `bReset`), status readbacks and `_astIn : ARRAY[1..GVL_Cell.N_ROBOTS] OF ST_RobotIn`. The body has three parts:
1. Per-robot input copy: `_astIn[1].abySafeIn[0] := GVL_IO.bySafeIn0_R1; … _astIn[1].aVassIn := GVL_IO.aVassInputData_R1;`
2. `fbCell(...)` with `=>` readbacks.
3. Per-robot output copy (`GVL_IO.bySafeOut0_R1 := fbCell.astOut[1].abySafeOut[0]; … GVL_IO.aVassOutputData_R1 := fbCell.astOut[1].aVassOut;`), then the one-shot buttons are cleared (`bStart := FALSE; bStop := FALSE; bReset := FALSE;`).

### 5.2 Per-robot device sim (`codesys/robots/<ns>/`), project `RobotSim_<cell>_<tag>`
| # | Object | Kind | Origin |
|---|---|---|---|
| 1-9 | E_RobotMode, E_ProgramState, ST_SafeInput, ST_SafeOutput, ST_PROFIsafe_Trailer, ST_VASS_StatusWord, ST_VASS_ControlWord, ST_VASS_InputTelegram, ST_VASS_OutputTelegram | DUT | verbatim, split from VASS_Types.st |
| 10 | E_PlayerState | DUT enum | new: `PS_IDLE:=0, PS_NEXT_OP:=10, PS_WAIT_GRANT:=20, PS_DELAY:=30, PS_MOVE:=40, PS_HOLD:=50, PS_WAIT_RELEASE:=60, PS_DONE:=90, PS_ERROR:=99` |
| 11 | ST_OpDef | DUT | new: `sName STRING(32); nFirstWp, nLastWp INT; dwZoneMask DWORD; tDelay, tHold TIME; rVelScale REAL; bBlindHome BOOL` |
| 12 | ST_Waypoint | DUT | new, flat fields for portability: `rT, rA1..rA6 : REAL` (deg, KUKA convention after joint mapping) |
| 13 | GVL_IO | GVL | verbatim |
| 14 | GVL_DeviceParams | GVL CONSTANT | verbatim |
| 15 | GVL_Program | GVL CONSTANT | **generated** |
| 16 | FB_PROFIsafe_Simulator (+Body) | FB | verbatim |
| 17 | FB_JointAngleEncoder (+Body) | FB | verbatim |
| 18 | FB_ProgramPlayer (+Body) | FB | new template |
| 19 | FB_VASS_DeviceSimulator (+Body) | FB | adapted template |
| 20 | PLC_PRG (+Body) | PRG | adapted template (replaces the default) |

FB_JointAngleSimulator is intentionally omitted: the player supplies the angles.

**GVL_Program (generated, robot1, excerpt)**:
```
{attribute 'qualified_only'}
VAR_GLOBAL CONSTANT
    ROBOT_TAG           : STRING(16) := 'R1';
    ROBOT_NS            : STRING(32) := 'robot1';
    ROBOT_INDEX         : WORD := 1;
    PLAN_ID             : WORD := 16#5A3C;
    PROD_FOLGE          : WORD := 11;
    HOME_FOLGE          : WORD := 125;
    JOB_NUMBER_BYTE     : BOOL := FALSE;
    STEP_BYTE           : INT := 4;
    REQ_BYTE            : INT := 28;
    GRANT_BYTE          : INT := 12;
    PLAN_ID_BYTE        : INT := 4;
    T_HOME              : TIME := T#7S;
    R_AT_HOME_TOL_DEG   : REAL := 0.5;
    R_START_TOL_DEG     : REAL := 1.0;
    ERR_UNKNOWN_FOLGE   : WORD := 16#0101;
    ERR_PLAN_MISMATCH   : WORD := 16#0103;
    WARN_START_DEVIATION: WORD := 16#0201;
    N_OPS               : INT := 8;
    N_WP                : INT := 211;
    arHomeDeg           : ARRAY[1..6] OF REAL := [0.0000, -90.0000, 90.0000, 0.0000, 0.0000, 0.0000];
    aOps                : ARRAY[1..8] OF ST_OpDef := [
        (sName := 'W01', nFirstWp := 1, nLastWp := 47, dwZoneMask := 16#00000001, tDelay := T#0MS, tHold := T#500MS, rVelScale := 1.0, bBlindHome := FALSE),
        (sName := 'W02', nFirstWp := 48, nLastWp := 68, dwZoneMask := 16#00000002, tDelay := T#0MS, tHold := T#500MS, rVelScale := 1.0, bBlindHome := FALSE),
        (* ... *)
        (sName := 'HOME', nFirstWp := 170, nLastWp := 211, dwZoneMask := 16#00000002, tDelay := T#0MS, tHold := T#0MS, rVelScale := 1.0, bBlindHome := FALSE)
    ];
    aWp                 : ARRAY[1..211] OF ST_Waypoint := [
        (* op 1 W01 - traj_robot1_W01 *)
        (rT := 0.0000, rA1 := 0.0000, rA2 := -90.0000, rA3 := 90.0000, rA4 := 0.0000, rA5 := 0.0000, rA6 := 0.0000),
        (rT := 0.2677, rA1 := -0.2925, rA2 := -89.9560, rA3 := 90.0355, rA4 := 0.0753, rA5 := 0.1975, rA6 := 0.0896),
        (* ... *)
    ];
END_VAR
```

#### FB_ProgramPlayer
```
(*
 * FB_ProgramPlayer
 *
 * Plays the bt2plc robot program (GVL_Program) for PROD_FOLGE, or a blind joint
 * move to the home pose for HOME_FOLGE. Robot side of the zone protocol:
 *   op k: wStep := k; REQUEST := mask; wait GRANT = mask; delay; trajectory; hold;
 *         REQUEST := 0; wait GRANT = 0 (4-phase)
 * Joints (deg) are linearly interpolated and published as rA1..rA6_deg.
 *)

FUNCTION_BLOCK FB_ProgramPlayer
VAR_INPUT
    bStart          : BOOL;             (* Rising edge: start wJob *)
    wJob            : WORD;
    bAbort          : BOOL;             (* Level: stop / cancel / drives off / E-stop *)
    dwGrant         : DWORD;            (* Zone GRANT bits from head PLC *)
    rRate           : REAL := 1.0;      (* Speed override factor *)
END_VAR

VAR_OUTPUT
    eState          : E_PlayerState;
    bBusy           : BOOL;
    bDone           : BOOL;             (* Latched until next start *)
    bError          : BOOL;             (* Latched until next start *)
    wErrorCode      : WORD;
    wWarningCode    : WORD;
    wStep           : WORD;             (* 1-based op index, 0 = none *)
    dwRequest       : DWORD;
    bAtHome         : BOOL;
    rA1_deg         : REAL;
    rA2_deg         : REAL;
    rA3_deg         : REAL;
    rA4_deg         : REAL;
    rA5_deg         : REAL;
    rA6_deg         : REAL;
END_VAR

VAR
    _eState         : E_PlayerState := E_PlayerState.PS_IDLE;
    _rtrigStart     : R_TRIG;
    _nOp            : INT;
    _nWp            : INT;
    _nLast          : INT;
    _dwMask         : DWORD;
    _bHomeMove      : BOOL;
    _rT             : REAL;
    _rDur           : REAL;
    _rAlpha         : REAL;
    _rSpan          : REAL;
    _arQ            : ARRAY[1..6] OF REAL;
    _arQ0           : ARRAY[1..6] OF REAL;
    _tonDelay       : TON;
    _tonHold        : TON;
    _tNow           : TIME;
    _tLast          : TIME;
    _rDt            : REAL;
    _bFirstCall     : BOOL := TRUE;
    _i              : INT;
END_VAR
```
```
(*
 * FB_ProgramPlayer - Implementation
 *)

(* ============================================================================
   INIT (robot powers up at home) / CYCLE TIME
   ============================================================================ *)
IF _bFirstCall THEN
    FOR _i := 1 TO 6 DO
        _arQ[_i] := GVL_Program.arHomeDeg[_i];
    END_FOR
    _tLast := TIME();
    _bFirstCall := FALSE;
END_IF
_tNow := TIME();
_rDt := TIME_TO_REAL(_tNow - _tLast) / 1000.0;
IF _rDt <= 0.0 OR _rDt > 1.0 THEN
    _rDt := 0.02;                           (* device MainTask interval *)
END_IF
_tLast := _tNow;

_rtrigStart(CLK := bStart);

(* ============================================================================
   ABORT / START
   ============================================================================ *)
IF bAbort AND bBusy THEN
    _eState := E_PlayerState.PS_IDLE;       (* pose freezes *)
END_IF

IF _rtrigStart.Q AND NOT bBusy THEN
    bDone := FALSE;
    bError := FALSE;
    wErrorCode := 0;
    wWarningCode := 0;
    _tonDelay(IN := FALSE);
    _tonHold(IN := FALSE);
    IF wJob = GVL_Program.HOME_FOLGE THEN
        _nOp := 0;
        FOR _i := 1 TO 6 DO
            _arQ0[_i] := _arQ[_i];
        END_FOR
        _rDur := TIME_TO_REAL(GVL_Program.T_HOME) / 1000.0;
        _rT := 0.0;
        _bHomeMove := TRUE;
        bBusy := TRUE;
        _eState := E_PlayerState.PS_MOVE;
    ELSIF wJob = GVL_Program.PROD_FOLGE AND GVL_Program.N_OPS > 0 THEN
        _nOp := 0;
        _eState := E_PlayerState.PS_NEXT_OP;
    ELSE
        wErrorCode := GVL_Program.ERR_UNKNOWN_FOLGE;
        _eState := E_PlayerState.PS_ERROR;
    END_IF
END_IF

(* ============================================================================
   STATE MACHINE
   ============================================================================ *)
CASE _eState OF

E_PlayerState.PS_IDLE:
    bBusy := FALSE;
    wStep := 0;
    dwRequest := 0;

E_PlayerState.PS_NEXT_OP:
    bBusy := TRUE;
    _nOp := _nOp + 1;
    IF _nOp > GVL_Program.N_OPS THEN
        _eState := E_PlayerState.PS_DONE;
    ELSE
        wStep := INT_TO_WORD(_nOp);
        _dwMask := GVL_Program.aOps[_nOp].dwZoneMask;
        _bHomeMove := GVL_Program.aOps[_nOp].bBlindHome;
        _rT := 0.0;
        IF _bHomeMove THEN
            FOR _i := 1 TO 6 DO
                _arQ0[_i] := _arQ[_i];
            END_FOR
            _rDur := TIME_TO_REAL(GVL_Program.T_HOME) / 1000.0;
        ELSE
            _nWp := GVL_Program.aOps[_nOp].nFirstWp;
            _nLast := GVL_Program.aOps[_nOp].nLastWp;
            IF ABS(_arQ[1] - GVL_Program.aWp[_nWp].rA1) > GVL_Program.R_START_TOL_DEG
               OR ABS(_arQ[2] - GVL_Program.aWp[_nWp].rA2) > GVL_Program.R_START_TOL_DEG
               OR ABS(_arQ[3] - GVL_Program.aWp[_nWp].rA3) > GVL_Program.R_START_TOL_DEG
               OR ABS(_arQ[4] - GVL_Program.aWp[_nWp].rA4) > GVL_Program.R_START_TOL_DEG
               OR ABS(_arQ[5] - GVL_Program.aWp[_nWp].rA5) > GVL_Program.R_START_TOL_DEG
               OR ABS(_arQ[6] - GVL_Program.aWp[_nWp].rA6) > GVL_Program.R_START_TOL_DEG THEN
                wWarningCode := GVL_Program.WARN_START_DEVIATION;
            END_IF
        END_IF
        IF _dwMask <> 0 THEN
            dwRequest := _dwMask;
            _eState := E_PlayerState.PS_WAIT_GRANT;
        ELSE
            _eState := E_PlayerState.PS_DELAY;
        END_IF
    END_IF

E_PlayerState.PS_WAIT_GRANT:
    IF (dwGrant AND _dwMask) = _dwMask THEN
        _eState := E_PlayerState.PS_DELAY;
    END_IF

E_PlayerState.PS_DELAY:
    _tonDelay(IN := TRUE, PT := GVL_Program.aOps[_nOp].tDelay);
    IF _tonDelay.Q THEN
        _tonDelay(IN := FALSE);
        _eState := E_PlayerState.PS_MOVE;
    END_IF

E_PlayerState.PS_MOVE:
    IF _bHomeMove THEN
        _rT := _rT + _rDt * rRate;
        IF _rDur > 0.0 THEN
            _rAlpha := _rT / _rDur;
        ELSE
            _rAlpha := 1.0;
        END_IF
        IF _rAlpha > 1.0 THEN
            _rAlpha := 1.0;
        END_IF
        FOR _i := 1 TO 6 DO
            _arQ[_i] := _arQ0[_i] + _rAlpha * (GVL_Program.arHomeDeg[_i] - _arQ0[_i]);
        END_FOR
        IF _rAlpha >= 1.0 THEN
            IF _nOp = 0 THEN
                _eState := E_PlayerState.PS_DONE;     (* HOME_FOLGE *)
            ELSE
                _eState := E_PlayerState.PS_HOLD;     (* blind MoveHome op *)
            END_IF
        END_IF
    ELSE
        _rT := _rT + _rDt * rRate * GVL_Program.aOps[_nOp].rVelScale;
        WHILE _nWp < _nLast DO
            IF GVL_Program.aWp[_nWp + 1].rT <= _rT THEN
                _nWp := _nWp + 1;
            ELSE
                EXIT;
            END_IF
        END_WHILE
        IF _nWp >= _nLast THEN
            _arQ[1] := GVL_Program.aWp[_nLast].rA1;
            _arQ[2] := GVL_Program.aWp[_nLast].rA2;
            _arQ[3] := GVL_Program.aWp[_nLast].rA3;
            _arQ[4] := GVL_Program.aWp[_nLast].rA4;
            _arQ[5] := GVL_Program.aWp[_nLast].rA5;
            _arQ[6] := GVL_Program.aWp[_nLast].rA6;
            _eState := E_PlayerState.PS_HOLD;
        ELSE
            _rSpan := GVL_Program.aWp[_nWp + 1].rT - GVL_Program.aWp[_nWp].rT;
            IF _rSpan > 0.0 THEN
                _rAlpha := (_rT - GVL_Program.aWp[_nWp].rT) / _rSpan;
            ELSE
                _rAlpha := 1.0;
            END_IF
            _arQ[1] := GVL_Program.aWp[_nWp].rA1 + _rAlpha * (GVL_Program.aWp[_nWp + 1].rA1 - GVL_Program.aWp[_nWp].rA1);
            _arQ[2] := GVL_Program.aWp[_nWp].rA2 + _rAlpha * (GVL_Program.aWp[_nWp + 1].rA2 - GVL_Program.aWp[_nWp].rA2);
            _arQ[3] := GVL_Program.aWp[_nWp].rA3 + _rAlpha * (GVL_Program.aWp[_nWp + 1].rA3 - GVL_Program.aWp[_nWp].rA3);
            _arQ[4] := GVL_Program.aWp[_nWp].rA4 + _rAlpha * (GVL_Program.aWp[_nWp + 1].rA4 - GVL_Program.aWp[_nWp].rA4);
            _arQ[5] := GVL_Program.aWp[_nWp].rA5 + _rAlpha * (GVL_Program.aWp[_nWp + 1].rA5 - GVL_Program.aWp[_nWp].rA5);
            _arQ[6] := GVL_Program.aWp[_nWp].rA6 + _rAlpha * (GVL_Program.aWp[_nWp + 1].rA6 - GVL_Program.aWp[_nWp].rA6);
        END_IF
    END_IF

E_PlayerState.PS_HOLD:
    _tonHold(IN := TRUE, PT := GVL_Program.aOps[_nOp].tHold);
    IF _tonHold.Q THEN
        _tonHold(IN := FALSE);
        IF _dwMask <> 0 THEN
            dwRequest := 0;
            _eState := E_PlayerState.PS_WAIT_RELEASE;
        ELSE
            _eState := E_PlayerState.PS_NEXT_OP;
        END_IF
    END_IF

E_PlayerState.PS_WAIT_RELEASE:
    IF (dwGrant AND _dwMask) = 0 THEN
        _eState := E_PlayerState.PS_NEXT_OP;
    END_IF

E_PlayerState.PS_DONE:
    bBusy := FALSE;
    bDone := TRUE;
    wStep := 0;
    dwRequest := 0;
    _eState := E_PlayerState.PS_IDLE;

E_PlayerState.PS_ERROR:
    bBusy := FALSE;
    bError := TRUE;
    wStep := 0;
    dwRequest := 0;
    _eState := E_PlayerState.PS_IDLE;

END_CASE

(* ============================================================================
   PUBLISH
   ============================================================================ *)
bAtHome := TRUE;
FOR _i := 1 TO 6 DO
    IF ABS(_arQ[_i] - GVL_Program.arHomeDeg[_i]) > GVL_Program.R_AT_HOME_TOL_DEG THEN
        bAtHome := FALSE;
    END_IF
END_FOR
rA1_deg := _arQ[1];
rA2_deg := _arQ[2];
rA3_deg := _arQ[3];
rA4_deg := _arQ[4];
rA5_deg := _arQ[5];
rA6_deg := _arQ[6];
eState := _eState;
```

**FB_VASS_DeviceSimulator (adapted)** is a copy of the existing FB. Changes:
1. New VARs `_fbPlayer : FB_ProgramPlayer; _bPlayerStart : BOOL; _dwGrant : DWORD; _wPlanId : WORD;` and VAR_OUTPUT `rA1_deg..rA6_deg`. `_tonJobExecution` and `_tonHoming` are removed.
2. Job number read: `IF GVL_Program.JOB_NUMBER_BYTE THEN _wRequestedJob := BYTE_TO_WORD(byte 2) ELSE` (WORD LE as today).
3. In the job-accept branch also latch `_wPlanId := bytes PLAN_ID_BYTE..+1 (LE)`.
4. Program start (replaces the existing block): `IF _rtrigProgStart.Q AND _bReady AND _wActiveJob > 0 THEN`
   - `IF _wActiveJob = HOME_FOLGE OR (_wActiveJob = PROD_FOLGE AND N_OPS > 0 AND _wPlanId = PLAN_ID)`: running := TRUE, finished := FALSE, InPosition := FALSE, PRG_RUNNING, `_bPlayerStart := TRUE`.
   - `ELSIF _wActiveJob = PROD_FOLGE`: `_bError := TRUE; _wErrorCode := ERR_PLAN_MISMATCH`.
   - `ELSE`: `_bError := TRUE; _wErrorCode := ERR_UNKNOWN_FOLGE`.
5. A HomeRequest bit `[0].6` rising edge while Ready behaves like HOME_FOLGE.
6. After the existing Stop/Cancel block:
   - `_dwGrant := bytes GRANT_BYTE..+3 (LE)`
   - `_fbPlayer(bStart := _bPlayerStart, wJob := _wActiveJob, bAbort := NOT _bProgRunning, dwGrant := _dwGrant, rRate := WORD_TO_REAL(_wSpeed) / 100.0); _bPlayerStart := FALSE;`
   - while running: player error → `_bError`, its code, `_bProgRunning := FALSE`, PRG_ERROR; player done → `_bProgRunning := FALSE; _bProgFinished := TRUE; _bInPosition := TRUE; PRG_FINISHED`
   - `_wCurrentStep := _fbPlayer.wStep; _bAtHome := _fbPlayer.bAtHome; _wWarningCode := _fbPlayer.wWarningCode;` and the outputs `rAx_deg := _fbPlayer.rAx_deg`.
7. Section 5: bytes 2-3 echo honour byte mode (byte 3 := 0); bytes 28-31 ← `_fbPlayer.dwRequest` (LE) instead of zeros.

**PLC_PRG (adapted)**: same as the existing one without fbJointSimulator. `fbJointEncoder(rA1_deg := fbVassSimulator.rA1_deg, …)`, `wRobotID := GVL_Program.ROBOT_INDEX`. The monitor/decode and error-injection code is kept.

---

## 6. Outputs

### 6.1 Tree for `bt2plc generate bt_1790934549_welding_cell_2_small.json -o out/`
```
out/
├── .bt2plc-manifest                 list of written files (for --force cleanup)
├── cell_plan.json                   IR dump (schema bt2plc/cell-plan@1)
├── ZONES.md                         zones, robot programs, edge→zone coverage, schedule, warnings
├── PROJECT.md                       human build guide (device trees, IPs, IO, tasks, objects, ops)
├── MCP_PROMPT.md                    agent prompt for building the projects with a CODESYS MCP server
├── project.json                     machine-readable build manifest (schema bt2plc/codesys-project@1)
├── robots/robot1/program.json       robot program spec (future KRL generator)
├── robots/robot2/program.json
├── docker/docker-compose.robots.yml extra device containers (robot ≥ 2)
├── docker/README.md
└── codesys/
    ├── build_project.py             optional CODESYS ScriptEngine script (unverified API, see §9)
    ├── head/
    │   ├── io_mapping.csv
    │   ├── DUT/{E_GrantPolicy,E_CellError,E_RobotState,E_CellState,ST_Ticket,ST_OpInfo,ST_RobotIn,ST_RobotOut}.st
    │   ├── GVL/{GVL_Cell,GVL_IO}.st
    │   └── POU/{FB_VassRobot,FB_VassRobot_Body,FB_ZoneArbiter,FB_ZoneArbiter_Body,FB_Cell,FB_Cell_Body,PLC_PRG,PLC_PRG_Body}.st
    └── robots/
        ├── robot1/  io_mapping.csv, DUT/(12 files), GVL/{GVL_IO,GVL_DeviceParams,GVL_Program}.st,
        │            POU/{FB_PROFIsafe_Simulator,…_Body,FB_JointAngleEncoder,…_Body,FB_ProgramPlayer,…_Body,
        │                 FB_VASS_DeviceSimulator,…_Body,PLC_PRG,PLC_PRG_Body}.st
        └── robot2/  (same)
```

### 6.2 project.json schema (abridged with the welding values)
```json
{
  "schema": "bt2plc/codesys-project@1",
  "generator": "bt2plc 0.1.0",
  "plan": { "tree_id": "bt_1790934549_welding_cell_2_small", "cell_id": "welding_cell_2_small",
            "fingerprint": "fnv1a64:…", "plan_id": "16#5A3C", "grant_policy": "planned" },
  "ide": { "product": "CODESYS", "version": "CODESYS V3.5 SP22 Patch 1", "os": "windows" },
  "prerequisites": [
    { "id": "gsdml", "kind": "device_repository_install",
      "file": ".../kuka-vass-device-simulator/gsdml/GSDML-V2.25-KUKA-Roboter-GmbH-VKR C4-Device-V1.0-20121030.xml",
      "provides": { "type": 81, "id": "GSDML-019B-0301-V2.25-KUKA-Roboter-GmbH-VKR C4-Device-V1.0-20121030 - DIM 2", "version": "V8.2 - HW 1" } },
    { "id": "pn_device_desc", "kind": "manual", "how": "Tools > Miscellaneous > Generate PROFINET Device from GSDML (KUKA GSDML)",
      "provides": { "type": 83, "id": "0x019B_0x0301_DIM 2", "version": "1 V8.2" } } ],
  "projects": [
   { "key": "head", "role": "head_plc", "name": "HeadPLC_welding_cell_2_small",
     "file": "HeadPLC_welding_cell_2_small.project", "source_dir": "codesys/head",
     "runtime": { "container": "codesys-master", "ip": "192.168.100.10", "gateway_port": 11740, "opcua_port": 4840 },
     "device_tree": {
       "name": "Device", "device_id": { "type": 4102, "id": "0000 0005", "version": "4.20.0.0" },
       "children": [ { "name": "Ethernet", "device_id": { "type": 110, "id": "0000 0002", "version": "4.2.0.0" },
         "parameters": [ { "id": 0, "name": "IPAddress", "value": "[192,168,100,10]" },
                         { "id": 1, "name": "SubnetMask", "value": "[255,255,255,0]" },
                         { "id": 2, "name": "Gateway", "value": "[0,0,0,0]" },
                         { "id": 10, "name": "InterfaceName", "value": "'eth1'" },
                         { "id": 12, "name": "InterfaceName", "value": "\"eth1\"" } ],
         "children": [ { "name": "PN_Controller", "device_id": { "type": 80, "id": "0000 0002", "version": "4.7.1.0" },
           "parameters": [ { "id": 130, "name": "StationName", "value": "'controller'" },
                           { "id": 384, "name": "IPRangeFirst", "value": "[192,168,100,2]" },
                           { "id": 385, "name": "IPRangeLast", "value": "[192,168,100,254]" } ],
           "children": [
             { "name": "kuka_robi1", "robot": "robot1",
               "device_id": { "type": 81, "id": "GSDML-019B-0301-V2.25-KUKA-Roboter-GmbH-VKR C4-Device-V1.0-20121030 - DIM 2", "version": "V8.2 - HW 1" },
               "parameters": [ { "id": 512, "member": "StationName", "value": "'kuka-robi1'" },
                               { "id": 512, "member": "IPAddress", "value": "[192,168,100,11]" } ],
               "slots": [
                 { "slot": 1, "name": "SafeIO_FDIO64_R1", "module_id": "1", "ident": "16#8F", "profisafe": true,
                   "io": [ { "param_id": 77824, "channel": "8 safe digital inputs",    "sub": 0, "dir": "in",  "variable": "Application.GVL_IO.bySafeIn0_R1" },
                           "… sub 1..7 → bySafeIn1_R1..7, channel suffix @1..@7 …",
                           { "param_id": 81920, "channel": "8 safe digital outputs",   "sub": 0, "dir": "out", "variable": "Application.GVL_IO.bySafeOut0_R1" } ] },
                 { "slot": 2, "name": "VASS_DIO256_R1", "module_id": "7", "ident": "16#1000",
                   "io": [ { "param_id": 77824, "channel": "Inputs",  "sub": null, "dir": "in",  "variable": "Application.GVL_IO.aVassInputData_R1" },
                           { "param_id": 81920, "channel": "Outputs", "sub": null, "dir": "out", "variable": "Application.GVL_IO.aVassOutputData_R1" } ] } ] },
             { "name": "kuka_robi2", "robot": "robot2", "…": "station kuka-robi2, IP [192,168,100,12], modules *_R2" } ] } ] } ] },
     "libraries": [
       { "name": "Standard", "vendor": "System", "version": "*", "mode": "ensure" },
       { "name": "IoStandard", "mode": "auto" }, { "name": "IoDrvEthernet", "version": "4.2.0.0", "mode": "auto" },
       { "name": "IoDrvProfinet", "version": "4.7.1.0", "mode": "auto" }, { "name": "ProfinetCommon", "version": "4.7.0.0", "mode": "auto" },
       { "name": "CommFB", "version": "3.5.17.10", "mode": "auto" }, { "name": "CAA Device Diagnosis", "mode": "auto" },
       { "name": "3SLicense", "mode": "auto" } ],
     "tasks": [
       { "name": "MainTask", "kind": "cyclic", "interval": "t#4ms", "priority": 1, "pous": ["PLC_PRG"], "mode": "ensure" },
       { "name": "Profinet_CommunicationTask", "kind": "cyclic", "interval": "t#10ms", "priority": 14, "pous": ["PN_Controller.CommCycle"], "mode": "verify_or_create" },
       { "name": "Profinet_IOTask", "kind": "cyclic", "interval": "t#1ms", "priority": 1, "pous": [], "bus_cycle_task_of": "PN_Controller", "mode": "verify_or_create" } ],
     "objects": [
       { "order": 1, "name": "E_GrantPolicy", "kind": "dut", "declaration": "codesys/head/DUT/E_GrantPolicy.st", "origin": "template" },
       { "order": 9, "name": "GVL_Cell", "kind": "gvl", "declaration": "codesys/head/GVL/GVL_Cell.st", "origin": "generated" },
       { "order": 11, "name": "FB_VassRobot", "kind": "pou", "pou_type": "function_block", "language": "ST",
         "declaration": "codesys/head/POU/FB_VassRobot.st", "implementation": "codesys/head/POU/FB_VassRobot_Body.st", "origin": "template" },
       { "order": 14, "name": "PLC_PRG", "kind": "pou", "pou_type": "program", "language": "ST", "replace_existing": true,
         "declaration": "codesys/head/POU/PLC_PRG.st", "implementation": "codesys/head/POU/PLC_PRG_Body.st", "origin": "generated" } ],
     "io_mapping_csv": "codesys/head/io_mapping.csv",
     "steps": [
       { "op": "create_project", "template": "Standard project", "device": { "type": 4102, "id": "0000 0005", "version": "4.20.0.0" }, "language": "ST" },
       { "op": "add_device", "parent": "Device", "ref": "Ethernet" }, { "op": "set_device_parameters", "ref": "Ethernet" },
       { "op": "add_device", "parent": "Ethernet", "ref": "PN_Controller" }, { "op": "set_device_parameters", "ref": "PN_Controller" },
       { "op": "add_device", "parent": "PN_Controller", "ref": "kuka_robi1" }, { "op": "set_device_parameters", "ref": "kuka_robi1" },
       { "op": "ensure_module", "device": "kuka_robi1", "slot": 1, "module_id": "1", "name": "SafeIO_FDIO64_R1" },
       { "op": "ensure_module", "device": "kuka_robi1", "slot": 2, "module_id": "7", "name": "VASS_DIO256_R1" },
       "… robot2 …",
       { "op": "ensure_libraries" }, { "op": "create_objects", "order": "objects[].order" },
       { "op": "configure_tasks" }, { "op": "map_io", "source": "slots[].io (or io_mapping_csv)" },
       { "op": "build", "expect_errors": 0 }, { "op": "save" } ],
     "acceptance": { "build_errors": 0, "objects": ["E_GrantPolicy","…","PLC_PRG"], "devices": ["Ethernet","PN_Controller","kuka_robi1","kuka_robi2"] } },
   { "key": "robot1-sim", "role": "robot_sim", "robot": "robot1", "name": "RobotSim_welding_cell_2_small_R1",
     "source_dir": "codesys/robots/robot1",
     "runtime": { "container": "codesys-device", "ip": "192.168.100.11", "gateway_port": 11741, "opcua_port": 4841 },
     "device_tree": { "name": "Device", "device_id": { "type": 4102, "id": "0000 0005", "version": "4.20.0.0" },
       "children": [ { "name": "Ethernet", "device_id": { "type": 110, "id": "0000 0002", "version": "4.2.0.0" },
         "parameters": [ { "id": 0, "value": "[192,168,100,11]" }, { "id": 1, "value": "[255,255,255,0]" }, { "id": 2, "value": "[0,0,0,0]" },
                         { "id": 10, "value": "'eth1'" }, { "id": 12, "value": "\"eth1\"" } ],
         "children": [ { "name": "kuka_robi1", "device_id": { "type": 83, "id": "0x019B_0x0301_DIM 2", "version": "1 V8.2" },
           "parameters": [ { "id": 2, "name": "StationName", "value": "'kuka-robi1'" } ],
           "slots": [ { "slot": 1, "name": "SafeIO_FDIO64", "module_id": "1", "io": [ "… same channels, variables GVL_IO.bySafeIn0..7 / bySafeOut0..7 (device side: robot→PLC = %Q) …" ] },
                      { "slot": 2, "name": "VASS_DIO256", "module_id": "7",
                        "io": [ { "param_id": 77824, "channel": "Input Data",  "variable": "Application.GVL_IO.aVassInputData" },
                                { "param_id": 81920, "channel": "Output Data", "variable": "Application.GVL_IO.aVassOutputData" } ] } ] } ] } ] },
     "libraries": [ { "name": "Standard", "mode": "ensure" }, { "name": "IoDrvProfinetDevice", "version": "4.7.0.0", "mode": "auto" }, "…" ],
     "tasks": [ { "name": "MainTask", "interval": "t#20ms", "priority": 1, "pous": ["PLC_PRG"] },
                { "name": "Profinet_CommunicationTask", "interval": "t#10ms", "priority": 30, "pous": ["kuka_robi1.CommCycle"] },
                { "name": "Profinet_IOTask", "interval": "t#1ms", "priority": 1, "pous": [] } ],
     "objects": [ "… 20 objects in §5.2 order …" ], "steps": [ "… as head, without PN_Controller …" ] } ]
}
```
`io_mapping.csv` follows the observed "CoDeSys Mapping Export V1.2" layout. The three header lines are copied verbatim. Rows are `Application.GVL_IO.bySafeIn1_R1,8 safe digital inputs@1,,R1 safe status byte 1,,SafeIO_FDIO64_R1`; DIO256 rows use `Inputs`/`Outputs` (head) or `Input Data`/`Output Data` (device). The IEC address column is empty (see §9).

**program.json** (`bt2plc/robot-program@1`, `schema_minor: 1`; minor versions only add fields) contains:
- `plan{tree_id, cell_id, plan_id}`, `robot{ns, tag, index, station, ip, model}`, `folgen{production, home}`
  (`model` from `[[robot]] model`, default `"unknown"`; used by bt2krl for soft limits and axis speeds)
- `home_pose_deg`, `joint_mapping{sign, offset_deg, source_unit:"rad"}`
- `protocol{step{byte:4,len:2,endian:"little"}, request{byte:28,len:4}, grant{byte:12,len:4}, plan_id{byte:4,len:2,dir:"plc_to_robot"}, permit_bit:31, handshake:"4-phase", grant_policy,
  profile:"sim", job_number, error_codes{unknown_folge:257, plan_mismatch:259, start_deviation:513}, at_home_tol_deg, start_tol_deg, home_return_time_s, handshake_s, abort_limit_s, home_limit_s}`
  (the robot-side constants of the device simulator, and the head timeouts the robot must respect)
- `zones[{name, bit}]` (all zones of the plan)
- `ops[{index, operation_id, kind, bt_path, zones, zone_mask, waits_for, start_delay_s, hold_s, velocity_scale, duration_s, expected_s, limit_s, delay_exact, blind_home, trajectory_key, waypoints[{t_s, q_deg[6], phase}]}]`

**cell_plan.json** (`bt2plc/cell-plan@1`) contains:
- `generator`; `source{file, format, tree_id, cell_id, created_s, fingerprint}`; the effective `config`
- `robots[]`; `ops[]` (fields as above plus `asap{start_s,end_s}`, `depth`, `zone_deadline_s`, `pos`)
- `cross_edges[{from:"robot2:W13", to:"robot1:W02", zone:"IL02"}]`; `zones[{name, bit, tickets[{robot, index, operation_id}]}]`
- `schedule{makespan_s: 64.2969, critical_path[]}`; `diagnostics[]`

### 6.3 PROJECT.md outline
1. Overview: cell, plan id, robots table (tag, ns, station, device, IP, op count, zones used), makespan.
2. Network: Docker bridge 192.168.100.0/24, IP and port table, containers.
3. Prerequisites: GSDML install; one-time "Generate PROFINET Device from GSDML" (not scriptable).
4. Head PLC project: device tree diagram, parameter table, module/IO mapping tables, tasks, libraries, object list with creation order.
5. Device-sim projects (one section per robot): same tables.
6. Build: MCP route (MCP_PROMPT.md), script route (`CODESYS.exe --profile="CODESYS V3.5 SP22 Patch 1" --runscript=build_project.py`), manual route.
7. Download and run: login targets, order (start device runtimes first), `docker compose -f … -f docker-compose.robots.yml up -d`.
8. Operation: PLC_PRG flags (bAutoRun, bStart, bStop, bReset); watch `fbCell.eState`/`eError`/`nErrorRobot`/`nErrorOp`/`nErrorZone`; error code table (E_CellError + device 16#01xx/16#02xx).
9. Protocol reference: link to §4 content, plus the zones (copy of the ZONES.md summary).
10. Known limitations.

### 6.4 MCP_PROMPT.md outline
1. Role and goal: build the CODESYS projects described in `project.json`. Never modify existing projects; create the new files in `<out>/projects/`.
2. Environment checks: Windows; IDE version; MCP server connected; **list the server's tools and read their schemas first**; never guess parameter names.
3. Capability mapping table. For each step: Codesys-MCP-Master tool (`add_device`/`plug_device`/`set_device_parameter`/`import_io_mappings_csv`/`create_task`/`add_pou_to_task`/`add_library`/`import_plcopen_xml`, POU code tools); luke-harriman `eval_python` fallback snippets (ScriptEngine); codesys-mcp-toolkit (`create_pou` + `set_pou_code(declarationCode, implementationCode)` + `compile` only, so devices, DUTs, GVLs, tasks and IO are marked HUMAN steps).
4. Prerequisite verification: the device IDs from `prerequisites[].provides` must be addable. Stop and report if the generated PN device description is missing.
5. Procedure per project, in `steps[]` order, with a verification checkpoint after each group (device tree listing, parameter read-back, object list, task list).
6. Code insertion rule: the declaration file goes to the declaration part and the `_Body` file to the implementation part, byte-exact; DUT/GVL use the declaration file only; replace the default PLC_PRG.
7. IO mapping: prefer `slots[].io` (per channel); fallback CSV import; read back and compare.
8. Build: 0 errors required. On errors, report the message list verbatim and do not edit generated code except for an obvious template typo, which must be reported.
9. Save and report: a JSON report (`projects[].{file, built, errors, warnings, deviations[]}`).
10. Safety notes: simulation only; zones are logical, not safety-rated.

### 6.5 docker/docker-compose.robots.yml (robot1 reuses the existing `device` service)
```yaml
# generated by bt2plc for welding_cell_2_small - use from profinet-headplc-robot-simulation/docker:
#   cp -r device-config device-r2-config          (once per extra robot)
#   docker compose -f docker-compose.yml -f <out>/docker/docker-compose.robots.yml up -d --build
# R1 robot1 kuka-robi1 192.168.100.11 -> existing service 'device'
services:
  device-r2:                       # R2 robot2 kuka-robi2
    build:
      context: .
      args:
        LIB_DIR: ${LIB_DIR:-x86_64-linux-gnu}
    container_name: codesys-device-r2
    cap_add: [NET_ADMIN, NET_RAW]
    sysctls:
      - net.ipv4.ip_forward=1
    networks:
      profinet_net:
        ipv4_address: 192.168.100.12
      codemeter_net:
    restart: unless-stopped
    volumes:
      - ./device-r2-config:/etc/codesyscontrol
      - ./device-r2-data:/var/opt/codesys
      - /var/run:/var/run
```
docker/README.md covers:
- the copy step; ports stay 11741/4841 (unique IPs)
- the per-container CODESYS license/demo mode caveat
- each runtime hosts exactly one PN device with a unique station name

---

## 7. CLI
```
bt2plc generate <INPUT> -o <DIR> [-c FILE] [--flavour codesys|twincat3|tia-scl|plcopen-xml|rockwell-l5x]
       [--source auto|json|xml] [--blackboard FILE] [--grant planned|fcfs] [--job-number word-le|byte]
       [--prod-folge N] [--home-folge N] [--robots ns,ns] [--no-robot-sim] [--no-build-script]
       [--decimate DEG] [--strict] [--force] [--dry-run]
bt2plc check    <INPUT> [-c] [--source] [--strict] [--cross-check] [--json]
bt2plc zones    <INPUT> [-c] [--format text|md|json]
bt2plc simulate <INPUT> [-c] [--seed N] [--runs N] [--jitter F] [--latency-ms N] [--grant …] [--format text|json]
bt2plc config   --print-default
bt2plc flavours
global: -q/--quiet, -v/--verbose
```
- `--source auto` uses the JSON `root` mirror if present, otherwise `xml`.
- `--cross-check` lowers both the JSON root and the XML and requires identical ops, HB edges and zones.
- `--grant fcfs` prints a multi-line warning banner (W011).
- `--dry-run` lists files and writes nothing.
- Diagnostics go to stderr as `warning[W003] /Sequence[0]/Parallel[0]/ExecuteTrajectory[0] (weld_robot1_W01): …`. The summary goes to stdout: robots, ops, zones, makespan, files.

Exit codes:

| Code | Meaning |
|---|---|
| 0 | OK |
| 1 | plan rejected (E-codes, JSON/XML errors) |
| 2 | usage (clap) |
| 3 | config invalid |
| 4 | I/O (non-empty out dir without `--force`, write failure) |
| 5 | flavour not implemented |
| 6 | `--strict` with warnings that are promoted |
| 7 | `simulate`: protocol check failed |

Diagnostic codes:

| Kind | Codes |
|---|---|
| Errors | E001 status≠ok · E002 unsupported node/precondition · E003 Parallel thresholds · E004 robot concurrency · E005 dependency cycle · E006 dangling/non-Execute ref · E007 duplicate (robot, op id) · E008 missing trajectory · E009 >max_zones · E010 bad trajectory · E011 Folge out of range · E013 unresolvable `{key}` · E014 no ops · E015 decorator child count · E017 joint out of INT16 range |
| Warnings promoted by `--strict` | W001 depends_on_joints · W002 depends_on_positions · W003 has_interlock/interlock_target · W004 depends_on_robots · W005 CheckSafety mapped to continuous supervision · W006 multi-op Timeout · W007 inexact Delay · W009 chain gap · W015 home fallback · W016 not starting/ending at home |
| Warnings never promoted | W010 blind MoveHome · W011 FCFS · W012 decimation · W013 idle robot · W014 legacy dialect · W019 num_operations mismatch · W020 time shift · I008 velocity_scale≠1 |

---

## 8. Test plan

**Unit tests (inline, per module).**

| Module | Tests |
|---|---|
| envelope | both samples parse; `BTCPP_format` as string or number; status "failed" → E001; bare tree; missing `behaviour_tree` |
| json_tree | legacy mapping incl. `precomputed_trajectory` and `start_delay_s` → Delay; Fallback/ReactiveSequence/Retry → E002 with exact path |
| xml_tree | the welding `xml` field gives the same NormNode shape as the JSON root; SubTree remap chain; `_skipIf` → E002; JSON-in-attribute `depends_on_ops` |
| blackboard | `{key}`; missing → E008; 5-joint → E010; non-monotonic time → E010; rad→deg; joint mapping |
| bitset | basic operations |
| lower | barrier creation only when both sides > 1; Delay/Timeout attributes; CheckSafety transparent; Parallel `success_count=1` → E003; empty `depends_on_ops` transparent |
| hb | cycle message lists the cycle; SP extension = DFS; same robot in two Parallel branches → E004 |
| reduce | welding gives exactly the 13 edges of §3.9; 2-robot transitive-reduction property; 3-robot pruning (a→b→c drops a→c) |
| zones | welding IL01/IL02 exact; each tie-break rule; E009 at 33 zones; masks |
| schedule | welding makespan 64.2969 s; depths; limits/deadlines equal the GVL_Cell table above |
| config | defaults round-trip; unknown field → error; overlapping bytes; byte mode with Folge 300 → error |
| st_fmt | `T#0MS`, `T#1M3S261MS`, `T#1H…`; `-0.0` → `0.0000`; `$'`/`$$` escaping; comment sanitising |
| tables | CSR invariants |

**`tests/st_lint.rs`** runs over every generated `.st` for both samples and checks:
- no `//`, ASCII only, no tabs
- balanced IF/CASE/FOR/WHILE/VAR blocks; no `END_FUNCTION_BLOCK`/`END_PROGRAM` in split files
- every `E_X.Y` refers to a declared enum value
- every identifier used in the head FBs exists in GVL_Cell/DUTs
- creation order satisfies type dependencies (types referenced by a file are created earlier)

**`tests/protocol_sim.rs`** uses `src/sim`: time-stepped at 1 ms, PLC scan 4 ms, robot cycle 20 ms with random phase, IO latency up/down 1–12 ms seeded per message. FB mirrors are line-by-line from FB_ZoneArbiter and FB_ProgramPlayer, using the same `CellTables` that feed GVL_Cell. Durations are drawn as expected × U(0.3, 2.0) from a seeded SplitMix64. Assertions per run:
1. For every HB pair (u,v), `start(v) ≥ end(u)`. This covers every reduced edge.
2. Mutual exclusion per zone.
3. No deadlock: all robots finish within Σd + 10 s; no fault raised.
4. Equivalence: `|start_sim(x) − ASAP_d(x)| ≤ depth(x)·(L_up+L_down+4+20+2) ms + delay rounding`.
5. Overlap: for every concurrent cross-robot pair whose ASAP_d intervals overlap by more than 2× the slack, the simulated intervals overlap. For the welding sample all 6 Parallel weld pairs and the home pair are asserted explicitly.
6. FCFS negative test: an adversarial duration set on the welding sample shows an HB inversion (e.g. W02 before W13) while mutual exclusion holds and nothing deadlocks.
7. Fault injection: a robot error mid-op produces ERR_RELEASE_INVALID/abort with no re-grant; a step skip produces ERR_TICKET_SKIPPED; a request mask ≠ table gives ERR_REQ_MISMATCH after 200 ms.

**`tests/proptest_trees.rs`.** Generators (2–4 robots):
- (a) random SP trees: Sequence/Parallel/Delay, where Parallel children get disjoint robot subsets;
- (b) coordinated layout: a random global interleaving, `Parallel[Sequence per robot]` with WaitForRobot gates referencing earlier ops of other robots (acyclic by construction), split ids, early `<arm>_HOME`, a final home Parallel or Sequence;
- (c) mutations expected to fail: same robot in two branches → E004, back-reference cycle → E005.

Properties:
- pipeline OK; zones are chains; every reduced edge is adjacent in some zone;
- closure(PO ∪ ticket adjacency) == HB on ops (exactly, no added constraint);
- concurrent ops never share a zone; |zones| ≤ |E|;
- protocol sim properties 1–5 (64 cases);
- determinism: plan twice → byte-identical cell_plan.json;
- JSON → XML (test emitter) → XML path gives identical ops and zones.

**Golden and determinism tests.**
- `tests/golden/welding_cell_2_small/`: ZONES.md, cell_plan.json, project.json, codesys/head/GVL/{GVL_Cell,GVL_IO}.st, PLC_PRG_Body.st, io_mapping.csv, robots/robot1/program.json, codesys/robots/robot1/GVL/GVL_Program.st.
- `tests/golden/cell_2robot/`: same set (1 robot, 0 zones, `*_DIM=1`).
- Generate twice in-process and compare; `tests/cli.rs` runs the binary twice in separate processes (different HashMap seeds) and compares directories byte-for-byte. It also checks exit codes for each class.

---

## 9. Risks and open points (most severe first)

1. **Safety.** Zones are logical ordering mutexes on a standard (non-F) PLC. They are derived from the BT, not from geometry. Collision freedom rests on the planner's validation and on the tables and robot programs matching (mitigated by PLAN_ID and the step, mask and ticket checks). A real cell needs safety-rated workspace monitoring (KUKA SafeOperation, F-PLC). FCFS removes even the ordering guarantee.
2. **CODESYS automation feasibility.**
   - PN device generation from GSDML is manual.
   - MCP servers differ: the toolkit cannot create devices, DUTs, GVLs or tasks, and the Master's exact tool parameters are unknown until listed.
   - The project.json parameter IDs (Ethernet 0/1/2/10/12, controller 130/384/385, KUKA 512 members, device 2, IO 77824/81920) come from the exports and must be read back.
   - The CSV import with an empty IEC address column is unverified.
   - `build_project.py` uses ScriptEngine calls (`projects.create`, `create_pou`, `textual_declaration.replace`, `create_dut`, `create_gvl`, `device_parameters.by_id`, `io_mapping.variable`, `create_task`, `add_library`, `build`) that must be checked against the SP22 docs.
   - A PLCopen-XML emitter (the `plcopen_xml` flavour) would unlock `import_plcopen_xml` for DUTs and GVLs.
3. **Generated ST is not compiled in CI.** The ST lint plus the simulator mirror reduce the risk, but the first IDE compile is the real acceptance gate. Specific items to confirm:
   - nested struct-array initializers with TIME/STRING fields
   - `ARRAY[1..GVL_Cell.N_ROBOTS] OF FB_VassRobot`
   - global constants used as array bounds in a qualified_only GVL
   - CASE with comma labels
4. **Communication loss detection** relies on inputs dropping to zero or freezing (handled fail-closed by F1 plus the watchdogs). A positive `bCommOK` from the implicit `PNSlave` instance (e.g. `kuka_robi1.xRunning`) or CAA Device Diagnosis is an open point. A heartbeat byte would cost zone capacity (bytes 28-31).
5. **Abort and recovery.** A cycle restarts from op 1 after blind, sequential homing. Welds are redone, and blind homing from mid-weld poses can collide in a real cell. Resume-from-op and planned retract paths are open.
6. **Timing assumptions.** Watchdogs use trajectory durations: real speed, the override (watchdogs do not scale with `[1].4` override yet), PROFINET latency and start skew are only absorbed by the margins (f=1.5, m=5 s, m_zone=10 s). The reference executor's +1 s margin is not mirrored (`completion_margin_s=0`).
7. **Joint conventions.** URDF radians are converted to KUKA A1..A6 degrees with identity sign/offset by default; the per-robot `joint_sign`/`joint_offset_deg` must be verified for the KR240. This affects visualisation and AtHome, not ordering.
8. **Zone capacity.** At most 32 zones (one DWORD). Greedy cover is not minimal for ≥3 robots or irregular trees. Optional exact minimisation (Dilworth via matching on the edge poset) is an open point. Third-robot pruning is already on.
9. **Semantic approximations.** Delay after grants is conservative (W007); Timeout is per op (W006); CheckSafety maps to always-on supervision (W005); `depends_on_joints/positions/robots` and `has_interlock` are ignored (W001–W004, errors under `--strict`).
10. **VASS field reuse.** wCurrentStep (4-5), params (4-5 PLAN_ID), user words (12-15) and free bytes (28-31) are repurposed. Real KUKA/VW configurations may use them; offsets are configurable. Job number is WORD-LE by default (groot copy), with `byte` mode for the other copy and the real-robot captures (Folge 49274 = 0xC07A suggests byte 3 is a separate field).
11. **Infrastructure.** Each robot needs its own CODESYS runtime container (license/demo-mode limits, resources). Each container needs a copied config dir.
12. **Coupling to bt-core** through a path dependency outside its workspace; API drift in `Document/Tree/Port`. Mitigation: a thin adapter in `xml_tree.rs` plus the cross-check test.
13. **Device memory.** Constant waypoint tables of about 30 KB per robot in the samples; capped at 4000 per robot by default, with optional decimation.
14. **Open, deferred.** KRL generator (from program.json); TwinCAT 3, TIA SCL, PLCopen XML and L5X backends (the FBs are kept portable: no pointers, REFERENCE or OOP; flat structs; CSR tables); GRANT-as-permit mode on the robot side; HMI/OPC UA tags.

---

### Critical Files for Implementation
- `groot/profinet-headplc-robot-simulation/codesys-project/kuka-vass-device-simulator/src/FB_VASS` (in the behaviour-tree project)