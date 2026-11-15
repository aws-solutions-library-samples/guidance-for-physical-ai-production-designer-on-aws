//! `bt2plc.toml`: cell, protocol, timing, network and per-robot addressing.
//!
//! Every field has a default that reproduces the docker simulation in
//! `profinet-headplc-robot-simulation` (head PLC 192.168.100.10, robot i at .(10+i),
//! station `kuka-robi<i>`). Unknown fields are rejected so typos surface.

use std::net::Ipv4Addr;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::error::{Error, Result};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, clap::ValueEnum)]
#[serde(rename_all = "snake_case")]
pub enum JobNumber {
    /// Folge number as WORD little-endian in bytes 2-3 (deployed simulator).
    WordLe,
    /// Folge number in byte 2, byte 3 reserved (real KUKA VASS captures).
    Byte,
}

/// Robot-side protocol the head PLC talks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, clap::ValueEnum)]
#[serde(rename_all = "kebab-case")]
pub enum Profile {
    /// The device-simulator telegram: step/REQUEST/GRANT/PLAN_ID in free bytes, 4-phase
    /// handshake, blind HOME Folge (docker simulation, plain KSS controllers).
    Sim,
    /// VW KS24 robots: `PRG_NR`/`SRB`/`PRG_NR_REFL`/Folge-end, the standard robot interlocks
    /// A41-56 / A81-96, op index on `P_PktNr_SPS`, no permit bit, no blind homing.
    VwKs24,
}

impl Profile {
    pub fn name(self) -> &'static str {
        match self {
            Profile::Sim => "sim",
            Profile::VwKs24 => "vw-ks24",
        }
    }

    pub fn is_vw(self) -> bool {
        self == Profile::VwKs24
    }
}

/// VW KS24 signal indices the head PLC uses (`samples-config.md`, `vw_usr_s.dat`).
pub mod vw_ks24 {
    /// `O_R_Bereit`: robot ready.
    pub const READY_OUT: u16 = 9;
    /// `I_R_Ant_Ein`: drives on (PLC -> robot).
    pub const DRIVES_IN: u16 = 10;
    /// `O_R_RK100`: safety circuit / E-stop group OK.
    pub const RK100_OUT: u16 = 10;
    /// `O_R_Auto`: automatic mode.
    pub const AUTO_OUT: u16 = 12;
    /// `O_R_Notaus`: E-stop reset.
    pub const ESTOP_OUT: u16 = 13;
    /// `O_R_F_ENDE` / `LPKT`: Folge end.
    pub const FOLGE_END_OUT: u16 = 14;
    /// `O_R_PFO`: robot in home position.
    pub const PF0_OUT: u16 = 15;
    /// `I_R_SRB`: Folge start (PLC -> robot).
    pub const SRB_IN: u16 = 9;
    /// `I_R_Quit_St`: acknowledge a fault (PLC -> robot).
    pub const RESET_IN: u16 = 26;
    /// First `$OUT` of `P_PktNr_SPS` (op index, 16 bits).
    pub const STEP_OUT: u16 = 97;
    /// First `$OUT` of the interlock releases A41-56.
    pub const RELEASE_OUT: u16 = 41;
    /// First `$OUT` of the interlock requests A81-96.
    pub const REQUEST_OUT: u16 = 81;
    /// First `$OUT` of `VW_FEH_BIN_A` (error code, 8 bits).
    pub const ERROR_OUT: u16 = 225;
    /// Maximum number of interlocks per robot.
    pub const MAX_INTERLOCKS: u8 = 16;
    /// Start bytes for PLAN_ID (16 bits: the byte and the next): PLC -> robot bytes 12, 13, 14,
    /// 17, 19, 23, 28, 29, 30 are unused by the VW KS24 standard on all 8 sample robots, and
    /// only these four start a pair of two free bytes (byte 31 carries E249/E250).
    pub const FREE_IN_BYTES: [u8; 4] = [12, 13, 28, 29];
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, clap::ValueEnum)]
#[serde(rename_all = "snake_case")]
pub enum GrantPolicy {
    /// Zones are granted in the planned ticket order (exact BT semantics).
    Planned,
    /// First come, first served (mutual exclusion only; BT order not enforced).
    Fcfs,
}

/// URDF -> KUKA joint mapping preset (`[[robot]] joint_map`).
///
/// The model presets are the inverse of krl-runner's cell data replay map (`cell-data-replay/joint_map.yaml`)
/// (`q_urdf = rad(sign * (a + off))`, so `a = sign * deg(q_urdf) - off`). That calibration is
/// UNVALIDATED (fitted from recordings; KR160 has no data). Identity is the default.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointMapPreset {
    Identity,
    Kr240R2900Ultra,
    Kr210R2700Extra,
    Kr270R2700Ultra,
    Kr160R1570Nano,
}

impl JointMapPreset {
    pub fn name(self) -> &'static str {
        match self {
            JointMapPreset::Identity => "identity",
            JointMapPreset::Kr240R2900Ultra => "kr240_r2900_ultra",
            JointMapPreset::Kr210R2700Extra => "kr210_r2700_extra",
            JointMapPreset::Kr270R2700Ultra => "kr270_r2700_ultra",
            JointMapPreset::Kr160R1570Nano => "kr160_r1570_nano",
        }
    }

    /// `(joint_sign, joint_offset_deg)` for `a_deg = sign * deg(q) + offset`.
    pub fn sign_offset(self) -> ([i8; 6], [f64; 6]) {
        // joint_map.yaml: signs and offsets_deg (KUKA -> URDF); the offset is negated here.
        const S: [i8; 6] = [1, 1, 1, -1, 1, -1];
        let yaml_off: [f64; 6] = match self {
            JointMapPreset::Identity => return ([1; 6], [0.0; 6]),
            JointMapPreset::Kr240R2900Ultra => [0.0, 90.0, -90.0, 0.0, -74.8009, 0.0],
            JointMapPreset::Kr210R2700Extra => [0.0, 90.0, -90.0, 0.0, -90.0, 0.0],
            JointMapPreset::Kr270R2700Ultra => [0.0, 90.0, -90.0, 0.0, -180.0, 0.0],
            JointMapPreset::Kr160R1570Nano => [0.0, 90.0, -90.0, 0.0, 0.0, 0.0],
        };
        let mut off = [0.0; 6];
        for i in 0..6 {
            off[i] = if yaml_off[i] == 0.0 {
                0.0
            } else {
                -yaml_off[i]
            };
        }
        (S, off)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HomeStrategy {
    Sequential,
    Parallel,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
#[derive(Default)]
pub struct Config {
    pub cell: CellCfg,
    pub protocol: ProtocolCfg,
    pub timing: TimingCfg,
    pub network: NetworkCfg,
    pub codesys: CodesysCfg,
    pub robot_sim: RobotSimCfg,
    /// Off by default (and then not serialised, so the plan fingerprint stays as it was).
    #[serde(skip_serializing_if = "OpcuaCfg::is_default")]
    pub opcua: OpcuaCfg,
    #[serde(rename = "robot")]
    pub robots: Vec<RobotCfg>,
}

/// `[opcua]` (`[plc.opcua]` in plan2cell.toml): the head project's OPC UA server content.
///
/// `symbols = true` adds a Symbol Configuration (with OPC UA support) to the head project that
/// exports the cell status read-only (`PLC_PRG` and `FB_Cell`, see
/// `project_json::head_symbol_config`): `project.json` gets `symbol_config` and a step
/// `symbol_config` before `save`. It changes neither the PLC code nor the plan id.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct OpcuaCfg {
    pub symbols: bool,
}

impl OpcuaCfg {
    pub fn is_default(&self) -> bool {
        *self == Self::default()
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct CellCfg {
    pub prod_folge: u16,
    pub home_folge: u16,
    pub job_number: JobNumber,
    pub grant_policy: GrantPolicy,
    pub auto_run: bool,
    pub home_strategy: HomeStrategy,
    pub prune_transitive: bool,
    /// Downgrade depends_on_joints/positions and has_interlock from errors to warnings (they are then ignored).
    pub ignore_unsupported_gates: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ProtocolCfg {
    /// Robot-side protocol (`sim` or `vw-ks24`).
    pub profile: Profile,
    /// Robot -> PLC: WORD LE current op index (wCurrentStep).
    pub step_byte: u8,
    /// Robot -> PLC: DWORD LE zone REQUEST bits.
    pub request_byte: u8,
    /// PLC -> robot: DWORD LE zone GRANT bits.
    pub grant_byte: u8,
    /// PLC -> robot: WORD LE plan id written with the production job.
    pub plan_id_byte: u8,
    pub max_zones: u8,
    pub reset_pulse_ms: u32,
    pub start_pulse_ms: u32,
    pub stop_pulse_ms: u32,
    pub handshake_timeout_ms: u32,
    /// Bring-up (safety enable) limit: covers the PROFINET/PROFIsafe connection set-up.
    pub bringup_limit_ms: u32,
    pub mismatch_debounce_ms: u32,
    /// RS_JOB_RUNNING: ProgRunning and ProgFinished both low this long before
    /// ERR_ROBOT_STOPPED (`T_STOP_DEBOUNCE`). Covers a robot that publishes the two bits in
    /// different bus frames at the Folge end; ProgFinished within it is a normal job end.
    pub stop_debounce_ms: u32,
    pub cycle_pause_ms: u32,
    pub start_limit_ms: u32,
    pub abort_limit_ms: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct TimingCfg {
    pub watchdog_factor: f64,
    pub watchdog_margin_s: f64,
    pub zone_wait_margin_s: f64,
    /// Duration of a blind MoveHome and of the HOME Folge.
    pub home_return_time_s: f64,
    /// Added to every op's expected duration (the reference executor uses 1.0).
    pub completion_margin_s: f64,
    /// Release-to-grant latency budget per dependency hop (zone-wait deadlines).
    pub hop_latency_s: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(from = "NetworkToml")]
pub struct NetworkCfg {
    pub head_ip: Ipv4Addr,
    pub netmask: Ipv4Addr,
    pub gateway: Ipv4Addr,
    pub nic: String,
    pub controller_station: String,
    /// The PN controller's DCP address pool (FirstIPAddress / LastIPAddress). Unset in the
    /// config: [`NetworkCfg::default_pool`] of the head subnet (so it follows `head_ip` /
    /// `netmask`); set, it must lie in the head subnet ([`crate::hardware::check`]).
    pub ip_pool_first: Ipv4Addr,
    pub ip_pool_last: Ipv4Addr,
    /// Robot i (1-based) gets host part `robot_host_base + i`.
    pub robot_host_base: u8,
    pub head_gateway_port: u16,
    pub head_opcua_port: u16,
    pub device_gateway_port: u16,
    pub device_opcua_port: u16,
    pub docker_network: String,
}

/// [`NetworkCfg`] as written in a config: the pool is optional (derived from the head subnet).
#[derive(Deserialize)]
#[serde(default, deny_unknown_fields)]
struct NetworkToml {
    head_ip: Ipv4Addr,
    netmask: Ipv4Addr,
    gateway: Ipv4Addr,
    nic: String,
    controller_station: String,
    ip_pool_first: Option<Ipv4Addr>,
    ip_pool_last: Option<Ipv4Addr>,
    robot_host_base: u8,
    head_gateway_port: u16,
    head_opcua_port: u16,
    device_gateway_port: u16,
    device_opcua_port: u16,
    docker_network: String,
}

impl Default for NetworkToml {
    fn default() -> Self {
        let d = NetworkCfg::default();
        Self {
            head_ip: d.head_ip,
            netmask: d.netmask,
            gateway: d.gateway,
            nic: d.nic,
            controller_station: d.controller_station,
            ip_pool_first: None,
            ip_pool_last: None,
            robot_host_base: d.robot_host_base,
            head_gateway_port: d.head_gateway_port,
            head_opcua_port: d.head_opcua_port,
            device_gateway_port: d.device_gateway_port,
            device_opcua_port: d.device_opcua_port,
            docker_network: d.docker_network,
        }
    }
}

impl From<NetworkToml> for NetworkCfg {
    fn from(t: NetworkToml) -> Self {
        let (first, last) = NetworkCfg::default_pool(t.head_ip, t.netmask);
        Self {
            head_ip: t.head_ip,
            netmask: t.netmask,
            gateway: t.gateway,
            nic: t.nic,
            controller_station: t.controller_station,
            ip_pool_first: t.ip_pool_first.unwrap_or(first),
            ip_pool_last: t.ip_pool_last.unwrap_or(last),
            robot_host_base: t.robot_host_base,
            head_gateway_port: t.head_gateway_port,
            head_opcua_port: t.head_opcua_port,
            device_gateway_port: t.device_gateway_port,
            device_opcua_port: t.device_opcua_port,
            docker_network: t.docker_network,
        }
    }
}

impl NetworkCfg {
    /// The DCP pool of the subnet of `head_ip` / `netmask`: its second host address up to its
    /// last one (192.168.100.2 .. 192.168.100.254 for 192.168.100.10/24).
    pub fn default_pool(head_ip: Ipv4Addr, netmask: Ipv4Addr) -> (Ipv4Addr, Ipv4Addr) {
        let mask = u32::from(netmask);
        let net = u32::from(head_ip) & mask;
        let bcast = net | !mask;
        let last = bcast.saturating_sub(1).max(net);
        let first = net.saturating_add(2).min(last);
        (Ipv4Addr::from(first), Ipv4Addr::from(last))
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct CodesysCfg {
    pub ide_version: String,
    /// `{cell}` is replaced by the cell id.
    pub head_project: String,
    /// `{cell}` and `{tag}` are replaced.
    pub device_project: String,
    pub main_task_ms: u32,
    pub device_main_task_ms: u32,
    pub gsdml_file: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RobotSimCfg {
    pub enabled: bool,
    /// Joint-space decimation tolerance in degrees (0 = keep every waypoint).
    pub decimate_tol_deg: f64,
    pub max_waypoints_per_robot: u32,
    pub at_home_tol_deg: f64,
    pub start_tol_deg: f64,
    pub chain_tol_deg: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RobotCfg {
    pub ns: String,
    pub tag: Option<String>,
    pub station: Option<String>,
    pub device_name: Option<String>,
    pub ip: Option<Ipv4Addr>,
    pub home_pose_deg: Option<[f64; 6]>,
    pub joint_sign: Option<[i8; 6]>,
    pub joint_offset_deg: Option<[f64; 6]>,
    /// Mapping preset instead of `joint_sign`/`joint_offset_deg` (not both).
    pub joint_map: Option<JointMapPreset>,
    /// Profile `vw-ks24`: zone name -> VW interlock index (1..16). Shared with bt2krl, so both
    /// sides use one source of truth. Empty = zones in name order -> 1, 2, ...
    #[serde(default)]
    pub interlocks: std::collections::BTreeMap<String, u8>,
    pub prod_folge: Option<u16>,
    /// Robot model for the KRL generator (soft limits, axis speeds), e.g. `KR240R2900_ULTRA`.
    /// Only written to program.json (default `"unknown"`).
    pub model: Option<String>,
    /// Include the robot even if the BT has no op for it (bring-up and homing only).
    pub include_if_idle: bool,
    /// The PROFINET device is a real or virtual robot controller (e.g. a KUKA VC), not a
    /// CODESYS robot simulator: no `RobotSim_*` project and no docker service for it; the
    /// head keeps its `kuka_robi<i>` device.
    pub external: bool,
    /// SIMULATION ONLY: the head does not require the PROFIsafe slot-1 bits (SafeIn0.0 drives
    /// OK, SafeIn1.4 E-stop OK) of this robot. For a virtual controller whose FDIO64 module
    /// carries no F-data to a non-safety CODESYS controller. Never on a real cell.
    pub safety_bypass: bool,
    /// The head's PN device of this robot has the PROFIsafe module (FDIO64, slot 1). `false`
    /// (SIMULATION ONLY, needs `safety_bypass` and `external`): the KUKA GSDML's DAP `DIM 3`
    /// (V8.3), where FDIO64 is only a default module, with slot 1 left empty and only DIO256 in
    /// slot 2. For a KUKA VC (KSS without the safety multiplexer: it refuses the F-parameters of
    /// slot 1 with "Application libSpIoMultiplexer not connected to this bus").
    pub safety_module: bool,
}

impl Default for RobotCfg {
    fn default() -> Self {
        Self {
            ns: String::new(),
            tag: None,
            station: None,
            device_name: None,
            ip: None,
            home_pose_deg: None,
            joint_sign: None,
            joint_offset_deg: None,
            joint_map: None,
            interlocks: Default::default(),
            prod_folge: None,
            model: None,
            include_if_idle: false,
            external: false,
            safety_bypass: false,
            safety_module: true,
        }
    }
}

impl Default for CellCfg {
    fn default() -> Self {
        Self {
            prod_folge: 11,
            home_folge: 125,
            job_number: JobNumber::WordLe,
            grant_policy: GrantPolicy::Planned,
            auto_run: true,
            home_strategy: HomeStrategy::Sequential,
            prune_transitive: true,
            ignore_unsupported_gates: false,
        }
    }
}

impl Default for ProtocolCfg {
    fn default() -> Self {
        Self {
            profile: Profile::Sim,
            step_byte: 4,
            request_byte: 28,
            grant_byte: 12,
            plan_id_byte: 4,
            max_zones: 31,
            reset_pulse_ms: 1000,
            start_pulse_ms: 500,
            stop_pulse_ms: 1000,
            handshake_timeout_ms: 5000,
            bringup_limit_ms: 60000,
            mismatch_debounce_ms: 200,
            stop_debounce_ms: 200,
            cycle_pause_ms: 2000,
            start_limit_ms: 15000,
            abort_limit_ms: 10000,
        }
    }
}

impl TimingCfg {
    /// Head limit for the HOME Folge (`T_HOME_LIMIT`): `f * home_return_time_s + m`.
    pub fn home_limit_s(&self) -> f64 {
        self.watchdog_factor * self.home_return_time_s + self.watchdog_margin_s
    }
}

impl Default for TimingCfg {
    fn default() -> Self {
        Self {
            watchdog_factor: 1.5,
            watchdog_margin_s: 5.0,
            zone_wait_margin_s: 10.0,
            home_return_time_s: 7.0,
            completion_margin_s: 0.0,
            hop_latency_s: 0.2,
        }
    }
}

impl Default for NetworkCfg {
    fn default() -> Self {
        Self {
            head_ip: Ipv4Addr::new(192, 168, 100, 10),
            netmask: Ipv4Addr::new(255, 255, 255, 0),
            gateway: Ipv4Addr::new(0, 0, 0, 0),
            nic: "eth1".into(),
            controller_station: "controller".into(),
            ip_pool_first: Ipv4Addr::new(192, 168, 100, 2),
            ip_pool_last: Ipv4Addr::new(192, 168, 100, 254),
            robot_host_base: 10,
            head_gateway_port: 11740,
            head_opcua_port: 4840,
            device_gateway_port: 11741,
            device_opcua_port: 4841,
            docker_network: "profinet_net".into(),
        }
    }
}

impl Default for CodesysCfg {
    fn default() -> Self {
        Self {
            ide_version: "CODESYS V3.5 SP22 Patch 1".into(),
            head_project: "HeadPLC_{cell}".into(),
            device_project: "RobotSim_{cell}_{tag}".into(),
            main_task_ms: 4,
            device_main_task_ms: 20,
            gsdml_file: "GSDML-V2.25-KUKA-Roboter-GmbH-VKR C4-Device-V1.0-20121030.xml".into(),
        }
    }
}

impl Default for RobotSimCfg {
    fn default() -> Self {
        Self {
            enabled: true,
            decimate_tol_deg: 0.0,
            max_waypoints_per_robot: 4000,
            at_home_tol_deg: 0.5,
            start_tol_deg: 1.0,
            chain_tol_deg: 0.5,
        }
    }
}

/// Resolved addressing of one robot (config values or simulation defaults).
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct RobotAddressing {
    pub tag: String,
    pub station: String,
    pub device_name: String,
    pub ip: Ipv4Addr,
    pub prod_folge: u16,
    pub joint_sign: [i8; 6],
    pub joint_offset_deg: [f64; 6],
    /// See [`RobotCfg::external`].
    pub external: bool,
    /// See [`RobotCfg::safety_bypass`] (simulation only).
    pub safety_bypass: bool,
    /// See [`RobotCfg::safety_module`] (`false`: simulation only).
    pub safety_module: bool,
    /// `"identity"`, a [`JointMapPreset`] name, or `"custom"` (explicit sign/offset).
    #[serde(skip)]
    pub joint_map: String,
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).map_err(|e| Error::io(path, e))?;
        let cfg: Config = toml::from_str(&text).map_err(|e| Error::Config(e.to_string()))?;
        cfg.validate()?;
        Ok(cfg)
    }

    pub fn default_toml() -> String {
        toml::to_string_pretty(&Config::default()).expect("default config serialises")
    }

    pub fn robot_cfg(&self, ns: &str) -> Option<&RobotCfg> {
        self.robots.iter().find(|r| r.ns == ns)
    }

    /// Addressing for robot `index` (1-based, BT order).
    pub fn addressing(&self, ns: &str, index: usize) -> Result<RobotAddressing> {
        let rc = self.robot_cfg(ns);
        let host = self.network.robot_host_base as usize + index;
        if host > 254 {
            return Err(Error::Config(format!(
                "robot {ns}: default IP host part {host} > 254"
            )));
        }
        let [a, b, c, _] = self.network.head_ip.octets();
        let preset = rc
            .and_then(|r| r.joint_map)
            .unwrap_or(JointMapPreset::Identity);
        Ok(RobotAddressing {
            tag: rc
                .and_then(|r| r.tag.clone())
                .unwrap_or_else(|| format!("R{index}")),
            station: rc
                .and_then(|r| r.station.clone())
                .unwrap_or_else(|| format!("kuka-robi{index}")),
            device_name: rc
                .and_then(|r| r.device_name.clone())
                .unwrap_or_else(|| format!("kuka_robi{index}")),
            ip: rc
                .and_then(|r| r.ip)
                .unwrap_or(Ipv4Addr::new(a, b, c, host as u8)),
            prod_folge: rc
                .and_then(|r| r.prod_folge)
                .unwrap_or(self.cell.prod_folge),
            joint_sign: rc
                .and_then(|r| r.joint_sign)
                .unwrap_or(preset.sign_offset().0),
            joint_offset_deg: rc
                .and_then(|r| r.joint_offset_deg)
                .unwrap_or(preset.sign_offset().1),
            external: rc.is_some_and(|r| r.external),
            safety_bypass: rc.is_some_and(|r| r.safety_bypass),
            safety_module: rc.is_none_or(|r| r.safety_module),
            joint_map: if rc.is_some_and(|r| r.joint_sign.is_some() || r.joint_offset_deg.is_some())
            {
                "custom".into()
            } else {
                preset.name().into()
            },
        })
    }

    pub fn validate(&self) -> Result<()> {
        let c = &self.cell;
        let p = &self.protocol;
        if c.prod_folge == 0 || c.home_folge == 0 {
            return Err(Error::Config("Folge numbers must be > 0".into()));
        }
        if c.prod_folge == c.home_folge {
            return Err(Error::Config(
                "prod_folge and home_folge must differ".into(),
            ));
        }
        let max_folge = match c.job_number {
            JobNumber::Byte => 255,
            JobNumber::WordLe => u16::MAX,
        };
        for f in [c.prod_folge, c.home_folge]
            .into_iter()
            .chain(self.robots.iter().filter_map(|r| r.prod_folge))
        {
            if f > max_folge {
                return Err(Error::Config(format!(
                    "Folge {f} does not fit job_number mode {:?}",
                    c.job_number
                )));
            }
        }
        if p.max_zones == 0 || p.max_zones > 31 {
            return Err(Error::Config(
                "max_zones must be 1..=31 (bit 31 of the DWORD is the permit bit)".into(),
            ));
        }
        if p.profile.is_vw() {
            if p.max_zones > vw_ks24::MAX_INTERLOCKS {
                return Err(Error::Config(format!(
                    "profile vw-ks24: max_zones must be <= {} (the VW standard has that many \
                     robot interlocks, and it has no permit bit)",
                    vw_ks24::MAX_INTERLOCKS
                )));
            }
            if c.job_number != JobNumber::Byte {
                return Err(Error::Config(
                    "profile vw-ks24: job_number must be \"byte\" (PRG_NR is 8 bits, $IN[1..8])"
                        .into(),
                ));
            }
            if c.prod_folge > 255 || c.home_folge > 255 {
                return Err(Error::Config(
                    "profile vw-ks24: Folge numbers must be 1..=255".into(),
                ));
            }
            // The PLAN_ID check needs a telegram byte pair the VW standard does not use: the
            // sim default (4-5) is $IN[33..48] on these robots, i.e. the Fertigmeldung
            // acknowledge and the partner interlock releases E41-48.
            if p.plan_id_byte != 0 && !vw_ks24::FREE_IN_BYTES.contains(&p.plan_id_byte) {
                return Err(Error::Config(format!(
                    "profile vw-ks24: plan_id_byte {} is used by the VW KS24 standard (bytes 4-5 \
                     carry the partner interlock releases E41-48); set 0 (no PLAN_ID check) or \
                     a free PLC -> robot byte: {:?}",
                    p.plan_id_byte,
                    vw_ks24::FREE_IN_BYTES
                )));
            }
            for r in &self.robots {
                for (z, &n) in &r.interlocks {
                    if n == 0 || n > vw_ks24::MAX_INTERLOCKS {
                        return Err(Error::Config(format!(
                            "robot {}: interlock {z} = {n} is outside 1..={}",
                            r.ns,
                            vw_ks24::MAX_INTERLOCKS
                        )));
                    }
                }
                let mut seen = std::collections::BTreeMap::new();
                for (z, &n) in &r.interlocks {
                    if let Some(other) = seen.insert(n, z.clone()) {
                        return Err(Error::Config(format!(
                            "robot {}: interlock {n} is used by {other} and {z}",
                            r.ns
                        )));
                    }
                }
            }
        } else if self.robots.iter().any(|r| !r.interlocks.is_empty()) {
            return Err(Error::Config(
                "[[robot]] interlocks is only used by protocol.profile = \"vw-ks24\"".into(),
            ));
        }
        // Robot -> PLC byte usage: status 0-3, step (2), error 10-11, joints 16-27, request (4).
        // Profile vw-ks24 uses the VW signal map instead (fixed by the KS24 standard).
        if p.profile.is_vw() {
            return self.validate_tail();
        }
        let mut used_in = [None::<&str>; 32];
        claim(&mut used_in, 0, 4, "status/echo")?;
        claim(&mut used_in, 6, 4, "speed/tool")?;
        claim(&mut used_in, 10, 2, "error code")?;
        claim(&mut used_in, 12, 4, "warning/position")?;
        claim(&mut used_in, p.step_byte as usize, 2, "step")?;
        claim(&mut used_in, 16, 12, "joint angles")?;
        claim(&mut used_in, p.request_byte as usize, 4, "zone request")?;
        // PLC -> robot: control/job 0-3, plan id (2), speed 8-9, grant (4).
        let mut used_out = [None::<&str>; 32];
        claim(&mut used_out, 0, 4, "control/job")?;
        claim(&mut used_out, p.plan_id_byte as usize, 2, "plan id")?;
        claim(&mut used_out, 8, 2, "speed")?;
        claim(&mut used_out, p.grant_byte as usize, 4, "zone grant")?;
        self.validate_tail()
    }

    fn validate_tail(&self) -> Result<()> {
        let c = &self.cell;
        let p = &self.protocol;
        if p.reset_pulse_ms < 500 {
            return Err(Error::Config(
                "reset_pulse_ms must be >= 500 (robot needs the reset held 500 ms)".into(),
            ));
        }
        if self.timing.watchdog_factor < 1.0 {
            return Err(Error::Config("watchdog_factor must be >= 1.0".into()));
        }
        let mut seen = std::collections::BTreeSet::new();
        for r in &self.robots {
            if let Some(f) = r.prod_folge
                && (f == 0 || f == c.home_folge)
            {
                return Err(Error::Config(format!(
                    "robot {}: prod_folge {f} must be > 0 and differ from home_folge",
                    r.ns
                )));
            }
            if r.ns.is_empty() {
                return Err(Error::Config("[[robot]] needs ns".into()));
            }
            if r.joint_map.is_some() && (r.joint_sign.is_some() || r.joint_offset_deg.is_some()) {
                return Err(Error::Config(format!(
                    "robot {}: joint_map and joint_sign/joint_offset_deg are mutually exclusive",
                    r.ns
                )));
            }
            if !(r.safety_module || (r.safety_bypass && r.external)) {
                return Err(Error::Config(format!(
                    "robot {}: safety_module = false (simulation only) needs safety_bypass = true \
                     (the head then ignores the PROFIsafe bits) and external = true (a CODESYS \
                     robot simulator's PN device has FDIO64 fixed in slot 1)",
                    r.ns
                )));
            }
            if !seen.insert(r.ns.clone()) {
                return Err(Error::Config(format!("duplicate [[robot]] ns {}", r.ns)));
            }
            if let Some(ip) = r.ip {
                if ip == self.network.head_ip {
                    return Err(Error::Config(format!(
                        "robot {} uses the head PLC IP",
                        r.ns
                    )));
                }
                if !same_subnet(ip, self.network.head_ip, self.network.netmask) {
                    return Err(Error::Config(format!(
                        "robot {} IP {ip} is outside the head subnet",
                        r.ns
                    )));
                }
            }
            if let Some(st) = &r.station
                && let Some(p) = crate::hardware::station_name_problem(st)
            {
                return Err(Error::Config(format!(
                    "robot {}: station name '{st}' {p} (PROFINET NameOfStation)",
                    r.ns
                )));
            }
        }
        if let Some(p) = crate::hardware::station_name_problem(&self.network.controller_station) {
            return Err(Error::Config(format!(
                "network.controller_station '{}' {p} (PROFINET NameOfStation)",
                self.network.controller_station
            )));
        }
        Ok(())
    }
}

fn claim<'a>(
    map: &mut [Option<&'a str>; 32],
    start: usize,
    len: usize,
    what: &'a str,
) -> Result<()> {
    if start + len > 32 {
        return Err(Error::Config(format!(
            "{what} bytes {start}..{} exceed the 32-byte telegram",
            start + len - 1
        )));
    }
    for slot in &mut map[start..start + len] {
        if let Some(other) = slot {
            return Err(Error::Config(format!(
                "{what} overlaps {other} in the VASS telegram"
            )));
        }
        *slot = Some(what);
    }
    Ok(())
}

fn same_subnet(a: Ipv4Addr, b: Ipv4Addr, mask: Ipv4Addr) -> bool {
    let m = u32::from(mask);
    u32::from(a) & m == u32::from(b) & m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_round_trips_and_validates() {
        let text = Config::default_toml();
        let back: Config = toml::from_str(&text).unwrap();
        assert_eq!(back, Config::default());
        back.validate().unwrap();
    }

    #[test]
    fn example_file_equals_the_defaults() {
        let c: Config = toml::from_str(include_str!("../bt2plc.example.toml")).unwrap();
        assert_eq!(c, Config::default());
    }

    #[test]
    fn unknown_field_is_rejected() {
        assert!(toml::from_str::<Config>("[cell]\nprod_folg = 3\n").is_err());
    }

    #[test]
    fn overlapping_bytes_are_rejected() {
        let mut c = Config::default();
        c.protocol.request_byte = 16;
        assert!(c.validate().is_err());
        let mut c = Config::default();
        c.protocol.grant_byte = 8;
        assert!(c.validate().is_err());
    }

    #[test]
    fn byte_mode_limits_folge() {
        let mut c = Config::default();
        c.cell.job_number = JobNumber::Byte;
        c.cell.prod_folge = 300;
        assert!(c.validate().is_err());
    }

    #[test]
    fn vw_profile_constraints() {
        let mut c = Config::default();
        c.protocol.profile = Profile::VwKs24;
        // byte job numbers and at most 16 zones
        assert!(c.validate().is_err());
        c.cell.job_number = JobNumber::Byte;
        c.protocol.max_zones = 31;
        assert!(c.validate().is_err());
        c.protocol.max_zones = 16;
        // PLAN_ID never on a byte the VW standard uses: the sim default 4-5 is E33-48 there
        // (Fertigmeldung acknowledge and the partner releases E41-48).
        assert_eq!(c.protocol.plan_id_byte, 4);
        let e = c.validate().unwrap_err().to_string();
        assert!(e.contains("E41-48"), "{e}");
        c.protocol.plan_id_byte = 14; // free, but byte 15 is not
        assert!(c.validate().is_err());
        c.protocol.plan_id_byte = 12;
        c.validate().unwrap();
        c.protocol.plan_id_byte = 0; // no PLAN_ID check
        c.validate().unwrap();
        // the interlock map is checked and only allowed in this profile
        c.robots.push(RobotCfg {
            ns: "robot1".into(),
            interlocks: [("IL01".to_string(), 1u8), ("IL02".into(), 1)]
                .into_iter()
                .collect(),
            ..Default::default()
        });
        assert!(c.validate().is_err());
        c.robots[0].interlocks.insert("IL02".into(), 11);
        c.validate().unwrap();
        c.robots[0].interlocks.insert("IL03".into(), 17);
        assert!(c.validate().is_err());
        c.robots[0].interlocks.remove("IL03");
        c.protocol.profile = Profile::Sim;
        assert!(c.validate().is_err(), "interlocks must be vw-ks24 only");
    }

    #[test]
    fn joint_map_presets_invert_joint_map_yaml() {
        let (s, o) = JointMapPreset::Kr240R2900Ultra.sign_offset();
        assert_eq!(s, [1, 1, 1, -1, 1, -1]);
        assert_eq!(o, [0.0, -90.0, 90.0, 0.0, 74.8009, 0.0]);
        // a = s*deg(q) + o inverts q = rad(s*(a + yaml_off)).
        let yaml_off = [0.0, 90.0, -90.0, 0.0, -74.8009, 0.0];
        for i in 0..6 {
            let a = 12.5_f64;
            let q = (f64::from(s[i]) * (a + yaml_off[i])).to_radians();
            assert!((f64::from(s[i]) * q.to_degrees() + o[i] - a).abs() < 1e-9);
        }
        let mut c = Config::default();
        c.robots.push(RobotCfg {
            ns: "robot1".into(),
            joint_map: Some(JointMapPreset::Kr240R2900Ultra),
            ..Default::default()
        });
        c.validate().unwrap();
        let a = c.addressing("robot1", 1).unwrap();
        assert_eq!(
            (a.joint_sign, a.joint_map.as_str()),
            (s, "kr240_r2900_ultra")
        );
        c.robots[0].joint_sign = Some([1; 6]);
        assert!(c.validate().is_err());
        assert_eq!(
            Config::default().addressing("r", 1).unwrap().joint_map,
            "identity"
        );
    }

    #[test]
    fn default_addressing_matches_docker_sim() {
        let c = Config::default();
        let a = c.addressing("robot1", 1).unwrap();
        assert_eq!(a.station, "kuka-robi1");
        assert_eq!(a.device_name, "kuka_robi1");
        assert_eq!(a.ip, Ipv4Addr::new(192, 168, 100, 11));
        assert_eq!(a.tag, "R1");
    }
}
