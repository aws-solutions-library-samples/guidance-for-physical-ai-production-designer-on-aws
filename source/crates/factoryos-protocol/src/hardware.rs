//! Hardware Configuration: the PROFINET addressing of the active cell (head PLC network, each
//! robot's station name, device name and IP, and the KUKA virtual controllers' KLI addresses).
//!
//! * `GET` [`crate::api::CELL_HARDWARE`] (`?plan=<plan>`, else the newest plan of the cell):
//!   [`HardwareView`], the effective configuration (the saved one over plan2cell's values for that
//!   plan, else those values) plus [`HardwareView::defaults`] for "Reset to defaults".
//! * `PUT` [`crate::api::CELL_HARDWARE`] with a [`HardwareSave`]: checked with [`HardwareConfig::validate`]
//!   and by plan2cell itself; `422` [`HardwareRejected`] with every problem, else saved as
//!   `<cell>/plan2cell/hardware.json` and answered with the new [`HardwareView`].
//!
//! Every Control Code Export then uses it (`[plc.network]` and `[[plc.robot]]` of the build's
//! plan2cell.toml). The rules are bt2plc's (`bt2plc::hardware::check` and its device-name check),
//! repeated here so a form can show them inline; the backend checks again with plan2cell.

use std::collections::BTreeMap;
use std::net::Ipv4Addr;

use serde::{Deserialize, Serialize};

/// Longest PROFINET NameOfStation (IEC 61158-6-10).
pub const STATION_MAX_LEN: usize = 240;
/// Longest label of a NameOfStation.
pub const STATION_LABEL_MAX_LEN: usize = 63;
/// Longest head-project device name (an IEC identifier).
pub const DEVICE_NAME_MAX_LEN: usize = 64;

/// Shown with the form: what changing the addresses does and does not do.
pub const VIRTUAL_CELL_NOTE: &str = "The virtual cell is wired for the default addresses \
(head 192.168.100.10, robots .11/.12, stations kuka-robi1/kuka-robi2). Other values change the generated \
code (and the cell.toml the backend renders), but the virtual cell itself is not re-addressed.";

/// The KUKA VCs' KLI gateway on a plan2cell virtual cell (the PLC host's PROFINET bridge): the
/// head's `[plc.network] gateway` there. With another (or none) the head sends DCP Set IP to the
/// controllers.
pub const VIRTUAL_CELL_GATEWAY: &str = "192.168.100.1";

/// `[plc.network]`: the head PLC's segment.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct NetworkConfig {
    pub head_ip: String,
    pub netmask: String,
    /// `0.0.0.0` (or empty) = none.
    pub gateway: String,
    /// The head PLC's own PROFINET station name.
    pub controller_station: String,
}

/// One robot of the plan: `[[plc.robot]]` station / device_name / ip / external / safety_bypass /
/// safety_module; the rest is shown only.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct RobotHardware {
    pub ns: String,
    /// `R1`, ... (read-only).
    pub tag: String,
    /// PROFINET NameOfStation.
    pub station: String,
    /// The head project's PN device (an IEC identifier).
    pub device_name: String,
    pub ip: String,
    /// Read-only: `[[plc.robot]]` / `[[krl.robot]]` model.
    pub model: String,
    /// A real or virtual robot controller (KUKA VC), not a CODESYS simulator.
    pub external: bool,
    /// SIMULATION ONLY: the head does not require the PROFIsafe bits of this robot.
    pub safety_bypass: bool,
    /// The head's PN device of this robot has the PROFIsafe module (FDIO64, slot 1); `false`
    /// (simulation only, needs `safety_bypass` and `external`): DAP `DIM 3`, slot 1 empty, as a
    /// KUKA VC needs (it has no safety slot).
    pub safety_module: bool,
}

/// bt2plc's defaults: a CODESYS-simulated robot with the safety module (an older client or
/// file without the flags reads as that).
impl Default for RobotHardware {
    fn default() -> Self {
        Self {
            ns: String::new(),
            tag: String::new(),
            station: String::new(),
            device_name: String::new(),
            ip: String::new(),
            model: String::new(),
            external: false,
            safety_bypass: false,
            safety_module: true,
        }
    }
}

/// A KUKA virtual controller of the cell: the KLI address and station it answers on (the
/// PLC addresses the robot there, so by default the robot's IP and station).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct KukaVcController {
    pub robot_ns: String,
    pub kli_ip: String,
    pub station: String,
}

/// The hardware configuration of a cell, as the form edits it and `hardware.json` keeps it.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct HardwareConfig {
    pub network: NetworkConfig,
    pub robots: Vec<RobotHardware>,
    pub kuka_vc: Vec<KukaVcController>,
}

/// One problem (or warning) with a field: `network.headIp`, `network.netmask`, `network.gateway`,
/// `network.controllerStation`, `robots.<ns>.station|deviceName|ip`, `kukaVc.<ns>.kliIp|station`
/// (the names of `plan2cell hardware --json`), or `config` for the whole.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct FieldIssue {
    pub field: String,
    pub message: String,
}

impl FieldIssue {
    pub fn new(field: impl Into<String>, message: impl Into<String>) -> Self {
        Self { field: field.into(), message: message.into() }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HardwareSource {
    /// Nothing saved: plan2cell's values for the cell's plan2cell.toml.
    #[default]
    Defaults,
    /// `<cell>/plan2cell/hardware.json`.
    Saved,
}

impl HardwareSource {
    pub fn label(self) -> &'static str {
        match self {
            Self::Defaults => "defaults",
            Self::Saved => "saved",
        }
    }
}

/// How the connected virtual cell is wired (the server's `/etc/plan2cell/cell.toml` and
/// plan2cell's default head address), to warn about values that differ.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CellWiring {
    /// Where it was read from.
    pub source: String,
    pub head_ip: String,
    pub netmask: String,
    pub controllers: Vec<KukaVcController>,
}

/// `GET /api/v1/cell/hardware`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct HardwareView {
    pub cell: String,
    /// The plan whose robots are listed ([`crate::PlanInfo::path`]).
    pub plan: String,
    pub source: HardwareSource,
    /// When `hardware.json` was saved (Unix seconds).
    pub saved_s: Option<f64>,
    /// `hardware.json`'s ETag (as a build's [`crate::codegen::BuildInfo::hardware_etag`]), if saved.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub etag: Option<String>,
    /// The effective configuration: saved values over [`Self::defaults`].
    pub config: HardwareConfig,
    /// plan2cell's values for this plan and the cell's plan2cell.toml, without the saved ones.
    pub defaults: HardwareConfig,
    /// [`HardwareConfig::validate`] of [`Self::config`] (a saved file can predate a rule).
    pub errors: Vec<FieldIssue>,
    /// [`HardwareConfig::wiring_warnings`] of [`Self::config`].
    pub warnings: Vec<FieldIssue>,
    /// The connected virtual cell's wiring, if the server has one (`/etc/plan2cell/cell.toml`).
    pub wiring: Option<CellWiring>,
    /// The `plan2cell cell` config rendered from it (`<cell>/plan2cell/cell.toml`), if any.
    pub cell_config: Option<String>,
    /// The robots of the plan that run on the server's plan2cell virtual cell (KUKA VCs): their
    /// defaults are [`HardwareConfig::with_virtual_cell_values`]'s, and
    /// [`HardwareConfig::virtual_cell_warnings`] names values that will not work there. Empty: no
    /// virtual cell (real hardware), plan2cell's own defaults.
    pub virtual_cell_robots: Vec<String>,
    /// [`VIRTUAL_CELL_NOTE`].
    pub note: String,
}

/// `PUT /api/v1/cell/hardware`.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct HardwareSave {
    /// The plan to check it against (as in `GET`); empty: the newest.
    pub plan: String,
    pub config: HardwareConfig,
}

/// The `422` answer to a [`HardwareSave`].
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct HardwareRejected {
    pub errors: Vec<FieldIssue>,
}

impl HardwareConfig {
    pub fn robot(&self, ns: &str) -> Option<&RobotHardware> {
        self.robots.iter().find(|r| r.ns == ns)
    }

    pub fn controller(&self, ns: &str) -> Option<&KukaVcController> {
        self.kuka_vc.iter().find(|c| c.robot_ns == ns)
    }

    /// The gateway as plan2cell writes it: `0.0.0.0` for none (an empty field).
    pub fn gateway(&self) -> String {
        let gateway = self.network.gateway.trim();
        if gateway.is_empty() { "0.0.0.0".to_owned() } else { gateway.to_owned() }
    }

    /// Every rule broken (empty = valid), bt2plc's: the netmask is contiguous (/1 to /30); the
    /// head IP is a host address of its subnet; the gateway (`0.0.0.0` = none) a host address of
    /// it that is not the head; every station name (the head's and each robot's) a valid
    /// NameOfStation, all unique; device names IEC identifiers, unique ignoring case; robot IPs
    /// host addresses of the head subnet, not the head nor the gateway, unique. KUKA VC
    /// controllers: their robot exists, a valid KLI IP in the subnet and station name, unique.
    pub fn validate(&self) -> Vec<FieldIssue> {
        let mut out = Vec::new();
        let mut issue = |field: String, message: String| out.push(FieldIssue { field, message });
        let n = &self.network;
        let ip = |field: &str, value: &str, what: &str, out: &mut dyn FnMut(String, String)| -> Option<Ipv4Addr> {
            match value.trim().parse::<Ipv4Addr>() {
                Ok(ip) => Some(ip),
                Err(_) => {
                    out(field.to_owned(), format!("{what} '{value}' is not an IPv4 address"));
                    None
                }
            }
        };
        let head = ip("network.headIp", &n.head_ip, "head IP", &mut issue);
        let mask = ip("network.netmask", &n.netmask, "netmask", &mut issue);
        let gateway = ip("network.gateway", &self.gateway(), "gateway", &mut issue);
        if let Some(problem) = station_name_problem(&n.controller_station) {
            issue("network.controllerStation".into(), format!("station name '{}' {problem}", n.controller_station));
        }
        let subnet = match (head, mask) {
            (Some(head), Some(mask)) => {
                let m = u32::from(mask);
                if m.leading_ones() + m.trailing_zeros() != 32 || m.leading_ones() > 30 || m == 0 {
                    issue("network.netmask".into(), format!("netmask {mask} must be a contiguous mask from /1 to /30"));
                    None
                } else {
                    Some(Subnet { net: u32::from(head) & m, mask: m })
                }
            }
            _ => None,
        };
        let has_gw = gateway.is_some_and(|g| !g.is_unspecified());
        if let Some(s) = &subnet {
            if let Some(p) = head.and_then(|h| s.host_problem(h)) {
                issue("network.headIp".into(), format!("head IP {p}"));
            }
            if let Some(g) = gateway.filter(|_| has_gw) {
                if let Some(p) = s.host_problem(g) {
                    issue("network.gateway".into(), format!("gateway {p}"));
                } else if Some(g) == head {
                    issue("network.gateway".into(), format!("gateway {g} is the head IP"));
                }
            }
        }

        let mut stations: BTreeMap<&str, String> = BTreeMap::new();
        stations.insert(n.controller_station.as_str(), "the head PLC".into());
        let mut devices: BTreeMap<String, String> = BTreeMap::new();
        let mut ips: BTreeMap<Ipv4Addr, String> = BTreeMap::new();
        for r in &self.robots {
            let f = |k: &str| format!("robots.{}.{k}", r.ns);
            if let Some(p) = station_name_problem(&r.station) {
                issue(f("station"), format!("station name '{}' {p}", r.station));
            }
            if let Some(other) = stations.insert(r.station.as_str(), format!("robot {}", r.ns)) {
                issue(f("station"), format!("station name '{}' is also used by {other}", r.station));
            }
            if let Some(p) = device_name_problem(&r.device_name) {
                issue(f("deviceName"), format!("device name '{}' {p}", r.device_name));
            }
            if let Some(other) = devices.insert(r.device_name.to_ascii_lowercase(), format!("robot {}", r.ns)) {
                issue(f("deviceName"), format!("device name '{}' is also used by {other}", r.device_name));
            }
            let Some(addr) = ip(&f("ip"), &r.ip, "IP", &mut issue) else { continue };
            if let Some(p) = subnet.as_ref().and_then(|s| s.host_problem(addr)) {
                issue(f("ip"), format!("IP {p}"));
            } else if Some(addr) == head {
                issue(f("ip"), format!("IP {addr} is the head PLC IP"));
            } else if has_gw && Some(addr) == gateway {
                issue(f("ip"), format!("IP {addr} is the gateway"));
            }
            if let Some(other) = ips.insert(addr, format!("robot {}", r.ns)) {
                issue(f("ip"), format!("IP {addr} is also used by {other}"));
            }
        }

        for r in &self.robots {
            if !(r.safety_module || (r.safety_bypass && r.external)) {
                issue(
                    format!("robots.{}.safetyModule", r.ns),
                    "no safety module (simulation only) needs External and Safety bypass on \
                     (a CODESYS robot simulator's PN device has FDIO64 fixed in slot 1)"
                        .into(),
                );
            }
        }

        let mut kli: BTreeMap<Ipv4Addr, &str> = BTreeMap::new();
        let mut kli_stations: BTreeMap<&str, &str> = BTreeMap::new();
        for c in &self.kuka_vc {
            let f = |k: &str| format!("kukaVc.{}.{k}", c.robot_ns);
            if self.robot(&c.robot_ns).is_none() {
                issue(f("kliIp"), format!("no robot {} in the plan", c.robot_ns));
                continue;
            }
            if let Some(p) = station_name_problem(&c.station) {
                issue(f("station"), format!("station name '{}' {p}", c.station));
            }
            if let Some(other) = kli_stations.insert(c.station.as_str(), c.robot_ns.as_str()) {
                issue(f("station"), format!("station name '{}' is also used by the controller of {other}", c.station));
            }
            let Some(addr) = ip(&f("kliIp"), &c.kli_ip, "KLI IP", &mut issue) else { continue };
            if let Some(p) = subnet.as_ref().and_then(|s| s.host_problem(addr)) {
                issue(f("kliIp"), format!("KLI IP {p}"));
            } else if Some(addr) == head {
                issue(f("kliIp"), format!("KLI IP {addr} is the head PLC IP"));
            }
            if let Some(other) = kli.insert(addr, c.robot_ns.as_str()) {
                issue(f("kliIp"), format!("KLI IP {addr} is also used by the controller of {other}"));
            }
        }
        out
    }

    /// What will not work as saved, but does not block saving: a KUKA VC controller whose KLI IP
    /// or station differs from its robot's (the PLC addresses the robot there), and every value
    /// that differs from the connected virtual cell's `wiring`.
    pub fn wiring_warnings(&self, wiring: Option<&CellWiring>) -> Vec<FieldIssue> {
        let mut out = Vec::new();
        for c in &self.kuka_vc {
            let Some(r) = self.robot(&c.robot_ns) else { continue };
            if c.kli_ip.trim() != r.ip.trim() {
                out.push(FieldIssue::new(
                    format!("kukaVc.{}.kliIp", c.robot_ns),
                    format!("KLI IP {} differs from robot {}'s IP {}: the head PLC addresses the robot at {}", c.kli_ip, r.ns, r.ip, r.ip),
                ));
            }
            if c.station != r.station {
                out.push(FieldIssue::new(
                    format!("kukaVc.{}.station", c.robot_ns),
                    format!("station {} differs from robot {}'s station {}", c.station, r.ns, r.station),
                ));
            }
        }
        let Some(w) = wiring else { return out };
        let cell = "the virtual cell";
        if !w.head_ip.is_empty() && self.network.head_ip.trim() != w.head_ip {
            out.push(FieldIssue::new("network.headIp", format!("{cell}'s head PLC is wired at {}", w.head_ip)));
        }
        if !w.netmask.is_empty() && self.network.netmask.trim() != w.netmask {
            out.push(FieldIssue::new("network.netmask", format!("{cell}'s segment uses netmask {}", w.netmask)));
        }
        for wc in &w.controllers {
            if let Some(r) = self.robot(&wc.robot_ns) {
                if r.ip.trim() != wc.kli_ip {
                    out.push(FieldIssue::new(
                        format!("robots.{}.ip", r.ns),
                        format!("{cell}'s controller of {} answers at {}", r.ns, wc.kli_ip),
                    ));
                }
                if r.station != wc.station {
                    out.push(FieldIssue::new(
                        format!("robots.{}.station", r.ns),
                        format!("{cell}'s controller of {} is station {}", r.ns, wc.station),
                    ));
                }
            }
            if let Some(c) = self.controller(&wc.robot_ns) {
                if c.kli_ip.trim() != wc.kli_ip {
                    out.push(FieldIssue::new(
                        format!("kukaVc.{}.kliIp", c.robot_ns),
                        format!("{cell}'s controller of {} has KLI {} (the cell is not re-addressed)", c.robot_ns, wc.kli_ip),
                    ));
                }
                if c.station != wc.station {
                    out.push(FieldIssue::new(
                        format!("kukaVc.{}.station", c.robot_ns),
                        format!("{cell}'s controller of {} is station {}", c.robot_ns, wc.station),
                    ));
                }
            }
        }
        out
    }
}

impl HardwareConfig {
    /// The values a plan2cell virtual cell needs for `robots` (its KUKA VCs): External on, Safety
    /// bypass on, no safety module (the VCs' GSDML DAP `DIM 3` has no safety slot), and the head's
    /// gateway [`VIRTUAL_CELL_GATEWAY`]. The rest stays.
    pub fn with_virtual_cell_values(&self, robots: &[String]) -> HardwareConfig {
        let mut out = self.clone();
        if robots.is_empty() {
            return out;
        }
        out.network.gateway = VIRTUAL_CELL_GATEWAY.to_owned();
        for r in out.robots.iter_mut().filter(|r| robots.contains(&r.ns)) {
            r.external = true;
            r.safety_bypass = true;
            r.safety_module = false;
        }
        out
    }

    /// What will not work on a plan2cell virtual cell whose KUKA VCs are `robots` (empty: none):
    /// a safety module (Start production times out), a robot not External or without the safety
    /// bypass, and another head gateway than [`VIRTUAL_CELL_GATEWAY`].
    /// [`Self::with_virtual_cell_values`] fixes them all.
    pub fn virtual_cell_warnings(&self, robots: &[String]) -> Vec<FieldIssue> {
        let mut out = Vec::new();
        if robots.is_empty() {
            return out;
        }
        for r in self.robots.iter().filter(|r| robots.contains(&r.ns)) {
            if r.safety_module {
                out.push(FieldIssue::new(
                    format!("robots.{}.safetyModule", r.ns),
                    "Safety module on, but the virtual cell's controllers have no safety slot: \
                     Start production will time out (ERR_SAFETY_TIMEOUT)",
                ));
            }
            if !r.external {
                out.push(FieldIssue::new(
                    format!("robots.{}.external", r.ns),
                    "External off, but this robot is a KUKA VC of the virtual cell (not a CODESYS robot simulator)",
                ));
            }
            if !r.safety_bypass {
                out.push(FieldIssue::new(
                    format!("robots.{}.safetyBypass", r.ns),
                    "Safety bypass off: the head waits for PROFIsafe bits the virtual cell's controllers never send \
                     (ERR_SAFETY_TIMEOUT)",
                ));
            }
        }
        if self.gateway() != VIRTUAL_CELL_GATEWAY {
            out.push(FieldIssue::new(
                "network.gateway",
                format!(
                    "the virtual cell's KUKA VCs use gateway {VIRTUAL_CELL_GATEWAY} (here: {}): with another, the head \
                     re-addresses them (DCP Set IP)",
                    if self.gateway() == "0.0.0.0" { "none".to_owned() } else { self.gateway() }
                ),
            ));
        }
        out
    }
}

struct Subnet {
    net: u32,
    mask: u32,
}

impl Subnet {
    fn host_problem(&self, ip: Ipv4Addr) -> Option<String> {
        let v = u32::from(ip);
        let subnet = format!("{}/{}", Ipv4Addr::from(self.net), self.mask.leading_ones());
        if v & self.mask != self.net {
            Some(format!("{ip} is outside the head subnet {subnet}"))
        } else if v == self.net {
            Some(format!("{ip} is the network address of {subnet}"))
        } else if v == self.net | !self.mask {
            Some(format!("{ip} is the broadcast address of {subnet}"))
        } else {
            None
        }
    }
}

/// Why `name` is not a valid PROFINET NameOfStation, or `None` (bt2plc's
/// `hardware::station_name_problem`): 1 to 240 characters, labels of 1 to 63 separated by `.`, only
/// `a-z`, `0-9` and `-`, no label starting or ending with `-`, not shaped like an IPv4 address, not
/// `port-xyz` / `port-xyz-abcde`.
pub fn station_name_problem(name: &str) -> Option<String> {
    if name.is_empty() {
        return Some("is empty".into());
    }
    if name.len() > STATION_MAX_LEN {
        return Some(format!("is {} characters long (at most {STATION_MAX_LEN})", name.len()));
    }
    if let Some(c) = name.chars().find(|c| !(c.is_ascii_lowercase() || c.is_ascii_digit() || *c == '-' || *c == '.')) {
        return Some(format!("contains '{c}' (only lowercase letters a-z, digits, '-' and '.' are allowed)"));
    }
    let labels: Vec<&str> = name.split('.').collect();
    for l in &labels {
        if l.is_empty() {
            return Some("has an empty label (a leading, trailing or double '.')".into());
        }
        if l.len() > STATION_LABEL_MAX_LEN {
            return Some(format!("has a label of {} characters ('{l}'; at most {STATION_LABEL_MAX_LEN})", l.len()));
        }
        if l.starts_with('-') || l.ends_with('-') {
            return Some(format!("has a label that starts or ends with '-' ('{l}')"));
        }
    }
    if labels.len() == 4 && labels.iter().all(|l| l.len() <= 3 && l.bytes().all(|b| b.is_ascii_digit())) {
        return Some("has the form of an IPv4 address (n.n.n.n)".into());
    }
    let digits = |s: &str, n: usize| s.len() == n && s.bytes().all(|b| b.is_ascii_digit());
    if let Some(rest) = labels[0].strip_prefix("port-") {
        let reserved = match rest.split_once('-') {
            None => digits(rest, 3),
            Some((a, b)) => digits(a, 3) && digits(b, 5),
        };
        if reserved {
            return Some(format!("starts with '{}' (port-xyz / port-xyz-abcde are reserved for port names)", labels[0]));
        }
    }
    None
}

/// Names a robot's device name must not have (case-insensitive): the generated CODESYS projects'
/// own objects and IEC words. A copy of plan2cell's `bt2plc::hardware::RESERVED_DEVICE_NAMES`
/// (a test compares them), so the form shows what the server's check would reject.
#[rustfmt::skip]
pub const RESERVED_DEVICE_NAMES: &[&str] = &[
    // the device tree, tasks, libraries and generated GVLs/modules of the generated projects
    "Device", "Application", "Ethernet", "PN_Controller", "Library_Manager",
    "Task_Configuration", "MainTask", "Profinet_CommunicationTask", "Profinet_IOTask",
    "GVL_Cell", "GVL_Program", "SafeIO_FDIO64", "VASS_DIO256", "Standard", "IoStandard",
    "IoDrvEthernet", "IoDrvProfinet", "IoDrvProfinetDevice", "ProfinetCommon", "CommFB",
    // the ST templates' objects (templates/codesys/{head,device})
    "E_PlayerState", "E_ProgramState", "E_RobotMode", "ST_OpDef", "ST_PROFIsafe_Trailer",
    "ST_SafeInput", "ST_SafeOutput", "ST_VASS_ControlWord", "ST_VASS_InputTelegram",
    "ST_VASS_OutputTelegram", "ST_VASS_StatusWord", "ST_Waypoint", "GVL_DeviceParams",
    "GVL_IO", "FB_JointAngleEncoder", "FB_PROFIsafe_Simulator", "FB_ProgramPlayer",
    "FB_VASS_DeviceSimulator", "PLC_PRG", "E_CellError", "E_CellState", "E_GrantPolicy",
    "E_RobotState", "ST_OpInfo", "ST_RobotIn", "ST_RobotOut", "ST_Ticket", "FB_Cell",
    "FB_VassRobot", "FB_ZoneArbiter",
    // IEC 61131-3 / CODESYS keywords
    "ABSTRACT", "ACTION", "END_ACTION", "AND", "AND_THEN", "ARRAY", "AT", "BY", "CASE",
    "END_CASE", "CONFIGURATION", "END_CONFIGURATION", "CONSTANT", "CONTINUE", "DO", "ELSE",
    "ELSIF", "EN", "ENO", "EXIT", "EXTENDS", "FALSE", "FINAL", "FOR", "END_FOR", "FUNCTION",
    "END_FUNCTION", "FUNCTION_BLOCK", "END_FUNCTION_BLOCK", "IF", "END_IF", "IMPLEMENTS",
    "INTERFACE", "END_INTERFACE", "INTERNAL", "JMP", "METHOD", "END_METHOD", "MOD", "NOT",
    "NULL", "OF", "OR", "OR_ELSE", "OVERLAP", "PERSISTENT", "POINTER", "PRIVATE", "PROGRAM",
    "END_PROGRAM", "PROPERTY", "END_PROPERTY", "PROTECTED", "PUBLIC", "READ_ONLY",
    "READ_WRITE", "REF", "REFERENCE", "REF_TO", "REPEAT", "END_REPEAT", "RESOURCE",
    "END_RESOURCE", "RETAIN", "RETURN", "STEP", "END_STEP", "STRUCT", "END_STRUCT", "SUPER",
    "TASK", "THEN", "THIS", "TO", "TRANSITION", "END_TRANSITION", "TRUE", "TYPE", "END_TYPE",
    "UNION", "END_UNION", "UNTIL", "VAR", "VAR_ACCESS", "VAR_CONFIG", "VAR_EXTERNAL",
    "VAR_GLOBAL", "VAR_IN_OUT", "VAR_INPUT", "VAR_INST", "VAR_OUTPUT", "VAR_STAT", "VAR_TEMP",
    "END_VAR", "WHILE", "END_WHILE", "WITH", "XOR",
    // elementary and generic data types
    "ANY", "ANY_BIT", "ANY_DATE", "ANY_DERIVED", "ANY_ELEMENTARY", "ANY_INT", "ANY_MAGNITUDE",
    "ANY_NUM", "ANY_REAL", "ANY_STRING", "BIT", "BOOL", "BYTE", "CHAR", "DATE",
    "DATE_AND_TIME", "DINT", "DT", "DWORD", "INT", "LDATE", "LDATE_AND_TIME", "LDT", "LINT",
    "LREAL", "LTIME", "LTIME_OF_DAY", "LTOD", "LWORD", "REAL", "SINT", "STRING", "TIME",
    "TIME_OF_DAY", "TOD", "UDINT", "UINT", "ULINT", "USINT", "WCHAR", "WORD", "WSTRING",
    // standard function blocks and operators
    "TON", "TOF", "TP", "RTC", "R_TRIG", "F_TRIG", "CTU", "CTD", "CTUD", "RS", "SR", "SEMA",
    "ABS", "SQRT", "LN", "LOG", "EXP", "SIN", "COS", "TAN", "ASIN", "ACOS", "ATAN", "EXPT",
    "ADD", "SUB", "MUL", "DIV", "MOVE", "SEL", "MAX", "MIN", "LIMIT", "MUX", "SHL", "SHR",
    "ROL", "ROR", "GT", "GE", "EQ", "LE", "LT", "NE", "SIZEOF", "ADR", "BITADR", "INDEXOF",
    "TRUNC", "LEN", "LEFT", "RIGHT", "MID", "CONCAT", "INSERT", "DELETE", "REPLACE", "FIND",
];

/// Prefixes of the per-robot module names (`bt2plc::hardware::RESERVED_DEVICE_NAME_PREFIXES`).
pub const RESERVED_DEVICE_NAME_PREFIXES: &[&str] = &["SafeIO_FDIO64_", "VASS_DIO256_"];

/// Why `name` is not a valid robot device name, or `None` (plan2cell's rules): an IEC identifier
/// (a letter, then letters, digits and `_`, at most 64 characters) without `__`, not ending in
/// `_`, and none of [`RESERVED_DEVICE_NAMES`] / [`RESERVED_DEVICE_NAME_PREFIXES`].
pub fn device_name_problem(name: &str) -> Option<String> {
    if name.is_empty() {
        return Some("is empty".into());
    }
    if name.len() > DEVICE_NAME_MAX_LEN {
        return Some(format!("is {} characters long (at most {DEVICE_NAME_MAX_LEN})", name.len()));
    }
    if !name.as_bytes()[0].is_ascii_alphabetic() || !name.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_') {
        return Some("must be an IEC identifier (a letter, then letters, digits and '_')".into());
    }
    if name.contains("__") || name.ends_with('_') {
        return Some("must not contain '__' or end with '_' (reserved by CODESYS)".into());
    }
    if let Some(r) = RESERVED_DEVICE_NAMES.iter().find(|r| r.eq_ignore_ascii_case(name)) {
        return Some(format!("is reserved ('{r}' is a name of the generated project or an IEC word)"));
    }
    let upper = name.to_ascii_uppercase();
    if let Some(p) = RESERVED_DEVICE_NAME_PREFIXES.iter().find(|p| upper.starts_with(&p.to_ascii_uppercase())) {
        return Some(format!("starts with '{p}' (reserved for the robots' module names)"));
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell() -> HardwareConfig {
        let robot = |i: u8| RobotHardware {
            ns: format!("robot{i}"),
            tag: format!("R{i}"),
            station: format!("kuka-robi{i}"),
            device_name: format!("kuka_robi{i}"),
            ip: format!("192.168.100.{}", 10 + i),
            model: "KR240R2900_ULTRA".into(),
            external: true,
            ..Default::default()
        };
        let robots = vec![robot(1), robot(2)];
        let kuka_vc = robots
            .iter()
            .map(|r| KukaVcController { robot_ns: r.ns.clone(), kli_ip: r.ip.clone(), station: r.station.clone() })
            .collect();
        HardwareConfig {
            network: NetworkConfig {
                head_ip: "192.168.100.10".into(),
                netmask: "255.255.255.0".into(),
                gateway: "0.0.0.0".into(),
                controller_station: "controller".into(),
            },
            robots,
            kuka_vc,
        }
    }

    fn fields(issues: &[FieldIssue]) -> Vec<&str> {
        issues.iter().map(|i| i.field.as_str()).collect()
    }

    #[test]
    fn the_default_cell_is_valid() {
        let c = cell();
        assert_eq!(c.validate(), vec![]);
        assert_eq!(c.wiring_warnings(None), vec![]);
        let mut c = c;
        c.network.gateway = String::new();
        assert_eq!(c.gateway(), "0.0.0.0");
        assert_eq!(c.validate(), vec![]);
    }

    #[test]
    fn station_and_device_names() {
        for ok in ["kuka-robi1", "controller", "plc.cell-1.line2", "port-01", "1.2.3"] {
            assert_eq!(station_name_problem(ok), None, "{ok}");
        }
        for bad in ["", "Kuka", "kuka_robi1", "-kuka", "a..b", "192.168.100.11", "port-001", "port-001-00001"] {
            assert!(station_name_problem(bad).is_some(), "{bad}");
        }
        assert_eq!(device_name_problem("kuka_robi1"), None);
        for bad in ["", "1kuka", "kuka-robi", "a b", "PLC_PRG", "pn_controller", "IF", "TON", "VASS_DIO256_R1", "a__b", "x_"] {
            assert!(device_name_problem(bad).is_some(), "{bad}");
        }
    }

    /// The same lists as plan2cell's (bt2plc's hardware.rs, in this repository).
    #[test]
    fn reserved_device_names_match_plan2cell() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../plan2cell/crates/bt2plc/src/hardware.rs");
        let Ok(source) = std::fs::read_to_string(&path) else {
            eprintln!("skipped: no {}", path.display());
            return;
        };
        let list = |name: &str| -> Vec<String> {
            let start = source.find(&format!("pub const {name}: &[&str] = &[")).unwrap_or_else(|| panic!("{name} not in bt2plc"));
            let body = &source[start..];
            let body = &body[body.find("= &[").unwrap() + 4..body.find("];").unwrap()];
            body.lines()
                .filter(|l| !l.trim_start().starts_with("//"))
                .flat_map(|l| l.split('"').skip(1).step_by(2).map(str::to_owned).collect::<Vec<_>>())
                .collect()
        };
        assert_eq!(list("RESERVED_DEVICE_NAMES"), RESERVED_DEVICE_NAMES.to_vec());
        assert_eq!(list("RESERVED_DEVICE_NAME_PREFIXES"), RESERVED_DEVICE_NAME_PREFIXES.to_vec());
    }

    #[test]
    fn every_problem_is_listed() {
        let mut c = cell();
        c.network.gateway = "192.168.100.1".into();
        c.robots[0].ip = "192.168.100.0".into();
        c.robots[0].device_name = "KUKA_ROBI2".into();
        c.robots[1].station = "controller".into();
        c.robots[1].ip = "192.168.100.1".into();
        c.kuka_vc[1].kli_ip = "10.0.0.1".into();
        c.kuka_vc.push(KukaVcController { robot_ns: "robot9".into(), kli_ip: "192.168.100.19".into(), station: "x".into() });
        let issues = c.validate();
        assert_eq!(
            fields(&issues),
            ["robots.robot1.ip", "robots.robot2.station", "robots.robot2.deviceName", "robots.robot2.ip", "kukaVc.robot2.kliIp", "kukaVc.robot9.kliIp"],
            "{issues:#?}"
        );
        assert!(issues[0].message.contains("network address"));
        assert!(issues[1].message.contains("the head PLC"));
        assert!(issues[2].message.contains("also used by robot robot1"));
        assert!(issues[3].message.contains("gateway"));
        assert!(issues[4].message.contains("outside the head subnet 192.168.100.0/24"));

        let mut c = cell();
        c.network.netmask = "255.0.255.0".into();
        c.network.head_ip = "x".into();
        c.robots[0].ip = "".into();
        assert_eq!(fields(&c.validate()), ["network.headIp", "robots.robot1.ip"]);
        c.network.head_ip = "192.168.100.10".into();
        assert_eq!(fields(&c.validate()), ["network.netmask", "robots.robot1.ip"]);
        let mut c = cell();
        c.network.netmask = "255.255.255.254".into();
        assert_eq!(fields(&c.validate()), ["network.netmask"]);
        let mut c = cell();
        c.network.gateway = c.network.head_ip.clone();
        c.network.controller_station = "port-123".into();
        assert_eq!(fields(&c.validate()), ["network.controllerStation", "network.gateway"]);
    }

    #[test]
    fn warnings_name_what_differs_from_the_wiring() {
        let mut c = cell();
        let wiring = CellWiring {
            source: "/etc/plan2cell/cell.toml".into(),
            head_ip: "192.168.100.10".into(),
            netmask: "255.255.255.0".into(),
            controllers: c.kuka_vc.clone(),
        };
        assert_eq!(c.wiring_warnings(Some(&wiring)), vec![]);
        c.robots[0].ip = "192.168.100.21".into();
        let w = c.wiring_warnings(Some(&wiring));
        assert_eq!(fields(&w), ["kukaVc.robot1.kliIp", "robots.robot1.ip"], "{w:#?}");
        c.kuka_vc[0].kli_ip = "192.168.100.21".into();
        assert_eq!(fields(&c.wiring_warnings(Some(&wiring))), ["robots.robot1.ip", "kukaVc.robot1.kliIp"]);
        assert_eq!(c.wiring_warnings(None), vec![], "no virtual cell: only the controller/robot consistency");
    }

    #[test]
    fn no_safety_module_needs_external_and_the_bypass() {
        let mut c = cell();
        c.robots[0].safety_module = false;
        assert_eq!(fields(&c.validate()), ["robots.robot1.safetyModule"]);
        c.robots[0].safety_bypass = true;
        assert_eq!(c.validate(), vec![]);
        c.robots[0].external = false;
        assert_eq!(fields(&c.validate()), ["robots.robot1.safetyModule"]);
        // An older client or file without the flags: bt2plc's defaults, valid.
        let r: RobotHardware = serde_json::from_str(r#"{"ns":"robot1"}"#).unwrap();
        assert!(!r.external && !r.safety_bypass && r.safety_module);
    }

    #[test]
    fn the_virtual_cells_values_and_warnings() {
        let vc = vec!["robot1".to_owned(), "robot2".to_owned()];
        // scene_fit's saved configuration: CODESYS-simulator flags and no gateway.
        let mut c = cell();
        for r in &mut c.robots {
            r.external = false;
        }
        assert_eq!(c.virtual_cell_warnings(&[]), vec![], "real hardware: nothing to say");
        let w = c.virtual_cell_warnings(&vc);
        assert_eq!(
            fields(&w),
            [
                "robots.robot1.safetyModule",
                "robots.robot1.external",
                "robots.robot1.safetyBypass",
                "robots.robot2.safetyModule",
                "robots.robot2.external",
                "robots.robot2.safetyBypass",
                "network.gateway"
            ]
        );
        assert!(w[0].message.contains("ERR_SAFETY_TIMEOUT"), "{}", w[0].message);
        assert!(w[6].message.contains("192.168.100.1 (here: none)"), "{}", w[6].message);

        let fixed = c.with_virtual_cell_values(&vc);
        assert_eq!(fixed.virtual_cell_warnings(&vc), vec![]);
        assert_eq!(fixed.validate(), vec![]);
        assert_eq!(fixed.network.gateway, VIRTUAL_CELL_GATEWAY);
        assert!(fixed.robots.iter().all(|r| r.external && r.safety_bypass && !r.safety_module));
        assert_eq!(fixed.robots[0].ip, c.robots[0].ip, "the addresses stay");
        // Only the robots that are VCs.
        let one = c.with_virtual_cell_values(&vc[..1]);
        assert!(!one.robots[0].safety_module && one.robots[1].safety_module);
        assert_eq!(c.with_virtual_cell_values(&[]), c);
    }

    #[test]
    fn wire_format_is_camel_case() {
        let json = serde_json::to_value(cell()).unwrap();
        assert_eq!(json["network"]["headIp"], "192.168.100.10");
        assert_eq!(json["robots"][0]["deviceName"], "kuka_robi1");
        assert_eq!(json["kukaVc"][0]["kliIp"], "192.168.100.11");
        let view: HardwareView = serde_json::from_str(r#"{"source":"saved","config":{"robots":[{"ns":"r"}]}}"#).unwrap();
        assert_eq!(view.source, HardwareSource::Saved);
        assert_eq!(view.config.robots[0].ns, "r");
    }
}
