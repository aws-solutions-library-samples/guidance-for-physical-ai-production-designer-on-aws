//! The effective hardware of a plan and its configuration (`plan2cell hardware --json`): the
//! PROFINET network, every robot's station name, device name and IP, and the KUKA virtual
//! controllers they imply, resolved by bt2plc's own defaults ([`bt2plc::Config::addressing`]),
//! so it is exactly what [`crate::build`] uses.
//!
//! Hardware overrides come from plan2cell.toml: `[plc.network]` (`head_ip`, `netmask`,
//! `gateway`, `controller_station`, `robot_host_base`, ...) and `[[plc.robot]]` (`station`,
//! `device_name`, `ip`, `model`, `external`, ...).
//!
//! Unlike a build, a hardware report does not stop at the first problem: every rule of
//! [`bt2plc::hardware::check`] that is broken is listed in [`Hardware::issues`].

use std::net::Ipv4Addr;

use serde::{Deserialize, Serialize};

use crate::{BuildConfig, BuildOptions, PlanInput, Result};

pub const HARDWARE_SCHEMA: &str = "plan2cell/hardware@1";

/// `plan2cell hardware --json`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Hardware {
    pub schema: String,
    /// No issue: a build with this configuration accepts the hardware.
    pub ok: bool,
    pub tree_id: String,
    pub cell_id: String,
    pub network: Network,
    /// [`Network`] with no `[plc.network]` override (what a "reset" goes back to).
    pub network_defaults: Network,
    /// BT order, then the idle robots of the configuration (`include_if_idle`).
    pub robots: Vec<Robot>,
    pub kuka_vc: KukaVc,
    pub issues: Vec<Issue>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Network {
    pub head_ip: Ipv4Addr,
    pub netmask: Ipv4Addr,
    /// Prefix length of `netmask` (24 for 255.255.255.0).
    pub prefix: u32,
    /// `0.0.0.0` = none (the head then sets the robots' IPs over DCP).
    pub gateway: Ipv4Addr,
    /// The head PLC's PROFINET NameOfStation.
    pub controller_station: String,
    pub nic: String,
    /// Robot i (1-based) gets host part `robot_host_base + i` unless `[[plc.robot]] ip` is set.
    pub robot_host_base: u8,
    pub ip_pool_first: Ipv4Addr,
    pub ip_pool_last: Ipv4Addr,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Robot {
    pub ns: String,
    /// 1-based position (the default IP and names count from it).
    pub index: usize,
    pub tag: String,
    /// PROFINET NameOfStation of the robot controller.
    pub station: String,
    /// The head project's PN device (an IEC identifier).
    pub device_name: String,
    pub ip: Ipv4Addr,
    /// `[[plc.robot]] model`, else `[[krl.robot]] model`, else `"unknown"`.
    pub model: String,
    pub external: bool,
    pub safety_bypass: bool,
    pub safety_module: bool,
    /// No op in the tree (included for bring-up and homing only).
    pub idle: bool,
    /// The values without this robot's `[[plc.robot]]` station / device_name / ip.
    pub defaults: RobotDefaults,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RobotDefaults {
    pub station: String,
    pub device_name: String,
    pub ip: Ipv4Addr,
}

/// The KUKA virtual controllers the configuration implies: one per `external` robot, on the
/// head's PROFINET subnet (`kvc-cell.env` `C<N>_KLI_IP`, `C<N>_STATION`, `KLI_PREFIX`, `KLI_GW`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct KukaVc {
    pub kli_prefix: u32,
    /// `None` when `[plc.network] gateway` is `0.0.0.0`.
    pub kli_gw: Option<Ipv4Addr>,
    pub head_ip: Ipv4Addr,
    pub controllers: Vec<KukaController>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct KukaController {
    pub robot_ns: String,
    /// = the robot's IP.
    pub kli_ip: Ipv4Addr,
    /// = the robot's station name.
    pub station: String,
}

/// One broken rule (see [`bt2plc::hardware::Issue`]); `field` is `config` for a problem that
/// the plan reports but no hardware rule names (e.g. an invalid `device_name`).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Issue {
    pub field: String,
    pub message: String,
}

impl From<bt2plc::hardware::Issue> for Issue {
    fn from(i: bt2plc::hardware::Issue) -> Self {
        Self {
            field: i.field,
            message: i.message,
        }
    }
}

fn network(n: &bt2plc::config::NetworkCfg) -> Network {
    Network {
        head_ip: n.head_ip,
        netmask: n.netmask,
        prefix: u32::from(n.netmask).leading_ones(),
        gateway: n.gateway,
        controller_station: n.controller_station.clone(),
        nic: n.nic.clone(),
        robot_host_base: n.robot_host_base,
        ip_pool_first: n.ip_pool_first,
        ip_pool_last: n.ip_pool_last,
    }
}

/// The config without any hardware override: the robots (and their order) of a plan do not
/// depend on the addressing, so they are found even when the addressing is invalid.
fn without_hardware(cfg: &bt2plc::Config) -> bt2plc::Config {
    let mut c = cfg.clone();
    let d = bt2plc::config::NetworkCfg::default();
    c.network.head_ip = d.head_ip;
    c.network.netmask = d.netmask;
    c.network.gateway = d.gateway;
    c.network.controller_station = d.controller_station;
    c.network.robot_host_base = d.robot_host_base;
    c.network.ip_pool_first = d.ip_pool_first;
    c.network.ip_pool_last = d.ip_pool_last;
    for r in &mut c.robots {
        r.station = None;
        r.device_name = None;
        r.ip = None;
    }
    c
}

/// The effective hardware of `input` with `cfg` (which may be loaded with
/// [`BuildConfig::parse_unchecked`]: its hardware problems become [`Hardware::issues`]).
/// Errors: the plan itself, or configuration problems that are not about hardware.
pub fn resolve(input: &PlanInput, cfg: &BuildConfig, opts: &BuildOptions) -> Result<Hardware> {
    let eff = crate::effective_unchecked(cfg, opts)?;
    let plc = &eff.plc;
    let mut d = bt2plc::Diagnostics::default();
    let doc = input.doc(&mut d)?;
    let plan = bt2plc::plan(&doc, &without_hardware(plc), &mut d)?;
    let mut issues: Vec<Issue> = Vec::new();
    let mut robots = Vec::new();
    for (i, r) in plan.robots.iter().enumerate() {
        let index = i + 1;
        let rc = plc.robot_cfg(&r.ns);
        let mut bare = plc.clone();
        if let Some(b) = bare.robots.iter_mut().find(|x| x.ns == r.ns) {
            b.station = None;
            b.device_name = None;
            b.ip = None;
        }
        let def = bare.addressing(&r.ns, index);
        let addr = match plc.addressing(&r.ns, index) {
            Ok(a) => a,
            Err(e) => {
                issues.push(Issue {
                    field: format!("robots.{}.ip", r.ns),
                    message: e.to_string(),
                });
                r.addr.clone()
            }
        };
        let def = def.unwrap_or_else(|_| addr.clone());
        let model = rc
            .and_then(|c| c.model.clone())
            .or_else(|| {
                eff.krl
                    .robots
                    .iter()
                    .find(|k| k.ns == r.ns)
                    .and_then(|k| k.model.clone())
            })
            .unwrap_or_else(|| "unknown".into());
        robots.push(Robot {
            ns: r.ns.clone(),
            index,
            tag: addr.tag.clone(),
            station: addr.station.clone(),
            device_name: addr.device_name.clone(),
            ip: addr.ip,
            model,
            external: addr.external,
            safety_bypass: addr.safety_bypass,
            safety_module: addr.safety_module,
            idle: r.idle,
            defaults: RobotDefaults {
                station: def.station,
                device_name: def.device_name,
                ip: def.ip,
            },
        });
    }
    let devices: Vec<bt2plc::hardware::DeviceAddr<'_>> = robots
        .iter()
        .map(|r| bt2plc::hardware::DeviceAddr {
            ns: &r.ns,
            station: &r.station,
            ip: r.ip,
        })
        .collect();
    issues.extend(
        bt2plc::hardware::check(&plc.network, &devices)
            .into_iter()
            .map(Issue::from),
    );
    // Device names: IEC identifiers that clash with nothing in the generated projects.
    let mut device_names: std::collections::BTreeMap<String, &str> = Default::default();
    for r in &robots {
        let field = format!("robots.{}.deviceName", r.ns);
        if let Some(p) = bt2plc::hardware::device_name_problem(&r.device_name) {
            issues.push(Issue {
                field: field.clone(),
                message: format!("device name '{}' {p}", r.device_name),
            });
        }
        if let Some(other) = device_names.insert(r.device_name.to_ascii_lowercase(), &r.ns) {
            issues.push(Issue {
                field,
                message: format!(
                    "device name '{}' is also used by robot {other} (case-insensitive)",
                    r.device_name
                ),
            });
        }
    }
    if issues.is_empty() {
        // Everything else a build checks about these values (device names, tags, ...).
        let mut d = bt2plc::Diagnostics::default();
        let problem = match bt2plc::plan(&doc, plc, &mut d) {
            Err(e) => Some(e.to_string()),
            Ok(_) => eff.krl.validate().err().map(|e| e.to_string()),
        };
        if let Some(message) = problem {
            issues.push(Issue {
                field: "config".into(),
                message,
            });
        }
    }
    let n = network(&plc.network);
    let kuka_vc = KukaVc {
        kli_prefix: n.prefix,
        kli_gw: (!n.gateway.is_unspecified()).then_some(n.gateway),
        head_ip: n.head_ip,
        controllers: robots
            .iter()
            .filter(|r| r.external)
            .map(|r| KukaController {
                robot_ns: r.ns.clone(),
                kli_ip: r.ip,
                station: r.station.clone(),
            })
            .collect(),
    };
    Ok(Hardware {
        schema: HARDWARE_SCHEMA.into(),
        ok: issues.is_empty(),
        tree_id: plan.tree_id.clone(),
        cell_id: plan.cell_id.clone(),
        network: n,
        network_defaults: network(&bt2plc::config::NetworkCfg::default()),
        robots,
        kuka_vc,
        issues,
    })
}
