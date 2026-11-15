//! `project.json` (schema `bt2plc/codesys-project@1`): everything an agent (via a CODESYS MCP
//! server) or a human needs to build the CODESYS projects: device trees with exact DeviceIDs and
//! parameter ids (taken from `HeadPLC_VASS_Sim.export` / `KUKA_VASS_Robot_Sim.export`), IO
//! mappings, tasks, libraries, objects in creation order and the build steps.

use std::net::Ipv4Addr;

use serde::Serialize;

use crate::backend::codesys::GENERATOR;
use crate::backend::model::{ObjKind, PlcProject, ProjectRole};
use crate::config::Config;
use crate::ir::CellPlan;

pub const KUKA_GSDML_ID: &str =
    "GSDML-019B-0301-V2.25-KUKA-Roboter-GmbH-VKR C4-Device-V1.0-20121030 - DIM 2";
pub const KUKA_GSDML_VERSION: &str = "V8.2 - HW 1";
/// The same GSDML's DAP `DIM 3` (V8.3): FDIO64 is only the default module of slot 1
/// (`UsedInSlots`, removable), DIO256 (module 7) in slot 2 as in `DIM 2`. For a robot with
/// `safety_module = false` (simulation only).
pub const KUKA_GSDML_ID_DIM3: &str =
    "GSDML-019B-0301-V2.25-KUKA-Roboter-GmbH-VKR C4-Device-V1.0-20121030 - DIM 3";
pub const KUKA_GSDML_VERSION_DIM3: &str = "V8.3 - HW 1";
pub const KUKA_PN_DEVICE_ID: &str = "0x019B_0x0301_DIM 2";
pub const KUKA_PN_DEVICE_VERSION: &str = "1 V8.2";
/// Channel parameter ids of the PROFINET modules: ChannelType Input / Output.
pub const PARAM_IN: u32 = 77824;
pub const PARAM_OUT: u32 = 81920;

#[derive(Serialize)]
pub struct Manifest {
    pub schema: &'static str,
    pub generator: &'static str,
    pub plan: PlanInfo,
    pub ide: Ide,
    pub prerequisites: Vec<Prerequisite>,
    pub projects: Vec<Project>,
}

#[derive(Serialize)]
pub struct PlanInfo {
    pub tree_id: String,
    pub cell_id: String,
    pub fingerprint: String,
    pub plan_id: String,
    pub grant_policy: String,
    pub zones: usize,
    pub permit_edges: usize,
}

#[derive(Serialize)]
pub struct Ide {
    pub product: &'static str,
    pub version: String,
    pub os: &'static str,
}

#[derive(Serialize)]
pub struct Prerequisite {
    pub id: &'static str,
    pub kind: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub file: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub how: Option<&'static str>,
    pub needed_by: Vec<&'static str>,
    pub provides: DeviceId,
}

#[derive(Clone, Serialize)]
pub struct DeviceId {
    #[serde(rename = "type")]
    pub ty: u32,
    pub id: String,
    pub version: String,
}

#[derive(Serialize)]
pub struct Param {
    pub id: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<&'static str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub member: Option<&'static str>,
    pub value: String,
}

#[derive(Serialize)]
pub struct IoMap {
    pub param_id: u32,
    pub channel: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sub: Option<u32>,
    /// Direction as seen by THIS project's PLC: "in" = received, "out" = sent.
    pub dir: &'static str,
    pub variable: String,
}

/// A slot of a PN device. `empty`: the slot must be empty (`module_id` null, no name, no IO):
/// a module the device description plugs there by default is removed.
#[derive(Serialize)]
pub struct Slot {
    pub slot: u32,
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub empty: bool,
    #[serde(skip_serializing_if = "String::is_empty")]
    pub name: String,
    pub module_id: Option<&'static str>,
    #[serde(skip_serializing_if = "str::is_empty")]
    pub ident: &'static str,
    pub profisafe: bool,
    pub io: Vec<IoMap>,
}

#[derive(Serialize)]
pub struct DeviceNode {
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robot: Option<String>,
    pub device_id: DeviceId,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub parameters: Vec<Param>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub slots: Vec<Slot>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub children: Vec<DeviceNode>,
}

#[derive(Serialize)]
pub struct Library {
    pub name: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vendor: Option<&'static str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<&'static str>,
    /// "ensure" = add if missing; "auto" = normally added by the device tree, verify only.
    pub mode: &'static str,
}

#[derive(Serialize)]
pub struct Task {
    pub name: &'static str,
    pub kind: &'static str,
    pub interval: String,
    pub priority: u32,
    pub pous: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bus_cycle_task_of: Option<String>,
    /// "ensure" = create/configure; "verify_or_create" = normally created by the PN stack.
    pub mode: &'static str,
}

#[derive(Serialize)]
pub struct Object {
    pub order: usize,
    pub name: String,
    pub kind: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pou_type: Option<&'static str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub language: Option<&'static str>,
    pub parent: &'static str,
    pub declaration: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub implementation: Option<String>,
    pub origin: crate::backend::model::Origin,
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub replace_existing: bool,
}

#[derive(Serialize)]
pub struct Step {
    pub op: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parent: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub device: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub slot: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub module_id: Option<&'static str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

impl Step {
    fn new(op: &'static str) -> Self {
        Step {
            op,
            parent: None,
            device: None,
            slot: None,
            module_id: None,
            name: None,
            note: None,
        }
    }
}

#[derive(Serialize)]
pub struct Runtime {
    pub container: String,
    pub ip: String,
    pub gateway_port: u16,
    pub opcua_port: u16,
}

#[derive(Serialize)]
pub struct Acceptance {
    pub build_errors: u32,
    pub objects: Vec<String>,
    pub devices: Vec<String>,
}

#[derive(Serialize)]
pub struct Project {
    pub key: String,
    pub role: ProjectRole,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robot: Option<String>,
    pub name: String,
    pub file: String,
    pub source_dir: String,
    pub runtime: Runtime,
    pub device_tree: DeviceNode,
    pub libraries: Vec<Library>,
    pub tasks: Vec<Task>,
    pub objects: Vec<Object>,
    pub io_mapping_csv: String,
    pub steps: Vec<Step>,
    pub acceptance: Acceptance,
    /// `[opcua] symbols`: the Symbol Configuration (step `symbol_config`), head PLC only.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub symbol_config: Option<SymbolConfig>,
}

/// A Symbol Configuration for the application: which variables the runtime publishes (OPC UA
/// with `support_opc_ua`), with which access. Built with the CODESYS MCP tools
/// `create_symbol_config`, `set_symbol_config_settings`, `set_signature_access_bulk` (an entry
/// without `variable`: every variable of the signature) and `set_symbol_access`; read back with
/// `list_configured_symbols`.
#[derive(Serialize)]
pub struct SymbolConfig {
    pub support_opc_ua: bool,
    pub symbols: Vec<SymbolAccess>,
    /// `Signature.variable` that must be exported readable after the step (acceptance).
    pub require: Vec<String>,
    /// The OPC UA browse paths of the published cell status (below the application node,
    /// `ns=4;s=|var|<device type name>.Application.`), for the clients (documentation).
    pub opc_ua_paths: Vec<String>,
}

#[derive(Serialize)]
pub struct SymbolAccess {
    /// The signature (POU / GVL) by name as the symbol configuration lists it (`PLC_PRG`).
    pub signature: String,
    /// One variable; absent: every variable of the signature.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub variable: Option<String>,
    /// `ReadOnly`, `WriteOnly`, `ReadWrite` or `None`.
    pub access: &'static str,
    /// A failure is a warning, not a project error (e.g. a constant the compiler replaces).
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub optional: bool,
}

/// The cell status FB_Cell / PLC_PRG publish (read-only) for a live view: cell state and fault,
/// cycles, per robot state / op index / zone requests and grants, zone holders.
pub const HEAD_STATUS: &[&str] = &[
    "PLC_PRG.eCellState",
    "PLC_PRG.eCellError",
    "PLC_PRG.bCellError",
    "PLC_PRG.bRecoveryRequired",
    "PLC_PRG.udiCycles",
    "PLC_PRG.bAutoRun",
    "PLC_PRG.fbCell.eState",
    "PLC_PRG.fbCell.eError",
    "PLC_PRG.fbCell.nErrorRobot",
    "PLC_PRG.fbCell.nErrorOp",
    "PLC_PRG.fbCell.nErrorZone",
    "PLC_PRG.fbCell.bCycleActive",
    "PLC_PRG.fbCell.udiCycles",
    "PLC_PRG.fbCell.tLastCycle",
    "PLC_PRG.fbCell.aeRobotState",
    "PLC_PRG.fbCell.aeRobotError",
    "PLC_PRG.fbCell.awStep",
    "PLC_PRG.fbCell.adwReq",
    "PLC_PRG.fbCell.adwGrant",
    "PLC_PRG.fbCell.anDone",
    "PLC_PRG.fbCell.anHolder",
];

/// `[opcua] symbols`: PLC_PRG read-only as a whole (one build; nothing of it is writable over OPC
/// UA): its status copies and the `fbCell` instance with its members. Function blocks are no
/// signatures of their own in the symbol configuration (CODESYS SP22: "Signature 'FB_Cell' not
/// found"), so the cell's arrays come through `PLC_PRG.fbCell`. `GVL_Cell.PLAN_ID` if the compiler
/// keeps the constant.
pub fn head_symbol_config() -> SymbolConfig {
    let sig = |s: &str, v: Option<&str>, optional: bool| SymbolAccess {
        signature: s.into(),
        variable: v.map(str::to_string),
        access: "ReadOnly",
        optional,
    };
    // Read back: the direct PLC_PRG variables; the fbCell members are checked over OPC UA.
    let require: Vec<String> = HEAD_STATUS
        .iter()
        .filter(|p| !p.starts_with("PLC_PRG.fbCell."))
        .map(|p| p.to_string())
        .collect();
    let mut opc_ua_paths: Vec<String> = HEAD_STATUS.iter().map(|p| p.to_string()).collect();
    opc_ua_paths.push("GVL_Cell.PLAN_ID".into());
    SymbolConfig {
        support_opc_ua: true,
        symbols: vec![
            sig("PLC_PRG", None, false),
            sig("GVL_Cell", Some("PLAN_ID"), true),
        ],
        require,
        opc_ua_paths,
    }
}

fn ip_arr(ip: Ipv4Addr) -> String {
    let [a, b, c, d] = ip.octets();
    format!("[{a},{b},{c},{d}]")
}

/// The head's GSDML device of a KUKA robot: DAP `DIM 2`, or `DIM 3` without the safety module.
fn kuka_gsdml(safety_module: bool) -> DeviceId {
    let (id, version) = if safety_module {
        (KUKA_GSDML_ID, KUKA_GSDML_VERSION)
    } else {
        (KUKA_GSDML_ID_DIM3, KUKA_GSDML_VERSION_DIM3)
    };
    DeviceId {
        ty: 81,
        id: id.into(),
        version: version.into(),
    }
}

fn plc_device() -> DeviceId {
    DeviceId {
        ty: 4102,
        id: "0000 0005".into(),
        version: "4.20.0.0".into(),
    }
}

fn ethernet(ip: Ipv4Addr, cfg: &Config, children: Vec<DeviceNode>) -> DeviceNode {
    let n = &cfg.network;
    DeviceNode {
        name: "Ethernet".into(),
        robot: None,
        device_id: DeviceId {
            ty: 110,
            id: "0000 0002".into(),
            version: "4.2.0.0".into(),
        },
        parameters: vec![
            Param {
                id: 0,
                name: Some("IPAddress"),
                member: None,
                value: ip_arr(ip),
            },
            Param {
                id: 1,
                name: Some("SubnetMask"),
                member: None,
                value: ip_arr(n.netmask),
            },
            Param {
                id: 2,
                name: Some("GatewayAddress"),
                member: None,
                value: ip_arr(n.gateway),
            },
            Param {
                id: 10,
                name: Some("NetworkInterfaceName"),
                member: None,
                value: format!("'{}'", n.nic),
            },
            Param {
                id: 12,
                name: Some("NetworkInterfaceName (WSTRING)"),
                member: None,
                value: format!("\"{}\"", n.nic),
            },
        ],
        slots: Vec::new(),
        children,
    }
}

/// FDIO64 (slot 1, 8 byte sub-channels each way) + DIO256 (slot 2, one 32-byte channel each way).
/// `head`: the PN-Controller view (inputs = from robot). Device view: inputs = from the head PLC.
/// `!safety_module`: slot 1 empty (DAP `DIM 3`).
fn slots(tag: Option<&str>, head: bool, safety_module: bool) -> Vec<Slot> {
    let sfx = tag.map(|t| format!("_{t}")).unwrap_or_default();
    let v = |s: &str| format!("Application.GVL_IO.{s}{sfx}");
    let mut fd = Vec::new();
    // In both projects 77824 = ChannelType Input, 81920 = Output (as seen by that PLC).
    let (in_ch, out_ch, in_var, out_var) = if head {
        (
            "8 safe digital inputs",
            "8 safe digital outputs",
            "bySafeIn",
            "bySafeOut",
        )
    } else {
        (
            "8 safe digital outputs",
            "8 safe digital inputs",
            "bySafeOut",
            "bySafeIn",
        )
    };
    for k in 0..8u32 {
        fd.push(IoMap {
            param_id: PARAM_IN,
            channel: in_ch.into(),
            sub: Some(k),
            dir: "in",
            variable: v(&format!("{in_var}{k}")),
        });
    }
    for k in 0..8u32 {
        fd.push(IoMap {
            param_id: PARAM_OUT,
            channel: out_ch.into(),
            sub: Some(k),
            dir: "out",
            variable: v(&format!("{out_var}{k}")),
        });
    }
    let dio = if head {
        vec![
            IoMap {
                param_id: PARAM_IN,
                channel: "Inputs".into(),
                sub: None,
                dir: "in",
                variable: v("aVassInputData"),
            },
            IoMap {
                param_id: PARAM_OUT,
                channel: "Outputs".into(),
                sub: None,
                dir: "out",
                variable: v("aVassOutputData"),
            },
        ]
    } else {
        vec![
            IoMap {
                param_id: PARAM_IN,
                channel: "Output Data".into(),
                sub: None,
                dir: "in",
                variable: v("aVassOutputData"),
            },
            IoMap {
                param_id: PARAM_OUT,
                channel: "Input Data".into(),
                sub: None,
                dir: "out",
                variable: v("aVassInputData"),
            },
        ]
    };
    let slot1 = if safety_module {
        Slot {
            slot: 1,
            empty: false,
            name: format!("SafeIO_FDIO64{sfx}"),
            module_id: Some("1"),
            ident: "16#0000008F",
            profisafe: true,
            io: fd,
        }
    } else {
        Slot {
            slot: 1,
            empty: true,
            name: String::new(),
            module_id: None,
            ident: "",
            profisafe: false,
            io: Vec::new(),
        }
    };
    vec![
        slot1,
        Slot {
            slot: 2,
            empty: false,
            name: format!("VASS_DIO256{sfx}"),
            module_id: Some("7"),
            ident: "16#00001000",
            profisafe: false,
            io: dio,
        },
    ]
}

fn objects(p: &PlcProject) -> Vec<Object> {
    p.objects
        .iter()
        .enumerate()
        .map(|(i, o)| Object {
            order: i + 1,
            name: o.name.clone(),
            kind: match o.kind {
                ObjKind::Dut => "dut",
                ObjKind::Gvl => "gvl",
                ObjKind::FunctionBlock | ObjKind::Program => "pou",
            },
            pou_type: match o.kind {
                ObjKind::FunctionBlock => Some("function_block"),
                ObjKind::Program => Some("program"),
                _ => None,
            },
            language: matches!(o.kind, ObjKind::FunctionBlock | ObjKind::Program).then_some("ST"),
            parent: "Application",
            declaration: p.decl_path(o),
            implementation: p.body_path(o),
            origin: o.origin,
            replace_existing: o.name == "PLC_PRG",
        })
        .collect()
}

fn tasks(main_ms: u32, comm_prio: u32, comm_pou: String, bus_owner: String) -> Vec<Task> {
    vec![
        Task {
            name: "MainTask",
            kind: "cyclic",
            interval: format!("t#{main_ms}ms"),
            priority: 1,
            pous: vec!["PLC_PRG".into()],
            bus_cycle_task_of: None,
            mode: "ensure",
        },
        Task {
            name: "Profinet_CommunicationTask",
            kind: "cyclic",
            interval: "t#10ms".into(),
            priority: comm_prio,
            pous: vec![comm_pou],
            bus_cycle_task_of: None,
            mode: "verify_or_create",
        },
        Task {
            name: "Profinet_IOTask",
            kind: "cyclic",
            interval: "t#1ms".into(),
            priority: 1,
            pous: Vec::new(),
            bus_cycle_task_of: Some(bus_owner),
            mode: "verify_or_create",
        },
    ]
}

fn common_libs(device_side: bool) -> Vec<Library> {
    let mut v = vec![
        Library {
            name: "Standard",
            vendor: Some("System"),
            version: Some("*"),
            mode: "ensure",
        },
        Library {
            name: "IoStandard",
            vendor: Some("System"),
            version: None,
            mode: "auto",
        },
        Library {
            name: "IoDrvEthernet",
            vendor: Some("3S - Smart Software Solutions GmbH"),
            version: Some("4.2.0.0"),
            mode: "auto",
        },
    ];
    if device_side {
        v.push(Library {
            name: "IoDrvProfinetDevice",
            vendor: Some("CODESYS"),
            version: Some("4.7.0.0"),
            mode: "auto",
        });
    } else {
        v.push(Library {
            name: "IoDrvProfinet",
            vendor: Some("CODESYS"),
            version: Some("4.7.1.0"),
            mode: "auto",
        });
    }
    v.push(Library {
        name: "ProfinetCommon",
        vendor: Some("CODESYS"),
        version: Some("4.7.0.0"),
        mode: "auto",
    });
    v.push(Library {
        name: "CommFB",
        vendor: Some("3S - Smart Software Solutions GmbH"),
        version: Some("3.5.17.10"),
        mode: "auto",
    });
    v.push(Library {
        name: "CAA Device Diagnosis",
        vendor: Some("CAA Technical Workgroup"),
        version: None,
        mode: "auto",
    });
    v.push(Library {
        name: "3SLicense",
        vendor: Some("CODESYS"),
        version: None,
        mode: "auto",
    });
    v
}

fn build_steps(dev_steps: Vec<Step>, symbols: bool) -> Vec<Step> {
    let mut s = vec![{
        let mut st = Step::new("create_project");
        st.note = Some("Standard project, device CODESYS Control for Linux SL (4102 / '0000 0005' / 4.20.0.0), language ST".into());
        st
    }];
    s.extend(dev_steps);
    for (op, note) in [
        (
            "ensure_libraries",
            "libraries[] with mode ensure; verify the auto ones",
        ),
        (
            "create_objects",
            "objects[] in order: declaration file -> declaration part, implementation file -> implementation part",
        ),
        ("configure_tasks", "tasks[]; PLC_PRG only in MainTask"),
        (
            "map_io",
            "device_tree slots[].io (fallback: import io_mapping_csv)",
        ),
        ("build", "0 errors required"),
        (
            "symbol_config",
            "symbol_config: Symbol Configuration with OPC UA support, symbols[] with their \
             access (signature without variable: all of its variables), read back require[]",
        ),
        ("save", ""),
    ] {
        if op == "symbol_config" && !symbols {
            continue;
        }
        let mut st = Step::new(op);
        if !note.is_empty() {
            st.note = Some(note.into());
        }
        s.push(st);
    }
    s
}

fn add_device(parent: &str, name: &str) -> Step {
    let mut s = Step::new("add_device");
    s.parent = Some(parent.into());
    s.name = Some(name.into());
    s
}

fn set_params(name: &str) -> Step {
    let mut s = Step::new("set_device_parameters");
    s.device = Some(name.into());
    s
}

/// The slot must be empty: a default module there is removed (DAP `DIM 3`, slot 1).
fn empty_slot(device: &str, slot: u32) -> Step {
    let mut s = Step::new("ensure_empty_slot");
    s.device = Some(device.into());
    s.slot = Some(slot);
    s.note = Some(
        "SIMULATION ONLY (safety_module = false): remove the module the device description plugs \
         into this slot by default (FDIO64 on DAP DIM 3), then read back that the slot is empty"
            .into(),
    );
    s
}

fn module(device: &str, slot: u32, module_id: &'static str, name: String) -> Step {
    let mut s = Step::new("ensure_module");
    s.device = Some(device.into());
    s.slot = Some(slot);
    s.module_id = Some(module_id);
    s.name = Some(name);
    s
}

pub fn manifest(plan: &CellPlan, cfg: &Config, projects: &[PlcProject]) -> Manifest {
    let n = &cfg.network;
    let mut out_projects = Vec::new();
    for p in projects {
        match p.role {
            ProjectRole::HeadPlc => {
                let mut kukas = Vec::new();
                let mut dev_steps = vec![
                    add_device("Device", "Ethernet"),
                    set_params("Ethernet"),
                    add_device("Ethernet", "PN_Controller"),
                    set_params("PN_Controller"),
                ];
                for r in &plan.robots {
                    let safe = r.addr.safety_module;
                    kukas.push(DeviceNode {
                        name: r.addr.device_name.clone(),
                        robot: Some(r.ns.clone()),
                        device_id: kuka_gsdml(safe),
                        parameters: vec![
                            Param {
                                id: 512,
                                name: None,
                                member: Some("Identification.StationName"),
                                value: format!("'{}'", r.addr.station),
                            },
                            Param {
                                id: 512,
                                name: None,
                                member: Some("Identification.IPAddress"),
                                value: ip_arr(r.addr.ip),
                            },
                            Param {
                                id: 512,
                                name: None,
                                member: Some("Identification.SubnetMask"),
                                value: ip_arr(n.netmask),
                            },
                        ]
                        .into_iter()
                        // An external controller (KUKA VC) keeps its own KLI gateway and cannot take a
                        // DCP Set IP: the head must expect exactly that gateway, or it sends one.
                        .chain(
                            (r.addr.external && !n.gateway.is_unspecified()).then(|| Param {
                                id: 512,
                                name: None,
                                member: Some("Identification.DefaultGateway"),
                                value: ip_arr(n.gateway),
                            }),
                        )
                        .collect(),
                        slots: slots(Some(&r.addr.tag), true, safe),
                        children: Vec::new(),
                    });
                    dev_steps.push(add_device("PN_Controller", &r.addr.device_name));
                    dev_steps.push(set_params(&r.addr.device_name));
                    dev_steps.push(if safe {
                        module(
                            &r.addr.device_name,
                            1,
                            "1",
                            format!("SafeIO_FDIO64_{}", r.addr.tag),
                        )
                    } else {
                        empty_slot(&r.addr.device_name, 1)
                    });
                    dev_steps.push(module(
                        &r.addr.device_name,
                        2,
                        "7",
                        format!("VASS_DIO256_{}", r.addr.tag),
                    ));
                }
                let controller = DeviceNode {
                    name: "PN_Controller".into(),
                    robot: None,
                    device_id: DeviceId {
                        ty: 80,
                        id: "0000 0002".into(),
                        version: "4.7.1.0".into(),
                    },
                    parameters: vec![
                        Param {
                            id: 130,
                            name: Some("StationName"),
                            member: None,
                            value: format!("'{}'", n.controller_station),
                        },
                        Param {
                            id: 384,
                            name: Some("FirstIPAddress"),
                            member: None,
                            value: ip_arr(n.ip_pool_first),
                        },
                        Param {
                            id: 385,
                            name: Some("LastIPAddress"),
                            member: None,
                            value: ip_arr(n.ip_pool_last),
                        },
                        Param {
                            id: 386,
                            name: Some("DefaultSubnetMask"),
                            member: None,
                            value: ip_arr(n.netmask),
                        },
                    ],
                    slots: Vec::new(),
                    children: kukas,
                };
                let mut devices = vec!["Ethernet".to_string(), "PN_Controller".to_string()];
                devices.extend(plan.robots.iter().map(|r| r.addr.device_name.clone()));
                out_projects.push(Project {
                    key: p.key.clone(),
                    role: p.role,
                    robot: None,
                    name: p.name.clone(),
                    file: format!("{}.project", p.name),
                    source_dir: p.source_dir(),
                    runtime: Runtime {
                        container: "codesys-master".into(),
                        ip: n.head_ip.to_string(),
                        gateway_port: n.head_gateway_port,
                        opcua_port: n.head_opcua_port,
                    },
                    device_tree: DeviceNode {
                        name: "Device".into(),
                        robot: None,
                        device_id: plc_device(),
                        parameters: Vec::new(),
                        slots: Vec::new(),
                        children: vec![ethernet(n.head_ip, cfg, vec![controller])],
                    },
                    libraries: common_libs(false),
                    tasks: tasks(
                        cfg.codesys.main_task_ms,
                        14,
                        "PN_Controller.CommCycle".into(),
                        "PN_Controller".into(),
                    ),
                    objects: objects(p),
                    io_mapping_csv: format!("{}/io_mapping.csv", p.source_dir()),
                    steps: build_steps(dev_steps, cfg.opcua.symbols),
                    acceptance: Acceptance {
                        build_errors: 0,
                        objects: p.objects.iter().map(|o| o.name.clone()).collect(),
                        devices,
                    },
                    symbol_config: cfg.opcua.symbols.then(head_symbol_config),
                });
            }
            ProjectRole::RobotSim => {
                let r = &plan.robots[p.robot.expect("robot sim has a robot")];
                let dev = DeviceNode {
                    name: r.addr.device_name.clone(),
                    robot: Some(r.ns.clone()),
                    device_id: DeviceId {
                        ty: 83,
                        id: KUKA_PN_DEVICE_ID.into(),
                        version: KUKA_PN_DEVICE_VERSION.into(),
                    },
                    parameters: vec![
                        Param {
                            id: 2,
                            name: None,
                            member: Some("StationName"),
                            value: format!("'{}'", r.addr.station),
                        },
                        Param {
                            id: 5,
                            name: Some("DriverOptions"),
                            member: None,
                            value: "1".into(),
                        },
                    ],
                    slots: slots(None, false, true),
                    children: Vec::new(),
                };
                let dn = r.addr.device_name.clone();
                let dev_steps = vec![
                    add_device("Device", "Ethernet"),
                    set_params("Ethernet"),
                    add_device("Ethernet", &dn),
                    set_params(&dn),
                    module(&dn, 1, "1", "SafeIO_FDIO64".into()),
                    module(&dn, 2, "7", "VASS_DIO256".into()),
                ];
                let container = if r.index == 0 {
                    "codesys-device".to_string()
                } else {
                    format!("codesys-device-{}", r.addr.tag.to_lowercase())
                };
                out_projects.push(Project {
                    key: p.key.clone(),
                    role: p.role,
                    robot: Some(r.ns.clone()),
                    name: p.name.clone(),
                    file: format!("{}.project", p.name),
                    source_dir: p.source_dir(),
                    runtime: Runtime {
                        container,
                        ip: r.addr.ip.to_string(),
                        gateway_port: n.device_gateway_port,
                        opcua_port: n.device_opcua_port,
                    },
                    device_tree: DeviceNode {
                        name: "Device".into(),
                        robot: None,
                        device_id: plc_device(),
                        parameters: Vec::new(),
                        slots: Vec::new(),
                        children: vec![ethernet(r.addr.ip, cfg, vec![dev])],
                    },
                    libraries: common_libs(true),
                    tasks: tasks(
                        cfg.codesys.device_main_task_ms,
                        30,
                        format!("{dn}.CommCycle"),
                        dn.clone(),
                    ),
                    objects: objects(p),
                    io_mapping_csv: format!("{}/io_mapping.csv", p.source_dir()),
                    steps: build_steps(dev_steps, false),
                    acceptance: Acceptance {
                        build_errors: 0,
                        objects: p.objects.iter().map(|o| o.name.clone()).collect(),
                        devices: vec!["Ethernet".into(), dn],
                    },
                    symbol_config: None,
                });
            }
        }
    }
    let gsdml = format!(
        "profinet-headplc-robot-simulation/codesys-project/kuka-vass-device-simulator/gsdml/{}",
        cfg.codesys.gsdml_file
    );
    let mut prerequisites = vec![Prerequisite {
        id: "gsdml",
        kind: "device_repository_install",
        file: Some(gsdml.clone()),
        how: None,
        needed_by: vec!["head_plc"],
        provides: kuka_gsdml(true),
    }];
    if plan.robots.iter().any(|r| !r.addr.safety_module) {
        prerequisites.push(Prerequisite {
            id: "gsdml_dim3",
            kind: "device_repository_install",
            file: Some(gsdml),
            how: Some(
                "the same GSDML file: its DAP 'DIM 3' (V8.3), for the robots with safety_module = false",
            ),
            needed_by: vec!["head_plc"],
            provides: kuka_gsdml(false),
        });
    }
    prerequisites.push(Prerequisite {
        id: "pn_device_desc",
        kind: "manual",
        file: None,
        how: Some(
            "Tools > Miscellaneous > Generate PROFINET Device from GSDML (select the KUKA GSDML, DAP 'DIM 2' VKRC nexxt Device V8.2); not scriptable",
        ),
        needed_by: vec!["robot_sim"],
        provides: DeviceId {
            ty: 83,
            id: KUKA_PN_DEVICE_ID.into(),
            version: KUKA_PN_DEVICE_VERSION.into(),
        },
    });
    Manifest {
        schema: "bt2plc/codesys-project@1",
        generator: GENERATOR,
        plan: PlanInfo {
            tree_id: plan.tree_id.clone(),
            cell_id: plan.cell_id.clone(),
            fingerprint: format!("fnv1a64:{:016x}", plan.fingerprint),
            plan_id: format!("16#{:04X}", plan.plan_id),
            grant_policy: format!("{:?}", plan.grant_policy).to_lowercase(),
            zones: plan.zones.len(),
            permit_edges: plan.permit_edges.len(),
        },
        ide: Ide {
            product: "CODESYS",
            version: cfg.codesys.ide_version.clone(),
            os: "windows",
        },
        prerequisites,
        projects: out_projects,
    }
}
