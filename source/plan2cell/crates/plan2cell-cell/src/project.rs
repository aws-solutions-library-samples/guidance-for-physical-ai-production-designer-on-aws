//! `plc/project.json` (schema `bt2plc/codesys-project@1`, written by bt2plc's
//! `backend/codesys/project_json.rs`), read back.

use serde::{Deserialize, Serialize};

use crate::{Error, Result};

pub const SCHEMA: &str = "bt2plc/codesys-project@1";

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Manifest {
    pub schema: String,
    pub generator: String,
    pub plan: PlanInfo,
    pub ide: Ide,
    #[serde(default)]
    pub prerequisites: Vec<Prerequisite>,
    pub projects: Vec<Project>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlanInfo {
    pub tree_id: String,
    pub cell_id: String,
    pub fingerprint: String,
    pub plan_id: String,
    pub grant_policy: String,
    pub zones: usize,
    pub permit_edges: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Ide {
    pub product: String,
    pub version: String,
    pub os: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Prerequisite {
    pub id: String,
    pub kind: String,
    pub file: Option<String>,
    pub how: Option<String>,
    #[serde(default)]
    pub needed_by: Vec<String>,
    pub provides: DeviceId,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct DeviceId {
    #[serde(rename = "type")]
    pub ty: u32,
    pub id: String,
    pub version: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Param {
    pub id: u32,
    pub name: Option<String>,
    pub member: Option<String>,
    pub value: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct IoMap {
    pub param_id: u32,
    pub channel: String,
    pub sub: Option<u32>,
    pub dir: String,
    pub variable: String,
}

/// A slot of a PN device. `empty`: the slot must hold no module (`module_id` null, no name, no
/// IO; bt2plc's `safety_module = false`); a module the device comes with there is unplugged.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Slot {
    pub slot: u32,
    #[serde(default)]
    pub empty: bool,
    #[serde(default)]
    pub name: String,
    pub module_id: Option<String>,
    #[serde(default)]
    pub ident: String,
    #[serde(default)]
    pub profisafe: bool,
    #[serde(default)]
    pub io: Vec<IoMap>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DeviceNode {
    pub name: String,
    pub robot: Option<String>,
    pub device_id: DeviceId,
    #[serde(default)]
    pub parameters: Vec<Param>,
    #[serde(default)]
    pub slots: Vec<Slot>,
    #[serde(default)]
    pub children: Vec<DeviceNode>,
}

impl DeviceNode {
    /// Every node below this one, depth first, with its parent's name.
    pub fn descendants(&self) -> Vec<(&str, &DeviceNode)> {
        let mut out = Vec::new();
        fn walk<'a>(n: &'a DeviceNode, out: &mut Vec<(&'a str, &'a DeviceNode)>) {
            for c in &n.children {
                out.push((n.name.as_str(), c));
                walk(c, out);
            }
        }
        walk(self, &mut out);
        out
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Library {
    pub name: String,
    pub vendor: Option<String>,
    pub version: Option<String>,
    pub mode: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Task {
    pub name: String,
    pub kind: String,
    pub interval: String,
    pub priority: u32,
    #[serde(default)]
    pub pous: Vec<String>,
    pub bus_cycle_task_of: Option<String>,
    pub mode: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Object {
    pub order: usize,
    pub name: String,
    pub kind: String,
    pub pou_type: Option<String>,
    pub language: Option<String>,
    pub parent: String,
    pub declaration: String,
    pub implementation: Option<String>,
    pub origin: String,
    #[serde(default)]
    pub replace_existing: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Step {
    pub op: String,
    pub parent: Option<String>,
    pub device: Option<String>,
    pub slot: Option<u32>,
    pub module_id: Option<String>,
    pub name: Option<String>,
    pub note: Option<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Runtime {
    pub container: String,
    pub ip: String,
    pub gateway_port: u16,
    pub opcua_port: u16,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Acceptance {
    pub build_errors: u32,
    #[serde(default)]
    pub objects: Vec<String>,
    #[serde(default)]
    pub devices: Vec<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Role {
    HeadPlc,
    RobotSim,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Project {
    pub key: String,
    pub role: Role,
    pub robot: Option<String>,
    pub name: String,
    pub file: String,
    pub source_dir: String,
    pub runtime: Runtime,
    pub device_tree: DeviceNode,
    #[serde(default)]
    pub libraries: Vec<Library>,
    #[serde(default)]
    pub tasks: Vec<Task>,
    #[serde(default)]
    pub objects: Vec<Object>,
    pub io_mapping_csv: String,
    #[serde(default)]
    pub steps: Vec<Step>,
    pub acceptance: Acceptance,
    /// bt2plc `[opcua] symbols`: the Symbol Configuration of the step `symbol_config`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub symbol_config: Option<SymbolConfig>,
}

/// The application's Symbol Configuration (what the runtime publishes over OPC UA).
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct SymbolConfig {
    #[serde(default)]
    pub support_opc_ua: bool,
    #[serde(default)]
    pub symbols: Vec<SymbolAccess>,
    /// `Signature.variable` that must be exported readable (read back).
    #[serde(default)]
    pub require: Vec<String>,
    /// The published browse paths below the application node (documentation).
    #[serde(default)]
    pub opc_ua_paths: Vec<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SymbolAccess {
    pub signature: String,
    /// `None`: every variable of the signature.
    pub variable: Option<String>,
    pub access: String,
    #[serde(default)]
    pub optional: bool,
}

impl Manifest {
    pub fn parse(text: &str) -> Result<Self> {
        let m: Manifest =
            serde_json::from_str(text).map_err(|e| Error::Config(format!("project.json: {e}")))?;
        if m.schema != SCHEMA {
            return Err(Error::Config(format!(
                "project.json: schema \"{}\", want \"{SCHEMA}\"",
                m.schema
            )));
        }
        if m.projects.is_empty() {
            return Err(Error::Config("project.json: no projects".into()));
        }
        for p in &m.projects {
            let t = &p.device_tree;
            for n in std::iter::once(t).chain(t.descendants().into_iter().map(|(_, n)| n)) {
                for s in &n.slots {
                    let ok = if s.empty {
                        s.module_id.is_none() && s.io.is_empty()
                    } else {
                        s.module_id.is_some() && !s.name.is_empty()
                    };
                    if !ok {
                        return Err(Error::Config(format!(
                            "project.json: {} {} slot {}: an empty slot has no module_id and no \
                             io, any other a module_id and a name",
                            p.name, n.name, s.slot
                        )));
                    }
                }
            }
            let has_step = p.steps.iter().any(|s| s.op == "symbol_config");
            if has_step != p.symbol_config.is_some() {
                return Err(Error::Config(format!(
                    "project.json: {}: a step symbol_config needs symbol_config, and the other \
                     way round",
                    p.name
                )));
            }
            for a in p.symbol_config.iter().flat_map(|c| &c.symbols) {
                if !["None", "ReadOnly", "WriteOnly", "ReadWrite"].contains(&a.access.as_str()) {
                    return Err(Error::Config(format!(
                        "project.json: {}: symbol {} access \"{}\" (want None, ReadOnly, \
                         WriteOnly or ReadWrite)",
                        p.name, a.signature, a.access
                    )));
                }
            }
        }
        Ok(m)
    }

    /// The order the projects are built and deployed in: robot simulators first (the head PLC's
    /// PROFINET devices must exist when it starts), the head PLC last.
    pub fn ordered(&self) -> Vec<&Project> {
        let mut v: Vec<&Project> = self
            .projects
            .iter()
            .filter(|p| p.role == Role::RobotSim)
            .collect();
        v.extend(self.projects.iter().filter(|p| p.role == Role::HeadPlc));
        v
    }

    /// Every DeviceID the projects use, once.
    pub fn device_ids(&self) -> Vec<DeviceId> {
        let mut v: Vec<DeviceId> = Vec::new();
        for p in &self.projects {
            let t = &p.device_tree;
            for d in std::iter::once(&t.device_id)
                .chain(t.descendants().into_iter().map(|(_, n)| &n.device_id))
            {
                if !v.contains(d) {
                    v.push(d.clone());
                }
            }
        }
        v
    }
}
