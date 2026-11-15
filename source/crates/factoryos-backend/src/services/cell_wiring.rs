//! The wiring of a workspace's own virtual cell (`factoryos-cell:ws-<alias>`): every launch builds
//! the cell's instances fresh, so their addresses change. Once the cell is `ready`, the backend
//! renders what the cell's clients need from the Lambda's `wiring` and writes it, atomically and
//! owned by the backend's user, to [`WIRE_DIR`]:
//!
//! * `cell.toml`: the `plan2cell cell` config (`FACTORY_OS_PLAN2CELL_CELL_CONFIG` points at it):
//!   the IDE's MCP URL, the PLC host and KUKA VC hosts by instance id (SSM), and `[address_map]`:
//!   the PROFINET addresses inside the cell (`192.168.100.x`, the same in every cell) to the PLC
//!   host's secondary IPs that reach them (1:1 NAT), for the CODESYS gateway's binds.
//! * `fos-live-bridge.toml`: the live bridge's config with the head's OPC UA and the KUKA VCs'
//!   Zenoh endpoints mapped the same way; and, once an export is deployed onto the cell
//!   ([`WireFiles::apply_export`]), that export's cell name, telegram bytes, zones and each robot's
//!   operations (from its `plc/cell_plan.json`; the template's are another cell's).
//! * `dcv.env`: `FACTORY_OS_DCV_SERVER=<ide ip>` (the IDE's DCV server; the backend's DCV gateway
//!   already finds it through the cell status).
//! * `generation`: `<launch> <fingerprint>`, written last: the root unit
//!   `factoryos-cell-wire.path` watches it and restarts `fos-live-bridge` with the new config.
//!
//! Nothing is rewritten while the generation file already names the same wiring (a backend restart
//! does not restart the bridge).
//!
//! A template-resolved launch's wiring is `factoryos/wiring@2` (docs/tasks/cell-templates.md §11):
//! today's fields plus `components` (each host's semantic, driver and endpoints). Every template of
//! builtin kinds still has today's projection (`plcHost`, `windowsIde`, `kukaVc`), and the files are
//! rendered from it, byte for byte as before; only [`Wiring::complete_for`] reads the components.
//!
//! Engine 2 (E8-E10 there): a wiring with a `driver = "hook"` component (a `pod@1` head or robots
//! driven through `factoryos/hook@1`) has the projection of its builtin components only; its
//! cell.toml and `cell-<net>.toml` are rendered from the components ([`render_hook_cell_toml`]:
//! `[plc_host] driver = "hook"`, `[[kuka_vc.controller]] driver = "hook"`, `[codesys_mcp]` only
//! with `deploy.head.codesys-mcp`), and the live bridge reads the hooks' `watch` over `kubectl exec`
//! with the kubeconfig ([`render_hook_bridge_toml`]: `[exec]`, `[[robot]] source = "exec"`).

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use anyhow::Context as _;
use serde::{Deserialize, Serialize};

/// Where the backend writes a launch's wiring.
pub const WIRE_DIR: &str = "/var/lib/factoryos/cell";
/// The `plan2cell cell` config template (the shared cell's, synced by connect-cell); built-in
/// defaults without it.
pub const CELL_TEMPLATE: &str = "/opt/plan2cell/test-server/cell.toml";
/// The live bridge's config template: an own cell's (`s3://<bucket>/infra/cell-viewer/`, which
/// connect-cell mirrors to /opt/plan2cell/cell-viewer and then removes /opt/plan2cell/viewer).
pub const BRIDGE_TEMPLATE: &str = "/opt/plan2cell/cell-viewer/fos-live-bridge.toml";
/// The shared cell's (`s3://<bucket>/infra/viewer/`, test-server connect-cell.sh), read when
/// [`BRIDGE_TEMPLATE`] is the configured template and does not exist.
pub const BRIDGE_TEMPLATE_SHARED: &str = "/opt/plan2cell/viewer/fos-live-bridge.toml";

/// The bridge template's text: `configured`, or, when that is `default` and missing, `fallback`.
/// An explicitly configured template (FACTORY_OS_CELL_WIRE_BRIDGE_TEMPLATE) has no fallback.
fn read_bridge_template(configured: &Path, default: &Path, fallback: &Path) -> std::io::Result<String> {
    match std::fs::read_to_string(configured) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound && configured == default => std::fs::read_to_string(fallback),
        r => r,
    }
}
pub const CELL_TOML: &str = "cell.toml";
pub const BRIDGE_TOML: &str = "fos-live-bridge.toml";
pub const DCV_ENV: &str = "dcv.env";
pub const GENERATION: &str = "generation";
/// An EKS cell's kubeconfig (`kubectl exec` of plan2cell cell; the cell.toml names it).
pub const KUBECONFIG: &str = "kubeconfig";
/// Where cell.toml says the kubeconfig is (the files' directory on a workspace).
pub const KUBECONFIG_PATH: &str = "/var/lib/factoryos/cell/kubeconfig";
/// The export the live bridge's config follows (`{"cell", "cellPlan"}`), kept for the next launch.
pub const BRIDGE_EXPORT: &str = "bridge-export.json";

/// What the live bridge takes from an export (`plc/cell_plan.json`, schema `bt2plc/cell-plan@1`).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct BridgeExport {
    pub cell: String,
    pub cell_plan: PathBuf,
}

/// `doc` (a fos-live-bridge config) for `cell` running the export whose cell plan is `plan`: the
/// cell's name, the telegram bytes and zones (in bit order) of its protocol, and per `[[robot]]`
/// whose `robot_id` is a robot of the plan, its operations and tag. Robots the plan does not have
/// keep the template's.
pub fn apply_cell_plan(doc: &mut toml::Table, cell: &str, plan: &serde_json::Value) {
    use toml::Value;
    let table = |doc: &mut toml::Table, key: &str| -> toml::Table {
        doc.get(key).and_then(Value::as_table).cloned().unwrap_or_default()
    };
    let mut sink = table(doc, "sink");
    sink.insert("cell_name".into(), Value::String(cell.to_owned()));
    doc.insert("sink".into(), Value::Table(sink));
    let mut backend = table(doc, "backend");
    backend.insert("cell".into(), Value::String(cell.to_owned()));
    doc.insert("backend".into(), Value::Table(backend));
    let mut proto = table(doc, "protocol");
    let p = &plan["config"]["protocol"];
    for (key, from) in [("step_byte", "step_byte"), ("req_byte", "request_byte"), ("grant_byte", "grant_byte"), ("plan_id_byte", "plan_id_byte")] {
        if let Some(n) = p[from].as_i64() {
            proto.insert(key.into(), Value::Integer(n));
        }
    }
    if let Some(zones) = plan["zones"].as_array() {
        let mut zones: Vec<(i64, String)> =
            zones.iter().filter_map(|z| Some((z["bit"].as_i64()?, z["name"].as_str()?.to_owned()))).collect();
        zones.sort();
        proto.insert("zones".into(), Value::Array(zones.into_iter().map(|(_, n)| Value::String(n)).collect()));
    }
    doc.insert("protocol".into(), Value::Table(proto));
    let robots = plan["robots"].as_array().cloned().unwrap_or_default();
    if let Some(entries) = doc.get_mut("robot").and_then(Value::as_array_mut) {
        for entry in entries.iter_mut().filter_map(Value::as_table_mut) {
            let Some(id) = entry.get("robot_id").and_then(Value::as_str).map(str::to_owned) else { continue };
            let Some(r) = robots.iter().find(|r| r["ns"].as_str() == Some(id.as_str())) else { continue };
            let ops = r["ops"].as_array().map(|o| o.iter().filter_map(|v| v.as_str().map(|s| Value::String(s.to_owned()))).collect());
            if let Some(ops) = ops {
                entry.insert("ops".into(), Value::Array(ops));
            }
            if let Some(tag) = r["tag"].as_str() {
                entry.insert("name".into(), Value::String(tag.to_owned()));
            }
        }
    }
}

/// The Lambda's `wiring` (only from `configuring` on).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct Wiring {
    pub launch: u32,
    pub plc_host: Host,
    pub windows_ide: Ide,
    pub kuka_vc: Vec<KukaVcHost>,
    pub profinet: Profinet,
    /// `192.168.100.10` → the PLC host's secondary IP that reaches it, ...
    pub address_map: BTreeMap<String, String>,
    pub mcp_token_secret: Option<String>,
    /// An EKS cell's network name (`n1`..) when this is one of [`Wiring::networks`].
    pub name: Option<String>,
    /// An EKS cell's PROFINET networks, each wired as a cell of its own (the top level is the first's).
    pub networks: Vec<Wiring>,
    /// An EKS cell's cluster (kubectl exec into its pods).
    pub kubernetes: Option<Kubernetes>,
    /// `factoryos/wiring@2` from a template-resolved launch (`None`: today's wiring).
    pub schema: Option<String>,
    /// A template-resolved launch's hosts, every network's (empty: today's wiring).
    pub components: Vec<WiringComponent>,
}

/// The semantics of a [`WiringComponent`].
pub const SEMANTIC_HEAD_PLC: &str = "head-plc";
pub const SEMANTIC_ROBOT_CONTROLLER: &str = "robot-controller";
pub const SEMANTIC_COMPILE: &str = "compile";
pub const SEMANTIC_IDE: &str = "ide";
/// A `pod@1` head or robot driven through `factoryos/hook@1` (docs/tasks/cell-templates.md E7, E8).
pub const DRIVER_HOOK: &str = "hook";
/// The hook in a hook image, and the only argv the live bridge runs (`[exec]`, E10).
pub const HOOK_PATH: &str = "/opt/factoryos/hook";
/// The live bridge's exec source: its poll period and how long one `watch` runs (E10).
pub const WATCH_PERIOD_MS: i64 = 200;
pub const WATCH_MAX_S: i64 = 600;

/// One host of a `wiring@2` (its keys that do not apply are absent).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct WiringComponent {
    /// The host role key: `plcHost`, `kukaVc1`, `compile`, `windowsIde`; an EKS cell's `n1/plcHost`, ...
    pub name: String,
    /// [`SEMANTIC_HEAD_PLC`], `robot-controller`, [`SEMANTIC_COMPILE`], [`SEMANTIC_IDE`] or `aux`.
    pub semantic: String,
    /// `codesys` (a head PLC), `kss` (a robot controller); `hook` from engine 2.
    pub driver: Option<String>,
    pub network: Option<String>,
    pub robot_ns: Option<String>,
    pub station: Option<String>,
    /// Its PROFINET address inside the cell.
    pub address: Option<String>,
    /// An EC2 host (SSM).
    pub instance_id: Option<String>,
    /// An EKS pod: `<namespace>/<pod>/<container>`.
    pub kubectl_pod: Option<String>,
    /// The private IP that reaches it.
    pub ip: Option<String>,
    pub hook_container: Option<String>,
    /// `opcua`, `gateway`, `zenoh`, `smartpad`, `mcp`, `dcv`: each only with its capability.
    pub endpoints: BTreeMap<String, String>,
}

impl WiringComponent {
    fn endpoint(&self, key: &str) -> bool {
        self.endpoints.get(key).is_some_and(|e| !e.is_empty())
    }

    /// Driven through `factoryos/hook@1` ([`DRIVER_HOOK`]).
    pub fn hook(&self) -> bool {
        self.driver.as_deref() == Some(DRIVER_HOOK)
    }

    /// Its exec target as the live bridge takes it: `(<namespace>/<pod>, <container>)`.
    fn exec_target(&self) -> Option<(String, String)> {
        let pod = self.kubectl_pod.as_deref().filter(|p| valid_kubectl_pod(p))?;
        let (ns_pod, container) = pod.rsplit_once('/')?;
        Some((ns_pod.to_owned(), container.to_owned()))
    }
}

impl Wiring {
    /// The extra networks of an EKS cell (all but the first, which the top level wires): `(name, its wiring)`, the
    /// cell-wide parts (launch, MCP token secret) filled in.
    pub fn extra_networks(&self) -> Vec<(String, Wiring)> {
        self.networks
            .iter()
            .skip(1)
            .filter_map(|n| {
                let name = n.name.clone().filter(|x| x.len() <= 3 && x.chars().all(|c| c.is_ascii_alphanumeric()))?;
                let mut w = n.clone();
                w.launch = self.launch;
                w.mcp_token_secret = self.mcp_token_secret.clone();
                w.kubernetes = self.kubernetes.clone();
                w.networks.clear();
                // A template's components are listed at the top level only: this network's, and the
                // cell-wide ones (compile, IDE).
                w.schema = self.schema.clone();
                w.components = self.components.iter().filter(|c| c.network.as_deref().is_none_or(|x| x == name)).cloned().collect();
                Some((name, w))
            })
            .collect()
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct Host {
    pub instance_id: String,
    pub ip: String,
    /// An EKS cell's pod: `<namespace>/<pod>/<container>` for `kubectl exec` (then no SSM).
    pub kubectl_pod: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct Ide {
    pub instance_id: String,
    pub ip: String,
    /// `http://<ip>:8766/mcp`.
    pub mcp_url: Option<String>,
    /// `<ip>:8443`.
    pub dcv_server: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct KukaVcHost {
    /// `kukaVc1`, ...
    pub role: String,
    pub instance_id: String,
    pub ip: String,
    pub station: Option<String>,
    pub kli_ip: Option<String>,
    pub netns: Option<String>,
    pub instance: Option<String>,
    /// An EKS cell's KUKA VC pod: `<namespace>/<pod>/<container>` for `kubectl exec` (then no SSM).
    pub kubectl_pod: Option<String>,
}

/// An EKS cell's cluster, for the workspace's kubeconfig (its API endpoint is private: inside the VPC only).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct Kubernetes {
    pub cluster: String,
    /// `https://<id>.<region>.eks.amazonaws.com`.
    pub endpoint: String,
    /// The cluster CA, base64 PEM.
    pub ca: String,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct Profinet {
    pub cidr: String,
    pub gateway: String,
    /// The head PLC's address on the segment (EKS: 10.77.<k>.10; empty: the EC2 cell's default).
    pub head: String,
    pub vni: Option<u32>,
}

impl Wiring {
    /// Some component is driven through `factoryos/hook@1` (an engine-2 launch): the files are
    /// rendered from the components.
    pub fn has_hook(&self) -> bool {
        self.components.iter().any(WiringComponent::hook)
    }

    /// The network the top level wires: its `name` if components are of it, else the first head
    /// PLC's (`None`: a cell of one network without names, every component).
    fn scope(&self) -> Option<&str> {
        self.name
            .as_deref()
            .filter(|n| self.components.iter().any(|c| c.network.as_deref() == Some(*n)))
            .or_else(|| self.components.iter().find(|c| c.semantic == SEMANTIC_HEAD_PLC).and_then(|c| c.network.as_deref()))
    }

    /// The components of `semantic` of the top level's network ([`Self::scope`]) and the cell-wide ones, in order.
    pub fn scoped<'a>(&'a self, semantic: &'a str) -> impl Iterator<Item = &'a WiringComponent> + 'a {
        let scope = self.scope();
        self.components
            .iter()
            .filter(move |c| c.semantic == semantic && (scope.is_none() || c.network.is_none() || c.network.as_deref() == scope))
    }
}

impl KukaVcHost {
    /// The robot of the build this controller runs: `kukaVc2` → `robot2`.
    pub fn robot_ns(&self) -> String {
        match self.role.strip_prefix("kukaVc").filter(|n| !n.is_empty() && n.chars().all(|c| c.is_ascii_digit())) {
            Some(n) => format!("robot{n}"),
            None => self.role.clone(),
        }
    }
}

impl Wiring {
    /// Usable: the PLC host and the IDE have addresses.
    pub fn complete(&self) -> bool {
        !self.plc_host.instance_id.is_empty() && !self.windows_ide.ip.is_empty()
    }

    /// Usable for a cell of these template `capabilities` (`None`: a cell without a template). A
    /// `wiring@2` with components: the first head PLC has an address and an exec target, a compile
    /// host with its MCP endpoint is there if the cell deploys with the CODESYS MCP
    /// (`deploy.head.codesys-mcp`), an IDE with its DCV endpoint if it has `ide.dcv`. Without
    /// components or capabilities: [`Self::complete`]. The files are rendered from the legacy
    /// projection, which must be there too (every template of builtin kinds has it), except for a
    /// hook head (engine 2: no `plcHost`; the files come from the components).
    pub fn complete_for(&self, capabilities: Option<&[String]>) -> bool {
        let Some(caps) = capabilities.filter(|_| !self.components.is_empty()) else { return self.complete() };
        let has = |c: &str| caps.iter().any(|x| x == c);
        let some = |v: &Option<String>| v.as_deref().is_some_and(|s| !s.is_empty());
        let any = |semantic: &str, ok: &dyn Fn(&WiringComponent) -> bool| self.components.iter().filter(|c| c.semantic == semantic).any(ok);
        let first = self.components.iter().find(|c| c.semantic == SEMANTIC_HEAD_PLC);
        let head = first.is_some_and(|h| some(&h.address) && (some(&h.instance_id) || some(&h.kubectl_pod)));
        let compile = !has("deploy.head.codesys-mcp") || any(SEMANTIC_COMPILE, &|c| c.endpoint("mcp"));
        let ide = !has("ide.dcv") || any(SEMANTIC_IDE, &|c| c.endpoint("dcv"));
        let projection = first.is_some_and(WiringComponent::hook) || !self.plc_host.instance_id.is_empty();
        head && compile && ide && projection
    }

    /// What identifies this wiring (the launch and every address): a change rewrites the files.
    /// `wiring@2`'s `schema` and `components` are not part of it: a template-launched cell of the
    /// same addresses writes the same generation (the bridge is not restarted for nothing). A
    /// wiring with hook drivers has no projection to tell its launches' pods apart: its
    /// components' pods and addresses count too.
    pub fn fingerprint(&self) -> String {
        let mut parts = vec![self.plc_host.instance_id.clone(), self.plc_host.ip.clone(), self.windows_ide.ip.clone()];
        parts.extend(self.kuka_vc.iter().map(|k| format!("{}={}@{}", k.role, k.instance_id, k.ip)));
        parts.extend(self.address_map.iter().map(|(k, v)| format!("{k}>{v}")));
        if self.has_hook() {
            parts.extend(self.components.iter().map(|c| {
                let v = |o: &Option<String>| o.clone().unwrap_or_default();
                format!("{}:{}={}@{}", c.name, v(&c.driver), v(&c.kubectl_pod), v(&c.address))
            }));
        }
        format!("{:016x}", fnv1a(parts.join(",").as_bytes()))
    }

    /// The generation line: `<launch> <fingerprint>`.
    pub fn generation(&self) -> String {
        format!("{} {}", self.launch, self.fingerprint())
    }

    /// The address `ip` (inside the cell) is reached at from outside it.
    pub fn map<'a>(&'a self, ip: &'a str) -> &'a str {
        self.address_map.get(ip).map_or(ip, String::as_str)
    }

    /// The MCP URL (default: the IDE's `:8766/mcp`).
    pub fn mcp_url(&self) -> String {
        self.windows_ide.mcp_url.clone().unwrap_or_else(|| format!("http://{}:8766/mcp", self.windows_ide.ip))
    }

    /// The IDE's DCV server host.
    pub fn dcv_host(&self) -> String {
        self.windows_ide
            .dcv_server
            .as_deref()
            .map(|s| s.rsplit_once(':').map_or(s, |(h, _)| h).to_owned())
            .unwrap_or_else(|| self.windows_ide.ip.clone())
    }
}

fn fnv1a(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325_u64, |h, b| (h ^ u64::from(*b)).wrapping_mul(0x0100_0000_01b3))
}

/// The built-in base of `cell.toml` (as the shared cell's, without its instances).
const CELL_BASE: &str = r#"
[codesys_mcp]
token_env = "PLAN2CELL_CODESYS_MCP_TOKEN"
windows_project_dir = 'C:\ProgramData\plan2cell\projects\{cell}'
install = "SP22 Patch 1"
io_csv = "mirror"
timeout_s = 1800

[plc_host]
ssh_host = "plc-host"
setup = "/opt/plan2cell/plc-host/setup.sh"
compose_dir = "/opt/plan2cell/plc-host"

[kuka_vc]
"#;

const SPS_BASE: &str = "iiQKA.OS2_KSS_9.2.0";
const KUKA_INSTANCE: &str = "iiQKA.OS2_KSS_9.2.0_vass";

/// `cell.toml` for this wiring: `template` (else the built-in base) with the IDE's MCP URL, the
/// PLC host's and the KUKA VC hosts' instances and `[address_map]`. A template controller of the
/// same `robot_ns` gives the defaults the wiring has not (instance, netns, sps_base, ...).
pub fn render_cell_toml(w: &Wiring, template: Option<&str>, region: Option<&str>) -> anyhow::Result<String> {
    use toml::Value as V;
    let mut doc: toml::Table = template.unwrap_or(CELL_BASE).parse().context("the cell.toml template is not valid TOML")?;
    let mut mcp = take_table(&mut doc, "codesys_mcp")?;
    codesys_mcp(w, &mut mcp);
    doc.insert("codesys_mcp".into(), V::Table(mcp));
    let mut plc = take_table(&mut doc, "plc_host")?;
    codesys_plc_host(w, &mut plc, region);
    doc.insert("plc_host".into(), V::Table(plc));
    let mut kuka = take_table(&mut doc, "kuka_vc")?;
    let old = old_controllers(&mut kuka);
    let controllers = kss_controllers(w, &old);
    if let Some(region) = region {
        kuka.entry("region").or_insert_with(|| V::String(region.to_owned()));
    }
    if !controllers.is_empty() {
        if w.kuka_vc.iter().any(|k| k.kubectl_pod.is_some()) {
            kuka.insert("kubeconfig".into(), V::String(KUBECONFIG_PATH.to_owned()));
        }
        kuka.insert("controller".into(), V::Array(controllers));
        doc.insert("kuka_vc".into(), V::Table(kuka));
    }
    address_map_and_profinet(w, &mut doc)?;
    Ok(cell_toml_header(w) + &toml::to_string_pretty(&doc)?)
}

/// `[key]` of `doc`, removed (an empty table without it).
fn take_table(doc: &mut toml::Table, key: &str) -> anyhow::Result<toml::Table> {
    match doc.remove(key) {
        Some(toml::Value::Table(t)) => Ok(t),
        None => Ok(toml::Table::new()),
        Some(_) => anyhow::bail!("[{key}] in the cell.toml template is not a table"),
    }
}

/// `[codesys_mcp]`: the IDE's MCP URL.
fn codesys_mcp(w: &Wiring, mcp: &mut toml::Table) {
    use toml::Value as V;
    mcp.insert("url".into(), V::String(w.mcp_url()));
    mcp.entry("token_env").or_insert_with(|| V::String("PLAN2CELL_CODESYS_MCP_TOKEN".into()));
    // Nobody works in this cell's IDE by hand (it is built fresh on every launch): a CODESYS the MCP
    // server did not start that is in its way and has no unsaved changes is closed, and the stage goes
    // on (plan2cell `other_codesys`; the shared cell's template keeps the default, refuse).
    mcp.insert("other_codesys".into(), V::String("close".into()));
}

/// `[plc_host]` of a CODESYS head (the projection's `plcHost`).
fn codesys_plc_host(w: &Wiring, plc: &mut toml::Table, region: Option<&str>) {
    use toml::Value as V;
    // An EKS cell's PLC pod: kubectl exec (exactly one way: no ssm_instance then).
    match &w.plc_host.kubectl_pod {
        Some(t) => {
            plc.remove("ssm_instance");
            plc.insert("kubectl_pod".into(), V::String(t.clone()));
            plc.insert("kubeconfig".into(), V::String(KUBECONFIG_PATH.to_owned()));
        }
        None => {
            plc.remove("kubectl_pod");
            plc.insert("ssm_instance".into(), V::String(w.plc_host.instance_id.clone()));
        }
    }
    if let Some(region) = region {
        plc.entry("region").or_insert_with(|| V::String(region.to_owned()));
    }
}

/// The template's `[[kuka_vc.controller]]` entries, removed from `kuka`.
fn old_controllers(kuka: &mut toml::Table) -> Vec<toml::Table> {
    match kuka.remove("controller") {
        Some(toml::Value::Array(list)) => list.into_iter().filter_map(|c| c.as_table().cloned()).collect(),
        _ => Vec::new(),
    }
}

/// One `[[kuka_vc.controller]]` per KUKA VC of the projection (`kukaVc`), the template's entry of
/// the same `robot_ns` for the defaults.
fn kss_controllers(w: &Wiring, old: &[toml::Table]) -> Vec<toml::Value> {
    use toml::Value as V;
    w.kuka_vc
        .iter()
        .map(|k| {
            let ns = k.robot_ns();
            let mut c = old.iter().find(|c| c.get("robot_ns").and_then(V::as_str) == Some(ns.as_str())).cloned().unwrap_or_default();
            c.remove("ssh_host");
            c.insert("robot_ns".into(), V::String(ns));
            match &k.kubectl_pod {
                Some(t) => {
                    c.remove("ssm_instance");
                    c.insert("kubectl_pod".into(), V::String(t.clone()));
                }
                None => {
                    c.remove("kubectl_pod");
                    c.insert("ssm_instance".into(), V::String(k.instance_id.clone()));
                }
            }
            let mut set = |key: &str, value: Option<&String>, default: &str| match value {
                Some(v) => {
                    c.insert(key.into(), V::String(v.clone()));
                }
                None => {
                    c.entry(key).or_insert_with(|| V::String(default.to_owned()));
                }
            };
            set("instance", k.instance.as_ref(), KUKA_INSTANCE);
            set("netns", k.netns.as_ref(), "krc1");
            set("sps_base", None, SPS_BASE);
            if let Some(v) = &k.kli_ip {
                c.insert("kli_ip".into(), V::String(v.clone()));
            }
            if let Some(v) = &k.station {
                c.insert("station".into(), V::String(v.clone()));
            }
            V::Table(c)
        })
        .collect()
}

/// `[address_map]` and `[profinet]` (the PROFINET segment as the deployability check expects it,
/// `plan2cell cell check`).
fn address_map_and_profinet(w: &Wiring, doc: &mut toml::Table) -> anyhow::Result<()> {
    use toml::Value as V;
    let map: toml::Table = w.address_map.iter().map(|(k, v)| (k.clone(), V::String(v.clone()))).collect();
    doc.insert("address_map".into(), V::Table(map));
    let mut profinet = take_table(doc, "profinet")?;
    if !w.profinet.gateway.is_empty() {
        profinet.insert("gateway".into(), V::String(w.profinet.gateway.clone()));
    }
    if !w.profinet.head.is_empty() {
        profinet.insert("head_ip".into(), V::String(w.profinet.head.clone()));
    }
    if let Some(mask) = cidr_netmask(&w.profinet.cidr) {
        profinet.insert("netmask".into(), V::String(mask));
    }
    if !profinet.is_empty() {
        doc.insert("profinet".into(), V::Table(profinet));
    }
    Ok(())
}

fn cell_toml_header(w: &Wiring) -> String {
    format!(
        "# Written by Physical AI Production Designer (services/cell_wiring.rs) for launch {} of this workspace's virtual cell.\n\
         # Rewritten whenever a launch's addresses change; do not edit.\n\n",
        w.launch
    )
}

/// `cell.toml` of a wiring with hook drivers (docs/tasks/cell-templates.md E8), from its components
/// (the top level's network): a hook head is `[plc_host] driver = "hook"` with its pod and the
/// kubeconfig (no SSM, no SSH), hook robots are `[[kuka_vc.controller]] driver = "hook"` with their
/// namespace, station, KLI address and pod (no instance, netns, sps_base). Builtin parts give the
/// keys [`render_cell_toml`] writes; `[codesys_mcp]` only with `deploy.head.codesys-mcp`.
pub fn render_hook_cell_toml(w: &Wiring, template: Option<&str>, region: Option<&str>, capabilities: &[String]) -> anyhow::Result<String> {
    use toml::Value as V;
    let has = |c: &str| capabilities.iter().any(|x| x == c);
    let mut doc: toml::Table = template.unwrap_or(CELL_BASE).parse().context("the cell.toml template is not valid TOML")?;
    let mut mcp = take_table(&mut doc, "codesys_mcp")?;
    if has("deploy.head.codesys-mcp") {
        codesys_mcp(w, &mut mcp);
        doc.insert("codesys_mcp".into(), V::Table(mcp));
    }
    let head = w.scoped(SEMANTIC_HEAD_PLC).next().context("the wiring has no head-plc component")?;
    let mut plc = take_table(&mut doc, "plc_host")?;
    if head.hook() {
        let pod = head.kubectl_pod.as_deref().filter(|p| valid_kubectl_pod(p)).with_context(|| format!("hook head {}: no kubectlPod", head.name))?;
        // Only the hook's keys: the template's CODESYS host keys (ssh_host, setup, ...) are not this head's.
        plc = toml::Table::new();
        plc.insert("driver".into(), V::String(DRIVER_HOOK.into()));
        plc.insert("kubectl_pod".into(), V::String(pod.to_owned()));
        plc.insert("kubeconfig".into(), V::String(KUBECONFIG_PATH.to_owned()));
        if let Some(region) = region {
            plc.insert("region".into(), V::String(region.to_owned()));
        }
    } else {
        codesys_plc_host(w, &mut plc, region);
    }
    doc.insert("plc_host".into(), V::Table(plc));
    let mut kuka = take_table(&mut doc, "kuka_vc")?;
    let old = old_controllers(&mut kuka);
    let robots: Vec<&WiringComponent> = w.scoped(SEMANTIC_ROBOT_CONTROLLER).collect();
    let hook = robots.iter().filter(|r| r.hook()).count();
    anyhow::ensure!(hook == 0 || hook == robots.len(), "the wiring mixes hook and KSS robot controllers");
    if hook == 0 {
        let controllers = kss_controllers(w, &old);
        if let Some(region) = region {
            kuka.entry("region").or_insert_with(|| V::String(region.to_owned()));
        }
        if !controllers.is_empty() {
            if w.kuka_vc.iter().any(|k| k.kubectl_pod.is_some()) {
                kuka.insert("kubeconfig".into(), V::String(KUBECONFIG_PATH.to_owned()));
            }
            kuka.insert("controller".into(), V::Array(controllers));
            doc.insert("kuka_vc".into(), V::Table(kuka));
        }
    } else {
        let mut controllers = Vec::new();
        for r in robots {
            let (Some(ns), Some(pod)) = (r.robot_ns.as_deref().filter(|n| valid_name(n)), r.kubectl_pod.as_deref().filter(|p| valid_kubectl_pod(p))) else {
                anyhow::bail!("hook robot-controller {}: no robotNs or kubectlPod", r.name);
            };
            let mut c = toml::Table::new();
            c.insert("driver".into(), V::String(DRIVER_HOOK.into()));
            c.insert("robot_ns".into(), V::String(ns.to_owned()));
            if let Some(station) = r.station.as_deref().filter(|s| valid_name(s)) {
                c.insert("station".into(), V::String(station.to_owned()));
            }
            if let Some(kli) = r.address.as_deref().filter(|a| a.parse::<std::net::Ipv4Addr>().is_ok()) {
                c.insert("kli_ip".into(), V::String(kli.to_owned()));
            }
            c.insert("kubectl_pod".into(), V::String(pod.to_owned()));
            controllers.push(V::Table(c));
        }
        kuka.insert("kubeconfig".into(), V::String(KUBECONFIG_PATH.to_owned()));
        if let Some(region) = region {
            kuka.entry("region").or_insert_with(|| V::String(region.to_owned()));
        }
        kuka.insert("controller".into(), V::Array(controllers));
        doc.insert("kuka_vc".into(), V::Table(kuka));
    }
    address_map_and_profinet(w, &mut doc)?;
    Ok(cell_toml_header(w) + &toml::to_string_pretty(&doc)?)
}

/// A Kubernetes object name or a robot namespace / station (`[a-z0-9]` and `-`, at most 63).
fn valid_name(s: &str) -> bool {
    (1..=63).contains(&s.len()) && s.chars().all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-') && !s.starts_with('-') && !s.ends_with('-')
}

/// `<namespace>/<pod>/<container>`, each a [`valid_name`].
fn valid_kubectl_pod(s: &str) -> bool {
    let parts: Vec<&str> = s.split('/').collect();
    parts.len() == 3 && parts.iter().all(|p| valid_name(p))
}

/// The cell.toml of `w` for a cell of these template `capabilities` (`None`: no template): with a
/// hook driver from the components ([`render_hook_cell_toml`]), else today's ([`render_cell_toml`]).
pub fn render_cell_toml_for(w: &Wiring, template: Option<&str>, region: Option<&str>, capabilities: Option<&[String]>) -> anyhow::Result<String> {
    match capabilities.filter(|_| w.has_hook()) {
        Some(caps) => render_hook_cell_toml(w, template, region, caps),
        None => render_cell_toml(w, template, region),
    }
}

/// `192.168.100.0/24` → `255.255.255.0`.
fn cidr_netmask(cidr: &str) -> Option<String> {
    let bits: u32 = cidr.split_once('/')?.1.trim().parse().ok().filter(|b| *b <= 32)?;
    let mask = if bits == 0 { 0 } else { u32::MAX << (32 - bits) };
    Some(std::net::Ipv4Addr::from(mask).to_string())
}

/// `scheme://host:port...` with `host` mapped.
fn map_endpoint(w: &Wiring, endpoint: &str) -> String {
    let (scheme, rest) = endpoint.split_once("://").or_else(|| endpoint.split_once('/')).map_or(("", endpoint), |(s, r)| (s, r));
    let sep = if endpoint.contains("://") { "://" } else { "/" };
    let (host, tail) = rest.split_once(':').map_or((rest, None), |(h, t)| (h, Some(t)));
    let host = w.map(host);
    let joined = match tail {
        Some(t) => format!("{host}:{t}"),
        None => host.to_owned(),
    };
    if scheme.is_empty() { joined } else { format!("{scheme}{sep}{joined}") }
}

/// The live bridge's config: `template` with `[opcua] endpoint` and every `[[robot]] endpoint`
/// mapped through `[address_map]`. An EKS network (its `profinet.head`): the head's endpoint on the
/// network's head address, and one `[[robot]]` per KUKA VC of the wiring (robotN: its KLI's vass-io,
/// controller 1 of its own pod), the template's entry of that robot (or its first) for the rest; the
/// template is the EC2 cell's two robots on 192.168.100.x.
pub fn render_bridge_toml(w: &Wiring, template: &str) -> anyhow::Result<String> {
    let mut doc: toml::Table = template.parse().context("the fos-live-bridge.toml template is not valid TOML")?;
    if !w.profinet.head.is_empty() {
        if let Some(e) = doc.get_mut("opcua").and_then(|o| o.get_mut("endpoint"))
            && let Some(s) = e.as_str()
        {
            let port = s.rsplit_once(':').map(|(_, p)| p).filter(|p| p.chars().all(|c| c.is_ascii_digit())).unwrap_or("4840");
            *e = toml::Value::String(format!("opc.tcp://{}:{port}", w.profinet.head));
        }
        let template_robots: Vec<toml::Table> = doc
            .get("robot")
            .and_then(|r| r.as_array())
            .map(|a| a.iter().filter_map(|r| r.as_table().cloned()).collect())
            .unwrap_or_default();
        let robots: Vec<toml::Value> = w
            .kuka_vc
            .iter()
            .filter_map(|k| {
                let ns = k.robot_ns();
                let kli = k.kli_ip.as_deref().filter(|ip| !ip.is_empty())?;
                let mut r = template_robots
                    .iter()
                    .find(|t| t.get("robot_id").and_then(|v| v.as_str()) == Some(ns.as_str()))
                    .or_else(|| template_robots.first())
                    .cloned()
                    .unwrap_or_default();
                let n = ns.trim_start_matches("robot");
                r.insert("robot_id".into(), toml::Value::String(ns.clone()));
                r.insert("name".into(), toml::Value::String(format!("R{n}")));
                r.insert("endpoint".into(), toml::Value::String(format!("tcp/{kli}:7447")));
                r.insert("prefix".into(), toml::Value::String("kuka/vass/r1".into()));
                Some(toml::Value::Table(r))
            })
            .collect();
        if !robots.is_empty() {
            doc.insert("robot".into(), toml::Value::Array(robots));
        }
    }
    if let Some(e) = doc.get_mut("opcua").and_then(|o| o.get_mut("endpoint"))
        && let Some(s) = e.as_str()
    {
        *e = toml::Value::String(map_endpoint(w, s));
    }
    if let Some(robots) = doc.get_mut("robot").and_then(|r| r.as_array_mut()) {
        for r in robots.iter_mut().filter_map(|r| r.as_table_mut()) {
            if let Some(s) = r.get("endpoint").and_then(|e| e.as_str()).map(str::to_owned) {
                r.insert("endpoint".into(), toml::Value::String(map_endpoint(w, &s)));
            }
        }
    }
    let header = format!("# Written by Physical AI Production Designer for launch {} of this workspace's virtual cell (mapped endpoints).\n\n", w.launch);
    Ok(header + &toml::to_string_pretty(&doc)?)
}

/// The live bridge's config of a wiring with hook drivers (docs/tasks/cell-templates.md E10), by the
/// cell's live capabilities, from the components of the top level's network: `live.head.exec`
/// turns OPC UA off and reads the head's `watch` over `[exec] head`, `live.head.opcua` keeps the
/// builtin head's OPC UA endpoint; `live.robot.exec` gives one `[[robot]] source = "exec"` per hook
/// robot, `live.robot.vass-zenoh` the builtin robots' Zenoh endpoints. Without a head source OPC UA
/// is off; without a robot source there is no `[[robot]]`. The template's entry of a robot (or its
/// first) gives the rest, as [`render_bridge_toml`] does.
pub fn render_hook_bridge_toml(w: &Wiring, template: &str, capabilities: &[String]) -> anyhow::Result<String> {
    use toml::Value as V;
    let has = |c: &str| capabilities.iter().any(|x| x == c);
    let mut doc: toml::Table = template.parse().context("the fos-live-bridge.toml template is not valid TOML")?;
    let argv = || V::Array(vec![V::String(HOOK_PATH.into()), V::String("watch".into())]);
    let off = || {
        let mut t = toml::Table::new();
        t.insert("enabled".into(), V::Boolean(false));
        V::Table(t)
    };
    let head = w.scoped(SEMANTIC_HEAD_PLC).next();
    let mut exec_head = None;
    if has("live.head.exec") {
        let h = head.filter(|h| h.hook()).context("live.head.exec: no hook head-plc component")?;
        let (pod, container) = h.exec_target().with_context(|| format!("hook head {}: no kubectlPod", h.name))?;
        let mut t = toml::Table::new();
        t.insert("pod".into(), V::String(pod));
        t.insert("container".into(), V::String(container));
        t.insert("argv".into(), argv());
        exec_head = Some(t);
        doc.insert("opcua".into(), off());
    } else if let Some(endpoint) = head.and_then(|h| h.endpoints.get("opcua")).filter(|e| has("live.head.opcua") && e.starts_with("opc.tcp://")) {
        let mut opcua = take_table(&mut doc, "opcua")?;
        opcua.insert("endpoint".into(), V::String(endpoint.clone()));
        doc.insert("opcua".into(), V::Table(opcua));
    } else {
        doc.insert("opcua".into(), off());
    }
    let template_robots: Vec<toml::Table> =
        doc.get("robot").and_then(V::as_array).map(|a| a.iter().filter_map(|r| r.as_table().cloned()).collect()).unwrap_or_default();
    let base = |ns: &str| -> toml::Table {
        let mut r = template_robots
            .iter()
            .find(|t| t.get("robot_id").and_then(V::as_str) == Some(ns))
            .or_else(|| template_robots.first())
            .cloned()
            .unwrap_or_default();
        for key in ["endpoint", "prefix", "source", "pod", "container", "argv"] {
            r.remove(key);
        }
        r.insert("robot_id".into(), V::String(ns.to_owned()));
        r.insert("name".into(), V::String(format!("R{}", ns.trim_start_matches("robot"))));
        r
    };
    let mut robots = Vec::new();
    for r in w.scoped(SEMANTIC_ROBOT_CONTROLLER) {
        let Some(ns) = r.robot_ns.as_deref().filter(|n| valid_name(n)) else { continue };
        if has("live.robot.exec") && r.hook() {
            let (pod, container) = r.exec_target().with_context(|| format!("hook robot-controller {}: no kubectlPod", r.name))?;
            let mut t = base(ns);
            t.insert("source".into(), V::String("exec".into()));
            t.insert("pod".into(), V::String(pod));
            t.insert("container".into(), V::String(container));
            t.insert("argv".into(), argv());
            robots.push(V::Table(t));
        } else if has("live.robot.vass-zenoh") && !r.hook()
            && let Some(endpoint) = r.endpoints.get("zenoh").filter(|e| e.starts_with("tcp/"))
        {
            let mut t = base(ns);
            t.insert("endpoint".into(), V::String(endpoint.clone()));
            t.insert("prefix".into(), V::String("kuka/vass/r1".into()));
            robots.push(V::Table(t));
        }
    }
    if robots.is_empty() {
        doc.remove("robot");
    } else {
        doc.insert("robot".into(), V::Array(robots.clone()));
    }
    let exec_robots = robots.iter().any(|r| r.get("source").and_then(V::as_str) == Some("exec"));
    if exec_head.is_some() || exec_robots {
        let mut exec = toml::Table::new();
        exec.insert("kubeconfig".into(), V::String(KUBECONFIG_PATH.into()));
        exec.insert("period_ms".into(), V::Integer(WATCH_PERIOD_MS));
        exec.insert("max_s".into(), V::Integer(WATCH_MAX_S));
        if let Some(h) = exec_head {
            exec.insert("head".into(), V::Table(h));
        }
        doc.insert("exec".into(), V::Table(exec));
    } else {
        doc.remove("exec");
    }
    let header = format!("# Written by Physical AI Production Designer for launch {} of this workspace's virtual cell (hook drivers).\n\n", w.launch);
    Ok(header + &toml::to_string_pretty(&doc)?)
}

/// The live bridge's config of `w` for a cell of these template `capabilities` (`None`: no
/// template): with a hook driver [`render_hook_bridge_toml`], else today's [`render_bridge_toml`].
pub fn render_bridge_toml_for(w: &Wiring, template: &str, capabilities: Option<&[String]>) -> anyhow::Result<String> {
    match capabilities.filter(|_| w.has_hook()) {
        Some(caps) => render_hook_bridge_toml(w, template, caps),
        None => render_bridge_toml(w, template),
    }
}

/// An EKS cell's kubeconfig: its cluster (endpoint, CA) and credentials from `aws eks get-token` with this
/// workspace's own role (its access entry: group `factoryos:ws-<alias>`, RBAC in its cell's namespace only). `None`:
/// not an EKS cell, or a cluster without a valid endpoint.
pub fn render_kubeconfig(w: &Wiring, region: Option<&str>) -> Option<String> {
    let k = w.kubernetes.as_ref()?;
    let host_ok = k.endpoint.strip_prefix("https://").is_some_and(|h| !h.is_empty() && h.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '.' | '-')));
    let name_ok = !k.cluster.is_empty() && k.cluster.chars().all(|c| c.is_ascii_alphanumeric() || c == '-');
    let ca_ok = !k.ca.is_empty() && k.ca.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '+' | '/' | '='));
    if !(host_ok && name_ok && ca_ok) {
        return None;
    }
    // The region of `<id>.<suffix>.<region>.eks.amazonaws.com`, else the backend's.
    let parts: Vec<&str> = k.endpoint.trim_start_matches("https://").split('.').collect();
    let region = parts.iter().position(|p| *p == "eks").and_then(|i| i.checked_sub(1)).map(|i| parts[i]).or(region)?;
    Some(format!(
        "apiVersion: v1\nkind: Config\nclusters:\n- name: {c}\n  cluster:\n    server: {e}\n    certificate-authority-data: {ca}\n\
         users:\n- name: factoryos-workspace\n  user:\n    exec:\n      apiVersion: client.authentication.k8s.io/v1beta1\n      command: aws\n\
         \x20     args: [eks, get-token, --cluster-name, {c}, --region, {r}, --output, json]\n      interactiveMode: Never\n\
         contexts:\n- name: {c}\n  context:\n    cluster: {c}\n    user: factoryos-workspace\ncurrent-context: {c}\n",
        c = k.cluster,
        e = k.endpoint,
        ca = k.ca,
        r = region
    ))
}

pub fn render_dcv_env(w: &Wiring) -> String {
    format!("# Written by Physical AI Production Designer for launch {}: the virtual cell's Windows IDE.\nFACTORY_OS_DCV_SERVER={}\n", w.launch, w.dcv_host())
}

/// Where the files go and their templates.
#[derive(Clone, Debug)]
pub struct WireFiles {
    pub dir: PathBuf,
    pub cell_template: PathBuf,
    pub bridge_template: PathBuf,
    pub region: Option<String>,
}

impl WireFiles {
    pub fn cell_toml(&self) -> PathBuf {
        self.dir.join(CELL_TOML)
    }

    /// The generation the files have (`None`: never written).
    pub fn current(&self) -> Option<String> {
        std::fs::read_to_string(self.dir.join(GENERATION)).ok().map(|s| s.trim().to_owned())
    }

    /// Write the wiring's files unless they already are this wiring's; `true` when written.
    /// `capabilities`: the cell's template's ([`Wiring::complete_for`]; `None`: no template).
    pub fn write(&self, w: &Wiring, capabilities: Option<&[String]>) -> anyhow::Result<bool> {
        anyhow::ensure!(w.complete_for(capabilities), "the wiring has no PLC host or Windows IDE (or the hosts its template needs) yet");
        let generation = w.generation();
        if self.current().as_deref() == Some(generation.as_str()) {
            return Ok(false);
        }
        std::fs::create_dir_all(&self.dir).with_context(|| format!("creating {}", self.dir.display()))?;
        let template = std::fs::read_to_string(&self.cell_template).ok();
        let region = self.region.as_deref();
        write_atomic(&self.cell_toml(), render_cell_toml_for(w, template.as_deref(), region, capabilities)?.as_bytes())?;
        // An EKS cell's other networks: one plan2cell cell config each (`plan2cell cell ... --cell-config cell-n2.toml`).
        for (name, n) in w.extra_networks() {
            write_atomic(&self.dir.join(format!("cell-{name}.toml")), render_cell_toml_for(&n, template.as_deref(), region, capabilities)?.as_bytes())?;
        }
        match read_bridge_template(&self.bridge_template, Path::new(BRIDGE_TEMPLATE), Path::new(BRIDGE_TEMPLATE_SHARED)) {
            Ok(t) => {
                let mut text = render_bridge_toml_for(w, &t, capabilities)?;
                if let Some(export) = self.bridge_export() {
                    text = with_export(&text, &export).unwrap_or(text);
                }
                write_atomic(&self.dir.join(BRIDGE_TOML), text.as_bytes())?
            }
            Err(err) => tracing::warn!("Virtual Cell: no live bridge config ({}: {err}); fos-live-bridge keeps its own", self.bridge_template.display()),
        }
        // A hook cell without an IDE has no DCV server (and no older launch's).
        if w.has_hook() && w.dcv_host().is_empty() {
            std::fs::remove_file(self.dir.join(DCV_ENV)).ok();
        } else {
            write_atomic(&self.dir.join(DCV_ENV), render_dcv_env(w).as_bytes())?;
        }
        if let Some(k) = render_kubeconfig(w, self.region.as_deref()) {
            write_atomic(&self.dir.join(KUBECONFIG), k.as_bytes())?;
            #[cfg(unix)]
            {
                use std::os::unix::fs::PermissionsExt as _;
                std::fs::set_permissions(self.dir.join(KUBECONFIG), std::fs::Permissions::from_mode(0o600))?;
            }
        }
        write_atomic(&self.dir.join(GENERATION), format!("{generation}\n").as_bytes())?;
        Ok(true)
    }
}

impl WireFiles {
    fn bridge_export(&self) -> Option<BridgeExport> {
        serde_json::from_slice(&std::fs::read(self.dir.join(BRIDGE_EXPORT)).ok()?).ok()
    }

    /// The live bridge follows `export` from now on (a deploy of it onto the cell): its config is
    /// rewritten, and the generation file again (unchanged), which restarts the bridge through
    /// `factoryos-cell-wire.path`. `false`: no wired cell yet (the next launch's files get it).
    pub fn apply_export(&self, export: &BridgeExport) -> anyhow::Result<bool> {
        std::fs::create_dir_all(&self.dir).with_context(|| format!("creating {}", self.dir.display()))?;
        write_atomic(&self.dir.join(BRIDGE_EXPORT), &serde_json::to_vec_pretty(export)?)?;
        let path = self.dir.join(BRIDGE_TOML);
        let (Some(generation), Ok(text)) = (self.current(), std::fs::read_to_string(&path)) else { return Ok(false) };
        let updated = with_export(&text, export)?;
        if updated == text {
            return Ok(false);
        }
        write_atomic(&path, updated.as_bytes())?;
        write_atomic(&self.dir.join(GENERATION), format!("{generation}\n").as_bytes())?;
        Ok(true)
    }
}

/// `text` (a fos-live-bridge config) with `export`'s cell plan applied ([`apply_cell_plan`]).
fn with_export(text: &str, export: &BridgeExport) -> anyhow::Result<String> {
    let plan: serde_json::Value = serde_json::from_slice(
        &std::fs::read(&export.cell_plan).with_context(|| format!("reading {}", export.cell_plan.display()))?,
    )?;
    let (header, body): (String, &str) = match text.find("\n\n") {
        Some(i) if text.starts_with('#') => (text[..i + 2].to_owned(), &text[i + 2..]),
        _ => (String::new(), text),
    };
    let mut doc: toml::Table = body.parse().context("the live bridge config is not valid TOML")?;
    apply_cell_plan(&mut doc, &export.cell, &plan);
    Ok(header + &toml::to_string_pretty(&doc)?)
}

/// Write `path` through a temporary file next to it and a rename (readers never see half a file).
fn write_atomic(path: &Path, bytes: &[u8]) -> anyhow::Result<()> {
    let tmp = path.with_extension(format!("tmp-{}", std::process::id()));
    std::fs::write(&tmp, bytes).with_context(|| format!("writing {}", tmp.display()))?;
    std::fs::rename(&tmp, path).with_context(|| format!("writing {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    pub(crate) fn wiring(launch: u32) -> Wiring {
        serde_json::from_value(serde_json::json!({
            "launch": launch,
            "plcHost": {"instanceId": "i-plc", "ip": "10.0.5.10"},
            "windowsIde": {"instanceId": "i-ide", "ip": "10.0.5.20", "mcpUrl": "http://10.0.5.20:8766/mcp", "dcvServer": "10.0.5.20:8443"},
            "kukaVc": [
                {"role": "kukaVc1", "instanceId": "i-k1", "ip": "10.0.5.31", "station": "kuka-robi1", "kliIp": "192.168.100.11", "netns": "krc1", "instance": "iiQKA.OS2_KSS_9.2.0_vass"},
                {"role": "kukaVc2", "instanceId": "i-k2", "ip": "10.0.5.32", "station": "kuka-robi2", "kliIp": "192.168.100.12"}
            ],
            "profinet": {"cidr": "192.168.100.0/24", "gateway": "192.168.100.1", "vni": 1101},
            "addressMap": {"192.168.100.10": "10.0.5.11", "192.168.100.11": "10.0.5.12", "192.168.100.12": "10.0.5.13"},
            "mcpTokenSecret": "factoryos/cell/ws-jdoe-1"
        }))
        .unwrap()
    }

    #[test]
    fn cell_toml_has_the_instances_and_the_address_map() {
        let w = wiring(3);
        assert!(w.complete());
        assert_eq!(w.kuka_vc[1].robot_ns(), "robot2");
        let template = "[codesys_mcp]\nurl = \"http://old:8766/mcp\"\ninstall = \"SP22 Patch 1\"\n[plc_host]\nssm_instance = \"i-old\"\nbucket = \"b\"\n\
                        [kuka_vc]\nregion = \"eu-central-1\"\n[[kuka_vc.controller]]\nrobot_ns = \"robot2\"\ninstance = \"X\"\nnetns = \"krc9\"\nssm_instance = \"i-old2\"\nsps_base = \"S\"\n";
        let text = render_cell_toml(&w, Some(template), Some("eu-central-1")).unwrap();
        let doc: toml::Table = text.parse().unwrap();
        assert_eq!(doc["codesys_mcp"]["url"].as_str(), Some("http://10.0.5.20:8766/mcp"));
        assert_eq!(doc["codesys_mcp"]["other_codesys"].as_str(), Some("close"));
        assert_eq!(doc["codesys_mcp"]["install"].as_str(), Some("SP22 Patch 1"));
        assert_eq!(doc["plc_host"]["ssm_instance"].as_str(), Some("i-plc"));
        assert_eq!(doc["plc_host"]["bucket"].as_str(), Some("b"));
        let c = doc["kuka_vc"]["controller"].as_array().unwrap();
        assert_eq!(c.len(), 2);
        assert_eq!((c[0]["robot_ns"].as_str(), c[0]["ssm_instance"].as_str(), c[0]["sps_base"].as_str()), (Some("robot1"), Some("i-k1"), Some(SPS_BASE)));
        // The template's robot2 keeps what the wiring does not say.
        assert_eq!((c[1]["instance"].as_str(), c[1]["netns"].as_str(), c[1]["sps_base"].as_str()), (Some("X"), Some("krc9"), Some("S")));
        assert_eq!(c[1]["ssm_instance"].as_str(), Some("i-k2"));
        assert_eq!(c[1]["kli_ip"].as_str(), Some("192.168.100.12"));
        assert_eq!(doc["address_map"]["192.168.100.10"].as_str(), Some("10.0.5.11"));
        // The PROFINET segment for plan2cell cell check.
        assert_eq!(doc["profinet"]["gateway"].as_str(), Some("192.168.100.1"));
        assert_eq!(doc["profinet"]["netmask"].as_str(), Some("255.255.255.0"));
        assert!(doc["profinet"].get("head_ip").is_none(), "EC2: the default head");
        // The built-in base parses as a plan2cell cell.toml would.
        let doc: toml::Table = render_cell_toml(&w, None, None).unwrap().parse().unwrap();
        assert_eq!(doc["codesys_mcp"]["token_env"].as_str(), Some("PLAN2CELL_CODESYS_MCP_TOKEN"));
        assert_eq!(doc["kuka_vc"]["controller"][1]["instance"].as_str(), Some(KUKA_INSTANCE));
    }

    /// An EKS cell's wiring (plan2cell/infra/lambda/cell-control/eks.js): the pods' managed instances (mi-), their VPC
    /// IPs in the address map, a container IDE without DCV (`dcvServer: null`) and the Lambda's extra keys.
    #[test]
    fn an_eks_cells_wiring_renders_as_an_ec2_ones() {
        let w: Wiring = serde_json::from_value(serde_json::json!({
            "launch": 1, "runtime": "eks", "ide": "container", "namespace": "cell-jdoe",
            "plcHost": {"instanceId": "mi-0123456789abcdef0", "ip": "10.0.60.11"},
            "windowsIde": {"instanceId": "pod/cell-jdoe/ide", "ip": "10.0.60.40", "mcpUrl": "http://10.0.60.40:8766/mcp", "dcvServer": null},
            "kukaVc": [{"role": "kukaVc1", "instanceId": "mi-1234567890abcdef0", "ip": "10.0.60.21", "station": "kuka-robi1", "kliIp": "192.168.100.11", "netns": "krc1", "instance": "iiQKA.OS2_KSS_9.2.0_vass"}],
            "profinet": {"cidr": "192.168.100.0/24", "gateway": "192.168.100.1", "vni": 1101, "overlay": "multus bridge pn1101 + VXLAN 1101 across the nodes"},
            "addressMap": {"192.168.100.10": "10.0.60.11", "192.168.100.11": "10.0.60.21"},
            "mcpTokenSecret": "factoryos/cell/ws-jdoe/1"  // pragma: allowlist secret
        }))
        .unwrap();
        assert!(w.complete());
        let doc: toml::Table = render_cell_toml(&w, None, Some("eu-central-1")).unwrap().parse().unwrap();
        assert_eq!(doc["plc_host"]["ssm_instance"].as_str(), Some("mi-0123456789abcdef0"));
        assert_eq!(doc["kuka_vc"]["controller"][0]["ssm_instance"].as_str(), Some("mi-1234567890abcdef0"));
        assert_eq!(doc["codesys_mcp"]["url"].as_str(), Some("http://10.0.60.40:8766/mcp"));
        assert_eq!(doc["address_map"]["192.168.100.10"].as_str(), Some("10.0.60.11"));
        assert!(w.extra_networks().is_empty());
    }

    /// An EKS cell's pods over kubectl exec: cell.toml has kubectl_pod (no ssm_instance) and the kubeconfig; the
    /// kubeconfig points at the cluster with the workspace's own role (aws eks get-token).
    #[test]
    fn an_eks_cell_is_reached_with_kubectl() {
        let w: Wiring = serde_json::from_value(serde_json::json!({
            "launch": 2,
            "plcHost": {"instanceId": "pod/cell-jdoe/n1-plc", "ip": "10.0.60.11", "kubectlPod": "cell-jdoe/n1-plc/agent"},
            "windowsIde": {"instanceId": "pod/cell-jdoe/compile", "ip": "10.0.60.40", "mcpUrl": "http://10.0.60.40:8766/mcp"},
            "kukaVc": [{"role": "kukaVc1", "instanceId": "pod/cell-jdoe/n1-kvc-1", "ip": "10.0.60.21", "kubectlPod": "cell-jdoe/n1-kvc-1/kvc", "kliIp": "10.77.1.11"}],
            "profinet": {"cidr": "10.77.1.0/24", "gateway": "10.77.1.1", "vni": 1101},
            "addressMap": {"10.77.1.10": "10.0.60.11", "10.77.1.11": "10.0.60.21"},
            "kubernetes": {"cluster": "factoryos-cells", "endpoint": "https://ABC123.gr7.eu-central-1.eks.amazonaws.com", "ca": "Q0E="}
        }))
        .unwrap();
        let template = "[codesys_mcp]\nurl = \"http://old:8766/mcp\"\n[plc_host]\nssm_instance = \"i-old\"\n[kuka_vc]\n[[kuka_vc.controller]]\nrobot_ns = \"robot1\"\ninstance = \"X\"\nssm_instance = \"i-k\"\n";
        let doc: toml::Table = render_cell_toml(&w, Some(template), Some("eu-central-1")).unwrap().parse().unwrap();
        assert_eq!(doc["plc_host"]["kubectl_pod"].as_str(), Some("cell-jdoe/n1-plc/agent"));
        assert!(doc["plc_host"].get("ssm_instance").is_none());
        assert_eq!(doc["plc_host"]["kubeconfig"].as_str(), Some(KUBECONFIG_PATH));
        let c = &doc["kuka_vc"]["controller"][0];
        assert_eq!(c["kubectl_pod"].as_str(), Some("cell-jdoe/n1-kvc-1/kvc"));
        assert!(c.get("ssm_instance").is_none());
        assert_eq!(doc["kuka_vc"]["kubeconfig"].as_str(), Some(KUBECONFIG_PATH));
        let k = render_kubeconfig(&w, None).unwrap();
        assert!(k.contains("server: https://ABC123.gr7.eu-central-1.eks.amazonaws.com"));
        assert!(k.contains("args: [eks, get-token, --cluster-name, factoryos-cells, --region, eu-central-1, --output, json]"));
        assert_line_yaml(&k);
        // Anything odd in the cluster's fields: no kubeconfig.
        let mut bad = w.clone();
        bad.kubernetes.as_mut().unwrap().endpoint = "https://evil.example\nx: y".into();
        assert!(render_kubeconfig(&bad, Some("eu-central-1")).is_none());
        // An EC2 cell: none, and cell.toml as before.
        assert!(render_kubeconfig(&wiring(1), None).is_none());
    }

    /// The kubeconfig is line-based YAML: every line is `key: value`, a list item or indented (no stray lines).
    fn assert_line_yaml(text: &str) {
        for l in text.lines() {
            assert!(l.contains(": ") || l.ends_with(':') || l.trim_start().starts_with("- "), "{l:?}");
        }
    }

    /// An EKS cell of two networks: the top level wires n1, `extra_networks` gives n2 as a cell of its own.
    #[test]
    fn an_eks_cells_second_network_is_a_cell_of_its_own() {
        let net = |name: &str, k: u32, plc: &str, kvc: &str| serde_json::json!({
            "name": name, "vni": 1100 + k, "subnet": format!("10.77.{k}.0/24"),
            "plcHost": {"instanceId": format!("pod/cell-jdoe/{name}-plc"), "ip": plc, "pod": format!("{name}-plc")},
            "windowsIde": {"instanceId": format!("pod/cell-jdoe/{name}-ide"), "ip": "10.0.60.40", "mcpUrl": "http://10.0.60.40:8766/mcp", "dcvServer": null},
            "kukaVc": [{"role": "kukaVc1", "instanceId": format!("pod/cell-jdoe/{name}-kvc-1"), "ip": kvc, "station": "kuka-robi1", "kliIp": format!("10.77.{k}.11")}],
            "profinet": {"cidr": format!("10.77.{k}.0/24"), "gateway": format!("10.77.{k}.1"), "head": format!("10.77.{k}.10"), "vni": 1100 + k},
            "addressMap": {format!("10.77.{k}.10"): plc, format!("10.77.{k}.11"): kvc}
        });
        let (n1, n2) = (net("n1", 1, "10.0.60.11", "10.0.60.21"), net("n2", 2, "10.0.60.12", "10.0.60.22"));
        let mut top = n1.clone();
        top["launch"] = serde_json::json!(3);
        top["mcpTokenSecret"] = serde_json::json!("factoryos/cell/ws-jdoe/1");
        top["networks"] = serde_json::json!([n1, n2]);
        let w: Wiring = serde_json::from_value(top).unwrap();
        assert_eq!(w.networks.len(), 2);
        let extra = w.extra_networks();
        assert_eq!(extra.len(), 1);
        let (name, n) = &extra[0];
        assert_eq!(name, "n2");
        assert_eq!((n.launch, n.mcp_token_secret.as_deref()), (3, Some("factoryos/cell/ws-jdoe/1")));
        let doc: toml::Table = render_cell_toml(n, None, Some("eu-central-1")).unwrap().parse().unwrap();
        assert_eq!(doc["address_map"]["10.77.2.10"].as_str(), Some("10.0.60.12"));
        assert_eq!(doc["kuka_vc"]["controller"][0]["kli_ip"].as_str(), Some("10.77.2.11"));
        // The deployability check's head address: the network's, not the EC2 cell's 192.168.100.10.
        assert_eq!(doc["profinet"]["head_ip"].as_str(), Some("10.77.2.10"));
    }

    /// An EKS network: the head on the network's address and one robot per KUKA VC, all through the address map.
    #[test]
    fn an_eks_cells_bridge_has_its_head_and_every_kuka_vc() {
        let kvc: Vec<serde_json::Value> = (1..=3)
            .map(|n| serde_json::json!({"role": format!("kukaVc{n}"), "instanceId": format!("pod/cell-jdoe/n1-kvc-{n}"),
                "ip": format!("10.0.60.2{n}"), "kliIp": format!("10.77.1.1{n}")}))
            .collect();
        let mut addr = serde_json::Map::new();
        addr.insert("10.77.1.10".into(), "10.0.60.11".into());
        for n in 1..=3 {
            addr.insert(format!("10.77.1.1{n}"), format!("10.0.60.2{n}").into());
        }
        let w: Wiring = serde_json::from_value(serde_json::json!({
            "launch": 4,
            "plcHost": {"instanceId": "pod/cell-jdoe/n1-plc", "ip": "10.0.60.11"},
            "windowsIde": {"instanceId": "pod/cell-jdoe/compile", "ip": "10.0.60.40"},
            "kukaVc": kvc,
            "profinet": {"cidr": "10.77.1.0/24", "gateway": "10.77.1.1", "head": "10.77.1.10", "vni": 1101},
            "addressMap": addr,
        }))
        .unwrap();
        let t = "[opcua]\nendpoint = \"opc.tcp://192.168.100.10:4840\"\n[[robot]]\nrobot_id = \"robot1\"\nmodel = \"KR240\"\nendpoint = \"tcp/192.168.100.11:7447\"\nprefix = \"kuka/vass/r1\"\n";
        let doc: toml::Table = render_bridge_toml(&w, t).unwrap().parse().unwrap();
        assert_eq!(doc["opcua"]["endpoint"].as_str(), Some("opc.tcp://10.0.60.11:4840"));
        let robots = doc["robot"].as_array().unwrap();
        let got: Vec<(&str, &str, &str)> = robots
            .iter()
            .map(|r| (r["robot_id"].as_str().unwrap(), r["endpoint"].as_str().unwrap(), r["model"].as_str().unwrap()))
            .collect();
        assert_eq!(got, [
            ("robot1", "tcp/10.0.60.21:7447", "KR240"),
            ("robot2", "tcp/10.0.60.22:7447", "KR240"),
            ("robot3", "tcp/10.0.60.23:7447", "KR240"),
        ]);
    }

    #[test]
    fn bridge_endpoints_are_mapped() {
        let w = wiring(1);
        let t = "[opcua]\nendpoint = \"opc.tcp://192.168.100.10:4840\"\n[[robot]]\nendpoint = \"tcp/192.168.100.11:7447\"\n[[robot]]\nendpoint = \"tcp/10.0.0.1:7447\"\n";
        let doc: toml::Table = render_bridge_toml(&w, t).unwrap().parse().unwrap();
        assert_eq!(doc["opcua"]["endpoint"].as_str(), Some("opc.tcp://10.0.5.11:4840"));
        assert_eq!(doc["robot"][0]["endpoint"].as_str(), Some("tcp/10.0.5.12:7447"));
        assert_eq!(doc["robot"][1]["endpoint"].as_str(), Some("tcp/10.0.0.1:7447"), "not in the map: as it is");
        assert_eq!(render_dcv_env(&w).lines().last(), Some("FACTORY_OS_DCV_SERVER=10.0.5.20"));
    }

    const CELL_PLAN: &str = r#"{"config": {"protocol": {"step_byte": 4, "request_byte": 28, "grant_byte": 12, "plan_id_byte": 4}},
        "zones": [{"name": "IL02", "bit": 1}, {"name": "IL01", "bit": 0}],
        "robots": [{"ns": "robot1", "tag": "R1", "ops": ["W02", "W04", "HOME"]}, {"ns": "robot2", "tag": "R2", "ops": ["W01", "W03", "HOME"]}]}"#;

    #[test]
    fn the_bridge_follows_the_deployed_export() {
        let mut doc: toml::Table = "[backend]\ncell = \"welding_cell_2_small\"\nurl = \"http://127.0.0.1\"\n[protocol]\nzones = [\"Z9\"]\n\
            [[robot]]\nrobot_id = \"robot1\"\nname = \"R1\"\nops = [\"W01\", \"W02\", \"W03\"]\n[[robot]]\nrobot_id = \"robot3\"\nops = [\"X\"]\n"
            .parse()
            .unwrap();
        apply_cell_plan(&mut doc, "scene_fit", &serde_json::from_str(CELL_PLAN).unwrap());
        assert_eq!(doc["sink"]["cell_name"].as_str(), Some("scene_fit"));
        assert_eq!((doc["backend"]["cell"].as_str(), doc["backend"]["url"].as_str()), (Some("scene_fit"), Some("http://127.0.0.1")));
        assert_eq!(doc["protocol"]["req_byte"].as_integer(), Some(28));
        let zones: Vec<_> = doc["protocol"]["zones"].as_array().unwrap().iter().map(|z| z.as_str().unwrap()).collect();
        assert_eq!(zones, ["IL01", "IL02"], "in bit order");
        let ops: Vec<_> = doc["robot"][0]["ops"].as_array().unwrap().iter().map(|z| z.as_str().unwrap()).collect();
        assert_eq!(ops, ["W02", "W04", "HOME"]);
        assert_eq!(doc["robot"][1]["ops"].as_array().unwrap().len(), 1, "a robot the plan does not have keeps the template's");
    }

    #[test]
    fn an_export_rewrites_the_wired_bridge_and_the_next_launchs() {
        let dir = std::env::temp_dir().join(format!("factoryos-cell-wiring-export-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("bridge.toml"), "[backend]\ncell = \"other\"\n[[robot]]\nrobot_id = \"robot1\"\nendpoint = \"tcp/192.168.100.11:7447\"\n").unwrap();
        std::fs::write(dir.join("cell_plan.json"), CELL_PLAN).unwrap();
        let files = WireFiles {
            dir: dir.join("cell"),
            cell_template: dir.join("missing.toml"),
            bridge_template: dir.join("bridge.toml"),
            region: None,
        };
        let export = BridgeExport { cell: "scene_fit".into(), cell_plan: dir.join("cell_plan.json") };
        assert!(!files.apply_export(&export).unwrap(), "not wired yet: kept for the launch");
        assert!(files.write(&wiring(1), None).unwrap());
        let read = || -> toml::Table {
            let text = std::fs::read_to_string(files.dir.join(BRIDGE_TOML)).unwrap();
            assert!(text.starts_with("# Written by Physical AI Production Designer"), "{text}");
            text.parse().unwrap()
        };
        let doc = read();
        assert_eq!(doc["backend"]["cell"].as_str(), Some("scene_fit"));
        assert_eq!(doc["robot"][0]["endpoint"].as_str(), Some("tcp/10.0.5.12:7447"), "mapped as before");
        // Another export of the wired cell: rewritten, the generation touched (unchanged).
        let export = BridgeExport { cell: "cell_b".into(), ..export };
        assert!(files.apply_export(&export).unwrap());
        assert_eq!(read()["sink"]["cell_name"].as_str(), Some("cell_b"));
        assert_eq!(files.current(), Some(wiring(1).generation()));
        assert!(!files.apply_export(&export).unwrap(), "the same export: nothing rewritten");
        std::fs::remove_dir_all(&dir).ok();
    }

    /// `wiring` with what a template-resolved launch adds (docs/tasks/cell-templates.md §11): the schema and
    /// one component per host (every network's).
    fn wiring_v2(mut json: serde_json::Value, components: serde_json::Value) -> Wiring {
        json["schema"] = serde_json::json!("factoryos/wiring@2");
        json["components"] = components;
        serde_json::from_value(json).unwrap()
    }

    fn ec2_json(launch: u32) -> serde_json::Value {
        serde_json::to_value(wiring(launch)).unwrap()
    }

    /// The EC2 fixture's hosts as the Lambda's templates/wiring.js lists them.
    fn ec2_components() -> serde_json::Value {
        serde_json::json!([
            {"name": "plcHost", "semantic": "head-plc", "driver": "codesys", "address": "192.168.100.10", "instanceId": "i-plc", "ip": "172.31.5.10",
             "endpoints": {"opcua": "opc.tcp://172.31.5.11:4840", "gateway": "172.31.5.11:11740"}},
            {"name": "kukaVc1", "semantic": "robot-controller", "driver": "kss", "robotNs": "robot1", "station": "kuka-robi1", "address": "192.168.100.11",
             "instanceId": "i-k1", "ip": "172.31.5.31", "endpoints": {"zenoh": "tcp/172.31.5.12:7447", "smartpad": "https://172.31.5.12:443/"}},
            {"name": "kukaVc2", "semantic": "robot-controller", "driver": "kss", "robotNs": "robot2", "station": "kuka-robi2", "address": "192.168.100.12",
             "instanceId": "i-k2", "ip": "172.31.5.32", "endpoints": {"zenoh": "tcp/172.31.5.13:7447", "smartpad": "https://172.31.5.13:443/"}},
            {"name": "compile", "semantic": "compile", "instanceId": "i-ide", "ip": "172.31.5.20", "endpoints": {"mcp": "http://172.31.5.20:8766/mcp"}},
            {"name": "windowsIde", "semantic": "ide", "instanceId": "i-ide", "ip": "172.31.5.20", "endpoints": {"dcv": "172.31.5.20:8443"}}
        ])
    }

    /// The legacy templates' capabilities (factoryos_protocol::infra::legacy_capabilities).
    fn caps(runtime: &str, ide: Option<&str>) -> Vec<String> {
        factoryos_protocol::infra::legacy_capabilities(runtime, ide).into_iter().map(str::to_owned).collect()
    }

    /// A template-launched legacy cell renders cell.toml, cell-<net>.toml, the bridge, dcv.env, the kubeconfig and
    /// the generation line byte for byte as today's wiring of the same addresses does.
    #[test]
    fn a_legacy_templates_wiring_writes_todays_files() {
        let dir = std::env::temp_dir().join(format!("factoryos-cell-wiring-v2-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        std::fs::create_dir_all(&dir).unwrap();
        let cell_template = "[codesys_mcp]\nurl = \"http://old:8766/mcp\"\ninstall = \"SP22 Patch 1\"\n[plc_host]\nssm_instance = \"i-old\"\n\
                             [[kuka_vc.controller]]\nrobot_ns = \"robot1\"\ninstance = \"iiQKA.OS2_KSS_9.2.0_vass\"\nssm_instance = \"i-old\"\n";
        let bridge_template = "[opcua]\nendpoint = \"opc.tcp://192.168.100.10:4840\"\n[[robot]]\nrobot_id = \"robot1\"\nmodel = \"KR240\"\n\
                               endpoint = \"tcp/192.168.100.11:7447\"\nprefix = \"kuka/vass/r1\"\n[[robot]]\nrobot_id = \"robot2\"\nendpoint = \"tcp/192.168.100.12:7447\"\n";
        std::fs::write(dir.join("cell.toml"), cell_template).unwrap();
        std::fs::write(dir.join("bridge.toml"), bridge_template).unwrap();
        let files = |name: &str| WireFiles {
            dir: dir.join(name),
            cell_template: dir.join("cell.toml"),
            bridge_template: dir.join("bridge.toml"),
            region: Some("eu-central-1".into()),
        };
        let read_all = |files: &WireFiles| -> BTreeMap<String, String> {
            let mut out = BTreeMap::new();
            for entry in std::fs::read_dir(&files.dir).unwrap() {
                let path = entry.unwrap().path();
                out.insert(path.file_name().unwrap().to_string_lossy().into_owned(), std::fs::read_to_string(&path).unwrap());
            }
            out
        };

        // EC2 (ec2-codesys-kuka): the components change nothing of what is written.
        let today = wiring(3);
        let v2 = wiring_v2(ec2_json(3), ec2_components());
        assert_eq!((v2.schema.as_deref(), v2.components.len()), (Some("factoryos/wiring@2"), 5));
        assert_eq!(v2.generation(), today.generation(), "the generation ignores schema and components");
        let ec2 = caps("ec2", None);
        assert!(v2.complete_for(Some(&ec2)) && today.complete_for(Some(&ec2)) && today.complete_for(None));
        assert_eq!(render_cell_toml(&v2, Some(cell_template), Some("eu-central-1")).unwrap(), render_cell_toml(&today, Some(cell_template), Some("eu-central-1")).unwrap());
        assert_eq!(render_bridge_toml(&v2, bridge_template).unwrap(), render_bridge_toml(&today, bridge_template).unwrap());
        let (old, new) = (files("ec2-today"), files("ec2-v2"));
        assert!(old.write(&today, None).unwrap() && new.write(&v2, Some(&ec2)).unwrap());
        let written = read_all(&old);
        assert_eq!(written.keys().collect::<Vec<_>>(), ["cell.toml", "dcv.env", "fos-live-bridge.toml", "generation"]);
        assert_eq!(read_all(&new), written);

        // EKS (eks-codesys-kuka, container IDE, two networks over kubectl): the same.
        let net = |name: &str, k: u32| serde_json::json!({
            "name": name, "vni": 1100 + k, "subnet": format!("10.77.{k}.0/24"),
            "plcHost": {"instanceId": format!("pod/cell-jdoe/{name}-plc"), "ip": format!("172.31.60.1{k}"), "kubectlPod": format!("cell-jdoe/{name}-plc/agent")},
            "windowsIde": {"instanceId": "pod/cell-jdoe/compile", "ip": "172.31.60.40", "mcpUrl": "http://172.31.60.40:8766/mcp", "dcvServer": null},
            "kukaVc": [{"role": "kukaVc1", "instanceId": format!("pod/cell-jdoe/{name}-kvc-1"), "ip": format!("172.31.60.2{k}"),
                        "kubectlPod": format!("cell-jdoe/{name}-kvc-1/kvc"), "station": "kuka-robi1", "kliIp": format!("10.77.{k}.11")}],
            "profinet": {"cidr": format!("10.77.{k}.0/24"), "gateway": format!("10.77.{k}.1"), "head": format!("10.77.{k}.10"), "vni": 1100 + k},
            "addressMap": {format!("10.77.{k}.10"): format!("172.31.60.1{k}"), format!("10.77.{k}.11"): format!("172.31.60.2{k}")}
        });
        let mut eks = net("n1", 1);
        eks["launch"] = serde_json::json!(5);
        eks["runtime"] = serde_json::json!("eks");
        eks["ide"] = serde_json::json!("container");
        eks["namespace"] = serde_json::json!("cell-jdoe");
        eks["mcpTokenSecret"] = serde_json::json!("factoryos/cell/ws-jdoe/5");
        eks["networks"] = serde_json::json!([net("n1", 1), net("n2", 2)]);
        eks["kubernetes"] = serde_json::json!({"cluster": "factoryos-cells", "endpoint": "https://ABC123.gr7.eu-central-1.eks.amazonaws.com", "ca": "Q0E="});
        let component = |n: u32| {
            serde_json::json!([
                {"name": format!("n{n}/plcHost"), "semantic": "head-plc", "driver": "codesys", "network": format!("n{n}"), "address": format!("10.77.{n}.10"),
                 "kubectlPod": format!("cell-jdoe/n{n}-plc/agent"), "ip": format!("172.31.60.1{n}"),
                 "endpoints": {"opcua": format!("opc.tcp://172.31.60.1{n}:4840"), "gateway": format!("172.31.60.1{n}:11740")}},
                {"name": format!("n{n}/kukaVc1"), "semantic": "robot-controller", "driver": "kss", "network": format!("n{n}"), "robotNs": "robot1",
                 "station": "kuka-robi1", "address": format!("10.77.{n}.11"), "kubectlPod": format!("cell-jdoe/n{n}-kvc-1/kvc"), "ip": format!("172.31.60.2{n}"),
                 "endpoints": {"zenoh": format!("tcp/172.31.60.2{n}:7447"), "smartpad": format!("https://172.31.60.2{n}:443/")}}
            ])
        };
        let mut components = component(1).as_array().unwrap().clone();
        components.extend(component(2).as_array().unwrap().clone());
        components.push(serde_json::json!({"name": "compile", "semantic": "compile", "kubectlPod": "cell-jdoe/compile/ide", "ip": "172.31.60.40",
            "endpoints": {"mcp": "http://172.31.60.40:8766/mcp"}}));
        let today: Wiring = serde_json::from_value(eks.clone()).unwrap();
        let v2 = wiring_v2(eks, serde_json::Value::Array(components));
        let container = caps("eks", Some("container"));
        assert!(v2.complete_for(Some(&container)) && today.complete());
        assert_eq!(v2.generation(), today.generation());
        let (old, new) = (files("eks-today"), files("eks-v2"));
        assert!(old.write(&today, None).unwrap() && new.write(&v2, Some(&container)).unwrap());
        let written = read_all(&old);
        assert_eq!(written.keys().collect::<Vec<_>>(), ["cell-n2.toml", "cell.toml", "dcv.env", "fos-live-bridge.toml", "generation", "kubeconfig"]);
        assert_eq!(read_all(&new), written);
        std::fs::remove_dir_all(&dir).ok();
    }

    /// complete() with components: the head's address and exec target, the compile host only with the CODESYS MCP,
    /// the IDE only with DCV; without capabilities (no template), today's rule.
    #[test]
    fn a_templates_wiring_is_complete_by_its_capabilities() {
        let ec2 = caps("ec2", None);
        let without = |name: &str| {
            let list: Vec<serde_json::Value> = ec2_components().as_array().unwrap().iter().filter(|c| c["name"] != name).cloned().collect();
            wiring_v2(ec2_json(1), serde_json::Value::Array(list))
        };
        assert!(!without("compile").complete_for(Some(&ec2)), "deploy.head.codesys-mcp needs the compile host");
        assert!(!without("windowsIde").complete_for(Some(&ec2)), "ide.dcv needs the IDE");
        assert!(!without("plcHost").complete_for(Some(&ec2)), "the head is required");
        assert!(without("windowsIde").complete_for(None), "no template: today's rule (the projection has the IDE)");
        let no_dcv: Vec<String> = ec2.iter().filter(|c| *c != "ide.dcv").cloned().collect();
        assert!(without("windowsIde").complete_for(Some(&no_dcv)));
        let no_mcp: Vec<String> = no_dcv.iter().filter(|c| *c != "deploy.head.codesys-mcp").cloned().collect();
        assert!(without("compile").complete_for(Some(&no_mcp)));
        // A head without an address or an exec target is not usable.
        let mut list = ec2_components();
        list[0]["address"] = serde_json::Value::Null;
        assert!(!wiring_v2(ec2_json(1), list).complete_for(Some(&ec2)));
        let mut list = ec2_components();
        list[0].as_object_mut().unwrap().remove("instanceId");
        assert!(!wiring_v2(ec2_json(1), list).complete_for(Some(&ec2)));
        let mut list = ec2_components();
        list[0].as_object_mut().unwrap().remove("instanceId");
        list[0]["kubectlPod"] = serde_json::json!("cell-jdoe/n1-plc/agent");
        assert!(wiring_v2(ec2_json(1), list).complete_for(Some(&ec2)));
        // Without components (an older Lambda, templates off) the capabilities change nothing.
        assert!(wiring(1).complete_for(Some(&no_mcp)) && !Wiring::default().complete_for(Some(&ec2)));
        // An older backend ignores the new keys; this one reads keys it does not know.
        let mut json = ec2_json(1);
        json["components"] = serde_json::json!([{"name": "x", "semantic": "aux", "future": 1, "endpoints": {"hook": "exec"}}]);
        assert_eq!(serde_json::from_value::<Wiring>(json).unwrap().components[0].semantic, "aux");
    }

    /// The smoke template's capabilities (docs/tasks/cell-templates.md E12).
    const SMOKE_CAPS: [&str; 6] = ["check.hook", "deploy.head.hook", "deploy.robot.hook", "live.head.exec", "live.robot.exec", "runtime.hook"];

    fn strings(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| (*s).to_owned()).collect()
    }

    /// An engine-2 EKS launch of `networks` (`(name, robots)`) as the Lambda's templates/wiring.js lists it (E8):
    /// per network a hook head pod and one hook robot pod per robot, no projection, no address map (nothing
    /// is reached by DNAT), the cell's kubernetes.
    fn hook_wiring(launch: u32, networks: &[(&str, u32)]) -> Wiring {
        let net = |name: &str, k: u32| serde_json::json!({
            "name": name, "vni": 1100 + k, "subnet": format!("10.77.{k}.0/24"),
            "profinet": {"cidr": format!("10.77.{k}.0/24"), "gateway": format!("10.77.{k}.1"), "head": format!("10.77.{k}.10"), "vni": 1100 + k},
            "addressMap": {}
        });
        let mut components = Vec::new();
        for (k, (name, robots)) in networks.iter().enumerate() {
            let k = k as u32 + 1;
            components.push(serde_json::json!({"name": format!("{name}/head"), "semantic": "head-plc", "driver": "hook", "network": name,
                "address": format!("10.77.{k}.10"), "kubectlPod": format!("cell-jdoe/{name}-head/head"), "hookContainer": "head", "ip": format!("172.31.60.1{k}")}));
            for n in 1..=*robots {
                components.push(serde_json::json!({"name": format!("{name}/robot-{n}"), "semantic": "robot-controller", "driver": "hook", "network": name,
                    "robotNs": format!("robot{n}"), "station": format!("kuka-robi{n}"), "address": format!("10.77.{k}.{}", 10 + n),
                    "kubectlPod": format!("cell-jdoe/{name}-robot-{n}/robot"), "hookContainer": "robot", "ip": format!("172.31.60.{}{n}", 2 + k)}));
            }
        }
        let mut json = net(networks[0].0, 1);
        json["launch"] = serde_json::json!(launch);
        json["runtime"] = serde_json::json!("eks");
        json["namespace"] = serde_json::json!("cell-jdoe");
        json["networks"] = serde_json::Value::Array(networks.iter().enumerate().map(|(k, (n, _))| net(n, k as u32 + 1)).collect());
        json["kubernetes"] = serde_json::json!({"cluster": "factoryos-cells", "endpoint": "https://ABC123.gr7.eu-central-1.eks.amazonaws.com", "ca": "Q0E="});
        wiring_v2(json, serde_json::Value::Array(components))
    }

    /// E8: a hook head and two hook robots render exactly the contract's cell.toml (no `[codesys_mcp]`, no SSM or
    /// SSH, no instance / netns / sps_base), also on top of the shared cell's template.
    #[test]
    fn a_hook_cells_cell_toml_has_the_hook_drivers() {
        let w = hook_wiring(3, &[("n1", 2)]);
        let caps = strings(&SMOKE_CAPS);
        assert!(w.has_hook() && w.complete_for(Some(&caps)), "no plcHost projection needed for a hook head");
        assert!(!w.complete(), "today's rule would wait for a plcHost forever");
        // The contract's example, in the order the writer gives (tables and keys sorted, as today's files).
        let expected = "# Written by Physical AI Production Designer (services/cell_wiring.rs) for launch 3 of this workspace's virtual cell.\n\
            # Rewritten whenever a launch's addresses change; do not edit.\n\n\
            [address_map]\n\n\
            [kuka_vc]\nkubeconfig = \"/var/lib/factoryos/cell/kubeconfig\"\nregion = \"eu-central-1\"\n\n\
            [[kuka_vc.controller]]\ndriver = \"hook\"\nkli_ip = \"10.77.1.11\"\nkubectl_pod = \"cell-jdoe/n1-robot-1/robot\"\nrobot_ns = \"robot1\"\nstation = \"kuka-robi1\"\n\n\
            [[kuka_vc.controller]]\ndriver = \"hook\"\nkli_ip = \"10.77.1.12\"\nkubectl_pod = \"cell-jdoe/n1-robot-2/robot\"\nrobot_ns = \"robot2\"\nstation = \"kuka-robi2\"\n\n\
            [plc_host]\ndriver = \"hook\"\nkubeconfig = \"/var/lib/factoryos/cell/kubeconfig\"\nkubectl_pod = \"cell-jdoe/n1-head/head\"\nregion = \"eu-central-1\"\n\n\
            [profinet]\ngateway = \"10.77.1.1\"\nhead_ip = \"10.77.1.10\"\nnetmask = \"255.255.255.0\"\n";
        assert_eq!(render_cell_toml_for(&w, None, Some("eu-central-1"), Some(&caps)).unwrap(), expected);
        // The shared cell's template: its CODESYS and KSS keys are not the hooks'.
        let template = "[codesys_mcp]\nurl = \"http://old:8766/mcp\"\ninstall = \"SP22 Patch 1\"\n[plc_host]\nssh_host = \"plc-host\"\nssm_instance = \"i-old\"\nsetup = \"/x\"\n\
                        [kuka_vc]\nregion = \"eu-central-1\"\n[[kuka_vc.controller]]\nrobot_ns = \"robot1\"\ninstance = \"X\"\nnetns = \"krc9\"\nssm_instance = \"i-old2\"\nsps_base = \"S\"\n";
        let doc: toml::Table = render_cell_toml_for(&w, Some(template), Some("eu-central-1"), Some(&caps)).unwrap().parse().unwrap();
        assert!(doc.get("codesys_mcp").is_none(), "no deploy.head.codesys-mcp: no [codesys_mcp]");
        assert_eq!(doc["plc_host"].as_table().unwrap().keys().collect::<Vec<_>>(), ["driver", "kubeconfig", "kubectl_pod", "region"]);
        let c = doc["kuka_vc"]["controller"].as_array().unwrap();
        assert_eq!(c[0].as_table().unwrap().keys().collect::<Vec<_>>(), ["driver", "kli_ip", "kubectl_pod", "robot_ns", "station"]);
        assert_eq!(doc["kuka_vc"]["region"].as_str(), Some("eu-central-1"));
        // Without the template's capabilities (no template): today's renderer, which has nothing to say about pods.
        assert_eq!(render_cell_toml_for(&w, None, None, None).unwrap(), render_cell_toml(&w, None, None).unwrap());
    }

    /// E7.2, E8: a builtin CODESYS head (`builtin:eks-plc@1`, deployed through the CODESYS MCP) with hook robots:
    /// `[codesys_mcp]` and `[plc_host]` exactly as today's renderer writes them, the robots through their hooks.
    #[test]
    fn a_codesys_head_with_hook_robots() {
        let mut json = serde_json::to_value(hook_wiring(4, &[("n1", 2)])).unwrap();
        json["plcHost"] = serde_json::json!({"instanceId": "pod/cell-jdoe/n1-plc", "ip": "172.31.60.11", "kubectlPod": "cell-jdoe/n1-plc/agent"});
        json["windowsIde"] = serde_json::json!({"instanceId": "pod/cell-jdoe/compile", "ip": "172.31.60.40", "mcpUrl": "http://172.31.60.40:8766/mcp"});
        json["addressMap"] = serde_json::json!({"10.77.1.10": "172.31.60.11"});
        json["components"][0] = serde_json::json!({"name": "n1/plcHost", "semantic": "head-plc", "driver": "codesys", "network": "n1", "address": "10.77.1.10",
            "kubectlPod": "cell-jdoe/n1-plc/agent", "ip": "172.31.60.11",
            "endpoints": {"opcua": "opc.tcp://172.31.60.11:4840", "gateway": "172.31.60.11:11740"}});
        json["components"].as_array_mut().unwrap().push(serde_json::json!({"name": "compile", "semantic": "compile", "kubectlPod": "cell-jdoe/compile/ide",
            "ip": "172.31.60.40", "endpoints": {"mcp": "http://172.31.60.40:8766/mcp"}}));
        let w: Wiring = serde_json::from_value(json).unwrap();
        let caps = strings(&["check.hook", "deploy.head.codesys-mcp", "deploy.robot.hook", "live.head.opcua", "live.robot.exec", "runtime.codesys"]);
        assert!(w.complete_for(Some(&caps)));
        let text = render_cell_toml_for(&w, None, Some("eu-central-1"), Some(&caps)).unwrap();
        let doc: toml::Table = text.parse().unwrap();
        let today: toml::Table = render_cell_toml(&w, None, Some("eu-central-1")).unwrap().parse().unwrap();
        assert_eq!(doc["codesys_mcp"], today["codesys_mcp"]);
        assert_eq!(doc["plc_host"], today["plc_host"]);
        assert_eq!(doc["plc_host"]["kubectl_pod"].as_str(), Some("cell-jdoe/n1-plc/agent"));
        assert_eq!((doc["address_map"].clone(), doc["profinet"].clone()), (today["address_map"].clone(), today["profinet"].clone()));
        let c = doc["kuka_vc"]["controller"].as_array().unwrap();
        assert_eq!(c.len(), 2);
        assert_eq!((c[1]["driver"].as_str(), c[1]["kubectl_pod"].as_str(), c[1].get("instance")), (Some("hook"), Some("cell-jdoe/n1-robot-2/robot"), None));
        // The bridge: the builtin head's OPC UA, the robots' watch.
        let t = "[opcua]\nendpoint = \"opc.tcp://192.168.100.10:4840\"\npoll_hz = 5.0\n[[robot]]\nrobot_id = \"robot1\"\nmodel = \"KR240\"\nendpoint = \"tcp/192.168.100.11:7447\"\nprefix = \"kuka/vass/r1\"\n";
        let doc: toml::Table = render_bridge_toml_for(&w, t, Some(&caps)).unwrap().parse().unwrap();
        assert_eq!((doc["opcua"]["endpoint"].as_str(), doc["opcua"]["poll_hz"].as_float()), (Some("opc.tcp://172.31.60.11:4840"), Some(5.0)));
        assert!(doc["exec"].get("head").is_none());
        assert_eq!(doc["robot"][1]["source"].as_str(), Some("exec"));
        // A head live source and no robot one (E9): the head's OPC UA, no [[robot]] (fos_live_bridge.py takes that).
        let head_only = strings(&["deploy.head.codesys-mcp", "deploy.robot.hook", "live.head.opcua"]);
        let doc: toml::Table = render_bridge_toml_for(&w, t, Some(&head_only)).unwrap().parse().unwrap();
        assert_eq!(doc["opcua"]["endpoint"].as_str(), Some("opc.tcp://172.31.60.11:4840"));
        assert!(doc.get("robot").is_none() && doc.get("exec").is_none(), "{doc:?}");
        // A mix of hook and KSS robots is no composition of E1.4.
        let mut mixed = w.clone();
        mixed.components[1].driver = Some("kss".into());
        assert!(render_cell_toml_for(&mixed, None, None, Some(&caps)).unwrap_err().to_string().contains("mixes"));
    }

    /// The viewer's bridge template the tests render (`tests/fixtures/fos-live-bridge.toml`): plan2cell/infra/viewer's
    /// fos-live-bridge.toml but for its comments, so the tests need no infra files (the public export carries none).
    const BRIDGE_FIXTURE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/fos-live-bridge.toml");

    /// Where the infra file is (this repo), the fixture must be it but for comments: a change there fails here until
    /// the copy has it too.
    #[test]
    fn the_bridge_fixture_is_the_viewers_template() {
        let infra = concat!(env!("CARGO_MANIFEST_DIR"), "/../../plan2cell/infra/viewer/fos-live-bridge.toml");
        let Ok(template) = std::fs::read_to_string(infra) else {
            eprintln!("skipped: no {infra} (the fixture stands in for it)");
            return;
        };
        let fixture: toml::Table = std::fs::read_to_string(BRIDGE_FIXTURE).unwrap().parse().unwrap();
        assert_eq!(fixture, template.parse::<toml::Table>().unwrap(), "the fixture is the viewer's template but for comments");
    }

    /// E10: the bridge reads the head's and every robot's `watch` over kubectl exec with the kubeconfig; OPC UA is
    /// off and no Zenoh endpoint is left.
    #[test]
    fn a_hook_cells_bridge_reads_the_hooks_watch() {
        let w = hook_wiring(3, &[("n1", 2)]);
        let caps = strings(&SMOKE_CAPS);
        let t = std::fs::read_to_string(BRIDGE_FIXTURE).unwrap();
        let text = render_bridge_toml_for(&w, &t, Some(&caps)).unwrap();
        assert!(text.starts_with("# Written by Physical AI Production Designer for launch 3"), "{text}");
        let doc: toml::Table = text.parse().unwrap();
        assert_eq!(doc["opcua"].as_table().unwrap().keys().collect::<Vec<_>>(), ["enabled"]);
        assert_eq!(doc["opcua"]["enabled"].as_bool(), Some(false));
        let exec = doc["exec"].as_table().unwrap();
        assert_eq!(exec["kubeconfig"].as_str(), Some(KUBECONFIG_PATH));
        assert_eq!((exec["period_ms"].as_integer(), exec["max_s"].as_integer()), (Some(200), Some(600)));
        let argv = toml::Value::Array(vec!["/opt/factoryos/hook".into(), "watch".into()]);
        assert_eq!((exec["head"]["pod"].as_str(), exec["head"]["container"].as_str()), (Some("cell-jdoe/n1-head"), Some("head")));
        assert_eq!(exec["head"]["argv"], argv);
        let robots = doc["robot"].as_array().unwrap();
        let got: Vec<(&str, &str, &str, &str, &str)> = robots
            .iter()
            .map(|r| (r["robot_id"].as_str().unwrap(), r["name"].as_str().unwrap(), r["source"].as_str().unwrap(), r["pod"].as_str().unwrap(), r["container"].as_str().unwrap()))
            .collect();
        assert_eq!(got, [("robot1", "R1", "exec", "cell-jdoe/n1-robot-1", "robot"), ("robot2", "R2", "exec", "cell-jdoe/n1-robot-2", "robot")]);
        for r in robots {
            assert!(r.get("endpoint").is_none() && r.get("prefix").is_none(), "{r:?}");
            assert_eq!(r["argv"], argv);
            assert_eq!(r["model"].as_str(), Some("KR240 R2900 ultra"), "the template's robot gives the rest");
        }
        // The sink, backend and protocol stay the template's.
        assert_eq!(doc["sink"]["endpoint"].as_str(), Some("tcp/127.0.0.1:7447"));
        // Without live capabilities: no head source (OPC UA off), no robots, no [exec].
        let doc: toml::Table = render_bridge_toml_for(&w, &t, Some(&strings(&["deploy.head.hook", "deploy.robot.hook"]))).unwrap().parse().unwrap();
        assert_eq!(doc["opcua"]["enabled"].as_bool(), Some(false));
        assert!(doc.get("robot").is_none() && doc.get("exec").is_none());
    }

    /// A hook cell's files: cell.toml and cell-<net>.toml from each network's components, the bridge with [exec],
    /// the kubeconfig, no dcv.env (no IDE); a new pod is a new generation, the same wiring writes nothing.
    #[test]
    fn a_hook_cells_files_are_written_per_network() {
        let dir = std::env::temp_dir().join(format!("factoryos-cell-wiring-hook-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("bridge.toml"), "[opcua]\nendpoint = \"opc.tcp://192.168.100.10:4840\"\n[[robot]]\nrobot_id = \"robot1\"\nendpoint = \"tcp/192.168.100.11:7447\"\nprefix = \"kuka/vass/r1\"\n").unwrap();
        let files = WireFiles { dir: dir.join("cell"), cell_template: dir.join("missing.toml"), bridge_template: dir.join("bridge.toml"), region: Some("eu-central-1".into()) };
        let caps = strings(&SMOKE_CAPS);
        std::fs::create_dir_all(&files.dir).unwrap();
        std::fs::write(files.dir.join(DCV_ENV), "FACTORY_OS_DCV_SERVER=172.31.5.20\n").unwrap();
        let w = hook_wiring(5, &[("n1", 1), ("n2", 2)]);
        assert!(files.write(&w, Some(&caps)).unwrap());
        let mut names: Vec<String> = std::fs::read_dir(&files.dir).unwrap().map(|e| e.unwrap().file_name().to_string_lossy().into_owned()).collect();
        names.sort();
        assert_eq!(names, ["cell-n2.toml", "cell.toml", "fos-live-bridge.toml", "generation", "kubeconfig"]);
        let read = |name: &str| -> toml::Table { std::fs::read_to_string(files.dir.join(name)).unwrap().parse().unwrap() };
        let n1 = read("cell.toml");
        assert_eq!(n1["plc_host"]["kubectl_pod"].as_str(), Some("cell-jdoe/n1-head/head"));
        assert_eq!(n1["kuka_vc"]["controller"].as_array().unwrap().len(), 1);
        let n2 = read("cell-n2.toml");
        assert_eq!(n2["plc_host"]["kubectl_pod"].as_str(), Some("cell-jdoe/n2-head/head"));
        let c = n2["kuka_vc"]["controller"].as_array().unwrap();
        assert_eq!(c.iter().map(|c| c["kubectl_pod"].as_str().unwrap()).collect::<Vec<_>>(), ["cell-jdoe/n2-robot-1/robot", "cell-jdoe/n2-robot-2/robot"]);
        assert_eq!((c[1]["kli_ip"].as_str(), n2["profinet"]["head_ip"].as_str()), (Some("10.77.2.12"), Some("10.77.2.10")));
        let bridge = read(BRIDGE_TOML);
        assert_eq!(bridge["exec"]["head"]["pod"].as_str(), Some("cell-jdoe/n1-head"), "the first network's, as today");
        assert_eq!(bridge["robot"].as_array().unwrap().len(), 1);
        assert!(!files.write(&w, Some(&caps)).unwrap(), "the same wiring: nothing rewritten");
        // The same addresses, another pod (no projection tells them apart): a new generation.
        let mut moved = w.clone();
        moved.components[0].kubectl_pod = Some("cell-jdoe/n1-head2/head".into());
        assert_ne!(moved.generation(), w.generation());
        // A hook head without its pod is not complete; a robot without one is not rendered.
        let mut no_pod = w.clone();
        no_pod.components[0].kubectl_pod = None;
        assert!(!no_pod.complete_for(Some(&caps)));
        let mut no_robot_pod = w.clone();
        no_robot_pod.components[1].kubectl_pod = Some("cell-jdoe/../x".into());
        assert!(render_cell_toml_for(&no_robot_pod, None, None, Some(&caps)).is_err());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn files_are_written_once_per_wiring() {
        let dir = std::env::temp_dir().join(format!("factoryos-cell-wiring-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("bridge.toml"), "[opcua]\nendpoint = \"opc.tcp://192.168.100.10:4840\"\n").unwrap();
        let files = WireFiles {
            dir: dir.join("cell"),
            cell_template: dir.join("missing.toml"),
            bridge_template: dir.join("bridge.toml"),
            region: Some("eu-central-1".into()),
        };
        let w = wiring(1);
        assert!(files.write(&w, None).unwrap());
        assert!(!files.write(&w, None).unwrap(), "the same wiring: nothing rewritten");
        assert_eq!(files.current(), Some(w.generation()));
        assert!(std::fs::read_to_string(files.dir.join(BRIDGE_TOML)).unwrap().contains("10.0.5.11:4840"));
        let mut moved = wiring(2);
        moved.windows_ide.ip = "10.0.6.20".into();
        assert_ne!(moved.fingerprint(), w.fingerprint());
        assert!(files.write(&moved, None).unwrap());
        assert!(std::fs::read_to_string(files.cell_toml()).unwrap().contains("launch 2"));
        assert!(files.write(&Wiring::default(), None).is_err());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn the_bridge_template_falls_back_to_the_shared_cells_only_as_the_default() {
        let d = std::env::temp_dir().join(format!("factoryos-cell-wiring-bridge-template-{}", std::process::id()));
        std::fs::create_dir_all(&d).unwrap();
        let (own, shared, other) = (d.join("own.toml"), d.join("shared.toml"), d.join("other.toml"));
        std::fs::write(&shared, "shared").unwrap();
        // The default missing: the shared cell's template.
        assert_eq!(read_bridge_template(&own, &own, &shared).unwrap(), "shared");
        // The default there: it.
        std::fs::write(&own, "own").unwrap();
        assert_eq!(read_bridge_template(&own, &own, &shared).unwrap(), "own");
        // A configured template that is not the default: no fallback.
        assert!(read_bridge_template(&other, &own, &shared).is_err());
        std::fs::remove_dir_all(&d).ok();
    }
}
