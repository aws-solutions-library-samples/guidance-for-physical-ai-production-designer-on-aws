//! `cell.toml`: where the cell's machines are. The workspace setup writes it to
//! `~/.config/plan2cell/cell.toml`.
//!
//! ```toml
//! [codesys_mcp]
//! url = "http://10.0.0.5:8766/mcp"
//! token_env = "PLAN2CELL_CODESYS_MCP_TOKEN"   # bearer token from this variable (the IDE requires it)
//! # optional:
//! windows_project_dir = 'C:\ProgramData\plan2cell\projects\{cell}'   # {cell}: the cell id
//! install = "SP22 Patch 1"                     # the CODESYS install every tool call names
//! device_user_env = "PLAN2CELL_DEVICE_USER"    # runtime login: the variables holding user and
//! device_password_env = "PLAN2CELL_DEVICE_PASSWORD"  # password (unset: the server's own)
//! io_csv = "mirror"                            # how the IO mapping CSV reaches the IDE
//! timeout_s = 1800                             # longest a single tool call may run
//! connect_wait_s = 300                         # how long a stage waits for the MCP bridge to answer
//! other_codesys = "refuse"                     # a CODESYS the server did not start in its way:
//!                                              # "refuse" (exit 3) or "close" (an unattended cell)
//!
//! [plc_host]
//! ssh_host = "plc-host"
//! setup = "/opt/plan2cell/plc-host/setup.sh"
//! compose_dir = "/opt/plan2cell/plc-host"
//! # optional (default: $PLAN2CELL_BUCKET, $AWS_REGION / $AWS_DEFAULT_REGION):
//! bucket = "plan2cell-software-<account>-<region>"
//! region = "eu-central-1"
//!
//! # optional: KUKA virtual controllers that run robots' KRL (`plan2cell cell krl`), one table
//! # per controller; several can share a host (one netns each)
//! [kuka_vc]
//! region = "eu-central-1"                      # SSM access (default: $AWS_REGION)
//! profile = "dev"                              # SSM access (default: $AWS_PROFILE)
//! dir = "/var/lib/plan2cell/kuka-vc"           # helper and KRL bundles on the host
//!
//! [[kuka_vc.controller]]
//! robot_ns = "robot1"                          # the build's krl/<robot_ns>
//! instance = "iiQKA.OS2_KSS_9.2.0_vass"        # must be the active instance of the netns
//! netns = "krc1"
//! kli_ip = "10.200.0.11"
//! station = "kuka-robi1"
//! ssm_instance = "i-0123456789abcdef0"         # or: ssh_host = "kuka-vc" (~/.ssh/config)
//! sps_base = "iiQKA.OS2_KSS_9.2.0"             # optional: reset sps.sub to this instance's first
//!
//! [[kuka_vc.controller]]                       # a concurrent controller on the same host
//! robot_ns = "robot2"                          # (infra/kuka-vc/kvc-add-controller.sh)
//! instance = "iiQKA.OS2_KSS_9.2.0_vass_r2"
//! netns = "krc2"
//! kli_ip = "10.200.0.12"
//! station = "kuka-robi2"
//! ssm_instance = "i-0123456789abcdef0"
//! ```
//!
//! A workspace's own virtual cell adds (Factory OS writes it per launch):
//!
//! ```toml
//! [address_map]                                # inside the cell -> reachable from outside
//! "192.168.100.10" = "172.31.5.11"
//! ```
//!
//! and, optionally, the cell's PROFINET segment as the deployability check expects it (default
//! with `[kuka_vc]`: the plan2cell virtual cell's, head `192.168.100.10/24`, gateway `.1`):
//!
//! ```toml
//! [profinet]
//! head_ip = "192.168.100.10"
//! netmask = "255.255.255.0"
//! gateway = "192.168.100.1"                    # "0.0.0.0": none
//! ```
//!
//! A cell of the generic path (docs/tasks/cell-templates.md E8) drives a head and/or robots
//! through `factoryos/hook@1` ([`crate::hook`]) instead of CODESYS and p2c-kvc:
//!
//! ```toml
//! [plc_host]
//! driver = "hook"                              # default "codesys"
//! kubectl_pod = "cell-jdoe/n1-head/head"       # <namespace>/<pod>/<container with the hook>
//! kubeconfig = "/var/lib/factoryos/cell/kubeconfig"
//!
//! [[kuka_vc.controller]]
//! driver = "hook"                              # default "kss"
//! robot_ns = "robot1"
//! station = "kuka-robi1"
//! kli_ip = "10.77.1.11"
//! kubectl_pod = "cell-jdoe/n1-robot-1/robot"   # instance, netns, sps_base: not used
//! ```
//!
//! `[codesys_mcp]` may be absent when no stage needs it (a hook head: no projects, no MCP).
//!
//! Unknown keys are ignored (an older file with `script_tool`, `project_dir` or `gateway` still
//! loads).

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::{Error, Result};

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CellConfig {
    #[serde(default)]
    pub codesys_mcp: CodesysMcp,
    #[serde(default)]
    pub plc_host: PlcHost,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub kuka_vc: Option<KukaVc>,
    /// `[address_map]`: an address of the cell's PROFINET network (the same in every cell,
    /// `192.168.100.x`) to the address that reaches it from outside the cell (a workspace's own
    /// virtual cell: the PLC host's secondary IP NATed 1:1 to it). The CODESYS gateway's `bind`
    /// uses the mapped address; everything inside the cell keeps the original.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub address_map: std::collections::BTreeMap<String, String>,
    /// `[profinet]`: the segment the head and the controllers are on (`plan2cell cell check`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub profinet: Option<Profinet>,
}

/// `[profinet]`: unset values are the plan2cell virtual cell's.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Profinet {
    pub head_ip: Option<String>,
    pub netmask: Option<String>,
    pub gateway: Option<String>,
}

/// `[kuka_vc]`: KUKA virtual controllers (iiQKA.OS2 / KSS 9.2) that run robots' KRL.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct KukaVc {
    /// Where plan2cell keeps its helper and the KRL bundles on the host.
    pub dir: String,
    /// SSM: the AWS region and profile (unset: the environment's).
    pub region: Option<String>,
    pub profile: Option<String>,
    /// Longest one remote command may run.
    pub timeout_s: u64,
    /// kubectl: the kubeconfig of the controllers' `kubectl_pod` (unset: `$KUBECONFIG`, else kubectl's default).
    pub kubeconfig: Option<String>,
    #[serde(rename = "controller")]
    pub controllers: Vec<KukaController>,
}

impl Default for KukaVc {
    fn default() -> Self {
        Self {
            dir: "/var/lib/plan2cell/kuka-vc".into(),
            region: None,
            profile: None,
            timeout_s: 600,
            kubeconfig: None,
            controllers: Vec::new(),
        }
    }
}

/// How plan2cell drives a robot controller (`[[kuka_vc.controller]] driver`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum RobotDriver {
    /// A KUKA KSS virtual controller: p2c-kvc.py, kvc-ack.sh, kvc-mode.sh on its host.
    #[default]
    Kss,
    /// A pod with `/opt/factoryos/hook` (`factoryos/hook@1`, [`crate::hook`]), over `kubectl_pod`.
    Hook,
}

/// How plan2cell drives the head PLC (`[plc_host] driver`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum HeadDriver {
    /// CODESYS: the PLC host's runtimes, the projects over the CODESYS MCP server.
    #[default]
    Codesys,
    /// A pod with `/opt/factoryos/hook` (`factoryos/hook@1`), over `plc_host.kubectl_pod`.
    Hook,
}

fn is_default<T: Default + PartialEq>(t: &T) -> bool {
    *t == T::default()
}

/// One controller: an iiQKA.OS2 instance running in a network namespace of a KUKA VC host, or
/// (`driver = "hook"`) a pod that implements `factoryos/hook@1`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct KukaController {
    /// `kss` (default) or `hook`.
    #[serde(default, skip_serializing_if = "is_default")]
    pub driver: RobotDriver,
    /// The robot of the build whose `krl/<robot_ns>` runs on this controller.
    pub robot_ns: String,
    /// The iiQKA.OS2 instance (`/opt/kuka/instances/<instance>`); must be the active one.
    /// Required for `kss`; not used by `hook` (empty).
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub instance: String,
    #[serde(default = "default_netns")]
    pub netns: String,
    #[serde(default)]
    pub kli_ip: Option<String>,
    #[serde(default)]
    pub station: Option<String>,
    /// Access over `aws ssm send-command` (as root) ...
    #[serde(default)]
    pub ssm_instance: Option<String>,
    /// ... or over ssh (a host of `~/.ssh/config`, sudo without a password) ...
    #[serde(default)]
    pub ssh_host: Option<String>,
    /// ... or over `kubectl exec` into `<namespace>/<pod>/<container>` (an EKS cell's KUKA VC pod).
    #[serde(default)]
    pub kubectl_pod: Option<String>,
    /// Reset `sps.sub` to this instance's file on disk before adding `BT_SPS()` (e.g. the stock
    /// instance, to drop another application's submit loop); unset: patch the current file.
    #[serde(default)]
    pub sps_base: Option<String>,
}

fn default_netns() -> String {
    "krc1".into()
}

/// How plan2cell reaches a controller's host.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum Access {
    Ssm(String),
    Ssh(String),
    /// `<namespace>/<pod>/<container>`.
    Kubectl(String),
}

impl std::fmt::Display for Access {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Access::Ssm(i) => write!(f, "ssm:{i}"),
            Access::Ssh(h) => write!(f, "ssh:{h}"),
            Access::Kubectl(t) => write!(f, "kubectl:{t}"),
        }
    }
}

impl KukaController {
    pub fn is_hook(&self) -> bool {
        self.driver == RobotDriver::Hook
    }

    /// What the reports call the controller: its instance, or `hook` for a hook controller.
    pub fn label(&self) -> &str {
        if self.is_hook() { "hook" } else { &self.instance }
    }

    pub fn access(&self) -> Access {
        match (&self.ssm_instance, &self.kubectl_pod, &self.ssh_host) {
            (Some(i), _, _) => Access::Ssm(i.clone()),
            (None, Some(t), _) => Access::Kubectl(t.clone()),
            (None, None, Some(h)) => Access::Ssh(h.clone()),
            (None, None, None) => Access::Ssh(String::new()),
        }
    }
}

impl KukaVc {
    fn validate(&self) -> Result<()> {
        let mut seen = std::collections::HashSet::new();
        let mut slots = std::collections::HashSet::new();
        for c in &self.controllers {
            let what = format!("kuka_vc.controller robot_ns = \"{}\"", c.robot_ns);
            if c.is_hook() {
                if c.robot_ns.is_empty() {
                    return Err(Error::Config(format!("{what}: robot_ns must not be empty")));
                }
                match &c.kubectl_pod {
                    Some(t) if c.ssm_instance.is_none() && c.ssh_host.is_none() => {
                        crate::remote::Kubectl::parse(t, None)
                            .map_err(|e| Error::Config(format!("{what}: {e}")))?;
                    }
                    _ => {
                        return Err(Error::Config(format!(
                            "{what}: driver = \"hook\" needs kubectl_pod (and no ssm_instance or ssh_host)"
                        )));
                    }
                }
                if !seen.insert(c.robot_ns.clone()) {
                    return Err(Error::Config(format!("{what}: robot_ns given twice")));
                }
                continue;
            }
            if c.robot_ns.is_empty() || c.instance.is_empty() || c.netns.is_empty() {
                return Err(Error::Config(format!(
                    "{what}: robot_ns, instance and netns must not be empty"
                )));
            }
            let ways = [c.ssm_instance.is_some(), c.ssh_host.is_some(), c.kubectl_pod.is_some()];
            if ways.iter().filter(|w| **w).count() != 1 {
                return Err(Error::Config(format!(
                    "{what}: give exactly one of ssm_instance, ssh_host and kubectl_pod"
                )));
            }
            if let Some(t) = &c.kubectl_pod {
                crate::remote::Kubectl::parse(t, None).map_err(|e| Error::Config(format!("{what}: {e}")))?;
            }
            if !seen.insert(c.robot_ns.clone()) {
                return Err(Error::Config(format!("{what}: robot_ns given twice")));
            }
            for slot in [("netns", &c.netns), ("instance", &c.instance)] {
                if !slots.insert((c.access(), slot.0, slot.1.clone())) {
                    return Err(Error::Config(format!(
                        "{what}: {} {} is used by another controller on {}",
                        slot.0,
                        slot.1,
                        c.access()
                    )));
                }
            }
        }
        if !self.dir.starts_with('/') {
            return Err(Error::Config(format!(
                "kuka_vc.dir \"{}\": want an absolute path",
                self.dir
            )));
        }
        Ok(())
    }
}

/// What a stage does when a CODESYS the MCP server did not start is in its way (the server asks
/// "cannot be taken over": one opened by hand, or one an ended server left while it was still
/// coming up).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OtherCodesys {
    /// Stop with exit 3: a person may work in it (the shared cell, a cell someone uses over DCV).
    #[default]
    Refuse,
    /// Close it (`launch_codesys` `otherInstances: close`, like its window's X) and go on, but
    /// only when every such CODESYS has no unsaved changes; else refuse. For a cell nobody works
    /// in by hand: Factory OS's per-workspace cells.
    Close,
}

/// How `map_io` gets the IO mapping CSV to a Windows path for `import_io_mappings_csv` (the
/// server has no tool that writes a file from text).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IoCsv {
    /// The CSV becomes the declaration of a scratch GVL, `mirror_export` writes it to a file,
    /// `import_io_mappings_csv` reads that file, the GVL is deleted again.
    #[default]
    Mirror,
    /// The CSV is already at `<windows_project_dir>\<key>_io_mapping.csv` (copied there by hand
    /// or over SSM); only the import runs.
    Staged,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CodesysMcp {
    pub url: String,
    pub token_env: Option<String>,
    /// Where the projects are created on the Windows IDE; `{cell}` is replaced by the cell id.
    pub windows_project_dir: String,
    /// The CODESYS install every tool call names (`install`); empty: the server's choice.
    pub install: String,
    pub device_user_env: String,
    pub device_password_env: String,
    pub io_csv: IoCsv,
    /// Longest a single tool call may run.
    pub timeout_s: u64,
    /// How long the first call waits for the bridge (connection refused, no answer to
    /// initialize: an IDE that is still starting) before the stage fails.
    pub connect_wait_s: u64,
    pub other_codesys: OtherCodesys,
}

impl Default for CodesysMcp {
    fn default() -> Self {
        Self {
            url: "http://127.0.0.1:8766/mcp".into(),
            token_env: None,
            windows_project_dir: r"C:\ProgramData\plan2cell\projects\{cell}".into(),
            install: "SP22 Patch 1".into(),
            device_user_env: "PLAN2CELL_DEVICE_USER".into(),
            device_password_env: "PLAN2CELL_DEVICE_PASSWORD".into(),
            io_csv: IoCsv::Mirror,
            timeout_s: 1800,
            connect_wait_s: 300,
            other_codesys: OtherCodesys::Refuse,
        }
    }
}

fn env(var: &str) -> Option<String> {
    if var.is_empty() {
        return None;
    }
    std::env::var(var).ok().filter(|v| !v.is_empty())
}

impl CodesysMcp {
    /// The bearer token, if `token_env` names one.
    pub fn token(&self) -> Result<Option<String>> {
        let Some(var) = &self.token_env else {
            return Ok(None);
        };
        match env(var) {
            Some(t) => Ok(Some(t)),
            None => Err(Error::Config(format!(
                "codesys_mcp.token_env = \"{var}\" but ${var} is not set"
            ))),
        }
    }

    /// The runtime login (user, password) from `device_user_env` / `device_password_env`; `None`
    /// unless both are set.
    pub fn device_credentials(&self) -> Option<(String, String)> {
        Some((env(&self.device_user_env)?, env(&self.device_password_env)?))
    }

    /// `windows_project_dir` for this cell, without a trailing backslash.
    pub fn project_dir(&self, cell_id: &str) -> String {
        self.windows_project_dir
            .replace("{cell}", cell_id)
            .trim_end_matches(['\\', '/'])
            .to_string()
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PlcHost {
    /// `codesys` (default) or `hook`: a hook head needs `kubectl_pod` (no `ssm_instance`).
    #[serde(skip_serializing_if = "is_default")]
    pub driver: HeadDriver,
    pub ssh_host: String,
    /// Reach the PLC host over SSM instead of `ssh_host` (e.g. from a laptop without the
    /// workspace's ssh config); `region` / `profile` as for SSM.
    pub ssm_instance: Option<String>,
    /// Reach the PLC host over `kubectl exec` into `<namespace>/<pod>/<container>` (an EKS cell's PLC pod, its
    /// agent container), with `kubeconfig` (unset: `$KUBECONFIG`, else kubectl's default).
    pub kubectl_pod: Option<String>,
    pub kubeconfig: Option<String>,
    pub profile: Option<String>,
    pub setup: String,
    pub compose_dir: String,
    pub bucket: Option<String>,
    pub region: Option<String>,
}

impl Default for PlcHost {
    fn default() -> Self {
        Self {
            driver: HeadDriver::Codesys,
            ssh_host: "plc-host".into(),
            ssm_instance: None,
            kubectl_pod: None,
            kubeconfig: None,
            profile: None,
            setup: "/opt/plan2cell/plc-host/setup.sh".into(),
            compose_dir: "/opt/plan2cell/plc-host".into(),
            bucket: None,
            region: None,
        }
    }
}

impl PlcHost {
    pub fn is_hook(&self) -> bool {
        self.driver == HeadDriver::Hook
    }

    /// The software bucket and region `setup.sh` installs from.
    pub fn bucket_region(&self) -> Result<(String, String)> {
        let bucket = self.bucket.clone().or_else(|| env("PLAN2CELL_BUCKET"));
        let region = self
            .region
            .clone()
            .or_else(|| env("AWS_REGION"))
            .or_else(|| env("AWS_DEFAULT_REGION"));
        match (bucket, region) {
            (Some(b), Some(r)) => Ok((b, r)),
            _ => Err(Error::Config(
                "the PLC host setup needs the software bucket and region: set plc_host.bucket / \
                 plc_host.region in cell.toml, or $PLAN2CELL_BUCKET and $AWS_REGION"
                    .into(),
            )),
        }
    }
}

impl CellConfig {
    /// `~/.config/plan2cell/cell.toml`.
    pub fn default_path() -> PathBuf {
        let home = std::env::var_os("HOME")
            .map(PathBuf::from)
            .unwrap_or_default();
        home.join(".config/plan2cell/cell.toml")
    }

    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).map_err(|e| match e.kind() {
            std::io::ErrorKind::NotFound => Error::Config(format!(
                "{} not found (the workspace setup writes it; or give --cell-config)",
                path.display()
            )),
            _ => Error::io(path, e),
        })?;
        Self::parse(&text).map_err(|e| match e {
            Error::Config(m) => Error::Config(format!("{}: {m}", path.display())),
            other => other,
        })
    }

    pub fn parse(text: &str) -> Result<Self> {
        let c: CellConfig = toml::from_str(text).map_err(|e| Error::Config(e.to_string()))?;
        let m = &c.codesys_mcp;
        if m.url.is_empty() {
            return Err(Error::Config("codesys_mcp.url must not be empty".into()));
        }
        if !m.windows_project_dir.contains(['\\', '/']) {
            return Err(Error::Config(format!(
                "codesys_mcp.windows_project_dir \"{}\": want an absolute Windows path",
                m.windows_project_dir
            )));
        }
        if let Some(k) = &c.kuka_vc {
            k.validate()?;
        }
        if let Some(t) = &c.plc_host.kubectl_pod {
            crate::remote::Kubectl::parse(t, None).map_err(|e| Error::Config(format!("plc_host: {e}")))?;
            if c.plc_host.ssm_instance.is_some() {
                return Err(Error::Config("plc_host: give one of ssm_instance and kubectl_pod".into()));
            }
        }
        if c.plc_host.is_hook() && c.plc_host.kubectl_pod.is_none() {
            return Err(Error::Config(
                "plc_host: driver = \"hook\" needs kubectl_pod (<namespace>/<pod>/<container>)".into(),
            ));
        }
        Ok(c)
    }

    /// The `[[kuka_vc.controller]]`s (none without `[kuka_vc]`).
    pub fn controllers(&self) -> &[KukaController] {
        self.kuka_vc.as_ref().map(|k| k.controllers.as_slice()).unwrap_or_default()
    }

    /// Some controller is driven through its hook.
    pub fn has_hook_robots(&self) -> bool {
        self.controllers().iter().any(KukaController::is_hook)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CONTRACT: &str = r#"
[codesys_mcp]
url = "http://10.0.0.5:8766/mcp"
token_env = "PLAN2CELL_CODESYS_MCP_TOKEN"
windows_project_dir = 'C:\ProgramData\plan2cell\projects\{cell}'
install = "SP22 Patch 1"
device_user_env = "PLAN2CELL_DEVICE_USER"
device_password_env = "PLAN2CELL_DEVICE_PASSWORD"
[plc_host]
ssh_host = "plc-host"
setup = "/opt/plan2cell/plc-host/setup.sh"
compose_dir = "/opt/plan2cell/plc-host"
"#;

    #[test]
    fn the_workspace_file_parses() {
        let c = CellConfig::parse(CONTRACT).unwrap();
        assert_eq!(c.codesys_mcp.url, "http://10.0.0.5:8766/mcp");
        assert_eq!(
            c.codesys_mcp.token_env.as_deref(),
            Some("PLAN2CELL_CODESYS_MCP_TOKEN")
        );
        assert_eq!(c.plc_host.ssh_host, "plc-host");
        assert_eq!(
            c.codesys_mcp.project_dir("welding_cell_2_small"),
            r"C:\ProgramData\plan2cell\projects\welding_cell_2_small"
        );
        assert_eq!(c.codesys_mcp.install, "SP22 Patch 1");
        assert_eq!(c.codesys_mcp.io_csv, IoCsv::Mirror);
    }

    #[test]
    fn other_codesys_defaults_to_refuse() {
        let c = CellConfig::parse(CONTRACT).unwrap();
        assert_eq!(c.codesys_mcp.other_codesys, OtherCodesys::Refuse);
        assert_eq!(c.codesys_mcp.connect_wait_s, 300);
        let c =
            CellConfig::parse("[codesys_mcp]\nother_codesys = \"close\"\nconnect_wait_s = 30\n")
                .unwrap();
        assert_eq!(c.codesys_mcp.other_codesys, OtherCodesys::Close);
        assert_eq!(c.codesys_mcp.connect_wait_s, 30);
        assert!(CellConfig::parse("[codesys_mcp]\nother_codesys = \"kill\"\n").is_err());
    }

    #[test]
    fn unknown_keys_are_ignored() {
        let c = CellConfig::parse(
            "[codesys_mcp]\nscript_tool = \"execute_script\"\nscript_arg = \"script\"\n\
             gateway = \"localhost:1217\"\nproject_dir = 'C:\\x'\n[other]\nx = 1\n",
        )
        .unwrap();
        assert_eq!(c.codesys_mcp, CodesysMcp::default());
    }

    #[test]
    fn bad_values_are_rejected() {
        for t in [
            "[codesys_mcp]\nurl = \"\"\n",
            "[codesys_mcp]\nwindows_project_dir = \"projects\"\n",
            "[codesys_mcp]\nio_csv = \"ftp\"\n",
            "[codesys_mcp]\ntimeout_s = \"long\"\n",
        ] {
            assert!(CellConfig::parse(t).is_err(), "{t}");
        }
    }

    #[test]
    fn kuka_controllers_parse_and_are_checked() {
        let base = "[kuka_vc]\nprofile = \"dev\"\n\
                    [[kuka_vc.controller]]\nrobot_ns = \"robot1\"\ninstance = \"A\"\n\
                    ssm_instance = \"i-1\"\nsps_base = \"S\"\n\
                    [[kuka_vc.controller]]\nrobot_ns = \"robot2\"\ninstance = \"B\"\n\
                    netns = \"krc2\"\nssm_instance = \"i-1\"\n";
        let c = CellConfig::parse(base).unwrap();
        let k = c.kuka_vc.unwrap();
        assert_eq!(k.dir, "/var/lib/plan2cell/kuka-vc");
        assert_eq!(k.profile.as_deref(), Some("dev"));
        assert_eq!(k.controllers.len(), 2);
        assert_eq!(k.controllers[0].netns, "krc1");
        assert_eq!(k.controllers[0].access(), Access::Ssm("i-1".into()));
        assert_eq!(k.controllers[1].netns, "krc2");
        assert!(CellConfig::parse("").unwrap().kuka_vc.is_none());
        for bad in [
            base.replace("netns = \"krc2\"", "netns = \"krc1\""),
            base.replace("instance = \"B\"", "instance = \"A\""),
            base.replace("robot_ns = \"robot2\"", "robot_ns = \"robot1\""),
            base.replace("ssm_instance = \"i-1\"\nsps_base", "sps_base"),
            base.replace("sps_base = \"S\"", "ssh_host = \"vc\""),
            base.replace("[kuka_vc]\n", "[kuka_vc]\ndir = \"rel\"\n"),
        ] {
            assert!(CellConfig::parse(&bad).is_err(), "{bad}");
        }
        // Another host may reuse the netns.
        CellConfig::parse(&base.replace(
            "netns = \"krc2\"\nssm_instance = \"i-1\"",
            "ssh_host = \"vc2\"",
        ))
        .unwrap();
    }

    #[test]
    fn address_map_parses() {
        let c = CellConfig::parse("[address_map]\n\"192.168.100.10\" = \"172.31.5.11\"\n").unwrap();
        assert_eq!(
            c.address_map.get("192.168.100.10").map(String::as_str),
            Some("172.31.5.11")
        );
        assert!(CellConfig::parse("").unwrap().address_map.is_empty());
    }

    #[test]
    fn profinet_parses() {
        let c = CellConfig::parse("[profinet]\ngateway = \"0.0.0.0\"\n").unwrap();
        let p = c.profinet.unwrap();
        assert_eq!((p.gateway.as_deref(), p.head_ip), (Some("0.0.0.0"), None));
        assert!(CellConfig::parse("").unwrap().profinet.is_none());
    }

    #[test]
    fn bucket_from_the_file() {
        let c = CellConfig::parse("[plc_host]\nbucket = \"b\"\nregion = \"r\"\n").unwrap();
        assert_eq!(
            c.plc_host.bucket_region().unwrap(),
            ("b".into(), "r".into())
        );
    }

    #[test]
    fn kubectl_pods_for_an_eks_cell() {
        let base = "[codesys_mcp]\nurl = \"http://ide:8766/mcp\"\n[plc_host]\nkubectl_pod = \"cell-jdoe/n1-plc/agent\"\n\
                    [kuka_vc]\nkubeconfig = \"/var/lib/factoryos/cell/kubeconfig\"\n[[kuka_vc.controller]]\nrobot_ns = \"robot1\"\n\
                    instance = \"A\"\nkubectl_pod = \"cell-jdoe/n1-kvc-1/kvc\"\n";
        let c = CellConfig::parse(base).unwrap();
        assert_eq!(c.plc_host.kubectl_pod.as_deref(), Some("cell-jdoe/n1-plc/agent"));
        let k = c.kuka_vc.as_ref().unwrap();
        assert_eq!(k.controllers[0].access(), Access::Kubectl("cell-jdoe/n1-kvc-1/kvc".into()));
        assert_eq!(k.kubeconfig.as_deref(), Some("/var/lib/factoryos/cell/kubeconfig"));
        // Exactly one way per controller; well-formed targets only.
        assert!(CellConfig::parse(&base.replace("kubectl_pod = \"cell-jdoe/n1-kvc-1/kvc\"", "kubectl_pod = \"cell-jdoe/n1-kvc-1/kvc\"\nssh_host = \"vc\"")).is_err());
        assert!(CellConfig::parse(&base.replace("cell-jdoe/n1-kvc-1/kvc", "cell-jdoe/n1-kvc-1")).is_err());
        assert!(CellConfig::parse(&base.replace("cell-jdoe/n1-plc/agent", "Cell/plc/agent")).is_err());
    }

    /// The smoke cell's cell.toml (docs/tasks/cell-templates.md E8): no [codesys_mcp].
    const HOOKS: &str = r#"
[plc_host]
driver = "hook"
kubectl_pod = "cell-jdoe/n1-head/head"
kubeconfig = "/var/lib/factoryos/cell/kubeconfig"
region = "eu-central-1"

[kuka_vc]
kubeconfig = "/var/lib/factoryos/cell/kubeconfig"
region = "eu-central-1"

[[kuka_vc.controller]]
driver = "hook"
robot_ns = "robot1"
station = "kuka-robi1"
kli_ip = "10.77.1.11"
kubectl_pod = "cell-jdoe/n1-robot-1/robot"

[[kuka_vc.controller]]
driver = "hook"
robot_ns = "robot2"
station = "kuka-robi2"
kli_ip = "10.77.1.12"
kubectl_pod = "cell-jdoe/n1-robot-2/robot"

[address_map]

[profinet]
gateway = "10.77.1.1"
head_ip = "10.77.1.10"
netmask = "255.255.255.0"
"#;

    #[test]
    fn hook_drivers_parse() {
        let c = CellConfig::parse(HOOKS).unwrap();
        assert!(c.plc_host.is_hook());
        assert_eq!(c.codesys_mcp, CodesysMcp::default());
        let k = c.controllers();
        assert_eq!(k.len(), 2);
        assert!(k.iter().all(KukaController::is_hook));
        assert_eq!(k[1].instance, "");
        assert_eq!(k[1].label(), "hook");
        assert_eq!(k[1].access(), Access::Kubectl("cell-jdoe/n1-robot-2/robot".into()));
        assert!(c.has_hook_robots());
        // Two hook controllers in one pod's default netns slot are fine (no instance / netns).
        // Old files: the defaults.
        let old = CellConfig::parse(CONTRACT).unwrap();
        assert_eq!(old.plc_host.driver, HeadDriver::Codesys);
        assert!(!old.has_hook_robots());
        let kss = CellConfig::parse("[[kuka_vc.controller]]\nrobot_ns = \"r\"\ninstance = \"A\"\nssh_host = \"h\"\n").unwrap();
        assert_eq!(kss.controllers()[0].driver, RobotDriver::Kss);
    }

    #[test]
    fn hook_drivers_are_checked() {
        for bad in [
            // A hook head needs its pod.
            HOOKS.replace("kubectl_pod = \"cell-jdoe/n1-head/head\"\n", ""),
            // A hook controller needs kubectl_pod, and only that way.
            HOOKS.replace("kubectl_pod = \"cell-jdoe/n1-robot-1/robot\"", ""),
            HOOKS.replace("kubectl_pod = \"cell-jdoe/n1-robot-1/robot\"", "kubectl_pod = \"cell-jdoe/n1-robot-1/robot\"\nssm_instance = \"i-1\""),
            HOOKS.replace("robot_ns = \"robot2\"", "robot_ns = \"robot1\""),
            HOOKS.replace("robot_ns = \"robot2\"", "robot_ns = \"\""),
            HOOKS.replace("driver = \"hook\"\nrobot_ns = \"robot1\"", "driver = \"krl-sim\"\nrobot_ns = \"robot1\""),
            // A KSS controller still needs its instance.
            HOOKS.replace("driver = \"hook\"\nrobot_ns = \"robot1\"", "robot_ns = \"robot1\""),
        ] {
            assert!(CellConfig::parse(&bad).is_err(), "{bad}");
        }
    }
}