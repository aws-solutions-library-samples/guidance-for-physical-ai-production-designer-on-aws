//! The `krl` stage's pieces: the bt2krl output of one robot as a bundle for the KUKA VC host,
//! the host-side helper (`infra/kuka-vc/p2c-kvc.py`) and its answers, and the remotes of the
//! controllers.
//!
//! ```text
//! krl     per [[kuka_vc.controller]]: check krl/<robot_ns> against bt2krl-manifest.json (fnv1a64),
//!         copy the helper and the bundle to the host, `p2c-kvc.py deploy`: upload what differs,
//!         $config.dat / sps.sub between ;BT2KRL markers, compile errors, submit restart,
//!         acknowledgement messages confirmed
//! verify  `p2c-kvc.py verify`: files read back with the manifest's hashes, markers, no compile
//!         errors, submit running, PLAN_ID of the production Folge, no STATE / stop messages
//! ```

use std::collections::{BTreeMap, HashMap};

use serde::{Deserialize, Serialize};

use crate::config::{Access, KukaController, KukaVc};
use crate::remote::{Remote, Ssh, Ssm, base64};
use crate::stages::Tree;
use crate::{Error, Result};

/// The helper plan2cell runs on the KUKA VC host (as root).
pub const HELPER: &str = include_str!("../../../infra/kuka-vc/p2c-kvc.py");
/// The head stub for tests in IO mode sim (installed next to the helper).
pub const HEADSTUB: &str = include_str!("../../../infra/kuka-vc/p2c-headstub.py");
/// The T1 test of the home and production Folge with the head stub (installed next to the helper).
pub const T1TEST: &str = include_str!("../../../infra/kuka-vc/kvc-t1-test.sh");
pub const BUNDLE_SCHEMA: &str = "plan2cell/krl-bundle@1";
pub const MANIFEST_SCHEMA: &str = "bt2krl/manifest@1";

/// 64-bit FNV-1a, as bt2krl writes it into the manifest.
pub fn fnv1a64(b: &[u8]) -> String {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &x in b {
        h ^= u64::from(x);
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    format!("{h:016x}")
}

#[derive(Clone, Debug, Deserialize)]
pub struct KrlManifest {
    pub schema: String,
    pub plan_id: String,
    #[serde(default)]
    pub profile: String,
    pub robot: KrlRobot,
    pub files: Vec<KrlFile>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct KrlRobot {
    pub ns: String,
}

#[derive(Clone, Debug, Deserialize)]
pub struct KrlFile {
    pub path: String,
    pub fnv1a64: String,
}

/// `krl/<ns>/` of a build, checked against its manifest.
pub struct Bundle {
    pub manifest: KrlManifest,
    /// The bundle file the helper reads (`plan2cell/krl-bundle@1`).
    pub json: String,
}

pub fn manifest_path(ns: &str) -> String {
    format!("krl/{ns}/bt2krl-manifest.json")
}

impl Bundle {
    /// Read `krl/<ns>` of the build; every file of the manifest must have its hash.
    pub fn load(tree: &Tree, ns: &str) -> Result<Self> {
        let mtext = tree.read(&manifest_path(ns))?;
        let raw: serde_json::Value = serde_json::from_str(&mtext)
            .map_err(|e| Error::Config(format!("{}: {e}", manifest_path(ns))))?;
        let manifest: KrlManifest = serde_json::from_value(raw.clone())
            .map_err(|e| Error::Config(format!("{}: {e}", manifest_path(ns))))?;
        if manifest.schema != MANIFEST_SCHEMA {
            return Err(Error::Config(format!(
                "{}: schema {}, want {MANIFEST_SCHEMA}",
                manifest_path(ns),
                manifest.schema
            )));
        }
        if manifest.robot.ns != ns {
            return Err(Error::Config(format!(
                "{}: robot {}, want {ns}",
                manifest_path(ns),
                manifest.robot.ns
            )));
        }
        let mut files = BTreeMap::new();
        let mut bad = Vec::new();
        for f in &manifest.files {
            if f.path.contains("..") || f.path.starts_with('/') {
                return Err(Error::Config(format!(
                    "{}: bad path {}",
                    manifest_path(ns),
                    f.path
                )));
            }
            let text = tree.read(&format!("krl/{ns}/{}", f.path))?;
            let h = fnv1a64(text.as_bytes());
            if h != f.fnv1a64 {
                bad.push(format!("{} (fnv1a64 {h}, manifest {})", f.path, f.fnv1a64));
            }
            files.insert(f.path.clone(), base64(text.as_bytes()));
        }
        if !bad.is_empty() {
            return Err(Error::Failed {
                stage: "krl",
                code: 7,
                msg: format!(
                    "krl/{ns} does not match its manifest: {} (rebuild)",
                    bad.join(", ")
                ),
            });
        }
        let json = serde_json::json!({
            "schema": BUNDLE_SCHEMA,
            "robot_ns": ns,
            "manifest": raw,
            "files": files,
        })
        .to_string();
        Ok(Self { manifest, json })
    }

    /// `ns plan_id`, the tag of the ;BT2KRL markers.
    pub fn tag(&self) -> String {
        format!("{} {}", self.manifest.robot.ns, self.manifest.plan_id)
    }
}

/// The paths on the host.
pub struct HostPaths {
    pub helper: String,
    pub headstub: String,
    pub t1test: String,
    pub bundle: String,
}

pub fn host_paths(k: &KukaVc, ns: &str) -> HostPaths {
    let d = k.dir.trim_end_matches('/');
    HostPaths {
        helper: format!("{d}/p2c-kvc.py"),
        headstub: format!("{d}/p2c-headstub.py"),
        t1test: format!("{d}/kvc-t1-test.sh"),
        bundle: format!("{d}/bundles/{ns}.json"),
    }
}

/// A helper call: always exits 0 and ends with `p2c-exit=<code>` (so stdout survives a failure).
pub fn helper_cmd(k: &KukaVc, c: &KukaController, sub: &str, extra: &[String]) -> String {
    use crate::stages::sh;
    let p = host_paths(k, &c.robot_ns);
    let mut cmd = format!(
        "sudo -n python3 -I {} {sub} --netns {} --instance {}",
        sh(&p.helper),
        sh(&c.netns),
        sh(&c.instance)
    );
    if sub != "status" {
        cmd.push_str(&format!(" --bundle {}", sh(&p.bundle)));
    }
    for e in extra {
        cmd.push(' ');
        cmd.push_str(&sh(e));
    }
    format!("{cmd} 2>&1; echo p2c-exit=$?")
}

/// The answer of the helper: its exit code and the JSON object it printed last.
#[derive(Debug)]
pub struct Answer {
    pub code: i32,
    pub json: serde_json::Value,
    /// Lines that were not the answer (warnings, a traceback).
    pub other: Vec<String>,
}

pub fn parse_answer(out: &str) -> Result<Answer> {
    let mut code = None;
    let mut json = None;
    let mut other = Vec::new();
    for line in out.lines() {
        let l = line.trim();
        if let Some(c) = l.strip_prefix("p2c-exit=") {
            code = c.trim().parse().ok();
        } else if l.starts_with('{')
            && let Ok(v) = serde_json::from_str::<serde_json::Value>(l)
        {
            json = Some(v);
        } else if !l.is_empty() {
            other.push(l.to_string());
        }
    }
    match (code, json) {
        (Some(code), Some(json)) => Ok(Answer { code, json, other }),
        (code, _) => Err(Error::Remote(format!(
            "the KUKA VC helper gave no answer (exit {}): {}",
            code.map_or("?".into(), |c| c.to_string()),
            other
                .iter()
                .rev()
                .take(5)
                .rev()
                .cloned()
                .collect::<Vec<_>>()
                .join(" | ")
        ))),
    }
}

/// The strings of a JSON array (or `[]`).
pub fn strings(v: &serde_json::Value) -> Vec<String> {
    v.as_array()
        .map(|a| {
            a.iter()
                .map(|x| match x {
                    serde_json::Value::String(s) => s.clone(),
                    other => other.to_string(),
                })
                .collect()
        })
        .unwrap_or_default()
}

/// The messages of an answer as `KEY TYPE text`.
pub fn messages(v: &serde_json::Value) -> Vec<String> {
    v.as_array()
        .map(|a| {
            a.iter()
                .map(|m| {
                    format!(
                        "{} {}{} {}",
                        m["key"].as_str().unwrap_or("?"),
                        m["type"].as_str().unwrap_or(""),
                        if m["stop"].as_bool() == Some(true) {
                            " (stop)"
                        } else {
                            ""
                        },
                        m["text"].as_str().unwrap_or("")
                    )
                    .trim_end()
                    .to_string()
                })
                .collect()
        })
        .unwrap_or_default()
}

/// The compile errors of an answer as `module:line error n param`.
pub fn compile_errors(v: &serde_json::Value) -> Vec<String> {
    v.as_array()
        .map(|a| {
            a.iter()
                .map(|e| match e["error"].as_str() {
                    Some(t) => format!("{}: {t}", e["module"].as_str().unwrap_or("?")),
                    None => format!(
                        "{}:{} error {} {}",
                        e["module"].as_str().unwrap_or("?"),
                        e["line"],
                        e["number"],
                        e["param"].as_str().unwrap_or("")
                    )
                    .trim_end()
                    .to_string(),
                })
                .collect()
        })
        .unwrap_or_default()
}

/// The remote of each controller's host.
pub trait Remotes {
    fn remote(&mut self, c: &KukaController) -> Result<&mut dyn Remote>;
}

/// SSM or ssh per `[[kuka_vc.controller]]`; one connection object per host.
pub struct VcRemotes {
    cfg: KukaVc,
    map: HashMap<Access, Box<dyn Remote>>,
}

impl VcRemotes {
    pub fn new(cfg: KukaVc) -> Self {
        Self {
            cfg,
            map: HashMap::new(),
        }
    }
}

impl Remotes for VcRemotes {
    fn remote(&mut self, c: &KukaController) -> Result<&mut dyn Remote> {
        let a = c.access();
        if !self.map.contains_key(&a) {
            let r: Box<dyn Remote> = match &a {
                Access::Ssm(i) => {
                    let mut s = Ssm::new(i.clone());
                    s.region = self.cfg.region.clone();
                    s.profile = self.cfg.profile.clone();
                    s.timeout = std::time::Duration::from_secs(self.cfg.timeout_s);
                    Box::new(s)
                }
                Access::Ssh(h) => Box::new(Ssh::new(h.clone())),
                Access::Kubectl(t) => Box::new(crate::remote::Kubectl::parse(t, self.cfg.kubeconfig.clone())?),
            };
            self.map.insert(a.clone(), r);
        }
        Ok(self.map.get_mut(&a).expect("inserted").as_mut())
    }
}

/// One remote per host for tests (or any [`Remote`]).
impl<R: Remote> Remotes for HashMap<Access, R> {
    fn remote(&mut self, c: &KukaController) -> Result<&mut dyn Remote> {
        let a = c.access();
        self.get_mut(&a)
            .map(|r| r as &mut dyn Remote)
            .ok_or_else(|| Error::Remote(format!("no remote for {a}")))
    }
}

/// The `krl` / `verify` results of one controller in the report.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ControllerReport {
    pub robot_ns: String,
    pub instance: String,
    pub host: String,
    pub plan_id: Option<String>,
    pub deployed: Option<bool>,
    #[serde(default)]
    pub changed: Vec<String>,
    #[serde(default)]
    pub errors: Vec<String>,
    #[serde(default)]
    pub messages: Vec<String>,
    pub verified: Option<bool>,
    #[serde(default)]
    pub checks: Vec<crate::stages::CheckResult>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    fn example() -> Tree {
        Tree::Dir(Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/welding_cell_2_small"))
    }

    #[test]
    fn the_example_bundle_matches_its_manifest() {
        let b = Bundle::load(&example(), "robot1").unwrap();
        assert_eq!(b.tag(), "robot1 16#F77E");
        let v: serde_json::Value = serde_json::from_str(&b.json).unwrap();
        assert_eq!(v["schema"], BUNDLE_SCHEMA);
        assert_eq!(v["manifest"]["plan_id"], "16#F77E");
        assert_eq!(v["files"].as_object().unwrap().len(), 9);
        assert!(
            v["files"]["KRC/R1/Folgen/folge11.src"]
                .as_str()
                .unwrap()
                .len()
                > 100
        );
    }

    #[test]
    fn a_changed_file_is_refused() {
        let Tree::Dir(d) = example() else {
            unreachable!()
        };
        let mut files = BTreeMap::new();
        let root = d.join("krl/robot1");
        for p in [
            "bt2krl-manifest.json",
            "INSTALL.md",
            "KRC/R1/Folgen/folge11.dat",
            "KRC/R1/Folgen/folge11.src",
            "KRC/R1/Folgen/folge125.dat",
            "KRC/R1/Folgen/folge125.src",
            "KRC/R1/Program/bt_cell.dat",
            "KRC/R1/Program/bt_cell.src",
            "KRC/R1/System/bt_config.dat",
            "KRC/R1/System/bt_sps.src",
        ] {
            files.insert(
                format!("krl/robot1/{p}"),
                std::fs::read_to_string(root.join(p)).unwrap(),
            );
        }
        let ok = Tree::Mem {
            dir: d.clone(),
            files: files.clone(),
        };
        Bundle::load(&ok, "robot1").unwrap();
        files
            .get_mut("krl/robot1/KRC/R1/Folgen/folge11.src")
            .unwrap()
            .push(' ');
        let e = Bundle::load(&Tree::Mem { dir: d, files }, "robot1")
            .err()
            .unwrap();
        assert_eq!(e.exit_code(), 7);
        assert!(e.to_string().contains("KRC/R1/Folgen/folge11.src"), "{e}");
        assert!(Bundle::load(&ok, "robot2").is_err());
    }

    #[test]
    fn answers() {
        let a = parse_answer("warning x\n{\"ok\":false,\"error\":\"e\"}\np2c-exit=4\n").unwrap();
        assert_eq!(a.code, 4);
        assert_eq!(a.json["error"], "e");
        assert_eq!(a.other, ["warning x"]);
        assert!(parse_answer("Traceback\np2c-exit=1\n").is_err());
        let e = serde_json::json!([{"module":"KRC:\\R1\\x.src","line":3,"number":2057,"param":"BT_X"},
                                   {"module":"m","error":"failed"}]);
        assert_eq!(
            compile_errors(&e),
            ["KRC:\\R1\\x.src:3 error 2057 BT_X", "m: failed"]
        );
        let m = serde_json::json!([{"key":"M_1350","type":"STATE","stop":true,"text":"BCO"}]);
        assert_eq!(messages(&m), ["M_1350 STATE (stop) BCO"]);
    }

    #[test]
    fn helper_commands() {
        let k = KukaVc::default();
        let c = KukaController {
            driver: Default::default(),
            robot_ns: "robot1".into(),
            instance: "iiQKA.OS2_KSS_9.2.0_vass".into(),
            netns: "krc1".into(),
            kli_ip: None,
            station: None,
            ssm_instance: Some("i-1".into()),
            ssh_host: None,
            kubectl_pod: None,
            sps_base: None,
        };
        assert_eq!(
            helper_cmd(&k, &c, "deploy", &["--sps-base".into(), "a b".into()]),
            "sudo -n python3 -I /var/lib/plan2cell/kuka-vc/p2c-kvc.py deploy --netns krc1 \
             --instance iiQKA.OS2_KSS_9.2.0_vass --bundle /var/lib/plan2cell/kuka-vc/bundles/robot1.json \
             --sps-base 'a b' 2>&1; echo p2c-exit=$?"
        );
        assert!(HELPER.contains("def cmd_deploy"));
        assert!(HEADSTUB.contains("t1-bco-unlatch"));
    }
}
