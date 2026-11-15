//! The deployability check: can this export be deployed to, and run on, its target cell?
//!
//! Read-only and fast (no CODESYS, no controller writes): every rule reads the export's files
//! (what `plan2cell cell projects / deploy / krl / start` would put on the cell) and, with a
//! [`Target`], the cell's facts (its KUKA virtual controllers, their PROFINET addresses, the
//! globals their `$config.dat` declares). Each rule ends `pass`, `warn`, `fail` or `skip`, with
//! a precise message, a hint (which setting, where in Factory OS) and, where the fix is
//! mechanical, [`Fix`]es (a Hardware Configuration value, an export option, a re-export).
//!
//! Without a target only the cell-independent rules run ([`Scope::Export`]): `plan2cell build`
//! adds their findings to its diagnostics. `plan2cell cell check DIR` runs all of them against
//! the cell of its cell.toml (`$config.dat` read over SSM where it can, else the bundled list).
//!
//! The failures it exists for (2026-10-16): a Folge writing the VW package's module-local
//! `P_PktNr_SPS` (KSS error 2137 in the krl stage, after 10 min of projects / deploy); a safety
//! module, no safety bypass or no External on the virtual cell's KUKA VCs (Start production:
//! `ERR_SAFETY_TIMEOUT`); gateway `0.0.0.0` (the head re-addresses the VCs with DCP Set IP);
//! CODESYS robot simulators deployed to the KUKA VCs' addresses ("No gateway is configured").

use std::collections::{BTreeMap, BTreeSet};
use std::net::Ipv4Addr;
use std::path::PathBuf;

use serde::{Deserialize, Serialize};
use serde_json::Value;

pub const SCHEMA: &str = "plan2cell/deploy-check@1";

/// Where a hint sends the user (the Factory OS viewer's sidebar).
const HW: &str = "Hardware Configuration (step 2, below Export PLC + robot code)";
const REEXPORT: &str = "then Export PLC + robot code again (step 2)";

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Status {
    Pass,
    Skip,
    Warn,
    Fail,
}

impl Status {
    pub fn mark(self) -> &'static str {
        match self {
            Status::Pass => "✓",
            Status::Skip => "·",
            Status::Warn => "⚠",
            Status::Fail => "✗",
        }
    }
}

/// What a rule needs: the export alone, or the cell too.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Scope {
    Export,
    Cell,
}

/// A mechanical fix of a rule. Every one needs a re-export afterwards (the code is generated from
/// these settings); [`Fix::Reexport`] alone: only that (the generator changed, or the settings
/// already are right but the export is older).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Fix {
    /// A Hardware Configuration value: `field` as the form names it (`network.gateway`,
    /// `robots.robot1.safetyModule`, ...), `value` the one the target needs.
    Hardware { field: String, value: Value },
    /// An option of the export (`profile`, `jobNumber`, ... as the export form sends them).
    Export { option: String, value: Value },
    /// Export the same plan again with the same settings.
    Reexport { reason: String },
}

/// One rule's outcome.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RuleResult {
    /// `krl.names`, `cell.safety`, ...
    pub id: String,
    pub title: String,
    pub scope: Scope,
    pub status: Status,
    /// What was found (pass: what was checked).
    pub message: String,
    /// What to change, and where.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub hint: Option<String>,
    /// The robots the finding is about.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub robots: Vec<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub fixes: Vec<Fix>,
    /// The generated code itself is wrong for the target (not a setting): which generator, what,
    /// and where it is fixed (`re-export` then gets the fixed code).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub generator_bug: Option<String>,
    /// More lines (each finding), for the report's details.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub details: Vec<String>,
}

impl RuleResult {
    fn new(id: &str, title: &str, scope: Scope) -> Self {
        Self {
            id: id.into(),
            title: title.into(),
            scope,
            status: Status::Pass,
            message: String::new(),
            hint: None,
            robots: Vec::new(),
            fixes: Vec::new(),
            generator_bug: None,
            details: Vec::new(),
        }
    }

    fn pass(mut self, message: impl Into<String>) -> Self {
        self.status = Status::Pass;
        self.message = message.into();
        self
    }

    fn skip(mut self, message: impl Into<String>) -> Self {
        self.status = Status::Skip;
        self.message = message.into();
        self
    }

    fn with(mut self, status: Status, message: impl Into<String>, hint: Option<String>) -> Self {
        self.status = status;
        self.message = message.into();
        self.hint = hint;
        self
    }

    /// One line: `✗ cell.safety: message (hint)`.
    pub fn line(&self) -> String {
        let mut s = format!("{} {}: {}", self.status.mark(), self.title, self.message);
        if let Some(h) = &self.hint
            && self.status >= Status::Warn
        {
            s.push_str(&format!(" Fix: {h}"));
        }
        s
    }
}

/// The report of [`check`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CheckReport {
    pub schema: String,
    /// The export's directory (or build id), as given.
    pub export: String,
    pub cell_id: String,
    pub plan_id: String,
    pub profile: String,
    /// The target, e.g. "virtual cell (2 KUKA VCs: robot1 kuka-robi1 192.168.100.11, ...)";
    /// `None`: cell-independent rules only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target: Option<String>,
    /// No rule failed.
    pub ok: bool,
    pub fails: usize,
    pub warns: usize,
    pub rules: Vec<RuleResult>,
    /// How long it took (seconds).
    #[serde(default)]
    pub elapsed_s: f64,
}

impl CheckReport {
    pub fn new(export: &str, rules: Vec<RuleResult>) -> Self {
        let mut r = Self { schema: SCHEMA.into(), export: export.into(), rules, ..Default::default() };
        r.count();
        r
    }

    /// Recount after rules were added or changed.
    pub fn count(&mut self) {
        self.fails = self.rules.iter().filter(|r| r.status == Status::Fail).count();
        self.warns = self.rules.iter().filter(|r| r.status == Status::Warn).count();
        self.ok = self.fails == 0;
    }

    /// Every fix of the failing (and, with `warnings`, warning) rules, `rules` only (empty: all).
    pub fn fixes(&self, rules: &[String], warnings: bool) -> Vec<(&str, &Fix)> {
        self.rules
            .iter()
            .filter(|r| r.status == Status::Fail || (warnings && r.status == Status::Warn))
            .filter(|r| rules.is_empty() || rules.iter().any(|x| x == &r.id))
            .flat_map(|r| r.fixes.iter().map(move |f| (r.id.as_str(), f)))
            .collect()
    }

    /// The text report: one line per rule.
    pub fn text(&self) -> String {
        let mut s = format!(
            "deployability of {} (PLAN_ID {}, profile {}){}: {}\n",
            self.export,
            self.plan_id,
            self.profile,
            self.target.as_deref().map(|t| format!(" on {t}")).unwrap_or_default(),
            if self.ok {
                if self.warns > 0 { format!("deployable, {} warning(s)", self.warns) } else { "deployable".into() }
            } else {
                format!("NOT deployable: {} problem(s), {} warning(s)", self.fails, self.warns)
            }
        );
        for r in &self.rules {
            s.push_str(&format!("  {}\n", r.line()));
            if r.status >= Status::Warn {
                for d in &r.details {
                    s.push_str(&format!("      {d}\n"));
                }
            }
        }
        s
    }

    /// The findings of the cell-independent rules as build diagnostics (warn: warning, fail:
    /// error).
    pub fn diagnostics(&self) -> Vec<crate::report::Diagnostic> {
        self.rules
            .iter()
            .filter(|r| r.status >= Status::Warn)
            .map(|r| {
                let text = match &r.hint {
                    Some(h) => format!("deployability: {}: {} Fix: {h}", r.title, r.message),
                    None => format!("deployability: {}: {}", r.title, r.message),
                };
                let mut d = if r.status == Status::Fail {
                    crate::report::Diagnostic::error("deploy", text)
                } else {
                    crate::report::Diagnostic::warning("deploy", text)
                };
                d.code = Some(r.id.clone());
                d
            })
            .collect()
    }
}

// ------------------------------------------------------------------ the target

/// The plan2cell virtual cell's PROFINET segment (every launch's, `192.168.100.0/24`): the head
/// PLC, the PLC host's bridge as the KUKA VCs' gateway, robots from `.11`.
pub const VIRTUAL_CELL_HEAD_IP: &str = "192.168.100.10";
pub const VIRTUAL_CELL_NETMASK: &str = "255.255.255.0";
pub const VIRTUAL_CELL_GATEWAY: &str = "192.168.100.1";

/// The globals a controller declares, and where they came from.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Globals {
    /// e.g. `$config.dat of iiQKA.OS2_KSS_9.2.0_vass on ssm:i-0ae2…`, or `the bundled vw-ks24 list`.
    pub source: String,
    pub names: BTreeSet<String>,
    /// Read from the controller (else the bundled list, and why not).
    pub read: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// One KUKA virtual controller of the cell.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Controller {
    pub robot_ns: String,
    /// The address and station it answers on (its KLI, inside the cell's PROFINET segment).
    pub kli_ip: Option<String>,
    pub station: Option<String>,
    /// `iiQKA.OS2_KSS_9.2.0_vass on ssm:i-…`.
    pub host: String,
    /// The globals its `$config.dat` declares (`None`: not asked; the bundled list is used).
    pub globals: Option<Globals>,
}

/// The cell an export is checked against.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Target {
    /// "virtual cell", "cell.toml".
    pub name: String,
    pub head_ip: String,
    pub netmask: String,
    /// The head's gateway the controllers need (`0.0.0.0` / empty: none).
    pub gateway: String,
    pub controllers: Vec<Controller>,
}

impl Target {
    /// A plan2cell virtual cell with these controllers.
    pub fn virtual_cell(controllers: Vec<Controller>) -> Self {
        Self {
            name: "the virtual cell".into(),
            head_ip: VIRTUAL_CELL_HEAD_IP.into(),
            netmask: VIRTUAL_CELL_NETMASK.into(),
            gateway: VIRTUAL_CELL_GATEWAY.into(),
            controllers,
        }
    }

    pub fn controller(&self, ns: &str) -> Option<&Controller> {
        self.controllers.iter().find(|c| c.robot_ns == ns)
    }

    pub fn describe(&self) -> String {
        let list: Vec<String> = self
            .controllers
            .iter()
            .map(|c| {
                format!(
                    "{} {} {}",
                    c.robot_ns,
                    c.station.as_deref().unwrap_or("?"),
                    c.kli_ip.as_deref().unwrap_or("?")
                )
            })
            .collect();
        format!(
            "{} (head {}/{}, gateway {}; {} KUKA VC(s): {})",
            self.name,
            self.head_ip,
            self.netmask,
            if self.gateway.is_empty() { "none" } else { &self.gateway },
            self.controllers.len(),
            list.join(", ")
        )
    }
}

// ------------------------------------------------------------------ the export

/// Read access to an export: a directory, or a build in memory.
pub trait Files {
    fn read(&self, rel: &str) -> Option<String>;
}

impl Files for BTreeMap<String, String> {
    fn read(&self, rel: &str) -> Option<String> {
        self.get(rel).cloned()
    }
}

/// An export directory.
pub struct Dir(pub PathBuf);

impl Files for Dir {
    fn read(&self, rel: &str) -> Option<String> {
        if rel.contains("..") {
            return None;
        }
        std::fs::read_to_string(self.0.join(rel)).ok()
    }
}

/// One robot of the export, as its files say.
#[derive(Clone, Debug, Default)]
struct Robot {
    ns: String,
    /// The head project's PN device of it.
    device: Option<Value>,
    /// A `RobotSim_*` project for it.
    sim_project: Option<String>,
    /// Its docker service runs a CODESYS robot simulator (not `external`).
    sim_service: bool,
    external: bool,
    safety_bypass: Option<bool>,
}

struct Export<'a> {
    files: &'a dyn Files,
    summary: Option<Value>,
    project: Option<Value>,
    gvl: Option<String>,
    robots: Vec<Robot>,
    profile: String,
    plan_id: String,
}

fn json(files: &dyn Files, rel: &str) -> Option<Value> {
    serde_json::from_str(&files.read(rel)?).ok()
}

/// `NAME : TYPE := VALUE;` of a GVL.
fn gvl_value(gvl: &str, name: &str) -> Option<String> {
    gvl.lines().find_map(|l| {
        let l = l.split("(*").next().unwrap_or("").trim();
        let (n, rest) = l.split_once(':')?;
        if n.trim() != name {
            return None;
        }
        let v = rest.split_once(":=")?.1.trim().trim_end_matches(';').trim();
        Some(v.to_string())
    })
}

/// `[192,168,100,10]` / `'kuka-robi1'` → `192.168.100.10` / `kuka-robi1`.
fn param_text(v: &str) -> String {
    let v = v.trim();
    if let Some(inner) = v.strip_prefix('[').and_then(|x| x.strip_suffix(']')) {
        return inner.split(',').map(str::trim).collect::<Vec<_>>().join(".");
    }
    v.trim_matches(|c| c == '\'' || c == '"').to_string()
}

fn params(node: &Value) -> Vec<(String, String)> {
    node["parameters"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|p| {
            let key = p["member"].as_str().or(p["name"].as_str()).unwrap_or("").to_string();
            (key, param_text(p["value"].as_str().unwrap_or("")))
        })
        .collect()
}

fn param(node: &Value, key: &str) -> Option<String> {
    params(node).into_iter().find(|(k, _)| k == key).map(|(_, v)| v)
}

fn walk<'v>(n: &'v Value, out: &mut Vec<&'v Value>) {
    out.push(n);
    for c in n["children"].as_array().into_iter().flatten() {
        walk(c, out);
    }
}

fn nodes(project: &Value) -> Vec<&Value> {
    let mut out = Vec::new();
    walk(&project["device_tree"], &mut out);
    out
}

impl<'a> Export<'a> {
    fn load(files: &'a dyn Files) -> Self {
        let summary = json(files, "plan2cell-build.json");
        let project = json(files, "plc/project.json");
        let gvl = files.read("plc/codesys/head/GVL/GVL_Cell.st");
        let profile = summary.as_ref().and_then(|s| s["options"]["profile"].as_str()).unwrap_or("").to_string();
        let plan_id = summary.as_ref().and_then(|s| s["plan_id"].as_str()).unwrap_or("").to_string();
        let compose = files.read("plc/docker/docker-compose.robots.yml").unwrap_or_default();
        let external_services: BTreeSet<String> = compose
            .lines()
            .find_map(|l| l.strip_prefix("# plan2cell-external-services:"))
            .map(|l| l.split_whitespace().map(str::to_owned).collect())
            .unwrap_or_default();
        let bypass: Vec<bool> = gvl
            .as_deref()
            .and_then(|g| gvl_value(g, "abSafetyBypass"))
            .map(|v| v.trim_matches(|c| c == '[' || c == ']').split(',').map(|x| x.trim() == "TRUE").collect())
            .unwrap_or_default();
        let head = project.as_ref().and_then(|p| {
            p["projects"].as_array()?.iter().find(|x| x["role"] == "head_plc").cloned()
        });
        let mut robots = Vec::new();
        let list = summary.as_ref().and_then(|s| s["robots"].as_array().cloned()).unwrap_or_default();
        for (i, r) in list.iter().enumerate() {
            let ns = r["ns"].as_str().unwrap_or("").to_string();
            let tag = r["tag"].as_str().unwrap_or("").to_string();
            let device = head.as_ref().and_then(|h| nodes(h).into_iter().find(|n| n["robot"].as_str() == Some(&ns)).cloned());
            let sim_project = project.as_ref().and_then(|p| {
                p["projects"]
                    .as_array()?
                    .iter()
                    .find(|x| x["role"] == "robot_sim" && x["robot"].as_str() == Some(&ns))
                    .and_then(|x| x["name"].as_str().map(str::to_owned))
            });
            let service = if i == 0 { "device".to_string() } else { format!("device-{}", tag.to_lowercase()) };
            let external = external_services.contains(&service);
            let sim_service = !external && (compose.contains(&format!("  {service}:")) || (i == 0 && compose.contains("existing service 'device'") && !compose.contains("(external: not started)")));
            robots.push(Robot {
                ns,
                device,
                sim_project,
                sim_service,
                external,
                safety_bypass: bypass.get(i).copied(),
            });
        }
        Self { files, summary, project, gvl, robots, profile, plan_id }
    }

    fn head(&self) -> Option<&Value> {
        self.project.as_ref()?["projects"].as_array()?.iter().find(|x| x["role"] == "head_plc")
    }

    fn head_ethernet(&self) -> Option<&Value> {
        let head = self.head()?;
        nodes(head).into_iter().find(|n| param(n, "IPAddress").is_some())
    }

    /// The files of `krl/<ns>/` its manifest lists (path relative to it, text).
    fn krl_files(&self, ns: &str) -> Option<(Value, Vec<(String, String)>)> {
        let m = json(self.files, &format!("krl/{ns}/bt2krl-manifest.json"))?;
        let files = m["files"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|f| {
                let p = f["path"].as_str()?.to_string();
                let t = self.files.read(&format!("krl/{ns}/{p}"))?;
                Some((p, t))
            })
            .collect();
        Some((m, files))
    }
}

fn fnv1a64(b: &[u8]) -> String {
    let mut h: u64 = 0xcbf29ce484222325;
    for x in b {
        h ^= u64::from(*x);
        h = h.wrapping_mul(0x100000001b3);
    }
    format!("{h:016x}")
}

// ------------------------------------------------------------------ known generator bugs

/// Names generated code once used that a target does not declare, and the commit that fixed the
/// generator: a re-export gets the fixed code.
pub const FIXED_GENERATOR_BUGS: &[(&str, &str, &str)] = &[(
    "P_PktNr_SPS",
    "a2717d1",
    "bt2krl vw-ks24 wrote the op index into the VW package's P_PktNr_SPS, which is local to vw_usr_r (KSS error 2137 \
     \"name unknown\" in the krl stage); since a2717d1 it writes the global BT_PKTNR its bt_config.dat declares on the \
     same outputs ($OUT[97..112])",
)];

// ------------------------------------------------------------------ the rules

/// Run every rule: the cell-independent ones always, the cell's with `target`.
pub fn check(files: &dyn Files, export: &str, target: Option<&Target>) -> CheckReport {
    let started = std::time::Instant::now();
    let e = Export::load(files);
    let mut rules = vec![
        rule_files(&e),
        rule_plan_id(&e),
        rule_protocol(&e),
        rule_plc_safety(&e),
        rule_krl_names(&e, target),
    ];
    if let Some(t) = target {
        rules.push(rule_robots(&e, t));
        rules.push(rule_simulators(&e, t));
        rules.push(rule_safety(&e, t));
        rules.push(rule_network(&e, t));
        rules.push(rule_addresses(&e, t));
    }
    let mut r = CheckReport::new(export, rules);
    r.cell_id = e.summary.as_ref().and_then(|s| s["cell_id"].as_str()).unwrap_or("").to_string();
    r.plan_id = e.plan_id.clone();
    r.profile = e.profile.clone();
    r.target = target.map(Target::describe);
    r.elapsed_s = started.elapsed().as_secs_f64();
    r
}

/// The cell-independent rules on a build in memory (`plan2cell build`).
pub fn check_build(files: &BTreeMap<String, String>) -> CheckReport {
    check(files, "this export", None)
}

fn rule_files(e: &Export) -> RuleResult {
    let r = RuleResult::new("export.files", "Export complete", Scope::Export);
    let mut missing = Vec::new();
    if e.summary.is_none() {
        missing.push("plan2cell-build.json".to_string());
    }
    if e.project.is_none() {
        missing.push("plc/project.json (the CODESYS projects)".to_string());
    }
    if e.gvl.is_none() && e.project.is_some() {
        missing.push("plc/codesys/head/GVL/GVL_Cell.st".to_string());
    }
    if !missing.is_empty() {
        return r.with(
            Status::Fail,
            format!("missing {}", missing.join(", ")),
            Some(format!("Export PLC + robot code again with the PLC dialect codesys (step 2); {REEXPORT}")),
        );
    }
    let mut bad = Vec::new();
    let mut krl = 0;
    for robot in &e.robots {
        let Some((m, files)) = e.krl_files(&robot.ns) else { continue };
        krl += 1;
        for f in m["files"].as_array().into_iter().flatten() {
            let p = f["path"].as_str().unwrap_or("");
            match files.iter().find(|(x, _)| x == p) {
                None => bad.push(format!("krl/{}/{p} missing", robot.ns)),
                Some((_, t)) if f["fnv1a64"].as_str() != Some(fnv1a64(t.as_bytes()).as_str()) => {
                    bad.push(format!("krl/{}/{p} changed after the export (hash differs from its manifest)", robot.ns))
                }
                _ => {}
            }
        }
    }
    if !bad.is_empty() {
        let mut r = r.with(
            Status::Fail,
            format!("{} KRL file(s) do not match their bt2krl manifest: the krl stage refuses them", bad.len()),
            Some(format!("Export PLC + robot code again (step 2): an export is not edited by hand")),
        );
        r.details = bad;
        r.fixes = vec![Fix::Reexport { reason: "the KRL files differ from their manifests".into() }];
        return r;
    }
    r.pass(format!("plan2cell-build.json, plc/project.json and {krl} KRL bundle(s) match their manifests"))
}

fn rule_plan_id(e: &Export) -> RuleResult {
    let r = RuleResult::new("export.plan_id", "PLAN_ID", Scope::Export);
    if e.plan_id.is_empty() {
        return r.with(
            Status::Fail,
            "the export has no PLAN_ID (plan2cell-build.json)",
            Some(format!("Export PLC + robot code again (step 2)")),
        );
    }
    let want = plan_id_number(&e.plan_id);
    let mut seen = vec![("plan2cell-build.json".to_string(), e.plan_id.clone())];
    if let Some(p) = e.project.as_ref().and_then(|p| p["plan"]["plan_id"].as_str()) {
        seen.push(("plc/project.json".into(), p.to_string()));
    }
    if let Some(v) = e.gvl.as_deref().and_then(|g| gvl_value(g, "PLAN_ID")) {
        seen.push(("GVL_Cell.PLAN_ID (head PLC)".into(), v));
    }
    for robot in &e.robots {
        if let Some(m) = json(e.files, &format!("krl/{}/bt2krl-manifest.json", robot.ns)) {
            seen.push((format!("krl/{} manifest", robot.ns), m["plan_id"].as_str().unwrap_or("").to_string()));
        }
    }
    let differ: Vec<String> = seen
        .iter()
        .filter(|(_, v)| plan_id_number(v) != want)
        .map(|(w, v)| format!("{w}: {v}"))
        .collect();
    if !differ.is_empty() {
        let mut r = r.with(
            Status::Fail,
            format!("the parts of the export carry different PLAN_IDs (want {}): the head's PLAN_ID guard refuses them", e.plan_id),
            Some(format!("Export PLC + robot code again (step 2): parts of two exports were mixed")),
        );
        r.details = differ;
        r.fixes = vec![Fix::Reexport { reason: "the parts carry different PLAN_IDs".into() }];
        return r;
    }
    let byte = e.gvl.as_deref().and_then(|g| gvl_value(g, "PLAN_ID_BYTE")).and_then(|v| v.parse::<i64>().ok());
    if byte == Some(0) && !e.robots.is_empty() {
        return r.with(
            Status::Warn,
            format!(
                "PLAN_ID {} is in the head and every robot program, but the robots do not check it (profile {}: no PLAN_ID \
                 byte): a robot still running another plan's Folge is not refused",
                e.plan_id, e.profile
            ),
            Some("set [plc.protocol] plan_id_byte = 12 in the cell's plan2cell/plan2cell.toml (a free telegram byte), then Export again".into()),
        );
    }
    r.pass(format!("{} in {} part(s) of the export, checked by the head and the robots", e.plan_id, seen.len()))
}

fn rule_protocol(e: &Export) -> RuleResult {
    let r = RuleResult::new("export.protocol", "Job number and profile", Scope::Export);
    let Some(gvl) = e.gvl.as_deref() else { return r.skip("no head PLC code") };
    let head_byte = gvl_value(gvl, "JOB_NUMBER_BYTE").map(|v| v == "TRUE");
    let head_vw = gvl_value(gvl, "PROFILE_VW").map(|v| v == "TRUE");
    let mut bad = Vec::new();
    let want_vw = e.profile == "vw-ks24";
    if head_vw.is_some_and(|v| v != want_vw) {
        bad.push(format!("the head PLC's PROFILE_VW is {} but the export's profile is {}", head_vw.unwrap_or(false), e.profile));
    }
    if want_vw && head_byte == Some(false) {
        bad.push("profile vw-ks24 needs job number byte (PRG_NR is $IN[1..8]); the head sends a WORD".into());
    }
    for robot in &e.robots {
        if let Some(p) = json(e.files, &format!("plc/robots/{}/program.json", robot.ns)) {
            let job = p["protocol"]["job_number"].as_str().unwrap_or("");
            if let Some(b) = head_byte
                && !job.is_empty()
                && (job == "byte") != b
            {
                bad.push(format!(
                    "{}: robot program job_number {job}, the head sends {}",
                    robot.ns,
                    if b { "byte" } else { "word_le" }
                ));
            }
            let pp = p["protocol"]["profile"].as_str().unwrap_or("");
            if !pp.is_empty() && !e.profile.is_empty() && pp != e.profile {
                bad.push(format!("{}: robot program profile {pp}, the export's {}", robot.ns, e.profile));
            }
        }
        if let Some(m) = json(e.files, &format!("krl/{}/bt2krl-manifest.json", robot.ns)) {
            let kp = m["profile"].as_str().unwrap_or("");
            if !kp.is_empty() && !e.profile.is_empty() && kp != e.profile {
                bad.push(format!("{}: KRL for profile {kp}, the head PLC for {}", robot.ns, e.profile));
            }
        }
    }
    if !bad.is_empty() {
        let mut r = r.with(
            Status::Fail,
            format!("the head PLC and the robots would not understand each other: {}", bad[0]),
            Some(format!("Export PLC + robot code again with one profile (step 2: Profile)")),
        );
        r.details = bad;
        r.fixes = vec![Fix::Reexport { reason: "head and robots disagree on the telegram".into() }];
        return r;
    }
    r.pass(format!(
        "profile {} with job number {} in the head, the robot programs and the KRL",
        if e.profile.is_empty() { "?" } else { &e.profile },
        match head_byte {
            Some(true) => "byte",
            Some(false) => "WORD LE",
            None => "?",
        }
    ))
}

/// A robot device's slot 1 holds the PROFIsafe module (FDIO64).
fn safety_module(device: &Value) -> Option<bool> {
    let slot = device["slots"].as_array()?.iter().find(|s| s["slot"] == 1)?;
    Some(!slot["empty"].as_bool().unwrap_or(false) && slot["profisafe"].as_bool().unwrap_or(false))
}

fn rule_plc_safety(e: &Export) -> RuleResult {
    let r = RuleResult::new("plc.safety", "Safety settings consistent", Scope::Export);
    let mut bad = Vec::new();
    for robot in &e.robots {
        let Some(device) = &robot.device else { continue };
        if safety_module(device) == Some(false) && !(robot.safety_bypass == Some(true) && robot.external) {
            bad.push(robot.ns.clone());
        }
    }
    if !bad.is_empty() {
        let mut r = r.with(
            Status::Fail,
            format!(
                "{}: no safety module, but not External with Safety bypass: the head waits for PROFIsafe bits nobody sends",
                bad.join(", ")
            ),
            Some(format!("{HW}: {} → External on, Safety bypass on; {REEXPORT}", bad.join(", "))),
        );
        for ns in &bad {
            r.fixes.push(Fix::Hardware { field: format!("robots.{ns}.external"), value: Value::Bool(true) });
            r.fixes.push(Fix::Hardware { field: format!("robots.{ns}.safetyBypass"), value: Value::Bool(true) });
        }
        r.robots = bad;
        return r;
    }
    r.pass("every robot without a safety module is External with Safety bypass")
}

fn rule_krl_names(e: &Export, target: Option<&Target>) -> RuleResult {
    let scope = if target.is_some() { Scope::Cell } else { Scope::Export };
    let r = RuleResult::new("krl.names", "KRL names known to the controllers", scope);
    let bundled = bt2krl::names::bundled_globals(&e.profile);
    let mut findings = Vec::new();
    let mut sources = Vec::new();
    let mut checked = 0;
    let mut robots = Vec::new();
    for robot in &e.robots {
        let Some((m, files)) = e.krl_files(&robot.ns) else { continue };
        let profile = m["profile"].as_str().unwrap_or(&e.profile).to_string();
        let globals = target.and_then(|t| t.controller(&robot.ns)).and_then(|c| c.globals.as_ref()).filter(|g| g.read);
        let names = match globals {
            Some(g) => {
                sources.push(format!("{}: {}", robot.ns, g.source));
                g.names.clone()
            }
            None => {
                let why = target
                    .and_then(|t| t.controller(&robot.ns))
                    .and_then(|c| c.globals.as_ref())
                    .and_then(|g| g.error.clone())
                    .map(|e| format!(" ({e})"))
                    .unwrap_or_default();
                sources.push(format!("{}: the bundled {profile} list{why}", robot.ns));
                if profile == e.profile { bundled.clone() } else { bt2krl::names::bundled_globals(&profile) }
            }
        };
        checked += files.iter().filter(|(p, _)| p.ends_with(".src") && p.starts_with("KRC/")).count();
        let unknown = bt2krl::names::unknown_names(files.iter().map(|(p, t)| (p.as_str(), t.as_str())), &names);
        if !unknown.is_empty() {
            robots.push(robot.ns.clone());
        }
        for u in unknown {
            findings.push((robot.ns.clone(), u));
        }
    }
    if checked == 0 {
        return r.skip("no KRL in the export");
    }
    if findings.is_empty() {
        return r.pass(format!("every name of {checked} KRL module(s) is declared ({})", sources.join("; ")));
    }
    let names: BTreeSet<&str> = findings.iter().map(|(_, u)| u.name.as_str()).collect();
    let first = &findings[0];
    let mut r = r.with(
        Status::Fail,
        format!(
            "{} uses {} which the controller does not declare ({}:{}: `{}`): KSS error 2137 \"name unknown\" in the krl stage, \
             after the CODESYS projects and the deploy (~10 min)",
            first.0,
            names.iter().copied().collect::<Vec<_>>().join(", "),
            first.1.file,
            first.1.line,
            first.1.text
        ),
        None,
    );
    r.robots = robots;
    r.details = findings
        .iter()
        .map(|(ns, u)| format!("krl/{ns}/{}:{}: {} ({})", u.file, u.line, u.name, u.text))
        .chain(sources.iter().map(|s| format!("globals: {s}")))
        .collect();
    let fixed: Vec<&(&str, &str, &str)> = FIXED_GENERATOR_BUGS.iter().filter(|(n, _, _)| names.iter().any(|x| x.eq_ignore_ascii_case(n))).collect();
    if !fixed.is_empty() && fixed.len() == names.len() {
        let bug = fixed.iter().map(|(_, c, what)| format!("{what} (fixed in {c})")).collect::<Vec<_>>().join("; ");
        r.generator_bug = Some(format!("generator bug fixed in {}; re-export", fixed.iter().map(|(_, c, _)| *c).collect::<Vec<_>>().join(", ")));
        r.hint = Some(format!("not a setting: a generator bug. {bug}. Export PLC + robot code again (step 2) to get the fixed KRL"));
        r.fixes = vec![Fix::Reexport { reason: format!("generator bug fixed in {}", fixed[0].1) }];
    } else {
        r.generator_bug = Some("bt2krl writes names this controller does not declare (not fixed yet)".into());
        r.hint = Some(
            "not a setting: the generated KRL is wrong for this controller (a bt2krl bug, or a controller without the \
             package the profile needs, e.g. vw-ks24 on a stock KSS: pick the profile the controller runs). Report it with \
             the export"
                .into(),
        );
    }
    r
}

fn rule_robots(e: &Export, t: &Target) -> RuleResult {
    let r = RuleResult::new("cell.robots", "Robots match the cell's controllers", Scope::Cell);
    let mut bad = Vec::new();
    let mut warn = Vec::new();
    for robot in &e.robots {
        match t.controller(&robot.ns) {
            None => bad.push(format!("{} has no KUKA VC in {} (its controllers: {})", robot.ns, t.name, t.controllers.iter().map(|c| c.robot_ns.as_str()).collect::<Vec<_>>().join(", "))),
            Some(_) if e.files.read(&format!("krl/{}/bt2krl-manifest.json", robot.ns)).is_none() => {
                bad.push(format!("{} runs on a KUKA VC, but the export has no KRL for it (krl/{}/)", robot.ns, robot.ns))
            }
            Some(_) => {}
        }
    }
    for c in &t.controllers {
        if !e.robots.iter().any(|r| r.ns == c.robot_ns) {
            warn.push(format!("{}'s controller is not used by this plan (it keeps the program it has)", c.robot_ns));
        }
    }
    if !bad.is_empty() {
        let mut r = r.with(
            Status::Fail,
            bad[0].clone(),
            Some(format!(
                "plan for the cell's robots ({}; step 1), or Export with Robot language KRL (step 2)",
                t.controllers.iter().map(|c| c.robot_ns.as_str()).collect::<Vec<_>>().join(", ")
            )),
        );
        r.details = bad;
        return r;
    }
    if !warn.is_empty() {
        let mut r = r.with(Status::Warn, warn.join("; "), None);
        r.details = warn;
        return r;
    }
    r.pass(format!("{} robot(s), each with a KUKA VC and its KRL", e.robots.len()))
}

fn rule_simulators(e: &Export, t: &Target) -> RuleResult {
    let r = RuleResult::new("cell.simulators", "No robot simulators on the KUKA VCs' addresses", Scope::Cell);
    let mut bad = Vec::new();
    let mut robots = Vec::new();
    for robot in e.robots.iter().filter(|r| t.controller(&r.ns).is_some()) {
        let mut what = Vec::new();
        if let Some(p) = &robot.sim_project {
            what.push(format!("the CODESYS project {p}"));
        }
        if robot.sim_service {
            what.push("a CODESYS robot simulator container".to_string());
        }
        if !robot.external && what.is_empty() {
            what.push("External off".to_string());
        }
        if !what.is_empty() {
            bad.push(format!("{}: {}", robot.ns, what.join(" and ")));
            robots.push(robot.ns.clone());
        }
    }
    if !bad.is_empty() {
        let mut r = r.with(
            Status::Fail,
            format!(
                "{}: the robot is a KUKA VC of the cell, but the export simulates it with CODESYS (deploying a RobotSim \
                 project to the controller's address fails, \"No gateway is configured\"; its container takes the VC's IP)",
                bad.join("; ")
            ),
            Some(format!("{HW}: {} → External on; {REEXPORT}", robots.join(", "))),
        );
        for ns in &robots {
            r.fixes.push(Fix::Hardware { field: format!("robots.{ns}.external"), value: Value::Bool(true) });
        }
        r.robots = robots;
        return r;
    }
    r.pass("every robot on a KUKA VC is External (no RobotSim project, no simulator container)")
}

fn rule_safety(e: &Export, t: &Target) -> RuleResult {
    let r = RuleResult::new("cell.safety", "Safety settings the KUKA VCs can satisfy", Scope::Cell);
    let mut found = Vec::new();
    let mut fixes = Vec::new();
    let mut robots = Vec::new();
    let mut gui = Vec::new();
    for robot in e.robots.iter().filter(|r| t.controller(&r.ns).is_some()) {
        let mut what = Vec::new();
        let mut set = Vec::new();
        if robot.device.as_ref().and_then(safety_module) == Some(true) {
            what.push("a safety module (FDIO64 in slot 1; the VCs' DAP has no safety slot)");
            set.push("Safety module off");
            fixes.push(Fix::Hardware { field: format!("robots.{}.safetyModule", robot.ns), value: Value::Bool(false) });
        }
        if robot.safety_bypass == Some(false) {
            what.push("Safety bypass off (the head waits for PROFIsafe bits the VCs never send)");
            set.push("Safety bypass on");
            fixes.push(Fix::Hardware { field: format!("robots.{}.safetyBypass", robot.ns), value: Value::Bool(true) });
        }
        if !robot.external {
            what.push("External off");
            set.push("External on");
            fixes.push(Fix::Hardware { field: format!("robots.{}.external", robot.ns), value: Value::Bool(true) });
        }
        if !what.is_empty() {
            found.push(format!("{}: {}", robot.ns, what.join(", ")));
            gui.push(format!("{} → {}", robot.ns, set.join(", ")));
            robots.push(robot.ns.clone());
        }
    }
    if found.is_empty() {
        return r.pass("no safety module, Safety bypass and External on every KUKA VC robot (simulation only)");
    }
    let mut r = r.with(
        Status::Fail,
        format!("{}: Start production will fail with ERR_SAFETY_TIMEOUT", found.join("; ")),
        Some(format!("{HW}: {} (or \"Use the virtual cell's values\"); {REEXPORT}", gui.join("; "))),
    );
    r.fixes = fixes;
    r.robots = robots;
    r
}

fn same_ip(a: &str, b: &str) -> bool {
    match (a.trim().parse::<Ipv4Addr>(), b.trim().parse::<Ipv4Addr>()) {
        (Ok(x), Ok(y)) => x == y,
        _ => a.trim() == b.trim(),
    }
}

fn no_gateway(g: &str) -> bool {
    g.trim().is_empty() || g.trim() == "0.0.0.0"
}

fn rule_network(e: &Export, t: &Target) -> RuleResult {
    let r = RuleResult::new("cell.network", "Head PLC network", Scope::Cell);
    let Some(eth) = e.head_ethernet() else { return r.skip("no head PLC Ethernet device in plc/project.json") };
    let ip = param(eth, "IPAddress").unwrap_or_default();
    let mask = param(eth, "SubnetMask").unwrap_or_default();
    let gw = param(eth, "GatewayAddress").unwrap_or_default();
    let mut bad = Vec::new();
    let mut fixes = Vec::new();
    let mut gui = Vec::new();
    if !t.head_ip.is_empty() && !same_ip(&ip, &t.head_ip) {
        bad.push(format!("head IP {ip}, the cell's head PLC is {}", t.head_ip));
        fixes.push(Fix::Hardware { field: "network.headIp".into(), value: Value::String(t.head_ip.clone()) });
        gui.push(format!("Head IP {}", t.head_ip));
    }
    if !t.netmask.is_empty() && !same_ip(&mask, &t.netmask) {
        bad.push(format!("netmask {mask}, the cell's segment {}", t.netmask));
        fixes.push(Fix::Hardware { field: "network.netmask".into(), value: Value::String(t.netmask.clone()) });
        gui.push(format!("Netmask {}", t.netmask));
    }
    let want_none = no_gateway(&t.gateway);
    if (want_none && !no_gateway(&gw)) || (!want_none && !same_ip(&gw, &t.gateway)) {
        bad.push(format!(
            "gateway {}, the KUKA VCs' is {}: with another the head sends DCP Set IP to the controllers (re-addresses them) \
             and their PROFINET ARs do not come up",
            if no_gateway(&gw) { "none (0.0.0.0)".to_string() } else { gw.clone() },
            if want_none { "none".to_string() } else { t.gateway.clone() }
        ));
        fixes.push(Fix::Hardware {
            field: "network.gateway".into(),
            value: Value::String(if want_none { "0.0.0.0".into() } else { t.gateway.clone() }),
        });
        gui.push(format!("Gateway {}", if want_none { "0.0.0.0" } else { &t.gateway }));
    }
    if bad.is_empty() {
        return r.pass(format!("head {ip}/{mask}, gateway {gw}: as {} is wired", t.name));
    }
    let mut r = r.with(
        Status::Fail,
        bad.join("; "),
        Some(format!("{HW}: Network → {}; {REEXPORT}", gui.join(", "))),
    );
    if bad.len() > 1 {
        r.details = bad;
    }
    r.fixes = fixes;
    r
}

fn rule_addresses(e: &Export, t: &Target) -> RuleResult {
    let r = RuleResult::new("cell.addresses", "Robot stations and addresses", Scope::Cell);
    let mut bad = Vec::new();
    let mut fixes = Vec::new();
    let mut gui = Vec::new();
    let mut robots = Vec::new();
    let mut checked = 0;
    for robot in &e.robots {
        let (Some(c), Some(device)) = (t.controller(&robot.ns), robot.device.as_ref()) else { continue };
        checked += 1;
        let station = param(device, "Identification.StationName").unwrap_or_default();
        let ip = param(device, "Identification.IPAddress").unwrap_or_default();
        let mut set = Vec::new();
        if let Some(want) = c.kli_ip.as_deref().filter(|w| !same_ip(&ip, w)) {
            bad.push(format!("{}: the head expects it at {ip}, its KUKA VC answers at {want}", robot.ns));
            fixes.push(Fix::Hardware { field: format!("robots.{}.ip", robot.ns), value: Value::String(want.to_owned()) });
            set.push(format!("IP {want}"));
        }
        if let Some(want) = c.station.as_deref().filter(|w| !station.eq_ignore_ascii_case(w)) {
            bad.push(format!("{}: the head expects station {station}, its KUKA VC is {want}", robot.ns));
            fixes.push(Fix::Hardware { field: format!("robots.{}.station", robot.ns), value: Value::String(want.to_owned()) });
            set.push(format!("Station {want}"));
        }
        if !set.is_empty() {
            gui.push(format!("{} → {}", robot.ns, set.join(", ")));
            robots.push(robot.ns.clone());
        }
    }
    if checked == 0 {
        return r.skip("no robot device of the head on a KUKA VC");
    }
    if bad.is_empty() {
        return r.pass(format!(
            "{}",
            e.robots
                .iter()
                .filter_map(|r| {
                    let c = t.controller(&r.ns)?;
                    Some(format!("{} {} {}", r.ns, c.station.as_deref().unwrap_or("?"), c.kli_ip.as_deref().unwrap_or("?")))
                })
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    let mut r = r.with(
        Status::Fail,
        format!("{}: the head's PROFINET devices would not find the controllers", bad.join("; ")),
        Some(format!("{HW}: {}; {REEXPORT}", gui.join("; "))),
    );
    r.details = bad;
    r.fixes = fixes;
    r.robots = robots;
    r
}

/// The command that prints the declarations of a controller instance's `$config.dat` (read-only,
/// on its host, as root): without plan2cell's own `;BT2KRL` block (the export brings its own),
/// comments and values, sorted, gzipped and base64 (one line; SSM returns at most 24000
/// characters), then `p2c-ok`.
pub fn config_dat_command(instance: &str) -> String {
    let file = format!("/opt/kuka/instances/{instance}/ROBOTER/KRC/R1/System/$config.dat");
    format!(
        "F='{}'; test -r \"$F\" || {{ echo \"p2c-missing $F\"; exit 0; }}; \
         sed -e '/^[[:space:]]*;BT2KRL BEGIN/,/^[[:space:]]*;BT2KRL END/d' \"$F\" | tr -d '\\r' | sed -e 's/;.*//' -e 's/=.*//' \
         -e 's/\\[.*//' | tr -s ' \\t' '  ' | sed -e 's/^ //' -e 's/ $//' | grep -v '^$' | sort -u | gzip -9 | base64 -w0; \
         echo; echo p2c-ok",
        file.replace('\'', "")
    )
}

/// The globals of [`config_dat_command`]'s output (`Err`: why not).
pub fn parse_config_dat_output(out: &str, gunzip: impl Fn(&[u8]) -> Option<String>) -> Result<BTreeSet<String>, String> {
    if let Some(l) = out.lines().find(|l| l.starts_with("p2c-missing")) {
        return Err(format!("{} not found", l.trim_start_matches("p2c-missing ").trim()));
    }
    if !out.lines().any(|l| l.trim() == "p2c-ok") {
        return Err("no complete answer".into());
    }
    let b64 = out.lines().find(|l| !l.trim().is_empty() && l.trim() != "p2c-ok").unwrap_or("").trim();
    let gz = base64_decode(b64).ok_or("the answer is not base64")?;
    let text = gunzip(&gz).ok_or("the answer is not gzip")?;
    let names = bt2krl::names::declared(&text);
    if names.is_empty() {
        return Err("$config.dat declares nothing".into());
    }
    Ok(names)
}

/// The globals of a robot hook's `globals` answer (`factoryos/hook@1`, handoff §2.2: `{source,
/// names, read, text, error}`; `exit` the hook's, `problem` why its answer did not count). Its
/// `names` replace the `$config.dat` read; without names, `text` (the `config_dat_command` lines)
/// is read like that command's output; an empty set, `read: false` or a failed answer falls back to
/// the profile's bundled list (with why).
pub fn hook_globals(fallback_source: &str, answer: &Value, exit: i32, problem: Option<&str>) -> Globals {
    let source = answer["source"]
        .as_str()
        .filter(|s| !s.is_empty())
        .map(|s| format!("{s} (hook globals)"))
        .unwrap_or_else(|| fallback_source.to_string());
    let fail = |error: String| Globals { source: source.clone(), names: BTreeSet::new(), read: false, error: Some(error) };
    if let Some(p) = problem {
        return fail(format!("hook globals: {p}"));
    }
    if exit != 0 || answer["read"] != Value::Bool(true) {
        let why = answer["error"].as_str().unwrap_or("not read");
        return fail(format!("hook globals: exit {exit}: {why}"));
    }
    let mut names: BTreeSet<String> = answer["names"]
        .as_array()
        .map(|a| a.iter().filter_map(|x| x.as_str()).map(str::to_string).filter(|n| !n.is_empty()).collect())
        .unwrap_or_default();
    if names.is_empty()
        && let Some(t) = answer["text"].as_str()
    {
        names = bt2krl::names::declared(t);
    }
    if names.is_empty() {
        return fail("the hook's $config.dat declares nothing".into());
    }
    Globals { source, names, read: true, error: None }
}

fn base64_decode(s: &str) -> Option<Vec<u8>> {
    fn val(c: u8) -> Option<u32> {
        Some(match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' => 62,
            b'/' => 63,
            _ => return None,
        } as u32)
    }
    let s: Vec<u8> = s.bytes().filter(|c| !c.is_ascii_whitespace()).collect();
    let mut out = Vec::with_capacity(s.len() * 3 / 4);
    for chunk in s.chunks(4) {
        let mut n = 0u32;
        let mut k = 0usize;
        for &c in chunk {
            if c == b'=' {
                break;
            }
            n = n << 6 | val(c)?;
            k += 1;
        }
        n <<= 6 * (4 - k);
        let bytes = [(n >> 16) as u8, (n >> 8) as u8, n as u8];
        out.extend_from_slice(&bytes[..k.saturating_sub(1)]);
    }
    Some(out)
}

/// The export directory's report file (written by `plan2cell cell check`).
pub const REPORT_FILE: &str = "plan2cell-deploy-check.json";

/// The number of a PLAN_ID as the parts of an export write it: `16#A1BE`, `WORD#16#A1BE`, `41406`.
pub fn plan_id_number(s: &str) -> Option<u32> {
    let s = s.trim().trim_matches('\'');
    let s = s.rsplit_once(':').map_or(s, |(_, v)| v).trim();
    let s = match s.split_once('#') {
        Some((ty, rest)) if !ty.is_empty() && ty.chars().all(|c| c.is_ascii_alphabetic()) => rest,
        _ => s,
    };
    match s.split_once('#') {
        Some((base, digits)) => u32::from_str_radix(&digits.replace('_', ""), base.parse().ok()?).ok(),
        None => s.parse().ok(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base64_round_trips() {
        assert_eq!(base64_decode("aGVsbG8=").unwrap(), b"hello");
        assert_eq!(base64_decode("aGVsbG8gd29ybGQ").unwrap(), b"hello world");
        assert!(base64_decode("a$b").is_none());
        assert_eq!(plan_id_number("16#A1BE"), Some(0xA1BE));
        assert_eq!(plan_id_number("WORD#16#A1BE"), Some(0xA1BE));
        assert_eq!(plan_id_number("41406"), Some(41406));
    }

    #[test]
    fn gvl_values_and_params() {
        let g = "    PLAN_ID                 : WORD := WORD#16#A1BE;             (* fold *)\n    abSafetyBypass : ARRAY[1..2] OF BOOL := [FALSE, TRUE];(* x *)\n";
        assert_eq!(gvl_value(g, "PLAN_ID").as_deref(), Some("WORD#16#A1BE"));
        assert_eq!(gvl_value(g, "abSafetyBypass").as_deref(), Some("[FALSE, TRUE]"));
        assert_eq!(param_text("[192,168,100,10]"), "192.168.100.10");
        assert_eq!(param_text("'kuka-robi1'"), "kuka-robi1");
    }

    #[test]
    fn the_config_dat_command_is_read_only() {
        let c = config_dat_command("iiQKA.OS2_KSS_9.2.0_vass");
        assert!(c.contains("/opt/kuka/instances/iiQKA.OS2_KSS_9.2.0_vass/ROBOTER/KRC/R1/System/$config.dat"));
        for w in [" > ", "rm ", "mv ", "cp ", "tee", "sed -i"] {
            assert!(!c.contains(w), "{w}: {c}");
        }
        assert!(parse_config_dat_output("p2c-missing /x\n", |_| None).unwrap_err().contains("/x"));
        assert!(parse_config_dat_output("abc\n", |_| None).is_err());
    }

    #[test]
    fn hook_globals_replace_the_config_dat_read() {
        let a = serde_json::json!({"source": "$config.dat of krl-sim 1 robot1 (built-in)", "names": ["$IN", "BT_X"], "read": true, "text": ""});
        let g = hook_globals("hook on kubectl:x", &a, 0, None);
        assert!(g.read && g.names.contains("BT_X") && g.source.ends_with("(hook globals)"), "{g:?}");
        // The stub declares nothing: the bundled list is used, and why.
        let stub = serde_json::json!({"source": "hook-stub", "names": [], "read": true, "text": ""});
        let g = hook_globals("hook on kubectl:x", &stub, 0, None);
        assert!(!g.read && g.error.as_deref().unwrap().contains("declares nothing"), "{g:?}");
        let refused = serde_json::json!({"read": false, "error": "nothing installed"});
        let g = hook_globals("hook on kubectl:x", &refused, 3, None);
        assert_eq!((g.read, g.source.as_str()), (false, "hook on kubectl:x"));
        assert!(g.error.unwrap().contains("exit 3: nothing installed"));
        assert!(!hook_globals("s", &Value::Null, 4, Some("hook answer missing or inconsistent")).read);
    }
}
