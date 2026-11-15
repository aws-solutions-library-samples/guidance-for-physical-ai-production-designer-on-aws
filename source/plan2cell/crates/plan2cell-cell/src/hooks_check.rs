//! `plan2cell cell hooks-check`: the conformance kit of `factoryos/hook@1`
//! (docs/tasks/cell-templates.md E11). Image authors run it against a live pod (`--exec`, over
//! [`crate::remote::Kubectl`]) or a local image (`--docker`, a throwaway container on an
//! `--internal` network, [`DockerRun`]); every probe uses only `sh`, `cat`, `mv` and the hook.
//!
//! ```text
//! HC01  bash, sh, cat, mv exist
//! HC02  describe exits 0 with a valid envelope and answer (--docker: also without a workload)
//! HC03  every verb of the role answers an allowed exit with its envelope; another role's verb is
//!       2 "unimplemented: ..."; describe.ops contains the template's ops (--ops)
//! HC04  verb bogus, stdin `{` and `{"zzz":1}` -> 2
//! HC05  note: prefixes, one JSON object last, stdout <= 1 MiB, ok == (exit == 0)
//! HC06  /proc/1/status: CapEff = CapPrm = CapBnd of the profile, CapAmb 0, NoNewPrivs 1, Seccomp 2, Uid
//! HC07  /opt not writable, /run and the inbox writable
//! HC08  listeners (/proc/net/{tcp,tcp6,udp,udp6}): TCP LISTEN on loopback only; UDP on loopback
//!       or the PN address with port 34964 or >= 32768 (--strict-udp: loopback only)
//! HC09  a KUBECTL_COPY into the inbox works; a request naming a .p2c-tmp file -> 3
//! HC10  health: runtime is a valid plan2cell/cell-runtime@1
//! HC11  every thread SCHED_OTHER, VmLck 0 kB
//! HC12  --docker: ready without a PROFINET controller within 60 s (robot: info.ar "none")
//! ```
//!
//! `--exec` runs only what changes nothing (describe, ready, health, read, globals, op status,
//! watch for 1 s, the refusals of HC04) unless `mutating`; `--docker` runs every verb. The JSON
//! Schemas image authors validate against are `schema/hook@1/*.schema.json` of this crate; the
//! checks here follow them field by field (`*_REQUIRED`, pinned by the tests).

use std::collections::BTreeSet;
use std::net::{Ipv4Addr, Ipv6Addr};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::hook::{self, Answer, CONTRACT, HOOK, INBOX, Role, Verb, WATCH_SCHEMA};
use crate::remote::{Exec, ExecLimits, Remote};
use crate::{Error, Result};

pub const REPORT_SCHEMA: &str = "plan2cell/hooks-check@1";
/// The JSON Schemas of the contract, published with plan2cell (E0).
pub const SCHEMAS: [(&str, &str); 8] = [
    ("envelope", include_str!("../schema/hook@1/envelope.schema.json")),
    ("describe", include_str!("../schema/hook@1/describe.schema.json")),
    ("ready", include_str!("../schema/hook@1/ready.schema.json")),
    ("health", include_str!("../schema/hook@1/health.schema.json")),
    ("watch-robot", include_str!("../schema/hook@1/watch-robot.schema.json")),
    ("watch-head", include_str!("../schema/hook@1/watch-head.schema.json")),
    ("head-bundle", include_str!("../schema/hook@1/head-bundle.schema.json")),
    ("wiring", include_str!("../schema/hook@1/wiring.schema.json")),
];
pub const ENVELOPE_REQUIRED: [&str; 4] = ["schema", "verb", "ok", "exit"];
pub const DESCRIBE_REQUIRED: [&str; 5] = ["contract", "role", "runtime", "verbs", "versions"];
pub const DESCRIBE_ROBOT_REQUIRED: [&str; 3] = ["ops", "bundle_schemas", "inbox"];
pub const READY_REQUIRED: [&str; 2] = ["ready", "checks"];
pub const HEALTH_REQUIRED: [&str; 1] = ["runtime"];
pub const WATCH_ROBOT_REQUIRED: [&str; 6] = ["schema", "kind", "t_ms", "component", "state", "io"];
pub const WATCH_HEAD_REQUIRED: [&str; 5] = ["schema", "kind", "t_ms", "component", "values"];

/// The PROFINET port every PN device listens on (UDP), and the lowest RPC client port.
pub const PN_PORT: u16 = 34964;
pub const PN_CLIENT_MIN: u16 = 32768;

/// The docker network of `--docker` and the container's PN addresses on it.
pub const DOCKER_SUBNET: &str = "10.77.250.0/24";
pub fn docker_ip(role: Role) -> &'static str {
    match role {
        Role::Head => "10.77.250.10",
        Role::Robot => "10.77.250.11",
    }
}

/// The 13 default capabilities pn-raw drops (E2.2; containerd keeps the 14th, NET_RAW).
pub const PN_RAW_DROP: [&str; 13] = [
    "AUDIT_WRITE",
    "CHOWN",
    "DAC_OVERRIDE",
    "FOWNER",
    "FSETID",
    "KILL",
    "MKNOD",
    "NET_BIND_SERVICE",
    "SETFCAP",
    "SETGID",
    "SETPCAP",
    "SETUID",
    "SYS_CHROOT",
];

/// The security profiles of `pod@1` (E2.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Profile {
    None,
    PnRaw,
    PnAdmin,
}

impl Profile {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "none" => Some(Profile::None),
            "pn-raw" => Some(Profile::PnRaw),
            "pn-admin" => Some(Profile::PnAdmin),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Profile::None => "none",
            Profile::PnRaw => "pn-raw",
            Profile::PnAdmin => "pn-admin",
        }
    }

    /// CapEff = CapPrm = CapBnd in the container: none 0, NET_RAW (bit 13), plus NET_ADMIN (bit 12).
    pub fn caps(self) -> u64 {
        match self {
            Profile::None => 0,
            Profile::PnRaw => 0x2000,
            Profile::PnAdmin => 0x3000,
        }
    }

    pub fn uid(self) -> u32 {
        match self {
            Profile::None => 65532,
            Profile::PnRaw | Profile::PnAdmin => 0,
        }
    }

    /// The profile as `docker run` flags (E11).
    pub fn docker_flags(self) -> Vec<String> {
        let mut v = Vec::new();
        match self {
            Profile::None => v.extend(["--user", "65532:65532", "--cap-drop", "ALL"].map(String::from)),
            Profile::PnRaw | Profile::PnAdmin => {
                v.extend(["--user", "0:0"].map(String::from));
                for c in PN_RAW_DROP {
                    v.extend(["--cap-drop".to_string(), c.to_string()]);
                }
                if self == Profile::PnAdmin {
                    v.extend(["--cap-add", "NET_ADMIN"].map(String::from));
                }
            }
        }
        v
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Mode {
    Exec,
    Docker,
}

/// What `hooks-check` is told.
#[derive(Clone, Debug)]
pub struct Options {
    pub mode: Mode,
    /// Else `describe`'s.
    pub role: Option<Role>,
    /// Else unknown: HC06 is skipped.
    pub profile: Option<Profile>,
    /// The pod's PN addresses (no prefix length; a head has `.10` and `.1`).
    pub pn_addresses: Vec<String>,
    pub strict_udp: bool,
    /// The template's `consumer.ops` names (`--ops`).
    pub ops: Vec<String>,
    /// `--exec`: the changing verbs too (a throwaway cell). `--docker`: always.
    pub mutating: bool,
    /// HC12: how long `ready` may take, and the pause between tries.
    pub ready_wait: Duration,
    pub ready_poll: Duration,
}

impl Options {
    pub fn new(mode: Mode) -> Self {
        Self {
            mode,
            role: None,
            profile: None,
            pn_addresses: Vec::new(),
            strict_udp: false,
            ops: Vec::new(),
            mutating: mode == Mode::Docker,
            ready_wait: Duration::from_secs(60),
            ready_poll: Duration::from_secs(2),
        }
    }

    fn changes(&self) -> bool {
        self.mutating || self.mode == Mode::Docker
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CheckResult {
    pub id: String,
    pub ok: bool,
    pub detail: String,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub skipped: bool,
}

/// `plan2cell/hooks-check@1`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HooksReport {
    pub schema: String,
    pub target: String,
    pub mode: Mode,
    pub role: Option<String>,
    pub profile: Option<String>,
    pub checks: Vec<CheckResult>,
    pub ok: bool,
    /// The target did not answer at all (exit 4).
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub unreachable: bool,
}

impl HooksReport {
    /// 0 every check ok, 7 a check failed, 4 the target did not answer.
    pub fn exit_code(&self) -> i32 {
        if self.unreachable {
            4
        } else if self.ok {
            0
        } else {
            7
        }
    }

    pub fn text(&self) -> String {
        let mut s = format!(
            "hooks-check {} ({}, role {}, profile {})\n",
            self.target,
            match self.mode {
                Mode::Exec => "exec",
                Mode::Docker => "docker",
            },
            self.role.as_deref().unwrap_or("?"),
            self.profile.as_deref().unwrap_or("?")
        );
        for c in &self.checks {
            let mark = if c.skipped { "skip" } else if c.ok { "ok" } else { "FAIL" };
            s.push_str(&format!("{} {mark:<4} {}\n", c.id, c.detail));
        }
        s.push_str(if self.ok { "conformant\n" } else { "NOT conformant\n" });
        s
    }
}

// ------------------------------------------------------------------ parsers (canned /proc files)

/// The fields of `/proc/<pid>/status` HC06 reads.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ProcStatus {
    pub cap_eff: Option<u64>,
    pub cap_prm: Option<u64>,
    pub cap_bnd: Option<u64>,
    pub cap_amb: Option<u64>,
    pub no_new_privs: Option<u32>,
    pub seccomp: Option<u32>,
    pub uid: Option<u32>,
}

pub fn parse_status(text: &str) -> ProcStatus {
    let mut s = ProcStatus::default();
    for l in text.lines() {
        let Some((k, v)) = l.split_once(':') else { continue };
        let v = v.trim();
        let hex = || u64::from_str_radix(v, 16).ok();
        let first = || v.split_whitespace().next().and_then(|x| x.parse().ok());
        match k.trim() {
            "CapEff" => s.cap_eff = hex(),
            "CapPrm" => s.cap_prm = hex(),
            "CapBnd" => s.cap_bnd = hex(),
            "CapAmb" => s.cap_amb = hex(),
            "NoNewPrivs" => s.no_new_privs = first(),
            "Seccomp" => s.seccomp = first(),
            "Uid" => s.uid = first(),
            _ => {}
        }
    }
    s
}

/// HC06 for this profile: the problems (none: ok).
pub fn status_problems(s: &ProcStatus, p: Profile) -> Vec<String> {
    let mut bad = Vec::new();
    let want = p.caps();
    for (name, v) in [("CapEff", s.cap_eff), ("CapPrm", s.cap_prm), ("CapBnd", s.cap_bnd)] {
        if v != Some(want) {
            bad.push(format!("{name} {}, want {want:016x}", v.map_or("?".into(), |v| format!("{v:016x}"))));
        }
    }
    if s.cap_amb != Some(0) {
        bad.push(format!("CapAmb {:?}, want 0", s.cap_amb));
    }
    if s.no_new_privs != Some(1) {
        bad.push(format!("NoNewPrivs {:?}, want 1", s.no_new_privs));
    }
    if s.seccomp != Some(2) {
        bad.push(format!("Seccomp {:?}, want 2 (filter)", s.seccomp));
    }
    if s.uid != Some(p.uid()) {
        bad.push(format!("Uid {:?}, want {}", s.uid, p.uid()));
    }
    bad
}

/// One socket of `/proc/net/{tcp,tcp6,udp,udp6}`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Socket {
    pub udp: bool,
    pub addr: std::net::IpAddr,
    pub port: u16,
    /// The `st` column (TCP 0A = LISTEN).
    pub state: u8,
}

fn ip_of(hex: &str) -> Option<std::net::IpAddr> {
    match hex.len() {
        8 => Some(Ipv4Addr::from(u32::from_str_radix(hex, 16).ok()?.to_le_bytes()).into()),
        32 => {
            let mut b = [0u8; 16];
            for i in 0..4 {
                let w = u32::from_str_radix(&hex[i * 8..i * 8 + 8], 16).ok()?;
                b[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
            }
            Some(Ipv6Addr::from(b).into())
        }
        _ => None,
    }
}

/// The sockets of one `/proc/net/<file>`.
pub fn parse_net(text: &str, udp: bool) -> Vec<Socket> {
    text.lines()
        .skip(1)
        .filter_map(|l| {
            let f: Vec<&str> = l.split_whitespace().collect();
            let (a, p) = f.get(1)?.split_once(':')?;
            Some(Socket {
                udp,
                addr: ip_of(a)?,
                port: u16::from_str_radix(p, 16).ok()?,
                state: u8::from_str_radix(f.get(3)?, 16).ok()?,
            })
        })
        .collect()
}

fn loopback(a: &std::net::IpAddr) -> bool {
    match a {
        std::net::IpAddr::V4(v) => v.is_loopback(),
        std::net::IpAddr::V6(v) => v.is_loopback() || v.to_ipv4_mapped().is_some_and(|m| m.is_loopback()),
    }
}

fn plain(a: &std::net::IpAddr) -> String {
    match a {
        std::net::IpAddr::V6(v) => v.to_ipv4_mapped().map_or(v.to_string(), |m| m.to_string()),
        v => v.to_string(),
    }
}

/// HC08: the sockets that break the listener rule (E1): TCP LISTEN off loopback; UDP off
/// loopback unless on a PN address with port 34964 or >= 32768 (`strict`: never).
pub fn listener_problems(sockets: &[Socket], pn: &[String], strict: bool) -> Vec<String> {
    let mut bad = Vec::new();
    for s in sockets {
        if loopback(&s.addr) {
            continue;
        }
        let a = plain(&s.addr);
        if !s.udp {
            if s.state == 0x0A {
                bad.push(format!("TCP listener on {a}:{}", s.port));
            }
            continue;
        }
        let pn_ok = !strict && pn.contains(&a) && (s.port == PN_PORT || s.port >= PN_CLIENT_MIN);
        if !pn_ok {
            bad.push(format!("UDP socket on {a}:{}", s.port));
        }
    }
    bad.sort();
    bad.dedup();
    bad
}

/// HC11: the threads not `SCHED_OTHER` (`/proc/*/task/*/stat` field 41, after the `(comm)`).
pub fn realtime_threads(stats: &str) -> Vec<String> {
    stats
        .lines()
        .filter_map(|l| {
            let (head, rest) = l.rsplit_once(')')?;
            let f: Vec<&str> = rest.split_whitespace().collect();
            let policy: u32 = f.get(38)?.parse().ok()?;
            (policy != 0).then(|| format!("{}) policy {policy}", head.trim()))
        })
        .collect()
}

/// HC11: the `VmLck:` lines that are not `0 kB`.
pub fn locked_memory(statuses: &str) -> Vec<String> {
    statuses
        .lines()
        .filter_map(|l| l.strip_prefix("VmLck:"))
        .map(str::trim)
        .filter(|v| *v != "0 kB")
        .map(|v| format!("VmLck {v}"))
        .collect()
}

/// The profile of `container` in a `kubectl get pod -o json` (its securityContext, E2.2).
pub fn profile_from_pod(pod: &Value, container: &str) -> Option<Profile> {
    let c = pod["spec"]["containers"]
        .as_array()?
        .iter()
        .find(|c| c["name"] == container)?;
    let caps = &c["securityContext"]["capabilities"];
    let names = |k: &str| -> BTreeSet<String> {
        caps[k].as_array().map(|a| a.iter().filter_map(|x| x.as_str().map(str::to_string)).collect()).unwrap_or_default()
    };
    let (drop, add) = (names("drop"), names("add"));
    if drop.contains("ALL") && add.is_empty() {
        return Some(Profile::None);
    }
    let raw: BTreeSet<String> = PN_RAW_DROP.iter().map(|s| s.to_string()).collect();
    if drop == raw {
        if add.is_empty() {
            return Some(Profile::PnRaw);
        }
        if add.len() == 1 && add.contains("NET_ADMIN") {
            return Some(Profile::PnAdmin);
        }
    }
    None
}

/// The addresses of interface `iface` in the pod's Multus `network-status` annotation.
pub fn pn_addresses_from_pod(pod: &Value, iface: &str) -> Vec<String> {
    let Some(text) = pod["metadata"]["annotations"]["k8s.v1.cni.cncf.io/network-status"].as_str() else {
        return Vec::new();
    };
    let Ok(Value::Array(nets)) = serde_json::from_str::<Value>(text) else {
        return Vec::new();
    };
    nets.iter()
        .filter(|n| n["interface"] == iface)
        .flat_map(|n| crate::krl::strings(&n["ips"]))
        .map(|ip| ip.split('/').next().unwrap_or("").to_string())
        .collect()
}

// ------------------------------------------------------------------ the run

fn missing_fields(v: &Value, required: &[&str]) -> Vec<String> {
    required.iter().filter(|k| v.get(**k).is_none()).map(|k| k.to_string()).collect()
}

// The exits HC03 allows per verb (handoff §2.2, §2.3).
const X_READY: &[i32] = &[0, 3];
const X_HEALTH: &[i32] = &[0, 4];
const X_READ: &[i32] = &[0, 2, 3, 4];
const X_DEPLOY: &[i32] = &[0, 3, 4, 7, 8];
const X_START: &[i32] = &[0, 3, 4, 7];
const X_WRITE: &[i32] = &[0, 3];
const X_STOP: &[i32] = &[0, 3, 4];
const X_GLOBALS: &[i32] = &[0, 3];
const X_OP: &[i32] = &[0, 2, 3, 4];
const X_VERIFY: &[i32] = &[0, 3, 7];
const X_PREPARE: &[i32] = &[0, 3, 7];

/// The verbs HC03 sends, their requests and the exits allowed, and whether they change something.
fn plan(role: Role, wiring: &Value) -> Vec<(Verb, Value, &'static [i32], bool)> {
    let none = "hooks-check-none.json";
    let mut v = vec![(Verb::Ready, json!({}), X_READY, false), (Verb::Health, json!({}), X_HEALTH, false)];
    match role {
        Role::Head => v.extend([
            (Verb::Read, hook::read_request(&["PLC_PRG.eCellState"]), X_READ, false),
            (Verb::Deploy, json!({"bundle": none, "wiring": wiring, "force": false}), X_DEPLOY, true),
            (Verb::Start, json!({"restart": "cold"}), X_START, true),
            (Verb::Write, hook::write_request(&[("PLC_PRG.bReset", true)]), X_WRITE, true),
            (Verb::Stop, json!({"runtime": false}), X_STOP, true),
        ]),
        Role::Robot => v.extend([
            (Verb::Globals, json!({}), X_GLOBALS, false),
            (Verb::Op, hook::op_request("status", None), X_OP, false),
            (Verb::Deploy, json!({"bundle": none, "force": false, "dry_run": false}), X_DEPLOY, true),
            (Verb::Verify, json!({}), X_VERIFY, true),
            (Verb::Prepare, json!({"mode": "EXT", "ack": true, "force": false}), X_PREPARE, true),
        ]),
    }
    v
}

/// The other role's verbs (2 "unimplemented: ..."); the head's optional `verify` not among them.
fn foreign(role: Role) -> Vec<(Verb, Value)> {
    match role {
        Role::Head => vec![
            (Verb::Prepare, json!({"mode": "EXT", "ack": true, "force": false})),
            (Verb::Op, hook::op_request("status", None)),
            (Verb::Globals, json!({})),
        ],
        Role::Robot => vec![
            (Verb::Start, json!({"restart": "cold"})),
            (Verb::Stop, json!({"runtime": false})),
            (Verb::Read, hook::read_request(&["PLC_PRG.eCellState"])),
            (Verb::Write, hook::write_request(&[("PLC_PRG.bReset", true)])),
        ],
    }
}

/// HC05 on one output: the problems.
pub fn output_problems(e: &Exec) -> Vec<String> {
    let mut bad = Vec::new();
    if e.overflow || e.stdout.len() > hook::MAX_STDOUT {
        bad.push("more than 1 MiB of stdout".into());
    }
    let text = String::from_utf8_lossy(&e.stdout);
    let lines: Vec<&str> = text.lines().filter(|l| !l.trim().is_empty()).collect();
    let Some((last, before)) = lines.split_last() else {
        return vec!["no output".into()];
    };
    let notes = before.iter().filter(|l| l.starts_with("note: ")).count();
    if notes != before.len() {
        bad.push(format!("{} line(s) before the answer without `note: `", before.len() - notes));
    }
    if notes > hook::MAX_NOTES {
        bad.push(format!("{notes} note lines, at most {}", hook::MAX_NOTES));
    }
    if before.iter().any(|l| l.len() > hook::MAX_NOTE + 6) {
        bad.push("a note longer than 1 KiB".into());
    }
    match serde_json::from_str::<Value>(last.trim()) {
        Ok(v @ Value::Object(_)) => {
            if v["ok"].as_bool() != Some(v["exit"].as_i64() == Some(0)) {
                bad.push(format!("ok {} with exit {}", v["ok"], v["exit"]));
            }
            if e.code.map(i64::from) != v["exit"].as_i64() {
                bad.push(format!("exit {} in the answer, {:?} from the program", v["exit"], e.code));
            }
        }
        _ => bad.push("the last line is not one JSON object".into()),
    }
    bad
}

struct Run<'r> {
    remote: &'r mut dyn Remote,
    checks: Vec<CheckResult>,
    /// Every hook output, for HC05.
    outputs: Vec<(String, Exec)>,
}

impl Run<'_> {
    fn push(&mut self, id: &str, ok: bool, detail: impl Into<String>) {
        self.checks.push(CheckResult { id: id.into(), ok, detail: detail.into(), skipped: false });
    }

    fn skip(&mut self, id: &str, why: impl Into<String>) {
        self.checks.push(CheckResult { id: id.into(), ok: true, detail: why.into(), skipped: true });
    }

    fn exec(&mut self, argv: &[&str], stdin: &[u8], limits: &ExecLimits) -> Result<Exec> {
        self.remote.exec(argv, stdin, limits)
    }

    /// `hook <verb>` with raw stdin, kept for HC05.
    fn raw(&mut self, verb: &str, stdin: &[u8], limits: &ExecLimits) -> Result<Exec> {
        let e = self.remote.exec(&[HOOK, verb], stdin, limits)?;
        self.outputs.push((verb.to_string(), e.clone()));
        Ok(e)
    }

    fn hook(&mut self, verb: Verb, request: &Value) -> Result<Answer> {
        let text = hook::request_text(request);
        let e = self.raw(verb.name(), text.as_bytes(), &verb.limits())?;
        Ok(hook::parse(verb, &e))
    }

    /// A probe through `sh -c` (the images guarantee sh): its stdout, or why not.
    fn sh(&mut self, script: &str) -> std::result::Result<String, String> {
        match self.exec(&["sh", "-c", script], b"", &ExecLimits::default()) {
            Ok(e) if e.code == Some(0) => Ok(e.stdout_text()),
            Ok(e) => Err(format!("exit {:?}: {}", e.code, e.stderr_text().trim())),
            Err(e) => Err(e.to_string()),
        }
    }
}

/// Run every check on `remote` (a pod's container or a docker container). `no_workload`: the
/// `--docker` describe with `--entrypoint /opt/factoryos/hook` (HC02).
pub fn run(
    remote: &mut dyn Remote,
    target: &str,
    o: &Options,
    no_workload: Option<Exec>,
) -> HooksReport {
    let mut r = Run { remote, checks: Vec::new(), outputs: Vec::new() };
    let mut report = HooksReport {
        schema: REPORT_SCHEMA.into(),
        target: target.into(),
        mode: o.mode,
        role: o.role.map(|x| x.semantic().to_string()),
        profile: o.profile.map(|p| p.name().to_string()),
        checks: Vec::new(),
        ok: false,
        unreachable: false,
    };
    // HC01
    let probe = "for c in bash sh cat mv; do command -v \"$c\" >/dev/null 2>&1 || echo \"missing $c\"; done; echo p2c-ok";
    let hc01 = r.sh(probe);
    match &hc01 {
        Ok(out) if out.lines().any(|l| l.trim() == "p2c-ok") => {
            let missing: Vec<&str> = out.lines().filter_map(|l| l.strip_prefix("missing ")).collect();
            r.push("HC01", missing.is_empty(), if missing.is_empty() { "bash, sh, cat, mv".to_string() } else { format!("missing: {}", missing.join(", ")) });
        }
        Ok(out) => r.push("HC01", false, format!("no answer from sh: {}", out.trim())),
        Err(e) => r.push("HC01", false, format!("sh: {e}")),
    }
    // HC02
    let d = match r.hook(Verb::Describe, &json!({})) {
        Ok(a) => a,
        Err(e) => {
            r.push("HC02", false, format!("describe: {e}"));
            report.unreachable = true;
            report.checks = r.checks;
            return report;
        }
    };
    if d.problem.is_some() && hc01.is_err() {
        r.push("HC02", false, d.error());
        report.unreachable = true;
        report.checks = r.checks;
        return report;
    }
    let mut hc02 = Vec::new();
    if !d.ok() {
        hc02.push(d.failure("describe"));
    }
    let missing = missing_fields(&d.json, &DESCRIBE_REQUIRED);
    if d.ok() && !missing.is_empty() {
        hc02.push(format!("describe lacks {}", missing.join(", ")));
    }
    if d.ok() && d.json["contract"] != CONTRACT {
        hc02.push(format!("contract {}", d.json["contract"]));
    }
    let described = d.json["role"].as_str().and_then(Role::parse);
    let role = o.role.or(described);
    // E7.2: describe names the component's role; --role does not excuse an image that names none.
    if d.ok() && described.is_none() {
        hc02.push(format!("describe role {}, want head-plc or robot-controller", d.json["role"]));
    }
    if let (Some(want), Some(got)) = (o.role, described)
        && want != got
    {
        hc02.push(format!("describe role {}, checked as {}", got.semantic(), want.semantic()));
    }
    if role == Some(Role::Robot) {
        let m = missing_fields(&d.json, &DESCRIBE_ROBOT_REQUIRED);
        if !m.is_empty() {
            hc02.push(format!("a robot's describe lacks {}", m.join(", ")));
        }
        if d.json.get("inbox").is_some() && d.json["inbox"] != INBOX {
            hc02.push(format!("inbox {}, want {INBOX}", d.json["inbox"]));
        }
    }
    if let Some(e) = &no_workload {
        let a = hook::parse(Verb::Describe, e);
        r.outputs.push(("describe (no workload)".into(), e.clone()));
        if !a.ok() {
            hc02.push(format!("without a workload: {}", a.failure("describe")));
        }
    }
    let detail = if hc02.is_empty() {
        format!("{} {}", d.json["runtime"].as_str().unwrap_or("?"), role.map_or("?", |x| x.semantic()))
    } else {
        hc02.join("; ")
    };
    r.push("HC02", hc02.is_empty(), detail);
    report.role = role.map(|x| x.semantic().to_string());
    let Some(role) = role else {
        for id in ["HC03", "HC09", "HC10", "HC12"] {
            r.skip(id, "role unknown (give --role)");
        }
        finish_static(&mut r, o, role);
        report.checks = sorted(r.checks);
        report.ok = report.checks.iter().all(|c| c.ok);
        return report;
    };
    // HC12 first in a fresh container: the workload comes up.
    if o.mode == Mode::Docker {
        let until = Instant::now() + o.ready_wait;
        let mut last = String::new();
        let ok = loop {
            match r.hook(Verb::Ready, &json!({})) {
                Ok(a) if a.ok() && a.json["ready"] == true => {
                    if role == Role::Robot && a.json["info"]["ar"] != "none" {
                        last = format!("ready, but info.ar {} (want \"none\": no PROFINET controller)", a.json["info"]["ar"]);
                    } else {
                        break true;
                    }
                }
                Ok(a) => last = if a.ok() { "ready: false".into() } else { a.error() },
                Err(e) => last = e.to_string(),
            }
            if Instant::now() + o.ready_poll > until {
                break false;
            }
            std::thread::sleep(o.ready_poll);
        };
        r.push("HC12", ok, if ok { "ready without a PROFINET controller".to_string() } else { format!("not ready within {} s: {last}", o.ready_wait.as_secs()) });
    } else {
        r.skip("HC12", "--docker only");
    }
    // HC03, HC10
    let wiring = json!({
        "head_ip": o.pn_addresses.first().cloned().unwrap_or_else(|| docker_ip(Role::Head).into()),
        "netmask": "255.255.255.0",
        "gateway": "0.0.0.0",
        "controllers": [],
    });
    let mut hc03 = Vec::new();
    let mut ran = vec!["describe"];
    let mut health = None;
    for (verb, req, allowed, changes) in plan(role, &wiring) {
        if changes && !o.changes() {
            continue;
        }
        ran.push(verb.name());
        match r.hook(verb, &req) {
            Ok(a) => {
                if a.problem.is_some() || !allowed.contains(&a.exit) {
                    hc03.push(format!("{}: {}", verb.name(), if a.problem.is_some() { a.error() } else { format!("exit {} (allowed {allowed:?}): {}", a.exit, a.error()) }));
                } else if verb == Verb::Ready && a.ok() && !missing_fields(&a.json, &READY_REQUIRED).is_empty() {
                    hc03.push(format!("ready lacks {}", missing_fields(&a.json, &READY_REQUIRED).join(", ")));
                }
                if verb == Verb::Health {
                    health = Some(a);
                }
            }
            Err(e) => hc03.push(format!("{}: {e}", verb.name())),
        }
    }
    if o.changes() {
        for (verb, req) in foreign(role) {
            ran.push(verb.name());
            match r.hook(verb, &req) {
                Ok(a) if a.exit == 2 && a.json["error"].as_str().is_some_and(|e| e.starts_with("unimplemented")) => {}
                Ok(a) => hc03.push(format!("{} (not a {} verb): exit {}, want 2 \"unimplemented: ...\" ({})", verb.name(), role.semantic(), a.exit, a.error())),
                Err(e) => hc03.push(format!("{}: {e}", verb.name())),
            }
        }
    }
    // watch for 1 s
    ran.push("watch");
    let limits = ExecLimits { timeout: Some(Duration::from_secs(15)), ..Verb::Watch.limits() };
    let req = hook::request_text(&json!({"period_ms": 200, "max_s": 1}));
    match r.exec(&[HOOK, "watch"], req.as_bytes(), &limits) {
        Ok(e) => {
            let a = hook::parse(Verb::Watch, &e);
            if !a.ok() {
                hc03.push(format!("watch: {}", a.error()));
            }
            hc03.extend(watch_problems(&e, role));
        }
        Err(e) => hc03.push(format!("watch: {e}")),
    }
    if role == Role::Robot && !o.ops.is_empty() {
        let has: BTreeSet<&str> = d.json["ops"].as_array().map(|a| a.iter().filter_map(|x| x["name"].as_str()).collect()).unwrap_or_default();
        let lacking: Vec<&str> = o.ops.iter().map(String::as_str).filter(|x| !has.contains(x)).collect();
        if !lacking.is_empty() {
            hc03.push(format!("describe.ops lacks the template's {}", lacking.join(", ")));
        }
    }
    r.push("HC03", hc03.is_empty(), if hc03.is_empty() { format!("{} answered{}", ran.join(", "), if o.changes() { "" } else { " (changing verbs not run: --mutating)" }) } else { hc03.join("; ") });
    match health {
        Some(a) if a.ok() => match crate::runtime::from_hook_health(&a.json) {
            Ok(rt) => r.push("HC10", true, format!("{} runtime(s)", rt.runtimes.len())),
            Err(e) => r.push("HC10", false, e.to_string()),
        },
        Some(a) => r.push("HC10", false, format!("health: {}", a.error())),
        None => r.push("HC10", false, "health not answered"),
    }
    // HC04
    let mut hc04 = Vec::new();
    for (verb, stdin) in [("bogus", &b"{}"[..]), ("ready", &b"{"[..]), ("ready", &br#"{"zzz":1}"#[..])] {
        match r.raw(verb, stdin, &Verb::Ready.limits()) {
            Ok(e) if e.code == Some(2) => {}
            Ok(e) => hc04.push(format!("{verb} < {}: exit {:?}, want 2", String::from_utf8_lossy(stdin), e.code)),
            Err(e) => hc04.push(format!("{verb}: {e}")),
        }
    }
    r.push("HC04", hc04.is_empty(), if hc04.is_empty() { "bogus verb, `{`, unknown field: 2".to_string() } else { hc04.join("; ") });
    // HC09
    if o.changes() {
        let mut hc09 = Vec::new();
        let local = std::env::temp_dir().join(format!("plan2cell-hooks-check-{}.txt", std::process::id()));
        let dest = format!("{INBOX}/hooks-check.txt");
        let wrote = std::fs::write(&local, "hooks-check\n").map_err(|e| e.to_string());
        let copied = wrote.and_then(|_| r.remote.copy(&local, &dest).map_err(|e| e.to_string()));
        let _ = std::fs::remove_file(&local);
        match copied {
            Ok(()) => match r.exec(&["cat", &dest], b"", &ExecLimits::default()) {
                Ok(e) if e.stdout_text() == "hooks-check\n" => {}
                Ok(e) => hc09.push(format!("{dest} reads back {:?}", e.stdout_text())),
                Err(e) => hc09.push(e.to_string()),
            },
            Err(e) => hc09.push(format!("copy into the inbox: {e}")),
        }
        let req = match role {
            Role::Head => json!({"bundle": "hooks-check.json.p2c-tmp", "wiring": wiring, "force": false}),
            Role::Robot => json!({"bundle": "hooks-check.json.p2c-tmp", "force": false, "dry_run": false}),
        };
        match r.hook(Verb::Deploy, &req) {
            Ok(a) if a.exit == 3 => {}
            Ok(a) => hc09.push(format!("deploy of a .p2c-tmp file: exit {}, want 3", a.exit)),
            Err(e) => hc09.push(e.to_string()),
        }
        r.push("HC09", hc09.is_empty(), if hc09.is_empty() { "copy into the inbox; .p2c-tmp refused".to_string() } else { hc09.join("; ") });
    } else {
        r.skip("HC09", "writes into the inbox (--mutating)");
    }
    finish_static(&mut r, o, Some(role));
    report.checks = sorted(r.checks);
    report.ok = report.checks.iter().all(|c| c.ok);
    report
}

/// HC05-HC08 and HC11: the static checks (no role needed).
fn finish_static(r: &mut Run, o: &Options, _role: Option<Role>) {
    // HC05
    let mut hc05 = Vec::new();
    for (verb, e) in r.outputs.clone() {
        for p in output_problems(&e) {
            hc05.push(format!("{verb}: {p}"));
        }
    }
    let n = r.outputs.len();
    r.push("HC05", hc05.is_empty(), if hc05.is_empty() { format!("{n} answer(s)") } else { hc05.join("; ") });
    // HC06
    match (o.profile, r.exec(&["cat", "/proc/1/status"], b"", &ExecLimits::default())) {
        (None, _) => r.skip("HC06", "profile unknown (give --profile)"),
        (Some(p), Ok(e)) if e.code == Some(0) => {
            let bad = status_problems(&parse_status(&e.stdout_text()), p);
            r.push("HC06", bad.is_empty(), if bad.is_empty() { format!("{} (CapEff {:016x})", p.name(), p.caps()) } else { bad.join("; ") });
        }
        (Some(_), Ok(e)) => r.push("HC06", false, format!("cat /proc/1/status: exit {:?}", e.code)),
        (Some(_), Err(e)) => r.push("HC06", false, e.to_string()),
    }
    // HC07
    match r.sh(&format!("for d in /opt /run {INBOX}; do if [ -w \"$d\" ]; then echo \"$d w\"; else echo \"$d ro\"; fi; done")) {
        Ok(out) => {
            let want = [("/opt", "ro"), ("/run", "w"), (INBOX, "w")];
            let bad: Vec<String> = want
                .iter()
                .filter(|(d, m)| !out.lines().any(|l| l.trim() == format!("{d} {m}")))
                .map(|(d, m)| format!("{d} should be {}", if *m == "w" { "writable" } else { "read-only" }))
                .collect();
            r.push("HC07", bad.is_empty(), if bad.is_empty() { "/opt read-only, /run and the inbox writable".to_string() } else { bad.join("; ") });
        }
        Err(e) => r.push("HC07", false, e),
    }
    // HC08
    match r.sh("for f in tcp tcp6 udp udp6; do echo \"== $f\"; cat /proc/net/$f 2>/dev/null; done") {
        Ok(out) => {
            let mut socks = Vec::new();
            for sec in out.split("== ").skip(1) {
                let (name, body) = sec.split_once('\n').unwrap_or((sec, ""));
                socks.extend(parse_net(body, name.trim().starts_with("udp")));
            }
            let bad = listener_problems(&socks, &o.pn_addresses, o.strict_udp);
            let pn = if o.pn_addresses.is_empty() { "PN address unknown".to_string() } else { format!("PN {}", o.pn_addresses.join(", ")) };
            r.push("HC08", bad.is_empty(), if bad.is_empty() { format!("{} socket(s), loopback or PROFINET only ({pn})", socks.len()) } else { format!("{} ({pn})", bad.join("; ")) });
        }
        Err(e) => r.push("HC08", false, e),
    }
    // HC11
    match r.sh("for f in /proc/[0-9]*/task/[0-9]*/stat; do cat \"$f\" 2>/dev/null; echo; done; echo '== status'; for f in /proc/[0-9]*/status; do cat \"$f\" 2>/dev/null; done") {
        Ok(out) => {
            let (stats, statuses) = out.split_once("== status").unwrap_or((out.as_str(), ""));
            let mut bad = realtime_threads(stats);
            bad.extend(locked_memory(statuses));
            let threads = stats.lines().filter(|l| l.contains(')')).count();
            r.push("HC11", bad.is_empty(), if bad.is_empty() { format!("{threads} thread(s) SCHED_OTHER, no locked memory") } else { bad.join("; ") });
        }
        Err(e) => r.push("HC11", false, e),
    }
}

fn sorted(mut v: Vec<CheckResult>) -> Vec<CheckResult> {
    v.sort_by(|a, b| a.id.cmp(&b.id));
    v
}

/// HC03 on a `watch` output: every data line a `factoryos/hook-watch@1` line of the role, then
/// the envelope with `lines` and `reason`.
pub fn watch_problems(e: &Exec, role: Role) -> Vec<String> {
    let mut bad = Vec::new();
    let text = e.stdout_text();
    let lines: Vec<&str> = text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with("note: ")).collect();
    let Some((last, data)) = lines.split_last() else {
        return vec!["watch: no output".into()];
    };
    let kind = match role {
        Role::Head => "head",
        Role::Robot => "robot",
    };
    let required: &[&str] = match role {
        Role::Head => &WATCH_HEAD_REQUIRED,
        Role::Robot => &WATCH_ROBOT_REQUIRED,
    };
    for (i, l) in data.iter().enumerate() {
        let v: Value = match serde_json::from_str(l) {
            Ok(v) => v,
            Err(_) => {
                bad.push(format!("watch line {}: not JSON", i + 1));
                continue;
            }
        };
        let m = missing_fields(&v, required);
        if v["schema"] != WATCH_SCHEMA || v["kind"] != kind || !m.is_empty() {
            bad.push(format!("watch line {}: not a {WATCH_SCHEMA} {kind} line{}", i + 1, if m.is_empty() { String::new() } else { format!(" (lacks {})", m.join(", ")) }));
        } else if role == Role::Robot {
            let hex64 = |x: &Value| x.as_str().is_some_and(|s| s.len() == 64 && s.chars().all(|c| c.is_ascii_hexdigit()));
            if !hex64(&v["io"]["in"]) || !hex64(&v["io"]["out"]) {
                bad.push(format!("watch line {}: io.in / io.out not 64 hex digits", i + 1));
            }
        }
        if bad.len() > 5 {
            break;
        }
    }
    match serde_json::from_str::<Value>(last) {
        Ok(v) if v["schema"] == CONTRACT && v["verb"] == "watch" => {
            if !v["lines"].is_u64() || !matches!(v["reason"].as_str(), Some("max_s" | "limit" | "closed")) {
                bad.push("watch: the envelope lacks lines / reason (max_s, limit, closed)".into());
            }
        }
        _ => bad.push("watch: the last line is not the factoryos/hook@1 envelope".into()),
    }
    bad
}

// ------------------------------------------------------------------ --docker

/// A throwaway container for `--docker` (E11): an `--internal` network, the container on it with
/// the profile; both removed on drop. Nothing is published: no port reaches the host's network.
pub struct DockerRun {
    pub network: String,
    pub container: String,
    started: bool,
}

/// What `--docker` starts the image with.
#[derive(Clone, Debug)]
pub struct DockerSpec {
    pub image: String,
    pub role: Role,
    pub profile: Profile,
    /// `--env NAME=VALUE` (e.g. `FOS_ROBOT_NS`), after the fixed ones.
    pub env: Vec<String>,
    /// The container's arguments (`-- ARGS`).
    pub args: Vec<String>,
}

impl DockerRun {
    pub fn names(pid: u32) -> (String, String) {
        (format!("p2c-hooks-check-{pid}"), format!("p2c-hooks-check-{pid}"))
    }

    pub fn network_args(network: &str) -> Vec<String> {
        ["network", "create", "--internal", "--subnet", DOCKER_SUBNET, network].map(String::from).to_vec()
    }

    /// `docker run -d ...` of the container (E11: the network endpoint named net1 at the role's
    /// address, read-only root, no-new-privileges, the profile, tmpfs /run and inbox).
    pub fn run_args(network: &str, container: &str, s: &DockerSpec) -> Vec<String> {
        let ip = docker_ip(s.role);
        let mut a: Vec<String> = ["run", "-d", "--name", container].map(String::from).to_vec();
        a.extend([
            "--network".to_string(),
            format!("name={network},driver-opt=com.docker.network.endpoint.ifname=net1,ip={ip}"),
            "--read-only".into(),
            "--security-opt".into(),
            "no-new-privileges".into(),
        ]);
        a.extend(s.profile.docker_flags());
        // emptyDir is world-writable; so are these (the none profile runs as 65532).
        a.extend(
            ["--tmpfs", "/run:rw,size=1g,mode=1777", "--tmpfs", "/var/lib/factoryos/inbox:rw,size=256m,mode=1777"]
                .map(String::from),
        );
        for e in [
            "FOS_COMPONENT=hooks-check".to_string(),
            format!("FOS_HOOK_ROLE={}", s.role.semantic()),
            "FOS_PN_IFACE=net1".to_string(),
            format!("FOS_PN_ADDRESS={ip}/24"),
        ]
        .into_iter()
        .chain(s.env.iter().cloned())
        {
            a.extend(["-e".to_string(), e]);
        }
        a.push(s.image.clone());
        a.extend(s.args.iter().cloned());
        a
    }

    /// `docker run --rm -i` of the hook alone (`describe`, no workload, no network).
    pub fn describe_args(image: &str) -> Vec<String> {
        [
            "run", "--rm", "-i", "--network", "none", "--read-only", "--security-opt", "no-new-privileges",
            "--cap-drop", "ALL", "--user", "65532:65532", "--entrypoint", HOOK, image, "describe",
        ]
        .map(String::from)
        .to_vec()
    }

    fn docker(args: &[String]) -> Result<String> {
        let o = std::process::Command::new("docker")
            .args(args)
            .stdin(std::process::Stdio::null())
            .output()
            .map_err(|e| Error::Remote(format!("docker: {e}")))?;
        if o.status.success() {
            Ok(String::from_utf8_lossy(&o.stdout).into_owned())
        } else {
            Err(Error::Remote(format!(
                "docker {}: {}",
                args.first().map(String::as_str).unwrap_or(""),
                String::from_utf8_lossy(&o.stderr).trim()
            )))
        }
    }

    /// Docker 26 or later (the endpoint `ifname` option).
    pub fn check_version() -> Result<String> {
        // `{{json ...}}` prints the version as a JSON string ("27.3.1").
        let v = Self::docker(&["version".into(), "--format".into(), "{{json .Server.Version}}".into()])?;
        let v = v.trim().trim_matches('"').to_string();
        let major: u32 = v.split('.').next().and_then(|m| m.parse().ok()).unwrap_or(0);
        if major < 26 {
            return Err(Error::Remote(format!("docker {v}: hooks-check --docker needs docker 26 or later")));
        }
        Ok(v)
    }

    /// The describe without a workload ([`DockerRun::describe_args`]).
    pub fn describe(image: &str) -> Result<Exec> {
        let mut c = std::process::Command::new("docker");
        c.args(Self::describe_args(image));
        crate::remote::exec_command(c, b"{}", &Verb::Describe.limits(), "docker run --entrypoint describe")
    }

    /// Create the network and start the container.
    pub fn start(s: &DockerSpec) -> Result<Self> {
        let (network, container) = Self::names(std::process::id());
        Self::docker(&Self::network_args(&network))?;
        let mut d = Self { network, container, started: false };
        Self::docker(&Self::run_args(&d.network, &d.container, s))?;
        d.started = true;
        Ok(d)
    }

    pub fn remote(&self) -> crate::remote::Docker {
        crate::remote::Docker { container: self.container.clone() }
    }
}

impl Drop for DockerRun {
    fn drop(&mut self) {
        if self.started {
            let _ = Self::docker(&["rm".into(), "-f".into(), self.container.clone()]);
        }
        let _ = Self::docker(&["network".into(), "rm".into(), self.network.clone()]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::remote::MockRemote;

    #[test]
    fn the_published_schemas_are_the_checked_fields() {
        let schema = |name: &str| -> Value {
            let (_, text) = SCHEMAS.iter().find(|(n, _)| *n == name).unwrap();
            serde_json::from_str(text).unwrap()
        };
        let req = |v: &Value| -> Vec<String> { crate::krl::strings(&v["required"]) };
        assert_eq!(req(&schema("envelope")), ENVELOPE_REQUIRED);
        assert_eq!(req(&schema("describe")), DESCRIBE_REQUIRED);
        assert_eq!(req(&schema("describe")["then"]), DESCRIBE_ROBOT_REQUIRED);
        assert_eq!(req(&schema("ready")), READY_REQUIRED);
        assert_eq!(req(&schema("health")), HEALTH_REQUIRED);
        assert_eq!(req(&schema("watch-robot")), WATCH_ROBOT_REQUIRED);
        assert_eq!(req(&schema("watch-head")), WATCH_HEAD_REQUIRED);
        assert_eq!(schema("envelope")["properties"]["schema"]["const"], CONTRACT);
        assert_eq!(schema("health")["properties"]["runtime"]["properties"]["schema"]["const"], crate::runtime::RUNTIME_SCHEMA);
        // The bundle and the wiring plan2cell sends have exactly the schemas' fields.
        let props = |v: &Value| -> Vec<String> { v["properties"].as_object().unwrap().keys().cloned().collect() };
        assert_eq!(props(&schema("head-bundle")), ["files", "key", "plan_id", "schema"]);
        assert_eq!(req(&schema("wiring")), ["head_ip", "netmask", "gateway", "controllers"]);
        let w = hook::wiring(&crate::CellConfig::default()).unwrap();
        let mut keys: Vec<String> = w.as_object().unwrap().keys().cloned().collect();
        keys.sort();
        let mut want = props(&schema("wiring"));
        want.sort();
        assert_eq!(keys, want);
    }

    const STATUS_PN_RAW: &str = "Name:\thook\nUid:\t0\t0\t0\t0\nNoNewPrivs:\t1\nSeccomp:\t2\nCapInh:\t0000000000000000\n\
        CapPrm:\t0000000000002000\nCapEff:\t0000000000002000\nCapBnd:\t0000000000002000\nCapAmb:\t0000000000000000\n";

    #[test]
    fn capabilities_per_profile() {
        let s = parse_status(STATUS_PN_RAW);
        assert_eq!((s.cap_eff, s.uid, s.seccomp), (Some(0x2000), Some(0), Some(2)));
        assert!(status_problems(&s, Profile::PnRaw).is_empty());
        let bad = status_problems(&s, Profile::None);
        assert!(bad.iter().any(|b| b.starts_with("CapEff 0000000000002000, want 0000000000000000")), "{bad:?}");
        assert!(bad.iter().any(|b| b.starts_with("Uid")), "{bad:?}");
        let admin = STATUS_PN_RAW.replace("0000000000002000", "0000000000003000");
        assert!(status_problems(&parse_status(&admin), Profile::PnAdmin).is_empty());
        let nnp = STATUS_PN_RAW.replace("NoNewPrivs:\t1", "NoNewPrivs:\t0");
        assert_eq!(status_problems(&parse_status(&nnp), Profile::PnRaw), ["NoNewPrivs Some(0), want 1"]);
        assert_eq!(Profile::PnRaw.docker_flags().iter().filter(|f| *f == "--cap-drop").count(), 13);
        assert!(Profile::PnAdmin.docker_flags().ends_with(&["--cap-add".into(), "NET_ADMIN".into()]));
        assert_eq!(Profile::None.docker_flags(), ["--user", "65532:65532", "--cap-drop", "ALL"]);
    }

    // /proc/net/{tcp,udp,udp6} as the kernel prints them.
    const TCP: &str = "  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode\n\
       0: 0100007F:1F90 00000000:0000 0A 00000000:00000000 00:00000000 00000000     0        0 1 1 0 100 0 0 10 0\n\
       1: 0A014D0A:C350 0B014D0A:8892 01 00000000:00000000 00:00000000 00000000     0        0 2 1 0 100 0 0 10 0\n";
    const TCP_ANY: &str = "  sl  local_address rem_address   st\n   0: 00000000:1F90 00000000:0000 0A 0\n";
    // 10.77.1.10:34964 and :49152, 127.0.0.1:5353, 0.0.0.0:68.
    const UDP: &str = "  sl  local_address rem_address   st\n\
       0: 0A014D0A:8894 00000000:0000 07 0\n   1: 0A014D0A:C000 00000000:0000 07 0\n\
       2: 0100007F:14E9 00000000:0000 07 0\n";
    const UDP_ANY: &str = "  sl  local_address rem_address   st\n   0: 00000000:0044 00000000:0000 07 0\n";
    const TCP6: &str = "  sl  local_address                         remote_address                        st\n\
       0: 00000000000000000000000001000000:1F90 00000000000000000000000000000000:0000 0A 0\n\
       1: 0000000000000000FFFF00000100007F:1F91 00000000000000000000000000000000:0000 0A 0\n";
    const TCP6_ANY: &str = "  sl  local_address                         remote_address                        st\n\
       0: 00000000000000000000000000000000:1F90 00000000000000000000000000000000:0000 0A 0\n";

    #[test]
    fn listeners_loopback_or_profinet_only() {
        let pn = vec!["10.77.1.10".to_string()];
        let mut s = parse_net(TCP, false);
        assert_eq!(s[0].addr.to_string(), "127.0.0.1");
        assert_eq!((s[0].port, s[0].state), (8080, 0x0A));
        s.extend(parse_net(UDP, true));
        s.extend(parse_net(TCP6, false));
        assert!(listener_problems(&s, &pn, false).is_empty(), "{:?}", listener_problems(&s, &pn, false));
        // --strict-udp: loopback only.
        assert_eq!(listener_problems(&s, &pn, true), ["UDP socket on 10.77.1.10:34964", "UDP socket on 10.77.1.10:49152"]);
        // Not the PN address, a wildcard, a TCP listener off loopback.
        assert_eq!(listener_problems(&parse_net(UDP, true), &[], false).len(), 2);
        assert_eq!(listener_problems(&parse_net(UDP_ANY, true), &pn, false), ["UDP socket on 0.0.0.0:68"]);
        assert_eq!(listener_problems(&parse_net(TCP_ANY, false), &pn, false), ["TCP listener on 0.0.0.0:8080"]);
        assert_eq!(listener_problems(&parse_net(TCP6_ANY, false), &pn, false), ["TCP listener on :::8080"]);
        // A PN address on a port below 32768 other than 34964.
        let low = "  sl  local_address rem_address   st\n   0: 0A014D0A:1F90 00000000:0000 07 0\n";
        assert_eq!(listener_problems(&parse_net(low, true), &pn, false), ["UDP socket on 10.77.1.10:8080"]);
    }

    #[test]
    fn realtime_threads_and_locked_memory() {
        let mut fields = vec!["0"; 50];
        let other = format!("12 (hook (x)) S {}", fields.join(" "));
        fields[37] = "1"; // field 41 (policy): the 38th after the state
        let fifo = format!("13 (krl-sim) S {}", fields.join(" "));
        assert!(realtime_threads(&other).is_empty());
        assert_eq!(realtime_threads(&format!("{other}\n{fifo}\n")), ["13 (krl-sim) policy 1"]);
        assert!(locked_memory("VmLck:\t       0 kB\n").is_empty());
        assert_eq!(locked_memory("VmLck:\t     128 kB\n"), ["VmLck 128 kB"]);
    }

    #[test]
    fn profile_and_pn_address_from_the_pod() {
        let pod = |sc: Value| {
            json!({"metadata": {"annotations": {"k8s.v1.cni.cncf.io/network-status":
                "[{\"name\":\"aws-cni\",\"interface\":\"eth0\",\"ips\":[\"172.31.5.9\"]},{\"name\":\"cell-jdoe/pn-n1\",\"interface\":\"net1\",\"ips\":[\"10.77.1.10\",\"10.77.1.1\"]}]"}},
                "spec": {"containers": [{"name": "head", "securityContext": sc}]}})
        };
        let raw = json!({"capabilities": {"drop": PN_RAW_DROP}, "runAsUser": 0});
        assert_eq!(profile_from_pod(&pod(raw.clone()), "head"), Some(Profile::PnRaw));
        assert_eq!(profile_from_pod(&pod(json!({"capabilities": {"drop": ["ALL"]}})), "head"), Some(Profile::None));
        assert_eq!(profile_from_pod(&pod(json!({"capabilities": {"drop": PN_RAW_DROP, "add": ["NET_ADMIN"]}})), "head"), Some(Profile::PnAdmin));
        assert_eq!(profile_from_pod(&pod(json!({"privileged": true})), "head"), None);
        assert_eq!(profile_from_pod(&pod(raw), "other"), None);
        assert_eq!(pn_addresses_from_pod(&pod(json!({})), "net1"), ["10.77.1.10", "10.77.1.1"]);
        assert!(pn_addresses_from_pod(&json!({}), "net1").is_empty());
    }

    #[test]
    fn docker_runs_the_profile_on_an_internal_network() {
        let s = DockerSpec {
            image: "factoryos-cell/hook-stub:dev".into(),
            role: Role::Robot,
            profile: Profile::None,
            env: vec!["FOS_ROBOT_NS=robot1".into()],
            args: vec![],
        };
        assert_eq!(DockerRun::network_args("n"), ["network", "create", "--internal", "--subnet", "10.77.250.0/24", "n"]);
        let a = DockerRun::run_args("n", "c", &s).join(" ");
        assert!(a.starts_with("run -d --name c --network name=n,driver-opt=com.docker.network.endpoint.ifname=net1,ip=10.77.250.11 --read-only --security-opt no-new-privileges --user 65532:65532 --cap-drop ALL"), "{a}");
        assert!(a.contains("-e FOS_PN_ADDRESS=10.77.250.11/24 -e FOS_ROBOT_NS=robot1 factoryos-cell/hook-stub:dev"), "{a}");
        assert!(!a.contains("-p ") && !a.contains("--publish"), "{a}");
        assert!(DockerRun::describe_args("i").join(" ").contains("--network none --read-only"));
    }

    #[test]
    fn output_rules() {
        let ok = Exec::exit(0, "note: x\n{\"schema\":\"factoryos/hook@1\",\"verb\":\"ready\",\"ok\":true,\"exit\":0}\n");
        assert!(output_problems(&ok).is_empty());
        let stray = Exec::exit(0, "hello\n{\"schema\":\"factoryos/hook@1\",\"verb\":\"ready\",\"ok\":true,\"exit\":0}\n");
        assert_eq!(output_problems(&stray), ["1 line(s) before the answer without `note: `"]);
        let wrong = Exec::exit(3, "{\"schema\":\"factoryos/hook@1\",\"verb\":\"ready\",\"ok\":true,\"exit\":3}\n");
        assert_eq!(output_problems(&wrong), ["ok true with exit 3"]);
        assert_eq!(output_problems(&Exec::exit(0, "")), ["no output"]);
    }

    /// A conforming fake robot hook over the mock remote: every check passes in docker mode.
    #[test]
    fn a_conforming_robot_passes_every_check() {
        let env = |verb: &str, exit: i32, extra: &str| {
            Exec::exit(exit, format!("note: fake\n{{\"schema\":\"factoryos/hook@1\",\"verb\":\"{verb}\",\"ok\":{},\"exit\":{exit}{extra}}}\n", exit == 0))
        };
        let status = STATUS_PN_RAW.replace("Uid:\t0", "Uid:\t65532").replace("0000000000002000", "0000000000000000");
        let mut remote = MockRemote::default().on_exec(move |argv, stdin| {
            let stdin = String::from_utf8_lossy(stdin).to_string();
            Ok(match argv.iter().map(String::as_str).collect::<Vec<_>>().as_slice() {
                ["sh", "-c", s] if s.starts_with("for c in bash") => Exec::exit(0, "p2c-ok\n"),
                ["sh", "-c", s] if s.starts_with("for d in /opt") => Exec::exit(0, "/opt ro\n/run w\n/var/lib/factoryos/inbox w\n"),
                ["sh", "-c", s] if s.contains("/proc/net") => Exec::exit(0, format!("== tcp\n{TCP}== udp\n{UDP}")),
                ["sh", "-c", s] if s.contains("task") => Exec::exit(0, format!("12 (hook) S {}\n== status\nVmLck:\t0 kB\n", vec!["0"; 50].join(" "))),
                ["cat", "/proc/1/status"] => Exec::exit(0, status.clone()),
                ["cat", p] if p.ends_with("hooks-check.txt") => Exec::exit(0, "hooks-check\n"),
                [HOOK, verb] => match (*verb, stdin.as_str()) {
                    (_, "{") | (_, "{\"zzz\":1}") | ("bogus", _) => env(verb, 2, ",\"error\":\"usage\""),
                    ("describe", _) => env("describe", 0, ",\"contract\":\"factoryos/hook@1\",\"role\":\"robot-controller\",\"runtime\":\"fake\",\"verbs\":[],\"versions\":{},\"ops\":[{\"name\":\"status\",\"args\":[],\"changes\":false}],\"bundle_schemas\":[\"plan2cell/krl-bundle@1\"],\"inbox\":\"/var/lib/factoryos/inbox\""),
                    ("ready", _) => env("ready", 0, ",\"ready\":true,\"checks\":[],\"info\":{\"ar\":\"none\"}"),
                    ("health", _) => env("health", 0, ",\"runtime\":{\"schema\":\"plan2cell/cell-runtime@1\",\"ok\":true,\"runtimes\":[]}"),
                    ("globals" | "op" | "prepare", _) => env(verb, 0, ""),
                    ("deploy" | "verify", _) => env(verb, 3, ",\"error\":\"no bundle\""),
                    ("watch", _) => Exec::exit(0, format!("{{\"schema\":\"factoryos/hook-watch@1\",\"kind\":\"robot\",\"t_ms\":1,\"component\":\"r\",\"state\":{{}},\"io\":{{\"in\":\"{0}\",\"out\":\"{0}\"}}}}\n{{\"schema\":\"factoryos/hook@1\",\"verb\":\"watch\",\"ok\":true,\"exit\":0,\"lines\":1,\"reason\":\"max_s\"}}\n", "0".repeat(64))),
                    (v, _) => env(v, 2, &format!(",\"error\":\"unimplemented: {v} for robot-controller\"")),
                },
                _ => Exec::exit(127, ""),
            })
        });
        let o = Options { profile: Some(Profile::None), pn_addresses: vec!["10.77.1.10".into()], ops: vec!["status".into()], ready_poll: Duration::ZERO, ..Options::new(Mode::Docker) };
        let r = run(&mut remote, "fake", &o, Some(env("describe", 0, ",\"contract\":\"factoryos/hook@1\"")));
        assert!(r.ok, "{}", r.text());
        assert_eq!(r.checks.iter().map(|c| c.id.as_str()).collect::<Vec<_>>(), ["HC01", "HC02", "HC03", "HC04", "HC05", "HC06", "HC07", "HC08", "HC09", "HC10", "HC11", "HC12"]);
        assert_eq!(r.exit_code(), 0);
        assert_eq!(r.role.as_deref(), Some("robot-controller"));
        // A template op the image lacks fails HC03.
        let o = Options { ops: vec!["status".into(), "home".into()], ..o };
        let r = run(&mut remote, "fake", &o, None);
        let hc03 = r.checks.iter().find(|c| c.id == "HC03").unwrap();
        assert!(!hc03.ok && hc03.detail.contains("lacks the template's home"), "{hc03:?}");
        assert_eq!(r.exit_code(), 7);
    }

    #[test]
    fn exec_mode_sends_nothing_that_changes_and_an_unreachable_pod_is_4() {
        let mut remote = MockRemote::default().on_exec(|argv, _| {
            Ok(match argv.get(1).map(String::as_str) {
                Some("describe") => Exec::exit(0, "{\"schema\":\"factoryos/hook@1\",\"verb\":\"describe\",\"ok\":true,\"exit\":0,\"contract\":\"factoryos/hook@1\",\"role\":\"head-plc\",\"runtime\":\"x\",\"verbs\":[],\"versions\":{}}\n"),
                _ => Exec::exit(0, ""),
            })
        });
        let r = run(&mut remote, "cell-jdoe/n1-head/head", &Options::new(Mode::Exec), None);
        let verbs: Vec<String> = remote
            .calls
            .iter()
            .filter_map(|c| match c {
                crate::remote::Call::Exec(a, _) if a[0] == HOOK => Some(a[1].clone()),
                _ => None,
            })
            .collect();
        for v in ["deploy", "start", "stop", "write", "prepare", "verify"] {
            assert!(!verbs.contains(&v.to_string()), "{v}: {verbs:?}");
        }
        assert!(r.checks.iter().any(|c| c.id == "HC09" && c.skipped));
        // A describe that names no role (or an unknown one) fails HC02, with or without --role,
        // so skipped verb checks never make a report conformant.
        for role in ["null", "\"plc\""] {
            for given in [None, Some(Role::Head)] {
                let mut nameless = MockRemote::default().on_exec(move |argv, _| {
                    Ok(match argv.get(1).map(String::as_str) {
                        Some("describe") => Exec::exit(0, format!("{{\"schema\":\"factoryos/hook@1\",\"verb\":\"describe\",\"ok\":true,\"exit\":0,\"contract\":\"factoryos/hook@1\",\"role\":{role},\"runtime\":\"x\",\"verbs\":[],\"versions\":{{}}}}\n")),
                        _ => Exec::exit(0, ""),
                    })
                });
                let o = Options { role: given, ..Options::new(Mode::Exec) };
                let r = run(&mut nameless, "cell-jdoe/n1-head/head", &o, None);
                let hc02 = r.checks.iter().find(|c| c.id == "HC02").unwrap();
                assert!(!hc02.ok && hc02.detail.contains(&format!("describe role {role}")), "{hc02:?}");
                assert!(!r.ok && r.exit_code() != 0, "{}", r.text());
            }
        }
        let mut dead = MockRemote::default().on_exec(|_, _| Ok(Exec { code: Some(1), stderr: b"error: unable to upgrade connection".to_vec(), ..Exec::default() }));
        let r = run(&mut dead, "cell-jdoe/gone/x", &Options::new(Mode::Exec), None);
        assert_eq!(r.exit_code(), 4, "{}", r.text());
    }
}
