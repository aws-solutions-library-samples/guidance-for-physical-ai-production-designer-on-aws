//! The client of `factoryos/hook@1` (docs/tasks/cell-templates.md E7; the handoff's §2.1-§2.5):
//! a head PLC or a robot controller that is a pod with `/opt/factoryos/hook`, driven over
//! [`Remote::exec`] (`kubectl exec -i <ns>/<pod>/<container> -- /opt/factoryos/hook <verb>`, the
//! workspace's own namespace Role; never a shell line).
//!
//! ```text
//! request   compact JSON on stdin, then closed (images refuse unknown fields with exit 2);
//!           bundles first, copied into /var/lib/factoryos/inbox ([`INBOX`]), the request names
//!           the file only
//! answer    `note: ` lines (at most 200, each cut to 1 KiB: the stage's notes), then the last
//!           non-empty line: {"schema":"factoryos/hook@1","verb":<verb>,"ok":ok,"exit":n,...},
//!           ok == (exit == 0), exit == kubectl's exit code; anything else counts as exit 4
//!           "hook answer missing or inconsistent"; more than 1 MiB of stdout: killed, exit 4
//! deadline  the hook's own + 30 s ([`Verb::deadline`]), then killed, exit 4
//! exit      0 ok; 2 -> stage 3 (the image does not implement what the template declares);
//!           3, 4, 7, 8 (build) as they are; anything else (kubectl 1, 126, 137, ...) -> 4
//!           ([`stage_exit`]); 4 is retried once after 5 s for the verbs that change nothing
//! ```
//!
//! The first contact with a pod in a session is `describe` ([`check_describe`]). The
//! `head-bundle@1` ([`head_bundle`]) and the deploy request's `wiring` ([`wiring`]) come from the
//! build and the per-scene cell.toml; `krl-bundle@1` is [`crate::krl::Bundle`] unchanged.

use std::collections::BTreeMap;
use std::time::Duration;

use serde_json::{Map, Value, json};

use crate::config::CellConfig;
use crate::project::{Manifest, Role as ProjectRole};
use crate::remote::{Exec, ExecLimits, Remote, base64};
use crate::stages::Tree;
use crate::{Error, Result};

pub const CONTRACT: &str = "factoryos/hook@1";
pub const WATCH_SCHEMA: &str = "factoryos/hook-watch@1";
pub const HEAD_BUNDLE_SCHEMA: &str = "factoryos/head-bundle@1";
/// The hook in every image.
pub const HOOK: &str = "/opt/factoryos/hook";
/// Where bundles are copied ([`Remote::copy`]: `<dest>.p2c-tmp`, then `mv`).
pub const INBOX: &str = "/var/lib/factoryos/inbox";
pub const HEAD_BUNDLE: &str = "head-bundle.json";
pub const KRL_BUNDLE: &str = "krl-bundle.json";
/// stdout read at most (more: killed, exit 4).
pub const MAX_STDOUT: usize = 1 << 20;
/// stderr kept (shown on failure only).
pub const MAX_STDERR: usize = 64 << 10;
/// `note: ` lines taken, and the length each is cut to.
pub const MAX_NOTES: usize = 200;
pub const MAX_NOTE: usize = 1024;
/// head-bundle@1: per file and in total (handoff §2.4).
pub const MAX_FILE: usize = 4 << 20;
pub const MAX_BUNDLE: usize = 32 << 20;
/// `read`: names per request.
pub const MAX_VARS: usize = 64;
/// The default pause before the one retry of a verb that changes nothing.
pub const RETRY_WAIT: Duration = Duration::from_secs(5);
pub const MISSING: &str = "hook answer missing or inconsistent";

/// The plan2cell virtual cell's PROFINET segment: the `wiring` of a cell.toml without
/// `[profinet]` (as deploy_check's defaults).
const HEAD_IP: &str = "192.168.100.10";
const NETMASK: &str = "255.255.255.0";
const GATEWAY: &str = "192.168.100.1";

/// The verbs of `factoryos/hook@1`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Verb {
    Describe,
    Ready,
    Health,
    Deploy,
    Verify,
    Prepare,
    Start,
    Stop,
    Read,
    Write,
    Op,
    Globals,
    Watch,
}

impl Verb {
    pub const ALL: [Verb; 13] = [
        Verb::Describe,
        Verb::Ready,
        Verb::Health,
        Verb::Deploy,
        Verb::Verify,
        Verb::Prepare,
        Verb::Start,
        Verb::Stop,
        Verb::Read,
        Verb::Write,
        Verb::Op,
        Verb::Globals,
        Verb::Watch,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Verb::Describe => "describe",
            Verb::Ready => "ready",
            Verb::Health => "health",
            Verb::Deploy => "deploy",
            Verb::Verify => "verify",
            Verb::Prepare => "prepare",
            Verb::Start => "start",
            Verb::Stop => "stop",
            Verb::Read => "read",
            Verb::Write => "write",
            Verb::Op => "op",
            Verb::Globals => "globals",
            Verb::Watch => "watch",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|v| v.name() == s)
    }

    /// The client's deadline: the hook's own (handoff §2.1) + 30 s; `watch` has none.
    pub fn deadline(self) -> Option<Duration> {
        Some(Duration::from_secs(match self {
            Verb::Deploy => 630,
            Verb::Start | Verb::Prepare => 150,
            Verb::Verify => 90,
            Verb::Ready => 5,
            Verb::Watch => return None,
            _ => 60,
        }))
    }

    /// Retried once on exit 4: the verbs that change nothing (`op` only for `status`, see
    /// [`retried`]).
    fn reads(self) -> bool {
        matches!(
            self,
            Verb::Describe | Verb::Ready | Verb::Read | Verb::Health | Verb::Globals
        )
    }

    pub fn limits(self) -> ExecLimits {
        ExecLimits {
            timeout: self.deadline(),
            max_stdout: MAX_STDOUT,
            max_stderr: MAX_STDERR,
        }
    }
}

/// Whether a call is retried once on exit 4 (E7.1: ready, read, health, globals, op status and
/// describe; never a changing verb).
pub fn retried(verb: Verb, request: &Value) -> bool {
    verb.reads() || (verb == Verb::Op && request["name"] == "status")
}

/// The role of a hook pod (`describe.role`, the template component's `semantic`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Role {
    Head,
    Robot,
}

impl Role {
    pub fn semantic(self) -> &'static str {
        match self {
            Role::Head => "head-plc",
            Role::Robot => "robot-controller",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "head" | "head-plc" => Some(Role::Head),
            "robot" | "robot-controller" => Some(Role::Robot),
            _ => None,
        }
    }

    /// The verbs the role implements (handoff §2.1; the head's optional `verify` not included).
    pub fn verbs(self) -> &'static [Verb] {
        match self {
            Role::Head => &[
                Verb::Describe,
                Verb::Ready,
                Verb::Health,
                Verb::Deploy,
                Verb::Start,
                Verb::Stop,
                Verb::Read,
                Verb::Write,
                Verb::Watch,
            ],
            Role::Robot => &[
                Verb::Describe,
                Verb::Ready,
                Verb::Health,
                Verb::Deploy,
                Verb::Verify,
                Verb::Prepare,
                Verb::Op,
                Verb::Globals,
                Verb::Watch,
            ],
        }
    }
}

/// The hook's exit code as the plan2cell stage's (lib.rs numbering; handoff E9: hook 8 is a build
/// error, hook 4 transient, the p2c-kvc mapping 4 -> 8 is not reused).
pub fn stage_exit(hook: i32) -> i32 {
    match hook {
        0 => 0,
        2 | 3 => 3,
        4 => 4,
        7 => 7,
        8 => 8,
        _ => 4,
    }
}

/// One answer of the hook.
#[derive(Clone, Debug, PartialEq)]
pub struct Answer {
    pub verb: Verb,
    /// The hook's exit code; 4 when the answer does not count ([`Answer::problem`]).
    pub exit: i32,
    /// The envelope (the last line); `Value::Null` without one.
    pub json: Value,
    /// The `note: ` lines (without the prefix).
    pub notes: Vec<String>,
    /// Why the answer does not count: missing, inconsistent, killed.
    pub problem: Option<String>,
    pub stderr: String,
}

impl Answer {
    /// What a dry run "answers": nothing was sent.
    pub fn dry(verb: Verb) -> Self {
        Self {
            verb,
            exit: 0,
            json: json!({}),
            notes: Vec::new(),
            problem: None,
            stderr: String::new(),
        }
    }

    pub fn ok(&self) -> bool {
        self.exit == 0
    }

    pub fn stage_exit(&self) -> i32 {
        stage_exit(self.exit)
    }

    /// Why it failed: the answer's `error`, else the problem, else stderr's last line, else the
    /// exit code.
    pub fn error(&self) -> String {
        if let Some(p) = &self.problem {
            let tail = last_line(&self.stderr);
            return if tail.is_empty() { p.clone() } else { format!("{p} (stderr: {tail})") };
        }
        match self.json["error"].as_str() {
            Some(e) if !e.is_empty() => e.to_string(),
            _ => match last_line(&self.stderr) {
                t if !t.is_empty() => t,
                _ => format!("exit {}", self.exit),
            },
        }
    }

    /// The message of a failed answer as a stage's error: exit 2 says the image does not
    /// implement what the template declares.
    pub fn failure(&self, who: &str) -> String {
        let verb = self.verb.name();
        match self.exit {
            2 => format!(
                "{who}: hook {verb}: unimplemented or bad request: the image does not implement what \
                 the template declares ({})",
                self.error()
            ),
            n if self.problem.is_some() => format!("{who}: hook {verb}: {} (exit {n})", self.error()),
            n => format!("{who}: hook {verb}: exit {n}: {}", self.error()),
        }
    }
}

fn last_line(s: &str) -> String {
    s.lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("").trim().to_string()
}

/// Cut `s` to at most `max` bytes at a character boundary.
fn cut(s: &str, max: usize) -> String {
    if s.len() <= max {
        return s.to_string();
    }
    let mut n = max;
    while !s.is_char_boundary(n) {
        n -= 1;
    }
    s[..n].to_string()
}

/// The answer of `verb` in what the program printed and its exit code (the envelope rules of
/// E7.1).
pub fn parse(verb: Verb, e: &Exec) -> Answer {
    let stderr = cut(&e.stderr_text(), MAX_STDERR);
    let text = String::from_utf8_lossy(&e.stdout);
    let notes: Vec<String> = text
        .lines()
        .filter_map(|l| l.strip_prefix("note: "))
        .take(MAX_NOTES)
        .map(|n| cut(n.trim_end_matches('\r'), MAX_NOTE))
        .collect();
    let bad = |problem: String, json: Value| Answer {
        verb,
        exit: 4,
        json,
        notes: notes.clone(),
        problem: Some(problem),
        stderr: stderr.clone(),
    };
    if e.overflow {
        return bad(format!("{MISSING}: more than {MAX_STDOUT} bytes of output"), Value::Null);
    }
    if e.timed_out {
        let s = verb.deadline().map_or(0, |d| d.as_secs());
        return bad(format!("{MISSING}: no answer within {s} s"), Value::Null);
    }
    let Some(code) = e.code else {
        return bad(format!("{MISSING}: the program was killed"), Value::Null);
    };
    let last = text.lines().rev().map(str::trim).find(|l| !l.is_empty());
    let json = match last.map(serde_json::from_str::<Value>) {
        Some(Ok(v @ Value::Object(_))) => v,
        _ => return bad(format!("{MISSING} (exit {code})"), Value::Null),
    };
    let ok = json["ok"].as_bool();
    let exit = json["exit"].as_i64();
    let why = if json["schema"] != CONTRACT {
        Some(format!("schema {}", json["schema"]))
    } else if json["verb"] != verb.name() {
        Some(format!("verb {}, asked {}", json["verb"], verb.name()))
    } else if ok.is_none() || exit.is_none() {
        Some("no ok / exit".to_string())
    } else if ok != Some(exit == Some(0)) {
        Some(format!("ok {} with exit {}", json["ok"], json["exit"]))
    } else if exit != Some(i64::from(code)) {
        Some(format!("exit {} in the answer, {code} from the program", json["exit"]))
    } else {
        None
    };
    if let Some(w) = why {
        return bad(format!("{MISSING}: {w}"), json);
    }
    Answer {
        verb,
        exit: code,
        json,
        notes,
        problem: None,
        stderr,
    }
}

/// The request as the hook reads it: compact JSON.
pub fn request_text(request: &Value) -> String {
    serde_json::to_string(request).expect("JSON serialises")
}

/// `hook <verb>` with `request` on stdin, on `remote`; the answer (also a failed one). With
/// `retry`, an exit 4 is tried once more after that pause. `Err` only when the program could not
/// be run (the remote cannot exec, kubectl missing).
pub fn call(
    remote: &mut dyn Remote,
    verb: Verb,
    request: &Value,
    retry: Option<Duration>,
) -> Result<Answer> {
    let text = request_text(request);
    let argv = [HOOK, verb.name()];
    let mut a = parse(verb, &remote.exec(&argv, text.as_bytes(), &verb.limits())?);
    if a.exit == 4
        && let Some(w) = retry
    {
        std::thread::sleep(w);
        a = parse(verb, &remote.exec(&argv, text.as_bytes(), &verb.limits())?);
    }
    Ok(a)
}

/// [`call`] with the retry rule of E7.1 ([`retried`]).
pub fn call_retried(
    remote: &mut dyn Remote,
    verb: Verb,
    request: &Value,
    wait: Duration,
) -> Result<Answer> {
    call(remote, verb, request, retried(verb, request).then_some(wait))
}

/// `describe` answered for `role`: exit 0, `contract` `factoryos/hook@1`, `role` the component's
/// semantic; else why not (stage exit 3).
pub fn check_describe(a: &Answer, role: Role) -> std::result::Result<(), String> {
    if !a.ok() {
        return Err(a.failure("describe"));
    }
    if a.json["contract"] != CONTRACT {
        return Err(format!(
            "describe: contract {}, want {CONTRACT}",
            a.json["contract"]
        ));
    }
    if a.json["role"] != role.semantic() {
        return Err(format!(
            "describe: role {}, the cell drives it as {}",
            a.json["role"],
            role.semantic()
        ));
    }
    Ok(())
}

/// `describe` then `verb` (one session's first contact, E7.2): the answer of `verb`; a failed
/// describe is `Err` (exit 3, or 4 when it did not answer).
pub fn described_call(
    remote: &mut dyn Remote,
    role: Role,
    verb: Verb,
    request: &Value,
    wait: Duration,
) -> Result<Answer> {
    let d = call_retried(remote, Verb::Describe, &json!({}), wait)?;
    if let Err(m) = check_describe(&d, role) {
        let code = if d.ok() { 3 } else { d.stage_exit() };
        return Err(Error::Failed { stage: "hook", code, msg: m });
    }
    if verb == Verb::Describe {
        return Ok(d);
    }
    call_retried(remote, verb, request, wait)
}

/// `read`'s values (text, handoff §2.3) and the names it could not read.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Values {
    pub values: BTreeMap<String, String>,
    pub missing: Vec<String>,
}

pub fn values(a: &Answer) -> Values {
    let text = |v: &Value| match v {
        Value::String(s) => s.clone(),
        other => other.to_string(),
    };
    let field = |k: &str| -> BTreeMap<String, String> {
        a.json[k]
            .as_object()
            .map(|m| m.iter().map(|(k, v)| (k.clone(), text(v))).collect())
            .unwrap_or_default()
    };
    Values {
        values: field("values"),
        missing: crate::krl::strings(&a.json["missing"]),
    }
}

/// `read {"vars": [...]}`.
pub fn read_request(names: &[&str]) -> Value {
    json!({ "vars": names })
}

/// `write {"vars": {name: value}}` (a pulse: one write of `true`).
pub fn write_request(vars: &[(&str, bool)]) -> Value {
    let m: Map<String, Value> = vars.iter().map(|(k, v)| (k.to_string(), Value::Bool(*v))).collect();
    json!({ "vars": m })
}

/// `op {"name": <op>, "arg": <arg>}` (`arg` omitted when none).
pub fn op_request(name: &str, arg: Option<&str>) -> Value {
    match arg {
        Some(a) => json!({"name": name, "arg": a}),
        None => json!({"name": name}),
    }
}

/// The deploy request's `wiring` (handoff §2.4, E7.3), from the per-scene cell.toml:
/// `[profinet]` (unset: the plan2cell virtual cell's) and every `[[kuka_vc.controller]]` in order;
/// no other fields.
pub fn wiring(cfg: &CellConfig) -> Result<Value> {
    let p = cfg.profinet.clone().unwrap_or_default();
    let mut controllers = Vec::new();
    for c in cfg.controllers() {
        let (Some(station), Some(kli)) = (&c.station, &c.kli_ip) else {
            return Err(Error::Config(format!(
                "kuka_vc.controller robot_ns = \"{}\": the hook head's wiring needs its station and kli_ip",
                c.robot_ns
            )));
        };
        controllers.push(json!({"robot_ns": c.robot_ns, "station": station, "kli_ip": kli}));
    }
    Ok(json!({
        "head_ip": p.head_ip.as_deref().unwrap_or(HEAD_IP),
        "netmask": p.netmask.as_deref().unwrap_or(NETMASK),
        "gateway": p.gateway.as_deref().unwrap_or(GATEWAY),
        "controllers": controllers,
    }))
}

/// A `factoryos/head-bundle@1` of a build (E7.3).
#[derive(Clone, Debug)]
pub struct HeadBundle {
    pub json: String,
    pub plan_id: String,
    pub key: String,
    /// The bundle's file names, sorted.
    pub files: Vec<String>,
}

fn plan_id_ok(s: &str) -> bool {
    s.strip_prefix("16#")
        .is_some_and(|h| h.len() == 4 && h.chars().all(|c| c.is_ascii_digit() || ('A'..='F').contains(&c)))
}

fn st_name_ok(s: &str) -> bool {
    let mut c = s.chars();
    s.len() <= 64
        && c.next().is_some_and(|f| f.is_ascii_alphabetic() || f == '_')
        && c.all(|x| x.is_ascii_alphanumeric() || x == '_')
}

/// The head bundle of the build in `tree` (bt2plc's `plc/`: `plc/project.json`,
/// `plc/cell_plan.json`, every `plc/trust/<key>/src/<Name>.st`, and `plan2cell-build.json` when
/// present; never `wiring.json`, the request carries it). Anything missing or too big: stage exit
/// 3 (before any copy).
pub fn head_bundle(tree: &Tree, m: &Manifest) -> Result<HeadBundle> {
    let bad = |msg: String| Error::Failed { stage: "deploy", code: 3, msg };
    let head = m
        .projects
        .iter()
        .find(|p| p.role == ProjectRole::HeadPlc)
        .ok_or_else(|| bad("plc/project.json has no head_plc project".into()))?;
    let key = head.key.clone();
    let plan_id = m.plan.plan_id.clone();
    if !plan_id_ok(&plan_id) {
        return Err(bad(format!(
            "plc/project.json plan.plan_id {plan_id}: a head-bundle@1 needs 16#XXXX (upper-case hex)"
        )));
    }
    let src = format!("plc/trust/{key}/src");
    let mut rels = vec!["plc/project.json".to_string(), "plc/cell_plan.json".to_string()];
    let mut sts = Vec::new();
    for f in tree.list(&src)? {
        let Some(name) = f.strip_suffix(".st") else { continue };
        if !st_name_ok(name) {
            return Err(bad(format!(
                "{src}/{f}: a head-bundle@1 takes <Name>.st with Name ^[A-Za-z_][A-Za-z0-9_]{{0,63}}$"
            )));
        }
        sts.push(format!("{src}/{f}"));
    }
    if sts.is_empty() {
        return Err(bad(format!(
            "the build has no {src}/*.st (bt2plc writes the head's structured text there)"
        )));
    }
    rels.extend(sts);
    if tree.exists("plan2cell-build.json") {
        rels.push("plan2cell-build.json".into());
    }
    let mut files = BTreeMap::new();
    let mut total = 0usize;
    for r in &rels {
        let text = tree.read(r).map_err(|e| bad(format!("{r}: {e}")))?;
        if text.len() > MAX_FILE {
            return Err(bad(format!("{r}: {} bytes, a head-bundle@1 file has at most {MAX_FILE}", text.len())));
        }
        total += text.len();
        files.insert(r.clone(), base64(text.as_bytes()));
    }
    if total > MAX_BUNDLE {
        return Err(bad(format!("the head bundle has {total} bytes, at most {MAX_BUNDLE}")));
    }
    let json = json!({
        "schema": HEAD_BUNDLE_SCHEMA,
        "plan_id": plan_id,
        "key": key,
        "files": files,
    })
    .to_string();
    let mut names: Vec<String> = files.keys().cloned().collect();
    names.sort();
    Ok(HeadBundle { json, plan_id, key, files: names })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::remote::MockRemote;
    use std::path::Path;

    fn envelope(verb: &str, exit: i32, extra: &str) -> String {
        format!(
            "{{\"schema\":\"factoryos/hook@1\",\"verb\":\"{verb}\",\"ok\":{},\"exit\":{exit}{extra}}}",
            exit == 0
        )
    }

    #[test]
    fn the_envelope_rules() {
        let ok = Exec::exit(0, format!("note: one\nother line\nnote: two\n{}\n\n", envelope("ready", 0, ",\"ready\":true")));
        let a = parse(Verb::Ready, &ok);
        assert_eq!((a.exit, a.problem.as_deref()), (0, None));
        assert_eq!(a.notes, ["one", "two"]);
        assert_eq!(a.json["ready"], true);
        // Missing, not JSON, another verb, ok != (exit == 0), exit != the program's.
        for (out, code) in [
            (String::new(), 0),
            ("not json\n".to_string(), 0),
            (envelope("health", 0, ""), 0),
            (envelope("ready", 0, "").replace("\"ok\":true", "\"ok\":false"), 0),
            (envelope("ready", 3, ""), 0),
            (envelope("ready", 0, "").replace("factoryos/hook@1", "factoryos/hook@2"), 0),
            ("[1,2]".to_string(), 0),
        ] {
            let a = parse(Verb::Ready, &Exec::exit(code, out.clone()));
            assert_eq!(a.exit, 4, "{out}");
            assert!(a.problem.as_deref().unwrap().starts_with(MISSING), "{out}");
        }
        // A failed answer counts with its own exit code.
        let a = parse(Verb::Deploy, &Exec::exit(8, envelope("deploy", 8, ",\"error\":\"1 compile error(s)\"")));
        assert_eq!((a.exit, a.stage_exit(), a.error()), (8, 8, "1 compile error(s)".to_string()));
        // Killed: too much output, the deadline, a signal.
        let over = Exec { overflow: true, ..Exec::exit(0, envelope("read", 0, "")) };
        assert_eq!(parse(Verb::Read, &over).exit, 4);
        let late = Exec { timed_out: true, code: None, ..Exec::default() };
        assert!(parse(Verb::Deploy, &late).error().contains("no answer within 630 s"));
        assert_eq!(parse(Verb::Read, &Exec::default()).exit, 4);
    }

    #[test]
    fn notes_are_bounded() {
        let mut out = String::new();
        for i in 0..250 {
            out.push_str(&format!("note: {i} {}\n", "x".repeat(2000)));
        }
        out.push_str(&envelope("health", 0, ""));
        let a = parse(Verb::Health, &Exec::exit(0, out));
        assert_eq!(a.notes.len(), MAX_NOTES);
        assert!(a.notes.iter().all(|n| n.len() <= MAX_NOTE));
    }

    #[test]
    fn exit_mapping() {
        assert_eq!(
            [0, 1, 2, 3, 4, 7, 8, 126, 127, 137, -1].map(stage_exit),
            [0, 4, 3, 3, 4, 7, 8, 4, 4, 4, 4]
        );
        let a = parse(Verb::Prepare, &Exec::exit(2, envelope("prepare", 2, ",\"error\":\"unimplemented: prepare for head-plc\"")));
        assert_eq!(a.stage_exit(), 3);
        assert!(a.failure("head").contains("the image does not implement what the template declares"));
    }

    #[test]
    fn deadlines_and_retries() {
        assert_eq!(Verb::Deploy.deadline(), Some(Duration::from_secs(630)));
        assert_eq!(Verb::Start.deadline(), Some(Duration::from_secs(150)));
        assert_eq!(Verb::Prepare.deadline(), Some(Duration::from_secs(150)));
        assert_eq!(Verb::Verify.deadline(), Some(Duration::from_secs(90)));
        assert_eq!(Verb::Ready.deadline(), Some(Duration::from_secs(5)));
        assert_eq!(Verb::Globals.deadline(), Some(Duration::from_secs(60)));
        assert_eq!(Verb::Watch.deadline(), None);
        assert!(retried(Verb::Read, &json!({})) && retried(Verb::Describe, &json!({})));
        assert!(retried(Verb::Op, &op_request("status", None)));
        assert!(!retried(Verb::Op, &op_request("mode", Some("EXT"))));
        for v in [Verb::Deploy, Verb::Verify, Verb::Prepare, Verb::Start, Verb::Stop, Verb::Write] {
            assert!(!retried(v, &json!({})), "{v:?}");
        }
    }

    #[test]
    fn a_transient_answer_is_retried_once_only_for_reading_verbs() {
        let mut n = 0;
        let mut r = MockRemote::default().on_exec(move |argv, stdin| {
            n += 1;
            assert_eq!(argv, [HOOK, "read"]);
            assert_eq!(stdin, br#"{"vars":["PLC_PRG.eCellState"]}"#);
            Ok(if n == 1 { Exec::exit(4, envelope("read", 4, ",\"error\":\"busy\"")) } else { Exec::exit(0, envelope("read", 0, ",\"values\":{}")) })
        });
        let a = call_retried(&mut r, Verb::Read, &read_request(&["PLC_PRG.eCellState"]), Duration::ZERO).unwrap();
        assert_eq!(a.exit, 0);
        assert_eq!(r.calls.len(), 2);
        let mut r = MockRemote::default().on_exec(|_, _| Ok(Exec::exit(4, envelope("write", 4, ""))));
        let a = call_retried(&mut r, Verb::Write, &write_request(&[("PLC_PRG.bReset", true)]), Duration::ZERO).unwrap();
        assert_eq!((a.exit, r.calls.len()), (4, 1));
    }

    #[test]
    fn describe_is_checked_for_contract_and_role() {
        let d = |role: &str| parse(Verb::Describe, &Exec::exit(0, envelope("describe", 0, &format!(",\"contract\":\"factoryos/hook@1\",\"role\":\"{role}\""))));
        check_describe(&d("head-plc"), Role::Head).unwrap();
        assert!(check_describe(&d("robot-controller"), Role::Head).unwrap_err().contains("role"));
        let other = parse(Verb::Describe, &Exec::exit(0, envelope("describe", 0, ",\"contract\":\"x\",\"role\":\"head-plc\"")));
        assert!(check_describe(&other, Role::Head).unwrap_err().contains("contract"));
        let mut r = MockRemote::default().on_exec(|argv, _| {
            Ok(match argv[1].as_str() {
                "describe" => Exec::exit(0, envelope("describe", 0, ",\"contract\":\"factoryos/hook@1\",\"role\":\"robot-controller\"")),
                v => Exec::exit(0, envelope(v, 0, "")),
            })
        });
        let e = described_call(&mut r, Role::Head, Verb::Health, &json!({}), Duration::ZERO).unwrap_err();
        assert_eq!(e.exit_code(), 3);
        assert!(described_call(&mut r, Role::Robot, Verb::Globals, &json!({}), Duration::ZERO).unwrap().ok());
    }

    #[test]
    fn requests_are_compact_and_exact() {
        assert_eq!(request_text(&op_request("mode", Some("EXT"))), r#"{"arg":"EXT","name":"mode"}"#);
        assert_eq!(request_text(&op_request("ack", None)), r#"{"name":"ack"}"#);
        assert_eq!(request_text(&write_request(&[("PLC_PRG.bStop", true)])), r#"{"vars":{"PLC_PRG.bStop":true}}"#);
        let a = parse(
            Verb::Read,
            &Exec::exit(0, envelope("read", 0, ",\"values\":{\"PLC_PRG.eCellState\":\"E_CellState.CS_RUNNING\",\"PLC_PRG.udiCycles\":7},\"missing\":[\"PLC_PRG.fbCell.nErrorRobot\"]")),
        );
        let v = values(&a);
        assert_eq!(v.values["PLC_PRG.eCellState"], "E_CellState.CS_RUNNING");
        assert_eq!(v.values["PLC_PRG.udiCycles"], "7");
        assert_eq!(v.missing, ["PLC_PRG.fbCell.nErrorRobot"]);
    }

    #[test]
    fn the_wiring_comes_from_cell_toml_only() {
        let c = CellConfig::parse(
            "[plc_host]\ndriver = \"hook\"\nkubectl_pod = \"cell-jdoe/n1-head/head\"\n\
             [[kuka_vc.controller]]\ndriver = \"hook\"\nrobot_ns = \"robot1\"\nstation = \"kuka-robi1\"\nkli_ip = \"10.77.1.11\"\nkubectl_pod = \"cell-jdoe/n1-robot-1/robot\"\n\
             [profinet]\nhead_ip = \"10.77.1.10\"\nnetmask = \"255.255.255.0\"\ngateway = \"10.77.1.1\"\n",
        )
        .unwrap();
        assert_eq!(
            request_text(&wiring(&c).unwrap()),
            r#"{"controllers":[{"kli_ip":"10.77.1.11","robot_ns":"robot1","station":"kuka-robi1"}],"gateway":"10.77.1.1","head_ip":"10.77.1.10","netmask":"255.255.255.0"}"#
        );
        let mut no_station = c.clone();
        no_station.kuka_vc.as_mut().unwrap().controllers[0].station = None;
        assert_eq!(wiring(&no_station).unwrap_err().exit_code(), 3);
        let bare = CellConfig::default();
        assert_eq!(wiring(&bare).unwrap()["head_ip"], HEAD_IP);
    }

    fn example() -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/welding_cell_2_small")
    }

    #[test]
    fn the_head_bundle_of_the_example() {
        let tree = Tree::Dir(example());
        let m = Manifest::parse(&tree.read("plc/project.json").unwrap()).unwrap();
        let b = head_bundle(&tree, &m).unwrap();
        assert_eq!((b.key.as_str(), b.plan_id.as_str()), ("head", "16#F77E"));
        let v: Value = serde_json::from_str(&b.json).unwrap();
        assert_eq!(v["schema"], HEAD_BUNDLE_SCHEMA);
        let files = v["files"].as_object().unwrap();
        assert!(files.contains_key("plc/project.json") && files.contains_key("plc/cell_plan.json"));
        assert!(files.contains_key("plc/trust/head/src/PLC_PRG.st"), "{:?}", b.files);
        assert!(files.contains_key("plan2cell-build.json"));
        assert!(!files.contains_key("wiring.json"));
        assert!(files.keys().all(|k| !k.starts_with("plc/trust/robot")));
        assert_eq!(
            crate::remote::base64_decode(files["plc/project.json"].as_str().unwrap()).unwrap(),
            tree.read("plc/project.json").unwrap().as_bytes()
        );
    }

    #[test]
    fn a_head_bundle_without_structured_text_or_too_big_is_exit_3() {
        let tree = Tree::Dir(example());
        let m = Manifest::parse(&tree.read("plc/project.json").unwrap()).unwrap();
        let mut files: BTreeMap<String, String> = ["plc/project.json", "plc/cell_plan.json"]
            .iter()
            .map(|r| (r.to_string(), tree.read(r).unwrap()))
            .collect();
        let mem = |files: &BTreeMap<String, String>| Tree::Mem { dir: example(), files: files.clone() };
        let e = head_bundle(&mem(&files), &m).unwrap_err();
        assert_eq!(e.exit_code(), 3);
        assert!(e.to_string().contains("plc/trust/head/src"), "{e}");
        files.insert("plc/trust/head/src/PLC_PRG.st".into(), "x".repeat(MAX_FILE + 1));
        assert!(head_bundle(&mem(&files), &m).unwrap_err().to_string().contains("at most"));
        files.insert("plc/trust/head/src/PLC_PRG.st".into(), "PROGRAM PLC_PRG END_PROGRAM".into());
        files.insert("plc/trust/head/src/bad-name.st".into(), "x".into());
        assert!(head_bundle(&mem(&files), &m).unwrap_err().to_string().contains("bad-name.st"));
        files.remove("plc/trust/head/src/bad-name.st");
        files.insert("plc/trust/head/src/README.md".into(), "not st".into());
        let b = head_bundle(&mem(&files), &m).unwrap();
        assert_eq!(b.files, ["plc/cell_plan.json", "plc/project.json", "plc/trust/head/src/PLC_PRG.st"]);
        assert!(!plan_id_ok("16#f77e") && plan_id_ok("16#F77E") && !plan_id_ok("63358"));
    }
}
