//! One operation on one KUKA virtual controller (`plan2cell cell kvc <robot> <op> [arg]`), over the
//! controller's own remote of cell.toml (SSM, ssh or `kubectl exec`), with the host helpers of
//! `infra/kuka-vc` the bring-up installed:
//!
//! | op               | on the host                                         | changes the cell |
//! |------------------|-----------------------------------------------------|------------------|
//! | `status`         | `p2c-kvc.py status`: state, mode, program, messages | no               |
//! | `mode show`      | `kvc-mode.sh N show`                                | no               |
//! | `mode T1|T2|AUT|EXT` | `kvc-mode.sh N <mode>` (through the smartPAD)    | yes              |
//! | `ack`            | `kvc-ack.sh --netns krcN`: stop messages, then EXT  | yes              |
//! | `restart`        | `kvc-restart.sh` (controller 1 of its host)         | yes              |
//! | `program show`   | `p2c-kvc.py program show`                           | no               |
//! | `program cancel` | `p2c-kvc.py program cancel` (only BT_CELL)          | yes              |
//! | `program select` | `p2c-kvc.py program select [--module M]`            | yes              |
//! | `braketest-need` | `p2c-kvc.py braketest-need`                         | no               |
//!
//! Only these, with their arguments checked: no free command reaches a host.
//!
//! A hook controller (`driver = "hook"`) gets the same operation as `hook op` with
//! `{"name": <op>, "arg": <arg>}` on stdin ([`KvcOp::hook_name_arg`]); whether it changes the cell
//! is [`KvcOp::changes`] (the caller's, from the template), never the answer.

use serde_json::{Value, json};

use super::{Cell, sh};
use crate::{Error, Result};

/// One allowed operation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum KvcOp {
    Status,
    ModeShow,
    Mode(String),
    Ack,
    Restart,
    ProgramShow,
    ProgramCancel,
    ProgramSelect(Option<String>),
    BraketestNeed,
}

const MODES: [&str; 4] = ["T1", "T2", "AUT", "EXT"];

impl KvcOp {
    /// `op` and its argument, as the command line has them.
    pub fn parse(op: &str, arg: Option<&str>) -> Result<Self> {
        let usage = || {
            Error::Usage(format!(
                "kvc {op} {}: one of status, mode show|T1|T2|AUT|EXT, ack, restart, program show|cancel|select [MODULE], braketest-need",
                arg.unwrap_or_default()
            ))
        };
        Ok(match (op, arg) {
            ("status", None) => Self::Status,
            ("mode", Some("show")) => Self::ModeShow,
            ("mode", Some(m)) if MODES.contains(&m.to_ascii_uppercase().as_str()) => Self::Mode(m.to_ascii_uppercase()),
            ("ack", None) => Self::Ack,
            ("restart", None) => Self::Restart,
            ("program", Some("show")) => Self::ProgramShow,
            ("program", Some("cancel")) => Self::ProgramCancel,
            ("program", Some("select")) => Self::ProgramSelect(None),
            ("program", Some(m)) if m.starts_with("select=") => {
                let module = &m["select=".len()..];
                let ok = !module.is_empty()
                    && module.len() <= 128
                    && module.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | ':' | '\\'));
                if !ok {
                    return Err(Error::Usage(format!("program select={module}: a KRL module path like KRC:\\R1\\Program\\BT_CELL")));
                }
                Self::ProgramSelect(Some(module.to_owned()))
            }
            ("braketest-need", None) => Self::BraketestNeed,
            _ => return Err(usage()),
        })
    }

    /// Whether it changes the controller (a caller asks the user first).
    pub fn changes(&self) -> bool {
        !matches!(self, Self::Status | Self::ModeShow | Self::ProgramShow | Self::BraketestNeed)
    }

    /// The `op` request's name and argument (handoff §2.2: the command line's).
    pub fn hook_name_arg(&self) -> (&'static str, Option<String>) {
        match self {
            Self::Status => ("status", None),
            Self::ModeShow => ("mode", Some("show".into())),
            Self::Mode(m) => ("mode", Some(m.clone())),
            Self::Ack => ("ack", None),
            Self::Restart => ("restart", None),
            Self::ProgramShow => ("program", Some("show".into())),
            Self::ProgramCancel => ("program", Some("cancel".into())),
            Self::ProgramSelect(None) => ("program", Some("select".into())),
            Self::ProgramSelect(Some(m)) => ("program", Some(format!("select={m}"))),
            Self::BraketestNeed => ("braketest-need", None),
        }
    }
}

/// `krcN` -> N.
fn number(netns: &str) -> Option<u32> {
    netns.strip_prefix("krc")?.parse().ok().filter(|n| *n >= 1)
}

/// What the host printed: the exit code of `; echo p2c-exit=$?`, the last JSON object line, the rest.
fn parse_output(out: &str) -> (Option<i32>, Option<Value>, Vec<String>) {
    let mut code = None;
    let mut json = None;
    let mut lines = Vec::new();
    for l in out.lines().map(str::trim).filter(|l| !l.is_empty()) {
        if let Some(c) = l.strip_prefix("p2c-exit=") {
            code = c.trim().parse().ok();
        } else if l.starts_with('{') && let Ok(v) = serde_json::from_str::<Value>(l) {
            json = Some(v);
        } else {
            lines.push(l.to_owned());
        }
    }
    (code, json, lines)
}

impl Cell<'_> {
    /// Run `op` on controller `robot_ns` of `[kuka_vc]`: `{robot_ns, op, exit, ok, result, output}`.
    pub fn kvc_op(&mut self, robot_ns: &str, op: &KvcOp) -> Result<Value> {
        let k = self.vc_config()?;
        let c = k
            .controllers
            .iter()
            .find(|c| c.robot_ns == robot_ns)
            .cloned()
            .ok_or_else(|| {
                let all: Vec<&str> = k.controllers.iter().map(|c| c.robot_ns.as_str()).collect();
                Error::Config(format!("cell.toml has no KUKA VC controller for {robot_ns} (controllers: {all:?})"))
            })?;
        if c.is_hook() {
            return self.kvc_op_hook(&c, op);
        }
        let dir = k.dir.trim_end_matches('/');
        let n = number(&c.netns).ok_or_else(|| Error::Config(format!("kuka_vc.controller {robot_ns}: netns {} is not krcN", c.netns)))?;
        let helper = format!("sudo -n python3 -I {}", sh(&format!("{dir}/p2c-kvc.py")));
        let netns = sh(&c.netns);
        let script = |name: &str| sh(&format!("{dir}/{name}"));
        let cmd = match op {
            KvcOp::Status => format!("{helper} status --netns {netns} --instance {}", sh(&c.instance)),
            KvcOp::ModeShow => format!("sudo -n {} {n} show", script("kvc-mode.sh")),
            KvcOp::Mode(m) => format!("sudo -n {} {n} {m}", script("kvc-mode.sh")),
            KvcOp::Ack => format!("sudo -n {} --netns {netns}", script("kvc-ack.sh")),
            KvcOp::Restart if n != 1 => {
                return Err(Error::Config(format!("{robot_ns} is controller {n} of its host: kvc-restart.sh restarts controller 1 only")));
            }
            KvcOp::Restart => format!("sudo -n {}", script("kvc-restart.sh")),
            KvcOp::ProgramShow => format!("{helper} program show --netns {netns}"),
            KvcOp::ProgramCancel => format!("{helper} program cancel --netns {netns}"),
            KvcOp::ProgramSelect(None) => format!("{helper} program select --netns {netns}"),
            KvcOp::ProgramSelect(Some(m)) => format!("{helper} program select --netns {netns} --module {}", sh(m)),
            KvcOp::BraketestNeed => format!("{helper} braketest-need --netns {netns}"),
        };
        let out = self.vc_run(&c, &format!("{cmd} 2>&1; echo p2c-exit=$?"))?;
        let (exit, result, output) = parse_output(&out);
        let ok = exit == Some(0) && result.as_ref().and_then(|r| r.get("ok")).and_then(Value::as_bool) != Some(false);
        Ok(json!({
            "robot_ns": robot_ns,
            "controller": n,
            "op": format!("{op:?}"),
            "changes": op.changes(),
            "exit": exit,
            "ok": ok || self.settings.dry_run,
            "result": result,
            "output": output.iter().rev().take(40).rev().collect::<Vec<_>>(),
            "calls": self.report.calls.clone(),
        }))
    }

    /// [`Cell::kvc_op`] on a hook controller: `hook op` (`controller` is `null`, `output` the
    /// answer's notes).
    fn kvc_op_hook(&mut self, c: &crate::config::KukaController, op: &KvcOp) -> Result<Value> {
        let (name, arg) = op.hook_name_arg();
        let who = super::hooks::Hooked::Robot(c.clone());
        let a = self.hook(
            super::Stage::Start,
            &who,
            crate::hook::Verb::Op,
            &crate::hook::op_request(name, arg.as_deref()),
        )?;
        let dry = self.settings.dry_run;
        Ok(json!({
            "robot_ns": c.robot_ns,
            "controller": Value::Null,
            "op": format!("{op:?}"),
            "changes": op.changes(),
            "exit": if dry { Value::Null } else { json!(a.exit) },
            "ok": a.ok() || dry,
            "result": if dry { Value::Null } else { a.json.clone() },
            "output": a.notes.iter().rev().take(40).rev().collect::<Vec<_>>(),
            "error": (!a.ok()).then(|| a.failure(&c.robot_ns)),
            "calls": self.report.calls.clone(),
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_the_listed_operations_parse() {
        assert_eq!(KvcOp::parse("mode", Some("ext")).unwrap(), KvcOp::Mode("EXT".into()));
        assert_eq!(KvcOp::parse("mode", Some("show")).unwrap(), KvcOp::ModeShow);
        assert!(KvcOp::parse("mode", Some("T1; rm -rf /")).is_err());
        assert_eq!(KvcOp::parse("program", Some("select=KRC:\\R1\\Program\\BT_CELL")).unwrap(), KvcOp::ProgramSelect(Some("KRC:\\R1\\Program\\BT_CELL".into())));
        assert!(KvcOp::parse("program", Some("select=a b")).is_err());
        assert!(KvcOp::parse("shell", Some("ls")).is_err());
        assert!(!KvcOp::Status.changes() && KvcOp::Ack.changes());
    }

    #[test]
    fn hook_ops_carry_the_command_line_names() {
        let p = |op: &str, arg: Option<&str>| KvcOp::parse(op, arg).unwrap().hook_name_arg();
        assert_eq!(p("status", None), ("status", None));
        assert_eq!(p("mode", Some("ext")), ("mode", Some("EXT".into())));
        assert_eq!(p("mode", Some("show")), ("mode", Some("show".into())));
        assert_eq!(p("program", Some("select=KRC:\\R1\\Program\\BT_CELL")), ("program", Some("select=KRC:\\R1\\Program\\BT_CELL".into())));
        assert_eq!(p("program", Some("select")), ("program", Some("select".into())));
        assert_eq!(p("braketest-need", None), ("braketest-need", None));
    }

    #[test]
    fn output_is_parsed() {
        let (code, json, rest) = parse_output("starting\n{\"ok\": true, \"mode_op\": \"#EX\"}\np2c-exit=0\n");
        assert_eq!(code, Some(0));
        assert_eq!(json.unwrap()["mode_op"], "#EX");
        assert_eq!(rest, ["starting"]);
    }
}
