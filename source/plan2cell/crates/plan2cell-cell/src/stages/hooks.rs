//! The stages on a cell whose head and/or robots are `factoryos/hook@1` pods
//! (docs/tasks/cell-templates.md E7.2; `[plc_host] driver = "hook"`, `[[kuka_vc.controller]]
//! driver = "hook"`). A head and its robots may use different drivers.
//!
//! ```text
//! preflight  ready on the head and every hook robot (describe first: contract, role)
//! runtimes   nothing (no PLC host); projects: nothing (the head builds at deploy)
//! deploy     head-bundle@1 of the build copied into the head's inbox, hook deploy with the
//!            wiring of cell.toml: built (or unchanged) for this plan, else exit 8 / 3 / 7
//! krl        per hook robot: krl-bundle@1 copied into its inbox, hook deploy (no errors), hook
//!            verify (every check ok)
//! verify     hook verify on every hook robot; the head's GVL_Cell.PLAN_ID, PLC_PRG.bCellError,
//!            <device>.xRunning read with hook read (names in `missing` are a note)
//! start      hook ready + prepare (EXT, acknowledged) on the robots, then ready + start (cold)
//!            on the head: running this plan, then the pulse loop of the CODESYS head over hook
//!            read / write
//! stop       hook stop {"runtime": false} (PLC_PRG.bStop)
//! ```

use serde_json::{Value, json};

use super::production::{HeadIo, status_text};
use super::*;
use crate::hook::{self, Answer, INBOX, Role as HookRole, Verb};

/// Who a hook call goes to (one per call, short-lived).
#[derive(Clone, Debug)]
#[allow(clippy::large_enum_variant)]
pub(super) enum Hooked {
    Head,
    Robot(KukaController),
}

impl Hooked {
    fn role(&self) -> HookRole {
        match self {
            Hooked::Head => HookRole::Head,
            Hooked::Robot(_) => HookRole::Robot,
        }
    }

    /// How the notes and errors name it.
    fn name(&self) -> String {
        match self {
            Hooked::Head => "head".into(),
            Hooked::Robot(c) => c.robot_ns.clone(),
        }
    }
}

/// One temporary file per bundle copy (runs in one process must not share it).
fn temp_file(what: &str) -> PathBuf {
    static N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let n = N.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    std::env::temp_dir().join(format!("plan2cell-{what}-{}-{n}.json", std::process::id()))
}

/// The diagnostics of a head deploy (`errors` / `warnings`: `{file, line, message}` objects, or
/// strings) as `file:line: message`.
fn diagnostics(v: &Value) -> Vec<String> {
    v.as_array()
        .map(|a| {
            a.iter()
                .map(|d| match d {
                    Value::String(s) => s.clone(),
                    d => {
                        let msg = d["message"].as_str().unwrap_or("");
                        match (d["file"].as_str(), d["line"].as_u64()) {
                            (Some(f), Some(l)) => format!("{f}:{l}: {msg}"),
                            (Some(f), None) => format!("{f}: {msg}"),
                            _ => msg.to_string(),
                        }
                    }
                })
                .collect()
        })
        .unwrap_or_default()
}

/// The running program a robot hook's deploy refused for (exit 3, hook@1; the wording is
/// p2c-kvc's, which fos-hooks keeps: "a robot program is running (NAME); …"), or the answer's
/// `pro_name1`. `None` for any other answer.
fn running_program(a: &hook::Answer) -> Option<String> {
    const RUNNING: &str = "a robot program is running";
    if a.exit != 3 || a.problem.is_some() {
        return None;
    }
    let e = a.error();
    let rest = e.split_once(RUNNING)?.1;
    if let Some(n) = a.json["pro_name1"].as_str().filter(|n| !n.trim().is_empty()) {
        return Some(n.trim().to_string());
    }
    let inner = rest.trim_start().strip_prefix('(')?.split_once(')')?.0;
    Some(inner.trim().to_string())
}

/// Whether a program name is the plan's cell program BT_CELL (any case, with or without a
/// path or the `()` of `$PRO_NAME1[]`).
fn is_bt_cell(name: &str) -> bool {
    let base = name.rsplit(['/', '\\']).next().unwrap_or(name);
    base.trim().trim_end_matches("()").eq_ignore_ascii_case("BT_CELL")
}

impl Cell<'_> {
    /// Whether this run's deploy stage built the head (its project report says deployed).
    fn head_deployed_this_run(&self) -> bool {
        self.head_project()
            .ok()
            .and_then(|h| self.report.projects.iter().find(|r| r.name == h.name).and_then(|r| r.deployed))
            == Some(true)
    }

    fn hook_label(&self, who: &Hooked) -> String {
        match who {
            Hooked::Head => format!(
                "kubectl:{}",
                self.cfg.plc_host.kubectl_pod.as_deref().unwrap_or("?")
            ),
            Hooked::Robot(c) => c.access().to_string(),
        }
    }

    /// One hook call, recorded in [`Report::calls`] (a dry run sends nothing and gets
    /// [`Answer::dry`]). The first contact with a pod in this session is `describe`: another
    /// contract or role is exit 3. The answer may have failed: see [`Cell::hook_ok`].
    pub(super) fn hook(
        &mut self,
        stage: Stage,
        who: &Hooked,
        verb: Verb,
        request: &Value,
    ) -> Result<Answer> {
        let label = self.hook_label(who);
        if verb != Verb::Describe {
            self.hook_contact(stage, who)?;
        }
        self.report.calls.push(format!(
            "{label}: hook {} {}",
            verb.name(),
            hook::request_text(request)
        ));
        if self.settings.dry_run {
            return Ok(Answer::dry(verb));
        }
        let wait = self.settings.hook_retry;
        let remote: &mut dyn Remote = match who {
            Hooked::Head => &mut *self.remote,
            Hooked::Robot(c) => self
                .vc
                .as_mut()
                .ok_or_else(|| Error::Config("no remote for the KUKA VC hosts".into()))?
                .remote(c)?,
        };
        hook::call_retried(remote, verb, request, wait)
    }

    /// The first contact with a pod in this session: `describe` (contract, role; another one is
    /// exit 3). Nothing in a dry run.
    fn hook_contact(&mut self, stage: Stage, who: &Hooked) -> Result<()> {
        let label = self.hook_label(who);
        if self.settings.dry_run || self.described.contains(&label) {
            return Ok(());
        }
        let d = self.hook(stage, who, Verb::Describe, &json!({}))?;
        if let Err(m) = hook::check_describe(&d, who.role()) {
            let code = if d.ok() { 3 } else { d.stage_exit() };
            return Err(failed(stage, code, format!("{} ({label}): {m}", who.name())));
        }
        self.described.insert(label);
        Ok(())
    }

    /// The answer's notes as the stage's, then `Err` (its stage exit) unless it succeeded.
    fn hook_ok(
        &self,
        stage: Stage,
        who: &Hooked,
        a: &Answer,
        notes: &mut Vec<String>,
    ) -> Result<()> {
        let name = who.name();
        notes.extend(a.notes.iter().map(|n| format!("{name}: {n}")));
        if a.ok() {
            Ok(())
        } else {
            Err(failed(stage, a.stage_exit(), a.failure(&name)))
        }
    }

    /// Copy a bundle (its text) into the pod's inbox under `name` ([`Remote::copy`]: a
    /// `.p2c-tmp` file, then `mv`).
    fn hook_copy(&mut self, stage: Stage, who: &Hooked, text: &str, name: &str) -> Result<()> {
        self.hook_contact(stage, who)?;
        let local = temp_file(name.trim_end_matches(".json"));
        let dest = format!("{INBOX}/{name}");
        if self.settings.dry_run {
            self.report.calls.push(format!(
                "{}: copy {} {dest}",
                self.hook_label(who),
                local.display()
            ));
            return Ok(());
        }
        std::fs::write(&local, text).map_err(|e| Error::io(&local, e))?;
        let r = match who {
            Hooked::Head => {
                self.report.calls.push(format!(
                    "{}: copy {} {dest}",
                    self.hook_label(who),
                    local.display()
                ));
                self.remote.copy(&local, &dest)
            }
            Hooked::Robot(c) => self.vc_copy(c, &local, &dest),
        };
        let _ = std::fs::remove_file(&local);
        r
    }

    /// `ready`: exit 0 and `ready: true` (its `info` never gates), else exit 3.
    fn hook_ready(&mut self, stage: Stage, who: &Hooked, notes: &mut Vec<String>) -> Result<()> {
        let a = self.hook(stage, who, Verb::Ready, &json!({}))?;
        if self.settings.dry_run {
            return Ok(());
        }
        self.hook_ok(stage, who, &a, notes)?;
        if a.json["ready"] != true {
            let bad: Vec<String> = a.json["checks"]
                .as_array()
                .map(|c| {
                    c.iter()
                        .filter(|c| c["ok"] != true)
                        .map(|c| {
                            format!(
                                "{}{}",
                                c["name"].as_str().unwrap_or("?"),
                                c["detail"].as_str().map(|d| format!(" ({d})")).unwrap_or_default()
                            )
                        })
                        .collect()
                })
                .unwrap_or_default();
            return Err(failed(
                stage,
                3,
                format!("{}: hook ready: not ready: {}", who.name(), bad.join(", ")),
            ));
        }
        Ok(())
    }

    /// Every hook robot ready (preflight).
    pub(super) fn hook_robots_ready(&mut self, notes: &mut Vec<String>) -> Result<()> {
        let hooks: Vec<KukaController> =
            self.cfg.controllers().iter().filter(|c| c.is_hook()).cloned().collect();
        for c in hooks {
            let who = Hooked::Robot(c);
            self.hook_ready(Stage::Preflight, &who, notes)?;
            if !self.settings.dry_run {
                notes.push(format!("{} (hook): ready", who.name()));
            }
        }
        Ok(())
    }

    pub(super) fn hook_preflight(&mut self, notes: &mut Vec<String>) -> Result<()> {
        self.hook_ready(Stage::Preflight, &Hooked::Head, notes)?;
        if !self.settings.dry_run {
            notes.push("head (hook): ready".into());
        }
        self.hook_robots_ready(notes)
    }

    fn head_project(&self) -> Result<Project> {
        self.manifest()
            .projects
            .iter()
            .find(|p| p.role == Role::HeadPlc)
            .cloned()
            .ok_or_else(|| Error::Config("project.json has no head_plc project".into()))
    }

    /// `deploy` on a hook head (E7.3): the head-bundle@1 into its inbox, then `deploy` with the
    /// wiring; `built: true` for this plan (`unchanged` allowed).
    pub(super) fn hook_deploy(&mut self, notes: &mut Vec<String>) -> Result<()> {
        let m = self.manifest().clone();
        let head = self.head_project()?;
        let b = hook::head_bundle(self.tree.as_ref().expect("checked by stage()"), &m)?;
        let wiring = hook::wiring(&self.cfg)?;
        self.hook_copy(Stage::Deploy, &Hooked::Head, &b.json, hook::HEAD_BUNDLE)?;
        let a = self.hook(
            Stage::Deploy,
            &Hooked::Head,
            Verb::Deploy,
            &json!({"bundle": hook::HEAD_BUNDLE, "wiring": wiring, "force": false}),
        )?;
        if self.settings.dry_run {
            return Ok(());
        }
        let built = a.ok() && a.json["built"] == true;
        let plan = a.json["planId"].as_str().map(str::to_string);
        {
            let rep = self.project_report(&head.name);
            rep.errors = diagnostics(&a.json["errors"]);
            rep.warnings = diagnostics(&a.json["warnings"]);
            rep.deployed = Some(built);
            rep.state = Some(if built { "built" } else { "not built" }.into());
            rep.verified = None;
            if !a.ok() {
                rep.errors.push(a.error());
            }
        }
        self.hook_ok(Stage::Deploy, &Hooked::Head, &a, notes)?;
        if !built {
            return Err(failed(
                Stage::Deploy,
                8,
                format!("head: hook deploy: not built ({} error(s))", diagnostics(&a.json["errors"]).len()),
            ));
        }
        if let Some(p) = &plan
            && !codesys::same_value(p, &b.plan_id)
        {
            return Err(failed(
                Stage::Deploy,
                7,
                format!("head: hook deploy built planId {p}, the bundle is plan {}", b.plan_id),
            ));
        }
        notes.push(format!(
            "head: plan {} {} ({} file(s){}{})",
            b.plan_id,
            if a.json["unchanged"] == true { "unchanged" } else { "built" },
            b.files.len(),
            a.json["deployment"].as_str().map(|d| format!(", deployment {d}")).unwrap_or_default(),
            match diagnostics(&a.json["warnings"]).len() {
                0 => String::new(),
                n => format!(", {n} warning(s)"),
            }
        ));
        Ok(())
    }

    /// `krl` on a hook robot (E7.2): the krl-bundle@1 into its inbox, `deploy` (exit 0 with no
    /// errors), then `verify` of that bundle (every check ok).
    pub(super) fn krl_hook(
        &mut self,
        c: &KukaController,
        plan: &str,
        notes: &mut Vec<String>,
    ) -> Result<()> {
        let ns = c.robot_ns.clone();
        let tree = self.tree.as_ref().expect("checked by stage()");
        if !tree.exists(&krl::manifest_path(&ns)) {
            return Err(Error::Config(format!(
                "the build has no {} (build with [krl] enabled for {ns})",
                krl::manifest_path(&ns)
            )));
        }
        let b = Bundle::load(tree, &ns)?;
        if b.manifest.plan_id != plan {
            return Err(failed(
                Stage::Krl,
                7,
                format!(
                    "krl/{ns} is for plan {}, the build is plan {plan} (rebuild)",
                    b.manifest.plan_id
                ),
            ));
        }
        {
            let rep = self.controller_report(c);
            rep.plan_id = Some(b.manifest.plan_id.clone());
            rep.deployed = None;
            rep.verified = None;
            rep.changed.clear();
            rep.errors.clear();
            rep.checks.clear();
        }
        let who = Hooked::Robot(c.clone());
        self.hook_copy(Stage::Krl, &who, &b.json, hook::KRL_BUNDLE)?;
        let mut a = self.hook(
            Stage::Krl,
            &who,
            Verb::Deploy,
            &json!({"bundle": hook::KRL_BUNDLE, "force": false, "dry_run": false}),
        )?;
        // The plan's own BT_CELL still running when this run has already deployed the head: the
        // launch's own restore started the previous build (a krl-sim robot runs BT_CELL as soon
        // as it is started; a KUKA VC's stays selected), and Run Control Code replaces it. Then,
        // once: `op program cancel` (hook@1), and deploy again, still without force. Any other
        // running program (a brake test, a program selected by hand), or no head deployed in this
        // run, stays the hook's refusal.
        if !self.settings.dry_run
            && running_program(&a).is_some_and(|p| is_bt_cell(&p))
            && self.head_deployed_this_run()
        {
            let c = self.hook(Stage::Krl, &who, Verb::Op, &hook::op_request("program", Some("cancel")))?;
            if !c.ok() {
                return Err(failed(
                    Stage::Krl,
                    c.stage_exit(),
                    format!("{ns}: BT_CELL ran after the head's deploy; {}", c.failure("program cancel")),
                ));
            }
            notes.push(format!(
                "{ns}: BT_CELL of the previous build ran after the head's deploy: cancelled, deployed again"
            ));
            a = self.hook(
                Stage::Krl,
                &who,
                Verb::Deploy,
                &json!({"bundle": hook::KRL_BUNDLE, "force": false, "dry_run": false}),
            )?;
        }
        let what = format!("{ns} on hook ({})", c.access());
        if !self.settings.dry_run {
            let changed = krl::strings(&a.json["changed"]);
            let errors = krl::compile_errors(&a.json["errors"]);
            let error = a.json["error"].as_str().unwrap_or("").to_string();
            let rep = self.controller_report(c);
            rep.changed = changed.clone();
            rep.errors = errors.clone();
            if !error.is_empty() {
                rep.errors.push(error);
            }
            rep.messages = krl::messages(&a.json["messages"]);
            rep.deployed = Some(a.ok() && errors.is_empty());
            notes.extend(a.notes.iter().map(|n| format!("{ns}: {n}")));
            if a.exit == 8 || (a.ok() && !errors.is_empty()) {
                let mut e = errors.clone();
                e.truncate(5);
                return Err(failed(
                    Stage::Krl,
                    8,
                    format!("{what}: {} compile error(s): {}", errors.len(), e.join("; ")),
                ));
            }
            if !a.ok() {
                return Err(failed(Stage::Krl, a.stage_exit(), a.failure(&what)));
            }
            notes.push(format!(
                "{what}: {} file(s) changed, {} unchanged",
                changed.len(),
                a.json["unchanged"]
            ));
        }
        if !self.verify_hook_robot(Stage::Krl, c, Some(hook::KRL_BUNDLE), notes)? {
            return Err(failed(
                Stage::Krl,
                7,
                format!("{what}: hook verify: checks failed (see controllers)"),
            ));
        }
        Ok(())
    }

    /// `verify` on a hook robot (`bundle`: that inbox file, else the installed one): whether
    /// every check passed (exit 0 with at least one check). An answer other than 0 / 7 is `Err`.
    pub(super) fn verify_hook_robot(
        &mut self,
        stage: Stage,
        c: &KukaController,
        bundle: Option<&str>,
        notes: &mut Vec<String>,
    ) -> Result<bool> {
        let who = Hooked::Robot(c.clone());
        let req = match bundle {
            Some(b) => json!({"bundle": b}),
            None => json!({}),
        };
        let a = self.hook(stage, &who, Verb::Verify, &req)?;
        if self.settings.dry_run {
            return Ok(true);
        }
        if !matches!(a.exit, 0 | 7) {
            notes.extend(a.notes.iter().map(|n| format!("{}: {n}", c.robot_ns)));
            let rep = self.controller_report(c);
            rep.verified = Some(false);
            return Err(failed(stage, a.stage_exit(), a.failure(&c.robot_ns)));
        }
        let checks = check_results(&a.json["checks"]);
        let ok = a.ok() && !checks.is_empty() && checks.iter().all(|x| x.ok);
        let errors = krl::compile_errors(&a.json["errors"]);
        let rep = self.controller_report(c);
        rep.checks = checks;
        rep.verified = Some(ok);
        rep.messages = krl::messages(&a.json["messages"]);
        if !errors.is_empty() {
            rep.errors = errors;
        }
        notes.extend(a.notes.iter().map(|n| format!("{}: {n}", c.robot_ns)));
        if ok {
            notes.push(format!("{} on hook verified", c.robot_ns));
        }
        Ok(ok)
    }

    /// `read` on the hook head: the values (`None` in a dry run). A name in `missing` is absent
    /// (shown as `?`); a failed read is the stage's error.
    pub(super) fn hook_values(
        &mut self,
        stage: Stage,
        names: &[&str],
    ) -> Result<Option<BTreeMap<String, String>>> {
        let mut values = BTreeMap::new();
        for chunk in names.chunks(hook::MAX_VARS) {
            let a = self.hook(stage, &Hooked::Head, Verb::Read, &hook::read_request(chunk))?;
            if self.settings.dry_run {
                return Ok(None);
            }
            if !a.ok() {
                return Err(failed(stage, a.stage_exit(), a.failure("head")));
            }
            values.extend(hook::values(&a).values);
        }
        Ok(Some(values))
    }

    /// One operator pulse on the hook head (`write` of `true`: one rising edge); a refused name
    /// is exit 3.
    pub(super) fn hook_pulse(&mut self, stage: Stage, name: &str) -> Result<()> {
        let a = self.hook(stage, &Hooked::Head, Verb::Write, &hook::write_request(&[(name, true)]))?;
        if self.settings.dry_run {
            return Ok(());
        }
        if !a.ok() {
            return Err(failed(stage, a.stage_exit(), a.failure("head")));
        }
        let refused = krl::strings(&a.json["refused"]);
        if !refused.is_empty() {
            return Err(failed(
                stage,
                3,
                format!("head: hook write: refused {}", refused.join(", ")),
            ));
        }
        Ok(())
    }

    /// A hook robot ready, acknowledged and in EXT (`prepare`, replaces kvc-ack.sh / kvc-mode.sh).
    pub(super) fn hook_prepare(&mut self, c: &KukaController, notes: &mut Vec<String>) -> Result<()> {
        let who = Hooked::Robot(c.clone());
        self.hook_ready(Stage::Start, &who, notes)?;
        let a = self.hook(
            Stage::Start,
            &who,
            Verb::Prepare,
            &json!({"mode": "EXT", "ack": true, "force": false}),
        )?;
        if self.settings.dry_run {
            return Ok(());
        }
        let name = format!("{} (hook)", c.robot_ns);
        notes.extend(a.notes.iter().map(|n| format!("{}: {n}", c.robot_ns)));
        if !a.ok() {
            return Err(failed(Stage::Start, a.stage_exit(), a.failure(&name)));
        }
        let s = |k: &str| a.json[k].as_str().unwrap_or("?").to_string();
        notes.push(format!(
            "{name}: prepare: {}, {}, program {}",
            s("mode_op"),
            s("pro_state0"),
            s("pro_name1")
        ));
        Ok(())
    }

    /// `verify` on a hook head: the robots' verify, then the head's acceptance values with `read`
    /// until every one passes or `verify_timeout` has passed.
    pub(super) fn hook_verify(&mut self, notes: &mut Vec<String>) -> Result<()> {
        if !self.cfg.controllers().is_empty() {
            self.verify_vc(notes)?;
        }
        let m = self.manifest().clone();
        let head = self.head_project()?;
        let expect = codesys::verify_checks(&m, &head);
        let names: Vec<&str> = expect.iter().map(|e| e.expr.as_str()).collect();
        let deadline = Instant::now() + self.settings.verify_timeout;
        let mut attempt = 0;
        loop {
            attempt += 1;
            let Some(values) = self.hook_values(Stage::Verify, &names)? else {
                return Ok(());
            };
            let missing: Vec<&str> =
                names.iter().copied().filter(|n| !values.contains_key(*n)).collect();
            let checks: Vec<CheckResult> = expect
                .iter()
                .filter(|e| values.contains_key(&e.expr))
                .map(|e| {
                    let value = values.get(&e.expr).cloned();
                    CheckResult {
                        expr: e.expr.clone(),
                        expect: e.expect.clone(),
                        ok: value.as_deref().is_some_and(|v| codesys::same_value(v, &e.expect)),
                        value,
                    }
                })
                .collect();
            let ok = !checks.is_empty() && checks.iter().all(|c| c.ok);
            let rep = self.project_report(&head.name);
            rep.checks = checks;
            rep.verified = Some(ok);
            if ok {
                if !missing.is_empty() {
                    notes.push(format!("head: not readable on the hook (missing): {}", missing.join(", ")));
                }
                notes.push(format!("verified after {attempt} attempt(s)"));
                return Ok(());
            }
            if Instant::now() + self.settings.verify_interval > deadline {
                return Err(failed(
                    Stage::Verify,
                    7,
                    format!(
                        "checks failed after {attempt} attempt(s): {} (see checks)",
                        head.name
                    ),
                ));
            }
            std::thread::sleep(self.settings.verify_interval);
        }
    }

    /// `start` on a hook head (E7.2): the robots first (ready, prepare), then the head ready and
    /// `start {"restart":"cold"}` (running this plan), then the pulse loop over read / write.
    pub(super) fn hook_start(&mut self, notes: &mut Vec<String>) -> Result<()> {
        let head = self.head_project()?;
        if !self.cfg.controllers().is_empty() {
            self.controllers_ready(None, notes)?;
        }
        self.hook_ready(Stage::Start, &Hooked::Head, notes)?;
        let a = self.hook(Stage::Start, &Hooked::Head, Verb::Start, &json!({"restart": "cold"}))?;
        if !self.settings.dry_run {
            self.hook_ok(Stage::Start, &Hooked::Head, &a, notes)?;
            let state = a.json["state"].as_str().unwrap_or("?");
            if state != "running" {
                return Err(failed(
                    Stage::Start,
                    7,
                    format!("head: hook start: state {state}, want running"),
                ));
            }
            let want = self.manifest().plan.plan_id.clone();
            match a.json["planId"].as_str() {
                Some(p) if codesys::same_value(p, &want) => {
                    notes.push(format!(
                        "head running plan {want} (= this export){}",
                        a.json["deployment"].as_str().map(|d| format!(", deployment {d}")).unwrap_or_default()
                    ));
                }
                p => {
                    return Err(failed(
                        Stage::Start,
                        7,
                        format!(
                            "the head runs PLAN_ID {} (hook start planId), this export is plan {want}: \
                             start the export the head runs, or deploy this one first (cell deploy, krl DIR)",
                            p.unwrap_or("?")
                        ),
                    ));
                }
            }
            self.project_report(&head.name).state = Some("running".into());
        }
        let deadline = Instant::now() + self.settings.start_timeout;
        self.pulse_into_production(&HeadIo::Hook, deadline, notes)
    }

    /// `stop` on a hook head: `stop {"runtime": false}` pulses PLC_PRG.bStop; the cell state from
    /// its answer (else read).
    pub(super) fn hook_stop(&mut self, notes: &mut Vec<String>) -> Result<()> {
        let a = self.hook(Stage::Stop, &Hooked::Head, Verb::Stop, &json!({"runtime": false}))?;
        if self.settings.dry_run {
            return Ok(());
        }
        self.hook_ok(Stage::Stop, &Hooked::Head, &a, notes)?;
        notes.push("PLC_PRG.bStop pulsed".into());
        let mut values: BTreeMap<String, String> = hook::values(&a).values;
        if !values.contains_key(codesys::CELL_STATUS[0]) {
            std::thread::sleep(self.settings.start_interval);
            values = self.hook_values(Stage::Stop, &codesys::CELL_STATUS)?.unwrap_or_default();
        }
        notes.push(status_text(&values));
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deploy_diagnostics_read_as_file_line_message() {
        let v = json!([{"severity":"error","code":"E1","message":"unknown name","file":"src/bt2plc/FB_Cell.st","line":12,"source":"trust"},
                       {"message":"no file"}, "plain"]);
        assert_eq!(diagnostics(&v), ["src/bt2plc/FB_Cell.st:12: unknown name", "no file", "plain"]);
    }

    #[test]
    fn bt_cell_is_the_cell_program_in_any_spelling() {
        for n in ["BT_CELL", "bt_cell", "BT_CELL()", "/R1/BT_CELL", "KRC:\\R1\\Program\\BT_CELL"] {
            assert!(is_bt_cell(n), "{n}");
        }
        for n in ["BRAKETEST_REQ", "BT_CELL2", "CELL", ""] {
            assert!(!is_bt_cell(n), "{n}");
        }
    }
}
