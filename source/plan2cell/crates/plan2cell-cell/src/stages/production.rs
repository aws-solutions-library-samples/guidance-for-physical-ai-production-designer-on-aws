//! `start` and `stop`: the deployed cell into production and out of it again.
//!
//! ```text
//! start  IDE       MCP: close_project of the head without saving (edits made in the PLC IDE
//!                  during the review are discarded: bind saves, the login would download them)
//!        runtimes  ssh: every project's container running and its gateway port open, else (or
//!                  with Settings::restart_runtime) `docker restart`, wait for the port (a demo
//!                  runtime stops after ~2 h; the boot application comes back with it)
//!        libraries MCP: install_libraries of the libraries stored with the project (an archive
//!                  without libraries.json, or a server without the tool: download_missing_libraries),
//!                  list_project_libraries, compile_project: any error stops here (exit 8)
//!        head      MCP: bind, login (a failed one: restart the head runtime once, then again
//!                  every start_interval within Settings::login_timeout; a build or library error
//!                  at login is exit 8 at once, see codesys::build_failure), start the
//!                  application if it does not run, refuse a head whose GVL_Cell.PLAN_ID is not
//!                  this export's plan (exit 7, before any pulse)
//!        KUKA VCs  every [[kuka_vc.controller]] at once, with the head logged in and (after a
//!                  runtime restart) Settings::ack_settle after it, so the PROFINET AR is up:
//!                  kvc-ack.sh --netns krcN when the host has it (M_2858 and the other stop
//!                  messages acknowledged in T1, back to EXT), then kvc-mode.sh N EXT (a no-op
//!                  when $MODE_OP = #EX; skipped when kvc-ack.sh just ended in EXT); older
//!                  bundles without kvc-ack.sh: kvc-mode.sh N EXT only. The kvc-ack.sh runs are
//!                  sent together, then waited for; kvc-mode.sh per controller. Not in EXT: exit 7
//!        pulses    read the cell state until it runs: CS_ERROR -> pulse bReset, CS_RECOVERY ->
//!                  pulse bRecoveryConfirm (never bStart: AUTO_RUN starts the cycle; CS_READY
//!                  three reads in a row fails); CS_RUNNING / CS_CYCLE_DONE: done. A robot still
//!                  in ERR_START_TIMEOUT / ERR_DRIVES_TIMEOUT after 3 pulses: its controller's
//!                  kvc-ack.sh once more, then on pulsing
//!        always    disconnect_from_device (a login left behind makes a later create_project hang)
//! stop   MCP: close_project of the head without saving (as start), bind, login, pulse bStop, read the cell state, disconnect_from_device
//!
//! The pulse loop is [`Pulses::decide`] over [`HeadIo`]: the same for a `factoryos/hook@1` head
//! (stages/hooks.rs: `read` / `write` instead of MCP), and hook robots get `prepare` instead of
//! kvc-ack.sh / kvc-mode.sh.
//! reset-ide  MCP: per project of the build (head first) disconnect_from_device and close_project
//!        without saving, then shutdown_codesys (unsaved changes discarded), launch_codesys,
//!        get_codesys_status: exit 0 when the current install is ready, else exit 8
//! ```

use super::*;

use crate::codesys::CELL_STATUS;

/// The plan the head runs (a constant of the head project; `verify` checks it too).
const HEAD_PLAN_ID: &str = "GVL_Cell.PLAN_ID";

/// The faults of a robot whose drives do not come up: what M_2858 (fieldbus stop) leaves.
const DRIVE_ERRORS: [&str; 2] = ["ERR_START_TIMEOUT", "ERR_DRIVES_TIMEOUT"];
/// After this many pulses with one robot still in a [`DRIVE_ERRORS`] fault, its controller's
/// `kvc-ack.sh` runs once more.
const ACK_AGAIN_AFTER: u32 = 3;

/// What `kvc-mode.sh` / `kvc-ack.sh` printed and its exit code (`p2c-exit=N`).
fn exit_of(out: &str) -> (Option<i32>, Vec<&str>) {
    let mut code = None;
    let mut lines = Vec::new();
    for l in out.lines() {
        match l.trim().strip_prefix("p2c-exit=") {
            Some(c) => code = c.trim().parse().ok(),
            None if !l.trim().is_empty() => lines.push(l.trim()),
            None => {}
        }
    }
    (code, lines)
}

/// `krcN` -> N (the controller number of `kvc-mode.sh`).
fn controller_number(netns: &str) -> Option<u32> {
    netns.strip_prefix("krc")?.parse().ok().filter(|n| *n >= 1)
}

impl Cell<'_> {
    pub(super) fn head(&self) -> Result<Project> {
        self.manifest()
            .projects
            .iter()
            .find(|p| p.role == Role::HeadPlc)
            .cloned()
            .ok_or_else(|| Error::Config("project.json has no head_plc project".into()))
    }

    pub(super) fn start(&mut self, notes: &mut Vec<String>) -> Result<()> {
        let head = self.head()?;
        // bind saves the project and the login would download what differs: never an edit made
        // in the PLC IDE during the review.
        self.discard_ide_edits(&head, notes);
        // Whether the server installs the libraries stored with the project.
        self.load_caps()?;
        let mut restarted = self.runtimes_up(notes)?;
        let mut restarted_at = (!restarted.is_empty()).then(Instant::now);
        let r = self.head_production(&head, &mut restarted, &mut restarted_at, notes);
        // Always: a login left behind blocks the next `projects`.
        let opts = self.options();
        self.best_effort(&codesys::disconnect(&head, &opts));
        r
    }

    pub(super) fn stop(&mut self, notes: &mut Vec<String>) -> Result<()> {
        let head = self.head()?;
        self.discard_ide_edits(&head, notes);
        let opts = self.options();
        let r = (|| {
            let mut o = Outcome::default();
            self.exec(
                Stage::Stop,
                Some(&head),
                &codesys::login(&head, &opts),
                &mut o,
            )?;
            self.exec(
                Stage::Stop,
                Some(&head),
                &[codesys::pulse(&head, &opts, "PLC_PRG.bStop")],
                &mut o,
            )?;
            if self.settings.dry_run {
                return Ok(());
            }
            notes.push("PLC_PRG.bStop pulsed".into());
            std::thread::sleep(self.settings.start_interval);
            let mut o = Outcome::default();
            self.exec(
                Stage::Stop,
                Some(&head),
                &[codesys::read(&head, &opts, &CELL_STATUS)],
                &mut o,
            )?;
            notes.push(status_text(&o.values));
            Ok(())
        })();
        self.best_effort(&codesys::disconnect(&head, &opts));
        r
    }

    /// `reset-ide`: the IDE's CODESYS restarted ([`codesys::reset_ide`]). With a build, its
    /// projects are logged out and closed without saving first (best effort, head first: the
    /// shutdown discards unsaved changes anyway). Healthy when `get_codesys_status` shows the
    /// current install ready; else exit 8. A shutdown or launch that fails is a note: the status
    /// decides.
    pub(super) fn reset_ide(&mut self, notes: &mut Vec<String>) -> Result<()> {
        let opts = self.options();
        if let Some(m) = self.manifest.clone() {
            for p in m.ordered().into_iter().rev() {
                if let Some(r) = self.best_effort(&codesys::disconnect(p, &opts))
                    && !r.failed()
                {
                    notes.push(format!("{}: logged out", p.name));
                }
                self.discard_ide_edits(p, notes);
            }
        } else if !self.settings.dry_run {
            notes.push(
                "no build directory: no project logged out or closed first (the shutdown \
                 discards unsaved changes)"
                    .into(),
            );
        }
        let calls = codesys::reset_ide(&opts);
        let mut state = None;
        for c in &calls {
            if self.settings.dry_run {
                self.best_effort(c);
                continue;
            }
            let r = self.send(c)?;
            let said = r.short().split_whitespace().collect::<Vec<_>>().join(" ");
            if c.check == Check::Status {
                if r.failed() {
                    return Err(failed(Stage::ResetIde, 8, format!("{}: {said}", c.tool)));
                }
                state = codesys::codesys_state(&r.text);
                notes.push(format!(
                    "{}: {}",
                    c.tool,
                    state.as_deref().unwrap_or("no state")
                ));
            } else if r.failed() {
                notes.push(format!("{} failed: {said}", c.tool));
            } else {
                notes.push(format!("{}: {said}", c.tool));
            }
        }
        if self.settings.dry_run {
            return Ok(());
        }
        match state {
            Some(s) if codesys::codesys_healthy(&s) => Ok(()),
            s => Err(failed(
                Stage::ResetIde,
                8,
                format!(
                    "CODESYS not ready after the restart (get_codesys_status: {})",
                    s.as_deref().unwrap_or("no state")
                ),
            )),
        }
    }

    /// Every project's runtime up (container running, gateway port open); the ones that are not
    /// (all with `restart_runtime`) restarted and waited for. The containers restarted.
    fn runtimes_up(&mut self, notes: &mut Vec<String>) -> Result<Vec<String>> {
        let m = self.manifest().clone();
        let ps = self.ssh("sudo docker ps --format '{{.Names}}'")?;
        let running: Vec<&str> = ps.lines().map(str::trim).collect();
        let mut down = Vec::new();
        for p in m.ordered() {
            let rt = &p.runtime;
            let gone = !running.contains(&rt.container.as_str()) && !self.settings.dry_run;
            let up = if self.settings.restart_runtime || gone {
                false
            } else {
                let out = self.ssh(&tcp_probe(&rt.ip, rt.gateway_port))?;
                self.settings.dry_run || out.lines().any(|l| l.trim() == "open")
            };
            if !up {
                down.push(p.clone());
            }
        }
        if let Some(h) = m.projects.iter().find(|p| p.role == Role::HeadPlc) {
            // The demo licence's countdown, for the report.
            let c = sh(&h.runtime.container);
            if let Ok(out) = self.ssh(&format!(
                "sudo docker exec {c} sh -c \"grep -E 'demo mode|shutdown in' \
                 /var/opt/codesys/codesyscontrol.log | tail -n 1\" 2>/dev/null || true"
            )) && let Some(l) = out.lines().rev().find(|l| !l.trim().is_empty())
            {
                notes.push(format!("{}: {}", h.runtime.container, l.trim()));
            }
        }
        if down.is_empty() {
            return Ok(Vec::new());
        }
        let names: Vec<String> = down.iter().map(|p| p.runtime.container.clone()).collect();
        self.restart(&down, notes)?;
        let why = if self.settings.restart_runtime {
            "as asked"
        } else {
            "it did not answer"
        };
        notes.push(format!("restarted {} ({why})", names.join(", ")));
        Ok(names)
    }

    /// `docker restart` the projects' containers, then wait for each gateway port.
    fn restart(&mut self, ps: &[Project], notes: &mut Vec<String>) -> Result<()> {
        let names: Vec<String> = ps.iter().map(|p| sh(&p.runtime.container)).collect();
        self.ssh(&format!("sudo docker restart {}", names.join(" ")))?;
        let tries = (self.settings.runtime_wait.as_secs() / 2).max(1);
        for p in ps {
            let rt = &p.runtime;
            let out = self.ssh(&format!(
                "for i in $(seq 1 {tries}); do if timeout 2 bash -c 'exec 3<>/dev/tcp/{}/{}' \
                 2>/dev/null; then echo open; exit 0; fi; sleep 2; done; echo closed",
                rt.ip, rt.gateway_port
            ))?;
            if self.settings.dry_run {
                continue;
            }
            if !out.lines().any(|l| l.trim() == "open") {
                return Err(failed(
                    Stage::Start,
                    7,
                    format!(
                        "{} restarted, but {}:{} did not open within {} s",
                        rt.container,
                        rt.ip,
                        rt.gateway_port,
                        self.settings.runtime_wait.as_secs()
                    ),
                ));
            }
            notes.push(format!(
                "{} answers on {}:{}",
                rt.container, rt.ip, rt.gateway_port
            ));
        }
        Ok(())
    }

    pub(super) fn has_controllers(&self) -> bool {
        self.cfg
            .kuka_vc
            .as_ref()
            .is_some_and(|k| !k.controllers.is_empty())
    }

    /// Every controller (`only`: the one of this robot_ns) out of its stop messages and in EXT:
    /// `kvc-ack.sh --netns krcN` where the host has it (acknowledges M_2858 & co. in T1, back to
    /// EXT; with nothing pending it leaves the mode alone), then `kvc-mode.sh N EXT` unless
    /// kvc-ack.sh just ended in EXT. Hosts without kvc-ack.sh (older bundles): kvc-mode.sh only.
    /// The probes and the kvc-ack.sh runs of all controllers are sent at once, then waited for
    /// (each kvc-ack.sh takes over a minute with M_1210 pending); kvc-mode.sh runs per controller,
    /// one after the other. Stop messages that remain are a note (the pulses decide); a
    /// controller not in EXT is exit 7.
    /// Hook robots (`driver = "hook"`): `ready`, then `prepare` (EXT, acknowledged) each, first.
    pub(super) fn controllers_ready(&mut self, only: Option<&str>, notes: &mut Vec<String>) -> Result<()> {
        let k = self.vc_config()?;
        for c in k
            .controllers
            .iter()
            .filter(|c| c.is_hook() && only.is_none_or(|ns| c.robot_ns == ns))
        {
            self.hook_prepare(c, notes)?;
        }
        let dir = k.dir.trim_end_matches('/');
        let (ack, mode) = (format!("{dir}/kvc-ack.sh"), format!("{dir}/kvc-mode.sh"));
        let dry = self.settings.dry_run;
        let mut ctrls = Vec::new();
        for c in k
            .controllers
            .iter()
            .filter(|c| !c.is_hook() && only.is_none_or(|ns| c.robot_ns == ns))
        {
            let n = controller_number(&c.netns).ok_or_else(|| {
                Error::Config(format!(
                    "kuka_vc.controller {}: netns {} is not krcN (kvc-mode.sh N)",
                    c.robot_ns, c.netns
                ))
            })?;
            ctrls.push((c, n, format!("{} ({})", c.robot_ns, c.netns)));
        }
        let probe = format!(
            "sudo -n test -x {} && echo present || echo missing",
            sh(&ack)
        );
        let probes: Vec<_> = ctrls.iter().map(|(c, _, _)| (*c, probe.clone())).collect();
        let mut has_ack = Vec::new();
        for r in self.vc_all(&probes) {
            has_ack.push(dry || r?.lines().any(|l| l.trim() == "present"));
        }
        let acks: Vec<_> = ctrls
            .iter()
            .zip(&has_ack)
            .filter(|(_, has)| **has)
            .map(|((c, _, _), _)| {
                (
                    *c,
                    format!(
                        "sudo -n {} --netns {} 2>&1; echo p2c-exit=$?",
                        sh(&ack),
                        c.netns
                    ),
                )
            })
            .collect();
        let mut acked = self.vc_all(&acks).into_iter();
        let mut first_err = None;
        let mut in_ext = vec![false; ctrls.len()];
        for (i, (_, n, who)) in ctrls.iter().enumerate() {
            if !has_ack[i] {
                notes.push(format!(
                    "{who}: no {ack} on the host (older bundle): kvc-mode.sh {n} EXT only"
                ));
                continue;
            }
            let out = match acked.next().expect("one answer per kvc-ack.sh") {
                Ok(o) => o,
                Err(e) => {
                    first_err.get_or_insert(e);
                    continue;
                }
            };
            if dry {
                continue;
            }
            let (code, lines) = exit_of(&out);
            let last = lines.last().copied().unwrap_or("no output");
            let tail = tail_of(&lines);
            // Its success line after an acknowledgement: "... controller N in EXT".
            in_ext[i] = code == Some(0) && last.ends_with("in EXT");
            notes.push(match code {
                Some(0) => format!("{who}: kvc-ack.sh: {last}"),
                Some(1) => format!("{who}: kvc-ack.sh: stop messages remain: {tail}"),
                Some(2) => format!("{who}: kvc-ack.sh refused (exit 2): {tail}"),
                c => format!(
                    "{who}: kvc-ack.sh exit {}: {tail}",
                    c.map_or("?".into(), |c| c.to_string())
                ),
            });
        }
        if let Some(e) = first_err {
            return Err(e);
        }
        let mut bad = Vec::new();
        for (i, (c, n, who)) in ctrls.iter().enumerate() {
            if in_ext[i] {
                continue;
            }
            let out = self.vc_run(
                c,
                &format!("sudo -n {} {n} EXT 2>&1; echo p2c-exit=$?", sh(&mode)),
            )?;
            if dry {
                continue;
            }
            let (code, lines) = exit_of(&out);
            let last = lines.last().copied().unwrap_or("no output");
            if code == Some(0) {
                notes.push(format!("{who}: {last}"));
            } else {
                bad.push(format!(
                    "{who}: exit {}: {}",
                    code.map_or("?".into(), |c| c.to_string()),
                    tail_of(&lines)
                ));
            }
        }
        if bad.is_empty() {
            Ok(())
        } else {
            Err(failed(
                Stage::Start,
                7,
                format!("KUKA VC not in EXT: {}", bad.join("; ")),
            ))
        }
    }

    /// The robot_ns of the head's robot `n` (`nErrorRobot`, 1-based): the n-th robot_sim project
    /// of project.json (the head numbers its robots in that order), else `robot<n>`.
    fn robot_ns_of(&self, n: u32) -> String {
        self.sim_robots
            .get(n.saturating_sub(1) as usize)
            .cloned()
            .flatten()
            .unwrap_or_else(|| format!("robot{n}"))
    }

    /// The head must run this export's plan (`GVL_Cell.PLAN_ID`, as `verify` reads it, against
    /// `plc/project.json` `plan.plan_id`): with another export's head the robots' KRL (deployed from
    /// this export) carries a PLAN_ID the head never sends, and no job runs. Refused before any pulse
    /// (exit 7); a PLAN_ID that does not read is only a note.
    fn head_plan_matches(&mut self, head: &Project, notes: &mut Vec<String>) -> Result<()> {
        let opts = self.options();
        let mut o = Outcome::default();
        self.exec(
            Stage::Start,
            Some(head),
            &[codesys::read(head, &opts, &[HEAD_PLAN_ID])],
            &mut o,
        )?;
        if self.settings.dry_run {
            return Ok(());
        }
        let want = self.manifest().plan.plan_id.clone();
        match o.values.get(HEAD_PLAN_ID) {
            Some(v) if !codesys::same_value(v, &want) => Err(failed(
                Stage::Start,
                7,
                format!(
                    "the head runs PLAN_ID {} ({HEAD_PLAN_ID}), this export is plan {want}: start the \
                     export the head runs, or deploy this one first (cell projects, deploy, krl DIR)",
                    v.trim().trim_matches('\'')
                ),
            )),
            Some(_) => {
                notes.push(format!("head PLAN_ID {want} = this export"));
                Ok(())
            }
            None => {
                notes.push(format!(
                    "{HEAD_PLAN_ID} not read: the head's plan is not checked against {want}"
                ));
                Ok(())
            }
        }
    }

    /// The libraries ready and the head built ([`codesys::libraries_ready`], any error is exit 8),
    /// then log in (one runtime restart if the login fails and the runtime was not restarted
    /// yet, then again within [`Settings::login_timeout`]; a build error at login is exit 8 at
    /// once), start the application if needed, then pulse it into production.
    fn head_production(
        &mut self,
        head: &Project,
        restarted: &mut Vec<String>,
        restarted_at: &mut Option<Instant>,
        notes: &mut Vec<String>,
    ) -> Result<()> {
        let opts = self.options();
        let dry = self.settings.dry_run;
        let interval = self.settings.start_interval;
        let mut o = Outcome::default();
        self.exec(
            Stage::Start,
            Some(head),
            &codesys::libraries_ready(head, &opts),
            &mut o,
        )?;
        notes.append(&mut o.notes);
        let mut login_until = Instant::now() + self.settings.login_timeout;
        loop {
            o = Outcome::default();
            match self.exec(
                Stage::Start,
                Some(head),
                &codesys::login(head, &opts),
                &mut o,
            ) {
                Ok(()) => break,
                Err(Error::Tool { tool, msg }) if !dry => {
                    if codesys::build_failure(&msg) {
                        // The login builds the application: the same error on every attempt.
                        return Err(failed(
                            Stage::Start,
                            8,
                            format!("{tool}: the head does not build: {msg}"),
                        ));
                    }
                    if !restarted.contains(&head.runtime.container) {
                        notes.push(format!(
                            "{tool} failed: restarting {}",
                            head.runtime.container
                        ));
                        self.restart(std::slice::from_ref(head), notes)?;
                        restarted.push(head.runtime.container.clone());
                        *restarted_at = Some(Instant::now());
                        login_until = Instant::now() + self.settings.login_timeout;
                    } else if Instant::now() + interval > login_until {
                        return Err(Error::Tool { tool, msg });
                    }
                    std::thread::sleep(interval);
                }
                Err(e) => return Err(e),
            }
        }
        // The pulses' budget starts with the login.
        let deadline = Instant::now() + self.settings.start_timeout;
        self.gateway_works();
        let run = |s: &Option<String>| {
            s.as_deref()
                .is_some_and(|s| s.to_ascii_lowercase().contains("run"))
        };
        if !dry && !run(&o.state) {
            notes.push(format!(
                "application {}: starting it",
                o.state.as_deref().unwrap_or("unknown")
            ));
            o = Outcome::default();
            self.exec(
                Stage::Start,
                Some(head),
                &codesys::start_application(head, &opts),
                &mut o,
            )?;
            if !run(&o.state) {
                return Err(failed(
                    Stage::Start,
                    7,
                    format!(
                        "{}: application {} after start",
                        head.name,
                        o.state.as_deref().unwrap_or("unknown")
                    ),
                ));
            }
        }
        if !dry {
            self.project_report(&head.name).state = o.state.clone();
        }
        self.head_plan_matches(head, notes)?;
        let vcs = self.has_controllers();
        if vcs {
            // The M_2858 of a restart comes with the PROFINET AR: acknowledge after it.
            if let Some(at) = *restarted_at
                && !dry
            {
                let wait =
                    (at + self.settings.ack_settle).saturating_duration_since(Instant::now());
                if !wait.is_zero() {
                    notes.push(format!(
                        "waiting {:.0} s after the runtime restart (PROFINET AR) before kvc-ack.sh",
                        wait.as_secs_f64()
                    ));
                    std::thread::sleep(wait);
                }
            }
            self.controllers_ready(None, notes)?;
        }
        self.pulse_into_production(&HeadIo::Codesys(head.clone()), deadline, notes)
    }

    /// The head's values of `names` (`None`: a dry run, nothing read): CODESYS `read_variables`,
    /// or the hook's `read` (a name in `missing` is absent).
    fn head_values(
        &mut self,
        io: &HeadIo,
        names: &[&str],
    ) -> Result<Option<BTreeMap<String, String>>> {
        match io {
            HeadIo::Codesys(head) => {
                let opts = self.options();
                let mut o = Outcome::default();
                self.exec(Stage::Start, Some(head), &[codesys::read(head, &opts, names)], &mut o)?;
                Ok((!self.settings.dry_run).then_some(o.values))
            }
            HeadIo::Hook => self.hook_values(Stage::Start, names),
        }
    }

    /// One operator pulse (one write of TRUE: one rising edge).
    fn head_pulse(&mut self, io: &HeadIo, name: &str) -> Result<()> {
        match io {
            HeadIo::Codesys(head) => {
                let opts = self.options();
                let mut o = Outcome::default();
                self.exec(Stage::Start, Some(head), &[codesys::pulse(head, &opts, name)], &mut o)
            }
            HeadIo::Hook => self.hook_pulse(Stage::Start, name),
        }
    }

    /// Read the cell state until it runs, pulsing it out of error / recovery ([`Pulses::decide`]);
    /// a robot still in a [`DRIVE_ERRORS`] fault after [`ACK_AGAIN_AFTER`] pulses is acknowledged
    /// once more ([`Cell::controllers_ready`]). The same loop for the CODESYS and the hook head.
    pub(super) fn pulse_into_production(
        &mut self,
        io: &HeadIo,
        deadline: Instant,
        notes: &mut Vec<String>,
    ) -> Result<()> {
        let interval = self.settings.start_interval;
        let vcs = self.has_controllers();
        let mut p = Pulses::default();
        loop {
            p.reads += 1;
            let Some(v) = self.head_values(io, &CELL_STATUS)? else {
                return Ok(());
            };
            let status = status_text(&v);
            let (pulse, ack_again, stuck) = match p.decide(&v, vcs) {
                Step::Done(note) => {
                    notes.push(note);
                    return Ok(());
                }
                Step::Fail(msg) => return Err(failed(Stage::Start, 7, msg)),
                Step::Go {
                    pulse,
                    ack_again,
                    stuck,
                } => (pulse, ack_again, stuck),
            };
            if let Some((err, n)) = ack_again {
                let ns = self.robot_ns_of(n);
                let c = self
                    .vc_config()?
                    .controllers
                    .iter()
                    .find(|c| c.robot_ns == ns)
                    .cloned();
                match c {
                    Some(c) => {
                        let how = if c.is_hook() { "hook prepare" } else { "kvc-ack.sh" };
                        notes.push(format!(
                            "robot {n} still {err} after {} pulse(s): {how} on {ns} once more",
                            p.drive_pulses(n)
                        ));
                        self.controllers_ready(Some(&ns), notes)?;
                    }
                    None => notes.push(format!(
                        "robot {n} still {err}: no [[kuka_vc.controller]] for {ns} to acknowledge"
                    )),
                }
            }
            if let Some(x) = pulse {
                if p.pulses >= self.settings.start_pulses {
                    return Err(failed(
                        Stage::Start,
                        7,
                        format!("not in production after {} pulse(s): {status}", p.pulses),
                    ));
                }
                self.head_pulse(io, x)?;
                p.pulsed(stuck);
                notes.push(format!("{status}: {x} pulsed"));
            }
            if Instant::now() + interval > deadline {
                return Err(failed(
                    Stage::Start,
                    7,
                    format!(
                        "not in production after {} s ({} pulse(s)): {status}",
                        self.settings.start_timeout.as_secs(),
                        p.pulses
                    ),
                ));
            }
            std::thread::sleep(interval);
        }
    }
}

/// How the start loop reads and pulses the head (one per start).
#[allow(clippy::large_enum_variant)]
pub(super) enum HeadIo {
    /// The CODESYS head over MCP (`read_variables`, `write_variable`), logged in.
    Codesys(Project),
    /// The `factoryos/hook@1` head (`read`, `write`).
    Hook,
}

/// What the start loop does after one read of the head's [`CELL_STATUS`].
#[derive(Debug, PartialEq, Eq)]
pub(super) enum Step {
    /// In production: the note.
    Done(String),
    /// Stage exit 7.
    Fail(String),
    /// Go on: acknowledge robot `ack_again` once more (its fault, its number), then pulse
    /// `pulse`; `stuck` is the robot of a [`DRIVE_ERRORS`] fault.
    Go {
        pulse: Option<&'static str>,
        ack_again: Option<(String, u32)>,
        stuck: Option<u32>,
    },
}

/// The start loop's counts: reads, pulses, READY in a row, pulses per robot with a
/// [`DRIVE_ERRORS`] fault, and the robots acknowledged once more.
#[derive(Debug, Default)]
pub(super) struct Pulses {
    pub(super) reads: u32,
    pub(super) pulses: u32,
    ready: u32,
    drive_pulses: BTreeMap<u32, u32>,
    acked_again: Vec<u32>,
}

impl Pulses {
    /// The decision after a read (the caller counts the read first): CS_RUNNING / CS_CYCLE_DONE
    /// done; no state: fail; CS_ERROR -> bReset, CS_RECOVERY -> bRecoveryConfirm; never bStart
    /// (AUTO_RUN starts a cycle from READY by itself; READY three reads in a row fails); with
    /// controllers (`vcs`), a robot of a drive fault after [`ACK_AGAIN_AFTER`] pulses is
    /// acknowledged once more, once.
    pub(super) fn decide(&mut self, v: &BTreeMap<String, String>, vcs: bool) -> Step {
        let state = v.get(CELL_STATUS[0]).and_then(|s| codesys::cell_state(s));
        let status = status_text(v);
        let pulse = match state {
            Some("CS_RUNNING" | "CS_CYCLE_DONE") => {
                return Step::Done(format!(
                    "in production after {} read(s) and {} pulse(s): {status}",
                    self.reads, self.pulses
                ));
            }
            None => return Step::Fail(format!("{} not read ({status})", CELL_STATUS[0])),
            Some("CS_ERROR") => Some("PLC_PRG.bReset"),
            Some("CS_RECOVERY") => Some("PLC_PRG.bRecoveryConfirm"),
            // AUTO_RUN starts a cycle from READY by itself (the head's own start
            // interlocks: every robot with a job at home). `start` never pulses bStart:
            // READY that stays means bAutoRun is off, the operator starts the cycle.
            Some("CS_READY") => {
                self.ready += 1;
                if self.ready >= 3 {
                    return Step::Fail(format!(
                        "{status} and not starting (PLC_PRG.bAutoRun FALSE?): start \
                         pulses only bReset / bRecoveryConfirm; start the cycle at the head"
                    ));
                }
                None
            }
            Some(_) => None,
        };
        if state != Some("CS_READY") {
            self.ready = 0;
        }
        let stuck = drive_error(v).filter(|_| state == Some("CS_ERROR"));
        let mut ack_again = None;
        if let Some((err, n)) = &stuck
            && vcs
            && !self.acked_again.contains(n)
            && self.drive_pulses.get(n).is_some_and(|p| *p >= ACK_AGAIN_AFTER)
        {
            self.acked_again.push(*n);
            ack_again = Some((err.clone(), *n));
        }
        Step::Go {
            pulse,
            ack_again,
            stuck: stuck.map(|(_, n)| n),
        }
    }

    /// A pulse was sent (with robot `stuck` in a drive fault).
    pub(super) fn pulsed(&mut self, stuck: Option<u32>) {
        self.pulses += 1;
        if let Some(n) = stuck {
            *self.drive_pulses.entry(n).or_default() += 1;
        }
    }

    pub(super) fn drive_pulses(&self, n: u32) -> u32 {
        self.drive_pulses.get(&n).copied().unwrap_or(0)
    }
}

/// A TCP connect to `ip:port` from the PLC host: prints `open` or `closed`.
pub fn tcp_probe(ip: &str, port: u16) -> String {
    opc_ua_probe(ip, port)
}

/// The last (up to) three lines of a helper's output, `a | b | c`.
fn tail_of(lines: &[&str]) -> String {
    lines[lines.len().saturating_sub(3)..].join(" | ")
}

/// The head's fault when it is one of [`DRIVE_ERRORS`], with its robot (`nErrorRobot` >= 1).
fn drive_error(v: &BTreeMap<String, String>) -> Option<(String, u32)> {
    let last = |k: &str| v.get(k).map(|s| s.rsplit('.').next().unwrap_or(s).trim());
    let err = last(CELL_STATUS[1]).filter(|e| DRIVE_ERRORS.contains(e))?;
    let n = last(CELL_STATUS[2])?.parse().ok().filter(|n| *n >= 1)?;
    Some((err.to_string(), n))
}

/// `eCellState ERR_x robot n, recovery, cycles` from the values of [`CELL_STATUS`].
pub(super) fn status_text(v: &BTreeMap<String, String>) -> String {
    let g = |k: &str| {
        v.get(k)
            .map(|s| s.rsplit('.').next().unwrap_or(s).trim().to_string())
            .unwrap_or_else(|| "?".into())
    };
    let mut s = g(CELL_STATUS[0]);
    let err = g(CELL_STATUS[1]);
    if err != "ERR_NONE" && err != "0" {
        s.push_str(&format!(" {err} robot {}", g(CELL_STATUS[2])));
    }
    if g(CELL_STATUS[3]).eq_ignore_ascii_case("TRUE") {
        s.push_str(", recovery required");
    }
    s.push_str(&format!(", udiCycles {}", g(CELL_STATUS[4])));
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn helper_exit_codes_and_controller_numbers() {
        let (c, l) = exit_of("controller 1 is already in EXT ($MODE_OP=#EX)\np2c-exit=0\n");
        assert_eq!(c, Some(0));
        assert_eq!(l, ["controller 1 is already in EXT ($MODE_OP=#EX)"]);
        assert_eq!(exit_of("boom").0, None);
        assert_eq!(controller_number("krc2"), Some(2));
        assert_eq!(controller_number("krc0"), None);
        assert_eq!(controller_number("ns1"), None);
    }

    #[test]
    fn status_reads_like_the_operator_sees_it() {
        let v: BTreeMap<String, String> = [
            ("PLC_PRG.eCellState", "E_CellState.CS_ERROR"),
            ("PLC_PRG.fbCell.eError", "ERR_START_TIMEOUT"),
            ("PLC_PRG.fbCell.nErrorRobot", "2"),
            ("PLC_PRG.bRecoveryRequired", "FALSE"),
            ("PLC_PRG.udiCycles", "7"),
        ]
        .into_iter()
        .map(|(a, b)| (a.to_string(), b.to_string()))
        .collect();
        assert_eq!(
            status_text(&v),
            "CS_ERROR ERR_START_TIMEOUT robot 2, udiCycles 7"
        );
    }
}
