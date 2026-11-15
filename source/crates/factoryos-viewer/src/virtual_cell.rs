//! The workspace's own virtual cell (`factoryos-cell:ws-<alias>`) in the "Virtual cell" box: which
//! controls it has in which state, the phase stepper, the cost hint and the delete confirmation.
//! Pure functions of [`InfraStatus::virtual_cell`] / [`InfraStatus::binding`] and the
//! [`api::INFRA_CELLS`](factoryos_client::protocol::api::INFRA_CELLS) list; [`crate::infra_ui`]
//! draws them.
//!
//! An own cell's off mode ([`VirtualCell::off_mode`], chosen per cell): `terminate` (the default):
//! Stop terminates its instances (nothing is paid while it is off), Start launches them fresh from
//! the latest images and the backend restores the last deployed control code; `stop`: Stop stops
//! them (their disks are paid), Start resumes them, Start fresh launches new ones. By default a
//! workspace has none: Create virtual cell makes one (off).
//!
//! Templates (docs/tasks/cell-templates.md §8, §13): a backend that lists them
//! ([`CellsList::templates`]) gets a template picker and a form generated from the template's
//! parameters instead of the runtime pickers; a cell that reports its template
//! ([`VirtualCell::template`]) gates its controls by the template's capabilities. An older backend
//! or Lambda (neither) keeps today's controls, ungated.

use factoryos_client::protocol::infra::{
    CellCost, CellImage, CellKind, CellLifecycle, CellTemplateInfo, CellsList, InfraStatus, JobState, OFF_MODE_STOP, PowerState,
    TEMPLATE_EC2_CODESYS_KUKA, TemplateParameter, VirtualCell, role_label,
};
use serde_json::{Map, Value};

/// The start stepper: the Lambda's phases, then the backend's restore of the control code
/// ([`VirtualCell::restore`]) after `ready`.
pub const START_STEPS: [(&str, &str); 6] = [
    ("launching", "launching"),
    ("booting", "booting"),
    ("configuring", "configuring"),
    ("bring-up", "bring-up"),
    ("restoring", "restoring control code"),
    ("ready", "ready"),
];
/// The stop stepper (Stop terminates the instances).
pub const STOP_STEPS: [(&str, &str); 3] = [("archiving", "archiving"), ("terminating", "terminating"), ("off", "off")];
/// The stop stepper of a stop cell (Stop stops the instances).
pub const STOP_MODE_STEPS: [(&str, &str); 3] = [("archiving", "archiving"), ("stopping", "stopping"), ("off", "stopped")];
/// How long a resume usually takes to ready (estimated, not measured).
pub const USUAL_RESUME: &str = "usually 4–5 min";
/// How long a fresh start usually takes (launch, first boot, bring-up, the fast restore).
pub const USUAL_START: &str = "usually 6–8 min";
/// The cost of a running cell when the backend reports none (eu-central-1 on-demand: Windows IDE,
/// PLC host, 2 KUKA VC hosts, their IPv4 addresses).
pub const DEFAULT_USD_PER_HOUR: f64 = 2.65;

/// What the "Virtual cell" box shows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// An older server (no binding, no own cells): the shared cell's controls as before.
    Legacy,
    /// Bound to the shared `plan2cell` cell: its controls, plus Create own virtual cell.
    Shared,
    /// The workspace's own cell (or none yet).
    Own,
}

/// Where the own cell is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OwnState {
    /// No cell: Create virtual cell.
    None,
    /// Created, no instances.
    Off,
    /// On its way: the index into [`START_STEPS`].
    Starting(usize),
    Ready,
    /// On, but not ready, partly running or failed: Start brings it up again, Stop terminates it.
    Trouble,
    /// The index into [`STOP_STEPS`].
    Stopping(usize),
    Deleting,
    /// Its lifecycle failed.
    Failed,
    /// A lifecycle this viewer does not know.
    Unknown,
}

impl OwnState {
    /// It changes by itself (the box polls faster).
    pub fn transitional(self) -> bool {
        matches!(self, Self::Starting(_) | Self::Stopping(_) | Self::Deleting)
    }

    /// `off`, `starting (configuring)`, `ready`, ...: the state in words.
    pub fn label(self) -> String {
        match self {
            Self::None => "none".to_owned(),
            Self::Off => "off".to_owned(),
            Self::Starting(i) => format!("starting ({})", START_STEPS[i.min(START_STEPS.len() - 1)].1),
            Self::Ready => "ready".to_owned(),
            Self::Trouble => "running, not ready".to_owned(),
            Self::Stopping(i) => format!("stopping ({})", STOP_STEPS[i.min(STOP_STEPS.len() - 1)].1),
            Self::Deleting => "deleting".to_owned(),
            Self::Failed => "failed".to_owned(),
            Self::Unknown => "unknown".to_owned(),
        }
    }
}

/// Which buttons the own cell's box enables.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Controls {
    pub create: bool,
    pub start: bool,
    pub stop: bool,
    pub delete: bool,
}

/// What the box shows: by the backend's binding; without one (an older server, or no cell-control
/// function yet), by the cells list (a backend that can create an own cell and is not bound to the
/// shared one).
pub fn mode(status: Option<&InfraStatus>, cells: Option<&CellsList>) -> Mode {
    match status.and_then(|s| s.binding) {
        Some(CellKind::Own) => Mode::Own,
        Some(CellKind::Shared) => Mode::Shared,
        None => match cells {
            Some(list) if list.available && list.shared.is_none() => Mode::Own,
            Some(list) if list.available => Mode::Shared,
            _ => Mode::Legacy,
        },
    }
}

/// The own cell: from the status (the bus, fresher), else from the list.
pub fn own_cell<'a>(status: Option<&'a InfraStatus>, cells: Option<&'a CellsList>) -> Option<&'a VirtualCell> {
    status
        .and_then(|s| s.virtual_cell.as_ref())
        .or_else(|| cells.and_then(|l| l.cells.iter().find(|c| c.kind == CellKind::Own).or(l.cells.first())))
}

/// The restore of the control code runs (or waits).
fn restoring(cell: &VirtualCell) -> bool {
    cell.restore.as_ref().is_some_and(|r| r.state.busy())
}

pub fn own_state(cell: Option<&VirtualCell>) -> OwnState {
    let Some(cell) = cell else { return OwnState::None };
    if cell.lifecycle == CellLifecycle::Deleting {
        return OwnState::Deleting;
    }
    let phase = cell.power.as_ref().map(|p| p.phase.as_str()).unwrap_or_default();
    match phase {
        "archiving" => return OwnState::Stopping(0),
        "terminating" | "stopping" => return OwnState::Stopping(1),
        _ => {}
    }
    match cell.lifecycle {
        CellLifecycle::Off => return OwnState::Off,
        CellLifecycle::Failed => return OwnState::Failed,
        CellLifecycle::Unknown => return OwnState::Unknown,
        CellLifecycle::On | CellLifecycle::Deleting => {}
    }
    let Some(power) = &cell.power else { return OwnState::Starting(0) };
    match phase {
        "launching" | "starting-instances" | "off" | "" if power.state != PowerState::Ready => OwnState::Starting(0),
        "booting" => OwnState::Starting(1),
        "configuring" => OwnState::Starting(2),
        "bring-up" => OwnState::Starting(3),
        "not-ready" | "partial" | "failed" => OwnState::Trouble,
        _ => match power.state {
            PowerState::Ready if restoring(cell) => OwnState::Starting(4),
            PowerState::Ready => OwnState::Ready,
            PowerState::Booting => OwnState::Starting(1),
            PowerState::Stopping => OwnState::Stopping(1),
            PowerState::Off => OwnState::Starting(0),
            PowerState::Degraded | PowerState::Partial | PowerState::Failed | PowerState::Unknown => OwnState::Trouble,
        },
    }
}

/// The buttons of `state`; `can_create`: the list allows one ([`CellsList::can_create`]).
pub fn controls(state: OwnState, can_create: bool) -> Controls {
    let none = Controls::default();
    match state {
        OwnState::None => Controls { create: can_create, ..none },
        OwnState::Off | OwnState::Failed => Controls { start: true, delete: true, ..none },
        // Stop during a start terminates what was launched.
        OwnState::Starting(_) | OwnState::Ready => Controls { stop: true, ..none },
        OwnState::Trouble => Controls { start: true, stop: true, ..none },
        OwnState::Stopping(_) | OwnState::Deleting => none,
        OwnState::Unknown => Controls { delete: true, ..none },
    }
}

/// `launching ✓ · booting ✓ · configuring … · bring-up · restoring control code · ready`.
pub fn stepper_text(steps: &[(&str, &str)], current: usize, failed: bool) -> String {
    steps
        .iter()
        .enumerate()
        .map(|(i, (_, label))| {
            let mark = match i.cmp(&current) {
                std::cmp::Ordering::Less => " ✓",
                std::cmp::Ordering::Equal if failed => " ✗",
                std::cmp::Ordering::Equal if i + 1 < steps.len() => " …",
                _ => "",
            };
            format!("{label}{mark}")
        })
        .collect::<Vec<_>>()
        .join(" · ")
}

/// The running cost per hour (the backend's, else [`DEFAULT_USD_PER_HOUR`]).
pub fn usd_per_hour(cost: Option<&CellCost>) -> f64 {
    cost.map(|c| c.running_usd_per_hour).filter(|c| *c > 0.0).unwrap_or(DEFAULT_USD_PER_HOUR)
}

/// The hint next to Create virtual cell (and on Start).
pub fn cost_hint(cost: Option<&CellCost>) -> String {
    format!(
        "~${:.2}/h while on, $0 while off: Stop removes the instances, Start builds them fresh from the latest images (~6–8 min)",
        usd_per_hour(cost)
    )
}

/// The cost of what a stopped cell keeps: its disks (gp3; ~650 GiB with 2 robots).
pub fn stopped_usd_per_day(cost: Option<&CellCost>) -> f64 {
    cost.map(|c| c.stopped_usd_per_day).filter(|c| *c > 0.0).unwrap_or(650.0 * 0.0952 / 30.0)
}

/// The cost hint of a cell by its off mode.
pub fn mode_cost_hint(cell: &VirtualCell) -> String {
    if !cell.stop_mode() {
        return cost_hint(Some(&cell.cost));
    }
    let gib = if cell.cost.ebs_gib > 0 { cell.cost.ebs_gib } else { 650 };
    format!(
        "~${:.2}/h while on; while stopped only its disks ({gib} GiB, ~${:.2}/day): Stop stops the instances, Start resumes them ({USUAL_RESUME} to ready)",
        usd_per_hour(Some(&cell.cost)),
        stopped_usd_per_day(Some(&cell.cost))
    )
}

/// The off-mode choice in words.
pub fn off_mode_label(mode: &str) -> &'static str {
    if mode == OFF_MODE_STOP { "Stop keeps the instances (resume)" } else { "Stop terminates (fresh start)" }
}

/// `3 d` / `5 h` / `12 min` between `since_s` and `now`.
pub fn age_text(since_s: f64, now: f64) -> String {
    let s = (now - since_s).max(0.0);
    if s >= 86_400.0 {
        format!("{} d", (s / 86_400.0).floor() as u64)
    } else if s >= 3600.0 {
        format!("{} h", (s / 3600.0).floor() as u64)
    } else {
        format!("{} min", (s / 60.0).floor() as u64)
    }
}

/// A stopped cell's lines: since when, the age of its images, its disks' cost, what the next Start does.
pub fn stopped_lines(cell: &VirtualCell, now: f64) -> Vec<String> {
    let mut out = Vec::new();
    if !cell.stopped {
        return out;
    }
    let resume = cell.resume.clone().unwrap_or_default();
    let since = resume.stopped_at.as_deref().and_then(iso_s).map(|s| format!(" for {}", age_text(s, now))).unwrap_or_default();
    out.push(format!("Stopped{since}: its disks cost ~${:.2}/day.", stopped_usd_per_day(Some(&cell.cost))));
    if let Some(date) = resume.image_date.as_deref() {
        let age = iso_s(date).map(|d| format!(", {} old", age_text(d, now))).unwrap_or_default();
        out.push(format!("Its images: built {}{age}.", date.split('T').next().unwrap_or(date)));
    }
    if !resume.newer_images.is_empty() {
        let list: Vec<String> = resume.newer_images.iter().map(|n| format!("{} {} → {}", role_label(&n.role), n.running, n.latest)).collect();
        out.push(format!("Newer images: {} (Resume keeps the old ones, Start fresh takes them).", list.join(", ")));
    }
    if let Some(t) = &resume.newer_template {
        out.push(format!("Newer template: {} {} → {} (Resume keeps the old one, Start fresh takes it).", t.id, t.running, t.latest));
    }
    if !resume.fresh_reasons.is_empty() {
        out.push(format!("The next Start is a fresh launch: {}.", resume.fresh_reasons.join("; ")));
    }
    out
}

/// A stopped cell resumes on its next Start (no fresh launch is due).
pub fn resumes(cell: &VirtualCell) -> bool {
    cell.stopped && cell.resume.as_ref().is_none_or(|r| r.possible)
}

/// Offer Start fresh: a stopped cell, or a resume that failed (its power or its restore).
pub fn offer_fresh(cell: &VirtualCell) -> bool {
    let failed_resume = cell.power.as_ref().is_some_and(|p| {
        p.session.as_ref().is_some_and(|s| s.resume) && matches!(p.phase.as_str(), "failed" | "not-ready" | "partial")
    }) || cell.restore.as_ref().is_some_and(|r| r.mode == "resume" && r.state == JobState::Failed);
    resumes(cell) || failed_resume
}

/// The stop stepper of the cell: by the stop under way, else by its off mode.
pub fn stop_steps(cell: &VirtualCell) -> &'static [(&'static str, &'static str); 3] {
    let mode = cell.power.as_ref().and_then(|p| p.session.as_ref()).filter(|s| s.op == "off").and_then(|s| s.mode.clone());
    match mode.as_deref() {
        Some("stop") => &STOP_MODE_STEPS,
        Some(_) => &STOP_STEPS,
        None if cell.stop_mode() => &STOP_MODE_STEPS,
        None => &STOP_STEPS,
    }
}

/// The state in words, a stop cell's own (`stopped`, `stopping (stopping)`).
pub fn own_label(cell: Option<&VirtualCell>, state: OwnState) -> String {
    match (cell, state) {
        (Some(c), OwnState::Off) if c.stopped => "stopped".to_owned(),
        (Some(c), OwnState::Stopping(i)) => format!("stopping ({})", stop_steps(c)[i.min(2)].1),
        _ => state.label(),
    }
}

/// `This session so far: $1.23 (~$2.65/h while on)`, if the backend reports it.
pub fn session_line(cost: &CellCost) -> Option<String> {
    cost.session_usd.map(|usd| format!("This session so far: ${usd:.2} (~${:.2}/h while on)", usd_per_hour(Some(cost))))
}

/// `PLC host 1.0.3 (2026-10-14)`.
pub fn image_line(image: &CellImage) -> String {
    let version = if image.recipe_version.is_empty() { "unknown version" } else { image.recipe_version.as_str() };
    let date = image.build_date.as_deref().map(|d| format!(" ({})", d.split('T').next().unwrap_or(d))).unwrap_or_default();
    format!("{} {version}{date}", role_label(&image.role))
}

/// The delete confirmation: the cell id typed again.
pub fn delete_confirmed(typed: &str, id: &str) -> bool {
    !id.is_empty() && typed.trim() == id
}

/// Unix seconds of an ISO 8601 UTC time (`2026-10-15T08:01:40.000Z`, `…+00:00`, no zone).
pub fn iso_s(iso: &str) -> Option<f64> {
    let (date, time) = iso.split_once('T')?;
    let mut d = date.split('-').map(|p| p.parse::<i64>().ok());
    let (y, m, day) = (d.next()??, d.next()??, d.next()??);
    if !(1..=12).contains(&m) || !(1..=31).contains(&day) {
        return None;
    }
    let time = time.trim_end_matches('Z');
    let time = time.split_once('+').map_or(time, |(t, _)| t);
    let mut t = time.split(':');
    let (h, min) = (t.next()?.parse::<i64>().ok()?, t.next()?.parse::<i64>().ok()?);
    let s = t.next().map_or(Some(0.0), |s| s.parse::<f64>().ok())?;
    // Days from 1970-01-01 (Howard Hinnant's days_from_civil).
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + day - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146_097 + doe - 719_468;
    Some((days * 86_400 + h * 3600 + min * 60) as f64 + s)
}

/// `12 min` / `1 h 05 min` since `since_s`.
pub fn elapsed_text(since_s: f64, now: f64) -> String {
    let min = ((now - since_s).max(0.0) / 60.0).floor() as u64;
    if min < 60 { format!("{min} min") } else { format!("{} h {:02} min", min / 60, min % 60) }
}

/// When the current start / stop began (the Lambda's session), Unix seconds.
pub fn session_since(cell: &VirtualCell) -> Option<f64> {
    let power = cell.power.as_ref()?;
    power.session.as_ref().and_then(|s| s.requested_at.as_deref()).or(power.since.as_deref()).and_then(iso_s)
}

/// Why the cell's jobs cannot run because of the own cell (none, off, deleting, starting,
/// restoring), else because of its power ([`InfraStatus::power_blocks`]); `None`: usable.
pub fn blocks(status: &InfraStatus) -> Option<String> {
    if status.binding == Some(CellKind::Own) {
        let cell = status.virtual_cell.as_ref();
        match own_state(cell) {
            OwnState::None => return Some("No virtual cell yet: Create virtual cell (in the Virtual cell box)".to_owned()),
            OwnState::Off => return Some("The virtual cell is off: Start virtual cell first (in the Virtual cell box)".to_owned()),
            OwnState::Deleting => return Some("The virtual cell is being deleted".to_owned()),
            OwnState::Failed => return Some("The virtual cell failed: Start it again, or Delete it".to_owned()),
            OwnState::Stopping(_) => return Some("The virtual cell is stopping (its instances are terminated)".to_owned()),
            OwnState::Starting(4) => return Some("Wait: the last control code is being restored onto the fresh cell".to_owned()),
            OwnState::Starting(i) => {
                return Some(format!("The virtual cell is starting ({}): wait until it is ready ({USUAL_START})", START_STEPS[i].1));
            }
            OwnState::Ready | OwnState::Trouble | OwnState::Unknown => {}
        }
    }
    status.power_blocks()
}

/// The restore line: `Restoring the control code (fast): …`, `✓ Control code restored (…)`.
pub fn restore_text(cell: &VirtualCell) -> Option<(JobState, String)> {
    let r = cell.restore.as_ref()?;
    if r.state == JobState::Idle {
        return None;
    }
    let what = match r.mode.as_str() {
        "fast" => "boot applications restored, KRL and start",
        "full" => "Deploy only",
        "none" => "nothing restored",
        other => other,
    };
    let of = match (r.build.as_deref(), r.plan_id.as_deref()) {
        (Some(b), Some(p)) => format!(" of {} (PLAN_ID {p})", crate::code_export::export_label(b)),
        (Some(b), None) => format!(" of {}", crate::code_export::export_label(b)),
        _ => String::new(),
    };
    let head = match r.state {
        JobState::Waiting | JobState::Running => "Restoring the control code",
        JobState::Done => "Control code restored",
        JobState::Failed => "Restoring the control code failed",
        JobState::Idle => "",
    };
    let message = r.message.as_deref().map(|m| format!(": {m}")).unwrap_or_default();
    Some((r.state, format!("{head}{of} ({what}){message}")))
}

/// The backend lists templates and they are on for this workspace: Create virtual cell picks one
/// (else the runtime pickers, as an older backend or Lambda, or templates off, have them; with
/// templates off the Lambda still lists the builtin legacy templates).
pub fn templates_offered(cells: Option<&CellsList>) -> bool {
    cells.is_some_and(|l| l.templates_enabled && !l.templates.is_empty())
}

/// The template Create virtual cell takes: `chosen` if it is listed, else today's default (the EC2
/// cell) if it is available, else the first available one, else the first listed.
pub fn create_template<'a>(cells: &'a CellsList, chosen: Option<&str>) -> Option<&'a CellTemplateInfo> {
    let list = &cells.templates;
    chosen
        .and_then(|id| list.iter().find(|t| t.id == id))
        .or_else(|| list.iter().find(|t| t.id == TEMPLATE_EC2_CODESYS_KUKA && t.available))
        .or_else(|| list.iter().find(|t| t.available))
        .or(list.first())
}

/// Why a template cannot be picked (`unavailable: …`); `None`: it can.
pub fn unavailable_text(t: &CellTemplateInfo) -> Option<String> {
    if t.available && !t.beyond_consumer() {
        return None;
    }
    let why = match t.reason.as_deref() {
        Some("engine") => "needs a newer cell-control Lambda",
        Some("runtime") => "this account has no EKS cluster for it",
        Some("denied") => "switched off by the operators",
        Some("unavailable") => "it could not be loaded",
        Some("consumer") => "needs a newer backend",
        Some(other) => other,
        None if t.beyond_consumer() => "needs a newer backend",
        None => "not offered",
    };
    Some(format!("unavailable: {why}"))
}

/// The cost hint of a template (at its default parameters), as [`cost_hint`].
pub fn template_cost_hint(t: &CellTemplateInfo) -> String {
    cost_hint(Some(&CellCost { running_usd_per_hour: t.cost.running_usd_per_hour, ..Default::default() }))
}

/// Every parameter of `t` at its default.
pub fn template_defaults(t: &CellTemplateInfo) -> Map<String, Value> {
    t.parameters.iter().map(|(name, p)| (name.clone(), p.default.clone())).collect()
}

/// The cell's parameter values: as its template resolved them, else as its record has them.
pub fn cell_params(cell: &VirtualCell) -> Map<String, Value> {
    match &cell.template {
        Some(t) if !t.parameters.is_empty() => t.parameters.clone(),
        _ => cell.params.clone().unwrap_or_default(),
    }
}

/// The cell offers what `capability` gates: by its template; a cell without one (an older Lambda,
/// templates off) as today, where nothing is gated.
pub fn allows(cell: &VirtualCell, capability: &str) -> bool {
    cell.template.as_ref().is_none_or(|t| t.has(capability))
}

/// [`allows`] for the cell this backend is bound to: only an own cell has a template (the shared
/// cell, or none: as today).
pub fn status_allows(status: Option<&InfraStatus>, capability: &str) -> bool {
    status
        .filter(|s| s.binding == Some(CellKind::Own))
        .and_then(|s| s.virtual_cell.as_ref())
        .is_none_or(|c| allows(c, capability))
}

/// Off and not stopped: a runtime (or template) change, and robots, wait for that today.
fn off_unstopped(cell: &VirtualCell) -> bool {
    cell.phase() == "off" && !cell.stopped
}

/// Whether the cell may get another template now (`Err`: why not).
pub fn template_changeable(cell: &VirtualCell) -> Result<(), &'static str> {
    if off_unstopped(cell) { Ok(()) } else { Err("The template changes only while the cell is off (not stopped)") }
}

/// Whether parameter `p` may change now (`Err`: why not): `always` any time, `never` only at
/// create, `off` (and a value this viewer does not know) only while the cell is off, not stopped.
pub fn param_changeable(p: &TemplateParameter, cell: &VirtualCell) -> Result<(), &'static str> {
    match p.mutable.as_str() {
        "always" => Ok(()),
        "never" => Err("Set when the cell is created only"),
        _ if off_unstopped(cell) => Ok(()),
        _ => Err("Changes only while the cell is off (not stopped)"),
    }
}

/// A `networks` value as (name, robots) pairs (anything else: none).
pub fn networks_value(value: &Value) -> Vec<(String, i64)> {
    let Some(list) = value.as_array() else { return Vec::new() };
    list.iter().filter_map(|n| Some((n.get("name")?.as_str()?.to_owned(), n.get("robots")?.as_i64()?))).collect()
}

/// `count` networks n1..: the ones in `have` keep their robots, new ones get `robots`.
pub fn networks_json(count: usize, have: &[(String, i64)], robots: i64) -> Value {
    let nets = (1..=count).map(|i| {
        let name = format!("n{i}");
        let robots = have.iter().find(|(n, _)| *n == name).map_or(robots, |(_, r)| *r);
        serde_json::json!({"name": name, "robots": robots})
    });
    Value::Array(nets.collect())
}

/// `Template ec2-codesys-kuka@1.0.0 (s3, stable, launch 7)`: what the cell runs (without a launch:
/// what its next launch resolves to).
pub fn template_line(cell: &VirtualCell) -> Option<String> {
    let t = cell.template.as_ref().filter(|t| !t.id.is_empty())?;
    let mut about: Vec<String> = [t.source.as_str(), t.channel.as_deref().unwrap_or_default()]
        .into_iter()
        .filter(|s| !s.is_empty())
        .map(str::to_owned)
        .collect();
    about.push(t.launch.map_or_else(|| "next launch".to_owned(), |n| format!("launch {n}")));
    Some(format!("Template {} ({})", t.label(), about.join(", ")))
}

#[cfg(test)]
mod tests {
    use factoryos_client::protocol::infra::{CellPower, CellRestore, PowerSession};

    use super::*;

    fn cell(lifecycle: CellLifecycle, power: Option<(PowerState, &str)>) -> VirtualCell {
        VirtualCell {
            id: "ws-jdoe".into(),
            workspace: "jdoe".into(),
            lifecycle,
            power: power.map(|(state, phase)| CellPower { state, phase: phase.into(), ..Default::default() }),
            ..Default::default()
        }
    }

    #[test]
    fn the_box_follows_the_binding() {
        let own = InfraStatus { binding: Some(CellKind::Own), ..Default::default() };
        let shared = InfraStatus { binding: Some(CellKind::Shared), ..Default::default() };
        let old = InfraStatus::default();
        assert_eq!(mode(Some(&own), None), Mode::Own);
        assert_eq!(mode(Some(&shared), None), Mode::Shared);
        assert_eq!(mode(Some(&old), None), Mode::Legacy);
        assert_eq!(mode(None, None), Mode::Legacy);
        // No binding (yet): by the list.
        let list = CellsList { available: true, can_create: true, ..Default::default() };
        assert_eq!(mode(Some(&old), Some(&list)), Mode::Own);
        let bound_shared = CellsList { shared: Some("plan2cell".into()), ..list.clone() };
        assert_eq!(mode(Some(&old), Some(&bound_shared)), Mode::Shared);
        assert_eq!(mode(Some(&old), Some(&CellsList::default())), Mode::Legacy);
        // The status's cell before the list's.
        let listed = CellsList { cells: vec![cell(CellLifecycle::Off, None)], ..list };
        let mut on = cell(CellLifecycle::On, Some((PowerState::Ready, "ready")));
        on.id = "ws-bus".into();
        let status = InfraStatus { virtual_cell: Some(on), ..own };
        assert_eq!(own_cell(Some(&status), Some(&listed)).unwrap().id, "ws-bus");
        assert_eq!(own_cell(Some(&old), Some(&listed)).unwrap().id, "ws-jdoe");
        assert!(own_cell(Some(&old), None).is_none());
    }

    #[test]
    fn each_state_has_its_controls() {
        use OwnState::{Deleting, Failed, Off, Ready, Starting, Stopping, Trouble, Unknown};
        assert_eq!(own_state(None), OwnState::None);
        assert_eq!(controls(OwnState::None, true), Controls { create: true, ..Default::default() });
        assert_eq!(controls(OwnState::None, false), Controls::default(), "the cap or no function: no Create");

        let off = cell(CellLifecycle::Off, None);
        assert_eq!(own_state(Some(&off)), Off);
        assert_eq!(controls(Off, true), Controls { start: true, delete: true, ..Default::default() });

        for (phase, step) in [("launching", 0), ("starting-instances", 0), ("booting", 1), ("configuring", 2), ("bring-up", 3)] {
            let c = cell(CellLifecycle::On, Some((PowerState::Booting, phase)));
            assert_eq!(own_state(Some(&c)), Starting(step), "{phase}");
        }
        assert_eq!(own_state(Some(&cell(CellLifecycle::On, None))), Starting(0), "launched, no power read yet");
        assert_eq!(controls(Starting(2), true), Controls { stop: true, ..Default::default() });

        let mut ready = cell(CellLifecycle::On, Some((PowerState::Ready, "ready")));
        assert_eq!(own_state(Some(&ready)), Ready);
        assert_eq!(controls(Ready, true), Controls { stop: true, ..Default::default() });
        ready.restore = Some(CellRestore { state: JobState::Running, mode: "fast".into(), ..Default::default() });
        assert_eq!(own_state(Some(&ready)), Starting(4), "restoring the control code");
        ready.restore.as_mut().unwrap().state = JobState::Done;
        assert_eq!(own_state(Some(&ready)), Ready);

        for (state, phase) in [(PowerState::Degraded, "not-ready"), (PowerState::Partial, "partial"), (PowerState::Failed, "failed")] {
            assert_eq!(own_state(Some(&cell(CellLifecycle::On, Some((state, phase))))), Trouble);
        }
        assert_eq!(controls(Trouble, true), Controls { start: true, stop: true, ..Default::default() });

        assert_eq!(own_state(Some(&cell(CellLifecycle::On, Some((PowerState::Stopping, "archiving"))))), Stopping(0));
        assert_eq!(own_state(Some(&cell(CellLifecycle::On, Some((PowerState::Stopping, "terminating"))))), Stopping(1));
        // The lifecycle may already say off while the instances terminate.
        assert_eq!(own_state(Some(&cell(CellLifecycle::Off, Some((PowerState::Stopping, "terminating"))))), Stopping(1));
        assert_eq!(own_state(Some(&cell(CellLifecycle::Off, Some((PowerState::Off, "off"))))), Off);
        assert_eq!(controls(Stopping(0), true), Controls::default());

        assert_eq!(own_state(Some(&cell(CellLifecycle::Deleting, Some((PowerState::Stopping, "terminating"))))), Deleting);
        assert_eq!(controls(Deleting, true), Controls::default());
        assert_eq!(own_state(Some(&cell(CellLifecycle::Failed, None))), Failed);
        assert_eq!(controls(Failed, true), Controls { start: true, delete: true, ..Default::default() });
        assert_eq!(own_state(Some(&cell(CellLifecycle::Unknown, None))), Unknown);
        assert_eq!(controls(Unknown, true), Controls { delete: true, ..Default::default() });

        assert!(Starting(0).transitional() && Stopping(1).transitional() && Deleting.transitional());
        assert!(!Off.transitional() && !Ready.transitional() && !OwnState::None.transitional());
        assert_eq!(Starting(4).label(), "starting (restoring control code)");
        assert_eq!(Stopping(1).label(), "stopping (terminating)");
    }

    #[test]
    fn the_stepper() {
        assert_eq!(
            stepper_text(&START_STEPS, 2, false),
            "launching ✓ · booting ✓ · configuring … · bring-up · restoring control code · ready"
        );
        assert_eq!(stepper_text(&START_STEPS, 5, false), "launching ✓ · booting ✓ · configuring ✓ · bring-up ✓ · restoring control code ✓ · ready");
        assert_eq!(stepper_text(&STOP_STEPS, 0, false), "archiving … · terminating · off");
        assert_eq!(stepper_text(&STOP_STEPS, 1, true), "archiving ✓ · terminating ✗ · off");
    }

    #[test]
    fn the_delete_needs_the_cell_id() {
        assert!(delete_confirmed("ws-jdoe", "ws-jdoe"));
        assert!(delete_confirmed(" ws-jdoe\n", "ws-jdoe"), "pasted with spaces");
        assert!(!delete_confirmed("ws-jdo", "ws-jdoe"));
        assert!(!delete_confirmed("WS-JDOE", "ws-jdoe"));
        assert!(!delete_confirmed("", ""), "no id: never");
    }

    #[test]
    fn costs_and_images() {
        assert_eq!(
            cost_hint(None),
            "~$2.65/h while on, $0 while off: Stop removes the instances, Start builds them fresh from the latest images (~6–8 min)"
        );
        let cost = CellCost { running_usd_per_hour: 2.7, session_usd: Some(1.234), ..Default::default() };
        assert!(cost_hint(Some(&cost)).starts_with("~$2.70/h"));
        assert_eq!(session_line(&cost).as_deref(), Some("This session so far: $1.23 (~$2.70/h while on)"));
        assert_eq!(session_line(&CellCost::default()), None);
        let image = CellImage { role: "kukaVc1".into(), recipe_version: "1.0.3".into(), build_date: Some("2026-10-14T09:00:00Z".into()), ..Default::default() };
        assert_eq!(image_line(&image), "KUKA VC host 1 1.0.3 (2026-10-14)");
        assert_eq!(image_line(&CellImage { role: "plcHost".into(), ..Default::default() }), "PLC host unknown version");
    }

    #[test]
    fn stop_mode_lines_and_controls() {
        use factoryos_client::protocol::infra::{CellResume, NewerImage, PowerSession};
        let now = iso_s("2026-10-19T08:00:00Z").unwrap();
        let mut c = cell(CellLifecycle::Off, Some((PowerState::Off, "off")));
        assert!(stopped_lines(&c, now).is_empty() && !offer_fresh(&c) && !resumes(&c));
        assert!(mode_cost_hint(&c).starts_with("~$2.65/h while on, $0 while off"));
        c.off_mode = "stop".into();
        c.stopped = true;
        c.cost.stopped_usd_per_day = 2.06;
        c.cost.ebs_gib = 650;
        c.resume = Some(CellResume {
            possible: true,
            stopped_at: Some("2026-10-17T07:00:00Z".into()),
            image_date: Some("2026-10-10T20:00:00Z".into()),
            newer_images: vec![NewerImage { role: "plcHost".into(), running: "1.0.0".into(), latest: "1.0.1".into(), build_date: None }],
            ..Default::default()
        });
        assert!(mode_cost_hint(&c).contains("650 GiB, ~$2.06/day"));
        assert_eq!(
            stopped_lines(&c, now),
            [
                "Stopped for 2 d: its disks cost ~$2.06/day.",
                "Its images: built 2026-10-10, 8 d old.",
                "Newer images: PLC host 1.0.0 → 1.0.1 (Resume keeps the old ones, Start fresh takes them).",
            ]
        );
        assert!(resumes(&c) && offer_fresh(&c));
        assert_eq!(own_label(Some(&c), OwnState::Off), "stopped");
        c.resume.as_mut().unwrap().possible = false;
        c.resume.as_mut().unwrap().fresh_reasons = vec!["stopped for 15 days".into()];
        assert!(!resumes(&c));
        assert!(stopped_lines(&c, now).last().unwrap().contains("fresh launch: stopped for 15 days"));
        // A failed resume offers Start fresh; the stop stepper follows the stop's mode.
        let mut failed = cell(CellLifecycle::On, Some((PowerState::Failed, "failed")));
        failed.power.as_mut().unwrap().session = Some(PowerSession { op: "on".into(), resume: true, ..Default::default() });
        assert!(offer_fresh(&failed));
        let mut stopping = cell(CellLifecycle::On, Some((PowerState::Stopping, "stopping")));
        assert_eq!(own_state(Some(&stopping)), OwnState::Stopping(1));
        stopping.power.as_mut().unwrap().session = Some(PowerSession { op: "off".into(), mode: Some("stop".into()), ..Default::default() });
        assert_eq!(own_label(Some(&stopping), OwnState::Stopping(1)), "stopping (stopping)");
        assert_eq!(stepper_text(stop_steps(&stopping), 2, false), "archiving ✓ · stopping ✓ · stopped");
        stopping.power.as_mut().unwrap().session.as_mut().unwrap().mode = Some("terminate".into());
        assert_eq!(own_label(Some(&stopping), OwnState::Stopping(1)), "stopping (terminating)");
        assert_eq!(age_text(0.0, 59.0 * 60.0), "59 min");
        assert_eq!(age_text(0.0, 5.0 * 3600.0), "5 h");
    }

    #[test]
    fn iso_times() {
        assert_eq!(iso_s("1970-01-01T00:00:00Z"), Some(0.0));
        assert_eq!(iso_s("2026-10-15T08:01:40.000Z"), Some(1_792_051_300.0));
        assert_eq!(iso_s("2000-03-01T00:00:00+00:00"), Some(951_868_800.0));
        assert_eq!(iso_s("soon"), None);
        assert_eq!(iso_s("2026-13-01T00:00:00Z"), None);
        assert_eq!(elapsed_text(0.0, 12.0 * 60.0 + 59.0), "12 min");
        assert_eq!(elapsed_text(0.0, 65.0 * 60.0), "1 h 05 min");
        let mut c = cell(CellLifecycle::On, Some((PowerState::Booting, "booting")));
        assert_eq!(session_since(&c), None);
        c.power.as_mut().unwrap().session = Some(PowerSession { requested_at: Some("1970-01-01T00:10:00Z".into()), ..Default::default() });
        assert_eq!(session_since(&c), Some(600.0));
    }

    #[test]
    fn the_steps_wait_for_the_own_cell() {
        let mut s = InfraStatus { available: true, binding: Some(CellKind::Own), ..Default::default() };
        assert!(blocks(&s).unwrap().contains("Create virtual cell"));
        s.virtual_cell = Some(cell(CellLifecycle::Off, None));
        assert!(blocks(&s).unwrap().contains("Start virtual cell first"));
        s.virtual_cell = Some(cell(CellLifecycle::On, Some((PowerState::Booting, "configuring"))));
        assert!(blocks(&s).unwrap().contains("starting (configuring)"));
        let mut ready = cell(CellLifecycle::On, Some((PowerState::Ready, "ready")));
        ready.restore = Some(CellRestore { state: JobState::Running, ..Default::default() });
        s.virtual_cell = Some(ready.clone());
        assert!(blocks(&s).unwrap().contains("restored"));
        ready.restore = None;
        s.virtual_cell = Some(ready);
        assert_eq!(blocks(&s), None);
        // Shared: by the power only.
        let shared = InfraStatus { available: true, binding: Some(CellKind::Shared), ..Default::default() };
        assert_eq!(blocks(&shared), None);
    }

    #[test]
    fn the_restore_line() {
        let mut c = cell(CellLifecycle::On, Some((PowerState::Ready, "ready")));
        assert_eq!(restore_text(&c), None);
        c.restore = Some(CellRestore { state: JobState::Running, mode: "fast".into(), plan_id: Some("16#1".into()), ..Default::default() });
        let (state, text) = restore_text(&c).unwrap();
        assert_eq!(state, JobState::Running);
        assert_eq!(text, "Restoring the control code (boot applications restored, KRL and start)");
        c.restore = Some(CellRestore { state: JobState::Failed, mode: "full".into(), message: Some("deploy failed".into()), ..Default::default() });
        assert_eq!(restore_text(&c).unwrap().1, "Restoring the control code failed (Deploy only): deploy failed");
    }

    /// The backend's `GET cells` with the Lambda's `templates` (docs/tasks/cell-templates.md §8), abbreviated.
    fn listed() -> CellsList {
        serde_json::from_value(serde_json::json!({
            "available": true, "canCreate": true, "runtimes": ["ec2", "eks"], "templatesEnabled": true,
            "templates": [
                {"id": "eks-codesys-kuka", "version": "1.0.0", "channel": "stable", "source": "s3", "title": "CODESYS head PLC and KUKA VCs on EKS",
                 "runtime": "eks", "available": true, "cost": {"runningUsdPerHour": 2.72, "note": ""},
                 "parameters": {
                     "ide": {"default": "container", "label": "IDE", "mutable": "off", "type": "enum", "values": ["container", "ec2"]},
                     "networks": {"default": [{"name": "n1", "robots": 2}], "label": "PROFINET networks", "max": 4, "min": 1, "mutable": "off",
                                  "robots": {"default": 2, "max": 6, "min": 1}, "type": "networks"},
                     "offMode": {"default": "terminate", "label": "Stop", "mutable": "always", "type": "enum", "values": ["terminate"]}
                 },
                 "capabilities": [{"name": "ide.dcv", "onlyIf": {"param": "ide", "equals": "ec2"}}]},
                {"id": "ec2-codesys-kuka", "version": "1.0.0", "title": "CODESYS head PLC and KUKA VCs on EC2", "runtime": "ec2", "available": true,
                 "parameters": {"robots": {"default": 2, "label": "Robots", "max": 3, "min": 1, "mutable": "off", "type": "integer"}}},
                {"id": "m1-probe", "version": "0.0.1", "available": false, "reason": "engine"}
            ]
        }))
        .unwrap()
    }

    #[test]
    fn an_older_backend_keeps_the_runtime_pickers() {
        let old: CellsList = serde_json::from_value(serde_json::json!({"available": true, "canCreate": true, "runtimes": ["ec2", "eks"]})).unwrap();
        assert!(!templates_offered(Some(&old)) && !templates_offered(None));
        // Templates off (or the workspace not enabled): the Lambda lists the builtin legacy templates, the
        // runtime pickers stay.
        let off = CellsList { templates_enabled: false, ..listed() };
        assert!(!off.templates.is_empty() && !templates_offered(Some(&off)));
        assert!(create_template(&old, None).is_none());
        // A cell from an older Lambda: no template, nothing gated, no template line.
        let c = cell(CellLifecycle::Off, None);
        assert!(allows(&c, "bringup.action") && allows(&c, "restore") && allows(&c, "smartpad"));
        assert_eq!(template_line(&c), None);
        assert!(cell_params(&c).is_empty());
    }

    #[test]
    fn create_picks_a_template_and_its_defaults() {
        let list = listed();
        assert!(templates_offered(Some(&list)));
        // Today's default (EC2) before the list's order; a choice that is listed wins.
        assert_eq!(create_template(&list, None).unwrap().id, "ec2-codesys-kuka");
        assert_eq!(create_template(&list, Some("eks-codesys-kuka")).unwrap().id, "eks-codesys-kuka");
        assert_eq!(create_template(&list, Some("gone")).unwrap().id, "ec2-codesys-kuka");
        let mut no_ec2 = list.clone();
        no_ec2.templates[1].available = false;
        assert_eq!(create_template(&no_ec2, None).unwrap().id, "eks-codesys-kuka", "the first available one");

        let eks = &list.templates[0];
        assert_eq!(unavailable_text(eks), None);
        assert_eq!(unavailable_text(&list.templates[2]).as_deref(), Some("unavailable: needs a newer cell-control Lambda"));
        assert!(template_cost_hint(eks).starts_with("~$2.72/h while on"));
        assert!(template_cost_hint(&list.templates[1]).starts_with("~$2.65/h"), "no cost: the default");
        let defaults = template_defaults(eks);
        assert_eq!(defaults["ide"], "container");
        assert_eq!(defaults["networks"], serde_json::json!([{"name": "n1", "robots": 2}]));
        assert_eq!(defaults.len(), 3);
    }

    #[test]
    fn a_templates_cell_gates_by_its_capabilities() {
        use factoryos_client::protocol::infra::CellTemplateRef;
        let mut c = cell(CellLifecycle::Off, None);
        c.runtime = "eks".into();
        c.template = Some(CellTemplateRef {
            id: "eks-codesys-kuka".into(),
            version: "1.0.0".into(),
            source: "s3".into(),
            channel: Some("stable".into()),
            launch: Some(7),
            parameters: serde_json::from_value(serde_json::json!({"ide": "ec2", "networks": [{"name": "n1", "robots": 4}]})).unwrap(),
            capabilities: vec!["ide.dcv".into(), "runtime.codesys".into(), "smartpad".into()],
            ..Default::default()
        });
        assert!(allows(&c, "smartpad") && allows(&c, "runtime.codesys"));
        assert!(!allows(&c, "bringup.action") && !allows(&c, "restore") && !allows(&c, "archive"));
        assert_eq!(template_line(&c).as_deref(), Some("Template eks-codesys-kuka@1.0.0 (s3, stable, launch 7)"));
        assert_eq!(cell_params(&c)["ide"], "ec2");
        c.template.as_mut().unwrap().launch = None;
        assert!(template_line(&c).unwrap().ends_with("(s3, stable, next launch)"));

        // Only the bound own cell gates the shared steps.
        let own = InfraStatus { binding: Some(CellKind::Own), virtual_cell: Some(c.clone()), ..Default::default() };
        assert!(!status_allows(Some(&own), "bringup.action") && status_allows(Some(&own), "smartpad"));
        let shared = InfraStatus { binding: Some(CellKind::Shared), virtual_cell: Some(c.clone()), ..Default::default() };
        assert!(status_allows(Some(&shared), "bringup.action") && status_allows(None, "bringup.action"));
    }

    #[test]
    fn parameters_change_when_their_mutable_allows() {
        let param = |mutable: &str| TemplateParameter { mutable: mutable.into(), ..Default::default() };
        let off = cell(CellLifecycle::Off, None);
        let on = cell(CellLifecycle::On, Some((PowerState::Ready, "ready")));
        let mut stopped = cell(CellLifecycle::Off, Some((PowerState::Off, "off")));
        stopped.stopped = true;
        assert_eq!(param_changeable(&param("always"), &on), Ok(()));
        assert_eq!(param_changeable(&param("off"), &off), Ok(()));
        assert!(param_changeable(&param("off"), &on).is_err() && param_changeable(&param("off"), &stopped).is_err());
        assert!(param_changeable(&param("never"), &off).is_err());
        assert!(param_changeable(&param("later"), &on).is_err(), "unknown: as off");
        assert_eq!(template_changeable(&off), Ok(()));
        assert!(template_changeable(&on).is_err() && template_changeable(&stopped).is_err());
    }

    #[test]
    fn the_networks_editor_keeps_the_robots_it_has() {
        let have = networks_value(&serde_json::json!([{"name": "n1", "robots": 4}, {"name": "n2", "robots": 1}]));
        assert_eq!(have, [("n1".to_owned(), 4), ("n2".to_owned(), 1)]);
        assert!(networks_value(&serde_json::json!("n1")).is_empty());
        assert_eq!(networks_json(3, &have, 2), serde_json::json!([{"name": "n1", "robots": 4}, {"name": "n2", "robots": 1}, {"name": "n3", "robots": 2}]));
        assert_eq!(networks_json(1, &have, 2), serde_json::json!([{"name": "n1", "robots": 4}]));
    }

    #[test]
    fn a_stopped_cell_names_a_newer_template() {
        use factoryos_client::protocol::infra::{CellResume, NewerTemplate};
        let mut c = cell(CellLifecycle::Off, Some((PowerState::Off, "off")));
        c.stopped = true;
        c.resume = Some(CellResume {
            possible: true,
            newer_template: Some(NewerTemplate { id: "ec2-codesys-kuka".into(), running: "1.0.0".into(), latest: "1.1.0".into() }),
            ..Default::default()
        });
        assert_eq!(
            stopped_lines(&c, 0.0).last().unwrap(),
            "Newer template: ec2-codesys-kuka 1.0.0 → 1.1.0 (Resume keeps the old one, Start fresh takes it)."
        );
    }
}
