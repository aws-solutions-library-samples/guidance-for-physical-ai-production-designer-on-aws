//! `plan2cell cell runtime`: how long the CODESYS runtimes of the PLC host still run, read-only.
//!
//! The Control for Linux SL runtimes of the cell have no licence: each runs in demo mode and stops
//! about 2 h after it started (`codesyscontrol.log`: `no runtime license - running in demo
//! mode(~2 hours)` at the start, `performing shutdown in 1 hour` / `10 minutes` / ... before the
//! end). One remote command (ssh or SSM, [`crate::remote::Remote`]) reads, per container, `docker
//! inspect` (running, start time) and the demo / shutdown lines of its log (`docker exec cat`, or
//! `docker cp` from a stopped container), and the host's clock:
//!
//! ```text
//! stopsS  the latest shutdown warning of this run + its countdown       source "shutdown warning"
//!         else this run's demo line + 2 h                              source "demo line"
//!         else the container's start + 2 h                             source "container start"
//! ```
//!
//! A log line belongs to this run when its timestamp is not before the container's start (less
//! [`SLACK_S`]); lines without a readable timestamp are taken in log order (after the last demo
//! line). Nothing on the host is changed.
//!
//! SSM returns only the first 24,000 characters of stdout: the command keeps the last [`LINES`]
//! demo / countdown lines per container and ends with `end`. A container whose section was cut
//! (no `loglines`) counts as not read, and an answer without `end` is `truncated`.

use serde::{Deserialize, Serialize};

use crate::Result;
use crate::project::{Manifest, Role};
use crate::remote::Remote;
use crate::stages::sh;

pub const RUNTIME_SCHEMA: &str = "plan2cell/cell-runtime@1";
/// The demo period of an unlicensed runtime.
pub const DEMO_S: i64 = 2 * 3600;
/// A log line this much before the container's start still belongs to the run (clock steps).
pub const SLACK_S: i64 = 120;
/// The runtime's log inside its container.
pub const LOG: &str = "/var/opt/codesys/codesyscontrol.log";
/// The head PLC's container (when the containers are found on the host rather than in a build).
pub const HEAD_CONTAINER: &str = "codesys-master";
/// Without a build: the CODESYS containers of the PLC host (setup.sh, the robots overlay).
const DISCOVER: &str = "^codesys-(master|device)";
/// The demo / countdown lines kept per container (a few runs' worth; ~1.5 KB).
pub const LINES: usize = 12;

/// `plan2cell cell runtime --json`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct RuntimeReport {
    pub schema: String,
    /// Every runtime was read.
    pub ok: bool,
    /// The PLC host's clock (Unix seconds) when it was read.
    pub now_s: Option<i64>,
    pub runtimes: Vec<RuntimeStatus>,
    pub error: Option<String>,
    /// The answer was cut (no `end`: SSM's 24,000 characters): the runtimes after the cut are
    /// missing or not read.
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub truncated: bool,
    /// The planned remote command (`--dry-run`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub command: Option<String>,
}

impl RuntimeReport {
    pub fn head(&self) -> Option<&RuntimeStatus> {
        self.runtimes.iter().find(|r| r.head)
    }

    /// The running runtime that stops first.
    pub fn first_stop(&self) -> Option<&RuntimeStatus> {
        self.runtimes
            .iter()
            .filter(|r| r.running && r.stops_s.is_some())
            .min_by_key(|r| r.stops_s)
    }
}

/// One runtime container.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct RuntimeStatus {
    pub container: String,
    /// The head PLC (else a robot simulator).
    pub head: bool,
    /// `docker inspect` found the container.
    pub exists: bool,
    pub running: bool,
    /// `docker inspect` `.State.Status` (`running`, `exited`, `restarting`, ...).
    pub state: Option<String>,
    pub started_at: Option<String>,
    pub started_s: Option<i64>,
    pub restart_count: Option<u64>,
    /// The log has a demo-mode line (`None`: the log was not read).
    pub demo: Option<bool>,
    /// This run's demo line and its time.
    pub demo_line: Option<String>,
    pub demo_start_s: Option<i64>,
    /// This run's latest shutdown warning, its time, and the countdown it announces.
    pub warning: Option<String>,
    pub warning_s: Option<i64>,
    pub warning_left_s: Option<i64>,
    /// When the demo period ends (an estimate, see `source`) and how long until then (0: over).
    pub stops_s: Option<i64>,
    pub seconds_left: Option<i64>,
    /// What `stopsS` is derived from (`shutdown warning`, `demo line`, `container start`, ...).
    pub source: Option<String>,
}

impl RuntimeStatus {
    /// Running, with its demo period over (the runtime has stopped the application or will at
    /// any moment).
    pub fn expired(&self) -> bool {
        self.running && self.seconds_left == Some(0)
    }
}

/// The containers of a build (head first) or `None`: the CODESYS containers found on the host.
pub fn containers(m: Option<&Manifest>) -> Option<Vec<(String, bool)>> {
    let m = m?;
    let mut v: Vec<(String, bool)> = m
        .ordered()
        .into_iter()
        .map(|p| (p.runtime.container.clone(), p.role == Role::HeadPlc))
        .collect();
    v.sort_by_key(|(_, head)| !head);
    Some(v)
}

/// The one remote command: `now <unix>`, then per container `container`, `state ...` and
/// `log ...` lines, `loglines <n>`; `end` last.
pub fn command(containers: Option<&[(String, bool)]>) -> String {
    let list = match containers {
        Some(c) => c.iter().map(|(n, _)| sh(n)).collect::<Vec<_>>().join(" "),
        None => {
            format!("$(sudo docker ps -a --format '{{{{.Names}}}}' | grep -E '{DISCOVER}' | sort)")
        }
    };
    format!(
        "echo \"now $(date -u +%s)\"; for c in {list}; do echo \"container $c\"; \
         sudo docker inspect -f 'state {{{{.State.Running}}}} {{{{.State.Status}}}} \
         {{{{.State.StartedAt}}}} {{{{.RestartCount}}}}' \"$c\" 2>/dev/null || echo 'state missing'; \
         {{ sudo docker exec \"$c\" cat {LOG} 2>/dev/null || sudo docker cp \"$c\":{LOG} - 2>/dev/null \
         | tar -xOf - 2>/dev/null; }} | awk '{{ l = tolower($0) }} l ~ /demo mode|shutdown in/ \
         {{ print \"log \" $0 }} END {{ print \"loglines \" NR }}' | tail -n {}; done; echo end",
        LINES + 1
    )
}

/// Read the runtimes (`containers`: see [`containers`]). `now` is used when the host's clock
/// did not come back.
pub fn read(
    remote: &mut dyn Remote,
    containers: Option<&[(String, bool)]>,
    now: i64,
) -> Result<RuntimeReport> {
    let out = remote.run(&command(containers))?;
    Ok(parse(&out, containers, now))
}

/// A `factoryos/hook@1` head's runtime (`runtime.hook`, E7.2): `describe`, then `health`, whose
/// `runtime` is a `plan2cell/cell-runtime@1` report (no demo period, no auto-restart). A failed
/// answer, or a `runtime` of another schema, is `Err`.
pub fn read_hook(remote: &mut dyn Remote, wait: std::time::Duration) -> Result<RuntimeReport> {
    use crate::hook::{self, Role, Verb};
    let a = hook::described_call(remote, Role::Head, Verb::Health, &serde_json::json!({}), wait)?;
    if !a.ok() {
        return Err(crate::Error::Failed {
            stage: "runtime",
            code: a.stage_exit(),
            msg: a.failure("head"),
        });
    }
    from_hook_health(&a.json)
}

/// The `runtime` of a hook's `health` answer.
pub fn from_hook_health(answer: &serde_json::Value) -> Result<RuntimeReport> {
    let r: RuntimeReport = serde_json::from_value(answer["runtime"].clone()).map_err(|e| crate::Error::Failed {
        stage: "runtime",
        code: 4,
        msg: format!("hook health: runtime is not {RUNTIME_SCHEMA}: {e}"),
    })?;
    if r.schema != RUNTIME_SCHEMA {
        return Err(crate::Error::Failed {
            stage: "runtime",
            code: 4,
            msg: format!("hook health: runtime schema {:?}, want {RUNTIME_SCHEMA}", r.schema),
        });
    }
    Ok(r)
}

/// The report of the command's output.
pub fn parse(out: &str, containers: Option<&[(String, bool)]>, now: i64) -> RuntimeReport {
    let mut host_now = None;
    let mut end = false;
    // (status, log lines, lines the log had)
    let mut found: Vec<(RuntimeStatus, Vec<String>, Option<u64>)> = Vec::new();
    for l in out.lines() {
        let l = l.trim_end_matches('\r');
        if l == "end" {
            end = true;
        } else if let Some(n) = l.strip_prefix("now ") {
            host_now = n.trim().parse().ok();
        } else if let Some(c) = l.strip_prefix("container ") {
            let c = c.trim().to_string();
            let head = match containers {
                Some(cs) => cs.iter().any(|(n, h)| *h && *n == c),
                None => c == HEAD_CONTAINER,
            };
            found.push((
                RuntimeStatus {
                    container: c,
                    head,
                    ..Default::default()
                },
                Vec::new(),
                None,
            ));
        } else if let Some((s, lines, n)) = found.last_mut() {
            if let Some(st) = l.strip_prefix("state ") {
                let f: Vec<&str> = st.split_whitespace().collect();
                if f.first() != Some(&"missing") && f.len() >= 3 {
                    s.exists = true;
                    s.running = f[0] == "true";
                    s.state = Some(f[1].to_string());
                    s.started_at = Some(f[2].to_string());
                    s.started_s = iso_s(f[2]).filter(|t| *t > 0);
                    s.restart_count = f.get(3).and_then(|n| n.parse().ok());
                }
            } else if let Some(x) = l.strip_prefix("log ") {
                lines.push(x.to_string());
            } else if let Some(x) = l.strip_prefix("loglines ") {
                *n = x.trim().parse().ok();
            }
        }
    }
    let now = host_now.unwrap_or(now);
    let mut runtimes: Vec<RuntimeStatus> = found
        .into_iter()
        .map(|(mut s, lines, n)| {
            // No `loglines`: the section was cut, its newest lines are missing.
            let lines = if n.is_some() { lines } else { Vec::new() };
            estimate(&mut s, &lines, n.unwrap_or(0) > 0, now);
            s
        })
        .collect();
    // A container of the build the host does not list at all.
    if let Some(cs) = containers {
        for (c, head) in cs {
            if !runtimes.iter().any(|r| r.container == *c) {
                runtimes.push(RuntimeStatus {
                    container: c.clone(),
                    head: *head,
                    ..Default::default()
                });
            }
        }
    }
    // The head first.
    runtimes.sort_by_key(|r| !r.head);
    let ok = host_now.is_some();
    RuntimeReport {
        schema: RUNTIME_SCHEMA.into(),
        ok,
        now_s: host_now,
        error: (!ok).then(|| "the PLC host's answer had no clock (`now`)".to_string()),
        truncated: ok && !end,
        runtimes,
        command: None,
    }
}

/// The demo period of one runtime from its log lines (oldest first).
fn estimate(s: &mut RuntimeStatus, lines: &[String], log_read: bool, now: i64) {
    let started = s.started_s;
    let this_run = |t: Option<i64>| match (t, started) {
        (Some(t), Some(st)) => t >= st - SLACK_S,
        _ => true,
    };
    let demo = |l: &str| l.to_ascii_lowercase().contains("demo mode");
    let any_demo = lines.iter().any(|l| demo(l));
    let start = lines
        .iter()
        .enumerate()
        .rev()
        .find(|(_, l)| demo(l) && this_run(log_time(l)))
        .map(|(i, l)| (i, l.trim().to_string(), log_time(l)));
    s.demo = if any_demo {
        Some(true)
    } else if log_read {
        Some(false)
    } else {
        None
    };
    let after = start.as_ref().map_or(0, |(i, ..)| *i + 1);
    if let Some((_, line, t)) = &start {
        s.demo_line = Some(line.clone());
        s.demo_start_s = *t;
    }
    // The latest countdown of this run.
    let warning = lines[after.min(lines.len())..]
        .iter()
        .rev()
        .filter(|l| this_run(log_time(l)))
        .find_map(|l| countdown(l).map(|left| (l.trim().to_string(), log_time(l), left)));
    if let Some((line, t, left)) = &warning {
        s.warning = Some(line.clone());
        s.warning_s = *t;
        s.warning_left_s = Some(*left);
    }
    if !s.running || s.demo == Some(false) {
        return;
    }
    let (stops, source) = match (&warning, &start) {
        (Some((_, Some(t), left)), _) => (Some(t + left), "shutdown warning"),
        (_, Some((_, _, Some(t)))) => (Some(t + DEMO_S), "demo line"),
        _ if s.demo.is_none() => (
            started.map(|t| t + DEMO_S),
            "container start (log not read)",
        ),
        _ => (started.map(|t| t + DEMO_S), "container start"),
    };
    s.stops_s = stops;
    s.source = stops.map(|_| source.to_string());
    s.seconds_left = stops.map(|t| (t - now).max(0));
}

/// The countdown of a `shutdown in N hour(s) / minute(s) / second(s)` line, in seconds.
pub fn countdown(line: &str) -> Option<i64> {
    let l = line.to_ascii_lowercase();
    let rest = &l[l.find("shutdown in ")? + "shutdown in ".len()..];
    let digits: String = rest.chars().take_while(char::is_ascii_digit).collect();
    let n: i64 = digits.parse().ok()?;
    let unit = rest[digits.len()..].trim_start();
    let mul = if unit.starts_with('h') {
        3600
    } else if unit.starts_with("mi") {
        60
    } else if unit.starts_with('s') {
        1
    } else {
        return None;
    };
    Some(n * mul)
}

/// The timestamp a log line starts with (`2026-10-12T12:00:00Z, ...`, `2026-10-12 12:00:00`, or
/// Unix seconds / milliseconds).
pub fn log_time(line: &str) -> Option<i64> {
    let l = line.trim_start();
    let first: String = l
        .chars()
        .take_while(|c| !matches!(c, ',' | ';' | '|' | '[' | ']'))
        .collect();
    let first = first.trim();
    if first.len() >= 19 && first.as_bytes()[4] == b'-' && first.as_bytes()[7] == b'-' {
        return iso_s(&first.replacen(' ', "T", 1));
    }
    match first.parse::<i64>() {
        Ok(n) if (1_000_000_000..10_000_000_000).contains(&n) => Some(n),
        Ok(n) if (1_000_000_000_000..10_000_000_000_000).contains(&n) => Some(n / 1000),
        _ => None,
    }
}

/// `2026-10-12T08:00:00.123456789Z` (or `+00:00`, or without a zone: UTC) -> Unix seconds.
pub fn iso_s(text: &str) -> Option<i64> {
    let (date, time) = text.trim().split_once('T')?;
    let mut d = date.split('-');
    let (y, m, day): (i64, i64, i64) = (
        d.next()?.parse().ok()?,
        d.next()?.parse().ok()?,
        d.next()?.parse().ok()?,
    );
    if !(1..=12).contains(&m) || !(1..=31).contains(&day) {
        return None;
    }
    let (clock, offset) = match time.find(['Z', 'z', '+', '-']) {
        Some(i) => (&time[..i], &time[i..]),
        None => (time, ""),
    };
    let mut t = clock.split(':');
    let h: i64 = t.next()?.parse().ok()?;
    let min: i64 = t.next()?.parse().ok()?;
    let s: i64 = t
        .next()
        .map(|s| s.split('.').next().unwrap_or("0"))
        .unwrap_or("0")
        .parse()
        .ok()?;
    let off = match offset.as_bytes().first() {
        Some(b'+') | Some(b'-') => {
            let sign = if offset.starts_with('-') { -1 } else { 1 };
            let o = offset[1..].replace(':', "");
            let (oh, om) = (
                o.get(..2)?.parse::<i64>().ok()?,
                o.get(2..4).unwrap_or("00").parse::<i64>().ok()?,
            );
            sign * (oh * 3600 + om * 60)
        }
        _ => 0,
    };
    // Howard Hinnant's days_from_civil.
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + day - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146_097 + doe - 719_468;
    Some(days * 86_400 + h * 3600 + min * 60 + s - off)
}

/// `HH:MM UTC` of Unix seconds.
pub fn hhmm(s: i64) -> String {
    let s = s.rem_euclid(86_400);
    format!("{:02}:{:02} UTC", s / 3600, s / 60 % 60)
}

/// One line per runtime, for the terminal.
pub fn text(r: &RuntimeReport) -> Vec<String> {
    let mut out = Vec::new();
    for s in &r.runtimes {
        let role = if s.head { "head" } else { "sim " };
        let what = if !s.exists {
            "no such container".to_string()
        } else if !s.running {
            format!("not running ({})", s.state.as_deref().unwrap_or("?"))
        } else {
            match (s.stops_s, s.seconds_left) {
                (Some(at), Some(0)) => format!(
                    "demo period over since {} ({})",
                    hhmm(at),
                    s.source.as_deref().unwrap_or("?")
                ),
                (Some(at), Some(left)) => format!(
                    "stops at {} (in {} min; {})",
                    hhmm(at),
                    (left + 59) / 60,
                    s.source.as_deref().unwrap_or("?")
                ),
                _ if s.demo == Some(false) => {
                    "running, no demo-mode line in its log (licensed?)".into()
                }
                _ => "running, stop time unknown".into(),
            }
        };
        let since = s
            .started_s
            .map(|t| {
                format!(
                    ", {} {}",
                    if s.running {
                        "up since"
                    } else {
                        "last started"
                    },
                    hhmm(t)
                )
            })
            .unwrap_or_default();
        out.push(format!("{role} {:<24} {what}{since}", s.container));
        if let Some(w) = &s.warning {
            out.push(format!("     last warning: {w}"));
        }
    }
    if r.truncated {
        out.push(
            "note: the PLC host's answer was cut (SSM output limit): later runtimes are incomplete"
                .into(),
        );
    }
    if let Some(e) = &r.error {
        out.push(format!("error: {e}"));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::remote::{Call, MockRemote};

    #[test]
    fn times_parse() {
        assert_eq!(iso_s("1970-01-02T00:00:01Z"), Some(86_401));
        assert_eq!(iso_s("2026-10-12T08:00:00.123456789Z"), Some(1_791_792_000));
        assert_eq!(iso_s("2026-10-12T10:00:00+02:00"), Some(1_791_792_000));
        assert_eq!(iso_s("0001-01-01T00:00:00Z").filter(|t| *t > 0), None);
        assert_eq!(iso_s("garbage"), None);
        assert_eq!(
            log_time("2026-10-12T08:00:00Z, 0x0000100d, 1, 0, 0, no runtime license"),
            Some(1_791_792_000)
        );
        assert_eq!(log_time("2026-10-12 08:00:00, x"), Some(1_791_792_000));
        assert_eq!(log_time("1791792000, 0x1, demo mode"), Some(1_791_792_000));
        assert_eq!(
            log_time("1791792000123, 0x1, demo mode"),
            Some(1_791_792_000)
        );
        assert_eq!(
            log_time("!!!! Warning: performing shutdown in 1 hour"),
            None
        );
        assert_eq!(hhmm(1_791_792_000 + 7200), "10:00 UTC");
    }

    #[test]
    fn countdowns() {
        assert_eq!(
            countdown("!!!! Warning: WARNING: performing shutdown in 1 hour"),
            Some(3600)
        );
        assert_eq!(countdown("performing shutdown in 2 hours"), Some(7200));
        assert_eq!(
            countdown("WARNING: performing shutdown in 10 minutes"),
            Some(600)
        );
        assert_eq!(countdown("performing shutdown in 1 minute"), Some(60));
        assert_eq!(countdown("performing shutdown in 30 seconds"), Some(30));
        assert_eq!(countdown("performing shutdown now"), None);
        assert_eq!(
            countdown("no runtime license - running in demo mode(~2 hours)"),
            None
        );
    }

    const T0: i64 = 1_791_792_000; // 2026-10-12T08:00:00Z

    fn out() -> String {
        [
            "now 1791797400", // 09:30
            "container codesys-master",
            "state true running 2026-10-12T08:00:03.5Z 2",
            // The previous run's lines (before this start) do not count.
            "log 2026-10-12T05:00:00Z, 0x1, 1, 0, 0, no runtime license - running in demo mode(~2 hours)",
            "log 2026-10-12T06:00:00Z, 0x1, 1, 0, 0, !!!! Warning: WARNING: performing shutdown in 1 hour",
            "log 2026-10-12T08:00:05Z, 0x1, 1, 0, 0, no runtime license - running in demo mode(~2 hours)",
            "log 2026-10-12T09:00:05Z, 0x1, 1, 0, 0, !!!! Warning: WARNING: performing shutdown in 1 hour",
            "loglines 812",
            "container codesys-device-robot1",
            "state true running 2026-10-12T08:30:00Z 0",
            "log 2026-10-12T08:30:02Z, 0x1, 1, 0, 0, no runtime license - running in demo mode(~2 hours)",
            "loglines 90",
            "container codesys-device-robot2",
            "state false exited 2026-10-12T06:00:00Z 0",
            "log 2026-10-12T06:00:01Z, no runtime license - running in demo mode(~2 hours)",
            "log 2026-10-12T07:50:01Z, performing shutdown in 10 minutes",
            "loglines 40",
            "container codesys-device-robot3",
            "state true running 2026-10-12T09:00:00Z 0",
            "loglines 0",
            "end",
        ]
        .join("\n")
    }

    #[test]
    fn the_demo_period_from_warnings_demo_lines_and_starts() {
        let r = parse(&out(), None, 0);
        assert!(r.ok && !r.truncated, "{r:?}");
        assert_eq!(r.now_s, Some(T0 + 5400));
        assert_eq!(r.runtimes.len(), 4);
        let h = &r.runtimes[0];
        assert!(h.head && h.running && h.exists);
        assert_eq!(h.started_s, Some(T0 + 3));
        assert_eq!(h.restart_count, Some(2));
        assert_eq!(h.demo, Some(true));
        assert_eq!(h.demo_start_s, Some(T0 + 5));
        assert_eq!(h.warning_s, Some(T0 + 3605));
        assert_eq!(h.stops_s, Some(T0 + 7205), "warning + 1 h");
        assert_eq!(h.seconds_left, Some(1805));
        assert_eq!(h.source.as_deref(), Some("shutdown warning"));
        assert_eq!(
            r.head().map(|h| h.container.as_str()),
            Some("codesys-master")
        );

        let s1 = &r.runtimes[1];
        assert!(!s1.head);
        assert_eq!(s1.stops_s, Some(T0 + 1800 + 2 + DEMO_S), "demo line + 2 h");
        assert_eq!(s1.source.as_deref(), Some("demo line"));
        assert_eq!(s1.warning, None);

        let s2 = &r.runtimes[2];
        assert!(s2.exists && !s2.running);
        assert_eq!(s2.state.as_deref(), Some("exited"));
        assert_eq!(
            (s2.stops_s, s2.seconds_left),
            (None, None),
            "stopped: no estimate"
        );
        assert_eq!(s2.warning_left_s, Some(600));

        let s3 = &r.runtimes[3];
        assert_eq!(s3.demo, None, "the log was not read");
        assert_eq!(s3.stops_s, Some(T0 + 3600 + DEMO_S));
        assert_eq!(s3.source.as_deref(), Some("container start (log not read)"));

        assert_eq!(
            r.first_stop().map(|s| s.container.as_str()),
            Some("codesys-master")
        );
        let t = text(&r).join("\n");
        assert!(t.contains("head codesys-master"), "{t}");
        assert!(
            t.contains("stops at 10:00 UTC (in 31 min; shutdown warning)"),
            "{t}"
        );
        assert!(t.contains("not running (exited)"), "{t}");
    }

    #[test]
    fn over_untimed_licensed_and_missing() {
        let out = [
            "now 1791800000",
            "container codesys-master",
            "state true running 2026-10-12T08:00:00Z 0",
            // No timestamps: log order decides (the warning after the last demo line).
            "log no runtime license - running in demo mode(~2 hours)",
            "log performing shutdown in 1 hour",
            "log no runtime license - running in demo mode(~2 hours)",
            "loglines 3",
            "container codesys-device",
            "state true running 2026-10-12T08:00:00Z 0",
            "loglines 50",
            "container codesys-device-gone",
            "state missing",
            "loglines 0",
        ]
        .join("\n");
        let cs = vec![
            ("codesys-master".to_string(), true),
            ("codesys-device".to_string(), false),
            ("codesys-device-gone".to_string(), false),
            ("codesys-device-r9".to_string(), false),
        ];
        let r = parse(&out, Some(&cs), 0);
        let h = &r.runtimes[0];
        assert_eq!(h.warning, None, "the warning was the previous run's");
        assert_eq!(h.source.as_deref(), Some("container start"));
        assert_eq!(h.stops_s, Some(T0 + DEMO_S));
        assert_eq!(h.seconds_left, Some(0), "past 10:00");
        assert!(h.expired());
        let d = &r.runtimes[1];
        assert_eq!(d.demo, Some(false));
        assert_eq!(d.stops_s, None, "licensed: no stop");
        assert!(!r.runtimes[2].exists && !r.runtimes[2].running);
        assert!(
            !r.runtimes[3].exists,
            "listed by the build, unknown on the host"
        );
        assert!(
            text(&r)
                .join("\n")
                .contains("demo period over since 10:00 UTC")
        );
    }

    #[test]
    fn a_cut_answer_is_not_read_as_complete() {
        // SSM cut the answer in the second container's log lines: its newest lines and its
        // `loglines` are gone, the third container is missing altogether.
        let out = [
            "now 1791797400",
            "container codesys-master",
            "state true running 2026-10-12T08:00:03Z 0",
            "log 2026-10-12T08:00:05Z, no runtime license - running in demo mode(~2 hours)",
            "loglines 400",
            "container codesys-device",
            "state true running 2026-10-12T08:30:00Z 0",
            "log 2026-10-12T04:30:02Z, no runtime license - running in demo mode(~2 hours)",
        ]
        .join("\n");
        let cs = vec![
            ("codesys-master".to_string(), true),
            ("codesys-device".to_string(), false),
            ("codesys-device-r2".to_string(), false),
        ];
        let r = parse(&out, Some(&cs), 0);
        assert!(r.ok && r.truncated, "{r:?}");
        assert_eq!(r.runtimes[0].source.as_deref(), Some("demo line"));
        let d = &r.runtimes[1];
        assert_eq!(d.demo, None, "cut: not read");
        assert_eq!(d.source.as_deref(), Some("container start (log not read)"));
        assert!(!r.runtimes[2].exists);
        assert!(text(&r).join("\n").contains("answer was cut"));
        let full = format!(
            "{out}\nloglines 9\ncontainer codesys-device-r2\nstate missing\nloglines 0\nend"
        );
        assert!(!parse(&full, Some(&cs), 0).truncated);
    }

    #[test]
    fn a_hook_heads_health_is_its_runtime() {
        let h = serde_json::json!({"runtime": {"schema": RUNTIME_SCHEMA, "ok": true, "nowS": 100, "runtimes": [
            {"container": "trust-runtime", "head": true, "exists": true, "running": true, "state": "running",
             "startedAt": "2026-11-09T10:00:00Z", "startedS": 90, "restartCount": 0, "demo": false,
             "stopsS": null, "secondsLeft": null, "source": "hook"}]}});
        let r = from_hook_health(&h).unwrap();
        assert!(r.ok && r.head().unwrap().running);
        assert_eq!(r.head().unwrap().demo, Some(false));
        assert!(r.first_stop().is_none());
        assert_eq!(from_hook_health(&serde_json::json!({"runtime": {"schema": "x"}})).unwrap_err().exit_code(), 4);
        assert!(from_hook_health(&serde_json::json!({})).is_err());
    }

    #[test]
    fn one_read_only_command_over_the_remote() {
        let mut m = MockRemote::default().answer("echo \"now", Ok(out()));
        let r = read(&mut m, None, 0).unwrap();
        assert_eq!(r.runtimes.len(), 4);
        assert_eq!(m.calls.len(), 1);
        let Call::Run(cmd) = &m.calls[0] else {
            panic!()
        };
        assert!(
            cmd.contains(
                "sudo docker ps -a --format '{{.Names}}' | grep -E '^codesys-(master|device)'"
            ),
            "{cmd}"
        );
        assert!(cmd.contains("sudo docker inspect -f 'state {{.State.Running}} {{.State.Status}} {{.State.StartedAt}} {{.RestartCount}}'"), "{cmd}");
        assert!(
            cmd.contains("sudo docker exec \"$c\" cat /var/opt/codesys/codesyscontrol.log"),
            "{cmd}"
        );
        // Only the demo / countdown lines, a few per container, and an end marker (SSM's limit).
        assert!(cmd.contains("l ~ /demo mode|shutdown in/"), "{cmd}");
        assert!(cmd.contains("| tail -n 13; done; echo end"), "{cmd}");
        assert!(
            cmd.contains("sudo docker cp \"$c\":/var/opt/codesys/codesyscontrol.log -"),
            "{cmd}"
        );
        for w in ["restart", "stop ", " rm ", "kill", "start ", ">"] {
            let shell = cmd.replace("2>/dev/null", "");
            assert!(!shell.contains(w), "read-only: {w:?} in {cmd}");
        }
        let cs = vec![
            ("codesys-master".to_string(), true),
            ("codesys-device-x y".to_string(), false),
        ];
        assert!(command(Some(&cs)).contains("for c in codesys-master 'codesys-device-x y'; do"));
        // An unreachable host is an error (exit 4), not an empty report.
        let mut m = MockRemote::default().answer(
            "echo",
            Err(crate::Error::Remote("ssm: ConnectionLost".into())),
        );
        assert_eq!(read(&mut m, None, 0).unwrap_err().exit_code(), 4);
        // No clock in the answer: not ok.
        let r = parse(
            "container codesys-master\nstate true running 2026-10-12T08:00:00Z 0\n",
            None,
            T0,
        );
        assert!(!r.ok && r.error.is_some());
        assert_eq!(
            r.runtimes[0].seconds_left,
            Some(DEMO_S),
            "the local clock instead"
        );
    }
}
