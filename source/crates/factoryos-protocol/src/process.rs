//! The process definition of a cell (`<cell>/plan2cell/process.yaml`): the IOs and process settings
//! its control code is generated with, editable by the user, the chat agent or Kiro in the IDE.
//!
//! * `telegram`: where the PLC ↔ robot handshake (the bt2plc / bt2krl contract, 256 bits each way)
//!   sits in each robot's `$IN` / `$OUT`; its byte layout is fixed by the contract.
//! * `processes`: per process (spotWeld, …) the KRL technology call and the operation type it
//!   applies to (profile vw-ks24), and the process parameters' defaults (a product's task overrides).
//! * `robots`: tool and base numbers, and the robot's own signals (gun, tip dresser, tool changer,
//!   clamps, …) outside the telegram, which hand-written or Kiro-written KRL uses.
//!
//! Saving merges the settings plan2cell has into the cell's `plan2cell/plan2cell.toml` (`[krl.sim]`
//! bases, `[krl.vw_ks24]` weld call, `[[krl.robot]]` tool / base numbers) and writes the robots'
//! signals as KRL declarations (`plan2cell/krl/<ns>_process_signals.dat`).

use std::collections::BTreeMap;

/// Telegram bits each way (a DIO256 module).
pub const TELEGRAM_BITS: u16 = 256;
/// KRL's digital IOs: `$IN[1..4096]`, `$OUT[1..4096]`.
pub const MAX_IO: u16 = 4096;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ProcessDefinition {
    /// `sim` or `vw-ks24` (as the export's profile).
    pub profile: String,
    pub description: String,
    pub telegram: Telegram,
    pub processes: BTreeMap<String, ProcessIo>,
    pub robots: Vec<RobotIo>,
}

impl Default for ProcessDefinition {
    fn default() -> Self {
        Self { profile: "sim".to_owned(), description: String::new(), telegram: Telegram::default(), processes: BTreeMap::new(), robots: Vec::new() }
    }
}

/// Where the handshake telegram sits: bit `8·byte + bit` is `$IN[in_base + 8·byte + bit + 1]`
/// (PLC → robot) and `$OUT[out_base + …]` (robot → PLC).
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Telegram {
    pub in_base: u16,
    pub out_base: u16,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ProcessIo {
    /// vw-ks24: the technology call during an operation's hold, e.g.
    /// `SEL_RES=SELECT(#MAKRO,580,TRUE,,,,,FALSE)`.
    pub weld_call: String,
    /// The `operation_type` the call applies to (e.g. `spot_weld`).
    pub op_type: String,
    /// Process parameter defaults (weldTime s, force kN, current kA, …).
    pub params: BTreeMap<String, serde_json::Value>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct RobotIo {
    /// The robot's namespace in the plan (`robot1`, …).
    pub ns: String,
    pub tool_no: Option<u16>,
    pub base_no: Option<u16>,
    pub signals: Vec<Signal>,
}

/// One digital signal of a robot, outside the telegram.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Signal {
    /// A KRL name: letters, digits, `_`, starting with a letter, at most 24 characters.
    pub name: String,
    /// `in` (`$IN`) or `out` (`$OUT`).
    pub dir: String,
    pub index: u16,
    /// Bits (1: a single bit; 2..16: `$IN[index] TO $IN[index + width - 1]`).
    pub width: Option<u16>,
    pub what: String,
}

/// One problem of a definition.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ProcessIssue {
    pub field: String,
    pub message: String,
}

/// `GET /api/v1/process` (and the answer of init / save).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ProcessView {
    pub cell: String,
    /// `process.yaml` on the backend host.
    pub path: String,
    /// Whether the cell has one (else `definition` is the profile's sample, not saved).
    pub exists: bool,
    pub definition: ProcessDefinition,
    /// The file as written (YAML, with its comments for a sample).
    pub yaml: String,
    pub issues: Vec<ProcessIssue>,
    /// After a save: what was written besides `process.yaml`.
    pub written: Vec<String>,
}

/// `POST /api/v1/process/init`: write the profile's sample (`sim` or `vw-ks24`).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct InitProcess {
    pub cell: Option<String>,
    pub profile: String,
    /// The plan's robots (default `robot1`, `robot2`).
    pub robots: Vec<String>,
    pub overwrite: bool,
}

/// `PUT /api/v1/process`: a definition (`definition`) or the YAML text (`yaml`).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SaveProcess {
    pub cell: Option<String>,
    pub definition: Option<ProcessDefinition>,
    pub yaml: Option<String>,
}

fn krl_name(name: &str) -> bool {
    let mut chars = name.chars();
    name.len() <= 24 && chars.next().is_some_and(|c| c.is_ascii_alphabetic()) && chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

impl ProcessDefinition {
    /// Every problem: profiles, ranges, names, signals overlapping the telegram or each other.
    pub fn validate(&self) -> Vec<ProcessIssue> {
        let mut issues = Vec::new();
        let mut issue = |field: String, message: String| issues.push(ProcessIssue { field, message });
        if !["sim", "vw-ks24"].contains(&self.profile.as_str()) {
            issue("profile".into(), format!("{:?}: sim or vw-ks24", self.profile));
        }
        for (field, base) in [("telegram.in_base", self.telegram.in_base), ("telegram.out_base", self.telegram.out_base)] {
            if base % 8 != 0 || base + TELEGRAM_BITS > MAX_IO {
                issue(field.into(), format!("{base}: a multiple of 8 with the 256 telegram bits within 1..{MAX_IO}"));
            }
        }
        for (name, p) in &self.processes {
            if !p.weld_call.is_empty() && p.op_type.trim().is_empty() {
                issue(format!("processes.{name}.op_type"), "a weld_call needs the op_type it applies to".into());
            }
        }
        let mut seen_ns = std::collections::BTreeSet::new();
        for (r, robot) in self.robots.iter().enumerate() {
            let at = |f: &str| format!("robots[{r}].{f}");
            if robot.ns.trim().is_empty() || !seen_ns.insert(robot.ns.as_str()) {
                issue(at("ns"), format!("{:?}: each robot once, by its namespace in the plan", robot.ns));
            }
            if robot.tool_no.is_some_and(|t| t == 0 || t > 16) {
                issue(at("tool_no"), "1..16".into());
            }
            if robot.base_no.is_some_and(|b| b > 32) {
                issue(at("base_no"), "0..32".into());
            }
            let mut names = std::collections::BTreeSet::new();
            let mut used: Vec<(String, u16, u16, String)> = Vec::new();
            for (s, signal) in robot.signals.iter().enumerate() {
                let at = |f: &str| format!("robots[{r}].signals[{s}].{f}");
                if !krl_name(&signal.name) {
                    issue(at("name"), format!("{:?}: a KRL name (letter first, letters/digits/_, ≤ 24)", signal.name));
                } else if !names.insert(signal.name.to_ascii_uppercase()) {
                    issue(at("name"), format!("{} is declared twice", signal.name));
                }
                let base = match signal.dir.as_str() {
                    "in" => self.telegram.in_base,
                    "out" => self.telegram.out_base,
                    other => {
                        issue(at("dir"), format!("{other:?}: in or out"));
                        continue;
                    }
                };
                let width = signal.width.unwrap_or(1);
                if width == 0 || width > 16 {
                    issue(at("width"), "1..16 bits".into());
                    continue;
                }
                let (lo, hi) = (signal.index, signal.index.saturating_add(width - 1));
                if lo == 0 || hi > MAX_IO {
                    issue(at("index"), format!("{lo}..{hi}: within 1..{MAX_IO}"));
                    continue;
                }
                let (t_lo, t_hi) = (base + 1, base + TELEGRAM_BITS);
                if lo <= t_hi && hi >= t_lo {
                    issue(at("index"), format!("${}[{lo}..{hi}] overlaps the telegram ${}[{t_lo}..{t_hi}]", io(&signal.dir), io(&signal.dir)));
                }
                for (other, o_lo, o_hi, dir) in &used {
                    if *dir == signal.dir && lo <= *o_hi && hi >= *o_lo {
                        issue(at("index"), format!("{} overlaps {other} (${}[{o_lo}..{o_hi}])", signal.name, io(dir)));
                    }
                }
                used.push((signal.name.clone(), lo, hi, signal.dir.clone()));
            }
        }
        issues
    }

    /// The robots' signals as KRL declarations (`DEFDAT`-free lines for a `$config.dat` or a
    /// `.dat` of their own), by robot namespace.
    pub fn krl_signals(&self) -> BTreeMap<String, String> {
        self.robots
            .iter()
            .filter(|r| !r.signals.is_empty())
            .map(|robot| {
                let mut text = format!("; Process signals of {} (process.yaml): generated, edit process.yaml.\n", robot.ns);
                for s in &robot.signals {
                    let reg = io(&s.dir);
                    let width = s.width.unwrap_or(1);
                    let target = if width > 1 { format!("${reg}[{}] TO ${reg}[{}]", s.index, s.index + width - 1) } else { format!("${reg}[{}]", s.index) };
                    let what = if s.what.is_empty() { String::new() } else { format!(" ; {}", s.what) };
                    text.push_str(&format!("SIGNAL {} {target}{what}\n", s.name));
                }
                (robot.ns.clone(), text)
            })
            .collect()
    }
}

fn io(dir: &str) -> &'static str {
    if dir == "in" { "IN" } else { "OUT" }
}

/// The sample definition of `profile`, as commented YAML, for `robots`.
pub fn sample_yaml(profile: &str, robots: &[String]) -> String {
    let vw = profile == "vw-ks24";
    let robots: Vec<String> = if robots.is_empty() { vec!["robot1".into(), "robot2".into()] } else { robots.to_vec() };
    let mut y = format!(
        "# Process definition of this cell: the IOs and process settings its control code is generated with.\n\
         # Saved with the process_save tool (or Save in the IDE, then process_save with the file) it is merged into\n\
         # plan2cell/plan2cell.toml and the robots' signals become plan2cell/krl/<ns>_process_signals.dat.\n\
         profile: {profile}\n\
         description: Spot welding cell\n\
         # The PLC <-> robot handshake (256 bits each way, layout fixed by the bt2plc/bt2krl contract):\n\
         # bit 8*byte+bit is $IN[in_base+8*byte+bit+1] (PLC -> robot) and $OUT[out_base+...] (robot -> PLC).\n\
         telegram:\n  in_base: 0\n  out_base: 0\n\
         processes:\n  spotWeld:\n"
    );
    if vw {
        y.push_str("    # The technology call during each weld's hold (VW KS24 SPS macro).\n    weld_call: SEL_RES=SELECT(#MAKRO,580,TRUE,,,,,FALSE)\n    op_type: spot_weld\n");
    } else {
        y.push_str("    weld_call: ''\n    op_type: spot_weld\n");
    }
    y.push_str("    # Defaults; a product's task (Product & Task) overrides them per task and weld.\n    params: {weldTime: 0.3, force: 3.0, current: 9.0, approachOffset: 0.05}\nrobots:\n");
    for (i, ns) in robots.iter().enumerate() {
        let base = 300 + 20 * i as u16;
        y.push_str(&format!(
            "  - ns: {ns}\n    tool_no: 1\n    base_no: 0\n    # The robot's own IOs (outside the telegram's $IN/$OUT[1..256]) for the gun and its periphery.\n    signals:\n\
             \x20     - {{name: GUN_CLOSE, dir: out, index: {}, what: close the gun}}\n\
             \x20     - {{name: GUN_OPEN, dir: out, index: {}, what: open the gun}}\n\
             \x20     - {{name: GUN_CLOSED, dir: in, index: {}, what: gun closed (feedback)}}\n\
             \x20     - {{name: GUN_OPENED, dir: in, index: {}, what: gun open (feedback)}}\n\
             \x20     - {{name: WELD_DONE, dir: in, index: {}, what: weld controller done}}\n\
             \x20     - {{name: WELD_PROG, dir: out, index: {}, width: 8, what: weld program number}}\n\
             \x20     - {{name: TIP_DRESS_REQ, dir: out, index: {}, what: tip dresser request}}\n",
            base + 1,
            base + 2,
            base + 1,
            base + 2,
            base + 3,
            base + 10,
            base + 3
        ));
    }
    y
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn samples_parse_and_validate() {
        for profile in ["sim", "vw-ks24"] {
            let text = sample_yaml(profile, &[]);
            let def: ProcessDefinition = serde_yaml::from_str(&text).unwrap_or_else(|e| panic!("{profile}: {e}\n{text}"));
            assert_eq!(def.robots.len(), 2);
            assert_eq!(def.validate(), [], "{profile}");
            let dat = def.krl_signals();
            assert!(dat["robot1"].contains("SIGNAL WELD_PROG $OUT[310] TO $OUT[317]"), "{}", dat["robot1"]);
        }
    }

    #[test]
    fn overlaps_and_names_are_issues() {
        let mut def: ProcessDefinition = serde_yaml::from_str(&sample_yaml("sim", &["r".into()])).unwrap();
        def.robots[0].signals.push(Signal { name: "IN_TELEGRAM".into(), dir: "in".into(), index: 10, ..Signal::default() });
        def.robots[0].signals.push(Signal { name: "GUN_CLOSE".into(), dir: "out".into(), index: 500, ..Signal::default() });
        def.robots[0].signals.push(Signal { name: "1bad".into(), dir: "out".into(), index: 600, ..Signal::default() });
        def.robots[0].signals.push(Signal { name: "CLASH".into(), dir: "out".into(), index: 312, ..Signal::default() });
        let messages: Vec<String> = def.validate().into_iter().map(|i| i.message).collect();
        assert!(messages.iter().any(|m| m.contains("overlaps the telegram")), "{messages:?}");
        assert!(messages.iter().any(|m| m.contains("declared twice")), "{messages:?}");
        assert!(messages.iter().any(|m| m.contains("a KRL name")), "{messages:?}");
        assert!(messages.iter().any(|m| m.contains("CLASH overlaps WELD_PROG") || m.contains("overlaps WELD_PROG")), "{messages:?}");
        def.telegram.in_base = 3;
        assert!(def.validate().iter().any(|i| i.field == "telegram.in_base"));
    }
}
