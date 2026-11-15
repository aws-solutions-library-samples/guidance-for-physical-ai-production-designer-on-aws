//! plan2cell: one behaviour-tree plan -> the head-PLC code and the robot programs of a cell, built
//! together so they agree.
//!
//! The PLC side is [`bt2plc`], the robot side [`bt2krl`]. Run separately, the two must be kept in
//! step by hand: the same protocol profile, the same PLAN_ID byte, the same zone -> interlock
//! map, and robot programs from the very bt2plc run whose PLAN_ID the head carries. [`build`]
//! does that in one pass and refuses combinations that cannot work.
//!
//! ```text
//! plan (JSON / BT.CPP XML) --bt2plc--> cell plan, head PLC, robot simulators, program.json
//!                                         '--bt2krl--> KUKA KRL per robot
//! ```
//!
//! Output layout of a build directory:
//!
//! ```text
//! <out>/plc/...        everything bt2plc writes (cell_plan.json, robots/<ns>/program.json, codesys/, trust/, ...)
//! <out>/krl/<ns>/...   everything bt2krl writes per robot
//! <out>/BUILD.md       what was built with which options
//! <out>/plan2cell-build.json   the same, machine-readable
//! ```

pub mod config;
pub mod deploy_check;
pub mod hardware;
pub mod report;
#[cfg(feature = "sim")]
pub mod sim;
pub mod targets;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

pub use config::{BuildConfig, BuildDefaults};

/// Robot-side protocol of the whole cell. The head PLC and the robot programs always use the same.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, clap::ValueEnum)]
#[serde(rename_all = "kebab-case")]
pub enum Profile {
    /// The bt2plc device-simulator telegram (docker simulation, OfficeLite, test robots).
    #[default]
    Sim,
    /// Real VW KS24 robots (KSS 8.2, TPVW 8.1.8): VW Folge handshake and robot interlocks.
    VwKs24,
}

impl Profile {
    pub fn name(self) -> &'static str {
        match self {
            Profile::Sim => "sim",
            Profile::VwKs24 => "vw-ks24",
        }
    }

    fn plc(self) -> bt2plc::config::Profile {
        match self {
            Profile::Sim => bt2plc::config::Profile::Sim,
            Profile::VwKs24 => bt2plc::config::Profile::VwKs24,
        }
    }

    fn krl(self) -> bt2krl::Profile {
        match self {
            Profile::Sim => bt2krl::Profile::Sim,
            Profile::VwKs24 => bt2krl::Profile::VwKs24,
        }
    }
}

/// Which head-PLC code to write.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, clap::ValueEnum)]
#[serde(rename_all = "kebab-case")]
pub enum PlcTarget {
    /// CODESYS V3.5 projects (ST objects, project.json, MCP build prompt, trust renderings).
    #[default]
    Codesys,
    /// No PLC code: only the flavour-independent plan outputs (`cell_plan.json`, `ZONES.md`,
    /// `robots/<ns>/program.json`) that the robot programs are generated from.
    None,
}

/// What to build. Every field has a CLI flag and a `[build]` default in plan2cell.toml.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BuildOptions {
    pub profile: Profile,
    pub plc: PlcTarget,
    /// The per-robot simulator projects (CODESYS device ST) next to the head PLC.
    pub robot_sim: bool,
    /// The KUKA KRL robot programs.
    pub krl: bool,
    /// Only these robots get KRL (`None` = all).
    pub krl_robots: Option<Vec<String>>,
    /// Type-check the generated ST with trust (feature `check`).
    pub st_check: bool,
    /// Warnings are errors.
    pub strict: bool,
}

impl Default for BuildOptions {
    fn default() -> Self {
        Self {
            profile: Profile::Sim,
            plc: PlcTarget::Codesys,
            robot_sim: true,
            krl: true,
            krl_robots: None,
            st_check: cfg!(feature = "check"),
            strict: false,
        }
    }
}

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("{0}")]
    Plc(#[from] bt2plc::Error),
    #[error("{0}")]
    Krl(#[from] bt2krl::Error),
    #[error("config: {0}")]
    Config(String),
    #[error("usage: {0}")]
    Usage(String),
    #[error("{}: {source}", path.display())]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("simulation check failed: {0}")]
    Sim(String),
    /// A planned target (PLC dialect, robot language) that is not generated yet.
    #[error("{0}")]
    NotImplemented(String),
}

pub type Result<T> = std::result::Result<T, Error>;

impl Error {
    /// The bt2plc / bt2krl exit-code numbering (0 ok, 1 input, 2 usage, 3 config, 4 I/O,
    /// 5 not implemented, 6 strict, 7 verification / simulation, 8 generated code rejected).
    pub fn exit_code(&self) -> i32 {
        match self {
            Error::Plc(e) => e.exit_code(),
            Error::Krl(e) => e.exit_code(),
            Error::Config(_) => 3,
            Error::Usage(_) => 2,
            Error::Io { .. } => 4,
            Error::Sim(_) => 7,
            Error::NotImplemented(_) => 5,
        }
    }

    /// `--strict` turned warnings into this error.
    pub fn is_strict(&self) -> bool {
        matches!(
            self,
            Error::Plc(bt2plc::Error::Strict(_)) | Error::Krl(bt2krl::Error::Strict(_))
        )
    }

    pub fn io(path: &Path, source: std::io::Error) -> Self {
        Error::Io {
            path: path.to_path_buf(),
            source,
        }
    }
}

/// A plan as text plus where it came from (the file name appears in the generated headers).
#[derive(Clone, Debug)]
pub struct PlanInput {
    pub text: String,
    pub file_name: String,
    /// Which tree to lower from a JSON plan: its `root` mirror or the embedded XML.
    pub source: bt2plc::input::SourceSel,
    /// Blackboard for a plain BT.CPP XML tree.
    pub blackboard: Option<BTreeMap<String, serde_json::Value>>,
}

impl PlanInput {
    pub fn from_text(text: impl Into<String>, file_name: impl Into<String>) -> Self {
        Self {
            text: text.into(),
            file_name: file_name.into(),
            source: bt2plc::input::SourceSel::Auto,
            blackboard: None,
        }
    }

    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).map_err(|e| Error::io(path, e))?;
        let name = path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();
        Ok(Self::from_text(text, name))
    }

    pub fn load_blackboard(&mut self, path: &Path) -> Result<()> {
        let t = std::fs::read_to_string(path).map_err(|e| Error::io(path, e))?;
        self.blackboard = Some(
            serde_json::from_str(&t)
                .map_err(|e| Error::Config(format!("{}: {e}", path.display())))?,
        );
        Ok(())
    }

    fn doc(&self, d: &mut bt2plc::Diagnostics) -> Result<bt2plc::input::InputDoc> {
        Ok(bt2plc::input::load_str(
            &self.text,
            &self.file_name,
            self.source,
            self.blackboard.clone(),
            d,
        )?)
    }
}

/// The effective configuration of a build: the profile and the shared settings applied to both
/// sides, plus notes on what was adjusted.
#[derive(Clone, Debug)]
pub struct Effective {
    pub plc: bt2plc::Config,
    pub krl: bt2krl::Config,
    pub notes: Vec<String>,
}

/// Apply the build options to the configuration, so that head PLC and robots agree.
pub fn effective(cfg: &BuildConfig, opts: &BuildOptions) -> Result<Effective> {
    let eff = effective_unchecked(cfg, opts)?;
    eff.plc.validate()?;
    eff.krl.validate()?;
    Ok(eff)
}

/// [`effective`] without the final validation of the two tool configs (for reports that list
/// what is wrong instead of failing at the first problem, like [`hardware::resolve`]).
pub fn effective_unchecked(cfg: &BuildConfig, opts: &BuildOptions) -> Result<Effective> {
    let mut plc = cfg.plc.clone();
    let mut krl = cfg.krl.clone();
    let mut notes = Vec::new();
    plc.protocol.profile = opts.profile.plc();
    if !opts.robot_sim || opts.plc == PlcTarget::None {
        plc.robot_sim.enabled = false;
    }
    if opts.profile == Profile::VwKs24 {
        if plc.cell.job_number != bt2plc::config::JobNumber::Byte {
            plc.cell.job_number = bt2plc::config::JobNumber::Byte;
            notes.push("vw-ks24: job_number set to \"byte\" (PRG_NR is 8 bits)".into());
        }
        if plc.protocol.max_zones > 16 {
            plc.protocol.max_zones = 16;
            notes.push("vw-ks24: max_zones set to 16 (the VW robot interlocks)".into());
        }
        let default_plan_byte = bt2plc::config::ProtocolCfg::default().plan_id_byte;
        if plc.protocol.plan_id_byte == default_plan_byte {
            // The sim default (bytes 4-5) carries the partner interlock releases E41-48 on VW
            // robots. Never write PLAN_ID there; without an explicit free byte there is no check.
            plc.protocol.plan_id_byte = 0;
            notes.push(
                "vw-ks24: no PLAN_ID check (set plc.protocol.plan_id_byte to a free byte, \
                 e.g. 12, to get one)"
                    .into(),
            );
        }
        if plc.robot_sim.enabled {
            plc.robot_sim.enabled = false;
            notes.push(
                "vw-ks24: no robot simulator projects (the device simulator speaks the sim \
                 telegram); the KRL executor stands in for the robots in `plan2cell simulate`"
                    .into(),
            );
        }
        // One source of truth for the PLAN_ID byte.
        if krl.vw_ks24.plan_id_byte != 0 && krl.vw_ks24.plan_id_byte != plc.protocol.plan_id_byte {
            return Err(Error::Config(format!(
                "krl.vw_ks24.plan_id_byte = {} but plc.protocol.plan_id_byte = {}: set it in \
                 [plc.protocol] only",
                krl.vw_ks24.plan_id_byte, plc.protocol.plan_id_byte
            )));
        }
        krl.vw_ks24.plan_id_byte = plc.protocol.plan_id_byte;
    }
    // The robot model is only needed by bt2krl, but it is shared: give bt2krl the one from
    // [[plc.robot]] when its own [[krl.robot]] does not name one.
    for pr in &plc.robots {
        if let Some(m) = &pr.model
            && let Some(kr) = krl.robots.iter_mut().find(|r| r.ns == pr.ns)
            && kr.model.is_none()
        {
            kr.model = Some(m.clone());
        }
    }
    Ok(Effective { plc, krl, notes })
}

/// The result of a build, all in memory.
pub struct Build {
    pub options: BuildOptions,
    pub plan: bt2plc::ir::CellPlan,
    pub plc: bt2plc::Generated,
    pub krl: Option<bt2krl::Generated>,
    /// Every file, relative to the build directory (`plc/...`, `krl/...`, `BUILD.md`, ...).
    pub files: BTreeMap<String, String>,
    pub diagnostics: Vec<String>,
    /// The same diagnostics, structured (severity, code, location).
    pub diags: Vec<report::Diagnostic>,
    pub notes: Vec<String>,
    pub summary: Summary,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Summary {
    pub schema: String,
    pub generator: String,
    pub tree_id: String,
    pub cell_id: String,
    pub plan_id: String,
    pub options: BuildOptions,
    pub robots: Vec<RobotSummary>,
    pub zones: Vec<ZoneSummary>,
    pub makespan_s: f64,
    pub st_checked: bool,
    pub files: usize,
    pub notes: Vec<String>,
    pub warnings: Vec<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RobotSummary {
    pub ns: String,
    pub tag: String,
    pub ops: usize,
    /// KRL Folgen written for it (empty when no KRL was built).
    pub folgen: Vec<u16>,
    pub krl_points: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ZoneSummary {
    pub name: String,
    pub tickets: Vec<String>,
}

const BUILD_SCHEMA: &str = "plan2cell/build@1";
pub const GENERATOR: &str = concat!("plan2cell ", env!("CARGO_PKG_VERSION"));

/// Plan only: zones, ticket order and schedule (no code).
pub fn plan(
    input: &PlanInput,
    cfg: &BuildConfig,
    opts: &BuildOptions,
) -> Result<(bt2plc::ir::CellPlan, Vec<String>)> {
    let eff = effective(cfg, opts)?;
    let mut d = bt2plc::Diagnostics::default();
    let doc = input.doc(&mut d)?;
    let p = bt2plc::plan(&doc, &eff.plc, &mut d)?;
    Ok((p, d.items().iter().map(|x| x.to_string()).collect()))
}

/// Build everything the options ask for, in memory.
pub fn build(input: &PlanInput, cfg: &BuildConfig, opts: &BuildOptions) -> Result<Build> {
    let eff = effective(cfg, opts)?;
    let mut d0 = bt2plc::Diagnostics::default();
    let doc = input.doc(&mut d0)?;
    let plc = bt2plc::generate(
        &doc,
        &eff.plc,
        &bt2plc::GenerateOptions {
            flavour: bt2plc::backend::Flavour::Codesys,
            strict: opts.strict,
            st_check: opts.st_check && opts.plc == PlcTarget::Codesys && cfg!(feature = "check"),
        },
    )?;
    let mut diagnostics: Vec<String> = d0
        .items()
        .iter()
        .chain(plc.diagnostics.items())
        .map(|x| x.to_string())
        .collect();
    let mut diags: Vec<report::Diagnostic> = d0
        .items()
        .iter()
        .chain(plc.diagnostics.items())
        .map(report::Diagnostic::from_plc)
        .collect();
    let mut files = BTreeMap::new();
    for (rel, text) in plc.output.files() {
        let keep = match opts.plc {
            PlcTarget::Codesys => true,
            PlcTarget::None => {
                rel == "cell_plan.json" || rel == "ZONES.md" || rel.starts_with("robots/")
            }
        };
        if keep {
            files.insert(format!("plc/{rel}"), text.clone());
        }
    }
    let krl = if opts.krl {
        let programs: Vec<bt2krl::program::Program> = plc
            .plan
            .robots
            .iter()
            .filter(|r| !r.idle)
            .map(|r| {
                let f = format!("robots/{}/program.json", r.ns);
                let text = plc
                    .output
                    .get(&f)
                    .ok_or_else(|| Error::Config(format!("bt2plc wrote no {f}")))?;
                Ok(bt2krl::program::Program::from_str(text, &f)?)
            })
            .collect::<Result<_>>()?;
        let g = bt2krl::generate(
            &programs,
            &eff.krl,
            &bt2krl::Options {
                profile: opts.profile.krl(),
                motion: None,
                robots: opts.krl_robots.clone(),
                strict: opts.strict,
            },
        )?;
        for (rel, text) in g.output.files() {
            files.insert(format!("krl/{rel}"), text.clone());
        }
        diagnostics.extend(g.warnings.iter().map(|w| format!("warning[krl] {w}")));
        diags.extend(
            g.warnings
                .iter()
                .map(|w| report::Diagnostic::warning("krl", w.clone())),
        );
        Some(g)
    } else {
        None
    };
    let p = &plc.plan;
    let summary = Summary {
        schema: BUILD_SCHEMA.into(),
        generator: GENERATOR.into(),
        tree_id: p.tree_id.clone(),
        cell_id: p.cell_id.clone(),
        plan_id: format!("16#{:04X}", p.plan_id),
        options: opts.clone(),
        robots: p
            .robots
            .iter()
            .map(|r| {
                let k = krl
                    .as_ref()
                    .and_then(|g| g.robots.iter().find(|x| x.ns == r.ns));
                RobotSummary {
                    ns: r.ns.clone(),
                    tag: r.addr.tag.clone(),
                    ops: r.ops.len(),
                    folgen: k
                        .map(|x| {
                            std::iter::once(x.lowered.prod.number)
                                .chain(x.lowered.home.as_ref().map(|h| h.number))
                                .collect()
                        })
                        .unwrap_or_default(),
                    krl_points: k.map(|x| x.lowered.prod.points.len()).unwrap_or(0),
                }
            })
            .collect(),
        zones: p
            .zones
            .iter()
            .map(|z| ZoneSummary {
                name: z.name.clone(),
                tickets: z.tickets.iter().map(|t| p.op_ref(t.op)).collect(),
            })
            .collect(),
        makespan_s: p.makespan_ms as f64 / 1000.0,
        st_checked: plc.st_checked,
        files: files.len() + 2,
        notes: eff.notes.clone(),
        warnings: diagnostics.clone(),
    };
    files.insert("BUILD.md".into(), build_md(&summary));
    files.insert(
        "plan2cell-build.json".into(),
        serde_json::to_string_pretty(&summary).expect("summary serialises") + "\n",
    );
    // The deployability rules that need no cell (deploy_check): their findings as diagnostics.
    if opts.plc == PlcTarget::Codesys {
        diags.extend(deploy_check::check_build(&files).diagnostics());
    }
    Ok(Build {
        options: opts.clone(),
        plan: plc.plan.clone(),
        plc,
        krl,
        files,
        diagnostics,
        diags,
        notes: eff.notes,
        summary,
    })
}

/// Declare `e_axes = "none"` for every robot of the plan whose `[[krl.robot]]` does not declare
/// its external axes (an explicit opt-in: bt2krl itself never guesses). Returns a note per robot.
pub fn declare_missing_e_axes(
    input: &PlanInput,
    cfg: &mut BuildConfig,
    opts: &BuildOptions,
) -> Result<Vec<String>> {
    let (p, _) = plan(input, cfg, opts)?;
    let mut notes = Vec::new();
    for r in p.robots.iter().filter(|r| !r.idle) {
        match cfg.krl.robots.iter_mut().find(|k| k.ns == r.ns) {
            Some(k) if k.e_axes.is_some() => continue,
            Some(k) => k.e_axes = Some(bt2krl::config::EAxes::None),
            None => cfg.krl.robots.push(bt2krl::config::RobotCfg {
                ns: r.ns.clone(),
                e_axes: Some(bt2krl::config::EAxes::None),
                ..Default::default()
            }),
        }
        notes.push(format!(
            "{}: e_axes = \"none\" assumed (no [[krl.robot]] e_axes; declare it in plan2cell.toml)",
            r.ns
        ));
    }
    Ok(notes)
}

impl Build {
    /// Write the build: atomic per file, a non-empty directory needs `force`, files of the
    /// previous build that are no longer produced are removed.
    pub fn write_to(&self, dir: &Path, force: bool) -> Result<()> {
        let mut out = bt2krl::output::Output::default();
        for (rel, text) in &self.files {
            out.add(rel.clone(), text.clone())?;
        }
        out.write_to(dir, force)?;
        Ok(())
    }
}

fn build_md(s: &Summary) -> String {
    let o = &s.options;
    let mut m = String::new();
    m.push_str(&format!(
        "# Build of {} (cell {})\n\n",
        s.tree_id, s.cell_id
    ));
    m.push_str(&format!(
        "Generated by {}. **PLAN_ID {}** — the head PLC and every robot program below come from \
         this one build; never mix files of different builds.\n\n",
        s.generator, s.plan_id
    ));
    m.push_str("| Option | Value |\n|---|---|\n");
    m.push_str(&format!("| profile | `{}` |\n", o.profile.name()));
    m.push_str(&format!(
        "| PLC code | {} |\n",
        match o.plc {
            PlcTarget::Codesys =>
                "CODESYS (`plc/codesys/`, `plc/project.json`, `plc/MCP_PROMPT.md`)",
            PlcTarget::None => "none (plan outputs only)",
        }
    ));
    m.push_str(&format!(
        "| robot simulators | {} |\n",
        if o.robot_sim && o.plc == PlcTarget::Codesys && o.profile == Profile::Sim {
            "yes (`plc/codesys/robots/`)"
        } else {
            "no"
        }
    ));
    m.push_str(&format!(
        "| KUKA KRL | {} |\n",
        if o.krl {
            "yes (`krl/<ns>/`, see each `INSTALL.md`)"
        } else {
            "no"
        }
    ));
    m.push_str(&format!(
        "| ST type check | {} |\n\n",
        if s.st_checked { "trust OK" } else { "not run" }
    ));
    m.push_str(&format!(
        "{} robot(s), {} zone(s), planned makespan {:.3} s.\n\n",
        s.robots.len(),
        s.zones.len(),
        s.makespan_s
    ));
    if !s.zones.is_empty() {
        m.push_str("| Zone | Ticket order |\n|---|---|\n");
        for z in &s.zones {
            m.push_str(&format!("| {} | {} |\n", z.name, z.tickets.join(" > ")));
        }
        m.push('\n');
    }
    m.push_str("| Robot | Ops | KRL Folgen | KRL points |\n|---|---|---|---|\n");
    for r in &s.robots {
        let f: Vec<String> = r.folgen.iter().map(|n| n.to_string()).collect();
        m.push_str(&format!(
            "| {} ({}) | {} | {} | {} |\n",
            r.ns,
            r.tag,
            r.ops,
            if f.is_empty() {
                "-".into()
            } else {
                f.join(", ")
            },
            r.krl_points
        ));
    }
    if !s.notes.is_empty() {
        m.push_str("\n## Adjusted for the profile\n\n");
        for n in &s.notes {
            m.push_str(&format!("- {n}\n"));
        }
    }
    if !s.warnings.is_empty() {
        m.push_str("\n## Warnings\n\n");
        for w in &s.warnings {
            m.push_str(&format!("- {w}\n"));
        }
    }
    m
}

/// One robot's KRL plan without writing anything (bt2krl `explain`), as text.
pub fn explain_robot(build: &Build, ns: &str) -> Result<String> {
    let g = build
        .krl
        .as_ref()
        .ok_or_else(|| Error::Usage("this build has no KRL (krl = false)".into()))?;
    let r = g
        .robots
        .iter()
        .find(|r| r.ns == ns)
        .ok_or_else(|| Error::Usage(format!("no robot {ns} in this build")))?;
    let mut s = format!(
        "robot {ns}: Folge {} ({} ops, {} points), model {}, profile {}\n",
        r.lowered.prod.number,
        r.lowered.prod.ops.len(),
        r.lowered.prod.points.len(),
        r.setup.model.name,
        build.options.profile.name()
    );
    for e in &r.lowered.estimates {
        s.push_str(&format!(
            "  op {:>3} {:<12} mask {} est {:.3} s (motion {:.3} / planned {:.3}), expected {:.3} s, limit {:.3} s\n",
            e.index,
            e.operation_id,
            e.zone_mask,
            e.est_s,
            e.est_motion_s,
            e.planned_motion_s,
            e.expected_s,
            e.limit_s
        ));
    }
    for w in &r.lowered.warnings {
        s.push_str(&format!("  warning: {w}\n"));
    }
    Ok(s)
}
