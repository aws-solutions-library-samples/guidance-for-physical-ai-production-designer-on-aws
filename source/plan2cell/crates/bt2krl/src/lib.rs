//! bt2krl: bt2plc robot programs (`robots/<ns>/program.json`) -> KUKA KRL (KSS 8.x).
//!
//! `program` (input) -> `lower` (IR per Folge, limit and timing checks) -> `render` (KRL text
//! per profile) -> `validate` (strict text checks) + `parse` (IR round trip) -> `output`.
//! See `TASK.md` for the specification.

pub mod config;
pub mod error;
pub mod exec;
pub mod io;
pub mod ir;
pub mod krl;
pub mod lower;
pub mod model;
pub mod motion;
pub mod names;
pub mod output;
pub mod parse;
pub mod program;
pub mod render;
pub mod render_vw;
pub mod report;
pub mod validate;
pub mod vw;

use std::collections::BTreeMap;
use std::path::Path;

pub use config::{Config, Profile};
pub use error::{Error, Result};

use config::MotionMode;
use io::SimIo;
use lower::{Lowered, RobotSetup};
use output::Output;
use program::Program;
use render::Ctx;
use validate::{CheckCtx, OpExpect};

#[derive(Clone, Debug)]
pub struct Options {
    pub profile: Profile,
    /// Overrides `[motion] mode`.
    pub motion: Option<MotionMode>,
    /// Only these robots (ns); `None` = all.
    pub robots: Option<Vec<String>>,
    /// Warnings become an error (exit 6).
    pub strict: bool,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            profile: Profile::Sim,
            motion: None,
            robots: None,
            strict: false,
        }
    }
}

/// One robot's generated program.
#[derive(Debug)]
pub struct RobotOut {
    pub ns: String,
    pub setup: RobotSetup,
    pub lowered: Lowered,
    pub io: SimIo,
    /// Paths relative to the robot directory `<ns>/`.
    pub files: BTreeMap<String, String>,
}

#[derive(Debug)]
pub struct Generated {
    pub robots: Vec<RobotOut>,
    /// All files relative to the output directory.
    pub output: Output,
    pub warnings: Vec<String>,
}

pub fn check_ctx(ctx: &Ctx, l: &Lowered, prod_path: &str) -> CheckCtx {
    CheckCtx {
        profile: ctx.profile.name().into(),
        line_ending: ctx.cfg.output.line_ending,
        in_range: match ctx.profile {
            Profile::VwKs24 => (1, 4096),
            Profile::Sim => ctx.io.in_range(),
        },
        out_range: match ctx.profile {
            Profile::VwKs24 => (1, 256),
            Profile::Sim => ctx.io.out_range(),
        },
        reserved_out: match ctx.profile {
            Profile::VwKs24 => crate::vw::RESERVED_OUT.to_vec(),
            Profile::Sim => Vec::new(),
        },
        reserved_in: match ctx.profile {
            Profile::VwKs24 => crate::vw::RESERVED_IN.to_vec(),
            Profile::Sim => Vec::new(),
        },
        request_base: match ctx.profile {
            Profile::VwKs24 => crate::vw::request_out(1),
            Profile::Sim => ctx.io.request,
        },
        grant_base: match ctx.profile {
            Profile::VwKs24 => crate::vw::grant_in(1),
            Profile::Sim => ctx.io.grant,
        },
        production: prod_path.into(),
        step_prefix: match ctx.profile {
            Profile::VwKs24 => format!("{}=", crate::vw::step_signal(&ctx.cfg.vw_ks24).1),
            Profile::Sim => "BT_STEP=".into(),
        },
        ops: l
            .prod
            .ops
            .iter()
            .map(|o| OpExpect {
                index: o.index,
                mask: o.mask,
                hold_ms: o
                    .instrs
                    .iter()
                    .find_map(|i| match i {
                        ir::Instr::Hold(h) => Some(*h),
                        _ => None,
                    })
                    .unwrap_or(0),
            })
            .collect(),
    }
}

/// Generate, validate and round-trip one robot program.
pub fn generate_program(p: &Program, cfg: &Config, opts: &Options) -> Result<RobotOut> {
    let mode = opts.motion.unwrap_or(cfg.motion.mode);
    if mode == MotionMode::Spline {
        return Err(Error::Usage(
            "motion mode spline is not available: that PTP_SPLINE with TIME_BLOCK runs on the \
             target KSS V8.2.29 is unverified (the controller exports list no spline technology \
             package and no sample robot uses SPLINE, SPTP or TIME_BLOCK), so bt2krl only emits \
             approximated joint PTP. Use ptp."
                .into(),
        ));
    }
    let setup = RobotSetup::resolve(p, cfg, true)?;
    let lowered = lower::lower(p, &setup, cfg, opts.profile)?;
    let io = SimIo::new(p, &cfg.sim);
    let ctx = Ctx {
        p,
        s: &setup,
        cfg,
        profile: opts.profile,
        io: io.clone(),
        il: lowered.interlocks.clone(),
    };
    let folgen = "KRC/R1/Folgen";
    let prod_stem = format!("{folgen}/{}", ctx.file_stem(lowered.prod.number));
    let mut files: Vec<(String, String)> = vec![
        (
            format!("{prod_stem}.src"),
            render::folge_src(&ctx, &lowered.prod),
        ),
        (
            format!("{prod_stem}.dat"),
            render::folge_dat(&ctx, &lowered.prod),
        ),
    ];
    let mut round_trip = vec![(lowered.prod.clone(), prod_stem.clone())];
    if let Some(home) = &lowered.home {
        let home_stem = format!("{folgen}/{}", ctx.file_stem(home.number));
        files.push((format!("{home_stem}.src"), render::folge_src(&ctx, home)));
        files.push((format!("{home_stem}.dat"), render::folge_dat(&ctx, home)));
        round_trip.push((home.clone(), home_stem));
    }
    match opts.profile {
        Profile::Sim => {
            let home = lowered.home.as_ref().expect("sim has a HOME Folge");
            files.push((
                "KRC/R1/Program/bt_cell.src".into(),
                render::bt_cell_src(&ctx, &lowered.prod, home),
            ));
            files.push((
                "KRC/R1/Program/bt_cell.dat".into(),
                render::bt_cell_dat(&ctx),
            ));
            files.push(("KRC/R1/System/bt_sps.src".into(), render::bt_sps_src(&ctx)));
            files.push((
                "KRC/R1/System/bt_config.dat".into(),
                render::bt_config_dat(&ctx, home),
            ));
        }
        Profile::VwKs24 => {
            files.push((
                "KRC/R1/System/bt_config.dat".into(),
                render_vw::config_dat(&ctx),
            ));
            if cfg.vw_ks24.emit_patches {
                files.push((
                    "patches/KRC/R1/Folgen/cell.src".into(),
                    render_vw::cell_src(&ctx, &lowered),
                ));
                files.push((
                    "patches/KRC/R1/System/$config.dat".into(),
                    render_vw::config_dat_full(&ctx),
                ));
            }
        }
    }
    files.sort();
    let map: BTreeMap<String, String> = files.iter().cloned().collect();
    let check = check_ctx(&ctx, &lowered, &prod_stem);
    let errors = validate::validate(&map, &check);
    if !errors.is_empty() {
        return Err(Error::Validate(format!(
            "robot {}:\n  {}",
            p.robot.ns,
            errors.join("\n  ")
        )));
    }
    for (f, stem) in &round_trip {
        let back = parse::parse_folge(
            &ctx,
            &map[&format!("{stem}.src")],
            &map[&format!("{stem}.dat")],
        )
        .map_err(|e| Error::Verify(format!("robot {}: {stem}: round trip: {e}", p.robot.ns)))?;
        if &back != f {
            return Err(Error::Verify(format!(
                "robot {}: {stem}: the KRL does not parse back into the IR",
                p.robot.ns
            )));
        }
    }
    let install = report::install_md(&ctx, &lowered, &files);
    files.push((report::INSTALL_FILE.into(), install));
    files.sort();
    let manifest = report::manifest(&ctx, &lowered, &files, check);
    let mut map: BTreeMap<String, String> = files.into_iter().collect();
    map.insert(report::MANIFEST_FILE.into(), manifest);
    Ok(RobotOut {
        ns: p.robot.ns.clone(),
        setup,
        lowered,
        io,
        files: map,
    })
}

/// `cell_plan.json` plan id and the robot programs of a bt2plc output directory.
pub fn load_bt2plc_dir(dir: &Path) -> Result<Vec<Program>> {
    let cp = dir.join("cell_plan.json");
    let text = std::fs::read_to_string(&cp).map_err(|e| Error::io(&cp, e))?;
    let plan: program::CellPlanRef =
        serde_json::from_str(&text).map_err(|e| Error::Input(format!("{}: {e}", cp.display())))?;
    let robots = dir.join("robots");
    let mut programs = Vec::new();
    let mut entries: Vec<_> = std::fs::read_dir(&robots)
        .map_err(|e| Error::io(&robots, e))?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .collect();
    entries.sort();
    for d in entries {
        let f = d.join("program.json");
        if !f.is_file() {
            continue;
        }
        let p = Program::load(&f)?;
        if p.plan.plan_id != plan.source.plan_id {
            return Err(Error::Input(format!(
                "{}: plan_id {} differs from cell_plan.json {} (stale bt2plc output)",
                f.display(),
                p.plan.plan_id,
                plan.source.plan_id
            )));
        }
        programs.push(p);
    }
    if programs.is_empty() {
        return Err(Error::Input(format!(
            "{}: no robots/<ns>/program.json",
            dir.display()
        )));
    }
    Ok(programs)
}

pub fn generate(programs: &[Program], cfg: &Config, opts: &Options) -> Result<Generated> {
    cfg.validate()?;
    let mut out = Output::default();
    let mut robots = Vec::new();
    let mut warnings = Vec::new();
    if let Some(sel) = &opts.robots {
        for ns in sel {
            if !programs.iter().any(|p| &p.robot.ns == ns) {
                return Err(Error::Usage(format!("--robots: no program for robot {ns}")));
            }
        }
    }
    for c in &cfg.robots {
        if !programs.iter().any(|p| p.robot.ns == c.ns) {
            warnings.push(format!("bt2krl.toml: [[robot]] {} has no program", c.ns));
        }
    }
    for p in programs {
        if let Some(sel) = &opts.robots
            && !sel.contains(&p.robot.ns)
        {
            continue;
        }
        if !output::safe_rel(&p.robot.ns) || p.robot.ns.contains('/') {
            return Err(Error::Input(format!(
                "robot ns {:?} is not a directory name",
                p.robot.ns
            )));
        }
        let r = generate_program(p, cfg, opts)?;
        warnings.extend(r.lowered.warnings.iter().cloned());
        for (rel, text) in &r.files {
            out.add(format!("{}/{rel}", r.ns), text.clone())?;
        }
        robots.push(r);
    }
    if opts.strict && !warnings.is_empty() {
        return Err(Error::Strict(warnings.join("; ")));
    }
    Ok(Generated {
        robots,
        output: out,
        warnings,
    })
}

/// `bt2krl check <DIR>`: re-validate the KRL of every robot directory against its manifest.
pub fn check_dir(dir: &Path) -> Result<Vec<String>> {
    let mut report = Vec::new();
    let mut found = false;
    let mut entries: Vec<_> = std::fs::read_dir(dir)
        .map_err(|e| Error::io(dir, e))?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .collect();
    entries.sort();
    let mut errors = Vec::new();
    for d in entries {
        let mf = d.join(report::MANIFEST_FILE);
        if !mf.is_file() {
            continue;
        }
        found = true;
        let text = std::fs::read_to_string(&mf).map_err(|e| Error::io(&mf, e))?;
        let m: report::Manifest = serde_json::from_str(&text)
            .map_err(|e| Error::Input(format!("{}: {e}", mf.display())))?;
        let mut files = BTreeMap::new();
        for fh in &m.files {
            let p = d.join(&fh.path);
            let t = std::fs::read_to_string(&p).map_err(|e| Error::io(&p, e))?;
            let h = format!("{:016x}", krl::fnv1a64(t.as_bytes()));
            if h != fh.fnv1a64 {
                errors.push(format!("{}: changed since generation (hash)", p.display()));
            }
            files.insert(fh.path.clone(), t);
        }
        let e = validate::validate(&files, &m.check);
        report.push(format!(
            "{}: {} files, {} ops, profile {}, {}",
            m.robot.ns,
            files.len(),
            m.check.ops.len(),
            m.profile,
            if e.is_empty() {
                "valid".to_string()
            } else {
                format!("{} errors", e.len())
            }
        ));
        errors.extend(e.into_iter().map(|x| format!("{}: {x}", m.robot.ns)));
    }
    if !found {
        return Err(Error::Input(format!(
            "{}: no <ns>/{} found",
            dir.display(),
            report::MANIFEST_FILE
        )));
    }
    if errors.iter().any(|e| e.contains("(hash)")) && errors.iter().all(|e| e.contains("(hash)")) {
        return Err(Error::Verify(errors.join("\n")));
    }
    if !errors.is_empty() {
        return Err(Error::Validate(errors.join("\n")));
    }
    Ok(report)
}
