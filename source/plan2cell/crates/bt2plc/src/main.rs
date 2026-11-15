//! `bt2plc` command line.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{Parser, Subcommand, ValueEnum};

use bt2plc::backend::Flavour;
use bt2plc::config::{GrantPolicy, JobNumber, RobotCfg};
use bt2plc::input::{LoadOptions, SourceSel, load};
use bt2plc::{Config, Diagnostics, Error, GenerateOptions};

#[derive(Parser)]
#[command(name = "bt2plc", version)]
#[command(
    about = "Behaviour-tree plan -> head-PLC code with robot-held interlock zones (CODESYS), robot simulators and build manifests"
)]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
    /// Less output (diagnostics only).
    #[arg(short, long, global = true)]
    quiet: bool,
}

#[derive(Clone, Copy, ValueEnum)]
enum FlavourArg {
    Codesys,
    Twincat3,
    TiaScl,
    PlcopenXml,
    RockwellL5x,
}

impl From<FlavourArg> for Flavour {
    fn from(f: FlavourArg) -> Self {
        match f {
            FlavourArg::Codesys => Flavour::Codesys,
            FlavourArg::Twincat3 => Flavour::Twincat3,
            FlavourArg::TiaScl => Flavour::TiaScl,
            FlavourArg::PlcopenXml => Flavour::PlcopenXml,
            FlavourArg::RockwellL5x => Flavour::RockwellL5x,
        }
    }
}

#[derive(Clone, Copy, ValueEnum)]
enum ZonesFormat {
    Text,
    Md,
    Json,
}

#[derive(clap::Args)]
struct PlanArgs {
    /// Plan JSON (umarell envelope / behaviour_tree object) or BT.CPP v4 XML.
    input: PathBuf,
    /// bt2plc.toml (all fields optional; defaults reproduce the docker simulation).
    #[arg(short, long)]
    config: Option<PathBuf>,
    /// Which tree to lower from a JSON plan: the `root` mirror or the embedded `xml`.
    #[arg(long, value_enum, default_value = "auto")]
    source: SourceSel,
    /// Blackboard JSON (for plain XML input).
    #[arg(long)]
    blackboard: Option<PathBuf>,
    /// Zone grant order.
    #[arg(long, value_enum)]
    grant: Option<GrantPolicy>,
    /// Job (Folge) number layout in the VASS telegram.
    #[arg(long, value_enum)]
    job_number: Option<JobNumber>,
    /// Production Folge number (default 11).
    #[arg(long, value_parser = clap::value_parser!(u16).range(1..))]
    prod_folge: Option<u16>,
    /// Home Folge number (default 125).
    #[arg(long, value_parser = clap::value_parser!(u16).range(1..))]
    home_folge: Option<u16>,
    /// Also include these robots (comma-separated ns) even without ops (bring-up/homing only).
    #[arg(long, value_delimiter = ',')]
    robots: Vec<String>,
    /// Treat depends_on_joints/positions and has_interlock as warnings and ignore them.
    #[arg(long)]
    ignore_unsupported_gates: bool,
    /// Promote approximation warnings to errors.
    #[arg(long)]
    strict: bool,
}

#[derive(Subcommand)]
enum Cmd {
    /// Generate PLC code, robot simulators, project/hardware manifests and docs.
    Generate {
        #[command(flatten)]
        plan: PlanArgs,
        /// Output directory.
        #[arg(short, long)]
        out: PathBuf,
        #[arg(long, value_enum, default_value = "codesys")]
        flavour: FlavourArg,
        /// Do not generate the per-robot simulator projects.
        #[arg(long)]
        no_robot_sim: bool,
        /// Skip the trust-platform check of the generated ST.
        #[arg(long)]
        no_check: bool,
        /// Overwrite a non-empty output directory.
        #[arg(long)]
        force: bool,
        /// List the files that would be written.
        #[arg(long)]
        dry_run: bool,
    },
    /// Plan + in-memory generation + trust check of the ST; writes nothing.
    Check {
        #[command(flatten)]
        plan: PlanArgs,
        /// Print the trust diagnostics (incl. warnings) as JSON.
        #[arg(long)]
        json: bool,
        /// Lower both the JSON root mirror and the embedded XML and require identical plans.
        #[arg(long)]
        cross_check: bool,
        /// Also list trust warnings (style lints such as complexity and time values).
        #[arg(long)]
        warnings: bool,
    },
    /// Show the interlock zones and robot programs.
    Zones {
        #[command(flatten)]
        plan: PlanArgs,
        #[arg(long, value_enum, default_value = "text")]
        format: ZonesFormat,
    },
    /// Co-simulate the generated head PLC and robot simulators (trust-runtime).
    Simulate {
        #[command(flatten)]
        plan: PlanArgs,
        /// Number of production cycles to run.
        #[arg(long, default_value_t = 1)]
        cycles: u32,
        /// Print the protocol timeline.
        #[arg(long)]
        timeline: bool,
    },
    /// Print the default configuration (TOML).
    Config,
    /// List the PLC flavours.
    Flavours,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    match run(cli) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::from(e.exit_code() as u8)
        }
    }
}

fn config(a: &PlanArgs) -> Result<Config, Error> {
    let mut cfg = match &a.config {
        Some(p) => Config::load(p)?,
        None => Config::default(),
    };
    if let Some(g) = a.grant {
        cfg.cell.grant_policy = g;
    }
    if let Some(j) = a.job_number {
        cfg.cell.job_number = j;
    }
    if let Some(f) = a.prod_folge {
        cfg.cell.prod_folge = f;
    }
    if let Some(f) = a.home_folge {
        cfg.cell.home_folge = f;
    }
    if a.ignore_unsupported_gates {
        cfg.cell.ignore_unsupported_gates = true;
    }
    for ns in &a.robots {
        match cfg.robots.iter_mut().find(|r| &r.ns == ns) {
            Some(r) => r.include_if_idle = true,
            None => cfg.robots.push(RobotCfg {
                ns: ns.clone(),
                include_if_idle: true,
                ..Default::default()
            }),
        }
    }
    cfg.validate()?;
    Ok(cfg)
}

fn load_doc(a: &PlanArgs, d: &mut Diagnostics) -> Result<bt2plc::input::InputDoc, Error> {
    load(
        &a.input,
        &LoadOptions {
            source: a.source,
            blackboard: a.blackboard.clone(),
        },
        d,
    )
}

fn print_diags(d: &Diagnostics) {
    for x in d.items() {
        eprintln!("{x}");
    }
}

fn summary(g: &bt2plc::Generated) -> String {
    let p = &g.plan;
    format!(
        "{} robot(s), {} op(s), {} dependency edge(s), {} zone(s){}, makespan {:.3} s",
        p.robots.len(),
        p.ops.len(),
        p.cross_edges.len(),
        p.zones.len(),
        if p.permit_edges.is_empty() {
            String::new()
        } else {
            format!(" + {} permit edge(s)", p.permit_edges.len())
        },
        p.makespan_ms as f64 / 1000.0
    )
}

fn run(cli: Cli) -> Result<(), Error> {
    match cli.cmd {
        Cmd::Config => {
            print!("{}", Config::default_toml());
            Ok(())
        }
        Cmd::Flavours => {
            for f in Flavour::ALL {
                println!("{:<14} {}", f.to_string(), f.describe());
            }
            Ok(())
        }
        Cmd::Generate {
            plan,
            out,
            flavour,
            no_robot_sim,
            no_check,
            force,
            dry_run,
        } => {
            let mut cfg = config(&plan)?;
            if no_robot_sim {
                cfg.robot_sim.enabled = false;
            }
            let mut d0 = Diagnostics::default();
            let doc = load_doc(&plan, &mut d0)?;
            print_diags(&d0);
            let opts = GenerateOptions {
                flavour: flavour.into(),
                strict: plan.strict,
                st_check: !no_check && cfg!(feature = "check"),
            };
            let g = bt2plc::generate(&doc, &cfg, &opts)?;
            print_diags(&g.diagnostics);
            if dry_run {
                for f in g.output.files().keys() {
                    println!("{f}");
                }
                return Ok(());
            }
            g.output.write_to(&out, force)?;
            if !cli.quiet {
                println!("{}", summary(&g));
                for z in &g.plan.zones {
                    let t: Vec<String> = z.tickets.iter().map(|t| g.plan.op_ref(t.op)).collect();
                    println!("  {} = {}", z.name, t.join(" > "));
                }
                println!(
                    "wrote {} files to {} (ST check: {})",
                    g.output.files().len(),
                    out.display(),
                    if g.st_checked { "trust OK" } else { "skipped" }
                );
            }
            Ok(())
        }
        Cmd::Check {
            plan,
            json,
            cross_check,
            warnings,
        } => {
            let cfg = config(&plan)?;
            let mut d0 = Diagnostics::default();
            let doc = load_doc(&plan, &mut d0)?;
            print_diags(&d0);
            if cross_check {
                cross_check_sources(&plan, &cfg)?;
            }
            let opts = GenerateOptions {
                flavour: Flavour::Codesys,
                strict: plan.strict,
                st_check: false,
            };
            let g = bt2plc::generate(&doc, &cfg, &opts)?;
            print_diags(&g.diagnostics);
            check_st(&g, json, warnings)?;
            if !cli.quiet {
                println!("{}", summary(&g));
            }
            Ok(())
        }
        Cmd::Zones { plan, format } => {
            let cfg = config(&plan)?;
            let mut d = Diagnostics::default();
            let doc = load_doc(&plan, &mut d)?;
            let p = bt2plc::plan(&doc, &cfg, &mut d)?;
            print_diags(&d);
            match format {
                ZonesFormat::Md => print!("{}", bt2plc::backend::common::zones_md::render(&p, &d)),
                ZonesFormat::Json => print!(
                    "{}",
                    bt2plc::backend::common::cell_plan_json::render(&p, &cfg, &d)
                ),
                ZonesFormat::Text => {
                    for z in &p.zones {
                        let t: Vec<String> = z.tickets.iter().map(|t| p.op_ref(t.op)).collect();
                        println!("{} = {}", z.name, t.join(" > "));
                    }
                    for r in &p.robots {
                        let ops: Vec<String> = r
                            .ops
                            .iter()
                            .map(|&o| {
                                let op = &p.ops[o];
                                let mut tags: Vec<String> =
                                    op.zones.iter().map(|&z| p.zones[z].name.clone()).collect();
                                if !op.permit_preds.is_empty() {
                                    tags.push("PERMIT".into());
                                }
                                if tags.is_empty() {
                                    op.operation_id.clone()
                                } else {
                                    format!("{}[{}]", op.operation_id, tags.join(","))
                                }
                            })
                            .collect();
                        println!("{}: {}", r.ns, ops.join(" "));
                    }
                    println!("makespan {:.3} s", p.makespan_ms as f64 / 1000.0);
                }
            }
            Ok(())
        }
        Cmd::Simulate {
            plan,
            cycles,
            timeline,
        } => simulate(&plan, cycles, timeline, cli.quiet),
    }
}

fn cross_check_sources(plan: &PlanArgs, cfg: &Config) -> Result<(), Error> {
    let mut d = Diagnostics::default();
    let opts = |s| LoadOptions {
        source: s,
        blackboard: plan.blackboard.clone(),
    };
    let j = bt2plc::plan(
        &load(&plan.input, &opts(SourceSel::Json), &mut d)?,
        cfg,
        &mut d,
    )?;
    let x = bt2plc::plan(
        &load(&plan.input, &opts(SourceSel::Xml), &mut d)?,
        cfg,
        &mut d,
    )?;
    let sig = |p: &bt2plc::ir::CellPlan| {
        let zones: Vec<Vec<String>> = p
            .zones
            .iter()
            .map(|z| z.tickets.iter().map(|t| p.op_ref(t.op)).collect())
            .collect();
        let edges: Vec<(String, String)> = p
            .cross_edges
            .iter()
            .map(|e| (p.op_ref(e.from), p.op_ref(e.to)))
            .collect();
        (zones, edges, p.makespan_ms)
    };
    if sig(&j) != sig(&x) {
        return Err(Error::plan(
            bt2plc::DiagCode::E005,
            None,
            "the JSON root mirror and the embedded XML lower to different plans",
        ));
    }
    Ok(())
}

#[cfg(feature = "check")]
fn check_st(g: &bt2plc::Generated, json: bool, warnings: bool) -> Result<(), Error> {
    let diags = bt2plc::trust::check::check_projects(&g.projects, true);
    let n_warn = diags.iter().filter(|d| d.severity != "error").count();
    if json {
        println!(
            "{}",
            serde_json::to_string_pretty(&diags).unwrap_or_default()
        );
    } else {
        for x in diags.iter().filter(|d| warnings || d.severity == "error") {
            eprintln!("{x}");
        }
        let n_err = diags.len() - n_warn;
        eprintln!(
            "trust: {} project(s), {n_err} error(s), {n_warn} warning(s){}",
            g.projects.len(),
            if warnings || n_warn == 0 {
                ""
            } else {
                " (--warnings to list)"
            }
        );
    }
    let errors: Vec<String> = diags
        .iter()
        .filter(|d| d.severity == "error")
        .map(ToString::to_string)
        .collect();
    if errors.is_empty() {
        Ok(())
    } else {
        Err(Error::StCheck(errors.join("\n")))
    }
}

#[cfg(not(feature = "check"))]
fn check_st(_g: &bt2plc::Generated, _json: bool, _warnings: bool) -> Result<(), Error> {
    Err(Error::Config(
        "bt2plc was built without the 'check' feature (trust-platform)".into(),
    ))
}

#[cfg(feature = "sim")]
fn simulate(plan: &PlanArgs, cycles: u32, timeline: bool, quiet: bool) -> Result<(), Error> {
    let cfg = config(plan)?;
    let mut d0 = Diagnostics::default();
    let doc = load_doc(plan, &mut d0)?;
    let g = bt2plc::generate(
        &doc,
        &cfg,
        &GenerateOptions {
            st_check: false,
            ..Default::default()
        },
    )?;
    let report = bt2plc::trust::cosim::run(
        &g.plan,
        &cfg,
        &g.projects,
        &bt2plc::trust::cosim::SimOptions {
            cycles,
            ..Default::default()
        },
    )?;
    if timeline {
        for e in &report.events {
            println!("{e}");
        }
    }
    if !quiet {
        println!("{}", report.summary());
    }
    report.verify(&g.plan).map_err(Error::Sim)
}

#[cfg(not(feature = "sim"))]
fn simulate(_plan: &PlanArgs, _cycles: u32, _timeline: bool, _quiet: bool) -> Result<(), Error> {
    Err(Error::Config(
        "bt2plc was built without the 'sim' feature (cargo build --features sim)".into(),
    ))
}

#[allow(dead_code)]
fn exists(p: &Path) -> bool {
    p.exists()
}
