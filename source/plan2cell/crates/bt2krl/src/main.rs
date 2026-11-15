//! `bt2krl` command line (TASK section 4.1).

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{Parser, Subcommand};

use bt2krl::config::MotionMode;
use bt2krl::{Config, Options, Profile, Result};

#[derive(Parser)]
#[command(
    name = "bt2krl",
    version,
    about = "bt2plc robot programs -> KUKA KRL (production/home Folge with the head-PLC zone interlocks)"
)]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Generate the KRL of every robot of a bt2plc output directory.
    Generate {
        /// Output directory of `bt2plc generate` (cell_plan.json, robots/<ns>/program.json).
        bt2plc_out: PathBuf,
        #[arg(short, long)]
        out: PathBuf,
        #[arg(short, long)]
        config: Option<PathBuf>,
        #[arg(long, value_enum, default_value = "sim")]
        profile: Profile,
        #[arg(long, value_enum)]
        motion: Option<MotionMode>,
        /// Comma-separated robot namespaces (default: all).
        #[arg(long, value_delimiter = ',')]
        robots: Option<Vec<String>>,
        /// Overwrite a non-empty output directory.
        #[arg(long)]
        force: bool,
        /// Generate and validate, write nothing.
        #[arg(long)]
        dry_run: bool,
        /// Warnings are errors (exit 6).
        #[arg(long)]
        strict: bool,
    },
    /// Re-validate generated KRL (strict validator, file hashes).
    Check { dir: PathBuf },
    /// Print the op / zone / IO plan of one robot program (writes nothing).
    Explain {
        program: PathBuf,
        #[arg(short, long)]
        config: Option<PathBuf>,
    },
    /// Print the default bt2krl.toml.
    Config,
}

fn load_cfg(p: &Option<PathBuf>) -> Result<Config> {
    match p {
        Some(p) => Config::load(p),
        None => Ok(Config::default()),
    }
}

fn run(cli: Cli) -> Result<()> {
    match cli.cmd {
        Cmd::Config => {
            print!("{}", Config::default_toml());
            Ok(())
        }
        Cmd::Generate {
            bt2plc_out,
            out,
            config,
            profile,
            motion,
            robots,
            force,
            dry_run,
            strict,
        } => {
            let cfg = load_cfg(&config)?;
            let programs = bt2krl::load_bt2plc_dir(&bt2plc_out)?;
            let opts = Options {
                profile,
                motion,
                robots,
                strict,
            };
            let g = bt2krl::generate(&programs, &cfg, &opts)?;
            for w in &g.warnings {
                eprintln!("warning: {w}");
            }
            for r in &g.robots {
                let home = match &r.lowered.home {
                    Some(h) => format!("Folge {} home", h.number),
                    None => "no home Folge (vw-ks24: cell.src homes the robot)".into(),
                };
                println!(
                    "{}: Folge {} ({} ops, {} points), {home}, profile {}, model {}",
                    r.ns,
                    r.lowered.prod.number,
                    r.lowered.prod.ops.len(),
                    r.lowered.prod.points.len(),
                    profile.name(),
                    r.setup.model.name
                );
            }
            if dry_run {
                for f in g.output.files().keys() {
                    println!("  {f}");
                }
                println!(
                    "dry run: {} files validated, nothing written",
                    g.output.files().len()
                );
            } else {
                g.output.write_to(&out, force)?;
                println!(
                    "wrote {} files to {} (validator OK, IR round trip OK)",
                    g.output.files().len(),
                    out.display()
                );
            }
            Ok(())
        }
        Cmd::Check { dir } => {
            for line in bt2krl::check_dir(&dir)? {
                println!("{line}");
            }
            Ok(())
        }
        Cmd::Explain { program, config } => {
            let cfg = load_cfg(&config)?;
            explain(&program, &cfg)
        }
    }
}

fn explain(path: &Path, cfg: &Config) -> Result<()> {
    use bt2krl::ir::Instr;
    let p = bt2krl::program::Program::load(path)?;
    let s = bt2krl::lower::RobotSetup::resolve(&p, cfg, false)?;
    let l = bt2krl::lower::lower(&p, &s, cfg, Profile::Sim)?;
    let io = bt2krl::io::SimIo::new(&p, &cfg.sim);
    println!(
        "robot {} ({}), plan {} PLAN_ID {}, model {}, joint mapping {}",
        p.robot.ns,
        p.robot.tag,
        p.plan.tree_id,
        p.plan.plan_id,
        s.model.name,
        p.joint_mapping.describe()
    );
    println!(
        "Folge {} production, {}; job number {}",
        l.prod.number,
        l.home
            .as_ref()
            .map_or("no home Folge".to_string(), |h| format!(
                "Folge {} home",
                h.number
            )),
        p.protocol.job_number
    );
    println!(
        "sim IO: STEP $OUT[{}..{}], REQUEST bit b $OUT[{}+b], GRANT bit b $IN[{}+b], PLAN_ID $IN[{}..{}]",
        io.step,
        io.step + 15,
        io.request,
        io.grant,
        io.plan_id,
        io.plan_id + 15
    );
    for z in &p.zones {
        println!(
            "zone {} bit {} -> REQUEST $OUT[{}], GRANT $IN[{}]",
            z.name,
            z.bit,
            io.req_bit(u32::from(z.bit)),
            io.grant_bit(u32::from(z.bit))
        );
    }
    for (o, e) in l.prod.ops.iter().zip(&l.estimates) {
        let moves = o
            .instrs
            .iter()
            .filter(|i| matches!(i, Instr::Move { .. }))
            .count();
        let src = p.ops.iter().find(|x| x.index == o.index);
        println!(
            "op {:>3} {:<12} mask 16#{:08X} {:<16} {:>3} points  est {:>7.3} s  expected {:>7.3} s  limit {:>7.3} s{}",
            o.index,
            o.operation_id,
            o.mask,
            src.map(|x| x.zones.join(",")).unwrap_or_default(),
            moves,
            e.est_s,
            e.expected_s,
            e.limit_s,
            src.filter(|x| !x.waits_for.is_empty())
                .map(|x| format!("  waits for {}", x.waits_for.join(", ")))
                .unwrap_or_default()
        );
    }
    for w in &l.warnings {
        println!("warning: {w}");
    }
    Ok(())
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
