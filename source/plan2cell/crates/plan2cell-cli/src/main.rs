//! `plan2cell`: one command for the whole cell: plan -> head-PLC code + KUKA robot programs,
//! co-simulation, the same as an MCP server, and the build brought up on the cell.

use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{Args, Parser, Subcommand};

use plan2cell::report::{BuildReport, Diagnostic};
use plan2cell::targets::RobotLanguage;
use plan2cell::{BuildConfig, BuildOptions, PlanInput, PlcTarget, Profile};

#[derive(Parser)]
#[command(
    name = "plan2cell",
    version,
    about = "Behaviour-tree plan -> head-PLC code (CODESYS) and KUKA KRL robot programs that work together"
)]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Args)]
struct PlanArgs {
    /// Plan JSON (umarell envelope / behaviour_tree object) or BT.CPP v4 XML.
    plan: PathBuf,
    /// plan2cell.toml ([build], [plc] = bt2plc.toml, [krl] = bt2krl.toml).
    #[arg(short, long)]
    config: Option<PathBuf>,
    /// Blackboard JSON for a plain BT.CPP XML tree.
    #[arg(long)]
    blackboard: Option<PathBuf>,
    /// Robot-side protocol of the whole cell (head PLC and robots always agree).
    #[arg(long, value_enum)]
    profile: Option<Profile>,
    /// Head-PLC code to write.
    #[arg(long, value_enum)]
    plc: Option<PlcTarget>,
    /// Write the robot simulator projects (profile sim).
    #[arg(long, overrides_with = "no_robot_sim")]
    robot_sim: bool,
    #[arg(long)]
    no_robot_sim: bool,
    /// Write the KUKA KRL robot programs.
    #[arg(long, overrides_with = "no_krl")]
    krl: bool,
    #[arg(long)]
    no_krl: bool,
    /// Only these robots get KRL (comma-separated namespaces).
    #[arg(long, value_delimiter = ',')]
    krl_robots: Option<Vec<String>>,
    /// Skip the trust type check of the generated ST.
    #[arg(long)]
    no_check: bool,
    /// Warnings are errors (exit 6).
    #[arg(long)]
    strict: bool,
    /// PLC dialect (`plan2cell options` lists them); only codesys is implemented, the others
    /// fail with exit code 5.
    #[arg(long, default_value = "codesys")]
    plc_dialect: String,
    /// Robot program language; only krl is implemented, the others fail with exit code 5.
    #[arg(long, value_enum, default_value_t = RobotLanguage::Krl)]
    robot_language: RobotLanguage,
    /// Declare `e_axes = "none"` for every robot whose [[krl.robot]] does not declare its
    /// external axes (an explicit opt-in; each such robot gets a note).
    #[arg(long, value_parser = ["none"])]
    krl_e_axes_default: Option<String>,
}

#[derive(Subcommand)]
enum Cmd {
    /// Build: <out>/plc (bt2plc), <out>/krl (bt2krl), <out>/BUILD.md.
    Build {
        #[command(flatten)]
        plan: PlanArgs,
        #[arg(short, long)]
        out: PathBuf,
        /// Overwrite a non-empty output directory (stale files of the last build are removed).
        #[arg(long)]
        force: bool,
        /// Build and validate, write nothing.
        #[arg(long)]
        dry_run: bool,
        /// Print one JSON report on stdout (ok, summary, diagnostics, files, notes), also when
        /// the build fails; the exit code stays the same.
        #[arg(long)]
        json: bool,
    },
    /// What can be built: PLC dialects, robot languages, profiles and the defaults.
    Options {
        /// As JSON (for front ends).
        #[arg(long)]
        json: bool,
    },
    /// Build in memory and validate everything; write nothing.
    Check {
        #[command(flatten)]
        plan: PlanArgs,
        /// Print the build summary as JSON.
        #[arg(long)]
        json: bool,
    },
    /// The effective hardware of the plan and configuration (what `build` uses): the PROFINET
    /// network, each robot's station name / device name / IP / model, the KUKA virtual
    /// controllers of the external robots, and every broken addressing rule (exit 3 if any).
    /// Overrides: plan2cell.toml [plc.network] and [[plc.robot]] station / device_name / ip.
    Hardware {
        #[command(flatten)]
        plan: PlanArgs,
        /// Print the report as JSON (also when it fails: {"ok": false, "error": ...}).
        #[arg(long)]
        json: bool,
    },
    /// The interlock zones, their ticket order and each robot's ops.
    Zones {
        #[command(flatten)]
        plan: PlanArgs,
    },
    /// One robot's KRL plan (ops, masks, estimated vs planned times).
    Explain {
        #[command(flatten)]
        plan: PlanArgs,
        /// Robot namespace.
        #[arg(long)]
        robot: String,
    },
    /// Co-simulate the head PLC with the robots and check the trace (build with --features sim).
    Simulate {
        #[command(flatten)]
        plan: PlanArgs,
        #[arg(long, default_value_t = 1)]
        cycles: u32,
        /// Who plays the robots: the executor of the generated KRL, or the device-simulator ST.
        #[arg(long, value_parser = ["krl", "device-st"], default_value = "krl")]
        robots: String,
    },
    /// Print the default plan2cell.toml.
    Config,
    /// Serve the plan2cell tools over MCP.
    Mcp {
        /// Serve over stdin/stdout (for a client that starts this process).
        #[arg(long, conflicts_with = "http")]
        stdio: bool,
        /// Serve streamable HTTP at http://ADDR/mcp (e.g. 127.0.0.1:8765).
        #[arg(long)]
        http: Option<SocketAddr>,
        /// Plans, configs and outputs are paths relative to this directory.
        #[arg(long, default_value = ".")]
        root: PathBuf,
        /// Default plan2cell.toml (relative to --root).
        #[arg(short, long)]
        config: Option<PathBuf>,
        /// Allow plan2cell_build to write into --root.
        #[arg(long)]
        allow_write: bool,
        /// Bearer token for HTTP (required for a non-loopback address). Default: $PLAN2CELL_MCP_TOKEN.
        #[arg(long, env = "PLAN2CELL_MCP_TOKEN", hide_env_values = true)]
        token: Option<String>,
        /// Extra Host header values to accept over HTTP (a DNS name or IP clients use).
        #[arg(long, value_delimiter = ',')]
        allowed_host: Vec<String>,
    },
    /// Bring a build up on the cell: PLC host runtimes, CODESYS projects over MCP, deploy, verify
    /// (build with --features cell).
    Cell {
        #[command(flatten)]
        args: CellArgs,
        #[command(subcommand)]
        cmd: CellCmd,
    },
}

#[derive(Args)]
struct CellArgs {
    /// cell.toml with the endpoints of the cell. Default: ~/.config/plan2cell/cell.toml.
    #[arg(long, global = true)]
    cell_config: Option<PathBuf>,
    /// Print the planned ssh and MCP calls; send nothing.
    #[arg(long, global = true)]
    dry_run: bool,
    /// Print the report as JSON.
    #[arg(long, global = true)]
    json: bool,
    /// verify: read again until every check passes or this many seconds have passed.
    #[arg(long, global = true, default_value_t = 120)]
    verify_timeout: u64,
}

#[derive(Subcommand)]
enum CellCmd {
    /// Build the plan into --out, then preflight, runtimes, projects, deploy, krl (with
    /// [kuka_vc]) and verify.
    Up {
        #[command(flatten)]
        plan: PlanArgs,
        #[arg(short, long)]
        out: PathBuf,
        /// Overwrite a non-empty directory that holds no plan2cell build.
        #[arg(long)]
        force: bool,
    },
    /// Check the PLC host (ssh), the CODESYS MCP server (tools/list has every tool plan2cell
    /// calls) and the IDE (no CODESYS opened by hand in the way, device descriptions of DIR
    /// installed).
    Preflight { dir: Option<PathBuf> },
    /// Push the robots overlay of DIR to the PLC host and run its setup (one container per robot).
    Runtimes { dir: PathBuf },
    /// Create, build and save every CODESYS project of DIR with the typed CODESYS MCP tools
    /// (robot simulators first), reading back what each call set.
    Projects { dir: PathBuf },
    /// Log in by direct IP, download, create the boot application and start (same order).
    Deploy { dir: PathBuf },
    /// Put the KRL of DIR onto the KUKA virtual controllers of [kuka_vc] in cell.toml (SSM or
    /// ssh): hash check, upload what differs, $config.dat / sps.sub between ;BT2KRL markers,
    /// compile errors, submit restart.
    Krl { dir: PathBuf },
    /// Read the acceptance values online (KUKA VC checks, PLAN_ID, state, PROFINET connections).
    Verify { dir: PathBuf },
    /// Bring the deployed cell into production: restart runtimes that do not answer (a demo
    /// runtime stops after ~2 h), log in to the head, acknowledge the KUKA VCs' stop messages
    /// (kvc-ack.sh: M_2858 after a restart) and put them into EXT (kvc-mode.sh), then pulse the
    /// head's bReset / bRecoveryConfirm until it runs (CS_RUNNING / CS_CYCLE_DONE); a robot still
    /// in ERR_START_TIMEOUT / ERR_DRIVES_TIMEOUT after 3 pulses gets kvc-ack.sh once more; logs
    /// out at the end.
    Start {
        dir: PathBuf,
        /// Restart the runtime containers even if they answer (a fresh demo period).
        #[arg(long)]
        restart_runtime: bool,
        /// Give up after this many seconds.
        #[arg(long, default_value_t = 300)]
        timeout: u64,
        /// At most this many operator pulses.
        #[arg(long, default_value_t = 8)]
        pulses: u32,
        /// Try the head's login again (gateway, connection, refused login) for this many seconds,
        /// counted again after a runtime restart; a build or library error is exit 8 at once.
        #[arg(long, default_value_t = 60)]
        login_timeout: u64,
    },
    /// Abort the running cell (pulse PLC_PRG.bStop; the head ends in CS_ERROR ERR_OPERATOR_STOP).
    Stop { dir: PathBuf },
    /// Restart the IDE's CODESYS through the MCP server: with DIR, log out of and close its
    /// projects without saving, then shutdown_codesys (unsaved changes discarded), launch_codesys
    /// and get_codesys_status. Exit 0 when CODESYS is ready again, 8 when not, 4 when the MCP
    /// server does not answer, 3 for a dialog nobody answered.
    ResetIde { dir: Option<PathBuf> },
    /// Read-only, in seconds: can DIR be deployed to and run on the cell of cell.toml? Every rule
    /// passes, warns or fails with a hint and, where it is mechanical, a fix (see
    /// plan2cell::deploy_check). The KUKA VCs' $config.dat is read over SSM / ssh (else the
    /// profile's bundled list). Exit 7 when a rule fails.
    Check {
        dir: PathBuf,
        /// Do not read the controllers' $config.dat (the profile's bundled list).
        #[arg(long)]
        offline: bool,
        /// Only the rules that need no cell (no cell.toml needed).
        #[arg(long)]
        export_only: bool,
    },
    /// Read-only: per CODESYS runtime container of the PLC host (DIR's, else every
    /// codesys-master / codesys-device* there) whether it runs, since when, its demo-mode and
    /// shutdown-warning log lines and when its 2 h demo period ends (`start --restart-runtime`
    /// begins a new one).
    Runtime { dir: Option<PathBuf> },
    /// One operation on one KUKA virtual controller of [kuka_vc] (its own SSM / ssh / kubectl
    /// remote), from a fixed list: status, mode show|T1|T2|AUT|EXT, ack, restart, program
    /// show|cancel|select[=MODULE], braketest-need. Prints JSON; exit 1 when it failed.
    Kvc {
        /// The robot (its namespace in the plan, `robot_ns` in cell.toml).
        robot: String,
        op: String,
        arg: Option<String>,
    },
    /// The conformance kit of factoryos/hook@1 (image authors): describe, the verbs of the role,
    /// refusals, the output rules, capabilities, read-only root, listeners, the inbox, health,
    /// no realtime (HC01-HC12). --exec runs against a live pod's container and changes nothing
    /// unless --mutating; --docker starts the image on an --internal docker network (docker 26 or
    /// later; nothing published) and removes it afterwards. Exit 0 conformant, 7 a check failed,
    /// 2 usage, 4 the target did not answer. No cell.toml needed.
    HooksCheck {
        /// A live pod: <namespace>/<pod>/<container>.
        #[arg(long, conflicts_with = "docker", required_unless_present = "docker")]
        exec: Option<String>,
        /// A local image (its container's arguments after --).
        #[arg(long)]
        docker: Option<String>,
        /// --exec: the kubeconfig (default: $KUBECONFIG, else kubectl's).
        #[arg(long, requires = "exec")]
        kubeconfig: Option<String>,
        /// --exec: also the verbs that change something (a throwaway cell only).
        #[arg(long, requires = "exec")]
        mutating: bool,
        /// head or robot (default: describe's role).
        #[arg(long)]
        role: Option<String>,
        /// none, pn-raw or pn-admin (--exec default: the pod's securityContext; --docker: none).
        #[arg(long)]
        profile: Option<String>,
        /// The PN address (--exec default: the pod's net1 addresses from Multus).
        #[arg(long)]
        pn_address: Vec<String>,
        /// UDP on loopback only (no PROFINET exception).
        #[arg(long)]
        strict_udp: bool,
        /// A cell template whose consumer.ops a robot's describe.ops must contain.
        #[arg(long)]
        ops: Option<PathBuf>,
        /// --docker: extra NAME=VALUE environment for the container (e.g. FOS_ROBOT_NS=robot1).
        #[arg(long, requires = "docker")]
        env: Vec<String>,
        /// --docker: the container's arguments.
        #[arg(last = true)]
        args: Vec<String>,
    },
}

fn load(a: &PlanArgs) -> plan2cell::Result<(PlanInput, BuildConfig, BuildOptions)> {
    let (input, cfg, opts, notes) = load_full(a)?;
    for n in notes {
        eprintln!("note: {n}");
    }
    Ok((input, cfg, opts))
}

/// The plan, configuration and options of the flags, and notes on what the flags assumed.
fn load_full(
    a: &PlanArgs,
) -> plan2cell::Result<(PlanInput, BuildConfig, BuildOptions, Vec<String>)> {
    load_with(a, true)
}

/// `checked = false`: the configuration is parsed but not validated (`hardware` reports its
/// problems instead of failing at the first).
fn load_with(
    a: &PlanArgs,
    checked: bool,
) -> plan2cell::Result<(PlanInput, BuildConfig, BuildOptions, Vec<String>)> {
    let dialect = plan2cell::targets::plc_dialect(&a.plc_dialect).ok_or_else(|| {
        let ids: Vec<String> = plan2cell::targets::options()
            .plc_dialects
            .into_iter()
            .map(|c| c.id)
            .collect();
        plan2cell::Error::Usage(format!(
            "unknown PLC dialect {} (one of {})",
            a.plc_dialect,
            ids.join(", ")
        ))
    })?;
    plan2cell::targets::check(dialect, a.robot_language)?;
    let mut cfg = match &a.config {
        Some(p) if checked => BuildConfig::load(p)?,
        Some(p) => BuildConfig::load_unchecked(p)?,
        None => BuildConfig::default(),
    };
    let mut input = PlanInput::load(&a.plan)?;
    if let Some(b) = &a.blackboard {
        input.load_blackboard(b)?;
    }
    let mut o = cfg.options();
    if let Some(p) = a.profile {
        o.profile = p;
    }
    if let Some(p) = a.plc {
        o.plc = p;
    }
    if a.robot_sim {
        o.robot_sim = true;
    }
    if a.no_robot_sim {
        o.robot_sim = false;
    }
    if a.krl {
        o.krl = true;
    }
    if a.no_krl {
        o.krl = false;
    }
    if a.krl_robots.is_some() {
        o.krl_robots = a.krl_robots.clone();
    }
    if a.no_check {
        o.st_check = false;
    }
    if a.strict {
        o.strict = true;
    }
    let notes = if a.krl_e_axes_default.is_some() && o.krl {
        plan2cell::declare_missing_e_axes(&input, &mut cfg, &o)?
    } else {
        Vec::new()
    };
    Ok((input, cfg, o, notes))
}

fn report(b: &plan2cell::Build) {
    for d in &b.diagnostics {
        eprintln!("{d}");
    }
    for n in &b.notes {
        eprintln!("note: {n}");
    }
    println!("{}", summary_line(b));
    for z in &b.summary.zones {
        println!("  {} = {}", z.name, z.tickets.join(" > "));
    }
}

fn summary_line(b: &plan2cell::Build) -> String {
    let s = &b.summary;
    format!(
        "{} (PLAN_ID {}): {} robot(s), {} zone(s), makespan {:.3} s; profile {}, PLC {}, KRL {}, ST check {}",
        s.tree_id,
        s.plan_id,
        s.robots.len(),
        s.zones.len(),
        s.makespan_s,
        s.options.profile.name(),
        match s.options.plc {
            PlcTarget::Codesys => "codesys",
            PlcTarget::None => "none",
        },
        if s.options.krl { "yes" } else { "no" },
        if s.st_checked { "trust OK" } else { "not run" }
    )
}

/// `build --json`: build (and write, unless `out` is `None`), then print one [`BuildReport`].
fn build_json(a: &PlanArgs, out: Option<&Path>, force: bool) -> Result<(), (i32, String)> {
    let mut notes = Vec::new();
    let r = build_and_write(a, out, force, &mut notes);
    let rep = match &r {
        Ok(rep) => rep.clone(),
        Err(e) => BuildReport::failed(e, failed_diagnostics(a, e), notes),
    };
    println!("{}", serde_json::to_string_pretty(&rep).unwrap_or_default());
    r.map(|_| ()).map_err(|e| (e.exit_code(), e.to_string()))
}

fn build_and_write(
    a: &PlanArgs,
    out: Option<&Path>,
    force: bool,
    notes: &mut Vec<String>,
) -> plan2cell::Result<BuildReport> {
    let (input, cfg, opts, n) = load_full(a)?;
    notes.extend(n);
    let b = plan2cell::build(&input, &cfg, &opts)?;
    let mut rep = BuildReport::ok(&b, summary_line(&b), notes.clone());
    if let Some(out) = out {
        b.write_to(out, force)?;
        rep.out = Some(out.display().to_string());
        rep.written = true;
    }
    Ok(rep)
}

/// The diagnostics of a failed build. `--strict` reports only how many warnings it promoted:
/// build again without it (and without the ST check) to list them.
fn failed_diagnostics(a: &PlanArgs, e: &plan2cell::Error) -> Vec<Diagnostic> {
    let mut out = plan2cell::report::error_diagnostics(e);
    if e.is_strict()
        && let Ok((input, cfg, mut opts, _)) = load_full(a)
    {
        opts.strict = false;
        opts.st_check = false;
        if let Ok(b) = plan2cell::build(&input, &cfg, &opts) {
            out.extend(b.diags.into_iter().filter(|d| d.severity != "info"));
        }
    }
    out
}

/// `hardware`: exit 0 when the hardware is valid, 3 when it has issues (the report lists them),
/// the plan's or configuration's own exit code when it cannot be resolved.
fn hardware(a: &PlanArgs, json: bool) -> Result<(), (i32, String)> {
    let r = load_with(a, false).and_then(|(input, cfg, opts, notes)| {
        for n in notes {
            eprintln!("note: {n}");
        }
        plan2cell::hardware::resolve(&input, &cfg, &opts)
    });
    let h = match r {
        Ok(h) => h,
        Err(e) => {
            if json {
                let v = serde_json::json!({
                    "schema": plan2cell::hardware::HARDWARE_SCHEMA,
                    "ok": false,
                    "error": {"code": e.exit_code(), "message": e.to_string()},
                    "issues": [],
                });
                println!("{}", serde_json::to_string_pretty(&v).unwrap_or_default());
            }
            return Err((e.exit_code(), e.to_string()));
        }
    };
    if json {
        println!("{}", serde_json::to_string_pretty(&h).unwrap_or_default());
    } else {
        let n = &h.network;
        println!(
            "head PLC {} ({}/{}), station {}, gateway {}",
            n.head_ip,
            n.head_ip,
            n.prefix,
            n.controller_station,
            if n.gateway.is_unspecified() {
                "none".to_string()
            } else {
                n.gateway.to_string()
            }
        );
        for r in &h.robots {
            println!(
                "  {:<10} {:<4} station {:<14} device {:<12} ip {:<15} model {}{}{}",
                r.ns,
                r.tag,
                r.station,
                r.device_name,
                r.ip,
                r.model,
                if r.external { ", external" } else { "" },
                if r.idle { ", idle" } else { "" }
            );
        }
        for c in &h.kuka_vc.controllers {
            println!(
                "  KUKA VC for {}: KLI {}/{}, station {}",
                c.robot_ns, c.kli_ip, h.kuka_vc.kli_prefix, c.station
            );
        }
        for i in &h.issues {
            println!("issue {}: {}", i.field, i.message);
        }
    }
    if h.ok {
        Ok(())
    } else {
        Err((
            3,
            format!(
                "{} hardware issue(s): {}",
                h.issues.len(),
                h.issues
                    .iter()
                    .map(|i| format!("{}: {}", i.field, i.message))
                    .collect::<Vec<_>>()
                    .join("; ")
            ),
        ))
    }
}

fn print_options(json: bool) {
    let o = plan2cell::targets::options();
    if json {
        println!("{}", serde_json::to_string_pretty(&o).unwrap_or_default());
        return;
    }
    let list = |title: &str, items: &[plan2cell::targets::Choice]| {
        println!("{title}:");
        for c in items {
            println!(
                "  {:<14} {}{}",
                c.id,
                c.label,
                if c.implemented {
                    ""
                } else {
                    " (not implemented yet)"
                }
            );
        }
    };
    list("PLC dialects (--plc-dialect)", &o.plc_dialects);
    list("robot languages (--robot-language)", &o.robot_languages);
    list("profiles (--profile)", &o.profiles);
    list("job numbers ([plc.cell] job_number)", &o.job_numbers);
}

fn run(cli: Cli) -> Result<(), (i32, String)> {
    let e = |e: plan2cell::Error| (e.exit_code(), e.to_string());
    match cli.cmd {
        Cmd::Config => {
            print!("{}", BuildConfig::default_toml());
            Ok(())
        }
        Cmd::Options { json } => {
            print_options(json);
            Ok(())
        }
        Cmd::Build {
            plan,
            out,
            force,
            dry_run,
            json,
        } => {
            if json {
                return build_json(&plan, (!dry_run).then_some(out.as_path()), force);
            }
            let (input, cfg, opts) = load(&plan).map_err(e)?;
            let b = plan2cell::build(&input, &cfg, &opts).map_err(e)?;
            report(&b);
            if dry_run {
                for f in b.files.keys() {
                    println!("  {f}");
                }
                println!("dry run: {} files, nothing written", b.files.len());
            } else {
                b.write_to(&out, force).map_err(e)?;
                println!("wrote {} files to {}", b.files.len(), out.display());
            }
            Ok(())
        }
        Cmd::Check { plan, json } => {
            let (input, cfg, opts) = load(&plan).map_err(e)?;
            let b = plan2cell::build(&input, &cfg, &opts).map_err(e)?;
            if json {
                println!(
                    "{}",
                    serde_json::to_string_pretty(&b.summary).unwrap_or_default()
                );
            } else {
                report(&b);
                println!("ok: {} files generated and validated", b.files.len());
            }
            Ok(())
        }
        Cmd::Hardware { plan, json } => hardware(&plan, json),
        Cmd::Zones { plan } => {
            let (input, cfg, opts) = load(&plan).map_err(e)?;
            let (p, diags) = plan2cell::plan(&input, &cfg, &opts).map_err(e)?;
            for d in diags {
                eprintln!("{d}");
            }
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
            Ok(())
        }
        Cmd::Explain { plan, robot } => {
            let (input, cfg, mut opts) = load(&plan).map_err(e)?;
            opts.krl = true;
            opts.st_check = false;
            let b = plan2cell::build(&input, &cfg, &opts).map_err(e)?;
            print!("{}", plan2cell::explain_robot(&b, &robot).map_err(e)?);
            Ok(())
        }
        Cmd::Simulate {
            plan,
            cycles,
            robots,
        } => simulate(&plan, cycles, &robots),
        Cmd::Mcp {
            stdio,
            http,
            root,
            config,
            allow_write,
            token,
            allowed_host,
        } => {
            let settings = plan2cell_mcp::Settings {
                root,
                config,
                allow_write,
            };
            let rt = tokio::runtime::Runtime::new().map_err(|x| (4, x.to_string()))?;
            rt.block_on(async move {
                match (stdio, http) {
                    (_, Some(addr)) => {
                        let s = plan2cell_mcp::serve_http(
                            settings,
                            plan2cell_mcp::HttpOptions {
                                addr,
                                token,
                                allowed_hosts: allowed_host,
                            },
                        )
                        .await
                        .map_err(|x| (3, x.to_string()))?;
                        eprintln!(
                            "plan2cell MCP server on http://{}/mcp (health: /healthz){}; Ctrl-C to stop",
                            s.local_addr,
                            if s.local_addr.ip().is_loopback() {
                                ""
                            } else {
                                ", bearer token required"
                            }
                        );
                        s.run_until_ctrl_c().await;
                        Ok(())
                    }
                    (true, None) => plan2cell_mcp::serve_stdio(settings)
                        .await
                        .map_err(|x| (4, x.to_string())),
                    (false, None) => Err((2, "give --stdio or --http ADDR".into())),
                }
            })
        }
        Cmd::Cell { args, cmd } => cell(&args, cmd),
    }
}

#[cfg(feature = "cell")]
fn cell(a: &CellArgs, cmd: CellCmd) -> Result<(), (i32, String)> {
    use plan2cell_cell::stages::REPORT_FILE;
    use plan2cell_cell::{Cell, CellConfig, Report, Settings, Stage, Tree};

    let e = |e: plan2cell_cell::Error| (e.exit_code(), e.to_string());
    let pe = |e: plan2cell::Error| (e.exit_code(), e.to_string());
    let path = a
        .cell_config
        .clone()
        .unwrap_or_else(CellConfig::default_path);
    if let CellCmd::Check {
        dir,
        offline,
        export_only,
    } = &cmd
    {
        return check(a, &path, dir, *offline || a.dry_run, *export_only);
    }
    if let CellCmd::HooksCheck { .. } = &cmd {
        return hooks_check(a, cmd);
    }
    let cfg = if a.dry_run && !path.exists() {
        eprintln!(
            "note: {} not found; dry run with the defaults",
            path.display()
        );
        CellConfig::default()
    } else {
        CellConfig::load(&path).map_err(e)?
    };
    let mut settings = Settings {
        dry_run: a.dry_run,
        verify_timeout: std::time::Duration::from_secs(a.verify_timeout),
        ..Settings::default()
    };
    let (stages, tree) = match cmd {
        CellCmd::Up { plan, out, force } => {
            let (input, bcfg, opts) = load(&plan).map_err(pe)?;
            let b = plan2cell::build(&input, &bcfg, &opts).map_err(pe)?;
            if !a.json {
                report(&b);
            }
            let tree = if a.dry_run {
                Tree::Mem {
                    dir: out,
                    files: b.files.clone(),
                }
            } else {
                let again = out.join("plan2cell-build.json").is_file();
                b.write_to(&out, force || again).map_err(pe)?;
                eprintln!("wrote {} files to {}", b.files.len(), out.display());
                Tree::Dir(out)
            };
            (Stage::ALL.to_vec(), Some(tree))
        }
        CellCmd::Preflight { dir } => (vec![Stage::Preflight], dir.map(Tree::Dir)),
        CellCmd::Runtimes { dir } => (vec![Stage::Runtimes], Some(Tree::Dir(dir))),
        CellCmd::Projects { dir } => (vec![Stage::Projects], Some(Tree::Dir(dir))),
        CellCmd::Deploy { dir } => (vec![Stage::Deploy], Some(Tree::Dir(dir))),
        CellCmd::Krl { dir } => (vec![Stage::Krl], Some(Tree::Dir(dir))),
        CellCmd::Verify { dir } => (vec![Stage::Verify], Some(Tree::Dir(dir))),
        CellCmd::Start {
            dir,
            restart_runtime,
            timeout,
            pulses,
            login_timeout,
        } => {
            settings.restart_runtime = restart_runtime;
            settings.start_timeout = std::time::Duration::from_secs(timeout);
            settings.start_pulses = pulses;
            settings.login_timeout = std::time::Duration::from_secs(login_timeout);
            (vec![Stage::Start], Some(Tree::Dir(dir)))
        }
        CellCmd::Stop { dir } => (vec![Stage::Stop], Some(Tree::Dir(dir))),
        CellCmd::ResetIde { dir } => (vec![Stage::ResetIde], dir.map(Tree::Dir)),
        CellCmd::Runtime { dir } => return runtime(a, &cfg, dir.as_deref()),
        CellCmd::Kvc { robot, op, arg } => return kvc(cfg, settings, &robot, &op, arg.as_deref()),
        CellCmd::Check { .. } | CellCmd::HooksCheck { .. } => unreachable!("handled above"),
    };
    let report_path = match &tree {
        Some(Tree::Dir(d)) if !a.dry_run => Some(d.join(REPORT_FILE)),
        _ => None,
    };
    let previous = report_path.as_deref().and_then(Report::load);
    let mut remote = plc_remote(&cfg);
    let mut tools = plan2cell_cell::mcp::McpTools::new(cfg.codesys_mcp.clone()).map_err(e)?;
    let mut c = Cell::new(cfg, tree, settings, remote.as_mut(), &mut tools, previous).map_err(e)?;
    let r = c.run(&stages);
    let rep = c.report.clone();
    drop(c);
    tools.close();
    if let Some(p) = &report_path {
        rep.write(p).map_err(e)?;
    }
    if a.json {
        println!("{}", serde_json::to_string_pretty(&rep).unwrap_or_default());
    } else {
        if a.dry_run {
            println!("dry run: {} planned call(s)", rep.calls.len());
            for call in &rep.calls {
                println!("  {call}");
            }
        }
        for s in &rep.stages {
            println!(
                "{:<9} {}{}",
                s.stage.name(),
                if s.ok { "ok" } else { "FAILED" },
                s.error
                    .as_ref()
                    .map(|x| format!(": {x}"))
                    .or_else(|| (!s.notes.is_empty()).then(|| format!(" ({})", s.notes.join("; "))))
                    .unwrap_or_default()
            );
        }
        for c in &rep.controllers {
            for e in &c.errors {
                println!("{} on {}: {e}", c.robot_ns, c.instance);
            }
            for x in c.checks.iter().filter(|x| !x.ok) {
                println!(
                    "{} check {}: {}, want {}",
                    c.robot_ns,
                    x.expr,
                    x.value.as_deref().unwrap_or("-"),
                    x.expect
                );
            }
        }
        for p in rep.projects.iter().filter(|p| !p.human_steps.is_empty()) {
            for h in &p.human_steps {
                println!("human step for {}: {h}", p.name);
            }
        }
        if let Some(p) = &report_path {
            println!("report: {}", p.display());
        }
    }
    r.map_err(e)
}

/// The PLC host: over SSM with `plc_host.ssm_instance`, `kubectl exec` with `plc_host.kubectl_pod` (an EKS cell's PLC
/// pod; a bad target is a config error at load), else ssh.
#[cfg(feature = "cell")]
fn plc_remote(cfg: &plan2cell_cell::CellConfig) -> Box<dyn plan2cell_cell::remote::Remote> {
    if let Some(t) = &cfg.plc_host.kubectl_pod
        && let Ok(k) = plan2cell_cell::remote::Kubectl::parse(t, cfg.plc_host.kubeconfig.clone())
    {
        return Box::new(k);
    }
    match &cfg.plc_host.ssm_instance {
        Some(i) => {
            let mut s = plan2cell_cell::remote::Ssm::new(i.clone());
            s.region = cfg.plc_host.region.clone();
            s.profile = cfg.plc_host.profile.clone();
            Box::new(s)
        }
        None => Box::new(plan2cell_cell::remote::Ssh::new(
            cfg.plc_host.ssh_host.clone(),
        )),
    }
}

/// `plan2cell cell check DIR`: the deployability report of DIR on the cell of `path` (only the
/// export's rules without a cell.toml, or with `export_only`). Exit 7 when a rule fails.
#[cfg(feature = "cell")]
fn check(
    a: &CellArgs,
    path: &Path,
    dir: &Path,
    offline: bool,
    export_only: bool,
) -> Result<(), (i32, String)> {
    use plan2cell::deploy_check;

    if !dir.is_dir() {
        return Err((4, format!("{}: not an export directory", dir.display())));
    }
    let (target, note) = if export_only {
        (
            None,
            Some("no cell (--export-only): the export's rules only".to_string()),
        )
    } else if !path.exists() {
        (
            None,
            Some(format!(
                "no cell ({} not found): the export's rules only",
                path.display()
            )),
        )
    } else {
        let cfg =
            plan2cell_cell::CellConfig::load(path).map_err(|x| (x.exit_code(), x.to_string()))?;
        match check_target(&cfg, dir, offline) {
            Some(t) => (Some(t), None),
            None => (
                None,
                Some(format!(
                    "a cell without KUKA VCs ({} has no [[kuka_vc.controller]]): the export's rules only",
                    path.display()
                )),
            ),
        }
    };
    let mut rep = deploy_check::check(
        &deploy_check::Dir(dir.to_path_buf()),
        &dir.display().to_string(),
        target.as_ref(),
    );
    if let Some(n) = note {
        rep.target = Some(n);
    }
    if a.json {
        println!("{}", serde_json::to_string_pretty(&rep).unwrap_or_default());
    } else {
        print!("{}", rep.text());
    }
    if rep.ok {
        Ok(())
    } else {
        Err((7, format!("not deployable: {} rule(s) failed", rep.fails)))
    }
}

/// The cell of `cfg` as the deployability check sees it: its KUKA VCs (their `$config.dat` read
/// in parallel, read-only, unless `offline`) and its PROFINET segment.
#[cfg(feature = "cell")]
fn check_target(
    cfg: &plan2cell_cell::CellConfig,
    dir: &Path,
    offline: bool,
) -> Option<plan2cell::deploy_check::Target> {
    use plan2cell::deploy_check::{
        self, Controller, Globals, Target, config_dat_command, parse_config_dat_output,
    };

    let k = cfg.kuka_vc.as_ref().filter(|k| !k.controllers.is_empty())?;
    // Only the controllers of the export's robots are read.
    let robots: Vec<String> = std::fs::read_to_string(dir.join("plan2cell-build.json"))
        .ok()
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
        .and_then(|v| {
            v["robots"].as_array().map(|r| {
                r.iter()
                    .filter_map(|x| x["ns"].as_str().map(str::to_owned))
                    .collect()
            })
        })
        .unwrap_or_default();
    let gunzip = |gz: &[u8]| -> Option<String> {
        use std::io::Write as _;
        let mut child = std::process::Command::new("gzip")
            .arg("-dc")
            .stdin(std::process::Stdio::piped())
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::null())
            .spawn()
            .ok()?;
        child.stdin.take()?.write_all(gz).ok()?;
        let out = child.wait_with_output().ok()?;
        out.status
            .success()
            .then(|| String::from_utf8_lossy(&out.stdout).into_owned())
    };
    let read = |c: &plan2cell_cell::config::KukaController| -> Globals {
        if c.is_hook() {
            // factoryos/hook@1 `globals` (check.hook): its names replace the $config.dat read.
            use plan2cell_cell::hook::{self, Role, Verb};
            let source = format!("hook on {}", c.access());
            let answer = plan2cell_cell::remote::Kubectl::parse(c.kubectl_pod.as_deref().unwrap_or(""), k.kubeconfig.clone())
                .and_then(|mut r| hook::described_call(&mut r, Role::Robot, Verb::Globals, &serde_json::json!({}), hook::RETRY_WAIT));
            return match answer {
                Ok(a) => deploy_check::hook_globals(&source, &a.json, a.exit, a.problem.as_deref()),
                Err(e) => Globals {
                    source,
                    names: Default::default(),
                    read: false,
                    error: Some(e.to_string()),
                },
            };
        }
        let source = format!("$config.dat of {} on {}", c.instance, c.access());
        let cmd = config_dat_command(&c.instance);
        let out = match c.access() {
            plan2cell_cell::config::Access::Ssm(i) => {
                let mut s = plan2cell_cell::remote::Ssm::new(i);
                s.region = k.region.clone();
                s.profile = k.profile.clone();
                s.timeout = std::time::Duration::from_secs(30);
                s.poll = std::time::Duration::from_millis(700);
                plan2cell_cell::remote::Remote::run(&mut s, &cmd)
            }
            plan2cell_cell::config::Access::Ssh(h) => {
                plan2cell_cell::remote::Remote::run(&mut plan2cell_cell::remote::Ssh::new(h), &cmd)
            }
            plan2cell_cell::config::Access::Kubectl(t) => plan2cell_cell::remote::Kubectl::parse(&t, k.kubeconfig.clone())
                .and_then(|mut r| plan2cell_cell::remote::Remote::run(&mut r, &cmd)),
        };
        match out
            .map_err(|e| e.to_string())
            .and_then(|o| parse_config_dat_output(&o, gunzip))
        {
            Ok(names) => Globals {
                source,
                names,
                read: true,
                error: None,
            },
            Err(error) => Globals {
                source,
                names: Default::default(),
                read: false,
                error: Some(error),
            },
        }
    };
    let controllers: Vec<Controller> = std::thread::scope(|s| {
        let handles: Vec<_> = k
            .controllers
            .iter()
            .map(|c| {
                let wanted = !offline && (robots.is_empty() || robots.contains(&c.robot_ns));
                let read = &read;
                (c, wanted.then(|| s.spawn(move || read(c))))
            })
            .collect();
        handles
            .into_iter()
            .map(|(c, h)| Controller {
                robot_ns: c.robot_ns.clone(),
                kli_ip: c.kli_ip.clone(),
                station: c.station.clone(),
                host: format!("{} on {}", c.label(), c.access()),
                globals: h.and_then(|h| h.join().ok()),
            })
            .collect()
    });
    let mut t = Target::virtual_cell(controllers);
    if let Some(p) = &cfg.profinet {
        if let Some(v) = &p.head_ip {
            t.head_ip = v.clone();
        }
        if let Some(v) = &p.netmask {
            t.netmask = v.clone();
        }
        if let Some(v) = &p.gateway {
            t.gateway = v.clone();
        }
    }
    Some(t)
}

/// `plan2cell cell runtime [DIR]`: one read-only remote command; with `--json` the report (also
/// when the host could not be read: `ok: false`, `error`, exit 4).
/// `plan2cell cell kvc`: one allowed operation on one KUKA VC controller, printed as JSON; exit 1
/// when it failed (the JSON says why).
#[cfg(feature = "cell")]
fn kvc(cfg: plan2cell_cell::CellConfig, settings: plan2cell_cell::Settings, robot: &str, op: &str, arg: Option<&str>) -> Result<(), (i32, String)> {
    let e = |e: plan2cell_cell::Error| (e.exit_code(), e.to_string());
    let op = plan2cell_cell::KvcOp::parse(op, arg).map_err(e)?;
    let mut remote = plc_remote(&cfg);
    let mut tools = plan2cell_cell::mcp::McpTools::new(cfg.codesys_mcp.clone()).map_err(e)?;
    let mut c = plan2cell_cell::Cell::new(cfg, None, settings, remote.as_mut(), &mut tools, None).map_err(e)?;
    let out = c.kvc_op(robot, &op);
    drop(c);
    tools.close();
    let v = match out {
        Ok(v) => v,
        Err(err) => serde_json::json!({"ok": false, "robot_ns": robot, "error": err.to_string()}),
    };
    println!("{}", serde_json::to_string_pretty(&v).unwrap_or_default());
    if v["ok"] == true { Ok(()) } else { Err((1, format!("kvc {robot}: failed (see the JSON)"))) }
}

#[cfg(feature = "cell")]
fn runtime(
    a: &CellArgs,
    cfg: &plan2cell_cell::CellConfig,
    dir: Option<&Path>,
) -> Result<(), (i32, String)> {
    use plan2cell_cell::runtime;

    let e = |e: plan2cell_cell::Error| (e.exit_code(), e.to_string());
    let manifest = match dir {
        Some(d) => {
            let p = d.join("plc/project.json");
            let text =
                std::fs::read_to_string(&p).map_err(|x| e(plan2cell_cell::Error::io(&p, x)))?;
            Some(plan2cell_cell::project::Manifest::parse(&text).map_err(e)?)
        }
        None => None,
    };
    let containers = runtime::containers(manifest.as_ref());
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs() as i64);
    let hook = cfg.plc_host.is_hook();
    let result = if a.dry_run {
        Ok(runtime::RuntimeReport {
            schema: runtime::RUNTIME_SCHEMA.into(),
            ok: true,
            command: Some(if hook {
                format!(
                    "kubectl:{}: hook health {{}}",
                    cfg.plc_host.kubectl_pod.as_deref().unwrap_or("?")
                )
            } else {
                runtime::command(containers.as_deref())
            }),
            ..Default::default()
        })
    } else if hook {
        // runtime.hook: the head's own cell-runtime@1 (no demo period, no auto-restart).
        runtime::read_hook(plc_remote(cfg).as_mut(), plan2cell_cell::hook::RETRY_WAIT)
    } else {
        runtime::read(plc_remote(cfg).as_mut(), containers.as_deref(), now)
    };
    let (rep, err) = match result {
        Ok(r) => (r, None),
        Err(x) => (
            runtime::RuntimeReport {
                schema: runtime::RUNTIME_SCHEMA.into(),
                error: Some(x.to_string()),
                ..Default::default()
            },
            Some(e(x)),
        ),
    };
    if a.json {
        println!("{}", serde_json::to_string_pretty(&rep).unwrap_or_default());
    } else if let Some(c) = &rep.command {
        if hook {
            println!("dry run: 1 planned call\n  {c}");
        } else {
            println!(
                "dry run: 1 planned call\n  ssh {}: {c}",
                cfg.plc_host.ssh_host
            );
        }
    } else {
        for l in runtime::text(&rep) {
            println!("{l}");
        }
    }
    match err {
        Some(x) => Err(x),
        None => Ok(()),
    }
}

/// `plan2cell cell hooks-check`: the conformance kit of factoryos/hook@1
/// (plan2cell_cell::hooks_check). Exit 0 conformant, 7 a check failed, 2 usage, 4 unreachable.
#[cfg(feature = "cell")]
fn hooks_check(a: &CellArgs, cmd: CellCmd) -> Result<(), (i32, String)> {
    use plan2cell_cell::hook::Role;
    use plan2cell_cell::hooks_check::{self, DockerRun, DockerSpec, Mode, Options, Profile};

    let CellCmd::HooksCheck { exec, docker, kubeconfig, mutating, role, profile, pn_address, strict_udp, ops, env, args } = cmd else {
        unreachable!("called for hooks-check only")
    };
    let usage = |m: String| (2, m);
    let role = role
        .as_deref()
        .map(|r| Role::parse(r).ok_or_else(|| usage(format!("--role {r}: head or robot"))))
        .transpose()?;
    let profile = profile
        .as_deref()
        .map(|p| Profile::parse(p).ok_or_else(|| usage(format!("--profile {p}: none, pn-raw or pn-admin"))))
        .transpose()?;
    let template_ops = match &ops {
        Some(p) => {
            let t: serde_json::Value = std::fs::read_to_string(p)
                .map_err(|e| e.to_string())
                .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
                .map_err(|e| usage(format!("--ops {}: {e}", p.display())))?;
            t["consumer"]["ops"]
                .as_array()
                .map(|o| o.iter().filter_map(|x| x["name"].as_str().map(str::to_string)).collect())
                .ok_or_else(|| usage(format!("--ops {}: no consumer.ops", p.display())))?
        }
        None => Vec::new(),
    };
    for e in &env {
        let ok = e.split_once('=').is_some_and(|(n, _)| {
            !n.is_empty() && n.chars().all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_')
        });
        if !ok {
            return Err(usage(format!("--env {e}: want NAME=VALUE")));
        }
    }
    let pn: Vec<String> = pn_address.iter().map(|p| p.split('/').next().unwrap_or("").to_string()).collect();
    let report = if let Some(target) = exec {
        let mut remote = plan2cell_cell::remote::Kubectl::parse(&target, kubeconfig.clone())
            .map_err(|e| usage(e.to_string()))?;
        // The profile and the PN addresses from the pod itself (read-only), unless given.
        let pod: Option<serde_json::Value> = (profile.is_none() || pn.is_empty())
            .then(|| {
                let mut c = std::process::Command::new("kubectl");
                if let Some(k) = &kubeconfig {
                    c.args(["--kubeconfig", k]);
                }
                c.args(["get", "pod", "-n", &remote.namespace, &remote.pod, "-o", "json"]);
                c.output().ok().filter(|o| o.status.success()).and_then(|o| serde_json::from_slice(&o.stdout).ok())
            })
            .flatten();
        let mut o = Options::new(Mode::Exec);
        o.role = role;
        o.profile = profile.or_else(|| pod.as_ref().and_then(|p| hooks_check::profile_from_pod(p, &remote.container)));
        o.pn_addresses = if pn.is_empty() {
            pod.as_ref().map(|p| hooks_check::pn_addresses_from_pod(p, "net1")).unwrap_or_default()
        } else {
            pn
        };
        o.strict_udp = strict_udp;
        o.ops = template_ops;
        o.mutating = mutating;
        hooks_check::run(&mut remote, &target, &o, None)
    } else {
        let image = docker.expect("clap: --exec or --docker");
        let unreachable = |e: plan2cell_cell::Error| (4, e.to_string());
        DockerRun::check_version().map_err(unreachable)?;
        let described = DockerRun::describe(&image).map_err(unreachable)?;
        let desc = plan2cell_cell::hook::parse(plan2cell_cell::hook::Verb::Describe, &described);
        let role = role
            .or_else(|| desc.json["role"].as_str().and_then(Role::parse))
            .ok_or_else(|| usage(format!("{image}: describe names no role ({}); give --role", desc.error())))?;
        let profile = profile.unwrap_or(Profile::None);
        let spec = DockerSpec { image: image.clone(), role, profile, env, args };
        let run = DockerRun::start(&spec).map_err(unreachable)?;
        let mut remote = run.remote();
        let mut o = Options::new(Mode::Docker);
        o.role = Some(role);
        o.profile = Some(profile);
        o.pn_addresses = if pn.is_empty() { vec![hooks_check::docker_ip(role).to_string()] } else { pn };
        o.strict_udp = strict_udp;
        o.ops = template_ops;
        let r = hooks_check::run(&mut remote, &image, &o, Some(described));
        drop(run);
        r
    };
    if a.json {
        println!("{}", serde_json::to_string_pretty(&report).unwrap_or_default());
    } else {
        print!("{}", report.text());
    }
    match report.exit_code() {
        0 => Ok(()),
        4 => Err((4, format!("{}: the target did not answer", report.target))),
        n => Err((n, format!("{}: not conformant ({} check(s) failed)", report.target, report.checks.iter().filter(|c| !c.ok).count()))),
    }
}

#[cfg(not(feature = "cell"))]
fn cell(_a: &CellArgs, _cmd: CellCmd) -> Result<(), (i32, String)> {
    Err((
        5,
        "plan2cell was built without the cell orchestrator: cargo build -p plan2cell-cli --features cell"
            .into(),
    ))
}

#[cfg(feature = "sim")]
fn simulate(plan: &PlanArgs, cycles: u32, robots: &str) -> Result<(), (i32, String)> {
    let e = |e: plan2cell::Error| (e.exit_code(), e.to_string());
    let (input, cfg, mut opts) = load(plan).map_err(e)?;
    opts.st_check = false;
    let robots = if robots == "device-st" {
        plan2cell::sim::Robots::DeviceSt
    } else {
        opts.krl = true;
        plan2cell::sim::Robots::Krl
    };
    let b = plan2cell::build(&input, &cfg, &opts).map_err(e)?;
    let (_, r) = plan2cell::sim::simulate(&b, &cfg, robots, cycles.max(1)).map_err(e)?;
    println!("{}", r.summary);
    if r.ok {
        Ok(())
    } else {
        for l in &r.tail {
            println!("{l}");
        }
        Err((7, r.errors.join("\n")))
    }
}

#[cfg(not(feature = "sim"))]
fn simulate(_plan: &PlanArgs, _cycles: u32, _robots: &str) -> Result<(), (i32, String)> {
    Err((
        5,
        "plan2cell was built without co-simulation: cargo build -p plan2cell-cli --features sim"
            .into(),
    ))
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err((code, msg)) => {
            eprintln!("error: {msg}");
            ExitCode::from(code as u8)
        }
    }
}
