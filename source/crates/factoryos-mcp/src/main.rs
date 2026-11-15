//! `factoryos-mcp`: the Physical AI Production Designer workflow as a stdio MCP server, for the workspace's kiro-cli.
//!
//! ```text
//! factoryos-mcp [--backend http://127.0.0.1:80] [--timeout-s 900] [--artifact-dir DIR]
//! factoryos-mcp --list          # the tools, one per line (name, read-only / destructive)
//! factoryos-mcp --openapi       # the OpenAPI document the tools are made from
//! ```
//!
//! kiro-cli starts it per chat session (`deploy/kiro/factoryos.json`); it ends when stdin closes.
//! `FACTORYOS_BACKEND_URL` is the default `--backend`. Logs go to stderr (`RUST_LOG`).

#![forbid(unsafe_code)]

mod proxy;
mod spec;

use std::path::PathBuf;
use std::time::Duration;

const INSTRUCTIONS: &str = "\
Physical AI Production Designer on this workspace: every tool is a route of its backend, acting on the user's products, catalogue, \
scenes, cells, plans, the simulated cell and the virtual cell. The workflow, in order:
1. Product & Task (product_*): the part and its welds, in the product's frame (metres, z up). Add, move, \
regroup, remove or generate welds; snap points onto the surface; product_check_welds before planning.
2. Robo Dresser (dresser_*): a robot dressed with a tool (spot welding gun). dresser_orient_sweep tries tool \
mount orientations against the welds and ranks them by reach; dresser_dress writes the dressed robot.
3. Robo Fitter (scene_*): the cell's scene. scene_place_product (height auto adds a support), \
scene_auto_place_robots (1..N robots to cover the welds), scene_reach_check, scene_create_cell.
4. Planning: through the factoryos-planner-router tools (recommend_planner, plan, plan_status); cell_* here \
for the cell's plans and task programs.
5. Behaviour tree (bt_*, trajectory_*) in the simulated cell, and the 3D view (view_*).
6. Control code (codegen_*, process_*): KRL / PLC exports; the process definition holds the IOs.
7. Virtual cell (vcell_*): the workspace's virtual cell, booted from a template (vcell_list_templates: its \
parameters, robot operations and notes; the legacy templates are a CODESYS head PLC and KUKA VCs). Deploying, \
production and controller operations change a running cell: only with the user's explicit go.
Read tools are safe to call any time. Before a write, say what will change; destructive tools always ask.";

#[derive(Debug)]
struct Args {
    backend: String,
    timeout: Duration,
    artifact_dir: Option<PathBuf>,
    list: bool,
    openapi: bool,
}

fn args() -> Result<Args, String> {
    let mut args = Args {
        backend: std::env::var("FACTORYOS_BACKEND_URL").unwrap_or_else(|_| "http://127.0.0.1:80".to_owned()),
        timeout: Duration::from_secs(900),
        artifact_dir: None,
        list: false,
        openapi: false,
    };
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        let mut value = |name: &str| it.next().ok_or_else(|| format!("{name} needs a value"));
        match arg.as_str() {
            "--backend" => args.backend = value("--backend")?,
            "--timeout-s" => args.timeout = Duration::from_secs(value("--timeout-s")?.parse().map_err(|e| format!("--timeout-s: {e}"))?),
            "--artifact-dir" => args.artifact_dir = Some(PathBuf::from(value("--artifact-dir")?)),
            "--list" => args.list = true,
            "--openapi" => args.openapi = true,
            "-h" | "--help" => {
                println!("factoryos-mcp [--backend URL] [--timeout-s N] [--artifact-dir DIR] [--list] [--openapi]");
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument {other:?} (see --help)")),
        }
    }
    Ok(args)
}

fn facade(args: &Args) -> Result<openapi_mcp::Facade<proxy::Proxy>, String> {
    let tools = spec::tools()?;
    let openapi = spec::openapi(&tools)?;
    let mut options = openapi_mcp::Options::new(env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));
    options.instructions = Some(INSTRUCTIONS.to_owned());
    options.artifact_dir = args.artifact_dir.clone();
    openapi_mcp::Facade::new(proxy::Proxy::new(&args.backend, args.timeout), &openapi, options).map_err(|e| e.to_string())
}

#[tokio::main]
async fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).target(env_logger::Target::Stderr).init();
    let args = match args() {
        Ok(args) => args,
        Err(e) => {
            eprintln!("factoryos-mcp: {e}");
            std::process::exit(2);
        }
    };
    if args.openapi {
        let doc = spec::tools().and_then(|t| spec::openapi(&t));
        match doc {
            Ok(doc) => println!("{}", serde_json::to_string_pretty(&doc).expect("json")),
            Err(e) => {
                eprintln!("factoryos-mcp: {e}");
                std::process::exit(1);
            }
        }
        return;
    }
    let facade = match facade(&args) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("factoryos-mcp: {e}");
            std::process::exit(1);
        }
    };
    if args.list {
        for tool in facade.tools() {
            let a = tool.annotations.as_ref();
            let kind = if a.and_then(|a| a.read_only_hint) == Some(true) {
                "read"
            } else if a.and_then(|a| a.destructive_hint) == Some(true) {
                "destructive"
            } else {
                "write"
            };
            println!("{}\t{kind}", tool.name);
        }
        return;
    }
    log::info!("{} tools, backend {}", facade.tools().len(), args.backend);
    if let Err(e) = facade.serve_stdio().await {
        eprintln!("factoryos-mcp: {e}");
        std::process::exit(1);
    }
}
