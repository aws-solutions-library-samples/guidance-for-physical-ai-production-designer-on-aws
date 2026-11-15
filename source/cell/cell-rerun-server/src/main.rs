//! cell-rerun-server — a single self-contained supervisor binary that embeds
//! the cell-runner backend (and its shared libraries) and drives it over an
//! NDJSON stdin/stdout control channel, for factoryos-backend's simulated cell
//! (`services/cell.rs`).
//!
//! Subcommands:
//!   stdio-serve            Long-running NDJSON control loop over stdin/stdout
//!                          (the embedding mode; see `stdio.rs`).
//!   run [flags]            Foreground mode for manual testing: start the
//!                          cell-runner and block until Ctrl-C / stdin EOF.
//!                          Flags: --cell <dir> --env <dir> [--scene <name>]
//!                                 [--cell-port N]
//!   --version | -V
//!   --help    | -h

mod stdio;
mod supervisor;

use std::process;

use supervisor::{backends_embedded, StartParams, Supervisor};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cmd = args.first().map(String::as_str);

    match cmd {
        Some("stdio-serve") => {
            if let Err(e) = stdio::run() {
                eprintln!("cell-rerun-server: stdio loop failed: {e:#}");
                process::exit(1);
            }
        }
        Some("run") => {
            if let Err(e) = run_foreground(&args[1..]) {
                eprintln!("cell-rerun-server: {e:#}");
                process::exit(1);
            }
        }
        Some("--version") | Some("-V") => {
            println!("cell-rerun-server {}", env!("CARGO_PKG_VERSION"));
        }
        Some("--help") | Some("-h") | None => {
            print_usage();
        }
        Some(other) => {
            eprintln!("unknown command: {other}\n");
            print_usage();
            process::exit(2);
        }
    }
}

fn print_usage() {
    eprintln!(
        "cell-rerun-server {} — embedded cell-runner supervisor\n\
         backends embedded: {}\n\n\
         USAGE:\n  \
         cell-rerun-server stdio-serve            NDJSON control loop over stdin/stdout\n  \
         cell-rerun-server run --cell <dir> --env <dir> [--scene <name>] [--cell-port 9902]\n  \
         cell-rerun-server --version\n",
        env!("CARGO_PKG_VERSION"),
        backends_embedded()
    );
}

/// Foreground mode: parse flags, start the backends, then block until stdin EOF
/// (Ctrl-D) or the process is signalled. Handy for manual testing outside the
/// backend.
fn run_foreground(args: &[String]) -> anyhow::Result<()> {
    let params = parse_run_flags(args)?;
    let mut supervisor = Supervisor::new();
    let info = supervisor.start(&params)?;
    eprintln!(
        "cell-rerun-server: cell-runner up.\n  controller: {}\n  static /env: {}",
        format_args!("ws://127.0.0.1:{}/ws", info.cell_port),
        format_args!("http://127.0.0.1:{}", info.cell_port),
    );
    eprintln!("Press Ctrl-D to stop.");

    // Block until stdin closes.
    let mut sink = String::new();
    use std::io::Read;
    let _ = std::io::stdin().read_to_string(&mut sink);
    supervisor.stop();
    Ok(())
}

fn parse_run_flags(args: &[String]) -> anyhow::Result<StartParams> {
    // Build a JSON object from the flags and deserialize via StartParams so the
    // defaults (cell port) are applied consistently with the stdio path.
    let mut cell_dir: Option<String> = None;
    let mut env_dir: Option<String> = None;
    let mut obj = serde_json::Map::new();

    let mut i = 0;
    while i < args.len() {
        let flag = args[i].as_str();
        let take = |name: &str| -> anyhow::Result<String> {
            let v = args
                .get(i + 1)
                .ok_or_else(|| anyhow::anyhow!("missing value for {name}"))?;
            Ok(v.clone())
        };
        match flag {
            "--cell" => cell_dir = Some(take("--cell")?),
            "--env" => env_dir = Some(take("--env")?),
            "--scene" => {
                obj.insert("scene".into(), take("--scene")?.into());
            }
            "--cell-port" => {
                obj.insert("cellPort".into(), take("--cell-port")?.parse::<u16>()?.into());
            }
            other => anyhow::bail!("unknown flag: {other}"),
        }
        i += 2;
    }

    obj.insert(
        "cellDir".into(),
        cell_dir.ok_or_else(|| anyhow::anyhow!("--cell <dir> is required"))?.into(),
    );
    obj.insert(
        "envDir".into(),
        env_dir.ok_or_else(|| anyhow::anyhow!("--env <dir> is required"))?.into(),
    );

    Ok(serde_json::from_value(serde_json::Value::Object(obj))?)
}

