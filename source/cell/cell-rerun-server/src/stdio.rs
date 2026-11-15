//! NDJSON stdin/stdout control channel — the same wire protocol the
//! robo-fitting-room `stdio-serve` transport uses. factoryos-backend's
//! `services/cell.rs` is the client.
//!
//! Framing: exactly one JSON object per line.
//!   in :  {"id": <u64>, "method": "<name>", "params": <any>}
//!   out:  {"id": <u64>, "result": <any>}   |   {"id": <u64|null>, "error": {code, message}}
//!
//! Methods:
//!   health.check          -> {status, backendsEmbedded, running}
//!   start   {StartParams} -> RunningInfo (idempotent)
//!   status                -> status snapshot
//!   stop                  -> {stopped: true}
//!
//! The loop reads until stdin closes (EOF) — at which point the parent process
//! is gone, so the cell-runner is stopped and the process exits 0.

use std::io::{BufRead, BufReader, Write};

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::supervisor::{backends_embedded, StartParams, Supervisor};

#[derive(Deserialize)]
struct Request {
    id: u64,
    method: String,
    #[serde(default)]
    params: Value,
}

#[derive(Serialize)]
struct ErrorObj {
    code: i32,
    message: String,
}

/// JSON-RPC-style reserved codes (parse / unknown method / bad params) plus a
/// generic application failure code.
const CODE_PARSE_ERROR: i32 = -32700;
const CODE_METHOD_NOT_FOUND: i32 = -32601;
const CODE_INVALID_PARAMS: i32 = -32602;
const CODE_APP_ERROR: i32 = 500;

fn write_result<W: Write>(w: &mut W, id: u64, result: Value) -> std::io::Result<()> {
    let line = serde_json::json!({ "id": id, "result": result });
    writeln!(w, "{}", line)?;
    w.flush()
}

fn write_error<W: Write>(w: &mut W, id: Option<u64>, code: i32, message: String) -> std::io::Result<()> {
    let line = serde_json::json!({ "id": id, "error": ErrorObj { code, message } });
    writeln!(w, "{}", line)?;
    w.flush()
}

/// Run the NDJSON control loop against a fresh {@link Supervisor}. Blocks until
/// stdin EOF, then stops the cell-runner and returns.
pub fn run() -> anyhow::Result<()> {
    let mut supervisor = Supervisor::new();

    let stdin = std::io::stdin();
    let mut reader = BufReader::new(stdin.lock());
    let stdout = std::io::stdout();
    let mut writer = stdout.lock();

    let mut line = String::new();
    loop {
        line.clear();
        if reader.read_line(&mut line)? == 0 {
            // stdin closed — parent gone; stop the cell-runner and exit cleanly.
            supervisor.stop();
            return Ok(());
        }
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }

        let req: Request = match serde_json::from_str(trimmed) {
            Ok(r) => r,
            Err(e) => {
                write_error(&mut writer, None, CODE_PARSE_ERROR, format!("parse error: {e}"))?;
                continue;
            }
        };

        match req.method.as_str() {
            "health.check" => {
                write_result(
                    &mut writer,
                    req.id,
                    serde_json::json!({
                        "status": "ok",
                        "backendsEmbedded": backends_embedded(),
                        "running": supervisor.is_running(),
                    }),
                )?;
            }
            "status" => {
                write_result(&mut writer, req.id, supervisor.status())?;
            }
            "start" => {
                let params: StartParams = match serde_json::from_value(req.params.clone()) {
                    Ok(p) => p,
                    Err(e) => {
                        write_error(
                            &mut writer,
                            Some(req.id),
                            CODE_INVALID_PARAMS,
                            format!("invalid start params: {e}"),
                        )?;
                        continue;
                    }
                };
                match supervisor.start(&params) {
                    Ok(info) => write_result(&mut writer, req.id, info.to_json())?,
                    Err(e) => {
                        write_error(&mut writer, Some(req.id), CODE_APP_ERROR, format!("{e:#}"))?
                    }
                }
            }
            "stop" => {
                supervisor.stop();
                write_result(&mut writer, req.id, serde_json::json!({ "stopped": true }))?;
            }
            other => {
                write_error(
                    &mut writer,
                    Some(req.id),
                    CODE_METHOD_NOT_FOUND,
                    format!("unknown method: {other}"),
                )?;
            }
        }
    }
}
