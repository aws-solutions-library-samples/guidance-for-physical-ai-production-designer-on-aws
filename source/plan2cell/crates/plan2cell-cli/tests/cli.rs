//! The `plan2cell` binary: commands, options, exit codes, and the MCP stdio transport.

use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use serde_json::{Value, json};

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn plan2cell(args: &[&str]) -> (i32, String, String) {
    let o = Command::new(env!("CARGO_BIN_EXE_plan2cell"))
        .args(args)
        .current_dir(repo())
        .output()
        .unwrap();
    (
        o.status.code().unwrap_or(-1),
        String::from_utf8_lossy(&o.stdout).into_owned(),
        String::from_utf8_lossy(&o.stderr).into_owned(),
    )
}

const WELDING: &str = "test_data/bt_1790934549_welding_cell_2_small.json";
const CFG: &str = "examples/plan2cell.toml";

#[test]
fn build_with_options() {
    let d = tempfile::tempdir().unwrap();
    let out = d.path().join("w");
    let o = out.to_str().unwrap();
    let (c, s, e) = plan2cell(&["build", WELDING, "-c", CFG, "-o", o, "--no-check"]);
    assert_eq!(c, 0, "{s}{e}");
    assert!(s.contains("IL01 = robot1.W01"), "{s}");
    assert!(out.join("plc/codesys/head/POU/FB_Cell_Body.st").is_file());
    assert!(out.join("krl/robot2/KRC/R1/Folgen/folge11.src").is_file());
    assert!(out.join("BUILD.md").is_file());
    // Non-empty directory: I/O (4) without --force.
    assert_eq!(
        plan2cell(&["build", WELDING, "-c", CFG, "-o", o, "--no-check"]).0,
        4
    );
    // PLC only, no KRL: the krl tree of the previous build is removed.
    let (c, _, e) = plan2cell(&[
        "build",
        WELDING,
        "-c",
        CFG,
        "-o",
        o,
        "--no-check",
        "--no-krl",
        "--force",
    ]);
    assert_eq!(c, 0, "{e}");
    assert!(!out.join("krl").join("robot1").join("KRC").exists());
    // KRL only.
    let k = d.path().join("k");
    let (c, _, e) = plan2cell(&[
        "build",
        WELDING,
        "-c",
        CFG,
        "-o",
        k.to_str().unwrap(),
        "--plc",
        "none",
        "--no-check",
    ]);
    assert_eq!(c, 0, "{e}");
    assert!(!k.join("plc/codesys").exists());
    assert!(k.join("plc/robots/robot1/program.json").is_file());
    assert!(k.join("krl/robot1/KRC/R1/Folgen/folge11.src").is_file());
    // Dry run writes nothing.
    let n = d.path().join("n");
    let (c, s, _) = plan2cell(&[
        "build",
        WELDING,
        "-c",
        CFG,
        "-o",
        n.to_str().unwrap(),
        "--dry-run",
        "--no-check",
    ]);
    assert_eq!(c, 0);
    assert!(s.contains("nothing written") && !n.exists());
}

#[test]
fn check_zones_explain_config_and_errors() {
    let (c, s, e) = plan2cell(&["check", WELDING, "-c", CFG, "--no-check", "--json"]);
    assert_eq!(c, 0, "{e}");
    let v: Value = serde_json::from_str(&s).unwrap();
    assert_eq!(v["schema"], "plan2cell/build@1");
    let (c, s, _) = plan2cell(&["zones", WELDING]);
    assert_eq!(c, 0);
    assert!(s.contains("robot1: W01[IL01] W02[IL02]"), "{s}");
    let (c, s, _) = plan2cell(&["explain", WELDING, "-c", CFG, "--robot", "robot1"]);
    assert_eq!(c, 0);
    assert!(s.contains("op   8 HOME"), "{s}");
    let (c, s, _) = plan2cell(&["config"]);
    assert_eq!(c, 0);
    assert!(s.contains("[plc.cell]") && s.contains("[krl.motion]"));
    // KRL without e_axes: config (3). Bad flag: usage (2). Missing file: I/O (4).
    assert_eq!(plan2cell(&["check", WELDING, "--no-check"]).0, 3);
    assert_eq!(plan2cell(&["check", WELDING, "--nope"]).0, 2);
    assert_eq!(plan2cell(&["check", "nope.json"]).0, 4);
    assert_eq!(plan2cell(&["mcp"]).0, 2);
}

#[test]
fn mcp_over_stdio() {
    let mut child = Command::new(env!("CARGO_BIN_EXE_plan2cell"))
        .args(["mcp", "--stdio", "--root", ".", "-c", CFG])
        .current_dir(repo())
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();
    let mut stdin = child.stdin.take().unwrap();
    let mut lines = BufReader::new(child.stdout.take().unwrap()).lines();
    let mut send = |v: Value| {
        writeln!(stdin, "{v}").unwrap();
        stdin.flush().unwrap();
    };
    let mut recv = |id: i64| -> Value {
        loop {
            let l = lines.next().expect("server closed stdout").unwrap();
            let v: Value = serde_json::from_str(&l).unwrap();
            if v["id"] == id {
                return v;
            }
        }
    };
    send(
        json!({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
        "protocolVersion": "2025-06-18", "capabilities": {},
        "clientInfo": {"name": "cli-test", "version": "1"}}}),
    );
    let init = recv(1);
    assert!(init["result"]["serverInfo"].is_object(), "{init}");
    send(json!({"jsonrpc": "2.0", "method": "notifications/initialized"}));
    send(json!({"jsonrpc": "2.0", "id": 2, "method": "tools/list"}));
    let tools = recv(2);
    let names: Vec<&str> = tools["result"]["tools"]
        .as_array()
        .unwrap()
        .iter()
        .map(|t| t["name"].as_str().unwrap())
        .collect();
    assert!(names.contains(&"plan2cell_build"), "{names:?}");
    send(
        json!({"jsonrpc": "2.0", "id": 3, "method": "tools/call", "params": {
        "name": "plan2cell_zones", "arguments": {"plan_path": WELDING}}}),
    );
    let z = recv(3);
    let text = z["result"]["content"][0]["text"].as_str().unwrap();
    let v: Value = serde_json::from_str(text).unwrap();
    assert_eq!(v["zones"][1]["name"], "IL02");
    let _ = child.kill();
    let _ = child.wait();
}

#[cfg(not(feature = "cell"))]
#[test]
fn cell_needs_the_feature() {
    assert_eq!(plan2cell(&["cell", "preflight", "--dry-run"]).0, 5);
}

#[cfg(feature = "cell")]
#[test]
fn cell_up_dry_run() {
    let d = tempfile::tempdir().unwrap();
    let out = d.path().join("w");
    let none = d.path().join("cell.toml");
    let (o, n) = (out.to_str().unwrap(), none.to_str().unwrap());
    let (c, s, e) = plan2cell(&[
        "cell",
        "up",
        WELDING,
        "-c",
        CFG,
        "-o",
        o,
        "--no-check",
        "--dry-run",
        "--cell-config",
        n,
    ]);
    assert_eq!(c, 0, "{s}{e}");
    assert!(s.contains("planned call(s)"), "{s}");
    // The typed CODESYS MCP tools, robot simulators first, the head PLC last (after the
    // logouts, head first).
    let at = |t: &str| s.find(t).unwrap_or_else(|| panic!("{t} not planned: {s}"));
    assert!(at("create_project(filePath=") < at("compile_project("));
    assert!(
        at("disconnect_from_device(") < at("create_project(filePath="),
        "logouts first: {s}"
    );
    let created = |p: &str| {
        s.lines()
            .position(|l| l.contains("create_project(") && l.contains(p))
            .unwrap_or_else(|| panic!("{p} not created: {s}"))
    };
    assert!(
        created("RobotSim_welding_cell_2_small_R1.project")
            < created("HeadPLC_welding_cell_2_small.project")
    );
    assert!(at("compile_project(") < at("download_to_device("));
    assert!(at("download_to_device(") < at("read_variables("));
    assert!(
        s.contains("--robots-overlay /opt/plan2cell/plc-host/docker-compose.robots.yml"),
        "{s}"
    );
    assert!(!out.exists(), "a dry run writes nothing");
    let (c, s, _) = plan2cell(&[
        "cell",
        "up",
        WELDING,
        "-c",
        CFG,
        "-o",
        o,
        "--no-check",
        "--dry-run",
        "--json",
        "--cell-config",
        n,
    ]);
    assert_eq!(c, 0);
    let v: Value = serde_json::from_str(&s).unwrap();
    assert_eq!(v["schema"], "plan2cell/cell-report@1");
    assert_eq!(v["projects"].as_array().unwrap().len(), 3);
    // Without a cell.toml only a dry run works: config (3). A stage without a build: usage (2).
    assert_eq!(
        plan2cell(&[
            "cell",
            "verify",
            "examples/welding_cell_2_small",
            "--cell-config",
            n
        ])
        .0,
        3
    );
    assert_eq!(
        plan2cell(&["cell", "verify", "--dry-run", "--cell-config", n]).0,
        2
    );
}

#[test]
fn options_json_lists_targets_and_defaults() {
    let (c, s, e) = plan2cell(&["options", "--json"]);
    assert_eq!(c, 0, "{e}");
    let v: Value = serde_json::from_str(&s).unwrap();
    assert_eq!(v["schema"], "plan2cell/options@1");
    let dialects: Vec<(&str, bool)> = v["plc_dialects"]
        .as_array()
        .unwrap()
        .iter()
        .map(|d| {
            (
                d["id"].as_str().unwrap(),
                d["implemented"].as_bool().unwrap(),
            )
        })
        .collect();
    assert_eq!(
        dialects,
        [
            ("codesys", true),
            ("twincat3", false),
            ("tia-scl", false),
            ("plcopen-xml", false),
            ("rockwell-l5x", false)
        ]
    );
    let langs: Vec<&str> = v["robot_languages"]
        .as_array()
        .unwrap()
        .iter()
        .map(|l| l["id"].as_str().unwrap())
        .collect();
    assert_eq!(langs, ["krl", "rapid", "urscript", "karel", "inform"]);
    assert_eq!(v["robot_languages"][0]["label"], "KUKA KRL");
    assert_eq!(v["robot_languages"][1]["implemented"], false);
    assert_eq!(v["profiles"][0]["id"], "sim");
    assert_eq!(v["profiles"][1]["id"], "vw-ks24");
    assert_eq!(v["defaults"]["job_number"], "word_le");
    assert_eq!(v["defaults"]["opcua_symbols"], false);
    assert_eq!(v["defaults"]["plc_dialect"], "codesys");
    let (c, s, _) = plan2cell(&["options"]);
    assert_eq!(c, 0);
    assert!(
        s.contains("twincat3") && s.contains("not implemented yet"),
        "{s}"
    );
}

#[test]
fn build_json_reports_files_diagnostics_and_failures() {
    let d = tempfile::tempdir().unwrap();
    let out = d.path().join("j");
    let o = out.to_str().unwrap();
    // No [[krl.robot]] e_axes: the opt-in default declares them, with a note per robot.
    let (c, s, e) = plan2cell(&[
        "build",
        WELDING,
        "-o",
        o,
        "--no-check",
        "--json",
        "--krl-e-axes-default",
        "none",
    ]);
    assert_eq!(c, 0, "{s}{e}");
    let v: Value = serde_json::from_str(&s).expect("stdout is one JSON document");
    assert_eq!(v["schema"], "plan2cell/build-report@1");
    assert_eq!(v["ok"], true);
    assert_eq!(v["exit_code"], 0);
    assert_eq!(v["written"], true);
    assert_eq!(
        v["summary"]["tree_id"],
        "bt_1790934549_welding_cell_2_small"
    );
    let notes: Vec<&str> = v["notes"]
        .as_array()
        .unwrap()
        .iter()
        .map(|n| n.as_str().unwrap())
        .collect();
    assert!(
        notes.iter().any(|n| n.starts_with("robot1: e_axes")),
        "{notes:?}"
    );
    let files = v["files"].as_array().unwrap();
    let head = files
        .iter()
        .find(|f| f["path"] == "plc/codesys/head/POU/FB_Cell_Body.st")
        .expect("head PLC in the file list");
    assert!(head["bytes"].as_u64().unwrap() > 0);
    assert!(out.join("plc/codesys/head/POU/FB_Cell_Body.st").is_file());
    assert!(out.join("krl/robot1/KRC/R1/Folgen/folge11.src").is_file());
    for d in v["diagnostics"].as_array().unwrap() {
        assert!(
            ["error", "warning", "info"].contains(&d["severity"].as_str().unwrap()),
            "{d}"
        );
        assert!(d["message"].is_string());
    }
    // A failing build still prints the report: KRL without e_axes is a config error (3).
    let (c, s, _) = plan2cell(&[
        "build",
        WELDING,
        "-o",
        d.path().join("f").to_str().unwrap(),
        "--json",
        "--no-check",
    ]);
    assert_eq!(c, 3);
    let v: Value = serde_json::from_str(&s).unwrap();
    assert_eq!(
        (v["ok"].as_bool(), v["exit_code"].as_i64()),
        (Some(false), Some(3))
    );
    assert_eq!(v["diagnostics"][0]["severity"], "error");
    assert!(v["error"].as_str().unwrap().contains("e_axes"), "{v}");
    // --strict: the promoted warnings are listed.
    let (c, s, _) = plan2cell(&[
        "build",
        WELDING,
        "-o",
        d.path().join("s").to_str().unwrap(),
        "--json",
        "--no-check",
        "--strict",
        "--krl-e-axes-default",
        "none",
    ]);
    assert_eq!(c, 6);
    let v: Value = serde_json::from_str(&s).unwrap();
    assert!(
        v["diagnostics"]
            .as_array()
            .unwrap()
            .iter()
            .any(|d| d["severity"] == "warning"),
        "{v}"
    );
    // --dry-run: the report, nothing written.
    let n = d.path().join("n");
    let (c, s, _) = plan2cell(&[
        "build",
        WELDING,
        "-c",
        CFG,
        "-o",
        n.to_str().unwrap(),
        "--json",
        "--dry-run",
        "--no-check",
    ]);
    assert_eq!(c, 0);
    let v: Value = serde_json::from_str(&s).unwrap();
    assert_eq!(v["written"], false);
    assert!(!n.exists());
}

#[test]
fn planned_dialects_and_languages_fail_cleanly() {
    let d = tempfile::tempdir().unwrap();
    let o = d.path().join("x");
    for dialect in ["twincat3", "tia-scl", "plcopen-xml", "rockwell-l5x"] {
        let (c, s, e) = plan2cell(&[
            "build",
            WELDING,
            "-c",
            CFG,
            "-o",
            o.to_str().unwrap(),
            "--json",
            "--plc-dialect",
            dialect,
        ]);
        assert_eq!(c, 5, "{dialect}: {s}{e}");
        let v: Value = serde_json::from_str(&s).unwrap();
        assert_eq!(v["ok"], false);
        assert!(
            v["error"].as_str().unwrap().contains("not implemented"),
            "{v}"
        );
        assert!(!e.contains("panicked"), "{e}");
    }
    for lang in ["rapid", "urscript", "karel", "inform"] {
        let (c, _, e) = plan2cell(&["check", WELDING, "-c", CFG, "--robot-language", lang]);
        assert_eq!(c, 5, "{lang}: {e}");
        assert!(e.contains("not implemented"), "{e}");
    }
    // In the JSON report a planned robot language is the robot side's failure, not the PLC's.
    let (c, s, e) = plan2cell(&[
        "build",
        WELDING,
        "-c",
        CFG,
        "-o",
        o.to_str().unwrap(),
        "--json",
        "--robot-language",
        "rapid",
    ]);
    assert_eq!(c, 5, "{s}{e}");
    let v: Value = serde_json::from_str(&s).unwrap();
    assert_eq!(v["diagnostics"][0]["source"], "krl", "{v}");
    assert_eq!(v["diagnostics"][0]["severity"], "error", "{v}");
    assert!(!o.exists());
    // Unknown ids are usage errors (2).
    assert_eq!(plan2cell(&["check", WELDING, "--plc-dialect", "s7"]).0, 2);
    assert_eq!(
        plan2cell(&["check", WELDING, "--robot-language", "vplus"]).0,
        2
    );
}

/// plan2cell.toml = the example config plus `extra` (hardware overrides).
fn cfg_with(dir: &Path, extra: &str) -> PathBuf {
    let p = dir.join("plan2cell.toml");
    let base = std::fs::read_to_string(repo().join(CFG)).unwrap();
    std::fs::write(&p, format!("{base}\n{extra}")).unwrap();
    p
}

#[test]
fn hardware_reports_the_defaults() {
    let (c, s, e) = plan2cell(&["hardware", WELDING, "-c", CFG, "--json"]);
    assert_eq!(c, 0, "{s}{e}");
    let v: Value = serde_json::from_str(&s).unwrap();
    assert_eq!(v["schema"], "plan2cell/hardware@1");
    assert_eq!(v["ok"], true);
    assert_eq!(v["issues"], json!([]));
    assert_eq!(
        v["network"],
        json!({
            "headIp": "192.168.100.10", "netmask": "255.255.255.0", "prefix": 24,
            "gateway": "0.0.0.0", "controllerStation": "controller", "nic": "eth1",
            "robotHostBase": 10, "ipPoolFirst": "192.168.100.2", "ipPoolLast": "192.168.100.254"
        })
    );
    assert_eq!(v["networkDefaults"], v["network"]);
    let r = &v["robots"];
    assert_eq!(r.as_array().unwrap().len(), 2, "{v}");
    for (i, r) in r.as_array().unwrap().iter().enumerate() {
        let n = i + 1;
        assert_eq!(r["ns"], format!("robot{n}"));
        assert_eq!(r["station"], format!("kuka-robi{n}"));
        assert_eq!(r["deviceName"], format!("kuka_robi{n}"));
        assert_eq!(r["ip"], format!("192.168.100.{}", 10 + n));
        assert_eq!(r["model"], "KR240R2900_ULTRA");
        assert_eq!(r["external"], false);
        assert_eq!(r["defaults"]["station"], r["station"]);
    }
    // No external robot: no KUKA VC.
    assert_eq!(v["kukaVc"]["controllers"], json!([]));
    assert_eq!(v["kukaVc"]["kliGw"], Value::Null);
    // Text form.
    let (c, s, _) = plan2cell(&["hardware", WELDING, "-c", CFG]);
    assert_eq!(c, 0);
    assert!(s.contains("station kuka-robi2"), "{s}");
}

#[test]
fn hardware_overrides_flow_into_the_build() {
    let d = tempfile::tempdir().unwrap();
    let cfg = cfg_with(
        d.path(),
        "[plc.network]\nhead_ip = \"10.20.0.5\"\ngateway = \"10.20.0.1\"\n\
         controller_station = \"head-plc\"\n\
         [[plc.robot]]\nns = \"robot1\"\nstation = \"weld-r1.line2\"\nip = \"10.20.0.31\"\n\
         external = true\nsafety_bypass = true\nsafety_module = false\n",
    );
    let c = cfg.to_str().unwrap();
    let (code, s, e) = plan2cell(&["hardware", WELDING, "-c", c, "--json"]);
    assert_eq!(code, 0, "{s}{e}");
    let v: Value = serde_json::from_str(&s).unwrap();
    assert_eq!(v["network"]["headIp"], "10.20.0.5");
    assert_eq!(v["network"]["controllerStation"], "head-plc");
    assert_eq!(v["networkDefaults"]["headIp"], "192.168.100.10");
    let r1 = &v["robots"][0];
    assert_eq!(
        (&r1["station"], &r1["ip"], &r1["external"]),
        (&json!("weld-r1.line2"), &json!("10.20.0.31"), &json!(true))
    );
    // Defaults follow the head's subnet: robot i = .(10 + i).
    assert_eq!(r1["defaults"]["ip"], "10.20.0.11");
    assert_eq!(r1["defaults"]["station"], "kuka-robi1");
    assert_eq!(v["robots"][1]["ip"], "10.20.0.12");
    assert_eq!(
        v["kukaVc"],
        json!({"kliPrefix": 24, "kliGw": "10.20.0.1", "headIp": "10.20.0.5", "controllers": [
            {"robotNs": "robot1", "kliIp": "10.20.0.31", "station": "weld-r1.line2"}
        ]})
    );
    // The build uses the very same values.
    let out = d.path().join("b");
    let (code, s, e) = plan2cell(&[
        "build",
        WELDING,
        "-c",
        c,
        "-o",
        out.to_str().unwrap(),
        "--no-check",
    ]);
    assert_eq!(code, 0, "{s}{e}");
    let p: Value = serde_json::from_str(
        &std::fs::read_to_string(out.join("plc/robots/robot1/program.json")).unwrap(),
    )
    .unwrap();
    assert_eq!(p["robot"]["station"], "weld-r1.line2");
    assert_eq!(p["robot"]["ip"], "10.20.0.31");
    let p2: Value = serde_json::from_str(
        &std::fs::read_to_string(out.join("plc/robots/robot2/program.json")).unwrap(),
    )
    .unwrap();
    assert_eq!(p2["robot"]["ip"], "10.20.0.12");
    let head = walk(&out.join("plc"))
        .into_iter()
        .filter(|f| f.extension().is_some_and(|x| x == "json"))
        .map(|f| std::fs::read_to_string(f).unwrap())
        .collect::<String>();
    assert!(
        head.contains("weld-r1.line2"),
        "station not in the head project"
    );
    assert!(
        head.contains("10.20.0.5"),
        "head IP not in the head project"
    );
    // The PN controller's DCP pool follows the head subnet (FirstIPAddress / LastIPAddress).
    let project = std::fs::read_to_string(out.join("plc/project.json")).unwrap();
    assert!(
        project.contains("[10,20,0,2]") && project.contains("[10,20,0,254]"),
        "pool not in 10.20.0.0/24"
    );
    assert!(
        !project.contains("[192,168,100,"),
        "a 192.168.100.x address left in project.json"
    );
}

fn walk(dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for e in std::fs::read_dir(dir).unwrap() {
        let p = e.unwrap().path();
        if p.is_dir() {
            out.extend(walk(&p));
        } else {
            out.push(p);
        }
    }
    out
}

#[test]
fn hardware_lists_every_issue_and_build_rejects_them() {
    let d = tempfile::tempdir().unwrap();
    let cfg = cfg_with(
        d.path(),
        "[plc.network]\ngateway = \"192.168.100.1\"\ncontroller_station = \"port-001\"\n\
         [[plc.robot]]\nns = \"robot1\"\nstation = \"Kuka_R1\"\nip = \"192.168.100.255\"\n\
         [[plc.robot]]\nns = \"robot2\"\nstation = \"192.168.0.1\"\nip = \"192.168.100.1\"\n",
    );
    let c = cfg.to_str().unwrap();
    let (code, s, e) = plan2cell(&["hardware", WELDING, "-c", c, "--json"]);
    assert_eq!(code, 3, "{s}{e}");
    let v: Value = serde_json::from_str(&s).unwrap();
    assert_eq!(v["ok"], false);
    let fields: Vec<&str> = v["issues"]
        .as_array()
        .unwrap()
        .iter()
        .map(|i| i["field"].as_str().unwrap())
        .collect();
    assert_eq!(
        fields,
        [
            "network.controllerStation",
            "robots.robot1.station",
            "robots.robot1.ip",
            "robots.robot2.station",
            "robots.robot2.ip",
        ],
        "{v}"
    );
    // The robots are still listed with what the config asks for.
    assert_eq!(v["robots"][0]["station"], "Kuka_R1");
    // A build stops at the first one (config error, 3).
    let (code, _, e) = plan2cell(&["check", WELDING, "-c", c, "--no-check"]);
    assert_eq!(code, 3, "{e}");
    assert!(e.contains("NameOfStation"), "{e}");
    // A duplicate station is caught by bt2plc's plan as well.
    let cfg = cfg_with(
        d.path(),
        "[[plc.robot]]\nns = \"robot2\"\nstation = \"kuka-robi1\"\n",
    );
    let (code, s, _) = plan2cell(&["hardware", WELDING, "-c", cfg.to_str().unwrap(), "--json"]);
    assert_eq!(code, 3);
    let v: Value = serde_json::from_str(&s).unwrap();
    assert_eq!(v["issues"][0]["field"], "robots.robot2.station", "{v}");
    let (code, _, e) = plan2cell(&["check", WELDING, "-c", cfg.to_str().unwrap(), "--no-check"]);
    assert_eq!(code, 3, "{e}");
    // Device names that clash with the generated project's objects or IEC words.
    let cfg = cfg_with(
        d.path(),
        "[[plc.robot]]\nns = \"robot1\"\ndevice_name = \"PLC_PRG\"\n\
         [[plc.robot]]\nns = \"robot2\"\ndevice_name = \"pn_controller\"\n",
    );
    let (code, s, _) = plan2cell(&["hardware", WELDING, "-c", cfg.to_str().unwrap(), "--json"]);
    assert_eq!(code, 3, "{s}");
    let v: Value = serde_json::from_str(&s).unwrap();
    let fields: Vec<&str> = v["issues"]
        .as_array()
        .unwrap()
        .iter()
        .map(|i| i["field"].as_str().unwrap())
        .collect();
    assert_eq!(
        fields,
        ["robots.robot1.deviceName", "robots.robot2.deviceName"],
        "{v}"
    );
    let (code, _, e) = plan2cell(&["check", WELDING, "-c", cfg.to_str().unwrap(), "--no-check"]);
    assert_eq!(code, 3, "{e}");
    assert!(e.contains("reserved"), "{e}");
    // Another head subnet: the PN controller's IP pool follows it.
    let cfg = cfg_with(
        d.path(),
        "[plc.network]\nhead_ip = \"10.1.0.20\"\ngateway = \"10.1.0.1\"\n\
         [[plc.robot]]\nns = \"robot1\"\nip = \"10.1.0.21\"\n[[plc.robot]]\nns = \"robot2\"\nip = \"10.1.0.22\"\n",
    );
    let (code, s, _) = plan2cell(&["hardware", WELDING, "-c", cfg.to_str().unwrap(), "--json"]);
    assert_eq!(code, 0, "{s}");
    let v: Value = serde_json::from_str(&s).unwrap();
    assert_eq!(
        (
            v["network"]["ipPoolFirst"].as_str(),
            v["network"]["ipPoolLast"].as_str()
        ),
        (Some("10.1.0.2"), Some("10.1.0.254"))
    );
    // ... unless set: then it must fit.
    let cfg = cfg_with(
        d.path(),
        "[plc.network]\nhead_ip = \"10.1.0.20\"\nip_pool_first = \"192.168.100.2\"\n\
         [[plc.robot]]\nns = \"robot1\"\nip = \"10.1.0.21\"\n[[plc.robot]]\nns = \"robot2\"\nip = \"10.1.0.22\"\n",
    );
    let (code, s, _) = plan2cell(&["hardware", WELDING, "-c", cfg.to_str().unwrap(), "--json"]);
    assert_eq!(code, 3, "{s}");
    let v: Value = serde_json::from_str(&s).unwrap();
    assert_eq!(v["issues"][0]["field"], "network.ipPool", "{v}");
    // An unreadable plan: the JSON says why, with the plan's exit code.
    let (code, s, _) = plan2cell(&["hardware", "nope.json", "--json"]);
    assert_eq!(code, 4);
    let v: Value = serde_json::from_str(&s).unwrap();
    assert_eq!(
        (v["ok"].clone(), v["error"]["code"].clone()),
        (json!(false), json!(4))
    );
}

/// `cell runtime`: read-only; a dry run prints the one remote command (the build's containers,
/// or every CODESYS container of the host without one).
#[cfg(feature = "cell")]
#[test]
fn cell_runtime_dry_run() {
    let cfg = "/nonexistent/plan2cell-cell.toml";
    let (c, s, e) = plan2cell(&[
        "cell",
        "runtime",
        "examples/cell_2robot",
        "--cell-config",
        cfg,
        "--dry-run",
        "--json",
    ]);
    assert_eq!(c, 0, "{e}");
    let v: serde_json::Value = serde_json::from_str(&s).unwrap();
    assert_eq!(v["schema"], "plan2cell/cell-runtime@1");
    let cmd = v["command"].as_str().unwrap();
    assert!(
        cmd.contains("for c in codesys-master codesys-device"),
        "{cmd}"
    );
    let (c, s, _) = plan2cell(&["cell", "runtime", "--cell-config", cfg, "--dry-run"]);
    assert_eq!(c, 0);
    assert!(s.contains("grep -E '^codesys-(master|device)'"), "{s}");
    assert_eq!(
        plan2cell(&[
            "cell",
            "runtime",
            "/nonexistent/build",
            "--cell-config",
            cfg,
            "--dry-run"
        ])
        .0,
        4
    );
}

/// `plan2cell cell hooks-check --exec` with a fake `kubectl` first on PATH: `get pod` answers a
/// pn-raw pod, `exec` answers only `describe`.
#[cfg(all(feature = "cell", unix))]
#[test]
fn hooks_check_exec_reads_the_pod_and_reports_json() {
    use std::os::unix::fs::PermissionsExt;
    let d = tempfile::tempdir().unwrap();
    let kubectl = d.path().join("kubectl");
    std::fs::write(
        &kubectl,
        r#"#!/bin/sh
case " $* " in
  *" get pod "*)
    echo '{"metadata":{"annotations":{"k8s.v1.cni.cncf.io/network-status":"[{\"interface\":\"net1\",\"ips\":[\"10.77.1.10\",\"10.77.1.1\"]}]"}},"spec":{"containers":[{"name":"head","securityContext":{"capabilities":{"drop":["AUDIT_WRITE","CHOWN","DAC_OVERRIDE","FOWNER","FSETID","KILL","MKNOD","NET_BIND_SERVICE","SETFCAP","SETGID","SETPCAP","SETUID","SYS_CHROOT"]}}}]}}' ;;
  *" -- /opt/factoryos/hook describe "*)
    cat >/dev/null
    echo '{"schema":"factoryos/hook@1","verb":"describe","ok":true,"exit":0,"contract":"factoryos/hook@1","role":"head-plc","runtime":"fake","verbs":[],"versions":{}}' ;;
  *) cat >/dev/null 2>&1; echo "error: not here" >&2; exit 1 ;;
esac
"#,
    )
    .unwrap();
    std::fs::set_permissions(&kubectl, std::fs::Permissions::from_mode(0o755)).unwrap();
    let path = format!("{}:{}", d.path().display(), std::env::var("PATH").unwrap_or_default());
    let o = Command::new(env!("CARGO_BIN_EXE_plan2cell"))
        .args(["cell", "hooks-check", "--exec", "cell-jdoe/n1-head/head", "--kubeconfig", "/nonexistent", "--json"])
        .env("PATH", path)
        .current_dir(repo())
        .output()
        .unwrap();
    let v: Value = serde_json::from_slice(&o.stdout).unwrap();
    assert_eq!(v["schema"], "plan2cell/hooks-check@1");
    assert_eq!((v["mode"].as_str(), v["role"].as_str(), v["profile"].as_str()), (Some("exec"), Some("head-plc"), Some("pn-raw")));
    let check = |id: &str| v["checks"].as_array().unwrap().iter().find(|c| c["id"] == id).unwrap().clone();
    assert_eq!(check("HC02")["ok"], true);
    assert_eq!(check("HC09")["skipped"], true, "--exec without --mutating writes nothing");
    assert_eq!(check("HC06")["ok"], false);
    assert_eq!(o.status.code(), Some(7), "{}", String::from_utf8_lossy(&o.stderr));
}

#[cfg(feature = "cell")]
#[test]
fn hooks_check_usage() {
    assert_eq!(plan2cell(&["cell", "hooks-check"]).0, 2);
    assert_eq!(plan2cell(&["cell", "hooks-check", "--exec", "a/b/c", "--docker", "i"]).0, 2);
    assert_eq!(plan2cell(&["cell", "hooks-check", "--exec", "Bad/b"]).0, 2);
    assert_eq!(plan2cell(&["cell", "hooks-check", "--exec", "a/b/c", "--role", "plc"]).0, 2);
    assert_eq!(plan2cell(&["cell", "hooks-check", "--exec", "a/b/c", "--profile", "privileged"]).0, 2);
    assert_eq!(plan2cell(&["cell", "hooks-check", "--docker", "i", "--env", "lower=x"]).0, 2);
}
