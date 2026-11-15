//! `robots/<ns>/program.json` minor-1 additions agree with the plan (cell_plan.json) and the config.

mod common;
use bt2plc::config::RobotCfg;
use bt2plc::{Config, GenerateOptions};
use common::*;
use serde_json::Value;

fn generate(sample: &str, cfg: &Config) -> bt2plc::Generated {
    let mut d = bt2plc::Diagnostics::default();
    let doc = bt2plc::input::load_str(
        &fixture(sample),
        sample,
        bt2plc::input::SourceSel::Auto,
        None,
        &mut d,
    )
    .unwrap();
    bt2plc::generate(
        &doc,
        cfg,
        &GenerateOptions {
            st_check: false,
            ..Default::default()
        },
    )
    .unwrap()
}

fn json(g: &bt2plc::Generated, f: &str) -> Value {
    serde_json::from_str(g.output.get(f).unwrap()).unwrap()
}

#[test]
fn additions_match_the_cell_plan() {
    let g = generate(WELDING, &Config::default());
    let plan = json(&g, "cell_plan.json");
    for ns in ["robot1", "robot2"] {
        let p = json(&g, &format!("robots/{ns}/program.json"));
        assert_eq!(p["schema"], "bt2plc/robot-program@1");
        assert_eq!(p["schema_minor"], 1);
        assert_eq!(p["robot"]["model"], "unknown");
        let pr = &p["protocol"];
        assert_eq!(pr["profile"], "sim");
        assert_eq!(pr["job_number"], "word_le");
        assert_eq!(pr["error_codes"]["unknown_folge"], 0x0101);
        assert_eq!(pr["error_codes"]["plan_mismatch"], 0x0103);
        assert_eq!(pr["error_codes"]["start_deviation"], 0x0201);
        assert_eq!(pr["at_home_tol_deg"], 0.5);
        assert_eq!(pr["start_tol_deg"], 1.0);
        assert_eq!(pr["home_return_time_s"], 7.0);
        assert_eq!(pr["handshake_s"], 5.0);
        assert_eq!(pr["abort_limit_s"], 10.0);
        assert_eq!(pr["home_limit_s"], 15.5);
        let zones: Vec<(String, u64)> = p["zones"]
            .as_array()
            .unwrap()
            .iter()
            .map(|z| {
                (
                    z["name"].as_str().unwrap().to_string(),
                    z["bit"].as_u64().unwrap(),
                )
            })
            .collect();
        let want: Vec<(String, u64)> = plan["zones"]
            .as_array()
            .unwrap()
            .iter()
            .map(|z| {
                (
                    z["name"].as_str().unwrap().to_string(),
                    z["bit"].as_u64().unwrap(),
                )
            })
            .collect();
        assert_eq!(zones, want);
        for op in p["ops"].as_array().unwrap() {
            let id = op["operation_id"].as_str().unwrap();
            let po = plan["ops"]
                .as_array()
                .unwrap()
                .iter()
                .find(|o| o["robot"] == ns && o["operation_id"] == id)
                .unwrap_or_else(|| panic!("{ns}.{id} not in cell_plan.json"));
            assert_eq!(op["expected_s"], po["expected_s"], "{ns}.{id}");
            assert_eq!(op["limit_s"], po["limit_s"], "{ns}.{id}");
            assert_eq!(op["delay_exact"], po["delay_exact"], "{ns}.{id}");
            for w in op["waypoints"].as_array().unwrap() {
                assert!(
                    matches!(w["phase"].as_str(), Some("transit" | "approach")),
                    "{ns}.{id}: phase {}",
                    w["phase"]
                );
            }
        }
    }
    // W01 in the sample is 37 transit + 10 approach waypoints.
    let p = json(&g, "robots/robot1/program.json");
    let w01 = &p["ops"][0]["waypoints"];
    let n = |ph: &str| {
        w01.as_array()
            .unwrap()
            .iter()
            .filter(|w| w["phase"] == ph)
            .count()
    };
    assert_eq!((n("transit"), n("approach")), (37, 10));
    // Head watchdog of W01: 1.5 * (10.632 + 0.5) + 5.
    assert_eq!(p["ops"][0]["limit_s"], 21.698);
}

#[test]
fn model_and_job_number_come_from_the_config() {
    let mut cfg = Config::default();
    cfg.cell.job_number = bt2plc::config::JobNumber::Byte;
    cfg.robots.push(RobotCfg {
        ns: "robot2".into(),
        model: Some("KR240R2900_ULTRA".into()),
        ..Default::default()
    });
    let g = generate(WELDING, &cfg);
    let p1 = json(&g, "robots/robot1/program.json");
    let p2 = json(&g, "robots/robot2/program.json");
    assert_eq!(p1["robot"]["model"], "unknown");
    assert_eq!(p2["robot"]["model"], "KR240R2900_ULTRA");
    assert_eq!(p2["protocol"]["job_number"], "byte");
}

#[test]
fn opcua_symbols_add_the_symbol_config_step_and_keep_the_plan_id() {
    let off = generate(WELDING, &Config::default());
    let mut cfg = Config::default();
    cfg.opcua.symbols = true;
    let on = generate(WELDING, &cfg);
    // The plan id, the PLC code and the robot programs stay as they are.
    for (f, text) in off.output.files() {
        // cell_plan.json echoes the configuration.
        if f.ends_with(".md") || f == "project.json" || f == "cell_plan.json" {
            continue;
        }
        assert_eq!(on.output.get(f), Some(text.as_str()), "{f}");
    }
    assert_eq!(
        json(&on, "cell_plan.json")["source"],
        json(&off, "cell_plan.json")["source"]
    );
    assert_eq!(
        json(&on, "cell_plan.json")["config"]["opcua"]["symbols"],
        true
    );
    assert!(
        json(&off, "cell_plan.json")["config"]
            .get("opcua")
            .is_none()
    );
    let off_pj = json(&off, "project.json");
    let pj = json(&on, "project.json");
    for p in off_pj["projects"].as_array().unwrap() {
        assert!(p.get("symbol_config").is_none());
        assert!(
            p["steps"]
                .as_array()
                .unwrap()
                .iter()
                .all(|s| s["op"] != "symbol_config")
        );
    }
    let head = pj["projects"]
        .as_array()
        .unwrap()
        .iter()
        .find(|p| p["role"] == "head_plc")
        .unwrap();
    let ops: Vec<&str> = head["steps"]
        .as_array()
        .unwrap()
        .iter()
        .map(|s| s["op"].as_str().unwrap())
        .collect();
    assert_eq!(ops[ops.len() - 3..], ["build", "symbol_config", "save"]);
    let sc = &head["symbol_config"];
    assert_eq!(sc["support_opc_ua"], true);
    let syms: Vec<(String, Option<String>, String)> = sc["symbols"]
        .as_array()
        .unwrap()
        .iter()
        .map(|s| {
            (
                s["signature"].as_str().unwrap().to_string(),
                s["variable"].as_str().map(str::to_string),
                s["access"].as_str().unwrap().to_string(),
            )
        })
        .collect();
    assert_eq!(
        syms,
        [
            ("PLC_PRG".to_string(), None, "ReadOnly".to_string()),
            (
                "GVL_Cell".to_string(),
                Some("PLAN_ID".to_string()),
                "ReadOnly".to_string()
            ),
        ]
    );
    let req: Vec<&str> = sc["require"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_str().unwrap())
        .collect();
    for r in [
        "PLC_PRG.eCellState",
        "PLC_PRG.bCellError",
        "PLC_PRG.udiCycles",
    ] {
        assert!(req.contains(&r), "{r}");
    }
    // Every required variable exists in the generated declarations.
    let decl = |f: &str| on.output.get(f).unwrap().to_string();
    let fb = decl("codesys/head/POU/FB_Cell.st");
    let prg = decl("codesys/head/POU/PLC_PRG.st");
    for r in &req {
        let (sig, var) = r.split_once('.').unwrap();
        let text = if sig == "FB_Cell" { &fb } else { &prg };
        assert!(
            text.lines()
                .any(|l| l.trim_start().starts_with(&format!("{var} "))),
            "{r} is not declared"
        );
    }
    assert!(on.output.get("PROJECT.md").unwrap().contains("## OPC UA"));
    assert!(
        on.output
            .get("MCP_PROMPT.md")
            .unwrap()
            .contains("Symbol configuration")
    );
    assert!(!off.output.get("PROJECT.md").unwrap().contains("## OPC UA"));
}
