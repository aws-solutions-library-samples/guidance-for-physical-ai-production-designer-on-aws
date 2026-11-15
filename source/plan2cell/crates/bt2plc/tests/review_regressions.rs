//! Regression tests for confirmed code-review findings.

mod common;
use bt2plc::config::RobotCfg;
use bt2plc::input::{SourceSel, load_str};
use bt2plc::{Config, Diagnostics, GenerateOptions};
use common::*;
use serde_json::json;

fn plan_err(text: &str, cfg: &Config) -> String {
    match plan_json(text, cfg) {
        Ok(_) => panic!("expected an error"),
        Err(e) => e.to_string(),
    }
}

#[test]
fn xml_structural_problems_are_errors() {
    let missing = r#"<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M"><Sequence>
        <MoveHome robot_ns="r1"/><SubTree ID="Typo"/></Sequence></BehaviorTree></root>"#;
    let two_roots = r#"<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M">
        <MoveHome robot_ns="r1"/><MoveHome robot_ns="r2"/></BehaviorTree></root>"#;
    for x in [missing, two_roots] {
        let mut d = Diagnostics::default();
        let e = load_str(x, "t.xml", SourceSel::Auto, None, &mut d)
            .unwrap_err()
            .to_string();
        assert!(e.contains("E002"), "{e}");
    }
}

#[test]
fn childless_subtree_and_leaf_children_are_rejected() {
    let mut t = Tree::default();
    let root = seq(vec![
        t.exec("r1", "A", 1.0),
        json!({"type": "SubTree", "ports": {"ID": "R2Job"}}),
    ]);
    assert!(plan_err(&t.envelope(root), &Config::default()).contains("E002"));
    let mut t = Tree::default();
    let mut a = t.exec("r1", "A", 1.0);
    let b = t.exec("r2", "B", 1.0);
    a["children"] = json!([b]);
    let root = seq(vec![a]);
    assert!(plan_err(&t.envelope(root), &Config::default()).contains("E015"));
}

#[test]
fn json_pre_post_conditions_are_rejected() {
    let mut t = Tree::default();
    let mut a = t.exec("r1", "A", 1.0);
    a["ports"]["_skipIf"] = json!("skip_weld");
    assert!(plan_err(&t.envelope(seq(vec![a])), &Config::default()).contains("E002"));
}

#[test]
fn interlock_target_without_has_interlock_is_ignored() {
    let mut t = Tree::default();
    let mut a = t.exec("r1", "A", 1.0);
    a["ports"]["interlock_target"] = json!("{interlock_r1_A}");
    t.bb.insert(
        "interlock_r1_A".into(),
        json!({"ee_position": [0, 0, 1], "joint_positions": [0, 0, 0, 0, 0, 0]}),
    );
    plan_json(&t.envelope(seq(vec![a.clone()])), &Config::default()).unwrap();
    a["ports"]["has_interlock"] = json!(true);
    assert!(plan_err(&t.envelope(seq(vec![a])), &Config::default()).contains("E020"));
}

#[test]
fn xml_identifiers_stay_verbatim_and_private_subtree_keys_do_not_resolve() {
    let xml = r#"<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M"><Sequence>
        <ExecuteTrajectory robot_ns="r1" operation_id="1.10" trajectory="{t}"/></Sequence></BehaviorTree></root>"#;
    let mut d = Diagnostics::default();
    let mut bb = serde_json::Map::new();
    bb.insert("t".into(), json!([{"positions": [0,0,0,0,0,0], "time_s": 0.0}, {"positions": [0,0,0,0,0,0], "time_s": 1.0}]));
    let doc = load_str(
        xml,
        "t.xml",
        SourceSel::Auto,
        Some(bb.clone().into_iter().collect()),
        &mut d,
    )
    .unwrap();
    let p = bt2plc::plan(&doc, &Config::default(), &mut d).unwrap();
    assert_eq!(p.ops[0].operation_id, "1.10");
    let private = r#"<root BTCPP_format="4" main_tree_to_execute="M">
        <BehaviorTree ID="M"><Sequence><SubTree ID="Job" robot="r1"/></Sequence></BehaviorTree>
        <BehaviorTree ID="Job"><ExecuteTrajectory robot_ns="{robot}" operation_id="A" trajectory="{t}"/></BehaviorTree></root>"#;
    let doc = load_str(
        private,
        "t.xml",
        SourceSel::Auto,
        Some(bb.into_iter().collect()),
        &mut d,
    )
    .unwrap();
    let e = bt2plc::plan(&doc, &Config::default(), &mut d)
        .unwrap_err()
        .to_string();
    assert!(e.contains("E008") && e.contains("not remapped"), "{e}");
}

#[test]
fn robot_names_and_addresses_are_validated() {
    let mut t = Tree::default();
    let root = par(vec![t.exec("r1", "A", 1.0), t.exec("R1", "B", 1.0)]);
    assert!(plan_err(&t.envelope(root), &Config::default()).contains("case-insensitive"));
    let mut t = Tree::default();
    let root = seq(vec![t.exec("head", "A", 1.0)]);
    let text = t.envelope(root);
    assert!(plan_err(&text, &Config::default()).contains("E018"));
    let text = fixture(WELDING);
    let mut cfg = Config::default();
    cfg.robots.push(RobotCfg {
        ns: "../../esc".into(),
        include_if_idle: true,
        ..Default::default()
    });
    assert!(plan_err(&text, &cfg).contains("E018"));
    let mut cfg = Config::default();
    cfg.robots.push(RobotCfg {
        ns: "robot1".into(),
        tag: Some("R2".into()),
        ..Default::default()
    });
    assert!(plan_err(&text, &cfg).contains("same tag"));
    let mut cfg = Config::default();
    cfg.robots.push(RobotCfg {
        ns: "robot1".into(),
        tag: Some("R-1".into()),
        ..Default::default()
    });
    assert!(plan_err(&text, &cfg).contains("tag"));
    let mut cfg = Config::default();
    cfg.robots.push(RobotCfg {
        ns: "robot1".into(),
        prod_folge: Some(125),
        ..Default::default()
    });
    assert!(cfg.validate().is_err());
}

#[test]
fn hostile_cell_id_cannot_break_st_comments_or_paths() {
    let text =
        fixture(TWO_ROBOT).replace("\"cell_id\": \"cell_2robot\"", "\"cell_id\": \"a/b *) x\"");
    let mut d = Diagnostics::default();
    let doc = load_str(&text, "t.json", SourceSel::Auto, None, &mut d).unwrap();
    let g = bt2plc::generate(
        &doc,
        &Config::default(),
        &GenerateOptions {
            st_check: false,
            ..Default::default()
        },
    )
    .unwrap();
    for (path, body) in g.output.files() {
        if path.ends_with(".st") {
            assert!(!body.contains("*) x"), "{path}");
        }
    }
    assert!(
        g.projects
            .iter()
            .all(|p| !p.name.contains('/') && !p.name.contains(' '))
    );
}

#[test]
fn decimation_shrinks_the_robot_tables() {
    let mut d = Diagnostics::default();
    let doc = load_str(&fixture(WELDING), WELDING, SourceSel::Auto, None, &mut d).unwrap();
    let opts = GenerateOptions {
        st_check: false,
        ..Default::default()
    };
    let full = bt2plc::generate(&doc, &Config::default(), &opts).unwrap();
    let mut cfg = Config::default();
    cfg.robot_sim.decimate_tol_deg = 0.5;
    let dec = bt2plc::generate(&doc, &cfg, &opts).unwrap();
    let n = |g: &bt2plc::Generated| {
        g.output
            .get("codesys/robots/robot1/GVL/GVL_Program.st")
            .unwrap()
            .matches("(rT :=")
            .count()
    };
    assert!(n(&dec) < n(&full), "{} !< {}", n(&dec), n(&full));
}

#[test]
fn head_kuka_parameters_use_the_identification_path() {
    let mut d = Diagnostics::default();
    let doc = load_str(&fixture(WELDING), WELDING, SourceSel::Auto, None, &mut d).unwrap();
    let g = bt2plc::generate(
        &doc,
        &Config::default(),
        &GenerateOptions {
            st_check: false,
            ..Default::default()
        },
    )
    .unwrap();
    let pj = g.output.get("project.json").unwrap();
    assert!(pj.contains("\"member\": \"Identification.StationName\""));
    assert!(pj.contains("\"name\": \"DriverOptions\""));
}

#[test]
fn external_robots_have_no_simulator_and_may_bypass_slot1_safety() {
    let mut d = Diagnostics::default();
    let doc = load_str(&fixture(WELDING), WELDING, SourceSel::Auto, None, &mut d).unwrap();
    let mut cfg = Config::default();
    for ns in ["robot1", "robot2"] {
        cfg.robots.push(RobotCfg {
            ns: ns.into(),
            external: ns == "robot1",
            safety_bypass: ns == "robot1",
            ..Default::default()
        });
    }
    let opts = GenerateOptions {
        st_check: false,
        ..Default::default()
    };
    let g = bt2plc::generate(&doc, &cfg, &opts).unwrap();
    let keys: Vec<&str> = g.projects.iter().map(|p| p.key.as_str()).collect();
    assert_eq!(keys, ["head", "robot2"], "robot1 is external");
    // The head keeps both PROFINET devices.
    let pj = g.output.get("project.json").unwrap();
    assert!(pj.contains("kuka_robi1") && pj.contains("kuka_robi2"));
    let compose = g.output.get("docker/docker-compose.robots.yml").unwrap();
    assert!(
        compose.contains("# plan2cell-external-services: device\n"),
        "{compose}"
    );
    assert!(
        compose.contains("container_name: codesys-device-r2"),
        "{compose}"
    );
    let gvl = g.output.get("codesys/head/GVL/GVL_Cell.st").unwrap();
    assert!(gvl.contains("abSafetyBypass"), "{gvl}");
    assert!(
        gvl.contains("[TRUE, FALSE]") || gvl.contains("[TRUE,FALSE]"),
        "{gvl}"
    );
    let fb = g
        .output
        .files()
        .iter()
        .find(|(p, _)| p.starts_with("codesys/head/") && p.ends_with("FB_VassRobot_Body.st"))
        .map(|(_, b)| b.as_str());
    assert!(fb.is_some_and(|b| b.contains("GVL_Cell.abSafetyBypass[nRobot]")));

    // Both external: no simulator at all, no extra services.
    cfg.robots[1].external = true;
    let g = bt2plc::generate(&doc, &cfg, &opts).unwrap();
    assert_eq!(g.projects.len(), 1);
    let compose = g.output.get("docker/docker-compose.robots.yml").unwrap();
    assert!(compose.contains("# plan2cell-external-services: device device-r2\n"));
    assert!(compose.contains("services: {}"), "{compose}");
}

#[test]
fn without_the_safety_module_the_head_uses_dim3_with_slot1_empty() {
    let mut d = Diagnostics::default();
    let doc = load_str(&fixture(WELDING), WELDING, SourceSel::Auto, None, &mut d).unwrap();
    let mut cfg = Config::default();
    cfg.robots.push(RobotCfg {
        ns: "robot1".into(),
        external: true,
        safety_bypass: true,
        safety_module: false,
        ..Default::default()
    });
    cfg.validate().unwrap();
    let opts = GenerateOptions {
        st_check: false,
        ..Default::default()
    };
    let g = bt2plc::generate(&doc, &cfg, &opts).unwrap();
    let pj: serde_json::Value =
        serde_json::from_str(g.output.get("project.json").unwrap()).unwrap();
    let head = pj["projects"]
        .as_array()
        .unwrap()
        .iter()
        .find(|p| p["role"] == "head_plc")
        .unwrap();
    let kukas = head["device_tree"]["children"][0]["children"][0]["children"]
        .as_array()
        .unwrap();
    let (r1, r2) = (&kukas[0], &kukas[1]);
    assert_eq!(r1["name"], "kuka_robi1");
    assert_eq!(
        r1["device_id"],
        json!({"type": 81,
               "id": "GSDML-019B-0301-V2.25-KUKA-Roboter-GmbH-VKR C4-Device-V1.0-20121030 - DIM 3",
               "version": "V8.3 - HW 1"})
    );
    assert_eq!(
        r1["slots"][0],
        json!({"slot": 1, "empty": true, "module_id": null, "profisafe": false, "io": []})
    );
    assert_eq!(r1["slots"][1]["name"], "VASS_DIO256_R1");
    assert_eq!(r1["slots"][1]["module_id"], "7");
    assert_eq!(r1["slots"][1]["io"].as_array().unwrap().len(), 2);
    // robot2 keeps DIM 2 with FDIO64.
    assert!(r2["device_id"]["id"].as_str().unwrap().ends_with("DIM 2"));
    assert_eq!(r2["slots"][0]["name"], "SafeIO_FDIO64_R2");
    // Steps: slot 1 of robot1 must be empty, no FDIO64 module for it.
    let steps = head["steps"].as_array().unwrap();
    let of = |dev: &str| -> Vec<(String, u64)> {
        steps
            .iter()
            .filter(|s| s["device"] == dev && s["slot"].is_u64())
            .map(|s| {
                (
                    s["op"].as_str().unwrap().into(),
                    s["slot"].as_u64().unwrap(),
                )
            })
            .collect()
    };
    assert_eq!(
        of("kuka_robi1"),
        [
            ("ensure_empty_slot".to_string(), 1),
            ("ensure_module".into(), 2)
        ]
    );
    assert_eq!(
        of("kuka_robi2"),
        [
            ("ensure_module".to_string(), 1),
            ("ensure_module".into(), 2)
        ]
    );
    let add = steps
        .iter()
        .position(|s| s["op"] == "add_device" && s["name"] == "kuka_robi1")
        .unwrap();
    let empty = steps
        .iter()
        .position(|s| s["op"] == "ensure_empty_slot")
        .unwrap();
    assert!(add < empty, "the slot is emptied after add_device");
    assert!(!pj.to_string().contains("SafeIO_FDIO64_R1"));
    // The DIM 3 DeviceID is a prerequisite too.
    let pre: Vec<&str> = pj["prerequisites"]
        .as_array()
        .unwrap()
        .iter()
        .map(|p| p["id"].as_str().unwrap())
        .collect();
    assert_eq!(pre, ["gsdml", "gsdml_dim3", "pn_device_desc"]);
    // No slot-1 IO mapping rows for robot1 (CSV, PROJECT.md); robot2 keeps them.
    let csv = g.output.get("codesys/head/io_mapping.csv").unwrap();
    assert!(!csv.contains("SafeIO_FDIO64_R1"), "{csv}");
    assert!(csv.contains("aVassInputData_R1"), "{csv}");
    assert_eq!(csv.matches(",SafeIO_FDIO64_R2").count(), 16, "{csv}");
    let md = g.output.get("PROJECT.md").unwrap();
    assert!(!md.contains("SafeIO_FDIO64_R1"));
    assert!(md.contains("slot 1: **empty**"), "{md}");
    // The agent prompt says to unplug slot 1, and its ScriptEngine example uses DIM 3.
    let prompt = g.output.get("MCP_PROMPT.md").unwrap();
    assert!(prompt.contains("`\"empty\": true`"), "{prompt}");
    assert!(prompt.contains(&format!(
        "pnc.add('kuka_robi1', 81, '{}', '{}')",
        bt2plc::backend::codesys::project_json::KUKA_GSDML_ID_DIM3,
        bt2plc::backend::codesys::project_json::KUKA_GSDML_VERSION_DIM3
    )));
    // The bypass ignores the (unmapped) safe bits.
    let gvl = g.output.get("codesys/head/GVL/GVL_Cell.st").unwrap();
    assert!(gvl.contains("[TRUE, FALSE]") || gvl.contains("[TRUE,FALSE]"));

    // safety_module = false needs safety_bypass (and an external robot).
    cfg.robots[0].safety_bypass = false;
    let e = cfg.validate().unwrap_err();
    assert_eq!(e.exit_code(), 3);
    assert!(e.to_string().contains("safety_bypass"), "{e}");
    cfg.robots[0].safety_bypass = true;
    cfg.robots[0].external = false;
    assert!(cfg.validate().is_err());
}

#[test]
fn the_default_keeps_the_safety_module() {
    let c: Config = toml::from_str("[[robot]]\nns = \"robot1\"\n").unwrap();
    assert!(c.robots[0].safety_module);
    assert!(c.addressing("robot1", 1).unwrap().safety_module);
    assert!(Config::default().addressing("r", 1).unwrap().safety_module);
}
