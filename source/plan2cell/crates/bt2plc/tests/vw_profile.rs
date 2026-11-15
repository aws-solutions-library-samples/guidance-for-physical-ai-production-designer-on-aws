//! Head PLC in profile `vw-ks24` (Part B): the generated constants and ST, the shared interlock
//! map, and the CODESYS / trust rules over the generated files.

mod common;
use bt2plc::backend::model::strip_comments;
use bt2plc::config::{JobNumber, Profile, RobotCfg, vw_ks24 as vw};
use bt2plc::{Config, GenerateOptions};
use common::*;
use serde_json::Value;

fn vw_config(interlocks: &[(&str, u8)]) -> Config {
    let mut c = Config::default();
    c.protocol.profile = Profile::VwKs24;
    c.protocol.max_zones = 16;
    c.protocol.plan_id_byte = 12;
    c.cell.job_number = JobNumber::Byte;
    let map: std::collections::BTreeMap<String, u8> = interlocks
        .iter()
        .map(|(z, n)| ((*z).to_string(), *n))
        .collect();
    for ns in ["robot1", "robot2"] {
        c.robots.push(RobotCfg {
            ns: ns.into(),
            interlocks: map.clone(),
            ..Default::default()
        });
    }
    c
}

fn generated(name: &str, cfg: &Config) -> bt2plc::Generated {
    let mut d = bt2plc::Diagnostics::default();
    let doc = bt2plc::input::load_str(
        &fixture(name),
        name,
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

#[test]
fn gvl_cell_carries_the_vw_signal_map_and_the_interlock_table() {
    let g = generated(WELDING, &vw_config(&[("IL01", 1), ("IL02", 11)]));
    let gvl = g.output.get("codesys/head/GVL/GVL_Cell.st").unwrap();
    assert!(
        gvl.contains("PROFILE_VW              : BOOL := TRUE;"),
        "{gvl}"
    );
    for (name, v) in [
        ("VW_READY_OUT", vw::READY_OUT),
        ("VW_PF0_OUT", vw::PF0_OUT),
        ("VW_END_OUT", vw::FOLGE_END_OUT),
        ("VW_STEP_OUT", vw::STEP_OUT),
        ("VW_REL_OUT", vw::RELEASE_OUT),
        ("VW_REQ_OUT", vw::REQUEST_OUT),
        ("VW_SRB_IN", vw::SRB_IN),
    ] {
        assert!(
            gvl.lines().any(|l| {
                let l = l.trim();
                l.starts_with(name) && l.contains(&format!(": INT := {v};"))
            }),
            "{name} := {v} missing in GVL_Cell"
        );
    }
    // 2 robots x 2 zones, both robots on V1 / V11.
    assert!(gvl.contains("ARRAY[1..4] OF INT := [1,11,1,11]"), "{gvl}");
    // The sim profile keeps PROFILE_VW FALSE and an all-zero table.
    let g = generated(WELDING, &Config::default());
    let gvl = g.output.get("codesys/head/GVL/GVL_Cell.st").unwrap();
    assert!(gvl.contains("PROFILE_VW              : BOOL := FALSE;"));
    assert!(gvl.contains("ARRAY[1..4] OF INT := [0,0,0,0]"));
}

#[test]
fn program_json_shares_the_interlock_map_with_bt2krl() {
    let g = generated(WELDING, &vw_config(&[("IL01", 1), ("IL02", 11)]));
    for ns in ["robot1", "robot2"] {
        let p: Value =
            serde_json::from_str(g.output.get(&format!("robots/{ns}/program.json")).unwrap())
                .unwrap();
        assert_eq!(p["protocol"]["profile"], "vw-ks24");
        assert_eq!(p["protocol"]["job_number"], "byte");
        let zones = p["zones"].as_array().unwrap();
        assert_eq!(zones[0]["name"], "IL01");
        assert_eq!(zones[0]["interlock"], 1);
        assert_eq!(zones[1]["interlock"], 11);
    }
    // Without a configured map the zones a robot uses are numbered in order.
    let g = generated(WELDING, &vw_config(&[]));
    let p: Value =
        serde_json::from_str(g.output.get("robots/robot1/program.json").unwrap()).unwrap();
    assert_eq!(p["zones"][0]["interlock"], 1);
    assert_eq!(p["zones"][1]["interlock"], 2);
    // In profile sim there is no interlock field at all.
    let g = generated(WELDING, &Config::default());
    let p: Value =
        serde_json::from_str(g.output.get("robots/robot1/program.json").unwrap()).unwrap();
    assert!(p["zones"][0].get("interlock").is_none());
}

#[test]
fn the_vw_head_st_follows_the_dialect_rules() {
    let g = generated(WELDING, &vw_config(&[("IL01", 1), ("IL02", 11)]));
    for (path, text) in g.output.files() {
        if !path.starts_with("codesys/") || !path.ends_with(".st") {
            continue;
        }
        assert!(text.is_ascii(), "{path}: non-ASCII");
        assert!(!text.contains('\t'), "{path}: tab");
        let code = strip_comments(text);
        assert!(!code.contains("//"), "{path}: // comment");
        // Rule 2: never write an element of a GVL array.
        for l in code.lines() {
            let l = l.trim();
            // an assignment statement: "<lhs> := ..." with nothing else before it
            let Some((lhs, _)) = l.split_once(":=") else {
                continue;
            };
            let lhs = lhs.trim();
            if lhs.starts_with("GVL_") && lhs.contains('[') {
                panic!("{path}: writes a GVL array element: {l}");
            }
        }
    }
}

#[test]
fn the_vw_profile_rejects_what_it_cannot_express() {
    // The permit bit does not exist in this profile: max_zones must cover every dependency.
    let mut c = vw_config(&[]);
    c.protocol.max_zones = 17;
    assert!(c.validate().is_err());
    // word_le job numbers do not fit PRG_NR.
    let mut c = vw_config(&[]);
    c.cell.job_number = JobNumber::WordLe;
    assert!(c.validate().is_err());
    // Folge numbers must fit 8 bits.
    let mut c = vw_config(&[]);
    c.cell.prod_folge = 300;
    assert!(c.validate().is_err());
}

#[test]
fn a_plan_that_needs_the_permit_bit_warns_in_this_profile() {
    // 16 zones is the VW maximum; a tree with more cross-robot chains falls back to the permit
    // bit, which vw-ks24 cannot carry. bt2plc still plans it (and warns); bt2krl refuses it.
    let mut t = Tree::default();
    let mut branches = Vec::new();
    for r in 0..2 {
        let mut ops = Vec::new();
        for i in 1..=20 {
            if r == 1 {
                ops.push(t.wait("r1", &[("r0", &format!("A{i}"))]));
            }
            ops.push(t.exec(
                &format!("r{r}"),
                &format!("{}{i}", if r == 0 { "A" } else { "B" }),
                0.3,
            ));
        }
        branches.push(seq(ops));
    }
    let root = seq(vec![par(branches), par(vec![t.home("r0"), t.home("r1")])]);
    let mut c = Config::default();
    c.protocol.profile = Profile::VwKs24;
    c.protocol.max_zones = 16;
    c.protocol.plan_id_byte = 0;
    c.cell.job_number = JobNumber::Byte;
    let (plan, d) = plan_json(&t.envelope(root), &c).unwrap();
    assert!(
        !plan.permit_edges.is_empty(),
        "the tree must overflow 16 zones"
    );
    assert!(
        d.items().iter().any(|x| x.code == bt2plc::DiagCode::W021),
        "the permit overflow must warn: {:?}",
        d.items()
    );
}
