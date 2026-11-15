//! The hand-worked welding sample (DESIGN §3.9) and the single-robot sample.

mod common;
use common::*;

#[test]
fn welding_sample_gives_the_two_hand_worked_zones() {
    let p = plan_fixture(WELDING);
    assert_eq!(p.robots.len(), 2);
    assert_eq!(p.ops.len(), 18);
    assert_eq!(p.cross_edges.len(), 13);
    assert_eq!(
        zone_strings(&p),
        vec![
            "IL01 = robot1.W01 > robot2.W15 > robot1.W03 > robot2.W10 > robot1.W05 > robot2.W12 > robot1.R1_PARK > robot2.W09",
            "IL02 = robot2.W13 > robot1.W02 > robot2.W14 > robot1.W06 > robot2.W11 > robot1.W04 > robot2.W07 > robot1.HOME",
        ]
    );
    assert!(p.permit_edges.is_empty());
    assert_eq!(p.makespan_ms, 64297);
    let r1: Vec<String> = p.robot_ops(0).map(|o| o.operation_id.clone()).collect();
    assert_eq!(
        r1,
        ["W01", "W02", "W03", "W06", "W05", "W04", "R1_PARK", "HOME"]
    );
    let r2: Vec<String> = p.robot_ops(1).map(|o| o.operation_id.clone()).collect();
    assert_eq!(
        r2,
        [
            "W13", "W15", "W14", "W10", "W11", "W12", "W09", "W08", "W07", "HOME"
        ]
    );
    // Parallel pairs are never in one zone; W08 (robot2 alone) needs no zone.
    let w08 = p.ops.iter().find(|o| o.operation_id == "W08").unwrap();
    assert_eq!(w08.zone_mask, 0);
}

#[test]
fn welding_timing_matches_the_design_tables() {
    let p = plan_fixture(WELDING);
    let w01 = p.ops.iter().find(|o| o.operation_id == "W01").unwrap();
    assert_eq!(w01.expected_ms, 11132);
    assert_eq!(w01.limit_ms, 21698);
    let w02 = p.ops.iter().find(|o| o.operation_id == "W02").unwrap();
    assert_eq!(w02.asap_start_ms, 11132);
}

#[test]
fn single_robot_sample_has_no_zones() {
    let p = plan_fixture(TWO_ROBOT);
    assert_eq!(p.robots.len(), 1);
    assert_eq!(p.ops.len(), 2);
    assert!(p.zones.is_empty() && p.cross_edges.is_empty());
}

#[test]
fn json_and_xml_lower_to_the_same_plan() {
    for f in [WELDING, TWO_ROBOT] {
        let text = fixture(f);
        let cfg = bt2plc::Config::default();
        let mut d = bt2plc::Diagnostics::default();
        let j = bt2plc::input::load_str(&text, f, bt2plc::input::SourceSel::Json, None, &mut d)
            .unwrap();
        let x =
            bt2plc::input::load_str(&text, f, bt2plc::input::SourceSel::Xml, None, &mut d).unwrap();
        let pj = bt2plc::plan(&j, &cfg, &mut d).unwrap();
        let px = bt2plc::plan(&x, &cfg, &mut d).unwrap();
        assert_eq!(zone_strings(&pj), zone_strings(&px), "{f}");
        assert_eq!(pj.makespan_ms, px.makespan_ms, "{f}");
    }
}
