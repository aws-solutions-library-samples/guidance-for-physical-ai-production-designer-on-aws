//! Lowering, validation errors and the zone/permit construction on hand-made trees.

mod common;
use bt2plc::Config;
use common::*;

fn err(text: &str) -> String {
    match plan_json(text, &Config::default()) {
        Ok(_) => panic!("expected an error"),
        Err(e) => e.to_string(),
    }
}

#[test]
fn same_robot_in_two_parallel_branches_is_rejected() {
    let mut t = Tree::default();
    let root = par(vec![t.exec("r1", "A", 1.0), t.exec("r1", "B", 1.0)]);
    assert!(err(&t.envelope(root)).contains("E004"));
}

#[test]
fn partial_parallel_thresholds_are_rejected() {
    let mut t = Tree::default();
    let mut root = par(vec![t.exec("r1", "A", 1.0), t.exec("r2", "B", 1.0)]);
    root["ports"]["success_count"] = 1.into();
    assert!(err(&t.envelope(root)).contains("E003"));
}

#[test]
fn unsupported_nodes_report_the_path() {
    let mut t = Tree::default();
    let root = seq(vec![
        serde_json::json!({"type": "Fallback", "ports": {}, "children": [t.exec("r1", "A", 1.0)]}),
    ]);
    let e = err(&t.envelope(root));
    assert!(
        e.contains("E002") && e.contains("/Sequence/Fallback[0]"),
        "{e}"
    );
}

#[test]
fn dangling_and_cyclic_gates_are_rejected() {
    let mut t = Tree::default();
    let root = par(vec![
        seq(vec![t.exec("r1", "A", 1.0)]),
        seq(vec![
            t.wait("r2", &[("r1", "NOPE")]),
            t.exec("r2", "B", 1.0),
        ]),
    ]);
    assert!(err(&t.envelope(root)).contains("E006"));
    let mut t = Tree::default();
    let root = par(vec![
        seq(vec![t.wait("r1", &[("r2", "B")]), t.exec("r1", "A", 1.0)]),
        seq(vec![t.wait("r2", &[("r1", "A")]), t.exec("r2", "B", 1.0)]),
    ]);
    assert!(err(&t.envelope(root)).contains("E005"));
}

#[test]
fn joint_gates_are_errors_unless_explicitly_ignored() {
    let mut t = Tree::default();
    let gate = serde_json::json!({"type": "WaitForRobot", "ports": {"robot_ns": "r2",
        "depends_on_joints": [{"robot": "r1", "joint_positions": [0,0,0,0,0,0]}]}});
    let root = seq(vec![t.exec("r1", "A", 1.0), gate, t.exec("r2", "B", 1.0)]);
    let text = t.envelope(root);
    assert!(err(&text).contains("E019"));
    let mut cfg = Config::default();
    cfg.cell.ignore_unsupported_gates = true;
    let (_, d) = plan_json(&text, &cfg).unwrap();
    assert!(d.items().iter().any(|x| x.code == bt2plc::DiagCode::W001));
}

#[test]
fn coordinated_gates_become_zones_and_respect_hb() {
    // r2 waits for r1.A before B; r1 waits for r2.B before C.
    let mut t = Tree::default();
    let root = seq(vec![
        par(vec![
            seq(vec![
                t.exec("r1", "A", 2.0),
                t.wait("r1", &[("r2", "B")]),
                t.exec("r1", "C", 1.0),
            ]),
            seq(vec![
                t.exec("r2", "X", 1.0),
                t.wait("r2", &[("r1", "A")]),
                t.exec("r2", "B", 1.0),
            ]),
        ]),
        par(vec![t.home("r1"), t.home("r2")]),
    ]);
    let (p, _) = plan_json(&t.envelope(root), &Config::default()).unwrap();
    let zs = zone_strings(&p);
    assert!(
        zs.iter().any(|z| z.contains("r1.A > r2.B > r1.C")),
        "{zs:?}"
    );
    // r1.A and r2.X are BT-concurrent: never in one zone.
    let a = p.ops.iter().position(|o| o.operation_id == "A").unwrap();
    let x = p.ops.iter().position(|o| o.operation_id == "X").unwrap();
    assert!(p.hb.concurrent(a, x));
    assert_eq!(p.ops[a].zone_mask & p.ops[x].zone_mask, 0);
}

#[test]
fn leader_follower_overflow_uses_the_permit_bit() {
    // Algorithm review B1: 40 pairwise incomparable gate edges.
    let mut t = Tree::default();
    let leader: Vec<_> = (1..=40)
        .map(|i| t.exec("r1", &format!("A{i}"), 0.5))
        .collect();
    let mut follower = Vec::new();
    for i in 1..=40 {
        follower.push(t.wait("r2", &[("r1", &format!("A{i}"))]));
        follower.push(t.exec("r2", &format!("B{i}"), 0.5));
    }
    let root = par(vec![seq(leader), seq(follower)]);
    let (p, d) = plan_json(&t.envelope(root), &Config::default()).unwrap();
    assert_eq!(p.cross_edges.len(), 40);
    assert_eq!(p.zones.len(), 31);
    assert_eq!(p.permit_edges.len(), 9);
    assert!(d.items().iter().any(|x| x.code == bt2plc::DiagCode::W021));
    for e in &p.permit_edges {
        assert_ne!(p.ops[e.to].zone_mask & bt2plc::ir::PERMIT_BIT, 0);
    }
}

#[test]
fn three_robot_lockstep_prunes_transitive_edges() {
    let mut t = Tree::default();
    let root = seq(vec![
        par(vec![
            t.exec("r1", "A1", 1.0),
            t.exec("r2", "B1", 1.0),
            t.exec("r3", "C1", 1.0),
        ]),
        t.exec("r1", "A2", 1.0),
        t.exec("r2", "B2", 1.0),
        t.exec("r3", "C2", 1.0),
    ]);
    let (p, _) = plan_json(&t.envelope(root), &Config::default()).unwrap();
    // r3.C2 follows r2.B2 which follows r1.A2: the direct A2 -> C2 edge is implied.
    let s: Vec<(String, String)> = p
        .cross_edges
        .iter()
        .map(|e| (p.op_ref(e.from), p.op_ref(e.to)))
        .collect();
    assert!(!s.contains(&("r1.A2".into(), "r3.C2".into())), "{s:?}");
    assert!(s.contains(&("r2.B2".into(), "r3.C2".into())));
}

#[test]
fn inexact_delay_paths_warn() {
    let mut t = Tree::default();
    // r1.X -> Delay(500){ Wait(r2.Y) } -> r1.Z: one zero-delay path, one 500 ms path.
    let gate = t.wait("r1", &[("r2", "Y")]);
    let root = par(vec![
        seq(vec![
            t.exec("r1", "X", 1.0),
            delay(500, gate),
            t.exec("r1", "Z", 1.0),
        ]),
        t.exec("r2", "Y", 1.0),
    ]);
    let (p, d) = plan_json(&t.envelope(root), &Config::default()).unwrap();
    let z = p.ops.iter().find(|o| o.operation_id == "Z").unwrap();
    assert_eq!(z.start_delay_ms, 500);
    assert!(!z.delay_exact);
    assert!(d.items().iter().any(|x| x.code == bt2plc::DiagCode::W007));
}

#[test]
fn home_ids_do_not_collide_with_execute_ids() {
    let mut t = Tree::default();
    let root = seq(vec![t.exec("r1", "HOME", 1.0), t.home("r1")]);
    let (p, _) = plan_json(&t.envelope(root), &Config::default()).unwrap();
    let ids: Vec<&str> = p.ops.iter().map(|o| o.operation_id.as_str()).collect();
    assert_eq!(ids, ["HOME", "HOME_"]);
}

#[test]
fn inexact_delay_deadlines_use_the_plc_schedule() {
    // Review finding: Delay(100 s) before a gate on a 100 s op gave a 16.7 s zone deadline.
    let mut t = Tree::default();
    let gate = t.wait("r1", &[("r2", "Y")]);
    let root = par(vec![
        seq(vec![
            t.exec("r1", "X", 1.0),
            delay(100_000, gate),
            t.exec("r1", "Z", 1.0),
        ]),
        seq(vec![t.exec("r2", "Y", 100.0)]),
    ]);
    let (p, _) = plan_json(&t.envelope(root), &Config::default()).unwrap();
    let z = p.ops.iter().find(|o| o.operation_id == "Z").unwrap();
    assert_eq!(
        z.asap_start_ms, 100_500,
        "Z is granted when Y (100 s + 0.5 s hold) is done"
    );
    assert_eq!(z.asap_end_ms, 100_500 + 100_000 + 1_500);
    assert!(
        z.zone_deadline_ms > z.asap_start_ms * 3 / 2,
        "deadline {} too small",
        z.zone_deadline_ms
    );
    assert_eq!(p.makespan_ms, z.asap_end_ms);
}

#[test]
fn plan_id_depends_on_blackboard_and_config() {
    let mut t = Tree::default();
    let root = seq(vec![t.exec("r1", "A", 1.0)]);
    let text = t.envelope(root);
    let (a, _) = plan_json(&text, &Config::default()).unwrap();
    let mut t2 = Tree::default();
    let root2 = seq(vec![t2.exec("r1", "A", 2.0)]);
    let text2 = t2.envelope(root2);
    let (b, _) = plan_json(&text2, &Config::default()).unwrap();
    assert_ne!(a.plan_id, b.plan_id);
    let mut cfg = Config::default();
    cfg.robot_sim.at_home_tol_deg = 0.25;
    let (c, _) = plan_json(&text, &cfg).unwrap();
    assert_ne!(a.plan_id, c.plan_id);
    // Same input + config -> same id.
    let (d, _) = plan_json(&text, &Config::default()).unwrap();
    assert_eq!(a.plan_id, d.plan_id);
}
