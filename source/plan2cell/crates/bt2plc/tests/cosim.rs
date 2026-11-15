//! Co-simulation of the generated head-PLC ST with the generated robot simulators
//! (trust-runtime). Run with `cargo test --features sim --test cosim -- --nocapture`.

mod common;
use bt2plc::ir::CellPlan;
use bt2plc::trust::cosim::{EvKind, OpRun, Report, SimOptions, run};
use bt2plc::{Config, GenerateOptions};
use common::*;

fn generate(name: &str, cfg: &Config) -> bt2plc::Generated {
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

/// Plan op `op` of robot `ns`.
fn op_id(plan: &CellPlan, ns: &str, op: &str) -> usize {
    plan.ops
        .iter()
        .find(|o| plan.robots[o.robot].ns == ns && o.operation_id == op)
        .unwrap_or_else(|| panic!("{ns}.{op}"))
        .id
}

fn run_of<'a>(rep: &'a Report, plan: &CellPlan, cycle: u32, ns: &str, op: &str) -> &'a OpRun {
    rep.run_of(plan, cycle, op_id(plan, ns, op))
        .unwrap_or_else(|| panic!("cycle {cycle}: {ns}.{op} never ran"))
}

fn verify_or_dump(rep: &Report, plan: &CellPlan) {
    if let Err(e) = rep.verify(plan) {
        for ev in rep
            .events
            .iter()
            .rev()
            .take(60)
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
        {
            println!("{ev}");
        }
        panic!("{e}");
    }
}

#[test]
fn welding_cell_runs_two_cycles_respecting_every_dependency() {
    let cfg = Config::default();
    let g = generate(WELDING, &cfg);
    let rep = run(
        &g.plan,
        &cfg,
        &g.projects,
        &SimOptions {
            cycles: 2,
            ..Default::default()
        },
    )
    .unwrap();
    println!("{}", rep.summary());
    verify_or_dump(&rep, &g.plan);
    assert_eq!(rep.cycle_end_ms.len(), 2);
    // A correct run never moves a robot without its grant.
    assert!(
        rep.motion_violations.is_empty(),
        "{:?}",
        rep.motion_violations
    );
    // The six Parallel weld pairs and the final Parallel homing really run together.
    let pairs = [
        ("W01", "W13"),
        ("W02", "W15"),
        ("W03", "W14"),
        ("W06", "W10"),
        ("W05", "W11"),
        ("W04", "W12"),
        ("HOME", "HOME"),
    ];
    for cycle in 1..=2 {
        for (a, b) in pairs {
            let ov = rep
                .overlap_ms(
                    &g.plan,
                    cycle,
                    op_id(&g.plan, "robot1", a),
                    op_id(&g.plan, "robot2", b),
                )
                .unwrap();
            assert!(
                ov > 1000,
                "cycle {cycle}: robot1.{a} || robot2.{b} overlap only {ov} ms"
            );
        }
    }
    // Cycle time close to the planned makespan (+ handshake latency per dependency hop).
    for (s, e) in rep.cycle_start_ms.iter().zip(&rep.cycle_end_ms) {
        let dur = e - s;
        assert!(
            dur < g.plan.makespan_ms + 15_000,
            "cycle took {dur} ms, plan {} ms",
            g.plan.makespan_ms
        );
    }
}

#[test]
fn single_robot_sample_runs() {
    let cfg = Config::default();
    let g = generate(TWO_ROBOT, &cfg);
    let rep = run(&g.plan, &cfg, &g.projects, &SimOptions::default()).unwrap();
    println!("{}", rep.summary());
    verify_or_dump(&rep, &g.plan);
    assert_eq!(rep.cycle_end_ms.len(), 1);
}

#[test]
fn robot_error_mid_cycle_aborts_then_recovers_and_completes_a_cycle() {
    let cfg = Config::default();
    let g = generate(WELDING, &cfg);
    let opts = SimOptions {
        cycles: 1,
        inject_error: Some((0, 20_000)),
        reset_at_ms: Some(25_000),
        ..Default::default()
    };
    let rep = run(&g.plan, &cfg, &g.projects, &opts).unwrap();
    println!("{}", rep.summary());
    for ev in rep.events.iter().filter(|e| e.robot == usize::MAX) {
        println!("{ev}");
    }
    assert!(rep.runtime_errors.is_empty(), "{:?}", rep.runtime_errors);
    assert!(
        rep.exclusion_violations.is_empty(),
        "{:?}",
        rep.exclusion_violations
    );
    // The first fault is the injected one: robot 1 (PLC numbering), not before the injection.
    let first = rep
        .cell_errors
        .iter()
        .find(|e| !e.error.contains("ERR_NONE"))
        .expect("the injected robot error must stop the cell");
    assert!(
        first.error.contains("ERR_ROBOT_ERROR"),
        "{:?}",
        rep.cell_errors
    );
    assert!(first.t_ms >= 20_000, "{first:?}");
    assert_eq!(first.robot, 1, "{first:?}");
    let states: Vec<String> = rep
        .events
        .iter()
        .filter_map(|e| match &e.kind {
            EvKind::CellState(s) => Some(s.clone()),
            _ => None,
        })
        .collect();
    assert!(
        states.iter().any(|s| s.contains("CS_RECOVERY")),
        "{states:?}"
    );
    // The interrupted cycle is discarded; the completed one is the cycle after the recovery.
    assert_eq!(rep.aborted_cycles.len(), 1, "{:?}", rep.aborted_cycles);
    assert!(!rep.aborted_runs.is_empty());
    assert_eq!(
        rep.cycle_end_ms.len(),
        1,
        "a full cycle must complete after recovery: {states:?}"
    );
    assert!(
        rep.cycle_start_ms[0] > 27_000,
        "cycle 1 must start after the recovery confirm: {:?}",
        rep.cycle_start_ms
    );
    verify_or_dump(&rep, &g.plan);
}

#[test]
fn coordinated_tree_with_permit_overflow_runs() {
    // 33 leader/follower gates: 31 zones + 2 permit edges (A32 -> B32, A33 -> B33); exercises
    // the permit bit end to end. The permit predecessors are long, so a follower that ignored
    // the permit would start B32 / B33 long before A32 / A33 end.
    let mut t = Tree::default();
    let leader: Vec<_> = (1..=33)
        .map(|i| t.exec("r1", &format!("A{i}"), if i >= 32 { 3.0 } else { 0.3 }))
        .collect();
    let mut follower = Vec::new();
    for i in 1..=33 {
        follower.push(t.wait("r2", &[("r1", &format!("A{i}"))]));
        follower.push(t.exec("r2", &format!("B{i}"), 0.3));
    }
    let root = seq(vec![
        par(vec![seq(leader), seq(follower)]),
        par(vec![t.home("r1"), t.home("r2")]),
    ]);
    let text = t.envelope(root);
    let cfg = Config::default();
    let mut d = bt2plc::Diagnostics::default();
    let doc = bt2plc::input::load_str(
        &text,
        "t.json",
        bt2plc::input::SourceSel::Auto,
        None,
        &mut d,
    )
    .unwrap();
    let g = bt2plc::generate(
        &doc,
        &cfg,
        &GenerateOptions {
            st_check: false,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(g.plan.zones.len(), 31);
    assert_eq!(g.plan.permit_edges.len(), 2);
    let permit: Vec<(String, String)> = g
        .plan
        .permit_edges
        .iter()
        .map(|e| (g.plan.op_ref(e.from), g.plan.op_ref(e.to)))
        .collect();
    assert_eq!(
        permit,
        [
            ("r1.A32".to_string(), "r2.B32".to_string()),
            ("r1.A33".into(), "r2.B33".into())
        ]
    );
    let rep = run(
        &g.plan,
        &cfg,
        &g.projects,
        &SimOptions {
            cycles: 2,
            ..Default::default()
        },
    )
    .unwrap();
    println!("{}", rep.summary());
    verify_or_dump(&rep, &g.plan);
    assert_eq!(rep.cycle_end_ms.len(), 2);
    for cycle in 1..=2 {
        for i in [32, 33] {
            let a = run_of(&rep, &g.plan, cycle, "r1", &format!("A{i}"));
            let b = run_of(&rep, &g.plan, cycle, "r2", &format!("B{i}"));
            let prev = run_of(&rep, &g.plan, cycle, "r2", &format!("B{}", i - 1));
            assert!(
                b.start_ms >= a.end_ms,
                "cycle {cycle}: B{i} started at {} ms before A{i} ended at {} ms",
                b.start_ms,
                a.end_ms
            );
            // Only the permit holds B(i) back: the follower was free long before A(i) ended.
            assert!(
                prev.end_ms + 1_500 < a.end_ms,
                "cycle {cycle}: B{} ended at {} ms, A{i} at {} ms",
                i - 1,
                prev.end_ms,
                a.end_ms
            );
        }
    }
}

#[test]
fn motion_without_grant_is_detected() {
    // robot2's simulator gets its own REQUEST as GRANT from 6 s on (during W13): it then moves
    // into W15 (zone IL01) while robot1 still holds IL01 with W01 and the head grants nothing.
    let cfg = Config::default();
    let g = generate(WELDING, &cfg);
    let r2 = g.plan.robots.iter().position(|r| r.ns == "robot2").unwrap();
    let opts = SimOptions {
        bypass_grant: Some((r2, 6_000)),
        max_time_s: 15.0,
        ..Default::default()
    };
    let rep = run(&g.plan, &cfg, &g.projects, &opts).unwrap();
    println!("{}", rep.summary());
    let first = rep
        .motion_violations
        .first()
        .unwrap_or_else(|| panic!("motion without GRANT not detected: {}", rep.summary()));
    println!("{first} ({} violations)", rep.motion_violations.len());
    assert!(
        first.contains("R2 ") && first.contains("robot2.W15"),
        "{:?}",
        rep.motion_violations
    );
    assert!(rep.verify(&g.plan).is_err());
}
