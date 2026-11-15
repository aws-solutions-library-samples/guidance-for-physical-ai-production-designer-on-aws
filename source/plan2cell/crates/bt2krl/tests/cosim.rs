//! Co-simulation: the bt2krl IR executor (the behaviour of the generated KRL) against the head
//! PLC that bt2plc generates, through `bt2plc::trust::cosim::RobotModel`.
//! Run with `cargo test --features cosim --test cosim -- --nocapture`.
#![cfg(feature = "cosim")]

mod common;
use bt2krl::exec::Executor;
use bt2krl::program::Program;
use bt2krl::{Config, Options};
use bt2plc::ir::CellPlan;
use bt2plc::trust::cosim::{EvKind, Report, RobotModel, RobotModels, SimOptions, run_with};
use bt2plc::{GenerateOptions, Generated};
use common::*;

/// The executor as a cosim device. The generated KRL and this model come from the same IR.
struct KrlRobot(Executor);

impl RobotModel for KrlRobot {
    fn scan(&mut self, dt_ms: u64, cmd: &[u8; 32], safe_out: &[u8; 8]) -> ([u8; 32], [u8; 8]) {
        self.0.scan(dt_ms, cmd, safe_out)
    }

    fn inject_error(&mut self, on: bool) {
        self.0.inject_error(on);
    }
}

/// bt2plc head PLC without robot simulators (every robot is a bt2krl executor).
fn head_only(text: &str) -> (Generated, bt2plc::Config) {
    head_only_with(text, |_| {})
}

fn head_only_with(
    text: &str,
    tweak: impl FnOnce(&mut bt2plc::Config),
) -> (Generated, bt2plc::Config) {
    let mut cfg = bt2plc::Config::default();
    cfg.robot_sim.enabled = false;
    tweak(&mut cfg);
    let mut d = bt2plc::Diagnostics::default();
    let doc = bt2plc::input::load_str(text, "t.json", bt2plc::input::SourceSel::Auto, None, &mut d)
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
    (g, cfg)
}

/// program.json of every robot, and the KRL generated from it (so the KRL the validator and the
/// round trip accepted is the same IR the executors run).
fn models(g: &Generated, cfg: &Config) -> (Vec<Program>, RobotModels) {
    let programs: Vec<Program> = g
        .plan
        .robots
        .iter()
        .map(|r| {
            let f = format!("robots/{}/program.json", r.ns);
            Program::from_str(g.output.get(&f).unwrap(), &f).unwrap()
        })
        .collect();
    bt2krl::generate(&programs, cfg, &Options::default()).expect("bt2krl generates the KRL");
    let m: RobotModels = programs
        .iter()
        .enumerate()
        .map(|(ri, p)| {
            let e = bt2krl::exec::executor(p, cfg).unwrap();
            (ri, Box::new(KrlRobot(e)) as Box<dyn RobotModel>)
        })
        .collect();
    (programs, m)
}

fn op_id(plan: &CellPlan, ns: &str, op: &str) -> usize {
    plan.ops
        .iter()
        .find(|o| plan.robots[o.robot].ns == ns && o.operation_id == op)
        .unwrap_or_else(|| panic!("{ns}.{op}"))
        .id
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

fn plan_text(sample: &str) -> String {
    std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../test_data")
            .join(sample),
    )
    .unwrap()
}

const WELDING: &str = "bt_1790934549_welding_cell_2_small.json";
const TWO_ROBOT: &str = "bt_1791274820_cell_2robot.json";

#[test]
fn welding_cell_runs_two_cycles_against_the_head_plc() {
    let (g, plc) = head_only(&plan_text(WELDING));
    let (_p, m) = models(&g, &test_cfg());
    let rep = run_with(
        &g.plan,
        &plc,
        &g.projects,
        &SimOptions {
            cycles: 2,
            ..Default::default()
        },
        m,
    )
    .unwrap();
    println!("{}", rep.summary());
    verify_or_dump(&rep, &g.plan);
    assert_eq!(rep.cycle_end_ms.len(), 2);
    assert!(
        rep.motion_violations.is_empty(),
        "{:?}",
        rep.motion_violations
    );
    assert!(
        rep.exclusion_violations.is_empty(),
        "{:?}",
        rep.exclusion_violations
    );
    assert!(
        rep.cell_errors.iter().all(|e| e.error.contains("ERR_NONE")),
        "{:?}",
        rep.cell_errors
    );
    // The BT's six Parallel weld pairs and the final homing really run together.
    for (a, b) in [
        ("W01", "W13"),
        ("W02", "W15"),
        ("W03", "W14"),
        ("W06", "W10"),
        ("W05", "W11"),
        ("W04", "W12"),
        ("HOME", "HOME"),
    ] {
        for cycle in 1..=2 {
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
                "cycle {cycle}: robot1.{a} || robot2.{b} overlap {ov} ms"
            );
        }
    }
    for (s, e) in rep.cycle_start_ms.iter().zip(&rep.cycle_end_ms) {
        assert!(
            e - s < g.plan.makespan_ms + 15_000,
            "cycle took {} ms",
            e - s
        );
    }
}

#[test]
fn single_robot_sample_runs() {
    let (g, plc) = head_only(&plan_text(TWO_ROBOT));
    let (_p, m) = models(&g, &test_cfg());
    let rep = run_with(&g.plan, &plc, &g.projects, &SimOptions::default(), m).unwrap();
    println!("{}", rep.summary());
    verify_or_dump(&rep, &g.plan);
    assert_eq!(rep.cycle_end_ms.len(), 1);
}

#[test]
fn permit_overflow_holds_the_follower_back() {
    // Leader/follower gates beyond the zone capacity: the last two edges use the permit bit.
    let mut t = Tree {
        seed: 7,
        ..Default::default()
    };
    let n = 6;
    let leader: Vec<_> = (1..=n)
        .map(|i| t.exec("r1", &format!("A{i}"), if i >= n - 1 { 3.0 } else { 0.3 }))
        .collect();
    let mut follower = Vec::new();
    for i in 1..=n {
        follower.push(t.wait("r2", &[("r1", &format!("A{i}"))]));
        follower.push(t.exec("r2", &format!("B{i}"), 0.3));
    }
    let root = seq(vec![
        par(vec![seq(leader), seq(follower)]),
        par(vec![t.home("r1"), t.home("r2")]),
    ]);
    let text = t.envelope(root);
    let mut plc = bt2plc::Config::default();
    plc.robot_sim.enabled = false;
    plc.protocol.max_zones = n as u8 - 2;
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
        &plc,
        &GenerateOptions {
            st_check: false,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(g.plan.permit_edges.len(), 2, "{:?}", g.plan.permit_edges);
    let cfg = cfg_for(&["r1", "r2"]);
    let (_p, m) = models(&g, &cfg);
    let rep = run_with(
        &g.plan,
        &plc,
        &g.projects,
        &SimOptions {
            cycles: 1,
            ..Default::default()
        },
        m,
    )
    .unwrap();
    println!("{}", rep.summary());
    verify_or_dump(&rep, &g.plan);
    for i in [n - 1, n] {
        let a = rep
            .run_of(&g.plan, 1, op_id(&g.plan, "r1", &format!("A{i}")))
            .unwrap();
        let b = rep
            .run_of(&g.plan, 1, op_id(&g.plan, "r2", &format!("B{i}")))
            .unwrap();
        let prev = rep
            .run_of(&g.plan, 1, op_id(&g.plan, "r2", &format!("B{}", i - 1)))
            .unwrap();
        assert!(b.start_ms >= a.end_ms, "B{i} started before A{i} ended");
        // Only the permit held B(i) back: the follower was free long before A(i) ended.
        assert!(
            prev.end_ms + 1_000 < a.end_ms,
            "B{} ended at {} ms, A{i} at {} ms",
            i - 1,
            prev.end_ms,
            a.end_ms
        );
    }
}

#[test]
fn robot_error_aborts_then_recovers_and_completes_a_cycle() {
    let (g, plc) = head_only(&plan_text(WELDING));
    let (_p, m) = models(&g, &test_cfg());
    let rep = run_with(
        &g.plan,
        &plc,
        &g.projects,
        &SimOptions {
            cycles: 1,
            inject_error: Some((0, 20_000)),
            reset_at_ms: Some(25_000),
            ..Default::default()
        },
        m,
    )
    .unwrap();
    println!("{}", rep.summary());
    assert!(rep.runtime_errors.is_empty(), "{:?}", rep.runtime_errors);
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
    assert_eq!(rep.aborted_cycles.len(), 1, "{:?}", rep.aborted_cycles);
    // The robot released its zones and stopped without ProgFinished (no ERR_RELEASE_INVALID,
    // no ERR_STOP_TIMEOUT), and a full cycle runs after the recovery.
    assert_eq!(rep.cycle_end_ms.len(), 1, "{states:?}");
    assert!(rep.cycle_start_ms[0] > 27_000, "{:?}", rep.cycle_start_ms);
    verify_or_dump(&rep, &g.plan);
}

#[test]
fn a_wrong_plan_id_is_refused() {
    // The KRL checks PLAN_ID at the Folge start: a robot generated from another plan must not
    // run (error 16#0103, no ProgRunning), and the head must report a robot error.
    let (g, plc) = head_only(&plan_text(WELDING));
    let cfg = test_cfg();
    let (programs, _) = models(&g, &cfg);
    let mut m: RobotModels = Vec::new();
    for (ri, p) in programs.iter().enumerate() {
        let mut p = p.clone();
        if ri == 1 {
            p.plan.plan_id = "16#0042".into();
        }
        let e = bt2krl::exec::executor(&p, &cfg).unwrap();
        m.push((ri, Box::new(KrlRobot(e)) as Box<dyn RobotModel>));
    }
    let rep = run_with(
        &g.plan,
        &plc,
        &g.projects,
        &SimOptions {
            cycles: 1,
            max_time_s: 60.0,
            ..Default::default()
        },
        m,
    )
    .unwrap();
    println!("{}", rep.summary());
    assert_eq!(
        rep.cycle_end_ms.len(),
        0,
        "a plan mismatch must not run a cycle"
    );
    assert!(
        rep.cell_errors
            .iter()
            .any(|e| e.error.contains("ERR_ROBOT_ERROR")),
        "{:?}",
        rep.cell_errors
    );
    // robot2 never reported a step.
    assert!(
        !rep.events
            .iter()
            .any(|e| e.robot == 1 && matches!(e.kind, EvKind::Step(s) if s > 0)),
        "robot2 must not run any op"
    );
}

#[test]
fn motion_without_grant_would_be_detected() {
    // The executor receives its own REQUEST as GRANT from 6 s on: it then enters a zone the
    // head never granted, which the co-simulation's motion check must catch. This checks the
    // check (the executor itself never skips a WAIT FOR).
    let (g, plc) = head_only(&plan_text(WELDING));
    let (_p, m) = models(&g, &test_cfg());
    let r2 = g.plan.robots.iter().position(|r| r.ns == "robot2").unwrap();
    let rep = run_with(
        &g.plan,
        &plc,
        &g.projects,
        &SimOptions {
            bypass_grant: Some((r2, 6_000)),
            max_time_s: 20.0,
            ..Default::default()
        },
        m,
    )
    .unwrap();
    println!("{}", rep.summary());
    let first = rep
        .motion_violations
        .first()
        .unwrap_or_else(|| panic!("motion without GRANT not detected: {}", rep.summary()));
    println!("{first}");
    assert!(first.contains("R2 "), "{:?}", rep.motion_violations);
    assert!(rep.verify(&g.plan).is_err());
}

/// The bus as the live cell saw it: at a Folge end the robot's `$OUT` image is taken between
/// two submit statements, so for `gap_scans` frames ProgRunning (byte 0 bit 1) is already low
/// while ProgFinished (byte 1 bit 2) is still the old FALSE.
struct StaleFinish {
    inner: KrlRobot,
    gap_scans: u32,
    left: u32,
    last_run: bool,
    gaps: std::rc::Rc<std::cell::Cell<u32>>,
}

impl RobotModel for StaleFinish {
    fn scan(&mut self, dt_ms: u64, cmd: &[u8; 32], safe_out: &[u8; 8]) -> ([u8; 32], [u8; 8]) {
        let (mut st, safe) = self.inner.scan(dt_ms, cmd, safe_out);
        let run = st[0] & 0b10 != 0;
        let fin = st[1] & 0b100 != 0;
        if self.last_run && !run && fin {
            self.left = self.gap_scans;
            self.gaps.set(self.gaps.get() + 1);
        }
        self.last_run = run;
        if self.left > 0 {
            self.left -= 1;
            st[1] &= !0b100;
        }
        (st, safe)
    }

    fn inject_error(&mut self, on: bool) {
        self.inner.inject_error(on);
    }
}

/// Every robot of `sample` with a stale-ProgFinished gap of `gap_scans` device frames
/// (20 ms each) at every Folge end.
fn stale_finish_run(
    sample: &str,
    stop_debounce_ms: u32,
    gap_scans: u32,
    cycles: u32,
    max_time_s: f64,
) -> (Report, CellPlan, u32) {
    let (g, plc) = head_only_with(&plan_text(sample), |c| {
        c.protocol.stop_debounce_ms = stop_debounce_ms;
    });
    let cfg = test_cfg();
    let (programs, _) = models(&g, &cfg);
    let gaps = std::rc::Rc::new(std::cell::Cell::new(0));
    let m: RobotModels = programs
        .iter()
        .enumerate()
        .map(|(ri, p)| {
            let e = bt2krl::exec::executor(p, &cfg).unwrap();
            let r = StaleFinish {
                inner: KrlRobot(e),
                gap_scans,
                left: 0,
                last_run: false,
                gaps: gaps.clone(),
            };
            (ri, Box::new(r) as Box<dyn RobotModel>)
        })
        .collect();
    let rep = run_with(
        &g.plan,
        &plc,
        &g.projects,
        &SimOptions {
            cycles,
            max_time_s,
            ..Default::default()
        },
        m,
    )
    .unwrap();
    println!("{}", rep.summary());
    (rep, g.plan, gaps.get())
}

#[test]
fn a_one_frame_gap_between_progrunning_and_progfinished_is_a_normal_job_end() {
    // The live incident (CS_ERROR ERR_ROBOT_STOPPED after 67 good cycles, robots without a
    // fault): the head must debounce the gap (T_STOP_DEBOUNCE) and finish the job.
    let (rep, plan, gaps) = stale_finish_run(WELDING, 200, 1, 2, 0.0);
    assert!(gaps >= 4, "the gap was injected only {gaps} times");
    verify_or_dump(&rep, &plan);
    assert_eq!(rep.cycle_end_ms.len(), 2);
    assert!(
        rep.cell_errors.iter().all(|e| e.error.contains("ERR_NONE")),
        "{:?}",
        rep.cell_errors
    );
}

#[test]
fn without_the_stop_debounce_the_gap_reproduces_err_robot_stopped() {
    // T_STOP_DEBOUNCE = 0 is the old head: the same one-frame gap stops the cell.
    let (rep, _plan, gaps) = stale_finish_run(TWO_ROBOT, 0, 1, 1, 25.0);
    assert!(gaps >= 1);
    assert!(
        rep.cell_errors
            .iter()
            .any(|e| e.error.contains("ERR_ROBOT_STOPPED")),
        "{:?}",
        rep.cell_errors
    );
}

#[test]
fn progrunning_low_longer_than_the_debounce_is_still_a_stop() {
    // 20 frames x 20 ms = 400 ms > 200 ms: a real stop without ProgFinished is still reported.
    let (rep, _plan, gaps) = stale_finish_run(TWO_ROBOT, 200, 20, 1, 25.0);
    assert!(gaps >= 1);
    assert!(
        rep.cell_errors
            .iter()
            .any(|e| e.error.contains("ERR_ROBOT_STOPPED")),
        "{:?}",
        rep.cell_errors
    );
}

// ───────────────────────────── profile vw-ks24 (Part B) ─────────────────────────────

/// bt2plc head PLC in profile `vw-ks24` (no robot simulators), with the interlock map that both
/// sides share through program.json.
fn head_only_vw(text: &str, interlocks: &[(&str, u8)]) -> (Generated, bt2plc::Config) {
    let mut cfg = bt2plc::Config::default();
    cfg.robot_sim.enabled = false;
    cfg.protocol.profile = bt2plc::config::Profile::VwKs24;
    cfg.protocol.max_zones = 16;
    cfg.protocol.plan_id_byte = 12; // free PLC -> robot byte on the KS24 robots
    cfg.cell.job_number = bt2plc::config::JobNumber::Byte;
    // The head is generated for this task interval (a realistic cell-PLC cycle) and the
    // co-simulation runs it at exactly that rate: 20 ms instead of 4 ms keeps these runs
    // affordable. The release dwell bt2krl emits (50 ms) still covers a head scan plus a
    // device scan, which is what the VW scheme needs.
    cfg.codesys.main_task_ms = 20;
    let map: std::collections::BTreeMap<String, u8> = interlocks
        .iter()
        .map(|(z, n)| ((*z).to_string(), *n))
        .collect();
    for ns in ["robot1", "robot2", "r1", "r2"] {
        cfg.robots.push(bt2plc::config::RobotCfg {
            ns: ns.into(),
            interlocks: map.clone(),
            ..Default::default()
        });
    }
    let mut d = bt2plc::Diagnostics::default();
    let doc = bt2plc::input::load_str(text, "t.json", bt2plc::input::SourceSel::Auto, None, &mut d)
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
    (g, cfg)
}

fn vw_models(g: &Generated) -> (Vec<Program>, RobotModels) {
    let cfg = vw_cfg_for(g);
    let programs: Vec<Program> = g
        .plan
        .robots
        .iter()
        .map(|r| {
            let f = format!("robots/{}/program.json", r.ns);
            Program::from_str(g.output.get(&f).unwrap(), &f).unwrap()
        })
        .collect();
    // The KRL the executors mirror must pass the validator and the round trip.
    bt2krl::generate(
        &programs,
        &cfg,
        &Options {
            profile: bt2krl::Profile::VwKs24,
            ..Default::default()
        },
    )
    .expect("bt2krl generates the vw-ks24 KRL");
    let m: RobotModels = programs
        .iter()
        .enumerate()
        .map(|(ri, p)| {
            let e = vw_executor(p, &cfg);
            (ri, Box::new(KrlRobot(e)) as Box<dyn RobotModel>)
        })
        .collect();
    (programs, m)
}

/// bt2krl config for the vw profile: every robot KR240 without external axes, PLAN_ID on the
/// same free byte as the head, interlocks from program.json.
fn vw_cfg_for(g: &Generated) -> Config {
    let mut s = String::from("[vw_ks24]\nplan_id_byte = 12\n");
    for r in &g.plan.robots {
        s.push_str(&format!(
            "[[robot]]\nns = \"{}\"\nmodel = \"KR240R2900_ULTRA\"\ne_axes = \"none\"\n",
            r.ns
        ));
    }
    Config::parse(&s).unwrap()
}

fn vw_executor(p: &Program, cfg: &Config) -> Executor {
    let s = bt2krl::lower::RobotSetup::resolve(p, cfg, true).unwrap();
    let l = bt2krl::lower::lower(p, &s, cfg, bt2krl::Profile::VwKs24).unwrap();
    Executor::new(p, &s, cfg, &l).unwrap()
}

#[test]
fn vw_ks24_welding_cell_runs_two_cycles() {
    let (g, plc) = head_only_vw(&plan_text(WELDING), &[("IL01", 1), ("IL02", 11)]);
    assert!(plc.protocol.profile.is_vw());
    let (_p, m) = vw_models(&g);
    let rep = run_with(
        &g.plan,
        &plc,
        &g.projects,
        &SimOptions {
            cycles: 2,
            ..Default::default()
        },
        m,
    )
    .unwrap();
    println!("{}", rep.summary());
    verify_or_dump(&rep, &g.plan);
    assert_eq!(rep.cycle_end_ms.len(), 2);
    assert!(
        rep.motion_violations.is_empty(),
        "{:?}",
        rep.motion_violations
    );
    assert!(
        rep.exclusion_violations.is_empty(),
        "{:?}",
        rep.exclusion_violations
    );
    assert!(
        rep.cell_errors.iter().all(|e| e.error.contains("ERR_NONE")),
        "{:?}",
        rep.cell_errors
    );
    // The BT's Parallel pairs still overlap through the VW interlocks.
    for (a, b) in [("W01", "W13"), ("W03", "W14"), ("W05", "W11")] {
        for cycle in 1..=2 {
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
                "cycle {cycle}: robot1.{a} || robot2.{b} overlap {ov} ms"
            );
        }
    }
}

#[test]
fn vw_ks24_single_robot_sample_runs() {
    let (g, plc) = head_only_vw(&plan_text(TWO_ROBOT), &[]);
    let (_p, m) = vw_models(&g);
    let rep = run_with(&g.plan, &plc, &g.projects, &SimOptions::default(), m).unwrap();
    println!("{}", rep.summary());
    verify_or_dump(&rep, &g.plan);
    assert_eq!(rep.cycle_end_ms.len(), 1);
}

#[test]
fn vw_ks24_robot_fault_needs_manual_homing() {
    // An injected robot fault aborts the cycle. In this profile there is no blind HOME Folge:
    // the head must report that the robot has to be jogged home, not home it itself.
    let (g, plc) = head_only_vw(&plan_text(WELDING), &[("IL01", 1), ("IL02", 11)]);
    let (_p, m) = vw_models(&g);
    let rep = run_with(
        &g.plan,
        &plc,
        &g.projects,
        &SimOptions {
            cycles: 1,
            inject_error: Some((0, 20_000)),
            reset_at_ms: Some(25_000),
            max_time_s: 90.0,
            ..Default::default()
        },
        m,
    )
    .unwrap();
    println!("{}", rep.summary());
    assert!(
        rep.runtime_errors.iter().all(|e| e.contains("timeout")),
        "{:?}",
        rep.runtime_errors
    );
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
    assert_eq!(rep.aborted_cycles.len(), 1, "{:?}", rep.aborted_cycles);
    // The robot never reaches home by itself, so no new cycle starts.
    assert_eq!(rep.cycle_end_ms.len(), 0);
    assert!(
        rep.exclusion_violations.is_empty(),
        "{:?}",
        rep.exclusion_violations
    );
}

#[test]
#[ignore = "diagnostic: short vw-ks24 run with a full event dump"]
fn vw_ks24_trace() {
    let (g, plc) = head_only_vw(&plan_text(WELDING), &[("IL01", 1), ("IL02", 11)]);
    let (_p, m) = vw_models(&g);
    let rep = run_with(
        &g.plan,
        &plc,
        &g.projects,
        &SimOptions {
            cycles: 1,
            max_time_s: 14.0,
            ..Default::default()
        },
        m,
    )
    .unwrap();
    for ev in &rep.events {
        println!("{ev}");
    }
    println!("{}", rep.summary());
}
