//! The generated ST type-checks with trust-platform for the samples and for config/plan variants
//! that exercise other template paths (FCFS, byte job numbers, 3 robots, permit overflow).
#![cfg(feature = "check")]

mod common;
use bt2plc::config::{GrantPolicy, JobNumber};
use bt2plc::{Config, GenerateOptions};
use common::*;

fn check(text: &str, cfg: &Config) {
    let mut d = bt2plc::Diagnostics::default();
    let doc = bt2plc::input::load_str(text, "t.json", bt2plc::input::SourceSel::Auto, None, &mut d)
        .unwrap();
    let g = bt2plc::generate(
        &doc,
        cfg,
        &GenerateOptions {
            st_check: false,
            ..Default::default()
        },
    )
    .unwrap();
    let diags = bt2plc::trust::check::check_projects(&g.projects, false);
    assert!(
        diags.is_empty(),
        "{}",
        diags
            .iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join("\n")
    );
}

#[test]
fn samples_type_check() {
    check(&fixture(WELDING), &Config::default());
    check(&fixture(TWO_ROBOT), &Config::default());
}

#[test]
fn variants_type_check() {
    let mut cfg = Config::default();
    cfg.cell.grant_policy = GrantPolicy::Fcfs;
    cfg.cell.job_number = JobNumber::Byte;
    check(&fixture(WELDING), &cfg);

    let mut t = Tree::default();
    let root = seq(vec![
        par(vec![
            t.exec("r1", "A1", 1.0),
            t.exec("r2", "B1", 1.0),
            t.exec("r3", "C1", 1.0),
        ]),
        t.exec("r1", "A2", 1.0),
        par(vec![t.home("r1"), t.home("r2"), t.home("r3")]),
    ]);
    check(&t.envelope(root), &Config::default());

    let mut t = Tree::default();
    let leader: Vec<_> = (1..=35)
        .map(|i| t.exec("r1", &format!("A{i}"), 0.5))
        .collect();
    let mut follower = Vec::new();
    for i in 1..=35 {
        follower.push(t.wait("r2", &[("r1", &format!("A{i}"))]));
        follower.push(t.exec("r2", &format!("B{i}"), 0.5));
    }
    check(
        &t.envelope(par(vec![seq(leader), seq(follower)])),
        &Config::default(),
    );
}
