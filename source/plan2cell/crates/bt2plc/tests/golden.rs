//! Golden files for the plan-specific outputs. `BT2PLC_BLESS=1 cargo test --test golden` rewrites them.

mod common;
use bt2plc::{Config, GenerateOptions};
use common::*;

const FILES: &[&str] = &[
    "ZONES.md",
    "cell_plan.json",
    "project.json",
    "codesys/head/GVL/GVL_Cell.st",
    "codesys/head/GVL/GVL_IO.st",
    "codesys/head/POU/PLC_PRG.st",
    "codesys/head/POU/PLC_PRG_Body.st",
    "codesys/head/io_mapping.csv",
];

fn robot_files(ns: &str) -> Vec<String> {
    vec![
        format!("robots/{ns}/program.json"),
        format!("codesys/robots/{ns}/GVL/GVL_Program.st"),
        format!("codesys/robots/{ns}/io_mapping.csv"),
    ]
}

fn golden(sample: &str, dir: &str, robots: &[&str]) {
    let mut d = bt2plc::Diagnostics::default();
    let doc = bt2plc::input::load_str(
        &fixture(sample),
        sample,
        bt2plc::input::SourceSel::Auto,
        None,
        &mut d,
    )
    .unwrap();
    let g = bt2plc::generate(
        &doc,
        &Config::default(),
        &GenerateOptions {
            st_check: false,
            ..Default::default()
        },
    )
    .unwrap();
    let root = format!("{}/tests/golden/{dir}", env!("CARGO_MANIFEST_DIR"));
    let bless = std::env::var("BT2PLC_BLESS").is_ok();
    let mut files: Vec<String> = FILES.iter().map(|s| s.to_string()).collect();
    for r in robots {
        files.extend(robot_files(r));
    }
    for f in files {
        let got = g
            .output
            .get(&f)
            .unwrap_or_else(|| panic!("{f} not generated"));
        let path = format!("{root}/{f}");
        if bless {
            std::fs::create_dir_all(std::path::Path::new(&path).parent().unwrap()).unwrap();
            std::fs::write(&path, got).unwrap();
            continue;
        }
        let want = std::fs::read_to_string(&path)
            .unwrap_or_else(|_| panic!("missing golden {path}; run with BT2PLC_BLESS=1"));
        pretty_assertions::assert_eq!(want, got, "{f}");
    }
}

#[test]
fn welding_golden() {
    golden(WELDING, "welding_cell_2_small", &["robot1", "robot2"]);
}

#[test]
fn cell_2robot_golden() {
    golden(TWO_ROBOT, "cell_2robot", &["robot1"]);
}

#[test]
fn generation_is_deterministic_in_process() {
    let mut d = bt2plc::Diagnostics::default();
    let doc = bt2plc::input::load_str(
        &fixture(WELDING),
        WELDING,
        bt2plc::input::SourceSel::Auto,
        None,
        &mut d,
    )
    .unwrap();
    let opts = GenerateOptions {
        st_check: false,
        ..Default::default()
    };
    let a = bt2plc::generate(&doc, &Config::default(), &opts).unwrap();
    let b = bt2plc::generate(&doc, &Config::default(), &opts).unwrap();
    assert_eq!(a.output.files(), b.output.files());
}
