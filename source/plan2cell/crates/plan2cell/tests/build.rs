//! The combined build: options, profile consistency, output layout, determinism.

use std::path::{Path, PathBuf};

use plan2cell::{BuildConfig, BuildOptions, PlanInput, PlcTarget, Profile};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn welding() -> PlanInput {
    PlanInput::load(&root().join("test_data/bt_1790934549_welding_cell_2_small.json")).unwrap()
}

fn cfg() -> BuildConfig {
    BuildConfig::load(&root().join("examples/plan2cell.toml")).unwrap()
}

fn opts() -> BuildOptions {
    BuildOptions {
        st_check: false,
        ..Default::default()
    }
}

#[test]
fn a_full_build_has_plc_krl_and_a_summary() {
    let b = plan2cell::build(&welding(), &cfg(), &opts()).unwrap();
    for f in [
        "plc/cell_plan.json",
        "plc/robots/robot1/program.json",
        "plc/codesys/head/GVL/GVL_Cell.st",
        "plc/codesys/robots/robot1/GVL/GVL_Program.st",
        "krl/robot1/KRC/R1/Folgen/folge11.src",
        "krl/robot2/KRC/R1/Program/bt_cell.src",
        "BUILD.md",
        "plan2cell-build.json",
    ] {
        assert!(b.files.contains_key(f), "{f} missing");
    }
    // One PLAN_ID across the head PLC and every robot program.
    let id = &b.summary.plan_id;
    let gvl = &b.files["plc/codesys/head/GVL/GVL_Cell.st"];
    let n = u16::from_str_radix(id.trim_start_matches("16#"), 16).unwrap();
    assert!(gvl.contains("PLAN_ID") && gvl.contains(&format!("16#{n:04X}")));
    for ns in ["robot1", "robot2"] {
        let dat = &b.files[&format!("krl/{ns}/KRC/R1/Folgen/folge11.dat")];
        assert!(dat.contains(&format!("DECL INT PLAN_ID={n}")), "{ns}");
    }
    assert_eq!(b.summary.robots.len(), 2);
    assert_eq!(b.summary.robots[0].folgen, vec![11, 125]);
}

#[test]
fn the_examples_are_up_to_date() {
    // examples/<cell> is exactly what `plan2cell build` writes for the example config (with the
    // trust check, as the CLI default).
    for (cell, plan) in [
        (
            "welding_cell_2_small",
            "bt_1790934549_welding_cell_2_small.json",
        ),
        ("cell_2robot", "bt_1791274820_cell_2robot.json"),
    ] {
        let input = PlanInput::load(&root().join("test_data").join(plan)).unwrap();
        let b = plan2cell::build(&input, &cfg(), &cfg().options()).unwrap();
        for (rel, text) in &b.files {
            let path = root().join("examples").join(cell).join(rel);
            let on_disk = std::fs::read_to_string(&path)
                .unwrap_or_else(|_| panic!("{} missing: regenerate the examples", path.display()));
            assert!(
                on_disk == *text,
                "{} is stale: run `plan2cell build test_data/{plan} -c examples/plan2cell.toml -o examples/{cell} --force`",
                path.display()
            );
        }
    }
}

#[test]
fn options_select_what_is_built() {
    let b = plan2cell::build(
        &welding(),
        &cfg(),
        &BuildOptions {
            krl: false,
            ..opts()
        },
    )
    .unwrap();
    assert!(b.krl.is_none() && !b.files.keys().any(|k| k.starts_with("krl/")));
    let b = plan2cell::build(
        &welding(),
        &cfg(),
        &BuildOptions {
            plc: PlcTarget::None,
            ..opts()
        },
    )
    .unwrap();
    assert!(!b.files.keys().any(|k| k.starts_with("plc/codesys/")));
    assert!(b.files.contains_key("plc/robots/robot1/program.json"));
    assert!(b.files.contains_key("krl/robot1/KRC/R1/Folgen/folge11.src"));
    let b = plan2cell::build(
        &welding(),
        &cfg(),
        &BuildOptions {
            robot_sim: false,
            ..opts()
        },
    )
    .unwrap();
    assert!(!b.files.keys().any(|k| k.starts_with("plc/codesys/robots/")));
    let b = plan2cell::build(
        &welding(),
        &cfg(),
        &BuildOptions {
            krl_robots: Some(vec!["robot2".into()]),
            ..opts()
        },
    )
    .unwrap();
    assert!(
        b.files
            .keys()
            .filter(|k| k.starts_with("krl/"))
            .all(|k| k.starts_with("krl/robot2/"))
    );
}

#[test]
fn the_vw_profile_is_applied_to_both_sides() {
    let mut c = cfg();
    for ns in ["robot1", "robot2"] {
        c.plc.robots.push(bt2plc::config::RobotCfg {
            ns: ns.into(),
            interlocks: [("IL01".to_string(), 1u8), ("IL02".to_string(), 11u8)]
                .into_iter()
                .collect(),
            ..Default::default()
        });
    }
    c.plc.protocol.plan_id_byte = 12;
    let b = plan2cell::build(
        &welding(),
        &c,
        &BuildOptions {
            profile: Profile::VwKs24,
            ..opts()
        },
    )
    .unwrap();
    // Adjusted for the profile, and said so.
    assert!(b.notes.iter().any(|n| n.contains("job_number")));
    assert!(b.notes.iter().any(|n| n.contains("robot simulator")));
    let gvl = &b.files["plc/codesys/head/GVL/GVL_Cell.st"];
    assert!(gvl.contains("PROFILE_VW              : BOOL := TRUE;"));
    assert!(gvl.contains("[1,11,1,11]"));
    // The KRL uses the same interlocks and the same PLAN_ID byte.
    let src = &b.files["krl/robot1/KRC/R1/Folgen/folge11.src"];
    assert!(
        src.contains("$OUT[81]=TRUE ; request V1") && src.contains("$OUT[91]=TRUE ; request V11")
    );
    let cfgdat = &b.files["krl/robot1/KRC/R1/System/bt_config.dat"];
    assert!(
        cfgdat.contains("SIGNAL BT_PLANID $IN[97] TO $IN[112]"),
        "{cfgdat}"
    );
    assert!(!b.files.keys().any(|k| k.starts_with("plc/codesys/robots/")));
}

#[test]
fn the_vw_profile_never_puts_plan_id_on_the_interlock_bytes() {
    // Without an explicit PLAN_ID byte the sim default (4) would land on E33-48 of a VW robot,
    // i.e. on the partner interlock releases: the build turns the check off instead.
    let b = plan2cell::build(
        &welding(),
        &cfg(),
        &BuildOptions {
            profile: Profile::VwKs24,
            ..opts()
        },
    )
    .unwrap();
    assert!(b.notes.iter().any(|n| n.contains("no PLAN_ID check")));
    let gvl = &b.files["plc/codesys/head/GVL/GVL_Cell.st"];
    assert!(
        gvl.lines()
            .any(|l| l.contains("PLAN_ID_BYTE") && l.contains(":= 0;")),
        "{gvl}"
    );
    // A conflicting byte in [krl] is an error: one source of truth.
    let mut c = cfg();
    c.plc.protocol.plan_id_byte = 12;
    c.krl.vw_ks24.plan_id_byte = 13;
    let e = plan2cell::build(
        &welding(),
        &c,
        &BuildOptions {
            profile: Profile::VwKs24,
            ..opts()
        },
    )
    .err()
    .expect("conflicting PLAN_ID bytes");
    assert_eq!(e.exit_code(), 3, "{e}");
}

#[test]
fn errors_keep_the_tool_exit_codes() {
    // No [[krl.robot]] e_axes: bt2krl's config error (3).
    let e = plan2cell::build(&welding(), &BuildConfig::default(), &opts())
        .err()
        .unwrap();
    assert_eq!(e.exit_code(), 3, "{e}");
    // A broken plan: bt2plc's input error (1).
    let e = plan2cell::build(
        &PlanInput::from_text("{\"x\": 1}", "x.json"),
        &cfg(),
        &opts(),
    )
    .err()
    .unwrap();
    assert_eq!(e.exit_code(), 1, "{e}");
}

#[test]
fn writing_and_determinism() {
    let a = plan2cell::build(&welding(), &cfg(), &opts()).unwrap();
    let b = plan2cell::build(&welding(), &cfg(), &opts()).unwrap();
    assert_eq!(a.files, b.files);
    let d = tempfile::tempdir().unwrap();
    let out = d.path().join("out");
    a.write_to(&out, false).unwrap();
    assert!(out.join("krl/robot1/KRC/R1/Folgen/folge11.src").is_file());
    assert!(
        a.write_to(&out, false).is_err(),
        "non-empty directory needs force"
    );
    a.write_to(&out, true).unwrap();
    // The written KRL re-validates.
    bt2krl::check_dir(&out.join("krl")).unwrap();
}
