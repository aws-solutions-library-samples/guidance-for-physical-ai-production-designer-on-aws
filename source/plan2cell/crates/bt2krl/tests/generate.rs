//! Generation behaviour beyond the goldens: robot data, safety checks, profiles, IO bases.

mod common;
use bt2krl::config::EAxes;
use bt2krl::{Config, Error, Options};
use common::*;

fn welding() -> Vec<bt2krl::program::Program> {
    bt2krl::load_bt2plc_dir(&plc_dir("welding_cell_2_small")).unwrap()
}

fn cfg(extra: &str) -> Config {
    Config::parse(&format!(
        "{extra}\n[[robot]]\nns = \"robot1\"\nmodel = \"KR240R2900_ULTRA\"\ne_axes = \"none\"\n\
         [[robot]]\nns = \"robot2\"\nmodel = \"KR240R2900_ULTRA\"\ne_axes = \"none\"\n"
    ))
    .unwrap()
}

fn file<'a>(g: &'a bt2krl::Generated, rel: &str) -> &'a str {
    g.output
        .get(rel)
        .unwrap_or_else(|| panic!("{rel} not generated"))
}

#[test]
fn undeclared_e_axes_is_a_config_error() {
    let e = bt2krl::generate(&welding(), &Config::default(), &Options::default()).unwrap_err();
    assert!(matches!(e, Error::Config(_)), "{e}");
    assert_eq!(e.exit_code(), 3);
}

#[test]
fn fixed_e_axes_write_e6axis_points() {
    let mut c = cfg("");
    c.robots[0].e_axes = Some(EAxes::Fixed);
    c.robots[0].e_values = Some([95.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let g = bt2krl::generate(&welding(), &c, &Options::default()).unwrap();
    let dat = file(&g, "robot1/KRC/R1/Folgen/folge11.dat");
    assert!(dat.contains("DECL E6AXIS P1={A1 0.0,A2 -90.0,A3 90.0,A4 0.0,A5 0.0,A6 0.0,E1 95.0,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0}"));
    assert!(!dat.contains("DECL AXIS P"));
    assert!(file(&g, "robot2/KRC/R1/Folgen/folge11.dat").contains("DECL AXIS P1="));
    assert!(file(&g, "robot1/INSTALL.md").contains("the external axes MOVE"));
}

#[test]
fn joint_map_preset_is_caught_by_the_limit_check() {
    // The kr240 preset of cell-data-replay/joint_map.yaml maps the planner's home A2 -90 to -180,
    // outside the KR240 soft limit -140..-5: generation must refuse it (exit 1).
    let mut plc = bt2plc::Config::default();
    plc.robots.push(bt2plc::config::RobotCfg {
        ns: "robot1".into(),
        joint_map: Some(bt2plc::config::JointMapPreset::Kr240R2900Ultra),
        ..Default::default()
    });
    let text =
        std::fs::read_to_string(test_data("bt_1790934549_welding_cell_2_small.json")).unwrap();
    let programs = programs(&text, &plc);
    assert_eq!(programs[0].joint_mapping.describe(), "kr240_r2900_ultra");
    let e = bt2krl::generate(&programs, &cfg(""), &Options::default()).unwrap_err();
    assert_eq!(e.exit_code(), 1, "{e}");
    assert!(e.to_string().contains("software limits"), "{e}");
    // Identity (the default) passes, and the mapping is reported.
    let programs = programs_identity(&text);
    let g = bt2krl::generate(&programs, &cfg(""), &Options::default()).unwrap();
    assert!(file(&g, "robot1/INSTALL.md").contains("mapping `identity`"));
}

fn programs_identity(text: &str) -> Vec<bt2krl::program::Program> {
    programs(text, &bt2plc::Config::default())
}

#[test]
fn too_slow_motion_fails_the_head_watchdog_check() {
    let e = bt2krl::generate(
        &welding(),
        &cfg("[motion]\nmax_vel_pct = 2\n"),
        &Options::default(),
    )
    .unwrap_err();
    assert_eq!(e.exit_code(), 1, "{e}");
    assert!(e.to_string().contains("limit_s"), "{e}");
}

#[test]
fn h_pos_is_checked_against_the_home_pose() {
    let mut c = cfg("");
    c.robots[0].h_pos_deg = Some([-90.0, -120.0, 90.0, -5.0, 40.0, -5.0]);
    let e = bt2krl::generate(&welding(), &c, &Options::default()).unwrap_err();
    assert!(e.to_string().contains("$H_POS"), "{e}");
    c.robots[0].h_pos_deg = Some([1.0, -90.0, 90.0, 0.0, 0.0, -1.5]);
    bt2krl::generate(&welding(), &c, &Options::default()).unwrap();
}

#[test]
fn model_conflict_and_unknown_model() {
    let mut p = welding();
    p[0].robot.model = "KR210R2700_EXTRA".into();
    let e = bt2krl::generate(&p, &cfg(""), &Options::default()).unwrap_err();
    assert_eq!(e.exit_code(), 3, "{e}");
    let mut c = cfg("");
    c.robots[1].model = None;
    let g = bt2krl::generate(&welding(), &c, &Options::default()).unwrap();
    assert!(g.warnings.iter().any(|w| w.contains("model unknown")));
    let e = bt2krl::generate(
        &welding(),
        &c,
        &Options {
            strict: true,
            ..Default::default()
        },
    )
    .unwrap_err();
    assert_eq!(e.exit_code(), 6);
}

#[test]
fn io_bases_and_byte_job_mode_shift_the_signals() {
    let mut p = welding();
    for x in &mut p {
        x.protocol.job_number = "byte".into();
    }
    let g = bt2krl::generate(
        &p,
        &cfg("[sim]\nin_base = 1000\nout_base = 2000\n"),
        &Options::default(),
    )
    .unwrap();
    let c = file(&g, "robot1/KRC/R1/System/bt_config.dat");
    assert!(c.contains("SIGNAL BT_JOB $IN[1017] TO $IN[1024]"), "{c}");
    assert!(c.contains("SIGNAL BT_ECHO $OUT[2017] TO $OUT[2024]"));
    assert!(c.contains("SIGNAL BT_STEP $OUT[2033] TO $OUT[2048]"));
    assert!(c.contains("SIGNAL BT_PLANID $IN[1033] TO $IN[1048]"));
    let s = file(&g, "robot1/KRC/R1/Folgen/folge11.src");
    assert!(s.contains("$OUT[2225]=TRUE ; REQ IL01") && s.contains("WAIT FOR $IN[1097] ; GRANT"));
}

#[test]
fn permit_overflow_uses_bit_31() {
    // max_zones = 1: the second dependency chain goes to the permit bit.
    let mut t = Tree::default();
    let root = seq(vec![
        par(vec![t.exec("r0", "A1", 1.0), t.exec("r1", "B1", 1.0)]),
        par(vec![t.exec("r0", "A2", 1.0), t.exec("r1", "B2", 1.0)]),
        par(vec![t.exec("r0", "A3", 1.0), t.exec("r1", "B3", 1.0)]),
    ]);
    let mut plc = bt2plc::Config::default();
    plc.protocol.max_zones = 1;
    let programs = programs(&t.envelope(root), &plc);
    assert!(
        programs
            .iter()
            .flat_map(|p| &p.ops)
            .any(|o| o.mask() & (1 << 31) != 0)
    );
    let g = bt2krl::generate(&programs, &cfg_for(&["r0", "r1"]), &Options::default()).unwrap();
    let all: String = g
        .output
        .files()
        .iter()
        .filter(|(k, _)| k.ends_with("folge11.src"))
        .map(|(_, v)| v.as_str())
        .collect();
    assert!(all.contains("$OUT[256]=TRUE ; REQ PERMIT"), "{all}");
    assert!(all.contains("$IN[128]"));
}

#[test]
fn unimplemented_motion_mode_is_a_usage_error() {
    let e = bt2krl::generate(
        &welding(),
        &cfg("[motion]\nmode = \"spline\"\n"),
        &Options::default(),
    )
    .unwrap_err();
    assert_eq!(e.exit_code(), 2);
}

#[test]
fn robot_selection_and_unknown_robot() {
    let g = bt2krl::generate(
        &welding(),
        &cfg(""),
        &Options {
            robots: Some(vec!["robot2".into()]),
            ..Default::default()
        },
    )
    .unwrap();
    assert!(g.output.files().keys().all(|k| k.starts_with("robot2/")));
    let e = bt2krl::generate(
        &welding(),
        &cfg(""),
        &Options {
            robots: Some(vec!["robot9".into()]),
            ..Default::default()
        },
    )
    .unwrap_err();
    assert_eq!(e.exit_code(), 2);
}

#[test]
fn every_krl_file_is_ascii_crlf_and_identifiers_are_short() {
    let g = bt2krl::generate(&welding(), &cfg(""), &Options::default()).unwrap();
    for (rel, text) in g.output.files() {
        assert!(text.is_ascii(), "{rel}");
        if rel.ends_with(".src") || rel.ends_with(".dat") {
            assert!(!text.ends_with('\n'), "{rel}");
            assert!(
                text.split("\r\n")
                    .all(|l| !l.contains('\n') && !l.contains('\r')),
                "{rel}"
            );
            for l in text.split("\r\n") {
                assert!(l.len() <= 321, "{rel}: line of {} chars", l.len());
            }
        }
    }
}

#[test]
fn stale_program_is_rejected() {
    let d = tempfile::tempdir().unwrap();
    let src = plc_dir("welding_cell_2_small");
    std::fs::copy(src.join("cell_plan.json"), d.path().join("cell_plan.json")).unwrap();
    for ns in ["robot1", "robot2"] {
        std::fs::create_dir_all(d.path().join("robots").join(ns)).unwrap();
        let t = std::fs::read_to_string(src.join("robots").join(ns).join("program.json")).unwrap();
        let id = welding()[0].plan.plan_id.clone();
        let t = if ns == "robot2" {
            t.replace(&id, "16#1234")
        } else {
            t
        };
        std::fs::write(d.path().join("robots").join(ns).join("program.json"), t).unwrap();
    }
    let e = bt2krl::load_bt2plc_dir(d.path()).unwrap_err();
    assert!(e.to_string().contains("stale"), "{e}");
}

#[test]
fn write_check_and_force() {
    let d = tempfile::tempdir().unwrap();
    let out = d.path().join("out");
    let g = bt2krl::generate(&welding(), &cfg(""), &Options::default()).unwrap();
    g.output.write_to(&out, false).unwrap();
    assert!(
        g.output.write_to(&out, false).is_err(),
        "non-empty directory needs --force"
    );
    std::fs::write(out.join("robot1/KRC/R1/Folgen/old.src"), "x").unwrap();
    let rep = bt2krl::check_dir(&out).unwrap();
    assert_eq!(rep.len(), 2, "{rep:?}");
    // A changed file fails the hash check.
    let f = out.join("robot1/KRC/R1/Folgen/folge11.src");
    let t = std::fs::read_to_string(&f)
        .unwrap()
        .replace("BT_STEP=2", "BT_STEP=3");
    std::fs::write(&f, t).unwrap();
    let e = bt2krl::check_dir(&out).unwrap_err();
    assert!(
        e.to_string().contains("STEP 3") || e.to_string().contains("hash"),
        "{e}"
    );
    // --force rewrites, and files of the previous run are removed (old.src was not ours).
    let g1 = bt2krl::generate(
        &welding(),
        &cfg(""),
        &Options {
            robots: Some(vec!["robot1".into()]),
            ..Default::default()
        },
    )
    .unwrap();
    g1.output.write_to(&out, true).unwrap();
    assert!(!out.join("robot2/KRC/R1/Folgen/folge11.src").exists());
    assert!(out.join("robot1/KRC/R1/Folgen/old.src").exists());
    bt2krl::check_dir(&out).unwrap();
}

#[test]
fn generation_is_deterministic() {
    let a = bt2krl::generate(&welding(), &cfg(""), &Options::default()).unwrap();
    let b = bt2krl::generate(&welding(), &cfg(""), &Options::default()).unwrap();
    assert_eq!(a.output.files(), b.output.files());
}

#[test]
fn folge_end_publishes_progfinished_before_progrunning_drops() {
    // The Folge sets BT_FIN before it clears BT_RUN (robot interpreter); bt_sps (submit) must
    // read BT_RUN before BT_FIN and write BT_O_FIN before BT_O_RUN, or a bus frame taken
    // between the two writes shows ProgRunning and ProgFinished both low (ERR_ROBOT_STOPPED).
    let g = bt2krl::generate(&welding(), &cfg(""), &Options::default()).unwrap();
    let folge = file(&g, "robot1/KRC/R1/Folgen/folge11.src");
    let fin = folge.find("BT_FIN=TRUE").unwrap();
    assert!(fin < folge.rfind("BT_RUN=FALSE").unwrap());
    // ProgFinished only once the robot stopped: the advance-run stop precedes the end
    // assignments ($ADVANCE=3 would otherwise run them during the last exact-stop motion).
    for (path, fold) in [
        ("robot1/KRC/R1/Folgen/folge11.src", ";FOLD FINISH"),
        ("robot1/KRC/R1/Folgen/folge125.src", ";FOLD DONE"),
    ] {
        let src = file(&g, path);
        let end = &src[src.find(fold).unwrap()..];
        let stop = end.find("WAIT SEC 0 ; advance-run stop").unwrap();
        assert!(stop < end.find("BT_FIN=TRUE").unwrap(), "{path}");
        assert!(stop < end.find("BT_RUN=FALSE").unwrap(), "{path}");
    }
    let sps = file(&g, "robot1/KRC/R1/System/bt_sps.src");
    let pos = |s: &str| sps.find(s).unwrap_or_else(|| panic!("{s} not in bt_sps"));
    assert!(pos("BT_PRUN=BT_RUN AND NOT BT_ABRT") < pos("BT_O_FIN=BT_FIN"));
    assert!(pos("BT_O_FIN=BT_FIN") < pos("BT_O_RUN=BT_PRUN"));
    assert_eq!(sps.matches("BT_O_RUN=").count(), 1);
    assert_eq!(sps.matches("BT_O_FIN=").count(), 1);
}

// ───────────────────────────── profile vw-ks24 ─────────────────────────────

fn vw_opts() -> Options {
    Options {
        profile: bt2krl::Profile::VwKs24,
        ..Default::default()
    }
}

#[test]
fn vw_ks24_uses_the_vw_interlocks_and_template() {
    let g = bt2krl::generate(&welding(), &vw_cfg(), &vw_opts()).unwrap();
    let src = file(&g, "robot1/KRC/R1/Folgen/folge11.src");
    // The VW template, in order.
    let order = [
        "VW(#VW_AUE_INIT,TRUE)",
        "PENTER:",
        "VW(#VW_INIT,TRUE)",
        "PTP P0",
        "VW(#VW_AUE_WAIT,TRUE )",
        "GOTO PEXIT",
        "SEL_RES=SELECT(#MAKRO,50,TRUE,,,,,FALSE)",
        "BT_PKTNR=1",
        "SEL_RES=SELECT(#MAKRO,15,TRUE,,,,,FALSE)",
        "BT_PKTNR=0",
        "VW(#VW_AUE_END,TRUE)",
        "GOTO PENTER",
        "PEXIT:",
    ];
    let mut at = 0;
    for x in order {
        at = src[at..]
            .find(x)
            .map(|i| at + i + x.len())
            .unwrap_or_else(|| panic!("{x} missing or out of order in\n{src}"));
    }
    // IL01 -> V1, IL02 -> V11 (the configured map).
    assert!(src.contains("$OUT[81]=TRUE ; request V1"));
    assert!(src.contains("$OUT[41]=$OUT[41] OR $OUT[81]"));
    assert!(src.contains("WAIT FOR ($IN[81]) OR (NOT $LOOP_CONT AND ($MODE_OP<>#EX))"));
    assert!(src.contains("WAIT FOR ($IN[41]) OR (NOT $LOOP_CONT AND ($MODE_OP<>#EX))"));
    assert!(src.contains("$OUT[41]=FALSE ; occupied"));
    assert!(src.contains("$OUT[91]=TRUE ; request V11"));
    assert!(src.contains("$OUT[51]=TRUE ; release V11"));
    // Online driving condition while an interlock is held.
    assert!(
        src.contains(
            "$CYCFLAG[254]=($CYCFLAG[30] AND $IN[41]) AND $CYCFLAG[255] OR ($TIMER[61]<0)"
        )
    );
    assert!(src.contains("VW(#FB_CHECK,TRUE)"));
    // No sim signals, no home Folge, no bt_cell / bt_sps.
    assert!(!src.contains("BT_STEP") && !src.contains("BT_RUN"));
    assert!(g.output.get("robot1/KRC/R1/Folgen/folge125.src").is_none());
    assert!(g.output.get("robot1/KRC/R1/Program/bt_cell.src").is_none());
    assert!(g.output.get("robot1/KRC/R1/System/bt_sps.src").is_none());
    // The patches are fragments and say so.
    let cell = file(&g, "robot1/patches/KRC/R1/Folgen/cell.src");
    assert!(
        cell.contains("OPTIONAL PATCH") && cell.contains("CASE 11") && cell.contains("FOLGE11()")
    );
    assert!(file(&g, "robot1/patches/KRC/R1/System/$config.dat").contains("OPTIONAL PATCH"));
    // The .dat declares the VW externals and keeps E-axis-free AXIS points.
    let dat = file(&g, "robot1/KRC/R1/Folgen/folge11.dat");
    assert!(dat.contains("EXT VW (VW_COMMAND") && dat.contains("EXTFCT INT SELECT"));
    assert!(dat.contains("DECL AXIS P0={A1 0.0,A2 -90.0,A3 90.0,A4 0.0,A5 0.0,A6 0.0}"));
}

#[test]
fn vw_ks24_folge_names_resolve_on_a_ks24_controller() {
    // KSS compile check by name (error 2137 "name unknown", seen on the VW port of the KUKA
    // virtual controllers for P_PktNr_SPS, which is local to vw_usr_r): every plain name of the
    // production Folge is a KRL keyword, declared by its .dat, by bt_config.dat (merged into
    // $config.dat), a label, or one of the VW package's $config.dat globals
    // (bt2krl::names, the check plan2cell's deployability check runs on every export).
    for plan_id_byte in [0u8, 12] {
        let mut c = vw_cfg();
        c.vw_ks24.plan_id_byte = plan_id_byte;
        let g = bt2krl::generate(&welding(), &c, &vw_opts()).unwrap();
        for ns in ["robot1", "robot2"] {
            let src = file(&g, &format!("{ns}/KRC/R1/Folgen/folge11.src"));
            let files: Vec<(String, &str)> = g
                .output
                .files()
                .iter()
                .filter_map(|(rel, text)| Some((rel.strip_prefix(&format!("{ns}/"))?.to_string(), text.as_str())))
                .collect();
            let unknown = bt2krl::names::unknown_names(
                files.iter().map(|(p, t)| (p.as_str(), *t)),
                &bt2krl::names::bundled_globals("vw-ks24"),
            );
            assert!(
                unknown.is_empty(),
                "{ns} (plan_id_byte {plan_id_byte}): names a KS24 controller does not declare globally: {unknown:?}"
            );
            let cfgdat = file(&g, &format!("{ns}/KRC/R1/System/bt_config.dat"));
            assert!(!src.contains(bt2krl::vw::STEP_SIGNAL_VW), "{ns}");
            assert!(
                cfgdat.contains("SIGNAL BT_PKTNR $OUT[97] TO $OUT[112]"),
                "{ns}: {cfgdat}"
            );
        }
    }
}

#[test]
fn sim_names_resolve_on_a_stock_controller() {
    // The same check for the sim profile: a stock KSS declares nothing the code relies on beyond
    // the bundle's own bt_config.dat.
    let g = bt2krl::generate(&welding(), &cfg_for(&["robot1", "robot2"]), &Options::default()).unwrap();
    for ns in ["robot1", "robot2"] {
        let files: Vec<(String, &str)> = g
            .output
            .files()
            .iter()
            .filter_map(|(rel, text)| Some((rel.strip_prefix(&format!("{ns}/"))?.to_string(), text.as_str())))
            .collect();
        let unknown = bt2krl::names::unknown_names(
            files.iter().map(|(p, t)| (p.as_str(), *t)),
            &bt2krl::names::bundled_globals("sim"),
        );
        assert!(unknown.is_empty(), "{ns}: {unknown:?}");
    }
}

#[test]
fn vw_ks24_never_writes_a_reserved_signal() {
    let g = bt2krl::generate(&welding(), &vw_cfg(), &vw_opts()).unwrap();
    for (rel, text) in g.output.files() {
        if !rel.ends_with(".src") && !rel.ends_with(".dat") {
            continue;
        }
        for line in text.split("\r\n") {
            let code = line.split(';').next().unwrap_or("");
            for (lo, hi) in bt2krl::vw::RESERVED_OUT {
                for a in lo..=hi {
                    assert!(
                        !code.contains(&format!("$OUT[{a}]")),
                        "{rel}: writes the reserved $OUT[{a}]: {line}"
                    );
                }
            }
        }
    }
}

#[test]
fn vw_ks24_refuses_the_permit_bit_and_blind_home() {
    // Permit overflow: name the edges and refuse.
    let mut t = Tree::default();
    let root = seq(vec![
        par(vec![
            t.exec("robot1", "A1", 1.0),
            t.exec("robot2", "B1", 1.0),
        ]),
        par(vec![
            t.exec("robot1", "A2", 1.0),
            t.exec("robot2", "B2", 1.0),
        ]),
        par(vec![
            t.exec("robot1", "A3", 1.0),
            t.exec("robot2", "B3", 1.0),
        ]),
        par(vec![t.home("robot1"), t.home("robot2")]),
    ]);
    let mut plc = bt2plc::Config::default();
    plc.protocol.max_zones = 1;
    let permit = programs(&t.envelope(root), &plc);
    let e = bt2krl::generate(&permit, &vw_cfg(), &vw_opts()).unwrap_err();
    assert_eq!(e.exit_code(), 1, "{e}");
    assert!(e.to_string().contains("no permit bit"), "{e}");
    // A blind MoveHome cannot work on a VW controller.
    let mut t = Tree::default();
    let root = seq(vec![t.exec("robot1", "A1", 1.0), t.blind_home("robot1")]);
    let blind = programs(&t.envelope(root), &bt2plc::Config::default());
    let mut vwc = vw_cfg();
    for r in &mut vwc.robots {
        r.interlocks.clear(); // this tree has no zones
    }
    let e = bt2krl::generate(&blind, &vwc, &vw_opts()).unwrap_err();
    assert_eq!(e.exit_code(), 1, "{e}");
    assert!(e.to_string().contains("blind"), "{e}");
    // sim accepts both.
    bt2krl::generate(&blind, &cfg_for(&["robot1"]), &Options::default()).unwrap();
}

#[test]
fn vw_ks24_interlock_map_is_checked() {
    let mut c = vw_cfg();
    // A zone without an entry.
    c.robots[0].interlocks.remove("IL02");
    let e = bt2krl::generate(&welding(), &c, &vw_opts()).unwrap_err();
    assert_eq!(e.exit_code(), 3, "{e}");
    // Two zones on one index.
    let mut c = vw_cfg();
    c.robots[0].interlocks.insert("IL02".into(), 1);
    let e = bt2krl::generate(&welding(), &c, &vw_opts()).unwrap_err();
    assert!(e.to_string().contains("is used by"), "{e}");
    // An index outside 1..=16.
    let mut c = vw_cfg();
    c.robots[0].interlocks.insert("IL02".into(), 17);
    assert!(c.validate().is_err());
    // The default map numbers the zones in name order.
    let mut c = vw_cfg();
    for r in &mut c.robots {
        r.interlocks.clear();
    }
    let g = bt2krl::generate(&welding(), &c, &vw_opts()).unwrap();
    let src = file(&g, "robot1/KRC/R1/Folgen/folge11.src");
    assert!(
        src.contains("$OUT[81]=TRUE ; request V1") && src.contains("$OUT[82]=TRUE ; request V2")
    );
}

#[test]
fn vw_ks24_plan_id_is_optional() {
    // Configured: a free byte, a signal and a HALT guard.
    let g = bt2krl::generate(&welding(), &vw_cfg(), &vw_opts()).unwrap();
    let cfgdat = file(&g, "robot1/KRC/R1/System/bt_config.dat");
    assert!(
        cfgdat.contains("SIGNAL BT_PLANID $IN[97] TO $IN[112]"),
        "{cfgdat}"
    );
    let id = welding()[0].plan_id();
    assert!(
        cfgdat.contains(&format!("DECL INT PLAN_ID={id}")),
        "{cfgdat}"
    );
    let src = file(&g, "robot1/KRC/R1/Folgen/folge11.src");
    assert!(src.contains("IF BT_PLANID<>PLAN_ID THEN") && src.contains("HALT"));
    // Not configured: documented instead.
    let mut c = vw_cfg();
    c.vw_ks24.plan_id_byte = 0;
    let g = bt2krl::generate(&welding(), &c, &vw_opts()).unwrap();
    assert!(!file(&g, "robot1/KRC/R1/Folgen/folge11.src").contains("BT_PLANID"));
    assert!(file(&g, "robot1/INSTALL.md").contains("**No PLAN_ID check.**"));
    // A byte the VW standard uses is rejected.
    let mut c = vw_cfg();
    c.vw_ks24.plan_id_byte = 5;
    assert!(c.validate().is_err());
}

#[test]
fn vw_ks24_weld_call_and_makro_switches() {
    let mut c = vw_cfg();
    c.vw_ks24.weld_call = "SEL_RES=SELECT(#MAKRO,580,TRUE,,,,,FALSE)".into();
    c.vw_ks24.makro15 = 0;
    c.vw_ks24.makro0 = false;
    c.vw_ks24.makro50 = false;
    let g = bt2krl::generate(&welding(), &c, &vw_opts()).unwrap();
    let src = file(&g, "robot1/KRC/R1/Folgen/folge11.src");
    assert!(src.contains("SEL_RES=SELECT(#MAKRO,580,TRUE,,,,,FALSE)"));
    assert!(
        !src.contains("SELECT(#MAKRO,15,")
            && !src.contains("SELECT(#MAKRO,0,")
            && !src.contains("SELECT(#MAKRO,50,")
    );
    // Only ops of the configured type get the call (robot1: 6 welds, R1_PARK and HOME are
    // MoveHome ops).
    let welds = welding()[0]
        .ops
        .iter()
        .filter(|o| o.operation_type() == "weld")
        .count();
    assert_eq!((welds, src.matches("MAKRO,580").count()), (6, 6));
    // use_makro20 replaces the inline handshake with the macro call.
    let mut c = vw_cfg();
    c.vw_ks24.use_makro20 = true;
    let g = bt2krl::generate(&welding(), &c, &vw_opts()).unwrap();
    let src = file(&g, "robot1/KRC/R1/Folgen/folge11.src");
    assert!(src.contains("SEL_RES=SELECT(#MAKRO,20,TRUE,,,,,FALSE)"));
    assert!(!src.contains("$OUT[41]=$OUT[41] OR $OUT[81]"));
}

#[test]
fn the_motion_time_estimates_match_the_plan_within_10_percent() {
    // M4 acceptance: the estimated motion time of every op is within 10 % of the planned
    // duration_s, in both profiles and on both example cells.
    for (sample, cfg, opts) in [
        ("welding_cell_2_small", test_cfg(), Options::default()),
        ("cell_2robot", test_cfg(), Options::default()),
        ("welding_cell_2_small", vw_cfg(), vw_opts()),
    ] {
        let programs = bt2krl::load_bt2plc_dir(&plc_dir(sample)).unwrap();
        let g = bt2krl::generate(&programs, &cfg, &opts).unwrap();
        let mut worst = (0.0f64, String::new());
        for (r, p) in g.robots.iter().zip(&programs) {
            for (e, op) in r.lowered.estimates.iter().zip(&p.ops) {
                if op.duration_s <= 0.0 {
                    continue;
                }
                let err = (e.est_motion_s - op.duration_s) / op.duration_s * 100.0;
                if err.abs() > worst.0 {
                    worst = (
                        err.abs(),
                        format!(
                            "{sample} {} {} est {:.3} s vs duration_s {:.3} s ({err:+.1} %)",
                            r.ns, e.operation_id, e.est_motion_s, op.duration_s
                        ),
                    );
                }
            }
        }
        assert!(worst.0 <= 10.0, "{}", worst.1);
        println!("{sample} {}: worst {:.1} %", opts.profile.name(), worst.0);
    }
}
