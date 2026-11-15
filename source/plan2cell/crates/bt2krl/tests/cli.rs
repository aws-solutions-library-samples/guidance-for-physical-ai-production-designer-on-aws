//! The binary: commands, exit codes (bt2plc numbering), output directory handling.

mod common;
use common::*;
use std::process::Command;

fn bt2krl(args: &[&str]) -> (i32, String, String) {
    let o = Command::new(env!("CARGO_BIN_EXE_bt2krl"))
        .args(args)
        .output()
        .unwrap();
    (
        o.status.code().unwrap_or(-1),
        String::from_utf8_lossy(&o.stdout).into_owned(),
        String::from_utf8_lossy(&o.stderr).into_owned(),
    )
}

fn fixture_cfg() -> String {
    format!("{}/tests/fixtures/bt2krl.toml", env!("CARGO_MANIFEST_DIR"))
}

#[test]
fn generate_check_and_exit_codes() {
    let d = tempfile::tempdir().unwrap();
    let out = d.path().join("out");
    let welding = plc_dir("welding_cell_2_small");
    let (w, o) = (welding.to_str().unwrap(), out.to_str().unwrap());
    let cfg = fixture_cfg();

    let (c, s, e) = bt2krl(&["generate", w, "-o", o, "-c", &cfg]);
    assert_eq!(c, 0, "{s}{e}");
    assert!(s.contains("wrote 20 files"), "{s}");
    assert!(out.join("robot1/KRC/R1/Folgen/folge11.src").is_file());

    let (c, s, _) = bt2krl(&["check", o]);
    assert_eq!(c, 0);
    assert!(
        s.contains("robot1: 9 files, 8 ops, profile sim, valid"),
        "{s}"
    );

    // Non-empty output directory without --force: I/O (4); with --force: ok.
    assert_eq!(bt2krl(&["generate", w, "-o", o, "-c", &cfg]).0, 4);
    assert_eq!(
        bt2krl(&["generate", w, "-o", o, "-c", &cfg, "--force"]).0,
        0
    );

    // Dry run writes nothing.
    let dry = d.path().join("dry");
    let (c, s, _) = bt2krl(&[
        "generate",
        w,
        "-o",
        dry.to_str().unwrap(),
        "-c",
        &cfg,
        "--dry-run",
    ]);
    assert_eq!(c, 0);
    assert!(s.contains("nothing written") && !dry.exists());

    // Config without e_axes: config error (3). Bad TOML key: 3.
    let (c, _, e) = bt2krl(&["generate", w, "-o", dry.to_str().unwrap()]);
    assert_eq!(c, 3, "{e}");
    let bad = d.path().join("bad.toml");
    std::fs::write(&bad, "[motion]\nmod = 1\n").unwrap();
    assert_eq!(
        bt2krl(&["generate", w, "-o", "x", "-c", bad.to_str().unwrap()]).0,
        3
    );

    // Usage: unknown flag (2).
    assert_eq!(bt2krl(&["generate", w, "--nope"]).0, 2);

    // Profile vw-ks24 needs its own config (the interlock map) and writes no bt_cell / bt_sps.
    let vw = format!(
        "{}/tests/fixtures/bt2krl-vw.toml",
        env!("CARGO_MANIFEST_DIR")
    );
    let vwout = d.path().join("vw");
    let vwo = vwout.to_str().unwrap();
    let (c, s, e) = bt2krl(&["generate", w, "-o", vwo, "-c", &vw, "--profile", "vw-ks24"]);
    assert_eq!(c, 0, "{s}{e}");
    assert!(vwout.join("robot1/KRC/R1/Folgen/folge11.src").is_file());
    assert!(!vwout.join("robot1/KRC/R1/Program/bt_cell.src").exists());
    assert_eq!(bt2krl(&["check", vwo]).0, 0);

    // Missing input: I/O (4); not a bt2plc dir: 4.
    assert_eq!(
        bt2krl(&["generate", "/nonexistent", "-o", "x", "-c", &cfg]).0,
        4
    );

    // A tampered file: check fails with 7 (hash) or 8 (validator).
    let f = out.join("robot2/KRC/R1/Folgen/folge11.src");
    let t = std::fs::read_to_string(&f)
        .unwrap()
        .replace("WAIT FOR NOT $IN[97]", "WAIT SEC 0");
    std::fs::write(&f, t).unwrap();
    let c = bt2krl(&["check", o]).0;
    assert!(c == 8, "exit {c}");
}

#[test]
fn explain_and_config() {
    let p = plc_dir("welding_cell_2_small").join("robots/robot1/program.json");
    let (c, s, _) = bt2krl(&["explain", p.to_str().unwrap()]);
    assert_eq!(c, 0);
    assert!(
        s.contains("zone IL01 bit 0 -> REQUEST $OUT[225], GRANT $IN[97]"),
        "{s}"
    );
    assert!(s.contains("op   8 HOME"), "{s}");
    let (c, s, _) = bt2krl(&["config"]);
    assert_eq!(c, 0);
    bt2krl::Config::parse(&s).unwrap();
}
