//! Opt-in checks against the real VW KS24 exports: `BT2KRL_VW_SAMPLES=<robot_data_export>`.
//! Skipped (and reported as skipped) without that variable; no sample file is ever copied into
//! this repository. It catches drift of the assumptions in TASK 7.3 / src/vw.rs.

mod common;
use std::path::{Path, PathBuf};

use bt2krl::vw;

fn samples() -> Option<PathBuf> {
    let p = PathBuf::from(std::env::var("BT2KRL_VW_SAMPLES").ok()?);
    p.is_dir().then_some(p)
}

fn robots(root: &Path) -> Vec<PathBuf> {
    let mut v: Vec<PathBuf> = std::fs::read_dir(root)
        .unwrap()
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.join("KRC/R1/System/vw.src").is_file())
        .collect();
    v.sort();
    v
}

/// Is `needle` anywhere under `dir` (the declaration of a signal can sit in several places)?
fn grep_tree(dir: &Path, needle: &str) -> bool {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return false;
    };
    for e in rd.filter_map(|e| e.ok()) {
        let p = e.path();
        if p.is_dir() {
            if grep_tree(&p, needle) {
                return true;
            }
        } else if matches!(
            p.extension().and_then(|x| x.to_str()),
            Some("dat" | "src" | "sub")
        ) && read(&p).contains(needle)
        {
            return true;
        }
    }
    false
}

/// The exports are 7-bit ASCII for Folgen/Makros but ISO-8859 / UTF-8 for some system files.
fn read(p: &Path) -> String {
    let b = std::fs::read(p).unwrap_or_default();
    b.iter().map(|&c| c as char).collect()
}

#[test]
fn vw_routines_and_signals_exist_in_every_sample_robot() {
    let Some(root) = samples() else {
        eprintln!("skipped: set BT2KRL_VW_SAMPLES to the robot_data_export directory");
        return;
    };
    let rs = robots(&root);
    assert!(
        !rs.is_empty(),
        "no robot with KRC/R1/System/vw.src under {root:?}"
    );
    for r in &rs {
        let name = r.file_name().unwrap().to_string_lossy().to_string();
        let vwsrc = read(&r.join("KRC/R1/System/vw.src"));
        // The routines the generated Folge calls, and the dispatcher's signature.
        for cmd in [
            "#VW_AUE_INIT",
            "#VW_AUE_WAIT",
            "#VW_AUE_END",
            "#VW_INIT",
            "#FB_STOP",
            "#FB_CHECK",
        ] {
            assert!(vwsrc.contains(cmd), "{name}: vw.src has no {cmd}");
        }
        assert!(
            vwsrc.contains(
                "DEF  VW (COMMAND :IN,COND :IN,P1 :IN,P2 :IN,P3 :IN,P4 :IN,B1 :IN, POINT :IN )"
            ) || vwsrc.contains("DEF VW (COMMAND"),
            "{name}: unexpected VW() signature"
        );
        assert!(
            vwsrc.contains("$H_POS=$AXIS_ACT"),
            "{name}: AUE_WAIT no longer redefines $H_POS at P0"
        );
        // cell.src requires home before it starts a Folge (no blind home Folge in this profile).
        let cell = read(&r.join("KRC/R1/Folgen/cell.src"));
        assert!(
            cell.contains("$IN_HOME"),
            "{name}: cell.src does not wait for $IN_HOME"
        );
        assert!(
            cell.contains("SWITCH PGNO"),
            "{name}: cell.src has no SWITCH PGNO"
        );
        // The interlock signal map and the MAKRO20 handshake.
        let m20 = read(&r.join("KRC/R1/Makros/makro20.src"));
        for n in [1u8, 16] {
            let (rel, req, gr, par) = (
                vw::release_out(n),
                vw::request_out(n),
                vw::grant_in(n),
                vw::partner_in(n),
            );
            assert_eq!((rel, req), (40 + u16::from(n), 80 + u16::from(n)));
            assert!(
                m20.contains(&format!("$OUT[{rel}]")) && m20.contains(&format!("$OUT[{req}]")),
                "{name}: makro20 does not drive A{rel}/A{req}"
            );
            assert!(
                m20.contains(&format!("$IN[{gr}]")) && m20.contains(&format!("$IN[{par}]")),
                "{name}: makro20 does not wait on E{gr}/E{par}"
            );
        }
        assert!(
            m20.contains("NOT $LOOP_CONT AND ($MODE_OP<>#EX)"),
            "{name}: makro20 has no operator escape"
        );
        // The step signal and PF0.
        let usr = read(&r.join("KRC/R1/VW_User/vw_usr_r.dat"));
        assert!(
            usr.contains(&format!(
                "SIGNAL {} $OUT[{}]",
                vw::STEP_SIGNAL_VW,
                vw::STEP_OUT_VW
            )),
            "{name}: vw_usr_r.dat does not declare {} at $OUT[{}]",
            vw::STEP_SIGNAL_VW,
            vw::STEP_OUT_VW
        );
        let usr_s = read(&r.join("KRC/R1/VW_User/vw_usr_s.dat"));
        assert!(
            usr_s.contains(&format!("O_R_PFO={}", vw::PF0_OUT)),
            "{name}: O_R_PFO is not {}",
            vw::PF0_OUT
        );
        assert!(
            usr_s.contains("O_R_Anf_V1=81"),
            "{name}: O_R_Anf_V1 is not 81"
        );
        assert!(
            usr_s.contains("O_R_F_ENDE=14"),
            "{name}: O_R_F_ENDE is not 14"
        );
        // Folge selection on 8 bits, echoed on A1-A8.
        let conf = read(&r.join("KRC/R1/System/$config.dat"));
        assert!(
            conf.contains("SIGNAL PRG_NR $IN[1]") && conf.contains("TO $IN[8]"),
            "{name}: PRG_NR is not $IN[1..8]"
        );
        assert!(
            conf.contains("SIGNAL PRG_NR_REFL $OUT[1]"),
            "{name}: no PRG_NR_REFL"
        );
        assert!(conf.contains("INT SRB=9"), "{name}: SRB is not 9");
        // P_PktNr_SPS is local to vw_usr_r (no GLOBAL, not in $config.dat), so a Folge cannot
        // write it (KSS error 2137): bt2krl declares BT_PKTNR on the same outputs instead.
        assert!(
            !conf.contains(vw::STEP_SIGNAL_VW),
            "{name}: $config.dat declares {}",
            vw::STEP_SIGNAL_VW
        );
        assert!(
            !usr.lines().any(|l| l.contains(vw::STEP_SIGNAL_VW)
                && l.trim_start().to_ascii_uppercase().starts_with("GLOBAL")),
            "{name}: {} is GLOBAL in vw_usr_r.dat",
            vw::STEP_SIGNAL_VW
        );
        // The plain names the generated Folge takes from the VW package.
        for n in vw::VW_CONFIG_GLOBALS {
            assert!(
                conf.lines().any(|l| {
                    let w: Vec<&str> = l.split_whitespace().collect();
                    matches!(w.as_slice(), [_, x, ..] if x.split('=').next() == Some(n))
                }),
                "{name}: $config.dat does not declare {n}"
            );
        }
    }
    eprintln!("checked {} sample robots", rs.len());
}

#[test]
fn reserved_signals_are_really_used_by_the_vw_standard() {
    let Some(root) = samples() else {
        eprintln!("skipped: set BT2KRL_VW_SAMPLES to the robot_data_export directory");
        return;
    };
    for r in robots(&root) {
        let name = r.file_name().unwrap().to_string_lossy().to_string();
        let custom = read(&r.join("KRC/STEU/Mada/$custom.dat"));
        let mada = read(&r.join("KRC/STEU/Mada/$machine.dat"));
        // A33-36 Fertigmeldung ($BIN_OUT[18] at F_BIT 33), A40 $COLL_ALARM, A225-232 error code.
        assert!(
            custom.contains("F_BIT 33"),
            "{name}: no $BIN_OUT at F_BIT 33 (Fertigmeldung) - reserved range A33-40 may be stale"
        );
        assert!(
            mada.contains("$COLL_ALARM $OUT[40]"),
            "{name}: $COLL_ALARM is not $OUT[40]"
        );
        // A225-240: the error code / vehicle type. Where it is declared varies per robot
        // (STEU $custom.dat $BIN_OUT[17], or the VW_FEHLER application package), so search the
        // whole export.
        assert!(
            grep_tree(&r.join("KRC"), "VW_FEH_BIN_A") || custom.contains("F_BIT 225"),
            "{name}: nothing uses A225-232 - the reserved range may be stale"
        );
        for (a, b) in vw::RESERVED_IN {
            assert!(
                mada.contains(&format!("$IN[{a}]")) || mada.contains(&format!("$IN[{b}]")),
                "{name}: neither E{a} nor E{b} is a system signal"
            );
        }
    }
}

#[test]
fn the_sample_interlock_pairs_fit_the_generator_model() {
    let Some(root) = samples() else {
        eprintln!("skipped: set BT2KRL_VW_SAMPLES to the robot_data_export directory");
        return;
    };
    // Both partners of a pair use the same index n, and every index is 1..=16.
    for r in robots(&root) {
        let name = r.file_name().unwrap().to_string_lossy().to_string();
        let folgen = r.join("KRC/R1/Folgen");
        let mut found = 0;
        for e in std::fs::read_dir(&folgen).unwrap().filter_map(|e| e.ok()) {
            let p = e.path();
            if p.extension().is_none_or(|x| x != "src") {
                continue;
            }
            let text = read(&p);
            for n in 1..=16u8 {
                let req = format!("$OUT[{}] = TRUE", vw::request_out(n));
                if text.contains(&req) {
                    found += 1;
                    assert!(
                        text.contains(&format!("$OUT[{}] = TRUE", vw::release_out(n))),
                        "{name}: {:?} requests V{n} but never releases A{}",
                        p.file_name().unwrap(),
                        vw::release_out(n)
                    );
                }
            }
        }
        eprintln!("{name}: {found} interlock request sites");
    }
}
