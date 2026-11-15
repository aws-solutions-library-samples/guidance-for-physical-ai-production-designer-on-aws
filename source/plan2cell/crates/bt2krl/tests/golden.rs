//! Golden files for the bt2plc example outputs (profile sim, motion ptp).
//! `BT2KRL_BLESS=1 cargo test --test golden` rewrites them.

mod common;
use bt2krl::{Options, Profile};
use common::*;

fn golden(sample: &str) {
    golden_profile(sample, Profile::Sim, sample, &test_cfg());
}

fn golden_profile(sample: &str, profile: Profile, dir: &str, cfg: &bt2krl::Config) {
    let programs = bt2krl::load_bt2plc_dir(&plc_dir(sample)).unwrap();
    let g = bt2krl::generate(
        &programs,
        cfg,
        &Options {
            profile,
            ..Default::default()
        },
    )
    .unwrap();
    let root = format!("{}/tests/golden/{dir}", env!("CARGO_MANIFEST_DIR"));
    let bless = std::env::var("BT2KRL_BLESS").is_ok();
    if bless {
        let _ = std::fs::remove_dir_all(&root);
    }
    for (rel, text) in g.output.files() {
        let path = format!("{root}/{rel}");
        if bless {
            std::fs::create_dir_all(std::path::Path::new(&path).parent().unwrap()).unwrap();
            std::fs::write(&path, text).unwrap();
            continue;
        }
        let want = std::fs::read_to_string(&path)
            .unwrap_or_else(|_| panic!("missing golden {path}; run with BT2KRL_BLESS=1"));
        pretty_assertions::assert_eq!(want, *text, "{rel}");
    }
    // No stale goldens.
    if !bless {
        let mut n = 0;
        for e in walk(std::path::Path::new(&root)) {
            let rel = e
                .strip_prefix(&root)
                .unwrap()
                .to_string_lossy()
                .trim_start_matches('/')
                .to_string();
            assert!(g.output.get(&rel).is_some(), "stale golden {rel}");
            n += 1;
        }
        assert_eq!(n, g.output.files().len());
    }
}

fn walk(d: &std::path::Path) -> Vec<std::path::PathBuf> {
    let mut v = Vec::new();
    for e in std::fs::read_dir(d).unwrap() {
        let p = e.unwrap().path();
        if p.is_dir() {
            v.extend(walk(&p));
        } else {
            v.push(p);
        }
    }
    v
}

#[test]
fn welding_golden() {
    golden("welding_cell_2_small");
}

#[test]
fn cell_2robot_golden() {
    golden("cell_2robot");
}

#[test]
fn welding_vw_ks24_golden() {
    golden_profile(
        "welding_cell_2_small",
        Profile::VwKs24,
        "welding_cell_2_small_vw",
        &vw_cfg(),
    );
}
