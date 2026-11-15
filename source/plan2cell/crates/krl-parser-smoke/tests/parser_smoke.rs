//! Smoke check with the lenient krl-runner parser (`cargo test -p krl-parser-smoke`).
//! It cannot reject bad syntax; see src/smoke.rs for what is asserted.

use std::path::{Path, PathBuf};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn plc_dir(sample: &str) -> PathBuf {
    root().join("examples").join(sample).join("plc")
}

fn test_cfg() -> bt2krl::Config {
    bt2krl::Config::load(&root().join("crates/bt2krl/tests/fixtures/bt2krl.toml")).unwrap()
}

#[test]
fn parser_finds_the_folgen_and_their_motions() {
    for sample in ["welding_cell_2_small", "cell_2robot"] {
        let d = tempfile::tempdir().unwrap();
        let programs = bt2krl::load_bt2plc_dir(&plc_dir(sample)).unwrap();
        let g = bt2krl::generate(&programs, &test_cfg(), &bt2krl::Options::default()).unwrap();
        g.output.write_to(d.path(), false).unwrap();
        let rep = krl_parser_smoke::check_dir(d.path()).unwrap();
        assert_eq!(rep.len(), 2 * programs.len(), "{rep:?}");
    }
}
