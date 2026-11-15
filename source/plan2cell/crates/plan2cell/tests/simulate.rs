//! A build co-simulated end to end (feature `sim`): the head PLC from this build against the
//! executor of this build's KRL. Slow (trust-runtime): `cargo test -p plan2cell --features sim`.
#![cfg(feature = "sim")]

use std::path::{Path, PathBuf};

use plan2cell::sim::{Robots, simulate};
use plan2cell::{BuildConfig, BuildOptions, PlanInput};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

#[test]
fn the_single_robot_build_runs_a_cycle_with_its_krl() {
    let input = PlanInput::load(&root().join("test_data/bt_1791274820_cell_2robot.json")).unwrap();
    let cfg = BuildConfig::load(&root().join("examples/plan2cell.toml")).unwrap();
    let opts = BuildOptions {
        st_check: false,
        ..Default::default()
    };
    let b = plan2cell::build(&input, &cfg, &opts).unwrap();
    let (_, r) = simulate(&b, &cfg, Robots::Krl, 1).unwrap();
    assert!(r.ok, "{:?}\n{}", r.errors, r.tail.join("\n"));
    assert_eq!(r.cycles_completed, 1);
}
