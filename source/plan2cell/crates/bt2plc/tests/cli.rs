//! The binary: exit codes and byte-identical output across processes.

mod common;
use std::process::Command;

fn bin() -> Command {
    Command::new(env!("CARGO_BIN_EXE_bt2plc"))
}

fn fixture_path(name: &str) -> String {
    format!("{}/../../test_data/{name}", env!("CARGO_MANIFEST_DIR"))
}

#[test]
fn generate_twice_gives_identical_directories() {
    let a = tempfile::tempdir().unwrap();
    let b = tempfile::tempdir().unwrap();
    for d in [&a, &b] {
        let out = bin()
            .args([
                "generate",
                &fixture_path(common::WELDING),
                "--no-check",
                "--force",
                "-q",
                "-o",
            ])
            .arg(d.path())
            .output()
            .unwrap();
        assert!(
            out.status.success(),
            "{}",
            String::from_utf8_lossy(&out.stderr)
        );
    }
    let list = |p: &std::path::Path| {
        let mut v = Vec::new();
        let mut stack = vec![p.to_path_buf()];
        while let Some(d) = stack.pop() {
            for e in std::fs::read_dir(&d).unwrap() {
                let e = e.unwrap().path();
                if e.is_dir() {
                    stack.push(e);
                } else {
                    v.push((
                        e.strip_prefix(p).unwrap().to_path_buf(),
                        std::fs::read(&e).unwrap(),
                    ));
                }
            }
        }
        v.sort();
        v
    };
    assert_eq!(list(a.path()), list(b.path()));
}

#[test]
fn exit_codes() {
    let dir = tempfile::tempdir().unwrap();
    let failed = dir.path().join("failed.json");
    std::fs::write(
        &failed,
        r#"{"status":"failed","behaviour_tree":{"root":{"type":"Sequence","children":[]}}}"#,
    )
    .unwrap();
    let st = bin().args(["zones"]).arg(&failed).output().unwrap();
    assert_eq!(st.status.code(), Some(1));
    let out = dir.path().join("o");
    let st = bin()
        .args([
            "generate",
            &fixture_path(common::TWO_ROBOT),
            "--flavour",
            "twincat3",
            "-o",
        ])
        .arg(&out)
        .output()
        .unwrap();
    assert_eq!(st.status.code(), Some(5));
    std::fs::create_dir_all(&out).unwrap();
    std::fs::write(out.join("x"), "x").unwrap();
    let st = bin()
        .args([
            "generate",
            &fixture_path(common::TWO_ROBOT),
            "--no-check",
            "-o",
        ])
        .arg(&out)
        .output()
        .unwrap();
    assert_eq!(st.status.code(), Some(4));
}

#[test]
fn zones_command_prints_the_zones() {
    let out = bin()
        .args(["zones", &fixture_path(common::WELDING)])
        .output()
        .unwrap();
    let text = String::from_utf8_lossy(&out.stdout);
    assert!(text.contains("IL01 = robot1.W01 > robot2.W15"), "{text}");
    assert!(text.contains("robot1: W01[IL01] W02[IL02]"), "{text}");
}
