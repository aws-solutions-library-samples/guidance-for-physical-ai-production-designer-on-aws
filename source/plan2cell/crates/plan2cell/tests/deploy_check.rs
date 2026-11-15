//! The deployability check on today's exports (tests/fixtures/deploy_check, trimmed copies of
//! a dev workspace's builds): scene_fit with the Hardware Configuration that made Start
//! production fail (ERR_SAFETY_TIMEOUT, gateway 0.0.0.0), the scene_fit export whose Folge wrote
//! P_PktNr_SPS (KSS error 2137 in the krl stage), and welding_cell_2_small's working export.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use plan2cell::deploy_check::{self, CheckReport, Controller, Dir, Fix, Globals, Status, Target};
use serde_json::json;

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/deploy_check").join(name)
}

/// The workspace's virtual cell (cell.toml of launch 11): robot1 kuka-robi1 .11, robot2 kuka-robi2 .12.
fn cell() -> Target {
    Target::virtual_cell(
        [("robot1", "kuka-robi1", "192.168.100.11"), ("robot2", "kuka-robi2", "192.168.100.12")]
            .into_iter()
            .map(|(ns, st, ip)| Controller {
                robot_ns: ns.into(),
                kli_ip: Some(ip.into()),
                station: Some(st.into()),
                host: format!("iiQKA.OS2_KSS_9.2.0_vass on ssm:i-{ns}"),
                globals: None,
            })
            .collect(),
    )
}

fn run(dir: &Path, target: Option<&Target>) -> CheckReport {
    deploy_check::check(&Dir(dir.to_path_buf()), &dir.display().to_string(), target)
}

fn rule<'a>(r: &'a CheckReport, id: &str) -> &'a deploy_check::RuleResult {
    r.rules.iter().find(|x| x.id == id).unwrap_or_else(|| panic!("no rule {id}: {}", r.text()))
}

fn hardware_fixes(r: &deploy_check::RuleResult) -> Vec<(String, serde_json::Value)> {
    r.fixes
        .iter()
        .filter_map(|f| match f {
            Fix::Hardware { field, value } => Some((field.clone(), value.clone())),
            _ => None,
        })
        .collect()
}

#[test]
fn scene_fit_fails_on_safety_gateway_and_simulators_with_the_fix() {
    let r = run(&fixture("scene_fit"), Some(&cell()));
    assert!(!r.ok, "{}", r.text());
    assert_eq!((r.plan_id.as_str(), r.profile.as_str()), ("16#A1BE", "vw-ks24"));
    let safety = rule(&r, "cell.safety");
    assert_eq!(safety.status, Status::Fail, "{}", r.text());
    assert!(safety.message.contains("ERR_SAFETY_TIMEOUT"), "{}", safety.message);
    assert!(safety.hint.as_deref().unwrap().contains("Hardware Configuration"), "{:?}", safety.hint);
    let fixes = hardware_fixes(safety);
    for ns in ["robot1", "robot2"] {
        assert!(fixes.contains(&(format!("robots.{ns}.safetyModule"), json!(false))), "{fixes:?}");
        assert!(fixes.contains(&(format!("robots.{ns}.safetyBypass"), json!(true))), "{fixes:?}");
        assert!(fixes.contains(&(format!("robots.{ns}.external"), json!(true))), "{fixes:?}");
    }
    let net = rule(&r, "cell.network");
    assert_eq!(net.status, Status::Fail);
    assert!(net.message.contains("0.0.0.0") && net.message.contains("DCP Set IP"), "{}", net.message);
    assert_eq!(hardware_fixes(net), [("network.gateway".to_string(), json!("192.168.100.1"))]);
    assert!(net.hint.as_deref().unwrap().contains("Gateway 192.168.100.1"), "{:?}", net.hint);
    // robot2 would get a CODESYS simulator container on the VC's address (External off).
    let sims = rule(&r, "cell.simulators");
    assert_eq!(sims.status, Status::Fail, "{}", r.text());
    assert_eq!(sims.robots, ["robot1", "robot2"]);
    // The addresses, the KRL names (BT_PKTNR since a2717d1) and the parts agree.
    for id in ["cell.addresses", "cell.robots", "krl.names", "export.files", "export.protocol"] {
        assert_eq!(rule(&r, id).status, Status::Pass, "{id}: {}", r.text());
    }
    // vw-ks24 has no PLAN_ID byte: the robots do not check it.
    assert_eq!(rule(&r, "export.plan_id").status, Status::Warn);
    assert_eq!(r.fails, 3, "{}", r.text());
    // Every failure here has a mechanical fix (re-exported afterwards).
    assert!(r.rules.iter().filter(|x| x.status == Status::Fail).all(|x| !x.fixes.is_empty()));
    assert!(r.text().contains("✗ Safety settings the KUKA VCs can satisfy"), "{}", r.text());
}

#[test]
fn the_p_pktnr_sps_folge_fails_before_the_deploy_as_a_fixed_generator_bug() {
    for target in [None, Some(cell())] {
        let r = run(&fixture("scene_fit_pktnr"), target.as_ref());
        let names = rule(&r, "krl.names");
        assert_eq!(names.status, Status::Fail, "{}", r.text());
        assert!(names.message.contains("P_PktNr_SPS") && names.message.contains("2137"), "{}", names.message);
        assert!(names.message.contains("KRC/R1/Folgen/folge11.src:50"), "{}", names.message);
        assert_eq!(names.robots, ["robot1", "robot2"]);
        assert_eq!(names.generator_bug.as_deref(), Some("generator bug fixed in a2717d1; re-export"));
        assert!(matches!(names.fixes.as_slice(), [Fix::Reexport { .. }]), "{:?}", names.fixes);
        assert!(names.details.iter().any(|d| d.contains("krl/robot2/KRC/R1/Folgen/folge11.src")), "{:?}", names.details);
    }
    // Cell-independent: the same finding as a build diagnostic.
    let r = run(&fixture("scene_fit_pktnr"), None);
    let d = r.diagnostics();
    assert!(d.iter().any(|d| d.severity == "error" && d.code.as_deref() == Some("krl.names")), "{d:?}");
}

#[test]
fn welding_cell_2_small_passes() {
    let r = run(&fixture("welding_cell_2_small"), Some(&cell()));
    assert!(r.ok, "{}", r.text());
    assert_eq!(r.warns, 0, "{}", r.text());
    assert!(r.rules.iter().all(|x| x.status == Status::Pass), "{}", r.text());
    assert_eq!(r.rules.len(), 10);
    assert!(r.diagnostics().is_empty());
    assert!(r.elapsed_s < 2.0, "{}", r.elapsed_s);
    // Without a cell: the five cell-independent rules.
    let r = run(&fixture("welding_cell_2_small"), None);
    assert!(r.ok && r.rules.len() == 5 && r.target.is_none(), "{}", r.text());
}

/// A copy of `name` to change.
fn copy(name: &str) -> tempfile::TempDir {
    let t = tempfile::tempdir().unwrap();
    fn cp(from: &Path, to: &Path) {
        std::fs::create_dir_all(to).unwrap();
        for e in std::fs::read_dir(from).unwrap() {
            let e = e.unwrap();
            let p = e.path();
            if p.is_dir() {
                cp(&p, &to.join(e.file_name()));
            } else {
                std::fs::copy(&p, to.join(e.file_name())).unwrap();
            }
        }
    }
    cp(&fixture(name), t.path());
    t
}

fn edit(dir: &Path, rel: &str, from: &str, to: &str) {
    let p = dir.join(rel);
    let text = std::fs::read_to_string(&p).unwrap();
    assert!(text.contains(from), "{rel}: no {from}");
    std::fs::write(&p, text.replacen(from, to, 1)).unwrap();
}

#[test]
fn mixed_parts_job_numbers_and_edited_krl_fail() {
    // A head PLC of another plan.
    let t = copy("welding_cell_2_small");
    edit(t.path(), "plc/codesys/head/GVL/GVL_Cell.st", "WORD#16#4F80", "WORD#16#4F81");
    let r = run(t.path(), Some(&cell()));
    assert_eq!(rule(&r, "export.plan_id").status, Status::Fail, "{}", r.text());
    assert!(rule(&r, "export.plan_id").details.iter().any(|d| d.contains("GVL_Cell")), "{}", r.text());
    // The robots read the job number as a byte, the head sends a WORD.
    let t = copy("welding_cell_2_small");
    edit(t.path(), "plc/robots/robot2/program.json", "\"job_number\": \"word_le\"", "\"job_number\": \"byte\"");
    let r = run(t.path(), None);
    let p = rule(&r, "export.protocol");
    assert_eq!(p.status, Status::Fail, "{}", r.text());
    assert!(p.message.contains("robot2"), "{}", p.message);
    // vw-ks24 with a WORD job number in the head.
    let t = copy("scene_fit");
    edit(t.path(), "plc/codesys/head/GVL/GVL_Cell.st", "JOB_NUMBER_BYTE         : BOOL := TRUE", "JOB_NUMBER_BYTE         : BOOL := FALSE");
    let r = run(t.path(), None);
    assert!(rule(&r, "export.protocol").details.iter().any(|d| d.contains("PRG_NR")), "{}", r.text());
    // A Folge edited after the export: the krl stage refuses it (hash).
    let t = copy("welding_cell_2_small");
    edit(t.path(), "krl/robot1/KRC/R1/Folgen/folge11.src", "DEF ", "DEF  ");
    let r = run(t.path(), None);
    assert_eq!(rule(&r, "export.files").status, Status::Fail, "{}", r.text());
    assert!(matches!(rule(&r, "export.files").fixes.as_slice(), [Fix::Reexport { .. }]));
    // No CODESYS projects at all.
    let t = copy("welding_cell_2_small");
    std::fs::remove_file(t.path().join("plc/project.json")).unwrap();
    assert_eq!(rule(&run(t.path(), None), "export.files").status, Status::Fail);
}

#[test]
fn robots_and_addresses_against_the_cells_controllers() {
    // A cell with one controller: robot2 has none.
    let mut one = cell();
    one.controllers.truncate(1);
    let r = run(&fixture("welding_cell_2_small"), Some(&one));
    let robots = rule(&r, "cell.robots");
    assert_eq!(robots.status, Status::Fail, "{}", r.text());
    assert!(robots.message.contains("robot2 has no KUKA VC"), "{}", robots.message);
    // Controllers on other addresses / stations than the export's devices.
    let mut moved = cell();
    moved.controllers[1].kli_ip = Some("192.168.100.22".into());
    moved.controllers[0].station = Some("kuka-r1".into());
    let r = run(&fixture("welding_cell_2_small"), Some(&moved));
    let a = rule(&r, "cell.addresses");
    assert_eq!(a.status, Status::Fail, "{}", r.text());
    let fixes = hardware_fixes(a);
    assert!(fixes.contains(&("robots.robot2.ip".to_string(), json!("192.168.100.22"))), "{fixes:?}");
    assert!(fixes.contains(&("robots.robot1.station".to_string(), json!("kuka-r1"))), "{fixes:?}");
    // Another head address and gateway.
    let mut other = cell();
    other.head_ip = "192.168.100.20".into();
    other.gateway = "0.0.0.0".into();
    let n = rule(&run(&fixture("welding_cell_2_small"), Some(&other)), "cell.network").clone();
    assert_eq!(n.status, Status::Fail);
    assert_eq!(
        hardware_fixes(&n),
        [("network.headIp".to_string(), json!("192.168.100.20")), ("network.gateway".to_string(), json!("0.0.0.0"))]
    );
}

#[test]
fn a_controllers_own_config_dat_is_used_when_it_was_read() {
    // A stock controller (no VW package): vw-ks24's SEL_RES, N_VW, ... are unknown there, and
    // re-exporting does not change that (not a fixed generator bug).
    let mut stock = cell();
    for c in &mut stock.controllers {
        c.globals = Some(Globals {
            source: "$config.dat of iiQKA.OS2_KSS_9.2.0 (stock)".into(),
            names: BTreeSet::from(["PRG_NR".to_string()]),
            read: true,
            error: None,
        });
    }
    let r = run(&fixture("scene_fit"), Some(&stock));
    let names = rule(&r, "krl.names");
    assert_eq!(names.status, Status::Fail, "{}", r.text());
    assert!(names.message.contains("SEL_RES"), "{}", names.message);
    assert!(names.fixes.is_empty() && names.generator_bug.as_deref().unwrap().contains("not fixed"), "{names:?}");
    assert!(names.details.iter().any(|d| d.contains("globals: robot1: $config.dat of")), "{:?}", names.details);
    // Unreadable: the bundled list, and why.
    let mut unread = cell();
    unread.controllers[0].globals =
        Some(Globals { source: String::new(), names: BTreeSet::new(), read: false, error: Some("ssm: timed out".into()) });
    let r = run(&fixture("scene_fit"), Some(&unread));
    assert_eq!(rule(&r, "krl.names").status, Status::Pass, "{}", r.text());
    assert!(rule(&r, "krl.names").message.contains("the bundled vw-ks24 list (ssm: timed out)"), "{}", r.text());
}

#[test]
fn a_build_in_memory_gets_the_findings_as_diagnostics() {
    // The cell-independent rules see only the files: a build of the P_PktNr_SPS Folge.
    let root = fixture("scene_fit_pktnr");
    let mut files = std::collections::BTreeMap::new();
    fn add(root: &Path, dir: &Path, files: &mut std::collections::BTreeMap<String, String>) {
        for e in std::fs::read_dir(dir).unwrap() {
            let p = e.unwrap().path();
            if p.is_dir() {
                add(root, &p, files);
            } else {
                let rel = p.strip_prefix(root).unwrap().to_string_lossy().replace('\\', "/");
                files.insert(rel, std::fs::read_to_string(&p).unwrap());
            }
        }
    }
    add(&root, &root, &mut files);
    let r = deploy_check::check_build(&files);
    assert_eq!(rule(&r, "krl.names").status, Status::Fail);
    assert!(r.target.is_none());
}

#[test]
#[ignore]
fn print_reports() {
    for n in ["scene_fit", "scene_fit_pktnr", "welding_cell_2_small"] {
        println!("{}", run(&fixture(n), Some(&cell())).text());
    }
}
