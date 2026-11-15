//! The stages on a cell of `factoryos/hook@1` pods (docs/tasks/cell-templates.md E7, E8): a
//! scripted fake head and fake robots behind the mock remote, no CODESYS (any MCP call panics).

use std::cell::RefCell;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::time::Duration;

use serde_json::{Map, Value, json};

use plan2cell_cell::config::Access;
use plan2cell_cell::hook::{HOOK, INBOX};
use plan2cell_cell::mcp::{ToolInfo, ToolOutput, Tools};
use plan2cell_cell::remote::{Call, Exec, MockRemote};
use plan2cell_cell::{Cell, CellConfig, KvcOp, Settings, Stage, Tree};

fn example() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/welding_cell_2_small")
}

/// A hook head must never reach the CODESYS MCP server.
struct NoMcp;

impl Tools for NoMcp {
    fn list(&mut self) -> plan2cell_cell::Result<Vec<ToolInfo>> {
        panic!("tools/list on a hook cell")
    }
    fn call(&mut self, tool: &str, _: &Map<String, Value>) -> plan2cell_cell::Result<ToolOutput> {
        panic!("{tool} on a hook cell")
    }
    fn read_resource(&mut self, uri: &str) -> plan2cell_cell::Result<ToolOutput> {
        panic!("resources/read {uri} on a hook cell")
    }
}

/// The smoke template's per-scene cell.toml (E8), no [codesys_mcp].
const CELL_TOML: &str = r#"
[plc_host]
driver = "hook"
kubectl_pod = "cell-jdoe/n1-head/head"
kubeconfig = "/var/lib/factoryos/cell/kubeconfig"
region = "eu-central-1"

[kuka_vc]
kubeconfig = "/var/lib/factoryos/cell/kubeconfig"
region = "eu-central-1"

[[kuka_vc.controller]]
driver = "hook"
robot_ns = "robot1"
station = "kuka-robi1"
kli_ip = "10.77.1.11"
kubectl_pod = "cell-jdoe/n1-robot-1/robot"

[[kuka_vc.controller]]
driver = "hook"
robot_ns = "robot2"
station = "kuka-robi2"
kli_ip = "10.77.1.12"
kubectl_pod = "cell-jdoe/n1-robot-2/robot"

[address_map]

[profinet]
gateway = "10.77.1.1"
head_ip = "10.77.1.10"
netmask = "255.255.255.0"
"#;

fn quick() -> Settings {
    Settings {
        verify_timeout: Duration::ZERO,
        verify_interval: Duration::from_millis(1),
        start_timeout: Duration::from_secs(10),
        start_interval: Duration::from_millis(1),
        hook_retry: Duration::ZERO,
        ..Settings::default()
    }
}

/// An answer: the envelope with `extra`'s fields, after a note.
fn answer(verb: &str, exit: i32, extra: Value) -> Exec {
    let mut env = json!({"schema": "factoryos/hook@1", "verb": verb, "ok": exit == 0, "exit": exit,
                         "role": "x", "component": "fake", "hook": "fake 0", "elapsed_ms": 1});
    if let (Some(e), Some(x)) = (env.as_object_mut(), extra.as_object()) {
        e.extend(x.clone());
    }
    Exec::exit(exit, format!("note: {verb} by the fake\n{env}\n"))
}

/// What a fake does: its state, the requests it got, verbs made to fail.
#[derive(Default)]
struct State {
    /// head: CS_INIT after deploy, the first state after start, CS_RUNNING after a pulse.
    after_start: &'static str,
    deployed: bool,
    started: bool,
    cell: String,
    requests: Vec<(String, Value)>,
    /// verb -> its answer instead.
    fail: HashMap<String, Exec>,
    /// Names the head answers in `missing`.
    missing: Vec<String>,
    /// robot: the program running (deploy refuses while one runs; `op program cancel` ends it).
    running: Option<String>,
}

type Shared = Rc<RefCell<State>>;

fn fake(role: &'static str, ns: &'static str, s: Shared) -> MockRemote {
    MockRemote::default().on_exec(move |argv, stdin| {
        assert_eq!(argv[0], HOOK, "only the hook runs: {argv:?}");
        let verb = argv[1].clone();
        let req: Value = serde_json::from_slice(stdin).expect("compact JSON on stdin");
        let mut st = s.borrow_mut();
        st.requests.push((verb.clone(), req.clone()));
        if let Some(a) = st.fail.get(&verb) {
            return Ok(a.clone());
        }
        let un = || answer(&verb, 2, json!({"error": format!("unimplemented: {verb} for {role}")}));
        Ok(match (role, verb.as_str()) {
            (_, "describe") => answer("describe", 0, json!({"contract": "factoryos/hook@1", "role": role, "runtime": "fake", "verbs": [], "versions": {}})),
            (_, "ready") => answer("ready", 0, json!({"ready": true, "checks": [{"name": "init", "ok": true}], "info": {"ar": "none"}})),
            (_, "health") => answer("health", 0, json!({"runtime": {"schema": "plan2cell/cell-runtime@1", "ok": true, "runtimes": []}})),
            ("head-plc", "deploy") => {
                st.deployed = true;
                st.cell = "CS_INIT".into();
                answer("deploy", 0, json!({"built": true, "planId": "16#F77E", "deployment": "d1-abcd", "unchanged": false, "errors": [], "warnings": []}))
            }
            ("head-plc", "start") if !st.deployed => answer("start", 3, json!({"error": "nothing deployed"})),
            ("head-plc", "start") => {
                st.started = true;
                st.cell = st.after_start.into();
                answer("start", 0, json!({"state": "running", "planId": "16#F77E", "deployment": "d1-abcd", "restart": "cold"}))
            }
            ("head-plc", "read") => {
                let mut values = Map::new();
                let mut missing = Vec::new();
                for n in req["vars"].as_array().unwrap() {
                    let n = n.as_str().unwrap();
                    if st.missing.iter().any(|m| m == n) {
                        missing.push(n);
                        continue;
                    }
                    let v = match n {
                        "PLC_PRG.eCellState" => format!("E_CellState.{}", st.cell),
                        "PLC_PRG.fbCell.eError" => if st.cell == "CS_ERROR" { "E_CellError.ERR_OPERATOR_STOP".into() } else { "E_CellError.ERR_NONE".into() },
                        "PLC_PRG.fbCell.nErrorRobot" => "0".into(),
                        "PLC_PRG.bRecoveryRequired" | "PLC_PRG.bCellError" => "FALSE".into(),
                        "PLC_PRG.udiCycles" => "3".into(),
                        "GVL_Cell.PLAN_ID" => "16#F77E".into(),
                        x if x.ends_with(".xRunning") => "TRUE".into(),
                        _ => {
                            missing.push(n);
                            continue;
                        }
                    };
                    values.insert(n.into(), Value::String(v));
                }
                answer("read", 0, json!({"values": values, "missing": missing}))
            }
            ("head-plc", "write") => {
                let names: Vec<String> = req["vars"].as_object().unwrap().keys().cloned().collect();
                if names.iter().any(|n| n == "PLC_PRG.bReset" || n == "PLC_PRG.bRecoveryConfirm") {
                    st.cell = "CS_RUNNING".into();
                }
                answer("write", 0, json!({"written": names, "refused": []}))
            }
            ("head-plc", "stop") => {
                st.cell = "CS_READY".into();
                answer("stop", 0, json!({"pulsed": "PLC_PRG.bStop", "values": {"PLC_PRG.eCellState": "E_CellState.CS_READY", "PLC_PRG.udiCycles": "3"}, "runtime_state": "running"}))
            }
            ("robot-controller", "deploy") if st.running.is_some() => {
                assert_eq!(req["force"], false, "never force");
                let p = st.running.clone().unwrap();
                answer("deploy", 3, json!({"error": format!("a robot program is running ({p}); stop it or give force")}))
            }
            ("robot-controller", "deploy") => {
                assert_eq!(req, json!({"bundle": "krl-bundle.json", "force": false, "dry_run": false}));
                st.deployed = true;
                answer("deploy", 0, json!({"tag": format!("{ns} 16#F77E"), "changed": ["KRC:\\R1\\Program\\bt_cell.src"], "unchanged": 8, "errors": [], "messages": [], "confirmed": [], "notes": []}))
            }
            ("robot-controller", "verify") if !st.deployed => answer("verify", 3, json!({"error": "nothing deployed"})),
            ("robot-controller", "verify") => answer("verify", 0, json!({"tag": format!("{ns} 16#F77E"), "checks": [
                {"expr": "active_instance", "expect": format!("hook-stub/{ns}"), "value": format!("hook-stub/{ns}"), "ok": true},
                {"expr": "compile_errors", "expect": "0", "value": "0", "ok": true}], "errors": [], "messages": []})),
            ("robot-controller", "prepare") if !st.deployed => answer("prepare", 3, json!({"error": "no bundle installed"})),
            ("robot-controller", "prepare") => answer("prepare", 0, json!({"mode_op": "#EX", "pro_state0": "#P_ACTIVE", "pro_state1": "#P_RESET", "pro_name1": "BT_CELL"})),
            ("robot-controller", "op") if req["name"] == "program" && req["arg"] == "cancel" => {
                st.running = None;
                answer("op", 0, json!({"op": "program", "arg": "cancel", "changes": true, "result": {"pro_state1": "#P_FREE"}}))
            }
            ("robot-controller", "op") => answer("op", 0, json!({"op": req["name"], "arg": req["arg"], "changes": true, "result": {"mode_op": "#EX"}})),
            ("robot-controller", "globals") => answer("globals", 0, json!({"source": "fake", "names": ["BT_X"], "read": true, "text": ""})),
            _ => un(),
        })
    })
}

struct Rig {
    head: Shared,
    robots: Vec<Shared>,
    head_remote: MockRemote,
    robot_remotes: Vec<Rc<RefCell<MockRemote>>>,
}

fn rig(after_start: &'static str) -> Rig {
    let head = Rc::new(RefCell::new(State { after_start, ..State::default() }));
    let robots: Vec<Shared> = (0..2).map(|_| Shared::default()).collect();
    Rig {
        head_remote: fake("head-plc", "", head.clone()),
        robot_remotes: vec![
            Rc::new(RefCell::new(fake("robot-controller", "robot1", robots[0].clone()))),
            Rc::new(RefCell::new(fake("robot-controller", "robot2", robots[1].clone()))),
        ],
        head,
        robots,
    }
}

fn verbs(s: &Shared) -> Vec<String> {
    s.borrow().requests.iter().map(|(v, _)| v.clone()).collect()
}

/// Run `stages` on the rig: the result, the report.
fn run(r: &mut Rig, stages: &[Stage], settings: Settings) -> (plan2cell_cell::Result<()>, plan2cell_cell::Report) {
    let cfg = CellConfig::parse(CELL_TOML).unwrap();
    let mut map: HashMap<Access, Rc<RefCell<MockRemote>>> = HashMap::new();
    map.insert(Access::Kubectl("cell-jdoe/n1-robot-1/robot".into()), r.robot_remotes[0].clone());
    map.insert(Access::Kubectl("cell-jdoe/n1-robot-2/robot".into()), r.robot_remotes[1].clone());
    let mut tools = NoMcp;
    let mut c = Cell::new(cfg, Some(Tree::Dir(example())), settings, &mut r.head_remote, &mut tools, None)
        .unwrap()
        .with_controllers(Box::new(map));
    let res = c.run(stages);
    (res, c.report.clone())
}

fn notes(rep: &plan2cell_cell::Report, st: Stage) -> Vec<String> {
    rep.stages.iter().find(|s| s.stage == st).map(|s| s.notes.clone()).unwrap_or_default()
}

#[test]
fn the_smoke_cell_deploys_and_starts_through_the_hooks() {
    let mut r = rig("CS_ERROR");
    let (res, rep) = run(&mut r, &[Stage::Deploy, Stage::Krl, Stage::Start], quick());
    res.unwrap();
    assert!(rep.ok);
    // The head: describe once (first contact), the bundle copied first, then deploy with the
    // wiring of cell.toml; start after the robots; the pulse loop over read / write.
    let hv = verbs(&r.head);
    assert_eq!(&hv[..2], ["describe", "deploy"], "{hv:?}");
    assert_eq!(hv.iter().filter(|v| *v == "describe").count(), 1);
    let deploy = r.head.borrow().requests[1].1.clone();
    assert_eq!(deploy["bundle"], "head-bundle.json");
    assert_eq!(deploy["force"], false);
    assert_eq!(
        deploy["wiring"],
        json!({"head_ip": "10.77.1.10", "netmask": "255.255.255.0", "gateway": "10.77.1.1", "controllers": [
            {"robot_ns": "robot1", "station": "kuka-robi1", "kli_ip": "10.77.1.11"},
            {"robot_ns": "robot2", "station": "kuka-robi2", "kli_ip": "10.77.1.12"}]})
    );
    let copies = &r.head_remote.copied;
    assert_eq!(copies.len(), 1);
    assert_eq!(copies[0].0, format!("{INBOX}/head-bundle.json"));
    let bundle: Value = serde_json::from_slice(&copies[0].1).unwrap();
    assert_eq!((bundle["schema"].as_str(), bundle["plan_id"].as_str(), bundle["key"].as_str()), (Some("factoryos/head-bundle@1"), Some("16#F77E"), Some("head")));
    assert!(bundle["files"]["plc/trust/head/src/PLC_PRG.st"].is_string());
    // Robots: krl-bundle@1 into the inbox, deploy, verify; at start ready + prepare.
    for (i, ns) in ["robot1", "robot2"].iter().enumerate() {
        assert_eq!(verbs(&r.robots[i]), ["describe", "deploy", "verify", "ready", "prepare"], "{ns}");
        let rr = r.robot_remotes[i].borrow();
        assert_eq!(rr.copied.len(), 1);
        assert_eq!(rr.copied[0].0, format!("{INBOX}/krl-bundle.json"));
        let b: Value = serde_json::from_slice(&rr.copied[0].1).unwrap();
        assert_eq!((b["schema"].as_str(), b["robot_ns"].as_str()), (Some("plan2cell/krl-bundle@1"), Some(*ns)));
        // Every exec is the hook's argv, never a shell line.
        assert!(rr.calls.iter().all(|c| match c {
            Call::Exec(a, _) => a[0] == HOOK && a.len() == 2,
            Call::Copy(..) => true,
            Call::Run(x) => panic!("a shell command on a hook pod: {x}"),
        }));
    }
    let robot_prepare = r.robots[0].borrow().requests.iter().find(|(v, _)| v == "prepare").unwrap().1.clone();
    assert_eq!(robot_prepare, json!({"mode": "EXT", "ack": true, "force": false}));
    assert!(r.head.borrow().started);
    let start = r.head.borrow().requests.iter().find(|(v, _)| v == "start").unwrap().1.clone();
    assert_eq!(start, json!({"restart": "cold"}));
    let w = r.head.borrow().requests.iter().find(|(v, _)| v == "write").unwrap().1.clone();
    assert_eq!(w, json!({"vars": {"PLC_PRG.bReset": true}}));
    let n = notes(&rep, Stage::Start);
    let prep = n.iter().position(|x| x == "robot1 (hook): prepare: #EX, #P_ACTIVE, program BT_CELL").unwrap();
    let running = n.iter().position(|x| x.starts_with("head running plan 16#F77E")).unwrap();
    let pulsed = n.iter().position(|x| x.ends_with("PLC_PRG.bReset pulsed")).unwrap();
    assert!(prep < running && running < pulsed, "{n:#?}");
    assert!(n.last().unwrap().starts_with("in production after 2 read(s) and 1 pulse(s): CS_RUNNING"), "{n:#?}");
    assert!(n.iter().any(|x| x == "head: start by the fake"), "the hook's notes are the stage's: {n:#?}");
    assert_eq!(rep.controllers.len(), 2);
    assert!(rep.controllers.iter().all(|c| c.deployed == Some(true) && c.verified == Some(true) && c.instance == "hook"));
    let head = rep.projects.iter().find(|p| p.name == "HeadPLC_welding_cell_2_small").unwrap();
    assert_eq!((head.deployed, head.state.as_deref()), (Some(true), Some("running")));
    // The robot simulators of the export are left out (their robots are hook controllers).
    assert!(!rep.projects.iter().any(|p| p.name.starts_with("RobotSim")));
    assert!(rep.calls.iter().any(|c| c == "kubectl:cell-jdoe/n1-head/head: hook start {\"restart\":\"cold\"}"), "{:#?}", rep.calls);
}

#[test]
fn up_runs_on_a_hook_cell_without_codesys() {
    let mut r = rig("CS_RUNNING");
    let (res, rep) = run(&mut r, &Stage::ALL, quick());
    res.unwrap();
    assert!(notes(&rep, Stage::Runtimes).iter().any(|n| n.starts_with("nothing to do: the head is a factoryos/hook@1 pod")));
    assert!(notes(&rep, Stage::Projects).iter().any(|n| n.starts_with("nothing to do")));
    // verify: the robots' verify, then the head's acceptance values with read.
    let reads: Vec<Value> = r.head.borrow().requests.iter().filter(|(v, _)| v == "read").map(|(_, r)| r.clone()).collect();
    assert_eq!(reads.len(), 1);
    let vars = reads[0]["vars"].as_array().unwrap();
    assert!(vars.contains(&json!("GVL_Cell.PLAN_ID")) && vars.contains(&json!("PLC_PRG.bCellError")));
    assert!(vars.iter().any(|v| v.as_str().unwrap().ends_with(".xRunning")));
    let head = rep.projects.iter().find(|p| p.name == "HeadPLC_welding_cell_2_small").unwrap();
    assert_eq!(head.verified, Some(true));
    assert!(head.checks.iter().all(|c| c.ok) && !head.checks.is_empty());
}

#[test]
fn a_hook_build_error_is_exit_8_and_unimplemented_is_exit_3() {
    let mut r = rig("CS_RUNNING");
    r.robots[1].borrow_mut().fail.insert(
        "deploy".into(),
        answer("deploy", 8, json!({"error": "1 compile error(s)", "changed": [], "errors": [{"module": "KRC:\\R1\\Program\\bt_cell.src", "line": 12, "number": 2135, "param": "FOLGE12"}]})),
    );
    let (res, rep) = run(&mut r, &[Stage::Krl], quick());
    let e = res.unwrap_err();
    assert_eq!(e.exit_code(), 8, "{e}");
    assert!(e.to_string().contains("bt_cell.src:12 error 2135 FOLGE12"), "{e}");
    assert_eq!(rep.controllers[1].deployed, Some(false));
    // A head that does not implement deploy: hook exit 2 is stage exit 3 (not plan2cell's usage 2).
    let mut r = rig("CS_RUNNING");
    r.head.borrow_mut().fail.insert("deploy".into(), answer("deploy", 2, json!({"error": "unimplemented: deploy for head-plc"})));
    let e = run(&mut r, &[Stage::Deploy], quick()).0.unwrap_err();
    assert_eq!(e.exit_code(), 3);
    assert!(e.to_string().contains("the image does not implement what the template declares"), "{e}");
}

#[test]
fn a_missing_or_inconsistent_answer_is_exit_4_and_retried_only_when_reading() {
    // deploy (changing): no answer, sent once.
    let mut r = rig("CS_RUNNING");
    r.head.borrow_mut().fail.insert("deploy".into(), Exec::exit(1, "error: unable to upgrade connection\n"));
    let e = run(&mut r, &[Stage::Deploy], quick()).0.unwrap_err();
    assert_eq!(e.exit_code(), 4);
    assert!(e.to_string().contains("hook answer missing or inconsistent"), "{e}");
    assert_eq!(verbs(&r.head).iter().filter(|v| *v == "deploy").count(), 1);
    // ready (reading) answers another verb's envelope: retried once, then exit 4.
    let mut r = rig("CS_RUNNING");
    r.head.borrow_mut().fail.insert("ready".into(), answer("health", 0, json!({})));
    let e = run(&mut r, &[Stage::Preflight], quick()).0.unwrap_err();
    assert_eq!(e.exit_code(), 4);
    assert_eq!(verbs(&r.head), ["describe", "ready", "ready"]);
}

#[test]
fn another_role_or_contract_is_refused_at_first_contact() {
    let mut r = rig("CS_RUNNING");
    r.robots[0].borrow_mut().fail.insert(
        "describe".into(),
        answer("describe", 0, json!({"contract": "factoryos/hook@1", "role": "head-plc"})),
    );
    let e = run(&mut r, &[Stage::Krl], quick()).0.unwrap_err();
    assert_eq!(e.exit_code(), 3);
    assert!(e.to_string().contains("role \"head-plc\", the cell drives it as robot-controller"), "{e}");
    assert_eq!(verbs(&r.robots[0]), ["describe"], "nothing after a refused describe");
    assert!(r.robot_remotes[0].borrow().copied.is_empty(), "no bundle before describe");
}

#[test]
fn a_missing_name_shows_as_a_question_mark_and_the_state_must_read() {
    // truST cannot read nErrorRobot (handoff E10): `?`, the start still succeeds.
    let mut r = rig("CS_RECOVERY");
    r.head.borrow_mut().missing = vec!["PLC_PRG.fbCell.nErrorRobot".into()];
    r.head.borrow_mut().deployed = true;
    for s in &r.robots {
        s.borrow_mut().deployed = true;
    }
    let (res, rep) = run(&mut r, &[Stage::Start], quick());
    res.unwrap();
    let n = notes(&rep, Stage::Start);
    assert!(n.iter().any(|x| x.ends_with("PLC_PRG.bRecoveryConfirm pulsed")), "{n:#?}");
    // Without E_CellState: exit 7, before any pulse.
    let mut r = rig("CS_ERROR");
    r.head.borrow_mut().missing = vec!["PLC_PRG.eCellState".into()];
    r.head.borrow_mut().deployed = true;
    for s in &r.robots {
        s.borrow_mut().deployed = true;
    }
    let e = run(&mut r, &[Stage::Start], quick()).0.unwrap_err();
    assert_eq!(e.exit_code(), 7);
    assert!(e.to_string().contains("PLC_PRG.eCellState not read"), "{e}");
    assert!(!verbs(&r.head).contains(&"write".to_string()));
}

#[test]
fn start_before_deploy_is_a_precondition_and_a_refused_write_is_exit_3() {
    let mut r = rig("CS_ERROR");
    for s in &r.robots {
        s.borrow_mut().deployed = true;
    }
    let e = run(&mut r, &[Stage::Start], quick()).0.unwrap_err();
    assert_eq!(e.exit_code(), 3);
    assert!(e.to_string().contains("nothing deployed"), "{e}");
    let mut r = rig("CS_ERROR");
    r.head.borrow_mut().deployed = true;
    for s in &r.robots {
        s.borrow_mut().deployed = true;
    }
    r.head.borrow_mut().fail.insert("write".into(), answer("write", 0, json!({"written": [], "refused": ["PLC_PRG.bReset"]})));
    let e = run(&mut r, &[Stage::Start], quick()).0.unwrap_err();
    assert_eq!(e.exit_code(), 3);
    assert!(e.to_string().contains("refused PLC_PRG.bReset"), "{e}");
}

#[test]
fn stop_uses_the_hooks_stop() {
    let mut r = rig("CS_RUNNING");
    let (res, rep) = run(&mut r, &[Stage::Stop], quick());
    res.unwrap();
    assert_eq!(verbs(&r.head), ["describe", "stop"]);
    assert_eq!(r.head.borrow().requests[1].1, json!({"runtime": false}));
    let n = notes(&rep, Stage::Stop);
    assert!(n.contains(&"PLC_PRG.bStop pulsed".to_string()) && n.last().unwrap().starts_with("CS_READY"), "{n:#?}");
}

#[test]
fn a_kvc_op_on_a_hook_robot_is_hook_op() {
    let mut r = rig("CS_RUNNING");
    let cfg = CellConfig::parse(CELL_TOML).unwrap();
    let mut map: HashMap<Access, Rc<RefCell<MockRemote>>> = HashMap::new();
    map.insert(Access::Kubectl("cell-jdoe/n1-robot-2/robot".into()), r.robot_remotes[1].clone());
    let mut tools = NoMcp;
    let mut c = Cell::new(cfg, None, quick(), &mut r.head_remote, &mut tools, None).unwrap().with_controllers(Box::new(map));
    let v = c.kvc_op("robot2", &KvcOp::parse("mode", Some("ext")).unwrap()).unwrap();
    assert_eq!(v["ok"], true);
    assert_eq!(v["changes"], true);
    assert_eq!(v["controller"], Value::Null);
    assert_eq!(v["result"]["result"]["mode_op"], "#EX");
    let req = r.robots[1].borrow().requests.last().unwrap().clone();
    assert_eq!(req, ("op".to_string(), json!({"name": "mode", "arg": "EXT"})));
}

#[test]
fn a_dry_run_sends_nothing_to_the_hooks() {
    let mut r = rig("CS_RUNNING");
    let settings = Settings { dry_run: true, ..quick() };
    let (res, rep) = run(&mut r, &[Stage::Deploy, Stage::Krl, Stage::Start], settings);
    res.unwrap();
    assert!(r.head_remote.calls.is_empty() && r.robot_remotes.iter().all(|x| x.borrow().calls.is_empty()));
    let planned: Vec<&str> = rep.calls.iter().map(String::as_str).collect();
    assert!(planned.iter().any(|c| c.starts_with("kubectl:cell-jdoe/n1-head/head: copy ") && c.ends_with(" /var/lib/factoryos/inbox/head-bundle.json")), "{planned:#?}");
    assert!(planned.iter().any(|c| c.starts_with("kubectl:cell-jdoe/n1-robot-1/robot: hook deploy {\"bundle\":\"krl-bundle.json\"")), "{planned:#?}");
    assert!(planned.contains(&"kubectl:cell-jdoe/n1-robot-2/robot: hook prepare {\"ack\":true,\"force\":false,\"mode\":\"EXT\"}"), "{planned:#?}");
    assert!(planned.iter().any(|c| c.starts_with("kubectl:cell-jdoe/n1-head/head: hook read ")), "{planned:#?}");
}

#[test]
fn a_hook_heads_runtime_is_its_health() {
    let s = Shared::default();
    let mut remote = fake("head-plc", "", s.clone());
    let rt = plan2cell_cell::runtime::read_hook(&mut remote, Duration::ZERO).unwrap();
    assert_eq!(rt.schema, "plan2cell/cell-runtime@1");
    assert_eq!(verbs(&s), ["describe", "health"]);
}

/// The launch's restore left the previous build's BT_CELL running on a robot: Run Control Code,
/// having deployed the head, cancels exactly that program (op program cancel) and deploys again,
/// never with force.
#[test]
fn bt_cell_running_after_the_head_deploy_is_cancelled_and_deployed_again() {
    let mut r = rig("CS_RUNNING");
    r.robots[0].borrow_mut().running = Some("BT_CELL".into());
    let (res, rep) = run(&mut r, &[Stage::Deploy, Stage::Krl], quick());
    res.unwrap();
    assert_eq!(verbs(&r.robots[0]), ["describe", "deploy", "op", "deploy", "verify"]);
    let op = r.robots[0].borrow().requests[2].1.clone();
    assert_eq!((op["name"].as_str(), op["arg"].as_str()), (Some("program"), Some("cancel")));
    assert!(r.robots[0].borrow().requests.iter().filter(|(v, _)| v == "deploy").all(|(_, q)| q["force"] == false));
    assert_eq!(verbs(&r.robots[1]), ["describe", "deploy", "verify"]);
    assert!(notes(&rep, Stage::Krl).iter().any(|n| n.contains("robot1: BT_CELL of the previous build ran after the head's deploy")), "{:?}", notes(&rep, Stage::Krl));
    assert!(rep.controllers.iter().all(|c| c.deployed == Some(true)));
}

/// Without a head deploy in this run, or with another program running, the refusal stands and
/// nothing is cancelled.
#[test]
fn a_running_program_is_cancelled_only_for_bt_cell_after_the_head_deploy() {
    // krl alone (no head deploy in this run): exit 3, no op.
    let mut r = rig("CS_RUNNING");
    r.robots[0].borrow_mut().running = Some("BT_CELL".into());
    let e = run(&mut r, &[Stage::Krl], quick()).0.unwrap_err();
    assert_eq!(e.exit_code(), 3, "{e}");
    assert!(e.to_string().contains("a robot program is running (BT_CELL)"), "{e}");
    assert!(!verbs(&r.robots[0]).contains(&"op".to_string()));
    // Another program (a brake test) after the head deploy: exit 3, no op.
    let mut r = rig("CS_RUNNING");
    r.robots[0].borrow_mut().running = Some("KRC:\\R1\\TP\\BRAKETEST\\BRAKETEST_REQ".into());
    let e = run(&mut r, &[Stage::Deploy, Stage::Krl], quick()).0.unwrap_err();
    assert_eq!(e.exit_code(), 3, "{e}");
    assert!(!verbs(&r.robots[0]).contains(&"op".to_string()));
}
