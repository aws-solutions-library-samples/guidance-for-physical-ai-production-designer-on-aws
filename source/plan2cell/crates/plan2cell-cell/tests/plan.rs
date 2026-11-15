//! The planned tool calls for examples/welding_cell_2_small against the input schemas of the
//! real tools/list (tests/fixtures/codesys-mcp-tools.json).

mod common;

use common::{fixture, manifest, manifest_without_safety_module, schema_errors, source};
use plan2cell_cell::codesys::{
    self, Call, Caps, Check, Deferred, EmptySlotAction, ModuleAction, Options,
};
use plan2cell_cell::config::IoCsv;

/// Options for our overlay's server (add_device moduleId, device parameter elementPath).
fn options(io_csv: IoCsv) -> Options {
    Options {
        dir: r"C:\ProgramData\plan2cell\projects\welding_cell_2_small".into(),
        install: "SP22 Patch 1".into(),
        io_csv,
        credentials: Some(("plan2cell".into(), "not-a-real-password".into())),
        caps: Caps::ALL,
        address_map: Default::default(),
    }
}

/// Options for the plain Codesys-MCP-Master 0.19.0.
fn plain(io_csv: IoCsv) -> Options {
    Options {
        caps: Caps::default(),
        ..options(io_csv)
    }
}

/// Every tool call of every stage, for both IO CSV routes, with the calls `ensure_module` and
/// `ensure_empty_slot` make once they know the device's children (the structure read itself is
/// not a tool), for the example and for it with `safety_module = false` on robot2.
fn all_calls() -> Vec<Call> {
    let mut v = calls_of(manifest());
    v.extend(calls_of(manifest_without_safety_module("kuka_robi2")));
    v
}

fn calls_of(m: plan2cell_cell::project::Manifest) -> Vec<Call> {
    let mut v = codesys::preflight(&m.device_ids(), &options(IoCsv::Mirror));
    v.extend(codesys::reset_ide(&options(IoCsv::Mirror)));
    for o in [IoCsv::Mirror, IoCsv::Staged]
        .into_iter()
        .flat_map(|io| [options(io), plain(io)])
    {
        for p in m.ordered() {
            for c in codesys::project(p, &source, &o).unwrap() {
                if let Check::Module(md) = &c.check {
                    let pf = o.project_path(p);
                    for a in [
                        ModuleAction::Plug,
                        ModuleAction::Rename("_x".into()),
                        ModuleAction::FreeList(0),
                    ] {
                        v.extend(md.calls(&a, &o, &pf));
                    }
                }
                if let Check::EmptySlot(es) = &c.check {
                    let pf = o.project_path(p);
                    let calls = es.calls(&EmptySlotAction::Unplug("_x".into()), &o, &pf);
                    assert_eq!(calls.len(), 2);
                    v.extend(calls.into_iter().filter(|c| c.tool != codesys::STRUCTURE));
                }
                if c.tool != codesys::STRUCTURE {
                    v.push(c);
                }
            }
            v.extend(codesys::deploy(p, &o));
            v.extend(codesys::verify(&m, p, &o));
            v.extend(codesys::online(p, &o));
            v.push(codesys::download_missing_libraries(p, &o));
            v.push(codesys::add_device_user(p, &o).unwrap());
        }
    }
    v
}

/// The fixture with the overlay's library tools.
fn overlay_fixture() -> common::Schemas {
    let mut f = fixture();
    common::add_library_tools(&mut f);
    f
}

/// The overlay's tools, which the plain server lacks ([`codesys::Caps`]).
const OVERLAY_TOOLS: [&str; 2] = [codesys::EXPORT_LIBRARIES, codesys::INSTALL_LIBRARIES];

#[test]
fn every_call_fits_its_schema() {
    assert_eq!(fixture().len(), 132);
    let tools = overlay_fixture();
    let mut overlay = Vec::new();
    let mut errs = Vec::new();
    for c in all_calls() {
        let deferred: Vec<(&str, &str)> = c
            .deferred
            .iter()
            .map(|(k, d)| (*k, d.json_type()))
            .collect();
        errs.extend(schema_errors(&tools, c.tool, &c.args, &deferred));
        if OVERLAY_TOOLS.contains(&c.tool) {
            overlay.push(c.tool);
            continue;
        }
        assert!(
            codesys::TOOLS.contains(&c.tool),
            "{} is not in codesys::TOOLS",
            c.tool
        );
    }
    // Both overlay tools were checked against their schemas.
    for t in OVERLAY_TOOLS {
        assert!(overlay.contains(&t), "{t} not planned");
    }
    errs.sort();
    errs.dedup();
    assert!(errs.is_empty(), "{errs:#?}");
    for t in codesys::TOOLS {
        assert!(tools.contains_key(*t), "codesys::TOOLS names {t}");
    }
}

#[test]
fn every_project_tool_takes_the_install() {
    let tools = overlay_fixture();
    for c in all_calls() {
        let has = tools[c.tool].1["properties"].get("install").is_some();
        assert_eq!(c.args.contains_key("install"), has, "{}", c.tool);
    }
}

fn tools_of(calls: &[Call]) -> Vec<&'static str> {
    calls.iter().map(|c| c.tool).collect()
}

#[test]
fn the_robot_simulator_sequence_on_the_plain_server() {
    let m = manifest();
    let o = plain(IoCsv::Mirror);
    let r1 = m.ordered()[0];
    assert_eq!(r1.name, "RobotSim_welding_cell_2_small_R1");
    let calls = codesys::project(r1, &source, &o).unwrap();
    let path = r"C:\ProgramData\plan2cell\projects\welding_cell_2_small\RobotSim_welding_cell_2_small_R1.project";
    assert!(
        calls
            .iter()
            .all(|c| c.args.get("projectFilePath").is_none_or(|p| p == path))
    );
    let shown: Vec<String> = calls.iter().map(|c| c.show()).collect();
    let head = [
        r#"find_device_description(deviceId="0000 0005", deviceType=4102, deviceVersion="4.20.0.0", install="SP22 Patch 1")"#,
        r#"find_device_description(deviceId="0000 0002", deviceType=110, deviceVersion="4.2.0.0", install="SP22 Patch 1")"#,
        r#"find_device_description(deviceId="0x019B_0x0301_DIM 2", deviceType=83, deviceVersion="1 V8.2", install="SP22 Patch 1")"#,
        &format!(
            r#"create_project(filePath={}, install="SP22 Patch 1", deviceName=<name of 4102 '0000 0005' '4.20.0.0'>)"#,
            serde_json::Value::String(path.into())
        ),
    ];
    assert_eq!(&shown[..4], head);
    let s = |i: usize| {
        shown[i].replace(
            &format!(
                ", projectFilePath={}",
                serde_json::Value::String(path.into())
            ),
            "",
        )
    };
    assert_eq!(
        s(5),
        r#"add_device(deviceName="Ethernet", install="SP22 Patch 1", parentPath="Device", targetVersion="4.2.0.0", targetDeviceName=<name of 110 '0000 0002' '4.2.0.0'>)"#
    );
    assert!(shown.iter().any(|c| c.starts_with("set_device_parameter(devicePath=\"Device/Ethernet\", install=\"SP22 Patch 1\", parameterId=0") && c.contains(r#"value="[192,168,100,11]""#)), "{shown:#?}");
    // A struct member: its index is read first.
    let i = shown
        .iter()
        .position(|c| c.contains("elementIndex=<index of StationName in parameter 2>"))
        .unwrap();
    assert!(
        shown[i - 1].starts_with(r#"get_device_parameter(devicePath="Device/Ethernet/kuka_robi1""#)
    );
    // A module: the device's children decide (plug an empty slot, rename what is there).
    let module = calls
        .iter()
        .find_map(|c| match &c.check {
            Check::Module(m) if m.name == "VASS_DIO256" => Some(m),
            _ => None,
        })
        .unwrap();
    assert_eq!(
        (
            module.device.as_str(),
            module.slot,
            module.module_id.as_str()
        ),
        ("Device/Ethernet/kuka_robi1", 2, "7")
    );
    let plug = module.calls(&ModuleAction::Plug, &o, path);
    assert!(plug[0].show().starts_with(r#"plug_device(deviceId="0x019B_0x0301_DIM 2", deviceType=83, deviceVersion="1 V8.2", install="SP22 Patch 1", moduleId="7", name="VASS_DIO256", parentPath="Device/Ethernet/kuka_robi1""#) && plug[0].show().contains("slotIndex=2"));
    assert!(
        shown.iter().any(|c| c.starts_with(&format!(
            "resources/read(uri=\"codesys://project/{}/structure\")",
            path.replace('\\', "\\\\")
        ))),
        "{shown:#?}"
    );

    let mut seq = tools_of(&calls);
    seq.dedup();
    assert_eq!(
        seq,
        [
            "find_device_description",
            "create_project",
            "get_device_identification",
            "add_device",
            "get_device_identification",
            "set_device_parameter",
            "list_device_parameters",
            "add_device",
            "get_device_identification",
            "get_device_parameter",
            "set_device_parameter",
            "list_device_parameters",
            "get_device_parameter",
            "resources/read",
            "add_library",
            "download_missing_libraries",
            "list_project_libraries",
            "create_dut",
            "set_pou_code",
            "create_dut",
            "set_pou_code",
            "create_dut",
            "set_pou_code",
            "create_dut",
            "set_pou_code",
            "create_dut",
            "set_pou_code",
            "create_dut",
            "set_pou_code",
            "create_dut",
            "set_pou_code",
            "create_dut",
            "set_pou_code",
            "create_dut",
            "set_pou_code",
            "create_dut",
            "set_pou_code",
            "create_dut",
            "set_pou_code",
            "create_dut",
            "set_pou_code",
            "create_gvl",
            "create_pou",
            "set_pou_code",
            "get_all_pou_code",
            "list_tasks",
            "create_task",
            "configure_task",
            "add_pou_to_task",
            "create_task",
            "configure_task",
            "create_task",
            "configure_task",
            "set_plc_settings",
            "list_tasks",
            "get_plc_settings",
            "create_gvl",
            "mirror_export",
            "import_io_mappings_csv",
            "delete_object",
            "compile_project",
            "save_project",
        ]
    );
    // MainTask comes with create_project: created only if missing, PLC_PRG added only if missing.
    let ct: Vec<&Call> = calls.iter().filter(|c| c.tool == "create_task").collect();
    assert_eq!(ct.len(), 3);
    assert!(ct.iter().all(|c| c.when.is_some()));
    // PLC_PRG of create_project is replaced, the other POUs are created.
    assert!(calls.iter().any(|c| c.tool == "set_pou_code"
        && c.args["pouPath"] == "Application/PLC_PRG"
        && c.args.contains_key("implementationCode")));
    assert!(
        !calls
            .iter()
            .any(|c| c.tool == "create_pou" && c.args["name"] == "PLC_PRG")
    );
    // The IO mapping CSV goes through the mirror.
    let imp = calls
        .iter()
        .find(|c| c.tool == "import_io_mappings_csv")
        .unwrap();
    assert_eq!(
        imp.args["csvPath"],
        r"C:\ProgramData\plan2cell\projects\welding_cell_2_small\robot1_io\Device\Plc Logic\Application\P2C_IoMappingCsv.st"
    );
    let carrier = calls
        .iter()
        .find(|c| c.tool == "create_gvl" && c.args["name"] == codesys::IO_CARRIER)
        .unwrap();
    assert!(
        carrier.args["declarationCode"]
            .as_str()
            .unwrap()
            .starts_with("//CoDeSys Mapping Export V1.2")
    );
    // DUT kinds come from the declaration.
    let dut = |n: &str| {
        calls
            .iter()
            .find(|c| c.tool == "create_dut" && c.args["name"] == n)
            .unwrap()
            .args["dutType"]
            .clone()
    };
    assert_eq!(dut("E_PlayerState"), "Enumeration");
    assert_eq!(dut("ST_Waypoint"), "Structure");
}

#[test]
fn the_head_plc_and_a_staged_csv_on_the_plain_server() {
    let m = manifest();
    let o = plain(IoCsv::Staged);
    let head = m.ordered()[2];
    let calls = codesys::project(head, &source, &o).unwrap();
    let adds: Vec<String> = calls
        .iter()
        .filter(|c| c.tool == "add_device")
        .map(|c| {
            format!(
                "{} <- {}",
                c.args["parentPath"].as_str().unwrap(),
                c.args["deviceName"].as_str().unwrap()
            )
        })
        .collect();
    assert_eq!(
        adds,
        [
            "Device <- Ethernet",
            "Device/Ethernet <- PN_Controller",
            "Device/Ethernet/PN_Controller <- kuka_robi1",
            "Device/Ethernet/PN_Controller <- kuka_robi2",
        ]
    );
    // Identification.StationName etc. of a GSDML device: three members of parameter 512, one
    // read of its sub-elements before, one after.
    let k1 = |t: &str| {
        calls
            .iter()
            .filter(|c| {
                c.tool == t
                    && c.args
                        .get("devicePath")
                        .is_some_and(|d| d == "Device/Ethernet/PN_Controller/kuka_robi1")
            })
            .count()
    };
    assert_eq!(k1("set_device_parameter"), 3);
    assert_eq!(k1("get_device_parameter"), 2);
    let m1: Vec<&Deferred> = calls.iter().flat_map(|c| c.deferred.iter().map(|(_, d)| d)).filter(|d| matches!(d, Deferred::MemberIndex { member, .. } if member == "Identification.IPAddress")).collect();
    assert_eq!(m1.len(), 2);
    let ps = calls.iter().find(|c| c.tool == "set_plc_settings").unwrap();
    assert_eq!(ps.args["devicePath"], "Device/Ethernet/PN_Controller");
    assert_eq!(ps.args["busCycleTask"], "Profinet_IOTask");
    assert!(!calls.iter().any(|c| c.tool == "mirror_export"));
    let imp = calls
        .iter()
        .find(|c| c.tool == "import_io_mappings_csv")
        .unwrap();
    assert_eq!(
        imp.args["csvPath"],
        r"C:\ProgramData\plan2cell\projects\welding_cell_2_small\head_io_mapping.csv"
    );
    // PN_Controller.CommCycle comes with the device: never added by hand.
    assert!(
        !calls
            .iter()
            .any(|c| c.tool == "add_pou_to_task"
                && c.args["pouName"].as_str().unwrap().contains('.'))
    );
}

#[test]
fn struct_members_and_modules_with_the_overlay() {
    let m = manifest();
    let o = options(IoCsv::Mirror);
    let (r1, head) = (m.ordered()[0], m.ordered()[2]);
    let shown = |p| -> Vec<String> {
        codesys::project(p, &source, &o)
            .unwrap()
            .iter()
            .map(|c| c.show())
            .collect()
    };
    let path = |p: &plan2cell_cell::project::Project| {
        format!(
            ", projectFilePath={}",
            serde_json::Value::String(o.project_path(p))
        )
    };
    // The robot simulator: StationName is a member one level down; written by path, no index
    // read first, and read back from the whole parameter.
    let r1s: Vec<String> = shown(r1).iter().map(|c| c.replace(&path(r1), "")).collect();
    let k = r#"devicePath="Device/Ethernet/kuka_robi1""#;
    let at = |c: &str| {
        r1s.iter()
            .position(|x| x == c)
            .unwrap_or_else(|| panic!("{c}\n{r1s:#?}"))
    };
    let set = at(&format!(
        r#"set_device_parameter({k}, elementPath="StationName", install="SP22 Patch 1", parameterId=2, value="'kuka-robi1'")"#
    ));
    assert!(
        !r1s[..set]
            .iter()
            .any(|c| c.starts_with("get_device_parameter("))
    );
    assert!(
        at(&format!(
            r#"get_device_parameter({k}, install="SP22 Patch 1", parameterId=2)"#
        )) > set
    );
    assert!(!r1s.iter().any(|c| c.contains("elementIndex")));
    // The head: Identification.* of parameter 512, read back from Identification.
    let hs: Vec<String> = shown(head)
        .iter()
        .map(|c| c.replace(&path(head), ""))
        .collect();
    let k1 = r#"devicePath="Device/Ethernet/PN_Controller/kuka_robi1""#;
    for (member, value) in [
        ("Identification.StationName", "'kuka-robi1'"),
        ("Identification.IPAddress", "[192,168,100,11]"),
        ("Identification.SubnetMask", "[255,255,255,0]"),
    ] {
        let c = format!(
            "set_device_parameter({k1}, elementPath={}, install=\"SP22 Patch 1\", parameterId=512, value={})",
            serde_json::Value::String(member.into()),
            serde_json::Value::String(value.into())
        );
        assert!(hs.contains(&c), "{c}\n{hs:#?}");
    }
    let reads: Vec<&String> = hs
        .iter()
        .filter(|c| c.starts_with(&format!("get_device_parameter({k1}")))
        .collect();
    assert_eq!(
        reads,
        [&format!(
            r#"get_device_parameter({k1}, elementPath="Identification", install="SP22 Patch 1", parameterId=512)"#
        )]
    );
    let calls = codesys::project(head, &source, &o).unwrap();
    assert!(
        calls
            .iter()
            .all(|c| c.deferred.iter().all(|(k, _)| *k != "elementIndex"))
    );
    let members = calls
        .iter()
        .find_map(|c| match &c.check {
            Check::Members { device, want, .. } if device.ends_with("kuka_robi2") => Some(want),
            _ => None,
        })
        .unwrap();
    assert_eq!(members.len(), 3);
    // A module of a device without slots: add_device with the module id, under the device,
    // by the device's own description.
    let module = codesys::project(r1, &source, &o)
        .unwrap()
        .into_iter()
        .find_map(|c| match c.check {
            Check::Module(m) if m.name == "SafeIO_FDIO64" => Some(m),
            _ => None,
        })
        .unwrap();
    assert!(module.add);
    let add = module.calls(&ModuleAction::FreeList(0), &o, &o.project_path(r1));
    assert_eq!(
        add[0].show().replace(&path(r1), ""),
        r#"add_device(deviceName="SafeIO_FDIO64", install="SP22 Patch 1", moduleId="1", parentPath="Device/Ethernet/kuka_robi1", targetVersion="1 V8.2", targetDeviceName=<name of 83 '0x019B_0x0301_DIM 2' '1 V8.2'>)"#
    );
    assert_eq!(add[1].tool, "list_device_parameters");
}

#[test]
fn deploy_and_verify() {
    let m = manifest();
    let o = options(IoCsv::Mirror);
    let head = m.ordered()[2];
    let d = codesys::deploy(head, &o);
    assert_eq!(
        tools_of(&d),
        [
            "rebind_device_to_scan_result",
            "save_project",
            "connect_to_device",
            "download_to_device",
            "create_boot_application",
            "start_stop_application",
            "get_application_state",
            "disconnect_from_device",
        ]
    );
    assert_eq!(d[0].args["matchAddress"], "192.168.100.10:11740");
    // A workspace's own cell: the gateway binds the runtime at its mapped address.
    let mapped = Options {
        address_map: [("192.168.100.10".to_string(), "172.31.5.11".to_string())].into(),
        ..o.clone()
    };
    assert_eq!(
        codesys::deploy(head, &mapped)[0].args["matchAddress"],
        "172.31.5.11:11740"
    );
    assert_eq!(d[2].args["deviceUser"], "plan2cell");
    assert_eq!(d[4].args["online"], true);
    assert!(d.iter().all(|c| !c.show().contains("not-a-real-password")));
    let v = codesys::verify(&m, head, &o);
    assert_eq!(
        tools_of(&v[..3]),
        [
            "rebind_device_to_scan_result",
            "save_project",
            "connect_to_device"
        ]
    );
    let reads = &v[4].args["expressions"];
    assert_eq!(
        reads,
        &serde_json::json!([
            "GVL_Cell.PLAN_ID",
            "PLC_PRG.bCellError",
            "kuka_robi1.xRunning",
            "kuka_robi2.xRunning"
        ])
    );
    let r1 = m.ordered()[0];
    assert_eq!(
        codesys::verify(&m, r1, &o)[4].args["expressions"],
        serde_json::json!(["GVL_Program.PLAN_ID", "PLC_PRG.bSimActive"]),
        "the simulator's PROFINET device has no xRunning"
    );
    // Without credentials the server's own are used.
    let none = Options {
        credentials: None,
        ..o
    };
    assert!(
        !codesys::deploy(head, &none)[1]
            .args
            .contains_key("deviceUser")
    );
    assert!(codesys::add_device_user(head, &none).is_none());
}

#[test]
fn without_the_safety_module_slot_1_is_unplugged() {
    let m = manifest_without_safety_module("kuka_robi2");
    let o = options(IoCsv::Mirror);
    let head = m.ordered()[2];
    let calls = codesys::project(head, &source, &o).unwrap();
    // DIM 3 is looked up in the device repository, preflight included.
    let dim3 = |c: &Call| {
        c.tool == "find_device_description"
            && c.args["deviceId"].as_str().unwrap().ends_with(" - DIM 3")
            && c.args["deviceVersion"] == "V8.3 - HW 1"
    };
    assert!(calls.iter().any(dim3));
    assert!(codesys::preflight(&m.device_ids(), &o).iter().any(dim3));
    let empty: Vec<&codesys::EmptySlot> = calls
        .iter()
        .filter_map(|c| match &c.check {
            Check::EmptySlot(e) => Some(e),
            _ => None,
        })
        .collect();
    assert_eq!(empty.len(), 1);
    let e = empty[0];
    assert_eq!(
        (e.device.as_str(), e.slot),
        ("Device/Ethernet/PN_Controller/kuka_robi2", 1)
    );
    // After add_device of kuka_robi2; no FDIO64 module for it.
    let add = calls
        .iter()
        .position(|c| c.tool == "add_device" && c.args["deviceName"] == "kuka_robi2")
        .unwrap();
    let at = calls
        .iter()
        .position(|c| matches!(c.check, Check::EmptySlot(_)))
        .unwrap();
    assert!(add < at);
    let modules: Vec<&str> = calls
        .iter()
        .filter_map(|c| match &c.check {
            Check::Module(md) => Some(md.name.as_str()),
            _ => None,
        })
        .collect();
    assert_eq!(
        modules,
        ["SafeIO_FDIO64_R1", "VASS_DIO256_R1", "VASS_DIO256_R2"]
    );
    let st = "- p (x)\n  - Device (x) {g}\n    - Ethernet (x) {g}\n      - PN_Controller (x) {g}\n        - kuka_robi2 (x) {g}\n          - _64_safe_digital_in_and_outputs (x) {g}\n          - VASS_DIO256_R2 (x) {g}\n";
    let a = e.action(st);
    assert_eq!(
        a,
        EmptySlotAction::Unplug("_64_safe_digital_in_and_outputs".into())
    );
    let path = o.project_path(head);
    let plan = e.calls(&a, &o, &path);
    let shown: Vec<String> = plan.iter().map(|c| c.show()).collect();
    assert_eq!(
        shown[0],
        format!(
            r#"unplug_device(devicePath="Device/Ethernet/PN_Controller/kuka_robi2/_64_safe_digital_in_and_outputs", install="SP22 Patch 1", projectFilePath={})"#,
            serde_json::Value::String(path.clone())
        )
    );
    assert_eq!(plan[1].tool, codesys::STRUCTURE);
    assert_eq!(plan[1].check, Check::SlotEmpty(e.clone()));
    let empty_now = st.replace("_64_safe_digital_in_and_outputs", "<Empty>");
    assert_eq!(e.action(&empty_now), EmptySlotAction::Empty);
    assert!(e.calls(&EmptySlotAction::Empty, &o, &path).is_empty());
}
