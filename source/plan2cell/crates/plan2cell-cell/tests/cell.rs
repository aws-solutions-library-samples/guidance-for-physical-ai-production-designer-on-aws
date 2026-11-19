//! The stages end to end against a mock CODESYS MCP server (streamable HTTP, bearer token, the
//! fixture tools with their input schemas, answers in the real server's text formats, a small
//! model of each project) and a mock PLC host, on examples/welding_cell_2_small.

mod common;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use rmcp::model::{
    CallToolRequestParams, CallToolResponse, CallToolResult, ContentBlock, ListToolsResult,
    PaginatedRequestParams, ReadResourceRequestParams, ReadResourceResponse, ReadResourceResult,
    ResourceContents, ServerCapabilities, ServerConfig, Tool,
};
use rmcp::service::RequestContext;
use rmcp::{ErrorData, RoleServer, ServerHandler};
use serde_json::{Map, Value};

use common::{
    KUKA_DIM3, Schemas, example, example_without_safety_module, fixture, manifest, schema_errors,
};
use plan2cell_cell::codesys::{self, IO_CARRIER};
use plan2cell_cell::config::{CellConfig, CodesysMcp, OtherCodesys, PlcHost};
use plan2cell_cell::mcp::{McpTools, Tools};
use plan2cell_cell::project::{DeviceId, DeviceNode, Manifest, Project};
use plan2cell_cell::remote::{Call, MockRemote};
use plan2cell_cell::{Cell, Settings, Stage, Tree};

const TOKEN: &str = "0123456789abcdef0123456789abcdef";

#[derive(Clone, Default)]
struct Behaviour {
    /// find_device_description reports these not installed.
    missing_devices: Vec<DeviceId>,
    /// A CODESYS opened by hand blocks the MCP server's.
    interactive: bool,
    /// A CODESYS the server did not start is in its way (its question ends the line with this
    /// unsaved state) until `launch_codesys` `otherInstances: close` closes it.
    stray: Option<&'static str>,
    /// The stray CODESYS is one an ended MCP session left (else opened by hand), and the server
    /// takes it over by itself after this many questions (0: never).
    stray_left: Option<usize>,
    fail_build: Option<String>,
    /// The first compile_project of this project reports library-open errors (versions the
    /// repository lacks), the next builds clean (live, 2026-11-02: start's first build).
    library_errors_once: Option<String>,
    /// Every compile_project of this project before the login reports library-open errors
    /// (live, 2026-11-03: the offline build of a fresh compile node; the login's build is clean).
    library_errors_always: Option<String>,
    /// Verify reports the PROFINET connections down this many times per project.
    pn_down: usize,
    /// The runtimes refuse the first login until add_device_user ran.
    first_login: bool,
    /// set_pou_code of this object leaves the declaration as it was.
    lose_declaration: Option<String>,
    /// connect_to_device fails and echoes the password it was given.
    echo_password: bool,
    /// The robot simulators' PROFINET devices have no slots (a free child list, as the real
    /// KUKA PN device does).
    free_list: bool,
    /// The IDE has no gateway (CODESYS never opened a device editor).
    no_gateway: bool,
    /// The plain Codesys-MCP-Master 0.19.0: no add_device moduleId, no elementPath.
    plain_server: bool,
    /// The build directory and its manifest instead of the example's.
    tree: Option<(std::path::PathBuf, Manifest)>,
    /// unplug_device answers success but leaves the module in its slot.
    stuck_module: bool,
    /// The head's cell states `start` reads, in order: a pulse that fits the first one
    /// (CS_ERROR: bReset, CS_RECOVERY: bRecoveryConfirm) moves on, any other
    /// state moves on when read; the last one stays.
    cell_states: Vec<&'static str>,
    /// Pulses change nothing.
    stuck_cell: bool,
    /// The first this many fitting pulses change nothing (a robot whose drives come up only
    /// after its controller's stop messages were acknowledged).
    stuck_pulses: usize,
    /// connect_to_device fails this many times (a runtime that is down).
    login_down: usize,
    /// set_signature_access_bulk leaves this variable out (`Signature.variable`).
    lose_symbol: Option<String>,
    /// The overlay's auto-compile after each symbol-config edit fails (the edit lands) with
    /// library-open errors (live, 2026-11-18) and, with `auto_compile_error`, a real one.
    auto_compile_library_errors: bool,
    auto_compile_error: bool,
    /// The head runs another export: `GVL_Cell.PLAN_ID` reads this instead of the plan's.
    head_plan: Option<&'static str>,
    /// A CODESYS dialog mcp-bridge.cjs does not answer blocks the IDE: every call gets the
    /// bridge's answer for it.
    dialog: Option<&'static str>,
    /// rebind_device_to_scan_result times out this many times while the bridge answers the
    /// "No connection to the device" dialog (its note at the end of the failed answer, with a
    /// dialog open but not blocking).
    rebind_dialogs: usize,
    /// The overlay's library tools (export_project_libraries, install_libraries). Off by
    /// default: `projects` then copies the libraries into the build directory's
    /// `plc/libraries/`, and most tests run on the example itself.
    libraries: bool,
    /// install_libraries finds no libraries.json (an archive made before the export).
    no_libraries_json: bool,
    /// connect_to_device fails as CODESYS does when the login's build fails.
    login_build_error: bool,
    /// get_codesys_status shows the current install in this state (default `running`).
    codesys_state: Option<&'static str>,
}

/// One project as the mock IDE holds it.
#[derive(Default)]
struct Model {
    params: HashMap<(String, u32), String>,
    /// Struct members by (device, parameter, member path as in project.json).
    members: HashMap<(String, u32, String), String>,
    /// (path, kind, declaration, implementation)
    code: Vec<(String, &'static str, String, Option<String>)>,
    tasks: Vec<(String, Vec<String>)>,
    libs: Vec<String>,
    bus: HashMap<String, String>,
    users: bool,
    /// Children of the devices with slots, by device path.
    modules: HashMap<String, Vec<String>>,
    /// Symbol Configuration: None until create_symbol_config; the exported `Sig.var`.
    symbols: Option<Vec<String>>,
}

#[derive(Default)]
struct State {
    calls: Vec<String>,
    models: HashMap<String, Model>,
    reads: HashMap<String, usize>,
    /// The cell states still to come (Behaviour::cell_states).
    cell: Vec<&'static str>,
    /// write_variable: (expression, value).
    writes: Vec<(String, String)>,
    logins: usize,
    /// Fitting pulses so far (Behaviour::stuck_pulses).
    fitting: usize,
    /// Rebinds failed so far (Behaviour::rebind_dialogs).
    rebinds: usize,
    /// launch_codesys closed the stray CODESYS (Behaviour::stray).
    closed: bool,
    /// Questions about the stray CODESYS so far.
    asked: usize,
    /// export_project_libraries: its includeData argument, per call.
    include_data: Vec<bool>,
    /// Behaviour::library_errors_once answered.
    library_errors_given: bool,
}

/// The variables a head POU declares (`name : TYPE` lines), from the example's sources.
fn declared(sig: &str) -> Vec<String> {
    let dir = match sig {
        "GVL_Cell" | "GVL_IO" => "GVL",
        _ => "POU",
    };
    let Ok(text) = common::source(&format!("codesys/head/{dir}/{sig}.st")) else {
        return Vec::new();
    };
    text.lines()
        .filter_map(|l| {
            let (n, _) = l.split_once(':')?;
            let n = n.trim();
            (!n.is_empty() && n.chars().all(|c| c.is_ascii_alphanumeric() || c == '_'))
                .then(|| n.to_string())
        })
        .collect()
}

#[derive(Clone)]
struct MockCodesys {
    manifest: Arc<Manifest>,
    tools: Arc<Vec<Tool>>,
    schemas: Arc<Schemas>,
    state: Arc<Mutex<State>>,
    b: Behaviour,
}

/// The overlay's arguments, which the plain server does not have.
const OVERLAY_ARGS: &[(&str, &str)] = &[
    ("add_device", "moduleId"),
    ("get_device_parameter", "elementPath"),
    ("set_device_parameter", "elementPath"),
];

fn schemas(b: &Behaviour) -> Schemas {
    let mut f = fixture();
    if b.libraries {
        common::add_library_tools(&mut f);
    }
    if b.plain_server {
        for (tool, arg) in OVERLAY_ARGS {
            let props = f.get_mut(*tool).unwrap().1.get_mut("properties").unwrap();
            assert!(props.as_object_mut().unwrap().remove(*arg).is_some());
        }
    }
    f
}

fn fixture_tools(b: &Behaviour) -> Vec<Tool> {
    schemas(b)
        .into_iter()
        .map(|(name, (desc, schema))| Tool::new(name, desc, Arc::new(schema)))
        .collect()
}

/// The sub-elements of parameter `id` of `n` (`path`: of the struct member at that path), as
/// CODESYS has them: (label, the member path for a value, `None` for a struct). A member
/// `Identification.StationName` is a member of the struct `Identification`.
fn sub_elements(n: &DeviceNode, id: u32, path: Option<&str>) -> Vec<(String, Option<String>)> {
    let mut out: Vec<(String, Option<String>)> = Vec::new();
    for m in n
        .parameters
        .iter()
        .filter(|x| x.id == id)
        .filter_map(|x| x.member.as_deref())
    {
        let rest = match path {
            None => m,
            Some(p) => match m.strip_prefix(p).and_then(|r| r.strip_prefix('.')) {
                Some(r) => r,
                None => continue,
            },
        };
        match rest.split_once('.') {
            None => out.push((rest.to_string(), Some(m.to_string()))),
            Some((first, _)) if !out.iter().any(|(l, _)| l == first) => {
                out.push((first.to_string(), None))
            }
            Some(_) => {}
        }
    }
    out
}

fn name_of(d: &DeviceId) -> String {
    match d.ty {
        4102 => "CODESYS Control for Linux SL".into(),
        110 => "Ethernet".into(),
        80 => "PN-Controller".into(),
        t => format!("KUKA {t} {}", d.version),
    }
}

fn find_node<'a>(n: &'a DeviceNode, name: &str) -> Option<&'a DeviceNode> {
    if n.name == name {
        return Some(n);
    }
    n.children.iter().find_map(|c| find_node(c, name))
}

/// The DeviceID behind a device path: a node, or the device holding a slot of that name.
fn ident_of(p: &Project, path: &str) -> Option<DeviceId> {
    let last = path.rsplit('/').next()?;
    if let Some(n) = find_node(&p.device_tree, last) {
        return Some(n.device_id.clone());
    }
    let parent = path.rsplit('/').nth(1)?;
    let n = find_node(&p.device_tree, parent)?;
    n.slots
        .iter()
        .any(|s| s.name == last)
        .then(|| n.device_id.clone())
}

fn s<'a>(a: &'a Map<String, Value>, k: &str) -> &'a str {
    a.get(k).and_then(|v| v.as_str()).unwrap_or("")
}

/// The device paths of a tree with their nodes (root `Device`).
fn paths(n: &DeviceNode, path: String, out: &mut Vec<(String, DeviceNode)>) {
    out.push((path.clone(), n.clone()));
    for c in &n.children {
        paths(c, format!("{path}/{}", c.name), out);
    }
}

impl MockCodesys {
    /// A symbol-config edit that landed, with the overlay's auto-compile after it
    /// (`withAutoCompile`): clean, or failed as `auto_compile_library_errors` says.
    fn auto_compile(&self, file: &str, edit: &str) -> (String, bool) {
        if !self.b.auto_compile_library_errors {
            return (
                format!("{edit}\n\n[auto-compile] Compilation complete."),
                false,
            );
        }
        let mut errors = vec![
            "ERROR: Could not open library '#CmpUserMgr Implementation'. (Reason: The library \
             'CmpUserMgr Implementation, 3.5.21.0 (System)' has not been installed to the system.)",
            "ERROR: Could not open library '#CmpVisuHandler'. (Reason: The library \
             'CmpVisuHandler, 3.5.19.0 (System)' has not been installed to the system.)",
            "ERROR: Could not open library '#SysTask'. (Reason: The library 'SysTask, 3.5.17.0 \
             (System)' has not been installed to the system.)",
        ];
        if self.b.auto_compile_error {
            errors.push("ERROR: C0077: Unknown type: 'ST_Ticket' [PLC_PRG:12]");
        }
        (
            format!(
                "{edit}\n\n[auto-compile FAILED] Compilation complete for C:\\ProgramData\\plan2cell\\\
                 projects\\demo\\{file}.\n{} error(s), 0 warning(s).\n\nErrors:\n{}",
                errors.len(),
                errors.join("\n")
            ),
            true,
        )
    }

    /// The children of a device with slots, as CODESYS creates them: a PROFINET device under a
    /// controller (type 81) comes with its modules under their GSDML names, a PN device of the
    /// robot simulator (83) with empty slots, or with none (`free_list`).
    fn initial_modules(&self, n: &DeviceNode) -> Vec<String> {
        match n.device_id.ty {
            // An empty slot of DAP DIM 3 comes with its default module (FDIO64, UsedInSlots).
            81 => n
                .slots
                .iter()
                .map(|x| format!("_module_{}", x.module_id.as_deref().unwrap_or("1")))
                .collect(),
            _ if self.b.free_list => Vec::new(),
            _ => n.slots.iter().map(|_| "<Empty>".to_string()).collect(),
        }
    }

    fn structure(&self, p: &Project) -> String {
        let mut st = self.state.lock().unwrap();
        let mut all = Vec::new();
        paths(&p.device_tree, "Device".into(), &mut all);
        let mut out = String::from("- Unnamed (ExtendedObject[IScriptProject])\n");
        let Some(m) = st.models.get_mut(&p.file) else {
            return "Error retrieving structure.".into();
        };
        for (path, n) in &all {
            let depth = path.matches('/').count() + 1;
            out.push_str(&format!(
                "{}- {} (ExtendedObject[IScriptObject]) {{x}}\n",
                "  ".repeat(depth),
                n.name
            ));
            if path == "Device" {
                out.push_str("    - Plc Logic (ExtendedObject[IScriptObject]) {x}\n");
            }
            if !n.slots.is_empty() {
                let kids = m
                    .modules
                    .entry(path.clone())
                    .or_insert_with(|| self.initial_modules(n))
                    .clone();
                for k in kids {
                    out.push_str(&format!(
                        "{}- {k} (ExtendedObject[IScriptObject]) {{x}}\n",
                        "  ".repeat(depth + 1)
                    ));
                }
            }
        }
        out
    }

    fn project(&self, a: &Map<String, Value>) -> Option<&Project> {
        let f = s(a, "projectFilePath");
        let f = if f.is_empty() { s(a, "filePath") } else { f };
        let file = f.rsplit('\\').next()?;
        self.manifest.projects.iter().find(|p| p.file == file)
    }

    fn answer(&self, tool: &str, a: &Map<String, Value>) -> (String, bool) {
        let ok = |t: &str| (t.to_string(), false);
        let fail = |t: String| (format!("Operation failed. Output:\n{t}"), true);
        let errs = schema_errors(&self.schemas, tool, a, &[]);
        if !errs.is_empty() {
            return (format!("Invalid arguments: {errs:?}"), true);
        }
        if self.b.interactive {
            return (
                "Before starting another CODESYS (CODESYS V3.5 SP22 Patch 1): these CODESYS of the \
                 same install are running and cannot be taken over:\n  - PID 6632, opened by hand: \
                 (no project open); unsaved state unknown\nASK THE USER what to do with them"
                    .into(),
                true,
            );
        }
        if let Some(unsaved) = self.b.stray {
            let mut st = self.state.lock().unwrap();
            if tool == "launch_codesys" && s(a, "otherInstances") == "close" {
                st.closed = true;
                return ok(
                    "CODESYS SP22 Patch 1 launched successfully in persistent mode.\nOther CODESYS:\n  \
                     - PID 2540: closed",
                );
            }
            let left = self.b.stray_left;
            let taken = left.is_some_and(|n| n > 0 && st.asked >= n);
            if !st.closed && !taken {
                st.asked += 1;
                let kind = if left.is_some() {
                    "left by an ended MCP session"
                } else {
                    "opened by hand"
                };
                return (
                    format!(
                        "Before starting another CODESYS (CODESYS V3.5 SP22 Patch 1): these CODESYS of \
                         the same install are running and cannot be taken over:\n  - PID 2540, {kind}: \
                         CODESYS; {unsaved}\nASK THE USER what to do with them, then call \
                         launch_codesys with install 'CODESYS V3.5 SP22 Patch 1' and otherInstances set \
                         to the answer, and retry:"
                    ),
                    true,
                );
            }
        }
        if let Some(d) = self.b.dialog {
            return (
                format!(
                    "{} {d}. The MCP bridge answers only known dialogs and left this one open: \
                     answer it in the PLC IDE (Open PLC IDE, DCV), or POST /dismiss-dialogs \
                     {{\"hwnd\": 66052, \"button\": \"<button>\"}}, then run again",
                    plan2cell_cell::mcp::DIALOG
                ),
                true,
            );
        }
        if tool == "get_codesys_status" {
            return ok(&format!(
                "SP22 Patch 1  <- current\n  Exe: C:\\Program Files\\CODESYS 3.5.22.10\\CODESYS\\Common\\CODESYS.exe\n  State: {}\n  Mode: persistent",
                self.b.codesys_state.unwrap_or("running")
            ));
        }
        if tool == "shutdown_codesys" {
            return ok("CODESYS SP22 Patch 1 shut down (unsaved changes discarded).");
        }
        if tool == "launch_codesys" {
            return ok("CODESYS SP22 Patch 1 launched successfully in persistent mode.");
        }
        if tool == codesys::INSTALL_LIBRARIES {
            // install_libraries.py and server.ts libJsonResponse.
            let dir = s(a, "dir");
            if self.b.no_libraries_json {
                return fail(format!(
                    "DEBUG: install_libraries: Dir='{dir}'\nError installing the libraries of \
                     {dir}: NO_LIBRARIES_JSON: {dir}\\libraries.json not found.\nTraceback (most \
                     recent call last):\nRuntimeError: NO_LIBRARIES_JSON: {dir}\\libraries.json not \
                     found.\nSCRIPT_ERROR: Error installing the libraries of {dir}"
                ));
            }
            let json = serde_json::json!({
                "dir": dir, "repository": "System", "count": 2, "installed": 1,
                "already_installed": 1,
                "libraries": [
                    {"name": "Standard", "vendor": "System", "version": "3.5.17.0", "file": "Standard_3.5.17.0_System.compiled-library", "status": "already_installed"},
                    {"name": "IoStandard", "vendor": "System", "version": "3.5.17.0", "file": "IoStandard_3.5.17.0_System.compiled-library", "status": "installed", "call": "install_library(file, repo, False)"},
                ],
                "errors": [],
            });
            return ok(&format!(
                "Libraries of {dir}: 1 installed, 1 already installed, 0 error(s) (repository \
                 System).\n### INSTALL_LIBRARIES_START ###\n{json}\n### INSTALL_LIBRARIES_END ###"
            ));
        }
        if tool == "find_device_description" {
            let d = DeviceId {
                ty: a["deviceType"].as_u64().unwrap() as u32,
                id: s(a, "deviceId").into(),
                version: s(a, "deviceVersion").into(),
            };
            if self.b.missing_devices.contains(&d) {
                return ok("Installed: False");
            }
            return ok(&format!(
                "Installed: True\nDevice: {} | vendor V | type {} id {} version {}",
                name_of(&d),
                d.ty,
                d.id,
                d.version
            ));
        }
        let Some(p) = self.project(a) else {
            return fail(format!("no such project {a:?}"));
        };
        let mut guard = self.state.lock().unwrap();
        let st = &mut *guard;
        let key = p.file.clone();
        if tool == "compile_project"
            && ((self.b.library_errors_once.as_deref() == Some(p.name.as_str()) && !st.library_errors_given)
                || self.b.library_errors_always.as_deref() == Some(p.name.as_str()))
        {
            st.library_errors_given = true;
            return (
                "Compilation complete.\n2 error(s), 0 warning(s).\n\nErrors:\nERROR: Could not open library '#SysTask'. \
                 (Reason: The library 'SysTask, 3.5.17.0 (System)' has not been installed to the system.)\nERROR: Could not \
                 open library '#CmpVisuHandler'. (Reason: The library 'CmpVisuHandler, 3.5.19.0 (System)' has not been \
                 installed to the system.)"
                    .into(),
                true,
            );
        }
        if tool == "create_project" {
            assert_eq!(s(a, "deviceName"), name_of(&p.device_tree.device_id));
            st.models.insert(
                key.clone(),
                Model {
                    code: vec![(
                        "Device/Plc Logic/Application/PLC_PRG".to_string(),
                        "Program",
                        "PROGRAM PLC_PRG\nVAR\nEND_VAR\n".into(),
                        Some(String::new()),
                    )],
                    tasks: vec![("MainTask".into(), vec!["PLC_PRG".into()])],
                    ..Model::default()
                },
            );
            return ok("Project created");
        }
        let users = st.models.get(&key).map(|m| m.users).unwrap_or(false);
        let reads = st.reads.get(&key).copied().unwrap_or(0);
        // A project saved by an earlier run is on the IDE for the online tools.
        let online = [
            "rebind_device_to_scan_result",
            "connect_to_device",
            "add_device_user",
            "get_application_state",
            "read_variables",
            "write_variable",
            "start_stop_application",
            "list_configured_symbols",
            "disconnect_from_device",
            // start, before the login (libraries_ready).
            "download_missing_libraries",
            "list_project_libraries",
            "compile_project",
        ];
        if tool == "rebind_device_to_scan_result" && st.rebinds < self.b.rebind_dialogs {
            st.rebinds += 1;
            return (
                "Operation failed. Output:\nCommand rebind_device_to_scan_result timed out after \
                 180000ms\n[mcp-bridge] answered CODESYS dialog CODESYS: No connection to the \
                 device. Please rescan your network. [OK] with OK\n[mcp-bridge] open, not \
                 blocking: CODESYS: Scanning the network [Cancel]"
                    .into(),
                true,
            );
        }
        if online.contains(&tool) {
            st.models.entry(key.clone()).or_default();
            if self.b.no_gateway && tool != "disconnect_from_device" {
                return fail(
                    "SCRIPT_ERROR: RuntimeError: No gateway is configured in this CODESYS \
                     (Tools > Options > Gateway)."
                        .into(),
                );
            }
        }
        let Some(m) = st.models.get_mut(&key) else {
            return fail(format!("{key} does not exist"));
        };
        let dev = s(a, "devicePath").to_string();
        let obj = |n: &str| format!("Device/Plc Logic/Application/{n}");
        match tool {
            "get_device_identification" => match ident_of(p, &dev) {
                Some(d) => ok(&format!(
                    "Device: {}\ntype: {}\nid: {}\nversion: {}\ndevice_name: x",
                    dev.rsplit('/').next().unwrap(),
                    d.ty,
                    d.id,
                    d.version
                )),
                None => fail(format!("Device '{dev}' not found")),
            },
            "add_device" if a.contains_key("moduleId") => {
                // A module into the device's free child list, by the device's description.
                let parent = s(a, "parentPath").to_string();
                let n = find_node(&p.device_tree, parent.rsplit('/').next().unwrap()).unwrap();
                assert_eq!(s(a, "targetDeviceName"), name_of(&n.device_id));
                assert_eq!(s(a, "targetVersion"), n.device_id.version);
                let slot = n
                    .slots
                    .iter()
                    .find(|x| x.name == s(a, "deviceName"))
                    .unwrap();
                assert_eq!(Some(s(a, "moduleId")), slot.module_id.as_deref());
                m.modules.entry(parent).or_default().push(slot.name.clone());
                ok("Device added")
            }
            "add_device" => {
                let n = find_node(&p.device_tree, s(a, "deviceName")).unwrap();
                assert_eq!(s(a, "targetDeviceName"), name_of(&n.device_id));
                // CODESYS adds a PROFINET device's communication task with it.
                for t in &p.tasks {
                    let call = format!("{}.CommCycle", n.name);
                    if t.pous.contains(&call) && !m.tasks.iter().any(|(x, _)| *x == t.name) {
                        m.tasks.push((t.name.clone(), vec![call]));
                    }
                }
                ok("Device added")
            }
            "set_device_parameter" => {
                let id = a["parameterId"].as_u64().unwrap() as u32;
                let v = s(a, "value").to_string();
                let n = find_node(&p.device_tree, dev.rsplit('/').next().unwrap()).unwrap();
                let member = if let Some(path) = a.get("elementPath") {
                    let path = path.as_str().unwrap();
                    let known = n
                        .parameters
                        .iter()
                        .any(|x| x.id == id && x.member.as_deref() == Some(path));
                    if !known {
                        return fail(format!("No sub-element '{path}' below the parameter"));
                    }
                    Some(path.to_string())
                } else if let Some(i) = a.get("elementIndex") {
                    match sub_elements(n, id, None).get(i.as_u64().unwrap() as usize) {
                        Some((_, Some(m))) => Some(m.clone()),
                        Some((l, None)) => {
                            return fail(format!(
                                "Failed to write sub-element {l}: Operation not allowed on struct types"
                            ));
                        }
                        None => return fail(format!("elementIndex {i} out of range")),
                    }
                } else {
                    None
                };
                match member {
                    Some(mb) => m.members.insert((dev, id, mb), v),
                    None => m.params.insert((dev, id), v),
                };
                ok("Parameter set")
            }
            "get_device_parameter" => {
                let id = a["parameterId"].as_u64().unwrap() as u32;
                let n = find_node(&p.device_tree, dev.rsplit('/').next().unwrap()).unwrap();
                let path = a.get("elementPath").and_then(|v| v.as_str());
                let subs = sub_elements(n, id, path);
                if subs.is_empty() {
                    return fail(format!("No sub-element below {path:?}"));
                }
                let mut out = format!("P{id} = {}\nSubElements: {}\n", "{..}", subs.len());
                for (i, (label, member)) in subs.iter().enumerate() {
                    let v = match member {
                        Some(mb) => m
                            .members
                            .get(&(dev.clone(), id, mb.clone()))
                            .cloned()
                            .unwrap_or_default(),
                        None => "{..}".into(),
                    };
                    out.push_str(&format!("  [{i}] {label} = {v}\n"));
                }
                ok(&out)
            }
            "list_device_parameters" => {
                let mut out = String::from("Device parameters (scope\tid\tname\tvalue\tunit):\n");
                // A module: its ident in the Module parameters.
                if let Some((parent, name)) = dev.rsplit_once('/')
                    && m.modules
                        .get(parent)
                        .is_some_and(|k| k.iter().any(|x| x == name))
                {
                    let n = find_node(&p.device_tree, parent.rsplit('/').next().unwrap()).unwrap();
                    let slot = n.slots.iter().find(|x| x.name == name).unwrap();
                    if n.device_id.ty == 83 {
                        // The simulator's own PN device: the module is its first parameter.
                        out.push_str(&format!(
                            "connector[0]:PN-Device.0x019B_0x0301:host\t69632\t64 safe digital in- and outputs\t{{1, 1, {}, 16#00000001, 12, 12, 0}}\t\n",
                            slot.ident
                        ));
                    } else {
                        out.push_str(&format!(
                            "connector[0]:PNIO2-Module.x_{}:host\t513\tModule parameters\t{{{}, 1, 1}}\t\n",
                            slot.module_id.as_deref().unwrap(),
                            slot.ident
                        ));
                    }
                }
                for ((d, id), v) in &m.params {
                    if *d == dev {
                        // CODESYS answers arrays with blanks.
                        out.push_str(&format!(
                            "device\t{id}\tP{id}\t{}\t\n",
                            v.replace(',', ", ")
                        ));
                    }
                }
                ok(&out)
            }
            "plug_device" => {
                let parent = s(a, "parentPath").to_string();
                let slot = a["slotIndex"].as_u64().unwrap() as usize;
                let Some(kids) = m.modules.get_mut(&parent) else {
                    return fail(format!(
                        "slotIndex {slot} out of range 1..0 under '{parent}'."
                    ));
                };
                match kids.get_mut(slot - 1) {
                    Some(k) if k == "<Empty>" => {
                        *k = s(a, "name").into();
                        ok("Plugged")
                    }
                    Some(_) => fail("Devices can only be plugged into slots".into()),
                    None => fail(format!("slotIndex {slot} out of range")),
                }
            }
            "unplug_device" => {
                let Some((parent, name)) = dev.rsplit_once('/') else {
                    return fail(format!("Device '{dev}' not found"));
                };
                let Some(k) = m
                    .modules
                    .get_mut(parent)
                    .and_then(|kids| kids.iter_mut().find(|k| *k == name))
                else {
                    return fail(format!("Device '{dev}' not found"));
                };
                if !self.b.stuck_module {
                    *k = "<Empty>".into();
                }
                ok("Unplugged. Project saved.")
            }
            "rename_object" => {
                let path = s(a, "objectPath");
                let (parent, old) = path.rsplit_once('/').unwrap();
                let Some(k) = m
                    .modules
                    .get_mut(parent)
                    .and_then(|kids| kids.iter_mut().find(|k| *k == old))
                else {
                    return fail(format!("Object '{path}' not found"));
                };
                *k = s(a, "newName").into();
                ok("Renamed. Project saved.")
            }
            "download_missing_libraries" => ok("Missing libraries download requested: None."),
            "export_project_libraries" => {
                // export_project_libraries.py and server.ts libJsonResponse: this project's
                // libraries; with includeData the files, but one is over the answer's budget.
                let data = a.get("includeData").and_then(Value::as_bool) == Some(true);
                st.include_data.push(data);
                let out = s(a, "outDir");
                let mut libs = vec![serde_json::json!({
                    "name": "Standard", "vendor": "System", "version": "3.5.17.0",
                    "file": "Standard_3.5.17.0_System.compiled-library", "sha256": "aa",
                    "source": "C:\\repo\\Standard.compiled-library",
                })];
                for l in &m.libs {
                    libs.push(serde_json::json!({
                        "name": l, "vendor": "System", "version": "3.5.17.0",
                        "file": format!("{l}_3.5.17.0_System.compiled-library"), "sha256": "bb",
                        "source": "C:\\repo\\x",
                    }));
                }
                if p.role == plan2cell_cell::project::Role::HeadPlc {
                    libs.push(serde_json::json!({
                        "name": "BigLib", "vendor": "Acme", "version": "1.0.0.0",
                        "file": "BigLib_1.0.0.0_Acme.compiled-library", "sha256": "cc",
                        "source": "C:\\repo\\BigLib.compiled-library",
                    }));
                }
                if data {
                    for l in &mut libs {
                        if l["name"] == "BigLib" {
                            l["dataOmitted"] = "20000000 bytes, over the answer's budget of 16777216 bytes".into();
                        } else {
                            l["data"] = plan2cell_cell::remote::base64(
                                format!("library {}", l["name"].as_str().unwrap()).as_bytes(),
                            )
                            .into();
                        }
                    }
                }
                let json = serde_json::json!({
                    "project": s(a, "projectFilePath"), "outDir": out,
                    "manifest": format!("{out}\\libraries.json"), "manifestCount": libs.len(),
                    "removed": [], "count": libs.len(), "libraries": libs, "errors": [],
                });
                ok(&format!(
                    "Exported {} library file(s) of {} to {out} (libraries.json), 0 error(s).\n\
                     ### EXPORT_LIBRARIES_START ###\n{json}\n### EXPORT_LIBRARIES_END ###",
                    libs.len(),
                    s(a, "projectFilePath")
                ))
            }
            "add_library" => {
                m.libs.push(s(a, "libraryName").into());
                ok("Library added")
            }
            "list_project_libraries" => {
                let mut out =
                    String::from("Application (libman: Library Manager) - n reference(s)\n");
                for l in m.libs.iter().cloned().chain(
                    p.libraries
                        .iter()
                        .filter(|l| l.mode == "auto")
                        .map(|l| l.name.clone()),
                ) {
                    out.push_str(&format!("  - {l}, * (System) [placeholder]\n"));
                }
                ok(&out)
            }
            "create_dut" => {
                m.code.push((obj(s(a, "name")), "DUT", String::new(), None));
                ok("DUT created")
            }
            "create_gvl" => {
                m.code.push((
                    obj(s(a, "name")),
                    "GVL",
                    s(a, "declarationCode").into(),
                    None,
                ));
                ok("GVL created")
            }
            "create_pou" => {
                m.code.push((
                    obj(s(a, "name")),
                    "POU",
                    s(a, "declarationCode").into(),
                    Some(s(a, "implementationCode").into()),
                ));
                ok("POU created")
            }
            "set_pou_code" => {
                let name = s(a, "pouPath").rsplit('/').next().unwrap().to_string();
                let lose = self.b.lose_declaration.as_deref() == Some(name.as_str());
                let Some(c) = m.code.iter_mut().find(|c| c.0 == obj(&name)) else {
                    return fail(format!("no object {name}"));
                };
                if let Some(d) = a.get("declarationCode")
                    && !lose
                {
                    c.2 = d.as_str().unwrap().into();
                }
                if let Some(i) = a.get("implementationCode") {
                    c.3 = Some(i.as_str().unwrap().into());
                }
                ok("Code set")
            }
            "get_all_pou_code" => {
                let mut out = format!("{} object(s) with code:\n", m.code.len());
                for (path, kind, d, i) in &m.code {
                    out.push_str(&format!(
                        "\n=== {path} ({kind}) ===\n// ----- Declaration -----\n{}",
                        d.replace('\n', "\r\n")
                    ));
                    if let Some(i) = i {
                        out.push_str(&format!("\n// ----- Implementation -----\n{i}"));
                    }
                }
                ok(&out)
            }
            "delete_object" => {
                let path = obj(s(a, "objectPath").rsplit('/').next().unwrap());
                m.code.retain(|c| c.0 != path);
                ok("Deleted")
            }
            "list_tasks" => {
                let mut out = String::from("Task Configuration: Task Configuration\n");
                for (t, pous) in &m.tasks {
                    out.push_str(&format!(
                        "\nTask: {t}\n  Props: priority=1\n  POU calls ({}): {}\n",
                        pous.len(),
                        if pous.is_empty() {
                            "(none)".into()
                        } else {
                            pous.join(", ")
                        }
                    ));
                }
                ok(&out)
            }
            "create_task" => {
                let t = s(a, "taskName").to_string();
                if m.tasks.iter().any(|(x, _)| *x == t) {
                    return fail(format!("task {t} exists"));
                }
                m.tasks.push((t, Vec::new()));
                ok("Task created")
            }
            "configure_task" => {
                if !m.tasks.iter().any(|(x, _)| x == s(a, "taskName")) {
                    return fail("no such task".into());
                }
                ok("Task configured")
            }
            "add_pou_to_task" => {
                let Some(t) = m.tasks.iter_mut().find(|(x, _)| x == s(a, "taskName")) else {
                    return fail("no such task".into());
                };
                t.1.push(s(a, "pouName").into());
                ok("Added")
            }
            "set_plc_settings" => {
                m.bus.insert(dev, s(a, "busCycleTask").into());
                ok("PLC settings set")
            }
            "get_plc_settings" => ok(&format!(
                "Bus cycle task: {}\nUpdate IOs while in stop: False",
                m.bus
                    .get(&dev)
                    .map(String::as_str)
                    .unwrap_or("<unspecified>")
            )),
            "mirror_export" => ok("Mirror exported"),
            "import_io_mappings_csv" => {
                if s(a, "csvPath").ends_with(&format!("{IO_CARRIER}.st"))
                    && !m.code.iter().any(|c| c.0.ends_with(IO_CARRIER))
                {
                    return fail("CSV file does not exist".into());
                }
                ok("Imported")
            }
            "compile_project" => {
                if m.code.iter().any(|c| c.0.ends_with(IO_CARRIER)) {
                    return (
                        "Compilation complete.\n1 error(s), 0 warning(s).\n\nErrors:\nERROR: the CSV carrier is still there".into(),
                        true,
                    );
                }
                if self.b.fail_build.as_deref() == Some(p.name.as_str()) {
                    (
                        format!(
                            "Compilation complete for {}.\n1 error(s), 0 warning(s).\n\nErrors:\nERROR: C0077: Unknown type: 'ST_Ticket'",
                            p.file
                        ),
                        true,
                    )
                } else {
                    ok(
                        "Compilation complete.\n0 error(s), 1 warning(s).\n\nWarnings:\nWARNING: 3 unused variables",
                    )
                }
            }
            "save_project"
            | "rebind_device_to_scan_result"
            | "download_to_device"
            | "create_boot_application"
            | "start_stop_application"
            | "disconnect_from_device" => ok("Done"),
            "open_project" => ok(&format!("Project opened: {}", s(a, "filePath"))),
            "close_project" => ok(&format!("Project closed: {}", s(a, "projectFilePath"))),
            "connect_to_device" if st.logins < self.b.login_down => {
                st.logins += 1;
                fail("Unable to connect to the device: no answer from 192.168.100.10".into())
            }
            "connect_to_device" if self.b.login_build_error => fail(
                "DEBUG: login(OnlineChangeOption.Try, True)\nLogin to the PLC failed: Exception: \
                 Compile errors occurred. Fix them before logging in. (login(OnlineChangeOption.Try, \
                 True))"
                    .into(),
            ),
            "connect_to_device" => {
                if self.b.echo_password {
                    fail(format!(
                        "login failed for password {}",
                        s(a, "devicePassword")
                    ))
                } else if self.b.first_login && !users {
                    fail("Invalid user authentication on the target".into())
                } else {
                    ok("Connected")
                }
            }
            "add_device_user" => {
                m.users = true;
                ok("User added")
            }
            "get_application_state" => ok("Application: Application\nState: run\nLogged In: True"),
            "read_variables"
                if a["expressions"]
                    .as_array()
                    .is_some_and(|e| e.iter().any(|x| x == "PLC_PRG.eCellState")) =>
            {
                if st.cell.is_empty() {
                    st.cell = self.b.cell_states.clone();
                }
                let cur = st.cell.first().copied().unwrap_or("CS_RUNNING");
                let err = match cur {
                    "CS_ERROR" => "E_CellError.ERR_START_TIMEOUT",
                    _ => "E_CellError.ERR_NONE",
                };
                let out = format!(
                    "PLC_PRG.eCellState = E_CellState.{cur}\nPLC_PRG.fbCell.eError = {err}\n\
                     PLC_PRG.fbCell.nErrorRobot = {}\nPLC_PRG.bRecoveryRequired = {}\n\
                     PLC_PRG.udiCycles = 3\n",
                    if cur == "CS_ERROR" { 2 } else { 0 },
                    if cur == "CS_RECOVERY" {
                        "TRUE"
                    } else {
                        "FALSE"
                    },
                );
                if !["CS_ERROR", "CS_RECOVERY"].contains(&cur) && st.cell.len() > 1 {
                    st.cell.remove(0);
                }
                ok(&out)
            }
            "write_variable" => {
                let (x, v) = (s(a, "variablePath").to_string(), s(a, "value").to_string());
                assert_eq!(v, "TRUE", "a pulse");
                let cur = st.cell.first().copied().unwrap_or("CS_RUNNING");
                let fits = matches!(
                    (cur, x.as_str()),
                    ("CS_ERROR", "PLC_PRG.bReset") | ("CS_RECOVERY", "PLC_PRG.bRecoveryConfirm")
                );
                if x != "PLC_PRG.bStop" {
                    assert!(fits, "{x} pulsed in {cur}");
                }
                if fits {
                    st.fitting += 1;
                }
                if fits
                    && !self.b.stuck_cell
                    && st.fitting > self.b.stuck_pulses
                    && st.cell.len() > 1
                {
                    st.cell.remove(0);
                }
                st.writes.push((x, v));
                ok("Variable written")
            }
            "create_symbol_config" => {
                assert_eq!(a["supportOpcUa"], true);
                if m.symbols.is_none() {
                    m.symbols = Some(Vec::new());
                }
                self.auto_compile(&p.file, "Created Symbol Configuration: Device/Plc Logic/Application/Symbols")
            }
            "set_symbol_config_settings" => {
                assert_eq!(
                    a["contentFeatureFlags"],
                    serde_json::json!(["SupportOPCUA"])
                );
                self.auto_compile(&p.file, "Symbol Configuration settings updated")
            }
            "set_signature_access_bulk" | "set_symbol_access" => {
                let Some(syms) = m.symbols.as_mut() else {
                    return fail("no Symbol Configuration".into());
                };
                let sig = s(a, "signatureFqn").to_string();
                let vars = declared(&sig);
                if vars.is_empty() {
                    return fail(format!("Signature '{sig}' not found"));
                }
                let pick: Vec<String> = if tool == "set_symbol_access" {
                    let v = s(a, "variableName").to_string();
                    if sig == "GVL_Cell" {
                        // A constant the compiler replaces: not in sig.variables.
                        return fail(format!(
                            "Error in set_symbol_access: Variable '{v}' not found in signature '{sig}'"
                        ));
                    }
                    vec![v]
                } else {
                    vars
                };
                for v in pick {
                    let k = format!("{sig}.{v}");
                    if self.b.lose_symbol.as_deref() != Some(k.as_str()) && !syms.contains(&k) {
                        syms.push(k);
                    }
                }
                self.auto_compile(&p.file, &format!("Signature {sig} bulk access set to {}.", s(a, "access")))
            }
            "list_configured_symbols" => {
                let Some(syms) = m.symbols.as_ref() else {
                    return fail("no Symbol Configuration".into());
                };
                let mut sigs: Vec<(String, Vec<Value>)> = Vec::new();
                for k in syms {
                    let (sig, v) = k.split_once('.').unwrap();
                    let var = serde_json::json!({"name": v, "configured_access": "Read",
                        "maximal_access": "ReadWrite", "effective_access": "Read"});
                    match sigs.iter_mut().find(|(n, _)| n == sig) {
                        Some((_, vs)) => vs.push(var),
                        None => sigs.push((sig.to_string(), vec![var])),
                    }
                }
                let j = serde_json::json!({
                    "configured_signatures": sigs.iter().map(|(n, vs)| serde_json::json!(
                        {"fqn": n, "name": n, "library_id": null, "variables": vs})).collect::<Vec<_>>(),
                    "configured_datatypes": [],
                });
                ok(&format!(
                    "DEBUG: list_configured_symbols\n### SYMBOL_CONFIGURED_START ###\n{j}\n### SYMBOL_CONFIGURED_END ###\nSCRIPT_SUCCESS: list_configured_symbols completed."
                ))
            }
            "read_variables" => {
                let mut out = String::new();
                for c in codesys::verify_checks(&self.manifest, p) {
                    let v = if c.expr.ends_with(".xRunning") && reads < self.b.pn_down {
                        "FALSE".to_string()
                    } else if c.expr == "GVL_Cell.PLAN_ID" && self.b.head_plan.is_some() {
                        format!("'{}'", self.b.head_plan.unwrap())
                    } else if c.expr.ends_with(".PLAN_ID") {
                        // A STRING reads back quoted.
                        format!("'{}'", c.expect)
                    } else if c.expect.starts_with("16#") {
                        format!("WORD#{}", c.expect)
                    } else {
                        c.expect.clone()
                    };
                    out.push_str(&format!("{} = {v}\n", c.expr));
                }
                *st.reads.entry(key).or_default() += 1;
                ok(&out)
            }
            t => panic!("the mock does not know {t}"),
        }
    }
}

impl ServerHandler for MockCodesys {
    fn get_info(&self) -> ServerConfig {
        ServerConfig::new(
            ServerCapabilities::builder()
                .enable_tools()
                .enable_resources()
                .build(),
        )
    }

    async fn read_resource(
        &self,
        r: ReadResourceRequestParams,
        _c: RequestContext<RoleServer>,
    ) -> Result<ReadResourceResponse, ErrorData> {
        let file = r
            .uri
            .strip_suffix("/structure")
            .and_then(|u| u.rsplit('\\').next())
            .unwrap_or("");
        let p = self.manifest.projects.iter().find(|p| p.file == file);
        let text = match p {
            Some(p) => self.structure(p),
            None => format!("Error retrieving structure.\n\nno project {file}"),
        };
        self.state.lock().unwrap().calls.push(match p {
            Some(p) => format!("resources/read {}", p.name),
            None => "resources/read".into(),
        });
        Ok(ReadResourceResult::new(vec![ResourceContents::text(text, r.uri)]).into())
    }

    async fn list_tools(
        &self,
        _r: Option<PaginatedRequestParams>,
        _c: RequestContext<RoleServer>,
    ) -> Result<ListToolsResult, ErrorData> {
        self.state.lock().unwrap().calls.push("tools/list".into());
        Ok(ListToolsResult::with_all_items((*self.tools).clone()))
    }

    async fn call_tool(
        &self,
        r: CallToolRequestParams,
        _c: RequestContext<RoleServer>,
    ) -> Result<CallToolResponse, ErrorData> {
        let args = r.arguments.clone().unwrap_or_default();
        let (text, is_error) = self.answer(&r.name, &args);
        let who = self.project(&args).map(|p| p.name.clone());
        let mut st = self.state.lock().unwrap();
        st.calls.push(match who {
            Some(w) => format!("{} {w}", r.name),
            None => r.name.to_string(),
        });
        let mut res = CallToolResult::success(vec![ContentBlock::text(text)]);
        if is_error {
            res.is_error = Some(true);
        }
        Ok(res.into())
    }
}

struct Server {
    _rt: tokio::runtime::Runtime,
    url: String,
    state: Arc<Mutex<State>>,
}

impl Server {
    fn start(b: Behaviour) -> Self {
        Self::start_on(b, 0)
    }

    /// On this port of 127.0.0.1 (0: any).
    fn start_on(b: Behaviour, port: u16) -> Self {
        use rmcp::transport::streamable_http_server::{
            StreamableHttpServerConfig, StreamableHttpService, session::local::LocalSessionManager,
        };
        let rt = tokio::runtime::Runtime::new().unwrap();
        let state = Arc::new(Mutex::new(State::default()));
        let m = b
            .tree
            .as_ref()
            .map(|t| t.1.clone())
            .unwrap_or_else(manifest);
        let mock = MockCodesys {
            manifest: Arc::new(m),
            tools: Arc::new(fixture_tools(&b)),
            schemas: Arc::new(schemas(&b)),
            state: state.clone(),
            b,
        };
        let addr = rt.block_on(async move {
            let config = StreamableHttpServerConfig::default()
                .with_allowed_hosts(vec!["127.0.0.1".to_string(), "localhost".to_string()]);
            let service: StreamableHttpService<MockCodesys, LocalSessionManager> =
                StreamableHttpService::new(move || Ok(mock.clone()), Default::default(), config);
            let app =
                axum::Router::new()
                    .nest_service("/mcp", service)
                    .layer(axum::middleware::from_fn(
                        |req: axum::extract::Request, next: axum::middleware::Next| async move {
                            let ok = req
                                .headers()
                                .get("authorization")
                                .and_then(|v| v.to_str().ok())
                                == Some(&format!("Bearer {TOKEN}"));
                            if ok {
                                next.run(req).await
                            } else {
                                axum::response::IntoResponse::into_response((
                                    axum::http::StatusCode::UNAUTHORIZED,
                                    "token",
                                ))
                            }
                        },
                    ));
            let l = tokio::net::TcpListener::bind(("127.0.0.1", port))
                .await
                .unwrap();
            let addr = l.local_addr().unwrap();
            tokio::spawn(async move {
                let _ = axum::serve(l, app).await;
            });
            addr
        });
        Server {
            _rt: rt,
            url: format!("http://{addr}/mcp"),
            state,
        }
    }

    fn calls(&self) -> Vec<String> {
        self.state.lock().unwrap().calls.clone()
    }
}

const TOKEN_ENV: &str = "PLAN2CELL_CELL_TEST_TOKEN";
const USER_ENV: &str = "PLAN2CELL_CELL_TEST_DEVICE_USER";
const PASSWORD_ENV: &str = "PLAN2CELL_CELL_TEST_DEVICE_PASSWORD";

fn config(url: &str) -> CellConfig {
    // SAFETY: every test sets the same values; nothing reads the environment concurrently in a
    // way that could observe a torn write.
    unsafe {
        std::env::set_var(TOKEN_ENV, TOKEN);
        std::env::set_var(USER_ENV, "plan2cell");
        std::env::set_var(PASSWORD_ENV, "test-password-0123");
    }
    CellConfig {
        codesys_mcp: CodesysMcp {
            url: url.into(),
            token_env: Some(TOKEN_ENV.into()),
            device_user_env: USER_ENV.into(),
            device_password_env: PASSWORD_ENV.into(),
            timeout_s: 30,
            ..CodesysMcp::default()
        },
        plc_host: PlcHost {
            bucket: Some("plan2cell-software-123456789012-eu-central-1".into()),
            region: Some("eu-central-1".into()),
            ..PlcHost::default()
        },
        kuka_vc: None,
        address_map: Default::default(),
        profinet: None,
    }
}

fn plc_host() -> MockRemote {
    MockRemote::default()
        .answer("if test -x", Ok("ok\n".into()))
        .answer(
            "sudo docker ps",
            Ok("codesys-master\ncodesys-device\ncodesys-device-r2\n".into()),
        )
}

fn quick() -> Settings {
    Settings {
        dry_run: false,
        verify_timeout: Duration::ZERO,
        verify_interval: Duration::from_millis(10),
        start_timeout: Duration::from_secs(20),
        start_interval: Duration::from_millis(10),
        runtime_wait: Duration::from_secs(4),
        others_timeout: Duration::from_millis(300),
        others_interval: Duration::from_millis(10),
        ..Settings::default()
    }
}

const R1: &str = "RobotSim_welding_cell_2_small_R1";
const R2: &str = "RobotSim_welding_cell_2_small_R2";
const HEAD: &str = "HeadPLC_welding_cell_2_small";

fn run(
    b: Behaviour,
    stages: &[Stage],
    tree: bool,
    settings: Settings,
) -> (
    Server,
    MockRemote,
    plan2cell_cell::Result<()>,
    plan2cell_cell::Report,
) {
    run_cfg(b, stages, tree, settings, config)
}

/// [`run`] with this cell configuration.
fn run_cfg(
    b: Behaviour,
    stages: &[Stage],
    tree: bool,
    settings: Settings,
    cfg: impl Fn(&str) -> CellConfig,
) -> (
    Server,
    MockRemote,
    plan2cell_cell::Result<()>,
    plan2cell_cell::Report,
) {
    let dir = b.tree.as_ref().map(|t| t.0.clone()).unwrap_or_else(example);
    let s = Server::start(b);
    let mut remote = plc_host();
    let mut tools = McpTools::new(cfg(&s.url).codesys_mcp).unwrap();
    let mut c = Cell::new(
        cfg(&s.url),
        tree.then_some(Tree::Dir(dir)),
        settings,
        &mut remote,
        &mut tools,
        None,
    )
    .unwrap();
    let r = c.run(stages);
    let report = c.report.clone();
    drop(c);
    (s, remote, r, report)
}

#[test]
fn up_runs_every_stage_in_order() {
    let (s, remote, r, report) = run(Behaviour::default(), &Stage::ALL, true, quick());
    r.unwrap();
    let calls = s.calls();
    assert_eq!(calls[0], "tools/list");
    assert_eq!(calls[1], "get_codesys_status");
    // The projects, then deploy, then verify, each robot simulators first.
    let order: Vec<&str> = calls
        .iter()
        .filter(|c| {
            c.starts_with("create_project ")
                || c.starts_with("download_to_device ")
                || c.starts_with("read_variables ")
        })
        .map(String::as_str)
        .collect();
    let mut want = Vec::new();
    for t in ["create_project", "download_to_device", "read_variables"] {
        for p in [R1, R2, HEAD] {
            want.push(format!("{t} {p}"));
        }
    }
    assert_eq!(order, want);
    // deploy leaves the head project open in the PLC IDE (once, logged out, before verify).
    let opened: Vec<usize> = (0..calls.len())
        .filter(|&i| calls[i].starts_with("open_project"))
        .collect();
    assert_eq!(opened.len(), 1, "{calls:?}");
    assert_eq!(calls[opened[0]], format!("open_project {HEAD}"));
    assert_eq!(
        calls[opened[0] - 1],
        format!("disconnect_from_device {HEAD}")
    );
    let first_verify = calls
        .iter()
        .position(|c| *c == format!("read_variables {R1}"))
        .unwrap();
    assert!(opened[0] < first_verify);
    // Edits made in the PLC IDE never reach a PLC: the head is closed without saving after the
    // projects stage's logout (before create_project), first in deploy (before any bind).
    let closed: Vec<usize> = (0..calls.len())
        .filter(|&i| calls[i].starts_with("close_project"))
        .collect();
    assert_eq!(closed.len(), 2, "{calls:?}");
    assert!(
        closed
            .iter()
            .all(|&i| calls[i] == format!("close_project {HEAD}")),
        "{calls:?}"
    );
    let first = |what: &str| calls.iter().position(|c| c.starts_with(what)).unwrap();
    assert!(first("disconnect_from_device") < closed[0] && closed[0] < first("create_project"));
    let first_bind = first("rebind_device_to_scan_result");
    assert!(
        closed[1] < first_bind && calls[closed[1] + 1].starts_with("rebind_device_to_scan_result"),
        "{calls:?}"
    );
    let projects = report
        .stages
        .iter()
        .find(|st| st.stage == Stage::Projects)
        .unwrap();
    assert!(
        projects.notes.iter().any(|n| n
            == &format!(
                "{HEAD} closed in the PLC IDE without saving first (edits made there are discarded)"
            )),
        "{:?}",
        projects.notes
    );
    let deploy = report
        .stages
        .iter()
        .find(|st| st.stage == Stage::Deploy)
        .unwrap();
    assert!(
        deploy
            .notes
            .iter()
            .any(|n| n.starts_with(&format!("{HEAD} open in the PLC IDE (C:\\"))),
        "{:?}",
        deploy.notes
    );
    // Every call reached the mock with valid arguments (else it answers isError).
    assert!(
        report.calls.iter().all(|c| !c.contains("test-password")),
        "no secret in the report"
    );

    let overlay = example().join("plc/docker/docker-compose.robots.yml");
    assert_eq!(
        remote.calls,
        [
            Call::Run(
                "if test -x /opt/plan2cell/plc-host/setup.sh && sudo -n true; then echo ok; fi"
                    .into()
            ),
            Call::Copy(
                overlay.display().to_string(),
                "/tmp/plan2cell-docker-compose.robots.yml".into()
            ),
            Call::Run(
                "sudo install -m 0644 /tmp/plan2cell-docker-compose.robots.yml \
                 /opt/plan2cell/plc-host/docker-compose.robots.yml"
                    .into()
            ),
            Call::Run(
                "sudo /opt/plan2cell/plc-host/setup.sh --bucket \
                 plan2cell-software-123456789012-eu-central-1 --region eu-central-1 \
                 --robots-overlay /opt/plan2cell/plc-host/docker-compose.robots.yml"
                    .into()
            ),
            Call::Run("sudo docker ps --format '{{.Names}}'".into()),
        ]
    );

    assert!(report.ok, "{:#?}", report.projects);
    assert_eq!(report.plan_id.as_deref(), Some("16#F77E"));
    assert_eq!(report.stages.len(), 5);
    assert!(
        report.stages[0]
            .notes
            .iter()
            .any(|n| n.contains("CODESYS 3.5.22.10"))
    );
    let names: Vec<&str> = report.projects.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, [R1, R2, HEAD]);
    for p in &report.projects {
        assert!(
            p.built && p.errors.is_empty() && p.deviations.is_empty(),
            "{p:?}"
        );
        assert_eq!(p.deployed, Some(true));
        assert_eq!(p.verified, Some(true));
        assert!(p.checks.iter().all(|c| c.ok));
        assert_eq!(p.warnings, ["3 unused variables"]);
    }
    assert_eq!(
        report.projects[2].file,
        r"C:\ProgramData\plan2cell\projects\welding_cell_2_small\HeadPLC_welding_cell_2_small.project"
    );
    // The MCP_PROMPT.md §4 shape.
    let v = serde_json::to_value(&report).unwrap();
    for k in [
        "name",
        "file",
        "built",
        "errors",
        "warnings",
        "deviations",
        "human_steps",
    ] {
        assert!(v["projects"][0].get(k).is_some(), "{k}");
    }
}

#[test]
fn tasks_that_exist_are_not_created_again() {
    let (s, _, r, _) = run(Behaviour::default(), &[Stage::Projects], true, quick());
    r.unwrap();
    let calls = s.calls();
    let n = |c: &str| calls.iter().filter(|x| *x == c).count();
    // MainTask (create_project) and Profinet_CommunicationTask (comes with the device) exist.
    assert_eq!(n(&format!("create_task {HEAD}")), 1);
    assert_eq!(n(&format!("add_pou_to_task {HEAD}")), 0);
    assert_eq!(n(&format!("configure_task {HEAD}")), 3);
}

#[test]
fn a_build_error_is_exit_8_and_reported() {
    let (s, remote, r, rep) = run(
        Behaviour {
            fail_build: Some(HEAD.into()),
            ..Behaviour::default()
        },
        &[Stage::Projects, Stage::Deploy],
        true,
        quick(),
    );
    let e = r.unwrap_err();
    assert_eq!(e.exit_code(), 8, "{e}");
    assert!(!rep.ok);
    assert_eq!(rep.stages.len(), 1, "deploy never ran");
    assert!(rep.projects[0].built && rep.projects[1].built);
    assert!(!rep.projects[2].built);
    assert_eq!(rep.projects[2].errors, ["C0077: Unknown type: 'ST_Ticket'"]);
    let saved: Vec<String> = s
        .calls()
        .into_iter()
        .filter(|c| c.starts_with("save_project"))
        .collect();
    assert_eq!(saved.len(), 3, "a build with errors is still saved");
    assert!(remote.calls.is_empty());
}

#[test]
fn modules_are_plugged_into_empty_slots_or_renamed() {
    let (s, _, r, rep) = run(Behaviour::default(), &[Stage::Projects], true, quick());
    r.unwrap();
    let calls = s.calls();
    let n = |c: &str| calls.iter().filter(|x| *x == c).count();
    // The robot simulators' empty slots are plugged, the head's modules (which come with the
    // PROFINET devices) renamed.
    assert_eq!(n(&format!("plug_device {R1}")), 2);
    assert_eq!(n(&format!("rename_object {R1}")), 0);
    assert_eq!(n(&format!("plug_device {HEAD}")), 0);
    assert_eq!(n(&format!("rename_object {HEAD}")), 4);
    assert!(
        rep.stages[0]
            .notes
            .iter()
            .any(|x| x.contains("slot 1 (_module_1) renamed to SafeIO_FDIO64_R1")),
        "{:?}",
        rep.stages[0].notes
    );
}

/// bt2plc `safety_module = false` for robot2: the head's kuka_robi2 is DAP DIM 3, which CODESYS
/// adds with its default FDIO64 in slot 1; it is unplugged and slot 1 read back empty.
#[test]
fn without_the_safety_module_slot_1_is_unplugged_and_read_back_empty() {
    let dir = example_without_safety_module("kuka_robi2", "R2");
    let m = Manifest::parse(&std::fs::read_to_string(dir.path().join("plc/project.json")).unwrap())
        .unwrap();
    let (s, _, r, rep) = run(
        Behaviour {
            tree: Some((dir.path().to_path_buf(), m)),
            ..Behaviour::default()
        },
        &[Stage::Projects],
        true,
        quick(),
    );
    r.unwrap();
    let calls = s.calls();
    let n = |c: &str| calls.iter().filter(|x| *x == c).count();
    assert_eq!(n(&format!("unplug_device {HEAD}")), 1);
    // kuka_robi1: two renames; kuka_robi2: only DIO256.
    assert_eq!(n(&format!("rename_object {HEAD}")), 3);
    let i = calls
        .iter()
        .position(|c| *c == format!("unplug_device {HEAD}"))
        .unwrap();
    assert_eq!(calls[i + 1], format!("resources/read {HEAD}"), "read back");
    assert!(
        rep.calls.iter().any(|c| c.contains(
            r#"unplug_device(devicePath="Device/Ethernet/PN_Controller/kuka_robi2/_module_1""#
        )),
        "{:#?}",
        rep.calls
    );
    assert!(rep.calls.iter().any(|c| c.contains(KUKA_DIM3.0)));
    let head = rep.projects.iter().find(|p| p.name == HEAD).unwrap();
    assert!(head.built && head.deviations.is_empty(), "{head:?}");
    assert!(
        rep.stages[0]
            .notes
            .iter()
            .any(|x| x.contains("kuka_robi2: slot 1 (_module_1) unplugged")),
        "{:?}",
        rep.stages[0].notes
    );
}

#[test]
fn a_module_that_stays_in_a_slot_that_must_be_empty_is_a_deviation() {
    let dir = example_without_safety_module("kuka_robi2", "R2");
    let m = Manifest::parse(&std::fs::read_to_string(dir.path().join("plc/project.json")).unwrap())
        .unwrap();
    let (_, _, _, rep) = run(
        Behaviour {
            tree: Some((dir.path().to_path_buf(), m)),
            stuck_module: true,
            ..Behaviour::default()
        },
        &[Stage::Projects],
        true,
        quick(),
    );
    let head = rep.projects.iter().find(|p| p.name == HEAD).unwrap();
    assert!(
        head.deviations
            .iter()
            .any(|d| d.contains("kuka_robi2: slot 1 still holds _module_1")),
        "{head:?}"
    );
}

#[test]
fn a_device_without_slots_gets_its_modules_by_module_id() {
    let (s, _, r, rep) = run(
        Behaviour {
            free_list: true,
            ..Behaviour::default()
        },
        &[Stage::Projects],
        true,
        quick(),
    );
    r.unwrap();
    let calls = s.calls();
    let n = |c: &str| calls.iter().filter(|x| *x == c).count();
    // Ethernet, kuka_robi1 and the two modules.
    assert_eq!(n(&format!("add_device {R1}")), 4);
    assert_eq!(n(&format!("plug_device {R1}")), 0);
    for p in &rep.projects {
        assert!(
            p.built && p.deviations.is_empty() && p.human_steps.is_empty(),
            "{p:?}"
        );
        assert_eq!(p.warnings, ["3 unused variables"], "the idents read back");
    }
    assert!(
        rep.stages[0]
            .notes
            .iter()
            .any(|x| x.contains("module VASS_DIO256 (module id 7) added")),
        "{:?}",
        rep.stages[0].notes
    );
}

#[test]
fn the_plain_server_leaves_nested_members_to_a_person() {
    let (_, _, r, rep) = run(
        Behaviour {
            plain_server: true,
            ..Behaviour::default()
        },
        &[Stage::Preflight, Stage::Projects],
        true,
        quick(),
    );
    let e = r.unwrap_err();
    assert_eq!(e.exit_code(), 7, "{e}");
    assert!(
        rep.stages[0]
            .notes
            .iter()
            .any(|n| n.contains("elementPath")),
        "{:?}",
        rep.stages[0].notes
    );
    // StationName of the simulators is one level down: set by index.
    assert!(
        rep.projects[0].deviations.is_empty(),
        "{:?}",
        rep.projects[0]
    );
    let head = &rep.projects[2];
    assert_eq!(head.deviations.len(), 6, "{:?}", head.deviations);
    assert!(head.deviations[0].contains("Identification is a struct"));
    assert_eq!(
        head.human_steps
            .iter()
            .filter(|h| h.contains("in the device editor"))
            .count(),
        6
    );
}

#[test]
fn a_device_without_slots_is_a_human_step_on_the_plain_server() {
    let (s, _, r, rep) = run(
        Behaviour {
            free_list: true,
            plain_server: true,
            ..Behaviour::default()
        },
        &[Stage::Projects],
        true,
        quick(),
    );
    let e = r.unwrap_err();
    assert_eq!(e.exit_code(), 7, "{e}");
    let r1 = &rep.projects[0];
    assert!(r1.built, "{r1:?}");
    assert_eq!(r1.human_steps.len(), 2, "{:?}", r1.human_steps);
    assert!(r1.human_steps[0].contains("module id 1"));
    assert!(
        r1.deviations
            .iter()
            .any(|d| d.contains("IO mapping row(s) left out")),
        "{:?}",
        r1.deviations
    );
    assert!(!s.calls().contains(&format!("plug_device {R1}")));
    assert_eq!(
        s.calls()
            .iter()
            .filter(|c| **c == format!("add_device {R1}"))
            .count(),
        2
    );
}

#[test]
fn a_declaration_that_does_not_read_back_is_a_deviation() {
    let (_, _, r, rep) = run(
        Behaviour {
            lose_declaration: Some("ST_Ticket".into()),
            ..Behaviour::default()
        },
        &[Stage::Projects],
        true,
        quick(),
    );
    let e = r.unwrap_err();
    assert_eq!(e.exit_code(), 7, "{e}");
    assert_eq!(
        rep.projects[2].deviations,
        ["ST_Ticket: declaration differs after writing it"]
    );
}

#[test]
fn preflight_refuses_an_interactive_codesys() {
    let (_, _, r, _) = run(
        Behaviour {
            interactive: true,
            ..Behaviour::default()
        },
        &[Stage::Preflight],
        false,
        quick(),
    );
    let e = r.unwrap_err();
    assert_eq!(e.exit_code(), 3);
    assert!(e.to_string().contains("opened by hand"), "{e}");
    assert!(e.to_string().contains("PID 6632"), "{e}");
}

#[test]
fn the_first_call_waits_for_a_bridge_that_is_not_up_yet() {
    let port = std::net::TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port();
    let url = format!("http://127.0.0.1:{port}/mcp");
    let (up, is_up) = std::sync::mpsc::channel();
    let (done, finish) = std::sync::mpsc::channel::<()>();
    let server = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_secs(2));
        let _s = Server::start_on(Behaviour::default(), port);
        up.send(()).unwrap();
        let _ = finish.recv();
    });
    let mut cfg = config(&url).codesys_mcp;
    cfg.connect_wait_s = 30;
    let started = std::time::Instant::now();
    let tools = McpTools::new(cfg).unwrap().list().unwrap();
    assert!(started.elapsed() >= Duration::from_millis(1500));
    assert!(tools.iter().any(|t| t.name == "get_codesys_status"));
    is_up.recv().unwrap();

    // Without a wait the first call fails at once; a refused token never waits.
    let mut cfg = config("http://127.0.0.1:9/mcp").codesys_mcp;
    cfg.connect_wait_s = 0;
    assert!(McpTools::new(cfg).unwrap().list().is_err());
    let mut cfg = config(&url).codesys_mcp;
    cfg.token_env = None;
    cfg.connect_wait_s = 60;
    let started = std::time::Instant::now();
    let e = McpTools::new(cfg).unwrap().list().unwrap_err();
    assert!(started.elapsed() < Duration::from_secs(10), "{e}");
    done.send(()).unwrap();
    server.join().unwrap();
}

/// A per-workspace cell (`other_codesys = "close"`).
fn unattended(url: &str) -> CellConfig {
    let mut c = config(url);
    c.codesys_mcp.other_codesys = OtherCodesys::Close;
    c
}

fn launches(s: &Server) -> Vec<String> {
    s.calls()
        .into_iter()
        .filter(|c| c.starts_with("launch_codesys"))
        .collect()
}

#[test]
fn an_unattended_cell_closes_a_stray_codesys_without_unsaved_changes() {
    let (s, _, r, rep) = run_cfg(
        Behaviour {
            stray: Some("no unsaved changes"),
            ..Behaviour::default()
        },
        &[Stage::Preflight],
        false,
        quick(),
        unattended,
    );
    r.unwrap();
    assert_eq!(launches(&s).len(), 1, "{:?}", s.calls());
    let notes = &rep.stages[0].notes;
    assert!(
        notes.iter().any(
            |n| n.contains("PID 2540, opened by hand: CODESYS; no unsaved changes")
                && n.contains("closed")
                && n.contains("PID 2540: closed")
        ),
        "{notes:?}"
    );
}

#[test]
fn an_unattended_cell_keeps_a_stray_codesys_with_unsaved_changes() {
    for unsaved in ["UNSAVED CHANGES", "unsaved state unknown"] {
        let (s, _, r, rep) = run_cfg(
            Behaviour {
                stray: Some(unsaved),
                ..Behaviour::default()
            },
            &[Stage::Preflight],
            false,
            quick(),
            unattended,
        );
        let e = r.unwrap_err();
        assert_eq!(e.exit_code(), 3, "{e}");
        assert!(e.to_string().contains("opened by hand"), "{e}");
        assert!(launches(&s).is_empty(), "{:?}", s.calls());
        assert!(
            rep.stages[0]
                .notes
                .iter()
                .any(|n| n.contains("was not closed")),
            "{:?}",
            rep.stages[0].notes
        );
    }
}

#[test]
fn a_codesys_an_ended_mcp_session_left_is_waited_for_not_closed() {
    // Still loading: the server takes it over once its watcher answers (here after 3 questions),
    // on the shared cell (refuse) as on an unattended one.
    for cfg in [config as fn(&str) -> CellConfig, unattended] {
        let (s, _, r, _) = run_cfg(
            Behaviour {
                stray: Some("no unsaved changes"),
                stray_left: Some(3),
                ..Behaviour::default()
            },
            &[Stage::Preflight],
            false,
            quick(),
            cfg,
        );
        r.unwrap();
        assert!(launches(&s).is_empty(), "{:?}", s.calls());
        assert_eq!(
            s.calls()
                .iter()
                .filter(|c| c.starts_with("get_codesys_status"))
                .count(),
            4
        );
    }
    // Never taken over: refused after others_timeout, or closed on an unattended cell.
    let b = Behaviour {
        stray: Some("no unsaved changes"),
        stray_left: Some(0),
        ..Behaviour::default()
    };
    let (_, _, r, _) = run(b.clone(), &[Stage::Preflight], false, quick());
    assert_eq!(r.unwrap_err().exit_code(), 3);
    let (s, _, r, _) = run_cfg(b, &[Stage::Preflight], false, quick(), unattended);
    r.unwrap();
    assert_eq!(launches(&s).len(), 1, "{:?}", s.calls());
}

#[test]
fn a_stray_codesys_is_refused_by_default() {
    let (s, _, r, _) = run(
        Behaviour {
            stray: Some("no unsaved changes"),
            ..Behaviour::default()
        },
        &[Stage::Preflight],
        false,
        quick(),
    );
    let e = r.unwrap_err();
    assert_eq!(e.exit_code(), 3, "{e}");
    assert!(e.to_string().contains("PID 2540"), "{e}");
    assert!(launches(&s).is_empty(), "{:?}", s.calls());
}

#[test]
fn preflight_names_the_human_step_for_a_missing_device_description() {
    let (_, _, r, rep) = run(
        Behaviour {
            missing_devices: vec![DeviceId {
                ty: 83,
                id: "0x019B_0x0301_DIM 2".into(),
                version: "1 V8.2".into(),
            }],
            ..Behaviour::default()
        },
        &Stage::ALL,
        true,
        quick(),
    );
    let e = r.unwrap_err();
    assert_eq!(e.exit_code(), 3);
    assert!(rep.projects[0].human_steps[0].starts_with("pn_device_desc: Tools > Miscellaneous"));
    assert!(!rep.projects[1].human_steps.is_empty());
    assert!(
        rep.projects[2].human_steps.is_empty(),
        "the head PLC does not need it"
    );
}

#[test]
fn projects_without_a_device_description_is_a_config_error() {
    let (_, _, r, _) = run(
        Behaviour {
            missing_devices: vec![DeviceId {
                ty: 110,
                id: "0000 0002".into(),
                version: "4.2.0.0".into(),
            }],
            ..Behaviour::default()
        },
        &[Stage::Projects],
        true,
        quick(),
    );
    let e = r.unwrap_err();
    assert_eq!(e.exit_code(), 3, "{e}");
    assert!(
        e.to_string().contains("(110, '0000 0002', '4.2.0.0')"),
        "{e}"
    );
}

#[test]
fn first_login_creates_the_device_user() {
    let (s, _, r, rep) = run(
        Behaviour {
            first_login: true,
            ..Behaviour::default()
        },
        &[Stage::Projects, Stage::Deploy],
        true,
        quick(),
    );
    r.unwrap();
    let calls = s.calls();
    let i = calls
        .iter()
        .position(|c| *c == format!("add_device_user {R1}"))
        .unwrap();
    assert_eq!(calls[i - 1], format!("connect_to_device {R1}"));
    assert_eq!(calls[i + 1], format!("connect_to_device {R1}"));
    assert_eq!(calls[i + 2], format!("download_to_device {R1}"));
    assert!(
        rep.stages[1]
            .notes
            .iter()
            .any(|n| n == "first login: device user created")
    );
}

#[test]
fn an_ide_without_a_gateway_stops_deploy_with_a_human_step() {
    let (s, _, r, rep) = run(
        Behaviour {
            no_gateway: true,
            ..Behaviour::default()
        },
        &[Stage::Deploy],
        true,
        quick(),
    );
    let e = r.unwrap_err();
    assert_eq!(e.exit_code(), 3, "{e}");
    assert!(e.to_string().contains("No gateway is configured"), "{e}");
    assert_eq!(
        s.calls()
            .iter()
            .filter(|c| c.starts_with("rebind_device_to_scan_result"))
            .count(),
        1,
        "stops at the first project"
    );
    for p in &rep.projects {
        assert!(
            p.human_steps.iter().any(|h| h.contains("Gateway-1")),
            "{p:?}"
        );
    }
}

#[test]
fn a_password_in_a_tool_answer_is_masked() {
    let (_, _, r, rep) = run(
        Behaviour {
            echo_password: true,
            ..Behaviour::default()
        },
        &[Stage::Deploy],
        true,
        quick(),
    );
    let e = r.unwrap_err();
    assert_eq!(e.exit_code(), 7);
    let text = serde_json::to_string(&rep).unwrap() + &e.to_string();
    assert!(!text.contains("test-password-0123"), "{text}");
    assert!(
        rep.projects[0]
            .errors
            .iter()
            .any(|e| e.contains("login failed for password <secret>")),
        "{:?}",
        rep.projects[0].errors
    );
}

#[test]
fn verify_reads_again_until_profinet_is_up() {
    let settings = Settings {
        verify_timeout: Duration::from_secs(30),
        ..quick()
    };
    let (s, _, r, rep) = run(
        Behaviour {
            pn_down: 1,
            ..Behaviour::default()
        },
        &[Stage::Verify],
        true,
        settings,
    );
    r.unwrap();
    assert_eq!(rep.stages[0].notes, ["verified after 2 attempt(s)"]);
    let reads = s
        .calls()
        .iter()
        .filter(|c| c.starts_with("read_variables"))
        .count();
    // The simulators have no PROFINET check of their own: only the head reads again.
    assert_eq!(reads, 4);

    // Without time to wait it fails with the values read.
    let (_, _, r, rep) = run(
        Behaviour {
            pn_down: 1,
            ..Behaviour::default()
        },
        &[Stage::Verify],
        true,
        quick(),
    );
    let e = r.unwrap_err();
    assert_eq!(e.exit_code(), 7);
    let head = &rep.projects[2];
    let pn = head
        .checks
        .iter()
        .find(|c| c.expr == "kuka_robi2.xRunning")
        .unwrap();
    assert_eq!(pn.value.as_deref(), Some("FALSE"));
    assert!(!pn.ok);
    assert!(
        head.checks
            .iter()
            .find(|c| c.expr == "GVL_Cell.PLAN_ID")
            .unwrap()
            .ok
    );
}

#[test]
fn a_wrong_token_is_an_io_error() {
    let s = Server::start(Behaviour::default());
    let mut cfg = config(&s.url).codesys_mcp;
    cfg.token_env = Some("PLAN2CELL_CELL_TEST_WRONG_TOKEN".into());
    // SAFETY: as in config().
    unsafe { std::env::set_var("PLAN2CELL_CELL_TEST_WRONG_TOKEN", "wrong-token-0123456789") };
    let mut tools = McpTools::new(cfg).unwrap();
    let e = tools.list().unwrap_err();
    assert_eq!(e.exit_code(), 4, "{e}");
    assert!(s.calls().is_empty());
}

#[test]
fn dry_run_sends_nothing_and_prints_every_call() {
    let settings = Settings {
        dry_run: true,
        ..quick()
    };
    let (s, remote, r, rep) = run(Behaviour::default(), &Stage::ALL, true, settings);
    r.unwrap();
    assert!(remote.calls.is_empty());
    assert!(s.calls().is_empty());
    let calls = &rep.calls;
    assert!(calls[0].starts_with("ssh plc-host: if test -x"));
    assert!(calls[1].ends_with(": tools/list"));
    assert!(calls.iter().any(|c| c.contains(
        r#"create_project(filePath="C:\\ProgramData\\plan2cell\\projects\\welding_cell_2_small\\RobotSim_welding_cell_2_small_R1.project", install="SP22 Patch 1", deviceName=<name of 4102 '0000 0005' '4.20.0.0'>)"#
    )), "{calls:#?}");
    assert!(
        calls
            .iter()
            .any(|c| c.contains("create_task(") && c.ends_with("[if task MainTask is missing]"))
    );
    assert!(calls.iter().all(|c| !c.contains("test-password")));
    assert!(calls.iter().any(|c| c.contains("devicePassword=<secret>")));
    let last = calls.last().unwrap();
    assert!(
        last.contains("disconnect_from_device(") && last.contains(HEAD),
        "{last}"
    );
}

#[test]
fn a_later_run_replaces_the_human_steps_it_no_longer_needs() {
    // A plain server: module and member steps, and an IDE without a gateway.
    let (_, _, _, before) = run(
        Behaviour {
            free_list: true,
            plain_server: true,
            no_gateway: true,
            ..Behaviour::default()
        },
        &[Stage::Projects, Stage::Deploy],
        true,
        quick(),
    );
    assert!(before.projects.iter().all(|p| !p.human_steps.is_empty()));
    // The overlay's server and a gateway: the same build directory, the earlier report.
    let s = Server::start(Behaviour {
        free_list: true,
        ..Behaviour::default()
    });
    let mut remote = plc_host();
    let mut tools = McpTools::new(config(&s.url).codesys_mcp).unwrap();
    let mut c = Cell::new(
        config(&s.url),
        Some(Tree::Dir(example())),
        quick(),
        &mut remote,
        &mut tools,
        Some(before),
    )
    .unwrap();
    c.run(&[Stage::Projects, Stage::Deploy]).unwrap();
    for p in &c.report.projects {
        assert!(p.human_steps.is_empty(), "{p:?}");
        assert!(p.deviations.is_empty(), "{p:?}");
    }
}

// ---------------------------------------------------------------- KUKA VC (krl stage)

mod kvc {
    use super::*;
    use plan2cell_cell::config::{Access, KukaController, KukaVc};
    use std::cell::RefCell;
    use std::rc::Rc;

    const HELPER: &str = "sudo -n python3 -I /var/lib/plan2cell/kuka-vc/p2c-kvc.py";

    fn vc_config(url: &str) -> CellConfig {
        let mut c = config(url);
        c.kuka_vc = Some(KukaVc {
            controllers: vec![KukaController {
                driver: Default::default(),
                robot_ns: "robot1".into(),
                instance: "iiQKA.OS2_KSS_9.2.0_vass".into(),
                netns: "krc1".into(),
                kli_ip: Some("10.200.0.11".into()),
                station: None,
                ssm_instance: Some("i-0123456789abcdef0".into()),
                kubectl_pod: None,
                ssh_host: None,
                sps_base: Some("iiQKA.OS2_KSS_9.2.0".into()),
            }],
            ..KukaVc::default()
        });
        c
    }

    #[test]
    fn the_robot_simulator_of_a_kuka_vc_robot_is_left_out() {
        let (s, _, r, rep) = run_cfg(
            Behaviour::default(),
            &[Stage::Projects, Stage::Deploy],
            true,
            quick(),
            vc_config,
        );
        r.unwrap();
        let names: Vec<&str> = rep.projects.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(names, [R2, HEAD]);
        assert!(!s.calls().iter().any(|c| c.contains(R1)), "{:?}", s.calls());
        for st in &rep.stages {
            assert!(
                st.notes
                    .iter()
                    .any(|n| n.starts_with(&format!("{R1} left out: robot1 is a KUKA VC"))),
                "{:?}",
                st.notes
            );
        }
    }

    const DEPLOYED: &str = r#"{"ok":true,"tag":"robot1 16#F77E","changed":["KRC:\\R1\\Folgen\\folge11.src","KRC:\\R1\\System\\$config.dat"],"unchanged":7,"errors":[],"confirmed":["M_1362"],"messages":[]}
p2c-exit=0
"#;
    const VERIFIED: &str = r#"{"ok":true,"checks":[{"expr":"compile_errors","expect":"0","value":"0","ok":true},{"expr":"FOLGE11 PLAN_ID","expect":"32804","value":"32804","ok":true}],"errors":[],"messages":[{"key":"M_5001","type":"EVENT","stop":false,"text":"x"}]}
p2c-exit=0
"#;

    type Vc = Rc<RefCell<MockRemote>>;

    fn run_vc(
        b: Behaviour,
        stages: &[Stage],
        vc: MockRemote,
        cfg: impl Fn(&str) -> CellConfig,
    ) -> (
        Vc,
        plan2cell_cell::Result<()>,
        plan2cell_cell::Report,
        Vec<String>,
    ) {
        let s = Server::start(b);
        let mut remote = plc_host();
        let mut tools = McpTools::new(config(&s.url).codesys_mcp).unwrap();
        let shared: Vc = Rc::new(RefCell::new(vc));
        let mut map: HashMap<Access, Vc> = HashMap::new();
        map.insert(Access::Ssm("i-0123456789abcdef0".into()), shared.clone());
        let mut c = Cell::new(
            cfg(&s.url),
            Some(Tree::Dir(example())),
            quick(),
            &mut remote,
            &mut tools,
            None,
        )
        .unwrap()
        .with_controllers(Box::new(map));
        let r = c.run(stages);
        let report = c.report.clone();
        drop(c);
        let calls = s.calls();
        (shared, r, report, calls)
    }

    /// `start` with this head, PLC host, KUKA VC host, cell configuration and settings.
    fn start_vc(
        b: Behaviour,
        mut remote: MockRemote,
        vc: MockRemote,
        cfg: impl Fn(&str) -> CellConfig,
        settings: Settings,
    ) -> (
        Vc,
        plan2cell_cell::Result<()>,
        plan2cell_cell::Report,
        Vec<String>,
    ) {
        let s = Server::start(b);
        let mut tools = McpTools::new(config(&s.url).codesys_mcp).unwrap();
        let shared: Vc = Rc::new(RefCell::new(vc));
        let mut map: HashMap<Access, Vc> = HashMap::new();
        map.insert(Access::Ssm("i-0123456789abcdef0".into()), shared.clone());
        let mut c = Cell::new(
            cfg(&s.url),
            Some(Tree::Dir(example())),
            settings,
            &mut remote,
            &mut tools,
            None,
        )
        .unwrap()
        .with_controllers(Box::new(map));
        let r = c.run(&[Stage::Start]);
        let report = c.report.clone();
        drop(c);
        let calls = s.calls();
        (shared, r, report, calls)
    }

    fn start_with(
        vc: MockRemote,
    ) -> (
        Vc,
        plan2cell_cell::Result<()>,
        plan2cell_cell::Report,
        Vec<String>,
    ) {
        let b = Behaviour {
            cell_states: vec!["CS_ERROR", "CS_RUNNING"],
            ..Behaviour::default()
        };
        start_vc(b, super::plc_host_up(), vc, vc_config, quick())
    }

    /// As [`vc_config`], the controller running the head's robot 2 (the mock's faulted robot).
    fn vc_config_r2(url: &str) -> CellConfig {
        let mut c = vc_config(url);
        c.kuka_vc.as_mut().unwrap().controllers[0].robot_ns = "robot2".into();
        c
    }

    fn runs(vc: &Vc) -> Vec<String> {
        vc.borrow()
            .calls
            .iter()
            .filter_map(|c| match c {
                Call::Run(x) => Some(x.clone()),
                _ => None,
            })
            .collect()
    }

    fn index_of(notes: &[String], f: impl Fn(&str) -> bool) -> usize {
        notes
            .iter()
            .position(|n| f(n))
            .unwrap_or_else(|| panic!("{notes:#?}"))
    }

    const MODE: &str =
        "sudo -n /var/lib/plan2cell/kuka-vc/kvc-mode.sh 1 EXT 2>&1; echo p2c-exit=$?";
    const PROBE: &str =
        "sudo -n test -x /var/lib/plan2cell/kuka-vc/kvc-ack.sh && echo present || echo missing";
    const ACK: &str =
        "sudo -n /var/lib/plan2cell/kuka-vc/kvc-ack.sh --netns krc1 2>&1; echo p2c-exit=$?";
    const ALREADY_EXT: &str = "controller 1 is already in EXT ($MODE_OP=#EX)\np2c-exit=0\n";
    const ACKED: &str = "== kvc-ack.sh 2026-10-13T08:00:00Z on kvc-a: controller 1 (krc1, x)\n\
        stop messages pending:\n   M_2858 QUITT Ackn. stop for error fieldbus\n\
        ok: acknowledged in T1, no stop message left; controller 1 in EXT\np2c-exit=0\n";

    #[test]
    fn without_kvc_ack_start_falls_back_to_ext_after_the_head_login() {
        // An older bundle: no kvc-ack.sh on the host.
        let vc = MockRemote::default()
            .answer(PROBE, Ok("missing\n".into()))
            .answer(MODE, Ok(ALREADY_EXT.into()));
        let (vc, r, rep, mcp) = start_with(vc);
        r.unwrap();
        assert_eq!(runs(&vc), [PROBE, MODE]);
        assert!(mcp.contains(&format!("write_variable {HEAD}")));
        let notes = &rep.stages[0].notes;
        let login = index_of(notes, |n| n.starts_with("head PLAN_ID"));
        let fallback = index_of(notes, |n| {
            n == "robot1 (krc1): no /var/lib/plan2cell/kuka-vc/kvc-ack.sh on the host (older \
                  bundle): kvc-mode.sh 1 EXT only"
        });
        let ext = index_of(notes, |n| {
            n == "robot1 (krc1): controller 1 is already in EXT ($MODE_OP=#EX)"
        });
        let pulse = index_of(notes, |n| n.ends_with("PLC_PRG.bReset pulsed"));
        assert!(
            login < fallback && fallback < ext && ext < pulse,
            "{notes:#?}"
        );
    }

    #[test]
    fn start_acknowledges_the_stop_messages_with_kvc_ack_before_the_pulses() {
        let vc = MockRemote::default()
            .answer(PROBE, Ok("present\n".into()))
            .answer(ACK, Ok(ACKED.into()));
        let (vc, r, rep, _) = start_with(vc);
        r.unwrap();
        // kvc-ack.sh ended in EXT: no kvc-mode.sh after it.
        assert_eq!(runs(&vc), [PROBE, ACK]);
        let notes = &rep.stages[0].notes;
        let ack = index_of(notes, |n| {
            n == "robot1 (krc1): kvc-ack.sh: ok: acknowledged in T1, no stop message left; \
                  controller 1 in EXT"
        });
        let pulse = index_of(notes, |n| n.ends_with("PLC_PRG.bReset pulsed"));
        assert!(ack < pulse, "{notes:#?}");
        assert!(
            rep.calls
                .contains(&format!("ssm:i-0123456789abcdef0: {ACK}")),
            "{:#?}",
            rep.calls
        );
    }

    #[test]
    fn nothing_to_acknowledge_still_makes_sure_of_ext() {
        // kvc-ack.sh with nothing pending leaves the mode alone: kvc-mode.sh EXT after it.
        let vc = MockRemote::default()
            .answer(PROBE, Ok("present\n".into()))
            .answer(
                ACK,
                Ok("== kvc-ack.sh\nok: no stop messages on controller 1\np2c-exit=0\n".into()),
            )
            .answer(MODE, Ok(ALREADY_EXT.into()));
        let (vc, r, rep, _) = start_with(vc);
        r.unwrap();
        assert_eq!(runs(&vc), [PROBE, ACK, MODE]);
        let notes = &rep.stages[0].notes;
        index_of(notes, |n| {
            n == "robot1 (krc1): kvc-ack.sh: ok: no stop messages on controller 1"
        });
        index_of(notes, |n| {
            n == "robot1 (krc1): controller 1 is already in EXT ($MODE_OP=#EX)"
        });
    }

    #[test]
    fn stop_messages_that_remain_are_a_note_and_the_pulses_go_on() {
        let vc = MockRemote::default()
            .answer(PROBE, Ok("present\n".into()))
            .answer(
                ACK,
                Ok(
                    "remaining stop messages (controller 1 in EXT):\n   M_200 STATE x\n\
                    FAIL: stop messages remain on controller 1 (kss-msgs; docs/CELL.md M_2858)\n\
                    p2c-exit=1\n"
                        .into(),
                ),
            )
            .answer(MODE, Ok(ALREADY_EXT.into()));
        let (vc, r, rep, mcp) = start_with(vc);
        r.unwrap();
        assert_eq!(runs(&vc), [PROBE, ACK, MODE]);
        assert!(mcp.contains(&format!("write_variable {HEAD}")));
        index_of(&rep.stages[0].notes, |n| {
            n.starts_with("robot1 (krc1): kvc-ack.sh: stop messages remain: ")
                && n.contains("M_200 STATE x | FAIL: stop messages remain on controller 1")
        });
    }

    #[test]
    fn after_a_runtime_restart_kvc_ack_waits_for_the_profinet_ar() {
        let vc = MockRemote::default()
            .answer(PROBE, Ok("present\n".into()))
            .answer(ACK, Ok(ACKED.into()));
        let remote = super::plc_host_up().answer("for i in $(seq", Ok("open\n".into()));
        let settings = Settings {
            restart_runtime: true,
            ack_settle: Duration::from_millis(400),
            ..quick()
        };
        let b = Behaviour {
            cell_states: vec!["CS_ERROR", "CS_RUNNING"],
            ..Behaviour::default()
        };
        let t = std::time::Instant::now();
        let (vc, r, rep, _) = start_vc(b, remote, vc, vc_config, settings);
        r.unwrap();
        assert!(t.elapsed() >= Duration::from_millis(400));
        assert_eq!(runs(&vc), [PROBE, ACK]);
        let notes = &rep.stages[0].notes;
        let restarted = index_of(notes, |n| {
            n.starts_with("restarted ") && n.ends_with("(as asked)")
        });
        let wait = index_of(notes, |n| {
            n.starts_with("waiting ")
                && n.ends_with("after the runtime restart (PROFINET AR) before kvc-ack.sh")
        });
        let ack = index_of(notes, |n| n.starts_with("robot1 (krc1): kvc-ack.sh: ok"));
        assert!(restarted < wait && wait < ack, "{notes:#?}");
    }

    #[test]
    fn a_robot_still_in_start_timeout_after_3_pulses_gets_kvc_ack_once_more() {
        // The mock's fault: ERR_START_TIMEOUT robot 2; the 4th bReset clears it.
        let vc = MockRemote::default()
            .answer(PROBE, Ok("present\n".into()))
            .answer(ACK, Ok(ACKED.into()));
        let b = Behaviour {
            cell_states: vec!["CS_ERROR", "CS_RUNNING"],
            stuck_pulses: 3,
            ..Behaviour::default()
        };
        let (vc, r, rep, _) = start_vc(b, super::plc_host_up(), vc, vc_config_r2, quick());
        r.unwrap();
        assert_eq!(runs(&vc), [PROBE, ACK, PROBE, ACK]);
        let notes = &rep.stages[0].notes;
        let pulsed: Vec<usize> = notes
            .iter()
            .enumerate()
            .filter(|(_, n)| n.ends_with("PLC_PRG.bReset pulsed"))
            .map(|(i, _)| i)
            .collect();
        assert_eq!(pulsed.len(), 4, "{notes:#?}");
        let again = index_of(notes, |n| {
            n == "robot 2 still ERR_START_TIMEOUT after 3 pulse(s): kvc-ack.sh on robot2 once more"
        });
        assert!(pulsed[2] < again && again < pulsed[3], "{notes:#?}");
        assert!(
            notes
                .last()
                .unwrap()
                .starts_with("in production after 5 read(s) and 4 pulse(s)")
        );
    }

    #[test]
    fn kvc_ack_runs_once_more_per_robot_not_on_every_pulse() {
        let vc = MockRemote::default()
            .answer(PROBE, Ok("present\n".into()))
            .answer(ACK, Ok(ACKED.into()));
        let b = Behaviour {
            cell_states: vec!["CS_ERROR", "CS_RUNNING"],
            stuck_cell: true,
            ..Behaviour::default()
        };
        let (vc, r, _, mcp) = start_vc(b, super::plc_host_up(), vc, vc_config_r2, quick());
        assert_eq!(r.unwrap_err().exit_code(), 7);
        assert_eq!(runs(&vc), [PROBE, ACK, PROBE, ACK]);
        assert_eq!(
            mcp.iter()
                .filter(|c| c.starts_with("write_variable"))
                .count(),
            8
        );
    }

    #[test]
    fn a_robot_without_a_controller_is_only_a_note() {
        // The fault is on robot 2; the cell.toml has only robot1's controller.
        let vc = MockRemote::default()
            .answer(PROBE, Ok("present\n".into()))
            .answer(ACK, Ok(ACKED.into()));
        let b = Behaviour {
            cell_states: vec!["CS_ERROR", "CS_RUNNING"],
            stuck_pulses: 3,
            ..Behaviour::default()
        };
        let (vc, r, rep, _) = start_vc(b, super::plc_host_up(), vc, vc_config, quick());
        r.unwrap();
        assert_eq!(runs(&vc), [PROBE, ACK]);
        index_of(&rep.stages[0].notes, |n| {
            n == "robot 2 still ERR_START_TIMEOUT: no [[kuka_vc.controller]] for robot2 to acknowledge"
        });
    }

    #[test]
    fn a_controller_that_does_not_reach_ext_stops_start_before_any_pulse() {
        let vc = MockRemote::default().answer(
            MODE,
            Ok("cannot drive the smartPAD here: no playwright\np2c-exit=3\n".into()),
        );
        let (_, r, _, mcp) = start_with(vc);
        let e = r.unwrap_err();
        assert_eq!(e.exit_code(), 7);
        assert!(
            e.to_string()
                .contains("robot1 (krc1): exit 3: cannot drive the smartPAD here: no playwright"),
            "{e}"
        );
        assert!(mcp.iter().any(|c| c.starts_with("connect_to_device")));
        assert!(!mcp.iter().any(|c| c.starts_with("write_variable")));
        assert_eq!(
            mcp[0],
            format!("close_project {HEAD}"),
            "before the bind saves: {mcp:?}"
        );
        assert_eq!(
            mcp.last().unwrap(),
            &format!("disconnect_from_device {HEAD}")
        );
    }

    #[test]
    fn krl_copies_helper_and_bundle_then_deploys() {
        let vc = MockRemote::default().answer(&format!("{HELPER} deploy"), Ok(DEPLOYED.into()));
        let (vc, r, rep, mcp) = run_vc(Behaviour::default(), &[Stage::Krl], vc, vc_config);
        r.unwrap();
        assert!(mcp.is_empty(), "krl calls no MCP tool: {mcp:?}");
        let calls = vc.borrow().calls.clone();
        assert_eq!(calls.len(), 6, "{calls:#?}");
        let copies: Vec<&str> = calls
            .iter()
            .filter_map(|c| match c {
                Call::Copy(_, r) => Some(r.as_str()),
                _ => None,
            })
            .collect();
        assert_eq!(
            copies,
            [
                "/tmp/plan2cell-robot1-p2c-kvc.py",
                "/tmp/plan2cell-robot1-p2c-headstub.py",
                "/tmp/plan2cell-robot1-kvc-t1-test.sh",
                "/tmp/plan2cell-robot1-bundle.json"
            ]
        );
        let Call::Run(install) = &calls[4] else {
            panic!("{calls:?}")
        };
        assert!(install.contains(
            "sudo -n install -D -m 0644 /tmp/plan2cell-robot1-bundle.json \
             /var/lib/plan2cell/kuka-vc/bundles/robot1.json"
        ));
        assert_eq!(
            calls[5],
            Call::Run(format!(
                "{HELPER} deploy --netns krc1 --instance iiQKA.OS2_KSS_9.2.0_vass --bundle \
                 /var/lib/plan2cell/kuka-vc/bundles/robot1.json --sps-base iiQKA.OS2_KSS_9.2.0 \
                 2>&1; echo p2c-exit=$?"
            ))
        );
        let c = &rep.controllers[0];
        assert_eq!(c.robot_ns, "robot1");
        assert_eq!(c.host, "ssm:i-0123456789abcdef0");
        assert_eq!(c.plan_id.as_deref(), Some("16#F77E"));
        assert_eq!(c.deployed, Some(true));
        assert_eq!(c.changed.len(), 2);
        assert_eq!(rep.stages.len(), 1);
        assert!(
            rep.stages[0].notes[0].contains("2 file(s) changed, 7 unchanged, messages confirmed"),
            "{:?}",
            rep.stages[0].notes
        );
    }

    #[test]
    fn compile_errors_on_the_controller_are_exit_8() {
        let vc = MockRemote::default().answer(
            &format!("{HELPER} deploy"),
            Ok(r#"{"ok":false,"error":"1 compile error(s)","changed":[],"errors":[{"module":"KRC:\\R1\\System\\bt_sps.src","line":12,"number":2057,"param":"BT_X"}]}
p2c-exit=4
"#
            .into()),
        );
        let (_, r, rep, _) = run_vc(Behaviour::default(), &[Stage::Krl], vc, vc_config);
        let e = r.unwrap_err();
        assert_eq!(e.exit_code(), 8, "{e}");
        assert!(
            e.to_string().contains("bt_sps.src:12 error 2057 BT_X"),
            "{e}"
        );
        assert_eq!(rep.controllers[0].deployed, Some(false));
        assert!(!rep.ok);
    }

    #[test]
    fn a_wrong_instance_is_a_precondition_and_no_answer_an_io_error() {
        let vc = MockRemote::default().answer(
            &format!("{HELPER} deploy"),
            Ok("{\"ok\":false,\"error\":\"active instance A, want B\"}\np2c-exit=3\n".into()),
        );
        let (_, r, _, _) = run_vc(Behaviour::default(), &[Stage::Krl], vc, vc_config);
        assert_eq!(r.unwrap_err().exit_code(), 3);
        let vc = MockRemote::default().answer(
            &format!("{HELPER} deploy"),
            Ok("Traceback (most recent call last):\np2c-exit=1\n".into()),
        );
        let (_, r, _, _) = run_vc(Behaviour::default(), &[Stage::Krl], vc, vc_config);
        assert_eq!(r.unwrap_err().exit_code(), 4);
    }

    #[test]
    fn verify_checks_the_controller_before_the_plcs() {
        let vc = MockRemote::default().answer(&format!("{HELPER} verify"), Ok(VERIFIED.into()));
        let (vc, r, rep, mcp) = run_vc(Behaviour::default(), &[Stage::Verify], vc, vc_config);
        r.unwrap();
        assert!(mcp.iter().any(|c| c.starts_with("read_variables ")));
        assert_eq!(vc.borrow().calls.len(), 1);
        let c = &rep.controllers[0];
        assert_eq!(c.verified, Some(true));
        assert_eq!(c.checks.len(), 2);
        assert_eq!(c.messages, ["M_5001 EVENT x"]);

        let failing = VERIFIED.replace(
            r#""value":"32804","ok":true}],"errors""#,
            r#""value":null,"ok":false}],"errors""#,
        );
        let failing = failing.replacen("{\"ok\":true", "{\"ok\":false", 1);
        let vc = MockRemote::default().answer(&format!("{HELPER} verify"), Ok(failing));
        let (_, r, rep, mcp) = run_vc(Behaviour::default(), &[Stage::Verify], vc, vc_config);
        assert_eq!(r.unwrap_err().exit_code(), 7);
        assert!(
            mcp.is_empty(),
            "the PLCs are not read after a failed controller"
        );
        assert_eq!(rep.controllers[0].verified, Some(false));
        assert!(!rep.controllers[0].checks[1].ok);
    }

    #[test]
    fn without_controllers_krl_is_skipped_in_up_and_refused_alone() {
        let (_, r, rep, _) = run_vc(
            Behaviour::default(),
            &[Stage::Deploy, Stage::Krl],
            MockRemote::default(),
            config,
        );
        r.unwrap();
        assert_eq!(rep.stages.len(), 1);
        let (_, r, _, _) = run_vc(
            Behaviour::default(),
            &[Stage::Krl],
            MockRemote::default(),
            config,
        );
        assert_eq!(r.unwrap_err().exit_code(), 3);
    }

    #[test]
    fn dry_run_plans_the_controller_calls() {
        let s = Server::start(Behaviour::default());
        let mut remote = plc_host();
        let mut tools = McpTools::new(config(&s.url).codesys_mcp).unwrap();
        let mut c = Cell::new(
            vc_config(&s.url),
            Some(Tree::Dir(example())),
            Settings {
                dry_run: true,
                ..quick()
            },
            &mut remote,
            &mut tools,
            None,
        )
        .unwrap();
        c.run(&[Stage::Krl, Stage::Verify]).unwrap();
        let ssm: Vec<&String> = c
            .report
            .calls
            .iter()
            .filter(|x| x.starts_with("ssm:i-0123456789abcdef0: "))
            .collect();
        assert_eq!(ssm.len(), 7, "{ssm:#?}");
        assert!(ssm[5].contains(" deploy "));
        assert!(ssm[6].contains(" verify "));
    }
}

/// As [`run`], with this PLC host and cell configuration.
fn run_on(
    b: Behaviour,
    stages: &[Stage],
    settings: Settings,
    remote: MockRemote,
    cfg: impl Fn(&str) -> CellConfig,
) -> (
    Server,
    MockRemote,
    plan2cell_cell::Result<()>,
    plan2cell_cell::Report,
) {
    run_after(b, stages, settings, remote, cfg, None)
}

/// As [`run_on`], on the report of an earlier run on the same build directory.
fn run_after(
    b: Behaviour,
    stages: &[Stage],
    settings: Settings,
    mut remote: MockRemote,
    cfg: impl Fn(&str) -> CellConfig,
    previous: Option<plan2cell_cell::Report>,
) -> (
    Server,
    MockRemote,
    plan2cell_cell::Result<()>,
    plan2cell_cell::Report,
) {
    let dir = b.tree.as_ref().map(|t| t.0.clone()).unwrap_or_else(example);
    let s = Server::start(b);
    let mut tools = McpTools::new(config(&s.url).codesys_mcp).unwrap();
    let mut c = Cell::new(
        cfg(&s.url),
        Some(Tree::Dir(dir)),
        settings,
        &mut remote,
        &mut tools,
        previous,
    )
    .unwrap();
    let r = c.run(stages);
    let report = c.report.clone();
    drop(c);
    (s, remote, r, report)
}

/// A PLC host whose runtimes all answer (container running, gateway and OPC UA ports open).
fn plc_host_up() -> MockRemote {
    plc_host().answer("if timeout 3 bash", Ok("open\n".into()))
}

mod symbols {
    use super::*;

    #[test]
    fn projects_logs_out_first_and_builds_the_symbol_configuration() {
        let (dir, m) = common::example_with_symbols();
        let b = Behaviour {
            tree: Some((dir.path().to_path_buf(), m)),
            ..Behaviour::default()
        };
        let (s, _, r, rep) = run_on(b, &[Stage::Projects], quick(), plc_host(), config);
        r.unwrap();
        let calls = s.calls();
        // Logged out first, head first; before any project is created.
        assert_eq!(
            calls[1..4],
            [
                format!("disconnect_from_device {HEAD}"),
                format!("disconnect_from_device {R2}"),
                format!("disconnect_from_device {R1}"),
            ]
        );
        let head: Vec<&str> = calls
            .iter()
            .filter(|c| c.ends_with(HEAD))
            .map(|c| c.split(' ').next().unwrap())
            .collect();
        let at = head
            .iter()
            .position(|c| *c == "create_symbol_config")
            .unwrap();
        assert_eq!(head[at - 1], "compile_project");
        assert_eq!(
            head[at..at + 5],
            [
                "create_symbol_config",
                "set_symbol_config_settings",
                "set_signature_access_bulk",
                "set_symbol_access",
                "list_configured_symbols"
            ]
        );
        assert_eq!(head[at + 5], "save_project");
        // Only the head has a symbol configuration.
        assert!(
            !calls
                .iter()
                .any(|c| c.starts_with("create_symbol_config Robot"))
        );
        let h = rep.projects.iter().find(|p| p.name == HEAD).unwrap();
        assert!(h.built && h.deviations.is_empty(), "{h:?}");
        // The optional PLAN_ID (a replaced constant) is a warning.
        assert!(
            h.warnings
                .iter()
                .any(|w| w.starts_with("symbol GVL_Cell.PLAN_ID ReadOnly not set: ")),
            "{:?}",
            h.warnings
        );
        assert!(rep.ok);
    }

    #[test]
    fn a_required_symbol_that_is_not_exported_is_a_deviation() {
        let (dir, m) = common::example_with_symbols();
        let b = Behaviour {
            tree: Some((dir.path().to_path_buf(), m)),
            lose_symbol: Some("PLC_PRG.udiCycles".into()),
            ..Behaviour::default()
        };
        let (_, _, r, rep) = run_on(b, &[Stage::Projects], quick(), plc_host(), config);
        assert_eq!(r.unwrap_err().exit_code(), 7);
        let h = rep.projects.iter().find(|p| p.name == HEAD).unwrap();
        assert_eq!(
            h.deviations,
            ["symbol PLC_PRG.udiCycles not exported readable"]
        );
    }

    #[test]
    fn library_errors_in_the_symbol_auto_compile_are_warnings_and_the_head_builds() {
        // Live, 2026-11-18 (demo_body_weld_2r): set_signature_access_bulk landed, its
        // auto-compile failed only on library-open errors; the head was "not built".
        let (dir, m) = common::example_with_symbols();
        let b = Behaviour {
            tree: Some((dir.path().to_path_buf(), m)),
            auto_compile_library_errors: true,
            ..Behaviour::default()
        };
        let (s, _, r, rep) = run_on(b, &[Stage::Projects], quick(), plc_host(), config);
        r.unwrap();
        let h = rep.projects.iter().find(|p| p.name == HEAD).unwrap();
        assert!(
            h.built && h.errors.is_empty() && h.deviations.is_empty(),
            "{h:?}"
        );
        for tool in [
            "create_symbol_config",
            "set_symbol_config_settings",
            "set_signature_access_bulk",
        ] {
            assert!(
                h.warnings.iter().any(|w| w.starts_with(&format!(
                    "{tool}: 3 library error(s) in its auto-compile (the login builds again): \
                     Could not open library '#CmpUserMgr Implementation'."
                ))),
                "{tool}: {:#?}",
                h.warnings
            );
        }
        // The read back and the save ran after them.
        let calls = s.calls();
        let bulk = calls
            .iter()
            .position(|c| *c == format!("set_signature_access_bulk {HEAD}"))
            .unwrap();
        assert!(calls[bulk..].contains(&format!("list_configured_symbols {HEAD}")));
        assert!(calls[bulk..].contains(&format!("save_project {HEAD}")));
        assert!(rep.ok);
    }

    #[test]
    fn a_real_error_in_the_symbol_auto_compile_is_still_not_built() {
        let (dir, m) = common::example_with_symbols();
        let b = Behaviour {
            tree: Some((dir.path().to_path_buf(), m)),
            auto_compile_library_errors: true,
            auto_compile_error: true,
            ..Behaviour::default()
        };
        let (_, _, r, rep) = run_on(b, &[Stage::Projects], quick(), plc_host(), config);
        let e = r.unwrap_err();
        assert_eq!(e.exit_code(), 8, "{e}");
        assert!(e.to_string().contains(&format!("not built: {HEAD}")), "{e}");
        let h = rep.projects.iter().find(|p| p.name == HEAD).unwrap();
        assert!(!h.built, "{h:?}");
        assert!(
            h.errors
                .iter()
                .any(|x| x.starts_with("codesys tool create_symbol_config: ")
                    && x.contains("C0077: Unknown type: 'ST_Ticket'")),
            "{:#?}",
            h.errors
        );
    }

    #[test]
    fn library_errors_in_the_projects_build_still_fail_it() {
        // The auto-compile tolerance is only for the symbol-config edits: the projects stage's
        // own build has no library refresh and second build (as start's BuildClean does), and a
        // library really missing reads the same, so its library-open errors stay errors.
        let b = Behaviour {
            library_errors_always: Some(HEAD.into()),
            ..Behaviour::default()
        };
        let (_, _, r, rep) = run_on(b, &[Stage::Projects], quick(), plc_host(), config);
        let e = r.unwrap_err();
        assert_eq!(e.exit_code(), 8, "{e}");
        assert!(e.to_string().contains(&format!("not built: {HEAD}")), "{e}");
        let h = rep.projects.iter().find(|p| p.name == HEAD).unwrap();
        assert!(!h.built, "{h:?}");
        assert!(
            h.errors
                .iter()
                .any(|x| x.contains("Could not open library '#SysTask'")),
            "{:#?}",
            h.errors
        );
        assert!(
            !h.warnings.iter().any(|w| w.contains("library error(s)")),
            "{:#?}",
            h.warnings
        );
    }

    #[test]
    fn verify_checks_the_symbols_and_the_opc_ua_port() {
        let (dir, m) = common::example_with_symbols();
        let b = Behaviour {
            tree: Some((dir.path().to_path_buf(), m)),
            ..Behaviour::default()
        };
        let (s, remote, r, rep) = run_on(
            b,
            &[Stage::Projects, Stage::Verify],
            quick(),
            plc_host_up(),
            config,
        );
        r.unwrap();
        let h = rep.projects.iter().find(|p| p.name == HEAD).unwrap();
        let sym: Vec<&str> = h
            .checks
            .iter()
            .filter(|c| c.expr.starts_with("symbol "))
            .map(|c| c.expr.as_str())
            .collect();
        assert_eq!(sym.len(), 6);
        assert!(sym.contains(&"symbol PLC_PRG.udiCycles"));
        let port = h.checks.last().unwrap();
        assert_eq!(port.expr, "opc.tcp://192.168.100.10:4840");
        assert!(port.ok && h.checks.iter().all(|c| c.ok), "{:?}", h.checks);
        assert!(remote.calls.contains(&Call::Run(
            "if timeout 3 bash -c 'exec 3<>/dev/tcp/192.168.100.10/4840' 2>/dev/null; then echo \
             open; else echo closed; fi"
                .into()
        )));
        // The robot simulators have none.
        let r1 = rep.projects.iter().find(|p| p.name == R1).unwrap();
        assert!(!r1.checks.iter().any(|c| c.expr.starts_with("symbol ")));
        let head: Vec<String> = s
            .calls()
            .into_iter()
            .filter(|c| c.ends_with(HEAD))
            .collect();
        assert_eq!(
            head[head.len() - 2..],
            [
                format!("list_configured_symbols {HEAD}"),
                format!("disconnect_from_device {HEAD}")
            ]
        );
    }

    #[test]
    fn a_closed_opc_ua_port_fails_verify() {
        let (dir, m) = common::example_with_symbols();
        let b = Behaviour {
            tree: Some((dir.path().to_path_buf(), m)),
            ..Behaviour::default()
        };
        let remote = plc_host().answer("if timeout 3 bash", Ok("closed\n".into()));
        let (_, _, r, rep) = run_on(
            b,
            &[Stage::Projects, Stage::Verify],
            quick(),
            remote,
            config,
        );
        assert_eq!(r.unwrap_err().exit_code(), 7);
        let h = rep.projects.iter().find(|p| p.name == HEAD).unwrap();
        let bad: Vec<&str> = h
            .checks
            .iter()
            .filter(|c| !c.ok)
            .map(|c| c.expr.as_str())
            .collect();
        assert_eq!(bad, ["opc.tcp://192.168.100.10:4840"]);
    }
}

mod production {
    use super::*;

    fn pulses(s: &Server) -> Vec<String> {
        s.state
            .lock()
            .unwrap()
            .writes
            .iter()
            .map(|(x, _)| x.clone())
            .collect()
    }

    #[test]
    fn start_resets_and_confirms_the_head_into_production_then_logs_out() {
        let b = Behaviour {
            cell_states: vec![
                "CS_BRINGUP",
                "CS_ERROR",
                "CS_ERROR",
                "CS_RECOVERY",
                "CS_HOMING_RUN",
                "CS_RUNNING",
            ],
            ..Behaviour::default()
        };
        let (s, remote, r, rep) = run_on(b, &[Stage::Start], quick(), plc_host_up(), config);
        r.unwrap();
        assert_eq!(
            pulses(&s),
            [
                "PLC_PRG.bReset",
                "PLC_PRG.bReset",
                "PLC_PRG.bRecoveryConfirm"
            ]
        );
        let calls = s.calls();
        assert_eq!(
            calls.last().unwrap(),
            &format!("disconnect_from_device {HEAD}")
        );
        assert!(calls.contains(&format!("connect_to_device {HEAD}")));
        // Only the head is logged in to.
        assert!(
            !calls
                .iter()
                .any(|c| c.starts_with("connect_to_device Robot"))
        );
        // Every runtime was probed and none restarted.
        let runs: Vec<String> = remote
            .calls
            .iter()
            .filter_map(|c| match c {
                Call::Run(x) => Some(x.clone()),
                _ => None,
            })
            .collect();
        for ip in [
            "192.168.100.10/11740",
            "192.168.100.11/11741",
            "192.168.100.12/11741",
        ] {
            assert!(runs.iter().any(|x| x.contains(ip)), "{ip}: {runs:#?}");
        }
        assert!(!runs.iter().any(|x| x.contains("docker restart")));
        let st = &rep.stages[0];
        assert_eq!(st.stage, Stage::Start);
        assert!(st.ok);
        assert!(
            st.notes
                .iter()
                .any(|n| n
                    == "CS_ERROR ERR_START_TIMEOUT robot 2, udiCycles 3: PLC_PRG.bReset pulsed"),
            "{:?}",
            st.notes
        );
        assert!(
            st.notes.last().unwrap().starts_with(
                "in production after 6 read(s) and 3 pulse(s): CS_RUNNING, udiCycles 3"
            ),
            "{:?}",
            st.notes
        );
    }

    /// The report collects the stages across runs, sorted projects .. start, stop. An earlier
    /// stop's `CS_ERROR ERR_OPERATOR_STOP` must not stay after a later start (it read as the
    /// head's state after Start production), nor an earlier start's `CS_RUNNING` after a stop.
    #[test]
    fn start_and_stop_each_replace_the_others_entry_in_the_report() {
        let stage = |st: Stage, note: &str| plan2cell_cell::stages::StageReport {
            stage: st,
            ok: true,
            notes: vec![note.into()],
            error: None,
        };
        let previous = plan2cell_cell::Report {
            plan_id: Some("16#F77E".into()),
            ok: true,
            stages: vec![
                stage(Stage::Deploy, "deployed"),
                stage(Stage::Start, "in production after 1 read(s): CS_RUNNING"),
                stage(
                    Stage::Stop,
                    "CS_ERROR ERR_OPERATOR_STOP robot INT#0, udiCycles UDINT#10",
                ),
            ],
            ..Default::default()
        };
        let behaviour = |state: &'static str| Behaviour {
            cell_states: vec![state],
            ..Behaviour::default()
        };
        let kinds =
            |r: &plan2cell_cell::Report| r.stages.iter().map(|s| s.stage).collect::<Vec<_>>();

        let (_, _, r, rep) = run_after(
            behaviour("CS_RUNNING"),
            &[Stage::Start],
            quick(),
            plc_host_up(),
            config,
            Some(previous.clone()),
        );
        r.unwrap();
        assert_eq!(kinds(&rep), [Stage::Deploy, Stage::Start], "{rep:#?}");
        assert!(rep.ok);
        assert!(
            !rep.stages
                .iter()
                .flat_map(|s| &s.notes)
                .any(|n| n.contains("CS_ERROR")),
            "the earlier stop's state is gone: {rep:#?}"
        );

        let (_, _, r, rep) = run_after(
            behaviour("CS_RUNNING"),
            &[Stage::Stop],
            quick(),
            plc_host(),
            config,
            Some(rep),
        );
        r.unwrap();
        assert_eq!(kinds(&rep), [Stage::Deploy, Stage::Stop], "{rep:#?}");

        // A failed start keeps the stop: it is still the last state the head was put in.
        let (_, _, r, rep) = run_after(
            behaviour("CS_READY"),
            &[Stage::Start],
            quick(),
            plc_host_up(),
            config,
            Some(previous),
        );
        assert!(r.is_err());
        assert_eq!(
            kinds(&rep),
            [Stage::Deploy, Stage::Start, Stage::Stop],
            "{rep:#?}"
        );
        assert!(!rep.ok);
    }

    #[test]
    fn start_restarts_a_runtime_that_does_not_answer() {
        let b = Behaviour {
            cell_states: vec!["CS_RECOVERY", "CS_RUNNING"],
            ..Behaviour::default()
        };
        let remote = plc_host()
            .answer(
                "if timeout 3 bash -c 'exec 3<>/dev/tcp/192.168.100.10/11740'",
                Ok("closed\n".into()),
            )
            .answer("if timeout 3 bash", Ok("open\n".into()))
            .answer("for i in $(seq", Ok("open\n".into()));
        let (s, remote, r, rep) = run_on(b, &[Stage::Start], quick(), remote, config);
        r.unwrap();
        assert!(
            remote
                .calls
                .contains(&Call::Run("sudo docker restart codesys-master".into()))
        );
        assert!(remote.calls.contains(&Call::Run(
            "for i in $(seq 1 2); do if timeout 2 bash -c 'exec 3<>/dev/tcp/192.168.100.10/11740' \
             2>/dev/null; then echo open; exit 0; fi; sleep 2; done; echo closed"
                .into()
        )));
        assert_eq!(pulses(&s), ["PLC_PRG.bRecoveryConfirm"]);
        assert!(
            rep.stages[0]
                .notes
                .iter()
                .any(|n| n == "restarted codesys-master (it did not answer)"),
            "{:?}",
            rep.stages[0].notes
        );
    }

    #[test]
    fn a_failed_login_restarts_the_head_runtime_once() {
        let b = Behaviour {
            login_down: 1,
            ..Behaviour::default()
        };
        let remote = plc_host_up().answer("for i in $(seq", Ok("open\n".into()));
        let (s, remote, r, _) = run_on(b, &[Stage::Start], quick(), remote, config);
        r.unwrap();
        assert!(
            remote
                .calls
                .contains(&Call::Run("sudo docker restart codesys-master".into()))
        );
        let connects = s
            .calls()
            .iter()
            .filter(|c| c.starts_with("connect_to_device"))
            .count();
        assert_eq!(connects, 2);
        assert!(pulses(&s).is_empty(), "already running");
    }

    #[test]
    fn a_cell_that_stays_in_error_fails_after_the_pulses_and_still_logs_out() {
        let b = Behaviour {
            cell_states: vec!["CS_ERROR", "CS_RUNNING"],
            stuck_cell: true,
            ..Behaviour::default()
        };
        let settings = Settings {
            start_pulses: 3,
            ..quick()
        };
        let (s, _, r, rep) = run_on(b, &[Stage::Start], settings, plc_host_up(), config);
        let e = r.unwrap_err();
        assert_eq!(e.exit_code(), 7);
        assert!(
            e.to_string()
                .contains("not in production after 3 pulse(s): CS_ERROR ERR_START_TIMEOUT robot 2"),
            "{e}"
        );
        assert_eq!(pulses(&s).len(), 3);
        assert_eq!(
            s.calls().last().unwrap(),
            &format!("disconnect_from_device {HEAD}")
        );
        assert!(!rep.ok);
    }

    #[test]
    fn a_dialog_nobody_answers_fails_start_at_once_without_retries() {
        // Live 2026-10-13: a CODESYS message box on the IDE made every call time out (180 s each).
        let b = Behaviour {
            cell_states: vec!["CS_RUNNING"],
            dialog: Some("CODESYS: Do you want to download the application? [Yes, No]"),
            ..Behaviour::default()
        };
        let (s, remote, r, rep) = run_on(b, &[Stage::Start], quick(), plc_host_up(), config);
        let e = r.unwrap_err();
        assert_eq!(e.exit_code(), 3);
        let msg = e.to_string();
        assert!(
            msg.starts_with(
                "CODESYS shows a dialog: CODESYS: Do you want to download the application? \
                 [Yes, No]. The MCP bridge answers only known dialogs"
            ) && msg.ends_with("(codesys tool download_missing_libraries)"),
            "{msg}"
        );
        assert_eq!(rep.stages[0].error.as_deref(), Some(msg.as_str()));
        // No runtime restart, no second login attempt.
        assert!(
            !remote
                .calls
                .iter()
                .any(|c| matches!(c, Call::Run(x) if x.contains("docker restart")))
        );
        // The first call that meets the dialog ends start: the libraries come before the bind.
        let tried: Vec<String> = s
            .calls()
            .into_iter()
            .filter(|c| {
                c.starts_with("download_missing_libraries")
                    || c.starts_with("rebind_device_to_scan_result")
            })
            .collect();
        assert_eq!(tried, [format!("download_missing_libraries {HEAD}")]);
    }

    #[test]
    fn a_rebind_the_bridge_answered_a_dialog_for_is_retried() {
        let b = Behaviour {
            cell_states: vec!["CS_RUNNING"],
            rebind_dialogs: 1,
            ..Behaviour::default()
        };
        let remote = plc_host_up().answer("for i in $(seq", Ok("open\n".into()));
        let (_, _, r, rep) = run_on(b, &[Stage::Start], quick(), remote, config);
        r.unwrap();
        assert!(
            rep.stages[0]
                .notes
                .iter()
                .any(|n| n == "rebind_device_to_scan_result failed: restarting codesys-master"),
            "{:?}",
            rep.stages[0].notes
        );
    }

    #[test]
    fn start_refuses_a_head_that_runs_another_plan() {
        // Live 2026-10-12: the newest export (KRL 16#61BD) started against a head deployed from
        // an older one (16#9201).
        let b = Behaviour {
            cell_states: vec!["CS_ERROR", "CS_RUNNING"],
            head_plan: Some("16#9201"),
            ..Behaviour::default()
        };
        let (s, _, r, rep) = run_on(b, &[Stage::Start], quick(), plc_host_up(), config);
        let e = r.unwrap_err();
        assert_eq!(e.exit_code(), 7);
        let msg = e.to_string();
        assert!(
            msg.contains("the head runs PLAN_ID 16#9201 (")
                && msg.contains("this export is plan 16#F77E"),
            "{msg}"
        );
        assert!(pulses(&s).is_empty(), "{:?}", pulses(&s));
        assert_eq!(
            s.calls().last().unwrap(),
            &format!("disconnect_from_device {HEAD}")
        );
        assert!(!rep.stages[0].ok);
        // The same plan: a note, then production.
        let b = Behaviour {
            cell_states: vec!["CS_RUNNING"],
            ..Behaviour::default()
        };
        let (_, _, r, rep) = run_on(b, &[Stage::Start], quick(), plc_host_up(), config);
        r.unwrap();
        assert!(
            rep.stages[0]
                .notes
                .iter()
                .any(|n| n == "head PLAN_ID 16#F77E = this export"),
            "{:?}",
            rep.stages[0].notes
        );
    }

    #[test]
    fn start_never_pulses_bstart_a_ready_cell_without_auto_run_fails() {
        let b = Behaviour {
            cell_states: vec!["CS_READY"],
            ..Behaviour::default()
        };
        let (s, _, r, _) = run_on(b, &[Stage::Start], quick(), plc_host_up(), config);
        let e = r.unwrap_err();
        assert_eq!(e.exit_code(), 7);
        assert!(e.to_string().contains("bAutoRun"), "{e}");
        assert!(pulses(&s).is_empty(), "{:?}", pulses(&s));
        assert_eq!(
            s.calls().last().unwrap(),
            &format!("disconnect_from_device {HEAD}")
        );
    }

    #[test]
    fn stop_pulses_bstop_and_logs_out() {
        let b = Behaviour {
            cell_states: vec!["CS_RUNNING"],
            ..Behaviour::default()
        };
        let (s, _, r, rep) = run_on(b, &[Stage::Stop], quick(), plc_host(), config);
        r.unwrap();
        assert_eq!(pulses(&s), ["PLC_PRG.bStop"]);
        assert_eq!(
            s.calls().last().unwrap(),
            &format!("disconnect_from_device {HEAD}")
        );
        assert_eq!(rep.stages[0].stage, Stage::Stop);
    }

    #[test]
    fn a_dry_run_start_plans_every_step_and_sends_nothing() {
        let settings = Settings {
            dry_run: true,
            ..quick()
        };
        let (s, remote, r, rep) = run_on(
            Behaviour::default(),
            &[Stage::Start],
            settings,
            plc_host_up(),
            config,
        );
        r.unwrap();
        assert!(remote.calls.is_empty());
        assert!(s.calls().is_empty(), "{:?}", s.calls());
        let plan = rep.calls.join("\n");
        for x in [
            "ssh plc-host: sudo docker ps",
            "connect_to_device(",
            "read_variables(",
            "disconnect_from_device(",
        ] {
            assert!(plan.contains(x), "{x}: {plan}");
        }
    }
}

mod libraries {
    use super::*;

    fn at(calls: &[String], c: &str) -> usize {
        calls
            .iter()
            .position(|x| x == c)
            .unwrap_or_else(|| panic!("{c}: {calls:#?}"))
    }

    #[test]
    fn projects_store_the_libraries_with_the_projects_and_on_the_workspace() {
        let dir = common::copy_example();
        let b = Behaviour {
            libraries: true,
            tree: Some((dir.path().to_path_buf(), manifest())),
            ..Behaviour::default()
        };
        let (s, _, r, rep) = run_on(b, &[Stage::Projects], quick(), plc_host(), config);
        r.unwrap();
        let calls = s.calls();
        for p in [R1, R2, HEAD] {
            assert!(
                at(&calls, &format!("save_project {p}"))
                    < at(&calls, &format!("export_project_libraries {p}"))
            );
        }
        // Every export asks for the files.
        assert_eq!(s.state.lock().unwrap().include_data, [true, true, true]);
        // The workspace copy: every project's list merged, the files the answers carried.
        let libs = dir.path().join("plc/libraries");
        let list: Vec<Value> =
            serde_json::from_str(&std::fs::read_to_string(libs.join("libraries.json")).unwrap())
                .unwrap();
        let names: Vec<&str> = list.iter().map(|l| l["name"].as_str().unwrap()).collect();
        assert_eq!(
            names.iter().filter(|n| **n == "Standard").count(),
            1,
            "{names:?}"
        );
        assert!(names.contains(&"BigLib"), "{names:?}");
        assert!(
            list.iter()
                .all(|l| l.get("data").is_none() && l.get("dataOmitted").is_none())
        );
        assert_eq!(
            std::fs::read_to_string(libs.join("Standard_3.5.17.0_System.compiled-library"))
                .unwrap(),
            "library Standard"
        );
        assert!(!libs.join("BigLib_1.0.0.0_Acme.compiled-library").exists());
        let notes = &rep.stages[0].notes;
        assert!(
            notes.iter().any(|n| n.contains(
                "not in the answer (still only in C:\\ProgramData\\plan2cell\\projects\\welding_cell_2_small\\libraries on the IDE): BigLib_1.0.0.0_Acme.compiled-library (20000000 bytes, over the answer's budget of 16777216 bytes)"
            )),
            "{notes:#?}"
        );
        assert!(
            notes.iter().any(|n| n
                == &format!(
                    "{HEAD}: libraries stored in C:\\ProgramData\\plan2cell\\projects\\welding_cell_2_small\\libraries"
                )),
            "{notes:#?}"
        );
    }

    #[test]
    fn a_project_with_build_errors_stores_no_libraries() {
        let dir = common::copy_example();
        let b = Behaviour {
            libraries: true,
            fail_build: Some(HEAD.into()),
            tree: Some((dir.path().to_path_buf(), manifest())),
            ..Behaviour::default()
        };
        let (s, _, r, rep) = run_on(b, &[Stage::Projects], quick(), plc_host(), config);
        assert_eq!(r.unwrap_err().exit_code(), 8);
        let calls = s.calls();
        assert!(calls.contains(&format!("export_project_libraries {R1}")));
        assert!(!calls.contains(&format!("export_project_libraries {HEAD}")));
        let head = rep.projects.iter().find(|p| p.name == HEAD).unwrap();
        assert!(
            head.warnings
                .iter()
                .any(|w| w == "libraries not stored with the project: the build has 1 error(s)"),
            "{head:?}"
        );
    }

    #[test]
    fn start_installs_the_stored_libraries_and_builds_before_the_login() {
        let b = Behaviour {
            libraries: true,
            cell_states: vec!["CS_RUNNING"],
            ..Behaviour::default()
        };
        let (s, _, r, rep) = run_on(b, &[Stage::Start], quick(), plc_host_up(), config);
        r.unwrap();
        let calls = s.calls();
        let order = [
            "install_libraries".to_string(),
            format!("list_project_libraries {HEAD}"),
            format!("compile_project {HEAD}"),
            format!("rebind_device_to_scan_result {HEAD}"),
            format!("connect_to_device {HEAD}"),
        ];
        let idx: Vec<usize> = order.iter().map(|c| at(&calls, c)).collect();
        assert!(idx.windows(2).all(|w| w[0] < w[1]), "{calls:#?}");
        assert!(
            !calls
                .iter()
                .any(|c| c.starts_with("download_missing_libraries"))
        );
        let notes = &rep.stages[0].notes;
        assert!(
            notes.iter().any(|n| n
                == "install_libraries: Libraries of C:\\ProgramData\\plan2cell\\projects\\welding_cell_2_small\\libraries: 1 installed, 1 already installed, 0 error(s) (repository System)."),
            "{notes:#?}"
        );
        assert!(
            notes
                .iter()
                .any(|n| n == &format!("{HEAD} builds (1 warning(s))")),
            "{notes:#?}"
        );
    }

    #[test]
    fn an_archive_without_libraries_json_downloads_the_libraries_instead() {
        let b = Behaviour {
            libraries: true,
            no_libraries_json: true,
            cell_states: vec!["CS_RUNNING"],
            ..Behaviour::default()
        };
        let (s, _, r, rep) = run_on(b, &[Stage::Start], quick(), plc_host_up(), config);
        r.unwrap();
        let calls = s.calls();
        assert!(
            at(&calls, "install_libraries")
                < at(&calls, &format!("download_missing_libraries {HEAD}"))
                && at(&calls, &format!("download_missing_libraries {HEAD}"))
                    < at(&calls, &format!("list_project_libraries {HEAD}")),
            "{calls:#?}"
        );
        assert!(
            rep.stages[0].notes.iter().any(|n| n
                == "no libraries.json with the project (an older archive): download_missing_libraries"),
            "{:#?}",
            rep.stages[0].notes
        );
    }

    fn restarts(remote: &MockRemote) -> usize {
        remote
            .calls
            .iter()
            .filter(|c| matches!(c, Call::Run(x) if x.contains("docker restart")))
            .count()
    }

    #[test]
    fn library_errors_on_starts_first_build_are_built_again_and_start_goes_on() {
        let b = Behaviour {
            library_errors_once: Some(HEAD.into()),
            ..Behaviour::default()
        };
        let (s, _, r, rep) = run_on(b, &[Stage::Start], quick(), plc_host_up(), config);
        r.unwrap();
        let calls = s.calls();
        let builds = calls.iter().filter(|c| *c == &format!("compile_project {HEAD}")).count();
        assert!(builds >= 2, "{calls:#?}");
        // Refreshed between the builds: download_missing_libraries, the listing, then the build again.
        let first = at(&calls, &format!("compile_project {HEAD}"));
        let after: Vec<&String> = calls[first + 1..].iter().take_while(|c| !c.starts_with("compile_project")).collect();
        assert_eq!(
            after,
            [&format!("download_missing_libraries {HEAD}"), &format!("list_project_libraries {HEAD}")],
            "{calls:#?}"
        );
        assert!(
            rep.stages[0].notes.iter().any(|n| n.contains("library error(s) on the first build after the open; libraries refreshed")),
            "{:#?}",
            rep.stages[0].notes
        );
    }

    #[test]
    fn library_errors_that_stay_before_the_login_are_a_warning_and_the_login_builds() {
        let b = Behaviour {
            library_errors_always: Some(HEAD.into()),
            cell_states: vec!["CS_RUNNING"],
            ..Behaviour::default()
        };
        let (s, _, r, rep) = run_on(b, &[Stage::Start], quick(), plc_host_up(), config);
        r.unwrap();
        let calls = s.calls();
        assert!(calls.iter().any(|c| c.starts_with("connect_to_device")), "{calls:#?}");
        assert!(
            rep.stages[0].notes.iter().any(|n| n.contains("library error(s) before the login (the login builds again)")),
            "{:#?}",
            rep.stages[0].notes
        );
    }

    #[test]
    fn a_compile_error_in_start_is_exit_8_without_a_login_or_runtime_restart() {
        let b = Behaviour {
            fail_build: Some(HEAD.into()),
            ..Behaviour::default()
        };
        let remote = plc_host_up().answer("for i in $(seq", Ok("open\n".into()));
        let (s, remote, r, _) = run_on(b, &[Stage::Start], quick(), remote, config);
        let e = r.unwrap_err();
        assert_eq!(e.exit_code(), 8, "{e}");
        assert!(
            e.to_string().contains(&format!(
                "{HEAD}: 1 build error(s) before the login: C0077: Unknown type: 'ST_Ticket'"
            )),
            "{e}"
        );
        let calls = s.calls();
        assert!(
            !calls.iter().any(|c| c.starts_with("connect_to_device")),
            "{calls:#?}"
        );
        assert_eq!(restarts(&remote), 0);
        // Logged out all the same.
        assert_eq!(
            calls.last().unwrap(),
            &format!("disconnect_from_device {HEAD}")
        );
    }

    #[test]
    fn a_build_failure_at_login_is_exit_8_at_once_without_a_runtime_restart() {
        let b = Behaviour {
            login_build_error: true,
            ..Behaviour::default()
        };
        let remote = plc_host_up().answer("for i in $(seq", Ok("open\n".into()));
        let (s, remote, r, _) = run_on(b, &[Stage::Start], quick(), remote, config);
        let e = r.unwrap_err();
        assert_eq!(e.exit_code(), 8, "{e}");
        assert!(e.to_string().contains("the head does not build"), "{e}");
        assert!(e.to_string().contains("Compile errors occurred"), "{e}");
        let connects = s
            .calls()
            .iter()
            .filter(|c| c.starts_with("connect_to_device"))
            .count();
        assert_eq!(connects, 1);
        assert_eq!(restarts(&remote), 0);
    }
}

mod reset_ide {
    use super::*;

    #[test]
    fn reset_ide_logs_out_closes_and_restarts_codesys() {
        let (s, _, r, rep) = run(Behaviour::default(), &[Stage::ResetIde], true, quick());
        r.unwrap();
        let calls = s.calls();
        let n = calls.len();
        assert_eq!(
            calls[n - 3..],
            ["shutdown_codesys", "launch_codesys", "get_codesys_status"],
            "{calls:#?}"
        );
        // Every project logged out and closed without saving first, head first.
        let first = |c: String| calls.iter().position(|x| *x == c).unwrap();
        for p in [HEAD, R2, R1] {
            assert!(first(format!("close_project {p}")) < n - 3, "{calls:#?}");
        }
        assert!(
            first(format!("disconnect_from_device {HEAD}"))
                < first(format!("disconnect_from_device {R1}"))
        );
        let notes = &rep.stages[0].notes;
        assert!(
            notes.iter().any(|n| n == "get_codesys_status: running"),
            "{notes:#?}"
        );
        assert!(rep.stages[0].ok);
    }

    #[test]
    fn a_codesys_that_does_not_come_back_ready_is_exit_8() {
        let b = Behaviour {
            codesys_state: Some("error"),
            ..Behaviour::default()
        };
        let (s, _, r, _) = run(b, &[Stage::ResetIde], false, quick());
        let e = r.unwrap_err();
        assert_eq!(e.exit_code(), 8, "{e}");
        assert!(
            e.to_string()
                .contains("CODESYS not ready after the restart (get_codesys_status: error)"),
            "{e}"
        );
        // Without a build directory: no project to log out of.
        assert!(
            !s.calls()
                .iter()
                .any(|c| c.starts_with("disconnect_from_device"))
        );
    }
}

mod kvc_parallel {
    use super::*;
    use plan2cell_cell::config::{Access, KukaController, KukaVc};
    use plan2cell_cell::remote::{Pending, Remote};
    use std::cell::RefCell;
    use std::rc::Rc;

    type Log = Rc<RefCell<Vec<String>>>;

    /// A KUKA VC host that logs when a command is sent and when it is waited for.
    struct Host {
        name: &'static str,
        log: Log,
        inner: MockRemote,
    }

    impl Remote for Host {
        fn run(&mut self, cmd: &str) -> plan2cell_cell::Result<String> {
            self.log
                .borrow_mut()
                .push(format!("run {}: {cmd}", self.name));
            self.inner.run(cmd)
        }
        fn copy(&mut self, local: &std::path::Path, remote: &str) -> plan2cell_cell::Result<()> {
            self.inner.copy(local, remote)
        }
        fn start(&mut self, cmd: &str) -> plan2cell_cell::Result<Pending> {
            self.log
                .borrow_mut()
                .push(format!("start {}: {cmd}", self.name));
            Ok(Pending::Done(self.inner.run(cmd)))
        }
        fn wait(&mut self, p: Pending) -> plan2cell_cell::Result<String> {
            self.log.borrow_mut().push(format!("wait {}", self.name));
            match p {
                Pending::Done(r) => r,
                _ => unreachable!(),
            }
        }
    }

    const PROBE: &str =
        "sudo -n test -x /var/lib/plan2cell/kuka-vc/kvc-ack.sh && echo present || echo missing";
    fn ack(ns: &str) -> String {
        format!("sudo -n /var/lib/plan2cell/kuka-vc/kvc-ack.sh --netns {ns} 2>&1; echo p2c-exit=$?")
    }

    fn controller(robot: &str, netns: &str, ssm: &str) -> KukaController {
        KukaController {
            driver: Default::default(),
            robot_ns: robot.into(),
            instance: "iiQKA.OS2_KSS_9.2.0_vass".into(),
            netns: netns.into(),
            kli_ip: None,
            station: None,
            ssm_instance: Some(ssm.into()),
            kubectl_pod: None,
            ssh_host: None,
            sps_base: None,
        }
    }

    #[test]
    fn the_kvc_ack_runs_of_all_controllers_are_sent_before_any_is_waited_for() {
        let cfg = |url: &str| {
            let mut c = config(url);
            c.kuka_vc = Some(KukaVc {
                controllers: vec![
                    controller("robot1", "krc1", "i-aaaa"),
                    controller("robot2", "krc2", "i-bbbb"),
                ],
                ..KukaVc::default()
            });
            c
        };
        let log: Log = Rc::default();
        let host = |name: &'static str, ns: &str| {
            Host {
            name,
            log: log.clone(),
            inner: MockRemote::default()
                .answer(PROBE, Ok("present\n".into()))
                .answer(
                    &ack(ns),
                    Ok(format!("ok: acknowledged in T1, no stop message left; controller {} in EXT\np2c-exit=0\n", &ns[3..])),
                ),
        }
        };
        let mut map: HashMap<Access, Host> = HashMap::new();
        map.insert(Access::Ssm("i-aaaa".into()), host("a", "krc1"));
        map.insert(Access::Ssm("i-bbbb".into()), host("b", "krc2"));
        let b = Behaviour {
            cell_states: vec!["CS_ERROR", "CS_RUNNING"],
            ..Behaviour::default()
        };
        let s = Server::start(b);
        let mut remote = plc_host_up();
        let mut tools = McpTools::new(config(&s.url).codesys_mcp).unwrap();
        let mut c = Cell::new(
            cfg(&s.url),
            Some(Tree::Dir(example())),
            quick(),
            &mut remote,
            &mut tools,
            None,
        )
        .unwrap()
        .with_controllers(Box::new(map));
        let r = c.run(&[Stage::Start]);
        let rep = c.report.clone();
        drop(c);
        r.unwrap();
        assert_eq!(
            *log.borrow(),
            [
                format!("start a: {PROBE}"),
                format!("start b: {PROBE}"),
                "wait a".to_string(),
                "wait b".to_string(),
                format!("start a: {}", ack("krc1")),
                format!("start b: {}", ack("krc2")),
                "wait a".to_string(),
                "wait b".to_string(),
            ]
        );
        let notes = &rep.stages[0].notes;
        for (who, n) in [("robot1 (krc1)", 1), ("robot2 (krc2)", 2)] {
            let want = format!(
                "{who}: kvc-ack.sh: ok: acknowledged in T1, no stop message left; controller {n} in EXT"
            );
            assert!(notes.contains(&want), "{want}: {notes:#?}");
        }
    }
}
