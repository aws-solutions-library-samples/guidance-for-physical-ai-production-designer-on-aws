//! The CODESYS side as typed tool calls of Codesys-MCP-Master 0.19.0 (no generic script tool),
//! planned from `project.json` and the `codesys/` sources. Two arguments come from our overlay
//! of the server (`infra/windows/codesys-mcp-overlay`); [`Caps`] says whether the server has
//! them, and without them the plan falls back to what the plain 0.19.0 can do.
//!
//! A plan is a list of [`Call`]s. A call carries everything the stage needs to run and check
//! it: the tool and its arguments, arguments known only at run time ([`Deferred`]: a device's
//! display name from the device repository, the index of a struct member), a condition
//! ([`When`]: a task that may already exist) and how its answer is read back ([`Check`]).
//! A dry run prints the plan as it is.
//!
//! | `project.json` | tools |
//! |---|---|
//! | (every device) | `find_device_description` -> display name for `create_project` / `add_device` |
//! | `create_project` | `create_project(filePath, deviceName)`, `get_device_identification` |
//! | `add_device` | `add_device(parentPath, deviceName, targetDeviceName, targetVersion)`, `get_device_identification` |
//! | `set_device_parameters` | `set_device_parameter(parameterId, value)`; a struct member with [`Caps::element_path`]: `set_device_parameter(.., elementPath)` at any depth (`Identification.StationName`), read back `get_device_parameter(.., elementPath=<the struct above it>)`; without: `get_device_parameter` (member -> `elementIndex` by its visible name, `StationName` = `Station name`), `set_device_parameter(.., elementIndex)`, and a member of a nested struct is a human step (one level only); scalars read back with `list_device_parameters` |
//! | `ensure_module` | the `project-structure` resource (the device's children), then `plug_device(parentPath, slotIndex, name, <parent's DeviceID>, moduleId)` into an `<Empty>` slot or `rename_object` of the module the device came with; `list_device_parameters` (its ident). No slot at all (a free child list): `add_device(parentPath, name, <parent's name>, moduleId)` with [`Caps::module_id`], else a human step, see [`Module`] |
//! | `ensure_empty_slot` | the `project-structure` resource, then `unplug_device(devicePath)` of a module the device came with in that slot (FDIO64 on the KUKA DAP `DIM 3`), and the structure read again: the slot must be `<Empty>`, see [`EmptySlot`] |
//! | `ensure_libraries` | `add_library` (mode `ensure`), `download_missing_libraries`, `list_project_libraries` (the `auto` ones come with the devices) |
//! | `create_objects` | DUT: `create_dut` + `set_pou_code`; GVL: `create_gvl(declarationCode)`; POU: `create_pou(declarationCode, implementationCode)`, `set_pou_code` for `replace_existing` (PLC_PRG of `create_project`); read back `get_all_pou_code` |
//! | `configure_tasks` | `list_tasks`, `create_task` (if missing), `configure_task`, `add_pou_to_task` (programs, if missing), `set_plc_settings(busCycleTask)` + `get_plc_settings`, `list_tasks` |
//! | `map_io` | see [`IoCsv`]: `create_gvl` (CSV as text) + `mirror_export` + `import_io_mappings_csv` + `delete_object`, or only `import_io_mappings_csv` of a staged file |
//! | `build` | `compile_project` (its ERROR/WARNING lines) |
//! | `symbol_config` | `create_symbol_config(supportOpcUa)`, `set_symbol_config_settings(contentFeatureFlags=[SupportOPCUA])`, per `symbols[]` `set_signature_access_bulk` (a whole signature) or `set_symbol_access` (one variable; `optional`: a failure is a warning), read back `list_configured_symbols` (`require[]` exported readable) |
//! | `save` | `save_project`; after a `build` without errors, with [`Caps::export_libraries`]: `export_project_libraries(outDir=<dir>\libraries)` (every resolved library with its dependencies, and `libraries.json`), also copied next to the build's `plc/` on the workspace when the answer carries the files ([`exported_libraries`]) |
//! | deploy | first `close_project(saveFirst=false)` of the head (edits made in the PLC IDE discarded, best effort; `projects` does it too after its logout, `start` and `stop` first), then per project `rebind_device_to_scan_result(matchAddress ip:port)`, `save_project`, `connect_to_device` (first login: `add_device_user`, again), `download_to_device`, `create_boot_application(online)`, `start_stop_application(start)`, `get_application_state`, `disconnect_from_device`; after every project, `open_project` of the head (best effort: the project the PLC IDE shows) |
//! | verify | `rebind_device_to_scan_result` + `save_project` (as deploy), `connect_to_device`, `get_application_state`, `read_variables`, (`symbol_config`: `list_configured_symbols`), `disconnect_from_device` |
//! | start | `close_project(saveFirst=false)` of the head, then [`libraries_ready`] (`install_libraries(<dir>\libraries)` with [`Caps::install_libraries`], `download_missing_libraries` without it or for an archive without `libraries.json`, `list_project_libraries`, `compile_project`: an error stops `start`, exit 8), then as verify up to `get_application_state`, `start_stop_application(start)` if it is not running, `read_variables(GVL_Cell.PLAN_ID)` (= the export's plan, else exit 7), `read_variables` of [`CELL_STATUS`], `write_variable(.., TRUE)` pulses (`bReset`, `bRecoveryConfirm`; never `bStart`), `disconnect_from_device` (always) |
//! | stop | `close_project(saveFirst=false)` of the head, then as verify up to `get_application_state`, `write_variable(PLC_PRG.bStop, TRUE)`, `read_variables`, `disconnect_from_device` |
//!
//! `map_io` uses `io_mapping_csv`, not the per-channel `slots[].io`: the server has no tool that
//! maps one channel, and `import_io_mappings_csv` reads a file on the IDE, which no tool writes
//! from text; the mirror route ([`IoCsv::Mirror`]) is the only way with the server's own tools.

use std::collections::BTreeSet;

use serde_json::{Map, Value, json};

use crate::config::IoCsv;
use crate::project::{DeviceId, DeviceNode, Manifest, Project, Role, SymbolConfig};
use crate::{Error, Result};

/// The device `create_project` adds (`project.add('Device', ...)`).
pub const ROOT: &str = "Device";
pub const APP: &str = "Application";
/// The scratch GVL that carries the IO mapping CSV to a file ([`IoCsv::Mirror`]).
pub const IO_CARRIER: &str = "P2C_IoMappingCsv";

/// Every tool the backend calls; preflight checks the server has them.
pub const TOOLS: &[&str] = &[
    "get_codesys_status",
    "launch_codesys",
    "shutdown_codesys",
    "find_device_description",
    "create_project",
    "get_device_identification",
    "add_device",
    "list_device_parameters",
    "get_device_parameter",
    "set_device_parameter",
    "plug_device",
    "unplug_device",
    "add_library",
    "list_project_libraries",
    "download_missing_libraries",
    "rename_object",
    "create_dut",
    "create_gvl",
    "create_pou",
    "set_pou_code",
    "get_all_pou_code",
    "list_tasks",
    "create_task",
    "configure_task",
    "add_pou_to_task",
    "set_plc_settings",
    "get_plc_settings",
    "mirror_export",
    "import_io_mappings_csv",
    "delete_object",
    "compile_project",
    "save_project",
    "rebind_device_to_scan_result",
    "connect_to_device",
    "add_device_user",
    "download_to_device",
    "create_boot_application",
    "start_stop_application",
    "get_application_state",
    "read_variables",
    "write_variable",
    "disconnect_from_device",
    "open_project",
    "close_project",
    "create_symbol_config",
    "set_symbol_config_settings",
    "set_symbol_access",
    "set_signature_access_bulk",
    "list_configured_symbols",
];

/// Optional tool arguments the server may have (read from the input schemas of `tools/list`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Caps {
    /// `add_device(moduleId)`: a module into a free child list (a PN device without slots).
    pub module_id: bool,
    /// `get_device_parameter(elementPath)` / `set_device_parameter(elementPath)`: a member of a
    /// nested struct (`Identification.StationName`).
    pub element_path: bool,
    /// `export_project_libraries(projectFilePath, outDir)`: the project's libraries stored next
    /// to it (`libraries\` with `libraries.json`), so they travel with the project archive.
    pub export_libraries: bool,
    /// `install_libraries(dir)`: those libraries installed into the IDE's repository.
    pub install_libraries: bool,
}

impl Caps {
    /// Everything (our overlay of the server).
    pub const ALL: Caps = Caps {
        module_id: true,
        element_path: true,
        export_libraries: true,
        install_libraries: true,
    };

    pub fn from_tools(tools: &[crate::mcp::ToolInfo]) -> Self {
        let has = |tool: &str, arg: &str| tools.iter().any(|t| t.name == tool && t.has_arg(arg));
        Caps {
            module_id: has("add_device", "moduleId"),
            element_path: has("set_device_parameter", "elementPath")
                && has("get_device_parameter", "elementPath"),
            export_libraries: has(EXPORT_LIBRARIES, "outDir"),
            install_libraries: has(INSTALL_LIBRARIES, "dir"),
        }
    }

    /// What the server lacks, for a note.
    pub fn missing(&self) -> Vec<&'static str> {
        let mut v = Vec::new();
        if !self.module_id {
            v.push("add_device moduleId (modules of devices without slots are human steps)");
        }
        if !self.element_path {
            v.push("device parameter elementPath (members of nested structs are human steps)");
        }
        if !self.export_libraries {
            v.push(
                "export_project_libraries (the project's libraries are not stored with it; a \
                 restored archive relies on download_missing_libraries)",
            );
        }
        if !self.install_libraries {
            v.push("install_libraries (start downloads missing libraries instead)");
        }
        v
    }
}

/// The overlay tool that stores a project's libraries next to it ([`Caps::export_libraries`]).
pub const EXPORT_LIBRARIES: &str = "export_project_libraries";
/// The overlay tool that installs stored libraries ([`Caps::install_libraries`]).
pub const INSTALL_LIBRARIES: &str = "install_libraries";
/// The folder of the project directory with the stored libraries, and the file listing them.
pub const LIBRARIES_DIR: &str = "libraries";
pub const LIBRARIES_JSON: &str = "libraries.json";

/// Where and how the calls run.
#[derive(Clone, Debug)]
pub struct Options {
    /// `windows_project_dir` for this cell.
    pub dir: String,
    /// `install` on every call; empty: left out.
    pub install: String,
    pub io_csv: IoCsv,
    /// The runtime login; `None`: the server's `CODESYS_DEVICE_USER` / `_PASSWORD`.
    pub credentials: Option<(String, String)>,
    /// The optional arguments the server has.
    pub caps: Caps,
    /// `[address_map]` of the cell config: the gateway binds a runtime at its mapped address.
    pub address_map: std::collections::BTreeMap<String, String>,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            dir: r"C:\ProgramData\plan2cell\projects\cell".into(),
            install: "SP22 Patch 1".into(),
            io_csv: IoCsv::Mirror,
            credentials: None,
            caps: Caps::default(),
            address_map: Default::default(),
        }
    }
}

impl Options {
    /// The address the IDE's gateway reaches `ip` at (`[address_map]`, else `ip`).
    pub fn reach<'a>(&'a self, ip: &'a str) -> &'a str {
        self.address_map.get(ip).map_or(ip, String::as_str)
    }

    pub fn project_path(&self, p: &Project) -> String {
        format!("{}\\{}", self.dir, p.file)
    }

    /// `<dir>\libraries`: the libraries `export_project_libraries` stored, with `libraries.json`.
    pub fn libraries_dir(&self) -> String {
        format!("{}\\{LIBRARIES_DIR}", self.dir)
    }

    fn mirror_root(&self, p: &Project) -> String {
        format!("{}\\{}_io", self.dir, p.key)
    }

    /// The file `import_io_mappings_csv` reads.
    pub fn io_csv_path(&self, p: &Project) -> String {
        match self.io_csv {
            IoCsv::Mirror => format!(
                "{}\\{ROOT}\\Plc Logic\\{APP}\\{IO_CARRIER}.st",
                self.mirror_root(p)
            ),
            IoCsv::Staged => format!("{}\\{}_io_mapping.csv", self.dir, p.key),
        }
    }
}

/// An argument known only when the call runs.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Deferred {
    /// The display name of a device description (from `find_device_description`).
    DeviceName(DeviceId),
    /// The 0-based sub-element of a struct parameter named `member` (from `get_device_parameter`).
    MemberIndex {
        device: String,
        param: u32,
        member: String,
    },
}

impl Deferred {
    /// The JSON type the argument has once resolved.
    pub fn json_type(&self) -> &'static str {
        match self {
            Deferred::DeviceName(_) => "string",
            Deferred::MemberIndex { .. } => "integer",
        }
    }

    fn describe(&self) -> String {
        match self {
            Deferred::DeviceName(d) => format!("<name of {}>", show_id(d)),
            Deferred::MemberIndex { param, member, .. } => {
                format!("<index of {member} in parameter {param}>")
            }
        }
    }
}

/// A call that runs only if this holds for the last `list_tasks`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum When {
    TaskMissing(String),
    PouNotInTask { task: String, pou: String },
}

impl When {
    pub fn holds(&self, tasks: &[(String, Vec<String>)]) -> bool {
        let find = |t: &str| tasks.iter().find(|(n, _)| n.eq_ignore_ascii_case(t));
        match self {
            When::TaskMissing(t) => find(t).is_none(),
            When::PouNotInTask { task, pou } => {
                !find(task).is_some_and(|(_, pous)| pous.iter().any(|p| same_pou(p, pou)))
            }
        }
    }

    fn describe(&self) -> String {
        match self {
            When::TaskMissing(t) => format!("if task {t} is missing"),
            When::PouNotInTask { task, pou } => format!("if {task} does not call {pou}"),
        }
    }
}

pub fn same_pou(have: &str, want: &str) -> bool {
    let h = have.trim().trim_end_matches("()");
    h.eq_ignore_ascii_case(want) || h.rsplit('.').next() == Some(want)
}

/// A task as `configure_tasks` wants it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TaskWant {
    pub name: String,
    pub pous: Vec<String>,
    /// Only these POUs (mode `ensure`).
    pub exclusive: bool,
}

/// How a call's answer is read.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Check {
    /// Success is enough.
    Done,
    /// `get_codesys_status`: the IDE version, as a note.
    Status,
    /// `find_device_description`: installed, and its display name.
    Installed(DeviceId),
    /// `get_device_identification`: the device exists (and has this DeviceID).
    Ident {
        device: String,
        want: Option<DeviceId>,
    },
    /// `get_device_parameter` before a member write: the sub-element labels.
    SubElements { device: String, param: u32 },
    /// `list_device_parameters`: these scalar values read back.
    Params {
        device: String,
        want: Vec<(u32, String)>,
    },
    /// `get_device_parameter`: these members read back.
    Members {
        device: String,
        param: u32,
        want: Vec<(String, String)>,
    },
    /// `list_project_libraries`: `ensure` ones present (else a deviation), `auto` ones present
    /// (else a warning).
    Libraries {
        ensure: Vec<String>,
        auto: Vec<String>,
    },
    /// `get_all_pou_code`: every object with this text (name, declaration, implementation).
    Code(Vec<(String, String, Option<String>)>),
    /// `list_tasks`: remember the tasks; check these.
    Tasks(Vec<TaskWant>),
    /// `get_plc_settings`: this bus cycle task.
    BusCycle { device: String, task: String },
    /// The project structure: the module in a slot of a device (see [`Module`]).
    Module(Module),
    /// The project structure: the slot must be empty, a module there is unplugged (see
    /// [`EmptySlot`]).
    EmptySlot(EmptySlot),
    /// The project structure again after `unplug_device`: the slot is empty now.
    SlotEmpty(EmptySlot),
    /// `list_device_parameters` of a module: its `Module parameters` start with this ident.
    ModuleIdent { device: String, ident: String },
    /// `create_gvl` of the IO mapping CSV carrier: rows of modules that could not be added are
    /// left out first (see [`drop_rows`]).
    IoCarrier,
    /// `compile_project`: its errors and warnings (errors are not a failed call).
    Compile,
    /// `export_project_libraries` after `save`: skipped after a build with errors, a failure is
    /// a warning; the answer is kept for the copy on the workspace ([`exported_libraries`]).
    ExportLibraries,
    /// `install_libraries` before a login: an archive without `libraries.json` (older ones)
    /// gets `download_missing_libraries` instead ([`no_libraries_json`]); any other failure
    /// stops the stage (exit 8).
    InstallLibraries,
    /// `download_missing_libraries` before a login: a failure is a note (the build decides).
    DownloadLibraries,
    /// `list_project_libraries` before a login: references that resolve to nothing are kept for
    /// the build's message; an `ERROR` line or a failed call stops the stage (exit 8).
    LibrariesResolved,
    /// `compile_project` before a login: any error stops the stage at once (exit 8) with the
    /// compiler's messages.
    BuildClean,
    /// `connect_to_device`: a refused first login is answered with `add_device_user`.
    Connect,
    /// `get_application_state`.
    AppState,
    /// `read_variables`.
    Values,
    /// A call that may fail (an `optional` symbol): its failure is a warning.
    Optional(String),
    /// `list_configured_symbols`: these `Signature.variable` exported readable.
    Symbols(Vec<String>),
}

/// `ensure_module`: the module `name` in slot `slot` (1-based) of `device`.
///
/// What a device's children are decides the call: a child `name` is already there (nothing);
/// an empty slot (`<Empty>`) is plugged (`plug_device` with the parent's DeviceID and the
/// module id); a module the device description put into the slot (a PROFINET device under a
/// controller comes with its modules) is renamed (`rename_object`); a device without such a
/// child takes its modules as a free child list, which no tool of the server fills with a
/// module id with plain 0.19.0 (`add_device` has no `moduleId`): a human step; with
/// [`Caps::module_id`] the module is added to it (`add_device(.., moduleId)`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Module {
    pub device: String,
    pub slot: u32,
    pub name: String,
    pub parent: DeviceId,
    pub module_id: String,
    pub ident: Option<String>,
    /// The server adds a module by module id ([`Caps::module_id`]).
    pub add: bool,
}

/// What [`Module`] needs done, from the device's children.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ModuleAction {
    Present,
    Plug,
    Rename(String),
    /// No child at the slot position: (children there are). Added with `add_device` if the
    /// server can ([`Module::add`]), else a human step.
    FreeList(usize),
    /// The device is not in the project structure.
    NoDevice,
}

impl Module {
    pub fn action(&self, structure: &str) -> ModuleAction {
        let Some(kids) = children(structure, &self.device) else {
            return ModuleAction::NoDevice;
        };
        if kids.iter().any(|k| k == &self.name) {
            return ModuleAction::Present;
        }
        match kids.get((self.slot as usize).wrapping_sub(1)) {
            Some(k) if is_empty_slot(k) => ModuleAction::Plug,
            Some(k) => ModuleAction::Rename(k.clone()),
            None => ModuleAction::FreeList(kids.len()),
        }
    }

    pub fn path(&self) -> String {
        format!("{}/{}", self.device, self.name)
    }

    /// The calls for `action`, and the read back of the module's ident.
    pub fn calls(&self, action: &ModuleAction, o: &Options, project: &str) -> Vec<Call> {
        let pp = Some(project);
        let mut v = match action {
            ModuleAction::Plug => vec![Call::new(
                "plug_device",
                o,
                pp,
                json!({"parentPath": self.device, "slotIndex": self.slot, "name": self.name,
                       "deviceType": self.parent.ty, "deviceId": self.parent.id,
                       "deviceVersion": self.parent.version, "moduleId": self.module_id}),
            )],
            ModuleAction::Rename(from) => vec![Call::new(
                "rename_object",
                o,
                pp,
                json!({"objectPath": format!("{}/{from}", self.device), "newName": self.name,
                       "updateReferences": false}),
            )],
            ModuleAction::FreeList(_) if self.add => vec![
                Call::new(
                    "add_device",
                    o,
                    pp,
                    json!({"parentPath": self.device, "deviceName": self.name,
                           "targetVersion": self.parent.version, "moduleId": self.module_id}),
                )
                .defer(
                    "targetDeviceName",
                    Deferred::DeviceName(self.parent.clone()),
                ),
            ],
            ModuleAction::Present => Vec::new(),
            ModuleAction::FreeList(_) | ModuleAction::NoDevice => return Vec::new(),
        };
        if let Some(ident) = &self.ident {
            v.push(
                Call::new(
                    "list_device_parameters",
                    o,
                    pp,
                    json!({"devicePath": self.path()}),
                )
                .check(Check::ModuleIdent {
                    device: self.path(),
                    ident: ident.clone(),
                }),
            );
        }
        v
    }
}

/// `ensure_empty_slot`: slot `slot` (1-based) of `device` holds no module (`project.json` slot
/// `"empty": true`, `"module_id": null`). A device whose description plugs a default module
/// there when it is added (the KUKA GSDML's DAP `DIM 3`: FDIO64 `UsedInSlots` 1, removable)
/// has it removed with `unplug_device`; the structure is read again to see `<Empty>`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EmptySlot {
    pub device: String,
    pub slot: u32,
}

/// What [`EmptySlot`] needs done, from the device's children.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EmptySlotAction {
    /// The slot shows `<Empty>`.
    Empty,
    /// This module is in the slot.
    Unplug(String),
    /// The device has no such slot: (children there are).
    NoSlot(usize),
    /// The device is not in the project structure.
    NoDevice,
}

impl EmptySlot {
    pub fn action(&self, structure: &str) -> EmptySlotAction {
        let Some(kids) = children(structure, &self.device) else {
            return EmptySlotAction::NoDevice;
        };
        match kids.get((self.slot as usize).wrapping_sub(1)) {
            Some(k) if is_empty_slot(k) => EmptySlotAction::Empty,
            Some(k) => EmptySlotAction::Unplug(k.clone()),
            None => EmptySlotAction::NoSlot(kids.len()),
        }
    }

    /// For [`EmptySlotAction::Unplug`]: `unplug_device` of the module, then the structure read
    /// back ([`Check::SlotEmpty`]).
    pub fn calls(&self, action: &EmptySlotAction, o: &Options, project: &str) -> Vec<Call> {
        let EmptySlotAction::Unplug(module) = action else {
            return Vec::new();
        };
        vec![
            Call::new(
                "unplug_device",
                o,
                Some(project),
                json!({"devicePath": format!("{}/{module}", self.device)}),
            ),
            structure(project).check(Check::SlotEmpty(self.clone())),
        ]
    }
}

/// The IO mapping CSV without the rows whose device (last column) is one of `absent`, and the
/// number of rows left out. CODESYS reports each such row at the next build ("The device X
/// cannot be found below PLC").
pub fn drop_rows(csv: &str, absent: &[String]) -> (String, usize) {
    let mut dropped = 0;
    let mut out = Vec::new();
    for l in csv.lines() {
        let dev = l.rsplit(',').next().unwrap_or("").trim();
        if !l.trim_start().starts_with("//") && absent.iter().any(|a| a == dev) {
            dropped += 1;
            continue;
        }
        out.push(l);
    }
    let mut text = out.join("\n");
    if csv.ends_with('\n') {
        text.push('\n');
    }
    (text, dropped)
}

/// Not a tool: `resources/read` of the server's `project-structure` resource.
pub const STRUCTURE: &str = "resources/read";

/// Read the project tree (`codesys://project/{path}/structure`).
pub fn structure(project: &str) -> Call {
    let mut args = Map::new();
    args.insert(
        "uri".into(),
        Value::String(format!("codesys://project/{project}/structure")),
    );
    Call {
        tool: STRUCTURE,
        args,
        deferred: Vec::new(),
        when: None,
        check: Check::Done,
    }
}

/// The project structure resource as (depth, name): `  - Name (Type) {guid}`, two blanks a
/// level, the project itself at depth 0.
pub fn parse_structure(text: &str) -> Vec<(usize, String)> {
    text.lines()
        .filter_map(|l| {
            let rest = l.trim_start_matches(' ');
            let depth = (l.len() - rest.len()) / 2;
            let item = rest.strip_prefix("- ")?;
            let item = match item.rfind(" {") {
                Some(i) if item.ends_with('}') => &item[..i],
                _ => item,
            };
            let name = match item.rfind(" (") {
                Some(i) if item.ends_with(')') => &item[..i],
                _ => item,
            };
            Some((depth, name.trim().to_string()))
        })
        .collect()
}

/// The children of the object at `path` (`Device/Ethernet/kuka_robi1`), from the structure.
pub fn children(text: &str, path: &str) -> Option<Vec<String>> {
    let items = parse_structure(text);
    let mut at = 0; // index of the current node; 0 is the project
    let mut depth = items.first()?.0;
    for part in path.split('/').filter(|p| !p.is_empty()) {
        let mut found = None;
        for (i, (d, n)) in items.iter().enumerate().skip(at + 1) {
            if *d <= depth {
                break;
            }
            if *d == depth + 1 && n.eq_ignore_ascii_case(part) {
                found = Some(i);
                break;
            }
        }
        at = found?;
        depth = items[at].0;
    }
    let mut out = Vec::new();
    for (d, n) in items.iter().skip(at + 1) {
        if *d <= depth {
            break;
        }
        if *d == depth + 1 {
            out.push(n.clone());
        }
    }
    Some(out)
}

#[derive(Clone, Debug, PartialEq)]
pub struct Call {
    pub tool: &'static str,
    pub args: Map<String, Value>,
    pub deferred: Vec<(&'static str, Deferred)>,
    pub when: Option<When>,
    pub check: Check,
}

const SECRET_ARGS: &[&str] = &["devicePassword", "userPassword", "password"];

impl Call {
    fn new(tool: &'static str, o: &Options, project: Option<&str>, args: Value) -> Self {
        let mut a = Map::new();
        if let Some(p) = project {
            a.insert("projectFilePath".into(), Value::String(p.into()));
        }
        if let Value::Object(m) = args {
            a.extend(m);
        }
        if !o.install.is_empty() {
            a.insert("install".into(), Value::String(o.install.clone()));
        }
        Self {
            tool,
            args: a,
            deferred: Vec::new(),
            when: None,
            check: Check::Done,
        }
    }

    fn check(mut self, c: Check) -> Self {
        self.check = c;
        self
    }

    fn defer(mut self, arg: &'static str, d: Deferred) -> Self {
        self.deferred.push((arg, d));
        self
    }

    fn when(mut self, w: When) -> Self {
        self.when = Some(w);
        self
    }

    /// The secret argument values (passwords) this call sends, for masking answers.
    pub fn secrets(&self) -> Vec<&str> {
        SECRET_ARGS
            .iter()
            .filter_map(|k| self.args.get(*k).and_then(Value::as_str))
            .filter(|v| !v.is_empty())
            .collect()
    }

    /// `tool(arg=value, ...)`: secrets masked, code shortened, deferred arguments described.
    pub fn show(&self) -> String {
        let mut parts: Vec<String> = Vec::new();
        for (k, v) in &self.args {
            let s = if SECRET_ARGS.contains(&k.as_str()) {
                "<secret>".to_string()
            } else {
                match v {
                    Value::String(s) if s.contains('\n') || s.len() > 240 => {
                        format!("<{} lines>", s.lines().count())
                    }
                    other => other.to_string(),
                }
            };
            parts.push(format!("{k}={s}"));
        }
        for (k, d) in &self.deferred {
            parts.push(format!("{k}={}", d.describe()));
        }
        let mut s = format!("{}({})", self.tool, parts.join(", "));
        if let Some(w) = &self.when {
            s.push_str(&format!(" [{}]", w.describe()));
        }
        s
    }
}

fn show_id(d: &DeviceId) -> String {
    format!("{} '{}' '{}'", d.ty, d.id, d.version)
}

/// The device tree as (path, node), depth first; the root is [`ROOT`].
pub fn device_paths(root: &DeviceNode) -> Vec<(String, &DeviceNode)> {
    fn walk<'a>(n: &'a DeviceNode, path: String, out: &mut Vec<(String, &'a DeviceNode)>) {
        for c in &n.children {
            let p = format!("{path}/{}", c.name);
            out.push((p.clone(), c));
            walk(c, p, out);
        }
    }
    let mut out = vec![(ROOT.to_string(), root)];
    walk(root, ROOT.to_string(), &mut out);
    out
}

fn path_of(p: &Project, name: &str) -> Result<(String, DeviceNode)> {
    device_paths(&p.device_tree)
        .into_iter()
        .find(|(_, n)| n.name == name)
        .map(|(path, n)| (path, n.clone()))
        .ok_or_else(|| Error::Config(format!("{}: no device {name} in device_tree", p.name)))
}

/// `t#4ms` -> ("4", "ms").
pub fn interval(t: &str) -> Result<(String, String)> {
    let l = t.trim().to_ascii_lowercase();
    let v = l
        .strip_prefix("t#")
        .or_else(|| l.strip_prefix("time#"))
        .unwrap_or(&l);
    let split = v.find(|c: char| !c.is_ascii_digit()).unwrap_or(v.len());
    let (n, unit) = v.split_at(split);
    if n.is_empty() || !matches!(unit, "us" | "ms" | "s") {
        return Err(Error::Config(format!(
            "task interval {t}: want t#<n>us|ms|s"
        )));
    }
    Ok((n.into(), unit.into()))
}

/// ST text without comments and pragmas.
fn strip_st(s: &str) -> String {
    let b: Vec<char> = s.chars().collect();
    let (mut i, mut depth, mut out) = (0, 0usize, String::new());
    while i < b.len() {
        let two = (b[i], b.get(i + 1).copied().unwrap_or('\0'));
        if two == ('(', '*') {
            depth += 1;
            i += 2;
        } else if depth > 0 && two == ('*', ')') {
            depth -= 1;
            i += 2;
        } else if depth > 0 {
            i += 1;
        } else if two == ('/', '/') {
            while i < b.len() && b[i] != '\n' {
                i += 1;
            }
        } else if b[i] == '{' {
            while i < b.len() && b[i] != '}' {
                i += 1;
            }
            i += 1;
        } else {
            out.push(b[i]);
            i += 1;
        }
    }
    out
}

/// The `dutType` of `create_dut` for a declaration `TYPE X : ...`.
pub fn dut_type(decl: &str) -> Result<&'static str> {
    let upper = strip_st(decl).to_ascii_uppercase();
    let ident = |c: char| c.is_ascii_alphanumeric() || c == '_';
    let at = upper
        .match_indices("TYPE")
        .map(|(i, _)| i)
        .find(|&i| {
            !upper[..i].chars().next_back().is_some_and(ident)
                && !upper[i + 4..].chars().next().is_some_and(ident)
        })
        .ok_or_else(|| Error::Config("DUT declaration without TYPE".into()))?;
    let colon = upper[at..]
        .find(':')
        .ok_or_else(|| Error::Config("DUT declaration without ':'".into()))?;
    let rest = upper[at + colon + 1..].trim_start();
    Ok(if rest.starts_with('(') {
        "Enumeration"
    } else if rest.starts_with("STRUCT") {
        "Structure"
    } else if rest.starts_with("UNION") {
        "Union"
    } else {
        "Alias"
    })
}

/// `launch_codesys` with `otherInstances: close`: the server closes the CODESYS of the install
/// it asked about (those it cannot take over) like their window's X, then starts its own. Only
/// for a cell where nobody works in the IDE by hand (`codesys_mcp.other_codesys = "close"`).
pub fn close_other_codesys(o: &Options) -> Call {
    Call::new(
        "launch_codesys",
        o,
        None,
        json!({"otherInstances": "close"}),
    )
}

/// Check the IDE before anything is built: its status and every DeviceID in the device
/// repository.
pub fn preflight(devices: &[DeviceId], o: &Options) -> Vec<Call> {
    let mut v = vec![Call::new("get_codesys_status", o, None, json!({})).check(Check::Status)];
    for d in devices {
        v.push(find_device(d, o));
    }
    v
}

fn find_device(d: &DeviceId, o: &Options) -> Call {
    Call::new(
        "find_device_description",
        o,
        None,
        json!({"deviceType": d.ty, "deviceId": d.id, "deviceVersion": d.version}),
    )
    .check(Check::Installed(d.clone()))
}

/// Create, configure, build and save one project, in the order of its `steps[]`.
///
/// `source` returns a file of the build directory by its path relative to `plc/`
/// (`codesys/head/DUT/E_CellError.st`).
pub fn project(
    p: &Project,
    source: &dyn Fn(&str) -> Result<String>,
    o: &Options,
) -> Result<Vec<Call>> {
    let path = o.project_path(p);
    let pp = Some(path.as_str());
    let tree = &p.device_tree;
    let mut v = Vec::new();
    // The display names add_device and create_project select a description by.
    let mut ids = Vec::new();
    for (_, n) in device_paths(tree) {
        if !ids.contains(&n.device_id) {
            ids.push(n.device_id.clone());
        }
    }
    for d in &ids {
        v.push(find_device(d, o));
    }
    for st in &p.steps {
        let need = |x: &Option<String>, what: &str| {
            x.clone()
                .ok_or_else(|| Error::Config(format!("{}: step {} without {what}", p.name, st.op)))
        };
        match st.op.as_str() {
            "create_project" => {
                v.push(
                    Call::new("create_project", o, None, json!({"filePath": path}))
                        .defer("deviceName", Deferred::DeviceName(tree.device_id.clone())),
                );
                v.push(ident(o, pp, ROOT, Some(&tree.device_id)));
            }
            "add_device" => {
                let parent = need(&st.parent, "parent")?;
                let (ppath, _) = path_of(p, &parent)?;
                let (npath, n) = path_of(p, &need(&st.name, "name")?)?;
                v.push(
                    Call::new(
                        "add_device",
                        o,
                        pp,
                        json!({"parentPath": ppath, "deviceName": n.name,
                               "targetVersion": n.device_id.version}),
                    )
                    .defer(
                        "targetDeviceName",
                        Deferred::DeviceName(n.device_id.clone()),
                    ),
                );
                v.push(ident(o, pp, &npath, Some(&n.device_id)));
            }
            "set_device_parameters" => {
                let (dpath, n) = path_of(p, &need(&st.device, "device")?)?;
                v.extend(parameters(o, pp, &dpath, &n));
            }
            "ensure_module" => {
                let (dpath, n) = path_of(p, &need(&st.device, "device")?)?;
                let slot = st.slot.ok_or_else(|| {
                    Error::Config(format!("{}: ensure_module without slot", p.name))
                })?;
                let name = need(&st.name, "name")?;
                let module_id = need(&st.module_id, "module_id")?;
                let ident = n
                    .slots
                    .iter()
                    .find(|x| x.slot == slot)
                    .map(|x| x.ident.clone())
                    .filter(|x| !x.is_empty());
                v.push(structure(&path).check(Check::Module(Module {
                    device: dpath,
                    slot,
                    name,
                    parent: n.device_id.clone(),
                    module_id,
                    ident,
                    add: o.caps.module_id,
                })));
            }
            "ensure_empty_slot" => {
                let device = need(&st.device, "device")?;
                let (dpath, n) = path_of(p, &device)?;
                let slot = st.slot.ok_or_else(|| {
                    Error::Config(format!("{}: ensure_empty_slot without slot", p.name))
                })?;
                if !n.slots.iter().any(|x| x.slot == slot && x.empty) {
                    return Err(Error::Config(format!(
                        "{}: ensure_empty_slot {device} slot {slot}, but device_tree has no \
                         empty slot {slot} there",
                        p.name
                    )));
                }
                v.push(structure(&path).check(Check::EmptySlot(EmptySlot {
                    device: dpath,
                    slot,
                })));
            }
            "ensure_libraries" => {
                let (mut ensure, mut auto) = (Vec::new(), Vec::new());
                for l in &p.libraries {
                    if l.mode == "ensure" {
                        v.push(Call::new(
                            "add_library",
                            o,
                            pp,
                            json!({"libraryName": l.name}),
                        ));
                        ensure.push(l.name.clone());
                    } else {
                        auto.push(l.name.clone());
                    }
                }
                // The device descriptions pin placeholders to library versions the IDE may not
                // have (Linux SL 4.20.0.0: IoStandard 3.5.17.0 and its dependencies under
                // SP22): fetched from the CODESYS Store, a no-op when nothing is missing.
                v.push(Call::new("download_missing_libraries", o, pp, json!({})));
                v.push(
                    Call::new("list_project_libraries", o, pp, json!({}))
                        .check(Check::Libraries { ensure, auto }),
                );
            }
            "create_objects" => {
                let mut objs: Vec<_> = p.objects.iter().collect();
                objs.sort_by_key(|x| x.order);
                let mut want = Vec::new();
                for ob in objs {
                    if ob.parent != APP {
                        return Err(Error::Config(format!(
                            "{}: object {} under {}: only {APP} is supported",
                            p.name, ob.name, ob.parent
                        )));
                    }
                    let decl = source(&ob.declaration)?;
                    let at = format!("{APP}/{}", ob.name);
                    let mut imp = None;
                    match ob.kind.as_str() {
                        "dut" => {
                            let ty = dut_type(&decl)
                                .map_err(|e| Error::Config(format!("{}: {e}", ob.declaration)))?;
                            v.push(Call::new(
                                "create_dut",
                                o,
                                pp,
                                json!({"name": ob.name, "dutType": ty, "parentPath": APP}),
                            ));
                            v.push(Call::new(
                                "set_pou_code",
                                o,
                                pp,
                                json!({"pouPath": at, "declarationCode": decl}),
                            ));
                        }
                        "gvl" => v.push(Call::new(
                            "create_gvl",
                            o,
                            pp,
                            json!({"name": ob.name, "parentPath": APP, "declarationCode": decl}),
                        )),
                        "pou" => {
                            let ty = match ob.pou_type.as_deref() {
                                Some("function_block") => "FunctionBlock",
                                Some("program") => "Program",
                                Some("function") => "Function",
                                other => {
                                    return Err(Error::Config(format!(
                                        "{}: object {}: pou_type {other:?}",
                                        p.name, ob.name
                                    )));
                                }
                            };
                            let lang = ob.language.as_deref().unwrap_or("ST");
                            if lang != "ST" {
                                return Err(Error::Config(format!(
                                    "{}: object {}: only ST is supported",
                                    p.name, ob.name
                                )));
                            }
                            let body = match &ob.implementation {
                                Some(f) => source(f)?,
                                None => String::new(),
                            };
                            if ob.replace_existing {
                                v.push(Call::new(
                                    "set_pou_code",
                                    o,
                                    pp,
                                    json!({"pouPath": at, "declarationCode": decl,
                                           "implementationCode": body}),
                                ));
                            } else {
                                v.push(Call::new(
                                    "create_pou",
                                    o,
                                    pp,
                                    json!({"name": ob.name, "type": ty, "language": lang,
                                           "parentPath": APP, "declarationCode": decl,
                                           "implementationCode": body}),
                                ));
                            }
                            imp = Some(body);
                        }
                        k => {
                            return Err(Error::Config(format!(
                                "{}: object {}: kind {k}",
                                p.name, ob.name
                            )));
                        }
                    }
                    want.push((ob.name.clone(), decl, imp));
                }
                v.push(Call::new("get_all_pou_code", o, pp, json!({})).check(Check::Code(want)));
            }
            "configure_tasks" => v.extend(tasks(p, o, pp)?),
            "map_io" => {
                let csv = source(&p.io_mapping_csv)?;
                if o.io_csv == IoCsv::Mirror {
                    v.push(
                        Call::new(
                            "create_gvl",
                            o,
                            pp,
                            json!({"name": IO_CARRIER, "parentPath": APP, "declarationCode": csv}),
                        )
                        .check(Check::IoCarrier),
                    );
                    v.push(Call::new(
                        "mirror_export",
                        o,
                        pp,
                        json!({"mirrorRoot": o.mirror_root(p)}),
                    ));
                }
                v.push(Call::new(
                    "import_io_mappings_csv",
                    o,
                    pp,
                    json!({"csvPath": o.io_csv_path(p), "devicePath": ROOT}),
                ));
                if o.io_csv == IoCsv::Mirror {
                    v.push(Call::new(
                        "delete_object",
                        o,
                        pp,
                        json!({"objectPath": format!("{APP}/{IO_CARRIER}")}),
                    ));
                }
            }
            "build" => v.push(Call::new("compile_project", o, pp, json!({})).check(Check::Compile)),
            "symbol_config" => {
                let sc = p.symbol_config.as_ref().ok_or_else(|| {
                    Error::Config(format!(
                        "{}: step symbol_config without symbol_config",
                        p.name
                    ))
                })?;
                v.extend(symbol_config(sc, o, pp));
            }
            "save" => {
                v.push(Call::new("save_project", o, pp, json!({})));
                // After a good build (exec skips it after one with errors): the libraries go
                // with the project, so a restored archive can log in on a fresh IDE.
                if o.caps.export_libraries && v.iter().any(|c| c.check == Check::Compile) {
                    v.push(export_libraries(o, &path));
                }
            }
            op => {
                return Err(Error::Config(format!(
                    "{}: step {op} is not supported",
                    p.name
                )));
            }
        }
    }
    Ok(v)
}

/// `export_project_libraries` of a saved project into [`Options::libraries_dir`], the folder
/// the cell's projects share (the overlay replaces only this project's share of its
/// `libraries.json`). `includeData`: the files come back in the answer (base64, up to the
/// overlay's 16 MiB per answer) for the copy on the workspace.
fn export_libraries(o: &Options, project: &str) -> Call {
    Call::new(
        EXPORT_LIBRARIES,
        o,
        Some(project),
        json!({"outDir": o.libraries_dir(), "includeData": true}),
    )
    .check(Check::ExportLibraries)
}

/// `download_missing_libraries` of a project (the CODESYS Store), before a login.
pub fn download_missing_libraries(p: &Project, o: &Options) -> Call {
    Call::new(
        "download_missing_libraries",
        o,
        Some(&o.project_path(p)),
        json!({}),
    )
    .check(Check::DownloadLibraries)
}

/// Before the first login of a restored project (`start`): the libraries stored with it
/// installed (`install_libraries`; without the tool, `download_missing_libraries`), the
/// references listed and the application built. The login builds `Device.Application` again;
/// a library the fresh IDE lacks shows here with the compiler's own message instead of a
/// failed login.
pub fn libraries_ready(p: &Project, o: &Options) -> Vec<Call> {
    let path = o.project_path(p);
    let pp = Some(path.as_str());
    let first = if o.caps.install_libraries {
        Call::new(
            INSTALL_LIBRARIES,
            o,
            None,
            json!({"dir": o.libraries_dir()}),
        )
        .check(Check::InstallLibraries)
    } else {
        download_missing_libraries(p, o)
    };
    vec![
        first,
        Call::new("list_project_libraries", o, pp, json!({})).check(Check::LibrariesResolved),
        Call::new("compile_project", o, pp, json!({})).check(Check::BuildClean),
    ]
}

/// The Symbol Configuration: created with OPC UA support (idempotent), the OPC UA flag set
/// (a configuration that existed without it), the access of each entry, read back. Every
/// tool but the read back builds the application.
fn symbol_config(sc: &SymbolConfig, o: &Options, pp: Option<&str>) -> Vec<Call> {
    let mut v = vec![Call::new(
        "create_symbol_config",
        o,
        pp,
        json!({"supportOpcUa": sc.support_opc_ua}),
    )];
    if sc.support_opc_ua {
        v.push(Call::new(
            "set_symbol_config_settings",
            o,
            pp,
            json!({"contentFeatureFlags": ["SupportOPCUA"]}),
        ));
    }
    for a in &sc.symbols {
        let (c, what) = match &a.variable {
            None => (
                Call::new(
                    "set_signature_access_bulk",
                    o,
                    pp,
                    json!({"signatureFqn": a.signature, "access": a.access}),
                ),
                format!("symbol {} (all) {}", a.signature, a.access),
            ),
            Some(var) => (
                Call::new(
                    "set_symbol_access",
                    o,
                    pp,
                    json!({"signatureFqn": a.signature, "variableName": var, "access": a.access}),
                ),
                format!("symbol {}.{var} {}", a.signature, a.access),
            ),
        };
        v.push(if a.optional {
            c.check(Check::Optional(what))
        } else {
            c
        });
    }
    v.push(symbols_read_back(sc, o, pp));
    v
}

fn symbols_read_back(sc: &SymbolConfig, o: &Options, pp: Option<&str>) -> Call {
    Call::new("list_configured_symbols", o, pp, json!({})).check(Check::Symbols(sc.require.clone()))
}

fn ident(o: &Options, pp: Option<&str>, device: &str, want: Option<&DeviceId>) -> Call {
    Call::new(
        "get_device_identification",
        o,
        pp,
        json!({"devicePath": device}),
    )
    .check(Check::Ident {
        device: device.into(),
        want: want.cloned(),
    })
}

/// (member, value) pairs of one struct.
type Members = Vec<(String, String)>;

fn parameters(o: &Options, pp: Option<&str>, dpath: &str, n: &DeviceNode) -> Vec<Call> {
    let mut v = Vec::new();
    let mut scalars = Vec::new();
    // Members to read back, by parameter and the struct they are in (`None`: the parameter).
    let mut members: Vec<(u32, Option<String>, Members)> = Vec::new();
    let mut want = |id: u32, above: Option<String>, m: &str, value: &str| {
        let i = match members.iter().position(|(x, a, _)| *x == id && *a == above) {
            Some(i) => i,
            None => {
                members.push((id, above, Vec::new()));
                members.len() - 1
            }
        };
        members[i].2.push((m.to_string(), value.to_string()));
    };
    for prm in &n.parameters {
        match &prm.member {
            Some(m) if o.caps.element_path => {
                v.push(Call::new(
                    "set_device_parameter",
                    o,
                    pp,
                    json!({"devicePath": dpath, "parameterId": prm.id, "elementPath": m,
                           "value": prm.value}),
                ));
                want(
                    prm.id,
                    m.rsplit_once('.').map(|(a, _)| a.to_string()),
                    m,
                    &prm.value,
                );
            }
            None => {
                v.push(Call::new(
                    "set_device_parameter",
                    o,
                    pp,
                    json!({"devicePath": dpath, "parameterId": prm.id, "value": prm.value}),
                ));
                scalars.push((prm.id, prm.value.clone()));
            }
            Some(m) => {
                if !v.iter().any(|c: &Call| {
                    c.check
                        == Check::SubElements {
                            device: dpath.into(),
                            param: prm.id,
                        }
                }) {
                    v.push(
                        Call::new(
                            "get_device_parameter",
                            o,
                            pp,
                            json!({"devicePath": dpath, "parameterId": prm.id}),
                        )
                        .check(Check::SubElements {
                            device: dpath.into(),
                            param: prm.id,
                        }),
                    );
                }
                want(prm.id, None, m, &prm.value);
                v.push(
                    Call::new(
                        "set_device_parameter",
                        o,
                        pp,
                        json!({"devicePath": dpath, "parameterId": prm.id, "value": prm.value}),
                    )
                    .defer(
                        "elementIndex",
                        Deferred::MemberIndex {
                            device: dpath.into(),
                            param: prm.id,
                            member: m.clone(),
                        },
                    ),
                );
            }
        }
    }
    if !scalars.is_empty() {
        v.push(
            Call::new(
                "list_device_parameters",
                o,
                pp,
                json!({"devicePath": dpath}),
            )
            .check(Check::Params {
                device: dpath.into(),
                want: scalars,
            }),
        );
    }
    for (id, above, want) in members {
        let mut args = json!({"devicePath": dpath, "parameterId": id});
        if let Some(a) = above {
            args["elementPath"] = Value::String(a);
        }
        v.push(
            Call::new("get_device_parameter", o, pp, args).check(Check::Members {
                device: dpath.into(),
                param: id,
                want,
            }),
        );
    }
    v
}

fn tasks(p: &Project, o: &Options, pp: Option<&str>) -> Result<Vec<Call>> {
    let mut v = vec![Call::new("list_tasks", o, pp, json!({})).check(Check::Tasks(Vec::new()))];
    let mut want = Vec::new();
    let mut bus = Vec::new();
    for t in &p.tasks {
        if t.kind != "cyclic" {
            return Err(Error::Config(format!(
                "{}: task {}: kind {} (only cyclic)",
                p.name, t.name, t.kind
            )));
        }
        let (n, unit) = interval(&t.interval)?;
        v.push(
            Call::new("create_task", o, pp, json!({"taskName": t.name}))
                .when(When::TaskMissing(t.name.clone())),
        );
        v.push(Call::new(
            "configure_task",
            o,
            pp,
            json!({"taskName": t.name, "kind": "cyclic", "priority": t.priority.to_string(),
                   "interval": n, "intervalUnit": unit}),
        ));
        // A device's own call (PN_Controller.CommCycle) comes with the device: only checked.
        for pou in t.pous.iter().filter(|x| !x.contains('.')) {
            v.push(
                Call::new(
                    "add_pou_to_task",
                    o,
                    pp,
                    json!({"taskName": t.name, "pouName": pou}),
                )
                .when(When::PouNotInTask {
                    task: t.name.clone(),
                    pou: pou.clone(),
                }),
            );
        }
        if let Some(dev) = &t.bus_cycle_task_of {
            let (dpath, _) = path_of(p, dev)?;
            v.push(Call::new(
                "set_plc_settings",
                o,
                pp,
                json!({"devicePath": dpath, "busCycleTask": t.name}),
            ));
            bus.push((dpath, t.name.clone()));
        }
        want.push(TaskWant {
            name: t.name.clone(),
            pous: t.pous.clone(),
            exclusive: t.mode == "ensure",
        });
    }
    v.push(Call::new("list_tasks", o, pp, json!({})).check(Check::Tasks(want)));
    for (dpath, task) in bus {
        v.push(
            Call::new("get_plc_settings", o, pp, json!({"devicePath": dpath})).check(
                Check::BusCycle {
                    device: dpath,
                    task,
                },
            ),
        );
    }
    Ok(v)
}

fn credentials(o: &Options) -> Value {
    match &o.credentials {
        Some((u, pw)) => json!({"deviceUser": u, "devicePassword": pw}),
        None => json!({}),
    }
}

fn connect(o: &Options, pp: Option<&str>) -> Call {
    Call::new("connect_to_device", o, pp, credentials(o)).check(Check::Connect)
}

/// The first-login answer to a refused `connect_to_device`: create the configured user.
pub fn add_device_user(p: &Project, o: &Options) -> Option<Call> {
    let (u, pw) = o.credentials.as_ref()?;
    Some(Call::new(
        "add_device_user",
        o,
        Some(&o.project_path(p)),
        json!({"userName": u, "userPassword": pw}),
    ))
}

/// Bind the project's device to its runtime by IP through the gateway. Saved, so a later
/// `verify` (or the IDE) finds the binding; without it `connect_to_device` reports no gateway.
/// The address is the runtime's as the IDE reaches it ([`Options::reach`]: a workspace's own cell
/// maps its PROFINET addresses).
fn bind(p: &Project, o: &Options, pp: Option<&str>) -> [Call; 2] {
    let address = format!("{}:{}", o.reach(&p.runtime.ip), p.runtime.gateway_port);
    [
        Call::new(
            "rebind_device_to_scan_result",
            o,
            pp,
            json!({"matchAddress": address}),
        ),
        Call::new("save_project", o, pp, json!({})),
    ]
}

/// Bind the project's device to its runtime, log in, download, boot application, start.
pub fn deploy(p: &Project, o: &Options) -> Vec<Call> {
    let path = o.project_path(p);
    let pp = Some(path.as_str());
    let mut v = bind(p, o, pp).to_vec();
    v.extend([
        connect(o, pp),
        Call::new("download_to_device", o, pp, credentials(o)),
        Call::new("create_boot_application", o, pp, json!({"online": true})),
        Call::new("start_stop_application", o, pp, json!({"action": "start"})),
        Call::new("get_application_state", o, pp, json!({})).check(Check::AppState),
        Call::new("disconnect_from_device", o, pp, json!({})),
    ]);
    v
}

/// `open_project` of a project, for the IDE: the CODESYS the MCP drives is the one on the Windows
/// IDE's desktop (NICE DCV), so its primary project is what the user sees there. A project open
/// already stays as it is (no reload); one with unsaved changes is never closed for it. No login.
pub fn open_for_review(p: &Project, o: &Options) -> Call {
    Call::new(
        "open_project",
        o,
        None,
        json!({"filePath": o.project_path(p)}),
    )
}

/// `close_project` without saving: what the PLC IDE's user changed in the project (the review
/// over NICE DCV is view only) is discarded, so `bind`'s save never writes it to the file and the
/// login never downloads it; `create_project` refuses a project with unsaved changes. The next
/// call that names the project opens it again from its file. A different primary project with
/// unsaved changes is never closed for it (the call fails).
pub fn discard_ide_edits(p: &Project, o: &Options) -> Call {
    Call::new(
        "close_project",
        o,
        Some(&o.project_path(p)),
        json!({"saveFirst": false}),
    )
}

/// Bind (as `deploy`), log in and read the acceptance values.
pub fn verify(m: &Manifest, p: &Project, o: &Options) -> Vec<Call> {
    let path = o.project_path(p);
    let pp = Some(path.as_str());
    let exprs: Vec<String> = verify_checks(m, p).into_iter().map(|c| c.expr).collect();
    let mut v = bind(p, o, pp).to_vec();
    v.extend([
        connect(o, pp),
        Call::new("get_application_state", o, pp, json!({})).check(Check::AppState),
        Call::new("read_variables", o, pp, json!({"expressions": exprs})).check(Check::Values),
    ]);
    if let Some(sc) = &p.symbol_config {
        v.push(symbols_read_back(sc, o, pp));
    }
    v.push(disconnect(p, o));
    v
}

/// `disconnect_from_device` (logout). The server's script does not log in for it; a project
/// that is not open is opened first, one that does not exist fails.
pub fn disconnect(p: &Project, o: &Options) -> Call {
    Call::new(
        "disconnect_from_device",
        o,
        Some(&o.project_path(p)),
        json!({}),
    )
}

/// The head's status `start` reads: cell state, first fault and its robot, whether a recovery
/// confirmation is due, cycles.
pub const CELL_STATUS: [&str; 5] = [
    "PLC_PRG.eCellState",
    "PLC_PRG.fbCell.eError",
    "PLC_PRG.fbCell.nErrorRobot",
    "PLC_PRG.bRecoveryRequired",
    "PLC_PRG.udiCycles",
];

/// The libraries ready and the application built ([`libraries_ready`]), then bind (as
/// `deploy`), log in, read the application state ([`login`]).
pub fn online(p: &Project, o: &Options) -> Vec<Call> {
    let mut v = libraries_ready(p, o);
    v.extend(login(p, o));
    v
}

/// Bind (as `deploy`), log in, read the application state (`start` after [`libraries_ready`],
/// `stop`).
pub fn login(p: &Project, o: &Options) -> Vec<Call> {
    let path = o.project_path(p);
    let pp = Some(path.as_str());
    let mut v = bind(p, o, pp).to_vec();
    v.extend([
        connect(o, pp),
        Call::new("get_application_state", o, pp, json!({})).check(Check::AppState),
    ]);
    v
}

/// `plan2cell cell reset-ide`: the MCP server's CODESYS shut down without saving (the projects
/// are files; nothing done in the IDE since is kept), started again and its status read. The
/// logouts and `close_project(saveFirst=false)` of a build's projects come first
/// ([`disconnect`], [`discard_ide_edits`]).
pub fn reset_ide(o: &Options) -> Vec<Call> {
    vec![
        Call::new(
            "shutdown_codesys",
            o,
            None,
            json!({"unsavedChanges": "discard"}),
        ),
        Call::new("launch_codesys", o, None, json!({})),
        Call::new("get_codesys_status", o, None, json!({})).check(Check::Status),
    ]
}

/// Start the application of a logged-in project, read the state again.
pub fn start_application(p: &Project, o: &Options) -> Vec<Call> {
    let path = o.project_path(p);
    let pp = Some(path.as_str());
    vec![
        Call::new("start_stop_application", o, pp, json!({"action": "start"})),
        Call::new("get_application_state", o, pp, json!({})).check(Check::AppState),
    ]
}

/// `read_variables` of `exprs`.
pub fn read(p: &Project, o: &Options, exprs: &[&str]) -> Call {
    Call::new(
        "read_variables",
        o,
        Some(&o.project_path(p)),
        json!({"expressions": exprs}),
    )
    .check(Check::Values)
}

/// An operator pulse: `PLC_PRG` clears its buttons (`bStart`, `bStop`, `bReset`,
/// `bRecoveryConfirm`) at the end of every scan, so one write of TRUE is one rising edge.
pub fn pulse(p: &Project, o: &Options, expr: &str) -> Call {
    Call::new(
        "write_variable",
        o,
        Some(&o.project_path(p)),
        json!({"variablePath": expr, "value": "TRUE"}),
    )
}

/// `E_CellState` by name (`CS_ERROR`, `E_CellState.CS_ERROR`) or value (`99`).
pub fn cell_state(v: &str) -> Option<&'static str> {
    const STATES: [(&str, i64); 12] = [
        ("CS_INIT", 0),
        ("CS_BRINGUP", 10),
        ("CS_HOMING_SELECT", 20),
        ("CS_HOMING_RUN", 21),
        ("CS_READY", 30),
        ("CS_START_JOBS", 40),
        ("CS_RUNNING", 50),
        ("CS_CYCLE_DONE", 60),
        ("CS_ABORTING", 80),
        ("CS_RECOVERY", 85),
        ("CS_RESETTING", 90),
        ("CS_ERROR", 99),
    ];
    let v = v.trim();
    let tail = v.rsplit('.').next().unwrap_or(v).trim();
    // An enum may read back with its value: `CS_ERROR (99)`.
    let name = tail.split([' ', '(']).next().unwrap_or(tail);
    STATES
        .iter()
        .find(|(n, x)| n.eq_ignore_ascii_case(name) || tail.parse::<i64>().ok() == Some(*x))
        .map(|(n, _)| *n)
}

/// One value read online by `verify` and what it must be.
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize)]
pub struct Expect {
    pub expr: String,
    pub expect: String,
}

fn pn_devices(p: &Project) -> Vec<&str> {
    p.device_tree
        .descendants()
        .into_iter()
        .filter(|(_, n)| n.robot.is_some())
        .map(|(_, n)| n.name.as_str())
        .collect()
}

/// What `verify` reads from a running project and expects.
pub fn verify_checks(m: &Manifest, p: &Project) -> Vec<Expect> {
    let c = |e: &str, x: &str| Expect {
        expr: e.into(),
        expect: x.into(),
    };
    let plan_id = m.plan.plan_id.as_str();
    let mut v = match p.role {
        Role::HeadPlc => vec![
            c("GVL_Cell.PLAN_ID", plan_id),
            c("PLC_PRG.bCellError", "FALSE"),
        ],
        Role::RobotSim => vec![
            c("GVL_Program.PLAN_ID", plan_id),
            c("PLC_PRG.bSimActive", "TRUE"),
        ],
    };
    // The PROFINET link, from the controller's side: its device instances (PNIODevice) have
    // xRunning. The robot simulator's own device instance (IoDrvProfinetDevice ProfinetDevice)
    // has none: `kuka_robi1.xRunning` is "Invalid expression" there, and of the members that
    // do read (xError, xOnline) neither follows the link (FALSE / TRUE with the head's xRunning
    // FALSE, 2026-10-09), so the simulators have no PROFINET check of their own.
    if p.role == Role::HeadPlc {
        for d in pn_devices(p) {
            v.push(c(&format!("{d}.xRunning"), "TRUE"));
        }
    }
    v
}

/// An online value against what `verify` expects: IEC literals (`WORD#16#8024`, `16#8024`,
/// `32804`), booleans and enum names compare by value, anything else as trimmed text.
pub fn same_value(read: &str, expect: &str) -> bool {
    fn int(s: &str) -> Option<i128> {
        let mut parts: Vec<String> = s.trim().split('#').map(|p| p.replace('_', "")).collect();
        if parts.len() > 1 && parts[0].chars().any(|c| c.is_ascii_alphabetic()) {
            parts.remove(0);
        }
        match parts.as_slice() {
            [d] => d.parse().ok(),
            [base, digits] => {
                let b: u32 = base.parse().ok()?;
                (2..=36)
                    .contains(&b)
                    .then(|| i128::from_str_radix(digits, b).ok())
                    .flatten()
            }
            _ => None,
        }
    }
    // A STRING reads back quoted ('bt_1790934549'); the expectation is the bare text.
    let unquote = |s: &str| {
        let s = s.trim();
        s.strip_prefix('\'')
            .and_then(|x| x.strip_suffix('\''))
            .or_else(|| s.strip_prefix('"').and_then(|x| x.strip_suffix('"')))
            .unwrap_or(s)
            .to_string()
    };
    let (r, e) = (unquote(read), unquote(expect));
    let (r, e) = (r.as_str(), e.as_str());
    if let (Some(a), Some(b)) = (int(r), int(e)) {
        return a == b;
    }
    let tail = |s: &str| s.rsplit('.').next().unwrap_or(s).to_ascii_lowercase();
    r.eq_ignore_ascii_case(e) || tail(r) == tail(e)
}

/// A device parameter value as text, for comparing what was written with what reads back:
/// no blanks, no outer quotes, `{..}` as `[..]`, lower case.
pub fn norm_value(v: &str) -> String {
    let s: String = v.chars().filter(|c| !c.is_whitespace()).collect();
    let s = s.replace('{', "[").replace('}', "]");
    let s = s
        .strip_prefix('\'')
        .and_then(|x| x.strip_suffix('\''))
        .or_else(|| s.strip_prefix('"').and_then(|x| x.strip_suffix('"')))
        .unwrap_or(&s);
    s.to_lowercase()
}

/// Text as CODESYS stores it, for comparing: LF line ends, no trailing blanks.
pub fn norm_text(s: &str) -> String {
    s.replace("\r\n", "\n")
        .lines()
        .map(str::trim_end)
        .collect::<Vec<_>>()
        .join("\n")
        .trim()
        .to_string()
}

// --- reading the answers --------------------------------------------------------------------

/// `find_device_description`: `Some(name)` installed, `None` not.
pub fn parse_installed(text: &str) -> Option<String> {
    let installed = text
        .lines()
        .find_map(|l| l.trim().strip_prefix("Installed:"))
        .is_some_and(|v| v.trim().eq_ignore_ascii_case("true"));
    if !installed {
        return None;
    }
    let dev = text
        .lines()
        .find_map(|l| l.trim().strip_prefix("Device:"))?;
    Some(
        dev.split(" | vendor ")
            .next()
            .unwrap_or(dev)
            .trim()
            .to_string(),
    )
}

/// `get_device_identification`: the DeviceID.
pub fn parse_ident(text: &str) -> Option<DeviceId> {
    let get = |k: &str| {
        text.lines()
            .find_map(|l| l.trim().strip_prefix(k))
            .map(|v| v.trim().to_string())
    };
    Some(DeviceId {
        ty: get("type:")?.parse().ok()?,
        id: get("id:")?,
        version: get("version:")?,
    })
}

/// `get_device_parameter`: the sub-elements `[i] label = value`.
pub fn parse_sub_elements(text: &str) -> Vec<(i64, String, String)> {
    text.lines()
        .filter_map(|l| {
            let l = l.trim();
            let rest = l.strip_prefix('[')?;
            let (i, rest) = rest.split_once(']')?;
            let (label, value) = match rest.find(" =") {
                Some(i) => (&rest[..i], &rest[i + 2..]),
                None => (rest, ""),
            };
            Some((
                i.trim().parse().ok()?,
                label.trim().to_string(),
                value.trim().to_string(),
            ))
        })
        .collect()
}

/// A label or member name for comparing: letters and digits, lower case (`Station name` and
/// `StationName` are the same).
fn label_key(s: &str) -> String {
    s.chars()
        .filter(|c| c.is_alphanumeric())
        .flat_map(char::to_lowercase)
        .collect()
}

/// The sub-element of `member` (`StationName`, `Identification.StationName`): its label is the
/// member or its last part, compared without blanks and case (the server lists the visible
/// names, `Station name`).
pub fn member_index(subs: &[(i64, String, String)], member: &str) -> Option<i64> {
    let last = member.rsplit('.').next().unwrap_or(member);
    let find = |m: &str| subs.iter().find(|(_, l, _)| label_key(l) == label_key(m));
    find(member).or_else(|| find(last)).map(|(i, _, _)| *i)
}

/// A member below a struct sub-element (`Identification.StationName` where `Identification`
/// is itself a struct): the label of that sub-element. `set_device_parameter` writes one level
/// only, and a struct element refuses a whole write ("Operation not allowed on struct types").
pub fn nested_member<'a>(subs: &'a [(i64, String, String)], member: &str) -> Option<&'a str> {
    let (first, _) = member.split_once('.')?;
    subs.iter()
        .find(|(_, l, _)| label_key(l) == label_key(first))
        .map(|(_, l, _)| l.as_str())
}

/// `list_device_parameters`: (id, name, value) of the `scope\tid\tname\tvalue\tunit` rows.
pub fn parse_params(text: &str) -> Vec<(u32, String, String)> {
    text.lines()
        .filter_map(|l| {
            let f: Vec<&str> = l.split('\t').collect();
            (f.len() >= 4).then_some(())?;
            Some((
                f[1].trim().parse().ok()?,
                f[2].to_string(),
                f[3].to_string(),
            ))
        })
        .collect()
}

/// `list_project_libraries`: the reference names (`  - Standard, * (System) [...]`,
/// `  - #IoStandard [system, placeholder] ns=.. -> IoStandard, 3.5.17.0 (System)`); a
/// placeholder's `#` is dropped.
pub fn parse_libraries(text: &str) -> Vec<String> {
    text.lines()
        .filter_map(|l| l.trim_start().strip_prefix("- "))
        .map(|r| {
            r.split([',', '['])
                .next()
                .unwrap_or(r)
                .trim()
                .trim_start_matches('#')
                .to_string()
        })
        .collect()
}

/// `get_all_pou_code`: (name, declaration, implementation) of each `=== path (type) ===`
/// section.
pub fn parse_code(text: &str) -> Vec<(String, String, Option<String>)> {
    const DECL: &str = "// ----- Declaration -----";
    const IMPL: &str = "// ----- Implementation -----";
    let mut out: Vec<(String, String, Option<String>)> = Vec::new();
    let mut part = 0; // 1 declaration, 2 implementation
    for l in text.lines() {
        if let Some(h) = l.strip_prefix("=== ").and_then(|h| h.strip_suffix(" ===")) {
            let path = h.rsplit_once(" (").map(|(p, _)| p).unwrap_or(h);
            let name = path.rsplit('/').next().unwrap_or(path).to_string();
            out.push((name, String::new(), None));
            part = 0;
            continue;
        }
        let Some(cur) = out.last_mut() else { continue };
        if l == DECL {
            part = 1;
        } else if l == IMPL {
            part = 2;
            cur.2 = Some(String::new());
        } else if part == 1 {
            cur.1.push_str(l);
            cur.1.push('\n');
        } else if part == 2 {
            let s = cur.2.get_or_insert_with(String::new);
            s.push_str(l);
            s.push('\n');
        }
    }
    out
}

/// `list_tasks`: (task, POU calls).
pub fn parse_tasks(text: &str) -> Vec<(String, Vec<String>)> {
    let mut out: Vec<(String, Vec<String>)> = Vec::new();
    for l in text.lines() {
        let t = l.trim();
        if let Some(n) = t.strip_prefix("Task: ") {
            out.push((n.trim().to_string(), Vec::new()));
        } else if let Some(r) = t.strip_prefix("POU calls")
            && let Some(cur) = out.last_mut()
        {
            let list = r.split_once("):").map(|(_, x)| x.trim()).unwrap_or("");
            if list != "(none)" && !list.is_empty() {
                cur.1 = list.split(", ").map(|s| s.trim().to_string()).collect();
            }
        }
    }
    out
}

/// `compile_project`: (errors, warnings) from its `ERROR: ..` / `WARNING: ..` lines.
pub fn parse_compile(text: &str) -> (Vec<String>, Vec<String>) {
    let (mut e, mut w) = (Vec::new(), Vec::new());
    for l in text.lines() {
        if let Some(x) = l.strip_prefix("ERROR: ") {
            e.push(x.trim().to_string());
        } else if let Some(x) = l.strip_prefix("WARNING: ") {
            w.push(x.trim().to_string());
        }
    }
    (e, w)
}

/// The compiler's text of a library CODESYS could not open: "Could not open library
/// '#SysTask'. (Reason: The library 'SysTask, 3.5.17.0 (System)' has not been installed to the
/// system.)". The offline build resolves a library's own placeholders to versions no IDE here
/// has; the login's build, with the device bound to the running PLC, does not need them.
pub const LIBRARY_OPEN: &str = "Could not open library";

/// Errors of a build that are all library-open errors ([`LIBRARY_OPEN`]), and at least one.
pub fn library_open_only(errors: &[String]) -> bool {
    !errors.is_empty() && errors.iter().all(|x| x.contains(LIBRARY_OPEN))
}

/// How the overlay marks the build a symbol-config edit runs after it, when that build fails
/// (`withAutoCompile`): `<edit's answer>\n\n[auto-compile FAILED] <compile_project's answer>`.
pub const AUTO_COMPILE_FAILED: &str = "[auto-compile FAILED]";

/// A symbol-config edit (`create_symbol_config`, `set_symbol_config_settings`,
/// `set_signature_access_bulk`, `set_symbol_access`) that landed and whose auto-compile failed
/// only on library-open errors: those errors. `None` for anything else: the edit failed, the
/// build has another error, it names no errors (a build that did not run), or it lists fewer
/// `ERROR:` lines than its count says (an error text over several lines).
pub fn auto_compile_library_errors(text: &str) -> Option<Vec<String>> {
    let (edit, build) = text.split_once(AUTO_COMPILE_FAILED)?;
    if crate::mcp::ToolOutput::ok(edit).failed() {
        return None;
    }
    let (e, _) = parse_compile(build);
    let counted = build.lines().find_map(|l| {
        l.trim()
            .split_once(" error(s), ")
            .and_then(|(n, _)| n.parse::<usize>().ok())
    });
    (library_open_only(&e) && counted.is_none_or(|n| n == e.len())).then_some(e)
}

/// A failed login (or any online call) whose cause is the project, not the connection:
/// CODESYS builds `Device.Application` at login, and a build with errors or a library the IDE
/// lacks fails the same way on every attempt (no runtime restart or retry helps). Gateway,
/// connection and refused-login errors are not this. The texts CODESYS SP22 and the server
/// answer with; one place for them (a CODESYS update may change them: the test pins them).
pub fn build_failure(text: &str) -> bool {
    let t = text.to_ascii_lowercase();
    const BUILD: [&str; 5] = [
        "compile errors occurred",
        "build errors",
        "build error(s)",
        "compilation failed",
        "errors occurred during the build",
    ];
    if BUILD.iter().any(|p| t.contains(p)) {
        return true;
    }
    // A library CODESYS cannot resolve: "Library 'IoStandard, 3.5.17.0 (System)' could not be
    // found", "library ... is missing", "Unresolved library reference".
    t.lines().any(|l| {
        l.contains("librar")
            && [
                "not found",
                "could not be found",
                "cannot be found",
                "missing",
                "unresolved",
                "not installed",
                "could not be resolved",
                "cannot be resolved",
            ]
            .iter()
            .any(|p| l.contains(p))
    })
}

/// The overlay's fixed failure text of `install_libraries` for a folder without
/// `libraries.json`: `NO_LIBRARIES_JSON: <dir>\libraries.json not found.`
pub const NO_LIBRARIES_JSON: &str = "NO_LIBRARIES_JSON:";

/// `install_libraries` found no `libraries.json` in the folder: an archive made before
/// [`Caps::export_libraries`] (launch 15 and older). Only the overlay's fixed text
/// ([`NO_LIBRARIES_JSON`]) counts; an answer with the install's result (its markers) never
/// does, even when a library's error names `libraries.json`.
pub fn no_libraries_json(text: &str) -> bool {
    text.contains(NO_LIBRARIES_JSON) && !text.contains("### INSTALL_LIBRARIES_START ###")
}

/// `list_project_libraries`: the references that resolve to no library (a placeholder without
/// `-> name, version`), and the `ERROR` lines of the answer.
pub fn unresolved_libraries(text: &str) -> (Vec<String>, Vec<String>) {
    let mut unresolved = Vec::new();
    let mut errors = Vec::new();
    for l in text.lines() {
        let t = l.trim_start();
        if let Some(e) = t
            .strip_prefix("ERROR:")
            .or_else(|| t.strip_prefix("ERROR "))
        {
            errors.push(e.trim().to_string());
        } else if let Some(r) = t.strip_prefix("- ")
            && r.contains("placeholder")
            && !r.contains(" -> ")
        {
            unresolved.push(
                r.split([',', '['])
                    .next()
                    .unwrap_or(r)
                    .trim()
                    .trim_start_matches('#')
                    .to_string(),
            );
        }
    }
    (unresolved, errors)
}

/// One library `export_project_libraries` stored (`libraries.json`).
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct Library {
    pub name: String,
    #[serde(default)]
    pub vendor: String,
    #[serde(default)]
    pub version: String,
    /// The file in `libraries\`.
    #[serde(default)]
    pub file: String,
    #[serde(default)]
    pub sha256: String,
    /// The file's content (base64), when the answer carries it (`includeData`).
    #[serde(default, skip_serializing)]
    pub data: Option<String>,
    /// Why the answer has no `data` for it (the overlay's size budget per answer).
    #[serde(default, rename = "dataOmitted", skip_serializing)]
    pub data_omitted: Option<String>,
}

/// `export_project_libraries`' answer: `{project, outDir, manifest, manifestCount, removed,
/// count, libraries, errors}` between `### EXPORT_LIBRARIES_START ###` /
/// `### EXPORT_LIBRARIES_END ###`, `libraries` = this project's `[{name, vendor, version, file,
/// sha256, source}]`, each with `data` (the file in base64) or `dataOmitted` under
/// `includeData`. Without the markers, a JSON array or object in the answer. `None` when the
/// answer has no such list.
pub fn exported_libraries(text: &str) -> Option<Vec<Library>> {
    const START: &str = "### EXPORT_LIBRARIES_START ###";
    const END: &str = "### EXPORT_LIBRARIES_END ###";
    let json = match (text.find(START), text.find(END)) {
        (Some(a), Some(b)) if a < b => &text[a + START.len()..b],
        _ => {
            let a = text.find(['[', '{'])?;
            &text[a..=text.rfind([']', '}'])?]
        }
    };
    let v: Value = serde_json::from_str(json.trim()).ok()?;
    let list = match &v {
        Value::Array(_) => v,
        Value::Object(m) => m.get("libraries").cloned()?,
        _ => return None,
    };
    serde_json::from_value(list).ok()
}

/// `get_codesys_status`: the state of the current install (`ready`, `running`, `launching`,
/// `stopped`, `error`), else of the first one listed.
pub fn codesys_state(text: &str) -> Option<String> {
    let blocks: Vec<&str> = text.split("\n\n").collect();
    let block = blocks
        .iter()
        .find(|b| b.lines().next().is_some_and(|l| l.contains("<- current")))
        .or(blocks.first())?;
    block
        .lines()
        .find_map(|l| l.trim().strip_prefix("State:"))
        .map(|s| s.trim().to_ascii_lowercase())
        .filter(|s| !s.is_empty())
}

/// A CODESYS that takes tool calls (`ready`; `running` from older servers).
pub fn codesys_healthy(state: &str) -> bool {
    matches!(state, "ready" | "running")
}

/// `get_application_state`: the `State:` line.
pub fn parse_state(text: &str) -> Option<String> {
    text.lines()
        .find_map(|l| l.trim().strip_prefix("State:"))
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty() && s != "Unknown")
}

/// `read_variables`: `expr = value` lines.
pub fn parse_values(text: &str) -> Vec<(String, String)> {
    text.lines()
        .filter_map(|l| l.split_once(" = "))
        .map(|(e, v)| (e.trim().to_string(), v.trim().to_string()))
        .collect()
}

/// `list_configured_symbols`: every exported variable as `Signature.variable` with its effective
/// access (the configured one where none is reported), from the JSON between the script's
/// markers; `None` if there is none. Signatures of the application may be listed with the
/// application prefix (`Application.PLC_PRG`): it is dropped.
pub fn parse_symbols(text: &str) -> Option<Vec<(String, String)>> {
    let start = text.find("### SYMBOL_CONFIGURED_START ###")?;
    let rest = &text[start + "### SYMBOL_CONFIGURED_START ###".len()..];
    let json = &rest[..rest.find("### SYMBOL_CONFIGURED_END ###")?];
    let v: Value = serde_json::from_str(json.trim()).ok()?;
    let mut out = Vec::new();
    for list in ["configured_signatures", "configured_datatypes"] {
        for sig in v[list].as_array().into_iter().flatten() {
            let name = sig["fqn"]
                .as_str()
                .or(sig["name"].as_str())
                .unwrap_or("")
                .trim_start_matches("Application.");
            for var in sig["variables"].as_array().into_iter().flatten() {
                let Some(n) = var["name"].as_str() else {
                    continue;
                };
                let access = var["effective_access"]
                    .as_str()
                    .or(var["configured_access"].as_str())
                    .unwrap_or("None");
                out.push((format!("{name}.{n}"), access.to_string()));
            }
        }
    }
    Some(out)
}

/// An access value that includes reading (`Read`, `ReadOnly`, `ReadWrite`).
pub fn readable(access: &str) -> bool {
    access.to_ascii_lowercase().contains("read")
}

/// Every tool a list of calls uses, once.
pub fn tools_used(calls: &[Call]) -> BTreeSet<&'static str> {
    calls.iter().map(|c| c.tool).collect()
}

/// CODESYS shows an empty slot as `<Empty>`, and after `unplug_device` as `<EmptyN>` (N = slot).
fn is_empty_slot(name: &str) -> bool {
    name.strip_prefix("<Empty")
        .and_then(|r| r.strip_suffix('>'))
        .is_some_and(|n| n.chars().all(|c| c.is_ascii_digit()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_slot_names() {
        assert!(is_empty_slot("<Empty>"));
        assert!(is_empty_slot("<Empty1>"));
        assert!(!is_empty_slot("<Empty_x>"));
        assert!(!is_empty_slot("_64_safe_digital_in_and_outputs"));
    }

    #[test]
    fn intervals() {
        assert_eq!(interval("t#4ms").unwrap(), ("4".into(), "ms".into()));
        assert_eq!(interval("T#20MS").unwrap(), ("20".into(), "ms".into()));
        assert_eq!(interval("t#1s").unwrap(), ("1".into(), "s".into()));
        assert!(interval("t#1s500ms").is_err());
        assert!(interval("4").is_err());
    }

    #[test]
    fn values() {
        assert!(same_value("16#8024", "16#8024"));
        assert!(same_value("WORD#16#8024", "16#8024"));
        assert!(same_value("32804", "16#8024"));
        assert!(same_value("True", "TRUE"));
        assert!(same_value("E_CellState.CS_READY", "CS_READY"));
        assert!(same_value("'bt_1790934549'", "bt_1790934549"));
        assert!(same_value("\"bt_1\"", "bt_1"));
        assert!(!same_value("'bt_2'", "bt_1"));
        assert!(!same_value("FALSE", "TRUE"));
        assert!(!same_value("16#8025", "16#8024"));
    }

    #[test]
    fn dut_types() {
        let enum_ = "(*\n * E_X (TYPE in a comment: STRUCT)\n *)\n{attribute 'qualified_only'}\nTYPE E_X :\n(\n  A := 0\n);\nEND_TYPE\n";
        assert_eq!(dut_type(enum_).unwrap(), "Enumeration");
        assert_eq!(
            dut_type("TYPE ST_X :\nSTRUCT\n a : INT;\nEND_STRUCT\nEND_TYPE").unwrap(),
            "Structure"
        );
        assert_eq!(
            dut_type("TYPE ST_Y EXTENDS ST_X : STRUCT END_STRUCT END_TYPE").unwrap(),
            "Structure"
        );
        assert_eq!(
            dut_type("TYPE U : UNION a : INT; END_UNION END_TYPE").unwrap(),
            "Union"
        );
        assert_eq!(
            dut_type("TYPE T_Name : STRING(20); END_TYPE").unwrap(),
            "Alias"
        );
        assert!(dut_type("VAR_GLOBAL END_VAR").is_err());
    }

    #[test]
    fn answers() {
        assert_eq!(
            parse_installed(
                "Installed: True\nDevice: CODESYS Control for Linux SL | vendor CODESYS | type 4102 id 0000 0005 version 4.20.0.0"
            )
            .as_deref(),
            Some("CODESYS Control for Linux SL")
        );
        assert_eq!(parse_installed("Installed: False"), None);
        assert_eq!(
            parse_ident("Device: Device\ntype: 4102\nid: 0000 0005\nversion: 4.20.0.0\n"),
            Some(DeviceId {
                ty: 4102,
                id: "0000 0005".into(),
                version: "4.20.0.0".into()
            })
        );
        let subs = parse_sub_elements(
            "Identification = {..}\nSubElements: 2\n  [0] StationName = 'a'\n  [1] IPAddress = [1, 2, 3, 4]",
        );
        assert_eq!(subs.len(), 2);
        assert_eq!(member_index(&subs, "Identification.IPAddress"), Some(1));
        assert_eq!(member_index(&subs, "StationName"), Some(0));
        assert_eq!(member_index(&subs, "Other"), None);
        assert_eq!(
            parse_params(
                "Device parameters (scope\tid\tname\tvalue\tunit):\ndevice\t0\tIPAddress\t[192, 168, 100, 10]\t\n"
            ),
            [(0, "IPAddress".into(), "[192, 168, 100, 10]".into())]
        );
        assert_eq!(
            parse_libraries(
                "Application (libman: Library Manager) - 2 reference(s)\n  - Standard, * (System) [placeholder]\n  - IoStandard [system]\n"
            ),
            ["Standard", "IoStandard"]
        );
        // The real answer of 0.19.0 (placeholders with '#', resolved or not).
        assert_eq!(
            parse_libraries(
                "Application (libman: Library Manager) \u{2014} 3 reference(s)\n  - #IoStandard [system, placeholder]\n  - #CommFB [system, placeholder] ns=CommFB -> CommFB, 3.5.17.10 (3S)\n  - #Standard [placeholder, redirected] ns=Standard -> Standard, 3.5.22.0 (System)\n"
            ),
            ["IoStandard", "CommFB", "Standard"]
        );
        let code = parse_code(
            "2 object(s) with code:\n\n=== Device/Plc Logic/Application/PLC_PRG (Program) ===\n// ----- Declaration -----\nPROGRAM PLC_PRG\nVAR\nEND_VAR\n// ----- Implementation -----\nx := 1;\n\n=== Device/Plc Logic/Application/GVL (GVL) ===\n// ----- Declaration -----\nVAR_GLOBAL\nEND_VAR",
        );
        assert_eq!(code.len(), 2);
        assert_eq!(code[0].0, "PLC_PRG");
        assert_eq!(norm_text(code[0].2.as_deref().unwrap()), "x := 1;");
        assert_eq!(code[1].2, None);
        assert_eq!(
            parse_tasks(
                "Task Configuration: Task Configuration\n\nTask: MainTask\n  Props: priority=1\n  POU calls (1): PLC_PRG\n\nTask: Profinet_IOTask\n  POU calls (0): (none)\n"
            ),
            [
                ("MainTask".to_string(), vec!["PLC_PRG".to_string()]),
                ("Profinet_IOTask".to_string(), vec![])
            ]
        );
        assert_eq!(
            parse_compile(
                "Compilation complete.\n1 error(s), 1 warning(s).\n\nErrors:\nERROR: C0077: Unknown type [FB_Cell:3]\n\nWarnings:\nWARNING: unused\n"
            ),
            (
                vec!["C0077: Unknown type [FB_Cell:3]".to_string()],
                vec!["unused".to_string()]
            )
        );
        assert_eq!(
            parse_state("Application: Application\nState: run\nLogged In: True").as_deref(),
            Some("run")
        );
        assert_eq!(parse_state("State: Unknown"), None);
        assert_eq!(
            parse_values("A.x = 16#8024\nB.y = TRUE\n"),
            [
                ("A.x".to_string(), "16#8024".to_string()),
                ("B.y".to_string(), "TRUE".to_string())
            ]
        );
    }

    #[test]
    fn module_actions() {
        let st = "- Unnamed (ExtendedObject[IScriptProject])\n  - Device (ExtendedObject[IScriptObject]) {d}\n    - Plc Logic (X) {p}\n      - Application (X) {a}\n    - Ethernet (X) {e}\n      - PN_Controller (X) {c}\n        - kuka_robi1 (X) {k}\n          - _64_safe_digital_in_and_outputs (X) {m1}\n          - <Empty> (X) {m2}\n        - kuka_robi2 (X) {k2}\n";
        assert_eq!(
            children(st, "Device/Ethernet").unwrap(),
            ["PN_Controller".to_string()]
        );
        assert_eq!(children(st, "Device/Nope"), None);
        let m = |slot: u32, name: &str, dev: &str| Module {
            device: dev.into(),
            slot,
            name: name.into(),
            parent: DeviceId {
                ty: 81,
                id: "x".into(),
                version: "v".into(),
            },
            module_id: "1".into(),
            ident: Some("16#8F".into()),
            add: false,
        };
        let k1 = "Device/Ethernet/PN_Controller/kuka_robi1";
        assert_eq!(
            m(1, "SafeIO", k1).action(st),
            ModuleAction::Rename("_64_safe_digital_in_and_outputs".into())
        );
        assert_eq!(m(2, "VASS", k1).action(st), ModuleAction::Plug);
        assert_eq!(
            m(1, "_64_safe_digital_in_and_outputs", k1).action(st),
            ModuleAction::Present
        );
        assert_eq!(
            m(1, "SafeIO", "Device/Ethernet/PN_Controller/kuka_robi2").action(st),
            ModuleAction::FreeList(0)
        );
        assert_eq!(
            m(1, "SafeIO", "Device/X").action(st),
            ModuleAction::NoDevice
        );
        let o = Options::default();
        let calls = m(1, "SafeIO", k1).calls(&ModuleAction::Rename("old".into()), &o, "p");
        assert_eq!(
            calls.iter().map(|c| c.tool).collect::<Vec<_>>(),
            ["rename_object", "list_device_parameters"]
        );
        assert_eq!(calls[0].args["objectPath"], format!("{k1}/old"));
        assert!(
            m(1, "SafeIO", k1)
                .calls(&ModuleAction::FreeList(0), &o, "p")
                .is_empty()
        );
        // A server with add_device(moduleId) adds it to the free child list.
        let add = Module {
            add: true,
            ..m(1, "SafeIO", k1)
        };
        let calls = add.calls(&ModuleAction::FreeList(0), &o, "p");
        assert_eq!(
            calls.iter().map(|c| c.tool).collect::<Vec<_>>(),
            ["add_device", "list_device_parameters"]
        );
        assert_eq!(calls[0].args["parentPath"], k1);
        assert_eq!(calls[0].args["deviceName"], "SafeIO");
        assert_eq!(calls[0].args["moduleId"], "1");
        assert_eq!(calls[0].args["targetVersion"], "v");
        assert_eq!(
            calls[0].deferred,
            [("targetDeviceName", Deferred::DeviceName(add.parent.clone()))]
        );
        assert!(add.calls(&ModuleAction::NoDevice, &o, "p").is_empty());
    }

    #[test]
    fn caps_from_the_input_schemas() {
        use crate::mcp::ToolInfo;
        let t = |n: &str, a: &[&str]| ToolInfo {
            name: n.into(),
            args: a.iter().map(|x| x.to_string()).collect(),
        };
        let plain = [
            t("add_device", &["parentPath", "deviceName"]),
            t("set_device_parameter", &["parameterId", "elementIndex"]),
            t("get_device_parameter", &["parameterId"]),
        ];
        assert_eq!(Caps::from_tools(&plain), Caps::default());
        assert_eq!(Caps::default().missing().len(), 4);
        let ours = [
            t("add_device", &["parentPath", "moduleId"]),
            t("set_device_parameter", &["elementIndex", "elementPath"]),
            t("get_device_parameter", &["elementPath"]),
            t(
                EXPORT_LIBRARIES,
                &["projectFilePath", "outDir", "includeData"],
            ),
            t(INSTALL_LIBRARIES, &["dir"]),
        ];
        assert_eq!(Caps::from_tools(&ours), Caps::ALL);
        assert!(Caps::ALL.missing().is_empty());
        // elementPath on one of the two is not enough.
        assert!(!Caps::from_tools(&ours[..2]).element_path);
        assert!(!Caps::from_tools(&ours[..3]).export_libraries);
        assert!(!Caps::from_tools(&ours[..4]).install_libraries);
    }

    #[test]
    fn build_failures_at_login_are_told_from_connection_errors() {
        // CODESYS SP22 (launch 15): the login rebuilds Device.Application.
        for t in [
            "Operation failed. Output:\nLogin to the PLC failed: Exception: Compile errors occurred. Fix them before logging in. (login(OnlineChangeOption.Try, True))",
            "Operation failed. Output:\nBuild errors in Device.Application",
            "Operation failed. Output:\nLibrary 'IoStandard, 3.5.17.0 (System)' could not be found",
            "Error: the library CommFB is missing in the repository",
            "Unresolved library reference #IoStandard",
        ] {
            assert!(build_failure(t), "{t}");
        }
        for t in [
            "Operation failed. Output:\nUnable to connect to the device: no answer from 192.168.100.10",
            "Gateway not configured properly",
            "Operation failed. Output:\nInvalid user authentication on the target",
            "Command connect_to_device timed out after 180000ms",
            "TypeError: login() takes exactly 2 arguments (3 given)",
        ] {
            assert!(!build_failure(t), "{t}");
        }
    }

    #[test]
    fn auto_compile_with_only_library_open_errors() {
        // The live answer (2026-11-18, demo_body_weld_2r).
        let live = "Signature PLC_PRG bulk access set to ReadOnly.\n\n[auto-compile FAILED] \
            Compilation complete for C:\\ProgramData\\plan2cell\\projects\\demo_body_weld_2r\\\
            HeadPLC_demo_body_weld_2r.project.\n3 error(s), 0 warning(s).\n\nErrors:\n\
            ERROR: Could not open library '#CmpUserMgr Implementation'. (Reason: The library \
            'CmpUserMgr Implementation, 3.5.21.0 (System)' has not been installed to the system.)\n\
            ERROR: Could not open library '#CmpVisuHandler'. (Reason: The library 'CmpVisuHandler, \
            3.5.19.0 (System)' has not been installed to the system.)\n\
            ERROR: Could not open library '#SysTask'. (Reason: The library 'SysTask, 3.5.17.0 \
            (System)' has not been installed to the system.)";
        let e = auto_compile_library_errors(live).unwrap();
        assert_eq!(e.len(), 3);
        assert!(e[0].starts_with("Could not open library '#CmpUserMgr Implementation'"));
        // A real error with them: the project's error.
        let real = format!("{live}\nERROR: C0046: Identifier x not defined [PLC_PRG:3]")
            .replace("3 error(s)", "4 error(s)");
        assert_eq!(auto_compile_library_errors(&real), None);
        // Fewer ERROR lines than the count (an error over several lines): not trusted.
        assert_eq!(
            auto_compile_library_errors(&live.replace("3 error(s)", "4 error(s)")),
            None
        );
        // A build that did not run, a failed edit, a clean auto-compile, a plain build.
        for t in [
            "Bulk set\n\n[auto-compile FAILED] Failed initiating compilation for C:\\p. Output:\nx",
            "Bulk set\n\n[auto-compile FAILED] Compilation initiated for C:\\p. WARNING: Build \
             command reported errors. Use get_compile_messages for details.",
            "Bulk set\n\n[auto-compile] Compilation complete.",
            "Compilation complete.\n1 error(s), 0 warning(s).\n\nErrors:\nERROR: Could not open library '#SysTask'.",
        ] {
            assert_eq!(auto_compile_library_errors(t), None, "{t}");
        }
        let failed_edit = live.replace(
            "Signature PLC_PRG bulk access set to ReadOnly.",
            "Operation failed. Output:\nSignature 'X' not found",
        );
        assert_eq!(auto_compile_library_errors(&failed_edit), None);
        assert!(library_open_only(&e));
        assert!(!library_open_only(&[]));
    }

    #[test]
    fn library_answers() {
        // The overlay's texts (install_libraries.py, server.ts libJsonResponse).
        assert!(no_libraries_json(
            "Operation failed. Output:\nDEBUG: install_libraries: Dir='C:\\p\\libraries'\nError installing the libraries of C:\\p\\libraries: NO_LIBRARIES_JSON: C:\\p\\libraries\\libraries.json not found.\nTraceback ...\nSCRIPT_ERROR: ..."
        ));
        // A library's error naming libraries.json is no missing list.
        assert!(!no_libraries_json(
            "Libraries of C:\\p\\libraries: 0 installed, 1 already installed, 1 error(s) (repository System).\nERROR: IoStandard, 3.5.17.0 (System): sha256 ab, libraries.json says cd\n### INSTALL_LIBRARIES_START ###\n{\"errors\": [{\"library\": \"x\", \"error\": \"file y not found\"}]}\n### INSTALL_LIBRARIES_END ###"
        ));
        assert!(!no_libraries_json(
            "Operation failed. Output:\nNo IDE library manager with install_library"
        ));
        let (unresolved, errors) = unresolved_libraries(
            "Application (libman: Library Manager) \u{2014} 3 reference(s)\n  - #IoStandard [system, placeholder]\n  - #CommFB [system, placeholder] ns=CommFB -> CommFB, 3.5.17.10 (3S)\n  - Standard, * (System)\nERROR: libman: repository unreadable\n",
        );
        assert_eq!(unresolved, ["IoStandard"]);
        assert_eq!(errors, ["libman: repository unreadable"]);
        let libs = exported_libraries(
            "Exported 2 library file(s) of C:\\p\\head.project to C:\\p\\libraries (libraries.json), 0 error(s).\n### EXPORT_LIBRARIES_START ###\n{\"project\": \"C:\\\\p\\\\head.project\", \"outDir\": \"C:\\\\p\\\\libraries\", \"manifestCount\": 3, \"removed\": [], \"count\": 2, \"libraries\": [{\"name\": \"IoStandard\", \"vendor\": \"System\", \"version\": \"3.5.17.0\", \"file\": \"IoStandard_3.5.17.0_System.compiled-library\", \"sha256\": \"ab\", \"source\": \"C:\\\\r\\\\x\", \"data\": \"Zm9v\"}, {\"name\": \"CommFB\", \"vendor\": \"3S\", \"version\": \"3.5.17.10\", \"file\": \"x\", \"sha256\": \"cd\", \"dataOmitted\": \"9000000 bytes, over the answer's budget of 16777216 bytes\"}], \"errors\": []}\n### EXPORT_LIBRARIES_END ###",
        )
        .unwrap();
        assert_eq!(libs.len(), 2);
        assert_eq!(libs[0].name, "IoStandard");
        assert_eq!(libs[0].data.as_deref(), Some("Zm9v"));
        assert_eq!(libs[1].data, None);
        assert!(libs[1].data_omitted.as_deref().unwrap().contains("budget"));
        // The written list has no file data.
        let written = serde_json::to_string(&libs).unwrap();
        assert!(
            !written.contains("Zm9v") && !written.contains("budget"),
            "{written}"
        );
        // Without the markers: a bare JSON array, or an object with `libraries`.
        assert_eq!(
            exported_libraries("[{\"name\": \"A\"}]").unwrap()[0].name,
            "A"
        );
        assert_eq!(
            exported_libraries("{\"libraries\": [{\"name\": \"B\"}]}").unwrap()[0].name,
            "B"
        );
        assert_eq!(exported_libraries("Exported."), None);
    }

    #[test]
    fn codesys_states() {
        let two = "SP21 Patch 5\n  Exe: C:\\a\\CODESYS.exe\n  State: stopped\n  Mode: persistent\n\nSP22 Patch 1  <- current\n  Exe: C:\\b\\CODESYS.exe\n  State: ready\n  Mode: persistent\n  PID: 4242";
        assert_eq!(codesys_state(two).as_deref(), Some("ready"));
        assert_eq!(
            codesys_state("SP22 Patch 1\n  State: launching").as_deref(),
            Some("launching")
        );
        assert_eq!(codesys_state("nothing"), None);
        assert!(codesys_healthy("ready") && codesys_healthy("running"));
        assert!(!codesys_healthy("stopped") && !codesys_healthy("error"));
    }

    #[test]
    fn libraries_before_the_first_login() {
        let m = crate::project::Manifest::parse(
            &std::fs::read_to_string(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../examples/welding_cell_2_small/plc/project.json"),
            )
            .unwrap(),
        )
        .unwrap();
        let p = &m.projects[0];
        let o = Options::default();
        let tools = |v: &[Call]| v.iter().map(|c| c.tool).collect::<Vec<_>>();
        // A server without install_libraries: the CODESYS Store, as `projects` does.
        let v = online(p, &o);
        assert_eq!(
            tools(&v[..3]),
            [
                "download_missing_libraries",
                "list_project_libraries",
                "compile_project"
            ]
        );
        assert_eq!(v[2].check, Check::BuildClean);
        assert_eq!(&tools(&v[3..]), &tools(&login(p, &o)));
        let o = Options {
            caps: Caps::ALL,
            ..Options::default()
        };
        let v = libraries_ready(p, &o);
        assert_eq!(
            tools(&v),
            [
                INSTALL_LIBRARIES,
                "list_project_libraries",
                "compile_project"
            ]
        );
        assert_eq!(
            v[0].args["dir"],
            r"C:\ProgramData\plan2cell\projects\cell\libraries"
        );
        assert_eq!(v[0].check, Check::InstallLibraries);
        // reset-ide: shutdown without saving, launch, status.
        let r = reset_ide(&o);
        assert_eq!(
            tools(&r),
            ["shutdown_codesys", "launch_codesys", "get_codesys_status"]
        );
        assert_eq!(r[0].args["unsavedChanges"], "discard");
        assert_eq!(r[2].check, Check::Status);
    }

    #[test]
    fn members_by_visible_name() {
        let robot = parse_sub_elements(
            "SubElements: 4\n  [0] IP address = [0,0,0,0]\n  [1] Subnet mask = [0,0,0,0]\n  [2] Default gateway = [0,0,0,0]\n  [3] Station name = 'PN-Device'",
        );
        assert_eq!(member_index(&robot, "StationName"), Some(3));
        assert_eq!(nested_member(&robot, "StationName"), None);
        let head = parse_sub_elements(
            "SubElements: 3\n  [0] DeviceId = 16#0301\n  [1] VendorId = 16#019B\n  [2] Identification = {[192,168,100,2], [255,255,255,0], [0,0,0,0], [16#00,16#00,16#00,16#00,16#00,16#00], 'kuka-noname'}",
        );
        assert_eq!(member_index(&head, "Identification.StationName"), None);
        assert_eq!(
            nested_member(&head, "Identification.StationName"),
            Some("Identification")
        );
    }

    #[test]
    fn csv_rows_of_absent_modules() {
        let csv = "//CoDeSys Mapping Export V1.2\n//x\nApplication.GVL_IO.a,8 safe digital inputs,,d,,SafeIO_FDIO64\nApplication.GVL_IO.b,ch,,d,,VASS_DIO256\n";
        let (t, n) = drop_rows(csv, &["SafeIO_FDIO64".to_string()]);
        assert_eq!(n, 1);
        assert_eq!(
            t,
            "//CoDeSys Mapping Export V1.2\n//x\nApplication.GVL_IO.b,ch,,d,,VASS_DIO256\n"
        );
        assert_eq!(drop_rows(csv, &[]), (csv.to_string(), 0));
    }

    #[test]
    fn parameter_values_compare_loosely() {
        assert_eq!(
            norm_value("[192, 168, 100, 10]"),
            norm_value("[192,168,100,10]")
        );
        assert_eq!(
            norm_value("{192,168,100,10}"),
            norm_value("[192,168,100,10]")
        );
        assert_eq!(norm_value("'eth1'"), norm_value("eth1"));
        assert_eq!(norm_value("\"eth1\""), "eth1");
        assert_ne!(norm_value("1"), norm_value("0"));
    }

    #[test]
    fn conditions() {
        let tasks = vec![("MainTask".to_string(), vec!["PLC_PRG".to_string()])];
        assert!(!When::TaskMissing("MainTask".into()).holds(&tasks));
        assert!(When::TaskMissing("Profinet_IOTask".into()).holds(&tasks));
        assert!(
            !When::PouNotInTask {
                task: "MainTask".into(),
                pou: "PLC_PRG".into()
            }
            .holds(&tasks)
        );
        assert!(
            When::PouNotInTask {
                task: "Other".into(),
                pou: "PLC_PRG".into()
            }
            .holds(&tasks)
        );
    }

    #[test]
    fn bind_uses_the_mapped_address() {
        let o = Options::default();
        assert_eq!(o.reach("192.168.100.10"), "192.168.100.10");
        let o = Options {
            address_map: [("192.168.100.10".to_string(), "172.31.5.11".to_string())].into(),
            ..Options::default()
        };
        assert_eq!(o.reach("192.168.100.10"), "172.31.5.11");
        assert_eq!(o.reach("192.168.100.20"), "192.168.100.20");
    }

    #[test]
    fn shown_calls_hide_secrets_and_code() {
        let o = Options {
            credentials: Some(("admin".into(), "hunter2-secret".into())),
            ..Options::default()
        };
        let c = Call::new(
            "set_pou_code",
            &o,
            Some(r"C:\p\x.project"),
            json!({"pouPath": "Application/X", "declarationCode": "A\nB\nC"}),
        );
        assert_eq!(
            c.show(),
            r#"set_pou_code(declarationCode=<3 lines>, install="SP22 Patch 1", pouPath="Application/X", projectFilePath="C:\\p\\x.project")"#
        );
        let c = connect(&o, Some("p"));
        assert!(!c.show().contains("hunter2"), "{}", c.show());
        assert!(c.show().contains("devicePassword=<secret>"));
    }
}
