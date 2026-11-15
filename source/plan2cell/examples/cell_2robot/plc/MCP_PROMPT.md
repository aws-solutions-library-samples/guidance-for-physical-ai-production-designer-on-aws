# Task: build the CODESYS projects for cell `cell_2robot`

You are an automation engineer agent with access to a CODESYS MCP server (CODESYS CODESYS V3.5 SP22 Patch 1 on Windows). Build 2 CODESYS project(s) - `HeadPLC_cell_2robot`, `RobotSim_cell_2robot_R1` - exactly as described in `project.json` (in this directory). All PLC code is already written and was type-checked with the trust-platform compiler; your job is project creation, hardware configuration, code insertion, IO mapping, tasks and a clean build. Do **not** modify any existing CODESYS project; create new project files.

## Inputs

- `project.json`: device trees with exact DeviceIDs `(type, id, version)` and parameter ids, IO mappings, tasks, libraries, objects in creation order, steps, acceptance criteria. **It is the source of truth.**
- `codesys/head/` and `codesys/robots/<ns>/`: one file per object: `DUT/<Name>.st` and `GVL/<Name>.st` (declaration only), `POU/<Name>.st` (declaration) + `POU/<Name>_Body.st` (implementation).
- `codesys/*/io_mapping.csv`: fallback IO mapping import (CoDeSys Mapping Export V1.2).
- `PROJECT.md`: the same information as tables, for humans.

## 0. Before you start

1. List the MCP server's tools and read their parameter schemas. Never guess tool or parameter names.
2. Confirm the CODESYS version and that the scripting engine is available.
3. Check the prerequisites below are installed in the device repository (search by the DeviceIDs). If the PN device description (type 83) is missing, STOP and ask the human to run the manual step.

- `gsdml`: profinet-headplc-robot-simulation/codesys-project/kuka-vass-device-simulator/gsdml/GSDML-V2.25-KUKA-Roboter-GmbH-VKR C4-Device-V1.0-20121030.xml -> `(81, 'GSDML-019B-0301-V2.25-KUKA-Roboter-GmbH-VKR C4-Device-V1.0-20121030 - DIM 2', 'V8.2 - HW 1')`
- `pn_device_desc`: Tools > Miscellaneous > Generate PROFINET Device from GSDML (select the KUKA GSDML, DAP 'DIM 2' VKRC nexxt Device V8.2); not scriptable -> `(83, '0x019B_0x0301_DIM 2', '1 V8.2')`

## 1. Capability mapping

| Step | Codesys-MCP-Master (phobicdotno) | luke-harriman Codesys-MCP | codesys-mcp-toolkit (johannesPettersson80) |
|---|---|---|---|
| create project | `create_project` (device by repository name) | `create_project` | `create_project` (Standard template only) |
| device tree | `add_device`, `plug_device` | `add_device` | HUMAN |
| device parameters | `set_device_parameter` (+ read back) | `set_device_parameter` (experimental) / `eval_python` | HUMAN |
| libraries | `add_library` | `add_library` | HUMAN |
| DUT / GVL | `create_dut`, `create_gvl` + set declaration | `create_dut`, `create_gvl` | HUMAN (or PLCopen import) |
| POU code | `create_pou` + `set_pou_code` | `create_pou` + `set_pou_code` | `create_pou` + `set_pou_code(declarationCode, implementationCode)` |
| tasks | `create_task`, `configure_task`, `add_pou_to_task` | `eval_python` | HUMAN |
| IO mapping | per channel or `import_io_mappings_csv` | `map_io_channel` | HUMAN |
| build | `compile_project` + messages | `compile_project`, `get_compile_messages` | `compile_project` |

Prefer Codesys-MCP-Master. Where a step is HUMAN for your server, collect the exact instructions from `PROJECT.md` and hand them to the human in your final report.

## 2. Procedure (per project in `projects[]`, head PLC first)

1. **Create** the project `<name>.project` with device `CODESYS Control for Linux SL` `(4102, '0000 0005', '4.20.0.0')`, ST, Standard template. Delete nothing else the template creates except replacing `PLC_PRG` (see 4).
2. **Device tree** in `device_tree` order: add each child under its parent with the exact DeviceID; use the node `name` as the object name (it is referenced by IO mapping and tasks, e.g. `PN_Controller.CommCycle`). For PROFINET devices, plug/ensure the modules in `slots[]` (slot number, module id, object name). Set every `parameters[]` entry by id; `member` is the path of a sub-element inside a struct parameter, with `.` separating nesting levels (e.g. parameter 512, member `Identification.StationName`). Read every parameter back and compare. Checkpoint: list the device tree.
3. **Libraries**: add those with `mode: ensure`; verify that the `auto` ones are present after adding the devices.
4. **Objects** in `objects[]` order under `Application`: DUTs and GVLs get the declaration file content as their only text. POUs: create with the given `pou_type` and language ST, then set the declaration part to the `declaration` file content and the implementation part to the `implementation` file content - byte-exact, both parts in one call if possible (some servers clear an omitted part). `PLC_PRG` already exists: replace both parts. Checkpoint: list the objects.
5. **Tasks**: `tasks[]`. `MainTask` cyclic with the given interval/priority and only `PLC_PRG`. The PROFINET tasks are normally created by the PN stack: verify interval, priority and POU; create them only if missing. Set the bus cycle task as given.
6. **IO mapping**: for every `slots[].io` entry map the channel (`param_id`, `channel`, `sub`) to `variable` ('Map to existing variable'). If your server cannot do this per channel, import `io_mapping_csv`. Read back and compare.
7. **Build**: 0 errors required. On errors, report the message list verbatim; do not edit generated code except for an obvious, minimal dialect fix, which you must report.
8. **Save** and continue with the next project.

## 3. ScriptEngine fallback (IronPython 2.7, e.g. via `eval_python`) - verify each call against your CODESYS version

```python
proj = projects.primary
dev = proj.find('Device', False)[0]
dev.add('Ethernet', 110, '0000 0002', '4.2.0.0')
eth = proj.find('Ethernet', True)[0]
eth.add('PN_Controller', 80, '0000 0002', '4.7.1.0')
pnc = proj.find('PN_Controller', True)[0]
pnc.add('kuka_robi1', 81, 'GSDML-019B-0301-V2.25-KUKA-Roboter-GmbH-VKR C4-Device-V1.0-20121030 - DIM 2', 'V8.2 - HW 1')
# modules: plug into the empty slot children of the KUKA device (module id is the last argument)
# parameters: for c in node.connectors: for p in c.host_parameters: if p.id == 512: ...
# io mapping: p.io_mapping.variable = 'Application.GVL_IO.aVassInputData_R1'
app = proj.find('Application', True)[0]
gvl = app.create_gvl('GVL_Cell'); gvl.textual_declaration.replace(open(r'...GVL_Cell.st').read())
fb = app.create_pou('FB_Cell', PouType.FunctionBlock)
fb.textual_declaration.replace(DECL); fb.textual_implementation.replace(BODY)
app.build()
```

## 4. Acceptance and report

For each project: build with 0 errors, all `acceptance.objects` present, all `acceptance.devices` present with the parameter values read back. Finish with a JSON report:

```json
{"projects": [{"name": "...", "file": "...", "built": true, "errors": [], "warnings": [], "deviations": [], "human_steps": []}]}
```

## 5. Context you may need

- Coordination: each robot runs its whole op list in production Folge 11 and requests interlock zones (REQUEST bits, robot -> PLC bytes 28-31) before ops that depend on other robots; the head PLC grants them (GRANT bits, PLC -> robot bytes 12-15) in the planned ticket order. Op index: robot -> PLC bytes 4-5. Plan id: PLC -> robot bytes 4-5.
- Zones:

No cross-robot dependencies: no interlock zones.
- Simulation only: zones are logical, not safety-rated.
