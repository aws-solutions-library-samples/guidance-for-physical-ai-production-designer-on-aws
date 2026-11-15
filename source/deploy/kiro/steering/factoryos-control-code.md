# Physical AI Production Designer: PLC and KRL code (codegen_*, process_*, plan2cell_*)

* The export (codegen_build: plan, profile sim | vw-ks24, PLC dialect) generates the head PLC's code (bt2plc) and each
  robot's KRL (bt2krl) from a plan, with the cell's Hardware Configuration (hardware_get) and process definition.
  Follow it with codegen_builds / codegen_build_info; read files with codegen_file; plan2cell_explain_robot,
  plan2cell_check, plan2cell_simulate explain and check an export in depth.
* The process definition (process_get; plan2cell/process.yaml in the cell) holds the IOs:
  * telegram: where the PLC <-> robot handshake sits ($IN/$OUT in_base / out_base; its 256-bit layout is fixed);
  * processes: per process the KRL technology call (vw-ks24 weld_call, e.g. SEL_RES=SELECT(#MAKRO,580,…)) and the op type
    it applies to, and the parameter defaults;
  * robots: tool_no / base_no, and the robot's own signals (gun close / open and their feedback, weld controller program
    and done, tip dresser, tool changer, clamps): name, dir in / out, index, width, what.
  No definition yet: process_init profile <sim | vw-ks24> robots [...] writes a commented sample. Edit, then process_save
  (validated: signals outside the telegram, no overlaps, KRL names): merged into plan2cell.toml (backup kept), the
  signals written as KRL declarations plan2cell/krl/<ns>_process_signals.dat. Then export again.
* For more control over the robot code, suggest the IDE route: open the cell's plan2cell/process.yaml and the export's
  krl/ files in the workspace IDE and let Kiro write or change the KRL there (gun sequences, tip dressing, tool change,
  error handling), with process.yaml as the only source of IO numbers (use the SIGNAL names of
  <ns>_process_signals.dat, never raw $OUT numbers). After editing process.yaml in the IDE, process_save with no
  arguments applies the file as saved. Every export regenerates its files: keep hand-written KRL in modules of its own (a
  change to a generated file is gone with the next export).
