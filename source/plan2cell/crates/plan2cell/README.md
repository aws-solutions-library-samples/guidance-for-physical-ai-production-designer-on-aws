# plan2cell (library)

The combined pipeline behind the `plan2cell` command and the MCP server: one behaviour-tree plan in,
the head-PLC code ([`bt2plc`](../bt2plc/README.md)) and the KUKA robot programs
([`bt2krl`](../bt2krl/README.md)) out, built in one pass so they agree.

```rust
use plan2cell::{BuildConfig, BuildOptions, PlanInput, Profile};

let input = PlanInput::load("test_data/bt_1790934549_welding_cell_2_small.json".as_ref())?;
let cfg = BuildConfig::load("examples/plan2cell.toml".as_ref())?;
let opts = BuildOptions { profile: Profile::Sim, ..cfg.options() };
let build = plan2cell::build(&input, &cfg, &opts)?;   // everything in memory
build.write_to("out/welding".as_ref(), false)?;    // plc/, krl/, BUILD.md, plan2cell-build.json
```

- [`effective`](src/lib.rs) applies the options to both tool configs (profile, robot simulators,
  the VW job-number / zone / PLAN_ID-byte rules, the shared robot model) and reports what it
  adjusted.
- [`plan`](src/lib.rs) plans without generating code; [`explain_robot`](src/lib.rs) prints one
  robot's KRL plan.
- [`sim::simulate`](src/sim.rs) (feature `sim`) co-simulates a build: the generated head-PLC ST
  against the device-simulator ST or the executor of the generated KRL.
- [`BuildConfig`](src/config.rs) is `plan2cell.toml`: `[build]` defaults, `[plc]` = bt2plc.toml,
  `[krl]` = bt2krl.toml.

Errors keep the tools' exit codes (`Error::exit_code`). The usage is described in the
[repository README](../../README.md).
