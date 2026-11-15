# bt2krl user guide

This guide takes a behaviour-tree plan all the way to KUKA robot programs and a matching head PLC:
which commands to run, which settings matter, what to install on the controller, and what you
must still check by hand before the first automatic run.

The specification is [`../TASK.md`](../TASK.md). How the generator works inside is in
[`design.md`](design.md).

> **Not tested on a controller.** No local tool compiles KRL. Everything bt2krl writes passes its
> own strict validator, an IR round trip and (both profiles) a co-simulation against the
> generated head PLC. The final check is still a real KSS 8.x controller; see
> [Before the first automatic run](#before-the-first-automatic-run).

## 1. Pick a profile

bt2plc (the head PLC) and bt2krl (the robots) must run with the same profile.

| | `sim` | `vw-ks24` |
|---|---|---|
| Robots | docker device simulators, KUKA OfficeLite, a test robot **without** the VW package | real VW KS24 robots (KSS V8.2.29, TPVW 8.1.8) |
| Who does the Folge handshake | bt2krl's `bt_cell` + `bt_sps` | the VW tech package (`cell.src`, `AUE_WAIT`/`AUE_END`) |
| Zones | REQUEST/GRANT bits, 4-phase, up to 31 zones + permit bit | VW robot interlocks A41-56 / A81-96, at most 16, no permit bit |
| Homing | blind HOME Folge | manual (VW `cell.src` starts a Folge only at home) |
| PLAN_ID check | always | optional, on a free telegram byte |

**Never install `sim` programs on a VW KS24 robot.** Every field of the simulator telegram collides
with signals the VW programs already use (Fertigmeldung, interlocks, `PRG_NR`, error code).

## 2. Generate the head PLC and the robot programs

The simplest way is one combined build: the head PLC and every robot program from one run, with
the profile, the PLAN_ID byte and the interlock map applied to both sides. From the repository
root:

```sh
cargo run --release -p plan2cell-cli -- config > plan2cell.toml   # [build], [plc], [krl]
cargo run --release -p plan2cell-cli -- build <plan.json> -c plan2cell.toml -o out/cell --profile sim
```

`out/cell/plc/` is the head PLC (bt2plc), `out/cell/krl/<ns>/` the robot programs (bt2krl),
`out/cell/BUILD.md` what was built. In `plan2cell.toml` the `[plc]` section is exactly
`bt2plc.toml` and `[krl]` exactly `bt2krl.toml`; the tables below name their keys without that
prefix. The same is available as MCP tools (`plan2cell mcp`, see `docs/MCP.md` at the repository
root).

The two tools can also run separately:

```sh
cargo run --release -p bt2plc -- generate <plan.json> -o out/cell -c bt2plc.toml
```

For `sim` the defaults work. For `vw-ks24` the head needs at least:

```toml
[cell]
job_number = "byte"            # PRG_NR is 8 bits ($IN[1..8])

[protocol]
profile = "vw-ks24"
max_zones = 16                 # the VW standard has 16 robot interlocks and no permit bit
plan_id_byte = 12              # optional: a free PLC -> robot byte for the PLAN_ID check (0 = off)

[[robot]]
ns = "robot1"
interlocks = { IL01 = 1, IL02 = 11 }   # zone -> VW interlock index (1..16)

[[robot]]
ns = "robot2"
interlocks = { IL01 = 1, IL02 = 11 }   # both partners of a zone use the SAME index
```

`bt2plc zones <plan.json>` lists the zones (`IL01`, `IL02`, ...) the plan needs. Without
`interlocks`, each robot's zones are numbered 1, 2, ... in zone order. The map is written into
every `robots/<ns>/program.json`, so bt2krl uses the same numbers.

Optional per robot in bt2plc: `model = "KR240R2900_ULTRA"` (otherwise bt2krl must be told the
model) and `joint_map = "identity" | "kr240_r2900_ultra" | ...` for the URDF -> KUKA mapping.

## 3. Generate the robot programs separately (bt2krl)

```sh
cargo run --release -p bt2krl -- config > bt2krl.toml        # every default, commented
# edit bt2krl.toml: one [[robot]] per robot, e_axes is required
cargo run --release -p bt2krl -- generate out/cell -o out/krl -c bt2krl.toml --profile sim
cargo run --release -p bt2krl -- check out/krl               # re-validate later (validator + file hashes)
cargo run --release -p bt2krl -- explain out/cell/robots/robot1/program.json   # op / zone / IO plan
```

With `plan2cell build` the same checks run, and `plan2cell check` / `plan2cell explain` work on the
plan directly.

Generation refuses, rather than emitting something unsafe, when:

| Exit | Cause | What to do |
|---|---|---|
| 1 | a waypoint or the home pose is outside the model's software limits | check the joint mapping (section 6) and the model |
| 1 | `home_pose_deg` differs from the configured `$H_POS` by more than `h_pos_tol_deg` | fix the plan's home pose or `h_pos_deg` |
| 1 | an op's estimated time exceeds the head watchdog `limit_s` | the speed limits are too low for this robot model |
| 1 | `program.json` and `cell_plan.json` disagree on PLAN_ID | regenerate with bt2plc (stale output) |
| 1 | `vw-ks24`: the plan needs the permit bit, or has a blind `MoveHome` | reduce the cross-robot dependencies, plan a path home, or use `sim` |
| 2 | `--motion spline` | not available (unverified on KSS 8.2); use `ptp` |
| 3 | `e_axes` not declared, unknown model, interlock map conflict | fix `bt2krl.toml` |
| 4 | output directory not empty | add `--force` (stale files of the previous run are removed) |
| 6 | `--strict` and a warning | read the warning |
| 7 / 8 | round trip / validator failure | a bt2krl bug: please report it with the input |

## 4. bt2krl.toml reference

Every field has a default except `[[robot]] e_axes`. Unknown keys are rejected.

### `[[robot]]` (one per robot)

| Key | Default | Meaning |
|---|---|---|
| `ns` | — | robot namespace as in the plan (`robot1`) |
| `e_axes` | **required** | `"none"`: the robot has no external axes, points are `AXIS` (A1-A6). `"fixed"`: points are `E6AXIS` with `e_values`; **the external axes move to those values**. `"keep"` is refused: that a PTP to an `AXIS` target leaves E1 alone is unverified on KSS 8.2. |
| `e_values` | — | E1..E6 for `"fixed"` |
| `model` | program.json `robot.model` | `KR240R2900_ULTRA`, `KR210R2700_EXTRA`, `KR270R2700_ULTRA`, `KR160R1570_NANO` or `unknown` (no limit check, warning) |
| `controller` | `ns` | controller id for headers and INSTALL.md (`000001R01`) |
| `h_pos_deg` / `h_pos_tol_deg` | — / 2.0 | the controller's `$H_POS`; checked against the plan's home pose |
| `tool_no` / `base_no` | 0 / 0 | `BAS(#TOOL,n)` / `BAS(#BASE,n)`; 0 keeps the current one |
| `interlocks` | from program.json | `vw-ks24` only; must agree with bt2plc's map if both are set |

### `[motion]`

| Key | Default | Meaning |
|---|---|---|
| `mode` | `"ptp"` | `"spline"` is refused |
| `decimate_tol_deg` | 0.5 | joint-space decimation; first/last point and phase changes are kept |
| `approx_ptp_pct` | 100 | `$APO.CPTP` of the points inside an op (the last point is an exact stop) |
| `max_vel_pct` / `min_vel_pct` | 100 / 1 | clamp of the per-segment `BAS(#VEL_PTP,v)` |
| `time_margin` | 1.0 | >= 1; larger values make the robot slower than planned |
| `ramp_time_s` | 0.25 | estimate only: acceleration + deceleration per op |

### `[sim]`

| Key | Default | Meaning |
|---|---|---|
| `in_base` / `out_base` | 0 / 0 | telegram byte b, bit i = `$IN/$OUT[base + 8b + i + 1]` |
| `interrupt_no` | 20 | priority of the abort interrupt in `bt_cell` (must be free) |
| `reset_timer` | 20 | `$TIMER[n]` used by `bt_sps` for the 500 ms reset |
| `joint_encoder` | true | publish A1..A6 in bytes 16-27 (monitoring) |

### `[vw_ks24]`

| Key | Default | Meaning |
|---|---|---|
| `step_signal` | `"out:97:16"` | op index, written as `BT_PKTNR` (declared in `bt_config.dat`); the default is the outputs of the VW `P_PktNr_SPS`, which is local to `vw_usr_r` and cannot be written from a Folge |
| `online_stop` | true | partner releases are part of the online driving condition while inside |
| `release_dwell_ms` | 100 | hold the release before the next request, so the head sees it (VW has no 4-phase handshake) |
| `use_makro20` | false | call `MAKRO20` instead of the inline handshake (depends on the cell's macro numbering) |
| `makro0` / `makro50` / `makro15` | true / true / 15 | the VW reset, flag-init and Gesamtfertigmeldung macros (`makro15 = 0`: none) |
| `weld_call` / `weld_op_type` | `""` / `"weld"` | a technology call emitted during the hold of ops of that type |
| `plan_id_byte` | 0 | must match bt2plc's `protocol.plan_id_byte`; valid start bytes (the byte and the next must both be free on the sample robots): 12, 13, 28, 29. Never 4: bytes 4-5 carry the partner releases E41-48 |
| `emit_patches` | true | also write `patches/` with the `cell.src` and `$config.dat` fragments |
| `max_interlocks` | 16 | upper bound of the interlock indices |

### `[output]`

`line_ending = "crlf"` (KSS standard), `program_name_style = "lower"` (`folge11.src`) or `"upper"`.

## 5. Install on the controller

Each robot directory has its own `INSTALL.md` with the exact files, lines and signal tables for
that robot. The short version:

**`sim`**
1. Copy `KRC/R1/Folgen/folge<prod>.*`, `folge<home>.*`, `KRC/R1/Program/bt_cell.*`,
   `KRC/R1/System/bt_sps.src`.
2. Paste `bt_config.dat` into the `USER GLOBALS` fold of `KRC/R1/System/$config.dat`.
3. Add `BT_SPS()` to the `LOOP` of `sps.sub` (fold `USER PLC`).
4. Map Automatic External in the machine data (`$EXT_START`, `$DRIVES_ON`, `$DRIVES_OFF`,
   `$MOVE_ENABLE`, `$CONF_MESS`; INSTALL.md lists suggested inputs) and select `bt_cell` as the
   program EXT starts. Keep the program override at 100 %.

**`vw-ks24`**
1. Copy `KRC/R1/Folgen/folge<prod>.*`.
2. Add `CASE <prod>` / `FOLGE<prod>()` to `SWITCH PGNO` in `cell.src`.
3. Paste `bt_config.dat` into `USER GLOBALS` of `$config.dat`.
   `patches/` has both edits as fragments. They are **not** replacements for the controller's
   files, which carry cell-specific content.
4. Use the head PLC bt2plc generated with `protocol.profile = "vw-ks24"`, not the cell's
   existing VASS interlock blocks: those arbitrate first-come-first-served and do not keep the
   behaviour tree's order.

## 6. Before the first automatic run

These are the points no local check can cover. Go through them on the controller, in T1 first.

1. **The files compile.** Select each Folge; KSS reports syntax and declaration errors at
   selection time.
2. **Joint mapping.** The URDF -> KUKA mapping is not validated for these robots. Jog to the first
   points of the production Folge in T1 and compare them with the planned poses. The limit check
   only catches gross errors.
3. **External axes.** With `e_axes = "none"`, make sure the robot really has none. With
   `"fixed"`, check that the configured E values are safe positions.
4. **Speeds.** Run one op at reduced override and compare its time with the `Motion s` column of
   INSTALL.md. Then restore 100 %: the head's watchdogs assume plan speed.
5. **`sim`: abort.** Stop the cell while a Folge runs: the robot must brake, report ProgRunning
   off and step 0. Then stop the robot interpreter itself (stop key / E-stop) mid-op and check
   that the interrupt fires on restart. Until that is confirmed, recover from an interpreter stop
   by cancelling and re-selecting `bt_cell`, **never by restarting it**: otherwise the robot may
   finish the current op's motion without its grant.
6. **`vw-ks24`: interlocks.** With two robots, check that each waits for its grant and the
   partner release, and that withdrawing a partner release stops the robot on its path.
7. **`vw-ks24`: VW routines.** The Folge calls `VW(#VW_AUE_INIT/VW_INIT/VW_AUE_WAIT/VW_AUE_END)`,
   `VW(#FB_CHECK)` and `SELECT(#MAKRO,...)` from plain KRL. Their signatures were checked against
   all 8 exported robots, their behaviour outside a TPVW inline-form program was not.

## 7. Running the tests

From the repository root:

```sh
cargo test -p bt2krl                         # unit, generation, CLI, goldens, proptests, frame tests
cargo test -p krl-parser-smoke                # + krl-runner parser smoke (crates/kuka-krl-parser)
BT2KRL_VW_SAMPLES=<.../robot_data_export> cargo test -p bt2krl --test vw_samples   # VW export drift checks
cargo test -p bt2krl --features cosim        # + co-simulation with the generated head PLC (~40 min)
BT2KRL_BLESS=1 cargo test -p bt2krl --test golden   # rewrite the goldens after an intended change
```

The VW sample tests skip without `BT2KRL_VW_SAMPLES`; no sample file is ever copied into this
repository.
