# bt2krl

KUKA KRL generator for plan2cell. It reads the per-robot programs that [`../bt2plc`](../bt2plc/README.md)
(bt2plc) writes (`robots/<ns>/program.json`, `schema_minor` >= 1) and generates, for each robot,
KSS 8.x programs that implement the robot side of the bt2plc protocol:
- the production Folge: the op list with joint trajectories, the zone REQUEST/GRANT handshake,
  the op index ("step") and the PLAN_ID check;
- the HOME Folge (blind joint PTP to `home_pose_deg`; profile `sim` only);
- the job loop `bt_cell`, the submit part `bt_sps` (job handshake, status bits, abort latch) and
  the `SIGNAL`/global declarations to merge into `$config.dat`;
- `INSTALL.md` and `bt2krl-manifest.json` (file hashes, time estimates, validator context).

**Start here:** [`docs/USER_GUIDE.md`](docs/USER_GUIDE.md) — from a plan to installed robot
programs, the full `bt2krl.toml` reference, and the checklist before the first automatic run.
Specification: [`TASK.md`](TASK.md). Internals: [`docs/design.md`](docs/design.md).

## Status

| Milestone | State |
|---|---|
| M0 program.json additions in bt2plc | done |
| M1 profile `sim`, motion `ptp`: CLI, config, IR, renderer, validator, round trip, goldens, proptests, INSTALL.md | done |
| M2 co-simulation against the generated head PLC | done |
| M3 profile `vw-ks24` + Part B (bt2plc head-PLC profile) | done |
| M4 timing estimates | done; `spline` motion refused as unverified (see below) |

**Not verified on a controller.** No local tool can compile KRL; see "Verification" below.

## Usage

Most users run bt2krl through the combined `plan2cell build` (see the [repository
README](../../README.md)); the standalone command works on a bt2plc output directory. From the
repository root:

```sh
cargo run -p bt2krl -- config > bt2krl.toml          # defaults; add one [[robot]] per robot (e_axes is required)
cargo run -p bt2krl -- generate examples/welding_cell_2_small/plc -o out -c crates/bt2krl/tests/fixtures/bt2krl.toml
cargo run -p bt2krl -- check out                     # re-validate (strict validator + file hashes)
cargo run -p bt2krl -- explain examples/welding_cell_2_small/plc/robots/robot1/program.json
```

```
bt2krl generate <BT2PLC_OUT_DIR> -o <DIR> [-c bt2krl.toml] [--profile sim|vw-ks24]
       [--motion ptp|spline] [--robots ns,..] [--force] [--dry-run] [--strict]
bt2krl check <DIR>
bt2krl explain <program.json> [-c bt2krl.toml]
bt2krl config
```

Exit codes: 0 ok, 1 input rejected (limits, `$H_POS`, head watchdog, stale program), 2 usage,
3 config, 4 I/O, 6 `--strict`, 7 verification failed (round trip, hashes), 8 generated KRL
rejected by the validator.

Per robot, under `<DIR>/<ns>/` (KRC path layout):

| File | Content |
|---|---|
| `KRC/R1/Folgen/folge<prod>.src/.dat` | production Folge |
| `KRC/R1/Folgen/folge<home>.src/.dat` | HOME Folge |
| `KRC/R1/Program/bt_cell.src/.dat` | job loop (started once in EXT), abort interrupt |
| `KRC/R1/System/bt_sps.src` | submit part, called from `sps.sub` |
| `KRC/R1/System/bt_config.dat` | declarations to merge into `$config.dat` |
| `INSTALL.md`, `bt2krl-manifest.json` | installation steps and warnings; hashes, estimates |

Output rules: ASCII, CRLF without a final line break, 3-space indentation, identifiers <= 24
characters, byte-for-byte deterministic, atomic writes, `--force` for a non-empty directory.

## Profile `vw-ks24`

The VW tech package on the robot keeps the Folge handshake, so the generated Folge only follows
its template (`VW(#VW_AUE_INIT)`, P0 at 10 %, "Warte auf Folgenstart", the ops, `VW(#VW_AUE_END)`,
`GOTO PENTER`) and drives the standard robot interlocks inline with the MAKRO20 semantics:
request `A(80+n)`, wait for the PLC grant `E(80+n)`, wait for the partner release `E(40+n)`, drop
the own release `A(40+n)` ("occupied"), clear the request; on leaving, set `A(40+n)` again. Every
wait carries the VW operator escape, which cannot trigger in EXT mode. While an interlock is held
the partner release is part of the online driving condition `$CYCFLAG[254]`.

Written per robot: `KRC/R1/Folgen/folge<prod>.src/.dat`, `KRC/R1/System/bt_config.dat` (what to
add to `$config.dat`), `patches/` (ready-made `cell.src` CASE and `$config.dat` fragments),
`INSTALL.md`, `bt2krl-manifest.json`. There is no home Folge, no `bt_cell` and no `bt_sps`: VW
`cell.src` starts a Folge only once the robot reports PF0, so recovery stays manual.

Refused in this profile: the permit bit (the error names the edges it stands for), blind
`MoveHome` ops, more zones than the 16 VW interlocks, and writing a reserved signal (A33-A40,
A57-A80, A225-A240, E249/E250 — the validator enforces it).

The head PLC must be the one bt2plc generates with `protocol.profile = "vw-ks24"`. The cell's own
VASS blocks arbitrate first-come-first-served and would not keep the behaviour tree's order.
Because the VW scheme has no 4-phase handshake, the release level is held for
`vw_ks24.release_dwell_ms` (default 100 ms) so the head can see it and drop its grant.

## Configuration

`bt2krl config` prints every default; the full reference is in
[`docs/USER_GUIDE.md`](docs/USER_GUIDE.md#4-bt2krltoml-reference). Per robot (`[[robot]]`):
- `e_axes` is **required**: `"none"` (no external axes, `AXIS` points) or `"fixed"` with
  `e_values` (`E6AXIS` points; the external axes move to those values). `"keep"` (AXIS points on a
  robot with external axes) is refused until it is verified on KSS 8.2 that this leaves E1 alone.
- `model` (default: program.json `robot.model`): soft-limit and axis-speed table. `unknown`
  disables the limit check with a warning and uses the fastest table speeds.
- `controller`, `h_pos_deg`/`h_pos_tol_deg`, `tool_no`, `base_no`, `interlocks` (vw-ks24).

The joint values are taken from program.json as they are: bt2plc applies the URDF -> KUKA mapping
(identity by default, or a `[[robot]] joint_map` preset from `cell-data-replay/joint_map.yaml`).
Neither is validated; the limit check catches gross errors only.

## Motion (`ptp`)

Per op: joint-space RDP decimation (`decimate_tol_deg`, phase boundaries kept), then one
`BAS(#VEL_PTP,v)` per segment with `v = ceil(100 * max_i(|dq_i| / dt / vmax_i) / time_margin)`
clamped to `[min_vel_pct, max_vel_pct]`, approximated `PTP Pk C_PTP`, exact stop at the last
point. The time estimate (segment times + `ramp_time_s`, plus start delay and hold) is compared
with `expected_s` (warning above 1.2x) and the head watchdog `limit_s` (error).

## Why there is no spline mode

TASK M4 asked for `PTP_SPLINE` with `TIME_BLOCK` after a check against the KSS 8.2 manual. That
check cannot be made from the sources here: the controller exports (`am.ini`) list no spline
technology package, none of the 8 sample robots uses `SPLINE`, `SPTP` or `TIME_BLOCK` anywhere,
and there is no KUKA manual in the repository. A controller that does not accept `PTP_SPLINE`
fails at program selection, not at generation time, so `--motion spline` refuses with exit 2 and
names the missing check. Approximated joint PTP already meets the head PLC's watchdogs on both
example cells with room to spare.

The timing half of M4 is done: every op carries `est_motion_s` next to the planned `duration_s`
in the manifest and in `INSTALL.md`, and a test keeps every op of both cells (both profiles)
within 10 % of the plan (worst case 7.6 %).

## Verification

- **Strict validator** (`src/validate.rs`) on every generate and in `check`: encoding, line
  endings, block balance, identifiers, declared/used motion targets, IO ranges, and the protocol
  order of the production Folge (STEP +1, REQUEST = mask, grant wait before motion, exact stop,
  release after motion and hold, 4-phase wait).
- **IR round trip** (`src/parse.rs`): the rendered Folgen are parsed back and must equal the IR.
- **Parser smoke** ([`crates/krl-parser-smoke`](../krl-parser-smoke), with the copy of krl-runner's
  `kuka-krl-parser` in [`crates/kuka-krl-parser`](../kuka-krl-parser)):
  Folge found by DEF name, motion count, REQUEST outputs visible. It cannot reject syntax.
- **Co-simulation** (`--features cosim`): [`exec.rs`](src/exec.rs) executes the same IR as a
  `bt2plc::trust::cosim::RobotModel`, so the generated head-PLC ST runs against the behaviour of
  the generated KRL (job handshake, step, REQUEST/GRANT, joint angles on the wire). Checked in
  profile `sim`: the welding sample over 2 cycles, the single-robot sample, permit overflow, and
  an injected robot fault with recovery; plus that a wrong PLAN_ID never runs and that the motion
  check catches a bypassed grant. In profile `vw-ks24`: the welding sample over 2 cycles, the
  single-robot sample, and that a robot fault ends in manual homing.
- **Fast frame tests** (`tests/vw_exec_dbg.rs`, no PLC): the `vw-ks24` signal map on the wire, a
  full VW job handshake, the online driving condition, and both robots of the welding sample
  against a Rust twin of the head's interlock arbitration (mutual exclusion and the
  step-vs-grant handshake). These run in milliseconds and found the release-dwell bug.
- Tests: `cargo test` (unit, generation, CLI, goldens, proptests on random bt2plc programs),
  `cargo test -p krl-parser-smoke`, `cargo test -p bt2krl --features cosim` (slow: builds
  trust-runtime, ~40 min of simulation), `BT2KRL_BLESS=1 cargo test --test golden`.

Not verified (needs a KSS 8.x controller): that the files compile; the joint mapping; the abort
path (`INTERRUPT ... WHEN BT_ABRT`, BRAKE, RESUME) after an interpreter stop; the Automatic
External mapping; `BAS(#VEL_PTP)` timing against the plan; for `vw-ks24`, the VW routines called
from plain KRL. The checklist is in [`docs/USER_GUIDE.md`](docs/USER_GUIDE.md#6-before-the-first-automatic-run).
