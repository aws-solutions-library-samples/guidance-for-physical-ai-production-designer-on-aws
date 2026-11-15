# bt2krl design notes

The specification is [`../TASK.md`](../TASK.md). This file records how the implementation maps it
and which choices were made where the spec left room.

## Pipeline

```
program.json --program--> Program --lower--> IR (Folge: points, prologue, op blocks, epilogue)
                                               |--render--> KRL text --validate--> errors (exit 8)
                                               |               '--parse--> IR' == IR (exit 7)
                                               '--(M2) executor--> co-simulation
```

`lower` rejects programs that are unsafe or cannot run: a waypoint or the home pose outside the
model's software limits, `home_pose_deg` vs a configured `$H_POS`, or an estimated op time above
the head watchdog `limit_s`.

## Co-simulation (M2)

`exec::Executor` runs the same IR as the renderer: one `scan(dt, cmd, safe_out)` is one controller
cycle that plays `bt_sps` (job handshake, status, abort latch, step/REQUEST clearing, joint
encoder) and the robot interpreter. bt2plc's `trust::cosim::run_with` accepts such a model instead
of the generated device ST, so the head PLC is checked against the robot programs it will really
talk to. Differences from the real robot, both conservative (the model is never faster):
approximation (`C_PTP`) is not modelled, so the executor stops at every point; acceleration is a
flat `ramp_time_s` per op; the drive start-up is the device simulator's 2 s.

## IR per op

```
AbortGuard                      IF BT_ABRT THEN RETURN
SetStep(k)                      BT_STEP=k            (16-bit SIGNAL, one write)
Request(m) WaitGrant(m)         per-bit $OUT / WAIT FOR $IN AND ...   (mask != 0)
AbortGuard                                           (mask != 0)
Delay(ms)                       WAIT SEC             (start_delay_s > 0)
Move{point, vel, approx}...     BAS(#VEL_PTP,v) / PTP Pk [C_PTP]; the last one exact
Hold(ms)                        WAIT SEC             (hold_s > 0)
Dwell(50)                       WAIT SEC 0.05        (op shorter than 50 ms: step stays visible)
Release(m) WaitReleased(m)      per-bit $OUT=FALSE / WAIT FOR NOT $IN AND NOT ...
```

The first waypoint of an op is skipped when it equals the previous op's end pose (ops are
continuous in bt2plc plans). Op 1 keeps it: it is the block-coincidence move and the reference of
the start-deviation check. A blind-home op is one PTP to `PHOME` whose velocity is computed from
the previous op's end pose and `home_return_time_s / velocity_scale`. The HOME Folge cannot know
its start pose, so it computes the velocity at run time from `$AXIS_ACT`.

Velocity: `ceil(100 * ratio / time_margin)`. TASK 6.2 writes `* time_margin`, but its config
comment says "> 1 makes motion slower"; the implementation divides and requires
`time_margin >= 1`. The ceiling makes the cruise speed of a segment at most one percentage point
above the planned peak; with acceleration the real op is slower than the estimate's cruise part.

## Profile `sim`: who writes what

| Signal / flag | Writer |
|---|---|
| `BT_STEP`, REQUEST bits | Folge (robot interpreter); `bt_sps` clears them while no Folge runs or after an abort |
| `BT_RUN` (ProgRunning source) | Folge (`Running`, `Finish`/`Done`), `bt_cell` / `BT_ABORT` on abort |
| `BT_FIN` (ProgFinished) | Folge end; cleared at JobAck (`bt_sps`) and on abort |
| `BT_ERR`, `BT_ECODE` | Folge (plan mismatch), `bt_cell` (unknown Folge); cleared by Reset held 500 ms |
| `BT_ABRT` | latched by `bt_sps` (Stop, Cancel, drives off, E-stop, interpreter not active, while a Folge runs); cleared by `bt_cell` after the abort |
| status bits, echo, codes, joint bytes | `bt_sps` only, every cycle |

ProgRunning = `BT_RUN AND NOT BT_ABRT`; Ready = drives, EXT, no error, no running Folge, safety.
Order at the end (`Finish`, HOME `Done`): `WAIT SEC 0` (advance-run stop), `BT_STEP=0`,
`BT_FIN=TRUE`, `BT_RUN=FALSE` (robot interpreter: ProgFinished never later than ProgRunning
drops). The advance-run stop comes first: with `$ADVANCE=3` the advance run would execute the
assignments while the last exact-stop motion still moves, and the head would see ProgFinished
with the robot not yet at its end point. An abort sets no ProgFinished (`BT_ABORT` brakes first). The two bits reach `$OUT` in `bt_sps` (submit, another
interpreter): a submit pass can be interrupted between any two statements and the bus takes the
`$OUT` image at any moment, so "the same `bt_sps` cycle" is not enough. The STATUS fold reads
`BT_RUN` once first (`BT_PRUN=BT_RUN AND NOT BT_ABRT`; once it reads FALSE, `BT_FIN` already reads
TRUE), then writes `BT_O_FIN`, then `BT_O_RUN=BT_PRUN`. No frame shows ProgRunning low with a stale
ProgFinished low at a normal end (with `BT_O_RUN` written first, one did: `ERR_ROBOT_STOPPED` after
67 good cycles on the virtual cell). The head also debounces that state (`T_STOP_DEBOUNCE`).

## Abort

`bt_cell` declares `INTERRUPT DECL n WHEN BT_ABRT DO BT_ABORT()` and switches it on only around
the Folge call. `BT_ABORT` brakes, clears STEP/REQUEST/ProgRunning (no ProgFinished) and RESUMEs
to `bt_cell`, which never resumes the cycle (the head restarts from op 1 after recovery). The
`AbortGuard` lines are defense in depth for the case that the interrupt does not fire, e.g. when
the robot interpreter itself was stopped: then the robot can still finish the current op's motion
after a restart. This is the main unverified point (INSTALL.md says how to recover until then).

## Profile vw-ks24 (M3)

The IR gains the VW instructions (`VwAueInit`, `VwP0`, `VwAueWait`, `VwCycleStart`, `VwLock`,
`VwUnlock`, `VwOnlineStop`, `VwMakro15`, `VwTechCall`, `VwAueEnd`); `SetStep` renders as
`BT_PKTNR=k` (a global SIGNAL of `bt_config.dat` on `$OUT[97..112]`: the VW `P_PktNr_SPS` there is
local to `vw_usr_r`, so a Folge that writes it fails with KSS error 2137) and `CheckPlan` as a `HALT` guard (there is no error-code signal bt2krl may
write). `lower` refuses what the profile cannot express: the permit bit, blind `MoveHome`, more
zones than interlocks.

**Why the release dwell exists.** VW has no 4-phase handshake. The head PLC derives its REQUEST
bit from `A(80+n) OR NOT A(40+n)`, so the only moment it can see a release is the window where
`A(80+n)` is clear and `A(40+n)` is set. Without a dwell that window lasts less than one robot
cycle when the next op requests an interlock straight away, the head never drops its grant, and
the next step change is a handshake violation. The two-robot test in `tests/vw_exec_dbg.rs`
reproduces that in milliseconds; `vw_ks24.release_dwell_ms` (default 100 ms) closes it with
margin over one PROFINET cycle (8 ms on the sample robots) plus one head scan.

**What the executor models.** The VW job handshake (PRG_NR / echo / SRB / Folge end), PF0, the
ready and safety bits, the interlock levels, and the online driving condition (a withdrawn
partner release stops the motion). It does not model the TPVW inline-form machinery, `$H_POS`
capture, or the technology calls, which are no-ops for the protocol.

## Timing estimates (M4)

Per op: the sum of the segment times at the velocity percentages that were actually emitted, plus
`motion.ramp_time_s` once for the acceleration and deceleration of the op (the points inside an op
are approximated, so the robot does not stop between them). The KUKA machine data of the sample
robots carries no usable per-axis acceleration -- only the jog reduction factors `$RED_ACC_AXC` --
so the ramp is a constant, tuned so that both example cells stay within 10 % of the planned
`duration_s` (worst case 7.6 %, the two shortest ops). `est_motion_s` is reported next to
`planned_motion_s`; the op-level check against the head watchdog `limit_s` uses the full estimate
(start delay + motion + hold).

`spline` is refused rather than emitted: see README. The validator already balances
`SPLINE`/`ENDSPLINE`, so adding the mode later only needs the renderer, the parser arm and the
point-type check.
