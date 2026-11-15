# Template brief (authoritative amendments to DESIGN.md §4–§5)

DESIGN.md §4 (wire protocol) and §5 (generated CODESYS objects, near-complete ST) are the base.
This file overrides them where they differ. Reviews: `docs/reviews/{algorithm,plc,rust}.md`.
Dialect rules: `docs/ST_DIALECT.md` (MANDATORY — every rule was probed against trust).

## A. Protocol amendments
1. **Permit bit (capacity fix, algorithm review B1).** REQUEST/GRANT DWORD bit 31 is the PERMIT bit;
   zones use bits 0..30 (max 31 zones). An op whose mask has bit 31 set has extra cross-robot predecessors
   listed in `GVL_Cell.aPreds` (see C). The robot handles bit 31 exactly like a zone bit (4-phase).
   The arbiter grants the PERMIT bit of op (r,k) iff for every pred (q,j): `anDone[q] >= j`.
   `anDone[q]` (VAR_INPUT of FB_ZoneArbiter, computed by FB_Cell every scan):
   job done → `anRobotOpCount[q]`; production job active and wStep >= 1 → wStep - 1; otherwise 0
   (reset to 0 at cycle start). Grants stay all-or-nothing over zone bits + permit bit.
   The permit bit is never "held" against other robots (it is not a mutex); release = REQUEST bit 31 drops.
2. **Handshake violation (algorithm M1).** The arbiter latches `_anGrantStep[r] := step` when it grants robot r.
   Every scan, before RELEASE: if `adwGrant[r] <> DWORD#0` and the robot's step <> `_anGrantStep[r]` →
   fault `ERR_HANDSHAKE` (new E_CellError value 41).
3. **Ticket/mismatch checks are debounced** by `GVL_Cell.T_MISMATCH_DEBOUNCE` (algorithm m3): evaluate
   ERR_TICKET_SKIPPED / ERR_REQ_MISMATCH only when (step, REQUEST) has been stable that long.
4. **FCFS reservation (algorithm m1):** per scan, older requesters that are visited but not granted reserve their
   expected mask; younger requesters need `(mask AND reserved) = DWORD#0`.
5. **Step supervision independent of cell state (algorithm M2):** FB_VassRobot exposes `wLastStep` (latched max
   non-zero step during the production job, cleared at job accept). FB_Cell runs the op index tracking whenever
   that robot's production job is commanded (so the job end, index -> 0, is seen too) and the watchdogs whenever
   it runs (ProgRunning), not only in CS_RUNNING, and checks `wLastStep = anRobotOpCount[r]` at job done
   (ERR_PROGRAM_INCOMPLETE).
6. **Recovery (algorithm M7):** FB_Cell goes to `CS_RECOVERY` (new E_CellState value 85) instead of auto-homing
   and waits for the operator input `bRecoveryConfirm` (rising edge) before blind homing (HOME Folge, always
   sequential) when
   - an abort happens while a robot is *engaged* (production job commanded and: op index > 0; or last op index
     > 0 and not done; or done with `wLastStep <> anRobotOpCount`; or not at home with a non-zero last op index /
     running program in the current cycle), or holds a grant;
   - the **first bring-up after CS_INIT** (head start, restart, download) completes with any robot (active or
     idle) not at home: its pose is unknown (it may stand inside the shared space).
   Start-up homing stays automatic only when every robot is already at home (the simulator powers up at
   home). An operator stop that cancels the recovery reset (A13) restores the requirement (confirm again).
7. **Stop path (PLC review B1):** in FB_VassRobot move the "state changed → reset timers" block AFTER the global
   transitions, immediately before CASE; in RS_ERROR / RS_RESET keep `Stop := Cancel := bProgRunning`
   (never drop Stop while the robot still runs). Same pattern for FB_Cell timers. Additionally:
   - ProgRunning while the driver is not in a job (RS_OFF, bring-up states, RS_IDLE) → RS_ERROR with
     `ERR_UNEXPECTED_RUN` (new E_CellError value 23); RS_ERROR then holds Stop + Cancel.
   - RS_RESET: robot still ProgRunning after the reset pulse → RS_ERROR (`ERR_STOP_TIMEOUT`, Stop + Cancel held);
     `bCmdReset` must stay TRUE during the pulse, dropping it cancels the reset (Reset low → RS_ERROR).
   - FB_Cell CS_RESETTING completes only when no driver is in error or busy AND no robot reports ProgRunning.
8. **Latency (algorithm M5):** FB_Cell calls FB_ZoneArbiter BEFORE the GRANT bytes are encoded (grants computed from
   this scan's inputs reach the outputs in the same scan; the drivers write GRANT = 0, FB_Cell overwrites the
   bytes at the end of its body). FB_ProgramPlayer falls through zero-time transitions within one call (bounded
   loop, max 8 transitions). **No grant in a fault scan:** the fault sources known before the arbiter call
   (driver error, latched operator stop, latched fault, op index tracking faults) give the arbiter
   `bEnable := FALSE`, and the GRANT bytes are encoded only while no fault is latched (also covers faults found
   after the arbiter: watchdogs, state machine); `FB_Cell.adwGrant` shows the GRANT as sent.
9. **No TIME() anywhere.** The device player gets `rDt : REAL` (seconds per call) from PLC_PRG
   (`GVL_Program.R_TASK_S`); interpolation time is REAL seconds accumulated `rT := rT + rDt * rRate`.
   Hold/delay may use TON (virtual time in trust). There is NO TIME<->number conversion in trust.
10. **Device IO (PLC review M3/M4, rule 2 of ST_DIALECT):** device FBs must not write GVL array elements.
    PLC_PRG copies `GVL_IO.aVassOutputData`/`bySafeOut0..7` (commands from the head) into locals, calls the FBs with
    array/struct inputs, and writes `GVL_IO.aVassInputData` (whole array) and `bySafeIn0..7` (scalars) back.
    Replace non-IEC constructs: no `WORD_TO_TIME` (use TIME constants), no SHR/AND on INT
    (use `INT_TO_WORD` first), typed literals everywhere.
11. **Robot→PLC bytes 6-15 are device-written** (speed, tool, error, warning, PosX): keep them; step at
    `GVL_Program.STEP_BYTE` (4-5), request at `GVL_Program.REQ_BYTE` (28-31). Use the GVL offsets, not literals.
12. **Bring-up limit:** FB_VassRobot RS_SAFETY_ENABLE waits up to `GVL_Cell.T_BRINGUP_LIMIT` (default 60 s) for the
    safe status (PROFINET AR + PROFIsafe connection set-up after a cold start / download), then ERR_SAFETY_TIMEOUT.
    RS_DRIVES_ON / RS_MODE_EXT (link already up) and every job handshake keep `T_HANDSHAKE`.
13. **Operator stop / reset latch:** a `bStop` edge is latched in every cell state until it is consumed:
    in the normal states it starts the abort (ERR_OPERATOR_STOP unless another fault came first); in CS_INIT and
    CS_RESETTING it cancels the (re)start → CS_ERROR with ERR_OPERATOR_STOP, the driver Reset commands are
    dropped (drivers in RS_RESET return to RS_ERROR); in CS_ABORTING it stays latched; in CS_ERROR it is cleared.
    A `bReset` edge in CS_ABORTING is latched and applied when CS_ERROR is reached; a later `bStop` edge
    cancels it.
14. **Op index 0 while the production program runs:** with 0 < last validated index < op count →
    ERR_STEP_SEQUENCE; before the first op (last index 0) a stamp of the free-running clock limits it to
    `T_HANDSHAKE`, after the last op to MAX(last op `tLimit`, `T_HANDSHAKE`) → ERR_ROBOT_STALLED (nErrorRobot set,
    nErrorOp 0).
15. **Zone bypass:** when the op index leaves op k (to k+1, to anything else, or to 0 at the job end) and
    `aOps[k].dwZoneMask <> 0` but the op was never granted → ERR_ZONE_BYPASSED (new E_CellError value 42,
    nErrorOp = k), raised in the same scan before the new index is accepted, under both grant policies.

## B. Template placeholders
Head templates may use `{{N_ROBOTS}}`, `{{N_ZONES_DIM}}`, `{{N_TICKETS_DIM}}`, `{{N_OPS_DIM}}`, `{{N_PREDS_DIM}}`
for ARRAY bounds (never `GVL_Cell.*` in bounds), `{{GENERATOR}}`, `{{TREE_ID}}`, `{{CELL_ID}}` in header comments,
and `{{#robots}} … {{/robots}}` blocks with `{{r.i}} {{r.tag}} {{r.ns}} {{r.station}} {{r.device_name}} {{r.ip}}`
(used by GVL_IO and PLC_PRG). Device templates may use `{{GENERATOR}} {{TREE_ID}} {{CELL_ID}} {{ROBOT_TAG}}
{{ROBOT_NS}} {{STATION}}` (header comments only).

## C. Generated GVLs (emitted by Rust — templates must use exactly these names and types)
### Head `GVL_Cell` — `{attribute 'qualified_only'}` `VAR_GLOBAL CONSTANT`
```
N_ROBOTS : INT;  N_ZONES : INT;  N_OPS : INT;  N_PREDS : INT;          (* real counts *)
PLAN_ID : WORD;  PROD_FOLGE : WORD;  HOME_FOLGE : WORD;  JOB_NUMBER_BYTE : BOOL;
GRANT_POLICY : E_GrantPolicy;  HOME_SEQUENTIAL : BOOL;  AUTO_RUN : BOOL;
STEP_BYTE : INT;  REQ_BYTE : INT;  GRANT_BYTE : INT;  PLAN_ID_BYTE : INT;
PERMIT_MASK : DWORD := DWORD#16#80000000;
T_RESET_PULSE, T_START_PULSE, T_STOP_PULSE, T_HANDSHAKE, T_BRINGUP_LIMIT, T_MISMATCH_DEBOUNCE, T_STOP_DEBOUNCE,
T_CYCLE_PAUSE, T_START_LIMIT, T_ABORT_LIMIT, T_HOME_LIMIT : TIME;
asRobotTag : ARRAY[1..N_ROBOTS] OF STRING(16);   asRobotNs : ARRAY[1..N_ROBOTS] OF STRING(32);
abRobotActive : ARRAY[1..N_ROBOTS] OF BOOL;      (* FALSE = idle robot: bring-up/homing only *)
awRobotProdFolge : ARRAY[1..N_ROBOTS] OF WORD;
anRobotFirstOp : ARRAY[1..N_ROBOTS] OF INT;      (* op k of robot r: aOps[anRobotFirstOp[r] + k - 1] *)
anRobotOpCount : ARRAY[1..N_ROBOTS] OF INT;
asZoneName : ARRAY[1..N_ZONES_DIM] OF STRING(8);
anZoneFirst : ARRAY[1..N_ZONES_DIM] OF INT;  anZoneCount : ARRAY[1..N_ZONES_DIM] OF INT;
aTickets : ARRAY[1..N_TICKETS_DIM] OF ST_Ticket;  (* zone z: aTickets[anZoneFirst[z] .. +anZoneCount[z]-1] *)
aOps : ARRAY[1..N_OPS_DIM] OF ST_OpInfo;
aPreds : ARRAY[1..N_PREDS_DIM] OF ST_Ticket;      (* permit preds of op: aPreds[nPredFirst .. +nPredCount-1] *)
```
(Bounds are literals in the emitted file. `*_DIM = MAX(1, n)`; dummy rows are zero.)
`TYPE ST_Ticket : STRUCT nRobot : INT; nOp : INT; END_STRUCT END_TYPE` (1-based; 0 = none)
`TYPE ST_OpInfo : STRUCT nRobot : INT; nOp : INT; dwZoneMask : DWORD; nPredFirst : INT; nPredCount : INT;
  tExpected : TIME; tLimit : TIME; tZoneDeadline : TIME; sName : STRING(32); END_STRUCT END_TYPE`
`dwZoneMask` includes bit 31 when nPredCount > 0. Robot index r is 1-based.
Welding sample values: see DESIGN §5.1 GVL_Cell (2 robots, 2 zones, 16 tickets, 18 ops, 0 preds).

### Device `GVL_Program` — `{attribute 'qualified_only'}` `VAR_GLOBAL CONSTANT`
```
ROBOT_TAG : STRING(16);  ROBOT_NS : STRING(32);  ROBOT_INDEX : WORD;
PLAN_ID : WORD;  PROD_FOLGE : WORD;  HOME_FOLGE : WORD;  JOB_NUMBER_BYTE : BOOL;
STEP_BYTE : INT;  REQ_BYTE : INT;  GRANT_BYTE : INT;  PLAN_ID_BYTE : INT;
R_TASK_S : REAL;            (* device MainTask interval in s, e.g. 0.02 *)
R_HOME_S : REAL;            (* blind home duration, 7.0 *)
R_AT_HOME_TOL_DEG : REAL;  R_START_TOL_DEG : REAL;
ERR_UNKNOWN_FOLGE : WORD := WORD#16#0101;  ERR_PLAN_MISMATCH : WORD := WORD#16#0103;
WARN_START_DEVIATION : WORD := WORD#16#0201;
N_OPS : INT;  N_WP : INT;
arHomeDeg : ARRAY[1..6] OF REAL;
aOps : ARRAY[1..N_OPS_DIM] OF ST_OpDef;
aWp  : ARRAY[1..N_WP_DIM]  OF ST_Waypoint;
```
`TYPE ST_OpDef : STRUCT sName : STRING(32); nFirstWp : INT; nLastWp : INT; dwZoneMask : DWORD;
  tDelay : TIME; tHold : TIME; rDelay_s : REAL; rHold_s : REAL; rVelScale : REAL; bBlindHome : BOOL; END_STRUCT END_TYPE`
`TYPE ST_Waypoint : STRUCT rT : REAL; rA1 : REAL; rA2 : REAL; rA3 : REAL; rA4 : REAL; rA5 : REAL; rA6 : REAL; END_STRUCT END_TYPE`
(rT = seconds from op start; angles in degrees after joint sign/offset mapping. Blind home op: nFirstWp = nLastWp = 0,
bBlindHome = TRUE → interpolate from the current pose to arHomeDeg over R_HOME_S.)

## D. Object lists
Head (`templates/codesys/head`): DUT: E_GrantPolicy, E_CellError, E_RobotState, E_CellState, ST_Ticket, ST_OpInfo,
ST_RobotIn, ST_RobotOut; GVL: GVL_IO (template with robots block); POU: FB_VassRobot, FB_ZoneArbiter, FB_Cell,
PLC_PRG (template with robots block; replaces the default PLC_PRG). GVL_Cell is generated by Rust.
PLC_PRG must expose operator inputs as plain VARs (bAutoRun, bStart, bStop, bReset, bRecoveryConfirm) and the
cell status by calling `fbCell` (status read via `PLC_PRG.fbCell.<VAR_OUTPUT>`).
Device (`templates/codesys/device`): DUT split of VASS_Types.st (E_RobotMode, E_ProgramState, ST_SafeInput,
ST_SafeOutput, ST_PROFIsafe_Trailer, ST_VASS_StatusWord, ST_VASS_ControlWord, ST_VASS_InputTelegram,
ST_VASS_OutputTelegram) + new E_PlayerState, ST_OpDef, ST_Waypoint; GVL: GVL_IO (same names as the original
device project), GVL_DeviceParams; POU: FB_PROFIsafe_Simulator, FB_JointAngleEncoder, FB_ProgramPlayer,
FB_VASS_DeviceSimulator, PLC_PRG. GVL_Program is generated by Rust. Device PLC_PRG keeps the debug inputs
bTest_InjectError, bTest_InjectSafetyFault, wTest_ErrorCode and exposes bSimulatorEnable.
