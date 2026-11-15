(*
 * FB_Cell - Implementation
 *)

IF NOT bEnable THEN
    _eState := E_CellState.CS_INIT;
END_IF

(* ============================================================================
   TIME BASE / EDGES
   ============================================================================ *)
_tonClock(IN := TRUE, PT := T#1H);
_tNow := _tClockBase + _tonClock.ET;
IF _tonClock.Q THEN
    _tClockBase := _tNow;
    _tonClock(IN := FALSE);
    _tonClock(IN := TRUE, PT := T#1H);
END_IF

_rtrigStart(CLK := bStart);
_rtrigStop(CLK := bStop);
_rtrigReset(CLK := bReset);
_rtrigConfirm(CLK := bRecoveryConfirm);

(* Operator stop: latched in every state until consumed (abort, cancelled reset, CS_ERROR);
   it also cancels a reset that is pending from CS_ABORTING *)
IF _rtrigStop.Q THEN
    _bStopReq := TRUE;
    _bResetReq := FALSE;
END_IF

FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
    _abHasJob[_r] := GVL_Cell.abRobotActive[_r] AND (GVL_Cell.anRobotOpCount[_r] > 0);
END_FOR

(* ============================================================================
   ROBOT DRIVERS (commands from the previous scan; GRANT bytes are added below)
   ============================================================================ *)
{{#robots}}
(* {{r.i}}: {{r.tag}} - {{r.ns}} *)
fbRobot_{{r.tag}}(
    bEnable     := bEnable,
    bPowerOn    := _bPowerOn,
    bCmdJob     := _abJobCmd[{{r.i}}],
    wJobNr      := _awJob[{{r.i}}],
    bProdJob    := _abProdCmd[{{r.i}}],
    bCmdStop    := _abStopCmd[{{r.i}}],
    bCmdReset   := _abResetCmd[{{r.i}}],
    nRobot      := {{r.i}},
    stIn        := astIn[{{r.i}}]);
astOut[{{r.i}}]         := fbRobot_{{r.tag}}.stOut;
aeRobotState[{{r.i}}]   := fbRobot_{{r.tag}}.eState;
aeRobotError[{{r.i}}]   := fbRobot_{{r.tag}}.eError;
awStep[{{r.i}}]         := fbRobot_{{r.tag}}.wStep;
adwReq[{{r.i}}]         := fbRobot_{{r.tag}}.dwZoneReq;
_awLastStep[{{r.i}}]    := fbRobot_{{r.tag}}.wLastStep;
_abIdle[{{r.i}}]        := fbRobot_{{r.tag}}.bIdle;
_abBusy[{{r.i}}]        := fbRobot_{{r.tag}}.bBusy;
_abDrvError[{{r.i}}]    := fbRobot_{{r.tag}}.bError;
_abJobActive[{{r.i}}]   := fbRobot_{{r.tag}}.bJobActive;
_abJobDone[{{r.i}}]     := fbRobot_{{r.tag}}.bJobDone;
_abAtHome[{{r.i}}]      := fbRobot_{{r.tag}}.bAtHome;
_abRobotError[{{r.i}}]  := fbRobot_{{r.tag}}.bRobotError;
_abProgRunning[{{r.i}}] := fbRobot_{{r.tag}}.bProgRunning;
{{/robots}}

(* A robot inside its production program or holding a grant (previous scan's commands):
   inside an op, between two ops, finished with an incomplete program, or left the
   home pose during the current cycle *)
_bEngaged := FALSE;
FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
    IF _abJobCmd[_r] AND _abProdCmd[_r] THEN
        IF awStep[_r] <> WORD#0 THEN
            _bEngaged := TRUE;
        ELSIF (_awLastStep[_r] <> WORD#0) AND NOT _abJobDone[_r] THEN
            _bEngaged := TRUE;
        ELSIF _abJobDone[_r] AND (WORD_TO_INT(_awLastStep[_r]) <> GVL_Cell.anRobotOpCount[_r]) THEN
            _bEngaged := TRUE;
        ELSIF NOT _abAtHome[_r]
              AND ((_awLastStep[_r] <> WORD#0) OR (_anLastStep[_r] <> 0) OR _abJobActive[_r]) THEN
            _bEngaged := TRUE;
        END_IF
    END_IF
    IF fbArbiter.adwGrant[_r] <> DWORD#0 THEN
        _bEngaged := TRUE;
    END_IF
END_FOR

(* ============================================================================
   OP INDEX TRACKING (production job commanded, any cell state)
   The index must advance by exactly +1. The op that is left (also at the job end,
   index -> 0) must have been granted if it has a zone / permit mask (both policies).
   Index 0 while the program runs is valid only before the first / after the last op.
   ============================================================================ *)
_eNew := E_CellError.ERR_NONE;
_nNewRobot := 0;
_nNewOp := 0;
FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
    IF _abJobCmd[_r] AND _abProdCmd[_r] THEN
        _nStep := WORD_TO_INT(awStep[_r]);
        IF _nStep <> _anStepSeen[_r] THEN
            _nPrev := _anStepSeen[_r];
            _bBypass := FALSE;
            IF (_nPrev >= 1) AND (_nPrev <= GVL_Cell.anRobotOpCount[_r]) AND NOT _abGranted[_r] THEN
                _iOp := GVL_Cell.anRobotFirstOp[_r] + _nPrev - 1;
                IF GVL_Cell.aOps[_iOp].dwZoneMask <> DWORD#0 THEN
                    _bBypass := TRUE;               (* zone / permit op left without a grant *)
                END_IF
            END_IF
            IF _bBypass THEN
                IF _eNew = E_CellError.ERR_NONE THEN
                    _eNew := E_CellError.ERR_ZONE_BYPASSED;
                    _nNewRobot := _r;
                    _nNewOp := _nPrev;
                END_IF
            ELSIF _nStep <> 0 THEN
                IF (_nStep = _anLastStep[_r] + 1) AND (_nStep <= GVL_Cell.anRobotOpCount[_r]) THEN
                    _anLastStep[_r] := _nStep;
                ELSIF _eNew = E_CellError.ERR_NONE THEN
                    _eNew := E_CellError.ERR_STEP_SEQUENCE;
                    _nNewRobot := _r;
                    _nNewOp := _nStep;
                END_IF
            END_IF
            _anStepSeen[_r] := _nStep;
            _abGranted[_r] := FALSE;
            _abOpRun[_r] := FALSE;
            _abStallRun[_r] := FALSE;
            _abZeroRun[_r] := FALSE;
        END_IF

        (* index 0 while the program runs, between two ops *)
        IF _abJobActive[_r] AND (_nStep = 0) AND (_anLastStep[_r] > 0)
           AND (_anLastStep[_r] < GVL_Cell.anRobotOpCount[_r])
           AND (_eNew = E_CellError.ERR_NONE) THEN
            _eNew := E_CellError.ERR_STEP_SEQUENCE;
            _nNewRobot := _r;
            _nNewOp := 0;
        END_IF

        (* ops completed (permit predecessors): validated index - 1, all at job done *)
        IF _abJobDone[_r] AND (_anLastStep[_r] = GVL_Cell.anRobotOpCount[_r]) THEN
            _nDone := GVL_Cell.anRobotOpCount[_r];
        ELSIF _anLastStep[_r] > 0 THEN
            _nDone := _anLastStep[_r] - 1;
        ELSE
            _nDone := 0;
        END_IF
        _anDone[_r] := MAX(_anDone[_r], _nDone);
    END_IF
END_FOR

(* ============================================================================
   ZONE ARBITER (this scan's robot inputs)
   No new grants in a scan in which the cell already knows it will abort:
   driver error, latched operator stop, latched fault, supervision fault above.
   ============================================================================ *)
_bFaultScan := _bFault OR _bStopReq OR (_eNew <> E_CellError.ERR_NONE);
FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
    _abRobotOk[_r] := _abJobActive[_r] AND _abJobCmd[_r] AND _abProdCmd[_r] AND NOT _abRobotError[_r];
    IF _abDrvError[_r] THEN
        _bFaultScan := TRUE;
    END_IF
END_FOR
fbArbiter(
    bReset      := _bArbReset,
    bEnable     := ((_eState = E_CellState.CS_START_JOBS) OR (_eState = E_CellState.CS_RUNNING))
                   AND NOT _bFaultScan,
    ePolicy     := GVL_Cell.GRANT_POLICY,
    tDebounce   := GVL_Cell.T_MISMATCH_DEBOUNCE,
    tNow        := _tNow,
    abRobotOk   := _abRobotOk,
    awStep      := awStep,
    adwReq      := adwReq,
    anDone      := _anDone);
_bArbReset := FALSE;

(* ============================================================================
   WATCHDOGS (production job running, any cell state)
   op watchdog from the step change (mask 0) or from the grant; stall; zone wait;
   op index 0: T_HANDSHAKE before the first op, MAX(last op tLimit, T_HANDSHAKE)
   after the last op
   ============================================================================ *)
FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
    IF _abJobCmd[_r] AND _abProdCmd[_r] AND _abJobActive[_r] THEN
        _nStep := _anStepSeen[_r];
        IF (_nStep >= 1) AND (_nStep <= GVL_Cell.anRobotOpCount[_r]) THEN
            _iOp := GVL_Cell.anRobotFirstOp[_r] + _nStep - 1;
            IF GVL_Cell.aOps[_iOp].dwZoneMask = DWORD#0 THEN
                IF NOT _abOpRun[_r] THEN
                    _abOpRun[_r] := TRUE;
                    _atOpStart[_r] := _tNow;
                END_IF
            ELSE
                IF fbArbiter.adwGrant[_r] <> DWORD#0 THEN
                    _abGranted[_r] := TRUE;
                END_IF
                IF _abGranted[_r] AND NOT _abOpRun[_r] THEN
                    _abOpRun[_r] := TRUE;
                    _atOpStart[_r] := _tNow;
                END_IF

                (* zone op with neither REQUEST nor grant *)
                IF NOT _abGranted[_r] AND (adwReq[_r] = DWORD#0) THEN
                    IF NOT _abStallRun[_r] THEN
                        _abStallRun[_r] := TRUE;
                        _atStallStart[_r] := _tNow;
                    ELSIF ((_tNow - _atStallStart[_r]) >= GVL_Cell.T_HANDSHAKE)
                          AND (_eNew = E_CellError.ERR_NONE) THEN
                        _eNew := E_CellError.ERR_ROBOT_STALLED;
                        _nNewRobot := _r;
                        _nNewOp := _nStep;
                    END_IF
                ELSE
                    _abStallRun[_r] := FALSE;
                END_IF

                (* zone wait past the deadline (measured since the cycle started RUNNING) *)
                IF NOT _abGranted[_r] AND (adwReq[_r] <> DWORD#0)
                   AND (_eState = E_CellState.CS_RUNNING)
                   AND ((_tNow - _tCycleStart) > GVL_Cell.aOps[_iOp].tZoneDeadline)
                   AND (_eNew = E_CellError.ERR_NONE) THEN
                    _eNew := E_CellError.ERR_ZONE_WAIT_TIMEOUT;
                    _nNewRobot := _r;
                    _nNewOp := _nStep;
                END_IF
            END_IF

            IF _abOpRun[_r] AND ((_tNow - _atOpStart[_r]) >= GVL_Cell.aOps[_iOp].tLimit)
               AND (_eNew = E_CellError.ERR_NONE) THEN
                _eNew := E_CellError.ERR_OP_WATCHDOG;
                _nNewRobot := _r;
                _nNewOp := _nStep;
            END_IF
        ELSIF _nStep = 0 THEN
            (* program running without an op: before the first / after the last op *)
            IF NOT _abZeroRun[_r] THEN
                _abZeroRun[_r] := TRUE;
                _atZeroStart[_r] := _tNow;
            END_IF
            _tZeroLimit := GVL_Cell.T_HANDSHAKE;
            IF (_anLastStep[_r] >= 1) AND (_anLastStep[_r] = GVL_Cell.anRobotOpCount[_r]) THEN
                _iOp := GVL_Cell.anRobotFirstOp[_r] + _anLastStep[_r] - 1;
                IF GVL_Cell.aOps[_iOp].tLimit > _tZeroLimit THEN
                    _tZeroLimit := GVL_Cell.aOps[_iOp].tLimit;
                END_IF
            END_IF
            IF ((_tNow - _atZeroStart[_r]) >= _tZeroLimit) AND (_eNew = E_CellError.ERR_NONE) THEN
                _eNew := E_CellError.ERR_ROBOT_STALLED;
                _nNewRobot := _r;
                _nNewOp := 0;
            END_IF
        END_IF
    ELSE
        _abZeroRun[_r] := FALSE;
    END_IF
END_FOR

(* ============================================================================
   FAULT COLLECTION (first wins: robot > supervision > arbiter > operator)
   ============================================================================ *)
IF (_eState <> E_CellState.CS_INIT) AND (_eState <> E_CellState.CS_ABORTING)
   AND (_eState <> E_CellState.CS_ERROR) AND (_eState <> E_CellState.CS_RESETTING) THEN
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        IF _abDrvError[_r] AND NOT _bFault AND (_eState <> E_CellState.CS_RECOVERY) THEN
            _bFault := TRUE;
            eError := aeRobotError[_r];
            nErrorRobot := _r;
            nErrorOp := WORD_TO_INT(awStep[_r]);
            nErrorZone := 0;
        END_IF
    END_FOR
    IF (_eNew <> E_CellError.ERR_NONE) AND NOT _bFault THEN
        _bFault := TRUE;
        eError := _eNew;
        nErrorRobot := _nNewRobot;
        nErrorOp := _nNewOp;
        nErrorZone := 0;
    END_IF
    IF fbArbiter.bFault AND NOT _bFault THEN
        _bFault := TRUE;
        eError := fbArbiter.eFault;
        nErrorRobot := fbArbiter.nFaultRobot;
        nErrorOp := 0;
        IF (fbArbiter.nFaultRobot >= 1) AND (fbArbiter.nFaultRobot <= GVL_Cell.N_ROBOTS) THEN
            nErrorOp := WORD_TO_INT(awStep[fbArbiter.nFaultRobot]);
        END_IF
        nErrorZone := fbArbiter.nFaultZone;
    END_IF
    IF _bStopReq THEN
        _bStopReq := FALSE;                 (* consumed by this abort *)
        IF NOT _bFault THEN
            _bFault := TRUE;
            eError := E_CellError.ERR_OPERATOR_STOP;
            nErrorRobot := 0;
            nErrorOp := 0;
            nErrorZone := 0;
        END_IF
    END_IF
    IF _bFault THEN
        _eState := E_CellState.CS_ABORTING;
    END_IF
END_IF

(* ============================================================================
   STATE TIMER (after the transitions above: a new state starts with a fresh timer)
   ============================================================================ *)
IF _eState <> _eLastState THEN
    IF _eState = E_CellState.CS_ABORTING THEN
        _bConfirmLatch := FALSE;
        IF _bEngaged THEN
            _bNeedRecovery := TRUE;         (* aborted with a robot inside its program *)
        END_IF
    END_IF
    _tonState(IN := FALSE);
    _eLastState := _eState;
END_IF
CASE _eState OF
E_CellState.CS_HOMING_RUN:
    _tLimit := GVL_Cell.T_HOME_LIMIT;
E_CellState.CS_START_JOBS:
    _tLimit := GVL_Cell.T_START_LIMIT;
E_CellState.CS_CYCLE_DONE:
    _tLimit := GVL_Cell.T_CYCLE_PAUSE;
E_CellState.CS_ABORTING, E_CellState.CS_RESETTING:
    _tLimit := GVL_Cell.T_ABORT_LIMIT;
ELSE
    _tLimit := T#24H;
END_CASE
_tonState(IN := TRUE, PT := _tLimit);

(* Recovery confirm: accepted once the robots are stopped (CS_ERROR on), used in CS_RECOVERY *)
IF _rtrigConfirm.Q AND _bNeedRecovery
   AND ((_eState = E_CellState.CS_ERROR) OR (_eState = E_CellState.CS_RESETTING)
        OR (_eState = E_CellState.CS_BRINGUP) OR (_eState = E_CellState.CS_RECOVERY)) THEN
    _bConfirmLatch := TRUE;
END_IF

(* ============================================================================
   STATE MACHINE
   ============================================================================ *)
CASE _eState OF

E_CellState.CS_INIT:
    _bPowerOn := FALSE;
    _bArbReset := TRUE;
    _bFirstBringup := TRUE;                 (* head start / restart: robot poses unknown *)
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        _abJobCmd[_r] := FALSE;
        _abProdCmd[_r] := FALSE;
        _abStopCmd[_r] := FALSE;
        _abResetCmd[_r] := FALSE;
        _abHoming[_r] := FALSE;
    END_FOR
    IF bEnable THEN
        IF _bStopReq THEN
            (* operator stop before the start-up: no automatic bring-up *)
            _bStopReq := FALSE;
            _bResetReq := FALSE;
            _bFault := TRUE;
            eError := E_CellError.ERR_OPERATOR_STOP;
            nErrorRobot := 0;
            nErrorOp := 0;
            nErrorZone := 0;
            _eState := E_CellState.CS_ERROR;
        ELSE
            _eState := E_CellState.CS_BRINGUP;
        END_IF
    END_IF

E_CellState.CS_BRINGUP:
    _bPowerOn := TRUE;
    _bAll := TRUE;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        IF NOT _abIdle[_r] THEN
            _bAll := FALSE;
        END_IF
    END_FOR
    IF _bAll THEN
        IF _bFirstBringup THEN
            (* first bring-up after INIT: any robot away from home (also an idle one)
               may stand anywhere in the shared space (head restart / download mid-cycle) *)
            _bFirstBringup := FALSE;
            FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
                IF NOT _abAtHome[_r] AND NOT _bNeedRecovery THEN
                    _bNeedRecovery := TRUE;
                    _bConfirmLatch := FALSE;
                END_IF
            END_FOR
        END_IF
        IF _bNeedRecovery THEN
            _eState := E_CellState.CS_RECOVERY;
        ELSE
            _eState := E_CellState.CS_HOMING_SELECT;
        END_IF
    END_IF

E_CellState.CS_RECOVERY:
    (* robots may be anywhere inside the shared space: the operator moves them
       clear (T1, driver errors are ignored here) and confirms; then the drivers
       are reset, brought up again and blind homing runs one robot at a time *)
    IF _bConfirmLatch THEN
        _bConfirmLatch := FALSE;
        _bNeedRecovery := FALSE;
        _bRecoveryHoming := TRUE;
        _eState := E_CellState.CS_RESETTING;
    END_IF

E_CellState.CS_HOMING_SELECT:
    IF GVL_Cell.PROFILE_VW THEN
        (* VW cell.src starts a Folge only once the robot reports PF0, and there is no blind
           home Folge: wait for the operator to jog every robot home (ERR_HOME_FAILED on
           T_HOME_LIMIT tells which robot is not at home). *)
        _bAll := TRUE;
        FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
            IF NOT _abAtHome[_r] THEN
                _bAll := FALSE;
                bHomingRequired := TRUE;
            END_IF
        END_FOR
        IF _bAll THEN
            bHomingRequired := FALSE;
            _bRecoveryHoming := FALSE;
            _eState := E_CellState.CS_READY;
        ELSIF _tonState.Q THEN
            _bFault := TRUE;
            eError := E_CellError.ERR_HOME_FAILED;
            nErrorRobot := 0;
            FOR _r := GVL_Cell.N_ROBOTS TO 1 BY -1 DO
                IF NOT _abAtHome[_r] THEN
                    nErrorRobot := _r;
                END_IF
            END_FOR
            nErrorOp := 0;
            nErrorZone := 0;
            _eState := E_CellState.CS_ERROR;
        END_IF
    ELSE
        _bSequential := GVL_Cell.HOME_SEQUENTIAL OR _bRecoveryHoming;
        _nHomeCount := 0;
        FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
            _abHoming[_r] := FALSE;
            IF NOT _abAtHome[_r] AND (NOT _bSequential OR (_nHomeCount = 0)) THEN
                _abHoming[_r] := TRUE;
                _awJob[_r] := GVL_Cell.HOME_FOLGE;
                _abProdCmd[_r] := FALSE;
                _nHomeCount := _nHomeCount + 1;
            END_IF
        END_FOR
        IF _nHomeCount = 0 THEN
            _bRecoveryHoming := FALSE;
            _eState := E_CellState.CS_READY;
        ELSE
            _eState := E_CellState.CS_HOMING_RUN;
        END_IF
    END_IF

E_CellState.CS_HOMING_RUN:
    _bAll := TRUE;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        IF _abHoming[_r] THEN
            _abJobCmd[_r] := TRUE;
            IF NOT _abJobDone[_r] THEN
                _bAll := FALSE;
            ELSIF NOT _abAtHome[_r] AND NOT _bFault THEN
                _bFault := TRUE;
                eError := E_CellError.ERR_HOME_FAILED;
                nErrorRobot := _r;
                nErrorOp := 0;
                nErrorZone := 0;
            END_IF
        END_IF
    END_FOR
    IF _bFault THEN
        _eState := E_CellState.CS_ABORTING;
    ELSIF _bAll THEN
        FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
            IF _abHoming[_r] THEN
                _abJobCmd[_r] := FALSE;
                _abHoming[_r] := FALSE;
            END_IF
        END_FOR
        _eState := E_CellState.CS_HOMING_SELECT;
    ELSIF _tonState.Q THEN
        _bFault := TRUE;
        eError := E_CellError.ERR_HOME_FAILED;
        nErrorRobot := 0;
        FOR _r := GVL_Cell.N_ROBOTS TO 1 BY -1 DO
            IF _abHoming[_r] AND NOT _abJobDone[_r] THEN
                nErrorRobot := _r;
            END_IF
        END_FOR
        nErrorOp := 0;
        nErrorZone := 0;
        _eState := E_CellState.CS_ABORTING;
    END_IF

E_CellState.CS_READY:
    IF _rtrigStart.Q OR bAutoRun THEN
        _bAll := TRUE;
        FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
            IF _abHasJob[_r] AND NOT _abAtHome[_r] THEN
                _bAll := FALSE;             (* programs start at the home pose *)
            END_IF
        END_FOR
        IF _bAll THEN
            _bArbReset := TRUE;             (* rewind tickets for this cycle *)
            FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
                _anStepSeen[_r] := 0;
                _anLastStep[_r] := 0;
                _anDone[_r] := 0;
                _abGranted[_r] := FALSE;
                _abOpRun[_r] := FALSE;
                _abStallRun[_r] := FALSE;
                _abZeroRun[_r] := FALSE;
                _abProdCmd[_r] := FALSE;
                IF _abHasJob[_r] THEN
                    _awJob[_r] := GVL_Cell.awRobotProdFolge[_r];
                    _abProdCmd[_r] := TRUE;
                END_IF
            END_FOR
            _eState := E_CellState.CS_START_JOBS;
        ELSE
            _eState := E_CellState.CS_HOMING_SELECT;
        END_IF
    END_IF

E_CellState.CS_START_JOBS:
    _bAll := TRUE;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        IF _abHasJob[_r] THEN
            _abJobCmd[_r] := TRUE;
            IF NOT (_abJobActive[_r] OR _abJobDone[_r]) THEN
                _bAll := FALSE;
            END_IF
        END_IF
    END_FOR
    IF _bAll THEN
        _tCycleStart := _tNow;
        _eState := E_CellState.CS_RUNNING;
    ELSIF _tonState.Q THEN
        _bFault := TRUE;
        eError := E_CellError.ERR_START_TIMEOUT;
        nErrorRobot := 0;
        FOR _r := GVL_Cell.N_ROBOTS TO 1 BY -1 DO
            IF _abHasJob[_r] AND NOT (_abJobActive[_r] OR _abJobDone[_r]) THEN
                nErrorRobot := _r;
            END_IF
        END_FOR
        nErrorOp := 0;
        nErrorZone := 0;
        _eState := E_CellState.CS_ABORTING;
    END_IF

E_CellState.CS_RUNNING:
    _bAll := TRUE;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        IF _abHasJob[_r] THEN
            IF NOT _abJobDone[_r] THEN
                _bAll := FALSE;
            ELSIF (WORD_TO_INT(_awLastStep[_r]) <> GVL_Cell.anRobotOpCount[_r]) AND NOT _bFault THEN
                _bFault := TRUE;
                eError := E_CellError.ERR_PROGRAM_INCOMPLETE;
                nErrorRobot := _r;
                nErrorOp := WORD_TO_INT(_awLastStep[_r]);
                nErrorZone := 0;
            END_IF
        END_IF
    END_FOR
    IF _bFault THEN
        _eState := E_CellState.CS_ABORTING;
    ELSIF _bAll THEN
        IF NOT fbArbiter.bAllConsumed THEN
            _bFault := TRUE;
            eError := E_CellError.ERR_TICKETS_LEFT;
            nErrorRobot := 0;
            nErrorOp := 0;
            nErrorZone := 0;
            _eState := E_CellState.CS_ABORTING;
        ELSE
            FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
                _abJobCmd[_r] := FALSE;
            END_FOR
            udiCycles := udiCycles + 1;
            tLastCycle := _tNow - _tCycleStart;
            _eState := E_CellState.CS_CYCLE_DONE;
        END_IF
    END_IF

E_CellState.CS_CYCLE_DONE:
    IF _tonState.Q THEN
        _eState := E_CellState.CS_READY;
    END_IF

E_CellState.CS_ABORTING:
    _bArbReset := TRUE;                     (* drop all grants, rewind tickets *)
    _bAll := TRUE;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        _abJobCmd[_r] := FALSE;
        _abHoming[_r] := FALSE;
        _abStopCmd[_r] := TRUE;
        IF _abBusy[_r] THEN
            _bAll := FALSE;
        END_IF
    END_FOR
    IF _rtrigReset.Q AND NOT _rtrigStop.Q THEN
        _bResetReq := TRUE;                 (* applied once CS_ERROR is reached *)
        _bStopReq := FALSE;                 (* the abort in progress already stops *)
    END_IF
    IF _bAll OR _tonState.Q THEN
        _eState := E_CellState.CS_ERROR;
    END_IF

E_CellState.CS_ERROR:
    _bArbReset := TRUE;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        _abStopCmd[_r] := FALSE;            (* drivers keep Stop while a robot still runs *)
        _abResetCmd[_r] := FALSE;           (* re-arm the reset edges *)
    END_FOR
    IF _bStopReq THEN
        _bStopReq := FALSE;                 (* already stopped *)
        _bResetReq := FALSE;
    ELSIF _rtrigReset.Q OR _bResetReq THEN
        _bResetReq := FALSE;
        _eState := E_CellState.CS_RESETTING;
    END_IF

E_CellState.CS_RESETTING:
    IF _bStopReq THEN
        (* operator stop cancels the reset: driver Reset low, no bring-up / homing *)
        _bStopReq := FALSE;
        _bResetReq := FALSE;
        FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
            _abResetCmd[_r] := FALSE;
        END_FOR
        IF _bRecoveryHoming THEN
            _bRecoveryHoming := FALSE;
            _bNeedRecovery := TRUE;         (* confirm again after the next reset *)
        END_IF
        _bConfirmLatch := FALSE;
        _bFault := TRUE;
        eError := E_CellError.ERR_OPERATOR_STOP;
        nErrorRobot := 0;
        nErrorOp := 0;
        nErrorZone := 0;
        _eState := E_CellState.CS_ERROR;
    ELSE
        _bAll := TRUE;
        FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
            _abResetCmd[_r] := _abDrvError[_r];
            IF _abDrvError[_r] OR _abBusy[_r] OR _abProgRunning[_r] THEN
                _bAll := FALSE;
            END_IF
        END_FOR
        IF _bAll THEN
            FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
                _abResetCmd[_r] := FALSE;
                _abProdCmd[_r] := FALSE;
            END_FOR
            _bFault := FALSE;
            eError := E_CellError.ERR_NONE;
            nErrorRobot := 0;
            nErrorOp := 0;
            nErrorZone := 0;
            _bArbReset := TRUE;
            _eState := E_CellState.CS_BRINGUP;  (* then recovery (if needed) or blind homing *)
        ELSIF _tonState.Q THEN
            IF NOT _bFault THEN                 (* reset after a recovery confirm failed *)
                _bFault := TRUE;
                eError := E_CellError.ERR_RESET_FAILED;
                nErrorRobot := 0;
                FOR _r := GVL_Cell.N_ROBOTS TO 1 BY -1 DO
                    IF _abDrvError[_r] OR _abBusy[_r] OR _abProgRunning[_r] THEN
                        nErrorRobot := _r;
                    END_IF
                END_FOR
                nErrorOp := 0;
                nErrorZone := 0;
            END_IF
            _eState := E_CellState.CS_ERROR;
        END_IF
    END_IF

END_CASE

(* ============================================================================
   GRANT BYTES (PLC -> robot, DWORD LE): non-zero only while the job runs and no
   fault is latched (also covers faults decided after the arbiter in this scan)
   ============================================================================ *)
FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
    IF _abJobActive[_r] AND NOT _bFault AND NOT _bFaultScan THEN
        _dwGrant := fbArbiter.adwGrant[_r];
    ELSE
        _dwGrant := DWORD#0;
    END_IF
    adwGrant[_r] := _dwGrant;
    IF GVL_Cell.PROFILE_VW THEN
        (* Per interlock n of robot r: grant E(80+n) = the arbiter's zone bit, partner release
           E(40+n) = AND of the releases A(40+n) of the other members of that zone (the robot
           waits for it before it enters, and its online driving condition keeps watching it). *)
        FOR _z := 1 TO GVL_Cell.N_ZONES DO
            _nIl := GVL_Cell.anInterlock[(_r - 1) * GVL_Cell.N_ZONES + _z];
            IF (_nIl >= 1) AND (_nIl <= 16) THEN
                _bGrantBit := (_dwGrant AND SHL(DWORD#1, _z - 1)) <> DWORD#0;
                _bPartner := TRUE;
                FOR _q := 1 TO GVL_Cell.N_ROBOTS DO
                    IF _q <> _r THEN
                        _nIlQ := GVL_Cell.anInterlock[(_q - 1) * GVL_Cell.N_ZONES + _z];
                        IF (_nIlQ >= 1) AND (_nIlQ <= 16) THEN
                            _nIdx := GVL_Cell.VW_REL_OUT + _nIlQ - 1;
                            _nB := (_nIdx - 1) / 8;
                            _nBit := (_nIdx - 1) MOD 8;
                            IF (SHR(BYTE_TO_WORD(astIn[_q].aVassIn[_nB]), _nBit) AND WORD#1) = WORD#0 THEN
                                _bPartner := FALSE;
                            END_IF
                        END_IF
                    END_IF
                END_FOR
                _nIdx := GVL_Cell.VW_REQ_OUT + _nIl - 1;
                _nB := (_nIdx - 1) / 8;
                _nBit := (_nIdx - 1) MOD 8;
                IF _bGrantBit THEN
                    _wBits := BYTE_TO_WORD(astOut[_r].aVassOut[_nB]) OR SHL(WORD#1, _nBit);
                    astOut[_r].aVassOut[_nB] := WORD_TO_BYTE(_wBits);
                END_IF
                _nIdx := GVL_Cell.VW_REL_OUT + _nIl - 1;
                _nB := (_nIdx - 1) / 8;
                _nBit := (_nIdx - 1) MOD 8;
                IF _bPartner AND NOT _bFault AND NOT _bFaultScan THEN
                    _wBits := BYTE_TO_WORD(astOut[_r].aVassOut[_nB]) OR SHL(WORD#1, _nBit);
                    astOut[_r].aVassOut[_nB] := WORD_TO_BYTE(_wBits);
                END_IF
            END_IF
        END_FOR
    ELSE
        astOut[_r].aVassOut[GVL_Cell.GRANT_BYTE]     := DWORD_TO_BYTE(_dwGrant AND DWORD#16#FF);
        astOut[_r].aVassOut[GVL_Cell.GRANT_BYTE + 1] := DWORD_TO_BYTE(SHR(_dwGrant, 8) AND DWORD#16#FF);
        astOut[_r].aVassOut[GVL_Cell.GRANT_BYTE + 2] := DWORD_TO_BYTE(SHR(_dwGrant, 16) AND DWORD#16#FF);
        astOut[_r].aVassOut[GVL_Cell.GRANT_BYTE + 3] := DWORD_TO_BYTE(SHR(_dwGrant, 24) AND DWORD#16#FF);
    END_IF
END_FOR

(* ============================================================================
   OUTPUTS
   ============================================================================ *)
eState := _eState;
bError := _bFault;
bCycleActive := (_eState = E_CellState.CS_START_JOBS) OR (_eState = E_CellState.CS_RUNNING);
bRecoveryRequired := _bNeedRecovery;
anHolder := fbArbiter.anHolder;
anTicket := fbArbiter.anTicket;
anDone := _anDone;
