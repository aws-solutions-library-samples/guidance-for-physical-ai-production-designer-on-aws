(*
 * FB_ProgramPlayer - Implementation
 *)

(* ============================================================================
   INIT - the robot powers up at the home pose
   ============================================================================ *)
IF _bFirstCall THEN
    FOR _i := 1 TO 6 DO
        _arQ[_i] := GVL_Program.arHomeDeg[_i];
    END_FOR
    _bFirstCall := FALSE;
END_IF

_rDtCall := LIMIT(0.0, rDt, 1.0);
_rRateCall := MAX(rRate, 0.0);
_rtrigStart(CLK := bStart);

(* ============================================================================
   ABORT - stop / cancel / drives off / E-stop: REQUEST := 0, wStep := 0, hold pose
   ============================================================================ *)
IF bAbort AND bBusy THEN
    _eState := E_PlayerState.PS_IDLE;
    bBusy := FALSE;
    wStep := WORD#0;
    dwRequest := DWORD#0;
END_IF

(* ============================================================================
   CLOCK - advance the timer of the state that is active at the start of the call
   (a state entered during this call starts counting with the next call)
   ============================================================================ *)
IF _eState = E_PlayerState.PS_DELAY OR _eState = E_PlayerState.PS_HOLD THEN
    _rTimer := _rTimer + _rDtCall;
ELSIF _eState = E_PlayerState.PS_MOVE THEN
    _rT := _rT + _rDtCall * _rRateCall * _rVel;
END_IF

(* ============================================================================
   START
   ============================================================================ *)
IF _rtrigStart.Q AND NOT bBusy AND NOT bAbort THEN
    bDone := FALSE;
    bError := FALSE;
    wErrorCode := WORD#0;
    wWarningCode := WORD#0;
    wStep := WORD#0;
    dwRequest := DWORD#0;
    _nOp := 0;
    _dwMask := DWORD#0;
    _rT := 0.0;
    _rTimer := 0.0;
    IF wJob = GVL_Program.HOME_FOLGE THEN
        (* Blind joint move from the current pose to arHomeDeg: no step, no zones *)
        FOR _i := 1 TO 6 DO
            _arQ0[_i] := _arQ[_i];
        END_FOR
        _bHomeMove := TRUE;
        _nLast := 0;
        _rVel := 1.0;
        _rDur := GVL_Program.R_HOME_S;
        bBusy := TRUE;
        _eState := E_PlayerState.PS_MOVE;
    ELSIF wJob = GVL_Program.PROD_FOLGE AND GVL_Program.N_OPS > 0 THEN
        bBusy := TRUE;
        _eState := E_PlayerState.PS_NEXT_OP;
    ELSE
        wErrorCode := GVL_Program.ERR_UNKNOWN_FOLGE;
        _eState := E_PlayerState.PS_ERROR;
    END_IF
END_IF

(* ============================================================================
   STATE MACHINE - falls through zero-time transitions (bounded)
   ============================================================================ *)
_bStepSet := FALSE;
FOR _nPass := 1 TO N_MAX_TRANSITIONS DO
    _bAdvance := FALSE;

    CASE _eState OF

    E_PlayerState.PS_IDLE:
        bBusy := FALSE;
        wStep := WORD#0;
        dwRequest := DWORD#0;

    E_PlayerState.PS_NEXT_OP:
        (* wStep changes at most once per call: the head checks step + 1 per telegram *)
        IF NOT _bStepSet THEN
            _nOp := _nOp + 1;
            IF _nOp > GVL_Program.N_OPS THEN
                _eState := E_PlayerState.PS_DONE;
            ELSE
                wStep := INT_TO_WORD(_nOp);
                _bStepSet := TRUE;
                _dwMask := GVL_Program.aOps[_nOp].dwZoneMask;
                IF _dwMask <> DWORD#0 THEN
                    dwRequest := _dwMask;           (* same telegram as wStep *)
                    _eState := E_PlayerState.PS_WAIT_GRANT;
                ELSE
                    _rTimer := 0.0;
                    _eState := E_PlayerState.PS_DELAY;
                END_IF
            END_IF
            _bAdvance := TRUE;
        END_IF

    E_PlayerState.PS_WAIT_GRANT:
        IF (dwGrant AND _dwMask) = _dwMask THEN
            _rTimer := 0.0;
            _eState := E_PlayerState.PS_DELAY;
            _bAdvance := TRUE;
        END_IF

    E_PlayerState.PS_DELAY:
        IF _rTimer >= GVL_Program.aOps[_nOp].rDelay_s THEN
            (* Begin the motion of op _nOp *)
            _rT := 0.0;
            _rVel := GVL_Program.aOps[_nOp].rVelScale;
            IF _rVel <= 0.0 THEN
                _rVel := 1.0;
            END_IF
            _bHomeMove := GVL_Program.aOps[_nOp].bBlindHome;
            IF _bHomeMove THEN
                FOR _i := 1 TO 6 DO
                    _arQ0[_i] := _arQ[_i];
                END_FOR
                _rDur := GVL_Program.R_HOME_S;
                _nLast := 0;
            ELSE
                _nWp := GVL_Program.aOps[_nOp].nFirstWp;
                _nLast := GVL_Program.aOps[_nOp].nLastWp;
                IF _nWp < 1 OR _nLast < _nWp OR _nLast > GVL_Program.N_WP THEN
                    _nLast := 0;                    (* no valid trajectory: zero-length move *)
                ELSE
                    (* Start deviation: the trajectory should start at the current pose *)
                    _arQa[1] := GVL_Program.aWp[_nWp].rA1;
                    _arQa[2] := GVL_Program.aWp[_nWp].rA2;
                    _arQa[3] := GVL_Program.aWp[_nWp].rA3;
                    _arQa[4] := GVL_Program.aWp[_nWp].rA4;
                    _arQa[5] := GVL_Program.aWp[_nWp].rA5;
                    _arQa[6] := GVL_Program.aWp[_nWp].rA6;
                    FOR _i := 1 TO 6 DO
                        IF ABS(_arQ[_i] - _arQa[_i]) > GVL_Program.R_START_TOL_DEG THEN
                            wWarningCode := GVL_Program.WARN_START_DEVIATION;
                        END_IF
                    END_FOR
                END_IF
            END_IF
            _eState := E_PlayerState.PS_MOVE;
            _bAdvance := TRUE;
        END_IF

    E_PlayerState.PS_MOVE:
        _bMoveDone := FALSE;
        IF _bHomeMove THEN
            (* Blind joint move: linear in joint space over R_HOME_S *)
            IF _rDur > 0.0 THEN
                _rAlpha := LIMIT(0.0, _rT / _rDur, 1.0);
            ELSE
                _rAlpha := 1.0;
            END_IF
            FOR _i := 1 TO 6 DO
                _arQ[_i] := _arQ0[_i] + _rAlpha * (GVL_Program.arHomeDeg[_i] - _arQ0[_i]);
            END_FOR
            _bMoveDone := _rAlpha >= 1.0;
        ELSIF _nLast < 1 THEN
            _bMoveDone := TRUE;
        ELSE
            (* Trajectory: find the segment [_nWp, _nWp + 1] that contains _rT *)
            WHILE _nWp < _nLast DO
                IF GVL_Program.aWp[_nWp + 1].rT <= _rT THEN
                    _nWp := _nWp + 1;
                ELSE
                    EXIT;
                END_IF
            END_WHILE
            _arQa[1] := GVL_Program.aWp[_nWp].rA1;
            _arQa[2] := GVL_Program.aWp[_nWp].rA2;
            _arQa[3] := GVL_Program.aWp[_nWp].rA3;
            _arQa[4] := GVL_Program.aWp[_nWp].rA4;
            _arQa[5] := GVL_Program.aWp[_nWp].rA5;
            _arQa[6] := GVL_Program.aWp[_nWp].rA6;
            IF _nWp >= _nLast THEN
                FOR _i := 1 TO 6 DO
                    _arQ[_i] := _arQa[_i];
                END_FOR
                _bMoveDone := TRUE;
            ELSE
                _arQb[1] := GVL_Program.aWp[_nWp + 1].rA1;
                _arQb[2] := GVL_Program.aWp[_nWp + 1].rA2;
                _arQb[3] := GVL_Program.aWp[_nWp + 1].rA3;
                _arQb[4] := GVL_Program.aWp[_nWp + 1].rA4;
                _arQb[5] := GVL_Program.aWp[_nWp + 1].rA5;
                _arQb[6] := GVL_Program.aWp[_nWp + 1].rA6;
                _rSpan := GVL_Program.aWp[_nWp + 1].rT - GVL_Program.aWp[_nWp].rT;
                IF _rSpan > 0.0 THEN
                    _rAlpha := LIMIT(0.0, (_rT - GVL_Program.aWp[_nWp].rT) / _rSpan, 1.0);
                ELSE
                    _rAlpha := 1.0;
                END_IF
                FOR _i := 1 TO 6 DO
                    _arQ[_i] := _arQa[_i] + _rAlpha * (_arQb[_i] - _arQa[_i]);
                END_FOR
            END_IF
        END_IF
        IF _bMoveDone THEN
            IF _nOp = 0 THEN
                _eState := E_PlayerState.PS_DONE;   (* HOME_FOLGE finished *)
            ELSE
                _rTimer := 0.0;
                _eState := E_PlayerState.PS_HOLD;
            END_IF
            _bAdvance := TRUE;
        END_IF

    E_PlayerState.PS_HOLD:
        IF _rTimer >= GVL_Program.aOps[_nOp].rHold_s THEN
            IF _dwMask <> DWORD#0 THEN
                dwRequest := DWORD#0;
                _eState := E_PlayerState.PS_WAIT_RELEASE;
            ELSE
                _eState := E_PlayerState.PS_NEXT_OP;
            END_IF
            _bAdvance := TRUE;
        END_IF

    E_PlayerState.PS_WAIT_RELEASE:
        (* 4-phase: the GRANT must drop before the next REQUEST *)
        IF (dwGrant AND _dwMask) = DWORD#0 THEN
            _eState := E_PlayerState.PS_NEXT_OP;
            _bAdvance := TRUE;
        END_IF

    E_PlayerState.PS_DONE:
        bBusy := FALSE;
        bDone := TRUE;
        wStep := WORD#0;
        dwRequest := DWORD#0;
        _eState := E_PlayerState.PS_IDLE;

    E_PlayerState.PS_ERROR:
        bBusy := FALSE;
        bError := TRUE;
        wStep := WORD#0;
        dwRequest := DWORD#0;
        _eState := E_PlayerState.PS_IDLE;

    END_CASE

    IF NOT _bAdvance THEN
        EXIT;
    END_IF
END_FOR

(* ============================================================================
   PUBLISH
   ============================================================================ *)
bAtHome := TRUE;
FOR _i := 1 TO 6 DO
    IF ABS(_arQ[_i] - GVL_Program.arHomeDeg[_i]) > GVL_Program.R_AT_HOME_TOL_DEG THEN
        bAtHome := FALSE;
    END_IF
END_FOR
rA1_deg := _arQ[1];
rA2_deg := _arQ[2];
rA3_deg := _arQ[3];
rA4_deg := _arQ[4];
rA5_deg := _arQ[5];
rA6_deg := _arQ[6];
eState := _eState;
