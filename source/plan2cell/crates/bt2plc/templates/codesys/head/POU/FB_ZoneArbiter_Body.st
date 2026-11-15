(*
 * FB_ZoneArbiter - Implementation
 * Order per scan: debounce + handshake check, release, grant, fault latch.
 *)

IF bReset THEN
    FOR _z := 1 TO GVL_Cell.N_ZONES DO
        anHolder[_z] := 0;
        anTicket[_z] := GVL_Cell.anZoneFirst[_z];
    END_FOR
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        adwGrant[_r] := DWORD#0;
        _audiWaitSince[_r] := 0;
        _anGrantStep[_r] := 0;
        _awPrevStep[_r] := awStep[_r];
        _adwPrevReq[_r] := adwReq[_r];
        _atChange[_r] := tNow;
        _abStable[_r] := FALSE;
    END_FOR
    _udiScan := 0;
    bFault := FALSE;
    eFault := E_CellError.ERR_NONE;
    nFaultRobot := 0;
    nFaultZone := 0;
    bAllConsumed := (GVL_Cell.N_ZONES = 0) OR (ePolicy = E_GrantPolicy.GP_FCFS);
    RETURN;
END_IF

IF bFault THEN
    RETURN;                                 (* latched until bReset *)
END_IF

_udiScan := _udiScan + 1;
_eNew := E_CellError.ERR_NONE;
_nNewRobot := 0;
_nNewZone := 0;

(* ============================================================================
   0. DEBOUNCE / 4-PHASE HANDSHAKE
   ============================================================================ *)
FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
    IF (awStep[_r] <> _awPrevStep[_r]) OR (adwReq[_r] <> _adwPrevReq[_r]) THEN
        _atChange[_r] := tNow;
        _awPrevStep[_r] := awStep[_r];
        _adwPrevReq[_r] := adwReq[_r];
    END_IF
    _abStable[_r] := (tNow - _atChange[_r]) >= tDebounce;

    (* A correct robot changes its op index only after it has seen GRANT = 0. Profile vw-ks24
       has no 4-phase handshake: there the robot only holds its release level long enough for
       the head to see it (bt2krl emits a dwell after every release), so the check needs the
       debounce to avoid firing on scan jitter. *)
    IF (adwGrant[_r] <> DWORD#0) AND (WORD_TO_INT(awStep[_r]) <> _anGrantStep[_r])
       AND (_abStable[_r] OR NOT GVL_Cell.PROFILE_VW)
       AND (_eNew = E_CellError.ERR_NONE) THEN
        _eNew := E_CellError.ERR_HANDSHAKE;
        _nNewRobot := _r;
        _nNewZone := 0;
    END_IF
END_FOR

(* ============================================================================
   1. RELEASE - holder dropped its REQUEST bit
   ============================================================================ *)
FOR _z := 1 TO GVL_Cell.N_ZONES DO
    _rr := anHolder[_z];
    IF _rr <> 0 THEN
        _dwBit := SHL(DWORD#1, _z - 1);
        IF (adwReq[_rr] AND _dwBit) = DWORD#0 THEN
            IF abRobotOk[_rr] THEN
                anHolder[_z] := 0;
                adwGrant[_rr] := adwGrant[_rr] AND NOT _dwBit;
                IF ePolicy = E_GrantPolicy.GP_PLANNED THEN
                    anTicket[_z] := anTicket[_z] + 1;
                END_IF
            ELSIF _eNew = E_CellError.ERR_NONE THEN
                (* request vanished because the robot stopped / faulted / lost
                   connection: the zone may still be occupied - never re-grant *)
                _eNew := E_CellError.ERR_RELEASE_INVALID;
                _nNewRobot := _rr;
                _nNewZone := _z;
            END_IF
        END_IF
    END_IF
END_FOR

(* Permit bit: not a mutex, released as soon as REQUEST bit 31 drops *)
FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
    IF ((adwGrant[_r] AND GVL_Cell.PERMIT_MASK) <> DWORD#0)
       AND ((adwReq[_r] AND GVL_Cell.PERMIT_MASK) = DWORD#0) THEN
        adwGrant[_r] := adwGrant[_r] AND NOT GVL_Cell.PERMIT_MASK;
    END_IF
END_FOR

(* ============================================================================
   2. GRANT
   ============================================================================ *)
FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
    _abPicked[_r] := FALSE;
    IF (adwReq[_r] = DWORD#0) OR (adwGrant[_r] <> DWORD#0) THEN
        _audiWaitSince[_r] := 0;
    ELSIF _audiWaitSince[_r] = 0 THEN
        _audiWaitSince[_r] := _udiScan;
    END_IF
END_FOR
_dwReserved := DWORD#0;

FOR _k := 1 TO GVL_Cell.N_ROBOTS DO
    (* --- visiting order: PLANNED = robot index, FCFS = oldest request first --- *)
    _rSel := 0;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        IF NOT _abPicked[_r] THEN
            IF _rSel = 0 THEN
                _rSel := _r;
            ELSIF (ePolicy = E_GrantPolicy.GP_FCFS) AND (_audiWaitSince[_r] <> 0) THEN
                IF (_audiWaitSince[_rSel] = 0) OR (_audiWaitSince[_r] < _audiWaitSince[_rSel]) THEN
                    _rSel := _r;
                END_IF
            END_IF
        END_IF
    END_FOR
    _rr := _rSel;
    _abPicked[_rr] := TRUE;

    (* --- expected mask of the op the robot reports --- *)
    _nStep := WORD_TO_INT(awStep[_rr]);
    _iOp := 0;
    _dwExp := DWORD#0;
    IF (_nStep >= 1) AND (_nStep <= GVL_Cell.anRobotOpCount[_rr]) THEN
        _iOp := GVL_Cell.anRobotFirstOp[_rr] + _nStep - 1;
        _dwExp := GVL_Cell.aOps[_iOp].dwZoneMask;
    END_IF
    _dwZones := _dwExp AND NOT GVL_Cell.PERMIT_MASK;

    _bPending := bEnable AND abRobotOk[_rr] AND (adwReq[_rr] <> DWORD#0) AND (adwGrant[_rr] = DWORD#0);

    (* --- request must equal the op mask (debounced) --- *)
    IF _bPending AND _abStable[_rr] AND (adwReq[_rr] <> _dwExp)
       AND (_eNew = E_CellError.ERR_NONE) THEN
        _eNew := E_CellError.ERR_REQ_MISMATCH;
        _nNewRobot := _rr;
        _nNewZone := 0;
    END_IF

    (* --- a holder must not raise bits it was not granted --- *)
    IF (adwGrant[_rr] <> DWORD#0) AND ((adwReq[_rr] AND NOT adwGrant[_rr]) <> DWORD#0)
       AND (_eNew = E_CellError.ERR_NONE) THEN
        _eNew := E_CellError.ERR_UNEXPECTED_REQUEST;
        _nNewRobot := _rr;
        _nNewZone := 0;
    END_IF

    IF _bPending AND (adwReq[_rr] = _dwExp) THEN
        (* --- zones: all free, (planned) ticket = this op, (FCFS) not reserved --- *)
        _bOk := TRUE;
        FOR _z := 1 TO GVL_Cell.N_ZONES DO
            _dwBit := SHL(DWORD#1, _z - 1);
            IF (_dwZones AND _dwBit) <> DWORD#0 THEN
                IF anHolder[_z] <> 0 THEN
                    _bOk := FALSE;
                ELSIF ePolicy = E_GrantPolicy.GP_PLANNED THEN
                    _nTicket := anTicket[_z];
                    _nLastTicket := GVL_Cell.anZoneFirst[_z] + GVL_Cell.anZoneCount[_z] - 1;
                    IF (_nTicket < GVL_Cell.anZoneFirst[_z]) OR (_nTicket > _nLastTicket) THEN
                        _bOk := FALSE;              (* zone consumed *)
                        IF _abStable[_rr] AND (_eNew = E_CellError.ERR_NONE) THEN
                            _eNew := E_CellError.ERR_TICKET_SKIPPED;
                            _nNewRobot := _rr;
                            _nNewZone := _z;
                        END_IF
                    ELSIF (GVL_Cell.aTickets[_nTicket].nRobot <> _rr)
                          OR (GVL_Cell.aTickets[_nTicket].nOp <> _nStep) THEN
                        _bOk := FALSE;              (* not this op's turn yet *)
                        IF (GVL_Cell.aTickets[_nTicket].nRobot = _rr) AND _abStable[_rr]
                           AND (_eNew = E_CellError.ERR_NONE) THEN
                            _eNew := E_CellError.ERR_TICKET_SKIPPED;
                            _nNewRobot := _rr;
                            _nNewZone := _z;
                        END_IF
                    END_IF
                ELSIF (_dwReserved AND _dwBit) <> DWORD#0 THEN
                    _bOk := FALSE;                  (* FCFS: reserved by an older requester *)
                END_IF
            END_IF
        END_FOR

        (* --- permit: every predecessor op done --- *)
        _bPermitOk := TRUE;
        IF ((_dwExp AND GVL_Cell.PERMIT_MASK) <> DWORD#0) AND (_iOp >= 1) THEN
            (* GVL array elements are copied to locals before use as call arguments *)
            _nPredFirst := GVL_Cell.aOps[_iOp].nPredFirst;
            _nPredLast := _nPredFirst + GVL_Cell.aOps[_iOp].nPredCount - 1;
            _nPredFirst := MAX(1, _nPredFirst);
            _nPredLast := MIN(GVL_Cell.N_PREDS, _nPredLast);
            FOR _p := _nPredFirst TO _nPredLast DO
                _q := GVL_Cell.aPreds[_p].nRobot;
                IF (_q < 1) OR (_q > GVL_Cell.N_ROBOTS) THEN
                    _bPermitOk := FALSE;
                ELSIF anDone[_q] < GVL_Cell.aPreds[_p].nOp THEN
                    _bPermitOk := FALSE;
                END_IF
            END_FOR
        END_IF

        IF _bOk AND _bPermitOk THEN
            FOR _z := 1 TO GVL_Cell.N_ZONES DO
                IF (_dwZones AND SHL(DWORD#1, _z - 1)) <> DWORD#0 THEN
                    anHolder[_z] := _rr;
                END_IF
            END_FOR
            adwGrant[_rr] := _dwExp;
            _anGrantStep[_rr] := _nStep;
            _audiWaitSince[_rr] := 0;
        ELSIF _bPermitOk AND (ePolicy = E_GrantPolicy.GP_FCFS) THEN
            (* older requester blocked only by occupied / reserved zones: reserve them
               (a requester still waiting for its permit reserves nothing - no deadlock) *)
            _dwReserved := _dwReserved OR _dwZones;
        END_IF
    END_IF
END_FOR

(* ============================================================================
   3. FAULT LATCH / CYCLE COMPLETENESS
   ============================================================================ *)
IF _eNew <> E_CellError.ERR_NONE THEN
    bFault := TRUE;
    eFault := _eNew;
    nFaultRobot := _nNewRobot;
    nFaultZone := _nNewZone;
    FOR _r := 1 TO GVL_Cell.N_ROBOTS DO
        adwGrant[_r] := DWORD#0;
    END_FOR
END_IF

bAllConsumed := TRUE;
FOR _z := 1 TO GVL_Cell.N_ZONES DO
    IF anHolder[_z] <> 0 THEN
        bAllConsumed := FALSE;
    ELSIF (ePolicy = E_GrantPolicy.GP_PLANNED)
          AND (anTicket[_z] <= GVL_Cell.anZoneFirst[_z] + GVL_Cell.anZoneCount[_z] - 1) THEN
        bAllConsumed := FALSE;
    END_IF
END_FOR
