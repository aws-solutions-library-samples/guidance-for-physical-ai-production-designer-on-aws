(*
 * FB_VASS_DeviceSimulator - Implementation
 *
 * Reads commands from aVassOutputData (PLC -> Robot, 32 bytes) and abySafeOut (8 bytes).
 * Writes status to aVassInputData (Robot -> PLC, 32 bytes) and abySafeIn (8 bytes).
 *)

IF NOT bEnable THEN
    bActive := FALSE;
    RETURN;
END_IF

bActive := TRUE;


(* ============================================================================
   SECTION 1: READ PLC COMMANDS from aVassOutputData[0..31]
   Byte 0-1 = Control word, Byte 2-3 = Job number, PLAN_ID / GRANT per GVL_Program
   ============================================================================ *)

(* Decode the control bits into BOOLs (a bare bit access is never passed as a call argument) *)
_bCmdDrivesOn       := aVassOutputData[0].0;
_bCmdProgStart      := aVassOutputData[0].1;
_bCmdProgStop       := aVassOutputData[0].2;
_bCmdProgCancel     := aVassOutputData[0].3;
_bCmdModeExt        := aVassOutputData[0].4;
_bCmdReset          := aVassOutputData[0].5;
_bCmdHome           := aVassOutputData[0].6;
_bCmdJobReq         := aVassOutputData[1].0;
_bCmdGripOpen       := aVassOutputData[1].1;
_bCmdGripClose      := aVassOutputData[1].2;
_bCmdSpeedOverride  := aVassOutputData[1].4;
_bSafeCmdDriveEnable := abySafeOut[0].0;
_bSafeCmdEStop      := abySafeOut[0].1;
_bSafeCmdSS1        := abySafeOut[0].2;

(* Edge detection on control signals - output byte 0 *)
_rtrigDrivesOn(CLK  := _bCmdDrivesOn);
_rtrigProgStart(CLK := _bCmdProgStart);
_rtrigProgStop(CLK  := _bCmdProgStop);
_rtrigReset(CLK     := _bCmdReset);
_rtrigHome(CLK      := _bCmdHome);

(* Output byte 1 *)
_rtrigJobReq(CLK    := _bCmdJobReq);
_rtrigGripOpen(CLK  := _bCmdGripOpen);
_rtrigGripClose(CLK := _bCmdGripClose);

(* Safety edge detection from safe output byte 0 *)
_rtrigSafeStop1(CLK := _bSafeCmdSS1);
_rtrigSafeEStop(CLK := _bSafeCmdEStop);

(* Requested job number: bytes 2-3 WORD LE, or byte 2 only (GVL_Program.JOB_NUMBER_BYTE) *)
IF GVL_Program.JOB_NUMBER_BYTE THEN
    _wRequestedJob := BYTE_TO_WORD(aVassOutputData[2]);
ELSE
    _wRequestedJob := BYTE_TO_WORD(aVassOutputData[2])
                      OR SHL(BYTE_TO_WORD(aVassOutputData[3]), 8);
END_IF

(* Zone GRANT: DWORD LE at bytes GVL_Program.GRANT_BYTE..+3 *)
_nIdx := GVL_Program.GRANT_BYTE;
_dwGrant := BYTE_TO_DWORD(aVassOutputData[_nIdx])
            OR SHL(BYTE_TO_DWORD(aVassOutputData[_nIdx + 1]), 8)
            OR SHL(BYTE_TO_DWORD(aVassOutputData[_nIdx + 2]), 16)
            OR SHL(BYTE_TO_DWORD(aVassOutputData[_nIdx + 3]), 24);


(* ============================================================================
   SECTION 2: SAFETY LOGIC (PROFIsafe Slot 1)
   Reads abySafeOut, status is written to abySafeIn in section 4
   ============================================================================ *)

(* Process safe outputs from PLC - byte 0 *)
_bSafe_DrivesOK := _bSafeCmdDriveEnable AND _bSafe_EStopOK AND NOT _bSafe_Fault;

(* E-Stop handling *)
IF _bSafeCmdEStop THEN  (* External E-Stop *)
    _bSafe_EStopOK := FALSE;
    _bSafe_STO := TRUE;
    _bDrivesPowered := FALSE;
    _bProgRunning := FALSE;
    eProgramState := E_ProgramState.PRG_STOPPED;
END_IF

(* Safe Stop 1 request *)
IF _bSafeCmdSS1 AND _bProgRunning THEN
    _bSafe_SS1 := TRUE;
    _bProgRunning := FALSE;
    _bSafe_Stopped := TRUE;
    eProgramState := E_ProgramState.PRG_STOPPED;
END_IF

(* Recovery from safety state *)
IF NOT _bSafeCmdEStop AND NOT _bSafeCmdSS1 THEN
    IF _bCmdReset THEN  (* Reset bit *)
        _bSafe_EStopOK := TRUE;
        _bSafe_STO := FALSE;
        _bSafe_SS1 := FALSE;
        _bSafe_Stopped := FALSE;
        _bSafe_Fault := FALSE;
    END_IF
END_IF

(* Overall safety status *)
bSafetyOK := _bSafe_DrivesOK AND _bSafe_EStopOK AND _bSafe_GatesClosed AND NOT _bSafe_Fault;
_bSafe_GatesClosed := TRUE; (* Always closed in simulation *)


(* ============================================================================
   SECTION 3: STANDARD I/O LOGIC (VASS Telegram Slot 2)
   ============================================================================ *)

(* --- Drive Power Control --- *)
_tonDriveStartup(IN := _bCmdDrivesOn AND bSafetyOK AND NOT _bDrivesPowered, PT := T#2S);
IF _tonDriveStartup.Q THEN
    _bDrivesPowered := TRUE;
    _bSafe_Stopped := FALSE;
END_IF

IF NOT _bCmdDrivesOn OR NOT bSafetyOK THEN
    _bDrivesPowered := FALSE;
    _bProgRunning := FALSE;
    IF eProgramState = E_ProgramState.PRG_RUNNING THEN
        eProgramState := E_ProgramState.PRG_STOPPED;
    END_IF
END_IF

(* --- Mode Control --- *)
_bInExtMode := _bCmdModeExt AND _bDrivesPowered AND bSafetyOK;
IF _bInExtMode THEN
    eCurrentMode := E_RobotMode.MODE_EXT;
ELSE
    eCurrentMode := E_RobotMode.MODE_T1;
END_IF

(* --- Ready State --- *)
_bReady := _bDrivesPowered
           AND _bInExtMode
           AND NOT _bError
           AND NOT _bProgRunning
           AND bSafetyOK;

(* --- Warning reset: any Reset edge clears a pending warning (no error needed) --- *)
IF _rtrigReset.Q THEN
    _wWarningCode := WORD#0;
END_IF

(* --- Error Reset (Reset held 500 ms) --- *)
_tonErrorReset(IN := _bCmdReset AND _bError, PT := T#500MS);
IF _tonErrorReset.Q THEN
    _bError := FALSE;
    _wErrorCode := WORD#0;
    _wWarningCode := WORD#0;
    IF eProgramState = E_ProgramState.PRG_ERROR THEN
        eProgramState := E_ProgramState.PRG_IDLE;
    END_IF
END_IF

(* --- Job Request Handling --- *)
IF _rtrigJobReq.Q AND _bReady THEN
    _wActiveJob := _wRequestedJob;
    _bJobAcknowledged := TRUE;
    _bProgFinished := FALSE;
    (* PLAN_ID: WORD LE at bytes GVL_Program.PLAN_ID_BYTE..+1 (production job only) *)
    _nIdx := GVL_Program.PLAN_ID_BYTE;
    _wPlanId := BYTE_TO_WORD(aVassOutputData[_nIdx])
                OR SHL(BYTE_TO_WORD(aVassOutputData[_nIdx + 1]), 8);
END_IF

IF NOT _bCmdJobReq THEN
    _bJobAcknowledged := FALSE;
END_IF

(* --- Program Start --- *)
_bStartOk := FALSE;
IF _rtrigProgStart.Q AND _bReady AND _wActiveJob <> WORD#0 THEN
    _wPlayerJob := _wActiveJob;
    IF _wActiveJob = GVL_Program.HOME_FOLGE THEN
        _bStartOk := TRUE;
    ELSIF _wActiveJob = GVL_Program.PROD_FOLGE AND GVL_Program.N_OPS > 0 THEN
        IF _wPlanId = GVL_Program.PLAN_ID THEN
            _bStartOk := TRUE;
        ELSE
            _bError := TRUE;
            _wErrorCode := GVL_Program.ERR_PLAN_MISMATCH;
            eProgramState := E_ProgramState.PRG_ERROR;
        END_IF
    ELSE
        _bError := TRUE;
        _wErrorCode := GVL_Program.ERR_UNKNOWN_FOLGE;
        eProgramState := E_ProgramState.PRG_ERROR;
    END_IF
ELSIF _rtrigHome.Q AND _bReady AND NOT _bAtHome THEN
    (* HomeRequest [0].6 edge while Ready: behaves like HOME_FOLGE *)
    _wPlayerJob := GVL_Program.HOME_FOLGE;
    _bStartOk := TRUE;
END_IF

IF _bStartOk THEN
    _bProgRunning := TRUE;
    _bProgFinished := FALSE;
    _bInPosition := FALSE;
    _bSafe_Stopped := FALSE;
    _bPlayerStart := TRUE;
    eProgramState := E_ProgramState.PRG_RUNNING;
END_IF

(* --- Program Stop/Cancel (level) --- *)
IF (_bCmdProgStop OR _bCmdProgCancel) AND _bProgRunning THEN
    _bProgRunning := FALSE;
    _bProgFinished := FALSE;
    _bInPosition := TRUE;
    eProgramState := E_ProgramState.PRG_STOPPED;
END_IF

(* --- Program Execution: FB_ProgramPlayer --- *)
(* Any stop path above (Stop/Cancel, drives off, E-stop, SS1) aborts the player in this scan:
   REQUEST := 0, step := 0, the pose is held. *)
_rRate := UINT_TO_REAL(WORD_TO_UINT(_wSpeed)) / 100.0;
_fbPlayer(
    bStart  := _bPlayerStart,
    wJob    := _wPlayerJob,
    bAbort  := NOT _bProgRunning,
    dwGrant := _dwGrant,
    rRate   := _rRate,
    rDt     := rDt
);
_bPlayerStart := FALSE;

IF _bProgRunning THEN
    IF _fbPlayer.bError THEN
        _bProgRunning := FALSE;
        _bError := TRUE;
        _wErrorCode := _fbPlayer.wErrorCode;
        eProgramState := E_ProgramState.PRG_ERROR;
    ELSIF _fbPlayer.bDone THEN
        (* Same scan as step := 0: ProgRunning drops and ProgFinished rises together *)
        _bProgRunning := FALSE;
        _bProgFinished := TRUE;
        _bInPosition := TRUE;
        eProgramState := E_ProgramState.PRG_FINISHED;
    END_IF
END_IF

_wCurrentStep := _fbPlayer.wStep;
_dwRequest := _fbPlayer.dwRequest;
_bAtHome := _fbPlayer.bAtHome;

(* Player warnings are copied on change, so an error reset clears them *)
IF _fbPlayer.wWarningCode <> _wPlayerWarning THEN
    _wPlayerWarning := _fbPlayer.wWarningCode;
    IF _wPlayerWarning <> WORD#0 THEN
        _wWarningCode := _wPlayerWarning;
    END_IF
END_IF

(* --- Gripper Simulation --- *)
_tonGripperAction(IN := _bCmdGripClose AND NOT _bGripperClosed, PT := T#500MS);
IF _tonGripperAction.Q THEN
    _bGripperClosed := TRUE;
    _bPartPresent := TRUE;
END_IF

IF _bCmdGripOpen THEN
    _bGripperClosed := FALSE;
    _bPartPresent := FALSE;
END_IF

(* --- Speed Override (bytes 8-9 WORD LE, applied with [1].4) --- *)
_wSpeedCmd := BYTE_TO_WORD(aVassOutputData[8]) OR SHL(BYTE_TO_WORD(aVassOutputData[9]), 8);
IF _bCmdSpeedOverride AND WORD_TO_UINT(_wSpeedCmd) > 0 AND WORD_TO_UINT(_wSpeedCmd) <= 100 THEN
    _wSpeed := _wSpeedCmd;
END_IF


(* ============================================================================
   SECTION 4: WRITE SAFETY STATUS to abySafeIn[0..7]
   ============================================================================ *)

(* Byte 0: Safe drive/motion status *)
abySafeIn[0].0 := _bSafe_DrivesOK;
abySafeIn[0].1 := _bSafe_Stopped;
abySafeIn[0].2 := FALSE; (* Reduced speed *)
abySafeIn[0].3 := NOT _bSafe_Fault; (* Axis monitor OK *)
abySafeIn[0].4 := TRUE;  (* Tool monitor OK *)
abySafeIn[0].5 := TRUE;  (* Workspace OK *)
abySafeIn[0].6 := FALSE;
abySafeIn[0].7 := FALSE;

(* Byte 1: Gate/protection status *)
abySafeIn[1].0 := _bSafe_GatesClosed;  (* Gate 1 *)
abySafeIn[1].1 := _bSafe_GatesClosed;  (* Gate 2 *)
abySafeIn[1].2 := _bSafe_GatesClosed;  (* Gate 3 *)
abySafeIn[1].3 := TRUE;   (* Light curtain OK *)
abySafeIn[1].4 := _bSafe_EStopOK;
abySafeIn[1].5 := FALSE;  (* Enabling device *)
abySafeIn[1].6 := TRUE;   (* Operator safe *)
abySafeIn[1].7 := FALSE;

(* Byte 2: Extended safe functions *)
abySafeIn[2].0 := _bSafe_STO;
abySafeIn[2].1 := _bSafe_SS1;
abySafeIn[2].2 := FALSE;  (* SS2 *)
abySafeIn[2].3 := _bSafe_Stopped AND NOT _bSafe_STO;  (* SOS *)
abySafeIn[2].4 := FALSE;  (* SLS *)
abySafeIn[2].5 := FALSE;  (* SLP *)
abySafeIn[2].6 := FALSE;
abySafeIn[2].7 := _bSafe_Fault;

(* Bytes 3-7: zero *)
abySafeIn[3] := BYTE#0;
abySafeIn[4] := BYTE#0;
abySafeIn[5] := BYTE#0;
abySafeIn[6] := BYTE#0;
abySafeIn[7] := BYTE#0;


(* ============================================================================
   SECTION 5: WRITE VASS STATUS to aVassInputData[0..31]
   ============================================================================ *)

(* Unused bytes are zero; bytes 16-27 (joint angles) are filled by PLC_PRG *)
FOR _i := 0 TO 31 DO
    aVassInputData[_i] := BYTE#0;
END_FOR

(* Status Word - Byte 0 *)
aVassInputData[0].0 := _bDrivesPowered;
aVassInputData[0].1 := _bProgRunning;
aVassInputData[0].2 := _bAtHome;
aVassInputData[0].3 := _bInPosition;
aVassInputData[0].4 := _bInExtMode;
aVassInputData[0].5 := _bReady;
aVassInputData[0].6 := _bError;
aVassInputData[0].7 := _wWarningCode <> WORD#0;

(* Status Word - Byte 1 *)
aVassInputData[1].0 := _bSafe_GatesClosed;
aVassInputData[1].1 := _bSafe_EStopOK;
aVassInputData[1].2 := _bProgFinished;
aVassInputData[1].3 := _bJobAcknowledged;
aVassInputData[1].4 := _bGripperClosed;
aVassInputData[1].5 := _bPartPresent;
aVassInputData[1].6 := TRUE;  (* Periph ready *)
aVassInputData[1].7 := FALSE; (* Maint not required *)

(* Active job number echo - Bytes 2-3 (byte 2 only in byte mode, byte 3 = 0) *)
aVassInputData[2] := WORD_TO_BYTE(_wActiveJob AND WORD#16#FF);
IF NOT GVL_Program.JOB_NUMBER_BYTE THEN
    aVassInputData[3] := WORD_TO_BYTE(SHR(_wActiveJob, 8));
END_IF

(* Speed actual - Bytes 6-7 *)
aVassInputData[6] := WORD_TO_BYTE(_wSpeed AND WORD#16#FF);
aVassInputData[7] := WORD_TO_BYTE(SHR(_wSpeed, 8));

(* Tool number - Bytes 8-9 *)
aVassInputData[8] := WORD_TO_BYTE(_wToolNr AND WORD#16#FF);
aVassInputData[9] := WORD_TO_BYTE(SHR(_wToolNr, 8));

(* Error code - Bytes 10-11 *)
aVassInputData[10] := WORD_TO_BYTE(_wErrorCode AND WORD#16#FF);
aVassInputData[11] := WORD_TO_BYTE(SHR(_wErrorCode, 8));

(* Warning code - Bytes 12-13 *)
aVassInputData[12] := WORD_TO_BYTE(_wWarningCode AND WORD#16#FF);
aVassInputData[13] := WORD_TO_BYTE(SHR(_wWarningCode, 8));

(* Position X - Bytes 14-15 *)
aVassInputData[14] := WORD_TO_BYTE(INT_TO_WORD(_iPositionX) AND WORD#16#FF);
aVassInputData[15] := WORD_TO_BYTE(SHR(INT_TO_WORD(_iPositionX), 8));

(* Bytes 16-27: A129_224_Option area - RESERVED for the joint angle encoder
   (PLC_PRG -> fbJointEncoder, 6x INT big-endian = A1..A6 at 0.02 deg/count) *)

(* Current step (op index) - WORD LE at bytes GVL_Program.STEP_BYTE..+1 *)
_nIdx := GVL_Program.STEP_BYTE;
aVassInputData[_nIdx] := WORD_TO_BYTE(_wCurrentStep AND WORD#16#FF);
aVassInputData[_nIdx + 1] := WORD_TO_BYTE(SHR(_wCurrentStep, 8));

(* Zone REQUEST - DWORD LE at bytes GVL_Program.REQ_BYTE..+3 *)
_nIdx := GVL_Program.REQ_BYTE;
aVassInputData[_nIdx] := DWORD_TO_BYTE(_dwRequest AND DWORD#16#FF);
aVassInputData[_nIdx + 1] := DWORD_TO_BYTE(SHR(_dwRequest, 8) AND DWORD#16#FF);
aVassInputData[_nIdx + 2] := DWORD_TO_BYTE(SHR(_dwRequest, 16) AND DWORD#16#FF);
aVassInputData[_nIdx + 3] := DWORD_TO_BYTE(SHR(_dwRequest, 24));


(* ============================================================================
   SECTION 6: OUTPUTS (joint angles, diagnostics)
   ============================================================================ *)
rA1_deg := _fbPlayer.rA1_deg;
rA2_deg := _fbPlayer.rA2_deg;
rA3_deg := _fbPlayer.rA3_deg;
rA4_deg := _fbPlayer.rA4_deg;
rA5_deg := _fbPlayer.rA5_deg;
rA6_deg := _fbPlayer.rA6_deg;

wActiveJob := _wActiveJob;
wCurrentStep := _wCurrentStep;
wErrorCode := _wErrorCode;
dwRequest := _dwRequest;
dwGrant := _dwGrant;
ePlayerState := _fbPlayer.eState;
