(* ============================================================================
   INPUTS: copy the head PLC commands into the local images
   ============================================================================ *)
aVassOutputData := GVL_IO.aVassOutputData;
abySafeOut[0] := GVL_IO.bySafeOut0;
abySafeOut[1] := GVL_IO.bySafeOut1;
abySafeOut[2] := GVL_IO.bySafeOut2;
abySafeOut[3] := GVL_IO.bySafeOut3;
abySafeOut[4] := GVL_IO.bySafeOut4;
abySafeOut[5] := GVL_IO.bySafeOut5;
abySafeOut[6] := GVL_IO.bySafeOut6;
abySafeOut[7] := GVL_IO.bySafeOut7;


(* ============================================================================
   SIMULATION
   ============================================================================ *)

(* --- Call PROFIsafe simulator (Slot 1) --- *)
fbSafetySimulator(
    bEnable         := bSimulatorEnable,
    dwF_Source_Add  := dwF_Source_Add,
    dwF_Dest_Add    := dwF_Dest_Add,
    tF_WD_Time      := tF_WD_Time,
    abySafeOut      := abySafeOut,
    bCommOK         => bSafeCommOK
);

(* --- Call VASS simulator (Slot 2) with the program player --- *)
fbVassSimulator(
    bEnable         := bSimulatorEnable,
    wRobotID        := GVL_Program.ROBOT_INDEX,
    aVassOutputData := aVassOutputData,
    abySafeOut      := abySafeOut,
    rDt             := GVL_Program.R_TASK_S,
    bActive         => bSimActive,
    eCurrentMode    => eMode,
    eProgramState   => eProgState,
    bSafetyOK       => bSafetyHealthy
);
aVassInputData := fbVassSimulator.aVassInputData;
abySafeIn := fbVassSimulator.abySafeIn;
wActiveJob := fbVassSimulator.wActiveJob;
wCurrentStep := fbVassSimulator.wCurrentStep;
dwRequest := fbVassSimulator.dwRequest;
dwGrant := fbVassSimulator.dwGrant;
ePlayerState := fbVassSimulator.ePlayerState;

(* --- Joint angles from the player -> VASS Option area (bytes 16-27, big-endian) --- *)
fbJointEncoder(
    rA1_deg         := fbVassSimulator.rA1_deg,
    rA2_deg         := fbVassSimulator.rA2_deg,
    rA3_deg         := fbVassSimulator.rA3_deg,
    rA4_deg         := fbVassSimulator.rA4_deg,
    rA5_deg         := fbVassSimulator.rA5_deg,
    rA6_deg         := fbVassSimulator.rA6_deg
);
FOR _i := 0 TO 11 DO
    aVassInputData[16 + _i] := fbJointEncoder.abyOption[_i];
END_FOR

(* Monitor: decode back for quick verification in debug view *)
rMonitor_A1_deg := INT_TO_REAL(fbJointEncoder.iA1_count) * 0.02;
rMonitor_A2_deg := INT_TO_REAL(fbJointEncoder.iA2_count) * 0.02;
rMonitor_A3_deg := INT_TO_REAL(fbJointEncoder.iA3_count) * 0.02;
rMonitor_A4_deg := INT_TO_REAL(fbJointEncoder.iA4_count) * 0.02;
rMonitor_A5_deg := INT_TO_REAL(fbJointEncoder.iA5_count) * 0.02;
rMonitor_A6_deg := INT_TO_REAL(fbJointEncoder.iA6_count) * 0.02;

(* --- Debug: Error injection --- *)
(* Set bTest_InjectError := TRUE in online/debug view to simulate a robot error *)
IF bTest_InjectError THEN
    aVassInputData[0] := aVassInputData[0] OR BYTE#16#40;   (* Set bit 6 = Error *)
    aVassInputData[10] := WORD_TO_BYTE(wTest_ErrorCode AND WORD#16#FF);
    aVassInputData[11] := WORD_TO_BYTE(SHR(wTest_ErrorCode, 8));
END_IF

IF bTest_InjectSafetyFault THEN
    abySafeIn[2] := abySafeIn[2] OR BYTE#16#80;     (* Set bit 7 = Fault *)
    abySafeIn[0] := abySafeIn[0] AND BYTE#16#FE;    (* Clear bit 0 = Drives NOT OK *)
END_IF


(* ============================================================================
   OUTPUTS: write the robot status back to GVL_IO (whole array / scalars)
   ============================================================================ *)
GVL_IO.aVassInputData := aVassInputData;
GVL_IO.bySafeIn0 := abySafeIn[0];
GVL_IO.bySafeIn1 := abySafeIn[1];
GVL_IO.bySafeIn2 := abySafeIn[2];
GVL_IO.bySafeIn3 := abySafeIn[3];
GVL_IO.bySafeIn4 := abySafeIn[4];
GVL_IO.bySafeIn5 := abySafeIn[5];
GVL_IO.bySafeIn6 := abySafeIn[6];
GVL_IO.bySafeIn7 := abySafeIn[7];
