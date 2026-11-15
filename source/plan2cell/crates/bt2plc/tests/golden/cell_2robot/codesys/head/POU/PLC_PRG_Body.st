(*
 * PLC_PRG - Implementation
 *)

IF _bFirstScan THEN
    bAutoRun := GVL_Cell.AUTO_RUN;
    _bFirstScan := FALSE;
END_IF

(* ============================================================================
   INPUTS: GVL_IO -> fbCell (robot -> PLC)
   ============================================================================ *)

(* 1: R1 - robot1 - kuka_robi1 (kuka-robi1, 192.168.100.11) *)
_astIn[1].abySafeIn[0] := GVL_IO.bySafeIn0_R1;
_astIn[1].abySafeIn[1] := GVL_IO.bySafeIn1_R1;
_astIn[1].abySafeIn[2] := GVL_IO.bySafeIn2_R1;
_astIn[1].abySafeIn[3] := GVL_IO.bySafeIn3_R1;
_astIn[1].abySafeIn[4] := GVL_IO.bySafeIn4_R1;
_astIn[1].abySafeIn[5] := GVL_IO.bySafeIn5_R1;
_astIn[1].abySafeIn[6] := GVL_IO.bySafeIn6_R1;
_astIn[1].abySafeIn[7] := GVL_IO.bySafeIn7_R1;
_astIn[1].aVassIn := GVL_IO.aVassInputData_R1;


(* ============================================================================
   CELL
   ============================================================================ *)
fbCell(
    bEnable             := TRUE,
    bAutoRun            := bAutoRun,
    bStart              := bStart,
    bStop               := bStop,
    bReset              := bReset,
    bRecoveryConfirm    := bRecoveryConfirm,
    astIn               := _astIn,
    eState              => eCellState,
    eError              => eCellError,
    bError              => bCellError,
    bRecoveryRequired   => bRecoveryRequired,
    udiCycles           => udiCycles);

(* ============================================================================
   OUTPUTS: fbCell -> GVL_IO (PLC -> robot)
   ============================================================================ *)

(* 1: R1 - robot1 *)
GVL_IO.bySafeOut0_R1 := fbCell.astOut[1].abySafeOut[0];
GVL_IO.bySafeOut1_R1 := fbCell.astOut[1].abySafeOut[1];
GVL_IO.bySafeOut2_R1 := fbCell.astOut[1].abySafeOut[2];
GVL_IO.bySafeOut3_R1 := fbCell.astOut[1].abySafeOut[3];
GVL_IO.bySafeOut4_R1 := fbCell.astOut[1].abySafeOut[4];
GVL_IO.bySafeOut5_R1 := fbCell.astOut[1].abySafeOut[5];
GVL_IO.bySafeOut6_R1 := fbCell.astOut[1].abySafeOut[6];
GVL_IO.bySafeOut7_R1 := fbCell.astOut[1].abySafeOut[7];
GVL_IO.aVassOutputData_R1 := fbCell.astOut[1].aVassOut;


(* One-shot operator buttons *)
bStart := FALSE;
bStop := FALSE;
bReset := FALSE;
bRecoveryConfirm := FALSE;
