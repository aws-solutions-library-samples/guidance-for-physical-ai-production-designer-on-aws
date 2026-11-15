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
{{#robots}}
(* {{r.i}}: {{r.tag}} - {{r.ns}} - {{r.device_name}} ({{r.station}}, {{r.ip}}) *)
_astIn[{{r.i}}].abySafeIn[0] := GVL_IO.bySafeIn0_{{r.tag}};
_astIn[{{r.i}}].abySafeIn[1] := GVL_IO.bySafeIn1_{{r.tag}};
_astIn[{{r.i}}].abySafeIn[2] := GVL_IO.bySafeIn2_{{r.tag}};
_astIn[{{r.i}}].abySafeIn[3] := GVL_IO.bySafeIn3_{{r.tag}};
_astIn[{{r.i}}].abySafeIn[4] := GVL_IO.bySafeIn4_{{r.tag}};
_astIn[{{r.i}}].abySafeIn[5] := GVL_IO.bySafeIn5_{{r.tag}};
_astIn[{{r.i}}].abySafeIn[6] := GVL_IO.bySafeIn6_{{r.tag}};
_astIn[{{r.i}}].abySafeIn[7] := GVL_IO.bySafeIn7_{{r.tag}};
_astIn[{{r.i}}].aVassIn := GVL_IO.aVassInputData_{{r.tag}};
{{/robots}}

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
{{#robots}}
(* {{r.i}}: {{r.tag}} - {{r.ns}} *)
GVL_IO.bySafeOut0_{{r.tag}} := fbCell.astOut[{{r.i}}].abySafeOut[0];
GVL_IO.bySafeOut1_{{r.tag}} := fbCell.astOut[{{r.i}}].abySafeOut[1];
GVL_IO.bySafeOut2_{{r.tag}} := fbCell.astOut[{{r.i}}].abySafeOut[2];
GVL_IO.bySafeOut3_{{r.tag}} := fbCell.astOut[{{r.i}}].abySafeOut[3];
GVL_IO.bySafeOut4_{{r.tag}} := fbCell.astOut[{{r.i}}].abySafeOut[4];
GVL_IO.bySafeOut5_{{r.tag}} := fbCell.astOut[{{r.i}}].abySafeOut[5];
GVL_IO.bySafeOut6_{{r.tag}} := fbCell.astOut[{{r.i}}].abySafeOut[6];
GVL_IO.bySafeOut7_{{r.tag}} := fbCell.astOut[{{r.i}}].abySafeOut[7];
GVL_IO.aVassOutputData_{{r.tag}} := fbCell.astOut[{{r.i}}].aVassOut;
{{/robots}}

(* One-shot operator buttons *)
bStart := FALSE;
bStop := FALSE;
bReset := FALSE;
bRecoveryConfirm := FALSE;
