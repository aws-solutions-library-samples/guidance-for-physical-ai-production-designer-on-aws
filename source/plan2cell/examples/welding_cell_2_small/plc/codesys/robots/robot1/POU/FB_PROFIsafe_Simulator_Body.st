(*
 * FB_PROFIsafe_Simulator - Implementation
 * Works on the input image abySafeOut (PLC_PRG copies GVL_IO.bySafeOut0..7).
 *)

IF NOT bEnable THEN
    bCommOK := FALSE;
    bDataValid := FALSE;
    RETURN;
END_IF

IF _nCycleCount < UDINT#4000000000 THEN
    _nCycleCount := _nCycleCount + 1;
ELSE
    _nCycleCount := 0;
END_IF

(* ============================================================================
   MONITOR PLC SAFE OUTPUT DATA
   ============================================================================ *)
_bPLCSafeActive := (abySafeOut[0] <> BYTE#0) OR (abySafeOut[1] <> BYTE#0);

(* Watchdog *)
_tonWatchdog(IN := NOT _bPLCSafeActive, PT := tF_WD_Time);
bWatchdogError := _tonWatchdog.Q;

bCommOK := _bPLCSafeActive AND NOT bWatchdogError;
bDataValid := bCommOK;
