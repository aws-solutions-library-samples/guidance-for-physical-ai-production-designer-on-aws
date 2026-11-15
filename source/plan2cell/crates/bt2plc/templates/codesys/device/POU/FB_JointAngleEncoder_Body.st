(*
 * FB_JointAngleEncoder - Implementation
 *
 * Converts 6x REAL degree angles to scaled INT counts (0.02 deg per count)
 * and writes them big-endian into abyOption[0..11] (telegram bytes 16..27).
 *
 * Conversion:  count = ROUND(angle / 0.02)  clamped to INT range
 * Byte order:  Big-endian (high byte at lower address) per S7/PROFINET convention
 * Two's complement is formed in DINT (count + 65536 for negative counts), so no
 * bit operation is applied to a signed type.
 *)

_arDeg[1] := rA1_deg;
_arDeg[2] := rA2_deg;
_arDeg[3] := rA3_deg;
_arDeg[4] := rA4_deg;
_arDeg[5] := rA5_deg;
_arDeg[6] := rA6_deg;

FOR _i := 1 TO 6 DO
    (* === Scale and clamp === *)
    _diTemp := REAL_TO_DINT(LIMIT(REAL#-32768.0, _arDeg[_i] * INV_SCALE, REAL#32767.0));
    IF _diTemp > COUNT_MAX THEN _diTemp := COUNT_MAX; END_IF
    IF _diTemp < COUNT_MIN THEN _diTemp := COUNT_MIN; END_IF
    _aiCount[_i] := DINT_TO_INT(_diTemp);

    (* === Big-endian into the Option area: high byte first === *)
    IF _diTemp < 0 THEN
        _diTemp := _diTemp + 65536;
    END_IF
    _wTemp := DINT_TO_WORD(_diTemp);
    abyOption[2 * (_i - 1)] := WORD_TO_BYTE(SHR(_wTemp, 8));
    abyOption[2 * (_i - 1) + 1] := WORD_TO_BYTE(_wTemp AND WORD#16#FF);
END_FOR

iA1_count := _aiCount[1];
iA2_count := _aiCount[2];
iA3_count := _aiCount[3];
iA4_count := _aiCount[4];
iA5_count := _aiCount[5];
iA6_count := _aiCount[6];
