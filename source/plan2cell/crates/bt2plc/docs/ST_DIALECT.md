# ST dialect rules for bt2plc templates

Generated ST must compile in **CODESYS V3.5 SP22** (the target) and, after the mechanical
"trust rendering", in **trust-platform v0.24.70** (the embedded checker and co-simulator).
trust is strict IEC 61131-3 Ed.3; CODESYS is lenient. Templates are written in the common
subset below. Every rule was probed against trust (see `docs/TRUST.md` and the probe results).

## Object files (CODESYS object model)
- `DUT/<Name>.st`: one `TYPE … END_TYPE` per file (enum or struct).
- `GVL/<Name>.st`: `{attribute 'qualified_only'}` + `VAR_GLOBAL [CONSTANT] … END_VAR` (declaration only).
- `POU/<Name>.st` (declaration: header comment, `FUNCTION_BLOCK X` / `PROGRAM X`, VAR blocks) and
  `POU/<Name>_Body.st` (implementation only). No `END_FUNCTION_BLOCK` / `END_PROGRAM`.
- Matches CODESYS Interface/Implementation text documents and MCP `set_pou_code(declarationCode, implementationCode)`.

## Trust rendering (done by bt2plc, never by hand)
- DUT as is; GVL wrapped in `NAMESPACE <GvlName> … END_NAMESPACE` with `VAR_GLOBAL CONSTANT` → `VAR_GLOBAL`
  (trust rejects aggregate initializers in CONSTANT globals); POU = declaration + body + `END_FUNCTION_BLOCK`/`END_PROGRAM`.
- Enum access `E_X.Y` → `E_X#Y` for every enum type declared in the project (trust rejects `E_X.Y`).

## Template syntax
- `{{KEY}}` scalar placeholders (e.g. `{{N_ROBOTS}}`, `{{N_ZONES_DIM}}`, `{{N_TICKETS_DIM}}`, `{{N_OPS_DIM}}`,
  `{{GENERATOR}}`, `{{TREE_ID}}`, `{{CELL_ID}}`).
- `{{#robots}} … {{/robots}}` repeats its body per robot; inside, `{{r.i}}` (1-based index), `{{r.tag}}`, `{{r.ns}}`,
  `{{r.station}}`, `{{r.device_name}}`, `{{r.ip}}`. One nesting level only.
- **Array bounds are literals or placeholders**, never constant expressions (`ARRAY[1..{{N_ROBOTS}}] OF …`).

## Language subset
1. Enums: explicit values, accessed qualified `E_X.Y` in the source (renderer adds `#`). CASE labels are
   literals or enum values only, never GVL constants (use IF/ELSIF for constants).
2. **Never write an element of a GVL array** (`GVL_IO.a[i] := …`, `GVL_IO.a[i].3 := …`): trust cannot lower it.
   Read/write GVL arrays only as a whole (`a := GVL_IO.aIn; GVL_IO.aOut := a;`). Scalar GVL writes are fine.
   FBs work on their own VAR_INPUT/VAR_OUTPUT/VAR copies; PLC_PRG copies whole arrays in and out.
   Reading elements and bits of GVL arrays is fine.
3. Bit-string types need **typed literals**: `BYTE#0`, `BYTE#16#FF`, `WORD#11`, `DWORD#0`, `DWORD#16#00000001`.
   This covers assignments (`a[i] := BYTE#0`), comparisons (`dw <> DWORD#0`, `w = WORD#125`) and masks
   (`w AND WORD#16#FF`). INT/DINT/UINT/UDINT/REAL accept plain literals.
4. **No TIME↔number conversions** (TIME_TO_REAL/DINT/…, REAL_TO_TIME are rejected) and no TIME/TIME division.
   Allowed: TON/TOF/TP, `t1 + t2`, `t1 - t2`, `t * intVar`, TIME comparisons, TIME literals `T#1S500MS`.
   Ratios (trajectory interpolation) use REAL seconds accumulated from a `rDt : REAL` input (task interval).
5. No arithmetic on BYTE/WORD/DWORD (`byte + BYTE#1` is rejected): convert (`BYTE_TO_INT`, `WORD_TO_UINT`, …).
   Bit ops (`AND OR XOR NOT SHL SHR`) on bit strings are fine; `SHL(DWORD#1, i)` is fine.
6. No `BOOL_TO_BYTE`; use `SEL(b, BYTE#0, BYTE#1)`.
7. Negative REAL literals as call arguments: `REAL#-1.5`.
8. Avoid identifiers that are keywords in either dialect (`by`, `on`, `at`, `to`, `do`, `of`, …).
9. No methods, properties, actions, `REFERENCE TO`, `POINTER TO`, `__NEW`, `TIME()`, `CURRENT_DT`, `//` comments.
   ASCII only, 4-space indent, `(* … *)` comments.
10. Allowed and probed: FB instance arrays, struct member arrays with bit access (`stOut.aVassOut[0].1 := TRUE`),
    FB calls with struct/array inputs (incl. passing a whole GVL array), `R_TRIG`, `TON`, `RETURN`, `EXIT`,
    `FOR`/`WHILE`, `SEL`, `LIMIT`, `MIN`, `MAX`, `ABS`, `SHL`/`SHR`, `*_TO_*` integer/real conversions,
    `STRING(n)` compare/assign, array-of-struct initializers in non-CONSTANT VAR.

## Style (as in the existing project)
Hungarian prefixes (`b`, `by`, `w`, `dw`, `n`, `i`, `r`, `t`, `s`, `a`, `e`, `st`, `fb`, `ton`, `rtrig`), private
members `_x`, header block `(*\n * Name\n * description\n *)`, banner sections
`(* ===… TITLE ===… *)`, aligned `:` in declarations, explicit enum values.

## Rules found while writing templates
- **Never pass a bare bit access as a call argument** (device templates). `r(CLK := a[0].1)`, `fb(x := byX.1)`,
  `SEL(a[0].1, …)`, `BOOL_TO_INT(a[0].1)` parse, type-check and compile in trust, but fault at runtime
  (`TypeMismatch` for an array-element bit, `NullReference` for a scalar bit). There is no source location,
  and the resource is faulted for every later cycle. First decode the bit into a BOOL variable
  (`_bCmdReset := aVassOutputData[0].5; _rtrigReset(CLK := _bCmdReset);`). Compound expressions
  (`a[0].0 AND b`, `NOT a[0].1`, `(a[0].1)`), IF conditions and plain assignments (`b := a[0].1`,
  `a[0].1 := b`) work.
- **Untyped integer literals must fit in DINT** (device templates). `ud := 4000000000`, `ud < 4000000000` and
  `ud : UDINT := 4000000000` fail the bytecode compile with "integer literal out of range" (no source location;
  the type checker accepts them). Write a typed literal instead: `UDINT#4000000000`. `16#FF59` for a UINT and
  `DWORD#16#FFFFFFFF` work.
- **No arrays of FB instances** (head templates; overrides "FB instance arrays" in rule 10, which the original
  probe never got to compile). `a : ARRAY[1..2] OF TON;` (also `R_TRIG`, user FBs, in a PROGRAM or an FB)
  passes parse and type check but fails the bytecode compile with "PROGRAM init error: init failed for
  X.a: type mismatch" (trust-runtime materialises array defaults per element and has no FunctionBlock case).
  Portable forms: one named instance per element (`{{#robots}}fbRobot_{{r.tag}} : FB_VassRobot;{{/robots}}`,
  called in a robots block, outputs copied into plain arrays for the FOR loops), and for timer arrays
  TIME stamps of one free-running clock: `_tonClock(IN := TRUE, PT := T#1H); _tNow := _tBase + _tonClock.ET;`
  (restart and add to `_tBase` when Q), start `_atStart[i] := _tNow;`, expired
  `(_tNow - _atStart[i]) >= tLimit`. Scalar FB instances (also nested in FBs) are fine.
- **Never pass a GVL array element (or a field of one) directly as a call argument** (head templates).
  `MAX(1, GVL_C.aN[i])`, `WORD_TO_INT(GVL_C.aW[i])`, `MAX(1, GVL_C.aS[i].n)`, `ton(PT := GVL_C.aS[i].tm)`
  pass the type checker but fail the bytecode compile with "unsupported CALL_NATIVE argument target:
  Index { target: Field { target: Name("GVL_C") ... }". Copy into a local first
  (`_n := GVL_C.aS[i].n; _n := MAX(1, _n);`). GVL scalars (`MAX(1, GVL_C.N)`), expressions
  (`GVL_C.aS[i].n + 0`, `GVL_C.W AND WORD#16#FF`), locals / FB members (`MAX(1, loc[i].n)`) and whole GVL
  arrays as FB inputs work; reading GVL elements in assignments and conditions works.
- **Variable initializers must not reference GVL constants** (head templates). `bAutoRun : BOOL :=
  GVL_Cell.AUTO_RUN;` fails with E202 "variable initializer must be a literal or constant expression:
  mutable dependency" because the trust rendering turns `VAR_GLOBAL CONSTANT` into `VAR_GLOBAL`.
  Initialise with a literal and assign on the first scan (`IF _bFirstScan THEN bAutoRun := GVL_Cell.AUTO_RUN;
  _bFirstScan := FALSE; END_IF`). Same for array bounds (already rule "bounds are literals or placeholders").
- **Keywords are case-insensitive, also inside Hungarian names** (rule 8): `aT` (array of TIME) is `AT`,
  `bY` is `BY`; both give parse errors far from the cause ("expected variable name", "expected field name").
- Verified OK in trust (head templates, incl. runtime): element writes into nested locals / own outputs
  (`astOut[r].aVassOut[GVL_Cell.GRANT_BYTE] := ...`, `_astIn[1].abySafeIn[0] := GVL_IO.bySafeIn0_R1;`),
  whole-array copies between GVL, FB outputs and struct members (`GVL_IO.aOut_R1 := fbCell.astOut[1].aVassOut;`),
  `=>` output binding, `FOR ... TO 1 BY -1`, CASE with comma-separated enum labels, `WORD_TO_INT(WORD#40000)`
  (no runtime fault), `TIME - TIME`, `TIME + TIME`, TIME comparisons, `T#1H` / `T#24H`, `SEL` on TIME,
  a TIME field of a GVL array-of-struct element compared with / assigned to a local
  (`IF GVL_Cell.aOps[i].tLimit > _t THEN _t := GVL_Cell.aOps[i].tLimit; END_IF` - use this instead of
  `MAX` on TIME, which was not probed), FB outputs of scalar instances combined in conditions
  (`_rtrigReset.Q AND NOT _rtrigStop.Q`).
- Co-simulation note (head templates): the robot simulators run on a slower task (20 ms) than the head (4 ms),
  so after an instantaneous head restart in trust the drivers still see ProgRunning for a few scans and latch
  ERR_UNEXPECTED_RUN (Stop + Cancel). That is the intended behaviour, not a dialect problem; a restart with a
  link outage (the robots see the outputs drop first) goes straight to CS_RECOVERY.
