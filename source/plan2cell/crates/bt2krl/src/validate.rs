//! Strict validator on the rendered text (TASK 8.1).
//!
//! No local tool can reject bad KRL (critic.md section c), so every generated file is checked
//! here: encoding and line endings, block balance, identifiers, declared and used motion
//! targets, `$IN`/`$OUT` ranges and reserved signals, and the per-op protocol order of the
//! production Folge (one +1 STEP write per op, REQUEST = mask, grant before motion, exact stop,
//! release after motion and hold, 4-phase wait before the next step).
//!
//! The context comes from the manifest, so `bt2krl check` can re-run it on a directory.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::config::LineEnding;
use crate::krl::is_ident;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OpExpect {
    pub index: u16,
    pub mask: u32,
    pub hold_ms: u64,
}

/// What the validator needs to know about one robot's files.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CheckCtx {
    pub profile: String,
    pub line_ending: LineEnding,
    /// Allowed `$IN` / `$OUT` index ranges (inclusive).
    pub in_range: (u16, u16),
    pub out_range: (u16, u16),
    /// Signals generated code must never write (profile tables).
    pub reserved_out: Vec<(u16, u16)>,
    pub reserved_in: Vec<(u16, u16)>,
    /// `$OUT` of REQUEST bit 0 / `$IN` of GRANT bit 0 (sim) or the VW request / grant bases.
    pub request_base: u16,
    pub grant_base: u16,
    /// Relative path (without extension) of the production Folge, e.g. `KRC/R1/Folgen/folge11`.
    pub production: String,
    /// Assignment prefix of the step signal (`BT_STEP=`, `BT_PKTNR=`).
    #[serde(default = "step_prefix_default")]
    pub step_prefix: String,
    pub ops: Vec<OpExpect>,
}

fn step_prefix_default() -> String {
    "BT_STEP=".into()
}

impl CheckCtx {
    fn vw(&self) -> bool {
        self.profile == "vw-ks24"
    }
}

fn strip(l: &str) -> &str {
    let l = l.trim();
    if l.starts_with(";FOLD") || l.starts_with(";ENDFOLD") {
        return l;
    }
    match l.find(';') {
        Some(i) => l[..i].trim_end(),
        None => l,
    }
}

fn first_word(l: &str) -> &str {
    l.split(|c: char| c.is_whitespace() || c == '(')
        .next()
        .unwrap_or("")
}

/// Encoding, line endings, no trailing line break.
fn check_text(name: &str, text: &str, le: LineEnding, e: &mut Vec<String>) {
    if let Some(p) = text.bytes().position(|b| b >= 0x80) {
        e.push(format!("{name}: non-ASCII byte at offset {p}"));
    }
    if text.contains('\t') {
        e.push(format!("{name}: tab character"));
    }
    let b = text.as_bytes();
    for (i, &c) in b.iter().enumerate() {
        let crlf_ok = i > 0 && b[i - 1] == b'\r';
        match (le, c) {
            (LineEnding::Crlf, b'\n') if !crlf_ok => {
                e.push(format!("{name}: bare LF at offset {i}"));
                break;
            }
            (LineEnding::Crlf, b'\r') if b.get(i + 1) != Some(&b'\n') => {
                e.push(format!("{name}: bare CR at offset {i}"));
                break;
            }
            (LineEnding::Lf, b'\r') => {
                e.push(format!("{name}: CR in an LF file"));
                break;
            }
            _ => {}
        }
    }
    if text.ends_with('\n') {
        e.push(format!(
            "{name}: the last line must not end with a line break"
        ));
    }
}

/// DEF/END, DEFDAT/ENDDAT, FOLD/ENDFOLD, IF/ENDIF, LOOP, SWITCH, FOR, WHILE, REPEAT, SPLINE.
fn check_blocks(name: &str, text: &str, e: &mut Vec<String>) {
    let mut st: Vec<(&'static str, usize)> = Vec::new();
    for (n, raw) in text.lines().enumerate() {
        let l = strip(raw);
        let w = first_word(l);
        let open = match w {
            ";FOLD" => Some("FOLD"),
            "DEF" | "GLOBAL" if l.contains("DEF ") || w == "DEF" => Some("DEF"),
            "DEFFCT" => Some("DEFFCT"),
            "DEFDAT" => Some("DEFDAT"),
            "IF" => Some("IF"),
            "LOOP" => Some("LOOP"),
            "SWITCH" => Some("SWITCH"),
            "FOR" => Some("FOR"),
            "WHILE" => Some("WHILE"),
            "REPEAT" => Some("REPEAT"),
            "SPLINE" | "PTP_SPLINE" => Some("SPLINE"),
            _ => None,
        };
        if let Some(o) = open {
            if o == "IF" && !l.ends_with("THEN") {
                e.push(format!("{name}:{}: IF without THEN at the line end", n + 1));
            }
            st.push((o, n + 1));
            continue;
        }
        let close = match w {
            ";ENDFOLD" => Some("FOLD"),
            "END" => Some("DEF"),
            "ENDFCT" => Some("DEFFCT"),
            "ENDDAT" => Some("DEFDAT"),
            "ENDIF" => Some("IF"),
            "ENDLOOP" => Some("LOOP"),
            "ENDSWITCH" => Some("SWITCH"),
            "ENDFOR" => Some("FOR"),
            "ENDWHILE" => Some("WHILE"),
            "UNTIL" => Some("REPEAT"),
            "ENDSPLINE" => Some("SPLINE"),
            _ => None,
        };
        if let Some(c) = close {
            match st.pop() {
                Some((o, _)) if o == c => {}
                Some((o, at)) => {
                    e.push(format!(
                        "{name}:{}: {w} closes {o} opened at line {at}",
                        n + 1
                    ));
                    return;
                }
                None => {
                    e.push(format!("{name}:{}: {w} without an open block", n + 1));
                    return;
                }
            }
        }
        if matches!(w, "ELSE" | "CASE" | "DEFAULT")
            && !matches!(st.last(), Some(("IF" | "SWITCH", _)))
        {
            e.push(format!("{name}:{}: {w} outside IF/SWITCH", n + 1));
        }
    }
    for (o, at) in st {
        e.push(format!("{name}: {o} opened at line {at} is not closed"));
    }
}

/// Names introduced by DEF, DEFDAT, DECL, SIGNAL: valid and at most 24 characters.
fn check_idents(name: &str, text: &str, e: &mut Vec<String>) -> BTreeSet<String> {
    let mut targets = BTreeSet::new();
    for (n, raw) in text.lines().enumerate() {
        let l = strip(raw);
        let mut names: Vec<&str> = Vec::new();
        if let Some(r) = l.strip_prefix("DEF ").or_else(|| l.strip_prefix("DEFDAT ")) {
            names.push(
                r.split(|c: char| c == '(' || c.is_whitespace())
                    .next()
                    .unwrap_or(""),
            );
        } else if let Some(r) = l.strip_prefix("SIGNAL ") {
            names.push(r.split_whitespace().next().unwrap_or(""));
        } else if let Some(r) = l.strip_prefix("DECL ") {
            let mut it = r.splitn(2, ' ');
            let ty = it.next().unwrap_or("");
            let rest = it.next().unwrap_or("");
            let decl = rest.split('=').next().unwrap_or("");
            for v in decl.split(',') {
                let v = v.trim();
                names.push(v);
                if matches!(ty, "AXIS" | "E6AXIS") {
                    targets.insert(v.to_string());
                }
            }
        }
        for v in names {
            if !is_ident(v) {
                e.push(format!(
                    "{name}:{}: '{v}' is not a valid KRL identifier of at most 24 characters",
                    n + 1
                ));
            }
        }
    }
    targets
}

fn indices(l: &str, prefix: &str) -> Vec<u16> {
    let mut out = Vec::new();
    let mut rest = l;
    while let Some(i) = rest.find(prefix) {
        rest = &rest[i + prefix.len()..];
        if let Some(j) = rest.find(']')
            && let Ok(v) = rest[..j].parse()
        {
            out.push(v);
        }
    }
    out
}

fn in_ranges(v: u16, r: &[(u16, u16)]) -> bool {
    r.iter().any(|&(a, b)| v >= a && v <= b)
}

fn check_io(name: &str, text: &str, c: &CheckCtx, e: &mut Vec<String>) {
    for (n, raw) in text.lines().enumerate() {
        let l = strip(raw);
        for (prefix, range, reserved) in [
            ("$IN[", c.in_range, &c.reserved_in),
            ("$OUT[", c.out_range, &c.reserved_out),
        ] {
            for v in indices(l, prefix) {
                if v < range.0 || v > range.1 {
                    e.push(format!(
                        "{name}:{}: {prefix}{v}] outside {}..{}",
                        n + 1,
                        range.0,
                        range.1
                    ));
                }
                if in_ranges(v, reserved) {
                    e.push(format!("{name}:{}: {prefix}{v}] is reserved", n + 1));
                }
            }
        }
    }
}

fn ptp_target(l: &str) -> Option<(&str, bool)> {
    let r = l.strip_prefix("PTP ")?;
    let mut it = r.split_whitespace();
    let t = it.next()?;
    Some((t, it.next() == Some("C_PTP")))
}

#[derive(Default)]
struct OpState {
    k: u16,
    mask: u32,
    hold_ms: u64,
    req: u32,
    granted: bool,
    moved: bool,
    last_approx: bool,
    hold_seen: bool,
    rel: u32,
    rel_waited: bool,
}

/// Per-op protocol order of the production Folge in profile `vw-ks24`: one step write per op in
/// +1 order, the interlocks locked (request + MAKRO20 handshake) before the op's first motion and
/// released after its last motion and its hold, exact stop at the last point, and no interlock
/// signal written outside the VWSTART / VWLOCK / VWUNLOCK folds.
fn check_protocol_vw(name: &str, text: &str, c: &CheckCtx, e: &mut Vec<String>) {
    #[derive(Default)]
    struct Op {
        k: u16,
        mask: u32,
        hold_ms: u64,
        locked: u32,
        unlocked: u32,
        moved: bool,
        last_approx: bool,
        hold_seen: bool,
        lock_after_move: bool,
    }
    let mut expect: u16 = 1;
    let mut cur: Option<Op> = None;
    let mut zeroed = false;
    let mut fold: Vec<String> = Vec::new();
    let close = |o: &Op, e: &mut Vec<String>| {
        let at = format!("{name}: op {}", o.k);
        if !o.moved {
            e.push(format!("{at}: no motion"));
        }
        if o.last_approx {
            e.push(format!(
                "{at}: the last point is approximated (needs an exact stop)"
            ));
        }
        if o.mask != 0 {
            if o.locked != o.mask {
                e.push(format!(
                    "{at}: locked 16#{:08X} != mask 16#{:08X}",
                    o.locked, o.mask
                ));
            }
            if o.lock_after_move {
                e.push(format!("{at}: interlock locked after the motion started"));
            }
            if o.unlocked != o.mask {
                e.push(format!("{at}: released 16#{:08X} != mask", o.unlocked));
            }
        } else if o.locked != 0 {
            e.push(format!("{at}: interlock locked for an op without zones"));
        }
    };
    for (n, raw) in text.lines().enumerate() {
        let l = strip(raw);
        let at = format!("{name}:{}", n + 1);
        if let Some(t) = l.strip_prefix(";FOLD ") {
            let w: Vec<&str> = t.split_whitespace().collect();
            fold.push(w.first().copied().unwrap_or("").to_string());
            match w.as_slice() {
                ["VWLOCK", m] | ["VWUNLOCK", m] => {
                    let mask = crate::program::parse_hex(m).unwrap_or(u64::MAX) as u32;
                    match cur.as_mut() {
                        None => e.push(format!("{at}: {} outside an op", w[0])),
                        Some(o) => {
                            if w[0] == "VWLOCK" {
                                o.locked |= mask;
                                if o.moved {
                                    o.lock_after_move = true;
                                }
                            } else {
                                if !o.moved {
                                    e.push(format!("{at}: release before the motion"));
                                }
                                if o.hold_ms > 0 && !o.hold_seen {
                                    e.push(format!("{at}: release before the hold"));
                                }
                                o.unlocked |= mask;
                            }
                        }
                    }
                }
                _ => {}
            }
            continue;
        }
        if l.starts_with(";ENDFOLD") {
            fold.pop();
            continue;
        }
        let in_il_fold = fold
            .iter()
            .any(|f| matches!(f.as_str(), "VWLOCK" | "VWUNLOCK" | "VWSTART" | "VWP0"));
        if let Some(k) = l.strip_prefix(&c.step_prefix) {
            let Ok(k) = k.parse::<u16>() else {
                e.push(format!(
                    "{at}: the step signal is written with a non-literal value"
                ));
                continue;
            };
            if let Some(o) = cur.take() {
                close(&o, e);
            }
            if k == 0 {
                zeroed = true;
                continue;
            }
            if zeroed || k != expect {
                e.push(format!(
                    "{at}: step {k} where {expect} is due (+1 only, once per op)"
                ));
            }
            let x = c.ops.iter().find(|o| o.index == k);
            if x.is_none() {
                e.push(format!("{at}: step {k} is not an op"));
            }
            expect = k + 1;
            cur = Some(Op {
                k,
                mask: x.map(|o| o.mask).unwrap_or(0),
                hold_ms: x.map(|o| o.hold_ms).unwrap_or(0),
                ..Default::default()
            });
            continue;
        }
        if ptp_target(l).is_some() {
            match cur.as_mut() {
                None => {
                    // P0 of the VW template is the only motion outside an op.
                    if !fold.iter().any(|f| f == "VWP0") {
                        e.push(format!("{at}: motion outside an op"));
                    }
                }
                Some(o) => {
                    if o.mask != 0 && o.locked != o.mask {
                        e.push(format!("{at}: motion before the interlocks are locked"));
                    }
                    if o.unlocked != 0 {
                        e.push(format!("{at}: motion after the release"));
                    }
                    o.moved = true;
                    o.last_approx = ptp_target(l).map(|x| x.1).unwrap_or(false);
                }
            }
            continue;
        }
        if l.starts_with("WAIT SEC ")
            && let Some(o) = cur.as_mut()
            && o.moved
        {
            o.hold_seen = true;
        }
        // Interlock signals may only be touched inside the dedicated folds.
        if !in_il_fold {
            for v in indices(l, "$OUT[") {
                if (41..=56).contains(&v) || (81..=96).contains(&v) {
                    e.push(format!(
                        "{at}: interlock signal $OUT[{v}] written outside a VWLOCK/VWUNLOCK/VWSTART fold"
                    ));
                }
            }
        }
    }
    if let Some(o) = cur.take() {
        close(&o, e);
    }
    if !zeroed {
        e.push(format!(
            "{name}: the step signal is never set to 0 at the end"
        ));
    }
    if expect as usize != c.ops.len() + 1 {
        e.push(format!(
            "{name}: {} ops written, {} expected",
            expect - 1,
            c.ops.len()
        ));
    }
}

/// Per-op protocol order of the production Folge (sim: per-bit REQUEST/GRANT).
fn check_protocol(name: &str, text: &str, c: &CheckCtx, e: &mut Vec<String>) {
    let mut expect: u16 = 1;
    let mut cur: Option<OpState> = None;
    let mut finished = false;
    let close = |o: &OpState, e: &mut Vec<String>| {
        let at = format!("{name}: op {}", o.k);
        if !o.moved {
            e.push(format!("{at}: no motion"));
        }
        if o.last_approx {
            e.push(format!(
                "{at}: the last point is approximated (needs an exact stop)"
            ));
        }
        if o.mask != 0 {
            if o.req != o.mask {
                e.push(format!(
                    "{at}: REQUEST 16#{:08X} != mask 16#{:08X}",
                    o.req, o.mask
                ));
            }
            if !o.granted {
                e.push(format!("{at}: no wait for the GRANT"));
            }
            if o.rel != o.mask {
                e.push(format!("{at}: release 16#{:08X} != mask", o.rel));
            }
            if !o.rel_waited {
                e.push(format!("{at}: no 4-phase wait for the GRANT to drop"));
            }
        } else if o.req != 0 || o.rel != 0 {
            e.push(format!("{at}: REQUEST written for an op without zones"));
        }
    };
    let bitmask = |ixs: &[u16], base: u16| -> Option<u32> {
        let mut m = 0;
        for &v in ixs {
            let b = v.checked_sub(base).filter(|&b| b < 32)?;
            m |= 1u32 << b;
        }
        Some(m)
    };
    for (n, raw) in text.lines().enumerate() {
        let l = strip(raw);
        let at = format!("{name}:{}", n + 1);
        if let Some(k) = l.strip_prefix("BT_STEP=") {
            let Ok(k) = k.parse::<u16>() else {
                e.push(format!("{at}: STEP written with a non-literal value"));
                continue;
            };
            if let Some(o) = cur.take() {
                close(&o, e);
            }
            if k == 0 {
                finished = true;
                continue;
            }
            if finished || k != expect {
                e.push(format!(
                    "{at}: STEP {k} where {expect} is due (+1 only, once per op)"
                ));
            }
            let x = c.ops.iter().find(|o| o.index == k);
            if x.is_none() {
                e.push(format!("{at}: STEP {k} is not an op"));
            }
            expect = k + 1;
            cur = Some(OpState {
                k,
                mask: x.map(|o| o.mask).unwrap_or(0),
                hold_ms: x.map(|o| o.hold_ms).unwrap_or(0),
                ..Default::default()
            });
            continue;
        }
        let outs = indices(l, "$OUT[");
        let ins = indices(l, "$IN[");
        if let Some((_, approx)) = ptp_target(l) {
            match cur.as_mut() {
                None => e.push(format!("{at}: motion outside an op")),
                Some(o) => {
                    if o.mask != 0 && !o.granted {
                        e.push(format!("{at}: motion before the GRANT wait"));
                    }
                    if o.rel != 0 {
                        e.push(format!("{at}: motion after the release"));
                    }
                    o.moved = true;
                    o.last_approx = approx;
                }
            }
            continue;
        }
        if !outs.is_empty() && l.starts_with("$OUT[") {
            if let Some(m) = bitmask(&outs, c.request_base) {
                let Some(o) = cur.as_mut() else {
                    e.push(format!("{at}: REQUEST written outside an op"));
                    continue;
                };
                if l.ends_with("=TRUE") {
                    if o.granted || o.moved {
                        e.push(format!("{at}: REQUEST raised after the grant/motion"));
                    }
                    o.req |= m;
                } else {
                    if !o.moved {
                        e.push(format!("{at}: release before the motion"));
                    }
                    if o.hold_ms > 0 && !o.hold_seen {
                        e.push(format!("{at}: release before the hold"));
                    }
                    o.rel |= m;
                }
            }
            continue;
        }
        if let Some(cond) = l.strip_prefix("WAIT FOR ") {
            if let Some(m) = bitmask(&ins, c.grant_base).filter(|_| !ins.is_empty()) {
                let Some(o) = cur.as_mut() else {
                    e.push(format!("{at}: GRANT wait outside an op"));
                    continue;
                };
                if cond.starts_with("NOT ") {
                    if o.rel != o.mask {
                        e.push(format!("{at}: GRANT-drop wait before the full release"));
                    }
                    o.rel_waited = m == o.mask;
                } else {
                    if o.req != o.mask {
                        e.push(format!("{at}: GRANT wait before REQUEST = mask"));
                    }
                    if m != o.mask {
                        e.push(format!("{at}: GRANT wait for 16#{m:08X} != mask"));
                    }
                    o.granted = true;
                }
            }
            continue;
        }
        if l.starts_with("WAIT SEC ")
            && let Some(o) = cur.as_mut()
            && o.moved
        {
            o.hold_seen = true;
        }
    }
    if let Some(o) = cur.take() {
        close(&o, e);
    }
    if !finished {
        e.push(format!("{name}: no STEP := 0 at the end"));
    }
    if expect as usize != c.ops.len() + 1 {
        e.push(format!(
            "{name}: {} ops written, {} expected",
            expect - 1,
            c.ops.len()
        ));
    }
}

/// Validate the files of one robot (`rel path -> text`, relative to the robot directory).
pub fn validate(files: &BTreeMap<String, String>, c: &CheckCtx) -> Vec<String> {
    let mut e = Vec::new();
    let mut targets: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for (rel, text) in files {
        let krl = rel.ends_with(".src") || rel.ends_with(".dat") || rel.ends_with(".sub");
        if !krl {
            continue;
        }
        check_text(rel, text, c.line_ending, &mut e);
        check_blocks(rel, text, &mut e);
        let t = check_idents(rel, text, &mut e);
        check_io(rel, text, c, &mut e);
        if let Some(stem) = rel.strip_suffix(".dat") {
            targets.insert(stem.to_string(), t);
        }
    }
    for (rel, text) in files {
        let Some(stem) = rel.strip_suffix(".src") else {
            continue;
        };
        let declared = targets.get(stem).cloned().unwrap_or_default();
        let mut used = BTreeSet::new();
        for (n, raw) in text.lines().enumerate() {
            if let Some((t, _)) = ptp_target(strip(raw)) {
                if !declared.contains(t) {
                    e.push(format!(
                        "{rel}:{}: motion target {t} is not declared in {stem}.dat",
                        n + 1
                    ));
                }
                used.insert(t.to_string());
            }
        }
        for d in declared.difference(&used) {
            e.push(format!("{stem}.dat: point {d} is declared but never used"));
        }
        if stem == c.production {
            if c.vw() {
                check_protocol_vw(rel, text, c, &mut e);
            } else {
                check_protocol(rel, text, c, &mut e);
            }
        }
    }
    if !files.contains_key(&format!("{}.src", c.production)) {
        e.push(format!("missing production Folge {}.src", c.production));
    }
    e
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx() -> CheckCtx {
        CheckCtx {
            profile: "sim".into(),
            line_ending: LineEnding::Crlf,
            in_range: (1, 256),
            out_range: (1, 256),
            reserved_out: vec![],
            reserved_in: vec![],
            request_base: 225,
            grant_base: 97,
            production: "F/folge11".into(),
            step_prefix: "BT_STEP=".into(),
            ops: vec![OpExpect {
                index: 1,
                mask: 1,
                hold_ms: 500,
            }],
        }
    }

    fn files(body: &[&str]) -> BTreeMap<String, String> {
        let mut src = vec!["DEF FOLGE11()"];
        src.extend_from_slice(body);
        src.push("END");
        let mut m = BTreeMap::new();
        m.insert("F/folge11.src".into(), src.join("\r\n"));
        m.insert(
            "F/folge11.dat".into(),
            "DEFDAT FOLGE11\r\nDECL AXIS P1={A1 0.0,A2 0.0,A3 0.0,A4 0.0,A5 0.0,A6 0.0}\r\nENDDAT"
                .into(),
        );
        m
    }

    const GOOD: &[&str] = &[
        "BT_STEP=1",
        "$OUT[225]=TRUE",
        "WAIT FOR $IN[97]",
        "PTP P1",
        "WAIT SEC 0.5",
        "$OUT[225]=FALSE",
        "WAIT FOR NOT $IN[97]",
        "BT_STEP=0",
    ];

    #[test]
    fn good_program_passes() {
        assert_eq!(validate(&files(GOOD), &ctx()), Vec::<String>::new());
    }

    #[test]
    fn protocol_violations_are_found() {
        let cases: &[(&[&str], &str)] = &[
            (
                &[
                    "BT_STEP=1",
                    "$OUT[225]=TRUE",
                    "PTP P1",
                    "WAIT FOR $IN[97]",
                    "WAIT SEC 0.5",
                    "$OUT[225]=FALSE",
                    "WAIT FOR NOT $IN[97]",
                    "BT_STEP=0",
                ],
                "motion before the GRANT",
            ),
            (
                &[
                    "BT_STEP=1",
                    "$OUT[225]=TRUE",
                    "WAIT FOR $IN[97]",
                    "PTP P1",
                    "$OUT[225]=FALSE",
                    "WAIT SEC 0.5",
                    "WAIT FOR NOT $IN[97]",
                    "BT_STEP=0",
                ],
                "release before the hold",
            ),
            (
                &[
                    "BT_STEP=1",
                    "$OUT[225]=TRUE",
                    "WAIT FOR $IN[97]",
                    "PTP P1",
                    "WAIT SEC 0.5",
                    "$OUT[225]=FALSE",
                    "BT_STEP=0",
                ],
                "4-phase",
            ),
            (
                &[
                    "BT_STEP=2",
                    "$OUT[225]=TRUE",
                    "WAIT FOR $IN[97]",
                    "PTP P1",
                    "WAIT SEC 0.5",
                    "$OUT[225]=FALSE",
                    "WAIT FOR NOT $IN[97]",
                    "BT_STEP=0",
                ],
                "+1 only",
            ),
            (
                &[
                    "BT_STEP=1",
                    "$OUT[226]=TRUE",
                    "WAIT FOR $IN[97]",
                    "PTP P1",
                    "WAIT SEC 0.5",
                    "$OUT[225]=FALSE",
                    "WAIT FOR NOT $IN[97]",
                    "BT_STEP=0",
                ],
                "REQUEST",
            ),
            (
                &[
                    "BT_STEP=1",
                    "$OUT[225]=TRUE",
                    "WAIT FOR $IN[97]",
                    "PTP P1 C_PTP",
                    "WAIT SEC 0.5",
                    "$OUT[225]=FALSE",
                    "WAIT FOR NOT $IN[97]",
                    "BT_STEP=0",
                ],
                "exact stop",
            ),
        ];
        for (body, want) in cases {
            let e = validate(&files(body), &ctx());
            assert!(e.iter().any(|m| m.contains(want)), "{want}: {e:?}");
        }
    }

    #[test]
    fn text_blocks_idents_and_io_are_checked() {
        let mut f = files(GOOD);
        f.insert("F/x.src".into(), "DEF X()\nIF A THEN\r\nEND\r\n".into());
        f.insert(
            "F/y.src".into(),
            "DEF ABCDEFGHIJKLMNOPQRSTUVWXYZ()\r\n$OUT[300]=TRUE\r\nPTP Q9\r\nEND".into(),
        );
        f.insert(
            "F/y.dat".into(),
            "DEFDAT Y\r\nDECL AXIS Q8={A1 0.0,A2 0.0,A3 0.0,A4 0.0,A5 0.0,A6 0.0}\r\nENDDAT".into(),
        );
        let e = validate(&f, &ctx()).join("\n");
        for want in [
            "bare LF",
            "line break",
            "IF opened",
            "identifier",
            "$OUT[300] outside",
            "Q9 is not declared",
            "Q8 is declared but never used",
        ] {
            assert!(e.contains(want), "{want} missing in\n{e}");
        }
    }
}
