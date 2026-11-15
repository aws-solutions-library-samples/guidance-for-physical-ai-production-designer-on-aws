//! ST literal formatting shared by every IEC-ST backend (deterministic, ASCII only).

/// `T#1M3S261MS`; zero is `T#0MS`.
pub fn time_lit(ms: u64) -> String {
    if ms == 0 {
        return "T#0MS".into();
    }
    let (d, rem) = (ms / 86_400_000, ms % 86_400_000);
    let (h, rem) = (rem / 3_600_000, rem % 3_600_000);
    let (m, rem) = (rem / 60_000, rem % 60_000);
    let (s, msr) = (rem / 1000, rem % 1000);
    let mut out = String::from("T#");
    for (v, u) in [(d, "D"), (h, "H"), (m, "M"), (s, "S"), (msr, "MS")] {
        if v > 0 {
            out.push_str(&format!("{v}{u}"));
        }
    }
    out
}

/// Fixed 4 decimals, never `-0.0000`.
pub fn real_lit(x: f64) -> String {
    let s = format!("{x:.4}");
    if s.trim_start_matches('-')
        .chars()
        .all(|c| c == '0' || c == '.')
    {
        "0.0000".into()
    } else {
        s
    }
}

pub fn word_lit(v: u16) -> String {
    format!("WORD#{v}")
}

pub fn word_hex(v: u16) -> String {
    format!("WORD#16#{v:04X}")
}

pub fn dword_hex(v: u32) -> String {
    format!("DWORD#16#{v:08X}")
}

/// `'text'` with `$`/`'` escaped, non-ASCII replaced by `?`, truncated to `max` characters.
pub fn string_lit(s: &str, max: usize) -> String {
    let mut out = String::from("'");
    for c in s.chars().take(max) {
        match c {
            '$' => out.push_str("$$"),
            '\'' => out.push_str("$'"),
            c if c.is_ascii() && !c.is_ascii_control() => out.push(c),
            _ => out.push('?'),
        }
    }
    out.push('\'');
    out
}

/// Text that is safe inside `(* ... *)`.
pub fn comment(s: &str) -> String {
    s.chars()
        .map(|c| {
            if c.is_ascii() && !c.is_ascii_control() {
                c
            } else {
                '?'
            }
        })
        .collect::<String>()
        .replace("(*", "( *")
        .replace("*)", "* )")
}

pub fn bool_lit(b: bool) -> &'static str {
    if b { "TRUE" } else { "FALSE" }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn literals() {
        assert_eq!(time_lit(0), "T#0MS");
        assert_eq!(time_lit(63_261), "T#1M3S261MS");
        assert_eq!(time_lit(9_008), "T#9S8MS");
        assert_eq!(time_lit(3_600_000), "T#1H");
        assert_eq!(real_lit(-0.00001), "0.0000");
        assert_eq!(real_lit(-90.0), "-90.0000");
        assert_eq!(string_lit("a'b$c\u{b0}", 32), "'a$'b$$c?'");
        assert_eq!(string_lit("abcdef", 3), "'abc'");
        assert_eq!(comment("x *) y (* z"), "x * ) y ( * z");
        assert_eq!(dword_hex(1), "DWORD#16#00000001");
        assert_eq!(word_hex(0x5a3c), "WORD#16#5A3C");
    }
}
