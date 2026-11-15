//! KRL text helpers: line builder, number format, ASCII transliteration, identifiers.

use crate::config::LineEnding;

/// KRL identifiers are at most 24 characters.
pub const MAX_IDENT: usize = 24;
pub const INDENT: &str = "   ";

/// Line builder with 3-space indentation.
#[derive(Default)]
pub struct Text {
    lines: Vec<String>,
    depth: usize,
}

impl Text {
    pub fn line(&mut self, s: impl AsRef<str>) {
        let s = s.as_ref();
        if s.is_empty() {
            self.lines.push(String::new());
        } else {
            self.lines.push(format!("{}{s}", INDENT.repeat(self.depth)));
        }
    }

    /// Line at column 0 (headers, DEF/END).
    pub fn raw(&mut self, s: impl Into<String>) {
        self.lines.push(s.into());
    }

    pub fn comment(&mut self, s: impl AsRef<str>) {
        self.line(format!("; {}", ascii(s.as_ref())));
    }

    pub fn open(&mut self, s: impl AsRef<str>) {
        self.line(s);
        self.depth += 1;
    }

    pub fn close(&mut self, s: impl AsRef<str>) {
        self.depth = self.depth.saturating_sub(1);
        self.line(s);
    }

    pub fn indent(&mut self) {
        self.depth += 1;
    }

    pub fn dedent(&mut self) {
        self.depth = self.depth.saturating_sub(1);
    }

    /// KSS style: CRLF between lines, no line break after the last line.
    pub fn finish(self, le: LineEnding) -> String {
        let sep = match le {
            LineEnding::Crlf => "\r\n",
            LineEnding::Lf => "\n",
        };
        self.lines.join(sep)
    }
}

/// `f64` with at most 4 decimals, at least one (`-90.0`, `12.3457`), never `-0.0`.
pub fn num(x: f64) -> String {
    let r = crate::ir::r4(x);
    let mut s = format!("{r:.4}");
    while s.ends_with('0') && !s.ends_with(".0") {
        s.pop();
    }
    s
}

/// Seconds from milliseconds: `0.5`, `0.05`, `7.0`.
pub fn secs(ms: u64) -> String {
    let mut s = format!("{}.{:03}", ms / 1000, ms % 1000);
    while s.ends_with('0') && !s.ends_with(".0") {
        s.pop();
    }
    s
}

/// ASCII only: umlauts transliterated (ae/oe/ue/ss), other non-ASCII and control characters
/// become `?`.
pub fn ascii(s: &str) -> String {
    let mut o = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            'ä' => o.push_str("ae"),
            'ö' => o.push_str("oe"),
            'ü' => o.push_str("ue"),
            'Ä' => o.push_str("Ae"),
            'Ö' => o.push_str("Oe"),
            'Ü' => o.push_str("Ue"),
            'ß' => o.push_str("ss"),
            c if c.is_ascii() && !c.is_ascii_control() => o.push(c),
            _ => o.push('?'),
        }
    }
    o
}

/// One whitespace-free ASCII token (fold titles, comments that are parsed back).
pub fn token(s: &str) -> String {
    let t: String = ascii(s)
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.') {
                c
            } else {
                '_'
            }
        })
        .collect();
    if t.is_empty() { "_".into() } else { t }
}

/// A valid KRL identifier of at most 24 characters (trajectory-to-krl `sanitize_krl_name`).
pub fn ident(s: &str) -> String {
    let mut t: String = ascii(s)
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect();
    if !t.starts_with(|c: char| c.is_ascii_alphabetic()) {
        t.insert_str(0, "P_");
    }
    t.truncate(MAX_IDENT);
    t
}

pub fn is_ident(s: &str) -> bool {
    let mut c = s.chars();
    matches!(c.next(), Some(f) if f.is_ascii_alphabetic() || f == '_')
        && c.all(|c| c.is_ascii_alphanumeric() || c == '_')
        && s.len() <= MAX_IDENT
}

/// 64-bit FNV-1a (file hashes in the manifest).
pub fn fnv1a64(b: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &x in b {
        h ^= u64::from(x);
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    h
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn numbers() {
        assert_eq!(num(-90.0), "-90.0");
        assert_eq!(num(-0.00001), "0.0");
        assert_eq!(num(12.34567), "12.3457");
        assert_eq!(num(0.25), "0.25");
        assert_eq!(secs(500), "0.5");
        assert_eq!(secs(50), "0.05");
        assert_eq!(secs(7000), "7.0");
    }

    #[test]
    fn names() {
        assert_eq!(ascii("Schweißen Größe"), "Schweissen Groesse");
        assert_eq!(ident("000001R01"), "P_000001R01");
        assert_eq!(
            ident("a very long name with spaces"),
            "a_very_long_name_with_sp"
        );
        assert!(is_ident("BT_STEP") && !is_ident("1X") && !is_ident(&"A".repeat(25)));
        assert_eq!(token("W 01/ä"), "W_01_ae");
    }

    #[test]
    fn crlf_without_final_newline() {
        let mut t = Text::default();
        t.raw("DEF X()");
        t.indent();
        t.line("A=1");
        t.dedent();
        t.raw("END");
        assert_eq!(t.finish(LineEnding::Crlf), "DEF X()\r\n   A=1\r\nEND");
    }
}
