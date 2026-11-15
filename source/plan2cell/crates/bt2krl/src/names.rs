//! KSS compile check by name: every plain (non-`$`, non-`#`) name a generated `.src` uses must be
//! a KRL keyword or built-in, declared by the bundle itself (its `.dat` files, `bt_config.dat`
//! merged into `$config.dat`, a `DEF` / `DEFFCT` or a label), or a global of the target
//! controller (its `$config.dat`). A name none of them declares is KSS error 2137 "name unknown"
//! when the module is loaded (`plan2cell cell krl`), e.g. the VW `P_PktNr_SPS`, which is local to
//! `vw_usr_r` on every KS24 controller.
//!
//! [`unknown_names`] runs on the files of a robot's bundle (`krl/<ns>/`) and the target's
//! globals: [`crate::vw::VW_CONFIG_GLOBALS`] for the VW KS24 standard (`vw-ks24`), nothing more
//! for a stock controller (`sim`), or [`declared`] of the controller's own `$config.dat`.

use std::collections::BTreeSet;

/// KRL keywords, types and built-ins generated code may use (matched case-insensitively).
pub const KRL_WORDS: &[&str] = &[
    "DEF", "END", "DEFFCT", "ENDFCT", "DEFDAT", "ENDDAT", "PUBLIC", "GLOBAL", "DECL", "SIGNAL", "STRUC",
    "ENUM", "EXT", "EXTFCT", "IF", "THEN", "ELSE", "ENDIF", "GOTO", "WAIT", "FOR", "TO", "STEP",
    "ENDFOR", "WHILE", "ENDWHILE", "REPEAT", "UNTIL", "LOOP", "ENDLOOP", "EXIT", "RETURN", "HALT",
    "SEC", "SWITCH", "CASE", "DEFAULT", "ENDSWITCH", "CONTINUE", "INTERRUPT", "WHEN", "DO", "ON", "OFF",
    "ENABLE", "DISABLE", "TRIGGER", "DISTANCE", "DELAY", "PRIO", "PTP", "LIN", "CIRC", "PTP_REL",
    "LIN_REL", "C_PTP", "C_DIS", "C_VEL", "C_ORI", "AND", "OR", "NOT", "EXOR", "B_AND", "B_OR", "B_NOT",
    "B_EXOR", "TRUE", "FALSE", "IN", "OUT", "INT", "BOOL", "REAL", "CHAR", "AXIS", "E6AXIS", "POS", "E6POS",
    "FRAME", "A1", "A2", "A3", "A4", "A5", "A6", "E1", "E2", "E3", "E4", "E5", "E6", "X", "Y", "Z", "A", "B",
    "C", "S", "T", "ABS", "SQRT", "SIN", "COS", "TAN", "ACOS", "ATAN2", "MAX", "MIN", "BRAKE", "RESUME",
    "PULSE", "ANIN", "ANOUT", "SREAD", "SWRITE", "CWRITE", "CREAD", "STRCLEAR", "STRLEN", "STRCOPY",
    "STRADD", "STRCOMP", "STRFIND", "VARSTATE", "GET_SIG_INF",
];

/// The plain names of the KRL code in `text`, outside strings and comments, with their line
/// (1-based) and the line's text.
pub fn plain_names(text: &str) -> Vec<(usize, String, String)> {
    let mut v = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let line = line.trim_end_matches('\r');
        let mut code = String::new();
        let mut in_str = false;
        for c in line.chars() {
            match c {
                '"' => in_str = !in_str,
                ';' if !in_str => break,
                _ if in_str => {}
                _ => code.push(c),
            }
        }
        let b = code.as_bytes();
        let mut i = 0;
        while i < b.len() {
            let c = b[i] as char;
            if c.is_ascii_alphabetic() || c == '_' || c == '$' || c == '#' || c == '&' {
                let st = i;
                i += 1;
                while i < b.len() && ((b[i] as char).is_ascii_alphanumeric() || b[i] == b'_') {
                    i += 1;
                }
                if c.is_ascii_alphabetic() || c == '_' {
                    v.push((n + 1, code[st..i].to_string(), line.to_string()));
                } else if c == '&' {
                    break; // &ACCESS / &REL / &COMMENT header line
                }
            } else if c.is_ascii_digit() || c == '.' {
                // Numbers (also 16#.. literals and exponents) and member accesses (`.X`).
                while i < b.len() && ((b[i] as char).is_ascii_alphanumeric() || b[i] == b'#' || b[i] == b'.' || b[i] == b'_') {
                    i += 1;
                }
            } else {
                i += 1;
            }
        }
    }
    v
}

fn ident(s: &str) -> bool {
    let mut c = s.chars();
    matches!(c.next(), Some(f) if f.is_ascii_alphabetic() || f == '_') && c.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

fn words(code: &str) -> Vec<&str> {
    code.split(|c: char| !(c.is_ascii_alphanumeric() || c == '_' || c == '$')).filter(|x| !x.is_empty()).collect()
}

fn is_word(w: &str, k: &str) -> bool {
    w.eq_ignore_ascii_case(k)
}

/// The names a data list (`.dat`, `$config.dat`, `bt_config.dat`) declares: `SIGNAL n`, `EXT n`,
/// `EXTFCT t n`, `STRUC n`, `ENUM n`, and `[DECL] [GLOBAL] TYPE n[, m...]` (a data list may leave
/// out `DECL`). Comments and the values after `=` are ignored.
pub fn declared(text: &str) -> BTreeSet<String> {
    declared_in(text, true)
}

/// [`declared`] for a `.src`: only lines that begin with `DECL` / `SIGNAL` / `EXT` / ... declare
/// (an assignment `X=Y` does not), plus the module's `DEF` / `DEFFCT` names and its labels.
pub fn defined_in_src(text: &str) -> BTreeSet<String> {
    let mut d = declared_in(text, false);
    for line in text.lines() {
        let code = line.split(';').next().unwrap_or("").trim();
        let w = words(code);
        let mut i = 0;
        if w.first().is_some_and(|x| is_word(x, "GLOBAL")) {
            i = 1;
        }
        match &w[i.min(w.len())..] {
            [d0, n, ..] if is_word(d0, "DEF") && ident(n) => {
                d.insert((*n).to_string());
            }
            [d0, _, n, ..] if is_word(d0, "DEFFCT") && ident(n) => {
                d.insert((*n).to_string());
            }
            _ => {}
        }
        if let Some(l) = code.strip_suffix(':')
            && ident(l.trim())
        {
            d.insert(l.trim().to_string());
        }
    }
    d
}

fn declared_in(text: &str, data_list: bool) -> BTreeSet<String> {
    let mut d = BTreeSet::new();
    for line in text.lines() {
        let code = line.split(';').next().unwrap_or("").trim();
        if code.is_empty() || code.starts_with('&') {
            continue;
        }
        // The names part: before the first `=` (a value) and without `[..]` dimensions.
        let head = code.split('=').next().unwrap_or("");
        let w = words(head);
        let mut i = 0;
        let mut decl = false;
        if w.get(i).is_some_and(|x| is_word(x, "DECL")) {
            i += 1;
            decl = true;
        }
        if w.get(i).is_some_and(|x| is_word(x, "GLOBAL")) {
            i += 1;
        }
        let rest = &w[i.min(w.len())..];
        let first = rest.first().copied().unwrap_or("");
        let named = |d: &mut BTreeSet<String>, n: &str| {
            if ident(n) {
                d.insert(n.to_string());
            }
        };
        match rest {
            [k, n, ..] if is_word(k, "SIGNAL") || is_word(k, "EXT") || is_word(k, "STRUC") || is_word(k, "ENUM") => named(&mut d, n),
            [k, _, n, ..] if is_word(k, "EXTFCT") => named(&mut d, n),
            [k, ..] if ["DEF", "DEFFCT", "DEFDAT", "ENDDAT", "END", "ENDFCT", "INTERRUPT", "IF", "WAIT", "PTP", "LIN"]
                .iter()
                .any(|x| is_word(k, x)) => {}
            [_ty, _, ..] if decl || data_list => {
                // TYPE a[, b, c]: the names after the type, up to the value.
                let after = head.trim_start();
                let after = strip_word(after, "DECL");
                let after = strip_word(after, "GLOBAL");
                let after = after.trim_start().strip_prefix(first).unwrap_or(after);
                for part in after.split(',') {
                    let n = part.split('[').next().unwrap_or("").trim();
                    named(&mut d, n);
                }
            }
            _ => {}
        }
    }
    d
}

fn strip_word<'a>(s: &'a str, w: &str) -> &'a str {
    let t = s.trim_start();
    if t.len() >= w.len() && t[..w.len()].eq_ignore_ascii_case(w) && t[w.len()..].starts_with(|c: char| c.is_whitespace()) {
        &t[w.len()..]
    } else {
        s
    }
}

/// A name a module uses that nothing declares.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Unknown {
    /// The bundle file (relative to `krl/<ns>/`).
    pub file: String,
    pub line: usize,
    pub name: String,
    pub text: String,
}

/// The unknown names of a robot's bundle: `files` are its paths (relative to `krl/<ns>/`) and
/// texts, `globals` the target controller's (case-insensitive). Every `.src` below `KRC/` is
/// checked against [`KRL_WORDS`], everything the bundle's `.dat` / `.src` files declare, and
/// `globals`. The `patches/` (fragments for a person) are not code that is loaded.
pub fn unknown_names<'a>(files: impl IntoIterator<Item = (&'a str, &'a str)>, globals: &BTreeSet<String>) -> Vec<Unknown> {
    let files: Vec<(&str, &str)> = files.into_iter().filter(|(p, _)| p.starts_with("KRC/")).collect();
    let mut known: BTreeSet<String> = KRL_WORDS.iter().map(|w| w.to_ascii_uppercase()).collect();
    known.extend(globals.iter().map(|g| g.to_ascii_uppercase()));
    for (path, text) in &files {
        let lower = path.to_ascii_lowercase();
        let names = if lower.ends_with(".dat") {
            declared(text)
        } else if lower.ends_with(".src") || lower.ends_with(".sub") {
            defined_in_src(text)
        } else {
            continue;
        };
        known.extend(names.into_iter().map(|n| n.to_ascii_uppercase()));
    }
    let mut out = Vec::new();
    for (path, text) in &files {
        if !path.to_ascii_lowercase().ends_with(".src") {
            continue;
        }
        let mut seen = BTreeSet::new();
        for (line, name, code) in plain_names(text) {
            let up = name.to_ascii_uppercase();
            if !known.contains(&up) && seen.insert(up) {
                out.push(Unknown { file: path.to_string(), line, name, text: code.trim().to_string() });
            }
        }
    }
    out
}

/// The globals of the target a profile's code is written for, when its `$config.dat` cannot be
/// read: the VW KS24 standard's for `vw-ks24`, none beyond a stock controller's for `sim`.
pub fn bundled_globals(profile: &str) -> BTreeSet<String> {
    match profile {
        "vw-ks24" => crate::vw::VW_CONFIG_GLOBALS.iter().map(|x| x.to_string()).collect(),
        _ => BTreeSet::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn data_lists_declare_with_and_without_decl() {
        let d = declared(
            "DEFDAT $CONFIG\r\n;FOLD USER GLOBALS\r\nINT SRB=9\r\nDECL GLOBAL BOOL A, B=FALSE ; two\r\n\
             SIGNAL PRG_NR $IN[1] TO $IN[8]\r\nDECL E6POS XHOME1={A1 0.0,A2 -90.0}\r\nREAL R[10]\r\n\
             EXTFCT INT SELECT (SUBTYPE :IN)\r\nEXT VW (VW_COMMAND :IN)\r\nSTRUC MYS INT A\r\nENUM MYE X,Y\r\n\
             ; DECL INT COMMENTED\r\nENDDAT\r\n",
        );
        for n in ["SRB", "A", "B", "PRG_NR", "XHOME1", "R", "SELECT", "VW", "MYS", "MYE"] {
            assert!(d.contains(n), "{n}: {d:?}");
        }
        assert!(!d.contains("COMMENTED") && !d.contains("CONFIG") && !d.contains("FALSE"), "{d:?}");
    }

    #[test]
    fn a_src_declares_by_decl_def_and_labels_not_by_assignments() {
        let d = defined_in_src("DEF FOLGE11()\r\nDECL INT I\r\nX=Y\r\nPEXIT:\r\nEND\r\nGLOBAL DEFFCT BOOL F()\r\nENDFCT\r\n");
        for n in ["FOLGE11", "I", "PEXIT", "F"] {
            assert!(d.contains(n), "{n}: {d:?}");
        }
        assert!(!d.contains("Y") && !d.contains("X"), "{d:?}");
    }

    #[test]
    fn a_module_local_signal_is_unknown() {
        let src = "&ACCESS RVP\r\nDEF FOLGE11()\r\n   P_PktNr_SPS=1 ; op\r\n   BT_PKTNR=1\r\n   SEL_RES=SELECT(#MAKRO,0,TRUE)\r\n\
                   IF N_VW==TRUE THEN\r\n      GOTO PEXIT\r\n   ENDIF\r\n   PEXIT:\r\n   $OUT[15]=FALSE\r\n   M=\"P_X\"\r\nEND\r\n";
        let dat = "DEFDAT FOLGE11\r\nEXTFCT INT SELECT (SUBTYPE :IN)\r\nENDDAT\r\n";
        let cfg = "SIGNAL BT_PKTNR $OUT[97] TO $OUT[112]\r\nDECL INT M\r\n";
        let files = [("KRC/R1/Folgen/folge11.src", src), ("KRC/R1/Folgen/folge11.dat", dat), ("KRC/R1/System/bt_config.dat", cfg)];
        let u = unknown_names(files, &bundled_globals("vw-ks24"));
        assert_eq!(u.iter().map(|u| (u.name.as_str(), u.line)).collect::<Vec<_>>(), [("P_PktNr_SPS", 3)]);
        assert_eq!(u[0].text, "P_PktNr_SPS=1 ; op");
        // A stock controller does not declare the VW globals either.
        let u = unknown_names(files, &bundled_globals("sim"));
        assert_eq!(u.iter().map(|u| u.name.as_str()).collect::<Vec<_>>(), ["P_PktNr_SPS", "SEL_RES", "N_VW"]);
    }
}
