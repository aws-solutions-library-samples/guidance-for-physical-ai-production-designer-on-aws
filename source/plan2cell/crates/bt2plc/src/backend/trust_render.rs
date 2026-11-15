//! Render a [`PlcProject`] as trust-platform sources (IEC-complete text).
//!
//! - DUT: as is. GVL: `NAMESPACE <name>` … `END_NAMESPACE`, `VAR_GLOBAL CONSTANT` → `VAR_GLOBAL`
//!   (trust rejects aggregate initializers in CONSTANT globals).
//! - POU: declaration + implementation + `END_FUNCTION_BLOCK` / `END_PROGRAM`.
//! - Enum access `E_X.Y` → `E_X#Y` for the project's enum types (token-aware: comments and
//!   strings are untouched; one character for one, so columns are preserved).
//!
//! Every segment is inserted as whole lines and recorded, so a diagnostic in the rendered text
//! maps back to `file:line:col` of the CODESYS source.

use std::collections::BTreeSet;

use crate::backend::model::{ObjKind, PlcProject};

#[derive(Clone, Debug)]
pub struct Segment {
    /// First line of the segment in the rendered file (1-based).
    pub first_line: usize,
    pub n_lines: usize,
    /// CODESYS source file (output-relative path).
    pub source: String,
    /// Lines prepended before the source text inside this segment.
    pub source_offset: usize,
}

#[derive(Clone, Debug)]
pub struct RenderedFile {
    /// e.g. `FB_Cell.st`
    pub name: String,
    pub text: String,
    pub segments: Vec<Segment>,
}

impl RenderedFile {
    /// Map a 1-based rendered line to (source file, 1-based source line).
    pub fn map_line(&self, line: usize) -> (String, usize) {
        for s in &self.segments {
            if line >= s.first_line && line < s.first_line + s.n_lines {
                let l = line - s.first_line + 1;
                return (s.source.clone(), l.saturating_sub(s.source_offset).max(1));
            }
        }
        (format!("trust/{}", self.name), line)
    }
}

pub fn render(p: &PlcProject) -> Vec<RenderedFile> {
    let enums: BTreeSet<&str> = p.enums.iter().map(String::as_str).collect();
    p.objects
        .iter()
        .map(|o| {
            let mut text = String::new();
            let mut segments = Vec::new();
            let mut push =
                |text: &mut String, part: &str, source: Option<String>, offset: usize| {
                    let first_line = text.matches('\n').count() + 1;
                    let mut part = part.to_string();
                    if !part.ends_with('\n') {
                        part.push('\n');
                    }
                    let n_lines = part.matches('\n').count();
                    text.push_str(&part);
                    if let Some(source) = source {
                        segments.push(Segment {
                            first_line,
                            n_lines,
                            source,
                            source_offset: offset,
                        });
                    }
                };
            match o.kind {
                ObjKind::Dut => push(
                    &mut text,
                    &rewrite_enums(&o.decl, &enums),
                    Some(p.decl_path(o)),
                    0,
                ),
                ObjKind::Gvl => {
                    let decl =
                        rewrite_enums(&o.decl, &enums).replace("VAR_GLOBAL CONSTANT", "VAR_GLOBAL");
                    push(&mut text, &format!("NAMESPACE {}", o.name), None, 0);
                    push(&mut text, &decl, Some(p.decl_path(o)), 0);
                    push(&mut text, "END_NAMESPACE", None, 0);
                }
                ObjKind::FunctionBlock | ObjKind::Program => {
                    push(
                        &mut text,
                        &rewrite_enums(&o.decl, &enums),
                        Some(p.decl_path(o)),
                        0,
                    );
                    if let Some(b) = &o.body {
                        push(&mut text, &rewrite_enums(b, &enums), p.body_path(o), 0);
                    }
                    let end = if o.kind == ObjKind::FunctionBlock {
                        "END_FUNCTION_BLOCK"
                    } else {
                        "END_PROGRAM"
                    };
                    push(&mut text, end, None, 0);
                }
            }
            RenderedFile {
                name: format!("{}.st", o.name),
                text,
                segments,
            }
        })
        .collect()
}

/// `E_X.Y` → `E_X#Y` outside comments and strings, for known enum type names.
pub fn rewrite_enums(text: &str, enums: &BTreeSet<&str>) -> String {
    let b = text.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    let is_id = |c: u8| c.is_ascii_alphanumeric() || c == b'_';
    while i < b.len() {
        if b[i] == b'(' && b.get(i + 1) == Some(&b'*') {
            let end = text[i + 2..]
                .find("*)")
                .map(|k| i + 2 + k + 2)
                .unwrap_or(b.len());
            out.extend_from_slice(&b[i..end]);
            i = end;
        } else if b[i] == b'\'' {
            let mut j = i + 1;
            while j < b.len() && b[j] != b'\'' {
                j += if b[j] == b'$' { 2 } else { 1 };
            }
            let end = (j + 1).min(b.len());
            out.extend_from_slice(&b[i..end]);
            i = end;
        } else if is_id(b[i]) && (i == 0 || !is_id(b[i - 1])) {
            let mut j = i;
            while j < b.len() && is_id(b[j]) {
                j += 1;
            }
            let ident = &text[i..j];
            out.extend_from_slice(ident.as_bytes());
            let after_dot = i > 0 && b[i - 1] == b'.';
            if !after_dot
                && enums.contains(ident)
                && b.get(j) == Some(&b'.')
                && b.get(j + 1)
                    .is_some_and(|c| c.is_ascii_alphabetic() || *c == b'_')
            {
                out.push(b'#');
                j += 1;
            }
            i = j;
        } else {
            out.push(b[i]);
            i += 1;
        }
    }
    String::from_utf8(out).expect("ASCII-preserving rewrite")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enum_rewrite_is_token_aware() {
        let enums: BTreeSet<&str> = ["E_X"].into_iter().collect();
        let src = "a := E_X.A; (* E_X.B *) s := 'E_X.C'; b := st.E_X.D; c := GVL_Cell.N;\nCASE e OF E_X.A, E_X.B: x := 1; END_CASE";
        assert_eq!(
            rewrite_enums(src, &enums),
            "a := E_X#A; (* E_X.B *) s := 'E_X.C'; b := st.E_X.D; c := GVL_Cell.N;\nCASE e OF E_X#A, E_X#B: x := 1; END_CASE"
        );
    }
}
