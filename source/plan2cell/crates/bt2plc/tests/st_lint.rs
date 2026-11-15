//! CODESYS-specific rules that the trust check cannot see (docs/ST_DIALECT.md), over every
//! generated `.st` of both samples.

mod common;
use bt2plc::backend::model::{ObjKind, strip_comments};
use bt2plc::{Config, GenerateOptions};
use common::*;

fn generated(name: &str) -> bt2plc::Generated {
    let mut d = bt2plc::Diagnostics::default();
    let doc = bt2plc::input::load_str(
        &fixture(name),
        name,
        bt2plc::input::SourceSel::Auto,
        None,
        &mut d,
    )
    .unwrap();
    bt2plc::generate(
        &doc,
        &Config::default(),
        &GenerateOptions {
            st_check: false,
            ..Default::default()
        },
    )
    .unwrap()
}

#[test]
fn generated_st_follows_the_codesys_object_rules() {
    for f in [WELDING, TWO_ROBOT] {
        let g = generated(f);
        for (path, text) in g.output.files() {
            if !path.starts_with("codesys/") || !path.ends_with(".st") {
                continue;
            }
            assert!(text.is_ascii(), "{path}: non-ASCII");
            assert!(!text.contains('\t'), "{path}: tab");
            let code = strip_comments(text);
            assert!(!code.contains("//"), "{path}: // comment");
            for kw in ["END_FUNCTION_BLOCK", "END_PROGRAM", "END_FUNCTION"] {
                assert!(!code.contains(kw), "{path}: {kw} in a CODESYS object file");
            }
            let upper = code.to_uppercase();
            assert!(!upper.contains("TIME()"), "{path}: TIME() is not portable");
            assert!(
                !upper.contains("REFERENCE TO") && !upper.contains("POINTER TO"),
                "{path}: pointers"
            );
            // Rule 2: never write an element of a GVL array.
            for line in code.lines() {
                let l = line.trim_start();
                if l.starts_with("GVL_")
                    && let Some(lhs) = l.split(":=").next()
                    && l.contains(":=")
                    && lhs.contains('[')
                {
                    panic!("{path}: GVL array element write: {l}");
                }
            }
        }
        for p in &g.projects {
            let programs = p
                .objects
                .iter()
                .filter(|o| o.kind == ObjKind::Program)
                .count();
            assert_eq!(programs, 1, "{}: exactly one PROGRAM", p.key);
            assert!(p.object("PLC_PRG").is_some());
            // Creation order: every referenced object comes earlier.
            for (i, o) in p.objects.iter().enumerate() {
                let text =
                    strip_comments(&format!("{}\n{}", o.decl, o.body.as_deref().unwrap_or("")));
                let ids = bt2plc::backend::model::identifiers(&text);
                for later in &p.objects[i + 1..] {
                    assert!(
                        !ids.contains(later.name.as_str()),
                        "{}: {} uses {} which is created later",
                        p.key,
                        o.name,
                        later.name
                    );
                }
            }
        }
    }
}
