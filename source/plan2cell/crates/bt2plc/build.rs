//! Embeds every ST template under `templates/codesys/{head,device}/{DUT,GVL,POU}` as
//! `(folder, name, declaration, implementation)` tables (see `src/backend/codesys/templates.rs`).

use std::fmt::Write as _;
use std::path::Path;

fn main() {
    // Read at run time (not env!): a build script cached in a copied/moved target dir must
    // still find the templates of the current checkout.
    let manifest = std::env::var("CARGO_MANIFEST_DIR").expect("cargo sets CARGO_MANIFEST_DIR");
    let root = Path::new(&manifest).join("templates/codesys");
    println!("cargo:rerun-if-changed=templates");
    let mut out = String::new();
    for set in ["head", "device"] {
        let _ = writeln!(
            out,
            "pub static {}: &[(&str, &str, &str, Option<&str>)] = &[",
            set.to_uppercase()
        );
        let mut found = 0usize;
        for folder in ["DUT", "GVL", "POU"] {
            let dir = root.join(set).join(folder);
            println!("cargo:rerun-if-changed={}", dir.display());
            let Ok(rd) = std::fs::read_dir(&dir) else {
                continue;
            };
            let mut names: Vec<String> = rd
                .filter_map(|e| e.ok())
                .map(|e| e.file_name().to_string_lossy().into_owned())
                .filter(|n| n.ends_with(".st") && !n.ends_with("_Body.st"))
                .collect();
            names.sort();
            for f in names {
                found += 1;
                let name = f.trim_end_matches(".st");
                let decl = dir.join(&f);
                let body = dir.join(format!("{name}_Body.st"));
                println!("cargo:rerun-if-changed={}", decl.display());
                let body_expr = if body.exists() {
                    println!("cargo:rerun-if-changed={}", body.display());
                    format!("Some(include_str!({:?}))", body.display().to_string())
                } else {
                    "None".to_string()
                };
                let _ = writeln!(
                    out,
                    "    ({folder:?}, {name:?}, include_str!({:?}), {body_expr}),",
                    decl.display().to_string()
                );
            }
        }
        out.push_str("];\n");
        assert!(
            found > 0,
            "no ST templates found under {}",
            root.join(set).display()
        );
    }
    let dest = Path::new(&std::env::var("OUT_DIR").unwrap()).join("templates.rs");
    std::fs::write(dest, out).unwrap();
}
