//! Build script: embed the backend archive into the binary.
//!
//! The supervisor ships the cell-runner backend (its binary and shared
//! libraries) as a single zip that is `include_bytes!`-embedded so the produced
//! executable is self-contained. The archive is produced by
//! `cell/scripts/build-cell-rerun-server.sh` after it compiles the cell-runner, and its path is passed
//! in via the `CELL_RERUN_BACKEND_ARCHIVE` environment variable.
//!
//! When the variable is unset (e.g. a plain `cargo build` for development or
//! CI), we embed a ZERO-BYTE archive instead so the crate always compiles. The
//! runtime detects the empty archive and reports `backendsEmbedded: false`, and
//! `start` returns a descriptive error rather than crashing.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-env-changed=CELL_RERUN_BACKEND_ARCHIVE");

    let out_dir = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR not set"));
    let dest = out_dir.join("backend.zip");

    match env::var("CELL_RERUN_BACKEND_ARCHIVE") {
        Ok(src) if !src.is_empty() => {
            println!("cargo:rerun-if-changed={src}");
            let bytes = fs::read(&src)
                .unwrap_or_else(|e| panic!("failed to read CELL_RERUN_BACKEND_ARCHIVE '{src}': {e}"));
            fs::write(&dest, &bytes).expect("failed to stage backend archive into OUT_DIR");
        }
        _ => {
            // No archive supplied — embed an empty placeholder so the crate
            // still builds. The runtime treats an empty archive as "backends
            // not bundled".
            fs::write(&dest, []).expect("failed to write empty backend archive placeholder");
        }
    }
}
