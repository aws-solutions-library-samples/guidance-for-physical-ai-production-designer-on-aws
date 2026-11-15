//! ST templates embedded at build time (see `build.rs`).

include!(concat!(env!("OUT_DIR"), "/templates.rs"));
