//! Parser smoke check of bt2krl output with the lenient krl-runner parser (TASK 8.2).
//!
//! `kuka-krl-parser` is the copy in `crates/kuka-krl-parser` of krl-runner's parser
//! (`krl-runner/krl-parser`).
//!
//! It never rejects syntax and does not see AXIS/E6AXIS coordinates, SPTP or SIGNAL groups;
//! asserted are only: every Folge is found by its DEF name, the motion count matches the PTP
//! lines, and the REQUEST outputs of the production Folge appear among its IO operations.

use std::path::Path;

use bt2krl::report::{MANIFEST_FILE, Manifest};
use bt2krl::{Error, Result};

pub fn check_dir(dir: &Path) -> Result<Vec<String>> {
    let mut out = Vec::new();
    let mut entries: Vec<_> = std::fs::read_dir(dir)
        .map_err(|e| Error::io(dir, e))?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .collect();
    entries.sort();
    for d in entries {
        let mf = d.join(MANIFEST_FILE);
        if !mf.is_file() {
            continue;
        }
        let text = std::fs::read_to_string(&mf).map_err(|e| Error::io(&mf, e))?;
        let m: Manifest = serde_json::from_str(&text).map_err(|e| Error::Input(format!("{e}")))?;
        let progs = kuka_krl_parser::parser::parse_krl_project(&d.join("KRC"))
            .map_err(|e| Error::Verify(format!("{}: parser: {e}", m.robot.ns)))?;
        for f in &m.folgen {
            let name = format!("FOLGE{f}");
            let p = progs
                .iter()
                .find(|p| p.program_name.eq_ignore_ascii_case(&name))
                .ok_or_else(|| {
                    Error::Verify(format!("{}: parser does not find {name}", m.robot.ns))
                })?;
            let src_path = m
                .files
                .iter()
                .find(|h| {
                    h.path
                        .to_ascii_lowercase()
                        .ends_with(&format!("folge{f}.src"))
                })
                .map(|h| d.join(&h.path))
                .ok_or_else(|| Error::Verify(format!("{}: no file for {name}", m.robot.ns)))?;
            let src = std::fs::read_to_string(&src_path).map_err(|e| Error::io(&src_path, e))?;
            let ptp = src
                .lines()
                .filter(|l| l.trim_start().starts_with("PTP "))
                .count();
            if p.sequence.len() != ptp {
                return Err(Error::Verify(format!(
                    "{}: {name}: parser sees {} motions, the file has {ptp} PTP lines",
                    m.robot.ns,
                    p.sequence.len()
                )));
            }
            if m.check.production.ends_with(&format!("olge{f}")) {
                let zone_ops = m.check.ops.iter().filter(|o| o.mask != 0).count();
                let lo = u32::from(m.check.request_base);
                if m.profile == "vw-ks24" {
                    // The VW interlock index is not the zone bit, so only check that the request
                    // range is used at all; the strict validator checks the real mapping.
                    let seen = p
                        .ios
                        .iter()
                        .any(|io| io.io_type == "OUT" && io.channel >= lo && io.channel < lo + 16);
                    if zone_ops > 0 && !seen {
                        return Err(Error::Verify(format!(
                            "{}: {name}: parser sees no interlock request in $OUT[{lo}..{}]",
                            m.robot.ns,
                            lo + 15
                        )));
                    }
                } else {
                    for o in &m.check.ops {
                        for b in (0..32).filter(|b| o.mask & (1 << b) != 0) {
                            let ch = lo + b;
                            if !p
                                .ios
                                .iter()
                                .any(|io| io.io_type == "OUT" && io.channel == ch)
                            {
                                return Err(Error::Verify(format!(
                                    "{}: {name}: parser does not see REQUEST $OUT[{ch}]",
                                    m.robot.ns
                                )));
                            }
                        }
                    }
                }
            }
            out.push(format!(
                "{}: parser smoke {name}: {} motions, {} IO operations",
                m.robot.ns,
                p.sequence.len(),
                p.ios.len()
            ));
        }
    }
    Ok(out)
}
