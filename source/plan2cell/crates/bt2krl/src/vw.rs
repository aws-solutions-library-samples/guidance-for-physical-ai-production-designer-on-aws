//! Profile `vw-ks24`: signal map and interlock assignment of the VW KS24 standard
//! (TASK 7.3, `samples-programs.md` section 3, `samples-config.md` section 2).
//!
//! Verified against the 8 sample robots: the signal names and ranges come from
//! `KRC/R1/VW_User/vw_usr_s.dat` (`O_R_Anf_V1..V16 = 81..96`, `O_R_PFO = 15`, `O_R_F_ENDE = 14`),
//! the interlock handshake from `KRC/R1/Makros/makro20.src`, and the outputs of the op index
//! (`SIGNAL P_PktNr_SPS $OUT[97] TO $OUT[112]` in `VW_User/vw_usr_r.dat`).
//!
//! `P_PktNr_SPS` is a *module-local* signal of `vw_usr_r` (no `GLOBAL`, and `$config.dat` does
//! not declare it): the VW Folgen set it only through the technology package
//! (`VW_USR_R(#USR_ADV, ...)` with `PAR5`). A Folge that writes `P_PktNr_SPS=k` itself does not
//! compile (KSS error 2137 "name unknown"), on the VW controllers as on the VW port of the KUKA
//! virtual controllers. bt2krl therefore declares its own global signal [`STEP_SIGNAL`] on the
//! same outputs in `bt_config.dat` (merged into `$config.dat`); the head PLC reads the same
//! `$OUT[97..112]`.

use std::collections::BTreeMap;

use crate::config::{Config, VwKs24Cfg};
use crate::error::{Error, Result};
use crate::program::Program;

/// Interlock n (1..16): release "area free" (TRUE = this robot is NOT inside).
pub fn release_out(n: u8) -> u16 {
    40 + u16::from(n)
}

/// Interlock n: request to the PLC ("Anforderung Roboterverriegelung").
pub fn request_out(n: u8) -> u16 {
    80 + u16::from(n)
}

/// Interlock n: the PLC's acknowledgement of the request.
pub fn grant_in(n: u8) -> u16 {
    80 + u16::from(n)
}

/// Interlock n: the partner robot's release, routed by the PLC.
pub fn partner_in(n: u8) -> u16 {
    40 + u16::from(n)
}

/// `O_R_PFO`: robot in home position (Grundstellung).
pub const PF0_OUT: u16 = 15;
/// First `$OUT` of `VW_FEH_BIN_A` (error code, 8 bits).
pub const ERROR_OUT: u16 = 225;
/// `$CYCFLAG` of the online driving condition that INTERRUPT 10 watches.
pub const FB_CYCFLAG: u8 = 254;
/// `$CYCFLAG` term that the VW package keeps (`FB_CHECK`).
pub const FB_CYCFLAG_KEEP: u8 = 255;
/// "Maschinelle Sicherheiten" (MAKRO50), part of every driving condition in the samples.
pub const FB_CYCFLAG_SAFE: u8 = 30;
/// Timer the VW driving condition uses to bridge a restart (`$TIMER[61] < 0`).
pub const FB_TIMER: u8 = 61;
/// VW interrupt priority of the online stop (`VW(#FB_STOP)`).
pub const FB_INTERRUPT: u8 = 10;
/// The VW package's (module-local) signal on the op index outputs; generated code never names it.
pub const STEP_SIGNAL_VW: &str = "P_PktNr_SPS";
/// First `$OUT` of the standard `P_PktNr_SPS` group.
pub const STEP_OUT_VW: u16 = 97;
/// The global signal bt2krl declares in `bt_config.dat` for the op index.
pub const STEP_SIGNAL: &str = "BT_PKTNR";
/// The plain (non-`$`) names the generated Folge takes from the VW package's `$config.dat`
/// (declared there on every sample robot and on the VW port of the virtual controllers).
/// Every other plain name it assigns or reads must be declared by `bt_config.dat`.
pub const VW_CONFIG_GLOBALS: [&str; 4] = ["SEL_RES", "INT10", "VALCYCFL30", "N_VW"];

/// Signals generated code must never write (TASK 7.3), as `$OUT` ranges.
pub const RESERVED_OUT: [(u16, u16); 3] = [
    (33, 40),   // Arbeitsfertigmeldungen A33-36, Typinfo A37-39, $COLL_ALARM A40
    (57, 80),   // Werkzeugfreigaben
    (225, 240), // VW_FEH_BIN_A error code, Fzg_Typ_SPS vehicle type
];
/// Inputs generated code must never use as its own (brake / mastering test requests).
pub const RESERVED_IN: [(u16, u16); 1] = [(249, 250)];

/// Zone name -> VW interlock index for one robot.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Interlocks {
    pub map: BTreeMap<String, u8>,
}

impl Interlocks {
    /// Configured map, or the default "zones in name order -> 1, 2, ...".
    pub fn resolve(p: &Program, cfg: &Config) -> Result<Interlocks> {
        let v = &cfg.vw_ks24;
        let ns = &p.robot.ns;
        let used: Vec<&str> = p
            .zones
            .iter()
            .filter(|z| p.ops.iter().any(|o| o.zones.iter().any(|x| x == &z.name)))
            .map(|z| z.name.as_str())
            .collect();
        // program.json carries the map when bt2plc ran with protocol.profile = "vw-ks24":
        // that is the one source of truth for both sides (TASK section 9). A map in
        // bt2krl.toml may fill it in, but must not contradict it.
        let from_plan: BTreeMap<String, u8> = p
            .zones
            .iter()
            .filter_map(|z| z.interlock.filter(|&n| n > 0).map(|n| (z.name.clone(), n)))
            .collect();
        let own = cfg
            .robot(ns)
            .map(|r| r.interlocks.clone())
            .unwrap_or_default();
        for (z, n) in &from_plan {
            if let Some(c) = own.get(z)
                && c != n
            {
                return Err(Error::Config(format!(
                    "robot {ns}: zone {z} is interlock {c} in bt2krl.toml but {n} in \
                     program.json (bt2plc [[robot]] interlocks): they must agree"
                )));
            }
        }
        let cfgmap = if from_plan.is_empty() { own } else { from_plan };
        let mut map = BTreeMap::new();
        if cfgmap.is_empty() {
            if used.len() > v.max_interlocks as usize {
                return Err(Error::Input(format!(
                    "robot {ns}: {} zones need more than the {} VW interlocks; configure \
                     [[robot]] interlocks or reduce the zones",
                    used.len(),
                    v.max_interlocks
                )));
            }
            for (i, z) in used.iter().enumerate() {
                map.insert((*z).to_string(), i as u8 + 1);
            }
        } else {
            for z in &used {
                let n = cfgmap.get(*z).copied().ok_or_else(|| {
                    Error::Config(format!(
                        "robot {ns}: zone {z} has no entry in [[robot]] interlocks"
                    ))
                })?;
                map.insert((*z).to_string(), n);
            }
            let mut seen = BTreeMap::new();
            for (z, n) in &map {
                if let Some(other) = seen.insert(*n, z.clone()) {
                    return Err(Error::Config(format!(
                        "robot {ns}: interlock {n} is used by {other} and {z}"
                    )));
                }
            }
            for z in cfgmap.keys() {
                if !p.zones.iter().any(|x| &x.name == z) {
                    return Err(Error::Config(format!(
                        "robot {ns}: interlocks has zone {z}, which the plan does not have"
                    )));
                }
            }
        }
        Ok(Interlocks { map })
    }

    /// Interlock indices of a zone mask, ascending. The permit bit is not supported.
    pub fn indices(&self, p: &Program, mask: u32) -> Result<Vec<u8>> {
        let mut v = Vec::new();
        for b in crate::io::bits(mask) {
            if b == 31 {
                return Err(Error::Input(
                    "profile vw-ks24 does not support the permit bit".into(),
                ));
            }
            let z = p
                .zones
                .iter()
                .find(|z| u32::from(z.bit) == b)
                .ok_or_else(|| Error::Input(format!("zone bit {b} is not in zones[]")))?;
            let n =
                self.map.get(&z.name).copied().ok_or_else(|| {
                    Error::Input(format!("zone {} has no interlock index", z.name))
                })?;
            v.push(n);
        }
        v.sort_unstable();
        Ok(v)
    }
}

/// First `$OUT` of the step signal, and its name ([`STEP_SIGNAL`], declared by `bt_config.dat`).
pub fn step_signal(v: &VwKs24Cfg) -> (u16, String) {
    let base = v
        .step_signal
        .split(':')
        .nth(1)
        .and_then(|x| x.parse().ok())
        .unwrap_or(STEP_OUT_VW);
    (base, STEP_SIGNAL.to_string())
}

/// The permit bit cannot be mapped: name the edges it stands for.
pub fn permit_error(p: &Program) -> Error {
    let mut edges: Vec<String> = Vec::new();
    for op in &p.ops {
        if op.mask() & (1 << 31) != 0 {
            let w: Vec<&str> = op
                .waits_for
                .iter()
                .filter(|w| w.starts_with("PERMIT"))
                .map(String::as_str)
                .collect();
            edges.push(format!(
                "op {} {} ({})",
                op.index,
                op.operation_id,
                if w.is_empty() {
                    "permit bit set".to_string()
                } else {
                    w.join(", ")
                }
            ));
        }
    }
    Error::Input(format!(
        "robot {}: profile vw-ks24 has no permit bit, but the plan needs it for: {}. \
         Re-run bt2plc with fewer zones per robot (lower protocol.max_zones is not enough: the \
         dependencies themselves must fit into {} interlocks), or use profile sim.",
        p.robot.ns,
        edges.join("; "),
        16
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signal_map_matches_the_vw_standard() {
        assert_eq!(
            (release_out(1), request_out(1), grant_in(16), partner_in(16)),
            (41, 81, 96, 56)
        );
        let v = VwKs24Cfg::default();
        // Never the VW-local P_PktNr_SPS: a Folge cannot see it (KSS error 2137).
        assert_eq!(step_signal(&v), (97, "BT_PKTNR".into()));
        let v = VwKs24Cfg {
            step_signal: "out:113:16".into(),
            ..VwKs24Cfg::default()
        };
        assert_eq!(step_signal(&v), (113, "BT_PKTNR".into()));
    }

    #[test]
    fn reserved_ranges_cover_the_vw_signals() {
        for a in [33, 36, 40, 57, 70, 80, 225, 240] {
            assert!(RESERVED_OUT.iter().any(|&(l, h)| a >= l && a <= h), "A{a}");
        }
        for a in [15, 41, 56, 81, 96, 97, 112] {
            assert!(!RESERVED_OUT.iter().any(|&(l, h)| a >= l && a <= h), "A{a}");
        }
    }
}
