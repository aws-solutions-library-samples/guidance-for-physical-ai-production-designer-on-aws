//! `plan2cell.toml`: one file for the whole build.
//!
//! ```toml
//! [build]          # defaults of the CLI flags
//! profile = "sim"
//! plc = "codesys"
//! robot_sim = true
//! krl = true
//!
//! [plc]            # exactly bt2plc.toml (cell, protocol, timing, network, codesys, robot_sim, [[plc.robot]])
//! [krl]            # exactly bt2krl.toml (output, motion, sim, vw_ks24, [[krl.robot]])
//! ```
//!
//! Every field is optional except what bt2krl needs per robot (`[[krl.robot]] e_axes`) when KRL
//! is built. Unknown keys are rejected in every section.

use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::{BuildOptions, Error, PlcTarget, Profile, Result};

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct BuildConfig {
    pub build: BuildDefaults,
    pub plc: bt2plc::Config,
    pub krl: bt2krl::Config,
}

/// Defaults for the build options; the CLI flags and the MCP tool arguments override them.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct BuildDefaults {
    pub profile: Profile,
    pub plc: PlcTarget,
    pub robot_sim: bool,
    pub krl: bool,
    pub st_check: bool,
    pub strict: bool,
}

impl Default for BuildDefaults {
    fn default() -> Self {
        let o = BuildOptions::default();
        Self {
            profile: o.profile,
            plc: o.plc,
            robot_sim: o.robot_sim,
            krl: o.krl,
            st_check: true,
            strict: o.strict,
        }
    }
}

impl BuildConfig {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).map_err(|e| Error::io(path, e))?;
        Self::parse(&text).map_err(|e| match e {
            Error::Config(m) => Error::Config(format!("{}: {m}", path.display())),
            other => other,
        })
    }

    pub fn parse(text: &str) -> Result<Self> {
        let c = Self::parse_unchecked(text)?;
        c.plc.validate()?;
        c.krl.validate()?;
        Ok(c)
    }

    /// Parse only (unknown keys and wrong types are still errors); the values are not
    /// validated. For `plan2cell hardware`, which reports every problem of the file.
    pub fn parse_unchecked(text: &str) -> Result<Self> {
        toml::from_str(text).map_err(|e| Error::Config(e.to_string()))
    }

    /// [`Self::load`] with [`Self::parse_unchecked`].
    pub fn load_unchecked(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).map_err(|e| Error::io(path, e))?;
        Self::parse_unchecked(&text).map_err(|e| match e {
            Error::Config(m) => Error::Config(format!("{}: {m}", path.display())),
            other => other,
        })
    }

    /// The build options these defaults describe.
    pub fn options(&self) -> BuildOptions {
        let b = &self.build;
        BuildOptions {
            profile: b.profile,
            plc: b.plc,
            robot_sim: b.robot_sim,
            krl: b.krl,
            krl_robots: None,
            st_check: b.st_check && cfg!(feature = "check"),
            strict: b.strict,
        }
    }

    /// Every default, as a commented file.
    pub fn default_toml() -> String {
        let mut s = String::from(
            "# plan2cell.toml: one configuration for the head PLC ([plc], as bt2plc.toml) and the\n\
             # robot programs ([krl], as bt2krl.toml). Every value shown is the default.\n\
             #\n\
             # Per robot, KRL needs the external axes declared (bt2krl does not guess):\n\
             #   [[krl.robot]]\n\
             #   ns = \"robot1\"\n\
             #   e_axes = \"none\"            # or \"fixed\" with e_values = [...]\n\
             #   model = \"KR240R2900_ULTRA\"  # or [[plc.robot]] model\n\
             #\n\
             # Profile vw-ks24 (real VW KS24 robots) also wants the zone -> interlock map:\n\
             #   [[plc.robot]]\n\
             #   ns = \"robot1\"\n\
             #   interlocks = { IL01 = 1, IL02 = 11 }\n\
             #   [plc.protocol]\n\
             #   plan_id_byte = 12          # a free byte for the PLAN_ID check (0 = none)\n\n",
        );
        s.push_str(&toml::to_string_pretty(&BuildConfig::default()).expect("config serialises"));
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_round_trips() {
        let back = BuildConfig::parse(&BuildConfig::default_toml()).unwrap();
        assert_eq!(back, BuildConfig::default());
    }

    #[test]
    fn unknown_keys_are_rejected_in_every_section() {
        for t in [
            "[build]\nplcc = 1\n",
            "[plc.cell]\nprod_folg = 3\n",
            "[krl.motion]\nmod = 1\n",
        ] {
            assert!(BuildConfig::parse(t).is_err(), "{t}");
        }
    }

    #[test]
    fn sections_are_the_tool_configs() {
        let c = BuildConfig::parse(
            "[build]\nprofile = \"vw-ks24\"\nkrl = false\n\
             [plc.cell]\nprod_folge = 12\n\
             [[krl.robot]]\nns = \"r\"\ne_axes = \"none\"\n",
        )
        .unwrap();
        assert_eq!(c.build.profile, Profile::VwKs24);
        assert!(!c.options().krl);
        assert_eq!(c.plc.cell.prod_folge, 12);
        assert_eq!(c.krl.robots[0].ns, "r");
    }
}
