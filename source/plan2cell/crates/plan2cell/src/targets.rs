//! What plan2cell can generate, and what is planned: PLC dialects (bt2plc's [`Flavour`]s), robot
//! languages and protocol profiles, with the defaults. `plan2cell options --json` prints this for
//! front ends (the FactoryOS "Control Code Export" step), which show the planned ones disabled.

use bt2plc::backend::Flavour;
use bt2plc::config::JobNumber;
use serde::{Deserialize, Serialize};

use crate::{BuildConfig, Error, Profile, Result};

/// One selectable value.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Choice {
    pub id: String,
    pub label: String,
    pub implemented: bool,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub description: String,
}

impl Choice {
    fn new(
        id: impl Into<String>,
        label: impl Into<String>,
        implemented: bool,
        description: impl Into<String>,
    ) -> Self {
        Self {
            id: id.into(),
            label: label.into(),
            implemented,
            description: description.into(),
        }
    }
}

/// The robot-side program language. Only KUKA KRL is generated today.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, clap::ValueEnum)]
#[serde(rename_all = "kebab-case")]
pub enum RobotLanguage {
    #[default]
    Krl,
    Rapid,
    Urscript,
    Karel,
    Inform,
}

impl RobotLanguage {
    pub const ALL: [RobotLanguage; 5] = [
        RobotLanguage::Krl,
        RobotLanguage::Rapid,
        RobotLanguage::Urscript,
        RobotLanguage::Karel,
        RobotLanguage::Inform,
    ];

    pub fn id(self) -> &'static str {
        match self {
            RobotLanguage::Krl => "krl",
            RobotLanguage::Rapid => "rapid",
            RobotLanguage::Urscript => "urscript",
            RobotLanguage::Karel => "karel",
            RobotLanguage::Inform => "inform",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            RobotLanguage::Krl => "KUKA KRL",
            RobotLanguage::Rapid => "ABB RAPID",
            RobotLanguage::Urscript => "UR URScript",
            RobotLanguage::Karel => "FANUC KAREL",
            RobotLanguage::Inform => "Yaskawa INFORM",
        }
    }

    pub fn implemented(self) -> bool {
        self == RobotLanguage::Krl
    }
}

/// A PLC dialect by its id (`codesys`, `twincat3`, `tia-scl`, `plcopen-xml`, `rockwell-l5x`).
pub fn plc_dialect(id: &str) -> Option<Flavour> {
    Flavour::ALL.into_iter().find(|f| f.to_string() == id)
}

/// The short name of a flavour: its description up to the details (`" ("`) or `" - planned"`.
fn flavour_label(f: Flavour) -> String {
    let d = f.describe();
    let end = [d.find(" ("), d.find(" - ")]
        .into_iter()
        .flatten()
        .min()
        .unwrap_or(d.len());
    d[..end].to_owned()
}

/// Fail with exit code 5 (not implemented) unless both targets can be generated.
pub fn check(dialect: Flavour, language: RobotLanguage) -> Result<()> {
    if !dialect.implemented() {
        return Err(Error::Plc(bt2plc::Error::NotImplemented(dialect)));
    }
    if !language.implemented() {
        return Err(Error::NotImplemented(format!(
            "robot language {} ({}) is not implemented yet (planned; only krl is available)",
            language.id(),
            language.label()
        )));
    }
    Ok(())
}

/// The defaults a front end starts from: the build options and the `[plc]` knobs worth showing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Defaults {
    pub profile: Profile,
    pub plc_dialect: String,
    pub robot_language: RobotLanguage,
    pub robot_sim: bool,
    pub krl: bool,
    pub st_check: bool,
    pub strict: bool,
    /// `[plc.cell] job_number`: `word_le` or `byte`.
    pub job_number: JobNumber,
    /// `[plc.opcua] symbols`.
    pub opcua_symbols: bool,
}

/// What `plan2cell options --json` prints.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Options {
    pub schema: String,
    pub generator: String,
    pub plc_dialects: Vec<Choice>,
    pub robot_languages: Vec<Choice>,
    pub profiles: Vec<Choice>,
    pub job_numbers: Vec<Choice>,
    pub defaults: Defaults,
    /// Whether this binary can type-check the generated ST (feature `check`).
    pub st_check_available: bool,
    /// Whether this binary can co-simulate (feature `sim`).
    pub sim_available: bool,
}

pub const OPTIONS_SCHEMA: &str = "plan2cell/options@1";

pub fn options() -> Options {
    let cfg = BuildConfig::default();
    let o = cfg.options();
    Options {
        schema: OPTIONS_SCHEMA.into(),
        generator: crate::GENERATOR.into(),
        plc_dialects: Flavour::ALL
            .into_iter()
            .map(|f| {
                Choice::new(
                    f.to_string(),
                    flavour_label(f),
                    f.implemented(),
                    f.describe(),
                )
            })
            .collect(),
        robot_languages: RobotLanguage::ALL
            .into_iter()
            .map(|l| Choice::new(l.id(), l.label(), l.implemented(), ""))
            .collect(),
        profiles: vec![
            Choice::new(
                Profile::Sim.name(),
                "Simulation",
                true,
                "the device-simulator telegram (docker simulation, OfficeLite, KUKA virtual controllers)",
            ),
            Choice::new(
                Profile::VwKs24.name(),
                "VW KS24",
                true,
                "real VW KS24 robots (KSS 8.2, TPVW 8.1.8): VW Folge handshake and robot interlocks",
            ),
        ],
        job_numbers: vec![
            Choice::new(
                "word_le",
                "WORD, little-endian (bytes 2-3)",
                true,
                "the deployed simulator",
            ),
            Choice::new(
                "byte",
                "BYTE (byte 2)",
                true,
                "real KUKA VASS; profile vw-ks24 forces it",
            ),
        ],
        defaults: Defaults {
            profile: o.profile,
            plc_dialect: Flavour::Codesys.to_string(),
            robot_language: RobotLanguage::Krl,
            robot_sim: o.robot_sim,
            krl: o.krl,
            st_check: cfg.build.st_check,
            strict: o.strict,
            job_number: cfg.plc.cell.job_number,
            opcua_symbols: cfg.plc.opcua.symbols,
        },
        st_check_available: cfg!(feature = "check"),
        sim_available: cfg!(feature = "sim"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dialects_follow_bt2plc() {
        let o = options();
        let ids: Vec<_> = o
            .plc_dialects
            .iter()
            .map(|c| (c.id.as_str(), c.implemented))
            .collect();
        assert_eq!(
            ids,
            [
                ("codesys", true),
                ("twincat3", false),
                ("tia-scl", false),
                ("plcopen-xml", false),
                ("rockwell-l5x", false)
            ]
        );
        assert_eq!(o.plc_dialects[0].label, "CODESYS V3.5");
        assert_eq!(o.plc_dialects[1].label, "Beckhoff TwinCAT 3");
        assert_eq!(o.plc_dialects[2].label, "Siemens TIA Portal SCL");
        assert_eq!(
            o.robot_languages.iter().filter(|c| c.implemented).count(),
            1
        );
        assert_eq!(plc_dialect("tia-scl"), Some(Flavour::TiaScl));
        assert_eq!(plc_dialect("nope"), None);
    }

    #[test]
    fn planned_targets_are_not_implemented() {
        assert!(check(Flavour::Codesys, RobotLanguage::Krl).is_ok());
        assert_eq!(
            check(Flavour::Twincat3, RobotLanguage::Krl)
                .unwrap_err()
                .exit_code(),
            5
        );
        assert_eq!(
            check(Flavour::Codesys, RobotLanguage::Rapid)
                .unwrap_err()
                .exit_code(),
            5
        );
    }

    #[test]
    fn options_serialise_as_documented() {
        let v = serde_json::to_value(options()).unwrap();
        assert_eq!(v["defaults"]["job_number"], "word_le");
        assert_eq!(v["defaults"]["profile"], "sim");
        assert_eq!(v["defaults"]["robot_language"], "krl");
        assert_eq!(v["profiles"][1]["id"], "vw-ks24");
    }
}
