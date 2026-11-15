//! Control Code Export: a saved behaviour-tree plan of the active cell → head-PLC code and robot
//! programs, built by the backend with the `plan2cell` CLI (`plan2cell build --json`).
//!
//! * `GET` [`crate::api::CODEGEN_OPTIONS`]: [`CodegenOptions`], what can be built (planned
//!   targets come back with `implemented: false`) and the defaults.
//! * `POST` [`crate::api::CODEGEN_BUILD`] with a [`BuildRequest`]: `202` [`BuildAccepted`]; `409`
//!   while another build runs. Progress on [`crate::keys::CODEGEN_STATUS`] ([`CodegenStatus`]).
//! * `GET` [`crate::api::CODEGEN_BUILDS`]: [`BuildList`], newest first;
//!   [`crate::api::codegen_build`]: one [`BuildInfo`]; [`crate::api::codegen_file`]: one generated
//!   file, raw; [`crate::api::codegen_archive`]: the whole build as a zip.

use serde::{Deserialize, Serialize};

/// One selectable value: a PLC dialect, robot language, profile or job number mode.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct Choice {
    pub id: String,
    pub label: String,
    /// Planned but not generated yet: shown, but not selectable.
    pub implemented: bool,
    pub description: String,
}

/// The defaults of a build ([`BuildRequest`] without the plan).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CodegenDefaults {
    pub profile: String,
    pub plc_dialect: String,
    pub robot_language: String,
    pub robot_sim: bool,
    /// `[plc.cell] job_number`: `word_le` or `byte`.
    pub job_number: String,
    pub opcua_symbols: bool,
    pub st_check: bool,
    pub strict: bool,
}

impl Default for CodegenDefaults {
    fn default() -> Self {
        Self {
            profile: "sim".to_owned(),
            plc_dialect: "codesys".to_owned(),
            robot_language: "krl".to_owned(),
            robot_sim: true,
            job_number: "word_le".to_owned(),
            opcua_symbols: false,
            st_check: true,
            strict: false,
        }
    }
}

/// `GET /api/v1/codegen/options`: what the binary can build; [`Self::defaults`] are its defaults
/// with the active cell's own `plan2cell/plan2cell.toml` on top (the values a build of that cell
/// gets when the form is left alone).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CodegenOptions {
    pub plc_dialects: Vec<Choice>,
    pub robot_languages: Vec<Choice>,
    pub profiles: Vec<Choice>,
    pub job_numbers: Vec<Choice>,
    pub defaults: CodegenDefaults,
    /// The plan2cell binary's version (`plan2cell 0.1.0`).
    pub generator: String,
    /// Whether that binary can type-check the generated ST (else `stCheck` is ignored).
    pub st_check_available: bool,
    /// The active cell whose plan2cell.toml is in [`Self::defaults`] (empty: no active cell).
    pub cell: String,
}

/// `POST /api/v1/codegen/build`.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct BuildRequest {
    /// A saved plan of the active cell: its [`crate::PlanInfo::path`]
    /// (`agentic_planner/plans/<name>.json`) or just `<name>`.
    pub plan: String,
    pub profile: String,
    pub plc_dialect: String,
    pub robot_language: String,
    pub robot_sim: bool,
    pub job_number: String,
    pub opcua_symbols: bool,
    pub st_check: bool,
    pub strict: bool,
}

impl BuildRequest {
    pub fn new(plan: impl Into<String>, defaults: &CodegenDefaults) -> Self {
        Self {
            plan: plan.into(),
            profile: defaults.profile.clone(),
            plc_dialect: defaults.plc_dialect.clone(),
            robot_language: defaults.robot_language.clone(),
            robot_sim: defaults.robot_sim,
            job_number: defaults.job_number.clone(),
            opcua_symbols: defaults.opcua_symbols,
            st_check: defaults.st_check,
            strict: defaults.strict,
        }
    }
}

/// The `202` answer to a [`BuildRequest`].
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct BuildAccepted {
    pub build_id: String,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    Error,
    Warning,
    #[default]
    Info,
}

/// One diagnostic of a build.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct Diagnostic {
    pub severity: Severity,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub code: Option<String>,
    pub message: String,
    /// A file of the build (relative, as in [`BuildFile::path`]).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub file: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub line: Option<u64>,
    /// Where in the plan (behaviour-tree path and node), if anywhere.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub location: Option<String>,
    /// `plan`, `plc`, `krl`, `st` (the ST type check) or `backend`.
    pub source: String,
}

/// A file of a build, relative to it (`plc/codesys/head/POU/FB_Cell_Body.st`).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct BuildFile {
    pub path: String,
    pub bytes: u64,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct RobotSummary {
    pub ns: String,
    pub tag: String,
    pub ops: u64,
    /// The KRL Folgen written for it (production, home).
    pub folgen: Vec<u32>,
    pub krl_points: u64,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct ZoneSummary {
    pub name: String,
    /// The order robots get the zone in (`robot1.W01`).
    pub tickets: Vec<String>,
}

/// What a successful build made.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct BuildSummary {
    /// One line, as the CLI prints it.
    pub line: String,
    pub tree_id: String,
    pub cell_id: String,
    /// `16#8024`: the head PLC and every robot program carry it.
    pub plan_id: String,
    pub makespan_s: f64,
    pub st_checked: bool,
    pub robots: Vec<RobotSummary>,
    pub zones: Vec<ZoneSummary>,
}

/// `GET /api/v1/codegen/builds/{id}`, and `build.json` in the build's folder.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct BuildInfo {
    /// `<UTC time>-<plan>`, e.g. `20261012T153045123Z-bt_1790934549_welding_cell_2_small`.
    pub id: String,
    /// The plan, as [`crate::PlanInfo::path`].
    pub plan: String,
    /// Unix seconds.
    pub created_s: f64,
    pub duration_s: f64,
    pub request: BuildRequest,
    pub ok: bool,
    /// plan2cell's exit code (0 ok, 3 config, 5 not implemented, 6 strict, 8 ST rejected, ...).
    pub exit_code: Option<i32>,
    pub summary: Option<BuildSummary>,
    pub diagnostics: Vec<Diagnostic>,
    pub files: Vec<BuildFile>,
    pub notes: Vec<String>,
    pub error: Option<String>,
    pub generator: String,
    /// The plan file's ETag when it was built (as [`crate::api::CELL_PLAN`] sends it).
    pub plan_etag: String,
    /// The cell's own `plan2cell/plan2cell.toml` when it was built (its ETag), if it had one.
    pub base_config_etag: Option<String>,
    /// Whether the cell's saved Hardware Configuration went into the build's plan2cell.toml.
    pub hardware_source: crate::hardware::HardwareSource,
    /// That configuration (saved builds only).
    pub hardware: Option<crate::hardware::HardwareConfig>,
    /// `<cell>/plan2cell/hardware.json` when it was built (its ETag), if there was one.
    pub hardware_etag: Option<String>,
    /// The plan, the cell's plan2cell.toml or its hardware configuration changed since: computed
    /// when served.
    pub stale: bool,
}

impl BuildInfo {
    /// How many diagnostics of `severity` there are.
    pub fn count(&self, severity: Severity) -> usize {
        self.diagnostics.iter().filter(|d| d.severity == severity).count()
    }
}

/// One entry of [`BuildList`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct BuildEntry {
    pub id: String,
    pub plan: String,
    pub created_s: f64,
    pub ok: bool,
    pub stale: bool,
    pub errors: u64,
    pub warnings: u64,
    /// The newest build of its plan.
    pub latest: bool,
    /// Its PLAN_ID (`16#9201`, from its summary), if it has one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub plan_id: Option<String>,
    /// [`BuildInfo::hardware_etag`]: the saved Hardware Configuration it was built with, if any.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hardware_etag: Option<String>,
}

/// A PLAN_ID as an IEC literal without quotes and type prefix: `'WORD#16#9573'` → `16#9573`.
pub fn plain_plan_id(s: &str) -> &str {
    let s = s.trim().trim_matches('\'').trim();
    let s = s.rsplit_once(':').map_or(s, |(_, v)| v).trim();
    match s.split_once('#') {
        Some((ty, rest)) if !ty.is_empty() && ty.chars().all(|c| c.is_ascii_alphabetic()) => rest,
        _ => s,
    }
}

/// The number of a PLAN_ID as CODESYS or an export writes it: `16#9573`, `WORD#16#9573`,
/// `WORD#38259`, `UINT#38259`, `'16#9573'`, `38259` (digits only: decimal), `0x9573` or plain hex
/// with a letter (`95A3`).
pub fn plan_id_number(s: &str) -> Option<u64> {
    let s = plain_plan_id(s).replace('_', "");
    match s.split_once('#') {
        Some((base, digits)) => {
            let radix = base.parse::<u32>().ok().filter(|r| (2..=36).contains(r))?;
            u64::from_str_radix(digits, radix).ok()
        }
        None => match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
            Some(hex) => u64::from_str_radix(hex, 16).ok(),
            None if s.bytes().all(|c| c.is_ascii_digit()) => s.parse().ok(),
            None => u64::from_str_radix(&s, 16).ok(),
        },
    }
}

/// Two PLAN_IDs as CODESYS writes them (`16#9573`, `38259`, `'16#9573'`, `WORD#16#9573`) are the
/// same number ([`plan_id_number`]).
pub fn same_plan_id(a: &str, b: &str) -> bool {
    match (plan_id_number(a), plan_id_number(b)) {
        (Some(x), Some(y)) => x == y,
        _ => plain_plan_id(a).eq_ignore_ascii_case(plain_plan_id(b)),
    }
}

/// `GET /api/v1/codegen/builds`: the active cell's builds, newest first.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct BuildList {
    pub cell: String,
    pub builds: Vec<BuildEntry>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CodegenState {
    #[default]
    Idle,
    Running,
    Done,
    Failed,
}

/// On `factoryos/codegen/status` (also a queryable).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct CodegenStatus {
    pub state: CodegenState,
    /// The running or last build.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub build_id: Option<String>,
    /// The cell it builds (build ids are looked up in the active cell).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cell: Option<String>,
    /// Its plan.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub plan: Option<String>,
    /// What happens, or why it failed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
}

/// `path` for a URL: every byte outside `[A-Za-z0-9._~/-]` percent-encoded (the `/`s stay).
pub fn encode_path(path: &str) -> String {
    let mut out = String::with_capacity(path.len());
    for b in path.bytes() {
        if b.is_ascii_alphanumeric() || matches!(b, b'.' | b'_' | b'~' | b'/' | b'-') {
            out.push(b as char);
        } else {
            out.push_str(&format!("%{b:02X}"));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wire_format_is_camel_case() {
        let request = BuildRequest::new("agentic_planner/plans/bt_1.json", &CodegenDefaults::default());
        let json = serde_json::to_value(&request).unwrap();
        assert_eq!(json["plcDialect"], "codesys");
        assert_eq!(json["jobNumber"], "word_le");
        assert_eq!(json["opcuaSymbols"], false);
        let status = CodegenStatus {
            state: CodegenState::Running,
            build_id: Some("x".to_owned()),
            ..Default::default()
        };
        assert_eq!(serde_json::to_string(&status).unwrap(), r#"{"state":"running","buildId":"x"}"#);
        // Lenient: missing fields default.
        let info: BuildInfo = serde_json::from_str(r#"{"id":"a","ok":true,"diagnostics":[{"severity":"warning","message":"m"}]}"#).unwrap();
        assert_eq!(info.count(Severity::Warning), 1);
    }

    #[test]
    fn paths_are_encoded_for_urls() {
        assert_eq!(encode_path("plc/codesys/a b#.st"), "plc/codesys/a%20b%23.st");
        assert_eq!(encode_path("krl/robot1/KRC/R1/folge11.src"), "krl/robot1/KRC/R1/folge11.src");
    }
}
