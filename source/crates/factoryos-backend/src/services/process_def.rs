//! The cell's process definition (`<cell>/plan2cell/process.yaml`, [`factoryos_protocol::process`]):
//! read, a profile's sample written, and saved: validated, merged into the cell's
//! `plan2cell/plan2cell.toml` (the previous one kept as `plan2cell.toml.bak`) and the robots'
//! signals written as KRL declarations (`plan2cell/krl/<ns>_process_signals.dat`).

use std::path::{Path, PathBuf};

use axum::extract::{Query, State};
use axum::http::StatusCode;
use axum::routing::{get, post};
use axum::{Json, Router};
use factoryos_protocol::api;
use factoryos_protocol::process::{InitProcess, ProcessDefinition, ProcessIssue, ProcessView, SaveProcess, sample_yaml};

use crate::AppState;
use crate::config_store::ConfigStore;

const FILE: &str = "plan2cell/process.yaml";

pub fn router() -> Router<AppState> {
    Router::new().route(api::PROCESS, get(read).put(save)).route(api::PROCESS_INIT, post(init))
}

type HttpError = (StatusCode, String);

/// The named cell's folder, or the active cell's.
fn cell_root(config: &ConfigStore, cell: Option<&str>) -> Result<(String, PathBuf), HttpError> {
    match cell.map(str::trim).filter(|c| !c.is_empty()) {
        Some(name) => {
            let ok = name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-');
            let root = super::cells::cell_folder(config, name).filter(|r| ok && r.is_dir()).ok_or((StatusCode::NOT_FOUND, format!("no cell {name}")))?;
            Ok((name.to_owned(), root))
        }
        None => super::cells::active_cell(config).ok_or((StatusCode::NOT_FOUND, "no active cell: make one (scene_create_cell) or name one".to_owned())),
    }
}

fn parse(text: &str, path: &Path) -> Result<ProcessDefinition, HttpError> {
    serde_yaml::from_str(text).map_err(|err| (StatusCode::BAD_REQUEST, format!("{}: {err}", path.display())))
}

fn view(cell: String, root: &Path) -> Result<ProcessView, HttpError> {
    let path = root.join(FILE);
    match std::fs::read_to_string(&path) {
        Ok(yaml) => {
            let definition = parse(&yaml, &path)?;
            let issues = definition.validate();
            Ok(ProcessView { cell, path: path.display().to_string(), exists: true, definition, yaml, issues, written: Vec::new() })
        }
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => {
            let yaml = sample_yaml("sim", &[]);
            let definition = parse(&yaml, &path)?;
            Ok(ProcessView { cell, path: path.display().to_string(), exists: false, definition, yaml, ..ProcessView::default() })
        }
        Err(err) => Err((StatusCode::INTERNAL_SERVER_ERROR, format!("{}: {err}", path.display()))),
    }
}

#[derive(serde::Deserialize)]
struct CellQuery {
    cell: Option<String>,
}

async fn read(State(state): State<AppState>, Query(query): Query<CellQuery>) -> Result<Json<ProcessView>, HttpError> {
    let (cell, root) = cell_root(&state.config, query.cell.as_deref())?;
    view(cell, &root).map(Json)
}

async fn init(State(state): State<AppState>, Json(request): Json<InitProcess>) -> Result<Json<ProcessView>, HttpError> {
    let (cell, root) = cell_root(&state.config, request.cell.as_deref())?;
    let profile = if request.profile.trim().is_empty() { "sim" } else { request.profile.trim() };
    if !["sim", "vw-ks24"].contains(&profile) {
        return Err((StatusCode::BAD_REQUEST, format!("profile {profile}: sim or vw-ks24")));
    }
    let path = root.join(FILE);
    if path.exists() && !request.overwrite {
        return Err((StatusCode::CONFLICT, format!("{} exists: read it (process_get), or overwrite true", path.display())));
    }
    let yaml = sample_yaml(profile, &request.robots);
    let definition = parse(&yaml, &path)?;
    std::fs::create_dir_all(path.parent().expect("a folder")).map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    std::fs::write(&path, &yaml).map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, format!("{}: {err}", path.display())))?;
    let issues = definition.validate();
    Ok(Json(ProcessView { cell, path: path.display().to_string(), exists: true, definition, yaml, issues, written: Vec::new() }))
}

async fn save(State(state): State<AppState>, Json(request): Json<SaveProcess>) -> Result<Json<ProcessView>, (StatusCode, Json<serde_json::Value>)> {
    let fail = |status: StatusCode, message: String, issues: Vec<ProcessIssue>| (status, Json(serde_json::json!({"error": message, "issues": issues})));
    let (cell, root) = cell_root(&state.config, request.cell.as_deref()).map_err(|(s, m)| fail(s, m, Vec::new()))?;
    let path = root.join(FILE);
    let (definition, yaml) = match (request.definition, request.yaml) {
        (_, Some(yaml)) => (parse(&yaml, &path).map_err(|(s, m)| fail(s, m, Vec::new()))?, yaml),
        (Some(definition), None) => {
            let yaml = serde_yaml::to_string(&definition).map_err(|err| fail(StatusCode::INTERNAL_SERVER_ERROR, err.to_string(), Vec::new()))?;
            (definition, yaml)
        }
        // Saved in the IDE: apply the file as it is.
        (None, None) => {
            let yaml = std::fs::read_to_string(&path).map_err(|err| fail(StatusCode::NOT_FOUND, format!("{}: {err} (process_init first)", path.display()), Vec::new()))?;
            (parse(&yaml, &path).map_err(|(s, m)| fail(s, m, Vec::new()))?, yaml)
        }
    };
    let issues = definition.validate();
    if !issues.is_empty() {
        return Err(fail(StatusCode::UNPROCESSABLE_ENTITY, format!("{} problem(s); nothing saved", issues.len()), issues));
    }
    let written = apply(&root, &definition, &yaml).map_err(|err| fail(StatusCode::INTERNAL_SERVER_ERROR, format!("{err:#}"), Vec::new()))?;
    Ok(Json(ProcessView { cell, path: path.display().to_string(), exists: true, definition, yaml, issues, written }))
}

/// Write `process.yaml`, merge it into `plan2cell.toml` and write the signals' `.dat` files.
fn apply(root: &Path, definition: &ProcessDefinition, yaml: &str) -> anyhow::Result<Vec<String>> {
    let mut written = Vec::new();
    let dir = root.join("plan2cell");
    std::fs::create_dir_all(&dir)?;
    std::fs::write(root.join(FILE), yaml)?;
    written.push(FILE.to_owned());

    let toml_path = root.join(super::codegen::BASE_CONFIG);
    let before = std::fs::read_to_string(&toml_path).ok();
    let mut doc: toml::Table = before.as_deref().map(toml::from_str).transpose()?.unwrap_or_default();
    let notes = merge(&mut doc, definition);
    let after = toml::to_string_pretty(&doc)?;
    if before.as_deref() != Some(after.as_str()) {
        if let Some(before) = &before {
            std::fs::write(toml_path.with_extension("toml.bak"), before)?;
        }
        std::fs::write(&toml_path, &after)?;
        written.push(format!("{} (previous kept as plan2cell.toml.bak)", super::codegen::BASE_CONFIG));
    }
    written.extend(notes);

    let krl = dir.join("krl");
    for (ns, text) in definition.krl_signals() {
        std::fs::create_dir_all(&krl)?;
        let name = format!("{ns}_process_signals.dat");
        std::fs::write(krl.join(&name), text)?;
        written.push(format!("plan2cell/krl/{name}"));
    }
    Ok(written)
}

/// Put `definition`'s plan2cell settings into the `plan2cell.toml` table `doc`; notes on what
/// could not be applied.
fn merge(doc: &mut toml::Table, definition: &ProcessDefinition) -> Vec<String> {
    use toml::Value;
    let mut notes = Vec::new();
    let krl = doc.entry("krl").or_insert_with(|| Value::Table(toml::Table::new()));
    if !krl.is_table() {
        *krl = Value::Table(toml::Table::new());
    }
    let krl = krl.as_table_mut().expect("a table");
    {
        let sim = krl.entry("sim").or_insert_with(|| Value::Table(toml::Table::new()));
        if let Some(sim) = sim.as_table_mut() {
            sim.insert("in_base".into(), Value::Integer(definition.telegram.in_base.into()));
            sim.insert("out_base".into(), Value::Integer(definition.telegram.out_base.into()));
        }
    }
    if let Some((name, p)) = definition.processes.iter().find(|(_, p)| !p.weld_call.trim().is_empty()) {
        if definition.profile == "vw-ks24" {
            let vw = krl.entry("vw_ks24").or_insert_with(|| Value::Table(toml::Table::new()));
            if let Some(vw) = vw.as_table_mut() {
                vw.insert("weld_call".into(), Value::String(p.weld_call.clone()));
                vw.insert("weld_op_type".into(), Value::String(p.op_type.clone()));
            }
        } else {
            notes.push(format!("processes.{name}.weld_call: only the vw-ks24 profile emits a weld call (profile {})", definition.profile));
        }
    }
    let robots = krl.entry("robot").or_insert_with(|| Value::Array(Vec::new()));
    if let Some(robots) = robots.as_array_mut() {
        for robot in &definition.robots {
            if robot.tool_no.is_none() && robot.base_no.is_none() {
                continue;
            }
            let entry = robots.iter_mut().filter_map(Value::as_table_mut).find(|t| t.get("ns").and_then(Value::as_str) == Some(robot.ns.as_str()));
            match entry {
                Some(t) => {
                    if let Some(tool) = robot.tool_no {
                        t.insert("tool_no".into(), Value::Integer(tool.into()));
                    }
                    if let Some(base) = robot.base_no {
                        t.insert("base_no".into(), Value::Integer(base.into()));
                    }
                }
                None => notes.push(format!(
                    "{}: plan2cell.toml has no [[krl.robot]] ns = \"{}\" (it needs its e_axes and model first: Generate Hardware Config); tool_no / base_no not applied",
                    robot.ns, robot.ns
                )),
            }
        }
    }
    notes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn merges_into_plan2cell_toml() {
        let mut doc: toml::Table = toml::from_str("[[krl.robot]]\nns = \"robot1\"\ne_axes = \"none\"\n\n[plc]\nfoo = 1\n").unwrap();
        let mut def: ProcessDefinition = serde_yaml::from_str(&sample_yaml("vw-ks24", &[])).unwrap();
        def.telegram.in_base = 512;
        def.robots[0].tool_no = Some(3);
        let notes = merge(&mut doc, &def);
        let text = toml::to_string_pretty(&doc).unwrap();
        assert!(text.contains("in_base = 512"), "{text}");
        assert!(text.contains("tool_no = 3"), "{text}");
        assert!(text.contains("weld_call = \"SEL_RES=SELECT(#MAKRO,580,TRUE,,,,,FALSE)\""), "{text}");
        assert!(text.contains("foo = 1"), "the rest stays: {text}");
        assert_eq!(notes.len(), 1, "robot2 has no [[krl.robot]]: {notes:?}");
    }

    #[test]
    fn apply_writes_the_files() {
        let root = std::env::temp_dir().join(format!("factoryos-process-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        std::fs::create_dir_all(root.join("plan2cell")).unwrap();
        std::fs::write(root.join("plan2cell/plan2cell.toml"), "[[krl.robot]]\nns = \"robot1\"\ne_axes = \"none\"\n").unwrap();
        let yaml = sample_yaml("sim", &[]);
        let def: ProcessDefinition = serde_yaml::from_str(&yaml).unwrap();
        let written = apply(&root, &def, &yaml).unwrap();
        assert!(written.iter().any(|w| w.contains("robot1_process_signals.dat")), "{written:?}");
        assert!(root.join("plan2cell/plan2cell.toml.bak").is_file());
        assert!(std::fs::read_to_string(root.join("plan2cell/krl/robot2_process_signals.dat")).unwrap().contains("SIGNAL GUN_CLOSE $OUT[321]"));
        std::fs::remove_dir_all(&root).ok();
    }
}
