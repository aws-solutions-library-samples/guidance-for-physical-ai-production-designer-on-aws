//! The agent's routes: `POST /api/v1/reach/check` (a scene's reachability, synchronous) and
//! `POST /api/v1/reach/suggest` (robot moves, optionally written as a new scene; never overwrites).

use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicBool;

use axum::extract::State;
use axum::http::StatusCode;
use axum::routing::post;
use axum::{Json, Router};
use factoryos_protocol::api;
use factoryos_protocol::fitter::{ReachFileRequest, ReachReport, SceneFile, SuggestRequest, SuggestResponse, TaskSource};

use super::{CheckOptions, analyse, check_scene, suggest};
use crate::AppState;
use crate::config_store::ConfigStore;
use crate::services::scene_store::{self, SceneRef};
use crate::services::{cell_resolve, cells, scene_paths};

pub fn router() -> Router<AppState> {
    Router::new().route(api::REACH_CHECK, post(check)).route(api::REACH_SUGGEST, post(suggest_layout))
}

type Reply<T> = Result<Json<T>, (StatusCode, String)>;

fn bad(status: StatusCode, message: impl Into<String>) -> (StatusCode, String) {
    (status, message.into())
}

/// A scene to check, with where its paths start and the cell it belongs to.
#[derive(Debug)]
pub struct Resolved {
    pub scene: SceneFile,
    pub scene_base: PathBuf,
    pub cell_root: Option<PathBuf>,
    /// Its name in `scenes/`, if it is a Robo Fitter scene.
    pub file_name: Option<String>,
    /// The cell whose own `scene.yaml` it is.
    pub cell: Option<String>,
}

/// The scene YAML `path`, its relative paths from its folder (with the legacy fallback,
/// [`scene_store::read`]).
fn read_scene(path: &Path) -> Result<SceneFile, (StatusCode, String)> {
    if !path.is_file() {
        return Err(bad(StatusCode::NOT_FOUND, format!("{}: no such scene file", path.display())));
    }
    scene_store::read(path).map_err(|err| bad(StatusCode::BAD_REQUEST, format!("{err:#}")))
}

/// The scene of a request: `scene_file` (a scene name, a legacy `<name>.yaml`, or an absolute path
/// to a scene YAML: a cell's `scene.yaml` counts as that cell's), `cell` (its scene, or with
/// another scene: the cell it is linked to) or an inline `scene` (its paths as a scene folder's,
/// `../../urdf-dressed-robots/…`). A scene the active cell was made from is linked to it, as the
/// Robo Fitter links it on load.
pub fn resolve(
    config: &ConfigStore,
    scenes_dir: &Path,
    scene_file: Option<&str>,
    cell: Option<&str>,
    scene: Option<&SceneFile>,
) -> Result<Resolved, (StatusCode, String)> {
    let cell_folder = |name: &str| -> Result<PathBuf, (StatusCode, String)> {
        let ok = !name.is_empty() && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-');
        let root = cells::cell_folder(config, name).filter(|_| ok).ok_or_else(|| bad(StatusCode::NOT_FOUND, format!("no cell {name}")))?;
        if !root.is_dir() {
            return Err(bad(StatusCode::NOT_FOUND, format!("no cell {name} ({})", root.display())));
        }
        Ok(root)
    };
    let linked = cell.filter(|c| !c.trim().is_empty()).map(|c| cell_folder(c.trim())).transpose()?;
    if let Some(scene) = scene {
        let base = scene_store::unsaved_base(scenes_dir);
        let mut scene = scene.clone();
        scene_store::fix_legacy_paths(&mut scene, &base);
        return Ok(Resolved {
            scene,
            scene_base: base,
            cell_root: linked,
            file_name: None,
            cell: None,
        });
    }
    match scene_file.map(str::trim).filter(|f| !f.is_empty()) {
        Some(file) => {
            let reference = scene_store::parse_ref(scenes_dir, file);
            if let Some(SceneRef::Named(name)) = &reference {
                let name = name.clone();
                let mut path = scene_store::scene_file(scenes_dir, &name);
                if !path.is_file() {
                    // A flat file of the old layout not migrated yet (it is at the next start).
                    if let Some(flat) = ["yaml", "yml"].iter().map(|e| scenes_dir.join(format!("{name}.{e}"))).find(|p| p.is_file()) {
                        path = flat;
                    } else {
                        return Err(bad(StatusCode::NOT_FOUND, format!("no scene {name} (scenes/{name}/scene.yaml)")));
                    }
                }
                let scene = read_scene(&path)?;
                let base = path.parent().unwrap_or(scenes_dir).to_owned();
                // The scene's cell: the active cell if it is the scene's, else the scene's own (a
                // reference cell, or a copy made from it), if it has one.
                let cell_root = linked.or_else(|| {
                    cells::active_cell(config)
                        .filter(|(_, root)| cells::fitted_source(root).as_deref() == Some(name.as_str()))
                        .map(|(_, root)| root)
                        .or_else(|| {
                            let cell = cell_resolve::cell_for_scene(config, &name).ok()?;
                            let root = cells::cell_folder(config, &cell)?;
                            (cells::fitted_source(&root).as_deref() == Some(name.as_str())).then_some(root)
                        })
                });
                return Ok(Resolved { scene, scene_base: base, cell_root, file_name: Some(name), cell: None });
            }
            let Some(SceneRef::Path(path)) = reference else {
                return Err(bad(StatusCode::BAD_REQUEST, format!("{file}: not a scene (a scene name in scenes/, or an absolute path to a scene YAML)")));
            };
            let path = path.as_path();
            let dir = path.parent().unwrap_or(Path::new("/")).to_owned();
            let scene = read_scene(path)?;
            // A cell's own scene.yaml.
            let own_cell = (path.file_name().is_some_and(|n| n == "scene.yaml") && dir.join("agentic_planner").is_dir())
                .then(|| dir.file_name().unwrap_or_default().to_string_lossy().into_owned());
            let cell_root = linked.or_else(|| own_cell.as_ref().map(|_| dir.clone()));
            Ok(Resolved { scene, scene_base: dir, cell_root, file_name: None, cell: own_cell })
        }
        None => {
            let name = cell.map(str::trim).filter(|c| !c.is_empty()).ok_or_else(|| bad(StatusCode::BAD_REQUEST, "give scene_file, cell or scene"))?;
            let root = linked.expect("resolved above");
            // A legacy cell's own scene.yaml, or the Robo Fitter scene a reference cell points at
            // (its paths from `scenes/`).
            let resolved = cell_resolve::resolve_root(&root, scenes_dir)
                .map_err(|err| bad(StatusCode::NOT_FOUND, format!("cell {name}: {err:#}")))?;
            let mut scene = read_scene(&resolved.scene_path)?;
            if scene.name.is_empty() {
                scene.name = name.to_owned();
            }
            let file_name = (resolved.kind == cell_resolve::CellKind::Reference).then(|| resolved.fitted_scene.clone()).flatten();
            Ok(Resolved { scene, scene_base: resolved.scene_base, cell_root: Some(root), file_name, cell: Some(name.to_owned()) })
        }
    }
}

async fn check(State(state): State<AppState>, Json(request): Json<ReachFileRequest>) -> Reply<ReachReport> {
    let (config, fitter) = (state.config.clone(), state.fitter.clone());
    tokio::task::spawn_blocking(move || {
        let scenes_dir = fitter.scenes_dir();
        let resolved = resolve(&config, &scenes_dir, request.scene_file.as_deref(), request.cell.as_deref(), request.scene.as_ref())?;
        let options = CheckOptions { source: request.source, mode: request.mode, ..Default::default() };
        let ctx = fitter.reach_context();
        let mut report = check_scene(&ctx, &resolved.scene, &resolved.scene_base, resolved.cell_root.as_deref(), &options, |_| {}, &AtomicBool::new(false));
        if request.show_in_fitter
            && let Some(note) = fitter.show_report(resolved.file_name.as_deref(), resolved.cell.as_deref(), report.clone())
        {
            report.notes.push(note);
        }
        Ok(Json(report))
    })
    .await
    .map_err(|err| bad(StatusCode::INTERNAL_SERVER_ERROR, format!("reach check panicked: {err}")))?
}

async fn suggest_layout(State(state): State<AppState>, Json(request): Json<SuggestRequest>) -> Reply<SuggestResponse> {
    let (config, fitter) = (state.config.clone(), state.fitter.clone());
    tokio::task::spawn_blocking(move || {
        let scenes_dir = fitter.scenes_dir();
        let resolved = resolve(&config, &scenes_dir, request.scene_file.as_deref(), request.cell.as_deref(), None)?;
        let write_as = request.write_as.as_deref().map(str::trim).filter(|w| !w.is_empty()).map(new_scene_name).transpose()?;
        if let Some(name) = &write_as
            && scene_store::scene_dir(&scenes_dir, name).exists()
        {
            return Err(bad(StatusCode::CONFLICT, format!("scene {name} exists: scenes are never overwritten, pick another name")));
        }
        let max_move_m = if request.max_move_m.is_finite() && request.max_move_m > 0.0 { request.max_move_m.min(5.0) } else { 1.0 };
        let ctx = fitter.reach_context();
        let options = CheckOptions { source: TaskSource::Auto, estimate_only: true, ..Default::default() };
        let base = &resolved.scene_base;
        let cell_root = resolved.cell_root.as_deref();
        let before = check_scene(&ctx, &resolved.scene, base, cell_root, &options, |_| {}, &AtomicBool::new(false));
        let analysis = analyse(&ctx, &resolved.scene, base, cell_root, before.source);
        let moves = suggest::suggest(&analysis.robots, &before, &analysis.boxes(), max_move_m);
        let mut scene = suggest::apply(&resolved.scene, &moves);
        let mut report = check_scene(&ctx, &scene, base, cell_root, &CheckOptions { source: before.source, ..options }, |_| {}, &AtomicBool::new(false));
        let mut written = None;
        if let Some(name) = write_as.as_deref() {
            let (out, warnings) = write_new_scene(&config, &scenes_dir, base, &scene, name)?;
            report.notes.extend(warnings);
            fitter.refresh_palette();
            written = Some(name.to_owned());
            scene = out;
        }
        Ok(Json(SuggestResponse { scene, moves: moves.into_iter().map(|m| m.suggestion).collect(), report, written }))
    })
    .await
    .map_err(|err| bad(StatusCode::INTERNAL_SERVER_ERROR, format!("suggest panicked: {err}")))?
}

/// The scene name of a `write_as` (a name, or a legacy `<name>.yaml`).
pub fn new_scene_name(write_as: &str) -> Result<String, (StatusCode, String)> {
    scene_store::name_of(write_as).ok_or_else(|| {
        bad(StatusCode::BAD_REQUEST, format!("write_as {write_as}: a new scene name (letters, digits, _, - and .; it becomes scenes/<name>/)"))
    })
}

/// Write `scene` (its paths from `base`) as the new scene `name` (`scenes/<name>/scene.yaml`;
/// a legacy `<name>.yaml` is accepted), never overwriting; its paths reference the original
/// files, never a cell's copies, relative to its folder. Returns the scene as written and the
/// path rewrite's warnings.
pub fn write_new_scene(
    config: &ConfigStore,
    scenes_dir: &Path,
    base: &Path,
    scene: &SceneFile,
    name: &str,
) -> Result<(SceneFile, Vec<String>), (StatusCode, String)> {
    let name = new_scene_name(name)?;
    let name = name.as_str();
    let folder = scene_store::scene_dir(scenes_dir, name);
    let mut out = scene.clone();
    out.name = name.to_owned();
    // The new scene references the original files, never a cell's copies (a cell's own
    // scene's paths); its other relative paths are made absolute when it moves.
    let originals = scene_paths::Originals {
        catalogue: scenes_dir.parent().map(Path::to_path_buf).unwrap_or_else(|| scenes_dir.to_owned()),
        products: crate::services::product::products_dir(config),
        cells: cells::cells_folder(config),
    };
    let rewrite = scene_paths::to_originals(&mut out, base, &folder, &originals);
    let warnings = rewrite.warnings.into_iter().map(|w| format!("{name}: {w}")).collect();
    let text = serde_yaml::to_string(&out).map_err(|err| bad(StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    // create_dir (not _all): never into a folder that is there, even if it appeared meanwhile.
    std::fs::create_dir_all(scenes_dir).ok();
    std::fs::create_dir(&folder).map_err(|err| bad(StatusCode::CONFLICT, format!("scene {name}: {err}")))?;
    let path = scene_store::scene_file(scenes_dir, name);
    std::fs::write(&path, text.as_bytes()).map_err(|err| bad(StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    crate::storage::saved(&path);
    Ok((out, warnings))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scenes_are_found_by_name_path_or_cell() {
        let root = std::env::temp_dir().join(format!("factoryos-reach-http-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let scenes = root.join("catalogue/scenes");
        let cell = root.join("cells/jam2");
        std::fs::create_dir_all(&scenes).unwrap();
        std::fs::create_dir_all(cell.join("agentic_planner")).unwrap();
        let write_scene = |name: &str, text: &str| {
            std::fs::create_dir_all(scenes.join(name)).unwrap();
            std::fs::write(scenes.join(name).join("scene.yaml"), text).unwrap();
        };
        write_scene("jam2", "name: jam2\n");
        std::fs::write(cell.join("scene.yaml"), "robots: []\n").unwrap();
        // A legacy manifest: `jam2.yaml` is scene jam2.
        std::fs::write(cell.join("factoryos-cell.json"), r#"{"fitted_scene":"jam2.yaml"}"#).unwrap();
        let file = root.join("factoryos.config.json");
        std::fs::write(&file, format!(r#"{{"cellsDir":"{}","cellDir":"{}"}}"#, root.join("cells").display(), cell.display())).unwrap();
        let config = ConfigStore::open(file);

        let r = resolve(&config, &scenes, Some("jam2"), None, None).unwrap();
        assert_eq!((r.file_name.as_deref(), r.cell_root.as_deref()), (Some("jam2"), Some(cell.as_path())), "linked to the active cell made from it");
        assert_eq!(r.scene_base, scenes.join("jam2"), "its paths start at its folder");
        assert_eq!(resolve(&config, &scenes, Some("jam2.yaml"), None, None).unwrap().file_name.as_deref(), Some("jam2"), "the legacy file name");
        let abs = scenes.join("jam2/scene.yaml").display().to_string();
        assert_eq!(resolve(&config, &scenes, Some(&abs), None, None).unwrap().file_name.as_deref(), Some("jam2"));
        assert_eq!(resolve(&config, &scenes, Some("missing"), None, None).unwrap_err().0, StatusCode::NOT_FOUND);
        let own = resolve(&config, &scenes, Some(&cell.join("scene.yaml").display().to_string()), None, None).unwrap();
        assert_eq!((own.cell.as_deref(), own.scene_base.as_path()), (Some("jam2"), cell.as_path()));
        let by_cell = resolve(&config, &scenes, None, Some("jam2"), None).unwrap();
        assert_eq!(by_cell.scene.name, "jam2");
        assert_eq!(resolve(&config, &scenes, Some("../x.yaml"), None, None).unwrap_err().0, StatusCode::BAD_REQUEST);
        assert_eq!(resolve(&config, &scenes, None, Some("../etc"), None).unwrap_err().0, StatusCode::NOT_FOUND);
        assert_eq!(resolve(&config, &scenes, Some("/etc/passwd"), None, None).unwrap_err().0, StatusCode::BAD_REQUEST);

        // A reference cell (not the active one): its scene is the Robo Fitter's, its paths from
        // scenes/; the scene file is linked to it.
        let reference = root.join("cells/jam3");
        std::fs::create_dir_all(&reference).unwrap();
        write_scene("jam3", "name: jam3\nrobots: []\n");
        std::fs::write(reference.join("factoryos-cell.json"), r#"{"format":2,"kind":"reference","fitted_scene":"jam3"}"#).unwrap();
        let by_cell = resolve(&config, &scenes, None, Some("jam3"), None).unwrap();
        assert_eq!((by_cell.scene.name.as_str(), by_cell.scene_base.as_path()), ("jam3", scenes.join("jam3").as_path()));
        assert_eq!((by_cell.cell_root.as_deref(), by_cell.file_name.as_deref()), (Some(reference.as_path()), Some("jam3")));
        let by_file = resolve(&config, &scenes, Some("jam3"), None, None).unwrap();
        assert_eq!(by_file.cell_root.as_deref(), Some(reference.as_path()), "the scene's own cell");
        write_scene("lonely", "name: l\nrobots: []\n");
        assert_eq!(resolve(&config, &scenes, Some("lonely"), None, None).unwrap().cell_root, None, "no cell yet");
        // A flat file not migrated yet still reads (its paths from scenes/).
        std::fs::write(scenes.join("flat.yaml"), "name: flat\nrobots: []\n").unwrap();
        assert_eq!(resolve(&config, &scenes, Some("flat"), None, None).unwrap().scene_base, scenes);
        std::fs::remove_dir_all(&root).ok();
    }

    /// The user's scene-fit.yaml: written from cell jam2's own scene, it references the
    /// catalogue's robot and the product, not the cell's copies.
    #[test]
    fn scenes_written_from_a_cell_reference_the_originals() {
        let (root, o) = scene_paths::tests::host("write-as");
        let cell = o.cells.as_ref().unwrap().join("jam2");
        let mut scene: SceneFile = serde_yaml::from_str(
            "robots:\n- robot_id: r\n  dressed_urdf: robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf\n  position: {x: 0, y: 0, z: 0}\n\
             fixtures:\n- id: bug_cube\n  urdf: fixtures/bug_cube/bug_cube.urdf\n  position: {x: 0, y: 0, z: 0}\n",
        )
        .unwrap();
        scene_paths::to_originals(&mut scene, &cell, &o.scenes_dir().join("scene-fit"), &o);
        assert_eq!(scene.robots[0].dressed_urdf, "../../urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf");
        assert_eq!(scene.fixtures[0].urdf, o.products.join("bug_cube/bug_cube.urdf").display().to_string());
        std::fs::remove_dir_all(&root).ok();
    }
}
