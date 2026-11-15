//! Robo Fitter scenes edited one item at a time (`/api/v1/scene/*`) and robots placed
//! automatically (`/api/v1/reach/place`), for the chat agent's MCP tools
//! ([`factoryos_protocol::scene_edit`]).
//!
//! * A product goes in at a height, or at the height the robots reach its welds best, on a support
//!   (a box fixture written into the catalogue's `urdf-fixture-collection/support_*`).
//! * Robots are added, moved, re-dressed, put on pedestals, removed.
//! * `reach/place` puts 1..N robots around the products: candidate bases on rings around the
//!   welds, scored by the reach domes (the Robo Fitter's estimate), chosen greedily (a set cover:
//!   most uncovered welds first, bases kept apart and clear of the products), smallest N first.
//!
//! Every edit saves the scene as the Robo Fitter's Save does: the cells made from it are rebuilt.

use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicBool;

use axum::extract::{Path as UrlPath, Query, State};
use axum::http::StatusCode;
use axum::routing::{get, post};
use axum::{Json, Router};
use factoryos_protocol::api;
use factoryos_protocol::fitter::{Abc, FixtureYaml, PedestalYaml, RobotYaml, SceneEdit, SceneFile, TaskSource, TaskWeld, Xyz};
use factoryos_protocol::product::Target;
use factoryos_protocol::scene_edit::{
    AddRobot, ComfortSpec, CreateScene, Layout, LayoutQuality, LayoutQualityRequest, MoveRobot, PlaceProduct, PlaceRobots, PlaceRobotsResult,
    RemoveItems, RobotFit, SceneEdited, WeldFit,
};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};

use super::scene_store;
use super::reach::suggest::TARGET_MARGIN_M;
use super::reach::{CheckOptions, ReachContext, analyse, check_scene, comfort, product_collision, robot_info};
use crate::AppState;

pub fn router() -> Router<AppState> {
    Router::new()
        .route(api::SCENE_CREATE, post(create))
        .route(api::SCENE_GET, get(read))
        .route(api::SCENE_CURRENT, get(current))
        .route(api::SCENE_PLACE_PRODUCT, post(place_product))
        .route(api::SCENE_ROBOT_ADD, post(add_robot))
        .route(api::SCENE_ROBOT_MOVE, post(move_robot))
        .route(api::SCENE_ROBOT_REMOVE, post(remove_robots))
        .route(api::SCENE_FIXTURE_REMOVE, post(remove_fixtures))
        .route(api::REACH_PLACE, post(place_robots))
        .route(api::REACH_QUALITY, post(layout_quality))
}

type HttpError = (StatusCode, String);
type Reply<T> = Result<Json<T>, HttpError>;

fn bad(message: impl Into<String>) -> HttpError {
    (StatusCode::BAD_REQUEST, message.into())
}

const DEFAULT_PEDESTAL_DIAMETER_MM: f64 = 800.0;
/// The support is this much larger than the product's footprint on each side, metres.
const SUPPORT_MARGIN_M: f64 = 0.05;
/// Below this a product stands on the floor (no support).
const MIN_SUPPORT_M: f64 = 0.02;

/// The scene name `file` gives (a name, or a legacy `<name>.yaml`).
fn scene_name(file: &str) -> Result<String, HttpError> {
    scene_store::name_of(file).ok_or_else(|| bad(format!("{file}: not a scene name (letters, digits, _, - and .; the scene is scenes/<name>/)")))
}

/// Scene `file` (a name, or a legacy `<name>.yaml`) and its name.
fn load(scenes_dir: &Path, file: &str) -> Result<(SceneFile, String), HttpError> {
    let name = scene_name(file)?;
    let path = scene_store::scene_file(scenes_dir, &name);
    if !path.is_file() {
        return Err((StatusCode::NOT_FOUND, format!("no scene {name} ({})", path.display())));
    }
    let scene = scene_store::read(&path).map_err(|err| bad(format!("{err:#}")))?;
    Ok((scene, name))
}

fn empty_scene(name: &str, description: &str) -> SceneFile {
    SceneFile {
        name: name.to_owned(),
        description: description.to_owned(),
        scene_origin: None,
        robots: Vec::new(),
        fixtures: Vec::new(),
        floor_plan: None,
        extra: Default::default(),
    }
}

/// An id not used by any robot or fixture of `scene`.
fn unique_id(scene: &SceneFile, base: &str) -> String {
    let taken = |id: &str| scene.robots.iter().any(|r| r.robot_id == id) || scene.fixtures.iter().any(|f| f.id == id);
    if !taken(base) {
        return base.to_owned();
    }
    (2..).map(|i| format!("{base}_{i}")).find(|id| !taken(id)).expect("unbounded")
}

/// Load `file`, apply `change`, save as the Robo Fitter does, and rebuild its cells.
async fn edit(
    state: AppState,
    file: String,
    change: impl FnOnce(&mut SceneFile, &Ctx) -> Result<(Vec<String>, Vec<String>), String> + Send + 'static,
) -> Reply<SceneEdited> {
    let (fitter, config, product, agentic) = (state.fitter.clone(), state.config.clone(), state.product.clone(), state.agentic.clone());
    let edited = tokio::task::spawn_blocking(move || -> Result<SceneEdited, HttpError> {
        let scenes_dir = fitter.scenes_dir();
        let (mut scene, file) = load(&scenes_dir, &file)?;
        let scene_base = scene_store::scene_dir(&scenes_dir, &file);
        let ctx = Ctx { scenes_dir: scenes_dir.clone(), scene_base, products_dir: super::product::products_dir(&config), reach: fitter.reach_context(), product };
        let (changes, notes) = change(&mut scene, &ctx).map_err(bad)?;
        // An edit never renames the scene (saving names the folder after `name`).
        scene.name = file.clone();
        let cells = fitter.save(SceneEdit { file_name: file.clone(), scene: scene.clone() });
        Ok(SceneEdited { file_name: file, scene, changes, cells, notes })
    })
    .await
    .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, format!("scene edit panicked: {err}")))??;
    agentic.prepare_cells(&edited.cells);
    Ok(Json(edited))
}

/// What the edits need besides the scene.
struct Ctx {
    scenes_dir: PathBuf,
    /// The scene's folder (`scenes/<name>/`): where its relative paths start.
    scene_base: PathBuf,
    products_dir: PathBuf,
    reach: std::sync::Arc<ReachContext>,
    product: std::sync::Arc<super::product::ProductService>,
}

impl Ctx {
    fn catalogue(&self) -> PathBuf {
        self.scenes_dir.parent().map(Path::to_path_buf).unwrap_or_else(|| self.scenes_dir.clone())
    }

    /// A dressed robot's path as a scene references it, checked to exist.
    fn dressed(&self, name: &str) -> Result<String, String> {
        let ok = !name.is_empty() && !name.contains("..") && name.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.'));
        let file = self.catalogue().join("urdf-dressed-robots").join(name).join(format!("{name}.urdf"));
        if !ok || !file.is_file() {
            return Err(format!("no dressed robot {name} (urdf-dressed-robots/{name}/{name}.urdf): dress it first (dresser_dress)"));
        }
        // Relative to the scene's folder `scenes/<name>/`.
        Ok(format!("../../urdf-dressed-robots/{name}/{name}.urdf"))
    }
}

// --- routes ---

async fn create(State(state): State<AppState>, Json(request): Json<CreateScene>) -> Reply<SceneEdited> {
    let fitter = state.fitter.clone();
    tokio::task::spawn_blocking(move || {
        let scenes_dir = fitter.scenes_dir();
        let name = scene_name(&request.file_name)?;
        let scene = empty_scene(&name, &request.description);
        let text = serde_yaml::to_string(&scene).map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
        std::fs::create_dir_all(&scenes_dir).ok();
        // create_dir (not _all): never into a scene that is there, even if it appeared meanwhile.
        std::fs::create_dir(scene_store::scene_dir(&scenes_dir, &name))
            .map_err(|err| (StatusCode::CONFLICT, format!("scene {name}: {err} (scenes are never overwritten; pick another name)")))?;
        let path = scene_store::scene_file(&scenes_dir, &name);
        std::fs::write(&path, text.as_bytes()).map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
        crate::storage::saved(&path);
        fitter.refresh_palette();
        Ok(Json(SceneEdited { file_name: name, scene, changes: vec!["created".to_owned()], ..SceneEdited::default() }))
    })
    .await
    .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?
}

#[derive(serde::Deserialize)]
struct FileQuery {
    file: String,
}

async fn read(State(state): State<AppState>, Query(query): Query<FileQuery>) -> Reply<SceneFile> {
    load(&state.fitter.scenes_dir(), &query.file).map(|(scene, _)| Json(scene))
}

/// The Robo Fitter's scene in brief (its state minus the palette, meshes and per-weld detail).
async fn current(State(state): State<AppState>) -> Json<serde_json::Value> {
    Json(summary(&state.fitter.state()))
}

fn summary(s: &factoryos_protocol::fitter::FitterState) -> serde_json::Value {
    use serde_json::json;
    let count = |statuses: Vec<String>| {
        let mut m = std::collections::BTreeMap::<String, usize>::new();
        for st in statuses {
            *m.entry(st).or_default() += 1;
        }
        m
    };
    let status = |st: &factoryos_protocol::fitter::WeldStatus| serde_json::to_value(st).ok().and_then(|v| v.as_str().map(str::to_owned)).unwrap_or_default();
    let robots: Vec<_> = s
        .scene
        .robots
        .iter()
        .map(|r| {
            json!({
                "robot_id": r.robot_id,
                "dressed": Path::new(&r.dressed_urdf).file_stem().and_then(|n| n.to_str()),
                "position": [r.position.x, r.position.y, r.position.z],
                "yaw_deg": r.orientation.a,
                "pedestal_mm": r.pedestal.as_ref().map(|p| p.height_mm),
            })
        })
        .collect();
    let fixtures: Vec<_> = s
        .scene
        .fixtures
        .iter()
        .map(|f| json!({"id": f.id, "urdf": Path::new(&f.urdf).file_stem().and_then(|n| n.to_str()), "position": [f.position.x, f.position.y, f.position.z], "yaw_deg": f.orientation.a}))
        .collect();
    let read_only = s.linked_cell.is_some();
    let reach = s.reach.as_ref().map(|r| {
        json!({
            "method": r.method, "mode": r.mode, "running": r.running, "done": r.done, "total": r.total,
            "stale": r.scene_rev != s.scene_rev,
            "status": count(r.welds.iter().map(|w| status(&w.status)).collect()),
            "unreached": r.welds.iter().filter(|w| w.reached_by.is_empty()).map(|w| w.weld.label.clone()).collect::<Vec<_>>(),
        })
    });
    json!({
        "file_name": s.scene_file_name,
        "name": s.scene.name,
        "linked_cell": s.linked_cell,
        "cell_scene": s.cell_scene,
        "read_only": read_only,
        "editing": if read_only {
            "the scene is linked to its cell (simulated, read-only): change a copy (write_as a new scene name), or Stop simulation and edit (fitter/edit)"
        } else {
            "editable: the scene_* tools change this file"
        },
        "robots": robots,
        "fixtures": fixtures,
        "welds": s.task.len(),
        "estimate": count(s.estimate.iter().map(|e| status(&e.status)).collect()),
        "task_notes": s.task_notes,
        "reach": reach,
        "status": s.status,
    })
}

async fn place_product(State(state): State<AppState>, UrlPath(file): UrlPath<String>, Json(request): Json<PlaceProduct>) -> Reply<SceneEdited> {
    edit(state, file, move |scene, ctx| place_product_in(scene, ctx, &request)).await
}

fn place_product_in(scene: &mut SceneFile, ctx: &Ctx, request: &PlaceProduct) -> Result<(Vec<String>, Vec<String>), String> {
    let name = request.product.trim();
    let product = ctx.product.load(name).map_err(|err| format!("{err:#}"))?.product;
    // The product's fixture URDF (written by Save in Product & Task).
    let urdf = ctx.products_dir.join(name).join(format!("{name}.urdf"));
    if !urdf.is_file() {
        ctx.product.save(product.clone()).map_err(|err| format!("{err:#}"))?;
    }
    let mut notes = Vec::new();
    let [x, y] = request.position.unwrap_or([0.0, 0.0]);
    let id = match (&request.id, request.replace) {
        (Some(id), true) => id.clone(),
        (Some(id), false) => unique_id(scene, id),
        (None, _) => unique_id(scene, name),
    };
    let mut fixture = FixtureYaml {
        id: id.clone(),
        urdf: urdf.to_string_lossy().into_owned(),
        position: Xyz { x, y, z: 0.0 },
        orientation: Abc { a: request.yaw_deg, b: 0.0, c: 0.0 },
        ..FixtureYaml::default()
    };
    let height = match request.height_m.filter(|h| *h >= 0.0) {
        Some(h) => h,
        None => {
            let (h, note) = best_height(scene, ctx, &product, &fixture, request)?;
            notes.push(note);
            h
        }
    };
    fixture.position.z = height;
    let mut changes = vec![format!("{id}: product {name} at ({x:.3}, {y:.3}), {height:.3} m high, yaw {:.0}°", request.yaw_deg)];
    let support_id = format!("{id}_support");
    scene.fixtures.retain(|f| f.id != support_id && !(request.replace && f.id == id));
    if height >= MIN_SUPPORT_M && request.support.unwrap_or(true) {
        let surface = super::product_welds::Surface::of(&ctx.product, &product).map_err(|err| format!("{err:#}"))?;
        let (lo, hi) = surface.bounds().ok_or("the product has no geometry to put on a support")?;
        let size = [(hi[0] - lo[0]) as f64 + 2.0 * SUPPORT_MARGIN_M, (hi[1] - lo[1]) as f64 + 2.0 * SUPPORT_MARGIN_M, height];
        // The support's centre under the footprint's centre, turned with the product.
        let (cx, cy) = (((lo[0] + hi[0]) / 2.0) as f64, ((lo[1] + hi[1]) / 2.0) as f64);
        let (s, c) = request.yaw_deg.to_radians().sin_cos();
        let support_urdf = write_support(&ctx.catalogue(), size)?;
        scene.fixtures.push(FixtureYaml {
            id: support_id.clone(),
            urdf: support_urdf,
            position: Xyz { x: x + c * cx - s * cy, y: y + s * cx + c * cy, z: 0.0 },
            orientation: Abc { a: request.yaw_deg, b: 0.0, c: 0.0 },
            ..FixtureYaml::default()
        });
        changes.push(format!("{support_id}: support {:.2} × {:.2} × {:.2} m", size[0], size[1], size[2]));
    }
    scene.fixtures.push(fixture);
    Ok((changes, notes))
}

/// A box support of `size` (metres) standing on its origin, in the catalogue's fixtures (written
/// once per size); its path as a scene references it (from its folder `scenes/<name>/`).
fn write_support(catalogue: &Path, size: [f64; 3]) -> Result<String, String> {
    let mm = size.map(|v| (v * 1000.0).round() as i64);
    let name = format!("support_{}x{}x{}mm", mm[0], mm[1], mm[2]);
    let dir = catalogue.join("urdf-fixture-collection").join(&name);
    let urdf = dir.join(format!("{name}.urdf"));
    if !urdf.is_file() {
        std::fs::create_dir_all(dir.join("meshes")).map_err(|err| format!("{}: {err}", dir.display()))?;
        std::fs::write(dir.join("meshes").join(format!("{name}.obj")), super::product::box_obj(size.map(|v| v as f32)))
            .map_err(|err| format!("{}: {err}", dir.display()))?;
        let geometry = format!(r#"<geometry><mesh filename="meshes/{name}.obj" scale="1 1 1"/></geometry>"#);
        let text = format!(
            r#"<?xml version="1.0"?>
<!-- Written by Factory OS (scene_place_product): a support of {} x {} x {} mm under a product. -->
<robot name="{name}">
  <link name="World">
    <visual><origin xyz="0 0 0" rpy="0 0 0"/>{geometry}<material name="support"><color rgba="0.35 0.37 0.40 1.0"/></material></visual>
    <collision><origin xyz="0 0 0" rpy="0 0 0"/>{geometry}</collision>
  </link>
</robot>
"#,
            mm[0], mm[1], mm[2]
        );
        std::fs::write(&urdf, text).map_err(|err| format!("{}: {err}", urdf.display()))?;
    }
    Ok(format!("../../urdf-fixture-collection/{name}/{name}.urdf"))
}

/// Candidate heights for the automatic placement, metres.
fn heights() -> impl Iterator<Item = f64> {
    (0..=36).map(|i| i as f64 * 0.05)
}

/// The height (of the product's bottom) at which the scene's robots reach its welds best, or,
/// without robots, at which `request.robot`'s wrist reaches farthest.
fn best_height(scene: &SceneFile, ctx: &Ctx, product: &factoryos_protocol::product::Product, fixture: &FixtureYaml, request: &PlaceProduct) -> Result<(f64, String), String> {
    let points: Vec<([f32; 3], [f32; 3])> = product
        .tasks
        .iter()
        .flat_map(|t| &t.operations)
        .filter_map(|op| match &op.target {
            Target::Point { position, normal } => Some((*position, *normal)),
            Target::Path { points, normal } => points.first().map(|p| (*p, *normal)),
        })
        .collect();
    if points.is_empty() {
        return Ok((0.0, format!("{} has no welds yet: it stands on the floor (give height_m, or add welds first)", product.name)));
    }
    if !scene.robots.is_empty() {
        let mut best = (0.0, -1, f64::MIN);
        for h in heights() {
            let mut trial = scene.clone();
            trial.fixtures.retain(|f| f.id != fixture.id);
            let mut f = fixture.clone();
            f.position.z = h;
            trial.fixtures.push(f);
            let analysis = analyse(&ctx.reach, &trial, &ctx.scene_base, None, TaskSource::Product);
            let boxes = analysis.boxes();
            let welds: Vec<&TaskWeld> = analysis.welds.iter().filter(|w| w.fixture_id.as_deref() == Some(fixture.id.as_str())).collect();
            let (mut covered, mut total) = (0, 0.0);
            for weld in &welds {
                let m = analysis
                    .robots
                    .iter()
                    .filter(|r| product_collision(&boxes, weld, r.tool_length()).is_empty())
                    .filter_map(|r| r.margin(weld).map(|m| m.0))
                    .fold(f64::MIN, f64::max);
                if m >= TARGET_MARGIN_M {
                    covered += 1;
                }
                total += m.clamp(-1.0, 0.5);
            }
            if (covered, total) > (best.1, best.2) {
                best = (h, covered, total);
            }
        }
        return Ok((best.0, format!("height {:.2} m: the scene's robots reach {} of {} welds there (reach domes)", best.0, best.1.max(0), points.len())));
    }
    let Some(dressed) = request.robot.as_deref() else {
        return Ok((0.0, "no robots in the scene and no `robot` to choose the height for: it stands on the floor".to_owned()));
    };
    let rel = ctx.dressed(dressed)?;
    let kind = ctx.reach.robot_kind(&ctx.scene_base.join(rel)).map_err(|err| format!("{err:#}"))?;
    let envelope = kind.envelope.as_ref().ok_or("the robot has no reach dome (needs a 6-axis arm with a vertical A1)")?;
    let (z_best, _, _) = envelope.profile.iter().copied().fold((0.0, 0.0, f64::MIN), |a, p| if p.2 > a.2 { p } else { a });
    // The wrist is the tool's length behind the weld, against the approach (= -normal).
    let mean = points.iter().map(|(p, n)| p[2] as f64 + kind.tool_length * n[2] as f64).sum::<f64>() / points.len() as f64;
    let base = request.pedestal_mm.unwrap_or(0.0) / 1000.0;
    let h = (base + z_best - mean).clamp(0.0, 2.0);
    Ok((h, format!("height {h:.2} m: the welds at {dressed}'s farthest reach ({:.2} m above its base)", z_best)))
}

async fn add_robot(State(state): State<AppState>, UrlPath(file): UrlPath<String>, Json(request): Json<AddRobot>) -> Reply<SceneEdited> {
    edit(state, file, move |scene, ctx| {
        let urdf = ctx.dressed(request.dressed.trim())?;
        let id = unique_id(scene, request.robot_id.as_deref().unwrap_or(request.dressed.trim()));
        let [x, y] = request.position;
        let yaw = request.yaw_deg.unwrap_or_else(|| facing(scene, [x, y]).unwrap_or(0.0));
        scene.robots.push(RobotYaml {
            robot_id: id.clone(),
            dressed_urdf: urdf,
            position: Xyz { x, y, z: 0.0 },
            orientation: Abc { a: yaw, b: 0.0, c: 0.0 },
            home_pose_deg: request.home_pose_deg.clone(),
            pedestal: request.pedestal_mm.filter(|mm| *mm > 0.0).map(|mm| PedestalYaml::new(mm, DEFAULT_PEDESTAL_DIAMETER_MM)),
            ..RobotYaml::default()
        });
        Ok((vec![format!("{id}: {} at ({x:.3}, {y:.3}), yaw {yaw:.0}°", request.dressed.trim())], Vec::new()))
    })
    .await
}

/// The yaw (degrees) from `at` towards the centre of the scene's fixtures.
fn facing(scene: &SceneFile, at: [f64; 2]) -> Option<f64> {
    let n = scene.fixtures.len() as f64;
    if n == 0.0 {
        return None;
    }
    let cx = scene.fixtures.iter().map(|f| f.position.x).sum::<f64>() / n;
    let cy = scene.fixtures.iter().map(|f| f.position.y).sum::<f64>() / n;
    let (dx, dy) = (cx - at[0], cy - at[1]);
    (dx.hypot(dy) > 1e-6).then(|| (dy.atan2(dx).to_degrees() * 10.0).round() / 10.0)
}

async fn move_robot(State(state): State<AppState>, UrlPath(file): UrlPath<String>, Json(request): Json<MoveRobot>) -> Reply<SceneEdited> {
    edit(state, file, move |scene, ctx| {
        let urdf = request.dressed.as_deref().map(|d| ctx.dressed(d.trim())).transpose()?;
        let snapshot = scene.clone();
        let robot = scene.robots.iter_mut().find(|r| r.robot_id == request.robot_id).ok_or_else(|| format!("the scene has no robot {}", request.robot_id))?;
        let mut what = Vec::new();
        if let Some([x, y]) = request.position {
            (robot.position.x, robot.position.y) = (x, y);
            what.push(format!("to ({x:.3}, {y:.3})"));
        }
        if let Some([dx, dy]) = request.delta {
            robot.position.x += dx;
            robot.position.y += dy;
            what.push(format!("by ({dx:.3}, {dy:.3})"));
        }
        if request.face_products
            && let Some(yaw) = facing(&snapshot, [robot.position.x, robot.position.y])
        {
            robot.orientation.a = yaw;
            what.push(format!("facing the products ({yaw:.0}°)"));
        } else if let Some(yaw) = request.yaw_deg {
            robot.orientation.a = yaw;
            what.push(format!("yaw {yaw:.0}°"));
        }
        if let Some(mm) = request.pedestal_mm {
            robot.pedestal = (mm > 0.0).then(|| robot.pedestal.clone().map(|mut p| { p.height_mm = mm; p }).unwrap_or_else(|| PedestalYaml::new(mm, DEFAULT_PEDESTAL_DIAMETER_MM)));
            what.push(if mm > 0.0 { format!("pedestal {mm:.0} mm") } else { "no pedestal".to_owned() });
        }
        if let Some(pose) = &request.home_pose_deg {
            robot.home_pose_deg = Some(pose.clone());
            what.push("home pose".to_owned());
        }
        if let Some(urdf) = urdf {
            robot.dressed_urdf = urdf;
            what.push(format!("dressed {}", request.dressed.as_deref().unwrap_or_default().trim()));
        }
        if what.is_empty() {
            return Err("nothing to change: give position, delta, yaw_deg, face_products, pedestal_mm, home_pose_deg or dressed".to_owned());
        }
        Ok((vec![format!("{}: {}", request.robot_id, what.join(", "))], Vec::new()))
    })
    .await
}

async fn remove_robots(State(state): State<AppState>, UrlPath(file): UrlPath<String>, Json(request): Json<RemoveItems>) -> Reply<SceneEdited> {
    edit(state, file, move |scene, _| {
        let missing: Vec<&String> = request.ids.iter().filter(|id| !scene.robots.iter().any(|r| &r.robot_id == *id)).collect();
        if !missing.is_empty() {
            return Err(format!("the scene has no robot {missing:?}"));
        }
        scene.robots.retain(|r| !request.ids.contains(&r.robot_id));
        Ok((vec![format!("removed {}", request.ids.join(", "))], Vec::new()))
    })
    .await
}

async fn remove_fixtures(State(state): State<AppState>, UrlPath(file): UrlPath<String>, Json(request): Json<RemoveItems>) -> Reply<SceneEdited> {
    edit(state, file, move |scene, _| {
        let missing: Vec<&String> = request.ids.iter().filter(|id| !scene.fixtures.iter().any(|f| &f.id == *id)).collect();
        if !missing.is_empty() {
            return Err(format!("the scene has no fixture {missing:?}"));
        }
        // A product's support goes with it.
        let supports: Vec<String> = request.ids.iter().map(|id| format!("{id}_support")).collect();
        scene.fixtures.retain(|f| !request.ids.contains(&f.id) && !supports.contains(&f.id));
        Ok((vec![format!("removed {}", request.ids.join(", "))], Vec::new()))
    })
    .await
}

// --- placing robots ---

async fn place_robots(State(state): State<AppState>, Json(request): Json<PlaceRobots>) -> Reply<PlaceRobotsResult> {
    let (config, fitter) = (state.config.clone(), state.fitter.clone());
    tokio::task::spawn_blocking(move || {
        let scenes_dir = fitter.scenes_dir();
        let resolved = super::reach::http::resolve(&config, &scenes_dir, request.scene_file.as_deref(), request.cell.as_deref(), None)?;
        let write_as = request.write_as.as_deref().map(str::trim).filter(|n| !n.is_empty()).map(super::reach::http::new_scene_name).transpose()?;
        if let Some(name) = &write_as
            && scene_store::scene_dir(&scenes_dir, name).exists()
        {
            return Err((StatusCode::CONFLICT, format!("scene {name} exists: scenes are never overwritten, pick another name")));
        }
        let ctx = fitter.reach_context();
        let base = &resolved.scene_base;
        let dressed = match request.dressed.as_deref().map(str::trim).filter(|d| !d.is_empty()) {
            Some(d) => {
                let catalogue = scenes_dir.parent().map(Path::to_path_buf).unwrap_or_else(|| scenes_dir.clone());
                let file = catalogue.join("urdf-dressed-robots").join(d).join(format!("{d}.urdf"));
                if d.contains("..") || !file.is_file() {
                    return Err(bad(format!("no dressed robot {d}: dress it first (dresser_dress)")));
                }
                // As the scene references it: relative to its folder scenes/<name>/ (absolute for
                // another base, e.g. a cell's own scene).
                if base.parent().is_some_and(|p| p == scenes_dir) { format!("../../urdf-dressed-robots/{d}/{d}.urdf") } else { file.to_string_lossy().into_owned() }
            }
            None => resolved.scene.robots.first().map(|r| r.dressed_urdf.clone()).ok_or_else(|| bad("give `dressed` (<robot>__<tool>): the scene has no robot to copy"))?,
        };
        let mut result = place(&ctx, &resolved.scene, base, resolved.cell_root.as_deref(), &dressed, &request).map_err(bad)?;
        if let (Some(scene), Some(name)) = (&result.scene, write_as.as_deref()) {
            let (out, warnings) = super::reach::http::write_new_scene(&config, &scenes_dir, base, scene, name)?;
            result.notes.extend(warnings);
            result.scene = Some(out);
            result.written = Some(name.to_owned());
            fitter.refresh_palette();
        }
        Ok(Json(result))
    })
    .await
    .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, format!("placing robots panicked: {err}")))?
}

/// A candidate base `[x, y]` and, per weld, its penalty (see [`comfort::weld_comfort`]).
struct Candidate {
    xy: [f64; 2],
    cost: Vec<f64>,
}

/// The cost of a weld no robot (or not `coverage` robots) of a set reaches.
const MISSING: f64 = 30.0;

/// A weld's cost for a set of robots: its `coverage` best penalties (a missing robot costs [`MISSING`]).
fn weld_cost(costs: &mut Vec<f64>, coverage: usize) -> f64 {
    costs.sort_by(|a, b| a.total_cmp(b));
    (0..coverage).map(|i| costs.get(i).copied().unwrap_or(MISSING).min(MISSING)).sum()
}

fn set_cost(chosen: &[usize], kept: &[Vec<f64>], candidates: &[Candidate], welds: usize, coverage: usize) -> f64 {
    let mut total = 0.0;
    let mut costs = Vec::with_capacity(chosen.len() + kept.len());
    for w in 0..welds {
        costs.clear();
        costs.extend(kept.iter().map(|k| k[w]));
        costs.extend(chosen.iter().map(|&c| candidates[c].cost[w]));
        total += weld_cost(&mut costs, coverage);
    }
    total
}

/// The robots and welds of a layout as [`RobotFit`]s / [`WeldFit`]s: each weld goes to the robot that
/// does it best; each robot's yaw faces its welds.
pub(crate) fn fits(
    robots: &[(String, Isometry3<f64>)],
    welds: &[TaskWeld],
    boxes: &[&super::reach::task::ProductBox],
    envelope: &super::reach::envelope::Envelope,
    l: f64,
    spec: &ComfortSpec,
) -> (Vec<RobotFit>, Vec<WeldFit>) {
    let mut per_weld = Vec::new();
    let mut by_robot: Vec<Vec<(usize, comfort::WeldComfort)>> = vec![Vec::new(); robots.len()];
    for (w, weld) in welds.iter().enumerate() {
        let collides = !product_collision(boxes, weld, l).is_empty();
        let best = robots
            .iter()
            .enumerate()
            .map(|(r, (_, base))| {
                let (m, _) = envelope.margin(base, l, weld.position, weld.approach);
                (r, comfort::weld_comfort(envelope, base, l, weld, m >= 0.0 && !collides, spec))
            })
            .min_by(|a, b| a.1.penalty.total_cmp(&b.1.penalty));
        let Some((r, c)) = best else { continue };
        per_weld.push(WeldFit {
            weld: weld.label.clone(),
            robot: robots[r].0.clone(),
            fit: c.fit.name().to_owned(),
            ratio: round2(c.ratio),
            align_deg: c.align_deg.round(),
            height_m: round2(c.height_m),
            move_m: round2(c.move_m),
        });
        by_robot[r].push((w, c));
    }
    let robots_fit = robots
        .iter()
        .zip(&by_robot)
        .map(|((id, base), mine)| {
            let t = base.translation.vector;
            let yaw = base.rotation.euler_angles().2.to_degrees();
            let mut fit = RobotFit { robot: id.clone(), base: [round2(t.x), round2(t.y), yaw.round()], welds: mine.len(), ..RobotFit::default() };
            if mine.is_empty() {
                fit.advice = Some("does no weld best: remove it, or move it towards welds another robot does badly".to_owned());
                return fit;
            }
            for (_, c) in mine {
                *fit.fits.entry(c.fit.name().to_owned()).or_default() += 1;
            }
            fit.ratio_min = round2(mine.iter().map(|(_, c)| c.ratio).fold(f64::MAX, f64::min));
            fit.ratio_max = round2(mine.iter().map(|(_, c)| c.ratio).fold(f64::MIN, f64::max));
            // Angles of its welds about A1 (scene frame), and how far they spread.
            let mut angles: Vec<f64> = mine.iter().map(|(w, _)| (welds[*w].position[1] - t.y).atan2(welds[*w].position[0] - t.x)).collect();
            angles.sort_by(|a, b| a.total_cmp(b));
            let gaps = angles.windows(2).map(|p| p[1] - p[0]).chain(std::iter::once(angles[0] + std::f64::consts::TAU - angles[angles.len() - 1]));
            let largest_gap = gaps.fold(0.0_f64, f64::max);
            fit.spread_deg = (std::f64::consts::TAU - largest_gap).to_degrees().round();
            // Where it would stand comfortably: behind its welds, along their (mean) approach, at the
            // target share of its reach at their wrists' height.
            let not_ok = mine.iter().filter(|(_, c)| matches!(c.fit, comfort::Fit::Cramped | comfort::Fit::Stretched | comfort::Fit::Twisted | comfort::Fit::Unreachable)).count();
            let n = mine.len() as f64;
            let wrist = |w: usize| {
                let (p, a) = (welds[w].position, welds[w].approach);
                [p[0] - a[0] * l, p[1] - a[1] * l, p[2] - a[2] * l]
            };
            let (mut wx, mut wy, mut wz, mut ax, mut ay) = (0.0, 0.0, 0.0, 0.0, 0.0);
            for (w, _) in mine {
                let p = wrist(*w);
                wx += p[0] / n;
                wy += p[1] / n;
                wz += p[2] / n;
                ax += welds[*w].approach[0];
                ay += welds[*w].approach[1];
            }
            let a_len = ax.hypot(ay);
            if not_ok > 0 && a_len > 0.3 * n {
                let r_max = envelope.r_max_at(wz - t.z).unwrap_or(0.0);
                let d = spec.ratio_target * r_max;
                let target = [round2(wx - ax / a_len * d), round2(wy - ay / a_len * d)];
                let off = (target[0] - t.x).hypot(target[1] - t.y);
                if off > 0.25 {
                    let mut why = Vec::new();
                    for (name, label) in [("cramped", "cramped (too close)"), ("stretched", "stretched (too far)"), ("twisted", "twisted (gun far off its line)"), ("unreachable", "unreachable")] {
                        if let Some(k) = fit.fits.get(name) {
                            why.push(format!("{k} {label}"));
                        }
                    }
                    fit.advice = Some(format!(
                        "{} of its {} welds: move it {off:.2} m to about ({:.2}, {:.2}), behind its welds along their approach at {:.0} % of its reach",
                        why.join(", "),
                        mine.len(),
                        target[0],
                        target[1],
                        spec.ratio_target * 100.0
                    ));
                    fit.suggested_base = Some(target);
                } else if fit.fits.contains_key("twisted") {
                    fit.advice = Some(format!(
                        "its welds spread {:.0}° about A1 along the part: one robot can't stand behind all of them; another robot for the far ones makes them comfortable",
                        fit.spread_deg
                    ));
                }
            } else if fit.fits.keys().any(|k| k == "too_high" || k == "too_low") {
                fit.advice = Some("its welds are far off its best height: change the pedestal (pedestal_mm) or the product's height".to_owned());
            } else if fit.spread_deg > 120.0 {
                fit.advice = Some(format!("its welds spread {:.0}° about A1: it swings a lot between them; another robot or a more central base", fit.spread_deg));
            }
            fit
        })
        .collect();
    (robots_fit, per_weld)
}

fn round2(v: f64) -> f64 {
    (v * 100.0).round() / 100.0
}

/// The yaw (degrees) from `xy` to the centre of `welds`.
fn yaw_to(xy: [f64; 2], welds: &[&TaskWeld]) -> f64 {
    if welds.is_empty() {
        return 0.0;
    }
    let n = welds.len() as f64;
    let (cx, cy) = (welds.iter().map(|w| w.position[0]).sum::<f64>() / n, welds.iter().map(|w| w.position[1]).sum::<f64>() / n);
    ((cy - xy[1]).atan2(cx - xy[0]).to_degrees() * 10.0).round() / 10.0
}

fn iso(xy: [f64; 2], lift: f64, yaw_deg: f64) -> Isometry3<f64> {
    Isometry3::from_parts(Translation3::new(xy[0], xy[1], lift), UnitQuaternion::from_euler_angles(0.0, 0.0, yaw_deg.to_radians()))
}

/// Place 1..N robots of `dressed_urdf` around the scene's products (see the module doc). Every
/// robot figure comes from the dressed URDF ([`robot_info::describe`]): the dome and the tool
/// length rate the welds, the comfortable height band is the dome's own unless
/// `comfort.height_band_m` is given, and `min_spacing_m` / `clearance_m` left at 0 are the
/// robot's recommended spacing and clearance (1.5 / 0.4 m only when the URDF gives nothing to
/// derive them from; the result's notes say which).
pub(crate) fn place(
    ctx: &ReachContext,
    scene: &SceneFile,
    base: &Path,
    cell_root: Option<&Path>,
    dressed_urdf: &str,
    request: &PlaceRobots,
) -> Result<PlaceRobotsResult, String> {
    let mut result = PlaceRobotsResult::default();
    let kind = ctx.robot_kind(&base.join(dressed_urdf)).map_err(|err| format!("{err:#}"))?;
    let envelope = kind.envelope.clone().ok_or("the robot has no reach dome (needs a 6-axis arm with a vertical A1)")?;
    let l = kind.tool_length;
    let spec = request.comfort.clone();
    let mut kept = scene.clone();
    if !request.keep_robots {
        kept.robots.clear();
    }
    let analysis = analyse(ctx, &kept, base, cell_root, TaskSource::Auto);
    let welds = analysis.welds.clone();
    if welds.is_empty() {
        return Err("the scene has no welds: place a product with welds first (scene_place_product)".to_owned());
    }
    let boxes = analysis.boxes();
    let lift = request.pedestal_mm.unwrap_or(0.0).max(0.0) / 1000.0;
    let coverage = request.coverage.clamp(1, 4);
    let (min_n, max_n) = (request.min_robots.max(1), request.max_robots.clamp(1, 8).max(request.min_robots.max(1)));
    let info = robot_info::describe(ctx, &base.join(dressed_urdf), &spec).ok();
    let (spacing, clearance) = robot_info::spacing_and_clearance(request.min_spacing_m, request.clearance_m, info.as_ref(), &mut result.notes);
    let collides: Vec<bool> = welds.iter().map(|w| !product_collision(&boxes, w, l).is_empty()).collect();
    let cost_at = |b: &Isometry3<f64>, k: &super::reach::RobotKind| -> Vec<f64> {
        let env = k.envelope.as_ref();
        welds
            .iter()
            .zip(&collides)
            .map(|(w, col)| {
                let Some(env) = env else { return MISSING };
                let (m, _) = env.margin(b, k.tool_length, w.position, w.approach);
                comfort::weld_comfort(env, b, k.tool_length, w, m >= TARGET_MARGIN_M && !col, &spec).penalty
            })
            .collect()
    };
    // The kept robots' costs, and their bases (for the spacing).
    let kept_costs: Vec<Vec<f64>> = analysis.robots.iter().filter_map(|r| Some(cost_at(&r.base, r.kind.as_ref()?))).collect();
    let kept_bases: Vec<[f64; 2]> = analysis.robots.iter().map(|r| [r.base.translation.vector.x, r.base.translation.vector.y]).collect();
    // Candidate bases: a 0.2 m grid over the welds, as far out as the dome reaches, clear of the products.
    let r_out = envelope.profile.iter().map(|p| p.2).fold(0.0, f64::max) + l;
    let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
    for w in &welds {
        for i in 0..2 {
            lo[i] = lo[i].min(w.position[i] - r_out);
            hi[i] = hi[i].max(w.position[i] + r_out);
        }
    }
    const STEP: f64 = 0.2;
    let mut candidates = Vec::new();
    let mut x = lo[0];
    while x <= hi[0] {
        let mut y = lo[1];
        while y <= hi[1] {
            let clear = boxes.iter().all(|b| b.signed_distance([x, y, b.pose.translation.vector.z]) >= clearance)
                && kept_bases.iter().all(|k| (k[0] - x).hypot(k[1] - y) >= spacing);
            if clear {
                let cost = cost_at(&iso([x, y], lift, 0.0), &kind);
                if cost.iter().any(|c| *c < MISSING) {
                    candidates.push(Candidate { xy: [round2(x), round2(y)], cost });
                }
            }
            y += STEP;
        }
        x += STEP;
    }
    if candidates.is_empty() {
        return Err(format!("no base around the products reaches any weld with this robot (wrist reach plus tool {r_out:.2} m); try a pedestal (pedestal_mm) or a bigger robot"));
    }
    let n_welds = welds.len();
    let far = |chosen: &[usize], c: usize, skip: Option<usize>| {
        chosen.iter().enumerate().filter(|(i, _)| Some(*i) != skip).all(|(_, &j)| {
            (candidates[j].xy[0] - candidates[c].xy[0]).hypot(candidates[j].xy[1] - candidates[c].xy[1]) >= spacing
        })
    };
    let mut chosen: Vec<usize> = Vec::new();
    for robots in 1..=max_n {
        // Add the base that lowers the cost most, then improve every base in turn while that helps.
        let add = (0..candidates.len())
            .filter(|&c| !chosen.contains(&c) && far(&chosen, c, None))
            .min_by(|&a, &b| {
                let mut with_a = chosen.clone();
                with_a.push(a);
                let mut with_b = chosen.clone();
                with_b.push(b);
                set_cost(&with_a, &kept_costs, &candidates, n_welds, coverage).total_cmp(&set_cost(&with_b, &kept_costs, &candidates, n_welds, coverage))
            });
        let Some(add) = add else {
            result.notes.push(format!("no room for robot {robots} ({spacing:.1} m between bases, {clearance:.1} m from the products)"));
            break;
        };
        chosen.push(add);
        for _ in 0..4 {
            let mut improved = false;
            for i in 0..chosen.len() {
                let current = set_cost(&chosen, &kept_costs, &candidates, n_welds, coverage);
                let best = (0..candidates.len())
                    .filter(|&c| !chosen.contains(&c) && far(&chosen, c, Some(i)))
                    .map(|c| {
                        let mut trial = chosen.clone();
                        trial[i] = c;
                        (c, set_cost(&trial, &kept_costs, &candidates, n_welds, coverage))
                    })
                    .min_by(|a, b| a.1.total_cmp(&b.1));
                if let Some((c, cost)) = best
                    && cost < current - 1e-9
                {
                    chosen[i] = c;
                    improved = true;
                }
            }
            if !improved {
                break;
            }
        }
        if robots < min_n {
            continue;
        }
        // The layout: each robot faces the welds it does best.
        let cost = set_cost(&chosen, &kept_costs, &candidates, n_welds, coverage);
        let mut owner = vec![usize::MAX; n_welds];
        for (w, o) in owner.iter_mut().enumerate() {
            *o = (0..chosen.len()).min_by(|&a, &b| candidates[chosen[a]].cost[w].total_cmp(&candidates[chosen[b]].cost[w])).unwrap_or(usize::MAX);
        }
        let bases: Vec<[f64; 3]> = chosen
            .iter()
            .enumerate()
            .map(|(i, &c)| {
                let mine: Vec<&TaskWeld> = welds.iter().enumerate().filter(|(w, _)| owner[*w] == i).map(|(_, w)| w).collect();
                let xy = candidates[c].xy;
                [xy[0], xy[1], yaw_to(xy, if mine.is_empty() { welds.iter().collect::<Vec<_>>() } else { mine }.as_slice())]
            })
            .collect();
        let robots_iso: Vec<(String, Isometry3<f64>)> =
            bases.iter().enumerate().map(|(i, b)| (format!("new_{}", i + 1), iso([b[0], b[1]], lift, b[2]))).collect();
        let (robots_fit, per_weld) = fits(&robots_iso, &welds, &boxes, &envelope, l, &spec);
        let covered = (0..n_welds)
            .filter(|&w| {
                let mut c: Vec<f64> = kept_costs.iter().map(|k| k[w]).chain(chosen.iter().map(|&c| candidates[c].cost[w])).collect();
                weld_cost(&mut c, coverage) < MISSING * 0.66
            })
            .count();
        let uncovered: Vec<String> = per_weld.iter().filter(|f| f.fit == "unreachable").map(|f| f.weld.clone()).collect();
        let comfortable = per_weld.iter().filter(|f| f.fit == "comfortable").count();
        result.layouts.push(Layout {
            robots: robots + kept_bases.len(),
            bases,
            covered,
            welds: n_welds,
            uncovered,
            comfortable,
            robots_fit,
            cost: round2(cost),
        });
        // Fewest robots: stop once every weld is comfortable (an exact count goes on to it).
        if comfortable == n_welds && robots >= min_n && min_n != max_n {
            break;
        }
    }
    if result.layouts.is_empty() {
        return Err(format!("no layout of {min_n}..{max_n} robots fits ({spacing:.1} m between bases, {clearance:.1} m from the products)"));
    }
    // The fewest robots that do every weld comfortably, else that reach every weld, else the cheapest.
    result.best = result
        .layouts
        .iter()
        .position(|l| l.comfortable == l.welds)
        .or_else(|| result.layouts.iter().position(|l| l.uncovered.is_empty() && l.covered == l.welds))
        .or_else(|| result.layouts.iter().enumerate().min_by(|a, b| a.1.cost.total_cmp(&b.1.cost)).map(|(i, _)| i));
    if let Some(layout) = result.best.map(|i| result.layouts[i].clone()) {
        let mut out = kept.clone();
        let stem = Path::new(dressed_urdf).file_stem().and_then(|s| s.to_str()).unwrap_or("robot").to_owned();
        for [x, y, yaw] in &layout.bases {
            let id = unique_id(&out, &stem);
            out.robots.push(RobotYaml {
                robot_id: id,
                dressed_urdf: dressed_urdf.to_owned(),
                position: Xyz { x: *x, y: *y, z: 0.0 },
                orientation: Abc { a: *yaw, b: 0.0, c: 0.0 },
                pedestal: request.pedestal_mm.filter(|mm| *mm > 0.0).map(|mm| PedestalYaml::new(mm, DEFAULT_PEDESTAL_DIAMETER_MM)),
                ..RobotYaml::default()
            });
        }
        let options = CheckOptions { source: TaskSource::Auto, mode: request.verify.unwrap_or_default(), estimate_only: request.verify.is_none(), ..CheckOptions::default() };
        result.report = Some(check_scene(ctx, &out, base, cell_root, &options, |_| {}, &AtomicBool::new(false)));
        result.scene = Some(out);
    }
    if request.verify.is_none() {
        result.notes.push("checked with the reach domes only: confirm with scene_reach_check (mode full) before planning".to_owned());
    }
    Ok(result)
}

/// The comfort of a scene's own robots on its welds, each robot rated with its own dome and tool
/// (and its dome's height band unless `spec.height_band_m` is given). Notes the bases closer to
/// each other, or to a product, than their robot's recommended spacing / clearance
/// ([`robot_info::describe`]).
pub(crate) fn quality(ctx: &ReachContext, scene: &SceneFile, base: &Path, cell_root: Option<&Path>, spec: &ComfortSpec) -> Result<LayoutQuality, String> {
    let analysis = analyse(ctx, scene, base, cell_root, TaskSource::Auto);
    let mut q = LayoutQuality { welds: analysis.welds.len(), notes: analysis.notes.clone(), ..LayoutQuality::default() };
    let robots: Vec<&super::reach::SceneRobot> = analysis.robots.iter().filter(|r| r.kind.as_ref().is_some_and(|k| k.envelope.is_some())).collect();
    if robots.is_empty() {
        return Err("the scene has no robot with a reach dome".to_owned());
    }
    // One robot model per scene is the usual case; robots of other models are rated with theirs.
    let boxes = analysis.boxes();
    let mut per_weld: Vec<WeldFit> = Vec::new();
    let mut robots_fit = Vec::new();
    // Rate each weld against every robot with that robot's own dome and tool, then group.
    let mut owner: Vec<(usize, comfort::WeldComfort)> = Vec::new();
    for weld in &analysis.welds {
        let best = robots
            .iter()
            .enumerate()
            .map(|(i, r)| {
                let k = r.kind.as_ref().expect("filtered");
                let env = k.envelope.as_ref().expect("filtered");
                let (m, _) = env.margin(&r.base, k.tool_length, weld.position, weld.approach);
                let ok = m >= 0.0 && product_collision(&boxes, weld, k.tool_length).is_empty();
                (i, comfort::weld_comfort(env, &r.base, k.tool_length, weld, ok, spec))
            })
            .min_by(|a, b| a.1.penalty.total_cmp(&b.1.penalty))
            .expect("robots");
        owner.push(best);
    }
    for (i, r) in robots.iter().enumerate() {
        let k = r.kind.as_ref().expect("filtered");
        let env = k.envelope.as_ref().expect("filtered");
        let mine: Vec<TaskWeld> = analysis.welds.iter().zip(&owner).filter(|(_, (o, _))| *o == i).map(|(w, _)| w.clone()).collect();
        let (mut fit, weld_fits) = fits(&[(r.id.clone(), r.base)], &mine, &boxes, env, k.tool_length, spec);
        per_weld.extend(weld_fits);
        robots_fit.append(&mut fit);
    }
    for f in &per_weld {
        *q.fits.entry(f.fit.clone()).or_default() += 1;
    }
    let bases: Vec<[f64; 2]> = robots.iter().map(|r| [r.base.translation.vector.x, r.base.translation.vector.y]).collect();
    q.closest_bases_m = bases
        .iter()
        .enumerate()
        .flat_map(|(i, a)| bases[i + 1..].iter().map(move |b| (a[0] - b[0]).hypot(a[1] - b[1])))
        .fold(None, |m: Option<f64>, d| Some(m.map_or(d, |m| m.min(d))))
        .map(round2);
    // The bases against their robots' own spacing and clearance.
    let mut described: Vec<(PathBuf, Option<factoryos_protocol::robot::RobotInfo>)> = Vec::new();
    for r in &robots {
        if !described.iter().any(|(p, _)| *p == r.urdf) {
            described.push((r.urdf.clone(), robot_info::describe(ctx, &r.urdf, spec).ok()));
        }
    }
    let figures = |r: &super::reach::SceneRobot| described.iter().find(|(p, _)| *p == r.urdf).and_then(|(_, i)| i.as_ref()).map(|i| &i.placement);
    for (i, a) in robots.iter().enumerate() {
        for b in &robots[i + 1..] {
            let d = (a.base.translation.x - b.base.translation.x).hypot(a.base.translation.y - b.base.translation.y);
            let want = figures(a).and_then(|p| p.min_spacing_m).into_iter().chain(figures(b).and_then(|p| p.min_spacing_m)).fold(0.0, f64::max);
            if want > 0.0 && d < want - 1e-6 {
                q.notes.push(format!("{} and {} are {d:.2} m apart: under their recommended {want:.2} m (robot_describe)", a.id, b.id));
            }
        }
        if let Some(c) = figures(a).and_then(|p| p.clearance_m) {
            let t = a.base.translation;
            let near = boxes.iter().map(|b| b.signed_distance([t.x, t.y, b.pose.translation.vector.z])).fold(f64::MAX, f64::min);
            if near < c - 1e-6 {
                q.notes.push(format!("{} stands {near:.2} m from a product: under its recommended clearance {c:.2} m (robot_describe)", a.id));
            }
        }
    }
    q.robots = robots_fit;
    q.per_weld = per_weld;
    Ok(q)
}

async fn layout_quality(State(state): State<AppState>, Json(request): Json<LayoutQualityRequest>) -> Reply<LayoutQuality> {
    let (config, fitter) = (state.config.clone(), state.fitter.clone());
    tokio::task::spawn_blocking(move || {
        let scenes_dir = fitter.scenes_dir();
        let resolved = super::reach::http::resolve(&config, &scenes_dir, request.scene_file.as_deref(), request.cell.as_deref(), None)?;
        quality(&fitter.reach_context(), &resolved.scene, &resolved.scene_base, resolved.cell_root.as_deref(), &request.comfort).map(Json).map_err(bad)
    })
    .await
    .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, format!("layout quality panicked: {err}")))?
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_are_unique_and_robots_face_the_fixtures() {
        let mut scene = empty_scene("s", "");
        scene.fixtures.push(FixtureYaml { id: "cube".into(), position: Xyz { x: 2.0, y: 0.0, z: 0.0 }, ..FixtureYaml::default() });
        assert_eq!(unique_id(&scene, "cube"), "cube_2");
        assert_eq!(unique_id(&scene, "kr240"), "kr240");
        assert_eq!(facing(&scene, [0.0, 0.0]), Some(0.0));
        assert_eq!(facing(&scene, [2.0, 2.0]), Some(-90.0));
        assert_eq!(facing(&empty_scene("e", ""), [0.0, 0.0]), None);
    }

    #[test]
    fn robots_are_placed_to_reach_the_welds() {
        let scene = crate::services::reach::task::tests::jam2();
        let base = crate::services::reach::task::tests::fixtures().join("scenes/jam2");
        let dressed = scene.robots[0].dressed_urdf.clone();
        let ctx = crate::services::reach::tests_support::ctx();
        let request = PlaceRobots { min_robots: 1, max_robots: 3, ..PlaceRobots::default() };
        let result = place(&ctx, &scene, &base, None, &dressed, &request).unwrap();
        let best = &result.layouts[result.best.expect("a layout")];
        assert!(best.covered > 0, "{result:#?}");
        let placed = result.scene.as_ref().unwrap();
        assert_eq!(placed.robots.len(), best.robots);
        // Bases at least the robot's own min spacing apart (derived from its URDF, said in the notes).
        let info = robot_info::describe(&ctx, &base.join(&dressed), &ComfortSpec::default()).unwrap();
        let spacing = info.placement.min_spacing_m.expect("derived from the dome");
        assert!(result.notes.iter().any(|n| n.contains(&format!("{spacing:.2} m apart"))), "{:?}", result.notes);
        assert!(result.notes.iter().any(|n| n.contains("a fallback")), "no meshes, no footprint: {:?}", result.notes);
        for (i, a) in placed.robots.iter().enumerate() {
            for b in &placed.robots[i + 1..] {
                assert!((a.position.x - b.position.x).hypot(a.position.y - b.position.y) >= spacing - 1e-6);
            }
        }
    }

    /// body_weld_4x8: a car body with 4 welds per side, approached from outside; two robots.
    fn car_scene() -> (SceneFile, PathBuf, String) {
        use factoryos_protocol::product::{Geometry, Operation, Product, Target, Task};
        let dir = std::env::temp_dir().join(format!("factoryos-place-car-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        let product_dir = dir.join("products/car");
        std::fs::create_dir_all(&product_dir).unwrap();
        std::fs::create_dir_all(dir.join("catalogue/scenes")).unwrap();
        let welds = [
            ("W01", [-0.8933, -1.85, 1.0], [-0.988, -0.151, -0.028]),
            ("W02", [-0.927, -1.6, 0.75], [-0.978, -0.148, 0.144]),
            ("W03", [-0.9353, -0.875, 0.725], [-0.995, 0.023, 0.101]),
            ("W05", [-0.918, 1.175, 0.75], [-0.99, 0.144, -0.003]),
            ("W07", [0.8933, -1.85, 1.0], [0.988, -0.151, 0.026]),
            ("W08", [0.9273, -1.6, 0.75], [0.98, -0.142, 0.139]),
            ("W09", [0.9352, -0.875, 0.725], [0.996, 0.024, 0.08]),
            ("W11", [0.918, 1.175, 0.75], [0.99, 0.144, -0.003]),
        ];
        let product = Product {
            name: "car".into(),
            description: String::new(),
            geometry: Geometry::Box { size: [1.78, 4.6, 1.4] },
            tasks: vec![Task {
                name: "spot".into(),
                process: "spotWeld".into(),
                operations: welds
                    .iter()
                    .map(|(id, p, n)| Operation { id: (*id).into(), target: Target::Point { position: *p, normal: *n }, group: String::new(), params: Default::default() })
                    .collect(),
                ..Task::default()
            }],
        };
        std::fs::write(product_dir.join("product.json"), serde_json::to_string(&product).unwrap()).unwrap();
        std::fs::write(product_dir.join("car.urdf"), "<robot name='car'><link name='World'/></robot>").unwrap();
        let mut scene = empty_scene("car", "");
        scene.fixtures.push(FixtureYaml { id: "car".into(), urdf: product_dir.join("car.urdf").display().to_string(), ..FixtureYaml::default() });
        let robot = crate::services::reach::task::tests::fixtures().join("kr240r2900__sticktool/kr240r2900__sticktool.urdf");
        (scene, dir.join("catalogue/scenes"), robot.display().to_string())
    }

    #[test]
    fn two_robots_stand_back_from_a_car_body() {
        let (scene, base, robot) = car_scene();
        let ctx = crate::services::reach::tests_support::ctx();
        let request = PlaceRobots { min_robots: 2, max_robots: 2, ..PlaceRobots::default() };
        let started = std::time::Instant::now();
        let result = place(&ctx, &scene, &base, None, &robot, &request).unwrap();
        let layout = &result.layouts[result.best.expect("a layout")];
        eprintln!("{:.2} s: {}", started.elapsed().as_secs_f64(), serde_json::to_string_pretty(layout).unwrap());
        assert_eq!(layout.robots, 2);
        // One robot per side, standing well back from the welds (x ~ +-0.9).
        let mut xs: Vec<f64> = layout.bases.iter().map(|b| b[0]).collect();
        xs.sort_by(|a, b| a.total_cmp(b));
        assert!(xs[0] < -2.6 && xs[1] > 2.6, "{xs:?}");
        assert_eq!(layout.uncovered, Vec::<String>::new());
        // The hand-placed layout (x = +-2.4) is cramped, the placed one is not.
        let mut cramped = scene.clone();
        for (i, x) in [-2.4, 2.4].into_iter().enumerate() {
            cramped.robots.push(RobotYaml {
                robot_id: format!("r{i}"),
                dressed_urdf: robot.clone(),
                position: Xyz { x, y: -0.35, z: 0.0 },
                orientation: Abc { a: if x < 0.0 { 0.0 } else { 180.0 }, b: 0.0, c: 0.0 },
                ..RobotYaml::default()
            });
        }
        let q = quality(&ctx, &cramped, &base, None, &ComfortSpec::default()).unwrap();
        assert_eq!(q.fits.get("comfortable"), None, "{:?}", q.fits);
        for r in &q.robots {
            let to = r.suggested_base.expect("a move");
            assert!(to[0].abs() > 3.4 && (to[0] - r.base[0]).abs() > 1.0, "{r:?}");
        }
        let placed = quality(&ctx, result.scene.as_ref().unwrap(), &base, None, &ComfortSpec::default()).unwrap();
        assert!(placed.fits.get("comfortable").copied().unwrap_or(0) >= 4, "{:?}", placed.fits);
        // Two robots per side: every weld comfortable, bases apart.
        let four = place(&ctx, &scene, &base, None, &robot, &PlaceRobots { min_robots: 4, max_robots: 4, ..PlaceRobots::default() }).unwrap();
        let layout = &four.layouts[four.best.unwrap()];
        assert_eq!((layout.robots, layout.comfortable), (4, 8), "{layout:#?}");
        // Fewest robots for every weld comfortable: more than two.
        let fewest = place(&ctx, &scene, &base, None, &robot, &PlaceRobots { min_robots: 1, max_robots: 6, ..PlaceRobots::default() }).unwrap();
        let layout = &fewest.layouts[fewest.best.unwrap()];
        assert_eq!(layout.comfortable, 8, "{layout:#?}");
        assert!(layout.robots >= 3, "{layout:#?}");
    }

    #[test]
    fn the_open_scene_in_brief() {
        let mut state = factoryos_protocol::fitter::FitterState::default();
        state.scene_file_name = "body_weld_4x8".into();
        state.linked_cell = Some("body_weld_4x8".into());
        state.scene.robots.push(RobotYaml { robot_id: "r1".into(), dressed_urdf: "../urdf-dressed-robots/kr__gun/kr__gun.urdf".into(), ..RobotYaml::default() });
        let s = summary(&state);
        assert_eq!(s["read_only"], true);
        assert_eq!(s["robots"][0]["dressed"], "kr__gun");
        assert_eq!(s["file_name"], "body_weld_4x8");
    }

    #[test]
    fn supports_are_written_once_per_size() {
        let dir = std::env::temp_dir().join(format!("factoryos-support-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        let a = write_support(&dir, [0.7, 0.5, 0.8]).unwrap();
        assert_eq!(a, "../../urdf-fixture-collection/support_700x500x800mm/support_700x500x800mm.urdf");
        let urdf = dir.join("urdf-fixture-collection/support_700x500x800mm/support_700x500x800mm.urdf");
        assert!(std::fs::read_to_string(urdf).unwrap().contains("meshes/support_700x500x800mm.obj"));
        assert_eq!(write_support(&dir, [0.7, 0.5, 0.8]).unwrap(), a);
        std::fs::remove_dir_all(&dir).ok();
    }
}
