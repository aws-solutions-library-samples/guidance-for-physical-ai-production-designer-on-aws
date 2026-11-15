//! Build training job (see [`factoryos_protocol::training`]): the active cell's robots, products
//! and welds, cells drawn from a job around it ([`sample`]) rendered in 3D on
//! `factoryos/scene/training` ([`scene`]), and the job saved as a plan for factoryos-cell-planner ([`plan`]) in
//! `<cell>/training_jobs/<name>/`.
//!
//! The 3D view has one robot slot per drawn robot (`robots/slot<i>`): the cell's dressed URDF of
//! the slot's model, resolved once and moved to each drawn base; slots a smaller draw leaves
//! unused are hidden rather than removed, so the next draw does not fetch them again.

pub mod plan;
pub mod sample;
pub mod scene;

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use anyhow::Context as _;
use axum::extract::State;
use axum::http::StatusCode;
use axum::routing::post;
use axum::{Json, Router};
use factoryos_protocol::fitter::{SceneFile, TaskSource};
use factoryos_protocol::training::{
    CellRobot, JOBS_DIR, JobFile, LoadJobRequest, PreviewRequest, SampledProduct, SampledWeld, SaveRequest,
    TrainingCell, TrainingJob, TrainingState, valid_job_name,
};
use factoryos_protocol::{api, keys};
use factoryos_protocol::scene::ModelRef;

use crate::AppState;
use crate::bus::Bus;
use crate::config_store::ConfigStore;
use crate::services::reach::{self, ReachContext};
use crate::services::assets::AssetIndex;
use crate::services::scene_pub::ScenePublisher;
use crate::services::urdf_model::UrdfModel;
use scene::SlotScene;

/// The active cell, read once per Open.
struct Loaded {
    cell: TrainingCell,
    root: PathBuf,
    scene_base: PathBuf,
    /// The cell's own welds (3D view frame).
    welds: Vec<SampledWeld>,
    /// Model (as the scene names it) → its home pose, degrees.
    home: HashMap<String, Vec<f64>>,
}

impl Loaded {
    fn urdf(&self, model: &str) -> Option<PathBuf> {
        let path = self.scene_base.join(model);
        path.is_file().then_some(path)
    }
}

struct Slot {
    /// As the scene names it.
    model: String,
    urdf: PathBuf,
    /// Loaded in the background; `None` until then.
    loaded: Option<Arc<SlotModel>>,
}

/// A slot's URDF, parsed and resolved to assets.
struct SlotModel {
    urdf: UrdfModel,
    model: ModelRef,
}

#[derive(Default)]
struct Inner {
    state: TrainingState,
    loaded: Option<Loaded>,
    /// Whether this service's scene shows the loaded cell (set by Open).
    scene_open: bool,
    slots: Vec<Slot>,
    /// The job the shown cell was drawn from (its placement guides).
    job: Option<TrainingJob>,
}

pub struct TrainingService {
    config: Arc<ConfigStore>,
    scene: Arc<ScenePublisher>,
    assets: Arc<AssetIndex>,
    reach: Arc<ReachContext>,
    inner: Mutex<Inner>,
}

impl TrainingService {
    pub fn new(bus: &Bus, config: Arc<ConfigStore>, reach: Arc<ReachContext>, assets: Arc<AssetIndex>) -> Arc<Self> {
        Arc::new(Self { config, scene: ScenePublisher::new(bus, keys::SCENE_TRAINING), assets, reach, inner: Mutex::default() })
    }

    pub fn state(&self) -> TrainingState {
        self.inner.lock().unwrap().state.clone()
    }

    /// Read the active cell, start a new 3D view and show the cell in it.
    pub fn open(self: &Arc<Self>) {
        let mut inner = self.inner.lock().unwrap();
        inner.state.error = None;
        inner.state.saved_to = None;
        let loaded = match self.load_cell() {
            Ok(loaded) => loaded,
            Err(err) => {
                *inner = Inner::default();
                inner.state.error = Some(format!("{err:#}"));
                inner.state.status = "Open a cell first (Robo Fitter → Use as cell, or pick one in Agentic Planning).".to_owned();
                return;
            }
        };
        inner.state.suggested = Some(suggested_job(&loaded.cell));
        inner.state.cell = Some(loaded.cell.clone());
        inner.state.saved_jobs = saved_jobs(&loaded.root);
        inner.state.shown = Some(sample::loaded_cell(&loaded.cell, &loaded.welds));
        inner.state.status = format!(
            "Cell {}: {} robot(s), {} product box(es), {} weld(s).",
            loaded.cell.name,
            loaded.cell.robots.len(),
            loaded.cell.products.len(),
            loaded.cell.welds
        );
        inner.loaded = Some(loaded);
        inner.slots.clear();
        inner.job = None;
        inner.scene_open = true;
        self.show(&mut inner, true);
    }

    /// Show a cell drawn from the job (`seed`), or the loaded cell itself.
    pub fn preview(self: &Arc<Self>, request: PreviewRequest) {
        if self.inner.lock().unwrap().loaded.is_none() {
            self.open();
        }
        let mut inner = self.inner.lock().unwrap();
        let Some(loaded) = inner.loaded.as_ref() else { return };
        let shown = match request.seed {
            Some(seed) => sample::sample(&request.job, seed, &loaded.cell),
            None => sample::loaded_cell(&loaded.cell, &loaded.welds),
        };
        inner.state.status = match shown.seed {
            Some(seed) => format!(
                "Seed {seed}: {} robot(s), {} product(s), {} weld(s).",
                shown.robots.len(),
                shown.products.len(),
                shown.welds.len()
            ),
            None => format!("The loaded cell {}.", loaded.cell.name),
        };
        inner.state.error = None;
        inner.state.shown = Some(shown);
        inner.job = Some(request.job);
        self.show(&mut inner, false);
    }

    /// Save the job in the active cell's `training_jobs/<name>/`.
    pub fn save(self: &Arc<Self>, request: SaveRequest) {
        let mut inner = self.inner.lock().unwrap();
        let result = (|| -> anyhow::Result<(PathBuf, Vec<PathBuf>)> {
            let job = &request.job;
            anyhow::ensure!(valid_job_name(&job.name), "a job name is [A-Za-z0-9._-]+ (up to 64), got {:?}", job.name);
            anyhow::ensure!(job.robots.max_count() > 0 && !job.robots.counts().is_empty(), "give at least one robot count a weight");
            let root = inner.loaded.as_ref().map(|l| l.root.clone()).context("no cell is open")?;
            let dir = root.join(JOBS_DIR).join(&job.name);
            let files = plan::save(job, &dir)?;
            Ok((root, files))
        })();
        match result {
            Ok((root, files)) => {
                let dir = files[0].parent().unwrap_or(&root).display().to_string();
                inner.state.status = format!("Saved {} as a plan in {dir}.", request.job.name);
                inner.state.saved_to = Some(dir);
                inner.state.saved_notes = plan::notes(&request.job);
                inner.state.saved_jobs = saved_jobs(&root);
                inner.state.error = None;
            }
            Err(err) => inner.state.error = Some(format!("Save failed: {err:#}")),
        }
    }

    pub fn load(&self, name: &str) -> anyhow::Result<JobFile> {
        anyhow::ensure!(valid_job_name(name), "not a job name: {name:?}");
        let inner = self.inner.lock().unwrap();
        let root = inner.loaded.as_ref().map(|l| l.root.clone()).context("no cell is open")?;
        plan::load(&root.join(JOBS_DIR).join(name))
    }

    fn load_cell(&self) -> anyhow::Result<Loaded> {
        let resolved = super::cells::active_resolved(&self.config).context("no active cell")?;
        let text = std::fs::read_to_string(&resolved.scene_path)
            .with_context(|| format!("cell {} has no scene ({})", resolved.name, resolved.scene_path.display()))?;
        let mut scene: SceneFile = serde_yaml::from_str(&text).with_context(|| format!("reading {}", resolved.scene_path.display()))?;
        // A scene folder's legacy paths (see `scene_store::asset_path`).
        super::scene_store::fix_legacy_paths(&mut scene, &resolved.scene_base);
        let analysis = reach::analyse(&self.reach, &scene, &resolved.scene_base, Some(&resolved.root), TaskSource::Auto);
        let o = scene.scene_origin.unwrap_or_default();
        let shift = |p: [f64; 3]| [p[0] - o.x, p[1] - o.y, p[2] - o.z];

        let mut home = HashMap::new();
        let robots = scene
            .robots
            .iter()
            .map(|r| {
                if let Some(pose) = &r.home_pose_deg {
                    home.entry(r.dressed_urdf.clone()).or_insert_with(|| pose.clone());
                }
                let lift = r.pedestal.as_ref().map_or(0.0, |p| p.height_mm / 1000.0);
                CellRobot {
                    id: r.robot_id.clone(),
                    model: r.dressed_urdf.clone(),
                    position: shift([r.position.x, r.position.y, r.position.z + lift]),
                    yaw_deg: r.orientation.a,
                }
            })
            .collect();
        let boxes = analysis.boxes();
        let mut products: Vec<SampledProduct> = boxes
            .iter()
            .map(|b| SampledProduct {
                name: b.name.clone(),
                center: shift(b.pose.translation.vector.into()),
                half: b.half,
                yaw_deg: b.pose.rotation.euler_angles().2.to_degrees(),
            })
            .collect();
        let welds = analysis
            .welds
            .iter()
            .map(|w| SampledWeld {
                position: shift(w.position),
                approach: w.approach,
                product: nearest_box(&boxes, w.position),
            })
            .collect::<Vec<_>>();
        // A cell without product boxes (an umarell cell: the program's welds only) gets one under
        // its welds, as the Robo Fitter draws its panel.
        if products.is_empty()
            && let Some(product) = box_under(&welds)
        {
            products.push(product);
        }
        Ok(Loaded {
            cell: TrainingCell { name: resolved.name.clone(), robots, products, welds: welds.len() },
            root: resolved.root,
            scene_base: resolved.scene_base,
            welds,
            home,
        })
    }

    /// Draw the shown cell: robots (slots), products, welds and the placement guides. A new scene
    /// if `new_scene`, else an edit of the current one.
    fn show(self: &Arc<Self>, inner: &mut Inner, new_scene: bool) {
        let (true, Some(shown)) = (inner.scene_open, inner.state.shown.clone()) else {
            return;
        };
        // Put back at the end: the slots and the state change meanwhile.
        let Some(loaded) = inner.loaded.take() else {
            return;
        };

        // Robots: robot `i` goes in slot `i`; a slot whose URDF changes is resolved again.
        let mut missing = Vec::new();
        let mut missing_models = Vec::new();
        let mut stale = Vec::new();
        let mut load = Vec::new();
        for (i, robot) in shown.robots.iter().enumerate() {
            let Some(urdf) = loaded.urdf(&robot.model) else {
                missing.push(i);
                missing_models.push(robot.model.clone());
                continue;
            };
            while inner.slots.len() <= i {
                inner.slots.push(Slot { model: String::new(), urdf: PathBuf::new(), loaded: None });
            }
            if inner.slots[i].urdf != urdf {
                if !inner.slots[i].urdf.as_os_str().is_empty() {
                    stale.push(scene::slot_id(i));
                }
                inner.slots[i] = Slot { model: robot.model.clone(), urdf: urdf.clone(), loaded: None };
                load.push((i, urdf));
            }
        }
        if !missing_models.is_empty() {
            missing_models.dedup();
            inner.state.error = Some(format!("Not in the cell's scene folder, not shown: {}", missing_models.join(", ")));
        }
        let slots: Vec<Option<SlotScene>> = inner
            .slots
            .iter()
            .map(|slot| {
                let m = slot.loaded.as_ref()?;
                Some(SlotScene { model: &m.model, joints: m.urdf.home_joint_map(loaded.home.get(&slot.model).map(Vec::as_slice)) })
            })
            .collect();

        // Where the bases may go (a drawn cell only).
        let guides = match &inner.job {
            Some(job) if shown.seed.is_some() => Some(sample::placement_guides(job, &shown.products, &loaded.cell)),
            _ => None,
        };
        let nodes = scene::cell_nodes(&shown, &slots, &missing, guides.as_deref());
        let view = scene::view(&loaded.cell.name);
        if new_scene {
            self.scene.set(nodes, view);
        } else {
            // A stale slot's old model goes; its pedestal comes back with `nodes`.
            let mut remove = stale;
            if guides.is_none() {
                remove.push(scene::PLACEMENT.to_owned());
            }
            // Robots that only move (a new sample of the same cell) go as poses.
            self.scene.update_moving(nodes, remove, Some(view));
        }
        inner.loaded = Some(loaded);
        for (i, urdf) in load {
            self.resolve_urdf(i, urdf);
        }
    }

    /// Parse slot `i`'s URDF and resolve its assets in the background, then show it.
    fn resolve_urdf(self: &Arc<Self>, i: usize, urdf: PathBuf) {
        let this = self.clone();
        std::thread::spawn(move || {
            let result = UrdfModel::load(&urdf).and_then(|model| Ok(SlotModel { model: this.assets.model_ref(&urdf)?, urdf: model }));
            let mut inner = this.inner.lock().unwrap();
            match result {
                Ok(model) => {
                    if let Some(slot) = inner.slots.get_mut(i)
                        && slot.urdf == urdf
                        && slot.loaded.is_none()
                    {
                        slot.loaded = Some(Arc::new(model));
                        this.show(&mut inner, false);
                    }
                }
                Err(err) => {
                    tracing::warn!("Training job: failed to load {}: {err:#}", urdf.display());
                    inner.state.error = Some(format!("Failed to load {}: {err:#}", urdf.display()));
                }
            }
        });
    }
}

/// A job for `cell`: round 4's distribution with the cell's robot models.
fn suggested_job(cell: &TrainingCell) -> TrainingJob {
    let mut models: Vec<String> = Vec::new();
    for robot in &cell.robots {
        if !models.contains(&robot.model) {
            models.push(robot.model.clone());
        }
    }
    let mut job = TrainingJob { name: format!("{}-train", cell.name), cell: cell.name.clone(), ..Default::default() };
    job.robots.models = models;
    job
}

fn saved_jobs(root: &Path) -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir(root.join(JOBS_DIR))
        .into_iter()
        .flatten()
        .flatten()
        .filter(|entry| entry.path().join(plan::JOB_FILE).is_file())
        .map(|entry| entry.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    names
}

/// A box standing on the floor under `welds`: their footprint plus a margin, its top just below
/// the lowest; `None` without welds or with welds too low.
fn box_under(welds: &[SampledWeld]) -> Option<SampledProduct> {
    const MARGIN_M: f64 = 0.08;
    const GAP_M: f64 = 0.015;
    let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
    for w in welds {
        for i in 0..3 {
            lo[i] = lo[i].min(w.position[i]);
            hi[i] = hi[i].max(w.position[i]);
        }
    }
    let top = lo[2] - GAP_M;
    (!welds.is_empty() && top > 0.05).then(|| SampledProduct {
        name: "box under the welds".to_owned(),
        center: [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, top / 2.0],
        half: [(hi[0] - lo[0]) / 2.0 + MARGIN_M, (hi[1] - lo[1]) / 2.0 + MARGIN_M, top / 2.0],
        yaw_deg: 0.0,
    })
}

fn nearest_box(boxes: &[&reach::task::ProductBox], p: [f64; 3]) -> usize {
    boxes
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| a.signed_distance(p).total_cmp(&b.signed_distance(p)))
        .map_or(0, |(i, _)| i)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_box_stands_under_the_welds() {
        let weld = |x: f64, y: f64, z: f64| SampledWeld { position: [x, y, z], approach: [0.0, 0.0, -1.0], product: 0 };
        let b = box_under(&[weld(0.0, 0.0, 1.0), weld(1.0, 0.5, 1.2)]).unwrap();
        assert!((b.center[2] - b.half[2]).abs() < 1e-9, "on the floor");
        assert!((2.0 * b.half[2] - 0.985).abs() < 1e-9, "top below the lowest weld");
        assert!((b.half[0] - 0.58).abs() < 1e-9 && (b.half[1] - 0.33).abs() < 1e-9);
        assert!(box_under(&[]).is_none());
        assert!(box_under(&[weld(0.0, 0.0, 0.03)]).is_none());
    }
}

// --- HTTP ---

pub fn router() -> Router<AppState> {
    Router::new()
        .route(api::TRAINING_OPEN, post(open))
        .route(api::TRAINING_PREVIEW, post(preview))
        .route(api::TRAINING_SAVE, post(save))
        .route(api::TRAINING_LOAD, post(load))
}

/// Run `action` off the async runtime (it reads the cell and resolves assets).
async fn blocking(state: &AppState, action: impl FnOnce(&Arc<TrainingService>) + Send + 'static) -> Json<TrainingState> {
    let training = state.training.clone();
    let training = tokio::task::spawn_blocking(move || {
        action(&training);
        training
    })
    .await
    .expect("training action panicked");
    Json(training.state())
}

async fn open(State(state): State<AppState>) -> Json<TrainingState> {
    blocking(&state, |t| t.open()).await
}

async fn preview(State(state): State<AppState>, Json(request): Json<PreviewRequest>) -> Json<TrainingState> {
    blocking(&state, move |t| t.preview(request)).await
}

async fn save(State(state): State<AppState>, Json(request): Json<SaveRequest>) -> Json<TrainingState> {
    blocking(&state, move |t| t.save(request)).await
}

async fn load(State(state): State<AppState>, Json(request): Json<LoadJobRequest>) -> Result<Json<JobFile>, (StatusCode, String)> {
    state.training.load(&request.name).map(Json).map_err(|err| (StatusCode::NOT_FOUND, format!("{err:#}")))
}
