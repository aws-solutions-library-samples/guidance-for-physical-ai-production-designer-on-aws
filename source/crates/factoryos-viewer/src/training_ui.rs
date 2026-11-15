//! Build training job: the panel beside its 3D view (see [`factoryos_client::protocol::training`]).
//! The backend (`services/training/`) reads the active cell, draws cells from the job and renders
//! them on `factoryos/scene/training`; this edits the job, picks what the 3D view shows (the loaded
//! cell, or the cell a seed draws: redrawn as the job changes) and saves the job as a plan.

use std::sync::{Arc, Mutex};

use factoryos_client::Api;
use factoryos_client::protocol::api;
use factoryos_client::protocol::keys;
use factoryos_client::protocol::training::{
    JobFile, LoadJobRequest, Placement, PreviewRequest, Range, SaveRequest, TrainingJob, TrainingState, WeldFaces, YawMode,
    valid_job_name,
};
use web_time::{Duration, Instant};

use crate::backend::Backend;
use crate::scene3d::RouteRequest;
use crate::scene3d::Scene3d;
use crate::rr::egui::{self, RichText};
use crate::rr::re_log_types::StoreId;
use crate::theme;

/// Edits coalesce for this long before the drawn cell is redrawn.
const REDRAW_AFTER: Duration = Duration::from_millis(150);
/// The robot counts offered (PPON4 takes up to 4; more for other planners).
const MAX_ROBOTS: u32 = 8;

pub struct TrainingUi {
    api: Api,
    pub stream: Scene3d,
    /// The backend's state, as last answered.
    state: Arc<Mutex<TrainingState>>,
    /// A saved job, loaded: adopted next frame.
    loaded_job: Arc<Mutex<Option<JobFile>>>,
    repaint: Arc<dyn Fn() + Send + Sync>,
    job: TrainingJob,
    /// The cell the job was adopted for (a new cell adopts its suggested job).
    job_cell: Option<String>,
    /// What the 3D view shows: `None` the loaded cell, else the cell this seed draws.
    view_seed: Option<u64>,
    /// What was last asked for, and when.
    sent: Option<(Option<TrainingJob>, Option<u64>)>,
    sent_at: Instant,
    /// The notes of the job last loaded (what the planner does not take yet).
    notes: Vec<String>,
}

impl TrainingUi {
    pub fn new(backend: &Backend) -> Self {
        Self {
            api: backend.api.clone(),
            stream: Scene3d::for_backend(backend, keys::SCENE_TRAINING, "training_job"),
            state: Arc::default(),
            loaded_job: Arc::default(),
            repaint: Arc::new(backend.repaint_handle()),
            job: TrainingJob::default(),
            job_cell: None,
            view_seed: None,
            sent: None,
            sent_at: Instant::now(),
            notes: Vec::new(),
        }
    }

    /// Read the active cell (again) and show it.
    pub fn open(&mut self) {
        self.view_seed = None;
        self.sent = Some((None, None));
        self.sent_at = Instant::now();
        self.post(api::TRAINING_OPEN, &serde_json::json!({}));
    }

    pub fn preview_recording(&self) -> Option<StoreId> {
        self.stream.current()
    }

    pub fn take_route_request(&mut self) -> Option<RouteRequest> {
        self.stream.take_route_request()
    }

    fn post(&self, path: &'static str, body: &impl serde::Serialize) {
        let (state, repaint) = (self.state.clone(), self.repaint.clone());
        self.api.post_json(path, body, move |response: Result<TrainingState, String>| {
            let mut state = state.lock().unwrap();
            match response {
                Ok(update) => *state = update,
                Err(err) => state.error = Some(format!("Failed: {err}")),
            }
            repaint();
        });
    }

    fn load_job(&self, name: &str) {
        let (state, loaded, repaint) = (self.state.clone(), self.loaded_job.clone(), self.repaint.clone());
        self.api.post_json(api::TRAINING_LOAD, &LoadJobRequest { name: name.to_owned() }, move |response: Result<JobFile, String>| {
            match response {
                Ok(file) => *loaded.lock().unwrap() = Some(file),
                Err(err) => state.lock().unwrap().error = Some(format!("Load failed: {err}")),
            }
            repaint();
        });
    }

    /// Adopt the cell's suggested job when a new cell opens, and a job just loaded.
    fn adopt(&mut self, state: &TrainingState) {
        if let Some(file) = self.loaded_job.lock().unwrap().take() {
            self.job = file.job;
            self.notes = file.notes;
            self.job_cell = state.cell.as_ref().map(|c| c.name.clone());
            return;
        }
        let cell = state.cell.as_ref().map(|c| c.name.clone());
        if cell.is_some() && cell != self.job_cell {
            if let Some(job) = &state.suggested {
                self.job = job.clone();
                self.notes.clear();
            }
            self.job_cell = cell;
        }
    }

    /// Ask for the cell the view should show, once edits have settled.
    fn redraw(&mut self, ctx: &egui::Context) {
        let wanted = (self.view_seed.map(|_| self.job.clone()), self.view_seed);
        if self.sent.as_ref() == Some(&wanted) {
            return;
        }
        let wait = REDRAW_AFTER.saturating_sub(self.sent_at.elapsed());
        if !wait.is_zero() {
            ctx.request_repaint_after(wait);
            return;
        }
        self.post(api::TRAINING_PREVIEW, &PreviewRequest { job: self.job.clone(), seed: self.view_seed });
        self.sent = Some(wanted);
        self.sent_at = Instant::now();
    }

    pub fn ui(&mut self, ui: &mut egui::Ui) {
        let state = self.state.lock().unwrap().clone();
        self.adopt(&state);

        let Some(cell) = state.cell.clone() else {
            ui.label(RichText::new(if state.status.is_empty() { "Reading the active cell…" } else { &state.status }).weak());
            if let Some(err) = &state.error {
                ui.label(RichText::new(err).color(theme::current().danger));
            }
            if ui.button("Read the active cell again").clicked() {
                self.open();
            }
            return;
        };

        ui.label(
            RichText::new(format!(
                "Cell {}: {} robot(s), {} product box(es), {} weld(s)",
                cell.name,
                cell.robots.len(),
                cell.products.len(),
                cell.welds
            ))
            .strong(),
        );
        ui.small(
            RichText::new(
                "Define the cells the policy trains on, around this one. The 3D view shows the loaded cell or a cell the \
                 job draws (geometry only: the planner's IK filter still rejects some).",
            )
            .weak(),
        );
        self.view_ui(ui, &state);
        ui.separator();

        egui::CollapsingHeader::new(RichText::new("Robots").strong()).id_salt("training_robots").default_open(true).show(ui, |ui| {
            self.robots_ui(ui, &cell.robots.iter().map(|r| r.model.clone()).collect::<Vec<_>>());
        });
        egui::CollapsingHeader::new(RichText::new("Products").strong()).id_salt("training_products").default_open(true).show(ui, |ui| {
            self.products_ui(ui, &cell);
        });
        egui::CollapsingHeader::new(RichText::new("Welds").strong()).id_salt("training_welds").default_open(true).show(ui, |ui| {
            self.welds_ui(ui);
        });
        // Scene counts and the training hyperparameters: Advanced only (saved with their values).
        crate::ui_mode::advanced(ui, |ui| {
            egui::CollapsingHeader::new(RichText::new("Scenes").strong()).id_salt("training_scenes").default_open(false).show(ui, |ui| {
                self.scenes_ui(ui);
            });
            egui::CollapsingHeader::new(RichText::new("Training").strong()).id_salt("training_run").default_open(false).show(ui, |ui| {
                self.training_ui(ui);
            });
        });
        ui.separator();
        self.save_ui(ui, &state);
        self.redraw(ui.ctx());
    }

    /// What the 3D view shows, and the drawn cell's notes.
    fn view_ui(&mut self, ui: &mut egui::Ui, state: &TrainingState) {
        ui.horizontal_wrapped(|ui| {
            if ui.add(crate::theme::segment(self.view_seed.is_none(), "Loaded cell")).on_hover_text("The cell as it is").clicked() {
                self.view_seed = None;
            }
            let mut drawn = self.view_seed.is_some();
            if ui.add(crate::theme::segment(drawn, "Drawn cell")).on_hover_text("A cell the job draws, for the seed").clicked() && !drawn {
                drawn = true;
                self.view_seed = Some(self.job.scenes.preview_seed);
            }
            if drawn {
                let mut seed = self.view_seed.unwrap_or_default();
                if ui.small_button("◀").on_hover_text("Previous seed").clicked() {
                    seed = seed.saturating_sub(1);
                }
                ui.add(egui::DragValue::new(&mut seed).prefix("seed "));
                if ui.small_button("▶").on_hover_text("Next seed").clicked() {
                    seed += 1;
                }
                if ui.small_button("🎲").on_hover_text("A random seed").clicked() {
                    let now = web_time::SystemTime::now().duration_since(web_time::UNIX_EPOCH).unwrap_or_default();
                    seed = (now.as_nanos() as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15) >> 44;
                }
                self.view_seed = Some(seed);
                self.job.scenes.preview_seed = seed;
            }
        });
        if let Some(shown) = &state.shown {
            let what = match shown.seed {
                Some(seed) => format!("Seed {seed}: {} robot(s), {} product(s), {} weld(s)", shown.robots.len(), shown.products.len(), shown.welds.len()),
                None => format!("Loaded cell: {} robot(s), {} weld(s)", shown.robots.len(), shown.welds.len()),
            };
            ui.small(RichText::new(what).weak());
            for note in &shown.notes {
                ui.small(RichText::new(format!("⚠ {note}")).color(theme::current().warn));
            }
        }
        if let Some(err) = &state.error {
            ui.small(RichText::new(err).color(theme::current().danger));
        }
    }

    fn robots_ui(&mut self, ui: &mut egui::Ui, cell_models: &[String]) {
        let r = &mut self.job.robots;
        ui.small(RichText::new("Models (robot i is model i mod n)").weak());
        let mut models: Vec<String> = Vec::new();
        for m in cell_models.iter().chain(&r.models) {
            if !models.contains(m) {
                models.push(m.clone());
            }
        }
        for model in &models {
            let mut on = r.models.contains(model);
            if ui.checkbox(&mut on, model_name(model)).on_hover_text(model).changed() {
                if on {
                    r.models.push(model.clone());
                } else {
                    r.models.retain(|m| m != model);
                }
            }
        }
        if r.models.is_empty() {
            ui.small(RichText::new("No model picked: the cell's models are used.").color(theme::current().warn));
        }

        ui.add_space(4.0);
        ui.small(RichText::new("Robots per cell: weight of each count").weak());
        let total: f64 = r.count_weights.values().sum();
        egui::Grid::new("training_counts").num_columns(3).show(ui, |ui| {
            for n in 1..=MAX_ROBOTS {
                let mut w = r.count_weights.get(&n).copied().unwrap_or(0.0);
                ui.label(format!("{n}"));
                if ui.add(egui::Slider::new(&mut w, 0.0..=1.0).fixed_decimals(2)).changed() {
                    if w > 0.0 {
                        r.count_weights.insert(n, w);
                    } else {
                        r.count_weights.remove(&n);
                    }
                }
                let share = if total > 0.0 { 100.0 * w / total } else { 0.0 };
                ui.small(RichText::new(format!("{share:.0} %")).weak());
                ui.end_row();
            }
        });
        if r.max_count() > 4 {
            ui.small(RichText::new("The round 4 policy (PPON4) takes at most 4 robots.").color(theme::current().warn));
        }
        ui.horizontal(|ui| {
            if ui.small_button("Round 4 mix").on_hover_text("1: 15 %, 2: 25 %, 3: 30 %, 4: 30 %").clicked() {
                r.count_weights = factoryos_client::protocol::training::RobotSpec::default().count_weights;
            }
            if ui.small_button("As the cell").on_hover_text("Always the cell's robot count").clicked() {
                r.count_weights = [(cell_models.len().max(1) as u32, 1.0)].into();
            }
        });

        ui.add_space(4.0);
        egui::Grid::new("training_placement").num_columns(2).show(ui, |ui| {
            ui.label("Placement");
            egui::ComboBox::from_id_salt("training_placement_mode")
                .selected_text(placement_label(r.placement))
                .show_ui(ui, |ui| {
                    for p in [Placement::RandomRing, Placement::Circle, Placement::CellJitter] {
                        ui.selectable_value(&mut r.placement, p, placement_label(p)).on_hover_text(placement_hover(p));
                    }
                })
                .response
                .on_hover_text(placement_hover(r.placement));
            ui.end_row();
            if r.placement != Placement::CellJitter {
                range_row(ui, "Distance from products", &mut r.ring_m, 0.0..=6.0, " m");
            }
            if r.placement != Placement::RandomRing {
                value_row(ui, "Base jitter", &mut r.jitter_m, 0.0..=2.0, " m");
            }
            value_row(ui, "Min base spacing", &mut r.min_spacing_m, 0.0..=5.0, " m");
            value_row(ui, "Base footprint radius", &mut r.footprint_r_m, 0.0..=2.0, " m");
            ui.label("Heading");
            egui::ComboBox::from_id_salt("training_yaw_mode")
                .selected_text(match r.yaw {
                    YawMode::Uniform => "any",
                    YawMode::Facing => "facing the product",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut r.yaw, YawMode::Uniform, "any");
                    ui.selectable_value(&mut r.yaw, YawMode::Facing, "facing the product");
                });
            ui.end_row();
            if r.yaw == YawMode::Facing || r.placement == Placement::CellJitter {
                value_row(ui, "Heading jitter ±", &mut r.yaw_jitter_deg, 0.0..=180.0, "°");
            }
            range_row(ui, "Pedestal height", &mut r.base_z_m, 0.0..=2.0, " m");
        });
    }

    fn products_ui(&mut self, ui: &mut egui::Ui, cell: &factoryos_client::protocol::training::TrainingCell) {
        let p = &mut self.job.products;
        ui.add_enabled(!cell.products.is_empty(), egui::Checkbox::new(&mut p.from_cell, "Keep the cell's products"))
            .on_hover_text("The loaded cell's product boxes as they are; only the robots and welds vary")
            .on_disabled_hover_text("The loaded cell has no product boxes");
        if p.from_cell && !cell.products.is_empty() {
            return;
        }
        egui::Grid::new("training_products_grid").num_columns(2).show(ui, |ui| {
            ui.label("Products per cell");
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut p.count[0]).range(1..=12));
                ui.label("–");
                ui.add(egui::DragValue::new(&mut p.count[1]).range(1..=12));
            });
            ui.end_row();
            ui.label("Shape");
            ui.horizontal(|ui| {
                ui.selectable_value(&mut p.square, true, "cube (square)");
                ui.selectable_value(&mut p.square, false, "box");
            });
            ui.end_row();
            range_row(ui, if p.square { "Side length" } else { "Footprint sides" }, &mut p.size_m, 0.05..=4.0, " m");
            range_row(ui, "Height", &mut p.height_m, 0.05..=3.0, " m");
            range_row(ui, "Turned", &mut p.yaw_deg, 0.0..=360.0, "°");
            ui.label("Layout disk radius");
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut p.layout_radius_m[0]).range(0.0..=10.0).speed(0.01).suffix(" m"));
                ui.label("+");
                ui.add(egui::DragValue::new(&mut p.layout_radius_m[1]).range(0.0..=5.0).speed(0.01).suffix(" m"));
                ui.label("× count");
            });
            ui.end_row();
            value_row(ui, "Min gap", &mut p.min_gap_m, 0.0..=3.0, " m");
        });
        p.count[1] = p.count[1].max(p.count[0]);
        if let Some(b) = cell.products.first()
            && ui
                .small_button("Around the cell's product")
                .on_hover_text(format!(
                    "Sizes ±25 % around {} ({:.2} × {:.2} × {:.2} m), one product",
                    b.name,
                    2.0 * b.half[0],
                    2.0 * b.half[1],
                    2.0 * b.half[2]
                ))
                .clicked()
        {
            let (lo, hi) = (2.0 * b.half[0].min(b.half[1]), 2.0 * b.half[0].max(b.half[1]));
            p.size_m = [lo * 0.75, hi * 1.25];
            p.height_m = [2.0 * b.half[2] * 0.75, 2.0 * b.half[2] * 1.25];
            p.count = [1, 1];
        }
    }

    fn welds_ui(&mut self, ui: &mut egui::Ui) {
        let w = &mut self.job.welds;
        egui::Grid::new("training_welds_grid").num_columns(2).show(ui, |ui| {
            ui.label("Welds per cell");
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut w.total[0]).range(1..=200));
                ui.label("–");
                ui.add(egui::DragValue::new(&mut w.total[1]).range(1..=200));
            });
            ui.end_row();
            ui.label("At most per robot");
            ui.add(egui::DragValue::new(&mut w.per_robot_max).range(1..=50));
            ui.end_row();
            ui.label("Per product");
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut w.per_product[0]).range(0..=50));
                ui.label("–");
                ui.add(egui::DragValue::new(&mut w.per_product[1]).range(0..=50));
            });
            ui.end_row();
            ui.label("On");
            ui.horizontal(|ui| {
                ui.selectable_value(&mut w.faces, WeldFaces::Top, "top");
                ui.selectable_value(&mut w.faces, WeldFaces::TopAndSides, "top and sides");
            });
            ui.end_row();
            value_row(ui, "Edge inset", &mut w.edge_inset_m, 0.0..=0.5, " m");
            value_row(ui, "Min spacing", &mut w.min_spacing_m, 0.0..=0.5, " m");
            range_row(ui, "Approach tilt", &mut w.tilt_deg, 0.0..=60.0, "°");
        });
        w.total[1] = w.total[1].max(w.total[0]);
        w.per_product[1] = w.per_product[1].max(w.per_product[0]);
        ui.small(RichText::new("The count is kept within [robots + 1, per robot × robots].").weak());
    }

    fn scenes_ui(&mut self, ui: &mut egui::Ui) {
        let s = &mut self.job.scenes;
        egui::Grid::new("training_scenes_grid").num_columns(2).show(ui, |ui| {
            ui.label("Train scenes");
            ui.add(egui::DragValue::new(&mut s.train).range(1..=1_000_000).speed(100));
            ui.end_row();
            ui.label("First train seed");
            ui.add(egui::DragValue::new(&mut s.train_offset).range(0..=9_999_999).speed(100));
            ui.end_row();
            ui.label("Selection scenes");
            ui.add(egui::DragValue::new(&mut s.selection).range(0..=10_000));
            ui.end_row();
            ui.label("Test scenes");
            ui.add(egui::DragValue::new(&mut s.test).range(0..=10_000));
            ui.end_row();
        });
        ui.checkbox(&mut s.ood, "Out-of-distribution families (many products, tight, tilt, combo)");
    }

    fn training_ui(&mut self, ui: &mut egui::Ui) {
        let t = &mut self.job.training;
        ui.small(RichText::new("Round 4 recipe (PPON4): PPO fine-tuning from a checkpoint, BC steps optional.").weak());
        egui::Grid::new("training_run_grid").num_columns(2).show(ui, |ui| {
            ui.label("Start from");
            ui.add(egui::TextEdit::singleline(&mut t.init).hint_text("checkpoint (empty: from scratch)"));
            ui.end_row();
            ui.label("Timing profile");
            egui::ComboBox::from_id_salt("training_profile").selected_text(t.profile.clone()).show_ui(ui, |ui| {
                for profile in ["umarell", "full"] {
                    ui.selectable_value(&mut t.profile, profile.to_owned(), profile);
                }
            });
            ui.end_row();
            int_row(ui, "BC steps", &mut t.bc_steps, 0..=100_000);
            int_row(ui, "PPO iterations", &mut t.ppo_iters, 1..=10_000);
            int_row(ui, "Batch", &mut t.batch, 64..=1_000_000);
            int_row(ui, "Minibatch", &mut t.minibatch, 16..=1_000_000);
            int_row(ui, "Epochs", &mut t.epochs, 1..=50);
            float_row(ui, "Learning rate", &mut t.lr, 1e-7..=1e-1, 6);
            float_row(ui, "Clip", &mut t.clip, 0.01..=1.0, 3);
            float_row(ui, "Entropy", &mut t.ent, 0.0..=0.1, 5);
            float_row(ui, "Gamma (per s)", &mut t.gamma, 0.5..=1.0, 4);
            float_row(ui, "GAE λ", &mut t.lam, 0.5..=1.0, 3);
            value_row(ui, "Horizon", &mut t.horizon_s, 1.0..=600.0, " s");
            int_row(ui, "Scene pool / actor", &mut t.pool, 1..=64);
            int_row(ui, "Episodes per swap", &mut t.rotate, 1..=1000);
            int_row(ui, "Actor processes", &mut t.actors, 1..=100_000);
        });
        ui.small(RichText::new("Rewards").weak());
        let rw = &mut t.rewards;
        egui::Grid::new("training_rewards_grid").num_columns(2).show(ui, |ui| {
            float_row(ui, "Time penalty (per s)", &mut rw.time_penalty, 0.0..=100.0, 2);
            float_row(ui, "Weld", &mut rw.weld, -100.0..=100.0, 2);
            float_row(ui, "Success", &mut rw.success, -100.0..=1000.0, 2);
            float_row(ui, "Timeout", &mut rw.timeout, -1000.0..=0.0, 2);
            float_row(ui, "Deadlock", &mut rw.deadlock, -1000.0..=0.0, 2);
        });
    }

    fn save_ui(&mut self, ui: &mut egui::Ui, state: &TrainingState) {
        ui.label(RichText::new("Save as plan").strong());
        ui.horizontal(|ui| {
            ui.label("Name");
            ui.add(egui::TextEdit::singleline(&mut self.job.name).desired_width(f32::INFINITY));
        });
        ui.add(egui::TextEdit::multiline(&mut self.job.description).hint_text("What this job is for (optional)").desired_rows(2));
        let valid = valid_job_name(&self.job.name);
        let exists = state.saved_jobs.contains(&self.job.name);
        let label = if exists { "Save plan (replaces the saved job)" } else { "Save plan" };
        if crate::sidebar::button_response(ui, label, valid)
            .on_hover_text(format!(
                "Writes {}/training_jobs/{}/: plan.json (scenegen's batch plan), scenegen_config.json (the distribution) and \
                 job.json (this job and the commands that train it)",
                self.job_cell.as_deref().unwrap_or("<cell>"),
                self.job.name
            ))
            .on_disabled_hover_text("A name is [A-Za-z0-9._-]+, up to 64 characters")
            .clicked()
        {
            self.job.cell = self.job_cell.clone().unwrap_or_default();
            self.post(api::TRAINING_SAVE, &SaveRequest { job: self.job.clone() });
        }
        if let Some(dir) = &state.saved_to {
            ui.small(RichText::new(format!("Saved in {dir}")).color(theme::current().ok));
        }
        let notes = if state.saved_to.is_some() { &state.saved_notes } else { &self.notes };
        for note in notes {
            ui.small(RichText::new(format!("ⓘ {note}")).weak());
        }
        if !state.saved_jobs.is_empty() {
            ui.add_space(4.0);
            ui.small(RichText::new("Saved jobs of this cell").weak());
            for name in &state.saved_jobs {
                ui.horizontal(|ui| {
                    ui.label(name);
                    if ui.small_button("Load").on_hover_text("Edit this job (replaces the one in the panel)").clicked() {
                        self.load_job(name);
                    }
                });
            }
        }
        if ui.small_button("Read the active cell again").on_hover_text("After switching or editing the cell").clicked() {
            self.job_cell = None;
            self.open();
        }
    }
}

fn model_name(urdf: &str) -> String {
    let file = urdf.rsplit('/').next().unwrap_or(urdf);
    file.strip_suffix(".urdf").unwrap_or(file).to_owned()
}

fn placement_label(p: Placement) -> &'static str {
    match p {
        Placement::RandomRing => "random ring",
        Placement::Circle => "even circle",
        Placement::CellJitter => "cell layout ± jitter",
    }
}

fn placement_hover(p: Placement) -> &'static str {
    match p {
        Placement::RandomRing => "scenegen's (round 4): each base at a random distance from a random product, clear of the others",
        Placement::Circle => "Evenly on a circle around the products, turned at random, each base jittered",
        Placement::CellJitter => "The loaded cell's bases, each moved and turned a little",
    }
}

fn range_row(ui: &mut egui::Ui, label: &str, value: &mut Range, range: std::ops::RangeInclusive<f64>, suffix: &str) {
    ui.label(label);
    let speed = ((range.end() - range.start()) / 300.0).max(0.001);
    ui.horizontal(|ui| {
        ui.add(egui::DragValue::new(&mut value[0]).range(range.clone()).speed(speed).max_decimals(3).suffix(suffix));
        ui.label("–");
        ui.add(egui::DragValue::new(&mut value[1]).range(range).speed(speed).max_decimals(3).suffix(suffix));
    });
    value[1] = value[1].max(value[0]);
    ui.end_row();
}

fn value_row(ui: &mut egui::Ui, label: &str, value: &mut f64, range: std::ops::RangeInclusive<f64>, suffix: &str) {
    ui.label(label);
    let speed = ((range.end() - range.start()) / 300.0).max(0.001);
    ui.add(egui::DragValue::new(value).range(range).speed(speed).max_decimals(3).suffix(suffix));
    ui.end_row();
}

fn float_row(ui: &mut egui::Ui, label: &str, value: &mut f64, range: std::ops::RangeInclusive<f64>, decimals: usize) {
    ui.label(label);
    let speed = 10f64.powi(-(decimals as i32));
    ui.add(egui::DragValue::new(value).range(range).speed(speed).max_decimals(decimals));
    ui.end_row();
}

fn int_row(ui: &mut egui::Ui, label: &str, value: &mut u32, range: std::ops::RangeInclusive<u32>) {
    ui.label(label);
    let speed = (f64::from(*range.end() - *range.start()) / 500.0).max(1.0);
    ui.add(egui::DragValue::new(value).range(range).speed(speed));
    ui.end_row();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn model_names_are_the_urdf_stems() {
        assert_eq!(model_name("../urdf-dressed-robots/kr240__gun/kr240__gun.urdf"), "kr240__gun");
        assert_eq!(model_name("plain"), "plain");
    }
}
