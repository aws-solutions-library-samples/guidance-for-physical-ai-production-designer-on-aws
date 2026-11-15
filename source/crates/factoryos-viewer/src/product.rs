//! Product & Task, the first step of the flow: the part to produce and what to do to it, before
//! any cell exists. A product has tasks; each is one process (spot welding, seam welding, gluing,
//! …, see `factoryos_protocol::product::PROCESSES`) done with one tool, with that process's
//! parameters, and its operations are points or paths on the part.
//!
//! Products are stored by the backend (`services/product.rs`), which also renders the 3D preview.
//! The cells a product is placed in get their task programs from its tasks (the backend derives
//! `agentic_planner/tasks/<fixture>__<task>.json`, and re-derives them when the product is
//! saved): a task's welds (id, group, position and normal, per-operation priority), its tool and
//! process parameters, and its planning strategy (`STRATEGY` keys in the task's `params`).
//!
//! The catalogue's ready-made products are loaded by copying one into the products folder, and a
//! saved product (its tasks too) is saved to the catalogue for a next run. A product's geometry can
//! be a URDF: a zip uploaded into its `urdf/` folder, or one of the catalogue's fixtures copied there.

use std::sync::{Arc, Mutex};
use web_time::{Duration, Instant};

use factoryos_client::protocol::dresser::{DresserState, OpenRequest};
use factoryos_client::protocol::product::{
    CatalogueProducts, CatalogueTransfer, Geometry, LoadProduct, Operation, PROCESSES, Params, PreviewProduct, Process, Product,
    ProductFile, ProductUrdfs, ProductsInfo, Target, TargetKind, Task, UrdfFromCatalogue, process, valid_name,
};
use factoryos_client::protocol::product_edit::{FACES, FacePattern, face_outline, grid_points, next_id};
use factoryos_client::protocol::{api, keys};
use factoryos_client::Api;
use crate::theme;
use crate::rr::egui::{self, RichText};
use crate::rr::re_log_types::StoreId;

use crate::backend::Backend;
use crate::scene3d::RouteRequest;
use crate::scene3d::Scene3d;

/// The Rerun application the previews are recordings of.
const APP_ID: &str = "product_task";

/// Edits are previewed once they have settled for this long.
const PREVIEW_DEBOUNCE: Duration = Duration::from_millis(250);


/// The planning strategy of a task, stored in its `params` next to the process parameters; the
/// backend writes them into the derived task program's `extensions.strategy` / `cycle_time`.
pub const SEQUENCE_GROUPS_ORDER: &str = "sequence_groups_order";
pub const MAX_CONCURRENT_ROBOTS: &str = "max_concurrent_robots";
pub const PARALLEL_WITHIN_GROUP: &str = "parallel_within_group";
pub const CYCLE_TIME_TARGET_S: &str = "cycle_time_target_s";
pub const CYCLE_TIME_MAX_S: &str = "cycle_time_max_s";
const STRATEGY: [&str; 5] =
    [SEQUENCE_GROUPS_ORDER, MAX_CONCURRENT_ROBOTS, PARALLEL_WITHIN_GROUP, CYCLE_TIME_TARGET_S, CYCLE_TIME_MAX_S];
/// An operation's priority, in its `params` (the task program operation's `priority`).
pub const PRIORITY: &str = "priority";

/// What the pane asks the app to do.
pub enum ProductAction {
    /// Ask the agent for the task description of the current product.
    TaskDescription,
}

enum Reply {
    Listed(ProductsInfo),
    Loaded(ProductFile),
    Saved(ProductFile),
    Failed(String),
    Previewed(Result<(), String>),
    Catalogue(CatalogueProducts),
    Fixtures(Vec<String>),
    /// A catalogue product copied into the products folder.
    FromCatalogue(ProductFile),
    /// The product saved to the catalogue (its listing now).
    Published(String, CatalogueProducts),
    /// The product exists already where it was to be copied to: ask before replacing it.
    Exists(Transfer),
    /// A zip was picked and is being uploaded.
    Uploading,
    /// The URDFs in the product's `urdf/` folder, after an upload or a fixture copied.
    Urdfs(ProductUrdfs),
}

/// A copy to or from the catalogue, of the product named.
#[derive(Clone, Debug, PartialEq)]
enum Transfer {
    Load(String),
    Publish(String),
}

/// The generators' settings: a grid of points, or a face's outline as a path.
type Generator = FacePattern;

pub struct ProductPane {
    api: Api,
    pub stream: Scene3d,
    replies: Arc<Mutex<Vec<Reply>>>,
    /// The Robo Dresser catalogue's tools, offered for the tasks.
    catalogue_tools: Arc<Mutex<Vec<String>>>,
    listed: Option<ProductsInfo>,
    /// A listing was asked for and hasn't come back yet.
    listing: bool,
    product: Product,
    /// The product as last loaded or saved, and its file, to tell unsaved edits apart.
    saved: Option<(Product, String)>,
    status: Option<Result<String, String>>,
    preview_error: Option<String>,
    /// What the 3D view shows, and when the product last changed.
    previewed: Option<Product>,
    /// A preview was sent and its reply has not come back yet.
    preview_in_flight: bool,
    changed_at: Instant,
    generator: Generator,
    /// The process "Add task" adds.
    new_process: usize,
    /// The task to open (and scroll to) once its product is loaded: a task program's Edit.
    focus_task: Option<String>,
    /// The catalogue's products, and its fixtures (for a URDF product).
    catalogue: Option<CatalogueProducts>,
    fixtures: Vec<String>,
    /// A copy that would replace a product: asked about before it is done.
    confirm: Option<Transfer>,
    /// "Save to catalogue" with unsaved changes: saved first, then put into the catalogue.
    publish_after_save: bool,
    /// The URDFs found in the product's `urdf/` folder by the last upload or fixture copy.
    urdfs: Vec<String>,
    /// An upload or copy into `urdf/` is under way.
    urdf_busy: bool,
    /// "Publish to shared catalogue" (catalogue admins, S3).
    publish: crate::shared_publish::SharedPublish,
    /// The catalogue changes seen (crate::catalogue_events).
    catalogue_seen: u64,
}

impl ProductPane {
    pub fn new(backend: &Backend) -> Self {
        let catalogue_tools = Arc::new(Mutex::new(Vec::new()));
        let sink = catalogue_tools.clone();
        let tools_of = move |payload: &[u8]| {
            if let Ok(state) = serde_json::from_slice::<DresserState>(payload) {
                *sink.lock().unwrap() = state.tools.into_iter().map(|tool| tool.name).collect();
            }
        };
        // Also queried now and after a re-join.
        backend.bus.follow(keys::DRESSER_STATE, move |payload| tools_of(&payload)).ok();
        Self {
            api: backend.api.clone(),
            stream: Scene3d::for_backend(backend, keys::SCENE_PRODUCT, APP_ID),
            replies: Arc::default(),
            catalogue_tools,
            listed: None,
            listing: false,
            product: new_product(),
            saved: None,
            status: None,
            preview_error: None,
            previewed: None,
            preview_in_flight: false,
            changed_at: Instant::now(),
            generator: Generator {
                face: 0,
                rows: 2,
                columns: 3,
                margin: 0.05,
                group: "structural".to_owned(),
            },
            new_process: 0,
            focus_task: None,
            catalogue: None,
            fixtures: Vec::new(),
            confirm: None,
            publish_after_save: false,
            urdfs: Vec::new(),
            urdf_busy: false,
            publish: crate::shared_publish::SharedPublish::new(backend.api.clone()),
            catalogue_seen: crate::catalogue_events::generation(),
        }
    }

    /// The tool was opened: list the products and the catalogue's tools, and show the current product.
    pub fn open(&mut self) {
        self.refresh();
        // The dresser scans its catalogue on first use; the tools come with its state.
        self.api
            .post_json(api::DRESSER_OPEN, &OpenRequest { rescan: false }, |_: Result<DresserState, String>| {});
        self.previewed = None;
    }

    /// The tool was opened on `product` (a product-derived task program's Edit): load it, and
    /// open its task `task`, if named.
    pub fn open_on(&mut self, product: &str, task: Option<String>) {
        self.open();
        if self.saved.as_ref().is_none_or(|(saved, _)| saved.name != product) || !self.unsaved() {
            self.load(product);
        }
        self.focus_task = task;
    }

    /// The product the steps' agents work on: the one last loaded or saved, its name and its
    /// `product.json` on the backend host.
    pub fn current(&self) -> Option<(String, String)> {
        self.saved.as_ref().map(|(product, file)| (product.name.clone(), file.clone()))
    }

    pub fn preview_recording(&self) -> Option<StoreId> {
        self.stream.current()
    }

    pub fn take_route_request(&mut self) -> Option<RouteRequest> {
        self.stream.take_route_request()
    }

    /// Load `name` from the product catalogue, as a click on its row in the Catalogue section does
    /// (asking first, in the panel, if the products folder has it already).
    pub fn load_from_catalogue(&self, name: &str) {
        self.transfer(Transfer::Load(name.to_owned()), false);
    }

    /// Why the 3D view may not show the product as edited yet: replies to apply, a preview not
    /// sent yet (edits settling) or not answered yet. `None` once it does (automation).
    pub fn unsettled(&self) -> Option<&'static str> {
        if !self.replies.lock().unwrap().is_empty() {
            Some("product replies to apply")
        } else if self.previewed.as_ref() != Some(&self.product) {
            Some("product preview not sent yet")
        } else if self.preview_in_flight {
            Some("product preview in flight")
        } else {
            None
        }
    }

    /// The product being edited (its name).
    pub fn product_name(&self) -> &str {
        &self.product.name
    }

    fn refresh(&mut self) {
        self.listing = true;
        let replies = self.replies.clone();
        self.api.get(api::PRODUCT_LIST, move |r: Result<ProductsInfo, String>| {
            replies.lock().unwrap().push(r.map_or_else(Reply::Failed, Reply::Listed));
        });
        let replies = self.replies.clone();
        self.api.get(api::PRODUCT_CATALOGUE, move |r: Result<CatalogueProducts, String>| {
            replies.lock().unwrap().push(r.map_or_else(Reply::Failed, Reply::Catalogue));
        });
        let replies = self.replies.clone();
        self.api.get(api::PRODUCT_FIXTURES, move |r: Result<Vec<String>, String>| {
            // An older backend has none: no fixtures to offer.
            replies.lock().unwrap().push(Reply::Fixtures(r.unwrap_or_default()));
        });
    }

    /// Copy to or from the catalogue; `overwrite`: replace the product there.
    fn transfer(&self, transfer: Transfer, overwrite: bool) {
        let replies = self.replies.clone();
        let (path, name) = match &transfer {
            Transfer::Load(name) => (api::PRODUCT_CATALOGUE_LOAD, name.clone()),
            Transfer::Publish(name) => (api::PRODUCT_CATALOGUE_PUBLISH, name.clone()),
        };
        let request = CatalogueTransfer { name: name.clone(), overwrite };
        let conflict = move |err: String| if err.starts_with("409") { Reply::Exists(transfer) } else { Reply::Failed(err) };
        match path {
            api::PRODUCT_CATALOGUE_LOAD => self.api.post_json(path, &request, move |r: Result<ProductFile, String>| {
                replies.lock().unwrap().push(r.map_or_else(conflict, Reply::FromCatalogue));
            }),
            _ => self.api.post_json(path, &request, move |r: Result<CatalogueProducts, String>| {
                replies.lock().unwrap().push(r.map_or_else(conflict, |listed| Reply::Published(name, listed)));
            }),
        }
    }

    /// Save to the catalogue: the product as saved, so unsaved changes are saved first.
    fn publish(&mut self) {
        if self.unsaved() {
            self.publish_after_save = true;
            self.save();
        } else {
            self.transfer(Transfer::Publish(self.product.name.clone()), false);
        }
    }

    fn urdf_reply(replies: Arc<Mutex<Vec<Reply>>>) -> impl FnOnce(Result<ProductUrdfs, String>) + Send + 'static {
        move |r| replies.lock().unwrap().push(r.map_or_else(Reply::Failed, Reply::Urdfs))
    }

    /// Pick a zip (a URDF and its meshes) and upload it into the product's `urdf/` folder.
    fn upload_urdf(&mut self) {
        let (api, replies, name) = (self.api.clone(), self.replies.clone(), self.product.name.clone());
        // A cancelled dialog sends nothing.
        crate::ide_transfer::pick_zip("Upload the product's URDF (zip)", move |picked| match picked {
            Ok((_, bytes)) => {
                replies.lock().unwrap().push(Reply::Uploading);
                api.put_bytes(&api::product_urdf_upload(&name), bytes, Self::urdf_reply(replies));
            }
            Err(err) => replies.lock().unwrap().push(Reply::Failed(err)),
        });
    }

    fn urdf_from_fixture(&mut self, fixture: &str) {
        self.urdf_busy = true;
        let request = UrdfFromCatalogue { product: self.product.name.clone(), fixture: fixture.to_owned() };
        self.api.post_json(api::PRODUCT_URDF_FROM_CATALOGUE, &request, Self::urdf_reply(self.replies.clone()));
    }

    fn load(&self, name: &str) {
        let replies = self.replies.clone();
        let request = LoadProduct { name: name.to_owned() };
        self.api.post_json(api::PRODUCT_LOAD, &request, move |r: Result<ProductFile, String>| {
            replies.lock().unwrap().push(r.map_or_else(Reply::Failed, Reply::Loaded));
        });
    }

    fn save(&self) {
        let replies = self.replies.clone();
        self.api.post_json(api::PRODUCT_SAVE, &self.product, move |r: Result<ProductFile, String>| {
            replies.lock().unwrap().push(r.map_or_else(Reply::Failed, Reply::Saved));
        });
    }

    /// Apply replies, and preview the product once edits have settled. Call once per frame
    /// while the tool is shown.
    pub fn poll(&mut self) {
        let replies = std::mem::take(&mut *self.replies.lock().unwrap());
        for reply in replies {
            match reply {
                Reply::Listed(listed) => {
                    self.listed = Some(listed);
                    self.listing = false;
                }
                Reply::Loaded(file) => {
                    self.product = file.product.clone();
                    self.saved = Some((file.product, file.path));
                    self.status = None;
                    self.changed_at = Instant::now();
                }
                Reply::Saved(file) => {
                    self.status = Some(Ok(format!("Saved {}: the cells using it are updated.", file.path)));
                    if std::mem::take(&mut self.publish_after_save) {
                        self.transfer(Transfer::Publish(file.product.name.clone()), false);
                    }
                    self.saved = Some((file.product, file.path));
                    self.refresh();
                }
                Reply::Failed(err) => {
                    self.listing = false;
                    self.publish_after_save = false;
                    self.urdf_busy = false;
                    self.status = Some(Err(explain(err)));
                }
                Reply::Previewed(result) => {
                    self.preview_in_flight = false;
                    self.preview_error = result.err().map(explain);
                }
                Reply::Catalogue(catalogue) => self.catalogue = Some(catalogue),
                Reply::Fixtures(fixtures) => self.fixtures = fixtures,
                Reply::FromCatalogue(file) => {
                    self.status = Some(Ok(format!("Copied {} from the catalogue to {}.", file.product.name, file.path)));
                    self.product = file.product.clone();
                    self.saved = Some((file.product, file.path));
                    self.urdfs.clear();
                    // Its URDF may be new on disk under an unchanged path: draw it again.
                    self.previewed = None;
                    self.changed_at = Instant::now();
                    self.refresh();
                }
                Reply::Published(name, catalogue) => {
                    self.status = Some(Ok(format!("Saved {name} to the catalogue, {}.", catalogue.dir)));
                    self.catalogue = Some(catalogue);
                }
                Reply::Exists(transfer) => self.confirm = Some(transfer),
                Reply::Uploading => self.urdf_busy = true,
                Reply::Urdfs(found) => {
                    self.urdf_busy = false;
                    if found.urdfs.is_empty() {
                        continue;
                    }
                    if let Geometry::Urdf { path } = &mut self.product.geometry
                        && (found.urdfs.len() == 1 || !found.urdfs.contains(path))
                    {
                        *path = found.urdfs[0].clone();
                    }
                    self.status = Some(Ok(format!("{} URDF file(s) in the product's urdf/ folder.", found.urdfs.len())));
                    self.urdfs = found.urdfs;
                    self.previewed = None;
                    self.changed_at = Instant::now();
                }
            }
        }
        // The page can come back without "Open Product & Task" (e.g. following the backend's
        // preview after a reconnect): list the products then too.
        if self.listed.is_none() && !self.listing {
            self.refresh();
        }
        if self.previewed.as_ref() != Some(&self.product) && self.changed_at.elapsed() >= PREVIEW_DEBOUNCE {
            let mut product = self.product.clone();
            if product.name.trim().is_empty() {
                product.name = "unnamed".to_owned();
            }
            let replies = self.replies.clone();
            self.api.post_json(api::PRODUCT_PREVIEW, &PreviewProduct { product }, move |r: Result<(), String>| {
                replies.lock().unwrap().push(Reply::Previewed(r));
            });
            self.previewed = Some(self.product.clone());
            self.preview_in_flight = true;
        }
    }

    fn unsaved(&self) -> bool {
        self.saved.as_ref().is_none_or(|(saved, _)| *saved != self.product)
    }

    // ------------------------------------------------------------------------
    // UI

    /// The pane's main actions, pinned to its bottom ([`theme::action_row`]): Save, and (Advanced)
    /// Save to catalogue and Publish to the shared catalogue; their outcome and whether there are
    /// unsaved changes above them.
    pub fn actions_ui(&mut self, ui: &mut egui::Ui) {
        self.publish.status_ui(ui);
        match &self.status {
            Some(Ok(text)) => {
                ui.label(RichText::new(text).small().color(theme::current().ok));
            }
            Some(Err(text)) => {
                ui.label(RichText::new(text).small().color(theme::current().danger));
            }
            None => {}
        }
        theme::action_buttons(ui, |ui| {
            if ui
                .add_enabled(valid_name(&self.product.name), theme::primary(ui, "Save"))
                .on_disabled_hover_text("Name: letters, digits, _ and - only")
                .clicked()
            {
                self.save();
            }
            if crate::ui_mode::is_advanced(ui)
                && ui
                .add_enabled(valid_name(&self.product.name), egui::Button::new("Save to catalogue"))
                .on_hover_text("Save it, then copy it (tasks, URDF and meshes) into the catalogue, to load it again in a next run")
                .clicked()
            {
                self.publish();
            }
            if crate::ui_mode::is_advanced(ui) {
                // The saved product's folder into the shared catalogue's product collection.
                let name = self.product.name.clone();
                let saved = valid_name(&name) && self.saved.is_some() && !self.unsaved();
                self.publish.button_ui(ui, saved, &format!("products/{name}/"), &format!("product-collection/{name}/"));
            }
            if self.unsaved() {
                ui.small(RichText::new("unsaved changes").color(theme::current().warn));
            }
        });
    }

    pub fn ui(&mut self, ui: &mut egui::Ui) -> Option<ProductAction> {
        let before = self.product.clone();
        let mut action = None;

        section(ui, "Catalogue");
        if crate::catalogue_events::changed_since(&mut self.catalogue_seen) {
            self.refresh();
        }
        crate::catalogue_events::notice_ui(ui);
        self.catalogue_ui(ui);
        section(ui, "Product");
        self.product_ui(ui);
        section(ui, "Geometry");
        geometry_ui(ui, &mut self.product.geometry);
        if matches!(self.product.geometry, Geometry::Urdf { .. }) {
            self.urdf_ui(ui);
        }
        if let Some(err) = &self.preview_error {
            ui.label(RichText::new(err).small().color(theme::current().danger));
        }

        section(ui, "Tasks");
        ui.small(RichText::new("Each task is one process done with one tool; its operations are points or paths on the part.").weak());
        self.tasks_ui(ui);

        // Save, Save to catalogue and Publish are in the pane's action row (`actions_ui`).
        ui.add_space(12.0);
        ui.separator();
        ui.label(RichText::new("AGENT").small().strong());
        ui.small(
            RichText::new(
                "Works the tasks out into operations (tool orientation, approach and retract, process steps, order) \
                 in the product's own frame, as task.json next to the product. Placing the part and the robots \
                 comes later, in the Robo Fitter.",
            )
            .weak(),
        );
        let saved = !self.unsaved();
        let icon = crate::rr::re_ui::icons::AGENT.as_image();
        if ui
            .add_enabled(saved, egui::Button::image_and_text(icon, "Create task description"))
            .on_disabled_hover_text("Save the product first")
            .clicked()
        {
            action = Some(ProductAction::TaskDescription);
        }

        if self.product != before {
            self.changed_at = Instant::now();
        }
        action
    }

    fn catalogue_ui(&mut self, ui: &mut egui::Ui) {
        let Some(catalogue) = self.catalogue.clone() else {
            ui.weak("Listing…");
            return;
        };
        ui.horizontal(|ui| {
            crate::ui_mode::advanced(ui, |ui| ui.small(RichText::new(&catalogue.dir).weak().monospace()));
            if ui.small_button("⟳").on_hover_text("Re-list the catalogue").clicked() {
                self.refresh();
            }
        });
        if let Some(err) = &catalogue.error {
            ui.label(RichText::new(err).small().color(theme::current().danger));
        }
        if catalogue.products.is_empty() {
            ui.small(RichText::new("No ready-made products yet: Save to catalogue puts one here.").weak());
        }
        for product in &catalogue.products {
            let text = format!("{} · {} · {} task{}", product.name, product.geometry, product.tasks, if product.tasks == 1 { "" } else { "s" });
            let row = ui.add(egui::Button::selectable(false, text));
            let row = if product.description.is_empty() { row } else { row.on_hover_text(&product.description) };
            if row.clicked() {
                self.transfer(Transfer::Load(product.name.clone()), false);
            }
        }
        if let Some(transfer) = self.confirm.clone() {
            let question = match &transfer {
                Transfer::Load(name) => format!("{name} is in the products folder already: replace it with the catalogue's?"),
                Transfer::Publish(name) => format!("{name} is in the catalogue already: replace it?"),
            };
            ui.label(RichText::new(question).small().color(theme::current().warn));
            ui.horizontal(|ui| {
                if ui.button("Replace").clicked() {
                    self.transfer(transfer, true);
                    self.confirm = None;
                }
                if ui.button("Cancel").clicked() {
                    self.confirm = None;
                }
            });
        }
    }

    /// A URDF product's sources: an uploaded zip or a catalogue fixture, and which URDF to use.
    fn urdf_ui(&mut self, ui: &mut egui::Ui) {
        let named = valid_name(&self.product.name);
        ui.horizontal(|ui| {
            if ui
                .add_enabled(named && !self.urdf_busy, egui::Button::new("Upload zip…"))
                .on_hover_text("A zip of the URDF and its meshes, extracted into the product's urdf/ folder (replacing it)")
                .on_disabled_hover_text("Name the product first")
                .clicked()
            {
                self.upload_urdf();
            }
            let mut fixture = None;
            ui.add_enabled_ui(named && !self.urdf_busy, |ui| {
                egui::ComboBox::from_id_salt("product_fixture").selected_text("from the catalogue").show_ui(ui, |ui| {
                    if self.fixtures.is_empty() {
                        ui.weak("No fixtures in the catalogue");
                    }
                    for name in &self.fixtures {
                        if ui.selectable_label(false, name).clicked() {
                            fixture = Some(name.clone());
                        }
                    }
                });
            });
            if let Some(fixture) = fixture {
                self.urdf_from_fixture(&fixture);
            }
            if self.urdf_busy {
                ui.spinner();
            }
        });
        let Geometry::Urdf { path } = &mut self.product.geometry else { return };
        egui::Grid::new("urdf_fields").num_columns(2).show(ui, |ui| {
            ui.weak("URDF");
            ui.horizontal(|ui| {
                ui.add(egui::TextEdit::singleline(path).hint_text("urdf/part.urdf, in the product's folder"));
                if self.urdfs.len() > 1 {
                    egui::ComboBox::from_id_salt("product_urdf").selected_text("").show_ui(ui, |ui| {
                        for urdf in &self.urdfs {
                            ui.selectable_value(path, urdf.clone(), urdf);
                        }
                    });
                }
            });
            ui.end_row();
        });
        ui.small(RichText::new("Its root link's frame is the product's: task positions are in it.").weak());
    }

    fn product_ui(&mut self, ui: &mut egui::Ui) {
        let names = self.listed.as_ref().map(|l| l.products.clone()).unwrap_or_default();
        ui.horizontal(|ui| {
            let current = self.saved.as_ref().map_or("—", |(p, _)| p.name.as_str()).to_owned();
            egui::ComboBox::from_id_salt("product_pick").selected_text(current).width(180.0).truncate().show_ui(ui, |ui| {
                if names.is_empty() {
                    ui.weak("No saved products yet");
                }
                for name in &names {
                    if ui.selectable_label(false, name).clicked() {
                        self.load(name);
                    }
                }
            });
            if ui.button("New").on_hover_text("Start a new product: a box without tasks").clicked() {
                self.product = new_product();
                self.saved = None;
                self.status = None;
            }
            if ui.small_button("⟳").on_hover_text("Re-list the products folder").clicked() {
                self.refresh();
            }
        });
        if let Some(listed) = &self.listed {
            crate::ui_mode::advanced(ui, |ui| ui.small(RichText::new(&listed.dir).weak().monospace()));
            if let Some(err) = &listed.error {
                ui.label(RichText::new(err).small().color(theme::current().danger));
            }
        }
        egui::Grid::new("product_fields").num_columns(2).show(ui, |ui| {
            ui.weak("Name");
            ui.add(egui::TextEdit::singleline(&mut self.product.name).hint_text("e.g. door_panel"));
            ui.end_row();
            ui.weak("Description");
            ui.add(egui::TextEdit::singleline(&mut self.product.description).hint_text("what is produced"));
            ui.end_row();
        });
    }

    fn tasks_ui(&mut self, ui: &mut egui::Ui) {
        let tools = self.catalogue_tools.lock().unwrap().clone();
        let size = match self.product.geometry {
            Geometry::Box { size } => Some(size),
            Geometry::Mesh { .. } | Geometry::Urdf { .. } => None,
        };
        let mut remove = None;
        let count = self.product.tasks.len();
        // A task program's Edit: open its task once the product is loaded.
        let focus = self.focus_task.take_if(|name| self.product.tasks.iter().any(|t| t.name == *name));
        for (index, task) in self.product.tasks.iter_mut().enumerate() {
            let known = process(&task.process);
            let title = format!(
                "{} · {} · {} · {} operation{}",
                if task.name.is_empty() { "(unnamed)" } else { &task.name },
                known.map_or(task.process.as_str(), |p| p.label),
                if task.tool.is_empty() { "no tool" } else { &task.tool },
                task.operations.len(),
                if task.operations.len() == 1 { "" } else { "s" },
            );
            let focused = focus.as_deref() == Some(task.name.as_str());
            let header = egui::CollapsingHeader::new(title)
                .id_salt(("task", index))
                .default_open(index + 1 == count)
                .open(focus.as_ref().map(|_| focused))
                .show(ui, |ui| {
                    if task_ui(ui, index, task, &tools, size, &mut self.generator) {
                        remove = Some(index);
                    }
                });
            if focused {
                header.header_response.scroll_to_me(Some(egui::Align::TOP));
            }
        }
        if let Some(index) = remove {
            self.product.tasks.remove(index);
        }
        ui.horizontal(|ui| {
            egui::ComboBox::from_id_salt("new_task_process")
                .selected_text(PROCESSES[self.new_process].label)
                .show_ui(ui, |ui| {
                    for (index, process) in PROCESSES.iter().enumerate() {
                        ui.selectable_value(&mut self.new_process, index, process.label);
                    }
                });
            if ui.button("Add task").clicked() {
                let process = &PROCESSES[self.new_process];
                let same = self.product.tasks.iter().filter(|t| t.process == process.key).count();
                self.product.tasks.push(Task {
                    name: format!("{}_{}", snake(process.key), same + 1),
                    process: process.key.to_owned(),
                    tool: String::new(),
                    params: process.default_params(),
                    operations: Vec::new(),
                });
            }
        });
    }
}

/// One task's editor. Returns whether it should be removed.
fn task_ui(
    ui: &mut egui::Ui,
    index: usize,
    task: &mut Task,
    tools: &[String],
    size: Option<[f32; 3]>,
    generator: &mut Generator,
) -> bool {
    let mut remove = false;
    egui::Grid::new(("task_fields", index)).num_columns(2).show(ui, |ui| {
        ui.weak("Name");
        ui.add(egui::TextEdit::singleline(&mut task.name));
        ui.end_row();

        ui.weak("Process");
        let label = process(&task.process).map_or(task.process.clone(), |p| p.label.to_owned());
        egui::ComboBox::from_id_salt(("process", index)).selected_text(label).show_ui(ui, |ui| {
            for candidate in PROCESSES {
                if ui.selectable_label(task.process == candidate.key, candidate.label).clicked() && task.process != candidate.key {
                    task.process = candidate.key.to_owned();
                    task.params = switched_params(candidate, &task.params);
                }
            }
        });
        ui.end_row();

        ui.weak("Tool");
        let hint = process(&task.process).map_or("the tool it needs", |p| p.tool_hint);
        ui.horizontal(|ui| {
            ui.add(egui::TextEdit::singleline(&mut task.tool).hint_text(hint).desired_width(160.0));
            egui::ComboBox::from_id_salt(("tool", index)).selected_text("catalogue").show_ui(ui, |ui| {
                if tools.is_empty() {
                    ui.weak("No tools in the Robo Dresser catalogue");
                }
                for tool in tools {
                    ui.selectable_value(&mut task.tool, tool.clone(), tool);
                }
            });
        });
        ui.end_row();
    });

    let known = process(&task.process);
    // The process parameters and the planning strategy: Advanced only (Simple plans with their values).
    if let Some(process) = known {
        crate::ui_mode::advanced(ui, |ui| params_ui(ui, index, process, &mut task.params));
    } else {
        ui.small(RichText::new(format!("Unknown process {:?}: its parameters are kept as they are.", task.process)).color(theme::current().warn));
    }

    crate::ui_mode::advanced(ui, |ui| strategy_ui(ui, index, &mut task.params));

    let target = known.map_or(TargetKind::Point, |p| p.target);
    operations_ui(ui, index, task, target);
    let misfits = task.operations.iter().filter(|op| op.target.kind() != target).count();
    if misfits > 0 {
        let wanted = if target == TargetKind::Point { "points" } else { "paths" };
        ui.small(RichText::new(format!("{misfits} operation(s) don't fit this process, which works on {wanted}.")).color(theme::current().warn));
    }

    ui.horizontal(|ui| {
        if ui.button("Add operation").clicked() {
            let top = size.map_or(0.0, |s| s[2]);
            let id = next_id(task, 0);
            let target = match target {
                TargetKind::Point => Target::Point { position: [0.0, 0.0, top], normal: [0.0, 0.0, 1.0] },
                TargetKind::Path => Target::Path {
                    points: vec![[-0.1, 0.0, top], [0.1, 0.0, top]],
                    normal: [0.0, 0.0, 1.0],
                },
            };
            task.operations.push(Operation {
                id,
                target,
                group: generator.group.clone(),
                params: Params::new(),
            });
        }
        if !task.operations.is_empty() && ui.button("Remove all").clicked() {
            task.operations.clear();
        }
        if ui.button("Remove task").clicked() {
            remove = true;
        }
    });
    match size {
        Some(size) => generator_ui(ui, index, task, target, size, generator),
        None => {
            ui.small(RichText::new("The generators work on boxes; add a mesh's operations one by one.").weak());
        }
    }
    ui.add_space(4.0);
    remove
}

fn params_ui(ui: &mut egui::Ui, index: usize, process: &Process, params: &mut Params) {
    if process.params.is_empty() {
        return;
    }
    egui::Grid::new(("params", index)).num_columns(2).show(ui, |ui| {
        for param in process.params {
            ui.weak(param.label);
            let mut value = params.get(param.key).and_then(serde_json::Value::as_f64).unwrap_or(param.default);
            ui.horizontal(|ui| {
                let speed = (param.default.abs() / 100.0).max(0.001);
                if ui.add(egui::DragValue::new(&mut value).speed(speed).max_decimals(4)).changed() {
                    params.insert(param.key.to_owned(), serde_json::json!(value));
                }
                ui.small(RichText::new(param.unit).weak());
            });
            ui.end_row();
        }
    });
}

/// A task's parameters for another process: its defaults, keeping the values of the parameters
/// both have, and the planning strategy.
fn switched_params(process: &Process, params: &Params) -> Params {
    let mut switched = process.default_params();
    for (key, value) in params {
        if switched.contains_key(key) || STRATEGY.contains(&key.as_str()) {
            switched.insert(key.clone(), value.clone());
        }
    }
    switched
}

/// `"a, b ,,c"` → `["a", "b", "c"]`.
fn group_order(text: &str) -> Vec<String> {
    text.split(',').map(str::trim).filter(|g| !g.is_empty()).map(str::to_owned).collect()
}

/// A param's number, or `None` (unset).
fn number(params: &Params, key: &str) -> Option<f64> {
    params.get(key).and_then(serde_json::Value::as_f64)
}

/// A number field for an optional param: a checkbox sets or unsets it.
fn optional_number(ui: &mut egui::Ui, params: &mut Params, key: &str, default: f64, integer: bool, unit: &str) {
    ui.horizontal(|ui| {
        let mut set = params.contains_key(key);
        if ui.checkbox(&mut set, "").on_hover_text("Unset: the planner's default").changed() {
            if set {
                let value = if integer { serde_json::json!(default as i64) } else { serde_json::json!(default) };
                params.insert(key.to_owned(), value);
            } else {
                params.remove(key);
            }
        }
        if let Some(mut value) = number(params, key) {
            let drag = if integer {
                egui::DragValue::new(&mut value).speed(0.1).range(1.0..=16.0).max_decimals(0)
            } else {
                egui::DragValue::new(&mut value).speed(0.5).range(0.0..=3600.0).max_decimals(1)
            };
            if ui.add(drag).changed() {
                let value = if integer { serde_json::json!(value.round() as i64) } else { serde_json::json!(value) };
                params.insert(key.to_owned(), value);
            }
            ui.small(RichText::new(unit).weak());
        } else {
            ui.weak("planner's default");
        }
    });
}

/// The planning strategy of a task: the order of its sequence groups, the robots working at once,
/// and the cycle-time target. Unset values are left to the planner.
fn strategy_ui(ui: &mut egui::Ui, index: usize, params: &mut Params) {
    let set = STRATEGY.iter().filter(|key| params.contains_key(**key)).count();
    let title = if set == 0 { "Strategy (planner's defaults)".to_owned() } else { format!("Strategy ({set} set)") };
    egui::CollapsingHeader::new(title).id_salt(("strategy", index)).show(ui, |ui| {
        egui::Grid::new(("strategy_grid", index)).num_columns(2).show(ui, |ui| {
            ui.weak("Group order");
            let mut text = params
                .get(SEQUENCE_GROUPS_ORDER)
                .and_then(serde_json::Value::as_array)
                .map(|groups| groups.iter().filter_map(serde_json::Value::as_str).collect::<Vec<_>>().join(", "))
                .unwrap_or_default();
            let response = ui
                .add(egui::TextEdit::singleline(&mut text).hint_text("e.g. structural, secondary").desired_width(180.0))
                .on_hover_text("The sequence groups in the order they are done, comma-separated (empty: any order)");
            if response.changed() {
                let groups = group_order(&text);
                if groups.is_empty() {
                    params.remove(SEQUENCE_GROUPS_ORDER);
                } else {
                    params.insert(SEQUENCE_GROUPS_ORDER.to_owned(), serde_json::json!(groups));
                }
            }
            ui.end_row();

            ui.weak("Concurrent robots");
            optional_number(ui, params, MAX_CONCURRENT_ROBOTS, 2.0, true, "at most");
            ui.end_row();

            ui.weak("Parallel in a group");
            let mut parallel = params.get(PARALLEL_WITHIN_GROUP).and_then(serde_json::Value::as_bool);
            ui.horizontal(|ui| {
                for (label, value) in [("default", None), ("yes", Some(true)), ("no", Some(false))] {
                    if ui.add(theme::segment(parallel == value, label)).clicked() {
                        parallel = value;
                    }
                }
            });
            match parallel {
                Some(value) => params.insert(PARALLEL_WITHIN_GROUP.to_owned(), serde_json::json!(value)),
                None => params.remove(PARALLEL_WITHIN_GROUP),
            };
            ui.end_row();

            ui.weak("Cycle-time target");
            optional_number(ui, params, CYCLE_TIME_TARGET_S, 20.0, false, "s");
            ui.end_row();

            ui.weak("Cycle-time max");
            optional_number(ui, params, CYCLE_TIME_MAX_S, 30.0, false, "s");
            ui.end_row();
        });
    });
}

fn operations_ui(ui: &mut egui::Ui, index: usize, task: &mut Task, target: TargetKind) {
    if task.operations.is_empty() {
        ui.weak("No operations yet: add them one by one or generate them on a face.");
        return;
    }
    let mut remove = None;
    egui::ScrollArea::vertical().max_height(260.0).id_salt(("operations", index)).show(ui, |ui| {
        egui::Grid::new(("operations_grid", index)).num_columns(6).striped(true).show(ui, |ui| {
            let where_ = if target == TargetKind::Path { "Path (m)" } else { "Position (m)" };
            for header in ["Id", where_, "Normal", "Group", "Priority", ""] {
                ui.small(RichText::new(header).weak());
            }
            ui.end_row();
            for (op_index, op) in task.operations.iter_mut().enumerate() {
                ui.add(egui::TextEdit::singleline(&mut op.id).desired_width(40.0));
                match &mut op.target {
                    Target::Point { position, normal } => {
                        ui.horizontal(|ui| vector_ui(ui, position, 0.005));
                        ui.horizontal(|ui| vector_ui(ui, normal, 0.05));
                    }
                    Target::Path { points, normal } => {
                        path_ui(ui, (index, op_index), points);
                        ui.horizontal(|ui| vector_ui(ui, normal, 0.05));
                    }
                }
                ui.add(egui::TextEdit::singleline(&mut op.group).desired_width(80.0))
                    .on_hover_text("Its sequence group (the strategy orders the groups)");
                let mut priority = op.params.get(PRIORITY).and_then(serde_json::Value::as_i64).unwrap_or(0);
                if ui
                    .add(egui::DragValue::new(&mut priority).speed(0.1).range(0..=100))
                    .on_hover_text("Higher first within its group (0: no preference)")
                    .changed()
                {
                    if priority == 0 {
                        op.params.remove(PRIORITY);
                    } else {
                        op.params.insert(PRIORITY.to_owned(), serde_json::json!(priority));
                    }
                }
                if ui.small_button("✕").on_hover_text("Remove this operation").clicked() {
                    remove = Some(op_index);
                }
                ui.end_row();
            }
        });
    });
    if let Some(op_index) = remove {
        task.operations.remove(op_index);
    }
}

fn path_ui(ui: &mut egui::Ui, id: (usize, usize), points: &mut Vec<[f32; 3]>) {
    egui::CollapsingHeader::new(format!("{} points", points.len())).id_salt(("path", id)).show(ui, |ui| {
        let mut remove = None;
        for (index, point) in points.iter_mut().enumerate() {
            ui.horizontal(|ui| {
                vector_ui(ui, point, 0.005);
                if ui.small_button("✕").clicked() {
                    remove = Some(index);
                }
            });
        }
        if let Some(index) = remove.filter(|_| points.len() > 2) {
            points.remove(index);
        }
        if ui.small_button("Add point").clicked() {
            let next = points.last().copied().unwrap_or_default();
            points.push([next[0] + 0.05, next[1], next[2]]);
        }
    });
}

fn generator_ui(ui: &mut egui::Ui, index: usize, task: &mut Task, target: TargetKind, size: [f32; 3], generator: &mut Generator) {
    let title = match target {
        TargetKind::Point => "Generate a grid of points on a face",
        TargetKind::Path => "Generate a face's outline as a path",
    };
    egui::CollapsingHeader::new(title)
        .id_salt(("generator", index))
        .default_open(task.operations.is_empty())
        .show(ui, |ui| {
            egui::Grid::new(("generator_grid", index)).num_columns(2).show(ui, |ui| {
                ui.weak("Face");
                egui::ComboBox::from_id_salt(("generator_face", index))
                    .selected_text(FACES[generator.face].1)
                    .show_ui(ui, |ui| {
                        for (face, (_, name, _)) in FACES.iter().enumerate() {
                            ui.selectable_value(&mut generator.face, face, *name);
                        }
                    });
                ui.end_row();
                if target == TargetKind::Point {
                    ui.weak("Rows × columns");
                    ui.horizontal(|ui| {
                        ui.add(egui::DragValue::new(&mut generator.rows).range(1..=50));
                        ui.label("×");
                        ui.add(egui::DragValue::new(&mut generator.columns).range(1..=50));
                    });
                    ui.end_row();
                }
                ui.weak("Margin (m)");
                ui.add(egui::DragValue::new(&mut generator.margin).speed(0.005).range(0.0..=10.0));
                ui.end_row();
                ui.weak("Group");
                ui.add(egui::TextEdit::singleline(&mut generator.group).desired_width(120.0));
                ui.end_row();
            });
            ui.horizontal(|ui| {
                let add = ui.button("Add").clicked();
                let replace = ui.button("Replace all").clicked();
                if add || replace {
                    if replace {
                        task.operations.clear();
                    }
                    let generated = match target {
                        TargetKind::Point => grid_points(size, generator, task),
                        TargetKind::Path => vec![face_outline(size, generator, task)],
                    };
                    task.operations.extend(generated);
                }
            });
        });
}

/// A backend from before Product & Task answers its routes with the web viewer (or 405): say so.
fn explain(err: String) -> String {
    if err.starts_with("405") || err.starts_with("404") || err.starts_with("invalid response") {
        format!("This backend has no Product & Task yet: update it to a newer release. ({err})")
    } else {
        err
    }
}

fn new_product() -> Product {
    Product {
        name: "new_product".to_owned(),
        ..Product::default()
    }
}

fn geometry_ui(ui: &mut egui::Ui, geometry: &mut Geometry) {
    ui.horizontal(|ui| {
        let kind = geometry.kind();
        if ui.add(theme::segment(kind == "box", "Box")).clicked() && kind != "box" {
            *geometry = Geometry::Box { size: [0.6, 0.4, 0.3] };
        }
        if ui.add(theme::segment(kind == "mesh", "Mesh file")).clicked() && kind != "mesh" {
            *geometry = Geometry::Mesh {
                path: String::new(),
                scale: 1.0,
            };
        }
        if ui.add(theme::segment(kind == "urdf", "URDF")).clicked() && kind != "urdf" {
            *geometry = Geometry::Urdf { path: String::new() };
        }
    });
    match geometry {
        Geometry::Box { size } => {
            ui.horizontal(|ui| {
                ui.weak("Size (m)");
                for (axis, value) in ["x", "y", "z"].into_iter().zip(size.iter_mut()) {
                    ui.add(egui::DragValue::new(value).speed(0.01).range(0.001..=100.0).prefix(format!("{axis} ")));
                }
            });
            ui.small(RichText::new("Standing on the origin, z up.").weak());
        }
        Geometry::Mesh { path, scale } => {
            egui::Grid::new("mesh_fields").num_columns(2).show(ui, |ui| {
                ui.weak("File");
                ui.add(egui::TextEdit::singleline(path).hint_text("part.obj / .stl / .glb, in the product's folder"));
                ui.end_row();
                ui.weak("Scale");
                ui.add(egui::DragValue::new(scale).speed(0.001).range(0.000_001..=1000.0));
                ui.end_row();
            });
            ui.small(RichText::new("A path relative to the product's folder on the backend host, or absolute.").weak());
        }
        // Its sources are the pane's (`urdf_ui`).
        Geometry::Urdf { .. } => {}
    }
}

fn vector_ui(ui: &mut egui::Ui, v: &mut [f32; 3], speed: f64) {
    for value in v.iter_mut() {
        ui.add(egui::DragValue::new(value).speed(speed).max_decimals(3));
    }
}

fn section(ui: &mut egui::Ui, title: &str) {
    ui.add_space(10.0);
    ui.label(RichText::new(title.to_uppercase()).small().strong());
}

/// `spotWeld` → `spot_weld`.
fn snake(key: &str) -> String {
    let mut out = String::new();
    for c in key.chars() {
        if c.is_ascii_uppercase() {
            out.push('_');
            out.push(c.to_ascii_lowercase());
        } else {
            out.push(c);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn generator(face: usize, rows: u32, columns: u32, margin: f32) -> Generator {
        Generator {
            face,
            rows,
            columns,
            margin,
            group: "g".to_owned(),
        }
    }

    fn task(process: &str) -> Task {
        Task {
            process: process.to_owned(),
            ..Task::default()
        }
    }

    #[test]
    fn grid_on_top_face() {
        let ops = grid_points([1.0, 0.6, 0.2], &generator(0, 2, 3, 0.1), &task("spotWeld"));
        assert_eq!(ops.len(), 6);
        assert_eq!(ops[0].id, "W01");
        assert_eq!(ops[0].target, Target::Point { position: [-0.4, -0.2, 0.2], normal: [0.0, 0.0, 1.0] });
        assert_eq!(ops[5].target, Target::Point { position: [0.4, 0.2, 0.2], normal: [0.0, 0.0, 1.0] });
    }

    #[test]
    fn outline_of_a_side_face() {
        let op = face_outline([1.0, 1.0, 1.0], &generator(2, 1, 1, 0.1), &task("glue"));
        assert_eq!(op.id, "G01");
        let Target::Path { points, normal } = op.target else { panic!("not a path") };
        assert_eq!(normal, [1.0, 0.0, 0.0]);
        assert_eq!(points.len(), 5);
        assert_eq!(points[0], points[4]);
        assert!(points.iter().all(|p| p[0] == 0.5));
    }

    #[test]
    fn ids_continue_after_existing_operations() {
        let mut spots = task("spotWeld");
        spots.operations.push(Operation {
            id: "W07".to_owned(),
            target: Target::Point { position: [0.0; 3], normal: [0.0, 0.0, 1.0] },
            group: String::new(),
            params: Params::new(),
        });
        assert_eq!(next_id(&spots, 0), "W08");
        assert_eq!(grid_points([1.0; 3], &generator(0, 1, 2, 0.0), &spots)[1].id, "W09");
        assert_eq!(snake("spotWeld"), "spot_weld");
    }

    #[test]
    fn switching_process_keeps_the_strategy() {
        let mut params = Params::new();
        params.insert(MAX_CONCURRENT_ROBOTS.to_owned(), serde_json::json!(2));
        params.insert(SEQUENCE_GROUPS_ORDER.to_owned(), serde_json::json!(["a", "b"]));
        params.insert("not_a_param_of_glue".to_owned(), serde_json::json!(1));
        let glue = process("glue").or_else(|| PROCESSES.last()).unwrap();
        let switched = switched_params(glue, &params);
        assert_eq!(switched[MAX_CONCURRENT_ROBOTS], serde_json::json!(2));
        assert_eq!(switched[SEQUENCE_GROUPS_ORDER], serde_json::json!(["a", "b"]));
        assert!(!switched.contains_key("not_a_param_of_glue"));
        assert_eq!(group_order(" structural, ,secondary "), ["structural", "secondary"]);
    }
}
