//! The "Configuration" pane, mirroring the Kiro extension's `configuration-sidebar.ts` (Sept 2026
//! bundle): one section per tool (Robo Dresser, Robo Fitter, Create Cell Runner Scene, Cell Runner,
//! Agentic Planning, EDH Integration, …), below the config file and build info.

use std::path::Path;
use web_time::{Duration, Instant};

use factoryos_client::protocol::config::{ConfigResponse, Secrets as SecretsUpdate};
use crate::theme;
use crate::rr::egui::{self, RichText};

/// Remote config updates are ignored this long after a local edit, so our own (older) echoes
/// don't overwrite what's being typed.
const REMOTE_UPDATE_GRACE: Duration = Duration::from_secs(2);

/// Settings, owned by factoryos-backend and stored in its JSON config file.
/// Passwords are deliberately not part of this (see [`Secrets`]).
#[derive(Default)]
pub struct Config {
    pub settings: factoryos_client::protocol::Config,

    /// The config file on the backend host; relative paths resolve against its folder.
    pub file: String,
    pub file_error: Option<String>,
    pub reload_requested: bool,
    /// The backend runs on another machine, so its paths can't be picked with a file dialog here.
    pub remote_backend: bool,

    /// `settings` as last sent to (or received from) the backend.
    synced: factoryos_client::protocol::Config,
    last_local_edit: Option<Instant>,
}

impl std::ops::Deref for Config {
    type Target = factoryos_client::protocol::Config;

    fn deref(&self) -> &Self::Target {
        &self.settings
    }
}

impl std::ops::DerefMut for Config {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.settings
    }
}

/// Optional pre-configured passwords, sent to the backend (which keeps them in memory only).
#[derive(Default, Clone, PartialEq)]
pub struct Secrets {
    edh_password: String,
    cognito_password: String,
    os_password: String,
}

impl Secrets {
    pub fn to_update(&self) -> SecretsUpdate {
        SecretsUpdate {
            edh_password: Some(self.edh_password.clone()),
            cognito_password: Some(self.cognito_password.clone()),
            os_password: Some(self.os_password.clone()),
        }
    }
}

impl Config {
    pub fn from_response(response: ConfigResponse) -> Self {
        let mut config = Self::default();
        config.apply_response(response);
        config
    }

    /// Take the backend's config (after a reload, or a change by another client).
    pub fn apply_response(&mut self, response: ConfigResponse) {
        self.synced = response.config.clone();
        self.settings = response.config;
        self.file = response.file;
        self.file_error = response.file_error;
    }

    /// A config published by the backend; ignored while the user is editing.
    pub fn apply_remote(&mut self, settings: factoryos_client::protocol::Config) {
        if self.last_local_edit.is_some_and(|t| t.elapsed() < REMOTE_UPDATE_GRACE) {
            return;
        }
        self.synced = settings.clone();
        self.settings = settings;
    }

    /// The settings to send to the backend, if they changed since the last sync.
    pub fn take_changes(&mut self) -> Option<factoryos_client::protocol::Config> {
        if self.settings == self.synced {
            return None;
        }
        self.synced = self.settings.clone();
        self.last_local_edit = Some(Instant::now());
        Some(self.settings.clone())
    }

    /// `value` as an absolute path: relative paths resolve against the config file's folder.
    pub fn resolve(&self, value: &str) -> String {
        let path = Path::new(value.trim());
        if path.is_absolute() || value.trim().is_empty() {
            return value.trim().to_owned();
        }
        Path::new(&self.file)
            .parent()
            .unwrap_or(Path::new("."))
            .join(path)
            .to_string_lossy()
            .into_owned()
    }
}

type Settings = factoryos_client::protocol::Config;

/// One editable setting, like a field of the Kiro extension's `CONFIG_SECTIONS`.
struct Field {
    label: &'static str,
    /// Hint text shown in an empty field.
    hint: &'static str,
    /// Hover text (the extension's setting description).
    help: &'static str,
    kind: Kind,
}

enum Kind {
    Text(fn(&mut Settings) -> &mut String),
    /// Several lines of text, e.g. a prompt.
    MultiLine(fn(&mut Settings) -> &mut String),
    Folder(fn(&mut Settings) -> &mut String),
    File(fn(&mut Settings) -> &mut String),
    Number(fn(&mut Settings) -> &mut u32),
    Bool(fn(&mut Settings) -> &mut bool),
    Enum(&'static [&'static str], fn(&mut Settings) -> &mut String),
    Secret(fn(&mut Secrets) -> &mut String),
}

struct Section {
    title: &'static str,
    note: &'static str,
    fields: &'static [Field],
}

const fn field(label: &'static str, hint: &'static str, help: &'static str, kind: Kind) -> Field {
    Field { label, hint, help, kind }
}

/// The placeholders the steps' agent prompts can use.
const AGENT_PLACEHOLDERS: &str = "{product}, {product_file}, {product_dir}, {catalogue}, {asset_root}, {fitted_robots_dir}, {cell}, {cell_id}, {cell_dir}";

/// The placeholders of the reachability agent's prompt.
const REACH_PLACEHOLDERS: &str = "{scene_file}, {fitted_robots_dir}, {cell}, {cell_id}, {asset_root}, {catalogue}";

/// The link to an Isaac Sim stack's CloudFormation quick-create page, if the build has one: the internal build's
/// edge serves the dev account's (its internal module). Without one (the public build) there is no launch button,
/// and so no Isaac Sim section in the configuration.
#[cfg(feature = "internal")]
pub(crate) const ISAAC_LAUNCH_URL: Option<&str> = Some(crate::internal::ISAAC_LAUNCH_URL);
#[cfg(not(feature = "internal"))]
pub(crate) const ISAAC_LAUNCH_URL: Option<&str> = None;

/// Whether the configuration shows section `s`: the Isaac Sim one is its launch link only.
fn shown(s: &Section) -> bool {
    s.title != "Isaac Sim" || ISAAC_LAUNCH_URL.is_some()
}

/// The feature sections, top-down, mirroring the Kiro extension's Configuration sidebar.
const SECTIONS: &[Section] = &[
    Section {
        title: "Product & Task",
        note: "",
        fields: &[
            field("Products folder", "empty = products/ next to the config file",
                "Where products are saved (<name>/product.json): the part to produce and its task, e.g. weld spots.",
                Kind::Folder(|c| &mut c.products_dir)),
            field("Agent prompt", AGENT_PLACEHOLDERS,
                "What \"Create task description\" sends the agent: the product's tasks worked out into operations in its own frame. Empty = the default.",
                Kind::MultiLine(|c| &mut c.task_author_prompt)),
        ],
    },
    Section {
        title: "Robo Dresser",
        note: "",
        fields: &[
            field("Catalogue dir", "urdf-robot-collection/ + urdf-tool-collection/",
                "Catalogue directory containing urdf-robot-collection/ and urdf-tool-collection/. Dressed robots are written to urdf-dressed-robots/ inside it.",
                Kind::Folder(|c| &mut c.catalogue)),
            field("Port", "", "TCP port `serve --port` binds to.", Kind::Number(|c| &mut c.dresser_port)),
            field("Binary path", "bundled binary if empty", "Path to the urdf-robo-dresser binary.", Kind::File(|c| &mut c.dresser_binary_path)),
            field("Web root", "embedded bundle if empty", "Optional `serve --web-root` for live SPA editing. Usually left empty.", Kind::Folder(|c| &mut c.dresser_web_root)),
            field("Agent prompt", AGENT_PLACEHOLDERS,
                "What \"Suggest robots & tools\" sends the agent: the robot + tool pairs the current product's tasks need. Empty = the default.",
                Kind::MultiLine(|c| &mut c.dresser_agent_prompt)),
        ],
    },
    Section {
        title: "Robo Fitter",
        note: "",
        fields: &[
            field("Asset root", "asset root passed to --asset-root",
                "Asset root: urdf-dressed-robots/, urdf-fixture-collection/, floor-plans/; scenes are saved in scenes/.",
                Kind::Folder(|c| &mut c.asset_root)),
            field("Server URL", "", "Base URL of the robo-fitting-room REST server.", Kind::Text(|c| &mut c.fitter_server_url)),
            field("Binary path", "bundled binary if empty", "Path to the robo-fitting-room binary.", Kind::File(|c| &mut c.fitter_binary_path)),
            field("Host", "", "Host the managed server binds to (serve --host).", Kind::Text(|c| &mut c.fitter_server_host)),
            field("Port", "", "Port the managed server binds to (serve --port).", Kind::Number(|c| &mut c.fitter_server_port)),
            field("Web root", "embedded bundle if empty", "Optional --web-root for live SPA editing. Usually left empty.", Kind::Folder(|c| &mut c.fitter_web_root)),
            field("Agent prompt", AGENT_PLACEHOLDERS,
                "What \"Propose cell layout\" sends the agent: a scene with the robots and the part placed so every operation is reachable. Empty = the default.",
                Kind::MultiLine(|c| &mut c.fitter_agent_prompt)),
            field("Reach agent prompt", REACH_PLACEHOLDERS,
                "What \"Check reachability (agent)\" sends the agent: check that the open scene's robots reach every weld, explain what they can't, and propose a fixed layout as a new scene. Empty = the default.",
                Kind::MultiLine(|c| &mut c.reach_agent_prompt)),
            field("Cell planner Python", "empty = /opt/factoryos-cell-planner/router/venv/bin/python",
                "Advanced: the Python (with pinocchio) that runs the reachability check's IK.",
                Kind::File(|c| &mut c.cell_planner_python)),
            field("Cell planner src", "empty = /opt/factoryos-cell-planner/router/src",
                "Advanced: the cell planner's sources (trajgen), put on the IK script's path.",
                Kind::Folder(|c| &mut c.cell_planner_src)),
            field("Reach check script", "empty = the release's deploy/planner-router/reach_check.py",
                "Advanced: the reachability check's IK script.",
                Kind::File(|c| &mut c.reach_check_script)),
        ],
    },
    Section {
        title: "Create Cell Runner Scene",
        note: "",
        fields: &[field("Fitted-robots folder", "Fitter output; empty = <asset root>/scenes",
            "The Fitter's scenes folder (scene manifests referencing sibling asset libraries). 'Create Cellrunner Cell' reads scenes from here.",
            Kind::Folder(|c| &mut c.fitted_robots_dir))],
    },
    Section {
        title: "Cell Runner",
        note: "",
        fields: &[
            field("Cells folder", "folder that contains your cells (pick the scene in step 1 Behaviour Tree)",
                "The folder that contains your cells (each subfolder with a scene.yaml or cell.yaml is a cell). Empty = the active cell's parent folder.",
                Kind::Folder(|c| &mut c.cells_dir)),
            field("Mesh LOD", "", "Mesh level of detail in the 3D viewer: low (decimated, recommended), visual (full meshes) or collision (hulls).",
                Kind::Enum(&["low", "visual", "collision"], |c| &mut c.mesh_lod)),
            field("Trajectory dir", "", "Planned trajectory files (BehaviourTree JSON), relative to the cell directory.", Kind::Text(|c| &mut c.trajectory_dir)),
            field("Collision check on load", "", "Check a trajectory for scene collisions when it is loaded.", Kind::Bool(|c| &mut c.trajectory_collision_check)),
            field("Cell-runner WS port", "", "WebSocket port of the simulated cell-runner (step 1 Load & Play); a free one when it is taken.", Kind::Number(|c| &mut c.cell_runner_ws_port)),
        ],
    },
    Section {
        title: "Agentic Planning",
        note: "",
        fields: &[
            field("Checkout dir (repoDir)", "multi_robots_agents checkout", "Your installed multi_robots_agents checkout; all `make` commands run here.", Kind::Folder(|c| &mut c.agentic_repo_dir)),
            field("Orchestrator URL", "", "Base URL of the orchestrator's A2A HTTP API (missions, health).", Kind::Text(|c| &mut c.agentic_orchestrator_url)),
            field("Dashboard URL", "", "Base URL of the web dashboard.", Kind::Text(|c| &mut c.agentic_dashboard_url)),
            field("RViz URL", "", "URL of the RViz web viewer (noVNC). Requires 'Enable RViz' and a running simulation.", Kind::Text(|c| &mut c.agentic_rviz_url)),
            field("Trajectory backend", "", "Trajectory planning backend passed to `make sim`.", Kind::Enum(&["ompl", "rrt"], |c| &mut c.agentic_trajectory_backend)),
            field("LLM model", "checkout default if empty", "Startup LLM alias/id forwarded as LLM_MODEL (e.g. opus-5, sonnet-5).", Kind::Text(|c| &mut c.agentic_model)),
            field("AWS/Bedrock region", "e.g. us-east-1 (creds pasted at Start, not here)", "AWS/Bedrock region for the LLM agents, forwarded as BEDROCK_REGION.", Kind::Text(|c| &mut c.agentic_region)),
            field("Enable RViz (:6080)", "", "Also bring up the optional RViz web viewer when starting the simulation.", Kind::Bool(|c| &mut c.agentic_rviz_enabled)),
            field("Middleware (RMW)", "", "ROS 2 middleware: cyclonedds (peer-to-peer) or zenoh (brokered; requires 'Zenoh router').",
                Kind::Enum(&["cyclonedds", "zenoh"], |c| &mut c.agentic_rmw)),
            field("Zenoh router", "required for zenoh", "Endpoint of an existing Zenoh broker, e.g. tcp/broker.plant.local:7447.", Kind::Text(|c| &mut c.agentic_zenoh_router)),
            field("Trust all tools", "",
                "Let the planner agent use every tool (the planner's MCP tools, file reads and writes, shell commands) without asking first: kiro-cli acp --trust-all-tools. Takes effect when the agent restarts (Restart in the chat).",
                Kind::Bool(|c| &mut c.agentic_trust_all_tools)),
            field("Plan prompt", "{cell}, {cell_id}, {task} and {planner} are filled in",
                "What \"Plan Cell\" sends the planner agent. {cell}: the cell's folder name; {cell_id}: the planner's cell id; {task}: the picked task program; {planner}: the picked planner (auto: the planner router recommends one). Empty = the default; a prompt left at an earlier default follows the current one.",
                Kind::MultiLine(|c| &mut c.agentic_plan_prompt)),
        ],
    },
    Section {
        title: "EDH Integration",
        note: "Shared: used by Open Sim Scene, Open PLC IDE, and the AWS credential bootstrap. Passwords are optional: \
               they are kept in the backend's memory for this session only and never written to disk; left blank, \
               you are prompted once per session.",
        fields: &[
            field("Username prefix", "e.g. user@example.com",
                "Per-user key (typically your EDH/Cognito login) used to locate this user's credentials in AWS (Secrets Manager <prefix>/nicedcv, Parameter Store /<prefix>/nicedcv).",
                Kind::Text(|c| &mut c.credentials_prefix)),
            field("EDH base URL", "https://<id>.cloudfront.net", "Base URL of the EDH entry point (its CloudFront URL).", Kind::Text(|c| &mut c.edh_base_url)),
            field("EDH username", "", "Your EDH username (sent as X-EDH-USER).", Kind::Text(|c| &mut c.edh_username)),
            field("EDH password", "", "", Kind::Secret(|s| &mut s.edh_password)),
            field("Use Cognito", "", "Prepend a Cognito sign-in step to the EDH login. Requires 'Cognito username'.", Kind::Bool(|c| &mut c.use_cognito)),
            field("Cognito username", "", "Your Cognito username (only used with 'Use Cognito').", Kind::Text(|c| &mut c.cognito_username)),
            field("Cognito password", "", "", Kind::Secret(|s| &mut s.cognito_password)),
            field("Workspace (OS) username", "", "The desktop's own OS login, if different from your EDH username.", Kind::Text(|c| &mut c.os_username)),
            field("Workspace (OS) password", "", "", Kind::Secret(|s| &mut s.os_password)),
            field("Start timeout (s)", "", "How long to wait for a stopped workspace to reach 'running'.", Kind::Number(|c| &mut c.edh_start_timeout_seconds)),
            field("Poll interval (s)", "", "How often to poll EDH for a starting workspace's state.", Kind::Number(|c| &mut c.edh_poll_interval_seconds)),
        ],
    },
    Section {
        title: "Isaac Sim",
        note: "An Isaac Sim instance in any AWS account, linked to this workspace: the CloudFormation console \
               (signed in to that account) opens with the stack's template, its workspace and edge filled in.",
        fields: &[],
    },
    Section {
        title: "Open Sim Scene",
        note: "NICE DCV configuration lives under EDH Integration above.",
        fields: &[],
    },
    Section {
        title: "Control Code Generation",
        note: "No settings yet. Uses EDH Integration › NICE DCV for Open PLC IDE.",
        fields: &[],
    },
];

/// The whole config, for the full-window settings: `general` (e.g. the backend), the config file
/// and the build first, then the tool sections, flowed top-down through as many columns as fit.
pub fn ui(ui: &mut egui::Ui, config: &mut Config, secrets: &mut Secrets, general: impl FnOnce(&mut egui::Ui)) {
    // Simple: the general block only; the config file, build and each tool's paths and settings
    // are Advanced (they keep applying as set).
    if !crate::ui_mode::is_advanced(ui) {
        general(ui);
        ui.add_space(8.0);
        ui.small(RichText::new("The tools' directories, endpoints and credentials: switch to Advanced (top bar).").weak());
        return;
    }
    const COLUMN_WIDTH: f32 = 380.0;
    let columns = ((ui.available_width() / COLUMN_WIDTH) as usize).clamp(1, 4);

    // Roughly how tall each block is (a labelled field takes two rows), to balance the columns.
    let general_rows = 12;
    let rows = |s: &Section| {
        let fields: usize = s.fields.iter().map(|f| if matches!(f.kind, Kind::MultiLine(_)) { 5 } else { 2 }).sum();
        2 + fields + if s.note.is_empty() { 0 } else { 3 } + match s.title {
            "Cell Runner" => 4,
            "Isaac Sim" => 3,
            _ => 0,
        }
    };
    let sections: Vec<&Section> = SECTIONS.iter().filter(|s| shown(s)).collect();
    let weights: Vec<usize> = std::iter::once(general_rows).chain(sections.iter().map(|s| rows(s))).collect();
    let target = weights.iter().sum::<usize>().div_ceil(columns);
    let mut column_of = Vec::with_capacity(weights.len());
    let (mut column, mut filled) = (0, 0);
    for weight in &weights {
        // Start the next column when this block would mostly overflow the current one.
        if filled > 0 && filled + weight / 2 > target && column + 1 < columns {
            column += 1;
            filled = 0;
        }
        column_of.push(column);
        filled += weight;
    }

    let mut general = Some(general);
    ui.columns(columns, |uis| {
        for (block, &column) in column_of.iter().enumerate() {
            let ui = &mut uis[column];
            ui.spacing_mut().item_spacing.y = 4.0;
            match block {
                0 => {
                    if let Some(general) = general.take() {
                        general(ui);
                    }
                    file_and_build_ui(ui, config);
                }
                _ => section_ui(ui, sections[block - 1], config, secrets),
            }
        }
    });
}

fn file_and_build_ui(ui: &mut egui::Ui, config: &mut Config) {
    section(ui, "Config file");
    ui.horizontal(|ui| {
        ui.add(egui::Label::new(RichText::new(&config.file).small().monospace()).wrap());
    });
    ui.horizontal(|ui| {
        if ui.small_button("Reload").on_hover_text("Re-read the file, e.g. after editing it by hand").clicked() {
            config.reload_requested = true;
        }
        ui.small(RichText::new("Changes here are saved by the backend automatically.").weak());
    });
    if let Some(err) = &config.file_error {
        ui.label(RichText::new(err).small().color(theme::current().danger));
    }

    section(ui, "Build");
    ui.horizontal(|ui| {
        ui.weak("Version");
        ui.monospace(env!("CARGO_PKG_VERSION"));
    });
}

fn section_ui(ui: &mut egui::Ui, s: &Section, config: &mut Config, secrets: &mut Secrets) {
    section(ui, s.title);
    if !s.note.is_empty() {
        description(ui, s.note);
    }
    if s.title == "Cell Runner" {
        active_cell_ui(ui, config);
    }
    if s.title == "Isaac Sim"
        && let Some(url) = ISAAC_LAUNCH_URL
    {
        isaac_launch_ui(ui, url);
    }
    for f in s.fields {
        field_ui(ui, f, config, secrets);
    }
}

/// The link to launch an Isaac Sim stack (`url`): opened in a new tab, or copied (to open it signed in to another account).
fn isaac_launch_ui(ui: &mut egui::Ui, url: &str) {
    ui.horizontal(|ui| {
        if ui
            .button("Launch Isaac Sim stack")
            .on_hover_text("The CloudFormation console's Create stack page, in a new tab")
            .clicked()
        {
            // Web: in the click's user gesture (like a link). Native: the system browser.
            #[cfg(target_arch = "wasm32")]
            ui.ctx().open_url(egui::OpenUrl::new_tab(url));
            #[cfg(not(target_arch = "wasm32"))]
            crate::workspace_ui::open_url(url);
        }
        if ui.small_button("Copy link").on_hover_text(url).clicked() {
            ui.ctx().copy_text(url.to_owned());
        }
    });
    ui.add(egui::Label::new(RichText::new(url).small().monospace().weak()).wrap());
    description(ui, "Add ?region=<region> to create the stack in another region (default us-east-1).");
}

/// The active cell's paths (chosen in Designer Workflow → step 1 Behaviour Tree → Scene, or Robo Fitter → Open cell).
fn active_cell_ui(ui: &mut egui::Ui, config: &Config) {
    ui.weak("Active cell").on_hover_text("Pick it in Designer Workflow → step 1 Behaviour Tree → Scene (or Robo Fitter → Open cell)");
    for (label, value) in [("cell", &config.cell_dir), ("env", &config.env_dir), ("scene", &config.scene)] {
        ui.horizontal(|ui| {
            ui.small(RichText::new(label).weak());
            ui.add(egui::Label::new(RichText::new(value).small().monospace()).wrap());
        });
    }
    ui.add_space(4.0);
}

fn field_ui(ui: &mut egui::Ui, f: &Field, config: &mut Config, secrets: &mut Secrets) {
    let help = |response: egui::Response| if f.help.is_empty() { response } else { response.on_hover_text(f.help) };
    match &f.kind {
        Kind::Bool(value) => {
            help(ui.checkbox(value(&mut config.settings), f.label));
        }
        Kind::Secret(value) => password_field(ui, f.label, value(secrets)),
        Kind::Text(value) => {
            help(ui.weak(f.label));
            text_edit(ui, value(&mut config.settings), f.hint, 0.0);
        }
        Kind::MultiLine(value) => {
            help(ui.weak(f.label));
            ui.add(
                egui::TextEdit::multiline(value(&mut config.settings))
                    .hint_text(f.hint)
                    .desired_rows(4)
                    .desired_width(ui.available_width()),
            );
        }
        Kind::Number(value) => {
            ui.horizontal(|ui| {
                help(ui.weak(f.label));
                ui.add(egui::DragValue::new(value(&mut config.settings)).range(0..=u32::MAX));
            });
        }
        Kind::Enum(options, value) => {
            let value = value(&mut config.settings);
            ui.horizontal(|ui| {
                help(ui.weak(f.label));
                egui::ComboBox::from_id_salt(f.label).selected_text(value.as_str()).show_ui(ui, |ui| {
                    for option in *options {
                        ui.selectable_value(value, (*option).to_owned(), *option);
                    }
                });
            });
        }
        Kind::Folder(value) | Kind::File(value) => {
            help(ui.weak(f.label));
            let value = value(&mut config.settings);
            ui.horizontal(|ui| {
                text_edit(ui, value, f.hint, 28.0);
                // The paths are on the backend's host; a browser, or a viewer on another machine,
                // can't pick those.
                #[cfg(not(target_arch = "wasm32"))]
                if !config.remote_backend && browse_button(ui) {
                    let folder = matches!(f.kind, Kind::Folder(_));
                    let mut dialog = rfd::FileDialog::new();
                    if !value.is_empty() {
                        let start = Path::new(value.as_str());
                        dialog = dialog.set_directory(if folder { start } else { start.parent().unwrap_or(start) });
                    }
                    let picked = if folder { dialog.pick_folder() } else { dialog.pick_file() };
                    if let Some(path) = picked {
                        *value = path.to_string_lossy().into_owned();
                    }
                }
            });
        }
    }
}

fn section(ui: &mut egui::Ui, title: &str) {
    ui.add_space(10.0);
    ui.label(RichText::new(title.to_uppercase()).small().strong());
}

fn description(ui: &mut egui::Ui, text: &str) {
    ui.label(RichText::new(text).small().weak());
}

fn text_edit(ui: &mut egui::Ui, value: &mut String, hint: &str, reserve: f32) {
    ui.add(
        egui::TextEdit::singleline(value)
            .hint_text(hint)
            .desired_width(ui.available_width() - reserve),
    );
}

fn password_field(ui: &mut egui::Ui, label: &str, value: &mut String) {
    ui.weak(label);
    ui.horizontal(|ui| {
        let reserve = if value.is_empty() { 0.0 } else { 60.0 };
        ui.add(
            egui::TextEdit::singleline(value)
                .password(true)
                .hint_text("not set — prompt each session")
                .desired_width(ui.available_width() - reserve),
        );
        if !value.is_empty() && ui.small_button("Clear").clicked() {
            value.clear();
        }
    });
}

#[cfg(not(target_arch = "wasm32"))]
fn browse_button(ui: &mut egui::Ui) -> bool {
    ui.button("…").on_hover_text("Browse…").clicked()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_isaac_sim_section_is_its_launch_link() {
        let isaac = SECTIONS.iter().find(|s| s.title == "Isaac Sim").expect("an Isaac Sim section");
        assert!(isaac.fields.is_empty(), "nothing but the launch link to show");
        assert_eq!(shown(isaac), ISAAC_LAUNCH_URL.is_some());
        assert!(SECTIONS.iter().filter(|s| s.title != "Isaac Sim").all(shown));
        #[cfg(not(feature = "internal"))]
        assert_eq!(ISAAC_LAUNCH_URL, None, "the public build has no launch link");
    }
}
