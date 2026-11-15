//! The Factory OS viewer ⇄ backend protocol.
//!
//! * HTTP (`/api/v1/…`, see [`api`]) for request/response: config, starting/stopping the cell,
//!   Robo Dresser and Robo Fitter actions.
//! * A WebSocket for the Agentic Planning chat (`/chat`, see [`chat`]), which the backend bridges
//!   to the planner agent over ACP.
//! * Zenoh (see [`keys`]) for everything live: cell state, positions, 3D scenes, job progress.
//!   3D geometry itself is fetched over HTTP ([`api::ASSET`]).
//!   Native viewers connect with the `zenoh` crate, wasm viewers with zenoh-ts via `/zenoh`.
//!
//! All of it also works under a workspace's prefix, `/workspace/<name>/…` (see [`workspace`]), the
//! same path at the load balancer and on the backend.

pub mod agent;
pub mod agentic;
pub mod cell;
pub mod chat;
pub mod codegen;
pub mod config;
pub mod deploy_check;
pub mod dresser;
pub mod files;
pub mod fitter;
pub mod hardware;
pub mod infra;
pub mod process;
pub mod product;
pub mod product_edit;
pub mod robot;
pub mod rtc;
pub mod scene;
pub mod scene_edit;
pub mod smartpad;
pub mod storage;
pub mod terminal;
pub mod training;
pub mod view;
pub mod workspace;

/// The internal deployment's defaults; not in the public export (every use has a fallback without it).
#[cfg(feature = "internal")]
mod internal;

pub use cell::{
    BackendState, BackendStatus, CellCommand, CellEntry, CellKind, CellResponse, CellState, CellsInfo, CreateCell, LoadPlan, PlanInfo, PlansInfo,
    SampleCell, SetActiveCell, TaskProgram, TaskSource,
};
pub use chat::{ChatCommand, ChatEntry, ChatEvent, ChatRole, ChatSessionInfo, ChatState};
pub use config::{Config, Secrets};

/// Bumped on breaking changes to anything in this crate.
/// v2: native viewers' Zenoh over the HTTP port moved from `/` to [`api::ZENOH_NATIVE_WS`].
/// v3: the backend's simulated cell-runner moved to `factoryos/sim/*` ([`keys::SIM_STATE`], …);
/// `factoryos/cell/*` and `factoryos/rrd/cell` are the live cell's only.
/// v4: 3D is renderer-neutral ([`scene`] on [`keys::scene`], geometry from [`api::asset`]);
/// `factoryos/rrd/*` (Rerun RRD blobs) is gone.
pub const PROTOCOL_VERSION: u32 = 4;

/// HTTP routes served by factoryos-backend.
pub mod api {
    pub const HEALTH: &str = "/api/v1/health";
    pub const CONFIG: &str = "/api/v1/config";
    pub const SECRETS: &str = "/api/v1/secrets";

    /// `GET` a [`crate::scene::AssetRef`]'s bytes, `{sha256}` in lowercase hex. Immutable.
    pub const ASSET: &str = "/api/v1/assets/{sha256}";
    pub fn asset(sha256: &str) -> String {
        ASSET.replace("{sha256}", sha256)
    }

    pub const CELL_STATUS: &str = "/api/v1/cell/status";
    pub const CELL_START: &str = "/api/v1/cell/start";
    pub const CELL_STOP: &str = "/api/v1/cell/stop";
    pub const CELL_COMMAND: &str = "/api/v1/cell/command";
    pub const CELL_SAMPLES: &str = "/api/v1/cell/samples";
    pub const CELLS: &str = "/api/v1/cell/cells";
    pub const CELL_ACTIVE: &str = "/api/v1/cell/active";
    pub const CELL_CREATE: &str = "/api/v1/cell/create";
    pub const CELL_PLANS: &str = "/api/v1/cell/plans";
    /// `GET` one saved plan of the active cell, whole (blackboard included): `{name}` is its file
    /// name in `agentic_planner/plans/` without `.json` (see [`cell_plan`]). Weak ETag, `304` on
    /// `If-None-Match`, gzip when asked for.
    pub const CELL_PLAN: &str = "/api/v1/cell/plans/{name}";
    pub const CELL_PLAN_LOAD: &str = "/api/v1/cell/plan/load";
    pub const CELL_TASK: &str = "/api/v1/cell/task";
    pub const CELL_TASK_SAMPLE: &str = "/api/v1/cell/task/sample";

    pub const DRESSER_STATE: &str = "/api/v1/dresser/state";
    pub const DRESSER_OPEN: &str = "/api/v1/dresser/open";
    pub const DRESSER_PREVIEW: &str = "/api/v1/dresser/preview";
    pub const DRESSER_DRESS: &str = "/api/v1/dresser/dress";
    pub const DRESSER_VIEW: &str = "/api/v1/dresser/view";
    /// A tool's TCP (relative to its root link): `GET ?tool=<name>` returns
    /// [`crate::dresser::ToolTcp`]; `POST` [`crate::dresser::SetToolTcp`] saves it into the tool
    /// URDF, updates the dressed robots of the tool and returns [`crate::dresser::ToolTcpSaved`].
    pub const DRESSER_TOOL_TCP: &str = "/api/v1/dresser/tool_tcp";
    /// `POST` [`crate::dresser::SetToolTcp`]: draw that TCP in the Dresser's 3D view without
    /// saving it (live editing). Returns `()`.
    pub const DRESSER_TOOL_TCP_PREVIEW: &str = "/api/v1/dresser/tool_tcp/preview";
    /// `GET ?tool=<name>`: [`crate::dresser::ToolTipEstimate`], the snap-to-tool-tip point.
    pub const DRESSER_TOOL_TIP: &str = "/api/v1/dresser/tool_tip";
    /// [`crate::dresser::OrientSweepRequest`] → [`crate::dresser::OrientSweepResult`]: tool
    /// mountings ranked by the welds the robots reach with them (synchronous, up to ~10 min).
    pub const DRESSER_ORIENT_SWEEP: &str = "/api/v1/dresser/orient_sweep";

    pub const FITTER_STATE: &str = "/api/v1/fitter/state";
    pub const FITTER_OPEN: &str = "/api/v1/fitter/open";
    pub const FITTER_NEW: &str = "/api/v1/fitter/new";
    pub const FITTER_LOAD: &str = "/api/v1/fitter/load";
    pub const FITTER_SAVE: &str = "/api/v1/fitter/save";
    pub const FITTER_COMPOSE: &str = "/api/v1/fitter/compose";
    pub const FITTER_CREATE_CELL: &str = "/api/v1/fitter/create_cell";
    pub const FITTER_UNLINK: &str = "/api/v1/fitter/unlink";
    pub const FITTER_OPEN_CELL: &str = "/api/v1/fitter/open_cell";
    /// "Stop simulation and edit": stop the simulated cell-runner, and open the scene the linked
    /// cell was made from (or a new scene) for editing; an unlinked scene is only unlinked.
    pub const FITTER_EDIT: &str = "/api/v1/fitter/edit";
    /// Delete a saved scene ([`crate::fitter::LoadRequest`]: its file name).
    pub const FITTER_DELETE: &str = "/api/v1/fitter/delete";
    /// What the 3D view shows ([`crate::fitter::FitterView`]) → [`crate::fitter::FitterState`].
    pub const FITTER_VIEW: &str = "/api/v1/fitter/view";
    /// Start a reachability check ([`crate::fitter::ReachCheckRequest`]) →
    /// [`crate::fitter::FitterState`] at once, with `reach.running`; progress comes on
    /// [`crate::keys::FITTER_STATE`].
    pub const FITTER_REACH_CHECK: &str = "/api/v1/fitter/reach_check";
    /// Cancel the running check (`{}`) → [`crate::fitter::FitterState`].
    pub const FITTER_REACH_CANCEL: &str = "/api/v1/fitter/reach_cancel";
    /// Show the robots at a weld ([`crate::fitter::ReachPoseRequest`]) → [`crate::fitter::FitterState`].
    pub const FITTER_REACH_POSE: &str = "/api/v1/fitter/reach_pose";
    /// Check a scene's reachability, synchronously (up to 600 s):
    /// [`crate::fitter::ReachFileRequest`] → [`crate::fitter::ReachReport`].
    pub const REACH_CHECK: &str = "/api/v1/reach/check";

    /// Build training job ([`crate::training`]); each answers [`crate::training::TrainingState`].
    /// Open on the active cell (`{}`): its robots, products and welds, a suggested job, the 3D view.
    pub const TRAINING_OPEN: &str = "/api/v1/training/open";
    /// Show a cell drawn from a job, or the loaded cell ([`crate::training::PreviewRequest`]).
    pub const TRAINING_PREVIEW: &str = "/api/v1/training/preview";
    /// Save a job as a plan in the active cell ([`crate::training::SaveRequest`]).
    pub const TRAINING_SAVE: &str = "/api/v1/training/save";
    /// A saved job ([`crate::training::LoadJobRequest`]) → its [`crate::training::JobFile`].
    pub const TRAINING_LOAD: &str = "/api/v1/training/load";
    /// Suggest robot moves: [`crate::fitter::SuggestRequest`] → [`crate::fitter::SuggestResponse`].
    pub const REACH_SUGGEST: &str = "/api/v1/reach/suggest";
    /// Place 1..N robots around the products to reach the welds ([`crate::scene_edit::PlaceRobots`] →
    /// [`crate::scene_edit::PlaceRobotsResult`]).
    pub const REACH_PLACE: &str = "/api/v1/reach/place";
    /// How comfortably a scene's robots reach its welds ([`crate::scene_edit::LayoutQualityRequest`] →
    /// [`crate::scene_edit::LayoutQuality`]): per robot and weld, with the moves that fix it.
    pub const REACH_QUALITY: &str = "/api/v1/reach/quality";
    /// `GET` what a dressed robot is, from its URDF ([`crate::robot::RobotInfoQuery`] →
    /// [`crate::robot::RobotInfoList`]): joints, arm geometry, reach dome, tool, placement figures.
    pub const ROBOT_INFO: &str = "/api/v1/robot/info";
    /// `POST` a [`crate::view::ViewCommand`]: move the viewers' 3D view → [`crate::view::ViewCommandSent`].
    pub const VIEW_COMMAND: &str = "/api/v1/view/command";
    /// The cell's process definition ([`crate::process`]): `GET ?cell=` [`crate::process::ProcessView`];
    /// `PUT` [`crate::process::SaveProcess`] (validated, merged into the cell's plan2cell.toml).
    pub const PROCESS: &str = "/api/v1/process";
    /// `POST` [`crate::process::InitProcess`]: write the profile's sample `process.yaml`.
    pub const PROCESS_INIT: &str = "/api/v1/process/init";
    /// `GET` the simulated cell's latest state (`factoryos/sim/state`) and its behaviour tree playback.
    pub const CELL_SNAPSHOT: &str = "/api/v1/cell/snapshot";
    /// Scene edits ([`crate::scene_edit`]), each answering [`crate::scene_edit::SceneEdited`]: a new
    /// empty scene, then one product or robot at a time on `scenes/{file}`.
    pub const SCENE_CREATE: &str = "/api/v1/scene/create";
    /// `GET` a scene file as [`crate::fitter::SceneFile`] (with `?file=` in `scenes/`).
    pub const SCENE_GET: &str = "/api/v1/scene/get";
    /// `GET` what the Robo Fitter shows now, in brief: the scene's file, its linked cell (read-only
    /// while linked), robots, fixtures, the welds' reach (domes, and the last check) — JSON.
    pub const SCENE_CURRENT: &str = "/api/v1/scene/current";
    pub const SCENE_PLACE_PRODUCT: &str = "/api/v1/scene/{file}/place_product";
    pub const SCENE_ROBOT_ADD: &str = "/api/v1/scene/{file}/robots/add";
    pub const SCENE_ROBOT_MOVE: &str = "/api/v1/scene/{file}/robots/move";
    pub const SCENE_ROBOT_REMOVE: &str = "/api/v1/scene/{file}/robots/remove";
    pub const SCENE_FIXTURE_REMOVE: &str = "/api/v1/scene/{file}/fixtures/remove";

    pub const PRODUCT_LIST: &str = "/api/v1/product/list";
    pub const PRODUCT_LOAD: &str = "/api/v1/product/load";
    pub const PRODUCT_SAVE: &str = "/api/v1/product/save";
    pub const PRODUCT_PREVIEW: &str = "/api/v1/product/preview";
    /// The catalogue's ready-made products → [`crate::product::CatalogueProducts`].
    pub const PRODUCT_CATALOGUE: &str = "/api/v1/product/catalogue";
    /// Copy a catalogue product into the products dir ([`crate::product::CatalogueTransfer`]) →
    /// its [`crate::product::ProductFile`].
    pub const PRODUCT_CATALOGUE_LOAD: &str = "/api/v1/product/catalogue/load";
    /// Copy a saved product into the catalogue ([`crate::product::CatalogueTransfer`]).
    pub const PRODUCT_CATALOGUE_PUBLISH: &str = "/api/v1/product/catalogue/publish";
    /// The catalogue's fixtures (names), whose URDFs a product can use.
    pub const PRODUCT_FIXTURES: &str = "/api/v1/product/fixtures";
    /// [`crate::product::UrdfFromCatalogue`] → [`crate::product::ProductUrdfs`].
    pub const PRODUCT_URDF_FROM_CATALOGUE: &str = "/api/v1/product/urdf/from_catalogue";
    /// `PUT` a zip (a URDF and its meshes), extracted into the product's `urdf/` folder →
    /// [`crate::product::ProductUrdfs`]. See [`product_urdf_upload`].
    pub const PRODUCT_URDF_UPLOAD: &str = "/api/v1/product/{name}/urdf";
    /// Weld edits on a stored product ([`crate::product_edit`]), each answering
    /// [`crate::product_edit::WeldEdit`] (the product saved, its cells' task programs derived again):
    /// `add` [`crate::product_edit::AddWelds`], `remove` [`crate::product_edit::RemoveWelds`], `move`
    /// [`crate::product_edit::MoveWeld`], `regroup` [`crate::product_edit::RegroupWelds`], `generate`
    /// [`crate::product_edit::GenerateWelds`].
    pub const PRODUCT_WELDS_ADD: &str = "/api/v1/product/{name}/welds/add";
    pub const PRODUCT_WELDS_REMOVE: &str = "/api/v1/product/{name}/welds/remove";
    pub const PRODUCT_WELDS_MOVE: &str = "/api/v1/product/{name}/welds/move";
    pub const PRODUCT_WELDS_REGROUP: &str = "/api/v1/product/{name}/welds/regroup";
    pub const PRODUCT_WELDS_GENERATE: &str = "/api/v1/product/{name}/welds/generate";
    /// [`crate::product_edit::CheckWelds`] → [`crate::product_edit::WeldCheck`] (read-only).
    pub const PRODUCT_WELDS_CHECK: &str = "/api/v1/product/{name}/welds/check";
    /// [`crate::product_edit::SnapRequest`] → [`crate::product_edit::SnapResult`] (read-only).
    pub const PRODUCT_SNAP: &str = "/api/v1/product/{name}/snap";

    pub const AGENTIC_STATUS: &str = "/api/v1/agentic/status";
    pub const AGENTIC_START: &str = "/api/v1/agentic/start";
    pub const AGENTIC_STOP: &str = "/api/v1/agentic/stop";
    pub const AGENTIC_PREPARE: &str = "/api/v1/agentic/prepare";

    /// Control Code Export (see [`crate::codegen`]): `GET` [`crate::codegen::CodegenOptions`];
    /// `503` without a plan2cell binary.
    pub const CODEGEN_OPTIONS: &str = "/api/v1/codegen/options";
    /// `POST` a [`crate::codegen::BuildRequest`]: `202` [`crate::codegen::BuildAccepted`], `409`
    /// while a build runs.
    pub const CODEGEN_BUILD: &str = "/api/v1/codegen/build";
    /// `GET` the active cell's builds ([`crate::codegen::BuildList`]), newest first.
    pub const CODEGEN_BUILDS: &str = "/api/v1/codegen/builds";
    /// `GET` one build ([`crate::codegen::BuildInfo`]).
    pub const CODEGEN_BUILD_INFO: &str = "/api/v1/codegen/builds/{id}";
    /// `GET` one file of a build, raw (`text/plain` or `application/octet-stream`).
    pub const CODEGEN_BUILD_FILE: &str = "/api/v1/codegen/builds/{id}/files/{*path}";
    /// `GET` the whole build as a zip (`<id>.zip`).
    pub const CODEGEN_BUILD_ARCHIVE: &str = "/api/v1/codegen/builds/{id}/archive";

    /// Workspace IDE (see [`crate::files`]): `GET` the roots ([`crate::files::FileRoot`]s).
    pub const FILES_ROOTS: &str = "/api/v1/files/roots";
    /// `GET` a root's files ([`crate::files::FileTree`]).
    pub const FILES_TREE: &str = "/api/v1/files/{root}/tree";
    /// `GET` a text file ([`crate::files::FileText`]); `PUT` a [`crate::files::FileSave`]: `200`
    /// [`crate::files::FileSaved`], `409` changed on disk meanwhile.
    pub const FILES_FILE: &str = "/api/v1/files/{root}/file/{*path}";
    /// `GET` a file as it is, to download (`Content-Disposition: attachment`); `PUT` its bytes, an upload:
    /// `200` its [`crate::files::FileEntry`], `409` if it exists (unless `?overwrite=1`) or is a folder.
    pub const FILES_RAW: &str = "/api/v1/files/{root}/raw/{*path}";

    /// Workspace IDE terminals (see [`crate::terminal`]): `GET` the hosts ([`crate::terminal::TerminalHost`]s).
    pub const TERMINAL_HOSTS: &str = "/api/v1/terminals/hosts";
    /// `GET` the sessions ([`crate::terminal::TerminalInfo`]s); `POST` a [`crate::terminal::NewTerminal`]:
    /// `200` its [`crate::terminal::TerminalInfo`].
    pub const TERMINALS: &str = "/api/v1/terminals";
    /// `DELETE` a session (its shell is killed).
    pub const TERMINAL: &str = "/api/v1/terminals/{id}";
    /// `GET` upgrades to the session's WebSocket.
    pub const TERMINAL_WS: &str = "/api/v1/terminals/{id}/ws";
    /// A terminal agent's link ([`crate::terminal::link`]), `GET` upgrading to a WebSocket; only with the
    /// edge's `X-Terminal-Principal` (the agent's verified IAM identity) for that name.
    pub const TERMINAL_LINK: &str = "/terminal-link/{name}";

    /// Hardware Configuration (see [`crate::hardware`]): `GET` [`crate::hardware::HardwareView`]
    /// (`?plan=<plan>`, else the newest plan); `PUT` a [`crate::hardware::HardwareSave`]: `200` the
    /// new view, `422` [`crate::hardware::HardwareRejected`].
    pub const CELL_HARDWARE: &str = "/api/v1/cell/hardware";

    /// The Virtual Cell (see [`crate::infra`]): `GET` [`crate::infra::InfraStatus`].
    pub const INFRA_STATUS: &str = "/api/v1/infra/status";
    /// `POST` ([`crate::infra::StartStopRequest`], optional): start the cell's instances.
    pub const INFRA_START: &str = "/api/v1/infra/start";
    /// `POST` ([`crate::infra::StartStopRequest`], optional): stop them (never terminates).
    pub const INFRA_STOP: &str = "/api/v1/infra/stop";
    /// `POST` ([`crate::infra::BringupRequest`], optional): `202`, the KUKA VC bring-up.
    pub const INFRA_BRINGUP: &str = "/api/v1/infra/bringup";
    /// The power of the whole virtual cell (the cell-control Lambda's `power_*`): `GET`
    /// [`crate::infra::CellPower`] (read from the Lambda); `POST` [`crate::infra::CellPowerRequest`]
    /// (`{"action": "on" | "off", "force"?, "production"?}`): [`crate::infra::InfraStatus`] with
    /// [`crate::infra::InfraStatus::cell_power`]. `on` answers at once (the Lambda boots, brings up
    /// and checks the cell; see the phases); `off` is `409` while the cell is in production unless
    /// `force`.
    pub const INFRA_CELL_POWER: &str = "/api/v1/infra/cell/power";
    /// The workspace's own virtual cells (`factoryos-cell:ws-<alias>`): `GET`
    /// [`crate::infra::CellsList`]; `POST` [`crate::infra::CreateCellRequest`]: `202`
    /// [`crate::infra::VirtualCell`] (lifecycle `off`), `409` one exists, `429` the account's cap.
    /// [`INFRA_CELL_POWER`] acts on the bound cell (own or shared).
    pub const INFRA_CELLS: &str = "/api/v1/infra/cells";
    /// `DELETE` ([`crate::infra::DeleteCellRequest`]): `202` [`crate::infra::VirtualCell`]
    /// (lifecycle `deleting`), `400` the confirm does not match, `404` no such cell. `PATCH` / `PUT`
    /// ([`crate::infra::UpdateCellRequest`]): [`crate::infra::VirtualCell`] (its off mode, its
    /// robots while off).
    pub const INFRA_CELL_ITEM: &str = "/api/v1/infra/cells/{id}";
    /// `GET` [`crate::infra::CellTemplates`]: the virtual-cell templates this workspace may create a
    /// cell from (the `factoryos-cell` Lambda's `templates`, read at most every 30 s); `available:
    /// false` without an own cell binding or with a Lambda that has no templates (then the runtimes
    /// of [`INFRA_CELLS`]).
    pub const INFRA_TEMPLATES: &str = "/api/v1/infra/templates";
    /// `POST` ([`crate::infra::ProductionRequest`], optional): `202`, `plan2cell cell start
    /// --restart-runtime` on the export the head runs (of its PLAN_ID), or on `build`.
    pub const INFRA_PRODUCTION: &str = "/api/v1/infra/production";
    /// `POST` (no body): `202`, Stop production: `plan2cell cell stop` on the export the head runs
    /// (pulses `PLC_PRG.bStop`: the head ends in `CS_ERROR ERR_OPERATOR_STOP`; the instances keep
    /// running, Start production starts it again).
    pub const INFRA_PRODUCTION_STOP: &str = "/api/v1/infra/production/stop";
    /// `POST` ([`crate::infra::DeployRequest`], optional): `202`, Run Control Code: `plan2cell cell
    /// projects`, `deploy`, `krl`, `start --restart-runtime --pulses 16` on an export (default:
    /// the newest); `{"start": false}`: Deploy only, without `start`.
    pub const INFRA_DEPLOY: &str = "/api/v1/infra/deploy";
    /// `POST` ([`crate::deploy_check::DeployCheckRequest`], optional): [`crate::deploy_check::DeployCheck`],
    /// the deployability check of an export (read-only, seconds). [`INFRA_DEPLOY`] runs it first:
    /// `409` [`crate::deploy_check::DeployRefused`] when a rule fails, unless `force`.
    pub const INFRA_DEPLOY_CHECK: &str = "/api/v1/infra/deploy/check";
    /// `POST` [`crate::deploy_check::DeployFixRequest`]: [`crate::deploy_check::DeployFixResult`], the
    /// failing rules' fixes applied (Hardware Configuration saved, a backup of the previous one;
    /// export options), the plan exported again and checked.
    pub const INFRA_DEPLOY_FIX: &str = "/api/v1/infra/deploy/fix";
    /// `POST` [`crate::infra::KvcRequest`]: one operation on one KUKA VC controller (`plan2cell cell
    /// kvc`, over its SSM / kubectl remote): its JSON answer; `409` while another cell job runs (for
    /// an operation that changes the controller), `400` an operation not on the list.
    pub const INFRA_KVC: &str = "/api/v1/infra/kvc";
    /// [`INFRA_KVC`] for the operations that only read (`status`, `mode show`, `program show`,
    /// `braketest-need`); any other is `400`.
    pub const INFRA_KVC_READ: &str = "/api/v1/infra/kvc/read";
    /// `GET` [`crate::infra::AutoRestartStatus`]; `PUT` [`crate::infra::AutoRestartSettings`]: the
    /// active cell's auto-restart before the CODESYS demo stop.
    pub const INFRA_AUTO_RESTART: &str = "/api/v1/infra/auto-restart";
    /// `POST` (`Content-Type: application/json`, same origin only; optional body
    /// [`crate::infra::DcvSessionRequest`]): [`crate::infra::DcvSession`], a token for a DCV
    /// target's session (default `ide`, the Windows IDE's console); `503` when the backend has no
    /// DCV Connection Gateway, `404` for a target it doesn't have. `DELETE`:
    /// [`crate::infra::DcvRevoked`], every token revoked.
    pub const INFRA_DCV_SESSION: &str = "/api/v1/infra/dcv/session";
    /// NICE DCV web clients (any method, WebSocket upgrades too), proxied to the backend machine's
    /// DCV Connection Gateway of each target: `/dcv/<target>/…` is that gateway's `/…` (see
    /// [`dcv_target`]). `/dcv/…` without a target is the `ide` target's (older links).
    pub const DCV: &str = "/dcv";

    /// `/dcv/<target>/`: the DCV web client of `target` (e.g. `ide`).
    pub fn dcv_target(target: &str) -> String {
        format!("{DCV}/{target}/")
    }

    /// WebRTC livestreams (NVIDIA Isaac Sim's web viewer and its signaling WebSocket), proxied as
    /// [`DCV`] to the backend machine's loopback relay of each target: `/rtc/<target>/…` is that
    /// relay's `/…` (see [`rtc_target`]). The media (UDP) goes to the stream's host directly.
    pub const RTC: &str = "/rtc";

    /// An Isaac Sim instance's Zenoh link (a WebSocket, `/isaac-link/<name>`), connected outbound
    /// through the edge with an IAM identity it verified (`X-Isaac-Principal`), proxied to the
    /// instance's bridge on the backend machine.
    pub const ISAAC_LINK: &str = "/isaac-link";

    /// `GET`: [`crate::rtc::RtcTargets`], the targets of [`RTC`] (none: no Isaac Sim linked).
    pub const RTC_TARGETS: &str = "/api/v1/rtc/targets";

    /// `/rtc/<target>/`: the web viewer of `target` (e.g. `isaac`).
    pub fn rtc_target(target: &str) -> String {
        format!("{RTC}/{target}/")
    }

    /// The cell's KUKA smartPADs (any method, WebSocket upgrades too), proxied to each controller's KDM in the cell:
    /// `/smartpad/<robot>/…` is `https://<its KLI address>/…`. Only behind the workspace edge, which maps the
    /// smartPAD's own hostname to it ([`crate::smartpad`]): iiQKA.UI does not work under a path prefix.
    pub const SMARTPAD: &str = "/smartpad";

    /// `GET`: [`crate::smartpad::SmartPads`], the cell's controllers and their smartPAD URLs.
    pub const SMARTPADS: &str = "/api/v1/smartpads";

    /// The planner agent's sign-in ([`crate::agent::AgentLogin`]): `GET` the state, `POST` to
    /// start a device-flow login, `DELETE` to sign out.
    pub const AGENT_LOGIN: &str = "/api/v1/agent/login";

    /// WebSocket upgrade, proxied to the Zenoh remote-api plugin (zenoh-ts clients).
    pub const ZENOH_WS: &str = "/zenoh";
    /// WebSocket upgrade, proxied to the router's own `ws` listener: native viewers' Zenoh
    /// sessions, through a local bridge (Zenoh's `ws/<host>:<port>` can't carry a path).
    pub const ZENOH_NATIVE_WS: &str = "/zenoh-native";
    /// WebSocket upgrade for the Agentic Planning chat (see [`crate::chat`]): `?session=<id>`
    /// attaches to that session ([`chat_ws`]), none to [`crate::chat::DEFAULT_SESSION`].
    pub const CHAT_WS: &str = "/chat";
    /// The agent sessions (the chat's tabs): `GET` [`crate::chat::ChatSessions`]; `POST`
    /// [`crate::chat::NewChatSession`] → [`crate::chat::ChatSessionInfo`], `409` with the reason
    /// at the session limit.
    pub const CHAT_SESSIONS: &str = "/api/v1/chat/sessions";
    /// `DELETE`: close a session and stop its agent ([`chat_session`]); `204`, `404` if unknown.
    pub const CHAT_SESSION: &str = "/api/v1/chat/sessions/{id}";

    /// [`CHAT_WS`] for session `id`.
    pub fn chat_ws(id: &str) -> String {
        format!("{CHAT_WS}?session={}", crate::codegen::encode_path(id))
    }

    /// [`CHAT_SESSION`] for session `id`.
    pub fn chat_session(id: &str) -> String {
        CHAT_SESSION.replace("{id}", &crate::codegen::encode_path(id))
    }

    /// [`CELL_PLAN`] for the plan `name`.
    pub fn cell_plan(name: &str) -> String {
        CELL_PLAN.replace("{name}", name)
    }

    /// [`CELL_HARDWARE`] for `plan` (a [`crate::PlanInfo::path`] or name; `None`: the newest).
    pub fn cell_hardware(plan: Option<&str>) -> String {
        match plan.filter(|p| !p.is_empty()) {
            Some(plan) => format!("{CELL_HARDWARE}?plan={}", crate::codegen::encode_path(plan)),
            None => CELL_HARDWARE.to_owned(),
        }
    }

    /// [`CODEGEN_BUILD_INFO`] for build `id`.
    pub fn codegen_build(id: &str) -> String {
        CODEGEN_BUILD_INFO.replace("{id}", id)
    }

    /// [`CODEGEN_BUILD_FILE`] for the file `path` (relative, `/`-separated) of build `id`.
    pub fn codegen_file(id: &str, path: &str) -> String {
        CODEGEN_BUILD_FILE.replace("{id}", id).replace("{*path}", &crate::codegen::encode_path(path))
    }

    /// [`FILES_TREE`] for `root`.
    pub fn files_tree(root: &str) -> String {
        FILES_TREE.replace("{root}", &crate::codegen::encode_path(root))
    }

    /// [`FILES_TREE`] for one folder `dir` of `root` (`""`: its top), not what is below it.
    pub fn files_tree_dir(root: &str, dir: &str) -> String {
        format!("{}?dir={}", files_tree(root), crate::codegen::encode_path(dir))
    }

    /// [`FILES_RAW`] for the file `path` of `root`; `overwrite`: an upload replaces an existing file.
    pub fn files_raw(root: &str, path: &str, overwrite: bool) -> String {
        let url = FILES_RAW.replace("{root}", &crate::codegen::encode_path(root)).replace("{*path}", &crate::codegen::encode_path(path));
        if overwrite { format!("{url}?overwrite=1") } else { url }
    }

    /// [`PRODUCT_URDF_UPLOAD`] for product `name`.
    pub fn product_urdf_upload(name: &str) -> String {
        PRODUCT_URDF_UPLOAD.replace("{name}", &crate::codegen::encode_path(name))
    }

    /// [`FILES_FILE`] for the file `path` (relative, `/`-separated) of `root`.
    pub fn files_file(root: &str, path: &str) -> String {
        FILES_FILE.replace("{root}", &crate::codegen::encode_path(root)).replace("{*path}", &crate::codegen::encode_path(path))
    }

    /// [`TERMINAL`] for session `id`.
    pub fn terminal(id: &str) -> String {
        TERMINAL.replace("{id}", &crate::codegen::encode_path(id))
    }

    /// [`TERMINAL_WS`] for session `id`.
    pub fn terminal_ws(id: &str) -> String {
        TERMINAL_WS.replace("{id}", &crate::codegen::encode_path(id))
    }

    /// [`CODEGEN_BUILD_ARCHIVE`] for build `id`.
    pub fn codegen_archive(id: &str) -> String {
        CODEGEN_BUILD_ARCHIVE.replace("{id}", id)
    }
}

/// Zenoh key expressions.
pub mod keys {
    /// Latest [`crate::Config`] (JSON), published on every change. Also a queryable.
    pub const CONFIG: &str = "factoryos/config";

    /// The LIVE cell's latest [`crate::CellState`] (JSON), from fos-live-bridge on a server
    /// connected to the virtual cell (fos-live-scene answers queries). The backend does not write it.
    pub const CELL_STATE: &str = "factoryos/cell/state";
    /// The LIVE cell's [`crate::BackendStatus`] (fos-live-bridge / fos-live-scene). The backend
    /// does not write it.
    pub const CELL_STATUS: &str = "factoryos/cell/status";
    /// Unused since protocol v3 (the backend's simulation pushes [`SIM_POSITIONS`]).
    pub const CELL_POSITIONS: &str = "factoryos/cell/positions";
    /// Unused since protocol v3: commands for the simulation go to [`SIM_COMMAND`].
    pub const CELL_COMMAND: &str = "factoryos/cell/cmd";

    /// The backend's simulated cell-runner (Load & Play in step 1): its latest
    /// [`crate::CellState`] (JSON). Also a queryable, so late joiners get it immediately.
    pub const SIM_STATE: &str = "factoryos/sim/state";
    /// The simulation's [`crate::BackendStatus`] (JSON), published whenever it changes. Also a
    /// queryable.
    pub const SIM_STATUS: &str = "factoryos/sim/status";
    /// The simulated cell-runner's pushed `positions` frames, forwarded verbatim (JSON).
    pub const SIM_POSITIONS: &str = "factoryos/sim/positions";
    /// Queryable: [`crate::CellCommand`] in, [`crate::CellResponse`] out, for the simulation.
    pub const SIM_COMMAND: &str = "factoryos/sim/cmd";
    /// Backend log lines (UTF-8), one per sample.
    pub const LOG: &str = "factoryos/log";

    /// [`crate::dresser::DresserState`] (JSON). Also a queryable.
    pub const DRESSER_STATE: &str = "factoryos/dresser/state";
    /// A [`crate::storage::CatalogueChanged`] (JSON) whenever the shared catalogue changed.
    pub const CATALOGUE_CHANGED: &str = "factoryos/catalogue/changed";
    /// A [`crate::view::ViewCommand`] for the viewers' 3D view (from `POST /api/v1/view/command`).
    pub const VIEW_COMMAND: &str = "factoryos/view/command";
    /// Put [`crate::dresser::JointUpdate`] (JSON) here to move a joint of the dresser preview.
    pub const DRESSER_JOINT: &str = "factoryos/dresser/joint";
    /// [`crate::fitter::FitterState`] (JSON). Also a queryable.
    pub const FITTER_STATE: &str = "factoryos/fitter/state";
    /// Put [`crate::fitter::SceneEdit`] (JSON) here on every edit; the 3D view follows.
    pub const FITTER_SCENE: &str = "factoryos/fitter/scene";

    /// [`crate::agentic::AgenticStatus`] (JSON). Also a queryable.
    pub const AGENTIC_STATUS: &str = "factoryos/agentic/status";

    /// [`crate::codegen::CodegenStatus`] (JSON): the Control Code Export build. Also a queryable.
    pub const CODEGEN_STATUS: &str = "factoryos/codegen/status";

    /// [`crate::infra::InfraStatus`] (JSON): the Virtual Cell. Also a queryable.
    pub const INFRA_STATUS: &str = "factoryos/infra/status";

    /// [`crate::scene::SceneMsg`] (JSON) for `source` ([`SCENE_CELL`], [`SCENE_DRESSER`], …).
    pub fn scene(source: &str) -> String {
        format!("factoryos/scene/{source}")
    }

    /// Queryable: the latest [`crate::scene::SceneDoc`] (JSON) of `source`.
    pub fn scene_snapshot(source: &str) -> String {
        format!("factoryos/scene/{source}/snapshot")
    }

    /// [`crate::scene::JointFrame`]s (JSON) of `source`.
    pub fn scene_joints(source: &str) -> String {
        format!("factoryos/scene/{source}/joints")
    }

    /// [`crate::scene::PoseFrame`]s (JSON) of `source`.
    pub fn scene_poses(source: &str) -> String {
        format!("factoryos/scene/{source}/poses")
    }

    /// The backend process's boot id (JSON string), published at startup and a queryable: a
    /// changed id means the backend restarted (its in-memory state, e.g. asset registrations from
    /// other producers, may be gone).
    pub const BACKEND_BOOT: &str = "factoryos/backend/boot";

    /// Liveliness tokens: the backend declares [`backend_alive`] once it serves, so a client sees
    /// it appear on joining and again after every re-join (a restarted backend, a reconnect),
    /// when samples published meanwhile were missed and what it follows wants querying again.
    pub const BACKEND_ALIVE: &str = "factoryos/backend/alive/*";

    /// The backend's liveliness token: [`BACKEND_ALIVE`] with its boot id ([`BACKEND_BOOT`]).
    pub fn backend_alive(boot_id: &str) -> String {
        format!("factoryos/backend/alive/{boot_id}")
    }

    /// The LIVE cell (fos-live-scene); the backend does not write it.
    pub const SCENE_CELL: &str = "cell";
    pub const SCENE_DRESSER: &str = "dresser";
    pub const SCENE_FITTER: &str = "fitter";
    pub const SCENE_PRODUCT: &str = "product";
    pub const SCENE_TRAINING: &str = "training";

    /// Queryable: a producer on this machine that is not the backend (fos-live-scene) registers
    /// the files its scenes reference, a JSON list of absolute paths in, their
    /// [`crate::scene::AssetRef`]s out (see the backend's `services/asset_register.rs`).
    pub const ASSETS_REGISTER: &str = "factoryos/assets/register";

    /// Progress and log lines of a dresser/fitter job.
    pub fn job_progress(id: &str) -> String {
        format!("factoryos/jobs/{id}/progress")
    }
}

/// `GET /api/v1/health`.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Health {
    pub protocol_version: u32,
    pub backend_version: String,
    /// Zenoh endpoints native viewers can connect to, e.g. `tcp/127.0.0.1:7447`.
    pub zenoh_endpoints: Vec<String>,
    /// Whether native viewers can also open their Zenoh session over a WebSocket to this
    /// backend's own HTTP port ([`api::ZENOH_NATIVE_WS`]), for when `zenoh_endpoints` aren't
    /// reachable, e.g. a server behind a load balancer.
    #[serde(default)]
    pub zenoh_ws: bool,
    /// The workspace this backend serves (`--workspace`), if it's pinned to one. Unpinned, it
    /// answers under any workspace's prefix.
    #[serde(default)]
    pub workspace: Option<String>,
    /// The release (CI build id, e.g. `p1234` or a tag) this backend was deployed from, if known.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub release: Option<String>,
    /// Seconds since the last request other than health checks (for idle shutdown).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub idle_secs: Option<u64>,
    /// Open viewer WebSockets (Zenoh, chat): viewers that are connected but quiet.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub open_connections: Option<u32>,
    /// This backend process's boot id, as on [`keys::BACKEND_BOOT`].
    #[serde(default)]
    pub boot_id: String,
}
