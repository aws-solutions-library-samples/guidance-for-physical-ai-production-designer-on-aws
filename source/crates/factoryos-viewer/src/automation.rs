//! Opt-in automation for visual smoke tests (`scripts/visual_smoke.py`) and the browser E2E flow
//! (`scripts/browser_e2e.py`).
//!
//! Natively, with `FACTORY_OS_AUTOMATION=stdin` the viewer reads newline-delimited JSON commands
//! on stdin and answers each with one line on stdout, prefixed with [`REPLY_PREFIX`] (logs go to
//! stderr, but the prefix keeps replies apart from anything else that may print to stdout).
//!
//! On the web, with `?automation=1` in the page's URL, the page calls `automation(line)` (the wasm
//! module's export, [`submit`]) with one command and gets every reply line, prefix included, as
//! the argument of `globalThis.__factoryosAutomationReply` ([`JS_REPLY`]); `web/index.html` turns
//! that into `window.__factoryos.automation(command) -> Promise<reply>`. Screenshots come back in
//! the reply (`png_base64`), not written to `path`.
//!
//! Without the variable (or the query) nothing here runs and nothing changes.
//!
//! **No network listener of any kind.** Natively, commands come only on this process's stdin,
//! which only the parent process that started the viewer can write to; on the web, only from
//! script in the page itself. Nothing is bound or opened. When stdin closes (the parent went
//! away), the native viewer quits.
//!
//! Commands (an optional `"id"` is echoed back in the reply):
//!
//! * `{"cmd":"open","tool":"dresser"|"fitter"|"product"|"training"|"live"}`: what the sidebar's
//!   buttons do.
//! * `{"cmd":"wait_idle","timeout_s":30,"settle_s":0.5}`: replies once the shown page's 3D scene
//!   ([`crate::scene3d::Scene3d::pending`]) has nothing on its way, Rerun shows its recording and
//!   has ingested everything, for `settle_s` in a row; or with `"ok":false` and the reason at the
//!   timeout.
//! * `{"cmd":"screenshot","path":"/abs/out.png","region":"window"|"3d"}`: a PNG of the window (in
//!   physical pixels), or cropped to the 3D view ([`view_3d_rect`]). Replies with the path, the
//!   image's size and the crop.
//! * `{"cmd":"state"}`: the page, the tool Rerun shows, the active recording, the scene's title.
//! * `{"cmd":"set_window","width":1600,"height":1000}`: the window's inner size, in points.
//! * `{"cmd":"product_select","name":"weld_box","from":"catalogue"|"products"}`: Product & Task on
//!   that product, through the panel's own load (a catalogue row's click, or a saved product's
//!   Edit), so its own preview shows it instead of fighting with one set from outside.
//! * `{"cmd":"scene","nodes":["robots/robot_2"],"since":0}`: read-only facts of the recording
//!   Rerun shows ([`scene_facts`]): its id, entity and chunk counts, its size in bytes, any
//!   temporal chunks; every mesh entity (visual and collision geometry) with what it holds (a
//!   blob's SHA-256 and media type, or a `Mesh3D`); for each of `nodes`, its loaded/pending
//!   meshes and its links' world positions. Plus the mesh history ([`MeshHistory`], events after
//!   `since`) and the recordings the app has asked Rerun to close.
//! * `{"cmd":"camera"}`: read-only, the shown page's 3D view's eye as Rerun has it (its active
//!   blueprint's `EyeControls3D`: position, look target, up, a tracked entity) and the orbit
//!   centre that gives (the look target; none while it tracks an entity), the eye fitted to the
//!   scene and the bounds it was fitted to; and where the bounds' centre and corners are in the
//!   view ([`camera_facts`]).
//! * `{"cmd":"drag","dx":80,"dy":0,"button":"primary"|"secondary","steps":8}`: a mouse drag from
//!   the 3D view's centre by (`dx`, `dy`) points (primary: orbit, secondary: pan), injected into
//!   egui's input over `steps` frames ([`PendingPointer`]); replies once released.
//! * `{"cmd":"double_click","at":[0.03,0.5]}`: a double click at that point of the 3D view (as
//!   shares of its width and height; the default is near its left edge, clear of the scene, so
//!   that it resets the camera rather than focusing an entity); replies once done.
//! * `{"cmd":"real_pointer","on":true}`: let the real mouse through again (or not): a browser test
//!   that clicks and drags with the browser's own input.
//! * `{"cmd":"action","action":{"name":"use_scene","scene":"cell_a","open":true}}`: what a sidebar
//!   button does ([`SidebarCommand`]).
//! * `{"cmd":"export_code","plan":"agentic_planner/plans/bt_1.json"}`: step 2's "Export PLC +
//!   robot code" pressed (on that plan, else the form's), as soon as it is enabled; replies at once.
//! * `{"cmd":"quit"}`.
//!
//! While automation is on, the real mouse is ignored ([`without_pointer`]) until `real_pointer`:
//! wherever the pointer happens to be, it doesn't hover the 3D view (tooltips, highlights) or move
//! its camera, so the screenshots only show what the commands did (`drag` included).

#[cfg(not(target_arch = "wasm32"))]
use std::io::{BufRead as _, Write as _};
use std::path::PathBuf;
use std::sync::mpsc;

use serde_json::{Value, json};
use web_time::{Duration, Instant};

use crate::rr::egui;
use crate::rr::re_chunk::{EntityPath, RowId};
use crate::rr::re_chunk_store::{ChunkStoreGeneration, LatestAtQuery};
use crate::rr::re_entity_db::EntityDb;
use crate::rr::re_log_types::{StoreId, TimelineName};
use crate::rr::re_sdk_types::archetypes::{Asset3D, CoordinateFrame, Mesh3D};
use crate::rr::re_sdk_types::components::{Blob, MediaType, TransformFrameId};
use crate::rr::re_tf::{TransformForest, TransformFrameIdHash, TransformResolutionCache};

/// The environment variable that turns automation on (with the value `stdin`).
pub const ENV: &str = "FACTORY_OS_AUTOMATION";
/// On the web: the global function that gets the reply lines.
pub const JS_REPLY: &str = "__factoryosAutomationReply";
/// Every reply line starts with this.
pub const REPLY_PREFIX: &str = "@@automation ";
/// A screenshot that has not come back by then fails (e.g. a minimized window draws nothing).
const SCREENSHOT_TIMEOUT: Duration = Duration::from_secs(15);
/// How often the UI wakes while a wait or a screenshot is pending (egui only repaints on demand).
const POLL: Duration = Duration::from_millis(30);
/// Closed recordings kept for `scene`.
const MAX_CLOSED: usize = 64;
/// Mesh history events kept (the oldest go first).
const MAX_HISTORY: usize = 20_000;
/// Temporal entities named in `scene` (all are counted).
const MAX_TEMPORAL_NAMED: usize = 16;

/// Whether automation is on (`FACTORY_OS_AUTOMATION=stdin`).
#[cfg(not(target_arch = "wasm32"))]
pub fn enabled() -> bool {
    std::env::var(ENV).as_deref() == Ok("stdin")
}

/// Whether automation is on (`automation=1` in the page's query).
#[cfg(target_arch = "wasm32")]
pub fn enabled() -> bool {
    let search = web_sys::window().and_then(|window| window.location().search().ok()).unwrap_or_default();
    query_enables(&search)
}

/// Whether a URL query (`?a=b&automation=1`) asks for automation.
pub fn query_enables(search: &str) -> bool {
    search.trim_start_matches('?').split('&').any(|pair| pair == "automation=1")
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Tool {
    Dresser,
    Fitter,
    Product,
    Training,
    Live,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Deserialize)]
pub enum Region {
    #[default]
    #[serde(rename = "window")]
    Window,
    #[serde(rename = "3d")]
    View3d,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProductSource {
    /// A product saved in the products folder (a product-derived task program's Edit).
    #[default]
    Products,
    /// A ready-made product of the catalogue, copied into the products folder first.
    Catalogue,
}

#[derive(Clone, Debug, PartialEq, serde::Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case", deny_unknown_fields)]
pub enum Command {
    Open { tool: Tool },
    WaitIdle {
        #[serde(default = "default_timeout_s")]
        timeout_s: f64,
        #[serde(default = "default_settle_s")]
        settle_s: f64,
    },
    Screenshot {
        path: PathBuf,
        #[serde(default)]
        region: Region,
    },
    State {},
    SetWindow { width: f32, height: f32 },
    ProductSelect {
        name: String,
        #[serde(default)]
        from: ProductSource,
    },
    Scene {
        /// Scene node ids (`robots/robot_2`) to report meshes and link positions for.
        #[serde(default)]
        nodes: Vec<String>,
        /// Only mesh history events with a greater `seq`.
        #[serde(default)]
        since: u64,
    },
    Camera {},
    DoubleClick {
        #[serde(default = "default_double_click_at")]
        at: [f32; 2],
    },
    Drag {
        dx: f32,
        dy: f32,
        #[serde(default)]
        button: DragButton,
        #[serde(default = "default_drag_steps")]
        steps: u32,
    },
    RealPointer { on: bool },
    Action { action: SidebarCommand },
    ExportCode {
        #[serde(default)]
        plan: Option<String>,
    },
    Quit {},
}

/// A sidebar button's action (`action`), by name: `{"name":"open_cell","cell":"cell_a"}`.
#[derive(Clone, Debug, PartialEq, serde::Deserialize)]
#[serde(tag = "name", rename_all = "snake_case", deny_unknown_fields)]
pub enum SidebarCommand {
    OpenProduct {},
    OpenDresser {},
    OpenFitter {},
    OpenTrainingJob {},
    WatchCell {},
    EditScene {},
    RefreshCells {},
    RefreshPlans {},
    OpenCell { cell: String },
    SetActiveCell { cell: String },
    UseScene {
        scene: String,
        #[serde(default)]
        open: bool,
    },
    LoadPlan {
        path: String,
        #[serde(default)]
        cell: Option<String>,
    },
    TrajectoryStop {},
    TrajectoryUnload {},
    TrajectorySetLoop {
        #[serde(rename = "loop")]
        looping: bool,
    },
    TrajectorySetSpeed { speed: f64 },
    /// The Robo Fitter's weld spot labels (the sidebar's "Weld labels").
    SetWeldLabels { on: bool },
}

impl SidebarCommand {
    /// The sidebar's own action for it.
    pub(crate) fn action(self) -> crate::sidebar::Action {
        use crate::sidebar::Action;
        match self {
            Self::OpenProduct {} => Action::OpenProduct,
            Self::OpenDresser {} => Action::OpenDresser,
            Self::OpenFitter {} => Action::OpenFitter,
            Self::OpenTrainingJob {} => Action::OpenTrainingJob,
            Self::WatchCell {} => Action::WatchCell,
            Self::EditScene {} => Action::EditScene,
            Self::RefreshCells {} => Action::RefreshCells,
            Self::RefreshPlans {} => Action::RefreshPlans,
            Self::OpenCell { cell } => Action::OpenCell(cell),
            Self::SetActiveCell { cell } => Action::SetActiveCell(cell),
            Self::UseScene { scene, open } => Action::UseScene { scene, open },
            Self::LoadPlan { path, cell } => Action::LoadPlan { path, cell },
            Self::TrajectoryStop {} => Action::Trajectory("trajectory_stop", None),
            Self::TrajectoryUnload {} => Action::Trajectory("trajectory_unload", None),
            Self::TrajectorySetLoop { looping } => Action::Trajectory("trajectory_set_loop", Some(json!({ "loop": looping }))),
            Self::TrajectorySetSpeed { speed } => Action::Trajectory("trajectory_set_speed", Some(json!({ "speed": speed }))),
            Self::SetWeldLabels { on } => Action::SetWeldLabels(on),
        }
    }
}

/// The mouse button of a `drag`: Rerun's 3D view orbits with the primary one and pans with the
/// secondary one.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DragButton {
    #[default]
    Primary,
    Secondary,
}

fn default_drag_steps() -> u32 {
    8
}

fn default_double_click_at() -> [f32; 2] {
    [0.03, 0.5]
}

fn default_timeout_s() -> f64 {
    30.0
}

fn default_settle_s() -> f64 {
    0.5
}

impl Command {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Open { .. } => "open",
            Self::WaitIdle { .. } => "wait_idle",
            Self::Screenshot { .. } => "screenshot",
            Self::State {} => "state",
            Self::SetWindow { .. } => "set_window",
            Self::ProductSelect { .. } => "product_select",
            Self::Scene { .. } => "scene",
            Self::Camera {} => "camera",
            Self::DoubleClick { .. } => "double_click",
            Self::Drag { .. } => "drag",
            Self::RealPointer { .. } => "real_pointer",
            Self::Action { .. } => "action",
            Self::ExportCode { .. } => "export_code",
            Self::Quit {} => "quit",
        }
    }
}

/// One command, and the `id` to answer with.
#[derive(Clone, Debug, PartialEq)]
pub struct Request {
    pub id: Option<Value>,
    pub command: Command,
}

/// A command that can't be run: the reply's error.
#[derive(Clone, Debug, PartialEq)]
pub struct Rejected {
    pub id: Option<Value>,
    pub error: String,
}

/// Parse one line of stdin.
pub fn parse_line(line: &str) -> Result<Request, Rejected> {
    let mut value: Value = serde_json::from_str(line).map_err(|err| Rejected { id: None, error: format!("not JSON: {err}") })?;
    let id = value.as_object_mut().and_then(|object| object.remove("id"));
    let reject = |error: String| Rejected { id: id.clone(), error };
    let command: Command = serde_json::from_value(value).map_err(|err| reject(format!("bad command: {err}")))?;
    validate(&command).map_err(reject)?;
    Ok(Request { id, command })
}

fn validate(command: &Command) -> Result<(), String> {
    match command {
        Command::WaitIdle { timeout_s, settle_s } => {
            if !(timeout_s.is_finite() && *timeout_s > 0.0 && *timeout_s <= 3600.0) {
                return Err(format!("timeout_s must be in (0, 3600], not {timeout_s}"));
            }
            if !(settle_s.is_finite() && *settle_s >= 0.0 && settle_s < timeout_s) {
                return Err(format!("settle_s must be in [0, timeout_s), not {settle_s}"));
            }
        }
        Command::Screenshot { path, .. } => {
            // On the web the path only names the PNG (it comes back inline), and wasm32's paths
            // have no root anyway.
            if cfg!(not(target_arch = "wasm32")) && !path.is_absolute() {
                return Err(format!("path must be absolute: {}", path.display()));
            }
            if path.extension().and_then(|ext| ext.to_str()).is_none_or(|ext| !ext.eq_ignore_ascii_case("png")) {
                return Err(format!("path must end in .png: {}", path.display()));
            }
        }
        Command::SetWindow { width, height } => {
            for (what, value) in [("width", width), ("height", height)] {
                if !(value.is_finite() && (320.0..=8192.0).contains(value)) {
                    return Err(format!("{what} must be in [320, 8192], not {value}"));
                }
            }
        }
        Command::ProductSelect { name, .. } => {
            if name.trim().is_empty() {
                return Err("name must not be empty".to_owned());
            }
        }
        Command::Scene { nodes, .. } => {
            if nodes.len() > 64 || nodes.iter().any(|node| node.trim().is_empty()) {
                return Err("nodes: at most 64, none empty".to_owned());
            }
        }
        Command::Drag { dx, dy, steps, .. } => {
            if !(dx.is_finite() && dy.is_finite() && dx.abs() <= 2000.0 && dy.abs() <= 2000.0 && (dx.abs() + dy.abs()) >= 1.0) {
                return Err(format!("dx, dy must be finite, within 2000 points, and move: not {dx}, {dy}"));
            }
            if !(1..=120).contains(steps) {
                return Err(format!("steps must be in [1, 120], not {steps}"));
            }
        }
        Command::DoubleClick { at } => {
            if !at.iter().all(|v| (0.0..=1.0).contains(v)) {
                return Err(format!("at must be shares of the view in [0, 1], not {at:?}"));
            }
        }
        Command::Action { action } => match action {
            SidebarCommand::OpenCell { cell } | SidebarCommand::SetActiveCell { cell } if cell.trim().is_empty() => {
                return Err("cell must not be empty".to_owned());
            }
            SidebarCommand::UseScene { scene, .. } if scene.trim().is_empty() => return Err("scene must not be empty".to_owned()),
            SidebarCommand::LoadPlan { path, .. } if path.trim().is_empty() => return Err("path must not be empty".to_owned()),
            SidebarCommand::TrajectorySetSpeed { speed } if !(speed.is_finite() && *speed > 0.0 && *speed <= 10.0) => {
                return Err(format!("speed must be in (0, 10], not {speed}"));
            }
            _ => {}
        },
        Command::ExportCode { plan: Some(plan) } if plan.trim().is_empty() => return Err("plan must not be empty".to_owned()),
        Command::Open { .. }
        | Command::State {}
        | Command::Camera {}
        | Command::RealPointer { .. }
        | Command::ExportCode { .. }
        | Command::Quit {} => {}
    }
    Ok(())
}

/// The reply line for `cmd` (without the newline): `ok` with `result`'s fields, or the error and
/// the fields of `details`.
pub fn reply_line(id: Option<&Value>, cmd: &str, result: Result<Value, (String, Value)>) -> String {
    let mut reply = serde_json::Map::new();
    if let Some(id) = id {
        reply.insert("id".to_owned(), id.clone());
    }
    reply.insert("cmd".to_owned(), cmd.into());
    let (ok, fields) = match result {
        Ok(fields) => (true, fields),
        Err((error, details)) => {
            reply.insert("error".to_owned(), error.into());
            (false, details)
        }
    };
    reply.insert("ok".to_owned(), ok.into());
    match fields {
        Value::Object(fields) => {
            for (key, value) in fields {
                reply.entry(key).or_insert(value);
            }
        }
        Value::Null => {}
        other => {
            reply.insert("result".to_owned(), other);
        }
    }
    format!("{REPLY_PREFIX}{}", Value::Object(reply))
}

/// Write one reply line to stdout (whole, flushed). A closed stdout is ignored.
#[cfg(not(target_arch = "wasm32"))]
pub fn emit(line: &str) {
    let mut out = std::io::stdout().lock();
    writeln!(out, "{line}").ok();
    out.flush().ok();
}

/// Hand one reply line to the page's [`JS_REPLY`] (logged if it has none).
#[cfg(target_arch = "wasm32")]
pub fn emit(line: &str) {
    use wasm_bindgen::JsCast as _;
    let callback = js_sys::Reflect::get(&js_sys::global(), &JS_REPLY.into())
        .ok()
        .and_then(|value| value.dyn_into::<js_sys::Function>().ok());
    match callback {
        Some(callback) => {
            callback.call1(&wasm_bindgen::JsValue::NULL, &line.into()).ok();
        }
        None => crate::rr::re_log::warn!("Automation: no {JS_REPLY} in the page for {line}"),
    }
}

pub fn reply(id: Option<&Value>, cmd: &str, result: Result<Value, (String, Value)>) {
    emit(&reply_line(id, cmd, result));
}

/// `events` without the pointer's: the mouse over the window changes nothing, and egui is told
/// once (`first`) that the pointer is gone.
pub fn without_pointer(events: &mut Vec<egui::Event>, first: bool) {
    events.retain(|event| {
        !matches!(
            event,
            egui::Event::PointerMoved(_)
                | egui::Event::MouseMoved(_)
                | egui::Event::PointerButton { .. }
                | egui::Event::PointerGone
                | egui::Event::MouseWheel { .. }
                | egui::Event::Zoom(_)
                | egui::Event::Rotate(_)
                | egui::Event::Touch { .. }
        )
    });
    if first {
        events.push(egui::Event::PointerGone);
    }
}

/// The sidebar action that opens `tool`.
pub(crate) fn open_action(tool: Tool) -> crate::sidebar::Action {
    use crate::sidebar::Action;
    match tool {
        Tool::Dresser => Action::OpenDresser,
        Tool::Fitter => Action::OpenFitter,
        Tool::Product => Action::OpenProduct,
        Tool::Training => Action::OpenTrainingJob,
        Tool::Live => Action::WatchCell,
    }
}

/// A `wait_idle` in progress.
#[derive(Debug)]
pub struct IdleWait {
    pub id: Option<Value>,
    started: Instant,
    deadline: Instant,
    settle: Duration,
    /// Idle without a break since then.
    idle_since: Option<Instant>,
    /// Why it was busy the last time it was.
    last_busy: Option<String>,
    frames: u64,
}

impl IdleWait {
    pub fn new(id: Option<Value>, timeout_s: f64, settle_s: f64, now: Instant) -> Self {
        Self {
            id,
            started: now,
            deadline: now + Duration::from_secs_f64(timeout_s),
            settle: Duration::from_secs_f64(settle_s),
            idle_since: None,
            last_busy: None,
            frames: 0,
        }
    }

    /// One frame's verdict (`busy`: why the scene isn't fully shown, `None` if it is). The reply
    /// once the wait is over.
    pub fn step(&mut self, busy: Option<String>, now: Instant) -> Option<Result<Value, (String, Value)>> {
        self.frames += 1;
        let waited = now.saturating_duration_since(self.started).as_secs_f64();
        match busy {
            None => {
                let since = *self.idle_since.get_or_insert(now);
                if now.saturating_duration_since(since) >= self.settle {
                    return Some(Ok(json!({ "waited_s": round3(waited), "frames": self.frames })));
                }
            }
            Some(reason) => {
                self.idle_since = None;
                self.last_busy = Some(reason);
            }
        }
        if now >= self.deadline {
            let reason = match (&self.idle_since, &self.last_busy) {
                (Some(_), _) => "idle, but not yet for settle_s".to_owned(),
                (None, Some(reason)) => reason.clone(),
                (None, None) => "unknown".to_owned(),
            };
            return Some(Err((format!("timeout after {:.1} s", waited), json!({ "reason": reason, "frames": self.frames }))));
        }
        None
    }
}

fn round3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

/// A screenshot asked for, waiting for its `egui::Event::Screenshot`.
#[derive(Debug)]
struct PendingShot {
    id: Option<Value>,
    token: u64,
    path: PathBuf,
    region: Region,
    asked: Instant,
}

/// The 3D view's rect among the last frame's interactive widget rects (in points): Rerun's 3D view
/// takes all of its tile (`allocate_at_least(available_size, click_and_drag)`), so it is the
/// largest interactive widget that is not (nearly) the whole window and at least 100 points each way.
pub fn view_3d_rect(rects: &[egui::Rect], screen: egui::Rect) -> Option<egui::Rect> {
    let screen_area = screen.area();
    rects
        .iter()
        .map(|rect| rect.intersect(screen))
        .filter(|rect| rect.is_positive() && rect.width() >= 100.0 && rect.height() >= 100.0 && rect.area() < 0.9 * screen_area)
        .max_by(|a, b| a.area().total_cmp(&b.area()))
}

/// The pixel rect `[x, y, w, h]` of `region` in an image of `size` (physical pixels), given the
/// 3D view's rect in points (if found) and the pixels per point; and what it was cropped to.
pub fn crop_rect(region: Region, size: [usize; 2], pixels_per_point: f32, view_3d: Option<egui::Rect>) -> ([u32; 4], &'static str) {
    let full = ([0, 0, size[0] as u32, size[1] as u32], "window");
    match (region, view_3d) {
        (Region::Window, _) => full,
        (Region::View3d, None) => full,
        (Region::View3d, Some(rect)) => {
            let to_px = |v: f32, max: usize| ((v * pixels_per_point).round().max(0.0) as usize).min(max) as u32;
            let (x0, y0) = (to_px(rect.min.x, size[0]), to_px(rect.min.y, size[1]));
            let (x1, y1) = (to_px(rect.max.x, size[0]), to_px(rect.max.y, size[1]));
            if x1 <= x0 || y1 <= y0 {
                return full;
            }
            ([x0, y0, x1 - x0, y1 - y0], "3d_view")
        }
    }
}

/// A `drag` or `double_click` in progress: one frame's egui input after the other
/// ([`Automation::filter_input`]).
#[derive(Debug)]
pub struct PendingPointer {
    pub id: Option<Value>,
    pub cmd: &'static str,
    frames: std::collections::VecDeque<Vec<egui::Event>>,
    done: u32,
    reply: Value,
}

impl PendingPointer {
    /// The pointer onto `from`, `button` pressed, moved by `delta` in `steps`, released, gone.
    pub fn drag(id: Option<Value>, from: egui::Pos2, delta: egui::Vec2, button: DragButton, steps: u32) -> Self {
        let button = match button {
            DragButton::Primary => egui::PointerButton::Primary,
            DragButton::Secondary => egui::PointerButton::Secondary,
        };
        let steps = steps.max(1);
        let at = |i: u32| from + delta * (i as f32 / steps as f32);
        let press = |pos, pressed| egui::Event::PointerButton { pos, button, pressed, modifiers: egui::Modifiers::NONE };
        let mut frames = vec![vec![egui::Event::PointerMoved(from)], vec![press(from, true)]];
        frames.extend((1..steps).map(|i| vec![egui::Event::PointerMoved(at(i))]));
        frames.push(vec![egui::Event::PointerMoved(at(steps)), press(at(steps), false)]);
        frames.push(vec![egui::Event::PointerGone]);
        let to = at(steps);
        Self { id, cmd: "drag", frames: frames.into(), done: 0, reply: json!({ "from": [from.x, from.y], "to": [to.x, to.y] }) }
    }

    /// The pointer onto `at`, two primary clicks, gone.
    pub fn double_click(id: Option<Value>, at: egui::Pos2) -> Self {
        let press = |pressed| egui::Event::PointerButton { pos: at, button: egui::PointerButton::Primary, pressed, modifiers: egui::Modifiers::NONE };
        let frames = vec![
            vec![egui::Event::PointerMoved(at)],
            vec![press(true)],
            vec![press(false)],
            vec![press(true)],
            vec![press(false)],
            vec![egui::Event::PointerGone],
        ];
        Self { id, cmd: "double_click", frames: frames.into(), done: 0, reply: json!({ "at": [at.x, at.y] }) }
    }

    /// This frame's events; `None` once done.
    pub fn step(&mut self) -> Option<Vec<egui::Event>> {
        let events = self.frames.pop_front()?;
        self.done += 1;
        Some(events)
    }

    pub fn reply(&self) -> Value {
        let mut reply = self.reply.clone();
        reply["frames"] = self.done.into();
        reply
    }
}

/// The `camera` facts: the 3D view's eye (`view`, Rerun's active blueprint's), the eye fitted to
/// the scene and the bounds it was fitted to (`fitted`), and where those bounds are in the view
/// (`view_rect`, points): their centre and corners in normalized view coordinates (-1..1, y up)
/// through the view's eye with Rerun's field of view, and the share of corners in view.
pub fn camera_facts(
    view: Option<&crate::scene3d::rerun::ViewEye>,
    fitted: Option<(crate::scene3d::rerun::Eye, Option<crate::scene3d::rerun::Aabb>)>,
    view_rect: Option<egui::Rect>,
) -> Value {
    use crate::rr::re_sdk_types::external::glam::{Mat4, Vec3};
    let v3 = |v: Vec3| json!([v.x, v.y, v.z]);
    let orbit_centre = view.and_then(|eye| if eye.tracking.is_some() { None } else { eye.look_target });
    let bounds = fitted.and_then(|(_, bounds)| bounds);
    let mut facts = json!({
        "view_eye": view.map(|eye| json!({
            "position": eye.position.map(v3),
            "look_target": eye.look_target.map(v3),
            "eye_up": eye.eye_up.map(v3),
            "tracking": eye.tracking,
        })),
        "orbit_centre": orbit_centre.map(v3),
        "fitted": fitted.map(|(eye, _)| json!({ "position": v3(eye.position), "look_target": v3(eye.look_target) })),
        "bounds": bounds.map(|b| json!({ "min": v3(b.min), "max": v3(b.max), "center": v3(b.center()), "diagonal": b.diagonal() })),
        "orbit_centre_offset": orbit_centre.zip(bounds).map(|(c, b)| (c - b.center()).length()),
        "view_rect": view_rect.map(|r| json!([r.min.x, r.min.y, r.width(), r.height()])),
    });
    let eye = view.and_then(|eye| eye.position.zip(eye.look_target).map(|p| (p, eye.eye_up.unwrap_or(Vec3::Z))));
    if let (Some(((position, look_target), up)), Some(bounds), Some(rect)) = (eye.map(|(p, up)| (p, up)), bounds, view_rect) {
        let view_from_world = Mat4::look_at_rh(position, look_target, up);
        let projection = Mat4::perspective_infinite_rh(crate::scene3d::rerun::FOV_Y, rect.width() / rect.height().max(1.0), 0.01);
        let ndc = |p: Vec3| {
            let clip = projection * view_from_world * p.extend(1.0);
            (clip.w > 0.0).then(|| [clip.x / clip.w, clip.y / clip.w])
        };
        let corners = bounds.corners();
        let inside = corners.iter().filter(|c| ndc(**c).is_some_and(|[x, y]| x.abs() <= 1.0 && y.abs() <= 1.0)).count();
        facts["bounds_center_ndc"] = json!(ndc(bounds.center()));
        facts["corners_in_view"] = json!(inside as f64 / corners.len() as f64);
    }
    facts
}

/// Natively: read stdin on a thread of its own, each command into `tx`.
#[cfg(not(target_arch = "wasm32"))]
fn start_intake(tx: mpsc::Sender<Request>, ctx: egui::Context) -> Option<()> {
    let spawned = std::thread::Builder::new().name("automation-stdin".to_owned()).spawn(move || {
        let stdin = std::io::stdin();
        for line in stdin.lock().lines() {
            let Ok(line) = line else { break };
            if line.trim().is_empty() {
                continue;
            }
            match parse_line(&line) {
                Ok(request) => {
                    if tx.send(request).is_err() {
                        return;
                    }
                    ctx.request_repaint();
                }
                Err(rejected) => reply(rejected.id.as_ref(), "?", Err((rejected.error, Value::Null))),
            }
        }
        // The parent closed our stdin (or went away): nobody is driving any more.
        tx.send(Request { id: None, command: Command::Quit {} }).ok();
        ctx.request_repaint();
    });
    if let Err(err) = spawned {
        crate::rr::re_log::error!("Automation: failed to start the stdin reader: {err}");
        return None;
    }
    crate::rr::re_log::info!("Automation on ({ENV}=stdin): commands on stdin, replies on stdout prefixed {REPLY_PREFIX:?}");
    Some(())
}

#[cfg(not(target_arch = "wasm32"))]
fn ready_fields() -> Value {
    json!({ "pid": std::process::id() })
}

#[cfg(target_arch = "wasm32")]
thread_local! {
    /// Where [`submit`] puts the page's commands, and the context to wake.
    static WEB_INTAKE: std::cell::RefCell<Option<(mpsc::Sender<Request>, egui::Context)>> = const { std::cell::RefCell::new(None) };
}

/// On the web: the page's commands come through [`submit`].
#[cfg(target_arch = "wasm32")]
fn start_intake(tx: mpsc::Sender<Request>, ctx: egui::Context) -> Option<()> {
    WEB_INTAKE.with_borrow_mut(|intake| *intake = Some((tx, ctx)));
    crate::rr::re_log::info!("Automation on (?automation=1): commands from the page, replies to {JS_REPLY}");
    Some(())
}

#[cfg(target_arch = "wasm32")]
fn ready_fields() -> Value {
    json!({})
}

/// On the web: one command line from the page (answered through [`emit`]).
#[cfg(target_arch = "wasm32")]
pub fn submit(line: &str) {
    WEB_INTAKE.with_borrow(|intake| {
        let Some((tx, ctx)) = intake else {
            reply(None, "?", Err(("automation is off: no automation=1 in the page's query".to_owned(), Value::Null)));
            return;
        };
        match parse_line(line) {
            Ok(request) => {
                if tx.send(request).is_ok() {
                    ctx.request_repaint();
                }
            }
            Err(rejected) => reply(rejected.id.as_ref(), "?", Err((rejected.error, Value::Null))),
        }
    });
}

/// The automation's state on the UI thread.
pub struct Automation {
    rx: mpsc::Receiver<Request>,
    pub waits: Vec<IdleWait>,
    shots: Vec<PendingShot>,
    next_token: u64,
    /// The pointer has been taken away from egui once ([`without_pointer`]).
    pointer_gone: bool,
    /// `real_pointer`: the real mouse goes through to egui.
    real_pointer: bool,
    /// What the mesh entities of the recordings shown held, over time.
    pub history: MeshHistory,
    /// Recordings the app asked Rerun to close (replaced ones), oldest first.
    closed: Vec<String>,
    /// A `drag` or `double_click` being injected.
    pub pointer: Option<PendingPointer>,
}

impl Automation {
    /// With `FACTORY_OS_AUTOMATION=stdin` (natively) or `?automation=1` (web): start taking
    /// commands (`ctx` woken for each).
    pub fn from_env(ctx: &egui::Context) -> Option<Self> {
        if !enabled() {
            return None;
        }
        let (tx, rx) = mpsc::channel();
        start_intake(tx, ctx.clone())?;
        emit(&reply_line(None, "ready", Ok(ready_fields())));
        Some(Self {
            rx,
            waits: Vec::new(),
            shots: Vec::new(),
            next_token: 1,
            pointer_gone: false,
            real_pointer: false,
            history: MeshHistory::default(),
            closed: Vec::new(),
            pointer: None,
        })
    }

    /// The app asked Rerun to close `store` (kept for `scene`, the last [`MAX_CLOSED`]).
    pub fn note_closed(&mut self, store: &StoreId) {
        self.closed.push(store.to_string());
        if self.closed.len() > MAX_CLOSED {
            self.closed.remove(0);
        }
    }

    /// The recordings asked to be closed, oldest first.
    pub fn closed(&self) -> &[String] {
        &self.closed
    }

    /// Filter this frame's raw input (see [`without_pointer`]), and add a `drag`'s or
    /// `double_click`'s events.
    pub fn filter_input(&mut self, raw_input: &mut egui::RawInput) {
        if !self.real_pointer {
            without_pointer(&mut raw_input.events, !std::mem::replace(&mut self.pointer_gone, true));
        }
        let Some(pointer) = &mut self.pointer else { return };
        match pointer.step() {
            Some(events) => raw_input.events.extend(events),
            None => {
                let pointer = self.pointer.take().expect("a pointer gesture");
                reply(pointer.id.as_ref(), pointer.cmd, Ok(pointer.reply()));
            }
        }
    }

    /// Inject `pointer` from the next frame on (one that was still going fails).
    pub fn start_pointer(&mut self, pointer: PendingPointer) {
        if let Some(old) = self.pointer.replace(pointer) {
            reply(old.id.as_ref(), old.cmd, Err(("replaced by another pointer command".to_owned(), Value::Null)));
        }
    }

    /// Let the real mouse through (`on`) or not (egui told once that it is gone).
    pub fn set_real_pointer(&mut self, on: bool) {
        self.real_pointer = on;
        if !on {
            self.pointer_gone = false;
        }
    }

    /// The commands that arrived since the last call.
    pub fn take_requests(&mut self) -> Vec<Request> {
        self.rx.try_iter().collect()
    }

    /// Ask egui for a screenshot of the next frame; answered in [`Self::handle_screenshots`].
    pub fn request_screenshot(&mut self, ctx: &egui::Context, id: Option<Value>, path: PathBuf, region: Region) {
        let token = self.next_token;
        self.next_token += 1;
        self.shots.push(PendingShot { id, token, path, region, asked: Instant::now() });
        ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::new(token)));
        ctx.request_repaint();
    }

    /// Save the screenshots that came back this frame (encoding off the UI thread) and fail the
    /// ones that took too long.
    pub fn handle_screenshots(&mut self, ctx: &egui::Context) {
        if self.shots.is_empty() {
            return;
        }
        let (events, pixels_per_point, screen) = ctx.input(|input| {
            let events: Vec<_> = input
                .events
                .iter()
                .filter_map(|event| match event {
                    egui::Event::Screenshot { user_data, image, .. } => {
                        let token = user_data.data.as_ref().and_then(|data| data.downcast_ref::<u64>()).copied()?;
                        Some((token, image.clone()))
                    }
                    _ => None,
                })
                .collect();
            (events, input.pixels_per_point, input.content_rect())
        });
        let view_3d = if events.is_empty() { None } else { view_3d_rect(&ctx.interactive_rects_last_pass(), screen) };
        for (token, image) in events {
            let Some(index) = self.shots.iter().position(|shot| shot.token == token) else { continue };
            let shot = self.shots.remove(index);
            let (crop, cropped_to) = crop_rect(shot.region, image.size, pixels_per_point, view_3d);
            let finish = move || {
                let result = save_png(&image, crop, &shot.path).map(|(width, height, png)| {
                    json!({
                        "path": shot.path,
                        "png_base64": png,
                        "width": width,
                        "height": height,
                        "region": cropped_to,
                        "crop": crop,
                        "window_px": image.size,
                        "pixels_per_point": pixels_per_point,
                        "view_3d_points": view_3d.map(|rect| [rect.min.x, rect.min.y, rect.width(), rect.height()]),
                    })
                });
                reply(shot.id.as_ref(), "screenshot", result.map_err(|err| (err, Value::Null)));
            };
            // Encoding off the UI thread natively; the web has no threads (and the page writes it).
            #[cfg(not(target_arch = "wasm32"))]
            std::thread::spawn(finish);
            #[cfg(target_arch = "wasm32")]
            finish();
        }
        let now = Instant::now();
        self.shots.retain(|shot| {
            let late = now.saturating_duration_since(shot.asked) > SCREENSHOT_TIMEOUT;
            if late {
                let error = format!("no screenshot within {} s (is the window minimized or hidden?)", SCREENSHOT_TIMEOUT.as_secs());
                reply(shot.id.as_ref(), "screenshot", Err((error, Value::Null)));
            }
            !late
        });
        if !self.shots.is_empty() {
            ctx.request_repaint_after(POLL);
        }
    }

    /// Step the waits with this frame's verdict; replies to those that are over.
    pub fn step_waits(&mut self, ctx: &egui::Context, busy: Option<String>) {
        if self.waits.is_empty() {
            return;
        }
        let now = Instant::now();
        self.waits.retain_mut(|wait| match wait.step(busy.clone(), now) {
            Some(result) => {
                reply(wait.id.as_ref(), "wait_idle", result);
                false
            }
            None => true,
        });
        if !self.waits.is_empty() {
            ctx.request_repaint_after(POLL);
        }
    }
}

// --- Introspection (`scene`): read-only facts of the recording Rerun shows ---------------------

/// What a mesh entity (URDF visual or collision geometry) holds.
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize)]
pub struct MeshContent {
    /// `asset3d` (a file: GLB, OBJ, STL, ...), `mesh3d` (vertices, e.g. a converted OBJ), or `none`
    /// (logged, but its data has not arrived).
    pub kind: &'static str,
    /// The blob's SHA-256 (lowercase hex): the asset it came from, content-addressed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sha256: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub media_type: Option<String>,
    /// Bytes of the blob, or vertices of a `Mesh3D`.
    pub size: u64,
}

/// Whether `entity` is URDF geometry (`/<node>/<robot>/visual_geometries/…` or
/// `…/collision_geometries/…`), and which.
pub fn geometry_kind(entity: &str) -> Option<&'static str> {
    if entity.contains("/visual_geometries/") {
        Some("visual")
    } else if entity.contains("/collision_geometries/") {
        Some("collision")
    } else {
        None
    }
}

fn latest() -> LatestAtQuery {
    LatestAtQuery::latest(TimelineName::log_tick())
}

/// The row that last wrote `entity`'s mesh data (blob or vertices), and its kind.
fn mesh_row(db: &EntityDb, entity: &EntityPath) -> Option<(RowId, &'static str)> {
    let blob = Asset3D::descriptor_blob().component;
    let vertices = Mesh3D::descriptor_vertex_positions().component;
    let results = db.latest_at(&latest(), entity, [blob, vertices]);
    let row = |component| results.get(component).filter(|c| c.num_instances(component) > 0).and(results.component_row_id(component));
    match (row(blob), row(vertices)) {
        (Some(a), Some(b)) => Some(if a > b { (a, "asset3d") } else { (b, "mesh3d") }),
        (Some(a), None) => Some((a, "asset3d")),
        (None, Some(b)) => Some((b, "mesh3d")),
        (None, None) => None,
    }
}

/// What `entity` holds now.
fn mesh_content(db: &EntityDb, entity: &EntityPath) -> MeshContent {
    use sha2::Digest as _;
    let Some((_, kind)) = mesh_row(db, entity) else {
        return MeshContent { kind: "none", sha256: None, media_type: None, size: 0 };
    };
    if kind == "mesh3d" {
        let component = Mesh3D::descriptor_vertex_positions().component;
        let results = db.latest_at(&latest(), entity, [component]);
        let size = results.get(component).map_or(0, |c| c.num_instances(component)) as u64;
        return MeshContent { kind, sha256: None, media_type: None, size };
    }
    let blob = Asset3D::descriptor_blob().component;
    let media = Asset3D::descriptor_media_type().component;
    let results = db.latest_at(&latest(), entity, [blob, media]);
    let bytes = results.component_mono::<Blob>(blob);
    let bytes: &[u8] = bytes.as_ref().map_or(&[], |b| &b.0);
    let sha256 = (!bytes.is_empty()).then(|| hex(&sha2::Sha256::digest(bytes)));
    let media_type = results.component_mono::<MediaType>(media).map(|m| m.as_str().to_owned());
    MeshContent { kind, sha256, media_type, size: bytes.len() as u64 }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

/// The transform frame `entity` draws in.
fn frame_of(db: &EntityDb, entity: &EntityPath) -> Option<String> {
    let component = CoordinateFrame::descriptor_frame().component;
    let results = db.latest_at(&latest(), entity, [component]);
    results.component_mono::<TransformFrameId>(component).map(|f| f.as_str().to_owned())
}

/// What every mesh entity held, over time, per recording: an event whenever one's data changed
/// (e.g. the decimated LOD first, then the full GLB). Updated each frame from the recording
/// Rerun shows ([`MeshHistory::observe`]), only when its store changed.
#[derive(Default)]
pub struct MeshHistory {
    stores: std::collections::HashMap<StoreId, StoreSeen>,
    events: std::collections::VecDeque<Value>,
    seq: u64,
    started: Option<Instant>,
}

#[derive(Default)]
struct StoreSeen {
    generation: Option<ChunkStoreGeneration>,
    rows: std::collections::HashMap<EntityPath, RowId>,
}

impl MeshHistory {
    /// Look at the recording shown (if any): an event for each mesh entity whose data changed.
    pub fn observe(&mut self, db: Option<&EntityDb>) {
        let Some(db) = db else { return };
        let started = *self.started.get_or_insert_with(Instant::now);
        let seen = self.stores.entry(db.store_id().clone()).or_default();
        let generation = db.generation();
        if seen.generation == Some(generation) {
            return;
        }
        seen.generation = Some(generation);
        for entity in db.sorted_entity_paths() {
            let path = entity.to_string();
            let Some(geometry) = geometry_kind(&path) else { continue };
            let Some((row, _)) = mesh_row(db, entity) else { continue };
            if seen.rows.get(entity) == Some(&row) {
                continue;
            }
            seen.rows.insert(entity.clone(), row);
            let content = mesh_content(db, entity);
            self.seq += 1;
            self.events.push_back(json!({
                "seq": self.seq,
                "t_s": round3(started.elapsed().as_secs_f64()),
                "recording": db.store_id().to_string(),
                "entity": path,
                "geometry": geometry,
                "kind": content.kind,
                "sha256": content.sha256,
                "size": content.size,
            }));
            if self.events.len() > MAX_HISTORY {
                self.events.pop_front();
            }
        }
    }

    /// The events after `since`, and the last `seq`.
    pub fn since(&self, since: u64) -> (Vec<Value>, u64) {
        let events = self.events.iter().filter(|e| e["seq"].as_u64().is_some_and(|s| s > since)).cloned().collect();
        (events, self.seq)
    }
}

/// The `scene` reply's facts of recording `db` (the one Rerun shows): its size and shape, every
/// mesh entity's content and, for each of `nodes`, its meshes (loaded, pending, by content) and
/// its links' world positions (by the frames its visual geometry draws in).
pub fn scene_facts(db: &EntityDb, nodes: &[String]) -> Value {
    let store = db.storage_engine();
    let chunks = store.store().iter_physical_chunks();
    let mut temporal_chunks = 0usize;
    let mut temporal_entities = std::collections::BTreeSet::new();
    for chunk in chunks {
        if !chunk.is_static() {
            temporal_chunks += 1;
            if temporal_entities.len() < MAX_TEMPORAL_NAMED {
                temporal_entities.insert(chunk.entity_path().to_string());
            }
        }
    }
    drop(store);

    let mut meshes = serde_json::Map::new();
    for entity in db.sorted_entity_paths() {
        let path = entity.to_string();
        let Some(geometry) = geometry_kind(&path) else { continue };
        // Only the leaves that draw (their frame is set when they are logged).
        if frame_of(db, entity).is_none() && mesh_row(db, entity).is_none() {
            continue;
        }
        let mut content = serde_json::to_value(mesh_content(db, entity)).unwrap_or_default();
        content["geometry"] = geometry.into();
        meshes.insert(path, content);
    }

    let mut node_facts = serde_json::Map::new();
    if !nodes.is_empty() {
        let mut cache = TransformResolutionCache::new(db);
        cache.ensure_timeline_is_initialized(db.storage_engine().store(), TimelineName::log_tick());
        let forest = TransformForest::new(db, &cache, &latest());
        let world = |frame: &str| -> Option<[f64; 3]> {
            let (_, result) = forest
                .transform_from_to(TransformFrameIdHash::entity_path_hierarchy_root(), std::iter::once(TransformFrameIdHash::from_str(frame)), &|_| 1.0)
                .next()?;
            let t = result.ok()?.target_from_source.translation;
            Some([round6(t.x), round6(t.y), round6(t.z)])
        };
        for node in nodes {
            let prefix = format!("{}/", crate::rr::re_chunk::EntityPath::parse_forgiving(node));
            // A deeper node (`robots/robot_2/pedestal`) is its own.
            let deeper: Vec<String> = nodes.iter().filter(|n| *n != node && n.starts_with(&format!("{node}/"))).map(|n| format!("/{n}/")).collect();
            let mut visual = (0usize, 0usize);
            let mut collision = (0usize, 0usize);
            let mut contents: std::collections::BTreeMap<String, usize> = Default::default();
            let mut kinds: std::collections::BTreeMap<&'static str, usize> = Default::default();
            let mut links = serde_json::Map::new();
            for (path, content) in &meshes {
                if !path.starts_with(&prefix) || deeper.iter().any(|d| path.starts_with(d.as_str())) {
                    continue;
                }
                let loaded = content["kind"] != "none";
                let counter = if content["geometry"] == "visual" { &mut visual } else { &mut collision };
                counter.0 += 1;
                counter.1 += usize::from(loaded);
                if content["geometry"] == "visual" {
                    *kinds.entry(match content["kind"].as_str() {
                        Some("asset3d") => "asset3d",
                        Some("mesh3d") => "mesh3d",
                        _ => "none",
                    }).or_default() += 1;
                    if let Some(sha) = content["sha256"].as_str() {
                        *contents.entry(sha.to_owned()).or_default() += 1;
                    }
                    if let Some(frame) = frame_of(db, &EntityPath::parse_forgiving(path))
                        && let Some((_, link)) = frame.split_once('#')
                        && !links.contains_key(link)
                    {
                        links.insert(link.to_owned(), world(&frame).map_or(Value::Null, |p| json!(p)));
                    }
                }
            }
            node_facts.insert(
                node.clone(),
                json!({
                    "visual": { "entities": visual.0, "loaded": visual.1, "pending": visual.0 - visual.1 },
                    "collision": { "entities": collision.0, "loaded": collision.1, "pending": collision.0 - collision.1 },
                    "kinds": kinds,
                    "contents": contents,
                    "links": links,
                }),
            );
        }
    }

    json!({
        "recording": db.store_id().to_string(),
        "application": db.application_id().to_string(),
        "entities": db.sorted_entity_paths().count(),
        "chunks": db.num_physical_chunks(),
        "bytes": db.byte_size_of_physical_chunks(),
        "temporal_chunks": temporal_chunks,
        "temporal_entities": temporal_entities,
        "timelines": db.timelines().keys().map(|t| t.to_string()).collect::<Vec<_>>(),
        "meshes": meshes,
        "nodes": node_facts,
    })
}

fn round6(x: f64) -> f64 {
    (x * 1e6).round() / 1e6
}

/// `image` cropped to `[x, y, w, h]`.
fn crop_image(image: &egui::ColorImage, [x, y, w, h]: [u32; 4]) -> Result<image::RgbaImage, String> {
    let [width, height] = image.size;
    let bytes: Vec<u8> = image.pixels.iter().flat_map(|pixel| pixel.to_srgba_unmultiplied()).collect();
    let full = image::RgbaImage::from_raw(width as u32, height as u32, bytes).ok_or("the screenshot's pixels don't match its size")?;
    Ok(image::imageops::crop_imm(&full, x, y, w, h).to_image())
}

/// Crop `image` to `crop` and write it as a PNG at `path`; returns its size (and no inline PNG).
#[cfg(not(target_arch = "wasm32"))]
fn save_png(image: &egui::ColorImage, crop: [u32; 4], path: &std::path::Path) -> Result<(u32, u32, Option<String>), String> {
    let cropped = crop_image(image, crop)?;
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|err| format!("create {}: {err}", dir.display()))?;
    }
    cropped
        .save_with_format(path, image::ImageFormat::Png)
        .map_err(|err| format!("write {}: {err}", path.display()))?;
    Ok((cropped.width(), cropped.height(), None))
}

/// On the web: crop `image` to `crop` and encode it as a PNG, returned base64 (the page writes it).
#[cfg(target_arch = "wasm32")]
fn save_png(image: &egui::ColorImage, crop: [u32; 4], _path: &std::path::Path) -> Result<(u32, u32, Option<String>), String> {
    use base64::Engine as _;
    let cropped = crop_image(image, crop)?;
    let mut png = std::io::Cursor::new(Vec::new());
    cropped.write_to(&mut png, image::ImageFormat::Png).map_err(|err| format!("encode PNG: {err}"))?;
    Ok((cropped.width(), cropped.height(), Some(base64::engine::general_purpose::STANDARD.encode(png.into_inner()))))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(line: &str) -> Command {
        parse_line(line).unwrap_or_else(|err| panic!("{line}: {}", err.error)).command
    }

    fn error(line: &str) -> String {
        parse_line(line).expect_err(line).error
    }

    #[test]
    fn parses_the_browser_commands() {
        assert_eq!(parse(r#"{"cmd":"real_pointer","on":true}"#), Command::RealPointer { on: true });
        assert_eq!(
            parse(r#"{"cmd":"action","action":{"name":"use_scene","scene":"two.yaml","open":true}}"#),
            Command::Action { action: SidebarCommand::UseScene { scene: "two.yaml".into(), open: true } }
        );
        assert_eq!(
            parse(r#"{"cmd":"action","action":{"name":"load_plan","path":"plans/a.json"}}"#),
            Command::Action { action: SidebarCommand::LoadPlan { path: "plans/a.json".into(), cell: None } }
        );
        assert_eq!(
            parse(r#"{"cmd":"action","action":{"name":"trajectory_set_loop","loop":true}}"#),
            Command::Action { action: SidebarCommand::TrajectorySetLoop { looping: true } }
        );
        assert_eq!(parse(r#"{"cmd":"action","action":{"name":"refresh_cells"}}"#), Command::Action { action: SidebarCommand::RefreshCells {} });
        assert_eq!(
            parse(r#"{"cmd":"action","action":{"name":"set_weld_labels","on":false}}"#),
            Command::Action { action: SidebarCommand::SetWeldLabels { on: false } }
        );
        assert_eq!(parse(r#"{"cmd":"export_code"}"#), Command::ExportCode { plan: None });
        assert_eq!(
            parse(r#"{"cmd":"export_code","plan":"agentic_planner/plans/a.json"}"#),
            Command::ExportCode { plan: Some("agentic_planner/plans/a.json".into()) }
        );
        assert!(error(r#"{"cmd":"export_code","plan":""}"#).contains("plan must not be empty"));
        assert!(error(r#"{"cmd":"action","action":{"name":"open_cell","cell":" "}}"#).contains("cell must not be empty"));
        assert!(error(r#"{"cmd":"action","action":{"name":"trajectory_set_speed","speed":0}}"#).contains("speed"));
        assert!(error(r#"{"cmd":"action","action":{"name":"rm_rf"}}"#).contains("bad command"));
        assert!(error(r#"{"cmd":"action","action":{"name":"open_fitter","extra":1}}"#).contains("bad command"));
        assert_eq!(
            SidebarCommand::TrajectorySetSpeed { speed: 2.0 }.action(),
            crate::sidebar::Action::Trajectory("trajectory_set_speed", Some(json!({ "speed": 2.0 })))
        );
    }

    #[test]
    fn only_automation_1_in_the_query_turns_it_on() {
        assert!(query_enables("?automation=1"));
        assert!(query_enables("?a=b&automation=1"));
        assert!(!query_enables(""));
        assert!(!query_enables("?automation=0"));
        assert!(!query_enables("?automation=10"));
        assert!(!query_enables("?xautomation=1"));
    }

    #[test]
    fn parses_every_command() {
        assert_eq!(parse(r#"{"cmd":"open","tool":"dresser"}"#), Command::Open { tool: Tool::Dresser });
        assert_eq!(parse(r#"{"cmd":"open","tool":"live"}"#), Command::Open { tool: Tool::Live });
        assert_eq!(parse(r#"{"cmd":"wait_idle"}"#), Command::WaitIdle { timeout_s: 30.0, settle_s: 0.5 });
        assert_eq!(parse(r#"{"cmd":"wait_idle","timeout_s":5,"settle_s":0}"#), Command::WaitIdle { timeout_s: 5.0, settle_s: 0.0 });
        assert_eq!(
            parse(r#"{"cmd":"screenshot","path":"/tmp/a.png","region":"3d"}"#),
            Command::Screenshot { path: "/tmp/a.png".into(), region: Region::View3d }
        );
        assert_eq!(parse(r#"{"cmd":"screenshot","path":"/tmp/a.PNG"}"#), Command::Screenshot { path: "/tmp/a.PNG".into(), region: Region::Window });
        assert_eq!(parse(r#"{"cmd":"state"}"#), Command::State {});
        assert_eq!(parse(r#"{"cmd":"set_window","width":1600,"height":1000}"#), Command::SetWindow { width: 1600.0, height: 1000.0 });
        assert_eq!(
            parse(r#"{"cmd":"product_select","name":"weld_box","from":"catalogue"}"#),
            Command::ProductSelect { name: "weld_box".into(), from: ProductSource::Catalogue }
        );
        assert_eq!(parse(r#"{"cmd":"product_select","name":"p"}"#), Command::ProductSelect { name: "p".into(), from: ProductSource::Products });
        assert_eq!(parse(r#"{"cmd":"quit"}"#), Command::Quit {});
        assert_eq!(parse(r#"{"cmd":"scene"}"#), Command::Scene { nodes: vec![], since: 0 });
        assert_eq!(
            parse(r#"{"cmd":"scene","nodes":["robots/robot_2"],"since":7}"#),
            Command::Scene { nodes: vec!["robots/robot_2".into()], since: 7 }
        );
        assert_eq!(parse(r#"{"cmd":"camera"}"#), Command::Camera {});
        assert_eq!(parse(r#"{"cmd":"double_click"}"#), Command::DoubleClick { at: [0.03, 0.5] });
        assert_eq!(parse(r#"{"cmd":"drag","dx":80,"dy":-10}"#), Command::Drag { dx: 80.0, dy: -10.0, button: DragButton::Primary, steps: 8 });
        assert_eq!(
            parse(r#"{"cmd":"drag","dx":0,"dy":40,"button":"secondary","steps":3}"#),
            Command::Drag { dx: 0.0, dy: 40.0, button: DragButton::Secondary, steps: 3 }
        );
    }

    /// A drag: the pointer onto the start, pressed, moved in steps to the end, released, gone; a
    /// double click: two clicks in a row.
    #[test]
    fn pointer_gestures_take_a_frame_per_step() {
        let run = |mut pointer: PendingPointer| {
            let mut frames = Vec::new();
            while let Some(events) = pointer.step() {
                frames.push(events);
            }
            (frames, pointer.reply())
        };
        let press = |pos, pressed| egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed, modifiers: egui::Modifiers::NONE };
        let (start, end) = (egui::pos2(100.0, 50.0), egui::pos2(140.0, 30.0));
        let (frames, reply) = run(PendingPointer::drag(None, start, egui::vec2(40.0, -20.0), DragButton::Primary, 2));
        assert_eq!(
            frames,
            [
                vec![egui::Event::PointerMoved(start)],
                vec![press(start, true)],
                vec![egui::Event::PointerMoved(egui::pos2(120.0, 40.0))],
                vec![egui::Event::PointerMoved(end), press(end, false)],
                vec![egui::Event::PointerGone],
            ]
        );
        assert_eq!(reply["to"], json!([140.0, 30.0]));
        assert_eq!(reply["frames"], json!(5));

        let (frames, _) = run(PendingPointer::double_click(None, start));
        let presses: Vec<_> = frames.iter().flatten().filter(|e| matches!(e, egui::Event::PointerButton { pressed: true, .. })).collect();
        assert_eq!(presses.len(), 2);
    }

    /// `camera`: the fitted eye, as the view has it, orbits the bounds' centre, which is in the
    /// middle of the view, all corners in view; a tracked entity has no orbit centre of its own.
    #[test]
    fn camera_facts_place_the_bounds_in_the_view() {
        use crate::rr::re_sdk_types::external::glam::Vec3;
        use crate::scene3d::rerun::{Aabb, Eye, ViewEye};
        let bounds = Aabb { min: Vec3::new(-1.0, -2.0, 0.0), max: Vec3::new(3.0, 2.0, 1.5) };
        let eye = Eye { position: bounds.center() - Vec3::new(1.0, 3.0, -1.0) * 3.0, look_target: bounds.center() };
        let view = ViewEye { position: Some(eye.position), look_target: Some(eye.look_target), eye_up: Some(Vec3::Z), tracking: None };
        let rect = egui::Rect::from_min_size(egui::pos2(200.0, 40.0), egui::vec2(1000.0, 800.0));
        let facts = camera_facts(Some(&view), Some((eye, Some(bounds))), Some(rect));
        assert_eq!(facts["orbit_centre_offset"], json!(0.0));
        let ndc = facts["bounds_center_ndc"].as_array().unwrap();
        assert!(ndc.iter().all(|v| v.as_f64().unwrap().abs() < 1e-5), "{ndc:?}");
        assert_eq!(facts["corners_in_view"], json!(1.0));

        let tracking = ViewEye { tracking: Some("/__view_camera".into()), ..view };
        let facts = camera_facts(Some(&tracking), Some((eye, Some(bounds))), Some(rect));
        assert_eq!(facts["orbit_centre"], Value::Null);
        assert_eq!(facts["orbit_centre_offset"], Value::Null);
    }

    #[test]
    fn the_id_is_kept_also_for_rejected_commands() {
        let request = parse_line(r#"{"id":7,"cmd":"state"}"#).unwrap();
        assert_eq!(request.id, Some(json!(7)));
        let rejected = parse_line(r#"{"id":"x","cmd":"fly"}"#).unwrap_err();
        assert_eq!(rejected.id, Some(json!("x")));
        assert!(rejected.error.starts_with("bad command"), "{}", rejected.error);
    }

    #[test]
    fn rejects_bad_commands() {
        assert!(error("not json").starts_with("not JSON"));
        assert!(error(r#"{"cmd":"open","tool":"kitchen"}"#).starts_with("bad command"));
        assert!(error(r#"{"cmd":"open"}"#).starts_with("bad command"));
        assert!(error(r#"{"cmd":"state","extra":1}"#).starts_with("bad command"));
        assert!(error(r#"{"cmd":"screenshot","path":"rel.png"}"#).contains("absolute"));
        assert!(error(r#"{"cmd":"screenshot","path":"/tmp/a.jpg"}"#).contains(".png"));
        assert!(error(r#"{"cmd":"screenshot","path":"/tmp/a.png","region":"left"}"#).starts_with("bad command"));
        assert!(error(r#"{"cmd":"wait_idle","timeout_s":0}"#).contains("timeout_s"));
        assert!(error(r#"{"cmd":"wait_idle","timeout_s":1,"settle_s":2}"#).contains("settle_s"));
        assert!(error(r#"{"cmd":"set_window","width":10,"height":1000}"#).contains("width"));
        assert!(error(r#"{"cmd":"product_select","name":" "}"#).contains("name"));
        assert!(error(r#"{"cmd":"scene","nodes":[""]}"#).contains("nodes"));
        assert!(error(r#"{"cmd":"drag","dx":0,"dy":0}"#).contains("dx"));
        assert!(error(r#"{"cmd":"drag","dx":5,"dy":0,"steps":0}"#).contains("steps"));
        assert!(error(r#"{"cmd":"drag","dx":5}"#).starts_with("bad command"));
        assert!(error(r#"{"cmd":"double_click","at":[2,0]}"#).contains("at must"));
    }

    #[test]
    fn scene_facts_hash_mesh_blobs_and_see_no_temporal_data() {
        use crate::rr::re_chunk::{Chunk, TimePoint};
        use crate::rr::re_log_types::StoreKind;
        use crate::rr::re_sdk_types::components::MediaType;
        use sha2::Digest as _;

        let store = StoreId::random(StoreKind::Recording, "test");
        let mut db = EntityDb::new(store.clone());
        let glb = b"glTF fake mesh".to_vec();
        let mesh = EntityPath::parse_forgiving("robots/r1/kr/visual_geometries/link_1/visual_0");
        let pending = EntityPath::parse_forgiving("robots/r1/kr/visual_geometries/link_2/visual_0");
        let add = |db: &mut EntityDb, chunk: Chunk| {
            db.add_chunk(&std::sync::Arc::new(chunk)).unwrap();
        };
        add(
            &mut db,
            Chunk::builder(mesh.clone())
                .with_archetype(RowId::new(), TimePoint::default(), &Asset3D::from_file_contents(glb.clone(), Some(MediaType::glb())))
                .build()
                .unwrap(),
        );
        add(
            &mut db,
            Chunk::builder(pending.clone())
                .with_archetype(RowId::new(), TimePoint::default(), &CoordinateFrame::new("r1#link_2"))
                .build()
                .unwrap(),
        );
        let facts = scene_facts(&db, &["robots/r1".to_owned()]);
        assert_eq!(facts["temporal_chunks"], json!(0));
        let sha = hex(&sha2::Sha256::digest(&glb));
        let content = &facts["meshes"][mesh.to_string()];
        assert_eq!(content["kind"], json!("asset3d"));
        assert_eq!(content["sha256"], json!(sha));
        assert_eq!(content["media_type"], json!(MediaType::GLB));
        assert_eq!(facts["meshes"][pending.to_string()]["kind"], json!("none"));
        let node = &facts["nodes"]["robots/r1"];
        assert_eq!(node["visual"], json!({ "entities": 2, "loaded": 1, "pending": 1 }));
        assert_eq!(node["contents"][&sha], json!(1));

        let mut history = MeshHistory::default();
        history.observe(Some(&db));
        history.observe(Some(&db)); // unchanged: no new event
        let (events, seq) = history.since(0);
        assert_eq!((events.len(), seq), (1, 1));
        assert_eq!(events[0]["sha256"], json!(sha));
    }

    #[test]
    fn geometry_entities_are_told_apart() {
        assert_eq!(geometry_kind("/robots/r1/kr240/visual_geometries/link_1/visual_0"), Some("visual"));
        assert_eq!(geometry_kind("/robot/kr240/collision_geometries/mesh/link_1/collision_0"), Some("collision"));
        assert_eq!(geometry_kind("/robots/r1/pedestal"), None);
    }

    #[test]
    fn replies_are_one_prefixed_json_line() {
        let line = reply_line(Some(&json!(3)), "screenshot", Ok(json!({ "path": "/tmp/a.png", "width": 10 })));
        let body = line.strip_prefix(REPLY_PREFIX).expect("prefixed");
        assert!(!line.contains('\n'));
        let reply: Value = serde_json::from_str(body).unwrap();
        assert_eq!(reply, json!({ "id": 3, "cmd": "screenshot", "ok": true, "path": "/tmp/a.png", "width": 10 }));

        let line = reply_line(None, "wait_idle", Err(("timeout after 1.0 s".into(), json!({ "reason": "2 mesh(es) pending", "ok": true }))));
        let reply: Value = serde_json::from_str(line.strip_prefix(REPLY_PREFIX).unwrap()).unwrap();
        // The details can't override the verdict.
        assert_eq!(reply, json!({ "cmd": "wait_idle", "ok": false, "error": "timeout after 1.0 s", "reason": "2 mesh(es) pending" }));
    }

    #[test]
    fn open_does_what_the_sidebar_buttons_do() {
        use crate::sidebar::Action;
        assert!(matches!(open_action(Tool::Dresser), Action::OpenDresser));
        assert!(matches!(open_action(Tool::Fitter), Action::OpenFitter));
        assert!(matches!(open_action(Tool::Product), Action::OpenProduct));
        assert!(matches!(open_action(Tool::Training), Action::OpenTrainingJob));
        assert!(matches!(open_action(Tool::Live), Action::WatchCell));
    }

    #[test]
    fn a_wait_needs_idle_frames_for_the_settle_time() {
        let t0 = Instant::now();
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        let mut wait = IdleWait::new(None, 10.0, 0.5, t0);
        assert!(wait.step(Some("1 mesh(es) pending".into()), at(0)).is_none());
        assert!(wait.step(None, at(100)).is_none());
        // Busy again: the settle time starts over.
        assert!(wait.step(Some("live queue not empty".into()), at(300)).is_none());
        assert!(wait.step(None, at(400)).is_none());
        assert!(wait.step(None, at(800)).is_none());
        let done = wait.step(None, at(900)).unwrap().unwrap();
        assert_eq!(done["waited_s"], json!(0.9));
    }

    #[test]
    fn a_wait_times_out_with_the_reason() {
        let t0 = Instant::now();
        let mut wait = IdleWait::new(Some(json!(1)), 1.0, 0.0, t0);
        assert!(wait.step(Some("2 model(s) loading".into()), t0).is_none());
        let (error, details) = wait.step(Some("1 mesh(es) pending".into()), t0 + Duration::from_secs(2)).unwrap().unwrap_err();
        assert!(error.starts_with("timeout"), "{error}");
        assert_eq!(details["reason"], json!("1 mesh(es) pending"));
        // Settled at once with no settle time.
        let mut wait = IdleWait::new(None, 1.0, 0.0, t0);
        assert!(wait.step(None, t0).unwrap().is_ok());
    }

    #[test]
    fn the_3d_view_is_the_largest_interactive_rect_short_of_the_window() {
        let screen = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(1600.0, 1000.0));
        let rect = |x: f32, y: f32, w: f32, h: f32| egui::Rect::from_min_size(egui::pos2(x, y), egui::vec2(w, h));
        let view = rect(300.0, 60.0, 900.0, 800.0);
        let rects = [screen, rect(0.0, 0.0, 1600.0, 40.0), rect(0.0, 40.0, 300.0, 900.0), view, rect(1250.0, 40.0, 50.0, 50.0)];
        assert_eq!(view_3d_rect(&rects, screen), Some(view));
        assert_eq!(view_3d_rect(&[screen], screen), None);
    }

    #[test]
    fn the_real_pointer_is_ignored() {
        let mut events = vec![
            egui::Event::PointerMoved(egui::pos2(10.0, 10.0)),
            egui::Event::Key { key: egui::Key::A, physical_key: None, pressed: true, repeat: false, modifiers: Default::default() },
            egui::Event::PointerButton { pos: egui::pos2(1.0, 1.0), button: egui::PointerButton::Primary, pressed: true, modifiers: Default::default() },
        ];
        without_pointer(&mut events, true);
        assert_eq!(events.len(), 2);
        assert!(matches!(events[0], egui::Event::Key { .. }));
        assert!(matches!(events[1], egui::Event::PointerGone));
        let mut events = vec![egui::Event::PointerGone];
        without_pointer(&mut events, false);
        assert!(events.is_empty());
    }

    #[test]
    fn crops_to_the_3d_view_in_pixels() {
        let view = egui::Rect::from_min_size(egui::pos2(300.0, 60.0), egui::vec2(900.0, 800.0));
        assert_eq!(crop_rect(Region::View3d, [3200, 2000], 2.0, Some(view)), ([600, 120, 1800, 1600], "3d_view"));
        assert_eq!(crop_rect(Region::Window, [3200, 2000], 2.0, Some(view)), ([0, 0, 3200, 2000], "window"));
        // Not found: the whole window, and the reply says so.
        assert_eq!(crop_rect(Region::View3d, [3200, 2000], 2.0, None), ([0, 0, 3200, 2000], "window"));
        // Clamped to the image.
        let past = egui::Rect::from_min_size(egui::pos2(1500.0, 900.0), egui::vec2(500.0, 500.0));
        assert_eq!(crop_rect(Region::View3d, [3200, 2000], 2.0, Some(past)), ([3000, 1800, 200, 200], "3d_view"));
    }
}
