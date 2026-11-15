//! Robo Dresser, replacing the Kiro extension's `urdf-robo-dresser` web app.
//!
//! The dressing logic is the `urdf_robo_dresser` library itself (catalogue scan, validation,
//! collision hulls, merge: the same steps as its `/api/v1/dress` handler). Previews are
//! renderer-neutral scenes on `factoryos/scene/dresser` ([`factoryos_protocol::scene`]): the URDF
//! as a model, the reach envelope as a mesh and the TCP as primitives. The viewer draws them, and
//! the catalogue and inspector.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use anyhow::Context as _;

use axum::extract::{Query, State};
use axum::http::StatusCode;
use axum::routing::{get, post};
use axum::{Json, Router};
use factoryos_protocol::dresser::{
    AssetKind, CatalogueEntry, Diagnostic, DressOutcome, DressRequest, DresserState, Joint, JointUpdate,
    OpenRequest, PreviewInfo, PreviewRequest, Problem, SetToolTcp, ToolTcp, ToolTcpSaved, ToolTipEstimate, ViewRequest,
};
use factoryos_protocol::scene::{MeshAsset, ModelJoints, ModelRef, Node, NodeKind, Rgba, SceneDoc, ViewHints};
use factoryos_protocol::{api, keys};
use urdf_robo_dresser::catalogue::{self, AssetStatus};
use urdf_robo_dresser::collision::{HullBuffer, generate_hulls};
use urdf_robo_dresser::{MergeOptions, Origin, Severity, merge, parse_urdf, serialize_urdf, validate};

use crate::AppState;
use crate::bus::Bus;
use crate::config_store::ConfigStore;
use crate::services::assets::AssetIndex;
use crate::services::pose3::pose3_from_xyz_rpy;
use crate::services::scene_pub::ScenePublisher;
use crate::services::tcp::{self as tcp_service, CatalogueTool, SavedTcp};
use crate::services::urdf_model::{self, UrdfModel};
use urdf_robo_dresser::tcp as dresser_tcp;

const TITLE: &str = "Robo Dresser";
/// The previewed URDF's node.
const ROBOT_NODE: &str = "robot";
/// The reach envelope's node (a mesh on the robot's root link).
const REACH_NODE: &str = "reach_envelope";
/// The robot's name over it ([`crate::services::scene_pub::robot_label_node`]).
const ROBOT_LABEL_NODE: &str = "robot/name_label";
const REACH_COLOR: Rgba = [0x7c, 0x4d, 0xff, 0x40];
/// The TCP's frame (not the URDF's own `tcp` link: that one stays as saved while the TCP is
/// edited), and what is drawn in it.
const TCP_NODE: &str = "robot/tcp";
const TCP_AXES_NODE: &str = "robot/tcp/axes";
const TCP_MARKER_NODE: &str = "robot/tcp/marker";
/// The approach arrow, in the TCP's parent link (not the TCP frame): along the tool axis.
const TCP_APPROACH_NODE: &str = "robot/tcp/approach";
/// Length of the drawn TCP axes, and of the approach arrow along the tool axis (m).
const TCP_AXIS_M: f64 = 0.1;
const APPROACH_M: f64 = 0.25;
const TCP_MARKER_COLOR: Rgba = [0xFF, 0xD6, 0x00, 0xFF];
const TCP_MARKER_RADIUS: f32 = 0.008;
const APPROACH_COLOR: Rgba = [0xFF, 0x8F, 0x00, 0xFF];
const TCP_AXIS_RADIUS: f32 = 0.003;
const APPROACH_RADIUS: f32 = 0.002;

/// The URDF in the current preview scene.
struct Preview {
    urdf: UrdfModel,
    /// The TCP drawn in it (a tool, or a dressed robot of a tool).
    tcp: Option<TcpView>,
}

/// The TCP of the previewed tool or dressed robot, as drawn.
#[derive(Clone, Debug, PartialEq)]
struct TcpView {
    /// The catalogue tool (empty: the dressed robot's tool is not in the catalogue).
    tool: String,
    /// The link the TCP is fixed to: the tool's root link, or the dressed robot's tip link.
    parent_frame: String,
    /// The tool's root-link frame in `parent_frame` (identity for a tool on its own); `None`: the
    /// dressed robot's tool mount cannot be recovered, so an edited TCP cannot be placed on it.
    mount: Option<Origin>,
    /// The tool axis in `parent_frame` (unit): the planners' approach direction.
    axis: [f64; 3],
    /// The TCP in `parent_frame`.
    pose: Origin,
}

pub struct DresserService {
    bus: Bus,
    config: Arc<ConfigStore>,
    /// The preview, a renderer-neutral 3D scene.
    scene: Arc<ScenePublisher>,
    /// Makes the preview's URDF and meshes fetchable.
    assets: Arc<AssetIndex>,
    inner: Mutex<Inner>,
    /// Called after a successful dress, e.g. so the Robo Fitter's palette picks up the new robot.
    on_dressed: Mutex<Option<Box<dyn Fn(&Path) + Send + Sync>>>,
}

#[derive(Default)]
struct Inner {
    state: DresserState,
    scanned_root: Option<String>,
    preview: Option<Preview>,
    dress_seq: u64,
    /// Bumped by every [`DresserService::load_preview`]: a slower, older load doesn't replace a newer one.
    preview_seq: u64,
}

impl DresserService {
    pub fn new(bus: &Bus, config: Arc<ConfigStore>, assets: Arc<AssetIndex>) -> Arc<Self> {
        let this = Arc::new(Self {
            bus: bus.clone(),
            config,
            scene: ScenePublisher::new(bus, keys::SCENE_DRESSER),
            assets,
            inner: Mutex::default(),
            on_dressed: Mutex::default(),
        });
        {
            let this = this.clone();
            crate::bus::serve_latest(bus, keys::DRESSER_STATE, move || Some(this.state()));
        }
        this
    }

    pub fn set_on_dressed(&self, on_dressed: impl Fn(&Path) + Send + Sync + 'static) {
        *self.on_dressed.lock().unwrap() = Some(Box::new(on_dressed));
    }

    /// Move joints of the preview as `factoryos/dresser/joint` updates arrive.
    pub async fn serve_joints(self: Arc<Self>) -> anyhow::Result<()> {
        let subscriber = self
            .bus
            .session()
            .declare_subscriber(keys::DRESSER_JOINT)
            .await
            .map_err(|err| anyhow::anyhow!("{err}"))?;
        while let Ok(sample) = subscriber.recv_async().await {
            match serde_json::from_slice::<JointUpdate>(&sample.payload().to_bytes()) {
                Ok(update) => self.set_joint(&update),
                Err(err) => tracing::warn!("{}: invalid joint update: {err}", keys::DRESSER_JOINT),
            }
        }
        Ok(())
    }

    pub fn state(&self) -> DresserState {
        self.inner.lock().unwrap().state.clone()
    }

    fn update(&self, update: impl FnOnce(&mut DresserState)) {
        let state = {
            let mut inner = self.inner.lock().unwrap();
            update(&mut inner.state);
            inner.state.clone()
        };
        self.bus.put_json(keys::DRESSER_STATE, &state);
    }

    /// The catalogue: the one [`Self::open`] scanned, else the config's (before the Dresser is
    /// first opened after a start, e.g. for a tool's TCP in Product & Task or the Fitter).
    pub(crate) fn root(&self) -> PathBuf {
        let opened = self.inner.lock().unwrap().state.root.clone();
        catalogue_root(&opened, || self.config.resolve(&self.config.get().catalogue))
    }

    /// Use the config's catalogue, (re)scanning it when it changed or `rescan` is set.
    pub fn open(self: &Arc<Self>, rescan: bool) {
        let root = self.config.resolve(&self.config.get().catalogue);
        let changed = {
            let mut inner = self.inner.lock().unwrap();
            inner.state.root.clone_from(&root);
            let changed = inner.scanned_root.as_ref() != Some(&root);
            inner.scanned_root = Some(root);
            changed
        };
        if changed || rescan {
            self.rescan();
        } else {
            self.update(|_| {});
        }
    }

    fn rescan(self: &Arc<Self>) {
        let root = self.root();
        self.spawn("Scanning catalogue…", move |this| {
            let _catalogue = crate::storage::sync::in_use();
            let dressed = list_dressed(&root);
            let scanned = catalogue::scan(&root);
            this.update(|state| {
                state.dressed = dressed;
                match scanned {
                    Ok(cat) => {
                        state.robots = cat.robots.into_iter().map(|r| entry(r.name, &r.status)).collect();
                        state.tools = cat.tools.into_iter().map(|t| entry(t.name, &t.status)).collect();
                        state.catalogue_error = None;
                        state.status = format!("Catalogue: {} robots, {} tools.", state.robots.len(), state.tools.len());
                    }
                    Err(err) => {
                        state.catalogue_error = Some(format!("{err}"));
                        state.status = "Catalogue load failed.".to_owned();
                    }
                }
            });
            Ok(())
        });
    }

    /// Run `job` on a blocking thread, showing `what` as the busy status meanwhile.
    fn spawn(self: &Arc<Self>, what: &str, job: impl FnOnce(&Arc<Self>) -> anyhow::Result<()> + Send + 'static) {
        self.update(|state| {
            state.busy = Some(what.to_owned());
            what.clone_into(&mut state.status);
        });
        let this = self.clone();
        tokio::task::spawn_blocking(move || {
            let result = job(&this);
            this.update(|state| {
                if let Err(err) = &result {
                    tracing::warn!("Robo Dresser: {err:#}");
                    state.status = format!("Failed: {err:#}");
                }
                state.busy = None;
            });
        });
    }

    pub fn preview(self: &Arc<Self>, kind: AssetKind, name: &str) {
        let path = urdf_path(&self.root(), kind, name);
        let label = match kind {
            AssetKind::Robot => format!("robot {name}"),
            AssetKind::Tool => format!("tool {name}"),
            AssetKind::Dressed => format!("dressed {name}"),
        };
        let name = name.to_owned();
        self.spawn(&format!("Loading {label}…"), move |this| {
            this.load_preview(&path, label.clone(), Some((kind, name)))?;
            this.update(|state| state.status = format!("Viewing {label}."));
            Ok(())
        });
    }

    pub fn dress(self: &Arc<Self>, request: DressRequest) {
        let root = self.root();
        let combo = format!("{}__{}", request.robot, request.tool);
        self.spawn(&format!("Dressing {combo}…"), move |this| {
            // Reads the robot's and the tool's files: no catalogue sync deletes meanwhile.
            let _catalogue = crate::storage::sync::in_use();
            let mut outcome = dress_pair(&root, &request);
            let ok = outcome.error.is_none();
            this.update(|state| {
                state.status = match &outcome.error {
                    None => format!("Dressed {combo}."),
                    Some(err) => format!("Dress failed: {err}"),
                };
                state.dressed = list_dressed(&root);
            });
            {
                let mut inner = this.inner.lock().unwrap();
                inner.dress_seq += 1;
                outcome.seq = inner.dress_seq;
                inner.state.dress = Some(outcome);
            }
            if ok {
                if let Some(on_dressed) = &*this.on_dressed.lock().unwrap() {
                    on_dressed(&root.join("urdf-dressed-robots").join(&combo));
                }
                let path = root.join("urdf-dressed-robots").join(&combo).join(format!("{combo}.urdf"));
                this.load_preview(&path, format!("dressed {combo}"), Some((AssetKind::Dressed, combo.clone())))?;
            }
            Ok(())
        });
    }

    pub fn set_view(&self, view: ViewRequest) {
        self.update(|state| {
            state.show_collision = view.show_collision;
            state.show_reach = view.show_reach;
        });
        // Held while editing the scene, so a preview loading meanwhile is edited after it is set.
        let inner = self.inner.lock().unwrap();
        if inner.preview.is_some() {
            let toggled = view_toggles(&self.scene.doc(), view.show_collision, view.show_reach);
            self.scene.update(toggled, Vec::new(), None);
        }
    }

    fn set_joint(&self, update: &JointUpdate) {
        let mut inner = self.inner.lock().unwrap();
        let Some(preview) = &inner.preview else { return };
        if preview.urdf.joint(&update.name).is_none() {
            tracing::warn!("Robo Dresser: no joint {} in {}", update.name, preview.urdf.name());
            return;
        }
        self.scene.joints(vec![ModelJoints {
            node: ROBOT_NODE.to_owned(),
            joints: BTreeMap::from([(update.name.clone(), update.value)]),
        }]);
        // Remembered for late joiners, but not published: the slider that moved it knows already.
        if let Some(joint) = inner
            .state
            .preview
            .as_mut()
            .and_then(|p| p.joints.iter_mut().find(|j| j.name == update.name))
        {
            joint.value = update.value;
        }
    }

    /// The TCP of catalogue tool `tool` (root-link frame): declared, else the tool tip.
    pub fn tool_tcp(&self, tool: &str) -> anyhow::Result<ToolTcp> {
        let catalogue = self.root();
        Ok(CatalogueTool::open(&catalogue, tool)?.tcp(&catalogue))
    }

    /// "Snap to tool tip" for catalogue tool `tool` (root-link frame).
    pub fn tool_tip(&self, tool: &str) -> anyhow::Result<ToolTipEstimate> {
        let catalogue = self.root();
        let tip = CatalogueTool::open(&catalogue, tool)?
            .tip(&catalogue)
            .with_context(|| format!("tool {tool} has no readable geometry to find its tip in"))?;
        Ok(ToolTipEstimate { xyz_m: tip })
    }

    /// Draw `request`'s TCP in the preview, when it shows that tool or a dressed robot of it
    /// (live editing, nothing saved). False when the preview shows something else.
    pub fn preview_tool_tcp(&self, request: &SetToolTcp) -> anyhow::Result<bool> {
        let pose = tcp_service::requested_pose(request)?;
        let mut inner = self.inner.lock().unwrap();
        let Some(preview) = inner.preview.as_mut() else { return Ok(false) };
        let Some(view) = preview.tcp.as_mut().filter(|v| v.tool == request.tool) else { return Ok(false) };
        let Some(mount) = view.mount else { return Ok(false) };
        view.pose = dresser_tcp::compose(&mount, &pose);
        self.scene.update(tcp_nodes(view), Vec::new(), None);
        Ok(true)
    }

    /// Save `request`'s TCP into the catalogue tool and the dressed robots made with it (see
    /// [`tcp_service::save_tool_tcp`]), and draw it in the preview.
    pub fn save_tool_tcp(&self, request: &SetToolTcp, cells_dir: Option<&Path>) -> anyhow::Result<SavedTcp> {
        let saved = tcp_service::save_tool_tcp(&self.root(), Path::new(&crate::paths::default_catalogue()), cells_dir, request)?;
        let shown = SetToolTcp { tool: request.tool.clone(), xyz_m: saved.tcp.xyz_m, rpy_deg: saved.tcp.rpy_deg };
        if let Err(err) = self.preview_tool_tcp(&shown) {
            tracing::warn!("Robo Dresser: failed to draw the TCP: {err:#}");
        }
        let changed = saved.tool_changed || !saved.updated_robots.is_empty();
        self.update(|state| {
            state.status = if changed {
                format!("Saved the TCP of {}{}.", request.tool, robots_note(&saved.updated_robots))
            } else {
                format!("The TCP of {} is unchanged.", request.tool)
            };
        });
        Ok(saved)
    }

    /// Show `urdf_path` as a new scene, with the reach envelope (if any) and the TCP (if any).
    fn load_preview(&self, urdf_path: &Path, label: String, asset: Option<(AssetKind, String)>) -> anyhow::Result<()> {
        let (show_collision, show_reach, seq) = {
            let inner = &mut *self.inner.lock().unwrap();
            inner.preview_seq += 1;
            (inner.state.show_collision, inner.state.show_reach, inner.preview_seq)
        };
        let urdf = UrdfModel::load(urdf_path)?;
        let model = self.assets.model_ref(urdf_path)?;

        // The dresser's own parser knows the `<reach_envelope hull_mesh=…>` extension element.
        let reach = parse_urdf(urdf_path)
            .ok()
            .and_then(|robot| robot.reach)
            .filter(|reach| !reach.hull_obj_rel.is_empty())
            .map(|reach| urdf_path.parent().unwrap_or(Path::new(".")).join(reach.hull_obj_rel))
            .filter(|path| path.is_file())
            .and_then(|path| match self.assets.mesh_asset(&path) {
                Ok(mesh) => Some(mesh),
                Err(err) => {
                    tracing::warn!("Robo Dresser: reach envelope {}: {err:#}", path.display());
                    None
                }
            });

        let joints: Vec<Joint> = urdf
            .slider_joints()
            .into_iter()
            .filter_map(|j| {
                let (lower, upper) = urdf_model::limits(j)?;
                Some(Joint { name: j.name.clone(), lower, upper, value: 0.0_f64.clamp(lower, upper) })
            })
            .collect();

        let catalogue = self.root();
        // A robot (bare or dressed) is labelled with its catalogue name; a tool alone is not.
        let robot_name = match &asset {
            Some((AssetKind::Tool, _)) => None,
            Some((_, name)) => Some(name.clone()),
            None => Some(urdf.name().to_owned()).filter(|_| urdf.shoulder_height().is_some()),
        };
        let tcp = asset.and_then(|(kind, name)| tcp_view(&catalogue, kind, &name, urdf_path));

        let nodes = preview_nodes(&PreviewScene {
            model,
            root_link: urdf.root_link(),
            joints: joints.iter().map(|j| (j.name.clone(), j.value)).collect(),
            show_collision,
            reach,
            show_reach,
            tcp: tcp.as_ref(),
            robot_name: robot_name.map(|name| (name, urdf.shoulder_height())),
        });
        let info = PreviewInfo { label, has_reach: nodes.iter().any(|n| n.id == REACH_NODE), joints };
        let state = {
            let mut inner = self.inner.lock().unwrap();
            if inner.preview_seq != seq {
                return Ok(());
            }
            self.scene.set(nodes, ViewHints { title: TITLE.to_owned() });
            inner.preview = Some(Preview { urdf, tcp });
            // With the scene, so the sliders' joints are always the shown robot's.
            inner.state.preview = Some(info);
            inner.state.clone()
        };
        self.bus.put_json(keys::DRESSER_STATE, &state);
        Ok(())
    }
}

/// What a preview scene shows.
struct PreviewScene<'a> {
    model: ModelRef,
    root_link: &'a str,
    joints: BTreeMap<String, f64>,
    show_collision: bool,
    reach: Option<MeshAsset>,
    show_reach: bool,
    tcp: Option<&'a TcpView>,
    /// The robot's name and shoulder height, for its label ([`ROBOT_LABEL_NODE`]).
    robot_name: Option<(String, Option<f64>)>,
}

/// The nodes of a preview scene, parents first: the robot model ([`ROBOT_NODE`]), its name label
/// ([`ROBOT_LABEL_NODE`], a robot's only), the reach envelope on its root link ([`REACH_NODE`],
/// hidden unless `show_reach`), and the TCP ([`tcp_nodes`]).
fn preview_nodes(scene: &PreviewScene) -> Vec<Node> {
    let mut nodes = vec![Node::new(
        ROBOT_NODE,
        NodeKind::Model { model: scene.model.clone(), joints: scene.joints.clone(), show_collision: scene.show_collision },
    )];
    if let Some((name, shoulder)) = &scene.robot_name {
        nodes.push(crate::services::scene_pub::robot_label_node(ROBOT_NODE, name, *shoulder));
    }
    if let Some(mesh) = &scene.reach {
        let mut reach = Node::new(REACH_NODE, NodeKind::Mesh { mesh: mesh.clone(), scale: [1.0; 3], color: Some(REACH_COLOR) })
            .under(ROBOT_NODE, Some(scene.root_link.to_owned()));
        reach.visible = scene.show_reach;
        nodes.push(reach);
    }
    if let Some(view) = scene.tcp {
        nodes.extend(tcp_nodes(view));
    }
    nodes
}

/// The TCP `view`: its frame [`TCP_NODE`] on the robot's `parent_frame` link, with RGB axes and a
/// marker labelled "TCP" in it, and the approach arrow along the tool axis (in `parent_frame`),
/// labelled "approach". The TCP's label always shows; the arrow's only on hover: Rerun puts an
/// arrow's label at its middle, which seen along the tool axis sits on the TCP's ("TCPproach").
fn tcp_nodes(view: &TcpView) -> Vec<Node> {
    let link = Some(view.parent_frame.clone());
    vec![
        Node::new(TCP_NODE, NodeKind::Group)
            .under(ROBOT_NODE, link.clone())
            .at(pose3_from_xyz_rpy(view.pose.xyz, view.pose.rpy)),
        Node::new(TCP_AXES_NODE, NodeKind::Axes { length: TCP_AXIS_M, radius: Some(TCP_AXIS_RADIUS) }).under(TCP_NODE, None),
        Node::new(
            TCP_MARKER_NODE,
            NodeKind::Points {
                points: vec![[0.0; 3]],
                colors: vec![TCP_MARKER_COLOR],
                labels: vec!["TCP".to_owned()],
                radius: Some(TCP_MARKER_RADIUS),
                show_labels: Some(true),
            },
        )
        .under(TCP_NODE, None),
        Node::new(
            TCP_APPROACH_NODE,
            NodeKind::Arrows {
                origins: vec![view.pose.xyz],
                vectors: vec![view.axis.map(|v| v * APPROACH_M)],
                colors: vec![APPROACH_COLOR],
                labels: vec!["approach".to_owned()],
                radius: Some(APPROACH_RADIUS),
                show_labels: Some(false),
            },
        )
        .under(ROBOT_NODE, link),
    ]
}

/// The nodes of `doc` that change for the view toggles: the robot's `show_collision`, the reach
/// envelope's visibility. Only those that change.
fn view_toggles(doc: &SceneDoc, show_collision: bool, show_reach: bool) -> Vec<Node> {
    let mut changed = Vec::new();
    if let Some(robot) = doc.node(ROBOT_NODE) {
        let mut robot = robot.clone();
        if let NodeKind::Model { show_collision: shown, .. } = &mut robot.kind
            && *shown != show_collision
        {
            *shown = show_collision;
            changed.push(robot);
        }
    }
    if let Some(reach) = doc.node(REACH_NODE).filter(|n| n.visible != show_reach) {
        changed.push(Node { visible: show_reach, ..reach.clone() });
    }
    changed
}

fn urdf_path(root: &Path, kind: AssetKind, name: &str) -> PathBuf {
    let collection = match kind {
        AssetKind::Robot => "urdf-robot-collection",
        AssetKind::Tool => "urdf-tool-collection",
        AssetKind::Dressed => "urdf-dressed-robots",
    };
    root.join(collection).join(name).join(format!("{name}.urdf"))
}

/// The TCP to draw for the preview of `kind` `name` (`urdf_path`) from catalogue `catalogue`: a
/// tool's declared TCP (else its tip) on its root link; a dressed robot's TCP frame (else its
/// tool's declared TCP, placed by the mount) on its tip link. `None` for a bare robot, or with no
/// TCP to show.
fn tcp_view(catalogue: &Path, kind: AssetKind, name: &str, urdf_path: &Path) -> Option<TcpView> {
    match kind {
        AssetKind::Robot => None,
        AssetKind::Tool => {
            let tool = CatalogueTool::open(catalogue, name).ok()?;
            let tcp = tool.tcp(catalogue);
            Some(TcpView {
                tool: name.to_owned(),
                parent_frame: tool.root.clone(),
                mount: Some(Origin::default()),
                axis: tool.axis(catalogue),
                pose: Origin { xyz: tcp.xyz_m, rpy: tcp.rpy_deg.map(f64::to_radians) },
            })
        }
        AssetKind::Dressed => {
            let dressed = parse_urdf(urdf_path).ok()?;
            let tip = dresser_tcp::tip_link(&dressed)?.to_owned();
            let flange = dressed.links.iter().find(|l| l.name == tip)?;
            // `<robot>__<tool>`: the tool is what follows a `__`.
            let tool = name
                .match_indices("__")
                .find_map(|(at, _)| CatalogueTool::open(catalogue, &name[at + 2..]).ok());
            let mount = tool.as_ref().and_then(|t| dresser_tcp::mount_transform(&t.urdf, flange));
            let pose = dresser_tcp::dressed_tcp(&dressed)
                .map(|(_, pose)| pose)
                .or_else(|| Some(dresser_tcp::compose(&mount?, &tool.as_ref()?.declared()?)))?;
            Some(TcpView {
                tool: tool.map(|t| t.name).unwrap_or_default(),
                parent_frame: tip,
                mount,
                // The planners' convention: the tool points along -x of the tip link.
                axis: [-1.0, 0.0, 0.0],
                pose,
            })
        }
    }
}

/// ": the dressed robots a, b updated" for the status line.
fn robots_note(robots: &[String]) -> String {
    match robots {
        [] => String::new(),
        [one] => format!("; dressed robot {one} updated"),
        many => format!("; dressed robots {} updated", many.join(", ")),
    }
}

fn entry(name: String, status: &AssetStatus) -> CatalogueEntry {
    let problem = match status {
        AssetStatus::Valid => None,
        AssetStatus::Incomplete { issues } => Some(Problem {
            error: false,
            message: issues.join(", "),
        }),
        AssetStatus::Error(err) => Some(Problem {
            error: true,
            message: err.clone(),
        }),
    };
    CatalogueEntry { name, problem }
}

fn list_dressed(root: &Path) -> Vec<CatalogueEntry> {
    let mut entries: Vec<CatalogueEntry> = std::fs::read_dir(root.join("urdf-dressed-robots"))
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|dir| {
            let name = dir.file_name().to_string_lossy().into_owned();
            dir.path()
                .join(format!("{name}.urdf"))
                .is_file()
                .then_some(CatalogueEntry { name, problem: None })
        })
        .collect();
    entries.sort_by(|a, b| a.name.cmp(&b.name));
    entries
}

/// Dress `robot` with `tool`, like the dresser's `/api/v1/dress` handler.
fn dress_pair(root: &Path, request: &DressRequest) -> DressOutcome {
    let combo = format!("{}__{}", request.robot, request.tool);
    let mut outcome = dress_into(root, request, &root.join("urdf-dressed-robots").join(&combo), &combo);
    outcome.output_rel = outcome.output_rel.map(|_| format!("urdf-dressed-robots/{combo}/{combo}.urdf"));
    outcome
}

/// Dress `robot` with `tool` into `output_dir/<name>.urdf` (its hulls and meshes beside it);
/// `output_rel` is that file's absolute path.
pub(crate) fn dress_into(root: &Path, request: &DressRequest, output_dir: &Path, name: &str) -> DressOutcome {
    let DressRequest { robot, tool, hull, orient, spin_deg, attach_to } = request;
    let mut diagnostics = Vec::new();
    let result = (|| -> anyhow::Result<String> {
        let robot_path = root.join("urdf-robot-collection").join(robot).join(format!("{robot}.urdf"));
        let tool_path = root.join("urdf-tool-collection").join(tool).join(format!("{tool}.urdf"));
        let hull_buffer = HullBuffer::from_str(hull).map_err(anyhow::Error::msg)?;

        let base = parse_urdf(&robot_path)?;
        let mut tool_urdf = parse_urdf(&tool_path)?;

        let mut diags = validate(&base, &robot_path, false);
        diags.extend(validate(&tool_urdf, &tool_path, true));
        let blocking = diags.iter().any(|d| d.severity == Severity::Error);
        diagnostics = diags
            .into_iter()
            .map(|d| {
                let location = d
                    .location
                    .map(|l| format!(" ({}:{})", l.file.file_name().unwrap_or_default().to_string_lossy(), l.element))
                    .unwrap_or_default();
                Diagnostic {
                    error: d.severity == Severity::Error,
                    text: format!("{}: {}{location}", d.code, d.message),
                }
            })
            .collect();
        anyhow::ensure!(!blocking, "URDF validation produced errors (see Diagnostics)");
        if urdf_robo_dresser::tcp::tool_tcp(&tool_urdf).is_none() {
            diagnostics.push(Diagnostic {
                error: false,
                text: format!(
                    "W030: tool {tool} declares no TCP (a `tcp` link on a fixed joint from its root): the dressed robot gets no TCP \
                     frame, and Prepare Cell can only estimate the planner's tool frame from the tool's meshes"
                ),
            });
        }

        let output_dir = output_dir.to_owned();
        std::fs::create_dir_all(&output_dir)?;

        let robot_urdf_dir = robot_path.parent().expect("file path").to_path_buf();
        let tool_urdf_dir = tool_path.parent().expect("file path").to_path_buf();
        let _ = generate_hulls(&mut tool_urdf, &tool_urdf_dir, &output_dir, "tool", hull_buffer);
        let mut base_with_collision = base.clone();
        let _ = generate_hulls(&mut base_with_collision, &robot_urdf_dir, &output_dir, "robot", hull_buffer);

        let offset = if *spin_deg == 0.0 {
            Origin::default()
        } else {
            urdf_robo_dresser::merger::spin_offset(urdf_robo_dresser::merger::flange_axis(&base, attach_to.as_deref())?, *spin_deg)
        };
        let opts = MergeOptions {
            attach_to: attach_to.clone(),
            orient: *orient,
            offset,
            tool_prefix: "tool__".to_owned(),
            output_dir: output_dir.clone(),
            robot_urdf_dir,
            tool_urdf_dir,
        };
        let dressed = merge(&base_with_collision, &tool_urdf, &opts)?;
        let file = output_dir.join(format!("{name}.urdf"));
        std::fs::write(&file, serialize_urdf(&dressed))?;
        Ok(file.to_string_lossy().into_owned())
    })();

    match result {
        Ok(rel) => DressOutcome {
            seq: 0,
            output_rel: Some(rel),
            diagnostics,
            error: None,
        },
        Err(err) => DressOutcome {
            seq: 0,
            output_rel: None,
            diagnostics,
            error: Some(format!("{err:#}")),
        },
    }
}

// --- HTTP ---

pub fn router() -> Router<AppState> {
    Router::new()
        .route(api::DRESSER_STATE, get(state))
        .route(api::DRESSER_OPEN, post(open))
        .route(api::DRESSER_PREVIEW, post(preview))
        .route(api::DRESSER_DRESS, post(dress))
        .route(api::DRESSER_VIEW, post(view))
        .route(api::DRESSER_TOOL_TCP, get(get_tool_tcp).post(set_tool_tcp))
        .route(api::DRESSER_TOOL_TCP_PREVIEW, post(preview_tool_tcp))
        .route(api::DRESSER_TOOL_TIP, get(get_tool_tip))
}

#[derive(serde::Deserialize)]
struct ToolQuery {
    tool: String,
}

type HttpError = (StatusCode, String);

fn bad_request(err: anyhow::Error) -> HttpError {
    (StatusCode::BAD_REQUEST, format!("{err:#}"))
}

/// Run `job` on a blocking thread, its error a `400`.
async fn blocking<T: Send + 'static>(job: impl FnOnce() -> anyhow::Result<T> + Send + 'static) -> Result<T, HttpError> {
    tokio::task::spawn_blocking(job)
        .await
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, format!("{err}")))?
        .map_err(bad_request)
}

async fn get_tool_tcp(State(state): State<AppState>, Query(query): Query<ToolQuery>) -> Result<Json<ToolTcp>, HttpError> {
    let dresser = state.dresser.clone();
    blocking(move || dresser.tool_tcp(&query.tool)).await.map(Json)
}

async fn get_tool_tip(State(state): State<AppState>, Query(query): Query<ToolQuery>) -> Result<Json<ToolTipEstimate>, HttpError> {
    let dresser = state.dresser.clone();
    blocking(move || dresser.tool_tip(&query.tool)).await.map(Json)
}

async fn preview_tool_tcp(State(state): State<AppState>, Json(request): Json<SetToolTcp>) -> Result<Json<()>, HttpError> {
    let dresser = state.dresser.clone();
    blocking(move || dresser.preview_tool_tcp(&request).map(|_| ())).await.map(Json)
}

/// Save a tool's TCP: in the catalogue (and the robot copies of legacy cells). The cells using a
/// changed robot have a planner model with the old TCP: a legacy cell's is removed (unprepared); a
/// reference cell's is stale by its input hashes (nothing removed). The active one is prepared
/// again right away.
async fn set_tool_tcp(State(state): State<AppState>, Json(request): Json<SetToolTcp>) -> Result<Json<ToolTcpSaved>, HttpError> {
    let (dresser, config) = (state.dresser.clone(), state.config.clone());
    let saved = blocking(move || {
        let mut saved = dresser.save_tool_tcp(&request, crate::services::cells::cells_folder(&config).as_deref())?;
        for cell in &saved.cells {
            if let Some(root) = crate::services::cells::cell_folder(&config, cell) {
                crate::services::cells::unprepare(&root)?;
            }
        }
        for cell in crate::services::cells::reference_cells_using(&config, &saved.updated_urdfs) {
            if !saved.cells.contains(&cell) {
                saved.cells.push(cell);
            }
        }
        saved.cells.sort();
        Ok(saved)
    })
    .await?;
    if !saved.updated_robots.is_empty() {
        state.bus.log(format!("TCP of tool {}: updated {}", saved.tcp.tool, saved.updated_robots.join(", ")));
    }
    let active = crate::services::cells::active_cell(&state.config).map(|(cell, _)| cell);
    if active.is_some_and(|active| saved.cells.contains(&active)) {
        state.agentic.prepare();
    }
    Ok(Json(ToolTcpSaved { tcp: saved.tcp, updated_robots: saved.updated_robots, cells_needing_prepare: saved.cells }))
}

async fn state(State(state): State<AppState>) -> Json<DresserState> {
    Json(state.dresser.state())
}

async fn open(State(state): State<AppState>, Json(request): Json<OpenRequest>) -> Json<DresserState> {
    state.dresser.open(request.rescan);
    Json(state.dresser.state())
}

async fn preview(State(state): State<AppState>, Json(request): Json<PreviewRequest>) -> Json<DresserState> {
    state.dresser.preview(request.kind, &request.name);
    Json(state.dresser.state())
}

async fn dress(State(state): State<AppState>, Json(request): Json<DressRequest>) -> Json<DresserState> {
    state.dresser.dress(request);
    Json(state.dresser.state())
}

async fn view(State(state): State<AppState>, Json(request): Json<ViewRequest>) -> Json<DresserState> {
    state.dresser.set_view(request);
    Json(state.dresser.state())
}

/// `opened`, the catalogue the Dresser scanned, else (empty: not opened since the start) `configured`.
fn catalogue_root(opened: &str, configured: impl FnOnce() -> String) -> PathBuf {
    PathBuf::from(if opened.is_empty() { configured() } else { opened.to_owned() })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::services::tcp::tests::{EXPECTED_TCP, TOOL, TestDir, assert_close};

    #[test]
    fn before_the_dresser_is_opened_the_catalogue_is_the_configs() {
        let configured = || "/var/lib/factoryos/catalogue".to_owned();
        // Not "urdf-tool-collection/…" relative to the working directory: "no tool … in the catalogue".
        assert_eq!(catalogue_root("", configured), PathBuf::from("/var/lib/factoryos/catalogue"));
        assert_eq!(catalogue_root("/data/catalogue", configured), PathBuf::from("/data/catalogue"));
    }

    /// A KR240-like arm's last two axes, `link_6`'s mesh ending 0.2396745 m out along -x (where
    /// the merger puts the tool's docking interface).
    const ROBOT: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<robot name="kr240">
  <link name="World"/>
  <link name="base_link"/>
  <link name="link_5"/>
  <link name="link_6">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="./meshes/link_6.obj" scale="1 1 1"/></geometry>
    </visual>
  </link>
  <joint name="root_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="World"/>
    <child link="base_link"/>
  </joint>
  <joint name="joint_a5" type="revolute">
    <origin xyz="0 0 0.86" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" effort="1" velocity="1"/>
  </joint>
  <joint name="joint_a6" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 3.1415927"/>
    <parent link="link_5"/>
    <child link="link_6"/>
    <axis xyz="1 0 0"/>
    <limit lower="-6" upper="6" effort="1" velocity="1"/>
  </joint>
</robot>
"#;

    #[test]
    fn dressing_writes_the_tools_tcp_as_a_frame_on_the_flange() {
        let dir = TestDir::new();
        let root = dir.path();
        let robot_dir = root.join("urdf-robot-collection/kr240");
        std::fs::create_dir_all(robot_dir.join("meshes")).unwrap();
        std::fs::write(robot_dir.join("kr240.urdf"), ROBOT).unwrap();
        std::fs::write(robot_dir.join("meshes/link_6.obj"), "v -0.2396745 0 0\nv -0.5 0.1 0.1\nv -0.5 -0.1 -0.1\nf 1 2 3\n").unwrap();
        let tool_dir = root.join("urdf-tool-collection/sticktool");
        std::fs::create_dir_all(&tool_dir).unwrap();
        std::fs::write(tool_dir.join("sticktool.urdf"), TOOL).unwrap();

        // Turned like jam2's tool: 270° about the tool axis, then 180° about z.
        let request = DressRequest { robot: "kr240".into(), tool: "sticktool".into(), hull: "tight".into(), orient: [270, 0, 180], spin_deg: 0.0, attach_to: None };
        let outcome = dress_pair(root, &request);
        assert_eq!(outcome.error, None, "{:?}", outcome.diagnostics);
        assert!(!outcome.diagnostics.iter().any(|d| d.text.starts_with("W030")), "the tool declares its TCP");
        let dressed = urdf_robo_dresser::parse_urdf(&root.join(outcome.output_rel.unwrap())).unwrap();

        let tcp_joint = dressed.joints.iter().find(|j| j.name == urdf_robo_dresser::tcp::TCP_JOINT).expect("a tcp_joint");
        assert_eq!((tcp_joint.parent.as_str(), tcp_joint.child.as_str()), ("link_6", "tcp"));
        assert_eq!(tcp_joint.joint_type, urdf_robo_dresser::JointType::Fixed);
        let (tip, pose) = urdf_robo_dresser::tcp::dressed_tcp(&dressed).unwrap();
        assert_eq!(tip, "link_6");
        assert_close(pose.xyz, EXPECTED_TCP);
        // The tool geometry is on the flange, as before.
        let flange = dressed.links.iter().find(|l| l.name == "link_6").unwrap();
        assert!(flange.visuals.len() > 1);
        assert_eq!(dressed.name, "kr240__tool", "the tcp link does not name the tool");
    }

    #[test]
    fn dressing_a_tool_without_a_tcp_warns() {
        let dir = TestDir::new();
        let root = dir.path();
        let robot_dir = root.join("urdf-robot-collection/kr240");
        std::fs::create_dir_all(&robot_dir).unwrap();
        std::fs::write(robot_dir.join("kr240.urdf"), ROBOT).unwrap();
        let tool_dir = root.join("urdf-tool-collection/bare");
        std::fs::create_dir_all(&tool_dir).unwrap();
        std::fs::write(tool_dir.join("bare.urdf"), r#"<robot name="bare"><link name="World"/></robot>"#).unwrap();
        let request = DressRequest { robot: "kr240".into(), tool: "bare".into(), hull: "tight".into(), orient: [0, 0, 180], spin_deg: 0.0, attach_to: None };
        let outcome = dress_pair(root, &request);
        assert_eq!(outcome.error, None);
        assert!(outcome.diagnostics.iter().any(|d| d.text.starts_with("W030")));
    }

    #[test]
    fn the_tcp_is_drawn_on_the_tool_and_on_the_dressed_robot() {
        let dir = TestDir::new();
        let root = dir.path();
        let robot_dir = root.join("urdf-robot-collection/kr240");
        std::fs::create_dir_all(robot_dir.join("meshes")).unwrap();
        std::fs::write(robot_dir.join("kr240.urdf"), ROBOT).unwrap();
        std::fs::write(robot_dir.join("meshes/link_6.obj"), "v -0.2396745 0 0\nv -0.5 0.1 0.1\nv -0.5 -0.1 -0.1\nf 1 2 3\n").unwrap();
        let tool_dir = root.join("urdf-tool-collection/sticktool");
        std::fs::create_dir_all(&tool_dir).unwrap();
        std::fs::write(tool_dir.join("sticktool.urdf"), TOOL).unwrap();
        let request = DressRequest { robot: "kr240".into(), tool: "sticktool".into(), hull: "tight".into(), orient: [270, 0, 180], spin_deg: 0.0, attach_to: None };
        let outcome = dress_pair(root, &request);
        assert_eq!(outcome.error, None);
        let dressed_path = root.join(outcome.output_rel.unwrap());

        let on_robot = tcp_view(root, AssetKind::Dressed, "kr240__sticktool", &dressed_path).unwrap();
        assert_eq!((on_robot.tool.as_str(), on_robot.parent_frame.as_str()), ("sticktool", "link_6"));
        assert_close(on_robot.pose.xyz, EXPECTED_TCP);
        assert_eq!(on_robot.axis, [-1.0, 0.0, 0.0], "the planners' tool axis: -x of link_6");
        // An edited TCP is placed through the mount, as saving it would write it.
        let mount = on_robot.mount.unwrap();
        let edited = dresser_tcp::compose(&mount, &Origin { xyz: [0.68, -0.0005, -0.0056], rpy: [0.0; 3] });
        assert_close(edited.xyz, [-0.9196745, EXPECTED_TCP[1], EXPECTED_TCP[2]]);

        let tool_path = urdf_path(root, AssetKind::Tool, "sticktool");
        let on_tool = tcp_view(root, AssetKind::Tool, "sticktool", &tool_path).unwrap();
        assert_eq!(on_tool.parent_frame, "World");
        assert_eq!(on_tool.mount, Some(Origin::default()));
        assert_close(on_tool.pose.xyz, [0.6695255, -0.0005, -0.0056]);
        // Its axis is the dressed robot's -x of link_6, brought back through the mount.
        assert_close(on_tool.axis, [1.0, 0.0, 0.0]);

        let robot_path = urdf_path(root, AssetKind::Robot, "kr240");
        assert!(tcp_view(root, AssetKind::Robot, "kr240", &robot_path).is_none(), "a bare robot has no TCP");
    }

    fn mesh(sha: &str) -> MeshAsset {
        MeshAsset {
            asset: factoryos_protocol::scene::AssetRef { sha256: sha.into(), size: 1, media_type: "model/obj".into() },
            sidecars: BTreeMap::new(),
            ..MeshAsset::default()
        }
    }

    fn view() -> TcpView {
        TcpView {
            tool: "sticktool".into(),
            parent_frame: "link_6".into(),
            mount: Some(Origin::default()),
            axis: [-1.0, 0.0, 0.0],
            pose: Origin { xyz: [0.5, 0.0, 0.1], rpy: [0.0, 0.0, std::f64::consts::FRAC_PI_2] },
        }
    }

    fn scene<'a>(tcp: Option<&'a TcpView>, reach: bool) -> PreviewScene<'a> {
        PreviewScene {
            model: ModelRef { urdf: mesh("urdf").asset, meshes: BTreeMap::new() },
            root_link: "World",
            joints: BTreeMap::from([("joint_a5".into(), 0.0), ("joint_a6".into(), 0.0)]),
            show_collision: false,
            reach: reach.then(|| mesh("hull")),
            show_reach: false,
            tcp,
            robot_name: None,
        }
    }

    fn parent(node: &Node) -> Option<(&str, Option<&str>)> {
        node.parent.as_ref().map(|p| (p.node.as_str(), p.link.as_deref()))
    }

    #[test]
    fn a_preview_is_the_model_its_reach_envelope_and_its_tcp() {
        let view = view();
        let nodes = preview_nodes(&scene(Some(&view), true));
        let ids: Vec<_> = nodes.iter().map(|n| n.id.as_str()).collect();
        assert_eq!(ids, [ROBOT_NODE, REACH_NODE, TCP_NODE, TCP_AXES_NODE, TCP_MARKER_NODE, TCP_APPROACH_NODE]);
        let doc = SceneDoc { nodes, ..SceneDoc::default() };
        // Parents come first.
        for (i, node) in doc.nodes.iter().enumerate() {
            if let Some(p) = &node.parent {
                assert!(doc.nodes[..i].iter().any(|n| n.id == p.node), "{} before its parent {}", node.id, p.node);
            }
        }

        let robot = doc.node(ROBOT_NODE).unwrap();
        assert_eq!((robot.parent.as_ref(), robot.visible), (None, true));
        let NodeKind::Model { joints, show_collision, .. } = &robot.kind else { panic!("{robot:?}") };
        assert_eq!((joints.len(), *show_collision), (2, false));

        let reach = doc.node(REACH_NODE).unwrap();
        assert_eq!(parent(reach), Some((ROBOT_NODE, Some("World"))), "on the root link");
        assert!(!reach.visible, "show_reach is off");
        assert!(matches!(&reach.kind, NodeKind::Mesh { mesh, color: Some(REACH_COLOR), .. } if mesh.asset.sha256 == "hull"));

        let tcp = doc.node(TCP_NODE).unwrap();
        assert_eq!(parent(tcp), Some((ROBOT_NODE, Some("link_6"))));
        assert_eq!(tcp.kind, NodeKind::Group);
        assert_eq!(tcp.pose.xyz, [0.5, 0.0, 0.1]);
        let h = std::f64::consts::FRAC_1_SQRT_2;
        for (got, want) in tcp.pose.quat_xyzw.iter().zip([0.0, 0.0, h, h]) {
            assert!((got - want).abs() < 1e-9, "{:?}", tcp.pose);
        }
        let axes = doc.node(TCP_AXES_NODE).unwrap();
        assert_eq!((parent(axes), &axes.kind), (Some((TCP_NODE, None)), &NodeKind::Axes { length: 0.1, radius: Some(0.003) }));
        let marker = doc.node(TCP_MARKER_NODE).unwrap();
        assert_eq!(parent(marker), Some((TCP_NODE, None)));
        assert!(matches!(&marker.kind, NodeKind::Points { points, labels, show_labels: Some(true), .. } if points == &[[0.0; 3]] && labels == &["TCP"]));
        // The approach arrow is in the parent link, from the TCP along the tool axis.
        let approach = doc.node(TCP_APPROACH_NODE).unwrap();
        assert_eq!(parent(approach), Some((ROBOT_NODE, Some("link_6"))));
        assert_eq!(approach.pose, factoryos_protocol::scene::Pose3::IDENTITY);
        let NodeKind::Arrows { origins, vectors, labels, radius, show_labels, .. } = &approach.kind else { panic!("{approach:?}") };
        assert_eq!((origins.as_slice(), vectors.as_slice()), (&[[0.5, 0.0, 0.1]][..], &[[-0.25, 0.0, 0.0]][..]));
        assert_eq!((labels.as_slice(), *radius, *show_labels), (&["approach".to_owned()][..], Some(0.002), Some(false)));
    }

    #[test]
    fn a_bare_robot_without_reach_is_only_the_model() {
        let nodes = preview_nodes(&scene(None, false));
        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].id, ROBOT_NODE);
    }

    #[test]
    fn a_robot_preview_carries_the_robots_name() {
        let named = PreviewScene { robot_name: Some(("kr240".into(), Some(0.675))), ..scene(None, false) };
        let nodes = preview_nodes(&named);
        let ids: Vec<_> = nodes.iter().map(|n| n.id.as_str()).collect();
        assert_eq!(ids, [ROBOT_NODE, ROBOT_LABEL_NODE]);
        let label = &nodes[1];
        assert_eq!(parent(label), Some((ROBOT_NODE, None)), "moves with the robot");
        let NodeKind::Points { points, labels, show_labels, .. } = &label.kind else { panic!("{label:?}") };
        assert_eq!((labels.as_slice(), *show_labels), (["kr240".to_owned()].as_slice(), Some(true)));
        assert!(points[0][2] > 0.675, "over the shoulder: {points:?}");
        assert_eq!(label.id, crate::services::scene_pub::robot_label_id(ROBOT_NODE));
    }

    #[test]
    fn view_toggles_edit_only_what_changes() {
        let view = view();
        let doc = SceneDoc { nodes: preview_nodes(&scene(Some(&view), true)), ..SceneDoc::default() };
        assert!(view_toggles(&doc, false, false).is_empty(), "as shown");

        let changed = view_toggles(&doc, true, false);
        assert_eq!(changed.len(), 1);
        assert!(matches!(&changed[0].kind, NodeKind::Model { show_collision: true, joints, .. } if joints.len() == 2));

        let changed = view_toggles(&doc, false, true);
        assert_eq!(changed.len(), 1);
        assert_eq!((changed[0].id.as_str(), changed[0].visible), (REACH_NODE, true));
        assert_eq!(changed[0].parent, doc.node(REACH_NODE).unwrap().parent);

        // Without a reach envelope there is nothing to show.
        let bare = SceneDoc { nodes: preview_nodes(&scene(None, false)), ..SceneDoc::default() };
        assert!(view_toggles(&bare, false, true).is_empty());
        assert!(view_toggles(&SceneDoc::default(), true, true).is_empty(), "no preview");
    }
}
