//! The tool centre point of a dressed robot, for the planner (Prepare Cell) and for upgrading
//! robots dressed before the Robo Dresser wrote a TCP frame (see `urdf_robo_dresser::tcp`).
//!
//! In order of trust, a dressed URDF's TCP is:
//! 1. its own `tcp` frame (fixed to the tip link), which the Dresser writes from the tool's
//!    declared TCP;
//! 2. for a robot dressed before that: its tool's declared TCP (catalogue `urdf-tool-collection/`,
//!    else the release's copy of the same tool), placed by the mounting recovered from the tool
//!    geometry in the dressed URDF;
//! 3. an estimate from the tool's meshes (the vertices furthest out along the tool).

use std::path::{Path, PathBuf};

use anyhow::Context as _;
use urdf_robo_dresser::tcp as dresser_tcp;
use factoryos_protocol::dresser::{SetToolTcp, ToolTcp};
use urdf_robo_dresser::{Link, Origin, UrdfRobot, parse_urdf};

use crate::config_store::ConfigStore;

/// Where a TCP came from.
#[derive(Clone, Debug, PartialEq)]
pub enum TcpSource {
    /// The dressed URDF's own TCP frame.
    Frame,
    /// The tool URDF's declared TCP (the dressed URDF has no frame of its own yet).
    Tool(PathBuf),
    /// Estimated from the tool's meshes: neither the robot nor its tool declares one.
    Estimate,
}

/// The planner's tool frame for one dressed robot: the offset in the `tip_link` frame.
#[derive(Clone, Debug, PartialEq)]
pub struct PlannerTcp {
    pub tip_link: String,
    pub offset: [f64; 3],
    pub source: TcpSource,
    /// The direction the tool works in, in the `tip_link` frame (unit): for a TCP frame, the one
    /// of its ±x that points away from the flange ([`frame_axis`]); else the planners' -x.
    pub axis: [f64; 3],
    /// The TCP frame the planner targets instead of `tip_link` + `offset`: for a tool working off
    /// the flange axis (more than [`OFF_AXIS_DEG`]) whose TCP frame's -x is that axis (a gun
    /// mounted at 90°). The planners take the approach as -x of their tool frame, and umarell keeps
    /// the frame's rotation (robots.yaml `tip_link: <frame>`, `tcp_offset: [0, 0, 0]`).
    pub frame: Option<String>,
}

/// A tool works off the flange axis when its axis is more than this (degrees) from the tip link's
/// -x, the planners' default approach.
pub const OFF_AXIS_DEG: f64 = 10.0;

/// The direction a tool with its TCP frame at `pose` (in the tip link) works in: whichever of the
/// frame's ±x points away from the flange, along the offset. The sticktool's frame works along its
/// +x, a frame written for the planners (`tip_link: <frame>`) along its -x.
pub fn frame_axis(pose: &Origin) -> [f64; 3] {
    let x = dresser_tcp::rotate(pose, [1.0, 0.0, 0.0]);
    let along: f64 = (0..3).map(|i| x[i] * pose.xyz[i]).sum();
    if along >= 0.0 { x } else { x.map(|v| -v) }
}

/// `axis`'s angle (degrees) to the planners' default approach, the tip link's -x.
fn off_flange_axis_deg(axis: [f64; 3]) -> f64 {
    let n = axis.iter().map(|v| v * v).sum::<f64>().sqrt();
    if n < 1e-12 {
        return 0.0;
    }
    (-axis[0] / n).clamp(-1.0, 1.0).acos().to_degrees()
}

/// The planner's TCP from the dressed robot's TCP frame `pose` on `tip_link` (`frame`: its link).
fn from_frame(dressed: &UrdfRobot, tip_link: String, pose: &Origin) -> PlannerTcp {
    let axis = frame_axis(pose);
    let x = dresser_tcp::rotate(pose, [1.0, 0.0, 0.0]);
    let along_minus_x = (0..3).map(|i| x[i] * axis[i]).sum::<f64>() < 0.0;
    let frame = (off_flange_axis_deg(axis) > OFF_AXIS_DEG && along_minus_x)
        .then(|| dressed.links.iter().find(|l| dresser_tcp::is_tcp_link(&l.name) && l.name != tip_link))
        .flatten()
        .map(|l| l.name.clone());
    PlannerTcp { tip_link, offset: pose.xyz, source: TcpSource::Frame, axis, frame }
}

/// A tool's planner model sidecar, beside its URDF in `urdf-tool-collection/<tool>/` (see
/// [`super::planner_model::ToolSidecar`]).
pub const TOOL_SIDECAR: &str = "planner.yaml";

/// A robot model's planner sidecar, beside its URDF in `urdf-robot-collection/<model>/` (see
/// [`super::planner_model::RobotSidecar`]).
pub const ROBOT_SIDECAR: &str = "planner.yaml";

/// The robot model of dressed robot `combo`: the part before its first `__` (`kr240r2900` of
/// `kr240r2900__WELDING_GUN`), all of it for an undressed robot.
pub fn robot_model_of(combo: &str) -> &str {
    combo.split("__").next().unwrap_or_default()
}

/// The Robo Dresser catalogues tools are looked up in: the configured one, then the release's.
#[derive(Clone, Debug, Default)]
pub struct ToolCatalogues(Vec<PathBuf>);

impl ToolCatalogues {
    pub fn new(roots: impl IntoIterator<Item = PathBuf>) -> Self {
        let mut unique: Vec<PathBuf> = Vec::new();
        for root in roots {
            let key = std::fs::canonicalize(&root).unwrap_or_else(|_| root.clone());
            if !root.as_os_str().is_empty() && !unique.iter().any(|r| std::fs::canonicalize(r).unwrap_or_else(|_| r.clone()) == key) {
                unique.push(root);
            }
        }
        Self(unique)
    }

    /// The configured catalogue, then the one the release ships.
    pub fn from_config(store: &ConfigStore) -> Self {
        Self::new([PathBuf::from(store.resolve(&store.get().catalogue)), PathBuf::from(crate::paths::default_catalogue())])
    }

    /// The planner sidecar (`urdf-tool-collection/<tool>/planner.yaml`) of the tool of dressed robot
    /// `combo` (`<robot>__<tool>`), the first catalogue's that has one, and the tool's name.
    pub fn tool_sidecar(&self, combo: &str) -> Option<(String, PathBuf)> {
        self.tool_sidecars(combo).into_iter().next()
    }

    /// Every catalogue's planner sidecar for the tool of `combo`, in lookup order (the first is the
    /// one used): all of them count for a cell's input hash.
    pub fn tool_sidecars(&self, combo: &str) -> Vec<(String, PathBuf)> {
        let mut out = Vec::new();
        for (at, _) in combo.match_indices("__") {
            let tool_name = &combo[at + 2..];
            if tool_name.is_empty() || tool_name.contains(['/', '\\']) || tool_name.starts_with('.') {
                continue;
            }
            for root in &self.0 {
                let path = root.join("urdf-tool-collection").join(tool_name).join(TOOL_SIDECAR);
                if path.is_file() {
                    out.push((tool_name.to_owned(), path));
                }
            }
        }
        out
    }

    /// The planner sidecar (`urdf-robot-collection/<model>/planner.yaml`) of the robot model of
    /// dressed robot `combo` (`<model>__<tool>`, or a bare `<model>`), the first catalogue's that
    /// has one, and the model's name.
    pub fn robot_sidecar(&self, combo: &str) -> Option<(String, PathBuf)> {
        self.robot_sidecars(combo).into_iter().next()
    }

    /// Every catalogue's planner sidecar for the robot model of `combo`, in lookup order (the
    /// first is the one used): all of them count for a cell's input hash.
    pub fn robot_sidecars(&self, combo: &str) -> Vec<(String, PathBuf)> {
        let model = robot_model_of(combo);
        if model.is_empty() || model.contains(['/', '\\']) || model.starts_with('.') {
            return Vec::new();
        }
        self.0
            .iter()
            .map(|root| root.join("urdf-robot-collection").join(model).join(ROBOT_SIDECAR))
            .filter(|path| path.is_file())
            .map(|path| (model.to_owned(), path))
            .collect()
    }

    /// The TCP the tool of dressed robot `combo` (`<robot>__<tool>`) declares, in the frame of
    /// `flange` (the dressed robot's tip link, carrying the tool geometry), with the tool URDF.
    fn declared_tcp(&self, combo: &str, flange: &Link) -> Option<(PathBuf, Origin)> {
        for (at, _) in combo.match_indices("__") {
            let tool_name = &combo[at + 2..];
            for root in &self.0 {
                let path = root.join("urdf-tool-collection").join(tool_name).join(format!("{tool_name}.urdf"));
                let Ok(tool) = parse_urdf(&path) else { continue };
                let Some(tcp) = dresser_tcp::tool_tcp(&tool) else { continue };
                if let Some(mount) = dresser_tcp::mount_transform(&tool, flange) {
                    return Some((path, dresser_tcp::compose(&mount, &tcp)));
                }
            }
        }
        None
    }
}

/// The dressed robot `urdf`'s TCP for the planner. `Ok(None)`: no TCP and no tool geometry to
/// estimate one from (a bare robot: the flange is its TCP).
pub fn planner_tcp(urdf: &Path, tools: &ToolCatalogues) -> anyhow::Result<Option<PlannerTcp>> {
    let dressed = parse_urdf(urdf).with_context(|| format!("reading {}", urdf.display()))?;
    if let Some((tip_link, pose)) = dresser_tcp::dressed_tcp(&dressed) {
        return Ok(Some(from_frame(&dressed, tip_link, &pose)));
    }
    let Some(tip_link) = dresser_tcp::tip_link(&dressed).map(str::to_owned) else {
        return Ok(None);
    };
    if let Some((tool, pose)) = flange(&dressed, &tip_link).and_then(|link| tools.declared_tcp(&combo_name(urdf), link)) {
        return Ok(Some(PlannerTcp { tip_link, offset: pose.xyz, source: TcpSource::Tool(tool), axis: [-1.0, 0.0, 0.0], frame: None }));
    }
    let estimate = dresser_tcp::estimate_tool_tip(&dressed, urdf.parent().unwrap_or(Path::new(".")));
    Ok(estimate.map(|offset| PlannerTcp { tip_link, offset, source: TcpSource::Estimate, axis: [-1.0, 0.0, 0.0], frame: None }))
}

/// Give the dressed robot `urdf` the TCP frame its tool declares, if it has none yet: the
/// upgrade of a robot dressed before the Dresser wrote one. Returns the TCP added (in the
/// tip-link frame), `None` when there was nothing to add or no declared TCP to add.
pub fn add_missing_frame(urdf: &Path, tools: &ToolCatalogues) -> anyhow::Result<Option<[f64; 3]>> {
    Ok(match sync_frame(urdf, tools, false)? {
        FrameSync::Written(xyz) => Some(xyz),
        FrameSync::Unchanged | FrameSync::NoDeclaredTcp => None,
    })
}

/// What [`update_frame`] did to a dressed robot's TCP frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FrameSync {
    /// Written (added or rewritten): the TCP now, in the tip-link frame.
    Written([f64; 3]),
    /// Already the tool's TCP.
    Unchanged,
    /// Left as it was: its tool declares no TCP, or the mount cannot be recovered from the tool
    /// geometry on the flange.
    NoDeclaredTcp,
}

/// Set the dressed robot `urdf`'s TCP frame to the one its tool declares now (added if it has
/// none): after the tool's TCP was edited. The same path as [`add_missing_frame`], but an existing
/// frame is rewritten too.
pub fn update_frame(urdf: &Path, tools: &ToolCatalogues) -> anyhow::Result<FrameSync> {
    sync_frame(urdf, tools, true)
}

fn sync_frame(urdf: &Path, tools: &ToolCatalogues, replace: bool) -> anyhow::Result<FrameSync> {
    let dressed = parse_urdf(urdf).with_context(|| format!("reading {}", urdf.display()))?;
    let existing = dresser_tcp::dressed_tcp(&dressed);
    if existing.is_some() && !replace {
        return Ok(FrameSync::Unchanged);
    }
    let Some(tip_link) = dresser_tcp::tip_link(&dressed) else { return Ok(FrameSync::NoDeclaredTcp) };
    let Some((_, pose)) = flange(&dressed, tip_link).and_then(|link| tools.declared_tcp(&combo_name(urdf), link)) else {
        return Ok(FrameSync::NoDeclaredTcp);
    };
    if existing.is_some_and(|(_, current)| dresser_tcp::same_pose(&current, &pose, SAME_POSE)) {
        return Ok(FrameSync::Unchanged);
    }
    let text = std::fs::read_to_string(urdf)?;
    let updated = dresser_tcp::set_tcp_frame_in_urdf_text(&text, &dressed, tip_link, pose)
        .context("no tcp joint or </robot> in the URDF")?;
    write_atomically(urdf, &updated)?;
    Ok(FrameSync::Written(pose.xyz))
}

/// Poses closer than this (m, and rotation-matrix entries) are the same TCP: the URDF keeps 10
/// decimals.
const SAME_POSE: f64 = 1e-8;

/// Upgrade the catalogue (`catalogue`) and legacy cells (`cells_dir`) to TCP frames, once per
/// start: a catalogue tool with no declared TCP gets the one the release's copy of the same tool
/// (`release`, identical geometry) declares, and every dressed robot (catalogue
/// `urdf-dressed-robots/`, each legacy cell's `robots/` copies) without a TCP frame gets its
/// tool's. Reference cells have no copies: they read the catalogue's (and are stale by hash).
/// Returns what it changed, for the log.
pub fn upgrade_tcp_frames(catalogue: &Path, release: &Path, cells_dir: Option<&Path>) -> Vec<String> {
    let mut changed = Vec::new();
    for tool_dir in subdirs(&catalogue.join("urdf-tool-collection")) {
        let name = file_name(&tool_dir);
        let ours = tool_dir.join(format!("{name}.urdf"));
        let shipped = release.join("urdf-tool-collection").join(&name).join(format!("{name}.urdf"));
        match adopt_shipped_tcp(&ours, &shipped) {
            Ok(true) => changed.push(format!("tool {name}: TCP declaration from the release's copy")),
            Ok(false) => {}
            Err(err) => tracing::warn!("TCP upgrade of tool {}: {err:#}", ours.display()),
        }
    }
    let tools = ToolCatalogues::new([catalogue.to_owned(), release.to_owned()]);
    let mut dressed: Vec<PathBuf> = subdirs(&catalogue.join("urdf-dressed-robots"));
    if let Some(cells_dir) = cells_dir {
        for cell in legacy_cells(cells_dir) {
            dressed.extend(subdirs(&cell.join("robots")));
        }
    }
    for dir in dressed {
        let urdf = dir.join(format!("{}.urdf", file_name(&dir)));
        if !urdf.is_file() {
            continue;
        }
        match add_missing_frame(&urdf, &tools) {
            Ok(Some(xyz)) => changed.push(format!("{}: TCP frame added at {xyz:?} (tip-link frame)", urdf.display())),
            Ok(None) => {}
            Err(err) => tracing::warn!("TCP upgrade of {}: {err:#}", urdf.display()),
        }
    }
    changed
}

/// Copy the release's TCP declaration (`shipped`) into the catalogue's tool `ours` when `ours`
/// declares none and both have the same geometry. True if `ours` changed.
fn adopt_shipped_tcp(ours: &Path, shipped: &Path) -> anyhow::Result<bool> {
    if !ours.is_file() || !shipped.is_file() || same_file(ours, shipped) {
        return Ok(false);
    }
    let tool = parse_urdf(ours)?;
    if dresser_tcp::tool_tcp(&tool).is_some() {
        return Ok(false);
    }
    let release_tool = parse_urdf(shipped)?;
    let Some(tcp) = dresser_tcp::tool_tcp(&release_tool) else { return Ok(false) };
    let visuals = |t: &UrdfRobot| t.links.iter().flat_map(|l| l.visuals.clone()).collect::<Vec<_>>();
    if visuals(&tool) != visuals(&release_tool) {
        return Ok(false);
    }
    let Some(root) = tool.links.iter().find(|l| !tool.joints.iter().any(|j| j.child == l.name)) else {
        return Ok(false);
    };
    let (link, joint) = dresser_tcp::tcp_frame(&tool, &root.name, tcp);
    let text = std::fs::read_to_string(ours)?;
    let updated = dresser_tcp::add_frame_to_urdf_text(&text, &link, &joint).context("no </robot> in the URDF")?;
    write_atomically(ours, &updated)?;
    Ok(true)
}

// --- Editing a tool's TCP (the Robo Dresser's TCP section) ---

/// A dressed robot made with a tool: in the catalogue (`urdf-dressed-robots/<combo>`) or a legacy
/// cell's copy (`<cell>/robots/<combo>`).
#[derive(Clone, Debug, PartialEq)]
pub struct DressedCopy {
    /// `<combo>`, or `<cell>/<combo>` for a cell's copy.
    pub label: String,
    pub urdf: PathBuf,
    pub cell: Option<String>,
}

/// The dressed robots made with tool `tool` (`<robot>__<tool>`): the catalogue's, then each cell's.
pub fn dressed_with(catalogue: &Path, cells_dir: Option<&Path>, tool: &str) -> Vec<DressedCopy> {
    let suffix = format!("__{tool}");
    let urdf_of = |dir: &Path| dir.join(format!("{}.urdf", file_name(dir)));
    let mut copies: Vec<DressedCopy> = subdirs(&catalogue.join("urdf-dressed-robots"))
        .into_iter()
        .filter(|dir| file_name(dir).ends_with(&suffix) && urdf_of(dir).is_file())
        .map(|dir| DressedCopy { label: file_name(&dir), urdf: urdf_of(&dir), cell: None })
        .collect();
    if let Some(cells_dir) = cells_dir {
        for cell in legacy_cells(cells_dir) {
            let cell_name = file_name(&cell);
            for dir in subdirs(&cell.join("robots")) {
                if file_name(&dir).ends_with(&suffix) && urdf_of(&dir).is_file() {
                    let label = format!("{cell_name}/{}", file_name(&dir));
                    copies.push(DressedCopy { label, urdf: urdf_of(&dir), cell: Some(cell_name.clone()) });
                }
            }
        }
    }
    copies
}

/// The URDF of catalogue tool `tool`, after checking `tool` is a plain folder name.
pub fn tool_urdf_path(catalogue: &Path, tool: &str) -> anyhow::Result<PathBuf> {
    anyhow::ensure!(
        !tool.is_empty() && !tool.starts_with('.') && !tool.contains(['/', '\\']),
        "{tool:?} is not a tool name"
    );
    let path = catalogue.join("urdf-tool-collection").join(tool).join(format!("{tool}.urdf"));
    anyhow::ensure!(path.is_file(), "no tool {tool} in the catalogue ({})", path.display());
    Ok(path)
}

/// A catalogue tool, parsed, with its root link.
pub struct CatalogueTool {
    pub name: String,
    pub path: PathBuf,
    pub urdf: UrdfRobot,
    pub root: String,
}

impl CatalogueTool {
    pub fn open(catalogue: &Path, tool: &str) -> anyhow::Result<Self> {
        let path = tool_urdf_path(catalogue, tool)?;
        let urdf = parse_urdf(&path).with_context(|| format!("reading {}", path.display()))?;
        let root = dresser_tcp::tool_root(&urdf).context("the tool URDF has no root link")?.to_owned();
        Ok(Self { name: tool.to_owned(), path, urdf, root })
    }

    fn dir(&self) -> &Path {
        self.path.parent().unwrap_or(Path::new("."))
    }

    /// The declared TCP (root-link frame).
    pub fn declared(&self) -> Option<Origin> {
        dresser_tcp::tool_tcp(&self.urdf)
    }

    /// How the tool sits on the flange of a dressed robot made with it (`catalogue`'s first
    /// whose mount can be recovered from the tool geometry).
    pub fn mount(&self, catalogue: &Path) -> Option<Origin> {
        dressed_with(catalogue, None, &self.name).into_iter().find_map(|copy| {
            let dressed = parse_urdf(&copy.urdf).ok()?;
            let tip = dresser_tcp::tip_link(&dressed)?;
            dresser_tcp::mount_transform(&self.urdf, flange(&dressed, tip)?)
        })
    }

    /// The tool axis in the root-link frame: the planners' approach direction is -x of the tip
    /// link at the TCP (the training cell's convention, `tcp_offset` only), so it is that
    /// direction brought back through the mount of a dressed robot made with the tool. Without
    /// one, the dominant direction of the tool's geometry; else +x.
    pub fn axis(&self, catalogue: &Path) -> [f64; 3] {
        if let Some(mount) = self.mount(catalogue) {
            return dresser_tcp::rotate(&dresser_tcp::invert(&mount), [-1.0, 0.0, 0.0]);
        }
        dresser_tcp::tool_working_axis(&dresser_tcp::tool_geometry_vertices(&self.urdf, self.dir())).unwrap_or([1.0, 0.0, 0.0])
    }

    /// "Snap to tool tip": the farthest point of the tool's collision geometry along its axis.
    pub fn tip(&self, catalogue: &Path) -> Option<[f64; 3]> {
        dresser_tcp::tool_tip(&self.urdf, self.dir(), self.axis(catalogue))
    }

    /// The TCP to show and edit: the declared one, else the tool tip (rotation zero).
    pub fn tcp(&self, catalogue: &Path) -> ToolTcp {
        match self.declared() {
            Some(pose) => ToolTcp { tool: self.name.clone(), xyz_m: pose.xyz, rpy_deg: pose.rpy.map(f64::to_degrees), declared: true },
            None => ToolTcp {
                tool: self.name.clone(),
                xyz_m: self.tip(catalogue).unwrap_or_default(),
                rpy_deg: [0.0; 3],
                declared: false,
            },
        }
    }
}

/// The pose a [`SetToolTcp`] asks for (root-link frame).
pub fn requested_pose(request: &SetToolTcp) -> anyhow::Result<Origin> {
    let finite = request.xyz_m.iter().chain(&request.rpy_deg).all(|v| v.is_finite());
    anyhow::ensure!(finite, "the TCP must be finite numbers");
    anyhow::ensure!(request.xyz_m.iter().all(|v| v.abs() <= 10.0), "the TCP is more than 10 m from the tool");
    Ok(Origin { xyz: request.xyz_m, rpy: request.rpy_deg.map(f64::to_radians) })
}

/// Saving a tool's TCP: what changed.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SavedTcp {
    pub tcp: ToolTcp,
    /// Whether the tool URDF itself changed.
    pub tool_changed: bool,
    /// The dressed robots whose `tcp_joint` was updated ([`DressedCopy::label`]).
    pub updated_robots: Vec<String>,
    /// The legacy cells whose robot copies those are, sorted.
    pub cells: Vec<String>,
    /// The catalogue robots' URDFs among them (what reference cells read).
    pub updated_urdfs: Vec<PathBuf>,
}

/// Declare `request`'s TCP in the catalogue tool's URDF (its `tcp` joint rewritten, or a `tcp`
/// frame added), then bring every dressed robot made with the tool up to date ([`update_frame`]:
/// the catalogue's and each cell's copy under `cells_dir`). `release` is the release's catalogue,
/// looked in for tools after `catalogue`. Saving the same TCP again changes nothing.
pub fn save_tool_tcp(catalogue: &Path, release: &Path, cells_dir: Option<&Path>, request: &SetToolTcp) -> anyhow::Result<SavedTcp> {
    let pose = requested_pose(request)?;
    let tool = CatalogueTool::open(catalogue, &request.tool)?;
    let unchanged = tool.declared().is_some_and(|current| dresser_tcp::same_pose(&current, &pose, SAME_POSE));
    if !unchanged {
        let text = std::fs::read_to_string(&tool.path)?;
        let updated = dresser_tcp::set_tcp_frame_in_urdf_text(&text, &tool.urdf, &tool.root, pose)
            .context("no tcp joint or </robot> in the tool URDF")?;
        write_atomically(&tool.path, &updated)?;
        // The user's edit of the tool: to their own catalogue additions in the store (S3 mode).
        crate::storage::saved(&tool.path);
    }
    let tools = ToolCatalogues::new([catalogue.to_owned(), release.to_owned()]);
    let mut saved = SavedTcp { tool_changed: !unchanged, ..SavedTcp::default() };
    for copy in dressed_with(catalogue, cells_dir, &request.tool) {
        match update_frame(&copy.urdf, &tools) {
            Ok(FrameSync::Written(_)) => {
                crate::storage::saved(&copy.urdf);
                saved.updated_robots.push(copy.label);
                if copy.cell.is_none() {
                    saved.updated_urdfs.push(copy.urdf.clone());
                }
                if let Some(cell) = copy.cell
                    && !saved.cells.contains(&cell)
                {
                    saved.cells.push(cell);
                }
            }
            Ok(FrameSync::Unchanged) => {}
            Ok(FrameSync::NoDeclaredTcp) => tracing::warn!(
                "TCP of {}: the tool's mount cannot be recovered from the geometry on its flange; its tcp_joint is left as it was",
                copy.urdf.display()
            ),
            Err(err) => tracing::warn!("TCP of {}: {err:#}", copy.urdf.display()),
        }
    }
    saved.cells.sort();
    saved.tcp = CatalogueTool::open(catalogue, &request.tool)?.tcp(catalogue);
    Ok(saved)
}

fn flange<'a>(dressed: &'a UrdfRobot, tip_link: &str) -> Option<&'a Link> {
    dressed.links.iter().find(|l| l.name == tip_link)
}

/// `<robot>__<tool>`, the dressed robot's folder and file name.
fn combo_name(urdf: &Path) -> String {
    urdf.file_stem().unwrap_or_default().to_string_lossy().into_owned()
}

/// The cells in `cells_dir` that have their own robot copies: not reference cells (see
/// [`super::cell_resolve`]), which read the catalogue's.
fn legacy_cells(cells_dir: &Path) -> Vec<PathBuf> {
    subdirs(cells_dir)
        .into_iter()
        .filter(|cell| {
            let reference = super::cell_resolve::read_manifest(cell).is_some_and(|m| m.is_reference());
            !reference || cell.join("scene.yaml").is_file()
        })
        .collect()
}

fn subdirs(dir: &Path) -> Vec<PathBuf> {
    let mut dirs: Vec<PathBuf> = std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.is_dir() && !file_name(p).starts_with('.'))
        .collect();
    dirs.sort();
    dirs
}

fn file_name(path: &Path) -> String {
    path.file_name().unwrap_or_default().to_string_lossy().into_owned()
}

fn same_file(a: &Path, b: &Path) -> bool {
    matches!((std::fs::canonicalize(a), std::fs::canonicalize(b)), (Ok(a), Ok(b)) if a == b)
}

/// Replace `path` with `text` without a half-written file in between.
fn write_atomically(path: &Path, text: &str) -> anyhow::Result<()> {
    let tmp = path.with_extension(format!("urdf.tcp-{}", std::process::id()));
    std::fs::write(&tmp, text)?;
    std::fs::rename(&tmp, path).inspect_err(|_| {
        std::fs::remove_file(&tmp).ok();
    })?;
    Ok(())
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A scratch folder, removed when dropped.
    pub(crate) struct TestDir(PathBuf);

    impl TestDir {
        pub(crate) fn new() -> Self {
            static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
            let n = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            let dir = std::env::temp_dir().join(format!("factoryos-tcp-{}-{n}", std::process::id()));
            std::fs::remove_dir_all(&dir).ok();
            std::fs::create_dir_all(&dir).unwrap();
            Self(dir)
        }

        pub(crate) fn path(&self) -> &Path {
            &self.0
        }
    }

    impl Drop for TestDir {
        fn drop(&mut self) {
            std::fs::remove_dir_all(&self.0).ok();
        }
    }

    /// A stick tool like the catalogue's `sticktool`: two meshes, the electrode tip declared.
    pub(crate) const TOOL: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<robot name="">
  <link name="World">
    <visual>
      <origin xyz="0.6278846 -0.0014709 -0.0051023" rpy="1.5707964 0.795223 -1.5707964"/>
      <geometry><mesh filename="./meshes/electrode.obj" scale="1. 1. 1."/></geometry>
    </visual>
    <visual>
      <origin xyz="0.0489358 -0.0038173 -0.0063185" rpy="1.5707964 1.5509135 1.5707964"/>
      <geometry><mesh filename="./meshes/frame.obj" scale="1. 1. 1."/></geometry>
    </visual>
  </link>
  <link name="tcp"/>
  <joint name="tcp_joint" type="fixed">
    <origin xyz="0.6695255 -0.0005 -0.0056" rpy="0 0 0"/>
    <parent link="World"/>
    <child link="tcp"/>
  </joint>
</robot>
"#;

    /// That tool as the Fitter-era Dresser wrote it onto a KR240's `link_6`: geometry only, turned
    /// 90° about the tool axis against the training cell (the poses of the jam2 dressed URDF).
    pub(crate) const DRESSED_WITHOUT_TCP: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<robot name="robot__tool">
  <link name="World"/>
  <link name="base_link"/>
  <link name="link_5"/>
  <link name="link_6">
    <visual>
      <origin xyz="-0.867559091 0.0051023 0.0014709" rpy="1.5707965025 -0.7755733268 1.570796255"/>
      <geometry><mesh filename="./meshes/tool/electrode.obj" scale="1 1 1"/></geometry>
    </visual>
    <visual>
      <origin xyz="-0.288610291 0.0063185 0.0038173" rpy="-1.5707963268 0.0198828268 1.5707963253"/>
      <geometry><mesh filename="./meshes/tool/frame.obj" scale="1 1 1"/></geometry>
    </visual>
  </link>
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
  <joint name="root_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="World"/>
    <child link="base_link"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
"#;

    /// The TCP of [`DRESSED_WITHOUT_TCP`] in `link_6`: the training cell's
    /// `[-0.9092, 0.0005, -0.0056]` with the tool turned 90° about its axis.
    pub(crate) const EXPECTED_TCP: [f64; 3] = [-0.9092, 0.0056, 0.0005];

    pub(crate) fn assert_close(got: [f64; 3], want: [f64; 3]) {
        assert!(got.iter().zip(want).all(|(g, w)| (g - w).abs() < 1e-5), "{got:?} != {want:?}");
    }

    /// A catalogue (`urdf-tool-collection/sticktool`, `urdf-dressed-robots/kr240__sticktool`)
    /// under `root`, the tool declaring its TCP or not.
    pub(crate) fn write_catalogue(root: &Path, tool_declares_tcp: bool) -> PathBuf {
        let tool_dir = root.join("urdf-tool-collection/sticktool");
        std::fs::create_dir_all(&tool_dir).unwrap();
        let tool = if tool_declares_tcp { TOOL.to_owned() } else { without_tcp(TOOL) };
        std::fs::write(tool_dir.join("sticktool.urdf"), tool).unwrap();
        let dressed_dir = root.join("urdf-dressed-robots/kr240__sticktool");
        std::fs::create_dir_all(&dressed_dir).unwrap();
        let urdf = dressed_dir.join("kr240__sticktool.urdf");
        std::fs::write(&urdf, DRESSED_WITHOUT_TCP).unwrap();
        urdf
    }

    fn without_tcp(tool: &str) -> String {
        let start = tool.find("  <link name=\"tcp\"/>").unwrap();
        let end = tool.find("</robot>").unwrap();
        format!("{}{}", &tool[..start], &tool[end..])
    }

    #[test]
    fn old_dressed_robots_take_the_tcp_their_tool_declares() {
        let dir = TestDir::new();
        let urdf = write_catalogue(dir.path(), true);
        let tools = ToolCatalogues::new([dir.path().to_owned()]);

        let tcp = planner_tcp(&urdf, &tools).unwrap().unwrap();
        assert_eq!(tcp.tip_link, "link_6");
        assert_eq!(tcp.source, TcpSource::Tool(dir.path().join("urdf-tool-collection/sticktool/sticktool.urdf")));
        assert_close(tcp.offset, EXPECTED_TCP);

        // The upgrade writes the frame; from then on the URDF answers for itself.
        assert_close(add_missing_frame(&urdf, &tools).unwrap().unwrap(), EXPECTED_TCP);
        assert!(add_missing_frame(&urdf, &tools).unwrap().is_none(), "idempotent");
        let tcp = planner_tcp(&urdf, &ToolCatalogues::default()).unwrap().unwrap();
        assert_eq!(tcp.source, TcpSource::Frame);
        assert_close(tcp.offset, EXPECTED_TCP);
    }

    #[test]
    fn upgrade_adopts_the_release_tool_tcp_then_frames_robots_and_cells() {
        let dir = TestDir::new();
        let (catalogue, release, cells) = (dir.path().join("catalogue"), dir.path().join("release"), dir.path().join("cells"));
        let dressed = write_catalogue(&catalogue, false);
        write_catalogue(&release, true);
        let cell_robot = cells.join("jam2/robots/kr240__sticktool");
        std::fs::create_dir_all(&cell_robot).unwrap();
        std::fs::write(cell_robot.join("kr240__sticktool.urdf"), DRESSED_WITHOUT_TCP).unwrap();

        let changed = upgrade_tcp_frames(&catalogue, &release, Some(&cells));
        assert_eq!(changed.len(), 3, "{changed:?}");
        let tool = parse_urdf(&catalogue.join("urdf-tool-collection/sticktool/sticktool.urdf")).unwrap();
        assert_close(dresser_tcp::tool_tcp(&tool).unwrap().xyz, [0.6695255, -0.0005, -0.0056]);
        for urdf in [dressed, cell_robot.join("kr240__sticktool.urdf")] {
            let robot = parse_urdf(&urdf).unwrap();
            let (tip, pose) = dresser_tcp::dressed_tcp(&robot).unwrap();
            assert_eq!(tip, "link_6");
            assert_close(pose.xyz, EXPECTED_TCP);
        }
        assert!(upgrade_tcp_frames(&catalogue, &release, Some(&cells)).is_empty(), "idempotent");
    }

    #[test]
    fn without_any_declaration_the_tcp_is_estimated_from_the_tool_meshes() {
        let dir = TestDir::new();
        let urdf = write_catalogue(dir.path(), false);
        let meshes = urdf.parent().unwrap().join("meshes/tool");
        std::fs::create_dir_all(&meshes).unwrap();
        // A rod along -x of link_6 (the mesh frame is identity here), tip at x = -0.9.
        std::fs::write(meshes.join("rod.obj"), "v -0.1 0 0\nv -0.5 0.01 0\nv -0.9 0 0.002\nv -0.9 0 -0.002\n").unwrap();
        let text = DRESSED_WITHOUT_TCP.replace(
            "  </link>\n  <joint name=\"joint_a5\"",
            "    <visual>\n      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n      <geometry><mesh filename=\"./meshes/tool/rod.obj\" scale=\"1 1 1\"/></geometry>\n    </visual>\n  </link>\n  <joint name=\"joint_a5\"",
        );
        std::fs::write(&urdf, text).unwrap();
        let tcp = planner_tcp(&urdf, &ToolCatalogues::new([dir.path().to_owned()])).unwrap().unwrap();
        assert_eq!(tcp.source, TcpSource::Estimate);
        assert_close(tcp.offset, [-0.9, 0.0, 0.0]);
    }

    fn read(path: &Path) -> String {
        std::fs::read_to_string(path).unwrap()
    }

    #[test]
    fn saving_a_tool_tcp_updates_the_tool_and_its_dressed_robots_and_is_idempotent() {
        let dir = TestDir::new();
        let (catalogue, release, cells) = (dir.path().join("catalogue"), dir.path().join("release"), dir.path().join("cells"));
        let dressed = write_catalogue(&catalogue, true);
        // The catalogue's robot has its frame (the start-up upgrade's), the cell's copy none yet.
        assert!(add_missing_frame(&dressed, &ToolCatalogues::new([catalogue.clone()])).unwrap().is_some());
        let cell_robot = cells.join("jam2/robots/kr240__sticktool/kr240__sticktool.urdf");
        std::fs::create_dir_all(cell_robot.parent().unwrap()).unwrap();
        std::fs::write(&cell_robot, DRESSED_WITHOUT_TCP).unwrap();
        // A cell with a robot of another tool is not touched.
        let other = cells.join("other/robots/kr240__gun/kr240__gun.urdf");
        std::fs::create_dir_all(other.parent().unwrap()).unwrap();
        std::fs::write(&other, DRESSED_WITHOUT_TCP).unwrap();
        // A reference cell reads the catalogue's robot: anything under its folder is not a copy.
        let reference = cells.join("jam3/robots/kr240__sticktool/kr240__sticktool.urdf");
        std::fs::create_dir_all(reference.parent().unwrap()).unwrap();
        std::fs::write(&reference, DRESSED_WITHOUT_TCP).unwrap();
        std::fs::write(cells.join("jam3/factoryos-cell.json"), r#"{"format": 2, "kind": "reference", "fitted_scene": "jam3.yaml"}"#).unwrap();

        // 10.5 mm further out along the tool (its +x), turned 10° about z.
        let request = SetToolTcp { tool: "sticktool".into(), xyz_m: [0.68, -0.0005, -0.0056], rpy_deg: [0.0, 0.0, 10.0] };
        let saved = save_tool_tcp(&catalogue, &release, Some(&cells), &request).unwrap();
        assert!(saved.tool_changed);
        assert_eq!(saved.updated_robots, ["kr240__sticktool", "jam2/kr240__sticktool"]);
        assert_eq!(saved.cells, ["jam2"]);
        assert_eq!(saved.updated_urdfs, [dressed.clone()]);
        assert_eq!(read(&reference), DRESSED_WITHOUT_TCP, "not a copy");
        assert!(saved.tcp.declared);
        assert_close(saved.tcp.xyz_m, request.xyz_m);
        assert_close(saved.tcp.rpy_deg, request.rpy_deg);

        let tool_path = catalogue.join("urdf-tool-collection/sticktool/sticktool.urdf");
        let tool = parse_urdf(&tool_path).unwrap();
        let declared = dresser_tcp::tool_tcp(&tool).unwrap();
        assert!(dresser_tcp::same_pose(&declared, &Origin { xyz: request.xyz_m, rpy: [0.0, 0.0, 10f64.to_radians()] }, 1e-9));
        assert_eq!(tool.links.iter().filter(|l| dresser_tcp::is_tcp_link(&l.name)).count(), 1);
        // The tool points along -x of link_6: the TCP moved 10.5 mm that way.
        // (EXPECTED_TCP is rounded: the declared 0.6695255 sits at x = -0.9091745.)
        let moved = [-0.9196745, EXPECTED_TCP[1], EXPECTED_TCP[2]];
        for urdf in [&dressed, &cell_robot] {
            let robot = parse_urdf(urdf).unwrap();
            let (tip, pose) = dresser_tcp::dressed_tcp(&robot).unwrap();
            assert_eq!(tip, "link_6");
            assert_close(pose.xyz, moved);
            assert_eq!(robot.links.iter().filter(|l| dresser_tcp::is_tcp_link(&l.name)).count(), 1, "{}", urdf.display());
            // The frame's rotation is the tool's TCP rotation, mounted.
            let mount = dresser_tcp::mount_transform(&tool, robot.links.iter().find(|l| l.name == "link_6").unwrap()).unwrap();
            assert!(dresser_tcp::same_pose(&pose, &dresser_tcp::compose(&mount, &declared), 1e-6));
        }
        // Prepare Cell reads the new TCP.
        let planner = planner_tcp(&cell_robot, &ToolCatalogues::default()).unwrap().unwrap();
        assert_eq!(planner.source, TcpSource::Frame);
        assert_close(planner.offset, moved);
        assert_eq!(read(&other), DRESSED_WITHOUT_TCP);

        // Saving it again changes nothing.
        let files = [tool_path.clone(), dressed.clone(), cell_robot.clone()].map(|p| read(&p));
        let again = save_tool_tcp(&catalogue, &release, Some(&cells), &request).unwrap();
        assert!(!again.tool_changed);
        assert!(again.updated_robots.is_empty() && again.cells.is_empty(), "{again:?}");
        assert_eq!([tool_path, dressed, cell_robot].map(|p| read(&p)), files);
    }

    #[tokio::test]
    async fn a_saved_tool_tcp_and_the_updated_robots_are_written_through() {
        use crate::storage::sync::tests_support::{eventually, storage_at};
        use factoryos_protocol::storage::Scope;
        let dir = TestDir::new();
        let catalogue = dir.path().join("catalogue");
        write_catalogue(&catalogue, true);
        let (s3, storage) = storage_at(dir.path());
        let _registered = crate::storage::register(&storage);
        let request = SetToolTcp { tool: "sticktool".into(), xyz_m: [0.68, -0.0005, -0.0056], rpy_deg: [0.0, 0.0, 10.0] };
        let saved = save_tool_tcp(&catalogue, &dir.path().join("release"), None, &request).unwrap();
        assert!(saved.tool_changed && saved.updated_robots == ["kr240__sticktool"], "{saved:?}");
        let (tool, robot) = ("catalogue/urdf-tool-collection/sticktool/sticktool.urdf", "catalogue/urdf-dressed-robots/kr240__sticktool/kr240__sticktool.urdf");
        eventually("the tool and its robot", || s3.bytes(Scope::User, tool).is_some() && s3.bytes(Scope::User, robot).is_some()).await;
        assert_eq!(s3.bytes(Scope::User, tool).unwrap(), std::fs::read(catalogue.join("urdf-tool-collection/sticktool/sticktool.urdf")).unwrap());
        assert!(s3.keys(Scope::Catalogue).is_empty(), "the user's own additions, never the shared catalogue");
    }

    #[test]
    fn a_tool_without_a_tcp_gets_one_and_its_robots_their_frame() {
        let dir = TestDir::new();
        let dressed = write_catalogue(dir.path(), false);
        let tool = CatalogueTool::open(dir.path(), "sticktool").unwrap();
        assert!(!tool.tcp(dir.path()).declared);
        let request = SetToolTcp { tool: "sticktool".into(), xyz_m: [0.6695255, -0.0005, -0.0056], rpy_deg: [0.0; 3] };
        let saved = save_tool_tcp(dir.path(), &dir.path().join("release"), None, &request).unwrap();
        assert_eq!(saved.updated_robots, ["kr240__sticktool"]);
        assert!(saved.cells.is_empty());
        let (_, pose) = dresser_tcp::dressed_tcp(&parse_urdf(&dressed).unwrap()).unwrap();
        assert_close(pose.xyz, EXPECTED_TCP);
    }

    #[test]
    fn bad_tool_tcp_requests_are_refused() {
        let dir = TestDir::new();
        write_catalogue(dir.path(), true);
        let release = dir.path().join("release");
        let save = |tool: &str, xyz: [f64; 3]| save_tool_tcp(dir.path(), &release, None, &SetToolTcp { tool: tool.into(), xyz_m: xyz, rpy_deg: [0.0; 3] });
        assert!(save("../sticktool", [0.0; 3]).is_err());
        assert!(save("nope", [0.0; 3]).is_err());
        assert!(save("sticktool", [f64::NAN, 0.0, 0.0]).is_err());
        assert!(save("sticktool", [100.0, 0.0, 0.0]).is_err());
    }

    /// The release's sticktool: its axis from the dressed robot's mount is its +x, and its tip is
    /// the electrode's (the declared TCP, measured on the training cell, within a few mm).
    #[test]
    fn the_release_sticktool_axis_and_tip() {
        let catalogue = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/catalogue");
        let Ok(tool) = CatalogueTool::open(&catalogue, "sticktool") else {
            eprintln!("no release sticktool: skipped");
            return;
        };
        let axis = tool.axis(&catalogue);
        assert_close(axis, [1.0, 0.0, 0.0]);
        let tip = tool.tip(&catalogue).unwrap();
        let declared = tool.declared().unwrap().xyz;
        let off = (0..3).map(|i| (tip[i] - declared[i]).powi(2)).sum::<f64>().sqrt();
        assert!(off < 0.01, "tip {tip:?} vs declared {declared:?}");
    }

    /// `DRESSED_WITHOUT_TCP` with a TCP frame on `link_6` at `xyz`, turned by `rpy`.
    pub(crate) fn dressed_with_frame(dir: &Path, xyz: &str, rpy: &str) -> PathBuf {
        let path = dir.join("kr240__gun.urdf");
        let frame = format!(
            r#"<link name="tcp"/><joint name="tcp_joint" type="fixed"><origin xyz="{xyz}" rpy="{rpy}"/><parent link="link_6"/><child link="tcp"/></joint></robot>"#
        );
        std::fs::write(&path, DRESSED_WITHOUT_TCP.replace("</robot>", &frame)).unwrap();
        path
    }

    /// A gun mounted at 90° to the flange: its TCP frame's -x (out of the tip) is link_6's -z, so
    /// the planner targets the frame itself; a frame working along the flange axis (the sticktool's,
    /// along its +x) or along its own +x off it keeps tip_link + offset.
    #[test]
    fn a_tool_working_off_the_flange_axis_is_planned_at_its_tcp_frame() {
        let dir = TestDir::new();
        let tcp = |xyz: &str, rpy: &str| planner_tcp(&dressed_with_frame(dir.path(), xyz, rpy), &ToolCatalogues::default()).unwrap().unwrap();
        let gun = tcp("-0.4906 0 -0.2973", "0 -1.5707963268 0");
        assert_eq!(gun.source, TcpSource::Frame);
        assert_close(gun.axis, [0.0, 0.0, -1.0]);
        assert_eq!(gun.frame.as_deref(), Some("tcp"));
        assert_eq!(gun.tip_link, "link_6");
        assert_close(gun.offset, [-0.4906, 0.0, -0.2973]);
        let stick = tcp("-0.9092 0.0056 0.0005", "-1.5707963268 0 3.1415926536");
        assert_close(stick.axis, [-1.0, 0.0, 0.0]);
        assert_eq!(stick.frame, None);
        let plus_x = tcp("-0.4906 0 -0.2973", "0 1.5707963268 0");
        assert_close(plus_x.axis, [0.0, 0.0, -1.0]);
        assert_eq!(plus_x.frame, None, "its -x points back at the flange: not a frame the planners can target");
    }
}
