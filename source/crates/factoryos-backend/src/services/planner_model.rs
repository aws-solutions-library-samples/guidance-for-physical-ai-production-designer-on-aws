//! What Prepare Cell adds to the planner's cell model after umarell's converter
//! (`scripts/convert_isaac_scene.py`) wrote it, for what the converter does not derive:
//! * each robot's `tcp_offset` (and `tip_link`) in `agentic_planner/config/robots.yaml`, from its
//!   dressed URDF ([`super::tcp::planner_tcp`]); the converter only takes one `--tcp-offset` for
//!   all robots and defaults it to the flange;
//! * the pedestal a robot stands on (`scene.yaml` `pedestal: {height_mm, diameter_mm}`, as the
//!   Robo Fitter draws it): the base z includes its height, and `obstacles.yaml` gets it as a box;
//! * the tool's planner model from its sidecar `urdf-tool-collection/<tool>/planner.yaml`
//!   ([`ToolSidecar`]), when it has one: the robot's `tool_collision` block, its
//!   `tool_orientation` (how far the planner may turn and lean the tool), and the cell's
//!   `collision.tool_contact_radius` (robots.yaml) and `tool_link_radius` (obstacles.yaml). A tool
//!   without one keeps what the converter fitted to its meshes;
//! * the robot model's planner model from its sidecar `urdf-robot-collection/<model>/planner.yaml`
//!   ([`RobotSidecar`]), when it has one: the robot's `joint_acceleration_limits` (a URDF has no
//!   acceleration limits; the converter writes a `# REVIEW` placeholder) and, when the sidecar gives
//!   them, its `joint_velocity_limits` and `joint_limits_lower` / `joint_limits_upper` (the robot
//!   controller's axis speeds and software limits, in place of the converter's copies of the URDF's
//!   `<limit>`). A model without one keeps the converter's values.
//!
//! The converter's files are edited line by line, so its comments and layout stay. The umarell
//! checkout itself is not touched (it is re-extracted from a pinned archive on deploy).

use std::path::{Path, PathBuf};

use anyhow::Context as _;

use super::tcp::{PlannerTcp, TcpSource, ToolCatalogues, planner_tcp};

pub const ROBOTS_YAML: &str = "agentic_planner/config/robots.yaml";
pub const OBSTACLES_YAML: &str = "agentic_planner/config/obstacles.yaml";

/// The Robo Fitter's pedestal diameter when the scene gives only a height.
const DEFAULT_PEDESTAL_DIAMETER_MM: f64 = 800.0;
/// The converter's arm sphere radius when `obstacles.yaml` has no `robot_link_radius`.
const DEFAULT_ROBOT_LINK_RADIUS: f64 = 0.12;
const PEDESTAL_MARGIN: f64 = 0.05;
/// Extra room between the pedestal box and the robot's base spheres.
const PEDESTAL_SLACK: f64 = 0.01;

/// A robot of the cell's `scene.yaml`, as the planner models it.
#[derive(Clone, Debug, PartialEq)]
pub struct RobotModel {
    /// The converter's cell-local id: `robot<N>`, N its 1-based place in `scene.yaml`.
    pub id: String,
    pub scene_id: String,
    /// World position of the base: the scene position, raised by the pedestal.
    pub base: [f64; 3],
    pub pedestal: Option<Pedestal>,
    pub tcp: Option<PlannerTcp>,
    /// Its tool's planner sidecar, if the tool has one.
    pub tool: Option<ToolSidecar>,
    /// Its robot model's planner sidecar, if the model has one.
    pub robot: Option<RobotSidecar>,
}

/// A robot model's planner model, `urdf-robot-collection/<model>/planner.yaml`: what its URDF
/// does not carry, kept with the model so Prepare Cell writes it into every cell using it.
///
/// ```yaml
/// joint_acceleration_limits: [3, 3, 3, 6, 6, 8]   # rad/s^2, one per movable joint (URDF order)
/// source: "how the numbers were obtained"           # written next to them in robots.yaml
/// estimate: true                                    # not measured / not vendor data: flagged
/// joint_velocity_limits: [1.83, 1.76, 1.87, 2.37, 2.25, 3.59]   # rad/s, > 0
/// velocity_source: "where the speeds come from"
/// joint_limits_lower: [-3.23, -2.44, -2.09, -6.11, -2.14, -6.11]  # rad; lower and upper together,
/// joint_limits_upper: [3.23, -0.087, 2.71, 6.11, 2.14, 6.11]      # lower < upper on every joint
/// limits_source: "where the joint ranges come from"
/// ```
/// Every key is optional; the lists given have one value per movable joint (the same length).
/// `estimate` flags every value of the file.
#[derive(Clone, Debug, PartialEq)]
pub struct RobotSidecar {
    pub model: String,
    pub path: PathBuf,
    pub joint_acceleration_limits: Option<Vec<f64>>,
    pub source: Option<String>,
    pub estimate: bool,
    pub joint_velocity_limits: Option<Vec<f64>>,
    pub velocity_source: Option<String>,
    pub joint_limits_lower: Option<Vec<f64>>,
    pub joint_limits_upper: Option<Vec<f64>>,
    pub limits_source: Option<String>,
}

impl RobotSidecar {
    fn empty(model: &str, path: &Path) -> Self {
        RobotSidecar {
            model: model.to_owned(),
            path: path.to_path_buf(),
            joint_acceleration_limits: None,
            source: None,
            estimate: false,
            joint_velocity_limits: None,
            velocity_source: None,
            joint_limits_lower: None,
            joint_limits_upper: None,
            limits_source: None,
        }
    }

    /// The robots.yaml keys this sidecar sets: (key, values, unit, source).
    fn keys(&self) -> Vec<(&'static str, &[f64], &'static str, Option<&str>)> {
        [
            ("joint_acceleration_limits", &self.joint_acceleration_limits, "rad/s^2", &self.source),
            ("joint_velocity_limits", &self.joint_velocity_limits, "rad/s", &self.velocity_source),
            ("joint_limits_lower", &self.joint_limits_lower, "rad", &self.limits_source),
            ("joint_limits_upper", &self.joint_limits_upper, "rad", &self.limits_source),
        ]
        .into_iter()
        .filter_map(|(key, values, unit, source)| Some((key, values.as_deref()?, unit, source.as_deref())))
        .collect()
    }
}

/// Read and check the robot model sidecar `path` of model `model`.
pub fn read_robot_sidecar(model: &str, path: &Path) -> anyhow::Result<RobotSidecar> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    parse_robot_sidecar(model, path, &text).with_context(|| format!("robot model {model}'s planner model {}", path.display()))
}

fn parse_robot_sidecar(model: &str, path: &Path, text: &str) -> anyhow::Result<RobotSidecar> {
    use serde_yaml::Value;
    let doc: Value = serde_yaml::from_str(text).context("not YAML")?;
    let doc = match doc {
        Value::Null => serde_yaml::Mapping::new(),
        Value::Mapping(m) => m,
        _ => anyhow::bail!("not a mapping"),
    };
    let numbers = |key: &str, value: &Value, positive: bool| -> anyhow::Result<Vec<f64>> {
        let items = value.as_sequence().with_context(|| format!("{key} is not a list"))?;
        anyhow::ensure!(!items.is_empty(), "{key} is empty");
        let mut out = Vec::new();
        for item in items {
            let x = item.as_f64().with_context(|| format!("{key} holds a non-number"))?;
            anyhow::ensure!(x.is_finite(), "{key} must be finite, not {x}");
            if positive {
                anyhow::ensure!(x > 0.0, "{key} must be > 0, not {x}");
            }
            out.push(x);
        }
        Ok(out)
    };
    let line = |key: &str, value: &Value| -> anyhow::Result<Option<String>> {
        let text = value.as_str().with_context(|| format!("{key} is not text"))?.trim();
        anyhow::ensure!(!text.contains('\n'), "{key} must be one line");
        Ok((!text.is_empty()).then(|| text.to_owned()))
    };
    let mut out = RobotSidecar::empty(model, path);
    for (key, value) in &doc {
        let key = key.as_str().unwrap_or_default();
        match key {
            "joint_acceleration_limits" => out.joint_acceleration_limits = Some(numbers(key, value, true)?),
            "joint_velocity_limits" => out.joint_velocity_limits = Some(numbers(key, value, true)?),
            "joint_limits_lower" => out.joint_limits_lower = Some(numbers(key, value, false)?),
            "joint_limits_upper" => out.joint_limits_upper = Some(numbers(key, value, false)?),
            "source" => out.source = line(key, value)?,
            "velocity_source" => out.velocity_source = line(key, value)?,
            "limits_source" => out.limits_source = line(key, value)?,
            "estimate" => out.estimate = value.as_bool().context("estimate is not true/false")?,
            other => anyhow::bail!(
                "unknown key {other:?} (joint_acceleration_limits, source, estimate, joint_velocity_limits, velocity_source, joint_limits_lower, joint_limits_upper, limits_source)"
            ),
        }
    }
    match (&out.joint_limits_lower, &out.joint_limits_upper) {
        (Some(lo), Some(hi)) => {
            anyhow::ensure!(lo.len() == hi.len(), "joint_limits_lower has {} values, joint_limits_upper {}", lo.len(), hi.len());
            for (i, (l, h)) in lo.iter().zip(hi).enumerate() {
                anyhow::ensure!(l < h, "joint {}: joint_limits_lower {l} is not below joint_limits_upper {h}", i + 1);
            }
        }
        (None, None) => {}
        _ => anyhow::bail!("joint_limits_lower and joint_limits_upper go together"),
    }
    let keys = out.keys();
    if let Some((first, v, _, _)) = keys.first() {
        for (key, w, _, _) in &keys[1..] {
            anyhow::ensure!(w.len() == v.len(), "{key} has {} values, {first} {} (one per movable joint)", w.len(), v.len());
        }
    }
    Ok(out)
}

/// A tool's planner model, `urdf-tool-collection/<tool>/planner.yaml`: what the converter cannot
/// fit to the tool's meshes, kept with the tool so Prepare Cell writes it into every cell using it.
///
/// ```yaml
/// tool_collision:            # replaces the robot's robots.yaml tool_collision block
///   mount_link: link_6       # required; plates / spheres / axis as umarell's robot_kinematics reads them
///   plates:
///     - {center: [-0.3, 0, 0.1], rpy: [0, 1.5708, 0], size: [0.3, 0.45], grid: [5, 5]}
/// tool_orientation:          # the robot's robots.yaml tool_orientation keys given here
///   spin_free: true          # may the planner turn the tool about its own axis
///   max_tilt_deg: 8          # how far it may lean the tool off the weld's axis
///   axis_tolerance_deg: 9    # the weld-axis check (umarell keeps max_tilt inside it)
/// collision:
///   tool_contact_radius: 0.05   # robots.yaml collision.tool_contact_radius (the largest of the cell's tools)
/// tool_link_radius: 0.02        # obstacles.yaml tool_link_radius (the largest of the cell's tools)
/// ```
/// Every key is optional; a tool_collision block needs `mount_link` and one of `plates`, `spheres`
/// or `axis`.
#[derive(Clone, Debug, PartialEq)]
pub struct ToolSidecar {
    pub tool: String,
    pub path: PathBuf,
    pub tool_collision: Option<serde_yaml::Mapping>,
    /// `tool_orientation` keys, in the order the sidecar gives them: `spin_free` (bool),
    /// `max_tilt_deg`, `axis_tolerance_deg` (degrees >= 0).
    pub tool_orientation: Vec<(String, serde_yaml::Value)>,
    pub tool_contact_radius: Option<f64>,
    pub tool_link_radius: Option<f64>,
}

/// The tool geometry forms of a `tool_collision` block.
const TOOL_GEOMETRY_KEYS: [&str; 3] = ["plates", "spheres", "axis"];

/// Read and check the sidecar `path` of tool `tool`.
pub fn read_sidecar(tool: &str, path: &Path) -> anyhow::Result<ToolSidecar> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    parse_sidecar(tool, path, &text).with_context(|| format!("tool {tool}'s planner model {}", path.display()))
}

fn parse_sidecar(tool: &str, path: &Path, text: &str) -> anyhow::Result<ToolSidecar> {
    use serde_yaml::Value;
    let doc: Value = serde_yaml::from_str(text).context("not YAML")?;
    let doc = match doc {
        Value::Null => serde_yaml::Mapping::new(),
        Value::Mapping(m) => m,
        _ => anyhow::bail!("not a mapping"),
    };
    let positive = |v: &Value, key: &str| -> anyhow::Result<f64> {
        let x = v.as_f64().with_context(|| format!("{key} is not a number"))?;
        anyhow::ensure!(x.is_finite() && x >= 0.0, "{key} must be a length in m (>= 0), not {x}");
        Ok(x)
    };
    let mut out = ToolSidecar {
        tool: tool.to_owned(),
        path: path.to_path_buf(),
        tool_collision: None,
        tool_orientation: Vec::new(),
        tool_contact_radius: None,
        tool_link_radius: None,
    };
    for (key, value) in &doc {
        match key.as_str().unwrap_or_default() {
            "tool_collision" => {
                let block = value.as_mapping().context("tool_collision is not a mapping")?;
                let mount = block.get("mount_link").and_then(Value::as_str).unwrap_or_default();
                anyhow::ensure!(!mount.trim().is_empty(), "tool_collision needs a mount_link (the flange link, e.g. link_6)");
                anyhow::ensure!(
                    TOOL_GEOMETRY_KEYS.iter().any(|k| block.get(*k).is_some_and(|v| !v.is_null())),
                    "tool_collision needs plates, spheres or axis"
                );
                out.tool_collision = Some(block.clone());
            }
            "collision" => {
                let block = value.as_mapping().context("collision is not a mapping")?;
                for (k, v) in block {
                    match k.as_str().unwrap_or_default() {
                        "tool_contact_radius" => out.tool_contact_radius = Some(positive(v, "collision.tool_contact_radius")?),
                        other => anyhow::bail!("unknown key collision.{other} (only tool_contact_radius)"),
                    }
                }
            }
            "tool_orientation" => {
                let block = value.as_mapping().context("tool_orientation is not a mapping")?;
                for (k, v) in block {
                    let k = k.as_str().unwrap_or_default();
                    let v = match k {
                        "spin_free" => Value::Bool(v.as_bool().context("tool_orientation.spin_free is not true/false")?),
                        "max_tilt_deg" | "axis_tolerance_deg" => {
                            let x = v.as_f64().with_context(|| format!("tool_orientation.{k} is not a number"))?;
                            anyhow::ensure!(x.is_finite() && (0.0..=90.0).contains(&x), "tool_orientation.{k} must be degrees in 0..=90, not {x}");
                            Value::from(x)
                        }
                        other => anyhow::bail!("unknown key tool_orientation.{other} (spin_free, max_tilt_deg, axis_tolerance_deg)"),
                    };
                    out.tool_orientation.push((k.to_owned(), v));
                }
            }
            "tool_link_radius" => out.tool_link_radius = Some(positive(value, "tool_link_radius")?),
            other => anyhow::bail!("unknown key {other:?} (tool_collision, tool_orientation, collision, tool_link_radius)"),
        }
    }
    Ok(out)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pedestal {
    pub height_m: f64,
    pub diameter_m: f64,
}

/// The robots of the scene `scene_path` (a legacy cell's own `scene.yaml`, or the Robo Fitter scene
/// `scenes/<scene>/scene.yaml` a reference cell points at), their TCPs resolved from their dressed
/// URDFs (relative paths are relative to the scene file's folder).
pub fn read_scene(scene_path: &Path, tools: &ToolCatalogues) -> anyhow::Result<Vec<RobotModel>> {
    let scene_base = scene_path.parent().unwrap_or(Path::new("."));
    let text = std::fs::read_to_string(scene_path).with_context(|| format!("reading {}", scene_path.display()))?;
    let scene: serde_yaml::Value = serde_yaml::from_str(&text).with_context(|| format!("parsing {}", scene_path.display()))?;
    let robots = scene.get("robots").and_then(|r| r.as_sequence()).cloned().unwrap_or_default();
    let mut models = Vec::new();
    for (i, robot) in robots.iter().enumerate() {
        let num = |v: Option<&serde_yaml::Value>| v.and_then(serde_yaml::Value::as_f64).unwrap_or(0.0);
        let position = robot.get("position");
        let coord = |key: &str| num(position.and_then(|p| p.get(key)));
        let pedestal = robot.get("pedestal").and_then(|p| {
            let height_mm = num(p.get("height_mm"));
            let diameter_mm = p.get("diameter_mm").and_then(serde_yaml::Value::as_f64).unwrap_or(DEFAULT_PEDESTAL_DIAMETER_MM);
            (height_mm > 0.0).then_some(Pedestal { height_m: height_mm / 1000.0, diameter_m: diameter_mm / 1000.0 })
        });
        let lift = pedestal.map_or(0.0, |p| p.height_m);
        let (tcp, tool, robot_sidecar) = match robot.get("dressed_urdf").and_then(|u| u.as_str()) {
            Some(rel) => {
                let urdf: PathBuf = super::scene_store::asset_path(scene_base, rel);
                let tcp = planner_tcp(&urdf, tools).with_context(|| format!("the TCP of {rel}"))?;
                let combo = urdf.file_stem().unwrap_or_default().to_string_lossy().into_owned();
                let tool = match tools.tool_sidecar(&combo) {
                    Some((name, path)) => Some(read_sidecar(&name, &path)?),
                    None => None,
                };
                let model = match tools.robot_sidecar(&combo) {
                    Some((name, path)) => Some(read_robot_sidecar(&name, &path)?),
                    None => None,
                };
                (tcp, tool, model)
            }
            None => (None, None, None),
        };
        models.push(RobotModel {
            id: format!("robot{}", i + 1),
            scene_id: robot.get("robot_id").and_then(|r| r.as_str()).unwrap_or_default().to_owned(),
            base: [coord("x"), coord("y"), coord("z") + lift],
            pedestal,
            tcp,
            tool,
            robot: robot_sidecar,
        });
    }
    Ok(models)
}

/// The TCP offset all robots share, for the converter's `--tcp-offset` (its log then shows the
/// real one); `None` when they differ or none has one (robots.yaml is set per robot anyway).
pub fn shared_tcp(robots: &[RobotModel]) -> Option<[f64; 3]> {
    let first = robots.first()?.tcp.as_ref()?.offset;
    robots.iter().all(|r| r.tcp.as_ref().is_some_and(|t| t.offset == first)).then_some(first)
}

/// What to tell the user about `robots`' TCPs: one line each, warnings for the guessed ones; and
/// one per robot whose tool has a planner sidecar.
pub fn describe(robots: &[RobotModel]) -> Vec<String> {
    let mut lines = describe_tcps(robots);
    for r in robots {
        let Some(tool) = &r.tool else { continue };
        let mut what = Vec::new();
        if tool.tool_collision.is_some() {
            what.push("tool_collision".to_owned());
        }
        if !tool.tool_orientation.is_empty() {
            let keys = tool.tool_orientation.iter().map(|(k, v)| format!("{k} {}", flow(v))).collect::<Vec<_>>();
            what.push(format!("tool_orientation ({})", keys.join(", ")));
        }
        if let Some(v) = tool.tool_contact_radius {
            what.push(format!("collision.tool_contact_radius {}", fmt(v)));
        }
        if let Some(v) = tool.tool_link_radius {
            what.push(format!("tool_link_radius {}", fmt(v)));
        }
        lines.push(format!(
            "{} ({}): tool {}'s planner model from {}: {}",
            r.id,
            r.scene_id,
            tool.tool,
            tool.path.display(),
            if what.is_empty() { "nothing set".to_owned() } else { what.join(", ") }
        ));
    }
    for r in robots {
        let Some(model) = &r.robot else { continue };
        let keys = model.keys();
        if keys.is_empty() {
            lines.push(format!("{} ({}): robot model {}'s planner model from {}: nothing set", r.id, r.scene_id, model.model, model.path.display()));
            continue;
        }
        for (key, values, unit, source) in keys {
            let source = source.map(|s| format!(" ({s})")).unwrap_or_default();
            let what = format!("{} ({}): {key} {} {unit} from robot model {}'s planner model {}", r.id, r.scene_id, list(values), model.model, model.path.display());
            lines.push(if model.estimate { format!("Warning: {what}: an ESTIMATE, not vendor data{source}") } else { format!("{what}{source}") });
        }
    }
    lines
}

fn describe_tcps(robots: &[RobotModel]) -> Vec<String> {
    robots
        .iter()
        .map(|r| match &r.tcp {
            None => format!("{} ({}): no tool, so the TCP is the flange", r.id, r.scene_id),
            Some(tcp) => {
                let at = format!("tcp_offset {} on {}", list(&tcp.offset), tcp.tip_link);
                match &tcp.source {
                    TcpSource::Frame => format!("{} ({}): {at}, from the dressed URDF's TCP frame", r.id, r.scene_id),
                    TcpSource::Tool(tool) => format!(
                        "Warning: {} ({}): the dressed URDF has no TCP frame (dressed before the Robo Dresser wrote one); {at} from the TCP {} declares. Dress it again to keep it in the URDF.",
                        r.id,
                        r.scene_id,
                        tool.display()
                    ),
                    TcpSource::Estimate => format!(
                        "Warning: {} ({}): neither the dressed URDF nor its tool declares a TCP; {at} is ESTIMATED from the tool meshes. Declare the tool's TCP (a `tcp` link in its URDF) and dress it again.",
                        r.id, r.scene_id
                    ),
                }
            }
        })
        .collect()
}

/// Write `robots`' TCPs, bases and tool sidecars into the converted `robots.yaml` and their
/// pedestals (and the tools' `tool_link_radius`) into `obstacles.yaml`, in cell `cell_dir`.
pub fn apply(cell_dir: &Path, robots: &[RobotModel]) -> anyhow::Result<()> {
    let robots_yaml = cell_dir.join(ROBOTS_YAML);
    let text = std::fs::read_to_string(&robots_yaml).with_context(|| format!("reading {}", robots_yaml.display()))?;
    let patched = patch_robot_model_limits(&patch_tool_orientation(&patch_tool_models(&patch_robots_yaml(&text, robots), robots), robots), robots);
    std::fs::write(&robots_yaml, patched)?;
    crate::storage::saved(&robots_yaml);
    let obstacles_yaml = cell_dir.join(OBSTACLES_YAML);
    if let Ok(text) = std::fs::read_to_string(&obstacles_yaml) {
        std::fs::write(&obstacles_yaml, patch_tool_link_radius(&patch_obstacles_yaml(&text, robots), robots))?;
        crate::storage::saved(&obstacles_yaml);
    }
    Ok(())
}

/// The converter's `agentic_planner/cell.yaml`.
pub const CELL_YAML: &str = "agentic_planner/cell.yaml";

/// A reference cell's SRDFs live in the cell (`agentic_planner/srdf/`), not beside the catalogue's
/// URDFs where the converter looks for them: name them in `cell.yaml` for each robot whose `urdf:`
/// is a key of `srdfs` (its value: the SRDF, relative to the cell folder) and that has none yet.
pub fn apply_srdfs(cell_dir: &Path, srdfs: &[(PathBuf, String)]) -> anyhow::Result<()> {
    let path = cell_dir.join(CELL_YAML);
    let text = std::fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
    let patched = patch_cell_yaml_srdfs(&text, cell_dir, srdfs);
    if patched != text {
        std::fs::write(&path, patched).with_context(|| format!("writing {}", path.display()))?;
        crate::storage::saved(&path);
    }
    Ok(())
}

/// `cell.yaml` with an `srdf:` for each robot entry whose `urdf:` (absolute, or relative to
/// `cell_dir`) is in `srdfs`, in place of the converter's "srdf: NONE FOUND" comment.
pub fn patch_cell_yaml_srdfs(text: &str, cell_dir: &Path, srdfs: &[(PathBuf, String)]) -> String {
    let lines: Vec<&str> = text.lines().collect();
    let mut out: Vec<String> = Vec::new();
    // The SRDF for the robot entry being read, until it has one.
    let mut pending: Option<String> = None;
    let mut i = 0;
    while i < lines.len() {
        let line = lines[i];
        let trimmed = line.trim_start();
        let indent = line.len() - trimmed.len();
        if (trimmed.starts_with("- id:") || (indent < 4 && !trimmed.is_empty() && !trimmed.starts_with('#')))
            && let Some(srdf) = pending.take()
        {
            out.push(format!("    srdf: {srdf}"));
        }
        if let Some(urdf) = trimmed.strip_prefix("urdf:").filter(|_| indent == 4) {
            let urdf = Path::new(urdf.trim());
            let urdf = if urdf.is_absolute() { urdf.to_path_buf() } else { cell_dir.join(urdf) };
            pending = srdfs.iter().find(|(u, _)| *u == urdf).map(|(_, srdf)| srdf.clone());
        } else if trimmed.starts_with("srdf:") && indent == 4 {
            pending = None;
        } else if pending.is_some() && trimmed.starts_with("# srdf: NONE FOUND") {
            out.push(format!("    srdf: {}", pending.take().unwrap_or_default()));
            // and the comment's continuation line
            if lines.get(i + 1).is_some_and(|next| next.trim_start().starts_with("#       ")) {
                i += 1;
            }
            i += 1;
            continue;
        }
        out.push(line.to_owned());
        i += 1;
    }
    if let Some(srdf) = pending.take() {
        out.push(format!("    srdf: {srdf}"));
    }
    let mut patched = out.join("\n");
    if text.ends_with('\n') {
        patched.push('\n');
    }
    patched
}

/// `robots.yaml` with each robot block's `base_position`, `tip_link` and `tcp_offset` set from
/// `robots` (matched on the `robot<N>:` key).
pub fn patch_robots_yaml(text: &str, robots: &[RobotModel]) -> String {
    let mut current: Option<&RobotModel> = None;
    let mut out: Vec<String> = Vec::new();
    for line in text.lines() {
        let indent = line.len() - line.trim_start().len();
        let trimmed = line.trim();
        if indent == 4 && trimmed.ends_with(':') && !trimmed.starts_with('#') {
            let key = trimmed.trim_end_matches(':');
            current = robots.iter().find(|r| r.id == key);
        } else if indent < 4 && !trimmed.is_empty() && !trimmed.starts_with('#') {
            current = None;
        }
        let Some(robot) = current.filter(|_| indent == 6) else {
            out.push(line.to_owned());
            continue;
        };
        let pad = " ".repeat(indent);
        let replaced = match trimmed.split(':').next() {
            Some("base_position") => Some(format!(
                "{pad}base_position: {}   # world frame (m), from scene{}",
                list(&robot.base),
                robot.pedestal.map(|p| format!(" + its {} m pedestal", fmt(p.height_m))).unwrap_or_default()
            )),
            Some("tip_link") => robot.tcp.as_ref().map(|t| match &t.frame {
                Some(frame) => format!(
                    "{pad}tip_link: {frame}   # the dressed URDF's TCP frame: its tool works off the flange axis (its -x)"
                ),
                None => format!("{pad}tip_link: {}   # its parent, from the URDF", t.tip_link),
            }),
            Some("tcp_offset") if robot.tcp.as_ref().is_some_and(|t| t.frame.is_some()) => {
                Some(format!("{pad}tcp_offset: [0, 0, 0]   # m, the TCP frame itself (tip_link), from the dressed URDF (Prepare Cell)"))
            }
            Some("tcp_offset") => robot.tcp.as_ref().map(|t| {
                let source = match &t.source {
                    TcpSource::Frame => "the dressed URDF's TCP frame".to_owned(),
                    TcpSource::Tool(_) => "the tool's declared TCP".to_owned(),
                    TcpSource::Estimate => "ESTIMATED from the tool meshes (REVIEW)".to_owned(),
                };
                format!("{pad}tcp_offset: {}   # m, in the tip-link frame, from {source} (Prepare Cell)", list(&t.offset))
            }),
            _ => None,
        };
        out.push(replaced.unwrap_or_else(|| line.to_owned()));
    }
    let mut patched = out.join("\n");
    if text.ends_with('\n') {
        patched.push('\n');
    }
    patched
}

/// `obstacles.yaml` with a box per pedestal at the top of its `obstacles:` list. The box is the
/// pedestal cylinder's bounding box, standing on the floor like the Robo Fitter's, with its top
/// kept `robot_link_radius + margin` below the robot base: the planner models the robot's base
/// frames as spheres of that radius sitting on the pedestal, which must not collide with it.
pub fn patch_obstacles_yaml(text: &str, robots: &[RobotModel]) -> String {
    let link_radius = top_level_number(text, "robot_link_radius").unwrap_or(DEFAULT_ROBOT_LINK_RADIUS);
    let mut entries = String::new();
    for robot in robots {
        let Some(pedestal) = robot.pedestal else { continue };
        let id = format!("{}_pedestal", robot.id);
        if text.contains(&format!("- id: {id}\n")) {
            continue;
        }
        let top = robot.base[2] - link_radius - PEDESTAL_MARGIN - PEDESTAL_SLACK;
        if top <= 0.0 {
            continue; // Too low to model: the floor covers it.
        }
        let r = pedestal.diameter_m / 2.0;
        entries.push_str(&format!(
            "\n  # {} ({}) pedestal: {} m high, {} m across (Prepare Cell, from scene.yaml), as its bounding\n  # box; its top is kept {} m under the robot base, where the planner's base spheres sit.\n  - id: {id}\n    type: box\n    description: \"{} pedestal\"\n    center: {}\n    half_extents: {}\n    collision_margin: {}\n",
            robot.id,
            robot.scene_id,
            fmt(pedestal.height_m),
            fmt(pedestal.diameter_m),
            fmt(robot.base[2] - top),
            robot.scene_id,
            list(&[robot.base[0], robot.base[1], top / 2.0]),
            list(&[r, r, top / 2.0]),
            fmt(PEDESTAL_MARGIN),
        ));
    }
    if entries.is_empty() {
        return text.to_owned();
    }
    match text.find("\nobstacles:\n").map(|at| at + "\nobstacles:\n".len()).or_else(|| text.starts_with("obstacles:\n").then_some("obstacles:\n".len())) {
        Some(at) => format!("{}{}{}", &text[..at], entries.trim_start_matches('\n'), &text[at..]),
        None => format!("{text}\nobstacles:\n{}", entries.trim_start_matches('\n')),
    }
}

/// `robots.yaml` with the tool sidecars of `robots` in: each such robot's `tool_collision` block
/// (and the comment lines right above it, the converter's description of its own block) replaced
/// by the sidecar's, and `collision.tool_contact_radius` set to the largest any sidecar gives.
pub fn patch_tool_models(text: &str, robots: &[RobotModel]) -> String {
    let lines: Vec<&str> = text.lines().collect();
    let mut out: Vec<String> = Vec::new();
    let is_code = |l: &str| !l.trim().is_empty() && !l.trim_start().starts_with('#');
    let indent = |l: &str| l.len() - l.trim_start().len();
    let mut i = 0;
    while i < lines.len() {
        let line = lines[i];
        let trimmed = line.trim();
        let sidecar = (indent(line) == 4 && trimmed.ends_with(':') && !trimmed.starts_with('#'))
            .then(|| robots.iter().find(|r| r.id == trimmed.trim_end_matches(':')))
            .flatten()
            .and_then(|r| Some((r.tool.as_ref()?, r.tool.as_ref()?.tool_collision.as_ref()?)));
        out.push(line.to_owned());
        i += 1;
        let Some((tool, block)) = sidecar else { continue };
        // The robot's block: up to the next line of code at indent <= 4 ...
        let mut end = i;
        while end < lines.len() && !(is_code(lines[end]) && indent(lines[end]) <= 4) {
            end += 1;
        }
        // ... without the blank lines and comments before it.
        while end > i && !(is_code(lines[end - 1]) && indent(lines[end - 1]) > 4) {
            end -= 1;
        }
        let new_block = tool_collision_lines(tool, block);
        let mut replaced = false;
        let mut k = i;
        while k < end {
            let l = lines[k];
            if !replaced && indent(l) == 6 && l.trim_start().starts_with("tool_collision:") {
                // The comment lines right above it describe the converter's block.
                while out.last().is_some_and(|prev| indent(prev) == 6 && prev.trim_start().starts_with('#')) {
                    out.pop();
                }
                out.extend(new_block.iter().cloned());
                replaced = true;
                k += 1;
                // Its lines: everything indented deeper, up to the robot's next key.
                while k < end && (lines[k].trim().is_empty() || indent(lines[k]) > 6) {
                    k += 1;
                }
                continue;
            }
            out.push(l.to_owned());
            k += 1;
        }
        if !replaced {
            out.extend(new_block.iter().cloned());
        }
        i = end;
    }
    let radius = robots.iter().filter_map(|r| r.tool.as_ref()?.tool_contact_radius).reduce(f64::max);
    let mut patched = out.join("\n");
    if text.ends_with('\n') {
        patched.push('\n');
    }
    match radius {
        Some(r) => set_block_number(&patched, "collision", "tool_contact_radius", r, "the largest of the cell's tools' planner.yaml (Prepare Cell)"),
        None => patched,
    }
}

/// `robots.yaml` with each robot's `tool_orientation` keys set from its tool's sidecar: the keys the
/// sidecar gives replace the converter's (in place), missing ones are added to the block, and a
/// robot without the block gets one at the end of its entry.
pub fn patch_tool_orientation(text: &str, robots: &[RobotModel]) -> String {
    let lines: Vec<&str> = text.lines().collect();
    let is_code = |l: &str| !l.trim().is_empty() && !l.trim_start().starts_with('#');
    let indent = |l: &str| l.len() - l.trim_start().len();
    let mut out: Vec<String> = Vec::new();
    let mut i = 0;
    while i < lines.len() {
        let line = lines[i];
        let trimmed = line.trim();
        let tool = (indent(line) == 4 && trimmed.ends_with(':') && !trimmed.starts_with('#'))
            .then(|| robots.iter().find(|r| r.id == trimmed.trim_end_matches(':')))
            .flatten()
            .and_then(|r| r.tool.as_ref())
            .filter(|t| !t.tool_orientation.is_empty());
        out.push(line.to_owned());
        i += 1;
        let Some(tool) = tool else { continue };
        let note = format!("   # tool {}'s planner.yaml (Prepare Cell)", tool.tool);
        let entry = |pad: &str, key: &str, v: &serde_yaml::Value| format!("{pad}{key}: {}{note}", flow(v));
        let mut end = i;
        while end < lines.len() && !(is_code(lines[end]) && indent(lines[end]) <= 4) {
            end += 1;
        }
        while end > i && !(is_code(lines[end - 1]) && indent(lines[end - 1]) > 4) {
            end -= 1;
        }
        let mut found = false;
        let mut k = i;
        while k < end {
            let l = lines[k];
            if found || !(indent(l) == 6 && l.trim_start().starts_with("tool_orientation:")) {
                out.push(l.to_owned());
                k += 1;
                continue;
            }
            found = true;
            out.push(l.to_owned());
            k += 1;
            let mut seen = Vec::new();
            while k < end && (lines[k].trim().is_empty() || indent(lines[k]) > 6) {
                let child = lines[k];
                let key = child.trim_start().split(':').next().unwrap_or_default();
                match tool.tool_orientation.iter().find(|(name, _)| is_code(child) && indent(child) == 8 && name == key) {
                    Some((name, v)) => {
                        out.push(entry("        ", name, v));
                        seen.push(name.clone());
                    }
                    None => out.push(child.to_owned()),
                }
                k += 1;
            }
            for (name, v) in tool.tool_orientation.iter().filter(|(name, _)| !seen.contains(name)) {
                out.push(entry("        ", name, v));
            }
        }
        if !found {
            out.push("      tool_orientation:".to_owned());
            out.extend(tool.tool_orientation.iter().map(|(name, v)| entry("        ", name, v)));
        }
        i = end;
    }
    let mut patched = out.join("\n");
    if text.ends_with('\n') {
        patched.push('\n');
    }
    patched
}

/// `robots.yaml` with each robot's `joint_acceleration_limits` (and `joint_velocity_limits`,
/// `joint_limits_lower`, `joint_limits_upper`, when the sidecar gives them) set from its robot model's
/// sidecar (matched on the `robot<N>:` key), in place of the converter's (its `# REVIEW` placeholder,
/// its copies of the URDF's `<limit>`). A sidecar list with another length than the converter's
/// (another joint count) is not applied.
pub fn patch_robot_model_limits(text: &str, robots: &[RobotModel]) -> String {
    let mut current: Option<&RobotModel> = None;
    let mut out: Vec<String> = Vec::new();
    for line in text.lines() {
        let indent = line.len() - line.trim_start().len();
        let trimmed = line.trim();
        if indent == 4 && trimmed.ends_with(':') && !trimmed.starts_with('#') {
            current = robots.iter().find(|r| r.id == trimmed.trim_end_matches(':'));
        } else if indent < 4 && !trimmed.is_empty() && !trimmed.starts_with('#') {
            current = None;
        }
        let model = current.filter(|_| indent == 6).and_then(|r| r.robot.as_ref());
        let hit = model.and_then(|m| {
            m.keys().into_iter().find_map(|(key, values, unit, source)| {
                let rest = trimmed.strip_prefix(key)?.strip_prefix(':')?;
                Some((m, key, values, unit, source, rest))
            })
        });
        let Some((model, key, values, unit, source, rest)) = hit else {
            out.push(line.to_owned());
            continue;
        };
        let old = rest.split('#').next().unwrap_or_default().trim().trim_start_matches('[').trim_end_matches(']');
        if old.split(',').filter(|v| !v.trim().is_empty()).count() != values.len() {
            out.push(line.to_owned());
            continue;
        }
        let what = if model.estimate { "ESTIMATE" } else { unit };
        let source = source.map(|s| format!(": {s}")).unwrap_or_default();
        out.push(format!(
            "{}{key}: {}   # {what}, robot model {}'s planner.yaml (Prepare Cell){source}",
            " ".repeat(indent),
            list(values),
            model.model
        ));
    }
    let mut patched = out.join("\n");
    if text.ends_with('\n') {
        patched.push('\n');
    }
    patched
}

/// The robots.yaml lines of a sidecar's `tool_collision` block (at a robot's indent).
fn tool_collision_lines(tool: &ToolSidecar, block: &serde_yaml::Mapping) -> Vec<String> {
    let mut out = vec![
        format!("      # From tool {}'s planner model {} (Prepare Cell):", tool.tool, tool.path.display()),
        "      # edit it there; this block is rewritten on every Prepare Cell.".to_owned(),
        "      tool_collision:".to_owned(),
    ];
    emit_mapping(block, 8, &mut out);
    out
}

/// `map` as block YAML at `indent`; sequences of scalars, and the items of other sequences, in
/// flow style (`[x, y, z]`, `{center: [..], size: [..]}`).
fn emit_mapping(map: &serde_yaml::Mapping, indent: usize, out: &mut Vec<String>) {
    let pad = " ".repeat(indent);
    for (key, value) in map {
        let key = scalar(key);
        match value {
            serde_yaml::Value::Mapping(m) if !m.is_empty() => {
                out.push(format!("{pad}{key}:"));
                emit_mapping(m, indent + 2, out);
            }
            serde_yaml::Value::Sequence(items) if !items.is_empty() && !items.iter().all(is_scalar) => {
                out.push(format!("{pad}{key}:"));
                out.extend(items.iter().map(|item| format!("{pad}  - {}", flow(item))));
            }
            other => out.push(format!("{pad}{key}: {}", flow(other))),
        }
    }
}

fn is_scalar(v: &serde_yaml::Value) -> bool {
    !matches!(v, serde_yaml::Value::Sequence(_) | serde_yaml::Value::Mapping(_) | serde_yaml::Value::Tagged(_))
}

fn scalar(v: &serde_yaml::Value) -> String {
    match v {
        serde_yaml::Value::Number(n) if !n.is_i64() && !n.is_u64() => n.as_f64().map_or_else(|| n.to_string(), fmt),
        other => serde_yaml::to_string(other).map(|s| s.trim_end().to_owned()).unwrap_or_default(),
    }
}

/// `v` in flow style.
fn flow(v: &serde_yaml::Value) -> String {
    match v {
        serde_yaml::Value::Sequence(items) => format!("[{}]", items.iter().map(flow).collect::<Vec<_>>().join(", ")),
        serde_yaml::Value::Mapping(m) => {
            format!("{{{}}}", m.iter().map(|(k, v)| format!("{}: {}", scalar(k), flow(v))).collect::<Vec<_>>().join(", "))
        }
        serde_yaml::Value::Tagged(t) => flow(&t.value),
        other => scalar(other),
    }
}

/// `text` with `<block>.<key>: <value>` set (the top-level `block:` mapping added if missing).
fn set_block_number(text: &str, block: &str, key: &str, value: f64, note: &str) -> String {
    let lines: Vec<&str> = text.lines().collect();
    let header = format!("{block}:");
    let entry = format!("  {key}: {}   # {note}", fmt(value));
    let mut out: Vec<String> = lines.iter().map(|l| (*l).to_owned()).collect();
    match lines.iter().position(|l| l.split('#').next().unwrap_or_default().trim_end() == header) {
        Some(at) => {
            let end = (at + 1..lines.len())
                .find(|&k| !lines[k].trim().is_empty() && !lines[k].starts_with(' ') && !lines[k].starts_with('#'))
                .unwrap_or(lines.len());
            let prefix = format!("  {key}:");
            match (at + 1..end).find(|&k| lines[k].starts_with(&prefix)) {
                Some(k) => out[k] = entry,
                None => out.insert(at + 1, entry),
            }
        }
        None => {
            if out.last().is_some_and(|l| !l.trim().is_empty()) {
                out.push(String::new());
            }
            out.push(header);
            out.push(entry);
        }
    }
    out.join("\n") + "\n"
}

/// `obstacles.yaml` with the top-level `tool_link_radius` from the tools' sidecars: the largest
/// they give, or the converter's own value if that is larger and a tooled robot has no sidecar
/// value (its tool was measured by the converter, into the same one number).
pub fn patch_tool_link_radius(text: &str, robots: &[RobotModel]) -> String {
    let Some(mut radius) = robots.iter().filter_map(|r| r.tool.as_ref()?.tool_link_radius).reduce(f64::max) else {
        return text.to_owned();
    };
    let all = robots
        .iter()
        .filter(|r| r.tcp.is_some() || r.tool.is_some())
        .all(|r| r.tool.as_ref().is_some_and(|t| t.tool_link_radius.is_some()));
    if !all && let Some(converter) = top_level_number(text, "tool_link_radius") {
        radius = radius.max(converter);
    }
    let line = format!("tool_link_radius: {}   # from the cell's tools' planner.yaml (Prepare Cell)", fmt(radius));
    let mut found = false;
    let mut out: Vec<String> = text
        .lines()
        .map(|l| {
            if !found && l.strip_prefix("tool_link_radius").is_some_and(|rest| rest.trim_start().starts_with(':')) {
                found = true;
                line.clone()
            } else {
                l.to_owned()
            }
        })
        .collect();
    if !found {
        out.push(line);
    }
    out.join("\n") + "\n"
}

/// A top-level `key: <number>` of a YAML file.
fn top_level_number(text: &str, key: &str) -> Option<f64> {
    text.lines().find_map(|line| {
        let rest = line.strip_prefix(key)?.strip_prefix(':')?;
        rest.split('#').next()?.trim().parse().ok()
    })
}

fn fmt(v: f64) -> String {
    let s = format!("{v:.7}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".to_owned() } else { s.to_owned() }
}

fn list(values: &[f64]) -> String {
    format!("[{}]", values.iter().map(|v| fmt(*v)).collect::<Vec<_>>().join(", "))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::services::tcp::{ROBOT_SIDECAR, TOOL_SIDECAR};
    use crate::services::tcp::tests::{DRESSED_WITHOUT_TCP, EXPECTED_TCP, TestDir, assert_close, write_catalogue};

    const SCENE: &str = "name: jam2
robots:
- robot_id: kr240__sticktool
  dressed_urdf: robots/kr240__sticktool/kr240__sticktool.urdf
  position:
    x: 2.5
    y: 0.0
    z: 0.0
  orientation:
    a: 180.0
  pedestal:
    height_mm: 1000.0
    diameter_mm: 1500.0
- robot_id: kr240__sticktool_2
  dressed_urdf: robots/kr240__sticktool/kr240__sticktool.urdf
  position:
    x: -2.5
    y: 0.0
    z: 0.0
fixtures: []
";

    /// The converter's robots.yaml for SCENE (abridged), TCP on the flange, bases on the floor.
    const ROBOTS: &str = "robots:
  arms:                     # ROS namespace (ROBOT_NAMESPACE)
    robot1:
      name: kr240__sticktool
      base_position: [2.5, 0, 0]   # world frame (m), from scene
      base_orientation_yaw: 3.141593       # rad, from scene.yaml orientation.a (deg)
      end_effector_link: welding_tcp     # tool frame added at load time
      tip_link: link_6   # its parent, from the URDF
      tcp_offset: [0, 0, 0]   # m, in the tip-link frame
      tool_collision:
        mount_link: link_6          # substring of the flange link name
    robot2:
      name: kr240__sticktool
      base_position: [-2.5, 0, 0]   # world frame (m), from scene
      tip_link: link_6   # its parent, from the URDF
      tcp_offset: [0, 0, 0]   # m, in the tip-link frame

collision:
  min_clearance: 0.05
";

    const OBSTACLES: &str = "# Static collision obstacles
obstacles:

  # Cell floor
  - id: floor
    type: plane
    normal: [0.0, 0.0, 1.0]
    offset: -0.05
    collision_margin: 0.05

default_collision_margin: 0.05
robot_link_radius: 0.12
";

    /// A cell like jam2: two robots, one on a 1 m pedestal, both dressed with the stick tool
    /// before the Dresser wrote TCP frames (the release's tool declares its TCP).
    fn jam2_like_cell(dir: &Path) -> (PathBuf, ToolCatalogues) {
        let release = dir.join("release");
        write_catalogue(&release, true);
        let cell = dir.join("cells/jam2");
        let robot_dir = cell.join("robots/kr240__sticktool");
        std::fs::create_dir_all(&robot_dir).unwrap();
        std::fs::write(robot_dir.join("kr240__sticktool.urdf"), DRESSED_WITHOUT_TCP).unwrap();
        std::fs::write(cell.join("scene.yaml"), SCENE).unwrap();
        std::fs::create_dir_all(cell.join("agentic_planner/config")).unwrap();
        std::fs::write(cell.join(ROBOTS_YAML), ROBOTS).unwrap();
        std::fs::write(cell.join(OBSTACLES_YAML), OBSTACLES).unwrap();
        (cell, ToolCatalogues::new([dir.join("catalogue"), release]))
    }

    #[tokio::test]
    async fn the_patched_planner_model_is_written_through() {
        use crate::storage::sync::tests_support::{eventually, storage_at};
        use factoryos_protocol::storage::Scope;
        let dir = TestDir::new();
        let (cell, tools) = jam2_like_cell(dir.path());
        let (s3, storage) = storage_at(dir.path());
        let _registered = crate::storage::register(&storage);
        let robots = read_scene(&cell.join("scene.yaml"), &tools).unwrap();
        apply(&cell, &robots).unwrap();
        let keys = [format!("cells/jam2/{ROBOTS_YAML}"), format!("cells/jam2/{OBSTACLES_YAML}")];
        eventually("robots.yaml and obstacles.yaml", || keys.iter().all(|k| s3.bytes(Scope::User, k).is_some())).await;
        assert_eq!(s3.bytes(Scope::User, &keys[0]).unwrap(), std::fs::read(cell.join(ROBOTS_YAML)).unwrap());
    }

    #[test]
    fn prepare_sets_the_tcp_and_the_pedestal() {
        let dir = TestDir::new();
        let (cell, tools) = jam2_like_cell(dir.path());
        let robots = read_scene(&cell.join("scene.yaml"), &tools).unwrap();
        assert_eq!(robots.len(), 2);
        assert_eq!(robots[0].base, [2.5, 0.0, 1.0]);
        assert_eq!(robots[0].pedestal, Some(Pedestal { height_m: 1.0, diameter_m: 1.5 }));
        assert_eq!(robots[1].base, [-2.5, 0.0, 0.0]);
        assert_close(robots[0].tcp.as_ref().unwrap().offset, EXPECTED_TCP);
        assert_close(shared_tcp(&robots).unwrap(), EXPECTED_TCP);
        assert!(describe(&robots)[0].starts_with("Warning: robot1 (kr240__sticktool): the dressed URDF has no TCP frame"));

        apply(&cell, &robots).unwrap();
        let yaml: serde_yaml::Value = serde_yaml::from_str(&std::fs::read_to_string(cell.join(ROBOTS_YAML)).unwrap()).unwrap();
        let robot1 = &yaml["robots"]["arms"]["robot1"];
        let floats = |v: &serde_yaml::Value| -> Vec<f64> { v.as_sequence().unwrap().iter().map(|x| x.as_f64().unwrap()).collect() };
        assert_eq!(floats(&robot1["base_position"]), [2.5, 0.0, 1.0]);
        let tcp = floats(&robot1["tcp_offset"]);
        assert_close([tcp[0], tcp[1], tcp[2]], EXPECTED_TCP);
        assert_eq!(robot1["tip_link"].as_str(), Some("link_6"));
        assert_eq!(robot1["tool_collision"]["mount_link"].as_str(), Some("link_6"), "the rest of the block stays");
        assert_eq!(floats(&yaml["robots"]["arms"]["robot2"]["base_position"]), [-2.5, 0.0, 0.0]);
        assert_eq!(yaml["collision"]["min_clearance"].as_f64(), Some(0.05));

        let obstacles: serde_yaml::Value = serde_yaml::from_str(&std::fs::read_to_string(cell.join(OBSTACLES_YAML)).unwrap()).unwrap();
        let list = obstacles["obstacles"].as_sequence().unwrap();
        assert_eq!(list.len(), 2, "robot1's pedestal + the floor");
        let pedestal = &list[0];
        assert_eq!(pedestal["id"].as_str(), Some("robot1_pedestal"));
        assert_eq!(pedestal["type"].as_str(), Some("box"));
        // Top at 1 m - (0.12 + 0.05 + 0.01): clear of the base spheres; 1.5 m across.
        assert_eq!(floats(&pedestal["center"]), [2.5, 0.0, 0.41]);
        assert_eq!(floats(&pedestal["half_extents"]), [0.75, 0.75, 0.41]);
        assert_eq!(obstacles["robot_link_radius"].as_f64(), Some(0.12));

        // Prepare again over already patched files changes nothing.
        let before = std::fs::read_to_string(cell.join(OBSTACLES_YAML)).unwrap();
        apply(&cell, &robots).unwrap();
        assert_eq!(std::fs::read_to_string(cell.join(OBSTACLES_YAML)).unwrap(), before);
    }

    #[test]
    fn robots_without_tool_or_pedestal_keep_the_converter_values() {
        let robots = [RobotModel { id: "robot1".into(), scene_id: "r".into(), base: [1.0, 2.0, 0.0], pedestal: None, tcp: None, tool: None, robot: None }];
        let text = "robots:\n  arms:\n    robot1:\n      base_position: [1, 2, 0]   # x\n      tcp_offset: [0, 0, 0]   # y\n";
        let patched = patch_robots_yaml(text, &robots);
        assert!(patched.contains("      tcp_offset: [0, 0, 0]   # y\n"));
        assert!(patched.contains("      base_position: [1, 2, 0]   # world frame (m), from scene\n"));
        assert_eq!(patch_obstacles_yaml(OBSTACLES, &robots), OBSTACLES);
        assert_eq!(shared_tcp(&robots), None);
    }

    #[test]
    fn a_tool_working_off_the_flange_axis_gets_its_tcp_frame_as_tip_link() {
        let tcp = PlannerTcp {
            tip_link: "link_6".into(),
            offset: [-0.4906, 0.0, -0.2973],
            source: TcpSource::Frame,
            axis: [0.0, 0.0, -1.0],
            frame: Some("tcp".into()),
        };
        let robots = [RobotModel { id: "robot1".into(), scene_id: "r".into(), base: [1.0, 2.0, 0.0], pedestal: None, tcp: Some(tcp), tool: None, robot: None }];
        let text = "robots:\n  arms:\n    robot1:\n      tip_link: link_6   # x\n      tcp_offset: [0, 0, 0]   # y\n";
        let yaml: serde_yaml::Value = serde_yaml::from_str(&patch_robots_yaml(text, &robots)).unwrap();
        let robot1 = &yaml["robots"]["arms"]["robot1"];
        assert_eq!(robot1["tip_link"].as_str(), Some("tcp"));
        let offset: Vec<f64> = robot1["tcp_offset"].as_sequence().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
        assert_eq!(offset, [0.0, 0.0, 0.0]);
    }

    /// A reference cell: the scene is the Robo Fitter's `scenes/jam2/scene.yaml`, its URDF paths
    /// relative to that folder (the catalogue's `urdf-dressed-robots/`).
    #[test]
    fn a_reference_cells_scene_resolves_from_the_fitted_scenes_folder() {
        let dir = TestDir::new();
        let release = dir.path().join("release");
        write_catalogue(&release, true);
        let catalogue = dir.path().join("catalogue");
        let robot = catalogue.join("urdf-dressed-robots/kr240__sticktool");
        std::fs::create_dir_all(&robot).unwrap();
        std::fs::write(robot.join("kr240__sticktool.urdf"), DRESSED_WITHOUT_TCP).unwrap();
        std::fs::create_dir_all(catalogue.join("scenes/jam2")).unwrap();
        let scene = catalogue.join("scenes/jam2/scene.yaml");
        std::fs::write(&scene, SCENE.replace("dressed_urdf: robots/", "dressed_urdf: ../../urdf-dressed-robots/")).unwrap();
        let tools = ToolCatalogues::new([catalogue, release]);
        let robots = read_scene(&scene, &tools).unwrap();
        assert_eq!(robots.len(), 2);
        assert_close(robots[0].tcp.as_ref().unwrap().offset, EXPECTED_TCP);
        assert_eq!(robots[0].base, [2.5, 0.0, 1.0]);
    }

    /// The converter's robots.yaml with a mesh-fitted tool block (umarell 76ea13a) for robot1 and
    /// the placeholder axis (eccb1bc, or no meshes) for robot2.
    const ROBOTS_FITTED: &str = "robots:
  arms:
    robot1:
      name: kr240__WELDING_GUN
      tip_link: link_6   # its parent, from the URDF
      self_collision:
        disabled_pairs:
          - [link_5, link_6]
      # DERIVED from the tool's own meshes (the <collision> geometry on
      # Re-generated on every convert: change the meshes, not these numbers.
      tool_collision:
        mount_link: link_6
        spheres:
          - [0.1, 0, 0]
          - [0.2, 0, 0]
        # Against the floor and the fixtures: a finer cover
        static_radius_m: 0.03
        static_spheres:
          - [0.1, 0, 0]
    robot2:
      name: kr240__WELDING_GUN
      tip_link: link_6   # its parent, from the URDF
      # REVIEW: tool collision geometry is a generic placeholder — the URDF
      # has no meshes on tip_link to derive it from.
      tool_collision:
        mount_link: link_6          # substring of the flange link name
        axis:
          direction: [-1, 0, 0]
          start_m: 0.15
          end_m: 0.9
          step_m: 0.15

collision:
  min_clearance: 0.05
  warning_distance: 0.20

planning:
  default_planner: RRTConnect
";

    const SIDECAR: &str = "tool_collision:
  mount_link: link_6
  plates:
    - {center: [-0.3, 0, 0.1], rpy: [0, 1.5708, 0], size: [0.3, 0.45], grid: [5, 5]}
    - center: [-0.5, 0, 0.1]
      size: [0.2, 0.2]
collision:
  tool_contact_radius: 0.05
tool_link_radius: 0.02
";

    fn sidecar(text: &str) -> ToolSidecar {
        parse_sidecar("WELDING_GUN", Path::new("/cat/urdf-tool-collection/WELDING_GUN/planner.yaml"), text).unwrap()
    }

    fn robot(id: &str, tool: Option<ToolSidecar>) -> RobotModel {
        RobotModel { id: id.into(), scene_id: id.into(), base: [0.0; 3], pedestal: None, tcp: None, tool, robot: None }
    }

    fn test_tcp() -> PlannerTcp {
        PlannerTcp { tip_link: "link_6".into(), offset: [-0.5, 0.0, 0.0], source: TcpSource::Frame, axis: [-1.0, 0.0, 0.0], frame: None }
    }

    #[test]
    fn a_tool_sidecar_replaces_the_converters_tool_model() {
        let gun = sidecar(SIDECAR);
        assert_eq!(gun.tool_contact_radius, Some(0.05));
        assert_eq!(gun.tool_link_radius, Some(0.02));
        let robots = [robot("robot1", Some(gun.clone())), robot("robot2", Some(gun))];
        let patched = patch_tool_models(ROBOTS_FITTED, &robots);
        let yaml: serde_yaml::Value = serde_yaml::from_str(&patched).unwrap();
        for id in ["robot1", "robot2"] {
            let tc = &yaml["robots"]["arms"][id]["tool_collision"];
            assert_eq!(tc["mount_link"].as_str(), Some("link_6"), "{patched}");
            let plates = tc["plates"].as_sequence().unwrap();
            assert_eq!(plates.len(), 2);
            assert_eq!(plates[0]["grid"].as_sequence().unwrap().len(), 2);
            assert_eq!(plates[1]["center"][0].as_f64(), Some(-0.5));
            assert!(tc.get("spheres").is_none() && tc.get("static_spheres").is_none() && tc.get("axis").is_none(), "{patched}");
            assert_eq!(yaml["robots"]["arms"][id]["tip_link"].as_str(), Some("link_6"));
        }
        assert_eq!(yaml["robots"]["arms"]["robot1"]["self_collision"]["disabled_pairs"].as_sequence().unwrap().len(), 1);
        assert!(!patched.contains("DERIVED from the tool") && !patched.contains("REVIEW: tool collision"), "{patched}");
        assert_eq!(yaml["collision"]["tool_contact_radius"].as_f64(), Some(0.05));
        assert_eq!(yaml["collision"]["min_clearance"].as_f64(), Some(0.05));
        assert_eq!(yaml["planning"]["default_planner"].as_str(), Some("RRTConnect"));
        assert_eq!(patch_tool_models(&patched, &robots), patched, "idempotent");

        // Only robot1's tool has one: robot2 keeps the converter's block.
        let one = patch_tool_models(ROBOTS_FITTED, &[robots[0].clone(), robot("robot2", None)]);
        let yaml: serde_yaml::Value = serde_yaml::from_str(&one).unwrap();
        assert_eq!(yaml["robots"]["arms"]["robot2"]["tool_collision"]["axis"]["end_m"].as_f64(), Some(0.9));
        assert!(yaml["robots"]["arms"]["robot1"]["tool_collision"]["plates"].is_sequence());
        // No sidecar at all: unchanged.
        assert_eq!(patch_tool_models(ROBOTS_FITTED, &[robot("robot1", None)]), ROBOTS_FITTED);
    }

    #[test]
    fn a_sidecar_with_only_radii_keeps_the_fitted_spheres() {
        let radii = sidecar("collision: {tool_contact_radius: 0.04}\ntool_link_radius: 0.03\n");
        let robots = [robot("robot1", Some(radii))];
        let patched = patch_tool_models(ROBOTS, &robots);
        let yaml: serde_yaml::Value = serde_yaml::from_str(&patched).unwrap();
        assert_eq!(yaml["robots"]["arms"]["robot1"]["tool_collision"]["mount_link"].as_str(), Some("link_6"));
        assert_eq!(yaml["collision"]["tool_contact_radius"].as_f64(), Some(0.04));
        // No collision block yet: one is added.
        let bare = patch_tool_models("robots:\n  arms:\n    robot1:\n      tip_link: link_6\n", &robots);
        assert!(bare.ends_with("\ncollision:\n  tool_contact_radius: 0.04   # the largest of the cell's tools' planner.yaml (Prepare Cell)\n"), "{bare}");

        // obstacles.yaml: the sidecar's tool_link_radius replaces the converter's.
        let obstacles = format!("{OBSTACLES}tool_link_radius: 0.08\ntool_collision_margin: 0.05\n");
        let gun = RobotModel { tcp: Some(test_tcp()), ..robots[0].clone() };
        let patched = patch_tool_link_radius(&obstacles, std::slice::from_ref(&gun));
        let yaml: serde_yaml::Value = serde_yaml::from_str(&patched).unwrap();
        assert_eq!(yaml["tool_link_radius"].as_f64(), Some(0.03));
        assert_eq!(yaml["tool_collision_margin"].as_f64(), Some(0.05));
        assert_eq!(patch_tool_link_radius(&patched, std::slice::from_ref(&gun)), patched, "idempotent");
        // A tooled robot without a sidecar value: the converter's measured value stays if larger.
        let other = RobotModel { id: "robot2".into(), tcp: Some(test_tcp()), tool: None, ..robots[0].clone() };
        let mixed: serde_yaml::Value = serde_yaml::from_str(&patch_tool_link_radius(&obstacles, &[gun, other])).unwrap();
        assert_eq!(mixed["tool_link_radius"].as_f64(), Some(0.08));
        assert_eq!(patch_tool_link_radius(OBSTACLES, &[robot("robot1", None)]), OBSTACLES);
    }

    #[test]
    fn bad_sidecars_are_refused() {
        let bad = |text: &str| format!("{:#}", parse_sidecar("T", Path::new("p"), text).unwrap_err());
        assert!(bad("tool_collision: {plates: []}\n").contains("mount_link"));
        assert!(bad("tool_collision: {mount_link: link_6}\n").contains("plates, spheres or axis"));
        assert!(bad("tool_link_radus: 0.02\n").contains("unknown key"));
        assert!(bad("collision: {min_clearance: 0.1}\n").contains("collision.min_clearance"));
        assert!(bad("tool_link_radius: -1\n").contains(">= 0"));
        assert_eq!(parse_sidecar("T", Path::new("p"), "").unwrap().tool_collision, None);
    }

    /// Prepare Cell finds the sidecar of the dressed robot's tool (`<robot>__<tool>`) in the
    /// catalogue and writes it into the converted files.
    #[test]
    fn prepare_reads_the_tools_sidecar_from_the_catalogue() {
        let dir = TestDir::new();
        let (cell, tools) = jam2_like_cell(dir.path());
        let tool_dir = dir.path().join("release/urdf-tool-collection/sticktool");
        std::fs::write(tool_dir.join(TOOL_SIDECAR), SIDECAR).unwrap();
        std::fs::write(cell.join(OBSTACLES_YAML), format!("{OBSTACLES}tool_link_radius: 0.08\n")).unwrap();
        let robots = read_scene(&cell.join("scene.yaml"), &tools).unwrap();
        let tool = robots[0].tool.as_ref().unwrap();
        assert_eq!((tool.tool.as_str(), tool.path.clone()), ("sticktool", tool_dir.join(TOOL_SIDECAR)));
        assert!(describe(&robots).iter().any(|l| l.starts_with("robot2 (kr240__sticktool_2): tool sticktool's planner model from")));
        apply(&cell, &robots).unwrap();
        let yaml: serde_yaml::Value = serde_yaml::from_str(&std::fs::read_to_string(cell.join(ROBOTS_YAML)).unwrap()).unwrap();
        assert!(yaml["robots"]["arms"]["robot1"]["tool_collision"]["plates"].is_sequence());
        assert!(yaml["robots"]["arms"]["robot2"]["tool_collision"]["plates"].is_sequence(), "added: the converter wrote none");
        assert_eq!(yaml["collision"]["tool_contact_radius"].as_f64(), Some(0.05));
        let obstacles: serde_yaml::Value = serde_yaml::from_str(&std::fs::read_to_string(cell.join(OBSTACLES_YAML)).unwrap()).unwrap();
        assert_eq!(obstacles["tool_link_radius"].as_f64(), Some(0.02));
        // A broken sidecar fails Prepare Cell, naming the file.
        std::fs::write(tool_dir.join(TOOL_SIDECAR), "tool_collision: [1]\n").unwrap();
        let err = format!("{:#}", read_scene(&cell.join("scene.yaml"), &tools).unwrap_err());
        assert!(err.contains("planner.yaml") && err.contains("not a mapping"), "{err}");
    }

    /// Prepare Cell finds the robot model's sidecar (`urdf-robot-collection/<model>/planner.yaml`,
    /// `<model>` of `<model>__<tool>`) in the catalogue and writes its acceleration limits over the
    /// converter's `# REVIEW` placeholder, flagged as an estimate when the sidecar says so.
    #[test]
    fn prepare_reads_the_robot_models_sidecar_from_the_catalogue() {
        let dir = TestDir::new();
        let (cell, tools) = jam2_like_cell(dir.path());
        let with_acc = ROBOTS.replace(
            "      tcp_offset: [0, 0, 0]   # m, in the tip-link frame\n",
            "      tcp_offset: [0, 0, 0]   # m, in the tip-link frame\n      joint_acceleration_limits: [3, 3, 3, 6, 6, 8]   # REVIEW (rad/s^2)\n",
        );
        std::fs::write(cell.join(ROBOTS_YAML), &with_acc).unwrap();
        // No sidecar: the converter's placeholder stays.
        let robots = read_scene(&cell.join("scene.yaml"), &tools).unwrap();
        assert!(robots.iter().all(|r| r.robot.is_none()));
        assert_eq!(patch_robot_model_limits(&with_acc, &robots), with_acc);

        let model_dir = dir.path().join("release/urdf-robot-collection/kr240");
        std::fs::create_dir_all(&model_dir).unwrap();
        std::fs::write(
            model_dir.join(ROBOT_SIDECAR),
            "joint_acceleration_limits: [4, 4.5, 4, 8, 8, 12]\nsource: \"ramp time to the datasheet speeds\"\nestimate: true\n",
        )
        .unwrap();
        let robots = read_scene(&cell.join("scene.yaml"), &tools).unwrap();
        let model = robots[0].robot.as_ref().unwrap();
        assert_eq!((model.model.as_str(), model.path.clone(), model.estimate), ("kr240", model_dir.join(ROBOT_SIDECAR), true));
        assert_eq!(model.joint_acceleration_limits.as_deref(), Some(&[4.0, 4.5, 4.0, 8.0, 8.0, 12.0][..]));
        assert!(
            describe(&robots).iter().any(|l| l.starts_with("Warning: robot1 (kr240__sticktool): joint_acceleration_limits [4, 4.5, 4, 8, 8, 12] rad/s^2 from robot model kr240's planner model")
                && l.contains("ESTIMATE")),
            "{:?}",
            describe(&robots)
        );
        apply(&cell, &robots).unwrap();
        let text = std::fs::read_to_string(cell.join(ROBOTS_YAML)).unwrap();
        assert!(text.contains("      joint_acceleration_limits: [4, 4.5, 4, 8, 8, 12]   # ESTIMATE, robot model kr240's planner.yaml (Prepare Cell): ramp time to the datasheet speeds\n"), "{text}");
        assert!(!text.contains("REVIEW (rad/s^2)"), "{text}");
        let yaml: serde_yaml::Value = serde_yaml::from_str(&text).unwrap();
        assert_eq!(yaml["robots"]["arms"]["robot2"]["joint_acceleration_limits"][5].as_f64(), Some(12.0));
        assert_eq!(patch_robot_model_limits(&text, &robots), text, "idempotent");
        // Another joint count than the converter's: not applied.
        let seven = RobotSidecar { joint_acceleration_limits: Some(vec![1.0; 7]), ..model.clone() };
        let r7 = [RobotModel { robot: Some(seven), ..robots[0].clone() }];
        assert_eq!(patch_robot_model_limits(&with_acc, &r7), with_acc);
        // A broken sidecar fails Prepare Cell, naming the file.
        std::fs::write(model_dir.join(ROBOT_SIDECAR), "joint_acceleration_limits: [3, -1]\n").unwrap();
        let err = format!("{:#}", read_scene(&cell.join("scene.yaml"), &tools).unwrap_err());
        assert!(err.contains("planner.yaml") && err.contains("> 0"), "{err}");
    }

    #[test]
    fn bad_robot_sidecars_are_refused() {
        let bad = |text: &str| format!("{:#}", parse_robot_sidecar("M", Path::new("p"), text).unwrap_err());
        assert!(bad("joint_acceleration_limits: 3\n").contains("not a list"));
        assert!(bad("joint_acceleration_limits: []\n").contains("empty"));
        assert!(bad("joint_acceleration_limits: [3, x]\n").contains("non-number"));
        assert!(bad("joint_accel_limits: [3]\n").contains("unknown key"));
        assert!(bad("estimate: maybe\n").contains("true/false"));
        assert!(bad("joint_velocity_limits: [1, 0]\n").contains("> 0"));
        assert!(bad("joint_velocity_limits: [1, .inf]\n").contains("finite"));
        assert!(bad("joint_limits_lower: [-1, -1]\n").contains("go together"));
        assert!(bad("joint_limits_lower: [-1, 1]\njoint_limits_upper: [1, 0.5]\n").contains("joint 2"));
        assert!(bad("joint_limits_lower: [-1, -1]\njoint_limits_upper: [1, 1, 1]\n").contains("3"));
        assert!(bad("joint_acceleration_limits: [3, 3]\njoint_velocity_limits: [1, 1, 1]\n").contains("one per movable joint"));
        assert!(bad("velocity_source: \"a\\nb\"\n").contains("one line"));
        let ok = parse_robot_sidecar("M", Path::new("p"), "joint_limits_lower: [-1, -2]\njoint_limits_upper: [1, -0.5]\n").unwrap();
        assert_eq!(ok.keys().len(), 2);
        let empty = parse_robot_sidecar("M", Path::new("p"), "").unwrap();
        assert_eq!((empty.joint_acceleration_limits, empty.source, empty.estimate), (None, None, false));
    }

    /// The release catalogue's KR 240 R2900 sidecar parses and gives the six axes: measured
    /// accelerations, the controllers' axis speeds and software limits.
    #[test]
    fn the_shipped_kr240r2900_sidecar_parses() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/catalogue/urdf-robot-collection/kr240r2900").join(ROBOT_SIDECAR);
        let model = read_robot_sidecar("kr240r2900", &path).unwrap();
        for (key, values, _, source) in model.keys() {
            assert_eq!(values.len(), 6, "{key}");
            assert!(source.is_some(), "{key} says where it comes from");
        }
        assert_eq!(model.keys().len(), 4);
        assert!(!model.estimate, "measured / controller data");
        // Below the URDF's 156 deg/s (A1-A3): the datasheet's 105 / 101 / 107 deg/s.
        let v = model.joint_velocity_limits.unwrap();
        assert!(v[..3].iter().all(|x| (1.7..1.9).contains(x)), "{v:?}");
        // A2's software limits are -140 / -5 deg (the URDF: -155 / +35).
        let (lo, hi) = (model.joint_limits_lower.unwrap(), model.joint_limits_upper.unwrap());
        assert!((lo[1].to_degrees() + 140.0).abs() < 1e-3 && (hi[1].to_degrees() + 5.0).abs() < 1e-3, "{lo:?} {hi:?}");
    }

    /// The release catalogue's sticktool sidecar is the one the catalogue's dressed robot gets and
    /// gives only the weld contact radius (see its comments for the geometry): the converter's
    /// mesh-fitted tool spheres, tool_link_radius and tool_orientation stay.
    #[test]
    fn the_shipped_sticktool_sidecar_sets_only_the_contact_radius() {
        let catalogue = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/catalogue");
        let tools = ToolCatalogues::new([catalogue.clone()]);
        let (name, path) = tools.tool_sidecar("kr240r2900__sticktool").expect("the catalogue sticktool has a planner.yaml");
        assert_eq!((name.as_str(), path.clone()), ("sticktool", catalogue.join("urdf-tool-collection/sticktool/planner.yaml")));
        let stick = read_sidecar(&name, &path).unwrap();
        assert_eq!(stick.tool_contact_radius, Some(0.025));
        assert_eq!((stick.tool_collision.as_ref(), stick.tool_link_radius, stick.tool_orientation.len()), (None, None, 0));
        let robots = [robot("robot1", Some(stick.clone())), robot("robot2", Some(stick))];
        let patched = patch_tool_models(ROBOTS_FITTED, &robots);
        let yaml: serde_yaml::Value = serde_yaml::from_str(&patched).unwrap();
        assert_eq!(yaml["collision"]["tool_contact_radius"].as_f64(), Some(0.025), "{patched}");
        let fitted: serde_yaml::Value = serde_yaml::from_str(ROBOTS_FITTED).unwrap();
        assert_eq!(yaml["robots"], fitted["robots"], "the converter's tool model stays");
        assert_eq!(patch_tool_orientation(&patched, &robots), patched);
        let obstacles = "obstacles: []\ntool_link_radius: 0.05\n";
        assert_eq!(patch_tool_link_radius(obstacles, &robots), obstacles);
    }

    /// A sidecar's joint speeds and ranges replace the converter's URDF copies in robots.yaml, each
    /// with its own source; the other robots.yaml keys stay.
    #[test]
    fn a_robot_sidecars_speeds_and_ranges_replace_the_converters() {
        let dir = TestDir::new();
        let (cell, tools) = jam2_like_cell(dir.path());
        let converted = ROBOTS.replace(
            "      tcp_offset: [0, 0, 0]   # m, in the tip-link frame\n",
            "      tcp_offset: [0, 0, 0]   # m, in the tip-link frame\n      joint_limits_lower: [-3.228859, -2.70526, -2.268928, -6.108652, -2.268928, -6.108652]\n      joint_limits_upper: [3.228859, 0.6108652, 2.687807, 6.108652, 2.268928, 6.108652]\n      joint_velocity_limits:     [2.722714, 2.722714, 2.722714, 5.759586, 5.759586, 10.73377]\n      joint_acceleration_limits: [3, 3, 3, 6, 6, 8]   # REVIEW (rad/s^2)\n",
        );
        std::fs::write(cell.join(ROBOTS_YAML), &converted).unwrap();
        let model_dir = dir.path().join("release/urdf-robot-collection/kr240");
        std::fs::create_dir_all(&model_dir).unwrap();
        std::fs::write(
            model_dir.join(ROBOT_SIDECAR),
            "joint_acceleration_limits: [7.5, 7, 11.5, 17, 13, 20]\nsource: \"drive traces\"\nestimate: false\n\
             joint_velocity_limits: [1.8325, 1.7621, 1.8675, 2.3692, 2.2475, 3.593]\nvelocity_source: \"datasheet\"\n\
             joint_limits_lower: [-3.228859, -2.44346, -2.094395, -6.108652, -2.138028, -6.108652]\n\
             joint_limits_upper: [3.228859, -0.087267, 2.70526, 6.108652, 2.138028, 6.108652]\nlimits_source: \"machine.dat\"\n",
        )
        .unwrap();
        let robots = read_scene(&cell.join("scene.yaml"), &tools).unwrap();
        let lines = describe(&robots);
        assert!(lines.iter().any(|l| l.starts_with("robot1 (kr240__sticktool): joint_velocity_limits [1.8325, 1.7621, 1.8675, 2.3692, 2.2475, 3.593] rad/s from robot model kr240's planner model") && l.ends_with("(datasheet)")), "{lines:?}");
        assert!(lines.iter().any(|l| l.contains("joint_limits_upper [3.228859, -0.087267, 2.70526, 6.108652, 2.138028, 6.108652] rad") && l.ends_with("(machine.dat)")), "{lines:?}");
        assert!(!lines.iter().any(|l| l.contains("ESTIMATE")), "{lines:?}");
        apply(&cell, &robots).unwrap();
        let text = std::fs::read_to_string(cell.join(ROBOTS_YAML)).unwrap();
        assert!(text.contains("      joint_velocity_limits: [1.8325, 1.7621, 1.8675, 2.3692, 2.2475, 3.593]   # rad/s, robot model kr240's planner.yaml (Prepare Cell): datasheet\n"), "{text}");
        assert!(text.contains("      joint_limits_lower: [-3.228859, -2.44346, -2.094395, -6.108652, -2.138028, -6.108652]   # rad, robot model kr240's planner.yaml (Prepare Cell): machine.dat\n"), "{text}");
        assert!(text.contains("      joint_acceleration_limits: [7.5, 7, 11.5, 17, 13, 20]   # rad/s^2, robot model kr240's planner.yaml (Prepare Cell): drive traces\n"), "{text}");
        let yaml: serde_yaml::Value = serde_yaml::from_str(&text).unwrap();
        let arm = &yaml["robots"]["arms"]["robot1"];
        assert_eq!(arm["joint_limits_upper"][1].as_f64(), Some(-0.087267));
        assert_eq!(arm["joint_velocity_limits"][5].as_f64(), Some(3.593));
        assert_eq!(arm["name"].as_str(), Some("kr240__sticktool"));
        assert_eq!(patch_robot_model_limits(&text, &robots), text, "idempotent");
        assert_eq!(yaml["robots"]["arms"]["robot2"]["joint_limits_lower"][2].as_f64(), Some(-2.094395));
    }

    #[test]
    fn a_reference_cells_srdfs_go_into_its_cell_yaml() {
        let cell = Path::new("/cells/jam2");
        let urdf = PathBuf::from("/assets/urdf-dressed-robots/kr/kr.urdf");
        let text = "name: jam2\ncell_id: jam2\nrobots:\n  - id: robot1\n    scene_id: kr\n    namespace: arms\n    model: kr\n    \
                    urdf: /assets/urdf-dressed-robots/kr/kr.urdf\n    # srdf: NONE FOUND beside the URDF — self-collision checking\n    \
                    #       will retain every pair. Add one to silence the noise.\n  - id: robot2\n    scene_id: other\n    \
                    urdf: robots/other/other.urdf\n    srdf: robots/other/other.srdf\n";
        let srdfs = [(urdf, "agentic_planner/srdf/kr.srdf".to_owned())];
        let patched = patch_cell_yaml_srdfs(text, cell, &srdfs);
        assert!(patched.contains("    urdf: /assets/urdf-dressed-robots/kr/kr.urdf\n    srdf: agentic_planner/srdf/kr.srdf\n  - id: robot2\n"), "{patched}");
        assert!(!patched.contains("NONE FOUND") && !patched.contains("will retain"));
        assert!(patched.contains("    srdf: robots/other/other.srdf\n"), "a legacy robot's own SRDF stays");
        assert_eq!(patch_cell_yaml_srdfs(&patched, cell, &srdfs), patched, "idempotent");
        // No comment (an older converter): added after the entry.
        let bare = "robots:\n  - id: robot1\n    urdf: /assets/urdf-dressed-robots/kr/kr.urdf\n";
        assert_eq!(patch_cell_yaml_srdfs(bare, cell, &srdfs), format!("{bare}    srdf: agentic_planner/srdf/kr.srdf\n"));
    }

    #[test]
    fn a_sidecars_tool_orientation_replaces_the_converters_keys() {
        let gun = sidecar("tool_orientation: {max_tilt_deg: 8, axis_tolerance_deg: 9, spin_free: true}\n");
        assert_eq!(gun.tool_orientation.len(), 3);
        let yaml = "robots:\n  arms:\n    robot1:\n      tip_link: tcp\n      tool_orientation:\n        spin_free: true   # REVIEW\n        max_tilt_deg: 4.5   # REVIEW\n      self_collision:\n        required: true\n    robot2:\n      tip_link: tcp\n    robot3:\n      tip_link: tcp\n      tool_orientation:\n        max_tilt_deg: 4.5\ncollision:\n  min_clearance: 0.05\n";
        let robots = [robot("robot1", Some(gun.clone())), robot("robot2", Some(gun.clone())), robot("robot3", None)];
        let patched = patch_tool_orientation(yaml, &robots);
        let doc: serde_yaml::Value = serde_yaml::from_str(&patched).unwrap();
        let arms = &doc["robots"]["arms"];
        for id in ["robot1", "robot2"] {
            let o = &arms[id]["tool_orientation"];
            assert_eq!((o["max_tilt_deg"].as_f64(), o["axis_tolerance_deg"].as_f64(), o["spin_free"].as_bool()), (Some(8.0), Some(9.0), Some(true)), "{id}: {patched}");
        }
        assert_eq!(arms["robot1"]["self_collision"]["required"].as_bool(), Some(true), "the next key stays");
        assert_eq!(arms["robot3"]["tool_orientation"]["max_tilt_deg"].as_f64(), Some(4.5), "no sidecar: the converter's");
        assert!(patched.contains("        max_tilt_deg: 8   # tool WELDING_GUN's planner.yaml (Prepare Cell)\n"), "{patched}");
        assert_eq!(patch_tool_orientation(&patched, &robots), patched, "idempotent");
        for bad in ["tool_orientation: {max_tilt_deg: -1}\n", "tool_orientation: {spin_free: 3}\n", "tool_orientation: {tilt: 5}\n"] {
            assert!(parse_sidecar("G", Path::new("/g"), bad).is_err(), "{bad}");
        }
    }
}
