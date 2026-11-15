//! Editing a product's operations one at a time: what the Product & Task pane does by hand, as
//! requests the backend applies to the stored product (`/api/v1/product/{name}/welds/*`, the chat
//! agent's MCP tools) and the generators both use.
//!
//! "Welds" are the point operations of a task (spot welds, screws, holes, …); `generate` also
//! writes a face's outline as a path for path processes. Positions and normals are in the
//! product's frame (metres, z up), as in [`crate::product`].

use crate::product::{Operation, Params, Product, Target, TargetKind, Task};

/// The faces of a box product: key, label, outward normal.
pub const FACES: [(&str, &str, [f32; 3]); 6] = [
    ("+Z", "+Z (top)", [0.0, 0.0, 1.0]),
    ("-Z", "−Z (bottom)", [0.0, 0.0, -1.0]),
    ("+X", "+X", [1.0, 0.0, 0.0]),
    ("-X", "−X", [-1.0, 0.0, 0.0]),
    ("+Y", "+Y", [0.0, 1.0, 0.0]),
    ("-Y", "−Y", [0.0, -1.0, 0.0]),
];

/// A face by its key (`+Z`, `-x`, `top`, `bottom`, `−Y`, …), as an index into [`FACES`].
pub fn face_index(name: &str) -> Option<usize> {
    let key = name.trim().replace('−', "-").to_ascii_uppercase();
    match key.as_str() {
        "TOP" => return Some(0),
        "BOTTOM" => return Some(1),
        _ => {}
    }
    let key = if key.len() == 1 { format!("+{key}") } else { key };
    FACES.iter().position(|(k, _, _)| *k == key)
}

/// A grid (or, for a path process, an outline) on a face of a box product.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FacePattern {
    /// An index into [`FACES`].
    pub face: usize,
    #[serde(default = "one")]
    pub rows: u32,
    #[serde(default = "one")]
    pub columns: u32,
    /// Distance from the face's edges, metres.
    #[serde(default)]
    pub margin: f32,
    #[serde(default)]
    pub group: String,
}

fn one() -> u32 {
    1
}

/// The id prefix of a task's operations: the process's first letter (`W` for welding spots).
pub fn id_prefix(task: &Task) -> char {
    match task.process.as_str() {
        "spotWeld" | "seamWeld" => 'W',
        other => other.chars().next().map_or('O', |c| c.to_ascii_uppercase()),
    }
}

/// The next free `<prefix><nn>` id in `task`, `offset` ids after the highest one in use.
pub fn next_id(task: &Task, offset: usize) -> String {
    let prefix = id_prefix(task);
    let highest = task
        .operations
        .iter()
        .filter_map(|op| op.id.strip_prefix(prefix)?.parse::<usize>().ok())
        .max()
        .unwrap_or(0);
    format!("{prefix}{:02}", highest + 1 + offset)
}

/// A `(lo, hi)` range along one axis.
type Range = (f32, f32);

/// A face of a box of `size` (standing on the origin): the ranges of its two in-plane axes, and
/// the point for a (u, v) on them.
pub fn face_frame(size: [f32; 3], face: usize) -> (Range, Range, impl Fn(f32, f32) -> [f32; 3]) {
    let [sx, sy, sz] = size;
    let (half_x, half_y) = ((-sx / 2.0, sx / 2.0), (-sy / 2.0, sy / 2.0));
    let (u_range, v_range) = match face {
        0 | 1 => (half_x, half_y),
        2 | 3 => (half_y, (0.0, sz)),
        _ => (half_x, (0.0, sz)),
    };
    let point = move |u: f32, v: f32| match face {
        0 => [u, v, sz],
        1 => [u, v, 0.0],
        2 => [sx / 2.0, u, v],
        3 => [-sx / 2.0, u, v],
        4 => [u, sy / 2.0, v],
        _ => [u, -sy / 2.0, v],
    };
    (u_range, v_range, point)
}

/// `(lo, hi)` moved `margin` in from both ends (to its middle if too short).
fn inset((lo, hi): Range, margin: f32) -> Range {
    if hi - lo > 2.0 * margin { (lo + margin, hi - margin) } else { ((lo + hi) / 2.0, (lo + hi) / 2.0) }
}

/// Millimetre precision, as the pane shows them.
pub fn rounded(p: [f32; 3]) -> [f32; 3] {
    p.map(|c| (c * 1000.0).round() / 1000.0)
}

/// `rows × columns` point operations evenly spread over a face, `margin` in from its edges.
pub fn grid_points(size: [f32; 3], g: &FacePattern, task: &Task) -> Vec<Operation> {
    let face = g.face.min(FACES.len() - 1);
    let (u_range, v_range, point) = face_frame(size, face);
    let ((u0, u1), (v0, v1)) = (inset(u_range, g.margin), inset(v_range, g.margin));
    let spread = |lo: f32, hi: f32, count: u32, i: u32| if count <= 1 { (lo + hi) / 2.0 } else { lo + (hi - lo) * i as f32 / (count - 1) as f32 };
    let mut operations = Vec::new();
    for row in 0..g.rows {
        for column in 0..g.columns {
            let position = rounded(point(spread(u0, u1, g.columns, column), spread(v0, v1, g.rows, row)));
            operations.push(Operation {
                id: next_id(task, operations.len()),
                target: Target::Point { position, normal: FACES[face].2 },
                group: g.group.clone(),
                params: Params::new(),
            });
        }
    }
    operations
}

/// A closed path around a face, `margin` in from its edges.
pub fn face_outline(size: [f32; 3], g: &FacePattern, task: &Task) -> Operation {
    let face = g.face.min(FACES.len() - 1);
    let (u_range, v_range, point) = face_frame(size, face);
    let ((u0, u1), (v0, v1)) = (inset(u_range, g.margin), inset(v_range, g.margin));
    let points = [(u0, v0), (u1, v0), (u1, v1), (u0, v1), (u0, v0)].map(|(u, v)| rounded(point(u, v))).to_vec();
    Operation {
        id: next_id(task, 0),
        target: Target::Path { points, normal: FACES[face].2 },
        group: g.group.clone(),
        params: Params::new(),
    }
}

// --- requests ---

/// One point to add: where, and optionally its normal, id, group and parameter overrides.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct NewWeld {
    pub position: [f32; 3],
    /// The outward surface normal. Left out: the surface's at the (snapped) point.
    #[serde(default)]
    pub normal: Option<[f32; 3]>,
    /// Left out: the next free id (`W07`, …).
    #[serde(default)]
    pub id: Option<String>,
    #[serde(default)]
    pub group: Option<String>,
    #[serde(default)]
    pub params: Params,
}

/// `POST /api/v1/product/{name}/welds/add`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AddWelds {
    /// The task's name; left out: the product's only (or first spot welding) task. A task that
    /// does not exist is created with `process` (default `spotWeld`) and `tool`.
    #[serde(default)]
    pub task: Option<String>,
    #[serde(default)]
    pub process: Option<String>,
    #[serde(default)]
    pub tool: Option<String>,
    pub welds: Vec<NewWeld>,
    /// Move each point onto the part's surface first (its normal then the surface's). Default true.
    #[serde(default = "yes")]
    pub snap: bool,
}

fn yes() -> bool {
    true
}

/// `POST /api/v1/product/{name}/welds/remove`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RemoveWelds {
    /// Left out: every task.
    #[serde(default)]
    pub task: Option<String>,
    /// Operation ids; `["*"]` every operation of the task(s).
    pub ids: Vec<String>,
}

/// `POST /api/v1/product/{name}/welds/move`: `position` or `delta` (or neither, to only re-orient).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MoveWeld {
    #[serde(default)]
    pub task: Option<String>,
    pub id: String,
    #[serde(default)]
    pub position: Option<[f32; 3]>,
    #[serde(default)]
    pub delta: Option<[f32; 3]>,
    #[serde(default)]
    pub normal: Option<[f32; 3]>,
    /// Onto the surface after moving (normal from the surface unless given). Default true.
    #[serde(default = "yes")]
    pub snap: bool,
}

/// `POST /api/v1/product/{name}/welds/regroup`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RegroupWelds {
    #[serde(default)]
    pub task: Option<String>,
    pub ids: Vec<String>,
    pub group: String,
    /// The operations' `priority` parameter (lower first), if given.
    #[serde(default)]
    pub priority: Option<f64>,
}

/// `POST /api/v1/product/{name}/welds/generate`: a grid of points (point process) or the outline
/// (path process) on a face of a box product.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct GenerateWelds {
    #[serde(default)]
    pub task: Option<String>,
    #[serde(default)]
    pub process: Option<String>,
    #[serde(default)]
    pub tool: Option<String>,
    /// `+Z`/`top`, `-Z`/`bottom`, `+X`, `-X`, `+Y`, `-Y`.
    pub face: String,
    #[serde(default = "one")]
    pub rows: u32,
    #[serde(default = "one")]
    pub columns: u32,
    /// Instead of rows/columns: the spacing between points along both axes, metres.
    #[serde(default)]
    pub pitch: Option<f32>,
    #[serde(default)]
    pub margin: f32,
    #[serde(default)]
    pub group: String,
    /// Replace the task's operations instead of adding to them.
    #[serde(default)]
    pub replace: bool,
}

/// `POST /api/v1/product/{name}/snap`: points to put onto the part's surface.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SnapRequest {
    pub points: Vec<[f32; 3]>,
}

/// One snapped point.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Snapped {
    pub input: [f32; 3],
    pub position: [f32; 3],
    /// The outward surface normal there.
    pub normal: [f32; 3],
    /// How far the point moved, metres.
    pub distance: f32,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SnapResult {
    pub points: Vec<Snapped>,
    /// `box`, `mesh` or `urdf`: what was snapped onto.
    pub surface: String,
    pub triangles: usize,
}

/// `POST /api/v1/product/{name}/welds/check`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CheckWelds {
    #[serde(default)]
    pub task: Option<String>,
    /// Points closer than this are flagged, metres (default 0.02).
    #[serde(default = "min_pitch")]
    pub min_pitch: f32,
    /// Points closer than this to a box face's edge are flagged, metres (default 0.01).
    #[serde(default = "edge_margin")]
    pub edge_margin: f32,
    /// Points farther than this from the surface are flagged, metres (default 0.003).
    #[serde(default = "surface_tolerance")]
    pub surface_tolerance: f32,
}

impl Default for CheckWelds {
    fn default() -> Self {
        Self { task: None, min_pitch: min_pitch(), edge_margin: edge_margin(), surface_tolerance: surface_tolerance() }
    }
}

fn min_pitch() -> f32 {
    0.02
}
fn edge_margin() -> f32 {
    0.01
}
fn surface_tolerance() -> f32 {
    0.003
}

/// One finding of a check.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct WeldIssue {
    pub task: String,
    pub id: String,
    /// `off_surface`, `inside_part`, `normal_mismatch`, `too_close`, `near_edge`, `zero_normal`,
    /// `duplicate_id`.
    pub kind: String,
    pub message: String,
}

/// The answer of every weld edit: the saved product, what changed, and the check's findings.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct WeldEdit {
    pub product: Product,
    /// `product.json` on the backend host.
    pub path: String,
    /// The ids added, moved, regrouped or removed.
    pub changed: Vec<String>,
    pub issues: Vec<WeldIssue>,
}

/// The answer of a check.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct WeldCheck {
    pub welds: usize,
    pub issues: Vec<WeldIssue>,
}

// --- applying them ---

/// The task a request names, or the product's only / first spot welding / first one.
pub fn find_task<'a>(product: &'a mut Product, name: Option<&str>) -> Result<&'a mut Task, String> {
    let index = match name.map(str::trim).filter(|n| !n.is_empty()) {
        Some(name) => product.tasks.iter().position(|t| t.name == name).ok_or_else(|| {
            let names: Vec<&str> = product.tasks.iter().map(|t| t.name.as_str()).collect();
            format!("product {} has no task {name:?} (tasks: {names:?})", product.name)
        })?,
        None => product
            .tasks
            .iter()
            .position(|t| t.process == "spotWeld")
            .or((!product.tasks.is_empty()).then_some(0))
            .ok_or_else(|| format!("product {} has no task yet: name one (task, process, tool) to create it", product.name))?,
    };
    Ok(&mut product.tasks[index])
}

/// The task a request names, created (with `process`, default `spotWeld`, its default parameters
/// and `tool`) if the product has none of that name.
pub fn task_or_new<'a>(product: &'a mut Product, name: Option<&str>, process: Option<&str>, tool: Option<&str>) -> Result<&'a mut Task, String> {
    let wanted = name.map(str::trim).filter(|n| !n.is_empty());
    let exists = match wanted {
        Some(n) => product.tasks.iter().any(|t| t.name == n),
        None => !product.tasks.is_empty(),
    };
    if !exists {
        let process = process.unwrap_or("spotWeld");
        let params = crate::product::process(process).map(|p| p.default_params()).unwrap_or_default();
        let name = wanted.map(str::to_owned).unwrap_or_else(|| format!("{}_{}", product.name, snake(process)));
        product.tasks.push(Task { name: name.clone(), process: process.to_owned(), tool: tool.unwrap_or_default().to_owned(), params, operations: Vec::new() });
        return find_task(product, Some(&name));
    }
    let task = find_task(product, wanted)?;
    if let Some(tool) = tool.filter(|t| !t.trim().is_empty()) {
        task.tool = tool.to_owned();
    }
    Ok(task)
}

/// `spotWeld` → `spot_weld`.
pub fn snake(key: &str) -> String {
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

pub fn normalized(v: [f32; 3]) -> Option<[f32; 3]> {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    (len > 1e-6).then(|| v.map(|c| c / len))
}

/// Add `welds` (positions and normals already final) to `task`; returns their ids.
pub fn add_welds(task: &mut Task, welds: Vec<NewWeld>) -> Result<Vec<String>, String> {
    let point_process = crate::product::process(&task.process).is_none_or(|p| p.target == TargetKind::Point);
    if !point_process {
        return Err(format!("task {} ({}) has path operations, not points", task.name, task.process));
    }
    let mut ids = Vec::new();
    for (n, weld) in welds.into_iter().enumerate() {
        let normal = weld.normal.and_then(normalized).ok_or_else(|| format!("weld {} has no usable normal", n + 1))?;
        let id = match weld.id.filter(|id| !id.trim().is_empty()) {
            Some(id) if task.operations.iter().any(|op| op.id == id) => return Err(format!("task {} already has an operation {id}", task.name)),
            Some(id) => id,
            None => next_id(task, 0),
        };
        task.operations.push(Operation {
            id: id.clone(),
            target: Target::Point { position: rounded(weld.position), normal },
            group: weld.group.unwrap_or_default(),
            params: weld.params,
        });
        ids.push(id);
    }
    Ok(ids)
}

/// Remove operations `ids` (`*`: all) from the named task or every task; returns those removed.
pub fn remove_welds(product: &mut Product, request: &RemoveWelds) -> Result<Vec<String>, String> {
    let all = request.ids.iter().any(|id| id == "*");
    let mut removed = Vec::new();
    let task_name = request.task.as_deref().map(str::trim).filter(|n| !n.is_empty());
    if let Some(name) = task_name {
        find_task(product, Some(name))?;
    }
    for task in product.tasks.iter_mut().filter(|t| task_name.is_none_or(|n| t.name == n)) {
        task.operations.retain(|op| {
            let gone = all || request.ids.contains(&op.id);
            if gone {
                removed.push(op.id.clone());
            }
            !gone
        });
    }
    let missing: Vec<&String> = request.ids.iter().filter(|id| *id != "*" && !removed.contains(id)).collect();
    if !missing.is_empty() {
        return Err(format!("no operation {missing:?} in product {}", product.name));
    }
    Ok(removed)
}

/// The operation `id` of the named task (or of any task).
pub fn find_operation<'a>(product: &'a mut Product, task: Option<&str>, id: &str) -> Result<&'a mut Operation, String> {
    let name = product.name.clone();
    let task = task.map(str::trim).filter(|n| !n.is_empty());
    product
        .tasks
        .iter_mut()
        .filter(|t| task.is_none_or(|n| t.name == n))
        .flat_map(|t| t.operations.iter_mut())
        .find(|op| op.id == id)
        .ok_or_else(|| format!("no operation {id} in product {name}{}", task.map(|t| format!(" task {t}")).unwrap_or_default()))
}

/// Set the group (and priority) of operations `ids`.
pub fn regroup_welds(product: &mut Product, request: &RegroupWelds) -> Result<Vec<String>, String> {
    let mut changed = Vec::new();
    for id in &request.ids {
        let op = find_operation(product, request.task.as_deref(), id)?;
        op.group = request.group.clone();
        if let Some(priority) = request.priority {
            op.params.insert("priority".to_owned(), serde_json::json!(priority));
        }
        changed.push(id.clone());
    }
    Ok(changed)
}

/// The face pattern of a generate request, its rows and columns from `pitch` if given.
pub fn pattern(size: [f32; 3], request: &GenerateWelds) -> Result<FacePattern, String> {
    let face = face_index(&request.face).ok_or_else(|| format!("unknown face {:?}: one of +Z/top, -Z/bottom, +X, -X, +Y, -Y", request.face))?;
    let (mut rows, mut columns) = (request.rows.max(1), request.columns.max(1));
    if let Some(pitch) = request.pitch.filter(|p| *p > 1e-4) {
        let (u, v, _) = face_frame(size, face);
        let along = |(lo, hi): Range| (((hi - lo - 2.0 * request.margin).max(0.0) / pitch).floor() as u32 + 1).clamp(1, 200);
        columns = along(u);
        rows = along(v);
    }
    if rows * columns > 2000 {
        return Err(format!("{rows} × {columns} points is too many (at most 2000)"));
    }
    Ok(FacePattern { face, rows, columns, margin: request.margin.max(0.0), group: request.group.clone() })
}

/// Checks that need no surface: normals, ids, spacing, and for a box its faces and edges.
pub fn check_basic(product: &Product, request: &CheckWelds) -> Vec<WeldIssue> {
    let mut issues = Vec::new();
    let task_name = request.task.as_deref().map(str::trim).filter(|n| !n.is_empty());
    for task in product.tasks.iter().filter(|t| task_name.is_none_or(|n| t.name == n)) {
        let issue = |id: &str, kind: &str, message: String| WeldIssue { task: task.name.clone(), id: id.to_owned(), kind: kind.to_owned(), message };
        let mut seen = std::collections::BTreeSet::new();
        let points: Vec<(&str, [f32; 3], [f32; 3])> = task
            .operations
            .iter()
            .filter_map(|op| match &op.target {
                Target::Point { position, normal } => Some((op.id.as_str(), *position, *normal)),
                Target::Path { .. } => None,
            })
            .collect();
        for op in &task.operations {
            if !seen.insert(op.id.as_str()) {
                issues.push(issue(&op.id, "duplicate_id", format!("id {} is used twice", op.id)));
            }
            if normalized(op.target.normal()).is_none() {
                issues.push(issue(&op.id, "zero_normal", "the normal is zero".to_owned()));
            }
        }
        for (i, (a, pa, _)) in points.iter().enumerate() {
            for (b, pb, _) in &points[i + 1..] {
                let d = dist(*pa, *pb);
                if d < request.min_pitch {
                    issues.push(issue(a, "too_close", format!("{a} and {b} are {:.1} mm apart (minimum {:.1} mm)", d * 1000.0, request.min_pitch * 1000.0)));
                }
            }
        }
        if let crate::product::Geometry::Box { size } = &product.geometry {
            for (id, p, n) in &points {
                let Some((face, offset)) = nearest_face(*size, *p) else { continue };
                if offset.abs() > request.surface_tolerance {
                    let kind = if offset < 0.0 { "inside_part" } else { "off_surface" };
                    issues.push(issue(id, kind, format!("{:.1} mm {} the nearest face {}", offset.abs() * 1000.0, if offset < 0.0 { "inside" } else { "off" }, FACES[face].0)));
                    continue;
                }
                if let Some(n) = normalized(*n) {
                    let want = FACES[face].2;
                    let angle = dot(n, want).clamp(-1.0, 1.0).acos().to_degrees();
                    if angle > 10.0 {
                        issues.push(issue(id, "normal_mismatch", format!("normal is {angle:.0}° off face {}'s ({:?})", FACES[face].0, want)));
                    }
                }
                let edge = edge_distance(*size, face, *p);
                if edge < request.edge_margin {
                    issues.push(issue(id, "near_edge", format!("{:.1} mm from an edge of face {} (minimum {:.1} mm)", edge * 1000.0, FACES[face].0, request.edge_margin * 1000.0)));
                }
            }
        }
    }
    issues
}

fn dist(a: [f32; 3], b: [f32; 3]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// The face of a box of `size` nearest to `p`, and `p`'s signed distance from its plane (outward positive).
pub fn nearest_face(size: [f32; 3], p: [f32; 3]) -> Option<(usize, f32)> {
    let [sx, sy, sz] = size;
    let offsets = [p[2] - sz, -p[2], p[0] - sx / 2.0, -sx / 2.0 - p[0], p[1] - sy / 2.0, -sy / 2.0 - p[1]];
    let inside = offsets.iter().all(|o| *o <= 0.0);
    // Inside: the face nearest is the one with the largest (least negative) offset; outside: the
    // largest positive offset dominates the distance too.
    let (face, offset) = offsets.iter().copied().enumerate().max_by(|a, b| a.1.total_cmp(&b.1))?;
    Some((face, if inside { offset } else { offset.max(0.0) }))
}

/// The distance of `p` (on face `face`) to the face's nearest edge.
fn edge_distance(size: [f32; 3], face: usize, p: [f32; 3]) -> f32 {
    let ((u0, u1), (v0, v1), _) = face_frame(size, face);
    let (u, v) = match face {
        0 | 1 => (p[0], p[1]),
        2 | 3 => (p[1], p[2]),
        _ => (p[0], p[2]),
    };
    (u - u0).min(u1 - u).min(v - v0).min(v1 - v)
}

/// `p` projected onto the surface of a box of `size`, with that face's normal.
pub fn snap_to_box(size: [f32; 3], p: [f32; 3]) -> ([f32; 3], [f32; 3]) {
    let [sx, sy, sz] = size;
    let clamp = |p: [f32; 3]| [p[0].clamp(-sx / 2.0, sx / 2.0), p[1].clamp(-sy / 2.0, sy / 2.0), p[2].clamp(0.0, sz)];
    let (face, _) = nearest_face(size, p).unwrap_or((0, 0.0));
    let mut q = clamp(p);
    match face {
        0 => q[2] = sz,
        1 => q[2] = 0.0,
        2 => q[0] = sx / 2.0,
        3 => q[0] = -sx / 2.0,
        4 => q[1] = sy / 2.0,
        _ => q[1] = -sy / 2.0,
    }
    // Outside a corner region the clamped point is on several faces: the nearest face's normal.
    (q, FACES[face].2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::product::Geometry;

    fn pattern_of(face: usize, rows: u32, columns: u32, margin: f32) -> FacePattern {
        FacePattern { face, rows, columns, margin, group: "g".to_owned() }
    }

    fn task(process: &str) -> Task {
        Task { process: process.to_owned(), ..Task::default() }
    }

    fn cube() -> Product {
        Product { name: "cube".to_owned(), geometry: Geometry::Box { size: [1.0, 0.6, 0.2] }, ..Product::default() }
    }

    #[test]
    fn grid_on_top_face() {
        let ops = grid_points([1.0, 0.6, 0.2], &pattern_of(0, 2, 3, 0.1), &task("spotWeld"));
        assert_eq!(ops.len(), 6);
        assert_eq!(ops[0].id, "W01");
        assert_eq!(ops[0].target, Target::Point { position: [-0.4, -0.2, 0.2], normal: [0.0, 0.0, 1.0] });
        assert_eq!(ops[5].target, Target::Point { position: [0.4, 0.2, 0.2], normal: [0.0, 0.0, 1.0] });
    }

    #[test]
    fn faces_by_name() {
        assert_eq!(face_index("top"), Some(0));
        assert_eq!(face_index("−Z"), Some(1));
        assert_eq!(face_index("x"), Some(2));
        assert_eq!(face_index("-y"), Some(5));
        assert_eq!(face_index("diagonal"), None);
    }

    #[test]
    fn pitch_sets_rows_and_columns() {
        let request = GenerateWelds {
            task: None,
            process: None,
            tool: None,
            face: "+Z".to_owned(),
            rows: 1,
            columns: 1,
            pitch: Some(0.1),
            margin: 0.1,
            group: String::new(),
            replace: false,
        };
        let p = pattern([1.0, 0.6, 0.2], &request).unwrap();
        assert_eq!((p.columns, p.rows), (9, 5));
    }

    #[test]
    fn add_move_regroup_remove() {
        let mut product = cube();
        let task = task_or_new(&mut product, None, None, Some("gun")).unwrap();
        assert_eq!(task.name, "cube_spot_weld");
        assert_eq!(task.tool, "gun");
        let ids = add_welds(task, vec![NewWeld { position: [0.0, 0.0, 0.2], normal: Some([0.0, 0.0, 2.0]), ..NewWeld::default() }; 2]).unwrap();
        assert_eq!(ids, ["W01", "W02"]);
        assert_eq!(task.operations[0].target.normal(), [0.0, 0.0, 1.0]);
        let changed = regroup_welds(&mut product, &RegroupWelds { task: None, ids: vec!["W02".into()], group: "b".into(), priority: Some(2.0) }).unwrap();
        assert_eq!(changed, ["W02"]);
        assert_eq!(product.tasks[0].operations[1].group, "b");
        assert!(remove_welds(&mut product, &RemoveWelds { task: None, ids: vec!["W09".into()] }).is_err());
        assert_eq!(remove_welds(&mut product, &RemoveWelds { task: None, ids: vec!["W01".into()] }).unwrap(), ["W01"]);
        assert_eq!(product.tasks[0].operations.len(), 1);
    }

    #[test]
    fn box_checks_and_snap() {
        let mut product = cube();
        let task = task_or_new(&mut product, Some("frame"), None, None).unwrap();
        add_welds(
            task,
            vec![
                NewWeld { position: [0.0, 0.0, 0.2], normal: Some([0.0, 0.0, 1.0]), ..NewWeld::default() },
                NewWeld { position: [0.005, 0.0, 0.2], normal: Some([1.0, 0.0, 0.0]), ..NewWeld::default() },
                NewWeld { position: [0.0, 0.0, 0.1], normal: Some([0.0, 0.0, 1.0]), ..NewWeld::default() },
                NewWeld { position: [0.498, 0.0, 0.1], normal: Some([1.0, 0.0, 0.0]), ..NewWeld::default() },
            ],
        )
        .unwrap();
        let issues = check_basic(&product, &CheckWelds::default());
        let kinds: Vec<(&str, &str)> = issues.iter().map(|i| (i.id.as_str(), i.kind.as_str())).collect();
        assert!(kinds.contains(&("W01", "too_close")), "{kinds:?}");
        assert!(kinds.contains(&("W02", "normal_mismatch")), "{kinds:?}");
        assert!(kinds.contains(&("W03", "inside_part")), "{kinds:?}");
        assert!(!kinds.iter().any(|(id, _)| *id == "W04"), "{kinds:?}");
        let (q, n) = snap_to_box([1.0, 0.6, 0.2], [0.1, 0.0, 0.25]);
        assert_eq!((q, n), ([0.1, 0.0, 0.2], [0.0, 0.0, 1.0]));
        let (q, n) = snap_to_box([1.0, 0.6, 0.2], [0.45, 0.0, 0.1]);
        assert_eq!((q, n), ([0.5, 0.0, 0.1], [1.0, 0.0, 0.0]));
    }
}
