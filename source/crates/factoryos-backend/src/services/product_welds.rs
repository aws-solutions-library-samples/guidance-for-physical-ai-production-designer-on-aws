//! Weld edits on a stored product, one operation at a time (`/api/v1/product/{name}/welds/*`), and
//! snapping points onto the part's surface (`/api/v1/product/{name}/snap`): what the Product & Task
//! pane does by hand, for the chat agent's MCP tools ([`factoryos_protocol::product_edit`]).
//!
//! Every edit loads `product.json`, applies the change, saves it as the pane's Save does (the
//! Robo Fitter fixture, the cells' derived task programs, the store) and draws the preview, so the
//! 3D view shows the result. The surface is the box itself, the mesh file (OBJ, STL), or a URDF
//! product's links (collision geometry, else visual) with every joint at zero.

use std::path::Path;

use axum::extract::{Path as UrlPath, State};
use axum::http::StatusCode;
use axum::routing::post;
use axum::{Json, Router};
use factoryos_protocol::api;
use factoryos_protocol::product::{Geometry, Product, Target};
use factoryos_protocol::product_edit::{
    self as edit, AddWelds, CheckWelds, GenerateWelds, MoveWeld, NewWeld, RegroupWelds, RemoveWelds, SnapRequest, SnapResult, Snapped, WeldCheck,
    WeldEdit, WeldIssue,
};

use crate::AppState;

pub fn router() -> Router<AppState> {
    Router::new()
        .route(api::PRODUCT_WELDS_ADD, post(add))
        .route(api::PRODUCT_WELDS_REMOVE, post(remove))
        .route(api::PRODUCT_WELDS_MOVE, post(move_weld))
        .route(api::PRODUCT_WELDS_REGROUP, post(regroup))
        .route(api::PRODUCT_WELDS_GENERATE, post(generate))
        .route(api::PRODUCT_WELDS_CHECK, post(check))
        .route(api::PRODUCT_SNAP, post(snap))
}

type Reply<T> = Result<Json<T>, (StatusCode, String)>;

fn bad(message: impl std::fmt::Display) -> (StatusCode, String) {
    (StatusCode::BAD_REQUEST, message.to_string())
}

/// Load product `name`, apply `change` (with the part's surface at hand), save, preview, check.
async fn edit_product(
    state: AppState,
    name: String,
    change: impl FnOnce(&mut Product, &Surface) -> Result<Vec<String>, String> + Send + 'static,
) -> Reply<WeldEdit> {
    let service = state.product.clone();
    let saved = tokio::task::spawn_blocking(move || -> Result<_, (StatusCode, String)> {
        let file = service.load(&name).map_err(|err| (StatusCode::NOT_FOUND, format!("{err:#}")))?;
        let mut product = file.product;
        let surface = Surface::of(&service, &product).map_err(|err| bad(format!("{err:#}")))?;
        let changed = change(&mut product, &surface).map_err(bad)?;
        let saved = service.save(product).map_err(|err| bad(format!("{err:#}")))?;
        let mut issues = edit::check_basic(&saved.product, &CheckWelds::default());
        issues.extend(surface.check(&saved.product, &CheckWelds::default()));
        issues.retain(|i| changed.contains(&i.id));
        // The pane's 3D view follows (a failure to draw is not the edit's).
        if let Err(err) = service.preview(&saved.product) {
            tracing::debug!("Product & Task: preview after a weld edit: {err:#}");
        }
        Ok(WeldEdit { product: saved.product, path: saved.path, changed, issues })
    })
    .await
    .map_err(|err| bad(err.to_string()))??;
    super::product::after_change(&state, &saved.product.name).await;
    super::product::write_through(&state, &saved.product.name);
    Ok(Json(saved))
}

/// A new weld's final position and normal: snapped (if asked) and normal from the surface unless given.
fn place(surface: &Surface, weld: &mut NewWeld, snap: bool) -> Result<(), String> {
    if snap || weld.normal.is_none() {
        let s = surface.snap(weld.position).ok_or("the part has no surface to snap onto (no geometry yet)")?;
        if snap {
            weld.position = s.position;
        }
        weld.normal = weld.normal.or(Some(s.normal));
    }
    Ok(())
}

async fn add(State(state): State<AppState>, UrlPath(name): UrlPath<String>, Json(request): Json<AddWelds>) -> Reply<WeldEdit> {
    edit_product(state, name, move |product, surface| {
        let mut welds = request.welds;
        if welds.is_empty() {
            return Err("no welds given".to_owned());
        }
        for weld in &mut welds {
            place(surface, weld, request.snap)?;
        }
        let task = edit::task_or_new(product, request.task.as_deref(), request.process.as_deref(), request.tool.as_deref())?;
        edit::add_welds(task, welds)
    })
    .await
}

async fn remove(State(state): State<AppState>, UrlPath(name): UrlPath<String>, Json(request): Json<RemoveWelds>) -> Reply<WeldEdit> {
    edit_product(state, name, move |product, _| edit::remove_welds(product, &request)).await
}

async fn move_weld(State(state): State<AppState>, UrlPath(name): UrlPath<String>, Json(request): Json<MoveWeld>) -> Reply<WeldEdit> {
    edit_product(state, name, move |product, surface| {
        let op = edit::find_operation(product, request.task.as_deref(), &request.id)?;
        let Target::Point { position, normal } = &mut op.target else {
            return Err(format!("{} is a path; only points can be moved", request.id));
        };
        let mut to = request.position.unwrap_or(*position);
        if let Some(d) = request.delta {
            to = [to[0] + d[0], to[1] + d[1], to[2] + d[2]];
        }
        let mut weld = NewWeld { position: to, normal: request.normal, ..NewWeld::default() };
        // Only moved points are snapped and get the surface's normal; a pure re-orientation keeps its place.
        let moved = request.position.is_some() || request.delta.is_some();
        if moved {
            place(surface, &mut weld, request.snap)?;
        }
        *position = edit::rounded(weld.position);
        if let Some(n) = weld.normal.and_then(edit::normalized) {
            *normal = n;
        }
        Ok(vec![request.id])
    })
    .await
}

async fn regroup(State(state): State<AppState>, UrlPath(name): UrlPath<String>, Json(request): Json<RegroupWelds>) -> Reply<WeldEdit> {
    edit_product(state, name, move |product, _| edit::regroup_welds(product, &request)).await
}

async fn generate(State(state): State<AppState>, UrlPath(name): UrlPath<String>, Json(request): Json<GenerateWelds>) -> Reply<WeldEdit> {
    edit_product(state, name, move |product, _| {
        let Geometry::Box { size } = product.geometry.clone() else {
            return Err("generate works on box products; for a mesh or URDF part add points with add (snap true)".to_owned());
        };
        let pattern = edit::pattern(size, &request)?;
        let task = edit::task_or_new(product, request.task.as_deref(), request.process.as_deref(), request.tool.as_deref())?;
        if request.replace {
            task.operations.clear();
        }
        let path_process = factoryos_protocol::product::process(&task.process)
            .is_some_and(|p| p.target == factoryos_protocol::product::TargetKind::Path);
        let generated = if path_process { vec![edit::face_outline(size, &pattern, task)] } else { edit::grid_points(size, &pattern, task) };
        let ids = generated.iter().map(|op| op.id.clone()).collect();
        task.operations.extend(generated);
        Ok(ids)
    })
    .await
}

async fn check(State(state): State<AppState>, UrlPath(name): UrlPath<String>, Json(request): Json<CheckWelds>) -> Reply<WeldCheck> {
    let service = state.product.clone();
    tokio::task::spawn_blocking(move || {
        let product = service.load(&name).map_err(|err| (StatusCode::NOT_FOUND, format!("{err:#}")))?.product;
        let surface = Surface::of(&service, &product).map_err(|err| bad(format!("{err:#}")))?;
        let mut issues = edit::check_basic(&product, &request);
        issues.extend(surface.check(&product, &request));
        let task = request.task.as_deref().map(str::trim).filter(|n| !n.is_empty());
        let welds = product
            .tasks
            .iter()
            .filter(|t| task.is_none_or(|n| t.name == n))
            .flat_map(|t| &t.operations)
            .filter(|op| matches!(op.target, Target::Point { .. }))
            .count();
        Ok(Json(WeldCheck { welds, issues }))
    })
    .await
    .map_err(|err| bad(err.to_string()))?
}

async fn snap(State(state): State<AppState>, UrlPath(name): UrlPath<String>, Json(request): Json<SnapRequest>) -> Reply<SnapResult> {
    let service = state.product.clone();
    tokio::task::spawn_blocking(move || {
        let product = service.load(&name).map_err(|err| (StatusCode::NOT_FOUND, format!("{err:#}")))?.product;
        let surface = Surface::of(&service, &product).map_err(|err| bad(format!("{err:#}")))?;
        let points = request.points.iter().map(|p| surface.snap(*p).ok_or_else(|| bad("the part has no surface yet"))).collect::<Result<_, _>>()?;
        Ok(Json(SnapResult { points, surface: product.geometry.kind().to_owned(), triangles: surface.triangle_count() }))
    })
    .await
    .map_err(|err| bad(err.to_string()))?
}

// --- the part's surface ---

pub(crate) type V3 = [f32; 3];

/// The part's surface in the product's frame.
pub(crate) enum Surface {
    Box([f32; 3]),
    Triangles(Vec<[V3; 3]>),
}

impl Surface {
    pub(crate) fn of(service: &super::product::ProductService, product: &Product) -> anyhow::Result<Self> {
        Ok(match &product.geometry {
            Geometry::Box { size } => Self::Box(*size),
            Geometry::Mesh { path, scale } => {
                let file = service.mesh_path(&product.name, path);
                let triangles = mesh_triangles(&file)?;
                Self::Triangles(triangles.into_iter().map(|t| t.map(|v| v.map(|c| c * scale))).collect())
            }
            Geometry::Urdf { path } if path.trim().is_empty() => Self::Triangles(Vec::new()),
            Geometry::Urdf { path } => {
                let file = service.mesh_path(&product.name, path);
                Self::Triangles(urdf_triangles(&file, &service.dir().join(&product.name).join(super::product::URDF_DIR))?)
            }
        })
    }

    /// The part's bounding box in the product frame, `(min, max)`; `None` without geometry.
    pub(crate) fn bounds(&self) -> Option<(V3, V3)> {
        match self {
            Self::Box([x, y, z]) => Some(([-x / 2.0, -y / 2.0, 0.0], [x / 2.0, y / 2.0, *z])),
            Self::Triangles(t) => {
                let mut points = t.iter().flatten();
                let first = *points.next()?;
                Some(points.fold((first, first), |(lo, hi), p| ([0, 1, 2].map(|i| lo[i].min(p[i])), [0, 1, 2].map(|i| hi[i].max(p[i])))))
            }
        }
    }

    fn triangle_count(&self) -> usize {
        match self {
            Self::Box(_) => 12,
            Self::Triangles(t) => t.len(),
        }
    }

    /// `p` on the surface, with the outward normal there.
    pub(crate) fn snap(&self, p: V3) -> Option<Snapped> {
        let (position, normal) = match self {
            Self::Box(size) => edit::snap_to_box(*size, p),
            Self::Triangles(triangles) => {
                let (q, tri) = triangles
                    .iter()
                    .map(|t| (closest_on_triangle(p, t), t))
                    .min_by(|a, b| dist2(p, a.0).total_cmp(&dist2(p, b.0)))?;
                let mut n = edit::normalized(cross(sub(tri[1], tri[0]), sub(tri[2], tri[0]))).unwrap_or([0.0, 0.0, 1.0]);
                // A point given off the surface tells which side is outside better than a mesh's winding.
                let away = sub(p, q);
                if dot(away, away) > 1e-6 && dot(n, away) < 0.0 && !self.inside(p) {
                    n = n.map(|c| -c);
                }
                (q, n)
            }
        };
        let position = edit::rounded(position);
        Some(Snapped { input: p, position, normal, distance: dist2(p, position).sqrt() })
    }

    /// Whether `p` is inside the part (an odd number of crossings along a slightly tilted ray).
    fn inside(&self, p: V3) -> bool {
        match self {
            Self::Box([sx, sy, sz]) => p[0].abs() < sx / 2.0 && p[1].abs() < sy / 2.0 && p[2] > 0.0 && p[2] < *sz,
            Self::Triangles(triangles) => {
                let dir = edit::normalized([0.0123, 0.0071, 1.0]).expect("non-zero");
                triangles.iter().filter(|t| ray_hits(p, dir, t)).count() % 2 == 1
            }
        }
    }

    /// The checks that need the surface (a box's are in [`edit::check_basic`]).
    fn check(&self, product: &Product, request: &CheckWelds) -> Vec<WeldIssue> {
        let Self::Triangles(triangles) = self else { return Vec::new() };
        if triangles.is_empty() {
            return Vec::new();
        }
        let task_name = request.task.as_deref().map(str::trim).filter(|n| !n.is_empty());
        let mut issues = Vec::new();
        for task in product.tasks.iter().filter(|t| task_name.is_none_or(|n| t.name == n)) {
            for op in &task.operations {
                let Target::Point { position, normal } = &op.target else { continue };
                let Some(s) = self.snap(*position) else { continue };
                let issue = |kind: &str, message: String| WeldIssue { task: task.name.clone(), id: op.id.clone(), kind: kind.to_owned(), message };
                if s.distance > request.surface_tolerance {
                    let inside = self.inside(*position);
                    let kind = if inside { "inside_part" } else { "off_surface" };
                    issues.push(issue(kind, format!("{:.1} mm {} the surface (snap it: nearest {:?})", s.distance * 1000.0, if inside { "inside" } else { "off" }, s.position)));
                } else if let Some(n) = edit::normalized(*normal) {
                    let angle = dot(n, s.normal).clamp(-1.0, 1.0).acos().to_degrees();
                    if angle > 15.0 {
                        issues.push(issue("normal_mismatch", format!("normal is {angle:.0}° off the surface's {:?}", s.normal)));
                    }
                }
            }
        }
        issues
    }
}

/// The triangles of an OBJ or STL file.
pub(crate) fn mesh_triangles(file: &Path) -> anyhow::Result<Vec<[V3; 3]>> {
    let bytes = std::fs::read(file).map_err(|err| anyhow::anyhow!("{}: {err}", file.display()))?;
    let ext = file.extension().and_then(|e| e.to_str()).unwrap_or_default().to_ascii_lowercase();
    match ext.as_str() {
        "obj" => {
            let options = tobj::LoadOptions { single_index: true, triangulate: true, ..Default::default() };
            let (models, _) = tobj::load_obj_buf(&mut std::io::Cursor::new(&bytes), &options, |_| Err(tobj::LoadError::OpenFileFailed))
                .map_err(|err| anyhow::anyhow!("{}: {err}", file.display()))?;
            let mut out = Vec::new();
            for model in models {
                let (p, i) = (&model.mesh.positions, &model.mesh.indices);
                let v = |k: u32| [p[3 * k as usize], p[3 * k as usize + 1], p[3 * k as usize + 2]];
                out.extend(i.chunks_exact(3).map(|t| [v(t[0]), v(t[1]), v(t[2])]));
            }
            Ok(out)
        }
        "stl" => {
            let mesh = stl_io::read_stl(&mut std::io::Cursor::new(&bytes)).map_err(|err| anyhow::anyhow!("{}: {err}", file.display()))?;
            let v = |k: usize| {
                let p = mesh.vertices[k];
                [p[0], p[1], p[2]]
            };
            Ok(mesh.faces.iter().map(|f| [v(f.vertices[0]), v(f.vertices[1]), v(f.vertices[2])]).collect())
        }
        other => anyhow::bail!("{}: snapping reads OBJ and STL meshes, not .{other}", file.display()),
    }
}

/// A URDF's geometry (collision, else visual) with every joint at zero, in its root link's frame.
fn urdf_triangles(file: &Path, search: &Path) -> anyhow::Result<Vec<[V3; 3]>> {
    let robot = urdf_robo_dresser::parse_urdf(file).map_err(|err| anyhow::anyhow!("{}: {err}", file.display()))?;
    let dir = file.parent().unwrap_or(Path::new("."));
    let children: std::collections::BTreeSet<&str> = robot.joints.iter().map(|j| j.child.as_str()).collect();
    let root = robot.links.iter().find(|l| !children.contains(l.name.as_str())).map(|l| l.name.clone()).unwrap_or_default();
    // Each link's pose, from the root along the joints.
    let mut poses = std::collections::BTreeMap::from([(root.clone(), Pose::IDENTITY)]);
    let mut queue = vec![root];
    while let Some(parent) = queue.pop() {
        let base = poses[&parent];
        for joint in robot.joints.iter().filter(|j| j.parent == parent) {
            poses.insert(joint.child.clone(), base.then(Pose::from(joint.origin)));
            queue.push(joint.child.clone());
        }
    }
    let mut out = Vec::new();
    for link in &robot.links {
        let Some(pose) = poses.get(&link.name).copied() else { continue };
        let shapes: Vec<(urdf_robo_dresser::Origin, &urdf_robo_dresser::Geometry)> = if link.collisions.is_empty() {
            link.visuals.iter().map(|v| (v.origin, &v.geometry)).collect()
        } else {
            link.collisions.iter().map(|c| (c.origin, &c.geometry)).collect()
        };
        for (origin, geometry) in shapes {
            let at = pose.then(Pose::from(origin));
            let local = match geometry {
                urdf_robo_dresser::Geometry::Mesh { filename, scale } => {
                    let Some(path) = super::product::resolve_mesh(filename, dir, search) else {
                        anyhow::bail!("{}: mesh not found: {filename}", file.display());
                    };
                    let s = scale.map(|c| c as f32);
                    mesh_triangles(&path)?.into_iter().map(|t| t.map(|v| [v[0] * s[0], v[1] * s[1], v[2] * s[2]])).collect()
                }
                urdf_robo_dresser::Geometry::Box { size } => centered_box(size.map(|c| c as f32)),
                // Cylinders and spheres: their bounding box, close enough to snap welds onto.
                urdf_robo_dresser::Geometry::Cylinder { radius, length } => centered_box([2.0 * *radius as f32, 2.0 * *radius as f32, *length as f32]),
                urdf_robo_dresser::Geometry::Sphere { radius } => centered_box([2.0 * *radius as f32; 3]),
            };
            out.extend(local.into_iter().map(|t: [V3; 3]| t.map(|v| at.apply(v))));
        }
    }
    Ok(out)
}

/// 12 outward triangles of a box of `size` centred on the origin.
fn centered_box([x, y, z]: [f32; 3]) -> Vec<[V3; 3]> {
    let (x, y, z) = (x / 2.0, y / 2.0, z / 2.0);
    let c = [[-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z], [-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]];
    [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]]
        .into_iter()
        .map(|[a, b, d]| [c[a], c[b], c[d]])
        .collect()
}

#[derive(Clone, Copy, Debug)]
struct Pose {
    r: [[f32; 3]; 3],
    t: V3,
}

impl Pose {
    const IDENTITY: Self = Self { r: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: [0.0; 3] };

    fn apply(&self, v: V3) -> V3 {
        let r = &self.r;
        [0, 1, 2].map(|i| r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2] + self.t[i])
    }

    /// `self` then `other` (other's frame expressed in self's).
    fn then(&self, other: Self) -> Self {
        let mut r = [[0.0; 3]; 3];
        for (i, row) in r.iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = (0..3).map(|k| self.r[i][k] * other.r[k][j]).sum();
            }
        }
        Self { r, t: self.apply(other.t) }
    }
}

impl From<urdf_robo_dresser::Origin> for Pose {
    /// URDF rpy: fixed-axis roll (x), pitch (y), yaw (z): R = Rz · Ry · Rx.
    fn from(o: urdf_robo_dresser::Origin) -> Self {
        let [roll, pitch, yaw] = o.rpy;
        let (sr, cr) = roll.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sy, cy) = yaw.sin_cos();
        let r = [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ];
        Self { r: r.map(|row| row.map(|c| c as f32)), t: o.xyz.map(|c| c as f32) }
    }
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: V3, b: V3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn dist2(a: V3, b: V3) -> f32 {
    let d = sub(a, b);
    dot(d, d)
}
fn lerp(a: V3, d: V3, s: f32) -> V3 {
    [a[0] + d[0] * s, a[1] + d[1] * s, a[2] + d[2] * s]
}

/// The point of triangle `t` closest to `p` (Ericson, Real-Time Collision Detection 5.1.5).
fn closest_on_triangle(p: V3, t: &[V3; 3]) -> V3 {
    let [a, b, c] = *t;
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return lerp(a, ab, d1 / (d1 - d3));
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return lerp(a, ac, d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return lerp(b, sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denom = 1.0 / (va + vb + vc);
    let (v, w) = (vb * denom, vc * denom);
    [a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w, a[2] + ab[2] * v + ac[2] * w]
}

/// Whether the ray `origin + s·dir` (s > 0) crosses triangle `t` (Möller–Trumbore).
fn ray_hits(origin: V3, dir: V3, t: &[V3; 3]) -> bool {
    let (e1, e2) = (sub(t[1], t[0]), sub(t[2], t[0]));
    let h = cross(dir, e2);
    let a = dot(e1, h);
    if a.abs() < 1e-9 {
        return false;
    }
    let f = 1.0 / a;
    let s = sub(origin, t[0]);
    let u = f * dot(s, h);
    if !(0.0..=1.0).contains(&u) {
        return false;
    }
    let q = cross(s, e1);
    let v = f * dot(dir, q);
    if v < 0.0 || u + v > 1.0 {
        return false;
    }
    f * dot(e2, q) > 1e-7
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_cube() -> Surface {
        Surface::Triangles(centered_box([1.0, 1.0, 1.0]))
    }

    #[test]
    fn snaps_onto_a_mesh_with_its_normal() {
        let s = unit_cube().snap([0.1, 0.2, 0.9]).unwrap();
        assert_eq!(s.position, [0.1, 0.2, 0.5]);
        assert_eq!(s.normal, [0.0, 0.0, 1.0]);
        assert!((s.distance - 0.4).abs() < 1e-5);
        let s = unit_cube().snap([0.45, 0.0, 0.0]).unwrap();
        assert_eq!(s.position, [0.5, 0.0, 0.0]);
        assert_eq!(s.normal, [1.0, 0.0, 0.0]);
    }

    #[test]
    fn inside_by_parity() {
        assert!(unit_cube().inside([0.1, 0.1, 0.1]));
        assert!(!unit_cube().inside([0.1, 0.1, 0.7]));
        assert!(Surface::Box([1.0, 1.0, 1.0]).inside([0.0, 0.0, 0.5]));
    }

    #[test]
    fn mesh_checks_flag_inside_and_normals() {
        let mut product = Product { name: "m".into(), ..Product::default() };
        let task = edit::task_or_new(&mut product, Some("t"), None, None).unwrap();
        edit::add_welds(
            task,
            vec![
                NewWeld { position: [0.0, 0.0, 0.5], normal: Some([0.0, 0.0, 1.0]), ..NewWeld::default() },
                NewWeld { position: [0.0, 0.0, 0.0], normal: Some([0.0, 0.0, 1.0]), ..NewWeld::default() },
                NewWeld { position: [0.2, 0.0, 0.5], normal: Some([1.0, 0.0, 0.0]), ..NewWeld::default() },
            ],
        )
        .unwrap();
        let issues = unit_cube().check(&product, &CheckWelds::default());
        let kinds: Vec<(&str, &str)> = issues.iter().map(|i| (i.id.as_str(), i.kind.as_str())).collect();
        assert_eq!(kinds, [("W02", "inside_part"), ("W03", "normal_mismatch")]);
    }

    #[test]
    fn urdf_poses_compose() {
        let o = urdf_robo_dresser::Origin { xyz: [1.0, 0.0, 0.0], rpy: [0.0, 0.0, std::f64::consts::FRAC_PI_2] };
        let p = Pose::IDENTITY.then(Pose::from(o));
        let v = p.apply([1.0, 0.0, 0.0]);
        assert!((v[0] - 1.0).abs() < 1e-6 && (v[1] - 1.0).abs() < 1e-6, "{v:?}");
    }
}
