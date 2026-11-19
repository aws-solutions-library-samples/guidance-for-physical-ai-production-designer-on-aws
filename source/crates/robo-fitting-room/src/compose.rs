use std::collections::{BTreeMap, HashMap, HashSet};
use std::fs::File;
use std::io::{BufWriter, Seek, Write};
use std::path::{Path, PathBuf};

use urdf_robo_dresser::{
    parse_urdf, Collision, Geometry, Joint, JointType, Link, Origin, UrdfRobot, Visual,
};
use zip::write::SimpleFileOptions;
use zip::CompressionMethod;

use crate::camera_rig::{self, Aabb, CameraRig, Viewpoint};
use crate::drives::{self, DriveParams, HullMesh, MassProps, PrimitiveSolid};
use crate::error::FittingRoomError;
use crate::floor_plan::FloorPlan;
use crate::math3::{self, Mat4};
use crate::mtl::{parse_mtl, Material};
use crate::obj::{parse_obj, ObjMesh};
use crate::scene::{scene_warnings, FixturePlacement, RobotPlacement, SceneDefinition};
use crate::tool::{self, ToolLayout};

#[derive(Debug, Clone)]
pub struct ComposeReport {
    pub zip_path: PathBuf,
    pub usda_name: String,
    pub manifest_name: String,
    pub robot_count: usize,
    pub fixture_count: usize,
    pub floor_plan_segments: usize,
    pub mesh_count: usize,
    pub total_vertices: usize,
    pub total_faces: usize,
    pub usda_bytes: u64,
    pub zip_bytes: u64,
    /// The bundle's planning digest (lowercase hex sha256), also in its zip comment: see
    /// [`PLANNING_DIGEST_KEY`].
    pub planning_sha256: String,
    /// The sha256 of the bundle as composers before the planning digest wrote it, with these
    /// options and lean, also in its zip comment: see [`LEGACY_SHA256_KEY`].
    pub legacy_sha256: Vec<String>,
}

/// Caches parsed OBJs by canonical source path so the same mesh used by
/// MeshCache parses each OBJ once. Geometry is then emitted inline at every
/// visual/collision — same OBJ used in 200 places writes the points/indices
/// 200 times. Larger USDA, but each prim is fully self-contained which
/// avoids reference resolution issues we saw earlier.
///
/// Every mesh is parsed as OBJ, whatever its extension: a mesh that yields no
/// face (an STL or DAE file, BL-25) is reported once, as is the count of
/// OBJs with more than one material, of which only the first is bound
/// (BL-26).
#[derive(Default)]
struct MeshCache {
    map: BTreeMap<PathBuf, ObjMesh>,
    /// The OBJs with more than one `usemtl` material, and their materials.
    multi_material: BTreeMap<PathBuf, Vec<String>>,
}

impl MeshCache {
    fn load(&mut self, urdf_dir: &Path, mesh_filename: &str) -> Result<&ObjMesh, FittingRoomError> {
        let abs = urdf_dir.join(mesh_filename);
        let key = std::fs::canonicalize(&abs).unwrap_or_else(|_| abs.clone());
        if !self.map.contains_key(&key) {
            let mesh = parse_obj(&abs)?;
            if mesh.face_vertex_counts.is_empty() {
                log::warn!(
                    "{}: no faces read (only OBJ meshes are read; STL/DAE is BL-25): it composes \
                     as an empty prim, without geometry, collider or mass",
                    abs.display()
                );
            }
            if mesh.materials.len() > 1 {
                log::debug!(
                    "{}: {} materials ({}); only the first is bound",
                    abs.display(),
                    mesh.materials.len(),
                    mesh.materials.join(", ")
                );
                self.multi_material
                    .insert(key.clone(), mesh.materials.clone());
            }
            self.map.insert(key.clone(), mesh);
        }
        Ok(self.map.get(&key).unwrap())
    }

    /// One warning for every OBJ with more than one material that was read.
    fn warn_multi_material(&self) {
        if let Some((path, materials)) = self.multi_material.iter().next() {
            log::warn!(
                "{} OBJ mesh(es) use more than one material; each is bound to its first material \
                 only (per-face materials are BL-26), e.g. {} ({})",
                self.multi_material.len(),
                path.display(),
                materials.join(", ")
            );
        }
    }
}

/// Tracks materials encountered while emitting visuals. Keyed by globally-unique
/// material name (`<urdf_stem>__<mtl_name>` sanitised) so the same material
/// name from different URDFs is emitted as separate Material prims (in case Kd
/// values differ across URDFs). Within one URDF the first MTL that defines a
/// name wins (keying by MTL file is BL-28).
#[derive(Default)]
struct MaterialRegistry {
    /// usd-prim-name → material data, ordered by insertion (BTreeMap = sorted = deterministic).
    materials: BTreeMap<String, Material>,
    /// Cache of parsed .mtl files by canonical path → material lookup table.
    mtl_cache: BTreeMap<PathBuf, std::collections::HashMap<String, Material>>,
}

impl MaterialRegistry {
    /// Resolve and register the material referenced by an OBJ.
    /// Returns the USD prim name (under `/World/Looks/...`) to bind via
    /// `material:binding`, or None if the OBJ has no material.
    ///
    /// `obj_path` is the absolute path to the OBJ file — the MTL filename
    /// in `mesh.mtllib` is resolved relative to the OBJ's directory.
    ///
    /// FR-111: an MTL material's diffuse colour (`Kd`, sRGB) is written
    /// linear ([`Material::diffuse_linear`]), the colour space of
    /// UsdPreviewSurface's `diffuseColor`; written as is, an 8-bit CAD colour
    /// would come out 1.3–1.9× too bright. A material whose colour isn't known
    /// (no `Kd` line, not in its MTL, or an MTL that can't be read) gets the
    /// default material's linear grey ([`crate::mtl::DEFAULT_DIFFUSE_LINEAR`]).
    fn register_for_mesh(
        &mut self,
        mesh: &ObjMesh,
        obj_path: &Path,
        urdf_stem: &str,
    ) -> Option<String> {
        let mtllib = mesh.mtllib.as_deref()?;
        let usemtl = mesh.usemtl.as_deref()?;
        let obj_dir = obj_path.parent()?;
        let mtl_path = obj_dir.join(mtllib);
        let key = std::fs::canonicalize(&mtl_path).unwrap_or_else(|_| mtl_path.clone());
        if !self.mtl_cache.contains_key(&key) {
            match parse_mtl(&mtl_path) {
                Ok(map) => {
                    self.mtl_cache.insert(key.clone(), map);
                }
                Err(e) => {
                    log::warn!(
                        "MTL parse failed for {}: {}; using placeholder material",
                        mtl_path.display(),
                        e
                    );
                    self.mtl_cache
                        .insert(key.clone(), std::collections::HashMap::new());
                }
            }
        }
        let mtl_table = self.mtl_cache.get(&key).unwrap();
        let material = mtl_table
            .get(usemtl)
            .cloned()
            .unwrap_or_else(Material::placeholder);

        let prim_name = sanitize_prim_name(&format!("{}__{}", urdf_stem, usemtl));
        self.materials.entry(prim_name.clone()).or_insert(material);
        Some(prim_name)
    }

    /// The neutral matte material of meshes without one of their own (an unbound mesh gets the
    /// renderer's default: glossy in Isaac Sim's RTX): the lean robots' hulls and the pedestals.
    fn default_material(&mut self) -> String {
        // No colour: the linear mid grey, `mtl::DEFAULT_DIFFUSE_LINEAR`.
        self.materials
            .entry(DEFAULT_MATERIAL.to_owned())
            .or_insert_with(Material::placeholder);
        DEFAULT_MATERIAL.to_owned()
    }
}

/// [`MaterialRegistry::default_material`]'s prim under `/World/Looks/`.
const DEFAULT_MATERIAL: &str = "factoryos_default";

/// Optional knobs for the compose pipeline.
#[derive(Debug, Clone)]
pub struct ComposeOptions {
    /// When true, emit full visual meshes for robots. When false (default),
    /// robots are emitted with only their collision hulls visible — leaner
    /// scene, identical articulation behaviour. Fixtures are unaffected.
    pub full_robots: bool,
    /// When true (default), keep full visual meshes for tool geometry
    /// merged into a robot URDF. When false, drop tool visuals and emit
    /// only their collision hulls. Tool meshes are detected by the
    /// `meshes/tool/` path prefix that urdf-robo-dresser writes (or are all
    /// of a named payload link's geometry, FR-159, [`ToolLayout`]); other
    /// visuals are treated as robot geometry and follow the `full_robots`
    /// flag.
    pub full_tools: bool,
}

impl Default for ComposeOptions {
    fn default() -> Self {
        Self {
            full_robots: false,
            full_tools: true,
        }
    }
}

// FR-140
pub fn compose_stage(
    scene: &SceneDefinition,
    manifest_path: &Path,
    out_path: &Path,
) -> Result<ComposeReport, FittingRoomError> {
    compose_stage_with_options(scene, manifest_path, out_path, &ComposeOptions::default())
}

pub fn compose_stage_with_options(
    scene: &SceneDefinition,
    manifest_path: &Path,
    out_path: &Path,
    options: &ComposeOptions,
) -> Result<ComposeReport, FittingRoomError> {
    let ext = out_path.extension().and_then(|e| e.to_str()).unwrap_or("");
    if ext != "zip" {
        return Err(FittingRoomError::UnsupportedFormat(format!(
            ".{} (only .zip is supported)",
            ext
        )));
    }

    // Validate the output has a usable stem (we don't otherwise read it —
    // the USDA name inside the zip is derived from the MANIFEST stem so
    // transient names like `.__compose_*.zip` don't leak into the user-
    // visible filename).
    out_path
        .file_stem()
        .and_then(|s| s.to_str())
        .ok_or_else(|| FittingRoomError::UnsupportedFormat("output path has no stem".into()))?;
    let manifest_stem = manifest_path
        .file_stem()
        .and_then(|s| s.to_str())
        .map(|s| s.trim_start_matches('.').trim_start_matches("__compose_"))
        .map(|s| s.trim_start_matches('.'))
        .filter(|s| !s.is_empty())
        .unwrap_or("scene")
        .to_string();
    let usda_name = format!("{}.usda", manifest_stem);
    let manifest_name = manifest_path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| "manifest.yaml".to_string());
    let manifest_bytes = std::fs::read(manifest_path).map_err(|e| {
        FittingRoomError::UsdWrite(format!("read manifest {}: {}", manifest_path.display(), e))
    })?;
    // FR-109: the offset parse_scene subtracted from every placement, as layer provenance.
    let scene_origin = manifest_scene_origin(&String::from_utf8_lossy(&manifest_bytes));

    let (usda_buf, stats) = compose_usda(scene, &usda_name, scene_origin, options, true)?;
    // The bundle's planning digest: that of the lean stage (robots and tools as their collision
    // hulls), the same whatever visuals were asked for, see [`PLANNING_DIGEST_KEY`].
    let lean = ComposeOptions { full_robots: false, full_tools: false };
    let lean_buf = if options.full_robots == lean.full_robots && options.full_tools == lean.full_tools {
        None
    } else {
        Some(compose_usda(scene, &usda_name, scene_origin, &lean, false)?.0)
    };
    let planning = planning_digest(&manifest_bytes, lean_buf.as_deref().unwrap_or(&usda_buf));

    let usda_bytes = usda_buf.len() as u64;
    let floor_plan_segments = scene
        .floor_plan
        .as_ref()
        .map(|fp| fp.segments.len() + fp.footprint_segments.len())
        .unwrap_or(0);

    // The bundle, and the sha256 of its bytes as composers before the planning digest wrote them
    // (the same bytes without the zip comment): with these options, and lean (see
    // [`LEGACY_SHA256_KEY`]).
    let plain = write_bundle(&manifest_name, &manifest_bytes, &usda_name, &usda_buf)?;
    let mut legacy = vec![sha256_hex(&plain)];
    if let Some(lean_buf) = &lean_buf {
        let lean_plain = write_bundle(&manifest_name, &manifest_bytes, &usda_name, lean_buf)?;
        legacy.push(sha256_hex(&lean_plain));
    }
    legacy.dedup();
    let mut comment = format!(
        "{PLANNING_DIGEST_KEY}{planning} full_robots={} full_tools={}",
        options.full_robots, options.full_tools
    );
    for hash in &legacy {
        comment += &format!(" {LEGACY_SHA256_KEY}{hash}");
    }
    let bundle = with_comment(plain, &comment)?;
    std::fs::write(out_path, &bundle).map_err(|e| {
        FittingRoomError::UsdWrite(format!("write {}: {}", out_path.display(), e))
    })?;

    let zip_bytes = std::fs::metadata(out_path).map(|m| m.len()).unwrap_or(0);

    Ok(ComposeReport {
        zip_path: out_path.to_path_buf(),
        usda_name,
        manifest_name,
        robot_count: scene.robots.len(),
        fixture_count: scene.fixtures.len(),
        floor_plan_segments,
        mesh_count: stats.mesh_count,
        total_vertices: stats.total_vertices,
        total_faces: stats.total_faces,
        usda_bytes,
        zip_bytes,
        planning_sha256: planning,
        legacy_sha256: legacy,
    })
}

/// The zip comment's key of a USDA bundle's planning digest: the comment reads
/// `planning-sha256=<hex> full_robots=<bool> full_tools=<bool>`. The digest is the sha256 of
/// `manifest\0<manifest bytes>\0usda\0<lean USDA>`, the USDA composed lean (robots and tools as
/// their collision hulls, `full_robots`/`full_tools` off) whatever the bundle was composed with:
/// what the bundle's visual options change (the robots' and tools' visual meshes and materials)
/// is left out, everything else (the manifest, the placements, the fixtures, the hulls, the
/// physics, the composer's own output format) is in it. Two composes of the same scene with the
/// same inputs have the same digest whichever visuals they asked for; the cells' input hashes
/// use it for their USDA bundle (a planner reads the stage's poses, hulls and fixtures, not the
/// robots' visual meshes).
pub const PLANNING_DIGEST_KEY: &str = "planning-sha256=";

/// The planning digest a bundle's zip comment records ([`PLANNING_DIGEST_KEY`]), if any.
pub fn bundle_planning_digest(comment: &[u8]) -> Option<String> {
    let comment = std::str::from_utf8(comment).ok()?;
    let digest = comment.split_whitespace().find_map(|word| word.strip_prefix(PLANNING_DIGEST_KEY))?;
    (digest.len() == 64 && digest.bytes().all(|b| b.is_ascii_hexdigit())).then(|| digest.to_ascii_lowercase())
}

/// The zip comment's key of a sha256 the bundle had before composers recorded a comment: the
/// sha256 of the same bundle without it, the bytes an older composer wrote for the same scene with
/// the same visual options (the first, `legacy-sha256=<hex>`) and lean (the second, when the bundle
/// isn't lean). Outputs recorded before the planning digest hashed their USDA bundle by its file,
/// so a cell counts those hashes as its bundle's planning digest: Generate USDA of an unchanged
/// scene after the upgrade doesn't stale outputs made from the bundle the older composer wrote,
/// with these visual options or lean.
pub const LEGACY_SHA256_KEY: &str = "legacy-sha256=";

/// The legacy sha256s a bundle's zip comment records ([`LEGACY_SHA256_KEY`]), in order.
pub fn bundle_legacy_hashes(comment: &[u8]) -> Vec<String> {
    let Ok(comment) = std::str::from_utf8(comment) else { return Vec::new() };
    comment
        .split_whitespace()
        .filter_map(|word| word.strip_prefix(LEGACY_SHA256_KEY))
        .filter(|hash| hash.len() == 64 && hash.bytes().all(|b| b.is_ascii_hexdigit()))
        .map(str::to_ascii_lowercase)
        .collect()
}

fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::{Digest as _, Sha256};
    Sha256::digest(bytes).iter().map(|b| format!("{b:02x}")).collect()
}

/// A USDA bundle of the manifest and the USDA, without a zip comment: byte for byte what
/// composers before [`PLANNING_DIGEST_KEY`] wrote (their entries, order and options; dated the
/// zip epoch, as theirs were without zip's `time` feature).
fn write_bundle(
    manifest_name: &str,
    manifest: &[u8],
    usda_name: &str,
    usda: &[u8],
) -> Result<Vec<u8>, FittingRoomError> {
    let mut zip = zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
    let opts = entry_options();
    zip.start_file(manifest_name, opts)
        .map_err(|e| FittingRoomError::UsdWrite(format!("zip manifest entry: {}", e)))?;
    zip.write_all(manifest)
        .map_err(|e| FittingRoomError::UsdWrite(format!("write manifest: {}", e)))?;
    zip.start_file(usda_name, opts)
        .map_err(|e| FittingRoomError::UsdWrite(format!("zip usda entry: {}", e)))?;
    zip.write_all(usda)
        .map_err(|e| FittingRoomError::UsdWrite(format!("write usda: {}", e)))?;
    let cursor = zip
        .finish()
        .map_err(|e| FittingRoomError::UsdWrite(format!("zip finalize: {}", e)))?;
    Ok(cursor.into_inner())
}

/// `zip` (a zip without a comment, as [`write_bundle`] writes it) with the zip comment `comment`:
/// the end of central directory record, its last 22 bytes, gets the comment's length and the
/// comment after it, nothing else changes (what `ZipWriter::set_comment` writes too).
fn with_comment(mut zip: Vec<u8>, comment: &str) -> Result<Vec<u8>, FittingRoomError> {
    const EOCD: usize = 22;
    let len = u16::try_from(comment.len())
        .map_err(|_| FittingRoomError::UsdWrite("zip comment too long".into()))?;
    let start = zip.len().checked_sub(EOCD);
    match start {
        Some(start) if zip[start..start + 4] == [0x50, 0x4b, 0x05, 0x06] && zip[zip.len() - 2..] == [0, 0] => {
            let end = zip.len();
            zip[end - 2..].copy_from_slice(&len.to_le_bytes());
            zip.extend_from_slice(comment.as_bytes());
            Ok(zip)
        }
        _ => Err(FittingRoomError::UsdWrite("zip comment: no end of central directory record".into())),
    }
}

/// [`PLANNING_DIGEST_KEY`]'s digest of a manifest and its lean USDA.
fn planning_digest(manifest: &[u8], lean_usda: &[u8]) -> String {
    use sha2::{Digest as _, Sha256};
    let mut hasher = Sha256::new();
    hasher.update(b"manifest\0");
    hasher.update(manifest);
    hasher.update(b"\0usda\0");
    hasher.update(lean_usda);
    hasher.finalize().iter().map(|b| format!("{b:02x}")).collect()
}

/// The options of every entry of a bundle: deflated, dated 1980-01-01 00:00 (the zip epoch)
/// whatever the clock says, so that the same content makes the same bytes (the zip crate would
/// date entries now with its `time` feature, which any crate of the build may turn on).
fn entry_options() -> SimpleFileOptions {
    SimpleFileOptions::default()
        .compression_method(CompressionMethod::Deflated)
        .last_modified_time(zip::DateTime::default())
}

/// The USDA of `scene` (named `usda_name`): its text and mesh statistics. `report` logs the
/// scene's and the meshes' warnings (off for the lean pass of [`planning_digest`]).
fn compose_usda(
    scene: &SceneDefinition,
    usda_name: &str,
    scene_origin: [f64; 3],
    options: &ComposeOptions,
    report: bool,
) -> Result<(Vec<u8>, MeshStats), FittingRoomError> {
    // The scene's warnings (FR-103): what composes, but not as the manifest may expect.
    if report {
        for warning in scene_warnings(scene) {
            log::warn!("{}", warning);
        }
    }

    let mut cache = MeshCache::default();
    let mut materials = MaterialRegistry::default();
    let mut stats = MeshStats::default();

    // Body buffer: holds robots / fixtures / materials / actiongraphs.
    // The Meshes scope is emitted FIRST in the final output so that
    // `prepend references = </World/Meshes/...>` resolves at parse time.
    let mut body_buf: Vec<u8> = Vec::new();
    {
        let mut w = BufWriter::new(&mut body_buf);

        // FR-109: model kinds — /World an assembly, the robots and fixtures scopes groups of
        // component models.
        if !scene.robots.is_empty() {
            writeln!(w, "    def Scope \"robots\" (")?;
            writeln!(w, "        kind = \"group\"")?;
            writeln!(w, "    )")?;
            writeln!(w, "    {{")?;
            for placement in &scene.robots {
                emit_robot(
                    &mut w,
                    placement,
                    &mut cache,
                    &mut materials,
                    &mut stats,
                    options.full_robots,
                    options.full_tools,
                )?;
            }
            writeln!(w, "    }}")?;
            writeln!(w)?;
        }

        if !scene.fixtures.is_empty() {
            writeln!(w, "    def Scope \"fixtures\" (")?;
            writeln!(w, "        kind = \"group\"")?;
            writeln!(w, "    )")?;
            writeln!(w, "    {{")?;
            for placement in &scene.fixtures {
                emit_fixture(&mut w, placement, &mut cache, &mut materials, &mut stats)?;
            }
            writeln!(w, "    }}")?;
        }

        // FR-119: static mounting pedestals beneath any robot with height > 0.
        let pedestal_robots: Vec<&RobotPlacement> =
            scene.robots.iter().filter(|r| has_pedestal(r)).collect();
        if !pedestal_robots.is_empty() {
            writeln!(w, "    def Scope \"pedestals\"")?;
            writeln!(w, "    {{")?;
            for placement in pedestal_robots {
                emit_pedestal(&mut w, placement, &mut materials)?;
            }
            writeln!(w, "    }}")?;
            writeln!(w)?;
        }

        // FR-107: the floor plan, drawn on the ground.
        if let Some(floor_plan) = &scene.floor_plan {
            emit_floor_plan(&mut w, floor_plan)?;
        }

        emit_materials(&mut w, &materials)?;
        emit_action_graphs(&mut w, &scene.name, &scene.robots)?;

        w.flush()?;
    }

    // FR-108: the camera rig and Kit's opening view frame what was just emitted (whose OBJs are
    // already in the mesh cache). Nothing to frame: no rig, and the header's default view.
    // NFR-108: the ground lies under the same box.
    let bounds = compute_scene_bounds(scene, &mut cache)?;
    if report {
        cache.warn_multi_material();
    }
    let rig = bounds.as_ref().map(CameraRig::framing);
    let perspective = bounds.as_ref().map(camera_rig::perspective_seed);

    let mut usda_buf: Vec<u8> = Vec::new();
    {
        let mut w = BufWriter::new(&mut usda_buf);
        write_header(
            &mut w,
            scene,
            usda_name,
            scene_origin,
            perspective.as_ref(),
        )?;
        writeln!(w, "def Xform \"World\" (")?;
        writeln!(w, "    kind = \"assembly\"")?;
        writeln!(w, ")")?;
        writeln!(w, "{{")?;
        emit_physics_scene(&mut w)?;

        w.write_all(&body_buf)?;

        if let Some(rig) = &rig {
            emit_camera_rig(&mut w, rig)?;
        }

        writeln!(w, "}}")?;
        writeln!(w)?;

        emit_environment(
            &mut w,
            &GroundSquare::under(bounds.as_ref(), scene.floor_plan.as_ref()),
        )?;
        emit_render_block(&mut w)?;

        w.flush()?;
    }

    Ok((usda_buf, stats))
}

#[derive(Default)]
struct MeshStats {
    mesh_count: usize,
    total_vertices: usize,
    total_faces: usize,
}

// =============================================================================
// URDF-source bundle export (additive — does NOT touch USDA composition above).
// Produces a SECOND, self-contained zip: the scene YAML (rewritten to reference
// bundled URDFs by relative path) + every referenced source URDF copied
// byte-for-byte (each in its own folder with its `meshes/` subtree) + the
// floor-plan OBJ when present. Extract it anywhere and `parse_scene` on the
// bundled YAML resolves every asset locally.
// =============================================================================

#[derive(Debug, Clone)]
pub struct UrdfBundleReport {
    pub zip_path: PathBuf,
    pub scene_yaml_name: String,
    pub robot_urdf_count: usize,
    pub fixture_urdf_count: usize,
    pub bundle_dir_count: usize,
    pub floor_plan_bundled: bool,
    pub file_count: usize,
    pub zip_bytes: u64,
}

/// A distinct URDF directory to copy into the bundle, plus the zip folder it
/// was assigned. Keyed in the caller by the canonicalised URDF directory so
/// two placements sharing one dressed URDF collapse to a single entry.
struct BundleEntry {
    /// Directory to copy from (the URDF's own parent dir), as originally resolved.
    src_dir: PathBuf,
    /// Absolute path to the `.urdf` file itself.
    urdf_file: PathBuf,
    /// Assigned bundle folder name (unique within its section).
    folder: String,
}

// FR-147, FR-148
/// Build the URDF-source bundle zip. `manifest_path` is read verbatim to
/// recover the authored YAML (incl. floor-plan path + fields the in-memory
/// `SceneDefinition` doesn't retain); only path fields are rewritten.
pub fn compose_urdf_bundle(
    scene: &SceneDefinition,
    manifest_path: &Path,
    out_path: &Path,
) -> Result<UrdfBundleReport, FittingRoomError> {
    let ext = out_path.extension().and_then(|e| e.to_str()).unwrap_or("");
    if ext != "zip" {
        return Err(FittingRoomError::UnsupportedFormat(format!(
            ".{} (only .zip is supported)",
            ext
        )));
    }

    let manifest_dir = manifest_path.parent().unwrap_or_else(|| Path::new("."));
    let manifest_stem = manifest_path
        .file_stem()
        .and_then(|s| s.to_str())
        .map(|s| s.trim_start_matches('.').trim_start_matches("__compose_"))
        .map(|s| s.trim_start_matches('.'))
        .filter(|s| !s.is_empty())
        .unwrap_or("scene")
        .to_string();
    let scene_yaml_name = format!("{}.yaml", manifest_stem);

    // Collect + dedup URDF dirs, keyed by canonical dir. Robots under `urdf/`,
    // fixtures under `fixtures/`, so the two sections never collide.
    let robot_urdfs: Vec<&Path> = scene.robots.iter().map(|r| r.dressed_urdf.as_path()).collect();
    let fixture_urdfs: Vec<&Path> = scene.fixtures.iter().map(|f| f.urdf.as_path()).collect();
    let robot_entries = collect_bundle_entries(&robot_urdfs)?;
    let fixture_entries = collect_bundle_entries(&fixture_urdfs)?;

    // Reject any URDF whose meshes escape its own directory — such a ref can't
    // be made self-contained by a whole-dir copy. Read-only inspection.
    for entry in robot_entries.values().chain(fixture_entries.values()) {
        check_urdf_meshes_local(&entry.urdf_file)?;
    }

    // canonical URDF dir -> zip-relative folder (e.g. "robots/foo"), used to
    // rewrite the YAML path fields to match where files actually landed.
    let mut robot_dir_to_folder: BTreeMap<PathBuf, String> = BTreeMap::new();
    for (canon, e) in &robot_entries {
        robot_dir_to_folder.insert(canon.clone(), format!("robots/{}", e.folder));
    }
    let mut fixture_dir_to_folder: BTreeMap<PathBuf, String> = BTreeMap::new();
    for (canon, e) in &fixture_entries {
        fixture_dir_to_folder.insert(canon.clone(), format!("fixtures/{}", e.folder));
    }

    // Floor plan: recover its source OBJ path from the manifest YAML (the
    // in-memory FloorPlan struct doesn't keep it) and bundle it under
    // `floorplans/<stem>/<basename>` so it mirrors the robots/ + fixtures/
    // per-item-subfolder layout.
    let manifest_bytes = std::fs::read(manifest_path).map_err(|e| {
        FittingRoomError::UsdWrite(format!("read manifest {}: {}", manifest_path.display(), e))
    })?;
    let manifest_yaml = String::from_utf8_lossy(&manifest_bytes).into_owned();
    let floor_plan_src = floor_plan_geometry_file(&manifest_yaml);
    let floor_plan_target = floor_plan_src.as_ref().and_then(|rel| {
        let name = Path::new(rel).file_name()?.to_string_lossy().into_owned();
        let stem = Path::new(rel)
            .file_stem()
            .map(|s| sanitize_path_segment(&s.to_string_lossy()))
            .unwrap_or_else(|| "plan".to_string());
        Some(format!("floorplans/{}/{}", stem, name))
    });

    // Rewrite the scene YAML path fields (only). Everything else is preserved.
    let rewritten_yaml = rewrite_scene_yaml_paths(
        &manifest_yaml,
        &robot_dir_to_folder,
        &fixture_dir_to_folder,
        manifest_dir,
        floor_plan_target.as_deref(),
    )?;

    let zip_file = File::create(out_path).map_err(|e| {
        FittingRoomError::UsdWrite(format!("create {}: {}", out_path.display(), e))
    })?;
    let mut zip = zip::ZipWriter::new(zip_file);
    let opts = entry_options();

    let mut file_count = 0usize;

    // 1. Scene YAML first.
    zip.start_file(&scene_yaml_name, opts)
        .map_err(|e| FittingRoomError::UsdWrite(format!("zip scene yaml entry: {}", e)))?;
    zip.write_all(rewritten_yaml.as_bytes())
        .map_err(|e| FittingRoomError::UsdWrite(format!("write scene yaml: {}", e)))?;
    file_count += 1;

    // 2. Robot URDF dirs (BTreeMap = sorted = deterministic), then fixtures.
    for e in robot_entries.values() {
        file_count += add_dir_to_zip(&mut zip, opts, &e.src_dir, &format!("robots/{}", e.folder))?;
    }
    for e in fixture_entries.values() {
        file_count +=
            add_dir_to_zip(&mut zip, opts, &e.src_dir, &format!("fixtures/{}", e.folder))?;
    }

    // 3. Floor-plan OBJ last.
    let mut floor_plan_bundled = false;
    if let (Some(src_rel), Some(target)) = (&floor_plan_src, &floor_plan_target) {
        let src = manifest_dir.join(src_rel);
        let bytes = std::fs::read(&src).map_err(|e| {
            FittingRoomError::FloorPlanNotFound(format!("{}: {}", src.display(), e))
        })?;
        zip.start_file(target.as_str(), opts)
            .map_err(|e| FittingRoomError::UsdWrite(format!("zip floor-plan entry: {}", e)))?;
        zip.write_all(&bytes)
            .map_err(|e| FittingRoomError::UsdWrite(format!("write floor-plan: {}", e)))?;
        file_count += 1;
        floor_plan_bundled = true;
    }

    zip.finish()
        .map_err(|e| FittingRoomError::UsdWrite(format!("zip finalize: {}", e)))?;
    let zip_bytes = std::fs::metadata(out_path).map(|m| m.len()).unwrap_or(0);

    Ok(UrdfBundleReport {
        zip_path: out_path.to_path_buf(),
        scene_yaml_name,
        robot_urdf_count: robot_entries.len(),
        fixture_urdf_count: fixture_entries.len(),
        bundle_dir_count: robot_entries.len() + fixture_entries.len(),
        floor_plan_bundled,
        file_count,
        zip_bytes,
    })
}

/// Dedup URDF paths by canonical directory and assign a deterministic,
/// path-safe folder name per distinct directory. Collisions between different
/// source dirs get a numeric suffix in sorted-key order.
fn collect_bundle_entries(
    urdfs: &[&Path],
) -> Result<BTreeMap<PathBuf, BundleEntry>, FittingRoomError> {
    // canonical dir -> (src_dir, urdf_file)
    let mut dirs: BTreeMap<PathBuf, (PathBuf, PathBuf)> = BTreeMap::new();
    for urdf in urdfs {
        let src_dir = urdf
            .parent()
            .map(|p| p.to_path_buf())
            .unwrap_or_else(|| PathBuf::from("."));
        let canon_dir = std::fs::canonicalize(&src_dir).unwrap_or_else(|_| src_dir.clone());
        dirs.entry(canon_dir)
            .or_insert_with(|| (src_dir.clone(), urdf.to_path_buf()));
    }

    let mut used: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut out: BTreeMap<PathBuf, BundleEntry> = BTreeMap::new();
    for (canon, (src_dir, urdf_file)) in dirs {
        // Base name from the dir; fall back to the URDF file stem.
        let base = src_dir
            .file_name()
            .and_then(|n| n.to_str())
            .or_else(|| urdf_file.file_stem().and_then(|s| s.to_str()))
            .unwrap_or("urdf");
        let sanitized = sanitize_path_segment(base);
        let mut folder = sanitized.clone();
        let mut n = 2;
        while used.contains(&folder) {
            folder = format!("{}-{}", sanitized, n);
            n += 1;
        }
        used.insert(folder.clone());
        out.insert(
            canon,
            BundleEntry {
                src_dir,
                urdf_file,
                folder,
            },
        );
    }
    Ok(out)
}

/// Verify every `<mesh>` in the URDF resolves inside the URDF's own directory.
/// A mesh referenced by an absolute path or via `..` can't be captured by a
/// whole-dir copy, so the bundle wouldn't be self-contained — reject it.
fn check_urdf_meshes_local(urdf_file: &Path) -> Result<(), FittingRoomError> {
    let urdf_dir = urdf_file.parent().unwrap_or_else(|| Path::new("."));
    let canon_dir = std::fs::canonicalize(urdf_dir).unwrap_or_else(|_| urdf_dir.to_path_buf());
    let robot = parse_urdf(urdf_file)
        .map_err(|e| FittingRoomError::UrdfParse(format!("{}: {}", urdf_file.display(), e)))?;
    for link in &robot.links {
        let metas = link
            .visuals
            .iter()
            .map(|v| &v.geometry)
            .chain(link.collisions.iter().map(|c| &c.geometry));
        for geom in metas {
            if let Geometry::Mesh { filename, .. } = geom {
                let p = Path::new(filename);
                if p.is_absolute() {
                    return Err(FittingRoomError::Validation(format!(
                        "URDF {} references mesh by absolute path ({}); cannot build a \
                         self-contained bundle",
                        urdf_file.display(),
                        filename
                    )));
                }
                let joined = urdf_dir.join(filename);
                let canon = std::fs::canonicalize(&joined).unwrap_or(joined);
                if !canon.starts_with(&canon_dir) {
                    return Err(FittingRoomError::Validation(format!(
                        "URDF {} references mesh outside its directory ({}); cannot build a \
                         self-contained bundle",
                        urdf_file.display(),
                        filename
                    )));
                }
            }
        }
    }
    Ok(())
}

/// Recursively copy every file under `src_root` into the zip beneath
/// `zip_prefix`, byte-for-byte. Directory entries are sorted so the output is
/// deterministic. Symlinks are skipped with a warning. Returns files written.
fn add_dir_to_zip<W: Write + Seek>(
    zip: &mut zip::ZipWriter<W>,
    opts: SimpleFileOptions,
    src_root: &Path,
    zip_prefix: &str,
) -> Result<usize, FittingRoomError> {
    let mut count = 0usize;
    // Stack of (dir, zip-prefix-for-dir). Sort children per directory.
    let mut stack: Vec<(PathBuf, String)> = vec![(src_root.to_path_buf(), zip_prefix.to_string())];
    while let Some((dir, prefix)) = stack.pop() {
        let mut children: Vec<std::fs::DirEntry> = std::fs::read_dir(&dir)
            .map_err(|e| {
                FittingRoomError::UsdWrite(format!("read dir {}: {}", dir.display(), e))
            })?
            .collect::<Result<Vec<_>, _>>()
            .map_err(|e| {
                FittingRoomError::UsdWrite(format!("read dir {}: {}", dir.display(), e))
            })?;
        children.sort_by_key(|e| e.file_name());
        // Process in sorted order. Push subdirs so they pop in order too; files
        // are written immediately.
        let mut subdirs: Vec<(PathBuf, String)> = Vec::new();
        for child in children {
            let name = child.file_name();
            let name = name.to_string_lossy();
            let meta = child.metadata().map_err(|e| {
                FittingRoomError::UsdWrite(format!("stat {}: {}", child.path().display(), e))
            })?;
            if meta.file_type().is_symlink() {
                log::warn!("skipping symlink in bundle: {}", child.path().display());
                continue;
            }
            let entry_zip_path = format!("{}/{}", prefix, name);
            if meta.is_dir() {
                subdirs.push((child.path(), entry_zip_path));
            } else {
                let bytes = std::fs::read(child.path()).map_err(|e| {
                    FittingRoomError::UsdWrite(format!(
                        "read {}: {}",
                        child.path().display(),
                        e
                    ))
                })?;
                zip.start_file(entry_zip_path.as_str(), opts).map_err(|e| {
                    FittingRoomError::UsdWrite(format!("zip entry {}: {}", entry_zip_path, e))
                })?;
                zip.write_all(&bytes).map_err(|e| {
                    FittingRoomError::UsdWrite(format!("write {}: {}", entry_zip_path, e))
                })?;
                count += 1;
            }
        }
        // Reverse so popping the stack visits subdirs in sorted order.
        for sd in subdirs.into_iter().rev() {
            stack.push(sd);
        }
    }
    Ok(count)
}

/// Extract `floor_plan.geometry_file` (a relative path string) from the raw
/// manifest YAML, if present. Uses a loose `serde_yaml::Value` walk so the rest
/// of the manifest shape is irrelevant.
fn floor_plan_geometry_file(manifest_yaml: &str) -> Option<String> {
    let doc: serde_yaml::Value = serde_yaml::from_str(manifest_yaml).ok()?;
    doc.get("floor_plan")?
        .get("geometry_file")?
        .as_str()
        .map(|s| s.to_string())
}

/// Extract `scene_origin` (x, y, z) from the raw manifest YAML, the same loose
/// way (the in-memory `SceneDefinition` keeps only the shifted placements);
/// (0, 0, 0) where absent, as in `parse_scene`.
fn manifest_scene_origin(manifest_yaml: &str) -> [f64; 3] {
    let doc: Option<serde_yaml::Value> = serde_yaml::from_str(manifest_yaml).ok();
    let origin = doc.as_ref().and_then(|d| d.get("scene_origin"));
    ["x", "y", "z"].map(|axis| {
        origin
            .and_then(|o| o.get(axis))
            .and_then(serde_yaml::Value::as_f64)
            .unwrap_or(0.0)
    })
}

/// Re-serialise the manifest with only the URDF/fixture/floor-plan path fields
/// pointed at their bundled locations. Parsing to `serde_yaml::Value` (not the
/// Deserialize-only scene structs) preserves scene_origin, home_pose_deg,
/// pedestal, offsets, description, and any other keys verbatim.
fn rewrite_scene_yaml_paths(
    manifest_yaml: &str,
    robot_dir_to_folder: &BTreeMap<PathBuf, String>,
    fixture_dir_to_folder: &BTreeMap<PathBuf, String>,
    manifest_dir: &Path,
    floor_plan_target: Option<&str>,
) -> Result<String, FittingRoomError> {
    use serde_yaml::Value;

    let mut doc: Value = serde_yaml::from_str(manifest_yaml)
        .map_err(|e| FittingRoomError::SceneParse(format!("yaml parse: {}", e)))?;

    // Map an authored URDF path (relative to the manifest dir) to its bundled
    // location by resolving to the canonical directory and looking it up.
    let bundled_path = |raw: &str, map: &BTreeMap<PathBuf, String>| -> Option<String> {
        let joined = manifest_dir.join(raw);
        let dir = joined.parent()?.to_path_buf();
        let canon_dir = std::fs::canonicalize(&dir).unwrap_or(dir);
        let folder = map.get(&canon_dir)?;
        let file = Path::new(raw).file_name()?.to_string_lossy().into_owned();
        Some(format!("{}/{}", folder, file))
    };

    if let Some(robots) = doc.get_mut("robots").and_then(Value::as_sequence_mut) {
        for r in robots {
            if let Some(raw) = r.get("dressed_urdf").and_then(Value::as_str) {
                if let Some(new_path) = bundled_path(raw, robot_dir_to_folder) {
                    r["dressed_urdf"] = Value::String(new_path);
                }
            }
        }
    }
    if let Some(fixtures) = doc.get_mut("fixtures").and_then(Value::as_sequence_mut) {
        for f in fixtures {
            if let Some(raw) = f.get("urdf").and_then(Value::as_str) {
                if let Some(new_path) = bundled_path(raw, fixture_dir_to_folder) {
                    f["urdf"] = Value::String(new_path);
                }
            }
        }
    }
    if let Some(target) = floor_plan_target {
        if let Some(fp) = doc.get_mut("floor_plan") {
            if fp.get("geometry_file").is_some() {
                fp["geometry_file"] = Value::String(target.to_string());
            }
        }
    }

    serde_yaml::to_string(&doc)
        .map_err(|e| FittingRoomError::UsdWrite(format!("yaml serialize: {}", e)))
}

/// Sanitise a string into a single, filesystem/zip-safe path segment:
/// ASCII alphanumerics, `_`, `-` kept; everything else becomes `_`. Never
/// empty, never `.`/`..`.
fn sanitize_path_segment(name: &str) -> String {
    let mut out = String::with_capacity(name.len());
    for c in name.chars() {
        if c.is_ascii_alphanumeric() || c == '_' || c == '-' {
            out.push(c);
        } else {
            out.push('_');
        }
    }
    if out.is_empty() || out == "." || out == ".." {
        out = "urdf".to_string();
    }
    out
}

/// The PhysicsScene, tuned for a multi-robot scene (FR-154, FR-155) and written
/// to the schemas (FR-156):
/// - `PhysxSceneAPI` is the applied-schema name of PhysX's scene API
///   (`PhysxSchemaPhysxSceneAPI` is only its C++ class name). OpenUSD skips an
///   unknown name in `apiSchemas` without a warning, and PhysX reads its
///   attributes by the schema's names and types, so every name and type below
///   is the schema's: the GPU capacities and the step rate are `uint`.
/// - Gravity is UsdPhysics' `physics:gravityDirection` / `physics:gravityMagnitude`.
/// - `maxPositionIterationCount` / `maxVelocityIterationCount` cap every
///   actor's own iteration counts, so the position cap must be at least the
///   articulations' 64 (FR-154); a lower cap silently lowers them.
/// - No `NewtonSceneAPI`: Isaac Sim 6.x applies it itself.
fn emit_physics_scene(w: &mut impl Write) -> Result<(), FittingRoomError> {
    writeln!(w, "    def PhysicsScene \"PhysicsScene\" (")?;
    writeln!(w, "        prepend apiSchemas = [\"PhysxSceneAPI\"]")?;
    writeln!(w, "    )")?;
    writeln!(w, "    {{")?;
    writeln!(w, "        vector3f physics:gravityDirection = (0, 0, -1)")?;
    writeln!(w, "        float physics:gravityMagnitude = 9.81")?;
    writeln!(w, "        bool physxScene:enableGPUDynamics = 1")?;
    // GPU buffers for a dense multi-robot scene, none below PhysX's default
    // (FR-155).
    writeln!(
        w,
        "        uint physxScene:gpuFoundLostAggregatePairsCapacity = 8192"
    )?;
    writeln!(
        w,
        "        uint physxScene:gpuFoundLostPairsCapacity = 262144"
    )?;
    writeln!(
        w,
        "        uint physxScene:gpuMaxRigidContactCount = 1048576"
    )?;
    writeln!(w, "        uint physxScene:gpuMaxRigidPatchCount = 81920")?;
    writeln!(
        w,
        "        uint physxScene:gpuTotalAggregatePairsCapacity = 4096"
    )?;
    // Caps, not targets: the articulations ask for 64 / 4 themselves (FR-154).
    writeln!(
        w,
        "        uniform uint physxScene:maxPositionIterationCount = 64"
    )?;
    writeln!(
        w,
        "        uniform uint physxScene:maxVelocityIterationCount = 4"
    )?;
    // Step rate: 120 Hz, dt ~8.3 ms. PhysX solves the joint drives
    // implicitly, so their gains (stiffness 1.5e5 per degree, 8.6e6 N·m/rad;
    // damping for ζ 0.7, FR-158) don't bound the step; contacts and the
    // once-per-step articulation bias forces do.
    writeln!(w, "        uint physxScene:timeStepsPerSecond = 120")?;
    writeln!(w, "    }}")?;
    writeln!(w)?;
    Ok(())
}

/// The layer metadata: NVIDIA's flat-scene template (NFR-108) plus FR-109's
/// provenance, doc and frame rate. `perspective` seeds Kit's opening view
/// (FR-108); without it the template's default stays. NFR-108: the
/// `renderSettings` dictionary is empty, as Kit 110 saves it, so Isaac Sim's
/// own (quality) render settings apply; the template's legacy keys turned
/// anti-aliasing, the denoisers, indirect light, reflections and translucency
/// off.
fn write_header(
    w: &mut impl Write,
    scene: &SceneDefinition,
    usda_name: &str,
    scene_origin: [f64; 3],
    perspective: Option<&Viewpoint>,
) -> Result<(), FittingRoomError> {
    writeln!(w, "#usda 1.0")?;
    writeln!(w, "(")?;
    writeln!(w, "    customLayerData = {{")?;
    writeln!(w, "        dictionary cameraSettings = {{")?;
    writeln!(w, "            dictionary Front = {{")?;
    writeln!(w, "                double3 position = (5, 0, 0)")?;
    writeln!(w, "                double radius = 5")?;
    writeln!(w, "            }}")?;
    writeln!(w, "            dictionary Perspective = {{")?;
    match perspective {
        Some(seed) => {
            let [px, py, pz] = seed.position;
            let [tx, ty, tz] = seed.target;
            writeln!(
                w,
                "                double3 position = ({}, {}, {})",
                f_exact(px),
                f_exact(py),
                f_exact(pz)
            )?;
            writeln!(
                w,
                "                double3 target = ({}, {}, {})",
                f_exact(tx),
                f_exact(ty),
                f_exact(tz)
            )?;
        }
        None => {
            writeln!(w, "                double3 position = (5, 5, 5)")?;
            writeln!(
                w,
                "                double3 target = (-3.978038609631085e-8, -3.978038609631085e-8, 7.956076952808644e-8)"
            )?;
        }
    }
    writeln!(w, "            }}")?;
    writeln!(w, "            dictionary Right = {{")?;
    writeln!(w, "                double3 position = (0, -5, 0)")?;
    writeln!(w, "                double radius = 5")?;
    writeln!(w, "            }}")?;
    writeln!(w, "            dictionary Top = {{")?;
    writeln!(w, "                double3 position = (0, 0, 5)")?;
    writeln!(w, "                double radius = 5")?;
    writeln!(w, "            }}")?;
    writeln!(w, "            string boundCamera = \"/OmniverseKit_Persp\"")?;
    writeln!(w, "        }}")?;
    writeln!(w, "        dictionary omni_layer = {{")?;
    writeln!(
        w,
        "            string authoring_layer = \"./{}\"",
        escape_str(usda_name)
    )?;
    writeln!(w, "            dictionary locked = {{")?;
    writeln!(w, "            }}")?;
    writeln!(w, "            dictionary muteness = {{")?;
    writeln!(w, "            }}")?;
    writeln!(w, "        }}")?;
    writeln!(w, "        dictionary renderSettings = {{")?;
    writeln!(w, "        }}")?;
    writeln!(
        w,
        "        string \"robo_fitting_room:generator\" = \"robo-fitting-room {}\"",
        env!("CARGO_PKG_VERSION")
    )?;
    let [ox, oy, oz] = scene_origin;
    writeln!(
        w,
        "        double3 \"robo_fitting_room:scene_origin_m\" = ({}, {}, {})",
        f_exact(ox),
        f_exact(oy),
        f_exact(oz)
    )?;
    writeln!(w, "    }}")?;
    writeln!(w, "    defaultPrim = \"World\"")?;
    writeln!(w, "    endTimeCode = 1000000")?;
    writeln!(w, "    framesPerSecond = 60")?;
    writeln!(w, "    metersPerUnit = 1")?;
    writeln!(w, "    startTimeCode = 0")?;
    writeln!(w, "    timeCodesPerSecond = 60")?;
    writeln!(w, "    upAxis = \"Z\"")?;
    writeln!(w, "    doc = \"{}\"", escape_str(&layer_doc(scene)))?;
    writeln!(w, ")")?;
    writeln!(w)?;
    Ok(())
}

/// FR-109: the layer's doc, `<name>: <description>`, or the bare name when
/// the scene has no description.
fn layer_doc(scene: &SceneDefinition) -> String {
    let description = scene.description.trim();
    if description.is_empty() {
        scene.name.clone()
    } else {
        format!("{}: {}", scene.name, description)
    }
}

/// NFR-108: the world orientation (w, x, y, z) of the DistantLight of Kit's
/// Default light rig in a Z-up stage, its `xformOp:rotateXYZ = (55, 0, 135)`
/// (see [`emit_environment`]): the sun stands 35° above the horizon in the
/// north-east (azimuth 45°, counter-clockwise from +X); its light travels
/// along (−0.5792, −0.5792, −0.5736).
const DEFAULT_RIG_SUN_ORIENT: [f64; 4] = [
    0.33944435018566016,
    0.17670354420260315,
    0.42660009293331813,
    0.8194911538891423,
];

/// NFR-108: the world orientation (w, x, y, z) of the Default rig's DomeLight
/// in a Z-up stage, its `xformOp:rotateXYZ = (0, 0, 62.30000092834234)`. It
/// turns a dome texture; a dome without one lights the same in any
/// orientation.
const DEFAULT_RIG_DOME_ORIENT: [f64; 4] = [0.8558159940603218, 0.0, 0.0, 0.5172803730188719];

/// NFR-108: the ground's colour: 18 % grey (linear), a grey card's mid grey.
const GROUND_ALBEDO: f64 = 0.18;

/// NFR-108: the ground's roughness: matte.
const GROUND_ROUGHNESS: f64 = 0.85;

/// NFR-108: how many times as far as the scene the ground reaches from its
/// centre (see [`GroundSquare::under`]).
const GROUND_REACH_PER_SCENE_REACH: f64 = 10.0;

/// NFR-108: the ground's shortest side (m), its side where nothing lies on it.
const GROUND_MIN_SIZE_M: f64 = 100.0;

/// NFR-108: the ground's square on the floor (z = 0): its centre and half its
/// side (m).
#[derive(Debug, Clone, Copy, PartialEq)]
struct GroundSquare {
    center: [f64; 2],
    half_side: f64,
}

impl GroundSquare {
    /// The square under the scene: centred on its bounds (FR-108's box; the
    /// origin without one), reaching [`GROUND_REACH_PER_SCENE_REACH`] times as
    /// far as the box and the floor plan (FR-107) reach from that centre (the
    /// larger of the x and y distances), but with a side of at least
    /// [`GROUND_MIN_SIZE_M`]: far enough that the cameras see the scene stand
    /// on it, not on its edge.
    fn under(bounds: Option<&Aabb>, floor_plan: Option<&FloorPlan>) -> Self {
        let center = bounds.map_or([0.0; 2], |b| {
            let [x, y, _] = b.center();
            [x, y]
        });
        let reach_of = |p: &[f64; 3]| (p[0] - center[0]).abs().max((p[1] - center[1]).abs());
        let mut reach = bounds.map_or(0.0, |b| reach_of(&b.min).max(reach_of(&b.max)));
        if let Some(fp) = floor_plan {
            let points = fp.segments.iter().chain(&fp.footprint_segments).flatten();
            for p in points.filter(|p| p.iter().all(|v| v.is_finite())) {
                reach = reach.max(reach_of(p));
            }
        }
        Self {
            center,
            half_side: (GROUND_REACH_PER_SCENE_REACH * reach).max(GROUND_MIN_SIZE_M / 2.0),
        }
    }
}

/// NFR-108: `/Environment`, the stage's lights and ground, all inline (no
/// asset reference: the stage loads offline).
///
/// The lights replicate Kit's "Default" viewport light rig
/// (`omni.kit.viewport.menubar.lighting` 107.3.4, `data/usd/Default.usda`, as
/// Isaac Sim 6.2 / Kit 110 ship it), which Kit lights a stage without lights
/// with: every attribute its two lights author, as authored. Kit references
/// the rig (a Y-up layer) into a Z-up stage under the turn
/// `(x, y, z) → (z, x, y)` (`_add_rig_reference`), and the rig's `Lights`
/// group, `rotateXYZ = (0, -90, -90)`, is that turn's inverse: in a Z-up
/// stage each light's world rotation is its own `rotateXYZ`
/// ([`DEFAULT_RIG_SUN_ORIENT`], [`DEFAULT_RIG_DOME_ORIENT`]). A distant or a
/// dome light's position doesn't matter, so they stay at the origin (the rig
/// puts them 305 units up). No `omni:rtx:usdluxVersion`, as in the rig.
fn emit_environment(w: &mut impl Write, ground: &GroundSquare) -> Result<(), FittingRoomError> {
    writeln!(w, "def Xform \"Environment\"")?;
    writeln!(w, "{{")?;
    writeln!(w, "    quatd xformOp:orient = (1, 0, 0, 0)")?;
    writeln!(w, "    double3 xformOp:scale = (1, 1, 1)")?;
    writeln!(w, "    double3 xformOp:translate = (0, 0, 0)")?;
    writeln!(
        w,
        "    uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\", \"xformOp:scale\"]"
    )?;
    writeln!(w)?;
    // The sun: intensity · 2^exposure = 1024, 7250 K, the sun's 0.53° disc.
    writeln!(w, "    def DistantLight \"defaultLight\" (")?;
    writeln!(w, "        prepend apiSchemas = [\"ShapingAPI\"]")?;
    writeln!(w, "    )")?;
    writeln!(w, "    {{")?;
    writeln!(w, "        float inputs:angle = 0.53")?;
    writeln!(w, "        float inputs:colorTemperature = 7250")?;
    writeln!(w, "        bool inputs:enableColorTemperature = 1")?;
    writeln!(w, "        float inputs:exposure = 10")?;
    writeln!(w, "        float inputs:intensity = 1")?;
    writeln!(w, "        float inputs:shaping:cone:angle = 180")?;
    writeln!(w, "        token visibility = \"inherited\"")?;
    emit_light_xform(w, &DEFAULT_RIG_SUN_ORIENT)?;
    writeln!(w, "    }}")?;
    writeln!(w)?;
    // The sky: 2^9 = 512, 6150 K, no texture, unseen by the camera (a dark
    // backdrop: the render product's background is the dome).
    writeln!(w, "    def DomeLight \"domeLight\" (")?;
    writeln!(w, "        prepend apiSchemas = [\"ShapingAPI\"]")?;
    writeln!(w, "    )")?;
    writeln!(w, "    {{")?;
    writeln!(w, "        bool visibleInPrimaryRay = 0")?;
    writeln!(w, "        float inputs:colorTemperature = 6150")?;
    writeln!(w, "        bool inputs:enableColorTemperature = 1")?;
    writeln!(w, "        float inputs:exposure = 9")?;
    writeln!(w, "        float inputs:intensity = 1")?;
    writeln!(w, "        float inputs:shaping:cone:angle = 180")?;
    writeln!(w, "        token inputs:texture:format = \"latlong\"")?;
    writeln!(w, "        token visibility = \"inherited\"")?;
    emit_light_xform(w, &DEFAULT_RIG_DOME_ORIENT)?;
    writeln!(w, "    }}")?;
    writeln!(w)?;
    emit_ground(w, ground)?;
    writeln!(w, "}}")?;
    writeln!(w)?;
    Ok(())
}

/// A light's xformOps under `/Environment`: at the origin, turned by `orient`
/// (w, x, y, z).
fn emit_light_xform(w: &mut impl Write, orient: &[f64; 4]) -> Result<(), FittingRoomError> {
    let [qw, qx, qy, qz] = *orient;
    writeln!(
        w,
        "        quatd xformOp:orient = ({}, {}, {}, {})",
        f_exact(qw),
        f_exact(qx),
        f_exact(qy),
        f_exact(qz)
    )?;
    writeln!(w, "        double3 xformOp:scale = (1, 1, 1)")?;
    writeln!(w, "        double3 xformOp:translate = (0, 0, 0)")?;
    writeln!(
        w,
        "        uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\", \"xformOp:scale\"]"
    )?;
    Ok(())
}

/// NFR-108: the ground the scene stands on: a matte grey square mesh
/// (`ground`, facing up, in `/Environment/Looks/M_ground`) and its collider,
/// an infinite plane at z = 0 ([`GROUND_COLLIDER_PATH`]; `guide`, so it isn't
/// drawn), which every robot's welded root links filter (FR-119).
fn emit_ground(w: &mut impl Write, ground: &GroundSquare) -> Result<(), FittingRoomError> {
    writeln!(w, "    def Scope \"Looks\"")?;
    writeln!(w, "    {{")?;
    writeln!(w, "        def Material \"M_ground\"")?;
    writeln!(w, "        {{")?;
    writeln!(
        w,
        "            token outputs:surface.connect = </Environment/Looks/M_ground/Shader.outputs:surface>"
    )?;
    writeln!(w)?;
    writeln!(w, "            def Shader \"Shader\"")?;
    writeln!(w, "            {{")?;
    writeln!(
        w,
        "                uniform token info:id = \"UsdPreviewSurface\""
    )?;
    let albedo = f(GROUND_ALBEDO);
    writeln!(
        w,
        "                color3f inputs:diffuseColor = ({albedo}, {albedo}, {albedo})"
    )?;
    writeln!(w, "                float inputs:metallic = 0")?;
    writeln!(
        w,
        "                float inputs:roughness = {}",
        f(GROUND_ROUGHNESS)
    )?;
    writeln!(w, "                token outputs:surface")?;
    writeln!(w, "            }}")?;
    writeln!(w, "        }}")?;
    writeln!(w, "    }}")?;
    writeln!(w)?;
    let (lo, hi) = (f(-ground.half_side), f(ground.half_side));
    writeln!(w, "    def Mesh \"Ground\" (")?;
    writeln!(w, "        prepend apiSchemas = [\"MaterialBindingAPI\"]")?;
    writeln!(w, "    )")?;
    writeln!(w, "    {{")?;
    writeln!(
        w,
        "        float3[] extent = [({lo}, {lo}, 0), ({hi}, {hi}, 0)]"
    )?;
    writeln!(w, "        int[] faceVertexCounts = [4]")?;
    writeln!(w, "        int[] faceVertexIndices = [0, 1, 2, 3]")?;
    writeln!(
        w,
        "        rel material:binding = </Environment/Looks/M_ground>"
    )?;
    writeln!(
        w,
        "        normal3f[] normals = [(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)] ("
    )?;
    writeln!(w, "            interpolation = \"vertex\"")?;
    writeln!(w, "        )")?;
    writeln!(
        w,
        "        point3f[] points = [({lo}, {lo}, 0), ({hi}, {lo}, 0), ({hi}, {hi}, 0), ({lo}, {hi}, 0)]"
    )?;
    writeln!(w, "        uniform token subdivisionScheme = \"none\"")?;
    writeln!(
        w,
        "        double3 xformOp:translate = ({}, {}, 0)",
        f(ground.center[0]),
        f(ground.center[1])
    )?;
    writeln!(
        w,
        "        uniform token[] xformOpOrder = [\"xformOp:translate\"]"
    )?;
    writeln!(w, "    }}")?;
    writeln!(w)?;
    writeln!(w, "    def Plane \"GroundCollider\" (")?;
    writeln!(w, "        prepend apiSchemas = [\"PhysicsCollisionAPI\"]")?;
    writeln!(w, "    )")?;
    writeln!(w, "    {{")?;
    writeln!(w, "        uniform token axis = \"Z\"")?;
    writeln!(w, "        uniform token purpose = \"guide\"")?;
    writeln!(w, "    }}")?;
    Ok(())
}

/// NFR-108: Kit's `/Render` scope: the viewport's render product, the global
/// render settings and the `LdrColor` render var, typed as Isaac Sim 6.x (Kit
/// 110) saves them (`uint` sample and region limits, the var's `raw` source
/// and its scope's metadata). No `omni:rtx:rendermode`: the app's own
/// default (RealTimePathTracing in Isaac Sim 6.x) renders the stage, not
/// Interactive Path Tracing.
fn emit_render_block(w: &mut impl Write) -> Result<(), FittingRoomError> {
    writeln!(w, "def \"Render\" (")?;
    writeln!(w, "    hide_in_stage_window = true")?;
    writeln!(w, "    no_delete = true")?;
    writeln!(w, ")")?;
    writeln!(w, "{{")?;
    writeln!(w, "    def \"OmniverseKit\"")?;
    writeln!(w, "    {{")?;
    writeln!(w, "        def \"HydraTextures\" (")?;
    writeln!(w, "            hide_in_stage_window = true")?;
    writeln!(w, "            no_delete = true")?;
    writeln!(w, "        )")?;
    writeln!(w, "        {{")?;
    writeln!(
        w,
        "            def RenderProduct \"omni_kit_widget_viewport_ViewportTexture_0\" ("
    )?;
    writeln!(
        w,
        "                prepend apiSchemas = [\"OmniRtxSettingsCommonAdvancedAPI_1\", \"OmniRtxSettingsRtAdvancedAPI_1\", \"OmniRtxSettingsPtAdvancedAPI_1\", \"OmniRtxPostColorGradingAPI_1\", \"OmniRtxPostChromaticAberrationAPI_1\", \"OmniRtxPostBloomPhysicalAPI_1\", \"OmniRtxPostMatteObjectAPI_1\", \"OmniRtxPostCompositingAPI_1\", \"OmniRtxPostDofAPI_1\", \"OmniRtxPostMotionBlurAPI_1\", \"OmniRtxPostTvNoiseAPI_1\", \"OmniRtxPostTonemapIrayReinhardAPI_1\", \"OmniRtxPostDebugSettingsAPI_1\", \"OmniRtxDebugSettingsAPI_1\"]"
    )?;
    writeln!(w, "                hide_in_stage_window = true")?;
    writeln!(w, "                no_delete = true")?;
    writeln!(w, "            )")?;
    writeln!(w, "            {{")?;
    writeln!(w, "                rel camera = </OmniverseKit_Persp>")?;
    writeln!(
        w,
        "                token omni:rtx:background:source:texture:textureMode = \"repeatMirrored\""
    )?;
    writeln!(
        w,
        "                token omni:rtx:background:source:type = \"domeLight\""
    )?;
    writeln!(w, "                bool omni:rtx:dlss:frameGeneration = 0")?;
    writeln!(
        w,
        "                bool omni:rtx:material:db:nonVisualMaterialCSV:enabled = 0"
    )?;
    writeln!(w, "                bool omni:rtx:material:db:syncLoads = 1")?;
    writeln!(
        w,
        "                token omni:rtx:post:dlss:execMode = \"performance\""
    )?;
    writeln!(
        w,
        "                bool omni:rtx:post:registeredCompositing:invertColorCorrection = 1"
    )?;
    writeln!(
        w,
        "                bool omni:rtx:post:registeredCompositing:invertToneMap = 1"
    )?;
    writeln!(
        w,
        "                bool omni:rtx:pt:lightcache:cached:dontResolveConflicts = 1"
    )?;
    writeln!(w, "                uint omni:rtx:pt:maxSamplesPerLaunch = 2073600")?;
    writeln!(
        w,
        "                uint omni:rtx:pt:mgpu:maxPixelsPerRegionExponent = 12"
    )?;
    writeln!(
        w,
        "                color3f omni:rtx:rt:ambientLight:color = (0.1, 0.1, 0.1)"
    )?;
    writeln!(w, "                bool omni:rtx:rt:demoire = 0")?;
    writeln!(
        w,
        "                bool omni:rtx:rt:lightcache:spatialCache:dontResolveConflicts = 1"
    )?;
    writeln!(
        w,
        "                bool omni:rtx:scene:hydra:materialSyncLoads = 1"
    )?;
    writeln!(w, "                bool omni:rtx:scene:hydra:mdlMaterialWarmup = 1")?;
    writeln!(w, "                uint omni:rtx:viewTile:limit = 4294967295")?;
    writeln!(w, "                rel orderedVars = </Render/Vars/LdrColor>")?;
    writeln!(w, "                custom bool overrideClipRange = 0")?;
    let (width, height) = camera_rig::RENDER_RESOLUTION;
    writeln!(
        w,
        "                uniform int2 resolution = ({}, {})",
        width, height
    )?;
    writeln!(w, "            }}")?;
    writeln!(w, "        }}")?;
    writeln!(w, "    }}")?;
    writeln!(w)?;
    writeln!(
        w,
        "    def RenderSettings \"OmniverseGlobalRenderSettings\" ("
    )?;
    writeln!(
        w,
        "        prepend apiSchemas = [\"OmniRtxSettingsGlobalRtAdvancedAPI_1\", \"OmniRtxSettingsGlobalPtAdvancedAPI_1\"]"
    )?;
    writeln!(w, "        no_delete = true")?;
    writeln!(w, "    )")?;
    writeln!(w, "    {{")?;
    writeln!(
        w,
        "        rel products = </Render/OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_0>"
    )?;
    writeln!(w, "    }}")?;
    writeln!(w)?;
    writeln!(w, "    def \"Vars\" (")?;
    writeln!(w, "        hide_in_stage_window = true")?;
    writeln!(w, "        no_delete = true")?;
    writeln!(w, "    )")?;
    writeln!(w, "    {{")?;
    writeln!(w, "        def RenderVar \"LdrColor\" (")?;
    writeln!(w, "            hide_in_stage_window = true")?;
    writeln!(w, "            no_delete = true")?;
    writeln!(w, "        )")?;
    writeln!(w, "        {{")?;
    writeln!(w, "            uniform string sourceName = \"LdrColor\"")?;
    writeln!(w, "            uniform token sourceType = \"raw\"")?;
    writeln!(w, "        }}")?;
    writeln!(w, "    }}")?;
    writeln!(w, "}}")?;
    writeln!(w)?;
    Ok(())
}

// FR-110, FR-111, FR-112, FR-113, FR-114, FR-115, FR-157, FR-158, FR-159
fn emit_robot(
    w: &mut impl Write,
    placement: &RobotPlacement,
    cache: &mut MeshCache,
    materials: &mut MaterialRegistry,
    stats: &mut MeshStats,
    full_visuals: bool,
    full_tools: bool,
) -> Result<(), FittingRoomError> {
    let robot = parse_urdf(&placement.dressed_urdf).map_err(|e| {
        FittingRoomError::UrdfParse(format!("{}: {}", placement.dressed_urdf.display(), e))
    })?;
    let urdf_dir = placement
        .dressed_urdf
        .parent()
        .unwrap_or(Path::new("."))
        .to_path_buf();
    let urdf_stem = placement
        .dressed_urdf
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("unknown")
        .to_string();

    let robot_id = sanitize_prim_name(&placement.robot_id);
    let quat = abc_deg_to_quat(placement.orientation_deg);

    // FR-119: the robot foot, raised onto its pedestal. The raised z must be
    // applied to BOTH the Xform translate and the world_anchor localPos0
    // below, or PhysX snaps the joint back together.
    let raised = raised_position(placement);

    // Apply both PhysicsArticulationRootAPI and PhysxArticulationAPI:
    // - PhysicsArticulationRootAPI marks the prim as the start of an articulation tree
    // - PhysxArticulationAPI exposes PhysX-specific solver attributes and is what
    //   Isaac Sim's articulation tools (joint sliders, articulation view) key off
    // Both together = full recognition by Isaac Sim's articulation panel.
    // FR-109: each robot is a component model.
    writeln!(w, "    def Xform \"{}\" (", robot_id)?;
    writeln!(
        w,
        "        prepend apiSchemas = [\"PhysicsArticulationRootAPI\", \"PhysxArticulationAPI\"]"
    )?;
    writeln!(w, "        kind = \"component\"")?;
    writeln!(w, "    )")?;
    writeln!(w, "    {{")?;
    writeln!(w, "        uniform bool physxArticulation:enabledSelfCollisions = 0")?;
    // FR-154: 64 position iterations for heavy arms carrying a payload;
    // velocity iterations at the recommended baseline of 4.
    writeln!(w, "        uniform int physxArticulation:solverPositionIterationCount = 64")?;
    writeln!(w, "        uniform int physxArticulation:solverVelocityIterationCount = 4")?;
    writeln!(w, "        uniform float physxArticulation:sleepThreshold = 0.005")?;
    writeln!(w, "        uniform float physxArticulation:stabilizationThreshold = 0.001")?;
    writeln!(
        w,
        "        double3 xformOp:translate = ({}, {}, {})",
        f(raised[0]),
        f(raised[1]),
        f(raised[2])
    )?;
    writeln!(
        w,
        "        quatd xformOp:orient = ({}, {}, {}, {})",
        f(quat[3]),
        f(quat[0]),
        f(quat[1]),
        f(quat[2])
    )?;
    writeln!(
        w,
        "        uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\"]"
    )?;
    writeln!(w)?;

    // FR-157: one home value per actuated joint feeds the link bind poses,
    // the drive targets and the initial joint states alike.
    let homes = drives::actuated_home_values(&robot, placement.home_pose_deg.as_deref());
    let bind_poses = compute_link_bind_poses(&robot, &homes);
    // FR-159: the robot's tool — which geometry is the tool's, and the link a
    // payload merges into (the manifest's `payload.link`, else found).
    let layout = ToolLayout::of(
        &robot,
        placement.payload.as_ref().and_then(|p| p.link.as_deref()),
    );
    // The welded root links don't collide with what the base stands on, and
    // nothing moves them (FR-113's mass fallback).
    let welded = welded_root_links(&robot);
    // FR-113 / FR-159: the mass properties every link is emitted with, the
    // payload merged in; the drives are tuned on the same ones (FR-158).
    let masses = robot_mass_properties(
        &robot,
        placement,
        &urdf_dir,
        &layout,
        &welded,
        &bind_poses,
        cache,
    );
    let filter_targets = welded_filter_targets(placement);
    for link in &robot.links {
        let pose = bind_poses
            .get(&link.name)
            .copied()
            .unwrap_or(math3::IDENTITY);
        let mass = masses
            .get(&link.name)
            .copied()
            .unwrap_or_else(MassProps::fallback);
        let kind = LinkKind::Articulated {
            mass: &mass,
            filtered_pairs: if welded.contains(link.name.as_str()) {
                &filter_targets
            } else {
                &[]
            },
            all_tool: layout.all_tool(&link.name),
        };
        emit_link(
            w,
            link,
            &urdf_dir,
            &urdf_stem,
            cache,
            materials,
            stats,
            8,
            kind,
            &pose,
            full_visuals,
            full_tools,
        )?;
    }

    // `homes` holds one value per actuated (non-fixed) joint, in order.
    let mut actuated_homes = homes.iter().copied();
    for joint in &robot.joints {
        let drive = (joint.joint_type != JointType::Fixed).then(|| {
            let home = actuated_homes.next().unwrap_or(0.0);
            drives::drive_params(&robot, joint, home, &bind_poses, &masses)
        });
        emit_joint(w, joint, &robot_id, 8, drive.as_ref())?;
    }

    if let Some(root_link) = find_root_link(&robot) {
        let safe_root = sanitize_prim_name(root_link);
        // FixedJoint anchoring root link to world inertial frame.
        // - body0 = empty rel (the world inertial frame)
        // - body1 = root link (positioned via parent Xform's xformOp)
        // - localPos0/localRot0 must match the body1's current world pose so
        //   PhysX doesn't try to snap them together.
        writeln!(w, "        def PhysicsFixedJoint \"world_anchor\"")?;
        writeln!(w, "        {{")?;
        writeln!(w, "            rel physics:body0")?;
        writeln!(
            w,
            "            rel physics:body1 = </World/robots/{}/{}>",
            robot_id, safe_root
        )?;
        writeln!(
            w,
            "            point3f physics:localPos0 = ({}, {}, {})",
            f(raised[0]),
            f(raised[1]),
            f(raised[2])
        )?;
        writeln!(
            w,
            "            quatf physics:localRot0 = ({}, {}, {}, {})",
            f(quat[3]),
            f(quat[0]),
            f(quat[1]),
            f(quat[2])
        )?;
        writeln!(w, "            point3f physics:localPos1 = (0, 0, 0)")?;
        writeln!(w, "            quatf physics:localRot1 = (1, 0, 0, 0)")?;
        writeln!(w, "        }}")?;
    }

    writeln!(w, "    }}")?;
    writeln!(w)?;
    Ok(())
}

/// FR-113 / FR-159: every link's mass properties by name
/// ([`drives::link_mass_properties`]) from its URDF inertial and its geometry
/// ([`link_meshes`]) at the robot's link density (its manifest's
/// `link_density_kg_m3`, else the default), the robot's payload merged into
/// the tool's carrier link ([`ToolLayout`]): its explicit centre of mass and
/// inertia, or else spread over the tool's collision geometry — that of the
/// carrier and of the links fixed below it (their bind poses `bind_poses`
/// bring it into the carrier's frame), which isn't weighed again. A link with
/// neither geometry nor a usable inertial gets the 10 kg fallback when it is
/// one of the `welded` root links, else the frame link's small mass.
fn robot_mass_properties(
    robot: &UrdfRobot,
    placement: &RobotPlacement,
    urdf_dir: &Path,
    layout: &ToolLayout,
    welded: &HashSet<&str>,
    bind_poses: &HashMap<String, Mat4>,
    cache: &mut MeshCache,
) -> HashMap<String, MassProps> {
    let density = drives::link_density(placement.link_density_kg_m3);
    let carrier = match (&placement.payload, &layout.carrier) {
        (Some(payload), Some(carrier)) => Some((carrier.as_str(), payload)),
        (Some(_), None) => {
            log::warn!(
                "{}: no link to carry the payload; ignored",
                placement.robot_id
            );
            None
        }
        _ => None,
    };
    let mut meshes: Vec<drives::LinkMeshes> = robot
        .links
        .iter()
        .map(|link| {
            let context = format!("{} link {}", placement.robot_id, link.name);
            link_meshes(link, layout, urdf_dir, cache, &context)
        })
        .collect();
    // The payload's body, spread over the tool's collision geometry in the
    // carrier's frame; the tool's links other than the carrier don't weigh
    // theirs again.
    let payload = carrier.map(|(carrier, payload)| {
        let to_carrier = bind_poses
            .get(carrier)
            .map(math3::rigid_inverse)
            .unwrap_or(math3::IDENTITY);
        let (mut hulls, mut primitives) = (Vec::new(), Vec::new());
        for (link, link_meshes) in robot.links.iter().zip(&mut meshes) {
            if !layout.is_tool_link(&link.name) {
                continue;
            }
            let pose = math3::multiply(
                &to_carrier,
                &bind_poses
                    .get(&link.name)
                    .copied()
                    .unwrap_or(math3::IDENTITY),
            );
            hulls.extend(link_meshes.tool.iter().map(|hull| {
                HullMesh {
                    name: hull.name.clone(),
                    points: hull
                        .points
                        .iter()
                        .map(|p| math3::transform_point(&pose, *p))
                        .collect(),
                    triangle_indices: hull.triangle_indices.clone(),
                }
            }));
            primitives.extend(
                link_meshes
                    .tool_primitives
                    .iter()
                    .map(|p| p.transformed(&pose)),
            );
            if link.name != carrier {
                link_meshes.tool.clear();
                link_meshes.tool_primitives.clear();
            }
        }
        let body = drives::payload_body(
            payload.mass_kg,
            payload.center_of_mass_m,
            payload.inertia_kgm2,
            &hulls,
            &primitives,
            &format!("{} payload on {}", placement.robot_id, carrier),
        );
        (carrier, body)
    });
    robot
        .links
        .iter()
        .zip(&meshes)
        .map(|(link, link_meshes)| {
            let context = format!("{} link {}", placement.robot_id, link.name);
            let body = payload
                .as_ref()
                .filter(|(carrier, _)| *carrier == link.name)
                .map(|(_, body)| body);
            let props = drives::link_mass_properties(
                link,
                link_meshes,
                body,
                density,
                welded.contains(link.name.as_str()),
                &context,
            );
            (link.name.clone(), props)
        })
        .collect()
}

/// FR-113: the geometry `link`'s mass is taken from, in the link's frame (each
/// mesh's scale and origin applied; primitives analytically): its robot
/// geometry — the collisions that aren't the tool's ([`ToolLayout::is_tool`]),
/// or its visuals that aren't when it has no such collision — and its tool's
/// collisions. A mesh that doesn't load is skipped with a warning naming
/// `context`; wherever compose emits it, it fails on it.
fn link_meshes(
    link: &Link,
    layout: &ToolLayout,
    urdf_dir: &Path,
    cache: &mut MeshCache,
    context: &str,
) -> drives::LinkMeshes {
    let is_tool = |geometry: &Geometry| layout.is_tool(&link.name, geometry);
    let robot_collisions: Vec<(&Origin, &Geometry)> = link
        .collisions
        .iter()
        .filter(|c| !is_tool(&c.geometry))
        .map(|c| (&c.origin, &c.geometry))
        .collect();
    let robot = if robot_collisions.is_empty() {
        link.visuals
            .iter()
            .filter(|v| !is_tool(&v.geometry))
            .map(|v| (&v.origin, &v.geometry))
            .collect()
    } else {
        robot_collisions
    };
    let tool: Vec<(&Origin, &Geometry)> = link
        .collisions
        .iter()
        .filter(|c| is_tool(&c.geometry))
        .map(|c| (&c.origin, &c.geometry))
        .collect();
    let mut load = |shapes: &[(&Origin, &Geometry)]| -> (Vec<HullMesh>, Vec<PrimitiveSolid>) {
        let meshes = shapes
            .iter()
            .filter_map(|(origin, geometry)| {
                closed_mesh(origin, geometry, urdf_dir, cache, context)
            })
            .collect();
        let primitives = shapes
            .iter()
            .filter_map(|(origin, geometry)| primitive_solid(origin, geometry, context))
            .collect();
        (meshes, primitives)
    };
    let (robot, robot_primitives) = load(&robot);
    let (tool, tool_primitives) = load(&tool);
    drives::LinkMeshes {
        robot,
        tool,
        robot_primitives,
        tool_primitives,
    }
}

/// A URDF mesh as a closed triangle mesh in its link's frame (its scale and
/// `origin` applied); `None` for a primitive or a mesh that doesn't load.
fn closed_mesh(
    origin: &Origin,
    geometry: &Geometry,
    urdf_dir: &Path,
    cache: &mut MeshCache,
    context: &str,
) -> Option<HullMesh> {
    let Geometry::Mesh { filename, scale } = geometry else {
        return None;
    };
    let mesh = match cache.load(urdf_dir, filename) {
        Ok(mesh) => mesh,
        Err(e) => {
            log::warn!("{context}: mass: skipping mesh {filename}: {e}");
            return None;
        }
    };
    let pose = math3::from_translation_rpy(origin.xyz, origin.rpy);
    Some(HullMesh {
        name: filename.clone(),
        points: mesh
            .points
            .iter()
            .map(|p| {
                let scaled = [p[0] * scale[0], p[1] * scale[1], p[2] * scale[2]];
                math3::transform_point(&pose, scaled)
            })
            .collect(),
        triangle_indices: mesh.face_vertex_indices.clone(),
    })
}

/// A URDF primitive (box, cylinder, sphere) as a solid in its link's frame,
/// placed by its `origin`; `None` for a mesh, and for a primitive whose
/// dimensions aren't finite and > 0 (with a warning naming `context`).
fn primitive_solid(origin: &Origin, geometry: &Geometry, context: &str) -> Option<PrimitiveSolid> {
    if matches!(geometry, Geometry::Mesh { .. }) {
        return None;
    }
    let name = primitive_name(geometry);
    let pose = math3::from_translation_rpy(origin.xyz, origin.rpy);
    let solid = PrimitiveSolid::of(geometry, &pose, name);
    if solid.is_none() {
        log::warn!("{context}: mass: skipping a degenerate {name}");
    }
    solid
}

/// The URDF element name of a primitive: `box`, `cylinder` or `sphere`.
fn primitive_name(geometry: &Geometry) -> &'static str {
    match geometry {
        Geometry::Box { .. } => "box",
        Geometry::Cylinder { .. } => "cylinder",
        Geometry::Sphere { .. } => "sphere",
        Geometry::Mesh { .. } => "mesh",
    }
}

/// The robot's welded root links: the URDF root link and every link joined to
/// it through fixed joints only (`world_link` and the base of a dressed robot).
/// The world anchor holds them still.
fn welded_root_links(robot: &UrdfRobot) -> HashSet<&str> {
    let mut welded = HashSet::new();
    let Some(root) = find_root_link(robot) else {
        return welded;
    };
    welded.insert(root);
    let mut stack = vec![root];
    while let Some(parent) = stack.pop() {
        for joint in robot
            .joints
            .iter()
            .filter(|j| j.parent == parent && j.joint_type == JointType::Fixed)
        {
            if welded.insert(joint.child.as_str()) {
                stack.push(joint.child.as_str());
            }
        }
    }
    welded
}

/// The colliders a robot's welded root links don't collide with
/// (`PhysicsFilteredPairsAPI`): its own pedestal's cylinder (FR-119), if it
/// has one, and the ground's collider (NFR-108). The base hull sits a few
/// millimetres into what it stands on, and neither can move, so their contact
/// would only flood the base's contact reports.
fn welded_filter_targets(placement: &RobotPlacement) -> Vec<String> {
    let mut targets = Vec::new();
    if has_pedestal(placement) {
        targets.push(pedestal_collider_path(placement));
    }
    targets.push(GROUND_COLLIDER_PATH.to_owned());
    targets
}

/// The ground's collider ([`emit_ground`]), which the welded root links of
/// every robot filter out ([`welded_filter_targets`]).
pub const GROUND_COLLIDER_PATH: &str = "/Environment/GroundCollider";

// FR-102
fn emit_fixture(
    w: &mut impl Write,
    placement: &FixturePlacement,
    cache: &mut MeshCache,
    materials: &mut MaterialRegistry,
    stats: &mut MeshStats,
) -> Result<(), FittingRoomError> {
    let robot = parse_urdf(&placement.urdf)
        .map_err(|e| FittingRoomError::UrdfParse(format!("{}: {}", placement.urdf.display(), e)))?;

    for joint in &robot.joints {
        if joint.joint_type != JointType::Fixed {
            return Err(FittingRoomError::Validation(format!(
                "Fixture '{}' has non-fixed joint '{}' (type: {}); fixtures must use fixed joints only",
                placement.id,
                joint.name,
                joint.joint_type.as_str()
            )));
        }
    }

    let urdf_dir = placement
        .urdf
        .parent()
        .unwrap_or(Path::new("."))
        .to_path_buf();
    let urdf_stem = placement
        .urdf
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("unknown")
        .to_string();
    let fixture_id = sanitize_prim_name(&placement.id);
    let quat = abc_deg_to_quat(placement.orientation_deg);

    // FR-109: each fixture is a component model.
    writeln!(w, "        def Xform \"{}\" (", fixture_id)?;
    writeln!(w, "            kind = \"component\"")?;
    writeln!(w, "        )")?;
    writeln!(w, "        {{")?;
    writeln!(
        w,
        "            double3 xformOp:translate = ({}, {}, {})",
        f(placement.position[0]),
        f(placement.position[1]),
        f(placement.position[2])
    )?;
    writeln!(
        w,
        "            quatd xformOp:orient = ({}, {}, {}, {})",
        f(quat[3]),
        f(quat[0]),
        f(quat[1]),
        f(quat[2])
    )?;
    writeln!(
        w,
        "            uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\"]"
    )?;
    writeln!(w)?;

    // FR-102: every link a static collider at its place (fixtures have only
    // fixed joints, so no home pose), with no rigid body: no world anchor and
    // no joints between the links either, which would only join colliders
    // that can't move.
    let bind_poses = compute_link_bind_poses(&robot, &[]);
    for link in &robot.links {
        let pose = bind_poses
            .get(&link.name)
            .copied()
            .unwrap_or(math3::IDENTITY);
        // Fixtures always emit at full visual fidelity — the lean robot
        // and tool toggles don't apply to fixtures.
        emit_link(
            w,
            link,
            &urdf_dir,
            &urdf_stem,
            cache,
            materials,
            stats,
            12,
            LinkKind::Static,
            &pose,
            /* full_visuals = */ true,
            /* full_tools   = */ true,
        )?;
    }

    writeln!(w, "        }}")?;
    writeln!(w)?;
    Ok(())
}

// FR-119
fn emit_pedestal(
    w: &mut impl Write,
    placement: &RobotPlacement,
    materials: &mut MaterialRegistry,
) -> Result<(), FittingRoomError> {
    // Only called for robots with a positive-height pedestal.
    let ped = placement.pedestal.expect("pedestal present");
    let height_m = ped.height_mm / 1000.0;
    let radius_m = (ped.diameter_mm / 1000.0) / 2.0;
    let half_h = height_m / 2.0;
    let name = pedestal_prim_name(placement);

    // A USD Cylinder is centred on its origin along the axis, so translating
    // the wrapping Xform to z = position.z + height/2 makes the solid span
    // world z∈[position.z, position.z + h] at the robot's XY: from where the
    // robot is placed (the floor at z = 0, or a platform) up to its raised
    // foot, the top face flush under it. Orientation is identity — a vertical
    // cylinder is rotationally symmetric about Z.
    writeln!(w, "        def Xform \"{}\"", name)?;
    writeln!(w, "        {{")?;
    writeln!(
        w,
        "            double3 xformOp:translate = ({}, {}, {})",
        f(placement.position[0]),
        f(placement.position[1]),
        f(placement.position[2] + half_h)
    )?;
    writeln!(w, "            quatd xformOp:orient = (1, 0, 0, 0)")?;
    writeln!(
        w,
        "            uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\"]"
    )?;
    writeln!(w)?;
    // Static collider: PhysicsCollisionAPI without a rigid-body API is
    // immovable in PhysX — exactly what a mounting fundament needs. Matte
    // (the default material), not the renderer's glossy default.
    writeln!(w, "            def Cylinder \"socket\" (")?;
    writeln!(
        w,
        "                prepend apiSchemas = [\"PhysicsCollisionAPI\", \"MaterialBindingAPI\"]"
    )?;
    writeln!(w, "            )")?;
    writeln!(w, "            {{")?;
    writeln!(
        w,
        "                rel material:binding = </World/Looks/{}>",
        materials.default_material()
    )?;
    writeln!(w, "                uniform token axis = \"Z\"")?;
    writeln!(w, "                double height = {}", f(height_m))?;
    writeln!(w, "                double radius = {}", f(radius_m))?;
    writeln!(
        w,
        "                float3[] extent = [({}, {}, {}), ({}, {}, {})]",
        f(-radius_m),
        f(-radius_m),
        f(-half_h),
        f(radius_m),
        f(radius_m),
        f(half_h)
    )?;
    writeln!(w, "            }}")?;
    writeln!(w, "        }}")?;
    writeln!(w)?;
    Ok(())
}

/// FR-119: whether the robot stands on a pedestal (one of positive height).
fn has_pedestal(placement: &RobotPlacement) -> bool {
    placement.pedestal.is_some_and(|p| p.height_mm > 0.0)
}

/// FR-119: the name of the robot's pedestal Xform under `/World/pedestals`.
fn pedestal_prim_name(placement: &RobotPlacement) -> String {
    format!("pedestal_{}", sanitize_prim_name(&placement.robot_id))
}

/// FR-119: the prim path of the robot's pedestal collider, the cylinder
/// [`emit_pedestal`] writes.
fn pedestal_collider_path(placement: &RobotPlacement) -> String {
    format!("/World/pedestals/{}/socket", pedestal_prim_name(placement))
}

/// FR-119: where a robot's foot stands: its placement, raised by its
/// pedestal's height (mm → m) so the base sits on top of the mounting cylinder.
fn raised_position(placement: &RobotPlacement) -> [f64; 3] {
    let raise_m = placement
        .pedestal
        .map(|p| p.height_mm / 1000.0)
        .filter(|h| *h > 0.0)
        .unwrap_or(0.0);
    [
        placement.position[0],
        placement.position[1],
        placement.position[2] + raise_m,
    ]
}

/// A placement as a matrix: translate to `position`, then rotate by the ABC
/// angles as [`abc_deg_to_quat`] does (A about Z, B about Y, C about X).
fn placement_transform(position: [f64; 3], abc_deg: [f64; 3]) -> Mat4 {
    let [a, b, c] = abc_deg.map(f64::to_radians);
    math3::from_translation_rpy(position, [c, b, a])
}

/// FR-108: the extent the camera rig frames; `None` when there is nothing to
/// frame. Robots count with their collisions (meshes and primitives) at the
/// home pose (a link without collisions with its visuals) and their
/// pedestals; fixtures with their visuals and collisions. Neither the floor plan (FR-107)
/// nor the ground (NFR-108), which lies under this box, counts.
fn compute_scene_bounds(
    scene: &SceneDefinition,
    cache: &mut MeshCache,
) -> Result<Option<Aabb>, FittingRoomError> {
    let mut bounds = Aabb::EMPTY;
    for placement in &scene.robots {
        let root = placement_transform(raised_position(placement), placement.orientation_deg);
        let home_pose = placement.home_pose_deg.as_deref();
        add_urdf_bounds(
            &mut bounds,
            &placement.dressed_urdf,
            &root,
            home_pose,
            true,
            cache,
        )?;
        // The pedestal's cylinder spans the robot's placement height to its
        // raised foot.
        if let Some(ped) = placement.pedestal.filter(|p| p.height_mm > 0.0) {
            let height_m = ped.height_mm / 1000.0;
            let radius_m = (ped.diameter_mm / 1000.0) / 2.0;
            let [x, y, z] = placement.position;
            bounds.add_point([x - radius_m, y - radius_m, z]);
            bounds.add_point([x + radius_m, y + radius_m, z + height_m]);
        }
    }
    for placement in &scene.fixtures {
        let root = placement_transform(placement.position, placement.orientation_deg);
        add_urdf_bounds(&mut bounds, &placement.urdf, &root, None, false, cache)?;
    }
    Ok((!bounds.is_empty()).then_some(bounds))
}

/// Adds the geometry of the URDF at `urdf_path` to `bounds`: its root link at
/// `root`, the other links at their bind poses for `home_pose_deg` (see
/// [`drives::actuated_home_values`] and [`compute_link_bind_poses`]). With
/// `hulls`, a link counts with its collisions (meshes and primitives), or its
/// visuals when it has no collision; without, with both.
fn add_urdf_bounds(
    bounds: &mut Aabb,
    urdf_path: &Path,
    root: &Mat4,
    home_pose_deg: Option<&[f64]>,
    hulls: bool,
    cache: &mut MeshCache,
) -> Result<(), FittingRoomError> {
    let robot = parse_urdf(urdf_path)
        .map_err(|e| FittingRoomError::UrdfParse(format!("{}: {}", urdf_path.display(), e)))?;
    let urdf_dir = urdf_path.parent().unwrap_or(Path::new("."));
    let homes = drives::actuated_home_values(&robot, home_pose_deg);
    let bind_poses = compute_link_bind_poses(&robot, &homes);
    for link in &robot.links {
        let bind_pose = bind_poses
            .get(&link.name)
            .copied()
            .unwrap_or(math3::IDENTITY);
        let pose = math3::multiply(root, &bind_pose);
        let visuals = link.visuals.iter().map(|v| (&v.origin, &v.geometry));
        let collisions = link.collisions.iter().map(|c| (&c.origin, &c.geometry));
        let shapes: Vec<(&Origin, &Geometry)> = if !hulls {
            visuals.chain(collisions).collect()
        } else if link.collisions.is_empty() {
            visuals.collect()
        } else {
            collisions.collect()
        };
        for (origin, geometry) in shapes {
            add_geometry_bounds(bounds, &pose, origin, geometry, urdf_dir, cache);
        }
    }
    Ok(())
}

/// Adds a URDF geometry, placed by `link_pose` and its `origin`, to
/// `bounds`: a mesh's vertices, a primitive's bounding-box corners (a box's
/// corners, a cylinder's or a sphere's enclosing box, in their own axes). A
/// mesh that doesn't load is skipped; wherever compose emits it, it fails on
/// it.
fn add_geometry_bounds(
    bounds: &mut Aabb,
    link_pose: &Mat4,
    origin: &Origin,
    geometry: &Geometry,
    urdf_dir: &Path,
    cache: &mut MeshCache,
) {
    let origin_pose = math3::from_translation_rpy(origin.xyz, origin.rpy);
    let pose = math3::multiply(link_pose, &origin_pose);
    let half = match geometry {
        Geometry::Mesh { filename, scale } => {
            let mesh = match cache.load(urdf_dir, filename) {
                Ok(mesh) => mesh,
                Err(e) => {
                    log::warn!("scene bounds: skipping mesh {}: {}", filename, e);
                    return;
                }
            };
            for p in &mesh.points {
                let scaled = [p[0] * scale[0], p[1] * scale[1], p[2] * scale[2]];
                bounds.add_point(math3::transform_point(&pose, scaled));
            }
            return;
        }
        Geometry::Box { size } => math3::scale(*size, 0.5),
        Geometry::Cylinder { radius, length } => [*radius, *radius, *length / 2.0],
        Geometry::Sphere { radius } => [*radius; 3],
    };
    if !half.iter().all(|h| h.is_finite()) {
        return;
    }
    for i in 0..8 {
        let corner = |k: usize| if i & (1 << k) == 0 { -half[k] } else { half[k] };
        bounds.add_point(math3::transform_point(
            &pose,
            [corner(0), corner(1), corner(2)],
        ));
    }
}

/// FR-108: the `/World/Cameras` rig, six cameras in [`camera_rig::views`]
/// order. They share the lens and clipping range; Kit's camera lock keeps
/// viewport navigation from moving them. Each carries RTX's exposure APIs with
/// Kit's own exposure ([`camera_rig::EXPOSURE_F_STOP`] etc., the values Kit
/// syncs onto its viewport camera), so a render product other than the
/// viewport's sees the scene as bright as the viewport does instead of at
/// USD's exposure scale 1 (10 EV brighter).
fn emit_camera_rig(w: &mut impl Write, rig: &CameraRig) -> Result<(), FittingRoomError> {
    let [near, far] = rig.clipping_range;
    writeln!(w, "    def Scope \"Cameras\"")?;
    writeln!(w, "    {{")?;
    for (i, camera) in rig.cameras.iter().enumerate() {
        if i > 0 {
            writeln!(w)?;
        }
        let [qw, qx, qy, qz] = camera.orient_wxyz;
        let [tx, ty, tz] = camera.translate;
        writeln!(w, "        def Camera \"{}\" (", camera.name)?;
        writeln!(
            w,
            "            prepend apiSchemas = [\"OmniRtxCameraAutoExposureAPI_1\", \"OmniRtxCameraExposureAPI_1\"]"
        )?;
        writeln!(w, "        )")?;
        writeln!(w, "        {{")?;
        writeln!(
            w,
            "            float2 clippingRange = ({}, {})",
            f(near),
            f(far)
        )?;
        writeln!(
            w,
            "            float exposure:fStop = {}",
            f_exact(camera_rig::EXPOSURE_F_STOP)
        )?;
        writeln!(
            w,
            "            float exposure:responsivity = {}",
            f_exact(camera_rig::EXPOSURE_RESPONSIVITY)
        )?;
        writeln!(
            w,
            "            float exposure:time = {}",
            f_exact(camera_rig::EXPOSURE_TIME_S)
        )?;
        writeln!(
            w,
            "            float focalLength = {}",
            f(rig.lens.focal_length)
        )?;
        writeln!(
            w,
            "            float horizontalAperture = {}",
            f(rig.lens.horizontal_aperture)
        )?;
        writeln!(
            w,
            "            float verticalAperture = {}",
            f(rig.lens.vertical_aperture)
        )?;
        writeln!(w, "            custom bool omni:kit:cameraLock = 1")?;
        writeln!(
            w,
            "            quatd xformOp:orient = ({}, {}, {}, {})",
            f_exact(qw),
            f_exact(qx),
            f_exact(qy),
            f_exact(qz)
        )?;
        writeln!(
            w,
            "            double3 xformOp:translate = ({}, {}, {})",
            f_exact(tx),
            f_exact(ty),
            f_exact(tz)
        )?;
        writeln!(
            w,
            "            uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\"]"
        )?;
        writeln!(w, "        }}")?;
    }
    writeln!(w, "    }}")?;
    writeln!(w)?;
    Ok(())
}

/// How [`emit_link`] simulates a link.
#[derive(Debug, Clone, Copy)]
enum LinkKind<'a> {
    /// A body of a robot's articulation (FR-110): a rigid body with `mass`
    /// (FR-113), contact reporting and a contact sensor when it has colliders
    /// (FR-160), and `PhysicsFilteredPairsAPI` against the colliders in
    /// `filtered_pairs` (prim paths) when there are any. With `all_tool`
    /// every visual and collision of the link is the tool's (FR-159's named
    /// payload link, [`ToolLayout::all_tool`]); else its `meshes/tool/` ones.
    Articulated {
        mass: &'a MassProps,
        filtered_pairs: &'a [String],
        all_tool: bool,
    },
    /// A static collider (a fixture's, FR-102): its visuals and collision
    /// meshes only, no rigid body, mass, contact report or sensor.
    Static,
}

fn emit_link(
    w: &mut impl Write,
    link: &Link,
    urdf_dir: &Path,
    urdf_stem: &str,
    cache: &mut MeshCache,
    materials: &mut MaterialRegistry,
    stats: &mut MeshStats,
    indent: usize,
    kind: LinkKind<'_>,
    bind_pose: &Mat4,
    full_visuals: bool,
    full_tools: bool,
) -> Result<(), FittingRoomError> {
    let pad = " ".repeat(indent);
    let safe_name = sanitize_prim_name(&link.name);

    // Every articulated link — including the URDF root — must carry
    // PhysicsRigidBodyAPI; PhysX articulation walks bodies-by-link and treats
    // a link without RigidBodyAPI as broken. Fixed-base behaviour is
    // achieved by the world_anchor FixedJoint, not by omitting the API.
    // Every PhysicsRigidBodyAPI prim MUST also carry PhysicsMassAPI with
    // positive mass; otherwise PhysX warns "negative mass / small sphere
    // approximation" every frame and propagates an invalid inertia tensor
    // into the articulation solver. We always apply both schemas, with the
    // mass properties `drives::link_mass_properties` picked.
    //
    // FR-160 / FR-161 / FR-162: when the link has at least one
    // <collision> we ALSO apply PhysxContactReportAPI so PhysX queues
    // contact events for this body. Without it the IsaacReadContactSensor
    // reads empty arrays every frame, its `execOut` never fires, and the
    // ROS2PublishContactSensor downstream never advertises a topic — the
    // bridge silently no-ops. Threshold = 0.0 so every contact reports;
    // any FR-168-style force filtering happens in the cell-aggregator pod
    // (BL-24), not here.
    //
    // FR-102: a static link (a fixture's) is none of this: colliders
    // without a rigid body are static in UsdPhysics.
    let has_collision = !link.collisions.is_empty();
    let articulated = match kind {
        LinkKind::Articulated {
            mass,
            filtered_pairs,
            ..
        } => Some((mass, filtered_pairs)),
        LinkKind::Static => None,
    };
    let all_tool = matches!(kind, LinkKind::Articulated { all_tool: true, .. });
    let is_tool = |geometry: &Geometry| all_tool || tool::is_tool_mesh(geometry);
    if let Some((_, filtered_pairs)) = articulated {
        let mut apis: Vec<&str> = vec!["PhysicsRigidBodyAPI", "PhysicsMassAPI"];
        if has_collision {
            apis.push("PhysxContactReportAPI");
        }
        if !filtered_pairs.is_empty() {
            apis.push("PhysicsFilteredPairsAPI");
        }
        let api_list = apis
            .iter()
            .map(|a| format!("\"{}\"", a))
            .collect::<Vec<_>>()
            .join(", ");
        writeln!(w, "{}def Xform \"{}\" (", pad, safe_name)?;
        writeln!(w, "{}    prepend apiSchemas = [{}]", pad, api_list)?;
        writeln!(w, "{})", pad)?;
    } else {
        writeln!(w, "{}def Xform \"{}\"", pad, safe_name)?;
    }
    writeln!(w, "{}{{", pad)?;

    // Bind-pose as a single matrix4d xformOp:transform — more reliable than
    // split translate+orient when the prim is part of a PhysX articulation.
    emit_xform_transform(w, bind_pose, indent + 4)?;

    if let Some((mass, filtered_pairs)) = articulated {
        // The colliders whose contacts with this body PhysX skips.
        if !filtered_pairs.is_empty() {
            let targets = filtered_pairs
                .iter()
                .map(|path| format!("<{}>", path))
                .collect::<Vec<_>>()
                .join(", ");
            writeln!(w, "{}    rel physics:filteredPairs = [{}]", pad, targets)?;
        }

        // PhysxContactReportAPI threshold attribute. 0.0 = report every
        // contact; anything positive means "only contacts with impulse
        // magnitude above threshold queue events". Keep at 0.0 — filtering
        // is the aggregator's job (FR-168 / BL-24).
        if has_collision {
            writeln!(
                w,
                "{}    float physxContactReport:threshold = 0",
                pad
            )?;
        }

        // FR-113 / FR-159.
        emit_mass_props(w, mass, indent + 4)?;
    }

    // FR-111. Per-visual decision: the tool's visuals (path prefix
    // `meshes/tool/`, or all of a named payload link's, FR-159) are gated by
    // `full_tools`; everything else is gated by `full_visuals`. Joints
    // reference link Xform prims (not the mesh children), so dropping any
    // subset of visuals never breaks the articulation kinematic chain.
    for (i, visual) in link.visuals.iter().enumerate() {
        let keep = if is_tool(&visual.geometry) {
            full_tools
        } else {
            full_visuals
        };
        if keep {
            emit_visual_mesh(
                w, visual, i, urdf_dir, urdf_stem, cache, materials, stats, indent + 4,
            )?;
        }
    }

    // FR-112. Collision hulls are always emitted (they're the physics
    // geometry); their viewport visibility tracks whether the matching
    // visual is present. Per-collision: if it's a tool collision and
    // `full_tools` is OFF, surface it (purpose=default). Otherwise apply
    // the link-wide rule (visible iff no visuals were emitted on this link).
    //
    // We can't perfectly pair a collision with "its" visual without a
    // matching mechanism URDF doesn't provide, so use the same tool rule
    // (the path prefix, or a named payload link's geometry) on the collision.
    // A shown hull wears its link's paint (the material of the link's first non-tool mesh visual),
    // so a lean robot still looks like itself; none: the matte default.
    let link_material = link
        .visuals
        .iter()
        .filter(|v| !is_tool(&v.geometry))
        .find_map(|v| match &v.geometry {
            Geometry::Mesh { filename, .. } => cache
                .load(urdf_dir, filename)
                .ok()
                .and_then(|mesh| materials.register_for_mesh(mesh, &urdf_dir.join(filename), urdf_stem)),
            _ => None,
        });
    for (i, collision) in link.collisions.iter().enumerate() {
        let visible = if is_tool(&collision.geometry) {
            !full_tools
        } else {
            !full_visuals
        };
        emit_collision_mesh(
            w,
            collision,
            i,
            urdf_dir,
            cache,
            materials,
            link_material.as_deref(),
            stats,
            indent + 4,
            visible,
        )?;
    }

    // FR-160 / FR-161 / FR-162. When the link has any collision geometry,
    // emit an `IsaacContactSensor` child prim. The OmniGraph
    // `IsaacReadContactSensor` node's `inputs:csPrim` rel must point at
    // THIS sensor prim (not the link Xform); without it Isaac warns
    // "no valid sensor reading, is the sensor enabled?" and the bridge
    // never publishes. Sensor name is fixed (`ContactSensor_node`) so
    // the OmniGraph emission can build a deterministic path. A static link
    // reports no contacts, so it gets none (and no graph reads one).
    if has_collision && articulated.is_some() {
        emit_link_contact_sensor_prim(w, indent + 4)?;
    }

    writeln!(w, "{}}}", pad)?;
    writeln!(w)?;
    Ok(())
}

/// FR-160. Emit a per-link `IsaacContactSensor` child prim. Lives at
/// `<link>/ContactSensor_node` so the OmniGraph `csPrim` rel resolves
/// deterministically. `radius = -1` means the sensor reports any contact
/// on the parent body (no spatial filter); `enabled = 1` is required —
/// Isaac defaults to disabled and silently drops events otherwise.
fn emit_link_contact_sensor_prim(
    w: &mut impl Write,
    indent: usize,
) -> Result<(), FittingRoomError> {
    let pad = " ".repeat(indent);
    writeln!(w, "{}def IsaacContactSensor \"ContactSensor_node\"", pad)?;
    writeln!(w, "{}{{", pad)?;
    writeln!(w, "{}    bool enabled = 1", pad)?;
    // -1 = "use parent rigid body's full extent", any contact reports.
    writeln!(w, "{}    float radius = -1", pad)?;
    // 0 = no minimum threshold; filtering is the aggregator's job (BL-24, FR-168).
    writeln!(w, "{}    float minThreshold = 0", pad)?;
    writeln!(w, "{}    float maxThreshold = 100000", pad)?;
    // Sensor's local pose at the link origin — position 0/0/0 is fine,
    // PhysX measures contacts on the parent body, the sensor just reads.
    writeln!(w, "{}    float3 translate = (0, 0, 0)", pad)?;
    writeln!(w, "{}}}", pad)?;
    writeln!(w)?;
    Ok(())
}

/// FR-113 / FR-159: a link's `PhysicsMassAPI` attributes
/// ([`drives::link_mass_properties`]). `physics:principalAxes` comes with
/// every `physics:diagonalInertia` (UsdPhysics wants both or neither), the
/// identity (1, 0, 0, 0) when the tensor is diagonal in the link's axes.
fn emit_mass_props(
    w: &mut impl Write,
    props: &MassProps,
    indent: usize,
) -> Result<(), FittingRoomError> {
    let pad = " ".repeat(indent);
    let [cx, cy, cz] = props.center_of_mass;
    let [ix, iy, iz] = props.diagonal_inertia;
    writeln!(w, "{}float physics:mass = {}", pad, f(props.mass))?;
    writeln!(
        w,
        "{}point3f physics:centerOfMass = ({}, {}, {})",
        pad,
        f(cx),
        f(cy),
        f(cz)
    )?;
    writeln!(
        w,
        "{}float3 physics:diagonalInertia = ({}, {}, {})",
        pad,
        f(ix),
        f(iy),
        f(iz)
    )?;
    let [qw, qx, qy, qz] = props.principal_axes_wxyz();
    writeln!(
        w,
        "{}quatf physics:principalAxes = ({}, {}, {}, {})",
        pad,
        f(qw),
        f(qx),
        f(qy),
        f(qz)
    )?;
    Ok(())
}

fn emit_visual_mesh(
    w: &mut impl Write,
    visual: &Visual,
    index: usize,
    urdf_dir: &Path,
    urdf_stem: &str,
    cache: &mut MeshCache,
    materials: &mut MaterialRegistry,
    stats: &mut MeshStats,
    indent: usize,
) -> Result<(), FittingRoomError> {
    let pad = " ".repeat(indent);

    // Resolve material before opening the prim so we can attach MaterialBindingAPI.
    let material_prim_name = if let Geometry::Mesh { filename, .. } = &visual.geometry {
        let obj_path = urdf_dir.join(filename);
        let mesh = cache.load(urdf_dir, filename)?;
        materials.register_for_mesh(mesh, &obj_path, urdf_stem)
    } else {
        None
    };
    let material_prim_name = material_prim_name.unwrap_or_else(|| materials.default_material());

    // A mesh is a Mesh prim; a URDF box, cylinder or sphere its UsdGeom
    // primitive (FR-111).
    writeln!(
        w,
        "{}def {} \"visual_{}\" (",
        pad,
        geometry_prim_type(&visual.geometry),
        index
    )?;
    writeln!(
        w,
        "{}    prepend apiSchemas = [\"MaterialBindingAPI\"]",
        pad
    )?;
    writeln!(w, "{})", pad)?;
    writeln!(w, "{}{{", pad)?;
    writeln!(
        w,
        "{}    rel material:binding = </World/Looks/{}>",
        pad, material_prim_name
    )?;
    emit_geometry_inline(w, &visual.geometry, urdf_dir, cache, stats, indent + 4)?;
    emit_origin_xform(
        w,
        &visual.origin,
        primitive_scale(&visual.geometry),
        indent + 4,
    )?;
    writeln!(w, "{}}}", pad)?;
    Ok(())
}

fn emit_collision_mesh(
    w: &mut impl Write,
    collision: &Collision,
    index: usize,
    urdf_dir: &Path,
    cache: &mut MeshCache,
    materials: &mut MaterialRegistry,
    link_material: Option<&str>,
    stats: &mut MeshStats,
    indent: usize,
    visible: bool,
) -> Result<(), FittingRoomError> {
    let pad = " ".repeat(indent);
    // FR-112: a mesh is a convex-hull Mesh collider; a URDF box, cylinder or
    // sphere its UsdGeom primitive, which PhysX collides with as it is.
    let is_mesh = matches!(collision.geometry, Geometry::Mesh { .. });
    writeln!(
        w,
        "{}def {} \"collision_{}\" (",
        pad,
        geometry_prim_type(&collision.geometry),
        index
    )?;
    // A shown hull (a lean robot's) is matte, not the renderer's glossy default.
    let mut schemas = vec!["\"PhysicsCollisionAPI\""];
    if is_mesh {
        schemas.push("\"PhysicsMeshCollisionAPI\"");
    }
    if visible {
        schemas.push("\"MaterialBindingAPI\"");
    }
    writeln!(
        w,
        "{}    prepend apiSchemas = [{}]",
        pad,
        schemas.join(", ")
    )?;
    writeln!(w, "{})", pad)?;
    writeln!(w, "{}{{", pad)?;
    if visible {
        let material = link_material.map_or_else(|| materials.default_material(), str::to_owned);
        writeln!(w, "{}    rel material:binding = </World/Looks/{}>", pad, material)?;
    }
    // purpose: "guide" hides the hull behind the visual mesh; "default"
    // keeps it visible. Lean robots want hulls visible; full robots
    // (`--full-robots`) and fixtures hide them so the visual mesh wins the draw.
    if !visible {
        writeln!(w, "{}    uniform token purpose = \"guide\"", pad)?;
    }
    if is_mesh {
        writeln!(
            w,
            "{}    uniform token physics:approximation = \"convexHull\"",
            pad
        )?;
    }
    emit_geometry_inline(w, &collision.geometry, urdf_dir, cache, stats, indent + 4)?;
    emit_origin_xform(
        w,
        &collision.origin,
        primitive_scale(&collision.geometry),
        indent + 4,
    )?;
    writeln!(w, "{}}}", pad)?;
    Ok(())
}

/// The USD prim type a URDF geometry is emitted as: a mesh's `Mesh`, a box's
/// `Cube`, a cylinder's `Cylinder`, a sphere's `Sphere`.
fn geometry_prim_type(geometry: &Geometry) -> &'static str {
    match geometry {
        Geometry::Mesh { .. } => "Mesh",
        Geometry::Box { .. } => "Cube",
        Geometry::Cylinder { .. } => "Cylinder",
        Geometry::Sphere { .. } => "Sphere",
    }
}

/// The `xformOp:scale` a geometry's prim needs: a box's size (its `Cube` is
/// the unit cube), none for the others.
fn primitive_scale(geometry: &Geometry) -> Option<[f64; 3]> {
    match geometry {
        Geometry::Box { size } => Some(*size),
        _ => None,
    }
}

/// Emit USDA mesh attributes inline. For URDF Mesh geometry, parses the OBJ
/// and emits `points` / `faceVertexIndices` / `faceVertexCounts` directly so
/// Isaac Sim doesn't need to invoke the asset converter at load time.
fn emit_geometry_inline(
    w: &mut impl Write,
    geometry: &Geometry,
    urdf_dir: &Path,
    cache: &mut MeshCache,
    stats: &mut MeshStats,
    indent: usize,
) -> Result<(), FittingRoomError> {
    let pad = " ".repeat(indent);
    match geometry {
        Geometry::Mesh { filename, scale } => {
            let mesh = cache.load(urdf_dir, filename)?;
            let scale = *scale;
            stats.mesh_count += 1;
            // Polygons as they are: USD's default is Catmull-Clark subdivision, which an RTX renderer
            // (Isaac Sim) applies, rounding a box into a pillow.
            writeln!(w, "{}uniform token subdivisionScheme = \"none\"", pad)?;
            stats.total_vertices += mesh.points.len();
            stats.total_faces += mesh.face_vertex_counts.len();

            // points
            write!(w, "{}point3f[] points = [", pad)?;
            for (i, p) in mesh.points.iter().enumerate() {
                if i > 0 {
                    write!(w, ", ")?;
                }
                write!(
                    w,
                    "({}, {}, {})",
                    f(p[0] * scale[0]),
                    f(p[1] * scale[1]),
                    f(p[2] * scale[2])
                )?;
            }
            writeln!(w, "]")?;

            // faceVertexIndices
            write!(w, "{}int[] faceVertexIndices = [", pad)?;
            for (i, idx) in mesh.face_vertex_indices.iter().enumerate() {
                if i > 0 {
                    write!(w, ", ")?;
                }
                write!(w, "{}", idx)?;
            }
            writeln!(w, "]")?;

            // faceVertexCounts
            write!(w, "{}int[] faceVertexCounts = [", pad)?;
            for (i, c) in mesh.face_vertex_counts.iter().enumerate() {
                if i > 0 {
                    write!(w, ", ")?;
                }
                write!(w, "{}", c)?;
            }
            writeln!(w, "]")?;
        }
        // A URDF box: the unit `Cube`, scaled to its size by the prim's
        // `xformOp:scale` (`primitive_scale`).
        Geometry::Box { .. } => {
            writeln!(w, "{}double size = 1", pad)?;
            writeln!(
                w,
                "{}float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]",
                pad
            )?;
        }
        // A URDF cylinder runs along its Z, centred, as a USD `Cylinder` with
        // axis Z does.
        Geometry::Cylinder { radius, length } => {
            let (r, h) = (*radius, *length / 2.0);
            writeln!(w, "{}uniform token axis = \"Z\"", pad)?;
            writeln!(w, "{}double height = {}", pad, f(*length))?;
            writeln!(w, "{}double radius = {}", pad, f(r))?;
            writeln!(
                w,
                "{}float3[] extent = [({}, {}, {}), ({}, {}, {})]",
                pad,
                f(-r),
                f(-r),
                f(-h),
                f(r),
                f(r),
                f(h)
            )?;
        }
        Geometry::Sphere { radius } => {
            let r = *radius;
            writeln!(w, "{}double radius = {}", pad, f(r))?;
            writeln!(
                w,
                "{}float3[] extent = [({}, {}, {}), ({}, {}, {})]",
                pad,
                f(-r),
                f(-r),
                f(-r),
                f(r),
                f(r),
                f(r)
            )?;
        }
    }
    Ok(())
}

/// Emit a `matrix4d xformOp:transform` attribute for a 4x4 pose matrix.
/// This single-matrix form is preferred for PhysX articulation links because
/// some renderers ignore split translate/orient on articulated bodies.
fn emit_xform_transform(
    w: &mut impl Write,
    m: &Mat4,
    indent: usize,
) -> Result<(), FittingRoomError> {
    let pad = " ".repeat(indent);
    // USD `matrix4d` is column-major in the type sense but USDA writes it
    // as row vectors of (col, col, col, col). The transpose is taken so the
    // emitted text matches the convention Isaac/Hydra parses correctly.
    writeln!(
        w,
        "{}matrix4d xformOp:transform = ( ({}, {}, {}, {}), ({}, {}, {}, {}), ({}, {}, {}, {}), ({}, {}, {}, {}) )",
        pad,
        f(m[0][0]), f(m[1][0]), f(m[2][0]), f(m[3][0]),
        f(m[0][1]), f(m[1][1]), f(m[2][1]), f(m[3][1]),
        f(m[0][2]), f(m[1][2]), f(m[2][2]), f(m[3][2]),
        f(m[0][3]), f(m[1][3]), f(m[2][3]), f(m[3][3]),
    )?;
    writeln!(
        w,
        "{}uniform token[] xformOpOrder = [\"xformOp:transform\"]",
        pad
    )?;
    Ok(())
}

/// A visual's or collision's `origin` as `xformOp:translate` and
/// `xformOp:orient`, then `scale` (a box's size, [`primitive_scale`]) as
/// `xformOp:scale` when given.
fn emit_origin_xform(
    w: &mut impl Write,
    origin: &Origin,
    scale: Option<[f64; 3]>,
    indent: usize,
) -> Result<(), FittingRoomError> {
    let pad = " ".repeat(indent);
    let quat = rpy_rad_to_quat(origin.rpy);
    writeln!(
        w,
        "{}double3 xformOp:translate = ({}, {}, {})",
        pad,
        f(origin.xyz[0]),
        f(origin.xyz[1]),
        f(origin.xyz[2])
    )?;
    writeln!(
        w,
        "{}quatd xformOp:orient = ({}, {}, {}, {})",
        pad,
        f(quat[3]),
        f(quat[0]),
        f(quat[1]),
        f(quat[2])
    )?;
    match scale {
        Some([sx, sy, sz]) => {
            writeln!(
                w,
                "{}double3 xformOp:scale = ({}, {}, {})",
                pad,
                f(sx),
                f(sy),
                f(sz)
            )?;
            writeln!(
                w,
                "{}uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\", \"xformOp:scale\"]",
                pad
            )?;
        }
        None => writeln!(
            w,
            "{}uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\"]",
            pad
        )?,
    }
    Ok(())
}

/// A URDF joint as a UsdPhysics joint between its links' prims. An actuated
/// joint gets its `drive` ([`drives::drive_params`]): FR-158's drive and limits
/// and FR-157's initial joint state.
fn emit_joint(
    w: &mut impl Write,
    joint: &Joint,
    robot_id: &str,
    indent: usize,
    drive: Option<&DriveParams>,
) -> Result<(), FittingRoomError> {
    let pad = " ".repeat(indent);
    let safe_name = sanitize_prim_name(&joint.name);
    let parent = sanitize_prim_name(&joint.parent);
    let child = sanitize_prim_name(&joint.child);

    let prim_type = match joint.joint_type {
        JointType::Revolute | JointType::Continuous => "PhysicsRevoluteJoint",
        JointType::Prismatic => "PhysicsPrismaticJoint",
        JointType::Fixed => "PhysicsFixedJoint",
        JointType::Floating | JointType::Planar => {
            log::warn!(
                "Joint '{}' has unsupported type '{}'; skipping",
                joint.name,
                joint.joint_type.as_str()
            );
            return Ok(());
        }
    };

    // Actuated joints get `PhysicsDriveAPI:<type>` so Isaac/PhysX exposes a
    // position target via the articulation. Without it the joint is "free"
    // and only moves under gravity — there's no UI handle to drive it.
    // FR-157: `PhysicsJointStateAPI:<type>` next to it sets the joint's
    // initial position and velocity; without it PhysX starts every joint at
    // 0 and overwrites the authored link poses. FR-158: `PhysxJointAPI`
    // carries the joint's velocity limit.
    let drive_token = match joint.joint_type {
        JointType::Revolute | JointType::Continuous => Some("angular"),
        JointType::Prismatic => Some("linear"),
        _ => None,
    };
    let drive = drive_token.zip(drive);
    if let Some((tok, drive)) = drive {
        let mut schemas = vec![
            format!("PhysicsDriveAPI:{}", tok),
            format!("PhysicsJointStateAPI:{}", tok),
        ];
        if drive.max_joint_velocity.is_some() {
            schemas.push("PhysxJointAPI".to_owned());
        }
        let schemas = schemas
            .iter()
            .map(|s| format!("\"{}\"", s))
            .collect::<Vec<_>>()
            .join(", ");
        writeln!(w, "{}def {} \"{}\" (", pad, prim_type, safe_name)?;
        writeln!(w, "{}    prepend apiSchemas = [{}]", pad, schemas)?;
        writeln!(w, "{})", pad)?;
    } else {
        writeln!(w, "{}def {} \"{}\"", pad, prim_type, safe_name)?;
    }
    writeln!(w, "{}{{", pad)?;
    writeln!(
        w,
        "{}    rel physics:body0 = </World/robots/{}/{}>",
        pad, robot_id, parent
    )?;
    writeln!(
        w,
        "{}    rel physics:body1 = </World/robots/{}/{}>",
        pad, robot_id, child
    )?;

    // FR-157: the joint's axis and frames. An axis along +X, +Y or +Z is that
    // token with the joint frames the URDF's (body0's the joint origin,
    // body1's the child's origin); any other — a negative or skewed one — is
    // folded into both frames, which turn +X onto it (`JointAxis::frame`),
    // with the token X. Either way the joint's angle (or slide) is the URDF's,
    // sign included, so the limits, drive target and joint state below are
    // written in the URDF's convention and match the link bind poses. A fixed
    // joint has no axis.
    let axis = matches!(
        joint.joint_type,
        JointType::Revolute | JointType::Continuous | JointType::Prismatic
    )
    .then(|| joint_axis(joint));
    let frame = axis.as_ref().and_then(|a| a.frame.as_ref());
    if let Some(axis) = &axis {
        writeln!(
            w,
            "{}    uniform token physics:axis = \"{}\"",
            pad, axis.token
        )?;

        if matches!(joint.joint_type, JointType::Revolute | JointType::Prismatic) {
            if let Some(limit) = &joint.limit {
                // PhysX/USD: revolute limits in degrees, prismatic in metres
                let (lower, upper) = match joint.joint_type {
                    JointType::Revolute => (limit.lower.to_degrees(), limit.upper.to_degrees()),
                    _ => (limit.lower, limit.upper),
                };
                writeln!(w, "{}    float physics:lowerLimit = {}", pad, f(lower))?;
                writeln!(w, "{}    float physics:upperLimit = {}", pad, f(upper))?;
            }
        }

        // FR-158: the drive. Stiffness 1.5e5 per degree (8.6e6 N·m/rad: PhysX
        // solves the drive implicitly, so it doesn't bound the step); damping
        // for a damping ratio of 0.7 on the composite inertia the joint moves
        // at home; maxForce the URDF effort limit, or a derived cap above
        // nominal tracking where the URDF gives a placeholder.
        if let Some((tok, drive)) = drive {
            writeln!(
                w,
                "{}    float drive:{}:physics:stiffness = {}",
                pad,
                tok,
                f(drive.stiffness)
            )?;
            writeln!(
                w,
                "{}    float drive:{}:physics:damping = {}",
                pad,
                tok,
                f(drive.damping)
            )?;
            writeln!(
                w,
                "{}    float drive:{}:physics:maxForce = {}",
                pad,
                tok,
                f(drive.max_force)
            )?;
            // FR-157: the target and the initial state are both the home
            // value (`drives::actuated_home_values`: degrees, unwrapped on a
            // revolute joint; metres on a prismatic one), the very text, so
            // the joint starts at rest on its target. The link bind poses
            // show the same pose but don't set PhysX's start state; this
            // state does.
            let home = f(drive.home);
            writeln!(
                w,
                "{}    float drive:{}:physics:targetPosition = {}",
                pad, tok, home
            )?;
            writeln!(
                w,
                "{}    float state:{}:physics:position = {}",
                pad, tok, home
            )?;
            writeln!(w, "{}    float state:{}:physics:velocity = 0", pad, tok)?;
            // FR-158: the URDF velocity limit, deg/s (m/s on a prismatic joint).
            if let Some(velocity) = drive.max_joint_velocity {
                writeln!(
                    w,
                    "{}    float physxJoint:maxJointVelocity = {}",
                    pad,
                    f(velocity)
                )?;
            }
        }
    }

    let origin = &joint.origin;
    if origin.xyz != [0.0; 3] || origin.rpy != [0.0; 3] || frame.is_some() {
        let rot0 = match frame {
            // The joint origin's rotation, then the turn of +X onto the axis.
            Some(frame) => math3::quat_of(&math3::multiply(
                &math3::from_translation_rpy([0.0; 3], origin.rpy),
                frame,
            )),
            None => rpy_rad_to_quat(origin.rpy),
        };
        writeln!(
            w,
            "{}    point3f physics:localPos0 = ({}, {}, {})",
            pad,
            f(origin.xyz[0]),
            f(origin.xyz[1]),
            f(origin.xyz[2])
        )?;
        writeln!(
            w,
            "{}    quatf physics:localRot0 = ({}, {}, {}, {})",
            pad,
            f(rot0[3]),
            f(rot0[0]),
            f(rot0[1]),
            f(rot0[2])
        )?;
    }
    if let Some(frame) = frame {
        let rot1 = math3::quat_of(frame);
        writeln!(w, "{}    point3f physics:localPos1 = (0, 0, 0)", pad)?;
        writeln!(
            w,
            "{}    quatf physics:localRot1 = ({}, {}, {}, {})",
            pad,
            f(rot1[3]),
            f(rot1[0]),
            f(rot1[1]),
            f(rot1[2])
        )?;
    }

    writeln!(w, "{}}}", pad)?;
    writeln!(w)?;
    Ok(())
}

/// FR-157: how a joint's URDF axis is written ([`joint_axis`]).
#[derive(Debug, Clone, PartialEq)]
struct JointAxis {
    /// `physics:axis`: the joint frames' axis the joint turns about (slides
    /// along).
    token: &'static str,
    /// For an axis other than +X, +Y and +Z: the rotation taking +X onto the
    /// unit URDF axis, folded into both joint frames (`localRot0` = the
    /// origin's rotation · it, `localRot1` = it), so the joint frames' +X is
    /// the URDF axis in both bodies.
    frame: Option<Mat4>,
}

/// FR-157: `joint`'s axis as UsdPhysics takes it: the token of a URDF axis
/// along +X, +Y or +Z (within 1e-9 per component, after normalising), else
/// the token X and the rotation folding the axis into the joint frames. A zero
/// axis (rejected by validation on an actuated joint) is URDF's default, +X.
fn joint_axis(joint: &Joint) -> JointAxis {
    let axis = drives::unit_joint_axis(joint.axis);
    let along = |unit: [f64; 3]| (0..3).all(|k| (axis[k] - unit[k]).abs() <= 1e-9);
    for (token, unit) in [
        ("X", [1.0, 0.0, 0.0]),
        ("Y", [0.0, 1.0, 0.0]),
        ("Z", [0.0, 0.0, 1.0]),
    ] {
        if along(unit) {
            return JointAxis { token, frame: None };
        }
    }
    JointAxis {
        token: "X",
        frame: Some(math3::rotation_taking_x_to(axis)),
    }
}

fn emit_materials(
    w: &mut impl Write,
    registry: &MaterialRegistry,
) -> Result<(), FittingRoomError> {
    if registry.materials.is_empty() {
        return Ok(());
    }
    writeln!(w, "    def Scope \"Looks\"")?;
    writeln!(w, "    {{")?;
    for (name, mat) in &registry.materials {
        writeln!(w, "        def Material \"{}\"", name)?;
        writeln!(w, "        {{")?;
        writeln!(
            w,
            "            token outputs:surface.connect = </World/Looks/{}/Shader.outputs:surface>",
            name
        )?;
        writeln!(w)?;
        writeln!(w, "            def Shader \"Shader\"")?;
        writeln!(w, "            {{")?;
        writeln!(w, "                uniform token info:id = \"UsdPreviewSurface\"")?;
        let [r, g, b] = mat.diffuse_linear();
        writeln!(
            w,
            "                color3f inputs:diffuseColor = ({}, {}, {})",
            f(r),
            f(g),
            f(b)
        )?;
        writeln!(
            w,
            "                float inputs:metallic = {}",
            f(mat.metallic())
        )?;
        writeln!(
            w,
            "                float inputs:roughness = {}",
            f(mat.roughness())
        )?;
        writeln!(
            w,
            "                float inputs:opacity = {}",
            f(mat.opacity)
        )?;
        writeln!(w, "                token outputs:surface")?;
        writeln!(w, "            }}")?;
        writeln!(w, "        }}")?;
        writeln!(w)?;
    }
    writeln!(w, "    }}")?;
    writeln!(w)?;
    Ok(())
}

// FR-150, FR-151, FR-152, FR-153, FR-160, FR-161, FR-162
//
// Emit OmniGraph ActionGraphs for ROS2 control of every robot, plus a
// scene-level /clock publisher. Each robot gets:
//   * ROS2PublishJointState on `/<cell_id>/<robot_id>/joint_states`
//   * ROS2SubscribeJointState on `/<cell_id>/<robot_id>/joint_command`
//   * ArticulationController fed from the subscribed targets
//   * IsaacReadSimulationTime providing timestamps
//   * One IsaacReadContactSensor + ROS2PublishContactSensor pair per
//     collision-bearing link (FR-160/161/162). The publisher's execIn is
//     wired to the sensor's execOut, NOT OnPlaybackTick — so the topic is
//     silent until a contact event lands in the current frame.
//
// Topic schema is two-segment: `/<cell_id>/<robot_id>/<channel>`. The
// `cell_id` is the manifest's `name` field (sanitised the same way
// `robot_id` is). See `ros_namespace`.
fn emit_action_graphs(
    w: &mut impl Write,
    cell_id: &str,
    robots: &[RobotPlacement],
) -> Result<(), FittingRoomError> {
    if robots.is_empty() {
        return Ok(());
    }

    let cell_ns = sanitize_ros_segment(cell_id);
    emit_clock_action_graph(w, &cell_ns)?;

    for placement in robots {
        let robot = match parse_urdf(&placement.dressed_urdf) {
            Ok(r) => r,
            Err(e) => {
                log::warn!(
                    "Skipping ActionGraph for {}: URDF parse failed: {}",
                    placement.robot_id,
                    e
                );
                continue;
            }
        };
        let joint_names: Vec<String> = robot
            .joints
            .iter()
            .filter(|j| !matches!(j.joint_type, JointType::Fixed))
            .map(|j| j.name.clone())
            .collect();
        if joint_names.is_empty() {
            continue;
        }
        let collision_links = collect_collision_link_names(&robot);
        emit_robot_action_graph(
            w,
            &cell_ns,
            &placement.robot_id,
            &joint_names,
            &collision_links,
        )?;
    }
    Ok(())
}

/// Sanitise one segment of a ROS 2 namespace. The output is constrained
/// to match the rmw token regex `[A-Za-z_][A-Za-z0-9_]*` so the emitted
/// USDA validates equally under FastDDS (Isaac default) **and**
/// `rmw_cyclonedds_cpp` (the Cyclone DDS RMW), per
/// `rmw/validate_namespace.h` and `rmw/validate_topic_name.h`. Rules:
///   * Digit-leading raw values get an `r_` prefix (e.g. `1234R01` →
///     `r_1234R01`), preserving every original character.
///   * Non-alphanumeric characters become `_`.
///   * If the result is empty (e.g. caller passed a blank string),
///     fall back to `r_anon` so we never emit an empty segment.
/// Used for both the `cell_id` and the `robot_id` segments of the
/// `<cell_id>/<robot_id>/<channel>` topic schema. Idempotent — calling it
/// twice on the same input produces the same output. Multi-segment
/// composition is the caller's job (see `ros_namespace`); each segment
/// is validated independently so the resulting full path passes both
/// rmw validators end-to-end.
fn sanitize_ros_segment(segment: &str) -> String {
    let mut out = String::with_capacity(segment.len() + 2);
    let needs_prefix = segment
        .chars()
        .next()
        .map(|c| c.is_ascii_digit())
        .unwrap_or(true);
    if needs_prefix {
        out.push_str("r_");
    }
    for c in segment.chars() {
        if c.is_ascii_alphanumeric() || c == '_' {
            out.push(c);
        } else {
            out.push('_');
        }
    }
    if out.is_empty() {
        return "r_anon".to_string();
    }
    out
}

/// Compose the two-segment ROS 2 namespace `<cell_id>/<robot_id>` from the
/// raw manifest values. Each segment passes through `sanitize_ros_segment`.
fn ros_namespace(cell_id: &str, robot_id: &str) -> String {
    format!(
        "{}/{}",
        sanitize_ros_segment(cell_id),
        sanitize_ros_segment(robot_id)
    )
}

/// FR-162. Sanitised names of links carrying at least one `<collision>`
/// block. Derived purely from the URDF — no manifest knob.
fn collect_collision_link_names(robot: &UrdfRobot) -> Vec<String> {
    robot
        .links
        .iter()
        .filter(|link| !link.collisions.is_empty())
        .map(|link| sanitize_prim_name(&link.name))
        .collect()
}

// OmniGraph emission notes (Isaac 5.1):
// - The graph prim is `def OmniGraph "...."` (not a plain `def` with
//   apiSchemas = ["OmniGraphAPI"]). Plain defs with the API schema can
//   load as half-instantiated graphs whose node rels resolve to -1
//   handles, causing access-violation crashes when the bridge ticks.
// - Attribute connections (`.connect`) must use `add` list-edit op so
//   the OmniGraph rel walker finds them. `prepend` and the default
//   (un-listed) form are both unreliable across Kit minor versions.
// - Rel-typed inputs like `inputs:targetPrim` must be declared once with
//   their target via `add rel <name> = <path>` — declaring `custom rel`
//   first and then `prepend rel ... = ...` separately leaves the bridge
//   resolving the first (empty) declaration, returning -1.

fn emit_clock_action_graph(
    w: &mut impl Write,
    cell_ns: &str,
) -> Result<(), FittingRoomError> {
    writeln!(w, "    def OmniGraph \"ActionGraph_clock\"")?;
    writeln!(w, "    {{")?;
    writeln!(w, "        token evaluationMode = \"Automatic\"")?;
    writeln!(w, "        token evaluator:type = \"execution\"")?;
    writeln!(w, "        token fabricCacheBacking = \"Shared\"")?;
    writeln!(w, "        int2 fileFormatVersion = (1, 9)")?;
    writeln!(w, "        token pipelineStage = \"pipelineStageSimulation\"")?;
    writeln!(w)?;
    writeln!(w, "        def OmniGraphNode \"OnPlaybackTick\"")?;
    writeln!(w, "        {{")?;
    writeln!(
        w,
        "            token node:type = \"omni.graph.action.OnPlaybackTick\""
    )?;
    writeln!(w, "            int node:typeVersion = 1")?;
    writeln!(w, "            custom uint outputs:tick (")?;
    writeln!(w, "                customData = {{ bool isExecution = 1 }}")?;
    writeln!(w, "            )")?;
    writeln!(w, "        }}")?;
    writeln!(w)?;
    writeln!(w, "        def OmniGraphNode \"IsaacReadSimulationTime\"")?;
    writeln!(w, "        {{")?;
    writeln!(
        w,
        "            token node:type = \"isaacsim.core.nodes.IsaacReadSimulationTime\""
    )?;
    writeln!(w, "            int node:typeVersion = 1")?;
    writeln!(w, "            custom bool inputs:resetOnStop = 0")?;
    writeln!(w, "            custom double outputs:simulationTime")?;
    writeln!(w, "        }}")?;
    writeln!(w)?;
    writeln!(w, "        def OmniGraphNode \"ROS2PublishClock\"")?;
    writeln!(w, "        {{")?;
    writeln!(
        w,
        "            token node:type = \"isaacsim.ros2.bridge.ROS2PublishClock\""
    )?;
    writeln!(w, "            int node:typeVersion = 1")?;
    writeln!(w, "            custom uint inputs:execIn (")?;
    writeln!(w, "                customData = {{ bool isExecution = 1 }}")?;
    writeln!(w, "            )")?;
    writeln!(
        w,
        "            add uint inputs:execIn.connect = </World/ActionGraph_clock/OnPlaybackTick.outputs:tick>"
    )?;
    // FR-152: clock topic is cell-scoped (`/<cell_id>/clock`) so multiple
    // cells sharing one ROS_DOMAIN_ID don't collide on `/clock`.
    writeln!(w, "            custom string inputs:nodeNamespace = \"{}\"", cell_ns)?;
    writeln!(w, "            custom string inputs:topicName = \"clock\"")?;
    writeln!(w, "            custom double inputs:timeStamp = 0")?;
    writeln!(
        w,
        "            add double inputs:timeStamp.connect = </World/ActionGraph_clock/IsaacReadSimulationTime.outputs:simulationTime>"
    )?;
    writeln!(w, "        }}")?;
    writeln!(w, "    }}")?;
    writeln!(w)?;
    Ok(())
}

fn emit_robot_action_graph(
    w: &mut impl Write,
    cell_ns: &str,
    robot_id: &str,
    joint_names: &[String],
    collision_link_names: &[String],
) -> Result<(), FittingRoomError> {
    let safe_id = sanitize_prim_name(robot_id);
    let graph_name = format!("ActionGraph_{}", safe_id);
    let articulation_path = format!("/World/robots/{}", safe_id);
    // FR-151: two-segment namespace `<cell_id>/<robot_id>`. Both segments
    // are sanitised through `sanitize_ros_segment` (digit-leading values
    // get an `r_` prefix; everything else is alphanumerics + underscores).
    let ns = ros_namespace(cell_ns, robot_id);
    let ns = ns.as_str();

    let joint_token_array = joint_names
        .iter()
        .map(|j| format!("\"{}\"", j))
        .collect::<Vec<_>>()
        .join(", ");

    writeln!(w, "    def OmniGraph \"{}\"", graph_name)?;
    writeln!(w, "    {{")?;
    writeln!(w, "        token evaluationMode = \"Automatic\"")?;
    writeln!(w, "        token evaluator:type = \"execution\"")?;
    writeln!(w, "        token fabricCacheBacking = \"Shared\"")?;
    writeln!(w, "        int2 fileFormatVersion = (1, 9)")?;
    writeln!(w, "        token pipelineStage = \"pipelineStageSimulation\"")?;
    writeln!(w)?;

    // OnPlaybackTick — drives the per-frame execution chain.
    writeln!(w, "        def OmniGraphNode \"OnPlaybackTick\"")?;
    writeln!(w, "        {{")?;
    writeln!(
        w,
        "            token node:type = \"omni.graph.action.OnPlaybackTick\""
    )?;
    writeln!(w, "            int node:typeVersion = 1")?;
    writeln!(w, "            custom uint outputs:tick (")?;
    writeln!(w, "                customData = {{ bool isExecution = 1 }}")?;
    writeln!(w, "            )")?;
    writeln!(w, "        }}")?;
    writeln!(w)?;

    writeln!(w, "        def OmniGraphNode \"IsaacReadSimulationTime\"")?;
    writeln!(w, "        {{")?;
    writeln!(
        w,
        "            token node:type = \"isaacsim.core.nodes.IsaacReadSimulationTime\""
    )?;
    writeln!(w, "            int node:typeVersion = 1")?;
    writeln!(w, "            custom bool inputs:resetOnStop = 0")?;
    writeln!(w, "            custom double outputs:simulationTime")?;
    writeln!(w, "        }}")?;
    writeln!(w)?;

    writeln!(w, "        def OmniGraphNode \"PublishJointState\"")?;
    writeln!(w, "        {{")?;
    writeln!(
        w,
        "            token node:type = \"isaacsim.ros2.bridge.ROS2PublishJointState\""
    )?;
    writeln!(w, "            int node:typeVersion = 1")?;
    writeln!(w, "            custom uint inputs:execIn (")?;
    writeln!(w, "                customData = {{ bool isExecution = 1 }}")?;
    writeln!(w, "            )")?;
    writeln!(
        w,
        "            add uint inputs:execIn.connect = </World/{}/OnPlaybackTick.outputs:tick>",
        graph_name
    )?;
    // Single-statement rel: declares custom + sets target in one pass so
    // the resolver doesn't see two competing declarations.
    writeln!(
        w,
        "            add rel inputs:targetPrim = <{}>",
        articulation_path
    )?;
    writeln!(
        w,
        "            custom string inputs:nodeNamespace = \"{}\"",
        ns
    )?;
    writeln!(
        w,
        "            custom string inputs:topicName = \"joint_states\""
    )?;
    writeln!(w, "            custom double inputs:timeStamp = 0")?;
    writeln!(
        w,
        "            add double inputs:timeStamp.connect = </World/{}/IsaacReadSimulationTime.outputs:simulationTime>",
        graph_name
    )?;
    writeln!(w, "        }}")?;
    writeln!(w)?;

    writeln!(w, "        def OmniGraphNode \"SubscribeJointState\"")?;
    writeln!(w, "        {{")?;
    writeln!(
        w,
        "            token node:type = \"isaacsim.ros2.bridge.ROS2SubscribeJointState\""
    )?;
    writeln!(w, "            int node:typeVersion = 1")?;
    writeln!(w, "            custom uint inputs:execIn (")?;
    writeln!(w, "                customData = {{ bool isExecution = 1 }}")?;
    writeln!(w, "            )")?;
    writeln!(
        w,
        "            add uint inputs:execIn.connect = </World/{}/OnPlaybackTick.outputs:tick>",
        graph_name
    )?;
    writeln!(
        w,
        "            custom string inputs:nodeNamespace = \"{}\"",
        ns
    )?;
    writeln!(
        w,
        "            custom string inputs:topicName = \"joint_command\""
    )?;
    writeln!(w, "            custom uint outputs:execOut (")?;
    writeln!(w, "                customData = {{ bool isExecution = 1 }}")?;
    writeln!(w, "            )")?;
    writeln!(w, "            custom token[] outputs:jointNames")?;
    writeln!(w, "            custom double[] outputs:positionCommand")?;
    writeln!(w, "            custom double[] outputs:velocityCommand")?;
    writeln!(w, "            custom double[] outputs:effortCommand")?;
    writeln!(w, "        }}")?;
    writeln!(w)?;

    writeln!(w, "        def OmniGraphNode \"ArticulationController\"")?;
    writeln!(w, "        {{")?;
    writeln!(
        w,
        "            token node:type = \"isaacsim.core.nodes.IsaacArticulationController\""
    )?;
    writeln!(w, "            int node:typeVersion = 1")?;
    writeln!(w, "            custom uint inputs:execIn (")?;
    writeln!(w, "                customData = {{ bool isExecution = 1 }}")?;
    writeln!(w, "            )")?;
    writeln!(
        w,
        "            add uint inputs:execIn.connect = </World/{}/SubscribeJointState.outputs:execOut>",
        graph_name
    )?;
    writeln!(
        w,
        "            add rel inputs:targetPrim = <{}>",
        articulation_path
    )?;
    writeln!(
        w,
        "            custom token[] inputs:jointNames = [{}]",
        joint_token_array
    )?;
    writeln!(
        w,
        "            add token[] inputs:jointNames.connect = </World/{}/SubscribeJointState.outputs:jointNames>",
        graph_name
    )?;
    writeln!(w, "            custom double[] inputs:positionCommand")?;
    writeln!(
        w,
        "            add double[] inputs:positionCommand.connect = </World/{}/SubscribeJointState.outputs:positionCommand>",
        graph_name
    )?;
    writeln!(w, "            custom double[] inputs:velocityCommand")?;
    writeln!(
        w,
        "            add double[] inputs:velocityCommand.connect = </World/{}/SubscribeJointState.outputs:velocityCommand>",
        graph_name
    )?;
    writeln!(w, "            custom double[] inputs:effortCommand")?;
    writeln!(
        w,
        "            add double[] inputs:effortCommand.connect = </World/{}/SubscribeJointState.outputs:effortCommand>",
        graph_name
    )?;
    writeln!(w, "        }}")?;
    writeln!(w)?;

    // FR-160 / FR-161 / FR-162. Per-link contact sensor + ROS 2 publisher
    // pairs, inlined into this same per-robot graph so we reuse the
    // existing OnPlaybackTick. Wiring is automatic (URDF-derived); the
    // publisher's execIn is the sensor's execOut, NOT the tick — so the
    // topic stays silent until a contact event lands.
    for link_name in collision_link_names {
        emit_contact_sensor_pair(w, ns, &graph_name, &safe_id, link_name)?;
    }

    writeln!(w, "    }}")?;
    writeln!(w)?;
    Ok(())
}

/// FR-160 / FR-161 / FR-162. Emit one `IsaacReadContactSensor` + a
/// generic `ROS2Publisher` pair for a single collision link, inside the
/// already-open per-robot OmniGraph.
///
/// Wire shape:
///   OnPlaybackTick.tick → ContactSensor_<link>.execIn
///   ContactSensor_<link>.execOut → PublishContact_<link>.execIn   (FR-161 silence)
///   ContactSensor_<link>.inContact → PublishContact_<link>.data   (`std_msgs/msg/Bool`)
///
/// We deliberately use the generic `isaacsim.ros2.bridge.ROS2Publisher`
/// rather than `ROS2PublishContactSensor` because the latter does not
/// exist in Isaac Sim 5.1 — Isaac silently drops the node, the topic
/// never advertises, and `ros2 topic list` never sees the contact
/// channel. The generic publisher supports any registered ROS 2 message
/// type by name; we ship a `std_msgs/msg/Bool` payload carrying the
/// sensor's `inContact` flag. Rich force/normal context comes from the
/// cell-aggregator pod (BL-24, FR-213) which subscribes to these Bool
/// topics, queries TF + PhysX for the impact frame on the rising edge,
/// and re-publishes a `cell_sim_contact_msgs/msg/CollisionEvent` on
/// `/<cell>/contacts/v1`.
///
/// Topic FQT: `/<cell_id>/<robot_id>/contacts/<link_name>` —
/// `inputs:nodeNamespace = "<cell>/<robot>"` + `inputs:topicName =
/// "contacts/<link>"`. Matches the FR-151 + FR-160 contract.
fn emit_contact_sensor_pair(
    w: &mut impl Write,
    namespace: &str,
    graph_name: &str,
    safe_robot_id: &str,
    link_name: &str,
) -> Result<(), FittingRoomError> {
    let sensor_node = format!("ContactSensor_{}", link_name);
    let publisher_node = format!("PublishContact_{}", link_name);
    // Path to the per-link `IsaacContactSensor` USD prim emitted by
    // `emit_link_contact_sensor_prim`. The OmniGraph's `csPrim` rel
    // MUST point at this sensor prim, not the link Xform — pointing at
    // the link gives Isaac's "no valid sensor reading, is the sensor
    // enabled?" warning and silently drops events.
    let sensor_prim_path = format!(
        "/World/robots/{}/{}/ContactSensor_node",
        safe_robot_id, link_name
    );

    // Sensor — reads contact events from PhysX for the link prim.
    // PhysX must have PhysxContactReportAPI on the rigid body for events
    // to surface here (applied in emit_link when link.collisions is non-empty).
    writeln!(w, "        def OmniGraphNode \"{}\"", sensor_node)?;
    writeln!(w, "        {{")?;
    writeln!(
        w,
        "            token node:type = \"isaacsim.sensors.physics.IsaacReadContactSensor\""
    )?;
    writeln!(w, "            int node:typeVersion = 1")?;
    writeln!(w, "            custom uint inputs:execIn (")?;
    writeln!(w, "                customData = {{ bool isExecution = 1 }}")?;
    writeln!(w, "            )")?;
    writeln!(
        w,
        "            add uint inputs:execIn.connect = </World/{}/OnPlaybackTick.outputs:tick>",
        graph_name
    )?;
    writeln!(
        w,
        "            add rel inputs:csPrim = <{}>",
        sensor_prim_path
    )?;
    writeln!(w, "            custom uint outputs:execOut (")?;
    writeln!(w, "                customData = {{ bool isExecution = 1 }}")?;
    writeln!(w, "            )")?;
    writeln!(w, "            custom bool outputs:inContact")?;
    writeln!(w, "        }}")?;
    writeln!(w)?;

    // Generic ROS2Publisher emitting `std_msgs/msg/Bool` carrying the
    // sensor's inContact flag. The exec chain (sensor → publisher)
    // gives the FR-161 silence guarantee; the data input wires the
    // payload field by attribute connection.
    writeln!(w, "        def OmniGraphNode \"{}\"", publisher_node)?;
    writeln!(w, "        {{")?;
    writeln!(
        w,
        "            token node:type = \"isaacsim.ros2.bridge.ROS2Publisher\""
    )?;
    writeln!(w, "            int node:typeVersion = 1")?;
    writeln!(w, "            custom uint inputs:execIn (")?;
    writeln!(w, "                customData = {{ bool isExecution = 1 }}")?;
    writeln!(w, "            )")?;
    writeln!(
        w,
        "            add uint inputs:execIn.connect = </World/{}/{}.outputs:execOut>",
        graph_name, sensor_node
    )?;
    writeln!(
        w,
        "            custom string inputs:messagePackage = \"std_msgs\""
    )?;
    writeln!(
        w,
        "            custom string inputs:messageSubfolder = \"msg\""
    )?;
    writeln!(
        w,
        "            custom string inputs:messageName = \"Bool\""
    )?;
    writeln!(
        w,
        "            custom string inputs:nodeNamespace = \"{}\"",
        namespace
    )?;
    writeln!(
        w,
        "            custom string inputs:topicName = \"contacts/{}\"",
        link_name
    )?;
    writeln!(w, "            custom bool inputs:data = 0")?;
    writeln!(
        w,
        "            add bool inputs:data.connect = </World/{}/{}.outputs:inContact>",
        graph_name, sensor_node
    )?;
    writeln!(w, "        }}")?;
    writeln!(w)?;
    Ok(())
}

/// FR-107: half the width (m) of a floor-plan strip: 2.5 cm wide lines,
/// visible at cell scale.
const FLOOR_PLAN_HALF_WIDTH_M: f64 = 0.0125;

/// FR-107: how far (m) the floor plan lies above the ground (z = 0), so the
/// two don't fight.
const FLOOR_PLAN_LIFT_M: f64 = 0.002;

/// FR-107: the floor plan's colour, slate (sRGB #3d4653), linear.
const FLOOR_PLAN_COLOR: [f64; 3] = [0.046665, 0.061246, 0.0865];

/// FR-107: the floor plan's roughness: matte.
const FLOOR_PLAN_ROUGHNESS: f64 = 0.9;

/// FR-107: the floor plan's material, which both its layers wear.
const FLOOR_PLAN_MATERIAL: &str = "/World/floor_plan/Looks/M_floorplan";

/// FR-107: the floor plan, drawn on the ground as `/World/floor_plan`: its
/// general lines (`lines`) and its footprints (`footprints`), each one flat
/// mesh of [`segment_strips`], both in the matte slate `M_floorplan`, lifted
/// [`FLOOR_PLAN_LIFT_M`] above the ground. Rendered geometry (no `guide`
/// purpose, which Kit hides) without a collider: a drawing, nothing to stand
/// on. A layer without a segment of positive length gets no mesh, a floor plan
/// without any no prim.
fn emit_floor_plan(w: &mut impl Write, fp: &FloorPlan) -> Result<(), FittingRoomError> {
    let layers: Vec<(&str, Vec<[[f64; 3]; 4]>)> = [
        ("lines", &fp.segments),
        ("footprints", &fp.footprint_segments),
    ]
    .into_iter()
    .map(|(name, segments)| (name, segment_strips(segments)))
    .filter(|(_, strips)| !strips.is_empty())
    .collect();
    if layers.is_empty() {
        return Ok(());
    }

    writeln!(w, "    def Xform \"floor_plan\"")?;
    writeln!(w, "    {{")?;
    writeln!(
        w,
        "        double3 xformOp:translate = (0, 0, {})",
        f(FLOOR_PLAN_LIFT_M)
    )?;
    writeln!(
        w,
        "        uniform token[] xformOpOrder = [\"xformOp:translate\"]"
    )?;
    writeln!(w)?;
    writeln!(w, "        def Scope \"Looks\"")?;
    writeln!(w, "        {{")?;
    writeln!(w, "            def Material \"M_floorplan\"")?;
    writeln!(w, "            {{")?;
    writeln!(
        w,
        "                token outputs:surface.connect = <{}/Shader.outputs:surface>",
        FLOOR_PLAN_MATERIAL
    )?;
    writeln!(w)?;
    writeln!(w, "                def Shader \"Shader\"")?;
    writeln!(w, "                {{")?;
    writeln!(
        w,
        "                    uniform token info:id = \"UsdPreviewSurface\""
    )?;
    let [r, g, b] = FLOOR_PLAN_COLOR;
    writeln!(
        w,
        "                    color3f inputs:diffuseColor = ({}, {}, {})",
        f(r),
        f(g),
        f(b)
    )?;
    writeln!(w, "                    float inputs:metallic = 0")?;
    writeln!(
        w,
        "                    float inputs:roughness = {}",
        f(FLOOR_PLAN_ROUGHNESS)
    )?;
    writeln!(w, "                    token outputs:surface")?;
    writeln!(w, "                }}")?;
    writeln!(w, "            }}")?;
    writeln!(w, "        }}")?;
    for (name, strips) in &layers {
        writeln!(w)?;
        emit_segment_mesh(w, name, strips, 8)?;
    }
    writeln!(w, "    }}")?;
    writeln!(w)?;
    Ok(())
}

/// FR-107: `segments` as strips 2 · [`FLOOR_PLAN_HALF_WIDTH_M`] wide, at the
/// segments' own height: one quad per segment, its corners in order around
/// (counter-clockwise from above, so it faces up), skipping a segment shorter
/// than 1 nm or with a non-finite coordinate.
fn segment_strips(segments: &[[[f64; 3]; 2]]) -> Vec<[[f64; 3]; 4]> {
    segments
        .iter()
        .filter(|[a, b]| a.iter().chain(b).all(|v| v.is_finite()))
        .filter_map(|&[a, b]| {
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let len = (dx * dx + dy * dy).sqrt();
            if len < 1e-9 {
                return None;
            }
            // Half the width, to the left of a → b.
            let (nx, ny) = (
                -dy / len * FLOOR_PLAN_HALF_WIDTH_M,
                dx / len * FLOOR_PLAN_HALF_WIDTH_M,
            );
            Some([
                [a[0] + nx, a[1] + ny, a[2]],
                [a[0] - nx, a[1] - ny, a[2]],
                [b[0] - nx, b[1] - ny, b[2]],
                [b[0] + nx, b[1] + ny, b[2]],
            ])
        })
        .collect()
}

/// FR-107: one floor-plan layer as a flat mesh: two triangles per strip of
/// `strips` ([`segment_strips`]), normals up, double-sided, in
/// [`FLOOR_PLAN_MATERIAL`].
fn emit_segment_mesh(
    w: &mut impl Write,
    name: &str,
    strips: &[[[f64; 3]; 4]],
    indent: usize,
) -> Result<(), FittingRoomError> {
    let pad = " ".repeat(indent);
    let mut extent = Aabb::EMPTY;
    for p in strips.iter().flatten() {
        extent.add_point(*p);
    }
    let ([x0, y0, z0], [x1, y1, z1]) = (extent.min, extent.max);

    writeln!(w, "{pad}def Mesh \"{name}\" (")?;
    writeln!(w, "{pad}    prepend apiSchemas = [\"MaterialBindingAPI\"]")?;
    writeln!(w, "{pad})")?;
    writeln!(w, "{pad}{{")?;
    writeln!(w, "{pad}    uniform bool doubleSided = 1")?;
    writeln!(
        w,
        "{pad}    float3[] extent = [({}, {}, {}), ({}, {}, {})]",
        f(x0),
        f(y0),
        f(z0),
        f(x1),
        f(y1),
        f(z1)
    )?;
    write!(w, "{pad}    int[] faceVertexCounts = [")?;
    for i in 0..2 * strips.len() {
        write!(w, "{}3", if i > 0 { ", " } else { "" })?;
    }
    writeln!(w, "]")?;
    // Triangles (0, 1, 2) and (0, 2, 3) of every strip's 4 corners.
    write!(w, "{pad}    int[] faceVertexIndices = [")?;
    for i in 0..strips.len() {
        let b = 4 * i;
        write!(
            w,
            "{}{}, {}, {}, {}, {}, {}",
            if i > 0 { ", " } else { "" },
            b,
            b + 1,
            b + 2,
            b,
            b + 2,
            b + 3
        )?;
    }
    writeln!(w, "]")?;
    writeln!(w, "{pad}    rel material:binding = <{FLOOR_PLAN_MATERIAL}>")?;
    write!(w, "{pad}    normal3f[] normals = [")?;
    for i in 0..4 * strips.len() {
        write!(w, "{}(0, 0, 1)", if i > 0 { ", " } else { "" })?;
    }
    writeln!(w, "] (")?;
    writeln!(w, "{pad}        interpolation = \"vertex\"")?;
    writeln!(w, "{pad}    )")?;
    write!(w, "{pad}    point3f[] points = [")?;
    for (i, p) in strips.iter().flatten().enumerate() {
        write!(
            w,
            "{}({}, {}, {})",
            if i > 0 { ", " } else { "" },
            f(p[0]),
            f(p[1]),
            f(p[2])
        )?;
    }
    writeln!(w, "]")?;
    writeln!(w, "{pad}    uniform token subdivisionScheme = \"none\"")?;
    writeln!(w, "{pad}}}")?;
    Ok(())
}

fn find_root_link(robot: &UrdfRobot) -> Option<&str> {
    tool::root_link(robot)
}

/// Compute each link's bind-pose transform relative to the URDF root link,
/// at the home pose `homes` ([`drives::actuated_home_values`]: one value per
/// actuated joint, degrees, metres for prismatic joints). The root link maps
/// to the identity matrix; every other link is its parent's pose × its
/// joint's origin × the joint's home motion (a turn about, or a slide along,
/// the joint axis).
///
/// These are the authored link transforms: they keep every joint's two frames
/// together (omni.physx checks the bodies against the joint frames at load and
/// warns "disjointed body transforms") and show the home pose before Play.
/// They don't set the simulation's start state: when an articulation is added
/// to the scene, PhysX overwrites its non-root link poses from the joint
/// positions, which start at the authored `PhysicsJointStateAPI` state
/// (FR-157), and at 0 without one.
///
/// FR-117 / FR-118 (home pose): the angles are the robot's `home_pose_deg`, or
/// else the KUKA stowed default. Fixtures (fixed joints only) pass none.
fn compute_link_bind_poses(robot: &UrdfRobot, homes: &[f64]) -> HashMap<String, Mat4> {
    let mut by_name: HashMap<String, Mat4> = HashMap::new();
    let root = match find_root_link(robot) {
        Some(r) => r.to_string(),
        None => return by_name,
    };
    by_name.insert(root.clone(), math3::IDENTITY);

    // Each actuated joint's home value, by joint name; `homes` is indexed by
    // actuated joint (URDF declaration order, fixed joints skipped).
    let home_by_joint: HashMap<&str, f64> = drives::actuated_joints(robot)
        .map(|j| j.name.as_str())
        .zip(homes.iter().copied())
        .collect();

    // Index joints by parent link name for tree traversal.
    let mut children_of: HashMap<&str, Vec<&urdf_robo_dresser::Joint>> = HashMap::new();
    for j in &robot.joints {
        children_of.entry(j.parent.as_str()).or_default().push(j);
    }

    // BFS from the root.
    let mut stack: Vec<String> = vec![root];
    while let Some(parent) = stack.pop() {
        let parent_pose = by_name.get(&parent).copied().unwrap_or(math3::IDENTITY);
        if let Some(joints) = children_of.get(parent.as_str()) {
            for joint in joints {
                let joint_local =
                    math3::from_translation_rpy(joint.origin.xyz, joint.origin.rpy);
                // Apply the home value as an additional motion about (along)
                // the joint's axis, expressed in the joint's local frame.
                let home = home_by_joint.get(joint.name.as_str()).copied().unwrap_or(0.0);
                // The axis as `emit_joint` writes it: unit, signed, a zero
                // axis URDF's default +X (`drives::unit_joint_axis`).
                let axis = drives::unit_joint_axis(joint.axis);
                let home_local = if home == 0.0 {
                    math3::IDENTITY
                } else {
                    match joint.joint_type {
                        JointType::Revolute | JointType::Continuous => {
                            math3::rotation_about_axis(axis, home.to_radians())
                        }
                        JointType::Prismatic => {
                            math3::from_translation_rpy(math3::scale(axis, home), [0.0; 3])
                        }
                        _ => math3::IDENTITY,
                    }
                };
                let with_home = math3::multiply(&joint_local, &home_local);
                let child_pose = math3::multiply(&parent_pose, &with_home);
                by_name.insert(joint.child.clone(), child_pose);
                stack.push(joint.child.clone());
            }
        }
    }

    by_name
}

fn sanitize_prim_name(name: &str) -> String {
    // Avoid "World" as a child prim name — it shadows the stage root and
    // confuses PhysX's transform hierarchy walk.
    if name == "World" {
        return "world_link".to_string();
    }
    let mut out = String::with_capacity(name.len());
    for (i, c) in name.chars().enumerate() {
        if i == 0 {
            if c.is_ascii_alphabetic() || c == '_' {
                out.push(c);
            } else {
                out.push('_');
                if c.is_ascii_alphanumeric() {
                    out.push(c);
                }
            }
        } else if c.is_ascii_alphanumeric() || c == '_' {
            out.push(c);
        } else {
            out.push('_');
        }
    }
    out
}

/// Escapes `s` for a double-quoted USDA string: backslash and quote, and
/// newline, carriage return and tab as `\n`, `\r`, `\t` (a one-line string
/// can't hold a raw line break).
fn escape_str(s: &str) -> String {
    s.replace('\\', "\\\\")
        .replace('"', "\\\"")
        .replace('\n', "\\n")
        .replace('\r', "\\r")
        .replace('\t', "\\t")
}

fn f(v: f64) -> String {
    if !v.is_finite() {
        return "0".to_string();
    }
    let abs = v.abs();
    // For very large or very small magnitudes, emit scientific notation —
    // otherwise `{:.6}` produces 30+ digit integer literals (URDF effort
    // defaults of f32::MAX expand to 3.4028e38 → the USD parser warns
    // "Integer literal out of range" and re-parses as double).
    if abs != 0.0 && (abs >= 1.0e15 || abs < 1.0e-6) {
        return format!("{:e}", v);
    }
    let s = format!("{:.6}", v);
    if s.contains('.') {
        let trimmed = s.trim_end_matches('0').trim_end_matches('.');
        if trimmed.is_empty() || trimmed == "-" {
            "0".to_string()
        } else {
            trimmed.to_string()
        }
    } else {
        s
    }
}

/// A double at full precision: the shortest decimal that reads back as the
/// same `f64`, where [`f`] rounds to 6 decimals; very large and very small
/// magnitudes and non-finite values as in [`f`]. For derived `double`
/// attributes whose precision matters: rounded, the camera rig's poses would
/// aim it ~1e-5 m off its target and push a framed corner out of view.
fn f_exact(v: f64) -> String {
    if !v.is_finite() {
        return "0".to_string();
    }
    let abs = v.abs();
    if abs != 0.0 && !(1.0e-6..1.0e15).contains(&abs) {
        format!("{:e}", v)
    } else {
        v.to_string()
    }
}

pub fn abc_deg_to_quat(abc_deg: [f64; 3]) -> [f64; 4] {
    let a = abc_deg[0].to_radians();
    let b = abc_deg[1].to_radians();
    let c = abc_deg[2].to_radians();
    rpy_zyx_to_quat(c, b, a)
}

pub fn rpy_rad_to_quat(rpy: [f64; 3]) -> [f64; 4] {
    let (roll, pitch, yaw) = (rpy[0], rpy[1], rpy[2]);
    rpy_zyx_to_quat(roll, pitch, yaw)
}

fn rpy_zyx_to_quat(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let cr = (roll * 0.5).cos();
    let sr = (roll * 0.5).sin();
    let cp = (pitch * 0.5).cos();
    let sp = (pitch * 0.5).sin();
    let cy = (yaw * 0.5).cos();
    let sy = (yaw * 0.5).sin();

    let qw = cr * cp * cy + sr * sp * sy;
    let qx = sr * cp * cy - cr * sp * sy;
    let qy = cr * sp * cy + sr * cp * sy;
    let qz = cr * cp * sy - sr * sp * cy;
    // Canonicalise so w >= 0 (PhysX prefers this; mathematically equivalent rotation).
    if qw < 0.0 {
        [-qx, -qy, -qz, -qw]
    } else {
        [qx, qy, qz, qw]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Read;
    // `Write` is referenced implicitly by `writeln!` macros below; the
    // compiler doesn't see it as a name lookup so we don't import it
    // explicitly.

    fn write_minimal_urdf(path: &Path, name: &str, n_links: usize) {
        let dir = path.parent().unwrap();
        std::fs::create_dir_all(dir.join("meshes")).unwrap();
        for i in 0..n_links {
            let mesh_path = dir.join(format!("meshes/link_{}.obj", i));
            let mut f = std::fs::File::create(&mesh_path).unwrap();
            writeln!(f, "v 0 0 0").unwrap();
            writeln!(f, "v 1 0 0").unwrap();
            writeln!(f, "v 0 1 0").unwrap();
            writeln!(f, "f 1 2 3").unwrap();
        }

        let mut s = format!("<?xml version=\"1.0\"?>\n<robot name=\"{}\">\n", name);
        for i in 0..n_links {
            s.push_str(&format!(
                "  <link name=\"link_{}\">\n    <visual>\n      <geometry>\n        <mesh filename=\"./meshes/link_{}.obj\"/>\n      </geometry>\n    </visual>\n    <collision>\n      <geometry>\n        <mesh filename=\"./meshes/link_{}.obj\"/>\n      </geometry>\n    </collision>\n  </link>\n",
                i, i, i
            ));
        }
        for i in 0..n_links.saturating_sub(1) {
            s.push_str(&format!(
                "  <joint name=\"joint_{}\" type=\"revolute\">\n    <parent link=\"link_{}\"/>\n    <child link=\"link_{}\"/>\n    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n    <axis xyz=\"0 0 1\"/>\n    <limit lower=\"-1.5\" upper=\"1.5\" effort=\"100\" velocity=\"1\"/>\n  </joint>\n",
                i, i, i + 1
            ));
        }
        s.push_str("</robot>\n");
        std::fs::File::create(path)
            .unwrap()
            .write_all(s.as_bytes())
            .unwrap();
    }

    fn write_fixed_only_urdf(path: &Path, name: &str) {
        let s = format!(
            "<?xml version=\"1.0\"?>\n<robot name=\"{}\">\n  <link name=\"base\"/>\n  <link name=\"plate\"/>\n  <joint name=\"j_fix\" type=\"fixed\">\n    <parent link=\"base\"/>\n    <child link=\"plate\"/>\n    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n  </joint>\n</robot>\n",
            name
        );
        std::fs::File::create(path)
            .unwrap()
            .write_all(s.as_bytes())
            .unwrap();
    }

    fn write_dummy_manifest(path: &Path) {
        std::fs::write(path, "name: \"test\"\nrobots: []\nfixtures: []\n").unwrap();
    }

    fn list_zip_entries(zip_path: &Path) -> Vec<String> {
        let f = File::open(zip_path).unwrap();
        let mut z = zip::ZipArchive::new(f).unwrap();
        (0..z.len())
            .map(|i| z.by_index(i).unwrap().name().unwrap().into_owned())
            .collect()
    }

    fn read_zip_entry(zip_path: &Path, entry: &str) -> String {
        let f = File::open(zip_path).unwrap();
        let mut z = zip::ZipArchive::new(f).unwrap();
        let mut e = z.by_name(entry).unwrap();
        let mut s = String::new();
        e.read_to_string(&mut s).unwrap();
        s
    }

    #[test]
    fn single_link_inline_mesh() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("r1.urdf");
        write_minimal_urdf(&urdf, "r1", 1);
        let manifest = dir.path().join("scene.yaml");
        write_dummy_manifest(&manifest);

        let scene = SceneDefinition {
            name: "t".into(),
            description: "".into(),
            robots: vec![RobotPlacement {
                robot_id: "ROBO_A".into(),
                dressed_urdf: urdf,
                position: [0.0; 3],
                orientation_deg: [0.0; 3],
                home_pose_deg: None,
                pedestal: None,
                payload: None,
                link_density_kg_m3: None,
            }],
            fixtures: vec![],
            floor_plan: None,
        };
        let out = dir.path().join("scene.zip");
        // Use the full-robots compose so both visual + collision mesh
        // emission paths are exercised.
        let opts = ComposeOptions {
            full_robots: true,
            full_tools: true,
        };
        let report =
            compose_stage_with_options(&scene, &manifest, &out, &opts).unwrap();
        assert_eq!(report.robot_count, 1);
        assert_eq!(report.mesh_count, 2); // 1 visual + 1 collision

        let usda = read_zip_entry(&out, "scene.usda");
        assert!(usda.contains("def Xform \"ROBO_A\""));
        assert!(usda.contains("PhysicsArticulationRootAPI"));
        assert!(usda.contains("def Xform \"link_0\""));
        // Articulation requires every link — root included — to be a rigid body.
        assert!(usda.contains("PhysicsRigidBodyAPI"));
        // Geometry is inlined per visual/collision, no /World/Meshes scope.
        assert!(!usda.contains("def Scope \"Meshes\""));
        assert!(usda.contains("point3f[] points = ["));
        assert!(usda.contains("int[] faceVertexIndices = ["));
        assert!(usda.contains("int[] faceVertexCounts = ["));

        let entries = list_zip_entries(&out);
        assert!(entries.contains(&"scene.yaml".to_string()));
        assert!(entries.contains(&"scene.usda".to_string()));
        // No meshes/ folder anymore
        assert!(!entries.iter().any(|e| e.starts_with("meshes/")));
    }

    #[test]
    fn lean_robots_drop_visuals_keep_collisions() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("r1.urdf");
        write_minimal_urdf(&urdf, "r1", 1);
        let manifest = dir.path().join("scene.yaml");
        write_dummy_manifest(&manifest);

        let scene = SceneDefinition {
            name: "lean".into(),
            description: "".into(),
            robots: vec![RobotPlacement {
                robot_id: "ROBO".into(),
                dressed_urdf: urdf,
                position: [0.0; 3],
                orientation_deg: [0.0; 3],
                home_pose_deg: None,
                pedestal: None,
                payload: None,
                link_density_kg_m3: None,
            }],
            fixtures: vec![],
            floor_plan: None,
        };
        let out = dir.path().join("scene.zip");
        // Default ComposeOptions has full_robots = false → lean path.
        let opts = ComposeOptions::default();
        let report =
            compose_stage_with_options(&scene, &manifest, &out, &opts).unwrap();
        assert_eq!(report.mesh_count, 1, "only collision mesh expected");

        let usda = read_zip_entry(&out, "scene.usda");
        // No `visual_*` prim, articulation root and joint still present.
        assert!(!usda.contains("def Mesh \"visual_"));
        assert!(usda.contains("def Mesh \"collision_"));
        // Isaac Sim's RTX: the hull is polygons as they are (no Catmull-Clark rounding) and, shown,
        // matte (the default material, not metallic) instead of the renderer's glossy default.
        assert_eq!(usda.matches("subdivisionScheme = \"none\"").count(), usda.matches("def Mesh ").count());
        assert!(usda.contains("rel material:binding = </World/Looks/factoryos_default>"));
        assert!(usda.contains("def Material \"factoryos_default\""));
        assert!(usda.contains("float inputs:metallic = 0"));
        assert!(usda.contains("PhysicsArticulationRootAPI"));
        // Hull purpose flipped to default (visible) in lean mode — the
        // viewport-hiding `purpose = "guide"` line should be absent (the
        // ground's collider alone is a guide).
        let robots = block(&usda, "    def Scope \"robots\" (");
        assert!(!robots.contains("uniform token purpose = \"guide\""));
        assert_eq!(usda.matches("uniform token purpose = \"guide\"").count(), 1);
    }

    // FR-119
    #[test]
    fn pedestal_raises_robot_and_emits_cylinder() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("r1.urdf");
        write_minimal_urdf(&urdf, "r1", 1);
        let manifest = dir.path().join("scene.yaml");
        write_dummy_manifest(&manifest);

        let scene = SceneDefinition {
            name: "ped".into(),
            description: "".into(),
            robots: vec![RobotPlacement {
                robot_id: "ROBO".into(),
                dressed_urdf: urdf,
                position: [1.0, 2.0, 3.0],
                orientation_deg: [0.0; 3],
                home_pose_deg: None,
                pedestal: Some(crate::scene::Pedestal {
                    height_mm: 500.0,
                    diameter_mm: 800.0,
                }),
                payload: None,
                link_density_kg_m3: None,
            }],
            fixtures: vec![],
            floor_plan: None,
        };
        let out = dir.path().join("scene.zip");
        compose_stage_with_options(&scene, &manifest, &out, &ComposeOptions::default()).unwrap();
        let usda = read_zip_entry(&out, "scene.usda");

        // Static pedestal cylinder: 800mm dia → r=0.4, 500mm tall → h=0.5,
        // centred at z = 3 + 0.25 so it spans the robot's placement height (3)
        // to its raised foot (3.5).
        assert!(usda.contains("def Scope \"pedestals\""));
        assert!(usda.contains("def Xform \"pedestal_ROBO\""));
        assert!(usda.contains("def Cylinder \"socket\""));
        assert!(usda.contains("PhysicsCollisionAPI"));
        assert!(usda.contains("double radius = 0.4"));
        assert!(usda.contains("double height = 0.5"));
        assert!(usda.contains("double3 xformOp:translate = (1, 2, 3.25)"));
        // Matte: the default material, not the renderer's glossy default.
        let socket = block(&usda, "            def Cylinder \"socket\" (");
        assert!(socket.starts_with(
            "            def Cylinder \"socket\" (\n\
             \x20               prepend apiSchemas = [\"PhysicsCollisionAPI\", \"MaterialBindingAPI\"]\n\
             \x20           )\n\
             \x20           {\n\
             \x20               rel material:binding = </World/Looks/factoryos_default>\n"
        ));
        let looks = block(&usda, "    def Scope \"Looks\"");
        let default = block(looks, "        def Material \"factoryos_default\"");
        assert!(default.contains("color3f inputs:diffuseColor = (0.45, 0.45, 0.45)"));
        assert!(default.contains("float inputs:metallic = 0"));

        // Robot foot raised by the pedestal height: z 3.0 → 3.5, on both the
        // Xform translate and the world_anchor localPos0.
        assert!(usda.contains("double3 xformOp:translate = (1, 2, 3.5)"));
        assert!(usda.contains("point3f physics:localPos0 = (1, 2, 3.5)"));
    }

    // FR-119
    #[test]
    fn no_pedestal_means_no_scope_and_unchanged_z() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("r1.urdf");
        write_minimal_urdf(&urdf, "r1", 1);
        let manifest = dir.path().join("scene.yaml");
        write_dummy_manifest(&manifest);

        let scene = SceneDefinition {
            name: "flat".into(),
            description: "".into(),
            robots: vec![RobotPlacement {
                robot_id: "ROBO".into(),
                dressed_urdf: urdf,
                position: [1.0, 2.0, 3.0],
                orientation_deg: [0.0; 3],
                home_pose_deg: None,
                pedestal: None,
                payload: None,
                link_density_kg_m3: None,
            }],
            fixtures: vec![],
            floor_plan: None,
        };
        let out = dir.path().join("scene.zip");
        compose_stage_with_options(&scene, &manifest, &out, &ComposeOptions::default()).unwrap();
        let usda = read_zip_entry(&out, "scene.usda");

        assert!(!usda.contains("def Scope \"pedestals\""));
        assert!(usda.contains("double3 xformOp:translate = (1, 2, 3)"));
    }

    /// Build a one-link URDF with two visuals: one robot mesh under
    /// `./meshes/robot/`, one tool mesh under `./meshes/tool/`. Used to
    /// verify the `--full-tools=false` toggle independently of `--full-robots`.
    fn write_robot_with_tool_urdf(path: &Path) {
        let dir = path.parent().unwrap();
        std::fs::create_dir_all(dir.join("meshes/robot")).unwrap();
        std::fs::create_dir_all(dir.join("meshes/tool")).unwrap();
        for sub in ["robot", "tool"] {
            let p = dir.join(format!("meshes/{}/m.obj", sub));
            let mut f = std::fs::File::create(&p).unwrap();
            writeln!(f, "v 0 0 0").unwrap();
            writeln!(f, "v 1 0 0").unwrap();
            writeln!(f, "v 0 1 0").unwrap();
            writeln!(f, "f 1 2 3").unwrap();
        }
        let s = "<?xml version=\"1.0\"?>\n<robot name=\"rt\">\n  <link name=\"link_0\">\n    <visual>\n      <geometry>\n        <mesh filename=\"./meshes/robot/m.obj\"/>\n      </geometry>\n    </visual>\n    <visual>\n      <geometry>\n        <mesh filename=\"./meshes/tool/m.obj\"/>\n      </geometry>\n    </visual>\n    <collision>\n      <geometry>\n        <mesh filename=\"./meshes/robot/m.obj\"/>\n      </geometry>\n    </collision>\n    <collision>\n      <geometry>\n        <mesh filename=\"./meshes/tool/m.obj\"/>\n      </geometry>\n    </collision>\n  </link>\n</robot>\n";
        std::fs::File::create(path)
            .unwrap()
            .write_all(s.as_bytes())
            .unwrap();
    }

    /// "Generate USDA" twice of the same scene makes the same bundle, byte for byte (fixed entry
    /// dates and order, no clock, no random ids); the visual options change the bundle, not its
    /// planning digest (the zip comment the cells' input hashes use); a moved robot changes both.
    #[test]
    fn same_compose_same_bundle_and_visual_options_keep_the_planning_digest() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("rt.urdf");
        write_robot_with_tool_urdf(&urdf);
        let manifest = dir.path().join("scene.yaml");
        write_dummy_manifest(&manifest);
        let placement = |x: f64| RobotPlacement {
            robot_id: "RT".into(),
            dressed_urdf: urdf.clone(),
            position: [x, 0.0, 0.0],
            orientation_deg: [0.0; 3],
            home_pose_deg: None,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        };
        let scene_at = |x: f64| SceneDefinition {
            name: "same".into(),
            description: "".into(),
            robots: vec![placement(x)],
            fixtures: vec![],
            floor_plan: None,
        };
        let compose = |name: &str, scene: &SceneDefinition, full_robots: bool, full_tools: bool| {
            let out = dir.path().join(name).join("scene.zip");
            std::fs::create_dir_all(out.parent().unwrap()).unwrap();
            let opts = ComposeOptions { full_robots, full_tools };
            let report = compose_stage_with_options(scene, &manifest, &out, &opts).unwrap();
            let bytes = std::fs::read(&out).unwrap();
            let mut archive = zip::ZipArchive::new(std::io::Cursor::new(bytes.clone())).unwrap();
            let comment = bundle_planning_digest(archive.comment());
            assert_eq!(comment.as_deref(), Some(report.planning_sha256.as_str()), "{name}: the zip comment records the digest");
            assert_eq!(bundle_legacy_hashes(archive.comment()), report.legacy_sha256, "{name}: the zip comment records the legacy hashes");
            let mut entries = Vec::new();
            for i in 0..archive.len() {
                let mut entry = archive.by_index(i).unwrap();
                assert_eq!(entry.last_modified(), Some(zip::DateTime::default()), "{name}: {:?} is dated the zip epoch", entry.name());
                let mut content = Vec::new();
                std::io::Read::read_to_end(&mut entry, &mut content).unwrap();
                entries.push((entry.name().unwrap().into_owned(), content));
            }
            // The first legacy hash: the bundle as the composer before the planning digest wrote
            // it (its code, verbatim), the same bytes without the comment.
            assert_eq!(report.legacy_sha256.first(), Some(&sha256_hex(&composed_before_the_digest(&entries))), "{name}");
            // The comment, as ZipWriter::set_comment would write it.
            let mut zip = zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
            zip.set_comment(String::from_utf8(archive.comment().to_vec()).unwrap()).unwrap();
            for (entry, content) in &entries {
                zip.start_file(entry.as_str(), entry_options()).unwrap();
                zip.write_all(content).unwrap();
            }
            assert!(zip.finish().unwrap().into_inner() == bytes, "{name}: the comment is written as set_comment writes it");
            (bytes, report.planning_sha256, report.legacy_sha256, entries)
        };
        let scene = scene_at(0.0);
        let (full_a, digest_full_a, legacy_full, _) = compose("full-a", &scene, true, true);
        let (full_b, digest_full_b, _, _) = compose("full-b", &scene, true, true);
        assert!(full_a == full_b, "two composes of the same scene with the same options differ");
        assert_eq!(digest_full_a, digest_full_b);

        let (lean, digest_lean, legacy_lean, lean_entries) = compose("lean", &scene, false, false);
        let (_, digest_tools, legacy_tools, _) = compose("tools", &scene, false, true);
        let (_, digest_robots, _, _) = compose("robots", &scene, true, false);
        assert!(lean != full_a, "the visual options change the bundle");
        assert_eq!(digest_lean, digest_full_a, "the visual options keep the planning digest");
        assert_eq!(digest_tools, digest_full_a);
        assert_eq!(digest_robots, digest_full_a);

        // The legacy hashes: a lean bundle has its own; another records its own and the lean one,
        // the file hash outputs recorded for a lean bundle an older composer wrote.
        let lean_before = sha256_hex(&composed_before_the_digest(&lean_entries));
        assert_eq!(legacy_lean, vec![lean_before.clone()]);
        assert_eq!(legacy_full.len(), 2);
        assert_eq!(legacy_full[1], lean_before);
        assert_eq!(legacy_tools[1], lean_before);
        assert_ne!(legacy_full[0], legacy_tools[0]);

        let (moved, digest_moved, legacy_moved, _) = compose("moved", &scene_at(0.5), true, true);
        assert!(moved != full_a);
        assert_ne!(digest_moved, digest_full_a, "a moved robot changes the planning digest");
        assert!(legacy_moved.iter().all(|hash| !legacy_full.contains(hash)), "and its legacy hashes");
    }

    /// A bundle of `entries` as `compose_stage_with_options` wrote it before the planning digest
    /// (v0.1.29's code, verbatim but for the in-memory file).
    fn composed_before_the_digest(entries: &[(String, Vec<u8>)]) -> Vec<u8> {
        let mut zip = zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
        let opts = SimpleFileOptions::default().compression_method(CompressionMethod::Deflated);
        for (name, content) in entries {
            zip.start_file(name.as_str(), opts).unwrap();
            zip.write_all(content).unwrap();
        }
        zip.finish().unwrap().into_inner()
    }

    /// Opt-in (`FR_LEGACY_BUNDLE=<zip>`, a bundle an older composer wrote, e.g. a workspace's
    /// `scenes/<name>/<name>.zip` from before the planning digest): this build writes its entries
    /// into the same bytes, so the legacy hash Generate USDA records is the hash outputs recorded.
    #[test]
    fn a_legacy_bundle_is_rewritten_byte_for_byte() {
        let Ok(path) = std::env::var("FR_LEGACY_BUNDLE") else { return };
        let bytes = std::fs::read(&path).unwrap();
        let mut archive = zip::ZipArchive::new(std::io::Cursor::new(bytes.clone())).unwrap();
        assert!(archive.comment().is_empty(), "{path} has a comment: not an older composer's");
        let mut entries = Vec::new();
        for i in 0..archive.len() {
            let mut entry = archive.by_index(i).unwrap();
            let mut content = Vec::new();
            std::io::Read::read_to_end(&mut entry, &mut content).unwrap();
            entries.push((entry.name().unwrap().into_owned(), content));
        }
        let [(manifest_name, manifest), (usda_name, usda)] = &entries[..] else { panic!("{path}: {} entries", entries.len()) };
        let rewritten = write_bundle(manifest_name, manifest, usda_name, usda).unwrap();
        assert_eq!(sha256_hex(&rewritten), sha256_hex(&bytes), "{path} rewritten");
        eprintln!("{path}: rewritten byte for byte, sha256 {}", sha256_hex(&bytes));
    }

    #[test]
    fn bundle_planning_digest_reads_the_comment() {
        let hex = "ab".repeat(32);
        let comment = format!("{PLANNING_DIGEST_KEY}{hex} full_robots=true full_tools=true");
        assert_eq!(bundle_planning_digest(comment.as_bytes()), Some(hex.clone()));
        assert_eq!(bundle_planning_digest(format!("{PLANNING_DIGEST_KEY}{}", hex.to_uppercase()).as_bytes()), Some(hex.clone()));
        assert_eq!(bundle_planning_digest(b""), None);
        assert_eq!(bundle_planning_digest(b"planning-sha256=abc"), None);
        assert_eq!(bundle_planning_digest(b"made by something else"), None);

        let other = "cd".repeat(32);
        let comment = format!("{comment} {LEGACY_SHA256_KEY}{} {LEGACY_SHA256_KEY}{other} {LEGACY_SHA256_KEY}abc", hex.to_uppercase());
        assert_eq!(bundle_planning_digest(comment.as_bytes()), Some(hex.clone()));
        assert_eq!(bundle_legacy_hashes(comment.as_bytes()), vec![hex, other]);
        assert!(bundle_legacy_hashes(b"").is_empty());
    }

    #[test]
    fn full_tools_off_drops_only_tool_visuals() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("rt.urdf");
        write_robot_with_tool_urdf(&urdf);
        let manifest = dir.path().join("scene.yaml");
        write_dummy_manifest(&manifest);

        let scene = SceneDefinition {
            name: "tools-off".into(),
            description: "".into(),
            robots: vec![RobotPlacement {
                robot_id: "RT".into(),
                dressed_urdf: urdf,
                position: [0.0; 3],
                orientation_deg: [0.0; 3],
                home_pose_deg: None,
                pedestal: None,
                payload: None,
                link_density_kg_m3: None,
            }],
            fixtures: vec![],
            floor_plan: None,
        };
        let out = dir.path().join("scene.zip");
        // full_robots=true keeps the robot visual; full_tools=false drops
        // the tool visual. We expect exactly one `def Mesh "visual_*"` (the
        // robot one at index 0) and both collisions emitted.
        let opts = ComposeOptions {
            full_robots: true,
            full_tools: false,
        };
        let _report =
            compose_stage_with_options(&scene, &manifest, &out, &opts).unwrap();

        let usda = read_zip_entry(&out, "scene.usda");
        assert!(
            usda.contains("def Mesh \"visual_0\""),
            "robot visual should be emitted"
        );
        assert!(
            !usda.contains("def Mesh \"visual_1\""),
            "tool visual should be suppressed by --full-tools=false"
        );
        // Both collisions still present (physics geometry is non-optional).
        assert!(usda.contains("def Mesh \"collision_0\""));
        assert!(usda.contains("def Mesh \"collision_1\""));
    }

    #[test]
    fn revolute_joint_emits_physics_revolute() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("r2.urdf");
        write_minimal_urdf(&urdf, "r2", 2);
        let manifest = dir.path().join("scene.yaml");
        write_dummy_manifest(&manifest);

        let scene = SceneDefinition {
            name: "t".into(),
            description: "".into(),
            robots: vec![RobotPlacement {
                robot_id: "R".into(),
                dressed_urdf: urdf,
                position: [0.0; 3],
                orientation_deg: [0.0; 3],
                home_pose_deg: None,
                pedestal: None,
                payload: None,
                link_density_kg_m3: None,
            }],
            fixtures: vec![],
            floor_plan: None,
        };
        let out = dir.path().join("scene.zip");
        compose_stage(&scene, &manifest, &out).unwrap();
        let usda = read_zip_entry(&out, "scene.usda");
        assert!(usda.contains("def PhysicsRevoluteJoint \"joint_0\""));
        assert!(usda.contains("rel physics:body0 = </World/robots/R/link_0>"));
        assert!(usda.contains("rel physics:body1 = </World/robots/R/link_1>"));
        // Revolute limits are emitted in degrees (URDF stores radians).
        assert!(usda.contains("physics:lowerLimit = -85.943669"));
        assert!(usda.contains("physics:upperLimit = 85.943669"));
        // Non-root link gets RigidBodyAPI; root (link_0) does not
        assert!(usda.contains("PhysicsRigidBodyAPI"));

        // FR-157 / FR-158: drive, initial joint state and velocity limit. link_1 (no inertial, a
        // triangle enclosing no volume: a frame link, 0.1 kg as a 5 cm sphere at the joint) moves
        // 1e-4 kg·m² about Z: damping 2·0.7·√(k·1e-4) per degree, k = 1.5e5·180/π N·m/rad. The
        // URDF effort 100 is the force limit, its velocity 1 rad/s the velocity limit; the KUKA
        // default's first joint, 0°, the target and the start.
        let damping =
            2.0 * 0.7 * (1.5e5 * 180.0 / std::f64::consts::PI * 1e-4).sqrt() * std::f64::consts::PI
                / 180.0;
        let expected = format!(
            "        def PhysicsRevoluteJoint \"joint_0\" (\n\
             \x20           prepend apiSchemas = [\"PhysicsDriveAPI:angular\", \"PhysicsJointStateAPI:angular\", \"PhysxJointAPI\"]\n\
             \x20       )\n\
             \x20       {{\n\
             \x20           rel physics:body0 = </World/robots/R/link_0>\n\
             \x20           rel physics:body1 = </World/robots/R/link_1>\n\
             \x20           uniform token physics:axis = \"Z\"\n\
             \x20           float physics:lowerLimit = -85.943669\n\
             \x20           float physics:upperLimit = 85.943669\n\
             \x20           float drive:angular:physics:stiffness = 150000\n\
             \x20           float drive:angular:physics:damping = {}\n\
             \x20           float drive:angular:physics:maxForce = 100\n\
             \x20           float drive:angular:physics:targetPosition = 0\n\
             \x20           float state:angular:physics:position = 0\n\
             \x20           float state:angular:physics:velocity = 0\n\
             \x20           float physxJoint:maxJointVelocity = 57.29578\n\
             \x20       }}\n",
            f(damping)
        );
        assert_eq!(
            block(&usda, "        def PhysicsRevoluteJoint \"joint_0\" ("),
            expected
        );
        assert_eq!(f(damping), "0.716329");
        let link_1 = block(&usda, "        def Xform \"link_1\" (");
        assert_eq!(attr(link_1, "physics:mass"), "0.1");
        assert_eq!(
            attr(link_1, "physics:diagonalInertia"),
            "(0.0001, 0.0001, 0.0001)"
        );
    }

    /// A robot `base` → `arm` on one joint `j1` of `kind` about Z at the origin, with `limit`
    /// (`<limit …/>` attributes); each link carries a triangle collision.
    fn write_one_joint_urdf(path: &Path, kind: &str, limit: &str) {
        write_triangle_obj(&path.parent().unwrap().join("meshes/tri.obj"), 1.0);
        let link = |name: &str| {
            format!(
                "  <link name=\"{name}\">\n    <collision><geometry><mesh filename=\"./meshes/tri.obj\"/></geometry></collision>\n  </link>\n"
            )
        };
        let s = format!(
            "<?xml version=\"1.0\"?>\n<robot name=\"one\">\n{}{}  <joint name=\"j1\" type=\"{kind}\">\n    <parent link=\"base\"/>\n    <child link=\"arm\"/>\n    <axis xyz=\"0 0 1\"/>\n    <limit {limit}/>\n  </joint>\n</robot>\n",
            link("base"),
            link("arm"),
        );
        std::fs::write(path, s).unwrap();
    }

    /// The value right of the `=` of `key`'s line in `text`.
    fn attr<'a>(text: &'a str, key: &str) -> &'a str {
        let line = text
            .lines()
            .find(|l| l.contains(&format!(" {key} = ")))
            .unwrap_or_else(|| panic!("no {key} in\n{text}"));
        line.split(" = ").nth(1).unwrap()
    }

    /// The emitted `matrix4d xformOp:transform` line of `m`, at `indent`.
    fn transform_line(m: &Mat4, indent: usize) -> String {
        let mut out = Vec::new();
        emit_xform_transform(&mut out, m, indent).unwrap();
        String::from_utf8(out).unwrap().lines().next().unwrap().to_owned()
    }

    // FR-157
    #[test]
    fn limited_revolute_state_stays_unwrapped() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("one.urdf");
        // ±6 rad (±343.8°: a wrist that turns further than a full turn).
        write_one_joint_urdf(
            &urdf,
            "revolute",
            "lower=\"-6\" upper=\"6\" effort=\"100\" velocity=\"5\"",
        );
        let mut robot = robot_at(urdf, [0.0; 3], 0.0);
        robot.home_pose_deg = Some(vec![190.0]);
        let usda = compose_usda(dir.path(), &scene_of(vec![robot], vec![]));
        let joint = block(&usda, "        def PhysicsRevoluteJoint \"j1\" (");
        assert_eq!(attr(joint, "drive:angular:physics:targetPosition"), "190");
        assert_eq!(attr(joint, "state:angular:physics:position"), "190");
        assert_eq!(attr(joint, "state:angular:physics:velocity"), "0");
        assert_eq!(attr(joint, "physics:upperLimit"), "343.774677");
        // 5 rad/s.
        assert_eq!(attr(joint, "physxJoint:maxJointVelocity"), "286.478898");
        // The bind pose turned by the same 190°.
        let arm = block(&usda, "        def Xform \"arm\" (");
        let turned = math3::rotation_about_axis([0.0, 0.0, 1.0], 190f64.to_radians());
        assert!(arm.contains(&transform_line(&turned, 12)), "{arm}");
    }

    // FR-157
    #[test]
    fn continuous_joint_target_and_state_follow_bind_pose() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("one.urdf");
        write_one_joint_urdf(&urdf, "continuous", "effort=\"100\" velocity=\"1\"");
        let mut robot = robot_at(urdf, [0.0; 3], 0.0);
        robot.home_pose_deg = Some(vec![190.0]);
        let usda = compose_usda(dir.path(), &scene_of(vec![robot], vec![]));
        let joint = block(&usda, "        def PhysicsRevoluteJoint \"j1\" (");
        // Wrapped into (−180°, 180°], and no limits on a continuous joint.
        assert_eq!(attr(joint, "drive:angular:physics:targetPosition"), "-170");
        assert_eq!(attr(joint, "state:angular:physics:position"), "-170");
        assert!(!joint.contains("Limit"));
        let arm = block(&usda, "        def Xform \"arm\" (");
        let turned = math3::rotation_about_axis([0.0, 0.0, 1.0], (-170f64).to_radians());
        assert!(arm.contains(&transform_line(&turned, 12)), "{arm}");
    }

    // FR-158
    #[test]
    fn placeholder_effort_gets_finite_derived_cap() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("one.urdf");
        // The dresser's placeholder: FLT_MAX.
        write_one_joint_urdf(
            &urdf,
            "revolute",
            "lower=\"-3\" upper=\"3\" effort=\"340282346638528859811704183484516925440\" velocity=\"2\"",
        );
        let usda = compose_usda(dir.path(), &scene_of(vec![robot_at(urdf.clone(), [0.0; 3], 0.0)], vec![]));
        let joint = block(&usda, "        def PhysicsRevoluteJoint \"j1\" (");
        // By hand: arm (no inertial, no volume: a frame link, 0.1 kg as a 5 cm sphere, its centre
        // on the axis) moves I = 1e-4 kg·m², holds no gravity torque (r = 0); v = 2 rad/s. The
        // cap: 2·0 + I·v/0.25 s + D·v, D = 2·0.7·√(1.5e5·180/π·I) N·m·s/rad.
        let inertia = 1e-4;
        let d_rad = 2.0 * 0.7 * (1.5e5 * 180.0 / std::f64::consts::PI * inertia).sqrt();
        let cap = inertia * 2.0 / 0.25 + d_rad * 2.0;
        assert_eq!(attr(joint, "drive:angular:physics:maxForce"), f(cap));
        assert_eq!(f(cap), "82.086022");
        assert!(!joint.contains("e38"));

        // A real effort is the limit.
        write_one_joint_urdf(
            &urdf,
            "revolute",
            "lower=\"-3\" upper=\"3\" effort=\"100\" velocity=\"2\"",
        );
        let usda = compose_usda(dir.path(), &scene_of(vec![robot_at(urdf, [0.0; 3], 0.0)], vec![]));
        let joint = block(&usda, "        def PhysicsRevoluteJoint \"j1\" (");
        assert_eq!(attr(joint, "drive:angular:physics:maxForce"), "100");
    }

    // FR-102
    #[test]
    fn fixtures_are_static_colliders() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("fx/fx.urdf");
        write_meshed_fixture_urdf(&urdf);
        let usda = compose_usda(
            dir.path(),
            &scene_of(vec![], vec![fixture_at(urdf, [5.0, 0.0, 0.0], 90.0)]),
        );
        let fixtures = block(&usda, "    def Scope \"fixtures\" (");
        // Both links, at their places, with their visual and collision meshes.
        assert!(fixtures.contains("            def Xform \"base\"\n            {\n"));
        assert!(fixtures.contains("            def Xform \"plate\"\n            {\n"));
        let plate = block(fixtures, "            def Xform \"plate\"");
        // (Through a product like the bind poses, which turns from_translation_rpy's −0 into 0.)
        let at_2m = math3::multiply(
            &math3::IDENTITY,
            &math3::from_translation_rpy([2.0, 0.0, 0.0], [0.0; 3]),
        );
        assert!(plate.contains(&transform_line(&at_2m, 16)), "{plate}");
        assert!(fixtures.contains("def Mesh \"visual_0\""));
        assert!(plate.contains("prepend apiSchemas = [\"PhysicsCollisionAPI\", \"PhysicsMeshCollisionAPI\"]"));
        // No body, mass, contact reporting or sensor; no anchor, no joints.
        for gone in [
            "PhysicsRigidBodyAPI",
            "PhysicsMassAPI",
            "physics:mass",
            "PhysxContactReportAPI",
            "physxContactReport",
            "ContactSensor",
            "world_anchor",
            "Joint",
            "physics:body",
        ] {
            assert!(!fixtures.contains(gone), "{gone} in\n{fixtures}");
        }
        // Nothing refers to the fixture's links but the fixture itself.
        assert_eq!(usda.matches("/World/fixtures/").count(), 0);
    }

    /// A robot `World` → (fixed) `base` → (revolute about Z) `tool0`, where `tool0` carries a
    /// robot collision triangle and a tool collision box of `half` extents, placed by `origin`
    /// (xyz, rpy); `base` a collision triangle.
    fn write_tool_robot_urdf(path: &Path, half: [f64; 3], origin: ([f64; 3], [f64; 3])) {
        let dir = path.parent().unwrap();
        write_triangle_obj(&dir.join("meshes/robot/tri.obj"), 1.0);
        let [hx, hy, hz] = half;
        let mut obj = String::new();
        for i in 0..8 {
            let s = |bit: usize, h: f64| if i & (1 << bit) == 0 { -h } else { h };
            obj.push_str(&format!("v {} {} {}\n", s(0, hx), s(1, hy), s(2, hz)));
        }
        // Outward faces (OBJ indices are 1-based).
        for [a, b, c, d] in [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]] {
            obj.push_str(&format!("f {} {} {} {}\n", a + 1, b + 1, c + 1, d + 1));
        }
        std::fs::create_dir_all(dir.join("meshes/tool")).unwrap();
        std::fs::write(dir.join("meshes/tool/box.obj"), obj).unwrap();
        let ([x, y, z], [r, p, yaw]) = origin;
        let s = format!(
            r#"<?xml version="1.0"?>
<robot name="tooled">
  <link name="World"/>
  <link name="base">
    <collision><geometry><mesh filename="./meshes/robot/tri.obj"/></geometry></collision>
  </link>
  <link name="tool0">
    <collision><geometry><mesh filename="./meshes/robot/tri.obj"/></geometry></collision>
    <collision>
      <origin xyz="{x} {y} {z}" rpy="{r} {p} {yaw}"/>
      <geometry><mesh filename="./meshes/tool/box.obj"/></geometry>
    </collision>
  </link>
  <joint name="root_joint" type="fixed">
    <parent link="World"/>
    <child link="base"/>
  </joint>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="tool0"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="340282346638528859811704183484516925440" velocity="1"/>
  </joint>
</robot>
"#
        );
        std::fs::write(path, s).unwrap();
    }

    /// The numbers of `key`'s tuple in `text`.
    fn attr_tuple(text: &str, key: &str) -> Vec<f64> {
        let value = attr(text, key);
        value
            .trim_start_matches('(')
            .trim_end_matches(')')
            .split(',')
            .map(|v| v.trim().parse().unwrap())
            .collect()
    }

    // FR-159
    #[test]
    fn tool_payload_merges_into_the_tool_link() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("tooled.urdf");
        // In tool0's XY plane, so the merged tensor's only product is the yawed box's XY one.
        let (half, xyz, rpy) = ([0.2, 0.1, 0.3], [0.3, 0.0, 0.0], [0.0, 0.0, 0.5]);
        write_tool_robot_urdf(&urdf, half, (xyz, rpy));
        let mut robot = robot_at(urdf, [0.0; 3], 0.0);
        robot.payload = Some(crate::scene::Payload {
            mass_kg: 50.0,
            center_of_mass_m: None,
            inertia_kgm2: None,
            link: None,
        });
        let scene = scene_of(vec![robot.clone()], vec![]);
        let usda = compose_usda(dir.path(), &scene);
        let tool0 = block(&usda, "        def Xform \"tool0\" (");

        // By hand: 50 kg spread over the 0.4 × 0.2 × 0.6 box, centred at its collision origin and
        // turned by its yaw: the box tensor m·diag((b² + c²)/12, …) in the box axes. tool0 has no
        // inertial and its robot triangle encloses no volume, so the payload is all it weighs.
        let size = math3::scale(half, 2.0);
        let [a, b, c] = size;
        let origin = math3::from_translation_rpy(xyz, rpy);
        let box_tensor = math3::conjugate(
            &math3::rotation_of(&origin),
            &math3::diag([
                50.0 * (b * b + c * c) / 12.0,
                50.0 * (a * a + c * c) / 12.0,
                50.0 * (a * a + b * b) / 12.0,
            ]),
        );
        let payload = drives::Body {
            mass: 50.0,
            center_of_mass: xyz,
            inertia: box_tensor,
        };
        let link = Link {
            name: "tool0".into(),
            inertial: None,
            visuals: vec![],
            collisions: vec![],
        };
        let want = drives::link_mass_properties(
            &link,
            &drives::LinkMeshes::default(),
            Some(&payload),
            drives::DEFAULT_LINK_DENSITY_KG_M3,
            false,
            "t",
        );
        assert_eq!((want.mass, want.center_of_mass), (50.0, xyz));
        assert_eq!(attr(tool0, "physics:mass"), "50");
        let close = |got: Vec<f64>, want: &[f64]| {
            assert_eq!(got.len(), want.len());
            for (g, w) in got.iter().zip(want) {
                assert!((g - w).abs() < 2e-6, "{got:?} != {want:?}");
            }
        };
        close(attr_tuple(tool0, "physics:centerOfMass"), &want.center_of_mass);
        close(attr_tuple(tool0, "physics:diagonalInertia"), &want.diagonal_inertia);
        // The yawed box makes an XY product: principal axes turned about Z.
        let axes = want.principal_axes_wxyz();
        close(attr_tuple(tool0, "physics:principalAxes"), &axes);
        assert!(axes[3].abs() > 1e-3 && axes[1].abs() < 1e-9 && axes[2].abs() < 1e-9, "{axes:?}");
        // The payload is the tool link's alone: base keeps its own (no inertial, no volume, welded
        // to the root: 10 kg), on its own axes, which are authored too: the identity.
        let base = block(&usda, "        def Xform \"base\" (");
        assert_eq!(attr(base, "physics:mass"), "10");
        assert_eq!(attr(base, "physics:diagonalInertia"), "(1, 1, 1)");
        assert_eq!(attr(base, "physics:principalAxes"), "(1, 0, 0, 0)");
        // Every diagonal inertia comes with its principal axes.
        assert_eq!(
            usda.matches("physics:diagonalInertia").count(),
            usda.matches("quatf physics:principalAxes").count()
        );
        // The drive is tuned on the merged link: I about Z through the joint (0, 0, 0.5).
        let joint = block(&usda, "        def PhysicsRevoluteJoint \"j1\" (");
        let inertia = math3::conjugate(&want.principal_axes, &math3::diag(want.diagonal_inertia))[2][2]
            + 50.0 * (want.center_of_mass[0].powi(2) + want.center_of_mass[1].powi(2));
        let damping = 2.0 * 0.7 * (1.5e5 * 180.0 / std::f64::consts::PI * inertia).sqrt()
            * std::f64::consts::PI
            / 180.0;
        let got: f64 = attr(joint, "drive:angular:physics:damping").parse().unwrap();
        assert!((got - damping).abs() < 1e-5, "{got} != {damping}");
        // Deterministic.
        assert_eq!(usda, compose_usda(dir.path(), &scene));
    }

    // FR-119, NFR-108
    #[test]
    fn welded_root_links_filter_their_pedestal_and_the_ground() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("tooled.urdf");
        write_tool_robot_urdf(&urdf, [0.1, 0.1, 0.1], ([0.0; 3], [0.0; 3]));
        let mut robot = robot_at(urdf, [1.0, 2.0, 0.0], 0.0);
        robot.robot_id = "7R1".into();
        robot.pedestal = Some(crate::scene::Pedestal {
            height_mm: 600.0,
            diameter_mm: 1000.0,
        });
        let welded = [
            ("world_link", "[\"PhysicsRigidBodyAPI\", \"PhysicsMassAPI\", \"PhysicsFilteredPairsAPI\"]"),
            (
                "base",
                "[\"PhysicsRigidBodyAPI\", \"PhysicsMassAPI\", \"PhysxContactReportAPI\", \"PhysicsFilteredPairsAPI\"]",
            ),
        ];
        let usda = compose_usda(dir.path(), &scene_of(vec![robot.clone()], vec![]));
        let rel = "            rel physics:filteredPairs = [</World/pedestals/pedestal__7R1/socket>, </Environment/GroundCollider>]\n";
        // The root and the link welded to it filter the pedestal and the ground; the moving
        // link doesn't.
        for (link, schemas) in welded {
            let prim = block(&usda, &format!("        def Xform \"{link}\" ("));
            assert!(prim.contains(&format!("prepend apiSchemas = {schemas}")), "{prim}");
            assert!(prim.contains(rel), "{prim}");
        }
        let tool0 = block(&usda, "        def Xform \"tool0\" (");
        assert!(!tool0.contains("FilteredPairs") && !tool0.contains("filteredPairs"));
        // The targets are the pedestal's collider and the ground's.
        assert!(usda.contains("        def Xform \"pedestal__7R1\"\n"));
        assert!(block(&usda, "        def Xform \"pedestal__7R1\"").contains("def Cylinder \"socket\""));
        assert_eq!(GROUND_COLLIDER_PATH, "/Environment/GroundCollider");
        let environment = block(&usda, "def Xform \"Environment\"");
        assert!(environment.contains(
            "\n    def Plane \"GroundCollider\" (\n        prepend apiSchemas = [\"PhysicsCollisionAPI\"]\n    )\n"
        ));
        assert_eq!(usda.matches("filteredPairs").count(), 2);

        // On the floor (no pedestal): the ground alone.
        robot.pedestal = None;
        let usda = compose_usda(dir.path(), &scene_of(vec![robot], vec![]));
        for (link, schemas) in welded {
            let prim = block(&usda, &format!("        def Xform \"{link}\" ("));
            assert!(prim.contains(&format!("prepend apiSchemas = {schemas}")), "{prim}");
            assert!(
                prim.contains("            rel physics:filteredPairs = [</Environment/GroundCollider>]\n"),
                "{prim}"
            );
        }
        assert_eq!(usda.matches("filteredPairs").count(), 2);
    }

    #[test]
    fn fixture_with_actuated_joint_rejected() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("bad_fixture.urdf");
        write_minimal_urdf(&urdf, "bad", 2);
        let manifest = dir.path().join("scene.yaml");
        write_dummy_manifest(&manifest);

        let scene = SceneDefinition {
            name: "t".into(),
            description: "".into(),
            robots: vec![],
            fixtures: vec![FixturePlacement {
                id: "F1".into(),
                urdf,
                position: [0.0; 3],
                orientation_deg: [0.0; 3],
            }],
            floor_plan: None,
        };
        let out = dir.path().join("scene.zip");
        let err = compose_stage(&scene, &manifest, &out).unwrap_err();
        match err {
            FittingRoomError::Validation(msg) => assert!(msg.contains("non-fixed joint")),
            other => panic!("expected Validation, got {:?}", other),
        }
    }

    #[test]
    fn fixture_with_fixed_joint_accepted() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("fix.urdf");
        write_fixed_only_urdf(&urdf, "fix");
        let manifest = dir.path().join("scene.yaml");
        write_dummy_manifest(&manifest);

        let scene = SceneDefinition {
            name: "t".into(),
            description: "".into(),
            robots: vec![],
            fixtures: vec![FixturePlacement {
                id: "FX1".into(),
                urdf,
                position: [10.0, 20.0, 0.0],
                orientation_deg: [0.0; 3],
            }],
            floor_plan: None,
        };
        let out = dir.path().join("scene.zip");
        let report = compose_stage(&scene, &manifest, &out).unwrap();
        assert_eq!(report.fixture_count, 1);
        let usda = read_zip_entry(&out, "scene.usda");
        assert!(usda.contains("def Scope \"fixtures\""));
        assert!(usda.contains("def Xform \"FX1\""));
    }

    #[test]
    fn unsupported_format_rejected() {
        let dir = tempfile::tempdir().unwrap();
        let manifest = dir.path().join("scene.yaml");
        write_dummy_manifest(&manifest);

        let scene = SceneDefinition {
            name: "t".into(),
            description: "".into(),
            robots: vec![],
            fixtures: vec![],
            floor_plan: None,
        };
        let out = dir.path().join("scene.usda");
        let err = compose_stage(&scene, &manifest, &out).unwrap_err();
        assert!(matches!(err, FittingRoomError::UnsupportedFormat(_)));
    }

    /// Two lines (and a zero-length one) and a footprint.
    fn small_floor_plan() -> FloorPlan {
        FloorPlan {
            segments: vec![
                [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0]],
                [[5.0, 5.0, 0.0], [5.0, 5.0, 0.0]],
            ],
            footprint_segments: vec![[[2.0, 2.0, 0.0], [3.0, 2.0, 0.0]]],
            ..FloorPlan::default()
        }
    }

    // FR-107
    #[test]
    fn floor_plan_emits_flat_strip_meshes() {
        let dir = tempfile::tempdir().unwrap();
        let mut scene = scene_of(vec![], vec![]);
        scene.floor_plan = Some(small_floor_plan());
        let manifest = dir.path().join("scene.yaml");
        write_dummy_manifest(&manifest);
        let out = dir.path().join("scene.zip");
        let report = compose_stage(&scene, &manifest, &out).unwrap();
        assert_eq!(report.floor_plan_segments, 4);
        let usda = read_zip_entry(&out, "scene.usda");

        // 2.5 cm strips (two triangles each), facing up, 2 mm above the ground, in one matte
        // slate material; the zero-length segment is dropped.
        let normals = |n: usize| vec!["(0, 0, 1)"; n].join(", ");
        let expected = format!(
            "    def Xform \"floor_plan\"\n\
             \x20   {{\n\
             \x20       double3 xformOp:translate = (0, 0, 0.002)\n\
             \x20       uniform token[] xformOpOrder = [\"xformOp:translate\"]\n\
             \n\
             \x20       def Scope \"Looks\"\n\
             \x20       {{\n\
             \x20           def Material \"M_floorplan\"\n\
             \x20           {{\n\
             \x20               token outputs:surface.connect = </World/floor_plan/Looks/M_floorplan/Shader.outputs:surface>\n\
             \n\
             \x20               def Shader \"Shader\"\n\
             \x20               {{\n\
             \x20                   uniform token info:id = \"UsdPreviewSurface\"\n\
             \x20                   color3f inputs:diffuseColor = (0.046665, 0.061246, 0.0865)\n\
             \x20                   float inputs:metallic = 0\n\
             \x20                   float inputs:roughness = 0.9\n\
             \x20                   token outputs:surface\n\
             \x20               }}\n\
             \x20           }}\n\
             \x20       }}\n\
             \n\
             \x20       def Mesh \"lines\" (\n\
             \x20           prepend apiSchemas = [\"MaterialBindingAPI\"]\n\
             \x20       )\n\
             \x20       {{\n\
             \x20           uniform bool doubleSided = 1\n\
             \x20           float3[] extent = [(0, -0.0125, 0), (1.0125, 1, 0)]\n\
             \x20           int[] faceVertexCounts = [3, 3, 3, 3]\n\
             \x20           int[] faceVertexIndices = [0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7]\n\
             \x20           rel material:binding = </World/floor_plan/Looks/M_floorplan>\n\
             \x20           normal3f[] normals = [{}] (\n\
             \x20               interpolation = \"vertex\"\n\
             \x20           )\n\
             \x20           point3f[] points = [(0, 0.0125, 0), (0, -0.0125, 0), (1, -0.0125, 0), (1, 0.0125, 0), (0.9875, 0, 0), (1.0125, 0, 0), (1.0125, 1, 0), (0.9875, 1, 0)]\n\
             \x20           uniform token subdivisionScheme = \"none\"\n\
             \x20       }}\n\
             \n\
             \x20       def Mesh \"footprints\" (\n\
             \x20           prepend apiSchemas = [\"MaterialBindingAPI\"]\n\
             \x20       )\n\
             \x20       {{\n\
             \x20           uniform bool doubleSided = 1\n\
             \x20           float3[] extent = [(2, 1.9875, 0), (3, 2.0125, 0)]\n\
             \x20           int[] faceVertexCounts = [3, 3]\n\
             \x20           int[] faceVertexIndices = [0, 1, 2, 0, 2, 3]\n\
             \x20           rel material:binding = </World/floor_plan/Looks/M_floorplan>\n\
             \x20           normal3f[] normals = [{}] (\n\
             \x20               interpolation = \"vertex\"\n\
             \x20           )\n\
             \x20           point3f[] points = [(2, 2.0125, 0), (2, 1.9875, 0), (3, 1.9875, 0), (3, 2.0125, 0)]\n\
             \x20           uniform token subdivisionScheme = \"none\"\n\
             \x20       }}\n\
             \x20   }}\n",
            normals(8),
            normals(4),
        );
        assert_eq!(block(&usda, "    def Xform \"floor_plan\""), expected);
        // Drawn, not a guide (Kit hides guides), and not a collider.
        let floor_plan = block(&usda, "    def Xform \"floor_plan\"");
        for gone in ["purpose", "guide", "Collision", "physics:", "displayColor"] {
            assert!(!floor_plan.contains(gone), "{gone}");
        }
        // Inside /World, before /Environment.
        assert!(usda.find("def Xform \"World\"").unwrap() < usda.find("    def Xform \"floor_plan\"").unwrap());
        assert!(usda.find("    def Xform \"floor_plan\"").unwrap() < usda.find("def Xform \"Environment\"").unwrap());

        // A layer without a segment of positive length gets no mesh, a plan without any no prim.
        scene.floor_plan = Some(FloorPlan {
            segments: vec![[[5.0, 5.0, 0.0], [5.0, 5.0, 0.0]]],
            footprint_segments: vec![[[2.0, 2.0, 0.0], [3.0, 2.0, 0.0]]],
            ..FloorPlan::default()
        });
        let usda = compose_usda(dir.path(), &scene);
        let floor_plan = block(&usda, "    def Xform \"floor_plan\"");
        assert!(!floor_plan.contains("\"lines\"") && floor_plan.contains("def Mesh \"footprints\" ("));
        scene.floor_plan = Some(FloorPlan {
            segments: vec![[[5.0, 5.0, 0.0], [5.0, 5.0, 0.0]]],
            ..FloorPlan::default()
        });
        assert!(!compose_usda(dir.path(), &scene).contains("floor_plan"));
    }

    // FR-107, FR-108
    #[test]
    fn floor_plan_sits_after_the_cell_and_outside_the_rig_bounds() {
        let dir = tempfile::tempdir().unwrap();
        let robot_urdf = dir.path().join("r2.urdf");
        write_minimal_urdf(&robot_urdf, "r2", 2);
        let fixture_urdf = dir.path().join("fx/fx.urdf");
        write_meshed_fixture_urdf(&fixture_urdf);
        let mut robot = robot_at(robot_urdf, [1.0, 2.0, 0.0], 45.0);
        robot.pedestal = Some(crate::scene::Pedestal {
            height_mm: 600.0,
            diameter_mm: 800.0,
        });
        let mut scene = scene_of(vec![robot], vec![fixture_at(fixture_urdf, [5.0, 0.0, 0.0], 90.0)]);
        let without = compose_usda(dir.path(), &scene);
        let bounds = bounds_of(&scene);
        // A plan reaching far beyond the cell.
        scene.floor_plan = Some(FloorPlan {
            segments: vec![[[-40.0, -30.0, 0.0], [60.0, 50.0, 0.0]]],
            ..FloorPlan::default()
        });
        let with = compose_usda(dir.path(), &scene);
        // After the robots, fixtures and pedestals; before the materials, graphs and cameras
        // (/World's children, each at the start of its line).
        let at = |usda: &str, head: &str| usda.find(head).unwrap_or_else(|| panic!("no {head}"));
        let order = [
            "\n    def Scope \"robots\" (\n",
            "\n    def Scope \"fixtures\" (\n",
            "\n    def Scope \"pedestals\"\n",
            "\n    def Xform \"floor_plan\"\n",
            "\n    def Scope \"Looks\"\n",
            "\n    def OmniGraph ",
            "\n    def Scope \"Cameras\"\n",
        ];
        let positions: Vec<usize> = order.iter().map(|head| at(&with, head)).collect();
        assert!(positions.windows(2).all(|p| p[0] < p[1]), "{positions:?}");
        // The rig frames the same box: the floor plan doesn't count.
        assert_eq!(bounds_of(&scene), bounds);
        assert_eq!(
            block(&with, "    def Scope \"Cameras\""),
            block(&without, "    def Scope \"Cameras\"")
        );
        // Nor does the ground: it lies under the box, the same with or without the plan, only
        // larger to reach under the plan too.
        let ground = |usda: &str| block(usda, "    def Mesh \"Ground\" (").to_owned();
        let center = bounds.unwrap().center();
        let translate = format!("double3 xformOp:translate = ({}, {}, 0)", f(center[0]), f(center[1]));
        assert!(ground(&without).contains(&translate) && ground(&with).contains(&translate));
        assert_ne!(ground(&without), ground(&with));
    }

    #[test]
    fn manifest_yaml_in_zip() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("r.urdf");
        write_minimal_urdf(&urdf, "r", 1);
        let manifest = dir.path().join("demo-cell.yaml");
        std::fs::write(&manifest, "name: \"CELL01\"\nrobots: []\nfixtures: []\n").unwrap();

        let scene = SceneDefinition {
            name: "CELL01".into(),
            description: "".into(),
            robots: vec![RobotPlacement {
                robot_id: "R".into(),
                dressed_urdf: urdf,
                position: [0.0; 3],
                orientation_deg: [0.0; 3],
                home_pose_deg: None,
                pedestal: None,
                payload: None,
                link_density_kg_m3: None,
            }],
            fixtures: vec![],
            floor_plan: None,
        };
        let out = dir.path().join("demo-cell.zip");
        let report = compose_stage(&scene, &manifest, &out).unwrap();
        assert_eq!(report.manifest_name, "demo-cell.yaml");
        assert_eq!(report.usda_name, "demo-cell.usda");

        let yaml = read_zip_entry(&out, "demo-cell.yaml");
        assert!(yaml.contains("CELL01"));
    }

    #[test]
    fn abc_to_quat_zero() {
        let q = abc_deg_to_quat([0.0, 0.0, 0.0]);
        assert!(q[0].abs() < 1e-9);
        assert!(q[1].abs() < 1e-9);
        assert!(q[2].abs() < 1e-9);
        assert!((q[3] - 1.0).abs() < 1e-9);
    }

    #[test]
    fn sanitize_handles_special_chars() {
        assert_eq!(sanitize_prim_name("1234R01"), "_1234R01");
        assert_eq!(sanitize_prim_name("A-B"), "A_B");
        assert_eq!(sanitize_prim_name("normal_name"), "normal_name");
    }

    #[test]
    fn joint_axes_keep_their_sign_and_direction() {
        let joint = |axis: [f64; 3]| Joint {
            name: "j".into(),
            joint_type: JointType::Revolute,
            parent: "a".into(),
            child: "b".into(),
            origin: Origin::default(),
            axis,
            limit: None,
        };
        // +X, +Y, +Z (any length): their own token, the URDF's joint frames.
        for (axis, token) in [
            ([0.0, 0.0, 1.0], "Z"),
            ([2.0, 0.0, 0.0], "X"),
            ([0.0, 0.5, 0.0], "Y"),
        ] {
            assert_eq!(joint_axis(&joint(axis)), JointAxis { token, frame: None });
        }
        // Negative and skewed axes (and a zero one, URDF's default +X... no: X itself) keep their
        // direction: the token X, both frames turning +X onto the unit axis.
        let s = 0.5f64.sqrt();
        for axis in [
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -2.0, 0.0],
            [s, 0.0, s],
            [0.3, -0.4, 0.5],
        ] {
            let got = joint_axis(&joint(axis));
            assert_eq!(got.token, "X");
            let frame = got.frame.expect("folded");
            let unit = math3::scale(axis, 1.0 / math3::norm(axis));
            let x = math3::rotate(&frame, [1.0, 0.0, 0.0]);
            for k in 0..3 {
                assert!((x[k] - unit[k]).abs() < 1e-12, "{axis:?}: {x:?}");
            }
        }
        // A zero axis is URDF's default, +X.
        assert_eq!(
            joint_axis(&joint([0.0; 3])),
            JointAxis {
                token: "X",
                frame: None
            }
        );
    }

    /// The rigid transform of `localPos`/`localRot` (w, x, y, z) as written.
    fn joint_frame(pos: &[f64], rot: &[f64]) -> Mat4 {
        let [w, x, y, z] = [rot[0], rot[1], rot[2], rot[3]];
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
                pos[0],
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
                pos[1],
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
                pos[2],
            ],
            [0.0, 0.0, 0.0, 1.0],
        ]
    }

    /// `a` and `b` agree to `tol` in every entry.
    fn assert_mat4_near(a: &Mat4, b: &Mat4, tol: f64, what: &str) {
        for i in 0..4 {
            for j in 0..4 {
                assert!((a[i][j] - b[i][j]).abs() < tol, "{what}: {a:?} != {b:?}");
            }
        }
    }

    // FR-157
    #[test]
    fn negative_and_skewed_axes_fold_into_the_joint_frames() {
        let dir = tempfile::tempdir().unwrap();
        write_triangle_obj(&dir.path().join("meshes/tri.obj"), 1.0);
        let urdf = dir.path().join("axes.urdf");
        let link = |name: &str| {
            format!("  <link name=\"{name}\">\n    <collision><geometry><mesh filename=\"./meshes/tri.obj\"/></geometry></collision>\n  </link>\n")
        };
        std::fs::write(
            &urdf,
            format!(
                "<?xml version=\"1.0\"?>\n<robot name=\"axes\">\n{}{}{}  <joint name=\"j_neg\" type=\"revolute\">\n    <parent link=\"base\"/>\n    <child link=\"arm\"/>\n    <origin xyz=\"0 0 0.5\" rpy=\"0 0 0.3\"/>\n    <axis xyz=\"0 0 -1\"/>\n    <limit lower=\"-1\" upper=\"2\" effort=\"100\" velocity=\"1\"/>\n  </joint>\n  <joint name=\"j_skew\" type=\"prismatic\">\n    <parent link=\"arm\"/>\n    <child link=\"slider\"/>\n    <origin xyz=\"1 0 0\" rpy=\"0 0 0\"/>\n    <axis xyz=\"0.6 0 0.8\"/>\n    <limit lower=\"0\" upper=\"1.5\" effort=\"100\" velocity=\"0.5\"/>\n  </joint>\n</robot>\n",
                link("base"),
                link("arm"),
                link("slider"),
            ),
        )
        .unwrap();
        let mut robot = robot_at(urdf, [0.0; 3], 0.0);
        robot.home_pose_deg = Some(vec![30.0, 0.4]);
        let usda = compose_usda(dir.path(), &scene_of(vec![robot], vec![]));

        // The URDF's convention throughout: limits as written (−1..2 rad), target and state the
        // home angle — not mirrored for the −Z axis.
        let neg = block(&usda, "        def PhysicsRevoluteJoint \"j_neg\" (");
        assert_eq!(attr(neg, "physics:axis"), "\"X\"");
        assert_eq!(attr(neg, "physics:lowerLimit"), "-57.29578");
        assert_eq!(attr(neg, "physics:upperLimit"), "114.591559");
        assert_eq!(attr(neg, "drive:angular:physics:targetPosition"), "30");
        assert_eq!(attr(neg, "state:angular:physics:position"), "30");
        // The frames: body0's the origin turned by the fold, body1's the fold alone, which takes
        // +X onto −Z.
        let fold = math3::rotation_taking_x_to([0.0, 0.0, -1.0]);
        let rot1 = attr_tuple(neg, "physics:localRot1");
        let q1 = math3::quat_of(&fold);
        for (g, w) in rot1.iter().zip([q1[3], q1[0], q1[1], q1[2]]) {
            assert!((g - w).abs() < 1e-6, "{rot1:?}");
        }
        assert_eq!(attr(neg, "physics:localPos1"), "(0, 0, 0)");
        let frame0 = joint_frame(
            &attr_tuple(neg, "physics:localPos0"),
            &attr_tuple(neg, "physics:localRot0"),
        );
        let frame1 = joint_frame(&[0.0; 3], &rot1);
        let x1 = math3::rotate(&frame1, [1.0, 0.0, 0.0]);
        assert!((x1[2] + 1.0).abs() < 1e-6, "{x1:?}");
        // The bind pose turns the arm −30° about the parent's Z (+30° about −Z), and the joint
        // frames agree with it at the state: base · frame0 · Rx(30°) = arm · frame1.
        let origin = math3::from_translation_rpy([0.0, 0.0, 0.5], [0.0, 0.0, 0.3]);
        let arm_pose = math3::multiply(
            &origin,
            &math3::rotation_about_axis([0.0, 0.0, -1.0], 30f64.to_radians()),
        );
        let arm = block(&usda, "        def Xform \"arm\" (");
        assert!(arm.contains(&transform_line(&arm_pose, 12)), "{arm}");
        let via_joint = math3::multiply(
            &frame0,
            &math3::rotation_about_axis([1.0, 0.0, 0.0], 30f64.to_radians()),
        );
        assert_mat4_near(
            &via_joint,
            &math3::multiply(&arm_pose, &frame1),
            1e-5,
            "j_neg frames",
        );

        // The skewed prismatic axis (0.6, 0, 0.8): the token X, metres as written, the slider slid
        // 0.4 m along the axis.
        let skew = block(&usda, "        def PhysicsPrismaticJoint \"j_skew\" (");
        assert!(skew.contains("prepend apiSchemas = [\"PhysicsDriveAPI:linear\", \"PhysicsJointStateAPI:linear\", \"PhysxJointAPI\"]"), "{skew}");
        assert_eq!(attr(skew, "physics:axis"), "\"X\"");
        assert_eq!(attr(skew, "physics:lowerLimit"), "0");
        assert_eq!(attr(skew, "physics:upperLimit"), "1.5");
        assert_eq!(attr(skew, "drive:linear:physics:targetPosition"), "0.4");
        assert_eq!(attr(skew, "state:linear:physics:position"), "0.4");
        assert_eq!(attr(skew, "physxJoint:maxJointVelocity"), "0.5");
        let slider_pose = math3::multiply(
            &arm_pose,
            &math3::from_translation_rpy([1.0 + 0.24, 0.0, 0.32], [0.0; 3]),
        );
        let slider = block(&usda, "        def Xform \"slider\" (");
        assert!(
            slider.contains(&transform_line(&slider_pose, 12)),
            "{slider}"
        );
        let frame0 = joint_frame(
            &attr_tuple(skew, "physics:localPos0"),
            &attr_tuple(skew, "physics:localRot0"),
        );
        let frame1 = joint_frame(&[0.0; 3], &attr_tuple(skew, "physics:localRot1"));
        let slid = math3::multiply(
            &math3::multiply(&arm_pose, &frame0),
            &math3::from_translation_rpy([0.4, 0.0, 0.0], [0.0; 3]),
        );
        assert_mat4_near(
            &slid,
            &math3::multiply(&slider_pose, &frame1),
            1e-5,
            "j_skew frames",
        );
        // A linear drive: the moved weight over 1 mm (the slider: a frame link's 0.1 kg, its
        // triangle enclosing nothing): 981 N/m.
        assert_eq!(attr(skew, "drive:linear:physics:stiffness"), "981");
    }

    // FR-111, FR-112, FR-113
    #[test]
    fn primitives_compose_as_usd_geom_prims() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("prims.urdf");
        std::fs::write(
            &urdf,
            r#"<?xml version="1.0"?>
<robot name="prims">
  <link name="World"/>
  <link name="base">
    <visual><geometry><cylinder radius="0.2" length="1"/></geometry></visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry><box size="1 2 3"/></geometry>
    </collision>
  </link>
  <link name="arm">
    <visual><geometry><sphere radius="0.3"/></geometry></visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5707963267948966 0 0"/>
      <geometry><cylinder radius="0.1" length="0.6"/></geometry>
    </collision>
  </link>
  <joint name="root_joint" type="fixed">
    <parent link="World"/>
    <child link="base"/>
  </joint>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <origin xyz="0 0 2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="100" velocity="1"/>
  </joint>
</robot>
"#,
        )
        .unwrap();
        let manifest = dir.path().join("scene.yaml");
        write_dummy_manifest(&manifest);
        let scene = scene_of(vec![robot_at(urdf, [0.0; 3], 0.0)], vec![]);
        let out = dir.path().join("scene.zip");
        let opts = ComposeOptions {
            full_robots: true,
            full_tools: true,
        };
        let report = compose_stage_with_options(&scene, &manifest, &out, &opts).unwrap();
        // Primitives aren't inline meshes.
        assert_eq!(report.mesh_count, 0);
        let usda = read_zip_entry(&out, "scene.usda");
        assert!(!usda.contains("# inline"));
        let base = block(&usda, "        def Xform \"base\" (");
        // The box collider: a unit Cube scaled to its size, at its origin; a collider PhysX takes
        // as it is (no mesh collision API, no convex hull); hidden behind the visuals.
        let cube = block(base, "            def Cube \"collision_0\" (");
        assert!(
            cube.contains("prepend apiSchemas = [\"PhysicsCollisionAPI\"]\n"),
            "{cube}"
        );
        assert!(cube.contains("uniform token purpose = \"guide\""), "{cube}");
        assert!(
            !cube.contains("approximation") && !cube.contains("MeshCollision"),
            "{cube}"
        );
        assert_eq!(attr(cube, "size"), "1");
        assert_eq!(
            attr(cube, "extent"),
            "[(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]"
        );
        assert_eq!(attr(cube, "xformOp:translate"), "(0, 0, 0.5)");
        assert_eq!(attr(cube, "xformOp:scale"), "(1, 2, 3)");
        assert_eq!(
            attr(cube, "xformOpOrder"),
            "[\"xformOp:translate\", \"xformOp:orient\", \"xformOp:scale\"]"
        );
        // The cylinder visual: a Z Cylinder, matte.
        let cylinder = block(base, "            def Cylinder \"visual_0\" (");
        assert!(
            cylinder.contains("prepend apiSchemas = [\"MaterialBindingAPI\"]"),
            "{cylinder}"
        );
        assert!(cylinder.contains("rel material:binding = </World/Looks/factoryos_default>"));
        assert_eq!(attr(cylinder, "axis"), "\"Z\"");
        assert_eq!(attr(cylinder, "height"), "1");
        assert_eq!(attr(cylinder, "radius"), "0.2");
        assert_eq!(
            attr(cylinder, "extent"),
            "[(-0.2, -0.2, -0.5), (0.2, 0.2, 0.5)]"
        );
        assert!(!cylinder.contains("subdivisionScheme") && !cylinder.contains("xformOp:scale"));
        let arm = block(&usda, "        def Xform \"arm\" (");
        let sphere = block(arm, "            def Sphere \"visual_0\" (");
        assert_eq!(attr(sphere, "radius"), "0.3");
        assert!(
            arm.contains("            def Cylinder \"collision_0\" (\n"),
            "{arm}"
        );
        // Weighed analytically: the arm's cylinder (π·0.1²·0.6 m³) at the link density; it
        // reports contacts like any collider.
        let mass: f64 = attr(arm, "physics:mass").parse().unwrap();
        assert!(
            (mass - 1200.0 * std::f64::consts::PI * 0.01 * 0.6).abs() < 1e-4,
            "{mass}"
        );
        assert!(
            arm.contains("PhysxContactReportAPI")
                && arm.contains("def IsaacContactSensor \"ContactSensor_node\"")
        );
        let base_mass: f64 = attr(base, "physics:mass").parse().unwrap();
        assert!((base_mass - 1200.0 * 6.0).abs() < 1e-6, "{base_mass}");
        // The camera rig frames the colliders: the base's box (z from 0.5 − 1.5 to 0.5 + 1.5) and
        // the arm's cylinder, turned onto Y, 2 m up (z 2 ± 0.1).
        assert_bounds(bounds_of(&scene), [-0.5, -1.0, -1.0], [0.5, 1.0, 2.1]);

        // Lean: the primitive colliders show, matte.
        let usda = compose_usda(dir.path(), &scene);
        let cube = block(&usda, "            def Cube \"collision_0\" (");
        assert!(
            cube.contains("prepend apiSchemas = [\"PhysicsCollisionAPI\", \"MaterialBindingAPI\"]"),
            "{cube}"
        );
        assert!(!cube.contains("guide"));
        assert!(!usda.contains("def Cylinder \"visual_0\""));
    }

    /// A robot `World` → (fixed) `base` → (revolute about Z) `link_6` → (fixed, 0.1 m up)
    /// `flange` → (fixed, 0.05 m up) `gripper` → (fixed) `tool0`. `link_6` carries a robot box
    /// under `meshes/robot/` and a URDF inertial; the gripper's visual and collision box
    /// (0.2 × 0.2 × 0.4 m, centred 0.2 m above it) live under `gripper/`, not `meshes/tool/`;
    /// `flange` and `tool0` are frames.
    fn write_vendor_tool_urdf(path: &Path) {
        let dir = path.parent().unwrap();
        write_box_obj(&dir.join("meshes/robot/l6.obj"), [0.1, 0.1, 0.1]);
        write_box_obj(&dir.join("gripper/body.obj"), [0.1, 0.1, 0.2]);
        std::fs::write(
            path,
            r#"<?xml version="1.0"?>
<robot name="vendor">
  <link name="World"/>
  <link name="base"/>
  <link name="link_6">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="12"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
    <visual><geometry><mesh filename="./meshes/robot/l6.obj"/></geometry></visual>
    <collision><geometry><mesh filename="./meshes/robot/l6.obj"/></geometry></collision>
  </link>
  <link name="flange"/>
  <link name="gripper">
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry><mesh filename="./gripper/body.obj"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry><mesh filename="./gripper/body.obj"/></geometry>
    </collision>
  </link>
  <link name="tool0"/>
  <joint name="root_joint" type="fixed"><parent link="World"/><child link="base"/></joint>
  <joint name="a6" type="revolute">
    <parent link="base"/>
    <child link="link_6"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="100" velocity="1"/>
  </joint>
  <joint name="mount" type="fixed">
    <parent link="link_6"/>
    <child link="flange"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
  <joint name="adapter" type="fixed">
    <parent link="flange"/>
    <child link="gripper"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>
  <joint name="tip" type="fixed">
    <parent link="gripper"/>
    <child link="tool0"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
  </joint>
</robot>
"#,
        )
        .unwrap();
    }

    // FR-113, FR-159
    #[test]
    fn a_named_payload_link_carries_a_tool_outside_meshes_tool() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("vendor.urdf");
        write_vendor_tool_urdf(&urdf);
        let mass_of = |usda: &str, link: &str| -> f64 {
            attr(
                block(usda, &format!("        def Xform \"{link}\" (")),
                "physics:mass",
            )
            .parse()
            .unwrap()
        };
        let mut robot = robot_at(urdf, [0.0; 3], 0.0);
        robot.home_pose_deg = Some(vec![0.0]);

        // Without a payload: the frames a joint moves weigh 0.1 kg, the welded root 10 kg; the
        // gripper is robot geometry (no meshes/tool/), weighed at the density.
        let usda = compose_usda(dir.path(), &scene_of(vec![robot.clone()], vec![]));
        assert_eq!(mass_of(&usda, "world_link"), 10.0);
        assert_eq!(mass_of(&usda, "base"), 10.0);
        assert_eq!(mass_of(&usda, "flange"), 0.1);
        assert_eq!(mass_of(&usda, "tool0"), 0.1);
        assert_eq!(mass_of(&usda, "link_6"), 12.0);
        assert!((mass_of(&usda, "gripper") - 1200.0 * 0.016).abs() < 1e-6);

        // Named on the flange: the tool is the flange and the links fixed below it. The payload
        // is spread over the gripper's box (in the flange's frame: centred 0.05 + 0.2 m up), its
        // geometry isn't weighed again (the gripper is a frame then), and link_6 keeps its own.
        robot.payload = Some(crate::scene::Payload {
            mass_kg: 20.0,
            center_of_mass_m: None,
            inertia_kgm2: None,
            link: Some("flange".into()),
        });
        let scene = scene_of(vec![robot.clone()], vec![]);
        assert!(validate_scene_ok(&scene));
        let usda = compose_usda(dir.path(), &scene);
        let flange = block(&usda, "        def Xform \"flange\" (");
        assert_eq!(attr(flange, "physics:mass"), "20");
        assert_eq!(attr(flange, "physics:centerOfMass"), "(0, 0, 0.25)");
        let [a, b, c] = [0.2, 0.2, 0.4];
        let want = [
            20.0 * (b * b + c * c) / 12.0,
            20.0 * (a * a + c * c) / 12.0,
            20.0 * (a * a + b * b) / 12.0,
        ];
        for (g, w) in attr_tuple(flange, "physics:diagonalInertia")
            .iter()
            .zip(want)
        {
            assert!((g - w).abs() < 1e-6, "{flange}");
        }
        assert_eq!(mass_of(&usda, "gripper"), 0.1);
        assert_eq!(mass_of(&usda, "tool0"), 0.1);
        assert_eq!(mass_of(&usda, "link_6"), 12.0);
        // Lean: the tool's visual stays (it is the tool's, by the named link), the arm's goes;
        // the tool's collider hides behind its visual.
        let gripper = block(&usda, "        def Xform \"gripper\" (");
        assert!(gripper.contains("def Mesh \"visual_0\""), "{gripper}");
        assert!(block(gripper, "            def Mesh \"collision_0\" (")
            .contains("purpose = \"guide\""));
        assert!(!block(&usda, "        def Xform \"link_6\" (").contains("def Mesh \"visual_0\""));
        // The drive moves the payload: a6 turns link_6 (Izz 0.05), the payload on the flange and
        // the gripper's and tool0's frames (1e-4 kg·m² each), all centred on its Z axis.
        let a6 = block(&usda, "        def PhysicsRevoluteJoint \"a6\" (");
        let damping: f64 = attr(a6, "drive:angular:physics:damping").parse().unwrap();
        let inertia = 0.05 + want[2] + 2.0 * 1e-4;
        let expected = 2.0
            * 0.7
            * (1.5e5 * 180.0 / std::f64::consts::PI * inertia).sqrt()
            * std::f64::consts::PI
            / 180.0;
        assert!((damping - expected).abs() < 1e-4, "{damping} != {expected}");

        // Unnamed: no meshes/tool/ anywhere, so the deepest leaf (tool0) carries it as a point
        // mass (a 5 cm sphere's inertia, never zero), and the gripper weighs its box again.
        robot.payload = Some(crate::scene::Payload {
            link: None,
            ..robot.payload.clone().unwrap()
        });
        let usda = compose_usda(dir.path(), &scene_of(vec![robot], vec![]));
        let tool0 = block(&usda, "        def Xform \"tool0\" (");
        assert_eq!(attr(tool0, "physics:mass"), "20");
        assert_eq!(attr(tool0, "physics:diagonalInertia"), "(0.02, 0.02, 0.02)");
        assert!((mass_of(&usda, "gripper") - 1200.0 * 0.016).abs() < 1e-6);
    }

    /// Whether `scene` validates without errors.
    fn validate_scene_ok(scene: &SceneDefinition) -> bool {
        let errors = crate::scene::validate_scene(scene);
        assert!(errors.is_empty(), "{errors:?}");
        true
    }

    // --- Scene globals: camera rig, lights, physics scene, layer metadata, kinds ---

    fn write_triangle_obj(path: &Path, size: f64) {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(
            path,
            format!("v 0 0 0\nv {size} 0 0\nv 0 {size} 0\nf 1 2 3\n"),
        )
        .unwrap();
    }

    /// A fixture: link `base` with a visual triangle 1 m up, fixed to link
    /// `plate` 2 m along X, which carries a collision triangle.
    fn write_meshed_fixture_urdf(path: &Path) {
        write_triangle_obj(&path.parent().unwrap().join("meshes/tri.obj"), 1.0);
        let s = r#"<?xml version="1.0"?>
<robot name="fx">
  <link name="base">
    <visual>
      <origin xyz="0 0 1" rpy="0 0 0"/>
      <geometry><mesh filename="./meshes/tri.obj"/></geometry>
    </visual>
  </link>
  <link name="plate">
    <collision>
      <geometry><mesh filename="./meshes/tri.obj"/></geometry>
    </collision>
  </link>
  <joint name="j_fix" type="fixed">
    <parent link="base"/>
    <child link="plate"/>
    <origin xyz="2 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"#;
        std::fs::write(path, s).unwrap();
    }

    fn robot_at(urdf: PathBuf, position: [f64; 3], yaw_deg: f64) -> RobotPlacement {
        RobotPlacement {
            robot_id: "R".into(),
            dressed_urdf: urdf,
            position,
            orientation_deg: [yaw_deg, 0.0, 0.0],
            home_pose_deg: None,
            pedestal: None,
            payload: None,
            link_density_kg_m3: None,
        }
    }

    fn fixture_at(urdf: PathBuf, position: [f64; 3], yaw_deg: f64) -> FixturePlacement {
        FixturePlacement {
            id: "FX1".into(),
            urdf,
            position,
            orientation_deg: [yaw_deg, 0.0, 0.0],
        }
    }

    fn scene_of(robots: Vec<RobotPlacement>, fixtures: Vec<FixturePlacement>) -> SceneDefinition {
        SceneDefinition {
            name: "t".into(),
            description: String::new(),
            robots,
            fixtures,
            floor_plan: None,
        }
    }

    /// Composes `scene` against `dir/scene.yaml` (written as a dummy manifest
    /// when absent) and returns the USDA.
    fn compose_usda(dir: &Path, scene: &SceneDefinition) -> String {
        let manifest = dir.join("scene.yaml");
        if !manifest.exists() {
            write_dummy_manifest(&manifest);
        }
        let out = dir.join("scene.zip");
        compose_stage(scene, &manifest, &out).unwrap();
        read_zip_entry(&out, "scene.usda")
    }

    fn bounds_of(scene: &SceneDefinition) -> Option<Aabb> {
        compute_scene_bounds(scene, &mut MeshCache::default()).unwrap()
    }

    fn assert_bounds(bounds: Option<Aabb>, min: [f64; 3], max: [f64; 3]) {
        let b = bounds.expect("non-empty bounds");
        let got = b.min.iter().chain(&b.max);
        for (g, w) in got.zip(min.iter().chain(&max)) {
            assert!((g - w).abs() < 1e-12, "{:?} != {:?} .. {:?}", b, min, max);
        }
    }

    /// The text of the first block whose head line starts with `head`
    /// (indentation included), through its closing brace.
    fn block<'a>(usda: &'a str, head: &str) -> &'a str {
        let start = usda.find(head).unwrap_or_else(|| panic!("no {head}"));
        let indent = &head[..head.len() - head.trim_start().len()];
        let close = format!("\n{indent}}}\n");
        let end = usda[start..].find(&close).unwrap() + close.len();
        &usda[start..start + end]
    }

    #[test]
    fn camera_rig_is_emitted_inside_world_in_view_order() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("r1.urdf");
        write_minimal_urdf(&urdf, "r1", 2);
        let scene = scene_of(vec![robot_at(urdf, [1.0, 2.0, 3.0], 30.0)], vec![]);
        let usda = compose_usda(dir.path(), &scene);

        // Inside /World, after the body (the last ActionGraph), before /Environment.
        let world = usda.find("def Xform \"World\" (").unwrap();
        let graphs = usda.rfind("def OmniGraph ").unwrap();
        let cameras = usda.find("    def Scope \"Cameras\"\n").unwrap();
        let environment = usda.find("def Xform \"Environment\"").unwrap();
        assert!(world < graphs && graphs < cameras && cameras < environment);
        let rig_block = block(&usda, "    def Scope \"Cameras\"");
        assert!(usda[cameras + rig_block.len()..].starts_with("\n}\n"));

        // Six cameras, in view order, with the rig's lens, clipping, lock and Kit's exposure.
        let names = ["ISO_NE", "ISO_NW", "PLAN", "ISO_SE", "ISO_SW", "ELEVATION"];
        let at: Vec<usize> = names
            .iter()
            .map(|n| rig_block.find(&format!("def Camera \"{n}\" (\n")).unwrap())
            .collect();
        assert!(at.windows(2).all(|p| p[0] < p[1]), "{at:?}");
        assert_eq!(rig_block.matches("def Camera ").count(), 6);
        assert_eq!(usda.matches("def Camera ").count(), 6);

        let bounds = bounds_of(&scene).unwrap();
        let rig = CameraRig::framing(&bounds);
        for (name, camera) in names.iter().zip(&rig.cameras) {
            assert_eq!(*name, camera.name);
            let [qw, qx, qy, qz] = camera.orient_wxyz;
            let [tx, ty, tz] = camera.translate;
            let expected = format!(
                "        def Camera \"{name}\" (\n\
                 \x20           prepend apiSchemas = [\"OmniRtxCameraAutoExposureAPI_1\", \"OmniRtxCameraExposureAPI_1\"]\n\
                 \x20       )\n\
                 \x20       {{\n\
                 \x20           float2 clippingRange = ({}, 10000)\n\
                 \x20           float exposure:fStop = 5\n\
                 \x20           float exposure:responsivity = 1.1026709\n\
                 \x20           float exposure:time = 0.02\n\
                 \x20           float focalLength = 32.992786\n\
                 \x20           float horizontalAperture = 20.955\n\
                 \x20           float verticalAperture = 11.787187\n\
                 \x20           custom bool omni:kit:cameraLock = 1\n\
                 \x20           quatd xformOp:orient = ({}, {}, {}, {})\n\
                 \x20           double3 xformOp:translate = ({}, {}, {})\n\
                 \x20           uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\"]\n\
                 \x20       }}\n",
                f(rig.clipping_range[0]),
                f_exact(qw),
                f_exact(qx),
                f_exact(qy),
                f_exact(qz),
                f_exact(tx),
                f_exact(ty),
                f_exact(tz),
            );
            assert!(
                rig_block.contains(&expected),
                "{expected}\nnot in\n{rig_block}"
            );
        }
        // Kit's exposure, scale time / fStop² · responsivity = 1 / 1133.6 at ISO 100.
        let scale = camera_rig::EXPOSURE_TIME_S / camera_rig::EXPOSURE_F_STOP.powi(2)
            * camera_rig::EXPOSURE_RESPONSIVITY;
        assert!((1.0 / scale - 1133.6).abs() < 0.05, "{}", 1.0 / scale);
        assert_eq!(usda.matches("float exposure:fStop = 5\n").count(), 6);

        // Kit's opening view: still /OmniverseKit_Persp, seeded to frame the cell.
        assert!(usda.contains("string boundCamera = \"/OmniverseKit_Persp\""));
        assert!(usda.contains("rel camera = </OmniverseKit_Persp>"));
        let seed = camera_rig::perspective_seed(&bounds);
        let [px, py, pz] = seed.position;
        let [cx, cy, cz] = seed.target;
        let perspective = format!(
            "            dictionary Perspective = {{\n\
             \x20               double3 position = ({}, {}, {})\n\
             \x20               double3 target = ({}, {}, {})\n",
            f_exact(px),
            f_exact(py),
            f_exact(pz),
            f_exact(cx),
            f_exact(cy),
            f_exact(cz),
        );
        assert!(usda.contains(&perspective), "{perspective}");
        assert!(!usda.contains("double3 position = (5, 5, 5)"));
    }

    #[test]
    fn nothing_to_frame_emits_no_rig_and_keeps_the_default_view() {
        let dir = tempfile::tempdir().unwrap();
        // A fixture without any mesh is still nothing to frame.
        let urdf = dir.path().join("fix.urdf");
        write_fixed_only_urdf(&urdf, "fix");
        for scene in [
            scene_of(vec![], vec![]),
            scene_of(vec![], vec![fixture_at(urdf, [10.0, 20.0, 0.0], 0.0)]),
        ] {
            assert_eq!(bounds_of(&scene), None);
            let usda = compose_usda(dir.path(), &scene);
            assert!(!usda.contains("Cameras") && !usda.contains("def Camera"));
            assert!(usda.contains(
                "            dictionary Perspective = {\n\
                 \x20               double3 position = (5, 5, 5)\n\
                 \x20               double3 target = (-3.978038609631085e-8, -3.978038609631085e-8, 7.956076952808644e-8)\n\
                 \x20           }\n"
            ));
        }
    }

    #[test]
    fn scene_bounds_cover_a_yawed_robot_on_its_pedestal() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("r1.urdf");
        write_minimal_urdf(&urdf, "r1", 1);
        let mut robot = robot_at(urdf, [1.0, 2.0, 3.0], 90.0);
        robot.pedestal = Some(crate::scene::Pedestal {
            height_mm: 500.0,
            diameter_mm: 800.0,
        });
        // The hull (0,0,0) (1,0,0) (0,1,0), yawed 90° onto (0,0) (0,1) (-1,0),
        // stands on the 0.5 m pedestal at (1, 2, 3.5); the pedestal spans
        // (1 ± 0.4, 2 ± 0.4) from the robot's placement height, 3 m, to 3.5 m.
        assert_bounds(
            bounds_of(&scene_of(vec![robot], vec![])),
            [0.0, 1.6, 3.0],
            [1.4, 3.0, 3.5],
        );
    }

    #[test]
    fn scene_bounds_follow_the_home_pose() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("r2.urdf");
        write_minimal_urdf(&urdf, "r2", 2);
        // joint_0 at 0° (the KUKA default's first joint): both triangles coincide.
        let mut robot = robot_at(urdf, [0.0; 3], 0.0);
        let scene = scene_of(vec![robot.clone()], vec![]);
        assert_bounds(bounds_of(&scene), [0.0; 3], [1.0, 1.0, 0.0]);
        // joint_0 at 90°: link_1's triangle turns onto (0,0) (0,1) (-1,0).
        robot.home_pose_deg = Some(vec![90.0]);
        let scene = scene_of(vec![robot], vec![]);
        assert_bounds(bounds_of(&scene), [-1.0, 0.0, 0.0], [1.0, 1.0, 0.0]);
    }

    #[test]
    fn scene_bounds_take_hulls_and_visuals_of_links_without_hulls() {
        let dir = tempfile::tempdir().unwrap();
        write_triangle_obj(&dir.path().join("meshes/big.obj"), 10.0);
        write_triangle_obj(&dir.path().join("meshes/tri.obj"), 1.0);
        let urdf = dir.path().join("vc.urdf");
        let s = r#"<?xml version="1.0"?>
<robot name="vc">
  <link name="arm">
    <visual><geometry><mesh filename="./meshes/big.obj"/></geometry></visual>
    <collision><geometry><mesh filename="./meshes/tri.obj"/></geometry></collision>
  </link>
  <link name="tool">
    <visual>
      <origin xyz="0 0 5" rpy="0 0 0"/>
      <geometry><mesh filename="./meshes/tri.obj"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 5" rpy="0 0 0"/>
      <geometry><box size="1 2 2"/></geometry>
    </collision>
  </link>
  <link name="flange">
    <visual>
      <origin xyz="0 0 -1" rpy="0 0 0"/>
      <geometry><mesh filename="./meshes/tri.obj" scale="2 1 1"/></geometry>
    </visual>
  </link>
  <joint name="j_tool" type="fixed">
    <parent link="arm"/>
    <child link="tool"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="j_flange" type="fixed">
    <parent link="arm"/>
    <child link="flange"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"#;
        std::fs::write(&urdf, s).unwrap();
        // arm: its hull (1 m), not its 10 m visual; tool: its box collider
        // (a Cube, FR-112), 1 × 2 × 2 m centred 5 m up, not its visual; flange
        // (no collision): its visual, scaled 2× along X, 1 m down.
        let scene = scene_of(vec![robot_at(urdf, [0.0; 3], 0.0)], vec![]);
        assert_bounds(bounds_of(&scene), [-0.5, -1.0, -1.0], [2.0, 1.0, 6.0]);
    }

    #[test]
    fn scene_bounds_cover_fixture_visuals_and_collisions() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("fx.urdf");
        write_meshed_fixture_urdf(&urdf);
        // Yawed 90° at (10, 20, 0): base's visual 1 m up spans (9..10, 20..21),
        // plate's hull 2 m along the fixture's X lands at (9..10, 22..23, 0).
        let scene = scene_of(vec![], vec![fixture_at(urdf, [10.0, 20.0, 0.0], 90.0)]);
        assert_bounds(bounds_of(&scene), [9.0, 20.0, 0.0], [10.0, 23.0, 1.0]);
    }

    #[test]
    fn placement_transform_matches_abc_deg_to_quat() {
        for abc in [
            [0.0, 0.0, 0.0],
            [90.0, 0.0, 0.0],
            [323.5, 0.0, 0.0],
            [30.0, -20.0, 45.0],
        ] {
            let m = placement_transform([1.0, -2.0, 3.0], abc);
            assert_eq!(math3::translation_of(&m), [1.0, -2.0, 3.0]);
            let (q, r) = (math3::quat_of(&m), abc_deg_to_quat(abc));
            let same: f64 = q.iter().zip(r).map(|(a, b)| a * b).sum();
            assert!((same.abs() - 1.0).abs() < 1e-12, "{abc:?}: {q:?} vs {r:?}");
        }
    }

    #[test]
    fn physics_scene_is_schema_conformant() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("r1.urdf");
        write_minimal_urdf(&urdf, "r1", 2);
        let usda = compose_usda(
            dir.path(),
            &scene_of(vec![robot_at(urdf, [0.0; 3], 0.0)], vec![]),
        );
        let expected = "    def PhysicsScene \"PhysicsScene\" (\n\
            \x20       prepend apiSchemas = [\"PhysxSceneAPI\"]\n\
            \x20   )\n\
            \x20   {\n\
            \x20       vector3f physics:gravityDirection = (0, 0, -1)\n\
            \x20       float physics:gravityMagnitude = 9.81\n\
            \x20       bool physxScene:enableGPUDynamics = 1\n\
            \x20       uint physxScene:gpuFoundLostAggregatePairsCapacity = 8192\n\
            \x20       uint physxScene:gpuFoundLostPairsCapacity = 262144\n\
            \x20       uint physxScene:gpuMaxRigidContactCount = 1048576\n\
            \x20       uint physxScene:gpuMaxRigidPatchCount = 81920\n\
            \x20       uint physxScene:gpuTotalAggregatePairsCapacity = 4096\n\
            \x20       uniform uint physxScene:maxPositionIterationCount = 64\n\
            \x20       uniform uint physxScene:maxVelocityIterationCount = 4\n\
            \x20       uint physxScene:timeStepsPerSecond = 120\n\
            \x20   }\n";
        assert_eq!(
            block(&usda, "    def PhysicsScene \"PhysicsScene\" ("),
            expected
        );
        // Still the first prim under /World.
        assert!(usda.contains(
            "def Xform \"World\" (\n    kind = \"assembly\"\n)\n{\n    def PhysicsScene"
        ));
        for gone in [
            "PhysxSchemaPhysxSceneAPI",
            "basic:gravity",
            "enableGpuSdfApproximation",
            "physxScene:positionIterations",
            "physxScene:velocityIterations",
            " int physxScene:",
            " float physxScene:",
            "NewtonSceneAPI",
        ] {
            assert!(!usda.contains(gone), "{gone}");
        }
        // The scene-wide cap doesn't undercut the articulations' own counts.
        assert!(usda.contains("uniform int physxArticulation:solverPositionIterationCount = 64"));
    }

    // NFR-108
    #[test]
    fn environment_replicates_kits_default_light_rig_on_a_matte_ground() {
        let dir = tempfile::tempdir().unwrap();
        let usda = compose_usda(dir.path(), &scene_of(vec![], vec![]));
        // Nothing to lie under: the smallest ground, at the origin.
        let expected = "def Xform \"Environment\"\n\
            {\n\
            \x20   quatd xformOp:orient = (1, 0, 0, 0)\n\
            \x20   double3 xformOp:scale = (1, 1, 1)\n\
            \x20   double3 xformOp:translate = (0, 0, 0)\n\
            \x20   uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\", \"xformOp:scale\"]\n\
            \n\
            \x20   def DistantLight \"defaultLight\" (\n\
            \x20       prepend apiSchemas = [\"ShapingAPI\"]\n\
            \x20   )\n\
            \x20   {\n\
            \x20       float inputs:angle = 0.53\n\
            \x20       float inputs:colorTemperature = 7250\n\
            \x20       bool inputs:enableColorTemperature = 1\n\
            \x20       float inputs:exposure = 10\n\
            \x20       float inputs:intensity = 1\n\
            \x20       float inputs:shaping:cone:angle = 180\n\
            \x20       token visibility = \"inherited\"\n\
            \x20       quatd xformOp:orient = (0.33944435018566016, 0.17670354420260315, 0.42660009293331813, 0.8194911538891423)\n\
            \x20       double3 xformOp:scale = (1, 1, 1)\n\
            \x20       double3 xformOp:translate = (0, 0, 0)\n\
            \x20       uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\", \"xformOp:scale\"]\n\
            \x20   }\n\
            \n\
            \x20   def DomeLight \"domeLight\" (\n\
            \x20       prepend apiSchemas = [\"ShapingAPI\"]\n\
            \x20   )\n\
            \x20   {\n\
            \x20       bool visibleInPrimaryRay = 0\n\
            \x20       float inputs:colorTemperature = 6150\n\
            \x20       bool inputs:enableColorTemperature = 1\n\
            \x20       float inputs:exposure = 9\n\
            \x20       float inputs:intensity = 1\n\
            \x20       float inputs:shaping:cone:angle = 180\n\
            \x20       token inputs:texture:format = \"latlong\"\n\
            \x20       token visibility = \"inherited\"\n\
            \x20       quatd xformOp:orient = (0.8558159940603218, 0, 0, 0.5172803730188719)\n\
            \x20       double3 xformOp:scale = (1, 1, 1)\n\
            \x20       double3 xformOp:translate = (0, 0, 0)\n\
            \x20       uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\", \"xformOp:scale\"]\n\
            \x20   }\n\
            \n\
            \x20   def Scope \"Looks\"\n\
            \x20   {\n\
            \x20       def Material \"M_ground\"\n\
            \x20       {\n\
            \x20           token outputs:surface.connect = </Environment/Looks/M_ground/Shader.outputs:surface>\n\
            \n\
            \x20           def Shader \"Shader\"\n\
            \x20           {\n\
            \x20               uniform token info:id = \"UsdPreviewSurface\"\n\
            \x20               color3f inputs:diffuseColor = (0.18, 0.18, 0.18)\n\
            \x20               float inputs:metallic = 0\n\
            \x20               float inputs:roughness = 0.85\n\
            \x20               token outputs:surface\n\
            \x20           }\n\
            \x20       }\n\
            \x20   }\n\
            \n\
            \x20   def Mesh \"Ground\" (\n\
            \x20       prepend apiSchemas = [\"MaterialBindingAPI\"]\n\
            \x20   )\n\
            \x20   {\n\
            \x20       float3[] extent = [(-50, -50, 0), (50, 50, 0)]\n\
            \x20       int[] faceVertexCounts = [4]\n\
            \x20       int[] faceVertexIndices = [0, 1, 2, 3]\n\
            \x20       rel material:binding = </Environment/Looks/M_ground>\n\
            \x20       normal3f[] normals = [(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)] (\n\
            \x20           interpolation = \"vertex\"\n\
            \x20       )\n\
            \x20       point3f[] points = [(-50, -50, 0), (50, -50, 0), (50, 50, 0), (-50, 50, 0)]\n\
            \x20       uniform token subdivisionScheme = \"none\"\n\
            \x20       double3 xformOp:translate = (0, 0, 0)\n\
            \x20       uniform token[] xformOpOrder = [\"xformOp:translate\"]\n\
            \x20   }\n\
            \n\
            \x20   def Plane \"GroundCollider\" (\n\
            \x20       prepend apiSchemas = [\"PhysicsCollisionAPI\"]\n\
            \x20   )\n\
            \x20   {\n\
            \x20       uniform token axis = \"Z\"\n\
            \x20       uniform token purpose = \"guide\"\n\
            \x20   }\n\
            }\n";
        assert_eq!(block(&usda, "def Xform \"Environment\""), expected);
        // Inline: no FlatGrid, its sphere light or any other asset reference; nothing to fetch.
        for gone in [
            "FlatGrid",
            "SphereLight",
            "default_environment",
            "references",
            "@",
            "http",
            "omni:rtx:usdluxVersion",
        ] {
            assert!(!usda.contains(gone), "{gone}");
        }
        // The dome is the render product's background (unseen: a dark backdrop).
        assert!(usda.contains("token omni:rtx:background:source:type = \"domeLight\"\n"));
        // The render product keeps its resolution (the lenses' aspect ratio).
        assert!(usda.contains("                uniform int2 resolution = (1280, 720)\n"));
    }

    // NFR-108
    #[test]
    fn default_rig_orientations_follow_kits_z_up_import() {
        // USD's rotateXYZ (a, b, c): Rz(c) · Ry(b) · Rx(a), acting on column vectors.
        let rotate_xyz = |[a, b, c]: [f64; 3]| {
            let r = |axis: [f64; 3], deg: f64| math3::rotation_about_axis(axis, deg.to_radians());
            math3::multiply(
                &r([0.0, 0.0, 1.0], c),
                &math3::multiply(&r([0.0, 1.0, 0.0], b), &r([1.0, 0.0, 0.0], a)),
            )
        };
        // Kit's Y-up rig → Z-up stage turn, (x, y, z) → (z, x, y) (`_add_rig_reference`'s
        // row-vector `Gf.Matrix4d(0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1)`).
        let kit_z_up = [
            [0.0, 0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let lights_group = rotate_xyz([0.0, -90.0, -90.0]);
        for (own, orient) in [
            ([55.0, 0.0, 135.0], DEFAULT_RIG_SUN_ORIENT),
            ([0.0, 0.0, 62.30000092834234], DEFAULT_RIG_DOME_ORIENT),
        ] {
            // The rig's group undoes Kit's turn: each light keeps its own rotation.
            let group_in_stage = math3::multiply(&kit_z_up, &lights_group);
            for (i, row) in group_in_stage.iter().enumerate() {
                for (j, v) in row.iter().enumerate() {
                    assert!((v - math3::IDENTITY[i][j]).abs() < 1e-15, "{group_in_stage:?}");
                }
            }
            let world = math3::multiply(&group_in_stage, &rotate_xyz(own));
            let [x, y, z, w] = math3::quat_of(&world);
            for (got, want) in [w, x, y, z].iter().zip(orient) {
                assert!((got - want).abs() < 1e-14, "{own:?}: {:?} != {orient:?}", [w, x, y, z]);
            }
            let [w, x, y, z] = orient;
            assert!((x * x + y * y + z * z + w * w - 1.0).abs() < 1e-15);
        }
        // The sun: 35° up in the north-east (azimuth 45°), shining along its −Z.
        let along = math3::rotate(&rotate_xyz([55.0, 0.0, 135.0]), [0.0, 0.0, -1.0]);
        assert!(((-along[2]).asin().to_degrees() - 35.0).abs() < 1e-12, "{along:?}");
        assert!(((-along[1]).atan2(-along[0]).to_degrees() - 45.0).abs() < 1e-12, "{along:?}");
    }

    // NFR-108
    #[test]
    fn ground_lies_under_the_scene_and_its_floor_plan() {
        let bounds = Aabb {
            min: [-1.0, 2.0, 0.0],
            max: [3.0, 4.0, 5.0],
        };
        // 10× the box's reach (2 m in x from the centre (1, 3)), at least 100 m across.
        assert_eq!(
            GroundSquare::under(Some(&bounds), None),
            GroundSquare { center: [1.0, 3.0], half_side: 50.0 }
        );
        let wide = Aabb {
            min: [-10.0, -4.0, 0.0],
            max: [10.0, 4.0, 1.0],
        };
        assert_eq!(GroundSquare::under(Some(&wide), None).half_side, 100.0);
        // A floor plan reaching 12 m beyond the centre (in y) widens it; non-finite points don't.
        let plan = FloorPlan {
            segments: vec![[[0.0, 15.0, 0.0], [1.0, 3.0, 0.0]]],
            footprint_segments: vec![[[f64::NAN, 0.0, 0.0], [f64::INFINITY, 1.0, 0.0]]],
            ..FloorPlan::default()
        };
        assert_eq!(
            GroundSquare::under(Some(&bounds), Some(&plan)),
            GroundSquare { center: [1.0, 3.0], half_side: 120.0 }
        );
        // Nothing to lie under: the origin.
        assert_eq!(
            GroundSquare::under(None, None),
            GroundSquare { center: [0.0, 0.0], half_side: 50.0 }
        );

        // Composed: under the robot's bounds.
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("r1.urdf");
        write_minimal_urdf(&urdf, "r1", 1);
        let scene = scene_of(vec![robot_at(urdf, [30.0, -20.0, 0.0], 0.0)], vec![]);
        let usda = compose_usda(dir.path(), &scene);
        let ground = block(&usda, "    def Mesh \"Ground\" (");
        // The triangle (0,0) (1,0) (0,1) at (30, −20): centre (30.5, −19.5).
        assert!(ground.contains("        double3 xformOp:translate = (30.5, -19.5, 0)\n"), "{ground}");
        assert!(ground.contains("        float3[] extent = [(-50, -50, 0), (50, 50, 0)]\n"));
    }

    // NFR-108
    #[test]
    fn render_settings_leave_the_look_to_isaac_sim() {
        let dir = tempfile::tempdir().unwrap();
        let usda = compose_usda(dir.path(), &scene_of(vec![], vec![]));
        // An empty renderSettings dictionary, as Kit 110 saves it: the app's own settings.
        assert!(usda.contains(
            "        dictionary omni_layer = {\n\
             \x20           string authoring_layer = \"./scene.usda\"\n\
             \x20           dictionary locked = {\n\
             \x20           }\n\
             \x20           dictionary muteness = {\n\
             \x20           }\n\
             \x20       }\n\
             \x20       dictionary renderSettings = {\n\
             \x20       }\n\
             \x20       string \"robo_fitting_room:generator\""
        ));
        for gone in ["rtx:post:aa", "rtx:indirectLighting", "rtx:realtime:denoiser", "rendermode", "PathTracing"] {
            assert!(!usda.contains(gone), "{gone}");
        }
        let render = block(&usda, "def \"Render\" (");
        // Isaac Sim 6.x's types.
        assert!(render.contains("                uint omni:rtx:pt:maxSamplesPerLaunch = 2073600\n"));
        assert!(render.contains("                uint omni:rtx:pt:mgpu:maxPixelsPerRegionExponent = 12\n"));
        assert!(!render.contains(" int omni:rtx:pt:"));
        assert!(render.ends_with(
            "    def \"Vars\" (\n\
             \x20       hide_in_stage_window = true\n\
             \x20       no_delete = true\n\
             \x20   )\n\
             \x20   {\n\
             \x20       def RenderVar \"LdrColor\" (\n\
             \x20           hide_in_stage_window = true\n\
             \x20           no_delete = true\n\
             \x20       )\n\
             \x20       {\n\
             \x20           uniform string sourceName = \"LdrColor\"\n\
             \x20           uniform token sourceType = \"raw\"\n\
             \x20       }\n\
             \x20   }\n\
             }\n"
        ));
        assert!(usda.ends_with(render) || usda.ends_with(&format!("{render}\n")));
    }

    /// A one-link robot whose visual and hull meshes use the MTL material `Paint` with `kd`.
    fn write_painted_urdf(path: &Path, kd: &str) {
        let dir = path.parent().unwrap();
        std::fs::create_dir_all(dir.join("meshes")).unwrap();
        std::fs::write(
            dir.join("meshes/paint.obj"),
            "mtllib paint.mtl\nusemtl Paint\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
        )
        .unwrap();
        std::fs::write(dir.join("meshes/paint.mtl"), format!("newmtl Paint\nKd {kd}\nNs 10\n")).unwrap();
        std::fs::write(
            path,
            "<?xml version=\"1.0\"?>\n<robot name=\"painted\">\n  <link name=\"link_0\">\n    <visual><geometry><mesh filename=\"./meshes/paint.obj\"/></geometry></visual>\n    <collision><geometry><mesh filename=\"./meshes/paint.obj\"/></geometry></collision>\n  </link>\n</robot>\n",
        )
        .unwrap();
    }

    // FR-111
    #[test]
    fn mtl_kd_is_linearised() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("painted.urdf");
        write_painted_urdf(&urdf, "0.5 0.5 0.5");
        let mut robot = robot_at(urdf.clone(), [0.0; 3], 0.0);
        robot.pedestal = Some(crate::scene::Pedestal {
            height_mm: 500.0,
            diameter_mm: 800.0,
        });
        let usda = compose_usda(dir.path(), &scene_of(vec![robot], vec![]));
        let looks = block(&usda, "    def Scope \"Looks\"");
        // The MTL's sRGB 0.5 is linear 0.214041; the default grey (the pedestal's) stays 0.45.
        let paint = block(looks, "        def Material \"painted__Paint\"");
        assert!(paint.contains("color3f inputs:diffuseColor = (0.214041, 0.214041, 0.214041)\n"), "{paint}");
        assert!(paint.contains("float inputs:roughness = 0.666667\n"));
        let default = block(looks, "        def Material \"factoryos_default\"");
        assert!(default.contains("color3f inputs:diffuseColor = (0.45, 0.45, 0.45)\n"));
        // The lean robot's hull wears the link's (linearised) paint.
        assert!(usda.contains("rel material:binding = </World/Looks/painted__Paint>"));

        // Per channel, both segments of the curve.
        write_painted_urdf(&urdf, "1 0.02 0.75");
        let usda = compose_usda(dir.path(), &scene_of(vec![robot_at(urdf, [0.0; 3], 0.0)], vec![]));
        assert!(usda.contains(&format!(
            "color3f inputs:diffuseColor = (1, {}, 0.522522)\n",
            f(0.02 / 12.92)
        )));
    }

    /// A one-link robot `mat` whose visual mesh names `m.mtl` and `usemtl`; `mtl` the MTL's text,
    /// none for an MTL that doesn't exist.
    fn write_material_urdf(path: &Path, mtl: Option<&str>, usemtl: &str) {
        let dir = path.parent().unwrap();
        std::fs::create_dir_all(dir.join("meshes")).unwrap();
        std::fs::write(
            dir.join("meshes/m.obj"),
            format!("mtllib m.mtl\nusemtl {usemtl}\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"),
        )
        .unwrap();
        let mtl_path = dir.join("meshes/m.mtl");
        match mtl {
            Some(text) => std::fs::write(&mtl_path, text).unwrap(),
            None => {
                let _ = std::fs::remove_file(&mtl_path);
            }
        }
        std::fs::write(
            path,
            "<?xml version=\"1.0\"?>\n<robot name=\"mat\">\n  <link name=\"link_0\">\n    <visual><geometry><mesh filename=\"./meshes/m.obj\"/></geometry></visual>\n  </link>\n</robot>\n",
        )
        .unwrap();
    }

    // FR-111
    #[test]
    fn colourless_materials_get_the_default_linear_grey() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("mat.urdf");
        let diffuse = |mtl: Option<&str>, usemtl: &str| {
            write_material_urdf(&urdf, mtl, usemtl);
            let mut robot = robot_at(urdf.clone(), [0.0; 3], 0.0);
            robot.pedestal = Some(crate::scene::Pedestal {
                height_mm: 500.0,
                diameter_mm: 800.0,
            });
            let usda = compose_usda(dir.path(), &scene_of(vec![robot], vec![]));
            let looks = block(&usda, "    def Scope \"Looks\"");
            let colour = |name: &str| {
                let material = block(looks, &format!("        def Material \"{name}\""));
                attr(material, "inputs:diffuseColor").to_owned()
            };
            (
                colour(&format!("mat__{usemtl}")),
                colour("factoryos_default"),
            )
        };
        // No Kd line, a material its MTL lacks, an MTL that doesn't exist: the default material's
        // linear 0.45, neither a lighter placeholder grey nor one decoded from sRGB.
        for (mtl, usemtl) in [
            (Some("newmtl Bare\nNs 10\n"), "Bare"),
            (Some("newmtl Other\nKd 1 0 0\n"), "Missing"),
            (None, "Gone"),
        ] {
            let (material, default) = diffuse(mtl, usemtl);
            assert_eq!(material, "(0.45, 0.45, 0.45)", "{usemtl}");
            assert_eq!(material, default, "{usemtl}");
        }
        // A Kd is decoded from sRGB.
        let (paint, _) = diffuse(Some("newmtl Paint\nKd 0.5 0.5 0.5\n"), "Paint");
        assert_eq!(paint, "(0.214041, 0.214041, 0.214041)");
    }

    /// An outward-wound box OBJ of `half` extents about the origin.
    fn write_box_obj(path: &Path, half: [f64; 3]) {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        let [hx, hy, hz] = half;
        let mut obj = String::new();
        for i in 0..8 {
            let s = |bit: usize, h: f64| if i & (1 << bit) == 0 { -h } else { h };
            obj.push_str(&format!("v {} {} {}\n", s(0, hx), s(1, hy), s(2, hz)));
        }
        for [a, b, c, d] in [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ] {
            obj.push_str(&format!("f {} {} {} {}\n", a + 1, b + 1, c + 1, d + 1));
        }
        std::fs::write(path, obj).unwrap();
    }

    /// A robot `World` → (fixed) `base` → (revolute about Z) `arm`. `arm` carries a 2 kg URDF
    /// inertial, a robot collision box 1 × 0.5 × 0.2 m (0.1 m³) at (0.3, 0, 0.1) and a tool
    /// collision box 0.4 × 0.2 × 0.6 m (0.048 m³) at (−0.5, 0, 0.2); `base` only a visual box
    /// 0.2 m wide (0.008 m³) and no inertial.
    fn write_density_robot_urdf(path: &Path) {
        let dir = path.parent().unwrap();
        write_box_obj(&dir.join("meshes/robot/arm.obj"), [0.5, 0.25, 0.1]);
        write_box_obj(&dir.join("meshes/robot/base.obj"), [0.1, 0.1, 0.1]);
        write_box_obj(&dir.join("meshes/tool/grip.obj"), [0.2, 0.1, 0.3]);
        std::fs::write(
            path,
            r#"<?xml version="1.0"?>
<robot name="dense">
  <link name="World"/>
  <link name="base">
    <visual><geometry><mesh filename="./meshes/robot/base.obj"/></geometry></visual>
  </link>
  <link name="arm">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <collision>
      <origin xyz="0.3 0 0.1" rpy="0 0 0"/>
      <geometry><mesh filename="./meshes/robot/arm.obj"/></geometry>
    </collision>
    <collision>
      <origin xyz="-0.5 0 0.2" rpy="0 0 0"/>
      <geometry><mesh filename="./meshes/tool/grip.obj"/></geometry>
    </collision>
  </link>
  <joint name="root_joint" type="fixed">
    <parent link="World"/>
    <child link="base"/>
  </joint>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="340282346638528859811704183484516925440" velocity="1"/>
  </joint>
</robot>
"#,
        )
        .unwrap();
    }

    // FR-113
    #[test]
    fn link_meshes_take_robot_collisions_else_visuals_and_the_tool_hulls() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("dense.urdf");
        write_density_robot_urdf(&urdf);
        let robot = parse_urdf(&urdf).unwrap();
        let layout = ToolLayout::of(&robot, None);
        let mut cache = MeshCache::default();
        let link = |name: &str| robot.links.iter().find(|l| l.name == name).unwrap();
        // arm: its robot collision box, placed by its origin; the tool box apart.
        let arm = link_meshes(link("arm"), &layout, dir.path(), &mut cache, "t");
        assert_eq!((arm.robot.len(), arm.tool.len()), (1, 1));
        assert_eq!(arm.robot[0].name, "./meshes/robot/arm.obj");
        assert_eq!(arm.tool[0].name, "./meshes/tool/grip.obj");
        assert!(
            arm.robot[0].points.contains(&[0.8, 0.25, 0.2]),
            "{:?}",
            arm.robot[0].points
        );
        assert!(
            arm.tool[0].points.contains(&[-0.3, 0.1, 0.5]),
            "{:?}",
            arm.tool[0].points
        );
        assert_eq!(arm.robot[0].triangle_indices.len(), 36);
        // base: no collision mesh, so its visual box.
        let base = link_meshes(link("base"), &layout, dir.path(), &mut cache, "t");
        assert_eq!((base.robot.len(), base.tool.len()), (1, 0));
        assert_eq!(base.robot[0].name, "./meshes/robot/base.obj");
        // World: nothing.
        assert_eq!(
            link_meshes(link("World"), &layout, dir.path(), &mut cache, "t"),
            drives::LinkMeshes::default()
        );
        // A link with a robot collision mesh doesn't take its visuals; one whose mesh doesn't load
        // has no geometry.
        let mut with_visual = link("arm").clone();
        with_visual.visuals = link("base").visuals.clone();
        assert_eq!(
            link_meshes(&with_visual, &layout, dir.path(), &mut cache, "t").robot,
            arm.robot
        );
        let mut missing = link("base").clone();
        missing.visuals[0].geometry = Geometry::Mesh {
            filename: "./meshes/robot/none.obj".into(),
            scale: [1.0; 3],
        };
        assert!(link_meshes(&missing, &layout, dir.path(), &mut cache, "t")
            .robot
            .is_empty());
    }

    // FR-113, FR-159
    #[test]
    fn link_masses_follow_the_urdf_or_the_link_density() {
        let dir = tempfile::tempdir().unwrap();
        let urdf = dir.path().join("dense.urdf");
        write_density_robot_urdf(&urdf);
        let mass_of = |usda: &str, link: &str| -> f64 {
            attr(
                block(usda, &format!("        def Xform \"{link}\" (")),
                "physics:mass",
            )
            .parse()
            .unwrap()
        };
        // At the default density: arm's 2 kg over 0.1 m³ is a placeholder, so its robot box and
        // its tool box weigh 1200 kg/m³ · 0.148 m³; base (no inertial) its visual box; World
        // (nothing) the fallback.
        let robot = robot_at(urdf.clone(), [0.0; 3], 0.0);
        let usda = compose_usda(dir.path(), &scene_of(vec![robot.clone()], vec![]));
        let density = drives::DEFAULT_LINK_DENSITY_KG_M3;
        assert!((mass_of(&usda, "arm") - density * 0.148).abs() < 1e-3);
        assert!((mass_of(&usda, "base") - density * 0.008).abs() < 1e-3);
        assert_eq!(mass_of(&usda, "world_link"), 10.0);
        // The arm's centre of mass: the boxes' volume-weighted centroid.
        let com = attr_tuple(
            block(&usda, "        def Xform \"arm\" ("),
            "physics:centerOfMass",
        );
        let want = [
            (0.1 * 0.3 - 0.048 * 0.5) / 0.148,
            0.0,
            (0.1 * 0.1 + 0.048 * 0.2) / 0.148,
        ];
        for (g, w) in com.iter().zip(want) {
            assert!((g - w).abs() < 2e-6, "{com:?} != {want:?}");
        }
        // Every rigid body's diagonal inertia comes with its principal axes, the identity for
        // the axis-aligned boxes and the fallback.
        assert_eq!(usda.matches("physics:diagonalInertia").count(), 3);
        assert_eq!(
            usda.matches("quatf physics:principalAxes = (1, 0, 0, 0)")
                .count(),
            2
        );
        assert_eq!(usda.matches("quatf physics:principalAxes").count(), 3);
        // The manifest's density: twice the mass.
        let mut dense = robot.clone();
        dense.link_density_kg_m3 = Some(2400.0);
        let usda = compose_usda(dir.path(), &scene_of(vec![dense], vec![]));
        assert!((mass_of(&usda, "arm") - 2400.0 * 0.148).abs() < 1e-3);
        assert!((mass_of(&usda, "base") - 2400.0 * 0.008).abs() < 1e-3);
        // A payload replaces the tool box's density: the robot box's 120 kg and the payload's 50.
        let mut carrying = robot;
        carrying.payload = Some(crate::scene::Payload {
            mass_kg: 50.0,
            center_of_mass_m: None,
            inertia_kgm2: None,
            link: None,
        });
        let usda = compose_usda(dir.path(), &scene_of(vec![carrying], vec![]));
        assert!((mass_of(&usda, "arm") - (density * 0.1 + 50.0)).abs() < 1e-3);
    }

    #[test]
    fn kinds_mark_the_model_hierarchy() {
        let dir = tempfile::tempdir().unwrap();
        let robot_urdf = dir.path().join("r1.urdf");
        write_minimal_urdf(&robot_urdf, "r1", 1);
        let fixture_urdf = dir.path().join("fx/fx.urdf");
        write_meshed_fixture_urdf(&fixture_urdf);
        let scene = scene_of(
            vec![robot_at(robot_urdf, [0.0; 3], 0.0)],
            vec![fixture_at(fixture_urdf, [5.0, 0.0, 0.0], 0.0)],
        );
        let usda = compose_usda(dir.path(), &scene);
        for model in [
            "def Xform \"World\" (\n    kind = \"assembly\"\n)\n{\n",
            "    def Scope \"robots\" (\n        kind = \"group\"\n    )\n    {\n",
            "    def Scope \"fixtures\" (\n        kind = \"group\"\n    )\n    {\n",
            "    def Xform \"R\" (\n        prepend apiSchemas = [\"PhysicsArticulationRootAPI\", \"PhysxArticulationAPI\"]\n        kind = \"component\"\n    )\n    {\n",
            "        def Xform \"FX1\" (\n            kind = \"component\"\n        )\n        {\n",
        ] {
            assert!(usda.contains(model), "{model}");
        }
        assert_eq!(usda.matches("kind = \"assembly\"").count(), 1);
        assert_eq!(usda.matches("kind = \"group\"").count(), 2);
        assert_eq!(usda.matches("kind = \"component\"").count(), 2);
    }

    #[test]
    fn header_carries_doc_provenance_and_frame_rate() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(
            dir.path().join("scene.yaml"),
            "name: \"x\"\nscene_origin: { x: 12.75, y: -3.5, z: 0 }\n",
        )
        .unwrap();
        let mut scene = scene_of(vec![], vec![]);
        scene.name = "Cell \"A\"".into();
        scene.description = "line 1\nline\t2\\ ".into();
        let usda = compose_usda(dir.path(), &scene);
        let header = &usda[..usda.find("\n)\n").unwrap() + 3];

        assert!(header.contains(&format!(
            "        string \"robo_fitting_room:generator\" = \"robo-fitting-room {}\"\n",
            env!("CARGO_PKG_VERSION")
        )));
        assert!(header.contains(
            "        double3 \"robo_fitting_room:scene_origin_m\" = (12.75, -3.5, 0)\n    }\n"
        ));
        assert!(header.contains(
            "    endTimeCode = 1000000\n    framesPerSecond = 60\n    metersPerUnit = 1\n"
        ));
        assert!(header.contains("    timeCodesPerSecond = 60\n"));
        // `<name>: <description>`, escaped onto one line.
        assert!(header.ends_with("    doc = \"Cell \\\"A\\\": line 1\\nline\\t2\\\\\"\n)\n"));

        // No description: the bare name; no scene_origin: (0, 0, 0).
        write_dummy_manifest(&dir.path().join("scene.yaml"));
        scene.description = " \n".into();
        let usda = compose_usda(dir.path(), &scene);
        assert!(usda.contains("    doc = \"Cell \\\"A\\\"\"\n)\n"));
        assert!(usda.contains("double3 \"robo_fitting_room:scene_origin_m\" = (0, 0, 0)\n"));
    }

    #[test]
    fn f_exact_keeps_every_digit_of_a_double() {
        assert_eq!(f_exact(0.1 + 0.2), "0.30000000000000004");
        assert_eq!(f_exact(10.0 + 2.0 / 3.0), "10.666666666666666");
        assert_eq!(f_exact(-0.01745240643728351), "-0.01745240643728351");
        assert_eq!(f_exact(12.75), "12.75");
        assert_eq!(f_exact(10000.0), "10000");
        assert_eq!(f_exact(0.0), "0");
        assert_eq!(f_exact(2f64.powi(-60)), "8.673617379884035e-19");
        assert_eq!(f_exact(3.4028234663852886e38), "3.4028234663852886e38");
        assert_eq!(f_exact(f64::NAN), "0");
        // `f` rounds the same value to 6 decimals.
        assert_eq!(f(10.0 + 2.0 / 3.0), "10.666667");
    }

    #[test]
    fn escape_str_escapes_quotes_backslashes_and_line_breaks() {
        assert_eq!(escape_str("a\\b\"c\nd\re\tf"), "a\\\\b\\\"c\\nd\\re\\tf");
        assert_eq!(escape_str("plain — ünïcode"), "plain — ünïcode");
    }

    #[test]
    fn manifest_scene_origin_reads_the_raw_manifest() {
        assert_eq!(
            manifest_scene_origin("name: x\nscene_origin: { x: 1.5, y: -2, z: 0.25 }\n"),
            [1.5, -2.0, 0.25]
        );
        assert_eq!(manifest_scene_origin("name: x\n"), [0.0; 3]);
        assert_eq!(manifest_scene_origin(": not yaml : ["), [0.0; 3]);
    }

    #[test]
    fn compose_is_deterministic_with_the_scene_globals() {
        let dir = tempfile::tempdir().unwrap();
        let robot_urdf = dir.path().join("r2.urdf");
        write_minimal_urdf(&robot_urdf, "r2", 2);
        let fixture_urdf = dir.path().join("fx/fx.urdf");
        write_meshed_fixture_urdf(&fixture_urdf);
        let mut robot = robot_at(robot_urdf, [1.0, 2.0, 0.0], 45.0);
        robot.pedestal = Some(crate::scene::Pedestal {
            height_mm: 600.0,
            diameter_mm: 800.0,
        });
        let mut scene = scene_of(
            vec![robot],
            vec![fixture_at(fixture_urdf, [5.0, 0.0, 0.0], 90.0)],
        );
        scene.floor_plan = Some(small_floor_plan());
        let first = compose_usda(dir.path(), &scene);
        let second = compose_usda(dir.path(), &scene);
        for part in [
            "def Scope \"Cameras\"",
            "def Xform \"floor_plan\"",
            "def Mesh \"Ground\"",
            "def Plane \"GroundCollider\"",
            "exposure:fStop",
        ] {
            assert!(first.contains(part), "{part}");
        }
        assert_eq!(first, second);
    }
}
