//! Meshes as self-contained binary glTF (GLB) for the viewers, plus a decimated LOD: what
//! [`super::assets::AssetIndex::mesh_asset`] puts into [`MeshAsset::glb`] and [`MeshAsset::lod`].
//!
//! - **OBJ + MTL**: positions, normals (the OBJ's `vn`, else area-weighted ones), triangles, and
//!   one glTF material per diffuse colour, the same colours `factoryos_urdf_materials::obj_to_mesh3d`
//!   gives (`Kd` read as sRGB, `d` as alpha), stored as glTF's linear `baseColorFactor`. Faces
//!   without a material colour keep the OBJ's own vertex colours (`COLOR_0`, linear) or are white;
//!   an OBJ with no colour at all gets no material (the consumer's default, as with Mesh3D).
//!   Textures (`map_Kd`) are not baked: `obj_to_mesh3d` ignores them too.
//! - **STL**: positions, normals smoothed within [`CREASE_DEGREES`] (CAD edges stay sharp), no
//!   material (a URDF `<material>` still applies).
//! - GLB and glTF are served as they are, DAE is not converted.
//!
//! Vertices keep the source's coordinates: no Y-up conversion, a URDF mesh's frame is its link's.
//! Everything is double-sided and non-metallic, as the viewers draw Mesh3Ds.
//!
//! [`MeshAsset::glb`]: factoryos_protocol::scene::MeshAsset::glb
//! [`MeshAsset::lod`]: factoryos_protocol::scene::MeshAsset::lod

use std::collections::HashMap;
use std::io::Cursor;
use std::path::Path;

use anyhow::{Context as _, bail};
use serde_json::json;

/// Part of the cache key: bump it when the output of a conversion changes.
pub const VERSION: u32 = 1;
/// The LOD keeps about this share of the triangles…
pub const LOD_RATIO: f32 = 0.15;
/// …but at least this many.
pub const LOD_MIN_TRIANGLES: usize = 500;
/// No LOD when it would keep more than this share (small meshes are their own LOD).
const LOD_MAX_SHARE: f32 = 0.6;
/// Faces meeting at a smaller angle share their vertex normals (STL, LOD).
pub const CREASE_DEGREES: f32 = 40.0;

/// A mesh format converted to GLB.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    Obj,
    Stl,
}

impl Source {
    /// By extension; `None` for formats served as they are (GLB, glTF) or not converted (DAE).
    pub fn of(path: &Path) -> Option<Self> {
        match path.extension()?.to_str()?.to_ascii_lowercase().as_str() {
            "obj" => Some(Self::Obj),
            "stl" => Some(Self::Stl),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Obj => "obj",
            Self::Stl => "stl",
        }
    }
}

/// A conversion's output.
#[derive(Debug)]
pub struct Converted {
    pub glb: Vec<u8>,
    /// `None` for meshes too small to decimate.
    pub lod: Option<Vec<u8>>,
    pub triangles: usize,
    pub lod_triangles: Option<usize>,
}

/// Converts the bytes of a mesh. `load_sidecar` returns an OBJ's `mtllib` by its name as written in
/// the OBJ (`None`: no colours from it).
pub fn convert(source: Source, bytes: &[u8], load_sidecar: &dyn Fn(&str) -> Option<Vec<u8>>) -> anyhow::Result<Converted> {
    let mesh = match source {
        Source::Obj => obj_mesh(bytes, load_sidecar)?,
        Source::Stl => stl_mesh(bytes)?,
    };
    let triangles = mesh.triangles();
    if triangles == 0 {
        bail!("no triangles");
    }
    let glb = write_glb(&mesh);
    let lod = lod(&mesh);
    let lod_triangles = lod.as_ref().map(Mesh::triangles);
    Ok(Converted { glb, lod: lod.as_ref().map(write_glb), triangles, lod_triangles })
}

/// Triangles with one material.
#[derive(Clone, Debug, Default)]
struct Primitive {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    /// Linear RGBA per vertex.
    colors: Option<Vec<[f32; 4]>>,
    indices: Vec<u32>,
    /// Linear RGBA; `None`: no material.
    base_color: Option<[f32; 4]>,
}

#[derive(Clone, Debug, Default)]
struct Mesh {
    primitives: Vec<Primitive>,
}

impl Mesh {
    fn triangles(&self) -> usize {
        self.primitives.iter().map(|p| p.indices.len() / 3).sum()
    }
}

/// How the faces of an OBJ model are coloured, as in `obj_to_mesh3d`.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
enum Coloring {
    /// sRGB(A) bytes of a material's `Kd` and `d`.
    Material([u8; 4]),
    VertexColors,
    White,
    /// No face of the OBJ has a colour.
    None,
}

fn obj_mesh(obj: &[u8], load_sidecar: &dyn Fn(&str) -> Option<Vec<u8>>) -> anyhow::Result<Mesh> {
    let options = tobj::LoadOptions { single_index: true, triangulate: true, ..Default::default() };
    let (models, materials) = tobj::load_obj_buf(&mut Cursor::new(obj), &options, |mtl_path| {
        let mtl = load_sidecar(&mtl_path.to_string_lossy()).ok_or(tobj::LoadError::OpenFileFailed)?;
        tobj::load_mtl_buf(&mut Cursor::new(mtl))
    })
    .context("parsing OBJ")?;
    let materials = materials.unwrap_or_else(|err| {
        tracing::debug!("OBJ materials: {err}");
        Vec::new()
    });
    let material_colors: Vec<Option<[u8; 4]>> = materials
        .iter()
        .map(|m| {
            let [r, g, b] = m.diffuse?;
            Some([r, g, b, m.dissolve.unwrap_or(1.0)].map(unit_to_u8))
        })
        .collect();
    let model_color = |model: &tobj::Model| model.mesh.material_id.and_then(|id| material_colors.get(id).copied().flatten());
    let has_vertex_colors =
        |model: &tobj::Model| !model.mesh.vertex_color.is_empty() && model.mesh.vertex_color.len() == model.mesh.positions.len();
    let with_colors = models.iter().any(|m| model_color(m).is_some() || has_vertex_colors(m));

    let mut groups: Vec<(Coloring, Primitive)> = Vec::new();
    for model in &models {
        let mesh = &model.mesh;
        let coloring = match model_color(model) {
            _ if !with_colors => Coloring::None,
            Some(color) => Coloring::Material(color),
            None if has_vertex_colors(model) => Coloring::VertexColors,
            None => Coloring::White,
        };
        let index = match groups.iter().position(|(c, _)| *c == coloring) {
            Some(index) => index,
            None => {
                let base_color = match coloring {
                    Coloring::Material(c) => Some([srgb_to_linear(c[0]), srgb_to_linear(c[1]), srgb_to_linear(c[2]), c[3] as f32 / 255.0]),
                    Coloring::VertexColors | Coloring::White => Some([1.0; 4]),
                    Coloring::None => None,
                };
                let colors = (coloring == Coloring::VertexColors).then(Vec::new);
                groups.push((coloring, Primitive { base_color, colors, ..Primitive::default() }));
                groups.len() - 1
            }
        };
        let primitive = &mut groups[index].1;
        let positions: Vec<[f32; 3]> = mesh.positions.chunks_exact(3).map(|p| [p[0], p[1], p[2]]).collect();
        let indices: Vec<u32> = mesh.indices.iter().copied().filter(|&i| (i as usize) < positions.len()).collect();
        if indices.len() != mesh.indices.len() {
            bail!("OBJ face with a vertex out of range");
        }
        let normals = if mesh.normals.len() == mesh.positions.len() {
            mesh.normals.chunks_exact(3).map(|n| [n[0], n[1], n[2]]).collect()
        } else {
            vertex_normals(&positions, &indices)
        };
        if let Some(colors) = &mut primitive.colors {
            colors.extend(mesh.vertex_color.chunks_exact(3).map(|c| {
                let [r, g, b] = [c[0], c[1], c[2]].map(|v| srgb_to_linear(unit_to_u8(v)));
                [r, g, b, 1.0]
            }));
        }
        let offset = primitive.positions.len() as u32;
        primitive.indices.extend(indices.iter().map(|i| i + offset));
        primitive.positions.extend(positions);
        primitive.normals.extend(normals);
    }
    let primitives = groups.into_iter().map(|(_, p)| dedup(p)).filter(|p| !p.indices.is_empty()).collect();
    Ok(Mesh { primitives })
}

fn stl_mesh(stl: &[u8]) -> anyhow::Result<Mesh> {
    let mesh = stl_io::read_stl(&mut Cursor::new(stl)).context("parsing STL")?;
    let positions: Vec<[f32; 3]> = mesh.vertices.iter().map(|v| v.0).collect();
    let mut indices = Vec::with_capacity(mesh.faces.len() * 3);
    for face in &mesh.faces {
        if face.vertices.iter().any(|&i| i >= positions.len()) {
            bail!("STL face with a vertex out of range");
        }
        indices.extend(face.vertices.map(|i| i as u32));
    }
    let primitive = crease(&positions, None, &indices, CREASE_DEGREES.to_radians().cos());
    Ok(Mesh { primitives: vec![primitive] })
}

/// `obj_to_mesh3d`'s byte of a 0..1 value.
fn unit_to_u8(c: f32) -> u8 {
    (c.clamp(0.0, 1.0) * 255.0).round() as u8
}

/// An sRGB byte as linear 0..1 (as Rerun and three.js decode sRGB colours).
fn srgb_to_linear(c: u8) -> f32 {
    let c = c as f32 / 255.0;
    if c <= 0.04045 { c / 12.92 } else { ((c + 0.055) / 1.055).powf(2.4) }
}

/// Inverse of [`srgb_to_linear`].
#[cfg(test)]
fn linear_to_srgb(c: f32) -> u8 {
    let c = if c <= 0.0031308 { c * 12.92 } else { 1.055 * c.powf(1.0 / 2.4) - 0.055 };
    unit_to_u8(c)
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn add_to(a: &mut [f32; 3], b: [f32; 3]) {
    a[0] += b[0];
    a[1] += b[1];
    a[2] += b[2];
}

fn normalize_or_zero(v: [f32; 3]) -> [f32; 3] {
    let len = dot(v, v).sqrt();
    if len > 0.0 && len.is_finite() { v.map(|c| c / len) } else { [0.0; 3] }
}

/// Twice the area times the unit normal of each triangle.
fn face_normals(positions: &[[f32; 3]], indices: &[u32]) -> Vec<[f32; 3]> {
    indices
        .chunks_exact(3)
        .map(|t| {
            let [a, b, c] = [t[0], t[1], t[2]].map(|i| positions[i as usize]);
            cross(sub(b, a), sub(c, a))
        })
        .collect()
}

/// Area-weighted vertex normals, as `obj_to_mesh3d` computes them for OBJs without `vn`.
fn vertex_normals(positions: &[[f32; 3]], indices: &[u32]) -> Vec<[f32; 3]> {
    let mut normals = vec![[0.0; 3]; positions.len()];
    for (t, n) in indices.chunks_exact(3).zip(face_normals(positions, indices)) {
        for &i in t {
            add_to(&mut normals[i as usize], n);
        }
    }
    normals.into_iter().map(normalize_or_zero).collect()
}

/// Merges binary-identical vertices (position, normal, colour): lossless.
fn dedup(p: Primitive) -> Primitive {
    let vertices: Vec<[f32; 10]> = (0..p.positions.len())
        .map(|i| {
            let [x, y, z] = p.positions[i];
            let [nx, ny, nz] = p.normals[i];
            let [r, g, b, a] = p.colors.as_ref().map_or([0.0; 4], |c| c[i]);
            [x, y, z, nx, ny, nz, r, g, b, a]
        })
        .collect();
    let (count, remap) = meshopt::generate_vertex_remap(&vertices, Some(&p.indices));
    let vertices = meshopt::remap_vertex_buffer(&vertices, count, &remap);
    let indices = meshopt::remap_index_buffer(Some(&p.indices), count, &remap);
    Primitive {
        positions: vertices.iter().map(|v| [v[0], v[1], v[2]]).collect(),
        normals: vertices.iter().map(|v| [v[3], v[4], v[5]]).collect(),
        colors: p.colors.is_some().then(|| vertices.iter().map(|v| [v[6], v[7], v[8], v[9]]).collect()),
        indices,
        base_color: p.base_color,
    }
}

/// Triangles over shared `positions` → a primitive whose vertex normals average the faces around
/// each corner that meet its face within the crease angle (`cos_crease`); corners with the same
/// position, normal and colour share a vertex.
fn crease(positions: &[[f32; 3]], colors: Option<&[[f32; 4]]>, indices: &[u32], cos_crease: f32) -> Primitive {
    let faces = face_normals(positions, indices);
    let units: Vec<[f32; 3]> = faces.iter().copied().map(normalize_or_zero).collect();
    // The faces around each vertex (CSR).
    let mut start = vec![0u32; positions.len() + 1];
    for &i in indices {
        start[i as usize + 1] += 1;
    }
    for i in 0..positions.len() {
        start[i + 1] += start[i];
    }
    let mut fill = start.clone();
    let mut around = vec![0u32; indices.len()];
    for (corner, &i) in indices.iter().enumerate() {
        around[fill[i as usize] as usize] = (corner / 3) as u32;
        fill[i as usize] += 1;
    }
    let mut out = Primitive { colors: colors.map(|_| Vec::new()), ..Primitive::default() };
    let mut seen: HashMap<(u32, [i16; 3]), u32> = HashMap::new();
    for (corner, &v) in indices.iter().enumerate() {
        let face = corner / 3;
        let mut normal = [0.0; 3];
        for &other in &around[start[v as usize] as usize..start[v as usize + 1] as usize] {
            if other as usize == face || dot(units[other as usize], units[face]) >= cos_crease {
                add_to(&mut normal, faces[other as usize]);
            }
        }
        let normal = match normalize_or_zero(normal) {
            [0.0, 0.0, 0.0] => units[face],
            n => n,
        };
        let key = (v, normal.map(|c| (c * 4096.0).round() as i16));
        let index = *seen.entry(key).or_insert_with(|| {
            out.positions.push(positions[v as usize]);
            out.normals.push(normal);
            if let (Some(out), Some(colors)) = (&mut out.colors, colors) {
                out.push(colors[v as usize]);
            }
            out.positions.len() as u32 - 1
        });
        out.indices.push(index);
    }
    out
}

/// The decimated mesh, if decimating pays off ([`LOD_RATIO`], [`LOD_MIN_TRIANGLES`]).
fn lod(mesh: &Mesh) -> Option<Mesh> {
    let total = mesh.triangles();
    let target = ((total as f32 * LOD_RATIO).ceil() as usize).max(LOD_MIN_TRIANGLES);
    if target as f32 > total as f32 * LOD_MAX_SHARE {
        return None;
    }
    let cos_crease = CREASE_DEGREES.to_radians().cos();
    let mut primitives = Vec::new();
    for p in &mesh.primitives {
        let triangles = p.indices.len() / 3;
        let goal = (triangles * target).div_ceil(total).max(1);
        // Welded by position only: the simplifier needs the topology, normals are recomputed.
        let (count, remap) = meshopt::generate_vertex_remap(&p.positions, Some(&p.indices));
        let positions = meshopt::remap_vertex_buffer(&p.positions, count, &remap);
        let colors = p.colors.as_ref().map(|c| meshopt::remap_vertex_buffer(c, count, &remap));
        let indices = meshopt::remap_index_buffer(Some(&p.indices), count, &remap);
        let simplified = if goal >= triangles {
            indices
        } else {
            let adapter = meshopt::VertexDataAdapter::new(meshopt::typed_to_bytes(&positions), 12, 0).ok()?;
            let options = meshopt::SimplifyOptions::Prune;
            let simplified = meshopt::simplify(&indices, &adapter, goal * 3, 0.02, options, None);
            if simplified.len() / 3 > goal * 2 {
                // Topology (many small parts, borders) held it back: cluster instead.
                meshopt::simplify_sloppy(&simplified, &adapter, goal * 3, f32::MAX, None)
            } else {
                simplified
            }
        };
        if simplified.is_empty() {
            continue;
        }
        let mut primitive = crease(&positions, colors.as_deref(), &simplified, cos_crease);
        primitive.base_color = p.base_color;
        primitives.push(primitive);
    }
    let lod = Mesh { primitives };
    let kept = lod.triangles();
    (kept > 0 && (kept as f32) <= total as f32 * LOD_MAX_SHARE).then_some(lod)
}

/// glTF component types and buffer targets.
const FLOAT: u32 = 5126;
const UNSIGNED_SHORT: u32 = 5123;
const UNSIGNED_INT: u32 = 5125;
const ARRAY_BUFFER: u32 = 34962;
const ELEMENT_ARRAY_BUFFER: u32 = 34963;

/// The binary glTF container: one node, one mesh, a primitive per material.
fn write_glb(mesh: &Mesh) -> Vec<u8> {
    let mut bin: Vec<u8> = Vec::new();
    let mut views = Vec::new();
    let mut accessors = Vec::new();
    let mut materials: Vec<serde_json::Value> = Vec::new();
    let mut material_of: Vec<[u32; 4]> = Vec::new();
    let mut view = |bin: &mut Vec<u8>, bytes: &[u8], target: u32| {
        while !bin.len().is_multiple_of(4) {
            bin.push(0);
        }
        views.push(json!({ "buffer": 0, "byteOffset": bin.len(), "byteLength": bytes.len(), "target": target }));
        bin.extend_from_slice(bytes);
        views.len() - 1
    };
    let floats = |values: &[f32]| values.iter().flat_map(|v| v.to_le_bytes()).collect::<Vec<u8>>();
    let mut primitives = Vec::new();
    for p in &mesh.primitives {
        let count = p.positions.len();
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        for v in &p.positions {
            for k in 0..3 {
                min[k] = min[k].min(v[k]);
                max[k] = max[k].max(v[k]);
            }
        }
        let position = view(&mut bin, &floats(p.positions.as_flattened()), ARRAY_BUFFER);
        accessors.push(json!({ "bufferView": position, "componentType": FLOAT, "count": count, "type": "VEC3", "min": min, "max": max }));
        let mut attributes = json!({ "POSITION": accessors.len() - 1 });
        let normal = view(&mut bin, &floats(p.normals.as_flattened()), ARRAY_BUFFER);
        accessors.push(json!({ "bufferView": normal, "componentType": FLOAT, "count": count, "type": "VEC3" }));
        attributes["NORMAL"] = json!(accessors.len() - 1);
        if let Some(colors) = &p.colors {
            let color = view(&mut bin, &floats(colors.as_flattened()), ARRAY_BUFFER);
            accessors.push(json!({ "bufferView": color, "componentType": FLOAT, "count": count, "type": "VEC4" }));
            attributes["COLOR_0"] = json!(accessors.len() - 1);
        }
        let (bytes, component): (Vec<u8>, u32) = if count <= u16::MAX as usize {
            (p.indices.iter().flat_map(|&i| (i as u16).to_le_bytes()).collect(), UNSIGNED_SHORT)
        } else {
            (p.indices.iter().flat_map(|i| i.to_le_bytes()).collect(), UNSIGNED_INT)
        };
        let indices = view(&mut bin, &bytes, ELEMENT_ARRAY_BUFFER);
        accessors.push(json!({ "bufferView": indices, "componentType": component, "count": p.indices.len(), "type": "SCALAR" }));
        let mut primitive = json!({ "attributes": attributes, "indices": accessors.len() - 1, "mode": 4 });
        if let Some(color) = p.base_color {
            let bits = color.map(f32::to_bits);
            let index = material_of.iter().position(|m| *m == bits).unwrap_or_else(|| {
                let mut material = json!({
                    "pbrMetallicRoughness": { "baseColorFactor": color, "metallicFactor": 0.0, "roughnessFactor": 0.8 },
                    "doubleSided": true,
                });
                if color[3] < 1.0 {
                    material["alphaMode"] = json!("BLEND");
                }
                materials.push(material);
                material_of.push(bits);
                materials.len() - 1
            });
            primitive["material"] = json!(index);
        }
        primitives.push(primitive);
    }
    while !bin.len().is_multiple_of(4) {
        bin.push(0);
    }
    let mut gltf = json!({
        "asset": { "version": "2.0", "generator": format!("factoryos-backend mesh_convert {VERSION}") },
        "scene": 0,
        "scenes": [{ "nodes": [0] }],
        "nodes": [{ "mesh": 0 }],
        "meshes": [{ "primitives": primitives }],
        "accessors": accessors,
        "bufferViews": views,
        "buffers": [{ "byteLength": bin.len() }],
    });
    if !materials.is_empty() {
        gltf["materials"] = json!(materials);
    }
    let mut json = serde_json::to_vec(&gltf).expect("glTF JSON");
    while !json.len().is_multiple_of(4) {
        json.push(b' ');
    }
    let total = 12 + 8 + json.len() + 8 + bin.len();
    let mut glb = Vec::with_capacity(total);
    glb.extend_from_slice(b"glTF");
    glb.extend_from_slice(&2u32.to_le_bytes());
    glb.extend_from_slice(&(total as u32).to_le_bytes());
    glb.extend_from_slice(&(json.len() as u32).to_le_bytes());
    glb.extend_from_slice(b"JSON");
    glb.extend_from_slice(&json);
    glb.extend_from_slice(&(bin.len() as u32).to_le_bytes());
    glb.extend_from_slice(b"BIN\0");
    glb.extend_from_slice(&bin);
    glb
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;

    /// The fixtures of `factoryos_urdf_materials::materials::obj`'s tests, with the colours
    /// `obj_to_mesh3d` gives them there (sRGB RGBA).
    const TWO_MATERIALS_OBJ: &str = "mtllib m.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nusemtl Red\nf 1 2 3\nusemtl Blue\nf 1 2 4\n";
    const TWO_MATERIALS_MTL: &str = "newmtl Red\nKd 1.0 0.0 0.0\nnewmtl Blue\nKd 0.0 0.0 0.5\nd 0.5\n";
    const MIXED_OBJ: &str = "mtllib a.mtl\nmtllib sub/b.mtl\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nv 0 0 1\nvn 0 0 1\n\
        o first\nusemtl Green\nf 1//1 2//1 3//1 4//1\no second\nusemtl Plain\nf 1 2 5\no third\nusemtl Grey\nf 2 3 5\no fourth\nf 3 4 5\n";
    const MIXED_MTL_A: &str = "newmtl Green\nKd 0.0 0.8 0.2\nnewmtl Plain\nKa 0.1 0.1 0.1\n";
    const MIXED_MTL_B: &str = "newmtl Grey\nKd 0.4 0.4 0.4\n";

    fn sidecars<'a>(files: &'a [(&'a str, &'a str)]) -> impl Fn(&str) -> Option<Vec<u8>> + 'a {
        move |name| files.iter().find(|(file, _)| *file == name).map(|(_, c)| c.as_bytes().to_vec())
    }

    /// A GLB read back with the `gltf` crate: per primitive, its vertex count, its triangles'
    /// positions, its sRGB material colour (`None`: no material) and whether it has normals.
    struct Read {
        primitives: Vec<(usize, Vec<[[f32; 3]; 3]>, Option<u32>, bool)>,
    }

    fn read(glb: &[u8]) -> Read {
        let (doc, buffers, _) = gltf::import_slice(glb).expect("a valid GLB");
        let mesh = doc.meshes().next().unwrap();
        let primitives = mesh
            .primitives()
            .map(|p| {
                let reader = p.reader(|b| Some(&buffers[b.index()]));
                let positions: Vec<[f32; 3]> = reader.read_positions().unwrap().collect();
                let indices: Vec<u32> = reader.read_indices().unwrap().into_u32().collect();
                let triangles = indices.chunks_exact(3).map(|t| [t[0], t[1], t[2]].map(|i| positions[i as usize])).collect();
                let color = p.material().index().map(|_| {
                    let [r, g, b, a] = p.material().pbr_metallic_roughness().base_color_factor();
                    u32::from_be_bytes([linear_to_srgb(r), linear_to_srgb(g), linear_to_srgb(b), unit_to_u8(a)])
                });
                (positions.len(), triangles, color, reader.read_normals().is_some())
            })
            .collect();
        Read { primitives }
    }

    #[test]
    fn obj_mtl_round_trip() {
        let mtls = [("m.mtl", TWO_MATERIALS_MTL)];
        let out = convert(Source::Obj, TWO_MATERIALS_OBJ.as_bytes(), &sidecars(&mtls)).unwrap();
        assert_eq!((out.triangles, out.lod_triangles), (2, None));
        let glb = read(&out.glb);
        // obj_to_mesh3d: 3 × 0xFF0000FF, 3 × 0x00008080.
        let summary: Vec<_> = glb.primitives.iter().map(|(v, t, c, n)| (*v, t.len(), *c, *n)).collect();
        assert_eq!(summary, [(3, 1, Some(0xFF0000FF), true), (3, 1, Some(0x00008080), true)]);
        assert_eq!(glb.primitives[1].1[0], [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]);
        let (doc, _, _) = gltf::import_slice(&out.glb).unwrap();
        let blue = doc.materials().nth(1).unwrap();
        assert_eq!(blue.alpha_mode(), gltf::material::AlphaMode::Blend);
        assert_eq!(blue.pbr_metallic_roughness().metallic_factor(), 0.0);
    }

    #[test]
    fn mixed_obj_colors_match_obj_to_mesh3d() {
        let mtls = [("a.mtl", MIXED_MTL_A), ("sub/b.mtl", MIXED_MTL_B)];
        let out = convert(Source::Obj, MIXED_OBJ.as_bytes(), &sidecars(&mtls)).unwrap();
        let glb = read(&out.glb);
        // obj_to_mesh3d: 4 × green (the quad), 3 × white (`Plain` has no Kd), 6 × grey (`fourth`
        // keeps `Grey`, as tobj reads it); 5 triangles.
        let summary: Vec<_> = glb.primitives.iter().map(|(v, t, c, _)| (*v, t.len(), *c)).collect();
        assert_eq!(summary, [(4, 2, Some(0x00CC33FF)), (3, 1, Some(0xFFFFFFFF)), (6, 2, Some(0x666666FF))]);
    }

    #[test]
    fn obj_without_materials_has_no_material() {
        let out = convert(Source::Obj, TWO_MATERIALS_OBJ.as_bytes(), &|_| None).unwrap();
        let glb = read(&out.glb);
        let summary: Vec<_> = glb.primitives.iter().map(|(v, t, c, _)| (*v, t.len(), *c)).collect();
        // Without materials, `usemtl` doesn't split the OBJ (tobj): 4 vertices, as obj_to_mesh3d.
        assert_eq!(summary, [(4, 2, None)]);
        let (doc, _, _) = gltf::import_slice(&out.glb).unwrap();
        assert_eq!(doc.materials().count(), 0);
    }

    #[test]
    fn obj_vertex_colors_become_color_0() {
        let obj = "v 0 0 0 1 0 0\nv 1 0 0 1 0 0\nv 0 1 0 1 0 0\nf 1 2 3\n";
        let out = convert(Source::Obj, obj.as_bytes(), &|_| None).unwrap();
        let (doc, buffers, _) = gltf::import_slice(&out.glb).unwrap();
        let p = doc.meshes().next().unwrap().primitives().next().unwrap();
        let colors: Vec<[f32; 4]> = p.reader(|b| Some(&buffers[b.index()])).read_colors(0).unwrap().into_rgba_f32().collect();
        assert_eq!(colors, [[1.0, 0.0, 0.0, 1.0]; 3]);
    }

    #[test]
    fn invalid_input_is_an_error() {
        assert!(convert(Source::Obj, b"f 1 2 3\n", &|_| None).is_err());
        assert!(convert(Source::Obj, b"v 0 0 0\n", &|_| None).is_err(), "no triangles");
        assert!(convert(Source::Stl, b"garbage", &|_| None).is_err());
    }

    /// A UV sphere: `rings × segments × 2` triangles over shared vertices.
    fn sphere_obj(rings: usize, segments: usize) -> String {
        let mut obj = String::from("mtllib s.mtl\nusemtl Orange\n");
        for r in 0..=rings {
            let theta = std::f32::consts::PI * r as f32 / rings as f32;
            for s in 0..segments {
                let phi = std::f32::consts::TAU * s as f32 / segments as f32;
                obj += &format!("v {} {} {}\n", theta.sin() * phi.cos(), theta.sin() * phi.sin(), theta.cos());
            }
        }
        for r in 0..rings {
            for s in 0..segments {
                let a = r * segments + s + 1;
                let b = r * segments + (s + 1) % segments + 1;
                let (c, d) = (a + segments, b + segments);
                obj += &format!("f {a} {b} {d}\nf {a} {d} {c}\n");
            }
        }
        obj
    }

    #[test]
    fn lod_reduces_triangles() {
        let obj = sphere_obj(60, 80);
        let mtls = [("s.mtl", "newmtl Orange\nKd 0.5120000243186951 0.2088140994310379 0.07027201354503632\n")];
        let out = convert(Source::Obj, obj.as_bytes(), &sidecars(&mtls)).unwrap();
        assert_eq!(out.triangles, 9600);
        let lod = out.lod.expect("a LOD");
        let kept = out.lod_triangles.unwrap();
        assert!((LOD_MIN_TRIANGLES..=out.triangles / 4).contains(&kept), "{kept} of {}", out.triangles);
        let read = read(&lod);
        assert_eq!(read.primitives.iter().map(|p| p.1.len()).sum::<usize>(), kept);
        assert_eq!(read.primitives[0].2, self::read(&out.glb).primitives[0].2, "the same colour");
        assert!(lod.len() * 3 < out.glb.len(), "{} vs {}", lod.len(), out.glb.len());
        // Small meshes are their own LOD.
        let small = convert(Source::Obj, sphere_obj(10, 20).as_bytes(), &sidecars(&mtls)).unwrap();
        assert_eq!((small.triangles, small.lod_triangles), (400, None));
    }

    #[test]
    fn stl_to_glb() {
        // A binary STL of a tetrahedron.
        let p = [[0.0f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let faces = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
        let mut stl = vec![0u8; 80];
        stl.extend_from_slice(&(faces.len() as u32).to_le_bytes());
        for f in faces {
            stl.extend_from_slice(&[0u8; 12]);
            for i in f {
                stl.extend(p[i].iter().flat_map(|c| c.to_le_bytes()));
            }
            stl.extend_from_slice(&[0u8; 2]);
        }
        let out = convert(Source::Stl, &stl, &|_| None).unwrap();
        assert_eq!(out.triangles, 4);
        let glb = read(&out.glb);
        // Every edge is sharper than the crease angle: flat, 3 vertices per face.
        assert_eq!(glb.primitives.iter().map(|p| (p.0, p.1.len(), p.2, p.3)).collect::<Vec<_>>(), [(12, 4, None, true)]);
    }

    #[test]
    fn colors_survive_linear_storage() {
        for c in 0..=255u8 {
            assert_eq!(linear_to_srgb(srgb_to_linear(c)), c);
        }
    }

    /// The starter catalogue's meshes: every OBJ and STL converts, and what that costs.
    /// `cargo test -p factoryos-backend --release catalogue_conversion -- --ignored --nocapture`
    #[test]
    #[ignore = "measures the starter catalogue"]
    fn catalogue_conversion() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/catalogue");
        let mut files = Vec::new();
        let mut stack = vec![root.clone()];
        while let Some(dir) = stack.pop() {
            for entry in std::fs::read_dir(&dir).into_iter().flatten().flatten() {
                let path = entry.path();
                if path.is_dir() {
                    stack.push(path);
                } else if Source::of(&path).is_some() {
                    files.push(path);
                }
            }
        }
        files.sort();
        let (mut source_bytes, mut glb_bytes, mut lod_bytes, mut first_bytes) = (0u64, 0u64, 0u64, 0u64);
        let (mut tris, mut lod_tris) = (0usize, 0usize);
        let start = std::time::Instant::now();
        let mut slowest = (std::time::Duration::ZERO, PathBuf::new());
        for path in &files {
            let bytes = std::fs::read(path).unwrap();
            let dir = path.parent().unwrap().to_owned();
            let mut sidecar_bytes = 0u64;
            let read_sidecar = |name: &str| std::fs::read(dir.join(name)).ok();
            let t = std::time::Instant::now();
            let out = convert(Source::of(path).unwrap(), &bytes, &read_sidecar).unwrap();
            let took = t.elapsed();
            if took > slowest.0 {
                slowest = (took, path.clone());
            }
            if Source::of(path) == Some(Source::Obj) {
                for name in std::str::from_utf8(&bytes).unwrap_or("").lines().filter_map(|l| l.strip_prefix("mtllib ")) {
                    sidecar_bytes += std::fs::metadata(dir.join(name.trim())).map_or(0, |m| m.len());
                }
            }
            source_bytes += bytes.len() as u64 + sidecar_bytes;
            glb_bytes += out.glb.len() as u64;
            lod_bytes += out.lod.as_ref().map_or(0, |l| l.len() as u64);
            first_bytes += out.lod.as_ref().map_or(out.glb.len(), Vec::len) as u64;
            tris += out.triangles;
            lod_tris += out.lod_triangles.unwrap_or(out.triangles);
        }
        let mb = |b: u64| b as f64 / 1e6;
        eprintln!(
            "{} meshes in {:.2?} (slowest {:.2?}: {}): source (OBJ+MTL, STL) {:.1} MB, GLB {:.1} MB, LOD {:.1} MB \
             (first frame, LOD else GLB: {:.1} MB); {tris} → {lod_tris} triangles",
            files.len(),
            start.elapsed(),
            slowest.0,
            slowest.1.display(),
            mb(source_bytes),
            mb(glb_bytes),
            mb(lod_bytes),
            mb(first_bytes),
        );
    }
}
