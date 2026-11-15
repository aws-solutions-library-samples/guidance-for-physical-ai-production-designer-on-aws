//! Wavefront OBJ with `.mtl` materials → [`Mesh3D`], with each material's diffuse color baked into the vertex colors.

use anyhow::Context as _;
use re_sdk_types::archetypes::Mesh3D;
use re_sdk_types::components::Color;
use re_sdk_types::encodings::Rgba32;
use re_sdk_types::external::glam::Vec3;

/// How to read the `Kd` (diffuse) values of an `.mtl` file.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ColorSpace {
    /// `Kd` is sRGB scaled to 0..1. This is how trimesh (viser, yourdfpy) and three.js read it.
    #[default]
    Srgb,

    /// `Kd` is linear, as written by e.g. Blender.
    Linear,
}

/// Converts the bytes of an OBJ file to a [`Mesh3D`], with the diffuse colors of its `mtllib`
/// materials as vertex colors. No filesystem access, so it also works in the web viewer.
///
/// `load_sidecar` returns the bytes of a `mtllib` file by its name exactly as written in the OBJ
/// (usually relative to the OBJ file), or `None` if it is not available.
///
/// Faces without a material color get the OBJ's own vertex colors, else white. If no face has
/// either (e.g. the `.mtl` is missing), the mesh has no vertex colors at all.
pub fn obj_to_mesh3d(
    obj: &[u8],
    load_sidecar: &dyn Fn(&str) -> Option<Vec<u8>>,
    color_space: ColorSpace,
) -> anyhow::Result<Mesh3D> {
    let obj = ParsedObj::parse(obj, load_sidecar, color_space, "")?;
    let with_colors = obj.has_material_colors() || obj.has_vertex_colors();
    Ok(obj.into_mesh(with_colors))
}

/// Loads an OBJ file together with the materials of its `mtllib`s (resolved next to the OBJ file):
/// [`obj_to_mesh3d`] with the files read from disk.
///
/// Returns `Ok(None)` if none of its faces use a material with a diffuse color,
/// i.e. when Rerun's own OBJ support already shows the file correctly.
#[cfg(not(target_arch = "wasm32"))]
pub fn load_obj_with_materials(
    path: &std::path::Path,
    color_space: ColorSpace,
) -> anyhow::Result<Option<Mesh3D>> {
    let obj = std::fs::read(path)
        .with_context(|| format!("Failed to load OBJ\nFile path: {}", path.display()))?;
    // Like `tobj::load_obj`.
    let load_sidecar = |name: &str| match path.parent() {
        Some(dir) => std::fs::read(dir.join(name)).ok(),
        None => std::fs::read(name).ok(),
    };
    let source = format!("\nFile path: {}", path.display());
    let obj = ParsedObj::parse(&obj, &load_sidecar, color_space, &source)?;
    Ok(obj.has_material_colors().then(|| obj.into_mesh(true)))
}

/// The models of an OBJ file and the colors of its materials.
struct ParsedObj {
    models: Vec<tobj::Model>,
    material_colors: Vec<Option<Color>>,
}

impl ParsedObj {
    /// `source` is appended to warnings and errors, e.g. `"\nFile path: …"`.
    fn parse(
        obj: &[u8],
        load_sidecar: &dyn Fn(&str) -> Option<Vec<u8>>,
        color_space: ColorSpace,
        source: &str,
    ) -> anyhow::Result<Self> {
        let options = tobj::LoadOptions {
            single_index: true,
            triangulate: true,
            ..Default::default()
        };
        let (models, materials) =
            tobj::load_obj_buf(&mut std::io::Cursor::new(obj), &options, |mtl_path| {
                // The same error as `tobj::load_mtl` for a missing file.
                let mtl = load_sidecar(&mtl_path.to_string_lossy())
                    .ok_or(tobj::LoadError::OpenFileFailed)?;
                tobj::load_mtl_buf(&mut std::io::Cursor::new(mtl))
            })
            .with_context(|| format!("Failed to load OBJ{source}"))?;

        let materials = match materials {
            Ok(materials) => materials,
            Err(err) => {
                re_log::warn_once!("Failed to load the materials of an OBJ: {err}{source}");
                Vec::new()
            }
        };
        if materials
            .iter()
            .any(|material| material.diffuse_texture.is_some())
        {
            re_log::warn_once!(
                "OBJ material textures are not supported, using the diffuse colors only{source}"
            );
        }

        let material_colors = materials
            .iter()
            .map(|material| {
                let [r, g, b] = material.diffuse?;
                Some(to_color(
                    [r, g, b, material.dissolve.unwrap_or(1.0)],
                    color_space,
                ))
            })
            .collect();
        Ok(Self {
            models,
            material_colors,
        })
    }

    fn model_color(&self, model: &tobj::Model) -> Option<Color> {
        model
            .mesh
            .material_id
            .and_then(|id| self.material_colors.get(id).copied().flatten())
    }

    /// Whether any face uses a material with a diffuse color.
    fn has_material_colors(&self) -> bool {
        self.models
            .iter()
            .any(|model| self.model_color(model).is_some())
    }

    /// Whether any model has `v x y z r g b` vertex colors.
    fn has_vertex_colors(&self) -> bool {
        self.models.iter().any(|model| {
            !model.mesh.vertex_color.is_empty()
                && model.mesh.vertex_color.len() == model.mesh.positions.len()
        })
    }

    /// `with_colors`: whether the mesh gets vertex colors (white for faces without any color).
    fn into_mesh(self, with_colors: bool) -> Mesh3D {
        let mut positions: Vec<Vec3> = Vec::new();
        let mut normals: Vec<Vec3> = Vec::new();
        let mut colors: Vec<Color> = Vec::new();
        let mut triangles: Vec<[u32; 3]> = Vec::new();

        for model in &self.models {
            let mesh = &model.mesh;
            let first_vertex = positions.len();
            let model_positions: Vec<Vec3> = mesh
                .positions
                .chunks_exact(3)
                .map(Vec3::from_slice)
                .collect();
            let model_triangles: Vec<[u32; 3]> = mesh
                .indices
                .chunks_exact(3)
                .map(|t| [t[0], t[1], t[2]])
                .collect();

            if mesh.normals.len() == mesh.positions.len() {
                normals.extend(mesh.normals.chunks_exact(3).map(Vec3::from_slice));
            } else {
                normals.extend(vertex_normals(&model_positions, &model_triangles));
            }

            if with_colors {
                match self.model_color(model) {
                    Some(color) => {
                        colors.extend(std::iter::repeat_n(color, model_positions.len()));
                    }
                    None if mesh.vertex_color.len() == mesh.positions.len() => colors.extend(
                        mesh.vertex_color
                            .chunks_exact(3)
                            .map(|c| to_color([c[0], c[1], c[2], 1.0], ColorSpace::Srgb)),
                    ),
                    None => colors.extend(std::iter::repeat_n(
                        Color::new(Rgba32::WHITE),
                        model_positions.len(),
                    )),
                }
            }

            let offset = first_vertex as u32;
            triangles.extend(model_triangles.iter().map(|t| t.map(|i| i + offset)));
            positions.extend(model_positions);
        }

        let mesh = Mesh3D::new(positions.iter().map(|p| p.to_array()))
            .with_triangle_indices(triangles)
            .with_vertex_normals(normals.iter().map(|n| n.to_array()));
        if with_colors {
            mesh.with_vertex_colors(colors)
        } else {
            mesh
        }
    }
}

fn to_color([r, g, b, a]: [f32; 4], color_space: ColorSpace) -> Color {
    match color_space {
        ColorSpace::Srgb => {
            let to_u8 = |c: f32| (c.clamp(0.0, 1.0) * 255.0).round() as u8;
            Color::from_unmultiplied_rgba(to_u8(r), to_u8(g), to_u8(b), to_u8(a))
        }
        ColorSpace::Linear => Color::new(Rgba32::from_linear_unmultiplied_rgba_f32(r, g, b, a)),
    }
}

/// Area-weighted vertex normals, for OBJ files without `vn`.
///
/// Vertices that are not shared between faces (common in CAD exports) get their face's normal, i.e. flat shading.
fn vertex_normals(positions: &[Vec3], triangles: &[[u32; 3]]) -> Vec<Vec3> {
    let mut normals = vec![Vec3::ZERO; positions.len()];
    for &[a, b, c] in triangles {
        let (a, b, c) = (a as usize, b as usize, c as usize);
        let (Some(pa), Some(pb), Some(pc)) = (positions.get(a), positions.get(b), positions.get(c))
        else {
            continue;
        };
        // Not normalized: its length is twice the triangle's area, which weights it.
        let face_normal = (*pb - *pa).cross(*pc - *pa);
        normals[a] += face_normal;
        normals[b] += face_normal;
        normals[c] += face_normal;
    }
    normals.into_iter().map(Vec3::normalize_or_zero).collect()
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use re_sdk_types::AsComponents as _;

    use super::*;

    fn write_files(dir: &Path, files: &[(&str, &str)]) {
        std::fs::create_dir_all(dir).unwrap();
        for (name, contents) in files {
            std::fs::write(dir.join(name), contents).unwrap();
        }
    }

    const TWO_MATERIALS_OBJ: &str = "mtllib m.mtl
v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
usemtl Red
f 1 2 3
usemtl Blue
f 1 2 4
";

    const TWO_MATERIALS_MTL: &str = "newmtl Red
Kd 1.0 0.0 0.0
newmtl Blue
Kd 0.0 0.0 0.5
d 0.5
";

    /// Two `mtllib`s (one in a subdirectory), a material without `Kd`, a quad (triangulated),
    /// `vn` normals for one model, and a model without `usemtl`.
    const MIXED_OBJ: &str = "mtllib a.mtl
mtllib sub/b.mtl
v 0 0 0
v 1 0 0
v 1 1 0
v 0 1 0
v 0 0 1
vn 0 0 1
o first
usemtl Green
f 1//1 2//1 3//1 4//1
o second
usemtl Plain
f 1 2 5
o third
usemtl Grey
f 2 3 5
o fourth
f 3 4 5
";

    const MIXED_MTL_A: &str = "newmtl Green
Kd 0.0 0.8 0.2
newmtl Plain
Ka 0.1 0.1 0.1
";

    const MIXED_MTL_B: &str = "newmtl Grey
Kd 0.4 0.4 0.4
";

    fn component_len(mesh: &Mesh3D, descriptor: &re_sdk_types::ComponentDescriptor) -> usize {
        mesh.as_serialized_batches()
            .into_iter()
            .find(|batch| &batch.descriptor == descriptor)
            .map_or(0, |batch| batch.array.len())
    }

    fn vertex_colors(mesh: &Mesh3D) -> Option<Vec<u32>> {
        let colors = mesh.vertex_colors.as_ref()?;
        let colors = colors
            .array
            .as_any()
            .downcast_ref::<re_sdk_types::external::arrow::array::UInt32Array>()
            .unwrap();
        Some(colors.values().to_vec())
    }

    /// In-memory sidecars, by their name in the OBJ.
    fn sidecars<'a>(files: &'a [(&'a str, &'a str)]) -> impl Fn(&str) -> Option<Vec<u8>> + 'a {
        move |name| {
            files
                .iter()
                .find(|(file, _)| *file == name)
                .map(|(_, contents)| contents.as_bytes().to_vec())
        }
    }

    #[test]
    fn material_colors_become_vertex_colors() {
        let dir = std::env::temp_dir().join("rerun_urdf_materials_two_materials");
        write_files(
            &dir,
            &[("m.obj", TWO_MATERIALS_OBJ), ("m.mtl", TWO_MATERIALS_MTL)],
        );

        let mesh = load_obj_with_materials(&dir.join("m.obj"), ColorSpace::Srgb)
            .unwrap()
            .unwrap();

        // One vertex set per material (`usemtl` splits the OBJ into models).
        assert_eq!(
            vertex_colors(&mesh).unwrap(),
            [0xFF0000FF; 3]
                .into_iter()
                .chain([0x00008080; 3])
                .collect::<Vec<_>>()
        );
        assert_eq!(
            component_len(&mesh, &Mesh3D::descriptor_vertex_positions()),
            6
        );
        assert_eq!(
            component_len(&mesh, &Mesh3D::descriptor_vertex_normals()),
            6
        );
        assert_eq!(
            component_len(&mesh, &Mesh3D::descriptor_triangle_indices()),
            2
        );
    }

    #[test]
    fn bytes_and_files_give_the_same_mesh() {
        for (name, obj, mtls) in [
            (
                "two_materials",
                TWO_MATERIALS_OBJ,
                &[("m.mtl", TWO_MATERIALS_MTL)][..],
            ),
            (
                "mixed",
                MIXED_OBJ,
                &[("a.mtl", MIXED_MTL_A), ("sub/b.mtl", MIXED_MTL_B)][..],
            ),
        ] {
            for color_space in [ColorSpace::Srgb, ColorSpace::Linear] {
                let dir = std::env::temp_dir().join(format!("rerun_urdf_materials_bytes_{name}"));
                write_files(&dir, &[("m.obj", obj)]);
                std::fs::create_dir_all(dir.join("sub")).unwrap();
                write_files(&dir, mtls);

                let from_file = load_obj_with_materials(&dir.join("m.obj"), color_space)
                    .unwrap()
                    .unwrap();
                let from_bytes =
                    obj_to_mesh3d(obj.as_bytes(), &sidecars(mtls), color_space).unwrap();
                assert_eq!(from_bytes, from_file, "{name} {color_space:?}");
            }
        }
    }

    #[test]
    fn mixed_obj_colors() {
        let mtls = [("a.mtl", MIXED_MTL_A), ("sub/b.mtl", MIXED_MTL_B)];
        let mesh = obj_to_mesh3d(MIXED_OBJ.as_bytes(), &sidecars(&mtls), ColorSpace::Srgb).unwrap();
        let green = 0x00CC33FF;
        let grey = 0x666666FF;
        let white = 0xFFFFFFFF;
        assert_eq!(
            vertex_colors(&mesh).unwrap(),
            [green; 4]
                .into_iter()
                .chain([white; 3])
                // `fourth` has no `usemtl`, so it keeps `Grey` (tobj).
                .chain([grey; 6])
                .collect::<Vec<_>>()
        );
        // The quad is two triangles.
        assert_eq!(
            component_len(&mesh, &Mesh3D::descriptor_triangle_indices()),
            5
        );
    }

    #[test]
    fn missing_sidecar_gives_a_mesh_without_colors() {
        let mesh = obj_to_mesh3d(TWO_MATERIALS_OBJ.as_bytes(), &|_| None, ColorSpace::Srgb).unwrap();
        assert_eq!(vertex_colors(&mesh), None);
        // Without materials, `usemtl` does not split the OBJ into models (tobj).
        assert_eq!(
            component_len(&mesh, &Mesh3D::descriptor_vertex_positions()),
            4
        );
        assert_eq!(
            component_len(&mesh, &Mesh3D::descriptor_vertex_normals()),
            4
        );
        assert_eq!(
            component_len(&mesh, &Mesh3D::descriptor_triangle_indices()),
            2
        );
    }

    #[test]
    fn sidecar_is_looked_up_by_its_name_in_the_obj() {
        let requested = std::cell::RefCell::new(Vec::new());
        let load_sidecar = |name: &str| {
            requested.borrow_mut().push(name.to_owned());
            None
        };
        obj_to_mesh3d(MIXED_OBJ.as_bytes(), &load_sidecar, ColorSpace::Srgb).unwrap();
        assert_eq!(*requested.borrow(), ["a.mtl", "sub/b.mtl"]);
    }

    #[test]
    fn obj_vertex_colors_are_kept_without_materials() {
        let obj = "v 0 0 0 1 0 0\nv 1 0 0 1 0 0\nv 0 1 0 1 0 0\nf 1 2 3\n";
        let mesh = obj_to_mesh3d(obj.as_bytes(), &|_| None, ColorSpace::Srgb).unwrap();
        assert_eq!(vertex_colors(&mesh).unwrap(), [0xFF0000FF; 3]);
    }

    #[test]
    fn invalid_obj_is_an_error() {
        assert!(obj_to_mesh3d(b"f 1 2 3\n", &|_| None, ColorSpace::Srgb).is_err());
    }

    #[test]
    fn linear_colors_are_converted_to_srgb() {
        let color = to_color([0.5, 0.0, 1.0, 1.0], ColorSpace::Linear);
        assert_eq!(color.0.to_array(), [188, 0, 255, 255]);
    }

    #[test]
    fn obj_without_materials_is_left_to_rerun() {
        let dir = std::env::temp_dir().join("rerun_urdf_materials_no_materials");
        write_files(
            &dir,
            &[("plain.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")],
        );
        assert!(
            load_obj_with_materials(&dir.join("plain.obj"), ColorSpace::Srgb)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn obj_with_missing_mtl_is_left_to_rerun() {
        let dir = std::env::temp_dir().join("rerun_urdf_materials_missing_mtl");
        write_files(&dir, &[("missing.obj", TWO_MATERIALS_OBJ)]);
        assert!(
            load_obj_with_materials(&dir.join("missing.obj"), ColorSpace::Srgb)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn flat_normals_for_unshared_vertices() {
        let positions = [Vec3::ZERO, Vec3::X, Vec3::Y];
        assert_eq!(vertex_normals(&positions, &[[0, 1, 2]]), vec![Vec3::Z; 3]);
    }
}
