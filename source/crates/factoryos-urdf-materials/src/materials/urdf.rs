//! Swaps the white OBJ meshes of Rerun's URDF importer for [`Mesh3D`]s colored by their `.mtl` files.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use re_chunk::{Chunk, ChunkBuilder, ChunkId, EntityPath, RowId, TimePoint};
use re_importer::UrdfTree;
use re_sdk_types::archetypes::Asset3D;
use re_importer::external::urdf_rs::Geometry;

use super::obj::{ColorSpace, load_obj_with_materials};

/// Emits URDF trees like [`UrdfTree::emit`], but with the `.mtl` colors of their OBJ meshes.
///
/// Only the OBJ meshes that Rerun would show white are replaced:
/// meshes with a URDF `<color>` keep it (as with Rerun's importer), and meshes without usable
/// materials, or in other formats, are passed through untouched.
#[derive(Clone, Copy, Debug, Default)]
pub struct UrdfMaterials {
    pub color_space: ColorSpace,
}

impl UrdfMaterials {
    /// Like [`UrdfTree::emit`].
    ///
    /// `urdf_dir` is the directory of the `.urdf` file, which relative mesh paths are resolved against.
    pub fn emit(
        &self,
        tree: &UrdfTree,
        urdf_dir: Option<&Path>,
        emit: &mut dyn FnMut(Chunk),
        timepoint: &TimePoint,
        include_joint_transforms: bool,
    ) -> anyhow::Result<()> {
        let obj_meshes = obj_meshes(tree, urdf_dir);
        tree.emit(
            &mut |chunk| self.replace(chunk, &obj_meshes, timepoint, emit),
            timepoint,
            include_joint_transforms,
        )
    }

    fn replace(
        &self,
        chunk: Chunk,
        obj_meshes: &HashMap<EntityPath, PathBuf>,
        timepoint: &TimePoint,
        emit: &mut dyn FnMut(Chunk),
    ) {
        let Some(obj_path) = obj_meshes.get(chunk.entity_path()) else {
            return emit(chunk);
        };
        let has = |descriptor| chunk.component_descriptors().any(|d| *d == descriptor);
        // An albedo factor means the URDF gave the mesh a color, which wins over the file's materials.
        if !has(Asset3D::descriptor_blob()) || has(Asset3D::descriptor_albedo_factor()) {
            return emit(chunk);
        }

        let mesh = match load_obj_with_materials(obj_path, self.color_space) {
            Ok(Some(mesh)) => mesh,
            Ok(None) => return emit(chunk),
            Err(err) => {
                re_log::warn!("{err:#}");
                return emit(chunk);
            }
        };
        match ChunkBuilder::new(ChunkId::new(), chunk.entity_path().clone())
            .with_archetype(RowId::new(), timepoint.clone(), &mesh)
            .build()
        {
            Ok(mesh_chunk) => emit(mesh_chunk),
            Err(err) => {
                re_log::warn!(
                    "Failed to build the colored mesh: {err}\nFile path: {}",
                    obj_path.display()
                );
                emit(chunk);
            }
        }
    }
}

/// The OBJ files of all visual and collision meshes of `tree`, by the entity path the importer logs them to.
fn obj_meshes(tree: &UrdfTree, urdf_dir: Option<&Path>) -> HashMap<EntityPath, PathBuf> {
    let mut meshes = HashMap::new();
    let mut add = |entity_path: EntityPath, geometry: &Geometry| {
        if let Geometry::Mesh { filename, .. } = geometry
            && let Some(path) = resolve_obj(filename, urdf_dir)
        {
            meshes.insert(entity_path, path);
        }
    };

    let mut links = vec![tree.root()];
    while let Some(link) = links.pop() {
        for (entity_path, visual) in tree.get_visual_geometries(link).unwrap_or_default() {
            add(entity_path, &visual.geometry);
        }
        for (entity_path, collision) in tree.get_collision_geometries(link).unwrap_or_default() {
            add(entity_path, &collision.geometry);
        }
        for joint in tree.get_children(&link.name).into_iter().flatten() {
            links.push(tree.get_joint_child(joint));
        }
    }
    meshes
}

/// Resolves a URDF mesh `filename` to a local OBJ file, the same way Rerun's importer finds it.
///
/// `None` for other formats and for remote (`http(s)://`) meshes.
fn resolve_obj(filename: &str, urdf_dir: Option<&Path>) -> Option<PathBuf> {
    let without_query = filename.split(['?', '#']).next().unwrap_or(filename);
    let is_obj = Path::new(without_query)
        .extension()
        .is_some_and(|ext| ext.eq_ignore_ascii_case("obj"));
    if !is_obj {
        return None;
    }

    let path = match filename.split_once("://") {
        Some(("file", path)) => PathBuf::from(path),
        Some(("package", path)) => resolve_package(path, urdf_dir)?,
        Some(_) => return None,
        None => urdf_dir?.join(filename),
    };
    path.is_file().then_some(path)
}

/// `package://<package>/<path>`, looked up in `ROS_PACKAGE_PATH` (ROS 1) and `AMENT_PREFIX_PATH` (ROS 2),
/// falling back to the URDF's own and parent directories.
fn resolve_package(path: &str, urdf_dir: Option<&Path>) -> Option<PathBuf> {
    let (package, rest) = path.split_once('/')?;
    let env_paths = |var: &str| -> Vec<PathBuf> {
        std::env::var_os(var)
            .map(|value| std::env::split_paths(&value).collect())
            .unwrap_or_default()
    };

    let mut candidates = Vec::new();
    for root in env_paths("ROS_PACKAGE_PATH") {
        candidates.push(root.join(package));
    }
    for prefix in env_paths("AMENT_PREFIX_PATH") {
        candidates.push(prefix.join("share").join(package));
    }
    for dir in urdf_dir.into_iter().flat_map(Path::ancestors) {
        if dir.file_name().is_some_and(|name| name == package) {
            candidates.push(dir.to_path_buf());
        }
    }
    candidates
        .into_iter()
        .map(|package_dir| package_dir.join(rest))
        .find(|path| path.is_file())
}
