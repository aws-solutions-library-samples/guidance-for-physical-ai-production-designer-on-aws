//! The colors of URDF OBJ meshes in Rerun (formerly a separate crate, docs/UPSTREAM.md).
//!
//! Rerun ignores the `mtllib` of OBJ files, so every OBJ mesh without a URDF `<color>` shows up
//! white. This crate reads the `.mtl` materials and makes a `Mesh3D` with the material colors as
//! vertex colors instead.
//!
//! - [`obj_to_mesh3d`]: an OBJ file's bytes → `Mesh3D`, with its `mtllib` files fetched through a
//!   callback (by their name in the OBJ). No filesystem, so it also builds for `wasm32`; the
//!   viewer uses it for meshes it fetches over HTTP.
//!
//! Native only, on top of that, with the files read from disk:
//!
//! - [`load_obj_with_materials`] for a single OBJ file.
//! - [`emit`] instead of `UrdfTree::emit`: Rerun's URDF importer, with its white OBJ meshes
//!   replaced by colored `Mesh3D`s on the same entity paths, so nothing else changes.
//! - [`register`] replaces Rerun's URDF importer (viewer file open, `log_file_from_path`). Needs
//!   `third_party/rerun/patches/rerun-override-builtin-importer.patch` (applied by
//!   `scripts/fetch-rerun.sh`).

mod materials {
    #[cfg(not(target_arch = "wasm32"))]
    pub mod importer;
    pub mod obj;
    #[cfg(not(target_arch = "wasm32"))]
    pub mod urdf;

    #[cfg(not(target_arch = "wasm32"))]
    pub use urdf::UrdfMaterials;
}

pub use materials::obj::{ColorSpace, obj_to_mesh3d};

#[cfg(not(target_arch = "wasm32"))]
pub use native::*;

#[cfg(not(target_arch = "wasm32"))]
mod native {
    use std::path::Path;

    use re_chunk::{Chunk, TimePoint};
    use re_importer::UrdfTree;

    pub use crate::materials::importer::UrdfMaterialsImporter;
    pub use crate::materials::obj::load_obj_with_materials;
    pub use crate::materials::urdf::UrdfMaterials;

    /// Replaces Rerun's URDF importer (files opened in the viewer, `log_file_from_path`).
    /// Call once at startup, in every process that imports URDF files.
    pub fn register() {
        crate::materials::importer::register();
    }

    /// Like [`UrdfTree::emit`], with the meshes' `.mtl` colors; `urdf_dir` is the `.urdf` file's
    /// directory.
    pub fn emit(
        tree: &UrdfTree,
        urdf_dir: Option<&Path>,
        emit: &mut dyn FnMut(Chunk),
        timepoint: &TimePoint,
        include_joint_transforms: bool,
    ) -> anyhow::Result<()> {
        UrdfMaterials::default().emit(tree, urdf_dir, emit, timepoint, include_joint_transforms)
    }
}
