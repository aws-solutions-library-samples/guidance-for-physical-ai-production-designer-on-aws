//! Runs against the sample Factory OS assets (the repo's `assets/catalogue`), if present (read-only).
//! `FACTORY_OS_ASSETS` overrides where they are.

use std::path::{Path, PathBuf};

use re_chunk::{Chunk, TimePoint};
use re_importer::{Importer as _, UrdfImporter, UrdfTree};
use re_sdk_types::archetypes::{Asset3D, Mesh3D};
use factoryos_urdf_materials::UrdfMaterials;

fn assets() -> Option<PathBuf> {
    // The repo's sample catalogue (assets/catalogue), unless FACTORY_OS_ASSETS points elsewhere.
    let dir = std::env::var_os("FACTORY_OS_ASSETS").map_or_else(
        || Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/catalogue"),
        PathBuf::from,
    );
    if dir.is_dir() {
        Some(dir)
    } else {
        eprintln!("Skipping: no assets at {}", dir.display());
        None
    }
}

/// (`Mesh3D` chunks, `Asset3D` chunks) emitted for `urdf`.
fn emit(urdf: &Path) -> (Vec<Chunk>, usize) {
    let tree = UrdfTree::from_file_path(urdf, None).unwrap();
    let mut chunks = Vec::new();
    UrdfMaterials::default()
        .emit(
            &tree,
            urdf.parent(),
            &mut |chunk| chunks.push(chunk),
            &TimePoint::STATIC,
            true,
        )
        .unwrap();
    let has = |chunk: &Chunk, descriptor| chunk.component_descriptors().any(|d| *d == descriptor);
    let assets = chunks
        .iter()
        .filter(|c| has(c, Asset3D::descriptor_blob()))
        .count();
    let meshes = chunks
        .into_iter()
        .filter(|c| has(c, Mesh3D::descriptor_vertex_colors()))
        .collect();
    (meshes, assets)
}

fn vertex_colors(chunk: &Chunk) -> Vec<u32> {
    let colors = chunk
        .components()
        .get_array(Mesh3D::descriptor_vertex_colors().component)
        .unwrap();
    colors
        .values()
        .as_any()
        .downcast_ref::<re_sdk_types::external::arrow::array::UInt32Array>()
        .unwrap()
        .values()
        .to_vec()
}

#[test]
fn robot_meshes_get_their_mtl_colors() {
    let Some(assets) = assets() else { return };
    let (meshes, assets_left) =
        emit(&assets.join("urdf-robot-collection/kr240r2900/kr240r2900.urdf"));

    // 7 links, each with a visual and a collision OBJ that have an .mtl.
    assert_eq!((meshes.len(), assets_left), (14, 0));

    let link_1 = meshes
        .iter()
        .find(|c| {
            c.entity_path()
                .to_string()
                .ends_with("like_kr240r2900_link_1/visual_0")
        })
        .expect("link 1 visual");
    let colors = vertex_colors(link_1);
    // RAL9005 (black) and RAL2003 (orange) from like_kr240r2900_link_1_visuals_link_1_Scene_mesh.mtl.
    assert!(colors.contains(&0x010101FF), "{:08X?}", &colors[..4]);
    assert!(colors.contains(&0x833512FF));
}

#[test]
fn tool_meshes_get_their_mtl_colors() {
    let Some(assets) = assets() else { return };
    let (meshes, assets_left) = emit(&assets.join("urdf-tool-collection/sticktool/sticktool.urdf"));
    assert_eq!((meshes.len(), assets_left), (10, 0));
    // Rosewood (0.3725 0 0) from the tool's .mtl files.
    assert!(
        meshes
            .iter()
            .any(|m| vertex_colors(m).contains(&0x5F0000FF))
    );
}

#[test]
fn urdf_colors_still_win() {
    let Some(assets) = assets() else { return };
    // Its OBJ has no .mtl but the visual has a URDF <color>: left to Rerun.
    let (meshes, assets_left) =
        emit(&assets.join("urdf-fixture-collection/weld_cube/weld_cube.urdf"));
    assert_eq!((meshes.len(), assets_left), (0, 2));
}

#[test]
fn register_replaces_the_builtin_urdf_importer() {
    factoryos_urdf_materials::register();
    let urdf_importers = re_importer::iter_importers()
        .filter(|importer| importer.name() == UrdfImporter.name())
        .count();
    assert_eq!(urdf_importers, 1);
}
