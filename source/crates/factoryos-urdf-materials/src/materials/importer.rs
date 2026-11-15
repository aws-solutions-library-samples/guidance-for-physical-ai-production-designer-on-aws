//! A URDF [`Importer`] that replaces Rerun's own, for files opened in the viewer or logged with
//! `RecordingStream::log_file_from_path`.

use std::path::{Path, PathBuf};

use anyhow::Context as _;
use crossbeam::channel::Sender;
use re_chunk::Chunk;
use re_importer::{
    ImportedData, Importer, ImporterError, ImporterName, ImporterSettings, UrdfImporter, UrdfTree,
};

use super::UrdfMaterials;

/// Rerun's URDF importer plus the `.mtl` colors of OBJ meshes, see [`UrdfMaterials`].
///
/// It has the same name as Rerun's importer, so [`register`] can replace that one.
#[derive(Clone, Copy, Debug, Default)]
pub struct UrdfMaterialsImporter(pub UrdfMaterials);

/// Replaces Rerun's built-in URDF importer with [`UrdfMaterialsImporter`], in this process.
///
/// Call it once at startup, in every process that imports URDF files:
/// the viewer (for files it opens) and processes calling `log_file_from_path`.
pub fn register() {
    re_importer::override_builtin_importer(UrdfMaterialsImporter::default());
}

impl Importer for UrdfMaterialsImporter {
    fn name(&self) -> ImporterName {
        UrdfImporter.name()
    }

    fn import_from_path(
        &self,
        settings: &ImporterSettings,
        filepath: PathBuf,
        tx: Sender<ImportedData>,
    ) -> Result<(), ImporterError> {
        if !is_urdf_file(&filepath) {
            return Err(ImporterError::Incompatible(filepath));
        }
        let robot = re_importer::external::urdf_rs::read_file(&filepath)
            .with_context(|| format!("Path: {}", filepath.display()))?;
        self.import(settings, robot, filepath.parent(), &tx)
    }

    fn import_from_file_contents(
        &self,
        settings: &ImporterSettings,
        filepath: PathBuf,
        contents: std::borrow::Cow<'_, [u8]>,
        tx: Sender<ImportedData>,
    ) -> Result<(), ImporterError> {
        if !is_urdf_file(&filepath) {
            return Err(ImporterError::Incompatible(filepath));
        }
        let robot = re_importer::external::urdf_rs::read_from_string(&String::from_utf8_lossy(&contents))
            .with_context(|| format!("Path: {}", filepath.display()))?;
        self.import(settings, robot, filepath.parent(), &tx)
    }
}

impl UrdfMaterialsImporter {
    fn import(
        &self,
        settings: &ImporterSettings,
        robot: re_importer::external::urdf_rs::Robot,
        urdf_dir: Option<&Path>,
        tx: &Sender<ImportedData>,
    ) -> Result<(), ImporterError> {
        let tree = UrdfTree::new(
            robot,
            urdf_dir.map(Path::to_path_buf),
            settings.entity_path_prefix.clone(),
        )
        .context("Failed to build URDF tree!")?;

        let name = self.name();
        let store_id = settings.opened_store_id_or_recommended();
        let mut send_error = None;
        let mut emit = |chunk: Chunk| {
            if send_error.is_none() {
                send_error = re_quota_channel::send_crossbeam(
                    tx,
                    ImportedData::Chunk(name.clone(), store_id.clone(), chunk),
                )
                .err();
            }
        };

        self.0
            .emit(
                &tree,
                urdf_dir,
                &mut emit,
                &settings.timepoint.clone().unwrap_or_default(),
                true,
            )
            .context("Failed to load URDF file!")?;

        if let Some(err) = send_error {
            return Err(anyhow::anyhow!(err.to_string()).into());
        }
        Ok(())
    }
}

fn is_urdf_file(path: &Path) -> bool {
    path.extension()
        .is_some_and(|ext| ext.eq_ignore_ascii_case("urdf"))
}
