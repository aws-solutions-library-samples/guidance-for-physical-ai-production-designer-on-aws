//! Workspace IDE: browse and edit the text files under the roots the backend allows
//! ([`crate::api::FILES_ROOTS`]). Only the workspace's owner (or a caller on loopback) may use them.

use serde::{Deserialize, Serialize};

/// Downloads and uploads ([`crate::api::FILES_RAW`]) up to this size.
pub const MAX_TRANSFER_BYTES: u64 = 256 * 1024 * 1024;

/// A folder the IDE may open; the backend's allowlist, not the viewer's choice.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FileRoot {
    pub id: String,
    pub label: String,
    pub writable: bool,
    /// Its folder on the backend's host (for Copy Path).
    #[serde(default)]
    pub path: String,
    /// Too big to list at once (`/`, `~`): its folders are listed one at a time ([`crate::api::files_tree_dir`]).
    #[serde(default)]
    pub lazy: bool,
    /// Its files are a storage scope's (e.g. the shared catalogue), listed and transferred through
    /// the storage routes ([`crate::storage::api`]) and a viewer's `DataSource`, not the files routes.
    #[serde(default)]
    pub storage: Option<crate::storage::Scope>,
}

/// A file or folder of a root, `/`-separated and relative to it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FileEntry {
    pub path: String,
    #[serde(default)]
    pub bytes: u64,
    #[serde(default)]
    pub dir: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FileTree {
    pub entries: Vec<FileEntry>,
    /// The listing stopped at the backend's limit.
    #[serde(default)]
    pub truncated: bool,
}

/// A file's text and its version on disk (to detect changes made meanwhile).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FileText {
    pub text: String,
    pub version: String,
}

/// `PUT`: the new text, saved only if the file is still at `base_version` (else `409`).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FileSave {
    pub text: String,
    /// `None`: overwrite whatever is on disk.
    #[serde(default)]
    pub base_version: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FileSaved {
    pub version: String,
}
