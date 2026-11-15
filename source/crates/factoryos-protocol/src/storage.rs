//! Storage: the backend's files as a shared file system, in S3 or on the backend's own disk.
//!
//! Two key spaces ([`Scope`]):
//! * [`Scope::User`]: the caller's own subtree (`users/<user-id>/<key>` in S3). The user id is never
//!   part of a request: the backend takes it from its own auth context (the workspace's owner).
//! * [`Scope::Catalogue`]: the shared catalogue (`catalogue/<key>` in S3): everyone reads it, callers
//!   with [`Roles::catalogue_admin`] also write and organise it ([`may_write`]).
//!
//! Keys ([`valid_key`]) are relative `/`-separated paths without `.`/`..`/empty segments, so no
//! request reaches outside its scope, in S3 or on disk.
//!
//! **Where bytes come from.** [`StorageInfo::mode`]: with [`StorageMode::S3`] a viewer asks the
//! backend for short-lived presigned URLs ([`api::STORAGE_PRESIGN`]) and fetches/uploads the bytes
//! from/to S3 itself; with [`StorageMode::Local`] (a backend on this machine, no bucket) it uses the
//! backend's HTTP routes ([`api::STORAGE_OBJECT`]). Listing always goes through the backend.
//!
//! **The edge path.** Behind the workspace edge (CloudFront), a backend may also name
//! [`StorageInfo::edge_path`] (`/_data`): the viewer then reads and writes the bucket through the
//! edge at its own origin, with its session cookie, without presigning ([`edge`]). Only for the
//! workspace's owner, since the edge maps every path into the signed-in user's own subtree.

use serde::{Deserialize, Serialize};

/// HTTP routes (also in [`crate::api`]'s namespace by name).
pub mod api {
    /// `GET` the [`super::StorageInfo`].
    pub const STORAGE_INFO: &str = "/api/v1/storage";
    /// `POST` a [`super::PresignRequest`]: `200` [`super::PresignResponse`]; `409` in local mode.
    pub const STORAGE_PRESIGN: &str = "/api/v1/storage/presign";
    /// `GET` an object's bytes, `PUT` them, `DELETE` it: `{scope}` is `user` or `catalogue`
    /// (`catalogue` is `GET` only).
    pub const STORAGE_OBJECT: &str = "/api/v1/storage/object/{scope}/{*key}";
    /// `GET ?prefix=` a [`super::ObjectList`] of `{scope}`.
    pub const STORAGE_LIST: &str = "/api/v1/storage/list/{scope}";
    /// `POST` a [`super::Commit`]: after a viewer's direct upload to S3, the backend takes the object
    /// into its working copy (its tools work on local files).
    pub const STORAGE_COMMIT: &str = "/api/v1/storage/commit";
    /// `POST` a [`super::Folder`]: a new (empty) folder, as its [`super::FOLDER_MARKER`] object.
    pub const STORAGE_FOLDER: &str = "/api/v1/storage/folder";
    /// `POST` a [`super::Move`]: rename/move a file or a folder (copy, then delete).
    pub const STORAGE_MOVE: &str = "/api/v1/storage/move";
    /// `POST` a [`super::Remove`]: delete a file or a folder.
    pub const STORAGE_REMOVE: &str = "/api/v1/storage/remove";
    /// `POST` a [`super::Publish`]: copy a file or folder of the user's subtree into the shared
    /// catalogue (catalogue admins only): `200` [`super::Published`].
    pub const STORAGE_PUBLISH: &str = "/api/v1/storage/publish";
    /// `POST` an [`super::EnsureAssets`] (S3 mode): the backend uploads the registered assets not in
    /// S3 yet (`users/<user>/assets/<sha256>`), so the edge path ([`super::StorageInfo::edge_path`])
    /// serves them: `200` [`super::EnsuredAssets`]; `409` in local mode.
    pub const STORAGE_ASSETS_ENSURE: &str = "/api/v1/storage/assets/ensure";

    pub fn object(scope: super::Scope, key: &str) -> String {
        format!("/api/v1/storage/object/{}/{key}", scope.as_str())
    }
    pub fn list(scope: super::Scope, prefix: &str) -> String {
        format!("/api/v1/storage/list/{}?prefix={}", scope.as_str(), encode_query(prefix))
    }

    fn encode_query(value: &str) -> String {
        let mut out = String::with_capacity(value.len());
        for b in value.bytes() {
            if b.is_ascii_alphanumeric() || matches!(b, b'-' | b'_' | b'.' | b'~' | b'/') {
                out.push(b as char);
            } else {
                out.push_str(&format!("%{b:02X}"));
            }
        }
        out
    }
}

/// Longest key accepted (S3's limit is 1024 bytes for the whole object key, prefixes included).
pub const MAX_KEY_BYTES: usize = 800;
/// Most items in one [`PresignRequest`].
pub const MAX_PRESIGN_ITEMS: usize = 64;
/// Most hashes in one [`EnsureAssets`].
pub const MAX_ENSURE_ITEMS: usize = 64;
/// Key prefixes no viewer may write, in either scope: the backend's own (content-addressed assets,
/// written only by the backend from verified bytes).
pub const BACKEND_ONLY_PREFIXES: &[&str] = &["assets/", REVISION_KEY];
/// The catalogue folder only the cell-templates CI publishes (signed virtual-cell templates,
/// docs/tasks/cell-templates.md §15): no caller writes it, in either scope (in the user scope it is
/// `catalogue/cell-templates`, the overlay that would shadow it), catalogue admins included.
pub const PLATFORM_ONLY_CATALOGUE_PREFIX: &str = "cell-templates";
/// The shared catalogue's change log ([`CatalogueRevision`]), at `catalogue/.revision`: written by
/// the backends that change the catalogue, polled by every other one.
pub const REVISION_KEY: &str = ".revision";
/// The object that stands for an empty folder (S3 has no folders).
pub const FOLDER_MARKER: &str = ".keep";
/// Most objects one move, remove or publish touches.
pub const MAX_BATCH_OBJECTS: usize = 5_000;
/// Where content-addressed assets live in [`Scope::User`]: `assets/<sha256>`.
pub fn asset_key(sha256: &str) -> String {
    format!("assets/{sha256}")
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Scope {
    /// The caller's own subtree.
    User,
    /// The shared catalogue (writable by catalogue admins only).
    Catalogue,
}

impl Scope {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::User => "user",
            Self::Catalogue => "catalogue",
        }
    }
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "user" => Some(Self::User),
            "catalogue" => Some(Self::Catalogue),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum StorageMode {
    /// The backend's own disk; bytes over its HTTP routes.
    #[default]
    Local,
    /// S3; bytes over presigned URLs.
    S3,
}

/// What the caller may do beyond their own subtree (resolved by the backend, per request; shown by
/// the viewer, enforced by the backend).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Roles {
    /// May write and organise the shared catalogue.
    #[serde(default)]
    pub catalogue_admin: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct StorageInfo {
    pub mode: StorageMode,
    /// The user whose subtree [`Scope::User`] is (informational; never sent back).
    #[serde(default)]
    pub user: Option<String>,
    /// How long a presigned URL is good for, in seconds.
    #[serde(default)]
    pub presign_ttl_secs: u64,
    /// The caller's.
    #[serde(default)]
    pub roles: Roles,
    /// S3 mode behind the workspace edge, for the workspace's owner: the path at the viewer's origin
    /// (`/_data`) through which the edge reads and writes the bucket ([`edge`]). Preferred over
    /// presigned URLs, which stay the fallback (and keep [`Self::mode`] [`StorageMode::S3`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub edge_path: Option<String>,
}

/// The edge path's layout ([`StorageInfo::edge_path`]), relative to it. The edge maps each to the
/// signed-in user's S3 keys, never to another user's:
/// * `user/<key>` → `users/<alias>/<key>` (`GET`, `HEAD`, `PUT`);
/// * `asset/<sha256>` → `users/<alias>/assets/<sha256>` (`GET`, `HEAD`; uploaded by the backend,
///   [`api::STORAGE_ASSETS_ENSURE`]);
/// * `catalogue/<key>` → `catalogue/<key>` (`GET`, `HEAD`; `PUT` for catalogue admins).
///
/// A missing object is `403` or `404` (the edge may not list the bucket); no session is `401`.
pub mod edge {
    use super::Scope;

    /// `<edge_path>/user|catalogue/<key>`, each segment of `key` percent-encoded.
    pub fn object(edge_path: &str, scope: Scope, key: &str) -> String {
        let key: Vec<String> = key.split('/').map(encode_segment).collect();
        format!("{}/{}/{}", edge_path.trim_end_matches('/'), scope.as_str(), key.join("/"))
    }

    /// `<edge_path>/asset/<sha256>`.
    pub fn asset(edge_path: &str, sha256: &str) -> String {
        format!("{}/asset/{}", edge_path.trim_end_matches('/'), encode_segment(sha256))
    }

    /// Everything but RFC 3986's unreserved characters, as `%XX`.
    pub fn encode_segment(segment: &str) -> String {
        let mut out = String::with_capacity(segment.len());
        for b in segment.bytes() {
            if b.is_ascii_alphanumeric() || matches!(b, b'-' | b'_' | b'.' | b'~') {
                out.push(b as char);
            } else {
                out.push_str(&format!("%{b:02X}"));
            }
        }
        out
    }
}

/// Assets for the backend to put into S3 ([`api::STORAGE_ASSETS_ENSURE`]): registered ones only.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct EnsureAssets {
    /// SHA-256s (lowercase hex), at most [`MAX_ENSURE_ITEMS`].
    pub sha256s: Vec<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct EnsuredAssets {
    /// One per requested hash, in order.
    pub items: Vec<EnsuredAsset>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EnsuredAsset {
    pub sha256: String,
    /// In S3 now (or already).
    pub ok: bool,
    /// Why not (e.g. not registered).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum Method {
    Get,
    Put,
}

/// One URL to presign.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum PresignItem {
    /// An object of `scope` at `key`.
    Object { scope: Scope, key: String, method: Method },
    /// A registered content-addressed asset (`GET` only), by SHA-256 (lowercase hex).
    Asset { sha256: String },
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PresignRequest {
    pub items: Vec<PresignItem>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Presigned {
    /// `None` with `error` set when this item was refused.
    pub url: Option<String>,
    /// Headers the request must carry as signed (e.g. none today; kept for content types).
    #[serde(default)]
    pub headers: Vec<(String, String)>,
    /// Unix seconds after which the URL no longer works.
    pub expires_at: u64,
    #[serde(default)]
    pub error: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PresignResponse {
    /// One per request item, in order.
    pub items: Vec<Presigned>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObjectInfo {
    /// Relative to the scope.
    pub key: String,
    pub bytes: u64,
    /// The store's version tag of the object (S3's ETag), when it has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub etag: Option<String>,
}

/// The shared catalogue's change log ([`REVISION_KEY`]): a revision bumped by every change, and the
/// last changes (a backend further behind than they reach lists the whole catalogue instead).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CatalogueRevision {
    pub revision: u64,
    #[serde(default)]
    pub changes: Vec<CatalogueChange>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CatalogueChange {
    /// The revision this change made.
    pub rev: u64,
    pub key: String,
    #[serde(default)]
    pub deleted: bool,
}

/// On [`crate::keys::CATALOGUE_CHANGED`]: the shared catalogue changed (here, or in another
/// workspace and synced here); viewers list it again.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CatalogueChanged {
    /// The catalogue's revision now (`0` in local mode, which keeps none).
    pub revision: u64,
    /// The keys that changed (may be cut short; empty: anything may have).
    #[serde(default)]
    pub keys: Vec<String>,
    /// The change came from another workspace (else this one made it).
    #[serde(default)]
    pub remote: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObjectList {
    pub objects: Vec<ObjectInfo>,
    #[serde(default)]
    pub truncated: bool,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Commit {
    /// Of [`Scope::User`] unless `scope` says otherwise.
    pub key: String,
    #[serde(default = "user_scope")]
    pub scope: Scope,
}

fn user_scope() -> Scope {
    Scope::User
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Folder {
    pub scope: Scope,
    /// The folder's key, without a trailing `/`.
    pub key: String,
}

/// `from` and `to` both files, or both folders (ending in `/`).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Move {
    pub scope: Scope,
    pub from: String,
    pub to: String,
    /// Replace what is at `to` (else `409` if anything is).
    #[serde(default)]
    pub overwrite: bool,
}

/// A file, or a folder (ending in `/`).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Remove {
    pub scope: Scope,
    pub key: String,
}

/// From the user's subtree (`from`, a file or a folder ending in `/`) to the shared catalogue (`to`,
/// of the same kind).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Publish {
    pub from: String,
    pub to: String,
    #[serde(default)]
    pub overwrite: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Published {
    pub objects: usize,
}

/// A file key or a folder key (`a/b/`): the key without its `/`, and whether it was a folder.
pub fn split_folder(key: &str) -> Option<(&str, bool)> {
    match key.strip_suffix('/') {
        Some(folder) => valid_key(folder).then_some((folder, true)),
        None => valid_key(key).then_some((key, false)),
    }
}

/// Whether `key` is a valid relative key: `/`-separated, each segment non-empty, not `.`/`..`, no
/// backslashes, control characters or leading/trailing `/`, at most [`MAX_KEY_BYTES`].
pub fn valid_key(key: &str) -> bool {
    !key.is_empty()
        && key.len() <= MAX_KEY_BYTES
        && !key.bytes().any(|b| b < 0x20 || b == 0x7f || b == b'\\')
        && key.split('/').all(|seg| !seg.is_empty() && seg != "." && seg != "..")
}

/// Whether `prefix` is a valid listing prefix: empty, or a valid key optionally ending in `/`.
pub fn valid_prefix(prefix: &str) -> bool {
    prefix.is_empty() || valid_key(prefix.strip_suffix('/').unwrap_or(prefix))
}

/// The folder of `scope` only the cell-templates CI writes ([`PLATFORM_ONLY_CATALOGUE_PREFIX`]).
pub fn platform_only_folder(scope: Scope) -> String {
    match scope {
        Scope::Catalogue => PLATFORM_ONLY_CATALOGUE_PREFIX.to_owned(),
        Scope::User => format!("catalogue/{PLATFORM_ONLY_CATALOGUE_PREFIX}"),
    }
}

/// Whether `key` (a file or folder key, without its `/`) is in [`platform_only_folder`] of `scope`.
/// The prefix is compared ASCII case folded: the local store may be on a case-insensitive file
/// system (APFS), where `Cell-Templates/` is `cell-templates/`.
pub fn is_platform_only(scope: Scope, key: &str) -> bool {
    strip_prefix_folded(key, &platform_only_folder(scope)).is_some_and(|rest| rest.is_empty() || rest.starts_with('/'))
}

/// Whether removing or moving folder `key` of `scope` would take [`platform_only_folder`] with it
/// (the user scope's `catalogue`): the batch is refused when that folder has objects.
pub fn contains_platform_only(scope: Scope, folder: &str) -> bool {
    strip_prefix_folded(&platform_only_folder(scope), folder).is_some_and(|rest| rest.starts_with('/'))
}

/// `s` without its prefix `prefix`, compared ASCII case folded.
fn strip_prefix_folded<'a>(s: &'a str, prefix: &str) -> Option<&'a str> {
    let head = s.get(..prefix.len())?;
    head.eq_ignore_ascii_case(prefix).then(|| &s[prefix.len()..])
}

/// Whether a caller with `roles` may write `key` (a file or folder key) of `scope`: their own subtree,
/// and the shared catalogue if a catalogue admin; never the backend's own prefixes, nor the
/// cell-templates CI's ([`is_platform_only`]).
pub fn may_write(roles: Roles, scope: Scope, key: &str) -> bool {
    let Some((key, _)) = split_folder(key) else { return false };
    let backend_only = BACKEND_ONLY_PREFIXES.iter().any(|p| key.starts_with(p) || format!("{key}/") == *p);
    !backend_only
        && !is_platform_only(scope, key)
        && match scope {
            Scope::User => true,
            Scope::Catalogue => roles.catalogue_admin,
        }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keys_stay_inside_their_scope() {
        for good in ["a", "products/box/product.json", "a.b/c-d_e", "x/..y/z"] {
            assert!(valid_key(good), "{good}");
        }
        for bad in ["", "/a", "a/", "a//b", "..", "a/../b", "./a", "a\\b", "a\nb", "a/./b", &"x".repeat(MAX_KEY_BYTES + 1)] {
            assert!(!valid_key(bad), "{bad:?}");
        }
        assert!(valid_prefix("") && valid_prefix("products/") && !valid_prefix("../") && !valid_prefix("/"));
        let (user, admin) = (Roles::default(), Roles { catalogue_admin: true });
        assert!(may_write(user, Scope::User, "products/a/product.json"));
        assert!(!may_write(user, Scope::User, "assets/abc") && !may_write(admin, Scope::User, "assets/abc"));
        assert!(!may_write(user, Scope::Catalogue, "products/a"));
        assert!(may_write(admin, Scope::Catalogue, "products/a") && may_write(admin, Scope::Catalogue, "products/"));
        assert!(!may_write(admin, Scope::Catalogue, "assets/x") && !may_write(admin, Scope::Catalogue, "assets/"));
        assert!(!may_write(admin, Scope::Catalogue, "../x") && !may_write(admin, Scope::Catalogue, "a//"));
        // The cell-templates CI's folder: nobody, catalogue admins included, in either scope.
        for (scope, key) in [
            (Scope::Catalogue, "cell-templates"),
            (Scope::Catalogue, "cell-templates/"),
            (Scope::Catalogue, "cell-templates/index.json"),
            (Scope::Catalogue, "cell-templates/ec2-codesys-kuka/1.0.0/template.json"),
            (Scope::User, "catalogue/cell-templates"),
            (Scope::User, "catalogue/cell-templates/"),
            (Scope::User, "catalogue/cell-templates/index.json.sig"),
            (Scope::Catalogue, "Cell-Templates/index.json"),
            (Scope::Catalogue, "CELL-TEMPLATES/"),
            (Scope::User, "catalogue/CELL-TEMPLATES/index.json"),
            (Scope::User, "CATALOGUE/cell-templates/index.json"),
        ] {
            assert!(!may_write(admin, scope, key) && !may_write(user, scope, key), "{scope:?} {key}");
        }
        assert!(may_write(admin, Scope::Catalogue, "cell-templates-old/x") && may_write(admin, Scope::Catalogue, "products/cell-templates/x"));
        assert!(may_write(user, Scope::User, "cell-templates/x") && may_write(user, Scope::User, "catalogue/cell-templatesx"));
        assert!(may_write(user, Scope::User, "catalogue/products/a"));
        assert!(contains_platform_only(Scope::User, "catalogue") && !contains_platform_only(Scope::User, "catalogue/products"));
        assert!(!contains_platform_only(Scope::User, "catalog") && !contains_platform_only(Scope::Catalogue, "products"));
        assert!(!contains_platform_only(Scope::Catalogue, "cell-templates"), "the folder itself: is_platform_only");
        assert!(contains_platform_only(Scope::User, "Catalogue") && !contains_platform_only(Scope::User, "catalogue/cell-templates"));
        assert_eq!(split_folder("a/b/"), Some(("a/b", true)));
        assert_eq!(split_folder("a/b"), Some(("a/b", false)));
        assert_eq!(split_folder("/"), None);
    }

    #[test]
    fn presign_items_round_trip() {
        let request = PresignRequest {
            items: vec![
                PresignItem::Asset { sha256: "ab".into() },
                PresignItem::Object { scope: Scope::User, key: "k".into(), method: Method::Put },
            ],
        };
        let json = serde_json::to_string(&request).unwrap();
        assert!(json.contains(r#""kind":"asset""#) && json.contains(r#""method":"PUT""#), "{json}");
        assert_eq!(serde_json::from_str::<PresignRequest>(&json).unwrap(), request);
    }

    #[test]
    fn edge_paths_encode_each_segment() {
        assert_eq!(edge::object("/_data", Scope::User, "products/my box/a#1.json"), "/_data/user/products/my%20box/a%231.json");
        assert_eq!(edge::object("/_data/", Scope::Catalogue, "robots/ä.urdf"), "/_data/catalogue/robots/%C3%A4.urdf");
        assert_eq!(edge::asset("/_data", &"a".repeat(64)), format!("/_data/asset/{}", "a".repeat(64)));
        // An older backend's info (no edge path) still reads; none is not sent.
        let info: StorageInfo = serde_json::from_str(r#"{"mode":"s3","presign_ttl_secs":300}"#).unwrap();
        assert_eq!(info.edge_path, None);
        assert!(!serde_json::to_string(&info).unwrap().contains("edge_path"));
    }
}
