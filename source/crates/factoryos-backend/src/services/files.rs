//! Workspace IDE ([`factoryos_protocol::files`]): the text files under the roots this backend allows, to
//! browse and edit in the viewer.
//!
//! * Roots: `cell` (the active cell's folder), `cell-<name>` (every other cell of the cells folder),
//!   `home` (`~`, this backend's user's) and `fs` (`/`), and those named by `--ide-root label=path`
//!   (`FACTORY_OS_IDE_ROOTS`, `;`-separated). Not in the config: the viewer may write that. `~` and `/`
//!   reach no more than the IDE's terminals (the same user) do; they are listed a folder at a time.
//! * Who: the workspace's owner, as the edge names them (`X-User-Alias` = `--workspace`); without the
//!   header, only a caller on loopback that asks for a loopback host (a native viewer, an SSM tunnel;
//!   not a web page that rebinds a name of its own to 127.0.0.1). Merged outside the CORS layer,
//!   so another site's page can't save either.
//! * The shared catalogue: listed as one more root, [`SHARED_CATALOGUE_ROOT`] (`storage: catalogue`),
//!   whose files go through the storage routes (`crate::storage::http`, S3 or the local catalogue
//!   folder), never these: browsing for everyone admitted, writing for catalogue admins
//!   ([`crate::authz`]; `writable` says which).
//! * What: existing UTF-8 files up to [`MAX_TEXT_BYTES`]; saved through a temporary file, and only if
//!   unchanged on disk since they were opened (else `409`). Downloads and uploads ([`api::FILES_RAW`])
//!   of any file up to [`MAX_TRANSFER_BYTES`]: an upload into an existing folder, a new file unless
//!   overwriting is asked for; never through a symlink. As for the terminals, no other site's `Origin`.
//! * Saves and uploads of a file in the user's folders (a cell's, the products', the catalogue's,
//!   whichever root reaches it) are written through to their store (S3 mode, [`crate::storage::saved`]).
//!   These routes neither rename nor delete; the storage routes do, in the store and the working copy.

use std::net::SocketAddr;
use std::path::{Component, Path, PathBuf};
use std::sync::{Arc, OnceLock};

use std::collections::HashMap;

use axum::body::{Body, Bytes};
use axum::extract::{ConnectInfo, DefaultBodyLimit, Path as UrlPath, Query, State};
use axum::http::{HeaderMap, StatusCode, header};
use axum::response::{IntoResponse as _, Response};
use axum::routing::get;
use axum::{Json, Router};
use factoryos_protocol::api;
use factoryos_protocol::files::{FileEntry, FileRoot, FileSave, FileSaved, FileText, FileTree};

use crate::config_store::ConfigStore;
use crate::services::cells;

type HttpError = (StatusCode, String);

/// Larger files are not opened (an editor buffer, not a log viewer).
pub const MAX_TEXT_BYTES: u64 = 512 * 1024;
pub use factoryos_protocol::files::MAX_TRANSFER_BYTES;
/// A tree stops listing here (`truncated`).
const MAX_ENTRIES: usize = 20_000;
/// Folders not listed: build output, dependencies, VCS.
const SKIPPED_DIRS: &[&str] = &[".git", "target", "node_modules", "__pycache__", ".venv", "venv", "dist", "build"];
const CELL_ROOT: &str = "cell";
const CELL_PREFIX: &str = "cell-";
const HOME_ROOT: &str = "home";
const FS_ROOT: &str = "fs";
/// The shared catalogue's root id (served by the storage routes, not these).
pub const SHARED_CATALOGUE_ROOT: &str = "shared-catalogue";

pub struct FilesService {
    config: Arc<ConfigStore>,
    /// `--ide-root`s: (id, label, folder).
    extra: Vec<(String, String, PathBuf)>,
    /// `--workspace`: the one `X-User-Alias` that may use the files.
    owner: Option<String>,
    /// The roles of a caller (main.rs); unset: no roles (fail closed).
    authz: OnceLock<Arc<dyn crate::authz::Authz>>,
}

impl FilesService {
    /// `roots`: `label=path` each (a bare path: its folder name is the label).
    pub fn new(config: Arc<ConfigStore>, roots: &[String], owner: Option<String>) -> anyhow::Result<Arc<Self>> {
        let mut extra: Vec<(String, String, PathBuf)> = Vec::new();
        for spec in roots.iter().map(|s| s.trim()).filter(|s| !s.is_empty()) {
            let (label, path) = match spec.split_once('=') {
                Some((label, path)) => (label.trim().to_owned(), PathBuf::from(path.trim())),
                None => {
                    let path = PathBuf::from(spec);
                    (path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_else(|| spec.to_owned()), path)
                }
            };
            anyhow::ensure!(path.is_absolute(), "--ide-root {spec}: the path must be absolute");
            let id = slug(&label);
            anyhow::ensure!(
                !id.is_empty() && ![CELL_ROOT, HOME_ROOT, FS_ROOT, SHARED_CATALOGUE_ROOT].contains(&id.as_str()) && !id.starts_with(CELL_PREFIX),
                "--ide-root {spec}: pick another label"
            );
            anyhow::ensure!(extra.iter().all(|(other, ..)| *other != id), "--ide-root {spec}: label used twice");
            extra.push((id, label, path));
        }
        Ok(Arc::new(Self { config, extra, owner, authz: OnceLock::new() }))
    }

    pub fn set_authz(&self, authz: Arc<dyn crate::authz::Authz>) {
        self.authz.set(authz).ok();
    }

    /// An admitted caller's roles.
    pub fn roles(&self, peer: SocketAddr, headers: &HeaderMap) -> crate::authz::Roles {
        self.authz.get().map(|authz| authz.roles(&crate::authz::Caller::of(peer, headers))).unwrap_or_default()
    }

    /// [`Self::roots`], then the shared catalogue (writable for catalogue admins).
    pub fn roots_for(&self, roles: crate::authz::Roles) -> Vec<FileRoot> {
        let mut roots = self.roots();
        let catalogue = self.config.resolve(&self.config.get().catalogue);
        roots.push(FileRoot {
            id: SHARED_CATALOGUE_ROOT.to_owned(),
            label: "Shared catalogue".to_owned(),
            writable: roles.catalogue_admin,
            path: if catalogue.trim().is_empty() { crate::paths::default_catalogue() } else { catalogue },
            lazy: false,
            storage: Some(factoryos_protocol::storage::Scope::Catalogue),
        });
        roots
    }

    /// Whether this request may use the files (see the module docs).
    pub fn admit(&self, peer: SocketAddr, headers: &HeaderMap) -> Result<(), HttpError> {
        let forbidden = |why: &str| Err((StatusCode::FORBIDDEN, format!("workspace files: {why}")));
        let alias = headers.get("x-user-alias").and_then(|v| v.to_str().ok()).map(str::trim).filter(|a| !a.is_empty());
        match (alias, &self.owner) {
            (Some(alias), Some(owner)) if alias == owner => Ok(()),
            (Some(alias), Some(_)) => forbidden(&format!("only the workspace's owner may edit them, not {alias}")),
            (Some(_), None) => forbidden("this backend has no owner (--workspace)"),
            (None, _) if !peer.ip().is_loopback() => forbidden("open them through the workspace edge"),
            (None, _) if !loopback_host(headers) => forbidden("not a loopback host"),
            (None, _) => Ok(()),
        }
    }

    /// The active cell, the other cells, the `--ide-root`s, then `~` and `/`.
    pub fn roots(&self) -> Vec<FileRoot> {
        let root = |id: String, label: String, dir: &Path, lazy: bool| FileRoot { id, label, writable: true, path: dir.display().to_string(), lazy, storage: None };
        let active = cells::active_cell(&self.config);
        let mut roots: Vec<FileRoot> = active.iter().map(|(name, dir)| root(CELL_ROOT.to_owned(), format!("Cell {name} (active)"), dir, false)).collect();
        for name in cells::info(&self.config).cells {
            if active.as_ref().is_some_and(|(a, _)| *a == name) {
                continue;
            }
            if let Some(dir) = cells::cell_folder(&self.config, &name) {
                roots.push(root(format!("{CELL_PREFIX}{name}"), format!("Cell {name}"), &dir, false));
            }
        }
        roots.extend(self.extra.iter().map(|(id, label, dir)| root(id.clone(), label.clone(), dir, false)));
        roots.push(root(HOME_ROOT.to_owned(), "~ (home)".to_owned(), &factoryos_pty_home(), true));
        roots.push(root(FS_ROOT.to_owned(), "/".to_owned(), Path::new("/"), true));
        roots
    }

    pub(crate) fn root_dir(&self, id: &str) -> Result<PathBuf, HttpError> {
        let dir = if id == CELL_ROOT {
            cells::active_cell(&self.config).map(|(_, dir)| dir)
        } else if id == HOME_ROOT {
            Some(factoryos_pty_home())
        } else if id == FS_ROOT {
            Some(PathBuf::from("/"))
        } else if let Some(name) = id.strip_prefix(CELL_PREFIX) {
            // Only a cell of the cells folder, by its listed name (not a path).
            cells::info(&self.config).cells.iter().any(|c| c == name).then(|| cells::cell_folder(&self.config, name)).flatten()
        } else {
            self.extra.iter().find(|(other, ..)| other == id).map(|(.., dir)| dir.clone())
        };
        let dir = dir.ok_or_else(|| (StatusCode::NOT_FOUND, format!("no root {id}")))?;
        std::fs::canonicalize(&dir).map_err(|err| (StatusCode::NOT_FOUND, format!("root {id}: {err}")))
    }

    /// One folder `dir` of `root` (`""`: its top), its entries only; what lies under its folders
    /// comes with their own listing. Symlinks are listed as what they point to, when that is in the root.
    pub fn folder(&self, root: &str, dir: &str) -> Result<FileTree, HttpError> {
        let top = self.root_dir(root)?;
        let folder = if dir.trim_matches('/').is_empty() {
            top.clone()
        } else {
            let rel = checked_path(dir.trim_matches('/'))?;
            let folder = std::fs::canonicalize(top.join(rel)).map_err(|_| (StatusCode::NOT_FOUND, format!("no folder {dir}")))?;
            if !folder.starts_with(&top) || !folder.is_dir() {
                return Err((StatusCode::NOT_FOUND, format!("no folder {dir}")));
            }
            folder
        };
        let prefix = dir.trim_matches('/');
        let mut tree = FileTree::default();
        let read = std::fs::read_dir(&folder).map_err(io_error)?;
        for entry in read.flatten() {
            let name = entry.file_name().to_string_lossy().into_owned();
            let Ok(meta) = std::fs::metadata(entry.path()) else { continue };
            if entry.path().symlink_metadata().is_ok_and(|m| m.file_type().is_symlink())
                && !std::fs::canonicalize(entry.path()).is_ok_and(|target| target.starts_with(&top))
            {
                continue;
            }
            if tree.entries.len() >= MAX_ENTRIES {
                tree.truncated = true;
                break;
            }
            let path = if prefix.is_empty() { name } else { format!("{prefix}/{name}") };
            tree.entries.push(FileEntry { path, bytes: if meta.is_dir() { 0 } else { meta.len() }, dir: meta.is_dir() });
        }
        tree.entries.sort_by(|a, b| a.path.cmp(&b.path));
        Ok(tree)
    }

    pub fn tree(&self, root: &str) -> Result<FileTree, HttpError> {
        if [HOME_ROOT, FS_ROOT].contains(&root) {
            return self.folder(root, "");
        }
        let dir = self.root_dir(root)?;
        let mut tree = FileTree::default();
        let mut stack = vec![dir.clone()];
        while let Some(d) = stack.pop() {
            let Ok(read) = std::fs::read_dir(&d) else { continue };
            for entry in read.flatten() {
                // Symlinks are not followed (nor listed): they may lead out of the root.
                let Ok(meta) = entry.path().symlink_metadata() else { continue };
                let name = entry.file_name().to_string_lossy().into_owned();
                if meta.file_type().is_symlink() || (meta.is_dir() && SKIPPED_DIRS.contains(&name.as_str())) {
                    continue;
                }
                let Some(rel) = relative(&dir, &entry.path()) else { continue };
                if tree.entries.len() >= MAX_ENTRIES {
                    tree.truncated = true;
                    break;
                }
                if meta.is_dir() {
                    stack.push(entry.path());
                }
                tree.entries.push(FileEntry { path: rel, bytes: if meta.is_dir() { 0 } else { meta.len() }, dir: meta.is_dir() });
            }
        }
        tree.entries.sort_by(|a, b| a.path.cmp(&b.path));
        Ok(tree)
    }

    pub fn read(&self, root: &str, path: &str) -> Result<FileText, HttpError> {
        let file = contained_file(&self.root_dir(root)?, path)?;
        let (bytes, version) = read_text(&file, path)?;
        let text = String::from_utf8(bytes).map_err(|_| (StatusCode::UNSUPPORTED_MEDIA_TYPE, format!("{path} is not UTF-8 text")))?;
        Ok(FileText { text, version })
    }

    pub fn save(&self, root: &str, path: &str, save: FileSave) -> Result<FileSaved, HttpError> {
        if save.text.len() as u64 > MAX_TEXT_BYTES {
            return Err((StatusCode::PAYLOAD_TOO_LARGE, format!("{path}: over {MAX_TEXT_BYTES} bytes")));
        }
        let file = contained_file(&self.root_dir(root)?, path)?;
        let meta = std::fs::metadata(&file).map_err(io_error)?;
        if let Some(base) = &save.base_version
            && *base != version(&meta)
        {
            return Err((StatusCode::CONFLICT, format!("{path} changed on disk since it was opened")));
        }
        let name = file.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
        let tmp = file.with_file_name(format!(".{name}.fos-ide-{}", std::process::id()));
        let written = std::fs::write(&tmp, save.text.as_bytes())
            .and_then(|()| std::fs::set_permissions(&tmp, meta.permissions()))
            .and_then(|()| std::fs::rename(&tmp, &file));
        if let Err(err) = written {
            let _ = std::fs::remove_file(&tmp);
            return Err(io_error(err));
        }
        let meta = std::fs::metadata(&file).map_err(io_error)?;
        // A file of the user's folders (a cell's, the products', the catalogue's) to their store (S3 mode).
        crate::storage::saved(&file);
        Ok(FileSaved { version: version(&meta) })
    }

    /// The file to download (and its size), up to [`MAX_TRANSFER_BYTES`].
    pub fn download(&self, root: &str, path: &str) -> Result<(PathBuf, u64), HttpError> {
        let file = contained_file(&self.root_dir(root)?, path)?;
        let len = std::fs::metadata(&file).map_err(io_error)?.len();
        if len > MAX_TRANSFER_BYTES {
            return Err((StatusCode::PAYLOAD_TOO_LARGE, format!("{path} is {len} bytes, over {MAX_TRANSFER_BYTES}")));
        }
        Ok((file, len))
    }

    /// An uploaded file `path` (its folder must exist), written through a temporary file; an existing
    /// one only with `overwrite`.
    pub fn upload(&self, root: &str, path: &str, bytes: &[u8], overwrite: bool) -> Result<FileEntry, HttpError> {
        let dir = self.root_dir(root)?;
        let rel = checked_path(path)?;
        let name = rel.file_name().ok_or_else(|| (StatusCode::BAD_REQUEST, format!("invalid file path: {path}")))?.to_owned();
        let parent = match rel.parent().filter(|p| !p.as_os_str().is_empty()) {
            Some(parent) => {
                let folder = std::fs::canonicalize(dir.join(parent)).map_err(|_| (StatusCode::NOT_FOUND, format!("no folder {}", parent.display())))?;
                if !folder.starts_with(&dir) || !folder.is_dir() {
                    return Err((StatusCode::NOT_FOUND, format!("no folder {}", parent.display())));
                }
                folder
            }
            None => dir,
        };
        let target = parent.join(&name);
        match target.symlink_metadata() {
            Ok(meta) if meta.file_type().is_symlink() => return Err((StatusCode::FORBIDDEN, format!("{path} is a symlink"))),
            Ok(meta) if meta.is_dir() => return Err((StatusCode::CONFLICT, format!("{path} is a folder"))),
            Ok(_) if !overwrite => return Err((StatusCode::CONFLICT, format!("{path} exists"))),
            _ => {}
        }
        let tmp = parent.join(format!(".{}.fos-upload-{}", name.to_string_lossy(), std::process::id()));
        if let Err(err) = std::fs::write(&tmp, bytes).and_then(|()| std::fs::rename(&tmp, &target)) {
            let _ = std::fs::remove_file(&tmp);
            return Err(io_error(err));
        }
        crate::storage::saved(&target);
        Ok(FileEntry { path: path.to_owned(), bytes: bytes.len() as u64, dir: false })
    }
}

/// `path` relative, `/`-separated, without `.`, `..`, or anything absolute.
fn checked_path(path: &str) -> Result<&Path, HttpError> {
    let bad = || (StatusCode::BAD_REQUEST, format!("invalid file path: {path}"));
    if path.is_empty() || path.len() > 1024 || path.contains(['\\', '\0', ':']) || path.starts_with('/') {
        return Err(bad());
    }
    let rel = Path::new(path);
    if !rel.components().all(|c| matches!(c, Component::Normal(_))) || path.split('/').any(|part| part.is_empty() || part == "." || part == "..") {
        return Err(bad());
    }
    Ok(rel)
}

/// A file under `dir`: `path` relative, `/`-separated, without `.`, `..`, or anything absolute; and
/// once symlinks are resolved, still inside `dir` (canonical).
pub(crate) fn contained_file(dir: &Path, path: &str) -> Result<PathBuf, HttpError> {
    let rel = checked_path(path)?;
    let not_found = || (StatusCode::NOT_FOUND, format!("no file {path}"));
    let file = std::fs::canonicalize(dir.join(rel)).map_err(|_| not_found())?;
    if !file.starts_with(dir) || !file.is_file() {
        return Err(not_found());
    }
    Ok(file)
}

fn read_text(file: &Path, path: &str) -> Result<(Vec<u8>, String), HttpError> {
    let meta = std::fs::metadata(file).map_err(io_error)?;
    if meta.len() > MAX_TEXT_BYTES {
        return Err((StatusCode::PAYLOAD_TOO_LARGE, format!("{path} is {} bytes, over {MAX_TEXT_BYTES}", meta.len())));
    }
    Ok((std::fs::read(file).map_err(io_error)?, version(&meta)))
}

/// What changes when the file does: its modification time and length.
fn version(meta: &std::fs::Metadata) -> String {
    let nanos = meta.modified().ok().and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok()).map_or(0, |d| d.as_nanos());
    format!("{nanos}-{}", meta.len())
}

fn relative(root: &Path, path: &Path) -> Option<String> {
    let rel = path.strip_prefix(root).ok()?;
    let parts: Option<Vec<&str>> = rel.components().map(|c| c.as_os_str().to_str()).collect();
    Some(parts?.join("/"))
}

fn slug(label: &str) -> String {
    label.chars().filter_map(|c| if c.is_ascii_alphanumeric() { Some(c.to_ascii_lowercase()) } else if matches!(c, '-' | '_' | ' ') { Some('-') } else { None }).collect()
}

fn loopback_host(headers: &HeaderMap) -> bool {
    let Some(host) = headers.get(header::HOST).and_then(|v| v.to_str().ok()) else { return false };
    let name = match host.strip_prefix('[') {
        Some(v6) => v6.split(']').next().unwrap_or(""),
        None => host.rsplit_once(':').map_or(host, |(name, _)| name),
    };
    name.eq_ignore_ascii_case("localhost") || name.parse::<std::net::IpAddr>().is_ok_and(|ip| ip.is_loopback())
}

fn io_error(err: std::io::Error) -> HttpError {
    (StatusCode::INTERNAL_SERVER_ERROR, err.to_string())
}

// --- HTTP ---

/// Its own router (with its own state), merged outside the CORS layer.
pub fn router(files: Arc<FilesService>) -> Router {
    Router::new()
        .route(api::FILES_ROOTS, get(roots))
        .route(api::FILES_TREE, get(tree))
        .route(api::FILES_FILE, get(read).put(save))
        .route(api::FILES_RAW, get(download).put(upload).layer(DefaultBodyLimit::max(MAX_TRANSFER_BYTES as usize)))
        .with_state(files)
}

async fn blocking<T: Send + 'static>(work: impl FnOnce() -> Result<T, HttpError> + Send + 'static) -> Result<T, HttpError> {
    tokio::task::spawn_blocking(work).await.map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?
}

async fn roots(State(files): State<Arc<FilesService>>, ConnectInfo(peer): ConnectInfo<SocketAddr>, headers: HeaderMap) -> Result<Json<Vec<FileRoot>>, HttpError> {
    files.admit(peer, &headers)?;
    let roles = files.roles(peer, &headers);
    blocking(move || Ok(files.roots_for(roles))).await.map(Json)
}

async fn tree(
    State(files): State<Arc<FilesService>>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    UrlPath(root): UrlPath<String>,
    Query(query): Query<HashMap<String, String>>,
) -> Result<Json<FileTree>, HttpError> {
    files.admit(peer, &headers)?;
    let dir = query.get("dir").cloned();
    blocking(move || match dir {
        Some(dir) => files.folder(&root, &dir),
        None => files.tree(&root),
    })
    .await
    .map(Json)
}

/// This backend's user's home (the terminals' too).
fn factoryos_pty_home() -> PathBuf {
    factoryos_pty::home_dir()
}

async fn read(
    State(files): State<Arc<FilesService>>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    UrlPath((root, path)): UrlPath<(String, String)>,
) -> Result<Json<FileText>, HttpError> {
    files.admit(peer, &headers)?;
    blocking(move || files.read(&root, &path)).await.map(Json)
}

/// The checks of every route, plus no other site's page (a download is a top-level navigation).
/// Product & Task's uploads and catalogue writes are admitted the same way.
pub(crate) fn admit_transfer(files: &FilesService, peer: SocketAddr, headers: &HeaderMap) -> Result<(), HttpError> {
    files.admit(peer, headers)?;
    if !crate::services::dcv::same_origin(headers) {
        return Err((StatusCode::FORBIDDEN, "workspace files: cross-origin request refused".to_owned()));
    }
    Ok(())
}

/// `attachment; filename="<ascii>"; filename*=UTF-8''<the name, percent-encoded>`.
fn attachment(path: &str) -> String {
    let name = path.rsplit('/').next().unwrap_or(path);
    let ascii: String = name.chars().map(|c| if c.is_ascii_graphic() && c != '"' && c != '\\' || c == ' ' { c } else { '_' }).collect();
    format!("attachment; filename=\"{ascii}\"; filename*=UTF-8''{}", factoryos_protocol::codegen::encode_path(name))
}

async fn download(
    State(files): State<Arc<FilesService>>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    UrlPath((root, path)): UrlPath<(String, String)>,
) -> Result<Response, HttpError> {
    admit_transfer(&files, peer, &headers)?;
    let name = path.clone();
    let (file, len) = blocking(move || files.download(&root, &path)).await?;
    let file = tokio::fs::File::open(&file).await.map_err(io_error)?;
    Ok((
        [
            (header::CONTENT_TYPE, "application/octet-stream".to_owned()),
            (header::CONTENT_LENGTH, len.to_string()),
            (header::CONTENT_DISPOSITION, attachment(&name)),
            (header::X_CONTENT_TYPE_OPTIONS, "nosniff".to_owned()),
            (header::CACHE_CONTROL, "no-store".to_owned()),
        ],
        Body::from_stream(tokio_util::io::ReaderStream::new(file)),
    )
        .into_response())
}

async fn upload(
    State(files): State<Arc<FilesService>>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    UrlPath((root, path)): UrlPath<(String, String)>,
    Query(query): Query<HashMap<String, String>>,
    body: Bytes,
) -> Result<Json<FileEntry>, HttpError> {
    admit_transfer(&files, peer, &headers)?;
    let overwrite = query.get("overwrite").is_some_and(|v| v == "1");
    blocking(move || files.upload(&root, &path, &body, overwrite)).await.map(Json)
}

async fn save(
    State(files): State<Arc<FilesService>>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    UrlPath((root, path)): UrlPath<(String, String)>,
    Json(body): Json<FileSave>,
) -> Result<Json<FileSaved>, HttpError> {
    files.admit(peer, &headers)?;
    blocking(move || files.save(&root, &path, body)).await.map(Json)
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use axum::body::Body;
    use axum::extract::connect_info::MockConnectInfo;
    use axum::http::Request;
    use tower::ServiceExt as _;

    fn store() -> Arc<ConfigStore> {
        Arc::new(ConfigStore::open(tmp().join("factoryos.config.json")))
    }

    fn service(dir: &Path, owner: Option<&str>) -> Arc<FilesService> {
        let config = store();
        FilesService::new(config, &[format!("Work={}", dir.display())], owner.map(str::to_owned)).unwrap()
    }

    fn headers(pairs: &[(&str, &str)]) -> HeaderMap {
        let mut map = HeaderMap::new();
        for (k, v) in pairs {
            map.insert(axum::http::HeaderName::from_bytes(k.as_bytes()).unwrap(), v.parse().unwrap());
        }
        map
    }

    fn tmp() -> PathBuf {
        let dir = std::env::temp_dir().join(format!("fos-files-{}-{}", std::process::id(), rand_suffix()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::canonicalize(dir).unwrap()
    }

    /// Unique per call (the clock alone repeats across parallel tests).
    fn rand_suffix() -> String {
        static N: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let n = N.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        format!("{}-{n}", std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos())
    }

    #[test]
    fn only_the_owner_or_a_local_caller() {
        let dir = tmp();
        let files = service(&dir, Some("jdoe"));
        let edge: SocketAddr = "10.0.0.5:4000".parse().unwrap();
        let local: SocketAddr = "127.0.0.1:4000".parse().unwrap();
        assert!(files.admit(edge, &headers(&[("x-user-alias", "jdoe")])).is_ok());
        assert_eq!(files.admit(edge, &headers(&[("x-user-alias", "alice")])).unwrap_err().0, StatusCode::FORBIDDEN);
        assert_eq!(files.admit(edge, &headers(&[("host", "localhost")])).unwrap_err().0, StatusCode::FORBIDDEN, "no alias from afar");
        assert!(files.admit(local, &headers(&[("host", "127.0.0.1:8080")])).is_ok());
        assert!(files.admit(local, &headers(&[("host", "[::1]:8080")])).is_ok());
        assert!(files.admit(local, &headers(&[("host", "localhost")])).is_ok());
        assert_eq!(files.admit(local, &headers(&[("host", "evil.example:8080")])).unwrap_err().0, StatusCode::FORBIDDEN, "DNS rebinding");
        assert_eq!(files.admit(local, &headers(&[])).unwrap_err().0, StatusCode::FORBIDDEN);
        let ownerless = service(&dir, None);
        assert_eq!(ownerless.admit(edge, &headers(&[("x-user-alias", "jdoe")])).unwrap_err().0, StatusCode::FORBIDDEN);
    }

    #[test]
    fn paths_stay_inside_the_root() {
        let dir = tmp();
        let outside = tmp();
        std::fs::write(outside.join("secret.txt"), "no").unwrap();
        std::fs::write(dir.join("a.st"), "x").unwrap();
        std::os::unix::fs::symlink(outside.join("secret.txt"), dir.join("link.txt")).unwrap();
        let files = service(&dir, None);
        for bad in ["../secret.txt", "/etc/passwd", "a/../a.st", "./a.st", "", "a\\b", "c:x"] {
            assert_eq!(files.read("work", bad).unwrap_err().0, StatusCode::BAD_REQUEST, "{bad}");
        }
        assert_eq!(files.read("work", "link.txt").unwrap_err().0, StatusCode::NOT_FOUND, "symlink out of the root");
        assert_eq!(files.read("nope", "a.st").unwrap_err().0, StatusCode::NOT_FOUND, "not an allowed root");
        assert_eq!(files.read("work", "a.st").unwrap().text, "x");
        let listed: Vec<_> = files.tree("work").unwrap().entries.into_iter().map(|e| e.path).collect();
        assert_eq!(listed, ["a.st"], "symlinks are not listed");
    }

    #[test]
    fn saves_unless_changed_meanwhile() {
        let dir = tmp();
        std::fs::create_dir(dir.join("plc")).unwrap();
        std::fs::create_dir(dir.join(".git")).unwrap();
        std::fs::write(dir.join("plc/main.st"), "PROGRAM main\nEND_PROGRAM\n").unwrap();
        let files = service(&dir, None);
        let tree = files.tree("work").unwrap();
        assert_eq!(tree.entries.iter().map(|e| (e.path.as_str(), e.dir)).collect::<Vec<_>>(), [("plc", true), ("plc/main.st", false)]);
        let opened = files.read("work", "plc/main.st").unwrap();
        let saved = files.save("work", "plc/main.st", FileSave { text: "PROGRAM main\n  x := 1;\nEND_PROGRAM\n".into(), base_version: Some(opened.version.clone()) }).unwrap();
        assert_ne!(saved.version, opened.version);
        assert!(std::fs::read_to_string(dir.join("plc/main.st")).unwrap().contains("x := 1"));
        let stale = files.save("work", "plc/main.st", FileSave { text: "lost".into(), base_version: Some(opened.version) });
        assert_eq!(stale.unwrap_err().0, StatusCode::CONFLICT);
        files.save("work", "plc/main.st", FileSave { text: "overwritten".into(), base_version: None }).unwrap();
        assert_eq!(std::fs::read_to_string(dir.join("plc/main.st")).unwrap(), "overwritten");
        assert_eq!(files.save("work", "plc/new.st", FileSave { text: "x".into(), base_version: None }).unwrap_err().0, StatusCode::NOT_FOUND, "no new files");
        assert!(std::fs::read_dir(dir.join("plc")).unwrap().all(|e| !e.unwrap().file_name().to_string_lossy().contains("fos-ide")), "no temporary file left");
    }

    #[test]
    fn uploads_into_folders_only_new_files_unless_overwriting() {
        let dir = tmp();
        let outside = tmp();
        std::fs::create_dir(dir.join("biw")).unwrap();
        std::os::unix::fs::symlink(&outside, dir.join("out")).unwrap();
        std::os::unix::fs::symlink(outside.join("x"), dir.join("biw/link")).unwrap();
        let files = service(&dir, None);
        let entry = files.upload("work", "biw/weld points.json", b"[1,2]", false).unwrap();
        assert_eq!((entry.path.as_str(), entry.bytes), ("biw/weld points.json", 5));
        assert_eq!(std::fs::read(dir.join("biw/weld points.json")).unwrap(), b"[1,2]");
        assert_eq!(files.upload("work", "biw/weld points.json", b"new", false).unwrap_err().0, StatusCode::CONFLICT);
        files.upload("work", "biw/weld points.json", b"new", true).unwrap();
        assert_eq!(std::fs::read(dir.join("biw/weld points.json")).unwrap(), b"new");
        files.upload("work", "top.bin", &[0, 255], false).unwrap();
        assert_eq!(files.upload("work", "biw", b"x", true).unwrap_err().0, StatusCode::CONFLICT, "a folder");
        assert_eq!(files.upload("work", "nope/a.txt", b"x", false).unwrap_err().0, StatusCode::NOT_FOUND, "no such folder");
        assert_eq!(files.upload("work", "out/a.txt", b"x", false).unwrap_err().0, StatusCode::NOT_FOUND, "a folder symlinked out of the root");
        assert_eq!(files.upload("work", "biw/link", b"x", true).unwrap_err().0, StatusCode::FORBIDDEN, "through a symlink");
        assert_eq!(files.upload("work", "../x", b"x", false).unwrap_err().0, StatusCode::BAD_REQUEST);
        assert!(std::fs::read_dir(&outside).unwrap().next().is_none(), "nothing written outside");
        let (file, len) = files.download("work", "top.bin").unwrap();
        assert_eq!((std::fs::read(file).unwrap(), len), (vec![0, 255], 2));
        assert!(std::fs::read_dir(dir.join("biw")).unwrap().all(|e| !e.unwrap().file_name().to_string_lossy().contains("fos-upload")));
    }

    #[test]
    fn downloads_are_attachments_with_their_name() {
        assert_eq!(attachment("biw/out/weld points.json"), "attachment; filename=\"weld points.json\"; filename*=UTF-8''weld%20points.json");
        assert_eq!(attachment("a/\"x\"ü.txt"), "attachment; filename=\"_x__.txt\"; filename*=UTF-8''%22x%22%C3%BC.txt");
    }

    #[tokio::test]
    async fn transfers_refuse_other_sites() {
        let dir = tmp();
        std::fs::write(dir.join("a.st"), "x").unwrap();
        let app = router(service(&dir, None)).layer(MockConnectInfo("127.0.0.1:4000".parse::<SocketAddr>().unwrap()));
        let get = |origin: Option<&str>| {
            let mut r = Request::get(api::files_raw("work", "a.st", false)).header("host", "127.0.0.1:8080");
            if let Some(o) = origin {
                r = r.header("origin", o);
            }
            r.body(Body::empty()).unwrap()
        };
        let ok = app.clone().oneshot(get(None)).await.unwrap();
        assert_eq!(ok.status(), StatusCode::OK);
        assert!(ok.headers()[header::CONTENT_DISPOSITION].to_str().unwrap().starts_with("attachment; filename=\"a.st\""));
        assert_eq!(app.clone().oneshot(get(Some("https://evil.example"))).await.unwrap().status(), StatusCode::FORBIDDEN);
        let put = Request::put(api::files_raw("work", "b.bin", false)).header("host", "127.0.0.1:8080").body(Body::from(vec![1u8, 2, 3])).unwrap();
        assert_eq!(app.oneshot(put).await.unwrap().status(), StatusCode::OK);
        assert_eq!(std::fs::read(dir.join("b.bin")).unwrap(), [1, 2, 3]);
    }

    #[test]
    fn refuses_large_and_binary_files() {
        let dir = tmp();
        std::fs::write(dir.join("big.txt"), vec![b'a'; MAX_TEXT_BYTES as usize + 1]).unwrap();
        std::fs::write(dir.join("bin.dat"), [0xff, 0xfe, 0x00]).unwrap();
        let files = service(&dir, None);
        assert_eq!(files.read("work", "big.txt").unwrap_err().0, StatusCode::PAYLOAD_TOO_LARGE);
        assert_eq!(files.read("work", "bin.dat").unwrap_err().0, StatusCode::UNSUPPORTED_MEDIA_TYPE);
    }

    #[test]
    fn folders_one_at_a_time_and_the_standard_roots() {
        let dir = tmp();
        let outside = tmp();
        std::fs::create_dir_all(dir.join("biw/out")).unwrap();
        std::fs::write(dir.join("biw/out/a.png"), "png").unwrap();
        std::fs::write(dir.join("top.txt"), "t").unwrap();
        std::os::unix::fs::symlink(dir.join("biw"), dir.join("biw-link")).unwrap();
        std::os::unix::fs::symlink(&outside, dir.join("away")).unwrap();
        let files = service(&dir, None);
        let paths = |t: FileTree| t.entries.into_iter().map(|e| (e.path, e.dir)).collect::<Vec<_>>();
        assert_eq!(paths(files.folder("work", "").unwrap()), [("biw".into(), true), ("biw-link".into(), true), ("top.txt".into(), false)], "a link out of the root is left out");
        assert_eq!(paths(files.folder("work", "biw").unwrap()), [("biw/out".into(), true)], "one level");
        assert_eq!(paths(files.folder("work", "biw/out/").unwrap()), [("biw/out/a.png".into(), false)]);
        for bad in ["../x", "away", "nope", "top.txt"] {
            assert!(files.folder("work", bad).is_err(), "{bad}");
        }
        let roots = files.roots();
        let ids: Vec<&str> = roots.iter().map(|r| r.id.as_str()).collect();
        assert!(ids.ends_with(&["work", "home", "fs"]), "{ids:?}");
        assert!(roots.iter().find(|r| r.id == "fs").is_some_and(|r| r.lazy && r.path == "/"));
        assert_eq!(roots.iter().find(|r| r.id == "work").unwrap().path, dir.display().to_string());
        assert!(files.tree("fs").unwrap().entries.iter().any(|e| e.path == "tmp" || e.path == "private"), "/ is listed a level at a time");
        assert_eq!(files.root_dir("cell-../..").unwrap_err().0, StatusCode::NOT_FOUND, "a cell only by its listed name");
        let config = store();
        for reserved in ["home=/tmp", "fs=/tmp", "cell-x=/tmp"] {
            assert!(FilesService::new(config.clone(), &[reserved.into()], None).is_err(), "{reserved}");
        }
    }

    #[test]
    fn roots_must_be_absolute_and_distinct() {
        let config = store();
        assert!(FilesService::new(config.clone(), &["Work=relative/dir".into()], None).is_err());
        assert!(FilesService::new(config.clone(), &["cell=/tmp".into()], None).is_err());
        assert!(FilesService::new(config.clone(), &["A=/tmp".into(), "a=/var".into()], None).is_err());
        assert_eq!(FilesService::new(config, &["/tmp/biw".into()], None).unwrap().extra[0].0, "biw");
    }

    #[tokio::test]
    async fn the_router_refuses_a_remote_caller_without_the_owner() {
        let dir = tmp();
        std::fs::write(dir.join("a.st"), "x").unwrap();
        let put = |alias: Option<&str>| {
            let mut request = Request::put(api::files_file("work", "a.st")).header("content-type", "application/json").header("host", "ws.example");
            if let Some(alias) = alias {
                request = request.header("x-user-alias", alias);
            }
            request.body(Body::from(r#"{"text":"y"}"#)).unwrap()
        };
        let app = router(service(&dir, Some("jdoe"))).layer(MockConnectInfo("10.0.0.5:4000".parse::<SocketAddr>().unwrap()));
        assert_eq!(app.clone().oneshot(put(None)).await.unwrap().status(), StatusCode::FORBIDDEN);
        assert_eq!(app.clone().oneshot(put(Some("alice"))).await.unwrap().status(), StatusCode::FORBIDDEN);
        assert_eq!(std::fs::read_to_string(dir.join("a.st")).unwrap(), "x");
        assert_eq!(app.oneshot(put(Some("jdoe"))).await.unwrap().status(), StatusCode::OK);
        assert_eq!(std::fs::read_to_string(dir.join("a.st")).unwrap(), "y");
    }

    #[test]
    fn the_shared_catalogue_is_a_storage_root() {
        let dir = tmp();
        let files = service(&dir, Some("jdoe"));
        let edge = SocketAddr::from(([10, 0, 0, 1], 1));
        let jdoe = headers(&[("x-user-alias", "jdoe")]);
        // No authz set: no roles (fail closed).
        assert!(!files.roles(edge, &jdoe).catalogue_admin);
        files.set_authz(crate::authz::ConfigAuthz::new(Some("jdoe"), Some("jdoe".into())).unwrap());
        let roles = files.roles(edge, &jdoe);
        let shared = |roles| files.roots_for(roles).into_iter().find(|r| r.id == SHARED_CATALOGUE_ROOT).unwrap();
        assert!(shared(roles).writable);
        assert_eq!(shared(roles).storage, Some(factoryos_protocol::storage::Scope::Catalogue));
        assert!(!shared(crate::authz::Roles::default()).writable);
        // Not reachable through the files routes (the storage routes check the roles).
        assert_eq!(files.root_dir(SHARED_CATALOGUE_ROOT).unwrap_err().0, StatusCode::NOT_FOUND);
        assert!(FilesService::new(store(), &["shared catalogue=/tmp".to_owned()], None).is_err());
    }

    #[tokio::test]
    async fn saves_and_uploads_in_the_users_folders_are_written_through() {
        use crate::storage::sync::tests_support::{eventually, storage_at};
        use factoryos_protocol::storage::Scope;
        let dir = tmp();
        let cell = dir.join("cells/c1");
        std::fs::create_dir_all(cell.join("krl")).unwrap();
        std::fs::write(cell.join("io_names.json"), "{}").unwrap();
        let (s3, storage) = storage_at(&dir);
        let _registered = crate::storage::register(&storage);
        let files = service(&cell, None);
        files.save("work", "io_names.json", FileSave { text: "{\"a\": 1}".into(), base_version: None }).unwrap();
        files.upload("work", "krl/main.src", b"DEF main()", false).unwrap();
        eventually("the save and the upload", || {
            s3.bytes(Scope::User, "cells/c1/io_names.json").as_deref() == Some(b"{\"a\": 1}".as_slice())
                && s3.bytes(Scope::User, "cells/c1/krl/main.src").as_deref() == Some(b"DEF main()".as_slice())
        })
        .await;
        // Outside the user's folders (another --ide-root): nothing.
        let other = tmp();
        std::fs::write(other.join("x.txt"), "x").unwrap();
        service(&other, None).save("work", "x.txt", FileSave { text: "y".into(), base_version: None }).unwrap();
        tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        assert_eq!(s3.keys(Scope::User), ["cells/c1/io_names.json", "cells/c1/krl/main.src"]);
        std::fs::remove_dir_all(&dir).ok();
    }
}
