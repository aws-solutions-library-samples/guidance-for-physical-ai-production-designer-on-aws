//! [`LocalStore`]: the backend's own folders as a [`Store`], where they are today. Also the working
//! copy's [`Layout`] in S3 mode (which local file a user key is).
//!
//! A key never leaves its folder: keys are validated ([`ObjectKey`]), and the deepest existing folder
//! on the way must canonicalize into the root (no symlink out); a symlink as the file itself is refused.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

use factoryos_protocol::storage::{Method, ObjectInfo, ObjectList, Scope, StorageMode, valid_prefix};
use futures::future::BoxFuture;

use super::{Fetched, MAX_SYNC_BYTES, ObjectKey, PutOutcome, Store, walk_files};

/// A local file's version tag: its size and modification time.
fn etag_of(meta: &std::fs::Metadata) -> String {
    let nanos = meta.modified().ok().and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok()).map_or(0, |d| d.as_nanos());
    format!("{}-{nanos}", meta.len())
}

/// One conditional write at a time (they read, compare, then write).
static CONDITIONAL: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Most objects one listing answers with.
pub const MAX_LIST: usize = 10_000;

/// Which folder each key prefix is.
#[derive(Clone, Debug, Default)]
pub struct Layout {
    /// [`Scope::User`]: `(first key segment, folder)`, e.g. `("products", /var/lib/factoryos/products)`.
    pub user: Vec<(String, PathBuf)>,
    /// [`Scope::Catalogue`]: the catalogue folder.
    pub catalogue: Option<PathBuf>,
}

impl Layout {
    /// The user's folders as the backend has them: products, cells, and the catalogue (where the
    /// user's own additions go: dressed robots, published products).
    pub fn of_config(config: &crate::config_store::ConfigStore) -> Self {
        let catalogue = {
            let dir = config.resolve(&config.get().catalogue);
            PathBuf::from(if dir.trim().is_empty() { crate::paths::default_catalogue() } else { dir })
        };
        let mut user = vec![("products".to_owned(), crate::services::product::products_dir(config))];
        if let Some(cells) = crate::services::cells::cells_folder(config) {
            user.push(("cells".to_owned(), cells));
        }
        user.push(("catalogue".to_owned(), catalogue.clone()));
        Self { user, catalogue: Some(catalogue) }
    }

    pub(super) fn root(&self, key: &ObjectKey) -> Option<(&Path, String)> {
        match key.scope {
            Scope::Catalogue => Some((self.catalogue.as_deref()?, catalogue_rel(key.key()))),
            Scope::User => {
                let (first, rest) = key.key().split_once('/')?;
                let (_, dir) = self.user.iter().find(|(prefix, _)| prefix == first)?;
                let rest = if first == "catalogue" { catalogue_rel(rest) } else { rest.to_owned() };
                Some((dir.as_path(), rest))
            }
        }
    }

    /// The file `key` is. `Ok(None)`: no folder for it; `Err`: it would lead out of its folder.
    pub fn path_of(&self, key: &ObjectKey) -> anyhow::Result<Option<PathBuf>> {
        let Some((root, rel)) = self.root(key) else { return Ok(None) };
        contained(root, &rel).map(Some)
    }

    /// The user key of `path` (a file in one of the user folders), if any.
    pub fn key_of(&self, path: &Path) -> Option<ObjectKey> {
        let parent = std::fs::canonicalize(path.parent()?).ok()?;
        let path = parent.join(path.file_name()?);
        // The most specific folder first (one may lie inside another).
        let mut roots: Vec<(&String, PathBuf)> =
            self.user.iter().filter_map(|(prefix, dir)| Some((prefix, std::fs::canonicalize(dir).ok()?))).collect();
        roots.sort_by_key(|(_, dir)| std::cmp::Reverse(dir.components().count()));
        roots.into_iter().find_map(|(prefix, dir)| {
            let rel = path.strip_prefix(&dir).ok()?;
            let rel: Vec<&str> = rel.components().map(|c| c.as_os_str().to_str()).collect::<Option<_>>()?;
            ObjectKey::new(Scope::User, &format!("{prefix}/{}", rel.join("/"))).ok()
        })
    }
}

/// A catalogue key's file, relative to the catalogue folder: the scenes of the old folder
/// (`fitted-robots/<f>`, keys written before the scenes moved) are the scenes folder's (`scenes/<f>`), and a
/// flat scene file's key (`scenes/<f>.yaml`, `fitted-robots/<f>.yaml`) is its folder's `scenes/<f>/scene.yaml`.
/// Their content keeps paths relative to the old folder, which the scenes' path resolution tolerates.
pub fn catalogue_rel(rel: &str) -> String {
    use factoryos_protocol::fitter::{LEGACY_SCENES_DIR, SCENES_DIR};
    let rest = rel
        .strip_prefix(LEGACY_SCENES_DIR)
        .and_then(|r| r.strip_prefix('/'))
        .or_else(|| rel.strip_prefix(SCENES_DIR).and_then(|r| r.strip_prefix('/')));
    // A flat scene file (`fitted-robots/<f>.yaml`, `scenes/<f>.yaml`: keys written before scenes were
    // folders) is that scene's folder file, so a shared one never lands beside the folder (and never
    // over the user's own: the sync compares these paths).
    if let Some(rest) = rest
        && !rest.contains('/')
        && let Some(stem) = rest.strip_suffix(".yaml").or_else(|| rest.strip_suffix(".yml"))
        && !stem.is_empty()
    {
        return format!("{SCENES_DIR}/{stem}/scene.yaml");
    }
    match rel.strip_prefix(LEGACY_SCENES_DIR).and_then(|r| r.strip_prefix('/')) {
        Some(rest) => format!("{SCENES_DIR}/{rest}"),
        None => rel.to_owned(),
    }
}

/// `root/rel`, checked to stay in `root`: no symlinked folder on the way out of it, no symlink as the file.
fn contained(root: &Path, rel: &str) -> anyhow::Result<PathBuf> {
    // `rel` comes from a validated key: no `.`/`..`/empty segments, no backslashes.
    let target = root.join(rel);
    let root = std::fs::canonicalize(root).map_err(|err| anyhow::anyhow!("{}: {err}", root.display()))?;
    let mut existing = target.parent().map(Path::to_owned);
    while let Some(dir) = existing.clone() {
        if dir.exists() {
            break;
        }
        existing = dir.parent().map(Path::to_owned);
    }
    let existing = existing.ok_or_else(|| anyhow::anyhow!("{}: no parent", target.display()))?;
    let canonical = std::fs::canonicalize(&existing)?;
    anyhow::ensure!(canonical.starts_with(&root), "{rel} leads out of {}", root.display());
    if target.symlink_metadata().is_ok_and(|m| m.file_type().is_symlink()) {
        anyhow::bail!("{rel} is a symlink");
    }
    // The canonical form of what exists, then the rest as it is.
    let rest = target.strip_prefix(&existing).unwrap_or(Path::new(""));
    Ok(canonical.join(rest))
}

/// Write `bytes` to `path` through a temporary file, replacing it; parents created.
pub fn replace_file(path: &Path, bytes: &[u8]) -> anyhow::Result<()> {
    let dir = path.parent().ok_or_else(|| anyhow::anyhow!("{}: no parent", path.display()))?;
    std::fs::create_dir_all(dir)?;
    let tmp = dir.join(format!(".{}.storage-{}", path.file_name().unwrap_or_default().to_string_lossy(), std::process::id()));
    std::fs::write(&tmp, bytes)?;
    std::fs::rename(&tmp, path).inspect_err(|_| {
        std::fs::remove_file(&tmp).ok();
    })?;
    Ok(())
}

/// Write `bytes` to `path` only if nothing is there yet.
pub fn write_new_file(path: &Path, bytes: &[u8]) -> anyhow::Result<()> {
    if path.symlink_metadata().is_ok() {
        return Ok(());
    }
    replace_file(path, bytes)
}

pub struct LocalStore {
    layout: Arc<dyn Fn() -> Layout + Send + Sync>,
}

impl LocalStore {
    pub fn new(layout: Arc<dyn Fn() -> Layout + Send + Sync>) -> Self {
        Self { layout }
    }

    fn path(&self, key: &ObjectKey) -> anyhow::Result<PathBuf> {
        (self.layout)().path_of(key)?.ok_or_else(|| anyhow::anyhow!("no folder for {}", key.key()))
    }

    fn list_blocking(&self, scope: Scope, prefix: &str) -> anyhow::Result<ObjectList> {
        anyhow::ensure!(valid_prefix(prefix), "not a valid prefix: {prefix:?}");
        let layout = (self.layout)();
        let roots: Vec<(String, PathBuf)> = match scope {
            Scope::Catalogue => layout.catalogue.into_iter().map(|dir| (String::new(), dir)).collect(),
            Scope::User => layout
                .user
                .into_iter()
                .map(|(p, dir)| (format!("{p}/"), dir))
                .filter(|(p, _)| p.starts_with(prefix) || prefix.starts_with(p.as_str()))
                .collect(),
        };
        let mut list = ObjectList::default();
        for (key_prefix, dir) in roots {
            let Ok(dir) = std::fs::canonicalize(&dir) else { continue };
            for file in walk_files(&dir) {
                let Some(rel) = file.strip_prefix(&dir).ok().and_then(|rel| {
                    rel.components().map(|c| c.as_os_str().to_str()).collect::<Option<Vec<_>>>().map(|parts| parts.join("/"))
                }) else {
                    continue;
                };
                let key = format!("{key_prefix}{rel}");
                if !key.starts_with(prefix) {
                    continue;
                }
                if list.objects.len() >= MAX_LIST {
                    list.truncated = true;
                    return Ok(list);
                }
                let meta = file.metadata().ok();
                let bytes = meta.as_ref().map_or(0, |m| m.len());
                list.objects.push(ObjectInfo { key, bytes, etag: meta.as_ref().map(etag_of) });
            }
        }
        list.objects.sort_by(|a, b| a.key.cmp(&b.key));
        Ok(list)
    }
}

fn blocking<T: Send + 'static>(work: impl FnOnce() -> anyhow::Result<T> + Send + 'static) -> BoxFuture<'static, anyhow::Result<T>> {
    Box::pin(async move { tokio::task::spawn_blocking(work).await? })
}

impl Store for LocalStore {
    fn mode(&self) -> StorageMode {
        StorageMode::Local
    }

    fn get<'a>(&'a self, key: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<Option<Vec<u8>>>> {
        let path = self.path(key);
        blocking(move || {
            let path = path?;
            match std::fs::metadata(&path) {
                Ok(meta) if meta.is_file() => {
                    anyhow::ensure!(meta.len() <= MAX_SYNC_BYTES, "{} is larger than {MAX_SYNC_BYTES} bytes", path.display());
                    Ok(Some(std::fs::read(&path)?))
                }
                Ok(_) => Ok(None),
                Err(err) if err.kind() == std::io::ErrorKind::NotFound => Ok(None),
                Err(err) => Err(err.into()),
            }
        })
    }

    fn head<'a>(&'a self, key: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<Option<ObjectInfo>>> {
        let path = self.path(key);
        let name = key.key().to_owned();
        blocking(move || {
            Ok(std::fs::metadata(path?).ok().filter(|m| m.is_file()).map(|m| ObjectInfo { key: name, bytes: m.len(), etag: Some(etag_of(&m)) }))
        })
    }

    fn get_if<'a>(&'a self, key: &'a ObjectKey, if_none_match: Option<&'a str>) -> BoxFuture<'a, anyhow::Result<Fetched>> {
        let path = self.path(key);
        let known = if_none_match.map(str::to_owned);
        blocking(move || {
            let path = path?;
            let Ok(meta) = std::fs::metadata(&path).map_err(|_| ()).and_then(|m| if m.is_file() { Ok(m) } else { Err(()) }) else {
                return Ok(Fetched::Missing);
            };
            let etag = etag_of(&meta);
            if known.as_deref() == Some(etag.as_str()) {
                return Ok(Fetched::NotModified);
            }
            anyhow::ensure!(meta.len() <= MAX_SYNC_BYTES, "{} is larger than {MAX_SYNC_BYTES} bytes", path.display());
            Ok(Fetched::Found { bytes: std::fs::read(&path)?, etag: Some(etag) })
        })
    }

    fn put_if<'a>(&'a self, key: &'a ObjectKey, bytes: Vec<u8>, if_match: Option<&'a str>) -> BoxFuture<'a, anyhow::Result<PutOutcome>> {
        let path = self.path(key);
        let want = if_match.map(str::to_owned);
        blocking(move || {
            let path = path?;
            let _one = CONDITIONAL.lock().unwrap();
            let now = std::fs::metadata(&path).ok().filter(|m| m.is_file()).map(|m| etag_of(&m));
            if now != want {
                return Ok(PutOutcome::Conflict);
            }
            replace_file(&path, &bytes)?;
            Ok(PutOutcome::Written(std::fs::metadata(&path).ok().map(|m| etag_of(&m))))
        })
    }

    // Who may write which scope is decided before a store is asked (storage::http, crate::authz).
    fn put<'a>(&'a self, key: &'a ObjectKey, bytes: Vec<u8>) -> BoxFuture<'a, anyhow::Result<()>> {
        let path = self.path(key);
        blocking(move || replace_file(&path?, &bytes))
    }

    fn delete<'a>(&'a self, key: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<()>> {
        let path = self.path(key);
        let top = (self.layout)().root(key).map(|(root, _)| root.to_owned());
        blocking(move || {
            let path = path?;
            match std::fs::remove_file(&path) {
                Err(err) if err.kind() != std::io::ErrorKind::NotFound => return Err(err.into()),
                _ => {}
            }
            // Folders left empty go too (S3 has none), up to the folder of the scope.
            if let Some(top) = top.and_then(|top| std::fs::canonicalize(top).ok()) {
                let mut dir = path.parent().map(Path::to_owned);
                while let Some(d) = dir {
                    if d == top || !d.starts_with(&top) || std::fs::remove_dir(&d).is_err() {
                        break;
                    }
                    dir = d.parent().map(Path::to_owned);
                }
            }
            Ok(())
        })
    }

    fn copy<'a>(&'a self, from: &'a ObjectKey, to: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<()>> {
        let (from, to) = (self.path(from), self.path(to));
        blocking(move || {
            let (from, to) = (from?, to?);
            if from == to {
                return Ok(());
            }
            let meta = std::fs::symlink_metadata(&from).map_err(|err| anyhow::anyhow!("{}: {err}", from.display()))?;
            anyhow::ensure!(meta.is_file() && meta.len() <= MAX_SYNC_BYTES, "{} is not a file to copy", from.display());
            replace_file(&to, &std::fs::read(&from)?)
        })
    }

    fn list<'a>(&'a self, scope: Scope, prefix: &'a str) -> BoxFuture<'a, anyhow::Result<ObjectList>> {
        let this = LocalStore { layout: self.layout.clone() };
        let prefix = prefix.to_owned();
        blocking(move || this.list_blocking(scope, &prefix))
    }

    fn presign<'a>(&'a self, _: &'a ObjectKey, _: Method, _: Duration) -> BoxFuture<'a, anyhow::Result<Option<(String, SystemTime)>>> {
        Box::pin(async { Ok(None) })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("factoryos-storage-{name}-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::canonicalize(dir).unwrap()
    }

    fn store(root: &Path) -> LocalStore {
        let (products, catalogue) = (root.join("products"), root.join("catalogue"));
        std::fs::create_dir_all(&products).unwrap();
        std::fs::create_dir_all(&catalogue).unwrap();
        LocalStore::new(Arc::new(move || Layout {
            user: vec![("products".into(), products.clone()), ("catalogue".into(), catalogue.clone())],
            catalogue: Some(catalogue.clone()),
        }))
    }

    #[tokio::test]
    async fn round_trip() {
        let root = temp("round-trip");
        let store = store(&root);
        let key = ObjectKey::new(Scope::User, "products/box/product.json").unwrap();
        assert_eq!(store.get(&key).await.unwrap(), None);
        store.put(&key, b"{}".to_vec()).await.unwrap();
        assert_eq!(std::fs::read(root.join("products/box/product.json")).unwrap(), b"{}");
        assert_eq!(store.get(&key).await.unwrap().as_deref(), Some(b"{}".as_slice()));
        assert_eq!(store.head(&key).await.unwrap().map(|o| o.bytes), Some(2));
        let list = store.list(Scope::User, "products/").await.unwrap();
        assert_eq!(list.objects.iter().map(|o| (o.key.as_str(), o.bytes)).collect::<Vec<_>>(), [("products/box/product.json", 2)]);
        // Conditional reads and writes.
        let etag = list.objects[0].etag.clone();
        assert_eq!(store.get_if(&key, etag.as_deref()).await.unwrap(), Fetched::NotModified);
        assert!(matches!(store.get_if(&key, Some("old")).await.unwrap(), Fetched::Found { .. }));
        assert_eq!(store.put_if(&key, b"x".to_vec(), None).await.unwrap(), PutOutcome::Conflict);
        assert!(matches!(store.put_if(&key, b"{1}".to_vec(), etag.as_deref()).await.unwrap(), PutOutcome::Written(_)));
        assert!(store.list(Scope::User, "cat").await.unwrap().objects.is_empty());
        assert_eq!(store.presign(&key, Method::Get, Duration::from_secs(1)).await.unwrap(), None);
        // The working copy's mapping both ways.
        let layout = (store.layout)();
        assert_eq!(layout.key_of(&root.join("products/box/product.json")), Some(key.clone()));
        assert_eq!(layout.key_of(&root.join("elsewhere.json")), None);
        store.delete(&key).await.unwrap();
        assert_eq!(store.get(&key).await.unwrap(), None);
        std::fs::remove_dir_all(&root).ok();
    }

    #[tokio::test]
    async fn the_local_catalogue_folder_is_the_shared_catalogue() {
        let root = temp("catalogue");
        let store = store(&root);
        std::fs::write(root.join("catalogue/a.urdf"), "<robot/>").unwrap();
        let key = ObjectKey::new(Scope::Catalogue, "a.urdf").unwrap();
        assert_eq!(store.get(&key).await.unwrap().as_deref(), Some(b"<robot/>".as_slice()));
        // Copies (across scopes too), and onto itself: nothing.
        let moved = ObjectKey::new(Scope::Catalogue, "robots/b.urdf").unwrap();
        store.copy(&key, &moved).await.unwrap();
        store.copy(&key, &ObjectKey::new(Scope::User, "catalogue/a.urdf").unwrap()).await.unwrap();
        assert_eq!(std::fs::read(root.join("catalogue/robots/b.urdf")).unwrap(), b"<robot/>");
        // A delete takes the folder it empties along, never the scope's own.
        store.delete(&moved).await.unwrap();
        assert!(!root.join("catalogue/robots").exists());
        store.delete(&key).await.unwrap();
        assert!(root.join("catalogue").is_dir());
        // Unknown user prefixes have no folder.
        assert!(store.get(&ObjectKey::new(Scope::User, "nope/x").unwrap()).await.is_err());
        std::fs::remove_dir_all(&root).ok();
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn no_way_out_through_symlinks() {
        let root = temp("symlinks");
        let outside = temp("symlinks-outside");
        std::fs::write(outside.join("secret"), "s").unwrap();
        let store = store(&root);
        std::os::unix::fs::symlink(&outside, root.join("products/escape")).unwrap();
        std::os::unix::fs::symlink(outside.join("secret"), root.join("products/link")).unwrap();
        for key in ["products/escape/secret", "products/escape/new/file", "products/link"] {
            let key = ObjectKey::new(Scope::User, key).unwrap();
            assert!(store.get(&key).await.is_err(), "{}", key.key());
            assert!(store.put(&key, b"x".to_vec()).await.is_err(), "{}", key.key());
        }
        assert_eq!(std::fs::read(outside.join("secret")).unwrap(), b"s");
        assert!(!outside.join("new").exists());
        // Listing doesn't follow them either.
        assert!(store.list(Scope::User, "").await.unwrap().objects.is_empty());
        std::fs::remove_dir_all(&root).ok();
        std::fs::remove_dir_all(&outside).ok();
    }
}
