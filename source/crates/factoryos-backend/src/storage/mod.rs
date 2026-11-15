//! Storage: the backend's files as a shared file system ([`factoryos_protocol::storage`]).
//!
//! One [`Store`] trait, two implementations, picked at startup ([`StorageOptions`]):
//! * [`local::LocalStore`]: the backend's own folders, exactly where they are today (products,
//!   cells, the catalogue). The default: a backend without a bucket works as it always has.
//! * [`s3::S3Store`]: an S3 bucket laid out as
//!   ```text
//!   <root>users/<user-id>/products/<name>/…      the user's own subtree (read/write)
//!   <root>users/<user-id>/cells/<cell>/…
//!   <root>users/<user-id>/catalogue/…             the user's own catalogue additions (dressed robots, published products)
//!   <root>users/<user-id>/assets/<sha256>         content-addressed scene assets (backend-written only)
//!   <root>catalogue/…                             the shared catalogue (everyone reads; catalogue admins write, crate::authz)
//!   ```
//!
//! **The user id** is the workspace's owner (`--workspace`), the same alias the edge's login
//! sets as `X-User-Alias` and the workspace's IAM role carries as its `FactoryosWorkspace` tag (whose
//! policy limits the role to `users/<that>/*`, so even a wrong key here can't reach another user's).
//! It never comes from a request.
//!
//! **Working copy.** The backend's tools (cell-runner, plan2cell, the planner, the dresser) work on
//! local files, so in S3 mode the local folders stay as a working copy and S3 is the store of record:
//! [`Storage::save_path`] / [`Storage::save_tree`] write a changed file or folder through to S3 after
//! an operation, [`Storage::hydrate`] fills the working copy with what is in S3 and not yet on disk
//! (a new instance), and [`Storage::commit`] takes a viewer's direct upload into it.
//!
//! **Write-through from the services.** A service that writes a user file calls [`saved`],
//! [`saved_tree`] or [`removed`] with the local path afterwards: it reaches every
//! [`register`]ed storage whose working copy holds that path (main.rs registers the backend's; a
//! test its own), in S3 mode only. Files written by other processes (the planner's plans, the
//! Virtual Cell's auto-restart setting) are picked up by [`watch::Watch`]. What a new instance gets
//! back ([`Storage::hydrate`]) and what the first S3 start uploads ([`migrate`]) are the same
//! [`USER_PREFIXES`].

pub mod assets;
pub mod http;
pub mod local;
pub mod migrate;
pub mod s3;
pub mod sync;
pub mod watch;

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime};

use futures::future::BoxFuture;
use factoryos_protocol::storage::{
    FOLDER_MARKER, MAX_BATCH_OBJECTS, Method, ObjectInfo, ObjectList, Roles, Scope, StorageInfo, StorageMode, is_platform_only, split_folder, valid_key,
    valid_prefix,
};

/// How long presigned URLs are good for.
pub const PRESIGN_TTL: Duration = Duration::from_secs(300);
/// Files larger than this are not written through (the working copy keeps them; e.g. logs, caches).
pub const MAX_SYNC_BYTES: u64 = factoryos_protocol::files::MAX_TRANSFER_BYTES;
/// Folders never written through: build output, dependencies, a reference cell's disposable view
/// of links (`<cell>/.view`, made again when it is used) and the cell migration's local backups.
const SKIPPED_DIRS: &[&str] = &[".git", "target", "node_modules", "__pycache__", ".venv", "venv", ".view", ".backup"];
/// The user's subtree, folder by folder: filled into a new instance at startup ([`Storage::hydrate`])
/// and uploaded by the first S3 start ([`migrate`], the catalogue's only what is the user's own).
pub const USER_PREFIXES: [&str; 3] = ["products/", "cells/", "catalogue/"];

/// A validated user id (the workspace owner's alias).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UserId(String);

impl UserId {
    pub fn new(id: &str) -> anyhow::Result<Self> {
        anyhow::ensure!(factoryos_protocol::workspace::is_valid_name(id), "not a user id: {id:?}");
        Ok(Self(id.to_owned()))
    }
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// An object of a scope, its key validated: only these reach a [`Store`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ObjectKey {
    pub scope: Scope,
    key: String,
}

impl ObjectKey {
    pub fn new(scope: Scope, key: &str) -> anyhow::Result<Self> {
        anyhow::ensure!(valid_key(key), "not a valid key: {key:?}");
        Ok(Self { scope, key: key.to_owned() })
    }
    pub fn key(&self) -> &str {
        &self.key
    }
    /// The S3 object key: `<root>users/<user>/<key>` or `<root>catalogue/<key>`.
    pub fn object_key(&self, root: &str, user: &UserId) -> String {
        format!("{}{}", scope_prefix(root, self.scope, user), self.key)
    }
}

/// `<root>users/<user>/` or `<root>catalogue/`.
pub fn scope_prefix(root: &str, scope: Scope, user: &UserId) -> String {
    match scope {
        Scope::User => format!("{root}users/{}/", user.as_str()),
        Scope::Catalogue => format!("{root}catalogue/"),
    }
}

/// Where bytes are kept. Async and object-safe (boxed futures), so the backend holds an `Arc<dyn Store>`.
pub trait Store: Send + Sync {
    fn mode(&self) -> StorageMode;
    /// The bytes of `key`, `None` if there is no such object.
    fn get<'a>(&'a self, key: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<Option<Vec<u8>>>>;
    /// The size and version tag of `key`, `None` if there is no such object.
    fn head<'a>(&'a self, key: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<Option<ObjectInfo>>>;
    /// `key`'s bytes and version tag, unless its tag is still `if_none_match` (a cheap poll).
    fn get_if<'a>(&'a self, key: &'a ObjectKey, if_none_match: Option<&'a str>) -> BoxFuture<'a, anyhow::Result<Fetched>>;
    fn put<'a>(&'a self, key: &'a ObjectKey, bytes: Vec<u8>) -> BoxFuture<'a, anyhow::Result<()>>;
    /// Write `bytes` only if `key` is still at version `if_match` (`None`: only if there is none yet):
    /// one writer of a read-modify-write wins, the other retries.
    fn put_if<'a>(&'a self, key: &'a ObjectKey, bytes: Vec<u8>, if_match: Option<&'a str>) -> BoxFuture<'a, anyhow::Result<PutOutcome>>;
    fn delete<'a>(&'a self, key: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<()>>;
    /// Copy `from` to `to` (any scopes), replacing `to`. A copy onto itself does nothing.
    fn copy<'a>(&'a self, from: &'a ObjectKey, to: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<()>>;
    /// The objects of `scope` under `prefix` (`""` or a valid key, optionally ending in `/`).
    fn list<'a>(&'a self, scope: Scope, prefix: &'a str) -> BoxFuture<'a, anyhow::Result<ObjectList>>;
    /// A URL that does `method` on `key` without further credentials, until the returned time.
    /// `None`: this store has no such URLs (local: bytes go over the backend's HTTP routes).
    fn presign<'a>(&'a self, key: &'a ObjectKey, method: Method, ttl: Duration) -> BoxFuture<'a, anyhow::Result<Option<(String, SystemTime)>>>;
}

/// What a conditional get found.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Fetched {
    /// Still the version asked about.
    NotModified,
    Missing,
    Found { bytes: Vec<u8>, etag: Option<String> },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PutOutcome {
    /// Written; its new version tag, if the store says.
    Written(Option<String>),
    /// Someone else wrote it meanwhile.
    Conflict,
}

/// From the command line / environment (main.rs).
#[derive(Clone, Debug, Default)]
pub struct StorageOptions {
    /// `local` (default) or `s3`.
    pub mode: Option<String>,
    pub bucket: Option<String>,
    /// The bucket's region; else `AWS_REGION` / `AWS_DEFAULT_REGION`, else the instance's (IMDS).
    pub region: Option<String>,
    /// A prefix for everything in the bucket (e.g. `dev/`), normally empty.
    pub root: Option<String>,
    /// Another S3 endpoint (path-style, e.g. a local MinIO for tests); normally AWS's.
    pub endpoint: Option<String>,
}

/// The store, whose it is, and the working copy's layout: shared by the services ([`crate::AppState`]).
pub struct Storage {
    store: Arc<dyn Store>,
    user: UserId,
    layout: Arc<dyn Fn() -> local::Layout + Send + Sync>,
}

impl Storage {
    /// `owner`: `--workspace`. S3 needs one (it is the user id); local mode uses `local` without.
    pub async fn open(
        options: &StorageOptions,
        owner: Option<&str>,
        layout: impl Fn() -> local::Layout + Send + Sync + 'static,
    ) -> anyhow::Result<Arc<Self>> {
        let layout: Arc<dyn Fn() -> local::Layout + Send + Sync> = Arc::new(layout);
        let mode = options.mode.as_deref().map(str::trim).filter(|m| !m.is_empty());
        let s3 = match mode {
            None | Some("local") => false,
            Some("s3") => true,
            Some(other) => anyhow::bail!("--storage {other}: local or s3"),
        };
        if !s3 {
            let user = UserId::new(owner.unwrap_or("local"))?;
            let store = Arc::new(local::LocalStore::new(layout.clone()));
            return Ok(Arc::new(Self { store, user, layout }));
        }
        // Fail closed: S3 mode without an owner would have no user prefix to scope anything to.
        let owner = owner.ok_or_else(|| anyhow::anyhow!("--storage s3 needs --workspace (the user id of the subtree)"))?;
        let user = UserId::new(owner)?;
        let store = s3::S3Store::open(options, user.clone()).await?;
        tracing::info!("Storage: S3, {}", store.describe());
        Ok(Arc::new(Self { store: Arc::new(store), user, layout }))
    }

    /// For tests: any store.
    #[cfg(test)]
    pub fn with_store(store: Arc<dyn Store>, user: UserId, layout: impl Fn() -> local::Layout + Send + Sync + 'static) -> Arc<Self> {
        Arc::new(Self { store, user, layout: Arc::new(layout) })
    }

    pub fn mode(&self) -> StorageMode {
        self.store.mode()
    }
    pub fn store(&self) -> &dyn Store {
        &*self.store
    }
    /// For a caller with `roles`.
    pub fn info(&self, roles: Roles) -> StorageInfo {
        StorageInfo {
            mode: self.mode(),
            user: Some(self.user.as_str().to_owned()),
            presign_ttl_secs: if self.mode() == StorageMode::S3 { PRESIGN_TTL.as_secs() } else { 0 },
            roles,
            // The routes' to set (storage::http): it depends on who asks.
            edge_path: None,
        }
    }
    /// Whose subtree [`Scope::User`] is (in S3 mode, `--workspace`).
    pub fn user(&self) -> &UserId {
        &self.user
    }
    pub fn layout(&self) -> local::Layout {
        (self.layout)()
    }

    /// The user key of a working-copy file (`None`: not in any synced folder).
    pub fn key_of(&self, path: &Path) -> Option<ObjectKey> {
        self.layout().key_of(path)
    }

    /// Write the working-copy file `path` through to the store (S3 mode; a no-op locally, where the
    /// working copy *is* the store). Errors are logged, not returned: the local save succeeded.
    pub async fn save_path(&self, path: &Path) {
        if self.mode() != StorageMode::S3 {
            return;
        }
        let Some(key) = self.key_of(path) else {
            tracing::debug!("storage: {} is outside the synced folders", path.display());
            return;
        };
        match tokio::fs::symlink_metadata(path).await {
            Ok(meta) if meta.is_file() && meta.len() <= MAX_SYNC_BYTES => {}
            Ok(_) => return,
            Err(err) if err.kind() == std::io::ErrorKind::NotFound => {
                if let Err(err) = self.store.delete(&key).await {
                    tracing::warn!("storage: delete {}: {err:#}", key.key());
                }
                return;
            }
            Err(err) => return tracing::warn!("storage: {}: {err}", path.display()),
        }
        match tokio::fs::read(path).await {
            Ok(bytes) => {
                if let Err(err) = self.store.put(&key, bytes).await {
                    tracing::warn!("storage: put {}: {err:#}", key.key());
                }
            }
            Err(err) => tracing::warn!("storage: {}: {err}", path.display()),
        }
    }

    /// [`Self::save_path`] for every file under `dir` (symlinks not followed; [`SKIPPED_DIRS`] skipped).
    pub async fn save_tree(&self, dir: &Path) {
        if self.mode() != StorageMode::S3 {
            return;
        }
        let dir = dir.to_owned();
        let files = tokio::task::spawn_blocking(move || walk_files(&dir)).await.unwrap_or_default();
        for file in files {
            self.save_path(&file).await;
        }
    }

    /// Fire-and-forget [`Self::save_path`] (from sync code).
    pub fn spawn_save_path(self: &Arc<Self>, path: PathBuf) {
        self.spawn_work(&path.clone(), |this| async move { this.save_path(&path).await });
    }

    /// Fire-and-forget [`Self::save_tree`] (after a blocking operation, from sync code).
    pub fn spawn_save_tree(self: &Arc<Self>, dir: PathBuf) {
        self.spawn_work(&dir.clone(), |this| async move { this.save_tree(&dir).await });
    }

    /// Mirror a removal: the object of `path` (a file or folder of the working copy) and every object
    /// under it whose working-copy file is gone are deleted from the store. What is still on disk is
    /// kept (a file written again meanwhile, one never filled in). S3 mode only.
    pub async fn prune(&self, path: &Path) {
        if self.mode() != StorageMode::S3 {
            return;
        }
        let Some(key) = self.key_of(path) else {
            return tracing::debug!("storage: {} is outside the synced folders", path.display());
        };
        let layout = self.layout();
        let gone = |key: &ObjectKey| match layout.path_of(key) {
            Ok(Some(file)) => file.symlink_metadata().is_err_and(|err| err.kind() == std::io::ErrorKind::NotFound),
            // Not a file of the working copy (or a symlink on the way): never deleted from here.
            _ => false,
        };
        let mut doomed = Vec::new();
        if gone(&key) && self.store.head(&key).await.is_ok_and(|info| info.is_some()) {
            doomed.push(key.clone());
        }
        match self.store.list(Scope::User, &format!("{}/", key.key())).await {
            Ok(list) => doomed.extend(list.objects.iter().filter_map(|o| ObjectKey::new(Scope::User, &o.key).ok()).filter(|k| gone(k))),
            Err(err) => tracing::warn!("storage: listing {}/: {err:#}", key.key()),
        }
        for key in doomed {
            if let Err(err) = self.store.delete(&key).await {
                tracing::warn!("storage: delete {}: {err:#}", key.key());
            }
        }
    }

    /// Run `work` on the runtime, fire-and-forget (S3 mode; from sync code).
    fn spawn_work<F>(self: &Arc<Self>, what: &Path, work: impl FnOnce(Arc<Self>) -> F)
    where
        F: std::future::Future<Output = ()> + Send + 'static,
    {
        if self.mode() != StorageMode::S3 {
            return;
        }
        let Ok(runtime) = tokio::runtime::Handle::try_current() else {
            return tracing::warn!("storage: {} not written through (no runtime)", what.display());
        };
        runtime.spawn(work(self.clone()));
    }

    /// Fill the working copy with what `scope` has under `prefix` and the disk doesn't (a new
    /// instance, an upload from another viewer, the shared catalogue's additions). Never overwrites a
    /// local file.
    pub async fn hydrate(&self, scope: Scope, prefix: &str) -> anyhow::Result<usize> {
        if self.mode() != StorageMode::S3 {
            return Ok(0);
        }
        anyhow::ensure!(valid_prefix(prefix), "not a valid prefix: {prefix:?}");
        let layout = self.layout();
        let list = self.store.list(scope, prefix).await?;
        let mut fetched = 0;
        for object in list.objects {
            let Ok(key) = ObjectKey::new(scope, &object.key) else { continue };
            let Ok(Some(path)) = layout.path_of(&key) else { continue };
            if tokio::fs::symlink_metadata(&path).await.is_ok() {
                continue;
            }
            if let Some(bytes) = self.store.get(&key).await? {
                local::write_new_file(&path, &bytes)?;
                fetched += 1;
            }
        }
        Ok(fetched)
    }

    /// Take object `key` (just written to S3, e.g. by a viewer) into the working copy, replacing the
    /// local file. Locally the store is the working copy: nothing to do. Who may write `key` is the
    /// caller's to check ([`factoryos_protocol::storage::may_write`]).
    pub async fn commit(&self, key: &ObjectKey) -> anyhow::Result<()> {
        if self.mode() != StorageMode::S3 {
            return Ok(());
        }
        let path = self.layout().path_of(key)?.ok_or_else(|| anyhow::anyhow!("{} is not in a synced folder", key.key()))?;
        let bytes = self.store.get(key).await?.ok_or_else(|| anyhow::anyhow!("no object {}", key.key()))?;
        anyhow::ensure!(bytes.len() as u64 <= MAX_SYNC_BYTES, "{} is larger than {MAX_SYNC_BYTES} bytes", key.key());
        local::replace_file(&path, &bytes)
    }

    /// S3 mode: `key` is gone from the store, so from the working copy too.
    async fn forget(&self, key: &ObjectKey) {
        if self.mode() == StorageMode::S3
            && let Ok(Some(path)) = self.layout().path_of(key)
        {
            tokio::fs::remove_file(&path).await.ok();
        }
    }

    /// The keys under folder `folder` (without its `/`), at most [`MAX_BATCH_OBJECTS`].
    async fn keys_under(&self, scope: Scope, folder: &str) -> anyhow::Result<Vec<String>> {
        let list = self.store.list(scope, &format!("{folder}/")).await?;
        anyhow::ensure!(!list.truncated && list.objects.len() <= MAX_BATCH_OBJECTS, "{folder}/ has more than {MAX_BATCH_OBJECTS} files");
        Ok(list.objects.into_iter().map(|o| o.key).collect())
    }

    /// `(from, to)` pairs for a file or folder (`a/` → `b/`), checked: both of a kind, no folder into itself.
    async fn pairs(&self, scope: Scope, from: &str, to_scope: Scope, to: &str) -> anyhow::Result<Vec<(ObjectKey, ObjectKey)>> {
        let ((from, from_dir), (to, to_dir)) = (
            split_folder(from).ok_or_else(|| anyhow::anyhow!("not a valid key: {from:?}"))?,
            split_folder(to).ok_or_else(|| anyhow::anyhow!("not a valid key: {to:?}"))?,
        );
        anyhow::ensure!(from_dir == to_dir, "a file moves to a file, a folder (ending in /) to a folder");
        if scope == to_scope {
            anyhow::ensure!(from != to, "{from} is where it is already");
            anyhow::ensure!(!(from_dir && to.starts_with(&format!("{from}/"))), "{from}/ can't go into itself");
        }
        if !from_dir {
            return Ok(vec![(ObjectKey::new(scope, from)?, ObjectKey::new(to_scope, to)?)]);
        }
        let keys = self.keys_under(scope, from).await?;
        anyhow::ensure!(!keys.is_empty(), "no folder {from}/");
        keys.iter()
            .map(|key| {
                let rest = &key[from.len() + 1..];
                Ok((ObjectKey::new(scope, key)?, ObjectKey::new(to_scope, &format!("{to}/{rest}"))?))
            })
            .collect()
    }

    /// None of the `targets` is the cell-templates CI's ([`is_platform_only`]): a folder moved or
    /// published into the user scope's `catalogue/` must not bring a `cell-templates/` with it.
    fn not_platform_only<'a>(mut targets: impl Iterator<Item = &'a ObjectKey>) -> anyhow::Result<()> {
        match targets.find(|key| is_platform_only(key.scope, key.key())) {
            Some(key) => Err(PlatformOnly(format!("{} {}: it is published by the cell-templates CI only", key.scope.as_str(), key.key())).into()),
            None => Ok(()),
        }
    }

    /// Nothing at any of the `targets`, unless `overwrite`.
    async fn free(&self, targets: impl Iterator<Item = &ObjectKey>, overwrite: bool) -> anyhow::Result<()> {
        if overwrite {
            return Ok(());
        }
        for key in targets {
            if self.store.head(key).await?.is_some() {
                return Err(Conflict(format!("{} exists", key.key())).into());
            }
        }
        Ok(())
    }

    /// A new folder `key` (its [`FOLDER_MARKER`]).
    pub async fn make_folder(&self, scope: Scope, key: &str) -> anyhow::Result<Changes> {
        let marker = ObjectKey::new(scope, &format!("{key}/{FOLDER_MARKER}"))?;
        self.free(std::iter::once(&marker), false).await?;
        self.store.put(&marker, Vec::new()).await?;
        self.commit(&marker).await?;
        Ok(vec![(marker.key().to_owned(), false)])
    }

    /// Rename/move a file or folder of `scope`: copy everything, then delete the originals. Returns
    /// what changed.
    pub async fn move_keys(&self, scope: Scope, from: &str, to: &str, overwrite: bool) -> anyhow::Result<Changes> {
        let pairs = self.pairs(scope, from, scope, to).await?;
        Self::not_platform_only(pairs.iter().map(|(_, to)| to))?;
        self.free(pairs.iter().map(|(_, to)| to), overwrite).await?;
        for (from, to) in &pairs {
            self.store.copy(from, to).await?;
            self.commit(to).await?;
        }
        for (from, _) in &pairs {
            self.store.delete(from).await?;
            self.forget(from).await;
        }
        Ok(pairs.iter().map(|(_, to)| (to.key().to_owned(), false)).chain(pairs.iter().map(|(from, _)| (from.key().to_owned(), true))).collect())
    }

    /// Delete a file or a folder (`a/`) of `scope`. Returns what was deleted.
    pub async fn remove(&self, scope: Scope, key: &str) -> anyhow::Result<Changes> {
        let (key, dir) = split_folder(key).ok_or_else(|| anyhow::anyhow!("not a valid key: {key:?}"))?;
        let keys = if dir { self.keys_under(scope, key).await? } else { vec![key.to_owned()] };
        for key in &keys {
            let key = ObjectKey::new(scope, key)?;
            self.store.delete(&key).await?;
            self.forget(&key).await;
        }
        Ok(keys.into_iter().map(|k| (k, true)).collect())
    }

    /// Copy a file or folder of the user's subtree into the shared catalogue. In S3 mode the working
    /// copy is written through first, so what is published is what the tools saved last.
    pub async fn publish(&self, from: &str, to: &str, overwrite: bool) -> anyhow::Result<Changes> {
        if self.mode() == StorageMode::S3
            && let Some((folder, dir)) = split_folder(from)
            && let Ok(Some(path)) = self.layout().path_of(&ObjectKey::new(Scope::User, folder)?)
        {
            if dir { self.save_tree(&path).await } else { self.save_path(&path).await }
        }
        let pairs = self.pairs(Scope::User, from, Scope::Catalogue, to).await?;
        Self::not_platform_only(pairs.iter().map(|(_, to)| to))?;
        self.free(pairs.iter().map(|(_, to)| to), overwrite).await?;
        for (from, to) in &pairs {
            self.store.copy(from, to).await?;
            self.commit(to).await?;
        }
        Ok(pairs.iter().map(|(_, to)| (to.key().to_owned(), false)).collect())
    }
}

// --- Write-through from the services ---

/// The storages [`saved`] and the others reach: the backend's (main.rs), a test's own.
static REGISTERED: Mutex<Vec<Arc<Storage>>> = Mutex::new(Vec::new());

/// While held, the write-through functions reach `storage` (dropped: no more).
#[must_use = "dropping it unregisters the storage"]
pub struct Registration(Arc<Storage>);

impl Drop for Registration {
    fn drop(&mut self) {
        REGISTERED.lock().unwrap_or_else(|e| e.into_inner()).retain(|s| !Arc::ptr_eq(s, &self.0));
    }
}

/// Let [`saved`], [`saved_tree`] and [`removed`] reach `storage` (S3 mode: locally the
/// working copy is the store, and nothing is registered).
pub fn register(storage: &Arc<Storage>) -> Registration {
    if storage.mode() == StorageMode::S3 {
        REGISTERED.lock().unwrap_or_else(|e| e.into_inner()).push(storage.clone());
    }
    Registration(storage.clone())
}

/// The registered storages whose working copy holds `path`.
fn owners_of(path: &Path) -> Vec<Arc<Storage>> {
    let registered: Vec<Arc<Storage>> = REGISTERED.lock().unwrap_or_else(|e| e.into_inner()).clone();
    registered.into_iter().filter(|s| s.key_of(path).is_some()).collect()
}

/// The file `path` was written (or removed): to the store, in the background. Errors are logged; the
/// local write stands.
pub fn saved(path: impl AsRef<Path>) {
    let path = path.as_ref();
    for storage in owners_of(path) {
        storage.spawn_save_path(path.to_owned());
    }
}

/// Every file under folder `dir` was written (a new cell, a build): to the store, in the background.
pub fn saved_tree(dir: impl AsRef<Path>) {
    let dir = dir.as_ref();
    for storage in owners_of(dir) {
        storage.spawn_save_tree(dir.to_owned());
    }
}

/// The file or folder `path` was removed: its objects go too ([`Storage::prune`]), in the background.
pub fn removed(path: impl AsRef<Path>) {
    let path = path.as_ref().to_owned();
    for storage in owners_of(&path) {
        let path = path.clone();
        storage.spawn_work(&path.clone(), |this| async move { this.prune(&path).await });
    }
}

/// Keys changed by an operation, `(key, deleted)` each.
pub type Changes = Vec<(String, bool)>;

/// Something is in the way (`409`).
#[derive(Debug)]
pub struct Conflict(pub String);

impl std::fmt::Display for Conflict {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Conflict {}

/// A batch would write the cell-templates CI's folder (`403`).
#[derive(Debug)]
pub struct PlatformOnly(pub String);

impl std::fmt::Display for PlatformOnly {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for PlatformOnly {}

/// The regular files under `dir` (no symlinks followed), skipping [`SKIPPED_DIRS`] and staging folders.
fn walk_files(dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let mut stack = vec![dir.to_owned()];
    while let Some(d) = stack.pop() {
        let Ok(read) = std::fs::read_dir(&d) else { continue };
        for entry in read.flatten() {
            let Ok(meta) = entry.path().symlink_metadata() else { continue };
            let name = entry.file_name().to_string_lossy().into_owned();
            if meta.is_dir() && !SKIPPED_DIRS.contains(&name.as_str()) && !name.starts_with(".staging") {
                stack.push(entry.path());
            } else if meta.is_file() {
                out.push(entry.path());
            }
        }
    }
    out.sort();
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn object_keys_are_scoped() {
        let user = UserId::new("jdoe").unwrap();
        let key = ObjectKey::new(Scope::User, "products/box/product.json").unwrap();
        assert_eq!(key.object_key("", &user), "users/jdoe/products/box/product.json");
        assert_eq!(key.object_key("dev/", &user), "dev/users/jdoe/products/box/product.json");
        let cat = ObjectKey::new(Scope::Catalogue, "urdf-robot-collection/a.urdf").unwrap();
        assert_eq!(cat.object_key("", &user), "catalogue/urdf-robot-collection/a.urdf");
        for bad in ["../other/x", "products/../../x", "/abs", "a//b", ""] {
            assert!(ObjectKey::new(Scope::User, bad).is_err(), "{bad}");
        }
        for bad in ["", "..", "a/b", "a b", "../x"] {
            assert!(UserId::new(bad).is_err(), "{bad}");
        }
    }
}
