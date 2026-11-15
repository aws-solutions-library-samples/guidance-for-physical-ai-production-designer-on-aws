//! Keeping every workspace's catalogue in step with the shared `catalogue/`, and its viewers told.
//!
//! **The working copy is a cache.** The local catalogue folder holds the shared catalogue's files
//! (and, before S3, the starter's). In S3 mode this backend makes it follow `catalogue/`: additions,
//! overwrites and deletes. The user's own additions (`users/<u>/catalogue/…`, the same folder on disk)
//! are an overlay on top: a key the user has is never overwritten or deleted by the sync. A delete
//! only touches files the sync itself brought (its manifest), never one it doesn't know (a starter
//! file, a file the user just wrote).
//!
//! **Change log, polled.** Every backend that changes the shared catalogue also bumps
//! `catalogue/.revision` ([`CatalogueRevision`]: a counter and the last [`MAX_CHANGES`] changed keys),
//! a read-modify-write made safe by S3's conditional writes (`If-Match`). Every backend polls it with
//! a conditional GET (`If-None-Match`: a `304`, no body, while nothing changed) every
//! [`POLL_EVERY`], applies the changes since the revision it has, and lists the whole catalogue every
//! [`RECONCILE_EVERY`] (and when it fell further behind than the log reaches) to catch what changed
//! outside Factory OS (the console, the CLI). Polling needs no inbound path to the workspace, no
//! infrastructure per workspace and nothing that can fall out of sync with the workspaces' lifecycle;
//! about 4,300 conditional GETs a day per running workspace cost well under a cent.
//!
//! **Files in use.** A file is replaced by renaming a complete one over it, so a reader (the dresser,
//! the planner) either has the old file open or gets the new one. Deletes wait while anything holds
//! an [`InUse`] guard ([`in_use`]) and are applied by a later tick.
//!
//! **Viewers.** Every change (synced here or made here, in either mode) is announced on
//! [`keys::CATALOGUE_CHANGED`] ([`CatalogueChanged`]) and to the backend's own listeners (the Robo
//! Fitter's palette, the Robo Dresser's catalogue); viewers list again.

use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use factoryos_protocol::storage::{CatalogueChange, CatalogueChanged, CatalogueRevision, REVISION_KEY, Scope, StorageMode, valid_key};
use serde::{Deserialize, Serialize};

use super::{Fetched, ObjectKey, PutOutcome, Storage, local};

/// How often the change log is polled.
pub const POLL_EVERY: Duration = Duration::from_secs(20);
/// How often the whole catalogue is listed.
pub const RECONCILE_EVERY: Duration = Duration::from_secs(600);
/// Changes kept in the log.
pub const MAX_CHANGES: usize = 500;
/// Keys a [`CatalogueChanged`] names at most.
const MAX_ANNOUNCED_KEYS: usize = 100;

static IN_USE: AtomicUsize = AtomicUsize::new(0);

/// While held, the sync deletes no catalogue files (a run reads them).
pub struct InUse(());

impl Drop for InUse {
    fn drop(&mut self) {
        IN_USE.fetch_sub(1, Ordering::SeqCst);
    }
}

/// Hold this while a job reads catalogue files (a dress, a planner run).
pub fn in_use() -> InUse {
    IN_USE.fetch_add(1, Ordering::SeqCst);
    InUse(())
}


type Listener = Box<dyn Fn(&CatalogueChanged) + Send + Sync>;

/// What the sync remembers across restarts (`catalogue-sync.json` in the state directory).
#[derive(Default, Serialize, Deserialize)]
struct Saved {
    revision: u64,
    etag: Option<String>,
    /// The shared files the sync brought into the working copy: key → their ETag then.
    manifest: BTreeMap<String, String>,
}

#[derive(Default)]
struct State {
    saved: Saved,
    /// Deletes waiting for no [`InUse`].
    pending_deletes: BTreeSet<String>,
}

pub struct CatalogueSync {
    storage: Arc<Storage>,
    state: tokio::sync::Mutex<State>,
    file: Option<PathBuf>,
    listeners: Mutex<Vec<Listener>>,
    /// [`in_use`]'s count (another one in tests, which run side by side).
    in_use: &'static AtomicUsize,
}

impl CatalogueSync {
    /// `state_dir`: where the manifest is kept (`None`: memory only).
    pub fn new(storage: Arc<Storage>, state_dir: Option<PathBuf>) -> Arc<Self> {
        let file = state_dir.map(|dir| dir.join("catalogue-sync.json"));
        let saved = file.as_ref().and_then(|f| std::fs::read(f).ok()).and_then(|b| serde_json::from_slice(&b).ok()).unwrap_or_default();
        Arc::new(Self { storage, state: tokio::sync::Mutex::new(State { saved, ..State::default() }), file, listeners: Mutex::default(), in_use: &IN_USE })
    }

    fn busy(&self) -> bool {
        self.in_use.load(Ordering::SeqCst) > 0
    }

    /// Call `listener` on every change (the bus, the fitter's palette, …).
    pub fn on_change(&self, listener: impl Fn(&CatalogueChanged) + Send + Sync + 'static) {
        self.listeners.lock().unwrap().push(Box::new(listener));
    }

    fn announce(&self, revision: u64, keys: Vec<String>, remote: bool) {
        let mut keys = keys;
        keys.truncate(MAX_ANNOUNCED_KEYS);
        let changed = CatalogueChanged { revision, keys, remote };
        for listener in self.listeners.lock().unwrap().iter() {
            listener(&changed);
        }
    }

    fn save(&self, state: &State) {
        let Some(file) = &self.file else { return };
        match serde_json::to_vec(&state.saved) {
            Ok(json) => {
                if let Err(err) = local::replace_file(file, &json) {
                    tracing::warn!("catalogue sync: {}: {err:#}", file.display());
                }
            }
            Err(err) => tracing::warn!("catalogue sync: {err}"),
        }
    }

    fn revision_key() -> ObjectKey {
        ObjectKey::new(Scope::Catalogue, REVISION_KEY).expect("a valid key")
    }

    /// This backend changed the shared catalogue (`(key, deleted)` each): in S3 mode the change log
    /// is bumped, so the other workspaces follow; in either mode the viewers are told.
    pub async fn record_local(&self, changes: Vec<(String, bool)>) {
        if changes.is_empty() {
            return;
        }
        let keys: Vec<String> = changes.iter().map(|(k, _)| k.clone()).collect();
        if self.storage.mode() != StorageMode::S3 {
            self.announce(0, keys, false);
            return;
        }
        let revision = match self.bump(&changes).await {
            Ok(revision) => revision,
            Err(err) => {
                // The others still catch up by their next full listing.
                tracing::warn!("catalogue sync: bumping {REVISION_KEY}: {err:#}");
                0
            }
        };
        self.announce(revision, keys, false);
    }

    /// Append `changes` to the change log (retrying when another backend wrote it meanwhile); the
    /// files' own new versions go into the manifest, so this backend doesn't fetch what it wrote.
    async fn bump(&self, changes: &[(String, bool)]) -> anyhow::Result<u64> {
        let store = self.storage.store();
        let key = Self::revision_key();
        for _ in 0..8 {
            let (mut log, etag) = match store.get_if(&key, None).await? {
                Fetched::Found { bytes, etag } => (serde_json::from_slice::<CatalogueRevision>(&bytes).unwrap_or_default(), etag),
                Fetched::Missing | Fetched::NotModified => (CatalogueRevision::default(), None),
            };
            let before = log.revision;
            log.revision += 1;
            log.changes.extend(changes.iter().map(|(k, deleted)| CatalogueChange { rev: log.revision, key: k.clone(), deleted: *deleted }));
            let drop = log.changes.len().saturating_sub(MAX_CHANGES);
            log.changes.drain(..drop);
            match store.put_if(&key, serde_json::to_vec(&log)?, etag.as_deref()).await? {
                PutOutcome::Conflict => continue,
                PutOutcome::Written(new_etag) => {
                    let mut state = self.state.lock().await;
                    for (k, deleted) in changes {
                        if *deleted {
                            state.saved.manifest.remove(k);
                        } else if let Ok(object) = ObjectKey::new(Scope::Catalogue, k)
                            && let Ok(Some(info)) = store.head(&object).await
                            && let Some(tag) = info.etag
                        {
                            state.saved.manifest.insert(k.clone(), tag);
                        }
                    }
                    // Up to date before: so after it too (else the next poll applies what came between).
                    if state.saved.revision == before {
                        state.saved.revision = log.revision;
                        state.saved.etag = new_etag;
                    }
                    self.save(&state);
                    return Ok(log.revision);
                }
            }
        }
        anyhow::bail!("{REVISION_KEY} kept changing; given up")
    }

    /// The user's own additions (keys of the shared catalogue's key space).
    async fn overlay(&self) -> anyhow::Result<HashSet<String>> {
        let list = self.storage.store().list(Scope::User, "catalogue/").await?;
        // As files: a user's scene under the old folder's key is the same file as under scenes/.
        Ok(list.objects.into_iter().filter_map(|o| o.key.strip_prefix("catalogue/").map(local::catalogue_rel)).collect())
    }

    /// Bring shared `key` into the working copy (unless the user has their own). Returns whether it changed.
    async fn fetch(&self, state: &mut State, overlay: &HashSet<String>, key: &str) -> anyhow::Result<bool> {
        if overlay.contains(&local::catalogue_rel(key)) || key == REVISION_KEY {
            return Ok(false);
        }
        if planner_cache(key) {
            // Not mirrored; a copy an earlier sync brought is removed.
            return self.delete(state, overlay, key);
        }
        let object = ObjectKey::new(Scope::Catalogue, key)?;
        let Some(path) = self.storage.layout().path_of(&object)? else { return Ok(false) };
        match self.storage.store().get_if(&object, None).await? {
            Fetched::Found { bytes, etag } => {
                local::replace_file(&path, &bytes)?;
                state.pending_deletes.remove(key);
                state.saved.manifest.insert(key.to_owned(), etag.unwrap_or_default());
                Ok(true)
            }
            // Gone again meanwhile: a delete.
            Fetched::Missing | Fetched::NotModified => self.delete(state, overlay, key),
        }
    }

    /// Delete shared `key` from the working copy, if the sync brought it and the user has no own.
    fn delete(&self, state: &mut State, overlay: &HashSet<String>, key: &str) -> anyhow::Result<bool> {
        if overlay.contains(&local::catalogue_rel(key)) || !state.saved.manifest.contains_key(key) {
            state.saved.manifest.remove(key);
            return Ok(false);
        }
        if self.busy() {
            state.pending_deletes.insert(key.to_owned());
            return Ok(false);
        }
        let object = ObjectKey::new(Scope::Catalogue, key)?;
        if let Some(path) = self.storage.layout().path_of(&object)? {
            match std::fs::remove_file(&path) {
                Err(err) if err.kind() != std::io::ErrorKind::NotFound => return Err(err.into()),
                _ => {}
            }
        }
        state.saved.manifest.remove(key);
        state.pending_deletes.remove(key);
        Ok(true)
    }

    /// One poll of the change log: a no-op (one `304`) while it is unchanged. Returns the files changed.
    pub async fn poll(&self) -> anyhow::Result<usize> {
        if self.storage.mode() != StorageMode::S3 {
            return Ok(0);
        }
        let mut state = self.state.lock().await;
        let mut changed = self.retry_deletes(&mut state).await?;
        let known = state.saved.etag.clone();
        let (log, etag) = match self.storage.store().get_if(&Self::revision_key(), known.as_deref()).await? {
            Fetched::NotModified => return Ok(changed),
            Fetched::Missing => return Ok(changed),
            Fetched::Found { bytes, etag } => (serde_json::from_slice::<CatalogueRevision>(&bytes)?, etag),
        };
        let have = state.saved.revision;
        if log.revision <= have {
            state.saved.etag = etag;
            self.save(&state);
            return Ok(changed);
        }
        // Further behind than the log reaches (or never synced): list everything instead.
        let reaches = have > 0 && log.changes.first().is_some_and(|c| c.rev <= have + 1);
        if !reaches {
            drop(state);
            return self.reconcile().await.map(|n| n + changed);
        }
        let mut latest: BTreeMap<&str, bool> = BTreeMap::new();
        // The log is written by other backends: a key that isn't one (e.g. `../x`) is skipped, never followed.
        for change in log.changes.iter().filter(|c| c.rev > have && valid_key(&c.key)) {
            latest.insert(&change.key, change.deleted);
        }
        let overlay = self.overlay().await?;
        let mut keys = Vec::new();
        for (key, deleted) in latest {
            let did = if deleted { self.delete(&mut state, &overlay, key)? } else { self.fetch(&mut state, &overlay, key).await? };
            if did {
                keys.push(key.to_owned());
            }
        }
        changed += keys.len();
        state.saved.revision = log.revision;
        state.saved.etag = etag;
        self.save(&state);
        drop(state);
        if !keys.is_empty() {
            self.announce(log.revision, keys, true);
        }
        Ok(changed)
    }

    /// List the whole shared catalogue and make the working copy follow it (out-of-band changes,
    /// a backend that fell behind, the first start). Returns the files changed.
    pub async fn reconcile(&self) -> anyhow::Result<usize> {
        if self.storage.mode() != StorageMode::S3 {
            return Ok(0);
        }
        let store = self.storage.store();
        // The log first: what it says now is what this listing covers.
        let (revision, etag) = match store.get_if(&Self::revision_key(), None).await? {
            Fetched::Found { bytes, etag } => (serde_json::from_slice::<CatalogueRevision>(&bytes).map(|l| l.revision).unwrap_or(0), etag),
            _ => (0, None),
        };
        let list = store.list(Scope::Catalogue, "").await?;
        if list.truncated {
            tracing::warn!("catalogue sync: the shared catalogue has more files than one listing gives; the rest follow by the change log only");
        }
        let overlay = self.overlay().await?;
        let mut state = self.state.lock().await;
        let mut keys = Vec::new();
        let mut remote = HashSet::new();
        for object in list.objects.iter().filter(|o| o.key != REVISION_KEY) {
            remote.insert(object.key.clone());
            let Ok(key) = ObjectKey::new(Scope::Catalogue, &object.key) else { continue };
            let local_missing = self.storage.layout().path_of(&key).ok().flatten().is_some_and(|p| !p.exists());
            let stale = state.saved.manifest.get(&object.key) != object.etag.as_ref();
            if (stale || local_missing) && self.fetch(&mut state, &overlay, &object.key).await? {
                keys.push(object.key.clone());
            }
        }
        if !list.truncated {
            let gone: Vec<String> = state.saved.manifest.keys().filter(|k| !remote.contains(*k)).cloned().collect();
            for key in gone {
                if self.delete(&mut state, &overlay, &key)? {
                    keys.push(key);
                }
            }
        }
        if revision >= state.saved.revision {
            state.saved.revision = revision;
            state.saved.etag = etag;
        }
        self.save(&state);
        drop(state);
        if !keys.is_empty() {
            self.announce(revision, keys.clone(), true);
        }
        Ok(keys.len())
    }

    async fn retry_deletes(&self, state: &mut State) -> anyhow::Result<usize> {
        if state.pending_deletes.is_empty() || self.busy() {
            return Ok(0);
        }
        let overlay = self.overlay().await?;
        let mut done = 0;
        for key in std::mem::take(&mut state.pending_deletes) {
            if self.delete(state, &overlay, &key)? {
                done += 1;
            }
        }
        Ok(done)
    }

    /// Poll and reconcile for good (S3 mode; nothing to do locally).
    pub fn spawn(self: &Arc<Self>) {
        if self.storage.mode() != StorageMode::S3 {
            return;
        }
        let this = self.clone();
        tokio::spawn(async move {
            let mut since_reconcile = Duration::MAX;
            loop {
                let result = if since_reconcile >= RECONCILE_EVERY {
                    since_reconcile = Duration::ZERO;
                    this.reconcile().await
                } else {
                    this.poll().await
                };
                match result {
                    Ok(0) => {}
                    Ok(n) => tracing::info!("Catalogue sync: {n} file(s) from the shared catalogue"),
                    Err(err) => tracing::warn!("Catalogue sync: {err:#}"),
                }
                tokio::time::sleep(POLL_EVERY).await;
                since_reconcile = since_reconcile.saturating_add(POLL_EVERY);
            }
        });
    }
}

/// A planner cache bundle in the shared catalogue (`catalogue/<name>-cache/…`: the planner router's
/// `u3-plan-cache/`, `umarell-jam-cache/`). The router pulls those itself, into its own cache
/// (deploy/planner-router/catalogue_cache.py), so the sync does not mirror them into every working
/// copy, where they would sit on disk twice and show in the IDE's shared catalogue.
fn planner_cache(key: &str) -> bool {
    key.split_once('/').is_some_and(|(top, _)| top.ends_with("-cache"))
}

#[cfg(test)]
pub(crate) mod tests_support {
    use super::*;
    use crate::storage::Store;
    use factoryos_protocol::storage::{Method, ObjectInfo, ObjectList};
    use futures::future::BoxFuture;
    use std::time::SystemTime;

    /// An S3 stand-in in memory: objects with ETags, conditional reads and writes, and a count of GETs.
    #[derive(Default)]
    pub struct FakeS3 {
        objects: Mutex<BTreeMap<String, (Vec<u8>, String)>>,
        next: AtomicUsize,
        pub gets: AtomicUsize,
        /// Unreachable: every request fails.
        pub down: std::sync::atomic::AtomicBool,
    }

    impl FakeS3 {
        fn full(key: &ObjectKey) -> String {
            format!("{}/{}", key.scope.as_str(), key.key())
        }
        /// Out of band (the console): no change log.
        pub fn set(&self, scope: Scope, key: &str, bytes: &[u8]) {
            let tag = format!("\"e{}\"", self.next.fetch_add(1, Ordering::SeqCst));
            self.objects.lock().unwrap().insert(format!("{}/{key}", scope.as_str()), (bytes.to_vec(), tag));
        }
        fn reachable(&self) -> anyhow::Result<()> {
            anyhow::ensure!(!self.down.load(Ordering::SeqCst), "S3 unreachable (fake)");
            Ok(())
        }
        pub fn remove(&self, scope: Scope, key: &str) {
            self.objects.lock().unwrap().remove(&format!("{}/{key}", scope.as_str()));
        }
        /// The bytes of `key`, if there is such an object.
        pub fn bytes(&self, scope: Scope, key: &str) -> Option<Vec<u8>> {
            self.objects.lock().unwrap().get(&format!("{}/{key}", scope.as_str())).map(|(b, _)| b.clone())
        }
        /// The keys of `scope`, sorted.
        pub fn keys(&self, scope: Scope) -> Vec<String> {
            let base = format!("{}/", scope.as_str());
            self.objects.lock().unwrap().keys().filter_map(|k| k.strip_prefix(&base).map(str::to_owned)).collect()
        }
    }

    /// A user's working copy in a fresh temporary folder `<tmp>/factoryos-<name>-<pid>-<n>/` with
    /// `products/`, `cells/` and `catalogue/` (canonical), over a fake S3, registered for the
    /// write-through ([`crate::storage::saved`] and the others) while the registration is held.
    pub fn user_storage(name: &str) -> (PathBuf, Arc<FakeS3>, Arc<Storage>, crate::storage::Registration) {
        static N: AtomicUsize = AtomicUsize::new(0);
        let root = std::env::temp_dir().join(format!("factoryos-{name}-{}-{}", std::process::id(), N.fetch_add(1, Ordering::SeqCst)));
        std::fs::remove_dir_all(&root).ok();
        for dir in ["products", "cells", "catalogue"] {
            std::fs::create_dir_all(root.join(dir)).unwrap();
        }
        let root = std::fs::canonicalize(&root).unwrap();
        let (s3, storage) = storage_at(&root);
        let registered = crate::storage::register(&storage);
        (root, s3, storage, registered)
    }

    /// A storage over a new fake S3 whose working copy is `root`'s `products/`, `cells/` and
    /// `catalogue/` (not registered).
    pub fn storage_at(root: &std::path::Path) -> (Arc<FakeS3>, Arc<Storage>) {
        let s3 = Arc::new(FakeS3::default());
        (s3.clone(), storage_over(root, s3))
    }

    /// [`storage_at`] over `s3`.
    pub fn storage_over(root: &std::path::Path, s3: Arc<FakeS3>) -> Arc<Storage> {
        storage_of(s3, root.join("products"), root.join("cells"), root.join("catalogue"))
    }

    /// A storage over `s3` whose working copy is these folders.
    pub fn storage_of(s3: Arc<FakeS3>, products: PathBuf, cells: PathBuf, catalogue: PathBuf) -> Arc<Storage> {
        let layout = move || local::Layout {
            user: vec![("products".into(), products.clone()), ("cells".into(), cells.clone()), ("catalogue".into(), catalogue.clone())],
            catalogue: Some(catalogue.clone()),
        };
        Storage::with_store(s3, crate::storage::UserId::new("jdoe").unwrap(), layout)
    }

    /// Wait (up to 5 s) until `done` holds: the write-through runs in the background.
    pub async fn eventually(what: &str, mut done: impl FnMut() -> bool) {
        for _ in 0..500 {
            if done() {
                return;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        panic!("not written through in time: {what}");
    }

    impl Store for FakeS3 {
        fn mode(&self) -> StorageMode {
            StorageMode::S3
        }
        fn get<'a>(&'a self, key: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<Option<Vec<u8>>>> {
            Box::pin(async move {
                self.reachable()?;
                Ok(self.objects.lock().unwrap().get(&Self::full(key)).map(|(b, _)| b.clone()))
            })
        }
        fn head<'a>(&'a self, key: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<Option<ObjectInfo>>> {
            Box::pin(async move {
                self.reachable()?;
                Ok(self.objects.lock().unwrap().get(&Self::full(key)).map(|(b, e)| ObjectInfo { key: key.key().into(), bytes: b.len() as u64, etag: Some(e.clone()) }))
            })
        }
        fn get_if<'a>(&'a self, key: &'a ObjectKey, if_none_match: Option<&'a str>) -> BoxFuture<'a, anyhow::Result<Fetched>> {
            Box::pin(async move {
                self.reachable()?;
                self.gets.fetch_add(1, Ordering::SeqCst);
                Ok(match self.objects.lock().unwrap().get(&Self::full(key)) {
                    None => Fetched::Missing,
                    Some((_, e)) if Some(e.as_str()) == if_none_match => Fetched::NotModified,
                    Some((b, e)) => Fetched::Found { bytes: b.clone(), etag: Some(e.clone()) },
                })
            })
        }
        fn put<'a>(&'a self, key: &'a ObjectKey, bytes: Vec<u8>) -> BoxFuture<'a, anyhow::Result<()>> {
            Box::pin(async move {
                self.reachable()?;
                self.set(key.scope, key.key(), &bytes);
                Ok(())
            })
        }
        fn put_if<'a>(&'a self, key: &'a ObjectKey, bytes: Vec<u8>, if_match: Option<&'a str>) -> BoxFuture<'a, anyhow::Result<PutOutcome>> {
            Box::pin(async move {
                self.reachable()?;
                let now = self.objects.lock().unwrap().get(&Self::full(key)).map(|(_, e)| e.clone());
                if now.as_deref() != if_match {
                    return Ok(PutOutcome::Conflict);
                }
                self.set(key.scope, key.key(), &bytes);
                Ok(PutOutcome::Written(self.objects.lock().unwrap().get(&Self::full(key)).map(|(_, e)| e.clone())))
            })
        }
        fn delete<'a>(&'a self, key: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<()>> {
            Box::pin(async move {
                self.reachable()?;
                self.remove(key.scope, key.key());
                Ok(())
            })
        }
        fn copy<'a>(&'a self, from: &'a ObjectKey, to: &'a ObjectKey) -> BoxFuture<'a, anyhow::Result<()>> {
            Box::pin(async move {
                self.reachable()?;
                let bytes = self.objects.lock().unwrap().get(&Self::full(from)).map(|(b, _)| b.clone()).unwrap_or_default();
                self.set(to.scope, to.key(), &bytes);
                Ok(())
            })
        }
        fn list<'a>(&'a self, scope: Scope, prefix: &'a str) -> BoxFuture<'a, anyhow::Result<ObjectList>> {
            Box::pin(async move {
                self.reachable()?;
                let base = format!("{}/", scope.as_str());
                let objects = self
                    .objects
                    .lock()
                    .unwrap()
                    .iter()
                    .filter_map(|(k, (b, e))| {
                        let rel = k.strip_prefix(&base)?;
                        rel.starts_with(prefix).then(|| ObjectInfo { key: rel.to_owned(), bytes: b.len() as u64, etag: Some(e.clone()) })
                    })
                    .collect();
                Ok(ObjectList { objects, truncated: false })
            })
        }
        fn presign<'a>(&'a self, _: &'a ObjectKey, _: Method, _: Duration) -> BoxFuture<'a, anyhow::Result<Option<(String, SystemTime)>>> {
            Box::pin(async { Ok(None) })
        }
    }

}

#[cfg(test)]
mod tests {
    use super::tests_support::FakeS3;
    use super::*;
    use crate::storage::{Store, UserId};

    struct World {
        s3: Arc<FakeS3>,
        root: PathBuf,
        /// Two workspaces' syncs over the same bucket: `a` writes, `b` follows.
        a: Arc<CatalogueSync>,
        b: Arc<CatalogueSync>,
        b_cat: PathBuf,
        announced: Arc<Mutex<Vec<CatalogueChanged>>>,
    }

    fn world(name: &str) -> World {
        let root = std::env::temp_dir().join(format!("factoryos-catalogue-sync-{name}-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let s3 = Arc::new(FakeS3::default());
        let sync = |ws: &str| {
            let cat = root.join(ws).join("catalogue");
            std::fs::create_dir_all(&cat).unwrap();
            let cat = std::fs::canonicalize(cat).unwrap();
            let layout = {
                let cat = cat.clone();
                move || local::Layout { user: vec![("catalogue".into(), cat.clone())], catalogue: Some(cat.clone()) }
            };
            let storage = Storage::with_store(s3.clone(), UserId::new(ws).unwrap(), layout);
            let mut sync = CatalogueSync::new(storage, Some(root.join(ws)));
            Arc::get_mut(&mut sync).unwrap().in_use = Box::leak(Box::new(AtomicUsize::new(0)));
            (sync, cat)
        };
        let ((a, _), (b, b_cat)) = (sync("alice"), sync("bob"));
        let announced = Arc::new(Mutex::new(Vec::new()));
        let sink = announced.clone();
        b.on_change(move |c| sink.lock().unwrap().push(c.clone()));
        World { s3, root, a, b, b_cat, announced }
    }

    /// `a`'s backend writes a shared file (what storage::http does), then records it.
    async fn shared_write(w: &World, key: &str, bytes: &[u8]) {
        w.s3.set(Scope::Catalogue, key, bytes);
        w.a.record_local(vec![(key.to_owned(), false)]).await;
    }

    #[tokio::test]
    async fn deltas_add_overwrite_and_delete() {
        let w = world("delta");
        // First start: a full listing (nothing yet), at revision 0.
        w.b.reconcile().await.unwrap();
        shared_write(&w, "robots/a.urdf", b"1").await;
        // Bob is at revision 0, which the log doesn't reach: lists everything.
        assert_eq!(w.b.poll().await.unwrap(), 1);
        assert_eq!(std::fs::read(w.b_cat.join("robots/a.urdf")).unwrap(), b"1");
        shared_write(&w, "robots/a.urdf", b"2").await;
        shared_write(&w, "tools/t.urdf", b"t").await;
        assert_eq!(w.b.poll().await.unwrap(), 2);
        assert_eq!(std::fs::read(w.b_cat.join("robots/a.urdf")).unwrap(), b"2");
        assert!(w.b_cat.join("tools/t.urdf").is_file());
        w.s3.remove(Scope::Catalogue, "robots/a.urdf");
        w.a.record_local(vec![("robots/a.urdf".to_owned(), true)]).await;
        assert_eq!(w.b.poll().await.unwrap(), 1);
        assert!(!w.b_cat.join("robots/a.urdf").exists());
        let last = w.announced.lock().unwrap().last().cloned().unwrap();
        assert!(last.remote && last.keys == ["robots/a.urdf"] && last.revision == 4, "{last:?}");
        std::fs::remove_dir_all(&w.root).ok();
    }

    #[tokio::test]
    async fn the_users_own_additions_are_never_clobbered() {
        let w = world("overlay");
        w.b.reconcile().await.unwrap();
        shared_write(&w, "x.urdf", b"shared").await;
        w.b.poll().await.unwrap();
        // Bob has his own x.urdf (users/bob/catalogue/x.urdf) and a local file the sync doesn't know.
        w.s3.set(Scope::User, "catalogue/x.urdf", b"mine");
        std::fs::write(w.b_cat.join("x.urdf"), "mine").unwrap();
        std::fs::write(w.b_cat.join("starter.urdf"), "starter").unwrap();
        shared_write(&w, "x.urdf", b"shared 2").await;
        w.b.poll().await.unwrap();
        assert_eq!(std::fs::read(w.b_cat.join("x.urdf")).unwrap(), b"mine");
        w.s3.remove(Scope::Catalogue, "x.urdf");
        w.a.record_local(vec![("x.urdf".to_owned(), true)]).await;
        w.b.poll().await.unwrap();
        w.b.reconcile().await.unwrap();
        assert_eq!(std::fs::read(w.b_cat.join("x.urdf")).unwrap(), b"mine");
        // Never brought by the sync: never deleted by it.
        assert!(w.b_cat.join("starter.urdf").is_file());
        std::fs::remove_dir_all(&w.root).ok();
    }

    #[tokio::test]
    async fn shared_flat_scenes_land_in_their_folder_unless_the_user_has_one() {
        let w = world("flat-scenes");
        w.b.reconcile().await.unwrap();
        // A shared scene still under a flat key: its folder's file, never beside it.
        shared_write(&w, "fitted-robots/shared.yaml", b"name: shared").await;
        w.b.poll().await.unwrap();
        assert_eq!(std::fs::read(w.b_cat.join("scenes/shared/scene.yaml")).unwrap(), b"name: shared");
        assert!(!w.b_cat.join("scenes/shared.yaml").exists() && !w.b_cat.join("fitted-robots").exists());
        // Bob's own (migrated) scene of that name: the shared flat one never overwrites it.
        w.s3.set(Scope::User, "catalogue/scenes/mine/scene.yaml", b"name: mine");
        std::fs::create_dir_all(w.b_cat.join("scenes/mine")).unwrap();
        std::fs::write(w.b_cat.join("scenes/mine/scene.yaml"), "name: mine").unwrap();
        shared_write(&w, "scenes/mine.yaml", b"name: shared mine").await;
        w.b.poll().await.unwrap();
        w.b.reconcile().await.unwrap();
        assert_eq!(std::fs::read(w.b_cat.join("scenes/mine/scene.yaml")).unwrap(), b"name: mine");
        assert!(!w.b_cat.join("scenes/mine.yaml").exists());
        std::fs::remove_dir_all(&w.root).ok();
    }

    #[tokio::test]
    async fn keys_in_the_log_that_are_not_keys_are_skipped() {
        let w = world("bad-keys");
        w.b.reconcile().await.unwrap();
        shared_write(&w, "ok.urdf", b"1").await;
        w.b.poll().await.unwrap();
        w.a.record_local(vec![("../../escape".to_owned(), false), ("a//b".to_owned(), true), ("ok2.urdf".to_owned(), false)]).await;
        w.s3.set(Scope::Catalogue, "ok2.urdf", b"2");
        assert_eq!(w.b.poll().await.unwrap(), 1);
        assert!(w.b_cat.join("ok2.urdf").is_file() && !w.root.join("escape").exists());
        std::fs::remove_dir_all(&w.root).ok();
    }

    /// What infra/lambda/catalogue-change-log writes for files changed outside Factory OS (the
    /// console, `aws s3 cp`): the same log, one revision per invocation. Applied as any other entry,
    /// and an entry logged twice (by a backend and the Lambda) is harmless: a poll applies each key
    /// once by its latest entry, a fetch writes what the object is now, a delete of a file it doesn't
    /// have does nothing.
    #[tokio::test]
    async fn entries_the_change_log_lambda_writes_are_applied() {
        let w = world("lambda");
        shared_write(&w, "a.urdf", b"1").await;
        w.b.poll().await.unwrap();
        w.b.reconcile().await.unwrap();
        // `aws s3 cp` of two files and `aws s3 rm` of a.urdf, then the Lambda's single write (as it
        // serialises it: JSON.stringify of {revision, changes: [{rev, key, deleted}]}).
        w.s3.set(Scope::Catalogue, "tools/new tool.urdf", b"t");
        w.s3.set(Scope::Catalogue, "robots/r.urdf", b"r");
        w.s3.remove(Scope::Catalogue, "a.urdf");
        let lambda_log = r#"{"revision":2,"changes":[{"rev":1,"key":"a.urdf","deleted":false},{"rev":2,"key":"tools/new tool.urdf","deleted":false},{"rev":2,"key":"robots/r.urdf","deleted":false},{"rev":2,"key":"a.urdf","deleted":true},{"rev":2,"key":"robots/r.urdf","deleted":false}]}"#;
        w.s3.set(Scope::Catalogue, REVISION_KEY, lambda_log.as_bytes());
        assert_eq!(w.b.poll().await.unwrap(), 3);
        assert_eq!(std::fs::read(w.b_cat.join("tools/new tool.urdf")).unwrap(), b"t");
        assert!(w.b_cat.join("robots/r.urdf").is_file() && !w.b_cat.join("a.urdf").exists());
        // The backend's own log write after the Lambda's (the same change twice): nothing more to do.
        w.a.record_local(vec![("robots/r.urdf".to_owned(), false), ("a.urdf".to_owned(), true)]).await;
        let log: CatalogueRevision = serde_json::from_slice(&w.s3.get(&CatalogueSync::revision_key()).await.unwrap().unwrap()).unwrap();
        assert_eq!(log.revision, 3, "appended to the Lambda's log, monotonic");
        assert_eq!(w.b.poll().await.unwrap(), 1, "r.urdf fetched again (same bytes), a.urdf already gone");
        assert_eq!(std::fs::read(w.b_cat.join("robots/r.urdf")).unwrap(), b"r");
        std::fs::remove_dir_all(&w.root).ok();
    }

    #[tokio::test]
    async fn an_unchanged_log_is_one_conditional_get() {
        let w = world("noop");
        shared_write(&w, "a.urdf", b"1").await;
        w.b.reconcile().await.unwrap();
        let before = w.s3.gets.load(Ordering::SeqCst);
        let announced = w.announced.lock().unwrap().len();
        for _ in 0..3 {
            assert_eq!(w.b.poll().await.unwrap(), 0);
        }
        assert_eq!(w.s3.gets.load(Ordering::SeqCst) - before, 3, "one If-None-Match GET per poll");
        assert_eq!(w.announced.lock().unwrap().len(), announced);
        std::fs::remove_dir_all(&w.root).ok();
    }

    #[tokio::test]
    async fn reconcile_catches_out_of_band_changes() {
        let w = world("reconcile");
        shared_write(&w, "a.urdf", b"1").await;
        shared_write(&w, "b.urdf", b"b").await;
        w.b.reconcile().await.unwrap();
        // The console: a new file, an overwrite and a delete, no change log.
        w.s3.set(Scope::Catalogue, "c.urdf", b"c");
        w.s3.set(Scope::Catalogue, "a.urdf", b"2");
        w.s3.remove(Scope::Catalogue, "b.urdf");
        assert_eq!(w.b.poll().await.unwrap(), 0, "the log didn't change");
        assert_eq!(w.b.reconcile().await.unwrap(), 3);
        assert_eq!(std::fs::read(w.b_cat.join("a.urdf")).unwrap(), b"2");
        assert!(w.b_cat.join("c.urdf").is_file() && !w.b_cat.join("b.urdf").exists());
        // The manifest survives a restart: nothing to fetch again.
        let again = CatalogueSync::new(w.b.storage.clone(), Some(w.root.join("bob")));
        assert_eq!(again.reconcile().await.unwrap(), 0);
        std::fs::remove_dir_all(&w.root).ok();
    }

    #[tokio::test]
    async fn planner_caches_are_not_mirrored() {
        let w = world("planner-cache");
        shared_write(&w, "robots/a.urdf", b"a").await;
        shared_write(&w, "u3-plan-cache/body_weld_3.tgz", b"u3").await;
        shared_write(&w, "umarell-jam-cache/body_weld_1__32e3896b-1c905fc8.tgz", b"jam").await;
        w.b.reconcile().await.unwrap();
        assert!(w.b_cat.join("robots/a.urdf").is_file());
        assert!(!w.b_cat.join("u3-plan-cache").exists() && !w.b_cat.join("umarell-jam-cache").exists());
        shared_write(&w, "u3-plan-cache/body_weld_4x8.tgz", b"u3").await;
        assert_eq!(w.b.poll().await.unwrap(), 0, "the log names a cache bundle only");
        assert!(!w.b_cat.join("u3-plan-cache").exists());
        // A folder that only ends in -cache further down is mirrored as before.
        shared_write(&w, "tools/gun-cache/t.urdf", b"t").await;
        assert_eq!(w.b.poll().await.unwrap(), 1);
        assert!(w.b_cat.join("tools/gun-cache/t.urdf").is_file());
        std::fs::remove_dir_all(&w.root).ok();
    }

    #[test]
    fn planner_cache_keys() {
        assert!(planner_cache("u3-plan-cache/body_weld_3.tgz") && planner_cache("umarell-jam-cache/x/y.tgz"));
        assert!(!planner_cache("u3-plan-cache-cells/body_weld_3/scene.yaml") && !planner_cache("u3-plan-cache"));
        assert!(!planner_cache("robots/a-cache.urdf") && !planner_cache("tools/gun-cache/t.urdf"));
    }

    #[tokio::test]
    async fn deletes_wait_while_files_are_in_use() {
        let w = world("in-use");
        shared_write(&w, "a.urdf", b"1").await;
        w.b.reconcile().await.unwrap();
        w.s3.remove(Scope::Catalogue, "a.urdf");
        w.a.record_local(vec![("a.urdf".to_owned(), true)]).await;
        w.b.in_use.fetch_add(1, Ordering::SeqCst);
        w.b.poll().await.unwrap();
        assert!(w.b_cat.join("a.urdf").is_file(), "kept while a run reads the catalogue");
        w.b.in_use.fetch_sub(1, Ordering::SeqCst);
        assert_eq!(w.b.poll().await.unwrap(), 1);
        assert!(!w.b_cat.join("a.urdf").exists());
        std::fs::remove_dir_all(&w.root).ok();
    }

    #[tokio::test]
    async fn concurrent_writers_both_land_in_the_log() {
        let w = world("writers");
        let (a, b) = (w.a.clone(), w.b.clone());
        w.s3.set(Scope::Catalogue, "a1", b"x");
        w.s3.set(Scope::Catalogue, "b1", b"x");
        tokio::join!(a.record_local(vec![("a1".into(), false)]), b.record_local(vec![("b1".into(), false)]));
        let log: CatalogueRevision = serde_json::from_slice(&w.s3.get(&CatalogueSync::revision_key()).await.unwrap().unwrap()).unwrap();
        assert_eq!(log.revision, 2);
        let keys: BTreeSet<&str> = log.changes.iter().map(|c| c.key.as_str()).collect();
        assert_eq!(keys, ["a1", "b1"].into());
        std::fs::remove_dir_all(&w.root).ok();
    }
}
