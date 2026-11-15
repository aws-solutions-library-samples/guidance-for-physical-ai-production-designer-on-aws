//! Content-addressed assets over HTTP. Neutral: no Rerun here.
//!
//! [`AssetCache::get`] never blocks: the first call for a hash starts `GET /api/v1/assets/<sha256>`
//! ([`api::asset`]) and answers [`AssetState::Pending`]; every registered [`Waker`] is woken when it
//! is done. The bytes are checked against the hash, so whatever serves them (the backend today,
//! a CDN later) needs no trust. Kept in memory up to [`MEMORY_BUDGET`] bytes (least recently used
//! out first) and, natively, on disk under the user's cache directory (`factoryos/assets/<sha256>`).
//!
//! At most [`MAX_FETCHES`] assets are read from disk or fetched at a time (natively, each is a
//! thread); the rest wait their turn, [`Priority::High`] ones (URDFs, decimated meshes: what a
//! first frame needs) before the others. The disk cache is kept under [`DISK_BUDGET`] bytes (oldest
//! first by modification time, which a read refreshes), pruned at startup and every
//! [`PRUNE_EVERY`] bytes written.
//!
//! Assets are fetched through a [`DataSource`]: straight from S3 with a URL the backend presigns
//! when the backend keeps its files there (a deployed workspace), else from the backend
//! ([`api::asset`]). Either way the bytes are checked against the hash.
//!
//! A fetch that has not answered after [`FETCH_TIMEOUT`] fails (retryably), so a request lost
//! with a backend that went away mid-request doesn't keep its asset pending, and its slot taken,
//! forever. After a backend restart, [`AssetCache::retry_failed`] retries the failures at once.

use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use factoryos_client::{Api, DataSource};
#[cfg(doc)]
use factoryos_client::protocol::api;
use factoryos_client::protocol::files::MAX_TRANSFER_BYTES;
use factoryos_client::protocol::scene::AssetRef;
use sha2::{Digest as _, Sha256};
use web_time::Instant;

use super::{AssetSource, AssetState, Priority, Waker};

/// Bytes kept in memory, at most (assets in flight and a single bigger asset aside).
pub const MEMORY_BUDGET: usize = 512 * 1024 * 1024;
/// A failed fetch is retried by the first `get` after this long.
pub const RETRY_AFTER: Duration = Duration::from_secs(5);
/// A fetch without an answer by then fails (retryably).
pub const FETCH_TIMEOUT: Duration = Duration::from_secs(240);
/// Assets read from disk or fetched at once, at most.
pub const MAX_FETCHES: usize = 8;
/// Bytes kept in the native disk cache, at most.
pub const DISK_BUDGET: u64 = 2 * 1024 * 1024 * 1024;
/// The disk cache is pruned again after this many bytes were written to it.
pub const PRUNE_EVERY: u64 = 256 * 1024 * 1024;

type Done = Box<dyn FnOnce(Result<Vec<u8>, String>) + Send>;
/// Fetches an asset's bytes by hash (no checks: [`AssetCache`] verifies them).
type Fetch = dyn Fn(&str, Done) + Send + Sync;

pub struct AssetCache {
    inner: Arc<Inner>,
}

struct Inner {
    fetch: Box<Fetch>,
    state: Mutex<State>,
    wakers: Mutex<Vec<Waker>>,
    retry_after: Duration,
    fetch_timeout: Duration,
    /// Native only: where verified downloads are kept across runs.
    disk: Option<std::path::PathBuf>,
    /// Bytes written to `disk` since it was last pruned.
    written: std::sync::atomic::AtomicU64,
    slots: Mutex<Slots>,
}

/// Reads/fetches in progress, and those waiting for one to finish (by priority, then in order).
#[derive(Default)]
struct Slots {
    busy: usize,
    high: VecDeque<String>,
    normal: VecDeque<String>,
}

impl Slots {
    #[cfg(test)]
    fn waiting(&self) -> usize {
        self.high.len() + self.normal.len()
    }

    /// `sha256`, if it waits at normal priority, waits at high priority now.
    fn raise(&mut self, sha256: &str) {
        if let Some(i) = self.normal.iter().position(|s| s == sha256)
            && let Some(sha256) = self.normal.remove(i)
        {
            self.high.push_back(sha256);
        }
    }
}

struct State {
    entries: HashMap<String, Entry>,
    /// Bytes of the `Ready` entries.
    bytes: usize,
    budget: usize,
    /// For least-recently-used order.
    clock: u64,
}

enum Entry {
    Pending,
    Ready { bytes: Arc<[u8]>, used: u64 },
    Failed { err: String, retryable: bool, at: Instant },
}

impl AssetCache {
    pub fn new(api: Api) -> Self {
        let max_bytes = usize::try_from(MAX_TRANSFER_BYTES).unwrap_or(usize::MAX);
        let source = DataSource::new(api);
        let fetch = move |sha256: &str, done: Done| source.get_asset(sha256, max_bytes, done);
        let disk = disk_cache_dir();
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(dir) = disk.clone() {
            std::thread::Builder::new()
                .name("asset-cache-prune".into())
                .spawn(move || prune_disk(&dir, DISK_BUDGET))
                .ok();
        }
        Self::with_fetch(Box::new(fetch), MEMORY_BUDGET, RETRY_AFTER, disk)
    }

    fn with_fetch(fetch: Box<Fetch>, budget: usize, retry_after: Duration, disk: Option<std::path::PathBuf>) -> Self {
        Self {
            inner: Arc::new(Inner {
                fetch,
                state: Mutex::new(State {
                    entries: HashMap::new(),
                    bytes: 0,
                    budget,
                    clock: 0,
                }),
                wakers: Mutex::new(Vec::new()),
                retry_after,
                fetch_timeout: FETCH_TIMEOUT,
                disk,
                written: Default::default(),
                slots: Mutex::default(),
            }),
        }
    }

    /// Wake `waker` too whenever an asset arrives (one per scene sharing this cache).
    pub fn set_waker(&self, waker: Waker) {
        let mut wakers = self.inner.wakers.lock().unwrap();
        if !wakers.iter().any(|w| Arc::ptr_eq(&w.0, &waker.0)) {
            wakers.push(waker);
        }
    }

    /// Bytes held in memory.
    pub fn memory_bytes(&self) -> usize {
        self.inner.state.lock().unwrap().bytes
    }

    /// Fetch what failed retryably again on its next `get`, without waiting for [`RETRY_AFTER`]
    /// (the backend restarted: its asset index is new). Wakes the scenes to ask.
    pub fn retry_failed(&self) {
        self.inner.state.lock().unwrap().entries.retain(|_, entry| !matches!(entry, Entry::Failed { retryable: true, .. }));
        let wakers = self.inner.wakers.lock().unwrap().clone();
        for waker in wakers {
            waker.wake();
        }
    }

    #[cfg(test)]
    fn set_fetch_timeout(&mut self, timeout: Duration) {
        Arc::get_mut(&mut self.inner).expect("not shared yet").fetch_timeout = timeout;
    }
}

impl AssetSource for AssetCache {
    fn get_with(&self, asset: &AssetRef, priority: Priority) -> AssetState {
        let sha256 = asset.sha256.to_ascii_lowercase();
        if !is_sha256(&sha256) {
            return AssetState::Failed {
                err: format!("not a SHA-256: {:?}", asset.sha256),
                retryable: false,
            };
        }
        {
            let state = &mut *self.inner.state.lock().unwrap();
            state.clock += 1;
            let now = state.clock;
            match state.entries.get_mut(&sha256) {
                Some(Entry::Pending) => {
                    if priority == Priority::High {
                        self.inner.slots.lock().unwrap().raise(&sha256);
                    }
                    return AssetState::Pending;
                }
                Some(Entry::Ready { bytes, used }) => {
                    *used = now;
                    return AssetState::Ready(bytes.clone());
                }
                Some(Entry::Failed { err, retryable, at }) if at.elapsed() < self.inner.retry_after => {
                    return AssetState::Failed {
                        err: err.clone(),
                        retryable: *retryable,
                    };
                }
                _ => {}
            }
            state.entries.insert(sha256.clone(), Entry::Pending);
        }
        // Not under the lock: a fetch may finish at once.
        self.inner.clone().start(sha256, priority);
        AssetState::Pending
    }
}

/// Why a fetch failed, and whether it may succeed later.
struct Failure {
    err: String,
    retryable: bool,
}

impl Failure {
    fn retryable(err: String) -> Self {
        Self { err, retryable: true }
    }
}

impl Inner {
    /// Read or fetch `sha256` once a slot is free.
    fn start(self: Arc<Self>, sha256: String, priority: Priority) {
        {
            let slots = &mut *self.slots.lock().unwrap();
            if slots.busy >= MAX_FETCHES {
                match priority {
                    Priority::High => slots.high.push_back(sha256),
                    Priority::Normal => slots.normal.push_back(sha256),
                }
                return;
            }
            slots.busy += 1;
        }
        self.run(sha256);
    }

    /// The slot taken for an asset is free again: start the next one waiting, if any.
    fn release(self: Arc<Self>) {
        let next = {
            let slots = &mut *self.slots.lock().unwrap();
            let next = slots.high.pop_front().or_else(|| slots.normal.pop_front());
            if next.is_none() {
                slots.busy -= 1;
            }
            next
        };
        if let Some(sha256) = next {
            self.run(sha256);
        }
    }

    /// With a slot taken: the disk cache (natively), else HTTP.
    fn run(self: Arc<Self>, sha256: String) {
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(dir) = self.disk.clone() {
            // Disk reads (and hashing) off the UI thread.
            let this = self.clone();
            let key = sha256.clone();
            let spawned = std::thread::Builder::new().name("asset-cache".into()).spawn(move || {
                let path = dir.join(&key);
                if let Ok(bytes) = std::fs::read(&path) {
                    if sha256_hex(&bytes) == key {
                        touch(&path);
                        this.done(&key, Ok(bytes));
                        return;
                    }
                    std::fs::remove_file(&path).ok();
                }
                this.fetch_http(key);
            });
            if let Err(err) = spawned {
                self.done(&sha256, Err(Failure::retryable(format!("no thread to read the disk cache: {err}"))));
            }
            return;
        }
        self.fetch_http(sha256);
    }

    fn fetch_http(self: Arc<Self>, sha256: String) {
        let key = sha256.clone();
        let this = self.clone();
        let what = format!("asset {sha256}");
        (self.fetch)(
            &key,
            factoryos_client::with_deadline(self.fetch_timeout, what, Box::new(move |result| {
                let result = result.map_err(Failure::retryable).and_then(|bytes| {
                    let got = sha256_hex(&bytes);
                    if got == sha256 {
                        Ok(bytes)
                    } else {
                        Err(Failure {
                            err: format!("asset {sha256}: content hash is {got}"),
                            retryable: false,
                        })
                    }
                });
                #[cfg(not(target_arch = "wasm32"))]
                if let (Ok(bytes), Some(dir)) = (&result, &this.disk) {
                    this.write_to_disk(dir, &sha256, bytes);
                }
                this.done(&sha256, result);
            })),
        );
    }

    /// Store the result, free the slot, wake the UI.
    fn done(self: Arc<Self>, sha256: &str, result: Result<Vec<u8>, Failure>) {
        self.finish(sha256, result);
        self.release();
    }

    /// Keep `bytes` on disk; prune now and then.
    #[cfg(not(target_arch = "wasm32"))]
    fn write_to_disk(&self, dir: &std::path::Path, sha256: &str, bytes: &[u8]) {
        use std::sync::atomic::Ordering;
        if !write_atomically(dir, sha256, bytes) {
            return;
        }
        let written = self.written.fetch_add(bytes.len() as u64, Ordering::Relaxed) + bytes.len() as u64;
        if written >= PRUNE_EVERY && self.written.compare_exchange(written, 0, Ordering::Relaxed, Ordering::Relaxed).is_ok() {
            prune_disk(dir, DISK_BUDGET);
        }
    }

    fn finish(&self, sha256: &str, result: Result<Vec<u8>, Failure>) {
        {
            let state = &mut *self.state.lock().unwrap();
            let entry = match result {
                Ok(bytes) => {
                    state.clock += 1;
                    state.bytes += bytes.len();
                    Entry::Ready {
                        bytes: bytes.into(),
                        used: state.clock,
                    }
                }
                Err(Failure { err, retryable }) => {
                    crate::rr::re_log::warn!("Asset {sha256}: {err}");
                    Entry::Failed {
                        err,
                        retryable,
                        at: Instant::now(),
                    }
                }
            };
            state.entries.insert(sha256.to_owned(), entry);
            state.evict(sha256);
        }
        let wakers = self.wakers.lock().unwrap().clone();
        for waker in wakers {
            waker.wake();
        }
    }
}

impl State {
    /// Least recently used `Ready` entries out until within budget, except `keep`.
    fn evict(&mut self, keep: &str) {
        while self.bytes > self.budget {
            let oldest = self
                .entries
                .iter()
                .filter(|(k, _)| k.as_str() != keep)
                .filter_map(|(k, e)| match e {
                    Entry::Ready { used, .. } => Some((*used, k.clone())),
                    _ => None,
                })
                .min();
            let Some((_, key)) = oldest else { break };
            if let Some(Entry::Ready { bytes, .. }) = self.entries.remove(&key) {
                self.bytes -= bytes.len();
            }
        }
    }
}

fn is_sha256(s: &str) -> bool {
    s.len() == 64 && s.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

fn sha256_hex(bytes: &[u8]) -> String {
    use std::fmt::Write as _;
    Sha256::digest(bytes).iter().fold(String::with_capacity(64), |mut s, b| {
        write!(s, "{b:02x}").ok();
        s
    })
}

/// `<user cache dir>/factoryos/assets` natively; none on the web.
fn disk_cache_dir() -> Option<std::path::PathBuf> {
    #[cfg(target_arch = "wasm32")]
    {
        None
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        use std::path::PathBuf;
        let env = |name: &str| std::env::var_os(name).filter(|v| !v.is_empty()).map(PathBuf::from);
        let base = if cfg!(target_os = "macos") {
            env("HOME").map(|home| home.join("Library/Caches"))
        } else if cfg!(windows) {
            env("LOCALAPPDATA")
        } else {
            env("XDG_CACHE_HOME").or_else(|| env("HOME").map(|home| home.join(".cache")))
        }?;
        Some(base.join("factoryos").join("assets"))
    }
}

/// Write `bytes` to `dir/<sha256>` via a temporary file and a rename, so a reader never sees a
/// partial file. Failures only cost the disk cache. Returns whether a file was written.
#[cfg(not(target_arch = "wasm32"))]
fn write_atomically(dir: &std::path::Path, sha256: &str, bytes: &[u8]) -> bool {
    let path = dir.join(sha256);
    if path.exists() {
        return false;
    }
    let tmp = dir.join(format!(".{sha256}.{}.{:?}.tmp", std::process::id(), std::thread::current().id()));
    let written = std::fs::create_dir_all(dir)
        .and_then(|()| std::fs::write(&tmp, bytes))
        .and_then(|()| std::fs::rename(&tmp, &path));
    if let Err(err) = written {
        crate::rr::re_log::debug!("Asset disk cache {}: {err}", path.display());
        std::fs::remove_file(&tmp).ok();
        return false;
    }
    true
}

/// Mark a cached file as just used (pruning goes by modification time).
#[cfg(not(target_arch = "wasm32"))]
fn touch(path: &std::path::Path) {
    if let Ok(file) = std::fs::File::options().write(true).open(path) {
        file.set_modified(std::time::SystemTime::now()).ok();
    }
}

/// Delete the least recently used files of the disk cache until it holds at most `budget` bytes,
/// and temporary files left behind (older than an hour).
#[cfg(not(target_arch = "wasm32"))]
fn prune_disk(dir: &std::path::Path, budget: u64) {
    use std::time::SystemTime;
    let Ok(read) = std::fs::read_dir(dir) else { return };
    let mut files = Vec::new();
    for entry in read.flatten() {
        let Ok(meta) = entry.metadata() else { continue };
        if !meta.is_file() {
            continue;
        }
        let modified = meta.modified().unwrap_or(SystemTime::UNIX_EPOCH);
        let name = entry.file_name();
        if name.to_string_lossy().starts_with('.') {
            if modified.elapsed().is_ok_and(|age| age > Duration::from_secs(3600)) {
                std::fs::remove_file(entry.path()).ok();
            }
            continue;
        }
        files.push((modified, meta.len(), entry.path()));
    }
    let mut total: u64 = files.iter().map(|(_, len, _)| len).sum();
    files.sort();
    for (_, len, path) in files {
        if total <= budget {
            break;
        }
        if std::fs::remove_file(&path).is_ok() {
            total -= len;
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicUsize, Ordering};

    use super::*;

    /// A fetch that answers from `files` (by hash) at once, counting calls; `corrupt` answers with
    /// other bytes.
    fn cache(files: Vec<Vec<u8>>, corrupt: bool, budget: usize, retry: Duration, disk: Option<std::path::PathBuf>) -> (AssetCache, Arc<AtomicUsize>) {
        let calls = Arc::new(AtomicUsize::new(0));
        let counter = calls.clone();
        let files: HashMap<String, Vec<u8>> = files.into_iter().map(|f| (sha256_hex(&f), f)).collect();
        let fetch = move |sha256: &str, done: Done| {
            counter.fetch_add(1, Ordering::SeqCst);
            match files.get(sha256) {
                Some(_) if corrupt => done(Ok(b"something else".to_vec())),
                Some(bytes) => done(Ok(bytes.clone())),
                None => done(Err("404".into())),
            }
        };
        (AssetCache::with_fetch(Box::new(fetch), budget, retry, disk), calls)
    }

    fn asset(bytes: &[u8]) -> AssetRef {
        AssetRef {
            sha256: sha256_hex(bytes),
            size: bytes.len() as u64,
            media_type: "model/obj".into(),
        }
    }

    fn ready(state: AssetState) -> Arc<[u8]> {
        match state {
            AssetState::Ready(bytes) => bytes,
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn fetches_once_then_serves_from_memory() {
        let file = b"v 0 0 0".to_vec();
        let (cache, calls) = cache(vec![file.clone()], false, MEMORY_BUDGET, RETRY_AFTER, None);
        // This fetch answers at once, so the first call is already pending → ready.
        assert!(matches!(cache.get(&asset(&file)), AssetState::Pending));
        assert_eq!(&*ready(cache.get(&asset(&file))), &file[..]);
        assert_eq!(&*ready(cache.get(&asset(&file))), &file[..]);
        assert_eq!(calls.load(Ordering::SeqCst), 1);
        assert_eq!(cache.memory_bytes(), file.len());
    }

    #[test]
    fn hash_mismatch_fails() {
        let file = b"solid x".to_vec();
        let (cache, _) = cache(vec![file.clone()], true, MEMORY_BUDGET, RETRY_AFTER, None);
        cache.get(&asset(&file));
        let AssetState::Failed { err, retryable } = cache.get(&asset(&file)) else { panic!() };
        assert!(err.contains("content hash"), "{err}");
        assert!(!retryable, "the same bytes will not match later either");
        assert_eq!(cache.memory_bytes(), 0);
    }

    #[test]
    fn not_a_hash_fails_without_fetching() {
        let (cache, calls) = cache(vec![], false, MEMORY_BUDGET, RETRY_AFTER, None);
        let bad = AssetRef {
            sha256: "../../etc/passwd".into(),
            ..Default::default()
        };
        assert!(matches!(cache.get(&bad), AssetState::Failed { retryable: false, .. }));
        assert_eq!(calls.load(Ordering::SeqCst), 0);
    }

    #[test]
    fn failures_retry_after_the_backoff() {
        let missing = asset(b"nowhere");
        let (cache, calls) = cache(vec![], false, MEMORY_BUDGET, Duration::from_secs(3600), None);
        cache.get(&missing);
        assert!(matches!(cache.get(&missing), AssetState::Failed { retryable: true, .. }), "a 404 may be temporary");
        assert_eq!(calls.load(Ordering::SeqCst), 1);

        let (cache, calls) = cache_retrying_at_once();
        cache.get(&missing);
        assert!(matches!(cache.get(&missing), AssetState::Pending));
        assert_eq!(calls.load(Ordering::SeqCst), 2);
    }

    #[test]
    fn a_fetch_without_an_answer_fails_retryably() {
        // A fetch that never answers (the request was lost with the backend).
        let calls = Arc::new(AtomicUsize::new(0));
        let counter = calls.clone();
        let fetch = move |_: &str, _: Done| {
            counter.fetch_add(1, Ordering::SeqCst);
        };
        let mut cache = AssetCache::with_fetch(Box::new(fetch), MEMORY_BUDGET, Duration::ZERO, None);
        cache.set_fetch_timeout(Duration::from_millis(20));
        let lost = asset(b"lost");
        assert!(matches!(cache.get(&lost), AssetState::Pending));
        let deadline = Instant::now() + Duration::from_secs(5);
        while matches!(cache.inner.state.lock().unwrap().entries.get(&lost.sha256), Some(Entry::Pending)) {
            assert!(Instant::now() < deadline, "still pending");
            std::thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(cache.inner.slots.lock().unwrap().busy, 0, "its slot is free again");
        assert!(matches!(cache.get(&lost), AssetState::Pending), "asked again");
        assert_eq!(calls.load(Ordering::SeqCst), 2);
    }

    #[test]
    fn retry_failed_retries_at_once() {
        let missing = asset(b"not yet");
        let (missing_cache, calls) = cache(vec![], false, MEMORY_BUDGET, Duration::from_secs(3600), None);
        missing_cache.get(&missing);
        assert!(matches!(missing_cache.get(&missing), AssetState::Failed { retryable: true, .. }));
        missing_cache.retry_failed();
        missing_cache.get(&missing);
        assert_eq!(calls.load(Ordering::SeqCst), 2, "fetched again without the backoff");
        // What will never succeed stays failed.
        let (corrupt, calls) = cache(vec![b"x".to_vec()], true, MEMORY_BUDGET, Duration::from_secs(3600), None);
        corrupt.get(&asset(b"x"));
        corrupt.retry_failed();
        assert!(matches!(corrupt.get(&asset(b"x")), AssetState::Failed { retryable: false, .. }));
        assert_eq!(calls.load(Ordering::SeqCst), 1);
    }

    fn cache_retrying_at_once() -> (AssetCache, Arc<AtomicUsize>) {
        cache(vec![], false, MEMORY_BUDGET, Duration::ZERO, None)
    }

    #[test]
    fn evicts_least_recently_used_beyond_the_budget() {
        let (a, b, c) = (vec![1u8; 40], vec![2u8; 40], vec![3u8; 40]);
        let (cache, calls) = cache(vec![a.clone(), b.clone(), c.clone()], false, 100, RETRY_AFTER, None);
        cache.get(&asset(&a));
        cache.get(&asset(&b));
        // `a` used more recently than `b`.
        ready(cache.get(&asset(&a)));
        cache.get(&asset(&c));
        assert_eq!(cache.memory_bytes(), 80);
        ready(cache.get(&asset(&a)));
        ready(cache.get(&asset(&c)));
        assert_eq!(calls.load(Ordering::SeqCst), 3);
        // `b` was evicted: fetched again.
        assert!(matches!(cache.get(&asset(&b)), AssetState::Pending));
        assert_eq!(calls.load(Ordering::SeqCst), 4);
    }

    #[test]
    fn an_asset_bigger_than_the_budget_is_kept() {
        let big = vec![7u8; 200];
        let (cache, _) = cache(vec![big.clone()], false, 100, RETRY_AFTER, None);
        cache.get(&asset(&big));
        ready(cache.get(&asset(&big)));
    }

    #[test]
    fn pending_entries_are_not_evicted() {
        // A fetch that never answers.
        let fetch = |_: &str, _: Done| {};
        let cache = AssetCache::with_fetch(Box::new(fetch), 0, RETRY_AFTER, None);
        let one = asset(b"one");
        assert!(matches!(cache.get(&one), AssetState::Pending));
        cache.inner.finish(&sha256_hex(b"two"), Ok(b"two".to_vec()));
        assert!(matches!(cache.get(&one), AssetState::Pending));
    }

    #[test]
    fn at_most_max_fetches_at_once() {
        // A fetch that answers only when told to.
        let open: Arc<Mutex<Vec<(String, Done)>>> = Arc::default();
        let calls = open.clone();
        let fetch = move |sha256: &str, done: Done| calls.lock().unwrap().push((sha256.to_owned(), done));
        let cache = AssetCache::with_fetch(Box::new(fetch), MEMORY_BUDGET, RETRY_AFTER, None);
        let files: Vec<Vec<u8>> = (0..MAX_FETCHES + 2).map(|i| vec![i as u8; 4]).collect();
        for file in &files {
            assert!(matches!(cache.get(&asset(file)), AssetState::Pending));
        }
        assert_eq!(open.lock().unwrap().len(), MAX_FETCHES, "the rest wait");

        // One finishes: the next starts.
        let (sha256, done) = open.lock().unwrap().remove(0);
        let file = files.iter().find(|f| sha256_hex(f) == sha256).unwrap().clone();
        done(Ok(file.clone()));
        ready(cache.get(&asset(&file)));
        assert_eq!(open.lock().unwrap().len(), MAX_FETCHES);
        // Failures free their slot too, and every asset gets its turn.
        for _ in 0..2 {
            let (_, done) = open.lock().unwrap().remove(0);
            done(Err("404".into()));
        }
        assert_eq!(open.lock().unwrap().len(), MAX_FETCHES - 1);
        assert_eq!(cache.inner.slots.lock().unwrap().waiting(), 0);
    }

    #[test]
    fn high_priority_fetches_go_first() {
        let open: Arc<Mutex<Vec<(String, Done)>>> = Arc::default();
        let calls = open.clone();
        let fetch = move |sha256: &str, done: Done| calls.lock().unwrap().push((sha256.to_owned(), done));
        let cache = AssetCache::with_fetch(Box::new(fetch), MEMORY_BUDGET, RETRY_AFTER, None);
        let busy: Vec<Vec<u8>> = (0..MAX_FETCHES).map(|i| vec![i as u8; 4]).collect();
        for file in &busy {
            cache.get(&asset(file));
        }
        let (full, raised, lod) = (b"full".to_vec(), b"raised".to_vec(), b"lod".to_vec());
        cache.get(&asset(&full));
        cache.get(&asset(&raised));
        cache.get_with(&asset(&lod), Priority::High);
        // Asked again at high priority while it waits: it goes before `full` now.
        assert!(matches!(cache.get_with(&asset(&raised), Priority::High), AssetState::Pending));
        assert_eq!(cache.inner.slots.lock().unwrap().waiting(), 3);

        let mut started = Vec::new();
        for _ in 0..3 {
            let (_, done) = open.lock().unwrap().remove(0);
            done(Err("404".into()));
            started.push(open.lock().unwrap().last().unwrap().0.clone());
        }
        assert_eq!(started, [sha256_hex(&lod), sha256_hex(&raised), sha256_hex(&full)]);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn the_disk_cache_is_pruned_oldest_first() {
        let dir = std::env::temp_dir().join(format!("factoryos-asset-prune-test-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        std::fs::create_dir_all(&dir).unwrap();
        let now = std::time::SystemTime::now();
        for (name, age) in [("old", 30), ("mid", 20), ("new", 10)] {
            let path = dir.join(name);
            std::fs::write(&path, [0u8; 100]).unwrap();
            let file = std::fs::File::options().write(true).open(&path).unwrap();
            file.set_modified(now - Duration::from_secs(age)).unwrap();
        }
        // A read refreshes a file: `old` is now the most recently used.
        touch(&dir.join("old"));
        prune_disk(&dir, 200);
        let mut left: Vec<_> = std::fs::read_dir(&dir).unwrap().map(|e| e.unwrap().file_name().into_string().unwrap()).collect();
        left.sort();
        assert_eq!(left, ["new", "old"]);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn disk_cache_survives_a_restart() {
        let dir = std::env::temp_dir().join(format!("factoryos-asset-cache-test-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        let file = b"mtllib x.mtl".to_vec();
        let wait = |cache: &AssetCache| {
            let start = std::time::Instant::now();
            loop {
                if let AssetState::Ready(bytes) = cache.get(&asset(&file)) {
                    return bytes;
                }
                assert!(start.elapsed() < Duration::from_secs(10), "timed out");
                std::thread::sleep(Duration::from_millis(5));
            }
        };

        let (first, calls) = cache(vec![file.clone()], false, MEMORY_BUDGET, RETRY_AFTER, Some(dir.clone()));
        assert_eq!(&*wait(&first), &file[..]);
        assert_eq!(calls.load(Ordering::SeqCst), 1);
        assert!(dir.join(sha256_hex(&file)).exists());

        let (second, calls) = cache(vec![], false, MEMORY_BUDGET, RETRY_AFTER, Some(dir.clone()));
        assert_eq!(&*wait(&second), &file[..]);
        assert_eq!(calls.load(Ordering::SeqCst), 0);

        // A corrupted file is dropped and fetched again.
        std::fs::write(dir.join(sha256_hex(&file)), b"garbage").unwrap();
        let (third, calls) = cache(vec![file.clone()], false, MEMORY_BUDGET, RETRY_AFTER, Some(dir.clone()));
        assert_eq!(&*wait(&third), &file[..]);
        assert_eq!(calls.load(Ordering::SeqCst), 1);
        assert_eq!(std::fs::read(dir.join(sha256_hex(&file))).unwrap(), file);

        std::fs::remove_dir_all(&dir).ok();
    }
}
