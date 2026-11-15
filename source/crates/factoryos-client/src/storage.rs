//! [`DataSource`]: where the viewer loads and saves files ([`crate::protocol::storage`]).
//!
//! The backend says which ([`api::STORAGE_INFO`]), asked once on first use:
//! * **Edge** (a deployed workspace behind CloudFront, for its owner: [`StorageInfo::edge_path`]):
//!   the viewer reads and writes the bucket through the edge at its own origin (`/_data/…`, the
//!   [`edge`] layout), with its session cookie like any other request; the edge maps every path into
//!   the signed-in user's own subtree. No presigning. An asset the edge doesn't have yet (`403`/`404`:
//!   it may not list the bucket) is asked for ([`api::STORAGE_ASSETS_ENSURE`]) and fetched once more.
//! * **S3** (a deployed workspace): the backend presigns each request ([`api::STORAGE_PRESIGN`],
//!   short-lived, scoped by the backend to its user's subtree and the shared catalogue, writable by catalogue admins only) and the
//!   viewer fetches/uploads the bytes from/to S3 itself, without cookies or AWS credentials of its
//!   own.
//! * **HTTP** (a backend on this machine without a bucket, an older backend, or a caller the
//!   backend doesn't presign for, e.g. another user on a shared workspace): the backend's own routes.
//!
//! An upload over the edge or to S3 is then committed ([`api::STORAGE_COMMIT`]) so the backend's
//! tools see it.
//!
//! **Falling back**, one step at a time: a failed edge transfer (not a missing file) goes on with
//! presigned URLs, a failed S3 transfer (no CORS for this origin, an expired URL) with HTTP, and the
//! source stays on the lower one from then on ([`Route::below`]).
//!
//! Nothing here waits forever: the mode, a presign and a download each have a deadline
//! ([`INFO_TIMEOUT`], [`PRESIGN_TIMEOUT`], [`DOWNLOAD_TIMEOUT`], [`UPLOAD_TIMEOUT`],
//! [`ENSURE_TIMEOUT`]) after which the next way is used. A mode that is not a clear answer (a
//! timeout, a backend away: 502/503) is not kept, and [`DataSource::forget_modes`] (a restarted
//! backend may have another) asks again.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::protocol::api as backend_api;
use crate::protocol::storage::{
    Commit, EnsureAssets, EnsuredAssets, Folder, Method, Move, ObjectList, PresignItem, PresignRequest, PresignResponse, Publish, Published,
    Remove, Scope, StorageInfo, StorageMode, api, edge,
};
use crate::{Api, Callback};

/// How long the storage mode may take to come ([`api::STORAGE_INFO`]): then HTTP, for now.
pub const INFO_TIMEOUT: Duration = Duration::from_secs(10);
/// How long a presign may take: then the backend's own route.
pub const PRESIGN_TIMEOUT: Duration = Duration::from_secs(15);
/// How long a download from S3 or over the edge may take: then the next way.
pub const DOWNLOAD_TIMEOUT: Duration = Duration::from_secs(120);
/// How long an upload over the edge may take: then the next way.
pub const UPLOAD_TIMEOUT: Duration = Duration::from_secs(120);
/// How long the backend may take to put missing assets into S3 ([`api::STORAGE_ASSETS_ENSURE`]).
pub const ENSURE_TIMEOUT: Duration = Duration::from_secs(60);

/// Bumped by [`DataSource::forget_modes`]: a mode decided before is asked again.
static MODE_EPOCH: AtomicU64 = AtomicU64::new(0);

#[derive(Clone)]
pub struct DataSource {
    api: Api,
    /// The mode, and the [`MODE_EPOCH`] it was decided in.
    mode: Arc<Mutex<(u64, Mode)>>,
}

/// Which way bytes go, best first.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Route {
    /// Through the workspace edge at the viewer's origin, under this path (`/_data`).
    Edge(String),
    /// Presigned URLs to S3.
    S3,
    /// The backend's own routes.
    Http,
}

impl Route {
    /// The next way when this one fails: edge → S3 → HTTP (which is the last).
    fn below(&self) -> Route {
        match self {
            Self::Edge(_) => Self::S3,
            Self::S3 | Self::Http => Self::Http,
        }
    }
}

/// What a [`api::STORAGE_INFO`] answer means.
#[derive(Debug, PartialEq, Eq)]
enum Answer {
    Edge(String),
    S3,
    Http,
    /// No clear answer (a timeout, a backend away): HTTP for the requests waiting, asked again.
    Unclear,
}

impl Answer {
    fn of(info: &Result<StorageInfo, String>) -> Self {
        match info {
            Ok(info) if info.mode == StorageMode::S3 => match info.edge_path.as_deref().filter(|p| is_edge_path(p)) {
                Some(path) => Self::Edge(path.to_owned()),
                None => Self::S3,
            },
            Ok(_) => Self::Http,
            // An older backend without the route, or a caller it doesn't serve: HTTP for good.
            Err(err) if ["400", "401", "403", "404", "405"].iter().any(|code| err.starts_with(code)) => Self::Http,
            Err(_) => Self::Unclear,
        }
    }
}

/// An absolute path at the viewer's own origin (`/_data`): never another host (`//h`, `@h`), since
/// the session cookie goes along.
fn is_edge_path(path: &str) -> bool {
    path.starts_with('/')
        && !path.starts_with("//")
        && path.bytes().all(|b| b.is_ascii_alphanumeric() || matches!(b, b'/' | b'_' | b'-' | b'.'))
        && !path.split('/').any(|seg| seg == "..")
}

enum Mode {
    /// Not asked yet / being asked: what waits for the answer.
    Unknown { asking: bool, waiting: Vec<Box<dyn FnOnce(Route) + Send>> },
    Known(Route),
}

/// The status an [`Api`] error starts with (`404 Not Found: …`), if it had one.
fn status_of(err: &str) -> Option<u16> {
    let code = err.split(' ').next()?;
    (code.len() == 3).then(|| code.parse().ok()).flatten()
}

/// What a `GET` over the edge found.
#[derive(Debug, PartialEq, Eq)]
enum EdgeGet {
    Found,
    /// `403`/`404`: no such object (S3 says `403` when the reader may not list the bucket).
    Missing,
    /// Anything else (no session, the edge or S3 away, a timeout): the next way.
    Failed,
}

fn edge_get(result: &Result<Vec<u8>, String>) -> EdgeGet {
    match result {
        Ok(_) => EdgeGet::Found,
        Err(err) if matches!(status_of(err), Some(403 | 404)) => EdgeGet::Missing,
        Err(_) => EdgeGet::Failed,
    }
}

/// After an asset `GET` over the edge (`retried`: the second, after asking the backend).
#[derive(Debug, PartialEq, Eq)]
enum AssetNext {
    Deliver,
    /// Missing: the backend puts it there ([`api::STORAGE_ASSETS_ENSURE`]), then once more.
    EnsureThenRetry,
    /// Failed, or missing although the backend put it there: presigned URLs from now on.
    StepDown,
}

fn asset_next(retried: bool, got: EdgeGet) -> AssetNext {
    match got {
        EdgeGet::Found => AssetNext::Deliver,
        EdgeGet::Missing if !retried => AssetNext::EnsureThenRetry,
        EdgeGet::Missing | EdgeGet::Failed => AssetNext::StepDown,
    }
}

/// Whether the backend put `sha256` into S3.
fn ensured(response: &Result<EnsuredAssets, String>, sha256: &str) -> bool {
    response.as_ref().is_ok_and(|r| r.items.iter().any(|item| item.sha256 == sha256 && item.ok))
}

/// After a `PUT` over the edge.
#[derive(Debug, PartialEq, Eq)]
enum EdgePut {
    /// Written: committed next.
    Commit,
    /// Refused for this request (not allowed, too large, a bad key): the next way, whose backend
    /// says why; the edge stays.
    Lower,
    /// Failed: presigned URLs from now on.
    StepDown,
}

fn edge_put(result: &Result<(), String>) -> EdgePut {
    match result {
        Ok(()) => EdgePut::Commit,
        Err(err) if matches!(status_of(err), Some(400 | 403 | 413)) => EdgePut::Lower,
        Err(_) => EdgePut::StepDown,
    }
}

impl DataSource {
    pub fn new(api: Api) -> Self {
        let epoch = MODE_EPOCH.load(Ordering::Relaxed);
        Self { api, mode: Arc::new(Mutex::new((epoch, Mode::Unknown { asking: false, waiting: Vec::new() }))) }
    }

    /// Always over the backend's HTTP routes (tests, or a caller that knows).
    pub fn http(api: Api) -> Self {
        Self { api, mode: Arc::new(Mutex::new((u64::MAX, Mode::Known(Route::Http)))) }
    }

    /// Every [`DataSource::new`]'s mode is asked again on next use (the backend restarted, and
    /// may keep its files elsewhere now).
    pub fn forget_modes() {
        MODE_EPOCH.fetch_add(1, Ordering::Relaxed);
    }

    pub fn api(&self) -> &Api {
        &self.api
    }

    /// Run `then(route)` once the mode is known.
    fn with_mode(&self, then: impl FnOnce(Route) + Send + 'static) {
        let ask = {
            let mut guard = self.mode.lock().unwrap();
            let (epoch, mode) = &mut *guard;
            let now = MODE_EPOCH.load(Ordering::Relaxed);
            // `http()`'s is for good; a decided mode of an earlier epoch is asked again.
            if *epoch != u64::MAX && *epoch != now {
                *epoch = now;
                if matches!(mode, Mode::Known(_)) {
                    *mode = Mode::Unknown { asking: false, waiting: Vec::new() };
                }
            }
            match mode {
                Mode::Known(route) => {
                    let route = route.clone();
                    drop(guard);
                    return then(route);
                }
                Mode::Unknown { asking, waiting } => {
                    waiting.push(Box::new(then));
                    !std::mem::replace(asking, true)
                }
            }
        };
        if ask {
            let state = self.mode.clone();
            self.api.with_timeout(INFO_TIMEOUT).get(api::STORAGE_INFO, move |info: Result<StorageInfo, String>| {
                let answer = Answer::of(&info);
                if let (Answer::Unclear, Err(err)) = (&answer, &info) {
                    log(&format!("storage: no storage mode from the backend ({err}); loading over the backend for now"));
                }
                let (next, route) = match answer {
                    Answer::Edge(path) => (Mode::Known(Route::Edge(path.clone())), Route::Edge(path)),
                    Answer::S3 => (Mode::Known(Route::S3), Route::S3),
                    Answer::Http => (Mode::Known(Route::Http), Route::Http),
                    Answer::Unclear => (Mode::Unknown { asking: false, waiting: Vec::new() }, Route::Http),
                };
                let waiting = match std::mem::replace(&mut state.lock().unwrap().1, next) {
                    Mode::Unknown { waiting, .. } => waiting,
                    Mode::Known(_) => Vec::new(),
                };
                for then in waiting {
                    then(route.clone());
                }
            });
        }
    }

    /// The way known now, if any.
    #[cfg(test)]
    fn route(&self) -> Option<Route> {
        match &self.mode.lock().unwrap().1 {
            Mode::Known(route) => Some(route.clone()),
            Mode::Unknown { .. } => None,
        }
    }

    /// From now on, `to` (a transfer the way above it failed): one step down at a time, so a late
    /// failure of a lower way never skips one.
    fn fall_back_to(&self, to: Route, why: &str) {
        let mut mode = self.mode.lock().unwrap();
        if let Mode::Known(now) = &mode.1
            && *now != to
            && now.below() == to
        {
            let (from, name) = (if matches!(now, Route::Edge(_)) { "the edge" } else { "S3" }, if to == Route::S3 { "presigned URLs" } else { "the backend" });
            log(&format!("storage: {from} failed ({why}); loading over {name} from now on"));
            mode.1 = Mode::Known(to);
        }
    }

    /// The API for the edge's paths: at the origin (the edge serves `/_data` there, whatever the
    /// backend's prefix), with the session cookie ([`Api`] sends it), with `timeout`.
    fn edge_api(&self, timeout: Duration) -> Api {
        self.api.at_origin().with_timeout(timeout)
    }

    /// One presigned URL for `item`.
    fn presign(&self, item: PresignItem, done: Callback<String>) {
        let request = PresignRequest { items: vec![item] };
        let api = self.api.with_timeout(PRESIGN_TIMEOUT);
        api.post_json(api::STORAGE_PRESIGN, &request, move |response: Result<PresignResponse, String>| {
            let presigned = response.and_then(|r| r.items.into_iter().next().ok_or_else(|| "no presigned URL".to_owned()));
            done(presigned.and_then(|p| p.url.ok_or_else(|| p.error.unwrap_or_else(|| "refused".to_owned()))));
        });
    }

    /// A content-addressed asset's bytes (unchecked: the caller verifies the hash).
    pub fn get_asset(&self, sha256: &str, max_bytes: usize, done: Callback<Vec<u8>>) {
        let this = self.clone();
        let sha256 = sha256.to_owned();
        self.with_mode(move |route| this.asset_by(route, sha256, max_bytes, done));
    }

    fn asset_by(&self, route: Route, sha256: String, max_bytes: usize, done: Callback<Vec<u8>>) {
        let http = {
            let (api, path) = (self.api.clone(), backend_api::asset(&sha256));
            move |done: Callback<Vec<u8>>| api.get_bytes(&path, max_bytes, done)
        };
        match route {
            Route::Http => http(done),
            Route::Edge(path) => self.asset_over_edge(path, sha256, max_bytes, false, done),
            Route::S3 => {
                let fetcher = self.clone();
                self.presign(PresignItem::Asset { sha256 }, Box::new(move |url| match url {
                    Ok(url) => fetch_bytes(url, max_bytes, Box::new(move |result| match result {
                        Ok(bytes) => done(Ok(bytes)),
                        Err(err) => {
                            fetcher.fall_back_to(Route::Http, &err);
                            http(done);
                        }
                    })),
                    // Not presigned (e.g. a registration still on its way): the backend serves it.
                    Err(_) => http(done),
                }));
            }
        }
    }

    /// `GET <edge>/asset/<sha256>`; missing: the backend uploads it, then once more ([`asset_next`]).
    fn asset_over_edge(&self, path: String, sha256: String, max_bytes: usize, retried: bool, done: Callback<Vec<u8>>) {
        let this = self.clone();
        let url = edge::asset(&path, &sha256);
        self.edge_api(DOWNLOAD_TIMEOUT).get_bytes(&url, max_bytes, move |result| match asset_next(retried, edge_get(&result)) {
            AssetNext::Deliver => done(result),
            AssetNext::EnsureThenRetry => {
                let request = EnsureAssets { sha256s: vec![sha256.clone()] };
                let api = this.api.with_timeout(ENSURE_TIMEOUT);
                api.post_json(api::STORAGE_ASSETS_ENSURE, &request, move |response: Result<EnsuredAssets, String>| {
                    if ensured(&response, &sha256) {
                        this.asset_over_edge(path, sha256, max_bytes, true, done);
                    } else {
                        // Not one the backend has (or can upload now): presigned URLs for this one
                        // (the backend's own route after them); the edge stays.
                        this.asset_by(Route::S3, sha256, max_bytes, done);
                    }
                });
            }
            AssetNext::StepDown => {
                let why = result.err().unwrap_or_else(|| "still missing after the upload".to_owned());
                this.fall_back_to(Route::S3, &why);
                this.asset_by(Route::S3, sha256, max_bytes, done);
            }
        });
    }

    /// The bytes of `key` in `scope`.
    pub fn get(&self, scope: Scope, key: &str, max_bytes: usize, done: Callback<Vec<u8>>) {
        let this = self.clone();
        let key = key.to_owned();
        self.with_mode(move |route| this.get_by(route, scope, key, max_bytes, done));
    }

    fn get_by(&self, route: Route, scope: Scope, key: String, max_bytes: usize, done: Callback<Vec<u8>>) {
        let http = {
            let (api, path) = (self.api.clone(), api::object(scope, &key));
            move |done: Callback<Vec<u8>>| api.get_bytes(&path, max_bytes, done)
        };
        match route {
            Route::Http => http(done),
            Route::Edge(path) => {
                let this = self.clone();
                let url = edge::object(&path, scope, &key);
                self.edge_api(DOWNLOAD_TIMEOUT).get_bytes(&url, max_bytes, move |result| match edge_get(&result) {
                    EdgeGet::Found => done(result),
                    // As the backend's route says it.
                    EdgeGet::Missing => done(Err(format!("404 Not Found: no object {key}"))),
                    EdgeGet::Failed => {
                        this.fall_back_to(Route::S3, result.as_ref().err().map_or("", String::as_str));
                        this.get_by(Route::S3, scope, key, max_bytes, done);
                    }
                });
            }
            Route::S3 => {
                let fetcher = self.clone();
                self.presign(PresignItem::Object { scope, key, method: Method::Get }, Box::new(move |url| match url {
                    Ok(url) => fetch_bytes(url, max_bytes, Box::new(move |result| match result {
                        Ok(bytes) => done(Ok(bytes)),
                        Err(err) if err.starts_with("404") => done(Err(err)),
                        Err(err) => {
                            fetcher.fall_back_to(Route::Http, &err);
                            http(done);
                        }
                    })),
                    // No presign in time (the backend away or busy): its own route then.
                    Err(err) if err.starts_with(crate::TIMED_OUT) => http(done),
                    Err(err) => done(Err(err)),
                }));
            }
        }
    }

    /// Save `bytes` as `key` of `scope` (the user's subtree; the shared catalogue for catalogue
    /// admins: the backend decides).
    pub fn put(&self, scope: Scope, key: &str, bytes: Vec<u8>, done: Callback<()>) {
        let this = self.clone();
        let key = key.to_owned();
        self.with_mode(move |route| this.put_by(route, scope, key, bytes, done));
    }

    fn put_by(&self, route: Route, scope: Scope, key: String, bytes: Vec<u8>, done: Callback<()>) {
        let http = {
            let (api, path) = (self.api.clone(), api::object(scope, &key));
            move |bytes: Vec<u8>, done: Callback<()>| api.put_bytes(&path, bytes, done)
        };
        match route {
            Route::Http => http(bytes, done),
            Route::Edge(path) => {
                let this = self.clone();
                let (url, retry) = (edge::object(&path, scope, &key), bytes.clone());
                self.edge_api(UPLOAD_TIMEOUT).put_raw(&url, bytes, move |result| match edge_put(&result) {
                    EdgePut::Commit => {
                        let commit = Commit { key, scope };
                        this.api.post_json(api::STORAGE_COMMIT, &commit, done);
                    }
                    EdgePut::Lower => this.put_by(Route::S3, scope, key, retry, done),
                    EdgePut::StepDown => {
                        this.fall_back_to(Route::S3, result.as_ref().err().map_or("", String::as_str));
                        this.put_by(Route::S3, scope, key, retry, done);
                    }
                });
            }
            Route::S3 => {
                let committer = self.clone();
                self.presign(PresignItem::Object { scope, key: key.clone(), method: Method::Put }, Box::new(move |url| match url {
                    Ok(url) => {
                        let retry = bytes.clone();
                        let request = ehttp::Request::new(ehttp::Method::PUT, url, &[("Content-Type", "application/octet-stream")]).with_body(bytes);
                        ehttp::fetch(no_credentials(request), move |response| match response {
                            Ok(response) if response.ok => {
                                let commit = Commit { key, scope };
                                committer.api.post_json(api::STORAGE_COMMIT, &commit, done);
                            }
                            other => {
                                let why = other.map_or_else(|err| err, |r| format!("{} {}", r.status, r.status_text));
                                committer.fall_back_to(Route::Http, &why);
                                http(retry, done);
                            }
                        });
                    }
                    Err(err) => done(Err(err)),
                }));
            }
        }
    }

    /// The objects of `scope` under `prefix` (always from the backend: metadata, not bytes).
    pub fn list(&self, scope: Scope, prefix: &str, done: Callback<ObjectList>) {
        self.api.get(&api::list(scope, prefix), done);
    }

    /// The backend's storage mode and the caller's roles, asked afresh.
    pub fn info(&self, done: Callback<StorageInfo>) {
        self.api.get(api::STORAGE_INFO, done);
    }

    /// A new empty folder `key` of `scope`.
    pub fn make_folder(&self, scope: Scope, key: &str, done: Callback<()>) {
        self.api.post_json(api::STORAGE_FOLDER, &Folder { scope, key: key.to_owned() }, done);
    }

    /// Rename/move a file, or a folder (both ending in `/`), within `scope`.
    pub fn move_keys(&self, scope: Scope, from: &str, to: &str, overwrite: bool, done: Callback<Published>) {
        self.api.post_json(api::STORAGE_MOVE, &Move { scope, from: from.to_owned(), to: to.to_owned(), overwrite }, done);
    }

    /// Delete a file, or a folder (ending in `/`).
    pub fn remove(&self, scope: Scope, key: &str, done: Callback<Published>) {
        self.api.post_json(api::STORAGE_REMOVE, &Remove { scope, key: key.to_owned() }, done);
    }

    /// Copy a file or folder of the user's subtree into the shared catalogue (catalogue admins).
    pub fn publish(&self, from: &str, to: &str, overwrite: bool, done: Callback<Published>) {
        self.api.post_json(api::STORAGE_PUBLISH, &Publish { from: from.to_owned(), to: to.to_owned(), overwrite }, done);
    }

    /// Where a browser downloads `key` of `scope` from (through the backend).
    pub fn download_url(&self, scope: Scope, key: &str) -> String {
        self.api.url_for(&api::object(scope, key))
    }
}

/// Requests to S3 carry no cookies (they'd be the backend's site's anyway) and no credentials.
fn no_credentials(request: ehttp::Request) -> ehttp::Request {
    #[cfg(target_arch = "wasm32")]
    let request = request.with_credentials(ehttp::Credentials::Omit);
    request
}

fn fetch_bytes(url: String, max_bytes: usize, done: Callback<Vec<u8>>) {
    let mut request = ehttp::Request::get(url);
    request.timeout = Some(DOWNLOAD_TIMEOUT);
    let done = crate::with_deadline(DOWNLOAD_TIMEOUT, "S3 download", done);
    ehttp::fetch(no_credentials(request), move |response| {
        done(response.and_then(|r| {
            if !r.ok {
                return Err(format!("{} {}", r.status, r.status_text));
            }
            if r.bytes.len() > max_bytes {
                return Err(format!("{} bytes, more than {max_bytes}", r.bytes.len()));
            }
            Ok(r.bytes)
        }))
    });
}

fn log(line: &str) {
    #[cfg(target_arch = "wasm32")]
    web_sys_log(line);
    #[cfg(not(target_arch = "wasm32"))]
    eprintln!("{line}");
}

#[cfg(target_arch = "wasm32")]
fn web_sys_log(line: &str) {
    #[wasm_bindgen::prelude::wasm_bindgen]
    extern "C" {
        #[wasm_bindgen::prelude::wasm_bindgen(js_namespace = console)]
        fn warn(s: &str);
    }
    warn(line);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn info(mode: StorageMode) -> Result<StorageInfo, String> {
        let mut info: StorageInfo = serde_json::from_value(serde_json::json!({ "mode": "local" })).unwrap();
        info.mode = mode;
        Ok(info)
    }

    #[test]
    fn the_edge_path_is_chosen_when_the_backend_names_one() {
        let mut edge = info(StorageMode::S3).unwrap();
        edge.edge_path = Some("/_data".into());
        assert_eq!(Answer::of(&Ok(edge.clone())), Answer::Edge("/_data".into()));
        // Only with S3 (a local backend's bytes are its own).
        edge.mode = StorageMode::Local;
        assert_eq!(Answer::of(&Ok(edge.clone())), Answer::Http);
        // Never another origin: presigned URLs then.
        edge.mode = StorageMode::S3;
        for bad in ["//evil.example/x", "@evil.example", "https://evil.example/_data", "_data", "/_data/../api", "/_data?x"] {
            edge.edge_path = Some(bad.into());
            assert_eq!(Answer::of(&Ok(edge.clone())), Answer::S3, "{bad}");
        }
    }

    #[test]
    fn falling_back_goes_one_step_at_a_time() {
        assert_eq!(Route::Edge("/_data".into()).below(), Route::S3);
        assert_eq!(Route::S3.below(), Route::Http);
        assert_eq!(Route::Http.below(), Route::Http);
        let source = DataSource::new(Api::new("http://127.0.0.1:1"));
        source.mode.lock().unwrap().1 = Mode::Known(Route::Edge("/_data".into()));
        // A late failure of a lower way doesn't skip the one between.
        source.fall_back_to(Route::Http, "late S3 failure");
        assert_eq!(source.route(), Some(Route::Edge("/_data".into())));
        source.fall_back_to(Route::S3, "edge failed");
        assert_eq!(source.route(), Some(Route::S3));
        source.fall_back_to(Route::S3, "again");
        assert_eq!(source.route(), Some(Route::S3));
        source.fall_back_to(Route::Http, "S3 failed");
        assert_eq!(source.route(), Some(Route::Http));
    }

    #[test]
    fn edge_misses_are_ensured_once_then_step_down() {
        assert_eq!(status_of("404 Not Found: x"), Some(404));
        assert_eq!(status_of(&format!("{}: GET: no answer", crate::TIMED_OUT)), None);
        assert_eq!(edge_get(&Ok(vec![1])), EdgeGet::Found);
        assert_eq!(edge_get(&Err("404 Not Found: ".into())), EdgeGet::Missing);
        // S3 without ListBucket.
        assert_eq!(edge_get(&Err("403 Forbidden: <Error><Code>AccessDenied</Code></Error>".into())), EdgeGet::Missing);
        for failed in ["401 Unauthorized: {}", "502 Bad Gateway: ", "500 Internal Server Error: ", "error sending request", "timed out: x"] {
            assert_eq!(edge_get(&Err(failed.into())), EdgeGet::Failed, "{failed}");
        }
        assert_eq!(asset_next(false, EdgeGet::Found), AssetNext::Deliver);
        assert_eq!(asset_next(false, EdgeGet::Missing), AssetNext::EnsureThenRetry);
        assert_eq!(asset_next(true, EdgeGet::Missing), AssetNext::StepDown, "missing although uploaded");
        assert_eq!(asset_next(false, EdgeGet::Failed), AssetNext::StepDown);
        assert_eq!(asset_next(true, EdgeGet::Found), AssetNext::Deliver);
        let sha = "a".repeat(64);
        let item = |ok| EnsuredAssets { items: vec![crate::protocol::storage::EnsuredAsset { sha256: sha.clone(), ok, error: None }] };
        assert!(ensured(&Ok(item(true)), &sha));
        assert!(!ensured(&Ok(item(false)), &sha));
        assert!(!ensured(&Ok(item(true)), &"b".repeat(64)));
        assert!(!ensured(&Err("404 Not Found: ".into()), &sha), "an older backend");
        assert_eq!(edge_put(&Ok(())), EdgePut::Commit);
        assert_eq!(edge_put(&Err("403 Forbidden: ".into())), EdgePut::Lower, "the backend says why");
        assert_eq!(edge_put(&Err("413 Payload Too Large: ".into())), EdgePut::Lower);
        assert_eq!(edge_put(&Err("503 Service Unavailable: ".into())), EdgePut::StepDown);
        assert_eq!(edge_put(&Err("timed out: x".into())), EdgePut::StepDown);
    }

    #[test]
    fn only_clear_answers_are_kept() {
        assert_eq!(Answer::of(&info(StorageMode::S3)), Answer::S3);
        assert_eq!(Answer::of(&info(StorageMode::Local)), Answer::Http);
        // An older backend (no route) or a caller it doesn't presign for: HTTP for good.
        assert_eq!(Answer::of(&Err("404 Not Found: ".to_owned())), Answer::Http);
        assert_eq!(Answer::of(&Err("403 Forbidden: ".to_owned())), Answer::Http);
        // A backend away or restarting, or no answer in time: asked again next time.
        assert_eq!(Answer::of(&Err("502 Bad Gateway: ".to_owned())), Answer::Unclear);
        assert_eq!(Answer::of(&Err("503 Service Unavailable: ".to_owned())), Answer::Unclear);
        assert_eq!(Answer::of(&Err(format!("{}: storage info: no answer within 10 s", crate::TIMED_OUT))), Answer::Unclear);
        assert_eq!(Answer::of(&Err("error sending request: connection refused".to_owned())), Answer::Unclear);
    }

    #[test]
    fn forgotten_modes_are_asked_again() {
        let source = DataSource::new(Api::new("http://127.0.0.1:1"));
        let http = DataSource::http(Api::new("http://127.0.0.1:1"));
        source.mode.lock().unwrap().1 = Mode::Known(Route::S3);
        DataSource::forget_modes();
        // `with_mode` asks again when the epoch it was decided in is over (the answer, from no
        // backend at all here, is no clear one: asked again next time too).
        source.with_mode(|_| {});
        assert!(matches!(source.mode.lock().unwrap().1, Mode::Unknown { .. }));
        let served = Arc::new(Mutex::new(None));
        let sink = served.clone();
        http.with_mode(move |route| *sink.lock().unwrap() = Some(route));
        assert_eq!(*served.lock().unwrap(), Some(Route::Http), "http() is for good");
    }
}
