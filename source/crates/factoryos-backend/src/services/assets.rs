//! Content-addressed files for renderer-neutral scenes ([`factoryos_protocol::scene`]): URDFs,
//! meshes and their sidecars (MTLs, textures, glTF buffers), served by `GET` [`api::ASSET`].
//!
//! Services register the files a scene references ([`AssetIndex::model_ref`],
//! [`AssetIndex::mesh_asset`], [`AssetIndex::register`]) and put the resulting [`AssetRef`]s into
//! their [`factoryos_protocol::scene::SceneDoc`]; viewers fetch the bytes by SHA-256. Only
//! registered hashes are served: the route takes no path, so it can't reach any other file.
//!
//! Paths come from user input (product geometry, URDF `<mesh>`es, an OBJ's `mtllib`, an MTL's
//! textures), so the index itself only registers what a scene may legitimately reference: regular
//! files of an [`ALLOWED_EXTENSIONS`] type whose canonical path (symlinks resolved) lies under one
//! of the roots it was made with (the backend's own data, `asset_register::allowed_roots`).
//!
//! A file is hashed once per (canonical path, mtime, size). If it changes on disk after it was
//! registered, it is re-hashed before it is served, and never served under its old hash.
//!
//! **Converted meshes.** [`AssetIndex::mesh_asset`] also gives OBJ and STL meshes as binary glTF
//! plus a decimated LOD ([`mesh_convert`]), cached under the state directory's `assets/meshes/`
//! by a key over the source's and its sidecars' hashes and [`mesh_convert::VERSION`], and served
//! like any other asset (the index trusts its own cache directory besides the roots). Converting
//! is done synchronously, on the caller's (background) thread: the whole starter catalogue (47
//! meshes, 265k triangles) takes about 0.2 s, its largest mesh 13 ms, once; later calls are cache
//! hits.
//!
//! **Persistence.** The index (`sha256 → canonical path, stamp, AssetRef`) is saved to
//! `assets/index.json` in the state directory ([`default_state_dir`]), at most once a second and
//! through a temporary file, and loaded at startup, so the hashes a producer (the live cell)
//! registered before a backend restart resolve at once. Loaded entries are checked as any other:
//! dropped at load when outside the current roots, of another type or no longer canonical; served
//! only after their bytes hash to the entry again.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::io::Read as _;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::Context as _;
use axum::Router;
use axum::extract::{FromRef, Path as UrlPath, State};
use axum::http::{HeaderMap, HeaderValue, StatusCode, header};
use axum::response::{IntoResponse, Response};
use axum::routing::get;
use factoryos_protocol::api;
use factoryos_protocol::scene::{AssetRef, MeshAsset, ModelRef};
use sha2::{Digest as _, Sha256};

use crate::services::mesh_convert;

/// `Cache-Control` of an asset. Its bytes never change (the URL is their hash), hence a year and
/// `immutable`. `private`, not `public`: assets are behind the edge's login like every
/// `/api/v1` path, so only the authenticated user's browser may keep them; a shared cache (a
/// proxy on the way) must not, it would hand them out without asking the edge. CloudFront honours
/// `private` and doesn't cache them; the edge's cached `/api/v1/assets/*` behaviours (auth on
/// every viewer request, cache hits included) turn it into `public` towards CloudFront's cache on
/// origin-response and back into `private` towards the browser on viewer-response.
const CACHE_CONTROL: &str = "private, max-age=31536000, immutable";

/// What a scene references: URDFs, meshes, their sidecars (MTLs, textures, glTF buffers).
pub const ALLOWED_EXTENSIONS: &[&str] = &[
    "urdf", "xml", "obj", "mtl", "stl", "glb", "gltf", "bin", "dae", "ply", "png", "jpg", "jpeg",
];

/// The canonical directories files may be registered from, asked on every registration (they
/// follow the config).
type Roots = Box<dyn Fn() -> Vec<PathBuf> + Send + Sync>;

/// Content-addressed index of files on this machine. Shared (`Arc`) between the services and the
/// [`router`].
pub struct AssetIndex {
    inner: Mutex<Inner>,
    /// `None` only in tests ([`AssetIndex::unrestricted`]).
    roots: Option<Roots>,
    /// Where converted meshes and the saved index go; `None`: neither.
    store: Option<Store>,
    /// The sidecar names an OBJ or MTL gives, by `<sha256>:<dir>` of that file.
    names: Mutex<HashMap<String, Vec<String>>>,
    /// One lock per conversion key: the same mesh is converted once at a time.
    converting: Mutex<HashMap<String, Arc<Mutex<()>>>>,
    /// Conversion keys that failed (unparsable meshes), not tried again.
    failed: Mutex<HashSet<String>>,
    /// Meshes converted so far.
    conversions: AtomicUsize,
    /// Told of every hash registered that wasn't before ([`Self::on_new_hash`]).
    on_new: Mutex<Option<NewHash>>,
}

type NewHash = Arc<dyn Fn(&str) + Send + Sync>;

/// The index's directory in the state directory.
struct Store {
    /// Canonical; converted meshes, trusted like a root.
    meshes: PathBuf,
    /// The saved index.
    index_file: PathBuf,
    /// Serializes saving.
    saving: Mutex<()>,
}

#[derive(Default)]
struct Inner {
    /// By canonical path.
    files: HashMap<PathBuf, Entry>,
    /// SHA-256 → the canonical paths with those bytes.
    by_hash: HashMap<String, BTreeSet<PathBuf>>,
    /// Bumped by every change; the saved index is of `saved`.
    generation: u64,
    saved: u64,
}

#[derive(Clone)]
struct Entry {
    stamp: Stamp,
    asset: AssetRef,
}

/// What tells a file changed without reading it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Stamp {
    mtime: Option<SystemTime>,
    size: u64,
}

impl Stamp {
    fn of(path: &Path) -> std::io::Result<Self> {
        let meta = std::fs::metadata(path)?;
        Ok(Self { mtime: meta.modified().ok(), size: meta.len() })
    }
}

impl Inner {
    /// Whether the hash is new to the index.
    fn insert(&mut self, path: PathBuf, entry: Entry) -> bool {
        self.forget(&path);
        self.generation += 1;
        let new = !self.by_hash.contains_key(&entry.asset.sha256);
        self.by_hash.entry(entry.asset.sha256.clone()).or_default().insert(path.clone());
        self.files.insert(path, entry);
        new
    }

    fn unindex(&mut self, path: &Path, sha256: &str) {
        if let Some(paths) = self.by_hash.get_mut(sha256) {
            paths.remove(path);
            if paths.is_empty() {
                self.by_hash.remove(sha256);
            }
        }
    }

    fn forget(&mut self, path: &Path) {
        if let Some(old) = self.files.remove(path) {
            self.unindex(path, &old.asset.sha256);
            self.generation += 1;
        }
    }
}

/// The saved index ([`Store::index_file`]).
#[derive(serde::Serialize, serde::Deserialize)]
struct Saved {
    version: u32,
    entries: Vec<SavedEntry>,
}

#[derive(serde::Serialize, serde::Deserialize)]
struct SavedEntry {
    path: PathBuf,
    /// Seconds and nanoseconds since the epoch.
    mtime: Option<(u64, u32)>,
    size: u64,
    asset: AssetRef,
}

const SAVED_VERSION: u32 = 1;
/// How often the index is saved, at most.
const SAVE_EVERY: Duration = Duration::from_secs(1);

/// The backend's state directory: `FACTORY_OS_STATE_DIR`, else `/var/lib/factoryos` on a server
/// (`deploy/provision.sh`), else `$XDG_STATE_HOME/factoryos` or `~/.local/state/factoryos`; the
/// first that is (or can be made) a writable directory.
pub fn default_state_dir() -> Option<PathBuf> {
    let var = |name: &str| std::env::var_os(name).filter(|v| !v.is_empty()).map(PathBuf::from);
    if let Some(dir) = var("FACTORY_OS_STATE_DIR") {
        return Some(dir);
    }
    let server = Path::new("/var/lib/factoryos");
    let local = var("XDG_STATE_HOME").or_else(|| var("HOME").map(|home| home.join(".local/state"))).map(|d| d.join("factoryos"));
    [server.is_dir().then(|| server.to_owned()), local].into_iter().flatten().find(|dir| writable(&dir.join("assets")))
}

/// Whether files can be created in `dir` (made if missing).
fn writable(dir: &Path) -> bool {
    if std::fs::create_dir_all(dir).is_err() {
        return false;
    }
    let probe = dir.join(format!(".probe-{}", std::process::id()));
    let ok = std::fs::write(&probe, b"").is_ok();
    let _ = std::fs::remove_file(&probe);
    ok
}

/// A name for a temporary file next to `path`, unique within this machine.
fn temp_path(path: &Path) -> PathBuf {
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let n = COUNTER.fetch_add(1, Ordering::Relaxed);
    let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
    path.with_file_name(format!(".{name}.{}.{n}.tmp", std::process::id()))
}

/// Writes `bytes` to `path` as one: through a temporary file and a rename.
fn write_atomic(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    let temp = temp_path(path);
    let result = std::fs::write(&temp, bytes).and_then(|()| std::fs::rename(&temp, path));
    if result.is_err() {
        let _ = std::fs::remove_file(&temp);
    }
    result
}

impl AssetIndex {
    /// An index that registers files under `roots` only (canonical directories), with its
    /// converted meshes and saved index in [`default_state_dir`].
    pub fn new(roots: impl Fn() -> Vec<PathBuf> + Send + Sync + 'static) -> Arc<Self> {
        Self::new_in(roots, default_state_dir())
    }

    /// [`Self::new`] with its state in `state_dir` (`<state_dir>/assets/`); `None`: no mesh
    /// conversion, nothing saved.
    pub fn new_in(roots: impl Fn() -> Vec<PathBuf> + Send + Sync + 'static, state_dir: Option<PathBuf>) -> Arc<Self> {
        Self::open(Some(Box::new(roots)), state_dir)
    }

    /// Any directory, without state, for tests.
    #[cfg(test)]
    pub fn unrestricted() -> Arc<Self> {
        Self::open(None, None)
    }

    /// Any directory, with its state in `state_dir`, for tests.
    #[cfg(test)]
    pub fn unrestricted_in(state_dir: &Path) -> Arc<Self> {
        Self::open(None, Some(state_dir.to_owned()))
    }

    fn open(roots: Option<Roots>, state_dir: Option<PathBuf>) -> Arc<Self> {
        let store = state_dir.and_then(|dir| {
            let dir = dir.join("assets");
            let meshes = dir.join("meshes");
            match std::fs::create_dir_all(&meshes).and_then(|()| meshes.canonicalize()) {
                Ok(meshes) => Some(Store { meshes, index_file: dir.join("index.json"), saving: Mutex::new(()) }),
                Err(err) => {
                    tracing::warn!("assets: no state in {}: {err}; meshes are not converted", dir.display());
                    None
                }
            }
        });
        let index = Arc::new(Self {
            inner: Mutex::default(),
            roots,
            store,
            names: Mutex::default(),
            converting: Mutex::default(),
            failed: Mutex::default(),
            conversions: AtomicUsize::new(0),
            on_new: Mutex::default(),
        });
        if index.store.is_some() {
            index.load();
            Self::keep_saved(Arc::downgrade(&index));
        }
        index
    }

    /// Loads the saved index: the entries that still pass [`Self::admit`] (current roots, allowed
    /// types) under their canonical path. Their bytes are checked when served, as always.
    fn load(&self) {
        let Some(store) = &self.store else { return };
        let Ok(bytes) = std::fs::read(&store.index_file) else { return };
        let saved: Saved = match serde_json::from_slice(&bytes) {
            Ok(saved) => saved,
            Err(err) => {
                tracing::warn!("{}: {err}; starting empty", store.index_file.display());
                return;
            }
        };
        if saved.version != SAVED_VERSION {
            return;
        }
        let roots = self.roots.as_ref().map(|roots| roots());
        let total = saved.entries.len();
        let mut inner = self.inner.lock().unwrap();
        for entry in saved.entries {
            let canonical = entry.path.canonicalize().is_ok_and(|c| c == entry.path);
            if !canonical || self.admit_in(&entry.path, roots.as_deref()).is_err() || !is_sha256(&entry.asset.sha256) {
                continue;
            }
            let mtime = entry.mtime.map(|(secs, nanos)| UNIX_EPOCH + Duration::new(secs, nanos));
            inner.insert(entry.path, Entry { stamp: Stamp { mtime, size: entry.size }, asset: entry.asset });
        }
        inner.saved = inner.generation;
        tracing::info!("assets: {} of {total} saved entries loaded from {}", inner.files.len(), store.index_file.display());
    }

    /// Saves the index whenever it changed, at most every [`SAVE_EVERY`], while it lives.
    fn keep_saved(index: Weak<Self>) {
        let spawned = std::thread::Builder::new().name("asset-index-save".into()).spawn(move || {
            loop {
                std::thread::sleep(SAVE_EVERY);
                let Some(index) = index.upgrade() else { return };
                index.save();
            }
        });
        if let Err(err) = spawned {
            tracing::warn!("assets: the index is not saved: {err}");
        }
    }

    /// Writes the index if it changed since it was last written.
    pub fn save(&self) {
        let Some(store) = &self.store else { return };
        let _saving = store.saving.lock().unwrap();
        let (generation, saved) = {
            let inner = self.inner.lock().unwrap();
            if inner.generation == inner.saved {
                return;
            }
            let mut entries: Vec<SavedEntry> = inner
                .files
                .iter()
                .map(|(path, entry)| SavedEntry {
                    path: path.clone(),
                    mtime: entry
                        .stamp
                        .mtime
                        .and_then(|t| t.duration_since(UNIX_EPOCH).ok())
                        .map(|d| (d.as_secs(), d.subsec_nanos())),
                    size: entry.stamp.size,
                    asset: entry.asset.clone(),
                })
                .collect();
            entries.sort_by(|a, b| a.path.cmp(&b.path));
            (inner.generation, Saved { version: SAVED_VERSION, entries })
        };
        let json = serde_json::to_vec(&saved).expect("the asset index as JSON");
        match write_atomic(&store.index_file, &json) {
            Ok(()) => {
                let mut inner = self.inner.lock().unwrap();
                inner.saved = inner.saved.max(generation);
            }
            Err(err) => tracing::warn!("{}: {err}", store.index_file.display()),
        }
    }

    /// Whether the canonical `path` may be registered.
    fn admit(&self, canonical: &Path) -> anyhow::Result<()> {
        let roots = self.roots.as_ref().map(|roots| roots());
        self.admit_in(canonical, roots.as_deref())?;
        if !canonical.is_file() {
            anyhow::bail!("{}: not a file", canonical.display());
        }
        Ok(())
    }

    /// [`Self::admit`] by name: an allowed type under `roots` (`None`: anywhere) or the cache.
    fn admit_in(&self, canonical: &Path, roots: Option<&[PathBuf]>) -> anyhow::Result<()> {
        if !ALLOWED_EXTENSIONS.contains(&extension(canonical).as_str()) {
            anyhow::bail!("{}: not a URDF, mesh, material or texture", canonical.display());
        }
        let cached = self.store.as_ref().is_some_and(|store| canonical.starts_with(&store.meshes));
        if let Some(roots) = roots
            && !cached
            && !roots.iter().any(|root| canonical.starts_with(root))
        {
            anyhow::bail!("{}: outside the backend's asset roots", canonical.display());
        }
        Ok(())
    }

    /// Hashes `path` (unless unchanged since last time) and makes it fetchable by its hash, if it
    /// is an allowed file under the roots.
    pub fn register(&self, path: &Path) -> anyhow::Result<AssetRef> {
        let canonical = path.canonicalize().with_context(|| format!("asset {}", path.display()))?;
        self.admit(&canonical)?;
        let stamp = Stamp::of(&canonical).with_context(|| format!("asset {}", path.display()))?;
        if let Some(entry) = self.inner.lock().unwrap().files.get(&canonical)
            && entry.stamp == stamp
        {
            return Ok(entry.asset.clone());
        }
        let (sha256, size) = hash_file(&canonical).with_context(|| format!("hashing {}", path.display()))?;
        let asset = AssetRef { sha256, size, media_type: media_type(&canonical).to_owned() };
        // The size read, not the size stat'ed: a file growing meanwhile is re-hashed next time.
        let stamp = Stamp { size, ..stamp };
        let new = self.inner.lock().unwrap().insert(canonical, Entry { stamp, asset: asset.clone() });
        if new {
            let on_new = self.on_new.lock().unwrap().clone();
            if let Some(on_new) = on_new {
                on_new(&asset.sha256);
            }
        }
        Ok(asset)
    }

    /// `then(sha256)` for every hash registered from now on that the index didn't have (e.g. to
    /// upload it ahead, storage::assets). One at a time: a second replaces the first.
    pub fn on_new_hash(&self, then: impl Fn(&str) + Send + Sync + 'static) {
        *self.on_new.lock().unwrap() = Some(Arc::new(then));
    }

    /// `path` and the files it references: an OBJ's `mtllib`s and their textures, a glTF's
    /// external buffers and images. Missing sidecars are skipped with a warning. (Collada images
    /// are not followed: the viewer's DAE loading doesn't texture.)
    ///
    /// OBJ and STL meshes also come as GLB and LOD ([`mesh_convert`]), converted now unless
    /// cached; `None` where that isn't possible (no state directory, a mesh that doesn't parse).
    pub fn mesh_asset(&self, path: &Path) -> anyhow::Result<MeshAsset> {
        let asset = self.register(path)?;
        let mut sidecars = BTreeMap::new();
        let dir = path.parent().unwrap_or(Path::new("."));
        let add = |sidecars: &mut BTreeMap<String, AssetRef>, name: &str, file: Option<PathBuf>, of: &Path| match file {
            Some(file) => match self.register(&file) {
                Ok(asset) => {
                    sidecars.insert(name.to_owned(), asset.clone());
                    Some(asset)
                }
                Err(err) => {
                    tracing::warn!("{}: sidecar {name}: {err:#}", of.display());
                    None
                }
            },
            None => {
                tracing::warn!("{}: sidecar {name} not found, skipped", of.display());
                None
            }
        };
        match extension(path).as_str() {
            "obj" => {
                for name in self.names_in(path, &asset, dir, |text| obj_mtllibs(text, dir))? {
                    let mtl = dir.join(&name);
                    let found = mtl.is_file().then_some(mtl.clone());
                    if let Some(mtl_asset) = add(&mut sidecars, &name, found, path) {
                        let mtl_dir = mtl.parent().unwrap_or(dir);
                        let Ok(textures) = self.names_in(&mtl, &mtl_asset, mtl_dir, |text| mtl_textures(text, mtl_dir)) else {
                            continue;
                        };
                        for texture in textures {
                            let file = mtl_dir.join(&texture);
                            add(&mut sidecars, &texture, file.is_file().then_some(file), &mtl);
                        }
                    }
                }
            }
            "gltf" => {
                let json: serde_json::Value =
                    serde_json::from_str(&read_text(path)?).with_context(|| format!("parsing {}", path.display()))?;
                for uri in gltf_uris(&json) {
                    let file = dir.join(percent_decode(&uri));
                    add(&mut sidecars, &uri, file.is_file().then_some(file), path);
                }
            }
            _ => {}
        }
        let (glb, lod) = self.converted(path, &asset, &sidecars);
        Ok(MeshAsset { asset, sidecars, glb, lod })
    }

    /// The sidecar names `find` reads from the text of `file` (registered as `asset`), once per
    /// content and directory.
    fn names_in(&self, file: &Path, asset: &AssetRef, dir: &Path, find: impl Fn(&str) -> Vec<String>) -> anyhow::Result<Vec<String>> {
        let key = format!("{}:{}", asset.sha256, dir.display());
        if let Some(names) = self.names.lock().unwrap().get(&key) {
            return Ok(names.clone());
        }
        let bytes = std::fs::read(file).with_context(|| format!("reading {}", file.display()))?;
        let names = find(&String::from_utf8_lossy(&bytes));
        // Only under the hash the names were read from.
        if hex_sha256(&bytes) == asset.sha256 {
            self.names.lock().unwrap().insert(key, names.clone());
        }
        Ok(names)
    }

    /// Meshes converted so far by this index (not cache hits).
    pub fn conversions(&self) -> usize {
        self.conversions.load(Ordering::Relaxed)
    }

    /// The GLB and LOD of mesh `path` (registered as `asset`, with `sidecars`): from the cache, or
    /// converted now and cached.
    fn converted(&self, path: &Path, asset: &AssetRef, sidecars: &BTreeMap<String, AssetRef>) -> (Option<AssetRef>, Option<AssetRef>) {
        let (Some(source), Some(store)) = (mesh_convert::Source::of(path), &self.store) else { return (None, None) };
        let key = conversion_key(source, asset, sidecars);
        if self.failed.lock().unwrap().contains(&key) {
            return (None, None);
        }
        let glb_file = store.meshes.join(format!("{key}.glb"));
        let lod_file = store.meshes.join(format!("{key}.lod.glb"));
        {
            let lock = self.converting.lock().unwrap().entry(key.clone()).or_default().clone();
            let _converting = lock.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
            // The GLB is written last: with it, the conversion is complete (a missing LOD: none).
            if !glb_file.is_file() {
                match self.convert(source, path, asset, sidecars) {
                    Ok(Some(converted)) => {
                        let lod = converted.lod.as_ref().map_or(Ok(()), |lod| write_atomic(&lod_file, lod));
                        if let Err(err) = lod.and_then(|()| write_atomic(&glb_file, &converted.glb)) {
                            tracing::warn!("{}: caching its GLB: {err}", path.display());
                            return (None, None);
                        }
                        self.conversions.fetch_add(1, Ordering::Relaxed);
                        tracing::debug!(
                            "{}: GLB {} bytes, LOD {:?} bytes ({} → {:?} triangles)",
                            path.display(),
                            converted.glb.len(),
                            converted.lod.as_ref().map(Vec::len),
                            converted.triangles,
                            converted.lod_triangles
                        );
                    }
                    // Changed meanwhile: the next call has its new hash.
                    Ok(None) => return (None, None),
                    Err(err) => {
                        tracing::warn!("{}: not converted to GLB: {err:#}", path.display());
                        self.failed.lock().unwrap().insert(key);
                        return (None, None);
                    }
                }
            }
        }
        let glb = self.register(&glb_file).ok();
        let lod = glb.as_ref().and_then(|_| lod_file.is_file().then(|| self.register(&lod_file).ok()).flatten());
        (glb, lod)
    }

    /// Converts mesh `path` from bytes that hash as registered; `Ok(None)` if they no longer do.
    fn convert(
        &self,
        source: mesh_convert::Source,
        path: &Path,
        asset: &AssetRef,
        sidecars: &BTreeMap<String, AssetRef>,
    ) -> anyhow::Result<Option<mesh_convert::Converted>> {
        let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
        if hex_sha256(&bytes) != asset.sha256 {
            return Ok(None);
        }
        let dir = path.parent().unwrap_or(Path::new("."));
        let changed = std::cell::Cell::new(false);
        let load_sidecar = |name: &str| {
            let sidecar = sidecars.get(name)?;
            let bytes = std::fs::read(dir.join(name)).ok()?;
            if hex_sha256(&bytes) != sidecar.sha256 {
                changed.set(true);
                return None;
            }
            Some(bytes)
        };
        let converted = mesh_convert::convert(source, &bytes, &load_sidecar)?;
        Ok((!changed.get()).then_some(converted))
    }

    /// `urdf_path` and every visual and collision mesh it references (with their sidecars), keyed
    /// by `<mesh filename>` as written. Meshes that can't be resolved locally (`http(s)://`,
    /// missing files) are skipped with a warning.
    pub fn model_ref(&self, urdf_path: &Path) -> anyhow::Result<ModelRef> {
        self.model_ref_with(urdf_path, &[])
    }

    /// [`Self::model_ref`], with meshes [`resolve_mesh`] doesn't find also looked for in
    /// `search`: see [`resolve_mesh_in`].
    pub fn model_ref_with(&self, urdf_path: &Path, search: &[PathBuf]) -> anyhow::Result<ModelRef> {
        let robot = urdf_rs::read_file(urdf_path).with_context(|| format!("parsing {}", urdf_path.display()))?;
        let urdf = self.register(urdf_path)?;
        let urdf_dir = urdf_path.parent();
        let filenames: BTreeSet<&str> = robot
            .links
            .iter()
            .flat_map(|link| {
                let visuals = link.visual.iter().map(|v| &v.geometry);
                visuals.chain(link.collision.iter().map(|c| &c.geometry))
            })
            .filter_map(|geometry| match geometry {
                urdf_rs::Geometry::Mesh { filename, .. } => Some(filename.as_str()),
                _ => None,
            })
            .collect();
        let mut meshes = BTreeMap::new();
        for filename in filenames {
            let Some(path) = resolve_mesh_in(filename, urdf_dir, search) else {
                tracing::warn!("{}: mesh {filename} not found, skipped", urdf_path.display());
                continue;
            };
            match self.mesh_asset(&path) {
                Ok(mesh) => {
                    meshes.insert(filename.to_owned(), mesh);
                }
                Err(err) => tracing::warn!("{}: mesh {filename}: {err:#}", urdf_path.display()),
            }
        }
        Ok(ModelRef { urdf, meshes })
    }

    /// Whether `sha256` is registered (without checking the file).
    pub fn contains(&self, sha256: &str) -> bool {
        self.inner.lock().unwrap().by_hash.contains_key(sha256)
    }

    /// A file whose bytes currently hash to `sha256`, if one is registered. A file that changed on
    /// disk since it was hashed is re-hashed first (and re-indexed under its new hash).
    pub fn path_for(&self, sha256: &str) -> Option<PathBuf> {
        self.find(sha256, |path, entry| {
            if Stamp::of(path).ok() == Some(entry.stamp) {
                return Verified::Yes(path.to_owned());
            }
            match self.rehash(path) {
                Some(asset) if asset.sha256 == sha256 => Verified::Yes(path.to_owned()),
                _ => Verified::No,
            }
        })
    }

    /// The registered asset `sha256` and its bytes, read and checked as one: a file that changed
    /// meanwhile is hashed from the bytes read, so the bytes always match the hash.
    pub fn read(&self, sha256: &str) -> Option<(AssetRef, Vec<u8>)> {
        self.find(sha256, |path, entry| {
            let Ok(bytes) = std::fs::read(path) else {
                self.inner.lock().unwrap().forget(path);
                return Verified::No;
            };
            // Always hashed: a same-size edit within the filesystem's mtime resolution keeps the
            // stamp, and a wrong answer would be cached for a year.
            if hex_sha256(&bytes) == sha256 {
                return Verified::Yes((entry.asset.clone(), bytes));
            }
            self.rehash(path);
            Verified::No
        })
    }

    /// The first of `sha256`'s paths that `verify` accepts.
    fn find<T>(&self, sha256: &str, verify: impl Fn(&Path, &Entry) -> Verified<T>) -> Option<T> {
        let candidates: Vec<(PathBuf, Entry)> = {
            let inner = self.inner.lock().unwrap();
            let paths = inner.by_hash.get(sha256)?;
            paths.iter().filter_map(|p| Some((p.clone(), inner.files.get(p)?.clone()))).collect()
        };
        candidates.iter().find_map(|(path, entry)| match verify(path, entry) {
            Verified::Yes(found) => Some(found),
            Verified::No => None,
        })
    }

    /// Re-registers `path` from its current bytes; forgets it if it's gone.
    fn rehash(&self, path: &Path) -> Option<AssetRef> {
        match self.register(path) {
            Ok(asset) => Some(asset),
            Err(_) => {
                self.inner.lock().unwrap().forget(path);
                None
            }
        }
    }
}

enum Verified<T> {
    Yes(T),
    No,
}

fn hash_file(path: &Path) -> std::io::Result<(String, u64)> {
    let mut file = std::fs::File::open(path)?;
    let mut hasher = Sha256::new();
    let mut buf = vec![0; 1 << 16];
    let mut size = 0u64;
    loop {
        let n = file.read(&mut buf)?;
        if n == 0 {
            break;
        }
        hasher.update(&buf[..n]);
        size += n as u64;
    }
    Ok((format!("{:x}", hasher.finalize()), size))
}

fn hex_sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

/// What a converted mesh is cached under: its source's and sidecars' hashes, the converter.
fn conversion_key(source: mesh_convert::Source, asset: &AssetRef, sidecars: &BTreeMap<String, AssetRef>) -> String {
    let mut hasher = Sha256::new();
    hasher.update(format!("factoryos mesh_convert {} {}\n{}\n", mesh_convert::VERSION, source.name(), asset.sha256));
    for (name, sidecar) in sidecars {
        hasher.update(format!("{name}\0{}\n", sidecar.sha256));
    }
    format!("{:x}", hasher.finalize())
}

fn extension(path: &Path) -> String {
    path.extension().and_then(|e| e.to_str()).unwrap_or_default().to_ascii_lowercase()
}

/// The media type of an asset, by extension.
pub fn media_type(path: &Path) -> &'static str {
    match extension(path).as_str() {
        "obj" => "model/obj",
        "stl" => "model/stl",
        "glb" => "model/gltf-binary",
        "gltf" => "model/gltf+json",
        "dae" => "model/vnd.collada+xml",
        "urdf" | "xml" => "application/xml",
        "mtl" => "text/plain",
        "png" => "image/png",
        "jpg" | "jpeg" => "image/jpeg",
        _ => "application/octet-stream",
    }
}

fn read_text(path: &Path) -> anyhow::Result<String> {
    let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    Ok(String::from_utf8_lossy(&bytes).into_owned())
}

/// The value of an OBJ/MTL statement line `<keyword> <rest>`.
fn statement<'a>(line: &'a str, keywords: &[&str]) -> Option<&'a str> {
    let line = line.trim();
    let (keyword, rest) = line.split_once(char::is_whitespace)?;
    keywords.contains(&keyword).then(|| rest.trim()).filter(|rest| !rest.is_empty())
}

/// An OBJ's `mtllib` file names as written. `mtllib a.mtl b.mtl` names two files, unless a file
/// `a.mtl b.mtl` exists (as tobj reads the line).
fn obj_mtllibs(obj: &str, dir: &Path) -> Vec<String> {
    let mut names = Vec::new();
    for rest in obj.lines().filter_map(|line| statement(line, &["mtllib"])) {
        if dir.join(rest).is_file() || !rest.contains(char::is_whitespace) {
            names.push(rest.to_owned());
        } else {
            names.extend(rest.split_whitespace().map(str::to_owned));
        }
    }
    names.dedup();
    names
}

/// An MTL's texture file names as written: the last argument of each map statement (after its
/// `-option value` arguments), or all of them if that names a file (a name with spaces).
fn mtl_textures(mtl: &str, dir: &Path) -> Vec<String> {
    const MAPS: &[&str] = &[
        "map_Ka", "map_Kd", "map_Ks", "map_Ke", "map_Ns", "map_d", "map_bump", "map_Bump", "bump", "disp", "decal",
        "refl", "norm", "map_Pr", "map_Pm", "map_Ps", "map_Tr",
    ];
    let mut names = BTreeSet::new();
    for rest in mtl.lines().filter_map(|line| statement(line, MAPS)) {
        let name = if dir.join(rest).is_file() { rest } else { rest.split_whitespace().last().unwrap_or(rest) };
        names.insert(name.to_owned());
    }
    names.into_iter().collect()
}

/// A glTF's external `buffers[].uri` and `images[].uri` (not `data:` URIs).
fn gltf_uris(json: &serde_json::Value) -> Vec<String> {
    let uris = ["buffers", "images"].into_iter().flat_map(|key| {
        json.get(key)
            .and_then(|v| v.as_array())
            .into_iter()
            .flatten()
            .filter_map(|item| item.get("uri")?.as_str())
    });
    uris.filter(|uri| !uri.starts_with("data:") && !uri.contains("://"))
        .map(str::to_owned)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect()
}

/// `%20` etc. in a glTF URI.
fn percent_decode(uri: &str) -> String {
    let bytes = uri.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'%'
            && let Some(byte) = uri.get(i + 1..i + 3).and_then(|hex| u8::from_str_radix(hex, 16).ok())
        {
            out.push(byte);
            i += 3;
            continue;
        }
        out.push(bytes[i]);
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// A URDF `<mesh filename>` as a local file: relative to the URDF's directory, `file://`, or
/// `package://` ([`resolve_package`]). `None` for remote meshes and missing files.
pub fn resolve_mesh(filename: &str, urdf_dir: Option<&Path>) -> Option<PathBuf> {
    let filename = filename.split(['?', '#']).next().unwrap_or(filename);
    let path = match filename.split_once("://") {
        Some(("file", path)) => PathBuf::from(path),
        Some(("package", path)) => resolve_package(path, urdf_dir)?,
        Some(_) => return None,
        None if Path::new(filename).is_absolute() => PathBuf::from(filename),
        None => urdf_dir?.join(filename),
    };
    path.is_file().then_some(path)
}

/// [`resolve_mesh`], else in the folders `search`, in order: `package://<package>/<path>` as
/// `<dir>/<package>/<path>` or `<dir>/<path>`, a relative filename as `<dir>/<filename>`.
pub fn resolve_mesh_in(filename: &str, urdf_dir: Option<&Path>, search: &[PathBuf]) -> Option<PathBuf> {
    if let Some(path) = resolve_mesh(filename, urdf_dir) {
        return Some(path);
    }
    let filename = filename.split(['?', '#']).next().unwrap_or(filename);
    let candidates: Vec<PathBuf> = match filename.split_once("://") {
        Some(("package", path)) => {
            let (package, rest) = path.split_once('/')?;
            search.iter().flat_map(|dir| [dir.join(package).join(rest), dir.join(rest)]).collect()
        }
        None if Path::new(filename).is_relative() => search.iter().map(|dir| dir.join(filename)).collect(),
        _ => return None,
    };
    candidates.into_iter().find(|path| path.is_file())
}

/// `package://<package>/<path>`, looked up in `ROS_PACKAGE_PATH` (ROS 1) and `AMENT_PREFIX_PATH`
/// (ROS 2), falling back to the URDF's own and parent directories named `<package>`, as Rerun's
/// importer finds it.
fn resolve_package(path: &str, urdf_dir: Option<&Path>) -> Option<PathBuf> {
    let (package, rest) = path.split_once('/')?;
    let env_paths = |var: &str| -> Vec<PathBuf> {
        std::env::var_os(var).map(|value| std::env::split_paths(&value).collect()).unwrap_or_default()
    };
    let mut candidates = Vec::new();
    for root in env_paths("ROS_PACKAGE_PATH") {
        candidates.push(root.join(package));
    }
    for prefix in env_paths("AMENT_PREFIX_PATH") {
        candidates.push(prefix.join("share").join(package));
    }
    for dir in urdf_dir.into_iter().flat_map(Path::ancestors) {
        if dir.file_name().is_some_and(|name| name == package) {
            candidates.push(dir.to_path_buf());
        }
    }
    candidates.into_iter().map(|package_dir| package_dir.join(rest)).find(|path| path.is_file())
}

/// `GET` [`api::ASSET`]: a registered asset's bytes, gzipped where that helps.
pub fn router<S>() -> Router<S>
where
    Arc<AssetIndex>: FromRef<S>,
    S: Clone + Send + Sync + 'static,
{
    Router::new()
        .route(api::ASSET, get(get_asset))
        .layer(tower_http::compression::CompressionLayer::new())
        .layer(axum::middleware::map_response(tag_encoded))
}

fn is_sha256(s: &str) -> bool {
    s.len() == 64 && s.bytes().all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f'))
}

/// The entity tag of `If-None-Match` that matches asset `sha256`, if any (`"<sha256>"` or a
/// content-coded `"<sha256>-gzip"`, see [`tag_encoded`]).
fn matching_tag<'a>(headers: &'a HeaderMap, sha256: &str) -> Option<&'a str> {
    let value = headers.get(header::IF_NONE_MATCH)?.to_str().ok()?;
    value.split(',').map(str::trim).find(|tag| {
        let opaque = tag.trim_start_matches("W/").trim_matches('"');
        *tag == "*" || opaque.split('-').next() == Some(sha256)
    })
}

async fn get_asset(State(index): State<Arc<AssetIndex>>, UrlPath(sha256): UrlPath<String>, headers: HeaderMap) -> Response {
    if !is_sha256(&sha256) {
        return (StatusCode::BAD_REQUEST, "expected a lowercase hex SHA-256").into_response();
    }
    if !index.contains(&sha256) {
        return StatusCode::NOT_FOUND.into_response();
    }
    // The bytes of a hash never change: whatever the client has under it is current.
    if let Some(tag) = matching_tag(&headers, &sha256) {
        let etag = if tag == "*" { format!("\"{sha256}\"") } else { tag.to_owned() };
        let mut response = StatusCode::NOT_MODIFIED.into_response();
        set_headers(response.headers_mut(), &etag, None);
        return response;
    }
    let read = {
        let sha256 = sha256.clone();
        tokio::task::spawn_blocking(move || index.read(&sha256)).await
    };
    let Ok(Some((asset, bytes))) = read else {
        return StatusCode::NOT_FOUND.into_response();
    };
    let mut response = bytes.into_response();
    set_headers(response.headers_mut(), &format!("\"{sha256}\""), Some(&asset.media_type));
    response
}

fn set_headers(headers: &mut HeaderMap, etag: &str, media_type: Option<&str>) {
    headers.insert(header::CACHE_CONTROL, HeaderValue::from_static(CACHE_CONTROL));
    if let Ok(etag) = HeaderValue::from_str(etag) {
        headers.insert(header::ETAG, etag);
    }
    if let Some(media_type) = media_type.and_then(|m| HeaderValue::from_str(m).ok()) {
        headers.insert(header::CONTENT_TYPE, media_type);
    }
    // Assets are data for the viewer, never documents of this origin: an XML (URDF) opened
    // directly must not run script here, and no type is sniffed into something else.
    headers.insert(header::X_CONTENT_TYPE_OPTIONS, HeaderValue::from_static("nosniff"));
    headers.insert(header::CONTENT_SECURITY_POLICY, HeaderValue::from_static("default-src 'none'; sandbox"));
}

/// A content-coded response is another representation, so its strong `ETag` differs:
/// `"<sha256>-gzip"`.
async fn tag_encoded(mut response: Response) -> Response {
    let coding = response.headers().get(header::CONTENT_ENCODING).and_then(|v| v.to_str().ok()).map(str::to_owned);
    let etag = response.headers().get(header::ETAG).and_then(|v| v.to_str().ok()).map(str::to_owned);
    if let (Some(coding), Some(etag)) = (coding, etag)
        && let Ok(tagged) = HeaderValue::from_str(&format!("{}-{coding}\"", etag.trim_end_matches('"')))
    {
        response.headers_mut().insert(header::ETAG, tagged);
    }
    response
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::body::Body;
    use axum::http::Request;
    use tower::ServiceExt as _;

    fn temp_dir(tag: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("factoryos-assets-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    const HELLO_SHA256: &str = "2cf24dba5fb0a30e26e83b2ac5b9e29e1b161e5c1fa7425e73043362938b9824";

    #[test]
    fn only_allowed_files_under_the_roots_are_registered() {
        let dir = temp_dir("roots");
        let root = dir.join("catalogue");
        std::fs::create_dir_all(&root).unwrap();
        std::fs::write(dir.join("secret.mtl"), "newmtl a\nKd 1 0 0\n").unwrap();
        std::fs::write(dir.join("id_rsa.obj"), "v 0 0 0\n").unwrap();
        std::fs::write(root.join("notes.yaml"), "x: 1\n").unwrap();
        std::fs::write(root.join("part.obj"), "mtllib ../secret.mtl\nmtllib /etc/hosts\nv 0 0 0\n").unwrap();
        #[cfg(unix)]
        std::os::unix::fs::symlink(dir.join("id_rsa.obj"), root.join("link.obj")).unwrap();
        let canonical_root = root.canonicalize().unwrap();
        let index = AssetIndex::new(move || vec![canonical_root.clone()]);

        assert!(index.register(&dir.join("id_rsa.obj")).is_err(), "outside the roots");
        assert!(index.register(&root.join("../id_rsa.obj")).is_err(), "a ../ escape");
        assert!(index.register(&root.join("notes.yaml")).is_err(), "not an asset type");
        #[cfg(unix)]
        assert!(index.register(&root.join("link.obj")).is_err(), "a link out of the roots");
        // Sidecars named by the file go through the same checks: left out, not served.
        let mesh = index.mesh_asset(&root.join("part.obj")).unwrap();
        assert!(mesh.sidecars.is_empty(), "{:?}", mesh.sidecars);
        assert!(index.contains(&mesh.asset.sha256));
    }

    #[test]
    fn register_hashes_and_dedupes() {
        let dir = temp_dir("register");
        std::fs::write(dir.join("a.obj"), "hello").unwrap();
        std::fs::write(dir.join("b.stl"), "hello").unwrap();
        let index = AssetIndex::unrestricted();
        let a = index.register(&dir.join("a.obj")).unwrap();
        assert_eq!(a, AssetRef { sha256: HELLO_SHA256.into(), size: 5, media_type: "model/obj".into() });
        assert_eq!(index.register(&dir.join("./a.obj")).unwrap(), a);
        let b = index.register(&dir.join("b.stl")).unwrap();
        assert_eq!((b.sha256.as_str(), b.media_type.as_str()), (HELLO_SHA256, "model/stl"));
        assert_eq!(index.inner.lock().unwrap().by_hash[HELLO_SHA256].len(), 2);
        assert!(index.path_for(HELLO_SHA256).is_some());
        assert!(index.path_for(&"0".repeat(64)).is_none());
        assert!(index.register(&dir.join("missing.obj")).is_err());
    }

    #[test]
    fn changed_files_are_rehashed_not_served_stale() {
        let dir = temp_dir("changed");
        let file = dir.join("m.obj");
        std::fs::write(&file, "hello").unwrap();
        let index = AssetIndex::unrestricted();
        let old = index.register(&file).unwrap();
        std::fs::write(&file, "hello, world").unwrap();
        // Other size: a change even within the mtime's resolution.
        assert_eq!(index.path_for(&old.sha256), None);
        assert!(index.read(&old.sha256).is_none());
        assert!(!index.contains(&old.sha256));
        let new = index.register(&file).unwrap();
        assert_ne!(new.sha256, old.sha256);
        assert_eq!(new.size, 12);
        let (asset, bytes) = index.read(&new.sha256).unwrap();
        assert_eq!((asset, bytes), (new.clone(), b"hello, world".to_vec()));
        std::fs::remove_file(&file).unwrap();
        assert!(index.read(&new.sha256).is_none());
        assert!(!index.contains(&new.sha256));
    }

    #[test]
    fn obj_sidecars() {
        let dir = temp_dir("obj");
        std::fs::create_dir_all(dir.join("tex")).unwrap();
        std::fs::write(dir.join("part.obj"), "mtllib part.mtl\nmtllib gone.mtl\nv 0 0 0\n").unwrap();
        std::fs::write(dir.join("part.mtl"), "newmtl a\nKd 1 0 0\nmap_Kd -s 1 1 1 tex/albedo.png\nmap_Bump missing.png\n").unwrap();
        std::fs::write(dir.join("tex/albedo.png"), [0x89, b'P', b'N', b'G']).unwrap();
        let mesh = AssetIndex::unrestricted().mesh_asset(&dir.join("part.obj")).unwrap();
        assert_eq!(mesh.asset.media_type, "model/obj");
        let keys: Vec<_> = mesh.sidecars.keys().map(String::as_str).collect();
        assert_eq!(keys, ["part.mtl", "tex/albedo.png"]);
        assert_eq!(mesh.sidecars["part.mtl"].media_type, "text/plain");
        assert_eq!(mesh.sidecars["tex/albedo.png"].media_type, "image/png");
    }

    #[test]
    fn gltf_sidecars() {
        let dir = temp_dir("gltf");
        std::fs::write(
            dir.join("m.gltf"),
            r#"{"buffers":[{"uri":"m%20data.bin"},{"uri":"data:application/octet-stream;base64,AA=="}],"images":[{"uri":"t.png"}]}"#,
        )
        .unwrap();
        std::fs::write(dir.join("m data.bin"), [0u8; 4]).unwrap();
        let mesh = AssetIndex::unrestricted().mesh_asset(&dir.join("m.gltf")).unwrap();
        assert_eq!(mesh.asset.media_type, "model/gltf+json");
        assert_eq!(mesh.sidecars.keys().collect::<Vec<_>>(), ["m%20data.bin"]);
    }

    #[test]
    fn model_ref_resolves_relative_and_package_meshes() {
        let root = temp_dir("urdf");
        let pkg = root.join("my_robot");
        std::fs::create_dir_all(pkg.join("urdf")).unwrap();
        std::fs::create_dir_all(pkg.join("meshes")).unwrap();
        std::fs::write(pkg.join("meshes/base.obj"), "mtllib base.mtl\nv 0 0 0\n").unwrap();
        std::fs::write(pkg.join("meshes/base.mtl"), "newmtl a\nKd 1 1 1\n").unwrap();
        std::fs::write(pkg.join("meshes/link1.stl"), "solid x\nendsolid x\n").unwrap();
        let urdf = pkg.join("urdf/robot.urdf");
        std::fs::write(
            &urdf,
            r#"<robot name="r">
                <link name="base">
                  <visual><geometry><mesh filename="../meshes/base.obj"/></geometry></visual>
                  <collision><geometry><mesh filename="package://my_robot/meshes/link1.stl"/></geometry></collision>
                </link>
                <link name="l1">
                  <visual><geometry><mesh filename="package://my_robot/meshes/missing.dae"/></geometry></visual>
                  <visual><geometry><mesh filename="https://example.com/x.obj"/></geometry></visual>
                  <collision><geometry><box size="1 1 1"/></geometry></collision>
                </link>
                <joint name="j1" type="revolute">
                  <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
                  <limit lower="-1" upper="1" effort="1" velocity="1"/>
                </joint>
              </robot>"#,
        )
        .unwrap();
        let index = AssetIndex::unrestricted();
        let model = index.model_ref(&urdf).unwrap();
        assert_eq!(model.urdf.media_type, "application/xml");
        let keys: Vec<_> = model.meshes.keys().map(String::as_str).collect();
        assert_eq!(keys, ["../meshes/base.obj", "package://my_robot/meshes/link1.stl"]);
        assert_eq!(model.meshes["../meshes/base.obj"].sidecars.keys().collect::<Vec<_>>(), ["base.mtl"]);
        assert_eq!(model.meshes["package://my_robot/meshes/link1.stl"].asset.media_type, "model/stl");
        assert!(index.path_for(&model.urdf.sha256).is_some());
        let file_url = format!("file://{}", pkg.join("meshes/link1.stl").display());
        assert!(resolve_mesh(&file_url, None).is_some());
    }

    #[test]
    fn model_ref_with_search_dirs() {
        let dir = temp_dir("search");
        // A product: its URDF in `urdf/robot/`, the meshes in `urdf/` (no `meshes` package folder).
        let search = dir.join("urdf");
        std::fs::create_dir_all(search.join("robot")).unwrap();
        std::fs::create_dir_all(search.join("meshes")).unwrap();
        std::fs::write(search.join("meshes/part.stl"), "solid").unwrap();
        std::fs::write(search.join("plate.obj"), "v 0 0 0").unwrap();
        let urdf = search.join("robot/part.urdf");
        std::fs::write(
            &urdf,
            r#"<robot name="part"><link name="base">
                  <visual><geometry><mesh filename="package://part_description/meshes/part.stl"/></geometry></visual>
                  <collision><geometry><mesh filename="plate.obj"/></geometry></collision>
                </link></robot>"#,
        )
        .unwrap();
        let index = AssetIndex::unrestricted();
        assert!(index.model_ref(&urdf).unwrap().meshes.is_empty(), "not found without the search dirs");
        let model = index.model_ref_with(&urdf, &[search.clone()]).unwrap();
        let keys: Vec<_> = model.meshes.keys().map(String::as_str).collect();
        assert_eq!(keys, ["package://part_description/meshes/part.stl", "plate.obj"]);
        assert_eq!(resolve_mesh_in("package://p/meshes/part.stl", None, &[search.clone()]), Some(search.join("meshes/part.stl")));
        assert_eq!(resolve_mesh_in("https://example.com/plate.obj", None, &[search]), None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn model_ref_of_the_starter_robot() {
        let urdf = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/catalogue/urdf-robot-collection/kr240r2900/kr240r2900.urdf");
        if !urdf.is_file() {
            eprintln!("Skipping: no {}", urdf.display());
            return;
        }
        let robot = urdf_rs::read_file(&urdf).unwrap();
        let mesh_count: BTreeSet<_> = robot
            .links
            .iter()
            .flat_map(|l| l.visual.iter().map(|v| &v.geometry).chain(l.collision.iter().map(|c| &c.geometry)))
            .filter_map(|g| match g {
                urdf_rs::Geometry::Mesh { filename, .. } => Some(filename.clone()),
                _ => None,
            })
            .collect();
        let model = AssetIndex::unrestricted().model_ref(&urdf).unwrap();
        assert!(!model.meshes.is_empty());
        assert_eq!(model.meshes.len(), mesh_count.len());
        for (filename, mesh) in &model.meshes {
            assert_eq!(mesh.asset.media_type, "model/obj", "{filename}");
            assert_eq!(mesh.sidecars.len(), 1, "{filename}: its .mtl");
        }
    }

    #[test]
    fn meshes_are_converted_once_and_cached() {
        let dir = temp_dir("convert");
        std::fs::write(dir.join("part.obj"), "mtllib part.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nusemtl a\nf 1 2 3\n").unwrap();
        std::fs::write(dir.join("part.mtl"), "newmtl a\nKd 1 0 0\n").unwrap();
        std::fs::write(dir.join("plain.dae"), "<COLLADA/>").unwrap();
        let state = dir.join("state");
        let index = AssetIndex::unrestricted_in(&state);
        let mesh = index.mesh_asset(&dir.join("part.obj")).unwrap();
        let glb = mesh.glb.clone().expect("a GLB");
        assert_eq!(glb.media_type, "model/gltf-binary");
        assert_eq!(mesh.lod, None, "one triangle: no LOD");
        assert_eq!(index.conversions(), 1);
        let (_, bytes) = index.read(&glb.sha256).expect("the GLB is served");
        assert_eq!(&bytes[..4], b"glTF");
        assert_eq!(index.mesh_asset(&dir.join("part.obj")).unwrap(), mesh);
        assert_eq!(index.conversions(), 1, "a cache hit");
        // A new index (a restart) finds it in the cache.
        let again = AssetIndex::unrestricted_in(&state);
        assert_eq!(again.mesh_asset(&dir.join("part.obj")).unwrap(), mesh);
        assert_eq!(again.conversions(), 0);
        // Another MTL is another key. (Another size, too: changes are noticed by mtime and size, and a
        // same-size rewrite can land on the same mtime on a coarse filesystem, as in CI's containers.)
        std::fs::write(dir.join("part.mtl"), "newmtl a\nKd 0 1 0.5\n").unwrap();
        let green = again.mesh_asset(&dir.join("part.obj")).unwrap();
        assert_ne!(green.glb, mesh.glb);
        assert_eq!(again.conversions(), 1);
        // Not converted: DAE; without a state directory nothing is.
        assert_eq!(again.mesh_asset(&dir.join("plain.dae")).unwrap().glb, None);
        assert_eq!(AssetIndex::unrestricted().mesh_asset(&dir.join("part.obj")).unwrap().glb, None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_index_is_saved_and_loaded() {
        let dir = temp_dir("persist");
        std::fs::create_dir_all(dir.join("cell")).unwrap();
        let root = dir.join("cell").canonicalize().unwrap();
        std::fs::write(root.join("robot.urdf"), "<robot name='r'/>").unwrap();
        std::fs::write(root.join("m.obj"), "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n").unwrap();
        std::fs::write(root.join("gone.stl"), "solid").unwrap();
        let state = dir.join("state");
        let roots = {
            let root = root.clone();
            move || vec![root.clone()]
        };
        let index = AssetIndex::new_in(roots.clone(), Some(state.clone()));
        let urdf = index.register(&root.join("robot.urdf")).unwrap();
        let mesh = index.mesh_asset(&root.join("m.obj")).unwrap();
        let gone = index.register(&root.join("gone.stl")).unwrap();
        index.save();
        let saved: Saved = serde_json::from_slice(&std::fs::read(state.join("assets/index.json")).unwrap()).unwrap();
        assert_eq!(saved.entries.len(), 4, "URDF, OBJ, its GLB, STL");
        std::fs::remove_file(root.join("gone.stl")).unwrap();

        // After a restart: served without registering again.
        let loaded = AssetIndex::new_in(roots, Some(state.clone()));
        assert!(loaded.contains(&urdf.sha256) && loaded.contains(&mesh.asset.sha256));
        assert_eq!(loaded.read(&urdf.sha256).unwrap().1, b"<robot name='r'/>");
        let glb = mesh.glb.unwrap();
        assert_eq!(loaded.read(&glb.sha256).unwrap().0, glb);
        assert!(!loaded.contains(&gone.sha256), "missing files are dropped at load");
        // Entries outside the current roots are dropped.
        let elsewhere = AssetIndex::new_in(|| vec![PathBuf::from("/nonexistent")], Some(state.clone()));
        assert!(!elsewhere.contains(&urdf.sha256));
        assert!(elsewhere.contains(&glb.sha256), "its own cache is trusted");
        // So are tampered entries: a disallowed type, or a path that isn't canonical.
        std::fs::write(root.join("secret.yaml"), "x").unwrap();
        let tampered = Saved {
            version: SAVED_VERSION,
            entries: vec![
                SavedEntry { path: root.join("secret.yaml"), mtime: None, size: 1, asset: AssetRef { sha256: "a".repeat(64), ..urdf.clone() } },
                SavedEntry { path: root.join("../cell/robot.urdf"), mtime: None, size: 1, asset: AssetRef { sha256: "b".repeat(64), ..urdf.clone() } },
            ],
        };
        std::fs::write(state.join("assets/index.json"), serde_json::to_vec(&tampered).unwrap()).unwrap();
        let index = AssetIndex::new_in(move || vec![root.clone()], Some(state));
        assert!(!index.contains(&"a".repeat(64)) && !index.contains(&"b".repeat(64)));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_starter_robot_converts() {
        let urdf = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/catalogue/urdf-robot-collection/kr240r2900/kr240r2900.urdf");
        if !urdf.is_file() {
            eprintln!("Skipping: no {}", urdf.display());
            return;
        }
        let state = temp_dir("starter");
        let model = AssetIndex::unrestricted_in(&state).model_ref(&urdf).unwrap();
        for (filename, mesh) in &model.meshes {
            let (glb, lod) = (mesh.glb.as_ref().expect(filename), mesh.lod.as_ref());
            assert!(glb.size < mesh.asset.size, "{filename}");
            if filename.contains("visuals") {
                assert!(lod.expect(filename).size * 4 < glb.size, "{filename}");
            }
        }
        let _ = std::fs::remove_dir_all(&state);
    }

    async fn get(router: &Router, uri: &str, if_none_match: Option<&str>, gzip: bool) -> Response {
        let mut request = Request::get(uri);
        if let Some(tag) = if_none_match {
            request = request.header(header::IF_NONE_MATCH, tag);
        }
        if gzip {
            request = request.header(header::ACCEPT_ENCODING, "gzip");
        }
        router.clone().oneshot(request.body(Body::empty()).unwrap()).await.unwrap()
    }

    #[tokio::test]
    async fn route_serves_registered_hashes_only() {
        let dir = temp_dir("route");
        let body = "v 0 0 0\n".repeat(100);
        std::fs::write(dir.join("m.obj"), &body).unwrap();
        let index = AssetIndex::unrestricted();
        let asset = index.register(&dir.join("m.obj")).unwrap();
        let router: Router = router().with_state(index);
        let uri = api::asset(&asset.sha256);

        assert_eq!(get(&router, &api::asset("nothex"), None, false).await.status(), StatusCode::BAD_REQUEST);
        assert_eq!(get(&router, &api::asset(&asset.sha256.to_uppercase()), None, false).await.status(), StatusCode::BAD_REQUEST);
        assert_eq!(get(&router, &api::asset(&"0".repeat(64)), None, false).await.status(), StatusCode::NOT_FOUND);
        assert_eq!(get(&router, "/api/v1/assets/..%2F..%2Fetc%2Fpasswd", None, false).await.status(), StatusCode::BAD_REQUEST);

        let response = get(&router, &uri, None, false).await;
        assert_eq!(response.status(), StatusCode::OK);
        let h = response.headers();
        assert_eq!(h[header::CONTENT_TYPE], "model/obj");
        assert_eq!(h[header::CACHE_CONTROL], CACHE_CONTROL);
        assert_eq!(h[header::ETAG], format!("\"{}\"", asset.sha256).as_str());
        assert_eq!(h[header::X_CONTENT_TYPE_OPTIONS], "nosniff");
        let bytes = axum::body::to_bytes(response.into_body(), 1 << 20).await.unwrap();
        assert_eq!(bytes, body.as_bytes());

        let response = get(&router, &uri, None, true).await;
        assert_eq!(response.status(), StatusCode::OK);
        assert_eq!(response.headers()[header::CONTENT_ENCODING], "gzip");
        let gzip_tag = format!("\"{}-gzip\"", asset.sha256);
        assert_eq!(response.headers()[header::ETAG], gzip_tag.as_str());

        for tag in [format!("\"{}\"", asset.sha256), gzip_tag.clone(), format!("W/\"x\", {gzip_tag}"), "*".to_owned()] {
            let response = get(&router, &uri, Some(&tag), true).await;
            assert_eq!(response.status(), StatusCode::NOT_MODIFIED, "{tag}");
            assert_eq!(response.headers()[header::CACHE_CONTROL], CACHE_CONTROL);
            assert!(response.headers().contains_key(header::ETAG));
        }
        assert_eq!(get(&router, &uri, Some("\"other\""), false).await.status(), StatusCode::OK);
    }
}
