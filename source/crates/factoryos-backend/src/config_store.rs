//! The backend owns the config: loaded from and saved to one JSON file, served over HTTP,
//! and republished on Zenoh whenever it changes.
//!
//! **S3 storage** ([`ConfigStore::attach_storage`]): the user's settings ([`USER_FIELDS`] and the
//! active cell by name, see [`Config`]) have their store of record at `users/<owner>/`
//! [`USER_CONFIG_KEY`]; the local file is the cache the tools read, and the only home of the
//! machine's settings ([`factoryos_protocol::config::MACHINE_FIELDS`]).
//! * Load: the store's settings win over the file's; with none in the store yet, the file's go up
//!   once. [`ConfigStore::refresh`] (every [`REFRESH_EVERY`], and on each `GET /api/v1/config`) is a
//!   conditional GET on the last ETag: another backend's change of the same user's settings shows up.
//! * Save ([`ConfigStore::try_set`]): the user's settings go to the store first, written only if it is
//!   still at the last ETag. Someone else wrote meanwhile: read again, and apply the fields this save
//!   changed on top, unless the other write changed one of them too ([`SaveError::Conflict`]). The
//!   store unreachable: nothing is saved ([`SaveError::Unavailable`]). Then the local file.
//!   Machine settings and [`Secrets`] never go to the store.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Duration;

use anyhow::Context as _;
use factoryos_protocol::config::{ACTIVE_CELL_FIELD, Config, ConfigResponse, Secrets, USER_CONFIG_KEY, USER_FIELDS};
use factoryos_protocol::storage::{Scope, StorageMode};
use serde_json::{Map, Value};

use crate::paths;
use crate::storage::{Fetched, ObjectKey, PutOutcome, Storage};

/// How often the user's settings are looked for in the store (S3 mode).
pub const REFRESH_EVERY: Duration = Duration::from_secs(20);

/// The user's settings as kept in the store: field name → value.
type UserDoc = Map<String, Value>;

/// Why a save did not happen (nothing was saved).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SaveError {
    /// Another backend of the same user changed a setting this save changes, meanwhile (`409`).
    Conflict(String),
    /// The store did not take it (`503`).
    Unavailable(String),
}

impl std::fmt::Display for SaveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Conflict(why) => write!(f, "not saved: {why}"),
            Self::Unavailable(why) => write!(f, "not saved: the settings' store (S3) did not take it: {why}"),
        }
    }
}

impl std::error::Error for SaveError {}

/// The user's settings in the store.
struct Remote {
    storage: Arc<Storage>,
    /// The runtime the store's requests run on (a save is sync: [`block_on`]).
    handle: tokio::runtime::Handle,
    state: Mutex<RemoteState>,
}

#[derive(Default)]
struct RemoteState {
    /// Read (or written) at least once.
    loaded: bool,
    etag: Option<String>,
    /// What the store has, as last read or written.
    doc: UserDoc,
}

/// Passwords, kept in memory only and never returned to clients.
#[derive(Default)]
struct StoredSecrets {
    edh_password: String,
    cognito_password: String,
    os_password: String,
}

pub struct ConfigStore {
    file: PathBuf,
    inner: Mutex<Inner>,
    /// S3 mode: the user's settings' store of record.
    remote: OnceLock<Remote>,
    /// One save at a time.
    writes: Mutex<()>,
}

struct Inner {
    config: Config,
    file_error: Option<String>,
    secrets: StoredSecrets,
}

impl ConfigStore {
    /// Load `file`, filling empty settings with defaults and applying `FACTORY_OS_*` overrides.
    /// A missing file is created.
    pub fn open(file: PathBuf) -> Self {
        let (mut config, file_error) = load_file(&file);
        fill_defaults(&mut config);
        let store = Self {
            file,
            inner: Mutex::new(Inner {
                config,
                file_error,
                secrets: StoredSecrets::default(),
            }),
            remote: OnceLock::new(),
            writes: Mutex::new(()),
        };
        if !store.file.exists() {
            store.save();
        }
        apply_env_overrides(&mut store.inner.lock().unwrap().config);
        store
    }

    pub fn get(&self) -> Config {
        self.inner.lock().unwrap().config.clone()
    }

    pub fn response(&self) -> ConfigResponse {
        let inner = self.inner.lock().unwrap();
        ConfigResponse {
            config: inner.config.clone(),
            file: self.file.display().to_string(),
            file_error: inner.file_error.clone(),
        }
    }

    /// Replace the config and save it. Returns `true` if it changed. A save that failed
    /// ([`Self::try_set`]) changes nothing, is logged and shows as the response's `fileError`.
    pub fn set(&self, config: Config) -> bool {
        match self.try_set(config) {
            Ok(changed) => changed,
            Err(err) => {
                tracing::warn!("Config: {err}");
                self.inner.lock().unwrap().file_error = Some(err.to_string());
                false
            }
        }
    }

    /// Replace the config and save it: S3 mode, the user's settings to the store first (see the
    /// module docs). Returns `true` if it changed.
    pub fn try_set(&self, config: Config) -> Result<bool, SaveError> {
        let _writing = self.writes.lock().unwrap_or_else(|e| e.into_inner());
        let old = self.get();
        if old == config {
            return Ok(false);
        }
        let Some(remote) = self.remote.get() else {
            self.inner.lock().unwrap().config = config;
            self.save();
            return Ok(true);
        };
        let unavailable = |err: anyhow::Error| SaveError::Unavailable(format!("{err:#}"));
        if !remote.state.lock().unwrap().loaded {
            block_on(&remote.handle, self.refresh_remote(remote)).map_err(unavailable)?;
        }
        // The fields this save changes (`None`: no longer there, e.g. no active cell).
        let (base, ours) = (self.user_doc(&old), self.user_doc(&config));
        let changes: Vec<(String, Option<Value>)> = base
            .keys()
            .chain(ours.keys())
            .filter(|k| base.get(*k) != ours.get(*k))
            .map(|k| (k.clone(), ours.get(k).cloned()))
            .collect::<std::collections::BTreeMap<_, _>>()
            .into_iter()
            .collect();
        let key = config_key();
        for attempt in 0.. {
            let (doc, etag) = {
                let state = remote.state.lock().unwrap();
                (state.doc.clone(), state.etag.clone())
            };
            let mut target = doc.clone();
            for (k, v) in &changes {
                match v {
                    Some(v) => target.insert(k.clone(), v.clone()),
                    None => target.remove(k),
                };
            }
            if target == doc {
                break;
            }
            let bytes = serde_json::to_vec_pretty(&Value::Object(target.clone())).map_err(|err| unavailable(err.into()))?;
            let store = remote.storage.store();
            match block_on(&remote.handle, store.put_if(&key, bytes, etag.as_deref())).map_err(unavailable)? {
                PutOutcome::Written(tag) => {
                    let tag = match tag {
                        Some(tag) => Some(tag),
                        None => block_on(&remote.handle, store.head(&key)).ok().flatten().and_then(|info| info.etag),
                    };
                    let mut state = remote.state.lock().unwrap();
                    (state.doc, state.etag) = (target, tag);
                    break;
                }
                PutOutcome::Conflict if attempt < 3 => {
                    // Another backend of the user wrote meanwhile: on top of theirs, unless they changed
                    // what this save changes.
                    block_on(&remote.handle, self.refresh_remote(remote)).map_err(unavailable)?;
                    let now = remote.state.lock().unwrap().doc.clone();
                    if let Some((k, _)) = changes.iter().find(|(k, v)| now.get(k) != base.get(k) && now.get(k) != v.as_ref()) {
                        return Err(SaveError::Conflict(format!("{k} was changed by another of your backends meanwhile; reload and try again")));
                    }
                }
                PutOutcome::Conflict => return Err(SaveError::Conflict("the settings kept changing in the store".to_owned())),
            }
        }
        // Locally: this save, with what the store has for the settings it didn't change.
        let mut merged = config;
        let mut others = remote.state.lock().unwrap().doc.clone();
        for (k, _) in &changes {
            others.remove(k);
        }
        apply_user_fields(&mut merged, &others);
        self.inner.lock().unwrap().config = merged;
        self.save();
        Ok(true)
    }

    /// Re-read the file, e.g. after it was edited by hand (S3 mode: the store's user settings on top).
    pub fn reload(&self) {
        let (mut config, file_error) = load_file(&self.file);
        fill_defaults(&mut config);
        if let Some(remote) = self.remote.get() {
            let state = remote.state.lock().unwrap();
            if state.loaded {
                apply_user_fields(&mut config, &state.doc);
            }
        }
        let mut inner = self.inner.lock().unwrap();
        inner.config = config;
        inner.file_error = file_error;
    }

    /// S3 mode: the user's settings come from `storage` from now on (see the module docs): read now
    /// (the store's win; none there yet: the file's go up). Returns whether the config changed. An
    /// error leaves it attached: saves fail until [`Self::refresh`] reads it.
    pub async fn attach_storage(&self, storage: Arc<Storage>) -> anyhow::Result<bool> {
        if storage.mode() != StorageMode::S3 {
            return Ok(false);
        }
        let remote = Remote { storage, handle: tokio::runtime::Handle::current(), state: Mutex::default() };
        if self.remote.set(remote).is_err() {
            anyhow::bail!("the config already has a store");
        }
        self.refresh().await
    }

    /// S3 mode: the store's user settings if they changed (one conditional GET), and the active
    /// cell they name. Returns whether the config changed.
    pub async fn refresh(&self) -> anyhow::Result<bool> {
        let Some(remote) = self.remote.get() else { return Ok(false) };
        let (changed, follow) = self.refresh_remote(remote).await?;
        if let Some(cell) = follow {
            // Another backend of the user made it the active cell: here too, from this cells folder.
            match crate::services::cells::set_active(self, &cell) {
                Ok(_) => return Ok(true),
                Err(err) => tracing::warn!("Config: the active cell {cell} (from your settings): {err:#}"),
            }
        }
        Ok(changed)
    }

    /// Read the store's user settings (conditionally once read) into the config; create them
    /// from the config when there are none. Returns whether the config changed, and the active
    /// cell to follow (another one than here, present here).
    async fn refresh_remote(&self, remote: &Remote) -> anyhow::Result<(bool, Option<String>)> {
        let key = config_key();
        let store = remote.storage.store();
        for _ in 0..3 {
            let (loaded, etag) = {
                let state = remote.state.lock().unwrap();
                (state.loaded, state.etag.clone())
            };
            match store.get_if(&key, if loaded { etag.as_deref() } else { None }).await? {
                Fetched::NotModified => return Ok((false, None)),
                Fetched::Found { bytes, etag } => {
                    let doc: UserDoc = serde_json::from_slice(&bytes).with_context(|| format!("the settings in the store ({USER_CONFIG_KEY})"))?;
                    let (changed, follow) = {
                        let mut inner = self.inner.lock().unwrap();
                        let before = inner.config.clone();
                        apply_user_fields(&mut inner.config, &doc);
                        let wanted = doc.get(ACTIVE_CELL_FIELD).and_then(Value::as_str).map(str::to_owned);
                        let follow = wanted.filter(|cell| {
                            self.active_cell_name(&inner.config).as_deref() != Some(cell.as_str()) && self.has_cell(&inner.config, cell)
                        });
                        (inner.config != before, follow)
                    };
                    {
                        let mut state = remote.state.lock().unwrap();
                        (state.loaded, state.etag, state.doc) = (true, etag, doc);
                    }
                    if changed {
                        self.save();
                    }
                    return Ok((changed, follow));
                }
                Fetched::Missing => {
                    // None in the store yet: this machine's go up (once; again if deleted out of band).
                    let doc = self.user_doc(&self.get());
                    match store.put_if(&key, serde_json::to_vec_pretty(&Value::Object(doc.clone()))?, None).await? {
                        PutOutcome::Written(tag) => {
                            let tag = match tag {
                                Some(tag) => Some(tag),
                                None => store.head(&key).await?.and_then(|info| info.etag),
                            };
                            tracing::info!("Config: your settings are in the store now ({USER_CONFIG_KEY})");
                            let mut state = remote.state.lock().unwrap();
                            (state.loaded, state.etag, state.doc) = (true, tag, doc);
                            return Ok((false, None));
                        }
                        // Another backend made them meanwhile: read those.
                        PutOutcome::Conflict => {}
                    }
                }
            }
        }
        anyhow::bail!("the settings in the store kept changing")
    }

    /// The user's settings of `config`, as the store keeps them: [`USER_FIELDS`], and the active
    /// cell by name when it is a cell of the cells folder.
    fn user_doc(&self, config: &Config) -> UserDoc {
        let value = serde_json::to_value(config).unwrap_or_default();
        let mut doc: UserDoc =
            value.as_object().into_iter().flatten().filter(|(k, _)| USER_FIELDS.contains(&k.as_str())).map(|(k, v)| (k.clone(), v.clone())).collect();
        if let Some(cell) = self.active_cell_name(config) {
            doc.insert(ACTIVE_CELL_FIELD.to_owned(), Value::String(cell));
        }
        doc
    }

    /// The active cell's folder name, if it is a cell of the (configured) cells folder.
    fn active_cell_name(&self, config: &Config) -> Option<String> {
        let cells = self.resolve(&config.cells_dir);
        let mut dir = PathBuf::from(self.resolve(&config.cell_dir));
        if cells.is_empty() || config.cell_dir.trim().is_empty() {
            return None;
        }
        if dir.file_name().is_some_and(|n| n == "cellrunner" || n == crate::services::cell_resolve::VIEW_DIR) {
            dir.pop();
        }
        let canonical = |p: &Path| std::fs::canonicalize(p).unwrap_or_else(|_| p.to_path_buf());
        (canonical(dir.parent()?) == canonical(Path::new(&cells))).then(|| dir.file_name()?.to_str().map(str::to_owned)).flatten()
    }

    /// Whether this machine's cells folder has cell `cell`.
    fn has_cell(&self, config: &Config, cell: &str) -> bool {
        let cells = self.resolve(&config.cells_dir);
        !cells.is_empty() && !cell.is_empty() && !cell.contains(['/', '\\']) && !cell.starts_with('.') && Path::new(&cells).join(cell).is_dir()
    }

    pub fn set_secrets(&self, secrets: Secrets) {
        let stored = &mut self.inner.lock().unwrap().secrets;
        for (value, update) in [
            (&mut stored.edh_password, secrets.edh_password),
            (&mut stored.cognito_password, secrets.cognito_password),
            (&mut stored.os_password, secrets.os_password),
        ] {
            if let Some(update) = update {
                *value = update;
            }
        }
    }

    /// `value` as an absolute path: relative paths resolve against the config file's folder.
    pub fn resolve(&self, value: &str) -> String {
        resolve(&self.file, value)
    }

    fn save(&self) {
        let json = serde_json::to_string_pretty(&self.inner.lock().unwrap().config).unwrap_or_default();
        let result = std::fs::write(&self.file, json + "\n");
        self.inner.lock().unwrap().file_error = result.err().map(|err| format!("Failed to save config: {err}"));
    }
}

/// The user's settings' key in their subtree.
fn config_key() -> ObjectKey {
    ObjectKey::new(Scope::User, USER_CONFIG_KEY).expect("a valid key")
}

/// `doc`'s user settings into `config` ([`USER_FIELDS`] only, each that fits; machine fields stay).
fn apply_user_fields(config: &mut Config, doc: &UserDoc) {
    for field in USER_FIELDS {
        let Some(value) = doc.get(*field) else { continue };
        let Ok(Value::Object(mut current)) = serde_json::to_value(&*config) else { return };
        if current.get(*field) == Some(value) {
            continue;
        }
        current.insert((*field).to_owned(), value.clone());
        match serde_json::from_value::<Config>(Value::Object(current)) {
            Ok(merged) => *config = merged,
            Err(err) => tracing::warn!("Config: {field} in the store: {err}"),
        }
    }
    // The store has every user's whole settings, the default scene at its old default too: it follows the current.
    config.upgrade_default_scene();
}

/// `future` on `handle`'s runtime, from sync code (any thread, a runtime's own included): on a
/// thread of its own, which waits.
fn block_on<F>(handle: &tokio::runtime::Handle, future: F) -> F::Output
where
    F: std::future::Future + Send,
    F::Output: Send,
{
    std::thread::scope(|scope| match scope.spawn(|| handle.block_on(future)).join() {
        Ok(output) => output,
        Err(panic) => std::panic::resume_unwind(panic),
    })
}

fn load_file(path: &Path) -> (Config, Option<String>) {
    match std::fs::read_to_string(path) {
        Ok(text) => match serde_json::from_str(&text) {
            Ok(config) => (config, None),
            Err(err) => (Config::default(), Some(format!("Failed to parse config: {err}"))),
        },
        Err(_) => (Config::default(), None),
    }
}

/// Point empty cell settings at the first sample cell present and empty asset dirs at the catalogue.
fn fill_defaults(config: &mut Config) {
    // A plan prompt left at an earlier default follows the current one (the planner router); the default scene too.
    config.upgrade_default_prompts();
    config.upgrade_default_scene();
    if let Some(cell) = paths::sample_cells().into_iter().next() {
        for (value, default) in [
            (&mut config.cell_dir, cell.cell_dir),
            (&mut config.env_dir, cell.env_dir),
            (&mut config.scene, cell.scene),
        ] {
            if value.trim().is_empty() {
                *value = default;
            }
        }
    }
    if config.agentic_repo_dir.trim().is_empty()
        && let Some(repo) = paths::default_agentic_repo()
    {
        config.agentic_repo_dir = repo;
    }
    for value in [&mut config.catalogue, &mut config.asset_root] {
        if value.trim().is_empty() {
            *value = paths::default_assets();
        }
    }
}

fn apply_env_overrides(config: &mut Config) {
    if let Some(catalogue) = std::env::var_os("FACTORY_OS_CATALOGUE") {
        config.catalogue = catalogue.to_string_lossy().into_owned();
    }
    if let Some(asset_root) = std::env::var_os("FACTORY_OS_ASSET_ROOT") {
        config.asset_root = asset_root.to_string_lossy().into_owned();
    }
    // `FACTORY_OS_CELL=<name>`: use one of the sample cells (e.g. `welding_cell_2_small`).
    if let Some(name) = std::env::var_os("FACTORY_OS_CELL")
        && let Some(cell) = paths::sample_cells().into_iter().find(|c| *c.name == *name)
    {
        config.cell_dir = cell.cell_dir;
        config.env_dir = cell.env_dir;
        config.scene = cell.scene;
    }
}

fn resolve(file: &Path, value: &str) -> String {
    let value = value.trim();
    let path = Path::new(value);
    if path.is_absolute() || value.is_empty() {
        return value.to_owned();
    }
    file.parent()
        .unwrap_or(Path::new("."))
        .join(path)
        .to_string_lossy()
        .into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_file(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("factoryos-config-{name}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        dir.join("factoryos.config.json")
    }

    #[test]
    fn creates_missing_file_with_defaults() {
        let file = temp_file("missing");
        std::fs::remove_file(&file).ok();
        let store = ConfigStore::open(file.clone());
        assert!(file.exists());
        assert!(!store.get().catalogue.is_empty());
        // The scene default comes from the first sample cell, and sample cells are only offered
        // when their data is present (large ones are fetched separately).
        if !paths::sample_cells().is_empty() {
            assert!(!store.get().scene.is_empty());
        }
    }

    #[test]
    fn keeps_values_and_reports_parse_errors() {
        let file = temp_file("values");
        std::fs::write(&file, r#"{"scene":"mine.yaml","cellDir":"cells/A"}"#).unwrap();
        let store = ConfigStore::open(file.clone());
        assert_eq!(store.get().scene, "mine.yaml");
        assert_eq!(store.resolve("cells/A"), file.parent().unwrap().join("cells/A").to_string_lossy());

        std::fs::write(&file, "{ not json").unwrap();
        store.reload();
        assert!(store.response().file_error.unwrap().contains("parse"));
    }

    #[test]
    fn set_saves_only_on_change() {
        let file = temp_file("set");
        let store = ConfigStore::open(file.clone());
        let mut config = store.get();
        assert!(!store.set(config.clone()));
        config.scene = "changed.yaml".to_owned();
        assert!(store.set(config));
        assert!(std::fs::read_to_string(&file).unwrap().contains("changed.yaml"));
    }

    #[test]
    fn secrets_are_never_serialized() {
        let file = temp_file("secrets");
        let store = ConfigStore::open(file.clone());
        store.set_secrets(Secrets {
            os_password: Some("hunter2".to_owned()),
            ..Default::default()
        });
        assert!(!serde_json::to_string(&store.response()).unwrap().contains("hunter2"));
        assert!(!std::fs::read_to_string(&file).unwrap().contains("hunter2"));
    }

    // --- S3 mode ---

    use crate::storage::sync::tests_support::{FakeS3, storage_at};
    use factoryos_protocol::config::MACHINE_FIELDS;
    use std::sync::atomic::Ordering;

    const KEY: &str = USER_CONFIG_KEY;

    /// A config file in a fresh folder with `cells/` (cells `weld`, `other`) and the file's `json`,
    /// and a fake S3 with the store's `doc` (if any) for the user's settings.
    fn s3_world(name: &str, json: serde_json::Value, doc: Option<serde_json::Value>) -> (PathBuf, ConfigStore, Arc<FakeS3>, Arc<Storage>) {
        let dir = temp_file(&format!("s3-{name}")).parent().unwrap().to_owned();
        std::fs::remove_dir_all(&dir).ok();
        for cell in ["weld", "other"] {
            std::fs::create_dir_all(dir.join("cells").join(cell)).unwrap();
            std::fs::write(dir.join("cells").join(cell).join("cell.yaml"), "name: x\n").unwrap();
        }
        let dir = std::fs::canonicalize(&dir).unwrap();
        let file = dir.join("factoryos.config.json");
        let mut json = json;
        json["cellsDir"] = dir.join("cells").display().to_string().into();
        json["cellDir"] = dir.join("cells/weld").display().to_string().into();
        std::fs::write(&file, json.to_string()).unwrap();
        let (s3, storage) = storage_at(&dir);
        if let Some(doc) = doc {
            s3.set(Scope::User, KEY, doc.to_string().as_bytes());
        }
        (file, ConfigStore::open(dir.join("factoryos.config.json")), s3, storage)
    }

    fn stored(s3: &FakeS3) -> serde_json::Map<String, serde_json::Value> {
        serde_json::from_slice(&s3.bytes(Scope::User, KEY).expect("the user's settings in the store")).unwrap()
    }

    #[test]
    fn every_field_is_a_user_or_a_machine_setting() {
        let value = serde_json::to_value(Config::default()).unwrap();
        let fields: Vec<&String> = value.as_object().unwrap().keys().collect();
        for field in &fields {
            let (user, machine) = (USER_FIELDS.contains(&field.as_str()), MACHINE_FIELDS.contains(&field.as_str()));
            assert!(user != machine, "{field}: user {user}, machine {machine}");
        }
        assert_eq!(USER_FIELDS.len() + MACHINE_FIELDS.len(), fields.len(), "no names that are not fields");
    }

    #[tokio::test]
    async fn the_stores_settings_win_at_startup_machine_fields_stay_local() {
        let local = serde_json::json!({"agenticModel": "local-model", "catalogue": "/here/catalogue", "dresserPort": 3030});
        let doc = serde_json::json!({"agenticModel": "s3-model", "defaultScene": "jam2.yaml", "catalogue": "/elsewhere", "dresserPort": 1});
        let (file, store, _s3, storage) = s3_world("wins", local, Some(doc));
        assert!(store.attach_storage(storage).await.unwrap());
        let config = store.get();
        assert_eq!((config.agentic_model.as_str(), config.default_scene.as_str()), ("s3-model", "jam2.yaml"));
        assert_eq!((config.catalogue.as_str(), config.dresser_port), ("/here/catalogue", 3030), "machine fields are this machine's");
        let cached: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&file).unwrap()).unwrap();
        assert_eq!(cached["agenticModel"], "s3-model", "the file is the cache");
        assert_eq!(cached["catalogue"], "/here/catalogue");
    }

    #[tokio::test]
    async fn the_files_settings_go_up_once_without_machine_fields_or_secrets() {
        let (_file, store, s3, storage) = s3_world("first", serde_json::json!({"agenticModel": "mine", "catalogue": "/x"}), None);
        store.set_secrets(Secrets { os_password: Some("hunter2".into()), ..Default::default() });
        assert!(!store.attach_storage(storage).await.unwrap());
        let doc = stored(&s3);
        assert_eq!(doc["agenticModel"], "mine");
        assert_eq!(doc[ACTIVE_CELL_FIELD], "weld", "the active cell, by name");
        assert!(doc.keys().all(|k| USER_FIELDS.contains(&k.as_str()) || k == ACTIVE_CELL_FIELD), "{doc:?}");
        assert!(!String::from_utf8(s3.bytes(Scope::User, KEY).unwrap()).unwrap().contains("hunter2"));
        // Not again: the next start reads it (unchanged).
        let before = s3.bytes(Scope::User, KEY);
        assert!(!store.refresh().await.unwrap());
        assert_eq!(s3.bytes(Scope::User, KEY), before);
    }

    #[tokio::test]
    async fn machine_settings_are_saved_locally_only() {
        let (file, store, s3, storage) = s3_world("machine", serde_json::json!({}), None);
        store.attach_storage(storage).await.unwrap();
        let before = s3.bytes(Scope::User, KEY);
        let mut config = store.get();
        config.catalogue = "/opt/mine".into();
        config.cell_runner_ws_port = 9999;
        config.fitter_binary_path = "/usr/local/bin/fitter".into();
        assert!(store.try_set(config).unwrap());
        assert_eq!(s3.bytes(Scope::User, KEY), before, "nothing of it in the store");
        let cached = std::fs::read_to_string(&file).unwrap();
        assert!(cached.contains("/opt/mine") && cached.contains("9999"));
        // A user setting does go up, and only the user settings.
        let mut config = store.get();
        config.agentic_plan_prompt = "plan {cell}".into();
        assert!(store.try_set(config).unwrap());
        let doc = stored(&s3);
        assert_eq!(doc["agenticPlanPrompt"], "plan {cell}");
        assert!(MACHINE_FIELDS.iter().all(|k| !doc.contains_key(*k)), "{doc:?}");
    }

    #[tokio::test]
    async fn a_field_level_save_survives_a_concurrent_write() {
        let (_file, store, s3, storage) = s3_world("race", serde_json::json!({}), None);
        store.attach_storage(storage).await.unwrap();
        // Another backend of the user changes another setting (a new ETag): this one's write gets a 412.
        let mut theirs = stored(&s3);
        theirs.insert("defaultScene".into(), "theirs.yaml".into());
        s3.set(Scope::User, KEY, serde_json::to_vec(&theirs).unwrap().as_slice());
        let mut config = store.get();
        config.agentic_model = "ours".into();
        assert!(store.try_set(config).unwrap());
        let doc = stored(&s3);
        assert_eq!((doc["agenticModel"].as_str(), doc["defaultScene"].as_str()), (Some("ours"), Some("theirs.yaml")));
        let config = store.get();
        assert_eq!((config.agentic_model.as_str(), config.default_scene.as_str()), ("ours", "theirs.yaml"));
        // They change the very setting this save changes: a conflict, nothing saved.
        let mut theirs = stored(&s3);
        theirs.insert("agenticModel".into(), "theirs".into());
        s3.set(Scope::User, KEY, serde_json::to_vec(&theirs).unwrap().as_slice());
        let mut config = store.get();
        config.agentic_model = "ours again".into();
        assert!(matches!(store.try_set(config), Err(SaveError::Conflict(_))));
        assert_eq!(stored(&s3)["agenticModel"], "theirs");
        assert_eq!(store.get().agentic_model, "theirs", "the store's, read on the conflict");
    }

    #[tokio::test]
    async fn an_out_of_band_change_is_picked_up_by_its_etag() {
        let (_file, store, s3, storage) = s3_world("etag", serde_json::json!({}), None);
        store.attach_storage(storage).await.unwrap();
        let gets = s3.gets.load(Ordering::SeqCst);
        assert!(!store.refresh().await.unwrap(), "unchanged: 304");
        assert_eq!(s3.gets.load(Ordering::SeqCst) - gets, 1, "one conditional GET");
        let mut doc = stored(&s3);
        doc.insert("meshLod".into(), "collision".into());
        doc.insert(ACTIVE_CELL_FIELD.into(), "other".into());
        s3.set(Scope::User, KEY, serde_json::to_vec(&doc).unwrap().as_slice());
        assert!(store.refresh().await.unwrap());
        let config = store.get();
        assert_eq!(config.mesh_lod, "collision");
        assert!(config.cell_dir.ends_with("cells/other"), "the active cell follows, from this cells folder: {}", config.cell_dir);
        assert!(!store.refresh().await.unwrap());
    }

    #[tokio::test]
    async fn a_save_fails_when_the_store_is_down() {
        let (file, store, s3, storage) = s3_world("down", serde_json::json!({"agenticModel": "before"}), None);
        store.attach_storage(storage).await.unwrap();
        let cached = std::fs::read_to_string(&file).unwrap();
        s3.down.store(true, Ordering::SeqCst);
        let mut config = store.get();
        config.agentic_model = "after".into();
        assert!(matches!(store.try_set(config.clone()), Err(SaveError::Unavailable(_))));
        assert_eq!(store.get().agentic_model, "before");
        assert_eq!(std::fs::read_to_string(&file).unwrap(), cached, "not kept locally either");
        assert!(!store.set(config));
        assert!(store.response().file_error.unwrap().contains("not saved"));
        // Down at startup too: attached, and saves fail until it is read.
        let (_file, store, s3, storage) = s3_world("down-start", serde_json::json!({}), None);
        s3.down.store(true, Ordering::SeqCst);
        assert!(store.attach_storage(storage).await.is_err());
        let mut config = store.get();
        config.agentic_model = "x".into();
        assert!(matches!(store.try_set(config), Err(SaveError::Unavailable(_))));
    }
}
