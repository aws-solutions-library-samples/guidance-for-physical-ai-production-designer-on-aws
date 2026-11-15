//! Content-addressed assets in S3: `users/<user>/assets/<sha256>`, put there by the backend only, from
//! the verified bytes of a registered asset ([`AssetIndex::read`]).
//!
//! The viewer reads them by presigned URL or through the edge path
//! ([`factoryos_protocol::storage::StorageInfo::edge_path`]); for the edge path they must be there
//! before the viewer asks, so a newly registered hash is uploaded ahead of time ([`Uploader::attach`]),
//! and a viewer that still misses one asks for it ([`factoryos_protocol::storage::api::STORAGE_ASSETS_ENSURE`]).
//! Content-addressed: once uploaded, an asset stays as it is; the uploaded set saves the `HEAD`s.

use std::collections::HashSet;
use std::sync::{Arc, Mutex, Weak};

use factoryos_protocol::storage::{Scope, StorageMode, asset_key};
use tokio::sync::Semaphore;

use super::{ObjectKey, Storage};
use crate::services::assets::AssetIndex;

/// Uploads at once (ahead of time and asked for together).
pub const MAX_CONCURRENT_UPLOADS: usize = 4;

/// Why an asset could not be ensured.
#[derive(Debug)]
pub enum AssetError {
    /// Not registered (or its file is gone): nothing to upload.
    NotRegistered,
    /// The store failed.
    Store(anyhow::Error),
}

impl std::fmt::Display for AssetError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotRegistered => f.write_str("not a registered asset"),
            Self::Store(err) => write!(f, "{err:#}"),
        }
    }
}

pub struct Uploader {
    storage: Arc<Storage>,
    assets: Arc<AssetIndex>,
    /// Known to be in the store already.
    uploaded: Mutex<HashSet<String>>,
    /// Being uploaded ahead of time (not spawned twice).
    pending: Mutex<HashSet<String>>,
    permits: Semaphore,
}

impl Uploader {
    pub fn new(storage: Arc<Storage>, assets: Arc<AssetIndex>) -> Arc<Self> {
        Arc::new(Self {
            storage,
            assets,
            uploaded: Mutex::default(),
            pending: Mutex::default(),
            permits: Semaphore::new(MAX_CONCURRENT_UPLOADS),
        })
    }

    /// Whether `sha256` is known to be in the store.
    pub fn is_uploaded(&self, sha256: &str) -> bool {
        self.uploaded.lock().unwrap().contains(sha256)
    }

    /// Put asset `sha256` (a lowercase hex SHA-256) into the store from its verified bytes, unless it
    /// is there. Only registered assets.
    pub async fn ensure(&self, sha256: &str) -> Result<(), AssetError> {
        if self.is_uploaded(sha256) {
            return Ok(());
        }
        if !self.assets.contains(sha256) {
            return Err(AssetError::NotRegistered);
        }
        let key = ObjectKey::new(Scope::User, &asset_key(sha256)).map_err(AssetError::Store)?;
        let _permit = self.permits.acquire().await.map_err(|err| AssetError::Store(err.into()))?;
        if self.is_uploaded(sha256) {
            return Ok(());
        }
        let store = self.storage.store();
        if store.head(&key).await.map_err(AssetError::Store)?.is_none() {
            let read = {
                let (assets, sha256) = (self.assets.clone(), sha256.to_owned());
                tokio::task::spawn_blocking(move || assets.read(&sha256)).await.map_err(|err| AssetError::Store(err.into()))?
            };
            let (_, bytes) = read.ok_or(AssetError::NotRegistered)?;
            store.put(&key, bytes).await.map_err(AssetError::Store)?;
        }
        self.uploaded.lock().unwrap().insert(sha256.to_owned());
        Ok(())
    }

    /// [`Self::ensure`] in the background (S3 mode; once per hash at a time). Errors are logged: a
    /// viewer that misses it asks again.
    pub fn spawn_ensure(self: &Arc<Self>, runtime: &tokio::runtime::Handle, sha256: &str) {
        if self.storage.mode() != StorageMode::S3 || self.is_uploaded(sha256) || !self.pending.lock().unwrap().insert(sha256.to_owned()) {
            return;
        }
        let (this, sha256) = (self.clone(), sha256.to_owned());
        runtime.spawn(async move {
            if let Err(err) = this.ensure(&sha256).await {
                tracing::debug!("storage: asset {sha256} not uploaded ahead: {err}");
            }
            this.pending.lock().unwrap().remove(&sha256);
        });
    }

    /// Upload every hash the index registers from now on, ahead of time (S3 mode, on the current
    /// runtime). Held weakly: the index doesn't keep the uploader alive.
    pub fn attach(self: &Arc<Self>) {
        if self.storage.mode() != StorageMode::S3 {
            return;
        }
        let Ok(runtime) = tokio::runtime::Handle::try_current() else {
            return tracing::warn!("storage: assets are not uploaded ahead (no runtime)");
        };
        let this: Weak<Self> = Arc::downgrade(self);
        self.assets.on_new_hash(move |sha256| {
            if let Some(this) = this.upgrade() {
                this.spawn_ensure(&runtime, sha256);
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::storage::sync::tests_support::{eventually, storage_at};

    fn temp(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("factoryos-storage-assets-{name}-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::canonicalize(dir).unwrap()
    }

    #[tokio::test]
    async fn registered_hashes_are_uploaded_ahead_once() {
        let root = temp("ahead");
        let (s3, storage) = storage_at(&root);
        let assets = AssetIndex::unrestricted();
        let uploader = Uploader::new(storage, assets.clone());
        uploader.attach();
        std::fs::write(root.join("part.obj"), "v 0 0 0\n").unwrap();
        let asset = assets.register(&root.join("part.obj")).unwrap();
        let key = asset_key(&asset.sha256);
        eventually("the asset in S3", || s3.bytes(Scope::User, &key).is_some()).await;
        assert_eq!(s3.bytes(Scope::User, &key).unwrap(), b"v 0 0 0\n");
        eventually("marked uploaded", || uploader.is_uploaded(&asset.sha256)).await;
        // Known: not uploaded again (even if gone from S3, e.g. by hand).
        s3.remove(Scope::User, &key);
        assets.register(&root.join("part.obj")).unwrap();
        uploader.ensure(&asset.sha256).await.unwrap();
        assert!(s3.bytes(Scope::User, &key).is_none());
        std::fs::remove_dir_all(&root).ok();
    }

    #[tokio::test]
    async fn only_registered_hashes_are_ensured() {
        let root = temp("ensure");
        let (s3, storage) = storage_at(&root);
        let assets = AssetIndex::unrestricted();
        let uploader = Uploader::new(storage, assets.clone());
        let unknown = "0".repeat(64);
        assert!(matches!(uploader.ensure(&unknown).await, Err(AssetError::NotRegistered)));
        assert!(s3.keys(Scope::User).is_empty());
        std::fs::write(root.join("a.stl"), "solid a").unwrap();
        let asset = assets.register(&root.join("a.stl")).unwrap();
        uploader.ensure(&asset.sha256).await.unwrap();
        assert_eq!(s3.keys(Scope::User), [asset_key(&asset.sha256)]);
        // The store down: an error, and not marked.
        std::fs::write(root.join("b.stl"), "solid b").unwrap();
        let b = assets.register(&root.join("b.stl")).unwrap();
        s3.down.store(true, std::sync::atomic::Ordering::SeqCst);
        assert!(matches!(uploader.ensure(&b.sha256).await, Err(AssetError::Store(_))));
        assert!(!uploader.is_uploaded(&b.sha256));
        std::fs::remove_dir_all(&root).ok();
    }
}
