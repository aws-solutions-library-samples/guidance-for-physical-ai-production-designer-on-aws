//! The storage routes ([`factoryos_protocol::storage::api`]).
//!
//! Every route is admitted as the workspace files are ([`crate::services::files::admit_transfer`]):
//! the workspace's owner as the edge's login names them (`X-User-Alias` = `--workspace`), or,
//! without the edge, a loopback caller asking for a loopback host; and no other site's page. Merged
//! outside the CORS layer (main.rs). The user subtree is always the backend's own user: no
//! request names a user, and keys are validated ([`ObjectKey`]) before they reach a store.
//!
//! **Writes** are checked on every request by [`may_write`] with the caller's [`Roles`] from the
//! [`Authz`] seam: the user's own subtree, the shared catalogue for catalogue admins, never the
//! backend's own prefixes (`assets/`) in either, and never the cell templates the cell-templates CI
//! publishes (`cell-templates/` of the catalogue, `catalogue/cell-templates/` of the user's; for
//! catalogue admins too, and no move or remove of a folder that holds them).
//!
//! * Presigning (S3 mode only): `GET`s of the user's subtree, of the shared catalogue and of
//!   registered assets; `PUT`s of what the caller may write. Each URL lives [`PRESIGN_TTL`] at most.
//! * Objects over HTTP: the local mode's data path (and a fallback in S3 mode, through the backend).
//! * The edge path ([`StorageInfo::edge_path`], `--edge-data`): named in the info for the workspace's
//!   owner in S3 mode only ([`edge_path`]); its assets are uploaded ahead ([`super::assets`]) or on a
//!   viewer's ask ([`api::STORAGE_ASSETS_ENSURE`]).
//! * Organising: new folder, move/rename, remove, and publishing into the shared catalogue.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::Arc;
use std::time::UNIX_EPOCH;

use axum::body::Bytes;
use axum::extract::{ConnectInfo, DefaultBodyLimit, Path as UrlPath, Query, State};
use axum::http::{HeaderMap, HeaderValue, StatusCode, header};
use axum::response::{IntoResponse as _, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use factoryos_protocol::files::MAX_TRANSFER_BYTES;
use factoryos_protocol::storage::{
    Commit, EnsureAssets, EnsuredAsset, EnsuredAssets, Folder, MAX_ENSURE_ITEMS, MAX_PRESIGN_ITEMS, Method, Move, ObjectList, PresignItem, PresignRequest, PresignResponse, Presigned, Publish,
    Published, Remove, Roles, Scope, StorageInfo, StorageMode, api, asset_key, contains_platform_only, is_platform_only, may_write, platform_only_folder,
    split_folder, valid_prefix,
};

use super::assets::{AssetError, Uploader};
use super::sync::CatalogueSync;
use super::{Changes, Conflict, ObjectKey, PRESIGN_TTL, PlatformOnly, Storage};
use crate::authz::{Authz, Caller};
use crate::services::assets::AssetIndex;
use crate::services::files::{FilesService, admit_transfer};

type HttpError = (StatusCode, String);

pub struct StorageRoutes {
    storage: Arc<Storage>,
    files: Arc<FilesService>,
    assets: Arc<AssetIndex>,
    authz: Arc<dyn Authz>,
    /// Told of every change of the shared catalogue (the other workspaces follow, viewers list again).
    sync: Arc<CatalogueSync>,
    /// Puts registered assets into S3 (`users/<user>/assets/<sha256>`), ahead of time and when asked.
    uploader: Arc<Uploader>,
    /// `--edge-data`: the workspace edge serves the bucket at [`EDGE_PATH`].
    edge_data: bool,
}

/// Where the workspace edge serves the bucket, at the viewer's origin.
pub const EDGE_PATH: &str = "/_data";

/// The edge path for a caller with the edge's `alias` (`None`: no edge, e.g. loopback): only in S3
/// mode with `--edge-data`, and only for the workspace's owner, since the edge maps every path to
/// the signed-in user's own subtree (another user on a shared workspace keeps presigned URLs).
pub fn edge_path(mode: StorageMode, edge_data: bool, alias: Option<&str>, owner: &str) -> Option<String> {
    (mode == StorageMode::S3 && edge_data && alias == Some(owner)).then(|| EDGE_PATH.to_owned())
}

/// `edge_data`: `--edge-data` ([`edge_path`]). In S3 mode, every asset `assets` registers from now
/// on is uploaded ahead ([`Uploader::attach`]).
pub fn router(
    storage: Arc<Storage>,
    files: Arc<FilesService>,
    assets: Arc<AssetIndex>,
    authz: Arc<dyn Authz>,
    sync: Arc<CatalogueSync>,
    edge_data: bool,
) -> Router {
    let uploader = Uploader::new(storage.clone(), assets.clone());
    uploader.attach();
    let state = Arc::new(StorageRoutes { storage, files, assets, authz, sync, uploader, edge_data });
    Router::new()
        .route(api::STORAGE_INFO, get(info))
        .route(api::STORAGE_PRESIGN, post(presign))
        .route(api::STORAGE_LIST, get(list))
        .route(api::STORAGE_COMMIT, post(commit))
        .route(api::STORAGE_FOLDER, post(make_folder))
        .route(api::STORAGE_MOVE, post(move_keys))
        .route(api::STORAGE_REMOVE, post(remove))
        .route(api::STORAGE_PUBLISH, post(publish))
        .route(api::STORAGE_ASSETS_ENSURE, post(ensure_assets))
        .route(
            api::STORAGE_OBJECT,
            get(get_object).put(put_object).delete(delete_object).layer(DefaultBodyLimit::max(MAX_TRANSFER_BYTES as usize)),
        )
        .with_state(state)
}

type St = State<Arc<StorageRoutes>>;

impl StorageRoutes {
    /// Admit the request, then the caller's roles.
    fn admit(&self, peer: SocketAddr, headers: &HeaderMap) -> Result<Roles, HttpError> {
        admit_transfer(&self.files, peer, headers)?;
        Ok(self.authz.roles(&Caller::of(peer, headers)))
    }

    /// After a write of `scope`: a change of the shared catalogue is recorded and announced.
    async fn changed(&self, scope: Scope, changes: Changes) {
        if scope == Scope::Catalogue {
            self.sync.record_local(changes).await;
        }
    }
}

fn scope_of(scope: &str) -> Result<Scope, HttpError> {
    Scope::parse(scope).ok_or_else(|| (StatusCode::NOT_FOUND, format!("no storage scope {scope}")))
}

fn key_of(scope: Scope, key: &str) -> Result<ObjectKey, HttpError> {
    ObjectKey::new(scope, key).map_err(|err| (StatusCode::BAD_REQUEST, err.to_string()))
}

/// `key` (a file, or a folder ending in `/`) of `scope`, if `roles` may write it.
fn check_write(roles: Roles, scope: Scope, key: &str) -> Result<(), HttpError> {
    if split_folder(key).is_none() {
        return Err((StatusCode::BAD_REQUEST, format!("not a valid key: {key:?}")));
    }
    if !may_write(roles, scope, key) {
        let why = if split_folder(key).is_some_and(|(k, _)| is_platform_only(scope, k)) {
            "it is published by the cell-templates CI only"
        } else if scope == Scope::Catalogue && !roles.catalogue_admin {
            "only catalogue admins may change it"
        } else {
            "it is read-only"
        };
        return Err((StatusCode::FORBIDDEN, format!("{} {key}: {why}", scope.as_str())));
    }
    Ok(())
}

/// A move or remove of folder `key` that would take the cell-templates CI's folder with it (the user
/// scope's `catalogue/`) is refused while that folder has objects.
async fn check_batch(s: &StorageRoutes, scope: Scope, key: &str) -> Result<(), HttpError> {
    let Some((folder, true)) = split_folder(key) else { return Ok(()) };
    if !contains_platform_only(scope, folder) {
        return Ok(());
    }
    let inside = platform_only_folder(scope);
    let list = s.storage.store().list(scope, &format!("{inside}/")).await.map_err(internal)?;
    if list.objects.is_empty() {
        return Ok(());
    }
    Err((StatusCode::FORBIDDEN, format!("{} {key}: it holds {inside}/, which is published by the cell-templates CI only", scope.as_str())))
}

fn writable(roles: Roles, scope: Scope, key: &str) -> Result<ObjectKey, HttpError> {
    let key = key_of(scope, key)?;
    check_write(roles, scope, key.key())?;
    Ok(key)
}

fn internal(err: anyhow::Error) -> HttpError {
    if let Some(conflict) = err.downcast_ref::<Conflict>() {
        return (StatusCode::CONFLICT, conflict.to_string());
    }
    if let Some(refused) = err.downcast_ref::<PlatformOnly>() {
        return (StatusCode::FORBIDDEN, refused.to_string());
    }
    tracing::warn!("storage: {err:#}");
    (StatusCode::BAD_GATEWAY, format!("storage: {err:#}"))
}

/// Errors of the request itself (bad keys, a folder into itself) are 400, not the store's.
fn bad_or_internal(err: anyhow::Error) -> HttpError {
    let text = format!("{err:#}");
    if err.downcast_ref::<Conflict>().is_none()
        && err.downcast_ref::<PlatformOnly>().is_none()
        && ["not a valid key", "moves to a file", "where it is already", "into itself", "no folder", "has more than"].iter().any(|m| text.contains(m))
    {
        return (StatusCode::BAD_REQUEST, text);
    }
    internal(err)
}

fn is_sha256(s: &str) -> bool {
    s.len() == 64 && s.bytes().all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f'))
}

async fn info(State(s): St, ConnectInfo(peer): ConnectInfo<SocketAddr>, headers: HeaderMap) -> Result<Response, HttpError> {
    let roles = s.admit(peer, &headers)?;
    let mut info: StorageInfo = s.storage.info(roles);
    info.edge_path = edge_path(info.mode, s.edge_data, Caller::of(peer, &headers).alias.as_deref(), s.storage.user().as_str());
    Ok(no_store(Json(info).into_response()))
}

async fn presign(
    State(s): St,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Json(request): Json<PresignRequest>,
) -> Result<Response, HttpError> {
    let roles = s.admit(peer, &headers)?;
    if s.storage.mode() != StorageMode::S3 {
        return Err((StatusCode::CONFLICT, "local storage: use the storage routes over HTTP".to_owned()));
    }
    if request.items.len() > MAX_PRESIGN_ITEMS {
        return Err((StatusCode::BAD_REQUEST, format!("at most {MAX_PRESIGN_ITEMS} items")));
    }
    let mut items = Vec::with_capacity(request.items.len());
    for item in request.items {
        items.push(match presign_one(&s, roles, item).await {
            Ok((url, expires_at)) => Presigned { url: Some(url), headers: Vec::new(), expires_at, error: None },
            Err((_, error)) => Presigned { url: None, headers: Vec::new(), expires_at: 0, error: Some(error) },
        });
    }
    Ok(no_store(Json(PresignResponse { items }).into_response()))
}

async fn presign_one(s: &StorageRoutes, roles: Roles, item: PresignItem) -> Result<(String, u64), HttpError> {
    let (key, method) = match item {
        // Files only: a PUT URL writes one object.
        PresignItem::Object { scope, key, method: Method::Put } => (writable(roles, scope, &key)?, Method::Put),
        PresignItem::Object { scope, key, method: Method::Get } => (key_of(scope, &key)?, Method::Get),
        PresignItem::Asset { sha256 } => {
            if !is_sha256(&sha256) {
                return Err((StatusCode::BAD_REQUEST, "expected a lowercase hex SHA-256".to_owned()));
            }
            // Only what the asset route would serve, and only once it is in the user's subtree.
            if !s.assets.contains(&sha256) {
                return Err((StatusCode::NOT_FOUND, format!("no asset {sha256}")));
            }
            let key = key_of(Scope::User, &asset_key(&sha256))?;
            s.uploader.ensure(&sha256).await.map_err(asset_error)?;
            (key, Method::Get)
        }
    };
    let (url, expires) = s.storage.store().presign(&key, method, PRESIGN_TTL).await.map_err(internal)?.ok_or_else(|| {
        (StatusCode::CONFLICT, "this store has no presigned URLs".to_owned())
    })?;
    Ok((url, expires.duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)))
}

fn asset_error(err: AssetError) -> HttpError {
    match err {
        AssetError::NotRegistered => (StatusCode::NOT_FOUND, "no such asset".to_owned()),
        AssetError::Store(err) => internal(err),
    }
}

/// A viewer missed assets on the edge path: the registered ones go into S3 (S3 mode).
async fn ensure_assets(
    State(s): St,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Json(request): Json<EnsureAssets>,
) -> Result<Response, HttpError> {
    s.admit(peer, &headers)?;
    if s.storage.mode() != StorageMode::S3 {
        return Err((StatusCode::CONFLICT, "local storage: no assets in S3".to_owned()));
    }
    if request.sha256s.len() > MAX_ENSURE_ITEMS {
        return Err((StatusCode::BAD_REQUEST, format!("at most {MAX_ENSURE_ITEMS} assets")));
    }
    let items = futures::future::join_all(request.sha256s.into_iter().map(|sha256| {
        let s = &s;
        async move {
            let result = if is_sha256(&sha256) {
                s.uploader.ensure(&sha256).await.map_err(|err| match err {
                    AssetError::NotRegistered => format!("no asset {sha256}"),
                    AssetError::Store(err) => format!("storage: {}", internal(err).1),
                })
            } else {
                Err("expected a lowercase hex SHA-256".to_owned())
            };
            EnsuredAsset { sha256, ok: result.is_ok(), error: result.err() }
        }
    }))
    .await;
    Ok(no_store(Json(EnsuredAssets { items }).into_response()))
}

async fn list(
    State(s): St,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    UrlPath(scope): UrlPath<String>,
    Query(query): Query<HashMap<String, String>>,
) -> Result<Response, HttpError> {
    s.admit(peer, &headers)?;
    let scope = scope_of(&scope)?;
    let prefix = query.get("prefix").map(String::as_str).unwrap_or("");
    if !valid_prefix(prefix) {
        return Err((StatusCode::BAD_REQUEST, format!("not a valid prefix: {prefix:?}")));
    }
    let mut list: ObjectList = s.storage.store().list(scope, prefix).await.map_err(internal)?;
    // The change log is the backends' own.
    list.objects.retain(|o| o.key != factoryos_protocol::storage::REVISION_KEY);
    Ok(no_store(Json(list).into_response()))
}

async fn get_object(
    State(s): St,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    UrlPath((scope, key)): UrlPath<(String, String)>,
) -> Result<Response, HttpError> {
    s.admit(peer, &headers)?;
    let key = key_of(scope_of(&scope)?, &key)?;
    let bytes = s.storage.store().get(&key).await.map_err(internal)?.ok_or_else(|| (StatusCode::NOT_FOUND, format!("no object {}", key.key())))?;
    let mut response = bytes.into_response();
    response.headers_mut().insert(header::CONTENT_TYPE, HeaderValue::from_static("application/octet-stream"));
    response.headers_mut().insert(header::X_CONTENT_TYPE_OPTIONS, HeaderValue::from_static("nosniff"));
    Ok(no_store(response))
}

async fn put_object(
    State(s): St,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    UrlPath((scope, key)): UrlPath<(String, String)>,
    body: Bytes,
) -> Result<StatusCode, HttpError> {
    let roles = s.admit(peer, &headers)?;
    let scope = scope_of(&scope)?;
    let key = writable(roles, scope, &key)?;
    s.storage.store().put(&key, body.to_vec()).await.map_err(internal)?;
    // S3 mode: into the working copy too, for the backend's tools.
    s.storage.commit(&key).await.map_err(internal)?;
    s.changed(scope, vec![(key.key().to_owned(), false)]).await;
    Ok(StatusCode::NO_CONTENT)
}

async fn delete_object(
    State(s): St,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    UrlPath((scope, key)): UrlPath<(String, String)>,
) -> Result<StatusCode, HttpError> {
    let roles = s.admit(peer, &headers)?;
    let scope = scope_of(&scope)?;
    let key = writable(roles, scope, &key)?;
    let changes = s.storage.remove(scope, key.key()).await.map_err(internal)?;
    s.changed(scope, changes).await;
    Ok(StatusCode::NO_CONTENT)
}

/// After a viewer's direct `PUT` to a presigned URL: into the working copy.
async fn commit(
    State(s): St,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Json(request): Json<Commit>,
) -> Result<StatusCode, HttpError> {
    let roles = s.admit(peer, &headers)?;
    let key = writable(roles, request.scope, &request.key)?;
    s.storage.commit(&key).await.map_err(internal)?;
    s.changed(request.scope, vec![(key.key().to_owned(), false)]).await;
    Ok(StatusCode::NO_CONTENT)
}

async fn make_folder(
    State(s): St,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Json(request): Json<Folder>,
) -> Result<StatusCode, HttpError> {
    let roles = s.admit(peer, &headers)?;
    let key = writable(roles, request.scope, request.key.trim_end_matches('/'))?;
    let changes = s.storage.make_folder(request.scope, key.key()).await.map_err(bad_or_internal)?;
    s.changed(request.scope, changes).await;
    Ok(StatusCode::NO_CONTENT)
}

async fn move_keys(
    State(s): St,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Json(request): Json<Move>,
) -> Result<Json<Published>, HttpError> {
    let roles = s.admit(peer, &headers)?;
    check_write(roles, request.scope, &request.from)?;
    check_write(roles, request.scope, &request.to)?;
    check_batch(&s, request.scope, &request.from).await?;
    let changes = s.storage.move_keys(request.scope, &request.from, &request.to, request.overwrite).await.map_err(bad_or_internal)?;
    let objects = changes.iter().filter(|(_, deleted)| !deleted).count();
    s.changed(request.scope, changes).await;
    Ok(Json(Published { objects }))
}

async fn remove(
    State(s): St,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Json(request): Json<Remove>,
) -> Result<Json<Published>, HttpError> {
    let roles = s.admit(peer, &headers)?;
    check_write(roles, request.scope, &request.key)?;
    check_batch(&s, request.scope, &request.key).await?;
    let changes = s.storage.remove(request.scope, &request.key).await.map_err(bad_or_internal)?;
    let objects = changes.len();
    s.changed(request.scope, changes).await;
    Ok(Json(Published { objects }))
}

async fn publish(
    State(s): St,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Json(request): Json<Publish>,
) -> Result<Json<Published>, HttpError> {
    let roles = s.admit(peer, &headers)?;
    if split_folder(&request.from).is_none() {
        return Err((StatusCode::BAD_REQUEST, format!("not a valid key: {:?}", request.from)));
    }
    check_write(roles, Scope::Catalogue, &request.to)?;
    let changes = s.storage.publish(&request.from, &request.to, request.overwrite).await.map_err(bad_or_internal)?;
    let objects = changes.len();
    s.changed(Scope::Catalogue, changes).await;
    tracing::info!("Storage: published {} file(s) of {} to the shared catalogue's {}", objects, request.from, request.to);
    Ok(Json(Published { objects }))
}

fn no_store(mut response: Response) -> Response {
    response.headers_mut().insert(header::CACHE_CONTROL, HeaderValue::from_static("no-store"));
    response
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::authz::ConfigAuthz;
    use crate::config_store::ConfigStore;
    use axum::body::Body;
    use axum::http::Request;
    use std::sync::Mutex;
    use tower::ServiceExt as _;

    fn temp(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("factoryos-storage-http-{name}-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        std::fs::create_dir_all(dir.join("products")).unwrap();
        std::fs::create_dir_all(dir.join("catalogue")).unwrap();
        std::fs::canonicalize(dir).unwrap()
    }

    async fn app(root: &std::path::Path, owner: Option<&str>, admins: Option<&str>) -> Router {
        app_told(root, owner, admins, Arc::default()).await
    }

    /// [`app`], its catalogue announcements into `told`.
    async fn app_told(
        root: &std::path::Path,
        owner: Option<&str>,
        admins: Option<&str>,
        told: Arc<Mutex<Vec<factoryos_protocol::storage::CatalogueChanged>>>,
    ) -> Router {
        let config = Arc::new(ConfigStore::open(root.join("factoryos.config.json")));
        let files = FilesService::new(config, &[], owner.map(str::to_owned)).unwrap();
        let (products, catalogue) = (root.join("products"), root.join("catalogue"));
        let storage = Storage::open(&Default::default(), owner, move || super::super::local::Layout {
            user: vec![("products".into(), products.clone()), ("catalogue".into(), catalogue.clone())],
            catalogue: Some(catalogue.clone()),
        })
        .await
        .unwrap();
        let sync = CatalogueSync::new(storage.clone(), None);
        sync.on_change(move |changed| told.lock().unwrap().push(changed.clone()));
        router(storage, files, AssetIndex::unrestricted(), ConfigAuthz::new(admins, owner.map(str::to_owned)).unwrap(), sync, false)
    }

    async fn call(app: &Router, method: &str, uri: &str, alias: Option<&str>, body: &[u8]) -> (StatusCode, Vec<u8>) {
        let mut request = Request::builder().method(method).uri(uri).header(header::HOST, "127.0.0.1:8080");
        if let Some(alias) = alias {
            request = request.header("x-user-alias", alias);
        }
        if body.first() == Some(&b'{') {
            request = request.header(header::CONTENT_TYPE, "application/json");
        }
        let mut request = request.body(Body::from(body.to_vec())).unwrap();
        request.extensions_mut().insert(ConnectInfo(SocketAddr::from(([127, 0, 0, 1], 5000))));
        let response = app.clone().oneshot(request).await.unwrap();
        let status = response.status();
        (status, axum::body::to_bytes(response.into_body(), 1 << 20).await.unwrap().to_vec())
    }

    fn json<T: serde::Serialize>(value: &T) -> Vec<u8> {
        serde_json::to_vec(value).unwrap()
    }

    #[tokio::test]
    async fn local_mode_round_trip_and_scoping() {
        let root = temp("local");
        let app = app(&root, Some("jdoe"), None).await;
        let (status, body) = call(&app, "GET", api::STORAGE_INFO, Some("jdoe"), b"").await;
        assert_eq!(status, StatusCode::OK);
        let info = serde_json::from_slice::<StorageInfo>(&body).unwrap();
        assert_eq!((info.mode, info.roles.catalogue_admin), (StorageMode::Local, false));
        let url = api::object(Scope::User, "products/box/product.json");
        assert_eq!(call(&app, "PUT", &url, Some("jdoe"), b"{}").await.0, StatusCode::NO_CONTENT);
        assert_eq!(call(&app, "GET", &url, Some("jdoe"), b"").await, (StatusCode::OK, b"{}".to_vec()));
        assert_eq!(std::fs::read(root.join("products/box/product.json")).unwrap(), b"{}");
        // Another user, by the edge's alias: refused.
        assert_eq!(call(&app, "GET", &url, Some("alice"), b"").await.0, StatusCode::FORBIDDEN);
        // Traversal: refused before any store sees it (also percent-encoded).
        for bad in ["/api/v1/storage/object/user/products/../../etc/passwd", "/api/v1/storage/object/user/products/%2E%2E/x"] {
            let (status, _) = call(&app, "GET", bad, Some("jdoe"), b"").await;
            assert!(status == StatusCode::BAD_REQUEST || status == StatusCode::NOT_FOUND, "{bad}: {status}");
        }
        // Backend-only prefixes are not writable.
        assert_eq!(call(&app, "PUT", &api::object(Scope::User, "assets/abc"), Some("jdoe"), b"x").await.0, StatusCode::FORBIDDEN);
        // No presigning in local mode.
        let request = json(&PresignRequest { items: vec![PresignItem::Asset { sha256: "0".repeat(64) }] });
        assert_eq!(call(&app, "POST", api::STORAGE_PRESIGN, Some("jdoe"), &request).await.0, StatusCode::CONFLICT);
        let (status, body) = call(&app, "GET", &api::list(Scope::User, "products/"), Some("jdoe"), b"").await;
        assert_eq!(status, StatusCode::OK);
        assert_eq!(serde_json::from_slice::<ObjectList>(&body).unwrap().objects.len(), 1);
        std::fs::remove_dir_all(&root).ok();
    }

    #[tokio::test]
    async fn only_catalogue_admins_change_the_shared_catalogue() {
        let root = temp("admins");
        std::fs::write(root.join("catalogue/a.urdf"), "<robot/>").unwrap();
        let object = api::object(Scope::Catalogue, "robots/b.urdf");
        let folder = json(&Folder { scope: Scope::Catalogue, key: "robots".into() });
        let moved = json(&Move { scope: Scope::Catalogue, from: "a.urdf".into(), to: "robots/a.urdf".into(), overwrite: false });
        let removed = json(&Remove { scope: Scope::Catalogue, key: "a.urdf".into() });
        let published = json(&Publish { from: "products/box/".into(), to: "product-collection/box/".into(), overwrite: false });

        // Not an admin (the edge's user, admins unset): reads only.
        let app = app(&root, Some("jdoe"), None).await;
        assert_eq!(call(&app, "GET", &api::object(Scope::Catalogue, "a.urdf"), Some("jdoe"), b"").await.0, StatusCode::OK);
        assert_eq!(call(&app, "PUT", &object, Some("jdoe"), b"x").await.0, StatusCode::FORBIDDEN);
        assert_eq!(call(&app, "DELETE", &api::object(Scope::Catalogue, "a.urdf"), Some("jdoe"), b"").await.0, StatusCode::FORBIDDEN);
        for (route, body) in [(api::STORAGE_FOLDER, &folder), (api::STORAGE_MOVE, &moved), (api::STORAGE_REMOVE, &removed), (api::STORAGE_PUBLISH, &published)] {
            assert_eq!(call(&app, "POST", route, Some("jdoe"), body).await.0, StatusCode::FORBIDDEN, "{route}");
        }
        let commit = json(&Commit { key: "a.urdf".into(), scope: Scope::Catalogue });
        assert_eq!(call(&app, "POST", api::STORAGE_COMMIT, Some("jdoe"), &commit).await.0, StatusCode::FORBIDDEN);
        assert_eq!(std::fs::read(root.join("catalogue/a.urdf")).unwrap(), b"<robot/>");
        // `none` takes it from the local user too.
        let app = self::app(&root, Some("jdoe"), Some("none")).await;
        assert_eq!(call(&app, "PUT", &object, None, b"x").await.0, StatusCode::FORBIDDEN);

        // An admin (by name): writes, makes folders, moves, removes, publishes.
        let app = self::app(&root, Some("jdoe"), Some("jdoe")).await;
        let (_, body) = call(&app, "GET", api::STORAGE_INFO, Some("jdoe"), b"").await;
        assert!(serde_json::from_slice::<StorageInfo>(&body).unwrap().roles.catalogue_admin);
        assert_eq!(call(&app, "POST", api::STORAGE_FOLDER, Some("jdoe"), &folder).await.0, StatusCode::NO_CONTENT);
        assert!(root.join("catalogue/robots/.keep").is_file());
        assert_eq!(call(&app, "POST", api::STORAGE_FOLDER, Some("jdoe"), &folder).await.0, StatusCode::CONFLICT);
        assert_eq!(call(&app, "PUT", &object, Some("jdoe"), b"<b/>").await.0, StatusCode::NO_CONTENT);
        assert_eq!(call(&app, "POST", api::STORAGE_MOVE, Some("jdoe"), &moved).await.0, StatusCode::OK);
        assert!(!root.join("catalogue/a.urdf").exists());
        assert_eq!(std::fs::read(root.join("catalogue/robots/a.urdf")).unwrap(), b"<robot/>");
        // A folder: renamed with everything in it; not into itself; not over something without overwrite.
        let rename = json(&Move { scope: Scope::Catalogue, from: "robots/".into(), to: "arms/".into(), overwrite: false });
        let (status, body) = call(&app, "POST", api::STORAGE_MOVE, Some("jdoe"), &rename).await;
        assert_eq!(status, StatusCode::OK, "{}", String::from_utf8_lossy(&body));
        assert_eq!(serde_json::from_slice::<Published>(&body).unwrap().objects, 3);
        assert!(root.join("catalogue/arms/a.urdf").is_file() && root.join("catalogue/arms/b.urdf").is_file() && !root.join("catalogue/robots").exists());
        let into_itself = json(&Move { scope: Scope::Catalogue, from: "arms/".into(), to: "arms/inner/".into(), overwrite: false });
        assert_eq!(call(&app, "POST", api::STORAGE_MOVE, Some("jdoe"), &into_itself).await.0, StatusCode::BAD_REQUEST);
        let file_to_folder = json(&Move { scope: Scope::Catalogue, from: "arms/a.urdf".into(), to: "x/".into(), overwrite: false });
        assert_eq!(call(&app, "POST", api::STORAGE_MOVE, Some("jdoe"), &file_to_folder).await.0, StatusCode::BAD_REQUEST);
        std::fs::write(root.join("catalogue/c.urdf"), "c").unwrap();
        let over = json(&Move { scope: Scope::Catalogue, from: "c.urdf".into(), to: "arms/a.urdf".into(), overwrite: false });
        assert_eq!(call(&app, "POST", api::STORAGE_MOVE, Some("jdoe"), &over).await.0, StatusCode::CONFLICT);
        // Traversal and the backend's prefixes, also for admins.
        for (from, to) in [("c.urdf", "../c.urdf"), ("../x", "c2.urdf"), ("c.urdf", "assets/c"), ("c.urdf", "a//b")] {
            let bad = json(&Move { scope: Scope::Catalogue, from: from.into(), to: to.into(), overwrite: true });
            let status = call(&app, "POST", api::STORAGE_MOVE, Some("jdoe"), &bad).await.0;
            assert!(status == StatusCode::BAD_REQUEST || status == StatusCode::FORBIDDEN, "{from} → {to}: {status}");
        }
        assert_eq!(call(&app, "PUT", &api::object(Scope::Catalogue, "assets/x"), Some("jdoe"), b"x").await.0, StatusCode::FORBIDDEN);
        // Publish from the user's products.
        std::fs::create_dir_all(root.join("products/box/urdf")).unwrap();
        std::fs::write(root.join("products/box/product.json"), "{}").unwrap();
        std::fs::write(root.join("products/box/urdf/box.urdf"), "<robot/>").unwrap();
        let (status, body) = call(&app, "POST", api::STORAGE_PUBLISH, Some("jdoe"), &published).await;
        assert_eq!(status, StatusCode::OK, "{}", String::from_utf8_lossy(&body));
        assert_eq!(std::fs::read(root.join("catalogue/product-collection/box/urdf/box.urdf")).unwrap(), b"<robot/>");
        assert_eq!(call(&app, "POST", api::STORAGE_PUBLISH, Some("jdoe"), &published).await.0, StatusCode::CONFLICT);
        // Remove a folder.
        let gone = json(&Remove { scope: Scope::Catalogue, key: "arms/".into() });
        assert_eq!(call(&app, "POST", api::STORAGE_REMOVE, Some("jdoe"), &gone).await.0, StatusCode::OK);
        assert!(!root.join("catalogue/arms").exists());
        // Another user than the owner stays out, admin list or not.
        assert_eq!(call(&app, "POST", api::STORAGE_REMOVE, Some("alice"), &removed).await.0, StatusCode::FORBIDDEN);

        // A pure local backend (no owner, admins unset): its user is the admin.
        let app = self::app(&root, None, None).await;
        assert_eq!(call(&app, "PUT", &api::object(Scope::Catalogue, "local.urdf"), None, b"l").await.0, StatusCode::NO_CONTENT);
        std::fs::remove_dir_all(&root).ok();
    }

    /// The cell templates are the cell-templates CI's: no write, move or remove there, catalogue admins
    /// included, in either scope; nor of a folder that holds them.
    #[tokio::test]
    async fn nobody_writes_the_cell_templates() {
        let root = temp("cell-templates");
        std::fs::create_dir_all(root.join("catalogue/cell-templates/ec2-codesys-kuka/1.0.0")).unwrap();
        std::fs::write(root.join("catalogue/cell-templates/index.json"), "{}").unwrap();
        std::fs::write(root.join("catalogue/a.urdf"), "<robot/>").unwrap();
        std::fs::create_dir_all(root.join("products/x/cell-templates")).unwrap();
        std::fs::write(root.join("products/x/cell-templates/index.json"), "forged").unwrap();
        std::fs::write(root.join("products/x/notes.md"), "x").unwrap();
        let app = app(&root, Some("jdoe"), Some("jdoe")).await;
        for (scope, key) in [
            (Scope::Catalogue, "cell-templates/index.json"),
            (Scope::Catalogue, "cell-templates/ec2-codesys-kuka/1.0.0/template.json"),
            (Scope::User, "catalogue/cell-templates/index.json"),
            // The local store may be on a case-insensitive file system (APFS).
            (Scope::Catalogue, "Cell-Templates/index.json"),
            (Scope::User, "catalogue/CELL-TEMPLATES/index.json"),
        ] {
            let (status, body) = call(&app, "PUT", &api::object(scope, key), Some("jdoe"), b"{}").await;
            assert_eq!(status, StatusCode::FORBIDDEN, "{scope:?} {key}");
            assert!(String::from_utf8_lossy(&body).contains("cell-templates CI"), "{}", String::from_utf8_lossy(&body));
            assert_eq!(call(&app, "DELETE", &api::object(scope, key), Some("jdoe"), b"").await.0, StatusCode::FORBIDDEN, "{scope:?} {key}");
        }
        let refused = [
            json(&Folder { scope: Scope::Catalogue, key: "cell-templates/x".into() }),
            json(&Move { scope: Scope::Catalogue, from: "a.urdf".into(), to: "cell-templates/a.urdf".into(), overwrite: false }),
            json(&Move { scope: Scope::Catalogue, from: "cell-templates/".into(), to: "old/".into(), overwrite: false }),
            json(&Remove { scope: Scope::Catalogue, key: "cell-templates/".into() }),
            json(&Remove { scope: Scope::User, key: "catalogue/cell-templates/".into() }),
            // The user scope's catalogue folder holds them: neither removed nor moved.
            json(&Remove { scope: Scope::User, key: "catalogue/".into() }),
            json(&Move { scope: Scope::User, from: "catalogue/".into(), to: "old-catalogue/".into(), overwrite: false }),
            json(&Publish { from: "products/".into(), to: "cell-templates/".into(), overwrite: true }),
            // Nor brought in by a folder moved into the user scope's catalogue folder.
            json(&Move { scope: Scope::User, from: "products/x/".into(), to: "catalogue/".into(), overwrite: true }),
            json(&Move { scope: Scope::User, from: "products/x/".into(), to: "catalogue/".into(), overwrite: false }),
        ];
        for (body, route) in refused.iter().zip([
            api::STORAGE_FOLDER,
            api::STORAGE_MOVE,
            api::STORAGE_MOVE,
            api::STORAGE_REMOVE,
            api::STORAGE_REMOVE,
            api::STORAGE_REMOVE,
            api::STORAGE_MOVE,
            api::STORAGE_PUBLISH,
            api::STORAGE_MOVE,
            api::STORAGE_MOVE,
        ]) {
            let (status, text) = call(&app, "POST", route, Some("jdoe"), body).await;
            assert_eq!(status, StatusCode::FORBIDDEN, "{route} {}: {}", String::from_utf8_lossy(body), String::from_utf8_lossy(&text));
        }
        assert_eq!(std::fs::read(root.join("catalogue/cell-templates/index.json")).unwrap(), b"{}");
        assert!(root.join("catalogue/a.urdf").is_file());
        assert!(root.join("products/x/cell-templates/index.json").is_file() && !root.join("catalogue/notes.md").exists(), "nothing of the batch moved");
        // Next to them, as before.
        assert_eq!(call(&app, "PUT", &api::object(Scope::Catalogue, "cell-templates-notes/a.md"), Some("jdoe"), b"x").await.0, StatusCode::NO_CONTENT);
        std::fs::remove_dir_all(&root).ok();
    }

    /// An S3-mode app over a fake S3 (owner `jdoe`), and its asset index.
    fn s3_app(root: &std::path::Path, edge_data: bool) -> (Router, Arc<super::super::sync::tests_support::FakeS3>, Arc<AssetIndex>) {
        let config = Arc::new(ConfigStore::open(root.join("factoryos.config.json")));
        let files = FilesService::new(config, &[], Some("jdoe".to_owned())).unwrap();
        let (s3, storage) = super::super::sync::tests_support::storage_at(root);
        let sync = CatalogueSync::new(storage.clone(), None);
        let assets = AssetIndex::unrestricted();
        let app = router(storage, files, assets.clone(), ConfigAuthz::new(None, Some("jdoe".to_owned())).unwrap(), sync, edge_data);
        (app, s3, assets)
    }

    async fn edge_path_of(app: &Router, alias: Option<&str>) -> Option<String> {
        let (status, body) = call(app, "GET", api::STORAGE_INFO, alias, b"").await;
        assert_eq!(status, StatusCode::OK, "{}", String::from_utf8_lossy(&body));
        serde_json::from_slice::<StorageInfo>(&body).unwrap().edge_path
    }

    #[test]
    fn the_edge_path_is_the_owners_in_s3_mode_with_the_flag() {
        assert_eq!(edge_path(StorageMode::S3, true, Some("jdoe"), "jdoe").as_deref(), Some(EDGE_PATH));
        // Another user (the edge maps to the caller's own subtree, not the workspace's).
        assert_eq!(edge_path(StorageMode::S3, true, Some("alice"), "jdoe"), None);
        // Loopback, without the edge.
        assert_eq!(edge_path(StorageMode::S3, true, None, "jdoe"), None);
        assert_eq!(edge_path(StorageMode::S3, false, Some("jdoe"), "jdoe"), None);
        assert_eq!(edge_path(StorageMode::Local, true, Some("jdoe"), "jdoe"), None);
    }

    #[tokio::test]
    async fn info_names_the_edge_path_only_when_it_serves() {
        let root = temp("edge-info");
        let (app, _, _) = s3_app(&root, true);
        assert_eq!(edge_path_of(&app, Some("jdoe")).await.as_deref(), Some(EDGE_PATH));
        assert_eq!(edge_path_of(&app, None).await, None, "loopback: no edge");
        // Another alias is not admitted at all.
        assert_eq!(call(&app, "GET", api::STORAGE_INFO, Some("alice"), b"").await.0, StatusCode::FORBIDDEN);
        let (app, _, _) = s3_app(&root, false);
        assert_eq!(edge_path_of(&app, Some("jdoe")).await, None, "without --edge-data");
        // Local mode, with the flag.
        let local = temp("edge-info-local");
        let config = Arc::new(ConfigStore::open(local.join("factoryos.config.json")));
        let files = FilesService::new(config, &[], Some("jdoe".to_owned())).unwrap();
        let (products, catalogue) = (local.join("products"), local.join("catalogue"));
        let storage = Storage::open(&Default::default(), Some("jdoe"), move || super::super::local::Layout {
            user: vec![("products".into(), products.clone())],
            catalogue: Some(catalogue.clone()),
        })
        .await
        .unwrap();
        let sync = CatalogueSync::new(storage.clone(), None);
        let app = router(storage, files, AssetIndex::unrestricted(), ConfigAuthz::new(None, Some("jdoe".to_owned())).unwrap(), sync, true);
        assert_eq!(edge_path_of(&app, Some("jdoe")).await, None, "local mode");
        let ensure = json(&EnsureAssets { sha256s: vec!["0".repeat(64)] });
        assert_eq!(call(&app, "POST", api::STORAGE_ASSETS_ENSURE, Some("jdoe"), &ensure).await.0, StatusCode::CONFLICT);
        std::fs::remove_dir_all(&root).ok();
        std::fs::remove_dir_all(&local).ok();
    }

    #[tokio::test]
    async fn ensure_uploads_registered_assets_only() {
        let root = temp("ensure");
        std::fs::create_dir_all(root.join("cells")).unwrap();
        let (app, s3, assets) = s3_app(&root, true);
        std::fs::write(root.join("products/part.stl"), "solid part").unwrap();
        let part = assets.register(&root.join("products/part.stl")).unwrap();
        // Uploaded ahead (the router attached the uploader to the index).
        let key = asset_key(&part.sha256);
        super::super::sync::tests_support::eventually("uploaded ahead", || s3.bytes(Scope::User, &key).is_some()).await;
        // One whose upload ahead failed (S3 away meanwhile) comes when a viewer asks.
        s3.down.store(true, std::sync::atomic::Ordering::SeqCst);
        std::fs::write(root.join("products/other.stl"), "solid other").unwrap();
        let other = assets.register(&root.join("products/other.stl")).unwrap();
        tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        s3.down.store(false, std::sync::atomic::Ordering::SeqCst);
        let unknown = "1".repeat(64);
        let request = json(&EnsureAssets { sha256s: vec![other.sha256.clone(), unknown.clone(), "../x".into(), part.sha256.clone()] });
        let (status, body) = call(&app, "POST", api::STORAGE_ASSETS_ENSURE, Some("jdoe"), &request).await;
        assert_eq!(status, StatusCode::OK, "{}", String::from_utf8_lossy(&body));
        let ensured = serde_json::from_slice::<EnsuredAssets>(&body).unwrap();
        let oks: Vec<bool> = ensured.items.iter().map(|i| i.ok).collect();
        assert_eq!(oks, [true, false, false, true]);
        assert_eq!(ensured.items[1].sha256, unknown);
        assert!(ensured.items[2].error.as_deref().unwrap().contains("SHA-256"));
        assert_eq!(s3.bytes(Scope::User, &asset_key(&other.sha256)).unwrap(), b"solid other");
        // Only the registered assets were written.
        let mut expected = vec![asset_key(&part.sha256), asset_key(&other.sha256)];
        expected.sort();
        assert_eq!(s3.keys(Scope::User), expected);
        // Admitted like the other storage routes; at most MAX_ENSURE_ITEMS.
        assert_eq!(call(&app, "POST", api::STORAGE_ASSETS_ENSURE, Some("alice"), &request).await.0, StatusCode::FORBIDDEN);
        let many = json(&EnsureAssets { sha256s: vec![unknown; MAX_ENSURE_ITEMS + 1] });
        assert_eq!(call(&app, "POST", api::STORAGE_ASSETS_ENSURE, Some("jdoe"), &many).await.0, StatusCode::BAD_REQUEST);
        std::fs::remove_dir_all(&root).ok();
    }

    #[tokio::test]
    async fn local_catalogue_writes_are_announced_as_in_s3_mode() {
        let root = temp("announce");
        let told = Arc::new(Mutex::new(Vec::new()));
        let app = app_told(&root, None, None, told.clone()).await;
        assert_eq!(call(&app, "PUT", &api::object(Scope::Catalogue, "robots/a.urdf"), None, b"a").await.0, StatusCode::NO_CONTENT);
        // The user's own files are no change of the shared catalogue.
        assert_eq!(call(&app, "PUT", &api::object(Scope::User, "products/p/product.json"), None, b"{}").await.0, StatusCode::NO_CONTENT);
        let gone = json(&Remove { scope: Scope::Catalogue, key: "robots/".into() });
        assert_eq!(call(&app, "POST", api::STORAGE_REMOVE, None, &gone).await.0, StatusCode::OK);
        let told = told.lock().unwrap().clone();
        assert_eq!(told.len(), 2, "{told:?}");
        assert_eq!((told[0].keys.as_slice(), told[0].remote, told[0].revision), (["robots/a.urdf".to_owned()].as_slice(), false, 0));
        assert_eq!(told[1].keys, ["robots/a.urdf"]);
        std::fs::remove_dir_all(&root).ok();
    }
}
