//! Product & Task: the products (`<products dir>/<name>/product.json`) and their 3D preview, the
//! part with its tasks' operations, published as a renderer-neutral scene on
//! `factoryos/scene/product` ([`factoryos_protocol::scene`]) like the Robo Dresser's. The task-author agent turns a product into a cell's task program; see
//! `factoryos_protocol::config::DEFAULT_TASK_PROMPT`.
//!
//! The catalogue's ready-made products (`<catalogue>/product-collection/<name>/`, the same folder
//! as in the products dir) are loaded by copying one into the products dir, and a saved product
//! (its tasks included) is put into the catalogue the same way back. A URDF product's URDF and
//! meshes are in its `urdf/` folder: an uploaded zip extracted, or a catalogue fixture copied.
//! Uploads and writes to the catalogue are admitted as the workspace files' are
//! (`files::admit_transfer`), merged outside the CORS layer.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use std::net::SocketAddr;

use axum::body::Bytes;
use axum::extract::{ConnectInfo, DefaultBodyLimit, Path as UrlPath, State};
use axum::http::{HeaderMap, StatusCode};
use axum::routing::{get, post, put};
use axum::{Json, Router};
use factoryos_protocol::files::MAX_TRANSFER_BYTES;
use factoryos_protocol::product::{
    CatalogueProduct, CatalogueProducts, CatalogueTransfer, Geometry, LoadProduct, PreviewProduct, Product, ProductFile, ProductUrdfs,
    ProductsInfo, Target, Task, UrdfFromCatalogue, valid_name,
};
use factoryos_protocol::scene::{MeshAsset, ModelRef, Node, NodeKind, Pose3, Rgba, ViewHints};
use factoryos_protocol::{api, keys};

use crate::AppState;
use crate::bus::Bus;
use crate::config_store::ConfigStore;
use crate::services::assets::AssetIndex;
use crate::services::scene_pub::ScenePublisher;

/// The 3D view's title.
const VIEW_TITLE: &str = "Product & Task";
const PRODUCT_FILE: &str = "product.json";
/// The catalogue's products, under the catalogue.
const CATALOGUE_PRODUCTS: &str = "product-collection";
/// The catalogue's fixtures, whose URDFs a product can use.
const CATALOGUE_FIXTURES: &str = "urdf-fixture-collection";
/// A URDF product's URDF and meshes, under its folder.
pub(crate) const URDF_DIR: &str = "urdf";
/// Marks the fixture files written by this version; older ones are rewritten.
const FIXTURE_FORMAT: &str = "factoryos-fixture-2";
const PART_COLOR: u32 = 0x9aa4b2ff;
/// The part's node: a box, a mesh, or a URDF model with its root link at the origin.
const PART_NODE: &str = "part";
/// One colour per task.
const TASK_COLORS: [u32; 6] = [0xff6b3dff, 0x3dbdffff, 0xffd23dff, 0x8cff3dff, 0xd23dffff, 0x3dffc4ff];

/// The scene the preview is drawn into, kept while the product's name and kind of geometry (and a
/// URDF's file) stay the same, so edits update the view in place instead of starting a new one.
struct Preview {
    scene_id: String,
    name: String,
    geometry: String,
    /// A URDF's modification time when drawn: it is drawn again only once changed (an upload).
    urdf_drawn: Option<std::time::SystemTime>,
    /// The task nodes drawn last, to remove those of tasks removed since.
    task_nodes: Vec<String>,
}

pub struct ProductService {
    config: Arc<ConfigStore>,
    /// The renderer-neutral 3D preview.
    scene: Arc<ScenePublisher>,
    assets: Arc<AssetIndex>,
    preview: Arc<Mutex<Option<Preview>>>,
    /// Why the URDF loaded last could not be drawn, for the next preview to answer with.
    urdf_error: Arc<Mutex<Option<String>>>,
}

impl ProductService {
    pub fn new(bus: &Bus, config: Arc<ConfigStore>, assets: Arc<AssetIndex>) -> Arc<Self> {
        Arc::new(Self {
            config,
            scene: ScenePublisher::new(bus, keys::SCENE_PRODUCT),
            assets,
            preview: Arc::default(),
            urdf_error: Arc::default(),
        })
    }

    pub fn dir(&self) -> PathBuf {
        products_dir(&self.config)
    }

    pub fn list(&self) -> ProductsInfo {
        let dir = self.dir();
        let mut info = ProductsInfo {
            dir: dir.to_string_lossy().into_owned(),
            ..ProductsInfo::default()
        };
        match std::fs::read_dir(&dir) {
            Ok(entries) => {
                info.products = entries
                    .flatten()
                    .filter(|entry| entry.path().join(PRODUCT_FILE).is_file())
                    .filter_map(|entry| entry.file_name().into_string().ok())
                    .collect();
                info.products.sort();
            }
            // No products yet: the folder is created by the first save.
            Err(err) if err.kind() == std::io::ErrorKind::NotFound => {}
            Err(err) => info.error = Some(format!("{}: {err}", dir.display())),
        }
        info
    }

    fn file(&self, name: &str) -> anyhow::Result<PathBuf> {
        anyhow::ensure!(valid_name(name), "not a product name: {name:?} (letters, digits, _ and - only)");
        Ok(self.dir().join(name).join(PRODUCT_FILE))
    }

    pub fn load(&self, name: &str) -> anyhow::Result<ProductFile> {
        let path = self.file(name)?;
        let text = std::fs::read_to_string(&path).map_err(|err| anyhow::anyhow!("{}: {err}", path.display()))?;
        let mut product: Product =
            serde_json::from_str(&text).map_err(|err| anyhow::anyhow!("{}: {err}", path.display()))?;
        product.name = name.to_owned();
        Ok(ProductFile {
            product,
            path: path.to_string_lossy().into_owned(),
        })
    }

    pub fn save(&self, product: Product) -> anyhow::Result<ProductFile> {
        let path = self.file(&product.name)?;
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir).map_err(|err| anyhow::anyhow!("{}: {err}", dir.display()))?;
        }
        std::fs::write(&path, serde_json::to_string_pretty(&product)? + "\n")
            .map_err(|err| anyhow::anyhow!("{}: {err}", path.display()))?;
        write_fixture(path.parent().unwrap_or(Path::new(".")), &product)?;
        tracing::info!("Product & Task: saved {}", path.display());
        Ok(ProductFile {
            product,
            path: path.to_string_lossy().into_owned(),
        })
    }

    /// Draw `product` (unsaved edits included) into the preview scene.
    pub fn preview(&self, product: &Product) -> anyhow::Result<()> {
        let geometry = match &product.geometry {
            Geometry::Urdf { path } => format!("urdf:{}", path.trim()),
            other => other.kind().to_owned(),
        };
        let mut preview = self.preview.lock().unwrap();
        let new_scene = preview.as_ref().is_none_or(|p| p.name != product.name || p.geometry != geometry);
        let part = self.resolve_part(product);
        let Ok((part, part_size)) = part else {
            if new_scene {
                // A new, empty scene: the previous product's isn't shown under this one's name.
                let scene_id = self.scene.set(Vec::new(), view());
                *self.urdf_error.lock().unwrap() = None;
                *preview = Some(Preview { scene_id, name: product.name.clone(), geometry, urdf_drawn: None, task_nodes: Vec::new() });
            }
            return part.map(|_| ());
        };
        let mut nodes: Vec<Node> = part_node(product, &part).into_iter().collect();
        let tasks = task_nodes(&product.tasks, part_size);
        let task_ids: Vec<String> = tasks.iter().map(|n| n.id.clone()).collect();
        nodes.extend(tasks);
        if new_scene {
            let scene_id = self.scene.set(nodes, view());
            // An error of the previous scene's URDF is not this one's.
            *self.urdf_error.lock().unwrap() = None;
            *preview = Some(Preview { scene_id, name: product.name.clone(), geometry, urdf_drawn: None, task_nodes: task_ids });
        } else {
            let p = preview.as_mut().expect("not a new scene");
            let gone = p.task_nodes.iter().filter(|id| !task_ids.contains(id)).cloned().collect();
            // A URDF's part node is not in `nodes`: it stays as loaded.
            self.scene.update(nodes, gone, Some(view()));
            p.task_nodes = task_ids;
        }
        let preview = preview.as_mut().expect("just set");
        if let Geometry::Urdf { path } = &product.geometry
            && !path.trim().is_empty()
        {
            if let Some(err) = self.urdf_error.lock().unwrap().take() {
                // Drawn again on the next preview.
                preview.urdf_drawn = None;
                anyhow::bail!(err);
            }
            let file = self.mesh_path(&product.name, path);
            let modified = std::fs::metadata(&file).and_then(|m| m.modified()).ok();
            if preview.urdf_drawn.is_none() || preview.urdf_drawn != modified {
                self.load_urdf(preview.scene_id.clone(), file, self.dir().join(&product.name), modified);
                preview.urdf_drawn = modified;
            }
        }
        Ok(())
    }

    /// The part's geometry for [`part_node`] (a URDF's is loaded by [`Self::load_urdf`]), and its
    /// size for the tasks' markers.
    fn resolve_part(&self, product: &Product) -> anyhow::Result<(Part, f32)> {
        Ok(match &product.geometry {
            Geometry::Box { size } => (Part::Box, size.map(|s| s.max(0.001)).into_iter().fold(0.0_f32, f32::max)),
            Geometry::Mesh { path, .. } => {
                let file = self.mesh_path(&product.name, path);
                anyhow::ensure!(file.is_file(), "mesh not found: {}", file.display());
                (Part::Mesh(self.assets.mesh_asset(&file)?), 1.0)
            }
            // None chosen yet: only the tasks.
            Geometry::Urdf { path } if path.trim().is_empty() => (Part::Pending, 1.0),
            Geometry::Urdf { path } => {
                let file = self.mesh_path(&product.name, path);
                anyhow::ensure!(file.is_file(), "URDF not found: {}", file.display());
                (Part::Pending, 1.0)
            }
        })
    }

    /// Register URDF `file` and its meshes in the background and add it as the part of scene
    /// `scene_id`, if that is still the one shown with this `modified` URDF; a failure goes to
    /// `urdf_error`. Large URDFs take a while to hash: the preview answers at once and it fills in.
    fn load_urdf(&self, scene_id: String, file: PathBuf, product_dir: PathBuf, modified: Option<std::time::SystemTime>) {
        let (assets, scene, preview, error) = (self.assets.clone(), self.scene.clone(), self.preview.clone(), self.urdf_error.clone());
        // Where `write_urdf_fixture` finds the meshes too (its `resolve_mesh`).
        let search = vec![file.parent().unwrap_or(&product_dir).to_owned(), product_dir.join(URDF_DIR)];
        std::thread::spawn(move || match assets.model_ref_with(&file, &search) {
            Ok(model) => {
                let preview = preview.lock().unwrap();
                if preview.as_ref().is_some_and(|p| p.scene_id == scene_id && p.urdf_drawn == modified) {
                    scene.upsert(model_node(model));
                }
            }
            Err(err) => {
                tracing::warn!("Product & Task: {}: {err:#}", file.display());
                // Only for the preview it was loaded for: a slow failure must not land on the next product.
                let preview = preview.lock().unwrap();
                if preview.as_ref().is_some_and(|p| p.scene_id == scene_id && p.urdf_drawn == modified) {
                    *error.lock().unwrap() = Some(format!("{}: {err:#}", file.display()));
                }
            }
        });
    }

    pub(crate) fn mesh_path(&self, product: &str, path: &str) -> PathBuf {
        let path = Path::new(path.trim());
        if path.is_absolute() { path.to_owned() } else { self.dir().join(product).join(path) }
    }

    fn catalogue(&self) -> PathBuf {
        PathBuf::from(self.config.resolve(&self.config.get().catalogue))
    }

    /// `<catalogue>/product-collection`.
    pub fn catalogue_dir(&self) -> PathBuf {
        self.catalogue().join(CATALOGUE_PRODUCTS)
    }

    pub fn catalogue_list(&self) -> CatalogueProducts {
        let dir = self.catalogue_dir();
        let mut info = CatalogueProducts {
            dir: dir.to_string_lossy().into_owned(),
            ..CatalogueProducts::default()
        };
        match std::fs::read_dir(&dir) {
            Ok(entries) => {
                info.products = entries
                    .flatten()
                    .filter_map(|entry| {
                        let name = entry.file_name().into_string().ok().filter(|name| valid_name(name))?;
                        let text = std::fs::read_to_string(entry.path().join(PRODUCT_FILE)).ok()?;
                        match serde_json::from_str::<Product>(&text) {
                            Ok(product) => Some(CatalogueProduct {
                                name,
                                description: product.description,
                                geometry: product.geometry.kind().to_owned(),
                                tasks: product.tasks.len(),
                            }),
                            Err(err) => {
                                tracing::warn!("{}: {err}", entry.path().join(PRODUCT_FILE).display());
                                None
                            }
                        }
                    })
                    .collect();
                info.products.sort_by(|a, b| a.name.cmp(&b.name));
            }
            Err(err) if err.kind() == std::io::ErrorKind::NotFound => {}
            Err(err) => info.error = Some(format!("{}: {err}", dir.display())),
        }
        info
    }

    /// Copy the catalogue's product `name` into the products dir.
    pub fn catalogue_load(&self, request: &CatalogueTransfer) -> anyhow::Result<ProductFile> {
        anyhow::ensure!(valid_name(&request.name), "not a product name: {:?}", request.name);
        let from = self.catalogue_dir().join(&request.name);
        anyhow::ensure!(from.join(PRODUCT_FILE).is_file(), "no product {} in the catalogue ({})", request.name, from.display());
        replace_dir(&from, &self.dir().join(&request.name), request.overwrite)?;
        let loaded = self.load(&request.name)?;
        // The copy's fixture URDF points into its own folder.
        write_fixture(&self.dir().join(&request.name), &loaded.product)?;
        tracing::info!("Product & Task: {} copied from {}", request.name, from.display());
        Ok(loaded)
    }

    /// Copy the saved product `name` (with its tasks, URDF and meshes) into the catalogue.
    pub fn catalogue_publish(&self, request: &CatalogueTransfer) -> anyhow::Result<CatalogueProducts> {
        self.file(&request.name)?;
        let from = self.dir().join(&request.name);
        anyhow::ensure!(from.join(PRODUCT_FILE).is_file(), "save product {} first", request.name);
        let to = self.catalogue_dir().join(&request.name);
        replace_dir(&from, &to, request.overwrite)?;
        tracing::info!("Product & Task: {} saved to the catalogue, {}", request.name, to.display());
        Ok(self.catalogue_list())
    }

    /// The catalogue's fixtures, by name.
    pub fn fixtures(&self) -> Vec<String> {
        let dir = self.catalogue().join(CATALOGUE_FIXTURES);
        let Ok(entries) = std::fs::read_dir(&dir) else { return Vec::new() };
        let mut names: Vec<String> = entries
            .flatten()
            .filter(|entry| entry.path().is_dir())
            .filter_map(|entry| entry.file_name().into_string().ok())
            .filter(|name| !name.starts_with('.'))
            .collect();
        names.sort();
        names
    }

    /// Product `name`'s folder, created if new (a product's URDF may come before its first save).
    fn product_dir(&self, name: &str) -> anyhow::Result<PathBuf> {
        anyhow::ensure!(valid_name(name), "not a product name: {name:?} (letters, digits, _ and - only)");
        let dir = self.dir().join(name);
        std::fs::create_dir_all(&dir).map_err(|err| anyhow::anyhow!("{}: {err}", dir.display()))?;
        Ok(dir)
    }

    /// Extract `zip` (a URDF and its meshes) into product `name`'s `urdf/` folder, replacing it.
    pub fn upload_urdf(&self, name: &str, zip: &[u8]) -> anyhow::Result<ProductUrdfs> {
        let dir = self.product_dir(name)?;
        let staging = staging_dir(&dir, URDF_DIR);
        let extracted = extract_zip(zip, &staging, MAX_TRANSFER_BYTES);
        if let Err(err) = extracted {
            std::fs::remove_dir_all(&staging).ok();
            return Err(err);
        }
        let found = urdfs_in(&staging, Path::new(URDF_DIR));
        if found.is_empty() {
            std::fs::remove_dir_all(&staging).ok();
            anyhow::bail!("the zip has no .urdf file");
        }
        swap_in(&staging, &dir.join(URDF_DIR))?;
        tracing::info!("Product & Task: {name}: URDF uploaded ({} file(s))", found.len());
        Ok(ProductUrdfs { urdfs: found })
    }

    /// Copy the catalogue's fixture into product `product`'s `urdf/` folder, replacing it.
    pub fn urdf_from_catalogue(&self, request: &UrdfFromCatalogue) -> anyhow::Result<ProductUrdfs> {
        anyhow::ensure!(
            !request.fixture.is_empty() && Path::new(&request.fixture).components().count() == 1 && !request.fixture.starts_with('.'),
            "not a fixture name: {:?}",
            request.fixture
        );
        let from = self.catalogue().join(CATALOGUE_FIXTURES).join(&request.fixture);
        anyhow::ensure!(from.is_dir(), "no fixture {} in the catalogue ({})", request.fixture, from.display());
        let dir = self.product_dir(&request.product)?;
        let staging = staging_dir(&dir, URDF_DIR);
        if let Err(err) = copy_tree(&from, &staging) {
            std::fs::remove_dir_all(&staging).ok();
            return Err(err);
        }
        swap_in(&staging, &dir.join(URDF_DIR))?;
        Ok(ProductUrdfs { urdfs: urdfs_in(&dir.join(URDF_DIR), Path::new(URDF_DIR)) })
    }
}

/// A product folder that is already there and is not to be replaced.
#[derive(Debug)]
struct Exists(String);

impl std::fmt::Display for Exists {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} exists already", self.0)
    }
}

impl std::error::Error for Exists {}

/// A fresh hidden folder next to `name` in `dir`, to fill before [`swap_in`].
fn staging_dir(dir: &Path, name: &str) -> PathBuf {
    let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_nanos());
    dir.join(format!(".{name}.fos-staging-{}-{nanos}", std::process::id()))
}

/// Put the filled `staging` folder at `to`, replacing what was there.
fn swap_in(staging: &Path, to: &Path) -> anyhow::Result<()> {
    if to.exists() {
        std::fs::remove_dir_all(to).map_err(|err| anyhow::anyhow!("{}: {err}", to.display()))?;
    }
    std::fs::rename(staging, to).map_err(|err| anyhow::anyhow!("{}: {err}", to.display()))
}

/// Copy folder `from` to `to` through a staging folder beside it, so a failed copy leaves `to` as
/// it was; an existing `to` only with `overwrite` (else [`Exists`]).
fn replace_dir(from: &Path, to: &Path, overwrite: bool) -> anyhow::Result<()> {
    if to.exists() && !overwrite {
        return Err(Exists(to.display().to_string()).into());
    }
    let parent = to.parent().unwrap_or(Path::new("."));
    std::fs::create_dir_all(parent).map_err(|err| anyhow::anyhow!("{}: {err}", parent.display()))?;
    let name = to.file_name().map_or_else(|| "product".to_owned(), |n| n.to_string_lossy().into_owned());
    let staging = staging_dir(parent, &name);
    if let Err(err) = copy_tree(from, &staging) {
        std::fs::remove_dir_all(&staging).ok();
        return Err(err);
    }
    swap_in(&staging, to)
}

/// Copy folder `from` to `to`: files and folders, not symlinks, nor the staging folders of
/// another copy.
fn copy_tree(from: &Path, to: &Path) -> anyhow::Result<()> {
    std::fs::create_dir_all(to).map_err(|err| anyhow::anyhow!("{}: {err}", to.display()))?;
    for entry in std::fs::read_dir(from).map_err(|err| anyhow::anyhow!("{}: {err}", from.display()))? {
        let entry = entry?;
        let kind = entry.file_type()?;
        if entry.file_name().to_string_lossy().contains(".fos-staging-") {
            continue;
        }
        let target = to.join(entry.file_name());
        if kind.is_dir() {
            copy_tree(&entry.path(), &target)?;
        } else if kind.is_file() {
            std::fs::copy(entry.path(), &target).map_err(|err| anyhow::anyhow!("{}: {err}", entry.path().display()))?;
        }
    }
    Ok(())
}

/// Extract `zip` into `dest`: only entries whose path stays inside it, no symlinks, and at most
/// `limit` bytes in all (counted as written, not as the zip claims).
fn extract_zip(zip: &[u8], dest: &Path, limit: u64) -> anyhow::Result<()> {
    use std::io::Read as _;
    let mut archive = zip::ZipArchive::new(std::io::Cursor::new(zip)).map_err(|err| anyhow::anyhow!("not a zip: {err}"))?;
    std::fs::create_dir_all(dest)?;
    let mut left = limit;
    for i in 0..archive.len() {
        let mut file = archive.by_index(i)?;
        let Some(rel) = file.enclosed_name() else {
            anyhow::bail!("zip entry {:?} points outside its folder", file.name().unwrap_or_default());
        };
        anyhow::ensure!(!file.is_symlink(), "zip entry {:?} is a symlink", file.name().unwrap_or_default());
        let path = dest.join(rel);
        if file.is_dir() {
            std::fs::create_dir_all(&path)?;
            continue;
        }
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let mut out = std::fs::File::create(&path)?;
        let written = std::io::copy(&mut (&mut file).take(left + 1), &mut out)?;
        anyhow::ensure!(written <= left, "the zip unpacks to more than {} MiB", limit / (1024 * 1024));
        left -= written;
    }
    Ok(())
}

/// The `.urdf` files under `dir`, as `prefix/<path below dir>` (`/`-separated), sorted.
fn urdfs_in(dir: &Path, prefix: &Path) -> Vec<String> {
    fn walk(dir: &Path, rel: &Path, out: &mut Vec<String>) {
        let Ok(entries) = std::fs::read_dir(dir) else { return };
        for entry in entries.flatten() {
            let Ok(kind) = entry.file_type() else { continue };
            let rel = rel.join(entry.file_name());
            if kind.is_dir() {
                walk(&entry.path(), &rel, out);
            } else if kind.is_file() && rel.extension().and_then(|e| e.to_str()).is_some_and(|e| e.to_ascii_lowercase() == "urdf") {
                out.push(rel.to_string_lossy().replace('\\', "/"));
            }
        }
    }
    let mut out = Vec::new();
    walk(dir, prefix, &mut out);
    out.sort();
    out
}

/// `products_dir`, else `products/` next to the config file: with the backend's own data, not in a
/// release or asset folder a deploy replaces.
pub fn products_dir(config: &ConfigStore) -> PathBuf {
    let products_dir = config.get().products_dir;
    let dir = if products_dir.trim().is_empty() { "products" } else { products_dir.as_str() };
    PathBuf::from(config.resolve(dir))
}

/// The products the Robo Fitter can place, by name: those with a `product.json`, whose fixture
/// URDF is (re)written here when it is missing or older than the product.
pub fn placeable_products(dir: &Path) -> Vec<String> {
    let Ok(entries) = std::fs::read_dir(dir) else { return Vec::new() };
    let mut names: Vec<String> = entries
        .flatten()
        .filter_map(|entry| {
            let folder = entry.path();
            let name = entry.file_name().into_string().ok()?;
            let json = folder.join(PRODUCT_FILE);
            let urdf = folder.join(format!("{name}.urdf"));
            let modified = |path: &Path| std::fs::metadata(path).and_then(|m| m.modified()).ok();
            let stale = modified(&urdf).is_none_or(|urdf_time| modified(&json).is_some_and(|json_time| json_time > urdf_time))
                || !std::fs::read_to_string(&urdf).is_ok_and(|text| text.contains(FIXTURE_FORMAT));
            if stale {
                let product: Product = serde_json::from_str(&std::fs::read_to_string(&json).ok()?).ok()?;
                let product = Product { name: name.clone(), ..product };
                if let Err(err) = write_fixture(&folder, &product) {
                    tracing::warn!("{}: fixture URDF: {err:#}", folder.display());
                    return None;
                }
            }
            Some(name)
        })
        .collect();
    names.sort();
    names
}

/// The product as a Robo Fitter fixture: `<folder>/<name>.urdf`, one fixed link with the part as
/// visual and collision geometry. A box becomes `meshes/<name>.obj` (the USDA composer reads OBJ
/// meshes, not URDF primitives); a mesh product references its own file.
fn write_fixture(folder: &Path, product: &Product) -> anyhow::Result<PathBuf> {
    let name = &product.name;
    let (mesh, scale) = match &product.geometry {
        Geometry::Urdf { path } => return write_urdf_fixture(folder, name, path),
        Geometry::Box { size } => {
            std::fs::create_dir_all(folder.join("meshes"))?;
            let file = format!("meshes/{name}.obj");
            std::fs::write(folder.join(&file), box_obj(*size))?;
            (file, 1.0)
        }
        Geometry::Mesh { path, scale } => (path.trim().to_owned(), *scale),
    };
    let geometry = format!(r#"<geometry><mesh filename="{mesh}" scale="{scale} {scale} {scale}"/></geometry>"#);
    let urdf = format!(
        r#"<?xml version="1.0"?>
<!-- {FIXTURE_FORMAT}: written by Factory OS from product.json (Product & Task); edit the product, not this file. -->
<robot name="{name}">
  <link name="World">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      {geometry}
      <material name="product"><color rgba="0.60 0.64 0.70 1.0"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      {geometry}
    </collision>
  </link>
</robot>
"#
    );
    let path = folder.join(format!("{name}.urdf"));
    std::fs::write(&path, urdf).map_err(|err| anyhow::anyhow!("{}: {err}", path.display()))?;
    Ok(path)
}

/// A URDF product as a Robo Fitter fixture: its URDF (`path`, relative to `folder` or absolute)
/// written to `<folder>/<name>.urdf` with every mesh resolved from `folder`.
fn write_urdf_fixture(folder: &Path, name: &str, path: &str) -> anyhow::Result<PathBuf> {
    let source = {
        let path = Path::new(path.trim());
        if path.is_absolute() { path.to_owned() } else { folder.join(path) }
    };
    let out = folder.join(format!("{name}.urdf"));
    anyhow::ensure!(source != out, "{}: the product's URDF goes in its {URDF_DIR}/ folder", source.display());
    let mut robot = urdf_robo_dresser::parse_urdf(&source).map_err(|err| anyhow::anyhow!("{}: {err}", source.display()))?;
    let source_dir = source.parent().unwrap_or(folder);
    let search = folder.join(URDF_DIR);
    for link in &mut robot.links {
        let geometries = link.visuals.iter_mut().map(|v| &mut v.geometry).chain(link.collisions.iter_mut().map(|c| &mut c.geometry));
        for geometry in geometries {
            if let urdf_robo_dresser::Geometry::Mesh { filename, .. } = geometry {
                let file = resolve_mesh(filename, source_dir, &search)
                    .ok_or_else(|| anyhow::anyhow!("{}: mesh not found: {filename}", source.display()))?;
                *filename = file.strip_prefix(folder).map_or_else(|_| file.to_string_lossy().into_owned(), |rel| rel.to_string_lossy().replace('\\', "/"));
            }
        }
    }
    robot.name = name.to_owned();
    let urdf = String::from_utf8(urdf_robo_dresser::serialize_urdf(&robot))?;
    let comment = format!("<!-- {FIXTURE_FORMAT}: written by Factory OS from product.json and {} (Product & Task); edit those, not this file. -->\n", path.trim());
    let urdf = match urdf.split_once('\n') {
        Some((header, rest)) if header.starts_with("<?xml") => format!("{header}\n{comment}{rest}"),
        _ => format!("{comment}{urdf}"),
    };
    std::fs::write(&out, urdf).map_err(|err| anyhow::anyhow!("{}: {err}", out.display()))?;
    Ok(out)
}

/// A URDF's mesh `filename`: relative to the URDF's folder, absolute, `file://`, or
/// `package://<package>/<path>`, looked for in a folder `<package>` under `search` (or the URDF's
/// folder and `search` themselves).
pub(crate) fn resolve_mesh(filename: &str, urdf_dir: &Path, search: &Path) -> Option<PathBuf> {
    if let Some(rest) = filename.strip_prefix("package://") {
        let (package, rel) = rest.split_once('/')?;
        let mut candidates = vec![
            urdf_dir.join(package).join(rel),
            search.join(package).join(rel),
            urdf_dir.join(rel),
            search.join(rel),
        ];
        candidates.extend(urdf_dir.ancestors().take_while(|dir| dir.starts_with(search)).filter(|dir| dir.ends_with(package)).map(|dir| dir.join(rel)));
        return candidates.into_iter().find(|path| path.is_file());
    }
    let path = Path::new(filename.strip_prefix("file://").unwrap_or(filename));
    let path = if path.is_absolute() { path.to_owned() } else { urdf_dir.join(path) };
    // Without `./` parts: `urdf/./meshes/a.obj` is `urdf/meshes/a.obj`.
    let path: PathBuf = path.components().filter(|c| !matches!(c, std::path::Component::CurDir)).collect();
    path.is_file().then_some(path)
}

/// A box of `size` standing on the origin, as OBJ: 8 vertices, 12 outward-facing triangles, and a
/// normal per face, so viewers shade each face flat and its edges show.
pub(crate) fn box_obj([sx, sy, sz]: [f32; 3]) -> String {
    let (x, y) = (sx / 2.0, sy / 2.0);
    let mut obj = String::from("# Factory OS product box\n");
    for (vx, vy, vz) in [(-x, -y, 0.0), (x, -y, 0.0), (x, y, 0.0), (-x, y, 0.0), (-x, -y, sz), (x, -y, sz), (x, y, sz), (-x, y, sz)] {
        obj.push_str(&format!("v {vx} {vy} {vz}\n"));
    }
    // Faces: bottom, top, −y, +x, +y, −x; two triangles each.
    let faces: [([i32; 3], [[u8; 3]; 2]); 6] = [
        ([0, 0, -1], [[1, 3, 2], [1, 4, 3]]),
        ([0, 0, 1], [[5, 6, 7], [5, 7, 8]]),
        ([0, -1, 0], [[1, 2, 6], [1, 6, 5]]),
        ([1, 0, 0], [[2, 3, 7], [2, 7, 6]]),
        ([0, 1, 0], [[3, 4, 8], [3, 8, 7]]),
        ([-1, 0, 0], [[4, 1, 5], [4, 5, 8]]),
    ];
    for ([nx, ny, nz], _) in &faces {
        obj.push_str(&format!("vn {nx} {ny} {nz}\n"));
    }
    for (n, (_, triangles)) in faces.iter().enumerate() {
        for [a, b, c] in triangles {
            let n = n + 1;
            obj.push_str(&format!("f {a}//{n} {b}//{n} {c}//{n}\n"));
        }
    }
    obj
}

/// A product's part, resolved for the scene.
enum Part {
    /// A box: drawn from the product's size.
    Box,
    Mesh(MeshAsset),
    /// No part to draw (yet): no URDF chosen, or a URDF, which [`model_node`] draws once loaded.
    Pending,
}

fn view() -> ViewHints {
    ViewHints { title: VIEW_TITLE.to_owned() }
}

fn rgba(color: u32) -> Rgba {
    color.to_be_bytes()
}

fn f64s(v: [f32; 3]) -> [f64; 3] {
    v.map(f64::from)
}

/// The part's node, [`PART_NODE`]: a solid box standing on the origin, labelled with the product's
/// name, or a mesh in the part's colour. (A URDF's is [`model_node`].)
fn part_node(product: &Product, part: &Part) -> Option<Node> {
    let kind = match (part, &product.geometry) {
        (Part::Box, Geometry::Box { size }) => {
            let size = size.map(|s| f64::from(s.max(0.001)));
            NodeKind::Boxes {
                centers: vec![[0.0, 0.0, size[2] / 2.0]],
                half_sizes: vec![size.map(|s| s / 2.0)],
                quats_xyzw: Vec::new(),
                colors: vec![rgba(PART_COLOR)],
                labels: vec![product.name.clone()],
                solid: true,
                show_labels: None,
            }
        }
        (Part::Mesh(mesh), Geometry::Mesh { scale, .. }) => {
            NodeKind::Mesh { mesh: mesh.clone(), scale: [f64::from(*scale); 3], color: Some(rgba(PART_COLOR)) }
        }
        _ => return None,
    };
    Some(Node::new(PART_NODE, kind))
}

/// A URDF part: as described, every joint at 0, its root link at the origin.
fn model_node(model: ModelRef) -> Node {
    Node::new(PART_NODE, NodeKind::Model { model, joints: Default::default(), show_collision: false }).at(Pose3::IDENTITY)
}

/// Each task under `tasks/<index>_<name>`, in its own colour: point operations as labelled points
/// (`…/points`), paths as labelled lines (`…/paths`), and every normal as an arrow
/// (`…/normals`). Only the non-empty ones.
fn task_nodes(tasks: &[Task], part_size: f32) -> Vec<Node> {
    let radius = (part_size * 0.015).clamp(0.003, 0.03);
    let arrow = (part_size * 0.12).clamp(0.02, 0.2);
    let mut nodes = Vec::new();
    for (index, task) in tasks.iter().enumerate() {
        let label = |id: &str| if task.name.is_empty() { id.to_owned() } else { format!("{}/{id}", task.name) };
        let color = vec![rgba(TASK_COLORS[index % TASK_COLORS.len()])];
        let (mut points, mut labels, mut paths, mut path_labels, mut origins, mut normals) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new());
        for op in &task.operations {
            match &op.target {
                Target::Point { position, .. } => {
                    points.push(f64s(*position));
                    labels.push(label(&op.id));
                    origins.push(f64s(*position));
                }
                Target::Path { points: path, .. } => {
                    let Some(first) = path.first() else { continue };
                    paths.push(path.iter().copied().map(f64s).collect());
                    path_labels.push(label(&op.id));
                    origins.push(f64s(*first));
                }
            }
            normals.push(f64s(normalized(op.target.normal()).map(|n| n * arrow)));
        }
        let id = format!("tasks/{index}_{}", entity_name(&task.name));
        if !points.is_empty() {
            nodes.push(Node::new(
                format!("{id}/points"),
                NodeKind::Points { points, colors: color.clone(), labels, radius: Some(radius), show_labels: None },
            ));
        }
        if !paths.is_empty() {
            nodes.push(Node::new(
                format!("{id}/paths"),
                NodeKind::Lines { strips: paths, colors: color.clone(), radius: Some(radius * 0.6), labels: path_labels, show_labels: None },
            ));
        }
        if !normals.is_empty() {
            nodes.push(Node::new(
                format!("{id}/normals"),
                NodeKind::Arrows { origins, vectors: normals, colors: color, labels: Vec::new(), radius: None, show_labels: None },
            ));
        }
    }
    nodes
}

/// `name` usable as one node id part.
fn entity_name(name: &str) -> String {
    let name: String = name.chars().map(|c| if c.is_ascii_alphanumeric() || c == '_' { c } else { '_' }).collect();
    if name.is_empty() { "task".to_owned() } else { name }
}

fn normalized(v: [f32; 3]) -> [f32; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-6 { [0.0, 0.0, 1.0] } else { v.map(|c| c / len) }
}

// --- HTTP ---

pub fn router() -> Router<AppState> {
    Router::new()
        .route(api::PRODUCT_LIST, get(list))
        .route(api::PRODUCT_LOAD, post(load))
        .route(api::PRODUCT_SAVE, post(save))
        .route(api::PRODUCT_PREVIEW, post(preview))
        .route(api::PRODUCT_CATALOGUE, get(catalogue))
        .route(api::PRODUCT_FIXTURES, get(fixtures))
}

/// The routes that write uploaded files or to the catalogue: admitted as the workspace files'
/// uploads are, and merged outside the CORS layer (see `main.rs`).
pub fn transfer_router() -> Router<AppState> {
    Router::new()
        .route(api::PRODUCT_CATALOGUE_LOAD, post(catalogue_load))
        .route(api::PRODUCT_CATALOGUE_PUBLISH, post(catalogue_publish))
        .route(api::PRODUCT_URDF_FROM_CATALOGUE, post(urdf_from_catalogue))
        .route(api::PRODUCT_URDF_UPLOAD, put(upload_urdf).layer(DefaultBodyLimit::max(MAX_TRANSFER_BYTES as usize)))
}

type Reply<T> = Result<Json<T>, (StatusCode, String)>;

fn bad_request(err: anyhow::Error) -> (StatusCode, String) {
    let status = if err.downcast_ref::<Exists>().is_some() { StatusCode::CONFLICT } else { StatusCode::BAD_REQUEST };
    (status, format!("{err:#}"))
}

async fn blocking<T: Send + 'static>(work: impl FnOnce() -> anyhow::Result<T> + Send + 'static) -> Reply<T> {
    tokio::task::spawn_blocking(work).await.map_err(|err| bad_request(err.into()))?.map(Json).map_err(bad_request)
}

async fn catalogue(State(state): State<AppState>) -> Json<CatalogueProducts> {
    let service = state.product.clone();
    Json(tokio::task::spawn_blocking(move || service.catalogue_list()).await.unwrap_or_default())
}

async fn fixtures(State(state): State<AppState>) -> Json<Vec<String>> {
    Json(state.product.fixtures())
}

async fn catalogue_load(
    State(state): State<AppState>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Json(request): Json<CatalogueTransfer>,
) -> Reply<ProductFile> {
    super::files::admit_transfer(&state.files, peer, &headers)?;
    let service = state.product.clone();
    let loaded = blocking(move || service.catalogue_load(&request)).await?;
    after_change(&state, &loaded.product.name).await;
    write_through(&state, &loaded.product.name);
    Ok(loaded)
}

async fn catalogue_publish(
    State(state): State<AppState>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Json(request): Json<CatalogueTransfer>,
) -> Reply<CatalogueProducts> {
    super::files::admit_transfer(&state.files, peer, &headers)?;
    let service = state.product.clone();
    let name = request.name.clone();
    let published = blocking(move || service.catalogue_publish(&request)).await?;
    // The viewers list the catalogue again (locally it is the shared one; in S3 mode the user's additions).
    state.bus.put_json(keys::CATALOGUE_CHANGED, &factoryos_protocol::storage::CatalogueChanged::default());
    // Into the user's own catalogue additions (`users/<user>/catalogue/…`), never the shared one.
    state.storage.spawn_save_tree(state.product.catalogue_dir().join(name));
    Ok(published)
}

async fn urdf_from_catalogue(
    State(state): State<AppState>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Json(request): Json<UrdfFromCatalogue>,
) -> Reply<ProductUrdfs> {
    super::files::admit_transfer(&state.files, peer, &headers)?;
    let service = state.product.clone();
    let name = request.product.clone();
    let urdfs = blocking(move || service.urdf_from_catalogue(&request)).await?;
    write_through(&state, &name);
    Ok(urdfs)
}

async fn upload_urdf(
    State(state): State<AppState>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    UrlPath(name): UrlPath<String>,
    body: Bytes,
) -> Reply<ProductUrdfs> {
    super::files::admit_transfer(&state.files, peer, &headers)?;
    let service = state.product.clone();
    let product = name.clone();
    let urdfs = blocking(move || service.upload_urdf(&name, &body)).await?;
    write_through(&state, &product);
    Ok(urdfs)
}

/// Product `name`'s folder to the store (S3 mode; locally the folder is the store).
pub(crate) fn write_through(state: &AppState, name: &str) {
    if valid_name(name) {
        state.storage.spawn_save_tree(state.product.dir().join(name));
    }
}

/// After product `name` changed on disk: the Robo Fitter can place it, and the cells using it get
/// its copy and their task programs derived from it again.
pub(crate) async fn after_change(state: &AppState, name: &str) {
    state.fitter.refresh_palette();
    let (config, name) = (state.config.clone(), name.to_owned());
    let lines = tokio::task::spawn_blocking(move || {
        super::cells::cells_using_product(&config, &name)
            .into_iter()
            .filter_map(|root| super::cells::derive_tasks(&config, &root))
            .collect::<Vec<_>>()
    })
    .await
    .unwrap_or_default();
    for line in lines {
        state.bus.log(line);
    }
}

async fn list(State(state): State<AppState>) -> Json<ProductsInfo> {
    Json(state.product.list())
}

async fn load(State(state): State<AppState>, Json(request): Json<LoadProduct>) -> Reply<ProductFile> {
    state.product.load(&request.name).map(Json).map_err(bad_request)
}

async fn save(State(state): State<AppState>, Json(product): Json<Product>) -> Reply<ProductFile> {
    let saved = state.product.save(product).map_err(bad_request)?;
    after_change(&state, &saved.product.name).await;
    write_through(&state, &saved.product.name);
    Ok(Json(saved))
}

async fn preview(State(state): State<AppState>, Json(request): Json<PreviewProduct>) -> Reply<()> {
    let service = state.product.clone();
    tokio::task::spawn_blocking(move || service.preview(&request.product))
        .await
        .map_err(|err| bad_request(err.into()))?
        .map(Json)
        .map_err(bad_request)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_product_fixture() {
        let dir = std::env::temp_dir().join(format!("factoryos-product-{}", std::process::id()));
        let folder = dir.join("cube");
        std::fs::create_dir_all(&folder).unwrap();
        let product = Product {
            name: "cube".to_owned(),
            ..Product::default()
        };
        std::fs::write(folder.join(PRODUCT_FILE), serde_json::to_string(&product).unwrap()).unwrap();
        assert_eq!(placeable_products(&dir), ["cube"]);
        let urdf = std::fs::read_to_string(folder.join("cube.urdf")).unwrap();
        assert!(urdf.contains(r#"<mesh filename="meshes/cube.obj""#), "{urdf}");
        let obj = std::fs::read_to_string(folder.join("meshes/cube.obj")).unwrap();
        assert_eq!(obj.lines().filter(|l| l.starts_with("v ")).count(), 8);
        assert_eq!(obj.lines().filter(|l| l.starts_with("f ")).count(), 12);
        assert_eq!(obj.lines().filter(|l| l.starts_with("vn ")).count(), 6);
        assert!(urdf.contains(FIXTURE_FORMAT));
        std::fs::remove_dir_all(&dir).ok();
    }

    fn temp(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("factoryos-product-{name}-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn zip_of(files: &[(&str, &[u8])]) -> Vec<u8> {
        use std::io::Write as _;
        let mut zip = zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
        for (name, bytes) in files {
            zip.start_file(*name, zip::write::SimpleFileOptions::default()).unwrap();
            zip.write_all(bytes).unwrap();
        }
        zip.finish().unwrap().into_inner()
    }

    const PART_URDF: &str = r#"<?xml version="1.0"?>
<robot name="part">
  <link name="base">
    <visual><geometry><mesh filename="./meshes/part.obj"/></geometry></visual>
    <collision><geometry><mesh filename="package://part_pkg/meshes/part.obj"/></geometry></collision>
  </link>
</robot>
"#;

    #[test]
    fn zip_extraction_stays_inside() {
        let dir = temp("zip");
        let zip = zip_of(&[("pkg/part.urdf", PART_URDF.as_bytes()), ("pkg/meshes/part.obj", b"v 0 0 0\n")]);
        extract_zip(&zip, &dir.join("ok"), 1024).unwrap();
        assert_eq!(urdfs_in(&dir.join("ok"), Path::new("urdf")), ["urdf/pkg/part.urdf"]);
        let evil = zip_of(&[("../escaped.txt", b"x")]);
        assert!(extract_zip(&evil, &dir.join("evil"), 1024).is_err());
        assert!(!dir.join("escaped.txt").exists());
        let big = zip_of(&[("a.bin", &[0u8; 2048])]);
        let err = extract_zip(&big, &dir.join("big"), 1024).unwrap_err();
        assert!(err.to_string().contains("more than"), "{err}");
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn urdf_product_fixture() {
        let dir = temp("urdf");
        let folder = dir.join("door");
        std::fs::create_dir_all(folder.join("urdf/part_pkg/meshes")).unwrap();
        std::fs::write(folder.join("urdf/part_pkg/part.urdf"), PART_URDF).unwrap();
        std::fs::write(folder.join("urdf/part_pkg/meshes/part.obj"), "v 0 0 0\n").unwrap();
        let product = Product {
            name: "door".to_owned(),
            geometry: Geometry::Urdf { path: "urdf/part_pkg/part.urdf".to_owned() },
            ..Product::default()
        };
        std::fs::write(folder.join(PRODUCT_FILE), serde_json::to_string(&product).unwrap()).unwrap();
        assert_eq!(placeable_products(&dir), ["door"]);
        let urdf = std::fs::read_to_string(folder.join("door.urdf")).unwrap();
        assert!(urdf.contains(FIXTURE_FORMAT), "{urdf}");
        assert_eq!(urdf.matches(r#"filename="urdf/part_pkg/meshes/part.obj""#).count(), 2, "{urdf}");
        assert!(urdf.contains(r#"<robot name="door">"#), "{urdf}");
        // A mesh that is not there is an error, not a broken fixture.
        std::fs::remove_file(folder.join("urdf/part_pkg/meshes/part.obj")).unwrap();
        assert!(write_fixture(&folder, &product).is_err());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn catalogue_copies_replace_only_when_asked() {
        let dir = temp("catalogue");
        let product = Product {
            name: "box".to_owned(),
            tasks: vec![Task { name: "welds".to_owned(), process: "spotWeld".to_owned(), ..Task::default() }],
            ..Product::default()
        };
        let working = dir.join("products/box");
        std::fs::create_dir_all(working.join("urdf")).unwrap();
        std::fs::write(working.join(PRODUCT_FILE), serde_json::to_string(&product).unwrap()).unwrap();
        std::fs::write(working.join("urdf/a.urdf"), "x").unwrap();
        let published = dir.join("catalogue/product-collection/box");
        replace_dir(&working, &published, false).unwrap();
        assert!(published.join("urdf/a.urdf").is_file());
        let err = replace_dir(&working, &published, false).unwrap_err();
        assert!(err.downcast_ref::<Exists>().is_some());
        assert_eq!(bad_request(err).0, StatusCode::CONFLICT);
        // Loading it again in a next run: the tasks come back.
        std::fs::remove_dir_all(&working).unwrap();
        replace_dir(&published, &working, false).unwrap();
        let loaded: Product = serde_json::from_str(&std::fs::read_to_string(working.join(PRODUCT_FILE)).unwrap()).unwrap();
        assert_eq!(loaded, product);
        std::fs::write(working.join("extra.txt"), "x").unwrap();
        replace_dir(&published, &working, true).unwrap();
        assert!(!working.join("extra.txt").exists(), "replaced, not merged");
        assert!(std::fs::read_dir(dir.join("products")).unwrap().all(|e| !e.unwrap().file_name().to_string_lossy().contains("fos-staging")));
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn normals_get_normalized() {
        assert_eq!(normalized([0.0, 0.0, 2.0]), [0.0, 0.0, 1.0]);
        assert_eq!(normalized([0.0, 0.0, 0.0]), [0.0, 0.0, 1.0]);
    }

    fn op(id: &str, target: Target) -> factoryos_protocol::product::Operation {
        factoryos_protocol::product::Operation { id: id.to_owned(), target, group: String::new(), params: Default::default() }
    }

    fn welded_box() -> Product {
        Product {
            name: "cube".to_owned(),
            geometry: Geometry::Box { size: [0.4, 0.3, 0.2] },
            tasks: vec![
                Task {
                    name: "spot welds".to_owned(),
                    operations: vec![
                        op("W01", Target::Point { position: [0.0, 0.0, 0.2], normal: [0.0, 0.0, 2.0] }),
                        op("S01", Target::Path { points: vec![[-0.1, 0.0, 0.2], [0.1, 0.0, 0.2]], normal: [1.0, 0.0, 0.0] }),
                        op("empty", Target::Path { points: vec![], normal: [1.0, 0.0, 0.0] }),
                    ],
                    ..Task::default()
                },
                Task { name: "".to_owned(), operations: vec![op("P", Target::Point { position: [0.0; 3], normal: [0.0; 3] })], ..Task::default() },
            ],
            ..Product::default()
        }
    }

    #[test]
    fn box_part_stands_on_the_origin() {
        let product = welded_box();
        let part = part_node(&product, &Part::Box).unwrap();
        assert_eq!(part.id, PART_NODE);
        assert_eq!(part.pose, Pose3::IDENTITY);
        let NodeKind::Boxes { centers, half_sizes, colors, labels, solid, .. } = part.kind else { panic!("{part:?}") };
        let close = |a: [f64; 3], b: [f64; 3]| a.iter().zip(b).all(|(a, b)| (a - b).abs() < 1e-6);
        assert!(close(centers[0], [0.0, 0.0, 0.1]) && close(half_sizes[0], [0.2, 0.15, 0.1]), "{centers:?} {half_sizes:?}");
        assert_eq!((colors, labels, solid), (vec![[0x9a, 0xa4, 0xb2, 0xff]], vec!["cube".to_owned()], true));
        assert_eq!(part_node(&product, &Part::Pending), None);
    }

    #[test]
    fn mesh_and_urdf_parts() {
        let mesh = MeshAsset::default();
        let product = Product { geometry: Geometry::Mesh { path: "m.obj".to_owned(), scale: 0.5 }, ..Product::default() };
        let node = part_node(&product, &Part::Mesh(mesh.clone())).unwrap();
        assert_eq!(node.kind, NodeKind::Mesh { mesh, scale: [0.5; 3], color: Some(rgba(PART_COLOR)) });
        let model = ModelRef::default();
        let node = model_node(model.clone());
        assert_eq!((node.id.as_str(), node.parent.as_ref()), (PART_NODE, None));
        assert_eq!(node.kind, NodeKind::Model { model, joints: Default::default(), show_collision: false });
    }

    #[test]
    fn tasks_become_points_lines_and_arrows() {
        let nodes = task_nodes(&welded_box().tasks, 0.4);
        let ids: Vec<_> = nodes.iter().map(|n| n.id.as_str()).collect();
        assert_eq!(ids, ["tasks/0_spot_welds/points", "tasks/0_spot_welds/paths", "tasks/0_spot_welds/normals", "tasks/1_task/points", "tasks/1_task/normals"]);
        let NodeKind::Points { points, colors, labels, radius, .. } = &nodes[0].kind else { panic!() };
        assert_eq!(points.len(), 1);
        assert_eq!(labels, &["spot welds/W01"]);
        assert_eq!(colors, &[rgba(TASK_COLORS[0])]);
        assert!((radius.unwrap() - 0.006).abs() < 1e-6);
        let NodeKind::Lines { strips, radius: line_radius, labels: path_labels, .. } = &nodes[1].kind else { panic!() };
        assert_eq!(strips.len(), 1, "the empty path is left out");
        assert_eq!(path_labels, &["spot welds/S01"], "one label per drawn path");
        assert!((line_radius.unwrap() - 0.0036).abs() < 1e-6);
        let NodeKind::Arrows { origins, vectors, .. } = &nodes[2].kind else { panic!() };
        // One normal per drawn operation, at its point or its path's start.
        assert_eq!((origins.len(), vectors.len()), (2, 2));
        assert!((vectors[0][2] - 0.048).abs() < 1e-6 && (vectors[1][0] - 0.048).abs() < 1e-6, "{vectors:?}");
        let NodeKind::Points { labels, colors, .. } = &nodes[3].kind else { panic!() };
        assert_eq!((labels.as_slice(), colors.as_slice()), (["P".to_owned()].as_slice(), [rgba(TASK_COLORS[1])].as_slice()));
    }
}
