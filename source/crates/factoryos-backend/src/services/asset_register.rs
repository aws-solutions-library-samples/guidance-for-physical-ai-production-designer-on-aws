//! `factoryos/assets/register` ([`factoryos_protocol::keys::ASSETS_REGISTER`]): lets a producer on
//! this machine that is not the backend (fos-live-scene, the live cell) make its scene's files
//! fetchable by `GET` [`api::ASSET`].
//!
//! The producer builds its [`factoryos_protocol::scene::SceneDoc`] with its own hashes and asks
//! here, by absolute path, for the files to be [`AssetIndex::register`]ed. The index lives in this
//! process's memory, so the producer asks again from time to time (cheap: unchanged files are not
//! re-hashed) and after a backend restart.
//!
//! The queryable is on the backend's Zenoh, which browsers reach through `/zenoh` (behind the edge's
//! login like `/api/v1`). So it only registers what a scene could legitimately reference: regular
//! files whose canonical path (symlinks resolved) lies under one of the backend's own data roots
//! ([`allowed_roots`]: catalogue, asset root, products, cells, the release's `assets/` and `cells/`)
//! and whose extension is a URDF, mesh, material or texture ([`ALLOWED_EXTENSIONS`]). Anything else
//! (`/etc/shadow`, a `../` escape, a link out of the roots, a cell's JSON or YAML) is refused.
//!
//! Request: a JSON array of absolute paths (at most [`MAX_PATHS`]). Reply: a JSON array of the same
//! length of [`Registered`], each `{"asset": {"asset": AssetRef, "glb"?: AssetRef, "lod"?: AssetRef}}`
//! or `{"error": "…"}`. `glb`/`lod` are an OBJ's or STL's converted variants
//! ([`AssetIndex::mesh_asset`]); a path's sidecars (MTLs, textures) are registered with it but
//! not listed, the producer asks for them by path too.
//!
//! [`api::ASSET`]: factoryos_protocol::api::ASSET

use std::path::{Path, PathBuf};
use std::sync::Arc;

use factoryos_protocol::keys::ASSETS_REGISTER as KEY;
use factoryos_protocol::scene::Registered;

use crate::bus::{Bus, reply_json};
use crate::config_store::ConfigStore;
use crate::paths;
use crate::services::assets::AssetIndex;
use crate::services::mesh_convert;

/// Paths per request, at most.
pub const MAX_PATHS: usize = 4096;
/// Request size, at most.
const MAX_REQUEST_BYTES: usize = 4 << 20;

pub use crate::services::assets::ALLOWED_EXTENSIONS;

/// The directories whose files may be registered: the backend's own data, canonical.
pub fn allowed_roots(config: &ConfigStore) -> Vec<PathBuf> {
    let c = config.get();
    let mut roots: Vec<PathBuf> = [&c.catalogue, &c.asset_root, &c.cells_dir, &c.cell_dir, &c.env_dir, &c.fitted_robots_dir]
        .into_iter()
        .filter(|value| !value.trim().is_empty())
        .map(|value| PathBuf::from(config.resolve(value)))
        .collect();
    roots.push(crate::services::product::products_dir(config));
    roots.extend(crate::services::cells::cells_folder(config));
    roots.push(PathBuf::from(paths::default_catalogue()));
    roots.push(PathBuf::from(paths::default_assets()));
    roots.push(paths::home().join("assets"));
    roots.push(paths::home().join("cells"));
    canonical_roots(roots)
}

/// Canonical, existing, without duplicates; never `/`.
fn canonical_roots(roots: Vec<PathBuf>) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = Vec::new();
    for root in roots {
        let Ok(root) = root.canonicalize() else { continue };
        if root.parent().is_some() && root.is_dir() && !out.contains(&root) {
            out.push(root);
        }
    }
    out
}

/// The canonical file `path` names, if it may be registered.
pub fn admit(path: &str, roots: &[PathBuf]) -> Result<PathBuf, String> {
    if path.contains('\0') || !Path::new(path).is_absolute() {
        return Err("not an absolute path".into());
    }
    let canonical = Path::new(path).canonicalize().map_err(|_| "not found".to_owned())?;
    let extension = canonical.extension().and_then(|e| e.to_str()).unwrap_or_default().to_ascii_lowercase();
    if !ALLOWED_EXTENSIONS.contains(&extension.as_str()) {
        return Err("not a URDF, mesh, material or texture".into());
    }
    if !roots.iter().any(|root| canonical.starts_with(root)) {
        return Err("outside the backend's asset roots".into());
    }
    if !canonical.is_file() {
        return Err("not a file".into());
    }
    Ok(canonical)
}

/// Registers what [`admit`] lets through, meshes with their GLB and LOD. Errors don't say more
/// than the reason, never contents.
pub fn register(index: &AssetIndex, paths: &[String], roots: &[PathBuf]) -> Vec<Registered> {
    paths.iter().map(|path| admit(path, roots).and_then(|file| register_file(index, &file)).unwrap_or_else(Registered::Error)).collect()
}

fn register_file(index: &AssetIndex, file: &Path) -> Result<Registered, String> {
    if mesh_convert::Source::of(file).is_some() {
        let mesh = index.mesh_asset(file).map_err(|_| "unreadable".to_owned())?;
        return Ok(Registered::Asset { asset: mesh.asset, glb: mesh.glb, lod: mesh.lod });
    }
    let asset = index.register(file).map_err(|_| "unreadable".to_owned())?;
    Ok(Registered::Asset { asset, glb: None, lod: None })
}

/// Parses a request: a JSON array of strings, within the limits.
pub fn parse_request(payload: &[u8]) -> Result<Vec<String>, String> {
    if payload.len() > MAX_REQUEST_BYTES {
        return Err("request too large".into());
    }
    let paths: Vec<String> = serde_json::from_slice(payload).map_err(|err| format!("expected a JSON array of paths: {err}"))?;
    if paths.len() > MAX_PATHS {
        return Err(format!("at most {MAX_PATHS} paths per request"));
    }
    Ok(paths)
}

/// Answers [`KEY`] until the bus closes.
pub fn serve(bus: &Bus, config: Arc<ConfigStore>, index: Arc<AssetIndex>) {
    let session = bus.session().clone();
    tokio::spawn(async move {
        let queryable = match session.declare_queryable(KEY).await {
            Ok(queryable) => queryable,
            Err(err) => {
                tracing::error!("{KEY}: {err}");
                return;
            }
        };
        while let Ok(query) = queryable.recv_async().await {
            let payload = query.payload().map(|p| p.to_bytes().into_owned()).unwrap_or_default();
            let paths = match parse_request(&payload) {
                Ok(paths) => paths,
                Err(err) => {
                    reply_json(&query, &serde_json::json!({ "error": err })).await;
                    continue;
                }
            };
            let (config, index) = (config.clone(), index.clone());
            let result = tokio::task::spawn_blocking(move || register(&index, &paths, &allowed_roots(&config))).await;
            match result {
                Ok(registered) => {
                    let refused = registered.iter().filter(|r| matches!(r, Registered::Error(_))).count();
                    if refused > 0 {
                        tracing::debug!("{KEY}: {refused} of {} paths refused", registered.len());
                    }
                    reply_json(&query, &registered).await;
                }
                Err(err) => tracing::warn!("{KEY}: {err}"),
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_dir(tag: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("fos-asset-register-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn registers_assets_under_the_roots_only() {
        let dir = temp_dir("roots");
        let root = dir.join("cells/c1");
        std::fs::create_dir_all(root.join("meshes")).unwrap();
        std::fs::write(root.join("robot.urdf"), "<robot name='r'/>").unwrap();
        std::fs::write(root.join("meshes/a.obj"), "v 0 0 0\n").unwrap();
        std::fs::write(root.join("scene.yaml"), "name: x\n").unwrap();
        std::fs::write(dir.join("secret.obj"), "outside").unwrap();
        #[cfg(unix)]
        std::os::unix::fs::symlink(dir.join("secret.obj"), root.join("meshes/escape.obj")).unwrap();
        let roots = canonical_roots(vec![root.clone(), PathBuf::from("/"), dir.join("missing")]);
        assert_eq!(roots, vec![root.canonicalize().unwrap()]);

        let index = AssetIndex::unrestricted();
        let s = |p: PathBuf| p.to_string_lossy().into_owned();
        let paths = vec![
            s(root.join("robot.urdf")),
            s(root.join("meshes/../meshes/a.obj")),
            s(root.join("scene.yaml")),
            s(root.join("../../secret.obj")),
            s(root.join("meshes/escape.obj")),
            "/etc/shadow".into(),
            "/etc/passwd".into(),
            "meshes/a.obj".into(),
            s(root.join("missing.obj")),
            s(root.join("meshes")),
        ];
        let out = register(&index, &paths, &roots);
        let Registered::Asset { asset: urdf, glb: None, lod: None } = &out[0] else { panic!("{out:?}") };
        assert_eq!(urdf.media_type, "application/xml");
        assert!(index.contains(&urdf.sha256));
        let Registered::Asset { asset: obj, .. } = &out[1] else { panic!("{out:?}") };
        assert_eq!((obj.size, obj.media_type.as_str()), (8, "model/obj"));
        for (i, r) in out.iter().enumerate().skip(2) {
            if cfg!(unix) || i != 4 {
                assert!(matches!(r, Registered::Error(_)), "{}: {r:?}", paths[i]);
            }
        }
        let json = serde_json::to_string(&out[..3]).unwrap();
        assert!(json.starts_with(r#"[{"asset":{"asset":{"sha256":""#) && json.contains(r#"{"error":"#), "{json}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn meshes_come_with_their_glb_and_lod() {
        let dir = temp_dir("variants");
        let root = dir.join("cell");
        std::fs::create_dir_all(&root).unwrap();
        // A UV sphere (2 × 40 × 60 triangles): big enough for a LOD.
        let (rings, segments) = (40, 60);
        let mut obj = String::from("mtllib m.mtl\nusemtl Red\n");
        for r in 0..=rings {
            let theta = std::f32::consts::PI * r as f32 / rings as f32;
            for s in 0..segments {
                let phi = std::f32::consts::TAU * s as f32 / segments as f32;
                obj += &format!("v {} {} {}\n", theta.sin() * phi.cos(), theta.sin() * phi.sin(), theta.cos());
            }
        }
        for r in 0..rings {
            for s in 0..segments {
                let (a, b) = (r * segments + s + 1, r * segments + (s + 1) % segments + 1);
                obj += &format!("f {a} {b} {}\nf {a} {} {}\n", b + segments, b + segments, a + segments);
            }
        }
        std::fs::write(root.join("sphere.obj"), obj).unwrap();
        std::fs::write(root.join("m.mtl"), "newmtl Red\nKd 1 0 0\n").unwrap();
        std::fs::write(root.join("tiny.obj"), "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n").unwrap();
        let roots = canonical_roots(vec![root.clone()]);
        let index = AssetIndex::unrestricted_in(&dir.join("state"));
        let s = |p: PathBuf| p.to_string_lossy().into_owned();
        let paths = [s(root.join("sphere.obj")), s(root.join("tiny.obj")), s(root.join("m.mtl"))];
        let out = register(&index, &paths, &roots);
        let Registered::Asset { asset, glb: Some(glb), lod: Some(lod) } = &out[0] else { panic!("{out:?}") };
        assert_eq!(asset.media_type, "model/obj");
        assert_eq!((glb.media_type.as_str(), lod.media_type.as_str()), ("model/gltf-binary", "model/gltf-binary"));
        assert!(lod.size < glb.size);
        assert!(index.read(&glb.sha256).is_some() && index.read(&lod.sha256).is_some(), "served");
        let Registered::Asset { glb: Some(_), lod: None, .. } = &out[1] else { panic!("a triangle is its own LOD: {out:?}") };
        let Registered::Asset { glb: None, lod: None, .. } = &out[2] else { panic!("{out:?}") };
        let json = serde_json::to_value(&out[0]).unwrap();
        assert_eq!(json["asset"]["glb"]["mediaType"], "model/gltf-binary", "{json}");
        assert_eq!(json["asset"]["asset"]["sha256"], asset.sha256.as_str());
        // Again: cache hits, the same answer.
        assert_eq!(index.conversions(), 2);
        assert_eq!(register(&index, &paths, &roots), out);
        assert_eq!(index.conversions(), 2);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn request_limits() {
        assert_eq!(parse_request(br#"["/a.obj"]"#).unwrap(), vec!["/a.obj".to_owned()]);
        assert!(parse_request(br#"{"paths": []}"#).is_err());
        assert!(parse_request(br#"[1]"#).is_err());
        let many = serde_json::to_vec(&vec!["/a"; MAX_PATHS + 1]).unwrap();
        assert!(parse_request(&many).is_err());
    }
}
