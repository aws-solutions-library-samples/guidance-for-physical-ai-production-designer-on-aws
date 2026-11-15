//! The Robo Fitter's scenes on disk: every scene is a folder `<scenes>/<name>/scene.yaml`
//! (`<scenes>` = `<assetRoot>/scenes/`, [`factoryos_protocol::fitter::SCENES_DIR`]), and a scene is
//! referred to everywhere (the Robo Fitter, cells' `factoryos-cell.json` `fitted_scene`, the HTTP
//! and MCP surface, the planner router) by `<name>`, its folder's name. The scene's YAML `name:` is
//! kept equal to it when it is saved.
//!
//! * Names ([`is_valid_name`]): letters, digits, `_`, `-` and `.`; no path separators, not starting
//!   with `.`. [`name_of`] also accepts the legacy file name `<name>.yaml`/`.yml` (what cells and
//!   clients said before the folder layout) and strips the extension.
//! * A scene's relative asset paths (`../../urdf-dressed-robots/…`) start at its folder.
//!   [`asset_path`] resolves one and falls back to the scenes folder (the old base, for content in
//!   the flat layout, e.g. what the shared catalogue sync brings) when the path does not exist from
//!   the scene's folder but does from there; [`fix_legacy_paths`] applies that to a scene as it is
//!   read, so every consumer (the reach check, the composer, the 3D view) sees working paths.
//! * Flat files (`<scenes>/<name>.yaml`) are migrated into folders at the backend's start
//!   ([`super::scene_migrate`]).

use std::path::{Component, Path, PathBuf};

use factoryos_protocol::fitter::{SCENES_DIR, SceneFile};

/// A scene's file in its folder.
pub const SCENE_FILE: &str = "scene.yaml";
/// The folder relative paths of a scene that has no folder yet (a new, unsaved scene; an inline
/// scene of a request) start at: inside the scenes folder, at a scene's depth, so the same paths
/// work. Hidden: never a scene.
const UNSAVED_DIR: &str = ".unsaved";

/// Whether `name` is a scene name: letters, digits, `_`, `-`, `.`; not empty, not starting with `.`.
pub fn is_valid_name(name: &str) -> bool {
    !name.is_empty()
        && !name.starts_with('.')
        && name.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.'))
}

/// The scene name a reference gives: a name, or a legacy `<name>.yaml`/`<name>.yml` (the extension
/// stripped). `None` if it is neither (a path, an invalid name).
pub fn name_of(reference: &str) -> Option<String> {
    let reference = reference.trim();
    let stem = reference.strip_suffix(".yaml").or_else(|| reference.strip_suffix(".yml")).unwrap_or(reference);
    is_valid_name(stem).then(|| stem.to_owned())
}

/// A scene name made of what the user typed: other characters become `-`, leading dots and dashes
/// go, and a legacy `.yaml`/`.yml` is stripped; `None` if nothing is left.
pub fn sanitize(typed: &str) -> Option<String> {
    let typed = typed.trim();
    let typed = typed.strip_suffix(".yaml").or_else(|| typed.strip_suffix(".yml")).unwrap_or(typed);
    if is_valid_name(typed) {
        return Some(typed.to_owned());
    }
    let name: String = typed.chars().map(|c| if c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.') { c } else { '-' }).collect();
    let name = name.trim_start_matches(['.', '-']).trim_end_matches(['.', '-']).to_owned();
    is_valid_name(&name).then_some(name)
}

/// The folder of scene `name`.
pub fn scene_dir(scenes_dir: &Path, name: &str) -> PathBuf {
    scenes_dir.join(name)
}

/// The file of scene `name`: `<scenes>/<name>/scene.yaml`.
pub fn scene_file(scenes_dir: &Path, name: &str) -> PathBuf {
    scene_dir(scenes_dir, name).join(SCENE_FILE)
}

/// Whether scene `name` (a valid name) is saved.
pub fn exists(scenes_dir: &Path, name: &str) -> bool {
    is_valid_name(name) && scene_file(scenes_dir, name).is_file()
}

/// The saved scenes' names (folders with a `scene.yaml`), sorted.
pub fn list(scenes_dir: &Path) -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir(scenes_dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|entry| {
            let name = entry.file_name().to_string_lossy().into_owned();
            (is_valid_name(&name) && entry.path().join(SCENE_FILE).is_file()).then_some(name)
        })
        .collect();
    names.sort();
    names
}

/// `base` if no scene (nor anything else) is called so in `scenes_dir`, else `base-2`, `base-3`, …
pub fn unique_name(scenes_dir: &Path, base: &str) -> String {
    (1..)
        .map(|i| if i == 1 { base.to_owned() } else { format!("{base}-{i}") })
        .find(|name| !scene_dir(scenes_dir, name).exists() && !flat_file_taken(scenes_dir, name))
        .expect("unbounded")
}

/// A flat legacy file `<name>.yaml`/`.yml` waits for migration under that name.
fn flat_file_taken(scenes_dir: &Path, name: &str) -> bool {
    ["yaml", "yml"].iter().any(|ext| scenes_dir.join(format!("{name}.{ext}")).is_file())
}

/// What a request's scene reference names.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SceneRef {
    /// A scene in the scenes folder, by name.
    Named(String),
    /// A scene YAML elsewhere (absolute; e.g. a cell's own `scene.yaml`).
    Path(PathBuf),
}

/// Parse a reference: a scene name, a legacy `<name>.yaml`, or an absolute path to a scene YAML
/// (`<scenes>/<name>/scene.yaml` and a legacy flat `<scenes>/<name>.yaml` are scene `<name>`).
/// `None`: not a reference (a relative path, an invalid name, an absolute path that is no YAML).
pub fn parse_ref(scenes_dir: &Path, reference: &str) -> Option<SceneRef> {
    let reference = reference.trim();
    let path = Path::new(reference);
    if !path.is_absolute() {
        return name_of(reference).map(SceneRef::Named);
    }
    if !path.extension().is_some_and(|e| e == "yaml" || e == "yml") {
        return None;
    }
    if let Some(name) = name_from_path(scenes_dir, path) {
        return Some(SceneRef::Named(name));
    }
    let parent = path.parent()?;
    if same_dir(parent, scenes_dir)
        && let Some(name) = name_of(&path.file_name()?.to_string_lossy())
    {
        return Some(SceneRef::Named(name));
    }
    Some(SceneRef::Path(path.to_owned()))
}

/// The name of the scene whose file is `path` (`<scenes>/<name>/scene.yaml`), if it is one.
pub fn name_from_path(scenes_dir: &Path, path: &Path) -> Option<String> {
    (path.file_name()? == SCENE_FILE).then_some(())?;
    let folder = path.parent()?;
    same_dir(folder.parent()?, scenes_dir).then_some(())?;
    let name = folder.file_name()?.to_string_lossy().into_owned();
    is_valid_name(&name).then_some(name)
}

/// Where the relative paths of scene `name` start: its folder, once it exists; before (a new scene
/// not saved yet) an existing folder at the same depth (`<scenes>/.unsaved`), so the same paths
/// resolve on disk.
pub fn base_for(scenes_dir: &Path, name: &str) -> PathBuf {
    let dir = scene_dir(scenes_dir, name);
    if is_valid_name(name) && dir.is_dir() { dir } else { unsaved_base(scenes_dir) }
}

/// The base of a scene with no folder (see [`base_for`]), made if missing.
pub fn unsaved_base(scenes_dir: &Path) -> PathBuf {
    let dir = scenes_dir.join(UNSAVED_DIR);
    std::fs::create_dir_all(&dir).ok();
    dir
}

/// A scene asset path `rel` (from the scene's YAML) resolved against the scene's folder `base`.
/// Legacy fallback: a relative path that does not exist from `base`, when `base` is a folder in a
/// scenes folder (`…/scenes/<name>`), is resolved against that scenes folder (the base of the flat
/// layout) if it exists from there.
pub fn asset_path(base: &Path, rel: &str) -> PathBuf {
    let path = Path::new(rel.trim());
    if path.is_absolute() {
        return path.to_owned();
    }
    let primary = base.join(path);
    if !primary.exists()
        && let Some(scenes) = legacy_base(base)
    {
        let legacy = scenes.join(path);
        if legacy.exists() {
            return legacy;
        }
    }
    primary
}

/// The scenes folder `base` (a scene's folder) is in, if it is in one.
fn legacy_base(base: &Path) -> Option<&Path> {
    base.parent().filter(|parent| parent.file_name().is_some_and(|n| n == SCENES_DIR))
}

/// [`asset_path`]'s legacy fallback applied to `scene` (its paths from `base`): a relative path
/// that only resolves from the scenes folder gets `../` in front, so it resolves from `base`. The
/// number of paths changed.
pub fn fix_legacy_paths(scene: &mut SceneFile, base: &Path) -> usize {
    if legacy_base(base).is_none() {
        return 0;
    }
    let mut changed = 0;
    let mut fix = |value: &mut String| {
        let rel = value.trim();
        if rel.is_empty() || Path::new(rel).is_absolute() || base.join(rel).exists() {
            return;
        }
        if asset_path(base, rel) != base.join(rel) {
            *value = format!("../{rel}");
            changed += 1;
        }
    };
    for robot in &mut scene.robots {
        fix(&mut robot.dressed_urdf);
    }
    for fixture in &mut scene.fixtures {
        fix(&mut fixture.urdf);
    }
    if let Some(plan) = scene.floor_plan.as_mut() {
        fix(&mut plan.geometry_file);
    }
    changed
}

/// Read and parse the scene YAML `path`, its paths from its folder (with the legacy fallback,
/// [`fix_legacy_paths`]).
pub fn read(path: &Path) -> anyhow::Result<SceneFile> {
    let text = std::fs::read_to_string(path).map_err(|err| anyhow::anyhow!("reading {}: {err}", path.display()))?;
    let mut scene = super::scene_paths::parse_scene_file(&text).map_err(|err| anyhow::anyhow!("{}: {err}", path.display()))?;
    if let Some(base) = path.parent() {
        fix_legacy_paths(&mut scene, base);
    }
    Ok(scene)
}

/// `path` without `.` and `..` (lexically).
pub fn normalize(path: &Path) -> PathBuf {
    let mut out = PathBuf::new();
    for component in path.components() {
        match component {
            Component::ParentDir => {
                out.pop();
            }
            Component::CurDir => {}
            other => out.push(other),
        }
    }
    out
}

fn same_dir(a: &Path, b: &Path) -> bool {
    let canonical = |p: &Path| std::fs::canonicalize(p).unwrap_or_else(|_| normalize(p));
    canonical(a) == canonical(b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_and_references() {
        for good in ["bw48-1", "jam2", "a.b", "two_arms", "2robot"] {
            assert!(is_valid_name(good), "{good}");
        }
        for bad in ["", ".hidden", "a/b", "a\\b", "..", "x y", "ä"] {
            assert!(!is_valid_name(bad), "{bad:?}");
        }
        assert_eq!(name_of("jam2.yaml").as_deref(), Some("jam2"));
        assert_eq!(name_of("jam2.yml").as_deref(), Some("jam2"));
        assert_eq!(name_of(" bw48-1 ").as_deref(), Some("bw48-1"));
        assert_eq!(name_of("../x.yaml"), None);
        assert_eq!(name_of(".yaml"), None);
        assert_eq!(sanitize("Two arms v2").as_deref(), Some("Two-arms-v2"));
        assert_eq!(sanitize("weld7.yaml").as_deref(), Some("weld7"));
        assert_eq!(sanitize("../cells/x").as_deref(), Some("cells-x"));
        assert_eq!(sanitize("  "), None);
        assert_eq!(sanitize("a-").as_deref(), Some("a-"), "a valid name stays as it is");
    }

    #[test]
    fn folders_list_and_resolve() {
        let root = std::env::temp_dir().join(format!("factoryos-scene-store-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let scenes = root.join("catalogue/scenes");
        let robot = root.join("catalogue/urdf-dressed-robots/r/r.urdf");
        std::fs::create_dir_all(robot.parent().unwrap()).unwrap();
        std::fs::write(&robot, "<robot/>").unwrap();
        std::fs::create_dir_all(scenes.join("a")).unwrap();
        std::fs::write(scene_file(&scenes, "a"), "name: a\nrobots:\n- robot_id: r\n  dressed_urdf: ../urdf-dressed-robots/r/r.urdf\n  position: {x: 0, y: 0, z: 0}\n").unwrap();
        std::fs::create_dir_all(scenes.join("empty")).unwrap();
        std::fs::write(scenes.join("flat.yaml"), "name: flat\n").unwrap();
        assert_eq!(list(&scenes), ["a"]);
        assert!(exists(&scenes, "a") && !exists(&scenes, "empty"));
        assert_eq!(unique_name(&scenes, "a"), "a-2");
        assert_eq!(unique_name(&scenes, "flat"), "flat-2", "a flat file waiting for migration counts");
        assert_eq!(unique_name(&scenes, "b"), "b");

        assert_eq!(parse_ref(&scenes, "a"), Some(SceneRef::Named("a".into())));
        assert_eq!(parse_ref(&scenes, "a.yaml"), Some(SceneRef::Named("a".into())));
        assert_eq!(parse_ref(&scenes, &scene_file(&scenes, "a").display().to_string()), Some(SceneRef::Named("a".into())));
        assert_eq!(parse_ref(&scenes, &scenes.join("flat.yaml").display().to_string()), Some(SceneRef::Named("flat".into())));
        assert_eq!(parse_ref(&scenes, "/cells/c/scene.yaml"), Some(SceneRef::Path("/cells/c/scene.yaml".into())));
        assert_eq!(parse_ref(&scenes, "/etc/passwd"), None);
        assert_eq!(parse_ref(&scenes, "../x.yaml"), None);

        // The legacy path (relative to scenes/) resolves through the fallback, and is fixed on read.
        let base = scene_dir(&scenes, "a");
        assert_eq!(normalize(&asset_path(&base, "../urdf-dressed-robots/r/r.urdf")), normalize(&robot));
        assert_eq!(normalize(&asset_path(&base, "../../urdf-dressed-robots/r/r.urdf")), normalize(&robot));
        let scene = read(&scene_file(&scenes, "a")).unwrap();
        assert_eq!(scene.robots[0].dressed_urdf, "../../urdf-dressed-robots/r/r.urdf");
        // A missing file stays as it is.
        assert_eq!(asset_path(&base, "missing.urdf"), base.join("missing.urdf"));
        // Not a scene folder: no fallback.
        let mut other = scene.clone();
        other.robots[0].dressed_urdf = "../urdf-dressed-robots/r/r.urdf".into();
        assert_eq!(fix_legacy_paths(&mut other, &root.join("cells/c")), 0);
        // A new scene's base is at a scene's depth.
        assert_eq!(base_for(&scenes, "new"), scenes.join(".unsaved"));
        assert!(base_for(&scenes, "new").join("../../urdf-dressed-robots/r/r.urdf").exists());
        assert_eq!(list(&scenes), ["a"], "the unsaved base is no scene");
        std::fs::remove_dir_all(&root).ok();
    }
}
