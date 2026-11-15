//! Robo Fitter scenes reference the original files: the catalogue's dressed robots, fixtures and
//! floor plans, and the products' fixture URDFs. Never a cell's copies (`cells/<cell>/robots/…`,
//! `fixtures/…`, `floorplans/…`): those belong to the cell and are replaced when it is rebuilt.
//!
//! A scene picks up cell paths when it is made from a cell's own `scene.yaml` (editing a cell, the
//! reach agent's `write_as` from a cell). [`to_originals`] maps them back; [`migrate_scenes`] fixes
//! the saved `scenes/` scenes on start. Only a cell's own `scene.yaml` is cell-relative.

use std::path::{Component, Path, PathBuf};

use factoryos_protocol::fitter::SceneFile;

/// The scene file `text` (a `robo_fitting_room` manifest). The keys of a mapping that
/// [`SceneFile`] doesn't model are kept as JSON values (`extra`), which can't hold everything
/// YAML can, so what they can't is turned into its YAML text first — the file loads, the key and
/// its content are kept (as a string) instead of the load failing:
/// - a mapping key that isn't a string (`1: x`, `[a, b]: x`);
/// - a tagged value (`!mark x`).
///
/// `.inf` and `.nan` in such a key still read as null (JSON has no non-finite numbers); in the
/// fields the scene models they are numbers, for validation to reject.
pub fn parse_scene_file(text: &str) -> Result<SceneFile, serde_yaml::Error> {
    let mut doc: serde_yaml::Value = serde_yaml::from_str(text)?;
    json_safe(&mut doc);
    serde_yaml::from_value(doc)
}

/// `value` with every mapping key a string and no tag: see [`parse_scene_file`].
fn json_safe(value: &mut serde_yaml::Value) {
    use serde_yaml::Value;
    let text = |v: &Value| serde_yaml::to_string(v).map(|s| s.trim_end().to_owned()).unwrap_or_default();
    match value {
        Value::Mapping(map) => {
            let entries = std::mem::take(map);
            for (key, mut item) in entries {
                json_safe(&mut item);
                let key = match key {
                    Value::String(_) => key,
                    other => Value::String(text(&other)),
                };
                map.insert(key, item);
            }
        }
        Value::Sequence(items) => items.iter_mut().for_each(json_safe),
        Value::Tagged(_) => *value = Value::String(text(value)),
        _ => {}
    }
}

/// Where the originals are.
#[derive(Clone, Debug)]
pub struct Originals {
    /// The asset root: `urdf-dressed-robots/`, `urdf-fixture-collection/`, `floor-plans/`,
    /// `scenes/`.
    pub catalogue: PathBuf,
    /// Product & Task's products folder.
    pub products: PathBuf,
    /// The cells folder, if known (a cell is also recognised by its `scene.yaml`).
    pub cells: Option<PathBuf>,
}

impl Originals {
    pub fn scenes_dir(&self) -> PathBuf {
        self.catalogue.join(factoryos_protocol::fitter::SCENES_DIR)
    }
}

/// What [`to_originals`] did.
#[derive(Debug, Default, PartialEq)]
pub struct Rewrite {
    /// Paths changed (cell copies to originals, or relative paths made absolute).
    pub changed: usize,
    /// Cell copies whose original is missing, kept as they are.
    pub warnings: Vec<String>,
}

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Robot,
    Fixture,
    FloorPlan,
}

/// A path into a cell's copies: `<cells>/<cell>/<robots|fixtures|floorplans>/<package>/<file>`.
struct CellCopy {
    cell: String,
    package: String,
    file: String,
}

fn normalize(path: &Path) -> PathBuf {
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

fn same_path(a: &Path, b: &Path) -> bool {
    let canonical = |p: &Path| std::fs::canonicalize(p).unwrap_or_else(|_| normalize(p));
    canonical(a) == canonical(b)
}

fn cell_copy(path: &Path, kind: Kind, cells: Option<&Path>) -> Option<CellCopy> {
    let name = |p: &Path| p.file_name().map(|n| n.to_string_lossy().into_owned());
    let file = name(path)?;
    let package_dir = path.parent()?;
    let kind_dir = package_dir.parent()?;
    let cell_dir = kind_dir.parent()?;
    let folder = match kind {
        Kind::Robot => "robots",
        Kind::Fixture => "fixtures",
        Kind::FloorPlan => "floorplans",
    };
    if name(kind_dir).as_deref() != Some(folder) {
        return None;
    }
    let in_cells = cells.is_some_and(|cells| cell_dir.parent().is_some_and(|parent| same_path(parent, cells)));
    if !in_cells && !cell_dir.join("scene.yaml").is_file() {
        return None;
    }
    Some(CellCopy { cell: name(cell_dir)?, package: name(package_dir)?, file })
}

/// `path` as a scene in folder `target` writes it: relative (`../../urdf-dressed-robots/…`) when
/// both are in the catalogue, else absolute.
fn written_from(target: &Path, path: &Path, catalogue: &Path) -> String {
    let (target, path, catalogue) = (normalize(target), normalize(path), normalize(catalogue));
    match (target.strip_prefix(&catalogue), path.strip_prefix(&catalogue)) {
        (Ok(up), Ok(down)) => {
            let ups = "../".repeat(up.components().count());
            format!("{ups}{}", down.display())
        }
        _ => path.display().to_string(),
    }
}

/// The original of a cell copy, as the Robo Fitter writes it in a scene in folder `target`
/// (`scenes/<name>/`), if it exists; else the places it was looked for.
fn original(copy: &CellCopy, kind: Kind, originals: &Originals, target: &Path) -> Result<String, Vec<PathBuf>> {
    let catalogue = |collection: &str, rel: String| {
        let path = originals.catalogue.join(collection).join(&rel);
        let text = written_from(target, &path, &originals.catalogue);
        (path, text)
    };
    let candidates = match kind {
        Kind::Robot => vec![catalogue("urdf-dressed-robots", format!("{}/{}", copy.package, copy.file))],
        Kind::Fixture => {
            // Products are referenced by their absolute path (as the palette adds them).
            let product = originals.products.join(&copy.package).join(&copy.file);
            let product_text = product.display().to_string();
            vec![(product, product_text), catalogue("urdf-fixture-collection", format!("{}/{}", copy.package, copy.file))]
        }
        Kind::FloorPlan => vec![catalogue("floor-plans", copy.file.clone())],
    };
    candidates
        .iter()
        .find(|(path, _)| path.is_file())
        .map(|(_, text)| text.clone())
        .ok_or_else(|| candidates.into_iter().map(|(path, _)| path).collect())
}

/// Rewrite `scene`'s asset paths that point into a cell's copies to the originals. Its relative
/// paths start at `base`; the result's start at `target` (the scene's folder, `scenes/<name>/`):
/// when the two differ, other relative paths are rewritten from `target` (relative when both are
/// in the catalogue, else absolute). A copy whose original is missing stays (absolute) with a
/// warning. Relative paths that only resolve from the scenes folder (the flat layout's base) are
/// read so ([`super::scene_store::asset_path`]).
pub fn to_originals(scene: &mut SceneFile, base: &Path, target: &Path, originals: &Originals) -> Rewrite {
    let mut rewrite = Rewrite::default();
    let moved = !same_path(base, target);
    let mut fix = |value: &mut String, kind: Kind, label: String| {
        if value.trim().is_empty() {
            return;
        }
        let raw = Path::new(value.as_str());
        let resolved = normalize(&super::scene_store::asset_path(base, value));
        let rebased = || written_from(target, &resolved, &originals.catalogue);
        let new_value = match cell_copy(&resolved, kind, originals.cells.as_deref()) {
            Some(copy) => match original(&copy, kind, originals, target) {
                Ok(text) => Some(text),
                Err(looked) => {
                    let looked: Vec<String> = looked.iter().map(|p| p.display().to_string()).collect();
                    rewrite.warnings.push(format!(
                        "{label} refers to cell {}'s copy: the original is missing ({})",
                        copy.cell,
                        looked.join(" or ")
                    ));
                    (moved && !raw.is_absolute()).then(|| resolved.display().to_string())
                }
            },
            None => (moved && !raw.is_absolute()).then(rebased),
        };
        if let Some(new_value) = new_value.filter(|v| v != value) {
            *value = new_value;
            rewrite.changed += 1;
        }
    };
    for robot in &mut scene.robots {
        fix(&mut robot.dressed_urdf, Kind::Robot, format!("Robot {}", robot.robot_id));
    }
    for fixture in &mut scene.fixtures {
        fix(&mut fixture.urdf, Kind::Fixture, format!("Fixture {}", fixture.id));
    }
    if let Some(plan) = scene.floor_plan.as_mut() {
        fix(&mut plan.geometry_file, Kind::FloorPlan, "The floor plan".to_owned());
    }
    rewrite
}

/// The warnings [`to_originals`] would give for a scene in folder `base` (copies it can't map).
pub fn cell_copy_warnings(scene: &SceneFile, base: &Path, originals: &Originals) -> Vec<String> {
    to_originals(&mut scene.clone(), base, base, originals).warnings
}

/// One scene file [`migrate_scenes`] rewrote.
#[derive(Debug, PartialEq)]
pub struct Migrated {
    pub file: String,
    pub changed: usize,
    pub backup: PathBuf,
    pub warnings: Vec<String>,
}

/// The first free `<file>.bak`, `<file>.bak.1`, … beside `path`.
fn backup_path(path: &Path) -> PathBuf {
    let name = path.file_name().unwrap_or_default().to_string_lossy().into_owned();
    (0..)
        .map(|i| path.with_file_name(if i == 0 { format!("{name}.bak") } else { format!("{name}.bak.{i}") }))
        .find(|p| !p.exists())
        .expect("unbounded")
}

/// Rewrite the saved scenes (`scenes/<name>/scene.yaml`) that reference a cell's copies to the
/// originals, keeping a `scene.yaml.bak` of each. Scenes without cell paths are not touched, so it
/// can run every start.
pub fn migrate_scenes(originals: &Originals) -> Vec<Migrated> {
    let scenes_dir = originals.scenes_dir();
    let mut migrated = Vec::new();
    for file in super::scene_store::list(&scenes_dir) {
        let path = super::scene_store::scene_file(&scenes_dir, &file);
        let folder = super::scene_store::scene_dir(&scenes_dir, &file);
        let Ok(text) = std::fs::read_to_string(&path) else { continue };
        let Ok(mut scene) = parse_scene_file(&text) else { continue };
        let rewrite = to_originals(&mut scene, &folder, &folder, originals);
        if rewrite.changed == 0 {
            for warning in &rewrite.warnings {
                tracing::warn!("Robo Fitter scene {file}: {warning}");
            }
            continue;
        }
        let result = (|| -> anyhow::Result<PathBuf> {
            let backup = backup_path(&path);
            std::fs::write(&backup, &text)?;
            std::fs::write(&path, serde_yaml::to_string(&scene)?)?;
            crate::storage::saved(&path);
            Ok(backup)
        })();
        match result {
            Ok(backup) => {
                tracing::info!(
                    "Robo Fitter scene {file}: {} path(s) to a cell's copies now reference the originals (backup {})",
                    rewrite.changed,
                    backup.display()
                );
                for warning in &rewrite.warnings {
                    tracing::warn!("Robo Fitter scene {file}: {warning}");
                }
                migrated.push(Migrated { file, changed: rewrite.changed, backup, warnings: rewrite.warnings });
            }
            Err(err) => tracing::warn!("Robo Fitter scene {file}: migrating its cell paths failed: {err:#}"),
        }
    }
    migrated
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    #[test]
    fn scene_files_with_what_json_cannot_hold_load_and_keep_it() {
        // A tag and non-string keys in keys the scene doesn't model: serde alone fails the load.
        let text = "name: s\nnote: !mark x\ntable: {1: x, true: y}\nlimit: .inf\nrobots:\n  - robot_id: R1\n    dressed_urdf: r.urdf\n    position: { x: 1.5, y: 2, z: 0 }\n    tags: { 7: seven }\n";
        assert!(serde_yaml::from_str::<SceneFile>(text).is_err());
        // The loader keeps them, as their YAML text; the modelled fields are untouched.
        let scene = parse_scene_file(text).unwrap();
        assert_eq!((scene.robots[0].robot_id.as_str(), scene.robots[0].position.x), ("R1", 1.5));
        assert_eq!(scene.extra["note"], "!mark x");
        assert_eq!(scene.extra["table"]["1"], "x");
        assert_eq!(scene.extra["table"]["true"], "y");
        assert_eq!(scene.robots[0].extra["tags"]["7"], "seven");
        // A non-finite number in such a key: null (JSON has none).
        assert_eq!(scene.extra["limit"], serde_json::Value::Null);
        // Saved, every key is still there.
        let saved: serde_yaml::Value = serde_yaml::from_str(&serde_yaml::to_string(&scene).unwrap()).unwrap();
        assert_eq!(saved["note"], serde_yaml::Value::from("!mark x"));
        assert_eq!(saved["table"]["1"], serde_yaml::Value::from("x"));
        assert_eq!(saved["robots"][0]["tags"]["7"], serde_yaml::Value::from("seven"));
        // A plain scene loads as serde reads it.
        let plain = "name: p\nrobots:\n  - robot_id: R2\n    dressed_urdf: r.urdf\n    position: { x: 0, y: 0, z: 0 }\n";
        assert_eq!(parse_scene_file(plain).unwrap(), serde_yaml::from_str::<SceneFile>(plain).unwrap());
    }

    /// The user's host: a catalogue, products and cell jam2 with copies of both.
    pub(crate) fn host(tag: &str) -> (PathBuf, Originals) {
        let root = std::env::temp_dir().join(format!("factoryos-scene-paths-{tag}-{}", std::process::id()));
        std::fs::remove_dir_all(&root).ok();
        let (catalogue, products, cells) = (root.join("catalogue"), root.join("products"), root.join("cells"));
        let files = [
            catalogue.join("urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf"),
            catalogue.join("urdf-fixture-collection/table/table.urdf"),
            products.join("bug_cube/bug_cube.urdf"),
            cells.join("jam2/robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf"),
            cells.join("jam2/fixtures/bug_cube/bug_cube.urdf"),
            cells.join("jam2/fixtures/table/table.urdf"),
            cells.join("jam2/fixtures/gone/gone.urdf"),
            cells.join("jam2/scene.yaml"),
        ];
        for file in files {
            std::fs::create_dir_all(file.parent().unwrap()).unwrap();
            std::fs::write(file, "<robot/>").unwrap();
        }
        std::fs::create_dir_all(catalogue.join("scenes")).unwrap();
        (root, Originals { catalogue, products, cells: Some(cells) })
    }

    /// `scene-fit.yaml` as the reach agent wrote it from cell jam2.
    pub(crate) fn scene_fit(cells: &Path) -> String {
        format!(
            "name: scene-fit\nrobots:\n- robot_id: kr240r2900__sticktool\n  dressed_urdf: {c}/jam2/robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf\n  \
             position: {{x: 2.5, y: 0.0, z: 0.0}}\nfixtures:\n- id: bug_cube\n  urdf: {c}/jam2/fixtures/bug_cube/bug_cube.urdf\n  \
             position: {{x: 0.0, y: 0.0, z: 0.0}}\n",
            c = cells.display()
        )
    }

    /// `text` as YAML, every number a float: what a scene file says, not how it is written.
    pub(crate) fn yaml_value(text: &str) -> serde_yaml::Value {
        fn floats(v: serde_yaml::Value) -> serde_yaml::Value {
            use serde_yaml::Value;
            match v {
                Value::Number(n) => Value::Number(n.as_f64().expect("a number").into()),
                Value::Sequence(items) => Value::Sequence(items.into_iter().map(floats).collect()),
                Value::Mapping(map) => Value::Mapping(map.into_iter().map(|(k, v)| (k, floats(v))).collect()),
                other => other,
            }
        }
        floats(serde_yaml::from_str(text).unwrap())
    }

    #[test]
    fn cell_copies_map_to_the_catalogue_and_products() {
        let (root, o) = host("map");
        let scenes = o.scenes_dir();
        let mut scene: SceneFile = serde_yaml::from_str(&scene_fit(o.cells.as_ref().unwrap())).unwrap();
        let rewrite = to_originals(&mut scene, &scenes, &scenes, &o);
        assert_eq!(rewrite, Rewrite { changed: 2, warnings: vec![] });
        assert_eq!(scene.robots[0].dressed_urdf, "../urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf");
        assert_eq!(scene.fixtures[0].urdf, o.products.join("bug_cube/bug_cube.urdf").display().to_string());
        assert_eq!(to_originals(&mut scene, &scenes, &scenes, &o), Rewrite::default(), "nothing left to do");
        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn a_cells_own_scene_becomes_an_editable_one() {
        let (root, o) = host("own");
        let cell = o.cells.as_ref().unwrap().join("jam2");
        let mut scene: SceneFile = serde_yaml::from_str(
            "name: jam2\nrobots:\n- robot_id: r\n  dressed_urdf: robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf\n  position: {x: 0, y: 0, z: 0}\n\
             fixtures:\n- id: table\n  urdf: fixtures/table/table.urdf\n  position: {x: 0, y: 0, z: 0}\n\
             - id: gone\n  urdf: fixtures/gone/gone.urdf\n  position: {x: 0, y: 0, z: 0}\n",
        )
        .unwrap();
        let rewrite = to_originals(&mut scene, &cell, &o.scenes_dir(), &o);
        assert_eq!(scene.robots[0].dressed_urdf, "../urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf");
        assert_eq!(scene.fixtures[0].urdf, "../urdf-fixture-collection/table/table.urdf", "no such product: the catalogue's");
        assert_eq!(scene.fixtures[1].urdf, cell.join("fixtures/gone/gone.urdf").display().to_string(), "kept, absolute");
        assert_eq!(rewrite.changed, 3);
        assert_eq!(rewrite.warnings.len(), 1);
        assert!(rewrite.warnings[0].starts_with("Fixture gone refers to cell jam2's copy: the original is missing ("), "{:?}", rewrite.warnings);
        assert_eq!(cell_copy_warnings(&scene, &o.scenes_dir(), &o), rewrite.warnings);
        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn other_paths_stay() {
        let (root, o) = host("other");
        let scenes = o.scenes_dir();
        let mut scene: SceneFile = serde_yaml::from_str(
            "name: s\nrobots:\n- robot_id: r\n  dressed_urdf: ../urdf-dressed-robots/x/x.urdf\n  position: {x: 0, y: 0, z: 0}\n\
             fixtures:\n- id: f\n  urdf: /elsewhere/fixtures/f/f.urdf\n  position: {x: 0, y: 0, z: 0}\n",
        )
        .unwrap();
        let before = scene.clone();
        assert_eq!(to_originals(&mut scene, &scenes, &scenes, &o), Rewrite::default());
        assert_eq!(scene, before);
        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn saved_scenes_are_migrated_once_with_a_backup() {
        let (root, o) = host("migrate");
        let scenes = o.scenes_dir();
        let text = scene_fit(o.cells.as_ref().unwrap());
        let (fit, jam2) = (scenes.join("scene-fit"), scenes.join("jam2"));
        std::fs::create_dir_all(&fit).unwrap();
        std::fs::create_dir_all(&jam2).unwrap();
        std::fs::write(fit.join("scene.yaml"), &text).unwrap();
        let clean = "name: jam2\nrobots: []\n# a comment\n";
        std::fs::write(jam2.join("scene.yaml"), clean).unwrap();

        let migrated = migrate_scenes(&o);
        assert_eq!(migrated.len(), 1, "{migrated:?}");
        assert_eq!((migrated[0].file.as_str(), migrated[0].changed), ("scene-fit", 2));
        assert_eq!(migrated[0].backup, fit.join("scene.yaml.bak"));
        assert_eq!(std::fs::read_to_string(fit.join("scene.yaml.bak")).unwrap(), text);
        let now = std::fs::read_to_string(fit.join("scene.yaml")).unwrap();
        assert!(now.contains("dressed_urdf: ../../urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf"), "{now}");
        assert!(now.contains(&format!("urdf: {}", o.products.join("bug_cube/bug_cube.urdf").display())), "{now}");
        assert!(!now.contains("cells/"), "{now}");
        assert_eq!(std::fs::read_to_string(jam2.join("scene.yaml")).unwrap(), clean, "untouched");

        assert!(migrate_scenes(&o).is_empty(), "idempotent");
        assert!(!fit.join("scene.yaml.bak.1").exists());
        assert_eq!(std::fs::read_to_string(fit.join("scene.yaml")).unwrap(), now);
        std::fs::remove_dir_all(&root).ok();
    }

    /// A scene written into another folder (the reach agent's `write_as` from scene `a`): its
    /// catalogue paths stay relative, from its own folder.
    #[test]
    fn a_scene_written_elsewhere_keeps_relative_catalogue_paths() {
        let (root, o) = host("rebase");
        let scenes = o.scenes_dir();
        let mut scene: SceneFile = serde_yaml::from_str(
            "name: a\nrobots:\n- robot_id: r\n  dressed_urdf: ../../urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf\n  position: {x: 0, y: 0, z: 0}\n",
        )
        .unwrap();
        std::fs::create_dir_all(scenes.join("a")).unwrap();
        let rewrite = to_originals(&mut scene, &scenes.join("a"), &scenes.join("b"), &o);
        assert_eq!(rewrite.warnings, Vec::<String>::new());
        assert_eq!(scene.robots[0].dressed_urdf, "../../urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf");
        std::fs::remove_dir_all(&root).ok();
    }

    /// Migrating rewrites the paths only: the scene's other keys (a payload, keys the backend
    /// doesn't model) stay.
    #[test]
    fn a_migrated_scene_keeps_its_other_keys() {
        let (root, o) = host("migrate-keys");
        let scenes = o.scenes_dir();
        let copy = o.cells.as_ref().unwrap().join("jam2/robots/arm/arm.urdf");
        for file in [&copy, &o.catalogue.join("urdf-dressed-robots/arm/arm.urdf")] {
            std::fs::create_dir_all(file.parent().unwrap()).unwrap();
            std::fs::write(file, "<robot/>").unwrap();
        }
        let text = format!(
            "name: keys\nlayout_rev: 3\nrobots:\n- robot_id: r\n  dressed_urdf: {}\n  position: {{x: 2.5, y: 0.0, z: 0.0}}\n  \
             payload: {{mass_kg: 12.0, source: datasheet}}\n  cable: [north, 2]\n",
            copy.display()
        );
        std::fs::create_dir_all(scenes.join("keys")).unwrap();
        std::fs::write(scenes.join("keys/scene.yaml"), &text).unwrap();
        assert_eq!(migrate_scenes(&o).len(), 1);
        let now = yaml_value(&std::fs::read_to_string(scenes.join("keys/scene.yaml")).unwrap());
        let mut expected = yaml_value(&text);
        expected["robots"][0]["dressed_urdf"] = "../../urdf-dressed-robots/arm/arm.urdf".into();
        expected["robots"][0]["orientation"] = yaml_value("{a: 0.0, b: 0.0, c: 0.0}");
        expected["fixtures"] = yaml_value("[]");
        assert_eq!(now, expected);
        std::fs::remove_dir_all(&root).ok();
    }
}
