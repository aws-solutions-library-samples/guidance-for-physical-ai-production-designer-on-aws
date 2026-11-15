//! The sidebar's cell list: the cells are the Robo Fitter's scenes. A cell folder only holds a
//! scene's outputs (prepared planner model, task programs, plans, exports) and references its scene
//! (`CellKind::Reference`); a legacy cell is a self-contained copy. A saved scene without a cell
//! folder is listed too: picking it makes its cell (the backend's `ensure_cell`, via
//! `POST /api/v1/cell/create`).

use factoryos_client::protocol::{CellKind, CellsInfo};

/// One entry of the cell pickers.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CellRow {
    /// The cell's name: its folder, or for a scene without one the name its cell will get.
    pub name: String,
    /// The Robo Fitter scene file (`scenes/`) it references or was copied from, if known.
    pub scene: Option<String>,
    /// What the folder is; `None` for an older backend that does not say.
    pub kind: Option<CellKind>,
    /// The cell has a folder (else it is a scene that was never used as a cell).
    pub folder: bool,
    /// Why its prepared planner model is stale, as the badge says it.
    pub badge: Option<String>,
}

/// What picking a row does.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CellTarget {
    /// An existing cell folder.
    Cell(String),
    /// A scene without a cell folder: make (or reuse) its cell first.
    Scene(String),
}

impl CellRow {
    pub fn target(&self) -> CellTarget {
        match (&self.scene, self.folder) {
            (Some(scene), false) => CellTarget::Scene(scene.clone()),
            _ => CellTarget::Cell(self.name.clone()),
        }
    }

    pub fn is_legacy(&self) -> bool {
        self.kind == Some(CellKind::Legacy)
    }

    /// `jam2`, `welding_cell_2_small · legacy`, `new_scene · not used yet`, plus ` · ⚠ stale`.
    pub fn label(&self) -> String {
        let mut label = self.name.clone();
        if !self.folder {
            label.push_str(" · new");
        } else if self.is_legacy() {
            label.push_str(" · legacy");
        }
        if self.badge.is_some() {
            label.push_str(" · ⚠ stale");
        }
        label
    }

    /// The row's hover text: its scene, what it is and why it is stale.
    pub fn hover(&self) -> String {
        let scene = self.scene.as_deref().map(|s| format!("scenes/{}/", s.strip_suffix(".yaml").or_else(|| s.strip_suffix(".yml")).unwrap_or(s)));
        let mut lines = vec![match (self.folder, self.kind, scene.as_deref()) {
            (false, _, Some(scene)) => {
                format!("Scene {scene}, not used as a cell yet: picking it makes its cell (a folder for its outputs)")
            }
            (_, Some(CellKind::Reference), Some(scene)) => {
                format!("References {scene}; robots and products come from the catalogue (nothing copied)")
            }
            (_, Some(CellKind::Reference), None) => "References its scene (nothing copied)".to_owned(),
            (_, Some(CellKind::Legacy), Some(scene)) => {
                format!("Legacy cell: its own copy of the scene, robots and fixtures (copied from {scene})")
            }
            (_, Some(CellKind::Legacy), None) => "Legacy cell: its own scene, robots and fixtures".to_owned(),
            _ => format!("Cell {}", self.name),
        }];
        if let Some(badge) = &self.badge {
            lines.push(badge.clone());
        }
        lines.join("\n")
    }
}

/// The badge for stale outputs: `⚠ stale: robot r1's URDF changed`. `None` when not stale.
pub fn stale_badge(stale: bool, reason: Option<&str>) -> Option<String> {
    match reason.map(str::trim).filter(|r| !r.is_empty()) {
        Some(reason) => Some(format!("⚠ stale: {reason}")),
        None if stale => Some("⚠ stale: its inputs changed since".to_owned()),
        None => None,
    }
}

/// The rows: the cell folders (as the backend lists them), then each saved scene no folder
/// references. From an older backend (no `entries`): the folders only, without tags.
pub fn cell_rows(info: &CellsInfo) -> Vec<CellRow> {
    if info.entries.is_empty() {
        let mut rows: Vec<CellRow> = info
            .cells
            .iter()
            .map(|name| CellRow { name: name.clone(), scene: None, kind: None, folder: true, badge: None })
            .collect();
        if info.cells.is_empty() {
            rows.extend(scene_rows(info, &[]));
        }
        return rows;
    }
    let mut rows: Vec<CellRow> = info
        .entries
        .iter()
        .map(|entry| CellRow {
            name: entry.name.clone(),
            scene: entry.scene.clone(),
            kind: Some(entry.kind),
            folder: true,
            badge: stale_badge(entry.prepared_stale, entry.stale_reason.as_deref()),
        })
        .collect();
    let used: Vec<&str> = info.entries.iter().filter_map(|e| e.scene.as_deref()).collect();
    let mut taken: Vec<String> = rows.iter().map(|r| r.name.clone()).collect();
    for mut row in scene_rows(info, &used) {
        row.name = unique(&row.name, &taken);
        taken.push(row.name.clone());
        rows.push(row);
    }
    rows
}

fn scene_rows(info: &CellsInfo, used: &[&str]) -> Vec<CellRow> {
    info.fitted_scenes
        .iter()
        .filter(|scene| !used.contains(&scene.as_str()))
        .map(|scene| CellRow {
            name: clean_name(scene),
            scene: Some(scene.clone()),
            kind: None,
            folder: false,
            badge: None,
        })
        .collect()
}

/// `name`, or `name_2`, `name_3`, … when a folder has it (as the backend names a new cell).
fn unique(name: &str, taken: &[String]) -> String {
    (1..)
        .map(|i| if i == 1 { name.to_owned() } else { format!("{name}_{i}") })
        .find(|candidate| !taken.contains(candidate))
        .expect("unbounded")
}

/// The cell name of a scene file, as the backend's `cell_resolve::clean_name`: the file stem with
/// everything but letters and digits as `_`, trimmed of `_` (it becomes a ROS namespace).
pub fn clean_name(scene: &str) -> String {
    let file = scene.rsplit('/').next().unwrap_or(scene);
    // A scene name; a legacy `<name>.yaml` without its extension.
    let stem = file.strip_suffix(".yaml").or_else(|| file.strip_suffix(".yml")).unwrap_or(file);
    let name: String = stem.chars().map(|c| if c.is_ascii_alphanumeric() { c } else { '_' }).collect();
    let name = name.trim_matches('_');
    if name.is_empty() { "cell".to_owned() } else { name.to_owned() }
}

/// The row of the active cell (or the picked one), if listed.
pub fn row<'a>(rows: &'a [CellRow], name: &str) -> Option<&'a CellRow> {
    rows.iter().find(|row| row.folder && row.name == name)
}

/// The fitted scene a reference cell `cell` references (`None` for a legacy cell or an older
/// backend): "Stop simulation and edit" opens it for editing.
pub fn referenced_scene(info: &CellsInfo, cell: &str) -> Option<String> {
    info.entries
        .iter()
        .find(|entry| entry.name == cell && entry.kind == CellKind::Reference)
        .and_then(|entry| entry.scene.clone())
}

#[cfg(test)]
mod tests {
    use super::*;
    use factoryos_client::protocol::CellEntry;

    fn entry(name: &str, scene: Option<&str>, kind: CellKind, reason: Option<&str>) -> CellEntry {
        CellEntry {
            name: name.to_owned(),
            scene: scene.map(str::to_owned),
            kind,
            prepared_stale: reason.is_some(),
            stale_reason: reason.map(str::to_owned),
        }
    }

    fn info(entries: Vec<CellEntry>, scenes: &[&str]) -> CellsInfo {
        CellsInfo {
            cells: entries.iter().map(|e| e.name.clone()).collect(),
            entries,
            fitted_scenes: scenes.iter().map(|s| (*s).to_owned()).collect(),
            ..CellsInfo::default()
        }
    }

    #[test]
    fn the_cell_list_is_the_list_of_scenes() {
        let info = info(
            vec![
                entry("cell_new_scene", Some("new-scene.yaml"), CellKind::Reference, None),
                entry("jam2", Some("jam2.yaml"), CellKind::Reference, Some("robot r1's URDF changed")),
                entry("welding_cell_2_small", None, CellKind::Legacy, None),
            ],
            &["jam2.yaml", "new-scene.yaml", "three robots.yaml", "jam2-copy.yaml"],
        );
        let rows = cell_rows(&info);
        let labels: Vec<String> = rows.iter().map(CellRow::label).collect();
        assert_eq!(
            labels,
            [
                "cell_new_scene",
                "jam2 · ⚠ stale",
                "welding_cell_2_small · legacy",
                "three_robots · new",
                "jam2_copy · new",
            ]
        );
        assert_eq!(rows[0].target(), CellTarget::Cell("cell_new_scene".into()), "new-scene.yaml reuses its folder");
        assert_eq!(rows[3].target(), CellTarget::Scene("three robots.yaml".into()));
        assert!(rows[2].is_legacy() && !rows[0].is_legacy() && !rows[3].is_legacy());
        assert_eq!(rows[1].hover(), "References scenes/jam2/; robots and products come from the catalogue (nothing copied)\n⚠ stale: robot r1's URDF changed");
        assert_eq!(rows[2].hover(), "Legacy cell: its own scene, robots and fixtures");
        assert_eq!(
            rows[3].hover(),
            "Scene scenes/three robots/, not used as a cell yet: picking it makes its cell (a folder for its outputs)"
        );
        assert_eq!(row(&rows, "jam2").map(|r| r.name.as_str()), Some("jam2"));
        assert_eq!(row(&rows, "three_robots"), None, "no folder yet");
        assert_eq!(referenced_scene(&info, "jam2").as_deref(), Some("jam2.yaml"));
        assert_eq!(referenced_scene(&info, "welding_cell_2_small"), None, "legacy");
    }

    #[test]
    fn a_scene_named_like_an_unrelated_folder_gets_the_next_name() {
        let info = info(vec![entry("jam2", None, CellKind::Legacy, None)], &["jam2.yaml"]);
        let rows = cell_rows(&info);
        assert_eq!(rows[1].name, "jam2_2");
        assert_eq!(rows[1].target(), CellTarget::Scene("jam2.yaml".into()));
    }

    #[test]
    fn an_older_backend_lists_folders_without_tags() {
        let old = CellsInfo { cells: vec!["a".into(), "b".into()], fitted_scenes: vec!["x.yaml".into()], ..CellsInfo::default() };
        let rows = cell_rows(&old);
        assert_eq!(rows.iter().map(CellRow::label).collect::<Vec<_>>(), ["a", "b"]);
        assert_eq!(rows[0].hover(), "Cell a");
        // No cells at all: the scenes are still listed.
        let empty = CellsInfo { fitted_scenes: vec!["x.yaml".into()], ..CellsInfo::default() };
        assert_eq!(cell_rows(&empty)[0].target(), CellTarget::Scene("x.yaml".into()));
    }

    #[test]
    fn stale_badges_say_why() {
        assert_eq!(stale_badge(false, None), None);
        assert_eq!(stale_badge(true, Some("the scene jam2.yaml changed")).as_deref(), Some("⚠ stale: the scene jam2.yaml changed"));
        assert_eq!(stale_badge(true, None).as_deref(), Some("⚠ stale: its inputs changed since"));
        assert_eq!(stale_badge(true, Some("  ")).as_deref(), Some("⚠ stale: its inputs changed since"));
        assert_eq!(stale_badge(false, Some("robot r1's URDF changed")).as_deref(), Some("⚠ stale: robot r1's URDF changed"), "a reason alone");
    }

    #[test]
    fn cell_names_follow_the_backend() {
        assert_eq!(clean_name("jam2.yaml"), "jam2");
        assert_eq!(clean_name("new-scene.yaml"), "new_scene");
        assert_eq!(clean_name("_x-.yml"), "x");
        assert_eq!(clean_name("--.yaml"), "cell");
        assert_eq!(clean_name("two robots.v2.yaml"), "two_robots_v2");
    }
}
