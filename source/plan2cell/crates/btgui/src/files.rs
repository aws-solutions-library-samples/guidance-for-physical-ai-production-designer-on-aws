//! Reading trees and plans from disk, and where the bundled examples are.
//!
//! bt-view and bt-cell never touch the file system (bt-view also runs in a browser), so the
//! window reads the files. [`read_plan`] is cell-runner's `parse_plan_file`
//! (`trajectory.rs:131-155`) around [`plan::parse_plan`] and [`plan::merge_sibling`], with one
//! difference: a sibling `.json` that cannot be used is a [note](ReadPlan::note), not an error,
//! so the bare tree still shows (without the trajectories the sibling would have given it).

use std::path::{Path, PathBuf};

use bt_view::bt_cell::plan::{self, ParsedPlan};

/// A tree or plan read from disk.
#[derive(Debug)]
pub struct ReadPlan {
    /// The plan, as [`BtPanel::load_plan`](bt_view::BtPanel::load_plan) takes it.
    pub plan: ParsedPlan,
    /// Something went wrong that did not stop the plan from loading: the `.json` next to a
    /// bare XML tree could not be read or is not a plan.
    pub note: Option<String>,
}

/// Read a BehaviorTree.CPP XML file, or a plan JSON in any layout cell-runner accepts.
///
/// A bare XML tree takes its blackboard, `tree_id`, `cell_id` and `num_operations` from the
/// `.json` the planner writes next to it, if there is one; without ids of its own it is named
/// after its file. Like cell-runner, a tree counts as bare XML by what the file holds, not by
/// its extension.
///
/// The error is "cannot read …" or what is wrong with the text; it does not repeat the name.
pub fn read_plan(path: &Path) -> Result<ReadPlan, String> {
    let text = read(path)?;
    let mut plan = plan::parse_plan(&text)?;
    let mut note = None;
    // `parse_plan` skips a byte-order mark, so this does too.
    if plan::is_xml(text.strip_prefix('\u{feff}').unwrap_or(&text)) {
        let sibling = path.with_extension("json");
        let side = (sibling != path && sibling.is_file()).then(|| read(&sibling));
        let stem = path.file_stem().and_then(|s| s.to_str());
        let side_text = side.as_ref().and_then(|s| s.as_deref().ok());
        let merged = plan::merge_sibling(&mut plan, side_text, stem);
        let problem = match side {
            Some(Err(e)) => Some(e),
            _ => merged.err(),
        };
        note = problem.map(|e| {
            format!(
                "loaded without the blackboard in {}: {e}",
                file_name(&sibling)
            )
        });
    }
    Ok(ReadPlan { plan, note })
}

/// A file's name as the window shows it (the whole path if it has none).
pub fn file_name(path: &Path) -> String {
    path.file_name().map_or_else(
        || path.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    )
}

/// Where the file picker starts before anything was opened: the `examples/` folder next to the
/// app bundle (as `scripts/package-macos.sh` lays out the zip), else the copy inside it, else, in
/// a build from source, this workspace's `test_data/`.
///
/// The copy inside is for an app that runs without the zip's folder around it: moved to
/// `/Applications`, or started straight from the download, which macOS runs from a read-only
/// copy of the bare app (App Translocation) until the quarantine is lifted.
pub fn examples_dir() -> Option<PathBuf> {
    let exe = std::env::current_exe().ok();
    examples_dir_for(exe.as_deref(), Path::new(env!("CARGO_MANIFEST_DIR")))
}

/// [`examples_dir`] for the executable at `exe`, built from the crate at `manifest_dir`.
fn examples_dir_for(exe: Option<&Path>, manifest_dir: &Path) -> Option<PathBuf> {
    let app = exe.and_then(|exe| {
        exe.ancestors()
            .find(|dir| dir.extension().is_some_and(|ext| ext == "app"))
    });
    let beside = app.and_then(Path::parent).map(|dir| dir.join("examples"));
    let inside = app.map(|app| app.join("Contents/Resources/examples"));
    let source = manifest_dir
        .parent()
        .and_then(Path::parent)
        .map(|root| root.join("test_data"));
    [beside, inside, source]
        .into_iter()
        .flatten()
        .find(|dir| dir.is_dir())
}

fn read(path: &Path) -> Result<String, String> {
    std::fs::read_to_string(path).map_err(|e| format!("cannot read {}: {e}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_data(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../test_data")
            .join(name)
    }

    /// An empty directory of this test's own.
    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("btgui-files-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn a_bare_xml_picks_up_its_sibling() {
        let read = read_plan(&test_data("orchestrator_two_robots.xml")).unwrap();
        assert_eq!(read.note, None);
        let plan = read.plan;
        // The tree is the XML file's, the rest the JSON's.
        assert!(plan.tree.xml.starts_with("<?xml"), "{}", plan.tree.xml);
        assert!(plan.tree.blackboard["traj_A"].is_array());
        assert_eq!(plan.tree_id.as_deref(), Some("bt_1"));
        assert_eq!(plan.cell_id.as_deref(), Some("c1"));
        assert_eq!(plan.num_operations, Some(2));
    }

    #[test]
    fn a_plan_json_reads_as_it_is() {
        let name = "bt_1790934549_welding_cell_2_small.json";
        let read = read_plan(&test_data(name)).unwrap();
        assert_eq!(read.note, None);
        assert_eq!(
            read.plan.tree_id.as_deref(),
            Some("bt_1790934549_welding_cell_2_small")
        );
        assert!(read.plan.tree.blackboard.contains_key("traj_robot1_W01"));
    }

    #[test]
    fn errors_say_what_is_wrong() {
        let missing = test_data("no_such_tree.xml");
        let e = read_plan(&missing).unwrap_err();
        assert!(
            e.starts_with(&format!("cannot read {}: ", missing.display())),
            "{e}"
        );

        let dir = scratch("errors");
        let broken = dir.join("broken.json");
        std::fs::write(&broken, "{ not json").unwrap();
        let e = read_plan(&broken).unwrap_err();
        assert!(e.starts_with("invalid plan JSON: "), "{e}");
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn a_bare_xml_without_a_sibling_is_named_after_its_file() {
        let dir = scratch("alone");
        let path = dir.join("tree.xml");
        std::fs::write(&path, "<root BTCPP_format=\"4\"/>").unwrap();
        let read = read_plan(&path).unwrap();
        assert_eq!(read.note, None);
        assert_eq!(read.plan.tree_id.as_deref(), Some("tree"));
        assert!(read.plan.tree.blackboard.is_empty());
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn a_byte_order_mark_does_not_hide_the_sibling() {
        let dir = scratch("bom");
        let path = dir.join("tree.xml");
        std::fs::write(&path, "\u{feff}<root BTCPP_format=\"4\"/>").unwrap();
        let sibling =
            r#"{"tree_id": "t1", "behaviour_tree": {"xml": "<x/>", "blackboard": {"k": 1}}}"#;
        std::fs::write(dir.join("tree.json"), sibling).unwrap();
        let read = read_plan(&path).unwrap();
        assert_eq!(read.note, None);
        assert_eq!(read.plan.tree.xml, "<root BTCPP_format=\"4\"/>");
        assert_eq!(read.plan.tree_id.as_deref(), Some("t1"));
        assert_eq!(read.plan.tree.blackboard["k"], 1);
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn a_broken_sibling_keeps_the_bare_tree_with_a_note() {
        let dir = scratch("broken_sibling");
        let path = dir.join("plan.xml");
        std::fs::write(&path, "<root BTCPP_format=\"4\"/>").unwrap();

        // Not a plan.
        std::fs::write(dir.join("plan.json"), "{ not json").unwrap();
        let read = read_plan(&path).unwrap();
        assert_eq!(read.plan.tree.xml, "<root BTCPP_format=\"4\"/>");
        assert_eq!(read.plan.tree_id.as_deref(), Some("plan"));
        assert!(read.plan.tree.blackboard.is_empty());
        let note = read.note.unwrap();
        assert!(
            note.starts_with("loaded without the blackboard in plan.json: invalid plan JSON: "),
            "{note}"
        );

        // Not text.
        std::fs::write(dir.join("plan.json"), [0xff, 0xfe, 0x00]).unwrap();
        let read = read_plan(&path).unwrap();
        assert_eq!(read.plan.tree_id.as_deref(), Some("plan"));
        let note = read.note.unwrap();
        assert!(
            note.starts_with("loaded without the blackboard in plan.json: cannot read "),
            "{note}"
        );
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn xml_in_a_json_file_is_not_its_own_sibling() {
        let dir = scratch("xml_as_json");
        let path = dir.join("tree.json");
        std::fs::write(&path, "<root BTCPP_format=\"4\"/>").unwrap();
        let read = read_plan(&path).unwrap();
        assert_eq!(read.note, None);
        assert_eq!(read.plan.tree_id.as_deref(), Some("tree"));
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn examples_are_next_to_the_app_or_in_it_else_in_test_data() {
        let manifest = Path::new(env!("CARGO_MANIFEST_DIR"));
        let test_data = manifest.ancestors().nth(2).unwrap().join("test_data");
        assert_eq!(examples_dir_for(None, manifest), Some(test_data.clone()));
        // Run from target/: not inside an app bundle.
        assert_eq!(examples_dir(), Some(test_data.clone()));

        // The zip that scripts/package-macos.sh writes.
        let zip = scratch("bundle");
        let exe = zip.join("btgui.app/Contents/MacOS/btgui");
        assert_eq!(examples_dir_for(Some(&exe), manifest), Some(test_data));
        let inside = zip.join("btgui.app/Contents/Resources/examples");
        std::fs::create_dir_all(&inside).unwrap();
        std::fs::create_dir(zip.join("examples")).unwrap();
        assert_eq!(
            examples_dir_for(Some(&exe), manifest),
            Some(zip.join("examples"))
        );
        // The app on its own (moved, or translocated): the copy inside it.
        std::fs::remove_dir(zip.join("examples")).unwrap();
        assert_eq!(examples_dir_for(Some(&exe), manifest), Some(inside.clone()));
        // None of them: the picker starts in the working directory.
        std::fs::remove_dir(&inside).unwrap();
        assert_eq!(examples_dir_for(Some(&exe), &zip.join("a/b")), None);
        let _ = std::fs::remove_dir_all(zip);
    }
}
