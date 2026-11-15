//! What the 3D view shows right now, as one of a few explicit modes, and the chip in the top bar
//! that says so (the 3D view's own title comes from the backend's blueprint).
//!
//! * Edit: the Robo Fitter's scene, unlinked, being edited.
//! * Cell open: the Fitter shows a cell's scene (read-only) and nothing simulates it.
//! * Simulation: the Fitter shows the cell the backend's simulated cell-runner runs (step 1 Load &
//!   Play), its robots following the behaviour tree.
//! * Live: the live cell's own recording (step 5 Watch the cell), from the virtual cell.
//! * A tool's preview (Product & Task, Robo Dresser), or the start page.

use crate::backend::BackendState;
use crate::rr::egui::Color32;

/// The page the app shows (its `Page`, as far as the 3D view goes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewPage {
    Home,
    Fitter,
    LiveCell,
    Product,
    Dresser,
    Training,
    /// Logs, placeholders: no 3D view.
    Other,
}

/// What [`visible`] decides from.
#[derive(Clone, Copy, Debug)]
pub struct ViewFacts<'a> {
    pub page: ViewPage,
    /// The Fitter's scene name and file.
    pub scene: &'a str,
    pub file: &'a str,
    /// The Fitter's scene has unsaved edits.
    pub unsaved: bool,
    /// The cell the Fitter is linked to.
    pub linked_cell: Option<&'a str>,
    /// The simulation (`factoryos/sim/status`): its state, the cell it runs, its error.
    pub sim: BackendState,
    pub sim_cell: Option<&'a str>,
    pub sim_error: Option<&'a str>,
    /// The behaviour tree the simulation has loaded, and whether it plays.
    pub bt: Option<&'a str>,
    pub playing: bool,
    /// The live cell's name (from its state), if one streams here.
    pub live_cell: Option<&'a str>,
    /// Rerun shows the live cell's recording.
    pub showing_live: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SimPhase {
    Starting,
    /// Up, no behaviour tree loaded.
    Ready,
    Playing,
    Paused,
    Failed(String),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tool {
    Product,
    Dresser,
    Training,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Visible {
    Home,
    /// Nothing 3D (logs, placeholders).
    None,
    Tool(Tool),
    Edit { scene: String, file: String, unsaved: bool },
    CellOpen { cell: String },
    Simulation { cell: String, bt: Option<String>, phase: SimPhase },
    /// `waiting`: the page is up, the live recording not shown yet.
    Live { cell: Option<String>, waiting: bool },
}

pub fn visible(f: &ViewFacts<'_>) -> Visible {
    match f.page {
        ViewPage::Home => Visible::Home,
        ViewPage::Other => Visible::None,
        ViewPage::Product => Visible::Tool(Tool::Product),
        ViewPage::Dresser => Visible::Tool(Tool::Dresser),
        ViewPage::Training => Visible::Tool(Tool::Training),
        ViewPage::LiveCell => Visible::Live { cell: f.live_cell.map(str::to_owned), waiting: !f.showing_live },
        ViewPage::Fitter => {
            let Some(cell) = f.linked_cell else {
                return Visible::Edit { scene: f.scene.to_owned(), file: f.file.to_owned(), unsaved: f.unsaved };
            };
            // An older backend does not say which cell it simulates: the linked one.
            let same_cell = f.sim_cell.is_none_or(|sim| sim == cell);
            let phase = match f.sim {
                _ if !same_cell => None,
                BackendState::Starting => Some(SimPhase::Starting),
                BackendState::Ready => Some(match (f.bt, f.playing) {
                    (Some(_), true) => SimPhase::Playing,
                    (Some(_), false) => SimPhase::Paused,
                    (None, _) => SimPhase::Ready,
                }),
                BackendState::Failed => {
                    Some(SimPhase::Failed(f.sim_error.unwrap_or("the simulated cell-runner failed").to_owned()))
                }
                BackendState::Stopped => None,
            };
            match phase {
                Some(phase) => Visible::Simulation { cell: cell.to_owned(), bt: f.bt.map(str::to_owned), phase },
                None => Visible::CellOpen { cell: cell.to_owned() },
            }
        }
    }
}

pub const EDIT: Color32 = Color32::from_rgb(110, 60, 230);
pub const READ_ONLY: Color32 = Color32::from_rgb(90, 90, 100);
pub const SIMULATION: Color32 = Color32::from_rgb(35, 134, 54);
pub const FAILED: Color32 = Color32::from_rgb(200, 50, 45);
pub const LIVE: Color32 = Color32::from_rgb(170, 30, 30);

/// The top bar's chip for `visible`: its text and fill; `None` without a 3D view.
pub fn label(visible: &Visible) -> Option<(String, Color32)> {
    Some(match visible {
        Visible::Home | Visible::None => return None,
        Visible::Tool(Tool::Product) => ("Product & Task".to_owned(), READ_ONLY),
        Visible::Tool(Tool::Dresser) => ("Robo Dresser".to_owned(), READ_ONLY),
        Visible::Tool(Tool::Training) => ("Build training job · drawn cells, not the scene".to_owned(), READ_ONLY),
        Visible::Edit { scene, file, unsaved } => {
            let what = if scene.is_empty() || scene == file { file.clone() } else { format!("{scene} ({file})") };
            (format!("✏ Editing · Robo Fitter · {what}{}", if *unsaved { " · unsaved" } else { "" }), EDIT)
        }
        Visible::CellOpen { cell } => (format!("🔒 Cell {cell} · read-only · no simulation"), READ_ONLY),
        Visible::Simulation { cell, bt, phase } => {
            let bt = bt.as_deref().map(|bt| format!(" · {bt}")).unwrap_or_default();
            let (phase, color) = match phase {
                SimPhase::Starting => (" · starting…".to_owned(), SIMULATION),
                SimPhase::Ready => (" · ready".to_owned(), SIMULATION),
                SimPhase::Playing => (" · playing".to_owned(), SIMULATION),
                SimPhase::Paused => (" · paused".to_owned(), SIMULATION),
                SimPhase::Failed(err) => (format!(" · failed: {err}"), FAILED),
            };
            (format!("▶ Simulation · {cell}{bt}{phase}"), color)
        }
        Visible::Live { cell, waiting } => {
            let cell = cell.as_deref().map(|cell| format!(" · {cell}")).unwrap_or_default();
            let waiting = if *waiting { " · waiting for its stream…" } else { "" };
            (format!("● Live cell{cell}{waiting}"), LIVE)
        }
    })
}

/// Opening the Fitter to edit (Open Fitter, a task program's Edit) asks first while it shows a
/// cell: editing stops the simulation and closes the cell's scene.
pub fn edit_needs_confirm(visible: &Visible) -> bool {
    matches!(visible, Visible::Simulation { .. } | Visible::CellOpen { .. })
}

/// The cell `Visible` shows (simulated or opened), if any.
pub fn shown_cell(visible: &Visible) -> Option<&str> {
    match visible {
        Visible::Simulation { cell, .. } | Visible::CellOpen { cell } => Some(cell),
        _ => None,
    }
}

/// The live cell's folder name from its `cell_name` (fos-live-bridge: `<cell> (live, <head state>)`).
pub fn live_cell_name(cell_name: &str) -> Option<String> {
    let name = cell_name.split(" (").next().unwrap_or_default().trim();
    (!name.is_empty()).then(|| name.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn facts() -> ViewFacts<'static> {
        ViewFacts {
            page: ViewPage::Fitter,
            scene: "two robots",
            file: "two.yaml",
            unsaved: false,
            linked_cell: None,
            sim: BackendState::Stopped,
            sim_cell: None,
            sim_error: None,
            bt: None,
            playing: false,
            live_cell: None,
            showing_live: false,
        }
    }

    #[test]
    fn edit_while_unlinked() {
        let edit = visible(&ViewFacts { unsaved: true, ..facts() });
        assert_eq!(edit, Visible::Edit { scene: "two robots".into(), file: "two.yaml".into(), unsaved: true });
        assert_eq!(label(&edit).unwrap(), ("✏ Editing · Robo Fitter · two robots (two.yaml) · unsaved".to_owned(), EDIT));
        assert!(!edit_needs_confirm(&edit));
        // A simulation of some cell runs: the unlinked Fitter is still the editor.
        let running = visible(&ViewFacts { sim: BackendState::Ready, sim_cell: Some("weld"), ..facts() });
        assert!(matches!(running, Visible::Edit { .. }));
    }

    #[test]
    fn a_linked_cell_without_simulation_is_read_only() {
        let open = visible(&ViewFacts { linked_cell: Some("weld"), ..facts() });
        assert_eq!(open, Visible::CellOpen { cell: "weld".into() });
        assert_eq!(label(&open).unwrap().0, "🔒 Cell weld · read-only · no simulation");
        assert!(edit_needs_confirm(&open));
    }

    #[test]
    fn simulation_of_the_linked_cell() {
        let base = ViewFacts { linked_cell: Some("weld"), sim_cell: Some("weld"), ..facts() };
        let starting = visible(&ViewFacts { sim: BackendState::Starting, ..base });
        assert_eq!(starting, Visible::Simulation { cell: "weld".into(), bt: None, phase: SimPhase::Starting });
        assert_eq!(label(&starting).unwrap().0, "▶ Simulation · weld · starting…");
        let playing = visible(&ViewFacts { sim: BackendState::Ready, bt: Some("bt_a"), playing: true, ..base });
        assert_eq!(label(&playing).unwrap(), ("▶ Simulation · weld · bt_a · playing".to_owned(), SIMULATION));
        assert!(edit_needs_confirm(&playing));
        let paused = visible(&ViewFacts { sim: BackendState::Ready, bt: Some("bt_a"), ..base });
        assert_eq!(label(&paused).unwrap().0, "▶ Simulation · weld · bt_a · paused");
        let failed = visible(&ViewFacts { sim: BackendState::Failed, sim_error: Some("boom"), ..base });
        assert_eq!(label(&failed).unwrap(), ("▶ Simulation · weld · failed: boom".to_owned(), FAILED));
    }

    #[test]
    fn a_simulation_of_another_cell_shows_the_cell_open() {
        let other = visible(&ViewFacts { linked_cell: Some("weld"), sim: BackendState::Ready, sim_cell: Some("glue"), bt: Some("bt_a"), ..facts() });
        assert_eq!(other, Visible::CellOpen { cell: "weld".into() });
    }

    #[test]
    fn live_cell() {
        let live = visible(&ViewFacts { page: ViewPage::LiveCell, live_cell: Some("welding_cell_2_small"), showing_live: true, ..facts() });
        assert_eq!(label(&live).unwrap(), ("● Live cell · welding_cell_2_small".to_owned(), LIVE));
        let unnamed = visible(&ViewFacts { page: ViewPage::LiveCell, showing_live: true, ..facts() });
        assert_eq!(label(&unnamed).unwrap().0, "● Live cell");
        let waiting = visible(&ViewFacts { page: ViewPage::LiveCell, ..facts() });
        assert_eq!(label(&waiting).unwrap().0, "● Live cell · waiting for its stream…");
        assert!(!edit_needs_confirm(&live));
    }

    #[test]
    fn tools_and_home() {
        assert_eq!(label(&visible(&ViewFacts { page: ViewPage::Home, ..facts() })), None);
        assert_eq!(label(&visible(&ViewFacts { page: ViewPage::Other, ..facts() })), None);
        assert_eq!(label(&visible(&ViewFacts { page: ViewPage::Product, ..facts() })).unwrap().0, "Product & Task");
        assert_eq!(label(&visible(&ViewFacts { page: ViewPage::Dresser, ..facts() })).unwrap().0, "Robo Dresser");
    }

    #[test]
    fn live_cell_names() {
        assert_eq!(live_cell_name("welding_cell_2_small (live, CS_RUNNING)").as_deref(), Some("welding_cell_2_small"));
        assert_eq!(live_cell_name("welding_cell_2_small (live)").as_deref(), Some("welding_cell_2_small"));
        assert_eq!(live_cell_name("cell").as_deref(), Some("cell"));
        assert_eq!(live_cell_name(""), None);
    }
}
