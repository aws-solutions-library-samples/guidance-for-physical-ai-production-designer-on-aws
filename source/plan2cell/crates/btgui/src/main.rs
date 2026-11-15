//! btgui: a window around [`bt_view::BtPanel`].
//!
//! ```text
//! btgui [tree.xml | plan.json] [--manual] [--run | --ticks N] [--select NAME]
//! ```
//!
//! Without a file, a file picker opens as soon as the window is up, in the examples that come with
//! the app (`test_data/` in a build from source); cancel it and the window offers "Open…" instead.
//! Files can also be opened with File ▸ Open… (⌘O), read again with File ▸ Reload (⌘R), or
//! dropped on the window. A bare `.xml` plan picks up the `.json` next to it ([`files`]).
//!
//! * `--manual` — every action (the cell leaves too) waits for an override instead of running.
//! * `--run` — run in virtual time until the tree finishes or halts.
//! * `--ticks N` — take N steps (each one tick, at the next instant something changes).
//! * `--select NAME` — select the node with this instance name.
//!
//! They apply to every file that loads.

mod files;

use std::path::{Path, PathBuf};

use bt_view::BtPanel;
use eframe::egui;
use egui::{Key, KeyboardShortcut, Modifiers};

const USAGE: &str = "\
btgui [tree.xml | plan.json] [--manual] [--run | --ticks N] [--select NAME]

  --manual       every action (the cell leaves too) waits for an override
  --run          run in virtual time until the tree finishes or halts
  --ticks N      take N steps
  --select NAME  select the node with this instance name

Without a file, a file picker opens once the window is up.";

/// File ▸ Open….
const OPEN: KeyboardShortcut = KeyboardShortcut::new(Modifiers::COMMAND, Key::O);
/// File ▸ Reload.
const RELOAD: KeyboardShortcut = KeyboardShortcut::new(Modifiers::COMMAND, Key::R);

/// What to do with a tree once it is loaded.
#[derive(Clone, Copy, Default, PartialEq)]
enum OnLoad {
    #[default]
    Nothing,
    /// Run in virtual time until the tree finishes or halts.
    Run,
    /// Take this many steps.
    Ticks(u32),
}

struct App {
    panel: BtPanel,
    /// The file last opened (or tried), for Reload.
    path: Option<PathBuf>,
    /// The folder it is in, where the picker starts next.
    last_dir: Option<PathBuf>,
    /// Open the picker once the window shows (no file was named on the command line).
    pick_on_start: bool,
    /// Frames drawn so far.
    frames: u64,
    manual: bool,
    on_load: OnLoad,
    select: Option<String>,
}

impl App {
    fn new() -> Self {
        let mut panel = BtPanel::new("btgui");
        // Whoever cancels the picker can still open a file from the empty canvas.
        panel.set_open_button(true);
        Self {
            panel,
            path: None,
            last_dir: None,
            pick_on_start: false,
            frames: 0,
            manual: false,
            on_load: OnLoad::Nothing,
            select: None,
        }
    }

    /// Open a tree or plan. What goes wrong shows in the panel, which keeps what it showed; the
    /// command-line options only apply to a file that loads.
    fn open(&mut self, path: &Path) {
        let name = files::file_name(path);
        let loaded = match files::read_plan(path) {
            Ok(read) => {
                // The panel shows its own load errors.
                let loaded = self.panel.load_plan(&name, read.plan).is_ok();
                if loaded && let Some(note) = read.note {
                    self.panel.set_note(format!("{name}: {note}"));
                }
                loaded
            }
            Err(e) => {
                self.panel.set_error(format!("{name}: {e}"));
                false
            }
        };
        // Reload tries again, and the picker starts there, whether or not it worked.
        self.path = Some(path.to_path_buf());
        self.last_dir = std::path::absolute(path)
            .ok()
            .and_then(|path| path.parent().map(Path::to_path_buf));
        if !loaded {
            return;
        }
        if self.manual
            && let Some(ex) = self.panel.executor_mut()
        {
            ex.set_all_policies(bt_view::NodeKind::Action, bt_view::LeafPolicy::Manual);
        }
        if let Some(name) = &self.select
            && self.panel.select_by_name(name).is_none()
        {
            eprintln!("no node named {name}");
        }
        match self.on_load {
            OnLoad::Nothing => {}
            OnLoad::Run => self.panel.run_to_wait(),
            OnLoad::Ticks(n) => {
                for _ in 0..n {
                    self.panel.step();
                }
            }
        }
    }

    /// Where the picker starts: the folder of the last file, else the examples, else the
    /// working directory.
    fn start_dir(&self) -> Option<PathBuf> {
        self.last_dir
            .clone()
            .filter(|dir| dir.is_dir())
            .or_else(files::examples_dir)
            .or_else(|| std::env::current_dir().ok())
    }

    /// Ask for a file, in front of the window; `None` if the user cancels.
    fn pick(&self, frame: &eframe::Frame) -> Option<PathBuf> {
        let mut dialog = rfd::FileDialog::new()
            .set_title("Open a behaviour tree or plan")
            .add_filter("Behaviour tree or plan", &["json", "xml"])
            .set_parent(frame);
        if let Some(dir) = self.start_dir() {
            dialog = dialog.set_directory(dir);
        }
        dialog.pick_file()
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.frames += 1;
        let mut pick = false;
        if self.pick_on_start {
            // The window only shows once its first frame is drawn, and the picker belongs in
            // front of it: draw that frame, then ask in the next.
            if self.frames == 1 {
                ctx.request_repaint();
            } else {
                self.pick_on_start = false;
                pick = true;
            }
        }
        let dropped = ctx.input(|i| i.raw.dropped_files.first().map(|f| f.path().to_path_buf()));
        if let Some(path) = dropped {
            self.open(&path);
        }
        // Before the panel draws, so that no field being typed in sees them.
        pick |= ctx.input_mut(|i| i.consume_shortcut(&OPEN));
        let mut reload = ctx.input_mut(|i| i.consume_shortcut(&RELOAD));
        let can_reload = self.path.is_some();
        egui::Panel::top("menu").show(ui, |ui| {
            egui::MenuBar::new().ui(ui, |ui| {
                ui.menu_button("File", |ui| {
                    let open = egui::Button::new("Open…").shortcut_text(ctx.format_shortcut(&OPEN));
                    if ui.add(open).clicked() {
                        pick = true;
                        ui.close();
                    }
                    let again =
                        egui::Button::new("Reload").shortcut_text(ctx.format_shortcut(&RELOAD));
                    if ui.add_enabled(can_reload, again).clicked() {
                        reload = true;
                        ui.close();
                    }
                });
            });
        });
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE)
            .show(ui, |ui| self.panel.ui(ui));
        // The empty canvas's "Open…".
        pick |= self.panel.take_open_request();
        let path = if pick {
            self.pick(frame)
        } else if reload {
            self.path.clone()
        } else {
            None
        };
        if let Some(path) = path {
            self.open(&path);
            // This frame showed what was there before.
            ctx.request_repaint();
        }
    }
}

fn main() -> eframe::Result {
    let mut app = App::new();
    let mut file = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--manual" => app.manual = true,
            "--run" => app.on_load = OnLoad::Run,
            "--select" => app.select = args.next(),
            "--ticks" => {
                app.on_load = OnLoad::Ticks(args.next().and_then(|n| n.parse().ok()).unwrap_or(1))
            }
            "-h" | "--help" => {
                println!("{USAGE}");
                return Ok(());
            }
            other => file = Some(PathBuf::from(other)),
        }
    }
    match file {
        Some(path) => app.open(&path),
        None => app.pick_on_start = true,
    }
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("btgui")
            .with_inner_size([1400.0, 900.0])
            .with_drag_and_drop(true),
        ..Default::default()
    };
    eframe::run_native(
        "btgui",
        options,
        Box::new(|cc| {
            // egui's default fonts are off (their Ubuntu-Light licence): bt-view's.
            bt_view::fonts::install(&cc.egui_ctx);
            Ok(Box::new(app))
        }),
    )
}

#[cfg(test)]
mod tests {
    use bt_view::{LeafPolicy, Stop};

    use super::*;

    fn test_data(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../test_data")
            .join(name)
    }

    #[test]
    fn a_file_that_cannot_be_read_is_an_error() {
        let mut app = App::new();
        let missing = test_data("no_such_tree.xml");
        app.open(&missing);
        let e = app.panel.error().expect("an error is shown");
        assert!(
            e.starts_with(&format!(
                "no_such_tree.xml: cannot read {}: ",
                missing.display()
            )),
            "{e}"
        );
        // Not a tree made of the error message.
        assert!(app.panel.executor().is_none());
        // Reload tries again, and the picker starts where the file was looked for.
        assert_eq!(app.path.as_deref(), Some(missing.as_path()));
        let dir = app.start_dir().unwrap();
        assert!(dir.is_absolute() && dir.ends_with("test_data"), "{dir:?}");
    }

    #[test]
    fn the_picker_starts_in_the_examples() {
        let app = App::new();
        let dir = app.start_dir().unwrap();
        assert_eq!(Some(&dir), files::examples_dir().as_ref());
        assert!(dir.join("v4_cell_with_gates.xml").is_file(), "{dir:?}");
    }

    #[test]
    fn options_apply_to_files_that_load() {
        let mut app = App::new();
        app.manual = true;
        app.select = Some("weld_r1_A".to_string());
        app.on_load = OnLoad::Run;
        app.open(&test_data("orchestrator_two_robots.xml"));
        assert_eq!(app.panel.error(), None);
        // The sibling JSON gives the bare XML its trajectories, so the cell leaves run.
        assert!(app.panel.cell().is_some());
        assert_eq!(
            app.panel.plan_info().unwrap().tree_id.as_deref(),
            Some("bt_1")
        );
        let ex = app.panel.executor().unwrap();
        let weld = app.panel.selected().expect("--select");
        assert_eq!(ex.tree().node(weld).name, "weld_r1_A");
        assert_eq!(ex.policy(weld), LeafPolicy::Manual);
        assert!(
            matches!(app.panel.last_stop(), Some(Stop::WaitingManual(uids)) if uids.contains(&weld)),
            "{:?}",
            app.panel.last_stop()
        );

        // A file that fails keeps the tree, and the options are not applied again.
        app.panel.reset();
        app.open(&test_data("no_such_tree.xml"));
        assert!(app.panel.error().is_some());
        assert_eq!(app.panel.tree_id(), Some("MainTree"));
        assert_eq!(app.panel.last_stop(), None);
    }

    /// The panel's error row, as one frame draws it (`None` if there is none).
    fn error_row(panel: &mut BtPanel) -> Option<String> {
        let ctx = egui::Context::default();
        ctx.enable_accesskit();
        let mut output = ctx.run_ui(egui::RawInput::default(), |ui| {
            egui::CentralPanel::default().show(ui, |ui| panel.ui(ui));
        });
        output.textures_delta.clear();
        let error = panel.error()?.to_string();
        let nodes = output.platform_output.accesskit_update?.nodes;
        nodes.into_iter().find_map(|(_, node)| {
            node.value()
                .filter(|v| v.contains(&error))
                .map(str::to_string)
        })
    }

    #[test]
    fn notes_and_load_errors_show_in_the_panel() {
        let dir = std::env::temp_dir().join(format!("btgui-main-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let xml = std::fs::read_to_string(test_data("orchestrator_two_robots.xml")).unwrap();
        std::fs::write(dir.join("plan.xml"), xml).unwrap();
        std::fs::write(dir.join("plan.json"), "{ not json").unwrap();
        std::fs::write(dir.join("empty.xml"), "<root BTCPP_format=\"4\"/>").unwrap();

        // The tree loads, on stubs, and the error says why it has no trajectories.
        let mut app = App::new();
        app.open(&dir.join("plan.xml"));
        assert!(app.panel.executor().is_some());
        assert!(app.panel.cell().is_none());
        let e = app.panel.error().unwrap();
        assert!(
            e.starts_with("plan.xml: loaded without the blackboard in plan.json: "),
            "{e}"
        );
        // It is about the tree on screen, not an older one.
        let row = error_row(&mut app.panel).expect("the error row");
        assert!(!row.contains("still showing"), "{row}");

        // A file that reads but does not load: the panel says why and keeps its tree.
        app.open(&dir.join("empty.xml"));
        assert!(app.panel.error().unwrap().starts_with("empty.xml: "));
        let row = error_row(&mut app.panel).expect("the error row");
        assert!(row.ends_with("(still showing plan.xml)"), "{row}");
        assert_eq!(app.panel.tree_id(), Some("MainTree"));
        assert_eq!(app.path, Some(dir.join("empty.xml")));
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn ticks_are_steps() {
        let mut app = App::new();
        app.on_load = OnLoad::Ticks(3);
        app.open(&test_data("bt_1791274820_cell_2robot.json"));
        // Steps land at 0, 0 and 4.73 s, where STAGE_R1 ends (the README's example).
        let now = app.panel.executor().unwrap().now();
        assert!((now - 4.7287).abs() < 1e-3, "{now}");
        assert_eq!(app.panel.executor().unwrap().ticks(), 3);
    }

    #[test]
    fn the_orchestrator_xml_runs_like_cell_runner() {
        let mut app = App::new();
        app.on_load = OnLoad::Run;
        app.open(&test_data("orchestrator_two_robots.xml"));
        let cell = app.panel.cell().expect("the cell leaves run");
        // cell-runner's own sample: 11.5 s (bt-cell's `orchestrator_sibling_compiles`).
        assert!((cell.schedule().duration - 11.5).abs() < 1e-9);
        assert!(cell.schedule().warnings.is_empty());
    }
}
