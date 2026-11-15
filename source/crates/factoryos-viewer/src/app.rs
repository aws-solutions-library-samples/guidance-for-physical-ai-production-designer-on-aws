//! The Physical AI Production Designer app: the Rerun Viewer wrapped with a "Designer Workflow" sidebar and the tools'
//! panels. Shared by the native (`main.rs`) and web (`web.rs`) entry points.
//!
//! One 3D view, in one of three explicit modes the top bar's chip names ([`view_mode`]): Edit (the
//! Robo Fitter's scene), Simulation (the Fitter's view of the cell step 1 simulates) and Live (the
//! live cell's own recording, step 5 Watch the cell).
//!
//! Based on rerun's `extend_viewer_ui` example.

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::{Arc, Mutex};

use factoryos_client::protocol::config::ConfigResponse;
use factoryos_client::protocol::{api, keys};
use web_time::{Duration, Instant};

use crate::rr::re_log_types::StoreId;
use crate::rr::re_ui::UiExt as _;
use crate::rr::re_viewer_context::{
    RecordingOrLocalTable, Route, SystemCommand, SystemCommandSender as _,
};
use crate::rr::{eframe, egui, re_log, re_log_channel, re_ui, re_viewer};

use crate::backend::{Backend, SharedBackend};
use crate::{bt_pane, chat, code_export, code_layout, config, dresser, fitter, hardware_ui, ide, infra_ui, isaac_ui, product, sidebar, task_editor, training_ui, theme, view_mode};

/// The app menu's About: opened in a new browser tab.
const ABOUT_URL: &str = "https://aws.amazon.com/manufacturing/";

/// What the main area shows.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Page {
    Home,
    BackendLogs,
    /// The Rerun viewer with the live cell's own recording (Watch the cell).
    LiveCell,
    /// The Rerun viewer with the product preview, plus the Product & Task panel.
    Product,
    /// The Rerun viewer with the dresser preview, plus the Robo Dresser panel.
    Dresser,
    /// The Rerun viewer with the fitted scene in 3D, plus (when opened as a tool, see
    /// [`fitter_panes`]) the Robo Fitter's Scene editor and panel.
    Fitter,
    /// The Rerun viewer with the cells a training job draws, plus the Build training job panel.
    Training,
}

/// What the native and web entry points hand [`new_app`].
pub struct Startup {
    pub backend: Backend,
    /// Log streams besides the backend's previews, e.g. the native gRPC server's.
    pub log_receivers: Vec<re_log_channel::LogReceiver>,
    pub main_thread_token: re_viewer::MainThreadToken,
}

/// Build the app once eframe has a window (native) or canvas (web).
pub fn new_app(cc: &eframe::CreationContext<'_>, startup: Startup) -> Result<Box<dyn eframe::App>, Box<dyn std::error::Error + Send + Sync>> {
    let Startup { backend, log_receivers, main_thread_token } = startup;
    theme::install_design_tokens();
    re_viewer::customize_eframe_and_setup_renderer(cc)?;
    theme::apply_style(&cc.egui_ctx);
    // The tools' recordings rarely need a timeline, whoever made their blueprint (an older
    // backend's previews don't say): hidden until the top bar's toggle shows it.
    re_viewer::hide_time_panel_by_default(true);

    let incoming_config = Arc::new(Mutex::new(None));
    {
        let rejoin_config = incoming_config.clone();
        let incoming_config = incoming_config.clone();
        backend
            .bus
            .subscribe(keys::CONFIG, move |payload| {
                if let Ok(settings) = serde_json::from_slice(&payload) {
                    *incoming_config.lock().unwrap() = Some(ConfigUpdate::Remote(settings));
                }
            })
            .map_err(|err| format!("subscribe {}: {err}", keys::CONFIG))?;
        // The initial config came over HTTP; a change published while the bus was away (a
        // restarted backend reads its file again) is queried after the re-join.
        let (bus, repaint) = (backend.bus.clone(), Arc::new(backend.repaint_handle()));
        backend.bus.on_rejoin(move || {
            let (incoming_config, repaint) = (rejoin_config.clone(), repaint.clone());
            bus.query(keys::CONFIG, None, move |reply| {
                if let Ok(Some(payload)) = reply
                    && let Ok(settings) = serde_json::from_slice(&payload)
                {
                    *incoming_config.lock().unwrap() = Some(ConfigUpdate::Remote(settings));
                    repaint();
                }
            });
            true
        });
    }

    let page = Rc::new(RefCell::new(Page::Home));
    // The tool whose preview Rerun shows right now (set every frame): its panes go into Rerun's
    // right panel then, and are side panels of their own otherwise — never both.
    let shown_tool: Rc<std::cell::Cell<Option<Page>>> = Rc::default();
    let cell = backend.shared.clone();
    let chat = Rc::new(RefCell::new(chat::Chats::default()));
    // The workspace switcher, the Workspace settings and the agent's sign-in (behind the edge).
    let workspaces = Rc::new(RefCell::new(crate::workspace_ui::Workspaces::new(
        backend.api.base(),
        backend.repaint_handle(),
    )));
    chat.borrow_mut().set_workspaces(workspaces.clone());
    // Shown once the user opens Agentic Planning's chat.
    let chat_open = Rc::new(std::cell::Cell::new(false));

    // Control Code Export: its form in the sidebar, its code page in the main area.
    let code_export = Rc::new(RefCell::new(code_export::CodeExport::new(backend.api.clone())));
    // The code page is shown in place of the 3D view (see `code_layout`); set every frame.
    let code_page = Rc::new(std::cell::Cell::new(false));
    // The Workspace IDE: in place of the 3D view too (the same `code_layout`), opened from the top bar.
    let ide = Rc::new(RefCell::new(ide::Ide::new(backend.api.clone())));
    // The IDE is shown (set every frame): Rerun's left panel and our panes are hidden, the chat stays.
    let ide_shown = Rc::new(std::cell::Cell::new(false));
    let ide_toggled = Rc::new(std::cell::Cell::new(false));
    // Rerun's welcome route doubles as the page for everything that isn't a recording.
    let welcome_screen = {
        let page = page.clone();
        let cell = cell.clone();
        let (code_export, code_page, ide) = (code_export.clone(), code_page.clone(), ide.clone());
        move |ui: &mut egui::Ui| {
            theme::with_controls(ui, |ui| {
                if code_page.get() && ide.borrow().open {
                    ide.borrow_mut().page_ui(ui);
                } else if code_page.get() {
                    code_export.borrow_mut().page_ui(ui);
                } else {
                    page_ui(ui, *page.borrow(), &cell);
                }
            });
        }
    };

    // The tools sidebar lives at the top of Rerun's left panel; clicks are handled in `FactoryOsApp::ui`.
    let pending_action = Rc::new(RefCell::new(None));

    // `FACTORY_OS_AUTOSTART=1`: start the simulation of the active cell and run its cell program
    // right away (for demos); the page stays as it is.
    let autostart = std::env::var_os("FACTORY_OS_AUTOSTART").is_some();
    if autostart {
        backend.start();
    }
    let config = Rc::new(RefCell::new(config::Config::from_response(backend.initial_config.clone())));
    config.borrow_mut().remote_backend = !factoryos_client::is_local(backend.api.base());
    let fitter = Rc::new(RefCell::new(fitter::Fitter::new(&backend)));
    // Whether the Planning Perspective waits for the simulation before it starts planning.
    let planning_waiting = Rc::new(std::cell::Cell::new(false));
    let product = Rc::new(RefCell::new(product::ProductPane::new(&backend)));
    // Hardware Configuration (its editor in a side panel) and the Virtual Cell (in the sidebar).
    let hardware = Rc::new(RefCell::new(hardware_ui::HardwareUi::new(backend.api.clone())));
    let infra = Rc::new(RefCell::new(infra_ui::InfraUi::new(backend.api.clone())));
    // The settings overlay (see `settings_overlay`), opened from the top bar's gear.
    let settings_open = Rc::new(std::cell::Cell::new(false));
    // "Watch in Isaac Sim" (step 5 and the Behaviour Tree pane); with none linked, it opens the settings.
    let isaac = Rc::new(RefCell::new(isaac_ui::IsaacSim::new(backend.api.clone(), settings_open.clone())));
    let tools_pane = {
        let isaac = isaac.clone();
        let code_export = code_export.clone();
        let hardware = hardware.clone();
        let infra = infra.clone();
        let planning_waiting = planning_waiting.clone();
        let pending_action = pending_action.clone();
        let cell = cell.clone();
        let fitter = fitter.clone();
        let product = product.clone();
        let form = RefCell::new(sidebar::SidebarForm::default());
        move |ui: &mut egui::Ui| {
            let (cells, error, agentic, plans, trajectory, plan_pending, plan_result, loaded_plan, cell_stream, live_cell, live_head, sim_cell) = {
                let cell = cell.lock().unwrap();
                // The cell the simulation runs (starting or up), not the one picked for the next Load & Play.
                let sim_cell = cell
                    .status
                    .cell
                    .clone()
                    .filter(|_| matches!(cell.status.state, crate::backend::BackendState::Starting | crate::backend::BackendState::Ready));
                (
                    cell.cells.clone(),
                    cell.cells_error.clone(),
                    cell.agentic.clone(),
                    cell.plans.clone(),
                    cell.cell_state.as_ref().map(|state| state["trajectory"].clone()),
                    cell.plan_in_flight,
                    cell.plan_result.clone(),
                    loaded_plan(&cell),
                    cell.recording_id.is_some(),
                    cell.live.as_ref().and_then(|live| live.cell.clone()),
                    cell.live_head.as_ref().map(|(head, at)| (head.clone(), at.elapsed().as_secs_f64())),
                    sim_cell,
                )
            };
            if let Some((_, age)) = &live_head {
                // Repaint when the head state turns stale, even without new data.
                let left = crate::flow::LIVE_HEAD_STALE_S - age;
                if left > 0.0 {
                    ui.ctx().request_repaint_after(Duration::from_secs_f64(left + 0.1));
                }
            }
            let live_head = crate::flow::LiveHead::new(live_head.as_ref().map(|(head, age)| (head, *age)));
            let linked_cell = fitter.borrow().linked_cell();
            let planning_waiting = planning_waiting.get();
            let current_product = product.borrow().current().map(|(name, _)| name);
            let runner = sidebar::CellRunner {
                cells: &cells,
                error: error.as_deref(),
                agentic: &agentic,
                plans: &plans,
                trajectory: trajectory.as_ref(),
                plan_pending,
                plan_result: plan_result.as_ref(),
                linked_cell: linked_cell.as_deref(),
                planning_waiting,
                product: current_product.as_deref(),
                loaded_plan: loaded_plan.as_deref(),
                cell_stream,
                live_cell: live_cell.as_deref(),
                live_head,
                sim_cell: sim_cell.as_deref(),
                weld_labels: fitter.borrow().weld_labels() && crate::scene3d::labels_shown(),
            };
            pane_ui(ui, "Designer Workflow", "Robotic cell design workflow.", |ui| {
                if let Some(action) = sidebar::ui(
                    ui,
                    &runner,
                    &mut form.borrow_mut(),
                    &mut code_export.borrow_mut(),
                    &mut hardware.borrow_mut(),
                    &mut infra.borrow_mut(),
                    &mut isaac.borrow_mut(),
                ) {
                    *pending_action.borrow_mut() = Some(action);
                }
            });
        }
    };
    let dresser = Rc::new(RefCell::new(dresser::Dresser::new(&backend)));
    // Product & Task: its pane in Rerun's right panel while its preview is shown.
    let on_product_page = {
        let (page, shown_tool) = (page.clone(), shown_tool.clone());
        Rc::new(move || *page.borrow() == Page::Product && shown_tool.get() == Some(Page::Product)) as Rc<dyn Fn() -> bool>
    };
    let product_pane = {
        let product = product.clone();
        let pending_action = pending_action.clone();
        move |ui: &mut egui::Ui| {
            pane_with_actions_ui(
                ui,
                "Product & Task",
                "The part to produce and its task (weld spots).",
                |ui| product.borrow_mut().actions_ui(ui),
                |ui| product_ui(ui, &product, &pending_action),
            );
        }
    };
    // The Robo Fitter's 3D view is shown (as the tool, or as the viewer of a behaviour tree the
    // cell plays); its Scene editor and panel only when the user opened the Fitter (see `fitter_panes`).
    let fitter_tool = Rc::new(std::cell::Cell::new(false));
    let on_fitter_page = {
        let (page, shown_tool) = (page.clone(), shown_tool.clone());
        Rc::new(move || *page.borrow() == Page::Fitter && shown_tool.get() == Some(Page::Fitter)) as Rc<dyn Fn() -> bool>
    };
    let on_fitter_tool = {
        let (page, shown_tool, fitter_tool) = (page.clone(), shown_tool.clone(), fitter_tool.clone());
        Rc::new(move || fitter_panes(*page.borrow(), shown_tool.get() == Some(Page::Fitter), fitter_tool.get()).in_right_panel)
            as Rc<dyn Fn() -> bool>
    };
    let fitter_palette_pane = {
        let fitter = fitter.clone();
        move |ui: &mut egui::Ui| {
            pane_ui(ui, FITTER_SCENES_TITLE, "The saved scenes to open, and the products, dressed robots, fixtures and floor plans to add to the scene.", |ui| {
                fitter.borrow_mut().palette_ui(ui);
            });
        }
    };
    let fitter_pane = {
        let fitter = fitter.clone();
        let product = product.clone();
        let pending_action = pending_action.clone();
        move |ui: &mut egui::Ui| {
            pane_with_actions_ui(
                ui,
                "Robo Fitter",
                "Scene, items, inspector, save and USDA export.",
                |ui| fitter.borrow_mut().actions_ui(ui),
                |ui| {
                    fitter.borrow_mut().ui(ui);
                    let scene_file = fitter.borrow().agent_scene_file();
                    agent_footer(ui, sidebar::AgentStep::CellLayout, &product, scene_file, &pending_action);
                },
            );
        }
    };
    // The active cell's task programs, next to its 3D view: opened from Agentic Planning ("Edit",
    // "Write Task Program"), closed with its ✕ (or by opening the Fitter itself).
    let task_editor = Rc::new(RefCell::new(task_editor::TaskEditor::new(backend.api.clone())));
    let task_editor_open = Rc::new(std::cell::Cell::new(false));
    let on_task_editor = {
        let on_fitter_page = on_fitter_page.clone();
        let task_editor_open = task_editor_open.clone();
        Rc::new(move || on_fitter_page() && task_editor_open.get()) as Rc<dyn Fn() -> bool>
    };
    let task_pane = {
        let task_editor = task_editor.clone();
        let task_editor_open = task_editor_open.clone();
        let cell = cell.clone();
        let pending_action = pending_action.clone();
        move |ui: &mut egui::Ui| {
            let plans = cell.lock().unwrap().plans.clone();
            let (title, hover) = ("Task Program", "The raw task program JSON: task programs come from the products.");
            let contents = |ui: &mut egui::Ui| {
                ui.horizontal(|ui| {
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.small_icon_button(&re_ui::icons::CLOSE, "Close the task program").clicked() {
                            task_editor_open.set(false);
                        }
                    });
                });
                let mut editor = task_editor.borrow_mut();
                if let Some(action) = editor.ui(ui, &plans) {
                    *pending_action.borrow_mut() = Some(action);
                }
                if let Some(listing) = editor.saved_listing.take() {
                    cell.lock().unwrap().plans = listing;
                }
            };
            if task_editor::TaskEditor::has_actions(&plans) {
                pane_with_actions_ui(ui, title, hover, |ui| task_editor.borrow_mut().actions_ui(ui, &plans), contents);
            } else {
                pane_ui(ui, title, hover, contents);
            }
        }
    };

    // The Robo Dresser's panes, stacked above Rerun's Selection view in the right panel.
    let on_dresser_page = {
        let (page, shown_tool) = (page.clone(), shown_tool.clone());
        Rc::new(move || *page.borrow() == Page::Dresser && shown_tool.get() == Some(Page::Dresser)) as Rc<dyn Fn() -> bool>
    };
    let catalogue_pane = {
        let dresser = dresser.clone();
        move |ui: &mut egui::Ui| {
            pane_ui(ui, "Catalogue", "Robots, tools and dressed robots.", |ui| {
                dresser.borrow_mut().catalogue_ui(ui);
            });
        }
    };
    let dresser_pane = {
        let dresser = dresser.clone();
        let product = product.clone();
        let pending_action = pending_action.clone();
        move |ui: &mut egui::Ui| {
            pane_with_actions_ui(
                ui,
                "Robo Dresser",
                "Dress options, output, diagnostics and joints.",
                |ui| dresser.borrow_mut().actions_ui(ui),
                |ui| {
                    dresser.borrow_mut().ui(ui);
                    agent_footer(ui, sidebar::AgentStep::Dressing, &product, None, &pending_action);
                },
            );
        }
    };

    // Build training job: its pane in Rerun's right panel while its 3D view is shown.
    let training = Rc::new(RefCell::new(training_ui::TrainingUi::new(&backend)));
    let on_training_page = {
        let (page, shown_tool) = (page.clone(), shown_tool.clone());
        Rc::new(move || *page.borrow() == Page::Training && shown_tool.get() == Some(Page::Training)) as Rc<dyn Fn() -> bool>
    };
    let training_pane = {
        let training = training.clone();
        move |ui: &mut egui::Ui| {
            pane_ui(ui, TRAINING_TITLE, TRAINING_HOVER, |ui| training.borrow_mut().ui(ui));
        }
    };

    let secrets = Rc::new(RefCell::new(config::Secrets::default()));
    #[cfg(not(target_arch = "wasm32"))]
    let backend_shown = (backend.api.base().to_owned(), backend.spawned());
    #[cfg(not(target_arch = "wasm32"))]
    let backend_chat = !factoryos_client::is_local(backend.api.base()) || crate::backend_choice::backend_chat();
    // On the web, where Rerun has no agent panel (nor its top-bar button), our own button toggles
    // the backend's planner chat (see `FactoryOsApp::ui`).
    let chat_toggled = Rc::new(std::cell::Cell::new(false));
    // What the 3D view shows (see `view_mode`), as a chip left of the gear; set every frame.
    let view_label: Rc<RefCell<Option<(String, egui::Color32)>>> = Rc::default();
    let settings_button = {
        let settings_open = settings_open.clone();
        let workspaces = workspaces.clone();
        let view_label = view_label.clone();
        let (ide_shown, ide_toggled) = (ide_shown.clone(), ide_toggled.clone());
        #[cfg(target_arch = "wasm32")]
        let (chat_open, chat_toggled) = (chat_open.clone(), chat_toggled.clone());
        move |ui: &mut egui::Ui| {
            workspaces.borrow_mut().top_bar_ui(ui);
            theme::toggle_button(ui);
            let mut open = settings_open.get();
            ui.medium_icon_toggle_button(&re_ui::icons::SETTINGS, "Settings", &mut open)
                .on_hover_text("Physical AI Production Designer settings: backend, tools and their paths");
            settings_open.set(open);
            if let Some((text, fill)) = view_label.borrow().as_ref() {
                view_chip(ui, text, *fill);
            }
            #[cfg(target_arch = "wasm32")]
            {
                let mut open = chat_open.get();
                if ui
                    .medium_icon_toggle_button(&re_ui::icons::AGENT, "Planner chat", &mut open)
                    .on_hover_text("The Kiro planner chat, run by the backend")
                    .clicked()
                {
                    chat_toggled.set(true);
                }
            }
            // Last: leftmost of ours, next to the agent button (laid out right to left).
            let mut open = ide_shown.get();
            if ui
                .medium_icon_toggle_button(&re_ui::icons::VIEW_TEXT, "Workspace IDE", &mut open)
                .on_hover_text("Workspace IDE: edit the workspace's files (the planner chat stays beside it)")
                .clicked()
            {
                ide_toggled.set(true);
            }
            ui.add_space(6.0);
            crate::ui_mode::toggle_ui(ui);
        }
    };

    // The behaviour tree the cell plays, under the 3D view (see `bt_pane`).
    let bt_pane = Rc::new(RefCell::new(bt_pane::BtPane::new(backend.api.clone(), isaac)));
    let outer_panels = Rc::new(std::cell::Cell::new(OuterPanels::default()));
    let below_top_bar = Rc::new(std::cell::Cell::new(None));
    // Rerun's right panel (the top bar's right-panel button), read every frame (`logic`): the
    // behaviour-tree pane beside the 3D view follows it.
    let right_panel_open = Rc::new(std::cell::Cell::new(true));
    let outer_panels_hook = {
        let panels = OuterPanelState {
            shown: outer_panels.clone(),
            below_top_bar: below_top_bar.clone(),
            chat: chat.clone(),
            chat_open: chat_open.clone(),
            dresser: dresser.clone(),
            training: training.clone(),
            product: product.clone(),
            pending_action: pending_action.clone(),
            fitter: fitter.clone(),
            cell: cell.clone(),
            bt_pane,
            hardware: hardware.clone(),
            ide_shown: ide_shown.clone(),
            right_panel_open: right_panel_open.clone(),
        };
        move |ui: &mut egui::Ui| outer_panels_ui(ui, &panels)
    };

    let mut startup_options = re_viewer::StartupOptions {
        app_menu_title: Some("Physical AI Production Designer on AWS".to_owned()),
        // The title's dropdown: only About, AWS for manufacturing (not Rerun's menu, its About overlay).
        app_menu: Some(Rc::new(|ui: &mut egui::Ui| {
            if ui.button("About").on_hover_text(ABOUT_URL).clicked() {
                crate::workspace_ui::open_url(ABOUT_URL);
                ui.close();
            }
        })),
        custom_welcome_screen: Some(Rc::new(welcome_screen)),
        left_panel_sections: vec![
            re_viewer::CustomLeftPanelSection {
                id: "factory_os_tools",
                default_height_fraction: 0.5,
                ui: Rc::new(tools_pane),
                is_visible: None,
            },
        ],
        right_panel_sections: vec![
            re_viewer::CustomLeftPanelSection {
                id: "robo_fitter_palette",
                default_height_fraction: 0.35,
                ui: Rc::new(fitter_palette_pane),
                is_visible: Some(on_fitter_tool.clone()),
            },
            re_viewer::CustomLeftPanelSection {
                id: "robo_fitter_inspector",
                default_height_fraction: 0.45,
                ui: Rc::new(fitter_pane),
                is_visible: Some(on_fitter_tool),
            },
            re_viewer::CustomLeftPanelSection {
                id: "task_program",
                default_height_fraction: 0.4,
                ui: Rc::new(task_pane),
                is_visible: Some(on_task_editor),
            },
            re_viewer::CustomLeftPanelSection {
                id: "product_task",
                default_height_fraction: 1.0,
                ui: Rc::new(product_pane),
                is_visible: Some(on_product_page),
            },
            re_viewer::CustomLeftPanelSection {
                id: "robo_dresser_catalogue",
                default_height_fraction: 0.35,
                ui: Rc::new(catalogue_pane),
                is_visible: Some(on_dresser_page.clone()),
            },
            re_viewer::CustomLeftPanelSection {
                id: "robo_dresser_inspector",
                default_height_fraction: 0.45,
                ui: Rc::new(dresser_pane),
                is_visible: Some(on_dresser_page),
            },
            re_viewer::CustomLeftPanelSection {
                id: "training_job",
                default_height_fraction: 1.0,
                ui: Rc::new(training_pane),
                is_visible: Some(on_training_page),
            },
        ],
        outer_panels: Some(Rc::new(outer_panels_hook)),
        top_bar_buttons: Some(Rc::new(settings_button)),
        left_panel_hidden: Some({
            let ide_shown = ide_shown.clone();
            Rc::new(move || ide_shown.get())
        }),
        // Designer Workflow gets the left panel's height, the tool panes the right one's; Sources and
        // Selection are bars at the bottom, opened by a click.
        compact_sources_panel: true,
        compact_selection_panel: true,
        // Hides the "Welcome to rerun" entry in the Sources panel; our pages replace it.
        hide_welcome_screen: true,
        ..without_rerun_services(Default::default())
    };
    // A live view: every scene is static, so a timeline has nothing to show or play back. Hidden,
    // and its toggle disabled (an override wins over every blueprint).
    startup_options.panel_state_overrides.time = Some(crate::rr::re_sdk_types::blueprint::components::PanelState::Hidden);

    // This is used for analytics, if the `analytics` feature is on in `Cargo.toml`
    let app_env = re_viewer::AppEnvironment::Custom("Factory OS".to_owned());

    backend.set_egui_ctx(&cc.egui_ctx);
    dresser.borrow().stream.set_egui_ctx(&cc.egui_ctx);
    training.borrow().stream.set_egui_ctx(&cc.egui_ctx);
    product.borrow().stream.set_egui_ctx(&cc.egui_ctx);
    fitter.borrow().stream.set_egui_ctx(&cc.egui_ctx);
    crate::live_queue::watch_page_visibility(&cc.egui_ctx);
    // `FACTORY_OS_PAGE=dresser` opens the Robo Dresser (`fitter`, `training`: those; `code`: the
    // Control Code Export page); `FACTORY_OS_DRESSER` runs one of its actions.
    match std::env::var("FACTORY_OS_PAGE").as_deref() {
        Ok("dresser") => *pending_action.borrow_mut() = Some(sidebar::Action::OpenDresser),
        Ok("fitter") => *pending_action.borrow_mut() = Some(sidebar::Action::OpenFitter),
        Ok("training") => *pending_action.borrow_mut() = Some(sidebar::Action::OpenTrainingJob),
        Ok("code") => code_export.borrow_mut().open = true,
        Ok("ide") => ide.borrow_mut().open = true,
        _ => {}
    }

    // `FACTORY_OS_AUTOMATION=stdin` (natively) or `?automation=1` (web): commands on stdin or from
    // the page (see `automation`); else `None`.
    let automation = crate::automation::Automation::from_env(&cc.egui_ctx);

    let mut rerun_app = re_viewer::App::new(
        main_thread_token,
        re_viewer::build_info(),
        app_env,
        startup_options,
        cc,
        None,
        re_viewer::AsyncRuntimeHandle::from_current_tokio_runtime_or_wasmbindgen()?,
    );
    // Agentic Planning needs the agent panel's top-bar button natively: it opens Rerun's agent
    // panel (kiro-cli here) or, with the backend's chat, ours. Rerun keeps the panel behind an
    // experimental option that is off by default (and in saved settings), hiding the button.
    #[cfg(not(target_arch = "wasm32"))]
    {
        rerun_app.app_options_mut().experimental.agent_panel = true;
    }
    raise_memory_limit(rerun_app.app_options_mut());
    // Ours, over Rerun's (which it may reset to System on an upgrade).
    theme::set_preference(&cc.egui_ctx, theme::load_preference(cc.storage));
    crate::ui_mode::set(&cc.egui_ctx, crate::ui_mode::load(cc.storage));
    crate::scene3d::set_labels_shown(crate::labels_overlay::load(cc.storage));
    for receiver in log_receivers {
        rerun_app.add_log_receiver(receiver);
    }
    // The tools' 3D scenes. Not with a backend of another protocol (see `backend::ProtocolCheck`):
    // their streams are dropped here, and the banner says why.
    let protocol_banner = backend.protocol.banner();
    for receiver in [
        product.borrow_mut().stream.take_receiver(),
        dresser.borrow_mut().stream.take_receiver(),
        training.borrow_mut().stream.take_receiver(),
        fitter.borrow_mut().stream.take_receiver(),
    ]
        .into_iter()
        .flatten()
        .filter(|_| protocol_banner.is_none())
    {
        rerun_app.add_log_receiver(receiver);
    }
    // A lost or restarted backend, picked up again without a page reload (see `reconnect`).
    let reconnect = crate::reconnect::Reconnect::new(&backend.api, &backend.bus, &backend.boot_id);
    Ok(Box::new(FactoryOsApp {
        reconnect,
        restarts_seen: 0,
        egui_ctx: cc.egui_ctx.clone(),
        rerun_app,
        page,
        pending_action,
        config,
        incoming_config,
        secrets,
        synced_secrets: config::Secrets::default(),
        backend,
        cell,
        dresser,
        training,
        product,
        fitter,
        code_export,
        code_layout: code_layout::CodeLayout::default(),
        code_page,
        ide,
        ide_shown,
        right_panel_open,
        ide_toggled,
        hardware,
        infra,
        task_editor,
        task_editor_open,
        fitter_tool,
        preview_route: None,
        dresser_command_run: false,
        fitter_command_run: false,
        fitter_scenes: Vec::new(),
        open_cell_recording: false,
        last_route_request: None,
        autostart_program: autostart,
        plans_listed: ago(Duration::from_secs(60)),
        keep_fitter_view_until: None,
        #[cfg(not(target_arch = "wasm32"))]
        planning_kickoff: None,
        planning_waiting,
        chat,
        chat_open,
        chat_toggled,
        workspaces,
        // Under automation, no planner chat (nor the kiro-cli it starts) unless asked for.
        chat_opened_at_start: automation.is_some(),
        #[cfg(not(target_arch = "wasm32"))]
        started: Instant::now(),
        outer_panels,
        tracked_recording: None,
        followed_live: Vec::new(),
        shown_tool,
        settings_open,
        below_top_bar,
        view_label,
        confirm: None,
        #[cfg(not(target_arch = "wasm32"))]
        backend_shown,
        #[cfg(not(target_arch = "wasm32"))]
        backend_chat,
        protocol_banner,
        automation,
    }))
}

struct FactoryOsApp {
    /// For saving the theme choice (`save` has no context).
    egui_ctx: egui::Context,
    rerun_app: re_viewer::App,
    page: Rc<RefCell<Page>>,
    pending_action: Rc<RefCell<Option<sidebar::Action>>>,
    config: Rc<RefCell<config::Config>>,
    /// Config replies and updates from the backend, applied at the start of the next frame.
    incoming_config: Arc<Mutex<Option<ConfigUpdate>>>,
    secrets: Rc<RefCell<config::Secrets>>,
    /// The passwords as last sent to the backend.
    synced_secrets: config::Secrets,
    backend: Backend,
    cell: SharedBackend,
    dresser: Rc<RefCell<dresser::Dresser>>,
    /// Build training job (see `training_ui`).
    training: Rc<RefCell<training_ui::TrainingUi>>,
    product: Rc<RefCell<product::ProductPane>>,
    fitter: Rc<RefCell<fitter::Fitter>>,
    /// Control Code Export (see `code_export`).
    code_export: Rc<RefCell<code_export::CodeExport>>,
    /// The code page in place of the 3D view (see `code_layout`), and whether it is shown.
    code_layout: code_layout::CodeLayout<StoreId>,
    code_page: Rc<std::cell::Cell<bool>>,
    /// The Workspace IDE (see `ide`), shown through `code_layout` like the code page.
    ide: Rc<RefCell<ide::Ide>>,
    /// The IDE is what the main area shows (set in `sync_code_layout`).
    ide_shown: Rc<std::cell::Cell<bool>>,
    /// Rerun's right panel is expanded (`read_right_panel`): the behaviour-tree pane follows it.
    right_panel_open: Rc<std::cell::Cell<bool>>,
    /// The top bar's IDE button was clicked.
    ide_toggled: Rc<std::cell::Cell<bool>>,
    /// Hardware Configuration (see `hardware_ui`).
    hardware: Rc<RefCell<hardware_ui::HardwareUi>>,
    /// The Virtual Cell (see `infra_ui`).
    infra: Rc<RefCell<infra_ui::InfraUi>>,
    task_editor: Rc<RefCell<task_editor::TaskEditor>>,
    /// The Task Program pane is open (see `task_pane`).
    task_editor_open: Rc<std::cell::Cell<bool>>,
    /// The user opened the Robo Fitter (its sidebar box, Open Cell, a task program), rather than
    /// the Fitter page only showing the cell play a behaviour tree (see `fitter_panes`).
    fitter_tool: Rc<std::cell::Cell<bool>>,
    /// Keep routing to this dresser/fitter preview until the viewer shows it.
    preview_route: Option<(Page, StoreId, Instant)>,
    dresser_command_run: bool,
    fitter_command_run: bool,
    /// The Fitter's saved scenes, to re-list the cells (the scenes) when they change.
    fitter_scenes: Vec<String>,

    /// Keep routing to the live cell's recording until the viewer has received it.
    open_cell_recording: bool,
    last_route_request: Option<Instant>,

    /// Set the robots ready and start the program once the simulation is connected
    /// (`FACTORY_OS_AUTOSTART`).
    autostart_program: bool,

    /// When the behaviour trees were last listed (they are re-listed every few seconds).
    plans_listed: Instant,
    /// A cell started from the Fitter page (e.g. to run a behaviour tree): until then, keep showing
    /// the Fitter's 3D view when the cell's own 3D stream arrives.
    keep_fitter_view_until: Option<Instant>,
    /// Planning Perspective: the prompt to send the planner agent once it and the simulation are up.
    #[cfg(not(target_arch = "wasm32"))]
    planning_kickoff: Option<PlanningKickoff>,
    /// Shared with the sidebar: the kick-off waits for the simulation.
    planning_waiting: Rc<std::cell::Cell<bool>>,
    /// The planner chat pane, for a planner in the backend (see `open_agent_chat`).
    chat: Rc<RefCell<chat::Chats>>,
    /// Whether the chat pane is shown.
    chat_open: Rc<std::cell::Cell<bool>>,
    /// The top bar's chat button was clicked (web only; natively Rerun's agent button does it).
    chat_toggled: Rc<std::cell::Cell<bool>>,
    /// See [`crate::workspace_ui`].
    workspaces: Rc<RefCell<crate::workspace_ui::Workspaces>>,
    /// The planner chat (Agentic Planning) is opened with the viewer, once.
    chat_opened_at_start: bool,
    #[cfg(not(target_arch = "wasm32"))]
    started: Instant,
    /// The recording the page last followed (see `track_cell_recording`).
    tracked_recording: Option<StoreId>,
    /// Live recordings set to follow the latest time since they were shown (see
    /// `follow_live_recordings`).
    followed_live: Vec<StoreId>,
    /// The tool whose preview Rerun shows (see `tool_page`), for where its panes go.
    shown_tool: Rc<std::cell::Cell<Option<Page>>>,
    /// Which of our side panels Rerun draws this frame (see `outer_panels_ui`).
    outer_panels: Rc<std::cell::Cell<OuterPanels>>,
    /// The settings overlay is open (toggled by the top bar's gear).
    settings_open: Rc<std::cell::Cell<bool>>,
    /// Where Rerun's top bar ends: the area the settings overlay covers.
    below_top_bar: Rc<std::cell::Cell<Option<egui::Rect>>>,
    /// What the 3D view shows (see `view_mode`), as the top bar's chip; set every frame.
    view_label: Rc<RefCell<Option<(String, egui::Color32)>>>,
    /// A confirmation the user has to answer first (see `confirm_ui`).
    confirm: Option<Confirm>,
    /// The backend's address, and whether this viewer started it, for the settings.
    #[cfg(not(target_arch = "wasm32"))]
    backend_shown: (String, bool),
    /// The planner chat is the backend's (our chat pane), not Rerun's agent panel: always with a
    /// backend on another machine, and with a local one if chosen (see `backend_choice`).
    #[cfg(not(target_arch = "wasm32"))]
    backend_chat: bool,
    /// The backend speaks another protocol: what to update (shown until then; no 3D).
    protocol_banner: Option<String>,
    /// Commands on stdin with `FACTORY_OS_AUTOMATION=stdin`, or from the page with `?automation=1`
    /// (see `automation`).
    automation: Option<crate::automation::Automation>,
    /// Notices a backend that went away or restarted (see `on_backend_restart`).
    reconnect: crate::reconnect::Reconnect,
    /// The restarts handled so far (`reconnect.restarts()`).
    restarts_seen: u64,
}

impl eframe::App for FactoryOsApp {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        // Store viewer state on disk
        self.rerun_app.save(storage);
        theme::save_preference(storage, &self.egui_ctx);
        crate::ui_mode::save(storage, &self.egui_ctx);
        crate::labels_overlay::save(storage);
    }

    /// Called whenever we need repainting, which could be 60 Hz.
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        theme::begin_frame(ui.ctx());
        self.automation_commands(ui.ctx());
        self.workspaces.borrow_mut().poll(ui.ctx());
        // Agentic Planning is where most work starts: its chat is open from the beginning.
        if !std::mem::replace(&mut self.chat_opened_at_start, true) {
            self.open_agent_chat(None, false);
        }
        self.sync_config();
        self.poll_backend();
        self.reconnect.poll(ui.ctx());
        let restarts = self.reconnect.restarts();
        if restarts != self.restarts_seen {
            self.restarts_seen = restarts;
            self.on_backend_restart();
        }
        if *self.page.borrow() == Page::Fitter {
            self.fitter.borrow_mut().sync_3d();
        }
        if self.plans_listed.elapsed() > Duration::from_secs(3) {
            self.plans_listed = Instant::now();
            self.backend.refresh_plans();
        }
        {
            let (status, cell, infra) = {
                let shared = self.cell.lock().unwrap();
                (shared.codegen.clone(), shared.plans.cell.clone(), shared.infra.clone())
            };
            self.code_export.borrow_mut().poll(ui.ctx(), &status, &cell);
            // The hardware of the export's plan (its robots), else of the plan the form shows.
            let plan = self.code_export.borrow().hardware_plan();
            self.hardware.borrow_mut().poll(ui.ctx(), &cell, plan.as_deref());
            if self.hardware.borrow_mut().take_saved() {
                // Saved after an export: the builds' STALE flags change.
                self.code_export.borrow().refresh(ui.ctx());
            }
            self.infra.borrow_mut().poll(ui.ctx(), infra.as_ref());
        }
        if self.infra.borrow_mut().take_reached_production() {
            self.watch_cell_in_production();
        }
        self.drive_planning_chat();
        // "Ask the agent about this file" on the code page: a chat of its own.
        let ask = self.code_export.borrow_mut().take_chat_request();
        if let Some(request) = ask {
            self.open_agent_chat(Some(request), false);
        }
        if self.ide_toggled.take() {
            let mut ide = self.ide.borrow_mut();
            ide.open = !self.ide_shown.get();
            if ide.open {
                self.code_export.borrow_mut().open = false;
            }
        }
        if self.chat_toggled.take() {
            if self.chat_open.get() {
                self.chat_open.set(false);
            } else {
                self.open_backend_chat(None);
            }
        }
        // With the backend's chat, Rerun's agent panel (reopened from the last session, or by its
        // top-bar button) would run a second kiro-cli here, inside Rerun's layout: show ours instead.
        #[cfg(not(target_arch = "wasm32"))]
        if self.backend_chat && self.rerun_app.agent_panel_open() {
            self.rerun_app.toggle_agent_panel();
            // Its button toggles: it closes our chat when that is open. Open right after the start,
            // it was restored from the last session, not clicked: ours stays open then.
            let restored = self.started.elapsed() < Duration::from_secs(3);
            if self.chat_open.get() && !restored {
                self.chat_open.set(false);
            } else {
                self.open_backend_chat(None);
            }
        }
        self.keep_fitter_view();
        let fitter_scenes = self.fitter.borrow().saved_scenes();
        if fitter_scenes != self.fitter_scenes {
            // A scene was saved (or removed) in the Fitter: re-list the fitted scenes.
            self.fitter_scenes = fitter_scenes;
            self.backend.cells_refresher()();
        }
        if *self.page.borrow() == Page::Product {
            self.product.borrow_mut().poll();
        }
        self.route_to_previews();
        self.track_cell_recording();
        self.route_to_live_recording();
        self.sync_code_layout();

        // Our side panels are drawn by Rerun right after its top bar (`outer_panels`), so they sit
        // below it at the window's right edge; which ones depends on what Rerun shows right now.
        //
        // Normally the dresser, fitter and cell panes live in Rerun's right panel (see
        // `right_panel_sections`), which is only drawn while a recording is shown — so before the
        // first preview, they are side panels of their own.
        let active = self.rerun_app.active_recording_id().cloned();
        let page = *self.page.borrow();
        let shown_tool = active.as_ref().and_then(tool_page);
        self.shown_tool.set(shown_tool);
        let showing_dresser = shown_tool == Some(Page::Dresser);
        let showing_product = shown_tool == Some(Page::Product);
        let showing_fitter = shown_tool == Some(Page::Fitter);
        let showing_training = shown_tool == Some(Page::Training);
        let showing_cell = {
            let cell_recording = self.cell.lock().unwrap().recording_id.clone();
            cell_recording.is_some() && active == cell_recording
        };
        let panels = OuterPanels {
            dresser: page == Page::Dresser && !showing_dresser,
            training: page == Page::Training && !showing_training,
            product: page == Page::Product && !showing_product,
            fitter: fitter_panes(page, showing_fitter, self.fitter_tool.get()).own_panel,
            bt: page == Page::Fitter,
        };
        let code_page = self.code_layout.active();
        self.outer_panels.set(shown_panels(panels, code_page));
        *self.view_label.borrow_mut() = if self.ide_shown.get() {
            Some(("</> Workspace IDE".to_owned(), view_mode::EDIT))
        } else if code_page {
            Some(("</> Control code · read-only".to_owned(), view_mode::READ_ONLY))
        } else {
            view_mode::label(&self.view_mode(page, showing_cell))
        };

        // Now show the Rerun Viewer in the remaining space:
        if !self.ide_shown.get() {
            consume_recording_shortcuts(ui.ctx());
        }
        self.rerun_app.ui(ui, frame);
        self.labels_overlay(ui.ctx(), !code_page && !self.ide_shown.get() && (shown_tool.is_some() || showing_cell));
        if let Some(banner) = &self.protocol_banner {
            protocol_banner_ui(ui.ctx(), banner, self.below_top_bar.get());
        } else if self.reconnect.reconnecting() {
            crate::reconnect::notice_ui(ui.ctx(), self.below_top_bar.get());
        }
        self.settings_overlay(ui.ctx());
        self.confirm_ui(ui.ctx());


        let action = self.pending_action.borrow_mut().take();
        if let Some(action) = action {
            self.handle_action(action);
        }
        if self.automation.as_ref().is_some_and(|automation| !automation.waits.is_empty()) {
            let busy = self.automation_busy();
            if let Some(automation) = &mut self.automation {
                automation.step_waits(ui.ctx(), busy);
            }
        }
    }

    /// Under automation, the real pointer is ignored (see `automation::without_pointer`).
    fn raw_input_hook(&mut self, _ctx: &egui::Context, raw_input: &mut egui::RawInput) {
        if let Some(automation) = &mut self.automation {
            automation.filter_input(raw_input);
        }
    }

    fn logic(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        self.read_right_panel(ctx);
        self.apply_view_commands();
        self.forward_live_streams(ctx);
        self.sync_view_eyes();
        self.follow_live_recordings();
        self.rerun_app.logic(ctx, frame);
    }

    fn on_exit(&mut self) {
        // A backend we started stops the cell-runner / rerun-viewer processes on its way out.
        self.backend.shutdown();
    }
}

impl FactoryOsApp {
    /// The backend restarted (another boot id; see `reconnect`): it starts with none of the
    /// tools open, and may keep its files elsewhere. The bus re-queries its followed states and
    /// the scenes' snapshots when it re-joins; here, what only a request re-establishes: the
    /// tools open here are opened again, the storage mode is asked again, failed assets are
    /// fetched again, the cells and plans listed again, and every scene asks for its snapshot.
    fn on_backend_restart(&mut self) {
        re_log::info!("factoryos-backend restarted: opening the tools again");
        factoryos_client::DataSource::forget_modes();
        self.backend.assets.retry_failed();
        let page = *self.page.borrow();
        self.fitter.borrow_mut().backend_restarted();
        self.dresser.borrow_mut().backend_restarted();
        match page {
            Page::Product => {
                let mut product = self.product.borrow_mut();
                product.open();
                product.stream.resync();
            }
            Page::Training => {
                let mut training = self.training.borrow_mut();
                training.open();
                training.stream.resync();
            }
            _ => {}
        }
        if let Some(scene) = &self.cell.lock().unwrap().cell_scene {
            scene.resync();
        }
        self.backend.cells_refresher()();
        self.backend.refresh_plans();
    }

    /// Send config and password edits to the backend, handle "Reload", and apply remote changes.
    fn sync_config(&mut self) {
        let mut config = self.config.borrow_mut();
        match self.incoming_config.lock().unwrap().take() {
            Some(ConfigUpdate::Response(response)) => config.apply_response(response),
            Some(ConfigUpdate::Remote(settings)) => {
                config.apply_remote(settings);
                self.backend.cells_refresher()(); // the cells folder or active cell may have changed
            }
            Some(ConfigUpdate::FileError(error)) => config.file_error = error,
            None => {}
        }
        if std::mem::take(&mut config.reload_requested) {
            let incoming = self.incoming_config.clone();
            self.backend.api.post(api::CONFIG, move |response: Result<ConfigResponse, String>| {
                *incoming.lock().unwrap() = Some(match response {
                    Ok(response) => ConfigUpdate::Response(response),
                    Err(err) => ConfigUpdate::FileError(Some(format!("Reload failed: {err}"))),
                });
            });
        }
        if let Some(settings) = config.take_changes() {
            let incoming = self.incoming_config.clone();
            let refresh_cells = self.backend.cells_refresher();
            self.backend.api.put_json(api::CONFIG, &settings, move |response: Result<ConfigResponse, String>| {
                refresh_cells();
                let error = match response {
                    Ok(response) => response.file_error,
                    Err(err) => Some(format!("Failed to save config: {err}")),
                };
                *incoming.lock().unwrap() = Some(ConfigUpdate::FileError(error));
            });
        }
        let secrets = self.secrets.borrow().clone();
        if secrets != self.synced_secrets {
            self.backend.api.put_json(api::SECRETS, &secrets.to_update(), |response: Result<(), String>| {
                if let Err(err) = response {
                    re_log::warn!("Failed to send passwords to the backend: {err}");
                }
            });
            self.synced_secrets = secrets;
        }
        let catalogue = config.resolve(&config.catalogue);
        drop(config);
        if *self.page.borrow() == Page::Dresser {
            self.dresser.borrow_mut().open(&catalogue); // Rescans only when the directory changed.
        }
    }

    fn poll_backend(&mut self) {
        self.backend.flush_commands();
        let mut backend = self.cell.lock().unwrap();
        if let Some(rx) = backend.new_receiver.take() {
            self.rerun_app.add_log_receiver(rx);
            if *self.page.borrow() == Page::Fitter {
                self.keep_fitter_view_until = Some(Instant::now() + Duration::from_secs(10));
            }
        }
        if self.autostart_program && backend.connected {
            self.autostart_program = false;
            backend.command("set_all_ready", None);
            backend.command("start", None);
        }
    }

    /// Hand the backend streams' decoded messages to Rerun, which ingests them in this frame's
    /// `logic`. They wait (and coalesce to the latest state) while Rerun is still busy with the last
    /// batch, see [`crate::live_queue`].
    fn forward_live_streams(&self, ctx: &egui::Context) {
        // A new scene (the live cell's, a tool's) is framed again once it is shown (`Scene3d::set_shown`).
        let active = self.rerun_app.active_recording_id();
        let mut waiting = self.cell.lock().unwrap().forward_stream(active);
        for stream in [&self.product.borrow().stream, &self.dresser.borrow().stream, &self.training.borrow().stream, &self.fitter.borrow().stream] {
            stream.set_active_recording(active);
        }
        waiting |= self.product.borrow().stream.forward();
        waiting |= self.dresser.borrow().stream.forward();
        waiting |= self.training.borrow().stream.forward();
        waiting |= self.fitter.borrow().stream.forward();
        if waiting {
            ctx.request_repaint();
        }
    }

    /// Keep each scene's 3D view on the eye fitted to it until the user moves it
    /// (`Scene3d::sync_eye`): written into Rerun's active blueprint, which this frame's `logic`
    /// applies.
    fn sync_view_eyes(&self) {
        let Some(hub) = self.rerun_app.store_hub() else { return };
        let mut appends = Vec::new();
        if let Some(scene) = &self.cell.lock().unwrap().cell_scene {
            appends.extend(scene.sync_eye(hub));
        }
        for stream in [&self.product.borrow().stream, &self.dresser.borrow().stream, &self.training.borrow().stream, &self.fitter.borrow().stream] {
            appends.extend(stream.sync_eye(hub));
        }
        for (store_id, chunks) in appends {
            self.rerun_app.command_sender.send_system(SystemCommand::AppendToStore(store_id, chunks));
        }
    }

    /// Keep live recordings (the cell, the Fitter's) on the latest time: when one is first shown,
    /// and again whenever the page becomes visible after being hidden, so nothing plays back at 1x
    /// from where the time cursor stood.
    fn follow_live_recordings(&mut self) {
        use crate::rr::re_sdk_types::blueprint::components::PlayState;
        use crate::rr::re_viewer_context::TimeControlCommand;

        if crate::live_queue::take_page_shown() {
            self.followed_live.clear();
        }
        let Some(active) = self.rerun_app.active_recording_id().cloned() else {
            return;
        };
        let live = [self.cell.lock().unwrap().recording_id.clone(), self.fitter.borrow().preview_recording()];
        if !live.contains(&Some(active.clone())) || self.followed_live.contains(&active) {
            return;
        }
        self.followed_live.push(active.clone());
        self.rerun_app.command_sender.send_system(SystemCommand::TimeControlCommands {
            store_id: active,
            time_commands: vec![TimeControlCommand::SetPlayState(PlayState::Following)],
        });
    }

    /// Show the Cell Viewer when the user opens the cell recording from the Sources panel.
    /// Stay on the Fitter's 3D view while a cell started from it brings up its own 3D stream.
    fn keep_fitter_view(&mut self) {
        let Some(until) = self.keep_fitter_view_until else {
            return;
        };
        if Instant::now() > until {
            self.keep_fitter_view_until = None;
            return;
        }
        let cell_recording = self.cell.lock().unwrap().recording_id.clone();
        let Some(fitter_recording) = self.fitter.borrow().preview_recording() else {
            return;
        };
        if cell_recording.is_some() && self.rerun_app.active_recording_id() == cell_recording.as_ref() {
            *self.page.borrow_mut() = Page::Fitter;
            self.rerun_app
                .command_sender
                .send_system(SystemCommand::SetRoute(Route::LocalRecording { recording_id: fitter_recording }));
        }
    }

    fn track_cell_recording(&mut self) {
        // `active_recording_id` follows the route, so this is `None` while a placeholder page is shown.
        let Some(active_id) = self.rerun_app.active_recording_id().cloned() else {
            return;
        };
        // Follow Rerun only when it shows another recording (e.g. picked in Sources), and not
        // while we are routing to one ourselves: until it arrives, Rerun still shows the last.
        if self.tracked_recording.as_ref() == Some(&active_id) {
            return;
        }
        if self.preview_route.is_some() || self.open_cell_recording {
            return;
        }
        self.tracked_recording = Some(active_id.clone());
        let active_id = &active_id;
        if self.cell.lock().unwrap().recording_id.as_ref() == Some(active_id) {
            *self.page.borrow_mut() = Page::LiveCell;
        } else if let Some(page) = tool_page(active_id) {
            // By application, not recording id: a tool's recording from before a backend restart
            // is still that tool's.
            *self.page.borrow_mut() = page;
        }
    }

    /// Keep routing to the live cell's recording (Watch the cell) until Rerun shows it. Close the
    /// ones a new live scene replaced; if one of them is shown, the new one is shown instead.
    fn route_to_live_recording(&mut self) {
        let replaced = std::mem::take(&mut self.cell.lock().unwrap().replaced_recordings);
        for store_id in replaced {
            if let Some(current) = self.cell.lock().unwrap().recording_id.clone() {
                self.code_layout.replace(&store_id, current);
            }
            if self.rerun_app.active_recording_id() == Some(&store_id) {
                self.open_live_cell();
            }
            self.automation_closed(&store_id);
            self.rerun_app
                .command_sender
                .send_system(SystemCommand::CloseRecordingOrTable(RecordingOrLocalTable::Recording { store_id }));
        }
        if self.code_layout.active() {
            return; // routed to once the code page is closed
        }
        if !self.open_cell_recording || *self.page.borrow() != Page::LiveCell {
            self.open_cell_recording = false;
            return;
        }
        let Some(recording_id) = self.cell.lock().unwrap().recording_id.clone() else {
            return; // its stream has not arrived yet
        };
        if self.rerun_app.active_recording_id() == Some(&recording_id) {
            self.open_cell_recording = false;
            return;
        }
        let due = self
            .last_route_request
            .is_none_or(|t| t.elapsed() > Duration::from_millis(250));
        if due {
            self.last_route_request = Some(Instant::now());
            self.rerun_app
                .command_sender
                .send_system(SystemCommand::SetRoute(Route::LocalRecording { recording_id }));
        }
    }

    /// Close replaced product/dresser/fitter previews, and keep routing to the current one until shown.
    fn route_to_previews(&mut self) {
        let requests = [
            (Page::Product, self.product.borrow_mut().take_route_request()),
            (Page::Dresser, self.dresser.borrow_mut().take_route_request()),
            (Page::Training, self.training.borrow_mut().take_route_request()),
            (Page::Fitter, self.fitter.borrow_mut().take_route_request()),
        ];
        for (page, request) in requests {
            let Some(request) = request else { continue };
            // The replaced one is shown (e.g. a scene rebuilt into a fresh recording, see
            // `scene3d/rerun/compact.rs`, or picked in Sources): its successor is shown instead.
            let shown = request.close.as_ref().is_some_and(|old| self.rerun_app.active_recording_id() == Some(old));
            if let Some(store_id) = request.close {
                self.automation_closed(&store_id);
                self.code_layout.replace(&store_id, request.open.clone());
                self.rerun_app
                    .command_sender
                    .send_system(SystemCommand::CloseRecordingOrTable(RecordingOrLocalTable::Recording {
                        store_id,
                    }));
            }
            if shown && *self.page.borrow() != page {
                self.rerun_app
                    .command_sender
                    .send_system(SystemCommand::SetRoute(Route::LocalRecording { recording_id: request.open.clone() }));
            }
            if *self.page.borrow() == page {
                self.preview_route = Some((page, request.open, ago(Duration::from_secs(1))));
            }
        }

        let Some((page, store_id, last_request)) = self.preview_route.clone() else {
            return;
        };
        if self.rerun_app.active_recording_id() == Some(&store_id) || *self.page.borrow() != page {
            self.preview_route = None;
        } else if self.code_layout.active() {
            // The new preview comes back with the 3D view when the code page is closed.
            self.code_layout.retarget(store_id);
            self.preview_route = None;
        } else if last_request.elapsed() > Duration::from_millis(250) {
            self.preview_route = Some((page, store_id.clone(), Instant::now()));
            self.rerun_app
                .command_sender
                .send_system(SystemCommand::SetRoute(Route::LocalRecording { recording_id: store_id }));
        }
    }

    /// Show the code page in place of the 3D view while it is open, and put the 3D view back when
    /// it is closed (see `code_layout`).
    fn sync_code_layout(&mut self) {
        let wanted = self.code_export.borrow().open || self.ide.borrow().open;
        let shown = self.rerun_app.active_recording_id().cloned();
        match self.code_layout.update(wanted, shown.as_ref()) {
            code_layout::Switch::None | code_layout::Switch::Restore(None) => {}
            code_layout::Switch::ShowCode => {
                self.rerun_app.command_sender.send_system(SystemCommand::SetRoute(Route::welcome_page()));
            }
            code_layout::Switch::Restore(Some(recording_id)) => {
                self.rerun_app.command_sender.send_system(SystemCommand::SetRoute(Route::LocalRecording { recording_id }));
            }
            code_layout::Switch::Interrupted => {
                self.code_export.borrow_mut().open = false;
                self.ide.borrow_mut().open = false;
            }
        }
        self.code_page.set(self.code_layout.active());
        self.ide_shown.set(self.code_layout.active() && self.ide.borrow().open);
    }

    fn handle_action(&mut self, action: sidebar::Action) {
        use sidebar::Action;

        if self.code_layout.active() && leaves_code_page(&action) {
            // Back to the layout before, then on to where the action goes.
            self.code_export.borrow_mut().open = false;
            self.ide.borrow_mut().open = false;
            self.sync_code_layout();
        }
        match action {
            Action::OpenFitter => self.edit_in_fitter(EditThen::OpenFitter),
            Action::EditScene => {
                self.fitter.borrow().stop_and_edit();
                self.edit_now(EditThen::OpenFitter, true);
            }
            Action::OpenProduct => self.open_product(None),
            Action::EditProductTask { product, task } => self.open_product(Some((product, task))),
            Action::OpenDresser => {
                self.fitter_tool.set(false);
                let catalogue = {
                    let config = self.config.borrow();
                    config.resolve(&config.catalogue)
                };
                self.dresser.borrow_mut().open(&catalogue);
                if let Some(command) = std::env::var_os("FACTORY_OS_DRESSER").filter(|_| !self.dresser_command_run) {
                    self.dresser_command_run = true;
                    self.dresser.borrow_mut().run_command(&command.to_string_lossy());
                }
                let preview = self.dresser.borrow().preview_recording();
                match preview {
                    Some(preview) => {
                        *self.page.borrow_mut() = Page::Dresser;
                        self.preview_route =
                            Some((Page::Dresser, preview, ago(Duration::from_secs(1))));
                    }
                    None => self.show_page(Page::Dresser),
                }
            }
            Action::OpenTrainingJob => {
                self.fitter_tool.set(false);
                self.task_editor_open.set(false);
                self.training.borrow_mut().open();
                let preview = self.training.borrow().preview_recording();
                match preview {
                    Some(preview) => {
                        *self.page.borrow_mut() = Page::Training;
                        self.preview_route = Some((Page::Training, preview, ago(Duration::from_secs(1))));
                    }
                    None => self.show_page(Page::Training),
                }
            }
            Action::ViewLogs => self.show_page(Page::BackendLogs),
            Action::SetActiveCell(cell) => self.backend.set_active_cell(&cell),
            Action::UseScene { scene, open: false } => self.backend.use_scene(&scene, |_| {}),
            Action::UseScene { scene, open: true } => {
                if *self.page.borrow() != Page::Fitter {
                    self.open_fitter(true);
                }
                self.fitter_tool.set(true);
                let open = self.fitter.borrow().cell_opener();
                self.backend.use_scene(&scene, open);
            }
            Action::PlanCell { task, planner } => {
                // Planning writes into the cell's `agentic_planner/`, which Prepare Cell creates.
                if !self.cell.lock().unwrap().plans.prepared {
                    self.backend.agentic(api::AGENTIC_PREPARE);
                }
                let title = format!("Plan {} · {planner}", self.planner_cell());
                let prompt = self.plan_prompt(&task, &planner).map(|prompt| (title, prompt));
                self.open_agent_chat(prompt, true);
            }
            Action::AskAgent { title, prompt } => self.open_agent_chat(Some((title, prompt)), false),
            Action::Agent(step) => {
                let prompt = self.agent_prompt(step).map(|prompt| (self.agent_title(step), prompt));
                if prompt.is_none() {
                    match step {
                        sidebar::AgentStep::ReachCheck => re_log::warn!("{step:?}: open a saved scene with robots in the Robo Fitter first"),
                        _ => re_log::warn!("{step:?}: save a product in Product & Task first"),
                    }
                }
                self.open_agent_chat(prompt, false);
            }
            Action::RefreshCells => self.backend.cells_refresher()(),
            Action::SetWeldLabels(on) => {
                // The same setting as the 3D view's "Labels" toggle.
                crate::scene3d::set_labels_shown(on);
                self.fitter.borrow().set_weld_labels(on);
            }
            Action::OpenCell(cell) => {
                if *self.page.borrow() != Page::Fitter {
                    self.open_fitter(true);
                }
                self.fitter_tool.set(true);
                self.fitter.borrow().open_cell(&cell);
            }
            Action::EditTask(task) => self.edit_in_fitter(EditThen::EditTask(task)),
            Action::RefreshPlans => {
                self.plans_listed = Instant::now();
                self.backend.refresh_plans();
            }
            Action::LoadPlan { path, cell } => {
                // Unsaved edits of an unlinked scene would go: the Fitter shows the cell's scene.
                let unsaved = self.fitter.borrow().unsaved_file().filter(|_| self.fitter.borrow().linked_cell().is_none());
                match unsaved {
                    Some(file) => self.confirm = Some(Confirm::LoadPlan { path, cell, file }),
                    None => self.load_and_play(path, cell),
                }
            }
            Action::Trajectory(kind, params) => self.cell.lock().unwrap().command(kind, params),
            Action::WatchCell => self.watch_cell(),
        }
    }

    /// Open Product & Task: on `product` and its task when given (a product-derived task
    /// program's Edit), else on the current product.
    fn open_product(&mut self, on: Option<(String, Option<String>)>) {
        self.fitter_tool.set(false);
        self.task_editor_open.set(false);
        match on {
            Some((product, task)) => self.product.borrow_mut().open_on(&product, task),
            None => self.product.borrow_mut().open(),
        }
        let preview = self.product.borrow().preview_recording();
        match preview {
            Some(preview) => {
                *self.page.borrow_mut() = Page::Product;
                self.preview_route = Some((Page::Product, preview, ago(Duration::from_secs(1))));
            }
            None => self.show_page(Page::Product),
        }
    }

    /// Step 1 Load & Play: the simulation of `cell`, shown in the Robo Fitter's 3D view as its
    /// viewer, without the Fitter's Scene editor and panel (also when the Fitter was open as the tool:
    /// Open Fitter brings them back).
    fn load_and_play(&mut self, path: String, cell: Option<String>) {
        if *self.page.borrow() != Page::Fitter {
            self.open_fitter(false);
        } else {
            self.fitter_tool.set(false);
            self.task_editor_open.set(false);
        }
        self.keep_fitter_view_until = Some(Instant::now() + Duration::from_secs(10));
        self.backend.load_plan(factoryos_client::protocol::LoadPlan { path, play: true, cell });
    }

    /// Open Fitter, or a task program's Edit: while the Fitter shows a cell (simulated or not),
    /// ask first whether to stop the simulation and edit. Asked from any page: what the Fitter
    /// would show decides.
    fn edit_in_fitter(&mut self, then: EditThen) {
        let fitter = self.view_mode(Page::Fitter, false);
        if !view_mode::edit_needs_confirm(&fitter) {
            return self.edit_now(then, true);
        }
        let cell = view_mode::shown_cell(&fitter).unwrap_or_default().to_owned();
        let bt = match &fitter {
            view_mode::Visible::Simulation { bt, .. } => bt.clone(),
            _ => None,
        };
        let simulating = matches!(fitter, view_mode::Visible::Simulation { .. });
        let scene = crate::cell_list::referenced_scene(&self.cell.lock().unwrap().cells, &cell);
        self.confirm = Some(Confirm::Edit { cell, bt, simulating, scene, then });
    }

    /// Open the Fitter (as a tool, or `as_tool == false` as a viewer) and the task program if asked.
    fn edit_now(&mut self, then: EditThen, as_tool: bool) {
        match then {
            EditThen::OpenFitter => self.open_fitter(as_tool),
            EditThen::EditTask(task) => {
                // The editor is a pane beside the Robo Fitter's 3D view.
                if *self.page.borrow() != Page::Fitter {
                    self.open_fitter(as_tool);
                }
                self.fitter_tool.set(as_tool);
                self.task_editor_open.set(true);
                self.task_editor.borrow_mut().open(task);
            }
        }
    }

    /// What the 3D view shows this frame (see `view_mode`).
    fn view_mode(&self, page: Page, showing_live: bool) -> view_mode::Visible {
        let fitter = self.fitter.borrow().view_facts();
        let shared = self.cell.lock().unwrap();
        let bt = loaded_plan(&shared);
        let playing = shared.cell_state.as_ref().is_some_and(|state| {
            state["trajectory"]["loaded"] == true && state["trajectory"]["playing"] == true
        });
        let live_cell = shared.live.as_ref().and_then(|live| live.cell.clone());
        view_mode::visible(&view_mode::ViewFacts {
            page: match page {
                Page::Home => view_mode::ViewPage::Home,
                Page::Fitter => view_mode::ViewPage::Fitter,
                Page::LiveCell => view_mode::ViewPage::LiveCell,
                Page::Product => view_mode::ViewPage::Product,
                Page::Dresser => view_mode::ViewPage::Dresser,
                Page::Training => view_mode::ViewPage::Training,
                Page::BackendLogs => view_mode::ViewPage::Other,
            },
            scene: &fitter.scene,
            file: &fitter.file,
            unsaved: fitter.unsaved,
            linked_cell: fitter.linked_cell.as_deref(),
            sim: shared.status.state,
            sim_cell: shared.status.cell.as_deref(),
            sim_error: shared.status.error.as_deref(),
            bt: bt.as_deref(),
            playing,
            live_cell: live_cell.as_deref(),
            showing_live,
        })
    }

    /// The open confirmation, as a modal; its answer acts right away.
    fn confirm_ui(&mut self, ctx: &egui::Context) {
        let Some(confirm) = &self.confirm else { return };
        let (title, text, yes, no) = confirm_texts(confirm);
        let mut answer = None;
        let response = egui::Modal::new(egui::Id::new("factory_os_confirm")).show(ctx, |ui| {
            theme::controls_style(ui);
            ui.set_max_width(440.0);
            ui.heading(title);
            ui.add_space(6.0);
            ui.label(text);
            ui.add_space(10.0);
            ui.horizontal(|ui| {
                if theme::primary_button(ui, yes).clicked() {
                    answer = Some(true);
                }
                if ui.button(no).clicked() {
                    answer = Some(false);
                }
            });
        });
        if response.should_close() && answer.is_none() {
            answer = Some(false);
        }
        let Some(yes) = answer else { return };
        let Some(confirm) = self.confirm.take() else { return };
        match (confirm, yes) {
            (Confirm::LoadPlan { path, cell, .. }, true) => self.load_and_play(path, cell),
            (Confirm::LoadPlan { .. }, false) => {}
            (Confirm::Edit { then, .. }, true) => {
                self.fitter.borrow().stop_and_edit();
                self.edit_now(then, true);
            }
            (Confirm::Edit { then, .. }, false) => self.edit_now(then, false),
        }
    }

    /// The Robo Fitter page: `as_tool` with its Scene editor and panel (the user opened it), else as the
    /// viewer of the cell playing a behaviour tree (see `fitter_panes`).
    fn open_fitter(&mut self, as_tool: bool) {
        // The Fitter itself, without the Task Program pane ("Edit" reopens it after this).
        self.task_editor_open.set(false);
        self.fitter_tool.set(as_tool);
        let asset_root = {
            let config = self.config.borrow();
            config.resolve(&config.asset_root)
        };
        *self.page.borrow_mut() = Page::Fitter;
        let mut fitter = self.fitter.borrow_mut();
        fitter.open(&asset_root);
        if let Some(command) = std::env::var_os("FACTORY_OS_FITTER").filter(|_| !self.fitter_command_run) {
            self.fitter_command_run = true;
            fitter.run_command(&command.to_string_lossy());
        }
        if let Some(preview) = fitter.preview_recording() {
            self.preview_route = Some((Page::Fitter, preview, ago(Duration::from_secs(1))));
        }
    }

    /// Watch the cell: the live cell's own 3D recording (`factoryos/rrd/cell`; on a server with
    /// the virtual cell, the robots' live poses) on the Live cell page. No Robo Fitter panes; the
    /// simulation of step 1 is left as it is.
    fn watch_cell(&mut self) {
        self.open_live_cell();
    }

    /// The cell reached production after a start from this viewer: show it live, unless the user
    /// is on another tool's page (see `watch_on_production`), or there is no stream to show.
    fn watch_cell_in_production(&mut self) {
        let stream = self.cell.lock().unwrap().recording_id.is_some();
        if stream && watch_on_production(*self.page.borrow(), self.fitter_tool.get()) {
            self.watch_cell();
        }
    }

    /// The Kiro chat for Agentic Planning, run by the backend (`kiro-cli acp` in the planner
    /// checkout, where its MCP servers and the `factoryos-planner` agent live) and shown in our
    /// chat pane.
    #[cfg(target_arch = "wasm32")]
    fn open_agent_chat(&mut self, prompt: Option<(String, String)>, _needs_simulation: bool) {
        self.open_backend_chat(prompt);
    }

    /// Open the chat pane on the backend's planner chat; with a `(title, prompt)`, in a new tab
    /// with a fresh agent that starts on the prompt (so planners and diagnoses run side by side).
    fn open_backend_chat(&mut self, request: Option<(String, String)>) {
        self.chat_open.set(true);
        let mut chat = self.chat.borrow_mut();
        chat.attach(&self.backend.api, self.backend.repaint_handle());
        if let Some((title, prompt)) = request {
            chat.start(title, Some(prompt));
        }
    }

    /// A chat tab's title for an agent `step`, e.g. "Layout jam2".
    fn agent_title(&self, step: sidebar::AgentStep) -> String {
        use sidebar::AgentStep;
        let product = self.product.borrow().current().map(|(name, _)| name).unwrap_or_default();
        let cell = self.planner_cell();
        match step {
            AgentStep::TaskDescription => format!("Task {product}"),
            AgentStep::Dressing => format!("Robots {product}"),
            AgentStep::CellLayout => format!("Layout {cell}"),
            AgentStep::ReachCheck => format!("Reach {cell}"),
        }
        .trim()
        .to_owned()
    }

    /// The cell the planner works on: the running planner's, else the active cell.
    fn planner_cell(&self) -> String {
        let backend = self.cell.lock().unwrap();
        Some(backend.agentic.cell.clone())
            .filter(|cell| !cell.is_empty())
            .unwrap_or_else(|| backend.cells.active.clone())
    }

    /// The "Plan prompt" for `task` and `planner` in the planner's cell; `None` without a cell.
    fn plan_prompt(&self, task: &str, planner: &str) -> Option<String> {
        let cell = self.planner_cell();
        let template = self.config.borrow().agentic_plan_prompt.clone();
        let plans = self.cell.lock().unwrap().plans.clone();
        (!cell.is_empty()).then(|| planning_prompt(&template, &cell, task, planner) + &no_task_program_note(&plans, task))
    }

    /// A step's agent prompt for the current product (for [`sidebar::AgentStep::ReachCheck`]: the
    /// Fitter's saved scene); `None` without one.
    fn agent_prompt(&self, step: sidebar::AgentStep) -> Option<String> {
        use factoryos_client::protocol::config::{
            DEFAULT_DRESSER_PROMPT, DEFAULT_FITTER_PROMPT, DEFAULT_REACH_PROMPT, DEFAULT_TASK_PROMPT,
        };
        use sidebar::AgentStep;

        let current = self.product.borrow().current();
        let config = self.config.borrow();
        let (template, default) = match step {
            AgentStep::TaskDescription => (&config.task_author_prompt, DEFAULT_TASK_PROMPT),
            AgentStep::Dressing => (&config.dresser_agent_prompt, DEFAULT_DRESSER_PROMPT),
            AgentStep::CellLayout => (&config.fitter_agent_prompt, DEFAULT_FITTER_PROMPT),
            AgentStep::ReachCheck => (&config.reach_agent_prompt, DEFAULT_REACH_PROMPT),
        };
        let template = if template.trim().is_empty() { default } else { template.as_str() };
        let asset_root = config.resolve(&config.asset_root);
        // As the backend's: a configured `…/scenes` (the old default) is the catalogue's scenes/ now.
        let configured = config.resolve(&config.fitted_robots_dir);
        let legacy = std::path::Path::new(&configured).file_name().is_some_and(|n| n == factoryos_client::protocol::fitter::LEGACY_SCENES_DIR);
        let fitted = if configured.trim().is_empty() || legacy {
            std::path::Path::new(&asset_root).join(factoryos_client::protocol::fitter::SCENES_DIR).to_string_lossy().into_owned()
        } else {
            configured
        };
        let cell = self.planner_cell();
        let scene_file = if step == AgentStep::ReachCheck {
            let fitter = self.fitter.borrow();
            let file = fitter.agent_scene_file()?;
            let linked = fitter.linked_cell();
            let cells_dir = self.cell.lock().unwrap().cells.cells_dir.clone();
            match linked.filter(|_| fitter.is_cell_scene()) {
                Some(linked) => reach_scene_path(&fitted, &file, Some((&cells_dir, &linked))),
                None => reach_scene_path(&fitted, &file, None),
            }
        } else {
            String::new()
        };
        let (product, file) = match (current, step) {
            (Some(current), _) => current,
            (None, AgentStep::ReachCheck) => (String::new(), String::new()),
            (None, _) => return None,
        };
        let product_dir = std::path::Path::new(&file).parent().map(|d| d.to_string_lossy().into_owned()).unwrap_or_default();
        Some(fill_agent_prompt(
            template,
            &[
                ("{scene_file}", &scene_file),
                ("{product_file}", &file),
                ("{product_dir}", &product_dir),
                ("{product}", &product),
                ("{catalogue}", &config.resolve(&config.catalogue)),
                ("{asset_root}", &asset_root),
                ("{fitted_robots_dir}", &fitted),
                ("{cell_id}", &planner_cell_id(&cell)),
                ("{cell_dir}", &config.resolve(&config.cell_dir)),
                ("{cell}", &cell),
            ],
        ))
    }

    #[cfg(not(target_arch = "wasm32"))]
    /// Open the agent's chat; with `prompt`, also send it once the agent is up (and the
    /// simulation, if the prompt `needs_simulation`: the planner's MCP tools run in it).
    ///
    /// With a local backend that is Rerun's agent panel running `kiro-cli acp` here (unless
    /// [`crate::backend_choice::backend_chat`]); with a backend on another machine, its planner and
    /// checkout are there, so it's the backend's chat.
    fn open_agent_chat(&mut self, prompt: Option<(String, String)>, needs_simulation: bool) {
        if self.backend_chat {
            return self.open_backend_chat(prompt);
        }
        // Rerun's agent panel is one session: the prompt goes there.
        let prompt = prompt.map(|(_title, prompt)| prompt);
        let repo = {
            let config = self.config.borrow();
            config.resolve(&config.agentic_repo_dir)
        };
        let agent_file = std::path::Path::new(&repo).join(format!(".kiro/agents/{PLANNER_AGENT}.json"));
        let mut kiro = if agent_file.is_file() {
            format!("kiro-cli acp --agent {PLANNER_AGENT}")
        } else {
            "kiro-cli acp".to_owned() // falls back to the checkout's .kiro/settings/mcp.json
        };
        if self.config.borrow().agentic_trust_all_tools {
            kiro.push_str(" --trust-all-tools");
        }
        // `AgentSettings::cwd` only goes into the ACP session request; the agent process itself
        // inherits the viewer's directory. kiro-cli finds `--agent` (and the checkout's MCP
        // servers) in its own working directory, so start it through a shell that changes there.
        let command_line = if cfg!(windows) || repo.trim().is_empty() {
            kiro
        } else {
            format!("/bin/sh -c \"cd {} && exec {kiro}\"", shell_quote(&repo))
        };
        if let Some(prompt) = prompt {
            self.planning_kickoff = Some(PlanningKickoff {
                prompt,
                needs_simulation,
                mode_requested: None,
            });
        }
        let settings = re_viewer::AgentSettings {
            profile_id: re_viewer::AgentSettings::CUSTOM_PROFILE_ID.to_owned(),
            custom_command_line: command_line,
            cwd: repo,
            setup_done: true,
            // The planner has no use for driving the viewer, and `rerun viewer-mcp`'s
            // `rerun_close_recordings` tool has a top-level `oneOf` schema the model API rejects.
            connect_viewer: false,
            ..self.rerun_app.agent_settings().clone()
        };
        self.rerun_app.open_agent_panel_with(settings);
    }

    /// On the web the planner chat is always the backend's, which gets its prompt right away (see
    /// `open_agent_chat`), so nothing ever waits for it.
    #[cfg(target_arch = "wasm32")]
    fn drive_planning_chat(&mut self) {
        self.planning_waiting.set(false);
    }

    /// Planning Perspective: once the simulation (planner MCP servers) and the agent are up, select
    /// the planner agent's mode and send the kick-off prompt, so the planning shows in the chat.
    #[cfg(not(target_arch = "wasm32"))]
    fn drive_planning_chat(&mut self) {
        use factoryos_client::protocol::agentic::SimState;
        let Some(kickoff) = self.planning_kickoff.as_mut() else {
            self.planning_waiting.set(false);
            return;
        };
        let simulation_ready =
            !kickoff.needs_simulation || self.cell.lock().unwrap().agentic.state == SimState::Ready;
        self.planning_waiting.set(!simulation_ready);
        let Some(panel) = self.rerun_app.agent_panel_mut().filter(|panel| panel.is_ready()) else {
            return;
        };
        if !simulation_ready {
            return;
        }
        // Kiro reports its agents as session modes; `--agent` may leave the default one current.
        if let Some(modes) = panel.modes() {
            let current = modes.current_mode_id.0.as_ref() == PLANNER_AGENT;
            let planner = modes.available_modes.iter().find(|mode| mode.id.0.as_ref() == PLANNER_AGENT);
            match (current, planner, kickoff.mode_requested) {
                (false, Some(planner), None) => {
                    let id = planner.id.clone();
                    panel.set_mode(id);
                    kickoff.mode_requested = Some(Instant::now());
                    return;
                }
                // Give the switch a moment before prompting.
                (false, Some(_), Some(at)) if at.elapsed() < Duration::from_secs(3) => return,
                _ => {}
            }
        }
        if panel.send_prompt(kickoff.prompt.clone()) {
            self.planning_kickoff = None;
            self.planning_waiting.set(false);
        }
    }

    /// Physical AI Production Designer settings (backend, then each tool's config) over everything below the top bar:
    /// a modal, closed with its ✕, Escape, the gear or a click on the top bar. Edits are saved by
    /// the backend as they are made.
    fn settings_overlay(&mut self, ctx: &egui::Context) {
        if !self.settings_open.get() {
            return;
        }
        let id = egui::Id::new("factory_os_settings");
        let rect = self.below_top_bar.get().unwrap_or_else(|| ctx.content_rect());
        let area = egui::Area::new(id)
            .kind(egui::UiKind::Modal)
            .sense(egui::Sense::hover())
            .order(egui::Order::Foreground)
            .interactable(true)
            .fixed_pos(rect.min);
        let margin = 24.0;
        let frame = egui::Frame::new()
            .fill(ctx.global_style().visuals.panel_fill)
            .inner_margin(margin);
        let response = egui::Modal::new(id).area(area).frame(frame).show(ctx, |ui| {
            theme::controls_style(ui);
            let inner = rect.shrink(margin);
            ui.set_width(inner.width());
            ui.set_height(inner.height());
            ui.horizontal(|ui| {
                ui.heading("Physical AI Production Designer Settings");
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.small_icon_button(&re_ui::icons::CLOSE, "Close").clicked() {
                        self.settings_open.set(false);
                    }
                });
            });
            ui.separator();
            // Only scrolls if the window is too small for the columns.
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                let workspaces = self.workspaces.clone();
                #[cfg(not(target_arch = "wasm32"))]
                let general = |ui: &mut egui::Ui| {
                    theme::settings_ui(ui);
                    ui.separator();
                    crate::backend_choice::ui(ui, &self.backend_shown.0, self.backend_shown.1);
                    workspaces.borrow_mut().settings_ui(ui);
                };
                #[cfg(target_arch = "wasm32")]
                let general = |ui: &mut egui::Ui| {
                    theme::settings_ui(ui);
                    ui.separator();
                    workspaces.borrow_mut().settings_ui(ui);
                };
                config::ui(ui, &mut self.config.borrow_mut(), &mut self.secrets.borrow_mut(), general);
            });
        });
        if response.should_close() {
            self.settings_open.set(false);
        }
    }

    /// Run the automation's commands that arrived, and save the screenshots that came back.
    fn automation_commands(&mut self, ctx: &egui::Context) {
        use crate::automation::{self, Command, ProductSource};
        use serde_json::{Value, json};

        let Some(automation) = &mut self.automation else { return };
        automation.handle_screenshots(ctx);
        if automation.pointer.is_some() {
            // One step of it per frame (`Automation::filter_input`).
            ctx.request_repaint();
        }
        automation.history.observe(self.rerun_app.recording_db());
        for request in automation_requests(&mut self.automation) {
            let id = request.id.as_ref();
            let name = request.command.name();
            let ok = |fields: Value| automation::reply(id, name, Ok(fields));
            match request.command {
                Command::Open { tool } => {
                    self.handle_action(automation::open_action(tool));
                    ok(json!({ "page": page_name(*self.page.borrow()) }));
                }
                Command::ProductSelect { name: product, from } => {
                    match from {
                        ProductSource::Catalogue => {
                            self.open_product(None);
                            self.product.borrow().load_from_catalogue(&product);
                        }
                        ProductSource::Products => self.handle_action(sidebar::Action::EditProductTask { product: product.clone(), task: None }),
                    }
                    ok(json!({ "page": page_name(*self.page.borrow()), "product": product }));
                }
                Command::WaitIdle { timeout_s, settle_s } => {
                    let wait = automation::IdleWait::new(request.id.clone(), timeout_s, settle_s, Instant::now());
                    if let Some(automation) = &mut self.automation {
                        automation.waits.push(wait);
                    }
                    ctx.request_repaint();
                }
                Command::Screenshot { path, region } => {
                    if let Some(automation) = &mut self.automation {
                        automation.request_screenshot(ctx, request.id.clone(), path, region);
                    }
                }
                Command::State {} => ok(self.automation_state(ctx)),
                Command::Scene { nodes, since } => ok(self.automation_scene_facts(&nodes, since)),
                Command::Camera {} => ok(self.automation_camera(ctx)),
                Command::Drag { .. } | Command::DoubleClick { .. } => {
                    let screen = ctx.input(|input| input.content_rect());
                    match automation::view_3d_rect(&ctx.interactive_rects_last_pass(), screen) {
                        Some(rect) => {
                            let pointer = match request.command {
                                Command::Drag { dx, dy, button, steps } => {
                                    automation::PendingPointer::drag(request.id.clone(), rect.center(), egui::vec2(dx, dy), button, steps)
                                }
                                Command::DoubleClick { at } => automation::PendingPointer::double_click(request.id.clone(), rect.lerp_inside(egui::vec2(at[0], at[1]))),
                                _ => unreachable!(),
                            };
                            if let Some(automation) = &mut self.automation {
                                automation.start_pointer(pointer);
                            }
                            ctx.request_repaint();
                        }
                        None => automation::reply(id, name, Err(("no 3D view to point at".to_owned(), Value::Null))),
                    }
                }
                Command::SetWindow { width, height } => {
                    ctx.send_viewport_cmd(egui::ViewportCommand::InnerSize(egui::vec2(width, height)));
                    ctx.request_repaint();
                    ok(json!({ "width": width, "height": height }));
                }
                Command::RealPointer { on } => {
                    if let Some(automation) = &mut self.automation {
                        automation.set_real_pointer(on);
                    }
                    ok(json!({ "on": on }));
                }
                Command::Action { action } => {
                    self.handle_action(action.action());
                    ok(json!({ "page": page_name(*self.page.borrow()) }));
                }
                Command::ExportCode { plan } => {
                    self.code_export.borrow_mut().request_export(plan.clone());
                    ctx.request_repaint();
                    ok(json!({ "plan": plan }));
                }
                Command::Quit {} => {
                    ok(Value::Null);
                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                }
            }
        }
    }

    /// The 3D scene of the page shown, if it has one.
    /// The "Labels" toggle over the 3D view (`labels_overlay`), while it shows a tool's scene
    /// (`shown`), and the Robo Fitter's weld labels in step with it (one setting).
    fn labels_overlay(&self, ctx: &egui::Context, shown: bool) {
        let fitter = self.fitter.borrow();
        crate::labels_overlay::follow_fitter(ctx, fitter.weld_labels());
        if !shown {
            return;
        }
        let screen = ctx.input(|input| input.content_rect());
        let Some(view) = crate::automation::view_3d_rect(&ctx.interactive_rects_last_pass(), screen) else { return };
        if let Some(on) = crate::labels_overlay::ui(ctx, view)
            && fitter.weld_labels() != on
        {
            fitter.set_weld_labels(on);
        }
    }

    fn automation_scene<T>(&self, page: Page, f: impl FnOnce(&crate::scene3d::Scene3d) -> T) -> Option<T> {
        match page {
            Page::Dresser => Some(f(&self.dresser.borrow().stream)),
            Page::Fitter => Some(f(&self.fitter.borrow().stream)),
            Page::Product => Some(f(&self.product.borrow().stream)),
            Page::Training => Some(f(&self.training.borrow().stream)),
            Page::LiveCell => self.cell.lock().unwrap().cell_scene.as_ref().map(f),
            Page::Home | Page::BackendLogs => None,
        }
    }

    /// Why the page's 3D view doesn't show all of its scene yet (`None`: it does), for `wait_idle`.
    fn automation_busy(&self) -> Option<String> {
        let page = *self.page.borrow();
        if self.code_layout.active() {
            return Some("the code page is shown".to_owned());
        }
        let Some((pending, current)) = self.automation_scene(page, |scene| (scene.pending(), scene.current())) else {
            return Some(format!("page {} has no 3D scene", page_name(page)));
        };
        if let Some(reason) = pending.busy_reason() {
            return Some(reason);
        }
        if page == Page::Product
            && let Some(reason) = self.product.borrow().unsettled()
        {
            return Some(reason.to_owned());
        }
        let Some(current) = current else { return Some("no recording yet".to_owned()) };
        if self.preview_route.is_some() || self.open_cell_recording {
            return Some("routing to the scene's recording".to_owned());
        }
        match self.rerun_app.active_recording_id() {
            Some(active) if *active == current => None,
            Some(active) => Some(format!("Rerun shows {active}, not the scene's {current}")),
            None => Some(format!("Rerun shows no recording, not the scene's {current}")),
        }
    }

    /// The `scene` reply: the facts of the recording Rerun shows (see `automation::scene_facts`),
    /// the page's scene, the mesh history after `since` and the recordings closed.
    fn automation_scene_facts(&self, nodes: &[String], since: u64) -> serde_json::Value {
        let page = *self.page.borrow();
        let scene = self.automation_scene(page, |scene| (scene.current(), scene.pending()));
        let (scene_recording, pending) = match scene {
            Some((current, pending)) => (current.map(|id| id.to_string()), Some(pending)),
            None => (None, None),
        };
        let store = self.rerun_app.recording_db().map(|db| crate::automation::scene_facts(db, nodes));
        let (history, last_seq) = self.automation.as_ref().map(|a| a.history.since(since)).unwrap_or_default();
        let closed = self.automation.as_ref().map(|a| a.closed().to_vec()).unwrap_or_default();
        serde_json::json!({
            "page": page_name(page),
            "active_recording": self.rerun_app.active_recording_id().map(|id| id.to_string()),
            "scene_recording": scene_recording,
            "pending": pending,
            "store": store,
            "history": history,
            "history_seq": last_seq,
            "closed_recordings": closed,
        })
    }

    /// The automation's `camera`: the shown page's 3D view's eye (see `automation::camera_facts`).
    fn automation_camera(&self, ctx: &egui::Context) -> serde_json::Value {
        let page = *self.page.borrow();
        let screen = ctx.input(|input| input.content_rect());
        let view_rect = crate::automation::view_3d_rect(&ctx.interactive_rects_last_pass(), screen);
        let eyes = self.rerun_app.store_hub().and_then(|hub| self.automation_scene(page, |scene| scene.eye(hub)));
        let (view, fitted) = eyes.unwrap_or((None, None));
        let mut facts = crate::automation::camera_facts(view.as_ref(), fitted, view_rect);
        facts["page"] = page_name(page).into();
        facts
    }

    /// Whether Rerun's right panel is expanded now, from the active blueprint of the recording shown
    /// (as Rerun reads it: expanded by default in a window wider than 1000 points).
    fn read_right_panel(&mut self, ctx: &egui::Context) {
        let default = ctx.content_rect().width() > 1000.0;
        let open = self
            .rerun_app
            .store_hub()
            .and_then(|hub| {
                let recording = self.rerun_app.active_recording_id()?;
                let blueprint = hub.active_blueprint_id_for_app(recording.application_id())?;
                hub.entity_db(blueprint).map(|db| crate::scene3d::rerun::blueprint::right_panel_expanded(db, default))
            })
            .unwrap_or(default);
        self.right_panel_open.set(open);
    }

    /// Camera commands from the backend ([`crate::view_commands`]): each moves the 3D view of the
    /// page it names (`page`), else of the page shown.
    fn apply_view_commands(&mut self) {
        for command in crate::view_commands::take() {
            let page = match command.page.as_deref() {
                Some("dresser") => Page::Dresser,
                Some("fitter") => Page::Fitter,
                Some("product") => Page::Product,
                Some("training") => Page::Training,
                Some("live") => Page::LiveCell,
                _ => *self.page.borrow(),
            };
            match self.automation_scene(page, |scene| scene.command_camera(&command)) {
                Some(Ok(_)) => {}
                Some(Err(err)) => re_log::warn!("view command {}: {err}", command.action),
                None => re_log::debug!("view command {}: page {} has no 3D view", command.action, page_name(page)),
            }
        }
    }

    /// Note a recording the app asks Rerun to close, for the automation's `scene`.
    fn automation_closed(&mut self, store_id: &StoreId) {
        if let Some(automation) = &mut self.automation {
            automation.note_closed(store_id);
        }
    }

    fn automation_state(&self, ctx: &egui::Context) -> serde_json::Value {
        let page = *self.page.borrow();
        let active = self.rerun_app.active_recording_id().cloned();
        let scene = self.automation_scene(page, |scene| (scene.current(), scene.title(), scene.pending()));
        let (scene_recording, title, pending) = match scene {
            Some((current, title, pending)) => (current.map(|id| id.to_string()), title, Some(pending)),
            None => (None, None, None),
        };
        let (screen, pixels_per_point) = ctx.input(|input| (input.content_rect(), input.pixels_per_point));
        serde_json::json!({
            "page": page_name(page),
            "shown_tool": active.as_ref().and_then(tool_page).map(page_name),
            "active_recording": active.map(|id| id.to_string()),
            "scene_recording": scene_recording,
            "title": title,
            "pending": pending,
            "busy": self.automation_busy(),
            "product": (page == Page::Product).then(|| self.product.borrow().product_name().to_owned()),
            "protocol_banner": self.protocol_banner,
            "backend_reconnecting": self.reconnect.reconnecting(),
            "scenes_connected": self.protocol_banner.is_none(),
            "window": { "width": screen.width(), "height": screen.height(), "pixels_per_point": pixels_per_point },
            "code_export": self.code_export.borrow().automation_facts(),
            "code_page": self.code_layout.active(),
        })
    }

    fn show_page(&self, page: Page) {
        self.fitter_tool.set(false);
        *self.page.borrow_mut() = page;
        self.rerun_app
            .command_sender
            .send_system(SystemCommand::SetRoute(Route::welcome_page()));
    }

    /// The Live cell page: the live cell's recording.
    fn open_live_cell(&mut self) {
        self.fitter_tool.set(false);
        // Not back to the Fitter's 3D view when the live stream arrives (see `keep_fitter_view`).
        self.keep_fitter_view_until = None;
        *self.page.borrow_mut() = Page::LiveCell;
        self.open_cell_recording = true;
        self.last_route_request = None;
    }
}

/// Rerun's memory limit, for scenes that are all static: Rerun's GC never drops static data
/// (`re_chunk_store/src/gc.rs`), so once over its limit it frees nothing of ours and closes the
/// recordings not shown instead (`StoreHub::purge_fraction_of_ram`); their next messages would open
/// them again half empty. So the limit must stay out of reach: at least Rerun's default for the
/// platform (75 % of RAM natively, 2.5 GB on the web), also if a lower one was saved in its settings.
/// What keeps the scenes small is rebuilding them (`scene3d/rerun/compact.rs`).
fn raise_memory_limit(options: &mut crate::rr::re_viewer_context::AppOptions) {
    let default = crate::rr::re_memory::MemoryLimit::default_for_current_platform();
    if options.memory_limit.as_bytes() < default.as_bytes() {
        re_log::info!("Raising Rerun's memory limit to {} (the scenes are static, Rerun can't free them)", default.as_bytes() >> 20);
        options.memory_limit = default;
    }
}

/// Rerun's shortcuts that save, export or open recordings: the scenes are live views, the
/// recording is the behaviour tree. Consumed before Rerun's UI sees them (not while the Workspace
/// IDE is shown, whose Cmd+S saves its file). Their command palette entries remain (a Rerun patch
/// would hide them).
fn consume_recording_shortcuts(ctx: &egui::Context) {
    use egui::{Key, KeyboardShortcut, Modifiers};
    const SHORTCUTS: [KeyboardShortcut; 4] = [
        KeyboardShortcut::new(Modifiers::COMMAND, Key::S),                      // save recording
        KeyboardShortcut::new(Modifiers::COMMAND.plus(Modifiers::ALT), Key::S), // save time selection
        KeyboardShortcut::new(Modifiers::COMMAND, Key::O),                      // open file (.rrd, …)
        KeyboardShortcut::new(Modifiers::COMMAND.plus(Modifiers::SHIFT), Key::O), // import into recording
    ];
    ctx.input_mut(|input| {
        for shortcut in &SHORTCUTS {
            input.consume_shortcut(shortcut);
        }
    });
}

/// The backend speaks another protocol: a banner below the top bar, always on top, until the
/// viewer or the backend is updated (see `backend::ProtocolCheck`).
fn protocol_banner_ui(ctx: &egui::Context, text: &str, below_top_bar: Option<egui::Rect>) {
    let top = below_top_bar.map_or(40.0, |rect| rect.top() + 8.0);
    let palette = theme::current();
    egui::Area::new(egui::Id::new("protocol_mismatch_banner"))
        .order(egui::Order::Foreground)
        .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, top))
        .interactable(false)
        .show(ctx, |ui| {
            egui::Frame::new()
                .fill(palette.danger_subtle)
                .stroke(egui::Stroke::new(1.0, palette.danger))
                .corner_radius(6.0)
                .inner_margin(egui::Margin::symmetric(14, 8))
                .show(ui, |ui| {
                    ui.label(egui::RichText::new(text).color(palette.text_strong).strong());
                });
        });
}

/// The viewer only uses Rerun's code, not its services (see docs/NETWORK.md): no Share button
/// (its links point at rerun.io's web viewer), no update check against GitHub, and no Rerun Cloud
/// credentials stored by the `rerun` CLI (loading them refreshes them with Rerun's login service).
fn without_rerun_services(options: re_viewer::StartupOptions) -> re_viewer::StartupOptions {
    re_viewer::StartupOptions {
        hide_share_button: true,
        disable_update_check: true,
        disable_stored_credentials: true,
        // The web viewer's "try the native Rerun Viewer" tip (Rerun's installer, not ours).
        hide_native_viewer_tip: true,
        ..options
    }
}

/// `duration` before now, or now if the clock started more recently than that (on the web it
/// starts at page load, so subtracting can underflow).
fn ago(duration: Duration) -> Instant {
    let now = Instant::now();
    now.checked_sub(duration).unwrap_or(now)
}

/// A config change from the backend.
enum ConfigUpdate {
    /// The full config, e.g. after "Reload".
    Response(ConfigResponse),
    /// Published on Zenoh after any client changed it.
    Remote(factoryos_client::protocol::Config),
    /// The outcome of saving our own edits.
    FileError(Option<String>),
}

/// A step's agent, at the bottom of its pane: what it does for the current product, and its button.
/// The Robo Fitter's footer also has "Check reachability (agent)" for its saved `scene_file`.
fn agent_footer(
    ui: &mut egui::Ui,
    step: sidebar::AgentStep,
    product: &Rc<RefCell<product::ProductPane>>,
    scene_file: Option<String>,
    pending_action: &Rc<RefCell<Option<sidebar::Action>>>,
) {
    use sidebar::AgentStep;
    let (label, what) = agent_step_texts(step);
    let current = product.borrow().current().map(|(name, _)| name);
    ui.add_space(12.0);
    ui.separator();
    ui.label(egui::RichText::new("AGENT").small().strong());
    ui.small(egui::RichText::new(what).weak());
    let icon = re_ui::icons::AGENT.as_image();
    let clicked = ui
        .add_enabled(current.is_some(), egui::Button::image_and_text(icon.clone(), label))
        .on_hover_text(format!(
            "Opens the agent chat with this step's prompt (Settings → its section){}",
            current.as_ref().map(|p| format!(", for product {p}")).unwrap_or_default()
        ))
        .on_disabled_hover_text("Save a product in Product & Task first")
        .clicked();
    if clicked {
        *pending_action.borrow_mut() = Some(sidebar::Action::Agent(step));
    }
    if step != AgentStep::CellLayout {
        return;
    }
    let (label, what) = agent_step_texts(AgentStep::ReachCheck);
    ui.small(egui::RichText::new(what).weak());
    let clicked = ui
        .add_enabled(scene_file.is_some(), egui::Button::image_and_text(icon, label))
        .on_hover_text(format!(
            "Opens the agent chat with the reachability prompt (Settings → Robo Fitter){}",
            scene_file.as_ref().map(|f| format!(", for scene {f}")).unwrap_or_default()
        ))
        .on_disabled_hover_text("Open a saved scene with robots first: the agent checks the saved file")
        .clicked();
    if clicked {
        *pending_action.borrow_mut() = Some(sidebar::Action::Agent(AgentStep::ReachCheck));
    }
}

/// A step's agent button and what it does.
fn agent_step_texts(step: sidebar::AgentStep) -> (&'static str, &'static str) {
    use sidebar::AgentStep;
    match step {
        AgentStep::TaskDescription => ("Create task description", "Works the product's tasks out into operations."),
        AgentStep::Dressing => ("Suggest robots & tools", "Which robot + tool pairs to dress for the product's tasks."),
        AgentStep::CellLayout => (
            "Propose cell layout",
            "Places robots and the part so every operation is reachable, as a scene to open here.",
        ),
        AgentStep::ReachCheck => (
            "Check reachability (agent)",
            "Moves each robot to every weld (IK), explains what it can't reach and proposes a fixed layout as a new scene.",
        ),
    }
}

/// The Product & Task pane; what it asks for goes to the app as a sidebar action.
fn product_ui(
    ui: &mut egui::Ui,
    product: &Rc<RefCell<product::ProductPane>>,
    pending_action: &Rc<RefCell<Option<sidebar::Action>>>,
) {
    if let Some(product::ProductAction::TaskDescription) = product.borrow_mut().ui(ui) {
        *pending_action.borrow_mut() = Some(sidebar::Action::Agent(sidebar::AgentStep::TaskDescription));
    }
}

/// Which of our side panels to draw, set each frame in `FactoryOsApp::ui` (it needs the Rerun app,
/// which is busy drawing when `outer_panels_ui` runs).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct OuterPanels {
    product: bool,
    dresser: bool,
    training: bool,
    fitter: bool,
    /// The behaviour-tree pane, with the simulation's 3D view (the Fitter page) only.
    bt: bool,
}

/// What a step that opens the Robo Fitter does once it is open.
#[derive(Clone, Debug, PartialEq, Eq)]
enum EditThen {
    OpenFitter,
    /// The Task Program editor beside the 3D view, on this task program (`None`: a new one).
    EditTask(Option<String>),
}

/// A question the user answers before an action that would close what the 3D view shows.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Confirm {
    /// Load & Play while the Fitter has unsaved changes to `file`.
    LoadPlan { path: String, cell: Option<String>, file: String },
    /// Open Fitter / Edit task while the Fitter shows `cell` (simulated: `bt`).
    /// `scene`: the fitted scene a reference cell references (`None`: a legacy cell).
    Edit { cell: String, bt: Option<String>, simulating: bool, scene: Option<String>, then: EditThen },
}

/// A confirmation's title, text and its two answers (yes, no).
fn confirm_texts(confirm: &Confirm) -> (&'static str, String, &'static str, &'static str) {
    match confirm {
        Confirm::LoadPlan { cell, file, .. } => (
            "Unsaved changes",
            format!(
                "The Robo Fitter has unsaved changes to {file}. Load & Play shows the simulation of {} in its 3D view \
                 and closes them.",
                cell.as_deref().unwrap_or("the active cell")
            ),
            "Discard and play",
            "Cancel",
        ),
        Confirm::Edit { cell, bt, simulating, scene, .. } => {
            let shows = match (simulating, bt) {
                (true, Some(bt)) => format!("the simulation of {cell} (behaviour tree {bt})"),
                (true, None) => format!("the simulation of {cell}"),
                (false, _) => format!("cell {cell}'s scene, read-only"),
            };
            let edits = match scene {
                Some(scene) => format!(
                    "Editing stops the simulation and opens its scene {scene} for editing: the cell references it, so \
                     saving marks the cell's plans and exports stale."
                ),
                None => "Editing stops the simulation and opens the scene the cell was made from (the cell itself is \
                         kept, with its own copy)."
                    .to_owned(),
            };
            (
                "Edit in the Robo Fitter",
                format!("The Robo Fitter shows {shows}. {edits}"),
                "Stop simulation and edit",
                "Just look",
            )
        }
    }
}

/// The top bar's chip: what the 3D view shows (see `view_mode`).
fn view_chip(ui: &mut egui::Ui, text: &str, fill: egui::Color32) {
    const MAX_CHARS: usize = 72;
    let short = if text.chars().count() > MAX_CHARS {
        format!("{}…", text.chars().take(MAX_CHARS - 1).collect::<String>())
    } else {
        text.to_owned()
    };
    egui::Frame::new()
        .fill(fill)
        .corner_radius(egui::CornerRadius::same(9))
        .inner_margin(egui::Margin::symmetric(8, 2))
        .show(ui, |ui| {
            ui.label(egui::RichText::new(short).small().color(egui::Color32::WHITE));
        })
        .response
        .on_hover_text(format!("The 3D view shows: {text}"));
}

/// The behaviour tree the simulation has loaded (its name), else the last one loaded.
fn loaded_plan(shared: &crate::backend::Shared) -> Option<String> {
    let last_loaded = match &shared.plan_result {
        Some(Ok(tree)) => Some(tree.as_str()),
        _ => None,
    };
    shared
        .cell_state
        .as_ref()
        .map(|state| &state["trajectory"])
        .filter(|trajectory| trajectory["loaded"] == true)
        .and_then(|trajectory| bt_pane::plan_name(trajectory, last_loaded))
        .or_else(|| last_loaded.map(str::to_owned))
}

/// What `outer_panels_ui` draws from.
struct OuterPanelState {
    shown: Rc<std::cell::Cell<OuterPanels>>,
    below_top_bar: Rc<std::cell::Cell<Option<egui::Rect>>>,
    chat: Rc<RefCell<chat::Chats>>,
    chat_open: Rc<std::cell::Cell<bool>>,
    dresser: Rc<RefCell<dresser::Dresser>>,
    training: Rc<RefCell<training_ui::TrainingUi>>,
    product: Rc<RefCell<product::ProductPane>>,
    pending_action: Rc<RefCell<Option<sidebar::Action>>>,
    fitter: Rc<RefCell<fitter::Fitter>>,
    cell: SharedBackend,
    bt_pane: Rc<RefCell<bt_pane::BtPane>>,
    hardware: Rc<RefCell<hardware_ui::HardwareUi>>,
    /// While the Workspace IDE is shown, only the chat stays beside it.
    ide_shown: Rc<std::cell::Cell<bool>>,
    /// Rerun's right panel is expanded (its top-bar toggle): the behaviour-tree pane shows with it.
    right_panel_open: Rc<std::cell::Cell<bool>>,
}

/// Our side panels, drawn by Rerun right after its top bar: below it, outermost first, so the
/// planner chat takes the window's right edge and the tool panels sit to its left. The
/// behaviour-tree pane comes last: a bottom panel under Rerun, between our side panels.
fn outer_panels_ui(ui: &mut egui::Ui, state: &OuterPanelState) {
    theme::with_controls(ui, |ui| outer_panels_controls_ui(ui, state));
}

fn outer_panels_controls_ui(ui: &mut egui::Ui, state: &OuterPanelState) {
    state.below_top_bar.set(Some(ui.available_rect_before_wrap()));
    if state.chat_open.get() {
        // At most 60% of the window, so in a narrow one (the web viewer) the other panels stay
        // usable and the chat's resize edge stays reachable.
        // Wide enough by default for the sessions column beside the chat (see `chat`); a new id,
        // so the width saved for the narrower pane before it does not apply.
        let max_width = (ui.available_width() * 0.6).min(860.0);
        egui::Panel::right("factory_os_chat_sessions")
            .resizable(true)
            .default_size(640.0_f32.min(max_width))
            .min_size(300.0_f32.min(max_width))
            .max_size(max_width)
            // Kiro Crew's chat: on the page colour, its own header (see `chat`).
            .frame(egui::Frame::new().fill(theme::palette(ui).bg))
            .show(ui, |ui| {
                if state.chat.borrow_mut().ui(ui) {
                    state.chat_open.set(false);
                }
            });
    }

    let shown = state.shown.get();
    if shown.product {
        egui::Panel::right("product_task")
            .default_size(420.0)
            .min_size(320.0)
            .show(ui, |ui| {
                pane_with_actions_ui(
                    ui,
                    "Product & Task",
                    "The part to produce and its task (weld spots).",
                    |ui| state.product.borrow_mut().actions_ui(ui),
                    |ui| product_ui(ui, &state.product, &state.pending_action),
                );
            });
    }
    if shown.dresser {
        // Right panel with stacked, resizable panes — like Rerun's left panel.
        egui::Panel::right("robo_dresser")
            .default_size(340.0)
            .min_size(280.0)
            .frame(egui::Frame::new().fill(ui.visuals().panel_fill))
            .show(ui, |ui| {
                ui.spacing_mut().item_spacing.y = 0.0;
                let available_height = ui.available_height();
                egui::Panel::top("robo_dresser_catalogue")
                    .frame(egui::Frame::new())
                    .resizable(true)
                    .min_size(80.0)
                    .max_size((available_height - 150.0).max(80.0))
                    .default_size(available_height * 0.45)
                    .show(ui, |ui| {
                        pane_ui(ui, "Catalogue", "Robots, tools and dressed robots.", |ui| {
                            state.dresser.borrow_mut().catalogue_ui(ui);
                        });
                    });
                pane_with_actions_ui(
                    ui,
                    "Robo Dresser",
                    "Dress options, output, diagnostics and joints.",
                    |ui| state.dresser.borrow_mut().actions_ui(ui),
                    |ui| {
                        state.dresser.borrow_mut().ui(ui);
                        agent_footer(ui, sidebar::AgentStep::Dressing, &state.product, None, &state.pending_action);
                    },
                );
            });
    }
    if shown.training {
        egui::Panel::right("training_job")
            .default_size(380.0)
            .min_size(300.0)
            .show(ui, |ui| {
                pane_ui(ui, TRAINING_TITLE, TRAINING_HOVER, |ui| state.training.borrow_mut().ui(ui));
            });
    }
    if shown.fitter {
        egui::Panel::right("robo_fitter")
            .default_size(340.0)
            .min_size(280.0)
            .show(ui, |ui| {
                pane_with_actions_ui(
                    ui,
                    "Robo Fitter",
                    "Scene, items, inspector, save and USDA export.",
                    |ui| state.fitter.borrow_mut().actions_ui(ui),
                    |ui| {
                        state.fitter.borrow_mut().ui(ui);
                        let scene_file = state.fitter.borrow().agent_scene_file();
                        agent_footer(ui, sidebar::AgentStep::CellLayout, &state.product, scene_file, &state.pending_action);
                    },
                );
            });
    }
    if state.hardware.borrow().open && !state.ide_shown.get() {
        let max_width = (ui.available_width() * 0.6).min(760.0);
        egui::Panel::right("factory_os_hardware")
            .resizable(true)
            .default_size(560.0_f32.min(max_width))
            .min_size(320.0_f32.min(max_width))
            .max_size(max_width)
            .frame(egui::Frame::new().fill(ui.visuals().panel_fill).inner_margin(8.0))
            .show(ui, |ui| state.hardware.borrow_mut().panel_ui(ui));
    }
    // Whenever the simulation plays a plan and its 3D view is shown: the tree in a right-hand
    // panel like the tools', the title bar and the cell's timeline under the 3D view; folded to
    // the title bar by its own toggle.
    let mut cell = state.cell.lock().unwrap();
    if shown.bt && bt_pane::BtPane::wanted(&cell) {
        let mut pane = state.bt_pane.borrow_mut();
        pane.prepare(ui.ctx(), &cell);
        // The tree beside the 3D view goes with Rerun's right panel (the top bar's toggle hides both).
        if pane.open && state.right_panel_open.get() {
            bt_tree_panel(ui).show(ui, |ui| pane.tree_ui(ui));
        }
        let frame = egui::Frame::new().fill(ui.visuals().panel_fill).inner_margin(4.0);
        // Folded, another panel (one id would keep the open pane's height).
        let panel = if pane.open {
            let max = (ui.available_height() * 0.5).max(120.0);
            egui::Panel::bottom("factory_os_bt_timeline").resizable(true).default_size(230.0_f32.min(max)).min_size(110.0).max_size(max)
        } else {
            egui::Panel::bottom("factory_os_bt_pane_folded").resizable(false)
        }
        .frame(frame);
        panel.show(ui, |ui| pane.bottom_ui(ui, &mut cell));
    }
}

/// The right-hand panel the behaviour tree shows in beside the 3D view: resizable by its left
/// edge (egui keeps the width it is dragged to, per id), 520 points wide at first.
fn bt_tree_panel(ui: &egui::Ui) -> egui::Panel {
    let max_width = (ui.available_width() * 0.6).max(320.0);
    // The tree's canvas is an egui::Scene, a layer of its own above this panel's: flush with the
    // panel's edge it would take the pointer from the resize handle on the inside (and the 3D
    // view, drawn after the panels, takes it on the outside), so the pane could only be resized
    // beside the toolbar and the inspector. The margin keeps the canvas clear of the handle.
    let grab = ui.style().interaction.resize_grab_radius_side.ceil().clamp(0.0, 100.0) as i8 + 1;
    egui::Panel::right("factory_os_bt_tree")
        .resizable(true)
        .default_size(520.0_f32.min(max_width))
        .min_size(320.0_f32.min(max_width))
        .max_size(max_width)
        .frame(egui::Frame::new().fill(ui.visuals().panel_fill).inner_margin(egui::Margin { left: grab, ..Default::default() }))
}

const TRAINING_TITLE: &str = "Build training job";
const TRAINING_HOVER: &str = "The cells a planner policy trains on (robots, products, welds), the scenes and the training; saved as a plan.";

/// The title of the Robo Fitter's upper pane beside the 3D view: its scenes, and what can be added
/// to the open scene (its "Add to scene" lists). The pane's section id stays `robo_fitter_palette`
/// (Rerun keeps the panes' sizes by it).
const FITTER_SCENES_TITLE: &str = "Scene editor";

/// A side-panel pane with a title bar like Rerun's own "Sources" pane, and scrollable content.
fn pane_ui(ui: &mut egui::Ui, title: &str, hover_text: &str, add_contents: impl FnOnce(&mut egui::Ui)) {
    ui.panel_content(|ui| {
        ui.panel_title_bar(title, Some(hover_text));
    });
    pane_body_ui(ui, title, add_contents);
}

/// [`pane_ui`] with the pane's main actions in a row pinned to its bottom ([`theme::action_row`]):
/// they stay in view while the content scrolls. `actions` is drawn first (it takes its room from
/// the bottom), then `add_contents` above it.
fn pane_with_actions_ui(
    ui: &mut egui::Ui,
    title: &str,
    hover_text: &str,
    actions: impl FnOnce(&mut egui::Ui),
    add_contents: impl FnOnce(&mut egui::Ui),
) {
    ui.panel_content(|ui| {
        ui.panel_title_bar(title, Some(hover_text));
    });
    theme::action_row(ui, egui::Id::new(("pane_actions", title)), |ui| ui.panel_content(actions));
    pane_body_ui(ui, title, add_contents);
}

/// A pane's scrollable content, below its title bar (and above its action row).
fn pane_body_ui(ui: &mut egui::Ui, title: &str, add_contents: impl FnOnce(&mut egui::Ui)) {
    egui::ScrollArea::vertical()
        .id_salt(title)
        .auto_shrink([false, false])
        .show(ui, |ui| {
            ui.panel_content(|ui| {
                ui.spacing_mut().item_spacing.y = 4.0;
                theme::controls_style(ui);
                add_contents(ui);
                ui.add_space(8.0);
            });
        });
}

/// Content of the non-recording pages, drawn in place of Rerun's welcome screen.
/// The planner agent in the checkout's `.kiro/agents/`.
#[cfg(not(target_arch = "wasm32"))]
const PLANNER_AGENT: &str = "factoryos-planner";

/// A prompt for the planner agent, sent once it and the simulation are up.
#[cfg(not(target_arch = "wasm32"))]
struct PlanningKickoff {
    prompt: String,
    /// Wait for the simulation too (planning uses the planner's MCP tools, which run in it).
    needs_simulation: bool,
    /// When the planner's session mode was requested.
    mode_requested: Option<Instant>,
}

/// A path as a single-quoted shell word.
#[cfg(not(target_arch = "wasm32"))]
fn shell_quote(path: &str) -> String {
    format!("'{}'", path.replace('\'', r"'\''"))
}

/// An agent prompt `template` with its `{placeholder}`s replaced, in order.
fn fill_agent_prompt(template: &str, values: &[(&str, &str)]) -> String {
    values.iter().fold(template.to_owned(), |text, (key, value)| text.replace(key, value))
}

/// The absolute scene file the reachability agent checks: a cell's own `scene.yaml` (`cell` =
/// its cells folder and name), else scene `file`'s (a name) `scene.yaml` in the scenes folder.
fn reach_scene_path(fitted_robots_dir: &str, file: &str, cell: Option<(&str, &str)>) -> String {
    let path = match cell {
        Some((cells_dir, cell)) if !cells_dir.is_empty() => std::path::Path::new(cells_dir).join(cell).join(file),
        _ => {
            let name = file.strip_suffix(".yaml").or_else(|| file.strip_suffix(".yml")).unwrap_or(file);
            std::path::Path::new(fitted_robots_dir).join(name).join("scene.yaml")
        }
    };
    path.to_string_lossy().into_owned()
}

/// The planner's id for `cell`: as the backend's `sanitize_cell_id`, a ROS namespace.
fn planner_cell_id(cell: &str) -> String {
    let id: String = cell.chars().map(|c| if c.is_ascii_alphanumeric() { c } else { '_' }).collect();
    if id.starts_with(|c: char| c.is_ascii_digit()) { format!("cell_{id}") } else { id }
}

/// The "Plan prompt" `template` (the default if empty) for `cell`, its `task` program and the
/// `planner` picked for it (empty: `auto`).
fn planning_prompt(template: &str, cell: &str, task: &str, planner: &str) -> String {
    let template = if template.trim().is_empty() { factoryos_client::protocol::config::DEFAULT_PLAN_PROMPT } else { template };
    let task = if task.is_empty() { "(none yet)" } else { task };
    let planner = if planner.is_empty() { "auto" } else { planner };
    template
        .replace("{cell_id}", &planner_cell_id(cell))
        .replace("{cell}", cell)
        .replace("{task}", task)
        .replace("{planner}", planner)
}



/// Plan Cell's words for a cell without a task program: its products' welds are the task (the planner
/// router's `task_program_from_products`), or there are none and the agent asks before inventing any.
fn no_task_program_note(plans: &factoryos_client::protocol::PlansInfo, task: &str) -> String {
    if !task.is_empty() {
        return String::new();
    }
    if plans.product_welds > 0 {
        format!(
            " The cell has no task program yet, but its product {} defines {} welds (Product & Task): call \
             `task_program_from_products` first and plan the task program it writes. Don't invent, add or move weld points.",
            plans.products.join(", "),
            plans.product_welds
        )
    } else {
        " The cell has no task program yet and no product with welds: call `task_program_from_products` to confirm, \
         then tell me which weld points you propose from the cell's fixtures and ask me before you save them and plan."
            .to_owned()
    }
}

/// The steps of the flow, as on the start page.
const FLOW: [(&str, &str); 8] = [
    ("Product & Task", "The part to produce and what to do to it, e.g. the spots to weld."),
    ("Robo Dresser", "Equip robots with their tools: one URDF with collision hulls per pair."),
    ("Robo Fitter", "Place the dressed robots and fixtures in a scene, and create the cell from it."),
    (
        "Agentic Planning",
        "An agent turns the product's task into the cell's task program and plans collision-free robot \
         motion for it, saved as a behaviour tree.",
    ),
    ("Behaviour Tree Executor", "Run the plan in the cell's 3D simulation and check it."),
    (
        "Control Code Generation",
        "Turn the behaviour tree into PLC code (IEC 61131-3, e.g. Structured Text) and robot programs \
         (e.g. KUKA KRL).",
    ),
    (
        "Virtual Commissioning",
        "Boot a virtual PLC and virtual robot controllers, and run the generated code on them.",
    ),
    (
        "Physics Validation",
        "Run the code against a physics simulation of the cell (e.g. NVIDIA Isaac Sim or MuJoCo) for the final \
         check before it goes to the real cell.",
    ),
];

/// The start page: what Physical AI Production Designer is for, and its flow.
fn home_ui(ui: &mut egui::Ui) {
    egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
        ui.vertical_centered(|ui| {
            ui.set_max_width(640.0);
            ui.add_space((ui.available_height() * 0.15).max(24.0));
            ui.heading("Physical AI Production Designer");
            ui.add_space(6.0);
            ui.label(
                "Design a robotic production cell from the product it makes, then plan and run it, with agents \
                 doing the heavy lifting.",
            );
            ui.add_space(18.0);
            for (index, (title, text)) in FLOW.iter().enumerate() {
                ui.horizontal(|ui| {
                    // Fixed width, so the titles line up whatever the digit.
                    ui.add_sized([22.0, 22.0], egui::Label::new(egui::RichText::new(format!("{}", index + 1)).strong().size(18.0)));
                    ui.vertical(|ui| {
                        ui.strong(*title);
                        ui.label(egui::RichText::new(*text).weak());
                    });
                });
                ui.add_space(8.0);
            }
            ui.add_space(10.0);
            ui.label(
                egui::RichText::new(
                    "Start with Designer Workflow on the left. The gear at the top right holds the settings.",
                )
                .weak(),
            );
        });
    });
}

/// Where the Robo Fitter's Scene editor and panel go.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct FitterPanes {
    /// Into Rerun's right panel (the Fitter's preview is shown).
    in_right_panel: bool,
    /// A side panel of their own (before its first preview).
    own_panel: bool,
}

/// The Robo Fitter's Scene editor and panel are for the user who opened the Fitter (its sidebar box,
/// Open Cell, a task program: `fitter_tool`). When the Fitter page only shows the simulation of a
/// behaviour tree (Load & Play in step 1), it is a viewer: the 3D view, the behaviour-tree pane
/// and the chat, no Scene editor or panel. The live cell is watched on the Live cell page (Watch the
/// cell), not here.
fn fitter_panes(page: Page, fitter_preview_shown: bool, fitter_tool: bool) -> FitterPanes {
    let tool = page == Page::Fitter && fitter_tool;
    FitterPanes { in_right_panel: tool && fitter_preview_shown, own_panel: tool && !fitter_preview_shown }
}

/// Our panels around the 3D view, none while the code page replaces it (the chat and the Hardware
/// Configuration, step 2's, stay: they are not in [`OuterPanels`]).
fn shown_panels(panels: OuterPanels, code_page: bool) -> OuterPanels {
    if code_page { OuterPanels::default() } else { panels }
}

/// Whether `action` goes elsewhere than the code page (another tool, the cell, the simulation).
fn leaves_code_page(action: &sidebar::Action) -> bool {
    use sidebar::Action;
    match action {
        Action::OpenProduct
        | Action::OpenDresser
        | Action::OpenFitter
        | Action::OpenCell(_)
        | Action::EditScene
        | Action::UseScene { open: true, .. }
        | Action::ViewLogs
        | Action::WatchCell
        | Action::OpenTrainingJob
        | Action::EditTask(_)
        | Action::EditProductTask { .. }
        | Action::LoadPlan { .. }
        | Action::Trajectory(..) => true,
        Action::Agent(_)
        | Action::AskAgent { .. }
        | Action::PlanCell { .. }
        | Action::SetActiveCell(_)
        | Action::UseScene { open: false, .. }
        | Action::RefreshCells
        | Action::RefreshPlans
        | Action::SetWeldLabels(_) => false,
    }
}

/// The cell reached production from the GUI: switch to its live view from these pages (the start
/// page, the Fitter as a viewer, the cell's own pages), not from a tool the user opened.
fn watch_on_production(page: Page, fitter_tool: bool) -> bool {
    match page {
        Page::Home | Page::LiveCell => true,
        Page::Fitter => !fitter_tool,
        Page::BackendLogs | Page::Product | Page::Dresser | Page::Training => false,
    }
}

/// Take the automation's new commands (empty without automation).
fn automation_requests(automation: &mut Option<crate::automation::Automation>) -> Vec<crate::automation::Request> {
    automation.as_mut().map(|automation| automation.take_requests()).unwrap_or_default()
}

/// A page's name in the automation's replies.
fn page_name(page: Page) -> &'static str {
    match page {
        Page::Home => "home",
        Page::BackendLogs => "backend_logs",
        Page::LiveCell => "live",
        Page::Product => "product",
        Page::Dresser => "dresser",
        Page::Fitter => "fitter",
        Page::Training => "training",
    }
}

/// The tool a recording belongs to, by its application id (the backend's previews).
fn tool_page(store: &StoreId) -> Option<Page> {
    match store.application_id().to_string().as_str() {
        "product_task" => Some(Page::Product),
        "robo_dresser" => Some(Page::Dresser),
        "robo_fitter" => Some(Page::Fitter),
        "training_job" => Some(Page::Training),
        _ => None,
    }
}

fn page_ui(ui: &mut egui::Ui, page: Page, cell: &SharedBackend) {
    match page {
        Page::Home => home_ui(ui),
        Page::Product => {
            ui.centered_and_justified(|ui| {
                ui.label("Opening the product preview…");
            });
        }
        Page::LiveCell => {
            ui.centered_and_justified(|ui| {
                ui.label("Waiting for the live cell's stream…");
            });
        }
        Page::Fitter => {
            ui.centered_and_justified(|ui| {
                ui.label("Opening the Robo Fitter scene…");
            });
        }
        Page::Training => {
            ui.centered_and_justified(|ui| {
                ui.label("Reading the active cell for the training job…");
            });
        }
        Page::Dresser => {
            ui.centered_and_justified(|ui| {
                ui.label("Select a robot, tool or dressed robot in the Robo Dresser catalogue to preview it.");
            });
        }
        Page::BackendLogs => {
            egui::Frame::new().inner_margin(24).show(ui, |ui| {
                ui.heading("Backend Logs");
                ui.separator();
                let cell = cell.lock().unwrap();
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        if cell.log.is_empty() {
                            ui.weak("Backend not started yet.");
                        }
                        for line in &cell.log {
                            ui.monospace(line);
                        }
                    });
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The behaviour tree's pane as the viewer lays it out (the tree beside a 3D view that takes
    /// the pointer where nothing else does), dragged by its left edge from `from` by `by`
    /// points: the width it is left with, as egui keeps it for the next frames.
    fn drag_the_bt_pane(from: egui::Pos2, by: f32) -> f32 {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../plan2cell/test_data/bt_1790934549_welding_cell_2_small.json");
        let mut tree = bt_view::BtPanel::new("factory_os_bt_pane");
        tree.load_text("welding.json", &std::fs::read_to_string(path).unwrap()).unwrap();
        let ctx = egui::Context::default();
        let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(1400.0, 900.0));
        let mut frame = |events: Vec<egui::Event>| {
            let input = egui::RawInput { screen_rect: Some(screen), events, ..Default::default() };
            let mut output = ctx.run_ui(input, |ui| {
                bt_tree_panel(ui).show(ui, |ui| tree.tree_ui(ui));
                // Rerun's 3D view, drawn after the panels, pans and orbits on a drag.
                let rest = ui.available_rect_before_wrap();
                ui.allocate_rect(rest, egui::Sense::click_and_drag());
            });
            output.textures_delta.clear();
        };
        let button = |pos, pressed| egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed, modifiers: Default::default() };
        for _ in 0..3 {
            frame(Vec::new());
        }
        frame(vec![egui::Event::PointerMoved(from)]);
        frame(vec![button(from, true)]);
        let to = from + egui::vec2(by, 0.0);
        for i in 1..=5 {
            frame(vec![egui::Event::PointerMoved(from + egui::vec2(by * i as f32 / 5.0, 0.0))]);
        }
        frame(vec![button(to, false)]);
        frame(Vec::new());
        egui::PanelState::load(&ctx, egui::Id::new("factory_os_bt_tree")).expect("the pane's state").size().x
    }

    /// The pane opens 520 points wide and its left edge drags it wider and narrower, beside the
    /// tree's canvas too (an egui::Scene, on a layer above the panel's, that took the pointer
    /// from the edge before) and beside its toolbar; within 320 points and 60% of the window.
    #[test]
    fn the_bt_pane_resizes_by_its_left_edge() {
        let edge = 1400.0 - 520.0;
        // Beside the canvas (under the toolbar, above the inspector), anywhere on the inner half
        // of the edge's handle (before, only within a point or two of the edge).
        let grab = egui::Style::default().interaction.resize_grab_radius_side;
        for y in [150.0, 330.0, 500.0] {
            for dx in [0.5, grab - 0.5] {
                // The edge goes where the pointer does.
                let wider = drag_the_bt_pane(egui::pos2(edge + dx, y), -100.0);
                assert!((wider - (620.0 - dx)).abs() < 1.0, "at {dx}, {y}: {wider}");
            }
        }
        let canvas = egui::pos2(edge + grab - 0.5, 330.0);
        let narrower = drag_the_bt_pane(canvas, 100.0);
        assert!((narrower - (420.0 - grab)).abs() < 1.0, "{narrower}");
        // Beside the toolbar.
        let toolbar = drag_the_bt_pane(egui::pos2(edge + grab - 0.5, 20.0), -100.0);
        assert!((toolbar - (620.0 - grab)).abs() < 1.0, "{toolbar}");
        // Within its limits.
        assert!((drag_the_bt_pane(canvas, 400.0) - 320.0).abs() < 2.0);
        assert!((drag_the_bt_pane(canvas, -800.0) - 840.0).abs() < 2.0);
    }

    #[test]
    fn the_plan_prompt_names_the_planner() {
        let default = planning_prompt("", "welding_cell_2_small", "", "hier_la_fu");
        assert!(default.contains("cell `welding_cell_2_small`"), "{default}");
        assert!(default.contains("task program `(none yet)`, planner `hier_la_fu`"), "{default}");
        assert!(default.contains("call `recommend_planner`") && !default.contains("{planner}"), "{default}");
        assert_eq!(planning_prompt("{cell_id} {task} {planner}", "2 cell", "t", ""), "cell_2_cell t auto");
        let mut plans = factoryos_client::protocol::PlansInfo::default();
        assert_eq!(no_task_program_note(&plans, "t"), "");
        assert!(no_task_program_note(&plans, "").contains("ask me before"));
        plans.products = vec!["bug_cube".to_owned()];
        plans.product_welds = 4;
        let note = no_task_program_note(&plans, "");
        assert!(note.contains("product bug_cube defines 4 welds") && note.contains("`task_program_from_products` first"));
    }

    #[test]
    fn the_reach_prompt_needs_no_product() {
        use factoryos_client::protocol::config::DEFAULT_REACH_PROMPT;
        let scene = reach_scene_path("/var/lib/factoryos/catalogue/scenes", "jam2", None);
        assert_eq!(scene, "/var/lib/factoryos/catalogue/scenes/jam2/scene.yaml");
        assert_eq!(reach_scene_path("/f", "jam2.yaml", None), "/f/jam2/scene.yaml", "a legacy file name");
        assert_eq!(reach_scene_path("/f", "scene.yaml", Some(("/cells", "cell_jam2"))), "/cells/cell_jam2/scene.yaml");
        assert_eq!(reach_scene_path("/f", "scene.yaml", Some(("", "cell_jam2"))), "/f/scene/scene.yaml", "no cells folder known");
        let prompt = fill_agent_prompt(
            DEFAULT_REACH_PROMPT,
            &[
                ("{scene_file}", &scene),
                ("{product}", ""),
                ("{fitted_robots_dir}", "/var/lib/factoryos/catalogue/scenes"),
                ("{cell}", "cell_jam2"),
            ],
        );
        assert!(prompt.contains("/var/lib/factoryos/catalogue/scenes/jam2/scene.yaml"), "{prompt}");
        assert!(prompt.contains("check_reachability"), "{prompt}");
        assert!(!prompt.contains("{scene_file}") && !prompt.contains("{fitted_robots_dir}") && !prompt.contains("{cell}"), "{prompt}");
        assert_eq!(fill_agent_prompt("{cell_id} {cell}", &[("{cell_id}", "a_b"), ("{cell}", "a b")]), "a_b a b", "longer keys first");
    }

    #[test]
    fn the_fitter_panes_only_for_the_fitter_tool() {
        let tool = fitter_panes(Page::Fitter, true, true);
        assert_eq!(tool, FitterPanes { in_right_panel: true, own_panel: false });
        assert_eq!(fitter_panes(Page::Fitter, false, true), FitterPanes { in_right_panel: false, own_panel: true });
        let none = FitterPanes { in_right_panel: false, own_panel: false };
        // The Fitter page as the viewer of the cell: no Scene editor, no Robo Fitter panel.
        assert_eq!(fitter_panes(Page::Fitter, true, false), none);
        assert_eq!(fitter_panes(Page::Fitter, false, false), none);
        for page in [Page::Home, Page::LiveCell, Page::Dresser, Page::Product] {
            assert_eq!(fitter_panes(page, true, true), none, "{page:?}");
        }
    }

    #[test]
    fn production_switches_to_the_cell_unless_a_tool_is_open() {
        assert!(watch_on_production(Page::Home, false));
        assert!(watch_on_production(Page::LiveCell, false));
        assert!(watch_on_production(Page::Fitter, false), "the Fitter as the simulation's viewer");
        assert!(!watch_on_production(Page::Fitter, true), "the Fitter opened as a tool");
        for page in [Page::Dresser, Page::Product, Page::Training, Page::BackendLogs] {
            assert!(!watch_on_production(page, false), "{page:?}");
        }
    }

    #[test]
    fn editing_a_shown_cell_asks_first() {
        let edit = Confirm::Edit {
            cell: "weld".into(),
            bt: Some("bt_a".into()),
            simulating: true,
            scene: None,
            then: EditThen::OpenFitter,
        };
        let (_, text, yes, no) = confirm_texts(&edit);
        assert_eq!(
            text,
            "The Robo Fitter shows the simulation of weld (behaviour tree bt_a). Editing stops the simulation and opens \
             the scene the cell was made from (the cell itself is kept, with its own copy)."
        );
        assert_eq!((yes, no), ("Stop simulation and edit", "Just look"));
        // A reference cell: its fitted scene opens for editing.
        let reference = Confirm::Edit {
            cell: "jam2".into(),
            bt: None,
            simulating: false,
            scene: Some("jam2.yaml".into()),
            then: EditThen::OpenFitter,
        };
        assert_eq!(
            confirm_texts(&reference).1,
            "The Robo Fitter shows cell jam2's scene, read-only. Editing stops the simulation and opens its scene \
             jam2.yaml for editing: the cell references it, so saving marks the cell's plans and exports stale."
        );
        let load = Confirm::LoadPlan { path: "p".into(), cell: Some("weld".into()), file: "two.yaml".into() };
        let (_, text, yes, _) = confirm_texts(&load);
        assert!(text.starts_with("The Robo Fitter has unsaved changes to two.yaml. Load & Play shows the simulation of weld"), "{text}");
        assert_eq!(yes, "Discard and play");
    }

    #[test]
    fn rerun_services_are_off() {
        let options = without_rerun_services(re_viewer::StartupOptions::default());
        assert!(options.hide_share_button);
        assert!(options.disable_update_check);
        assert!(options.disable_stored_credentials);
        assert!(options.hide_native_viewer_tip);
    }

    #[test]
    fn the_code_page_hides_the_panes_around_the_3d_view() {
        let around = OuterPanels { product: false, dresser: false, training: false, fitter: true, bt: true };
        let none = OuterPanels { product: false, dresser: false, training: false, fitter: false, bt: false };
        assert_eq!(shown_panels(around, true), none, "no Behaviour Tree pane, no Robo Fitter panel");
        assert_eq!(shown_panels(around, false), around, "back as they were");
    }

    #[test]
    fn going_elsewhere_leaves_the_code_page() {
        use sidebar::Action;
        for action in [Action::OpenFitter, Action::OpenDresser, Action::WatchCell, Action::OpenCell("weld".into())] {
            assert!(leaves_code_page(&action), "{action:?}");
        }
        assert!(leaves_code_page(&Action::LoadPlan { path: "p".into(), cell: None }));
        assert!(leaves_code_page(&Action::UseScene { scene: "s.yaml".into(), open: true }));
        for action in [
            Action::RefreshPlans,
            Action::RefreshCells,
            Action::SetActiveCell("weld".into()),
            Action::UseScene { scene: "s.yaml".into(), open: false },
        ] {
            assert!(!leaves_code_page(&action), "{action:?}: stays on the code");
        }
    }
}
