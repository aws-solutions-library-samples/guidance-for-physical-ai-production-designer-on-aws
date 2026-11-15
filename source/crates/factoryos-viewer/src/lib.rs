//! Physical AI Production Designer: the Rerun Viewer wrapped with a "Designer Workflow" sidebar and the tools' panels.
//!
//! Native binary in `main.rs`; on the web, `web.rs`'s `start` is called by `web/index.html`.

mod app;
/// Opt-in automation for visual smoke tests: over stdin natively, from the page on the web.
pub mod automation;
mod backend;
mod bt_pane;
mod catalogue_events;
#[cfg(not(target_arch = "wasm32"))]
pub mod backend_choice;
mod cell_list;
mod code_export;
mod code_layout;
mod code_view;
mod deploy_check_ui;
mod flow;
mod hardware_ui;
mod ide;
mod ide_terminal;
mod ide_transfer;
mod infra_ui;
/// The internal build's defaults (the chooser's dev server and its login hints, the Isaac Sim launch
/// link, the agent's sign-in defaults); not in the public export.
#[cfg(feature = "internal")]
mod internal;
mod isaac_ui;
mod labels_overlay;
mod live_queue;
mod chat;
mod config;
mod dresser;
mod dresser_tcp;
mod fitter;
mod product;
mod reconnect;
/// Exported for integration tests (`tests/scene_e2e.rs`); not an API.
#[doc(hidden)]
pub mod scene3d;
mod shared_publish;
mod sidebar;
mod term_view;
mod syntax;
mod task_editor;
mod training_ui;
mod theme;
mod ui_mode;
mod view_commands;
mod view_mode;
mod virtual_cell;
mod workspace_ui;
#[cfg(target_arch = "wasm32")]
mod web;

pub use app::{Startup, new_app};
pub use backend::{Backend, WorkspaceNotReady};
#[cfg(not(target_arch = "wasm32"))]
pub use backend::LoginNeeded;

/// The Rerun crates the viewer uses, under one path for native and web.
///
/// The `rerun` crate (SDK, gRPC server) is native-only, so these come from `re_viewer`.
pub(crate) mod rr {
    pub use re_viewer::external::*;
    pub use re_viewer;
    #[cfg(not(target_arch = "wasm32"))]
    pub use rerun::external::tokio;
}
