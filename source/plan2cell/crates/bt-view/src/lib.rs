//! A Groot2-style behaviour tree panel for egui.
//!
//! [`BtPanel`] draws a BehaviorTree.CPP tree (laid out with Groot's algorithm) and runs it with
//! [`bt_exec::Executor`], colouring nodes and edges with Groot's monitor colours. It only needs
//! an `&mut egui::Ui`, so it embeds in any egui app (eframe, the rerun viewer, …).
//!
//! It also loads factoryos plans (every layout cell-runner accepts) and, for a plan whose cell
//! leaves carry trajectories, runs those leaves with cell-runner's semantics ([`bt_cell`]):
//! motion is placed on a schedule as the tree runs, and the panel reports where and why the
//! tree stops ([`Stop`]).
//!
//! Where it halts, the panel says why and offers the decisions that let it go on: forcing a
//! node (control nodes too), switching a leaf's policy, releasing stalled gates, breakpoints,
//! and editing port and blackboard values before a node starts ([`Action`]). For a cell plan,
//! a cell panel under the canvas shows the robots, the motion placed so far as a Gantt chart,
//! and the warnings raised on the way.

mod canvas;
mod cell_panel;
mod edit;
pub mod fonts;
mod inspector;
mod panel;
pub mod style;
#[cfg(test)]
mod testing;

pub use panel::{Action, BtPanel, CellMode, Drift, PlanInfo};
// Re-exported so an embedder does not need bt-core / bt-exec / bt-cell in its own Cargo.toml.
pub use bt_cell;
pub use bt_core::{Document, NodeKind, NodeStatus, Tree, Uid};
pub use bt_exec::{Executor, LeafPolicy, Override, RunOptions, Stop};
