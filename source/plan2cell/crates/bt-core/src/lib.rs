//! BehaviorTree.CPP trees without a UI: the XML model, the builtin node manifest,
//! Groot's tree layout and the status types an executor reports.
//!
//! The model follows Groot's `BuildTreeFromXML` / `ReadTreeNodesModel` and extends it to
//! BTCPP_format 4 (bare custom tags, `SubTree` remapping with `{key}`).

pub mod builtins;
pub mod layout;
pub mod model;
pub mod status;

pub use model::{Document, Node, NodeKind, NodeModel, PortDirection, PortModel, Tree, Uid};
pub use status::{NodeStatus, Trace, Transition};
