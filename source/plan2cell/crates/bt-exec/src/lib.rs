//! A BehaviorTree.CPP 4 interpreter for watching a tree run.
//!
//! Control nodes and decorators behave like BT.CPP's; leaves that BT.CPP does not provide are
//! stubs driven by a [`LeafPolicy`] and can be forced with an [`Override`], so a tree can be
//! pushed past whatever it waits for. Every status change lands in a [`bt_core::Trace`].
//!
//! A [`LeafHandler`] gives stub leaves real semantics instead: installed with
//! [`Executor::set_leaf_handler`], it runs the leaves it claims ([`LeafPolicy::Native`]) through
//! a [`LeafCtx`] that reads ports and the blackboard, schedules wakeups and asks for a re-tick
//! at the same instant. [`Executor::run`] then ticks only at events, like cell-runner's virtual
//! clock, and reports where the tree halts ([`Stop`]).

pub mod blackboard;
pub mod executor;
pub mod leaf;
pub mod script;

pub use executor::{
    EPS, Executor, LeafPolicy, LogEntry, MAX_STALL_RELEASES, Override, RunOptions, RunReport, Stop,
    Wakeup, is_stub,
};
pub use leaf::{Dialect, HaltCause, LeafCtx, LeafHandler};
