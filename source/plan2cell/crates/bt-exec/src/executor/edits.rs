//! Breakpoints, and edits of port and blackboard values.
//!
//! A breakpoint holds a node *before* it starts: the node stays IDLE and its parent sees
//! RUNNING. It holds at most once per activation of the parent, so a `ReactiveSequence` that
//! re-ticks its first child every tick does not pause again and again. Resuming lets the node
//! start on the next tick.
//!
//! Edits change what a node reads from then on ([`Executor::set_port_value`] says when that is).
//! They survive [`Executor::reset`], can be reverted one by one or all at once, and never re-tick
//! by themselves: an edit between events must not make anything happen earlier than it would
//! have.

use std::collections::BTreeMap;

use bt_core::NodeStatus;
use bt_core::builtins::{POSTCONDITIONS, PRECONDITIONS, is_reserved_attribute};
use bt_core::model::{NodeKind, Port, PortDirection, Uid};
use serde_json::Value;

use super::Executor;

impl Executor {
    // ─────────────────────────────── breakpoints ───────────────────────────────

    /// Pause before `uid` starts (any node but the root).
    pub fn set_breakpoint(&mut self, uid: Uid, on: bool) {
        if uid == 0 || uid >= self.tree.len() {
            return;
        }
        if on {
            self.breakpoints[uid] = true;
            self.bp_spent[uid] = false;
        } else {
            // Only a node it holds can turn out differently; a RUNNING node is never held.
            if self.paused.contains(&uid) {
                self.dirty = true;
            }
            self.breakpoints[uid] = false;
            self.pass[uid] = false;
            self.paused.retain(|&p| p != uid);
        }
    }

    /// Whether `uid` has a breakpoint.
    pub fn breakpoint(&self, uid: Uid) -> bool {
        self.breakpoints[uid]
    }

    /// Nodes with a breakpoint.
    pub fn breakpoints(&self) -> Vec<Uid> {
        (0..self.tree.len())
            .filter(|&uid| self.breakpoints[uid])
            .collect()
    }

    /// Remove every breakpoint (paused nodes start on the next tick).
    pub fn clear_breakpoints(&mut self) {
        for uid in self.breakpoints() {
            self.set_breakpoint(uid, false);
        }
    }

    /// Nodes held at their breakpoint by the last tick.
    pub fn paused_at(&self) -> &[Uid] {
        &self.paused
    }

    /// Let every paused node start on the next tick.
    pub fn resume(&mut self) {
        for &uid in &self.paused {
            self.pass[uid] = true;
        }
        if !self.paused.is_empty() {
            self.dirty = true;
        }
    }

    /// Let one paused node start on the next tick.
    pub fn resume_node(&mut self, uid: Uid) {
        if self.paused.contains(&uid) {
            self.pass[uid] = true;
            self.dirty = true;
        }
    }

    /// Whether the breakpoint on `uid` holds it in this tick (called after its pre-conditions).
    pub(super) fn holds_at_breakpoint(&mut self, uid: Uid) -> bool {
        if !self.breakpoints[uid]
            || self.bp_spent[uid]
            || self.states[uid].status == NodeStatus::Running
        {
            return false;
        }
        // Resumed, or decided by an override instead: either way this activation's hold is used
        // up, so a parent that ticks it again (a `ReactiveSequence`) does not pause it again.
        let forced = self.overrides[uid].is_some();
        if std::mem::take(&mut self.pass[uid]) || forced {
            self.bp_spent[uid] = true;
            return false;
        }
        if !self.paused.contains(&uid) {
            self.paused.push(uid);
        }
        if !self.paused_prev.contains(&uid) {
            self.message(Some(uid), "breakpoint: paused before it starts".to_string());
        }
        self.holds += 1;
        true
    }

    // ──────────────────────────────── port edits ────────────────────────────────

    /// Set a port's attribute to `value`, as if the file said so. A port the node does not
    /// have yet is added (as an undeclared in/out port).
    ///
    /// Nothing is re-ticked: the node sees the value the next time it reads the port. A native
    /// leaf reads its ports when it starts, so a RUNNING one keeps what it read. `Sleep`,
    /// `Timeout`, `Delay` and controls with ports (`Parallel`, `Repeat`, `RetryUntilSuccessful`,
    /// `Switch`, `BlackboardCheck`, …) read them on every tick, so a running one picks the edit
    /// up at its next event: a timer edited while it runs is measured from its start again then,
    /// and one shortened past that point finishes at that event.
    pub fn set_port_value(&mut self, uid: Uid, port: &str, value: &str) -> Result<(), String> {
        let Some(node) = self.tree.nodes.get(uid) else {
            return Err(format!("no node {uid}"));
        };
        if uid == 0 {
            return Err("the root has no ports".to_string());
        }
        if node.kind == NodeKind::SubTree {
            return Err(format!(
                "SubTree {} remaps the blackboard; edit the entry instead",
                node.name
            ));
        }
        let port = port.trim();
        if port.is_empty()
            || is_reserved_attribute(port)
            || PRECONDITIONS.contains(&port)
            || POSTCONDITIONS.contains(&port)
        {
            return Err(format!("`{port}` is not a port"));
        }
        let key = (uid, port.to_string());
        let node = &mut self.tree.nodes[uid];
        match node.ports.iter_mut().find(|p| p.name == port) {
            Some(p) => {
                let original = self
                    .port_originals
                    .entry(key.clone())
                    .or_insert_with(|| p.value.clone());
                if original.as_deref() == Some(value) {
                    self.port_originals.remove(&key);
                }
                p.value = Some(value.to_string());
            }
            None => {
                node.ports.push(Port {
                    name: port.to_string(),
                    direction: PortDirection::InOut,
                    value: Some(value.to_string()),
                    default: None,
                    description: String::new(),
                    declared: false,
                });
                self.port_originals.insert(key, None);
            }
        }
        Ok(())
    }

    /// Back to the file's value; a port the edit added is removed again.
    pub fn revert_port_value(&mut self, uid: Uid, port: &str) {
        let Some(original) = self.port_originals.remove(&(uid, port.to_string())) else {
            return;
        };
        let ports = &mut self.tree.nodes[uid].ports;
        let Some(i) = ports.iter().position(|p| p.name == port) else {
            return;
        };
        if original.is_none() && !ports[i].declared {
            ports.remove(i);
        } else {
            ports[i].value = original;
        }
    }

    /// For an edited port: what the file said (`Some(None)`: no attribute). `None` if unedited.
    pub fn port_original(&self, uid: Uid, port: &str) -> Option<Option<&str>> {
        self.port_originals
            .get(&(uid, port.to_string()))
            .map(Option::as_deref)
    }

    /// Edited ports, by node.
    pub fn port_edits(&self) -> Vec<(Uid, String)> {
        self.port_originals.keys().cloned().collect()
    }

    // ───────────────────────────── blackboard edits ─────────────────────────────

    /// Set a main-blackboard entry now and on every reset.
    pub fn set_blackboard_value(&mut self, key: &str, value: Value) {
        self.blackboards.set(0, key, value.clone());
        self.bb_edits.insert(key.to_string(), value);
    }

    /// Back to the file's value (or no entry, if the file had none).
    pub fn revert_blackboard_value(&mut self, key: &str) {
        if self.bb_edits.remove(key).is_none() {
            return;
        }
        match self.initial_blackboard.get(key) {
            Some(value) => self.blackboards.set(0, key, value.clone()),
            None => self.blackboards.remove(0, key),
        }
    }

    /// Whether main-blackboard entry `key` was edited.
    pub fn blackboard_edited(&self, key: &str) -> bool {
        self.bb_edits.contains_key(key)
    }

    /// Main-blackboard edits, by key.
    pub fn blackboard_edits(&self) -> &BTreeMap<String, Value> {
        &self.bb_edits
    }

    /// The main blackboard as the file gave it.
    pub fn initial_blackboard(&self) -> &BTreeMap<String, Value> {
        &self.initial_blackboard
    }

    /// Revert every port and blackboard edit.
    pub fn revert_all_edits(&mut self) {
        for (uid, port) in self.port_edits() {
            self.revert_port_value(uid, &port);
        }
        for key in self.bb_edits.keys().cloned().collect::<Vec<_>>() {
            self.revert_blackboard_value(&key);
        }
    }
}
