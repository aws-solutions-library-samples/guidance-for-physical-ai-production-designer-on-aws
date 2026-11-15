//! Blackboards: one for the main tree and one per SubTree instance, linked by port remapping.

use std::collections::{BTreeMap, HashMap};

use bt_core::Uid;
use bt_core::model::{NodeKind, Tree, blackboard_pointer};
use serde_json::Value;

/// How one SubTree blackboard reaches its parent.
#[derive(Clone, Debug, Default)]
pub struct ScopeDef {
    pub parent: Option<usize>,
    /// The SubTree node that opened this scope (`None` for the main tree).
    pub owner: Option<Uid>,
    /// Local key → parent key.
    pub remaps: HashMap<String, String>,
    /// Literal port values, written into the scope when it starts.
    pub literals: BTreeMap<String, String>,
    pub autoremap: bool,
}

#[derive(Clone, Debug, Default)]
pub struct Blackboards {
    pub scopes: Vec<ScopeDef>,
    /// Scope of each node.
    pub scope_of: Vec<usize>,
    pub entries: Vec<BTreeMap<String, Value>>,
}

impl Blackboards {
    pub fn new(tree: &Tree) -> Self {
        let mut scopes = vec![ScopeDef::default()];
        let mut scope_of = vec![0; tree.len()];
        for node in &tree.nodes {
            let parent_scope = node.parent.map_or(0, |p| scope_of[p]);
            // The children of a SubTree live in its scope; the SubTree node itself in the parent's.
            scope_of[node.uid] = match node.parent {
                Some(p) if tree.nodes[p].kind == NodeKind::SubTree => {
                    let subtree = &tree.nodes[p];
                    let def = subtree.subtree.as_ref();
                    let mut scope = ScopeDef {
                        parent: Some(scope_of[p]),
                        owner: Some(p),
                        autoremap: def.is_some_and(|d| d.autoremap),
                        ..Default::default()
                    };
                    let legacy = def.is_some_and(|d| d.legacy_remap);
                    for port in &subtree.ports {
                        let Some(value) = port.value.as_deref() else {
                            continue;
                        };
                        if let Some(key) = blackboard_pointer(value) {
                            scope.remaps.insert(port.name.clone(), key.to_string());
                        } else if legacy {
                            // v3 <SubTree>: `port="parent_key"`.
                            scope
                                .remaps
                                .insert(port.name.clone(), value.trim().to_string());
                        } else {
                            scope.literals.insert(port.name.clone(), value.to_string());
                        }
                    }
                    scopes.push(scope);
                    scopes.len() - 1
                }
                _ => parent_scope,
            };
        }
        let entries = vec![BTreeMap::new(); scopes.len()];
        Self {
            scopes,
            scope_of,
            entries,
        }
    }

    /// Clear everything and seed the main blackboard.
    pub fn reset(&mut self, initial: &BTreeMap<String, Value>) {
        for e in &mut self.entries {
            e.clear();
        }
        self.entries[0] = initial.clone();
    }

    /// A SubTree starts: its literal port values are written into its scope.
    pub fn enter(&mut self, scope: usize) {
        let literals = self.scopes[scope].literals.clone();
        for (k, v) in literals {
            self.entries[scope].insert(k, Value::String(v));
        }
    }

    /// The scope and key that `key` in `scope` actually refers to.
    pub fn resolve(&self, mut scope: usize, key: &str) -> (usize, String) {
        let mut key = key.to_string();
        loop {
            if self.entries[scope].contains_key(&key) {
                return (scope, key);
            }
            let def = &self.scopes[scope];
            match (def.parent, def.remaps.get(&key)) {
                (Some(parent), Some(parent_key)) => {
                    key = parent_key.clone();
                    scope = parent;
                }
                (Some(parent), None) if def.autoremap => scope = parent,
                _ => return (scope, key),
            }
        }
    }

    pub fn get(&self, scope: usize, key: &str) -> Option<Value> {
        let (scope, key) = self.resolve(scope, key);
        self.entries[scope].get(&key).cloned()
    }

    /// Writes go where reads come from, so a new key in a remapped or autoremapped scope
    /// is created in the parent, as in BT.CPP.
    pub fn set(&mut self, scope: usize, key: &str, value: Value) {
        let (scope, key) = self.resolve(scope, key);
        self.entries[scope].insert(key, value);
    }

    pub fn remove(&mut self, scope: usize, key: &str) {
        let (scope, key) = self.resolve(scope, key);
        self.entries[scope].remove(&key);
    }
}
