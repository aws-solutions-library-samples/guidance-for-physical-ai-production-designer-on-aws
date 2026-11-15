//! The nodes BehaviorTree.CPP registers itself (`BehaviorTreeFactory`'s constructor), v4 plus
//! the v3 names Groot's samples still use.

use crate::model::{NodeKind, NodeModel, PortDirection, PortModel};

/// Name of the synthetic node Groot puts above a tree's first node.
pub const ROOT_ID: &str = "Root";

struct Builtin {
    id: &'static str,
    kind: NodeKind,
    /// `(name, direction, default, description)`
    ports: &'static [(
        &'static str,
        PortDirection,
        Option<&'static str>,
        &'static str,
    )],
}

use NodeKind::{Action, Condition, Control, Decorator};
use PortDirection::{InOut, Input};

const SWITCH_PORTS: &[(&str, PortDirection, Option<&str>, &str)] = &[
    ("variable", Input, None, ""),
    ("case_1", Input, None, ""),
    ("case_2", Input, None, ""),
    ("case_3", Input, None, ""),
    ("case_4", Input, None, ""),
    ("case_5", Input, None, ""),
    ("case_6", Input, None, ""),
];

const BLACKBOARD_CHECK_PORTS: &[(&str, PortDirection, Option<&str>, &str)] = &[
    ("value_A", Input, None, ""),
    ("value_B", Input, None, ""),
    ("return_on_mismatch", Input, None, ""),
];

const BUILTINS: &[Builtin] = &[
    // ── control ──
    Builtin {
        id: "Sequence",
        kind: Control,
        ports: &[],
    },
    Builtin {
        id: "SequenceWithMemory",
        kind: Control,
        ports: &[],
    },
    Builtin {
        id: "SequenceStar",
        kind: Control,
        ports: &[],
    },
    Builtin {
        id: "ReactiveSequence",
        kind: Control,
        ports: &[],
    },
    Builtin {
        id: "Fallback",
        kind: Control,
        ports: &[],
    },
    Builtin {
        id: "ReactiveFallback",
        kind: Control,
        ports: &[],
    },
    Builtin {
        id: "Parallel",
        kind: Control,
        ports: &[
            (
                "success_count",
                Input,
                Some("-1"),
                "number of children that must succeed (-1 = all)",
            ),
            (
                "failure_count",
                Input,
                Some("1"),
                "number of children that must fail (-1 = all)",
            ),
            // v3 names
            ("success_threshold", Input, None, ""),
            ("failure_threshold", Input, None, ""),
        ],
    },
    Builtin {
        id: "ParallelAll",
        kind: Control,
        ports: &[(
            "max_failures",
            Input,
            Some("1"),
            "failures (-1 = all) that make it fail",
        )],
    },
    Builtin {
        id: "IfThenElse",
        kind: Control,
        ports: &[],
    },
    Builtin {
        id: "WhileDoElse",
        kind: Control,
        ports: &[],
    },
    Builtin {
        id: "Switch2",
        kind: Control,
        ports: SWITCH_PORTS,
    },
    Builtin {
        id: "Switch3",
        kind: Control,
        ports: SWITCH_PORTS,
    },
    Builtin {
        id: "Switch4",
        kind: Control,
        ports: SWITCH_PORTS,
    },
    Builtin {
        id: "Switch5",
        kind: Control,
        ports: SWITCH_PORTS,
    },
    Builtin {
        id: "Switch6",
        kind: Control,
        ports: SWITCH_PORTS,
    },
    // ── decorators ──
    Builtin {
        id: "Inverter",
        kind: Decorator,
        ports: &[],
    },
    Builtin {
        id: "ForceSuccess",
        kind: Decorator,
        ports: &[],
    },
    Builtin {
        id: "ForceFailure",
        kind: Decorator,
        ports: &[],
    },
    Builtin {
        id: "KeepRunningUntilFailure",
        kind: Decorator,
        ports: &[],
    },
    Builtin {
        id: "Repeat",
        kind: Decorator,
        ports: &[(
            "num_cycles",
            Input,
            None,
            "repeat a successful child N times (-1 = forever)",
        )],
    },
    Builtin {
        id: "RetryUntilSuccessful",
        kind: Decorator,
        ports: &[(
            "num_attempts",
            Input,
            None,
            "attempts on failure (-1 = forever)",
        )],
    },
    Builtin {
        id: "Retry",
        kind: Decorator,
        ports: &[(
            "num_attempts",
            Input,
            None,
            "attempts on failure (-1 = forever)",
        )],
    },
    Builtin {
        id: "Timeout",
        kind: Decorator,
        ports: &[(
            "msec",
            Input,
            None,
            "after this many ms, halt the child and fail",
        )],
    },
    Builtin {
        id: "Delay",
        kind: Decorator,
        ports: &[(
            "delay_msec",
            Input,
            None,
            "tick the child after this many ms",
        )],
    },
    Builtin {
        id: "RunOnce",
        kind: Decorator,
        ports: &[(
            "then_skip",
            Input,
            Some("true"),
            "after the first run, skip (else return the first result)",
        )],
    },
    Builtin {
        id: "Precondition",
        kind: Decorator,
        ports: &[
            ("if", Input, None, "script condition"),
            (
                "else",
                Input,
                Some("FAILURE"),
                "status when the condition is false",
            ),
        ],
    },
    Builtin {
        id: "BlackboardCheckInt",
        kind: Decorator,
        ports: BLACKBOARD_CHECK_PORTS,
    },
    Builtin {
        id: "BlackboardCheckDouble",
        kind: Decorator,
        ports: BLACKBOARD_CHECK_PORTS,
    },
    Builtin {
        id: "BlackboardCheckString",
        kind: Decorator,
        ports: BLACKBOARD_CHECK_PORTS,
    },
    Builtin {
        id: "BlackboardCheckBool",
        kind: Decorator,
        ports: BLACKBOARD_CHECK_PORTS,
    },
    // ── leaves ──
    Builtin {
        id: "AlwaysSuccess",
        kind: Action,
        ports: &[],
    },
    Builtin {
        id: "AlwaysFailure",
        kind: Action,
        ports: &[],
    },
    Builtin {
        id: "Script",
        kind: Action,
        ports: &[("code", Input, None, "assignments, e.g. `a := 1; b := 'x'`")],
    },
    Builtin {
        id: "ScriptCondition",
        kind: Condition,
        ports: &[("code", Input, None, "expression")],
    },
    Builtin {
        id: "SetBlackboard",
        kind: Action,
        ports: &[
            ("value", Input, None, "value to write"),
            ("output_key", InOut, None, "blackboard entry to write"),
        ],
    },
    Builtin {
        id: "UnsetBlackboard",
        kind: Action,
        ports: &[("key", Input, None, "entry to remove")],
    },
    Builtin {
        id: "Sleep",
        kind: Action,
        ports: &[("msec", Input, None, "")],
    },
];

/// The builtin model with this registration ID.
pub fn model(id: &str) -> Option<NodeModel> {
    if id == ROOT_ID {
        return Some(NodeModel {
            id: ROOT_ID.to_string(),
            kind: NodeKind::Root,
            ports: Vec::new(),
            builtin: true,
        });
    }
    let b = BUILTINS.iter().find(|b| b.id == id)?;
    Some(NodeModel {
        id: b.id.to_string(),
        kind: b.kind,
        ports: b
            .ports
            .iter()
            .map(|&(name, direction, default, description)| PortModel {
                name: name.to_string(),
                direction,
                type_name: None,
                default: default.map(str::to_string),
                description: description.to_string(),
            })
            .collect(),
        builtin: true,
    })
}

pub fn is_builtin(id: &str) -> bool {
    id == ROOT_ID || BUILTINS.iter().any(|b| b.id == id)
}

/// Every builtin registration ID.
pub fn ids() -> impl Iterator<Item = &'static str> {
    BUILTINS.iter().map(|b| b.id)
}

/// Attributes that are neither ports nor pre/post conditions.
pub fn is_reserved_attribute(name: &str) -> bool {
    matches!(
        name,
        "ID" | "name" | "_autoremap" | "_description" | "_uid" | "_fullpath"
    )
}

/// BT.CPP v4 pre-conditions, checked before a node is ticked (`_skipIf`, …).
pub const PRECONDITIONS: &[&str] = &["_failureIf", "_successIf", "_skipIf", "_while"];
/// BT.CPP v4 post-conditions, scripts run when a node completes.
pub const POSTCONDITIONS: &[&str] = &["_onHalted", "_onFailure", "_onSuccess", "_post"];
