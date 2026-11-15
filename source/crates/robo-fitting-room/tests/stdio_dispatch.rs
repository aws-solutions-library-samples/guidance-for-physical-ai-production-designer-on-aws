//! Property-based test for the `stdio` transport's method dispatch totality.
//!
//! Gated behind the `stdio` feature so default-feature builds run it but a
//! `--no-default-features` build without `stdio` skips it. Run with:
//! `cargo test --features stdio --test stdio_dispatch`.
//!
//! ## Property 2: Total dispatch
//! **Validates: Requirements 3.2, 3.3, 3.4**
//!
//! For every `(method: String, params: Value)` pair, `dispatch` returns
//! either `Ok(Value)` or `Err(RpcError)` — it never panics and never fails
//! to produce a response. In particular:
//!
//! - Requirement 3.3: a method name outside the known dispatch table yields
//!   `Err` with the reserved `method_not_found` code (-32601).
//! - Requirement 3.4: a known, param-taking method given params that cannot
//!   deserialize into its expected shape yields `Err` with the reserved
//!   `invalid_params` code (-32602) — never a panic.
//!
//! (Requirement 3.2 — malformed JSON → parse error — is enforced one layer
//! up in `handle_line`, before a `RpcRequest` is ever constructed, so it is
//! out of scope for the `dispatch`-level property here and is covered by the
//! framing/handle_line tests instead.)
//!
//! The "never panics" invariant is checked structurally: `dispatch` is a
//! total `Result`-returning function, so if any input caused it to panic,
//! the proptest harness would surface the panicking counterexample as a test
//! failure. Every branch below simply calls `dispatch` and asserts on the
//! `Result` it returns.

#![cfg(feature = "stdio")]

use proptest::prelude::*;
use robo_fitting_room::stdio::dispatch::dispatch;
use robo_fitting_room::stdio::protocol::RpcError;
use robo_fitting_room::transport_common::AppState;
use serde_json::Value;
use tempfile::TempDir;

/// Every method name the dispatch table recognizes. Kept in lockstep with
/// the `match` arms in `src/stdio/dispatch.rs`.
const KNOWN_METHODS: &[&str] = &[
    "health.check",
    "health.ready",
    "assets.listRobots",
    "assets.listFixtures",
    "assets.listFloorPlans",
    "assets.uploadFloorPlanObj",
    "urdf.joints",
    "floorPlan.segments",
    "compose.validate",
    "compose.run",
    "compose.urdfBundle",
    "scene.load",
    "scene.save",
];

/// The subset of known methods that deserialize their `params` via
/// `from_params` before doing any work. For these, params that cannot be
/// coerced into the expected struct shape must produce an `invalid_params`
/// (-32602) error. The remaining known methods (`health.*`, `assets.list*`)
/// ignore their params entirely and always succeed, so they are excluded
/// from the invalid-params assertion.
const PARAM_METHODS: &[&str] = &[
    "assets.uploadFloorPlanObj",
    "urdf.joints",
    "floorPlan.segments",
    "compose.validate",
    "compose.run",
    "compose.urdfBundle",
    "scene.load",
    "scene.save",
];

/// Build an `AppState` rooted at a fresh, empty temp directory. The dispatch
/// handlers that touch the filesystem resolve everything under this root, so
/// no test artifact ever escapes the tempdir (which is removed when the
/// returned `TempDir` guard drops).
fn test_state() -> (TempDir, AppState) {
    let dir = tempfile::tempdir().expect("create temp asset root");
    let state = AppState::new(dir.path().to_path_buf(), None);
    (dir, state)
}

/// Recursive strategy for arbitrary `serde_json::Value` params, spanning all
/// JSON shapes (null/bool/number/string/array/object), including nested
/// containers whose keys and string values may carry control characters.
fn arb_json() -> impl Strategy<Value = Value> {
    let leaf = prop_oneof![
        Just(Value::Null),
        any::<bool>().prop_map(Value::Bool),
        any::<i64>().prop_map(Value::from),
        any::<f64>()
            .prop_filter("json has no NaN/Infinity", |f| f.is_finite())
            .prop_map(|f| serde_json::json!(f)),
        ".*".prop_map(Value::String),
    ];
    leaf.prop_recursive(3, 24, 6, |inner| {
        prop_oneof![
            prop::collection::vec(inner.clone(), 0..6).prop_map(Value::Array),
            prop::collection::hash_map(".*", inner, 0..6)
                .prop_map(|m| Value::Object(m.into_iter().collect())),
        ]
    })
}

/// Strategy for JSON values that are *not* JSON objects and cannot be
/// coerced by serde_json into a `#[derive(Deserialize)]` struct. Arrays are
/// deliberately excluded because serde_json can deserialize a struct from a
/// JSON sequence (positional fields), which would let a well-shaped array
/// succeed for a single-field method — that ambiguity would make an
/// `invalid_params` assertion flaky. Null, bool, number, and string can
/// never become a struct, so they reliably trigger `invalid_params`.
fn arb_clearly_wrong_params() -> impl Strategy<Value = Value> {
    prop_oneof![
        Just(Value::Null),
        any::<bool>().prop_map(Value::Bool),
        any::<i64>().prop_map(Value::from),
        any::<f64>()
            .prop_filter("json has no NaN/Infinity", |f| f.is_finite())
            .prop_map(|f| serde_json::json!(f)),
        ".*".prop_map(Value::String),
    ]
}

/// Strategy for an arbitrary method string. `\PC*` ranges over arbitrary
/// (printable-ish, newline-free) unicode, which will almost never collide
/// with a known method name.
fn arb_method() -> impl Strategy<Value = String> {
    prop_oneof![
        // Mostly-random method names.
        3 => "\\PC*",
        // Occasionally emit exactly one of the known names, so the "known
        // method + arbitrary params" path gets exercised by this generator
        // too (in addition to the dedicated cases below).
        1 => proptest::sample::select(KNOWN_METHODS).prop_map(|s| s.to_string()),
    ]
}

/// `floorPlan.segments` takes the manifest's footprint layers (`footprintLayers` or
/// `footprint_layers`) and splits them off as compose does; a DXF file is refused.
#[test]
fn floor_plan_segments_take_the_footprint_layers() {
    let (dir, state) = test_state();
    let plans = dir.path().join("floor-plans");
    std::fs::create_dir_all(&plans).unwrap();
    std::fs::write(
        plans.join("p.obj"),
        "v 0 0 0\nv 1 0 0\nv 1 1 0\no Machine Outline\nl 1 2 3\ng walls\nl 3 1\n",
    )
    .unwrap();
    let counts = |params: Value| {
        let v = dispatch(&state, "floorPlan.segments", params).unwrap();
        (
            v["segments"].as_array().unwrap().len(),
            v["footprint_segments"].as_array().unwrap().len(),
        )
    };
    assert_eq!(
        counts(serde_json::json!({ "path": "floor-plans/p.obj" })),
        (3, 0)
    );
    assert_eq!(
        counts(
            serde_json::json!({ "path": "floor-plans/p.obj", "footprintLayers": ["machine outline"] })
        ),
        (1, 2)
    );
    assert_eq!(
        counts(serde_json::json!({ "path": "floor-plans/p.obj", "footprint_layers": ["WALLS"] })),
        (2, 1)
    );
    std::fs::write(plans.join("p.dxf"), "0\nSECTION\n").unwrap();
    let err = dispatch(
        &state,
        "floorPlan.segments",
        serde_json::json!({ "path": "floor-plans/p.dxf" }),
    )
    .unwrap_err();
    assert_eq!(err.data.unwrap().code, "floor_plan_unsupported");
}

proptest! {
    /// Totality: `dispatch` returns a `Result` (never panics) for *any*
    /// method string and *any* JSON params. When the generated method is not
    /// in the known set, the error must be `method_not_found` (-32601).
    #[test]
    fn dispatch_is_total_for_any_method_and_params(
        method in arb_method(),
        params in arb_json(),
    ) {
        let (_dir, state) = test_state();

        // Calling dispatch at all proves it did not panic — the harness would
        // otherwise report the panicking counterexample.
        let result = dispatch(&state, &method, params);

        if !KNOWN_METHODS.contains(&method.as_str()) {
            // Requirement 3.3: unknown method → method_not_found, id echoed by
            // the caller (handle_line), never a panic.
            match result {
                Err(RpcError { code, ref data, .. }) => {
                    prop_assert_eq!(
                        code,
                        RpcError::CODE_METHOD_NOT_FOUND,
                        "unknown method {:?} must map to -32601",
                        method
                    );
                    let data = data.as_ref().expect("method_not_found carries data");
                    prop_assert_eq!(&data.code, "method_not_found");
                }
                Ok(v) => prop_assert!(
                    false,
                    "unknown method {:?} unexpectedly returned Ok({:?})",
                    method,
                    v
                ),
            }
        }
        // For a (rare) generated-known method the result may be Ok or Err
        // depending on params; either way we've already proven totality by
        // getting a Result back.
    }

    /// Requirement 3.4: for every known, param-taking method, params that
    /// clearly cannot deserialize into the method's expected shape (a null,
    /// bool, number, or string where a struct is required) yield
    /// `invalid_params` (-32602) rather than panicking.
    #[test]
    fn known_methods_reject_malformed_params_with_invalid_params(
        idx in proptest::sample::select(PARAM_METHODS),
        params in arb_clearly_wrong_params(),
    ) {
        let (_dir, state) = test_state();
        let method = idx;

        let result = dispatch(&state, method, params.clone());

        match result {
            Err(RpcError { code, ref data, .. }) => {
                prop_assert_eq!(
                    code,
                    RpcError::CODE_INVALID_PARAMS,
                    "method {:?} with non-object params {:?} must map to -32602",
                    method,
                    params
                );
                let data = data.as_ref().expect("invalid_params carries data");
                prop_assert_eq!(&data.code, "invalid_params");
            }
            Ok(v) => prop_assert!(
                false,
                "method {:?} with malformed params {:?} unexpectedly returned Ok({:?})",
                method,
                params,
                v
            ),
        }
    }

    /// The no-param known methods (`health.*`, `assets.list*`) ignore their
    /// params and must always succeed, regardless of what JSON is supplied —
    /// a complementary corner of totality that confirms `dispatch` doesn't
    /// spuriously reject inputs it is supposed to ignore.
    #[test]
    fn no_param_methods_always_succeed(
        method in proptest::sample::select(
            &["health.check", "health.ready", "assets.listRobots",
              "assets.listFixtures", "assets.listFloorPlans"][..]
        ),
        params in arb_json(),
    ) {
        let (_dir, state) = test_state();
        let result = dispatch(&state, method, params);
        prop_assert!(
            result.is_ok(),
            "no-param method {:?} should ignore params and return Ok, got {:?}",
            method,
            result
        );
    }
}
