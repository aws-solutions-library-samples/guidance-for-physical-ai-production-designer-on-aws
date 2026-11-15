# trust-platform research report: embedding it as the ST check and simulation engine for `bt2plc`

**Bottom line:** you can embed it. One library API, `trust_runtime::harness::TestHarness`, already covers the whole simulation plan: load several source strings, set and read variables, advance virtual time explicitly, step one scan.

Three problems will hit your generated CODESYS code:
1. **`E_X.Y` enum access is very likely rejected** (strong evidence from the code, but not run).
2. **Methods and plain `VAR` members default to `PROTECTED`** (CODESYS uses `PUBLIC`), so `fb.M()` calls from outside fail unless you write `METHOD PUBLIC`.
3. **A plain git dependency on `trust-runtime` will not compile on macOS** unless you add one `[patch]` line (section 6).

Plan mode was on, so I cloned nothing and built nothing. Everything here comes from reading the source through the GitHub API, raw file fetches and streamed tarball greps. Two kinds of claim need a quick check before you rely on them: those marked "by code reading", and build times.

Repository facts:
- Default branch `main`. HEAD is `be8d81a4a7ab16ca7554b8be0f4723161ec1a47b` (2026-09-03), which is also tag **v0.24.69**, released 2026-09-04.
- 221 stars. Topics include `codesys`, `twincat`, `plcopen`.
- Rate-limited API calls were used for metadata; the rest came from `raw.githubusercontent.com` and `codeload` tarball streaming.

---

## 1. Workspace

### Root `Cargo.toml` (verbatim)

```toml
[workspace]
resolver = "2"
members = [
    "xtask",
    "crates/trust-debug",
    "crates/trust-syntax",
    "crates/trust-hir",
    "crates/trust-ide",
    "crates/trust-lsp",
    "crates/trust-dev",
    "crates/trust-plcopen",
    "crates/trust-ads-core",
    "crates/trust-ads-server",
    "crates/verification-cases",
    "crates/trust-runtime-core",
    "crates/trust-runtime",
    "crates/trust-tcads-native",
    "crates/trust-wasm-analysis",
]
exclude = [
    "third_party/tiverse-mmap",
]

[workspace.package]
version = "0.24.69"
edition = "2021"
license = "MIT OR Apache-2.0"
repository = "https://github.com/johannesPettersson80/trust-platform"
rust-version = "1.95"
authors = ["Johannes Pettersson <johannes_salomon@hotmail.com>"]
keywords = ["lsp", "plc", "iec61131", "structured-text", "automation"]
categories = ["development-tools", "text-editors"]

[workspace.dependencies]
# Syntax
logos = "0.16"
rowan = "0.16"
text-size = "1.1"

# Semantic analysis
salsa = "0.26"

# LSP
tower-lsp = "0.20"
lsp-types = "0.97"
tower = "0.4"

# Async runtime
tokio = { version = "1", features = ["full"] }

# Serialization
serde = { version = "1", features = ["derive"] }
serde_json = "1"
toml = "1.0"
toml_edit = { version = "0.25", features = ["serde"] }
sha2 = "0.10"
clap = { version = "4", features = ["derive"] }
clap_complete = "4"
qrcode = { version = "0.14", default-features = false, features = ["svg"] }
urlencoding = "2"
percent-encoding = "2"
roxmltree = "0.21"
wasm-bindgen = "0.2"

# Error handling
thiserror = "2"
anyhow = "1"

# Testing
expect-test = "1.5"
insta = { version = "1", features = ["glob"] }
futures-util = "0.3"

# Utilities
smol_str = "0.2.2"
rustc-hash = "2.1"
indexmap = "2"
drop_bomb = "0.1"
tracing = "0.1"
tracing-subscriber = { version = "0.3", features = ["env-filter"] }
once_cell = "1.21"
parking_lot = "0.12"
crossbeam-channel = "0.5"
glob = "0.3"
crc32fast = "1"
ratatui = "0.30"
crossterm = "0.29"
dialoguer = "0.12"
indicatif = "0.18"
owo-colors = "4"
rumqttc = { version = "0.25", default-features = false, features = ["use-native-tls"] }
native-tls = { version = "0.2.18", features = ["alpn", "vendored"] }
time = "0.3.47"
rapier3d = { version = "0.32.0", default-features = false, features = ["dim3", "f32", "enhanced-determinism"] }

# Internal crates
trust-syntax = { path = "crates/trust-syntax" }
trust-hir = { path = "crates/trust-hir" }
trust-ide = { path = "crates/trust-ide" }
trust-dev = { path = "crates/trust-dev" }
trust-plcopen = { path = "crates/trust-plcopen" }
trust-ads-core = { path = "crates/trust-ads-core" }
trust-ads-server = { path = "crates/trust-ads-server" }
verification-cases = { path = "crates/verification-cases" }
trust-runtime-core = { path = "crates/trust-runtime-core" }
trust-runtime = { path = "crates/trust-runtime" }
trust-tcads-native = { path = "crates/trust-tcads-native" }
trust-wasm-analysis = { path = "crates/trust-wasm-analysis" }

[workspace.lints.rust]
unsafe_code = "forbid"

[workspace.lints.clippy]
all = "warn"
pedantic = "warn"

[profile.release]
lto = true
codegen-units = 1

[profile.wasm-release]
inherits = "release"
opt-level = "z"
lto = false
codegen-units = 16

[profile.dev]
debug = 1
incremental = true

[profile.test]
debug = 1
incremental = true

[profile.dev.package.insta]
opt-level = 3

[profile.dev.package.similar]
opt-level = 3

[patch.crates-io]
tiverse-mmap = { path = "third_party/tiverse-mmap" }
```

`.cargo/config.toml` sets `rustc-wrapper = "sccache"`. That only matters if you build inside a checkout of the repo; it does not affect projects that depend on these crates.

### Crates

Every crate is version 0.24.69, edition 2021, `rust-version` 1.95. No `rust-toolchain` file. The README badge also says MSRV 1.95.

| Cargo package | Purpose (its own `description`) | Key dependencies / weight |
|---|---|---|
| `trust-syntax` | "Lexer, parser, and concrete syntax tree for IEC 61131-3 Structured Text" | logos, rowan, text-size, drop_bomb. Light. |
| `trust-hir` | "High-level IR and semantic analysis…" (type checking and diagnostics) | trust-syntax, salsa 0.26, smol_str, rustc-hash, tracing, parking_lot. Light: roughly 60–90 pure-Rust crates, no C. |
| `trust-ide` | IDE features (hover, completion, etc.) | trust-hir, once_cell. Light. |
| `trust-wasm-analysis` | "Browser/WASM analysis adapter for truST diagnostics, hover, and completion" | trust-hir/ide/syntax, serde_json. `crate-type = ["cdylib","rlib"]`; wasm-bindgen is optional. |
| `trust-plcopen` | "PLCopen XML import/export helpers" | trust-syntax, roxmltree, serde_json, anyhow, crc32fast. Light. |
| `trust-runtime-core` | "Portable execution core scaffold" (values, bytecode format, VM core types) | indexmap, smol_str, thiserror, optional trust-hir. **Contains no compiler.** |
| `trust-runtime` | "Runtime for IEC 61131-3 Structured Text": the ST→runtime compiler, the bytecode VM, the harness, and the CLIs | **Very heavy**, see below. |
| `trust-lsp` | LSP server (binary `trust-lsp`) | tower-lsp, tokio, and **trust-runtime** |
| `trust-dev` | "Developer and workbench CLI" (binary `trust-dev`: `test`, `docs`, `agent serve`, `commit`) | trust-runtime, clap |
| `trust-debug` | DAP debug adapter (binary `trust-debug`) | trust-runtime |
| `trust-ads-core`, `trust-ads-server`, `trust-tcads-native` | Beckhoff ADS types, server, Windows FFI | small |
| `verification-cases`, `xtask` | internal (`publish = false`) | |

`trust-bundle-gen` and `trust-harness` are **not separate crates**. They are binaries inside `trust-runtime`, at `crates/trust-runtime/src/bin/trust-bundle-gen.rs` and `.../trust-harness.rs`, next to `trust-runtime.rs`.

**Heavy dependencies of `trust-runtime`** (from `crates/trust-runtime/Cargo.toml`):
- **Always pulled in:**
  - networking and TLS: zenoh `=1.7.2` (which brings tokio), rumqttc and `native-tls` with `vendored`, tiny_http (`ssl-rustls`), tungstenite, mdns-sd, if-addrs, ureq 3, rustls 0.23
  - simulation and terminal UI: rapier3d 0.32, ratatui, crossterm, dialoguer, indicatif
  - system: notify, region, signal-hook
  - local and git crates: `tiverse-mmap = "1.0"`, plus four git crates from `open-ot-experiments` pinned at rev `137f0e7…` (public, Apache-2.0, small dependencies)
  - unix only: `open-ot-shm` (memmap2)
  - Linux only: `gpio-cdev`, `rustix`
- **Pulled in by default features** (`debug`, `ethercat-wire`, `ads-wire`, `ads-server`, `opcua-wire`, `openot-database-all`): opcua with `vendored-openssl` (compiles OpenSSL from C), rusqlite `bundled` (compiles SQLite from C), postgres, mysql, tiberius, ethercrab, ads.
- **Not used:** LLVM, cranelift.
- **C toolchain needed:** aws-lc-sys (through rustls) and, with default features, OpenSSL and SQLite. CI builds on macos-latest with no extra system packages, so Xcode Command Line Tools should be enough on macOS.
- **Lockfile size:** 739 packages for the whole workspace. Upper bounds on the locked closure (all features, all platforms, dev-deps included): `trust-runtime` 619, `trust-hir` 174.

---

## 2. Library APIs

### Parsing (`trust-syntax`)

```rust
// crates/trust-syntax/src/parser/mod.rs
pub use parser::parse;          // pub fn parse(text: &str) -> Parse
pub struct Parse { green_node: rowan::GreenNode, errors: Vec<ParseError> }
impl Parse {
    pub fn syntax(&self) -> SyntaxNode;
    pub fn errors(&self) -> &[ParseError];
    pub fn ok(&self) -> bool;
}
pub struct ParseError { pub message: String, pub range: text_size::TextRange }
```

### Semantic diagnostics (`trust-hir`)

```rust
// crates/trust-hir/src/project.rs
pub enum SourceKey { Path(PathBuf), Virtual(String) }
impl SourceKey { pub fn from_path(p: impl AsRef<Path>) -> Self; pub fn from_virtual(name: impl Into<String>) -> Self; }
pub struct Project { db: Database, sources: SourceRegistry }
impl Project {
    pub fn new() -> Self;
    pub fn set_source_text(&mut self, key: SourceKey, text: String) -> FileId;
    pub fn database(&self) -> &Database;
    pub fn remove_source(&mut self, key: &SourceKey) -> Option<FileId>;
}
// crates/trust-hir/src/db/queries.rs
pub struct FileId(pub u32);
pub trait SemanticDatabase: SourceDatabase {
    fn diagnostics(&self, file_id: FileId) -> Arc<Vec<Diagnostic>>;
    fn analyze(&self, file_id: FileId) -> Arc<FileAnalysis>;
    /* file_symbols, type_of, … */
}
// crates/trust-hir/src/diagnostics.rs
pub struct Diagnostic {
    pub code: DiagnosticCode,          // .code() -> "E101", "W005", …
    pub severity: DiagnosticSeverity,  // Error | Warning | Info | Hint
    pub range: TextRange,              // byte offsets
    pub message: String,
    pub related: Vec<RelatedInfo>,
}
impl Diagnostic { pub fn is_error(&self) -> bool; }
// Display: "error[E101]: undefined identifier 'x' (at 10..11)"
```

- **`diagnostics()` does not include parser errors.** The runtime build and the LSP each call `parser::parse(text).errors()` separately, and `bt2plc check` has to do the same.
- **Ready-made alternative with line/column:** `trust_wasm_analysis::BrowserAnalysisEngine` (in `src/lib/lib_part_02.rs`):
  - `new()`
  - `replace_documents(Vec<DocumentInput{uri,text}>)`
  - `diagnostics(uri) -> Vec<DiagnosticItem{code,severity,message,range{start{line,character}},span,related}>`
  - Also hir-only; it does not report parse errors either.

### Compile, bytecode, run (`trust-runtime`)

```rust
// crates/trust-runtime/src/host/harness/mod.rs (re-exports)
pub use api::{bytecode_bytes_from_source, bytecode_bytes_from_sources, bytecode_module_from_sources_with_paths, …, CompileSession};
pub use harness::TestHarness;
pub use protocol::{HarnessAutomation, encode_json_value, decode_json_value, …};
pub use types::{CompileError, CycleResult, SourceFile};

// api.rs
impl CompileSession {
    pub fn from_source(source: impl Into<String>) -> Self;
    pub fn from_sources(sources: Vec<SourceFile>) -> Self;     // multi-file; errors labelled with path
    pub fn build_runtime(&self) -> Result<Runtime, CompileError>;
    pub fn build_bytecode_module(&self) -> Result<BytecodeModule, CompileError>;
    pub fn build_bytecode_bytes(&self) -> Result<Vec<u8>, CompileError>;   // STBC
}
// types.rs
pub struct SourceFile { pub path: Option<String>, pub text: String }
impl SourceFile { pub fn new(text) -> Self; pub fn with_path(path, text) -> Self; }
pub struct CycleResult { pub cycle_number: u64, pub elapsed_time: Duration, pub errors: Vec<RuntimeError> }
```

The compile pipeline is in `host/harness/build.rs`:
1. Parse every file; if any parse errors exist, fail with them joined.
2. Reject textual ACTION/SFC ("textual ACTION/SFC execution is unsupported by the runtime compiler").
3. Build a `trust_hir::Project` and stop on any hir error diagnostic.
4. Lower to the runtime model.
5. Encode bytecode.

`CompileError` is a single message string; it does not carry structured line/column information.

### Harness API (`host/harness/harness.rs`)

```rust
pub struct TestHarness { runtime: Runtime, cycle_count: u64 }
impl TestHarness {
    pub fn from_source(source: &str) -> Result<Self, CompileError>;
    pub fn from_sources(sources: &[&str]) -> Result<Self, CompileError>;
    pub fn set_input(&mut self, name: &str, value: impl Into<Value>);            // panics on error
    pub fn try_set_input(&mut self, name: &str, value: impl Into<Value>) -> Result<(), BoundaryError>;
    pub fn get_output(&self, name: &str) -> Option<Value>;
    pub fn try_get_output(&self, name: &str) -> Result<Value, BoundaryError>;
    pub fn cycle(&mut self) -> CycleResult;                // = runtime.execute_cycle()
    pub fn run_cycles(&mut self, count: u32) -> Vec<CycleResult>;
    pub fn run_until_max<F: Fn(&Runtime)->bool>(&mut self, f: F, max: u64) -> Vec<CycleResult>;
    pub fn advance_time(&mut self, duration: Duration);    // virtual clock
    pub fn current_time(&self) -> Duration;
    pub fn restart(&mut self, mode: RestartMode) -> Result<(), RuntimeError>;
    pub fn bind_direct / set_direct_input / get_direct_output (… "%IX0.0" …);
    pub fn runtime(&self) -> &Runtime;  pub fn runtime_mut(&mut self) -> &mut Runtime;
}
```

`Runtime` (`runtime/core/accessors.rs`, `cycle.rs`, `core/scheduling.rs`):
- `execute_cycle()`
- `advance_time(delta)` and `set_current_time(t)`
- `storage()` and `storage_mut() -> VariableStorage`
- `programs()`, `function_blocks()`
- `io()` and `io_mut()`
- `execute_function_block_by_name(&str)`

`VariableStorage`:
- `get_global(&str) -> Option<&Value>` (exact-case lookup)
- `set_global(impl Into<SmolStr>, Value)`
- `globals()`
- `get_instance_var(id, &str)`
- `get_instance_var_recursive(id, &str)`

Value types are in `trust_runtime::value` (re-exported from runtime-core):
- `Value` variants include `Bool`, `Byte(u8)`, `Word(u16)`, `DWord(u32)`, `Int(i16)`, `DInt(i32)`, `Time(Duration)`, `String(SmolStr)`, `Array(Box<ArrayValue>)`, `Struct(Arc<StructValue>)`, `Enum(Box<EnumValue>)`, `Instance(InstanceId)`.
- `ArrayValue::from_untyped_parts(elements, dims)`, `.elements()`, `.dimensions()`; `StructValue::field(name)`; `EnumValue::variant_name()`, `.numeric_value()`.
- **Pitfall:** `impl From<u8> for Value` produces `Value::USInt`, not `Byte`. For BYTE data, construct `Value::Byte(x)` explicitly.

### Time is fully virtual

- With no `CONFIGURATION`, every `PROGRAM` runs once per `cycle()` as a "background program", in declaration order (`runtime/cycle.rs`, `execute_background_programs`).
- Time only moves when you call `advance_time`. In `tests/harness.rs` `run_controls`, five cycles leave `current_time()` at `Duration::ZERO`.
- If you do declare `TASK X (INTERVAL := T#10ms)`, the task only fires once enough virtual time has passed (`tests/e2e_configuration.rs`: first cycle at t=0 gives `count = 0`; after `advance_time(10ms)` it gives 1). Leave out `CONFIGURATION`, or always advance by your scan time before each cycle.

### How names resolve (`host/boundary/resolver.rs`)

| Kind of name | Read with `get_output` | Write with `set_input` |
|---|---|---|
| Simple name, e.g. `count` | Searched across globals, then program variables (an error if ambiguous) | Same search, works |
| `Prog.var`, `Prog.fb.out`, `Prog.arr[3]` | Works (exact match, then a parsed path) | Works: the root `Prog` is an `Instance` global |
| `NAMESPACE GVL` global, e.g. `"GVL.shared"` | Works for the whole variable: `get_global(path)` is tried first (`tests/vars_access.rs:292-320` asserts `storage().get_global("GVL.shared")`) | **By code reading, fails:** the root `GVL` resolves to nothing, giving `UnresolvedName`. Same for `"GVL.arr[3]"` |

**For namespaced GVLs**, read and write the whole value through `runtime().storage().get_global("GVL_IO.aOut")` and `runtime_mut().storage_mut().set_global(...)`.

### End-to-end example from the repo

`crates/trust-runtime/tests/iec_timers.rs`:

```rust
let mut harness = TestHarness::from_source(source).unwrap();   // PROGRAM Test with ton : TON; pt : TIME := T#10ms …
harness.set_input("in_ton", true);
harness.advance_time(Duration::from_millis(5));
harness.cycle();
harness.assert_eq("et_ton", Value::Time(Duration::from_millis(5)));
harness.advance_time(Duration::from_millis(5));
harness.cycle();
harness.assert_eq("q_ton", Value::Bool(true));
```

Multi-file with namespaced GVL, from `tests/vars_access.rs::namespaced_globals_support_qualified_access`:

```rust
let gvl  = "NAMESPACE GVL\nVAR_GLOBAL\n    shared : INT := 3;\nEND_VAR\nEND_NAMESPACE\n";
let main = "PROGRAM Main\nVAR observed : INT; END_VAR\nobserved := GVL.shared;\nGVL.shared := GVL.shared + 1;\nEND_PROGRAM\n";
let mut harness = TestHarness::from_sources(&[gvl, main]).unwrap();
harness.cycle();
assert_eq!(harness.get_output("observed"), Some(Value::Int(3)));
// harness.runtime().storage().get_global("GVL.shared") == Int(4)
```

### Suggested wiring of the two PLCs in a Rust test

```rust
use trust_runtime::harness::TestHarness;
use trust_runtime::value::{Duration, Value};

let mut head  = TestHarness::from_sources(&head_srcs).expect("head compiles");
let mut robot = TestHarness::from_sources(&robot_srcs).expect("robot compiles");
let dt = Duration::from_millis(4);
for _ in 0..n {
    let out = head.runtime().storage().get_global("GVL_HeadIO.aToRobot").cloned().unwrap();
    robot.runtime_mut().storage_mut().set_global("GVL_RobotIO.aFromHead", out);   // Value::Array of Value::Byte
    let back = robot.runtime().storage().get_global("GVL_RobotIO.aToHead").cloned().unwrap();
    head.runtime_mut().storage_mut().set_global("GVL_HeadIO.aFromRobot", back);
    head.advance_time(dt);  robot.advance_time(dt);
    assert!(head.cycle().errors.is_empty());
    assert!(robot.cycle().errors.is_empty());
}
```

- `set_global` does no type check. Keep the bounds and element kinds identical on both sides.
- Keep the two PLCs as **separate harnesses**: it avoids POU and type name clashes, and copying in Rust between cycles gives the same one-scan delay either way.
- A runtime error in a cycle marks the resource faulted; every later cycle then returns `ResourceFaulted`.

---

## 3. Dialect coverage

| Feature | Status | Evidence |
|---|---|---|
| `{attribute '…'}` pragmas, `{attribute 'qualified_only'}` | **Parsed and ignored.** The lexer turns `\{[^}]*\}` into a `Pragma` token that counts as whitespace-like trivia. OpenOT validation only looks at `'oot'` attributes. qualified_only is **not enforced**. | `trust-syntax/src/lexer/tokens/token_kind.rs:17-18`; `token_kind_impl_classification.rs:6-11`; docs/guides/PLCOPEN_INTEROP_COMPATIBILITY.md:580: "Core truST does not yet enforce CODESYS `{attribute 'qualified_only'}`… bare global access is still accepted" |
| GVL with qualified access `GVL.x` | **Supported only through a `NAMESPACE GVL … END_NAMESPACE` wrapper.** A bare top-level `VAR_GLOBAL` is file-scope: names are unqualified and share one flat scope. | `tests/vars_access.rs:292-360`; the PLCopen importer renders qualified_only lists exactly this way (`trust-plcopen/src/plcopen/import_data_globals.rs:259-282`) |
| Enum with `:= value`, e.g. `TYPE E_X : (A := 0, B := 1); END_TYPE` | Supported. An optional base type is accepted after the list (`(…) INT`) and in IEC form before it (`INT (…)`). | `parser/grammar/declarations.rs:144-187, 189+`; `collector/types.rs:499-555`; `tests/corpus/function_blocks/motor.st` |
| **Enum access `E_X.Y`** | **Very likely rejected.** `resolve_qualified` only walks `Namespace` symbols. The base `E_X` is then checked as a value, and `non_value_role(Type)` raises **E202 "type 'E_X' cannot be used as a value"**. CASE labels go through the same expression checker. No repo test uses dotted enum access. Supported forms: `E_X#Y` and unqualified `Y` (unqualified is an error if ambiguous). | `trust-hir/src/symbols/table.rs:372-392`; `type_check/calls/exprs.rs:128-154`; `type_check/mod.rs:34-52`; `tests/semantic_type_checking/wrong_kind_resolution.rs:92-111`; enum tests use `Phase#IDLE` |
| Bit access on BYTE/WORD/DWORD (`b.7`, `b.%X3`), array element `values[2].%B1` | Supported, read and write. `a[0].3` is not tested literally, but the parser accepts any postfix expression followed by `.`+IntLiteral, and both halves are tested. | `tests/types_bit_access.rs` (`bit_val := b.7;`); `hir/tests/.../partial_access_acceptance.rs` (`value.3`, `values[2].%B1 :=`); `parser/grammar/expressions.rs:81-91` |
| TON / TOF / TP, R_TRIG / F_TRIG, RS / SR, CTU / CTD / CTUD | Supported | `stdlib/fbs/registry.rs`; `tests/iec_timers.rs`; `tests/fb_edges.rs` (F_TRIG fires on the first scan when CLK=FALSE, like CODESYS) |
| `T#500MS` | Supported; time literals are case-insensitive | `token_kind.rs:~400-410` `ignore(case)`; `tests/lexer_literals.rs` |
| ARRAY OF STRUCT initializers | Supported: `[3((x := 7))]`, explicit lists, type-level defaults | `tests/struct_initializers.rs:136-242` |
| `VAR_GLOBAL CONSTANT` | Supported, including across files and as `STRING(N)` bounds. The combination with an array-of-struct initializer is not tested literally. | `tests/compile_time_constants.rs` |
| `STRING(32)`, `STRING[32]` | Supported | snapshot `parser_types__string_type_with_parenthesized_length`; `compile_time_constants.rs` |
| `BYTE_TO_WORD`, `SHL` / `SHR` / `ROL` | Supported | `tests/stdlib_conv_full.rs:46`; `tests/stdlib_bit_full.rs:9-21` |
| `VAR_IN_OUT`, `VAR_TEMP`, `VAR_STAT`, `CASE`, `FOR`, `WHILE`, `REPEAT`, `EXIT`, `CONTINUE`, `JMP`, `RETURN` | Supported | lexer keyword list; parser snapshots `for_loop`, `exit_continue`, `case_statement` |
| CASE with enum labels | Supported for unqualified `A:` and `E#A:` | `bytecode_vm_differential.rs::register_and_stack_paths_match_for_unqualified_enum_case_labels` |
| `METHOD` (in FB and CLASS), `PROPERTY`, `INTERFACE`, `EXTENDS`, `THIS`/`SUPER` | Supported. **Default access is `PROTECTED`** (IEC Ed.3), and this applies to plain `VAR` members too. Calling `fb.M()` from outside needs `METHOD PUBLIC`. | `tests/pou_methods.rs`; `collector/collect.rs:386-398`; docs/IEC_DECISIONS.md "2026-07-30 Function-block ordinary-member default access" |
| Missing `VAR_EXTERNAL` | Allowed, as a documented deviation from IEC | docs/IEC_DEVIATIONS.md "Optional VAR_EXTERNAL…" |

**Not supported** (none of these is a keyword or used anywhere in the repo):
- textual `ACTION` execution (rejected by the runtime compiler)
- `VAR_INST`
- `REFERENCE TO` (trust has `REF_TO` and `POINTER TO`)
- `AND_THEN` / `OR_ELSE`
- the `S=` / `R=` assignment operators
- enforcement of `qualified_only`
- CODESYS's object model (separate declaration and implementation texts, GVL name taken from the object name)
- identifiers are ASCII-only

The `vendor_profile = "codesys"` setting in `trust-lsp.toml` only changes formatting and diagnostic defaults, not what the parser or type checker accepts.

---

## 4. Source and project format

**Each `.st` (or `.pou`) file must contain complete IEC text:**
- `FUNCTION_BLOCK … END_FUNCTION_BLOCK`, `PROGRAM … END_PROGRAM`, `TYPE … END_TYPE`
- top-level `VAR_GLOBAL … END_VAR` (file-scope)
- `NAMESPACE N … END_NAMESPACE` (namespaced)
- optionally `CONFIGURATION / RESOURCE / TASK / PROGRAM … WITH …`

Inside an FB, `METHOD … END_METHOD` blocks and body statements may appear in any order after the variable declarations (`pou_part_01.rs:146-186`). So **decl + impl + methods + `END_FUNCTION_BLOCK`** is valid.

**What `bt2plc` needs to do to produce trust-compatible text:**
1. FB / PROGRAM: concatenate the declaration text, the implementation text, each method (its declaration + implementation + `END_METHOD`), then `END_FUNCTION_BLOCK` / `END_PROGRAM`.
2. GVL: wrap in `NAMESPACE <GvlName> … END_NAMESPACE`. The pragma can stay.
3. Enums: write `E_X#Y` instead of `E_X.Y`. Whether CODESYS itself accepts `#` needs checking, so a trust-specific output mode is the safe choice.
4. Methods: write `METHOD PUBLIC`.

**Project layout:** there is no `trust.toml`. A project is a directory:
- `src/**/*.st|.pou`, found recursively; the default sources root is `<project>/src`
- `trust-lsp.toml`: `[project] vendor_profile`, `include_paths`, `[dependencies]`, `[[libraries]]`
- `runtime.toml`, `io.toml`, optional `simulation.toml`, `hmi/`
- `program.stbc` (build output)

Tasks are declared in ST `CONFIGURATION` blocks. The documented layout is in docs/public/develop/project-layout.md and concepts/project-model.md.

**PLCopen XML import is usable from Rust, but works on files:**
- `trust_plcopen::import_xml_to_project(xml_path: &Path, project_root: &Path) -> anyhow::Result<PlcopenImportReport>`
- `import_xml_to_project_with_options(…, PlcopenImportOptions { global_var_mode: NativeVendorParity | StrictIecAdapter })`

It writes ST into `project/src`, prefers CODESYS `interfaceasplaintext`, maps qualified_only GVLs to `NAMESPACE`, and rebuilds CODESYS methods. The crate is light (it depends on trust-syntax only), and is also re-exported as `trust_runtime::plcopen`.

---

## 5. Command line

**`trust-runtime build [--project P] [--sources DIR] [--ci]`**
- Needs only `src/`.
- Writes `program.stbc`.
- `--ci` prints JSON `{version, command:"build", status, program, sources…}`.
- Compile failure exits with code 11.

**`trust-runtime check [--project] [--sources] [--json] [--ci]`**
- **Requires `runtime.toml` and `io.toml`.**
- Compile errors are collapsed into one issue: `{"severity":"error","code":"compile","message":"program compile failed: …"}`, with no line or column.
- Config errors exit 10, compile errors exit 11 (`bin/trust-runtime/check.rs`).

**`trust-runtime validate --project P [--ci]`** validates the config and the bundle.

**`trust-dev test [--project] [--filter] [--list] [--timeout 5] [--output human|junit|tap|json] [--ci]`** is the headless ST unit-test runner. `trust-runtime test` is a deprecated alias for it.
- Test POUs: `TEST_PROGRAM … END_TEST_PROGRAM`, `TEST_FUNCTION_BLOCK … END_TEST_FUNCTION_BLOCK`.
- Assertions: `ASSERT_TRUE/FALSE/EQUAL/NOT_EQUAL/GREATER/LESS/GREATER_OR_EQUAL/LESS_OR_EQUAL/NEAR` (`stdlib/assertions.rs`).
- Each test runs once after a cold restart (`trust-dev/src/test_cmd/execute.rs`), so **virtual time does not advance inside a test**. TON-based behaviour cannot be tested this way; use the Rust harness.

**`trust-harness`** (a `trust-runtime` binary, **not included in the release tarballs**)
- Line-delimited JSON over stdin/stdout. Protocol v2 is the default.
- Commands: `load {source|sources}` (runs one initial cycle), `cycle {count, dt_ms, watch}` (dt is advanced *before* each cycle), `set_input`, `get_output`, `advance_time {duration_ms}`, `run_until`, `snapshot`, `restart`, `bind_direct`, …
- Values are typed JSON, e.g. `{"type":"BYTE","value":5}`, `{"type":"ARRAY","dimensions":[[0,31]],"elements":[…]}` (docs/guides/TRUST_HARNESS_PROTOCOL.md).

**`trust-dev agent serve --project P`** (JSON-RPC 2.0, one message per line on stdio; `trust-dev` *is* in the release)
- `lsp.diagnostics {project, path?, content?}` returns `issues[]` with `path`, `file`, **1-based `line`/`column`**, `severity`, `code`, `message`, plus error and warning counts. Parse errors are included because it uses the LSP path.
- `harness.load {inline_sources:[{path,text}]}`, `harness.cycle`, `harness.set_input`, `harness.get_output`, `harness.advance_time`, `harness.run_until`, `harness.execute`.
- It holds one harness session per process.

---

## 6. Can you embed it?

**Not on crates.io:** `trust-runtime`, `trust-hir`, `trust-runtime-core` and `trust-ide` do not exist there. The crates.io names `trust-syntax` and `trust-lsp` belong to an **unrelated** project (briannadoubt/trust). Always use git dependencies.

**License:** MIT OR Apache-2.0 (both license files present; GitHub shows Apache-2.0). The `deny.toml` allow-list for dependencies includes EPL-2.0 (zenoh is EPL-2.0 OR Apache-2.0) and MPL-2.0. Compatible with your project.

**MSRV:** 1.95. This machine has rustc 1.95.0 (the brief said 1.96); both are fine.

**macOS aarch64:** CI builds macos-latest, and darwin-arm64 binaries are released. No Homebrew or system libraries are needed; a C compiler is (aws-lc-sys, plus OpenSSL and SQLite with default features). On Linux, the non-optional `native-tls` with `vendored` compiles OpenSSL from source, which needs perl and make.

**Blocker you will hit:** the root `[patch.crates-io] tiverse-mmap = { path = "third_party/tiverse-mmap" }` is ignored when another project depends on these crates. I diffed the published **tiverse-mmap 1.0.0** (its only version) against the copy in the repo:
- `src/platform/unix.rs:299`, inside `#[cfg(not(target_os = "linux"))] fn new_huge`, calls `Self::new(file, offset, len, protection)`, but `new` takes five arguments (`…, mode: MappingMode`).
- So the published crate **does not compile on macOS** (and the Windows code changed too).
- The fix is to repeat the patch in your own workspace root (cargo can find that package inside the git repo).

**Other risks:**
- Cargo resolves dependency versions fresh for your project; the upstream `Cargo.lock` is not used.
- `default-features = false` is not exercised in CI. The feature `cfg` gating looks consistent, but it could still fail to build; if so, fall back to the defaults.
- I have not measured build time. Expect several minutes for a clean debug build and a multi-GB `target/` for `trust-runtime`; `trust-hir` alone builds quickly.

**Recommended `Cargo.toml` for bt2plc** (pin by commit; tag `v0.24.69` is the same commit):

```toml
[dependencies]
# light: parse errors + semantic diagnostics for `bt2plc check`
trust-syntax = { git = "https://github.com/johannesPettersson80/trust-platform", rev = "be8d81a4a7ab16ca7554b8be0f4723161ec1a47b" }
trust-hir    = { git = "https://github.com/johannesPettersson80/trust-platform", rev = "be8d81a4a7ab16ca7554b8be0f4723161ec1a47b" }
# optional: full compile gate inside the CLI (heavy) -> put behind a feature
trust-runtime = { git = "https://github.com/johannesPettersson80/trust-platform", rev = "be8d81a4a7ab16ca7554b8be0f4723161ec1a47b", default-features = false, optional = true }

[dev-dependencies]
# simulation tests (TestHarness); same rev so one checkout is shared
trust-runtime = { git = "https://github.com/johannesPettersson80/trust-platform", rev = "be8d81a4a7ab16ca7554b8be0f4723161ec1a47b", default-features = false }

[features]
trust-compile = ["dep:trust-runtime"]

[patch.crates-io]   # REQUIRED whenever trust-runtime is in the graph (crates.io 1.0.0 fails on macOS)
tiverse-mmap = { git = "https://github.com/johannesPettersson80/trust-platform", rev = "be8d81a4a7ab16ca7554b8be0f4723161ec1a47b" }
```

Shape of `bt2plc check`:
1. For each generated file, collect `trust_syntax::parser::parse(&text).errors()`.
2. Load all files into one `trust_hir::Project` (`project.set_source_text(SourceKey::from_virtual(name), text)`).
3. Collect `project.database().diagnostics(id)` for each file.
4. Convert `TextRange` byte offsets to line and column yourself.
5. With the `trust-compile` feature on, also run `CompileSession::from_sources(vec![SourceFile::with_path(..)]).build_bytecode_bytes()` to catch lowering-only errors such as ACTION.

**If embedding turns out impractical, use the released binaries** (`https://github.com/johannesPettersson80/trust-platform/releases/download/v0.24.69/trust-runtime-darwin-arm64.tar.gz`, which contains `trust-runtime` (50 MB), `trust-dev`, `trust-bundle-gen`, `INSTALL.md` and `SHA256SUMS`):
- Check: spawn `trust-dev agent serve --project <tmpdir>` and call `lsp.diagnostics`. That gives structured line/column results, which `trust-runtime check` does not. Alternatively, `trust-runtime build --project <tmpdir> --ci` and test for exit code 11.
- Simulation: one `trust-dev agent serve` process per PLC, using `harness.load` / `harness.cycle {dt_ms}` / `harness.get_output` / `harness.set_input` with typed JSON arrays. Keep the I/O exchange variables as program variables or file-scope globals, because path writes to namespaced GVLs appear to fail (section 2).
- `trust-harness` itself: `cargo install --git https://github.com/johannesPettersson80/trust-platform --rev be8d81a4… --locked trust-runtime --bin trust-harness`. I believe cargo applies the repo's own `[patch]` for git installs, but haven't verified it. If it doesn't, clone and `RUSTC_WRAPPER= cargo build --release --locked -p trust-runtime --bin trust-harness`.

**Before committing, run this one test** to settle the two open dialect questions. Expected per the code reading: `E_X.B` and `a[0].3` are the cases to watch, and `E_X#B` should pass.

```rust
TestHarness::from_sources(&[
  "TYPE E_X : (A := 0, B := 1); END_TYPE",
  "{attribute 'qualified_only'}\nNAMESPACE GVL_IO\nVAR_GLOBAL\n a : ARRAY[0..31] OF BYTE;\nEND_VAR\nEND_NAMESPACE",
  "PROGRAM P\nVAR e : E_X; t : TON; q : BOOL; END_VAR\nt(IN := GVL_IO.a[0].3, PT := T#500MS);\nq := t.Q;\nCASE e OF E_X.A: e := E_X.B; END_CASE\nEND_PROGRAM",
]).map(|_| ()).unwrap_err(); // then retry with E_X#A / E_X#B
```

---

# Verification of the report

Verified against trust-platform rev `be8d81a4a7ab16ca7554b8be0f4723161ec1a47b` (tag v0.24.69, still `main` HEAD on 2026-10-07). I read the source through the GitHub API, raw files, streamed tarballs and crates.io. Plan mode was on, so I cloned nothing and built or ran nothing: every verdict below comes from reading code. The report holds up except on one point it got wrong: **an array-of-struct initializer in `VAR_GLOBAL CONSTANT` makes the runtime compile fail**.

## (a) Crate names and Rust API

- **CONFIRMED** Library crates are `trust-syntax`, `trust-hir`, `trust-runtime` and `trust-runtime-core`. None of them sets `[lib] name`, so the Rust names are `trust_syntax`, `trust_hir` and `trust_runtime`.
- **REFUTED (the brief, not the report)** `trust-bundle-gen` is not a crate. It is a binary at `crates/trust-runtime/src/bin/trust-bundle-gen.rs`, next to `trust-harness.rs` and `trust-runtime.rs`.
- **CONFIRMED** None of `trust-runtime`, `trust-hir`, `trust-runtime-core` or `trust-ide` is on crates.io. `trust-syntax` and `trust-lsp` 0.2.2 there belong to github.com/briannadoubt/trust.
- **CONFIRMED** Parser API, in `trust-syntax/src/parser/parser.rs:13` and `parser/mod.rs`:
  - `pub fn parse(source: &str) -> Parse`, with `errors()` and `ok()`.
  - `ParseError` displays as `"{message} at {start}..{end}"`.
- **CONFIRMED** `trust_hir::{Project, SourceKey}` (`lib.rs:46`) and `trust_hir::db::{FileId, SemanticDatabase}` (`db/mod.rs:28`).
  - Calls: `Project::set_source_text(SourceKey, String) -> FileId`, then `database().diagnostics(FileId) -> Arc<Vec<Diagnostic>>`.
  - `Diagnostic` has `code`, `severity`, `range`, `message`, `related` and `is_error()`, and implements Display.
- **CONFIRMED** `diagnostics()` leaves out parse errors. `salsa_backend.rs:217` `parse_green` keeps only `parsed.syntax().green()`. `host/harness/build.rs` checks `parser::parse(..).errors()` separately before running the hir pass.
- **CONFIRMED** `harness/mod.rs` exports `CompileSession`, `TestHarness`, `SourceFile`, `CompileError`, `CycleResult` and `BoundaryError`.
  - `CompileError` is only `{ message: String }` (`types.rs`), with no line or column.
  - `from_sources` turns `label_errors` on when there is more than one file (`api.rs:33`).
- **CONFIRMED** `harness.rs` provides `from_sources(&[&str])`, `try_set_input`/`set_input`, `try_get_output`/`get_output`, `cycle()`, `run_cycles`, `run_until_max`, `advance_time`, `current_time`, `runtime()`/`runtime_mut()`, `bind_direct`, `set_direct_input`/`get_direct_output` and `restart`.
  - `cycle()` wraps `runtime.execute_cycle()`.
  - Building a harness does not run a cycle.
  - `TestHarness` cannot take file paths; for labelled errors use `CompileSession` with `SourceFile::with_path`.
- **CONFIRMED** Time is virtual.
  - `runtime/core/scheduling.rs:88` `advance_time` is the only thing that moves `current_time`; `execute_cycle` never touches it.
  - `iec_timers.rs` steps TON/TOF/TP by explicit `advance_time` calls.
  - `TIME()` returns this logical time, but `CURRENT_DT()` reads the host wall clock (`runtime_clock_source_contract_tests.rs`), so generated sim code must not use it.
- **CONFIRMED** Without a `CONFIGURATION`, each `PROGRAM` becomes an instance named after its type (`build.rs`: `name: program.name.clone(), type_name: program.name.clone()`).
  - Every PROGRAM not bound to a task runs once per cycle, in source order (`cycle.rs` `execute_background_programs`). With no PROGRAM at all the build fails with "missing PROGRAM declaration".
- **CONFIRMED** A runtime error faults the resource: `execute_cycle` starts with `if self.faults.is_faulted() { return Err(error::RuntimeError::ResourceFaulted) }`. `Runtime::clear_fault()` exists.
- **CONFIRMED** Storage and value types:
  - `VariableStorage::get_global(&str)` is an exact, case-sensitive `IndexMap::get`; `set_global` is an unchecked `insert` (`memory/storage.rs:9-21`).
  - Namespaced globals are stored as `"NS.var"` (`harness/util.rs:61` `namespace_qualified_name`; `tests/vars_access.rs:292`).
  - `impl From<u8> for Value` gives `Value::USInt` (`trust-runtime-core/src/value/types.rs:962`).
  - Simple-name writes are not coerced: test `boundary_resolver_contract_simple_write_preserves_runtime_value_identity` stores a `WString` into an INT. Always build the exact variant, e.g. `Value::Byte`.
- **CONFIRMED, with a correction** `trust_runtime::value::Duration` is a custom type (`struct Duration { nanos: i64 }`, `from_millis(i64)`), not `std::time::Duration`. `ArrayValue::from_untyped_parts` returns a `Result`. `elements_mut()` exists.
- **CONFIRMED by code** Path access to a namespaced GVL fails for writes and for element reads.
  - In `host/boundary/resolver.rs`, a non-simple path is parsed into `Field{Name("GVL_IO"),…}`. `classify_root("GVL_IO")` then finds no global and no program variable and returns `Err(BoundaryError::UnresolvedName…)`.
  - So `try_set_input("GVL_IO.aVassInputData[0]")` and `get_output("GVL_IO.aVassOutputData[0]")` both fail.
  - Reading the whole variable works: `resolve_read` tries `get_global(path)` first.
  - No repo test covers boundary paths into namespaced GVLs.

## (b) Dialect

- **CONFIRMED, stronger than the report** `E_X.Y` is rejected.
  - `symbols/table.rs` `resolve_qualified` walks only namespaces: `if !matches!(symbol.kind, SymbolKind::Namespace) { return None; }`.
  - The base `E_X` is then checked as a value and gives `E202` "type 'E_X' cannot be used as a value" (`type_check/expr.rs` together with `non_value_role`).
  - CASE labels are rejected outright: `stmt_impl_part_03.rs` `is_case_label_expr` accepts only Literal, Paren, negative Unary and NameRef (`_ => false`), and reports "case label must be a literal, enum value, or constant". The same rule rejects namespaced constants such as `GVL.cX:` as labels.
  - Every test uses `E#V` or the bare `V`. `tests/corpus/function_blocks/motor.st` does use `E_MotorState.Stopped`, but no test reads that directory (only `docs/specs/14-lsp.md` mentions it) and the file references undeclared `FB_BaseDevice`.
  - Whether CODESYS accepts `E_X#Y` is **UNVERIFIABLE** here.
- **CONFIRMED** Enums with explicit values and an optional base type parse (`parser/grammar/declarations.rs:143-187`).
- **CONFIRMED** Bit access works for reads and writes.
  - The parser takes `.`+IntLiteral after any postfix expression (`expressions.rs` `parse_postfix_expr`).
  - Tests: `types_bit_access.rs` (`bit_val := b.7;`) and `partial_access_acceptance.rs` (`values[2].%B1 := …`, `value.3`).
  - The exact form `a[0].3` is not tested.
- **REFUTED (the report's assumption)** An array-of-struct initializer in `VAR_GLOBAL CONSTANT` fails the runtime compile with "expression is not a compile-time constant".
  - `compiler/config/globals_access.rs`: `if qualifiers.constant && matches!(kind, VarBlockKind::Global) { … ctx.eval_compile_time_const_initializer(expr, type_id)?`.
  - `helper_eval/const_expr.rs:61-88` handles Literal, ArrayInitializer, Unary, Binary and SizeOf, then `_ => Err(ConstExprError::UnsupportedExpr)`.
  - Test `runtime_only_expression_forms_are_rejected_as_constants` lists `Expr::StructInitializer`.
  - `dependency_prelude.rs` `resolve_project_global_constants(…, allow_partial=false)` returns the last error.
  - The same applies to `VAR CONSTANT` inside POUs (`resolve_pou_local_constants`).
  - Without `CONSTANT`, struct and array-of-struct initializers do work (`struct_initializers.rs`: `array_of_structs_and_repetition_materialize_defaults`, `var_global_and_direct_address_aggregate_initializers_materialize`).
  - Whether the hir-only check accepts the CONSTANT form is **UNVERIFIABLE**.
- **CONFIRMED** `{attribute …}` pragmas are parsed and ignored.
  - `token_kind.rs:17` `#[regex(r"\{[^}]*\}")] Pragma`, treated as trivia.
  - OpenOT validation only reacts to `oot` attributes.
  - `docs/guides/PLCOPEN_INTEROP_COMPATIBILITY.md:580`: "Core truST does not yet enforce CODESYS `{attribute 'qualified_only'}`".
  - `GVL.x` resolves only inside `NAMESPACE GVL … END_NAMESPACE`, which is how the PLCopen importer renders it (`import_data_globals.rs:259-282`).
- **CONFIRMED** Library and literals:
  - TON/TOF/TP, R_TRIG/F_TRIG, RS/SR and CTU/CTD/CTUD are registered (`stdlib/fbs/registry.rs`). F_TRIG fires on the first scan (`fb_edges.rs`).
  - Time literals are case-insensitive: `T#500MS` matches (`token_kind.rs:406`).
  - `STRING(n)` is supported (parser snapshot `parser_types__string_type_with_parenthesized_length`).
  - `BYTE_TO_WORD`, `SHL` and `SHR` are tested only by calling the stdlib directly.
  - A CASE without ELSE is only a warning.
- **CONFIRMED** In function blocks, plain `VAR` members, methods and properties default to `PROTECTED`; `VAR_INPUT`/`VAR_OUTPUT` stay PUBLIC (`collector/collect.rs:386-398`; `docs/IEC_DECISIONS.md:175-192`). Emit `METHOD PUBLIC` and route cross-FB reads through `VAR_OUTPUT`.

## (c) Source text format

- **CONFIRMED** Each source must be complete IEC text.
- **CONFIRMED** In an FB, all `VAR` blocks must come first. METHOD/PROPERTY/ACTION blocks and statements can then appear in any order (`pou_part_01.rs` `while self.current().is_var_keyword()` followed by the loop).
  - So build each FB as: declaration text + implementation text + each method (declaration + implementation + `END_METHOD`) + `END_FUNCTION_BLOCK`.
- **CONFIRMED** Textual ACTION is rejected by the runtime compiler (`build.rs`).
- For trust output, bt2plc needs these changes:
  - Wrap each GVL in a `NAMESPACE` named after the GVL object.
  - Write `E_X#Y` instead of `E_X.Y`.
  - Drop `CONSTANT` on aggregate-initialized globals. An untested alternative keeps `CONSTANT`: declare `TYPE T : ARRAY[..] OF ST := [...]; END_TYPE` and then `VAR_GLOBAL CONSTANT c : T; END_VAR`. Constants without an initializer are skipped by the constant resolver, and type-level defaults are tested.

## (d) Git dependency on macOS aarch64

- **CONFIRMED** `rust-version = "1.95"` and there is no toolchain file. This machine has rustc 1.95.0; 1.96 is not installed but also meets the MSRV.
- **CONFIRMED** CI builds `macos-latest` (`ci.yml:101`) and checks MSRV with `dtolnay/rust-toolchain@1.95` (`ci.yml:416`).
- **CONFIRMED** `tiverse-mmap` blocks the build unless you patch it.
  - `crates/trust-runtime/Cargo.toml` asks for `tiverse-mmap = "1.0"`, and crates.io has only 1.0.0.
  - In that crate, `src/platform/unix.rs:29` declares `new(file, offset, len, protection, mode: MappingMode)`, but line 299, in `#[cfg(not(target_os = "linux"))]`, calls `Self::new(file, offset, len, protection)`. The vendored copy calls `…, MappingMode::Shared`.
  - The repo's root `[patch.crates-io]` does not apply to dependents, so repeat it in bt2plc's root.
- **UNVERIFIABLE** whether `default-features = false` builds. CI never builds without default features. The gating looks deliberate (`ads/mod.rs` `#[cfg(feature = "ads-wire")] pub mod backend_ads_rs;`, `ads/server/mod.rs`, per-database cfgs in `openot_persistence/mod.rs`). The only build script is a Windows-only linker flag. `open-ot-experiments` at `137f0e7` is public and Apache-2.0.
- **CONFIRMED** The release tarball contains `trust-runtime`, `trust-dev` and `trust-bundle-gen`, but not `trust-harness` (`release.yml:196-198`; darwin-arm64 tarball is 26 MB). `trust-dev agent serve` offers `lsp.diagnostics` and `harness.load`/`cycle`/`set_input`/`get_output`/`advance_time` (`trust-dev/src/agent.rs`).

## Minimal recipe

```toml
[dependencies]   # bt2plc check (light)
trust-syntax = { git = "https://github.com/johannesPettersson80/trust-platform", rev = "be8d81a4a7ab16ca7554b8be0f4723161ec1a47b" }
trust-hir    = { git = "https://github.com/johannesPettersson80/trust-platform", rev = "be8d81a4a7ab16ca7554b8be0f4723161ec1a47b" }
[dev-dependencies]   # simulation; if default-features=false fails to build, drop it
trust-runtime = { git = "https://github.com/johannesPettersson80/trust-platform", rev = "be8d81a4a7ab16ca7554b8be0f4723161ec1a47b", default-features = false }
[patch.crates-io]    # must be in bt2plc's workspace root
tiverse-mmap = { git = "https://github.com/johannesPettersson80/trust-platform", rev = "be8d81a4a7ab16ca7554b8be0f4723161ec1a47b" }
```

```rust
use trust_hir::{db::SemanticDatabase, Project, SourceKey};
use trust_runtime::harness::{CompileSession, SourceFile, TestHarness};
use trust_runtime::value::{Duration, Value};

fn st_check(files: &[(&str, &str)]) -> Vec<String> {
    let (mut out, mut project, mut ids) = (Vec::new(), Project::new(), Vec::new());
    for (name, text) in files {
        out.extend(trust_syntax::parser::parse(text).errors().iter().map(|e| format!("{name}: parse: {e}")));
        ids.push((*name, project.set_source_text(SourceKey::from_virtual(*name), text.to_string())));
    }
    for (name, id) in ids {
        out.extend(project.database().diagnostics(id).iter().filter(|d| d.is_error()).map(|d| format!("{name}: {d}")));
    }
    if out.is_empty() { // runtime-only rejections (e.g. CONSTANT struct initializers, ACTION)
        let srcs = files.iter().map(|(p, t)| SourceFile::with_path(*p, *t)).collect();
        if let Err(e) = CompileSession::from_sources(srcs).build_bytecode_bytes() { out.push(format!("compile: {e}")); }
    }
    out
}
fn set_elem(h: &mut TestHarness, g: &str, i: usize, v: Value) { // i = index - lower bound
    let st = h.runtime_mut().storage_mut();
    let Some(Value::Array(mut a)) = st.get_global(g).cloned() else { panic!("{g} missing") };
    a.elements_mut()[i] = v;
    st.set_global(g, Value::Array(a));
}
fn get_elem(h: &TestHarness, g: &str, i: usize) -> Value {
    match h.runtime().storage().get_global(g) { Some(Value::Array(a)) => a.elements()[i].clone(), o => panic!("{g}: {o:?}") }
}
#[test]
fn head_one_scan() {
    let gvl = "{attribute 'qualified_only'}\nNAMESPACE GVL_IO\nVAR_GLOBAL\n aVassInputData : ARRAY[0..31] OF BYTE;\n aVassOutputData : ARRAY[0..31] OF BYTE;\nEND_VAR\nEND_NAMESPACE\n";
    let prg = "PROGRAM PLC_PRG\nIF GVL_IO.aVassInputData[0].3 THEN GVL_IO.aVassOutputData[0] := BYTE#1; END_IF\nEND_PROGRAM\n";
    let files = [("GVL_IO.st", gvl), ("PLC_PRG.st", prg)];
    let d = st_check(&files); assert!(d.is_empty(), "{}", d.join("\n"));
    let texts: Vec<&str> = files.iter().map(|(_, t)| *t).collect();
    let mut h = TestHarness::from_sources(&texts).unwrap_or_else(|e| panic!("{e}")); // PLC_PRG auto-instantiated
    set_elem(&mut h, "GVL_IO.aVassInputData", 0, Value::Byte(0b1000));
    h.advance_time(Duration::from_millis(4));
    let r = h.cycle(); assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert_eq!(get_elem(&h, "GVL_IO.aVassOutputData", 0), Value::Byte(1));
}
```

For the two-PLC simulation, keep one harness per PLC and move whole arrays between them with `get_global(..).cloned()` and `set_global(..)` between cycles. Bounds and element variants must match on both sides.

The path API (`try_set_input("GVL_IO.x[0]")`, and the same through the `trust-dev agent`) cannot write namespaced GVLs. If you need it, generate a small bridge PROGRAM in trust mode, placed first so it runs first: `PROGRAM SIM_IN VAR a : ARRAY[0..31] OF BYTE; END_VAR GVL_IO.aVassInputData := a; END_PROGRAM`. Then write `SIM_IN.a[0]`, which resolves through the program root. Read outputs whole, by their exact name. Whole-array assignment in this bridge is untested.

Run `head_one_scan` first to settle three things I could not check here:
1. Whether `default-features = false` builds.
2. Whether namespaced array elements can be read and written from ST: the repo only tests scalar `GVL.shared`.
3. Whether `a[0].3` works on that path.

The two scratch files in `/private/tmp/stcheck` should, by this reading, give these results:
- `t.st` should fail with `E202`, because it uses `E_X.A` and `E_X.B`.
- `t2.st` should pass the hir check, since it uses bare `A` and `B`. It contains no PROGRAM, so the runtime compile step will still fail.
## Pin update (2026-11-11)

plan2cell now pins trust-platform tag **v0.24.70** (commit `ecbcb08d`, 2026-10-09) instead of v0.24.69. It is the first tag on salsa 0.28 (locked 0.28.5), which fixes RUSTSEC-2026-0308 (use-after-free in salsa 0.23–0.28.4 interned values; salsa 0.26.2 came in through trust-hir). bt2plc needed no code changes: the APIs it uses (`trust_syntax::parser::parse`, `trust_hir::Project` / `SemanticDatabase::diagnostics`, `trust_runtime::harness::{TestHarness, CompileSession, SourceFile}`, `trust_runtime::value`) are unchanged. The analysis above still describes v0.24.69. Unchanged in v0.24.70: trust-runtime still requires `zenoh = "=1.7.2"` and `tiny_http` with `ssl-rustls` (rustls 0.20.9), and the vendored tiverse-mmap is still needed.
