//! The MCP tools. Each one runs the plan2cell pipeline on a blocking thread and answers with a
//! JSON document (text content). A pipeline error is a tool result with `isError` set and the
//! plan2cell exit code, not a protocol error, so the client can show it and retry.

use std::path::PathBuf;
use std::sync::Arc;

use rmcp::handler::server::router::tool::ToolRouter;
use rmcp::handler::server::wrapper::Parameters;
use rmcp::model::{CallToolResult, ContentBlock, ServerCapabilities, ServerConfig};
use rmcp::schemars::{self, JsonSchema};
use rmcp::{ErrorData as McpError, ServerHandler, tool, tool_handler, tool_router};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use plan2cell::{BuildConfig, BuildOptions, PlanInput, PlcTarget, Profile};

use crate::{Error, Sandbox, Settings};

/// Largest file `plan2cell_read_file` returns and the largest inline file content of a build.
const MAX_FILE_BYTES: usize = 512 * 1024;

#[derive(Clone)]
pub struct Plan2cellServer {
    inner: Arc<Inner>,
    tool_router: ToolRouter<Self>,
}

struct Inner {
    sandbox: Sandbox,
    config: Option<PathBuf>,
    allow_write: bool,
}

// ───────────────────────────── arguments ─────────────────────────────

/// The plan: a file under the server root, or the plan text itself.
#[derive(Debug, Default, Deserialize, Serialize, JsonSchema)]
pub struct PlanArg {
    /// Plan file relative to the server root (umarell plan JSON or BT.CPP v4 XML).
    pub plan_path: Option<String>,
    /// The plan itself (JSON or XML), instead of `plan_path`.
    pub plan_text: Option<String>,
    /// File name for `plan_text` (it appears in the generated file headers).
    pub plan_name: Option<String>,
    /// Blackboard JSON file (relative to the root) for a plain BT.CPP XML tree.
    pub blackboard_path: Option<String>,
}

/// Configuration: a plan2cell.toml under the root, inline TOML, or the server default.
#[derive(Debug, Default, Deserialize, Serialize, JsonSchema)]
pub struct ConfigArg {
    /// plan2cell.toml relative to the server root.
    pub config_path: Option<String>,
    /// plan2cell.toml content, instead of `config_path`.
    pub config_toml: Option<String>,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "kebab-case")]
pub enum ProfileArg {
    /// bt2plc simulator telegram (docker simulation, OfficeLite, test robots).
    Sim,
    /// Real VW KS24 robots: VW Folge handshake and robot interlocks.
    VwKs24,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "kebab-case")]
pub enum PlcArg {
    /// CODESYS V3.5 projects.
    Codesys,
    /// No PLC code, only the plan outputs the robot programs come from.
    None,
}

/// What to build; every field defaults to the config's `[build]` section.
#[derive(Debug, Default, Deserialize, Serialize, JsonSchema)]
pub struct OptionsArg {
    pub profile: Option<ProfileArg>,
    pub plc: Option<PlcArg>,
    /// Robot simulator projects next to the head PLC (profile sim).
    pub robot_sim: Option<bool>,
    /// KUKA KRL robot programs.
    pub krl: Option<bool>,
    /// Only these robots get KRL.
    pub krl_robots: Option<Vec<String>>,
    /// Type-check the generated ST with trust.
    pub st_check: Option<bool>,
    /// Warnings are errors.
    pub strict: Option<bool>,
}

#[derive(Debug, Deserialize, Serialize, JsonSchema)]
pub struct PlanArgs {
    #[serde(flatten)]
    pub plan: PlanArg,
    #[serde(flatten)]
    pub config: ConfigArg,
    #[serde(flatten)]
    pub options: OptionsArg,
}

#[derive(Debug, Deserialize, Serialize, JsonSchema)]
pub struct BuildArgs {
    #[serde(flatten)]
    pub plan: PlanArg,
    #[serde(flatten)]
    pub config: ConfigArg,
    #[serde(flatten)]
    pub options: OptionsArg,
    /// Write the build into this directory (relative to the root). Needs a server started with
    /// writing allowed. Without it nothing is written.
    pub output_dir: Option<String>,
    /// Replace a non-empty output directory (files of the previous build are removed).
    #[serde(default)]
    pub overwrite: bool,
    /// Return the content of these generated files (paths as in the file list, e.g.
    /// `krl/robot1/KRC/R1/Folgen/folge11.src`).
    #[serde(default)]
    pub include_files: Vec<String>,
}

#[derive(Debug, Deserialize, Serialize, JsonSchema)]
pub struct ExplainArgs {
    #[serde(flatten)]
    pub plan: PlanArg,
    #[serde(flatten)]
    pub config: ConfigArg,
    #[serde(flatten)]
    pub options: OptionsArg,
    /// Robot namespace (e.g. `robot1`).
    pub robot: String,
}

#[derive(Debug, Deserialize, Serialize, JsonSchema)]
pub struct ReadArgs {
    /// File relative to the server root.
    pub path: String,
}

#[derive(Debug, Default, Deserialize, Serialize, JsonSchema)]
pub struct ListArgs {
    /// Directory relative to the server root (default: the root).
    pub dir: Option<String>,
}

#[cfg(feature = "sim")]
#[derive(Debug, Clone, Copy, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "kebab-case")]
pub enum RobotsArg {
    /// The executor of the generated KRL.
    Krl,
    /// The generated CODESYS device simulators (profile sim).
    DeviceSt,
}

#[cfg(feature = "sim")]
#[derive(Debug, Deserialize, Serialize, JsonSchema)]
pub struct SimulateArgs {
    #[serde(flatten)]
    pub plan: PlanArg,
    #[serde(flatten)]
    pub config: ConfigArg,
    #[serde(flatten)]
    pub options: OptionsArg,
    /// Production cycles to run (default 1).
    pub cycles: Option<u32>,
    /// Who plays the robots (default krl).
    pub robots: Option<RobotsArg>,
}

// ───────────────────────────── helpers ─────────────────────────────

fn ok_json(v: Value) -> CallToolResult {
    CallToolResult::success(vec![ContentBlock::text(
        serde_json::to_string_pretty(&v).unwrap_or_default(),
    )])
}

fn tool_error(code: i32, message: impl Into<String>) -> CallToolResult {
    let v = json!({"error": message.into(), "exit_code": code});
    CallToolResult::error(vec![ContentBlock::text(
        serde_json::to_string_pretty(&v).unwrap_or_default(),
    )])
}

fn from_plan2cell(e: plan2cell::Error) -> CallToolResult {
    tool_error(e.exit_code(), e.to_string())
}

/// Run blocking pipeline work off the async runtime.
async fn blocking<F>(f: F) -> Result<CallToolResult, McpError>
where
    F: FnOnce() -> CallToolResult + Send + 'static,
{
    tokio::task::spawn_blocking(f)
        .await
        .map_err(|e| McpError::internal_error(format!("tool task failed: {e}"), None))
}

impl Inner {
    fn plan(&self, a: &PlanArg) -> Result<PlanInput, CallToolResult> {
        let mut input = match (&a.plan_path, &a.plan_text) {
            (Some(p), None) => {
                let path = self.sandbox.existing(p).map_err(|e| tool_error(2, e))?;
                PlanInput::load(&path).map_err(from_plan2cell)?
            }
            (None, Some(t)) => PlanInput::from_text(
                t.clone(),
                a.plan_name.clone().unwrap_or_else(|| "plan.json".into()),
            ),
            _ => return Err(tool_error(2, "give exactly one of plan_path or plan_text")),
        };
        if let Some(b) = &a.blackboard_path {
            let path = self.sandbox.existing(b).map_err(|e| tool_error(2, e))?;
            input.load_blackboard(&path).map_err(from_plan2cell)?;
        }
        Ok(input)
    }

    fn config(&self, a: &ConfigArg) -> Result<BuildConfig, CallToolResult> {
        match (&a.config_path, &a.config_toml) {
            (Some(_), Some(_)) => Err(tool_error(
                2,
                "give at most one of config_path or config_toml",
            )),
            (Some(p), None) => {
                let path = self.sandbox.existing(p).map_err(|e| tool_error(2, e))?;
                BuildConfig::load(&path).map_err(from_plan2cell)
            }
            (None, Some(t)) => BuildConfig::parse(t).map_err(from_plan2cell),
            (None, None) => match &self.config {
                Some(p) => BuildConfig::load(p).map_err(from_plan2cell),
                None => Ok(BuildConfig::default()),
            },
        }
    }
}

fn options(cfg: &BuildConfig, a: &OptionsArg) -> BuildOptions {
    let mut o = cfg.options();
    if let Some(p) = a.profile {
        o.profile = match p {
            ProfileArg::Sim => Profile::Sim,
            ProfileArg::VwKs24 => Profile::VwKs24,
        };
    }
    if let Some(p) = a.plc {
        o.plc = match p {
            PlcArg::Codesys => PlcTarget::Codesys,
            PlcArg::None => PlcTarget::None,
        };
    }
    if let Some(b) = a.robot_sim {
        o.robot_sim = b;
    }
    if let Some(b) = a.krl {
        o.krl = b;
    }
    if a.krl_robots.is_some() {
        o.krl_robots = a.krl_robots.clone();
    }
    if let Some(b) = a.st_check {
        o.st_check = b && cfg!(feature = "check");
    }
    if let Some(b) = a.strict {
        o.strict = b;
    }
    o
}

macro_rules! tri {
    ($e:expr) => {
        match $e {
            Ok(v) => v,
            Err(r) => return r,
        }
    };
}

// ───────────────────────────── tools ─────────────────────────────

#[tool_router(router = tool_router)]
impl Plan2cellServer {
    pub fn new(settings: Settings) -> Result<Self, Error> {
        let sandbox = Sandbox::new(&settings.root)
            .map_err(|e| Error::Config(format!("root {}: {e}", settings.root.display())))?;
        let config = match settings.config {
            Some(p) if p.is_absolute() => Some(p),
            Some(p) => Some(sandbox.root().join(p)),
            None => None,
        };
        if let Some(p) = &config {
            BuildConfig::load(p).map_err(|e| Error::Config(e.to_string()))?;
        }
        Ok(Self {
            inner: Arc::new(Inner {
                sandbox,
                config,
                allow_write: settings.allow_write,
            }),
            tool_router: Self::routers(),
        })
    }

    #[cfg(feature = "sim")]
    fn routers() -> ToolRouter<Self> {
        Self::tool_router() + Self::sim_router()
    }

    #[cfg(not(feature = "sim"))]
    fn routers() -> ToolRouter<Self> {
        Self::tool_router()
    }

    #[tool(
        name = "plan2cell_zones",
        description = "Plan a behaviour tree without generating code: the interlock zones with \
                       their ticket order, each robot's op list and the planned makespan."
    )]
    async fn zones(&self, Parameters(a): Parameters<PlanArgs>) -> Result<CallToolResult, McpError> {
        let inner = self.inner.clone();
        blocking(move || {
            let input = tri!(inner.plan(&a.plan));
            let cfg = tri!(inner.config(&a.config));
            let opts = options(&cfg, &a.options);
            let (p, diags) = tri!(plan2cell::plan(&input, &cfg, &opts).map_err(from_plan2cell));
            ok_json(json!({
                "tree_id": p.tree_id,
                "cell_id": p.cell_id,
                "plan_id": format!("16#{:04X}", p.plan_id),
                "makespan_s": p.makespan_ms as f64 / 1000.0,
                "zones": p.zones.iter().map(|z| json!({
                    "name": z.name,
                    "tickets": z.tickets.iter().map(|t| p.op_ref(t.op)).collect::<Vec<_>>(),
                })).collect::<Vec<_>>(),
                "permit_edges": p.permit_edges.iter()
                    .map(|e| format!("{} -> {}", p.op_ref(e.from), p.op_ref(e.to)))
                    .collect::<Vec<_>>(),
                "robots": p.robots.iter().map(|r| json!({
                    "ns": r.ns,
                    "ops": r.ops.iter().map(|&o| {
                        let op = &p.ops[o];
                        json!({
                            "index": op.index,
                            "operation_id": op.operation_id,
                            "zones": op.zones.iter().map(|&z| p.zones[z].name.clone()).collect::<Vec<_>>(),
                            "permit": !op.permit_preds.is_empty(),
                            "expected_s": op.expected_ms as f64 / 1000.0,
                        })
                    }).collect::<Vec<_>>(),
                })).collect::<Vec<_>>(),
                "diagnostics": diags,
            }))
        })
        .await
    }

    #[tool(
        name = "plan2cell_check",
        description = "Generate everything in memory and validate it (PLC planning invariants, \
                       trust ST type check, the strict KRL validator and the KRL round trip) \
                       without writing anything. Returns the build summary and the warnings."
    )]
    async fn check(&self, Parameters(a): Parameters<PlanArgs>) -> Result<CallToolResult, McpError> {
        let inner = self.inner.clone();
        blocking(move || {
            let input = tri!(inner.plan(&a.plan));
            let cfg = tri!(inner.config(&a.config));
            let opts = options(&cfg, &a.options);
            let b = tri!(plan2cell::build(&input, &cfg, &opts).map_err(from_plan2cell));
            ok_json(json!({"ok": true, "summary": b.summary}))
        })
        .await
    }

    #[tool(
        name = "plan2cell_build",
        description = "Build the head-PLC code and/or the KUKA robot programs for a plan. \
                       Returns the summary and the list of generated files; optionally writes \
                       them to output_dir (under the server root, if writing is allowed) and \
                       returns the content of selected files."
    )]
    async fn build(
        &self,
        Parameters(a): Parameters<BuildArgs>,
    ) -> Result<CallToolResult, McpError> {
        let inner = self.inner.clone();
        blocking(move || {
            let input = tri!(inner.plan(&a.plan));
            let cfg = tri!(inner.config(&a.config));
            let opts = options(&cfg, &a.options);
            let b = tri!(plan2cell::build(&input, &cfg, &opts).map_err(from_plan2cell));
            let mut written = None;
            if let Some(dir) = &a.output_dir {
                if !inner.allow_write {
                    return tool_error(
                        2,
                        "this server does not allow writing (start it with --allow-write)",
                    );
                }
                let path = tri!(inner.sandbox.new_path(dir).map_err(|e| tool_error(2, e)));
                tri!(b.write_to(&path, a.overwrite).map_err(from_plan2cell));
                written = Some(dir.clone());
            }
            let mut contents = serde_json::Map::new();
            for f in &a.include_files {
                match b.files.get(f) {
                    Some(t) if t.len() <= MAX_FILE_BYTES => {
                        contents.insert(f.clone(), Value::String(t.clone()));
                    }
                    Some(t) => {
                        return tool_error(
                            2,
                            format!("{f} is {} bytes, more than {MAX_FILE_BYTES}", t.len()),
                        );
                    }
                    None => return tool_error(2, format!("no generated file {f}")),
                }
            }
            ok_json(json!({
                "summary": b.summary,
                "written_to": written,
                "files": b.files.iter().map(|(k, v)| json!({"path": k, "bytes": v.len()})).collect::<Vec<_>>(),
                "contents": contents,
            }))
        })
        .await
    }

    #[tool(
        name = "plan2cell_explain_robot",
        description = "One robot's KRL plan: its ops with zone masks, the estimated, planned and \
                       watchdog times, and the warnings."
    )]
    async fn explain_robot(
        &self,
        Parameters(a): Parameters<ExplainArgs>,
    ) -> Result<CallToolResult, McpError> {
        let inner = self.inner.clone();
        blocking(move || {
            let input = tri!(inner.plan(&a.plan));
            let cfg = tri!(inner.config(&a.config));
            let mut opts = options(&cfg, &a.options);
            opts.krl = true;
            opts.st_check = false;
            let b = tri!(plan2cell::build(&input, &cfg, &opts).map_err(from_plan2cell));
            let text = tri!(plan2cell::explain_robot(&b, &a.robot).map_err(from_plan2cell));
            CallToolResult::success(vec![ContentBlock::text(text)])
        })
        .await
    }

    #[tool(
        name = "plan2cell_default_config",
        description = "The default plan2cell.toml with every option and its default, commented."
    )]
    async fn default_config(&self) -> Result<CallToolResult, McpError> {
        Ok(CallToolResult::success(vec![ContentBlock::text(
            BuildConfig::default_toml(),
        )]))
    }

    #[tool(
        name = "plan2cell_list_files",
        description = "List plans, configs and build outputs under the server root (one level)."
    )]
    async fn list_files(
        &self,
        Parameters(a): Parameters<ListArgs>,
    ) -> Result<CallToolResult, McpError> {
        let inner = self.inner.clone();
        blocking(move || {
            let dir = match &a.dir {
                Some(d) => tri!(inner.sandbox.existing(d).map_err(|e| tool_error(2, e))),
                None => inner.sandbox.root().to_path_buf(),
            };
            let rd = tri!(std::fs::read_dir(&dir).map_err(|e| tool_error(4, e.to_string())));
            let mut entries: Vec<Value> = rd
                .filter_map(|e| e.ok())
                .filter(|e| !e.file_name().to_string_lossy().starts_with('.'))
                .map(|e| {
                    let rel = e
                        .path()
                        .strip_prefix(inner.sandbox.root())
                        .map(|p| p.to_string_lossy().into_owned())
                        .unwrap_or_default();
                    json!({"path": rel, "dir": e.path().is_dir()})
                })
                .collect();
            entries.sort_by_key(|v| v["path"].as_str().unwrap_or_default().to_string());
            ok_json(json!({"entries": entries}))
        })
        .await
    }

    #[tool(
        name = "plan2cell_read_file",
        description = "Read a file under the server root, e.g. a generated KRL program or INSTALL.md."
    )]
    async fn read_file(
        &self,
        Parameters(a): Parameters<ReadArgs>,
    ) -> Result<CallToolResult, McpError> {
        let inner = self.inner.clone();
        blocking(move || {
            let path = tri!(
                inner
                    .sandbox
                    .existing(&a.path)
                    .map_err(|e| tool_error(2, e))
            );
            let meta = tri!(std::fs::metadata(&path).map_err(|e| tool_error(4, e.to_string())));
            if !meta.is_file() || meta.len() as usize > MAX_FILE_BYTES {
                return tool_error(
                    2,
                    format!("{} is not a file of at most {MAX_FILE_BYTES} bytes", a.path),
                );
            }
            let t = tri!(std::fs::read_to_string(&path).map_err(|e| tool_error(4, e.to_string())));
            CallToolResult::success(vec![ContentBlock::text(t)])
        })
        .await
    }
}

#[cfg(feature = "sim")]
#[tool_router(router = sim_router)]
impl Plan2cellServer {
    #[tool(
        name = "plan2cell_simulate",
        description = "Co-simulate the build: the generated head-PLC ST runs against the robots \
                       (the executor of the generated KRL, or the CODESYS device simulators) and \
                       the trace is checked against the plan. Slow: minutes per cycle."
    )]
    async fn simulate(
        &self,
        Parameters(a): Parameters<SimulateArgs>,
    ) -> Result<CallToolResult, McpError> {
        let inner = self.inner.clone();
        blocking(move || {
            let input = tri!(inner.plan(&a.plan));
            let cfg = tri!(inner.config(&a.config));
            let mut opts = options(&cfg, &a.options);
            opts.st_check = false;
            let robots = match a.robots.unwrap_or(RobotsArg::Krl) {
                RobotsArg::Krl => plan2cell::sim::Robots::Krl,
                RobotsArg::DeviceSt => plan2cell::sim::Robots::DeviceSt,
            };
            if robots == plan2cell::sim::Robots::Krl {
                opts.krl = true;
            }
            let b = tri!(plan2cell::build(&input, &cfg, &opts).map_err(from_plan2cell));
            let (_, r) = tri!(
                plan2cell::sim::simulate(&b, &cfg, robots, a.cycles.unwrap_or(1).max(1))
                    .map_err(from_plan2cell)
            );
            let ok = r.ok;
            let v = serde_json::to_value(&r).unwrap_or_default();
            if ok {
                ok_json(v)
            } else {
                CallToolResult::error(vec![ContentBlock::text(v.to_string())])
            }
        })
        .await
    }
}

#[tool_handler(router = self.tool_router)]
impl ServerHandler for Plan2cellServer {
    fn get_info(&self) -> ServerConfig {
        ServerConfig::new(ServerCapabilities::builder().enable_tools().build()).with_instructions(
            "plan2cell turns a behaviour-tree plan of a robot cell into head-PLC code (CODESYS) and \
             KUKA KRL robot programs that work together. Plans and configs are files under the \
             server root (see plan2cell_list_files) or inline text. Start with plan2cell_zones to see \
             the interlock zones, plan2cell_check to validate, plan2cell_build to generate. Profile \
             'sim' targets the docker simulation; 'vw-ks24' real VW KS24 robots. Generated code \
             is not safety-rated and not yet tested on a controller: read each robot's INSTALL.md.",
        )
    }
}
