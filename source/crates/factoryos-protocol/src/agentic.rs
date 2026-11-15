//! Agentic Planning: the umarell multi-robot planner stack (`multi_robots_agents`), started and
//! stopped by the backend. Planning itself is driven from the Kiro chat (its MCP servers).

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimState {
    #[default]
    Stopped,
    Starting,
    Ready,
    Failed,
}

/// How the stack runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimMode {
    /// The full stack via `make sim-detached` (Docker): orchestrator, planner, cell controller, robots.
    Docker,
    /// Only the planner MCP server (:8890), from the checkout's `.venv-planner` (no Docker/ROS).
    Native,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PrepareState {
    #[default]
    Idle,
    Preparing,
    Done,
    Failed,
}

/// `GET /api/v1/agentic/status`, also published on `factoryos/agentic/status`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct AgenticStatus {
    pub state: SimState,
    pub mode: Option<SimMode>,
    /// The multi_robots_agents checkout.
    pub repo_dir: String,
    /// The cell the stack runs for.
    pub cell: String,
    pub error: Option<String>,
    pub prepare: PrepareState,
    pub prepare_error: Option<String>,
    /// The cell the last (or the running) Prepare Cell is for: the active cell unless the request named one.
    pub prepare_cell: String,
}

/// `POST /api/v1/agentic/prepare` body (optional): the cell folder to prepare, default the active cell.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct AgenticPrepare {
    pub cell: Option<String>,
}
