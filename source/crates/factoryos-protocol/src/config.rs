/// The default "Plan Cell" prompt for the planner agent. Placeholders: `{cell}` (the cell's folder
/// name), `{cell_id}` (the planner's cell id), `{task}` (the picked task program, or empty) and
/// `{planner}` (the planner picked in Agentic Planning: `auto`, `umarell`, `hier_la_fu`, …). The
/// agent plans through the planner router (deploy/planner-router), which runs umarell or one of
/// factoryos-cell-planner's learned policies.
pub const DEFAULT_PLAN_PROMPT: &str = "Plan the Physical AI Production Designer cell `{cell}` (planner cell id `{cell_id}`), task program `{task}`, \
planner `{planner}`. \
Plan through the planner router: call `recommend_planner` with the cell, the task program and planner `{planner}` \
(`auto`: you choose); pass `request` only with wishes I add in the chat, not this message. If it is ambiguous, ask me its question and wait for my answer; otherwise tell me which \
planner you use and name the alternatives in one line. Then call `plan` and follow it with `plan_status`, \
telling me what happens. When it is done, summarise the plan: planner, plan file, end time and checks, and say \
so if it was published as unverified. \
If the cell has no task program, call `task_program_from_products` first: it writes one from the Product & Task \
welds of the products in the cell; plan that and tell me what it wrote. Never invent weld points the product does \
not define; if the cell has no product with welds, tell me which points you propose and ask me before you save \
and plan them. \
If the planner router's tools are not available, plan with umarell directly: `get_cell_info`, \
`list_task_programs`, then `plan_trajectories` followed by `get_plan_status` / `get_plan_events`, and \
`get_plan_quality` at the end. \
Don't execute the plan: I will run the saved behaviour tree in the Physical AI Production Designer Behaviour Tree Executor.";

/// Earlier defaults of [`DEFAULT_PLAN_PROMPT`]: a saved config that still has one of them unchanged
/// gets the current default ([`Config::upgrade_default_prompts`]); an edited prompt is kept.
pub const PREVIOUS_PLAN_PROMPTS: &[&str] = &[PLAN_PROMPT_V1, PLAN_PROMPT_V2, PLAN_PROMPT_V3];

/// The plan prompt before the rename to Physical AI Production Designer.
const PLAN_PROMPT_V3: &str = "Plan the Factory OS cell `{cell}` (planner cell id `{cell_id}`), task program `{task}`, \
planner `{planner}`. \
Plan through the planner router: call `recommend_planner` with the cell, the task program and planner `{planner}` \
(`auto`: you choose); pass `request` only with wishes I add in the chat, not this message. If it is ambiguous, ask me its question and wait for my answer; otherwise tell me which \
planner you use and name the alternatives in one line. Then call `plan` and follow it with `plan_status`, \
telling me what happens. When it is done, summarise the plan: planner, plan file, end time and checks, and say \
so if it was published as unverified. \
If the cell has no task program, call `task_program_from_products` first: it writes one from the Product & Task \
welds of the products in the cell; plan that and tell me what it wrote. Never invent weld points the product does \
not define; if the cell has no product with welds, tell me which points you propose and ask me before you save \
and plan them. \
If the planner router's tools are not available, plan with umarell directly: `get_cell_info`, \
`list_task_programs`, then `plan_trajectories` followed by `get_plan_status` / `get_plan_events`, and \
`get_plan_quality` at the end. \
Don't execute the plan: I will run the saved behaviour tree in the Factory OS Behaviour Tree Executor.";

/// The planner router's plan prompt before `task_program_from_products` (it had the agent derive weld
/// points from the fixtures).
const PLAN_PROMPT_V2: &str = "Plan the Factory OS cell `{cell}` (planner cell id `{cell_id}`), task program `{task}`, \
planner `{planner}`. \
Plan through the planner router: call `recommend_planner` with the cell, the task program and planner `{planner}` \
(`auto`: you choose); pass `request` only with wishes I add in the chat, not this message. If it is ambiguous, ask me its question and wait for my answer; otherwise tell me which \
planner you use and name the alternatives in one line. Then call `plan` and follow it with `plan_status`, \
telling me what happens. When it is done, summarise the plan: planner, plan file, end time and checks, and say \
so if it was published as unverified. \
If umarell plans it and the cell has no task program, derive the operations from the cell's fixtures (their \
scene.yaml pose and geometry), save it with `save_task_program`, and tell me what you chose; if the cell has no \
fixture to work on, stop and say so instead of inventing one. \
If the planner router's tools are not available, plan with umarell directly: `get_cell_info`, \
`list_task_programs`, then `plan_trajectories` followed by `get_plan_status` / `get_plan_events`, and \
`get_plan_quality` at the end. \
Don't execute the plan: I will run the saved behaviour tree in the Factory OS Behaviour Tree Executor.";

/// The plan prompt before the planner router (umarell only).
const PLAN_PROMPT_V1: &str = "Plan the Factory OS cell `{cell}` (planner cell id `{cell_id}`), task program `{task}`. \
Start with `get_cell_info` and `list_task_programs`. \
If the cell has no task program, derive the operations from the cell's fixtures (their scene.yaml pose and \
geometry), save it with `save_task_program`, and tell me what you chose; if the cell has no fixture to work \
on, stop and say so instead of inventing one. \
Then run `plan_trajectories` and follow it with `get_plan_status` / `get_plan_events`, telling me what \
happens at each step. When it is done, summarise the plan with `get_plan_quality`. \
Don't execute the plan: I will run the saved behaviour tree in the Factory OS Behaviour Tree Executor.";

// The agent prompts of the flow's steps. Placeholders, filled in by the viewer: `{product}` (the
// current product's name), `{product_file}` (its `product.json`), `{product_dir}` (its folder),
// `{catalogue}`, `{asset_root}`, `{fitted_robots_dir}`, and the active cell's `{cell}`,
// `{cell_id}` and `{cell_dir}`.

/// Product & Task, "Create task description": the product's tasks worked out into operations, in
/// the product's own frame. Placing the part and the robots is the Robo Fitter's step.
pub const DEFAULT_TASK_PROMPT: &str = "Write the task description for the product `{product}` from its definition \
`{product_file}`: the part's geometry and its tasks. Each task is one process (e.g. spot welding, seam welding, \
gluing, screwing) done with one tool, with the process parameters; its operations are points or paths on the part, \
each with an outward surface normal, in metres in the product's own frame (z up; a box stands on its origin). \
Don't place the part or any robot: that happens later in the Robo Fitter, and the planner turns this description \
into a cell's task program then. For every operation work out, still in the product frame, the tool orientation \
(working against the surface normal), the approach and retract offsets, the process steps with their parameters \
(e.g. approach, contact, squeeze, weld, hold, retract for a spot weld) and the order: group and sequence the \
operations sensibly for the process. Note the tool each task needs and what it has to be able to do. \
Write it as `{product_dir}/task.json`: JSON with `product`, `frame` (\"product\"), `version`, `description`, \
`tasks` (name, process, tool, params) and `operations` (id, task, type, position or path, normal, \
approach_vector, orientation as a quaternion, sequence_group, priority, steps, params). Then summarise it.";

/// Robo Dresser, "Suggest robots & tools": which robot + tool pairs the product's tasks need.
pub const DEFAULT_DRESSER_PROMPT: &str = "Help me dress robots for the product `{product}`. Its task description \
is `{product_dir}/task.json` (if that doesn't exist yet, use the product definition `{product_file}`): each task \
names a process and the tool it needs. The Robo Dresser catalogue is `{catalogue}`: robots in \
urdf-robot-collection/, tools in urdf-tool-collection/, robots already dressed in urdf-dressed-robots/. Look at the \
robots (reach, payload where given) and the tools there, and tell me which robot + tool pairs to dress for the \
tasks, with the dress options (collision hull level, tool orientation in 90° steps), and which tasks no catalogue \
tool fits. Don't write into the catalogue: I dress the pairs in the Robo Dresser.";

/// Robo Fitter, "Propose cell layout": the robots and the part placed so every operation is reachable.
pub const DEFAULT_FITTER_PROMPT: &str = "Propose a cell layout for the product `{product}`. Its task description \
is `{product_dir}/task.json` (if that doesn't exist yet, use the product definition `{product_file}`); its \
operations are in the part's own frame. The Robo Fitter's asset root is `{asset_root}`: dressed robots in \
urdf-dressed-robots/, fixtures in urdf-fixture-collection/, floor plans in floor-plans/; saved scenes are \
folders `{fitted_robots_dir}/<name>/scene.yaml` (a scene is referred to by its name). Choose the dressed robots whose tools fit the tasks, place the part (on a fixture or a work \
table, as a fixture whose `urdf` is the product's `{product_dir}/{product}.urdf`) and the robots so that every \
operation is within reach with room for the approach, and keep the robots clear of each other. Write it as a new \
scene `{fitted_robots_dir}/{product}_cell/scene.yaml` in the YAML format of the existing scenes there (`name`: the \
folder's name, `{product}_cell`; its relative paths start at its folder, `../../urdf-dressed-robots/…`; `robots` \
with `robot_id`, `dressed_urdf`, `position` x/y/z in metres, `orientation` a/b/c in degrees, optional \
`home_pose_deg` and `pedestal`; `fixtures` with `id`, `urdf`, `position`, `orientation`); don't overwrite an \
existing scene. Before you finish, call `check_reachability` on it (mode `quick`, `show_in_fitter` true); if \
welds are not reached, try `suggest_layout` with `write_as` a new scene name and check that. Then tell me what you \
placed where, where the part sits in the cell, which robot reaches which weld, and why. I open it in the Robo \
Fitter to check and adjust it, and create the cell from there.";

/// Earlier defaults of [`DEFAULT_FITTER_PROMPT`] (see [`Config::upgrade_default_prompts`]).
pub const PREVIOUS_FITTER_PROMPTS: &[&str] = &[FITTER_PROMPT_V1, FITTER_PROMPT_V2];

/// The layout prompt of the flat scene layout (`scenes/<name>.yaml`).
const FITTER_PROMPT_V2: &str = "Propose a cell layout for the product `{product}`. Its task description \
is `{product_dir}/task.json` (if that doesn't exist yet, use the product definition `{product_file}`); its \
operations are in the part's own frame. The Robo Fitter's asset root is `{asset_root}`: dressed robots in \
urdf-dressed-robots/, fixtures in urdf-fixture-collection/, floor plans in floor-plans/; saved scenes are in \
`{fitted_robots_dir}`. Choose the dressed robots whose tools fit the tasks, place the part (on a fixture or a work \
table, as a fixture whose `urdf` is the product's `{product_dir}/{product}.urdf`) and the robots so that every \
operation is within reach with room for the approach, and keep the robots clear of each other. Write it as a new \
scene `{fitted_robots_dir}/{product}_cell.yaml` in the YAML format of the existing scenes there (`name`, `robots` \
with `robot_id`, `dressed_urdf`, `position` x/y/z in metres, `orientation` a/b/c in degrees, optional \
`home_pose_deg` and `pedestal`; `fixtures` with `id`, `urdf`, `position`, `orientation`); don't overwrite an \
existing scene. Before you finish, call `check_reachability` on it (mode `quick`, `show_in_fitter` true); if \
welds are not reached, try `suggest_layout` with `write_as` a new file name and check that. Then tell me what you \
placed where, where the part sits in the cell, which robot reaches which weld, and why. I open it in the Robo \
Fitter to check and adjust it, and create the cell from there.";

/// The layout prompt that asked for a `.json` scene and no reachability check.
const FITTER_PROMPT_V1: &str = "Propose a cell layout for the product `{product}`. Its task description \
is `{product_dir}/task.json` (if that doesn't exist yet, use the product definition `{product_file}`); its \
operations are in the part's own frame. The Robo Fitter's asset root is `{asset_root}`: dressed robots in \
urdf-dressed-robots/, fixtures in urdf-fixture-collection/, floor plans in floor-plans/; saved scenes are in \
`{fitted_robots_dir}`. Choose the dressed robots whose tools fit the tasks, place the part (on a fixture or a work \
table) and the robots so that every operation is within reach with room for the approach, and keep the robots \
clear of each other. Write it as a new scene `{fitted_robots_dir}/{product}_cell.json` in the format of the \
existing scenes there, then tell me what you placed where, where the part sits in the cell, and why. I open it in \
the Robo Fitter to check and adjust it, and create the cell from there.";

/// Robo Fitter, "Check reachability (agent)": the agent checks the open scene with the planner's
/// IK, explains what isn't reached and proposes a moved layout as a new scene. Placeholders:
/// `{scene_file}` (the scene's absolute path), `{fitted_robots_dir}` and `{cell}`.
pub const DEFAULT_REACH_PROMPT: &str = "Check whether the robots of the Robo Fitter scene `{scene_file}` (cell \
`{cell}`, if any) reach every weld of its task. Call `check_reachability(scene=\"{scene_file}\", mode=\"full\", \
show_in_fitter=true)` from the planner router; it moves the robots to each weld with the cell planner's IK. For \
every red (no robot) or orange (one robot) weld, explain the reason (out of reach by how much, joint limits, \
self collision, the tool inside the product, …). If welds are not reached, call `suggest_layout(scene=\"{scene_file}\", \
write_as=\"<name>-fit\")` (<name> is the scene's name, its folder in `{fitted_robots_dir}`; pick another new \
name if it exists), then `check_reachability` on the new scene; at most 3 rounds. Never overwrite `{scene_file}` or any \
other scene in `{fitted_robots_dir}`. Finally report, per weld, which robot reaches it with which joints, and what \
you moved (robot, by how much, pedestal) and in which new scene. I review it in the Robo Fitter.";
/// Earlier defaults of [`DEFAULT_REACH_PROMPT`] (see [`Config::upgrade_default_prompts`]).
pub const PREVIOUS_REACH_PROMPTS: &[&str] = &[REACH_PROMPT_V1];

/// The reach prompt of the flat scene layout (`write_as` a `<stem>-fit.yaml`).
const REACH_PROMPT_V1: &str = "Check whether the robots of the Robo Fitter scene `{scene_file}` (cell \
`{cell}`, if any) reach every weld of its task. Call `check_reachability(scene=\"{scene_file}\", mode=\"full\", \
show_in_fitter=true)` from the planner router; it moves the robots to each weld with the cell planner's IK. For \
every red (no robot) or orange (one robot) weld, explain the reason (out of reach by how much, joint limits, \
self collision, the tool inside the product, …). If welds are not reached, call `suggest_layout(scene=\"{scene_file}\", \
write_as=\"<stem>-fit.yaml\")` (<stem> is the scene's file name without `.yaml`; pick another new name if it \
exists), then `check_reachability` on the new scene; at most 3 rounds. Never overwrite `{scene_file}` or any \
other scene in `{fitted_robots_dir}`. Finally report, per weld, which robot reaches it with which joints, and what \
you moved (robot, by how much, pedestal) and in which new scene file. I review it in the Robo Fitter.";

/// Settings, owned by the backend and stored in its JSON config file. Mirrors the Kiro
/// extension's Configuration sidebar (`configuration-sidebar.ts`), section by section; defaults
/// are the extension's `package.json` ones. Passwords are deliberately not part of this (see [`Secrets`]).
///
/// **User and machine settings.** With S3 storage the user's settings roam: their store of record
/// is `users/<owner>/factoryos.config.json` ([`USER_CONFIG_KEY`]), shared by every backend of that
/// user (the workspace, a native viewer's local backend), and the local file is a cache that also
/// holds the machine's settings. The split, field by field:
/// * [`USER_FIELDS`] roam: the agent prompts, the Robo Fitter's default scene, the cell runner's
///   mesh LOD and collision check, the planner's backend, model and tool trust, the EDH usernames
///   and timings, and the active cell by name ([`ACTIVE_CELL_FIELD`]; each backend points its own
///   `cellDir`/`envDir`/`scene` at that cell of its own cells folder).
/// * [`MACHINE_FIELDS`] never leave the machine: every folder (products, catalogue, assets, cells,
///   the active cell's paths, trajectories, the umarell checkout), binaries, web roots, scripts and
///   venvs, local ports, hosts and URLs, the planner stack's RMW/Zenoh router/RViz, and the AWS
///   region. Provisioning (`deploy/provision.sh`, `factoryos-deploy`) and `FACTORY_OS_*` set these.
///
/// Loading merges the two (machine fields from the local file, user fields from S3, which wins);
/// saving writes only the user fields to S3, conditionally on its last ETag. Without S3 the file
/// holds both, as before.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct Config {
    // --- Product & Task
    /// Product definitions (`<name>/product.json`); empty = `products/` next to the config file.
    pub products_dir: String,
    /// "Create task description": the agent prompt of the Product & Task step ([`DEFAULT_TASK_PROMPT`]).
    pub task_author_prompt: String,

    // --- Robo Dresser
    /// Catalogue (`urdf-robot-collection/`, `urdf-tool-collection/`).
    pub catalogue: String,
    pub dresser_port: u32,
    pub dresser_binary_path: String,
    pub dresser_web_root: String,
    /// "Suggest robots & tools": the agent prompt of the Robo Dresser step ([`DEFAULT_DRESSER_PROMPT`]).
    pub dresser_agent_prompt: String,

    // --- Robo Fitter
    /// Asset root (`urdf-dressed-robots/`, `urdf-fixture-collection/`, `floor-plans/`, …).
    pub asset_root: String,
    pub fitter_server_url: String,
    pub fitter_binary_path: String,
    pub fitter_server_host: String,
    pub fitter_server_port: u32,
    pub fitter_web_root: String,
    /// "Propose cell layout": the agent prompt of the Robo Fitter step ([`DEFAULT_FITTER_PROMPT`]).
    pub fitter_agent_prompt: String,
    /// "Check reachability (agent)": the agent prompt of the Robo Fitter's reach check
    /// ([`DEFAULT_REACH_PROMPT`]).
    pub reach_agent_prompt: String,
    /// The cell planner's Python, which runs the reach check's IK; empty =
    /// `/opt/factoryos-cell-planner/router/venv/bin/python`.
    pub cell_planner_python: String,
    /// The cell planner's sources (`trajgen`, `robotmodel/`); empty = `/opt/factoryos-cell-planner/router/src`.
    pub cell_planner_src: String,
    /// The reach check script; empty = `/opt/factoryos/current/deploy/planner-router/reach_check.py`.
    pub reach_check_script: String,

    // --- Create Cell Runner Scene
    /// The Fitter's output folder with scene manifests; empty = `<asset_root>/scenes`.
    pub fitted_robots_dir: String,
    /// The scene (a file in the scenes folder) the Robo Fitter opens first, when it exists;
    /// empty = a new scene.
    pub default_scene: String,

    // --- Cell Runner
    /// Folder containing the cells; empty = the active cell's parent folder.
    pub cells_dir: String,
    /// The active cell (set from the Cell Runner's "Active cell" picker).
    pub cell_dir: String,
    pub env_dir: String,
    /// Scene YAML, relative to `<env_dir>/scenes` (or the cell root for self-contained cells).
    pub scene: String,
    /// `low`, `visual` or `collision`.
    pub mesh_lod: String,
    pub trajectory_dir: String,
    pub trajectory_collision_check: bool,
    pub cell_runner_ws_port: u32,

    // --- Agentic Planning (multi_robots_agents)
    pub agentic_repo_dir: String,
    pub agentic_orchestrator_url: String,
    pub agentic_dashboard_url: String,
    pub agentic_rviz_url: String,
    /// `ompl` or `rrt`.
    pub agentic_trajectory_backend: String,
    pub agentic_model: String,
    pub agentic_region: String,
    pub agentic_rviz_enabled: bool,
    /// `cyclonedds` or `zenoh`.
    pub agentic_rmw: String,
    pub agentic_zenoh_router: String,
    /// "Plan Cell": the prompt sent to the planner agent; `{cell}`, `{cell_id}`, `{task}` and
    /// `{planner}` are filled in.
    pub agentic_plan_prompt: String,
    /// Start the planner agent with `kiro-cli acp --trust-all-tools`: it uses every tool (the
    /// planner's MCP tools, file reads and writes, shell commands) without asking first.
    pub agentic_trust_all_tools: bool,

    // --- EDH Integration (NICE DCV)
    pub credentials_prefix: String,
    pub edh_base_url: String,
    pub edh_username: String,
    pub use_cognito: bool,
    pub cognito_username: String,
    pub os_username: String,
    pub edh_start_timeout_seconds: u32,
    pub edh_poll_interval_seconds: u32,
}

/// The scene the Robo Fitter opens first in an empty FactoryOS: the starter's body_weld_4x8 (body_weld_3's four
/// KR240 R2900 with WELDING_GUN around the car body, 4 spot welds per side), which U3 plans in minutes and
/// re-plans from the catalogue's plan cache in seconds.
/// A scene name (`scenes/<name>/scene.yaml`); the config accepts a legacy `<name>.yaml` too.
pub const DEFAULT_SCENE: &str = "body_weld_4x8";
/// Earlier values of [`DEFAULT_SCENE`]: a saved config still at one follows the current default
/// ([`Config::upgrade_default_scene`]).
pub const PREVIOUS_DEFAULT_SCENES: &[&str] = &["body_weld_3.yaml", "body_weld_4x8.yaml"];

impl Config {
    /// A default scene still at an earlier default ([`PREVIOUS_DEFAULT_SCENES`]) becomes [`DEFAULT_SCENE`]: the whole
    /// config is saved, so every user has the old default stored and would never see a new one. Returns whether it
    /// changed.
    pub fn upgrade_default_scene(&mut self) -> bool {
        if PREVIOUS_DEFAULT_SCENES.contains(&self.default_scene.trim()) {
            self.default_scene = DEFAULT_SCENE.to_owned();
            return true;
        }
        false
    }

    /// A plan prompt still at an earlier default ([`PREVIOUS_PLAN_PROMPTS`]) becomes the current
    /// one, so a saved config follows the default (e.g. to the planner router) unless it was edited.
    /// Returns whether anything changed.
    pub fn upgrade_default_prompts(&mut self) -> bool {
        let mut changed = false;
        let saved = self.agentic_plan_prompt.trim();
        if PREVIOUS_PLAN_PROMPTS.iter().any(|old| old.trim() == saved) {
            self.agentic_plan_prompt = DEFAULT_PLAN_PROMPT.to_owned();
            changed = true;
        }
        let saved = self.fitter_agent_prompt.trim();
        if PREVIOUS_FITTER_PROMPTS.iter().any(|old| old.trim() == saved) {
            self.fitter_agent_prompt = DEFAULT_FITTER_PROMPT.to_owned();
            changed = true;
        }
        let saved = self.reach_agent_prompt.trim();
        if PREVIOUS_REACH_PROMPTS.iter().any(|old| old.trim() == saved) {
            self.reach_agent_prompt = DEFAULT_REACH_PROMPT.to_owned();
            changed = true;
        }
        changed
    }

    /// [`Config::cell_planner_python`], or its default.
    pub fn cell_planner_python_or_default(&self) -> &str {
        or(&self.cell_planner_python, "/opt/factoryos-cell-planner/router/venv/bin/python")
    }

    /// [`Config::cell_planner_src`], or its default.
    pub fn cell_planner_src_or_default(&self) -> &str {
        or(&self.cell_planner_src, "/opt/factoryos-cell-planner/router/src")
    }

    /// [`Config::reach_check_script`], or its default.
    pub fn reach_check_script_or_default(&self) -> &str {
        or(&self.reach_check_script, "/opt/factoryos/current/deploy/planner-router/reach_check.py")
    }
}

fn or<'a>(value: &'a str, default: &'a str) -> &'a str {
    if value.trim().is_empty() { default } else { value.trim() }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            products_dir: String::new(),
            task_author_prompt: DEFAULT_TASK_PROMPT.to_owned(),

            catalogue: String::new(),
            dresser_port: 3030,
            dresser_binary_path: String::new(),
            dresser_web_root: String::new(),
            dresser_agent_prompt: DEFAULT_DRESSER_PROMPT.to_owned(),

            asset_root: String::new(),
            fitter_server_url: "http://localhost:3040".to_owned(),
            fitter_binary_path: String::new(),
            fitter_server_host: "127.0.0.1".to_owned(),
            fitter_server_port: 3040,
            fitter_web_root: String::new(),
            fitter_agent_prompt: DEFAULT_FITTER_PROMPT.to_owned(),
            reach_agent_prompt: DEFAULT_REACH_PROMPT.to_owned(),
            cell_planner_python: String::new(),
            cell_planner_src: String::new(),
            reach_check_script: String::new(),

            fitted_robots_dir: String::new(),
            default_scene: DEFAULT_SCENE.to_owned(),

            cells_dir: String::new(),
            cell_dir: String::new(),
            env_dir: String::new(),
            scene: String::new(),
            mesh_lod: "low".to_owned(),
            trajectory_dir: "trajectories".to_owned(),
            trajectory_collision_check: true,
            cell_runner_ws_port: 9902,

            agentic_repo_dir: String::new(),
            agentic_orchestrator_url: "http://localhost:8889".to_owned(),
            agentic_dashboard_url: "http://localhost:8888".to_owned(),
            agentic_rviz_url: "http://localhost:6080/vnc.html".to_owned(),
            agentic_trajectory_backend: "ompl".to_owned(),
            agentic_model: String::new(),
            agentic_region: "us-east-1".to_owned(),
            agentic_rviz_enabled: false,
            agentic_rmw: "cyclonedds".to_owned(),
            agentic_zenoh_router: String::new(),
            agentic_plan_prompt: DEFAULT_PLAN_PROMPT.to_owned(),
            agentic_trust_all_tools: true,

            credentials_prefix: String::new(),
            edh_base_url: String::new(),
            edh_username: String::new(),
            use_cognito: false,
            cognito_username: String::new(),
            os_username: String::new(),
            edh_start_timeout_seconds: 600,
            edh_poll_interval_seconds: 5,
        }
    }
}

/// The user's roaming settings in their S3 subtree (`users/<owner>/…`, see [`Config`]).
pub const USER_CONFIG_KEY: &str = "factoryos.config.json";
/// In the roaming settings: the active cell, by its folder name in the cells folder.
pub const ACTIVE_CELL_FIELD: &str = "activeCell";

/// The [`Config`] fields (their JSON names) that roam with the user (S3 storage).
pub const USER_FIELDS: &[&str] = &[
    "taskAuthorPrompt",
    "dresserAgentPrompt",
    "fitterAgentPrompt",
    "reachAgentPrompt",
    "defaultScene",
    "meshLod",
    "trajectoryCollisionCheck",
    "agenticTrajectoryBackend",
    "agenticModel",
    "agenticPlanPrompt",
    "agenticTrustAllTools",
    "credentialsPrefix",
    "edhBaseUrl",
    "edhUsername",
    "useCognito",
    "cognitoUsername",
    "osUsername",
    "edhStartTimeoutSeconds",
    "edhPollIntervalSeconds",
];

/// The [`Config`] fields (their JSON names) of this machine only: paths, binaries, ports, endpoints.
pub const MACHINE_FIELDS: &[&str] = &[
    "productsDir",
    "catalogue",
    "dresserPort",
    "dresserBinaryPath",
    "dresserWebRoot",
    "assetRoot",
    "fitterServerUrl",
    "fitterBinaryPath",
    "fitterServerHost",
    "fitterServerPort",
    "fitterWebRoot",
    "cellPlannerPython",
    "cellPlannerSrc",
    "reachCheckScript",
    "fittedRobotsDir",
    "cellsDir",
    "cellDir",
    "envDir",
    "scene",
    "trajectoryDir",
    "cellRunnerWsPort",
    "agenticRepoDir",
    "agenticOrchestratorUrl",
    "agenticDashboardUrl",
    "agenticRvizUrl",
    "agenticRegion",
    "agenticRvizEnabled",
    "agenticRmw",
    "agenticZenohRouter",
];

/// `GET /api/v1/config`: the config plus where the backend keeps it.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConfigResponse {
    pub config: Config,
    /// The config file on the backend host; relative paths resolve against its folder.
    pub file: String,
    pub file_error: Option<String>,
}

/// `PUT /api/v1/secrets`. Write-only: the backend keeps these in memory and never returns them.
/// `None` leaves a password unchanged, an empty string clears it.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct Secrets {
    pub edh_password: Option<String>,
    pub cognito_password: Option<String>,
    pub os_password: Option<String>,
}

#[cfg(test)]
mod tests {

    #[test]
    fn a_default_scene_at_an_earlier_default_follows_the_current_one() {
        let mut config = Config { default_scene: "body_weld_3.yaml".into(), ..Config::default() };
        assert!(config.upgrade_default_scene());
        assert_eq!(config.default_scene, DEFAULT_SCENE);
        let mut chosen = Config { default_scene: "my_cell.yaml".into(), ..Config::default() };
        assert!(!chosen.upgrade_default_scene(), "a scene the user chose stays");
        assert_eq!(chosen.default_scene, "my_cell.yaml");
        assert_eq!(Config::default().default_scene, DEFAULT_SCENE);
    }

    use super::*;

    #[test]
    fn older_files_keep_their_values_and_get_new_defaults() {
        let config: Config = serde_json::from_str(r#"{"scene":"scene.yaml","cellDir":"/c","catalogue":"/cat"}"#).unwrap();
        assert_eq!(config.scene, "scene.yaml");
        assert_eq!(config.catalogue, "/cat");
        assert_eq!(config.mesh_lod, "low");
        assert_eq!(config.cell_runner_ws_port, 9902);
        assert!(config.trajectory_collision_check);
    }

    #[test]
    fn dropped_fields_of_older_files_are_ignored() {
        let json = r#"{"scene":"s.yaml","rerunWebViewerUrl":"http://127.0.0.1:9090","rerunGrpcPort":9876,
            "cellViewerWsPort":9902,"viewer3dUrl":"http://127.0.0.1:9090","viewer3dGrpcPort":9876}"#;
        let config: Config = serde_json::from_str(json).unwrap();
        assert_eq!(config.scene, "s.yaml");
        let saved = serde_json::to_string(&config).unwrap();
        assert!(!saved.contains("rerun") && !saved.contains("viewer3d") && !saved.contains("cellViewerWsPort"), "{saved}");
    }

    #[test]
    fn an_unedited_plan_prompt_follows_the_default() {
        // The test server's saved prompt (the default before the planner router).
        let mut config = Config { agentic_plan_prompt: PLAN_PROMPT_V1.to_owned(), ..Config::default() };
        assert!(config.upgrade_default_prompts());
        assert_eq!(config.agentic_plan_prompt, DEFAULT_PLAN_PROMPT);
        assert!(DEFAULT_PLAN_PROMPT.contains("{planner}") && DEFAULT_PLAN_PROMPT.contains("recommend_planner"));
        assert!(!config.upgrade_default_prompts(), "already current");
        let mut v2 = Config { agentic_plan_prompt: PLAN_PROMPT_V2.to_owned(), ..Config::default() };
        assert!(v2.upgrade_default_prompts());
        assert!(v2.agentic_plan_prompt.contains("task_program_from_products"));
        let mut edited = Config { agentic_plan_prompt: "Plan {cell} my way.".to_owned(), ..Config::default() };
        assert!(!edited.upgrade_default_prompts());
        assert_eq!(edited.agentic_plan_prompt, "Plan {cell} my way.");
    }

    #[test]
    fn the_layout_prompt_asks_for_yaml_and_a_reach_check() {
        let mut config = Config { fitter_agent_prompt: FITTER_PROMPT_V1.to_owned(), ..Config::default() };
        assert!(FITTER_PROMPT_V1.contains("_cell.json"));
        assert!(config.upgrade_default_prompts());
        assert_eq!(config.fitter_agent_prompt, DEFAULT_FITTER_PROMPT);
        assert!(DEFAULT_FITTER_PROMPT.contains("{product}_cell/scene.yaml") && DEFAULT_FITTER_PROMPT.contains("check_reachability"));
        let mut config = Config { fitter_agent_prompt: FITTER_PROMPT_V2.to_owned(), reach_agent_prompt: REACH_PROMPT_V1.to_owned(), ..Config::default() };
        assert!(config.upgrade_default_prompts(), "the flat layout's prompts follow the folder layout");
        assert_eq!((config.fitter_agent_prompt.as_str(), config.reach_agent_prompt.as_str()), (DEFAULT_FITTER_PROMPT, DEFAULT_REACH_PROMPT));
        for p in ["{scene_file}", "{fitted_robots_dir}", "{cell}", "check_reachability", "suggest_layout", "-fit\""] {
            assert!(DEFAULT_REACH_PROMPT.contains(p), "{p}");
        }
        let config: Config = serde_json::from_str("{}").unwrap();
        assert_eq!(config.reach_agent_prompt, DEFAULT_REACH_PROMPT);
        assert_eq!(config.cell_planner_python_or_default(), "/opt/factoryos-cell-planner/router/venv/bin/python");
        let config: Config = serde_json::from_str(r#"{"reachCheckScript":"/x/r.py"}"#).unwrap();
        assert_eq!(config.reach_check_script_or_default(), "/x/r.py");
    }
}
