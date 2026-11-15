//! Build training job: the distribution of cells a learned planner policy is trained on (how many
//! robots and where, the products' sizes, the welds), with the training run's settings, defined
//! around the active cell and saved as a plan for factoryos-cell-planner.
//!
//! The planner's round 4 policy (PPON4) trains on scenes `scenegen` generates beforehand; the
//! distribution is its `SceneGenConfig` (`scenegen/config.py`), the scenes to make a batch plan
//! (`python -m scenegen batch --plan plan.json`: a list of `{split, seeds}`), and the training
//! itself the `rl.ppon4.train learner` flags. A saved job is a folder
//! `<cell>/training_jobs/<name>/` with `plan.json` (the batch plan), `scenegen_config.json` (the
//! distribution, `SceneGenConfig.from_dict`'s format) and `job.json` (this [`TrainingJob`] and the
//! commands that run it).
//!
//! The backend samples cells from the job (geometry only, no IK: the planner's feasibility filter
//! rejects some of them) and describes them in 3D on `factoryos/scene/training`, the cell's own robot
//! models at the sampled bases. Actions are HTTP (`/api/v1/training/…`) and answer with
//! [`TrainingState`].

use std::collections::BTreeMap;

/// A range `[min, max]`, both included.
pub type Range = [f64; 2];

/// What a job is saved as (`job.json`'s `format`).
pub const JOB_FORMAT: &str = "factoryos_training_job_v1";
/// The folder of a cell the jobs are saved in.
pub const JOBS_DIR: &str = "training_jobs";

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct TrainingJob {
    /// The folder name under [`JOBS_DIR`] (and the training run's name).
    pub name: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub description: String,
    /// The cell the job was made from (its robot models).
    pub cell: String,
    pub robots: RobotSpec,
    pub products: ProductSpec,
    pub welds: WeldSpec,
    pub scenes: SceneSpec,
    pub training: TrainingSpec,
}

impl Default for TrainingJob {
    fn default() -> Self {
        Self {
            name: "training-job".to_owned(),
            description: String::new(),
            cell: String::new(),
            robots: RobotSpec::default(),
            products: ProductSpec::default(),
            welds: WeldSpec::default(),
            scenes: SceneSpec::default(),
            training: TrainingSpec::default(),
        }
    }
}

/// How the robots' bases are placed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Placement {
    /// scenegen's: each robot at a random distance from a random product's footprint
    /// (`ring_m`), clear of the other products and robots (round 4).
    #[default]
    RandomRing,
    /// Evenly on a circle around the products, `ring_m` beyond their outline, the circle turned
    /// at random, each base moved by up to `jitter_m`.
    Circle,
    /// The loaded cell's bases (robot `i` at the cell's robot `i mod n`), each moved by up to
    /// `jitter_m` and turned by up to `yaw_jitter_deg`.
    CellJitter,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum YawMode {
    /// Any heading, U(0, 360).
    #[default]
    Uniform,
    /// Towards the product it is placed by, ± `yaw_jitter_deg`.
    Facing,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct RobotSpec {
    /// Dressed robot URDFs (as the cell's scene names them) the robots are drawn from, robot `i`
    /// being model `i mod n`.
    pub models: Vec<String>,
    /// Robots per cell → weight (drawn once per scene).
    pub count_weights: BTreeMap<u32, f64>,
    pub placement: Placement,
    /// Base distance from the product footprint (random ring) or the centre (circle), metres.
    pub ring_m: Range,
    /// Bases at least this far apart, metres.
    pub min_spacing_m: f64,
    /// The base's footprint radius (keeps it off the products), metres.
    pub footprint_r_m: f64,
    pub yaw: YawMode,
    pub yaw_jitter_deg: f64,
    /// Per-base displacement for `Circle` / `CellJitter`, metres.
    pub jitter_m: f64,
    /// Pedestal height, metres (`[0, 0]`: on the floor).
    pub base_z_m: Range,
}

impl Default for RobotSpec {
    fn default() -> Self {
        Self {
            models: Vec::new(),
            count_weights: BTreeMap::from([(1, 0.15), (2, 0.25), (3, 0.30), (4, 0.30)]),
            placement: Placement::RandomRing,
            ring_m: [0.8, 2.2],
            min_spacing_m: 1.5,
            footprint_r_m: 0.55,
            yaw: YawMode::Uniform,
            yaw_jitter_deg: 60.0,
            jitter_m: 0.2,
            base_z_m: [0.0, 0.0],
        }
    }
}

impl RobotSpec {
    /// The robot counts with a positive weight.
    pub fn counts(&self) -> Vec<u32> {
        self.count_weights.iter().filter(|(_, w)| **w > 0.0).map(|(n, _)| *n).collect()
    }

    pub fn max_count(&self) -> u32 {
        self.counts().into_iter().max().unwrap_or(1)
    }
}

/// The products: boxes standing on the floor, both workpiece and obstacle (scenegen's cubes).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ProductSpec {
    /// The loaded cell's products as they are (sizes and places); the welds are still drawn on
    /// them. The other fields are then unused.
    pub from_cell: bool,
    /// Products per cell.
    pub count: [u32; 2],
    /// Square footprint (a cube's): one side length for x and y.
    pub square: bool,
    /// Footprint side length, metres.
    pub size_m: Range,
    pub height_m: Range,
    pub yaw_deg: Range,
    /// Centres in a disk of radius `layout_radius_m[0] + layout_radius_m[1] × count`.
    pub layout_radius_m: [f64; 2],
    /// Products at least this far apart, metres.
    pub min_gap_m: f64,
}

impl Default for ProductSpec {
    fn default() -> Self {
        Self {
            from_cell: false,
            count: [1, 4],
            square: false,
            size_m: [0.3, 1.0],
            height_m: [0.5, 1.3],
            yaw_deg: [0.0, 90.0],
            layout_radius_m: [0.6, 0.4],
            min_gap_m: 0.25,
        }
    }
}

/// Which faces of a product carry welds.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WeldFaces {
    /// The top (scenegen's).
    #[default]
    Top,
    /// The top and the four sides, by area.
    TopAndSides,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct WeldSpec {
    /// Welds per cell, before the robot-count limits: `[max(min, robots + 1), min(max, per_robot × robots)]`.
    pub total: [u32; 2],
    pub per_robot_max: u32,
    /// Welds per product (then trimmed or padded to the total).
    pub per_product: [u32; 2],
    pub faces: WeldFaces,
    /// Kept this far from a face's edges, metres.
    pub edge_inset_m: f64,
    pub min_spacing_m: f64,
    /// The approach's tilt from the face normal, degrees (azimuth U(0, 360)).
    pub tilt_deg: Range,
}

impl Default for WeldSpec {
    fn default() -> Self {
        Self {
            total: [2, 24],
            per_robot_max: 6,
            per_product: [1, 6],
            faces: WeldFaces::Top,
            edge_inset_m: 0.05,
            min_spacing_m: 0.06,
            tilt_deg: [0.0, 20.0],
        }
    }
}

/// The scenes to generate (scenegen splits; train seeds are drawn from, the others evaluate).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SceneSpec {
    pub train: u32,
    /// The first train seed index (another job's seeds after these).
    pub train_offset: u32,
    pub selection: u32,
    pub test: u32,
    /// scenegen's out-of-distribution families (many products, tight, tilt, combo).
    pub ood: bool,
    /// The preview's seed.
    pub preview_seed: u64,
}

impl Default for SceneSpec {
    fn default() -> Self {
        Self { train: 20_000, train_offset: 0, selection: 100, test: 200, ood: false, preview_seed: 0 }
    }
}

/// The training run: round 4's recipe (BC on search labels, then PPO fine-tuning).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct TrainingSpec {
    /// `ppon4`.
    pub algorithm: String,
    /// The checkpoint to start from (empty: from scratch).
    pub init: String,
    /// Timing profile (`umarell`, `full`).
    pub profile: String,
    pub bc_steps: u32,
    pub ppo_iters: u32,
    pub batch: u32,
    pub minibatch: u32,
    pub epochs: u32,
    pub lr: f64,
    pub clip: f64,
    pub ent: f64,
    pub gamma: f64,
    pub lam: f64,
    /// Planning horizon, seconds.
    pub horizon_s: f64,
    /// Scenes an actor keeps, and episodes before it swaps one.
    pub pool: u32,
    pub rotate: u32,
    pub rewards: Rewards,
    /// Actor processes (each runs episodes on its scene pool).
    pub actors: u32,
}

impl Default for TrainingSpec {
    fn default() -> Self {
        Self {
            algorithm: "ppon4".to_owned(),
            init: "runs/ppon4_p2/selected_000050.pt".to_owned(),
            profile: "umarell".to_owned(),
            bc_steps: 0,
            ppo_iters: 150,
            batch: 49_152,
            minibatch: 4096,
            epochs: 3,
            lr: 1e-4,
            clip: 0.2,
            ent: 1e-4,
            gamma: 0.995,
            lam: 0.95,
            horizon_s: 60.0,
            pool: 4,
            rotate: 8,
            rewards: Rewards::default(),
            actors: 1500,
        }
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Rewards {
    pub time_penalty: f64,
    pub weld: f64,
    pub success: f64,
    pub timeout: f64,
    pub deadlock: f64,
}

impl Default for Rewards {
    fn default() -> Self {
        Self { time_penalty: 1.0, weld: 2.0, success: 10.0, timeout: -50.0, deadlock: -50.0 }
    }
}

// --- What the backend says ---

/// The active cell as the job sees it (scene frame, metres).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct TrainingCell {
    pub name: String,
    pub robots: Vec<CellRobot>,
    pub products: Vec<SampledProduct>,
    pub welds: usize,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct CellRobot {
    pub id: String,
    /// As the scene names it (a [`RobotSpec::models`] entry).
    pub model: String,
    pub position: [f64; 3],
    pub yaw_deg: f64,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SampledRobot {
    pub model: String,
    /// The base on the floor (or pedestal top), scene frame.
    pub position: [f64; 3],
    pub yaw_deg: f64,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SampledProduct {
    pub name: String,
    /// The box centre.
    pub center: [f64; 3],
    pub half: [f64; 3],
    pub yaw_deg: f64,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SampledWeld {
    pub position: [f64; 3],
    /// Unit, from the tool towards the weld.
    pub approach: [f64; 3],
    /// The product's index.
    pub product: usize,
}

/// One cell drawn from a job (or the loaded cell itself).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SampledCell {
    /// `None`: the loaded cell.
    pub seed: Option<u64>,
    pub robots: Vec<SampledRobot>,
    pub products: Vec<SampledProduct>,
    pub welds: Vec<SampledWeld>,
    /// What could not be placed as asked.
    pub notes: Vec<String>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct TrainingState {
    /// `None`: no active cell.
    pub cell: Option<TrainingCell>,
    /// A job made from the cell (round 4's distribution, the cell's robot models).
    pub suggested: Option<TrainingJob>,
    /// Shown in the 3D view.
    pub shown: Option<SampledCell>,
    /// Jobs saved in the cell (folder names).
    pub saved_jobs: Vec<String>,
    /// The folder last saved to, and what the planner does not take from that job yet.
    pub saved_to: Option<String>,
    pub saved_notes: Vec<String>,
    pub status: String,
    pub error: Option<String>,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PreviewRequest {
    pub job: TrainingJob,
    /// `None`: show the loaded cell itself.
    pub seed: Option<u64>,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SaveRequest {
    pub job: TrainingJob,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LoadJobRequest {
    pub name: String,
}

/// A saved job's `job.json`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct JobFile {
    pub format: String,
    pub job: TrainingJob,
    /// The files beside it.
    pub plan: String,
    pub scenegen_config: String,
    /// What runs it, in factoryos-cell-planner.
    pub commands: Vec<String>,
    pub notes: Vec<String>,
}

/// A valid job name (a folder name).
pub fn valid_job_name(name: &str) -> bool {
    !name.is_empty()
        && name.len() <= 64
        && !name.starts_with('.')
        && name.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '-' | '_' | '.'))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn jobs_round_trip_and_fill_missing_fields() {
        let job = TrainingJob::default();
        let text = serde_json::to_string(&job).unwrap();
        assert_eq!(serde_json::from_str::<TrainingJob>(&text).unwrap(), job);
        let partial: TrainingJob = serde_json::from_str(r#"{"name": "a", "robots": {"count_weights": {"2": 1.0}}}"#).unwrap();
        assert_eq!(partial.robots.counts(), vec![2]);
        assert_eq!(partial.welds, WeldSpec::default());
    }

    #[test]
    fn job_names_are_folder_names() {
        assert!(valid_job_name("jam2-4robots_v1.2"));
        assert!(!valid_job_name("../x"));
        assert!(!valid_job_name(".hidden"));
        assert!(!valid_job_name(""));
    }
}
