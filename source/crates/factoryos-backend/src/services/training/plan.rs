//! A training job as factoryos-cell-planner takes it: the scenegen batch plan (`plan.json`), the
//! distribution as a `SceneGenConfig` dict (`scenegen_config.json`) and `job.json` with the
//! commands that generate the scenes and train on them (see [`factoryos_protocol::training`]).

use std::path::{Path, PathBuf};

use factoryos_protocol::training::{JOB_FORMAT, JobFile, Placement, TrainingJob, WeldFaces, YawMode};
use serde_json::{Value, json};

pub const PLAN_FILE: &str = "plan.json";
pub const CONFIG_FILE: &str = "scenegen_config.json";
pub const JOB_FILE: &str = "job.json";
/// The only robot model scenegen v2 builds scenes for (`scenegen/sample.py` `layout_dicts`).
const SCENEGEN_ROBOT: &str = "kr240r2900__sticktool";
/// Actor processes per c8g.48xlarge node in round 4.
const PROCS_PER_NODE: u32 = 188;

/// A dressed robot's name: its URDF's file stem.
pub fn model_name(urdf: &str) -> String {
    Path::new(urdf).file_stem().map_or_else(|| urdf.to_owned(), |s| s.to_string_lossy().into_owned())
}

/// scenegen's batch plan: `[{split, seeds, gen_version}]`, each naming the distribution beside
/// it (`config`, which today's `run_jobs` does not read yet: see [`notes`]).
pub fn batch_plan(job: &TrainingJob) -> Value {
    let s = &job.scenes;
    let spec = |split: &str, seeds: String| json!({"split": split, "seeds": seeds, "gen_version": 2, "config": CONFIG_FILE});
    let mut specs = vec![spec("train", format!("{}:{}", s.train_offset, s.train_offset + s.train))];
    if s.selection > 0 {
        specs.push(spec("selection", format!("0:{}", s.selection)));
    }
    if s.test > 0 {
        specs.push(spec("test", format!("0:{}", s.test)));
    }
    if s.ood {
        specs.push(json!({"split": "ood", "gen_version": 2, "config": CONFIG_FILE}));
    }
    Value::Array(specs)
}

/// The job's distribution in `SceneGenConfig.from_dict`'s keys (which drops the ones it does not
/// know: the `factoryos_*` extensions say what scenegen cannot draw yet).
pub fn scenegen_config(job: &TrainingJob) -> Value {
    let (r, p, w) = (&job.robots, &job.products, &job.welds);
    let weights: serde_json::Map<String, Value> = r.count_weights.iter().filter(|(_, w)| **w > 0.0).map(|(n, w)| (n.to_string(), json!(w))).collect();
    let models: Vec<String> = r.models.iter().map(|m| model_name(m)).collect();
    let half = |[lo, hi]: [f64; 2]| [lo / 2.0, hi / 2.0];
    json!({
        "gen_version": 2,
        "robot_count_weights": weights,
        "robot_model": models.first().cloned().unwrap_or_else(|| SCENEGEN_ROBOT.to_owned()),
        "base_z": r.base_z_m[0],
        "yaw_mode": match r.yaw { YawMode::Uniform => "uniform", YawMode::Facing => "facing" },
        "yaw_jitter_deg": r.yaw_jitter_deg,
        "ring_min_m": r.ring_m[0],
        "ring_max_m": r.ring_m[1],
        "base_min_spacing_m": r.min_spacing_m,
        "base_footprint_r_m": r.footprint_r_m,
        "base_hull_check": true,
        "n_cubes": p.count,
        "cube_half_xy_m": half(p.size_m),
        "cube_height_m": p.height_m,
        "cube_yaw_deg": p.yaw_deg,
        "cube_layout_radius_a": p.layout_radius_m[0],
        "cube_layout_radius_b": p.layout_radius_m[1],
        "cube_min_gap_m": p.min_gap_m,
        "welds_per_cube": w.per_product,
        "n_welds_min": w.total[0],
        "n_welds_max": w.total[1],
        "n_welds_per_robot_max": w.per_robot_max,
        "weld_edge_inset_m": w.edge_inset_m,
        "weld_min_spacing_m": w.min_spacing_m,
        "tilt_deg": w.tilt_deg,
        "electrode_rule": {"enabled": true, "radius_m": 0.05, "min_distance_m": 0.0, "score_tol_m": 0.005},
        "factoryos_robot_models": models,
        "factoryos_placement": match r.placement {
            Placement::RandomRing => "random_ring",
            Placement::Circle => "circle",
            Placement::CellJitter => "cell_jitter",
        },
        "factoryos_placement_jitter_m": r.jitter_m,
        "factoryos_base_z_m": r.base_z_m,
        "factoryos_cube_square": p.square,
        "factoryos_products_from_cell": p.from_cell,
        "factoryos_weld_faces": match w.faces { WeldFaces::Top => "top", WeldFaces::TopAndSides => "top_and_sides" },
    })
}

/// What scenegen / the trainer do not take from the job yet, so the plan's reader knows.
pub fn notes(job: &TrainingJob) -> Vec<String> {
    let mut notes = vec![
        "scenegen's batch run (`run_jobs`) does not read a spec's `config` yet: it draws from `config_for_split`. \
         Generating this distribution needs `make_scene(cfg=SceneGenConfig.from_dict(scenegen_config.json))`."
            .to_owned(),
    ];
    let models: Vec<String> = job.robots.models.iter().map(|m| model_name(m)).collect();
    if models.iter().any(|m| m != SCENEGEN_ROBOT) || models.len() > 1 {
        notes.push(format!("scenegen v2 builds scenes for {SCENEGEN_ROBOT} only; this job asks for {}", models.join(", ")));
    }
    if job.robots.placement != Placement::RandomRing {
        notes.push("scenegen places robots on the random ring only (factoryos_placement)".to_owned());
    }
    if job.robots.base_z_m[0] != job.robots.base_z_m[1] {
        notes.push("scenegen has one base height (base_z): the pedestal range is in factoryos_base_z_m".to_owned());
    }
    if job.products.square {
        notes.push("scenegen draws the cubes' x and y half sizes independently (factoryos_cube_square)".to_owned());
    }
    if job.products.from_cell {
        notes.push("scenegen draws the products; keeping the cell's is factoryos_products_from_cell".to_owned());
    }
    if job.welds.faces != WeldFaces::Top {
        notes.push("scenegen welds the cubes' tops only (factoryos_weld_faces)".to_owned());
    }
    if job.robots.max_count() > 4 {
        notes.push("the PPON4 network takes at most 4 robots (R_MAX)".to_owned());
    }
    notes
}

/// The commands that run the job in factoryos-cell-planner (paths from its root), `dir`: where
/// the job is saved.
pub fn commands(job: &TrainingJob, dir: &Path) -> Vec<String> {
    let t = &job.training;
    let name = &job.name;
    let scenes = format!("data/{name}/scenes");
    let rendezvous = format!("<s3 prefix>/{name}/rendezvous.json");
    let mut commands = vec![format!(
        "python -m scenegen batch --plan {} --out {scenes} --workers 190",
        dir.join(PLAN_FILE).display()
    )];
    let mut learner = format!("python -m rl.ppon4.train learner --run {name} --rendezvous {rendezvous}");
    if !t.init.is_empty() {
        learner += &format!(" --init {}", t.init);
    }
    learner += &format!(
        " --profile {} --alloc --horizon {} --gamma {} --lam {} --time-penalty {} --rw-weld {} --rw-success {} --rw-timeout {} \
         --rw-deadlock {} --pool {} --rotate {} --bc-steps {} --ppo-iters {} --batch {} --minibatch {} --epochs {} --lr {} \
         --clip {} --ent {}",
        t.profile,
        t.horizon_s,
        t.gamma,
        t.lam,
        t.rewards.time_penalty,
        t.rewards.weld,
        t.rewards.success,
        t.rewards.timeout,
        t.rewards.deadlock,
        t.pool,
        t.rotate,
        t.bc_steps,
        t.ppo_iters,
        t.batch,
        t.minibatch,
        t.epochs,
        t.lr,
        t.clip,
        t.ent,
    );
    commands.push(learner);
    let nodes = t.actors.div_ceil(PROCS_PER_NODE).max(1);
    let procs = t.actors.div_ceil(nodes);
    let args = json!({"scenes": format!("{scenes}/train")});
    commands.push(format!(
        "python -m rl.dist.node --rendezvous {rendezvous} --procs {procs} --entry rl.ppon4.train:actor_main --args '{args}'  # on each of {nodes} node(s)"
    ));
    commands
}

/// Write the job's three files into `dir` (made if missing); returns them.
pub fn save(job: &TrainingJob, dir: &Path) -> anyhow::Result<Vec<PathBuf>> {
    std::fs::create_dir_all(dir)?;
    let plan = dir.join(PLAN_FILE);
    let config = dir.join(CONFIG_FILE);
    let job_path = dir.join(JOB_FILE);
    std::fs::write(&plan, serde_json::to_string_pretty(&batch_plan(job))? + "\n")?;
    std::fs::write(&config, serde_json::to_string_pretty(&scenegen_config(job))? + "\n")?;
    let file = JobFile {
        format: JOB_FORMAT.to_owned(),
        job: job.clone(),
        plan: PLAN_FILE.to_owned(),
        scenegen_config: CONFIG_FILE.to_owned(),
        commands: commands(job, dir),
        notes: notes(job),
    };
    std::fs::write(&job_path, serde_json::to_string_pretty(&file)? + "\n")?;
    Ok(vec![plan, config, job_path])
}

pub fn load(dir: &Path) -> anyhow::Result<JobFile> {
    let text = std::fs::read_to_string(dir.join(JOB_FILE))?;
    let file: JobFile = serde_json::from_str(&text)?;
    anyhow::ensure!(file.format == JOB_FORMAT, "{} is {}, not {JOB_FORMAT}", dir.join(JOB_FILE).display(), file.format);
    Ok(file)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_batch_plan_is_scenegens() {
        let mut job = TrainingJob::default();
        job.scenes.train = 500;
        job.scenes.train_offset = 1000;
        job.scenes.test = 0;
        let plan = batch_plan(&job);
        assert_eq!(plan[0]["split"], "train");
        assert_eq!(plan[0]["seeds"], "1000:1500");
        assert_eq!(plan[1]["split"], "selection");
        assert_eq!(plan.as_array().unwrap().len(), 2);
    }

    #[test]
    fn the_config_uses_scenegens_keys() {
        let mut job = TrainingJob::default();
        job.robots.models = vec!["../urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf".into()];
        let config = scenegen_config(&job);
        assert_eq!(config["robot_model"], "kr240r2900__sticktool");
        assert_eq!(config["robot_count_weights"]["3"], 0.30);
        assert_eq!(config["cube_half_xy_m"], json!([0.15, 0.5]));
        assert_eq!(config["n_cubes"], json!([1, 4]));
        // Round 4's distribution needs no extension.
        assert_eq!(notes(&job).len(), 1);
    }

    #[test]
    fn saved_jobs_load_back() {
        let dir = std::env::temp_dir().join(format!("fos-training-{}", std::process::id()));
        let job = TrainingJob { name: "t1".into(), ..Default::default() };
        let files = save(&job, &dir).unwrap();
        assert_eq!(files.len(), 3);
        let file = load(&dir).unwrap();
        assert_eq!(file.job, job);
        assert!(file.commands[0].contains("scenegen batch --plan"));
        assert!(file.commands[1].contains("--ppo-iters 150"));
        std::fs::remove_dir_all(&dir).ok();
    }
}
