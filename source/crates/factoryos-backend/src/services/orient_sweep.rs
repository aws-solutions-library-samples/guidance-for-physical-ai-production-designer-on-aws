//! Which way to mount the tool: `POST /api/v1/dresser/orient_sweep` dresses a robot with a tool
//! once per candidate mounting (into a scratch folder; the catalogue is untouched) and checks each
//! dressed robot in a scene against its welds with the Robo Fitter's reachability check, best
//! first. `apply` then dresses the best into the catalogue as the Robo Dresser's Dress does.

use std::path::Path;
use std::sync::atomic::AtomicBool;

use axum::extract::State;
use axum::http::StatusCode;
use axum::routing::post;
use axum::{Json, Router};
use factoryos_protocol::api;
use factoryos_protocol::dresser::{DressRequest, HULLS, OrientCandidate, OrientScore, OrientSweepRequest, OrientSweepResult};
use factoryos_protocol::fitter::{ReachReport, SceneFile, TaskSource};

use super::dresser::dress_into;
use super::reach::{CheckOptions, check_scene};
use crate::AppState;

const MAX_CANDIDATES: usize = 24;

pub fn router() -> Router<AppState> {
    Router::new().route(api::DRESSER_ORIENT_SWEEP, post(sweep))
}

type HttpError = (StatusCode, String);

async fn sweep(State(state): State<AppState>, Json(request): Json<OrientSweepRequest>) -> Result<Json<OrientSweepResult>, HttpError> {
    let (config, fitter, dresser) = (state.config.clone(), state.fitter.clone(), state.dresser.clone());
    tokio::task::spawn_blocking(move || {
        let scenes_dir = fitter.scenes_dir();
        let resolved = super::reach::http::resolve(&config, &scenes_dir, request.scene_file.as_deref(), request.cell.as_deref(), None)?;
        let root = dresser.root();
        let ctx = fitter.reach_context();
        let result = run(&root, &resolved.scene, &request, |scene, options| {
            check_scene(&ctx, scene, &resolved.scene_base, resolved.cell_root.as_deref(), options, |_| {}, &AtomicBool::new(false))
        })?;
        Ok(Json(result))
    })
    .await
    .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, format!("orientation sweep panicked: {err}")))?
}

/// The default candidates: the auto-orientation, spun a quarter turn at a time.
fn default_candidates() -> Vec<OrientCandidate> {
    [0.0, 90.0, 180.0, 270.0].into_iter().map(|spin_deg| OrientCandidate { orient: [0, 0, 0], spin_deg }).collect()
}

/// The robots of `scene` that carry the candidate: those named, else those dressed with `robot`, else all.
fn carriers(scene: &SceneFile, robot: &str, named: &[String]) -> Result<Vec<String>, HttpError> {
    if scene.robots.is_empty() {
        return Err((StatusCode::BAD_REQUEST, "the scene has no robots: place one first (scene_add_robot)".to_owned()));
    }
    if !named.is_empty() {
        let missing: Vec<&String> = named.iter().filter(|id| !scene.robots.iter().any(|r| &r.robot_id == *id)).collect();
        if !missing.is_empty() {
            return Err((StatusCode::BAD_REQUEST, format!("the scene has no robot {missing:?}")));
        }
        return Ok(named.to_vec());
    }
    let prefix = format!("{robot}__");
    let same: Vec<String> = scene
        .robots
        .iter()
        .filter(|r| Path::new(&r.dressed_urdf).file_name().and_then(|n| n.to_str()).is_some_and(|n| n.starts_with(&prefix)))
        .map(|r| r.robot_id.clone())
        .collect();
    Ok(if same.is_empty() { scene.robots.iter().map(|r| r.robot_id.clone()).collect() } else { same })
}

/// Score `report`: the welds reached once and twice, the unreached ones and why.
fn score(candidate: OrientCandidate, report: &ReachReport) -> OrientScore {
    let mut s = OrientScore { candidate, welds: report.welds.len(), ..OrientScore::default() };
    for weld in &report.welds {
        match weld.reached_by.len() {
            0 => s.unreached.push(weld.weld.id.clone()),
            1 => s.reached += 1,
            _ => {
                s.reached += 1;
                s.redundant += 1;
            }
        }
        if weld.reached_by.is_empty() {
            for attempt in weld.attempts.iter().filter(|a| !a.reachable) {
                for reason in &attempt.reasons {
                    let key = serde_json::to_value(reason).ok().and_then(|v| v.as_str().map(str::to_owned)).unwrap_or_else(|| format!("{reason:?}"));
                    *s.reasons.entry(key).or_default() += 1;
                }
            }
        }
    }
    s
}

/// The sweep, with `check` running the reachability check of a scene (the IK, or the domes).
pub(crate) fn run(
    root: &Path,
    scene: &SceneFile,
    request: &OrientSweepRequest,
    mut check: impl FnMut(&SceneFile, &CheckOptions) -> ReachReport,
) -> Result<OrientSweepResult, HttpError> {
    let bad = |m: String| (StatusCode::BAD_REQUEST, m);
    let valid = |n: &str| !n.is_empty() && n.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.')) && !n.contains("..");
    if !valid(&request.robot) || !valid(&request.tool) {
        return Err(bad("robot and tool: catalogue names (dresser_state lists them)".to_owned()));
    }
    let hull = if request.hull.trim().is_empty() { "tight".to_owned() } else { request.hull.trim().to_owned() };
    if !HULLS.contains(&hull.as_str()) {
        return Err(bad(format!("hull {hull}: one of {HULLS:?}")));
    }
    let candidates = if request.candidates.is_empty() { default_candidates() } else { request.candidates.clone() };
    if candidates.len() > MAX_CANDIDATES {
        return Err(bad(format!("{} candidates: at most {MAX_CANDIDATES} per sweep", candidates.len())));
    }
    let robots = carriers(scene, &request.robot, &request.robots)?;
    let scratch = std::env::temp_dir().join(format!("factoryos-orient-sweep-{}-{}", std::process::id(), unique()));
    let options = CheckOptions { source: TaskSource::Auto, mode: request.mode, estimate_only: request.estimate_only, ..CheckOptions::default() };
    let mut result = OrientSweepResult { robots: robots.clone(), ..OrientSweepResult::default() };
    let combo = format!("{}__{}", request.robot, request.tool);
    // Reads the robot's and the tool's files: no catalogue sync deletes meanwhile.
    let _catalogue = crate::storage::sync::in_use();
    for (i, candidate) in candidates.into_iter().enumerate() {
        let dress = DressRequest {
            robot: request.robot.clone(),
            tool: request.tool.clone(),
            hull: hull.clone(),
            orient: candidate.orient,
            spin_deg: candidate.spin_deg,
            attach_to: None,
        };
        let dir = scratch.join(format!("c{i:02}"));
        let outcome = dress_into(root, &dress, &dir, &combo);
        let Some(urdf) = outcome.output_rel.filter(|_| outcome.error.is_none()) else {
            result.scores.push(OrientScore { candidate, error: Some(outcome.error.unwrap_or_else(|| "dressing failed".to_owned())), ..OrientScore::default() });
            continue;
        };
        let mut trial = scene.clone();
        for robot in trial.robots.iter_mut().filter(|r| robots.contains(&r.robot_id)) {
            robot.dressed_urdf = urdf.clone();
        }
        let report = check(&trial, &options);
        if result.method.is_empty() {
            result.method = report.method.clone();
            result.notes = report.notes.iter().filter(|n| !n.contains(&scratch.to_string_lossy().to_string())).cloned().collect();
        }
        let mut s = score(candidate, &report);
        s.orient_used = (s.candidate.orient != [0, 0, 0]).then_some(s.candidate.orient);
        if report.welds.is_empty() {
            s.error = Some("the scene has no welds (place a product with welds, or link a cell with a task program)".to_owned());
        }
        result.scores.push(s);
    }
    std::fs::remove_dir_all(&scratch).ok();
    // Best first; a stable sort keeps the given order among equals.
    result.scores.sort_by(|a, b| (a.error.is_some(), b.reached, b.redundant).cmp(&(b.error.is_some(), a.reached, a.redundant)));
    if request.apply
        && let Some(best) = result.scores.first().filter(|s| s.error.is_none())
    {
        let dress = DressRequest {
            robot: request.robot.clone(),
            tool: request.tool.clone(),
            hull,
            orient: best.candidate.orient,
            spin_deg: best.candidate.spin_deg,
            attach_to: None,
        };
        let out = root.join("urdf-dressed-robots").join(&combo);
        let outcome = dress_into(root, &dress, &out, &combo);
        match outcome.error {
            None => result.applied = Some(format!("urdf-dressed-robots/{combo}/{combo}.urdf")),
            Some(err) => result.notes.push(format!("apply: {err}")),
        }
    }
    Ok(result)
}

fn unique() -> u128 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_nanos())
}

#[cfg(test)]
mod tests {
    use super::*;
    use factoryos_protocol::fitter::{ReachAttempt, ReachReason, RobotYaml, TaskWeld, WeldReach, WeldStatus};

    fn scene() -> SceneFile {
        let robot = |id: &str, urdf: &str| RobotYaml { robot_id: id.into(), dressed_urdf: urdf.into(), ..RobotYaml::default() };
        SceneFile {
            name: "s".into(),
            description: String::new(),
            scene_origin: None,
            robots: vec![robot("r1", "urdf-dressed-robots/kr240__gun/kr240__gun.urdf"), robot("r2", "urdf-dressed-robots/kr16__gun/kr16__gun.urdf")],
            fixtures: Vec::new(),
            floor_plan: None,
            extra: Default::default(),
        }
    }

    #[test]
    fn carriers_follow_the_robot() {
        assert_eq!(carriers(&scene(), "kr240", &[]).unwrap(), ["r1"]);
        assert_eq!(carriers(&scene(), "kr6", &[]).unwrap(), ["r1", "r2"]);
        assert!(carriers(&scene(), "kr240", &["r9".into()]).is_err());
    }

    #[test]
    fn scores_count_reach_and_reasons() {
        let weld = |id: &str, by: &[&str]| WeldReach {
            weld: TaskWeld { id: id.into(), ..TaskWeld::default() },
            attempts: vec![ReachAttempt { robot_id: "r1".into(), reachable: !by.is_empty(), reasons: if by.is_empty() { vec![ReachReason::OutOfReach] } else { vec![] }, ..ReachAttempt::default() }],
            reached_by: by.iter().map(|s| s.to_string()).collect(),
            status: WeldStatus::default(),
        };
        let report = ReachReport { welds: vec![weld("W01", &["r1"]), weld("W02", &["r1", "r2"]), weld("W03", &[])], ..ReachReport::default() };
        let s = score(OrientCandidate::default(), &report);
        assert_eq!((s.reached, s.redundant, s.welds), (2, 1, 3));
        assert_eq!(s.unreached, ["W03"]);
        assert_eq!(s.reasons.values().sum::<usize>(), 1);
    }
}
