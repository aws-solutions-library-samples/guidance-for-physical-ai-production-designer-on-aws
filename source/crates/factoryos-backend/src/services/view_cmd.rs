//! The chat agent's handles on what the user sees: `POST /api/v1/view/command` moves the viewers'
//! 3D view ([`factoryos_protocol::view`]; a robot or weld of the Robo Fitter's scene resolved to a
//! point here, published on `factoryos/view/command`), and `GET /api/v1/cell/snapshot` answers the
//! simulated cell's state and its behaviour tree playback in one call.

use axum::extract::State;
use axum::http::StatusCode;
use axum::routing::{get, post};
use axum::{Json, Router};
use factoryos_protocol::cell::CellCommand;
use factoryos_protocol::fitter::FitterState;
use factoryos_protocol::view::{ViewCommand, ViewCommandSent};
use factoryos_protocol::{api, keys};
use serde_json::{Value, json};

use crate::AppState;

pub fn router() -> Router<AppState> {
    Router::new().route(api::VIEW_COMMAND, post(command)).route(api::CELL_SNAPSHOT, get(snapshot))
}

/// A robot's look-at point: its base raised to about its shoulder (on its pedestal).
const SHOULDER_M: f64 = 0.8;
const PAGES: [&str; 5] = ["dresser", "fitter", "product", "training", "live"];

/// `command` with a robot or weld turned into its target (scene frame of the Robo Fitter).
fn resolve(mut command: ViewCommand, fitter: &FitterState) -> Result<ViewCommandSent, String> {
    let mut note = None;
    if let Some(page) = command.page.as_deref().map(str::trim).filter(|p| !p.is_empty()) {
        if !PAGES.contains(&page) {
            return Err(format!("unknown page {page}: one of {PAGES:?}"));
        }
    }
    if let Some(robot) = command.robot.as_deref().map(str::trim).filter(|r| !r.is_empty()) {
        let r = fitter.scene.robots.iter().find(|r| r.robot_id == robot).ok_or_else(|| {
            let ids: Vec<&str> = fitter.scene.robots.iter().map(|r| r.robot_id.as_str()).collect();
            format!("the Robo Fitter's scene has no robot {robot} (robots: {ids:?})")
        })?;
        let lift = r.pedestal.as_ref().map_or(0.0, |p| p.height_mm / 1000.0);
        command.target = Some([r.position.x, r.position.y, r.position.z + lift + SHOULDER_M]);
        note = Some(format!("robot {robot} in {}", fitter.scene_file_name));
    } else if let Some(weld) = command.weld.as_deref().map(str::trim).filter(|w| !w.is_empty()) {
        let w = fitter
            .task
            .iter()
            .find(|w| w.id == weld || w.label == weld || w.id.rsplit('/').next() == Some(weld))
            .ok_or_else(|| format!("the Robo Fitter's task has no weld {weld} ({} welds shown)", fitter.task.len()))?;
        command.target = Some(w.position);
        note = Some(format!("weld {} in {}", w.label, fitter.scene_file_name));
    }
    if (command.robot.is_some() || command.weld.is_some()) && command.page.is_none() {
        command.page = Some("fitter".to_owned());
    }
    // Checked here so that a mistake is the tool's error, not a silent no-op in the viewer.
    factoryos_protocol::view::resolve_eye(&command, [0.0, -3.0, 2.0], [0.0; 3], [0.0; 3])?;
    Ok(ViewCommandSent { command, note })
}

async fn command(State(state): State<AppState>, Json(command): Json<ViewCommand>) -> Result<Json<ViewCommandSent>, (StatusCode, String)> {
    let sent = resolve(command, &state.fitter.state()).map_err(|err| (StatusCode::BAD_REQUEST, err))?;
    state.bus.put_json(keys::VIEW_COMMAND, &sent.command);
    Ok(Json(sent))
}

/// The simulated cell's `state` and `trajectory_state` (null when the cell-runner is not running).
async fn snapshot(State(state): State<AppState>) -> Json<Value> {
    let cell = state.cell.command(CellCommand::new("state")).await;
    let playback = state.cell.command(CellCommand::new("trajectory_state")).await;
    let part = |r: factoryos_protocol::cell::CellResponse| if r.ok { r.result.unwrap_or(Value::Null) } else { json!({"error": r.error}) };
    Json(json!({"cell": part(cell), "behaviour_tree": part(playback)}))
}

#[cfg(test)]
mod tests {
    use super::*;
    use factoryos_protocol::fitter::{RobotYaml, TaskWeld, Xyz};

    #[test]
    fn robots_and_welds_become_targets() {
        let mut fitter = FitterState::default();
        fitter.scene.robots.push(RobotYaml { robot_id: "r1".into(), position: Xyz { x: 1.0, y: 2.0, z: 0.0 }, ..RobotYaml::default() });
        fitter.task.push(TaskWeld { id: "cube/W03".into(), label: "W03".into(), position: [0.1, 0.2, 0.3], ..TaskWeld::default() });
        let focus = |robot: Option<&str>, weld: Option<&str>| ViewCommand {
            action: "focus".into(),
            robot: robot.map(str::to_owned),
            weld: weld.map(str::to_owned),
            ..ViewCommand::default()
        };
        let sent = resolve(focus(Some("r1"), None), &fitter).unwrap();
        assert_eq!(sent.command.target, Some([1.0, 2.0, SHOULDER_M]));
        assert_eq!(sent.command.page.as_deref(), Some("fitter"));
        assert_eq!(resolve(focus(None, Some("W03")), &fitter).unwrap().command.target, Some([0.1, 0.2, 0.3]));
        assert!(resolve(focus(Some("r9"), None), &fitter).is_err());
        assert!(resolve(ViewCommand { action: "spin".into(), ..ViewCommand::default() }, &fitter).is_err());
    }
}
