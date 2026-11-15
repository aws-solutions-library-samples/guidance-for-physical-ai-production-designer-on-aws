//! Moving the viewers' 3D view from outside (the chat agent's view_* tools): `POST
//! /api/v1/view/command` with a [`ViewCommand`]; the backend resolves a robot or weld to a point and
//! publishes it on [`crate::keys::VIEW_COMMAND`]; every viewer applies it to the 3D view of the
//! page it names (or the page it shows), even over a camera the user moved.
//!
//! Points in the scene frame of that page (metres, z up).

/// What to do with the camera.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ViewCommand {
    /// `set` (eye + target), `focus` (look at target / robot / weld from `distance`), `preset`
    /// (`iso`, `top`, `front`, `back`, `left`, `right` around the target or the scene's centre),
    /// `orbit` (turn by `yaw_deg` / `pitch_deg` about the target), `zoom` (distance × `factor`),
    /// `fit` (frame the whole scene again).
    pub action: String,
    /// `dresser`, `fitter`, `product`, `training` or `live`; default: the page the viewer shows.
    pub page: Option<String>,
    pub eye: Option<[f64; 3]>,
    pub target: Option<[f64; 3]>,
    /// For `focus`: a robot of the Robo Fitter's scene (its base, raised to its shoulder).
    pub robot: Option<String>,
    /// For `focus`: a weld of the Robo Fitter's task (its id or label, e.g. `cube/W03`, `W03`).
    pub weld: Option<String>,
    pub preset: Option<String>,
    /// Eye to target, metres (`focus`, `preset`; default: the current distance).
    pub distance: Option<f64>,
    pub factor: Option<f64>,
    pub yaw_deg: Option<f64>,
    pub pitch_deg: Option<f64>,
}

/// The answer of `POST /api/v1/view/command`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ViewCommandSent {
    /// The command as published (a robot or weld resolved into `target`).
    pub command: ViewCommand,
    pub note: Option<String>,
}

/// The view direction (from the eye to the target) of a preset, `None` for an unknown one.
pub fn preset_direction(preset: &str) -> Option<[f64; 3]> {
    let d: [f64; 3] = match preset.trim().to_ascii_lowercase().as_str() {
        "iso" | "isometric" | "default" => [-0.25, -0.75, -0.25],
        "top" => [0.0, 0.0001, -1.0],
        "front" => [0.0, 1.0, -0.15],
        "back" => [0.0, -1.0, -0.15],
        "left" => [1.0, 0.0, -0.15],
        "right" => [-1.0, 0.0, -0.15],
        _ => return None,
    };
    let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    Some(d.map(|c| c / n))
}

/// The eye that results from `command`, given the view's current eye and target and the scene's
/// centre; `None` for `fit` (the viewer frames the scene itself), `Err` for a command it can't do.
pub fn resolve_eye(command: &ViewCommand, eye: [f64; 3], target: [f64; 3], center: [f64; 3]) -> Result<Option<([f64; 3], [f64; 3])>, String> {
    let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let len = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    let along = |t: [f64; 3], dir: [f64; 3], d: f64| [t[0] - dir[0] * d, t[1] - dir[1] * d, t[2] - dir[2] * d];
    let current = len(sub(target, eye)).max(0.3);
    let forward = {
        let f = sub(target, eye);
        let n = len(f);
        if n < 1e-9 { [0.0, 1.0, 0.0] } else { f.map(|c| c / n) }
    };
    Ok(Some(match command.action.trim() {
        "fit" => return Ok(None),
        "set" => {
            let (e, t) = (command.eye.ok_or("set needs eye")?, command.target.unwrap_or(target));
            if len(sub(t, e)) < 1e-6 {
                return Err("eye and target are the same point".to_owned());
            }
            (e, t)
        }
        "focus" => {
            let t = command.target.ok_or("focus needs target, robot or weld")?;
            let d = command.distance.unwrap_or(current.min(3.0)).max(0.1);
            (along(t, forward, d), t)
        }
        "preset" => {
            let name = command.preset.as_deref().unwrap_or("iso");
            let dir = preset_direction(name).ok_or_else(|| format!("unknown preset {name}: iso, top, front, back, left, right"))?;
            let t = command.target.unwrap_or(center);
            (along(t, dir, command.distance.unwrap_or(current).max(0.1)), t)
        }
        "zoom" => {
            let f = command.factor.filter(|f| *f > 0.01).ok_or("zoom needs factor > 0 (0.5: closer, 2: farther)")?;
            (along(target, forward, (current * f).max(0.05)), target)
        }
        "orbit" => {
            let (yaw, pitch) = (command.yaw_deg.unwrap_or(0.0).to_radians(), command.pitch_deg.unwrap_or(0.0).to_radians());
            let back = sub(eye, target);
            let r = len(back).max(0.05);
            let az = back[1].atan2(back[0]) + yaw;
            let el = ((back[2] / r).clamp(-1.0, 1.0).asin() + pitch).clamp(-1.5, 1.5);
            ([target[0] + r * el.cos() * az.cos(), target[1] + r * el.cos() * az.sin(), target[2] + r * el.sin()], target)
        }
        other => return Err(format!("unknown action {other:?}: set, focus, preset, orbit, zoom, fit")),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd(action: &str) -> ViewCommand {
        ViewCommand { action: action.into(), ..ViewCommand::default() }
    }

    #[test]
    fn eyes() {
        let (eye, target, center) = ([0.0, -4.0, 0.0], [0.0; 3], [1.0, 1.0, 0.0]);
        assert_eq!(resolve_eye(&cmd("fit"), eye, target, center), Ok(None));
        let zoom = ViewCommand { factor: Some(0.5), ..cmd("zoom") };
        assert_eq!(resolve_eye(&zoom, eye, target, center).unwrap(), Some(([0.0, -2.0, 0.0], target)));
        let focus = ViewCommand { target: Some([1.0, 0.0, 0.0]), distance: Some(1.0), ..cmd("focus") };
        assert_eq!(resolve_eye(&focus, eye, target, center).unwrap(), Some(([1.0, -1.0, 0.0], [1.0, 0.0, 0.0])));
        let top = ViewCommand { preset: Some("top".into()), distance: Some(2.0), ..cmd("preset") };
        let (e, t) = resolve_eye(&top, eye, target, center).unwrap().unwrap();
        assert_eq!(t, center);
        assert!((e[2] - 2.0).abs() < 1e-3, "{e:?}");
        let orbit = ViewCommand { yaw_deg: Some(90.0), ..cmd("orbit") };
        let (e, _) = resolve_eye(&orbit, eye, target, center).unwrap().unwrap();
        assert!((e[0] - 4.0).abs() < 1e-9 && e[1].abs() < 1e-9, "{e:?}");
        assert!(resolve_eye(&cmd("spin"), eye, target, center).is_err());
        assert!(resolve_eye(&cmd("focus"), eye, target, center).is_err());
    }
}
