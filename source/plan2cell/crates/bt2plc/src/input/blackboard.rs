//! Blackboard entries and trajectory parsing.
//!
//! A trajectory is a list of waypoints `{positions: [6 rad], time_s, phase}` (umarell
//! `ompl_planner`), relative to the op start, with TOPP-RA timing baked in.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use serde_json::Value;

use super::{NormNode, PortValue};
use crate::error::{DiagCode, Diagnostics, Error, Result};

#[derive(Clone, Debug, Default)]
pub struct Blackboard(pub BTreeMap<String, Value>);

impl Blackboard {
    pub fn get(&self, key: &str) -> Option<&Value> {
        self.0.get(key)
    }
}

/// A `depends_on_ops` entry.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
pub struct OpRef {
    pub robot: String,
    pub operation_id: String,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Phase {
    Transit,
    Approach,
    Other(String),
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Waypoint {
    pub t_s: f64,
    pub q_rad: [f64; 6],
    pub phase: Phase,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Trajectory {
    pub key: Option<String>,
    pub waypoints: Vec<Waypoint>,
}

impl Trajectory {
    pub fn duration_s(&self) -> f64 {
        self.waypoints.last().map(|w| w.t_s).unwrap_or(0.0)
    }

    pub fn first(&self) -> &[f64; 6] {
        &self.waypoints[0].q_rad
    }

    pub fn last(&self) -> &[f64; 6] {
        &self.waypoints[self.waypoints.len() - 1].q_rad
    }
}

#[derive(Deserialize)]
struct RawWaypoint {
    #[serde(default)]
    positions: Vec<f64>,
    #[serde(default)]
    time_s: f64,
    #[serde(default)]
    phase: Option<String>,
}

/// Resolve the `trajectory` port of `node` (`"{key}"`, or an inline list).
pub fn trajectory(
    node: &NormNode,
    bb: &Blackboard,
    d: &mut Diagnostics,
) -> Result<Option<Trajectory>> {
    let Some(v) = node.port("trajectory") else {
        return Ok(None);
    };
    let path = Some(node.path.0.as_str());
    let (key, json) = match v.pointer() {
        Some(k) => {
            let j = bb.get(k).ok_or_else(|| {
                Error::plan(
                    DiagCode::E008,
                    path,
                    format!("trajectory '{{{k}}}' is not on the blackboard"),
                )
            })?;
            (Some(k.to_string()), j.clone())
        }
        None => match v {
            PortValue::Json(j) => (None, j.clone()),
            PortValue::Text(_) => (None, v.to_json()),
        },
    };
    let raw: Vec<RawWaypoint> = serde_json::from_value(json).map_err(|e| {
        Error::plan(
            DiagCode::E010,
            path,
            format!("trajectory is not a waypoint list: {e}"),
        )
    })?;
    if raw.len() < 2 {
        return Err(Error::plan(
            DiagCode::E010,
            path,
            format!("trajectory has {} waypoint(s); need >= 2", raw.len()),
        ));
    }
    let t0 = raw[0].time_s;
    if t0 != 0.0 {
        d.warn(
            DiagCode::W020,
            path,
            format!("trajectory starts at t={t0} s; shifted to 0"),
        );
    }
    let mut waypoints = Vec::with_capacity(raw.len());
    let mut last_t = f64::NEG_INFINITY;
    for (i, w) in raw.iter().enumerate() {
        if w.positions.len() != 6 {
            return Err(Error::plan(
                DiagCode::E010,
                path,
                format!("waypoint {i} has {} joints; need 6", w.positions.len()),
            ));
        }
        if !w.positions.iter().all(|q| q.is_finite()) || !w.time_s.is_finite() {
            return Err(Error::plan(
                DiagCode::E010,
                path,
                format!("waypoint {i} has a non-finite value"),
            ));
        }
        let t = w.time_s - t0;
        if t < last_t {
            return Err(Error::plan(
                DiagCode::E010,
                path,
                format!("waypoint {i}: time_s decreases"),
            ));
        }
        last_t = t;
        for q in &w.positions {
            if q.to_degrees().abs() >= 655.34 {
                return Err(Error::plan(
                    DiagCode::E017,
                    path,
                    format!(
                        "waypoint {i}: joint {:.1} deg exceeds the INT16 encoding range",
                        q.to_degrees()
                    ),
                ));
            }
        }
        let mut q = [0.0; 6];
        q.copy_from_slice(&w.positions);
        let phase = match w.phase.as_deref() {
            Some("transit") | None => Phase::Transit,
            Some("approach") => Phase::Approach,
            Some(other) => Phase::Other(other.to_string()),
        };
        waypoints.push(Waypoint {
            t_s: t,
            q_rad: q,
            phase,
        });
    }
    Ok(Some(Trajectory { key, waypoints }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::{BtPath, NodeType};

    fn node(traj: Value) -> NormNode {
        let mut ports = BTreeMap::new();
        ports.insert("trajectory".to_string(), PortValue::Json(traj));
        NormNode {
            ty: NodeType::ExecuteTrajectory,
            raw_type: "ExecuteTrajectory".into(),
            legacy: false,
            name: None,
            id: None,
            ports,
            children: vec![],
            path: BtPath::root("ExecuteTrajectory"),
            dfs: 0,
            line: None,
        }
    }

    #[test]
    fn resolves_and_validates() {
        let mut bb = Blackboard::default();
        bb.0.insert(
            "t".into(),
            serde_json::json!([
                {"positions": [0,0,0,0,0,0], "time_s": 0.0, "phase": "transit"},
                {"positions": [0.1,0,0,0,0,0], "time_s": 1.5, "phase": "approach"}
            ]),
        );
        let mut d = Diagnostics::default();
        let t = trajectory(&node(Value::String("{t}".into())), &bb, &mut d)
            .unwrap()
            .unwrap();
        assert_eq!(t.duration_s(), 1.5);
        assert_eq!(t.waypoints[1].phase, Phase::Approach);
        assert!(trajectory(&node(Value::String("{missing}".into())), &bb, &mut d).is_err());
        let five = serde_json::json!([{"positions": [0,0,0,0,0], "time_s": 0.0}, {"positions": [0,0,0,0,0], "time_s": 1.0}]);
        assert!(
            trajectory(&node(five), &bb, &mut d)
                .unwrap_err()
                .to_string()
                .contains("E010")
        );
    }
}
