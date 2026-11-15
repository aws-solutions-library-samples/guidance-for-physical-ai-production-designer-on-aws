//! The task and the reach domes in the Robo Fitter's 3D view, as renderer-neutral scene nodes
//! ([`factoryos_protocol::scene`]; the backend makes them, the viewer only toggles them through
//! [`factoryos_protocol::fitter::FitterView`]): `world/task/{welds,approach,paths,selected}`
//! (points, arrows, lines, points) and `world/reach/<robot_id>/{outer,inner,rings}` (triangle
//! meshes and lines). Positions are relative to the scene origin, in the scene's world frame.

use std::collections::HashMap;

use factoryos_protocol::fitter::{ReachReport, TaskWeld, WeldStatus};
use factoryos_protocol::scene::{Node, NodeKind, Rgba};

use super::{Analysis, SceneRobot};

pub const TASK: &str = "world/task";
pub const REACH: &str = "world/reach";

const OUTER: Rgba = [0x2e, 0x9b, 0xff, 0x30];
const INNER: Rgba = [0xff, 0x4d, 0x4d, 0x30];
const RING: Rgba = [0x2e, 0x9b, 0xff, 0xc0];
const APPROACH: Rgba = [0xff, 0xb0, 0x00, 0xff];
const PATH: Rgba = [0xff, 0x5a, 0x1e, 0xff];
const SELECTED: Rgba = [0xff, 0xff, 0xff, 0x90];

/// Weld colours: green, orange, red, grey.
pub fn status_color(status: WeldStatus) -> Rgba {
    match status {
        WeldStatus::AllRobots => [0x2e, 0xcc, 0x40, 0xff],
        WeldStatus::OneRobot => [0xff, 0x9f, 0x1a, 0xff],
        WeldStatus::NoRobot => [0xff, 0x3b, 0x30, 0xff],
        WeldStatus::Unknown => [0x9e, 0x9e, 0x9e, 0xff],
    }
}

/// Each weld's status and label: from `report` when it is a finished IK check of this scene
/// (`"W02 r1"`: the robots that reach it), else the dome estimate's (`"W02~"`).
pub fn statuses_and_labels(analysis: &Analysis, report: Option<&ReachReport>) -> (Vec<WeldStatus>, Vec<String>) {
    let checked = report.filter(|r| r.scene_rev == analysis.scene_rev && r.method == "ik" && !r.running);
    let short: HashMap<&str, &str> = analysis.robots.iter().map(|r| (r.id.as_str(), r.short.as_str())).collect();
    analysis
        .welds
        .iter()
        .zip(&analysis.estimate)
        .map(|(weld, estimate)| {
            match checked.and_then(|r| r.welds.iter().find(|w| w.weld.id == weld.id)) {
                Some(w) => {
                    let by: Vec<&str> = w.reached_by.iter().map(|id| short.get(id.as_str()).copied().unwrap_or(id.as_str())).collect();
                    let by = if by.is_empty() { "–".to_owned() } else { by.join(" ") };
                    (w.status, format!("{} {by}", weld.label))
                }
                None => (estimate.status, format!("{}~", weld.label)),
            }
        })
        .unzip()
}

fn at(origin: [f64; 3], p: [f64; 3]) -> [f64; 3] {
    [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]]
}

/// The task's welds (nothing when `shown` is false), under [`TASK`]. `origin`: the scene origin.
/// `labels`: whether the welds carry their text labels.
pub fn task_nodes(
    origin: [f64; 3],
    analysis: &Analysis,
    report: Option<&ReachReport>,
    selected: Option<&TaskWeld>,
    shown: bool,
    labels: bool,
) -> Vec<Node> {
    if !shown || analysis.welds.is_empty() {
        return Vec::new();
    }
    let (statuses, texts) = statuses_and_labels(analysis, report);
    let texts = if labels { texts } else { Vec::new() };
    let positions: Vec<[f64; 3]> = analysis.welds.iter().map(|w| at(origin, w.position)).collect();
    let mut nodes = vec![Node::new(
        format!("{TASK}/welds"),
        NodeKind::Points {
            points: positions.clone(),
            colors: statuses.iter().map(|s| status_color(*s)).collect(),
            labels: texts,
            radius: Some(0.02),
            show_labels: Some(labels),
        },
    )];
    let len = 0.15;
    let vectors: Vec<[f64; 3]> = analysis.welds.iter().map(|w| w.approach.map(|v| v * len)).collect();
    let origins: Vec<[f64; 3]> = positions.iter().zip(&vectors).map(|(p, v)| [p[0] - v[0], p[1] - v[1], p[2] - v[2]]).collect();
    nodes.push(Node::new(
        format!("{TASK}/approach"),
        NodeKind::Arrows { origins, vectors, colors: vec![APPROACH], labels: Vec::new(), radius: Some(0.005), show_labels: None },
    ));
    let paths = analysis.task.paths();
    if !paths.is_empty() {
        let strips = paths.iter().map(|line| line.iter().map(|p| at(origin, *p)).collect()).collect();
        nodes.push(Node::new(format!("{TASK}/paths"), NodeKind::Lines { strips, colors: vec![PATH], radius: Some(0.006), labels: Vec::new(), show_labels: None }));
    }
    if let Some(weld) = selected {
        nodes.push(Node::new(
            format!("{TASK}/selected"),
            NodeKind::Points {
                points: vec![at(origin, weld.position)],
                colors: vec![SELECTED],
                labels: Vec::new(),
                radius: Some(0.045),
                show_labels: None,
            },
        ));
    }
    nodes
}

/// What [`reach_nodes`] draws depends on: the robots' bases and models, the dominant approach, the
/// weld heights, and whether it is shown at all.
pub fn reach_key(origin: [f64; 3], analysis: &Analysis, shown: bool) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    shown.hash(&mut h);
    if shown {
        let mut floats: Vec<f64> = origin.to_vec();
        floats.extend(analysis.dominant_approach());
        floats.extend(analysis.welds.iter().map(|w| (w.position[2] * 100.0).round()));
        for robot in &analysis.robots {
            robot.id.hash(&mut h);
            robot.urdf.hash(&mut h);
            floats.extend(robot.base.to_homogeneous().iter().copied());
            floats.push(robot.tool_length());
        }
        for f in floats {
            f.to_bits().hash(&mut h);
        }
    }
    h.finish()
}

/// Each robot's TCP dome for the task's dominant approach, with its `r_max` ring at every weld
/// height (nothing when `shown` is false), under [`REACH`].
pub fn reach_nodes(origin: [f64; 3], analysis: &Analysis, shown: bool) -> Vec<Node> {
    if !shown {
        return Vec::new();
    }
    let approach = analysis.dominant_approach();
    let mut heights: Vec<f64> = analysis.welds.iter().map(|w| (w.position[2] * 100.0).round() / 100.0).collect();
    heights.sort_by(f64::total_cmp);
    heights.dedup();
    analysis.robots.iter().flat_map(|robot| robot_dome(origin, robot, approach, &heights)).collect()
}

fn robot_dome(origin: [f64; 3], robot: &SceneRobot, approach: [f64; 3], heights: &[f64]) -> Vec<Node> {
    let Some(kind) = robot.kind.as_ref() else { return Vec::new() };
    let Some(envelope) = kind.envelope.as_ref() else { return Vec::new() };
    let l = kind.tool_length;
    let shift = approach.map(|v| v * l);
    let (outer, inner) = envelope.mesh(&robot.base, shift, 64);
    let id = format!("{REACH}/{}", robot.id);
    let mut nodes = Vec::new();
    for (name, surface, color) in [("outer", outer, OUTER), ("inner", inner, INNER)] {
        if surface.triangles.is_empty() {
            continue;
        }
        let vertices = surface.vertices.iter().map(|p| at(origin, *p)).collect();
        nodes.push(Node::new(format!("{id}/{name}"), NodeKind::TriMesh { vertices, triangles: surface.triangles, color: Some(color) }));
    }
    let rings: Vec<Vec<[f64; 3]>> = heights
        .iter()
        .filter_map(|z| envelope.ring(&robot.base, shift, *z, 96))
        .map(|ring| ring.iter().map(|p| at(origin, *p)).collect())
        .collect();
    if !rings.is_empty() {
        nodes.push(Node::new(format!("{id}/rings"), NodeKind::Lines { strips: rings, colors: vec![RING], radius: Some(0.006), labels: Vec::new(), show_labels: None }));
    }
    nodes
}

#[cfg(test)]
mod tests {
    use factoryos_protocol::fitter::{TaskSource, WeldReach};

    use super::super::{analyse, task, tests_support};
    use super::*;

    fn jam2() -> Analysis {
        analyse(&tests_support::ctx(), &task::tests::jam2(), &task::tests::fixtures().join("scenes/jam2"), None, TaskSource::Product)
    }

    fn ids(nodes: &[Node]) -> Vec<&str> {
        nodes.iter().map(|n| n.id.as_str()).collect()
    }

    #[test]
    fn task_nodes_show_the_welds_with_their_status() {
        let a = jam2();
        let origin = [1.0, -2.0, 0.5];
        assert!(task_nodes(origin, &a, None, None, false, true).is_empty(), "hidden");

        let nodes = task_nodes(origin, &a, None, Some(&a.welds[1]), true, true);
        assert_eq!(ids(&nodes), ["world/task/welds", "world/task/approach", "world/task/selected"]);
        let NodeKind::Points { points, colors, labels, radius, show_labels } = &nodes[0].kind else { panic!("{:?}", nodes[0]) };
        assert_eq!(points.len(), 4);
        assert_eq!(points[0], at(origin, a.welds[0].position), "relative to the scene origin");
        assert_eq!(colors[0], status_color(WeldStatus::OneRobot), "the estimate's");
        assert_eq!(labels[0], format!("{}~", a.welds[0].label));
        assert_eq!((*radius, *show_labels), (Some(0.02), Some(true)), "the labels show");
        let NodeKind::Arrows { origins, vectors, radius, .. } = &nodes[1].kind else { panic!() };
        assert_eq!(*radius, Some(0.005));
        // Approach [0, 0, -1]: 15 cm arrows that end at the weld.
        assert_eq!(vectors[0], [0.0, 0.0, -0.15]);
        assert!((origins[0][2] - (points[0][2] + 0.15)).abs() < 1e-12);
        let NodeKind::Points { points, .. } = &nodes[2].kind else { panic!() };
        assert_eq!(points, &[at(origin, a.welds[1].position)]);

        // A finished IK check of this scene labels the welds with the robots that reach them.
        let report = ReachReport {
            scene_rev: a.scene_rev,
            method: "ik".into(),
            welds: vec![WeldReach {
                weld: a.welds[0].clone(),
                reached_by: Vec::new(),
                status: WeldStatus::NoRobot,
                ..Default::default()
            }],
            ..Default::default()
        };
        let nodes = task_nodes(origin, &a, Some(&report), None, true, true);
        let NodeKind::Points { colors, labels, .. } = &nodes[0].kind else { panic!() };
        assert_eq!((colors[0], labels[0].as_str()), (status_color(WeldStatus::NoRobot), format!("{} –", a.welds[0].label).as_str()));

        // "Labels" off: the coloured spots stay, without text.
        let nodes = task_nodes(origin, &a, Some(&report), None, true, false);
        let NodeKind::Points { colors, labels, show_labels, .. } = &nodes[0].kind else { panic!() };
        assert_eq!((colors[0], labels.is_empty(), *show_labels), (status_color(WeldStatus::NoRobot), true, Some(false)));
    }

    #[test]
    fn reach_nodes_make_domes_and_rings_per_robot() {
        let a = jam2();
        assert!(reach_nodes([0.0; 3], &a, false).is_empty());
        let nodes = reach_nodes([0.0; 3], &a, true);
        assert!(!nodes.is_empty());
        for robot in &a.robots {
            let has = |part: &str| nodes.iter().any(|n| n.id == format!("{REACH}/{}/{part}", robot.id));
            if robot.kind.as_ref().is_some_and(|k| k.envelope.is_some()) {
                assert!(has("outer") && has("rings"), "{}: {:?}", robot.id, ids(&nodes));
            }
        }
        for node in &nodes {
            match &node.kind {
                NodeKind::TriMesh { vertices, triangles, color } => {
                    assert!(!triangles.is_empty());
                    assert!(triangles.iter().flatten().all(|i| (*i as usize) < vertices.len()));
                    assert!(matches!(color, Some(c) if *c == OUTER || *c == INNER));
                }
                NodeKind::Lines { strips, colors, radius, .. } => {
                    assert!(!strips.is_empty() && *colors == [RING] && *radius == Some(0.006));
                }
                other => panic!("{other:?}"),
            }
        }
    }
}
