//! Layout suggestions: the smallest robot moves (or a higher pedestal) that bring the welds no
//! robot reaches, and then those only one reaches, inside the domes. Checked against the domes
//! only: re-check with the IK to confirm.

use factoryos_protocol::fitter::{PedestalYaml, ReachReason, ReachReport, SceneFile, Suggestion, TaskWeld};
use nalgebra::{Isometry3, Translation3, Vector3};

use super::SceneRobot;
use super::task::ProductBox;

/// A suggested weld is this far inside its dome.
pub const TARGET_MARGIN_M: f64 = 0.05;
/// Robot bases stay this far apart (horizontally), unless they already were closer.
pub const MIN_BASE_DISTANCE_M: f64 = 1.5;
/// A base stays this far from a product (the pedestal's radius, about).
const PRODUCT_CLEARANCE_M: f64 = 0.4;
const STEP_M: f64 = 0.01;

/// A suggestion and what it does to the scene.
#[derive(Clone, Debug)]
pub struct Move {
    /// Index into the scene's robots.
    pub robot: usize,
    pub suggestion: Suggestion,
    pub moved_m: [f64; 3],
    pub pedestal_mm: Option<f64>,
}

fn horizontal(a: Vector3<f64>, b: Vector3<f64>) -> f64 {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()
}

fn margin(robot: &SceneRobot, base: &Isometry3<f64>, weld: &TaskWeld) -> Option<f64> {
    let kind = robot.kind.as_ref()?;
    Some(kind.envelope.as_ref()?.margin(base, kind.tool_length, weld.position, weld.approach).0)
}

fn names(welds: &[&TaskWeld]) -> String {
    welds.iter().map(|w| w.label.as_str()).collect::<Vec<_>>().join(", ")
}

/// The suggestions for `report`'s welds (reached as it says) in the scene of `robots`, moving no
/// robot further than `max_move_m`.
pub fn suggest(robots: &[SceneRobot], report: &ReachReport, boxes: &[&ProductBox], max_move_m: f64) -> Vec<Move> {
    let usable: Vec<usize> = (0..robots.len()).filter(|&i| robots[i].kind.as_ref().is_some_and(|k| k.envelope.is_some())).collect();
    if usable.is_empty() {
        return Vec::new();
    }
    // Welds no robot reaches go to the nearest robot; welds one of several reaches, to the
    // nearest other one. Welds inside the product are the task's problem, not the layout's.
    let mut assigned: Vec<(Vec<&TaskWeld>, bool)> = vec![(Vec::new(), false); robots.len()]; // welds, any unreached
    let mut moves = Vec::new();
    let mut inside = Vec::new();
    for weld in &report.welds {
        if weld.attempts.iter().any(|a| a.reasons.contains(&ReachReason::WeldInsideProduct)) {
            inside.push(&weld.weld);
            continue;
        }
        let red = weld.reached_by.is_empty();
        if !red && (weld.reached_by.len() >= 2 || robots.len() < 2) {
            continue;
        }
        let nearest = usable
            .iter()
            .filter(|&&i| !weld.reached_by.contains(&robots[i].id))
            .min_by(|&&a, &&b| {
                let d = |i: usize| horizontal(robots[i].base.translation.vector, Vector3::from(weld.weld.position));
                d(a).total_cmp(&d(b))
            });
        if let Some(&i) = nearest {
            assigned[i].0.push(&weld.weld);
            assigned[i].1 |= red;
        }
    }
    if !inside.is_empty() {
        moves.push(Move {
            robot: usize::MAX,
            suggestion: Suggestion {
                robot_id: None,
                text: format!(
                    "{} lie inside {}: fix the task (no layout reaches them)",
                    names(&inside),
                    boxes.iter().map(|b| b.name.as_str()).collect::<Vec<_>>().join(", ")
                ),
                ..Default::default()
            },
            moved_m: [0.0; 3],
            pedestal_mm: None,
        });
    }
    let mut bases: Vec<Isometry3<f64>> = robots.iter().map(|r| r.base).collect();
    for i in usable {
        let (welds, red) = &assigned[i];
        if welds.is_empty() {
            continue;
        }
        let robot = &robots[i];
        let keep: Vec<(&TaskWeld, f64)> = report
            .welds
            .iter()
            .filter(|w| w.reached_by.contains(&robot.id))
            .filter_map(|w| Some((&w.weld, margin(robot, &robot.base, &w.weld)?.min(TARGET_MARGIN_M))))
            .collect();
        let found = search(robots, &bases, i, welds, &keep, boxes, max_move_m, 0.0).map(|m| (m, 0.0)).or_else(|| {
            // Welds above the dome: a higher pedestal (the lift the highest one needs).
            let kind = robot.kind.as_ref()?;
            let envelope = kind.envelope.as_ref()?;
            let need = welds
                .iter()
                .map(|w| envelope.wrist_rz(&robot.base, kind.tool_length, w.position, w.approach).1 - envelope.z_max() + TARGET_MARGIN_M)
                .fold(f64::MIN, f64::max);
            (need > 0.0).then_some(())?;
            // From the lift the highest weld needs, in 10 cm steps up to 2 m more.
            let first = (need * 100.0).ceil() / 100.0;
            (0..=20).map(|k| first + 0.1 * k as f64).find_map(|lift| {
                search(robots, &bases, i, welds, &keep, boxes, max_move_m, lift).map(|m| (m, lift))
            })
        });
        let prefix = if *red { format!("To reach {}", names(welds)) } else { format!("For a second robot on {}", names(welds)) };
        let suggestion = match found {
            Some((step, lift)) => {
                bases[i] = Translation3::new(step[0], step[1], lift) * robot.base;
                let distance = (step[0].powi(2) + step[1].powi(2)).sqrt();
                let current = robot_pedestal_mm(robot);
                let pedestal_mm = (lift > 0.0).then_some(current + lift * 1000.0);
                let mut text = format!("{prefix}: ");
                if distance > 1e-9 {
                    text += &format!("move {} by {distance:.2} m (x {:+.2}, y {:+.2})", robot.id, step[0], step[1]);
                }
                if let Some(mm) = pedestal_mm {
                    text += &format!("{}put {} on a {mm:.0} mm pedestal", if distance > 1e-9 { " and " } else { "" }, robot.id);
                }
                text += &format!(": every weld then ≥ {:.2} m inside its dome; re-check to confirm", TARGET_MARGIN_M);
                Move {
                    robot: i,
                    suggestion: Suggestion {
                        robot_id: Some(robot.id.clone()),
                        text,
                        move_m: (distance > 1e-9).then_some([step[0], step[1], 0.0]),
                        pedestal_height_mm: pedestal_mm,
                    },
                    moved_m: [step[0], step[1], 0.0],
                    pedestal_mm,
                }
            }
            None => Move {
                robot: i,
                suggestion: Suggestion {
                    robot_id: Some(robot.id.clone()),
                    text: format!(
                        "{prefix}: no move of {} up to {max_move_m:.1} m does it (robots kept {MIN_BASE_DISTANCE_M} m apart): move the product or add a robot",
                        robot.id
                    ),
                    ..Default::default()
                },
                moved_m: [0.0; 3],
                pedestal_mm: None,
            },
        };
        moves.push(suggestion);
    }
    moves
}

fn robot_pedestal_mm(robot: &SceneRobot) -> f64 {
    // The scene's pedestal, not the base's z: a robot placed at z ≠ 0 keeps its position.z.
    robot.pedestal_mm
}

/// The shortest horizontal move (towards the welds' centroid) of robot `i`, lifted by `lift`,
/// after which `welds` are ≥ [`TARGET_MARGIN_M`] inside its dome and `keep` stay at their margins.
#[allow(clippy::too_many_arguments)]
fn search(
    robots: &[SceneRobot],
    bases: &[Isometry3<f64>],
    i: usize,
    welds: &[&TaskWeld],
    keep: &[(&TaskWeld, f64)],
    boxes: &[&ProductBox],
    max_move_m: f64,
    lift: f64,
) -> Option<[f64; 2]> {
    let robot = &robots[i];
    let from = robot.base.translation.vector;
    let n = welds.len() as f64;
    let centroid = welds.iter().fold(Vector3::zeros(), |s, w| s + Vector3::from(w.position)) / n;
    let towards = Vector3::new(centroid.x - from.x, centroid.y - from.y, 0.0);
    let direction = if towards.norm() < 1e-9 { Vector3::zeros() } else { towards.normalize() };
    let steps = (max_move_m / STEP_M).floor() as usize;
    (0..=steps).find_map(|k| {
        let s = k as f64 * STEP_M;
        if k > 0 && direction.norm() < 0.5 {
            return None;
        }
        let shift = direction * s;
        let base = Translation3::new(shift.x, shift.y, lift) * robot.base;
        let at = base.translation.vector;
        let clear = (0..robots.len()).filter(|&j| j != i).all(|j| {
            let now = horizontal(from, bases[j].translation.vector);
            horizontal(at, bases[j].translation.vector) >= MIN_BASE_DISTANCE_M.min(now) - 1e-9
        });
        let off_product = boxes.iter().all(|b| {
            let c = b.pose.translation.vector;
            b.signed_distance([at.x, at.y, c.z]) >= PRODUCT_CLEARANCE_M.min(b.signed_distance([from.x, from.y, c.z]))
        });
        let reaches = welds.iter().all(|w| margin(robot, &base, w).is_some_and(|m| m >= TARGET_MARGIN_M));
        let keeps = keep.iter().all(|(w, m)| margin(robot, &base, w).is_some_and(|now| now >= *m - 1e-9));
        (clear && off_product && reaches && keeps).then_some([shift.x, shift.y])
    })
}

/// `scene` with `moves` applied (robot positions and pedestals).
pub fn apply(scene: &SceneFile, moves: &[Move]) -> SceneFile {
    let mut scene = scene.clone();
    for m in moves {
        let Some(robot) = scene.robots.get_mut(m.robot) else { continue };
        robot.position.x += m.moved_m[0];
        robot.position.y += m.moved_m[1];
        if let Some(mm) = m.pedestal_mm {
            // Only the height changes: the pedestal's other keys stay.
            robot.pedestal.get_or_insert_with(|| PedestalYaml::new(mm, 800.0)).height_mm = mm;
        }
    }
    scene
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::services::reach::{CheckOptions, analyse, check_scene, task};
    use factoryos_protocol::fitter::TaskSource;
    use std::sync::atomic::AtomicBool;

    #[test]
    fn jam2_robots_move_in_for_the_far_welds() {
        let ctx = super::super::tests_support::ctx();
        let base = task::tests::fixtures().join("scenes/jam2");
        let scene = task::tests::jam2();
        let options = CheckOptions { source: TaskSource::Product, estimate_only: true, ..Default::default() };
        let report = check_scene(&ctx, &scene, &base, None, &options, |_| {}, &AtomicBool::new(false));
        let a = analyse(&ctx, &scene, &base, None, TaskSource::Product);
        let moves = suggest(&a.robots, &report, &a.boxes(), 1.0);
        assert_eq!(moves.len(), 2, "{moves:?}");
        // r1 (x = 2.5) moves towards −x, r2 towards +x, about 12 cm each.
        assert!(moves[0].moved_m[0] < -0.08 && moves[0].moved_m[0] > -0.2, "{moves:?}");
        assert!(moves[1].moved_m[0] > 0.08 && moves[1].moved_m[0] < 0.2, "{moves:?}");
        assert!(moves[0].suggestion.text.contains("re-check to confirm"));
        let moved = apply(&scene, &moves);
        let b = analyse(&ctx, &moved, &base, None, TaskSource::Product);
        for e in &b.estimate {
            assert_eq!(e.robots.len(), 2, "{e:?}");
            assert!(e.margins_m.values().all(|m| *m >= TARGET_MARGIN_M - 1e-9), "{e:?}");
        }
    }

    #[test]
    fn welds_above_the_dome_get_a_pedestal() {
        let ctx = super::super::tests_support::ctx();
        let base = task::tests::fixtures().join("scenes/jam2");
        let mut scene = task::tests::jam2();
        scene.robots.truncate(1);
        scene.robots[0].pedestal = None;
        scene.robots[0].position.x = 2.0;
        scene.fixtures[0].position.z = 1.2; // the top welds at 3.2 m: above the wrist's reach from the floor
        let options = CheckOptions { source: TaskSource::Product, estimate_only: true, ..Default::default() };
        let report = check_scene(&ctx, &scene, &base, None, &options, |_| {}, &AtomicBool::new(false));
        let a = analyse(&ctx, &scene, &base, None, TaskSource::Product);
        let moves = suggest(&a.robots, &report, &a.boxes(), 1.0);
        let lifted: Vec<&Move> = moves.iter().filter(|m| m.pedestal_mm.is_some()).collect();
        assert!(!lifted.is_empty(), "{moves:?} {:?}", report.welds);
        let moved = apply(&scene, &moves);
        let b = analyse(&ctx, &moved, &base, None, TaskSource::Product);
        assert!(b.estimate.iter().all(|e| e.robots.len() == 1), "{:?}", b.estimate);
    }

    /// A move changes the position and the pedestal's height only: the robot's and the
    /// pedestal's other keys stay; a robot without a pedestal gets one of the default diameter.
    #[test]
    fn a_move_changes_only_the_position_and_the_pedestal_height() {
        let scene: SceneFile = serde_yaml::from_str(
            "robots:\n- robot_id: a\n  dressed_urdf: a.urdf\n  position: {x: 0.0, y: 0.0, z: 0.0}\n  \
             pedestal: {height_mm: 300.0, diameter_mm: 900.0, material: steel}\n  payload: {mass_kg: 12.0}\n  cable: north\n\
             - robot_id: b\n  dressed_urdf: b.urdf\n  position: {x: 3.0, y: 0.0, z: 0.0}\n",
        )
        .unwrap();
        let raise = |robot, mm| Move { robot, suggestion: Suggestion::default(), moved_m: [0.25, 0.0, 0.0], pedestal_mm: Some(mm) };
        let moved = apply(&scene, &[raise(0, 650.0), raise(1, 400.0)]);
        let mut expected = scene.clone();
        expected.robots[0].position.x = 0.25;
        expected.robots[0].pedestal.as_mut().unwrap().height_mm = 650.0;
        expected.robots[1].position.x = 3.25;
        expected.robots[1].pedestal = Some(PedestalYaml::new(400.0, 800.0));
        assert_eq!(moved, expected);
        assert_eq!(moved.robots[0].pedestal.as_ref().unwrap().extra["material"], "steel");
        assert_eq!(moved.robots[0].extra["cable"], "north");
    }
}
