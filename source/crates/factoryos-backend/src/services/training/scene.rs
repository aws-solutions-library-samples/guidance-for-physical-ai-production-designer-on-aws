//! The training job's 3D view as renderer-neutral nodes ([`factoryos_protocol::scene`]): pure, from
//! the shown cell and the slots' resolved models.
//!
//! * `robots/slot<i>`: a [`NodeKind::Model`] at the drawn base, at its home pose; slots this draw
//!   does not use stay in the scene hidden, so the next draw does not fetch them again.
//! * `robots/slot<i>/pedestal`: a [`NodeKind::Cylinder`] (world frame, posed on the floor under
//!   the base) under a lifted base.
//! * `world/robot_labels`, `world/products`, `world/welds`, `world/weld_approach`, and
//!   `world/placement` (where the bases may go; a drawn cell only).
//!
//! Labels are short, as several slots and products stand close together: a robot's is `r<n>`
//! ([`LABEL_ABOVE_BASE_M`] above its base), a product's its name. Models and sizes are in the
//! training panel.

use std::collections::BTreeMap;

use factoryos_protocol::scene::{ModelRef, Node, NodeKind, Pose3, Rgba, ViewHints};
use factoryos_protocol::training::SampledCell;

use crate::services::pose3::pose3_from_xyz_yaw_deg;

/// How long the drawn approach arrows are, metres.
pub const ARROW_M: f64 = 0.15;
/// Lower bases stand on the floor (no pedestal).
const PEDESTAL_MIN_M: f64 = 0.005;
const PEDESTAL_RADIUS_M: f64 = 0.3;
/// A robot's label hangs this far above its base, over the arm, clear of the products' labels
/// (at their centres): with the slots close together, labels at the bases ran into each other.
pub const LABEL_ABOVE_BASE_M: f64 = 1.8;

pub const PLACEMENT: &str = "world/placement";

const PEDESTAL_COLOR: Rgba = rgba(0x5a5a64ff);
const LABEL_COLOR: Rgba = rgba(0x7c4dffff);
const PRODUCT_COLOR: Rgba = rgba(0x9fb4c8c0);
const WELD_COLOR: Rgba = rgba(0xff9f1cff);
const APPROACH_COLOR: Rgba = rgba(0xff9f1c99);
const PLACEMENT_COLOR: Rgba = rgba(0x3fb95080);

const fn rgba(c: u32) -> Rgba {
    c.to_be_bytes()
}

/// A slot's loaded model and its joint values (the home pose).
pub struct SlotScene<'a> {
    pub model: &'a ModelRef,
    pub joints: BTreeMap<String, f64>,
}

pub fn slot_id(i: usize) -> String {
    format!("robots/slot{i}")
}

pub fn view(cell: &str) -> ViewHints {
    ViewHints { title: format!("Training job · cell {cell}") }
}

/// The nodes that show `shown`. Robot `i` goes in slot `i`; `slots[i]` is that slot's model once it
/// has loaded (`None` while it loads: the robot gets its label and pedestal only). Robots in
/// `missing` (no URDF) are not shown. `guides` are the placement strips (on the floor), if any.
pub fn cell_nodes(shown: &SampledCell, slots: &[Option<SlotScene>], missing: &[usize], guides: Option<&[Vec<[f64; 2]>]>) -> Vec<Node> {
    let mut nodes = Vec::new();

    // Robots.
    let (mut label_at, mut labels) = (Vec::new(), Vec::new());
    for i in 0..slots.len().max(shown.robots.len()) {
        let robot = shown.robots.get(i).filter(|_| !missing.contains(&i));
        let id = slot_id(i);
        if let Some(slot) = slots.get(i).and_then(Option::as_ref) {
            let model = Node::new(
                id.as_str(),
                NodeKind::Model { model: slot.model.clone(), joints: slot.joints.clone(), show_collision: false },
            );
            nodes.push(match robot {
                Some(r) => model.at(pose3_from_xyz_yaw_deg(r.position, r.yaw_deg)),
                None => model.hidden(),
            });
        }
        let h = robot.map_or(0.0, |r| r.position[2]);
        // On the floor under the base, so a base that moves moves its pedestal by its pose alone.
        let pedestal = Node::new(
            format!("{id}/pedestal"),
            NodeKind::Cylinder {
                height: h,
                radius: PEDESTAL_RADIUS_M,
                center: [0.0, 0.0, h / 2.0],
                color: Some(PEDESTAL_COLOR),
                solid: true,
            },
        )
        .at(Pose3::from_xyz(robot.map_or([0.0; 3], |r| [r.position[0], r.position[1], 0.0])));
        nodes.push(if h > PEDESTAL_MIN_M { pedestal } else { pedestal.hidden() });
        if let Some(r) = robot {
            label_at.push([r.position[0], r.position[1], r.position[2] + LABEL_ABOVE_BASE_M]);
            labels.push(format!("r{}", i + 1));
        }
    }
    nodes.push(Node::new(
        "world/robot_labels",
        NodeKind::Points { points: label_at, colors: vec![LABEL_COLOR], labels, radius: Some(0.03), show_labels: None },
    ));

    // Products.
    let products = &shown.products;
    nodes.push(Node::new(
        "world/products",
        NodeKind::Boxes {
            centers: products.iter().map(|p| p.center).collect(),
            half_sizes: products.iter().map(|p| p.half).collect(),
            quats_xyzw: products.iter().map(|p| pose3_from_xyz_yaw_deg(p.center, p.yaw_deg).quat_xyzw).collect(),
            colors: vec![PRODUCT_COLOR],
            labels: products.iter().map(|p| p.name.clone()).collect(),
            solid: true,
            show_labels: None,
        },
    ));

    // Welds and their approach.
    let welds = &shown.welds;
    nodes.push(Node::new(
        "world/welds",
        NodeKind::Points {
            points: welds.iter().map(|w| w.position).collect(),
            colors: vec![WELD_COLOR],
            labels: Vec::new(),
            radius: Some(0.015),
            show_labels: None,
        },
    ));
    nodes.push(Node::new(
        "world/weld_approach",
        NodeKind::Arrows {
            origins: welds.iter().map(|w| std::array::from_fn(|k| w.position[k] - w.approach[k] * ARROW_M)).collect(),
            vectors: welds.iter().map(|w| w.approach.map(|v| v * ARROW_M)).collect(),
            colors: vec![APPROACH_COLOR],
            labels: Vec::new(),
            radius: None,
            show_labels: None,
        },
    ));

    // Where the bases may go.
    if let Some(guides) = guides {
        nodes.push(Node::new(
            PLACEMENT,
            NodeKind::Lines {
                strips: guides.iter().map(|strip| strip.iter().map(|&[x, y]| [x, y, 0.002]).collect()).collect(),
                colors: vec![PLACEMENT_COLOR],
                radius: Some(0.006),
                labels: Vec::new(),
                show_labels: None,
            },
        ));
    }
    nodes
}

#[cfg(test)]
mod tests {
    use factoryos_protocol::scene::AssetRef;
    use factoryos_protocol::training::{SampledProduct, SampledRobot, SampledWeld};

    use super::*;

    fn model(sha: &str) -> ModelRef {
        ModelRef { urdf: AssetRef { sha256: sha.into(), size: 1, media_type: "application/xml".into() }, meshes: BTreeMap::new() }
    }

    fn cell() -> SampledCell {
        SampledCell {
            seed: Some(7),
            robots: vec![
                SampledRobot { model: "robots/kr16.urdf".into(), position: [1.0, 2.0, 0.5], yaw_deg: 90.0 },
                SampledRobot { model: "robots/kr16.urdf".into(), position: [-1.0, 0.0, 0.0], yaw_deg: 0.0 },
            ],
            products: vec![SampledProduct { name: "p".into(), center: [0.0, 0.0, 0.4], half: [0.5, 0.25, 0.4], yaw_deg: 90.0 }],
            welds: vec![SampledWeld { position: [0.0, 0.0, 0.8], approach: [0.0, 0.0, -1.0], product: 0 }],
            notes: Vec::new(),
        }
    }

    fn node<'a>(nodes: &'a [Node], id: &str) -> &'a Node {
        nodes.iter().find(|n| n.id == id).unwrap_or_else(|| panic!("no {id}"))
    }

    #[test]
    fn robots_become_placed_models_at_home() {
        let (a, b, c) = (model("a"), model("b"), model("c"));
        let home = BTreeMap::from([("A2".to_owned(), -1.5)]);
        let slots = [
            Some(SlotScene { model: &a, joints: home.clone() }),
            Some(SlotScene { model: &b, joints: home.clone() }),
            Some(SlotScene { model: &c, joints: home.clone() }), // Left from a bigger draw.
        ];
        let nodes = cell_nodes(&cell(), &slots, &[], None);

        let r1 = node(&nodes, "robots/slot0");
        assert!(r1.visible && r1.parent.is_none());
        assert_eq!(r1.pose, pose3_from_xyz_yaw_deg([1.0, 2.0, 0.5], 90.0));
        let NodeKind::Model { model, joints, show_collision } = &r1.kind else { panic!() };
        assert_eq!((model, joints, *show_collision), (&a, &home, false));

        let unused = node(&nodes, "robots/slot2");
        assert!(!unused.visible, "unused slots are hidden, not removed");
        assert!(!node(&nodes, "robots/slot2/pedestal").visible);

        let pedestal = node(&nodes, "robots/slot0/pedestal");
        assert!(pedestal.visible && pedestal.parent.is_none());
        assert_eq!(pedestal.pose, Pose3::from_xyz([1.0, 2.0, 0.0]));
        assert!(matches!(pedestal.kind, NodeKind::Cylinder { height, radius, center, .. }
            if height == 0.5 && radius == 0.3 && center == [0.0, 0.0, 0.25]));
        assert!(!node(&nodes, "robots/slot1/pedestal").visible, "a base on the floor has none");

        // Parents before children.
        let at = |id: &str| nodes.iter().position(|n| n.id == id).unwrap();
        assert!(at("robots/slot0") < at("robots/slot0/pedestal"));

        let NodeKind::Points { points, labels, .. } = &node(&nodes, "world/robot_labels").kind else { panic!() };
        assert_eq!(points, &[[1.0, 2.0, 0.5 + LABEL_ABOVE_BASE_M], [-1.0, 0.0, LABEL_ABOVE_BASE_M]]);
        assert_eq!(labels, &["r1", "r2"]);
    }

    #[test]
    fn loading_and_missing_robots() {
        let a = model("a");
        let slots = [None, Some(SlotScene { model: &a, joints: BTreeMap::new() })];
        let nodes = cell_nodes(&cell(), &slots, &[1], None);
        assert!(nodes.iter().all(|n| n.id != "robots/slot0"), "still loading");
        assert!(node(&nodes, "robots/slot0/pedestal").visible);
        assert!(!node(&nodes, "robots/slot1").visible, "a robot without a URDF is not shown");
        let NodeKind::Points { labels, .. } = &node(&nodes, "world/robot_labels").kind else { panic!() };
        assert_eq!(labels, &["r1"]);
    }

    #[test]
    fn products_welds_and_guides() {
        let nodes = cell_nodes(&cell(), &[], &[], None);
        assert!(nodes.iter().all(|n| n.id != PLACEMENT));

        let NodeKind::Boxes { centers, half_sizes, quats_xyzw, labels, solid, .. } = &node(&nodes, "world/products").kind else { panic!() };
        assert_eq!((centers.as_slice(), half_sizes.as_slice(), *solid), (&[[0.0, 0.0, 0.4]][..], &[[0.5, 0.25, 0.4]][..], true));
        let q = quats_xyzw[0];
        assert!((q[2] - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-9 && (q[3] - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-9);
        assert_eq!(labels, &["p"]);

        let NodeKind::Points { points, .. } = &node(&nodes, "world/welds").kind else { panic!() };
        assert_eq!(points, &[[0.0, 0.0, 0.8]]);
        let NodeKind::Arrows { origins, vectors, .. } = &node(&nodes, "world/weld_approach").kind else { panic!() };
        assert_eq!(vectors, &[[0.0, 0.0, -ARROW_M]]);
        assert!((origins[0][2] - (0.8 + ARROW_M)).abs() < 1e-12, "the arrow ends at the weld");

        let guides = vec![vec![[0.0, 0.0], [1.0, 0.0]]];
        let nodes = cell_nodes(&cell(), &[], &[], Some(&guides));
        let NodeKind::Lines { strips, .. } = &node(&nodes, PLACEMENT).kind else { panic!() };
        assert_eq!(strips, &[vec![[0.0, 0.0, 0.002], [1.0, 0.0, 0.002]]]);
    }

    #[test]
    fn view_title() {
        assert_eq!(view("umarell").title, "Training job · cell umarell");
    }
}
