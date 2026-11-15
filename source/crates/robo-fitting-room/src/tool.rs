//! FR-111, FR-112, FR-159: which of a robot's geometry is its tool's, and the link its payload
//! merges into.
//!
//! By default the tool is what `urdf-robo-dresser` writes under `meshes/tool/` (visual and
//! collision meshes alike), on whichever link carries it; the payload merges into the link with the
//! most such collision meshes, else into the deepest leaf link. A manifest's `payload.link` names
//! the carrier instead, for a URDF whose tool doesn't follow the dresser's layout: then the tool is
//! that link and the links fixed below it — their `meshes/tool/` geometry if they have any, else
//! all of their geometry.

use std::collections::{HashMap, HashSet};

use urdf_robo_dresser::{Geometry, JointType, Link, UrdfRobot};

/// Whether a mesh file is the tool's by its path: `urdf-robo-dresser` writes the tool's meshes
/// under `meshes/tool/` and the robot's under `meshes/robot/` (either separator, any case).
pub fn is_tool_mesh_path(filename: &str) -> bool {
    let lower = filename.to_ascii_lowercase();
    lower.contains("meshes/tool/") || lower.contains("meshes\\tool\\")
}

/// Whether `geometry` is a mesh under `meshes/tool/` ([`is_tool_mesh_path`]).
pub fn is_tool_mesh(geometry: &Geometry) -> bool {
    matches!(geometry, Geometry::Mesh { filename, .. } if is_tool_mesh_path(filename))
}

/// How a robot's payload carrier was found ([`ToolLayout::carrier`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CarrierSource {
    /// The manifest's `payload.link`.
    Named,
    /// The link with the most `meshes/tool/` collision meshes.
    ToolMeshes,
    /// The deepest leaf link: no link has a `meshes/tool/` collision mesh.
    DeepestLeaf,
    /// The robot has no link.
    None,
}

/// FR-159: a robot's tool: the link its payload merges into, the links fixed below it and which
/// of their geometry is the tool's.
#[derive(Debug, Clone, PartialEq)]
pub struct ToolLayout {
    /// The link a payload merges into; `None` for a robot without links.
    pub carrier: Option<String>,
    /// The carrier and the links joined below it through fixed joints only (depth first, in URDF
    /// declaration order): the tool's links, whose tool collision hulls a payload is spread over.
    pub links: Vec<String>,
    /// Whether every visual and collision of [`Self::links`] is the tool's: a named carrier whose
    /// links have no `meshes/tool/` geometry. Otherwise the `meshes/tool/` geometry is, on every
    /// link.
    pub all_geometry: bool,
    pub source: CarrierSource,
}

impl ToolLayout {
    /// The tool layout of `robot`, its carrier named by `payload_link` (the manifest's
    /// `payload.link`) when that is one of its links, else found as [`CarrierSource`] lists.
    pub fn of(robot: &UrdfRobot, payload_link: Option<&str>) -> Self {
        let named = payload_link.and_then(|name| robot.links.iter().find(|l| l.name == name));
        let (carrier, source) = match named {
            Some(link) => (Some(link), CarrierSource::Named),
            None => match most_tool_collisions(robot) {
                Some(link) => (Some(link), CarrierSource::ToolMeshes),
                None => match deepest_leaf(robot) {
                    Some(link) => (Some(link), CarrierSource::DeepestLeaf),
                    None => (None, CarrierSource::None),
                },
            },
        };
        let links = carrier
            .map(|c| fixed_subtree(robot, &c.name))
            .unwrap_or_default();
        let all_geometry = source == CarrierSource::Named
            && !robot
                .links
                .iter()
                .filter(|l| links.contains(&l.name))
                .any(has_tool_meshes);
        Self {
            carrier: carrier.map(|l| l.name.clone()),
            links,
            all_geometry,
            source,
        }
    }

    /// Whether `link` is one of the tool's links ([`Self::links`]).
    pub fn is_tool_link(&self, link: &str) -> bool {
        self.links.iter().any(|l| l == link)
    }

    /// Whether every visual and collision of `link` is the tool's.
    pub fn all_tool(&self, link: &str) -> bool {
        self.all_geometry && self.is_tool_link(link)
    }

    /// Whether `geometry`, a visual or collision of `link`, is the tool's.
    pub fn is_tool(&self, link: &str, geometry: &Geometry) -> bool {
        self.all_tool(link) || is_tool_mesh(geometry)
    }

    /// Whether the tool has collision geometry a payload can be spread over: a `meshes/tool/`
    /// collision mesh on one of its links, or (with [`Self::all_geometry`]) any collision there.
    pub fn has_tool_collisions(&self, robot: &UrdfRobot) -> bool {
        robot
            .links
            .iter()
            .filter(|l| self.is_tool_link(&l.name))
            .any(|l| {
                l.collisions
                    .iter()
                    .any(|c| self.is_tool(&l.name, &c.geometry))
            })
    }
}

/// Whether `link` has a visual or collision mesh under `meshes/tool/`.
fn has_tool_meshes(link: &Link) -> bool {
    link.visuals.iter().any(|v| is_tool_mesh(&v.geometry))
        || link.collisions.iter().any(|c| is_tool_mesh(&c.geometry))
}

/// The link carrying the most `meshes/tool/` collision meshes (the first on a tie), if any does.
fn most_tool_collisions(robot: &UrdfRobot) -> Option<&Link> {
    let count = |link: &Link| {
        link.collisions
            .iter()
            .filter(|c| is_tool_mesh(&c.geometry))
            .count()
    };
    let mut carrier: Option<(&Link, usize)> = None;
    for link in &robot.links {
        let n = count(link);
        if n > 0 && carrier.is_none_or(|(_, best)| n > best) {
            carrier = Some((link, n));
        }
    }
    carrier.map(|(link, _)| link)
}

/// The deepest leaf link (no child joint; the first on a tie), counted in joints from the root.
fn deepest_leaf(robot: &UrdfRobot) -> Option<&Link> {
    let depths = link_depths(robot);
    let parents: HashSet<&str> = robot.joints.iter().map(|j| j.parent.as_str()).collect();
    let mut leaf: Option<(&Link, usize)> = None;
    for link in robot
        .links
        .iter()
        .filter(|l| !parents.contains(l.name.as_str()))
    {
        let depth = depths.get(link.name.as_str()).copied().unwrap_or(0);
        if leaf.is_none_or(|(_, deepest)| depth > deepest) {
            leaf = Some((link, depth));
        }
    }
    leaf.map(|(link, _)| link)
}

/// The URDF root link: the first link that is no joint's child.
pub fn root_link(robot: &UrdfRobot) -> Option<&str> {
    let children: HashSet<&str> = robot.joints.iter().map(|j| j.child.as_str()).collect();
    robot
        .links
        .iter()
        .find(|l| !children.contains(l.name.as_str()))
        .map(|l| l.name.as_str())
}

/// The number of joints between the URDF root link and every link it reaches.
fn link_depths(robot: &UrdfRobot) -> HashMap<&str, usize> {
    let mut depths = HashMap::new();
    let Some(root) = root_link(robot) else {
        return depths;
    };
    depths.insert(root, 0);
    let mut stack = vec![root];
    while let Some(parent) = stack.pop() {
        let depth = depths[parent];
        for joint in robot.joints.iter().filter(|j| j.parent == parent) {
            if !depths.contains_key(joint.child.as_str()) {
                depths.insert(joint.child.as_str(), depth + 1);
                stack.push(joint.child.as_str());
            }
        }
    }
    depths
}

/// `top` and every link joined below it through fixed joints only, depth first in URDF
/// declaration order.
pub fn fixed_subtree(robot: &UrdfRobot, top: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut stack = vec![top.to_owned()];
    while let Some(link) = stack.pop() {
        if out.contains(&link) {
            continue;
        }
        // Reversed, so the children pop in declaration order.
        for joint in robot
            .joints
            .iter()
            .rev()
            .filter(|j| j.parent == link && j.joint_type == JointType::Fixed)
        {
            stack.push(joint.child.clone());
        }
        out.push(link);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use urdf_robo_dresser::{Collision, Joint, Origin, Visual};

    fn mesh(file: &str) -> Geometry {
        Geometry::Mesh {
            filename: file.into(),
            scale: [1.0; 3],
        }
    }

    fn link(name: &str, collisions: &[&str], visuals: &[&str]) -> Link {
        Link {
            name: name.into(),
            inertial: None,
            visuals: visuals
                .iter()
                .map(|f| Visual {
                    origin: Origin::default(),
                    geometry: mesh(f),
                })
                .collect(),
            collisions: collisions
                .iter()
                .map(|f| Collision {
                    origin: Origin::default(),
                    geometry: mesh(f),
                })
                .collect(),
        }
    }

    fn joint(name: &str, kind: JointType, parent: &str, child: &str) -> Joint {
        Joint {
            name: name.into(),
            joint_type: kind,
            parent: parent.into(),
            child: child.into(),
            origin: Origin::default(),
            axis: [1.0, 0.0, 0.0],
            limit: None,
        }
    }

    fn robot(links: Vec<Link>, joints: Vec<Joint>) -> UrdfRobot {
        UrdfRobot {
            name: "r".into(),
            links,
            joints,
            reach: None,
        }
    }

    const TOOL: &str = "./meshes/tool/h.obj";
    const ROBOT: &str = "./meshes/robot/r.obj";

    #[test]
    fn tool_meshes_are_found_by_their_folder() {
        assert!(is_tool_mesh_path("./meshes/tool/a.obj"));
        assert!(is_tool_mesh_path("MESHES\\TOOL\\a.obj"));
        assert!(!is_tool_mesh_path("./meshes/robot/a.obj"));
        assert!(!is_tool_mesh_path("./tool/a.obj"));
        assert!(!is_tool_mesh(&Geometry::Box { size: [1.0; 3] }));
    }

    #[test]
    fn the_carrier_is_the_named_link_else_the_most_tool_collisions_else_the_deepest_leaf() {
        let joints = || {
            vec![
                joint("j1", JointType::Revolute, "root", "a"),
                joint("j2", JointType::Revolute, "a", "b"),
                joint("j3", JointType::Revolute, "root", "c"),
            ]
        };
        // Most tool collisions wins; the first on a tie.
        let r = robot(
            vec![
                link("root", &[ROBOT], &[]),
                link("a", &[TOOL, ROBOT], &[]),
                link("b", &[TOOL, TOOL, TOOL], &[]),
                link("c", &[TOOL, TOOL, TOOL], &[]),
            ],
            joints(),
        );
        let layout = ToolLayout::of(&r, None);
        assert_eq!(
            (layout.carrier.as_deref(), layout.source),
            (Some("b"), CarrierSource::ToolMeshes)
        );
        // No tool collisions: the deepest leaf (b at depth 2, not c at 1).
        let r = robot(
            vec![
                link("root", &[], &[]),
                link("c", &[], &[]),
                link("a", &[], &[]),
                link("b", &[], &[]),
            ],
            joints(),
        );
        let layout = ToolLayout::of(&r, None);
        assert_eq!(
            (layout.carrier.as_deref(), layout.source),
            (Some("b"), CarrierSource::DeepestLeaf)
        );
        // Named: that link, whatever carries tool meshes; a name the URDF lacks falls back.
        assert_eq!(ToolLayout::of(&r, Some("c")).carrier.as_deref(), Some("c"));
        assert_eq!(ToolLayout::of(&r, Some("c")).source, CarrierSource::Named);
        assert_eq!(
            ToolLayout::of(&r, Some("nope")).source,
            CarrierSource::DeepestLeaf
        );
        // No links: no carrier.
        let layout = ToolLayout::of(&robot(vec![], vec![]), None);
        assert_eq!((layout.carrier, layout.source), (None, CarrierSource::None));
    }

    #[test]
    fn a_named_carrier_and_its_fixed_children_are_the_tool() {
        // arm → (revolute) flange → (fixed) gripper → (fixed) finger; flange → (revolute) spin.
        let r = robot(
            vec![
                link("arm", &[ROBOT], &[ROBOT]),
                link("flange", &[], &[]),
                link("gripper", &["./grip/body.obj"], &["./grip/body.obj"]),
                link("finger", &["./grip/finger.obj"], &[]),
                link("spin", &["./grip/spin.obj"], &[]),
            ],
            vec![
                joint("a6", JointType::Revolute, "arm", "flange"),
                joint("mount", JointType::Fixed, "flange", "gripper"),
                joint("tip", JointType::Fixed, "gripper", "finger"),
                joint("rot", JointType::Revolute, "flange", "spin"),
            ],
        );
        let layout = ToolLayout::of(&r, Some("flange"));
        assert_eq!(layout.links, ["flange", "gripper", "finger"]);
        // No meshes/tool/ geometry there: all of it is the tool's, and none elsewhere.
        assert!(layout.all_geometry);
        assert!(layout.is_tool("gripper", &mesh("./grip/body.obj")));
        assert!(!layout.is_tool("spin", &mesh("./grip/spin.obj")));
        assert!(!layout.is_tool("arm", &mesh(ROBOT)));
        assert!(layout.has_tool_collisions(&r));
        // Without a payload link the dresser's folder alone marks the tool: none here.
        let default = ToolLayout::of(&r, None);
        assert!(!default.all_geometry && !default.is_tool("gripper", &mesh("./grip/body.obj")));
        assert!(!default.has_tool_collisions(&r));
    }

    #[test]
    fn a_named_carrier_with_tool_meshes_keeps_the_folder_rule() {
        // The dresser's layout: robot and tool meshes on one link; naming it changes nothing.
        let r = robot(
            vec![
                link("base", &[ROBOT], &[]),
                link("link_6", &[ROBOT, TOOL], &[ROBOT, TOOL]),
            ],
            vec![joint("a6", JointType::Revolute, "base", "link_6")],
        );
        for named in [None, Some("link_6")] {
            let layout = ToolLayout::of(&r, named);
            assert_eq!(layout.carrier.as_deref(), Some("link_6"));
            assert!(!layout.all_geometry);
            assert!(layout.is_tool("link_6", &mesh(TOOL)));
            assert!(!layout.is_tool("link_6", &mesh(ROBOT)));
            assert!(layout.has_tool_collisions(&r));
        }
    }

    #[test]
    fn fixed_subtrees_follow_fixed_joints_only() {
        let r = robot(
            vec![],
            vec![
                joint("f1", JointType::Fixed, "a", "b"),
                joint("f2", JointType::Fixed, "a", "c"),
                joint("r1", JointType::Revolute, "b", "d"),
                joint("f3", JointType::Fixed, "c", "e"),
            ],
        );
        assert_eq!(fixed_subtree(&r, "a"), ["a", "b", "c", "e"]);
        assert_eq!(fixed_subtree(&r, "d"), ["d"]);
        assert_eq!(
            root_link(&robot(vec![link("x", &[], &[])], vec![])),
            Some("x")
        );
    }
}
