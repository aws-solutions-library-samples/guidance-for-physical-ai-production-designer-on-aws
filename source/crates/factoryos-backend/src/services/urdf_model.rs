//! A parsed URDF for the services' logic (joint lists, root link, joint values by name), without
//! Rerun. Drawing it is the viewer's business: producers send a model's joint values by name
//! ([`factoryos_protocol::scene`]), the viewer computes the joint transforms.
//!
//! Joint order is the URDF's document order throughout, as Rerun's `UrdfTree::joints()` gave it, so
//! positional joint values (a KUKA's A1..A6, a `home_pose_deg`) map to the same joints as before.

use std::collections::{BTreeMap, HashSet};
use std::path::Path;

use anyhow::{Context as _, bail};
use urdf_rs::{Joint, JointType, Robot};

/// A URDF robot with exactly one root link and every joint between known links.
#[derive(Clone, Debug)]
pub struct UrdfModel {
    robot: Robot,
    /// Index of the root link in `robot.links`.
    root: usize,
}

impl std::str::FromStr for UrdfModel {
    type Err = anyhow::Error;

    /// Parse URDF XML.
    fn from_str(text: &str) -> anyhow::Result<Self> {
        Self::new(urdf_rs::read_from_string(text)?)
    }
}

impl UrdfModel {
    /// Read and check the URDF at `path`.
    pub fn load(path: &Path) -> anyhow::Result<Self> {
        let robot = urdf_rs::read_file(path).map_err(|err| anyhow::anyhow!("{}: {err}", path.display()))?;
        Self::new(robot).with_context(|| path.display().to_string())
    }

    /// Check `robot` as Rerun's importer did: one root link, joints between known links.
    pub fn new(robot: Robot) -> anyhow::Result<Self> {
        let links: HashSet<&str> = robot.links.iter().map(|l| l.name.as_str()).collect();
        for joint in &robot.joints {
            for (end, link) in [("parent", &joint.parent.link), ("child", &joint.child.link)] {
                if !links.contains(link.as_str()) {
                    bail!("Joint '{}' references unknown {end} link '{link}'", joint.name);
                }
            }
        }
        let children: HashSet<&str> = robot.joints.iter().map(|j| j.child.link.as_str()).collect();
        let mut roots = robot.links.iter().enumerate().filter(|(_, l)| !children.contains(l.name.as_str()));
        let root = match (roots.next(), roots.next()) {
            (None, _) => bail!("No root link found in URDF"),
            (Some((i, _)), None) => i,
            (Some(_), Some(_)) => bail!("Multiple roots in URDF"),
        };
        Ok(Self { robot, root })
    }

    /// The `<robot name=…>`.
    pub fn name(&self) -> &str {
        &self.robot.name
    }

    /// The link that is no joint's child.
    pub fn root_link(&self) -> &str {
        &self.robot.links[self.root].name
    }

    /// All joints, in document order.
    pub fn joints(&self) -> impl Iterator<Item = &Joint> {
        self.robot.joints.iter()
    }

    pub fn joint(&self, name: &str) -> Option<&Joint> {
        self.robot.joints.iter().find(|j| j.name == name)
    }

    /// The non-fixed joints in document order: what positional joint values (`home_pose_deg`, the
    /// Robo Fitter's DOF count) index. Floating / planar / spherical joints count here too, as
    /// they did in the Robo Fitter, though they take no scalar value (see [`Self::joint_map`]).
    pub fn actuated_joints(&self) -> Vec<&Joint> {
        self.joints().filter(|j| is_actuated(j)).collect()
    }

    /// The revolute and prismatic joints in document order: the Robo Dresser's sliders.
    pub fn slider_joints(&self) -> Vec<&Joint> {
        self.joints().filter(|j| matches!(j.joint_type, JointType::Revolute | JointType::Prismatic)).collect()
    }

    /// Joint values by name from `values` (radians / metres) in [`Self::actuated_joints`] order.
    /// Missing values are 0, extra ones ignored. Floating / planar / spherical joints are left
    /// out: they sit at their origin.
    pub fn joint_map(&self, values: &[f64]) -> BTreeMap<String, f64> {
        self.actuated_joints()
            .into_iter()
            .enumerate()
            .filter(|(_, j)| takes_scalar(j))
            .map(|(i, j)| (j.name.clone(), values.get(i).copied().unwrap_or(0.0)))
            .collect()
    }

    /// [`Self::joint_map`] from degrees, with prismatic joints in millimetres (a KUKA axis
    /// convention, as the Robo Fitter's home poses and IK results are).
    pub fn joint_map_from_deg(&self, values_deg: &[f64]) -> BTreeMap<String, f64> {
        let values: Vec<f64> =
            self.actuated_joints().iter().zip(values_deg).map(|(j, deg)| joint_value_from_deg(j, *deg)).collect();
        self.joint_map(&values)
    }

    /// The home pose: `home_pose_deg` (see [`Self::joint_map_from_deg`]), or else
    /// [`default_home_pose_deg`] for this robot's joint count.
    pub fn home_joint_map(&self, home_pose_deg: Option<&[f64]>) -> BTreeMap<String, f64> {
        let default_pose = default_home_pose_deg(self.actuated_joints().len());
        self.joint_map_from_deg(home_pose_deg.unwrap_or(&default_pose))
    }

    /// Joint values from an IK result: `values_deg[i]` belongs to joint `names[i]` (degrees /
    /// millimetres). Actuated joints not in `names` are 0.
    pub fn joint_map_from_named_deg(&self, names: &[String], values_deg: &[f64]) -> BTreeMap<String, f64> {
        let values: Vec<f64> = self
            .actuated_joints()
            .iter()
            .map(|j| {
                let deg = names.iter().position(|n| *n == j.name).and_then(|i| values_deg.get(i)).copied().unwrap_or(0.0);
                joint_value_from_deg(j, deg)
            })
            .collect();
        self.joint_map(&values)
    }

    /// How high the shoulder is above the root link: the z (in the root link's frame) of the
    /// second revolute joint's origin along its chain (a KUKA's A2), at every joint's zero.
    /// `None` without two revolute joints in one chain.
    pub fn shoulder_height(&self) -> Option<f64> {
        let revolute = |j: &&Joint| matches!(j.joint_type, JointType::Revolute | JointType::Continuous);
        // A2: the first revolute joint with a revolute joint above it.
        let a2 = self.joints().filter(revolute).find(|j| self.chain_to(&j.parent.link).iter().any(revolute))?;
        // Root → A2's origin: the joints' origins composed in chain order.
        let mut chain = self.chain_to(&a2.parent.link);
        chain.reverse();
        chain.push(a2);
        let (mut rot, mut pos) = (IDENTITY3, [0.0; 3]);
        for joint in chain {
            let (xyz, rpy) = (joint.origin.xyz.0, joint.origin.rpy.0);
            let offset = mul_vec(&rot, xyz);
            pos = [pos[0] + offset[0], pos[1] + offset[1], pos[2] + offset[2]];
            rot = mul_mat(&rot, &rpy_matrix(rpy));
        }
        Some(pos[2])
    }

    /// The joints from `link` up to the root link (the joint whose child `link` is first).
    fn chain_to(&self, link: &str) -> Vec<&Joint> {
        let mut chain = Vec::new();
        let mut link = link;
        while let Some(joint) = self.joints().find(|j| j.child.link == link) {
            if chain.len() > self.robot.joints.len() {
                break; // A cycle (not a tree): `new` doesn't rule it out.
            }
            chain.push(joint);
            link = &joint.parent.link;
        }
        chain
    }
}

type Mat3 = [[f64; 3]; 3];

const IDENTITY3: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// URDF's `rpy`: fixed-axis roll about x, pitch about y, yaw about z (`Rz · Ry · Rx`).
fn rpy_matrix([r, p, y]: [f64; 3]) -> Mat3 {
    let (sr, cr, sp, cp, sy, cy) = (r.sin(), r.cos(), p.sin(), p.cos(), y.sin(), y.cos());
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

fn mul_vec(m: &Mat3, v: [f64; 3]) -> [f64; 3] {
    [0, 1, 2].map(|i| m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2])
}

fn mul_mat(a: &Mat3, b: &Mat3) -> Mat3 {
    [0, 1, 2].map(|i| [0, 1, 2].map(|j| a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]))
}

/// Not a fixed joint.
pub fn is_actuated(joint: &Joint) -> bool {
    !matches!(joint.joint_type, JointType::Fixed)
}

/// Revolute or continuous: its value is an angle (radians).
pub fn is_rotational(joint: &Joint) -> bool {
    matches!(joint.joint_type, JointType::Revolute | JointType::Continuous)
}

pub fn is_prismatic(joint: &Joint) -> bool {
    matches!(joint.joint_type, JointType::Prismatic)
}

/// Revolute, continuous or prismatic: posed by one value.
pub fn takes_scalar(joint: &Joint) -> bool {
    is_rotational(joint) || is_prismatic(joint)
}

/// `(lower, upper)` of a revolute or prismatic joint, ordered even if the URDF swaps them.
pub fn limits(joint: &Joint) -> Option<(f64, f64)> {
    matches!(joint.joint_type, JointType::Revolute | JointType::Prismatic)
        .then(|| (joint.limit.lower.min(joint.limit.upper), joint.limit.upper.max(joint.limit.lower)))
}

/// A KUKA-style axis value in SI: prismatic millimetres → metres, anything else degrees → radians.
pub fn joint_value_from_deg(joint: &Joint, deg: f64) -> f64 {
    if is_prismatic(joint) { deg / 1000.0 } else { deg.to_radians() }
}

/// The KUKA stowed pose the composer falls back to (degrees), padded with 0 or cut to `dof`.
pub fn default_home_pose_deg(dof: usize) -> Vec<f64> {
    let mut pose = vec![0.0, -90.0, 90.0, 0.0, 0.0, 0.0];
    pose.resize(dof, 0.0);
    pose
}

#[cfg(test)]
mod tests {
    use super::*;

    /// world →(fixed) base →(revolute j1) l1 →(prismatic j2) l2 →(fixed tip) tool, plus a branch
    /// l1 →(continuous j3) side, and a floating joint world → loose. Written out of chain order.
    const SMALL: &str = r#"<robot name="small">
      <link name="l2"><visual><geometry><mesh filename="package://small/l2.obj"/></geometry></visual>
                     <collision><geometry><mesh filename="package://small/l2_hull.stl"/></geometry></collision></link>
      <link name="world"/>
      <link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
      <link name="l1"><visual><geometry><mesh filename="meshes/l1.obj"/></geometry></visual>
                     <collision><geometry><mesh filename="meshes/l1.obj"/></geometry></collision></link>
      <link name="tool"/>
      <link name="side"/>
      <link name="loose"/>
      <joint name="base_joint" type="fixed"><parent link="world"/><child link="base"/></joint>
      <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
        <limit lower="1.0" upper="-1.0" effort="1" velocity="1"/></joint>
      <joint name="j3" type="continuous"><parent link="l1"/><child link="side"/><axis xyz="1 0 0"/></joint>
      <joint name="j2" type="prismatic"><parent link="l1"/><child link="l2"/><axis xyz="1 0 0"/>
        <limit lower="0" upper="0.5" effort="1" velocity="1"/></joint>
      <joint name="tip" type="fixed"><parent link="l2"/><child link="tool"/></joint>
      <joint name="float" type="floating"><parent link="world"/><child link="loose"/></joint>
    </robot>"#;

    fn small() -> UrdfModel {
        SMALL.parse().unwrap()
    }

    fn names<'a>(joints: impl IntoIterator<Item = &'a Joint>) -> Vec<&'a str> {
        joints.into_iter().map(|j| j.name.as_str()).collect()
    }

    #[test]
    fn root_joints_and_kinds() {
        let m = small();
        assert_eq!(m.name(), "small");
        assert_eq!(m.root_link(), "world");
        assert_eq!(names(m.joints()), ["base_joint", "j1", "j3", "j2", "tip", "float"]);
        assert_eq!(names(m.actuated_joints()), ["j1", "j3", "j2", "float"], "document order, as UrdfTree gave");
        assert_eq!(names(m.slider_joints()), ["j1", "j2"]);
        assert_eq!(m.joint("j2").map(|j| j.child.link.as_str()), Some("l2"));
        assert!(m.joint("nope").is_none());

        let [j1, j3, j2, tip] = ["j1", "j3", "j2", "tip"].map(|n| m.joint(n).unwrap());
        assert!(is_rotational(j1) && is_rotational(j3) && !is_rotational(j2));
        assert!(is_prismatic(j2) && !is_actuated(tip) && !takes_scalar(m.joint("float").unwrap()));
        assert_eq!(limits(j1), Some((-1.0, 1.0)), "ordered");
        assert_eq!(limits(j2), Some((0.0, 0.5)));
        assert_eq!(limits(j3), None);
    }

    #[test]
    fn shoulder_height_is_a2s_origin_above_the_root() {
        // world →(fixed, up 0.2) base →(A1, up 0.5) l1 →(A2, out 0.3 and up 0.1, pitched) l2 →(A3) l3.
        let urdf = r#"<robot name="arm">
          <link name="world"/><link name="base"/><link name="l1"/><link name="l2"/><link name="l3"/>
          <joint name="mount" type="fixed"><parent link="world"/><child link="base"/><origin xyz="0 0 0.2" rpy="0 0 1.5707963"/></joint>
          <joint name="a1" type="revolute"><parent link="base"/><child link="l1"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
          <joint name="a2" type="revolute"><parent link="l1"/><child link="l2"/><origin xyz="0.3 0 0.1" rpy="1.5707963 0 0"/>
            <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
          <joint name="a3" type="revolute"><parent link="l2"/><child link="l3"/><origin xyz="0 2 0"/><axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
        </robot>"#;
        let m: UrdfModel = urdf.parse().unwrap();
        assert!((m.shoulder_height().unwrap() - 0.8).abs() < 1e-9, "{:?}", m.shoulder_height());
        // One revolute joint (j3 is continuous, on a branch beside j1): no shoulder.
        assert!((small().shoulder_height().unwrap() - 0.0).abs() < 1e-9, "j3 sits on j1's link");
        let one: UrdfModel = r#"<robot name="one"><link name="a"/><link name="b"/>
          <joint name="j" type="revolute"><parent link="a"/><child link="b"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint>
        </robot>"#.parse().unwrap();
        assert_eq!(one.shoulder_height(), None);
    }

    #[test]
    fn joint_maps() {
        let m = small();
        let map = m.joint_map(&[0.1, 0.2]);
        assert_eq!(map, BTreeMap::from([("j1".into(), 0.1), ("j3".into(), 0.2), ("j2".into(), 0.0)]), "missing at 0, no floating");

        let map = m.joint_map_from_deg(&[90.0, 180.0, 250.0, 7.0]);
        assert!((map["j1"] - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert!((map["j3"] - std::f64::consts::PI).abs() < 1e-12);
        assert!((map["j2"] - 0.25).abs() < 1e-12, "prismatic mm → m");
        assert_eq!(map.len(), 3);

        let home = m.home_joint_map(None); // [0, -90, 90, 0] over j1, j3, j2, float
        assert_eq!(home["j1"], 0.0);
        assert!((home["j3"] + std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert!((home["j2"] - 0.09).abs() < 1e-12);

        let ik = m.joint_map_from_named_deg(&["j2".into(), "j1".into()], &[100.0, -45.0]);
        assert!((ik["j1"] + std::f64::consts::FRAC_PI_4).abs() < 1e-12);
        assert!((ik["j2"] - 0.1).abs() < 1e-12);
        assert_eq!(ik["j3"], 0.0);
    }

    #[test]
    fn bad_trees_are_refused() {
        let two_roots = r#"<robot name="r"><link name="a"/><link name="b"/></robot>"#;
        assert!(two_roots.parse::<UrdfModel>().unwrap_err().to_string().contains("Multiple roots"));
        let unknown = r#"<robot name="r"><link name="a"/>
            <joint name="j" type="fixed"><parent link="a"/><child link="ghost"/></joint></robot>"#;
        assert!(unknown.parse::<UrdfModel>().unwrap_err().to_string().contains("unknown child link 'ghost'"));
        let cycle = r#"<robot name="r"><link name="a"/><link name="b"/>
            <joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint>
            <joint name="ba" type="fixed"><parent link="b"/><child link="a"/></joint></robot>"#;
        assert!(cycle.parse::<UrdfModel>().unwrap_err().to_string().contains("No root"));
    }

    #[test]
    fn the_kr240_maps_a1_to_a6_in_order() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/jam2/kr240r2900__sticktool/kr240r2900__sticktool.urdf");
        let m = UrdfModel::load(&path).unwrap();
        assert_eq!(m.root_link(), "World");
        assert_eq!(names(m.actuated_joints()), ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"]);
        assert_eq!(m.slider_joints().len(), 6);
        let home = m.home_joint_map(None);
        let want = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0].map(f64::to_radians);
        for (i, w) in want.iter().enumerate() {
            assert!((home[&format!("joint_a{}", i + 1)] - w).abs() < 1e-12);
        }
        // The reach chain walks the same joints.
        assert_eq!(crate::services::reach::kinematics::tip_link(&m.robot), Some("like_kr240r2900_link_6"));
        // This model's A2 axis sits on A1's origin, 860.6 mm above the base (A2 is offset sideways only).
        let shoulder = m.shoulder_height().unwrap();
        assert!((shoulder - 0.860625).abs() < 1e-6, "{shoulder}");
    }

    #[test]
    fn the_catalogue_urdfs_load() {
        let assets = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/catalogue");
        for rel in [
            "urdf-robot-collection/kr240r2900/kr240r2900.urdf",
            "urdf-tool-collection/sticktool/sticktool.urdf",
            "urdf-fixture-collection/weld_cube/weld_cube.urdf",
            "urdf-fixture-collection/weld_fixture_box/weld_fixture_box.urdf",
            "product-collection/weld_cube_urdf/urdf/weld_cube.urdf",
        ] {
            let m = UrdfModel::load(&assets.join(rel)).unwrap_or_else(|err| panic!("{rel}: {err:#}"));
            assert!(!m.root_link().is_empty(), "{rel}");
        }
    }
}
