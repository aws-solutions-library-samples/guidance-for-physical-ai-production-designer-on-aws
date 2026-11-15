//! A serial chain from a URDF (root link to the tip link), its forward kinematics and joint limits.
//!
//! The tip link is the child of the URDF's last movable joint (document order), as Prepare Cell's
//! converter (`convert_isaac_scene.py`) and the planners pick it.

use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    Revolute,
    Prismatic,
    Fixed,
}

#[derive(Clone, Debug)]
pub struct ChainJoint {
    pub name: String,
    pub kind: JointKind,
    /// The joint frame in its parent link's frame.
    pub origin: Isometry3<f64>,
    pub axis: Unit<Vector3<f64>>,
    pub lower: f64,
    pub upper: f64,
}

/// The joints from the root link to the tip link, in order.
#[derive(Clone, Debug)]
pub struct Chain {
    pub root: String,
    pub tip: String,
    pub joints: Vec<ChainJoint>,
}

/// A URDF `<origin>` as an isometry (rpy: roll about x, then pitch about y, then yaw about z).
pub fn pose(origin: &urdf_rs::Pose) -> Isometry3<f64> {
    let (xyz, rpy) = (origin.xyz.0, origin.rpy.0);
    Isometry3::from_parts(Translation3::new(xyz[0], xyz[1], xyz[2]), UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]))
}

/// The tip link of `robot`: the child of its last movable joint (document order).
pub fn tip_link(robot: &urdf_rs::Robot) -> Option<&str> {
    robot
        .joints
        .iter()
        .rev()
        .find(|j| !matches!(j.joint_type, urdf_rs::JointType::Fixed))
        .map(|j| j.child.link.as_str())
}

impl Chain {
    /// The chain from `robot`'s root to its tip link ([`tip_link`]).
    pub fn from_urdf(robot: &urdf_rs::Robot) -> anyhow::Result<Self> {
        let tip = tip_link(robot).ok_or_else(|| anyhow::anyhow!("{}: no movable joint", robot.name))?.to_owned();
        Self::to_link(robot, &tip)
    }

    /// The chain from `robot`'s root to `tip`.
    pub fn to_link(robot: &urdf_rs::Robot, tip: &str) -> anyhow::Result<Self> {
        let mut joints = Vec::new();
        let mut link = tip.to_owned();
        while let Some(joint) = robot.joints.iter().find(|j| j.child.link == link) {
            anyhow::ensure!(joints.len() <= robot.joints.len(), "{}: the joints form a loop", robot.name);
            let kind = match joint.joint_type {
                urdf_rs::JointType::Revolute | urdf_rs::JointType::Continuous => JointKind::Revolute,
                urdf_rs::JointType::Prismatic => JointKind::Prismatic,
                urdf_rs::JointType::Fixed => JointKind::Fixed,
                ref other => anyhow::bail!("{}: joint {} is {other:?}, not supported", robot.name, joint.name),
            };
            let continuous = matches!(joint.joint_type, urdf_rs::JointType::Continuous);
            let (lower, upper) = if continuous || joint.limit.lower >= joint.limit.upper {
                (-std::f64::consts::PI * 2.0, std::f64::consts::PI * 2.0)
            } else {
                (joint.limit.lower, joint.limit.upper)
            };
            let axis = Vector3::from(joint.axis.xyz.0);
            joints.push(ChainJoint {
                name: joint.name.clone(),
                kind,
                origin: pose(&joint.origin),
                axis: Unit::try_new(axis, 1e-12).unwrap_or(Vector3::z_axis()),
                lower,
                upper,
            });
            link = joint.parent.link.clone();
        }
        joints.reverse();
        Ok(Self { root: link, tip: tip.to_owned(), joints })
    }

    /// The actuated joints, in chain order.
    pub fn actuated(&self) -> impl Iterator<Item = &ChainJoint> {
        self.joints.iter().filter(|j| j.kind != JointKind::Fixed)
    }

    pub fn dof(&self) -> usize {
        self.actuated().count()
    }

    /// The actuated joints' `(lower, upper)` limits.
    pub fn limits(&self) -> Vec<(f64, f64)> {
        self.actuated().map(|j| (j.lower, j.upper)).collect()
    }

    pub fn joint_names(&self) -> Vec<String> {
        self.actuated().map(|j| j.name.clone()).collect()
    }

    /// Every joint's frame (before its motion) in the root frame, at `q` (actuated joints in chain
    /// order, missing ones at 0), and the tip link's pose last.
    pub fn joint_frames(&self, q: &[f64]) -> (Vec<Isometry3<f64>>, Isometry3<f64>) {
        let mut at = Isometry3::identity();
        let mut frames = Vec::with_capacity(self.joints.len());
        let mut i = 0;
        for joint in &self.joints {
            let frame = at * joint.origin;
            frames.push(frame);
            let motion = match joint.kind {
                JointKind::Fixed => Isometry3::identity(),
                JointKind::Revolute => {
                    let v = q.get(i).copied().unwrap_or(0.0);
                    i += 1;
                    Isometry3::from_parts(Translation3::identity(), UnitQuaternion::from_axis_angle(&joint.axis, v))
                }
                JointKind::Prismatic => {
                    let v = q.get(i).copied().unwrap_or(0.0);
                    i += 1;
                    Isometry3::from_parts(Translation3::from(joint.axis.into_inner() * v), UnitQuaternion::identity())
                }
            };
            at = frame * motion;
        }
        (frames, at)
    }

    /// The tip link's pose in the root frame at `q`.
    pub fn fk(&self, q: &[f64]) -> Isometry3<f64> {
        self.joint_frames(q).1
    }

    /// The frame of the `n`-th actuated joint (0-based) in the root frame at `q`.
    pub fn actuated_frame(&self, q: &[f64], n: usize) -> Option<Isometry3<f64>> {
        let (frames, _) = self.joint_frames(q);
        self.joints
            .iter()
            .zip(frames)
            .filter(|(j, _)| j.kind != JointKind::Fixed)
            .nth(n)
            .map(|(_, frame)| frame)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub fn kr240() -> urdf_rs::Robot {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/jam2/kr240r2900__sticktool/kr240r2900__sticktool.urdf");
        urdf_rs::read_file(path).unwrap()
    }

    #[test]
    fn the_kr240_chain() {
        let chain = Chain::from_urdf(&kr240()).unwrap();
        assert_eq!(chain.tip, "like_kr240r2900_link_6");
        assert_eq!(chain.root, "World");
        assert_eq!(chain.dof(), 6);
        assert_eq!(chain.joint_names()[0], "joint_a1");
        let (lo, hi) = chain.limits()[1];
        assert!((lo + 2.7052601).abs() < 1e-9 && (hi - 0.6108652).abs() < 1e-9);
        // At the zero pose the arm is stretched out level: the wrist centre (A5) is 0.44625 + 1.46625 +
        // 1.2750001 m from A1, at the shoulder's height (0.860625 m) less A4's 0.052275 m offset.
        let wrist = chain.actuated_frame(&[0.0; 6], 4).unwrap().translation.vector;
        assert!((wrist.x.hypot(wrist.y) - 3.1875001).abs() < 1e-6, "{wrist}");
        assert!((wrist.z - 0.860625).abs() < 0.06, "{wrist}");
        let tip = chain.fk(&[0.0; 6]).translation.vector;
        assert!((tip - wrist).norm() < 1e-9, "the flange origin is the wrist centre");
    }
}
