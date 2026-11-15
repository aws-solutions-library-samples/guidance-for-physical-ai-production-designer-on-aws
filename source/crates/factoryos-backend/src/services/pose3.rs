//! Neutral scene poses ([`Pose3`]: metres, quaternion `[x, y, z, w]`) from the services' pose
//! conventions, with the same math the Rerun drawing used.

use factoryos_protocol::scene::Pose3;
use robo_fitting_room::compose::abc_deg_to_quat;

/// KUKA `A, B, C` (degrees; rotations about Z, Y, X) as the Robo Fitter places robots and
/// fixtures. The quaternion has `w >= 0`.
pub fn pose3_from_xyz_abc_deg(xyz: [f64; 3], abc_deg: [f64; 3]) -> Pose3 {
    Pose3 { xyz, quat_xyzw: abc_deg_to_quat(abc_deg) }
}

/// A turn about Z by `yaw_deg` (the training sampler's robots and products).
pub fn pose3_from_xyz_yaw_deg(xyz: [f64; 3], yaw_deg: f64) -> Pose3 {
    pose3_from_xyz_abc_deg(xyz, [yaw_deg, 0.0, 0.0])
}

/// URDF `xyz` / `rpy` (radians; fixed X, then Y, then Z), as the Robo Dresser's TCP frame.
pub fn pose3_from_xyz_rpy(xyz: [f64; 3], rpy: [f64; 3]) -> Pose3 {
    let [(sr, cr), (sp, cp), (sy, cy)] = rpy.map(|a| (a / 2.0).sin_cos());
    let quat_xyzw = [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ];
    Pose3 { xyz, quat_xyzw }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;
    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2};

    fn close(got: [f64; 4], want: [f64; 4]) {
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() < 1e-9, "{got:?} != {want:?}");
        }
    }

    /// The same rotation (q and -q are one).
    fn same_rotation(a: [f64; 4], b: [f64; 4]) {
        let dot: f64 = a.iter().zip(b).map(|(x, y)| x * y).sum();
        assert!((dot.abs() - 1.0).abs() < 1e-9, "{a:?} vs {b:?}");
    }

    #[test]
    fn abc_and_yaw() {
        let p = pose3_from_xyz_abc_deg([1.0, 2.0, 3.0], [90.0, 0.0, 0.0]);
        assert_eq!(p.xyz, [1.0, 2.0, 3.0]);
        close(p.quat_xyzw, [0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2]);
        assert_eq!(pose3_from_xyz_yaw_deg([0.0; 3], 90.0), Pose3 { xyz: [0.0; 3], ..p });
        // The KUKA convention is the reach module's `from_euler_angles(C, B, A)`.
        for abc in [[30.0, -20.0, 10.0], [-170.0, 45.0, 120.0], [0.0, 90.0, 0.0]] {
            let q = pose3_from_xyz_abc_deg([0.0; 3], abc).quat_xyzw;
            let n = UnitQuaternion::from_euler_angles(abc[2].to_radians(), abc[1].to_radians(), abc[0].to_radians());
            same_rotation(q, [n.i, n.j, n.k, n.w]);
            assert!(q[3] >= 0.0);
        }
    }

    #[test]
    fn rpy_as_the_dresser_had_it() {
        // The Robo Dresser's `quat_from_rpy` test value.
        close(pose3_from_xyz_rpy([0.0; 3], [0.0, 0.0, FRAC_PI_2]).quat_xyzw, [0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2]);
        // URDF rpy is nalgebra's euler angles (the reach kinematics' `pose`), with the same sign.
        for rpy in [[0.3, -1.2, 2.0], [0.0, -FRAC_PI_2, 0.0], [3.0, 0.1, -3.0]] {
            let q = pose3_from_xyz_rpy([0.0; 3], rpy).quat_xyzw;
            let n = UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]);
            close(q, [n.i, n.j, n.k, n.w]);
        }
    }
}
