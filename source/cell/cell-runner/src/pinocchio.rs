use std::ffi::{c_void, CStr, CString};
use std::os::raw::c_char;
use std::path::Path;

mod ffi {
    use std::ffi::c_void;
    use std::os::raw::c_char;

    unsafe extern "C" {
        pub fn pinocchio_load_urdf(
            urdf_path: *const c_char,
            error_msg: *mut *mut c_char,
        ) -> *mut c_void;

        pub fn pinocchio_fk(
            ctx: *mut c_void,
            joint_positions: *const f64,
            pose_out: *mut f64,
        ) -> i32;

        pub fn pinocchio_ik_solve(
            ctx: *mut c_void,
            current_joints: *const f64,
            target_pose: *const f64,
            max_iterations: u32,
            tolerance: f64,
            joints_out: *mut f64,
            error_msg: *mut *mut c_char,
        ) -> i32;

        pub fn pinocchio_free(ctx: *mut c_void);

        pub fn pinocchio_free_string(s: *mut c_char);
    }
}

pub struct PinocchioModel {
    ctx: *mut c_void,
}

unsafe impl Send for PinocchioModel {}

impl PinocchioModel {
    pub fn from_urdf(urdf_path: &Path) -> Result<Self, String> {
        let path_str = urdf_path
            .to_str()
            .ok_or_else(|| "URDF path is not valid UTF-8".to_string())?;
        let c_path = CString::new(path_str)
            .map_err(|_| "URDF path contains null byte".to_string())?;

        let mut error_msg: *mut c_char = std::ptr::null_mut();
        let ctx = unsafe { ffi::pinocchio_load_urdf(c_path.as_ptr(), &mut error_msg) };

        if ctx.is_null() {
            let msg = if !error_msg.is_null() {
                let s = unsafe { CStr::from_ptr(error_msg) }
                    .to_string_lossy()
                    .into_owned();
                unsafe { ffi::pinocchio_free_string(error_msg) };
                s
            } else {
                "unknown error loading URDF".into()
            };
            return Err(msg);
        }

        Ok(Self { ctx })
    }

    /// Forward kinematics: joint angles (radians) → TCP pose [x_mm, y_mm, z_mm, a_deg, b_deg, c_deg]
    pub fn forward_kinematics(&mut self, joint_positions: &[f64; 6]) -> [f64; 6] {
        let mut pose_out = [0.0f64; 6];
        unsafe {
            ffi::pinocchio_fk(self.ctx, joint_positions.as_ptr(), pose_out.as_mut_ptr());
        }
        pose_out
    }

    /// Inverse kinematics: given current joints (radians) and target [x_mm,y_mm,z_mm,a_deg,b_deg,c_deg],
    /// returns joint angles in radians.
    pub fn inverse_kinematics(
        &mut self,
        current_joints: &[f64; 6],
        target_pose: &[f64; 6],
        max_iterations: u32,
        tolerance: f64,
    ) -> Result<[f64; 6], String> {
        let mut joints_out = [0.0f64; 6];
        let mut error_msg: *mut c_char = std::ptr::null_mut();

        let ret = unsafe {
            ffi::pinocchio_ik_solve(
                self.ctx,
                current_joints.as_ptr(),
                target_pose.as_ptr(),
                max_iterations,
                tolerance,
                joints_out.as_mut_ptr(),
                &mut error_msg,
            )
        };

        if ret != 0 {
            let msg = if !error_msg.is_null() {
                let s = unsafe { CStr::from_ptr(error_msg) }
                    .to_string_lossy()
                    .into_owned();
                unsafe { ffi::pinocchio_free_string(error_msg) };
                s
            } else {
                "unknown IK error".into()
            };
            return Err(msg);
        }

        Ok(joints_out)
    }
}

impl Drop for PinocchioModel {
    fn drop(&mut self) {
        if !self.ctx.is_null() {
            unsafe { ffi::pinocchio_free(self.ctx) };
        }
    }
}
