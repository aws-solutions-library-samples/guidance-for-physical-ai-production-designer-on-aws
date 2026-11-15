//! The TCP a robot is checked with: never robots.yaml's, always from the dressed URDF.
//!
//! In order: the dressed URDF's own `tcp` frame or its tool's declared TCP (see
//! [`crate::services::tcp::planner_tcp`]); for a robot dressed before either existed, the backend's
//! tool table ([`TOOLS`]: the tool's TCP in its own frame, placed by the mount recovered from one
//! of its visuals in the dressed URDF); last an estimate from the tool's meshes (`estimated`).

use std::path::Path;

use factoryos_protocol::fitter::ToolTcp;
use nalgebra::{Point3, Vector3};

use super::kinematics::{self, pose};
use crate::services::tcp::{PlannerTcp, TcpSource, ToolCatalogues, planner_tcp};

/// A tool the backend knows: its TCP and working direction in its own (docking interface) frame,
/// and one of its visuals, to recover how it was mounted.
pub struct KnownTool {
    pub name: &'static str,
    pub tcp: [f64; 3],
    pub axis: [f64; 3],
    /// A substring of the reference visual's mesh file name, and that visual's origin in the
    /// tool frame (xyz, rpy).
    pub visual: &'static str,
    pub visual_origin: ([f64; 3], [f64; 3]),
}

/// The tool table. sticktool: the electrode tip measured on the training cell welding_cell_2_small
/// ([-0.9092, 0.0005, -0.0056] in link_6, the tool 0.2396745 m out and turned 180° about z), in
/// the tool's frame; it points along the tool's +x.
pub const TOOLS: &[KnownTool] = &[KnownTool {
    name: "sticktool",
    tcp: [0.6695255, -0.0005, -0.0056],
    axis: [1.0, 0.0, 0.0],
    visual: "0500_________K00_EGT016_1_H_RAH_300X450_SW80",
    visual_origin: ([0.0489358, -0.0038173, -0.0063185], [1.5707964, 1.5509135, 1.5707964]),
}];

/// The tool names a dressed robot file `<robot>__<tool>[__…]` may carry.
fn tool_names(urdf_path: &Path) -> Vec<String> {
    let stem = urdf_path.file_stem().unwrap_or_default().to_string_lossy();
    stem.split("__").skip(1).map(str::to_owned).collect()
}

/// The TCP of the dressed robot `urdf_path` (parsed: `robot`), in its tip link.
pub fn resolve(urdf_path: &Path, robot: &urdf_rs::Robot, tools: &ToolCatalogues) -> ToolTcp {
    let tip = kinematics::tip_link(robot).unwrap_or_default().to_owned();
    let declared = planner_tcp(urdf_path, tools).ok().flatten();
    match &declared {
        Some(PlannerTcp { tip_link, offset, source: TcpSource::Frame, axis, .. }) => {
            // the frame's own working direction: a tool may work off the flange axis (a gun at 90°)
            return ToolTcp { tip_link: tip_link.clone(), offset: *offset, axis: *axis, source: "urdf:tcp".to_owned() };
        }
        Some(PlannerTcp { tip_link, offset, source: TcpSource::Tool(path), .. }) => {
            let name = path.file_stem().unwrap_or_default().to_string_lossy();
            return ToolTcp { tip_link: tip_link.clone(), offset: *offset, axis: snap_axis(*offset), source: format!("tool:{name}") };
        }
        _ => {}
    }
    if let Some(tcp) = from_table(urdf_path, robot, &tip) {
        return tcp;
    }
    match declared {
        Some(PlannerTcp { tip_link, offset, .. }) => {
            ToolTcp { tip_link, offset, axis: snap_axis(offset), source: "estimated".to_owned() }
        }
        None => ToolTcp { tip_link: tip, offset: [0.0; 3], axis: [-1.0, 0.0, 0.0], source: "flange".to_owned() },
    }
}

/// The table's TCP for the tool `urdf_path` names, placed by the mount of its reference visual
/// on `tip`.
pub fn from_table(urdf_path: &Path, robot: &urdf_rs::Robot, tip: &str) -> Option<ToolTcp> {
    let link = robot.links.iter().find(|l| l.name == tip)?;
    for name in tool_names(urdf_path) {
        let Some(tool) = TOOLS.iter().find(|t| t.name == name) else { continue };
        let visual = link.visual.iter().find(|v| match &v.geometry {
            urdf_rs::Geometry::Mesh { filename, .. } => filename.contains(tool.visual),
            _ => false,
        })?;
        let in_tool = pose(&urdf_rs::Pose {
            xyz: urdf_rs::Vec3(tool.visual_origin.0),
            rpy: urdf_rs::Vec3(tool.visual_origin.1),
        });
        // dressed = mount · in_tool  =>  mount = dressed · in_tool⁻¹
        let mount = pose(&visual.origin) * in_tool.inverse();
        let offset = mount * Point3::from(tool.tcp);
        let axis = mount.rotation * Vector3::from(tool.axis);
        return Some(ToolTcp {
            tip_link: tip.to_owned(),
            offset: [offset.x, offset.y, offset.z],
            axis: [axis.x, axis.y, axis.z],
            source: format!("table:{}", tool.name),
        });
    }
    None
}

/// The tool axis for an offset alone: its dominant direction.
fn snap_axis(offset: [f64; 3]) -> [f64; 3] {
    let i = (0..3).max_by(|&a, &b| offset[a].abs().total_cmp(&offset[b].abs())).unwrap_or(0);
    let mut axis = [0.0; 3];
    axis[i] = if offset[i] < 0.0 { -1.0 } else { 1.0 };
    if offset.iter().all(|v| v.abs() < 1e-9) {
        return [-1.0, 0.0, 0.0];
    }
    axis
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::services::reach::kinematics::tests::kr240;

    fn fixture() -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/jam2/kr240r2900__sticktool/kr240r2900__sticktool.urdf")
    }

    #[test]
    fn the_fitter_dressed_sticktool_tcp() {
        let tcp = resolve(&fixture(), &kr240(), &ToolCatalogues::default());
        assert_eq!(tcp.source, "table:sticktool");
        assert_eq!(tcp.tip_link, "like_kr240r2900_link_6");
        for (got, want) in tcp.offset.iter().zip([-0.9092, 0.0056, 0.0005]) {
            assert!((got - want).abs() < 1e-3, "{:?}", tcp.offset);
        }
        assert!((tcp.axis[0] + 1.0).abs() < 1e-3, "{:?}", tcp.axis);
    }

    #[test]
    fn a_tcp_frame_in_the_urdf_wins() {
        let dir = std::env::temp_dir().join(format!("factoryos-reach-tcp-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("kr240r2900__sticktool.urdf");
        let text = std::fs::read_to_string(fixture()).unwrap().replace(
            "</robot>",
            r#"<link name="tcp"/><joint name="tcp_joint" type="fixed"><origin xyz="-0.9 0.01 0.0" rpy="0 0 0"/>
               <parent link="like_kr240r2900_link_6"/><child link="tcp"/></joint></robot>"#,
        );
        std::fs::write(&path, text).unwrap();
        let robot = urdf_rs::read_file(&path).unwrap();
        let tcp = resolve(&path, &robot, &ToolCatalogues::default());
        assert_eq!(tcp.source, "urdf:tcp");
        assert_eq!(tcp.offset, [-0.9, 0.01, 0.0]);
        assert_eq!(tcp.axis, [-1.0, 0.0, 0.0]);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_gun_at_90_degrees_is_checked_along_its_own_axis() {
        let dir = crate::services::tcp::tests::TestDir::new();
        let path = crate::services::tcp::tests::dressed_with_frame(dir.path(), "-0.4906 0 -0.2973", "0 -1.5707963268 0");
        let robot = urdf_rs::read_file(&path).unwrap();
        let tcp = resolve(&path, &robot, &ToolCatalogues::default());
        assert_eq!(tcp.source, "urdf:tcp");
        for (got, want) in tcp.axis.iter().zip([0.0, 0.0, -1.0]) {
            assert!((got - want).abs() < 1e-6, "{:?}", tcp.axis);
        }
    }

    #[test]
    fn tool_names_come_from_the_file_name() {
        assert_eq!(tool_names(Path::new("a/kr240r2900__sticktool.urdf")), ["sticktool"]);
        assert_eq!(tool_names(Path::new("kr240r2900.urdf")), Vec::<String>::new());
        assert_eq!(snap_axis([0.1, -0.5, 0.0]), [0.0, -1.0, 0.0]);
    }
}
