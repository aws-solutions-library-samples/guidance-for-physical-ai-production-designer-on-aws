//! The synthetic asset tree the server and stdio integration tests run
//! against, written into a temporary asset root. Nothing in it comes from a
//! real cell:
//!
//! ```text
//! urdf-dressed-robots/r1/r1.urdf      a small arm: World → base_link (fixed),
//!                                     base_link → link_1 → link_2 (revolute
//!                                     joint_1, joint_2), the tool on link_2
//! urdf-dressed-robots/r1/meshes/robot/arm.obj   the arm's mesh (a triangle)
//! urdf-dressed-robots/r1/meshes/tool/tool.obj   the tool's (a closed box)
//! urdf-fixture-collection/f1/f1.urdf  a fixture: World and a plate on a fixed joint
//! floor-plans/empty.obj               one line segment
//! fitted-robots/scene.yaml            one robot (R1), no fixtures, no floor plan
//! ```

// Each test crate uses only part of this module.
#![allow(dead_code)]

use std::path::Path;

/// The arm's actuated joints, in URDF declaration order.
pub const ARM_JOINTS: [&str; 2] = ["joint_1", "joint_2"];

/// `fitted-robots/scene.yaml`: one robot on the arm, without a home pose, so
/// it starts in the default one.
pub const SCENE_YAML: &str = "name: \"t\"\nrobots:\n  - robot_id: \"R1\"\n    dressed_urdf: \"../urdf-dressed-robots/r1/r1.urdf\"\n    position: { x: 0.0, y: 0.0, z: 0.0 }\n    orientation: { a: 0.0, b: 0.0, c: 0.0 }\n";

/// Lays down the asset tree above under `root`.
pub fn populate_assets(root: &Path) {
    write_arm(&root.join("urdf-dressed-robots/r1"), "r1");
    write_fixture(&root.join("urdf-fixture-collection/f1"), "f1");

    let fp_dir = root.join("floor-plans");
    std::fs::create_dir_all(&fp_dir).unwrap();
    std::fs::write(fp_dir.join("empty.obj"), "v 0 0 0\nv 1 0 0\nl 1 2\n").unwrap();

    let s_dir = root.join("fitted-robots");
    std::fs::create_dir_all(&s_dir).unwrap();
    std::fs::write(s_dir.join("scene.yaml"), SCENE_YAML).unwrap();
}

/// Writes the arm as `dir/<name>.urdf`, with its meshes under `dir/meshes/`.
pub fn write_arm(dir: &Path, name: &str) {
    write_obj_triangle(&dir.join("meshes/robot/arm.obj"));
    write_box_obj(&dir.join("meshes/tool/tool.obj"), [0.1, 0.1, 0.2]);

    let robot_mesh = mesh_elements("./meshes/robot/arm.obj");
    let tool_mesh = mesh_elements("./meshes/tool/tool.obj");
    let mut s = String::from("<?xml version=\"1.0\"?>\n<robot name=\"arm\">\n");
    for link in ["World", "base_link", "link_1"] {
        s.push_str(&format!(
            "  <link name=\"{link}\">\n{robot_mesh}  </link>\n"
        ));
    }
    s.push_str(&format!(
        "  <link name=\"link_2\">\n{robot_mesh}{tool_mesh}  </link>\n"
    ));
    s.push_str(
        "  <joint name=\"world_to_base\" type=\"fixed\">\n    <parent link=\"World\"/>\n    <child link=\"base_link\"/>\n    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n  </joint>\n",
    );
    for (joint, (parent, child)) in ARM_JOINTS
        .iter()
        .zip([("base_link", "link_1"), ("link_1", "link_2")])
    {
        s.push_str(&format!(
            "  <joint name=\"{joint}\" type=\"revolute\">\n    <parent link=\"{parent}\"/>\n    <child link=\"{child}\"/>\n    <origin xyz=\"0 0 0.5\" rpy=\"0 0 0\"/>\n    <axis xyz=\"0 0 1\"/>\n    <limit lower=\"-3.14\" upper=\"3.14\" effort=\"100\" velocity=\"1\"/>\n  </joint>\n"
        ));
    }
    s.push_str("</robot>\n");
    std::fs::write(dir.join(format!("{name}.urdf")), s).unwrap();
}

/// Writes the fixture as `dir/<name>.urdf`, with its mesh under `dir/meshes/`.
pub fn write_fixture(dir: &Path, name: &str) {
    write_obj_triangle(&dir.join("meshes/m.obj"));
    let mesh = mesh_elements("./meshes/m.obj");
    let s = format!(
        "<?xml version=\"1.0\"?>\n<robot name=\"fixture\">\n  <link name=\"World\">\n{mesh}  </link>\n  <link name=\"plate\"/>\n  <joint name=\"j_fix\" type=\"fixed\">\n    <parent link=\"World\"/>\n    <child link=\"plate\"/>\n    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n  </joint>\n</robot>\n"
    );
    std::fs::write(dir.join(format!("{name}.urdf")), s).unwrap();
}

/// A link's visual and collision elements on the mesh `filename`.
fn mesh_elements(filename: &str) -> String {
    let geometry =
        format!("      <geometry>\n        <mesh filename=\"{filename}\"/>\n      </geometry>\n");
    format!("    <visual>\n{geometry}    </visual>\n    <collision>\n{geometry}    </collision>\n")
}

/// One triangle (no volume), creating `path`'s directory.
pub fn write_obj_triangle(path: &Path) {
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    std::fs::write(path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n").unwrap();
}

/// A closed box of `size` (m), centred on the z axis and standing on z = 0,
/// its 12 triangles wound counter-clockwise from outside; creates `path`'s
/// directory.
pub fn write_box_obj(path: &Path, size: [f64; 3]) {
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    let [hx, hy, top] = [size[0] / 2.0, size[1] / 2.0, size[2]];
    let mut s = String::new();
    for (x, y, z) in [
        (-hx, -hy, 0.0),
        (hx, -hy, 0.0),
        (hx, hy, 0.0),
        (-hx, hy, 0.0),
        (-hx, -hy, top),
        (hx, -hy, top),
        (hx, hy, top),
        (-hx, hy, top),
    ] {
        s.push_str(&format!("v {x} {y} {z}\n"));
    }
    for [a, b, c] in [
        [1, 3, 2],
        [1, 4, 3],
        [5, 6, 7],
        [5, 7, 8],
        [1, 2, 6],
        [1, 6, 5],
        [4, 8, 7],
        [4, 7, 3],
        [1, 5, 8],
        [1, 8, 4],
        [2, 3, 7],
        [2, 7, 6],
    ] {
        s.push_str(&format!("f {a} {b} {c}\n"));
    }
    std::fs::write(path, s).unwrap();
}
