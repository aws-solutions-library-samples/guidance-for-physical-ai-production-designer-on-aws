//! The tool centre point (TCP): declared by a tool, carried into the dressed robot as a frame.
//!
//! A tool URDF declares its TCP as a link named `tcp` (or `<something>_tcp`), reached from the
//! tool's root link (its docking interface, see the catalogue's NOTE.md) through fixed joints.
//! Its origin is the working point (a stick tool's electrode tip), its axes the tool's own.
//!
//! Dressing mounts the tool on the robot's flange link; [`crate::merge`] then adds a fixed joint
//! [`TCP_JOINT`] from the flange link to a [`TCP_LINK`] link at the declared point, with the
//! mounting rotation applied, so the dressed URDF says where the tool works. Planners read it
//! back with [`dressed_tcp`].
//!
//! Robots dressed before the Dresser wrote that frame have the tool as geometry only:
//! [`mount_transform`] recovers how the tool was mounted from that geometry, so the frame can be
//! added afterwards ([`add_frame_to_urdf_text`]), and [`estimate_tool_tip`] guesses the working
//! point from the tool's meshes when the tool declares none.

use std::path::Path;

use crate::merger::{mat3_multiply, rotate_origin_around_pivot, rotation_matrix_from_rpy, rpy_from_rotation_matrix};
use crate::types::{Geometry, Joint, JointType, Link, Origin, UrdfRobot};

/// The TCP link the Dresser adds to a dressed robot.
pub const TCP_LINK: &str = "tcp";
/// The fixed joint from the flange link to [`TCP_LINK`].
pub const TCP_JOINT: &str = "tcp_joint";

/// Where a dressed robot keeps its tool meshes (the merger's output layout).
const TOOL_MESH_DIR: &str = "meshes/tool/";

/// Whether link `name` declares a TCP: `tcp`, or `<anything>_tcp`.
pub fn is_tcp_link(name: &str) -> bool {
    let name = name.to_ascii_lowercase();
    name == TCP_LINK || name.ends_with("_tcp")
}

/// `a` then `b`: the pose `b` (given in frame `a`) in `a`'s parent frame.
pub fn compose(a: &Origin, b: &Origin) -> Origin {
    let rot = rotation_matrix_from_rpy(a.rpy[0], a.rpy[1], a.rpy[2]);
    rotate_origin_around_pivot(b, &[0.0; 3], &rot, a.xyz)
}

/// The pose of `link` in the frame of the first ancestor `stop` accepts, through fixed joints only
/// (a TCP behind a movable joint is no fixed tool frame). `None` if no such ancestor is reached.
fn pose_up_to(robot: &UrdfRobot, link: &str, stop: impl Fn(&str) -> bool) -> Option<(String, Origin)> {
    let mut pose = Origin::default();
    let mut current = link.to_owned();
    for _ in 0..=robot.joints.len() {
        if stop(&current) {
            return Some((current, pose));
        }
        let joint = robot.joints.iter().find(|j| j.child == current)?;
        if joint.joint_type != JointType::Fixed {
            return None;
        }
        pose = compose(&joint.origin, &pose);
        current = joint.parent.clone();
    }
    None
}

/// The TCP a tool URDF declares, in the frame of its root link (the docking interface).
pub fn tool_tcp(tool: &UrdfRobot) -> Option<Origin> {
    let is_root = |name: &str| !tool.joints.iter().any(|j| j.child == name);
    tool.links
        .iter()
        .filter(|l| is_tcp_link(&l.name) && !is_root(&l.name))
        .find_map(|l| pose_up_to(tool, &l.name, is_root))
        .map(|(_, pose)| pose)
}

/// The child link of a robot's last movable joint (document order): the link a planner attaches
/// its tool frame to, like `scripts/convert_isaac_scene.py`'s `tip_link`.
pub fn tip_link(robot: &UrdfRobot) -> Option<&str> {
    robot
        .joints
        .iter()
        .rfind(|j| j.joint_type != JointType::Fixed)
        .map(|j| j.child.as_str())
}

/// A dressed robot's TCP frame: `(tip link, pose in the tip-link frame)`, when the URDF has a TCP
/// link fixed to its tip link (see [`tip_link`]).
pub fn dressed_tcp(dressed: &UrdfRobot) -> Option<(String, Origin)> {
    let tip = tip_link(dressed)?.to_owned();
    dressed
        .links
        .iter()
        .filter(|l| is_tcp_link(&l.name) && l.name != tip)
        .find_map(|l| pose_up_to(dressed, &l.name, |name| name == tip))
}

/// The link and joint that put a TCP at `origin` in `parent` (`TCP_LINK`, or `TCP_LINK` with a
/// suffix when the robot already has a link of that name).
pub fn tcp_frame(robot: &UrdfRobot, parent: &str, origin: Origin) -> (Link, Joint) {
    let taken = |name: &str| robot.links.iter().any(|l| l.name == name) || robot.joints.iter().any(|j| j.name == name);
    let mut link = TCP_LINK.to_owned();
    let mut joint = TCP_JOINT.to_owned();
    let mut n = 2;
    while taken(&link) || taken(&joint) {
        link = format!("{TCP_LINK}_{n}");
        joint = format!("{TCP_JOINT}_{n}");
        n += 1;
    }
    (
        Link { name: link.clone(), inertial: None, visuals: Vec::new(), collisions: Vec::new() },
        Joint {
            name: joint,
            joint_type: JointType::Fixed,
            parent: parent.to_owned(),
            child: link,
            origin,
            axis: [0.0, 0.0, 1.0],
            limit: None,
        },
    )
}

/// How `tool` was mounted on `flange` (a dressed robot's flange link): the transform the merger
/// applied to every tool element, recovered by matching the tool's visuals (by mesh file name) to
/// the flange link's. `None` unless at least one visual matches and all matches agree.
pub fn mount_transform(tool: &UrdfRobot, flange: &Link) -> Option<Origin> {
    let file_name = |g: &Geometry| match g {
        Geometry::Mesh { filename, .. } => Path::new(filename).file_name().map(|f| f.to_string_lossy().into_owned()),
        _ => None,
    };
    let pairs: Vec<(&Origin, &Origin)> = tool
        .links
        .iter()
        .flat_map(|l| &l.visuals)
        .filter_map(|tv| {
            let name = file_name(&tv.geometry)?;
            let dv = flange.visuals.iter().find(|dv| file_name(&dv.geometry).as_deref() == Some(name.as_str()))?;
            Some((&tv.origin, &dv.origin))
        })
        .collect();
    let (tool_origin, dressed_origin) = pairs.first()?;
    // dressed = M · tool  =>  R_M = R_d · R_tᵀ,  t_M = x_d − R_M · x_t.
    let r_t = rotation_matrix_from_rpy(tool_origin.rpy[0], tool_origin.rpy[1], tool_origin.rpy[2]);
    let r_d = rotation_matrix_from_rpy(dressed_origin.rpy[0], dressed_origin.rpy[1], dressed_origin.rpy[2]);
    let r_m = mat3_multiply(&r_d, &transpose(&r_t));
    let rotated = apply(&r_m, tool_origin.xyz);
    let mount = Origin {
        xyz: [0, 1, 2].map(|i| dressed_origin.xyz[i] - rotated[i]),
        rpy: rpy_from_rotation_matrix(&r_m),
    };
    let agrees = pairs.iter().all(|(t, d)| {
        let placed = compose(&mount, t);
        let r_p = rotation_matrix_from_rpy(placed.rpy[0], placed.rpy[1], placed.rpy[2]);
        let r_d = rotation_matrix_from_rpy(d.rpy[0], d.rpy[1], d.rpy[2]);
        let position = (0..3).all(|i| (placed.xyz[i] - d.xyz[i]).abs() < 1e-4);
        let rotation = (0..3).all(|i| (0..3).all(|j| (r_p[i][j] - r_d[i][j]).abs() < 1e-4));
        position && rotation
    });
    agrees.then_some(mount)
}

/// The URDF text `urdf` with a fixed TCP frame (link + joint) at `origin` in `parent` added before
/// its closing `</robot>`, the rest of the file untouched. `None` if there is no `</robot>`.
pub fn add_frame_to_urdf_text(urdf: &str, link: &Link, joint: &Joint) -> Option<String> {
    let end = urdf.rfind("</robot>")?;
    let frame = format!(
        "  <!-- Tool centre point: the tool's declared working point (FactoryOS Robo Dresser). -->\n  <link name=\"{}\"/>\n  <joint name=\"{}\" type=\"fixed\">\n    {}\n    <parent link=\"{}\"/>\n    <child link=\"{}\"/>\n  </joint>\n",
        link.name,
        joint.name,
        origin_element(&joint.origin),
        joint.parent,
        joint.child,
    );
    Some(format!("{}{frame}{}", &urdf[..end], &urdf[end..]))
}

/// A guess at a dressed robot's tool working point, in the tip-link frame, for a tool that
/// declares no TCP: the tool mesh vertices (those under `meshes/tool/`) furthest out along the
/// tool's direction (flange to tool centroid), averaged over the last 5 mm. `urdf_dir` is the
/// dressed URDF's folder. `None` without readable tool meshes on the tip link.
pub fn estimate_tool_tip(dressed: &UrdfRobot, urdf_dir: &Path) -> Option<[f64; 3]> {
    let tip = tip_link(dressed)?;
    let link = dressed.links.iter().find(|l| l.name == tip)?;
    let mut vertices = Vec::new();
    for visual in &link.visuals {
        let Geometry::Mesh { filename, scale } = &visual.geometry else { continue };
        let clean = filename.trim_start_matches("./");
        if !clean.starts_with(TOOL_MESH_DIR) {
            continue;
        }
        let Ok(verts) = crate::mesh_reader::read_mesh_vertices(&urdf_dir.join(clean)) else { continue };
        let scaled: Vec<[f64; 3]> = verts.iter().map(|v| [0, 1, 2].map(|i| v[i] * scale[i])).collect();
        vertices.extend(crate::merger::apply_origin_to_vertices(&scaled, &visual.origin));
    }
    farthest_along(&vertices, centroid_direction(&vertices)?)
}

/// The unit direction from the origin to the centroid of `vertices`. `None` if empty or at the origin.
fn centroid_direction(vertices: &[[f64; 3]]) -> Option<[f64; 3]> {
    if vertices.is_empty() {
        return None;
    }
    unit(crate::mesh_reader::compute_centroid(vertices))
}

fn unit(v: [f64; 3]) -> Option<[f64; 3]> {
    let len = (0..3).map(|i| v[i] * v[i]).sum::<f64>().sqrt();
    (len > 1e-9).then(|| v.map(|c| c / len))
}

/// The tip of `vertices` along the unit direction `dir`: the vertices furthest out along it,
/// averaged over the last 5 mm. `None` without vertices.
pub fn farthest_along(vertices: &[[f64; 3]], dir: [f64; 3]) -> Option<[f64; 3]> {
    if vertices.is_empty() {
        return None;
    }
    let proj = |v: &[f64; 3]| (0..3).map(|i| v[i] * dir[i]).sum::<f64>();
    let max = vertices.iter().map(proj).fold(f64::NEG_INFINITY, f64::max);
    let near: Vec<[f64; 3]> = vertices.iter().filter(|v| proj(v) >= max - 0.005).copied().collect();
    Some(crate::mesh_reader::compute_centroid(&near))
}

/// A tool URDF's root link (its docking interface): the link no joint has as its child.
pub fn tool_root(tool: &UrdfRobot) -> Option<&str> {
    tool.links
        .iter()
        .find(|l| !tool.joints.iter().any(|j| j.child == l.name))
        .map(|l| l.name.as_str())
}

/// The vertices of a tool's geometry in its root-link frame: every link fixed to the root, its
/// collision geometry (its visuals where it has none). Meshes are read relative to `urdf_dir`,
/// primitives contribute their extreme points. Unreadable meshes are skipped.
pub fn tool_geometry_vertices(tool: &UrdfRobot, urdf_dir: &Path) -> Vec<[f64; 3]> {
    let Some(root) = tool_root(tool) else { return Vec::new() };
    let mut vertices = Vec::new();
    for link in &tool.links {
        let Some((_, link_pose)) = pose_up_to(tool, &link.name, |name| name == root) else { continue };
        let shapes: Vec<(&Origin, &Geometry)> = if link.collisions.is_empty() {
            link.visuals.iter().map(|v| (&v.origin, &v.geometry)).collect()
        } else {
            link.collisions.iter().map(|c| (&c.origin, &c.geometry)).collect()
        };
        for (origin, geometry) in shapes {
            let local: Vec<[f64; 3]> = match geometry {
                Geometry::Mesh { filename, scale } => {
                    let clean = filename.trim_start_matches("file://").trim_start_matches("./");
                    let Ok(verts) = crate::mesh_reader::read_mesh_vertices(&urdf_dir.join(clean)) else { continue };
                    verts.iter().map(|v| [0, 1, 2].map(|i| v[i] * scale[i])).collect()
                }
                Geometry::Box { size } => {
                    let h = size.map(|s| s / 2.0);
                    (0..8).map(|c| [0, 1, 2].map(|i| if c & (1 << i) == 0 { -h[i] } else { h[i] })).collect()
                }
                Geometry::Cylinder { radius, length } => (0..16)
                    .flat_map(|k| {
                        let a = f64::from(k) * std::f64::consts::TAU / 16.0;
                        let (x, y) = (radius * a.cos(), radius * a.sin());
                        [[x, y, -length / 2.0], [x, y, length / 2.0]]
                    })
                    .collect(),
                Geometry::Sphere { radius } => {
                    let r = *radius;
                    vec![[r, 0.0, 0.0], [-r, 0.0, 0.0], [0.0, r, 0.0], [0.0, -r, 0.0], [0.0, 0.0, r], [0.0, 0.0, -r]]
                }
            };
            let placed = crate::merger::apply_origin_to_vertices(&local, origin);
            vertices.extend(crate::merger::apply_origin_to_vertices(&placed, &link_pose));
        }
    }
    vertices
}

/// The direction a tool works in, in its root-link frame, when nothing better is known: the
/// dominant axis (±x, ±y or ±z) of the direction from the docking interface to the centroid of its
/// geometry. `None` without geometry.
pub fn tool_working_axis(vertices: &[[f64; 3]]) -> Option<[f64; 3]> {
    let dir = centroid_direction(vertices)?;
    let i = (0..3).max_by(|&a, &b| dir[a].abs().total_cmp(&dir[b].abs()))?;
    let mut axis = [0.0; 3];
    axis[i] = dir[i].signum();
    Some(axis)
}

/// The tool tip along `axis` (unit, root-link frame): the farthest point of the tool's collision
/// geometry along it (see [`tool_geometry_vertices`], [`farthest_along`]).
pub fn tool_tip(tool: &UrdfRobot, urdf_dir: &Path, axis: [f64; 3]) -> Option<[f64; 3]> {
    farthest_along(&tool_geometry_vertices(tool, urdf_dir), unit(axis)?)
}

/// The inverse of the pose `o`.
pub fn invert(o: &Origin) -> Origin {
    let rt = transpose(&rotation(o));
    let back = apply(&rt, o.xyz);
    Origin { xyz: back.map(|v| -v), rpy: rpy_from_rotation_matrix(&rt) }
}

/// The rotation matrix of the pose `o`.
pub fn rotation(o: &Origin) -> [[f64; 3]; 3] {
    rotation_matrix_from_rpy(o.rpy[0], o.rpy[1], o.rpy[2])
}

/// The vector `v` rotated by the pose `o` (no translation).
pub fn rotate(o: &Origin, v: [f64; 3]) -> [f64; 3] {
    apply(&rotation(o), v)
}

/// Whether two poses are the same (positions within `tol` m, rotation matrices within `tol`).
pub fn same_pose(a: &Origin, b: &Origin, tol: f64) -> bool {
    let (ra, rb) = (rotation(a), rotation(b));
    (0..3).all(|i| (a.xyz[i] - b.xyz[i]).abs() < tol) && (0..3).all(|i| (0..3).all(|j| (ra[i][j] - rb[i][j]).abs() < tol))
}

/// The TCP link fixed (through fixed joints) to `base`, and its pose in `base`.
fn tcp_on<'a>(robot: &'a UrdfRobot, base: &str) -> Option<(&'a Link, Origin)> {
    robot
        .links
        .iter()
        .filter(|l| is_tcp_link(&l.name) && l.name != base)
        .find_map(|l| pose_up_to(robot, &l.name, |name| name == base).map(|(_, pose)| (l, pose)))
}

/// The URDF text `urdf` (of `robot`) with its TCP frame on `base` at `pose` (in the `base` frame):
/// the origin of the TCP link's joint rewritten, or, without a TCP link on `base`, a fixed TCP frame
/// added ([`tcp_frame`], [`add_frame_to_urdf_text`]). The rest of the file is untouched. `None` if
/// the text has no such joint element (or no `</robot>`).
pub fn set_tcp_frame_in_urdf_text(urdf: &str, robot: &UrdfRobot, base: &str, pose: Origin) -> Option<String> {
    let Some((link, _)) = tcp_on(robot, base) else {
        let (link, joint) = tcp_frame(robot, base, pose);
        return add_frame_to_urdf_text(urdf, &link, &joint);
    };
    let joint = robot.joints.iter().find(|j| j.child == link.name)?;
    // The pose of the joint's parent in `base` (the TCP may hang off an intermediate fixed link).
    let (_, parent_pose) = pose_up_to(robot, &joint.parent, |name| name == base)?;
    let local = compose(&invert(&parent_pose), &pose);
    replace_joint_origin(urdf, &joint.name, &local)
}

/// `<origin xyz="…" rpy="…"/>`, numbers to 10 decimals without trailing zeros.
fn origin_element(o: &Origin) -> String {
    let num = |v: f64| {
        let s = format!("{v:.10}");
        let s = s.trim_end_matches('0').trim_end_matches('.');
        if s == "-0" || s.is_empty() { "0".to_owned() } else { s.to_owned() }
    };
    format!(
        "<origin xyz=\"{} {} {}\" rpy=\"{} {} {}\"/>",
        num(o.xyz[0]),
        num(o.xyz[1]),
        num(o.xyz[2]),
        num(o.rpy[0]),
        num(o.rpy[1]),
        num(o.rpy[2])
    )
}

/// The value of attribute `name` in the start tag `tag` (`<joint name="a" …>`).
fn attribute<'a>(tag: &'a str, name: &str) -> Option<&'a str> {
    let mut rest = tag;
    while let Some(at) = rest.find(name) {
        let before = rest[..at].chars().last();
        let after = rest[at + name.len()..].trim_start();
        rest = &rest[at + name.len()..];
        if !before.is_some_and(char::is_whitespace) {
            continue;
        }
        let Some(after) = after.strip_prefix('=') else { continue };
        let after = after.trim_start();
        let quote = after.chars().next()?;
        if quote != '"' && quote != '\'' {
            continue;
        }
        let value = &after[1..];
        return value.find(quote).map(|end| &value[..end]);
    }
    None
}

/// `urdf` with the `<origin>` of joint `joint` replaced by `origin` (inserted if it has none).
fn replace_joint_origin(urdf: &str, joint: &str, origin: &Origin) -> Option<String> {
    let mut from = 0;
    while let Some(found) = urdf[from..].find("<joint") {
        let start = from + found;
        let tag_end = start + urdf[start..].find('>')?;
        from = tag_end;
        let tag = &urdf[start..=tag_end];
        if tag.ends_with("/>") || attribute(tag, "name") != Some(joint) {
            continue;
        }
        let body_end = tag_end + urdf[tag_end..].find("</joint>")?;
        let body = &urdf[tag_end + 1..body_end];
        let element = origin_element(origin);
        return Some(match body.find("<origin") {
            Some(o) => {
                let o_start = tag_end + 1 + o;
                let close = urdf[o_start..].find('>')? + o_start;
                let o_end = if urdf[..=close].ends_with("/>") {
                    close + 1
                } else {
                    o_start + urdf[o_start..].find("</origin>")? + "</origin>".len()
                };
                format!("{}{element}{}", &urdf[..o_start], &urdf[o_end..])
            }
            None => format!("{}\n    {element}{}", &urdf[..=tag_end], &urdf[tag_end + 1..]),
        });
    }
    None
}


fn transpose(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    [0, 1, 2].map(|i| [0, 1, 2].map(|j| m[j][i]))
}

fn apply(m: &[[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [0, 1, 2].map(|i| m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::parse_urdf_bytes;

    const TOOL: &str = r#"<robot name="">
  <link name="World">
    <visual><origin xyz="0.6 -0.001 -0.005" rpy="1.5707964 0.795223 -1.5707964"/><geometry><mesh filename="./meshes/a.obj"/></geometry></visual>
    <visual><origin xyz="0.05 -0.004 -0.006" rpy="1.5707964 1.5509135 1.5707964"/><geometry><mesh filename="./meshes/b.obj"/></geometry></visual>
  </link>
  <link name="tcp"/>
  <joint name="tcp_joint" type="fixed">
    <origin xyz="0.6695255 -0.0005 -0.0056" rpy="0 0 0"/>
    <parent link="World"/>
    <child link="tcp"/>
  </joint>
</robot>"#;

    #[test]
    fn tool_declares_its_tcp() {
        let tool = parse_urdf_bytes(TOOL.as_bytes(), "tool").unwrap();
        let tcp = tool_tcp(&tool).unwrap();
        assert_eq!(tcp.xyz, [0.6695255, -0.0005, -0.0056]);
        let bare = parse_urdf_bytes(br#"<robot name=""><link name="World"/></robot>"#, "bare").unwrap();
        assert!(tool_tcp(&bare).is_none());
    }

    #[test]
    fn mount_is_recovered_from_the_tool_geometry() {
        let tool = parse_urdf_bytes(TOOL.as_bytes(), "tool").unwrap();
        // The tool turned 270° about its axis, then 180° about z, 0.24 m out from the flange origin.
        let mount = Origin { xyz: [-0.2396745, 0.0, 0.0], rpy: [-std::f64::consts::FRAC_PI_2, 0.0, std::f64::consts::PI] };
        let visuals = tool.links[0]
            .visuals
            .iter()
            .map(|v| {
                let mut v = v.clone();
                v.origin = compose(&mount, &v.origin);
                if let Geometry::Mesh { filename, .. } = &mut v.geometry {
                    *filename = filename.replace("./meshes/", "./meshes/tool/");
                }
                v
            })
            .collect();
        let flange = Link { name: "link_6".into(), inertial: None, visuals, collisions: Vec::new() };
        let recovered = mount_transform(&tool, &flange).unwrap();
        let tcp = compose(&recovered, &tool_tcp(&tool).unwrap());
        for (got, want) in tcp.xyz.iter().zip([-0.9092, 0.0056, 0.0005]) {
            assert!((got - want).abs() < 1e-6, "{:?}", tcp.xyz);
        }
    }

    #[test]
    fn frame_text_round_trips() {
        let robot = parse_urdf_bytes(
            br#"<robot name="r"><link name="base"/><link name="link_6"/>
<joint name="a6" type="revolute"><origin xyz="0 0 1" rpy="0 0 0"/><parent link="base"/><child link="link_6"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
</robot>"#,
            "r",
        )
        .unwrap();
        assert!(dressed_tcp(&robot).is_none());
        let (link, joint) = tcp_frame(&robot, "link_6", Origin { xyz: [-0.9092, 0.0056, 0.0005], rpy: [0.0; 3] });
        let text = std::str::from_utf8(&crate::serializer::serialize_urdf(&robot)).unwrap().to_owned();
        let with_tcp = add_frame_to_urdf_text(&text, &link, &joint).unwrap();
        let reparsed = parse_urdf_bytes(with_tcp.as_bytes(), "r").unwrap();
        let (tip, pose) = dressed_tcp(&reparsed).unwrap();
        assert_eq!(tip, "link_6");
        assert_eq!(pose.xyz, [-0.9092, 0.0056, 0.0005]);
    }

    #[test]
    fn setting_the_tcp_rewrites_its_joint_and_nothing_else() {
        let tool = parse_urdf_bytes(TOOL.as_bytes(), "tool").unwrap();
        let pose = Origin { xyz: [0.7, 0.001, -0.002], rpy: [0.0, 0.1, 0.0] };
        let text = TOOL.replace("<link name=\"tcp\"/>", "<!-- the electrode tip -->\n  <link name=\"tcp\"/>");
        let updated = set_tcp_frame_in_urdf_text(&text, &tool, "World", pose).unwrap();
        assert!(updated.contains("<!-- the electrode tip -->"), "comments stay");
        assert!(updated.contains(r#"<origin xyz="0.7 0.001 -0.002" rpy="0 0.1 0"/>"#), "{updated}");
        let reparsed = parse_urdf_bytes(updated.as_bytes(), "tool").unwrap();
        assert!(same_pose(&tool_tcp(&reparsed).unwrap(), &pose, 1e-9));
        assert_eq!(reparsed.links.len(), tool.links.len(), "no second tcp link");
        assert_eq!(reparsed.links[0].visuals, tool.links[0].visuals, "the geometry is untouched");
        // Setting the same pose again changes nothing.
        assert_eq!(set_tcp_frame_in_urdf_text(&updated, &reparsed, "World", pose).unwrap(), updated);
    }

    #[test]
    fn setting_the_tcp_of_a_tool_without_one_adds_a_frame() {
        let bare = r#"<robot name="t"><link name="World"/></robot>"#;
        let tool = parse_urdf_bytes(bare.as_bytes(), "t").unwrap();
        let pose = Origin { xyz: [0.3, 0.0, 0.0], rpy: [0.0; 3] };
        let updated = set_tcp_frame_in_urdf_text(bare, &tool, "World", pose).unwrap();
        let reparsed = parse_urdf_bytes(updated.as_bytes(), "t").unwrap();
        assert!(same_pose(&tool_tcp(&reparsed).unwrap(), &pose, 1e-9));
    }

    #[test]
    fn a_tcp_behind_an_intermediate_fixed_link_is_set_in_the_root_frame() {
        let text = r#"<robot name="t"><link name="World"/><link name="holder"/><link name="tcp"/>
<joint name="holder_joint" type="fixed"><origin xyz="0.1 0 0" rpy="0 0 1.5707963267948966"/><parent link="World"/><child link="holder"/></joint>
<joint name="tcp_joint" type="fixed"><origin xyz="0.2 0 0" rpy="0 0 0"/><parent link="holder"/><child link="tcp"/></joint>
</robot>"#;
        let tool = parse_urdf_bytes(text.as_bytes(), "t").unwrap();
        assert!((tool_tcp(&tool).unwrap().xyz[1] - 0.2).abs() < 1e-9);
        let pose = Origin { xyz: [0.5, 0.05, 0.0], rpy: [0.2, 0.0, 0.0] };
        let updated = set_tcp_frame_in_urdf_text(text, &tool, "World", pose).unwrap();
        let reparsed = parse_urdf_bytes(updated.as_bytes(), "t").unwrap();
        assert!(same_pose(&tool_tcp(&reparsed).unwrap(), &pose, 1e-9), "{:?}", tool_tcp(&reparsed));
        assert_eq!(reparsed.joints[0].origin, tool.joints[0].origin, "the holder stays");
    }

    #[test]
    fn the_tool_tip_is_the_far_end_of_its_collision_geometry() {
        // A 0.6 m bar along +x (collision), a long visual-only part that must not count.
        let text = r#"<robot name="t"><link name="World">
<visual><origin xyz="0.5 0 0" rpy="0 0 0"/><geometry><box size="2 0.01 0.01"/></geometry></visual>
<collision><origin xyz="0.3 0 0.01" rpy="0 0 0"/><geometry><box size="0.6 0.02 0.02"/></geometry></collision>
</link></robot>"#;
        let tool = parse_urdf_bytes(text.as_bytes(), "t").unwrap();
        let vertices = tool_geometry_vertices(&tool, Path::new("."));
        assert_eq!(tool_working_axis(&vertices), Some([1.0, 0.0, 0.0]));
        let tip = tool_tip(&tool, Path::new("."), [1.0, 0.0, 0.0]).unwrap();
        for (got, want) in tip.iter().zip([0.6, 0.0, 0.01]) {
            assert!((got - want).abs() < 1e-9, "{tip:?}");
        }
    }

    #[test]
    fn inverting_a_pose_undoes_it() {
        let a = Origin { xyz: [0.1, -0.2, 0.3], rpy: [0.4, -0.5, 0.6] };
        assert!(same_pose(&compose(&a, &invert(&a)), &Origin::default(), 1e-9));
        assert!(same_pose(&compose(&invert(&a), &a), &Origin::default(), 1e-9));
    }
}
