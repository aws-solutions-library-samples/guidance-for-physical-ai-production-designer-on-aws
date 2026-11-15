//! fos-live-scene (plan2cell/infra/viewer/fos_live_scene.py) writes the live cell's scene as JSON by
//! hand. `fixtures/live_cell_*.json` are what it publishes for a small test cell (its unit tests make
//! them: `FOS_UPDATE_FIXTURES=1 python3 -I plan2cell/infra/viewer/test_fos_live_scene.py`, and fail
//! when they are out of date). Here they must parse as the protocol's types, with no field lost: serde
//! ignores unknown fields, so a misspelt one (snake_case, a typo) would silently become its default.

use factoryos_protocol::scene::{JointFrame, NodeKind, SceneMsg};
use serde_json::Value;

fn fixture(name: &str) -> Value {
    let path = format!("{}/tests/fixtures/live_cell_{name}.json", env!("CARGO_MANIFEST_DIR"));
    serde_json::from_str(&std::fs::read_to_string(&path).unwrap_or_else(|err| panic!("{path}: {err}"))).unwrap()
}

/// Every field of `sent` is in `parsed` (the parsed value serialised again), with the same value
/// (numbers to f32 precision: radii are f32).
fn assert_kept(sent: &Value, parsed: &Value, at: &str) {
    match (sent, parsed) {
        (Value::Object(sent), Value::Object(parsed)) => {
            for (key, value) in sent {
                let Some(other) = parsed.get(key) else { panic!("{at}.{key}: not a field of the protocol's type") };
                assert_kept(value, other, &format!("{at}.{key}"));
            }
        }
        (Value::Array(sent), Value::Array(parsed)) => {
            assert_eq!(sent.len(), parsed.len(), "{at}: length");
            for (i, (a, b)) in sent.iter().zip(parsed).enumerate() {
                assert_kept(a, b, &format!("{at}[{i}]"));
            }
        }
        (Value::Number(a), Value::Number(b)) => {
            let (a, b) = (a.as_f64().unwrap(), b.as_f64().unwrap());
            assert!((a - b).abs() <= 1e-6 * a.abs().max(1.0), "{at}: {a} != {b}");
        }
        _ => assert_eq!(sent, parsed, "{at}"),
    }
}

fn parse_msg(name: &str) -> SceneMsg {
    let sent = fixture(name);
    let msg: SceneMsg = serde_json::from_value(sent.clone()).unwrap_or_else(|err| panic!("{name}: {err}"));
    assert_kept(&sent, &serde_json::to_value(&msg).unwrap(), name);
    msg
}

#[test]
fn the_live_cell_scene_parses_without_losing_fields() {
    let SceneMsg::Doc(mut doc) = parse_msg("doc") else { panic!("not a doc") };
    assert_eq!(doc.seq, 0);
    assert_eq!(doc.view.title, "Live cell · welding_cell_2_small");

    let kind = |id: &str| &doc.node(id).unwrap_or_else(|| panic!("no node {id}")).kind;
    let NodeKind::Model { model, joints, show_collision } = kind("world/robots/robot1") else { panic!("robot1") };
    assert!(joints.is_empty() && !show_collision);
    assert_eq!(model.urdf.media_type, "application/xml");
    assert_eq!(model.urdf.sha256.len(), 64);
    let obj = &model.meshes["meshes/base.obj"];
    assert_eq!(obj.asset.media_type, "model/obj");
    assert_eq!(obj.sidecars.keys().collect::<Vec<_>>(), ["base.mtl", "tex.png"]);
    // The backend's variants of an OBJ (its `Registered` reply), filled in before the doc goes out.
    assert_eq!(obj.glb.as_ref().map(|glb| glb.media_type.as_str()), Some("model/gltf-binary"));
    assert!(obj.lod.is_some());
    assert!(model.meshes["package://kr240r2900__sticktool/meshes/base_hull.stl"].glb.is_none(), "no variants made: none");
    assert!(model.meshes.contains_key("package://kr240r2900__sticktool/meshes/base_hull.stl"));
    let robot2 = doc.node("world/robots/robot2").unwrap();
    assert_eq!(robot2.pose.xyz[2], 0.5, "on its pedestal");
    assert!(matches!(kind("world/pedestals/robot2"), NodeKind::Cylinder { height, .. } if *height == 0.5));
    assert!(matches!(kind("world/fixtures/table"), NodeKind::Model { .. }));
    assert!(matches!(kind("world/static/cube"), NodeKind::Mesh { mesh, .. } if mesh.asset.media_type == "model/gltf-binary"));
    assert!(matches!(kind("world/floor_plan"), NodeKind::Mesh { mesh, .. } if mesh.glb.is_some() && mesh.lod.is_some()));
    assert!(matches!(kind("world/obstacles/boxes"), NodeKind::Boxes { half_sizes, labels, show_labels: Some(false), .. }
        if half_sizes.len() == 2 && labels.len() == 2));
    assert!(matches!(kind("world/obstacles/spheres/1"), NodeKind::Points { radius: Some(r), show_labels: Some(false), .. } if (*r - 0.2).abs() < 1e-6));
    assert!(matches!(kind("world/weld_points"), NodeKind::Points { labels, .. } if labels.len() == 4));
    // Centimetres apart: labelled on hover only (the current target by `targets`).
    assert!(matches!(kind("world/weld_points"), NodeKind::Points { show_labels: Some(false), .. }));
    assert!(matches!(kind("world/plan2cell/workpiece/welds"), NodeKind::Points { show_labels: Some(false), .. }));
    assert!(matches!(kind("world/plan2cell/labels"), NodeKind::Points { labels, show_labels: Some(true), .. } if labels.len() == 3));
    assert!(matches!(kind("world/plan2cell/paths/rest"), NodeKind::Lines { strips, .. } if !strips.is_empty()));
    assert!(matches!(kind("world/plan2cell/workpiece/panel"), NodeKind::Boxes { solid: true, .. }));
    assert!(matches!(kind("world/plan2cell/zones"), NodeKind::Boxes { labels, solid: false, .. } if labels.len() == 1));

    // The delta follows the doc and edits the annotations only.
    let SceneMsg::Delta(delta) = parse_msg("delta") else { panic!("not a delta") };
    assert!(delta.upsert.iter().all(|n| n.id.starts_with("world/plan2cell/")));
    let nodes = doc.nodes.len();
    assert!(doc.apply(&delta), "the delta follows the doc");
    assert_eq!((doc.seq, doc.nodes.len()), (1, nodes));

    // Joint frames name the robots' joints; they fold into the models.
    let sent = fixture("joints");
    let frame: JointFrame = serde_json::from_value(sent.clone()).unwrap();
    assert_kept(&sent, &serde_json::to_value(&frame).unwrap(), "joints");
    assert!(frame.t.is_some());
    assert!(doc.apply_joints(&frame));
    let NodeKind::Model { joints, .. } = &doc.node("world/robots/robot1").unwrap().kind else { panic!() };
    assert_eq!(joints.keys().collect::<Vec<_>>(), ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"]);
}
