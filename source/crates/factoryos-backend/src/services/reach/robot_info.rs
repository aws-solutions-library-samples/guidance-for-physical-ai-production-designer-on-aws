//! What a dressed robot is, discovered from its URDF ([`RobotInfo`], `GET /api/v1/robot/info`):
//! the chain's joints and limits, the arm's geometry at the zero pose, the reach dome
//! ([`Envelope`]), the tool ([`super::tcp`]) and the placement figures derived from them. Robot
//! placement (`reach/place`, `reach/quality`) and the chat agent take every robot fact from here.
//!
//! Derivations (`R`: the dome's reach at its sweet height, `L`: the tool length wrist → TCP,
//! ratios from [`ComfortSpec`]):
//! * sweet height: the root-frame height the wrist reaches farthest at; comfortable height band:
//!   the heights about it where `r_max ≥ 90 %` of its farthest ([`Envelope::comfort_band`]).
//! * comfortable wrist radius: `ratio_min·R .. ratio_max·R`; weld distance for a horizontal
//!   approach pointing away from the robot: `ratio_target·R + L` (the weld is `L` beyond the wrist).
//! * weld line span: `2·√(ratio_max² − ratio_target²)·R`, at most `2·tan(align_max)·ratio_target·R`.
//! * footprint radius: the farthest the base's geometry (the links fixed to the root) and the link
//!   A1 turns (its swept circle) reach from the A1 axis, collision meshes first, else visual ones.
//! * clearance from a product: `footprint + 0.1 m`; min spacing between bases:
//!   `max(ratio_min·R, 2·footprint + 0.1 m)`.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use axum::extract::{Query, State};
use axum::http::StatusCode;
use axum::routing::get;
use axum::{Json, Router};
use factoryos_protocol::api;
use factoryos_protocol::robot::{DomeInfo, JointInfo, LinkInfo, PlacementFigures, RobotInfo, RobotInfoList, RobotInfoQuery, ToolInfo};
use factoryos_protocol::scene_edit::ComfortSpec;
use nalgebra::{Isometry3, Point3};

use super::ReachContext;
use super::envelope::COMFORT_BAND_SHARE;
use super::kinematics::{JointKind, pose};
use crate::AppState;

/// The gap added to the base footprint for the clearance from a product and between two bases.
pub const FOOTPRINT_GAP_M: f64 = 0.1;
/// Used only when the URDF gives nothing to derive them from.
pub const FALLBACK_SPACING_M: f64 = 1.5;
pub const FALLBACK_CLEARANCE_M: f64 = 0.4;

pub fn router() -> Router<AppState> {
    Router::new().route(api::ROBOT_INFO, get(info))
}

fn round(v: f64, digits: i32) -> f64 {
    let k = 10f64.powi(digits);
    (v * k).round() / k
}

fn r3(v: f64) -> f64 {
    round(v, 3)
}

/// Describe the dressed robot `urdf` with the comfort bands of `spec`.
pub fn describe(ctx: &ReachContext, urdf: &Path, spec: &ComfortSpec) -> anyhow::Result<RobotInfo> {
    let kind = ctx.robot_kind(urdf)?;
    let robot = urdf_rs::read_file(urdf).map_err(|err| anyhow::anyhow!("{}: {err}", urdf.display()))?;
    let chain = &kind.chain;
    let dof = chain.dof();
    let mut info = RobotInfo {
        dressed: urdf.file_stem().unwrap_or_default().to_string_lossy().into_owned(),
        root_link: chain.root.clone(),
        tip_link: chain.tip.clone(),
        dof,
        ..RobotInfo::default()
    };
    // Joints, with the URDF's own kind and velocity limit.
    for j in chain.actuated() {
        let urdf_joint = robot.joints.iter().find(|u| u.name == j.name);
        let continuous = urdf_joint.is_some_and(|u| matches!(u.joint_type, urdf_rs::JointType::Continuous));
        let (kind_name, scale) = match j.kind {
            JointKind::Prismatic => ("prismatic", 1.0),
            _ if continuous => ("continuous", 180.0 / std::f64::consts::PI),
            _ => ("revolute", 180.0 / std::f64::consts::PI),
        };
        let velocity = urdf_joint.map(|u| u.limit.velocity).filter(|v| *v > 0.0 && v.is_finite());
        info.joints.push(JointInfo {
            name: j.name.clone(),
            kind: kind_name.to_owned(),
            lower: round(j.lower * scale, 2),
            upper: round(j.upper * scale, 2),
            max_velocity: velocity.map(|v| round(v * scale, 2)),
        });
    }
    // The arm's geometry at the zero pose.
    let q = vec![0.0; dof];
    let origins: Vec<(String, Point3<f64>)> = std::iter::once(("root".to_owned(), Point3::origin()))
        .chain(chain.actuated().enumerate().filter_map(|(i, j)| Some((j.name.clone(), Point3::from(chain.actuated_frame(&q, i)?.translation.vector)))))
        .chain(std::iter::once(("flange".to_owned(), Point3::from(chain.fk(&q).translation.vector))))
        .collect();
    for pair in origins.windows(2) {
        let d = pair[1].1 - pair[0].1;
        if pair[1].0 == "flange" && d.norm() < 1e-6 {
            continue;
        }
        info.links.push(LinkInfo { from: pair[0].0.clone(), to: pair[1].0.clone(), length_m: r3(d.norm()), horizontal_m: r3(d.x.hypot(d.y)), vertical_m: r3(d.z) });
    }
    let a1 = chain.actuated_frame(&q, 0).map(|f| f.translation.vector);
    info.a1_height_m = r3(a1.map_or(0.0, |v| v.z));
    let axis_xy = kind.envelope.as_ref().map(|e| [e.a1_origin[0], e.a1_origin[1]]).or(a1.map(|v| [v.x, v.y])).unwrap_or([0.0, 0.0]);
    if let Some(a2) = chain.actuated_frame(&q, 1).map(|f| f.translation.vector) {
        info.shoulder_height_m = Some(r3(a2.z));
        info.shoulder_offset_m = Some(r3((a2.x - axis_xy[0]).hypot(a2.y - axis_xy[1])));
    }
    info.wrist_centre = "the origin of the 5th actuated joint (A5): the point the reach dome is computed for; the wrist is the tool length behind the weld, against the approach".to_owned();
    info.wrist_zero_pose_m = chain.actuated_frame(&q, 4).map(|f| [r3(f.translation.x), r3(f.translation.y), r3(f.translation.z)]);
    // The tool.
    let tcp = &kind.tcp;
    let declared = !matches!(tcp.source.as_str(), "estimated" | "flange");
    info.tool = ToolInfo {
        tcp_link: tcp.tip_link.clone(),
        tcp_offset_m: tcp.offset.map(r3),
        tcp_axis: tcp.axis.map(r3),
        tool_length_m: r3(kind.tool_length),
        tcp_source: tcp.source.clone(),
        tcp_declared: declared,
    };
    if !declared {
        info.notes.push(format!("the TCP is {} (not declared by the robot or its tool): the tool length is approximate", tcp.source));
    }
    // The dome and the placement figures.
    let l = kind.tool_length;
    let mut p = PlacementFigures { comfort: spec.clone(), footprint_source: "none".to_owned(), ..PlacementFigures::default() };
    let mut reach_at_sweet = None;
    match kind.envelope.as_deref() {
        Some(env) => {
            let sweet = env.sweet_height();
            let r = env.r_max_at(sweet).unwrap_or_else(|| env.r_max());
            let (lo, hi) = env.comfort_band();
            info.dome = Some(DomeInfo {
                a1_axis_xy: axis_xy.map(r3),
                z_min_m: r3(env.z_min()),
                z_max_m: r3(env.z_max()),
                r_max_m: r3(env.r_max()),
                sweet_height_m: r3(sweet),
                r_min_at_sweet_m: r3(env.r_min_at(sweet).unwrap_or(0.0)),
                comfortable_height_band_m: [r3(lo), r3(hi)],
                band_share: COMFORT_BAND_SHARE,
            });
            let (zl, zh) = super::comfort::height_limits(env, spec);
            p.comfortable_wrist_radius_m = Some([r3(spec.ratio_min * r), r3(spec.ratio_max * r)]);
            p.target_wrist_radius_m = Some(r3(spec.ratio_target * r));
            p.weld_distance_m = Some(r3(spec.ratio_target * r + l));
            p.comfortable_weld_distance_m = Some([r3(spec.ratio_min * r + l), r3(spec.ratio_max * r + l)]);
            p.weld_height_band_horizontal_m = Some([r3(zl), r3(zh)]);
            p.weld_height_band_top_down_m = Some([r3(zl - l), r3(zh - l)]);
            let span = 2.0 * (spec.ratio_max.powi(2) - spec.ratio_target.powi(2)).max(0.0).sqrt() * r;
            let twist = 2.0 * spec.align_max_deg.to_radians().tan().abs() * spec.ratio_target * r;
            p.weld_line_span_m = Some(r3(span.min(twist)));
            p.rules.push(format!(
                "weld distance = ratio_target × R + L = {:.2} × {r:.2} + {l:.2} m (R: the wrist's reach at the sweet height {sweet:.2} m, L: the tool length)",
                spec.ratio_target
            ));
            p.rules.push("weld height band: the wrist heights where the reach is ≥ 90 % of the farthest, for a level tool; less L for a top-down approach (heights above the robot's mounting face: add the pedestal)".to_owned());
            reach_at_sweet = Some(r);
        }
        None => info.notes.push("no reach dome (needs 5+ joints and a vertical A1): no reach figures".to_owned()),
    }
    match footprint(&robot, urdf, chain, axis_xy) {
        Some((radius, source)) => {
            p.footprint_radius_m = Some(r3(radius));
            p.footprint_source = source.to_owned();
            p.clearance_m = Some(r3(radius + FOOTPRINT_GAP_M));
            p.rules.push(format!("clearance from a product = footprint + {FOOTPRINT_GAP_M} m (footprint: the base and A1's link, {source} geometry, from the A1 axis)"));
        }
        None => info.notes.push("the base footprint is unknown (no base or A1 link geometry loaded)".to_owned()),
    }
    p.min_spacing_m = match (reach_at_sweet, p.footprint_radius_m) {
        (Some(r), fp) => Some(r3((spec.ratio_min * r).max(fp.map_or(0.0, |f| 2.0 * f + FOOTPRINT_GAP_M)))),
        (None, Some(f)) => Some(r3(2.0 * f + FOOTPRINT_GAP_M)),
        (None, None) => None,
    };
    if p.min_spacing_m.is_some() {
        p.rules.push(format!(
            "min spacing between bases = max(ratio_min × R, 2 × footprint + {FOOTPRINT_GAP_M} m): a neighbour's base stays out of the arm's folded (cramped) zone and the bodies don't touch"
        ));
    }
    if let Some(span) = p.weld_line_span_m {
        p.rules.push(format!(
            "weld line span = 2 × √(ratio_max² − ratio_target²) × R, at most 2 × tan(align_max) × ratio_target × R = {span:.2} m: a straight weld line one robot does comfortably (robots per side ≈ the line's length / this, rounded up)"
        ));
    }
    info.placement = p;
    Ok(info)
}

/// The farthest the base (links fixed to the root) and A1's link reach from the A1 axis at `axis`,
/// and which geometry it comes from (`collision`, else `visual`).
fn footprint(robot: &urdf_rs::Robot, urdf: &Path, chain: &super::kinematics::Chain, axis: [f64; 2]) -> Option<(f64, &'static str)> {
    let a1 = chain.actuated().next()?.name.clone();
    // The links rigid with the root or with A1's child, at the zero pose.
    let mut poses: BTreeMap<String, Isometry3<f64>> = BTreeMap::from([(chain.root.clone(), Isometry3::identity())]);
    let mut queue = vec![chain.root.clone()];
    while let Some(parent) = queue.pop() {
        let at = poses[&parent];
        for joint in robot.joints.iter().filter(|j| j.parent.link == parent) {
            let rigid = matches!(joint.joint_type, urdf_rs::JointType::Fixed) || joint.name == a1;
            if rigid && !poses.contains_key(&joint.child.link) {
                poses.insert(joint.child.link.clone(), at * pose(&joint.origin));
                queue.push(joint.child.link.clone());
            }
        }
    }
    let dir = urdf.parent().unwrap_or(Path::new("."));
    for source in ["collision", "visual"] {
        let mut radius: Option<f64> = None;
        for link in robot.links.iter().filter(|l| poses.contains_key(&l.name)) {
            let geometries: Vec<(&urdf_rs::Pose, &urdf_rs::Geometry)> = if source == "collision" {
                link.collision.iter().map(|c| (&c.origin, &c.geometry)).collect()
            } else {
                link.visual.iter().map(|v| (&v.origin, &v.geometry)).collect()
            };
            for (origin, geometry) in geometries {
                let frame = poses[&link.name] * pose(origin);
                for p in points(geometry, dir) {
                    let w = frame * p;
                    let r = (w.x - axis[0]).hypot(w.y - axis[1]);
                    radius = Some(radius.map_or(r, |m| m.max(r)));
                }
            }
        }
        if let Some(r) = radius.filter(|r| *r > 1e-3) {
            return Some((r, source));
        }
    }
    None
}

/// The points bounding a URDF geometry in its own frame (mesh vertices, primitive box corners).
fn points(geometry: &urdf_rs::Geometry, dir: &Path) -> Vec<Point3<f64>> {
    let corners = |h: [f64; 3]| -> Vec<Point3<f64>> {
        (0..8).map(|i| Point3::new(if i & 1 == 0 { -h[0] } else { h[0] }, if i & 2 == 0 { -h[1] } else { h[1] }, if i & 4 == 0 { -h[2] } else { h[2] })).collect()
    };
    match geometry {
        urdf_rs::Geometry::Box { size } => corners([size[0] / 2.0, size[1] / 2.0, size[2] / 2.0]),
        urdf_rs::Geometry::Cylinder { radius, length } => corners([*radius, *radius, length / 2.0]),
        urdf_rs::Geometry::Capsule { radius, length } => corners([*radius, *radius, length / 2.0 + radius]),
        urdf_rs::Geometry::Sphere { radius } => corners([*radius; 3]),
        urdf_rs::Geometry::Mesh { filename, scale } => {
            let Some(file) = crate::services::assets::resolve_mesh(filename, Some(dir)) else { return Vec::new() };
            let s = scale.map_or([1.0; 3], |s| s.0);
            match crate::services::product_welds::mesh_triangles(&file) {
                Ok(triangles) => {
                    triangles.iter().flatten().map(|v| Point3::new(v[0] as f64 * s[0], v[1] as f64 * s[1], v[2] as f64 * s[2])).collect()
                }
                Err(err) => {
                    tracing::debug!("robot footprint: {err:#}");
                    Vec::new()
                }
            }
        }
    }
}

/// `min_spacing_m` and `clearance_m` for a placement: the request's when given (> 0), else the
/// robot's own ([`PlacementFigures`]), else [`FALLBACK_SPACING_M`] / [`FALLBACK_CLEARANCE_M`]; how
/// each was chosen goes into `notes`.
pub fn spacing_and_clearance(min_spacing_m: f64, clearance_m: f64, info: Option<&RobotInfo>, notes: &mut Vec<String>) -> (f64, f64) {
    let p = info.map(|i| &i.placement);
    let spacing = if min_spacing_m > 0.0 {
        min_spacing_m
    } else if let Some(s) = p.and_then(|p| p.min_spacing_m) {
        notes.push(format!("bases at least {s:.2} m apart: the robot's own min spacing (robot_describe)"));
        s
    } else {
        notes.push(format!("bases at least {FALLBACK_SPACING_M} m apart: a fallback, the robot's URDF gives no reach or footprint to derive it from"));
        FALLBACK_SPACING_M
    };
    let clearance = if clearance_m > 0.0 {
        clearance_m
    } else if let Some(c) = p.and_then(|p| p.clearance_m) {
        notes.push(format!("bases at least {c:.2} m from the products: the robot's footprint plus {FOOTPRINT_GAP_M} m (robot_describe)"));
        c
    } else {
        notes.push(format!("bases at least {FALLBACK_CLEARANCE_M} m from the products: a fallback, the robot's base meshes did not load (no footprint)"));
        FALLBACK_CLEARANCE_M
    };
    (spacing, clearance)
}

fn bad(status: StatusCode, message: impl Into<String>) -> (StatusCode, String) {
    (status, message.into())
}

async fn info(State(state): State<AppState>, Query(query): Query<RobotInfoQuery>) -> Result<Json<RobotInfoList>, (StatusCode, String)> {
    let (config, fitter) = (state.config.clone(), state.fitter.clone());
    tokio::task::spawn_blocking(move || {
        let scenes_dir = fitter.scenes_dir();
        let ctx = fitter.reach_context();
        let spec = ComfortSpec::default();
        let some = |s: &Option<String>| s.as_deref().map(str::trim).filter(|s| !s.is_empty()).map(str::to_owned);
        // The URDFs to describe, with the scene robots using each.
        let mut urdfs: Vec<(PathBuf, Vec<String>)> = Vec::new();
        if let Some(d) = some(&query.dressed) {
            let ok = !d.contains("..") && d.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.'));
            let catalogue = scenes_dir.parent().map(Path::to_path_buf).unwrap_or_else(|| scenes_dir.clone());
            let file = catalogue.join("urdf-dressed-robots").join(&d).join(format!("{d}.urdf"));
            if !ok || !file.is_file() {
                return Err(bad(StatusCode::NOT_FOUND, format!("no dressed robot {d} (urdf-dressed-robots/{d}/{d}.urdf): dress it first (dresser_dress)")));
            }
            urdfs.push((file, Vec::new()));
        } else if some(&query.scene_file).is_some() || some(&query.cell).is_some() {
            let resolved = super::http::resolve(&config, &scenes_dir, query.scene_file.as_deref(), query.cell.as_deref(), None)?;
            let wanted = some(&query.robot_id);
            for robot in resolved.scene.robots.iter().filter(|r| wanted.as_ref().is_none_or(|w| *w == r.robot_id)) {
                let path = resolved.scene_base.join(&robot.dressed_urdf);
                match urdfs.iter_mut().find(|(p, _)| *p == path) {
                    Some((_, ids)) => ids.push(robot.robot_id.clone()),
                    None => urdfs.push((path, vec![robot.robot_id.clone()])),
                }
            }
            if urdfs.is_empty() {
                return Err(bad(
                    StatusCode::NOT_FOUND,
                    match wanted {
                        Some(id) => format!("the scene has no robot {id}"),
                        None => "the scene has no robot: give dressed=<robot>__<tool>".to_owned(),
                    },
                ));
            }
        } else {
            return Err(bad(StatusCode::BAD_REQUEST, "give dressed=<robot>__<tool>, or scene_file / cell (and robot_id)"));
        }
        let mut list = RobotInfoList::default();
        for (urdf, ids) in urdfs {
            let mut info = describe(&ctx, &urdf, &spec).map_err(|err| bad(StatusCode::UNPROCESSABLE_ENTITY, format!("{err:#}")))?;
            info.robot_ids = ids;
            list.robots.push(info);
        }
        Ok(Json(list))
    })
    .await
    .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, format!("robot info panicked: {err}")))?
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/jam2/kr240r2900__sticktool/kr240r2900__sticktool.urdf")
    }

    #[test]
    fn the_kr240_discovered_from_its_urdf() {
        let ctx = super::super::tests_support::ctx();
        let info = describe(&ctx, &fixture(), &ComfortSpec::default()).unwrap();
        eprintln!("{}", serde_json::to_string_pretty(&info).unwrap());
        assert_eq!(info.dressed, "kr240r2900__sticktool");
        assert_eq!(info.dof, 6);
        assert_eq!(info.joints.len(), 6);
        assert_eq!(info.joints[0].name, "joint_a1");
        assert!((info.joints[1].lower + 155.0).abs() < 0.1 && (info.joints[1].upper - 35.0).abs() < 0.1, "{:?}", info.joints[1]);
        assert!(info.joints[0].max_velocity.is_some_and(|v| (v - 156.0).abs() < 1.0), "{:?}", info.joints[0]);
        assert!((info.shoulder_height_m.unwrap() - 0.86).abs() < 0.01, "{:?}", info.shoulder_height_m);
        assert!((info.shoulder_offset_m.unwrap() - 0.446).abs() < 0.01);
        assert!((info.a1_height_m - 0.86).abs() < 0.01);
        assert!(info.links.iter().any(|l| (l.length_m - 1.466).abs() < 0.01), "{:?}", info.links);
        let dome = info.dome.as_ref().expect("a dome");
        assert!(dome.r_max_m > 2.5 && dome.r_max_m < 3.5, "{dome:?}");
        let [lo, hi] = dome.comfortable_height_band_m;
        assert!(lo < dome.sweet_height_m && dome.sweet_height_m < hi, "{dome:?}");
        assert!(info.tool.tool_length_m > 0.5 && info.tool.tcp_declared, "{:?}", info.tool);
        let p = &info.placement;
        let d = p.weld_distance_m.unwrap();
        assert!((d - (0.62 * dome.r_max_m + info.tool.tool_length_m)).abs() < 0.05, "{p:?}");
        assert!(p.min_spacing_m.unwrap() > 1.0 && p.weld_line_span_m.unwrap() > 1.0, "{p:?}");
        // The trimmed fixture has no meshes: no footprint, so no clearance (placement falls back).
        assert_eq!((p.footprint_radius_m, p.footprint_source.as_str()), (None, "none"));
        let mut notes = Vec::new();
        let (s, c) = spacing_and_clearance(0.0, 0.0, Some(&info), &mut notes);
        assert_eq!((s, c), (p.min_spacing_m.unwrap(), FALLBACK_CLEARANCE_M));
        assert_eq!(spacing_and_clearance(2.0, 0.5, Some(&info), &mut notes), (2.0, 0.5));
    }

    #[test]
    fn the_footprint_comes_from_the_base_meshes() {
        // The release catalogue's dressed robot carries its meshes and collision hulls.
        let urdf = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/catalogue/urdf-dressed-robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf");
        if !urdf.is_file() {
            eprintln!("skipped: no release catalogue at {}", urdf.display());
            return;
        }
        let ctx = super::super::tests_support::ctx();
        let info = describe(&ctx, &urdf, &ComfortSpec::default()).unwrap();
        let p = &info.placement;
        eprintln!("{p:#?}");
        assert_eq!(p.footprint_source, "collision");
        let f = p.footprint_radius_m.unwrap();
        assert!(f > 0.3 && f < 1.2, "{f}");
        assert!((p.clearance_m.unwrap() - f - FOOTPRINT_GAP_M).abs() < 1e-3);
        assert!(p.min_spacing_m.unwrap() >= 2.0 * f + FOOTPRINT_GAP_M - 1e-3);
    }
}
