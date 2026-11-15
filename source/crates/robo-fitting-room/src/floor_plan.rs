use std::path::Path;

use crate::error::FittingRoomError;

/// Line segments, each its two end points.
pub type Segments = Vec<[[f64; 3]; 2]>;

#[derive(Debug, Clone, Default)]
pub struct FloorPlan {
    pub segments: Segments,
    pub footprint_segments: Segments,
    /// The names of the plan's layers (`# Layer:` comments) and groups (`o`, `g`), each once, in
    /// order of appearance.
    pub layers: Vec<String>,
}

/// FR-105: a layer or group whose name contains this (in any case) holds footprints, whatever the
/// manifest's `floor_plan.footprint_layers` lists.
pub const FOOTPRINT_NAME_KEYWORD: &str = "footprint";

// FR-105
/// [`load_floor_plan_with_footprint_layers`] without listed footprint layers: only the layers and
/// groups named like footprints ([`FOOTPRINT_NAME_KEYWORD`]) are footprints.
pub fn load_floor_plan(
    obj_path: &Path,
    offset: [f64; 3],
    scale: f64,
) -> Result<FloorPlan, FittingRoomError> {
    load_floor_plan_with_footprint_layers(obj_path, offset, scale, &[])
}

// FR-105
/// The floor plan in `obj_path`, an OBJ of lines: its line segments (`l` elements, a polyline of
/// n vertices making n − 1 segments), each vertex scaled by `scale` and moved by `offset`, split
/// into the general geometry and the footprints, the segments of the layers
/// ([`is_footprint_layer`]: `footprint_layers`, the manifest's `floor_plan.footprint_layers`, or
/// named like footprints). Vertices map to the world as they are, Z up (an up axis or rotation is
/// BL-27). A DXF file isn't read (BL-22): it yields no segment.
pub fn load_floor_plan_with_footprint_layers(
    obj_path: &Path,
    offset: [f64; 3],
    scale: f64,
    footprint_layers: &[String],
) -> Result<FloorPlan, FittingRoomError> {
    let content = std::fs::read_to_string(obj_path).map_err(|e| {
        FittingRoomError::FloorPlanNotFound(format!("{}: {}", obj_path.display(), e))
    })?;

    let (segments, footprint_segments, layers) =
        parse_obj_grouped(&content, offset, scale, footprint_layers);

    Ok(FloorPlan {
        segments,
        footprint_segments,
        layers,
    })
}

/// FR-105: whether `path` is a DXF file, which the floor-plan loader doesn't read (BL-22).
pub fn is_dxf(path: &Path) -> bool {
    path.extension()
        .and_then(|e| e.to_str())
        .is_some_and(|e| e.eq_ignore_ascii_case("dxf"))
}

/// FR-105: whether the layer (a `# Layer:` comment) or group (an `o` or `g` directive) `name` holds
/// footprints: it is one of `footprint_layers`, compared case-insensitively (surrounding
/// whitespace aside), or its name contains [`FOOTPRINT_NAME_KEYWORD`] in any case.
pub fn is_footprint_layer(name: &str, footprint_layers: &[String]) -> bool {
    let name = name.trim().to_lowercase();
    footprint_layers
        .iter()
        .any(|listed| listed.trim().to_lowercase() == name)
        || name.contains(FOOTPRINT_NAME_KEYWORD)
}

/// The vertex index of an OBJ index token (`v`, `v/vt`, `v/vt/vn` or `v//vn`): 1-based, or
/// negative counting back from the last vertex read; `None` for 0, a malformed token or an index
/// past the vertices read so far.
fn vertex_index(token: &str, vertex_count: usize) -> Option<usize> {
    let index: i64 = token.split('/').next()?.parse().ok()?;
    let n = vertex_count as i64;
    let zero_based = match index {
        i if i > 0 => i - 1,
        i if i < 0 => n + i,
        _ => return None,
    };
    (0..n).contains(&zero_based).then_some(zero_based as usize)
}

fn parse_obj_grouped(
    content: &str,
    offset: [f64; 3],
    scale: f64,
    footprint_layers: &[String],
) -> (Segments, Segments, Vec<String>) {
    let mut vertices: Vec<[f64; 3]> = Vec::new();
    let mut floor_segments: Segments = Vec::new();
    let mut footprint_segments: Segments = Vec::new();
    let mut layers: Vec<String> = Vec::new();
    let mut in_footprint_group = false;
    let enter = |name: &str, layers: &mut Vec<String>| {
        let name = name.trim();
        if !name.is_empty() && !layers.iter().any(|l| l == name) {
            layers.push(name.to_owned());
        }
        is_footprint_layer(name, footprint_layers)
    };

    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }

        // DXF→OBJ exports often encode layer info as `# Layer: <name>`
        // comments (no `o` group directives). Treat those identically.
        if let Some(rest) = trimmed.strip_prefix("# Layer:") {
            in_footprint_group = enter(rest, &mut layers);
            continue;
        }
        if trimmed.starts_with('#') {
            continue;
        }

        let parts: Vec<&str> = trimmed.split_whitespace().collect();
        if parts.is_empty() {
            continue;
        }

        match parts[0] {
            // An object (`o`) or group (`g`, CAD exporters' layers): the name is the rest of the
            // line.
            "o" | "g" => {
                in_footprint_group = enter(&trimmed[1..], &mut layers);
            }
            "v" => {
                if parts.len() < 4 {
                    continue;
                }
                let x = match parts[1].parse::<f64>() {
                    Ok(v) => v,
                    Err(_) => continue,
                };
                let y = match parts[2].parse::<f64>() {
                    Ok(v) => v,
                    Err(_) => continue,
                };
                let z = match parts[3].parse::<f64>() {
                    Ok(v) => v,
                    Err(_) => continue,
                };
                vertices.push([
                    x * scale + offset[0],
                    y * scale + offset[1],
                    z * scale + offset[2],
                ]);
            }
            // A line or polyline: a segment between each pair of consecutive vertices; a pair
            // with an index that doesn't resolve is skipped.
            "l" => {
                let indices: Vec<Option<usize>> = parts[1..]
                    .iter()
                    .map(|token| vertex_index(token, vertices.len()))
                    .collect();
                let target = if in_footprint_group {
                    &mut footprint_segments
                } else {
                    &mut floor_segments
                };
                for pair in indices.windows(2) {
                    if let [Some(a), Some(b)] = *pair {
                        target.push([vertices[a], vertices[b]]);
                    }
                }
            }
            _ => {}
        }
    }

    (floor_segments, footprint_segments, layers)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn layers(names: &[&str]) -> Vec<String> {
        names.iter().map(|n| n.to_string()).collect()
    }

    #[test]
    fn parse_basic_segments() {
        let content = "v 1.0 2.0 0.0\nv 4.0 5.0 0.0\nl 1 2\n";
        let (floor, footprints, _) = parse_obj_grouped(content, [0.0; 3], 1.0, &[]);
        assert_eq!(floor.len(), 1);
        assert_eq!(floor[0][0], [1.0, 2.0, 0.0]);
        assert_eq!(floor[0][1], [4.0, 5.0, 0.0]);
        assert!(footprints.is_empty());
    }

    #[test]
    fn parse_with_scale_and_offset() {
        let content = "v 1.0 2.0 0.0\nv 3.0 4.0 0.0\nl 1 2\n";
        let (floor, _, _) = parse_obj_grouped(content, [10.0, 20.0, 0.0], 2.0, &[]);
        assert_eq!(floor[0][0], [12.0, 24.0, 0.0]);
        assert_eq!(floor[0][1], [16.0, 28.0, 0.0]);
    }

    /// Three layers as `# Layer:` comments: `0`, `PLAN_LAYER_7` and `Machine Footprints`, one
    /// segment each.
    const LAYERED: &str = "\
# Layer: 0
v 0.0 0.0 0.0
v 1.0 0.0 0.0
l 1 2
# Layer: PLAN_LAYER_7
v 2.0 0.0 0.0
v 3.0 0.0 0.0
l 3 4
# Layer: Machine Footprints
v 4.0 0.0 0.0
v 5.0 0.0 0.0
l 5 6
";

    #[test]
    fn a_listed_layer_comment_holds_footprints() {
        // Listed, in another case and with surrounding whitespace: footprints, with the layer
        // named like footprints.
        let (floor, footprints, _) =
            parse_obj_grouped(LAYERED, [0.0; 3], 1.0, &layers(&[" plan_layer_7 "]));
        assert_eq!(floor.len(), 1);
        assert_eq!(floor[0][0], [0.0, 0.0, 0.0]);
        assert_eq!(footprints.len(), 2);
        assert_eq!(footprints[0][0], [2.0, 0.0, 0.0]);
        assert_eq!(footprints[1][0], [4.0, 0.0, 0.0]);
        // Not listed: only the layer named like footprints.
        let (floor, footprints, _) = parse_obj_grouped(LAYERED, [0.0; 3], 1.0, &[]);
        assert_eq!(floor.len(), 2);
        assert_eq!(footprints.len(), 1);
        assert_eq!(footprints[0][0], [4.0, 0.0, 0.0]);
        // A listed name matches the whole layer name, not a part of it.
        let (floor, footprints, _) =
            parse_obj_grouped(LAYERED, [0.0; 3], 1.0, &layers(&["PLAN_LAYER", "LAYER_7"]));
        assert_eq!((floor.len(), footprints.len()), (2, 1));
    }

    #[test]
    fn a_listed_or_footprint_named_group_holds_footprints() {
        let content = "\
v 0.0 0.0 0.0
v 1.0 0.0 0.0
l 1 2
o Station Outline 3
v 2.0 0.0 0.0
v 3.0 0.0 0.0
l 3 4
o cell_FOOTPRINT
v 4.0 0.0 0.0
v 5.0 0.0 0.0
l 5 6
o walls
l 1 2
";
        // A group's name is the rest of its line.
        let (floor, footprints, _) =
            parse_obj_grouped(content, [0.0; 3], 1.0, &layers(&["station outline 3"]));
        assert_eq!(floor.len(), 2);
        assert_eq!(footprints.len(), 2);
        assert_eq!(footprints[0][0], [2.0, 0.0, 0.0]);
        assert_eq!(footprints[1][0], [4.0, 0.0, 0.0]);
        let (floor, footprints, _) = parse_obj_grouped(content, [0.0; 3], 1.0, &[]);
        assert_eq!((floor.len(), footprints.len()), (3, 1));
    }

    #[test]
    fn footprint_layer_names_match_case_insensitively() {
        let listed = layers(&["Zone-A", "ÄUSSERE KONTUR"]);
        assert!(is_footprint_layer("zone-a", &listed));
        assert!(is_footprint_layer("  ZONE-A\t", &listed));
        assert!(is_footprint_layer("äussere kontur", &listed));
        assert!(!is_footprint_layer("zone-a2", &listed));
        assert!(!is_footprint_layer("", &listed));
        assert!(is_footprint_layer("Footprints", &[]));
        assert!(is_footprint_layer("robot_footprint_2", &listed));
        assert!(!is_footprint_layer("Zone-A", &[]));
    }

    #[test]
    fn load_floor_plan_takes_the_listed_layers() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("plan.obj");
        std::fs::write(&path, LAYERED).unwrap();
        let plan = load_floor_plan_with_footprint_layers(
            &path,
            [1.0, 0.0, 0.0],
            2.0,
            &layers(&["plan_layer_7"]),
        )
        .unwrap();
        assert_eq!((plan.segments.len(), plan.footprint_segments.len()), (1, 2));
        assert_eq!(plan.footprint_segments[0][0], [5.0, 0.0, 0.0]);
        let plan = load_floor_plan(&path, [0.0; 3], 1.0).unwrap();
        assert_eq!((plan.segments.len(), plan.footprint_segments.len()), (2, 1));
    }

    #[test]
    fn skips_comments_and_malformed() {
        let content = "# comment\nv 1.0 2.0\nv 1.0 2.0 3.0\nv 4.0 5.0 6.0\nl 1 2\n";
        let (floor, _, _) = parse_obj_grouped(content, [0.0; 3], 1.0, &[]);
        assert_eq!(floor.len(), 1);
    }

    #[test]
    fn skips_out_of_bounds_indices() {
        let content = "v 1.0 2.0 0.0\nv 3.0 4.0 0.0\nl 1 5\n";
        let (floor, _, _) = parse_obj_grouped(content, [0.0; 3], 1.0, &[]);
        assert!(floor.is_empty());
    }

    #[test]
    fn empty_input() {
        let (floor, footprints, layers) = parse_obj_grouped("", [0.0; 3], 1.0, &[]);
        assert!(floor.is_empty());
        assert!(footprints.is_empty());
        assert!(layers.is_empty());
    }

    #[test]
    fn polylines_make_a_segment_per_vertex_pair() {
        // `l 1 2 3 4`: three segments; an index past the vertices read breaks only its pairs.
        let content = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nl 1 2 3 4\nl 4 9 1 2\n";
        let (floor, _, _) = parse_obj_grouped(content, [0.0; 3], 1.0, &[]);
        assert_eq!(
            floor,
            vec![
                [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0]],
                [[1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
                [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            ]
        );
    }

    #[test]
    fn line_indices_take_texture_forms_and_count_back_when_negative() {
        // `v/vt` tokens use their vertex part; −1 is the last vertex read so far.
        let content = "v 0 0 0\nv 2 0 0\nl 1/1 2/2\nv 2 3 0\nl -2 -1\nl 0 1\nl x 1\n";
        let (floor, _, _) = parse_obj_grouped(content, [0.0; 3], 1.0, &[]);
        assert_eq!(
            floor,
            vec![
                [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
                [[2.0, 0.0, 0.0], [2.0, 3.0, 0.0]]
            ]
        );
        assert_eq!(vertex_index("3//7", 3), Some(2));
        assert_eq!(vertex_index("-3", 3), Some(0));
        assert_eq!(vertex_index("-4", 3), None);
        assert_eq!(vertex_index("4", 3), None);
    }

    #[test]
    fn g_groups_name_layers_like_o_and_layer_comments() {
        let content = "\
v 0 0 0
v 1 0 0
g walls
l 1 2
g Machine_Footprint
l 2 1
# Layer: Zone-B
l 1 2
o walls
l 1 2
";
        let (floor, footprints, layers) =
            parse_obj_grouped(content, [0.0; 3], 1.0, &layers(&["zone-b"]));
        assert_eq!((floor.len(), footprints.len()), (2, 2));
        // Each name once, in order of appearance.
        assert_eq!(layers, ["walls", "Machine_Footprint", "Zone-B"]);
    }

    #[test]
    fn dxf_files_are_told_apart() {
        assert!(is_dxf(Path::new("plans/cell.DXF")));
        assert!(is_dxf(Path::new("cell.dxf")));
        assert!(!is_dxf(Path::new("cell.obj")));
        assert!(!is_dxf(Path::new("dxf")));
    }
}
