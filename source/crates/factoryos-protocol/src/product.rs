//! Product & Task: the part a cell is to produce, and what is to be done to it, defined before the
//! cell exists. A product lives in `<products dir>/<name>/product.json`; the task-author agent
//! turns it into a cell's task program (`agentic_planner/tasks/<name>.json`).
//!
//! A product has tasks. A task is one process (spot welding, seam welding, gluing, screwing, …)
//! done with one tool, with that process's parameters; its operations are the targets on the part:
//! points (a spot, a screw, a hole) or paths (a seam, a glue bead), each with an optional parameter
//! override. [`PROCESSES`] describes the known processes; tasks of other processes still load.
//!
//! Positions and normals are in the product's own frame: metres, z up. A box's frame has its
//! origin at the centre of its bottom face.

use std::collections::BTreeMap;

/// Process parameters by key (see [`Param`]); numbers, but any JSON value is kept.
pub type Params = BTreeMap<String, serde_json::Value>;

/// A product: its geometry and its tasks.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Product {
    /// Folder name under the products dir, and the task program's name.
    pub name: String,
    #[serde(default)]
    pub description: String,
    pub geometry: Geometry,
    #[serde(default)]
    pub tasks: Vec<Task>,
}

impl Default for Product {
    fn default() -> Self {
        Self {
            name: String::new(),
            description: String::new(),
            geometry: Geometry::Box { size: [0.6, 0.4, 0.3] },
            tasks: Vec::new(),
        }
    }
}

/// The part's shape. New kinds are new variants; files written with older ones keep loading.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum Geometry {
    /// A box `size` (x, y, z) metres, standing on the frame's origin.
    Box { size: [f32; 3] },
    /// A mesh file (OBJ, STL, GLB/glTF), relative to the product's folder or absolute.
    Mesh {
        path: String,
        #[serde(default = "unit_scale")]
        scale: f32,
    },
    /// A URDF (with its meshes) under the product's folder, e.g. `urdf/part.urdf`: uploaded as a
    /// zip, or copied from the catalogue's fixtures. The product's frame is the URDF's root link's.
    Urdf { path: String },
}

impl Geometry {
    /// `box`, `mesh` or `urdf`, as in the JSON's `kind`.
    pub fn kind(&self) -> &'static str {
        match self {
            Geometry::Box { .. } => "box",
            Geometry::Mesh { .. } => "mesh",
            Geometry::Urdf { .. } => "urdf",
        }
    }
}

fn unit_scale() -> f32 {
    1.0
}

/// One process done with one tool, e.g. "the frame's spot welds, with the spot welding gun".
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Task {
    pub name: String,
    /// A [`Process::key`], e.g. `spotWeld`.
    pub process: String,
    /// The tool the process needs: a tool of the Robo Dresser catalogue (`urdf-tool-collection/`)
    /// or a description of one.
    #[serde(default)]
    pub tool: String,
    /// The process parameters for every operation of the task.
    #[serde(default)]
    pub params: Params,
    #[serde(default)]
    pub operations: Vec<Operation>,
}

/// One target of a task on the part.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Operation {
    pub id: String,
    pub target: Target,
    /// Operations of one group are done together (e.g. `structural` before `secondary`).
    #[serde(default)]
    pub group: String,
    /// Overrides of the task's parameters for this operation.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub params: Params,
}

/// Where on the part an operation happens. `normal` is the outward surface normal: the tool
/// works against it.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum Target {
    Point { position: [f32; 3], normal: [f32; 3] },
    /// A polyline over the surface, followed in order.
    Path { points: Vec<[f32; 3]>, normal: [f32; 3] },
}

impl Target {
    pub fn kind(&self) -> TargetKind {
        match self {
            Self::Point { .. } => TargetKind::Point,
            Self::Path { .. } => TargetKind::Path,
        }
    }

    pub fn normal(&self) -> [f32; 3] {
        match self {
            Self::Point { normal, .. } | Self::Path { normal, .. } => *normal,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TargetKind {
    Point,
    Path,
}

/// A process the tasks can use, with what its operations target and its parameters.
#[derive(Debug)]
pub struct Process {
    pub key: &'static str,
    pub label: &'static str,
    pub target: TargetKind,
    /// The kind of tool it needs, as a hint when none is picked.
    pub tool_hint: &'static str,
    pub params: &'static [Param],
}

/// A numeric process parameter.
#[derive(Debug)]
pub struct Param {
    pub key: &'static str,
    pub label: &'static str,
    pub unit: &'static str,
    pub default: f64,
}

const fn param(key: &'static str, label: &'static str, unit: &'static str, default: f64) -> Param {
    Param { key, label, unit, default }
}

/// The known processes. Add new ones here; the Product & Task pane and the 3D view follow.
pub const PROCESSES: &[Process] = &[
    Process {
        key: "spotWeld",
        label: "Spot welding",
        target: TargetKind::Point,
        tool_hint: "spot welding gun",
        params: &[
            param("weldTime", "Weld time", "s", 0.3),
            param("force", "Electrode force", "kN", 3.0),
            param("current", "Weld current", "kA", 9.0),
            param("approachOffset", "Approach offset", "m", 0.05),
        ],
    },
    Process {
        key: "seamWeld",
        label: "Seam (arc) welding",
        target: TargetKind::Path,
        tool_hint: "arc welding torch",
        params: &[
            param("speed", "Travel speed", "m/s", 0.008),
            param("current", "Current", "A", 180.0),
            param("wireFeed", "Wire feed", "m/min", 8.0),
            param("torchAngle", "Torch angle", "°", 45.0),
        ],
    },
    Process {
        key: "glue",
        label: "Gluing / sealing",
        target: TargetKind::Path,
        tool_hint: "glue dispenser",
        params: &[param("speed", "Travel speed", "m/s", 0.05), param("bead", "Bead width", "mm", 3.0)],
    },
    Process {
        key: "screw",
        label: "Screwing",
        target: TargetKind::Point,
        tool_hint: "screwdriver",
        params: &[param("torque", "Torque", "N·m", 5.0), param("depth", "Depth", "mm", 10.0)],
    },
    Process {
        key: "drill",
        label: "Drilling",
        target: TargetKind::Point,
        tool_hint: "spindle with drill",
        params: &[
            param("diameter", "Diameter", "mm", 6.0),
            param("depth", "Depth", "mm", 10.0),
            param("spindleSpeed", "Spindle speed", "rpm", 3000.0),
        ],
    },
    Process {
        key: "inspect",
        label: "Inspection",
        target: TargetKind::Point,
        tool_hint: "camera",
        params: &[param("standoff", "Standoff", "m", 0.3)],
    },
];

pub fn process(key: &str) -> Option<&'static Process> {
    PROCESSES.iter().find(|process| process.key == key)
}

impl Process {
    /// The parameters at their defaults.
    pub fn default_params(&self) -> Params {
        self.params.iter().map(|p| (p.key.to_owned(), serde_json::json!(p.default))).collect()
    }
}

/// `GET /api/v1/product/list`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ProductsInfo {
    /// The products dir on the backend host.
    pub dir: String,
    /// Product names, sorted.
    pub products: Vec<String>,
    #[serde(default)]
    pub error: Option<String>,
}

/// `POST /api/v1/product/load`.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct LoadProduct {
    pub name: String,
}

/// A product as stored: `POST /api/v1/product/load` and `/save` reply with it.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ProductFile {
    pub product: Product,
    /// `product.json` on the backend host.
    pub path: String,
}

/// `POST /api/v1/product/preview`: what the 3D view shows.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PreviewProduct {
    pub product: Product,
}

/// `GET /api/v1/product/catalogue`: the ready-made products, `<catalogue>/product-collection/<name>/`.
/// One is loaded by copying it into the products dir; a product is saved to it the same way back.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CatalogueProducts {
    /// The catalogue's product folder on the backend host.
    pub dir: String,
    /// Sorted by name.
    pub products: Vec<CatalogueProduct>,
    #[serde(default)]
    pub error: Option<String>,
}

/// One catalogue product, as listed.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CatalogueProduct {
    pub name: String,
    #[serde(default)]
    pub description: String,
    /// [`Geometry::kind`].
    pub geometry: String,
    /// How many tasks it has.
    pub tasks: usize,
}

/// `POST /api/v1/product/catalogue/load` (catalogue → products dir) and `/publish` (products dir →
/// catalogue). Without `overwrite`, an existing product is not replaced (`409`).
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CatalogueTransfer {
    pub name: String,
    #[serde(default)]
    pub overwrite: bool,
}

/// The URDFs in a product's `urdf/` folder, relative to the product's folder: the reply to an
/// upload (`PUT /api/v1/product/{name}/urdf`, a zip) or [`UrdfFromCatalogue`].
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ProductUrdfs {
    pub urdfs: Vec<String>,
}

/// `POST /api/v1/product/urdf/from_catalogue`: copy the catalogue's fixture `fixture`
/// (`urdf-fixture-collection/<fixture>/`) into product `product`'s `urdf/` folder.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UrdfFromCatalogue {
    pub product: String,
    pub fixture: String,
}

/// Whether `name` is usable as a product (folder) name.
pub fn valid_name(name: &str) -> bool {
    !name.is_empty()
        && name.len() <= 64
        && name.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '-'))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn product_json_round_trip() {
        let spot = process("spotWeld").unwrap();
        let product = Product {
            name: "cube".to_owned(),
            description: "A test part".to_owned(),
            geometry: Geometry::Box { size: [0.5, 0.5, 0.2] },
            tasks: vec![
                Task {
                    name: "frame".to_owned(),
                    process: spot.key.to_owned(),
                    tool: "spot_gun".to_owned(),
                    params: spot.default_params(),
                    operations: vec![Operation {
                        id: "W01".to_owned(),
                        target: Target::Point { position: [0.0, 0.0, 0.2], normal: [0.0, 0.0, 1.0] },
                        group: "structural".to_owned(),
                        params: Params::from([("weldTime".to_owned(), serde_json::json!(0.5))]),
                    }],
                },
                Task {
                    name: "seal".to_owned(),
                    process: "glue".to_owned(),
                    operations: vec![Operation {
                        id: "G01".to_owned(),
                        target: Target::Path { points: vec![[0.0; 3], [0.1, 0.0, 0.0]], normal: [0.0, 0.0, 1.0] },
                        group: String::new(),
                        params: Params::new(),
                    }],
                    ..Task::default()
                },
            ],
        };
        let json = serde_json::to_value(&product).unwrap();
        assert_eq!(json["geometry"]["kind"], "box");
        assert_eq!(json["tasks"][0]["operations"][0]["target"]["kind"], "point");
        assert_eq!(json["tasks"][1]["operations"][0]["target"]["kind"], "path");
        assert!(json["tasks"][1]["operations"][0].get("params").is_none());
        assert_eq!(serde_json::from_value::<Product>(json).unwrap(), product);
    }

    #[test]
    fn urdf_geometry() {
        let geometry = Geometry::Urdf { path: "urdf/part.urdf".to_owned() };
        let json = serde_json::to_value(&geometry).unwrap();
        assert_eq!(json, serde_json::json!({"kind": "urdf", "path": "urdf/part.urdf"}));
        assert_eq!(serde_json::from_value::<Geometry>(json).unwrap(), geometry);
        assert_eq!(geometry.kind(), "urdf");
        // Files written before URDFs keep loading.
        let old: Product = serde_json::from_str(r#"{"name":"a","geometry":{"kind":"mesh","path":"a.obj"}}"#).unwrap();
        assert_eq!(old.geometry, Geometry::Mesh { path: "a.obj".to_owned(), scale: 1.0 });
    }

    #[test]
    fn unknown_processes_still_load() {
        let task: Task = serde_json::from_str(r#"{"name":"x","process":"laserCut","operations":[]}"#).unwrap();
        assert!(process(&task.process).is_none());
    }

    #[test]
    fn processes_are_unique() {
        for (i, a) in PROCESSES.iter().enumerate() {
            assert!(PROCESSES[i + 1..].iter().all(|b| b.key != a.key), "{}", a.key);
        }
    }

    #[test]
    fn names() {
        assert!(valid_name("door_panel-2"));
        assert!(!valid_name("../etc"));
        assert!(!valid_name(""));
        assert!(!valid_name("a b"));
    }
}
