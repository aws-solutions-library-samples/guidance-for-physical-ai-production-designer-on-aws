#![forbid(unsafe_code)]

pub mod camera_rig;
pub mod compose;
pub mod drives;
pub mod error;
pub mod floor_plan;
pub mod math3;
pub mod mtl;
pub mod obj;
pub mod scene;
pub mod tool;

#[cfg(any(feature = "server", feature = "stdio"))]
pub mod transport_common;

#[cfg(feature = "server")]
pub mod server;

#[cfg(feature = "mcp")]
pub mod mcp;

#[cfg(feature = "stdio")]
pub mod stdio;

pub use compose::{
    compose_stage, compose_stage_with_options, compose_urdf_bundle, ComposeOptions, ComposeReport,
    UrdfBundleReport,
};
pub use error::FittingRoomError;
pub use floor_plan::FloorPlan;
pub use scene::{
    parse_scene, parse_scene_from_yaml, scene_warnings, validate_scene, FixturePlacement, Payload,
    Pedestal, RobotPlacement, SceneDefinition,
};
