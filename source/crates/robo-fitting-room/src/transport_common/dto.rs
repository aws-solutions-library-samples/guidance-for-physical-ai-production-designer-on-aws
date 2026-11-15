//! Wire types shared between transports and the SPA.
//!
//! These structs derive plain `serde` `Serialize`/`Deserialize`
//! unconditionally; only the `utoipa::ToSchema` derive is `server`-feature
//! gated (the `stdio` transport never generates an OpenAPI document). They
//! live here in `transport_common` — compiled whenever either the
//! `server` or the `stdio` feature is on — so both the HTTP handlers and
//! the stdio dispatch table reuse the exact same shapes with zero
//! duplication. `server::dto` re-exports them unchanged, so existing
//! `crate::server::dto::*` call sites keep working without modification.

use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize)]
#[cfg_attr(feature = "server", derive(utoipa::ToSchema))]
pub struct AssetEntry {
    pub id: String,
    pub urdf_rel: String,
}

#[derive(Debug, Serialize)]
#[cfg_attr(feature = "server", derive(utoipa::ToSchema))]
pub struct FloorPlanEntry {
    pub name: String,
    pub ext: String,
    pub rel: String,
}

#[derive(Debug, Serialize)]
#[cfg_attr(feature = "server", derive(utoipa::ToSchema))]
pub struct JointInfo {
    pub name: String,
    /// "revolute" / "continuous" / "prismatic" — non-Fixed only.
    pub kind: String,
    pub axis: [f64; 3],
    /// A revolute or continuous joint's URDF limits, degrees.
    pub lower_deg: Option<f64>,
    pub upper_deg: Option<f64>,
    /// A prismatic joint's URDF limits, metres (its home value is in metres too).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lower_m: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub upper_m: Option<f64>,
}

impl JointInfo {
    /// The listing entry of an actuated URDF `joint`: limits in degrees for a revolute or
    /// continuous joint, in metres for a prismatic one.
    pub fn of(joint: &urdf_robo_dresser::Joint) -> Self {
        use urdf_robo_dresser::JointType;
        let prismatic = joint.joint_type == JointType::Prismatic;
        let limits = |convert: fn(f64) -> f64| {
            joint
                .limit
                .as_ref()
                .map(|l| (convert(l.lower), convert(l.upper)))
                .unzip()
        };
        let (lower_deg, upper_deg) = if prismatic {
            (None, None)
        } else {
            limits(f64::to_degrees)
        };
        let (lower_m, upper_m) = if prismatic {
            limits(|v| v)
        } else {
            (None, None)
        };
        Self {
            name: joint.name.clone(),
            kind: match joint.joint_type {
                JointType::Revolute => "revolute",
                JointType::Continuous => "continuous",
                JointType::Prismatic => "prismatic",
                JointType::Fixed => "fixed",
                JointType::Floating => "floating",
                JointType::Planar => "planar",
            }
            .to_string(),
            axis: joint.axis,
            lower_deg,
            upper_deg,
            lower_m,
            upper_m,
        }
    }
}

#[derive(Debug, Serialize)]
#[cfg_attr(feature = "server", derive(utoipa::ToSchema))]
pub struct UrdfJointsResponse {
    pub joints: Vec<JointInfo>,
}

#[derive(Debug, Serialize)]
#[cfg_attr(feature = "server", derive(utoipa::ToSchema))]
pub struct FloorPlanSegments {
    /// Each segment is [[x, y, z], [x, y, z]].
    pub segments: Vec<[[f64; 3]; 2]>,
    pub footprint_segments: Vec<[[f64; 3]; 2]>,
}

#[derive(Debug, Serialize)]
#[cfg_attr(feature = "server", derive(utoipa::ToSchema))]
pub struct ValidateResponse {
    pub ok: bool,
    pub errors: Vec<String>,
    /// What composes, but probably not as the manifest means it (`scene_warnings`); these don't
    /// fail validation. Absent when there are none.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub warnings: Vec<String>,
}

#[derive(Debug, Deserialize)]
#[cfg_attr(feature = "server", derive(utoipa::ToSchema))]
pub struct PathQuery {
    /// The file's path relative to the asset root, e.g. `urdf-dressed-robots/r1/r1.urdf` or
    /// `fitted-robots/scene.yaml`. Absolute paths and `..` segments are rejected.
    pub path: String,
}

/// A floor-plan request: the OBJ's path under the asset root and, optionally, the manifest's
/// `floor_plan.footprint_layers`, so the preview splits footprints as compose does.
#[derive(Debug, Deserialize)]
#[cfg_attr(feature = "server", derive(utoipa::ToSchema))]
pub struct FloorPlanQuery {
    /// The OBJ's path relative to the asset root, e.g. `floor-plans/plan.obj`. Absolute paths
    /// and `..` segments are rejected.
    pub path: String,
    /// The layers (or OBJ groups) drawn as footprints, by name, case-insensitively; layers named
    /// like "footprint" are footprints anyway. Default: none.
    #[serde(default, alias = "footprintLayers")]
    #[cfg_attr(feature = "server", schema(default = json!([])))]
    pub footprint_layers: Vec<String>,
}

#[derive(Debug, Deserialize)]
#[cfg_attr(feature = "server", derive(utoipa::ToSchema))]
pub struct ScenePathBody {
    /// The manifest's path relative to the asset root, e.g. `fitted-robots/scene.yaml`; missing
    /// parent folders are created. Absolute paths and `..` segments are rejected.
    pub path: String,
    /// The manifest's YAML text, written verbatim (not validated).
    pub yaml: String,
}

#[derive(Debug, Serialize)]
#[cfg_attr(feature = "server", derive(utoipa::ToSchema))]
pub struct UploadResponse {
    pub stored_path: String,
    pub bytes: u64,
}

#[derive(Debug, Serialize)]
#[cfg_attr(feature = "server", derive(utoipa::ToSchema))]
pub struct SaveResponse {
    pub ok: bool,
    pub bytes: u64,
}

/// Liveness probe response. `GET /healthz` always returns 200 with this.
#[derive(Debug, Serialize)]
#[cfg_attr(feature = "server", derive(utoipa::ToSchema))]
pub struct HealthDto {
    pub status: String,
    pub version: String,
}

/// Readiness probe response. `GET /readyz` returns 200 with `ready: true`
/// when the asset_root canonicalises and at least one URDF is discoverable;
/// otherwise 503 with `ready: false` and a populated `errors` list.
#[derive(Debug, Serialize)]
#[cfg_attr(feature = "server", derive(utoipa::ToSchema))]
pub struct ReadinessDto {
    pub ready: bool,
    pub asset_root: String,
    pub urdf_count: usize,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub errors: Vec<String>,
}
