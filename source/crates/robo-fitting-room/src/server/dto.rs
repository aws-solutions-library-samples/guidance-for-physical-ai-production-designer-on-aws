//! Wire types shared between handlers and the SPA.
//!
//! The struct definitions now live in `crate::transport_common::dto` so
//! they compile independently of axum for the `stdio`-only build (the
//! `stdio` dispatch table reuses the exact same shapes). They're
//! re-exported here unchanged so existing `crate::server::dto::*` call
//! sites (handlers, `server::openapi`) keep working without modification.
//! `ErrorEnvelope` likewise lives in `crate::transport_common` and is
//! re-exported here for the same reason. `UploadObjForm` is HTTP-only
//! (stdio carries the upload as base64 JSON), so it is defined here.

pub use crate::transport_common::dto::{
    AssetEntry, FloorPlanEntry, FloorPlanQuery, FloorPlanSegments, HealthDto, JointInfo, PathQuery,
    ReadinessDto, SaveResponse, ScenePathBody, UploadResponse, UrdfJointsResponse,
    ValidateResponse,
};
pub use crate::transport_common::ErrorEnvelope;

// Documents the multipart request in the OpenAPI spec only: `upload_obj`
// reads the parts with axum's `Multipart`, so nothing builds this type.
/// The `multipart/form-data` fields of an OBJ floor-plan upload.
#[derive(Debug, utoipa::ToSchema)]
pub struct UploadObjForm {
    /// The name to store the file under, e.g. `plan.obj`; it must end in
    /// `.obj` (any case). Only its last path component is kept, and every
    /// character other than an ASCII letter, a digit, `.`, `_` or `-` becomes `_`.
    pub filename: String,
    /// The OBJ file.
    #[schema(value_type = String, format = Binary)]
    pub file: Vec<u8>,
}

// A binary response body, documented as `type: string, format: binary` (utoipa 5+ documents a
// bare `Vec<u8>` as an array of integers). A comment, not a doc comment: that would become its
// description in the spec.
#[derive(utoipa::ToSchema)]
#[schema(value_type = String, format = Binary)]
pub struct BinaryBody(pub Vec<u8>);
