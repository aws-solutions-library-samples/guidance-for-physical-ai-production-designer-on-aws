//! WebRTC streams (NVIDIA Isaac Sim's web viewer) of a workspace: [`crate::api::RTC_TARGETS`].

use serde::{Deserialize, Serialize};

/// `GET` [`crate::api::RTC_TARGETS`]: the streams at [`crate::api::rtc_target`], e.g. `isaac` or a linked
/// instance's id. None: no Isaac Sim is linked to this workspace (an Isaac Sim stack's instance is linked
/// once it is registered with the workspace).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct RtcTargets {
    pub targets: Vec<String>,
    /// Whether the asking user may open them (the workspace's owner, or `FACTORY_OS_RTC_USERS`).
    pub may_watch: bool,
    /// Why the streams are off although configured (a bad setting).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}
