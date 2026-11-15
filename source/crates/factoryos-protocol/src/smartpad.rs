//! The KUKA controllers' smartPADs (KDM's iiQKA.UI web app) of a workspace's cell: [`crate::api::SMARTPADS`].
//!
//! iiQKA.UI only works at `/` of a host of its own: it takes the controller instance from the first path segment and
//! calls `/auth`, `/sessions`, `/self-service/*` on its origin (plan2cell/docs/INTEGRATION-SPIKES.md S3). So each
//! controller of each workspace has a hostname on the workspace edge, `<robot>--<workspace>.<site>` ([`host`]), and the
//! edge sends its requests to that workspace's backend as `/workspace/<workspace>/smartpad/<robot>/…`
//! ([`crate::api::SMARTPAD`]), which proxies them to the controller's KLI address in the cell.

use serde::{Deserialize, Serialize};

/// Between the robot and the workspace in a smartPAD's hostname label.
pub const SEPARATOR: &str = "--";

/// `GET` [`crate::api::SMARTPADS`]: the cell's controllers (`[[kuka_vc.controller]]` of its `plan2cell cell` config).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct SmartPads {
    pub pads: Vec<SmartPad>,
    /// Whether the asking user may open them (the workspace's owner, or `FACTORY_OS_SMARTPAD_USERS`).
    pub may_open: bool,
    /// Why the pads have no [`SmartPad::url`] (no workspace edge with smartPAD hostnames, no cell config, ...).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct SmartPad {
    /// The controller's `robot_ns` (`robot1`, ...): the first label of its hostname.
    pub robot: String,
    /// Its KLI address inside the cell (`192.168.100.11`).
    pub kli_ip: String,
    /// Where the backend reaches it (`[address_map]`, else [`Self::kli_ip`]).
    pub address: String,
    /// `https://<robot>--<workspace>.<site>/` (`None`: see [`SmartPads::reason`]).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
}

/// A robot as a hostname part: `^[a-z][a-z0-9]{0,15}$` (no `-`, so the label splits at its first [`SEPARATOR`]).
pub fn valid_robot(robot: &str) -> bool {
    let mut chars = robot.chars();
    chars.next().is_some_and(|c| c.is_ascii_lowercase())
        && robot.len() <= 16
        && chars.all(|c| c.is_ascii_lowercase() || c.is_ascii_digit())
}

/// A workspace as a hostname part: lowercase letters, digits and inner `-`.
pub fn valid_workspace(workspace: &str) -> bool {
    !workspace.is_empty()
        && !workspace.starts_with('-')
        && !workspace.ends_with('-')
        && workspace.chars().all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
}

/// `<robot>--<workspace>.<site>` (`None`: a name that can't be one DNS label of at most 63 characters).
pub fn host(robot: &str, workspace: &str, site: &str) -> Option<String> {
    let label = format!("{robot}{SEPARATOR}{workspace}");
    (valid_robot(robot) && valid_workspace(workspace) && label.len() <= 63 && !site.is_empty())
        .then(|| format!("{label}.{}", site.to_ascii_lowercase()))
}

/// `https://<host>/`.
pub fn url(robot: &str, workspace: &str, site: &str) -> Option<String> {
    host(robot, workspace, site).map(|h| format!("https://{h}/"))
}

/// The `(robot, workspace)` of a smartPAD hostname of `site` (a port is ignored).
pub fn parse_host(host: &str, site: &str) -> Option<(String, String)> {
    let host = host.trim().to_ascii_lowercase();
    let host = host.rsplit_once(':').filter(|(_, port)| port.chars().all(|c| c.is_ascii_digit())).map_or(host.as_str(), |(h, _)| h);
    let label = host.strip_suffix(&site.to_ascii_lowercase())?.strip_suffix('.')?;
    let (robot, workspace) = label.split_once(SEPARATOR)?;
    (self::host(robot, workspace, site).as_deref() == Some(host)).then(|| (robot.to_owned(), workspace.to_owned()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hosts() {
        let site = "factoryos-dev.example.dev";
        assert_eq!(host("robot1", "jdoe", site).as_deref(), Some("robot1--jdoe.factoryos-dev.example.dev"));
        assert_eq!(url("robot2", "ci", site).as_deref(), Some("https://robot2--ci.factoryos-dev.example.dev/"));
        assert_eq!(parse_host("robot1--jdoe.factoryos-dev.example.dev", site), Some(("robot1".into(), "jdoe".into())));
        assert_eq!(parse_host("ROBOT1--jdoe.factoryos-dev.example.dev:443", site), Some(("robot1".into(), "jdoe".into())));
        assert_eq!(parse_host("robot1--j-doe.factoryos-dev.example.dev", site), Some(("robot1".into(), "j-doe".into())));
        for bad in [
            "factoryos-dev.example.dev",
            "robot1.factoryos-dev.example.dev",
            "robot1--jdoe.other.dev",
            "a.robot1--jdoe.factoryos-dev.example.dev",
            "robot-1--jdoe.factoryos-dev.example.dev",
            "robot1--jdoe-.factoryos-dev.example.dev",
            "robot1--.factoryos-dev.example.dev",
            "robot1--jdoe.factoryos-dev.example.dev.evil.dev",
        ] {
            assert_eq!(parse_host(bad, site), None, "{bad}");
        }
        assert_eq!(host("robot1", "J.Doe", site), None);
        assert_eq!(host("robot1", &"a".repeat(60), site), None);
        assert!(valid_robot("robot12") && !valid_robot("1robot") && !valid_robot("") && !valid_robot("robot_1"));
    }
}
