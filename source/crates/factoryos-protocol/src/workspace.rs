//! Workspaces: each workspace's backend is reached under `/workspace/<name>/…`. On the backend
//! (which strips the prefix before routing) and at the load balancer (which routes by it) the path
//! is the same. Below the prefix the paths are the backend's own ([`crate::api`]).
//!
//! Behind the workspace edge (the login in front of the workspaces) viewers don't use the prefix
//! themselves: they talk to the origin (`/api/…`, `/zenoh`, `/chat`) and pick the workspace with the
//! [`SELECT_COOKIE`] cookie, which the edge checks and turns into the prefix. The provisioner
//! ([`PROVISIONER_PREFIX`]) lists the workspaces a user may open, and starts them on demand.

use serde::{Deserialize, Serialize};

/// The start of every workspace's paths.
pub const PREFIX: &str = "/workspace/";

/// Cookie naming the workspace the viewer wants (behind the edge). Absent: the user's own.
pub const SELECT_COOKIE: &str = "fos_ws";

/// The workspace provisioner's routes (served next to the workspaces, not by a backend).
pub const PROVISIONER_PREFIX: &str = "/_workspace/";
/// `GET`: [`WorkspaceList`].
pub const LIST: &str = "/_workspace/list";
/// `GET`: [`WorkspaceStatus`] of the selected workspace; starts the user's own if it isn't running.
pub const STATUS: &str = "/_workspace/status";
/// `POST`: back the user's own workspace up to S3 now.
pub const BACKUP: &str = "/_workspace/backup";
/// `POST` [`Reprovision`]: replace the user's own workspace's instance.
pub const REPROVISION: &str = "/_workspace/reprovision";

/// Whether `name` can be a workspace: a non-empty path segment of `[A-Za-z0-9._-]`.
pub fn is_valid_name(name: &str) -> bool {
    !name.is_empty()
        && name != "."
        && name != ".."
        && name.bytes().all(|b| b.is_ascii_alphanumeric() || matches!(b, b'.' | b'_' | b'-'))
}

/// The base URL of workspace `name` on the server at `server` (`http://host[:port]`).
pub fn url(server: &str, name: &str) -> String {
    format!("{}{PREFIX}{name}", server.trim_end_matches('/'))
}

/// `/workspace/<name>/rest` → (`name`, `/rest`); `/workspace/<name>` → (`name`, ``).
/// `None` for paths outside a workspace or with an invalid name.
pub fn split_path(path: &str) -> Option<(&str, &str)> {
    let rest = path.strip_prefix(PREFIX)?;
    let (name, rest) = rest.find('/').map_or((rest, ""), |slash| rest.split_at(slash));
    is_valid_name(name).then_some((name, rest))
}

/// The workspace part of a page's path, e.g. `/workspace/jdoe` for `/workspace/jdoe/`;
/// empty for a backend served at `/`.
pub fn base_path(path: &str) -> &str {
    match split_path(path) {
        Some((name, _)) => &path[..PREFIX.len() + name.len()],
        None => "",
    }
}

/// The workspace in a backend URL (`http://host/workspace/<name>`), if any.
pub fn of_url(url: &str) -> Option<&str> {
    let rest = url.split_once("://").map_or(url, |(_, rest)| rest);
    let path = rest.find('/').map_or("", |slash| &rest[slash..]);
    split_path(path).map(|(name, _)| name)
}

/// `base` + `path`, pinned to `workspace`: a link opened outside the viewer (in the system browser,
/// whose [`SELECT_COOKIE`] may select another workspace) reaches the backend that made it.
/// * `base` has a workspace already (`…/workspace/<x>`): `base` + `path`.
/// * `base` is a bare origin (`https://host[:port]`) and `workspace` a valid name:
///   `base/workspace/<workspace>` + `path`.
/// * Else `base` + `path`.
pub fn pinned_url(base: &str, workspace: Option<&str>, path: &str) -> String {
    let base = base.trim_end_matches('/');
    let origin = base.split_once("://").is_some_and(|(scheme, rest)| !scheme.is_empty() && !rest.is_empty() && !rest.contains(['/', '?', '#']));
    match workspace {
        Some(name) if of_url(base).is_none() && origin && is_valid_name(name) => format!("{}{path}", url(base, name)),
        _ => format!("{base}{path}"),
    }
}

/// Whose a workspace is.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceKind {
    /// The user's own, on its own instance.
    #[default]
    Personal,
    /// One everybody may open (e.g. `ci`, the pipeline's deploy target): its state is shared.
    Shared,
}

/// Where a workspace's instance is.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceState {
    /// No instance yet; opening the workspace creates one.
    #[default]
    None,
    /// A new instance is being created (first boot: restore from backup, install).
    Launching,
    /// A stopped instance is starting again.
    Starting,
    /// The instance runs; its backend is not up yet.
    Booting,
    /// Serving.
    Ready,
    /// Stopping (idle, or for a replacement).
    Stopping,
    /// Stopped (idle); opening the workspace starts it.
    Stopped,
    /// Being replaced by a fresh instance (see [`Reprovision`]).
    Replacing,
    /// Something went wrong; [`WorkspaceStatus::message`] says what.
    Error,
}

impl WorkspaceState {
    /// Whether the workspace is on its way to [`Self::Ready`] (worth waiting for).
    pub fn is_coming_up(self) -> bool {
        matches!(self, Self::Launching | Self::Starting | Self::Booting | Self::Stopping | Self::Replacing)
    }
}

/// One workspace a user may open.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct WorkspaceEntry {
    pub name: String,
    #[serde(default)]
    pub kind: WorkspaceKind,
    #[serde(default)]
    pub state: WorkspaceState,
}

/// `GET` [`LIST`].
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct WorkspaceList {
    /// The logged-in user.
    pub alias: String,
    /// The workspace [`SELECT_COOKIE`] selects (after the edge checked it), else the user's own.
    pub selected: String,
    pub workspaces: Vec<WorkspaceEntry>,
}

/// `GET` [`STATUS`], and the body of a `503` for a workspace that is not ready yet.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct WorkspaceStatus {
    pub name: String,
    #[serde(default)]
    pub kind: WorkspaceKind,
    pub state: WorkspaceState,
    /// What the instance is doing while coming up, e.g. `restoring`, `installing`, `starting`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub step: Option<String>,
    /// Unix time (s) the current state began.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub since: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
    /// The release (CI build id) the workspace runs, or will run once up. The web viewer loads
    /// the same release's wasm, so viewer and backend always match.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub release: Option<String>,
    /// Unix time (s) of the newest backup in S3.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_backup: Option<u64>,
}

impl WorkspaceStatus {
    /// A line for the user: the state, with the step while coming up.
    pub fn describe(&self) -> String {
        let step = self.step.as_deref().map(step_label);
        let what = match self.state {
            WorkspaceState::None => "No workspace yet".to_owned(),
            WorkspaceState::Launching => step.unwrap_or("Launching a new instance").to_owned(),
            WorkspaceState::Starting => "Starting the instance".to_owned(),
            WorkspaceState::Booting => step.unwrap_or("Starting the backend").to_owned(),
            WorkspaceState::Ready => "Ready".to_owned(),
            WorkspaceState::Stopping => "Waiting for the instance to stop".to_owned(),
            WorkspaceState::Stopped => "Stopped (idle)".to_owned(),
            WorkspaceState::Replacing => step.unwrap_or("Replacing the instance").to_owned(),
            WorkspaceState::Error => "Error".to_owned(),
        };
        match &self.message {
            Some(message) if !message.is_empty() => format!("{what}: {message}"),
            _ => what,
        }
    }
}

/// The user-facing text of a [`WorkspaceStatus::step`].
pub fn step_label(step: &str) -> &str {
    match step {
        "launching" => "Launching a new instance",
        "restoring" => "Restoring your workspace from its backup",
        "installing" => "Installing Physical AI Production Designer",
        "starting" => "Starting the backend",
        "backing-up" => "Backing up the workspace",
        "archiving" => "Archiving the backup",
        "terminating" => "Removing the old instance",
        other => other,
    }
}

/// How [`REPROVISION`] replaces the instance.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReprovisionMode {
    /// Back up, then a fresh instance restores the backup.
    #[default]
    Keep,
    /// Back up and archive that backup, then a fresh instance starts from the starter content.
    Fresh,
}

/// `POST` [`REPROVISION`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Reprovision {
    pub mode: ReprovisionMode,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splits_paths() {
        assert_eq!(split_path("/workspace/jdoe/api/v1/health"), Some(("jdoe", "/api/v1/health")));
        assert_eq!(split_path("/workspace/jdoe/"), Some(("jdoe", "/")));
        assert_eq!(split_path("/workspace/jdoe"), Some(("jdoe", "")));
        assert_eq!(split_path("/workspace/"), None);
        assert_eq!(split_path("/workspace/../etc"), None);
        assert_eq!(split_path("/workspace/a%20b/"), None);
        assert_eq!(split_path("/api/v1/health"), None);
    }

    #[test]
    fn base_paths_and_urls() {
        assert_eq!(base_path("/workspace/jdoe/index.html"), "/workspace/jdoe");
        assert_eq!(base_path("/workspace/jdoe/"), "/workspace/jdoe");
        assert_eq!(base_path("/"), "");
        assert_eq!(url("http://203.0.113.10/", "jdoe"), "http://203.0.113.10/workspace/jdoe");
        assert_eq!(of_url("http://203.0.113.10/workspace/jdoe"), Some("jdoe"));
        assert_eq!(of_url("http://203.0.113.10/workspace/jdoe/"), Some("jdoe"));
        assert_eq!(of_url("http://127.0.0.1:8080"), None);
    }

    #[test]
    fn pinned_urls() {
        let path = "/dcv/ide/?authToken=jdoe.ab#console";
        // A native viewer behind the edge: the bare origin, the workspace a cookie the browser lacks.
        assert_eq!(pinned_url("https://d1.cloudfront.net", Some("jdoe"), path), format!("https://d1.cloudfront.net/workspace/jdoe{path}"));
        assert_eq!(pinned_url("https://d1.cloudfront.net/", Some("jdoe"), path), format!("https://d1.cloudfront.net/workspace/jdoe{path}"));
        // A test server reached directly: its own prefix, which its backend strips.
        assert_eq!(pinned_url("http://198.51.100.7", Some("plan2cell-test"), path), format!("http://198.51.100.7/workspace/plan2cell-test{path}"));
        // A base with a workspace already (the web viewer, plain http) is used as it is.
        assert_eq!(pinned_url("http://host/workspace/ci", Some("other"), path), format!("http://host/workspace/ci{path}"));
        assert_eq!(pinned_url("https://host", None, path), format!("https://host{path}"));
        assert_eq!(pinned_url("https://host", Some("../x"), path), format!("https://host{path}"));
        assert_eq!(pinned_url("https://host/sub", Some("ci"), path), format!("https://host/sub{path}"));
        assert_eq!(pinned_url("", Some("ci"), path), path);
    }

    #[test]
    fn status_wire_format() {
        let json = r#"{"name":"jdoe","kind":"personal","state":"launching","step":"restoring","release":"p42","since":1}"#;
        let status: WorkspaceStatus = serde_json::from_str(json).unwrap();
        assert_eq!(status.state, WorkspaceState::Launching);
        assert_eq!(status.release.as_deref(), Some("p42"));
        assert!(status.state.is_coming_up());
        assert_eq!(status.describe(), "Restoring your workspace from its backup");
        let back: WorkspaceStatus = serde_json::from_str(&serde_json::to_string(&status).unwrap()).unwrap();
        assert_eq!(back, status);

        let list: WorkspaceList = serde_json::from_str(
            r#"{"alias":"jdoe","selected":"ci","workspaces":[{"name":"jdoe","kind":"personal","state":"stopped"},{"name":"ci","kind":"shared","state":"ready"}]}"#,
        )
        .unwrap();
        assert_eq!(list.workspaces[1].kind, WorkspaceKind::Shared);
        assert_eq!(serde_json::to_string(&Reprovision { mode: ReprovisionMode::Fresh }).unwrap(), r#"{"mode":"fresh"}"#);
    }
}
