//! Signing the backend's planner agent (`kiro-cli`) in from the viewer: the backend runs
//! `kiro-cli login --use-device-flow` and hands the device code to the viewer, which opens the
//! approval page in a browser window ([`crate::api::AGENT_LOGIN`]).
//!
//! The agent signs in with an AWS Builder ID or through an organisation's IAM Identity Center
//! ([`LoginMode`]). A build with a default start URL ([`DEFAULT_START_URL`]) uses that one unless the
//! user names another; a build without one (the public export's) lets the user choose: a Builder ID,
//! or their organisation's start URL and region. The backend's `FACTORY_OS_KIRO_LOGIN_ARGS` fixes the
//! login whatever is asked ([`AgentLogin::preset`]).

use serde::{Deserialize, Serialize};

/// A device-flow login waiting for the user to approve it in the browser.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeviceCode {
    /// The page to approve the login on (it may already carry the code).
    pub verification_uri: String,
    /// The code the page asks for.
    #[serde(default)]
    pub user_code: String,
}

/// The start URL the agent signs in with by default (`kiro-cli login --identity-provider`), if the build
/// has one; `None`: the user chooses ([`LoginMode`]).
#[cfg(feature = "internal")]
pub const DEFAULT_START_URL: Option<&str> = Some(crate::internal::KIRO_START_URL);
/// The start URL the agent signs in with by default (`kiro-cli login --identity-provider`), if the build
/// has one; `None`: the user chooses ([`LoginMode`]).
#[cfg(not(feature = "internal"))]
pub const DEFAULT_START_URL: Option<&str> = None;
/// The region an organisation's sign-in starts with.
pub const DEFAULT_REGION: &str = "us-east-1";

/// How the agent signs in: `kiro-cli login`'s license and identity provider.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LoginMode {
    /// An AWS Builder ID: `--license free`.
    BuilderId,
    /// An organisation's (IAM Identity Center): `--license pro --identity-provider <start URL> --region
    /// <region>`. A request without a mode (an older viewer's, which always named a start URL) is this one.
    #[default]
    IdentityCenter,
}

impl LoginMode {
    /// Every mode, in the order a choice lists them.
    pub const ALL: [LoginMode; 2] = [LoginMode::BuilderId, LoginMode::IdentityCenter];

    /// What the viewer calls it.
    pub fn label(self) -> &'static str {
        match self {
            Self::BuilderId => "AWS Builder ID",
            Self::IdentityCenter => "IAM Identity Center",
        }
    }
}

/// `POST` [`crate::api::AGENT_LOGIN`]'s body: how and where to sign in. Fields left out take
/// [`AgentLoginRequest::default`]'s, except `mode` (an organisation's sign-in): an older viewer sent only
/// `start_url` and `region` (and an empty body for the defaults, as [`Default`] has them).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct AgentLoginRequest {
    /// A Builder ID or an organisation's sign-in (absent: the latter, see above).
    #[serde(default)]
    pub mode: LoginMode,
    /// The organisation's start URL (`https://…/start`); not used by a Builder ID sign-in.
    pub start_url: String,
    /// The organisation's region; not used by a Builder ID sign-in.
    pub region: String,
}

impl Default for AgentLoginRequest {
    /// The build's default sign-in: its start URL ([`DEFAULT_START_URL`]), else a Builder ID.
    fn default() -> Self {
        match DEFAULT_START_URL {
            Some(start_url) => {
                Self { mode: LoginMode::IdentityCenter, start_url: start_url.to_owned(), region: DEFAULT_REGION.to_owned() }
            }
            None => Self { mode: LoginMode::BuilderId, start_url: String::new(), region: DEFAULT_REGION.to_owned() },
        }
    }
}

impl AgentLoginRequest {
    /// Why it can't be passed to `kiro-cli login`, if it can't (a Builder ID sign-in names nothing).
    pub fn validate(&self) -> Result<(), String> {
        if self.mode == LoginMode::BuilderId {
            return Ok(());
        }
        let url = self.start_url.trim();
        if url.is_empty() {
            return Err("no start URL (your organisation's: https://…/start)".to_owned());
        }
        if !url.starts_with("https://")
            || url.len() < 12
            || url.chars().any(|c| c.is_whitespace() || matches!(c, '"' | '\'' | '`' | '$' | '\\' | ';' | '&' | '|' | '<' | '>'))
        {
            return Err(format!("not a start URL: {url} (https://…)"));
        }
        let region = self.region.trim();
        if region.is_empty() || !region.bytes().all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-') {
            return Err(format!("not a region: {region} (e.g. us-east-1)"));
        }
        Ok(())
    }
}

/// `GET` / `POST` [`crate::api::AGENT_LOGIN`]: whether the agent is signed in, or a login is pending.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct AgentLogin {
    pub logged_in: bool,
    /// Who it is signed in as (e.g. an email), if known.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub user: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pending: Option<DeviceCode>,
    /// Why the last login failed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// The server fixes how the agent signs in (the backend's `FACTORY_OS_KIRO_LOGIN_ARGS`): a request's
    /// mode, start URL and region are not used, so there is nothing to ask for or check. (Older backends
    /// leave it out: `false`.)
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub preset: bool,
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    /// The AWS CLI documentation's example start URL.
    const START_URL: &str = "https://my-sso-portal.awsapps.com/start";

    fn request(value: serde_json::Value) -> AgentLoginRequest {
        serde_json::from_value(value).unwrap()
    }

    fn organisation(start_url: &str, region: &str) -> AgentLoginRequest {
        AgentLoginRequest { mode: LoginMode::IdentityCenter, start_url: start_url.to_owned(), region: region.to_owned() }
    }

    #[test]
    fn older_viewers_requests_are_an_organisations_sign_ins() {
        // What viewers before the login modes sent: the start URL and region, always both.
        let old = request(json!({"start_url": START_URL, "region": "eu-west-1"}));
        assert_eq!(old, organisation(START_URL, "eu-west-1"));
        assert!(old.validate().is_ok());
        // ... and what an older backend reads from a newer viewer's request: both fields, as before.
        let sent = serde_json::to_value(AgentLoginRequest { mode: LoginMode::BuilderId, ..AgentLoginRequest::default() }).unwrap();
        assert_eq!(sent["mode"], "builder_id");
        assert!(sent["start_url"].is_string() && sent["region"].is_string(), "{sent}");
    }

    #[test]
    fn modes_on_the_wire() {
        let builder = request(json!({"mode": "builder_id"}));
        assert_eq!(builder.mode, LoginMode::BuilderId);
        assert!(builder.validate().is_ok(), "a Builder ID names no start URL");
        let named = request(json!({"mode": "identity_center", "start_url": format!(" {START_URL} "), "region": "us-west-2"}));
        assert_eq!(named.mode, LoginMode::IdentityCenter);
        assert!(named.validate().is_ok());
        assert_eq!(serde_json::to_value(LoginMode::IdentityCenter).unwrap(), "identity_center");
        assert_eq!(LoginMode::ALL.map(LoginMode::label), ["AWS Builder ID", "IAM Identity Center"]);
    }

    #[test]
    fn validates_an_organisations_sign_in() {
        let bad = |start_url: &str, region: &str| organisation(start_url, region).validate().is_err();
        assert!(bad("", "us-east-1") && bad("http://x.example.com", "us-east-1") && bad("https://a.b/x; rm -rf /", "us-east-1"));
        assert!(bad("https://a.example.com/start>/tmp/x", "us-east-1"));
        assert!(bad(START_URL, "us east") && bad(START_URL, ""));
        assert!(!bad(START_URL, "us-east-1"));
    }

    #[test]
    fn the_default_is_the_builds() {
        let default = AgentLoginRequest::default();
        assert!(default.validate().is_ok());
        assert_eq!(default.region, DEFAULT_REGION);
        match DEFAULT_START_URL {
            Some(start_url) => assert_eq!((default.mode, default.start_url.as_str()), (LoginMode::IdentityCenter, start_url)),
            None => assert_eq!((default.mode, default.start_url.as_str()), (LoginMode::BuilderId, "")),
        }
        #[cfg(not(feature = "internal"))]
        assert_eq!(DEFAULT_START_URL, None, "the public build has no built-in start URL");
        // An empty JSON object is the defaults too, but for the mode (an older viewer's request).
        let empty = request(json!({}));
        assert_eq!(empty, AgentLoginRequest { mode: LoginMode::IdentityCenter, ..default });
    }

    #[test]
    fn login_state_stays_readable_both_ways() {
        // An older backend's answer (no `preset`).
        let old: AgentLogin = serde_json::from_value(json!({"logged_in": false, "error": "x"})).unwrap();
        assert!(!old.preset);
        // `preset` only when set, so older viewers see the answer they know.
        assert_eq!(serde_json::to_value(AgentLogin::default()).unwrap(), json!({"logged_in": false}));
        let preset = AgentLogin { preset: true, ..AgentLogin::default() };
        assert_eq!(serde_json::to_value(&preset).unwrap(), json!({"logged_in": false, "preset": true}));
    }
}
