//! Logging in to a backend behind the workspace edge's login (CloudFront), natively. There every
//! request needs the edge's session cookie ([`SESSION_COOKIE`], `<alias>.<expiry>.<hmac>`);
//! without it the edge redirects to its login. The web viewer gets the cookie from the browser;
//! the native one gets it here, either
//! * silently, without a browser, where this build can ([`login_silently`]: the internal build's
//!   sign-in, in a module of its own; the public build has none), or
//! * through the browser ([`login_with_browser`]): the edge's `/oauth2/native` hands the session
//!   to a listener on this machine's loopback once the user has logged in.
//!
//! Sessions are kept per origin for the process (every request to that origin then carries the
//! cookie: [`cookie_header`]) and in the file set with [`set_store`], so a restart reuses them.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{LazyLock, Mutex, RwLock};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

pub const SESSION_COOKIE: &str = "ws_session";
/// Sessions this close to their expiry are renewed before use.
const RENEW_BEFORE: u64 = 60;
const BROWSER_TIMEOUT: Duration = Duration::from_secs(300);
/// Starts the errors [`crate::Api`] returns when the backend answered with the login instead.
pub const LOGIN_REQUIRED: &str = "login required";

static SESSIONS: LazyLock<RwLock<HashMap<String, String>>> = LazyLock::new(Default::default);
/// The workspace selected per origin ([`set_workspace`]); sent as the edge's selection cookie.
static WORKSPACES: LazyLock<RwLock<HashMap<String, String>>> = LazyLock::new(Default::default);
static STORE: Mutex<Option<PathBuf>> = Mutex::new(None);
/// One login at a time: requests that fail together renew the session once.
static LOGIN: Mutex<()> = Mutex::new(());

/// Why there is no session.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LoginError {
    /// The backend is behind the edge's login, but there is nothing to sign in with silently (no
    /// sign-in of the user's to go on, or an expired one): the browser would help, or what the
    /// message says.
    NoSilentSession(String),
    /// Anything else, e.g. the backend is not behind the edge's login at all, or this build has no
    /// silent login.
    Failed(String),
}

impl std::fmt::Display for LoginError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoSilentSession(why) | Self::Failed(why) => f.write_str(why),
        }
    }
}

impl std::error::Error for LoginError {}

/// Where sessions are saved between runs (the viewer's storage dir).
pub fn set_store(path: PathBuf) {
    *STORE.lock().unwrap() = Some(path);
}

/// `scheme://host[:port]` of `url`; WebSocket URLs count as their HTTP origin (`wss` → `https`),
/// the same cookies go to both.
pub fn origin(url: &str) -> Option<String> {
    let (scheme, rest) = url.split_once("://")?;
    let scheme = match scheme {
        "wss" => "https",
        "ws" => "http",
        scheme => scheme,
    };
    let host = rest.split(['/', '?', '#']).next().filter(|host| !host.is_empty())?;
    Some(format!("{scheme}://{host}"))
}

/// The `Cookie` header value for a request to `url`, if there is a session for its origin: the
/// session, and the selected workspace if one is ([`set_workspace`]).
pub fn cookie_header(url: &str) -> Option<String> {
    let origin = origin(url)?;
    let value = SESSIONS.read().unwrap().get(&origin).cloned()?;
    let mut header = format!("{SESSION_COOKIE}={value}");
    if let Some(workspace) = WORKSPACES.read().unwrap().get(&origin) {
        header.push_str(&format!("; {}={workspace}", crate::protocol::workspace::SELECT_COOKIE));
    }
    Some(header)
}

/// Select the workspace requests to `origin` go to (behind the edge, which checks it): `None`
/// for the user's own.
pub fn set_workspace(origin: &str, workspace: Option<&str>) {
    let mut workspaces = WORKSPACES.write().unwrap();
    match workspace.filter(|name| crate::protocol::workspace::is_valid_name(name)) {
        Some(name) => workspaces.insert(origin.to_owned(), name.to_owned()),
        None => workspaces.remove(origin),
    };
}

/// The workspace selected for `origin` with [`set_workspace`].
pub fn workspace(origin: &str) -> Option<String> {
    WORKSPACES.read().unwrap().get(origin).cloned()
}

/// The workspace user of a session value (`<alias>.<expiry>.<hmac>`).
pub fn alias(value: &str) -> Option<&str> {
    value.split('.').next().filter(|alias| !alias.is_empty())
}

fn expiry(value: &str) -> Option<u64> {
    value.split('.').nth(1)?.parse().ok()
}

pub(crate) fn now() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |since| since.as_secs())
}

/// Whether the session `value` is good for a while yet (not about to expire).
pub(crate) fn usable(value: &str) -> bool {
    expiry(value).is_some_and(|exp| exp > now() + RENEW_BEFORE)
}

/// The session for `origin`, if one is known (this run or saved) and not about to expire.
pub fn session(origin: &str) -> Option<String> {
    if let Some(value) = SESSIONS.read().unwrap().get(origin).filter(|value| usable(value)) {
        return Some(value.clone());
    }
    let value = load_saved().remove(origin).filter(|value| usable(value))?;
    SESSIONS.write().unwrap().insert(origin.to_owned(), value.clone());
    Some(value)
}

fn remember(origin: &str, value: &str) {
    SESSIONS.write().unwrap().insert(origin.to_owned(), value.to_owned());
    let Some(path) = STORE.lock().unwrap().clone() else { return };
    let mut saved = load_saved();
    saved.retain(|_, value| usable(value));
    saved.insert(origin.to_owned(), value.to_owned());
    if let Err(err) = write_private(&path, &serde_json::to_vec_pretty(&saved).unwrap_or_default()) {
        eprintln!("factoryos: failed to save the session to {}: {err}", path.display());
    }
}

fn load_saved() -> HashMap<String, String> {
    STORE
        .lock()
        .unwrap()
        .clone()
        .and_then(|path| std::fs::read(path).ok())
        .and_then(|bytes| serde_json::from_slice(&bytes).ok())
        .unwrap_or_default()
}

/// Sessions grant access to the user's workspace: only the user may read the file.
fn write_private(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let mut options = std::fs::OpenOptions::new();
    options.write(true).create(true).truncate(true);
    #[cfg(unix)]
    std::os::unix::fs::OpenOptionsExt::mode(&mut options, 0o600);
    std::io::Write::write_all(&mut options.open(path)?, bytes)
}

/// Renew the session for `url`'s origin after the backend asked for a login (it expired while the
/// viewer ran), silently ([`login_silently`]). Whether there is a fresh one now: never by itself
/// in a build without a silent login (the public one), only when another request renewed it.
pub fn renew(url: &str) -> bool {
    let Some(origin) = origin(url) else { return false };
    let stale = SESSIONS.read().unwrap().get(&origin).cloned();
    let _login = LOGIN.lock().unwrap();
    // Renewed by another request meanwhile.
    if SESSIONS.read().unwrap().get(&origin) != stale.as_ref() {
        return true;
    }
    login_silently(&origin).is_ok()
}

// --- Silent login -------------------------------------------------------------------------------

/// Log in to the backend at `origin` without a browser, where this build can: the internal build
/// has a silent login of its own (its internal module, not in the public export), which follows the
/// edge's login with the user's existing sign-in. The session is kept for `origin` (as
/// [`session`] finds it). Without a silent login (the public build), [`LoginError::Failed`]: the
/// backend is then used as it is, and [`login_with_browser`] is the way in. Blocking.
pub fn login_silently(origin: &str) -> Result<String, LoginError> {
    #[cfg(feature = "internal")]
    let found = crate::internal::login(origin);
    #[cfg(not(feature = "internal"))]
    let found: Result<String, LoginError> = Err(LoginError::Failed(format!("{origin}: no silent login in this build")));
    let value = found?;
    remember(origin, &value);
    Ok(value)
}

// --- Browser login ------------------------------------------------------------------------------

fn random_hex(bytes: usize) -> Result<String, String> {
    let mut buf = vec![0_u8; bytes];
    getrandom::fill(&mut buf).map_err(|err| format!("random state: {err}"))?;
    Ok(buf.iter().map(|byte| format!("{byte:02x}")).collect())
}

fn percent_decode(text: &str) -> String {
    let bytes = text.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            b'%' if i + 2 < bytes.len() => match std::str::from_utf8(&bytes[i + 1..i + 3])
                .ok()
                .and_then(|hex| u8::from_str_radix(hex, 16).ok())
            {
                Some(byte) => {
                    out.push(byte);
                    i += 3;
                    continue;
                }
                None => out.push(b'%'),
            },
            b'+' => out.push(b' '),
            byte => out.push(byte),
        }
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// `?a=1&b=2` of a request target → its value of `key`.
fn query_param(target: &str, key: &str) -> Option<String> {
    let query = target.split_once('?')?.1;
    query
        .split('&')
        .filter_map(|pair| pair.split_once('='))
        .find(|(name, _)| *name == key)
        .map(|(_, value)| percent_decode(value))
}

const CLOSE_TAB: &str = "<!doctype html><meta charset=utf-8><title>Factory OS</title>\
    <body style=\"font-family:sans-serif;background:#111;color:#ddd;padding:3em\">\
    <h2>Signed in to Factory OS</h2><p>You can close this tab and return to the viewer.</p>";

/// Log in to the backend at `origin` through the system browser: open the edge's
/// `/oauth2/native`, which after the edge's usual login redirects the browser to a listener on
/// this machine's loopback with the session. `open` opens a URL in the browser.
pub async fn login_with_browser(origin: &str, open: impl FnOnce(&str) -> Result<(), String>) -> Result<String, LoginError> {
    use tokio::io::{AsyncReadExt as _, AsyncWriteExt as _};

    let failed = |what: &str, err: &dyn std::fmt::Display| LoginError::Failed(format!("{what}: {err}"));
    let listener = tokio::net::TcpListener::bind(("127.0.0.1", 0)).await.map_err(|err| failed("login listener", &err))?;
    let port = listener.local_addr().map_err(|err| failed("login listener", &err))?.port();
    let state = random_hex(32).map_err(LoginError::Failed)?;
    open(&format!("{origin}/oauth2/native?port={port}&state={state}")).map_err(LoginError::Failed)?;

    let wait = async {
        loop {
            let (mut stream, _) = listener.accept().await.map_err(|err| failed("login listener", &err))?;
            let mut buf = vec![0_u8; 8192];
            let n = stream.read(&mut buf).await.unwrap_or(0);
            let head = String::from_utf8_lossy(&buf[..n]);
            let target = head.lines().next().and_then(|line| line.split(' ').nth(1)).unwrap_or("");
            let session = (target.starts_with("/callback?") && query_param(target, "state").as_deref() == Some(state.as_str()))
                .then(|| query_param(target, "session"))
                .flatten()
                .filter(|session| alias(session).is_some() && expiry(session).is_some());
            let (status, body) = if session.is_some() { ("200 OK", CLOSE_TAB) } else { ("400 Bad Request", "Bad request") };
            let reply = format!(
                "HTTP/1.1 {status}\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
                body.len()
            );
            stream.write_all(reply.as_bytes()).await.ok();
            stream.shutdown().await.ok();
            if let Some(session) = session {
                return Ok::<_, LoginError>(session);
            }
        }
    };
    let value = tokio::time::timeout(BROWSER_TIMEOUT, wait)
        .await
        .map_err(|_| LoginError::Failed("no login from the browser within 5 minutes".to_owned()))??;
    remember(origin, &value);
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sessions() {
        assert_eq!(alias("jdoe.1700000000.ab"), Some("jdoe"));
        assert_eq!(expiry("jdoe.1700000000.ab"), Some(1_700_000_000));
        assert!(!usable("jdoe.1700000000.ab"));
        assert!(usable(&format!("jdoe.{}.ab", now() + 3600)));
        assert_eq!(origin("https://h.example.com/workspace/x?y").as_deref(), Some("https://h.example.com"));
        assert_eq!(origin("wss://h.example.com/workspace/x/zenoh-native").as_deref(), Some("https://h.example.com"));
    }

    /// Without a silent login of its own the public build reports a plain failure, so the viewer
    /// uses the backend as it is (and renews nothing by itself).
    #[cfg(not(feature = "internal"))]
    #[test]
    fn no_silent_login_without_the_internal_build() {
        let origin = "https://silent.example.com";
        assert!(matches!(login_silently(origin), Err(LoginError::Failed(_))));
        assert_eq!(session(origin), None);
        assert!(!renew(&format!("{origin}/api/v1/health")));
    }

    #[test]
    fn selects_workspaces() {
        let origin = "https://select.example.com";
        remember_in_memory(origin, "jdoe.1.ab");
        assert_eq!(cookie_header(&format!("{origin}/api/v1/health")).as_deref(), Some("ws_session=jdoe.1.ab"));
        set_workspace(origin, Some("ci"));
        assert_eq!(cookie_header(&format!("{origin}/chat")).as_deref(), Some("ws_session=jdoe.1.ab; fos_ws=ci"));
        set_workspace(origin, Some("a;b"));
        assert_eq!(workspace(origin), None);
        set_workspace(origin, None);
        assert_eq!(cookie_header(origin).as_deref(), Some("ws_session=jdoe.1.ab"));
    }

    fn remember_in_memory(origin: &str, value: &str) {
        SESSIONS.write().unwrap().insert(origin.to_owned(), value.to_owned());
    }

    #[test]
    fn callback_query() {
        let target = "/callback?state=ab12&session=jdoe.1.f%30";
        assert_eq!(query_param(target, "state").as_deref(), Some("ab12"));
        assert_eq!(query_param(target, "session").as_deref(), Some("jdoe.1.f0"));
        assert_eq!(query_param(target, "nope"), None);
    }
}
