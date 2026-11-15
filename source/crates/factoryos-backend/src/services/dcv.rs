//! NICE DCV: the Windows IDE's CODESYS in the browser, through this backend, with no Windows
//! password. The backend machine runs a DCV Connection Gateway on loopback
//! (`plan2cell/infra/test-server/dcv-gateway.sh`), and this module is the rest:
//!
//! * [`api::DCV`]`/<target>/…` (also under `/workspace/<name>/`): a reverse proxy to the
//!   target's gateway. A target ([`Target`]) is a DCV server's session behind a gateway of this
//!   machine: `ide` (the Windows IDE's console) is the gateway `FACTORY_OS_DCV_GATEWAY` (loopback
//!   only; `https` with its self-signed certificate pinned by `FACTORY_OS_DCV_GATEWAY_CERT`),
//!   further ones come from `FACTORY_OS_DCV_TARGETS` (each with a gateway and resolver of its own,
//!   e.g. a smartPAD's). HTTP and WebSocket upgrades (spliced byte for byte, so the subprotocol and
//!   extensions are the client's and the gateway's). `/dcv/<target>/x` is the target gateway's `/x`:
//!   the DCV web client builds its URLs from the page's own path, so the prefix needs no rewriting
//!   in its HTML or JS. `/dcv/x` (no target first) is `ide`'s `/x`, for older links and tokens. The edge's cookies (`ws_session`, `fos_ws`, `oidc_*`) and headers
//!   (`x-amzn-*`, `x-origin-verify`) stay here; `Host` and `Origin` become the gateway's own, as a
//!   browser opening it directly would send them, so the origin is checked here first.
//! * `POST` [`api::INFRA_DCV_SESSION`] (`Content-Type: application/json`, optional body
//!   `{"target": "ide"}`): a random token for the target's session and the URL that opens it:
//!   `/dcv/ide/?authToken=<owner>.<hex>#console` (the viewer pins it to `/workspace/<owner>`).
//!   With a token in the URL the web client skips its login form. A token starts a connection within
//!   [`START_TTL`]; its first accepted connection binds it to that client's address, which may
//!   reconnect with it for [`SESSION_TTL`]. In memory: `DELETE` (or a restart) revokes them all.
//!   Tokens are kept across restarts (`FACTORY_OS_DCV_TOKEN_STORE`). A token's first accepted
//!   connection to `ide` (the Open PLC IDE window connecting) also runs [`DcvService::on_open`]'s
//!   hook: the backend brings CODESYS to the front of that desktop
//!   ([`crate::services::infra::InfraService::show_ide`]).
//! * Only the page's own origin gets in ([`DcvService::admit`]; these routes are outside the
//!   backend's CORS layer): `Sec-Fetch-Site` `same-origin` / `none`, else an `Origin` that is
//!   the request's `Host` (or `X-Forwarded-Host`); top-level navigations to `/dcv/` excepted (no
//!   token in them is any use to another site). Behind the workspace edge, its `X-User-Alias` must
//!   be the workspace's owner (`--workspace`) or in `FACTORY_OS_DCV_USERS`.
//! * The token verifier (`FACTORY_OS_DCV_VERIFIER_BIND`, default `127.0.0.1:8445`: loopback unless
//!   the deployment opens it, as dcv-gateway.sh does with `0.0.0.0:8445`, the security groups
//!   letting only the Windows IDE reach it; only private addresses are answered): the Windows DCV
//!   server's `auth-token-verifier`. DCV posts `sessionId` and `authenticationToken`
//!   and gets `<auth result="yes"><username>Administrator</username></auth>` (the console
//!   session's owner, `FACTORY_OS_DCV_USER`) or `result="no"`. Its own tokens: the session must be
//!   the token's target's, the DCV server asking the host the resolver named for it (when known),
//!   and the first client (the gateway DCV reports) one of `FACTORY_OS_DCV_GATEWAY_IPS` (unset:
//!   any). One DCV server has one verifier URL, so backends sharing it chain: a token
//!   `<alias>.<hex>` of another workspace goes to that workspace's verifier when `<alias>` is in
//!   `FACTORY_OS_DCV_VERIFIER_PEERS` (`alias=http://<private ip>:<port>/,…`; fixed URLs only),
//!   which checks it in full (with the DCV server's address as `dcvServer`); its yes / no is
//!   answered here (unknown aliases: no). A chained question (one with `dcvServer`) is about the
//!   asked backend's own tokens only: it is never chained on.
//! * The gateway's session resolver (`FACTORY_OS_DCV_RESOLVER_BIND`, default `127.0.0.1:8446`,
//!   loopback; one per gateway): `POST /resolveSession?sessionId=console&transport=HTTP` → the
//!   target of that resolver with that session: for `ide` the Windows IDE's private address from
//!   the cell-control Lambda's status, else `FACTORY_OS_DCV_SERVER`; port 8443, `/`.
//!
//! Settings come from the environment and `/etc/factoryos/dcv.env` (written by dcv-gateway.sh;
//! `FACTORY_OS_DCV_ENV` overrides its path). Without a target (`FACTORY_OS_DCV_GATEWAY`,
//! `FACTORY_OS_DCV_TARGETS`) none of it runs: `/dcv/` and the session are `503`, and no listener
//! opens.

use std::collections::HashMap;
use std::future::Future;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::path::PathBuf;
use std::pin::Pin;
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::Context as _;
use axum::body::{Body, Bytes};
use axum::extract::{ConnectInfo, Query, Request, State};
use axum::http::{HeaderMap, HeaderName, HeaderValue, StatusCode, Uri, Version, header};
use axum::response::{IntoResponse as _, Response};
use axum::routing::{any, post};
use axum::{Form, Json, Router};
use factoryos_protocol::infra::{DCV_CONSOLE_SESSION, DCV_TARGET_IDE, DcvRevoked, DcvSession, DcvSessionRequest};
use factoryos_protocol::{api, workspace};
use hyper_util::rt::TokioIo;
use serde::Deserialize;
use serde_json::{Value, json};
use tokio::io::{AsyncRead, AsyncWrite};
use tokio::net::TcpStream;

use crate::activity::Activity;

/// The env file dcv-gateway.sh writes.
pub const DCV_ENV: &str = "/etc/factoryos/dcv.env";
/// How long a new session token can start a connection.
pub const START_TTL: Duration = Duration::from_secs(5 * 60);
/// How long a used token lets its client (the same address) reconnect.
pub const SESSION_TTL: Duration = Duration::from_secs(8 * 3600);
/// The Windows DCV server's port (plan2cell versions.psd1 `Dcv.Port`).
pub const DCV_SERVER_PORT: u16 = 8443;
/// Tokens kept at most (the oldest go first).
const MAX_TOKENS: usize = 256;
/// Connecting to the gateway (loopback).
const CONNECT_TIMEOUT: Duration = Duration::from_secs(10);
/// A peer verifier's answer (DCV waits for this one's).
const CHAIN_TIMEOUT: Duration = Duration::from_secs(3);
/// The DCV web client's top-level paths: no target may be named like one (`/dcv/<x>/…` without a
/// target is `ide`'s).
const RESERVED_TARGETS: [&str; 16] =
    ["auth", "js", "css", "lib", "libs", "images", "img", "locales", "fonts", "dcvjs", "dcv", "sdk", "static", "assets", "resources", "www"];

type HttpError = (StatusCode, String);

/// The DCV Connection Gateway, on loopback.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Gateway {
    pub tls: bool,
    pub addr: SocketAddr,
}

impl Gateway {
    /// `https://127.0.0.1:8443` (or `http://`, `localhost`): loopback only, as the proxy accepts the
    /// gateway's self-signed certificate.
    pub fn parse(url: &str) -> Result<Self, String> {
        let url = url.trim();
        let (tls, rest) = if let Some(rest) = url.strip_prefix("https://") {
            (true, rest)
        } else if let Some(rest) = url.strip_prefix("http://") {
            (false, rest)
        } else {
            return Err(format!("FACTORY_OS_DCV_GATEWAY {url:?} is not an http(s):// URL"));
        };
        let rest = rest.trim_end_matches('/');
        let addr = match rest.strip_prefix("localhost:") {
            Some(port) => port.parse().map(|port| SocketAddr::from((Ipv4Addr::LOCALHOST, port))).ok(),
            None => rest.parse::<SocketAddr>().ok(),
        };
        match addr {
            Some(addr) if addr.ip().is_loopback() => Ok(Self { tls, addr }),
            Some(_) => Err(format!("FACTORY_OS_DCV_GATEWAY {url:?} is not on loopback")),
            None => Err(format!("FACTORY_OS_DCV_GATEWAY {url:?} is not <scheme>://<ip>:<port>")),
        }
    }

    pub(crate) fn origin(&self) -> String {
        format!("{}://{}", if self.tls { "https" } else { "http" }, self.addr)
    }
}

/// Where a target's DCV server is.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HostSource {
    /// The Windows IDE's private address (the cell-control status), else `FACTORY_OS_DCV_SERVER`.
    IdeLookup,
    Static(String),
}

/// A DCV session served at `/dcv/<name>/`: a DCV server's session behind a gateway of this machine.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Target {
    pub name: String,
    pub gateway: Gateway,
    /// The gateway's pinned certificate (`None`: any; loopback).
    pub cert: Option<PathBuf>,
    /// The resolver that gateway asks (loopback).
    pub resolver: SocketAddr,
    /// The DCV session on the server.
    pub session: String,
    pub host: HostSource,
    pub port: u16,
}

/// A target name: `^[a-z][a-z0-9-]{0,31}$`, none of the web client's own paths.
pub fn valid_target_name(name: &str) -> bool {
    let mut chars = name.chars();
    chars.next().is_some_and(|c| c.is_ascii_lowercase())
        && name.len() <= 32
        && chars.all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
        && !RESERVED_TARGETS.contains(&name)
}

/// A DCV session id as the resolver and verifier take it.
fn valid_session(session: &str) -> bool {
    !session.is_empty() && session.len() <= 64 && session.chars().all(|c| c.is_ascii_alphanumeric() || "._-".contains(c))
}

#[derive(Clone, Debug)]
pub struct Settings {
    /// The `ide` target's gateway (`FACTORY_OS_DCV_GATEWAY`).
    pub gateway: Option<Gateway>,
    /// The gateway's certificate (PEM): the only one the proxy accepts. `None`: any (loopback).
    pub gateway_cert: Option<PathBuf>,
    pub resolver_bind: SocketAddr,
    pub verifier_bind: SocketAddr,
    /// The DCV server when the cell-control status has no running Windows IDE.
    pub server: Option<String>,
    pub server_port: u16,
    /// The console session's owner, as the verifier names it.
    pub user: String,
    pub start_ttl: Duration,
    pub session_ttl: Duration,
    /// The workspace's owner (the backend's `--workspace`): the edge's `X-User-Alias` that may use DCV.
    pub owner: Option<String>,
    /// Where the tokens are kept across backend restarts (a deploy), so an open DCV window or a link the
    /// viewer fetched ahead still works. `None`: in memory only.
    pub token_store: Option<PathBuf>,
    /// Further aliases that may (`FACTORY_OS_DCV_USERS`, comma-separated).
    pub users: Vec<String>,
    /// Targets besides the default `ide` (`FACTORY_OS_DCV_TARGETS`; one named `ide` replaces it).
    pub targets: Vec<Target>,
    /// Other workspaces' verifiers, by alias (`FACTORY_OS_DCV_VERIFIER_PEERS`): `http://<ip>:<port>/`.
    pub peers: Vec<(String, String)>,
    /// The gateways' addresses DCV may report as a token's first client
    /// (`FACTORY_OS_DCV_GATEWAY_IPS`; `None`: any).
    pub gateway_ips: Option<Vec<IpAddr>>,
    /// How long a peer verifier may take ([`CHAIN_TIMEOUT`]).
    pub chain_timeout: Duration,
    /// Settings ignored (logged at start).
    pub warnings: Vec<String>,
    /// Why DCV is off although asked for (a bad setting).
    pub error: Option<String>,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            gateway: None,
            gateway_cert: None,
            resolver_bind: SocketAddr::from((Ipv4Addr::LOCALHOST, 8446)),
            // Loopback: the Windows IDE reaches it only where the deployment binds it wider.
            verifier_bind: SocketAddr::from((Ipv4Addr::LOCALHOST, 8445)),
            server: None,
            server_port: DCV_SERVER_PORT,
            user: "Administrator".to_owned(),
            start_ttl: START_TTL,
            session_ttl: SESSION_TTL,
            owner: None,
            token_store: None,
            users: Vec::new(),
            targets: Vec::new(),
            peers: Vec::new(),
            gateway_ips: None,
            chain_timeout: CHAIN_TIMEOUT,
            warnings: Vec::new(),
            error: None,
        }
    }
}

impl Settings {
    /// From the environment and [`DCV_ENV`] (the environment wins).
    pub fn from_env() -> Self {
        let file = std::env::var_os("FACTORY_OS_DCV_ENV").map_or_else(|| PathBuf::from(DCV_ENV), PathBuf::from);
        let from_file = std::fs::read_to_string(file).map(|text| crate::services::infra::parse_env_file(&text)).unwrap_or_default();
        Self::from_vars(|name| {
            std::env::var(name)
                .ok()
                .filter(|v| !v.trim().is_empty())
                .or_else(|| from_file.iter().find(|(k, _)| k == name).map(|(_, v)| v.clone()))
                .map(|v| v.trim().to_owned())
                .filter(|v| !v.is_empty())
        })
    }

    fn from_vars(var: impl Fn(&str) -> Option<String>) -> Self {
        let mut settings = Self::default();
        let mut errors = Vec::new();
        if let Some(url) = var("FACTORY_OS_DCV_GATEWAY") {
            match Gateway::parse(&url) {
                Ok(gateway) => settings.gateway = Some(gateway),
                Err(err) => errors.push(err),
            }
        }
        settings.gateway_cert = var("FACTORY_OS_DCV_GATEWAY_CERT").map(PathBuf::from);
        let mut bind = |name: &str, into: &mut SocketAddr| {
            if let Some(value) = var(name) {
                match value.parse() {
                    Ok(addr) => *into = addr,
                    Err(_) => errors.push(format!("{name} {value:?} is not <ip>:<port>")),
                }
            }
        };
        bind("FACTORY_OS_DCV_RESOLVER_BIND", &mut settings.resolver_bind);
        bind("FACTORY_OS_DCV_VERIFIER_BIND", &mut settings.verifier_bind);
        if !settings.resolver_bind.ip().is_loopback() {
            errors.push(format!("FACTORY_OS_DCV_RESOLVER_BIND {} is not on loopback", settings.resolver_bind));
        }
        settings.server = var("FACTORY_OS_DCV_SERVER").filter(|host| valid_host(host));
        if let Some(port) = var("FACTORY_OS_DCV_SERVER_PORT") {
            match port.parse() {
                Ok(port) => settings.server_port = port,
                Err(_) => errors.push(format!("FACTORY_OS_DCV_SERVER_PORT {port:?} is not a port")),
            }
        }
        if let Some(user) = var("FACTORY_OS_DCV_USER").filter(|u| u.chars().all(|c| c.is_ascii_alphanumeric() || "._-".contains(c))) {
            settings.user = user;
        }
        settings.token_store = var("FACTORY_OS_DCV_TOKEN_STORE").filter(|p| !p.is_empty()).map(PathBuf::from).or_else(|| {
            let data = std::path::Path::new(DATA_DIR);
            data.is_dir().then(|| data.join("dcv-tokens.json"))
        });
        settings.users = var("FACTORY_OS_DCV_USERS")
            .map(|users| users.split(',').map(str::trim).filter(|u| !u.is_empty()).map(str::to_owned).collect())
            .unwrap_or_default();
        if let Some(spec) = var("FACTORY_OS_DCV_TARGETS") {
            for entry in spec.split([';', '\n']).map(str::trim).filter(|e| !e.is_empty()) {
                match parse_target(entry, &settings) {
                    Ok(target) => settings.targets.push(target),
                    Err(err) => errors.push(format!("FACTORY_OS_DCV_TARGETS {entry:?}: {err}")),
                }
            }
        }
        if let Some(peers) = var("FACTORY_OS_DCV_VERIFIER_PEERS") {
            for entry in peers.split(',').map(str::trim).filter(|e| !e.is_empty()) {
                match parse_peer(entry) {
                    Ok(peer) if settings.peers.iter().any(|(alias, _)| *alias == peer.0) => {
                        settings.warnings.push(format!("FACTORY_OS_DCV_VERIFIER_PEERS: {} twice (the first counts)", peer.0))
                    }
                    Ok(peer) => settings.peers.push(peer),
                    Err(err) => settings.warnings.push(format!("FACTORY_OS_DCV_VERIFIER_PEERS {entry:?} ignored: {err}")),
                }
            }
        }
        if let Some(ips) = var("FACTORY_OS_DCV_GATEWAY_IPS") {
            let parsed: Result<Vec<IpAddr>, String> = ips
                .split(',')
                .map(str::trim)
                .filter(|ip| !ip.is_empty())
                .map(|ip| ip.parse::<IpAddr>().map(|ip| ip.to_canonical()).map_err(|_| format!("FACTORY_OS_DCV_GATEWAY_IPS: {ip:?} is not an IP address")))
                .collect();
            match parsed {
                Ok(ips) if ips.is_empty() => errors.push("FACTORY_OS_DCV_GATEWAY_IPS is empty".to_owned()),
                Ok(ips) => settings.gateway_ips = Some(ips),
                Err(err) => errors.push(err),
            }
        }
        if let Err(err) = check_targets(&settings.all_targets()) {
            errors.push(err);
        }
        if !errors.is_empty() {
            settings.gateway = None;
            settings.targets.clear();
            settings.error = Some(errors.join("; "));
        }
        settings
    }

    /// Every target: `ide` from [`Self::gateway`] (unless [`Self::targets`] names one), then the rest.
    pub fn all_targets(&self) -> Vec<Target> {
        let mut all: Vec<Target> = Vec::new();
        if let Some(gateway) = &self.gateway
            && !self.targets.iter().any(|t| t.name == DCV_TARGET_IDE)
        {
            all.push(Target {
                name: DCV_TARGET_IDE.to_owned(),
                gateway: gateway.clone(),
                cert: self.gateway_cert.clone(),
                resolver: self.resolver_bind,
                session: DCV_CONSOLE_SESSION.to_owned(),
                host: HostSource::IdeLookup,
                port: self.server_port,
            });
        }
        all.extend(self.targets.iter().cloned());
        all.sort_by_key(|t| t.name != DCV_TARGET_IDE);
        all
    }
}

/// `name=gateway:https://127.0.0.1:8453,resolver:127.0.0.1:8456,session:console,server:10.0.0.5[,port:8443][,cert:/path]`;
/// absent: the `ide` gateway, its resolver, `console`, the Windows IDE, `FACTORY_OS_DCV_SERVER_PORT`.
fn parse_target(entry: &str, settings: &Settings) -> Result<Target, String> {
    let (name, spec) = entry.split_once('=').map_or((entry, ""), |(n, s)| (n.trim(), s));
    if !valid_target_name(name) {
        return Err(format!("{name:?} is no target name ([a-z][a-z0-9-]{{0,31}}, not one of {})", RESERVED_TARGETS.join(", ")));
    }
    let mut target = Target {
        name: name.to_owned(),
        gateway: settings.gateway.clone().ok_or("no gateway: and no FACTORY_OS_DCV_GATEWAY")?,
        cert: None,
        resolver: settings.resolver_bind,
        session: DCV_CONSOLE_SESSION.to_owned(),
        host: HostSource::IdeLookup,
        port: settings.server_port,
    };
    let mut own_gateway = false;
    for part in spec.split(',').map(str::trim).filter(|p| !p.is_empty()) {
        let (key, value) = part.split_once(':').ok_or_else(|| format!("{part:?} is not key:value"))?;
        let value = value.trim();
        match key.trim() {
            "gateway" => {
                target.gateway = Gateway::parse(value)?;
                own_gateway = true;
            }
            "resolver" => {
                target.resolver = value.parse().map_err(|_| format!("resolver {value:?} is not <ip>:<port>"))?;
                if !target.resolver.ip().is_loopback() {
                    return Err(format!("resolver {value} is not on loopback"));
                }
            }
            "session" if valid_session(value) => target.session = value.to_owned(),
            "server" if valid_host(value) => target.host = HostSource::Static(value.to_owned()),
            "port" => target.port = value.parse().map_err(|_| format!("port {value:?} is not a port"))?,
            "cert" if !value.is_empty() => target.cert = Some(PathBuf::from(value)),
            other => return Err(format!("{other}:{value:?} is not gateway, resolver, session, server, port or cert")),
        }
    }
    if !own_gateway {
        target.cert = settings.gateway_cert.clone();
    }
    Ok(target)
}

/// Targets that a gateway / resolver can tell apart: unique names, one resolver per gateway, and
/// no two with the same session on one gateway.
fn check_targets(targets: &[Target]) -> Result<(), String> {
    for (i, a) in targets.iter().enumerate() {
        for b in &targets[i + 1..] {
            if a.name == b.name {
                return Err(format!("FACTORY_OS_DCV_TARGETS: {} twice", a.name));
            }
            if a.gateway == b.gateway && a.resolver != b.resolver {
                return Err(format!("FACTORY_OS_DCV_TARGETS: {} and {} share a gateway but not its resolver", a.name, b.name));
            }
            if (a.gateway == b.gateway || a.resolver == b.resolver) && a.session == b.session {
                return Err(format!(
                    "FACTORY_OS_DCV_TARGETS: {} and {} have the same session {:?} on one gateway / resolver: give one its own",
                    a.name, b.name, a.session
                ));
            }
        }
    }
    Ok(())
}

/// `alias=http://<private ip>:<port>/` → (`alias`, the URL as given, with its `/`).
fn parse_peer(entry: &str) -> Result<(String, String), String> {
    let (alias, url) = entry.split_once('=').ok_or("not alias=url")?;
    let (alias, url) = (alias.trim(), url.trim());
    if !workspace::is_valid_name(alias) {
        return Err(format!("{alias:?} is not a workspace name"));
    }
    let addr = url.strip_prefix("http://").map(|rest| rest.strip_suffix('/').unwrap_or(rest)).ok_or("not http://<ip>:<port>/")?;
    let addr: SocketAddr = addr.parse().map_err(|_| "not http://<ip>:<port>/".to_owned())?;
    if !private(addr.ip()) || addr.ip().is_loopback() {
        return Err(format!("{} is not a private address", addr.ip()));
    }
    Ok((alias.to_owned(), format!("http://{addr}/")))
}

fn unix_s(t: SystemTime) -> f64 {
    t.duration_since(UNIX_EPOCH).unwrap_or_default().as_secs_f64()
}

/// The unexpired tokens of a token store (none when it is missing or unreadable).
fn load_tokens(path: &std::path::Path) -> HashMap<String, Token> {
    let Ok(text) = std::fs::read_to_string(path) else { return HashMap::new() };
    let Ok(Value::Object(entries)) = serde_json::from_str::<Value>(&text) else { return HashMap::new() };
    let now = SystemTime::now();
    entries
        .into_iter()
        .filter_map(|(token, v)| {
            let expires = UNIX_EPOCH + Duration::from_secs_f64(v.get("expires_s")?.as_f64()?.max(0.0));
            let session = v.get("session")?.as_str()?.to_owned();
            let client = v.get("client").and_then(Value::as_str).map(str::to_owned);
            // Older stores have no target: theirs was the console, `ide`.
            let target = v.get("target").and_then(Value::as_str).unwrap_or(DCV_TARGET_IDE).to_owned();
            (expires > now).then_some((token, Token { target, session, expires, client }))
        })
        .collect()
}

fn save_tokens(path: &std::path::Path, tokens: &HashMap<String, Token>) -> anyhow::Result<()> {
    let entries: serde_json::Map<String, Value> = tokens
        .iter()
        .map(|(k, t)| (k.clone(), json!({"target": t.target, "session": t.session, "expires_s": unix_s(t.expires), "client": t.client})))
        .collect();
    let tmp = path.with_extension("json.tmp");
    {
        use std::io::Write as _;
        let mut options = std::fs::OpenOptions::new();
        options.write(true).create(true).truncate(true);
        #[cfg(unix)]
        std::os::unix::fs::OpenOptionsExt::mode(&mut options, 0o600);
        let mut file = options.open(&tmp).with_context(|| format!("open {}", tmp.display()))?;
        file.write_all(Value::Object(entries).to_string().as_bytes())?;
    }
    std::fs::rename(&tmp, path).with_context(|| format!("rename to {}", path.display()))?;
    Ok(())
}

/// A host name or IP address, as DCV takes it.
fn valid_host(host: &str) -> bool {
    !host.is_empty() && host.len() <= 253 && host.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '.' | '-' | ':'))
}

/// Run when a token's first connection is accepted (it must not block: spawn).
pub type OpenHook = Arc<dyn Fn() + Send + Sync>;

/// The Windows IDE's private address (the cell-control Lambda's status), if it runs.
pub type IdeLookup = Arc<dyn Fn() -> Pin<Box<dyn Future<Output = Option<String>> + Send>> + Send + Sync>;

/// The backend's data directory on a server (`deploy/provision.sh`); the default token store's home.
const DATA_DIR: &str = "/var/lib/factoryos";

struct Token {
    /// The target it opens.
    target: String,
    session: String,
    /// Until then it starts a connection (unused) or reconnects (used).
    expires: SystemTime,
    /// The client address its first connection came from (it is bound to it from then on).
    client: Option<String>,
}

/// A target and the TLS client for its gateway.
struct Upstream {
    target: Target,
    tls: Option<tokio_rustls::TlsConnector>,
}

/// Where a request under `/dcv/` goes.
#[derive(Debug, PartialEq, Eq)]
pub enum Route {
    /// To target `0`'s gateway, at path (and query) `1`.
    Proxy(String, String),
    /// `/dcv/<target>` → `/dcv/<target>/` (with the query).
    Slash(String, String),
}

pub struct DcvService {
    settings: Settings,
    upstreams: Vec<Upstream>,
    tokens: Mutex<HashMap<String, Token>>,
    ide: IdeLookup,
    activity: Arc<Activity>,
    /// Why the proxy is off (for its `503`).
    unavailable: Option<String>,
    on_open: Mutex<Option<OpenHook>>,
    /// The DCV server the resolver last named for each target: only it may redeem their tokens.
    resolved: Mutex<HashMap<String, String>>,
    /// For the peers' verifiers (no proxy, no redirects, [`Settings::chain_timeout`]).
    chain: reqwest::Client,
}

impl DcvService {
    pub fn new(settings: Settings, ide: IdeLookup, activity: Arc<Activity>) -> Arc<Self> {
        let mut unavailable = settings.error.clone();
        let mut upstreams = Vec::new();
        if unavailable.is_none() {
            for target in settings.all_targets() {
                let mut tls = None;
                // A pinned certificate that can't be read must not mean "accept any".
                if target.gateway.tls {
                    match tls_connector(target.cert.as_ref()) {
                        Ok(connector) => tls = Some(connector),
                        Err(err) => {
                            unavailable = Some(format!("{err:#}"));
                            break;
                        }
                    }
                }
                upstreams.push(Upstream { target, tls });
            }
            if unavailable.is_none() && upstreams.is_empty() {
                unavailable = Some(
                    "NICE DCV is not set up on this server: no DCV Connection Gateway (FACTORY_OS_DCV_GATEWAY; \
                     plan2cell/infra/test-server/dcv-gateway.sh)"
                        .to_owned(),
                );
            }
        }
        if unavailable.is_some() {
            upstreams.clear();
        }
        let tokens = Mutex::new(settings.token_store.as_deref().map(load_tokens).unwrap_or_default());
        let chain = reqwest::Client::builder()
            .timeout(settings.chain_timeout)
            .connect_timeout(settings.chain_timeout)
            .redirect(reqwest::redirect::Policy::none())
            .no_proxy()
            .build()
            .unwrap_or_default();
        Arc::new(Self {
            settings,
            upstreams,
            tokens,
            ide,
            activity,
            unavailable,
            on_open: Mutex::default(),
            resolved: Mutex::default(),
            chain,
        })
    }

    /// Keep the tokens in [`Settings::token_store`] (best effort: a failure only costs them on a restart).
    fn store(&self, tokens: &HashMap<String, Token>) {
        let Some(path) = &self.settings.token_store else { return };
        if let Err(err) = save_tokens(path, tokens) {
            tracing::warn!("NICE DCV: can't keep the tokens in {}: {err:#}", path.display());
        }
    }

    /// Run `hook` whenever a token's first connection to `ide` is accepted (a new Open PLC IDE
    /// window), not on its reconnects.
    pub fn on_open(&self, hook: OpenHook) {
        *self.on_open.lock().unwrap() = Some(hook);
    }

    /// The proxy, sessions and listeners work.
    pub fn enabled(&self) -> bool {
        self.unavailable.is_none()
    }

    /// There is a DCV server to use when the cell's status has no Windows IDE.
    pub fn has_fallback_server(&self) -> bool {
        self.enabled() && self.settings.server.is_some()
    }

    /// The targets served at `/dcv/<target>/` (none while off).
    pub fn target_names(&self) -> Vec<String> {
        self.upstreams.iter().map(|u| u.target.name.clone()).collect()
    }

    fn upstream(&self, name: &str) -> Result<&Upstream, HttpError> {
        if let Some(why) = &self.unavailable {
            return Err((StatusCode::SERVICE_UNAVAILABLE, why.clone()));
        }
        self.upstreams
            .iter()
            .find(|u| u.target.name == name)
            .ok_or_else(|| (StatusCode::NOT_FOUND, format!("no NICE DCV target {name:?} on this server")))
    }

    /// Open the resolver (one per gateway) and verifier listeners (when enabled).
    pub fn start(self: &Arc<Self>) {
        for warning in &self.settings.warnings {
            tracing::warn!("NICE DCV: {warning}");
        }
        if let Some(why) = &self.unavailable {
            if self.settings.gateway.is_none() && self.settings.targets.is_empty() && self.settings.error.is_none() {
                tracing::info!("NICE DCV: no gateway (FACTORY_OS_DCV_GATEWAY)");
            } else {
                tracing::warn!("NICE DCV off: {why}");
            }
            return;
        }
        for u in &self.upstreams {
            tracing::info!("NICE DCV: /dcv/{}/ → {} (resolver {}, session {:?})", u.target.name, u.target.gateway.origin(), u.target.resolver, u.target.session);
        }
        tracing::info!(
            "NICE DCV: token verifier {}, peers [{}], gateway IPs {}",
            self.settings.verifier_bind,
            self.settings.peers.iter().map(|(a, u)| format!("{a}={u}")).collect::<Vec<_>>().join(", "),
            self.settings.gateway_ips.as_ref().map_or("any".to_owned(), |ips| ips.iter().map(IpAddr::to_string).collect::<Vec<_>>().join(", "))
        );
        let mut resolvers: Vec<SocketAddr> = self.upstreams.iter().map(|u| u.target.resolver).collect();
        resolvers.sort();
        resolvers.dedup();
        let mut listeners: Vec<(&str, SocketAddr, Router)> =
            resolvers.into_iter().map(|addr| ("resolver", addr, resolver_router().with_state((self.clone(), addr)))).collect();
        listeners.push(("token verifier", self.settings.verifier_bind, verifier_router().with_state(self.clone())));
        for (what, addr, router) in listeners {
            let what = what.to_owned();
            tokio::spawn(async move {
                let listener = match tokio::net::TcpListener::bind(addr).await {
                    Ok(listener) => listener,
                    Err(err) => {
                        tracing::error!("NICE DCV {what}: cannot listen on {addr}: {err}");
                        return;
                    }
                };
                if let Err(err) = axum::serve(listener, router.into_make_service_with_connect_info::<SocketAddr>()).await {
                    tracing::error!("NICE DCV {what} stopped: {err}");
                }
            });
        }
    }

    /// A new token for `target`'s session, and the URL that opens it (`<owner>.<hex>`: a peer's
    /// verifier knows whose it is).
    pub fn mint(&self, target: &str) -> Result<DcvSession, HttpError> {
        let session = self.upstream(target)?.target.session.clone();
        let mut bytes = [0u8; 32];
        getrandom::fill(&mut bytes).map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, format!("no randomness: {err}")))?;
        let hex: String = bytes.iter().map(|b| format!("{b:02x}")).collect();
        let token = match &self.settings.owner {
            Some(owner) => format!("{owner}.{hex}"),
            None => hex,
        };
        let now = SystemTime::now();
        let expires = now + self.settings.start_ttl;
        {
            let mut tokens = self.tokens.lock().unwrap();
            tokens.retain(|_, t| t.expires > now);
            while tokens.len() >= MAX_TOKENS {
                let Some(oldest) = tokens.iter().min_by_key(|(_, t)| t.expires).map(|(k, _)| k.clone()) else { break };
                tokens.remove(&oldest);
            }
            tokens.insert(token.clone(), Token { target: target.to_owned(), session: session.clone(), expires, client: None });
            self.store(&tokens);
        }
        Ok(DcvSession {
            url: format!("{}?authToken={token}#{session}", api::dcv_target(target)),
            expires_s: unix_s(expires),
            issued_s: unix_s(now),
            workspace: self.settings.owner.clone(),
            target: target.to_owned(),
        })
    }

    /// The workspace a token is of, when it is another's (`<alias>.<hex>`; this backend's own
    /// tokens have its owner's prefix, or none).
    fn foreign<'t>(&self, token: &'t str) -> Option<&'t str> {
        let (prefix, _) = token.rsplit_once('.')?;
        (self.settings.owner.as_deref() != Some(prefix)).then_some(prefix)
    }

    /// Whether DCV may let this backend's `token` into `session` for `client` (the address DCV
    /// reports: the gateway's), asked by the DCV server `server`. The first yes binds the token to
    /// that address, for [`Settings::session_ttl`].
    pub fn verify(&self, session: &str, token: &str, client: Option<&str>, server: Option<&str>) -> bool {
        if self.foreign(token).is_some() {
            return false;
        }
        let now = SystemTime::now();
        let client = client_ip(client.unwrap_or_default());
        let target = {
            let mut tokens = self.tokens.lock().unwrap();
            let Some(t) = tokens.get_mut(token) else { return false };
            if t.session != session || t.expires <= now {
                return false;
            }
            // Only the DCV server the resolver named for its target redeems it.
            if let (Some(server), Some(host)) = (server.map(client_ip), self.known_host(&t.target))
                && host.parse::<IpAddr>().is_ok()
                && client_ip(&host) != server
            {
                tracing::warn!("NICE DCV: a token for {} from the DCV server {server}, not {host}", t.target);
                return false;
            }
            if let Some(bound) = &t.client {
                return *bound == client;
            }
            // Its first connection comes through a gateway of this machine.
            if let Some(ips) = &self.settings.gateway_ips
                && !client.parse::<IpAddr>().is_ok_and(|ip| ips.contains(&ip.to_canonical()))
            {
                tracing::warn!("NICE DCV: a token's first connection from {client:?}, not from this machine's gateway");
                return false;
            }
            t.client = Some(client);
            t.expires = now + self.settings.session_ttl;
            let target = t.target.clone();
            self.store(&tokens);
            target
        };
        if target == DCV_TARGET_IDE {
            let hook = self.on_open.lock().unwrap().clone();
            if let Some(hook) = hook {
                hook();
            }
        }
        true
    }

    /// The DCV server of `target`, if known: a static one, else the one the resolver last named.
    fn known_host(&self, target: &str) -> Option<String> {
        let upstream = self.upstreams.iter().find(|u| u.target.name == target)?;
        match &upstream.target.host {
            HostSource::Static(host) => Some(host.clone()),
            HostSource::IdeLookup => self.resolved.lock().unwrap().get(target).cloned(),
        }
    }

    /// The verifier's answer, asked by the DCV server `server`: this backend's own token
    /// ([`Self::verify`]), else (another workspace's, `<alias>.<hex>`) that workspace's verifier's
    /// if `alias` is a peer (then with `Some(alias)`), else no.
    pub async fn check(&self, session: &str, token: &str, client: Option<&str>, server: &str) -> (bool, Option<String>) {
        let Some(alias) = self.foreign(token) else { return (self.verify(session, token, client, Some(server)), None) };
        let Some((_, url)) = self.settings.peers.iter().find(|(a, _)| a == alias) else {
            tracing::warn!("NICE DCV: a token of workspace {alias:?}, which is not a verifier peer (FACTORY_OS_DCV_VERIFIER_PEERS)");
            return (false, None);
        };
        let mut form = vec![("sessionId", session), ("authenticationToken", token), ("dcvServer", server)];
        if let Some(client) = client {
            form.push(("clientAddress", client));
        }
        let answer = match self.chain.post(url).form(&form).send().await {
            Ok(response) if response.status().is_success() => response.text().await.unwrap_or_default(),
            Ok(response) => {
                tracing::warn!("NICE DCV: verifier peer {alias} ({url}): {}", response.status());
                String::new()
            }
            Err(err) => {
                tracing::warn!("NICE DCV: verifier peer {alias} ({url}): {err}");
                String::new()
            }
        };
        (said_yes(&answer), Some(alias.to_owned()))
    }

    /// Forget every token (running connections stay): how many there were.
    pub fn revoke(&self) -> usize {
        let mut tokens = self.tokens.lock().unwrap();
        let n = tokens.len();
        tokens.clear();
        self.store(&tokens);
        n
    }

    /// Whether the edge's user (`X-User-Alias`, set by the workspace edge) may use DCV here.
    fn may_use(&self, alias: &str) -> bool {
        self.settings.users.iter().any(|u| u == alias)
            || match &self.settings.owner {
                Some(owner) => owner == alias,
                None => self.settings.users.is_empty(),
            }
    }

    /// Whether `request` may reach the session or the proxy: the workspace's user, from the page's
    /// own origin (or a top-level navigation to `/dcv/`).
    pub fn admit(&self, request: &Request) -> Result<(), String> {
        let headers = request.headers();
        if let Some(alias) = headers.get("x-user-alias").and_then(|v| v.to_str().ok()).map(str::trim)
            && !self.may_use(alias)
        {
            return Err(format!("NICE DCV on this workspace is not open to {alias}"));
        }
        let navigation = matches!(*request.method(), axum::http::Method::GET | axum::http::Method::HEAD)
            && request.uri().path().starts_with(api::DCV)
            && headers.get("sec-fetch-mode").is_some_and(|v| v.as_bytes().eq_ignore_ascii_case(b"navigate"));
        if navigation || same_origin(headers) {
            Ok(())
        } else {
            Err("NICE DCV: cross-origin request refused".to_owned())
        }
    }

    /// The answer of the resolver at `resolver` for `session` (`None`: no target of that resolver
    /// has it, or no DCV server).
    pub async fn resolve(&self, resolver: SocketAddr, session: &str) -> Option<Value> {
        let target = &self.upstreams.iter().find(|u| u.target.resolver == resolver && u.target.session == session)?.target;
        let host = match &target.host {
            HostSource::Static(host) => host.clone(),
            HostSource::IdeLookup => (self.ide)().await.filter(|h| valid_host(h)).or_else(|| self.settings.server.clone())?,
        };
        self.resolved.lock().unwrap().insert(target.name.clone(), host.clone());
        Some(json!({
            "SessionId": target.session,
            "TransportProtocol": "HTTP",
            "DcvServerEndpoint": host,
            "Port": target.port,
            "WebUrlPath": "/",
        }))
    }

    /// Where a request under `/dcv/` goes (`None`: a bad path).
    pub fn route(&self, uri: &Uri) -> Option<Route> {
        upstream_path(uri, |name| self.upstreams.iter().any(|u| u.target.name == name))
    }

    /// Forward one request (an upgrade too) to its target's gateway.
    async fn forward(&self, mut request: Request) -> Result<Response, HttpError> {
        if let Some(why) = &self.unavailable {
            return Err((StatusCode::SERVICE_UNAVAILABLE, why.clone()));
        }
        let (name, path) = match self.route(request.uri()).ok_or((StatusCode::BAD_REQUEST, "bad DCV path".to_owned()))? {
            Route::Proxy(name, path) => (name, path),
            Route::Slash(name, query) => {
                let location = format!("{}{query}", api::dcv_target(&name));
                return Ok((StatusCode::PERMANENT_REDIRECT, [(header::LOCATION, location)]).into_response());
            }
        };
        let upstream = self.upstream(&name)?;
        let gateway = upstream.target.gateway.clone();
        let upgrade = is_upgrade(request.headers());
        let client_upgrade = upgrade.then(|| hyper::upgrade::on(&mut request));
        let (parts, body) = request.into_parts();
        let mut outgoing = hyper::Request::builder()
            .method(parts.method.clone())
            .uri(path)
            .version(Version::HTTP_11)
            .body(body)
            .map_err(|err| (StatusCode::BAD_REQUEST, format!("bad DCV request: {err}")))?;
        *outgoing.headers_mut() = upstream_headers(&parts.headers, &gateway, upgrade);

        let bad_gateway = |err: anyhow::Error| (StatusCode::BAD_GATEWAY, format!("NICE DCV gateway {} ({name}): {err:#}", gateway.origin()));
        let tcp = tokio::time::timeout(CONNECT_TIMEOUT, TcpStream::connect(gateway.addr))
            .await
            .map_err(|_| bad_gateway(anyhow::anyhow!("connect timed out")))?
            .map_err(|err| bad_gateway(err.into()))?;
        tcp.set_nodelay(true).ok();
        let mut response = match &upstream.tls {
            Some(tls) => {
                let server_name = rustls::pki_types::ServerName::IpAddress(gateway.addr.ip().into());
                let stream = tls.connect(server_name, tcp).await.context("TLS").map_err(bad_gateway)?;
                send(stream, outgoing).await.map_err(bad_gateway)?
            }
            None => send(tcp, outgoing).await.map_err(bad_gateway)?,
        };

        let switching = response.status() == StatusCode::SWITCHING_PROTOCOLS;
        if switching && let Some(client) = client_upgrade {
            let upstream = hyper::upgrade::on(&mut response);
            let open = self.activity.connection();
            tokio::spawn(async move {
                let _open = open;
                match (client.await, upstream.await) {
                    (Ok(client), Ok(upstream)) => {
                        let (mut client, mut upstream) = (TokioIo::new(client), TokioIo::new(upstream));
                        if let Err(err) = tokio::io::copy_bidirectional(&mut client, &mut upstream).await {
                            tracing::debug!("NICE DCV websocket closed: {err}");
                        }
                    }
                    (Err(err), _) | (_, Err(err)) => tracing::debug!("NICE DCV upgrade failed: {err}"),
                }
            });
        }
        let (mut parts, body) = response.into_parts();
        let mut headers = HeaderMap::new();
        for (name, value) in &parts.headers {
            if name == header::STRICT_TRANSPORT_SECURITY {
                // The gateway's, not this origin's policy.
                continue;
            }
            if name == header::CONTENT_SECURITY_POLICY {
                if let Some(value) = value.to_str().ok().and_then(without_upgrade_insecure).and_then(|v| HeaderValue::from_str(&v).ok()) {
                    headers.append(name.clone(), value);
                }
                continue;
            }
            if !is_hop_by_hop(name) || (switching && (name == header::CONNECTION || name == header::UPGRADE)) {
                headers.append(name.clone(), value.clone());
            }
        }
        if let Some(location) =
            headers.get(header::LOCATION).and_then(|v| v.to_str().ok()).and_then(|l| rewrite_location(l, &gateway, &name))
            && let Ok(value) = HeaderValue::from_str(&location)
        {
            headers.insert(header::LOCATION, value);
        }
        parts.headers = headers;
        Ok(Response::from_parts(parts, Body::new(body)))
    }
}

/// A verifier's `<auth result="yes">`.
fn said_yes(answer: &str) -> bool {
    let answer = answer.trim_start();
    answer.starts_with("<auth result=\"yes\"") || answer.starts_with("<auth result='yes'")
}

pub(crate) async fn send<S>(stream: S, request: hyper::Request<Body>) -> anyhow::Result<hyper::Response<hyper::body::Incoming>>
where
    S: AsyncRead + AsyncWrite + Send + Unpin + 'static,
{
    let (mut sender, connection) = hyper::client::conn::http1::handshake::<_, Body>(TokioIo::new(stream)).await?;
    tokio::spawn(async move {
        if let Err(err) = connection.with_upgrades().await {
            tracing::debug!("NICE DCV gateway connection: {err}");
        }
    });
    Ok(sender.send_request(request).await?)
}

/// `/dcv/<target>/x?q` → target's `/x?q`; `/dcv/<target>` → its `/dcv/<target>/`; `/dcv/x?q` (no
/// target first) → `ide`'s `/x?q` (`None`: not under `/dcv/`, or a segment that is, or hides, a `.`
/// / `..` or a separator once percent-decoded).
fn upstream_path(uri: &Uri, is_target: impl Fn(&str) -> bool) -> Option<Route> {
    let rest = uri.path().strip_prefix(api::DCV)?;
    if !rest.starts_with('/') || !rest.split('/').all(safe_segment) {
        return None;
    }
    let query = uri.query().map(|q| format!("?{q}")).unwrap_or_default();
    let (first, tail) = rest[1..].split_once('/').map_or((&rest[1..], None), |(first, tail)| (first, Some(tail)));
    Some(match tail {
        _ if !is_target(first) => Route::Proxy(DCV_TARGET_IDE.to_owned(), format!("{rest}{query}")),
        None => Route::Slash(first.to_owned(), query),
        Some(tail) => Route::Proxy(first.to_owned(), format!("/{tail}{query}")),
    })
}

/// A path segment that stays one plain segment however often it is decoded: no `.` / `..`, no `/`,
/// a backslash, NUL or `%` once percent-decoded, and valid percent-encoding.
pub(crate) fn safe_segment(segment: &str) -> bool {
    let hex = |b: u8| char::from(b).to_digit(16);
    let bytes = segment.as_bytes();
    let mut decoded = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'%' {
            match (bytes.get(i + 1).copied().and_then(hex), bytes.get(i + 2).copied().and_then(hex)) {
                (Some(hi), Some(lo)) => decoded.push((hi * 16 + lo) as u8),
                _ => return false,
            }
            i += 3;
        } else {
            decoded.push(bytes[i]);
            i += 1;
        }
    }
    decoded != b"." && decoded != b".." && !decoded.iter().any(|b| matches!(b, b'/' | b'\\' | 0 | b'%'))
}

/// The IP of a client address DCV reports (`192.0.2.4:5`, `[::1]:5`, `192.0.2.4`), else as it is.
fn client_ip(address: &str) -> String {
    let address = address.trim();
    address
        .parse::<SocketAddr>()
        .map(|a| a.ip())
        .or_else(|_| address.parse::<IpAddr>())
        .map_or_else(|_| address.to_owned(), |ip| ip.to_canonical().to_string())
}

/// `http(s)://host[:port]` → `host[:port]` without the scheme's default port.
fn authority(value: &str, https: bool) -> String {
    let value = value.trim().to_ascii_lowercase();
    let default = if https { ":443" } else { ":80" };
    value.strip_suffix(default).map_or(value.clone(), str::to_owned)
}

/// The request comes from the page's own origin: `Sec-Fetch-Site` says so (`none`: typed in, a
/// bookmark), or without it the `Origin` is the `Host` (or the edge's `X-Forwarded-Host`). Without
/// either it is no browser's cross-site request (a native client, curl).
pub(crate) fn same_origin(headers: &HeaderMap) -> bool {
    if let Some(site) = headers.get("sec-fetch-site") {
        return matches!(site.to_str().map(str::trim), Ok("same-origin" | "none"));
    }
    let Some(origin) = headers.get(header::ORIGIN) else { return true };
    let Some((https, rest)) = origin.to_str().ok().and_then(|o| {
        o.strip_prefix("https://").map(|r| (true, r)).or_else(|| o.strip_prefix("http://").map(|r| (false, r)))
    }) else {
        return false;
    };
    let origin = authority(rest, https);
    [header::HOST, HeaderName::from_static("x-forwarded-host")]
        .iter()
        .flat_map(|name| headers.get_all(name))
        .filter_map(|v| v.to_str().ok())
        .flat_map(|v| v.split(','))
        .any(|host| authority(host, https) == origin)
}

pub(crate) fn is_upgrade(headers: &HeaderMap) -> bool {
    headers.contains_key(header::UPGRADE)
        && headers
            .get_all(header::CONNECTION)
            .iter()
            .filter_map(|v| v.to_str().ok())
            .any(|v| v.split(',').any(|t| t.trim().eq_ignore_ascii_case("upgrade")))
}

pub(crate) fn is_hop_by_hop(name: &HeaderName) -> bool {
    matches!(
        name.as_str(),
        "connection" | "keep-alive" | "proxy-authenticate" | "proxy-authorization" | "proxy-connection" | "te" | "trailer"
            | "transfer-encoding" | "upgrade"
    )
}

/// The edge's own headers (CloudFront / the workspace ALB), not for the gateway.
pub(crate) fn is_edge_header(name: &HeaderName) -> bool {
    let name = name.as_str();
    name.starts_with("x-amzn-") || name.starts_with("x-origin-verify") || name == "x-forwarded-host"
}

/// The workspace edge's cookies (its session, the workspace selection, the login's state).
fn is_edge_cookie(name: &str) -> bool {
    name == "ws_session" || name == workspace::SELECT_COOKIE || name.starts_with("oidc_") || name.starts_with("AWSELB")
}

/// A `Cookie` header without the edge's cookies (`None`: nothing left).
pub(crate) fn strip_edge_cookies(value: &str) -> Option<String> {
    let kept: Vec<&str> = value
        .split(';')
        .map(str::trim)
        .filter(|c| !c.is_empty())
        .filter(|c| !is_edge_cookie(c.split_once('=').map_or(*c, |(name, _)| name).trim()))
        .collect();
    (!kept.is_empty()).then(|| kept.join("; "))
}

/// The client's headers for the gateway: no hop-by-hop (but an upgrade's), no edge cookies or
/// headers, and the gateway's own `Host` and `Origin`.
pub(crate) fn upstream_headers(from: &HeaderMap, gateway: &Gateway, upgrade: bool) -> HeaderMap {
    let mut headers = HeaderMap::new();
    for (name, value) in from {
        if is_hop_by_hop(name) || is_edge_header(name) || name == header::HOST || name == header::ORIGIN || name == header::COOKIE {
            continue;
        }
        headers.append(name.clone(), value.clone());
    }
    let cookies: Vec<String> = from.get_all(header::COOKIE).iter().filter_map(|v| v.to_str().ok()).filter_map(strip_edge_cookies).collect();
    if !cookies.is_empty()
        && let Ok(value) = HeaderValue::from_str(&cookies.join("; "))
    {
        headers.insert(header::COOKIE, value);
    }
    if let Ok(host) = HeaderValue::from_str(&gateway.addr.to_string()) {
        headers.insert(header::HOST, host);
    }
    if from.contains_key(header::ORIGIN)
        && let Ok(origin) = HeaderValue::from_str(&gateway.origin())
    {
        headers.insert(header::ORIGIN, origin);
    }
    if upgrade {
        headers.insert(header::CONNECTION, HeaderValue::from_static("upgrade"));
        if let Some(value) = from.get(header::UPGRADE) {
            headers.insert(header::UPGRADE, value.clone());
        }
    }
    headers
}

/// The gateway's `Content-Security-Policy` without `upgrade-insecure-requests` (`None`: nothing
/// left): on a plain-HTTP backend (a test server) the browser would fetch the web client's scripts
/// over HTTPS from a port that isn't there; behind the edge the page is HTTPS anyway.
fn without_upgrade_insecure(csp: &str) -> Option<String> {
    let kept: Vec<&str> =
        csp.split(';').map(str::trim).filter(|d| !d.is_empty() && !d.eq_ignore_ascii_case("upgrade-insecure-requests")).collect();
    (!kept.is_empty()).then(|| kept.join("; "))
}

/// A redirect of `target`'s gateway, back under `/dcv/<target>` (the workspace prefix is added after).
fn rewrite_location(location: &str, gateway: &Gateway, target: &str) -> Option<String> {
    let path = location.strip_prefix(&gateway.origin()).unwrap_or(location);
    if path.is_empty() {
        Some(api::dcv_target(target))
    } else if path.starts_with('/') && !path.starts_with("//") {
        Some(format!("{}/{target}{path}", api::DCV))
    } else {
        None
    }
}

/// The TLS client for the gateway: its certificate pinned (`pin`), else any (it is on loopback).
pub(crate) fn tls_connector(pin: Option<&PathBuf>) -> anyhow::Result<tokio_rustls::TlsConnector> {
    use rustls::pki_types::CertificateDer;
    use rustls::pki_types::pem::PemObject as _;
    let pinned = match pin {
        Some(path) => Some(
            CertificateDer::from_pem_file(path)
                .with_context(|| format!("reading the DCV gateway certificate {} (FACTORY_OS_DCV_GATEWAY_CERT)", path.display()))?
                .into_owned(),
        ),
        None => None,
    };
    let provider = Arc::new(rustls::crypto::ring::default_provider());
    let config = rustls::ClientConfig::builder_with_provider(provider.clone())
        .with_safe_default_protocol_versions()?
        .dangerous()
        .with_custom_certificate_verifier(Arc::new(Pinned { pinned, provider }))
        .with_no_client_auth();
    Ok(tokio_rustls::TlsConnector::from(Arc::new(config)))
}

/// Accepts exactly the pinned certificate (or, without one, any), with valid handshake signatures.
#[derive(Debug)]
struct Pinned {
    pinned: Option<rustls::pki_types::CertificateDer<'static>>,
    provider: Arc<rustls::crypto::CryptoProvider>,
}

impl rustls::client::danger::ServerCertVerifier for Pinned {
    fn verify_server_cert(
        &self,
        end_entity: &rustls::pki_types::CertificateDer<'_>,
        _intermediates: &[rustls::pki_types::CertificateDer<'_>],
        _server_name: &rustls::pki_types::ServerName<'_>,
        _ocsp: &[u8],
        _now: rustls::pki_types::UnixTime,
    ) -> Result<rustls::client::danger::ServerCertVerified, rustls::Error> {
        match &self.pinned {
            Some(pinned) if pinned.as_ref() != end_entity.as_ref() => {
                Err(rustls::Error::General("not the pinned DCV gateway certificate".to_owned()))
            }
            _ => Ok(rustls::client::danger::ServerCertVerified::assertion()),
        }
    }

    fn verify_tls12_signature(
        &self,
        message: &[u8],
        cert: &rustls::pki_types::CertificateDer<'_>,
        dss: &rustls::DigitallySignedStruct,
    ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
        rustls::crypto::verify_tls12_signature(message, cert, dss, &self.provider.signature_verification_algorithms)
    }

    fn verify_tls13_signature(
        &self,
        message: &[u8],
        cert: &rustls::pki_types::CertificateDer<'_>,
        dss: &rustls::DigitallySignedStruct,
    ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
        rustls::crypto::verify_tls13_signature(message, cert, dss, &self.provider.signature_verification_algorithms)
    }

    fn supported_verify_schemes(&self) -> Vec<rustls::SignatureScheme> {
        self.provider.signature_verification_algorithms.supported_schemes()
    }
}

/// The public routes: the proxy and the session (merged into the backend's router, outside its
/// CORS layer), each behind [`DcvService::admit`].
pub fn router(dcv: Arc<DcvService>) -> Router {
    Router::new()
        .route(api::INFRA_DCV_SESSION, post(session).delete(revoke))
        .route(api::DCV, any(to_slash))
        .route(&format!("{}/", api::DCV), any(proxy))
        .route(&format!("{}/{{*rest}}", api::DCV), any(proxy))
        .route_layer(axum::middleware::from_fn_with_state(dcv.clone(), guard))
        .with_state(dcv)
}

async fn guard(State(dcv): State<Arc<DcvService>>, request: Request, next: axum::middleware::Next) -> Response {
    match dcv.admit(&request) {
        Ok(()) => next.run(request).await,
        Err(why) => {
            tracing::warn!("{why}: {} {}", request.method(), request.uri().path());
            (StatusCode::FORBIDDEN, why).into_response()
        }
    }
}

/// A JSON `POST` only: never a CORS "simple" request, so another site's page can't send it without
/// a preflight (which these routes don't answer). Body: [`DcvSessionRequest`] (empty, `{}`: `ide`).
async fn session(State(dcv): State<Arc<DcvService>>, headers: HeaderMap, body: Bytes) -> Result<Json<DcvSession>, HttpError> {
    let json = headers
        .get(header::CONTENT_TYPE)
        .and_then(|v| v.to_str().ok())
        .is_some_and(|v| v.split(';').next().is_some_and(|t| t.trim().eq_ignore_ascii_case("application/json")));
    if !json {
        return Err((StatusCode::UNSUPPORTED_MEDIA_TYPE, "POST the DCV session with Content-Type: application/json".to_owned()));
    }
    let request: DcvSessionRequest = if body.iter().all(u8::is_ascii_whitespace) {
        DcvSessionRequest::default()
    } else {
        serde_json::from_slice(&body).map_err(|err| (StatusCode::BAD_REQUEST, format!("invalid DCV session request: {err}")))?
    };
    dcv.mint(request.target.as_deref().unwrap_or(DCV_TARGET_IDE)).map(Json)
}

async fn revoke(State(dcv): State<Arc<DcvService>>) -> Json<DcvRevoked> {
    let revoked = dcv.revoke();
    tracing::info!("NICE DCV: {revoked} session token(s) revoked");
    Json(DcvRevoked { revoked })
}

/// `/dcv` → `/dcv/`, so the web client's relative URLs resolve.
async fn to_slash(uri: Uri) -> Response {
    let query = uri.query().map(|q| format!("?{q}")).unwrap_or_default();
    (StatusCode::PERMANENT_REDIRECT, [(header::LOCATION, format!("{}/{query}", api::DCV))]).into_response()
}

async fn proxy(State(dcv): State<Arc<DcvService>>, request: Request) -> Response {
    match dcv.forward(request).await {
        Ok(response) => response,
        Err(err) => err.into_response(),
    }
}

/// A gateway's session resolver (loopback), with the address it listens on.
fn resolver_router() -> Router<(Arc<DcvService>, SocketAddr)> {
    Router::new().route("/resolveSession", post(resolve))
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct ResolveQuery {
    #[serde(default)]
    session_id: String,
    #[serde(default)]
    transport: Option<String>,
    #[serde(default)]
    client_ip_address: Option<String>,
}

async fn resolve(State((dcv, bind)): State<(Arc<DcvService>, SocketAddr)>, Query(query): Query<ResolveQuery>) -> Response {
    let answer = dcv.resolve(bind, &query.session_id).await;
    tracing::info!(
        "NICE DCV resolve {:?} on {bind} ({}, client {}): {}",
        query.session_id,
        query.transport.as_deref().unwrap_or("?"),
        query.client_ip_address.as_deref().unwrap_or("?"),
        answer.as_ref().map_or("unknown".to_owned(), |a| format!("{}:{}", a["DcvServerEndpoint"].as_str().unwrap_or("?"), a["Port"]))
    );
    match answer {
        Some(answer) => Json(answer).into_response(),
        None => (StatusCode::NOT_FOUND, "unknown DCV session").into_response(),
    }
}

/// The DCV server's `auth-token-verifier`.
fn verifier_router() -> Router<Arc<DcvService>> {
    Router::new().route("/", post(verify)).route("/verify", post(verify))
}

#[derive(Deserialize, Default)]
#[serde(default, rename_all = "camelCase")]
struct VerifyForm {
    session_id: String,
    authentication_token: String,
    client_address: Option<String>,
    /// A peer verifier chaining: the DCV server that asked it.
    dcv_server: Option<String>,
}

/// Only the cell's private network (and loopback) asks.
fn private(ip: IpAddr) -> bool {
    match ip {
        IpAddr::V4(ip) => ip.is_private() || ip.is_loopback(),
        IpAddr::V6(ip) => ip.is_loopback() || ip.to_ipv4_mapped().is_some_and(|ip| ip.is_private() || ip.is_loopback()),
    }
}

async fn verify(State(dcv): State<Arc<DcvService>>, ConnectInfo(peer): ConnectInfo<SocketAddr>, Form(form): Form<VerifyForm>) -> Response {
    if !private(peer.ip()) {
        return (StatusCode::FORBIDDEN, "not from the cell's network").into_response();
    }
    // Asked by a peer verifier on behalf of the DCV server, or by the DCV server itself.
    let server = form.dcv_server.as_deref().and_then(|s| s.trim().parse::<IpAddr>().ok()).unwrap_or(peer.ip()).to_canonical().to_string();
    // A peer's chained question is about this backend's own tokens only: never chained on (no
    // loops between peers that list each other, no relaying through this verifier).
    let (ok, chained) = if form.dcv_server.is_some() {
        (dcv.verify(&form.session_id, &form.authentication_token, form.client_address.as_deref(), Some(&server)), None)
    } else {
        dcv.check(&form.session_id, &form.authentication_token, form.client_address.as_deref(), &server).await
    };
    tracing::info!(
        "NICE DCV token for session {:?} from {}{} (client {}): {}{}",
        form.session_id,
        peer.ip(),
        if form.dcv_server.is_some() { format!(" for DCV server {server}") } else { String::new() },
        form.client_address.as_deref().unwrap_or("?"),
        chained.map(|alias| format!("chained to {alias}: ")).unwrap_or_default(),
        if ok { "accepted" } else { "rejected" }
    );
    let body = if ok {
        format!(r#"<auth result="yes"><username>{}</username></auth>"#, dcv.settings.user)
    } else {
        r#"<auth result="no"><message>invalid or expired token</message></auth>"#.to_owned()
    };
    ([(header::CONTENT_TYPE, "text/xml")], body).into_response()
}

#[cfg(test)]
mod tests {
    use super::*;
    use futures_util::{SinkExt as _, StreamExt as _};
    use tokio_tungstenite::tungstenite::Message;
    use tower::ServiceExt as _;

    fn service(settings: Settings, ide: Option<&'static str>) -> Arc<DcvService> {
        let lookup: IdeLookup = Arc::new(move || Box::pin(async move { ide.map(str::to_owned) }));
        DcvService::new(settings, lookup, Activity::new())
    }

    fn enabled(gateway: &str) -> Settings {
        Settings { gateway: Some(Gateway::parse(gateway).unwrap()), ..Settings::default() }
    }

    fn token_of(url: &str) -> &str {
        url.split_once("authToken=").unwrap().1.split('#').next().unwrap()
    }

    #[test]
    fn settings() {
        let vars = |pairs: &'static [(&'static str, &'static str)]| {
            move |name: &str| pairs.iter().find(|(k, _)| *k == name).map(|(_, v)| (*v).to_owned())
        };
        let s = Settings::from_vars(vars(&[]));
        assert!(s.gateway.is_none() && s.error.is_none());
        // Both listeners on loopback unless the deployment opens the verifier (dcv-gateway.sh: 0.0.0.0:8445).
        assert_eq!((s.verifier_bind, s.resolver_bind), ("127.0.0.1:8445".parse().unwrap(), "127.0.0.1:8446".parse().unwrap()));
        let s = Settings::from_vars(vars(&[
            ("FACTORY_OS_DCV_GATEWAY", "https://127.0.0.1:8443/"),
            ("FACTORY_OS_DCV_SERVER", "10.0.2.20"),
            ("FACTORY_OS_DCV_VERIFIER_BIND", "0.0.0.0:9445"),
        ]));
        assert_eq!(s.gateway, Some(Gateway { tls: true, addr: "127.0.0.1:8443".parse().unwrap() }));
        assert_eq!((s.server.as_deref(), s.verifier_bind.port(), s.resolver_bind.port()), (Some("10.0.2.20"), 9445, 8446));
        assert_eq!(Gateway::parse("http://localhost:9").unwrap().addr, "127.0.0.1:9".parse().unwrap());
        // Off the loopback, or a resolver anyone could ask: off, with the reason.
        assert!(Gateway::parse("https://10.0.0.1:8443").is_err());
        let s = Settings::from_vars(vars(&[("FACTORY_OS_DCV_GATEWAY", "https://127.0.0.1:8443"), ("FACTORY_OS_DCV_RESOLVER_BIND", "0.0.0.0:8446")]));
        assert!(s.gateway.is_none() && s.error.unwrap().contains("loopback"));
    }

    #[tokio::test]
    async fn tokens_survive_a_restart() {
        let dir = std::env::temp_dir().join(format!("dcv-tokens-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let store = dir.join("dcv-tokens.json");
        let settings = || Settings { token_store: Some(store.clone()), ..enabled("http://127.0.0.1:1") };
        let first = service(settings(), None);
        let url = first.mint("ide").unwrap().url;
        let token = token_of(&url).to_owned();
        assert!(first.verify("console", &token, Some("10.0.0.5:1234"), None));
        drop(first);
        let second = service(settings(), None); // the backend restarted (a deploy)
        assert!(second.verify("console", &token, Some("10.0.0.5:999"), None));
        assert!(!second.verify("console", &token, Some("10.0.0.6:1"), None)); // still bound to its client
        assert_eq!(second.revoke(), 1);
        assert!(load_tokens(&store).is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[tokio::test]
    async fn tokens_mint_and_verify() {
        let off = service(Settings::default(), None);
        assert_eq!(off.mint("ide").unwrap_err().0, StatusCode::SERVICE_UNAVAILABLE);
        assert!(!off.enabled());

        let dcv = service(enabled("http://127.0.0.1:1"), None);
        let opened = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        dcv.on_open({
            let opened = opened.clone();
            Arc::new(move || {
                opened.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            })
        });
        let a = dcv.mint("ide").unwrap();
        let b = dcv.mint("ide").unwrap();
        assert!(a.url.starts_with("/dcv/ide/?authToken=") && a.url.ends_with("#console"), "{}", a.url);
        assert_eq!((a.workspace.as_deref(), a.target.as_str()), (None, "ide"));
        let (ta, tb) = (token_of(&a.url), token_of(&b.url));
        assert_eq!(ta.len(), 64);
        assert_ne!(ta, tb);
        assert!(!dcv.verify("other", ta, Some("10.0.0.5:1"), None));
        assert!(!dcv.verify("console", "nope", Some("10.0.0.5:1"), None));
        // The first connection binds the token to its client's address (any port).
        assert!(dcv.verify("console", ta, Some("10.0.0.5:1"), None));
        assert!(dcv.verify("console", ta, Some("10.0.0.5:2"), None) && dcv.verify("console", ta, Some("10.0.0.5"), None));
        assert!(!dcv.verify("console", ta, Some("10.0.0.6:1"), None), "another client with a used token");
        assert!(dcv.verify("console", tb, Some("[::ffff:10.0.0.6]:9"), None) && dcv.verify("console", tb, Some("10.0.0.6:1"), None));
        // Open PLC IDE's hook: once per token, on its first accepted connection only.
        assert_eq!(opened.load(std::sync::atomic::Ordering::SeqCst), 2);
        let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs_f64();
        assert!(a.expires_s > now && a.expires_s <= now + START_TTL.as_secs_f64() + 1.0);
        assert_eq!(dcv.revoke(), 2);
        assert!(!dcv.verify("console", ta, Some("10.0.0.5:1"), None));

        // Unused, a token only starts a connection for a short while; used, it reconnects for longer.
        let short = service(
            Settings { start_ttl: Duration::from_millis(1), session_ttl: Duration::from_secs(60), ..enabled("http://127.0.0.1:1") },
            None,
        );
        let (unused, used) = (short.mint("ide").unwrap(), short.mint("ide").unwrap());
        assert!(short.verify("console", token_of(&used.url), None, None));
        tokio::time::sleep(Duration::from_millis(5)).await;
        assert!(!short.verify("console", token_of(&unused.url), None, None));
        assert!(short.verify("console", token_of(&used.url), None, None));
    }

    #[test]
    fn only_the_page_origin_and_the_workspace_owner_get_in() {
        let request = |method: &str, path: &str, headers: &[(&str, &str)]| {
            let mut builder = Request::builder().method(method).uri(path);
            for (k, v) in headers {
                builder = builder.header(*k, *v);
            }
            builder.body(Body::empty()).unwrap()
        };
        let dcv = service(Settings { owner: Some("jdoe".into()), ..enabled("http://127.0.0.1:1") }, None);
        let host = ("host", "198.51.100.7");
        let ok = |r: Request| dcv.admit(&r).is_ok();
        // A native client or curl: no browser headers.
        assert!(ok(request("POST", api::INFRA_DCV_SESSION, &[host])));
        assert!(ok(request("POST", api::INFRA_DCV_SESSION, &[host, ("sec-fetch-site", "same-origin")])));
        assert!(ok(request("POST", api::INFRA_DCV_SESSION, &[host, ("origin", "http://198.51.100.7")])));
        assert!(ok(request("POST", api::INFRA_DCV_SESSION, &[host, ("origin", "http://198.51.100.7:80")])));
        // Another site's page.
        assert!(!ok(request("POST", api::INFRA_DCV_SESSION, &[host, ("sec-fetch-site", "cross-site")])));
        assert!(!ok(request("POST", api::INFRA_DCV_SESSION, &[host, ("sec-fetch-site", "same-site")])));
        assert!(!ok(request("POST", api::INFRA_DCV_SESSION, &[host, ("origin", "http://evil.example")])));
        assert!(!ok(request("POST", api::INFRA_DCV_SESSION, &[host, ("origin", "null")])));
        assert!(!ok(request("GET", "/dcv/auth", &[host, ("origin", "http://evil.example"), ("upgrade", "websocket")])));
        assert!(!ok(request("GET", "/dcv/js/main.js", &[host, ("sec-fetch-site", "cross-site"), ("sec-fetch-mode", "no-cors")])));
        // Opening the page from a link elsewhere is fine (it needs a token to do anything); minting isn't.
        assert!(ok(request("GET", "/dcv/", &[host, ("sec-fetch-site", "cross-site"), ("sec-fetch-mode", "navigate")])));
        assert!(!ok(request("POST", api::INFRA_DCV_SESSION, &[host, ("sec-fetch-site", "cross-site"), ("sec-fetch-mode", "navigate")])));
        // Behind the edge: CloudFront's host in X-Forwarded-Host, the ALB's in Host; the owner only.
        let edge = [("host", "alb.example"), ("x-forwarded-host", "d1.cloudfront.net"), ("origin", "https://d1.cloudfront.net")];
        assert!(ok(request("POST", api::INFRA_DCV_SESSION, &[edge[0], edge[1], edge[2], ("x-user-alias", "jdoe")])));
        assert!(!ok(request("POST", api::INFRA_DCV_SESSION, &[edge[0], edge[1], edge[2], ("x-user-alias", "alice")])));
        assert!(!ok(request("GET", "/dcv/", &[host, ("sec-fetch-mode", "navigate"), ("x-user-alias", "alice")])));
        let shared = service(Settings { owner: Some("ci".into()), users: vec!["alice".into()], ..enabled("http://127.0.0.1:1") }, None);
        assert!(shared.admit(&request("GET", "/dcv/", &[host, ("x-user-alias", "alice")])).is_ok());
        assert!(shared.admit(&request("GET", "/dcv/", &[host, ("x-user-alias", "bob")])).is_err());
    }

    #[tokio::test]
    async fn verifier_answers_dcv() {
        let dcv = service(enabled("http://127.0.0.1:1"), None);
        let token = token_of(&dcv.mint("ide").unwrap().url).to_owned();
        let ask = |peer: &str, body: String| {
            let router = verifier_router().with_state(dcv.clone());
            let mut request = Request::post("/")
                .header(header::CONTENT_TYPE, "application/x-www-form-urlencoded")
                .body(Body::from(body))
                .unwrap();
            request.extensions_mut().insert(ConnectInfo(peer.parse::<SocketAddr>().unwrap()));
            async move {
                let response = router.oneshot(request).await.unwrap();
                let status = response.status();
                let body = axum::body::to_bytes(response.into_body(), 1 << 16).await.unwrap();
                (status, String::from_utf8(body.to_vec()).unwrap())
            }
        };
        let (status, body) = ask("10.0.2.20:50000", format!("sessionId=console&authenticationToken={token}&clientAddress=203.0.113.4%3A5")).await;
        assert_eq!((status, body.as_str()), (StatusCode::OK, r#"<auth result="yes"><username>Administrator</username></auth>"#));
        // Bound to that client now.
        let (_, body) = ask("10.0.2.20:50000", format!("sessionId=console&authenticationToken={token}&clientAddress=203.0.113.8%3A5")).await;
        assert!(body.starts_with(r#"<auth result="no">"#), "{body}");
        let (_, body) = ask("10.0.2.20:50000", "sessionId=console&authenticationToken=bad".to_owned()).await;
        assert!(body.starts_with(r#"<auth result="no">"#), "{body}");
        let (_, body) = ask("10.0.2.20:50000", format!("sessionId=other&authenticationToken={token}")).await;
        assert!(body.starts_with(r#"<auth result="no">"#), "{body}");
        let (status, _) = ask("198.51.100.7:50000", format!("sessionId=console&authenticationToken={token}")).await;
        assert_eq!(status, StatusCode::FORBIDDEN);
    }

    #[tokio::test]
    async fn resolver_finds_the_ide() {
        let ask = |dcv: Arc<DcvService>, query: &'static str| async move {
            let response = resolver_router().with_state((dcv, SocketAddr::from((Ipv4Addr::LOCALHOST, 8446)))).oneshot(Request::post(query).body(Body::empty()).unwrap()).await.unwrap();
            let status = response.status();
            let body = axum::body::to_bytes(response.into_body(), 1 << 16).await.unwrap();
            (status, serde_json::from_slice::<Value>(&body).unwrap_or(Value::Null))
        };
        let from_lambda = service(Settings { server: Some("10.9.9.9".into()), ..enabled("http://127.0.0.1:1") }, Some("10.0.2.20"));
        let (status, body) = ask(from_lambda.clone(), "/resolveSession?sessionId=console&transport=HTTP&clientIpAddress=127.0.0.1").await;
        assert_eq!(status, StatusCode::OK);
        assert_eq!(
            body,
            json!({"SessionId": "console", "TransportProtocol": "HTTP", "DcvServerEndpoint": "10.0.2.20", "Port": 8443, "WebUrlPath": "/"})
        );
        assert_eq!(ask(from_lambda, "/resolveSession?sessionId=other&transport=HTTP").await.0, StatusCode::NOT_FOUND);
        let fallback = service(Settings { server: Some("10.9.9.9".into()), ..enabled("http://127.0.0.1:1") }, None);
        assert_eq!(ask(fallback, "/resolveSession?sessionId=console").await.1["DcvServerEndpoint"], "10.9.9.9");
        let none = service(enabled("http://127.0.0.1:1"), None);
        assert_eq!(ask(none, "/resolveSession?sessionId=console").await.0, StatusCode::NOT_FOUND);
    }

    #[test]
    fn proxy_paths_and_headers() {
        let uri = |s: &str| s.parse::<Uri>().unwrap();
        let targets = |name: &str| ["ide", "robot1"].contains(&name);
        let path = |s: &str| match upstream_path(&uri(s), targets) {
            Some(Route::Proxy(target, path)) => Some(format!("{target} {path}")),
            Some(Route::Slash(target, query)) => Some(format!("{target} -> /{query}")),
            None => None,
        };
        // Legacy: no target first is `ide`'s.
        assert_eq!(path("/dcv/").as_deref(), Some("ide /"));
        assert_eq!(path("/dcv/js/main.js?v=1").as_deref(), Some("ide /js/main.js?v=1"));
        assert_eq!(path("/dcv/?authToken=t").as_deref(), Some("ide /?authToken=t"));
        assert_eq!(path("/dcv/x").as_deref(), Some("ide /x"));
        // A target's.
        assert_eq!(path("/dcv/ide/").as_deref(), Some("ide /"));
        assert_eq!(path("/dcv/ide/x").as_deref(), Some("ide /x"));
        assert_eq!(path("/dcv/ide/?authToken=t").as_deref(), Some("ide /?authToken=t"));
        assert_eq!(path("/dcv/robot1/auth?x=1").as_deref(), Some("robot1 /auth?x=1"));
        assert_eq!(path("/dcv/ide?authToken=t").as_deref(), Some("ide -> /?authToken=t"));
        assert_eq!(path("/dcv/robot2/x").as_deref(), Some("ide /robot2/x"), "not a target here: ide's");
        assert_eq!(path("/dcvx/"), None);
        assert_eq!(path("/dcv/../api/v1/config"), None);
        assert_eq!(path("/dcv/ide/../api/v1/config"), None);
        assert_eq!(path("/dcv/a/%2E%2E/b"), None);
        for bad in ["/dcv/.%2e/x", "/dcv/%2e./x", "/dcv/%2E./x", "/dcv/./x", "/dcv/..%2f/x", "/dcv/a%5cb", "/dcv/a%00", "/dcv/%252e%252e/x", "/dcv/%zz", "/dcv/ide/%2e%2e/x"] {
            assert_eq!(path(bad), None, "{bad}");
        }
        assert_eq!(path("/dcv/a%20b/c.d.js").as_deref(), Some("ide /a%20b/c.d.js"));

        assert_eq!(strip_edge_cookies("ws_session=a; fos_ws=b; oidc_state=c; dcv=keep").as_deref(), Some("dcv=keep"));
        assert_eq!(strip_edge_cookies("ws_session=a"), None);

        let gateway = Gateway::parse("https://127.0.0.1:8443").unwrap();
        let mut from = HeaderMap::new();
        from.insert(header::HOST, HeaderValue::from_static("d1.cloudfront.net"));
        from.insert(header::ORIGIN, HeaderValue::from_static("https://d1.cloudfront.net"));
        from.insert(header::COOKIE, HeaderValue::from_static("ws_session=s; x=1"));
        from.insert("x-origin-verify", HeaderValue::from_static("secret"));
        from.insert("x-amzn-oidc-data", HeaderValue::from_static("jwt"));
        from.insert(header::CONNECTION, HeaderValue::from_static("keep-alive, Upgrade"));
        from.insert(header::UPGRADE, HeaderValue::from_static("websocket"));
        from.insert("sec-websocket-protocol", HeaderValue::from_static("dcv"));
        assert!(is_upgrade(&from));
        let to = upstream_headers(&from, &gateway, true);
        assert_eq!(to[header::HOST], "127.0.0.1:8443");
        assert_eq!(to[header::ORIGIN], "https://127.0.0.1:8443");
        assert_eq!(to[header::COOKIE], "x=1");
        assert_eq!(to[header::CONNECTION], "upgrade");
        assert_eq!(to[header::UPGRADE], "websocket");
        assert_eq!(to["sec-websocket-protocol"], "dcv");
        assert!(!to.contains_key("x-origin-verify") && !to.contains_key("x-amzn-oidc-data"));

        assert_eq!(without_upgrade_insecure("upgrade-insecure-requests;"), None);
        assert_eq!(without_upgrade_insecure("default-src 'self'; upgrade-insecure-requests").as_deref(), Some("default-src 'self'"));

        assert_eq!(rewrite_location("/x/", &gateway, "ide").as_deref(), Some("/dcv/ide/x/"));
        assert_eq!(rewrite_location("https://127.0.0.1:8443/y", &gateway, "ide").as_deref(), Some("/dcv/ide/y"));
        assert_eq!(rewrite_location("https://127.0.0.1:8443", &gateway, "robot1").as_deref(), Some("/dcv/robot1/"));
        assert_eq!(rewrite_location("/z", &gateway, "robot1").as_deref(), Some("/dcv/robot1/z"));
        assert_eq!(rewrite_location("https://elsewhere/", &gateway, "ide"), None);
    }

    /// The whole path: `/workspace/<n>/dcv/…` through the workspace prefix and this proxy to a
    /// stand-in gateway (plain HTTP on loopback), a WebSocket upgrade included.
    #[tokio::test]
    async fn proxies_http_and_websockets_under_a_workspace() {
        use axum::extract::ws::{Message as WsMessage, WebSocketUpgrade};
        use axum::routing::get;

        let seen: Arc<Mutex<Vec<(String, HeaderMap)>>> = Arc::default();
        let gateway_app = {
            let seen = seen.clone();
            Router::new()
                .route(
                    "/auth",
                    get(|ws: WebSocketUpgrade| async move {
                        ws.protocols(["dcv.auth"]).on_upgrade(|mut socket| async move {
                            while let Some(Ok(WsMessage::Text(text))) = socket.recv().await {
                                if socket.send(WsMessage::text(format!("echo {}", text.as_str()))).await.is_err() {
                                    break;
                                }
                            }
                        })
                    }),
                )
                .route("/old", get(|| async { (StatusCode::FOUND, [(header::LOCATION, "/new/")]) }))
                .fallback(move |request: Request| {
                    let seen = seen.clone();
                    async move {
                        let path = request.uri().path_and_query().unwrap().to_string();
                        seen.lock().unwrap().push((path.clone(), request.headers().clone()));
                        (
                            [(header::CONTENT_SECURITY_POLICY, "upgrade-insecure-requests;"), (header::STRICT_TRANSPORT_SECURITY, "max-age=1")],
                            format!("gateway {path}"),
                        )
                    }
                })
        };
        let gateway = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let gateway_addr = gateway.local_addr().unwrap();
        tokio::spawn(async move { axum::serve(gateway, gateway_app).await.unwrap() });

        let dcv = service(Settings { owner: Some("jdoe".into()), ..enabled(&format!("http://{gateway_addr}")) }, None);
        let app: Router = router(dcv.clone());
        let app = tower::Layer::layer(
            &axum::middleware::from_fn_with_state(Some(Arc::<str>::from("jdoe")), crate::workspace_prefix::middleware),
            app,
        );
        let app = Router::new().fallback_service(app);
        let front = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let front_addr = front.local_addr().unwrap();
        tokio::spawn(async move { axum::serve(front, app).await.unwrap() });

        let client = reqwest::Client::builder().redirect(reqwest::redirect::Policy::none()).build().unwrap();
        let base = format!("http://{front_addr}/workspace/jdoe");
        let response = client
            .get(format!("{base}/dcv/ide/js/main.js?v=2"))
            .header(header::COOKIE, "ws_session=s; fos_ws=jdoe; keep=1")
            .header(header::ORIGIN, format!("http://{front_addr}"))
            .send()
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        assert!(!response.headers().contains_key(header::CONTENT_SECURITY_POLICY));
        assert!(!response.headers().contains_key(header::STRICT_TRANSPORT_SECURITY));
        assert_eq!(response.text().await.unwrap(), "gateway /js/main.js?v=2");
        {
            let seen = seen.lock().unwrap();
            let (path, headers) = seen.last().unwrap();
            assert_eq!(path, "/js/main.js?v=2");
            assert_eq!(headers[header::HOST], gateway_addr.to_string());
            assert_eq!(headers[header::ORIGIN], format!("http://{gateway_addr}"));
            assert_eq!(headers[header::COOKIE], "keep=1");
        }
        // An older link without the target: `ide`'s.
        let response = client.get(format!("{base}/dcv/js/old.js")).header(header::ORIGIN, format!("http://{front_addr}")).send().await.unwrap();
        assert_eq!(response.text().await.unwrap(), "gateway /js/old.js");
        // `/dcv`, `/dcv/ide` and the gateway's redirects stay in the workspace and the target.
        let response = client.get(format!("{base}/dcv?authToken=t")).send().await.unwrap();
        assert_eq!(response.headers()[header::LOCATION], "/workspace/jdoe/dcv/?authToken=t");
        let response = client.get(format!("{base}/dcv/ide?authToken=t")).send().await.unwrap();
        assert_eq!((response.status(), &response.headers()[header::LOCATION]), (StatusCode::PERMANENT_REDIRECT, &"/workspace/jdoe/dcv/ide/?authToken=t".parse::<HeaderValue>().unwrap()));
        let response = client.get(format!("{base}/dcv/ide/old")).send().await.unwrap();
        assert_eq!(response.headers()[header::LOCATION], "/workspace/jdoe/dcv/ide/new/");
        let response = client.get(format!("{base}/dcv/old")).send().await.unwrap();
        assert_eq!(response.headers()[header::LOCATION], "/workspace/jdoe/dcv/ide/new/");
        // Another workspace's prefix doesn't reach this backend.
        let response = client.get(format!("http://{front_addr}/workspace/alice/dcv/ide/")).send().await.unwrap();
        assert_eq!(response.status(), StatusCode::NOT_FOUND);
        let mint = || client.post(format!("{base}{}", api::INFRA_DCV_SESSION));
        assert_eq!(mint().send().await.unwrap().status(), StatusCode::UNSUPPORTED_MEDIA_TYPE);
        let cross = mint().header(header::CONTENT_TYPE, "application/json").header(header::ORIGIN, "http://evil.example").body("{}");
        assert_eq!(cross.send().await.unwrap().status(), StatusCode::FORBIDDEN);
        let session = mint().header(header::CONTENT_TYPE, "application/json").body("{}").send().await.unwrap().text().await.unwrap();
        let session: DcvSession = serde_json::from_str(&session).unwrap();
        assert!(session.url.starts_with("/dcv/ide/?authToken=jdoe.") && session.url.ends_with("#console"), "{}", session.url);
        assert_eq!((session.workspace.as_deref(), session.target.as_str()), (Some("jdoe"), "ide"));
        assert!(dcv.verify("console", token_of(&session.url), Some("127.0.0.1:1"), None));
        let json = || mint().header(header::CONTENT_TYPE, "application/json");
        let ide: DcvSession = serde_json::from_str(&json().body(r#"{"target":"ide"}"#).send().await.unwrap().text().await.unwrap()).unwrap();
        assert_eq!(ide.target, "ide");
        assert_eq!(json().body(r#"{"target":"robot9"}"#).send().await.unwrap().status(), StatusCode::NOT_FOUND);
        assert_eq!(json().body("not json").send().await.unwrap().status(), StatusCode::BAD_REQUEST);
        // Another site's page can't reach the gateway either.
        let response = client.get(format!("{base}/dcv/js/main.js")).header(header::ORIGIN, "https://cloudfront.example").send().await.unwrap();
        assert_eq!(response.status(), StatusCode::FORBIDDEN);

        // The WebSocket: upgraded end to end, the subprotocol the gateway's.
        let mut request = tokio_tungstenite::tungstenite::client::IntoClientRequest::into_client_request(format!(
            "ws://{front_addr}/workspace/jdoe/dcv/ide/auth"
        ))
        .unwrap();
        request.headers_mut().insert("sec-websocket-protocol", HeaderValue::from_static("dcv.auth"));
        let mut foreign = request.clone();
        foreign.headers_mut().insert(header::ORIGIN, HeaderValue::from_static("https://cloudfront.example"));
        assert!(tokio_tungstenite::connect_async(foreign).await.is_err(), "a foreign origin's WebSocket");
        request.headers_mut().insert(header::ORIGIN, HeaderValue::from_str(&format!("http://{front_addr}")).unwrap());
        let (mut socket, response) = tokio_tungstenite::connect_async(request).await.unwrap();
        assert_eq!(response.headers()["sec-websocket-protocol"], "dcv.auth");
        socket.send(Message::text("hello")).await.unwrap();
        let reply = socket.next().await.unwrap().unwrap();
        assert_eq!(reply.into_text().unwrap().as_str(), "echo hello");
        assert_eq!(dcv.activity.open_connections(), 1);
        socket.close(None).await.ok();
    }


    #[test]
    fn targets_peers_and_gateway_ips() {
        let vars = |pairs: Vec<(&'static str, String)>| move |name: &str| pairs.iter().find(|(k, _)| *k == name).map(|(_, v)| v.clone());
        let gw = ("FACTORY_OS_DCV_GATEWAY", "https://127.0.0.1:8443".to_owned());
        // Without targets: `ide`, the gateway's console.
        let s = Settings::from_vars(vars(vec![gw.clone()]));
        let all = s.all_targets();
        assert_eq!(all.len(), 1);
        assert_eq!((all[0].name.as_str(), all[0].session.as_str(), &all[0].host, all[0].resolver.port()), ("ide", "console", &HostSource::IdeLookup, 8446));
        // A smartPAD's own gateway and resolver.
        let robot = "robot1=gateway:https://127.0.0.1:8453,resolver:127.0.0.1:8456,session:console,server:10.0.0.5";
        let s = Settings::from_vars(vars(vec![gw.clone(), ("FACTORY_OS_DCV_TARGETS", robot.to_owned())]));
        assert!(s.error.is_none(), "{:?}", s.error);
        let names: Vec<String> = s.all_targets().into_iter().map(|t| t.name).collect();
        assert_eq!(names, ["ide", "robot1"]);
        let r = &s.targets[0];
        assert_eq!((r.gateway.addr.port(), r.resolver.port(), &r.host), (8453, 8456, &HostSource::Static("10.0.0.5".into())));
        // Two consoles on one gateway / resolver, a reserved or bad name, a resolver off loopback: off.
        for bad in [
            "robot1=session:console",
            "robot1=resolver:127.0.0.1:8446",
            "js=gateway:https://127.0.0.1:8453,resolver:127.0.0.1:8456",
            "Robot=gateway:https://127.0.0.1:8453,resolver:127.0.0.1:8456",
            "robot1=gateway:https://127.0.0.1:8453,resolver:10.0.0.1:8456",
            "robot1=gateway:https://127.0.0.1:8443,resolver:127.0.0.1:8456,session:x",
            "robot1=bogus:1",
        ] {
            let s = Settings::from_vars(vars(vec![gw.clone(), ("FACTORY_OS_DCV_TARGETS", bad.to_owned())]));
            assert!(s.error.is_some() && s.all_targets().is_empty(), "{bad}: {:?}", s.all_targets());
        }
        // Another session on the same gateway is fine.
        let s = Settings::from_vars(vars(vec![gw.clone(), ("FACTORY_OS_DCV_TARGETS", "pad=session:pad1".to_owned())]));
        assert!(s.error.is_none(), "{:?}", s.error);
        // Peers: fixed private http URLs only.
        let s = Settings::from_vars(vars(vec![
            gw.clone(),
            (
                "FACTORY_OS_DCV_VERIFIER_PEERS",
                "jdoe=http://10.0.1.10:8445/, evil=http://198.51.100.7:8445/, lo=http://127.0.0.1:8445/, x=https://10.0.0.1:8445/, bad name=http://10.0.0.2:1/, a=http://10.0.0.3:8445/path".to_owned(),
            ),
            ("FACTORY_OS_DCV_GATEWAY_IPS", "10.0.1.10, 127.0.0.1".to_owned()),
        ]));
        assert!(s.error.is_none());
        assert_eq!(s.peers, [("jdoe".to_owned(), "http://10.0.1.10:8445/".to_owned())]);
        assert_eq!(s.warnings.len(), 5, "{:?}", s.warnings);
        assert_eq!(s.gateway_ips, Some(vec!["10.0.1.10".parse().unwrap(), "127.0.0.1".parse().unwrap()]));
        assert!(parse_peer("jdoe=http://10.0.1.10:8445").is_ok());
        let s = Settings::from_vars(vars(vec![gw, ("FACTORY_OS_DCV_GATEWAY_IPS", "nope".to_owned())]));
        assert!(s.error.unwrap().contains("GATEWAY_IPS"));
        assert!(valid_target_name("ide") && valid_target_name("robot-1") && !valid_target_name("1x") && !valid_target_name("auth") && !valid_target_name(""));
    }

    #[tokio::test]
    async fn tokens_name_their_workspace_and_bind_to_its_gateway() {
        let gateway_ips = Some(vec!["10.0.1.10".parse().unwrap(), "127.0.0.1".parse().unwrap()]);
        let dcv = service(Settings { owner: Some("jdoe".into()), gateway_ips, ..enabled("http://127.0.0.1:1") }, Some("10.0.2.20"));
        let session = dcv.mint("ide").unwrap();
        assert!(session.url.starts_with("/dcv/ide/?authToken=jdoe."), "{}", session.url);
        let token = token_of(&session.url).to_owned();
        assert_eq!(token.len(), "jdoe.".len() + 64);
        assert_eq!(dcv.foreign(&token), None);
        assert_eq!(dcv.foreign("alice.00ff"), Some("alice"));
        assert_eq!(dcv.foreign("00ff"), None, "unprefixed: its own (older tokens)");
        assert!(!dcv.verify("console", "alice.00ff", Some("10.0.1.10:1"), None), "another workspace's");
        assert_eq!(dcv.mint("robot1").unwrap_err().0, StatusCode::NOT_FOUND);
        // The first client must be this machine's gateway (not, e.g., the test server's).
        assert!(!dcv.verify("console", &token, Some("10.0.2.1:5000"), None));
        // Once the resolver named the DCV server, only it redeems the tokens.
        assert!(dcv.resolve(SocketAddr::from((Ipv4Addr::LOCALHOST, 8446)), "console").await.is_some());
        assert!(!dcv.verify("console", &token, Some("10.0.1.10:5000"), Some("10.0.2.99")));
        assert!(dcv.verify("console", &token, Some("10.0.1.10:5000"), Some("10.0.2.20:40000")));
        assert!(dcv.verify("console", &token, Some("10.0.1.10:5001"), None), "a reconnect");
        assert!(!dcv.verify("other", &token, Some("10.0.1.10:5001"), None), "minted for ide: the console only");
    }

    /// A stand-in verifier answering `yes` / `no` (or slowly), recording what it was asked.
    async fn peer_verifier(answer: &'static str, delay: Duration) -> (String, Arc<Mutex<Vec<String>>>) {
        let seen: Arc<Mutex<Vec<String>>> = Arc::default();
        let app = Router::new().route(
            "/",
            post({
                let seen = seen.clone();
                move |body: String| async move {
                    seen.lock().unwrap().push(body);
                    tokio::time::sleep(delay).await;
                    format!(r#"<auth result="{answer}"><username>Administrator</username></auth>"#)
                }
            }),
        );
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let url = format!("http://{}/", listener.local_addr().unwrap());
        tokio::spawn(async move { axum::serve(listener, app).await.unwrap() });
        (url, seen)
    }

    #[tokio::test]
    async fn foreign_tokens_are_chained_to_their_workspace() {
        let (yes, asked) = peer_verifier("yes", Duration::ZERO).await;
        let (no, _) = peer_verifier("no", Duration::ZERO).await;
        let (slow, _) = peer_verifier("yes", Duration::from_secs(2)).await;
        let peers = vec![("alice".to_owned(), yes), ("bob".to_owned(), no), ("carol".to_owned(), slow)];
        let dcv = service(
            Settings { owner: Some("plan2cell-test".into()), peers, chain_timeout: Duration::from_millis(300), ..enabled("http://127.0.0.1:1") },
            None,
        );
        let (ok, chained) = dcv.check("console", "alice.0a", Some("10.0.1.10:5"), "10.0.2.20").await;
        assert_eq!((ok, chained.as_deref()), (true, Some("alice")));
        let body = asked.lock().unwrap()[0].clone();
        for part in ["sessionId=console", "authenticationToken=alice.0a", "dcvServer=10.0.2.20", "clientAddress=10.0.1.10%3A5"] {
            assert!(body.contains(part), "{body}");
        }
        assert_eq!(dcv.check("console", "bob.0a", None, "10.0.2.20").await, (false, Some("bob".into())));
        assert_eq!(dcv.check("console", "carol.0a", None, "10.0.2.20").await, (false, Some("carol".into())), "timed out");
        assert_eq!(dcv.check("console", "mallory.0a", None, "10.0.2.20").await, (false, None), "not a peer");
        // Its own tokens are checked here.
        let own = token_of(&dcv.mint("ide").unwrap().url).to_owned();
        assert!(own.starts_with("plan2cell-test."));
        assert_eq!(dcv.check("console", &own, Some("10.0.2.1:1"), "10.0.2.20").await, (true, None));
        assert!(said_yes(r#" <auth result="yes"><username>x</username></auth>"#) && !said_yes(r#"<auth result="no"/>"#) && !said_yes("yes"));

        // A peer's chained question (with dcvServer) is never chained on: no loops, no relaying.
        let before = asked.lock().unwrap().len();
        let mut request = Request::post("/")
            .header(header::CONTENT_TYPE, "application/x-www-form-urlencoded")
            .body(Body::from("sessionId=console&authenticationToken=alice.0a&clientAddress=10.0.1.10%3A5&dcvServer=10.0.2.20"))
            .unwrap();
        request.extensions_mut().insert(ConnectInfo("10.0.2.1:50000".parse::<SocketAddr>().unwrap()));
        let response = verifier_router().with_state(dcv.clone()).oneshot(request).await.unwrap();
        let body = String::from_utf8(axum::body::to_bytes(response.into_body(), 1 << 16).await.unwrap().to_vec()).unwrap();
        assert!(body.starts_with(r#"<auth result="no">"#), "{body}");
        assert_eq!(asked.lock().unwrap().len(), before, "alice's verifier not asked again");
    }

    /// The test server's verifier (the Windows DCV server's only one) chains jdoe's token to
    /// jdoe's, which binds it to its own gateway and runs its Open PLC IDE hook.
    #[tokio::test]
    async fn the_shared_dcv_server_reaches_each_workspace_through_one_verifier() {
        let opened = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let jdoe = service(
            Settings {
                owner: Some("jdoe".into()),
                gateway_ips: Some(vec!["10.0.1.10".parse().unwrap()]),
                ..enabled("http://127.0.0.1:1")
            },
            Some("10.0.2.20"),
        );
        jdoe.on_open({
            let opened = opened.clone();
            Arc::new(move || {
                opened.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            })
        });
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let peer_url = format!("http://{}/", listener.local_addr().unwrap());
        let router = verifier_router().with_state(jdoe.clone());
        tokio::spawn(async move { axum::serve(listener, router.into_make_service_with_connect_info::<SocketAddr>()).await.unwrap() });
        let test_server = service(
            Settings { owner: Some("plan2cell-test".into()), peers: vec![("jdoe".into(), peer_url)], ..enabled("http://127.0.0.1:1") },
            Some("10.0.2.20"),
        );
        let test_hook = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        test_server.on_open({
            let n = test_hook.clone();
            Arc::new(move || {
                n.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            })
        });
        // jdoe's viewer fetched the link; its gateway asked its resolver.
        let token = token_of(&jdoe.mint("ide").unwrap().url).to_owned();
        jdoe.resolve(SocketAddr::from((Ipv4Addr::LOCALHOST, 8446)), "console").await.unwrap();
        // The DCV server asks the test server's verifier.
        let ask = |client: &'static str, token: String| {
            let router = verifier_router().with_state(test_server.clone());
            let mut request = Request::post("/")
                .header(header::CONTENT_TYPE, "application/x-www-form-urlencoded")
                .body(Body::from(format!("sessionId=console&authenticationToken={token}&clientAddress={client}")))
                .unwrap();
            request.extensions_mut().insert(ConnectInfo("10.0.2.20:50000".parse::<SocketAddr>().unwrap()));
            async move {
                let response = router.oneshot(request).await.unwrap();
                String::from_utf8(axum::body::to_bytes(response.into_body(), 1 << 16).await.unwrap().to_vec()).unwrap()
            }
        };
        // Through the test server's public /dcv/ its gateway is the client: jdoe refuses.
        assert!(ask("10.0.2.1%3A40000", token.clone()).await.starts_with(r#"<auth result="no">"#));
        let yes = ask("10.0.1.10%3A40000", token.clone()).await;
        assert_eq!(yes, r#"<auth result="yes"><username>Administrator</username></auth>"#);
        assert!(ask("10.0.1.10%3A40001", token.clone()).await.starts_with(r#"<auth result="yes">"#), "a reconnect");
        assert_eq!(opened.load(std::sync::atomic::Ordering::SeqCst), 1, "jdoe's Open PLC IDE hook, once");
        assert_eq!(test_hook.load(std::sync::atomic::Ordering::SeqCst), 0, "not the test server's");
    }

    #[test]
    fn older_token_stores_load_as_ide() {
        let dir = std::env::temp_dir().join(format!("dcv-old-tokens-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let store = dir.join("dcv-tokens.json");
        let expires = unix_s(SystemTime::now()) + 60.0;
        std::fs::write(&store, json!({"00ab": {"session": "console", "expires_s": expires, "client": null}}).to_string()).unwrap();
        let opened = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let dcv = service(Settings { owner: Some("jdoe".into()), token_store: Some(store.clone()), ..enabled("http://127.0.0.1:1") }, None);
        dcv.on_open({
            let opened = opened.clone();
            Arc::new(move || {
                opened.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            })
        });
        assert!(dcv.verify("console", "00ab", Some("10.0.0.5:1"), None));
        assert_eq!(opened.load(std::sync::atomic::Ordering::SeqCst), 1);
        assert_eq!(load_tokens(&store)["00ab"].target, "ide");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[tokio::test]
    async fn each_resolver_answers_its_targets() {
        let robot = Target {
            name: "robot1".into(),
            gateway: Gateway::parse("http://127.0.0.1:2").unwrap(),
            cert: None,
            resolver: "127.0.0.1:8456".parse().unwrap(),
            session: "console".into(),
            host: HostSource::Static("10.0.0.5".into()),
            port: 8443,
        };
        let dcv = service(Settings { targets: vec![robot], ..enabled("http://127.0.0.1:1") }, Some("10.0.2.20"));
        assert_eq!(dcv.target_names(), ["ide", "robot1"]);
        let ide = dcv.resolve("127.0.0.1:8446".parse().unwrap(), "console").await.unwrap();
        assert_eq!((ide["SessionId"].as_str(), ide["DcvServerEndpoint"].as_str()), (Some("console"), Some("10.0.2.20")));
        let pad = dcv.resolve("127.0.0.1:8456".parse().unwrap(), "console").await.unwrap();
        assert_eq!(pad["DcvServerEndpoint"], "10.0.0.5");
        assert!(dcv.resolve("127.0.0.1:8457".parse().unwrap(), "console").await.is_none());
        let token = token_of(&dcv.mint("robot1").unwrap().url).to_owned();
        assert!(!dcv.verify("console", &token, Some("127.0.0.1:1"), Some("10.0.2.20")), "the IDE can't redeem a smartPAD's token");
        assert!(dcv.verify("console", &token, Some("127.0.0.1:1"), Some("10.0.0.5")));
    }

    #[tokio::test]
    async fn off_without_a_gateway() {
        let app: Router = router(service(Settings::default(), None));
        let response = app.oneshot(Request::get("/dcv/").body(Body::empty()).unwrap()).await.unwrap();
        assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);
    }

    #[test]
    fn pinned_tls_needs_a_readable_certificate() {
        let settings = Settings { gateway_cert: Some("/nonexistent/gateway.pem".into()), ..enabled("https://127.0.0.1:8443") };
        let dcv = service(settings, None);
        assert!(!dcv.enabled());
        assert!(dcv.mint("ide").unwrap_err().1.contains("FACTORY_OS_DCV_GATEWAY_CERT"));
        assert!(service(enabled("https://127.0.0.1:8443"), None).enabled());
    }
}
