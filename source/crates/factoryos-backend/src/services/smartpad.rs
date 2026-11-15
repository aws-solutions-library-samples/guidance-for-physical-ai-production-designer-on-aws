//! The cell's KUKA smartPADs in the browser (KDM's iiQKA.UI at `https://<KLI address>/` of each controller), through
//! this backend, one browser window per controller:
//!
//! * [`api::SMARTPAD`]`/<robot>/…` (also under `/workspace/<name>/`): a reverse proxy to that controller's KDM, HTTPS
//!   to its KLI address as the cell's `plan2cell cell` config names it (`[[kuka_vc.controller]]` `robot_ns`,
//!   `kli_ip`), mapped through its `[address_map]` (a workspace's own cell: the PLC host's secondary IP that reaches the
//!   station, 1:1 NAT; the shared cell: the address itself, routed to the PLC host). KDM's certificate is
//!   self-signed: any is accepted (a private address of the cell). HTTP and WebSocket upgrades (spliced byte for byte).
//!   iiQKA.UI only works at `/` of a host of its own (`factoryos_protocol::smartpad`), so a browser reaches it at
//!   `https://<robot>--<workspace>.<site>/`, which the workspace edge maps to this path. Towards KDM the request is
//!   one a browser in the cell would send (`Host` and `Origin` the KLI address, without the edge's cookies and
//!   headers); back, KDM's own address in redirects, `Set-Cookie` domains and JSON / HTML bodies becomes the
//!   smartPAD's hostname (`X-Forwarded-Host`).
//! * Only the page's own origin gets in, as for DCV (outside the backend's CORS layer): `Sec-Fetch-Site`
//!   `same-origin` / `none`, else an `Origin` that is the `Host` (or `X-Forwarded-Host`); top-level navigations
//!   excepted. Behind the workspace edge its `X-User-Alias` must be the workspace's owner (`--workspace`) or in
//!   `FACTORY_OS_SMARTPAD_USERS`: a smartPAD moves the robot (T1, EXT, its programs).
//! * `GET` [`api::SMARTPADS`]: [`SmartPads`], the controllers and their URLs: the edge names its smartPAD site in
//!   `X-SmartPad-Site` (none: no URLs, the edge has no smartPAD hostnames).
//! * A virtual cell whose template has no `smartpad` capability offers none ([`SmartPadService::set_gate`]; the
//!   proxy refuses too); a cell without a template, as before. A `driver = "hook"` controller is never one.

use std::net::IpAddr;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use anyhow::Context as _;
use axum::Router;
use axum::body::Body;
use axum::extract::{Request, State};
use axum::http::{HeaderMap, HeaderName, HeaderValue, StatusCode, Uri, Version, header};
use axum::response::{IntoResponse as _, Response};
use axum::routing::{any, get};
use factoryos_protocol::api;
use factoryos_protocol::smartpad::{SmartPad, SmartPads, valid_robot};
use hyper_util::rt::TokioIo;
use tokio::net::TcpStream;

use super::dcv::{is_edge_header, is_hop_by_hop, is_upgrade, safe_segment, same_origin, send, strip_edge_cookies, tls_connector};
use crate::activity::Activity;

/// KDM's port.
pub const KDM_PORT: u16 = 443;
/// The edge's header with its smartPAD site (`<robot>--<workspace>.<site>`).
pub const SITE_HEADER: &str = "x-smartpad-site";
const CONNECT_TIMEOUT: Duration = Duration::from_secs(10);
/// Bodies whose KDM address is rewritten at most (larger ones pass as they are).
const REWRITE_MAX: usize = 2 << 20;

type HttpError = (StatusCode, String);

#[derive(Clone, Debug)]
pub struct Settings {
    /// The `plan2cell cell` config (`FACTORY_OS_PLAN2CELL_CELL_CONFIG`, else `/etc/plan2cell/cell.toml`): read on
    /// every request, as a workspace's own cell rewrites it on each launch.
    pub config: PathBuf,
    /// The workspace's owner (the backend's `--workspace`).
    pub owner: Option<String>,
    /// Further aliases that may (`FACTORY_OS_SMARTPAD_USERS`, comma-separated).
    pub users: Vec<String>,
    /// KDM's port and scheme (tests: plain HTTP on another port).
    pub port: u16,
    pub tls: bool,
}

impl Settings {
    pub fn from_env(config: PathBuf) -> Self {
        let users = std::env::var("FACTORY_OS_SMARTPAD_USERS")
            .map(|users| users.split(',').map(str::trim).filter(|u| !u.is_empty()).map(str::to_owned).collect())
            .unwrap_or_default();
        Self { config, owner: None, users, port: KDM_PORT, tls: true }
    }
}

/// A controller of the cell config.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Controller {
    pub robot: String,
    pub kli_ip: String,
    /// Where this machine reaches it.
    pub address: IpAddr,
}

/// The controllers of a `plan2cell cell` config: each `[[kuka_vc.controller]]` with a `robot_ns` that can be a
/// hostname label and an IP `kli_ip`, mapped through `[address_map]`. A `driver = "hook"` controller (a robot
/// image driven through `factoryos/hook@1`, docs/tasks/cell-templates.md E8) has no KDM: never offered.
pub fn controllers(config: &str) -> anyhow::Result<Vec<Controller>> {
    let doc: toml::Table = config.parse().context("the cell config is not valid TOML")?;
    let map = doc.get("address_map").and_then(toml::Value::as_table);
    let list = doc.get("kuka_vc").and_then(|k| k.get("controller")).and_then(toml::Value::as_array).cloned().unwrap_or_default();
    Ok(list
        .iter()
        .filter(|c| c.get("driver").and_then(toml::Value::as_str).is_none_or(|d| d == "kss"))
        .filter_map(|c| {
            let robot = c.get("robot_ns")?.as_str()?.to_owned();
            let kli_ip = c.get("kli_ip")?.as_str()?.trim().to_owned();
            let address = map.and_then(|m| m.get(&kli_ip)).and_then(toml::Value::as_str).unwrap_or(&kli_ip).trim().parse().ok()?;
            (valid_robot(&robot) && kli_ip.parse::<IpAddr>().is_ok()).then_some(Controller { robot, kli_ip, address })
        })
        .collect())
}

/// `/smartpad/<robot>/x?q` → `(robot, /x?q)`; `/smartpad/<robot>` → `(robot, "")` (`None`: not a robot, or a
/// segment that is, or hides, a `.` / `..` or a separator once percent-decoded).
fn route(uri: &Uri) -> Option<(String, String)> {
    let rest = uri.path().strip_prefix(api::SMARTPAD)?.strip_prefix('/')?;
    if !rest.split('/').all(safe_segment) {
        return None;
    }
    let (robot, path) = rest.split_once('/').map_or((rest, None), |(r, p)| (r, Some(p)));
    if !valid_robot(robot) {
        return None;
    }
    let query = uri.query().map(|q| format!("?{q}")).unwrap_or_default();
    Some((robot.to_owned(), path.map(|p| format!("/{p}{query}")).unwrap_or_default()))
}

fn header_str<'a>(headers: &'a HeaderMap, name: &str) -> Option<&'a str> {
    headers.get(name).and_then(|v| v.to_str().ok()).map(str::trim).filter(|v| !v.is_empty())
}

/// The smartPAD's hostname the browser uses (the edge's `X-Forwarded-Host`).
fn public_host(headers: &HeaderMap) -> Option<String> {
    header_str(headers, "x-forwarded-host").and_then(|h| h.split(',').next()).map(|h| h.trim().to_ascii_lowercase())
}

/// `https://<kdm>[:443]` (KDM's own origin, as it writes it) → `https://<public>`, in `text`.
fn rewrite_origins(text: &str, kdm: &[String], public: &str) -> String {
    let mut out = text.to_owned();
    for addr in kdm {
        for scheme in ["https", "wss"] {
            out = out.replace(&format!("{scheme}://{addr}:443"), &format!("{scheme}://{public}"));
            out = out.replace(&format!("{scheme}://{addr}"), &format!("{scheme}://{public}"));
        }
    }
    out
}

/// A KDM redirect for the browser: its own origin (or a path) → the smartPAD's hostname, absolute (so the workspace
/// prefix stays off it); another site's as it is.
fn rewrite_location(location: &str, kdm: &[String], public: Option<&str>) -> Option<String> {
    let public = public?;
    if location.starts_with('/') && !location.starts_with("//") {
        return Some(format!("https://{public}{location}"));
    }
    let rewritten = rewrite_origins(location, kdm, public);
    (rewritten != location).then_some(rewritten)
}

/// A `Set-Cookie` without a `Domain` of KDM's (a host-only cookie of the smartPAD's hostname instead).
fn rewrite_set_cookie(value: &str, kdm: &[String]) -> String {
    value
        .split(';')
        .filter(|attr| {
            let attr = attr.trim();
            let Some((name, domain)) = attr.split_once('=') else { return true };
            !(name.trim().eq_ignore_ascii_case("domain") && kdm.iter().any(|a| a.eq_ignore_ascii_case(domain.trim().trim_start_matches('.'))))
        })
        .collect::<Vec<_>>()
        .join(";")
}

/// JSON and HTML that may name KDM's origin (kratos' flows); not compressed.
fn rewritable(headers: &HeaderMap) -> bool {
    let ty = header_str(headers, header::CONTENT_TYPE.as_str()).unwrap_or_default().to_ascii_lowercase();
    !headers.contains_key(header::CONTENT_ENCODING) && (ty.starts_with("application/json") || ty.contains("+json") || ty.starts_with("text/html"))
}

/// Why the cell offers no smartPADs now (`None`: it does): the cell's template capabilities.
pub type Gate = Arc<dyn Fn() -> Option<String> + Send + Sync>;

pub struct SmartPadService {
    settings: Settings,
    activity: Arc<Activity>,
    tls: Option<tokio_rustls::TlsConnector>,
    gate: std::sync::OnceLock<Gate>,
}

impl SmartPadService {
    pub fn new(settings: Settings, activity: Arc<Activity>) -> Arc<Self> {
        let tls = if settings.tls {
            match tls_connector(None) {
                Ok(tls) => Some(tls),
                Err(err) => {
                    tracing::warn!("smartPAD proxy: no TLS client: {err:#}");
                    None
                }
            }
        } else {
            None
        };
        Arc::new(Self { settings, activity, tls, gate: std::sync::OnceLock::new() })
    }

    /// Ask `gate` before every list and proxied request (set once, by main).
    pub fn set_gate(&self, gate: Gate) {
        let _ = self.gate.set(gate);
    }

    fn refusal(&self) -> Option<String> {
        self.gate.get().and_then(|gate| gate())
    }

    fn controllers(&self) -> Result<Vec<Controller>, String> {
        let path = &self.settings.config;
        let text = std::fs::read_to_string(path).map_err(|err| format!("no cell config {} ({err})", path.display()))?;
        controllers(&text).map_err(|err| format!("{}: {err:#}", path.display()))
    }

    fn may_use(&self, alias: &str) -> bool {
        self.settings.users.iter().any(|u| u == alias)
            || match &self.settings.owner {
                Some(owner) => owner == alias,
                None => self.settings.users.is_empty(),
            }
    }

    /// Whether `request` may reach a smartPAD: the workspace's user, from the page's own origin (or a navigation).
    pub fn admit(&self, request: &Request) -> Result<(), String> {
        let headers = request.headers();
        if let Some(alias) = header_str(headers, "x-user-alias")
            && !self.may_use(alias)
        {
            return Err(format!("the smartPADs of this workspace are not open to {alias}"));
        }
        let navigation = matches!(*request.method(), axum::http::Method::GET | axum::http::Method::HEAD)
            && headers.get("sec-fetch-mode").is_some_and(|v| v.as_bytes().eq_ignore_ascii_case(b"navigate"));
        if navigation || same_origin(headers) {
            Ok(())
        } else {
            Err("smartPAD: cross-origin request refused".to_owned())
        }
    }

    /// The controllers and their URLs, for the viewer's Open smartPAD buttons.
    pub fn list(&self, headers: &HeaderMap) -> SmartPads {
        let may_open = header_str(headers, "x-user-alias").is_none_or(|alias| self.may_use(alias));
        if let Some(why) = self.refusal() {
            return SmartPads { pads: Vec::new(), may_open, reason: Some(why) };
        }
        let controllers = match self.controllers() {
            Ok(c) => c,
            Err(err) => return SmartPads { pads: Vec::new(), may_open, reason: Some(err) },
        };
        let workspace = header_str(headers, "x-workspace").map(str::to_owned).or_else(|| self.settings.owner.clone());
        let site = header_str(headers, SITE_HEADER);
        let reason = match (site, &workspace) {
            (None, _) => Some(
                "The smartPAD opens on a hostname of its own on the workspace edge (robot1--<workspace>.<site>), which this \
                 connection doesn't come through (no X-SmartPad-Site)"
                    .to_owned(),
            ),
            (_, None) => Some("No workspace name on this server (--workspace)".to_owned()),
            _ => None,
        };
        if controllers.is_empty() {
            return SmartPads { pads: Vec::new(), may_open, reason: Some("No KUKA controllers in the cell config ([[kuka_vc.controller]])".to_owned()) };
        }
        let pads = controllers
            .into_iter()
            .map(|c| SmartPad {
                url: site.zip(workspace.as_deref()).and_then(|(site, ws)| factoryos_protocol::smartpad::url(&c.robot, ws, site)),
                robot: c.robot,
                kli_ip: c.kli_ip,
                address: c.address.to_string(),
            })
            .collect();
        SmartPads { pads, may_open, reason }
    }

    /// One request (an upgrade too) to its controller's KDM.
    async fn forward(&self, mut request: Request) -> Result<Response, HttpError> {
        let (robot, path) = route(request.uri()).ok_or((StatusCode::NOT_FOUND, "no such smartPAD".to_owned()))?;
        if let Some(why) = self.refusal() {
            return Err((StatusCode::NOT_FOUND, why));
        }
        if path.is_empty() {
            let query = request.uri().query().map(|q| format!("?{q}")).unwrap_or_default();
            return Ok((StatusCode::PERMANENT_REDIRECT, [(header::LOCATION, format!("{}/{robot}/{query}", api::SMARTPAD))]).into_response());
        }
        let controller = self
            .controllers()
            .map_err(|err| (StatusCode::SERVICE_UNAVAILABLE, err))?
            .into_iter()
            .find(|c| c.robot == robot)
            .ok_or_else(|| (StatusCode::NOT_FOUND, format!("no KUKA controller {robot} in this cell")))?;
        let public = public_host(request.headers());
        let kdm = vec![controller.kli_ip.clone(), controller.address.to_string()];
        let upgrade = is_upgrade(request.headers());
        let client_upgrade = upgrade.then(|| hyper::upgrade::on(&mut request));
        let (parts, body) = request.into_parts();
        let mut outgoing = hyper::Request::builder()
            .method(parts.method.clone())
            .uri(path)
            .version(Version::HTTP_11)
            .body(body)
            .map_err(|err| (StatusCode::BAD_REQUEST, format!("bad smartPAD request: {err}")))?;
        *outgoing.headers_mut() = self.upstream_headers(&parts.headers, &controller, upgrade);

        let target = std::net::SocketAddr::new(controller.address, self.settings.port);
        let bad_gateway = |err: anyhow::Error| (StatusCode::BAD_GATEWAY, format!("smartPAD {robot} (KDM at {target}): {err:#}"));
        let tcp = tokio::time::timeout(CONNECT_TIMEOUT, TcpStream::connect(target))
            .await
            .map_err(|_| bad_gateway(anyhow::anyhow!("connect timed out (is the KUKA VC host running?)")))?
            .map_err(|err| bad_gateway(err.into()))?;
        tcp.set_nodelay(true).ok();
        let mut response = match (&self.tls, self.settings.tls) {
            (Some(tls), _) => {
                let name = rustls::pki_types::ServerName::IpAddress(controller.address.into());
                let stream = tls.connect(name, tcp).await.context("TLS").map_err(bad_gateway)?;
                send(stream, outgoing).await.map_err(bad_gateway)?
            }
            (None, true) => return Err((StatusCode::SERVICE_UNAVAILABLE, "smartPAD proxy: no TLS client".to_owned())),
            (None, false) => send(tcp, outgoing).await.map_err(bad_gateway)?,
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
                            tracing::debug!("smartPAD websocket closed: {err}");
                        }
                    }
                    (Err(err), _) | (_, Err(err)) => tracing::debug!("smartPAD upgrade failed: {err}"),
                }
            });
        }
        let (mut parts, body) = response.into_parts();
        let mut headers = HeaderMap::new();
        for (name, value) in &parts.headers {
            if name == header::STRICT_TRANSPORT_SECURITY {
                // KDM's, not the edge's policy.
                continue;
            }
            if name == header::SET_COOKIE {
                if let Ok(v) = value.to_str()
                    && let Ok(v) = HeaderValue::from_str(&rewrite_set_cookie(v, &kdm))
                {
                    headers.append(name.clone(), v);
                }
                continue;
            }
            if !is_hop_by_hop(name) || (switching && (name == header::CONNECTION || name == header::UPGRADE)) {
                headers.append(name.clone(), value.clone());
            }
        }
        if let Some(location) = headers.get(header::LOCATION).and_then(|v| v.to_str().ok()).and_then(|l| rewrite_location(l, &kdm, public.as_deref()))
            && let Ok(value) = HeaderValue::from_str(&location)
        {
            headers.insert(header::LOCATION, value);
        }
        let small = headers
            .get(header::CONTENT_LENGTH)
            .and_then(|v| v.to_str().ok())
            .and_then(|v| v.parse::<usize>().ok())
            .is_some_and(|n| n <= REWRITE_MAX);
        parts.headers = headers;
        let body = Body::new(body);
        if let Some(public) = public.filter(|_| !switching && small && rewritable(&parts.headers)) {
            let bytes = axum::body::to_bytes(body, REWRITE_MAX).await.map_err(|err| bad_gateway(anyhow::anyhow!("{err}")))?;
            let text = String::from_utf8_lossy(&bytes);
            let rewritten = rewrite_origins(&text, &kdm, &public);
            if rewritten != text {
                parts.headers.insert(header::CONTENT_LENGTH, HeaderValue::from(rewritten.len()));
                return Ok(Response::from_parts(parts, Body::from(rewritten)));
            }
            return Ok(Response::from_parts(parts, Body::from(bytes)));
        }
        Ok(Response::from_parts(parts, body))
    }

    /// The browser's headers as a browser in the cell would send them to KDM: no hop-by-hop (but an upgrade's), none
    /// of the edge's cookies or headers, `Host` and `Origin` the KLI address.
    fn upstream_headers(&self, from: &HeaderMap, controller: &Controller, upgrade: bool) -> HeaderMap {
        let ours = |name: &HeaderName| {
            let n = name.as_str();
            n == "x-user-alias" || n == "x-workspace" || n == SITE_HEADER || n.starts_with("x-forwarded-") || n == "forwarded"
        };
        let mut headers = HeaderMap::new();
        for (name, value) in from {
            if is_hop_by_hop(name) || is_edge_header(name) || ours(name) || name == header::HOST || name == header::ORIGIN || name == header::COOKIE {
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
        let authority = if self.settings.port == KDM_PORT { controller.kli_ip.clone() } else { format!("{}:{}", controller.kli_ip, self.settings.port) };
        if let Ok(host) = HeaderValue::from_str(&authority) {
            headers.insert(header::HOST, host);
        }
        let scheme = if self.settings.tls { "https" } else { "http" };
        if from.contains_key(header::ORIGIN)
            && let Ok(origin) = HeaderValue::from_str(&format!("{scheme}://{authority}"))
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
}

/// The public routes (merged outside the backend's CORS layer): the proxy behind [`SmartPadService::admit`], the
/// list.
pub fn router(pads: Arc<SmartPadService>) -> Router {
    let proxy = Router::new()
        .route(&format!("{}/{{*rest}}", api::SMARTPAD), any(proxy))
        .route_layer(axum::middleware::from_fn_with_state(pads.clone(), guard));
    let list = Router::new().route(
        api::SMARTPADS,
        get(|State(pads): State<Arc<SmartPadService>>, headers: HeaderMap| async move { axum::Json(pads.list(&headers)) }),
    );
    proxy.merge(list).with_state(pads)
}

async fn guard(State(pads): State<Arc<SmartPadService>>, request: Request, next: axum::middleware::Next) -> Response {
    match pads.admit(&request) {
        Ok(()) => next.run(request).await,
        Err(why) => {
            tracing::warn!("{why}: {} {}", request.method(), request.uri().path());
            (StatusCode::FORBIDDEN, why).into_response()
        }
    }
}

async fn proxy(State(pads): State<Arc<SmartPadService>>, request: Request) -> Response {
    match pads.forward(request).await {
        Ok(response) => response,
        Err(err) => err.into_response(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use futures_util::{SinkExt as _, StreamExt as _};
    use tokio_tungstenite::tungstenite::Message;

    const CONFIG: &str = r#"
[kuka_vc]
[[kuka_vc.controller]]
robot_ns = "robot1"
kli_ip = "192.168.100.11"
[[kuka_vc.controller]]
robot_ns = "robot2"
kli_ip = "192.168.100.12"
[[kuka_vc.controller]]
robot_ns = "Bad-Name"
kli_ip = "192.168.100.13"

[address_map]
"192.168.100.10" = "10.0.5.11"
"192.168.100.11" = "10.0.5.12"
"#;

    #[test]
    fn controllers_of_the_cell_config() {
        let c = controllers(CONFIG).unwrap();
        assert_eq!(c.iter().map(|c| (c.robot.as_str(), c.kli_ip.as_str(), c.address.to_string())).collect::<Vec<_>>(), [
            ("robot1", "192.168.100.11", "10.0.5.12".to_owned()),
            ("robot2", "192.168.100.12", "192.168.100.12".to_owned()),
        ]);
        assert!(controllers("").unwrap().is_empty());
        assert!(controllers("[").is_err());
        // Hook robots (engine 2) have no smartPAD; `kss` is today's.
        let hook = "[kuka_vc]\n[[kuka_vc.controller]]\ndriver = \"hook\"\nrobot_ns = \"robot1\"\nkli_ip = \"10.77.1.11\"\nkubectl_pod = \"cell-jdoe/n1-robot-1/robot\"\n\
                    [[kuka_vc.controller]]\ndriver = \"kss\"\nrobot_ns = \"robot2\"\nkli_ip = \"10.77.1.12\"\n";
        assert_eq!(controllers(hook).unwrap().iter().map(|c| c.robot.as_str()).collect::<Vec<_>>(), ["robot2"]);
    }

    #[test]
    fn routes_and_rewrites() {
        let r = |p: &str| route(&p.parse().unwrap());
        assert_eq!(r("/smartpad/robot1/"), Some(("robot1".into(), "/".into())));
        assert_eq!(r("/smartpad/robot1/uid/iiQKA.UI/?a=1"), Some(("robot1".into(), "/uid/iiQKA.UI/?a=1".into())));
        assert_eq!(r("/smartpad/robot1"), Some(("robot1".into(), String::new())));
        for bad in ["/smartpad/", "/smartpad/robot1/../x", "/smartpad/robot1/%2e%2e/x", "/smartpad/Robot/", "/smartpadx/robot1/"] {
            assert_eq!(r(bad), None, "{bad}");
        }
        let kdm = ["192.168.100.11".to_owned(), "10.0.5.12".to_owned()];
        let public = Some("robot1--jdoe.site.dev");
        assert_eq!(rewrite_location("https://192.168.100.11/u/iiQKA.UI/", &kdm, public).as_deref(), Some("https://robot1--jdoe.site.dev/u/iiQKA.UI/"));
        assert_eq!(rewrite_location("https://10.0.5.12:443/x", &kdm, public).as_deref(), Some("https://robot1--jdoe.site.dev/x"));
        assert_eq!(rewrite_location("/login", &kdm, public).as_deref(), Some("https://robot1--jdoe.site.dev/login"));
        assert_eq!(rewrite_location("https://kuka.example/", &kdm, public), None);
        assert_eq!(rewrite_location("/login", &kdm, None), None);
        assert_eq!(rewrite_set_cookie("s=1; Path=/; Domain=192.168.100.11; Secure", &kdm), "s=1; Path=/; Secure");
        assert_eq!(rewrite_set_cookie("s=1; Domain=kuka.example", &kdm), "s=1; Domain=kuka.example");
        assert_eq!(rewrite_origins(r#"{"action":"https://192.168.100.11/self-service/login?flow=1","ws":"wss://192.168.100.11/hub"}"#, &kdm, "p.dev"),
            r#"{"action":"https://p.dev/self-service/login?flow=1","ws":"wss://p.dev/hub"}"#);
    }

    #[tokio::test]
    async fn proxies_a_controllers_kdm_for_the_owner_only() {
        use axum::extract::ws::{Message as WsMessage, WebSocketUpgrade};

        // A stand-in KDM: the redirect to its instance, a kratos flow, a cookie, the headers it got, a WebSocket.
        let kdm_app = Router::new()
            .route("/", get(|| async { (StatusCode::FOUND, [(header::LOCATION, "https://192.168.100.11/uid-1/iiQKA.UI/")]) }))
            .route(
                "/self-service/login/browser",
                get(|| async {
                    (
                        [(header::CONTENT_TYPE, "application/json"), (header::SET_COOKIE, "csrf=1; Path=/; Domain=192.168.100.11; Secure")],
                        r#"{"ui":{"action":"https://192.168.100.11/self-service/login?flow=f1"}}"#,
                    )
                }),
            )
            .route(
                "/headers",
                get(|headers: HeaderMap| async move {
                    let h = |n: &str| headers.get(n).and_then(|v| v.to_str().ok()).unwrap_or("-").to_owned();
                    format!("host={} origin={} cookie={} alias={} fwd={}", h("host"), h("origin"), h("cookie"), h("x-user-alias"), h("x-forwarded-host"))
                }),
            )
            .route(
                "/hub",
                get(|ws: WebSocketUpgrade| async move {
                    ws.on_upgrade(|mut socket| async move {
                        while let Some(Ok(WsMessage::Text(text))) = socket.recv().await {
                            if socket.send(WsMessage::text(format!("kdm {}", text.as_str()))).await.is_err() {
                                break;
                            }
                        }
                    })
                }),
            );
        let kdm = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = kdm.local_addr().unwrap().port();
        tokio::spawn(async move { axum::serve(kdm, kdm_app).await.unwrap() });

        let dir = std::env::temp_dir().join(format!("smartpad-test-{}-{port}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let config = dir.join("cell.toml");
        std::fs::write(&config, CONFIG.replace("10.0.5.12", "127.0.0.1")).unwrap();
        let activity = Activity::new();
        let settings = Settings { config, owner: Some("jdoe".into()), users: Vec::new(), port, tls: false };
        let pads = SmartPadService::new(settings, activity.clone());
        // The cell's template capabilities (none: a cell without a template, every smartPAD).
        let no_smartpad: Arc<std::sync::Mutex<Option<String>>> = Arc::default();
        let gate = no_smartpad.clone();
        pads.set_gate(Arc::new(move || gate.lock().unwrap().clone()));
        let app: Router = router(pads);
        let app = tower::Layer::layer(
            &axum::middleware::from_fn_with_state(Some(Arc::<str>::from("jdoe")), crate::workspace_prefix::middleware),
            app,
        );
        let app = Router::new().fallback_service(app);
        let front = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let front_addr = front.local_addr().unwrap();
        tokio::spawn(async move { axum::serve(front, app).await.unwrap() });

        // As the edge sends it: the smartPAD's hostname in X-Forwarded-Host, the user, the edge's cookies.
        let public = "robot1--jdoe.site.dev";
        let client = reqwest::Client::builder().redirect(reqwest::redirect::Policy::none()).build().unwrap();
        let base = format!("http://{front_addr}/workspace/jdoe/smartpad/robot1");
        let edge = |rb: reqwest::RequestBuilder| rb.header("x-forwarded-host", public).header("x-user-alias", "jdoe").header("x-workspace", "jdoe");
        let landing = edge(client.get(format!("{base}/"))).send().await.unwrap();
        assert_eq!(landing.status(), StatusCode::FOUND);
        assert_eq!(landing.headers()[header::LOCATION], "https://robot1--jdoe.site.dev/uid-1/iiQKA.UI/");
        let flow = edge(client.get(format!("{base}/self-service/login/browser"))).send().await.unwrap();
        assert_eq!(flow.headers()[header::SET_COOKIE], "csrf=1; Path=/; Secure");
        assert_eq!(flow.text().await.unwrap(), r#"{"ui":{"action":"https://robot1--jdoe.site.dev/self-service/login?flow=f1"}}"#);
        let seen = edge(client.get(format!("{base}/headers")))
            .header(header::ORIGIN, format!("https://{public}"))
            .header(header::COOKIE, "ws_session=jdoe.1.x; fos_ws=jdoe; ory_kratos_session=k")
            .send()
            .await
            .unwrap();
        assert_eq!(seen.text().await.unwrap(), format!("host=192.168.100.11:{port} origin=http://192.168.100.11:{port} cookie=ory_kratos_session=k alias=- fwd=-"));

        // The owner only; the page's own origin only; the cell's robots only.
        let other = client.get(format!("{base}/")).header("x-user-alias", "alice").send().await.unwrap();
        assert_eq!(other.status(), StatusCode::FORBIDDEN);
        let cross = edge(client.get(format!("{base}/"))).header(header::ORIGIN, "https://evil.example").send().await.unwrap();
        assert_eq!(cross.status(), StatusCode::FORBIDDEN);
        let unknown = edge(client.get(format!("http://{front_addr}/workspace/jdoe/smartpad/robot9/"))).send().await.unwrap();
        assert_eq!(unknown.status(), StatusCode::NOT_FOUND);
        // robot2 has no address_map entry: its KLI address itself, not reachable from here.
        let list: SmartPads = serde_json::from_str(&client
            .get(format!("http://{front_addr}/workspace/jdoe{}", api::SMARTPADS))
            .header("x-user-alias", "jdoe")
            .header("x-workspace", "jdoe")
            .header(SITE_HEADER, "site.dev")
            .send()
            .await
            .unwrap()
            .text()
            .await
            .unwrap())
        .unwrap();
        assert!(list.may_open && list.reason.is_none(), "{list:?}");
        assert_eq!(list.pads.iter().map(|p| (p.robot.as_str(), p.url.as_deref())).collect::<Vec<_>>(), [
            ("robot1", Some("https://robot1--jdoe.site.dev/")),
            ("robot2", Some("https://robot2--jdoe.site.dev/")),
        ]);
        let direct: SmartPads = serde_json::from_str(&client.get(format!("http://{front_addr}{}", api::SMARTPADS)).send().await.unwrap().text().await.unwrap()).unwrap();
        assert!(direct.reason.is_some() && direct.pads.iter().all(|p| p.url.is_none()), "{direct:?}");

        // The WebSocket, end to end; a foreign origin's refused.
        let mut request = tokio_tungstenite::tungstenite::client::IntoClientRequest::into_client_request(format!(
            "ws://{front_addr}/workspace/jdoe/smartpad/robot1/hub"
        ))
        .unwrap();
        request.headers_mut().insert("x-forwarded-host", HeaderValue::from_static(public));
        let mut foreign = request.clone();
        foreign.headers_mut().insert(header::ORIGIN, HeaderValue::from_static("https://evil.example"));
        assert!(tokio_tungstenite::connect_async(foreign).await.is_err(), "a foreign origin's WebSocket");
        request.headers_mut().insert(header::ORIGIN, HeaderValue::from_str(&format!("https://{public}")).unwrap());
        let (mut socket, _) = tokio_tungstenite::connect_async(request).await.unwrap();
        socket.send(Message::text("mode?")).await.unwrap();
        assert_eq!(socket.next().await.unwrap().unwrap().into_text().unwrap().as_str(), "kdm mode?");
        assert_eq!(activity.open_connections(), 1);
        socket.close(None).await.ok();

        // A cell whose template has no smartpad: none listed, none proxied.
        *no_smartpad.lock().unwrap() = Some("smartPAD: this virtual cell's template x@1.0.0 has no smartpad".into());
        let list: SmartPads = serde_json::from_str(&client
            .get(format!("http://{front_addr}/workspace/jdoe{}", api::SMARTPADS))
            .header("x-user-alias", "jdoe")
            .header(SITE_HEADER, "site.dev")
            .send()
            .await
            .unwrap()
            .text()
            .await
            .unwrap())
        .unwrap();
        assert!(list.pads.is_empty() && list.reason.as_deref().is_some_and(|r| r.contains("no smartpad")), "{list:?}");
        assert_eq!(edge(client.get(format!("{base}/"))).send().await.unwrap().status(), StatusCode::NOT_FOUND);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
