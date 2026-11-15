//! WebRTC livestreams in the browser (NVIDIA Isaac Sim / Omniverse Kit), through this backend, as NICE
//! DCV ([`super::dcv`]):
//!
//! * [`api::RTC`]`/<target>/…` (also under `/workspace/<name>/`): a reverse proxy to the target's
//!   relay on loopback: a fos-isaac-bridge (`plan2cell/infra/omniverse/isaac-bridge`) serving
//!   NVIDIA's web viewer and relaying its signaling WebSocket to the stream's host over Zenoh. HTTP
//!   and WebSocket upgrades, spliced byte for byte. `/rtc/<target>/x` is the relay's `/x`: the viewer
//!   is built with relative URLs and signals at its page's own `…/signal`, so nothing is rewritten.
//!   The media (UDP) goes from the browser to the stream's host directly; its ICE and DTLS
//!   credentials only travel through this signaling.
//! * Only the page's own origin gets in, as for DCV (these routes are outside the CORS layer):
//!   `Sec-Fetch-Site` `same-origin` / `none`, else an `Origin` that is the `Host` (or
//!   `X-Forwarded-Host`); top-level navigations to `/rtc/` excepted. Behind the workspace edge (its
//!   login), its `X-User-Alias` must be the workspace's owner (`--workspace`) or in `FACTORY_OS_RTC_USERS`:
//!   the stream's viewer controls the application (keyboard, mouse), so it is the owner's alone.
//!
//! * [`api::ISAAC_LINK`]`/<name>` (under `/workspace/<name>/`): an Isaac Sim instance's Zenoh router,
//!   connected outbound through the edge (a WebSocket), proxied to its bridge's own Zenoh listener on
//!   loopback (the instance never reaches this backend's bus). No user login: the edge has checked an IAM
//!   identity (the instance's role `factoryos-isaac-<name>`) and says so in `X-Isaac-Principal`, which
//!   must be this `<name>`; a request with an `X-User-Alias` (a person's) is refused. Behind the edge
//!   only (the ALB's origin check): a backend reachable otherwise must not have links configured.
//!
//! Targets: `FACTORY_OS_RTC_TARGETS` = `name=http://127.0.0.1:<port>[;…]` (loopback only), links
//! `FACTORY_OS_ISAAC_LINKS` likewise, from the environment and `/etc/factoryos/rtc.env`
//! (`FACTORY_OS_RTC_ENV` overrides its path). Without one, `/rtc/` (or the link) is `503`.

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use axum::body::Body;
use axum::extract::{Path, Request, State};
use axum::http::{HeaderMap, HeaderValue, StatusCode, Uri, Version, header};
use axum::response::{IntoResponse as _, Response};
use axum::routing::any;
use axum::Router;
use factoryos_protocol::api;
use hyper_util::rt::TokioIo;
use tokio::net::TcpStream;

use super::dcv::{Gateway, is_hop_by_hop, is_upgrade, safe_segment, same_origin, send, upstream_headers};
use crate::activity::Activity;

/// The env file of the relay's installer.
pub const RTC_ENV: &str = "/etc/factoryos/rtc.env";
const CONNECT_TIMEOUT: Duration = Duration::from_secs(10);

type HttpError = (StatusCode, String);

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Settings {
    /// `(name, relay)`: `/rtc/<name>/` is that relay's `/`.
    pub targets: Vec<(String, Gateway)>,
    /// `(name, listener)`: `/isaac-link/<name>` is that bridge's Zenoh WebSocket listener.
    pub links: Vec<(String, Gateway)>,
    /// The workspace's owner (the backend's `--workspace`): the edge's `X-User-Alias` that may watch.
    pub owner: Option<String>,
    /// Further aliases that may (`FACTORY_OS_RTC_USERS`, comma-separated).
    pub users: Vec<String>,
    /// Why `/rtc/` is off although asked for (a bad setting).
    pub error: Option<String>,
}

impl Settings {
    /// From the environment and [`RTC_ENV`] (the environment wins).
    pub fn from_env() -> Self {
        let file = std::env::var_os("FACTORY_OS_RTC_ENV").map_or_else(|| PathBuf::from(RTC_ENV), PathBuf::from);
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

    pub fn from_vars(var: impl Fn(&str) -> Option<String>) -> Self {
        let mut settings = Self::default();
        let mut errors = Vec::new();
        for (key, into) in [("FACTORY_OS_RTC_TARGETS", &mut settings.targets), ("FACTORY_OS_ISAAC_LINKS", &mut settings.links)] {
            let Some(spec) = var(key) else { continue };
            for entry in spec.split([';', '\n']).map(str::trim).filter(|e| !e.is_empty()) {
                let Some((name, url)) = entry.split_once('=') else {
                    errors.push(format!("{key} {entry:?}: not name=http://127.0.0.1:<port>"));
                    continue;
                };
                let name = name.trim();
                if !super::dcv::valid_target_name(name) {
                    errors.push(format!("{key} {entry:?}: bad target name"));
                } else if into.iter().any(|(n, _)| n == name) {
                    errors.push(format!("{key} {entry:?}: {name} twice"));
                } else {
                    match Gateway::parse(url).map_err(|err| err.replace("FACTORY_OS_DCV_GATEWAY", "relay")) {
                        Ok(gateway) if !gateway.tls => into.push((name.to_owned(), gateway)),
                        Ok(_) => errors.push(format!("{key} {entry:?}: the relay is plain http on loopback")),
                        Err(err) => errors.push(format!("{key} {entry:?}: {err}")),
                    }
                }
            }
        }
        settings.users = var("FACTORY_OS_RTC_USERS")
            .map(|users| users.split(',').map(str::trim).filter(|u| !u.is_empty()).map(str::to_owned).collect())
            .unwrap_or_default();
        settings.error = (!errors.is_empty()).then(|| errors.join("; "));
        settings
    }
}

/// Where a request under `/rtc/` goes.
#[derive(Debug, PartialEq, Eq)]
pub enum Route {
    /// To target `0`'s relay, at path (and query) `1`.
    Proxy(String, String),
    /// `/rtc/<target>` → `/rtc/<target>/` (with the query).
    Slash(String, String),
}

/// `/rtc/<target>/x?q` → the target's `/x?q`; `/rtc/<target>` → `/rtc/<target>/` (`None`: not a
/// target, or a segment that is, or hides, a `.` / `..` or a separator once percent-decoded).
pub fn route(uri: &Uri, is_target: impl Fn(&str) -> bool) -> Option<Route> {
    let rest = uri.path().strip_prefix(api::RTC)?.strip_prefix('/')?;
    if !rest.split('/').all(safe_segment) {
        return None;
    }
    let query = uri.query().map(|q| format!("?{q}")).unwrap_or_default();
    let (name, path) = rest.split_once('/').map_or((rest, None), |(n, p)| (n, Some(p)));
    if !is_target(name) {
        return None;
    }
    Some(match path {
        None => Route::Slash(name.to_owned(), query),
        Some(path) => Route::Proxy(name.to_owned(), format!("/{path}{query}")),
    })
}

pub struct RtcService {
    settings: Settings,
    activity: Arc<Activity>,
}

impl RtcService {
    pub fn new(settings: Settings, activity: Arc<Activity>) -> Arc<Self> {
        if let Some(err) = &settings.error {
            tracing::warn!("WebRTC proxy off: {err}");
        } else if !settings.targets.is_empty() {
            let names: Vec<_> = settings.targets.iter().map(|(n, g)| format!("{n} → {}", g.origin())).collect();
            tracing::info!("WebRTC proxy at {}/<target>/: {}", api::RTC, names.join(", "));
        }
        Arc::new(Self { settings, activity })
    }

    fn may_use(&self, alias: &str) -> bool {
        self.settings.users.iter().any(|u| u == alias)
            || match &self.settings.owner {
                Some(owner) => owner == alias,
                None => self.settings.users.is_empty(),
            }
    }

    /// Whether `request` may reach the relay: the workspace's user, from the page's own origin (or a
    /// top-level navigation to `/rtc/`).
    pub fn admit(&self, request: &Request) -> Result<(), String> {
        let headers = request.headers();
        if let Some(alias) = headers.get("x-user-alias").and_then(|v| v.to_str().ok()).map(str::trim)
            && !self.may_use(alias)
        {
            return Err(format!("the WebRTC streams of this workspace are not open to {alias}"));
        }
        let navigation = matches!(*request.method(), axum::http::Method::GET | axum::http::Method::HEAD)
            && headers.get("sec-fetch-mode").is_some_and(|v| v.as_bytes().eq_ignore_ascii_case(b"navigate"));
        if navigation || same_origin(headers) {
            Ok(())
        } else {
            Err("WebRTC: cross-origin request refused".to_owned())
        }
    }

    /// The streams, for the viewer's "Watch in Isaac Sim" (none while off).
    pub fn list(&self, headers: &HeaderMap) -> factoryos_protocol::rtc::RtcTargets {
        let alias = headers.get("x-user-alias").and_then(|v| v.to_str().ok()).map(str::trim);
        factoryos_protocol::rtc::RtcTargets {
            targets: if self.settings.error.is_some() { Vec::new() } else { self.settings.targets.iter().map(|(n, _)| n.clone()).collect() },
            may_watch: alias.is_none_or(|alias| self.may_use(alias)),
            error: self.settings.error.clone(),
        }
    }

    fn relay(&self, name: &str) -> Option<&Gateway> {
        self.settings.targets.iter().find(|(n, _)| n == name).map(|(_, g)| g)
    }

    fn link(&self, name: &str) -> Option<&Gateway> {
        self.settings.links.iter().find(|(n, _)| n == name).map(|(_, g)| g)
    }

    /// Whether `request` is the edge-verified link of Isaac instance `name`.
    pub fn admit_link(&self, request: &Request, name: &str) -> Result<(), String> {
        let headers = request.headers();
        if headers.contains_key("x-user-alias") {
            return Err("an Isaac link is no person's: refused".to_owned());
        }
        match headers.get("x-isaac-principal").and_then(|v| v.to_str().ok()) {
            Some(principal) if principal == name => Ok(()),
            Some(principal) => Err(format!("Isaac link {name}: the edge verified {principal}, not {name}")),
            None => Err(format!("Isaac link {name}: not verified by the edge")),
        }
    }

    /// The link's WebSocket to its bridge's Zenoh listener (`/`).
    async fn forward_link(&self, request: Request, name: &str) -> Result<Response, HttpError> {
        if let Some(err) = &self.settings.error {
            return Err((StatusCode::SERVICE_UNAVAILABLE, format!("Isaac links off: {err}")));
        }
        let listener = self.link(name).cloned().ok_or((StatusCode::NOT_FOUND, format!("no Isaac link {name} here")))?;
        if !is_upgrade(request.headers()) {
            return Err((StatusCode::BAD_REQUEST, "an Isaac link is a WebSocket".to_owned()));
        }
        self.splice(request, "/".to_owned(), &listener, name).await
    }

    /// Forward one request (an upgrade too) to its target's relay.
    async fn forward(&self, request: Request) -> Result<Response, HttpError> {
        if let Some(err) = &self.settings.error {
            return Err((StatusCode::SERVICE_UNAVAILABLE, format!("WebRTC proxy off: {err}")));
        }
        if self.settings.targets.is_empty() {
            return Err((StatusCode::SERVICE_UNAVAILABLE, "no WebRTC stream on this machine (FACTORY_OS_RTC_TARGETS)".to_owned()));
        }
        let (name, path) = match route(request.uri(), |n| self.relay(n).is_some()) {
            Some(Route::Proxy(name, path)) => (name, path),
            Some(Route::Slash(name, query)) => {
                let location = format!("{}{query}", api::rtc_target(&name));
                return Ok((StatusCode::PERMANENT_REDIRECT, [(header::LOCATION, location)]).into_response());
            }
            None => return Err((StatusCode::NOT_FOUND, "no such WebRTC stream".to_owned())),
        };
        let relay = self.relay(&name).cloned().expect("routed to a target");
        let mut response = self.splice(request, path, &relay, &name).await?;
        // The relay's own redirects (none today) stay under the target.
        if let Some(location) = response.headers().get(header::LOCATION).and_then(|v| v.to_str().ok()).filter(|l| l.starts_with('/'))
            && let Ok(value) = HeaderValue::from_str(&format!("{}/{name}{location}", api::RTC))
        {
            response.headers_mut().insert(header::LOCATION, value);
        }
        Ok(response)
    }

    /// One request (an upgrade too) to a loopback upstream, its WebSocket spliced byte for byte.
    async fn splice(&self, mut request: Request, path: String, relay: &Gateway, name: &str) -> Result<Response, HttpError> {
        let upgrade = is_upgrade(request.headers());
        let client_upgrade = upgrade.then(|| hyper::upgrade::on(&mut request));
        let (parts, body) = request.into_parts();
        let mut outgoing = hyper::Request::builder()
            .method(parts.method.clone())
            .uri(path)
            .version(Version::HTTP_11)
            .body(body)
            .map_err(|err| (StatusCode::BAD_REQUEST, format!("bad WebRTC request: {err}")))?;
        *outgoing.headers_mut() = upstream_headers(&parts.headers, relay, upgrade);

        let bad_gateway = |err: anyhow::Error| (StatusCode::BAD_GATEWAY, format!("WebRTC relay {} ({name}): {err:#}", relay.origin()));
        let tcp = tokio::time::timeout(CONNECT_TIMEOUT, TcpStream::connect(relay.addr))
            .await
            .map_err(|_| bad_gateway(anyhow::anyhow!("connect timed out")))?
            .map_err(|err| bad_gateway(err.into()))?;
        tcp.set_nodelay(true).ok();
        let mut response = send(tcp, outgoing).await.map_err(bad_gateway)?;

        let switching = response.status() == StatusCode::SWITCHING_PROTOCOLS;
        if switching && let Some(client) = client_upgrade {
            let upstream = hyper::upgrade::on(&mut response);
            // An open stream is use: the workspace's idle stop waits for it.
            let open = self.activity.connection();
            tokio::spawn(async move {
                let _open = open;
                match (client.await, upstream.await) {
                    (Ok(client), Ok(upstream)) => {
                        let (mut client, mut upstream) = (TokioIo::new(client), TokioIo::new(upstream));
                        if let Err(err) = tokio::io::copy_bidirectional(&mut client, &mut upstream).await {
                            tracing::debug!("WebRTC signaling closed: {err}");
                        }
                    }
                    (Err(err), _) | (_, Err(err)) => tracing::debug!("WebRTC upgrade failed: {err}"),
                }
            });
        }
        let (mut parts, body) = response.into_parts();
        let mut headers = HeaderMap::new();
        for (name, value) in &parts.headers {
            if !is_hop_by_hop(name) || (switching && (name == header::CONNECTION || name == header::UPGRADE)) {
                headers.append(name.clone(), value.clone());
            }
        }
        parts.headers = headers;
        Ok(Response::from_parts(parts, Body::new(body)))
    }
}

pub fn router(rtc: Arc<RtcService>) -> Router {
    let streams = Router::new()
        .route(api::RTC, any(to_slash))
        .route(&format!("{}/", api::RTC), any(|| async { (StatusCode::NOT_FOUND, "no such WebRTC stream") }))
        .route(&format!("{}/{{*rest}}", api::RTC), any(proxy))
        .route_layer(axum::middleware::from_fn_with_state(rtc.clone(), guard));
    // Not behind `guard` (no person's session): `link` admits the edge-verified instance only.
    let links = Router::new().route(&format!("{}/{{name}}", api::ISAAC_LINK), any(link));
    let list = Router::new().route(api::RTC_TARGETS, axum::routing::get(|State(rtc): State<Arc<RtcService>>, headers: HeaderMap| async move {
        axum::Json(rtc.list(&headers))
    }));
    streams.merge(links).merge(list).with_state(rtc)
}

async fn link(State(rtc): State<Arc<RtcService>>, Path(name): Path<String>, request: Request) -> Response {
    if let Err(why) = rtc.admit_link(&request, &name) {
        tracing::warn!("{why}: {} {}", request.method(), request.uri().path());
        return (StatusCode::FORBIDDEN, why).into_response();
    }
    match rtc.forward_link(request, &name).await {
        Ok(response) => response,
        Err(err) => err.into_response(),
    }
}

async fn guard(State(rtc): State<Arc<RtcService>>, request: Request, next: axum::middleware::Next) -> Response {
    match rtc.admit(&request) {
        Ok(()) => next.run(request).await,
        Err(why) => {
            tracing::warn!("{why}: {} {}", request.method(), request.uri().path());
            (StatusCode::FORBIDDEN, why).into_response()
        }
    }
}

async fn to_slash(uri: Uri) -> Response {
    let query = uri.query().map(|q| format!("?{q}")).unwrap_or_default();
    (StatusCode::PERMANENT_REDIRECT, [(header::LOCATION, format!("{}/{query}", api::RTC))]).into_response()
}

async fn proxy(State(rtc): State<Arc<RtcService>>, request: Request) -> Response {
    match rtc.forward(request).await {
        Ok(response) => response,
        Err(err) => err.into_response(),
    }
}

/// For tests: a relay's address.
#[cfg(test)]
fn relay_at(addr: std::net::SocketAddr) -> Gateway {
    Gateway::parse(&format!("http://{addr}")).unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    use futures_util::{SinkExt as _, StreamExt as _};
    use tokio_tungstenite::tungstenite::Message;

    fn vars(pairs: &'static [(&'static str, &'static str)]) -> impl Fn(&str) -> Option<String> {
        move |name: &str| pairs.iter().find(|(k, _)| *k == name).map(|(_, v)| (*v).to_owned())
    }

    #[test]
    fn settings() {
        let s = Settings::from_vars(vars(&[("FACTORY_OS_RTC_TARGETS", "isaac=http://127.0.0.1:8470; isaac2=http://localhost:8471"), ("FACTORY_OS_RTC_USERS", "a, b"), ("FACTORY_OS_ISAAC_LINKS", "i-0abc=http://127.0.0.1:7460")]));
        assert_eq!(s.error, None);
        assert_eq!(s.links.iter().map(|(n, g)| (n.as_str(), g.addr.port())).collect::<Vec<_>>(), [("i-0abc", 7460)]);
        assert_eq!(s.targets.iter().map(|(n, g)| (n.as_str(), g.addr.port())).collect::<Vec<_>>(), [("isaac", 8470), ("isaac2", 8471)]);
        assert_eq!(s.users, ["a", "b"]);
        for bad in ["isaac=http://10.0.0.5:8470", "isaac=https://127.0.0.1:8470", "isaac", "../x=http://127.0.0.1:1", "a=http://127.0.0.1:1;a=http://127.0.0.1:2"] {
            let spec: &'static str = Box::leak(bad.to_owned().into_boxed_str());
            let pairs: &'static [(&str, &str)] = Box::leak(vec![("FACTORY_OS_RTC_TARGETS", spec)].into_boxed_slice());
            assert!(Settings::from_vars(vars(pairs)).error.is_some(), "{bad}");
        }
        assert_eq!(Settings::from_vars(vars(&[])), Settings::default());
    }

    #[test]
    fn routes() {
        let is = |n: &str| n == "isaac";
        let r = |p: &str| route(&p.parse().unwrap(), is);
        assert_eq!(r("/rtc/isaac/"), Some(Route::Proxy("isaac".into(), "/".into())));
        assert_eq!(r("/rtc/isaac/signal/sign_in?peer_id=p&version=2"), Some(Route::Proxy("isaac".into(), "/signal/sign_in?peer_id=p&version=2".into())));
        assert_eq!(r("/rtc/isaac?x=1"), Some(Route::Slash("isaac".into(), "?x=1".into())));
        for bad in ["/rtc/other/", "/rtc/isaac/../x", "/rtc/isaac/%2e%2e/x", "/rtc/isaac/a%2fb", "/rtcx/isaac/", "/rtc"] {
            assert_eq!(r(bad), None, "{bad}");
        }
    }

    #[tokio::test]
    async fn proxies_the_viewer_and_its_signaling_for_the_owner_only() {
        use axum::extract::ws::{Message as WsMessage, WebSocketUpgrade};
        use axum::routing::get;

        let relay_app = Router::new()
            .route(
                "/signal/sign_in",
                get(|ws: WebSocketUpgrade| async move {
                    ws.on_upgrade(|mut socket| async move {
                        while let Some(Ok(WsMessage::Text(text))) = socket.recv().await {
                            if socket.send(WsMessage::text(format!("kit {}", text.as_str()))).await.is_err() {
                                break;
                            }
                        }
                    })
                }),
            )
            .fallback(|request: Request| async move {
                let host = request.headers().get(header::HOST).cloned();
                (StatusCode::OK, format!("relay {} host={host:?}", request.uri().path_and_query().unwrap()))
            });
        let relay = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let relay_addr = relay.local_addr().unwrap();
        tokio::spawn(async move { axum::serve(relay, relay_app).await.unwrap() });

        let activity = Activity::new();
        let settings = Settings { targets: vec![("isaac".into(), relay_at(relay_addr))], owner: Some("jdoe".into()), ..Settings::default() };
        let app: Router = router(RtcService::new(settings, activity.clone()));
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
        let origin = format!("http://{front_addr}");
        let page = client.get(format!("{base}/rtc/isaac/assets/a.js?v=1")).header(header::ORIGIN, &origin).send().await.unwrap();
        assert_eq!(page.status(), StatusCode::OK);
        assert_eq!(page.text().await.unwrap(), format!("relay /assets/a.js?v=1 host=Some({:?})", relay_addr.to_string()));
        // The owner only; the page's own origin only; known targets only; the slash.
        let other = client.get(format!("{base}/rtc/isaac/")).header("x-user-alias", "alice").send().await.unwrap();
        assert_eq!(other.status(), StatusCode::FORBIDDEN);
        let owner = client.get(format!("{base}/rtc/isaac/")).header("x-user-alias", "jdoe").send().await.unwrap();
        assert_eq!(owner.status(), StatusCode::OK);
        let cross = client.get(format!("{base}/rtc/isaac/")).header(header::ORIGIN, "https://evil.example").send().await.unwrap();
        assert_eq!(cross.status(), StatusCode::FORBIDDEN);
        assert_eq!(client.get(format!("{base}/rtc/robot/")).send().await.unwrap().status(), StatusCode::NOT_FOUND);
        let slash = client.get(format!("{base}/rtc/isaac?x=1")).send().await.unwrap();
        assert_eq!(slash.headers()[header::LOCATION], "/workspace/jdoe/rtc/isaac/?x=1");

        // The signaling WebSocket, end to end; a foreign origin's refused.
        let ws_url = format!("ws://{front_addr}/workspace/jdoe/rtc/isaac/signal/sign_in?peer_id=peer-1&version=2&peer_role=1");
        let mut request = tokio_tungstenite::tungstenite::client::IntoClientRequest::into_client_request(ws_url).unwrap();
        let mut foreign = request.clone();
        foreign.headers_mut().insert(header::ORIGIN, HeaderValue::from_static("https://evil.example"));
        assert!(tokio_tungstenite::connect_async(foreign).await.is_err(), "a foreign origin's WebSocket");
        request.headers_mut().insert(header::ORIGIN, HeaderValue::from_str(&origin).unwrap());
        let (mut socket, _) = tokio_tungstenite::connect_async(request).await.unwrap();
        socket.send(Message::text("offer?")).await.unwrap();
        assert_eq!(socket.next().await.unwrap().unwrap().into_text().unwrap().as_str(), "kit offer?");
        assert_eq!(activity.open_connections(), 1);
        socket.close(None).await.ok();
    }

    #[tokio::test]
    async fn isaac_links_only_for_the_edge_verified_instance() {
        use axum::extract::ws::{Message as WsMessage, WebSocketUpgrade};
        use axum::routing::get;

        let bridge_app = Router::new().route(
            "/",
            get(|ws: WebSocketUpgrade| async move {
                ws.on_upgrade(|mut socket| async move {
                    while let Some(Ok(WsMessage::Binary(b))) = socket.recv().await {
                        if socket.send(WsMessage::Binary(b)).await.is_err() {
                            break;
                        }
                    }
                })
            }),
        );
        let bridge = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let bridge_addr = bridge.local_addr().unwrap();
        tokio::spawn(async move { axum::serve(bridge, bridge_app).await.unwrap() });
        let settings = Settings { links: vec![("i-0abc".into(), relay_at(bridge_addr))], owner: Some("jdoe".into()), ..Settings::default() };
        let app: Router = router(RtcService::new(settings, Activity::new()));
        let app = tower::Layer::layer(
            &axum::middleware::from_fn_with_state(Some(Arc::<str>::from("jdoe")), crate::workspace_prefix::middleware),
            app,
        );
        let app = Router::new().fallback_service(app);
        let front = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let front_addr = front.local_addr().unwrap();
        tokio::spawn(async move { axum::serve(front, app).await.unwrap() });

        let ws = |principal: Option<&'static str>, alias: Option<&'static str>, name: &str| {
            let url = format!("ws://{front_addr}/workspace/jdoe/isaac-link/{name}");
            let mut request = tokio_tungstenite::tungstenite::client::IntoClientRequest::into_client_request(url).unwrap();
            if let Some(p) = principal {
                request.headers_mut().insert("x-isaac-principal", HeaderValue::from_static(p));
            }
            if let Some(a) = alias {
                request.headers_mut().insert("x-user-alias", HeaderValue::from_static(a));
            }
            request
        };
        let (mut socket, _) = tokio_tungstenite::connect_async(ws(Some("i-0abc"), None, "i-0abc")).await.unwrap();
        socket.send(Message::binary(vec![1u8, 2, 3])).await.unwrap();
        assert_eq!(socket.next().await.unwrap().unwrap().into_data().as_ref(), &[1u8, 2, 3]);
        socket.close(None).await.ok();
        for (principal, alias, name) in [(None, None, "i-0abc"), (Some("i-0other"), None, "i-0abc"), (Some("i-0abc"), Some("jdoe"), "i-0abc"), (Some("i-0zzz"), None, "i-0zzz")] {
            assert!(tokio_tungstenite::connect_async(ws(principal, alias, name)).await.is_err(), "{principal:?} {alias:?} {name}");
        }
    }

    #[tokio::test]
    async fn lists_the_targets_for_the_viewer() {
        let settings = Settings { targets: vec![("isaac".into(), relay_at("127.0.0.1:8470".parse().unwrap()))], owner: Some("jdoe".into()), ..Settings::default() };
        let app: Router = router(RtcService::new(settings, Activity::new()));
        let list = |alias: Option<&'static str>| {
            let app = app.clone();
            async move {
                let mut request = Request::builder().uri(api::RTC_TARGETS);
                if let Some(alias) = alias {
                    request = request.header("x-user-alias", alias);
                }
                let response = tower::ServiceExt::oneshot(app, request.body(Body::empty()).unwrap()).await.unwrap();
                assert_eq!(response.status(), StatusCode::OK);
                let body = axum::body::to_bytes(response.into_body(), 1 << 16).await.unwrap();
                serde_json::from_slice::<factoryos_protocol::rtc::RtcTargets>(&body).unwrap()
            }
        };
        let owner = list(Some("jdoe")).await;
        assert_eq!((owner.targets, owner.may_watch), (vec!["isaac".to_owned()], true));
        assert!(!list(Some("alice")).await.may_watch);
        let none: Router = router(RtcService::new(Settings::default(), Activity::new()));
        let response = tower::ServiceExt::oneshot(none, Request::builder().uri(api::RTC_TARGETS).body(Body::empty()).unwrap()).await.unwrap();
        let body = axum::body::to_bytes(response.into_body(), 1 << 16).await.unwrap();
        assert!(serde_json::from_slice::<factoryos_protocol::rtc::RtcTargets>(&body).unwrap().targets.is_empty());
    }

    #[tokio::test]
    async fn off_without_targets() {
        let app: Router = router(RtcService::new(Settings::default(), Activity::new()));
        let response = tower::ServiceExt::oneshot(app, Request::builder().uri("/rtc/isaac/").body(Body::empty()).unwrap()).await.unwrap();
        assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);
    }
}
