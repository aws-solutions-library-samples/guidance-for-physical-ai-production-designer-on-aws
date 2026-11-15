use std::sync::Arc;
use std::time::Duration;

use futures_util::{SinkExt as _, StreamExt as _};
use tokio_tungstenite::tungstenite::Message;

use zenoh::Wait as _;
use zenoh::query::ConsolidationMode;

use crate::Callback;
use crate::protocol::keys;
use crate::rejoin::{Rejoins, boot_id_of};

const QUERY_TIMEOUT: Duration = Duration::from_secs(10);

/// A Zenoh client session to factoryos-backend's router.
#[derive(Clone)]
pub struct Bus {
    session: zenoh::Session,
    runtime: tokio::runtime::Handle,
    /// Puts go through one queue, so they arrive in the order they were made.
    puts: tokio::sync::mpsc::UnboundedSender<(String, Vec<u8>)>,
    /// The backend's liveliness token as seen, for [`Bus::on_rejoin`].
    rejoins: Rejoins,
    /// For a `ws(s)://` endpoint: the local bridge the session goes through.
    _bridge: Option<Arc<Bridge>>,
}

impl Bus {
    /// Connect to `endpoint`: a Zenoh locator (e.g. `tcp/127.0.0.1:7447`), or a backend's
    /// `ws(s)://…/zenoh-native` URL, reached through a local bridge as Zenoh's own `ws/<host>:<port>`
    /// can't carry a path. Must be called from within a tokio runtime.
    pub async fn connect(endpoint: &str) -> Result<Self, String> {
        let bridge = if endpoint.starts_with("ws://") || endpoint.starts_with("wss://") {
            Some(Arc::new(Bridge::start(endpoint).await?))
        } else {
            None
        };
        let locator = bridge.as_ref().map_or_else(|| endpoint.to_owned(), |bridge| bridge.locator.clone());
        let endpoint = locator.as_str();
        let mut config = zenoh::Config::default();
        for (key, value) in [
            ("mode", r#""client""#.to_owned()),
            ("connect/endpoints", format!(r#"["{endpoint}"]"#)),
            ("scouting/multicast/enabled", "false".to_owned()),
        ] {
            config.insert_json5(key, &value).map_err(|err| format!("zenoh config {key}: {err}"))?;
        }
        let session = zenoh::open(config)
            .await
            .map_err(|err| format!("failed to connect to {endpoint}: {err}"))?;
        let (puts, mut queue) = tokio::sync::mpsc::unbounded_channel::<(String, Vec<u8>)>();
        let publisher = session.clone();
        tokio::spawn(async move {
            while let Some((key, payload)) = queue.recv().await {
                publisher.put(&key, payload).await.ok();
            }
        });
        // The session re-joins on its own after a lost connection (the backend restarted, or
        // the network); the router then declares the backend's token to it again.
        let rejoins = Rejoins::default();
        let seen = rejoins.clone();
        session
            .liveliness()
            .declare_subscriber(keys::BACKEND_ALIVE)
            .history(true)
            .callback(move |sample| {
                let boot_id = boot_id_of(sample.key_expr().as_str());
                match sample.kind() {
                    zenoh::sample::SampleKind::Put => seen.appeared(boot_id),
                    zenoh::sample::SampleKind::Delete => seen.vanished(Some(boot_id)),
                }
            })
            .background()
            .await
            .map_err(|err| format!("liveliness {}: {err}", keys::BACKEND_ALIVE))?;
        Ok(Self {
            session,
            runtime: tokio::runtime::Handle::current(),
            puts,
            rejoins,
            _bridge: bridge,
        })
    }

    pub(crate) fn rejoins(&self) -> &Rejoins {
        &self.rejoins
    }

    /// Nothing to do natively: the Zenoh session reconnects on its own (and its queries time out
    /// after [`QUERY_TIMEOUT`]). For the web bus, drops and reopens its session.
    pub fn reconnect(&self) {}

    /// Call `on_sample` with the payload of every sample on `key` for the rest of the session.
    pub fn subscribe(&self, key: &str, on_sample: impl Fn(Vec<u8>) + Send + Sync + 'static) -> Result<(), String> {
        self.session
            .declare_subscriber(key)
            .callback(move |sample| on_sample(sample.payload().to_bytes().into_owned()))
            .background()
            .wait()
            .map_err(|err| format!("subscribe {key}: {err}"))
    }

    /// Query `key` and call `on_reply` with the first reply's payload (`Ok(None)` if nobody replied).
    pub fn query(&self, key: &str, payload: Option<Vec<u8>>, on_reply: impl FnOnce(Result<Option<Vec<u8>>, String>) + Send + 'static) {
        let on_reply: Callback<Option<Vec<u8>>> = Box::new(on_reply);
        let session = self.session.clone();
        let key = key.to_owned();
        self.runtime.spawn(async move {
            let mut get = session
                .get(&key)
                .consolidation(ConsolidationMode::None)
                .timeout(QUERY_TIMEOUT);
            if let Some(payload) = payload {
                get = get.payload(payload);
            }
            let result = match get.await {
                Ok(replies) => match replies.recv_async().await {
                    Ok(reply) => match reply.result() {
                        Ok(sample) => Ok(Some(sample.payload().to_bytes().into_owned())),
                        Err(err) => Err(format!("{key}: {}", err.payload().try_to_string().unwrap_or_default())),
                    },
                    Err(_) => Ok(None),
                },
                Err(err) => Err(format!("{key}: {err}")),
            };
            on_reply(result);
        });
    }

    pub fn put(&self, key: &str, payload: Vec<u8>) {
        self.puts.send((key.to_owned(), payload)).ok();
    }
}

/// A Zenoh `ws` listener on loopback whose connections are piped to a backend's
/// `ws(s)://…/zenoh-native`, so a Zenoh session can reach a backend under a path prefix.
struct Bridge {
    /// `ws/127.0.0.1:<port>`, for the session to connect to.
    locator: String,
    accept: tokio::task::JoinHandle<()>,
}

impl Bridge {
    async fn start(url: &str) -> Result<Self, String> {
        let listener = tokio::net::TcpListener::bind(("127.0.0.1", 0))
            .await
            .map_err(|err| format!("zenoh bridge: {err}"))?;
        let addr = listener.local_addr().map_err(|err| format!("zenoh bridge: {err}"))?;
        let url = url.to_owned();
        let accept = tokio::spawn(async move {
            while let Ok((stream, _)) = listener.accept().await {
                let url = url.clone();
                // The Zenoh session reports (and retries) a failed connection itself, but not why.
                tokio::spawn(async move {
                    if let Err(err) = pipe(stream, &url).await {
                        eprintln!("factoryos: zenoh bridge to {url}: {err}");
                    }
                });
            }
        });
        Ok(Self {
            locator: format!("ws/{addr}"),
            accept,
        })
    }
}

impl Drop for Bridge {
    fn drop(&mut self) {
        self.accept.abort();
    }
}

async fn pipe(stream: tokio::net::TcpStream, url: &str) -> Result<(), WsError> {
    let local = tokio_tungstenite::accept_async(stream).await?;
    let (remote, _) = tokio_tungstenite::connect_async(websocket_request(url)?).await?;
    let (mut local_tx, mut local_rx) = local.split();
    let (mut remote_tx, mut remote_rx) = remote.split();
    // Either side closing ends the session.
    tokio::select! {
        result = forward(&mut local_rx, &mut remote_tx) => result,
        result = forward(&mut remote_rx, &mut local_tx) => result,
    }
}

type WsError = tokio_tungstenite::tungstenite::Error;

/// A WebSocket handshake to `url`, with the session cookie of a backend behind the edge's login
/// (see [`crate::auth`]), current at the time of each (re)connection.
pub(crate) fn websocket_request(url: &str) -> Result<tokio_tungstenite::tungstenite::handshake::client::Request, WsError> {
    crate::install_crypto_provider();
    use tokio_tungstenite::tungstenite::client::IntoClientRequest as _;
    use tokio_tungstenite::tungstenite::http::HeaderValue;

    let mut request = url.into_client_request()?;
    if let Some(cookie) = crate::auth::cookie_header(url).and_then(|cookie| HeaderValue::from_str(&cookie).ok()) {
        request.headers_mut().insert("Cookie", cookie);
    }
    Ok(request)
}

async fn forward(
    rx: &mut (impl futures_util::Stream<Item = Result<Message, WsError>> + Unpin),
    tx: &mut (impl futures_util::Sink<Message, Error = WsError> + Unpin),
) -> Result<(), WsError> {
    while let Some(msg) = rx.next().await {
        match msg? {
            Message::Close(_) => break,
            Message::Frame(_) => {}
            msg => tx.send(msg).await?,
        }
    }
    tx.close().await.ok();
    Ok(())
}
