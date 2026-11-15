//! The embedded Zenoh router, plus the remote-api plugin that bridges zenoh-ts (wasm viewers)
//! over WebSocket. The plugin listens on loopback only; browsers reach it through `/zenoh`
//! on the public HTTP(S) port (see [`crate::ws_proxy`]).

use std::net::SocketAddr;

use anyhow::Context as _;
use zenoh::Session;
use zenoh::bytes::Encoding;
use zenoh::internal::plugins::PluginsManager;
use zenoh::internal::runtime::RuntimeBuilder;
use zenoh_plugin_remote_api::RemoteApiPlugin;

const REMOTE_API: &str = "remote_api";

pub struct BusOptions {
    /// Zenoh endpoints for native peers, e.g. `tcp/0.0.0.0:7447`. May be empty.
    pub listen: Vec<String>,
    /// Loopback address of the remote-api plugin's WebSocket server.
    pub remote_api: SocketAddr,
}

/// Handle to the backend's Zenoh session.
///
/// Publishing is synchronous and ordered: every sample goes through one queue drained by one
/// task, so e.g. `starting` is never overtaken by `ready`, and scene deltas arrive in order.
/// Callable from async code and plain threads alike.
#[derive(Clone)]
pub struct Bus {
    session: Session,
    queue: tokio::sync::mpsc::UnboundedSender<Sample>,
}

struct Sample {
    key: String,
    payload: Vec<u8>,
    encoding: Encoding,
}

impl Bus {
    pub async fn start(options: &BusOptions) -> anyhow::Result<Self> {
        let mut config = zenoh::Config::default();
        let insert = |config: &mut zenoh::Config, key: &str, value: String| {
            config
                .insert_json5(key, &value)
                .map_err(|err| anyhow::anyhow!("zenoh config {key}: {err}"))
        };
        insert(&mut config, "mode", r#""router""#.to_owned())?;
        insert(&mut config, "listen/endpoints", serde_json::to_string(&options.listen)?)?;
        // A viewer backend, not a plant router: don't go looking for peers on the LAN.
        insert(&mut config, "scouting/multicast/enabled", "false".to_owned())?;
        insert(&mut config, "plugins_loading/enabled", "true".to_owned())?;
        insert(
            &mut config,
            &format!("plugins/{REMOTE_API}/websocket_port"),
            format!(r#""{}""#, options.remote_api),
        )?;

        let mut plugins = PluginsManager::static_plugins_only();
        plugins.declare_static_plugin::<RemoteApiPlugin, &str>(REMOTE_API, true);

        let mut runtime = RuntimeBuilder::new(config)
            .plugins_manager(plugins)
            .build()
            .await
            .map_err(|err| anyhow::anyhow!("zenoh runtime: {err}"))?;
        let session = zenoh::session::init(runtime.clone().into())
            .await
            .map_err(|err| anyhow::anyhow!("zenoh session: {err}"))?;
        runtime
            .start()
            .await
            .map_err(|err| anyhow::anyhow!("zenoh start: {err}"))
            .context("is the Zenoh listen port already in use?")?;

        let (queue, mut samples) = tokio::sync::mpsc::unbounded_channel::<Sample>();
        let publisher = session.clone();
        tokio::spawn(async move {
            while let Some(Sample { key, payload, encoding }) = samples.recv().await {
                if let Err(err) = publisher.put(&key, payload).encoding(encoding).await {
                    tracing::warn!("{key}: zenoh put failed: {err}");
                }
            }
        });
        Ok(Self { session, queue })
    }

    pub fn session(&self) -> &Session {
        &self.session
    }

    pub fn put_json(&self, key: &str, value: &impl serde::Serialize) {
        match serde_json::to_vec(value) {
            Ok(json) => self.put(key, json, Encoding::APPLICATION_JSON),
            Err(err) => tracing::warn!("{key}: failed to serialize: {err}"),
        }
    }

    pub fn put_bytes(&self, key: &str, bytes: Vec<u8>) {
        self.put(key, bytes, Encoding::APPLICATION_OCTET_STREAM);
    }

    pub fn log(&self, line: impl Into<String>) {
        let line = line.into();
        tracing::info!("{line}");
        self.put(factoryos_protocol::keys::LOG, line.into_bytes(), Encoding::TEXT_PLAIN);
    }

    fn put(&self, key: &str, payload: Vec<u8>, encoding: Encoding) {
        let sample = Sample {
            key: key.to_owned(),
            payload,
            encoding,
        };
        if self.queue.send(sample).is_err() {
            tracing::warn!("{key}: zenoh publisher stopped");
        }
    }
}

/// Reply to `query` with `value` as JSON.
pub async fn reply_json(query: &zenoh::query::Query, value: &impl serde::Serialize) {
    let Ok(json) = serde_json::to_vec(value) else { return };
    if let Err(err) = query
        .reply(query.key_expr().clone(), json)
        .encoding(Encoding::APPLICATION_JSON)
        .await
    {
        tracing::warn!("{}: reply failed: {err}", query.key_expr());
    }
}

/// Answer every query on `key` with the JSON `current()` returns (nothing if `None`).
pub fn serve_latest<T: serde::Serialize + Send + Sync + 'static>(
    bus: &Bus,
    key: &'static str,
    current: impl Fn() -> Option<T> + Send + Sync + 'static,
) {
    let session = bus.session.clone();
    tokio::spawn(async move {
        let queryable = match session.declare_queryable(key).await {
            Ok(queryable) => queryable,
            Err(err) => {
                tracing::error!("{key}: {err}");
                return;
            }
        };
        while let Ok(query) = queryable.recv_async().await {
            if let Some(value) = current() {
                reply_json(&query, &value).await;
            }
        }
    });
}

/// This process's boot id: the start time (Unix milliseconds) and the pid, in hex
/// (`<millis>-<pid>`, e.g. `19a1b2c3d4e-1f2a`). Another id on [`factoryos_protocol::keys::BACKEND_BOOT`]
/// means the backend restarted.
pub fn boot_id() -> String {
    let millis = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_millis()).unwrap_or_default();
    format!("{millis:x}-{:x}", std::process::id())
}

/// Publish `boot_id` on [`factoryos_protocol::keys::BACKEND_BOOT`] once, and answer queries there with it.
pub fn announce_boot(bus: &Bus, boot_id: String) {
    bus.put_json(factoryos_protocol::keys::BACKEND_BOOT, &boot_id);
    serve_latest(bus, factoryos_protocol::keys::BACKEND_BOOT, move || Some(boot_id.clone()));
}

/// Declare this backend's liveliness token ([`factoryos_protocol::keys::backend_alive`]), for as
/// long as the returned token lives. Call it once the queryables are declared and the backend
/// serves: a client that sees it (re)appear queries what it follows right away.
pub async fn declare_alive(bus: &Bus, boot_id: &str) -> anyhow::Result<zenoh::liveliness::LivelinessToken> {
    let key = factoryos_protocol::keys::backend_alive(boot_id);
    bus.session
        .liveliness()
        .declare_token(&key)
        .await
        .map_err(|err| anyhow::anyhow!("{key}: {err}"))
}

#[cfg(test)]
mod tests {
    #[test]
    fn boot_ids_are_hex_millis_and_pid() {
        let id = super::boot_id();
        let (millis, pid) = id.split_once('-').expect("<millis>-<pid>");
        assert!(u64::from_str_radix(millis, 16).unwrap() > 1_700_000_000_000, "{id}");
        assert_eq!(u32::from_str_radix(pid, 16).unwrap(), std::process::id());
    }
}
