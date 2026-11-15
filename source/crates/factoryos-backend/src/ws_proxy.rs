//! Zenoh over the public HTTP port, so viewers need no other port and share its TLS:
//!
//! * `GET /zenoh`: zenoh-ts (wasm viewers), piped to the remote-api plugin on loopback.
//! * `GET /zenoh-native`: native viewers' Zenoh sessions (through their local bridge, as Zenoh's
//!   `ws/<host>:<port>` can't carry a path), piped to the router's own `ws` listener on loopback.

use std::net::SocketAddr;
use std::sync::Arc;

use axum::extract::State;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::http::StatusCode;
use axum::response::{IntoResponse as _, Response};
use futures_util::{SinkExt as _, StreamExt as _};
use tokio_tungstenite::tungstenite::Message as UpstreamMessage;

use crate::AppState;
use crate::activity::Activity;

/// zenoh-ts can send large payloads (e.g. scene docs); don't cap them below Zenoh's own limits.
const MAX_MESSAGE_SIZE: usize = 256 * 1024 * 1024;

pub async fn handler(ws: WebSocketUpgrade, State(state): State<AppState>) -> Response {
    proxy(ws, state.remote_api, state.activity)
}

pub async fn native_handler(ws: WebSocketUpgrade, State(state): State<AppState>) -> Response {
    match state.zenoh_ws {
        Some(upstream) => proxy(ws, upstream, state.activity),
        None => (StatusCode::NOT_FOUND, "Zenoh over the HTTP port is disabled (--zenoh-ws-port 0)").into_response(),
    }
}

fn proxy(ws: WebSocketUpgrade, upstream: SocketAddr, activity: Arc<Activity>) -> Response {
    ws.max_message_size(MAX_MESSAGE_SIZE)
        .max_frame_size(MAX_MESSAGE_SIZE)
        .on_upgrade(move |socket| async move {
            let _open = activity.connection();
            if let Err(err) = pipe(socket, upstream).await {
                tracing::debug!("zenoh websocket closed: {err:#}");
            }
        })
}

async fn pipe(client: WebSocket, upstream: SocketAddr) -> anyhow::Result<()> {
    // The upstream side needs the same limits: tungstenite's defaults (16 MiB frames, 64 MiB
    // messages) drop the session on e.g. a Robo Fitter snapshot with a large fixture in it.
    let config = tokio_tungstenite::tungstenite::protocol::WebSocketConfig::default()
        .max_message_size(Some(MAX_MESSAGE_SIZE))
        .max_frame_size(Some(MAX_MESSAGE_SIZE));
    let (upstream, _) =
        tokio_tungstenite::connect_async_with_config(format!("ws://{upstream}"), Some(config), false).await?;
    let (mut up_tx, mut up_rx) = upstream.split();
    let (mut client_tx, mut client_rx) = client.split();

    let to_upstream = async {
        while let Some(msg) = client_rx.next().await {
            let msg = match msg? {
                Message::Binary(data) => UpstreamMessage::Binary(data),
                Message::Text(text) => UpstreamMessage::text(text.as_str()),
                Message::Ping(data) => UpstreamMessage::Ping(data),
                Message::Pong(data) => UpstreamMessage::Pong(data),
                Message::Close(_) => break,
            };
            up_tx.send(msg).await?;
        }
        up_tx.close().await.ok();
        anyhow::Ok(())
    };
    let to_client = async {
        while let Some(msg) = up_rx.next().await {
            let msg = match msg? {
                UpstreamMessage::Binary(data) => Message::Binary(data),
                UpstreamMessage::Text(text) => Message::text(text.as_str()),
                UpstreamMessage::Ping(data) => Message::Ping(data),
                UpstreamMessage::Pong(data) => Message::Pong(data),
                UpstreamMessage::Close(_) => break,
                UpstreamMessage::Frame(_) => continue,
            };
            client_tx.send(msg).await?;
        }
        client_tx.close().await.ok();
        anyhow::Ok(())
    };

    // Either side closing ends the session.
    tokio::select! {
        result = to_upstream => result,
        result = to_client => result,
    }
}
