//! The planner chat's HTTP side ([`crate::services::chat`]): `GET /chat` upgrades to a WebSocket
//! attached to one agent session, and `/api/v1/chat/sessions` lists, creates and closes sessions
//! (the viewer's chat tabs).
//!
//! `/chat?session=<id>` attaches to that session; `/chat` alone to the default one, the single chat
//! of older viewers. Every connection first receives the session as it stands (state, transcript,
//! any permission request still waiting), then the live events. Commands from any viewer go to the
//! session's agent process, so two open browser tabs see the same conversation rather than one each.

use std::collections::HashMap;

use axum::Json;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::{Path, Query, State};
use axum::http::StatusCode;
use axum::response::Response;
use axum::routing::{delete, get};
use factoryos_protocol::api;
use factoryos_protocol::chat::{ChatCommand, ChatEvent, ChatSessionInfo, ChatSessions, NewChatSession};
use futures_util::{SinkExt as _, StreamExt as _};
use tokio::sync::broadcast::error::RecvError;

use crate::AppState;

pub fn router() -> axum::Router<AppState> {
    axum::Router::new()
        .route(api::CHAT_WS, get(handler))
        .route(api::CHAT_SESSIONS, get(list_sessions).post(create_session))
        .route(api::CHAT_SESSION, delete(close_session))
}

async fn list_sessions(State(state): State<AppState>) -> Json<ChatSessions> {
    Json(state.chat.list())
}

async fn create_session(
    State(state): State<AppState>,
    Json(request): Json<NewChatSession>,
) -> Result<Json<ChatSessionInfo>, (StatusCode, String)> {
    // Starting the agent spawns a thread and a process: not on the async runtime's workers.
    let chat = state.chat.clone();
    let session = tokio::task::spawn_blocking(move || chat.create(&request.title, request.prompt))
        .await
        .map_err(|err| (StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?
        .map_err(|err| (StatusCode::CONFLICT, err))?;
    tracing::info!("chat session {} opened", session.id);
    Ok(Json(session.info()))
}

async fn close_session(State(state): State<AppState>, Path(id): Path<String>) -> StatusCode {
    if state.chat.close(&id) {
        tracing::info!("chat session {id} closed");
        StatusCode::NO_CONTENT
    } else {
        StatusCode::NOT_FOUND
    }
}

pub async fn handler(
    ws: WebSocketUpgrade,
    State(state): State<AppState>,
    Query(query): Query<HashMap<String, String>>,
) -> Response {
    let session = query.get("session").cloned();
    ws.on_upgrade(move |socket| async move {
        if let Err(err) = serve(socket, state, session).await {
            tracing::debug!("chat websocket closed: {err:#}");
        }
    })
}

async fn serve(socket: WebSocket, state: AppState, session: Option<String>) -> anyhow::Result<()> {
    let _open = state.activity.connection();
    let (mut tx, mut rx) = socket.split();
    let session = match session {
        Some(id) => state.chat.get(&id).ok_or_else(|| ChatEvent::SessionClosed),
        None => state.chat.default_session().map_err(|message| ChatEvent::Error { message }),
    };
    let session = match session {
        Ok(session) => session,
        Err(event) => {
            tx.send(Message::text(serde_json::to_string(&event)?)).await?;
            tx.close().await.ok();
            return Ok(());
        }
    };
    // Subscribe BEFORE the snapshot, so an event between the two is delivered rather than lost.
    let mut events = session.subscribe();
    for event in session.snapshot() {
        tx.send(Message::text(serde_json::to_string(&event)?)).await?;
    }

    let to_client = async {
        // Pings keep an idle chat open through the edge (the browser answers them by itself).
        let mut keepalive = tokio::time::interval_at(tokio::time::Instant::now() + crate::WS_KEEPALIVE, crate::WS_KEEPALIVE);
        loop {
            let event = tokio::select! {
                event = events.recv() => event,
                _ = keepalive.tick() => {
                    tx.send(Message::Ping(Vec::new().into())).await?;
                    continue;
                }
            };
            match event {
                Ok(event) => {
                    let closed = matches!(event, ChatEvent::SessionClosed);
                    tx.send(Message::text(serde_json::to_string(&event)?)).await?;
                    if closed {
                        break;
                    }
                }
                // A viewer that cannot keep up would need the transcript again to be consistent;
                // closing is simpler and the page reconnects with a fresh snapshot.
                Err(RecvError::Lagged(skipped)) => {
                    anyhow::bail!("chat client fell {skipped} events behind");
                }
                Err(RecvError::Closed) => break,
            }
        }
        tx.close().await.ok();
        anyhow::Ok(())
    };

    let from_client = async {
        while let Some(message) = rx.next().await {
            match message? {
                Message::Text(text) => match serde_json::from_str::<ChatCommand>(&text) {
                    Ok(command) => {
                        let session = session.clone();
                        // A prompt may spawn the agent: off the async workers.
                        tokio::task::spawn_blocking(move || session.handle(command)).await.ok();
                    }
                    Err(err) => tracing::warn!("ignoring a malformed chat command: {err}"),
                },
                Message::Close(_) => break,
                _ => {}
            }
        }
        anyhow::Ok(())
    };

    tokio::select! {
        result = to_client => result,
        result = from_client => result,
    }
}
