//! The viewer's end of the planner chat: a WebSocket to the backend's `/chat`
//! (see `factoryos_protocol::chat`).
//!
//! The wasm viewer always uses it; the native one when its backend runs on another machine (with a
//! local backend, Rerun's own agent panel runs `kiro-cli` next to the viewer instead).
//!
//! Events are queued as they arrive and drained from the UI thread once per frame, the same way the
//! rest of this crate hands Zenoh data to the viewer.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use factoryos_protocol::chat::{ChatCommand, ChatEvent};

/// Whether the socket is usable, so the UI can say why it is not.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SocketState {
    Connecting,
    Open,
    Closed,
}

#[derive(Default)]
struct Inbox {
    events: VecDeque<ChatEvent>,
    state: Option<SocketState>,
    /// Messages sent before the socket opened. A browser WebSocket rejects `send` while it is
    /// still connecting, so they wait here and go out from the `open` handler. (Natively the send
    /// queue holds them, hence unused there.)
    #[cfg_attr(not(target_arch = "wasm32"), expect(dead_code))]
    outbox: Vec<String>,
    #[cfg_attr(not(target_arch = "wasm32"), expect(dead_code))]
    open: bool,
}

/// A connection to the backend's chat. Dropping it closes the socket.
pub struct ChatSocket {
    inbox: Arc<Mutex<Inbox>>,
    #[cfg(target_arch = "wasm32")]
    socket: web_sys::WebSocket,
    /// To the connection's task; dropping it closes the connection.
    #[cfg(not(target_arch = "wasm32"))]
    outgoing: tokio::sync::mpsc::UnboundedSender<String>,
    state: SocketState,
}

impl ChatSocket {
    /// `base` is the backend's HTTP base URL; the scheme is swapped for `ws`/`wss`.
    ///
    /// `wake` is called whenever something arrives, so the UI repaints.
    pub fn connect(base: &str, wake: impl Fn() + Send + Sync + 'static) -> Result<Self, String> {
        Self::connect_session(base, None, wake)
    }

    /// Like [`Self::connect`], to agent session `session` (`None`: the backend's default one).
    #[cfg(target_arch = "wasm32")]
    pub fn connect_session(base: &str, session: Option<&str>, wake: impl Fn() + Send + Sync + 'static) -> Result<Self, String> {
        use wasm_bindgen::JsCast as _;
        use wasm_bindgen::closure::Closure;

        let url = websocket_url(base, session);
        let socket = web_sys::WebSocket::new(&url).map_err(|err| format!("{url}: {err:?}"))?;
        let inbox = Arc::new(Mutex::new(Inbox::default()));
        let wake = Arc::new(wake);

        // Text frames only: the protocol is JSON.
        let on_message = {
            let inbox = inbox.clone();
            let wake = wake.clone();
            Closure::<dyn FnMut(web_sys::MessageEvent)>::new(move |event: web_sys::MessageEvent| {
                let Some(text) = event.data().as_string() else { return };
                inbox.lock().unwrap().events.push_back(parse_event(&text));
                wake();
            })
        };
        socket.set_onmessage(Some(on_message.as_ref().unchecked_ref()));
        on_message.forget();

        let on_open = {
            let inbox = inbox.clone();
            let wake = wake.clone();
            let socket = socket.clone();
            Closure::<dyn FnMut()>::new(move || {
                let queued = {
                    let mut inbox = inbox.lock().unwrap();
                    inbox.state = Some(SocketState::Open);
                    inbox.open = true;
                    std::mem::take(&mut inbox.outbox)
                };
                for text in queued {
                    if let Err(err) = socket.send_with_str(&text) {
                        inbox.lock().unwrap().events.push_back(ChatEvent::Error {
                            message: format!("could not send to the chat: {err:?}"),
                        });
                    }
                }
                wake();
            })
        };
        socket.set_onopen(Some(on_open.as_ref().unchecked_ref()));
        on_open.forget();

        let on_close = {
            let inbox = inbox.clone();
            let wake = wake.clone();
            Closure::<dyn FnMut()>::new(move || {
                let mut inbox = inbox.lock().unwrap();
                inbox.state = Some(SocketState::Closed);
                inbox.open = false;
                drop(inbox);
                wake();
            })
        };
        socket.set_onclose(Some(on_close.as_ref().unchecked_ref()));
        on_close.forget();

        Ok(Self {
            inbox,
            socket,
            state: SocketState::Connecting,
        })
    }

    /// Natively: a task on the current tokio runtime owns the connection.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn connect_session(base: &str, session: Option<&str>, wake: impl Fn() + Send + Sync + 'static) -> Result<Self, String> {
        use futures_util::{SinkExt as _, StreamExt as _};
        use tokio_tungstenite::tungstenite::Message;

        let url = websocket_url(base, session);
        let runtime = tokio::runtime::Handle::try_current().map_err(|err| format!("chat: {err}"))?;
        let inbox = Arc::new(Mutex::new(Inbox::default()));
        let (outgoing, mut queue) = tokio::sync::mpsc::unbounded_channel::<String>();
        let task_inbox = inbox.clone();
        runtime.spawn(async move {
            let inbox = task_inbox;
            let set_state = |state| {
                inbox.lock().unwrap().state = Some(state);
                wake();
            };
            let request = crate::bus_native::websocket_request(&url);
            let socket = match async { tokio_tungstenite::connect_async(request?).await }.await {
                Ok((socket, _)) => socket,
                Err(err) => {
                    inbox.lock().unwrap().events.push_back(ChatEvent::Error {
                        message: format!("could not connect to {url}: {err}"),
                    });
                    return set_state(SocketState::Closed);
                }
            };
            set_state(SocketState::Open);
            let (mut tx, mut rx) = socket.split();
            loop {
                tokio::select! {
                    message = rx.next() => match message {
                        Some(Ok(Message::Text(text))) => {
                            inbox.lock().unwrap().events.push_back(parse_event(&text));
                            wake();
                        }
                        Some(Ok(Message::Close(_)) | Err(_)) | None => break,
                        Some(Ok(_)) => {}
                    },
                    text = queue.recv() => match text {
                        Some(text) => {
                            if tx.send(Message::text(text)).await.is_err() {
                                break;
                            }
                        }
                        // The ChatSocket was dropped.
                        None => {
                            tx.close().await.ok();
                            break;
                        }
                    },
                }
            }
            set_state(SocketState::Closed);
        });
        Ok(Self {
            inbox,
            outgoing,
            state: SocketState::Connecting,
        })
    }

    pub fn state(&self) -> SocketState {
        self.state
    }

    /// Everything that arrived since the last call, newest last.
    pub fn drain(&mut self) -> Vec<ChatEvent> {
        let mut inbox = self.inbox.lock().unwrap();
        if let Some(state) = inbox.state.take() {
            self.state = state;
        }
        inbox.events.drain(..).collect()
    }

    pub fn send(&self, command: &ChatCommand) -> Result<(), String> {
        let text = serde_json::to_string(command).map_err(|err| err.to_string())?;
        self.send_text(&text)
    }

    #[cfg(target_arch = "wasm32")]
    fn send_text(&self, text: &str) -> Result<(), String> {
        let mut inbox = self.inbox.lock().unwrap();
        if !inbox.open {
            if self.state == SocketState::Closed {
                return Err("the chat connection is closed; reload the page".to_owned());
            }
            inbox.outbox.push(text.to_owned());
            return Ok(());
        }
        drop(inbox);
        self.socket.send_with_str(text).map_err(|err| format!("{err:?}"))
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn send_text(&self, text: &str) -> Result<(), String> {
        self.outgoing
            .send(text.to_owned())
            .map_err(|_| "the chat connection is closed; reopen the chat".to_owned())
    }
}

#[cfg(target_arch = "wasm32")]
impl Drop for ChatSocket {
    fn drop(&mut self) {
        self.socket.close().ok();
    }
}

fn parse_event(text: &str) -> ChatEvent {
    serde_json::from_str(text).unwrap_or_else(|err| ChatEvent::Error {
        message: format!("could not read a chat event: {err}"),
    })
}

/// `http://host/…` → `ws://host/…/chat[?session=<id>]`, `https://` → `wss://`.
fn websocket_url(base: &str, session: Option<&str>) -> String {
    use factoryos_protocol::api;
    match session {
        Some(id) => crate::websocket_url(base, &api::chat_ws(id)),
        None => crate::websocket_url(base, api::CHAT_WS),
    }
}
