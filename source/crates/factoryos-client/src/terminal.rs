//! The viewer's end of an IDE terminal: a WebSocket to the backend's
//! `/api/v1/terminals/<id>/ws` (see `factoryos_protocol::terminal`), binary for the bytes, JSON text
//! for the rest. Events are queued as they arrive and drained from the UI thread once per frame,
//! like [`crate::chat::ChatSocket`]'s.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use factoryos_protocol::terminal::{TerminalControl, TerminalNotice};

pub use crate::chat::SocketState;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TermEvent {
    /// The shell's output (the history first, on each new socket).
    Output(Vec<u8>),
    Notice(TerminalNotice),
}

enum Outgoing {
    Bytes(Vec<u8>),
    Text(String),
}

#[derive(Default)]
struct Inbox {
    events: VecDeque<TermEvent>,
    state: Option<SocketState>,
    /// Sent before the socket opened (a browser WebSocket refuses them until then).
    #[cfg_attr(not(target_arch = "wasm32"), expect(dead_code))]
    outbox: Vec<Outgoing>,
    #[cfg_attr(not(target_arch = "wasm32"), expect(dead_code))]
    open: bool,
}

/// A connection to one terminal session. Dropping it closes the socket (not the shell).
pub struct TerminalSocket {
    inbox: Arc<Mutex<Inbox>>,
    #[cfg(target_arch = "wasm32")]
    socket: web_sys::WebSocket,
    #[cfg(not(target_arch = "wasm32"))]
    outgoing: tokio::sync::mpsc::UnboundedSender<Outgoing>,
    state: SocketState,
}

fn parse_notice(text: &str) -> Option<TermEvent> {
    serde_json::from_str(text).ok().map(TermEvent::Notice)
}

impl TerminalSocket {
    /// `base` is the backend's HTTP base URL; `wake` is called whenever something arrives.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn connect(base: &str, id: &str, wake: impl Fn() + Send + Sync + 'static) -> Result<Self, String> {
        use futures_util::{SinkExt as _, StreamExt as _};
        use tokio_tungstenite::tungstenite::Message;

        let url = crate::websocket_url(base, &factoryos_protocol::api::terminal_ws(id));
        let runtime = tokio::runtime::Handle::try_current().map_err(|err| format!("terminal: {err}"))?;
        let inbox = Arc::new(Mutex::new(Inbox::default()));
        let (outgoing, mut queue) = tokio::sync::mpsc::unbounded_channel::<Outgoing>();
        let task_inbox = inbox.clone();
        runtime.spawn(async move {
            let inbox = task_inbox;
            let set_state = |state| {
                inbox.lock().unwrap().state = Some(state);
                wake();
            };
            let request = crate::bus_native::websocket_request(&url);
            let Ok((socket, _)) = async { tokio_tungstenite::connect_async(request?).await }.await else {
                return set_state(SocketState::Closed);
            };
            set_state(SocketState::Open);
            let (mut tx, mut rx) = socket.split();
            loop {
                tokio::select! {
                    message = rx.next() => {
                        let event = match message {
                            Some(Ok(Message::Binary(bytes))) => Some(TermEvent::Output(bytes.to_vec())),
                            Some(Ok(Message::Text(text))) => parse_notice(&text),
                            Some(Ok(Message::Close(_)) | Err(_)) | None => break,
                            Some(Ok(_)) => None,
                        };
                        if let Some(event) = event {
                            inbox.lock().unwrap().events.push_back(event);
                            wake();
                        }
                    }
                    out = queue.recv() => {
                        let message = match out {
                            Some(Outgoing::Bytes(bytes)) => Message::binary(bytes),
                            Some(Outgoing::Text(text)) => Message::text(text),
                            None => {
                                tx.close().await.ok();
                                break;
                            }
                        };
                        if tx.send(message).await.is_err() {
                            break;
                        }
                    }
                }
            }
            set_state(SocketState::Closed);
        });
        Ok(Self { inbox, outgoing, state: SocketState::Connecting })
    }

    #[cfg(target_arch = "wasm32")]
    pub fn connect(base: &str, id: &str, wake: impl Fn() + Send + Sync + 'static) -> Result<Self, String> {
        use wasm_bindgen::JsCast as _;
        use wasm_bindgen::closure::Closure;

        let url = crate::websocket_url(base, &factoryos_protocol::api::terminal_ws(id));
        let socket = web_sys::WebSocket::new(&url).map_err(|err| format!("{url}: {err:?}"))?;
        socket.set_binary_type(web_sys::BinaryType::Arraybuffer);
        let inbox = Arc::new(Mutex::new(Inbox::default()));
        let wake = Arc::new(wake);

        let on_message = {
            let (inbox, wake) = (inbox.clone(), wake.clone());
            Closure::<dyn FnMut(web_sys::MessageEvent)>::new(move |event: web_sys::MessageEvent| {
                let data = event.data();
                let event = if let Some(text) = data.as_string() {
                    parse_notice(&text)
                } else if let Ok(buffer) = data.dyn_into::<js_sys::ArrayBuffer>() {
                    Some(TermEvent::Output(js_sys::Uint8Array::new(&buffer).to_vec()))
                } else {
                    None
                };
                if let Some(event) = event {
                    inbox.lock().unwrap().events.push_back(event);
                    wake();
                }
            })
        };
        socket.set_onmessage(Some(on_message.as_ref().unchecked_ref()));
        on_message.forget();

        let on_open = {
            let (inbox, wake, socket) = (inbox.clone(), wake.clone(), socket.clone());
            Closure::<dyn FnMut()>::new(move || {
                let queued = {
                    let mut inbox = inbox.lock().unwrap();
                    inbox.state = Some(SocketState::Open);
                    inbox.open = true;
                    std::mem::take(&mut inbox.outbox)
                };
                for out in queued {
                    let _ = match out {
                        Outgoing::Bytes(bytes) => socket.send_with_u8_array(&bytes),
                        Outgoing::Text(text) => socket.send_with_str(&text),
                    };
                }
                wake();
            })
        };
        socket.set_onopen(Some(on_open.as_ref().unchecked_ref()));
        on_open.forget();

        let on_close = {
            let (inbox, wake) = (inbox.clone(), wake.clone());
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

        Ok(Self { inbox, socket, state: SocketState::Connecting })
    }

    pub fn state(&self) -> SocketState {
        self.state
    }

    /// Everything that arrived since the last call, in order.
    pub fn drain(&mut self) -> Vec<TermEvent> {
        let mut inbox = self.inbox.lock().unwrap();
        if let Some(state) = inbox.state.take() {
            self.state = state;
        }
        inbox.events.drain(..).collect()
    }

    /// Keystrokes (or a paste) for the shell.
    pub fn send_bytes(&self, bytes: Vec<u8>) {
        self.send(Outgoing::Bytes(bytes));
    }

    pub fn resize(&self, cols: u16, rows: u16) {
        if let Ok(text) = serde_json::to_string(&TerminalControl::Resize { cols, rows }) {
            self.send(Outgoing::Text(text));
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn send(&self, out: Outgoing) {
        let _ = self.outgoing.send(out);
    }

    #[cfg(target_arch = "wasm32")]
    fn send(&self, out: Outgoing) {
        let mut inbox = self.inbox.lock().unwrap();
        if !inbox.open {
            if self.state != SocketState::Closed {
                inbox.outbox.push(out);
            }
            return;
        }
        drop(inbox);
        let _ = match out {
            Outgoing::Bytes(bytes) => self.socket.send_with_u8_array(&bytes),
            Outgoing::Text(text) => self.socket.send_with_str(&text),
        };
    }
}

#[cfg(target_arch = "wasm32")]
impl Drop for TerminalSocket {
    fn drop(&mut self) {
        self.socket.close().ok();
    }
}
