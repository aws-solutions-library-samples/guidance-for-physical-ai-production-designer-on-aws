//! Camera commands from outside ([`keys::VIEW_COMMAND`], published by the backend for the chat
//! agent's view_* tools): queued as they arrive (backend.rs), applied by the app once per frame to
//! the 3D view of the page they name, or of the page shown.
//!
//! [`keys::VIEW_COMMAND`]: factoryos_client::protocol::keys::VIEW_COMMAND

use std::sync::Mutex;

use factoryos_client::protocol::view::ViewCommand;

static QUEUE: Mutex<Vec<ViewCommand>> = Mutex::new(Vec::new());

/// A [`ViewCommand`] arrived.
pub fn received(payload: &[u8]) {
    match serde_json::from_slice::<ViewCommand>(payload) {
        Ok(command) => QUEUE.lock().unwrap().push(command),
        Err(err) => crate::rr::re_log::warn!("view command: {err}"),
    }
}

/// The commands that arrived since the last call, oldest first.
pub fn take() -> Vec<ViewCommand> {
    std::mem::take(&mut *QUEUE.lock().unwrap())
}
