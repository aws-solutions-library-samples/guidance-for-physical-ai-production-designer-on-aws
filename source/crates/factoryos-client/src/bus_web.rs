use js_sys::Uint8Array;
use wasm_bindgen::prelude::*;

use crate::protocol::keys;
use crate::rejoin::{Rejoins, boot_id_of};

#[wasm_bindgen(module = "/js/zenoh_bridge.js")]
extern "C" {
    #[wasm_bindgen(catch)]
    async fn zenoh_open(url: &str, on_lost: &Closure<dyn Fn()>) -> Result<JsValue, JsValue>;
    #[wasm_bindgen(catch)]
    async fn zenoh_liveliness_subscribe(session: &JsValue, key: &str, on_token: &Closure<dyn Fn(String, bool)>) -> Result<(), JsValue>;
    #[wasm_bindgen(catch)]
    async fn zenoh_subscribe(session: &JsValue, key: &str, on_sample: &Closure<dyn Fn(Uint8Array)>) -> Result<(), JsValue>;
    #[wasm_bindgen(catch)]
    async fn zenoh_get(session: &JsValue, key: &str, payload: Option<Uint8Array>) -> Result<JsValue, JsValue>;
    #[wasm_bindgen(catch)]
    async fn zenoh_put(session: &JsValue, key: &str, payload: Uint8Array) -> Result<(), JsValue>;
    fn zenoh_reconnect(session: &JsValue);
}

/// A zenoh-ts session over factoryos-backend's `/zenoh` WebSocket, opened again (subscribers and
/// all) when that closes; see `js/zenoh_bridge.js`.
#[derive(Clone)]
pub struct Bus {
    session: JsValue,
    /// The backend's liveliness token as seen, for [`Bus::on_rejoin`].
    rejoins: Rejoins,
}

impl Bus {
    /// Connect to `url`, e.g. `wss://host/zenoh` (see [`crate::zenoh_endpoint`]).
    pub async fn connect(url: &str) -> Result<Self, String> {
        let rejoins = Rejoins::default();
        let lost = rejoins.clone();
        // Both live as long as the page.
        let on_lost = Closure::<dyn Fn()>::new(move || lost.vanished(None));
        let session = zenoh_open(url, &on_lost).await.map_err(js_error)?;
        on_lost.forget();
        let seen = rejoins.clone();
        let on_token = Closure::<dyn Fn(String, bool)>::new(move |key: String, alive: bool| {
            if alive {
                seen.appeared(boot_id_of(&key));
            } else {
                seen.vanished(Some(boot_id_of(&key)));
            }
        });
        zenoh_liveliness_subscribe(&session, keys::BACKEND_ALIVE, &on_token)
            .await
            .map_err(|err| format!("liveliness {}: {}", keys::BACKEND_ALIVE, js_error(err)))?;
        on_token.forget();
        Ok(Self { session, rejoins })
    }

    pub(crate) fn rejoins(&self) -> &Rejoins {
        &self.rejoins
    }

    /// Drop the session and open a new one: the page knows the backend restarted (its health
    /// check says so) and this session has not re-joined it (a WebSocket left half-open).
    pub fn reconnect(&self) {
        zenoh_reconnect(&self.session);
    }

    /// Call `on_sample` with the payload of every sample on `key` for the rest of the session.
    pub fn subscribe(&self, key: &str, on_sample: impl Fn(Vec<u8>) + Send + Sync + 'static) -> Result<(), String> {
        let session = self.session.clone();
        let key = key.to_owned();
        wasm_bindgen_futures::spawn_local(async move {
            let closure = Closure::<dyn Fn(Uint8Array)>::new(move |bytes: Uint8Array| on_sample(bytes.to_vec()));
            match zenoh_subscribe(&session, &key, &closure).await {
                Ok(()) => closure.forget(), // lives as long as the session
                Err(err) => log_error(&format!("subscribe {key}: {}", js_error(err))),
            }
        });
        Ok(())
    }

    /// Query `key` and call `on_reply` with the first reply's payload (`Ok(None)` if nobody replied).
    pub fn query(&self, key: &str, payload: Option<Vec<u8>>, on_reply: impl FnOnce(Result<Option<Vec<u8>>, String>) + Send + 'static) {
        let session = self.session.clone();
        let key = key.to_owned();
        wasm_bindgen_futures::spawn_local(async move {
            let payload = payload.map(|bytes| Uint8Array::from(bytes.as_slice()));
            let result = match zenoh_get(&session, &key, payload).await {
                Ok(value) if value.is_null() => Ok(None),
                Ok(value) => Ok(Some(Uint8Array::new(&value).to_vec())),
                Err(err) => Err(js_error(err)),
            };
            on_reply(result);
        });
    }

    pub fn put(&self, key: &str, payload: Vec<u8>) {
        let session = self.session.clone();
        let key = key.to_owned();
        wasm_bindgen_futures::spawn_local(async move {
            if let Err(err) = zenoh_put(&session, &key, Uint8Array::from(payload.as_slice())).await {
                log_error(&format!("put {key}: {}", js_error(err)));
            }
        });
    }
}

fn js_error(err: JsValue) -> String {
    err.as_string()
        .or_else(|| js_sys::Error::from(err.clone()).message().as_string())
        .unwrap_or_else(|| format!("{err:?}"))
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console, js_name = error)]
    fn log_error(message: &str);
}
