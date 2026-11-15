//! Web entry point. `web/index.html` loads the wasm module and calls [`start`]; the viewer then
//! talks to the factoryos-backend on the page's origin (HTTP `/api/v1/…`, Zenoh over `/zenoh`),
//! under the page's workspace prefix (`/workspace/<name>/`) if it has one. Behind the edge the page
//! is just `/`: the edge sends requests to the workspace the `fos_ws` cookie selects (the loader in
//! `index.html` waits until that workspace is up before calling [`start`]).

use wasm_bindgen::JsCast as _;
use wasm_bindgen::prelude::*;

use crate::rr::{eframe, re_log, re_viewer};
use crate::{Backend, Startup, new_app};

/// Connect to the backend that served the page and run the viewer in the canvas `canvas_id`.
#[wasm_bindgen]
pub async fn start(canvas_id: String) -> Result<(), JsValue> {
    re_log::setup_logging();
    let main_thread_token = re_viewer::MainThreadToken::i_promise_i_am_on_the_main_thread();

    let window = web_sys::window().ok_or("no window; not running in a browser?")?;
    let location = window.location();
    let path = location.pathname()?;
    let base = format!("{}{}", location.origin()?, factoryos_client::protocol::workspace::base_path(&path));
    let canvas = window
        .document()
        .ok_or("no document")?
        .get_element_by_id(&canvas_id)
        .ok_or_else(|| format!("canvas '{canvas_id}' not found"))?
        .dyn_into::<web_sys::HtmlCanvasElement>()
        .map_err(|_| format!("'{canvas_id}' is not a canvas"))?;

    let backend = Backend::connect(base).await.map_err(|err| JsValue::from_str(&format!("{err:#}")))?;

    let web_options = eframe::WebOptions {
        wgpu_options: re_viewer::wgpu_options(None),
        depth_buffer: 0,
        dithering: true,
        ..Default::default()
    };
    eframe::WebRunner::new()
        .start(
            canvas,
            web_options,
            Box::new(move |cc| {
                new_app(
                    cc,
                    Startup {
                        backend,
                        log_receivers: Vec::new(),
                        main_thread_token,
                    },
                )
            }),
        )
        .await
}

/// One automation command (a JSON line, see `automation`), with `?automation=1` in the page's
/// URL; the reply goes to `globalThis.__factoryosAutomationReply`. `web/index.html` wraps it.
#[wasm_bindgen]
pub fn automation(line: String) {
    crate::automation::submit(&line);
}

/// Select workspace `name` (the edge's cookie) and reload: the loader waits for it to come up.
pub fn select_workspace(name: &str) {
    use factoryos_client::protocol::workspace::{SELECT_COOKIE, is_valid_name};
    if !is_valid_name(name) {
        return;
    }
    if let Some(document) = web_sys::window()
        .and_then(|window| window.document())
        .and_then(|document| document.dyn_into::<web_sys::HtmlDocument>().ok())
    {
        let cookie = format!("{SELECT_COOKIE}={name}; Path=/; Max-Age=31536000; Secure; SameSite=Lax");
        if let Err(err) = document.set_cookie(&cookie) {
            re_log::warn!("Could not select workspace {name}: {err:?}");
            return;
        }
    }
    reload();
}

/// Reload the page (back to the loader, which waits for the workspace).
pub fn reload() {
    if let Some(window) = web_sys::window() {
        window.location().reload().ok();
    }
}

/// Open `url` in a browser window of its own named `name` (a popup the size of a smartPAD, its page 1120 × 660; an
/// open one of that name gets the URL and comes to the front).
pub fn open_popup(url: &str, name: &str) {
    if let Some(window) = web_sys::window() {
        match window.open_with_url_and_target_and_features(url, name, "popup=yes,width=1120,height=660") {
            Ok(Some(opened)) => {
                opened.focus().ok();
            }
            _ => re_log::warn!("The browser blocked the window for {}; allow pop-ups for this site", crate::workspace_ui::without_query(url)),
        }
    }
}

/// Open `url` in a new browser window (may be blocked as a popup; the UI shows a link as well).
pub fn open_window(url: &str) {
    if let Some(window) = web_sys::window() {
        if window.open_with_url_and_target(url, "_blank").ok().flatten().is_none() {
            re_log::warn!("The browser blocked the window for {}; use the link instead", crate::workspace_ui::without_query(url));
        }
    }
}

/// The page around the canvas in the theme's background, also for the loader of the next visit
/// (index.html reads it before the viewer starts).
pub fn set_page_background(color: crate::rr::egui::Color32) {
    let css = format!("#{:02x}{:02x}{:02x}", color.r(), color.g(), color.b());
    let Some(window) = web_sys::window() else { return };
    if let Some(body) = window.document().and_then(|document| document.body()) {
        body.style().set_property("background", &css).ok();
    }
    if let Ok(Some(storage)) = window.local_storage() {
        storage.set_item("factoryos.page-bg", &css).ok();
    }
}
