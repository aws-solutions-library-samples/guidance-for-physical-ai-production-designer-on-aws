//! Physical AI Production Designer, native: a backend chooser, then the viewer window, and a local factoryos-backend if
//! that was chosen and none is running (with `FACTORY_OS_GRPC_BIND`, also a gRPC server for Rerun's
//! logging SDKs).

use rerun::external::{eframe, re_crash_handler, re_grpc_server, re_log, re_memory, re_viewer, tokio};

// By using `re_memory::AccountingAllocator` Rerun can keep track of exactly how much memory it is using,
// and prune the data store when it goes above a certain limit.
// By using `mimalloc` we get faster allocations.
#[global_allocator]
static GLOBAL: re_memory::AccountingAllocator<mimalloc::MiMalloc> =
    re_memory::AccountingAllocator::new(mimalloc::MiMalloc);

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let main_thread_token = re_viewer::MainThreadToken::i_promise_i_am_on_the_main_thread();
    // Before any TLS (cloud login, wss:// to the backend): see install_crypto_provider.
    factoryos_client::install_crypto_provider();

    // Direct calls using the `log` crate to stderr. Control with `RUST_LOG=debug` etc.
    re_log::setup_logging();

    // URDF files opened in the viewer get the colors of their OBJ meshes' .mtl files.
    factoryos_urdf_materials::register();

    // Install handlers for panics and crashes that prints to stderr and send
    // them to Rerun analytics (if the `analytics` feature is on in `Cargo.toml`).
    re_crash_handler::install_crash_handlers(re_viewer::build_info());

    // Rerun is only the live view of the scenes (the recording is the behaviour tree on disk), so
    // no recordings come in from outside by default. `FACTORY_OS_GRPC_BIND` (e.g. `127.0.0.1:9876`)
    // opens a gRPC receiver for Rerun's logging SDKs. It has no auth: keep it on loopback.
    let mut log_receivers = Vec::new();
    let _grpc_server_handle = match std::env::var("FACTORY_OS_GRPC_BIND") {
        Ok(bind) if !bind.is_empty() => {
            let (rx, handle) = re_grpc_server::spawn_with_recv(bind.parse()?, Default::default(), re_grpc_server::shutdown::never());
            re_log::info!("Taking recordings from Rerun SDKs on {bind} (FACTORY_OS_GRPC_BIND)");
            log_receivers.push(rx);
            Some(handle)
        }
        _ => None,
    };

    let mut native_options = re_viewer::native::eframe_options(None);
    native_options.viewport = native_options
        .viewport
        .with_app_id("factory_os")
        .with_icon(std::sync::Arc::new(eframe::icon_data::from_png_bytes(include_bytes!(
            "../assets/app_icon.png"
        ))?));

    // `FACTORY_OS_AUTOMATION=stdin` (visual smoke tests, see factoryos_viewer::automation): the
    // window's size is set by its commands, and the viewer's saved state (layout, panels) is the
    // run's own, in `FACTORY_OS_AUTOMATION_STATE_DIR` if set, so earlier sessions don't change it.
    // On top, too: eframe skips frames of a window that is covered (macOS occlusion), and a
    // skipped frame is an automation command that never gets its answer.
    if factoryos_viewer::automation::enabled() {
        native_options.persist_window = false;
        native_options.viewport = native_options.viewport.with_window_level(eframe::egui::WindowLevel::AlwaysOnTop);
        if let Some(dir) = std::env::var_os("FACTORY_OS_AUTOMATION_STATE_DIR").filter(|dir| !dir.is_empty()) {
            native_options.persistence_path = Some(dir.into());
        }
    }

    // Config and the cell live in factoryos-backend: `FACTORY_OS_BACKEND` (in workspace
    // `FACTORY_OS_WORKSPACE`, if set), or chosen in a small window first (this machine's is
    // started here if none is running). Behind the edge's login (https), the viewer signs in
    // silently where the build can (the internal one), else through the browser.
    factoryos_viewer::backend_choice::init_sessions();
    let backend = match std::env::var("FACTORY_OS_BACKEND") {
        Ok(url) => {
            let workspace = std::env::var("FACTORY_OS_WORKSPACE").unwrap_or_default();
            let url = factoryos_viewer::backend_choice::with_workspace(&url, &workspace).map_err(anyhow::Error::msg)?;
            let mut logged_in = false;
            loop {
                match factoryos_viewer::Backend::connect_to(&url).await {
                    Err(err) if !logged_in && err.downcast_ref::<factoryos_viewer::LoginNeeded>().is_some() => {
                        re_log::warn!("{err} Opening the browser to sign in.");
                        factoryos_viewer::Backend::login_with_browser(&url).await?;
                        logged_in = true;
                    }
                    // A workspace being created or started: wait for it.
                    Err(err) if err.downcast_ref::<factoryos_viewer::WorkspaceNotReady>().is_some() => {
                        re_log::info!("{err}; waiting");
                        tokio::time::sleep(std::time::Duration::from_secs(3)).await;
                    }
                    result => break result?,
                }
            }
        }
        Err(_) => match factoryos_viewer::backend_choice::choose()? {
            Some(backend) => backend,
            None => return Ok(()), // chooser closed
        },
    };

    eframe::run_native(
        "Physical AI Production Designer",
        native_options,
        Box::new(move |cc| {
            factoryos_viewer::new_app(
                cc,
                factoryos_viewer::Startup {
                    backend,
                    log_receivers,
                    main_thread_token,
                },
            )
        }),
    )?;

    // Closed to switch backends (Configuration → Backend): start over at the chooser.
    if factoryos_viewer::backend_choice::restart_requested() {
        let mut viewer = std::process::Command::new(std::env::current_exe()?);
        viewer.args(std::env::args_os().skip(1)).env_remove("FACTORY_OS_BACKEND").env_remove("FACTORY_OS_WORKSPACE").env_remove("FACTORY_OS_ZENOH");
        // A workspace switch goes straight to the chosen workspace; "Switch backend…" to the chooser.
        if factoryos_viewer::backend_choice::autoconnect_requested() {
            viewer.env("FACTORY_OS_AUTOCONNECT", "1");
        } else {
            viewer.env_remove("FACTORY_OS_AUTOCONNECT");
        }
        #[cfg(unix)]
        return Err(std::os::unix::process::CommandExt::exec(&mut viewer).into());
        #[cfg(not(unix))]
        viewer.spawn()?;
    }

    Ok(())
}
